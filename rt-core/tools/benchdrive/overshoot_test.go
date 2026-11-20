//go:build linux

package main

import (
	"bytes"
	"context"
	"encoding/json"
	"io"
	"math"
	"net/http"
	"net/http/httputil"
	"path/filepath"
	"strings"
	"sync"
	"testing"

	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// The same absolute feedback samples exercise catch-up, equality and breach;
// summaries must retain the offending peak even when execution fails.
func TestObservedVelocityAxisLimitAndSummary(t *testing.T) {
	for _, phase := range []string{"move", "program"} {
		for _, tc := range []struct {
			observed float64
			breach   bool
			fraction float64
		}{
			{0.075, false, 1}, {0.1, false, 5.0 / 3}, {0.125, true, 7.0 / 3},
		} {
			var sink, out bytes.Buffer
			r := &runner{o: options{phase: phase}, e: &evidence{sink: &sink}, out: &out,
				description: control.Description{NativeDescription: ipcclient.Description{Axes: []ipcclient.AxisDescription{{MaxVelocity: 0.1}}}}}
			m := r.beginVelocity(phase, 0, .0375)
			err := r.observeVelocity(m, tc.observed)
			if (err != nil) != tc.breach {
				t.Fatalf("velocity %g: %v", tc.observed, err)
			}
			if m.ObservedPeak != tc.observed || m.OvershootFraction == nil || math.Abs(*m.OvershootFraction-tc.fraction) > 1e-12 {
				t.Fatal(m)
			}
			r.summary(err)
			if !strings.Contains(out.String(), "axis=0 peak_observed_velocity_rad_s=") || !strings.Contains(sink.String(), `"velocity_measurements"`) || !strings.Contains(sink.String(), `"overshoot_fraction"`) {
				t.Fatal("missing peak summary")
			}
		}
	}
	var sink bytes.Buffer
	r := &runner{e: &evidence{sink: &sink}, description: control.Description{NativeDescription: ipcclient.Description{Axes: []ipcclient.AxisDescription{{MaxVelocity: .1}}}}}
	m := r.beginVelocity("program", 0, 0)
	if err := r.observeVelocity(m, .01); err != nil || m.OvershootFraction != nil || !strings.Contains(sink.String(), `"overshoot_fraction":null`) {
		t.Fatalf("stationary fraction: %v", err)
	}
	if err := r.observeVelocity(m, math.Inf(1)); err == nil {
		t.Fatal("accepted nonfinite feedback")
	}
}

// A real simulator/control run receives one synthetic feedback catch-up through
// its status transport. Fixed 0.075/0.125 rad/s probes straddle the 0.1 rad/s
// axis limit, independent of the planned trajectory or observed scheduler rate.
func TestSimulationVelocityOvershootAndBreach(t *testing.T) {
	for _, phase := range []string{"move", "program"} {
		for _, tc := range []struct {
			name     string
			velocity float64
			code     int
		}{
			{"catchup", .075, 0}, {"breach", .125, 1},
		} {
			t.Run(phase+"/"+tc.name, func(t *testing.T) {
				f := startFixture(t)
				f.phase(t, "home")
				client := control.NewUnixHTTPClient(f.socket)
				t.Cleanup(client.CloseIdleConnections)
				var mu sync.Mutex
				var previous ipcclient.AxisStatus
				injected := false
				proxy := &httputil.ReverseProxy{Director: func(req *http.Request) {
					req.URL.Scheme, req.URL.Host = "http", "localhost"
				}, Transport: client.Transport, ModifyResponse: func(response *http.Response) error {
					if response.Request.URL.Path != "/v1/status" {
						return nil
					}
					mu.Lock()
					defer mu.Unlock()
					var envelope control.Response
					err := json.NewDecoder(response.Body).Decode(&envelope)
					response.Body.Close()
					if err != nil {
						return err
					}
					var s control.Status
					if err := json.Unmarshal(envelope.Data, &s); err != nil {
						return err
					}
					// Repeated publications must retain the injected position until
					// a new axis timestamp arrives; otherwise this fabricates torn feedback.
					if injected && s.Core.Axes[0].TimeNS == previous.TimeNS {
						s.Core.Axes[0].LogicalPosition = previous.LogicalPosition
					}
					if s.Motion.TrajectoryID != 0 && s.Motion.CommandSequence != 0 && !s.Motion.Done && s.Core.Armed != 0 {
						a := &s.Core.Axes[0]
						if !injected && previous.TimeNS != 0 && a.TimeNS > previous.TimeNS {
							a.LogicalPosition = previous.LogicalPosition + tc.velocity*float64(a.TimeNS-previous.TimeNS)/1e9
							injected = true
						}
						previous = *a
					}
					envelope.Data, err = json.Marshal(s)
					if err != nil {
						return err
					}
					raw, err := json.Marshal(envelope)
					if err != nil {
						return err
					}
					response.Body = io.NopCloser(bytes.NewReader(raw))
					response.ContentLength = int64(len(raw))
					response.Header.Del("Content-Length")
					return nil
				}}
				proxySocket := unixServer(t, proxy.ServeHTTP)
				args := append(f.args(phase, "velocity", true), "--socket", proxySocket, "--delta-rad", "0.02", "--velocity-fraction", "0.5", "--accel-fraction", "0.5")
				var out bytes.Buffer
				if code := run(context.Background(), args, &out); code != tc.code {
					t.Fatalf("exit=%d want=%d: %s", code, tc.code, &out)
				}
				mu.Lock()
				didInject := injected
				mu.Unlock()
				if !didInject {
					t.Fatal("no active feedback pair injected")
				}
				records := readRecords(t, filepath.Join(f.build, "velocity.jsonl"))
				var summary struct {
					Measurements []velocityMeasurement `json:"velocity_measurements"`
				}
				last := records[len(records)-1]
				simfixture.Must(t, json.Unmarshal(last.Response, &summary))
				if len(summary.Measurements) == 0 {
					t.Fatal("no velocity measurements")
				}
				m := summary.Measurements[0]
				if m.Axis != 0 || m.Phase != phase || math.Abs(m.ObservedPeak-tc.velocity) > 1e-9 || m.Limit != .1 || m.OvershootFraction == nil || *m.OvershootFraction <= 0 {
					t.Fatalf("peak=%+v", m)
				}
				if tc.code != 0 && !strings.Contains(strings.Join(last.NotMet, " "), "exceeds axis max_velocity 0.1") {
					t.Fatal(last.NotMet)
				}
				f.poll(t, func(s control.Status) bool {
					return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Core.LeaseValid == 0
				})
			})
		}
	}
}
