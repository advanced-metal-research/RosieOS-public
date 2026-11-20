//go:build linux

package denseexec

import (
	"context"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"os"
	"strings"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/densejointtraj"
	"rosieos/rt-core/sdk/control"
)

func TestRTCoreLoadRejectsStalePlanningCaptureBeforeAuthority(t *testing.T) {
	for _, change := range []string{"current", "home", "configuration_epoch", "configuration_sha", "adapter", "daemon", "missing_epoch", "schema", "joint_order", "first_sample", "missing_axis"} {
		t.Run(change, func(t *testing.T) {
			r, f, id := rtFake(t)
			native := &dryRunRTCore{fakeRTCore: f}
			r.c = native
			f.status.Core.ConfigurationSHA256 = strings.Repeat("d", 64)
			f.status.Core.ConfigurationEpoch = 2
			f.status.Core.HomeEpoch = 3
			blob, plan := fixtureBlob(t)
			start := densejointtraj.MeasuredStart{Schema: "rosie.measured-planning-start.v1", CoordinateIdentity: fmt.Sprintf("%s:2:3", f.status.Core.ConfigurationSHA256), ObservationIdentity: "adapter:daemon"}
			for i, a := range f.description.Axes {
				start.JointNames = append(start.JointNames, a.ID)
				start.JointValues = append(start.JointValues, plan.Segments[0].Samples[0].QRad[i])
			}
			switch change {
			case "home":
				f.status.Core.HomeEpoch++
			case "configuration_epoch":
				f.status.Core.ConfigurationEpoch++
			case "configuration_sha":
				f.status.Core.ConfigurationSHA256 = strings.Repeat("e", 64)
			case "adapter":
				f.status.AdapterIncarnation = "restarted-adapter"
			case "daemon":
				f.status.DaemonIncarnation = "restarted-daemon"
			case "missing_epoch":
				f.status.Core.HomeEpoch = 0
			case "schema":
				start.Schema = "unknown"
			case "joint_order":
				start.JointNames[0], start.JointNames[5] = start.JointNames[5], start.JointNames[0]
			case "first_sample":
				start.JointValues[5] += 0.001
			case "missing_axis":
				start.JointValues = start.JointValues[:8]
			}
			n := int(binary.BigEndian.Uint32(blob[:4]))
			var header map[string]json.RawMessage
			if err := json.Unmarshal(blob[4:4+n], &header); err != nil {
				t.Fatal(err)
			}
			header["start_state"], _ = json.Marshal(start)
			h, _ := json.Marshal(header)
			altered := make([]byte, 4)
			binary.BigEndian.PutUint32(altered, uint32(len(h)))
			altered = append(altered, h...)
			altered = append(altered, blob[4+n:]...)
			r.source.cfg.FetchBlob = func(context.Context, string) ([]byte, error) { return altered, nil }
			loaded, err := r.Load(context.Background(), LoadRequest{Identity: id, DryRun: true})
			if change == "current" {
				if err != nil || loaded == nil {
					t.Fatalf("current capture rejected: %v", err)
				}
				if len(native.blobs) != 1 {
					t.Fatal("current capture was not prepared")
				}
				uploaded, e := densejointtraj.Decode(native.blobs[0])
				if e != nil || uploaded.Header.StartState == nil || uploaded.Header.StartState.CoordinateIdentity != start.CoordinateIdentity {
					t.Fatalf("dry run lost capture: %v", e)
				}
			} else {
				if err == nil || !strings.Contains(err.Error(), "planning_start_stale") {
					t.Fatalf("stale capture accepted: %v", err)
				}
				f.mu.Lock()
				defer f.mu.Unlock()
				if len(f.calls) != 0 || len(native.blobs) != 0 {
					t.Fatalf("stale capture acquired authority/uploaded: %v", f.calls)
				}
			}
		})
	}
}

// The optional export is private test evidence for the main-package capture
// regression. All coordinate/status fields come from actual isolated native
// simulator processes through the public SDK, never a manufactured axis map.
func TestRTCoreComposedPlanningCapturePublicStatus(t *testing.T) {
	c, d, _, _, cfg := rtComposedTransportFixture(t, false, func(machine map[string]any) {
		raw, err := os.ReadFile("../../../../rt-core/config/machines/bench/nine-motors-1to1.json")
		if err != nil {
			t.Fatal(err)
		}
		var fixture map[string]any
		if err := json.Unmarshal(raw, &fixture); err != nil {
			t.Fatal(err)
		}
		for key := range machine {
			delete(machine, key)
		}
		for key, value := range fixture {
			machine[key] = value
		}
		machine["max_cycle_lateness_ns"] = 20000000

	})
	r, err := NewRTCore(cfg, Config{ControllerID: "offline-programming:capture-contract"}, c)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	heartbeatCtx, stopHeartbeat := context.WithCancel(ctx)
	done := make(chan struct{})
	go func() {
		defer close(done)
		ticker := time.NewTicker(100 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-heartbeatCtx.Done():
				return
			case <-ticker.C:
				if session := r.statusSnapshot().Session; session != nil {
					_, _ = r.Heartbeat(session.ID)
				}
			}
		}
	}()
	t.Cleanup(func() { stopHeartbeat(); <-done })
	if err := r.Home(ctx); err != nil {
		t.Fatalf("isolated simulator Home: %v", err)
	}
	rtAwait(t, func() bool {
		s, err := c.Status(ctx)
		return err == nil && s.Core.HomeValidMask == 511 && s.Core.CommissioningPhase == 0
	})
	var observations []*StatusBody
	var previous control.Status
	captureDeadline := time.Now().Add(500 * time.Millisecond)
	for len(observations) < 5 && time.Now().Before(captureDeadline) {
		s := r.Status(context.Background())
		raw, _ := json.Marshal(s)
		if strings.Contains(string(raw), "logical_velocity") {
			t.Fatal("native contract unexpectedly emits logical velocity")
		}
		native := s.RTCore.Status
		advanced := native.Motion.TimeNS > previous.Motion.TimeNS
		for i, a := range native.Axes {
			if i < len(previous.Axes) && a.TimeNS <= previous.Axes[i].TimeNS {
				advanced = false
			}
		}
		if advanced {
			observations = append(observations, s)
			previous = *native
		}
		time.Sleep(25 * time.Millisecond)
	}
	if len(observations) < 5 {
		t.Fatal("native public status did not provide five advancing observations within 500ms")
	}

	raw, err := json.Marshal(struct {
		Description  any           `json:"description"`
		Observations []*StatusBody `json:"observations"`
	}{d, observations})
	if err != nil {
		t.Fatal(err)
	}
	if path := os.Getenv("RTCORE_CAPTURE_PUBLIC_STATUS"); path != "" {
		if err := os.WriteFile(path, raw, 0600); err != nil {
			t.Fatal(err)
		}
	}
	t.Logf("actual public simulator status: axes=%d, logical_velocity absent across %d observations", len(d.Axes), len(observations))
}
