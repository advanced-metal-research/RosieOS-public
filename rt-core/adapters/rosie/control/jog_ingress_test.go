//go:build linux

package control

import (
	"encoding/hex"
	"encoding/json"
	"math"
	"net"
	"os"
	"path/filepath"
	"reflect"
	"testing"
	"time"

	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
)

// All ingress transports must expose identical cumulative refusal accounting
// through status and diagnostics; accepted input cannot erase the last refusal.
func TestJogIngressRefusalAccounting(t *testing.T) {
	for _, path := range []string{"datagram", "wss", "UpdateJog"} {
		t.Run(path, func(t *testing.T) {
			f := startRemoteMotion(t)
			generation := f.begin(t)
			var ws *remoteWSClient
			var datagram *net.UnixConn
			if path == "wss" {
				ws = f.dial(t, generation)
				ws.calibrate(t)
			} else if path == "datagram" {
				directory, err := os.MkdirTemp("", "ingress-")
				if err != nil {
					t.Fatal(err)
				}
				defer os.RemoveAll(directory)
				socket := filepath.Join(directory, "j.sock")
				listener, err := ListenJog(socket)
				if err != nil {
					t.Fatal(err)
				}
				done := make(chan struct{})
				go func() { defer close(done); _ = f.controller.ServeJog(listener) }()
				defer func() { listener.Close(); <-done }()
				datagram, err = net.DialUnix("unixgram", nil, &net.UnixAddr{Name: socket, Net: "unixgram"})
				if err != nil {
					t.Fatal(err)
				}
				defer datagram.Close()
			}
			frame := func(sequence uint64) ipcclient.LocalJogUpdate {
				if ws != nil {
					return ws.frame(f, generation, sequence)
				}
				now := ipcclient.HostMonotonicNS()
				value := ipcclient.LocalJogUpdate{GrantGeneration: f.fence.Generation, JogGeneration: generation,
					SourceSequence: sequence, AxisMask: 511, OriginHostNs: now, DeadlineHostNs: now + 200_000_000}
				token, _ := hex.DecodeString(f.fence.Session)
				clock, _ := hex.DecodeString(f.clock.Incarnation)
				copy(value.SessionId[:], token)
				copy(value.ClockIncarnation[:], clock)
				return value
			}
			send := func(value ipcclient.LocalJogUpdate, refused bool) {
				if ws != nil {
					if refused {
						ws.send(t, 2, []byte{1})
						ws.rejection(t, 0, "jog_invalid_frame")
					} else {
						ws.update(t, value)
					}
				} else if datagram != nil {
					packet := ipcclient.EncodeLocalJogUpdate(value)
					body := packet[:]
					if refused {
						body = []byte{1}
					}
					if _, err := datagram.Write(body); err != nil {
						t.Fatal(err)
					}
				} else {
					if refused {
						value.Velocity[0] = math.NaN()
					}
					err := f.controller.UpdateJog(f.fence, generation, value.SourceSequence, value.OriginHostNs, value.DeadlineHostNs, 511, value.Velocity[:9])
					if (err != nil) != refused {
						t.Fatalf("UpdateJog refusal=%v: %v", refused, err)
					}
				}
			}
			// Decode the wire contract independently so the regression also runs
			// against the pre-counter implementation and detects missing fields.
			type refusal struct {
				SourceSequence uint64 `json:"source_sequence"`
				Reason         uint32 `json:"reason"`
				NowHostNS      uint64 `json:"now_host_ns"`
			}
			type observation struct {
				SourceSequence  uint64     `json:"source_sequence"`
				Refused         uint64     `json:"refused"`
				RefusedByReason [15]uint64 `json:"refused_by_reason"`
				LatestRefusal   refusal    `json:"latest_refusal"`
			}
			observe := func(status bool) observation {
				url := f.url + "/v1/jog/ingress"
				if status {
					url = f.url + "/v1/status"
				}
				response, err := f.http.Get(url)
				if err != nil {
					t.Fatal(err)
				}
				defer response.Body.Close()
				var envelope publicResetResponse
				if err := json.NewDecoder(response.Body).Decode(&envelope); err != nil || response.StatusCode != 200 {
					t.Fatalf("observation: %+v %v", envelope, err)
				}
				var out observation
				if status {
					var value struct {
						JogIngress observation `json:"jog_ingress"`
					}
					if err := json.Unmarshal(envelope.Data, &value); err != nil {
						t.Fatal(err)
					}
					out = value.JogIngress
				} else if err := json.Unmarshal(envelope.Data, &out); err != nil {
					t.Fatal(err)
				}
				return out
			}
			send(frame(1), false)
			waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence == 1 })
			if got := observe(false); got.Refused != 0 || got.LatestRefusal != (refusal{}) {
				t.Fatalf("accepted frame counted: %+v", got)
			}
			before := ipcclient.HostMonotonicNS()
			send(frame(2), true)
			// Unverified observation deadline: 2000 ms; success requires one counted refusal.
			var rejected observation
			if !simfixture.PollUntil(time.Now().Add(2*time.Second), time.Millisecond, func() bool {
				rejected = observe(false)
				return rejected.Refused != 0
			}) {
				t.Fatalf("refusal was not counted: %+v", rejected)
			}
			wantSequence := uint64(0)
			if path == "UpdateJog" {
				wantSequence = 2
			}
			if rejected.Refused != 1 || rejected.RefusedByReason[7] != 1 || rejected.LatestRefusal.Reason != 7 ||
				rejected.LatestRefusal.SourceSequence != wantSequence || rejected.LatestRefusal.NowHostNS < before {
				t.Fatalf("refusal accounting: %+v", rejected)
			}
			send(frame(3), false)
			waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence == 3 })
			after := observe(false)
			if after.Refused != 1 || after.RefusedByReason != rejected.RefusedByReason || after.LatestRefusal != rejected.LatestRefusal {
				t.Fatalf("acceptance changed refusal history: before=%+v after=%+v", rejected, after)
			}
			if status := observe(true); !reflect.DeepEqual(status, after) {
				t.Fatalf("status/diagnostics disagree: %+v / %+v", status, after)
			}
		})
	}
}

// Clock/session failures before publication and shared native refusals must
// each count once; acceptance cannot reset their cumulative diagnostic evidence.
func TestRemoteJogIngressPrePublicationAndSharedRefusals(t *testing.T) {
	f := startRemoteMotion(t)
	generation := f.begin(t)
	w := f.dial(t, generation)
	assertRefusal := func(total, sequence uint64, reason uint32) {
		t.Helper()
		o := f.controller.JogIngress()
		var sum uint64
		for _, count := range o.RefusedByReason {
			sum += count
		}
		if o.Refused != total || sum != total || o.LatestRefusal.SourceSequence != sequence || o.LatestRefusal.Reason != reason || o.LatestRefusal.NowHostNS == 0 {
			t.Fatalf("refusal total=%d sequence=%d reason=%d: %+v", total, sequence, reason, o)
		}
	}
	w.update(t, w.frame(f, generation, 1))
	w.rejection(t, 1, "jog_clock_unqualified")
	assertRefusal(1, 1, ipcclient.JogReasonNotReady)
	wrongSession := w.frame(f, generation, 2)
	wrongSession.SessionId[0] ^= 1
	w.update(t, wrongSession)
	w.rejection(t, 2, "jog_session_stale")
	assertRefusal(2, 2, ipcclient.JogReasonWrongIdentity)
	w.calibrate(t)
	w.update(t, w.frame(f, generation, 3))
	waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence == 3 })
	assertRefusal(2, 2, ipcclient.JogReasonWrongIdentity)
	w.update(t, w.frame(f, generation, 3))
	w.rejection(t, 3, "jog_native_rejected_6")
	assertRefusal(3, 3, ipcclient.JogReasonStaleSequence)
	w.update(t, w.frame(f, generation, 4))
	waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence == 4 })
	assertRefusal(3, 3, ipcclient.JogReasonStaleSequence)
}
