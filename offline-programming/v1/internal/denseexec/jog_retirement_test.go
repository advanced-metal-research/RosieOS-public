//go:build linux

package denseexec

import (
	"context"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

type staleEndLane struct{ *fakeJointLane }

func (s *staleEndLane) End(ctx context.Context) error {
	_ = s.fakeJointLane.End(ctx)
	return &control.Rejected{Reason: "jog_session_stale"}
}

// A stale End is cleanup only with current native evidence. Old generations,
// open input and changed authority must still fail closed.
func TestJogStaleEndRequiresNativeClosure(t *testing.T) {
	for _, name := range []string{"closed", "open", "generation", "grant", "incarnation", "missing"} {
		t.Run(name, func(t *testing.T) {
			r, f, lane, _ := jointFake(t)
			r.cancel()
			<-r.done
			f.mu.Lock()
			f.status.Jog = &control.JogObservation{JogGeneration: 17, AppGrantGeneration: r.fence.Generation}
			f.status.Grant = &ipcclient.GrantObservation{GrantActive: 1, AppGrantGeneration: r.fence.Generation}
			switch name {
			case "open":
				f.status.Jog.Open = 1
			case "generation":
				f.status.Jog.JogGeneration = 16
			case "grant":
				f.status.Grant.AppGrantGeneration++
			case "incarnation":
				f.status.AdapterIncarnation = "replacement"
			case "missing":
				f.status.Jog = nil
			}
			f.mu.Unlock()
			err := r.endJointJog(&staleEndLane{lane})
			if (err == nil) != (name == "closed") {
				t.Fatalf("cleanup=%v", err)
			}
		})
	}
}

// stalePathLane is a WSS path lane whose stream was refused: the adapter's
// handler already ended the session, so End is refused as stale.
type stalePathLane struct {
	*fakePathLane
	refused chan error
}

func (s *stalePathLane) End(ctx context.Context) error {
	_ = s.fakePathLane.End(ctx)
	return &control.Rejected{Reason: "jog_session_stale"}
}

func (s *stalePathLane) Rejection(ctx context.Context) error {
	select {
	case err := <-s.refused:
		return err
	case <-ctx.Done():
		return ctx.Err()
	}
}

// closedPathLane answers End as the SDK does when native already stops the
// lane along the path: end_jog refused Closed through client.Command.
type closedPathLane struct{ *fakePathLane }

func (s closedPathLane) End(ctx context.Context) error {
	_ = s.fakePathLane.End(ctx)
	return &control.Rejected{Reason: "RTCore rejected jog (reason 1)",
		NativeJogResult: &ipcclient.JogObservation{JogGeneration: s.Generation(), ControlReason: ipcclient.JogReasonClosed}}
}

// the 1430 cell 2026-10-07 15:33: the operator's input stalled past the 250 ms
// lifetime, native began the path stop on the line, OLP's End was refused
// Closed (native refuses control while stopping) and the Stop fence disarmed
// the cell at ~390 mm/s. That receipt is cleanup when native shows this
// generation fenced under the current grant; anything else still Stops.
func TestPathEndRefusedClosedWhileStoppingNeedsNativeProof(t *testing.T) {
	for _, name := range []string{"stopping", "open", "grant", "incarnation"} {
		t.Run(name, func(t *testing.T) {
			r, f, lane, _, in := pathFake(t)
			r.pathOpen = func(context.Context, uint32, float64, float64, time.Duration) (pathJogLane, error) {
				return closedPathLane{lane}, nil
			}
			if _, err := r.CartesianJog(context.Background(), "start", in); err != nil {
				t.Fatal(err)
			}
			f.mu.Lock()
			f.status.Jog = &control.JogObservation{JogGeneration: lane.Generation(), AppGrantGeneration: r.fence.Generation, StateReason: ipcclient.JogReasonRamping}
			f.status.Grant = &ipcclient.GrantObservation{GrantActive: 1, AppGrantGeneration: r.fence.Generation}
			switch name {
			case "open":
				f.status.Jog.Open = 1
			case "grant":
				f.status.Grant.AppGrantGeneration++
			case "incarnation":
				f.status.AdapterIncarnation = "replacement"
			}
			f.mu.Unlock()
			// The deadman: native expired the input and stops along the path.
			lane.mu.Lock()
			lane.obs.Stopping, lane.obs.StopReason = true, 2
			lane.mu.Unlock()
			_, err := r.CartesianJog(context.Background(), "halt", in)
			stopped := false
			for _, call := range jointCalls(f) {
				stopped = stopped || call == "stop"
			}
			if name == "stopping" {
				if err != nil || stopped {
					t.Fatalf("native-proven closed End: err=%v stop=%v", err, stopped)
				}
				f.mu.Lock()
				armed := f.status.Core.Armed
				f.mu.Unlock()
				if armed != 1 {
					t.Fatal("disarmed")
				}
			} else if err == nil || !stopped {
				t.Fatalf("unproven closed End: err=%v stop=%v", err, stopped)
			}
		})
	}
}

// the 1430 cell 2026-10-07: a refused path frame closed the WSS stream, the adapter
// ended the session itself, OLP's End was refused as stale and the Stop fence
// disarmed the cell. A stale path End is cleanup only when native shows this
// generation closed under the current grant (it stays open while stopping
// along the path); anything else still fails closed through Stop.
func TestPathStaleEndRequiresNativeClosure(t *testing.T) {
	for _, name := range []string{"closed", "stopping", "open", "grant", "incarnation"} {
		t.Run(name, func(t *testing.T) {
			r, f, lane, _, in := pathFake(t)
			stale := &stalePathLane{fakePathLane: lane, refused: make(chan error, 1)}
			r.pathOpen = func(context.Context, uint32, float64, float64, time.Duration) (pathJogLane, error) { return stale, nil }
			r.cfg.Address = "rt-core.test:443"
			if _, err := r.CartesianJog(context.Background(), "start", in); err != nil {
				t.Fatal(err)
			}
			f.mu.Lock()
			f.status.Jog = &control.JogObservation{JogGeneration: lane.Generation(), AppGrantGeneration: r.fence.Generation}
			f.status.Grant = &ipcclient.GrantObservation{GrantActive: 1, AppGrantGeneration: r.fence.Generation}
			switch name {
			case "stopping":
				f.status.Jog.Open = 1
				time.AfterFunc(150*time.Millisecond, func() {
					f.mu.Lock()
					// A new observation: returned statuses share the old pointer.
					closed := *f.status.Jog
					closed.Open = 0
					f.status.Jog = &closed
					f.mu.Unlock()
				})
			case "open":
				f.status.Jog.Open = 1
			case "grant":
				f.status.Grant.AppGrantGeneration++
			case "incarnation":
				f.status.AdapterIncarnation = "replacement"
			}
			f.mu.Unlock()
			stale.refused <- &control.PathJogRejection{Reason: "jog_clock_moved_backwards"}
			waitFor(t, "hold retirement", func() bool { return !r.JointJogState().Active })
			clean := name == "closed" || name == "stopping"
			stopped := func() bool {
				f.mu.Lock()
				defer f.mu.Unlock()
				for _, call := range f.calls {
					if call == "stop" {
						return true
					}
				}
				return false
			}
			if clean {
				time.Sleep(300 * time.Millisecond)
				if stopped() {
					t.Fatal("native-proven cleanup still issued Stop")
				}
				if s := r.JointJogState(); s.Reason != "jog_clock_moved_backwards" {
					t.Fatalf("retirement reason %q", s.Reason)
				}
			} else {
				deadline := time.Now().Add(r.callBudget() + 2*time.Second)
				for !stopped() {
					if time.Now().After(deadline) {
						t.Fatal("uncertain path cleanup did not Stop")
					}
					time.Sleep(10 * time.Millisecond)
				}
			}
		})
	}
}
