//go:build linux

package denseexec

import (
	"context"
	"rosieos/rt-core/sdk/control"
	"testing"
)

type slowMoveAdapter struct {
	*fakeMoveAdapter
	t                *testing.T
	delays, attempts int
	request          *control.Request
}

func (f *slowMoveAdapter) Command(ctx context.Context, request *control.Request) (control.Response, error) {
	if request.Operation == "prepare_trajectory" {
		f.attempts++
		if f.request != nil && f.request != request {
			f.t.Fatal("retry replaced request identity")
		}
		f.request = request
		if f.attempts <= f.delays {
			return control.Response{}, context.DeadlineExceeded
		}
	}
	return f.fakeMoveAdapter.Command(ctx, request)
}

// TestReviewJointMovePrepareTimeoutKeepsAuthority: a prepare_trajectory upload
// is sent once (a retry meets the adapter's busy refusal while it still decodes
// the first body) and its deadline is a typed refusal that leaves nothing in
// motion, so the grant, the armed state and the fence all survive for a retry.
func TestReviewJointMovePrepareTimeoutKeepsAuthority(t *testing.T) {
	r, f, in := moveFake(t)
	r.cancel()
	<-r.done
	slow := &slowMoveAdapter{fakeMoveAdapter: f, t: t, delays: 1}
	r.c = slow
	_, err := r.JointMove(context.Background(), in)
	if slow.attempts != 1 {
		t.Fatalf("attempts=%d want 1 (a prepare is never resent)", slow.attempts)
	}
	jointCode(t, err, "joint_move_prepare_timeout")
	snapshot := r.statusSnapshot()
	if !snapshot.Session.Lease.Held {
		t.Fatal("prepare deadline retired authority")
	}
	if snapshot.RTCore.Note != "rt_core_request_slow" {
		t.Fatalf("note=%q", snapshot.RTCore.Note)
	}
	if state := r.JointJogState().Move; state.State != "refused" || state.Reason != "joint_move_prepare_timeout" {
		t.Fatalf("move state %+v", state)
	}
}

type busyPrepareAdapter struct {
	*fakeMoveAdapter
	attempts int
}

func (f *busyPrepareAdapter) Command(ctx context.Context, request *control.Request) (control.Response, error) {
	if request.Operation == "prepare_trajectory" {
		f.attempts++
		return control.Response{}, &control.Rejected{Reason: "busy"}
	}
	return f.fakeMoveAdapter.Command(ctx, request)
}

// TestReviewJointMovePrepareBusyKeepsAuthority: the adapter's busy refusal of a
// prepare (another upload still decoding) is a refusal of this step, not a
// reason to Stop: the machine stays armed and the operator presses again.
func TestReviewJointMovePrepareBusyKeepsAuthority(t *testing.T) {
	r, f, in := moveFake(t)
	r.cancel()
	<-r.done
	busy := &busyPrepareAdapter{fakeMoveAdapter: f}
	r.c = busy
	_, err := r.JointMove(context.Background(), in)
	if busy.attempts != 1 {
		t.Fatalf("attempts=%d want 1", busy.attempts)
	}
	jointCode(t, err, "joint_move_prepare_busy")
	if !r.statusSnapshot().Session.Lease.Held {
		t.Fatal("busy prepare retired authority")
	}
}
