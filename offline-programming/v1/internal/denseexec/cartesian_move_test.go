//go:build linux

package denseexec

import (
	"context"
	"fmt"
	"math"
	"reflect"
	"testing"
	"time"
)

func cartesianMoveFake(t *testing.T) (*RTCoreController, *fakeMoveAdapter, CartesianMoveIntent) {
	t.Helper()
	r, f, joint := moveFake(t)
	r.cancel()
	<-r.done
	f.status.Core.TimeNS = 1
	r.cfg.CartesianModel = "rosie_1420_v1"
	for i := range f.description.Axes {
		f.description.Axes[i].ID, f.description.Axes[i].Index = fmt.Sprintf("J%d", i+1), uint32(i)
	}
	r.description = f.description
	r.SetCartesianResolver(func(_ context.Context, req CartesianResolveRequest) (CartesianResolution, error) {
		// The origin is the held command (0.25), not the measured 0.2 the fixture rests at.
		if req.Operation != "move" || req.Delta[0] != .1 || req.Pose[0] != .25 || req.Fraction != .5 {
			t.Errorf("request=%+v", req)
		}
		last := req.Pose
		last[0] += .1
		return CartesianResolution{Accepted: true, JointLimitScale: 1, SingularityScale: 1, Waypoints: [][6]float64{req.Pose, last}}, nil
	})
	return r, f, CartesianMoveIntent{TargetID: joint.TargetID, SessionID: joint.SessionID, Revision: joint.Revision, Frame: "base", Axis: 0, DistanceMM: 100, Fraction: .5}
}

func TestCartesianMoveAdmissionAndCompletion(t *testing.T) {
	r, f, in := cartesianMoveFake(t)
	state, err := r.CartesianMove(context.Background(), in)
	if err != nil || state.State != "done" || state.Revision != 1 {
		t.Fatalf("%+v %v", state, err)
	}
	if !reflect.DeepEqual(moveCalls(f), []string{"prepare_trajectory", "start_trajectory"}) {
		t.Fatal(moveCalls(f))
	}
	if f.points[0].Position[0] != .25 || math.Abs(f.points[len(f.points)-1].Position[0]-.35) > 1e-12 {
		t.Fatal("origin/endpoint changed")
	}
	_, err = r.CartesianMove(context.Background(), in)
	jointCode(t, err, "jog_session_stale")
	for _, tc := range []struct {
		name, code string
		mutate     func(*RTCoreController, *CartesianMoveIntent)
	}{
		{"target", "dense_target_unavailable", func(_ *RTCoreController, i *CartesianMoveIntent) { i.TargetID = "other" }},
		{"session", "jog_session_stale", func(_ *RTCoreController, i *CartesianMoveIntent) { i.SessionID = "other" }},
		{"grant", "no_grant", func(r *RTCoreController, _ *CartesianMoveIntent) { r.held = false }},
		{"armed", "jog_not_ready", func(r *RTCoreController, _ *CartesianMoveIntent) { r.observation.Core.Armed = 0 }},
		{"stale", "jog_not_ready", func(r *RTCoreController, _ *CartesianMoveIntent) { r.nativeSeen = time.Time{} }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			r, f, in := cartesianMoveFake(t)
			tc.mutate(r, &in)
			_, err := r.CartesianMove(context.Background(), in)
			jointCode(t, err, tc.code)
			if len(moveCalls(f)) != 0 {
				t.Fatal(moveCalls(f))
			}
		})
	}
}

func TestCartesianMoveRefusalBeforeMotion(t *testing.T) {
	for _, code := range []string{"ik_no_solution", "joint_limit", "cartesian_reach"} {
		t.Run(code, func(t *testing.T) {
			r, f, in := cartesianMoveFake(t)
			r.SetCartesianResolver(func(context.Context, CartesianResolveRequest) (CartesianResolution, error) {
				return CartesianResolution{Reason: code}, nil
			})
			state, err := r.CartesianMove(context.Background(), in)
			jointCode(t, err, code)
			if state.State != "refused" || state.Revision != 1 || len(moveCalls(f)) != 0 || !r.held || f.status.Core.Armed != 1 {
				t.Fatalf("state=%+v calls=%v", state, moveCalls(f))
			}
		})
	}
}

func TestCartesianMoveStopDuringResolveAndMotion(t *testing.T) {
	for _, tc := range []struct{ phase, action string }{{"resolve", "stop"}, {"motion", "stop"}, {"resolve", "disarm"}, {"motion", "disarm"}, {"motion", "selection_changed"}} {
		t.Run(tc.phase+"/"+tc.action, func(t *testing.T) {
			phase := tc.phase
			r, f, in := cartesianMoveFake(t)
			entered := make(chan struct{})
			if phase == "resolve" {
				r.SetCartesianResolver(func(ctx context.Context, _ CartesianResolveRequest) (CartesianResolution, error) {
					close(entered)
					<-ctx.Done()
					return CartesianResolution{}, ctx.Err()
				})
			} else {
				f.autoComplete = false
				entered = f.started
			}
			result := make(chan error, 1)
			go func() { _, err := r.CartesianMove(context.Background(), in); result <- err }()
			select {
			case <-entered:
			case <-time.After(3000 * time.Millisecond):
				t.Fatal("move did not reach requested phase")
			}
			var err error
			if tc.action == "disarm" {
				_, err = r.Arm(context.Background(), false)
			} else if tc.action == "selection_changed" {
				_, err = r.StopSelection(context.Background())
			} else {
				_, err = r.Stop(context.Background())
			}
			if err != nil {
				t.Fatal(err)
			}
			select {
			case err := <-result:
				jointCode(t, err, tc.action)
			case <-time.After(3000 * time.Millisecond):
				t.Fatal("Stop did not cancel move")
			}
			if phase == "resolve" {
				for _, call := range moveCalls(f) {
					if call == "prepare_trajectory" || call == "start_trajectory" {
						t.Fatal(moveCalls(f))
					}
				}
			}
			if state := r.JointJogState().Move; state.State != "stopped" || state.Reason != tc.action {
				t.Fatal(r.JointJogState())
			}
		})
	}
}

// Loss of the HTTP observer cannot cancel accepted preparation or motion;
// completion still publishes the reserved revision without Stop or Release.
func TestCartesianMoveClientCancellationKeepsAuthority(t *testing.T) {
	for _, phase := range []string{"resolve", "motion"} {
		t.Run(phase, func(t *testing.T) {
			r, f, in := cartesianMoveFake(t)
			ctx, disconnect := context.WithCancel(context.Background())
			defer disconnect()
			if phase == "resolve" {
				resolve := r.cartesianResolve
				r.SetCartesianResolver(func(ctx context.Context, req CartesianResolveRequest) (CartesianResolution, error) {
					disconnect()
					if ctx.Err() != nil {
						t.Error("request cancellation reached resolver")
					}
					return resolve(ctx, req)
				})
			} else {
				f.autoComplete = false
			}
			result := make(chan error, 1)
			go func() { _, err := r.CartesianMove(ctx, in); result <- err }()
			if phase == "motion" {
				select {
				case <-f.started:
				case <-time.After(3000 * time.Millisecond):
					t.Fatal("move did not start")
				}
				disconnect()
				f.mu.Lock()
				f.completeLocked()
				f.mu.Unlock()
			}
			select {
			case err := <-result:
				if err != nil {
					t.Fatal(err)
				}
			case <-time.After(3000 * time.Millisecond):
				t.Fatal("move did not complete")
			}
			state := r.JointJogState().Move
			if state.State != "done" || state.Revision != 1 || state.Reason != "" || !r.held || f.status.Core.Armed != 1 {
				t.Fatalf("state=%+v held=%v armed=%d", state, r.held, f.status.Core.Armed)
			}
			if !reflect.DeepEqual(moveCalls(f), []string{"prepare_trajectory", "start_trajectory"}) {
				t.Fatal(moveCalls(f))
			}
		})
	}
}

// Detaching the HTTP observer must not detach the completion budget: missing
// native completion retires the grant with a named failure instead of running on.
func TestCartesianMoveCompletionBudgetStopsNativeMotion(t *testing.T) {
	r, f, in := cartesianMoveFake(t)
	f.autoComplete = false
	result := make(chan error, 1)
	go func() { _, err := r.CartesianMove(context.Background(), in); result <- err }()
	select {
	case err := <-result:
		jointCode(t, err, "joint_move_completion_timeout")
	case <-time.After(5000 * time.Millisecond):
		t.Fatal("completion budget did not stop the move within 5000 ms")
	}
	if state := r.JointJogState().Move; state.State != "stopped" || state.Reason != "joint_move_completion_timeout" || r.held {
		t.Fatal(state)
	}
	if !reflect.DeepEqual(moveCalls(f), []string{"prepare_trajectory", "start_trajectory", "stop", "release", "discard_trajectory"}) {
		t.Fatal(moveCalls(f))
	}
}

// Resolver output cannot cross an adapter/daemon replacement or a changed TCP
// origin; each mutation must be observed before preparing any trajectory.
func TestCartesianMoveIdentityAndOriginMutationDuringResolve(t *testing.T) {
	for _, mutation := range []string{"adapter", "daemon", "pose"} {
		t.Run(mutation, func(t *testing.T) {
			r, f, in := cartesianMoveFake(t)
			resolve := r.cartesianResolve
			r.SetCartesianResolver(func(ctx context.Context, req CartesianResolveRequest) (CartesianResolution, error) {
				result, err := resolve(ctx, req)
				f.mu.Lock()
				switch mutation {
				case "adapter":
					f.status.AdapterIncarnation = "replacement"
				case "daemon":
					f.status.DaemonIncarnation = "replacement"
				case "pose":
					// The origin is the held command; a Home, jog or Stop moves it.
					f.status.Axes[0].LogicalTarget += .01
				}
				f.mu.Unlock()
				return result, err
			})
			state, err := r.CartesianMove(context.Background(), in)
			code := "rt_core_incarnation_changed"
			if mutation == "pose" {
				code = "cartesian_pose_stale"
			}
			jointCode(t, err, code)
			if state.State == "done" {
				t.Fatal(state)
			}
			for _, call := range moveCalls(f) {
				if call == "prepare_trajectory" || call == "start_trajectory" {
					t.Fatal(moveCalls(f))
				}
			}
		})
	}
}

func TestCartesianMoveProfileAxisSpeedAndLimits(t *testing.T) {
	r, _, _ := cartesianMoveFake(t)
	req := CartesianResolveRequest{Pose: [6]float64{.2}, Lower: [6]float64{-2, -2, -2, -2, -2, -2}, Upper: [6]float64{2, 2, 2, 2, 2, 2}, Fraction: .25}
	result := CartesianResolution{Accepted: true, JointLimitScale: 1, SingularityScale: 1, Waypoints: [][6]float64{req.Pose, {.21, .03, -.02}, {.23, .035, -.04}}}
	points, err := cartesianMoveProfile(r.description, [6]int{0, 1, 2, 3, 4, 5}, req, result, 0)
	if err != nil {
		t.Fatal(err)
	}
	for i := 1; i < len(points); i++ {
		for j := 0; j < 6; j++ {
			v := math.Abs(points[i].Position[j]-points[i-1].Position[j]) / .001
			if v > .5 {
				t.Fatalf("axis %d velocity=%g rad/s", j, v)
			}
			if i > 1 {
				acceleration := math.Abs(points[i].Position[j]-2*points[i-1].Position[j]+points[i-2].Position[j]) / .000001
				if acceleration > .5 {
					t.Fatalf("axis %d acceleration=%g rad/s^2", j, acceleration)
				}
			}
		}
	}
	result.Waypoints[1][1] = 3
	_, err = cartesianMoveProfile(r.description, [6]int{0, 1, 2, 3, 4, 5}, req, result, 0)
	jointCode(t, jointJogFailure(err), "joint_limit")
}

// A straight Cartesian step under the the 1430 cell jerk, 150 rad/s^3: the quintic
// clock's peak jerk (60 x slope / T^3) stays within it on every joint, the
// step takes longer than it does acceleration-limited, and its end is exact.
func TestCartesianMoveProfileBoundsJerk(t *testing.T) {
	d := armMoveDescription()
	req := CartesianResolveRequest{Pose: [6]float64{.2}, Lower: [6]float64{-2, -2, -2, -2, -2, -2}, Upper: [6]float64{2, 2, 2, 2, 2, 2}, Fraction: 1}
	result := CartesianResolution{Accepted: true, JointLimitScale: 1, SingularityScale: 1, Waypoints: [][6]float64{req.Pose, {.3, -.08, .05}}}
	const jerk, cycle = 150.0, .001
	// 0.1 rad on J1: acceleration-limited sqrt(6 x 0.1 / 4*pi) = 218 ms;
	// jerk-bounded cbrt(60 x 0.1 / 150) = 342 ms.
	free, err := cartesianMoveProfile(d, [6]int{0, 1, 2, 3, 4, 5}, req, result, 0)
	if err != nil {
		t.Fatal(err)
	}
	points, err := cartesianMoveProfile(d, [6]int{0, 1, 2, 3, 4, 5}, req, result, jerk)
	if err != nil {
		t.Fatal(err)
	}
	if len(free) != 220 || len(points) != 343 {
		t.Fatalf("acceleration-limited step %d samples (want 220), jerk-bounded %d (want 343)", len(free), len(points))
	}
	var peak float64
	for i := 3; i < len(points); i++ {
		for j := 0; j < 6; j++ {
			q := func(k int) float64 { return points[k].Position[j] }
			peak = math.Max(peak, math.Abs(q(i)-3*q(i-1)+3*q(i-2)-q(i-3))/(cycle*cycle*cycle))
		}
	}
	// Absolute allowance: 1 rad/s^3 for the third difference of a sampled quintic.
	if peak > jerk+1 {
		t.Fatalf("peak jerk %g rad/s^3 above %g", peak, jerk)
	}
	if last := points[len(points)-1].Position; last[0] != .3 || last[1] != -.08 || last[2] != .05 {
		t.Fatalf("end %v", last)
	}
	if _, err := cartesianMoveProfile(d, [6]int{0, 1, 2, 3, 4, 5}, req, result, math.NaN()); err == nil {
		t.Fatal("NaN jerk admitted")
	}
}

// A Cartesian step's fraction runs to 100 % like a joint move's (owner
// decision 2026-09-21); above 1 it is refused as input.
func TestCartesianMoveFractionRunsToOne(t *testing.T) {
	r, _, in := cartesianMoveFake(t)
	// The fixture's resolver pins the fraction at 0.5; this one only checks that 1 arrives.
	r.SetCartesianResolver(func(_ context.Context, req CartesianResolveRequest) (CartesianResolution, error) {
		if req.Fraction != 1 {
			t.Errorf("fraction %v did not reach the resolver", req.Fraction)
		}
		last := req.Pose
		last[0] += .1
		return CartesianResolution{Accepted: true, JointLimitScale: 1, SingularityScale: 1, Waypoints: [][6]float64{req.Pose, last}}, nil
	})
	in.Fraction = 1
	if state, err := r.CartesianMove(context.Background(), in); err != nil || state.State != "done" {
		t.Fatalf("%+v %v", state, err)
	}
	r, _, in = cartesianMoveFake(t)
	in.Fraction = 1.01
	state, err := r.CartesianMove(context.Background(), in)
	jointCode(t, err, "cartesian_input_invalid")
	if state.State != "refused" {
		t.Fatalf("%+v", state)
	}
}
