//go:build linux

package oracle

import (
	"math"
	"syscall"
	"testing"
	"time"

	ipc "rosieos/rt-core/ipcclient"
)

// Origin: test_runtime.RuntimeTests.test_prepare_is_inert_and_start_is_correlated.
func TestPrepareIsInertAndStartIsCorrelated(t *testing.T) {
	f := start(t, "simulation.json")
	f.arm(3)
	h := f.prepare(600, points(1, .02, 0, nil))
	f.stable(250*time.Millisecond, func(s ipc.Status) bool { return s.Axes[0].PositionCounts == 0 })
	seq, err := f.c.StartTrajectory(ctx, h)
	must(t, err)
	s := f.wait(func(s ipc.Status) bool { return s.Axes[0].PositionCounts > 100 })
	m := f.motion()
	if m.CommandSequence != seq || m.TrajectoryID != h || s.SafetyFaultMask != 0 {
		t.Fatalf("correlation: %+v %+v", m, s)
	}
}

// Origin: test_runtime.RuntimeTests.test_full_dense_plan_capacity_without_cyclic_copy.
func TestFullDensePlanCapacityWithoutCyclicCopy(t *testing.T) {
	f := start(t, "simulation.json")
	p := make([]ipc.Point, 250000)
	for i := range p {
		p[i] = ipc.Point{Time: time.Duration(i) * 10000, Position: []float64{0, 0}}
	}
	h := f.prepare(601, p)
	if f.status().Armed != 0 {
		t.Fatal("prepare armed")
	}
	f.arm(3)
	seq, err := f.c.StartTrajectory(ctx, h)
	must(t, err)
	poll(t, 4*time.Second, func() bool { m := f.motion(); return m.CommandSequence == seq && m.Done })
	if f.status().SafetyFaultMask != 0 {
		t.Fatal("dense plan fault")
	}
}

// Origin: test_runtime.RuntimeTests.test_mode_conflicts_preserve_active_owner.
func TestModeConflictsPreserveActiveOwner(t *testing.T) {
	f := start(t, "simulation.json")
	f.arm(3)
	f.trajectory(602, points(5, .1, 0, nil))
	seq, err := f.c.Jog(ctx, 1, []float64{.01, 0}, 250*time.Millisecond)
	must(t, err)
	_, err = f.c.WaitResult(ctx, seq)
	rejected(t, err, 3)
	if f.motion().TrajectoryID != 602 {
		t.Fatal("owner changed")
	}
	f.stop()
	f.arm(3)
	seq, err = f.c.Jog(ctx, 1, []float64{0, 0}, 250*time.Millisecond)
	must(t, err)
	_, err = f.c.WaitResult(ctx, seq)
	must(t, err)
	h := f.prepare(603, points(1, 0, 0, nil))
	_, err = f.c.StartTrajectory(ctx, h)
	rejected(t, err, 3)
	if f.status().Armed != 1 {
		t.Fatal("conflict disarmed owner")
	}
}

// Origin: test_runtime.RuntimeTests.test_prepared_handle_invalidated_by_stop_and_storage_is_bounded.
func TestPreparedHandleInvalidatedByStopAndStorageIsBounded(t *testing.T) {
	f := start(t, "simulation.json")
	p := points(1, 0, 0, nil)
	for _, h := range []uint64{610, 611, 612} {
		f.prepare(h, p)
	}
	_, err := f.c.PrepareTrajectory(ctx, 613, 3, p)
	rejected(t, err, 5)
	f.stop()
	f.arm(3)
	_, err = f.c.StartTrajectory(ctx, 610)
	rejected(t, err, 0)
	f.prepare(614, p)
}

// Origin: test_runtime.RuntimeTests.test_continuous_nonzero_start_is_accepted.
func TestContinuousNonzeroStartIsAccepted(t *testing.T) {
	f := start(t, "simulation.json")
	f.arm(3)
	f.trajectory(505, points(1, .02, 0, nil))
	s := f.wait(func(s ipc.Status) bool {
		m := f.rawStatus()["trajectory_completion"].(map[string]any)
		return m["final_due"] == float64(1) && s.Axes[0].PositionCounts >= 417
	})
	q := float64(s.Axes[0].PositionCounts) * 2 * math.Pi / 131072
	f.trajectory(506, []ipc.Point{{Position: []float64{q, 0}}, {Time: time.Second, Position: []float64{q + .02, 0}}})
	s = f.wait(func(s ipc.Status) bool { return s.Axes[0].PositionCounts >= 830 })
	if s.SafetyFaultMask != 0 {
		t.Fatal("continuous start fault")
	}
}

// Origin: test_runtime.RuntimeTests.test_quantized_motion_at_axis_velocity_limit_is_accepted.
func TestQuantizedMotionAtAxisVelocityLimitIsAccepted(t *testing.T) {
	f := start(t, "simulation.json")
	f.arm(3)
	f.trajectory(507, points(1, .1, 0, nil))
	s := f.wait(func(s ipc.Status) bool { return s.Axes[0].PositionCounts >= 2085 })
	if s.SafetyFaultMask != 0 {
		t.Fatal("quantized velocity fault")
	}
}

// Origin: test_runtime.RuntimeTests.test_stop_cancels_trajectory_and_rearm_does_not_replay.
func TestStopCancelsTrajectoryAndRearmDoesNotReplay(t *testing.T) {
	f := start(t, "simulation.json")
	f.arm(3)
	f.trajectory(501, points(10, .2, 0, []float64{.02, 0}))
	f.wait(func(s ipc.Status) bool { return s.Axes[0].PositionCounts > 150 })
	f.stop()
	p := f.status().Axes[0].PositionCounts
	f.arm(3)
	f.stable(400*time.Millisecond, func(s ipc.Status) bool { return s.Axes[0].PositionCounts == p })
}

// Origin: test_runtime.RuntimeTests.test_discontinuous_first_point_is_rejected_before_movement.
func TestDiscontinuousFirstPointIsRejectedBeforeMovement(t *testing.T) {
	f := start(t, "simulation.json")
	f.arm(3)
	_, err := f.c.Trajectory(ctx, 502, 3, []ipc.Point{{Position: []float64{.4, 0}, Velocity: []float64{0, 0}}, {Time: 5 * time.Second, Position: []float64{.4, 0}, Velocity: []float64{0, 0}}})
	rejected(t, err, 0)
	s := f.wait(func(s ipc.Status) bool { return s.Armed == 0 })
	if s.Axes[0].PositionCounts != 0 || s.SafetyFaultMask == 0 || s.ExecutionFaultReasons&1 == 0 {
		t.Fatalf("discontinuity: %+v", s)
	}
}

// Origin: test_runtime.RuntimeTests.test_excessive_cycle_lateness_revokes_motion_without_replay.
func TestExcessiveCycleLatenessRevokesMotionWithoutReplay(t *testing.T) {
	f := start(t, "simulation.json")
	f.arm(3)
	f.trajectory(503, points(10, .2, 0, []float64{.02, 0}))
	f.wait(func(s ipc.Status) bool { return s.Axes[0].PositionCounts > 150 })
	must(t, f.process.Process.Signal(syscall.SIGSTOP))
	time.Sleep(80 * time.Millisecond)
	must(t, f.process.Process.Signal(syscall.SIGCONT))
	s := f.wait(func(s ipc.Status) bool { return s.Armed == 0 })
	if s.SafetyFaultMask == 0 || s.ExecutionFaultReasons&4 == 0 {
		t.Fatalf("timing fault: %+v", s)
	}
	p := s.Axes[0].PositionCounts
	seq, err := f.c.Enable(ctx, 3)
	submitted(t, seq, err)
	seq, err = f.c.Arm(ctx)
	submitted(t, seq, err)
	f.stable(400*time.Millisecond, func(s ipc.Status) bool { return s.Armed == 0 && s.Axes[0].PositionCounts == p })
}

// Origin: test_runtime.RuntimeTests.test_mixed_drives_and_buffered_position_velocity.
func TestMixedDrivesAndBufferedPositionVelocity(t *testing.T) {
	f := start(t, "simulation.json")
	s := f.arm(3)
	if s.Backend != "simulation" || s.Axes[0].VendorID == s.Axes[1].VendorID {
		t.Fatal("mixed identities")
	}
	f.trajectory(10, points(1, .02, .02, []float64{.02, .02}))
	s = f.wait(func(s ipc.Status) bool { return s.Axes[0].PositionCounts >= 410 && s.Axes[1].PositionCounts <= -410 })
	if s.SafetyFaultMask != 0 || s.Armed != 1 {
		t.Fatalf("mixed motion: %+v", s)
	}
}

// Origin: test_runtime.RuntimeTests.test_disconnect_stops_and_reconnect_does_not_resume.
func TestDisconnectStopsAndReconnectDoesNotResume(t *testing.T) {
	f := start(t, "simulation.json")
	f.arm(3)
	f.trajectory(11, points(5, .1, .1, []float64{.02, .02}))
	f.wait(func(s ipc.Status) bool { return s.Axes[0].PositionCounts > 20 })
	must(t, f.c.Close())
	f.connect()
	s := f.wait(func(s ipc.Status) bool { return s.Armed == 0 && s.AxisEnableMask == 0 })
	p := s.Axes[0].PositionCounts
	f.arm(3)
	f.stable(400*time.Millisecond, func(s ipc.Status) bool { return s.Axes[0].PositionCounts == p })
}

// Origin: test_runtime.RuntimeTests.test_trajectory_limits_reject_entire_upload_before_motion.
func TestTrajectoryLimitsRejectEntireUploadBeforeMotion(t *testing.T) {
	f := start(t, "simulation.json")
	f.arm(3)
	_, err := f.c.Trajectory(ctx, 20, 3, points(10, .6, 0, nil))
	rejected(t, err, 0)
	s := f.wait(func(s ipc.Status) bool { return s.Armed == 0 })
	if s.Axes[0].PositionCounts != 0 {
		t.Fatal("invalid plan moved")
	}
}

// Origin: test_runtime.RuntimeTests.test_implicit_segment_speed_is_checked_without_velocity_feedforward.
func TestImplicitSegmentSpeedIsCheckedWithoutVelocityFeedforward(t *testing.T) {
	f := start(t, "simulation.json")
	f.arm(3)
	_, err := f.c.Trajectory(ctx, 21, 3, points(.1, .1, 0, nil))
	rejected(t, err, 0)
	s := f.wait(func(s ipc.Status) bool { return s.Armed == 0 })
	if s.Axes[0].PositionCounts != 0 {
		t.Fatal("excessive speed moved")
	}
}

// Origin: test_runtime.RuntimeTests.test_disarm_aborts_buffer_and_arm_does_not_resume.
func TestDisarmAbortsBufferAndArmDoesNotResume(t *testing.T) {
	f := start(t, "simulation.json")
	f.arm(3)
	f.trajectory(22, points(5, .1, .1, []float64{.02, .02}))
	f.wait(func(s ipc.Status) bool { return s.Axes[0].PositionCounts > 20 })
	must(t, f.c.Disarm())
	s := f.wait(func(s ipc.Status) bool { return s.Armed == 0 })
	p := s.Axes[0].PositionCounts
	f.arm(3)
	f.stable(400*time.Millisecond, func(s ipc.Status) bool { return s.Axes[0].PositionCounts == p })
}

// Origin: test_runtime.RuntimeTests.test_jog_timeout_stops_motion.
func TestJogTimeoutStopsMotion(t *testing.T) {
	f := start(t, "simulation.json")
	f.arm(3)
	seq, err := f.c.Jog(ctx, 1, []float64{.03, 0}, 120*time.Millisecond)
	submitted(t, seq, err)
	f.wait(func(s ipc.Status) bool { return s.Axes[0].PositionCounts > 0 })
	poll(t, 400*time.Millisecond, func() bool { m := f.motion(); return m.CommandSequence == seq && m.Done })
	p := f.status().Axes[0].PositionCounts
	f.stable(300*time.Millisecond, func(s ipc.Status) bool { return s.Axes[0].PositionCounts == p })
}

// Origin: test_runtime.HomingTests.test_stop_cancels_home_without_commissioning_completion.
func TestStopCancelsHomeWithoutCommissioningCompletion(t *testing.T) {
	f := start(t, "simulation-home.json")
	seq, err := f.c.Home(ctx, 1)
	submitted(t, seq, err)
	f.wait(func(s ipc.Status) bool { return s.NativeHomeActiveMask != 0 })
	f.stop()
	s := f.wait(func(s ipc.Status) bool { return s.NativeHomeActiveMask == 0 })
	if s.HomeValidMask != 0 || s.Armed != 0 {
		t.Fatal("cancelled home published success")
	}
}

// Origin: test_runtime.HomingTests.test_config_readback_and_home_gate.
func TestConfigReadbackAndHomeGate(t *testing.T) {
	f := start(t, "simulation-home.json")
	seq, err := f.c.Enable(ctx, 1)
	submitted(t, seq, err)
	seq, err = f.c.Arm(ctx)
	submitted(t, seq, err)
	f.stable(300*time.Millisecond, func(s ipc.Status) bool { return s.Armed == 0 })
	settings := f.rawStatus()["axes"].([]any)[0].(map[string]any)["startup_drive_configs"].([]any)
	if len(settings) != 1 {
		t.Fatalf("settings=%d", len(settings))
	}
	seq, err = f.c.Home(ctx, 1)
	submitted(t, seq, err)
	f.wait(func(s ipc.Status) bool { return s.HomeValidMask == 1 })
	f.arm(1)
}

// Origin: test_runtime.HomingTests.test_disconnect_during_home_never_publishes_success.
func TestDisconnectDuringHomeNeverPublishesSuccess(t *testing.T) {
	f := start(t, "simulation-home.json")
	seq, err := f.c.Home(ctx, 1)
	submitted(t, seq, err)
	f.wait(func(s ipc.Status) bool { return s.NativeHomeActiveMask != 0 })
	must(t, f.c.Close())
	f.connect()
	s := f.wait(func(s ipc.Status) bool { return s.Armed == 0 && s.NativeHomeActiveMask == 0 })
	if s.HomeValidMask != 0 {
		t.Fatal("disconnected home succeeded")
	}
}

// Origin: test_runtime.HomingTests.test_disarm_cancels_home_without_waiting_for_command_helper.
func TestDisarmCancelsHomeWithoutWaitingForCommandHelper(t *testing.T) {
	f := start(t, "simulation-home.json")
	seq, err := f.c.Home(ctx, 1)
	submitted(t, seq, err)
	f.wait(func(s ipc.Status) bool { return s.NativeHomeActiveMask != 0 })
	must(t, f.c.Disarm())
	s := f.wait(func(s ipc.Status) bool { return s.NativeHomeActiveMask == 0 })
	if s.HomeValidMask != 0 || s.Armed != 0 {
		t.Fatal("disarmed home succeeded")
	}
}

// Origin: test_runtime.LinearTests.test_linear_and_rotary_axes_use_independent_units.
func TestLinearAndRotaryAxesUseIndependentUnits(t *testing.T) {
	f := start(t, "simulation-linear.json")
	f.arm(3)
	f.trajectory(30, points(1, .0002, .02, []float64{.0002, .02}))
	s := f.wait(func(s ipc.Status) bool { return s.Axes[0].PositionCounts >= 2600 && s.Axes[1].PositionCounts <= -410 })
	if s.SafetyFaultMask != 0 {
		t.Fatal("independent units fault")
	}
}

// Origin: test_runtime.ScalarTests.test_signed_and_unsigned_startup_values_read_back_exactly_per_axis.
func TestSignedAndUnsignedStartupValuesReadBackExactlyPerAxis(t *testing.T) {
	f := start(t, "simulation-scalars.json")
	axes := f.rawStatus()["axes"].([]any)
	settings := axes[0].(map[string]any)["startup_drive_configs"].([]any)
	expected := map[string]float64{"u8": 255, "i8": -128, "u16": 65535, "i16": -32768, "u32": 4294967295, "i32": -2147483648}
	if len(settings) != 6 {
		t.Fatal("scalar count")
	}
	for _, v := range settings {
		s := v.(map[string]any)
		k := s["setting_key"].(string)
		want, ok := expected[k]
		if !ok || s["type"] != k || s["commanded"] != want || s["readback"] != want || s["verified"] != float64(1) {
			t.Fatalf("scalar %s: %+v", k, s)
		}
	}
	if len(axes[1].(map[string]any)["startup_drive_configs"].([]any)) != 0 {
		t.Fatal("settings leaked to axis 1")
	}
	f.arm(3)
}
