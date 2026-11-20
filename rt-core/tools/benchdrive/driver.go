//go:build linux

package main

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"sort"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// Unverified polling and renewal cadences: 50,000,000 and 100,000,000 ns.
const (
	pollCadence  = 50 * time.Millisecond
	renewCadence = 100 * time.Millisecond
)

// Unverified on hardware; CiA402 Operation Enabled requires (word & 0x006f)==0x0027.
const (
	operationMask = 0x006f
	operationWord = 0x0027
)

type runner struct {
	o                    options
	c                    *control.Client
	e                    *evidence
	out                  io.Writer
	description          control.Description
	last                 control.Status
	words                map[int]uint16
	expectations         map[string]bool
	owned                bool
	powered              bool
	homeEpoch            uint64
	configEpoch          uint64
	renewDone            chan struct{}
	rows                 []phaseResult
	completedMove        bool
	activePhase          string
	in                   io.ReadCloser
	sessionActive        bool
	sessionTransition    bool
	sessionGeneration    uint64
	retiredStop          control.Fence
	measurements         []axisMeasurement
	velocityMeasurements []*velocityMeasurement
	collisionTuning      []*collisionTuning
	collisionCursor      uint64
	collisionHeader      *ipcclient.TelemetryBatchHeaderV1
	// Per-axis brake-warning and supervisor sequences at the run's first status.
	warningBaseline []warningMark
}

type warningMark struct{ brake, supervisor uint64 }

func newRunner(o options, c *control.Client, e *evidence, out io.Writer) *runner {
	r := &runner{o: o, c: c, e: e, out: out, words: make(map[int]uint16), expectations: make(map[string]bool)}
	for _, name := range []string{"description_binding", "initially_inhibited", "acquire", "renewal", "release"} {
		r.expectations[name] = false
	}
	switch o.phase {
	case "inhibited":
		for _, name := range []string{"home_absent", "second_acquire_refused", "stop_generation", "stop_inhibited"} {
			r.expectations[name] = false
		}
		if o.confirm {
			r.expectations["unhomed_enable_refused"] = false
		}
	case "home":
		r.expectations["home_valid_idle"], r.expectations["home_inhibited"] = false, false
	case "enable", "jog", "move", "halt", "program", "session":
		for _, name := range []string{"home_and_coordinates_valid", "operation_enabled", "armed", "stop_generation", "stop_inhibited"} {
			r.expectations[name] = false
		}
		if o.phase == "enable" {
			r.expectations["hold_observed"] = false
		} else if o.phase == "jog" {
			for _, name := range []string{"jog_limits", "jog_input_observed", "jog_reversal", "jog_end", "jog_after"} {
				r.expectations[name] = false
			}
			if o.direction != 0 {
				delete(r.expectations, "jog_reversal")
			}
		}
		if o.phase == "move" && o.keepArmed {
			delete(r.expectations, "stop_generation")
			delete(r.expectations, "stop_inhibited")
		}
		if o.phase == "program" {
			for _, name := range []string{"program_prepared", "program_endpoint", "program_velocity_bound", "program_handle_terminal"} {
				r.expectations[name] = false
			}
		}
	}
	return r
}

func (r *runner) check(name string, ok bool) error {
	r.expectations[name] = ok
	if !ok {
		return fmt.Errorf("expectation failed: %s", name)
	}
	return nil
}

func (r *runner) summary(err error) error {
	r.printPhases()
	met, missing := []string{}, []string{}
	for name, ok := range r.expectations {
		if ok {
			met = append(met, name)
		} else {
			missing = append(missing, name)
		}
	}
	if err == nil && len(missing) != 0 {
		err = errors.New("required expectations were not met")
	}
	if err != nil {
		missing = append(missing, err.Error())
	}
	sort.Strings(met)
	sort.Strings(missing)
	code, class := outcome(err)
	measured := map[string]any{}
	if len(r.measurements) > 0 {
		measured = map[string]any{"measurements": r.measurements, "standstill_band_counts": standstillBandCounts,
			"standstill_window_ns": standstillWindowNS, "deceleration_window_ns": measurementWindowNS}
	}
	if len(r.velocityMeasurements) > 0 {
		measured["velocity_measurements"] = r.velocityMeasurements
		for _, m := range r.velocityMeasurements {
			fraction := "null (stationary plan)"
			if m.OvershootFraction != nil {
				fraction = fmt.Sprintf("%g", *m.OvershootFraction)
			}
			fmt.Fprintf(r.out, "%s axis=%d peak_observed_velocity_rad_s=%g peak_planned_velocity_rad_s=%g overshoot_fraction=%s axis_max_velocity_rad_s=%g\n",
				m.Phase, m.Axis, m.ObservedPeak, m.PlannedPeak, fraction, m.Limit)
		}
	}
	if len(r.collisionTuning) > 0 {
		measured["collision_watchdog_thresholds"] = r.collisionTuning
		for _, row := range r.collisionTuning {
			fmt.Fprintf(r.out, "%s axis=%d peak_torque_raw=%d peak_following_error_counts=%d margin=%g suggested_torque_abs_max_raw=%d suggested_following_error_counts_max=%d within_compiler_ranges=%t _unverified=true\n",
				row.Phase, row.Axis, row.PeakTorque, row.PeakError, row.Margin, row.SuggestedTorque, row.SuggestedError, row.Usable)
		}
	}
	return errors.Join(err, r.e.write(record{Kind: "summary", Class: class, ExitCode: code, Operation: r.o.phase, Met: met, NotMet: missing, Response: measured}))
}

// command uses the SDK's generated request and fence handling, retaining the
// actual ID for each attempt so evidence cannot confuse a retry with a new action.
func (r *runner) command(ctx context.Context, op string, mask uint32) (control.Response, error) {
	req := control.Request{Operation: op, AxisMask: mask}
	if op == "acquire" {
		req.Controller, req.Binding = r.o.controller, r.o.binding
	}
	return r.request(ctx, req)
}

func (r *runner) request(ctx context.Context, req control.Request) (control.Response, error) {
	ctx, cancel := timeout(ctx, r.o.deadlineNS)
	defer cancel()
	op := req.Operation
	start := ipcclient.HostMonotonicNS()
	response, err := r.c.Command(ctx, &req)
	if err == nil && (response.Operation != op || response.Schema != "rosie.rt-control.response.v1") {
		err = fmt.Errorf("%w: invalid command receipt", control.ErrProtocol)
	}
	if err == nil && (op == "home" || op == "enable" || op == "arm") && response.Sequence == 0 {
		err = fmt.Errorf("%w: missing native command sequence", control.ErrProtocol)
	}
	return response, r.e.receipt(op, start, req, response, err)
}

// status records compact native fields and every observed statusword change.
// Incarnation or stale publication changes fail closed instead of reusing Home.
func (r *runner) status(ctx context.Context) (control.Status, error) {
	ctx, cancel := timeout(ctx, r.o.deadlineNS)
	defer cancel()
	start := ipcclient.HostMonotonicNS()
	s, err := r.c.Status(ctx)
	if err != nil {
		return s, r.e.receipt("status", start, struct{}{}, nil, err)
	}
	core := map[string]any{
		"time_ns": s.Core.TimeNS, "control_generation": s.Core.ControlGeneration,
		"lease_valid": s.Core.LeaseValid, "armed": s.Core.Armed, "axis_enable_mask": s.Core.AxisEnableMask,
		"home_valid_mask": s.Core.HomeValidMask, "home_epoch": s.Core.HomeEpoch,
		"configuration_epoch": s.Core.ConfigurationEpoch, "config_verified_mask": s.Core.ConfigVerifiedMask,
		"commissioning_phase": s.Core.CommissioningPhase, "safety_fault_mask": s.Core.SafetyFaultMask,
		"execution_fault_reasons": s.Core.ExecutionFaultReasons,
		"recovery":                s.Core.Recovery,
	}
	axes := make([]map[string]any, len(s.Core.Axes))
	for i, a := range s.Core.Axes {
		// The SDK decodes core.axes[].pos_counts; publish the public position_counts
		// spelling too so readers do not silently obtain null from the compact record.
		axes[i] = map[string]any{"index": i, "time_ns": a.TimeNS, "statusword": a.Statusword, "pos_counts": a.PositionCounts, "position_counts": a.PositionCounts,
			"readiness": a.Readiness, "home_valid": a.HomeValid, "coordinate_valid": a.CoordinateValid,
			"coordinate_source_valid": a.CoordinateSourceValid, "logical_valid": a.LogicalValid, "pdo_fresh": a.PDOFresh,
			"logical_position": a.LogicalPosition, "logical_target": a.LogicalTarget,
			"torque_raw": a.TorqueRaw, "following_error_counts": a.FollowingErrorCounts,
			"following_error_valid": a.FollowingErrorValid, "collision_watchdog": a.CollisionWatchdog,
			"brake_hold_diagnostic": a.BrakeHoldDiagnostic, "drive_supervisor": a.DriveSupervisor}
		if previous, found := r.words[i]; found && previous != a.Statusword {
			r.e.write(record{Kind: "statusword_transition", Operation: r.activePhase, Response: map[string]any{
				"axis": i, "from": previous, "to": a.Statusword, "time_ns": a.TimeNS, "commissioning_phase": s.Core.CommissioningPhase}})
		}
		r.words[i] = a.Statusword
	}
	core["axes"] = axes
	if r.warningBaseline == nil {
		r.warningBaseline = make([]warningMark, len(s.Core.Axes))
		for i, a := range s.Core.Axes {
			r.warningBaseline[i] = warningMark{a.BrakeHoldDiagnostic.WarningSequence, a.DriveSupervisor.EventSequence}
		}
	}
	err = r.e.write(record{Kind: "status", Operation: "status", StartedNS: start, Request: struct{}{}, Response: map[string]any{
		"core": core, "motion": s.Motion, "execution": s.Execution, "jog": s.Jog, "grant": s.Grant, "generations": s.Generations,
		"daemon_incarnation": s.DaemonIncarnation, "adapter_incarnation": s.AdapterIncarnation}})
	if s.Core.TimeNS == 0 || len(s.Core.Axes) != len(r.description.Axes) || s.Core.TimeNS > ipcclient.HostMonotonicNS() ||
		ipcclient.HostMonotonicNS()-s.Core.TimeNS > uint64(r.o.deadlineNS) {
		err = errors.Join(err, errors.New("missing or stale native status"))
	}
	if r.last.Core.TimeNS != 0 && (s.DaemonIncarnation != r.last.DaemonIncarnation || s.AdapterIncarnation != r.last.AdapterIncarnation || s.Core.TimeNS < r.last.Core.TimeNS) {
		err = errors.Join(err, errors.New("status incarnation or monotonic time changed"))
	}
	r.last = s
	if r.sessionActive {
		err = errors.Join(err, r.sessionHealth(s))
	}
	return s, err
}

// observe grants each condition its own absolute deadline. Neither a successful
// receipt nor an elapsed polling interval substitutes for a native observation.
func (r *runner) observe(ctx context.Context, predicate func(control.Status) (bool, error)) (control.Status, error) {
	ctx, cancel := timeout(ctx, r.o.deadlineNS)
	defer cancel()
	ticker := time.NewTicker(pollCadence)
	defer ticker.Stop()
	for {
		s, err := r.status(ctx)
		if err != nil {
			if errors.Is(ctx.Err(), context.Canceled) && errors.Is(err, ctx.Err()) {
				return s, context.Cause(ctx)
			}
			return s, err
		}
		ok, err := predicate(s)
		if err != nil || ok {
			return s, err
		}
		select {
		case <-ctx.Done():
			return s, context.Cause(ctx)
		case <-ticker.C:
		}
	}
}

func (r *runner) inhibited(s control.Status) bool {
	return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Core.CommissioningPhase == 0
}

// run joins renewal and command producers before terminal cleanup. Stop and
// Release use fresh contexts even on cancellation, preventing a signal or lost
// observation from skipping output inhibition and authority retirement.
func (r *runner) run(parent context.Context) (err error) {
	ctx, cancel := context.WithCancelCause(parent)
	defer cancel(nil)
	defer func() {
		r.stopRenewal()
		if ctx.Err() != nil && !errors.Is(err, context.Cause(ctx)) {
			err = errors.Join(err, context.Cause(ctx))
		}
		// Signal contexts can carry a distinct cause. Retain caller cancellation
		// for classification without misclassifying helper failures as interrupts.
		if errors.Is(parent.Err(), context.Canceled) && !errors.Is(err, context.Canceled) {
			err = errors.Join(err, context.Canceled)
		}
		if r.owned || r.c.Fence().Session != "" {
			if err != nil || (r.o.phase != "home" && !(r.o.phase == "move" && r.o.keepArmed && r.completedMove)) {
				err = errors.Join(err, r.phase("stop", func() error { return r.stop(context.Background()) }))
			}
			// Renewal can report revocation before metrics publishes its fault.
			// Retain the fault observed during Stop before Release retires authority.
			err = r.retainObservedFault(err)
			releaseErr := r.phase("release", func() error { return r.release(context.Background()) })
			err = errors.Join(err, releaseErr)
			if r.powered {
				// A Stop/Release receipt may precede the drive-managed brake
				// transition. Preserve late faults in the overall verdict.
				err = errors.Join(err, r.postDisable(context.Background()))
			}
		}
		err = errors.Join(err, r.e.failure())
	}()
	dctx, done := timeout(ctx, r.o.deadlineNS)
	start := ipcclient.HostMonotonicNS()
	r.description, err = r.c.Describe(dctx)
	done()
	if err = r.e.receipt("describe", start, struct{}{}, r.description, err); err != nil {
		return err
	}
	d := r.description
	if r.o.mask == 0 {
		r.o.mask = uint64(describedAxisMask(d))
	}
	fmt.Fprintf(r.out, "backend=%s configuration=%s machine=%s deployment=%s capabilities=%s\n", d.Backend, d.ConfigurationSHA256, d.MachineSHA256, d.DeploymentSHA256, d.CapabilitiesDigest)
	for _, capability := range d.Capabilities {
		if capability.State != "implemented" {
			fmt.Fprintf(r.out, "capability %s: %s\n", capability.Name, capability.State)
		}
	}
	if err = r.check("description_binding", d.ContractVersion == 2 && len(d.Axes) > 0 && len(d.Axes) <= 16 && r.o.mask != 0 && r.o.mask>>len(d.Axes) == 0 &&
		(r.o.binding.MachineSHA256 == "" || r.o.binding.MachineSHA256 == d.MachineSHA256) &&
		(r.o.binding.ConfigurationSHA256 == "" || r.o.binding.ConfigurationSHA256 == d.ConfigurationSHA256)); err != nil {
		return err
	}
	for i, a := range d.Axes {
		if a.Index != uint32(i) {
			return errors.New("Describe axis indices are not contiguous")
		}
	}
	if r.o.phase == "program" {
		if err = programAxes(d); err != nil {
			return err
		}
	}
	s, err := r.status(ctx)
	if err != nil {
		return err
	}
	if err = r.check("initially_inhibited", r.inhibited(s) && (r.o.phase == "recovery" || s.Core.SafetyFaultMask == 0)); err != nil {
		return err
	}
	r.homeEpoch, r.configEpoch = s.Core.HomeEpoch, s.Core.ConfigurationEpoch
	if r.o.phase == "inhibited" || r.o.phase == "sequence" {
		if err = r.check("home_absent", s.Core.HomeValidMask&uint32(r.o.mask) == 0); err != nil {
			return err
		}
		for i, a := range d.Axes {
			if r.o.mask&(1<<i) != 0 && !a.RequireHome {
				return errors.New("inhibited refusal check requires Home-gated axes")
			}
		}
	} else if r.o.phase == "home" {
		for i, a := range d.Axes {
			if r.o.mask&(1<<i) != 0 && !a.NativeHome {
				return fmt.Errorf("axis %d has no native Home capability", i)
			}
		}
	} else if r.o.phase == "enable" || r.o.phase == "jog" || r.o.phase == "move" || r.o.phase == "halt" || r.o.phase == "program" || r.o.phase == "session" {
		for i, a := range s.Core.Axes {
			if r.o.mask&(1<<i) != 0 {
				fmt.Fprintf(r.out, "axis %d readiness=%s\n", i, a.Readiness)
			}
		}
		if err = r.preEnable(s); err != nil {
			return err
		}
	}
	if r.o.phase == "jog" {
		if err = r.jogLimits(s); err != nil {
			return err
		}
	}
	if _, err = r.command(ctx, "acquire", 0); err != nil {
		// A lost acquisition receipt can leave a native grant. Attempt urgent
		// inhibition on an uncertain outcome, but never Stop a definite refusal
		// belonging to another controller.
		r.owned = errors.Is(err, control.ErrTransport) || errors.Is(err, control.ErrProtocol)
		return err
	}
	r.owned = true
	if err = r.check("acquire", r.c.Grant().Session != "" && r.c.Grant().Generation != 0 && !r.c.Grant().Stopping); err != nil {
		return err
	}
	if err = r.startRenewal(ctx, cancel); err != nil {
		return err
	}
	switch r.o.phase {
	case "inhibited":
		return r.phase("inhibited", func() error { return r.inhibitedPhase(ctx) })
	case "home":
		return r.phase("home", func() error { return r.home(ctx) })
	case "discard":
		return r.phase("discard", func() error { return r.discard(ctx) })
	case "recovery":
		return r.phase("recovery", func() error { return r.recovery(ctx, cancel) })
	case "telemetry":
		return r.phase("telemetry", func() error { return r.telemetry(ctx) })
	case "sequence":
		return r.sequence(ctx)
	default:
		if err = r.enable(ctx); err != nil {
			return err
		}
		if r.o.phase == "jog" {
			return r.phase("jog", func() error { return r.jog(ctx, cancel) })
		}
		if r.o.phase == "move" || r.o.phase == "halt" {
			return r.phase(r.o.phase, func() error { return r.move(ctx) })
		}
		if r.o.phase == "program" {
			return r.phase("program", func() error { return r.program(ctx) })
		}
		if r.o.phase == "session" {
			return r.phase("session", func() error { return r.session(ctx, cancel) })
		}
		return r.phase("hold", func() error { return r.hold(ctx) })
	}
}

// retainObservedFault runs after Stop observation, preserving the fault that
// fenced renewal without delaying urgent inhibition for another status read.
func (r *runner) retainObservedFault(err error) error {
	if fault := sessionFault(r.last); err != nil && fault != nil {
		for i := range r.rows {
			if !r.rows[i].Met {
				r.rows[i].Reasons = append([]string{fault.Error()}, r.rows[i].Reasons...)
			}
		}
		return errors.Join(fault, err)
	}
	return err
}

func (r *runner) startRenewal(ctx context.Context, cancel context.CancelCauseFunc) error {
	updates, err := r.c.StartRenewal(ctx, renewCadence)
	if err != nil {
		return err
	}
	r.renewDone = make(chan struct{})
	renewed := make(chan struct{}, 1)
	renewFence := r.c.Fence()
	go func() {
		defer close(r.renewDone)
		defer func() {
			if value := recover(); value != nil {
				cancel(fmt.Errorf("renewal reader panic: %v", value))
			}
		}()
		for update := range updates {
			e := r.e.receipt("renew", update.StartedHostMonotonicNS, renewFence, update.Grant, update.Err)
			if e == nil && update.Grant.Stopping {
				e = errors.New("renewal reports stopping")
			}
			if e != nil {
				cancel(e)
				return
			}
			renewFence = control.Fence{Session: update.Grant.Session, Generation: update.Grant.Generation}
			select {
			case renewed <- struct{}{}:
			default:
			}
		}
	}()
	renewCtx, stopWait := timeout(ctx, r.o.deadlineNS)
	select {
	case <-renewed:
		r.expectations["renewal"] = true
	case <-renewCtx.Done():
		err = context.Cause(renewCtx)
	}
	stopWait()
	return err
}

func (r *runner) inhibitedPhase(ctx context.Context) error {
	_, err := r.command(ctx, "acquire", 0)
	var refused *control.Rejected
	if e := r.check("second_acquire_refused", errors.As(err, &refused) && refused.Reason == "control_already_owned"); e != nil {
		return errors.Join(e, err)
	}
	if !r.o.confirm {
		return r.e.write(record{Kind: "skipped", Operation: "enable", Reason: "motion_confirmation_required",
			Response: "Enable-refusal probe skipped: --confirm-motion is required because a missing daemon Home gate could energise the drive."})
	}
	_, err = r.command(ctx, "enable", uint32(r.o.mask))
	if e := r.check("unhomed_enable_refused", errors.As(err, &refused) && refused.Reason == "native_rejected" && refused.NativeResult != nil && refused.NativeResult.Reason == ipcclient.ReasonNotReady); e != nil {
		return errors.Join(e, err)
	}
	return nil
}

func (r *runner) home(ctx context.Context) error {
	for i, a := range r.description.Axes {
		if r.o.mask&(1<<i) != 0 && !a.NativeHome {
			return fmt.Errorf("axis %d has no native Home capability", i)
		}
	}
	ctx, cancel := timeout(ctx, r.o.deadlineNS)
	defer cancel()
	r.powered = true
	epoch := r.last.Core.HomeEpoch
	result := make(chan error, 1)
	go func() { _, err := r.command(ctx, "home", uint32(r.o.mask)); result <- err }()
	received := false
	defer func() {
		cancel()
		if !received {
			<-result
		}
	}()
	_, err := r.observe(ctx, func(s control.Status) (bool, error) {
		if s.Core.SafetyFaultMask != 0 {
			return false, errors.New("Home safety fault")
		}
		if !received {
			select {
			case err := <-result:
				received = true
				if err != nil {
					return false, err
				}
			default:
			}
		}
		return received && s.Core.HomeValidMask&uint32(r.o.mask) == uint32(r.o.mask) && s.Core.CommissioningPhase == 0 && s.Core.HomeEpoch > epoch, nil
	})
	if err != nil {
		return err
	}
	r.expectations["home_valid_idle"] = true
	r.homeEpoch, r.configEpoch = r.last.Core.HomeEpoch, r.last.Core.ConfigurationEpoch
	for i, a := range r.last.Core.Axes {
		if r.o.mask&(1<<i) != 0 {
			fmt.Fprintf(r.out, "axis %d home_valid=%t\n", i, a.HomeValid)
			if err := r.e.write(record{Kind: "home_axis", Operation: "home", Response: map[string]any{"axis": i, "home_valid": a.HomeValid}}); err != nil {
				return err
			}
		}
	}
	return r.check("home_inhibited", r.inhibited(r.last))
}

func (r *runner) preEnable(s control.Status) error {
	for i, a := range s.Core.Axes {
		if r.enabledMask()&(1<<i) == 0 {
			continue
		}
		// Native logical_valid applies the configured coordinate model, including
		// Home and continuous-source requirements; coordinate_valid alone describes
		// the optional continuous counter and is zero on nonwrapping Home axes.
		if !a.HomeValid || !a.LogicalValid || s.Core.HomeValidMask&(1<<i) == 0 {
			return fmt.Errorf("axis %d: Home or valid coordinates missing (%s)", i, a.Readiness)
		}
		if a.Readiness != "ready" && a.Readiness != "not_enabled" && a.Readiness != "not_operation_enabled" && a.Readiness != "brake_wait" {
			return fmt.Errorf("axis %d unexpected readiness: %s", i, a.Readiness)
		}
	}
	return r.check("home_and_coordinates_valid", true)
}

func (r *runner) ready(s control.Status, armed bool) error {
	if err := sessionFault(s); err != nil {
		return err
	}
	if s.Core.HomeEpoch != r.homeEpoch || s.Core.ConfigurationEpoch != r.configEpoch {
		return errors.New("Home or configuration epoch changed during motion qualification")
	}
	if r.c.Grant().Stopping || s.Core.LeaseValid != 1 || s.Core.SafetyFaultMask != 0 || s.Core.CommissioningPhase != 0 || s.Core.AxisEnableMask != r.enabledMask() || (armed && s.Core.Armed != 1) {
		return errors.New("unexpected group readiness")
	}
	for i, a := range s.Core.Axes {
		if r.enabledMask()&(1<<i) != 0 && (a.Readiness != "ready" || a.Statusword&operationMask != operationWord || !a.HomeValid || !a.LogicalValid) {
			return fmt.Errorf("axis %d unexpected readiness: %s, statusword=%d", i, a.Readiness, a.Statusword)
		}
	}
	return nil
}

func (r *runner) enable(ctx context.Context) error {
	if err := r.phase("enable", func() error { return r.enableSelection(ctx) }); err != nil {
		return err
	}
	return r.phase("arm", func() error { return r.arm(ctx) })
}

func (r *runner) enableSelection(ctx context.Context) error {
	if _, err := r.command(ctx, "enable", r.enabledMask()); err != nil {
		return err
	}
	_, err := r.observe(ctx, func(s control.Status) (bool, error) {
		if s.Core.SafetyFaultMask != 0 {
			return false, errors.New("Enable safety fault")
		}
		if err := r.preEnable(s); err != nil {
			return false, err
		}
		return s.Core.AxisEnableMask == r.enabledMask() && s.Core.CommissioningPhase == 0, nil
	})
	if err != nil {
		return err
	}
	return r.check("enable_selected", true)
}

func (r *runner) arm(ctx context.Context) error {
	// The daemon's want_enable is armed && enabled. Observe selection before
	// Arm, then require physical readiness before hold/jog; waiting for DS402
	// Operation Enabled before Arm would deadlock this native contract.
	r.powered = true // Include uncertain command outcomes in inhibited verification.
	if _, err := r.command(ctx, "arm", 0); err != nil {
		return err
	}
	_, err := r.observe(ctx, func(s control.Status) (bool, error) {
		if err := r.preEnable(s); err != nil {
			return false, err
		}
		if s.Core.SafetyFaultMask != 0 || s.Core.CommissioningPhase != 0 {
			return false, errors.New("Arm safety or commissioning gate changed")
		}
		return r.ready(s, true) == nil, nil
	})
	if err == nil {
		r.expectations["operation_enabled"] = true
		r.expectations["armed"] = true
	}
	return err
}

// stop confirms the receipt against a later inhibited publication. An empty
// returned session additionally requires lease retirement before cleanup may
// accept an already stale Release (docs/APPLICATION_API.md:1238).
func (r *runner) stop(ctx context.Context) error {
	r.retiredStop = control.Fence{}
	before := r.c.Fence().Generation
	response, err := r.command(ctx, "stop", 0)
	if err != nil {
		return err
	}
	// Stop drains writers and mirrors a replacement grant before replying.
	// Only a later publication can establish the resumed session's native fence.
	completed := ipcclient.HostMonotonicNS()
	var grant control.Grant
	if err := json.Unmarshal(response.Data, &grant); err != nil {
		return err
	}
	err = r.check("stop_generation", grant.Generation > before)
	_, observed := r.observe(ctx, func(s control.Status) (bool, error) {
		return s.Core.TimeNS >= completed && r.inhibited(s) && (grant.Session != "" || s.Core.LeaseValid == 0), nil
	})
	if observed == nil {
		r.expectations["stop_inhibited"] = true
	}
	if err == nil && observed == nil && grant.Session == "" {
		r.retiredStop = control.Fence{Generation: grant.Generation}
	}
	return errors.Join(err, observed)
}

func (r *runner) position(label string, s control.Status, origin []int32) error {
	positions, drift := []int32{}, []int64{}
	for i, a := range s.Core.Axes {
		positions = append(positions, a.PositionCounts)
		if origin != nil {
			drift = append(drift, int64(a.PositionCounts)-int64(origin[i]))
		}
	}
	return r.e.write(record{Kind: "position", Operation: label, Response: map[string]any{"time_ns": s.Core.TimeNS, "pos_counts": positions, "drift_counts": drift}})
}

func (r *runner) hold(ctx context.Context) error {
	origin := make([]int32, len(r.last.Core.Axes))
	for i, a := range r.last.Core.Axes {
		origin[i] = a.PositionCounts
	}
	if err := r.position("hold_before", r.last, origin); err != nil {
		return err
	}
	end := time.Now().Add(time.Duration(r.o.holdNS))
	ticker := time.NewTicker(pollCadence)
	defer ticker.Stop()
	for {
		s, err := r.status(ctx)
		if err != nil {
			return err
		}
		if err := r.ready(s, true); err != nil {
			return err
		}
		if err := r.position("hold_drift", s, origin); err != nil {
			return err
		}
		if !time.Now().Before(end) {
			return r.check("hold_observed", true)
		}
		select {
		case <-ctx.Done():
			return context.Cause(ctx)
		case <-ticker.C:
		}
	}
}
