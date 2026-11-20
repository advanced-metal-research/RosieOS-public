//go:build linux

package main

import (
	"context"
	"crypto/sha256"
	"errors"
	"fmt"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"strings"
	"time"

	"rosieos/rt-core/ipcclient"
	control "rosieos/rt-core/sdk/control"
)

// Unverified workload: maximum protocol plan size, with 1,000 ns point spacing.
const (
	planPoints  = 250000
	pointPeriod = time.Microsecond
)

// Unverified jog workload: 200 updates at 10 ms intervals, alternating direction
// across iterations at 0.001 rad/s; 100 ms input expiry bounds abandoned input.
const (
	jogUpdates   = 200
	jogCadence   = 10 * time.Millisecond
	jogVelocity  = 0.001
	jogInputLife = 100 * time.Millisecond
)

// Measured fixture scaling: simulation-program.json declares 131072 counts/rev.
const countsPerRevolution = 131072

// Measured native storage contract: three simultaneously resident plan slots.
const nativeSlotCapacity = 3

func runSoak(parent context.Context, opts options) (r record, resultErr error) {
	r = record{Schema: "rosie.rt-core.soak.v2", StartedUTC: time.Now().UTC(), Status: "running", Unavailable: map[string]string{}}
	r.Configuration = configuration{options: opts, CPUCount: runtime.NumCPU(), PageBytes: uint64(os.Getpagesize()), RSSPagesPerIteration: rssPagesPerIteration,
		WarmupIterations: warmupIterations, Points: planPoints, PlanDurationNS: uint64((planPoints - 1) * pointPeriod), JogUpdates: jogUpdates, JogCadenceNS: uint64(jogCadence), JogVelocity: jogVelocity,
		SamplePeriodNS: uint64(samplePeriod), ObservationPeriodNS: uint64(observationPeriod)}
	defer func() {
		r.CompletedUTC = time.Now().UTC()
		if resultErr != nil {
			r.Errors = append(r.Errors, resultErr.Error())
		}
		r.assess()
		r.Status = strings.ToLower(r.verdict())
		if r.Status == "fail" {
			resultErr = errors.Join(resultErr, errors.New("soak acceptance failed; see assertions and unavailable fields"))
		}
	}()
	git := exec.CommandContext(parent, "bash", "tools/packageinfo/git.sh", "rev-parse", "HEAD")
	git.Dir = opts.Root
	raw, err := git.CombinedOutput()
	if err != nil {
		return r, fmt.Errorf("source identity: %w: %s", err, raw)
	}
	r.Configuration.GitSHA = strings.TrimSpace(string(raw))
	if raw, err := os.ReadFile("/proc/sys/kernel/osrelease"); err == nil {
		r.Configuration.Kernel = strings.TrimSpace(string(raw))
	}
	if raw, err := os.ReadFile("/proc/cpuinfo"); err == nil {
		for _, line := range strings.Split(string(raw), "\n") {
			if strings.HasPrefix(line, "model name") {
				_, model, _ := strings.Cut(line, ":")
				r.Configuration.CPUModel = strings.TrimSpace(model)
				break
			}
		}
	}
	r.Configuration.BinarySHA256 = map[string]string{}
	for _, name := range []string{"rosie-rt-core-sim", "rt-control", "rtctl"} {
		raw, err := os.ReadFile(filepath.Join(opts.Root, "build", name))
		if err != nil {
			return r, err
		}
		r.Configuration.BinarySHA256[name] = fmt.Sprintf("%x", sha256.Sum256(raw))
	}
	s, err := startService(parent, &r)
	if err != nil {
		return r, err
	}
	defer func() { resultErr = errors.Join(resultErr, s.close()) }()
	ctx, cancel := context.WithCancel(parent)
	defer cancel()
	o := &observer{r: &r, s: s, phase: "warmup"}
	observerDone := make(chan error, 1)
	go func() {
		err := o.run(ctx)
		if err != nil {
			cancel()
		}
		observerDone <- err
	}()
	defer func() { cancel(); resultErr = errors.Join(resultErr, <-observerDone) }()
	defer func() {
		if resultErr != nil {
			lastCtx, lastCancel := context.WithTimeout(context.Background(), operationBudget)
			defer lastCancel()
			status, statusErr := s.client.Status(lastCtx)
			if statusErr == nil {
				r.FailureStatus = &status
			}
			jog, jogErr := s.client.JogStatus(lastCtx)
			if jogErr == nil {
				r.FailureJog = &jog
			}
			resultErr = errors.Join(resultErr, statusErr, jogErr, o.observeEvents(lastCtx))
			final, err := o.sample(lastCtx, "failure_final")
			if err == nil {
				r.Final = &final
			} else {
				resultErr = errors.Join(resultErr, err)
			}
		}
	}()
	if err := s.acquire(ctx); err != nil {
		return r, err
	}
	if _, err := o.sample(ctx, "startup"); err != nil {
		return r, err
	}
	for i := 1; i <= warmupIterations+opts.Iterations; i++ {
		phase, n := "run", i-warmupIterations
		if i <= warmupIterations {
			phase, n = "warmup", i
		}
		o.stage(phase, n)
		it, err := runIteration(ctx, s, o, n, phase == "warmup")
		it.Phase, it.Number = phase, n
		o.mu.Lock()
		r.Iterations = append(r.Iterations, it)
		if err == nil && phase == "run" {
			r.IterationsCompleted++
		}
		o.mu.Unlock()
		if err != nil {
			return r, fmt.Errorf("%s iteration %d: %w", phase, n, err)
		}
		fmt.Fprintf(os.Stderr, "soak %s iteration %d complete\n", phase, n)
		if i == warmupIterations {
			baseline, err := o.sample(ctx, "after_warmup")
			if err != nil {
				return r, err
			}
			r.Baseline = &baseline
		}
	}
	call, stop := context.WithTimeout(ctx, operationBudget)
	_, err = s.client.Release(call)
	stop()
	if err != nil {
		return r, err
	}
	if _, err := s.waitStatus(ctx, func(v control.Status) bool { return v.Core.Armed == 0 && v.Core.AxisEnableMask == 0 }); err != nil {
		return r, err
	}
	final, err := o.sample(ctx, "final")
	if err != nil {
		return r, err
	}
	r.Final = &final
	if err := o.observeEvents(ctx); err != nil {
		return r, err
	}
	return r, nil
}

func currentPosition(ctx context.Context, s *service) ([]float64, error) {
	call, cancel := context.WithTimeout(ctx, operationBudget)
	defer cancel()
	state, err := s.client.Status(call)
	if err != nil {
		return nil, err
	}
	if len(state.Core.Axes) != 9 {
		return nil, errors.New("soak requires its nine-axis fixture")
	}
	position := make([]float64, 9)
	position[0] = float64(state.Core.Axes[0].PositionCounts) * 2 * math.Pi / countsPerRevolution
	return position, nil
}

func fullPlan(position []float64) []control.Point {
	points := make([]control.Point, planPoints)
	for i := range points {
		points[i] = control.Point{Time: time.Duration(i) * pointPeriod, Position: position}
	}
	return points
}

func prepare(ctx context.Context, s *service, position []float64) (control.Response, string, error) {
	call, cancel := context.WithTimeout(ctx, uploadBudget)
	defer cancel()
	id, err := control.NewRequestID()
	if err != nil {
		return control.Response{}, "", err
	}
	request := control.Request{Operation: "prepare_trajectory", RequestID: &id, AxisMask: motionMask, Points: fullPlan(position)}
	reply, err := s.client.Command(call, &request)
	return reply, id, err
}

// command gives each lifecycle request its own receipt budget; a preceding bulk
// upload cannot consume that budget before the lifecycle request is submitted.
func command(ctx context.Context, s *service, request control.Request) (control.Response, error) {
	call, cancel := context.WithTimeout(ctx, operationBudget)
	defer cancel()
	return s.client.Command(call, &request)
}

func stopMotion(ctx context.Context, s *service) error {
	call, cancel := context.WithTimeout(ctx, operationBudget)
	defer cancel()
	if _, err := s.client.Stop(call); err != nil {
		return err
	}
	_, err := s.waitStatus(ctx, func(v control.Status) bool { return v.Core.Armed == 0 && v.Core.AxisEnableMask == 0 })
	return err
}

func runIteration(ctx context.Context, s *service, o *observer, n int, warmup bool) (it iteration, err error) {
	if err = s.enable(ctx, motionMask); err != nil {
		return it, err
	}
	position, err := currentPosition(ctx, s)
	if err != nil {
		return it, err
	}
	it.UploadStartedNS = ipcclient.HostMonotonicNS()
	prepared, id, err := prepare(ctx, s, position)
	it.UploadCompletedNS, it.RequestID, it.Handle = ipcclient.HostMonotonicNS(), id, prepared.Handle
	if err != nil {
		return it, fmt.Errorf("prepare: %w", err)
	}
	if warmup || n%4 == 0 {
		it.DiscardedHandle, it.DiscardedRequestID = it.Handle, it.RequestID
		it.DiscardUploadStartedNS, it.DiscardUploadCompletedNS = it.UploadStartedNS, it.UploadCompletedNS
		if _, err = command(ctx, s, control.Request{Operation: "discard_trajectory", Handle: it.Handle}); err != nil {
			return it, err
		}
		it.Discarded = true
		it.UploadStartedNS = ipcclient.HostMonotonicNS()
		prepared, it.RequestID, err = prepare(ctx, s, position)
		it.UploadCompletedNS = ipcclient.HostMonotonicNS()
		if err != nil {
			return it, err
		}
		it.Handle = prepared.Handle
	}
	started, err := command(ctx, s, control.Request{Operation: "start_trajectory", Handle: it.Handle})
	if err != nil {
		return it, fmt.Errorf("start: %w", err)
	}
	it.StartSequence = started.Sequence
	{
		_, err = s.waitStatus(ctx, func(v control.Status) bool {
			return v.Motion.TrajectoryID == it.Handle && v.Motion.CommandSequence == started.Sequence && v.Motion.Done && v.Motion.State == 4 && v.Motion.PointIndex == planPoints-1
		})
		if err != nil {
			return it, fmt.Errorf("correlated completion: %w", err)
		}
	}
	it.Completed = true
	if _, err = command(ctx, s, control.Request{Operation: "discard_trajectory", Handle: it.Handle}); err != nil {
		return it, err
	}
	it.ExecutedDiscarded = true
	if err = jogDuringUpload(ctx, s, o, &it, n); err != nil {
		return it, err
	}
	if err = stopMotion(ctx, s); err != nil {
		return it, err
	}
	if it.SlotCapacityProbe, err = probeSlots(ctx, s); err != nil {
		return it, err
	}
	if _, err = s.waitStatus(ctx, func(v control.Status) bool {
		for _, h := range v.Execution.Handles {
			if h.State == "prepared" || h.State == "started" {
				return false
			}
		}
		used, occupancyErr := occupiedSlots(v.BufferHealth)
		return occupancyErr == nil && used == 0 && v.Adapter.OpenPreparationSlots == 2 && v.Core.Armed == 0 && v.Core.AxisEnableMask == 0
	}); err != nil {
		return it, err
	}
	it.End, err = o.sample(ctx, "iteration_end")
	return it, err
}

// jogDuringUpload runs motion only between plans. The overlap upload is inert
// and discarded; its old starting position can never be activated after jog.
func jogDuringUpload(ctx context.Context, s *service, o *observer, it *iteration, n int) (resultErr error) {
	position, err := currentPosition(ctx, s)
	if err != nil {
		return err
	}
	call, cancel := context.WithTimeout(ctx, operationBudget)
	jog, err := s.client.PrepareJogSession(call, motionMask)
	cancel()
	if err != nil {
		return fmt.Errorf("begin jog: %w", err)
	}
	endJog := func() error {
		call, cancel := context.WithTimeout(ctx, operationBudget)
		defer cancel()
		return jog.End(call)
	}
	defer func() { resultErr = errors.Join(resultErr, endJog()) }()
	bulkCtx, bulkCancel := context.WithCancel(ctx)
	defer bulkCancel()
	type uploadResult struct {
		reply     control.Response
		err       error
		completed uint64
	}
	uploaded := make(chan uploadResult, 1)
	var completedUpload *uploadResult
	o.uploading(true)
	it.OverlapStartedNS = ipcclient.HostMonotonicNS()
	go func() {
		reply, _, err := prepare(bulkCtx, s, position)
		o.uploading(false)
		uploaded <- uploadResult{reply, err, ipcclient.HostMonotonicNS()}
	}()
	defer func() {
		if resultErr != nil {
			bulkCancel()
		}
		if completedUpload == nil {
			result := <-uploaded
			completedUpload = &result
		}
		result := *completedUpload
		it.OverlapCompletedNS, it.OverlapHandle = result.completed, result.reply.Handle
		if result.err != nil {
			resultErr = errors.Join(resultErr, fmt.Errorf("jog overlap upload: %w", result.err))
			return
		}
		cleanup, cancel := context.WithTimeout(ctx, operationBudget)
		defer cancel()
		_, err := s.client.DiscardTrajectory(cleanup, result.reply.Handle)
		it.OverlapDiscarded = err == nil
		resultErr = errors.Join(resultErr, err)
	}()
	ticker := time.NewTicker(jogCadence)
	defer ticker.Stop()
	velocity := jogVelocity
	if n%2 == 0 {
		velocity = -velocity
	}
	var last uint64
	for i := 1; i <= jogUpdates || completedUpload == nil; i++ {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-ticker.C:
		}
		select {
		case result := <-uploaded:
			completedUpload = &result
		default:
		}
		now := ipcclient.HostMonotonicNS()
		if last != 0 {
			it.JogCadenceMaxNS = max(it.JogCadenceMaxNS, now-last)
		}
		last = now
		if err := jog.Update([]float64{velocity}, now+uint64(jogInputLife)); err != nil {
			return err
		}
		it.JogSent++
	}
	// Last source acknowledgement proves actual native input admission, not
	// merely that 200 datagram sends returned successfully.
	err = poll(ctx, operationBudget, func() (bool, error) {
		call, cancel := context.WithTimeout(ctx, operationBudget)
		defer cancel()
		v, err := s.client.JogStatus(call)
		var rejected *control.Rejected
		if errors.As(err, &rejected) && rejected.Reason == "RTCore jog observation unavailable" {
			return false, nil
		}
		if err != nil {
			return false, err
		}
		if v.Open == 0 || v.UpdateReason != ipcclient.JogReasonNone {
			return false, fmt.Errorf("jog acknowledgement lost: observed_sequence=%d update_reason=%d state_reason=%d open=%d", v.ObservedSourceSequence, v.UpdateReason, v.StateReason, v.Open)
		}
		return v.ObservedSourceSequence == uint64(it.JogSent) && v.UpdateReason == ipcclient.JogReasonNone && v.HasInput == 1, nil
	})
	return errors.Join(err, endJog())
}

// probeSlots proves all three native reservations can coexist through public
// Prepare receipts. It is not a replacement for a continuous occupancy counter.
func probeSlots(ctx context.Context, s *service) (count int, resultErr error) {
	position, err := currentPosition(ctx, s)
	if err != nil {
		return 0, err
	}
	var handles []uint64
	call, cancel := context.WithTimeout(ctx, operationBudget)
	defer cancel()
	defer func() {
		for _, handle := range handles {
			_, err := s.client.DiscardTrajectory(call, handle)
			resultErr = errors.Join(resultErr, err)
		}
	}()
	for i := 0; i < nativeSlotCapacity; i++ {
		r, err := s.client.PrepareTrajectory(call, motionMask, []control.Point{{Position: position}, {Time: time.Millisecond, Position: position}})
		if err != nil {
			return count, fmt.Errorf("native capacity reservation %d: %w", i+1, err)
		}
		handles = append(handles, r.Handle)
		count++
	}
	return count, nil
}

// absoluteRoot locates the same source tree from package tests without chdir,
// preventing interference with other packages running on the shared host.
func absoluteRoot() (string, error) { return filepath.Abs("../..") }
