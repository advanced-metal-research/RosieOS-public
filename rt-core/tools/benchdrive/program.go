//go:build linux

package main

import (
	"context"
	"crypto/sha256"
	"errors"
	"fmt"
	"math"
	"time"

	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// Unverified operator default: 0.2 rad; the described soft limits still apply.
const defaultProgramDelta = 0.2

// Corpus dense_joint_trajectory_reference.rdt declares dt_s=0.01 (10,000,000 ns).
// Unverified on hardware; this is the source program grid, not the daemon cycle.
const programDT = 10 * time.Millisecond

// Unverified bench mapping: the dense format includes all nine J1..J9 axes.
const programAxisMask uint32 = (1 << densejointtraj.AxisCount) - 1

// describedAxisMask returns only valid described axes; invalid geometry cannot
// turn a default into an all-bits enable request.
func describedAxisMask(d control.Description) uint32 {
	if len(d.Axes) == 0 || len(d.Axes) > ipcclient.MaxAxes {
		return 0
	}
	return (1 << len(d.Axes)) - 1
}

// enabledMask includes the stationary described axes because native Start requires
// every uploaded axis enabled; a partial motion mask must not skip their gates.
func (r *runner) enabledMask() uint32 {
	if r.o.phase == "program" {
		return describedAxisMask(r.description)
	}
	return uint32(r.o.mask)
}

// programAxes restricts this bench producer to the J1..J9 order and radian
// coordinates, preventing a different machine layout from reinterpreting samples.
func programAxes(d control.Description) error {
	if len(d.Axes) != densejointtraj.AxisCount {
		return errors.New("program requires exactly nine J1..J9 axes")
	}
	for i, a := range d.Axes {
		if a.Index != uint32(i) || a.ID != fmt.Sprintf("J%d", i+1) || a.PositionUnit != "rad" {
			return errors.New("program requires J1..J9 in Describe order and radian units")
		}
	}
	return nil
}

// programProfile samples the bounded move trapezoid on the corpus grid, with
// stationary axes held at their published targets and zero boundary velocities.
// The move reserve leaves room for measured feedback overshoot. Capacity and
// every axis's limits are checked before upload, preventing native invalidation
// of prepared handles by an out-of-limit program.
func programProfile(d control.Description, o options, s control.Status) (*densejointtraj.Plan, error) {
	if err := programAxes(d); err != nil {
		return nil, err
	}
	if len(s.Core.Axes) != densejointtraj.AxisCount || o.mask == 0 || o.mask > uint64(programAxisMask) || d.CycleNS == 0 || o.binding.Revision == 0 {
		return nil, errors.New("invalid program geometry or revision")
	}
	origins := make(map[int]float64)
	moves := []moveSpec{}
	for i, a := range d.Axes {
		target := s.Core.Axes[i].LogicalTarget
		if !s.Core.Axes[i].LogicalValid || !finite(target) || !finite(a.MinPosition) || !finite(a.MaxPosition) ||
			target < a.MinPosition || target > a.MaxPosition || !finite(a.MaxVelocity) || a.MaxVelocity <= 0 ||
			!finite(a.CompletionTolerance) || a.CompletionTolerance <= 0 {
			return nil, fmt.Errorf("axis %d has invalid program target or limits", i)
		}
		origins[i] = target
		if o.mask&(1<<i) != 0 {
			moves = append(moves, moveSpec{axis: i, delta: o.delta})
		}
	}
	d.CycleNS = uint64(programDT)
	if d.MaxTrajectoryPoints > densejointtraj.MaxTotalSamples {
		d.MaxTrajectoryPoints = densejointtraj.MaxTotalSamples
	}
	points, err := moveProfileGroup(d, o, moves, origins)
	if err != nil {
		return nil, err
	}
	samples := make([]densejointtraj.Sample, len(points))
	for i, p := range points {
		samples[i].TimeS = p.Time.Seconds()
		for axis := range d.Axes {
			samples[i].QRad[axis] = origins[axis]
			if o.mask&(1<<axis) != 0 {
				samples[i].QRad[axis] = p.Position[axis]
				if i > 0 && i < len(points)-1 {
					samples[i].QdRadS[axis] = (points[i+1].Position[axis] - points[i-1].Position[axis]) / (2 * programDT.Seconds())
				}
			}
		}
	}
	limits := make([]float64, densejointtraj.AxisCount)
	for axis, a := range d.Axes {
		frame := densejointtraj.CommandFrame{CountsPerUnit: a.CountsPerUnit, Sign: a.Sign,
			HomeOffset: s.Core.Axes[axis].HomeOffset}
		var peak, tangent float64
		for i := 1; i < len(samples); i++ {
			first, second := samples[i-1], samples[i]
			bounds, err := densejointtraj.BoundHermiteSegment(frame,
				densejointtraj.HermiteKnot{TimeNS: int64(math.Round(first.TimeS * 1e9)), Position: first.QRad[axis], Velocity: first.QdRadS[axis]},
				densejointtraj.HermiteKnot{TimeNS: int64(math.Round(second.TimeS * 1e9)), Position: second.QRad[axis], Velocity: second.QdRadS[axis]})
			if err != nil {
				return nil, fmt.Errorf("axis %d sample %d: %w", axis, i, err)
			}
			peak = math.Max(peak, bounds.PeakVelocity)
			tangent = math.Max(tangent, math.Max(math.Abs(first.QdRadS[axis]), math.Abs(second.QdRadS[axis])))
		}
		cap := a.MaxVelocity * o.velocityFraction
		// Plan at 0.75*cap: nine-motor bench, 2026-09-14, bd-align-all-3.jsonl
		// measured 12.7% tracking overshoot (0.2029/0.18-1); 0.75*1.127
		// is 0.84525 of cap. Hermite peaks can exceed the planned reserve,
		// so declare their admission bound and still enforce the operator cap.
		// Stationary axes retain a positive reserve as the dense format requires.
		limits[axis] = cap * moveVelocityReserve
		if peak > 0 {
			limits[axis] = math.Max(tangent, peak/a.CountsPerUnit)
			// Round outward if conversion to rad/s would round below the guarded
			// count-frame peak when admission multiplies back by CountsPerUnit.
			if limits[axis]*a.CountsPerUnit < peak {
				limits[axis] = math.Nextafter(limits[axis], math.Inf(1))
			}
		}
		if !finite(limits[axis]) || limits[axis] <= 0 || limits[axis] > cap || peak > limits[axis]*a.CountsPerUnit {
			return nil, fmt.Errorf("axis %d Hermite velocity bound %g exceeds program cap %g", axis, limits[axis], cap)
		}
	}
	id, err := control.NewRequestID()
	if err != nil {
		return nil, err
	}
	segments := []densejointtraj.Segment{{Kind: "freespace", SourceID: "benchdrive-move", Samples: samples}}
	digest := densejointtraj.TrajectoryDigest(programAxisMask, programDT.Seconds(), limits, segments)
	duration := points[len(points)-1].Time.Seconds()
	return &densejointtraj.Plan{Header: densejointtraj.Header{
		Schema: densejointtraj.Schema, PlanID: "benchdrive-" + id, ProgramID: "benchdrive-program",
		ProgramDigest: digest, TrajectoryDigest: digest, ManifestRevision: o.binding.Revision, PlanRevision: 1,
		CreatedAt:      time.Now().UTC().Format(time.RFC3339Nano),
		Axes:           densejointtraj.HeaderAxes{AxisCount: densejointtraj.AxisCount, AxisMask: programAxisMask, PositionUnit: "rad", VelocityUnit: "rad_s"},
		Sampling:       densejointtraj.Sampling{DtS: programDT.Seconds(), TotalSampleCount: len(samples), TotalDurationS: duration},
		Limits:         densejointtraj.Limits{MaxAbsQdRadS: limits},
		Segments:       []densejointtraj.SegmentMeta{{Index: 0, Kind: segments[0].Kind, SourceID: segments[0].SourceID, SampleCount: len(samples), DurationS: duration}},
		SampleEncoding: densejointtraj.EncodingMeta{Encoding: densejointtraj.SampleEncoding, RecordBytes: densejointtraj.RecordBytes},
	}, Segments: segments}, nil
}

// program uploads once under continued renewal and starts the exact returned
// identity. Only correlated Done, a terminal handle, measured endpoints and
// position-derived velocity bounds establish completion; run owns Stop/Release.
func (r *runner) program(ctx context.Context) error {
	if err := r.ready(r.last, true); err != nil {
		return err
	}
	plan, err := programProfile(r.description, r.o, r.last)
	if err != nil {
		return err
	}
	blob, err := densejointtraj.Encode(plan.Header, plan.Segments)
	if err != nil {
		return err
	}
	plan, err = densejointtraj.Decode(blob)
	if err != nil {
		return err
	}
	samples := plan.Segments[0].Samples
	if err := r.e.write(record{Kind: "program", Operation: "program_source", Response: map[string]any{
		"header": plan.Header, "moving_axis_mask": r.o.mask, "enabled_axis_mask": programAxisMask,
		"first_sample": samples[0], "last_sample": samples[len(samples)-1],
	}}); err != nil {
		return err
	}
	uploadCtx, cancel := timeout(ctx, r.o.deadlineNS)
	start, fence := ipcclient.HostMonotonicNS(), r.c.Fence()
	prepared, err := r.c.PrepareProgram(uploadCtx, blob)
	cancel()
	if err = r.e.receipt("prepare_program", start, map[string]any{
		"session": fence.Session, "generation": fence.Generation, "bytes": len(blob), "blob_sha256": fmt.Sprintf("sha256:%x", sha256.Sum256(blob)),
	}, prepared, err); err != nil {
		return err
	}
	h := plan.Header
	want := control.Identity{PlanID: h.PlanID, ProgramID: h.ProgramID, ProgramDigest: h.ProgramDigest,
		TrajectoryDigest: h.TrajectoryDigest, SourceDigest: h.TrajectoryDigest, NormalisedDigest: h.TrajectoryDigest,
		ManifestRevision: h.ManifestRevision, PlanRevision: h.PlanRevision}
	if err := r.check("program_prepared", prepared.Identity == want && prepared.AxisMask == programAxisMask &&
		prepared.Samples == len(samples) && prepared.NormalisedSamples == len(samples) && prepared.Segments == 1 && !prepared.RequiresProcessIO); err != nil {
		return err
	}
	// Refresh feedback before Start so measured travel uses actual positions;
	// continuity still uses the earlier published held targets in the payload.
	before, err := r.status(ctx)
	if err != nil {
		return err
	}
	if err = r.ready(before, true); err != nil {
		return err
	}
	for i, a := range before.Core.Axes {
		if !finite(a.LogicalPosition) || a.TimeNS == 0 {
			return errors.New("invalid program feedback before Start")
		}
		if err := r.movePosition("program_before", i, a); err != nil {
			return err
		}
	}
	startCtx, cancel := timeout(ctx, r.o.deadlineNS)
	start, fence = ipcclient.HostMonotonicNS(), r.c.Fence()
	response, err := r.c.StartProgram(startCtx, prepared.Identity)
	cancel()
	if err = r.e.receipt("start_program", start, map[string]any{
		"session": fence.Session, "generation": fence.Generation, "identity": prepared.Identity,
	}, response, err); err != nil {
		return err
	}
	if response.Sequence == 0 || response.Operation != "start_program" || response.Schema != "rosie.rt-control.response.v1" {
		return control.ErrProtocol
	}
	velocities := make([]axisVelocity, len(before.Core.Axes))
	peaks := make([]*velocityMeasurement, len(before.Core.Axes))
	for i, a := range before.Core.Axes {
		peak := 0.0
		if r.o.mask&(1<<i) != 0 {
			// Header limits contain the count-frame Hermite peak, including
			// interpolation between knots, rather than only sampled tangents.
			peak = plan.Header.Limits.MaxAbsQdRadS[i]
		}
		peaks[i] = r.beginVelocity("program", i, peak)
		if _, _, err := velocities[i].observe(a); err != nil {
			return err
		}
	}
	_, err = r.observe(ctx, func(s control.Status) (bool, error) {
		if err := r.pollCollision(ctx, "program", programAxisMask); err != nil {
			return false, err
		}
		if err := r.ready(s, true); err != nil {
			return false, err
		}
		if s.Execution.State == "faulted" || s.Execution.State == "cancelled" {
			return false, fmt.Errorf("program execution %s: %s", s.Execution.State, s.Execution.Error)
		}
		settled := true
		for i, a := range s.Core.Axes {
			axis := r.description.Axes[i]
			velocity, fresh, err := velocities[i].observe(a)
			if err != nil {
				return false, err
			}
			if fresh {
				if err := r.observeVelocity(peaks[i], velocity); err != nil {
					return false, err
				}
				if err := r.movePosition("program_sample", i, a); err != nil {
					return false, err
				}
			}
			if math.Abs(a.LogicalPosition-samples[len(samples)-1].QRad[i]) > axis.CompletionTolerance {
				settled = false
			}
		}
		state, sequence := handleState(s, s.Motion.TrajectoryID)
		terminal := s.Motion.TrajectoryID != 0 && (state == "consumed" || state == "retired") && sequence == response.Sequence
		complete := s.Motion.CommandSequence == response.Sequence && s.Motion.Done && s.Motion.State == 4 &&
			s.Execution.State == "completed" && s.Execution.NativeSequence == response.Sequence && s.Execution.Identity == prepared.Identity
		return settled && terminal && complete, nil
	})
	if err != nil {
		return err
	}
	for i, a := range r.last.Core.Axes {
		delta := 0.0
		if r.o.mask&(1<<i) != 0 {
			delta = r.o.delta
		}
		travel := a.LogicalPosition - before.Core.Axes[i].LogicalPosition
		if err := r.e.write(record{Kind: "travel", Operation: "program_travel", Response: map[string]any{
			"axis": i, "start_logical_position": before.Core.Axes[i].LogicalPosition, "start_logical_target": samples[0].QRad[i],
			"end_logical_position": a.LogicalPosition, "end_logical_target": samples[len(samples)-1].QRad[i],
			"measured_travel_rad": travel, "delta_rad": delta, "completion_tolerance": r.description.Axes[i].CompletionTolerance,
			"velocity_bound_rad_s": r.description.Axes[i].MaxVelocity,
			"handle":               r.last.Motion.TrajectoryID, "sequence": response.Sequence, "execution_generation": r.last.Execution.Generation,
			"identity": prepared.Identity,
		}}); err != nil {
			return err
		}
		if math.Abs(travel-delta) > r.description.Axes[i].CompletionTolerance {
			return fmt.Errorf("axis %d measured program travel %g differs from delta %g", i, travel, delta)
		}
	}
	r.expectations["program_endpoint"], r.expectations["program_velocity_bound"], r.expectations["program_handle_terminal"] = true, true, true
	return r.e.write(record{Kind: "completion", Operation: "program", Response: r.last.Execution})
}
