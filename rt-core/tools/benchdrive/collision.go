//go:build linux

package main

import (
	"context"
	"fmt"
	"math"

	"rosieos/rt-core/ipcclient"
)

// pollCollision reads every retained cycle with the status observation's identity.
// Loss after the initial cursor, invalid torque or changed authority fails closed;
// status cadence cannot hide a torque sample or invent one for an unmapped PDO.
func (r *runner) pollCollision(ctx context.Context, phase string, mask uint32) error {
	if !r.o.tuneCollision {
		return nil
	}
	initial := r.collisionHeader == nil
	var observed uint32
	for {
		b, err := r.c.TelemetryBatches(ctx, r.collisionCursor)
		if err != nil {
			return err
		}
		h := b.Header
		if string(h.DaemonIncarnation[:]) != r.last.DaemonIncarnation || string(h.AdapterIncarnation[:]) != r.last.AdapterIncarnation ||
			string(h.MachineSha256[:]) != r.description.MachineSHA256 || string(h.DeploymentSha256[:]) != r.description.DeploymentSHA256 ||
			h.CyclePeriodNs != r.description.CycleNS || int(h.AxisCount) != len(r.description.Axes) ||
			(r.collisionHeader != nil && (h.Dropped != 0 || h.DaemonIncarnation != r.collisionHeader.DaemonIncarnation)) {
			return fmt.Errorf("collision tuning telemetry identity changed or records lost")
		}
		for _, sample := range b.Records {
			if sample.AppGrantGeneration != r.c.Fence().Generation || sample.GrantActive == 0 {
				continue
			}
			for axis := uint32(0); axis < sample.Axes; axis++ {
				if mask&(1<<axis) == 0 || sample.Ax[axis].CollisionSamples == 0 {
					continue
				}
				a := sample.Ax[axis]
				observed |= 1 << axis
				if a.TorqueValid != 1 || a.PdoFresh != 1 {
					return fmt.Errorf("axis %d: collision tuning capture torque unavailable", axis)
				}
				if err := r.e.write(record{Kind: "collision_cycle", Operation: phase, Response: map[string]any{
					"axis": axis, "sequence": sample.Seq, "cycle": sample.Cycle, "time_ns": sample.TNs,
					"torque_raw": a.TorqueRaw, "torque_valid": true, "following_error_counts": a.FollowingErrorCounts,
					"following_error_valid": a.FollowingErrorValid != 0, "collision_torque_cycles": a.CollisionTorqueCycles,
					"collision_following_error_cycles": a.CollisionFollowingErrorCycles,
					"safety_fault_mask":                sample.SafetyFaultMask, "execution_fault_reasons": sample.ExecutionFaultReasons}}); err != nil {
					return err
				}
				if err := r.observeCollision(phase, int(axis), ipcclient.AxisStatus{PDOFresh: 1, CollisionWatchdog: ipcclient.CollisionWatchdogStatus{
					Samples: a.CollisionSamples, PeakTorqueRaw: max(a.CollisionPeakTorqueRaw, uint32(absCounts(int64(a.TorqueRaw)))),
					PeakFollowingErrorCounts: a.CollisionPeakFollowingErrorCounts}}); err != nil {
					return err
				}
			}
		}
		r.collisionCursor, r.collisionHeader = h.LastSequence, &h
		if len(b.Records) == 0 || b.Records[len(b.Records)-1].TNs >= r.last.Core.TimeNS {
			if initial && observed&mask != mask {
				return fmt.Errorf("collision tuning requires fresh captured torque samples for every selected axis")
			}
			return nil
		}
	}
}

// Unverified tuning margin: 1.5 times the measured peak; robot loads require remeasurement.
const defaultCollisionMargin = 1.5

type collisionTuning struct {
	Phase           string  `json:"phase"`
	Axis            int     `json:"axis"`
	Samples         uint64  `json:"cycles_since_arm"`
	PeakTorque      uint32  `json:"peak_torque_raw"`
	PeakError       uint64  `json:"peak_following_error_counts"`
	Margin          float64 `json:"margin"`
	SuggestedTorque uint64  `json:"torque_abs_max_raw"`
	SuggestedError  uint64  `json:"following_error_counts_max"`
	Usable          bool    `json:"within_compiler_ranges"`
	Unverified      string  `json:"_unverified"`
	Source          string  `json:"_source"`
}

// observeCollision aggregates capture-owned peaks, including enabled hold before
// motion. Missing native samples fail
// closed; a suggested value beyond compiler bounds is reported, never clamped.
func (r *runner) observeCollision(phase string, axis int, a ipcclient.AxisStatus) error {
	if !r.o.tuneCollision {
		return nil
	}
	w := a.CollisionWatchdog
	if a.PDOFresh != 1 || w.Samples == 0 {
		return fmt.Errorf("axis %d: collision tuning requires fresh native peak samples and torque PDO", axis)
	}
	var row *collisionTuning
	for _, existing := range r.collisionTuning {
		if existing.Phase == phase && existing.Axis == axis {
			row = existing
			break
		}
	}
	if row == nil {
		row = &collisionTuning{Phase: phase, Axis: axis, Margin: r.o.collisionMargin,
			Unverified: "unverified: repeat on the loaded robot; sustained_cycles requires separate qualification",
			Source:     "V2 capture per-cycle torque and absolute peaks since arm, including enabled hold; drive 0x60F4 preferred, otherwise prior wire target minus feedback"}
		r.collisionTuning = append(r.collisionTuning, row)
	}
	row.Samples = max(row.Samples, w.Samples)
	row.PeakTorque = max(row.PeakTorque, w.PeakTorqueRaw)
	row.PeakError = max(row.PeakError, w.PeakFollowingErrorCounts)
	// Native peak error may span the signed 64-bit coordinate frame. Refuse an
	// unrepresentable suggestion before converting a floating-point product.
	errorSuggestion := math.Ceil(float64(row.PeakError) * row.Margin)
	if !finite(errorSuggestion) || errorSuggestion >= float64(math.MaxUint64) {
		return fmt.Errorf("axis %d: collision threshold suggestion overflows", axis)
	}
	row.SuggestedTorque = max(uint64(1), uint64(math.Ceil(float64(row.PeakTorque)*row.Margin)))
	row.SuggestedError = max(uint64(1), uint64(errorSuggestion))
	row.Usable = row.SuggestedTorque <= 1000 && row.SuggestedError <= math.MaxInt32
	return r.e.write(record{Kind: "collision_tuning", Operation: phase, Response: row})
}
