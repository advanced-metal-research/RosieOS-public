//go:build linux

package main

import (
	"context"
	"errors"
	"fmt"
	"math"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// Unverified measurement windows: cruise/deceleration 20 ms, stable hold 100 ms.
// The 16-count standstill band includes the measured 1..3-count resting spread
// on the nine-motor bench (registers-1.log, 2026-09-13), with unverified margin.
const (
	measurementWindowNS  = uint64(20_000_000)
	standstillWindowNS   = uint64(100_000_000)
	standstillBandCounts = int64(16)
	// Bounded capture allocation: 120,000 cycles (120 s at 1 ms). Mini bench
	// 2026-10-28: a single +719 degree move at the bench's 0.5 rad/s^2 axis
	// acceleration outran the earlier 20,000 cycles and was stopped at 11.99 rad.
	measurementMaxCycles = 120_000
)

type cycleMeasurement struct {
	r                                                 *runner
	moves                                             []moveSpec
	header                                            ipcclient.TelemetryBatchHeaderV1
	cursor                                            uint64
	samples                                           []ipcclient.CycleCaptureRecordV2
	cruise                                            map[int]float64
	startNS, haltStartNS, haltReceiptNS, haltSequence uint64
	fence                                             control.Fence
}

type axisMeasurement struct {
	PeakTorqueRaw            uint32  `json:"peak_torque_raw"`
	Axis                     int     `json:"axis"`
	Samples                  int     `json:"cycles"`
	PeakLag                  int64   `json:"peak_lag_counts"`
	CruiseLag                float64 `json:"lag_at_cruise_counts"`
	CruiseSamples            int     `json:"cruise_samples"`
	Overshoot                int64   `json:"catch_up_overshoot_counts"`
	SettlingNS               uint64  `json:"settling_time_ns"`
	StandstillNS             uint64  `json:"feedback_standstill_host_ns"`
	ZeroNS                   uint64  `json:"zero_commanded_velocity_host_ns"`
	ReceiptToZeroNS          int64   `json:"halt_receipt_to_zero_commanded_velocity_ns"`
	ReceiptToStandstillNS    int64   `json:"halt_receipt_to_feedback_standstill_ns"`
	RequestToZeroNS          int64   `json:"halt_request_to_zero_commanded_velocity_ns"`
	StoppingDistance         int64   `json:"stopping_distance_counts"`
	NetStoppingDistance      int64   `json:"net_stopping_displacement_counts"`
	DistanceReferenceNS      uint64  `json:"stopping_distance_reference_sample_host_ns"`
	ReceiptAlignmentNS       uint64  `json:"stopping_distance_receipt_alignment_ns"`
	PeakDeceleration         float64 `json:"peak_commanded_deceleration_counts_per_s2"`
	PeakFeedbackDeceleration float64 `json:"peak_feedback_deceleration_counts_per_s2"`
	AccelerationBound        float64 `json:"configured_acceleration_bound_counts_per_s2"`
	QuantizationBound        float64 `json:"deceleration_quantization_bound_counts_per_s2"`
	HeldTarget               int32   `json:"held_target_counts"`
	LastCommandedTarget      int32   `json:"last_commanded_target_counts"`
}

// measurementRefusal retains local qualification failures as typed refusals,
// preventing missing data or unreachable cruise from becoming success.
func (r *runner) measurementRefusal(reason string) error {
	err := &control.Rejected{Reason: reason}
	return r.e.receipt(r.o.phase+"_measurement", 0, nil, nil, err)
}

// beginMeasurement drains retained history before Start, then owns every cycle
// until stable hold. Startup history loss is explicit; subsequent loss is fatal.
func (r *runner) beginMeasurement(ctx context.Context, moves []moveSpec, points []control.Point) (*cycleMeasurement, error) {
	m := &cycleMeasurement{r: r, moves: moves, cruise: map[int]float64{}, fence: r.c.Fence()}
	if r.description.CycleNS != 1_000_000 {
		return nil, r.measurementRefusal("measurement_requires_1khz")
	}
	for _, move := range moves {
		a := r.description.Axes[move.axis]
		if !finite(a.CountsPerUnit) || a.CountsPerUnit <= 0 || (a.Sign != -1 && a.Sign != 1) {
			return nil, r.measurementRefusal("measurement_unsupported_count_frame")
		}
		for i := 1; i < len(points); i++ {
			v := math.Abs(points[i].Position[move.axis]-points[i-1].Position[move.axis]) * 1e9 / float64(r.description.CycleNS)
			m.cruise[move.axis] = math.Max(m.cruise[move.axis], v*a.CountsPerUnit)
		}
	}
	ctx, cancel := timeout(ctx, r.o.deadlineNS)
	defer cancel()
	cutoff := ipcclient.HostMonotonicNS()
	for {
		b, err := m.batch(ctx)
		if err != nil {
			return nil, err
		}
		if len(b.Records) > 0 && b.Records[len(b.Records)-1].TNs >= cutoff {
			last := b.Records[len(b.Records)-1]
			if err = m.append(last); err != nil {
				return nil, err
			}
			return m, nil
		}
		select {
		case <-ctx.Done():
			return nil, ctx.Err()
		case <-time.After(time.Millisecond):
		}
	}
}

func (m *cycleMeasurement) batch(ctx context.Context) (control.TelemetryBatch, error) {
	ctx, cancel := timeout(ctx, m.r.o.deadlineNS)
	defer cancel()
	start := ipcclient.HostMonotonicNS()
	b, err := m.r.c.TelemetryBatches(ctx, m.cursor)
	if err = m.r.e.receipt("telemetry", start, map[string]any{"after": m.cursor}, map[string]any{"header": b.Header, "decoded_records": len(b.Records)}, err); err != nil {
		return b, err
	}
	h := b.Header
	if h.CyclePeriodNs != m.r.description.CycleNS || int(h.AxisCount) != len(m.r.description.Axes) ||
		string(h.MachineSha256[:]) != m.r.description.MachineSHA256 || string(h.DeploymentSha256[:]) != m.r.description.DeploymentSHA256 ||
		string(h.AdapterIncarnation[:]) != m.r.last.AdapterIncarnation || string(h.DaemonIncarnation[:]) != m.r.last.DaemonIncarnation {
		return b, m.r.measurementRefusal("measurement_identity_changed")
	}
	if len(m.samples) > 0 && h.Dropped != 0 {
		return b, m.r.measurementRefusal("measurement_telemetry_gap")
	}
	m.header, m.cursor = h, h.LastSequence
	return b, nil
}

func (m *cycleMeasurement) append(s ipcclient.CycleCaptureRecordV2) error {
	if len(m.samples) >= measurementMaxCycles {
		return m.r.measurementRefusal("measurement_capture_limit")
	}
	if len(m.samples) > 0 {
		p := m.samples[len(m.samples)-1]
		if s.Seq != p.Seq+1 || s.Cycle != p.Cycle+1 || s.TNs <= p.TNs {
			return m.r.measurementRefusal("measurement_telemetry_gap")
		}
	}
	for _, move := range m.moves {
		a := s.Ax[move.axis]
		if a.VelocityActualValid != 1 || a.FollowingErrorValid != 1 || a.TorqueValid != 1 {
			return m.r.measurementRefusal("measurement_drive_fields_unavailable")
		}
		if a.Statusword&operationMask != operationWord || a.ErrorCode != 0 {
			return m.r.measurementRefusal("measurement_drive_not_enabled")
		}
		if err := m.r.e.write(record{Kind: "position", Operation: m.r.o.phase + "_cycle", Response: map[string]any{
			"axis": move.axis, "sequence": s.Seq, "cycle": s.Cycle, "time_ns": s.TNs,
			"commanded_target_counts": a.Target, "feedback_position_counts": a.Position,
			"torque_raw": a.TorqueRaw, "torque_valid": a.TorqueValid != 0,
			"target_velocity_counts_per_s": a.TargetVelocityCountsPerS, "target_velocity_valid": a.TargetVelocityValid != 0,
			"feedback_counts":        m.feedback(move.axis, a),
			"following_error_counts": a.FollowingErrorCounts, "actual_velocity_counts_per_s": a.VelocityActualCountsPerS,
			"following_error_valid": true, "actual_velocity_valid": true}}); err != nil {
			return err
		}
	}
	m.samples = append(m.samples, s)
	return nil
}

func (m *cycleMeasurement) feedback(axis int, a ipcclient.CycleCaptureAxisV2) int64 {
	return int64(a.Position)
}

func (m *cycleMeasurement) poll(ctx context.Context) error {
	b, err := m.batch(ctx)
	if err != nil {
		return err
	}
	for _, s := range b.Records {
		if err = m.append(s); err != nil {
			return err
		}
	}
	return nil
}

// windowStart uses actual monotonic timestamps. Count quantization is stated
// separately, preventing a rounded position derivative claiming exact velocity.
func (m *cycleMeasurement) windowStart(end int, ns uint64) int {
	for i := end - 1; i >= 0; i-- {
		if m.samples[end].TNs-m.samples[i].TNs >= ns {
			return i
		}
	}
	return -1
}
func (m *cycleMeasurement) velocity(axis, start, end int) float64 {
	return float64(int64(m.samples[end].Ax[axis].Target)-int64(m.samples[start].Ax[axis].Target)) * 1e9 / float64(m.samples[end].TNs-m.samples[start].TNs)
}
func (m *cycleMeasurement) atCruise(axis, end int) bool {
	start := m.windowStart(end, measurementWindowNS)
	if start < 0 || m.samples[start].TNs < m.startNS {
		return false
	}
	previous := m.windowStart(start, measurementWindowNS)
	if previous < 0 {
		return false
	}
	// Two endpoint roundings contribute at most two counts per window.
	tolerance := 2e9 / float64(measurementWindowNS)
	return math.Abs(math.Abs(m.velocity(axis, start, end))-m.cruise[axis]) <= tolerance &&
		math.Abs(math.Abs(m.velocity(axis, previous, start))-m.cruise[axis]) <= tolerance
}

// stable requires an unchanged wire target and feedback confined to a 16-count
// band near that target for 100 ms of consecutive samples, never elapsed sleep.
func (m *cycleMeasurement) stable(axis int) (zero, standstill int, ok bool) {
	end := len(m.samples) - 1
	if end < 1 {
		return
	}
	start := m.windowStart(end, standstillWindowNS)
	if start < 0 {
		return
	}
	target := m.samples[end].Ax[axis].Target
	lo, hi := m.feedback(axis, m.samples[end].Ax[axis]), m.feedback(axis, m.samples[end].Ax[axis])
	for i := end; i >= start; i-- {
		a := m.samples[i].Ax[axis]
		position := m.feedback(axis, a)
		lo, hi = min(lo, position), max(hi, position)
		if a.Target != target || hi-lo > standstillBandCounts || absCounts(position-int64(target)) > standstillBandCounts {
			return
		}
	}
	zero = start
	for zero > 0 && m.samples[zero-1].Ax[axis].Target == target {
		zero--
	}
	standstill = start
	for standstill > zero {
		a := m.feedback(axis, m.samples[standstill-1].Ax[axis])
		if max(hi, a)-min(lo, a) > standstillBandCounts || absCounts(a-int64(target)) > standstillBandCounts {
			break
		}
		lo, hi = min(lo, a), max(hi, a)
		standstill--
	}
	return zero, standstill, true
}
func absCounts(v int64) int64 {
	if v < 0 {
		return -v
	}
	return v
}

// finish reports signed latency from the public receipt arrival, plus the
// request-to-zero upper bound. Native receipt time is not exposed by this API;
// negative latency means the count-resolved stop preceded receipt delivery.
func (m *cycleMeasurement) finish() error {
	for _, move := range m.moves {
		axis := move.axis
		zero, standstill, ok := m.stable(axis)
		if !ok {
			return m.r.measurementRefusal("measurement_not_settled")
		}
		end := len(m.samples) - 1
		last := m.samples[end].Ax[axis]
		d := m.r.description.Axes[axis]
		bound := d.JogAcceleration
		if d.MaxAcceleration != nil {
			bound = *d.MaxAcceleration
		}
		v := axisMeasurement{Axis: axis, Samples: len(m.samples), ZeroNS: m.samples[zero].TNs, StandstillNS: m.samples[standstill].TNs,
			HeldTarget: last.Target, LastCommandedTarget: m.samples[zero].Ax[axis].Target, AccelerationBound: bound * d.CountsPerUnit,
			QuantizationBound: 2e18 / float64(measurementWindowNS*measurementWindowNS)}
		v.SettlingNS = v.StandstillNS - v.ZeroNS
		direction := int64(d.Sign)
		if move.delta < 0 {
			direction = -direction
		}
		var lagSum int64
		receiptIndex := -1
		for i, s := range m.samples {
			if s.TNs < m.startNS {
				continue
			}
			a := s.Ax[axis]
			v.PeakTorqueRaw = max(v.PeakTorqueRaw, uint32(absCounts(int64(a.TorqueRaw))))
			lag := int64(a.Target) - m.feedback(axis, a)
			v.PeakLag = max(v.PeakLag, absCounts(lag))
			v.Overshoot = max(v.Overshoot, -direction*lag)
			if (m.haltStartNS == 0 || s.TNs < m.haltStartNS) && m.atCruise(axis, i) {
				lagSum += direction * lag
				v.CruiseSamples++
			}
			if m.haltReceiptNS != 0 && s.TNs <= m.haltReceiptNS {
				receiptIndex = i
			}
			if m.haltStartNS == 0 || s.TNs < m.haltStartNS {
				continue
			}
			j := m.windowStart(i, measurementWindowNS)
			if j < 0 {
				continue
			}
			k := m.windowStart(j, measurementWindowNS)
			if k < 0 {
				continue
			}
			dt := float64(s.TNs-m.samples[k].TNs) / 2e9
			decel := float64(direction) * (m.velocity(axis, k, j) - m.velocity(axis, j, i)) / dt
			v.PeakDeceleration = math.Max(v.PeakDeceleration, decel)
			feedback := float64(direction) * (float64(m.samples[j].Ax[axis].VelocityActualCountsPerS) - float64(a.VelocityActualCountsPerS)) * 1e9 / float64(s.TNs-m.samples[j].TNs)
			v.PeakFeedbackDeceleration = math.Max(v.PeakFeedbackDeceleration, feedback)
		}
		if v.CruiseSamples == 0 {
			return m.r.measurementRefusal("measurement_cruise_not_observed")
		}
		v.CruiseLag = float64(lagSum) / float64(v.CruiseSamples)
		if m.haltReceiptNS != 0 {
			if receiptIndex < 0 || v.ZeroNS < m.haltStartNS {
				return m.r.measurementRefusal("halt_stop_not_correlated")
			}
			v.ReceiptToZeroNS = int64(v.ZeroNS) - int64(m.haltReceiptNS)
			v.ReceiptToStandstillNS = int64(v.StandstillNS) - int64(m.haltReceiptNS)
			v.RequestToZeroNS = int64(v.ZeroNS) - int64(m.haltStartNS)
			reference := m.feedback(axis, m.samples[receiptIndex].Ax[axis])
			v.DistanceReferenceNS = m.samples[receiptIndex].TNs
			v.ReceiptAlignmentNS = m.haltReceiptNS - v.DistanceReferenceNS
			v.NetStoppingDistance = direction * (m.feedback(axis, last) - reference)
			for _, sample := range m.samples[receiptIndex:] {
				v.StoppingDistance = max(v.StoppingDistance, direction*(m.feedback(axis, sample.Ax[axis])-reference))
			}
			if v.PeakDeceleration > v.AccelerationBound+v.QuantizationBound {
				return errors.Join(m.r.e.write(record{Kind: "travel", Operation: "halt_measurement", Response: v}), m.r.measurementRefusal("halt_deceleration_bound_exceeded"))
			}
		}
		if err := m.r.e.write(record{Kind: "travel", Operation: m.r.o.phase + "_measurement", Response: v}); err != nil {
			return err
		}
		m.r.measurements = append(m.r.measurements, v)
		fmt.Fprintf(m.r.out, "axis %d: peak lag %d counts, cruise lag %.3f counts, catch-up overshoot %d counts, settling %d ns\n", axis, v.PeakLag, v.CruiseLag, v.Overshoot, v.SettlingNS)
		if m.haltReceiptNS != 0 {
			fmt.Fprintf(m.r.out, "halt: receipt to zero %d ns, receipt to feedback standstill %d ns, stopping distance %d counts, peak command deceleration %.3f counts/s^2 (configured %.3f, quantization %.3f)\n", v.ReceiptToZeroNS, v.ReceiptToStandstillNS, v.StoppingDistance, v.PeakDeceleration, v.AccelerationBound, v.QuantizationBound)
		}
	}
	return m.r.check(m.r.o.phase+"_measurement_complete", true)
}
