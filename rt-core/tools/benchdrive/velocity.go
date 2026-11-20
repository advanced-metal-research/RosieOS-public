//go:build linux

package main

import (
	"fmt"
	"math"
)

type velocityMeasurement struct {
	Phase             string   `json:"phase"`
	Axis              int      `json:"axis"`
	PlannedPeak       float64  `json:"peak_planned_velocity_rad_s"`
	ObservedPeak      float64  `json:"peak_observed_velocity_rad_s"`
	Limit             float64  `json:"axis_max_velocity_rad_s"`
	OvershootFraction *float64 `json:"overshoot_fraction"`
}

// beginVelocity retains each execution and axis separately so a later session
// move cannot erase an earlier peak. A stationary plan has no defined fraction.
func (r *runner) beginVelocity(phase string, axis int, peak float64) *velocityMeasurement {
	m := &velocityMeasurement{Phase: phase, Axis: axis, PlannedPeak: peak, Limit: r.description.Axes[axis].MaxVelocity}
	if peak > 0 {
		m.OvershootFraction = new(float64)
	}
	r.velocityMeasurements = append(r.velocityMeasurements, m)
	return m
}

// observeVelocity records unsmoothed feedback peaks before enforcing the axis
// limit. Catch-up above plan is a measurement, preventing false run failures.
// Overshoot is max(0, observed/planned-1); stationary plans report null.
func (r *runner) observeVelocity(m *velocityMeasurement, velocity float64) error {
	if !finite(velocity) || velocity < 0 || !finite(m.Limit) || m.Limit <= 0 {
		return fmt.Errorf("axis %d invalid observed velocity or axis limit", m.Axis)
	}
	m.ObservedPeak = math.Max(m.ObservedPeak, velocity)
	if m.OvershootFraction != nil {
		*m.OvershootFraction = math.Max(0, m.ObservedPeak/m.PlannedPeak-1)
	}
	if err := r.e.write(record{Kind: "velocity", Operation: m.Phase + "_velocity", Response: m}); err != nil {
		return err
	}
	if velocity > m.Limit {
		return fmt.Errorf("axis %d observed velocity %g exceeds axis max_velocity %g", m.Axis, velocity, m.Limit)
	}
	return nil
}
