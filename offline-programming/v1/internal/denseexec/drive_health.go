//go:build linux

package denseexec

import (
	"math"
	"strconv"
)

// DriveHealthFrom converts one status body's rt-core observation into display
// units: per native axis, named by the description's axis id.
func DriveHealthFrom(body *StatusBody) (*DriveHealth, error) {
	if body == nil || body.RTCore == nil || body.RTCore.Status == nil {
		return nil, ErrDriveHealthUnavailable
	}
	described := body.RTCore.Description.Axes
	status := body.RTCore.Status.Core.Axes
	out := &DriveHealth{TimeNS: body.RTCore.Status.TimeNS, Joints: make([]JointHealth, 0, len(status))}
	if body.Target != nil {
		out.Cell = body.Target.Label
	}
	for i, axis := range status {
		joint := JointHealth{Readiness: axis.Readiness, DriveAlarm: axis.DriveAlarm, DriveAlarmText: axis.DriveAlarmText,
			WindowMS: axis.Load.WindowMS, Monitor: make([]DriveReading, 0, len(axis.Monitor))}
		joint.Name, joint.FollowingErrorUnit = "axis "+strconv.Itoa(i), "counts"
		countsPerUnit, unit := 0.0, ""
		if i < len(described) {
			joint.Name, countsPerUnit, unit = described[i].ID, described[i].CountsPerUnit, described[i].PositionUnit
		}
		if axis.Load.TorqueSamples > 0 {
			rms, peak := axis.Load.TorqueRMSRaw/10, float64(axis.Load.TorquePeakRaw)/10
			joint.TorqueRMSPercent, joint.TorquePeakPercent = &rms, &peak
		}
		if axis.Load.FollowingErrorSamples > 0 {
			peak := float64(axis.Load.FollowingErrorPeakCounts)
			switch {
			case unit == "rad" && countsPerUnit > 0:
				peak, joint.FollowingErrorUnit = peak/countsPerUnit*180/math.Pi, "deg"
			case unit == "m" && countsPerUnit > 0:
				peak, joint.FollowingErrorUnit = peak/countsPerUnit*1000, "mm"
			}
			joint.FollowingErrorPeak = &peak
		}
		for _, m := range axis.Monitor {
			joint.Monitor = append(joint.Monitor, DriveReading{Key: m.Key, Unit: m.Unit, Value: m.Value, AgeMS: m.AgeMS})
		}
		out.Joints = append(out.Joints, joint)
	}
	return out, nil
}
