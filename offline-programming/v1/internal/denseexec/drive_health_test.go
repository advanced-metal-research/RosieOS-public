//go:build linux

package denseexec

import (
	"errors"
	"math"
	"testing"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// Drive health is rt-core's per-axis load and monitor in display units: torque
// per-mille to percent, following-error counts to degrees (rotary) or mm
// (linear) through the description's counts per unit, and the monitor
// objects passed through. An empty window is null, never zero.
func TestDriveHealthFromStatusInDisplayUnits(t *testing.T) {
	var description control.Description
	description.Axes = []ipcclient.AxisDescription{
		{ID: "J1", PositionUnit: "rad", CountsPerUnit: 1000 / (math.Pi / 180)}, // 1000 counts a degree
		{ID: "J2", PositionUnit: "m", CountsPerUnit: 1e6},                      // 1000 counts a mm
	}
	temperature, age := 34.5, uint64(812)
	var status control.Status
	status.TimeNS = 42
	status.Core.Axes = []ipcclient.AxisStatus{
		{Readiness: "ready", Load: ipcclient.AxisLoad{WindowMS: 1000, TorqueSamples: 1000, TorqueRMSRaw: 123.4,
			TorquePeakRaw: 456, FollowingErrorSamples: 1000, FollowingErrorPeakCounts: 250},
			Monitor: []ipcclient.AxisMonitor{{Key: "heatsink_temperature", Unit: "degC", Value: &temperature, AgeMS: &age}}},
		{Readiness: "drive_alarm", DriveAlarm: "Er41.1", DriveAlarmText: "Motor over-temperature",
			Load:    ipcclient.AxisLoad{WindowMS: 1000, FollowingErrorSamples: 10, FollowingErrorPeakCounts: 2500},
			Monitor: []ipcclient.AxisMonitor{{Key: "heatsink_temperature", Unit: "degC"}}},
		{Readiness: "ready"}, // an axis the description does not name
	}
	health, err := DriveHealthFrom(&StatusBody{Target: &TargetIdentity{Label: "Rosie 1430 rosie_1420_v1"},
		RTCore: &RTCoreObservation{Description: description, Status: &status}})
	if err != nil {
		t.Fatal(err)
	}
	if health.TimeNS != 42 || health.Cell != "Rosie 1430 rosie_1420_v1" || len(health.Joints) != 3 {
		t.Fatalf("%+v", health)
	}
	j1, j2, j3 := health.Joints[0], health.Joints[1], health.Joints[2]
	near := func(p *float64, want float64) bool { return p != nil && math.Abs(*p-want) < 1e-9 }
	if j1.Name != "J1" || !near(j1.TorqueRMSPercent, 12.34) || !near(j1.TorquePeakPercent, 45.6) ||
		j1.FollowingErrorUnit != "deg" || !near(j1.FollowingErrorPeak, 0.25) || j1.WindowMS != 1000 {
		t.Fatalf("J1: %+v", j1)
	}
	if len(j1.Monitor) != 1 || j1.Monitor[0].Key != "heatsink_temperature" || !near(j1.Monitor[0].Value, 34.5) || *j1.Monitor[0].AgeMS != 812 {
		t.Fatalf("J1 monitor: %+v", j1.Monitor)
	}
	if j2.Name != "J2" || j2.TorqueRMSPercent != nil || j2.TorquePeakPercent != nil || j2.FollowingErrorUnit != "mm" ||
		!near(j2.FollowingErrorPeak, 2.5) || j2.DriveAlarm != "Er41.1" || j2.Monitor[0].Value != nil {
		t.Fatalf("J2: %+v", j2)
	}
	if j3.Name != "axis 2" || j3.FollowingErrorPeak != nil || j3.FollowingErrorUnit != "counts" || j3.Monitor == nil {
		t.Fatalf("J3: %+v", j3)
	}
	for _, body := range []*StatusBody{nil, {}, {RTCore: &RTCoreObservation{}}} {
		if _, err := DriveHealthFrom(body); !errors.Is(err, ErrDriveHealthUnavailable) {
			t.Fatalf("no status: %v", err)
		}
	}
}
