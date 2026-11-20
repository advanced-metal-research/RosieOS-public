package denseexec

import "errors"

// Drive health for the operator's drive page (docs/decisions/0001): each
// joint's last second of torque and following error and the drive profile's
// monitor objects (the A6-EC heatsink temperature), in display units, read
// from the connected cell's rt-core status. Diagnostics only: nothing here
// is a motion or safety input.

// ErrDriveHealthUnavailable: the selected cell has no connected rt-core
// status to read (no cell, a stopped controller, or another backend).
var ErrDriveHealthUnavailable = errors.New("the cell's controller status is not available")

// DriveHealth is GET .../dense-execution/drive-health.
type DriveHealth struct {
	// Cell is the selected cell's label, so the page names what it shows.
	Cell string `json:"cell,omitempty"`
	// TimeNS is the rt-core daemon's status time, for telling a fresh
	// answer from a repeated one.
	TimeNS uint64        `json:"time_ns"`
	Joints []JointHealth `json:"joints"`
}

// JointHealth is one joint in display units. A pointer is null when the
// window held no valid sample, so the page shows a dash, never a zero.
type JointHealth struct {
	Name           string `json:"name"`
	Readiness      string `json:"readiness"`
	DriveAlarm     string `json:"drive_alarm,omitempty"`
	DriveAlarmText string `json:"drive_alarm_text,omitempty"`
	WindowMS       uint64 `json:"window_ms"`
	// Percent of the drive's rated torque (0x6077 is per-mille).
	TorqueRMSPercent  *float64 `json:"torque_rms_percent"`
	TorquePeakPercent *float64 `json:"torque_peak_percent"`
	// FollowingErrorUnit is "deg" for a rotary joint, "mm" for a linear one.
	FollowingErrorUnit string         `json:"following_error_unit"`
	FollowingErrorPeak *float64       `json:"following_error_peak"`
	Monitor            []DriveReading `json:"monitor"`
}

// DriveReading is one drive-profile monitor object, as rt-core publishes it.
type DriveReading struct {
	Key   string   `json:"key"`
	Unit  string   `json:"unit"`
	Value *float64 `json:"value"`
	AgeMS *uint64  `json:"age_ms"`
}
