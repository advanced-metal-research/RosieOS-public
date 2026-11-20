package rtctl

import (
	"bytes"
	"crypto/sha256"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"rosieos/rt-core/ipcclient"
)

// RunTelemetryCLI emits commissioning-compatible JSONL from validated raw dumps
// or publications. Formatting occurs in this reader process and cannot delay the cycle.
func RunTelemetryCLI(args []string, stdout, stderr io.Writer) int {
	if len(args) != 2 || (args[0] != "dump-to-jsonl" && args[0] != "batch-to-jsonl") {
		fmt.Fprintln(stderr, "usage: rtctl telemetry {dump-to-jsonl|batch-to-jsonl} <file>")
		return 2
	}
	type absolute struct {
		Key   string `json:"k"`
		Value int32  `json:"v"`
		Valid uint8  `json:"ok"`
	}
	type axis struct {
		CoordinateCounts                          int64      `json:"coordinate_counts"`
		CoordinateEpoch                           uint64     `json:"coordinate_epoch"`
		PdoObservedTimeNs                         uint64     `json:"pdo_observed_time_ns"`
		CollisionPeakFollowingErrorCounts         uint64     `json:"collision_peak_following_error_counts"`
		CollisionTripCount                        uint64     `json:"collision_trip_count"`
		CollisionSamples                          uint64     `json:"collision_samples"`
		CollisionLastTripPeakFollowingErrorCounts uint64     `json:"collision_last_trip_peak_following_error_counts"`
		AbsoluteSourceCounts                      int32      `json:"absolute_source_counts"`
		CalibrationValid                          uint8      `json:"calibration_valid"`
		PositionState                             uint8      `json:"position_state"`
		AuditReason                               uint16     `json:"audit_reason"`
		AuditLastVerifiedNs                       uint64     `json:"audit_last_verified_ns"`
		AuditStartedNs                            uint64     `json:"audit_started_ns"`
		AuditCompletedNs                          uint64     `json:"audit_completed_ns"`
		AuditCoherenceErrorCounts                 uint64     `json:"audit_coherence_error_counts"`
		CoordinateReason                          uint32     `json:"coordinate_reason"`
		TargetVelocityCountsPerS                  int32      `json:"target_velocity_counts_per_s"`
		CollisionArmed                            bool       `json:"collision_armed"`
		CollisionTorqueCycles                     uint32     `json:"collision_torque_cycles"`
		CollisionFollowingErrorCycles             uint32     `json:"collision_following_error_cycles"`
		CollisionPeakTorqueRaw                    uint32     `json:"collision_peak_torque_raw"`
		CollisionLastTripQuantity                 uint32     `json:"collision_last_trip_quantity"`
		CollisionTripSustainedCycles              uint32     `json:"collision_trip_sustained_cycles"`
		CollisionLastTripPeakTorqueRaw            uint32     `json:"collision_last_trip_peak_torque_raw"`
		BrakeState                                uint32     `json:"brake_state"`
		NativeHomePositionOffset                  int32      `json:"native_home_position_offset"`
		NativeHomeLastAbortCode                   uint32     `json:"native_home_last_abort_code"`
		ExtPositionErrorCounts                    int32      `json:"ext_position_error_counts"`
		ExtMultiTurnLo                            int32      `json:"ext_multi_turn_lo"`
		ExtMultiTurnHi                            int32      `json:"ext_multi_turn_hi"`
		MaxAbsVelocityActualCountsPerS            uint32     `json:"max_abs_velocity_actual_counts_per_s"`
		MaxAbsFollowingErrorCounts                uint32     `json:"max_abs_following_error_counts"`
		TorqueRaw                                 int16      `json:"torque_raw"`
		ExtBusVoltageRaw                          uint16     `json:"ext_bus_voltage_raw"`
		ExtLoadRateRaw                            uint16     `json:"ext_load_rate_raw"`
		ExtIgbtTempRaw                            int16      `json:"ext_igbt_temp_raw"`
		ExtMotorTempRaw                           int16      `json:"ext_motor_temp_raw"`
		ExtDriveNotReadyBits                      uint16     `json:"ext_drive_not_ready_bits"`
		ExtMotorNotRotatingCode                   uint16     `json:"ext_motor_not_rotating_code"`
		ExtValidMask                              uint16     `json:"ext_valid_mask"`
		ModeDisplay                               int8       `json:"mode_display"`
		Ds402State                                uint8      `json:"ds402_state"`
		TorqueValid                               bool       `json:"torque_valid"`
		CoordinateValid                           bool       `json:"coordinate_valid"`
		CoordinateSourceValid                     bool       `json:"coordinate_source_valid"`
		NativeHomeState                           uint8      `json:"native_home_state"`
		SlaveAlState                              uint8      `json:"slave_al_state"`
		SlaveOnline                               bool       `json:"slave_online"`
		SlaveOperational                          bool       `json:"slave_operational"`
		PdoFresh                                  bool       `json:"pdo_fresh"`
		TargetVelocityValid                       bool       `json:"target_velocity_valid"`
		ExtValid                                  bool       `json:"ext_valid"`
		DIBits                                    uint32     `json:"di_bits"`
		DIValid                                   bool       `json:"di_valid"`
		ExternalEnableActive                      bool       `json:"external_enable_active"`
		ExternalEnableValid                       bool       `json:"external_enable_valid"`
		VelocityActualCountsPerS                  int32      `json:"velocity_actual_counts_per_s"`
		FollowingErrorCounts                      int32      `json:"following_error_counts"`
		VelocityActualValid                       bool       `json:"velocity_actual_valid"`
		FollowingErrorValid                       bool       `json:"following_error_valid"`
		I                                         uint32     `json:"i"`
		Position                                  int32      `json:"p"`
		Target                                    int32      `json:"tp"`
		Statusword                                uint16     `json:"sw"`
		Error                                     uint16     `json:"er"`
		Manufacturer                              uint32     `json:"mfr"`
		Absolute                                  []absolute `json:"af,omitempty"`
	}
	type cycle struct {
		CycleJitterNs           int64  `json:"cycle_jitter_ns"`
		ActiveTrajId            uint64 `json:"active_traj_id"`
		ActiveCommandSeq        uint64 `json:"active_command_seq"`
		JogGeneration           uint64 `json:"jog_generation"`
		AppGrantGeneration      uint64 `json:"app_grant_generation"`
		ControlGeneration       uint64 `json:"control_generation"`
		ConfigurationEpoch      uint64 `json:"configuration_epoch"`
		HomeEpoch               uint64 `json:"home_epoch"`
		GrantDeadlineHostNs     uint64 `json:"grant_deadline_host_ns"`
		JogInputDeadlineHostNs  uint64 `json:"jog_input_deadline_host_ns"`
		JogSourceSequence       uint64 `json:"jog_source_sequence"`
		BusSubmissionReturnCode int64  `json:"bus_submission_return_code"`
		Armed                   bool   `json:"armed"`
		AxisEnableMask          uint32 `json:"axis_enable_mask"`
		ActiveMode              uint32 `json:"active_mode"`
		State                   uint32 `json:"state"`
		CurrentPointIndex       uint32 `json:"current_point_index"`
		QueueDepth              uint32 `json:"queue_depth"`
		LastEventCode           uint32 `json:"last_event_code"`
		UnderrunCount           uint32 `json:"underrun_count"`
		StaleCommandFlag        uint32 `json:"stale_command_flag"`
		MotionDone              bool   `json:"motion_done"`
		CapabilityFlags         uint32 `json:"capability_flags"`
		JogSessionOpen          bool   `json:"jog_session_open"`
		JogHasInput             bool   `json:"jog_has_input"`
		JogAxisMask             uint32 `json:"jog_axis_mask"`
		JogRamping              bool   `json:"jog_ramping"`
		GrantActive             bool   `json:"grant_active"`
		GrantAxisMask           uint32 `json:"grant_axis_mask"`
		SafetyFaultMask         uint32 `json:"safety_fault_mask"`
		ExecutionFaultReasons   uint32 `json:"execution_fault_reasons"`
		SubmittedAxisMask       uint32 `json:"submitted_axis_mask"`
		BusSubmissionOperation  uint32 `json:"bus_submission_operation"`
		PdoFreshAxisMask        uint32 `json:"pdo_fresh_axis_mask"`
		ConfigVerifiedMask      uint32 `json:"config_verified_mask"`
		HomeValidMask           uint32 `json:"home_valid_mask"`
		ServiceModeAxisMask     uint32 `json:"service_mode_axis_mask"`
		WkcActual               uint32 `json:"wkc_actual"`
		WkcExpected             uint32 `json:"wkc_expected"`
		MasterState             uint32 `json:"master_state"`
		IoConfigured            bool   `json:"io_configured"`
		IoInputWord             uint32 `json:"io_input_word"`
		IoOutputWord            uint32 `json:"io_output_word"`
		IoInputValid            bool   `json:"io_input_valid"`
		IoArmed                 bool   `json:"io_armed"`
		IoReason                uint32 `json:"io_reason"`
		Time                    uint64 `json:"t_ns"`
		Sequence                uint64 `json:"seq"`
		Cycle                   uint64 `json:"cycle"`
		Work                    uint64 `json:"work_ns"`
		Axes                    []axis `json:"ax"`
	}
	encoder := json.NewEncoder(stdout)
	encoder.SetEscapeHTML(false)
	emit := func(record ipcclient.CycleCaptureRecordV2, index ipcclient.TelemetryDumpIndex) error {
		row := cycle{Time: record.TNs, Sequence: record.Seq, Cycle: record.Cycle, Work: record.WorkNs}
		row.CycleJitterNs = record.CycleJitterNs
		row.ActiveTrajId = record.ActiveTrajId
		row.ActiveCommandSeq = record.ActiveCommandSeq
		row.JogGeneration = record.JogGeneration
		row.AppGrantGeneration = record.AppGrantGeneration
		row.ControlGeneration = record.ControlGeneration
		row.ConfigurationEpoch = record.ConfigurationEpoch
		row.HomeEpoch = record.HomeEpoch
		row.GrantDeadlineHostNs = record.GrantDeadlineHostNs
		row.JogInputDeadlineHostNs = record.JogInputDeadlineHostNs
		row.JogSourceSequence = record.JogSourceSequence
		row.BusSubmissionReturnCode = record.BusSubmissionReturnCode
		row.Armed = record.Armed != 0
		row.AxisEnableMask = record.AxisEnableMask
		row.ActiveMode = record.ActiveMode
		row.State = record.State
		row.CurrentPointIndex = record.CurrentPointIndex
		row.QueueDepth = record.QueueDepth
		row.LastEventCode = record.LastEventCode
		row.UnderrunCount = record.UnderrunCount
		row.StaleCommandFlag = record.StaleCommandFlag
		row.MotionDone = record.MotionDone != 0
		row.CapabilityFlags = record.CapabilityFlags
		row.JogSessionOpen = record.JogSessionOpen != 0
		row.JogHasInput = record.JogHasInput != 0
		row.JogAxisMask = record.JogAxisMask
		row.JogRamping = record.JogRamping != 0
		row.GrantActive = record.GrantActive != 0
		row.GrantAxisMask = record.GrantAxisMask
		row.SafetyFaultMask = record.SafetyFaultMask
		row.ExecutionFaultReasons = record.ExecutionFaultReasons
		row.SubmittedAxisMask = record.SubmittedAxisMask
		row.BusSubmissionOperation = record.BusSubmissionOperation
		row.PdoFreshAxisMask = record.PdoFreshAxisMask
		row.ConfigVerifiedMask = record.ConfigVerifiedMask
		row.HomeValidMask = record.HomeValidMask
		row.ServiceModeAxisMask = record.ServiceModeAxisMask
		row.WkcActual = record.WkcActual
		row.WkcExpected = record.WkcExpected
		row.MasterState = record.MasterState
		row.IoConfigured = record.IoConfigured != 0
		row.IoInputWord = record.IoInputWord
		row.IoOutputWord = record.IoOutputWord
		row.IoInputValid = record.IoInputValid != 0
		row.IoArmed = record.IoArmed != 0
		row.IoReason = record.IoReason
		for i := uint32(0); i < record.Axes; i++ {
			a := record.Ax[i]
			out := axis{I: i, Position: a.Position, Target: a.Target, Statusword: a.Statusword, Error: a.ErrorCode, Manufacturer: a.ManufacturerErrorCode, VelocityActualCountsPerS: a.VelocityActualCountsPerS, FollowingErrorCounts: a.FollowingErrorCounts, VelocityActualValid: a.VelocityActualValid != 0, FollowingErrorValid: a.FollowingErrorValid != 0}
			out.DIBits, out.DIValid = a.DiBits, a.DiValid != 0
			out.ExternalEnableActive, out.ExternalEnableValid = a.ExternalEnableActive != 0, a.ExternalEnableValid != 0
			out.CoordinateCounts = a.CoordinateCounts
			out.CoordinateEpoch = a.CoordinateEpoch
			out.PdoObservedTimeNs = a.PdoObservedTimeNs
			out.CollisionPeakFollowingErrorCounts = a.CollisionPeakFollowingErrorCounts
			out.CollisionTripCount = a.CollisionTripCount
			out.CollisionSamples = a.CollisionSamples
			out.CollisionLastTripPeakFollowingErrorCounts = a.CollisionLastTripPeakFollowingErrorCounts
			out.AbsoluteSourceCounts = a.AbsoluteSourceCounts
			out.CoordinateReason = a.CoordinateReason
			out.CalibrationValid, out.PositionState, out.AuditReason = a.CalibrationValid, a.PositionState, a.AuditReason
			out.AuditLastVerifiedNs, out.AuditStartedNs, out.AuditCompletedNs = a.AuditLastVerifiedNs, a.AuditStartedNs, a.AuditCompletedNs
			out.AuditCoherenceErrorCounts = a.AuditCoherenceErrorCounts
			out.TargetVelocityCountsPerS = a.TargetVelocityCountsPerS
			out.CollisionArmed = a.CollisionArmed != 0
			out.CollisionTorqueCycles = a.CollisionTorqueCycles
			out.CollisionFollowingErrorCycles = a.CollisionFollowingErrorCycles
			out.CollisionPeakTorqueRaw = a.CollisionPeakTorqueRaw
			out.CollisionLastTripQuantity = a.CollisionLastTripQuantity
			out.CollisionTripSustainedCycles = a.CollisionTripSustainedCycles
			out.CollisionLastTripPeakTorqueRaw = a.CollisionLastTripPeakTorqueRaw
			out.BrakeState = a.BrakeState
			out.NativeHomePositionOffset = a.NativeHomePositionOffset
			out.NativeHomeLastAbortCode = a.NativeHomeLastAbortCode
			out.ExtPositionErrorCounts = a.ExtPositionErrorCounts
			out.ExtMultiTurnLo = a.ExtMultiTurnLo
			out.ExtMultiTurnHi = a.ExtMultiTurnHi
			out.MaxAbsVelocityActualCountsPerS = a.MaxAbsVelocityActualCountsPerS
			out.MaxAbsFollowingErrorCounts = a.MaxAbsFollowingErrorCounts
			out.TorqueRaw = a.TorqueRaw
			out.ExtBusVoltageRaw = a.ExtBusVoltageRaw
			out.ExtLoadRateRaw = a.ExtLoadRateRaw
			out.ExtIgbtTempRaw = a.ExtIgbtTempRaw
			out.ExtMotorTempRaw = a.ExtMotorTempRaw
			out.ExtDriveNotReadyBits = a.ExtDriveNotReadyBits
			out.ExtMotorNotRotatingCode = a.ExtMotorNotRotatingCode
			out.ExtValidMask = a.ExtValidMask
			out.ModeDisplay = a.ModeDisplay
			out.Ds402State = a.Ds402State
			out.TorqueValid = a.TorqueValid != 0
			out.CoordinateValid = a.CoordinateValid != 0
			out.CoordinateSourceValid = a.CoordinateSourceValid != 0
			out.NativeHomeState = a.NativeHomeState
			out.SlaveAlState = a.SlaveAlState
			out.SlaveOnline = a.SlaveOnline != 0
			out.SlaveOperational = a.SlaveOperational != 0
			out.PdoFresh = a.PdoFresh != 0
			out.TargetVelocityValid = a.TargetVelocityValid != 0
			out.ExtValid = a.ExtValid != 0
			for field, key := range index.AbsoluteKeys[i] {
				if key != "" {
					out.Absolute = append(out.Absolute, absolute{key, a.AbsoluteValue[field], a.AbsoluteValid[field]})
				}
			}
			row.Axes = append(row.Axes, out)
		}
		return encoder.Encode(row)
	}
	var err error
	if args[0] == "dump-to-jsonl" {
		_, err = ipcclient.ReadTelemetryDump(args[1], emit)
	} else {
		err = writeTelemetryBatchJSONL(args[1], encoder, emit)
	}
	if err != nil {
		fmt.Fprintln(stderr, err)
		return 1
	}
	return 0
}

type telemetryBatchJSONLHeader struct {
	Schema             string `json:"schema"`
	RawSHA256          string `json:"raw_sha256"`
	AdapterIncarnation string `json:"adapter_incarnation"`
	DaemonIncarnation  string `json:"daemon_incarnation"`
	MachineSHA256      string `json:"machine_sha256"`
	DeploymentSHA256   string `json:"deployment_sha256"`
	CyclePeriodNS      uint64 `json:"cycle_period_ns"`
	AxisCount          uint32 `json:"axis_count"`
	RecordLayoutDigest string `json:"record_layout_digest"`
	FirstSequence      uint64 `json:"first_sequence"`
	LastSequence       uint64 `json:"last_sequence"`
	Dropped            uint64 `json:"dropped"`
	RecordCount        uint32 `json:"record_count"`
}

func writeTelemetryBatchJSONL(path string, encoder *json.Encoder, emit func(ipcclient.CycleCaptureRecordV2, ipcclient.TelemetryDumpIndex) error) error {
	file, err := os.Open(path)
	if err != nil {
		return err
	}
	defer file.Close()
	raw, err := io.ReadAll(io.LimitReader(file, ipcclient.TelemetryBatchMaxBytes+1))
	if err != nil {
		return err
	}
	if len(raw) > ipcclient.TelemetryBatchMaxBytes {
		return fmt.Errorf("telemetry batch exceeds %d-byte limit", ipcclient.TelemetryBatchMaxBytes)
	}
	reader := bytes.NewReader(raw)
	batch, err := ipcclient.ReadTelemetryPublication(reader)
	if err != nil {
		return err
	}
	if reader.Len() != 0 {
		return fmt.Errorf("telemetry batch has %d trailing bytes", reader.Len())
	}
	// Validate the entire input before emitting the envelope or any cycle rows.
	h := batch.Header
	digest := sha256.Sum256(raw)
	if err := encoder.Encode(telemetryBatchJSONLHeader{
		Schema: "rosie.rt-core.batch-jsonl.v1", RawSHA256: fmt.Sprintf("%x", digest),
		AdapterIncarnation: string(h.AdapterIncarnation[:]), DaemonIncarnation: string(h.DaemonIncarnation[:]),
		MachineSHA256: string(h.MachineSha256[:]), DeploymentSHA256: string(h.DeploymentSha256[:]),
		CyclePeriodNS: h.CyclePeriodNs, AxisCount: h.AxisCount, RecordLayoutDigest: string(h.RecordLayoutDigest[:]),
		FirstSequence: h.FirstSequence, LastSequence: h.LastSequence, Dropped: h.Dropped, RecordCount: h.RecordCount,
	}); err != nil {
		return err
	}
	// Publications do not carry dump absolute-field names. Do not invent a catalog.
	index := ipcclient.TelemetryDumpIndex{AbsoluteKeys: make([][]string, h.AxisCount)}
	for _, record := range batch.Records {
		if err := emit(record, index); err != nil {
			return err
		}
	}
	return nil
}
