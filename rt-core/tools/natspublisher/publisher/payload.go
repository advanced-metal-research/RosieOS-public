//go:build linux

package publisher

import (
	"encoding/json"
	"time"

	"rosieos/rt-core/ipcclient"
)

// missingFields covers the v4 ring/status fields with no equivalent observation.
// Nulls preserve the distinction between unavailable evidence and a measured zero.
var missingFields = []string{
	"servo_control_enabled", "arm_disarm_requests", "axis_target_counts", "axis_error_counts",
	"axis_mode_display", "axis_brake_command", "axis_braking_command", "axis_voltage_mv", "axis_current_ma",
	"hm35_home_position_counts", "active_jog_drop_reason_code", "active_jog_drop_reason", "active_jog_drop_count",
	"speed_limit_enabled", "speed_limit_rad_s", "speed_limit_last_scale", "speed_limit_requested_max_rad_s",
	"speed_limit_applied_max_rad_s", "speed_limit_axis_mask", "speed_limit_command_count", "speed_limit_sender_id",
	"safe_low_raw_counts", "safe_high_raw_counts", "operational_slaves", "sync0_configured", "sync0_cycle_us",
	"sync0_cycle_multiple_250us", "sync0_error_state", "rtcore_loop_jitter_us", "pdo_fresh_sample",
	"commanded_actual_stop_sample", "commanded_actual_stop_commanded_zero", "commanded_actual_stop_target_feedback_aligned",
	"commanded_actual_stop_max_abs_target_feedback_counts", "axis_max_profile_velocity_counts", "axis_max_step_counts_per_cycle",
	"axis_active_jog_max_lead_counts", "active_jog_target_lead_seconds", "active_jog_target_max_lead_rad",
	"motion_output_enabled", "ethercat_output_enabled", "tour_start_alarm", "virtual_simulation_indicator",
	"recent_commands", "test_session_id", "test-session-id", "realtime_core_command_log_capacity",
	"realtime_core_command_log_captured_count", "realtime_core_command_log_gap_count", "realtime_core_command_log_latest_sequence",
	"realtime_core_command_log_source", "recent_commands_full_count", "recent_commands_payload_count",
	"recent_commands_payload_limit", "recent_commands_payload_truncated",
	"no_real_movement", "link_up", "responding_slaves", "latest_max_abs_qd_rad_s", "runtime_mode_epoch",
	"pair_config_jetson_id", "pair_config_command_id", "pair_config_revision", "pair_config_sha256",
	"pair_config_generated_urdf_sha256", "home_profile_key", "pair_config_gear_numerator", "pair_config_gear_denominator",
	"pair_config_home_position_rad", "pair_config_activation_epoch", "pair_config_home_epoch",
	"pair_config_ratio_reconciliation_state", "pair_config_ratio_reconciliation_simulated",
	"pair_config_ratio_reconciliation_abort_code", "pair_config_ratio_reconciliation_failed_axis_mask",
	"pair_config_observed_gear_numerator", "pair_config_observed_gear_denominator",
	"realtime_core_command_log_complete", "pair_config_drive_ratio_converged",
}

// payloads maps sampled evidence only. Raw target counts, physical output permits
// and HM35 anchors cannot be reconstructed from logical positions or enable intent.
func payloads(host string, s snapshot, health diagnostics, cadence time.Duration) ([]byte, []byte, error) {
	status := envelope(host, "robot.v4.rtcore.status.v1", "status", health.Drops)
	status["publisher_state"], status["last_error"] = health.State, health.LastError
	status["reconnects"], status["drops"] = health.Reconnects, health.Drops
	if s.status.Core.TimeNS == 0 {
		status["status"] = nil
		raw, err := json.Marshal(status)
		return raw, nil, err
	}
	core := s.status.Core
	fields := make(map[string]any, len(missingFields)+32)
	for _, name := range missingFields {
		fields[name] = nil
	}
	fields["name"], fields["state"] = "robot-v4-rtcore", "running"
	fields["mode"] = nil
	mode := "sim"
	if core.Backend == "ethercat" {
		mode = "live"
		fields["mode"] = "ethercat_live"
	}
	fields["rtcore_mode"], fields["runtime_mode"], fields["simulation_mode"] = mode, mode, core.Backend == "simulation"
	fields["ethercat_live"] = core.Backend == "ethercat"
	fields["servos_armed_requested"] = core.Armed != 0
	fields["home_valid_axis_mask"] = core.HomeValidMask
	fields["axis_enable_mask"] = core.AxisEnableMask
	fields["configured_axis_count"] = len(core.Axes)
	fields["ethercat_cycle_us"] = float64(s.description.CycleNS) / 1000
	fields["active_jog"] = nil
	if _, err := sourceAge(s.status.Motion.TimeNS); err == nil {
		fields["active_jog"] = s.status.Motion.Mode == ipcclient.MOTION_MODE_JOG
	}
	positions := make([]any, len(core.Axes))
	statuswords := make([]any, len(core.Axes))
	errors := make([]any, len(core.Axes))
	lower, upper := make([]any, len(core.Axes)), make([]any, len(core.Axes))
	var errorMask, faultMask uint32
	enabled := 0
	fresh := true
	for i, a := range core.Axes {
		axisFresh := a.PDOFresh != 0 && a.TimeNS == core.TimeNS
		fresh = fresh && axisFresh
		if axisFresh {
			positions[i], statuswords[i], errors[i] = a.PositionCounts, a.Statusword, a.ErrorCode
			// Measured CiA402 operation-enabled mask/value, matching ipcclient.AllOperationEnabled.
			if a.Statusword&0x006f == 0x0027 {
				enabled++
			}
			if a.ErrorCode != 0 {
				errorMask |= 1 << i
			}
			if a.Statusword&0x0008 != 0 {
				faultMask |= 1 << i
			}
		}
		if s.description.Axes[i].PositionUnit == "rad" {
			lower[i], upper[i] = s.description.Axes[i].MinPosition, s.description.Axes[i].MaxPosition
		}
	}
	fields["pdo_fresh"] = fresh
	fields["ds402_enabled_axes"], fields["axis_error_mask"], fields["axis_fault_mask"] = nil, nil, nil
	if fresh {
		fields["ds402_enabled_axes"], fields["axis_error_mask"], fields["axis_fault_mask"] = enabled, errorMask, faultMask
	}
	fields["axis_pos_counts"], fields["axis_statusword"] = positions, statuswords
	fields["axis_error_code"], fields["axis_603f_error_code"] = errors, errors
	fields["safe_min_rad"], fields["safe_max_rad"] = lower, upper
	fields["updated_at"] = s.updated.UTC().Format(time.RFC3339Nano)
	fields["dropped"] = health.Drops
	fields["tour_start_alarm_valid"], fields["virtual_simulation_indicator_valid"] = false, false
	fields["revpi_output_readback"] = map[string]any{
		"schema": "robot.v4.revpi-output-readback.v1", "output_count": nil, "values_mask": nil,
		"valid_mask": 0, "sampled_at": nil, "age_ms": nil, "fresh": false, "error": "unavailable from rt-control",
	}
	fields["rt_core"] = map[string]any{
		"core": core, "motion": s.status.Motion, "execution": s.status.Execution, "axes": s.status.Axes,
		"daemon_incarnation": s.status.DaemonIncarnation, "adapter_incarnation": s.status.AdapterIncarnation,
		"configuration_sha256": s.description.ConfigurationSHA256, "machine_sha256": s.description.MachineSHA256,
		"deployment_sha256": s.description.DeploymentSHA256, "capabilities_digest": s.description.CapabilitiesDigest,
	}
	status["status"] = fields
	statusRaw, err := json.Marshal(status)
	if err != nil {
		return nil, nil, err
	}
	record := make(map[string]any, len(fields)+3)
	for key, value := range fields {
		if key != "rt_core" {
			record[key] = value
		}
	}
	record["event"], record["sample"], record["time_ms"] = "timeseries", s.sequence-1, float64(core.TimeNS)/1e6
	batch := envelope(host, "robot.v4.rtcore.telemetry.batch.v1", "telemetry.batch", health.Drops)
	batch["published_at"] = status["published_at"]
	batch["sequence"], batch["source_monotonic_ms"], batch["interval_ms"] = s.sequence, float64(core.TimeNS)/1e6, cadence.Milliseconds()
	batch["count"], batch["records"] = 1, []any{record}
	batchRaw, err := json.Marshal(batch)
	return statusRaw, batchRaw, err
}

func envelope(host, schema, suffix string, dropped uint64) map[string]any {
	return map[string]any{
		"schema": schema, "stream": Stream, "subject": subject(host, suffix), "host": host, "component": "rtcore",
		"test_session_id": nil, "published_at": time.Now().UTC().Format(time.RFC3339Nano), "recent_commands": nil, "dropped": dropped,
	}
}
