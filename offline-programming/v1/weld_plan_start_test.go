package main

import (
	"context"
	"encoding/json"
	"errors"
	"strings"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
)

func startFixture() (map[string]any, denseexec.RobotCell) {
	robot := denseexec.RobotCell{Valid: true, ModelID: "rosie_1420_v1", RobotDescriptionSHA256: "sha256:model", MachinePlanningCalibrationSHA256: "sha256:calibration", MachineConfigurationSHA256: "sha256:config"}
	axes := []any{}
	described := []any{}
	for i := 0; i < 6; i++ {
		axes = append(axes, map[string]any{"time_ns": 100, "logical_position": float64(i) * .1, "logical_valid": true, "home_valid": true, "calibration_valid": true, "position_state": "trusted"})
		described = append(described, map[string]any{"id": []string{"J1", "J2", "J3", "J4", "J5", "J6"}[i], "index": i, "position_unit": "rad"})
	}
	return map[string]any{"available": true, "robot_cell": robot, "rt_core": map[string]any{"description": map[string]any{"axes": described}, "status": map[string]any{"time_ns": 100, "motion": map[string]any{"time_ns": 100, "mode": 0, "state": 0, "queue_depth": 0, "done": true}, "adapter_incarnation": "adapter", "daemon_incarnation": "daemon", "core": map[string]any{"configuration_sha256": "config", "configuration_epoch": 1, "home_epoch": 32}, "axes": axes}}}, robot
}

func TestMeasuredPlanningStartPreservesEveryNonzeroJoint(t *testing.T) {
	raw, robot := startFixture()
	b, _ := json.Marshal(raw)
	start, err := measuredStartObservation(b, robot)
	if err != nil {
		t.Fatal(err)
	}
	if len(start.JointNames) != 6 || start.JointNames[5] != "J6" || start.JointValues[5] != .5 || start.CoordinateIdentity != "config:1:32" || start.ObservationIdentity != "adapter:daemon" {
		t.Fatalf("unexpected start: %+v", start)
	}
	for _, op := range []string{"move", "home", "approach", "weld", "transit", "retract"} {
		if !hasMotionNodes(json.RawMessage(`{"nodes":[{"op":"`+op+`"}]}`)) || hasMotionNodes(json.RawMessage(`{"nodes":[{"op":"`+op+`","enabled":false}]}`)) {
			t.Fatalf("enabled motion detection for %s", op)
		}
	}
	if hasMotionNodes(json.RawMessage(`{"nodes":[{"op":"dwell"},{"op":"io"}]}`)) {
		t.Fatal("events alone are not a motion program")
	}
}

func TestMeasuredPlanningStartRejectsUnverifiedOrMovingJ6AndIdentityChanges(t *testing.T) {
	for _, field := range []string{"home_valid", "logical_valid", "calibration_valid", "logical_position", "active_motion", "open_jog", "missing_motion", "position_state", "configuration", "calibration", "fault"} {
		t.Run(field, func(t *testing.T) {
			raw, robot := startFixture()
			native := raw["rt_core"].(map[string]any)
			status := native["status"].(map[string]any)
			axis := status["axes"].([]any)[5].(map[string]any)
			switch field {
			case "logical_position":
				delete(axis, field)
			case "active_motion":
				status["motion"].(map[string]any)["state"] = 3
			case "open_jog":
				status["jog"] = map[string]any{"Open": 1}
			case "missing_motion":
				delete(status["motion"].(map[string]any), "mode")
			case "position_state":
				axis[field] = "unverified"
			case "configuration":
				robot.MachineConfigurationSHA256 = "sha256:changed"
			case "calibration":
				robot.MachinePlanningCalibrationSHA256 = "sha256:changed"
			case "fault":
				status["core"].(map[string]any)["safety_fault_mask"] = 1
			default:
				axis[field] = false
			}
			b, _ := json.Marshal(raw)
			if _, err := measuredStartObservation(b, robot); err == nil {
				t.Fatal("accepted unsafe capture")
			}
		})
	}
}

func TestMeasuredPlanningRequestCannotFallBackToOfflineReset(t *testing.T) {
	server := &Server{}
	_, _, err := server.packWeldPlan(context.Background(), weldPlanRequest{RequireMeasuredStart: true, WorkpieceAbsent: true, Program: json.RawMessage(`{"nodes":[{"op":"move"}]}`)})
	if err == nil || !strings.Contains(err.Error(), "no model-reset fallback") {
		t.Fatalf("wanted explicit refusal, got %v", err)
	}
}

func TestMeasuredPlanningStartRecoversStalePlanWithoutIgnoringNativeState(t *testing.T) {
	for _, scenario := range []string{"stale_plan", "unrelated_error", "wrong_kind", "wrong_refusal", "native_fault", "inhibited", "stop_uncertain", "invalid_coordinate", "active_motion"} {
		t.Run(scenario, func(t *testing.T) {
			raw, robot := startFixture()
			native := raw["rt_core"].(map[string]any)
			status := native["status"].(map[string]any)
			raw["daemon_error"] = "planning_start_stale: replan from verified current feedback"
			native["reason"] = raw["daemon_error"]
			native["refusal_kind"], native["refusal"] = "prepare_program", "planning_start_stale"
			status["motion"] = map[string]any{"time_ns": 100, "mode": 2, "state": 5, "queue_depth": 0, "done": true}
			switch scenario {
			case "unrelated_error":
				raw["daemon_error"] = "native_rejected"
			case "wrong_kind":
				native["refusal_kind"] = "start_program"
			case "wrong_refusal":
				native["refusal"] = "native_rejected"
			case "native_fault":
				status["core"].(map[string]any)["safety_fault_mask"] = 8
			case "inhibited":
				native["inhibited"] = true
			case "stop_uncertain":
				native["stop_uncertain"] = true
			case "invalid_coordinate":
				status["axes"].([]any)[5].(map[string]any)["logical_valid"] = false
			case "active_motion":
				status["motion"].(map[string]any)["state"] = 3
			}
			b, _ := json.Marshal(raw)
			start, err := measuredStartObservation(b, robot)
			if scenario == "stale_plan" {
				if err != nil || start.CoordinateIdentity != "config:1:32" {
					t.Fatalf("fresh read-only recovery failed: %v %+v", err, start)
				}
			} else if err == nil {
				t.Fatal("admitted invalid recovery state")
			}
		})
	}
}

func TestMeasuredPlanningStartAfterAcknowledgedOperatorStop(t *testing.T) {
	for _, scenario := range []string{"stopped", "no_session", "missing_ack", "active_grant", "armed", "enabled", "missing_armed", "wrong_session_state", "different_stop", "other_error", "fault", "inhibited", "uncertain", "active_motion", "invalid_home"} {
		t.Run(scenario, func(t *testing.T) {
			raw, robot := startFixture()
			native := raw["rt_core"].(map[string]any)
			status := native["status"].(map[string]any)
			core := status["core"].(map[string]any)
			raw["daemon_error"], native["reason"] = "operator_stop", "operator_stop"
			raw["session"] = map[string]any{"state": "stopped", "stop_reason": "operator_stop"}
			status["grant"] = map[string]any{"GrantActive": 0, "StopAckId": 123}
			core["armed"], core["axis_enable_mask"] = 0, 0
			switch scenario {
			case "missing_ack":
				status["grant"].(map[string]any)["StopAckId"] = 0
			case "active_grant":
				status["grant"].(map[string]any)["GrantActive"] = 1
			case "armed":
				core["armed"] = 1
			case "enabled":
				core["axis_enable_mask"] = 1
			case "missing_armed":
				delete(core, "armed")
			case "no_session":
				raw["session"] = nil
			case "wrong_session_state":
				raw["session"].(map[string]any)["state"] = "idle"
			case "different_stop":
				raw["session"].(map[string]any)["stop_reason"] = "lease_expired"
			case "other_error":
				raw["daemon_error"] = "operator_stop_failed"
			case "fault":
				core["safety_fault_mask"] = 8
			case "inhibited":
				native["inhibited"] = true
			case "uncertain":
				native["stop_uncertain"] = true
			case "active_motion":
				status["motion"].(map[string]any)["state"] = 3
			case "invalid_home":
				status["axes"].([]any)[0].(map[string]any)["home_valid"] = false
			}
			b, _ := json.Marshal(raw)
			_, err := measuredStartObservation(b, robot)
			if (scenario == "stopped" || scenario == "no_session") && err != nil {
				t.Fatal(err)
			}
			if scenario != "stopped" && scenario != "no_session" && err == nil {
				t.Fatal("accepted unconfirmed Stop or invalid native state")
			}
		})
	}
}

// Native Stop leaves the last mode as JOG, even after all authority is released,
// and so does a released hold while the robot stays armed. A finished jog is a
// planning start either way; an active one is refused with the instruction to
// let go, and every other refusal names its own cause.
// Match that public shape without including physical calibration/authority data.
func TestMeasuredPlanningStartAfterReleasedStoppedJog(t *testing.T) {
	admitted := map[string]bool{"released": true, "completed": true, "grant": true, "missing_ack": true, "lease": true, "missing_lease": true, "armed": true, "enabled": true, "armed_leased": true}
	moving := map[string]bool{"executing": true, "accepted": true, "not_done": true, "queued": true, "open": true, "input": true}
	for _, scenario := range []string{"released", "completed", "executing", "accepted", "not_done", "queued", "open", "input", "missing_jog", "grant", "missing_ack", "lease", "missing_lease", "armed", "enabled", "armed_leased", "fault", "home_active", "commissioning", "untrusted", "inhibited", "uncertain"} {
		t.Run(scenario, func(t *testing.T) {
			raw, robot := startFixture()
			native := raw["rt_core"].(map[string]any)
			status := native["status"].(map[string]any)
			core := status["core"].(map[string]any)
			motion := map[string]any{"time_ns": 100, "mode": 3, "state": 5, "queue_depth": 0, "done": true}
			jog := map[string]any{"Open": 0, "HasInput": 0}
			grant := map[string]any{"GrantActive": 0, "StopAckId": 123}
			status["motion"], status["jog"], status["grant"] = motion, jog, grant
			core["armed"], core["axis_enable_mask"], core["lease_valid"] = 0, 0, 0
			switch scenario {
			case "completed":
				motion["state"] = 4
			case "executing":
				motion["state"] = 3
			case "accepted":
				motion["state"], motion["done"] = 1, false
			case "armed_leased":
				// A released hold on an armed robot: lane closed, grant and lease held.
				grant["GrantActive"], grant["StopAckId"] = 1, 0
				core["armed"], core["axis_enable_mask"], core["lease_valid"] = 1, 63, 1
			case "not_done":
				motion["done"] = false
			case "queued":
				motion["queue_depth"] = 1
			case "open":
				jog["Open"] = 1
			case "input":
				jog["HasInput"] = 1
			case "missing_jog":
				delete(status, "jog")
			case "grant":
				grant["GrantActive"] = 1
			case "missing_ack":
				delete(grant, "StopAckId")
			case "lease":
				core["lease_valid"] = 1
			case "missing_lease":
				delete(core, "lease_valid")
			case "armed":
				core["armed"] = 1
			case "enabled":
				core["axis_enable_mask"] = 1
			case "fault":
				core["safety_fault_mask"] = 1
			case "home_active":
				core["native_home_active_axis_mask"] = 1
			case "commissioning":
				core["commissioning_phase"] = 1
			case "untrusted":
				status["axes"].([]any)[5].(map[string]any)["position_state"] = "unverified"
			case "inhibited":
				native["inhibited"] = true
			case "uncertain":
				native["stop_uncertain"] = true
			}
			b, _ := json.Marshal(raw)
			_, err := measuredStartObservation(b, robot)
			if admitted[scenario] != (err == nil) {
				t.Fatalf("%s: %v", scenario, err)
			}
			if err == nil {
				return
			}
			var refusal *planStartRefusal
			if !errors.As(err, &refusal) {
				t.Fatalf("%s: refusal has no code: %v", scenario, err)
			}
			if moving[scenario] {
				if refusal.code != planStartMoving || err.Error() != planStartMovingMessage {
					t.Fatalf("%s: an active jog must say to let go: %s %v", scenario, refusal.code, err)
				}
			} else if refusal.code != planStartUnverified || strings.Contains(err.Error(), "Release the jog") {
				t.Fatalf("%s: wrong cause: %s %v", scenario, refusal.code, err)
			}
		})
	}
}

// Each unverified refusal names its own cause: a fault is not "establish Home".
func TestMeasuredPlanningStartRefusalsNameTheirCause(t *testing.T) {
	for scenario, want := range map[string]string{
		"fault":             "safety fault (mask 0x8)",
		"home_active":       "Home is running",
		"commissioning":     "commissioning phase 2",
		"uncertain":         "stop_uncertain",
		"inhibited":         "outputs inhibited",
		"untrusted":         "J6: position unverified",
		"daemon_error":      `"native_rejected"`,
		"moving_trajectory": "A move is still running",
	} {
		t.Run(scenario, func(t *testing.T) {
			raw, robot := startFixture()
			native := raw["rt_core"].(map[string]any)
			status := native["status"].(map[string]any)
			core := status["core"].(map[string]any)
			switch scenario {
			case "fault":
				core["safety_fault_mask"] = 8
			case "home_active":
				core["native_home_active_axis_mask"] = 1
			case "commissioning":
				core["commissioning_phase"] = 2
			case "uncertain":
				native["stop_uncertain"] = true
			case "inhibited":
				native["inhibited"] = true
			case "untrusted":
				status["axes"].([]any)[5].(map[string]any)["position_state"] = "unverified"
			case "daemon_error":
				raw["daemon_error"] = "native_rejected"
			case "moving_trajectory":
				status["motion"] = map[string]any{"time_ns": 100, "mode": 2, "state": 3, "queue_depth": 0, "done": false}
			}
			b, _ := json.Marshal(raw)
			_, err := measuredStartObservation(b, robot)
			if err == nil || !strings.Contains(err.Error(), want) {
				t.Fatalf("want %q, got %v", want, err)
			}
		})
	}
}

// A finished Move-to or Cartesian step is a completed native trajectory
// (mode 2, state 4, done): admitted while the robot stays armed.
func TestMeasuredPlanningStartAfterFinishedMoveWhileArmed(t *testing.T) {
	raw, robot := startFixture()
	status := raw["rt_core"].(map[string]any)["status"].(map[string]any)
	status["motion"] = map[string]any{"time_ns": 100, "mode": 2, "state": 4, "queue_depth": 0, "done": true}
	status["jog"] = map[string]any{"Open": 0, "HasInput": 0}
	status["grant"] = map[string]any{"GrantActive": 1, "StopAckId": 0}
	core := status["core"].(map[string]any)
	core["armed"], core["axis_enable_mask"], core["lease_valid"] = 1, 63, 1
	b, _ := json.Marshal(raw)
	if _, err := measuredStartObservation(b, robot); err != nil {
		t.Fatal(err)
	}
}
