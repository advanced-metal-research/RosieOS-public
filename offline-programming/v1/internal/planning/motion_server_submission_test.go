package planning

import (
	"bytes"
	"encoding/json"
	"strings"
	"testing"
	"time"
)

func TestMotionServerSubmissionUsesCanonicalInProcessContract(t *testing.T) {
	now := time.Date(2026, 7, 30, 13, 0, 0, 0, time.UTC)
	request := validPlanRequest(now)
	request.Target = TargetIdentity{ProfileID: "local-simulator"}
	request.Rows[0].MoveType = "line_tcp"
	request.Rows[0].MotionPhase = "weld"
	request.Rows[0].ToolFrame = "torch_tcp"
	request.Rows[0].WorkFrame = "world"
	request.Rows[0].TCPVelocityMS = 0.05
	request.Rows[0].JointVelocityRadS = 0.25
	request.Rows[1] = request.Rows[0]
	request.Rows[1].RowID = "row-2"
	request.Rows[1].MoveType = "joint_ptp"
	request.Rows = append(request.Rows, PlanRow{
		RowID:    "comment-1",
		Revision: request.ProgramRevision,
		Enabled:  true,
		RowType:  "comment",
		Command:  "operator-note",
	})
	request = withRobotV4Program(t, request)

	submission, err := NewMotionServerSubmission(
		request,
		41,
		"robot.v4.local-simulator.plan",
	)
	if err != nil {
		t.Fatalf("create submission: %v", err)
	}
	if submission.Transport != MotionServerPlanTransport ||
		submission.PlanRevision != 41 ||
		submission.RequestID != request.RequestID {
		t.Fatalf("submission identity = %+v", submission)
	}
	var payload map[string]any
	if err := json.Unmarshal(submission.Payload, &payload); err != nil {
		t.Fatal(err)
	}
	if payload["execution_mode"] != "simulation" ||
		payload["test_unknown_program"] == nil {
		t.Fatalf("canonical payload metadata = %#v", payload)
	}
	if !bytes.Equal(submission.Payload, request.Program) {
		t.Fatal("shared Robot V4 program was reconstructed instead of preserved byte-for-byte")
	}
	if _, exists := payload["planned_path"]; exists {
		t.Fatal("authored request unexpectedly contains a planned path")
	}
	rows, ok := payload["rows"].([]any)
	if !ok || len(rows) != 3 {
		t.Fatalf("rows = %#v", payload["rows"])
	}
	first := rows[0].(map[string]any)
	second := rows[1].(map[string]any)
	comment := rows[2].(map[string]any)
	if first["move_type"] != "line_tcp" ||
		second["move_type"] != "joint_ptp" ||
		comment["row_type"] != "comment" ||
		comment["command"] != "operator-note" ||
		first["test_unknown_field"] != "preserve-me" {
		t.Fatalf("complete shared rows were not preserved = %#v %#v %#v", first, second, comment)
	}
	encoded := string(submission.Payload)
	for _, forbidden := range []string{
		`"command":"play"`,
		`"command":"arm"`,
		`ethercat`,
		`rtcore`,
		`udp`,
	} {
		if strings.Contains(strings.ToLower(encoded), forbidden) {
			t.Fatalf("submission contains forbidden command/output field %q: %s", forbidden, encoded)
		}
	}
}

func TestMotionServerSubmissionRejectsUnsupportedOrUnboundedIdentity(t *testing.T) {
	request := validPlanRequest(time.Now().UTC())
	request.Rows[0].MoveType = "teleport"
	request = withRobotV4Program(t, request)
	if _, err := NewMotionServerSubmission(request, 1, "local.plan"); err == nil {
		t.Fatal("unsupported move type was accepted")
	}
	request.Rows[0].MoveType = "line_tcp"
	request = withRobotV4Program(t, request)
	if _, err := NewMotionServerSubmission(request, 0, "local.plan"); err == nil {
		t.Fatal("zero plan revision was accepted")
	}
	if _, err := NewMotionServerSubmission(request, 1, "local plan"); err == nil {
		t.Fatal("whitespace-bearing subject was accepted")
	}
	var program map[string]any
	if err := json.Unmarshal(request.Program, &program); err != nil {
		t.Fatal(err)
	}
	program["trajectory"] = []any{}
	request.Program, _ = json.Marshal(program)
	if _, err := NewMotionServerSubmission(request, 1, "local.plan"); err == nil {
		t.Fatal("authored program carrying a trajectory was accepted")
	}
}
