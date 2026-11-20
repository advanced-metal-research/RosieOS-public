package main

import (
	"context"
	"net/http"
	"strings"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
)

// runBackend answers Run program the way the rt-core controller does: a
// confirmation first, playing once confirmed, a typed refusal at Load.
type runBackend struct {
	controlBackend
	presses []denseexec.RunProgramRequest
	clients []string
	refuse  bool
}

func (b *runBackend) RunProgram(ctx context.Context, in denseexec.RunProgramRequest) (*denseexec.RunProgramResult, error) {
	b.presses = append(b.presses, in)
	b.clients = append(b.clients, denseexec.ClientOf(ctx).Label)
	result := &denseexec.RunProgramResult{State: "confirm", Action: denseexec.RunProgramMoveAction, NeedsMove: true,
		Joints: []denseexec.RecoverJoint{{Axis: 0, ID: "J1", FromDeg: 40, ToDeg: 10, DeltaDeg: -30, Moves: true}}}
	if b.refuse {
		detail := "Load refused: trajectory_limits: J3 velocity"
		result.State, result.Refusal = "refused", &denseexec.RecoverRefusal{Step: "load", Code: "trajectory_limits", Detail: detail}
		return result, &denseexec.Error{Status: http.StatusConflict, Code: "trajectory_limits", Detail: detail}
	}
	if in.ConfirmMove {
		result.State, result.SessionID = "playing", "ds-run"
	}
	return result, nil
}

// The route forwards the Load body and the confirmation with the client that
// pressed, answers confirm and playing with 200, and a refusal with 409 whose
// detail is the operator's sentence and whose run member keeps the steps.
func TestRunProgramRoute(t *testing.T) {
	backend := &runBackend{controlBackend: controlBackend{controllerID: "offline-programming:server-olp001"}}
	server := NewServer(ServerOptions{DenseExecution: backend})
	path := denseExecutionPath + "/run-program"
	load := `"trajectory_digest":"sha256:` + strings.Repeat("a", 64) + `","plan_id":"p","program_id":"prog","program_digest":"d","manifest_revision":7,"plan_revision":2,"dry_run":true`
	response, body := controlRequest(t, server, deckClient, "POST", path, `{`+load+`,"fraction":0.1}`)
	if response.Code != http.StatusOK || body["state"] != "confirm" || body["action"] != "Move to program start and run" || body["needs_move"] != true {
		t.Fatalf("%d %s", response.Code, response.Body.String())
	}
	response, body = controlRequest(t, server, deckClient, "POST", path, `{`+load+`,"fraction":0.1,"confirm_move":true,"confirmed":[{"axis":0,"id":"J1","from_deg":40,"to_deg":10}]}`)
	if response.Code != http.StatusOK || body["state"] != "playing" || body["session_id"] != "ds-run" {
		t.Fatalf("%d %s", response.Code, response.Body.String())
	}
	second := backend.presses[1]
	if len(backend.presses) != 2 || !second.ConfirmMove || second.ConfirmArm || second.Confirmed[0].ToDeg != 10 || second.PlanRevision != 2 || !second.DryRun || backend.clients[1] != "rosie-pendant-v5 (steamdeck-pendant)" {
		t.Fatalf("%+v %v", backend.presses, backend.clients)
	}
	backend.refuse = true
	response, body = controlRequest(t, server, deckClient, "POST", path, `{`+load+`,"fraction":0.1}`)
	run, _ := body["run"].(map[string]any)
	if response.Code != http.StatusConflict || body["error"] != "trajectory_limits" || !strings.HasPrefix(body["detail"].(string), "Load refused") || run["state"] != "refused" {
		t.Fatalf("%d %s", response.Code, response.Body.String())
	}
	if response, _ := controlRequest(t, server, deckClient, "POST", path, `not json`); response.Code != http.StatusBadRequest || len(backend.presses) != 3 {
		t.Fatalf("invalid body reached the controller: %d", response.Code)
	}
}

func (b *runBackend) FinishLoop(ctx context.Context) (*denseexec.LoopStatus, error) {
	if denseexec.ClientOf(ctx).Label != "rosie-pendant-v5 (steamdeck-pendant)" {
		return nil, &denseexec.Error{Status: http.StatusConflict, Code: "loop_owner_only", Detail: "Only rosie-pendant-v5 (steamdeck-pendant), which started the loop, can finish it; STOP stops it from anywhere"}
	}
	return &denseexec.LoopStatus{Mode: "loop", Cycles: 4, State: "finishing", Owner: "rosie-pendant-v5 (steamdeck-pendant)", Attended: true}, nil
}

// Finish cycle reaches the controller with its client, through the gates
// that pass Stop; the owner check is the controller's.
func TestFinishLoopRoute(t *testing.T) {
	backend := &runBackend{controlBackend: controlBackend{controllerID: "offline-programming:server-olp001"}}
	server := NewServer(ServerOptions{DenseExecution: backend})
	response, body := controlRequest(t, server, deckClient, "POST", denseExecutionPath+"/run-program/finish", `{}`)
	loop, _ := body["loop"].(map[string]any)
	if response.Code != http.StatusOK || loop["state"] != "finishing" {
		t.Fatalf("%d %s", response.Code, response.Body.String())
	}
	response, body = controlRequest(t, server, browserTab, "POST", denseExecutionPath+"/run-program/finish", `{}`)
	if response.Code != http.StatusConflict || body["error"] != "loop_owner_only" {
		t.Fatalf("%d %s", response.Code, response.Body.String())
	}
}
