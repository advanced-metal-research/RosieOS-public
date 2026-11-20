package main

import (
	"context"
	"net/http"
	"strings"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
)

// recoverBackend answers the recovery the way the rt-core controller does:
// a confirmation first, the move once confirmed, a typed refusal with the
// result when J5 is still in alarm.
type recoverBackend struct {
	controlBackend
	presses []denseexec.RecoverToZeroRequest
	clients []string
	hot     bool
}

func (b *recoverBackend) RecoverToZero(ctx context.Context, in denseexec.RecoverToZeroRequest) (*denseexec.RecoverToZeroResult, error) {
	b.presses = append(b.presses, in)
	b.clients = append(b.clients, denseexec.ClientOf(ctx).Label)
	result := &denseexec.RecoverToZeroResult{Action: denseexec.RecoverToZeroAction, State: "confirm_arm",
		Joints: []denseexec.RecoverJoint{{Axis: 4, ID: "J5", FromDeg: 37.4, DeltaDeg: -37.4, Moves: true}}}
	if b.hot {
		detail := "J5 Er41.1 (Motor over-temperature due to locked- rotor) still active — let it cool, then press again"
		result.State, result.Refusal = "refused", &denseexec.RecoverRefusal{Step: "reset", Code: "fault_persists", Detail: detail}
		return result, &denseexec.Error{Status: http.StatusConflict, Code: "fault_persists", Detail: detail}
	}
	if in.ConfirmArm {
		result.State = "done"
	}
	return result, nil
}

// The route forwards the press with its client, answers the confirmation and
// the done state with 200, and a refusal with 409 whose detail is the
// operator's sentence and whose recovery member keeps the steps.
func TestRecoverToZeroRoute(t *testing.T) {
	backend := &recoverBackend{controlBackend: controlBackend{controllerID: "offline-programming:server-olp001"}}
	server := NewServer(ServerOptions{DenseExecution: backend})
	path := denseExecutionPath + "/recover-to-zero"
	response, body := controlRequest(t, server, deckClient, "POST", path, `{"fraction":0.1}`)
	if response.Code != http.StatusOK || body["state"] != "confirm_arm" || body["action"] != "Arm and move all joints to zero" {
		t.Fatalf("%d %s", response.Code, response.Body.String())
	}
	response, body = controlRequest(t, server, deckClient, "POST", path, `{"fraction":0.1,"confirm_arm":true,"confirmed":[{"axis":4,"id":"J5","from_deg":37.4}]}`)
	if response.Code != http.StatusOK || body["state"] != "done" {
		t.Fatalf("%d %s", response.Code, response.Body.String())
	}
	if len(backend.presses) != 2 || !backend.presses[1].ConfirmArm || backend.presses[1].Confirmed[0].FromDeg != 37.4 || backend.clients[1] != "rosie-pendant-v5 (steamdeck-pendant)" {
		t.Fatalf("%+v %v", backend.presses, backend.clients)
	}
	backend.hot = true
	response, body = controlRequest(t, server, deckClient, "POST", path, `{"fraction":0.1}`)
	recovery, _ := body["recovery"].(map[string]any)
	if response.Code != http.StatusConflict || body["error"] != "fault_persists" || !strings.HasPrefix(body["detail"].(string), "J5 Er41.1") || recovery["state"] != "refused" {
		t.Fatalf("%d %s", response.Code, response.Body.String())
	}
	if response, _ := controlRequest(t, server, deckClient, "POST", path, `not json`); response.Code != http.StatusBadRequest || len(backend.presses) != 3 {
		t.Fatalf("invalid body reached the controller: %d", response.Code)
	}
}
