package main

import (
	"context"
	"net/http"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
)

// settleArmBackend answers Arm the way the rt-core controller now does: a
// latched J5 drive alarm is reset first (counted), and a still-hot J5 refuses
// with the drive's words and the steps taken.
type settleArmBackend struct {
	controlBackend
	resets  int
	clients []string
	hot     bool
}

func (b *settleArmBackend) Arm(ctx context.Context, armed bool) (*denseexec.ArmResult, error) {
	if !armed {
		return b.controlBackend.Arm(ctx, armed)
	}
	b.resets++
	b.clients = append(b.clients, denseexec.ClientOf(ctx).Label)
	if b.hot {
		detail := "J5 drive alarm Er41.1 (Motor over-temperature due to locked- rotor) still active — let it cool, then Arm again"
		steps := []denseexec.RecoverStep{{Step: "reset", Outcome: "persists"}, {Step: "reset", Outcome: "refused", Detail: "fault_persists: " + detail}}
		return &denseexec.ArmResult{Steps: steps}, &denseexec.Error{Status: http.StatusConflict, Code: "fault_persists", Detail: detail}
	}
	result, err := b.controlBackend.Arm(ctx, armed)
	result.Steps = []denseexec.RecoverStep{{Step: "reset", Outcome: "done"}}
	return result, err
}

// Arm clears resettable faults for every client, but only for one the
// cross-arm checks admit: a background pendant and a pendant launched for
// another cell are refused before Arm, so nothing is reset for them. A
// refused Arm answers the drive's own sentence and the steps it took.
func TestArmResetsOnlyForAnAdmittedClient(t *testing.T) {
	backend := &settleArmBackend{controlBackend: controlBackend{controllerID: "offline-programming:server-olp001"}}
	server := NewServer(ServerOptions{DenseExecution: backend})
	for _, refused := range []struct {
		client map[string]string
		code   string
	}{
		{deckProcess("949429", armCell, clientBackground), clientNotFocused},
		{deckProcess("955082", otherArmCell, clientFocused), clientCellMismatch},
	} {
		if response, body := controlRequest(t, server, refused.client, "POST", denseExecutionPath+"/arm", `{"armed":true}`); response.Code != http.StatusConflict || body["error"] != refused.code {
			t.Fatalf("%d %s", response.Code, response.Body.String())
		}
	}
	if backend.resets != 0 || backend.armed {
		t.Fatalf("a refused client reached Arm's reset: resets=%d armed=%v", backend.resets, backend.armed)
	}

	backend.hot = true
	visible := deckProcess("1376204", armCell, clientFocused)
	response, body := controlRequest(t, server, visible, "POST", denseExecutionPath+"/arm", `{"armed":true}`)
	steps, _ := body["steps"].([]any)
	if response.Code != http.StatusConflict || body["error"] != "fault_persists" || body["detail"] != "J5 drive alarm Er41.1 (Motor over-temperature due to locked- rotor) still active — let it cool, then Arm again" || len(steps) != 2 {
		t.Fatalf("%d %s", response.Code, response.Body.String())
	}
	backend.hot = false
	if response, body = controlRequest(t, server, visible, "POST", denseExecutionPath+"/arm", `{"armed":true}`); response.Code != http.StatusOK || !backend.armed {
		t.Fatalf("%d %s", response.Code, response.Body.String())
	}
	if steps, _ := body["steps"].([]any); len(steps) != 1 || backend.resets != 2 || backend.clients[1] != "rosie-pendant-v5 (steamdeck-pendant)" {
		t.Fatalf("steps=%v resets=%d clients=%v", body["steps"], backend.resets, backend.clients)
	}
}
