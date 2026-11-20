package main

import (
	"bytes"
	"log"
	"maps"
	"net/http"
	"strings"
	"testing"
)

// steamdeck-pendant's pendants of 2026-10-11 as they now identify themselves: the
// visible one launched for the 1430 cell, and a hidden one (another process on the
// same Deck reading the same controller).
func deckProcess(pid, cell, focus string) map[string]string {
	client := maps.Clone(deckClient)
	client[clientPIDHeader], client[clientExpectCellHeader], client[clientFocusHeader] = pid, cell, focus
	return client
}

const (
	armCell = "example-arm-1420"
	otherArmCell = "other-arm-1420"
)

func captureLog(t *testing.T) *bytes.Buffer {
	t.Helper()
	var buffer bytes.Buffer
	previous, flags := log.Writer(), log.Flags()
	log.SetOutput(&buffer)
	log.SetFlags(0)
	t.Cleanup(func() { log.SetOutput(previous); log.SetFlags(flags) })
	return &buffer
}

// A pendant whose window does not have input focus never arms, jogs, moves,
// homes, loads, plays or connects -- every request controlGate guards -- and
// Stop, Disarm, the heartbeat and ending a hold still pass from it.
func TestBackgroundPendantIsRefusedEverythingThatTakesControlButStop(t *testing.T) {
	server, backend, _ := controlTestServer(t)
	hidden := deckProcess("949429", armCell, clientBackground)
	journal := captureLog(t)
	for _, request := range controlTakingRequests {
		response, body := controlRequest(t, server, hidden, request.method, request.path, request.body)
		if response.Code != http.StatusConflict || body["error"] != clientNotFocused {
			t.Fatalf("%s %s from a background pendant: %d %s", request.method, request.path, response.Code, response.Body.String())
		}
		if detail, _ := body["detail"].(string); !strings.Contains(detail, "in the background") || !strings.Contains(detail, "Stop always works") {
			t.Fatalf("refusal must say why and that Stop works: %q", detail)
		}
	}
	if backend.armed || backend.held {
		t.Fatal("a background pendant armed or took the grant")
	}
	if !strings.Contains(journal.String(), "control: refused POST "+denseExecutionPath+"/arm from rosie-pendant-v5 (steamdeck-pendant) id=deck-0b7e44d1 pid=949429 expect_cell="+armCell+" focus=background from 100.64.0.15:41000: client_not_focused") {
		t.Fatalf("the journal must name the refused process:\n%s", journal.String())
	}

	// The robot armed by the focused pendant: the hidden one may still stop it.
	visible := deckProcess("1376204", armCell, clientFocused)
	if response, _ := controlRequest(t, server, visible, "POST", denseExecutionPath+"/arm", `{"armed":true}`); response.Code != http.StatusOK || !backend.armed {
		t.Fatalf("focused pendant's arm: %d %s", response.Code, response.Body.String())
	}
	for _, request := range []struct{ path, body string }{
		{"/heartbeat", `{"session_id":"ds-1"}`}, {"/jog/end", `{}`}, {"/cartesian/stop", `{}`}, {"/arm", `{"armed":false}`}, {"/stop", `{}`},
	} {
		response, body := controlRequest(t, server, hidden, "POST", denseExecutionPath+request.path, request.body)
		if body["error"] == clientNotFocused {
			t.Fatalf("%s from a background pendant was refused: %d %s", request.path, response.Code, response.Body.String())
		}
	}
	if backend.armed {
		t.Fatal("Stop from a background pendant did not reach the robot")
	}
}

// A pendant launched for one cell is refused arm, motion and Connect on an OLP
// server that drives another (a crossed tunnel, a launcher pointing at the
// wrong port), and admitted on its own cell.
func TestPendantLaunchedForAnotherCellIsRefused(t *testing.T) {
	server, backend, _ := controlTestServer(t)
	journal := captureLog(t)
	wrong := deckProcess("955082", otherArmCell, clientFocused)
	for _, request := range controlTakingRequests {
		if strings.HasSuffix(request.path, "/targets/select") || strings.HasSuffix(request.path, "/target") {
			continue // these name a cell themselves; below
		}
		response, body := controlRequest(t, server, wrong, request.method, request.path, request.body)
		if response.Code != http.StatusConflict || body["error"] != clientCellMismatch || body["cell"] != armCell || body["expect_cell"] != otherArmCell {
			t.Fatalf("%s %s from a pendant for %s: %d %s", request.method, request.path, otherArmCell, response.Code, response.Body.String())
		}
	}
	if backend.armed || backend.held {
		t.Fatal("a pendant launched for another cell armed this one")
	}
	if !strings.Contains(journal.String(), "pid=955082 expect_cell="+otherArmCell+" focus=focused from 100.64.0.15:41000: client_cell_mismatch: rosie-pendant-v5 (steamdeck-pendant) was launched for cell "+otherArmCell+", but this OLP server drives "+armCell) {
		t.Fatalf("journal:\n%s", journal.String())
	}

	// Connecting names the cell: refused for another, admitted (as far as
	// this gate goes) for its own.
	for _, request := range []struct{ path, body string }{
		{"/api/offline-programming/v1/targets/select", `{"cell_id":"` + armCell + `","model_id":"rosie_1420_v1"}`},
		{denseExecutionPath + "/target", `{"cell":"` + armCell + `"}`},
	} {
		if response, body := controlRequest(t, server, wrong, "POST", request.path, request.body); body["error"] != clientCellMismatch {
			t.Fatalf("connect %s from a pendant for %s: %d %s", request.path, otherArmCell, response.Code, response.Body.String())
		}
		mine := deckProcess("955082", otherArmCell, clientFocused)
		body := strings.ReplaceAll(request.body, armCell, otherArmCell)
		if response, decoded := controlRequest(t, server, mine, "POST", request.path, body); decoded["error"] == clientCellMismatch {
			t.Fatalf("connect %s to its own cell was refused: %d %s", request.path, response.Code, response.Body.String())
		}
	}

	right := deckProcess("1376204", armCell, clientFocused)
	if response, _ := controlRequest(t, server, right, "POST", denseExecutionPath+"/arm", `{"armed":true}`); response.Code != http.StatusOK || !backend.armed {
		t.Fatalf("pendant for this cell: %d %s", response.Code, response.Body.String())
	}
	if !strings.Contains(journal.String(), "dense-execution: arm=true requested by rosie-pendant-v5 (steamdeck-pendant) id=deck-0b7e44d1 pid=1376204 expect_cell="+armCell+" focus=focused from 100.64.0.15:41000") {
		t.Fatalf("the arm line must name the process and its launcher's cell:\n%s", journal.String())
	}
	if response, body := controlRequest(t, server, wrong, "POST", denseExecutionPath+"/stop", `{}`); body["error"] == clientCellMismatch || backend.armed {
		t.Fatalf("Stop from a pendant for another cell must pass: %d %s", response.Code, response.Body.String())
	}
}

// Clients that say nothing (a browser, a script, a pendant built before this)
// are governed by the control rule alone; the connected-clients list carries
// what a pendant said on its latest request.
func TestClientIdentityIsOptionalAndListed(t *testing.T) {
	server, backend, _ := controlTestServer(t)
	if response, _ := controlRequest(t, server, browserTab, "POST", denseExecutionPath+"/arm", `{"armed":true}`); response.Code != http.StatusOK || !backend.armed {
		t.Fatalf("browser arm: %d %s", response.Code, response.Body.String())
	}
	controlRequest(t, server, browserTab, "POST", denseExecutionPath+"/stop", `{}`)
	controlRequest(t, server, deckProcess("1376204", armCell, clientBackground), "GET", denseExecutionPath+"/status", "")
	var deck *MaintenanceClient
	for _, client := range server.clients.list() {
		if client.Kind == "pendant" {
			deck = &client
		}
	}
	if deck == nil || deck.PID != 1376204 || deck.ExpectCell != armCell || deck.Focus != clientBackground {
		t.Fatalf("listed pendant: %+v", deck)
	}
	for _, bad := range []map[string]string{
		{clientPIDHeader: "-4", clientExpectCellHeader: "bad cell id", clientFocusHeader: "maybe"},
	} {
		request, _ := http.NewRequest("GET", "/", nil)
		for key, value := range bad {
			request.Header.Set(key, value)
		}
		if identity := clientIdentityOf(request); identity.PID != 0 || identity.ExpectCell != "" || identity.Focus != "" {
			t.Fatalf("malformed identity headers must be ignored: %+v", identity)
		}
	}
}
