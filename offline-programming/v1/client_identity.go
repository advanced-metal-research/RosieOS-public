// Which pendant process sent a request, and whether it may start motion.
//
// 2026-10-11 18:43-18:50 PDT (rt-core/docs/history/2026-10-11-pendant-crossarm.md):
// A Steam Deck ran four pendant processes at once, two for one Rosie 1430 arm
// and two for another. Every one of them reads the same physical controller,
// so each Arm the owner pressed on the second arm's visible pendant also armed
// the first arm through a hidden one. This server logged "arm=true requested by 127.0.0.1:36054": every Deck
// reaches it through an SSH tunnel, so the peer address named nothing, and
// nothing said which process, which cell it was launched for or whether
// anyone could see it.
//
// A pendant now sends, on every request:
//   - X-Rosie-Client-Pid: its process id on its host (the label names the host);
//   - X-Rosie-Client-Expect-Cell: the cell id its launcher was started for;
//   - X-Rosie-Client-Focus: "focused" while its window has input focus,
//     "background" otherwise.
//
// The rule, for every request that takes control or starts motion (the same
// set controlGate guards): a client that says it is in the background is
// refused 409 client_not_focused, and a client whose expected cell is not the
// cell this server has selected (or, for a select, the cell it asks for) is
// refused 409 client_cell_mismatch. Stop, Pause, Disarm, Disconnect, the
// heartbeat and ending a hold always pass. A client that sends none of the
// headers (a browser, a script, a pendant built before this) is not affected;
// they are a check against a confused client, not authentication.
package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net/http"
	"strconv"
	"strings"
)

const (
	clientPIDHeader        = "X-Rosie-Client-Pid"
	clientExpectCellHeader = "X-Rosie-Client-Expect-Cell"
	clientFocusHeader      = "X-Rosie-Client-Focus"
	clientFocused          = "focused"
	clientBackground       = "background"
	clientNotFocused       = "client_not_focused"
	clientCellMismatch     = "client_cell_mismatch"
)

// clientIdentity is everything a request says about its sender, for the
// journal and the gate below.
type clientIdentity struct {
	Label, ID, Address string
	PID                int
	ExpectCell, Focus  string
}

func clientIdentityOf(r *http.Request) clientIdentity {
	client := controlClientOf(r)
	identity := clientIdentity{Label: client.Label, Address: r.RemoteAddr}
	if id := strings.TrimSpace(r.Header.Get(clientIDHeader)); validClientID(id) {
		identity.ID = id
	}
	if pid, err := strconv.Atoi(strings.TrimSpace(r.Header.Get(clientPIDHeader))); err == nil && pid > 0 {
		identity.PID = pid
	}
	if cell := strings.TrimSpace(r.Header.Get(clientExpectCellHeader)); validClientID(cell) {
		identity.ExpectCell = cell
	}
	switch focus := strings.TrimSpace(r.Header.Get(clientFocusHeader)); focus {
	case clientFocused, clientBackground:
		identity.Focus = focus
	}
	return identity
}

// String is the journal's words for the sender: "rosie-pendant-v5
// (<deck>) id=deck-0b7e44d1 pid=1376204 expect_cell=<cell>
// focus=focused from 127.0.0.1:36054". Fields the client did not send are left out.
func (c clientIdentity) String() string {
	parts := []string{c.Label}
	if c.ID != "" {
		parts = append(parts, "id="+c.ID)
	}
	if c.PID > 0 {
		parts = append(parts, "pid="+strconv.Itoa(c.PID))
	}
	if c.ExpectCell != "" {
		parts = append(parts, "expect_cell="+c.ExpectCell)
	}
	if c.Focus != "" {
		parts = append(parts, "focus="+c.Focus)
	}
	return strings.Join(parts, " ") + " from " + c.Address
}

// requestedCell is the cell a select or target request asks for ("" for any
// other request, or a disconnect). The body is read and restored.
func requestedCell(w http.ResponseWriter, r *http.Request, action string) (string, error) {
	if action != "select" && action != "target" {
		return "", nil
	}
	raw, err := io.ReadAll(http.MaxBytesReader(w, r.Body, maxDenseRequestBytes))
	if err != nil {
		return "", err
	}
	r.Body = io.NopCloser(bytes.NewReader(raw))
	var input struct {
		CellID string `json:"cell_id"`
		Cell   string `json:"cell"`
	}
	if json.Unmarshal(raw, &input) != nil {
		return "", nil
	}
	return strings.TrimSpace(firstNonEmpty(input.CellID, input.Cell)), nil
}

// selectedCellID is the cell this server drives now: the picker's selection,
// else the status target; "" when none is selected.
func (server *Server) selectedCellID(r *http.Request) string {
	if picker, ok := server.denseExecution.(interface{ SelectedCell() string }); ok {
		return picker.SelectedCell()
	}
	if status := server.denseStatus(r.Context()); status != nil && status.Target != nil {
		return status.Target.CellID
	}
	return ""
}

// clientRefusal applies the rule above to a request that takes control or
// starts motion; nil admits it.
func (server *Server) clientRefusal(w http.ResponseWriter, r *http.Request, action string, identity clientIdentity) (map[string]any, error) {
	if identity.Focus == clientBackground {
		return map[string]any{"error": clientNotFocused, "detail": fmt.Sprintf(
			"%s is in the background (its window does not have input focus): a hidden pendant never arms or moves the robot. Bring it to the front to %s. Stop always works.",
			identity.Label, action)}, nil
	}
	if identity.ExpectCell == "" {
		return nil, nil
	}
	cell, err := requestedCell(w, r, action)
	if err != nil {
		return nil, err
	}
	if cell == "" {
		cell = server.selectedCellID(r)
	}
	if cell == "" || cell == identity.ExpectCell {
		return nil, nil
	}
	return map[string]any{"error": clientCellMismatch, "expect_cell": identity.ExpectCell, "cell": cell, "detail": fmt.Sprintf(
		"%s was launched for cell %s, but this OLP server drives %s: nothing was sent. Start the pendant's launcher for %s, or check its tunnel. Stop always works.",
		identity.Label, identity.ExpectCell, cell, cell)}, nil
}

// logClientRefusal journals a refusal with everything the sender said.
func logClientRefusal(r *http.Request, identity clientIdentity, refusal map[string]any) {
	log.Printf("control: refused %s %s from %s: %s: %s", r.Method, r.URL.Path, identity, refusal["error"], refusal["detail"])
}
