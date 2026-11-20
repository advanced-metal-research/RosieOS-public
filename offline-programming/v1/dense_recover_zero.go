package main

import (
	"errors"
	"net/http"

	"rosieos/offline-programming/v1/internal/denseexec"
)

// denseExecutionRecoverToZero is POST .../dense-execution/recover-to-zero,
// the one-press recovery behind All joints to 0° (denseexec/recover_zero_plan.go).
// The route is a robot action like any other: the control gate admits only
// the client that may control the robot (and makes it the controller), the
// maintenance gate refuses it during an update and the target fence pins it
// to the displayed cell, all before this handler runs.
// 200: state done, at_zero or confirm_arm. 409: the step that did not clear,
// {"error", "detail", "recovery"}; detail is the operator's sentence.
func (server *Server) denseExecutionRecoverToZero(w http.ResponseWriter, r *http.Request) {
	if !server.denseExecutionReady(w) {
		return
	}
	var input denseexec.RecoverToZeroRequest
	if err := decodeBoundedJSON(w, r, maxDenseRequestBytes, &input); err != nil {
		writeDenseError(w, http.StatusBadRequest, "recover_to_zero_invalid", `request needs {"fraction": 0..1, "confirm_arm": bool, "confirmed": [...]}`)
		return
	}
	backend, ok := server.denseExecution.(denseexec.RecoverToZeroBackend)
	if !ok {
		writeDenseError(w, http.StatusConflict, "capability_unimplemented", "Recover to zero needs an rt-core cell")
		return
	}
	result, err := backend.RecoverToZero(r.Context(), input)
	if err == nil && result != nil {
		writeJSON(w, http.StatusOK, result)
		return
	}
	var denseErr *denseexec.Error
	if result == nil || !errors.As(err, &denseErr) {
		writeDenseFailure(w, "recover-to-zero", err)
		return
	}
	writeJSON(w, denseErr.Status, map[string]any{"error": denseErr.Code, "detail": denseErr.Detail, "recovery": result})
}
