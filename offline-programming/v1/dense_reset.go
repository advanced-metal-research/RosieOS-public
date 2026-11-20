package main

import (
	"context"
	"encoding/json"
	"net/http"

	"rosieos/offline-programming/v1/internal/denseexec"
)

func (server *Server) denseExecutionResetFault(w http.ResponseWriter, r *http.Request) {
	if !server.denseExecutionReady(w) {
		return
	}
	var input map[string]json.RawMessage
	if err := decodeBoundedJSON(w, r, maxDenseRequestBytes, &input); err != nil || input == nil || len(input) != 0 {
		writeDenseError(w, 400, "reset_fault_invalid", "reset needs an empty JSON object")
		return
	}
	reset, ok := server.denseExecution.(interface {
		ResetFault(context.Context) (*denseexec.ArmResult, error)
	})
	if !ok {
		writeDenseError(w, 409, "reset_fault_unsupported", "selected backend does not support disarmed fault reset")
		return
	}
	result, err := reset.ResetFault(r.Context())
	if result == nil {
		writeDenseFailure(w, "reset-fault", err)
		return
	}
	// Completed refusals carry the observed native recovery outcomes to the UI.
	if err != nil {
		result.Accepted = false
		result.Error = err.Error()
	}
	writeJSON(w, http.StatusOK, result)
}
