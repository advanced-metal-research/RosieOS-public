package main

import (
	"rosieos/offline-programming/v1/internal/denseexec"
	"net/http"
)

func (server *Server) denseTrace(w http.ResponseWriter, r *http.Request) {
	controller, ok := server.denseExecution.(denseexec.TraceBackend)
	if !ok {
		writeDenseError(w, http.StatusConflict, "trace_unavailable", "The connected OLP does not support waypoint trace")
		return
	}
	if r.Method == http.MethodGet {
		writeJSON(w, http.StatusOK, controller.JointJogState())
		return
	}
	var in denseexec.TraceIntent
	if err := decodeBoundedJSON(w, r, maxDenseLoadRequestBytes, &in); err != nil {
		writeDenseError(w, http.StatusBadRequest, "trace_invalid", err.Error())
		return
	}
	state, err := controller.Trace(r.Context(), r.PathValue("action"), in)
	if err != nil {
		writeDenseFailure(w, "trace", err)
		return
	}
	writeJSON(w, http.StatusOK, state)
}
