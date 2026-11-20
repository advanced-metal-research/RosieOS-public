package main

import (
	"context"
	"rosieos/offline-programming/v1/internal/denseexec"
	"net/http"
)

func (server *Server) denseManualIO(w http.ResponseWriter, r *http.Request) {
	if !server.denseExecutionReady(w) {
		return
	}
	var in denseexec.ManualIORequest
	if err := decodeBoundedJSON(w, r, maxDenseRequestBytes, &in); err != nil {
		writeDenseError(w, 400, "io_test_invalid", err.Error())
		return
	}
	controller, ok := server.denseExecution.(interface {
		ManualIO(context.Context, denseexec.ManualIORequest) error
	})
	if !ok {
		writeDenseError(w, 409, "io_test_unavailable", "This controller does not support manual output tests")
		return
	}
	if err := controller.ManualIO(r.Context(), in); err != nil {
		writeDenseFailure(w, "manual I/O", err)
		return
	}
	writeJSON(w, 200, map[string]any{"accepted": true, "sequence": in.Sequence, "physical_feedback": false})
}

func (server *Server) denseBeginManualIO(w http.ResponseWriter, r *http.Request) {
	if !server.denseExecutionReady(w) {
		return
	}
	c, ok := server.denseExecution.(interface {
		BeginManualIO(context.Context) (string, error)
	})
	if !ok {
		writeDenseError(w, 409, "io_test_unavailable", "Manual testing unavailable")
		return
	}
	token, err := c.BeginManualIO(r.Context())
	if err != nil {
		writeDenseFailure(w, "manual I/O", err)
		return
	}
	writeJSON(w, 200, map[string]any{"session": token, "heartbeat_ms": 50, "expires_ms": denseexec.ManualIOSetupLifetime.Milliseconds()})
}
