package main

import (
	"context"
	"errors"
	"net/http"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
)

// denseExecutionRunProgram is POST .../dense-execution/run-program, the one
// press behind Run program and Run again (denseexec/run_program_plan.go):
// Go to plan start when the arm is elsewhere, Load unless the plan is
// staged, Arm when disarmed, Play. Like recover-to-zero it is a robot action:
// the control gate admits only the client that may control the robot (the
// focused pendant of the displayed cell, or the browser), the maintenance
// gate refuses it during an update and the target fence pins it to the
// displayed cell, all before this handler runs.
// 200: state playing or confirm. 409: the step that did not clear,
// {"error", "detail", "run"}; detail is the operator's sentence.
func (server *Server) denseExecutionRunProgram(w http.ResponseWriter, r *http.Request) {
	if !server.denseExecutionReady(w) {
		return
	}
	var input denseexec.RunProgramRequest
	// The kept plan may ride along, as on Load, when the planner lost it.
	if err := decodeBoundedJSON(w, r, maxDenseLoadRequestBytes, &input); err != nil {
		writeDenseError(w, http.StatusBadRequest, "run_program_invalid", `request needs the Load body plus {"fraction": 0..1, "confirm_arm": bool, "confirm_move": bool, "confirmed": [...]}`)
		return
	}
	backend, ok := server.denseExecution.(denseexec.RunProgramBackend)
	if !ok {
		writeDenseError(w, http.StatusConflict, "capability_unimplemented", "Run program needs an rt-core cell")
		return
	}
	// Before every further cycle of a loop: a maintenance notice ends it at
	// the program start (a notice never stops a running cycle by itself).
	input.Continue = func() string {
		if view := server.maintenanceView(context.Background()); view.Active {
			return maintenanceRefusalText(view, time.Now())
		}
		return ""
	}
	result, err := backend.RunProgram(r.Context(), input)
	if result != nil && result.SessionID != "" {
		// A press that staged the program holds it on the robot as Load does,
		// even when a later step (Arm, Play) refused.
		server.programs.noteLoaded(input.ProgramID, result.SessionID)
	}
	if err == nil && result != nil {
		writeJSON(w, http.StatusOK, result)
		return
	}
	var denseErr *denseexec.Error
	if result == nil || !errors.As(err, &denseErr) {
		writeDenseFailure(w, "run-program", err)
		return
	}
	writeJSON(w, denseErr.Status, map[string]any{"error": denseErr.Code, "detail": denseErr.Detail, "run": result})
}

// denseExecutionFinishLoop is POST .../dense-execution/run-program/finish:
// Finish cycle. The running loop ends at the program start after the current
// cycle; only the client that started an attended loop may ask. It passes the
// maintenance and control gates like Stop: it only ends motion sooner.
func (server *Server) denseExecutionFinishLoop(w http.ResponseWriter, r *http.Request) {
	if !server.denseExecutionReady(w) {
		return
	}
	backend, ok := server.denseExecution.(interface {
		FinishLoop(context.Context) (*denseexec.LoopStatus, error)
	})
	if !ok {
		writeDenseError(w, http.StatusConflict, "capability_unimplemented", "Loops need an rt-core cell")
		return
	}
	loop, err := backend.FinishLoop(r.Context())
	if err != nil {
		var denseErr *denseexec.Error
		if errors.As(err, &denseErr) {
			writeJSON(w, denseErr.Status, map[string]any{"error": denseErr.Code, "detail": denseErr.Detail, "loop": loop})
			return
		}
		writeDenseFailure(w, "run-program/finish", err)
		return
	}
	writeJSON(w, http.StatusOK, map[string]any{"loop": loop})
}
