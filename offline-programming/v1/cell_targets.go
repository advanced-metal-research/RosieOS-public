package main

import (
	"errors"
	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/targets"
	"log"
	"net/http"
)

func (server *Server) cellTargets(w http.ResponseWriter, r *http.Request) {
	picker, ok := server.denseExecution.(*targets.Picker)
	if !ok {
		writeJSON(w, http.StatusOK, targets.TargetList{Targets: []targets.CellView{}})
		return
	}
	writeJSON(w, http.StatusOK, picker.List(r.Context()))
}

// selectCellTarget accepts intent only; the picker verifies public identity and
// releases existing authority before committing a process-wide selection.
func (server *Server) selectCellTarget(w http.ResponseWriter, r *http.Request) {
	picker, ok := server.denseExecution.(*targets.Picker)
	if !ok {
		writeDenseError(w, http.StatusServiceUnavailable, "cell_catalogue_unavailable", "cell catalogue is not configured")
		return
	}
	var input targets.Selection
	if err := decodeBoundedJSON(w, r, maxDenseRequestBytes, &input); err != nil || input.CellID == "" || input.ModelID == "" {
		writeDenseError(w, http.StatusBadRequest, "cell_selection_invalid", "cell_id and model_id required")
		return
	}
	log.Printf("targets: select %s requested by %s", input.CellID, clientIdentityOf(r))
	view, err := picker.Select(r.Context(), input)
	if err != nil {
		code := "cell_unreachable"
		var failure *denseexec.Error
		if errors.As(err, &failure) {
			code = failure.Code
		}
		writeDenseError(w, http.StatusConflict, code, code)
		return
	}
	// Selection owns the pad now. Retire the preview process before returning
	// Connect, preventing its grant from competing with the selected local cell.
	if server.jogManager != nil {
		server.jogManager.Stop()
	}
	writeJSON(w, http.StatusOK, map[string]any{"selected": input, "target": view})
}
