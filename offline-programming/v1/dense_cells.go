package main

import (
	"net/http"

	"rosieos/offline-programming/v1/internal/denseexec"
)

func (server *Server) denseExecutionCells(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, http.StatusOK, server.denseCells.Cells(r.Context()))
}

// denseExecutionAddCell validates commissioned metadata through the Picker,
// preventing the dialog from redirecting a listener or replacing identity pins.
func (server *Server) denseExecutionAddCell(w http.ResponseWriter, r *http.Request) {
	var input denseexec.CellSpec
	if err := decodeBoundedJSON(w, r, maxDenseRequestBytes, &input); err != nil {
		writeDenseError(w, http.StatusBadRequest, "dense_cell_invalid", "cell is not valid JSON")
		return
	}
	if err := server.denseCells.Registry().Add(input); err != nil {
		writeDenseFailure(w, "add cell", err)
		return
	}
	writeJSON(w, http.StatusOK, server.denseCells.Cells(r.Context()))
}
func (server *Server) denseExecutionRemoveCell(w http.ResponseWriter, r *http.Request) {
	if r.PathValue("id") == server.denseCells.ActiveID() {
		writeDenseError(w, http.StatusConflict, "dense_cell_active", "disconnect the selected cell first")
		return
	}
	writeDenseFailure(w, "remove cell", server.denseCells.Registry().Remove(r.PathValue("id")))
}

// denseExecutionSelectTarget shares native discovery, Stop/Release and generation
// invalidation with /targets/select. An empty cell clears under admission.
func (server *Server) denseExecutionSelectTarget(w http.ResponseWriter, r *http.Request) {
	var input struct {
		Cell *string `json:"cell"`
	}
	if err := decodeBoundedJSON(w, r, maxDenseRequestBytes, &input); err != nil || input.Cell == nil {
		writeDenseError(w, http.StatusBadRequest, "dense_target_invalid", "cell is required")
		return
	}
	if err := server.denseCells.Select(r.Context(), *input.Cell); err != nil {
		writeDenseFailure(w, "select cell", err)
		return
	}
	if server.jogManager != nil {
		server.jogManager.Stop()
	}
	writeJSON(w, http.StatusOK, server.denseCells.Capabilities(r.Context()))
}
