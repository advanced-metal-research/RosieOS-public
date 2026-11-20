package main

import (
	"context"
	"log"
	"net/http"
	"os"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/simulator"
)

// cartesianResolver creates a persistent model-pinned service at Connect.
// No request carries a controller address or authority into the resolver.
func cartesianResolver(repo string) denseexec.CartesianResolverFactory {
	return func(model string) denseexec.CartesianResolverService {
		// OLP_CARTESIAN_RESOLVER may name a command line ("ssh user@cell
		// /path/robot-v4-cartesiand") so the cell's own motion server resolves
		// the kinematics; OLP_CARTESIAN_RESOLVER_REPO is then the tree on that
		// host. Unset, the canonical local jog binary resolves against this repo.
		binary := os.Getenv("OLP_CARTESIAN_RESOLVER")
		if binary == "" {
			binary, _ = simulator.ResolveCanonicalJogBinary(repo)
		}
		resolverRepo := os.Getenv("OLP_CARTESIAN_RESOLVER_REPO")
		if resolverRepo == "" {
			resolverRepo = repo
		}
		return denseexec.NewCartesianProcess(binary, resolverRepo, model)
	}
}

// denseCartesianJog requires the selected controller and its browser fences;
// machine failures never fall back to an isolated preview motion request.
func (server *Server) denseCartesianJog(w http.ResponseWriter, r *http.Request) {
	controller, ok := server.denseExecution.(denseexec.CartesianJogBackend)
	if !ok {
		writeDenseError(w, http.StatusConflict, "dense_target_unavailable", "Connect and arm the selected machine before Cartesian jog")
		return
	}
	if r.Method == http.MethodGet {
		writeJSON(w, http.StatusOK, controller.JointJogState())
		return
	}
	var in denseexec.CartesianJogIntent
	if err := decodeBoundedJSON(w, r, maxDenseRequestBytes, &in); err != nil {
		writeDenseError(w, http.StatusBadRequest, "cartesian_input_invalid", err.Error())
		return
	}
	state, err := controller.CartesianJog(r.Context(), r.PathValue("action"), in)
	if err != nil {
		writeDenseFailure(w, "cartesian", err)
		return
	}
	writeJSON(w, http.StatusOK, state)
}

// denseCartesianMove returns one correlated completion, so an HTTP receipt
// alone cannot label an unfinished machine move as done.
func (server *Server) denseCartesianMove(w http.ResponseWriter, r *http.Request) {
	controller, ok := server.denseExecution.(denseexec.CartesianMoveBackend)
	if !ok {
		writeDenseError(w, http.StatusConflict, "dense_target_unavailable", "Connect and arm the selected machine before a Cartesian move")
		return
	}
	var in denseexec.CartesianMoveIntent
	if err := decodeBoundedJSON(w, r, maxDenseRequestBytes, &in); err != nil {
		writeDenseError(w, http.StatusBadRequest, "cartesian_input_invalid", err.Error())
		return
	}
	// The controller owns accepted motion. Record loss of its HTTP observer
	// without translating that loss into an operator Stop.
	finished := context.AfterFunc(r.Context(), func() {
		log.Printf("dense-execution: cartesian observer cancelled: client_disconnected: session=%s revision=%d", in.SessionID, in.Revision)
	})
	defer finished()
	state, err := controller.CartesianMove(r.Context(), in)
	if err != nil {
		writeDenseFailure(w, "cartesian", err)
		return
	}
	writeJSON(w, http.StatusOK, state)
}
