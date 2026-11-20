package main

import (
	"net"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/programstore"
)

// TestCellProgramsComposedBrowserAndDeck runs the browser's and the Steam
// Deck's real sync code (ui/test/composed/cell-programs.ts) against this
// server's real handler on a real port: the Deck's program becomes current,
// the browser opens it, each follows the other's edits, a simultaneous edit
// leaves the cell with the first save and the second as a conflict kept as a
// copy; then the server restarts on the same store and a restarted Deck finds
// everything as it was. Needs node and ui/node_modules; skipped without them.
func TestCellProgramsComposedBrowserAndDeck(t *testing.T) {
	node, err := exec.LookPath("node")
	if err != nil {
		t.Skip("node is not installed")
	}
	if _, err := os.Stat(filepath.Join("ui", "node_modules", "happy-dom")); err != nil {
		t.Skip("ui/node_modules is not installed (npm ci in ui/)")
	}
	dir := filepath.Join(t.TempDir(), "programs")
	state := filepath.Join(t.TempDir(), "state.json")
	listener, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	addr := listener.Addr().String()
	serve := func(listener net.Listener) (*http.Server, *programstore.Store) {
		store, err := programstore.Open(dir)
		if err != nil {
			t.Fatal(err)
		}
		server := &http.Server{Handler: NewServer(ServerOptions{ProgramStore: store, StoreLabel: "example-relay"}).Handler(), ReadHeaderTimeout: 5 * time.Second}
		go server.Serve(listener)
		return server, store
	}
	phase := func(name string) {
		cmd := exec.Command(node, "--experimental-strip-types", "--no-warnings", "--import", "./test/support/seed.ts",
			"test/composed/cell-programs.ts", name, "http://"+addr, state)
		cmd.Dir = "ui"
		out, err := cmd.CombinedOutput()
		t.Logf("%s phase:\n%s", name, out)
		if err != nil {
			t.Fatalf("%s phase: %v", name, err)
		}
	}

	server, store := serve(listener)
	phase("edit")
	server.Close()
	store.Close()

	listener, err = net.Listen("tcp", addr)
	if err != nil {
		t.Fatal(err)
	}
	server, store = serve(listener)
	defer func() { server.Close(); store.Close() }()
	phase("restart")
}
