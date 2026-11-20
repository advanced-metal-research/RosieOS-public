//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// TestAdapterBootstrapPredecessorMetrics starts the actual adapter with a
// predecessor file beside its socket, then publishes matching diagnostics only
// after observing the retry. It prevents startup races from becoming fatal.
func TestAdapterBootstrapPredecessorMetrics(t *testing.T) {
	f := newRestartFixture(t)
	alias := filepath.Join(f.dir, "alias")
	// Unverified fixture permissions: socket alias and diagnostics are test-local.
	if err := os.Mkdir(alias, 0700); err != nil {
		t.Fatal(err)
	}
	socket := filepath.Join(alias, "ipc.sock")
	if err := os.Symlink(f.socket, socket); err != nil {
		t.Fatal(err)
	}
	write := func(name string, raw []byte) {
		t.Helper()
		path := filepath.Join(alias, name)
		if err := os.WriteFile(path+".tmp", raw, 0600); err != nil {
			t.Fatal(err)
		}
		if err := os.Rename(path+".tmp", path); err != nil {
			t.Fatal(err)
		}
	}
	stale := f.diagnostic()
	stale.DaemonIncarnation = strings.Repeat("f", 32)
	raw, err := json.Marshal(stale)
	if err != nil {
		t.Fatal(err)
	}
	write("metrics.json", raw)
	description, err := os.ReadFile(filepath.Join(f.dir, "description.json"))
	if err != nil {
		t.Fatal(err)
	}
	write("description.json", description)
	args := append([]string(nil), f.adapterArgs...)
	for i := range args {
		if args[i] == f.socket {
			args[i] = socket
		}
	}
	f.adapter = restartSpawn(t, f.dir, f.adapterBinary, args)
	restartEventually(t, "predecessor diagnostics retry", func() bool {
		log, _ := os.ReadFile(f.adapter.log.Name())
		return strings.Contains(string(log), "native bootstrap not ready, retrying: RTCore diagnostics not yet published")
	})
	restartEventually(t, "matching diagnostics bootstrap", func() bool {
		raw, err := os.ReadFile(filepath.Join(f.dir, "metrics.json"))
		if err != nil {
			return false
		}
		write("metrics.json", raw)
		return f.service.get("/v1/jog/clock", &f.service.clock) == nil
	})
}

// TestEstablishedDiagnosticIncarnationFence refuses replacement diagnostics
// after Dial and never classifies them as a retryable bootstrap publication.
func TestEstablishedDiagnosticIncarnationFence(t *testing.T) {
	f := startCoreWithRole(t, "simulation.json", false)
	state, err := f.client.Status()
	if err != nil {
		t.Fatal(err)
	}
	state.DaemonIncarnation = strings.Repeat("f", 32)
	raw, err := json.Marshal(state)
	if err != nil {
		t.Fatal(err)
	}
	dir := t.TempDir()
	// Unverified fixture permissions: immutable predecessor diagnostics.
	if err := os.WriteFile(filepath.Join(dir, "metrics.json"), raw, 0600); err != nil {
		t.Fatal(err)
	}
	// Reuse the established handshake and mapped identity, with an isolated diagnostic file.
	c := &Client{options: f.options, incarnation: f.client.incarnation, fast: f.client.fast, done: make(chan struct{})}
	c.options.SocketPath = filepath.Join(dir, "ipc.sock")
	_, err = c.Status()
	if !errors.Is(err, errDiagnosticIncarnation) || errors.Is(err, ErrDiagnosticsNotPublished) {
		t.Fatalf("replacement fence: %v", err)
	}
	_, err = c.Wait(context.Background(), func(Status) bool { t.Fatal("predecessor accepted"); return true })
	if !errors.Is(err, errDiagnosticIncarnation) {
		t.Fatalf("wait fence: %v", err)
	}
}
