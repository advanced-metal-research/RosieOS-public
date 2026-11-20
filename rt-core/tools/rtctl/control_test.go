//go:build linux

package rtctl

import (
	"bytes"
	"encoding/json"
	"net"
	"net/http"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"rosieos/rt-core/sdk/control"
)

// TestControlCLIUnsentTransportExit preserves transport exit 3 for an absent
// socket without reporting uncertain motion for a request that was never sent.
func TestControlCLIUnsentTransportExit(t *testing.T) {
	dir, err := os.MkdirTemp("", "c-")
	if err != nil {
		t.Fatal(err)
	}
	defer os.RemoveAll(dir)
	for _, operation := range []string{"status", "arm"} {
		var output, stderr bytes.Buffer
		code := RunControlCLI([]string{operation, "--socket", filepath.Join(dir, "absent.sock"), "--json"}, &output, &stderr)
		if code != 3 || output.Len() != 0 || !strings.Contains(stderr.String(), control.ErrConnectionUnsent.Error()) || strings.Contains(stderr.String(), control.ErrTransport.Error()) {
			t.Fatalf("%s: exit=%d stdout=%s stderr=%s", operation, code, &output, &stderr)
		}
	}
}

func TestControlCLIRejectsInvalidOptionsBeforeContact(t *testing.T) {
	for _, args := range [][]string{{"stop", "--socket", "a", "--remote", "b"}, {"stop", "--axis-mask", "4294967296"}, {"stop", "--timeout", "0"}, {"stop", "--cert", "unused"}, {"start", "unexpected"}} {
		var output bytes.Buffer
		if code := RunControlCLI(args, &output, &output); code != 2 {
			t.Fatalf("%v exit %d", args, code)
		}
	}
}

func TestControlCLIRenewsOnlyLongFencedOperations(t *testing.T) {
	for _, operation := range []string{"home", "reset-fault", "halt", "prepare", "start", "program-prepare", "program-start"} {
		if !controlCommandNeedsRenewal(operation) {
			t.Errorf("%s must renew while its request is in flight", operation)
		}
	}
	for _, operation := range []string{"acquire", "renew", "release", "stop", "status", "describe", "events", "enable", "arm", "discard"} {
		if controlCommandNeedsRenewal(operation) {
			t.Errorf("%s must not start an automatic lease keeper", operation)
		}
	}
}

func TestControlCLIPreservesSessionAndRequestID(t *testing.T) {
	dir, err := os.MkdirTemp("", "c-")
	if err != nil {
		t.Fatal(err)
	}
	defer os.RemoveAll(dir)
	socket := filepath.Join(dir, "c.sock")
	listener, err := net.Listen("unix", socket)
	if err != nil {
		t.Fatal(err)
	}
	requests := make(chan map[string]any, 1)
	server := &http.Server{Handler: http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		var request map[string]any
		if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
			t.Error(err)
		}
		requests <- request
		w.Header().Set("Content-Type", "application/json")
		_, _ = w.Write([]byte(`{"schema":"rosie.rt-control.response.v1","operation":"arm","sequence":9}`))
	})}
	go server.Serve(listener)
	defer server.Close()
	var output, stderr bytes.Buffer
	if code := RunControlCLI([]string{"arm", "--socket", socket, "--session", "test", "--generation", "7", "--request-id", "retry-id", "--json"}, &output, &stderr); code != 0 {
		t.Fatalf("exit %d: %s", code, &stderr)
	}
	request := <-requests
	if request["session"] != "test" || request["generation"] != float64(7) || request["request_id"] != "retry-id" {
		t.Fatalf("request: %v", request)
	}
}
