package main

import (
	"fmt"
	"io"
	"os"
	"strings"
	"testing"
)

// TestMain keeps every test of this package off real robot cells. The deploy
// commands reach a cell's relay and servo host through a handful of seams
// (relay sessions, the cell-lock holder, uploads, fetches, Tailscale SSH);
// a test that forgets to stub one used to run it for real: on 2026-10-07 the
// build-flow test, written before `build --test` took the cell lock, started a
// real lock holder on example-relay (both maintenance notices up, an idle wait
// against the operator) every time the package's tests ran. Here a seam left
// at its default refuses any ssh or tailscale argv, and the lock holder runs a
// local stand-in that only reports the lock held. Tests that install their own
// fakes are unaffected.
func TestMain(m *testing.M) {
	remote, start, upload, fetch, relay := rtCoreConfigRunRemote, cellLockStart, cellDeployUpload, cellDeployFetch, rtCoreRelayExec
	rtCoreConfigRunRemote = func(argv []string, p *rtCoreProgress) (int, error) {
		if testReachesAHost(argv) {
			return -1, fmt.Errorf("test reached a real host (relay session): %s", testArgvHead(argv))
		}
		return remote(argv, p)
	}
	cellLockStart = func(argv []string, p *rtCoreProgress) (*cellLockSession, error) {
		if testReachesAHost(argv) {
			argv = []string{"bash", "-c", `printf '@@locked test-stand-in\n'; cat > /dev/null`}
		}
		return start(argv, p)
	}
	cellDeployUpload = func(argv []string, file string, p *rtCoreProgress) (int, error) {
		if testReachesAHost(argv) {
			return -1, fmt.Errorf("test reached a real host (upload): %s", testArgvHead(argv))
		}
		return upload(argv, file, p)
	}
	cellDeployFetch = func(argv []string, out io.Writer, p *rtCoreProgress) (int, error) {
		if testReachesAHost(argv) {
			return -1, fmt.Errorf("test reached a real host (fetch): %s", testArgvHead(argv))
		}
		return fetch(argv, out, p)
	}
	rtCoreRelayExec = func(argv []string, stdin io.Reader, stdout, stderr io.Writer) error {
		if testReachesAHost(argv) {
			return fmt.Errorf("test reached a real host (tailscale): %s", testArgvHead(argv))
		}
		return relay(argv, stdin, stdout, stderr)
	}
	os.Exit(m.Run())
}

func testReachesAHost(argv []string) bool {
	if len(argv) == 0 {
		return false
	}
	switch argv[0] {
	case "ssh", "tailscale", "scp", "rsync":
		return true
	}
	return false
}

func testArgvHead(argv []string) string {
	if len(argv) > 4 {
		argv = argv[:4]
	}
	return strings.Join(argv, " ")
}

func TestTestsCannotReachARealHost(t *testing.T) {
	argv := rtCoreConfigRemoteArgv("tailscale", "u", "relay", "192.0.2.1", "true")
	if code, err := rtCoreConfigRunRemote(argv, newRTCoreProgress(io.Discard, nil)); err == nil || code != -1 {
		t.Fatalf("relay session: %d %v", code, err)
	}
	s, err := cellLockHold(newRTCoreProgress(io.Discard, nil), cellLockArgv("tailscale", cellDeployContext{}, "exit 9"))
	if err != nil {
		t.Fatalf("lock stand-in: %v", err)
	}
	if !s.release(5e9) {
		t.Fatal("lock stand-in did not end on stdin EOF")
	}
}
