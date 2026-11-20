//go:build linux

package control

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
	"time"
)

func runRtctl(t *testing.T, s *service, want int, args ...string) []byte {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	cmd := exec.CommandContext(ctx, filepath.Join(s.root, "build/rtctl"), args...)
	var stdout, stderr bytes.Buffer
	cmd.Stdout, cmd.Stderr = &stdout, &stderr
	err := cmd.Run()
	code := 0
	if err != nil {
		var exit *exec.ExitError
		if !errors.As(err, &exit) {
			t.Fatal(err)
		}
		code = exit.ExitCode()
	}
	if code != want {
		t.Fatalf("rtctl %v: exit %d want %d; %s %s", args, code, want, &stdout, &stderr)
	}
	if code != 0 {
		return stderr.Bytes()
	}
	if !json.Valid(stdout.Bytes()) {
		t.Fatalf("invalid JSON: %s", &stdout)
	}
	return stdout.Bytes()
}

func TestRtctlControlCommandsExitCodes(t *testing.T) {
	s := startService(t)
	runRtctl(t, s, 0, "describe", "--socket", s.socket, "--json")
	runRtctl(t, s, 0, "status", "--socket", s.socket, "--json")
	raw := runRtctl(t, s, 0, "acquire", "--socket", s.socket, "--pair-id", "bench", "--pair-revision", "7", "--configuration-sha256", s.hash, "--json")
	var response Response
	must(t, json.Unmarshal(raw, &response))
	grant, err := data[Grant](response, nil)
	must(t, err)
	s.client.SetFence(Fence{Session: grant.Session, Generation: grant.Generation})
	_, err = s.client.StartRenewal(context.Background(), 50*time.Millisecond)
	must(t, err)
	call := func(code int, command string, extra ...string) []byte {
		fence := s.client.Fence()
		args := []string{command, "--socket", s.socket, "--session", fence.Session, "--generation", strconv.FormatUint(fence.Generation, 10), "--json"}
		return runRtctl(t, s, code, append(args, extra...)...)
	}
	call(0, "enable", "--axis-mask", "511")
	call(0, "arm")
	s.waitStatus(t, func(v Status) bool { return v.Core.Armed == 1 })
	pointsFile := filepath.Join(s.build, "points.json")
	must(t, os.WriteFile(pointsFile, []byte(`[{"time_ns":0,"position":[0,0,0,0,0,0,0,0,0]},{"time_ns":50000000,"position":[0,0,0,0,0,0,0,0,0]}]`), 0600))
	raw = call(0, "prepare", "--axis-mask", "511", "--file", pointsFile)
	must(t, json.Unmarshal(raw, &response))
	call(0, "discard", "--handle", strconv.FormatUint(response.Handle, 10))
	raw = call(0, "prepare", "--axis-mask", "511", "--file", pointsFile)
	must(t, json.Unmarshal(raw, &response))
	raw = call(0, "start", "--handle", strconv.FormatUint(response.Handle, 10))
	must(t, json.Unmarshal(raw, &response))
	startedSequence := response.Sequence
	s.waitStatus(t, func(v Status) bool { return v.Motion.Done && v.Motion.CommandSequence == startedSequence })
	raw = call(2, "start", "--handle", "999999")
	if !strings.Contains(string(raw), "unknown_handle") {
		t.Fatalf("lost rejection reason: %s", raw)
	}
	programFile := filepath.Join(s.build, "program.rdt")
	must(t, os.WriteFile(programFile, oracleProgram(t, s.root, false), 0600))
	raw = call(0, "program-prepare", "--file", programFile)
	var program Program
	must(t, json.Unmarshal(raw, &program))
	identityFile := filepath.Join(s.build, "identity.json")
	raw, err = json.Marshal(program.Identity)
	must(t, err)
	must(t, os.WriteFile(identityFile, raw, 0600))
	call(0, "program-start", "--file", identityFile)
	s.waitStatus(t, func(v Status) bool { return v.Execution.State == "completed" })
	raw = call(0, "stop")
	must(t, json.Unmarshal(raw, &response))
	grant, err = data[Grant](response, nil)
	must(t, err)
	s.client.StopRenewal()
	s.client.SetFence(Fence{Session: grant.Session, Generation: grant.Generation})
	call(0, "release")
	raw = runRtctl(t, s, 3, "status", "--socket", filepath.Join(s.dir, "absent.sock"), "--json")
	if !strings.Contains(string(raw), ErrConnectionUnsent.Error()) || strings.Contains(string(raw), ErrTransport.Error()) {
		t.Fatalf("lost unsent transport reason: %s", raw)
	}
}
