package main

import (
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"strconv"
	"strings"
	"syscall"
	"testing"
	"time"
)

func tunnelWaitState(t *testing.T, path string, limit time.Duration, ok func(tunnelState) bool) tunnelState {
	t.Helper()
	deadline := time.Now().Add(limit)
	var s tunnelState
	for time.Now().Before(deadline) {
		if got, err := readTunnelState(path); err == nil {
			s = got
			if ok(s) {
				return s
			}
		}
		time.Sleep(20 * time.Millisecond)
	}
	raw, _ := os.ReadFile(path)
	log, _ := os.ReadFile(filepath.Join(filepath.Dir(path), "tunnel.log"))
	t.Fatalf("tunnel state never matched in %s; last:\n%s\nlog:\n%s", limit, raw, log)
	return s
}

func tunnelPidAlive(pid int) bool {
	return pid > 0 && syscall.Kill(pid, 0) == nil
}

// The supervisor against a fake `tailscale ssh`, shaped like the real one (a
// wrapper whose child is the ssh): a failed connect is retried and recorded
// with ssh's own message, a killed ssh comes back within the backoff, failures
// back off doubling to the cap, and stopping the supervisor takes the whole
// attempt (wrapper and ssh) with it.
func TestTunnelSupervisorRestartsSSH(t *testing.T) {
	if runtime.GOOS == "windows" {
		t.Skip("bash supervisor")
	}
	dir := t.TempDir()
	mode := filepath.Join(dir, "mode")
	inner := filepath.Join(dir, "inner.pid")
	fake := filepath.Join(dir, "tailscale")
	_ = os.WriteFile(mode, []byte("timeout\n"), 0o600)
	_ = os.WriteFile(fake, []byte(`#!/bin/bash
m=$(head -n 1 `+shellQuote(mode)+`)
case "$m" in
  timeout) echo "Timeout, server example-relay.example.ts.net. not responding." >&2; exit 255 ;;
  refuse)  echo "ssh: connect to host example-relay port 22: Connection refused" >&2; exit 255 ;;
  up)      exec 2>/dev/null; sleep 1000 & echo $! >`+shellQuote(inner)+`; wait $!; exit $? ;;
esac
`), 0o700)
	c := olpUITestConfig(dir)
	c.TunnelBin = fake
	c.backoffMinMS, c.backoffMaxMS, c.healthyAfterS = 100, 400, 2
	script := filepath.Join(dir, "tunnel.sh")
	_ = os.WriteFile(script, []byte(tunnelSupervisorScript(olpUITunnelSpec(c))), 0o700)
	sup := exec.Command("/bin/bash", script)
	if err := sup.Start(); err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = sup.Process.Kill(); _ = sup.Wait() })
	statePath := filepath.Join(dir, "tunnel.state")

	s := tunnelWaitState(t, statePath, 5*time.Second, func(s tunnelState) bool { return s.Restarts >= 1 })
	if s.LastRC != "255" || !strings.Contains(s.LastReason, "Timeout, server example-relay.example.ts.net. not responding.") {
		t.Fatalf("restart reason not recorded: %+v", s)
	}
	_ = os.WriteFile(mode, []byte("up\n"), 0o600)
	up := tunnelWaitState(t, statePath, 5*time.Second, func(s tunnelState) bool { return s.Phase == "up" && s.Attempt >= 2 })
	var innerPID int
	deadline := time.Now().Add(3 * time.Second)
	for time.Now().Before(deadline) {
		raw, _ := os.ReadFile(inner)
		if innerPID, _ = strconv.Atoi(strings.TrimSpace(string(raw))); innerPID > 0 {
			break
		}
		time.Sleep(20 * time.Millisecond)
	}
	// Stay up past healthy_after_s, then kill the inner ssh as the operator
	// would: the retry comes after the minimum delay.
	time.Sleep(2100 * time.Millisecond)
	killed := time.Now()
	if err := syscall.Kill(innerPID, syscall.SIGTERM); err != nil {
		t.Fatalf("kill inner ssh %d: %v", innerPID, err)
	}
	back := tunnelWaitState(t, statePath, 3*time.Second, func(s tunnelState) bool { return s.Phase == "up" && s.Attempt > up.Attempt })
	if took := time.Since(killed); took > 1500*time.Millisecond {
		t.Errorf("tunnel took %s to come back after the kill", took)
	}
	if back.LastRC != "143" || back.LastReason != "ssh ended by signal 15" {
		t.Errorf("kill not recorded as such: %+v", back)
	}

	// Repeated failures after a healthy run: the delay doubles from the
	// minimum to the cap.
	_ = os.WriteFile(mode, []byte("refuse\n"), 0o600)
	time.Sleep(2100 * time.Millisecond)
	raw, _ := os.ReadFile(inner)
	innerPID, _ = strconv.Atoi(strings.TrimSpace(string(raw)))
	_ = syscall.Kill(innerPID, syscall.SIGTERM)
	// tunnel.state is written just before each "retry in" log line: wait for
	// one restart more than the lines read.
	tunnelWaitState(t, statePath, 6*time.Second, func(s tunnelState) bool { return s.Restarts >= back.Restarts+6 })
	log, _ := os.ReadFile(filepath.Join(dir, "tunnel.log"))
	var delays []string
	for _, line := range strings.Split(string(log), "\n") {
		if strings.Contains(line, "Connection refused; retry in ") {
			delays = append(delays, line[strings.LastIndex(line, "retry in ")+len("retry in "):])
		}
	}
	if len(delays) < 4 || strings.Join(delays[:4], ",") != "200 ms,400 ms,400 ms,400 ms" {
		t.Errorf("backoff after a short run: %q (want the delay after the kill's 100 ms to double to the 400 ms cap)", delays)
	}

	waitInner := func() int {
		deadline := time.Now().Add(5 * time.Second)
		for time.Now().Before(deadline) {
			raw, _ := os.ReadFile(inner)
			if pid, _ := strconv.Atoi(strings.TrimSpace(string(raw))); pid > 0 {
				return pid
			}
			time.Sleep(20 * time.Millisecond)
		}
		t.Fatal("the fake ssh never came up")
		return 0
	}

	// A supervisor killed outright (SIGKILL: no trap) leaves its attempt in
	// its own process group; the one launchd starts next ends it first.
	_ = os.WriteFile(mode, []byte("up\n"), 0o600)
	_ = os.Remove(inner)
	orphanInner := waitInner()
	orphan := tunnelWaitState(t, statePath, 2*time.Second, func(s tunnelState) bool { return s.Phase == "up" })
	_ = sup.Process.Kill()
	_ = sup.Wait()
	if !tunnelPidAlive(orphanInner) {
		t.Fatal("test premise: the attempt should outlive a SIGKILLed supervisor")
	}
	_ = os.Remove(inner)
	sup = exec.Command("/bin/bash", script)
	if err := sup.Start(); err != nil {
		t.Fatal(err)
	}
	innerPID = waitInner()
	final := tunnelWaitState(t, statePath, 2*time.Second, func(s tunnelState) bool { return s.Phase == "up" && s.SupervisorPID == sup.Process.Pid })
	if tunnelPidAlive(orphan.SSHPID) || tunnelPidAlive(orphanInner) {
		t.Errorf("the new supervisor left the previous attempt running: wrapper %d, ssh %d", orphan.SSHPID, orphanInner)
	}

	// Stop: the supervisor ends the running attempt's whole process group.
	_ = sup.Process.Signal(syscall.SIGTERM)
	done := make(chan error, 1)
	go func() { done <- sup.Wait() }()
	select {
	case <-done:
	case <-time.After(3 * time.Second):
		t.Fatal("supervisor did not exit on SIGTERM")
	}
	time.Sleep(100 * time.Millisecond)
	if tunnelPidAlive(final.SSHPID) || tunnelPidAlive(innerPID) {
		t.Errorf("stop left the attempt running: wrapper %d alive=%v, ssh %d alive=%v", final.SSHPID, tunnelPidAlive(final.SSHPID), innerPID, tunnelPidAlive(innerPID))
	}
	if s, _ := readTunnelState(statePath); s.Phase != "stopped" {
		t.Errorf("state after stop: %+v", s)
	}
}
