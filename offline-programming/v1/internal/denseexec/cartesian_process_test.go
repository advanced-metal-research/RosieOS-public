//go:build linux

package denseexec

import (
	"bufio"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"syscall"
	"testing"
	"time"
)

// The helper speaks the real pipe protocol; its PID proves reuse and restart,
// and a delayed answer exercises cancellation without substituting a resolver.
func TestCartesianProcessHelper(t *testing.T) {
	args := os.Args
	if len(args) < 4 || args[len(args)-3] != "--resolve-only" {
		return
	}
	dir := args[len(args)-2]
	file, err := os.OpenFile(filepath.Join(dir, "starts"), os.O_CREATE|os.O_APPEND|os.O_WRONLY, 0600)
	if err != nil {
		os.Exit(2)
	}
	fmt.Fprintln(file, os.Getpid())
	file.Close()
	scanner := bufio.NewScanner(os.Stdin)
	for scanner.Scan() {
		var in CartesianResolveRequest
		if json.Unmarshal(scanner.Bytes(), &in) != nil {
			os.Exit(2)
		}
		if in.Pose[0] > 0 {
			<-time.NewTimer(time.Duration(in.Pose[0]) * time.Millisecond).C
		}
		if in.Frame == "oversized" {
			// Unverified response bound: reject more than 1048576 bytes.
			fmt.Println(strings.Repeat("x", 1048577))
			continue
		}
		json.NewEncoder(os.Stdout).Encode(CartesianResolution{Accepted: true, Velocities: []float64{float64(os.Getpid()), 0, 0, 0, 0, 0}, JointLimitScale: 1, SingularityScale: 1})
	}
	os.Exit(0)
}

func processFixture(t *testing.T) (*CartesianProcess, string) {
	t.Helper()
	dir := t.TempDir()
	binary, err := os.Executable()
	if err != nil {
		t.Fatal(err)
	}
	script := filepath.Join(dir, "resolver")
	if err := os.WriteFile(script, []byte("#!/bin/sh\nexec '"+strings.ReplaceAll(binary, "'", "'\\''")+"' -test.run=TestCartesianProcessHelper -- \"$@\"\n"), 0700); err != nil {
		t.Fatal(err)
	}
	p := NewCartesianProcess(script, dir, "rosie_1420_v1")
	t.Cleanup(p.Close)
	return p, dir
}

func processStarts(t *testing.T, dir string, count int) []string {
	t.Helper()
	var starts []string
	rtAwait(t, func() bool {
		raw, _ := os.ReadFile(filepath.Join(dir, "starts"))
		starts = strings.Fields(string(raw))
		return len(starts) == count
	})
	return starts
}

func TestCartesianProcessLifecycle(t *testing.T) {
	p, dir := processFixture(t)
	first := processStarts(t, dir, 1)
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	for i := 0; i < 2; i++ {
		out, err := p.Resolve(ctx, CartesianResolveRequest{Model: "rosie_1420_v1"})
		if err != nil || strconv.Itoa(int(out.Velocities[0])) != first[0] {
			t.Fatalf("warm reply=%+v err=%v", out, err)
		}
	}
	pid, _ := strconv.Atoi(first[0])
	if err := syscall.Kill(pid, syscall.SIGKILL); err != nil {
		t.Fatal(err)
	}
	starts := processStarts(t, dir, 2)
	out, err := p.Resolve(ctx, CartesianResolveRequest{Model: "rosie_1420_v1"})
	if err != nil || strconv.Itoa(int(out.Velocities[0])) != starts[1] || starts[0] == starts[1] {
		t.Fatalf("restart reply=%+v err=%v", out, err)
	}
	p.Close()
	pid, _ = strconv.Atoi(starts[1])
	if err := syscall.Kill(pid, 0); !errors.Is(err, syscall.ESRCH) {
		t.Fatalf("process survives close: %v", err)
	}
	if _, err := p.Resolve(ctx, CartesianResolveRequest{}); err == nil {
		t.Fatal("closed resolver answered")
	}
}

func TestCartesianProcessBudgetAndBounds(t *testing.T) {
	for _, tc := range []struct {
		name, frame string
		delay       float64
		accepted    bool
	}{
		{"below", "base", 200, true}, {"above", "base", 1000, false}, {"oversized", "oversized", 0, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			p, dir := processFixture(t)
			processStarts(t, dir, 1)
			ctx, cancel := context.WithTimeout(context.Background(), 750*time.Millisecond)
			defer cancel()
			out, err := p.Resolve(ctx, CartesianResolveRequest{Model: "rosie_1420_v1", Frame: tc.frame, Pose: [6]float64{tc.delay}})
			if (err == nil && out.Accepted) != tc.accepted {
				t.Fatalf("reply=%+v err=%v", out, err)
			}
			if tc.name == "above" && !errors.Is(err, context.DeadlineExceeded) {
				t.Fatal(err)
			}
			if tc.name == "oversized" && !errors.Is(err, bufio.ErrBufferFull) {
				t.Fatalf("oversized response error=%v", err)
			}
			if !tc.accepted {
				processStarts(t, dir, 2)
			}
		})
	}
}

func TestCartesianProcessUnavailable(t *testing.T) {
	p := NewCartesianProcess(filepath.Join(t.TempDir(), "missing"), "", "rosie_1420_v1")
	defer p.Close()
	ctx, cancel := context.WithTimeout(context.Background(), 750*time.Millisecond)
	defer cancel()
	if _, err := p.Resolve(ctx, CartesianResolveRequest{Model: "rosie_1420_v1"}); err == nil {
		t.Fatal("missing executable answered")
	}
}

// The resolver command is a command line: an ssh prefix runs the cell's own
// motion server, and the repo argument names the tree on that host.
func TestResolverArgvRunsOnTheCell(t *testing.T) {
	got := resolverArgv("ssh -o BatchMode=yes user@bench-relay /home/user/rt-core-olp/bin/robot-v4-cartesiand", "/home/user/rt-core-olp", "rosie_1420_v1")
	want := []string{"ssh", "-o", "BatchMode=yes", "user@bench-relay", "/home/user/rt-core-olp/bin/robot-v4-cartesiand", "--resolve-only", "/home/user/rt-core-olp", "rosie_1420_v1"}
	if strings.Join(got, "|") != strings.Join(want, "|") {
		t.Fatalf("argv=%q", got)
	}
	if got := resolverArgv("/usr/local/bin/robot-v4-cartesiand", "/repo", "rosie_1400_v3"); len(got) != 4 || got[0] != "/usr/local/bin/robot-v4-cartesiand" || got[1] != "--resolve-only" {
		t.Fatalf("plain argv=%q", got)
	}
}
