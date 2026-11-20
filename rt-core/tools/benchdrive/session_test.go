//go:build linux

package main

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"math"
	"net"
	"net/http"
	"net/http/httputil"
	"net/url"
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"sync"
	"sync/atomic"
	"syscall"
	"testing"

	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

func TestSimulationDirectionalJog(t *testing.T) {
	f := startFixture(t)
	f.phase(t, "home")
	for _, direction := range []int{1, -1} {
		name := fmt.Sprintf("direction-%d", direction)
		args := append(f.args("jog", name, true), "--direction", fmt.Sprint(direction), "--jog-fraction", "0.5", "--jog-duration-ns", "1000000000", "--motor-revs-per-axis-rev", "2")
		var out bytes.Buffer
		if code := run(context.Background(), args, &out); code != 0 {
			t.Fatalf("exit=%d: %s", code, &out)
		}
		records := readRecords(t, filepath.Join(f.build, name+".jsonl"))
		var before, after []int32
		frames, travel := 0, false
		for _, rec := range records {
			if rec.Operation == "jog_before" || rec.Operation == "jog_after" {
				var pos struct {
					Counts []int32 `json:"pos_counts"`
				}
				simfixture.Must(t, json.Unmarshal(rec.Response, &pos))
				if rec.Operation == "jog_before" {
					before = pos.Counts
				} else {
					after = pos.Counts
				}
			}
			if rec.Operation == "jog_reversal" {
				t.Fatal("directional jog reversed")
			}
			if rec.Operation == "update_jog" {
				var frame struct {
					Velocity  []float64 `json:"velocity"`
					Deadline  uint64    `json:"deadline_host_ns"`
					End       uint64    `json:"jog_end_host_ns"`
					Direction int       `json:"direction"`
					Terminal  bool      `json:"terminal_zero"`
					Mask      uint32    `json:"axis_mask"`
				}
				simfixture.Must(t, json.Unmarshal(rec.Request, &frame))
				if frame.Direction != direction || frame.Mask != 1 || frame.Deadline <= rec.StartedNS || frame.Deadline-rec.StartedNS > 100000000 {
					t.Fatalf("direction, mask or frame expiry: %s", rec.Request)
				}
				if !frame.Terminal && frame.Velocity[0] != float64(direction)*.05 {
					t.Fatalf("wrong signed velocity: %v", frame.Velocity)
				}
				if frame.Terminal && (rec.StartedNS < frame.End || frame.Velocity[0] != 0) {
					t.Fatal("directional jog neutralised before its duration")
				}
				for _, v := range frame.Velocity[1:] {
					if v != 0 {
						t.Fatal("unmasked axis received velocity")
					}
				}
				frames++
			}
			if rec.Operation == "jog_travel" {
				var v struct {
					Rad       float64 `json:"measured_travel_rad"`
					Revs      float64 `json:"motor_revolutions"`
					Direction int     `json:"direction"`
				}
				simfixture.Must(t, json.Unmarshal(rec.Response, &v))
				// Fixed 0.05 rad travel, with 0.01 rad allowance for frame expiry,
				// acceleration, deceleration and the 0.0002 rad completion tolerance.
				if v.Direction != direction || math.Abs(v.Rad-.05*float64(direction)) > .01 || math.Abs(v.Revs-v.Rad/math.Pi) > 1e-12 {
					t.Fatalf("signed travel or motor conversion: %+v", v)
				}
				travel = true
			}
		}
		if !travel || frames < 2 || len(before) != 9 || len(after) != 9 || !reflect.DeepEqual(before[1:], after[1:]) {
			t.Fatal("missing travel evidence or unmasked motion")
		}
		counts := (after[0] - before[0]) * int32(direction)
		if counts < 834 || counts > 1252 {
			t.Fatalf("signed travel=%d counts outside 834..1252", counts)
		}
	}
}

func requireSessionCleanup(t *testing.T, f *fixture, records []savedRecord, success bool) {
	t.Helper()
	stop, release := -1, -1
	for i, rec := range records {
		if rec.Kind == "request" && rec.Operation == "stop" {
			stop = i
		}
		if rec.Kind == "request" && rec.Operation == "release" {
			release = i
		}
	}
	if stop < 0 || release <= stop {
		t.Fatalf("terminal Stop/Release order: %d %d", stop, release)
	}
	last := records[len(records)-1]
	if (last.ExitCode == 0) != success || (!success && last.Class == "expectations_met") {
		t.Fatalf("wrong session exit class: %+v", last)
	}
	f.poll(t, func(s control.Status) bool {
		return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Core.LeaseValid == 0
	})
}

func TestSimulationSessionScript(t *testing.T) {
	f := startFixture(t)
	f.phase(t, "home")
	commands := []string{"move 0:0.02,8:-0.01 0.5 0.5", "status", "jog 0 -1 0.5 500000000", "stop", "move 0:0.01 0.5 0.5", "quit"}
	var out bytes.Buffer
	if code := runInput(context.Background(), f.args("session", "script", true), io.NopCloser(strings.NewReader(strings.Join(commands, "\n")+"\n")), &out); code != 0 {
		t.Fatalf("session exit=%d: %s", code, &out)
	}
	records := readRecords(t, filepath.Join(f.build, "script.jsonl"))
	requireSessionCleanup(t, f, records, true)
	var got []string
	enables, arms, acquires, moves, jogs := 0, 0, 0, 0, 0
	for _, rec := range records {
		if rec.Kind == "session_command" {
			var req struct {
				Line string `json:"line"`
			}
			simfixture.Must(t, json.Unmarshal(rec.Request, &req))
			got = append(got, req.Line)
			if !bytes.Contains(rec.Response, []byte("expectations_met")) {
				t.Fatalf("failed command: %s", rec.Response)
			}
		}
		if rec.Kind == "request" {
			switch rec.Operation {
			case "enable":
				enables++
			case "arm":
				arms++
			case "acquire":
				acquires++
			case "start_trajectory":
				moves++
			case "begin_jog":
				jogs++
			}
		}
	}
	if !reflect.DeepEqual(got, commands) || enables != 2 || arms != 2 || acquires != 1 || moves != 2 || jogs != 1 {
		t.Fatalf("commands=%v enables=%d arms=%d acquires=%d moves=%d jogs=%d", got, enables, arms, acquires, moves, jogs)
	}
	for _, required := range []string{"move axis:rad[,axis:rad...] [velocity-fraction] [accel-fraction]", "jog axis direction fraction duration_ns", "home mask", "axis 0 readiness=ready", "logical_position=", "home_valid=true", "statusword=0x0227"} {
		if !strings.Contains(out.String(), required) {
			t.Fatalf("missing operator output %q: %s", required, &out)
		}
	}
}

func TestSimulationSessionHomeAndEOF(t *testing.T) {
	f := startFixture(t)
	f.phase(t, "home")
	before := f.poll(t, func(s control.Status) bool { return s.Core.HomeValidMask == 511 })
	var out bytes.Buffer
	if code := runInput(context.Background(), f.args("session", "home-eof", true), io.NopCloser(strings.NewReader("home 1\nstatus\n")), &out); code != 0 {
		t.Fatalf("session exit=%d: %s", code, &out)
	}
	records := readRecords(t, filepath.Join(f.build, "home-eof.jsonl"))
	requireSessionCleanup(t, f, records, true)
	f.poll(t, func(s control.Status) bool {
		return s.Core.HomeEpoch == before.Core.HomeEpoch+1 && s.Core.HomeValidMask == 511
	})
}

func TestSimulationSessionFaultAndExternalStop(t *testing.T) {
	for _, fault := range []bool{true, false} {
		t.Run(fmt.Sprintf("fault-%t", fault), func(t *testing.T) {
			var extra []string
			if fault {
				// Unverified injection offset: 5,000,000,000 ns from first Enable,
				// including Home; the test requires a completed command before fault.
				extra = []string{"--sim-statusword-fault-after-enabled-ns", "5000000000"}
			}
			f := startFixtureMode(t, false, extra...)
			f.phase(t, "home")
			r := testRunner(t, f, "session", "interrupted-session")
			ready := &sessionReadyOutput{ready: make(chan struct{})}
			r.out = ready
			var input io.ReadCloser
			var writer io.WriteCloser
			if fault {
				input, writer = io.Pipe()
			} else {
				readFile, writeFile, err := os.Pipe()
				simfixture.Must(t, err)
				fd, err := syscall.Dup(int(readFile.Fd()))
				simfixture.Must(t, err)
				simfixture.Must(t, readFile.Close())
				// Fd returns blocking mode; NewFile matches inherited os.Stdin.
				input, writer = os.NewFile(uintptr(fd), "blocking-stdin"), writeFile
			}
			defer writer.Close()
			r.in = input
			ctx, cancel := context.WithTimeout(context.Background(), fixtureDeadline)
			defer cancel()
			done := make(chan error, 1)
			go func() { done <- r.summary(r.run(ctx)) }()
			joined := false
			defer func() {
				cancel()
				input.Close()
				if !joined {
					<-done
				}
			}()
			writeSessionLine(t, ctx, writer, "status\n")
			ready.wait(t, ctx)
			f.poll(t, func(s control.Status) bool { return s.Core.Armed == 1 && s.Core.Axes[0].Readiness == "ready" })
			if !fault {
				f.client.SetFence(r.c.Fence())
				_, err := f.client.Stop(ctx)
				simfixture.Must(t, err)
			}
			select {
			case err := <-done:
				joined = true
				if err == nil {
					t.Fatal("fault/Stop session succeeded")
				}
			case <-ctx.Done():
				t.Fatal("session did not terminate while stdin was idle")
			}
			records := readRecords(t, r.o.evidence)
			requireSessionCleanup(t, f, records, false)
			commandCompleted := false
			for _, rec := range records {
				commandCompleted = commandCompleted || rec.Kind == "session_command" && bytes.Contains(rec.Response, []byte("expectations_met"))
			}
			if !commandCompleted {
				t.Fatal("injection preceded the operator session")
			}
			if fault && !strings.Contains(strings.Join(records[len(records)-1].NotMet, " "), "drive_readiness (bit=7 axis_mask=1)") {
				t.Fatalf("fault exit lacks its reason: %+v", records[len(records)-1])
			}
		})
	}
}

// A pre-reply inhibited sample cannot establish the replacement native fence;
// otherwise the next command mistakes our own Stop for external cancellation.
func TestSessionStopWaitsForResumedGeneration(t *testing.T) {
	var stamp atomic.Uint64
	var reads atomic.Int32
	socket := unixServer(t, func(w http.ResponseWriter, req *http.Request) {
		var response any
		operation := "status"
		if req.URL.Path == "/v1/control" {
			var command control.Request
			if err := json.NewDecoder(req.Body).Decode(&command); err != nil {
				t.Error(err)
				return
			}
			operation = command.Operation
			if operation != "stop" && operation != "renew" {
				t.Errorf("unexpected operation %s", operation)
			}
			stamp.Store(ipcclient.HostMonotonicNS())
			response = control.Grant{Session: "owner", Generation: 4, DeadlineHostNS: stamp.Load() + 500000000}
		} else {
			s := control.Status{Core: ipcclient.Status{TimeNS: ipcclient.HostMonotonicNS(), ControlGeneration: 14, LeaseValid: 1}}
			if reads.Add(1) == 1 {
				s.Core.TimeNS, s.Core.ControlGeneration, s.Core.LeaseValid = stamp.Load(), 13, 0
			}
			response = s
		}
		if err := json.NewEncoder(w).Encode(map[string]any{"schema": "rosie.rt-control.response.v1", "operation": operation, "data": response}); err != nil {
			t.Error(err)
		}
	})
	c, err := control.Dial(control.UnixPath(socket))
	simfixture.Must(t, err)
	defer c.Close()
	c.SetFence(control.Fence{Session: "owner", Generation: 3})
	// Unverified observation ceiling: 1,000,000,000 ns; success requires two
	// distinct publications and native generation 14, never elapsed time alone.
	r := runner{c: c, e: &evidence{sink: io.Discard}, o: options{deadlineNS: 1000000000},
		words: map[int]uint16{}, expectations: map[string]bool{}, sessionActive: true, sessionGeneration: 10}
	ctx, cancel := context.WithCancelCause(context.Background())
	defer cancel(nil)
	defer r.stopRenewal()
	simfixture.Must(t, r.sessionStop(ctx, cancel))
	if reads.Load() != 2 || r.sessionGeneration != 14 || r.last.Core.LeaseValid != 1 {
		t.Fatalf("resumed without a post-reply native fence: reads=%d generation=%d lease=%d", reads.Load(), r.sessionGeneration, r.last.Core.LeaseValid)
	}
	simfixture.Must(t, r.sessionHealth(r.last))
	external := r.last
	external.Core.ControlGeneration = 15
	if err := r.sessionHealth(external); err == nil {
		t.Fatal("external Stop after an owned Stop was ignored")
	}
}

// Typed status reasons must survive even when the grant failure arrived first.
func TestSessionFaultReasons(t *testing.T) {
	for _, tc := range []struct {
		name string
		core ipcclient.Status
		want string
	}{
		{"recovery", ipcclient.Status{Recovery: &ipcclient.RecoveryStatus{Faults: []ipcclient.FaultRecovery{{Bit: 7, Name: "drive_readiness", AxisMask: 1}}}}, "drive_readiness (bit=7 axis_mask=1)"},
		{"execution", ipcclient.Status{ExecutionFaultReasons: 256}, "execution_fault_reasons=256"},
		{"axis", ipcclient.Status{Axes: []ipcclient.AxisStatus{{Statusword: 8, Readiness: "drive_fault"}}}, "readiness=drive_fault statusword=0x0008"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			err := sessionFault(control.Status{Core: tc.core})
			if err == nil || !strings.Contains(err.Error(), tc.want) {
				t.Fatalf("lost native reason %q: %v", tc.want, err)
			}
		})
	}
}

type failedSessionInput struct{}

func (failedSessionInput) Read([]byte) (int, error) { return 0, errors.New("input failed") }
func (failedSessionInput) Close() error             { return nil }

func TestSimulationSessionRejectsUnreadableAndInvalidCommands(t *testing.T) {
	f := startFixture(t)
	f.phase(t, "home")
	for i, in := range []io.ReadCloser{failedSessionInput{}, io.NopCloser(strings.NewReader("move 9:.01\n")), io.NopCloser(strings.NewReader(strings.Repeat("x", 4097) + "\n"))} {
		name := fmt.Sprintf("invalid-%d", i)
		if code := runInput(context.Background(), f.args("session", name, true), in, io.Discard); code != 1 {
			t.Fatalf("bad session input exit=%d", code)
		}
		records := readRecords(t, filepath.Join(f.build, name+".jsonl"))
		requireSessionCleanup(t, f, records, false)
		commands := 0
		for _, rec := range records {
			if rec.Kind == "session_command" {
				commands++
			}
		}
		if commands != 1 {
			t.Fatalf("bad command records=%d", commands)
		}
	}
}

func TestSimulationSessionLostRenewal(t *testing.T) {
	f := startFixture(t)
	f.phase(t, "home")
	var refuse atomic.Bool
	proxy := httputil.NewSingleHostReverseProxy(&url.URL{Scheme: "http", Host: "control"})
	transport := &http.Transport{DialContext: func(ctx context.Context, _, _ string) (net.Conn, error) {
		return (&net.Dialer{}).DialContext(ctx, "unix", f.socket)
	}}
	defer transport.CloseIdleConnections()
	proxy.Transport = transport
	socket := unixServer(t, func(w http.ResponseWriter, req *http.Request) {
		if req.URL.Path == "/v1/control" {
			raw, err := io.ReadAll(req.Body)
			if err != nil {
				t.Error(err)
				return
			}
			req.Body.Close()
			req.Body = io.NopCloser(bytes.NewReader(raw))
			var command control.Request
			if err := json.Unmarshal(raw, &command); err != nil {
				t.Error(err)
				return
			}
			if command.Operation == "renew" && refuse.Load() {
				w.WriteHeader(http.StatusConflict)
				io.WriteString(w, `{"schema":"rosie.rt-control.response.v1","operation":"renew","error":"control_session_stale"}`)
				return
			}
		}
		proxy.ServeHTTP(w, req)
	})
	input, writer := io.Pipe()
	defer writer.Close()
	ctx, cancel := context.WithTimeout(context.Background(), fixtureDeadline)
	defer cancel()
	done := make(chan int, 1)
	ready := &sessionReadyOutput{ready: make(chan struct{})}
	go func() {
		done <- runInput(ctx, append(f.args("session", "renewal-loss", true), "--socket", socket), input, ready)
	}()
	joined := false
	defer func() {
		cancel()
		input.Close()
		if !joined {
			<-done
		}
	}()
	writeSessionLine(t, ctx, writer, "status\n")
	ready.wait(t, ctx)
	f.poll(t, func(s control.Status) bool { return s.Core.Armed == 1 && s.Core.Axes[0].Readiness == "ready" })
	refuse.Store(true)
	select {
	case code := <-done:
		joined = true
		if code != 1 {
			t.Fatalf("lost renewal exit=%d", code)
		}
	case <-ctx.Done():
		t.Fatal("lost renewal did not terminate idle session")
	}
	records := readRecords(t, filepath.Join(f.build, "renewal-loss.jsonl"))
	requireSessionCleanup(t, f, records, false)
	for _, rec := range records {
		if rec.Operation == "renew" && rec.Reason == "control_session_stale" {
			return
		}
	}
	t.Fatal("lost renewal lacks typed evidence")
}

type sessionReadyOutput struct {
	ready chan struct{}
	once  sync.Once
}

func (o *sessionReadyOutput) Write(raw []byte) (int, error) {
	if bytes.Equal(raw, []byte("ok status\n")) {
		o.once.Do(func() { close(o.ready) })
	}
	return len(raw), nil
}

func (o *sessionReadyOutput) wait(t *testing.T, ctx context.Context) {
	t.Helper()
	select {
	case <-o.ready:
	case <-ctx.Done():
		t.Fatal("session did not complete status before injection")
	}
}

// writeSessionLine bounds blocked stdin when setup fails before opening the
// reader, preventing a failed simulation from hanging the entire test process.
func writeSessionLine(t *testing.T, ctx context.Context, writer io.WriteCloser, line string) {
	t.Helper()
	done := make(chan error, 1)
	go func() { _, err := io.WriteString(writer, line); done <- err }()
	select {
	case err := <-done:
		simfixture.Must(t, err)
	case <-ctx.Done():
		writer.Close()
		<-done
		t.Fatal("session did not read stdin before its deadline")
	}
}

func TestSessionCommandGrammar(t *testing.T) {
	r := runner{o: options{mask: 1, velocityFraction: .1, accelFraction: .1}}
	for _, line := range []string{"", "unknown", "quit now", "stop 1", "status 1", "move", "move 0:0", "move 1:.1", "move 0:.1 NaN", "move 0:.1 .5 .51", "move 0:.1 .5 .5 extra", "jog", "jog 0 0 .1 1000", "jog 0 1 .51 1000", "jog 0 1 .1 2000000001", "home", "home 0", "home 2", "home -1", "\xff", "status\x00"} {
		armed := false
		if _, err := r.sessionCommand(context.Background(), func(error) {}, line, &armed); err == nil {
			t.Fatalf("accepted invalid command %q", line)
		}
	}
}

type stalledJogOutput struct {
	writer            io.Writer
	ctx               context.Context
	entered, resume   chan struct{}
	blocked           bool
	started, deadline uint64
}

func (s *stalledJogOutput) Write(raw []byte) (int, error) {
	var rec savedRecord
	if err := json.Unmarshal(raw, &rec); err != nil {
		return 0, err
	}
	if rec.Operation == "update_jog" && !s.blocked {
		var frame struct {
			Deadline uint64 `json:"deadline_host_ns"`
		}
		if err := json.Unmarshal(rec.Request, &frame); err != nil {
			return 0, err
		}
		s.started, s.deadline = rec.StartedNS, frame.Deadline
		s.blocked = true
		close(s.entered)
		select {
		case <-s.resume:
		case <-s.ctx.Done():
		}
	}
	return s.writer.Write(raw)
}

func TestSimulationDirectionalJogProducerStall(t *testing.T) {
	// Unverified race-instrumented fixture allowance: 100 ms keeps host
	// scheduling noise separate from the producer-stall deadline asserted below.
	f := startFixtureAxesWithCycleBudget(t, false, false, 9, 100_000_000)
	f.phase(t, "home")
	r := testRunner(t, f, "jog", "stalled-jog")
	r.o.direction, r.o.fraction, r.o.motorRevsPerAxisRev = 1, .5, 1
	delete(r.expectations, "jog_reversal")
	ctx, cancel := context.WithTimeout(context.Background(), fixtureDeadline)
	defer cancel()
	sink := &stalledJogOutput{writer: r.e.file, ctx: ctx, entered: make(chan struct{}), resume: make(chan struct{})}
	r.e.sink = sink
	done := make(chan error, 1)
	go func() { done <- r.summary(r.run(ctx)) }()
	joined, resumed := false, false
	defer func() {
		cancel()
		if !resumed {
			close(sink.resume)
		}
		if !joined {
			<-done
		}
	}()
	select {
	case <-sink.entered:
	case <-ctx.Done():
		t.Fatal("producer never sent a frame")
	}
	s := f.poll(t, func(s control.Status) bool {
		return s.Core.Armed == 1 && s.Jog != nil && s.Jog.ObservedSourceSequence == 1 && s.Jog.Open == 0 && s.Jog.StateReason != ipcclient.JogReasonRamping
	})
	if s.Core.Axes[0].PositionCounts < 0 || s.Core.Axes[0].PositionCounts > 200 ||
		sink.deadline-sink.started != 100000000 || s.Jog.NowHostNs-sink.started > 350000000 {
		t.Fatalf("stalled frame exceeded 200 counts, 100000000 ns lifetime or 350000000 ns observed halt: %+v", s.Jog)
	}
	close(sink.resume)
	resumed = true
	select {
	case err := <-done:
		joined = true
		if err == nil {
			t.Fatal("producer stall reported successful jog")
		}
	case <-ctx.Done():
		t.Fatal("stalled jog did not stop and release")
	}
	requireSessionCleanup(t, f, readRecords(t, r.o.evidence), false)
}
