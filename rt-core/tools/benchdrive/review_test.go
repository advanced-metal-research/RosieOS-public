//go:build linux

package main

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"net"
	"net/http"
	"net/http/httptest"
	"os"
	"os/signal"
	"path/filepath"
	"strings"
	"sync"
	"syscall"
	"testing"
	"time"

	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

func testBuildDir(t *testing.T) string {
	t.Helper()
	root, err := filepath.Abs("../../build")
	simfixture.Must(t, err)
	dir, err := os.MkdirTemp(root, "benchdrive-review-")
	simfixture.Must(t, err)
	t.Cleanup(func() { os.RemoveAll(dir) })
	return dir
}

func testRunner(t *testing.T, f *fixture, phase, name string) *runner {
	t.Helper()
	o, err := parseOptions(f.args(phase, name, true), io.Discard)
	simfixture.Must(t, err)
	c, err := control.Dial(control.UnixPath(f.socket))
	simfixture.Must(t, err)
	t.Cleanup(func() { c.Close() })
	e, err := openEvidence(o.evidence)
	simfixture.Must(t, err)
	t.Cleanup(func() { simfixture.Must(t, e.close()) })
	return newRunner(o, c, e, io.Discard)
}

// assertRedactedSessions checks every nested session, including raw response
// data, so a new evidence shape cannot silently publish a complete capability.
func assertRedactedSessions(t *testing.T, raw []byte) {
	t.Helper()
	var value any
	simfixture.Must(t, json.Unmarshal(raw, &value))
	var visit func(any)
	visit = func(value any) {
		switch value := value.(type) {
		case map[string]any:
			for key, child := range value {
				if key == "session" {
					session, ok := child.(string)
					if !ok || (len(session) != 0 && len(session) != 8) {
						t.Fatal("evidence contains an unredacted session")
					}
				}
				visit(child)
			}
		case []any:
			for _, child := range value {
				visit(child)
			}
		}
	}
	visit(value)
}

func TestEvidenceRedactsTokensWithoutChangingRequests(t *testing.T) {
	e, err := openEvidence(filepath.Join(testBuildDir(t), "redaction.jsonl"))
	simfixture.Must(t, err)
	t.Cleanup(func() { simfixture.Must(t, e.close()) })
	session := "0123456789abcdef0123456789abcdef"
	fence := control.Fence{Session: session, Generation: 18446744073709551615}
	grant := control.Grant{Session: session, Generation: fence.Generation}
	data, err := json.Marshal(grant)
	simfixture.Must(t, err)
	for _, item := range []struct {
		op                string
		request, response any
	}{
		{"acquire", control.Request{Operation: "acquire"}, control.Response{Data: data}},
		{"renew", fence, grant},
		{"enable", control.Request{Fence: fence}, control.Response{Data: data}},
		{"update_jog", map[string]any{"fence": fence}, map[string]any{"nested": []any{grant}}},
	} {
		simfixture.Must(t, e.receipt(item.op, 123456789, item.request, item.response, nil))
	}
	raw, err := os.ReadFile(e.file.Name())
	simfixture.Must(t, err)
	if bytes.Contains(raw, []byte(session)) || !bytes.Contains(raw, []byte(`"session":"01234567"`)) || !bytes.Contains(raw, []byte(`18446744073709551615`)) {
		t.Fatal("redaction leaked a token, lost its prefix, or rounded a generation")
	}
	for _, line := range bytes.Split(bytes.TrimSpace(raw), []byte{'\n'}) {
		assertRedactedSessions(t, line)
	}
	if fence.Session != session || grant.Session != session || !bytes.Contains(data, []byte(session)) {
		t.Fatal("evidence changed a live request or grant")
	}
}

func unixServer(t *testing.T, handler http.HandlerFunc) string {
	t.Helper()
	dir, err := os.MkdirTemp("", "bd-http-")
	simfixture.Must(t, err)
	t.Cleanup(func() { os.RemoveAll(dir) })
	path := filepath.Join(dir, "c.sock")
	listener, err := net.Listen("unix", path)
	simfixture.Must(t, err)
	server := httptest.NewUnstartedServer(handler)
	simfixture.Must(t, server.Listener.Close())
	server.Listener = listener
	server.Start()
	t.Cleanup(server.Close)
	return path
}

// TestExitClasses uses controlled protocol responses for each failure class;
// simulation tests separately establish the real daemon's positive behavior.
func TestExitClasses(t *testing.T) {
	for _, tc := range []struct {
		name, class string
		code        int
	}{
		{"success", "expectations_met", 0},
		{"refused", "expectation_failed", 1},
		{"different_state", "expectation_failed", 1},
		{"usage", "usage", 2},
		{"confirmation", "motion_confirmation_required", 3},
		{"transport", "transport_or_protocol_loss", 4},
		{"protocol", "transport_or_protocol_loss", 4},
		{"receipt_operation", "transport_or_protocol_loss", 4},
		{"receipt_sequence", "transport_or_protocol_loss", 4},
		{"deadline", "observation_deadline_expired", 5},
		// A status deadline carries no independent evidence of transport loss.
		{"deadline_inflight", "observation_deadline_expired", 5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			f := fixture{hash: strings.Repeat("a", 64), build: testBuildDir(t)}
			var mu sync.Mutex
			grant := control.Grant{}
			stopped := false
			f.socket = unixServer(t, func(w http.ResponseWriter, req *http.Request) {
				mu.Lock()
				defer mu.Unlock()
				if tc.name == "protocol" {
					io.WriteString(w, "invalid JSON")
					return
				}
				response := map[string]any{"schema": "rosie.rt-control.response.v1"}
				switch req.URL.Path {
				case "/v1/describe":
					response["data"] = control.Description{ContractVersion: 2, NativeDescription: ipcclient.Description{
						ConfigurationSHA256: f.hash, Axes: []ipcclient.AxisDescription{{RequireHome: true, NativeHome: true}}}}
				case "/v1/status":
					if tc.name == "deadline_inflight" && stopped && grant.Session != "" {
						<-req.Context().Done()
						return
					}
					status := control.Status{Core: ipcclient.Status{TimeNS: ipcclient.HostMonotonicNS(), Axes: []ipcclient.AxisStatus{{}}}}
					response["data"] = status
				case "/v1/control":
					var command control.Request
					if err := json.NewDecoder(req.Body).Decode(&command); err != nil {
						t.Error(err)
						return
					}
					response["operation"] = command.Operation
					reason := ""
					switch command.Operation {
					case "acquire":
						if tc.name == "refused" || grant.Session != "" {
							reason = "control_already_owned"
						} else {
							grant = control.Grant{Session: "0123456789abcdef0123456789abcdef", Generation: 1, LeaseMS: 500}
						}
					case "enable":
						if tc.name != "different_state" && tc.name != "receipt_sequence" {
							reason = "native_rejected"
							response["native_result"] = ipcclient.CommandResult{Reason: ipcclient.ReasonNotReady}
						}
						response["sequence"] = 1
						if tc.name == "receipt_sequence" {
							delete(response, "sequence")
						}
					case "stop":
						grant.Generation++
						stopped = true
					case "release":
						grant = control.Grant{}
					}
					if tc.name == "receipt_operation" && command.Operation == "acquire" {
						response["operation"] = "wrong_operation"
					}
					response["data"] = grant
					if reason != "" {
						response["error"] = reason
						w.WriteHeader(http.StatusConflict)
					}
				default:
					t.Errorf("unexpected endpoint %s", req.URL.Path)
				}
				if err := json.NewEncoder(w).Encode(response); err != nil {
					t.Error(err)
				}
			})
			if tc.name == "transport" {
				f.socket = filepath.Join(filepath.Dir(f.socket), "missing.sock")
			}
			args := append(f.args("inhibited", "result", true), "--axis-mask", "1", "--deadline-ns", "500000000")
			if tc.name == "usage" {
				args = append(args, "--axis-mask", "0")
			} else if tc.name == "confirmation" {
				args = f.args("home", "result", false)
			} else if tc.name == "deadline" {
				// Unverified mock budget: 50,000,000 ns expires while awaiting
				// the first renewal, preventing an in-flight status race here.
				args = append(args, "--deadline-ns", "50000000")
			}
			var out bytes.Buffer
			if code := run(context.Background(), args, &out); code != tc.code {
				t.Fatalf("exit=%d, want %d: %s", code, tc.code, out.String())
			}
			records := readRecords(t, filepath.Join(f.build, "result.jsonl"))
			last := records[len(records)-1]
			if last.Class != tc.class || last.ExitCode != tc.code {
				t.Fatalf("summary class=%s code=%d", last.Class, last.ExitCode)
			}
		})
	}
}

func TestTransportLossPrecedesJoinedDeadline(t *testing.T) {
	for _, loss := range []error{control.ErrTransport, control.ErrConnectionUnsent, control.ErrProtocol} {
		code, class := outcome(errors.Join(context.DeadlineExceeded, loss))
		if code != 4 || class != "transport_or_protocol_loss" {
			t.Fatalf("joined failure class=%s code=%d", class, code)
		}
	}
}

func TestInterruptedOutcomePrecedence(t *testing.T) {
	for _, tc := range []struct {
		name, class string
		err         error
		code        int
	}{
		{"canceled", "interrupted", context.Canceled, 6},
		{"transport", "transport_or_protocol_loss", errors.Join(context.Canceled, control.ErrTransport), 4},
		{"unsent", "transport_or_protocol_loss", errors.Join(context.Canceled, control.ErrConnectionUnsent), 4},
		{"protocol", "transport_or_protocol_loss", errors.Join(context.Canceled, control.ErrProtocol), 4},
		{"deadline", "observation_deadline_expired", errors.Join(context.Canceled, context.DeadlineExceeded), 5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			if code, class := outcome(tc.err); code != tc.code || class != tc.class {
				t.Fatalf("class=%s code=%d, want %s %d", class, code, tc.class, tc.code)
			}
		})
	}
}

func TestObserveRetainsCancellationCause(t *testing.T) {
	for _, cause := range []error{&control.Rejected{Reason: "renewal_refused"}, errors.New("renewal reader panic: injected")} {
		t.Run(cause.Error(), func(t *testing.T) {
			parent, stop := context.WithTimeout(context.Background(), fixtureDeadline)
			defer stop()
			ctx, cancel := context.WithCancelCause(parent)
			defer cancel(nil)
			socket := unixServer(t, func(w http.ResponseWriter, req *http.Request) {
				cancel(cause)
				<-req.Context().Done()
			})
			c, err := control.Dial(control.UnixPath(socket))
			simfixture.Must(t, err)
			defer c.Close()
			r := runner{o: options{deadlineNS: 500000000}, c: c, e: &evidence{sink: io.Discard}}
			_, err = r.observe(ctx, func(control.Status) (bool, error) {
				t.Fatal("canceled status must not satisfy an observation")
				return true, nil
			})
			if !errors.Is(err, cause) || errors.Is(err, control.ErrTransport) || strings.Count(err.Error(), cause.Error()) != 1 {
				t.Fatalf("observation lost or duplicated cancellation cause: %v", err)
			}
		})
	}
}

func TestHelpDocumentsConfirmationAndExitClasses(t *testing.T) {
	var out bytes.Buffer
	if code := run(context.Background(), []string{"--help"}, &out); code != 0 {
		t.Fatalf("help exit=%d: %s", code, &out)
	}
	for _, text := range []string{
		"required for energising phases and the inhibited Enable probe",
		"-confirm-motion",
		"0 expectations_met",
		"1 expectation_failed",
		"2 usage",
		"3 motion_confirmation_required",
		"4 transport_or_protocol_loss",
		"5 observation_deadline_expired",
		"6 interrupted (SIGINT/SIGTERM or caller cancellation)",
		"Transport/protocol loss takes precedence over deadline expiry and interruption.",
		"Deadline expiry takes precedence over interruption.",
	} {
		if !strings.Contains(out.String(), text) {
			t.Errorf("help missing %q: %s", text, &out)
		}
	}
}

type interruptSink struct {
	writer io.Writer
	ctx    context.Context
	signal syscall.Signal
	sent   bool
}

// Write delivers an operator signal at the first observed hold record and waits
// for cancellation, preventing request timing from deciding the tested exit class.
func (s *interruptSink) Write(raw []byte) (int, error) {
	var r savedRecord
	if err := json.Unmarshal(raw, &r); err != nil {
		return 0, err
	}
	if !s.sent && r.Operation == "hold_drift" {
		if err := syscall.Kill(os.Getpid(), s.signal); err != nil {
			return 0, err
		}
		s.sent = true
		<-s.ctx.Done()
	}
	return s.writer.Write(raw)
}

func TestOperatorSignalsStopAndRelease(t *testing.T) {
	f := startFixture(t)
	f.phase(t, "home")
	for _, tc := range []struct {
		name   string
		signal syscall.Signal
	}{{"SIGINT", syscall.SIGINT}, {"SIGTERM", syscall.SIGTERM}} {
		t.Run(tc.name, func(t *testing.T) {
			parent, stop := context.WithTimeout(context.Background(), fixtureDeadline)
			defer stop()
			ctx, cancel := signal.NotifyContext(parent, os.Interrupt, syscall.SIGTERM)
			defer cancel()
			r := testRunner(t, f, "enable", tc.name)
			sink := &interruptSink{writer: r.e.file, ctx: ctx, signal: tc.signal}
			r.e.sink = sink
			var out bytes.Buffer
			code := execute(ctx, r.o, r.e, &out)
			if !sink.sent || code != 6 {
				t.Fatalf("signal sent=%v exit=%d: %s", sink.sent, code, &out)
			}
			records := readRecords(t, r.o.evidence)
			last := records[len(records)-1]
			if last.Class != "interrupted" || last.ExitCode != 6 {
				t.Fatalf("interruption summary: %+v", last)
			}
			stops, releases, stopIndex, releaseIndex := 0, 0, -1, -1
			for i, record := range records {
				if record.Kind != "request" {
					continue
				}
				switch record.Operation {
				case "stop":
					stops++
					stopIndex = i
				case "release":
					releases++
					releaseIndex = i
				}
			}
			if stops != 1 || releases != 1 || stopIndex >= releaseIndex {
				t.Fatalf("cleanup stops=%d releases=%d indices=%d,%d", stops, releases, stopIndex, releaseIndex)
			}
			f.poll(t, func(s control.Status) bool {
				return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Core.LeaseValid == 0
			})
		})
	}
}

func TestRenewalRequestStart(t *testing.T) {
	received := make(chan uint64, 1)
	socket := unixServer(t, func(w http.ResponseWriter, req *http.Request) {
		started := ipcclient.HostMonotonicNS()
		select {
		case received <- started:
		default:
		}
		io.WriteString(w, `{"schema":"rosie.rt-control.response.v1","operation":"renew","data":{"session":"0123456789abcdef0123456789abcdef","generation":1}}`)
	})
	c, err := control.Dial(control.UnixPath(socket))
	simfixture.Must(t, err)
	defer c.Close()
	c.SetFence(control.Fence{Session: "0123456789abcdef0123456789abcdef", Generation: 1})
	ctx, cancel := context.WithTimeout(context.Background(), fixtureDeadline)
	defer cancel()
	before := ipcclient.HostMonotonicNS()
	updates, err := c.StartRenewal(ctx, 100*time.Millisecond)
	simfixture.Must(t, err)
	select {
	case update := <-updates:
		simfixture.Must(t, update.Err)
		atServer := <-received
		if update.StartedHostMonotonicNS < before || update.StartedHostMonotonicNS > atServer {
			t.Fatal("renewal timestamp was not captured before the HTTP request")
		}
	case <-ctx.Done():
		t.Fatal("renewal did not report a receipt")
	}
}

func TestJogGuardUsesOperatorFraction(t *testing.T) {
	r := runner{o: options{axis: 0, fraction: 0.1}, description: control.Description{NativeDescription: ipcclient.Description{
		Axes: []ipcclient.AxisDescription{{MaxVelocity: 0.1}}}}}
	for _, velocity := range []float64{0.02, -0.02} {
		now := ipcclient.HostMonotonicNS()
		err := r.sendJog(context.Background(), nil, control.Fence{}, 1, velocity, now+1000000000, now+500000000)
		if err == nil || err.Error() != "jog velocity exceeds the fraction bound" {
			t.Fatalf("velocity=%f escaped operator's 0.01 bound: %v", velocity, err)
		}
	}
}

type panicSink struct {
	writer    io.Writer
	operation string
	panicked  bool
}

func (s *panicSink) Write(raw []byte) (int, error) {
	var r savedRecord
	if err := json.Unmarshal(raw, &r); err != nil {
		return 0, err
	}
	if !s.panicked && r.Operation == s.operation {
		s.panicked = true
		panic("injected evidence panic")
	}
	return s.writer.Write(raw)
}

func TestHelperPanicsStillStopAndRelease(t *testing.T) {
	f := startFixture(t)
	for _, tc := range []struct{ phase, operation, cause string }{
		{"inhibited", "renew", "renewal reader panic"},
		{"jog", "update_jog", "jog producer panic"},
	} {
		t.Run(tc.operation, func(t *testing.T) {
			if tc.phase == "jog" {
				f.phase(t, "home")
			}
			r := testRunner(t, f, tc.phase, tc.operation+"-panic")
			sink := &panicSink{writer: r.e.file, operation: tc.operation}
			r.e.sink = sink
			ctx, cancel := context.WithTimeout(context.Background(), fixtureDeadline)
			defer cancel()
			err := r.summary(r.run(ctx))
			cause := tc.cause + ": injected evidence panic"
			if !sink.panicked || err == nil || strings.Count(err.Error(), cause) != 1 {
				t.Fatalf("panic was not retained as cancellation cause: %v", err)
			}
			records := readRecords(t, r.o.evidence)
			if summary := records[len(records)-1]; strings.Count(strings.Join(summary.NotMet, "\n"), cause) != 1 {
				t.Fatalf("summary must contain the panic cause exactly once: %+v", summary)
			}
			stop, release, end := -1, -1, -1
			for i, record := range records {
				switch record.Operation {
				case "end_jog":
					end = i
				case "stop":
					stop = i
				case "release":
					release = i
				}
			}
			if stop < 0 || release <= stop || (tc.phase == "jog" && (end < 0 || end >= stop)) {
				t.Fatalf("cleanup order end=%d stop=%d release=%d", end, stop, release)
			}
			f.poll(t, func(s control.Status) bool {
				return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Core.LeaseValid == 0
			})
		})
	}
}
