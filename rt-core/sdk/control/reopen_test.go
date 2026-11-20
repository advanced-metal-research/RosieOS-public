//go:build linux

package control

import (
	"context"
	"encoding/json"
	"errors"
	"io"
	"net"
	"net/http"
	"net/http/httptest"
	"os"
	"strconv"
	"sync"
	"sync/atomic"
	"syscall"
	"testing"
	"time"
)

// TestReviewConnectionReopenedArm retains the bench observation on both sides
// of Acquire: peer-closed reservations must not become an Arm failure.
func TestReviewConnectionReopenedArm(t *testing.T) {
	for _, acquired := range []bool{false, true} {
		name := "connected_before_acquire"
		if acquired {
			name = "after_acquire"
		}
		t.Run(name, func(t *testing.T) {
			var arms, opened atomic.Int32
			server := httptest.NewUnstartedServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				payload := any(map[string]any{})
				operation := "describe"
				if r.Method == "POST" {
					var request Request
					must(t, json.NewDecoder(r.Body).Decode(&request))
					operation = request.Operation
					payload = Grant{Session: "owner", Generation: 7, LeaseMS: 3000}
					if operation == "arm" {
						arms.Add(1)
					}
				}
				// Peer close is explicit, making completion deterministic without a sleep.
				w.Header().Set("Connection", "close")
				body, _ := json.Marshal(payload)
				json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Operation: operation, Data: body})
			}))
			server.Config.ConnContext = func(ctx context.Context, _ net.Conn) context.Context { opened.Add(1); return ctx }
			server.Start()
			defer server.Close()
			c := newClient(server.URL, &http.Transport{})
			defer c.Close()
			// Unverified remote fixture: 250 ms RTT and 3000 ms lease ceiling.
			c.roundTrip, c.leaseCeiling = 250*time.Millisecond, 3000*time.Millisecond
			ctx := context.Background()
			must(t, c.warmConnections(ctx, true))
			if !acquired {
				server.CloseClientConnections()
			}
			_, err := c.Acquire(ctx, "test", Binding{})
			must(t, err)
			if acquired {
				server.CloseClientConnections()
			}
			before := opened.Load()
			_, err = c.Arm(ctx)
			must(t, err)
			if arms.Load() != 1 || opened.Load()-before != 1 || c.Fence().Generation != 7 {
				t.Fatalf("arms=%d new_connections=%d fence=%+v", arms.Load(), opened.Load()-before, c.Fence())
			}
		})
	}
}

// TestConnectionReopenedUnsentFailure forbids uncertain-outcome wording when
// the socket never connected, including expiration before any request bytes.
func TestConnectionReopenedUnsentFailure(t *testing.T) {
	for _, cause := range []error{syscall.ECONNREFUSED, context.DeadlineExceeded} {
		c := newClient("http://localhost", &http.Transport{DialContext: func(context.Context, string, string) (net.Conn, error) { return nil, cause }})
		_, err := c.Arm(context.Background())
		c.Close()
		if !errors.Is(err, ErrConnectionUnsent) || errors.Is(err, ErrTransport) {
			t.Fatalf("classification: %v", err)
		}
	}
}

// TestConnectionReopenedEveryRequestKind proves an unsent request is replayable
// even when a lost reply for that same request kind would remain uncertain.
func TestConnectionReopenedEveryRequestKind(t *testing.T) {
	for _, operation := range []string{"status", "acquire", "renew", "arm", "enable", "home", "stop", "release", "prepare_program", "prepare_trajectory", "start_trajectory", "start_program", "begin_jog", "end_jog"} {
		t.Run(operation, func(t *testing.T) {
			var requests atomic.Int32
			server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				requests.Add(1)
				io.Copy(io.Discard, r.Body)
				body, _ := json.Marshal(Grant{Session: "owner", Generation: 7, LeaseMS: 3000})
				json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Operation: operation, Data: body})
			}))
			defer server.Close()
			c := newClient(server.URL, &http.Transport{})
			defer c.Close()
			c.warmReady = true
			c.SetFence(Fence{Session: "owner", Generation: 7})
			var err error
			switch operation {
			case "status":
				_, err = c.Status(context.Background())
			case "prepare_program":
				_, err = c.PrepareProgram(context.Background(), []byte("immutable"))
			default:
				_, err = c.Command(context.Background(), &Request{Operation: operation})
			}
			must(t, err)
			if requests.Load() != 1 {
				t.Fatalf("requests=%d", requests.Load())
			}
		})
	}
}

// TestConnectionReopenedRetainsMutationAndStopsAtOneRetry prevents duplicate
// energising: both attempts carry identical bytes, including ID and generation.
func TestConnectionReopenedRetainsMutationAndStopsAtOneRetry(t *testing.T) {
	for _, failAgain := range []bool{false, true} {
		var calls, opened atomic.Int32
		bodies := make(chan string, 2)
		server := httptest.NewUnstartedServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			body, _ := io.ReadAll(r.Body)
			bodies <- string(body)
			if calls.Add(1) == 1 || failAgain {
				conn, _, _ := w.(http.Hijacker).Hijack()
				conn.Close()
				return
			}
			json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Operation: "arm"})
		}))
		server.Config.ConnContext = func(ctx context.Context, _ net.Conn) context.Context { opened.Add(1); return ctx }
		server.Start()
		c := newClient(server.URL, &http.Transport{})
		c.SetFence(Fence{Session: "owner", Generation: 7})
		_, err := c.Arm(context.Background())
		if (err != nil) != failAgain || calls.Load() != 2 || opened.Load() != 2 || <-bodies != <-bodies {
			t.Fatalf("calls=%d connections=%d error=%v", calls.Load(), opened.Load(), err)
		}
		c.Close()
		server.Close()
	}
}

// TestConnectionReopenedDelayedLiveResponse retains delayed requests until their
// response or timeout; neither a slow live peer nor a header timeout permits replay.
func TestConnectionReopenedDelayedLiveResponse(t *testing.T) {
	for _, reserved := range []bool{false, true} {
		for _, phase := range []string{"headers", "body", "header_timeout"} {
			t.Run(strconv.FormatBool(reserved)+"/"+phase, func(t *testing.T) {
				var calls, opened atomic.Int32
				entered, release := make(chan struct{}), make(chan struct{})
				var enteredOnce, releaseOnce sync.Once
				unblock := func() { releaseOnce.Do(func() { close(release) }) }
				server := httptest.NewUnstartedServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
					raw, _ := json.Marshal(Response{Schema: "rosie.rt-control.response.v1", Data: json.RawMessage(`{}`)})
					if r.Method != http.MethodPost {
						w.Write(raw)
						return
					}
					io.Copy(io.Discard, r.Body)
					calls.Add(1)
					if phase == "body" {
						w.Header().Set("Content-Length", strconv.Itoa(len(raw)))
						w.Write(raw[:1])
						w.(http.Flusher).Flush()
						raw = raw[1:]
					}
					enteredOnce.Do(func() { close(entered) })
					select {
					case <-release:
						w.Write(raw)
					case <-r.Context().Done():
					}
				}))
				server.Config.ConnContext = func(ctx context.Context, _ net.Conn) context.Context { opened.Add(1); return ctx }
				server.Start()
				defer server.Close()
				transport := &http.Transport{}
				if phase == "header_timeout" {
					// Unverified injected header timeout: 300 ms; request counts prove no retry.
					transport.ResponseHeaderTimeout = 300 * time.Millisecond
				}
				c := newClient(server.URL, transport)
				defer c.Close()
				// Unverified watchdog: 5000 ms; gates and receipts establish completion.
				ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
				defer cancel()
				defer unblock()
				if reserved {
					must(t, c.warmConnections(ctx, true))
				}
				c.SetFence(Fence{Session: "owner", Generation: 7})
				done := make(chan error, 1)
				go func() { _, err := c.Arm(ctx); done <- err }()
				select {
				case <-entered:
				case <-ctx.Done():
					t.Fatal(ctx.Err())
				}
				wantConnections := int32(1)
				if phase != "header_timeout" {
					_, err := c.Status(ctx)
					must(t, err)
					wantConnections = 2
					if reserved {
						must(t, c.warmConnections(ctx, false))
					}
					select {
					case err := <-done:
						t.Fatalf("delayed live request completed before release: %v", err)
					default:
					}
					unblock()
				}
				select {
				case err := <-done:
					if phase == "header_timeout" {
						if !errors.Is(err, ErrTransport) || errors.Is(err, ErrConnectionUnsent) || ctx.Err() != nil {
							t.Fatalf("header timeout classification: %v; context: %v", err, ctx.Err())
						}
					} else {
						must(t, err)
					}
				case <-ctx.Done():
					t.Fatal(ctx.Err())
				}
				if reserved {
					wantConnections = 7
				}
				if calls.Load() != 1 || opened.Load() != wantConnections || c.Fence().Generation != 7 {
					t.Fatalf("calls=%d connections=%d fence=%+v", calls.Load(), opened.Load(), c.Fence())
				}
			})
		}
	}
}

// TestConnectionReopenedRequiresCloseEvidence gives timeouts precedence over
// wrapped EOFs, so ambiguous transport failures fail closed without replay.
func TestConnectionReopenedRequiresCloseEvidence(t *testing.T) {
	for _, tc := range []struct {
		name   string
		err    error
		closed bool
	}{
		{"fresh_eof", io.EOF, true},
		{"truncated_response", io.ErrUnexpectedEOF, true},
		{"reset", syscall.ECONNRESET, true},
		{"broken_pipe", syscall.EPIPE, true},
		{"lost_reservation", ErrWarmConnection, true},
		{"deadline", context.DeadlineExceeded, false},
		{"canceled_eof", errors.Join(context.Canceled, io.EOF), false},
		{"deadline_eof", errors.Join(context.DeadlineExceeded, io.EOF), false},
		{"socket_timeout_eof", &net.OpError{Op: "read", Net: "tcp", Err: errors.Join(os.ErrDeadlineExceeded, io.EOF)}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			if got := connectionClosed(tc.err); got != tc.closed {
				t.Fatalf("connectionClosed(%v)=%v want=%v", tc.err, got, tc.closed)
			}
		})
	}
}
