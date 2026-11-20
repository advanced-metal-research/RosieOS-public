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
	"runtime"
	"strconv"
	"strings"
	"sync/atomic"
	"testing"
	"time"
)

// TestHalfOpenFreshDialDeadline requires even transport-detached connection
// establishment to observe the negotiated deadline instead of an OS timeout.
func TestHalfOpenFreshDialDeadline(t *testing.T) {
	seen := make(chan time.Duration, 2)
	c := newClient("http://localhost", &http.Transport{DialContext: func(ctx context.Context, _, _ string) (net.Conn, error) {
		deadline, ok := ctx.Deadline()
		if !ok {
			seen <- -1
			return nil, errors.New("dial has no deadline")
		}
		seen <- time.Until(deadline)
		<-ctx.Done()
		return nil, ctx.Err()
	}})
	defer c.Close()
	// Unverified fixture: 250 ms RTT and 500 ms lease; each dial gets 1250 ms.
	c.roundTrip, c.leaseCeiling = 250*time.Millisecond, 500*time.Millisecond
	_, err := c.Arm(context.Background())
	if !errors.Is(err, ErrConnectionUnsent) || !errors.Is(err, context.DeadlineExceeded) {
		t.Fatalf("dial outcome: %v", err)
	}
	count := 0
	for len(seen) != 0 {
		budget := <-seen
		count++
		if budget <= 0 || budget > 1250*time.Millisecond {
			t.Fatalf("dial budget_ns=%d", budget)
		}
	}
	if count == 0 || count > 2 {
		t.Fatalf("dial attempts=%d", count)
	}
}

// TestHalfOpenRequestDeadline requires a silent peer to release the caller even
// without a caller deadline; the stack identifies the read retaining the lane.
func TestHalfOpenRequestDeadline(t *testing.T) {
	entered := make(chan struct{}, 2)
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		io.Copy(io.Discard, r.Body)
		entered <- struct{}{}
		<-r.Context().Done()
	}))
	defer server.Close()
	c := newClient(server.URL, &http.Transport{})
	defer c.Close()
	// Unverified test timing: 250 ms RTT, 500 ms lease, 1250 ms per attempt.
	c.roundTrip, c.leaseCeiling = 250*time.Millisecond, 500*time.Millisecond
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	done := make(chan error, 1)
	go func() { _, err := c.Arm(ctx); done <- err }()
	<-entered
	stack := make([]byte, 1<<20)
	n := runtime.Stack(stack, true)
	if !strings.Contains(string(stack[:n]), "(*Client).call") {
		t.Fatal("missing request goroutine")
	}
	t.Logf("half-open request stack:\n%s", stack[:n])
	// Unverified absolute watchdog: two 1250 ms attempts must finish by 3500 ms.
	select {
	case err := <-done:
		if !errors.Is(err, context.DeadlineExceeded) {
			t.Fatalf("deadline classification: %v", err)
		}
	case <-time.After(3500 * time.Millisecond):
		cancel()
		<-done
		t.Fatal("half-open request retained its lane beyond 3500 ms")
	}
}

// TestHalfOpenReservedRetryPolicy requires exactly one fresh-socket replay with
// identical mutation bytes; uploads and Release retain uncertain outcomes.
func TestHalfOpenReservedRetryPolicy(t *testing.T) {
	for _, operation := range []string{"arm", "release", "upload"} {
		for _, phase := range []string{"headers", "body"} {
			t.Run(operation+"/"+phase, func(t *testing.T) {
				var calls atomic.Int32
				bodies := make(chan string, 2)
				server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
					raw, _ := json.Marshal(Response{Schema: "rosie.rt-control.response.v1", Data: json.RawMessage(`{}`)})
					if r.Method == "GET" {
						w.Write(raw)
						return
					}
					body, _ := io.ReadAll(r.Body)
					bodies <- string(body)
					if calls.Add(1) == 1 {
						if phase == "body" {
							w.Header().Set("Content-Length", strconv.Itoa(len(raw)))
							w.Write(raw[:1])
							w.(http.Flusher).Flush()
						}
						<-r.Context().Done()
						return
					}
					w.Write(raw)
				}))
				defer server.Close()
				c := newClient(server.URL, &http.Transport{})
				defer c.Close()
				// Unverified fixture budget: 25 ms RTT and 500 ms lease, 575 ms per attempt.
				c.roundTrip, c.leaseCeiling = 25*time.Millisecond, 500*time.Millisecond
				if err := c.warmConnections(context.Background(), true); err != nil {
					t.Fatal(err)
				}
				c.SetFence(Fence{Session: "original", Generation: 7})
				var err error
				if operation == "upload" {
					_, err = c.PrepareProgram(context.Background(), []byte("immutable"))
				} else {
					_, err = c.Command(context.Background(), &Request{Operation: operation})
				}
				if operation == "arm" {
					if err != nil || calls.Load() != 2 {
						t.Fatalf("reopen calls=%d err=%v", calls.Load(), err)
					}
					if <-bodies != <-bodies {
						t.Fatal("retry changed mutation bytes")
					}
				} else if !errors.Is(err, ErrRequestTimeout) || errors.Is(err, ErrConnectionUnsent) || calls.Load() != 1 {
					t.Fatalf("unsafe replay calls=%d err=%v", calls.Load(), err)
				}
			})
		}
	}
}

// TestHalfOpenStreamReadDeadline prevents a response with headers but no body
// from retaining an observation slot indefinitely after the caller starts reading.
func TestHalfOpenStreamReadDeadline(t *testing.T) {
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/event-stream")
		w.(http.Flusher).Flush()
		<-r.Context().Done()
	}))
	defer server.Close()
	c := newClient(server.URL, &http.Transport{})
	defer c.Close()
	// Unverified fixture timing: 25 ms RTT and 500 ms lease, 575 ms idle deadline.
	c.roundTrip, c.leaseCeiling = 25*time.Millisecond, 500*time.Millisecond
	started := time.Now()
	err := c.StreamEvents(context.Background(), 0, func(Event) error { t.Error("unexpected event"); return nil })
	// Unverified absolute stream result bound: 1000 ms, independent of measured RTT.
	if !errors.Is(err, ErrTransport) || time.Since(started) > 1000*time.Millisecond {
		t.Fatalf("silent stream: %v", err)
	}
}

// TestHomeRequestBudgetRetainsCommissioning permits a bounded synchronous Home
// while still refusing an unresponsive Arm under the ordinary call deadline.
func TestHomeRequestBudgetRetainsCommissioning(t *testing.T) {
	for _, operation := range []string{"home", "arm"} {
		t.Run(operation, func(t *testing.T) {
			server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				io.Copy(io.Discard, r.Body)
				// Unverified synchronous commissioning stimulus: 750 ms.
				timer := time.NewTimer(750 * time.Millisecond)
				defer timer.Stop()
				select {
				case <-timer.C:
				case <-r.Context().Done():
					return
				}
				json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Operation: operation})
			}))
			defer server.Close()
			c := newClient(server.URL, &http.Transport{})
			defer c.Close()
			// Unverified LAN fixture: 1 ns RTT, 500 ms lease.
			c.roundTrip, c.leaseCeiling = time.Nanosecond, 500*time.Millisecond
			_, err := c.Command(context.Background(), &Request{Operation: operation})
			if operation == "home" && err != nil {
				t.Fatal(err)
			}
			if operation == "arm" && !errors.Is(err, ErrRequestTimeout) {
				t.Fatalf("unbounded ordinary request: %v", err)
			}
		})
	}
}

// TestHalfOpenPreparationCancellation proves cancelled acquisition can leave
// reservation maintenance while its peer is silent, preventing a stuck Arm.
func TestHalfOpenPreparationCancellation(t *testing.T) {
	entered := make(chan struct{}, 7)
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		entered <- struct{}{}
		<-r.Context().Done()
	}))
	defer server.Close()
	c := newClient(server.URL, &http.Transport{})
	defer c.Close()
	// Unverified remote timing: 250 ms RTT and 3000 ms lease ceiling.
	c.roundTrip, c.leaseCeiling = 250*time.Millisecond, 3000*time.Millisecond
	maintenance, cancelMaintenance := context.WithCancel(context.Background())
	warmed := make(chan error, 1)
	go func() { warmed <- c.warmConnections(maintenance, true) }()
	defer func() { cancelMaintenance(); <-warmed }()
	<-entered
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { _, err := c.PrepareAcquisition(ctx); done <- err }()
	cancel()
	// Unverified cancellation watchdog: 500 ms; cancellation must release the wait.
	select {
	case err := <-done:
		if !errors.Is(err, context.Canceled) {
			t.Fatalf("cancellation: %v", err)
		}
	case <-time.After(500 * time.Millisecond):
		stack := make([]byte, 1<<20)
		n := runtime.Stack(stack, true)
		t.Logf("blocked preparation stack:\n%s", stack[:n])
		cancelMaintenance()
		<-done
		t.Fatal("cancelled preparation still waits on warmMu")
	}
}
