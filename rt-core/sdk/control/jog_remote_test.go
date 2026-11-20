//go:build linux

package control

import (
	"bufio"
	"context"
	"crypto/tls"
	"encoding/json"
	"errors"
	"io"
	"net"
	"net/http"
	"net/http/httptest"
	"os/exec"
	"path/filepath"
	"strings"
	"sync"
	"syscall"
	"testing"
	"time"

	adapter "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
)

// Unverified simulation qualification only, matching jog_ws_test.go: uncertainty
// 5,000,000 ns, drift 1 ppb, calibration age 1,000,000,000 ns. P0-10/P8-04 bench
// qualification must supply measured values before remote motion deployment.
var remoteJogTestFlags = []string{"--remote-jog-max-uncertainty-ns", "5000000", "--remote-jog-drift-ppb", "1", "--remote-jog-calibration-max-age-ns", "1000000000"}

func remoteJogService(t *testing.T, qualified bool) (*service, testPKI, string, *Client) {
	t.Helper()
	p := newPKI(t)
	addr, args := p.remoteArgs(t)
	if qualified {
		args = append(args, remoteJogTestFlags...)
	}
	s := startService(t, args...)
	c, err := DialTLS(addr, &tls.Config{RootCAs: p.roots, Certificates: []tls.Certificate{p.client}})
	must(t, err)
	t.Cleanup(func() { c.Close() })
	return s, p, addr, c
}

func remoteJogReady(t *testing.T, s *service, c *Client, ctx context.Context) {
	t.Helper()
	_, err := c.Acquire(ctx, "jog-conformance", Binding{PairID: "bench", Revision: 7, ConfigurationSHA256: s.hash})
	must(t, err)
	// Unverified fixture renewal cadence: 50 ms within the 500 ms grant lease.
	_, err = c.StartRenewal(ctx, 50*time.Millisecond)
	must(t, err)
	_, err = c.Enable(ctx, 511)
	must(t, err)
	_, err = c.Arm(ctx)
	must(t, err)
	// Unverified readiness observation ceiling: 3000 ms.
	poll(t, 3*time.Second, func() bool {
		o, e := c.Status(ctx)
		return e == nil && o.Core.Armed == 1 && o.Core.AxisEnableMask == 511 && ipcclient.AllOperationEnabled(o.Core)
	})
}

// TestRemoteJogProducerConformance exercises the public TLS boundary and native
// observations; write success alone cannot satisfy an input admission assertion.
func TestRemoteJogProducerConformance(t *testing.T) {
	s, _, _, c := remoteJogService(t, true)
	// Unverified total failure ceiling: 10000 ms; all success is observed below.
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	remoteJogReady(t, s, c, ctx)
	// Unverified source offset stimulus: 1,000,000,000 ns behind host monotonic.
	source := func() uint64 { return ipcclient.HostMonotonicNS() - 1_000_000_000 }
	// Unverified input lifetime: 100 ms. The unverified 5 ms cadence leaves
	// margin within the 100 ms input lifetime and 250 ms stream idle deadline.
	j, err := c.NewRemoteJogSession(ctx, 511, 100_000_000, source)
	must(t, err)
	defer j.End(ctx)
	ticker := time.NewTicker(5 * time.Millisecond)
	defer ticker.Stop()
	velocity := make([]float64, 9)
	velocity[0] = 0.05
	var last JogObservation
	for sequence := uint64(1); sequence <= 4; sequence++ {
		select {
		case <-ticker.C:
		case <-ctx.Done():
			t.Fatal(ctx.Err())
		}
		must(t, j.Update(velocity, source()))
		// Unverified per-sample observation ceiling: 1000 ms, absolute count.
		poll(t, time.Second, func() bool {
			o, e := j.Observe(ctx)
			if e == nil && o.HasInput == 1 {
				last = o
			}
			local, localErr := s.client.JogStatus(ctx)
			return e == nil && o.HasInput == 1 && o.ObservedSourceSequence == sequence && localErr == nil && local.HasInput == 1 && local.ObservedSourceSequence == sequence
		})
	}
	must(t, remoteJogTestFrame(j, 4, source(), velocity, false))
	err = j.Rejection(ctx)
	var rejected *JogRejection
	if !errors.As(err, &rejected) || rejected.Reason != "jog_native_rejected_6" || rejected.NativeReason != ipcclient.JogReasonStaleSequence || rejected.Sequence != 4 {
		t.Fatalf("stale sequence: %v", err)
	}
	must(t, remoteJogTestFrame(j, 5, source(), velocity, true))
	err = j.Rejection(ctx)
	if !errors.As(err, &rejected) || rejected.Reason != adapter.ReasonJogSessionStale || rejected.Sequence != 5 {
		t.Fatalf("out-of-fence frame: %v", err)
	}
	poll(t, time.Second, func() bool {
		// RFC 6455 unsolicited pong is accepted without refreshing native input.
		j.writeMu.Lock()
		e := j.send(10, nil)
		j.writeMu.Unlock()
		must(t, e)
		o := observeRemoteJogExpiry(t, j, ctx, 4)
		if o.HasInput == 1 {
			last = o
		}
		return o.StateReason == ipcclient.JogReasonExpired && o.Open == 0 && o.HasInput == 0 && o.ObservedSourceSequence == 4
	})
	// A receiver reply after closure proves WSS remained live through native
	// expiry, independently of the stream idle deadline.
	must(t, j.send(1, []byte(`{}`)))
	err = j.Rejection(ctx)
	if !errors.As(err, &rejected) || rejected.Reason != adapter.ReasonJogClockInvalidBudgetOrExchange {
		t.Fatalf("live stream after input deadline: %v", err)
	}
	o := observeRemoteJogExpiry(t, j, ctx, 4)
	if last.InputDeadlineHostNs == 0 || o.NowHostNs < last.InputDeadlineHostNs || o.StateReason != ipcclient.JogReasonExpired {
		t.Fatalf("native expiry observation: %+v, input: %+v", o, last)
	}
	err = j.End(ctx)
	var ended *Rejected
	if err != nil && (!errors.As(err, &ended) || ended.NativeJogResult == nil || ended.NativeJogResult.ControlReason != ipcclient.JogReasonClosed) {
		t.Fatalf("End after expiry: %v", err)
	}
	_, err = c.Release(ctx)
	must(t, err)
	t.Log("remote Go: 4 samples at 5 ms cadence; stale sequence jog_native_rejected_6; out-of-fence jog_session_stale; EndJog and release")
}

// The idle deadline is a transport rule: a typed refusal and EndJog ticket establish
// receiver cleanup independently of the native input deadline or client End.
func TestRemoteJogReceiverIdleClose(t *testing.T) {
	s, _, _, c := remoteJogService(t, true)
	// Unverified test failure ceiling: 10000 ms.
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	remoteJogReady(t, s, c, ctx)
	// Unverified input lifetime: 100,000,000 ns, below the 250 ms idle deadline.
	j, err := c.NewRemoteJogSession(ctx, 511, 100_000_000, nil)
	must(t, err)
	defer j.End(ctx)
	// First observe native closure while keeping WSS open. Idle cleanup must then
	// be a distinct transition, identified by its new EndJog control ticket.
	must(t, j.Update(make([]float64, 9), ipcclient.HostMonotonicNS()))
	poll(t, time.Second, func() bool {
		o, err := j.Observe(ctx)
		must(t, err)
		return o.HasInput == 1 && o.ObservedSourceSequence == 1
	})
	var before JogObservation
	poll(t, time.Second, func() bool {
		must(t, j.send(10, nil))
		before = observeRemoteJogExpiry(t, j, ctx, 1)
		return before.StateReason == ipcclient.JogReasonExpired && before.Open == 0
	})
	// This malformed calibration returns a reason and starts a known idle window.
	start := time.Now()
	must(t, j.send(1, []byte(`{}`)))
	var rejected *JogRejection
	if err := j.Rejection(ctx); !errors.As(err, &rejected) || rejected.Reason != adapter.ReasonJogClockInvalidBudgetOrExchange {
		t.Fatalf("idle marker: %v", err)
	}
	err = j.Rejection(ctx)
	if !errors.As(err, &rejected) || rejected.Reason != adapter.ReasonJogStreamIdle || rejected.Sequence != 0 {
		t.Fatalf("idle receiver close: %v", err)
	}
	if time.Since(start) < 250*time.Millisecond {
		t.Fatal("receiver closed before its 250 ms idle deadline")
	}
	if err := j.Rejection(ctx); !errors.Is(err, io.EOF) {
		t.Fatalf("EOF after idle refusal: %v", err)
	}
	poll(t, time.Second, func() bool {
		o := observeRemoteJogExpiry(t, j, ctx, 1)
		return o.Ticket == before.Ticket+1 && o.ControlAccepted == 1 && o.ControlReason == ipcclient.JogReasonNone && o.StateReason == ipcclient.JogReasonNone && o.Open == 0 && o.HasInput == 0
	})
	t.Log("idle close: jog_stream_idle before EOF; receiver EndJog ticket +1, accepted=1, control_reason=0, state_reason=0 (None), open=0")
	_, err = c.Release(ctx)
	must(t, err)
}

// Observe must surface native Expired as a typed rejection while preserving the
// observation; an elapsed deadline alone must never manufacture that refusal.
func observeRemoteJogExpiry(t *testing.T, j *RemoteJogSession, ctx context.Context, sequence uint64) JogObservation {
	t.Helper()
	o, err := j.Observe(ctx)
	if o.StateReason != ipcclient.JogReasonExpired {
		must(t, err)
		return o
	}
	var rejected *JogRejection
	if !errors.As(err, &rejected) || rejected.Reason != adapter.ReasonJogInputDeadlineExpired || rejected.NativeReason != 9 || rejected.Sequence != sequence {
		t.Fatalf("native expiry refusal: %+v %v", o, err)
	}
	return o
}

// Preserve native reasons even after a previously observed input deadline; End,
// Stop and idle cleanup must not be relabelled as input expiry by the SDK.
func TestRemoteJogObservePreservesReason(t *testing.T) {
	o := JogObservation{JogGeneration: 9, AppGrantGeneration: 7, ObservedSourceSequence: 4, InputDeadlineHostNs: 100, NowHostNs: 101, HasInput: 1, Open: 1}
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/v1/status" {
			t.Errorf("unexpected path: %s", r.URL.Path)
		}
		body, _ := json.Marshal(Status{Jog: &o})
		json.NewEncoder(w).Encode(Response{Schema: adapter.ResponseSchema, Operation: "status", Data: body})
	}))
	defer server.Close()
	c := &Client{base: server.URL, control: server.Client()}
	j := &RemoteJogSession{client: c, fence: Fence{Generation: 7}, generation: 9}
	_, err := j.Observe(context.Background())
	must(t, err)
	o.HasInput, o.Open = 0, 0
	for _, reason := range []uint32{ipcclient.JogReasonClosed, ipcclient.JogReasonRamping, ipcclient.JogReasonAuthorityLost, ipcclient.JogReasonExpired} {
		o.StateReason = reason
		got, err := j.Observe(context.Background())
		if got.StateReason != reason {
			t.Fatalf("native reason changed: %+v", got)
		}
		if reason != ipcclient.JogReasonExpired {
			must(t, err)
			continue
		}
		var rejected *JogRejection
		if !errors.As(err, &rejected) || rejected.Reason != adapter.ReasonJogInputDeadlineExpired || rejected.NativeReason != 9 || rejected.Sequence != 4 {
			t.Fatalf("explicit native expiry: %v", err)
		}
	}
}

// remoteJogTestFrame deliberately bypasses producer guards only for conformance;
// wire refusals must come from the receiver, never a locally fabricated label.
func remoteJogTestFrame(j *RemoteJogSession, seq, source uint64, velocity []float64, wrongGeneration bool) error {
	j.writeMu.Lock()
	defer j.writeMu.Unlock()
	generation := j.generation
	if wrongGeneration {
		generation++
	}
	frame, err := BuildJogFrame(j.fence, j.clock, generation, seq, j.mask, velocity, source, source+j.duration)
	if err != nil {
		return err
	}
	return j.send(2, frame[:])
}

func TestRemoteJogDefaultQualificationAndStopFence(t *testing.T) {
	s, _, _, c := remoteJogService(t, false)
	// Unverified test failure ceiling: 10000 ms.
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	remoteJogReady(t, s, c, ctx)
	j, err := c.NewRemoteJogSession(ctx, 511, 100_000_000, nil)
	must(t, err)
	defer j.End(ctx)
	must(t, j.Update(make([]float64, 9), ipcclient.HostMonotonicNS()))
	var rejected *JogRejection
	err = j.Rejection(ctx)
	if !errors.As(err, &rejected) || rejected.Reason != adapter.ReasonJogClockUnqualified {
		t.Fatalf("default qualification: %v", err)
	}
	old := j.fence
	grant, err := c.Stop(ctx)
	must(t, err)
	if grant.Generation != old.Generation+1 {
		t.Fatal("Stop generation did not advance by one")
	}
	err = j.Update(make([]float64, 9), ipcclient.HostMonotonicNS())
	if !errors.As(err, &rejected) || rejected.Reason != adapter.ReasonControlSessionStale || j.fence != old {
		t.Fatalf("Stop fence: %v", err)
	}
	// Exercise the retired fence on the wire; Stop must send its catalogue reason
	// before EOF even if the connection has already stopped accepting writes.
	writeErr := remoteJogTestFrame(j, 2, ipcclient.HostMonotonicNS(), make([]float64, 9), false)
	err = j.Rejection(ctx)
	if !errors.As(err, &rejected) || rejected.Reason != adapter.ReasonControlSessionStale || rejected.Sequence != 0 {
		t.Fatalf("post-Stop receiver closure: write=%v read=%v", writeErr, err)
	}
	// The retired frame may still sit in the receiver's queue when it closes, so the
	// close can arrive as a reset instead of EOF; both end the stream after the refusal.
	if err := j.Rejection(ctx); !errors.Is(err, io.EOF) && !errors.Is(err, syscall.ECONNRESET) {
		t.Fatalf("EOF or reset after Stop refusal: %v", err)
	}
	err = j.End(ctx)
	var refusal *Rejected
	if !errors.As(err, &refusal) || refusal.Reason != adapter.ReasonControlSessionStale {
		t.Fatalf("End adopted Stop fence: %v", err)
	}
	_, err = c.Release(ctx)
	must(t, err)
}

// authenticatedCPPStream supplies the existing C++ HttpStream hook with a test
// TLS bridge. It forwards bytes unchanged: C++ performs upgrade/calibration and
// generated-frame encoding; Go supplies verified TLS without a new C++ library.
func authenticatedCPPStream(t *testing.T, s *service, p testPKI, address string) string {
	t.Helper()
	path := filepath.Join(s.dir, "tls.sock")
	listener, err := net.Listen("unix", path)
	must(t, err)
	var wg sync.WaitGroup
	ctx, cancel := context.WithCancel(context.Background())
	t.Cleanup(func() { cancel(); listener.Close(); wg.Wait() })
	wg.Add(1)
	go func() {
		defer wg.Done()
		for {
			local, err := listener.Accept()
			if err != nil {
				return
			}
			wg.Add(1)
			go func() {
				defer wg.Done()
				defer local.Close()
				dialer := tls.Dialer{Config: &tls.Config{RootCAs: p.roots, Certificates: []tls.Certificate{p.client}, MinVersion: tls.VersionTLS13, NextProtos: []string{"http/1.1"}}}
				remote, err := dialer.DialContext(ctx, "tcp", address)
				if err != nil {
					return
				}
				defer remote.Close()
				stop := context.AfterFunc(ctx, func() { local.Close(); remote.Close() })
				defer stop()
				done := make(chan struct{})
				go func() { io.Copy(remote, local); remote.Close(); close(done) }()
				io.Copy(local, remote)
				local.Close()
				<-done
			}()
		}
	}()
	return path
}

func TestRemoteJogCPPConformance(t *testing.T) {
	s, p, addr, _ := remoteJogService(t, true)
	path := authenticatedCPPStream(t, s, p, addr)
	binary := simfixture.CompileCPP(t, s.root, s.build, "clients/cpp/examples/remote_jog_sequence.cpp", "remote-jog-sequence")
	// Unverified process failure ceiling: 15000 ms.
	ctx, cancel := context.WithTimeout(context.Background(), 15*time.Second)
	defer cancel()
	command := exec.CommandContext(ctx, binary, path, addr, s.hash, s.socket)
	if raw, err := command.CombinedOutput(); err != nil {
		t.Fatalf("C++ conformance: %v\n%s", err, raw)
	} else if !strings.Contains(string(raw), "remote C++ conformance complete") {
		t.Fatalf("missing result: %s", raw)
	} else {
		t.Log(string(raw))
	}
}

// The fixed remote handler reserves /v1/jog for upgrade. Record its exact plain
// GET refusal so consumers cannot accidentally depend on a local-only route.
func TestRemoteJogObservationRoute(t *testing.T) {
	_, _, _, c := remoteJogService(t, true)
	// Unverified HTTP observation failure bound: 1000 ms.
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	request, err := http.NewRequestWithContext(ctx, "GET", c.base+"/v1/jog", nil)
	must(t, err)
	response, err := c.control.Do(request)
	must(t, err)
	defer response.Body.Close()
	body, err := io.ReadAll(io.LimitReader(response.Body, 4096))
	must(t, err)
	if response.StatusCode != http.StatusConflict || strings.TrimSpace(string(body)) != adapter.ReasonJogSessionStale {
		t.Fatalf("plain remote GET /v1/jog: %d %q", response.StatusCode, body)
	}
}

// Malformed peer framing must close the connection; otherwise partial bytes
// could be interpreted as a later calibration or refusal on this stream.
func TestRemoteJogMalformedRefusalClosesStream(t *testing.T) {
	for name, wire := range map[string][]byte{
		"fragment":         {0x01, 0},
		"masked":           {0x81, 0x80},
		"noncanonical":     {0x81, 126, 0, 1},
		"oversize":         {0x81, 126, 0x10, 1},
		"wide_length":      {0x81, 127},
		"invalid_json":     {0x81, 1, '{'},
		"unexpected_reply": append([]byte{0x81, 2}, []byte("{}")...),
	} {
		t.Run(name, func(t *testing.T) {
			client, peer := net.Pipe()
			defer client.Close()
			defer peer.Close()
			done := make(chan struct{})
			go func() { defer close(done); peer.Write(wire) }()
			session := &RemoteJogSession{conn: client, reader: bufio.NewReader(client)}
			// Unverified parser failure ceiling: 1000 ms; no sleep establishes success.
			ctx, cancel := context.WithTimeout(context.Background(), time.Second)
			defer cancel()
			if err := session.Rejection(ctx); !errors.Is(err, ErrProtocol) {
				t.Fatalf("malformed refusal: %v", err)
			}
			if _, err := client.Write([]byte{0}); !errors.Is(err, io.ErrClosedPipe) {
				t.Fatalf("stream survived malformed response: %v", err)
			}
			<-done
		})
	}
}
