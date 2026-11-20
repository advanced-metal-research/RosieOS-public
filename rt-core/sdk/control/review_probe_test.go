//go:build linux

package control

import (
	"bufio"
	"context"
	"crypto/tls"
	"encoding/json"
	"fmt"
	"net/http"
	"net/http/httptest"
	"net/http/httputil"
	"net/url"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
)

// reviewInternetService starts only the simulator with compiled internet policy.
func reviewInternetService(t *testing.T, remoteArgs ...string) *service {
	t.Helper()
	root, err := filepath.Abs("../..")
	must(t, err)
	dir, err := os.MkdirTemp("", "rk-")
	must(t, err)
	t.Cleanup(func() { os.RemoveAll(dir) })
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "review-kill-")
	must(t, err)
	t.Cleanup(func() { os.RemoveAll(build) })
	raw, err := os.ReadFile(filepath.Join(root, "config/machines/simulation/simulation-program.json"))
	must(t, err)
	var cfg map[string]json.RawMessage
	must(t, json.Unmarshal(raw, &cfg))
	cfg["control"] = json.RawMessage(`{"link_profile":"internet"}`)
	raw, err = json.Marshal(cfg)
	must(t, err)
	input := filepath.Join(build, "internet.json")
	must(t, os.WriteFile(input, raw, 0600))
	compiled, args, err := simfixture.Compile(root, input, filepath.Join(build, "config"))
	must(t, err)
	coreSocket := filepath.Join(dir, "ipc.sock")
	core := process(t, build, "core", simfixture.Locate(root, "rosie-rt-core-sim"), append(args, "--socket-path", coreSocket)...)
	simfixture.WaitVerified(t, filepath.Join(dir, "metrics.json"), 3*time.Second, time.Millisecond, func(s simfixture.Verified) bool { return s.ConfigVerifiedMask == 511 })
	socket := filepath.Join(dir, "control.sock")
	args = simfixture.ControlArgs(coreSocket, socket, filepath.Base(compiled), "bench", 7)
	process(t, build, "control", simfixture.Locate(root, "rt-control"), append(args, remoteArgs...)...)
	client, err := Dial(UnixPath(socket))
	must(t, err)
	t.Cleanup(func() { client.Close() })
	poll(t, 3*time.Second, func() bool {
		ctx, cancel := context.WithTimeout(context.Background(), 100*time.Millisecond)
		defer cancel()
		state, err := client.Status(ctx)
		return err == nil && state.Motion.TimeNS != 0
	})
	return &service{client: client, core: core, root: root, dir: dir, build: build, socket: socket, hash: filepath.Base(compiled)}
}

// TestReviewKilledJogChild is killed by its parent without Stop or Release.
func TestReviewKilledJogChild(t *testing.T) {
	socket := os.Getenv("REVIEW_KILL_SOCKET")
	if socket == "" {
		t.Skip("child process only")
	}
	c, err := Dial(UnixPath(socket))
	must(t, err)
	ctx := context.Background()
	g, err := c.AcquireLease(ctx, "review-child", Binding{PairID: "bench", Revision: 7, ConfigurationSHA256: os.Getenv("REVIEW_KILL_HASH")}, 3*time.Second)
	must(t, err)
	if g.LeaseMS != 3000 {
		t.Fatalf("lease_ms=%d", g.LeaseMS)
	}
	_, err = c.StartRenewal(ctx, 0)
	must(t, err)
	_, err = c.Enable(ctx, 511)
	must(t, err)
	_, err = c.Arm(ctx)
	must(t, err)
	poll(t, 3*time.Second, func() bool {
		s, e := c.Status(ctx)
		return e == nil && s.Core.Armed == 1 && ipcclient.AllOperationEnabled(s.Core)
	})
	now := ipcclient.HostMonotonicNS()
	j, err := c.NewJogSessionAt(ctx, 1, now, now+750000000)
	must(t, err)
	// Unverified injection cadence: 10 ms; parent observes actual applied input.
	ticker := time.NewTicker(10 * time.Millisecond)
	for range ticker.C {
		now = ipcclient.HostMonotonicNS()
		must(t, j.UpdateAt([]float64{0.05}, now, now+750000000))
	}
}

// TestReviewKillMidJog requires a real killed client, a native expiry ramp and
// inhibited outputs after the exact native grant deadline.
func TestReviewKillMidJog(t *testing.T) {
	s := reviewInternetService(t)
	cmd := exec.Command(os.Args[0], "-test.run=^TestReviewKilledJogChild$", "-test.v")
	cmd.Env = append(os.Environ(), "REVIEW_KILL_SOCKET="+s.socket, "REVIEW_KILL_HASH="+s.hash)
	log, err := os.Create(filepath.Join(s.build, "child.log"))
	must(t, err)
	defer log.Close()
	cmd.Stdout, cmd.Stderr = log, log
	must(t, cmd.Start())
	defer cmd.Process.Kill()
	var active Status
	poll(t, 5*time.Second, func() bool {
		active, err = s.client.Status(context.Background())
		return err == nil && active.Grant != nil && active.Jog != nil && active.Jog.ObservedSourceSequence >= 15 && active.Jog.HasInput == 1 && active.Core.Armed == 1 && active.Core.Axes[0].PositionCounts >= 1
	})
	telemetry, err := s.client.TelemetryBatches(context.Background(), 0)
	must(t, err)
	cursor := telemetry.Header.LastSequence
	must(t, cmd.Process.Kill())
	if cmd.Wait() == nil {
		t.Fatal("child was not killed")
	}
	var ramp, inhibited Status
	inputDeadline := active.Jog.InputDeadlineHostNs
	poll(t, 2*time.Second, func() bool {
		ramp, err = s.client.Status(context.Background())
		if err == nil && ramp.Jog != nil && ramp.Jog.InputDeadlineHostNs > inputDeadline {
			inputDeadline = ramp.Jog.InputDeadlineHostNs
		}
		return err == nil && ramp.Jog != nil && ramp.Jog.StateReason == ipcclient.JogReasonRamping
	})
	if inputDeadline == 0 || ramp.Jog.NowHostNs < inputDeadline {
		t.Fatalf("ramp preceded capture expiry: %+v", ramp.Jog)
	}
	poll(t, 5*time.Second, func() bool {
		inhibited, err = s.client.Status(context.Background())
		return err == nil && inhibited.Grant != nil && inhibited.Grant.GrantActive == 0 && inhibited.Core.Armed == 0 && inhibited.Core.AxisEnableMask == 0
	})
	if inhibited.Grant.NowHostNs < active.Grant.DeadlineHostNs {
		t.Fatalf("authority lost before lease expiry: before=%+v after=%+v", active.Grant, inhibited.Grant)
	}
	// Metrics publish every 100000000 ns. Timestamp actual disabled feedback
	// from cyclic telemetry instead of combining slow metrics with a fresh grant.
	// Unverified inhibition allowance remains 50000000 ns after the grant deadline.
	var disabledNS uint64
	poll(t, 5*time.Second, func() bool {
		batch, e := s.client.TelemetryBatches(context.Background(), cursor)
		must(t, e)
		if batch.Header.Dropped != 0 {
			t.Fatal("lost inhibition telemetry")
		}
		cursor = batch.Header.LastSequence
		for _, record := range batch.Records {
			if record.Axes != 9 {
				t.Fatalf("axis count=%d, want 9", record.Axes)
			}
			disabled := true
			for _, axis := range record.Ax[:record.Axes] {
				disabled = disabled && axis.Statusword&0x6f != 0x27
			}
			if disabled {
				disabledNS = record.TNs
				return true
			}
		}
		return false
	})
	if disabledNS < active.Grant.DeadlineHostNs || disabledNS-active.Grant.DeadlineHostNs > 50000000 {
		t.Fatalf("inhibition outside grant bound: deadline_ns=%d disabled_ns=%d", active.Grant.DeadlineHostNs, disabledNS)
	}
	load, err := os.ReadFile("/proc/loadavg")
	must(t, err)
	t.Logf("inhibition_ns=%d deadline_ns=%d delta_ns=%d metrics_observation_delta_ns=%d loadavg=%s", disabledNS, active.Grant.DeadlineHostNs, disabledNS-active.Grant.DeadlineHostNs, inhibited.Grant.NowHostNs-active.Grant.DeadlineHostNs, strings.TrimSpace(string(load)))
	t.Logf("SIGKILL: sequence=%d position_counts=%d ramp_now_ns=%d input_deadline_ns=%d grant_before=%+v grant_after=%+v armed=%d enabled=%d", active.Jog.ObservedSourceSequence, active.Core.Axes[0].PositionCounts, ramp.Jog.NowHostNs, inputDeadline, active.Grant, inhibited.Grant, inhibited.Core.Armed, inhibited.Core.AxisEnableMask)
}

// TestReviewRemoteJogHTTPDelay requires first native input through the real TLS
// and WSS admission path, preventing clock-conversion-only tests from hiding expiry.
func TestReviewRemoteJogHTTPDelay(t *testing.T) {
	for _, test := range []struct {
		name             string
		delay, requested time.Duration
		wantLease        int
		duration         uint64
	}{
		{"local_auto_lease", 0, 0, 500, 100000000},
		{"zero_delay", 0, 3 * time.Second, 3000, 750000000},
		{"300ms", 300 * time.Millisecond, 3 * time.Second, 3000, 750000000},
		{"300ms_cpp", 300 * time.Millisecond, 3 * time.Second, 3000, 750000000},
	} {
		delay := test.delay
		t.Run(fmt.Sprint(test.name), func(t *testing.T) {
			p := newPKI(t)
			addr, args := p.remoteArgs(t)
			// Unverified simulator clock budgets, in ns, allow this injected transport.
			args = append(args, "--remote-jog-max-uncertainty-ns", "650000000", "--remote-jog-drift-ppb", "1", "--remote-jog-calibration-max-age-ns", "10000000000")
			s := reviewInternetService(t, args...)
			target, err := url.Parse("https://" + addr)
			must(t, err)
			proxy := httputil.NewSingleHostReverseProxy(target)
			transport := &http.Transport{TLSClientConfig: &tls.Config{RootCAs: p.roots, Certificates: []tls.Certificate{p.client}}}
			defer transport.CloseIdleConnections()
			proxy.Transport = transport
			proxy.ModifyResponse = func(r *http.Response) error {
				if delay > 0 {
					timer := time.NewTimer(delay)
					defer timer.Stop()
					select {
					case <-timer.C:
					case <-r.Request.Context().Done():
						return r.Request.Context().Err()
					}
				}
				return nil
			}
			cert, err := tls.LoadX509KeyPair(filepath.Join(p.dir, "server.pem"), filepath.Join(p.dir, "server-key.pem"))
			must(t, err)
			server := httptest.NewUnstartedServer(proxy)
			server.TLS = &tls.Config{Certificates: []tls.Certificate{cert}, ClientAuth: tls.RequireAndVerifyClientCert, ClientCAs: p.roots}
			server.StartTLS()
			defer server.Close()
			c, err := DialTLS(server.URL, &tls.Config{RootCAs: p.roots, Certificates: []tls.Certificate{p.client}})
			must(t, err)
			defer c.Close()
			var cppBinary string
			if strings.HasSuffix(test.name, "_cpp") {
				// Build setup is outside the live admission/lease watchdog.
				cppBinary = simfixture.CompileCPP(t, s.root, s.build, "tests/remote_jog_setup_probe.cpp", "remote-jog-setup")
			}
			// Unverified overall probe watchdog: 15000 ms; native receipts establish success.
			ctx, cancel := context.WithTimeout(context.Background(), 15*time.Second)
			defer cancel()
			g, err := c.AcquireLease(ctx, "review-remote", Binding{PairID: "bench", Revision: 7, ConfigurationSHA256: s.hash}, test.requested)
			must(t, err)
			if g.LeaseMS != test.wantLease {
				t.Fatalf("lease_ms=%d", g.LeaseMS)
			}
			_, err = c.StartRenewal(ctx, 0)
			must(t, err)
			_, err = c.Enable(ctx, 511)
			must(t, err)
			_, err = c.Arm(ctx)
			must(t, err)
			s.waitStatus(t, func(v Status) bool { return v.Core.Armed == 1 && ipcclient.AllOperationEnabled(v.Core) })
			var goJog *RemoteJogSession
			var beginSource uint64
			if strings.HasSuffix(test.name, "_cpp") {
				address := strings.TrimPrefix(server.URL, "https://")
				bridge := authenticatedCPPStream(t, s, p, address)
				cmd := exec.CommandContext(ctx, cppBinary, bridge, address, g.Session, fmt.Sprint(g.Generation))
				stdin, e := cmd.StdinPipe()
				must(t, e)
				stdout, e := cmd.StdoutPipe()
				must(t, e)
				cmd.Stderr = os.Stderr
				must(t, cmd.Start())
				defer func() { fmt.Fprint(stdin, "e"); stdin.Close(); must(t, cmd.Wait()) }()
				reader := bufio.NewScanner(stdout)
				if !reader.Scan() || !strings.HasPrefix(reader.Text(), "sent ") {
					t.Fatal("C++ first input missing")
				}
			} else {
				setupSamples := 0
				setupComplete := false
				inputClockReads := 0
				j, e := c.NewRemoteJogSession(ctx, 1, test.duration, func() uint64 {
					// The SDK retains this clock for input freshness and calibration
					// refresh after Begin. Only setup samples require native closed.
					if setupComplete {
						inputClockReads++
						return ipcclient.HostMonotonicNS()
					}
					before, e := s.client.Status(ctx)
					must(t, e)
					if before.Jog != nil && (before.Jog.Open != 0 || before.Jog.ObservedSourceSequence != 0) {
						t.Fatalf("native jog opened before calibration completed: %+v", before.Jog)
					}
					setupSamples++
					source := ipcclient.HostMonotonicNS()
					if setupSamples == 3 {
						beginSource = source
					}
					return source
				})
				must(t, e)
				setupComplete = true
				if setupSamples != 3 {
					t.Fatalf("calibration and Begin samples=%d", setupSamples)
				}
				defer j.End(ctx)
				goJog = j
				must(t, j.Update([]float64{.01}, ipcclient.HostMonotonicNS()))
				if inputClockReads == 0 {
					t.Fatal("input freshness did not consult the session clock")
				}
			}
			var observed Status
			poll(t, time.Second, func() bool {
				observed, err = s.client.Status(ctx)
				return err == nil && observed.Jog != nil && (observed.Jog.ObservedSourceSequence == 1 || observed.Jog.Open == 0)
			})
			if observed.Jog.ObservedSourceSequence != 1 || observed.Jog.HasInput != 1 {
				t.Fatalf("first input not applied: %+v", observed.Jog)
			}
			if goJog != nil {
				must(t, remoteJogTestFrame(goJog, 2, beginSource-1, []float64{.01}, false))
				refusal := goJog.Rejection(ctx)
				if refusal == nil || refusal.Error() != "jog_clock_moved_backwards" {
					t.Fatalf("pre-Begin capture: %v", refusal)
				}
			}
			t.Logf("delay=%s first input applied sequence=%d", delay, observed.Jog.ObservedSourceSequence)
		})
	}
}
