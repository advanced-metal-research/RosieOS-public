//go:build linux

package denseexec

import (
	"context"
	"crypto/tls"
	"crypto/x509"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"syscall"
	"testing"
	"time"

	codec "rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/sdk/control"
	"rosieos/rt-core/simfixture"
)

// TestRTCoreComposed executes through the OLP controller and both real public
// service processes. Opt-in prevents an absent build from becoming a fake pass.
func rtComposedFixture(t *testing.T, adjust ...func(map[string]any)) (*control.Client, control.Description, string, string) {
	t.Helper()
	c, d, root, socket, _ := rtComposedTransportFixture(t, false, adjust...)
	return c, d, root, socket
}

func TestRTCoreComposed(t *testing.T) {
	runRTCoreComposed(t, false)
}

// The HTTPS variant uses production PKI and the real remote listener, preventing
// Unix-socket coverage from masking remote principal or transport failures.
func TestRTCoreComposedHTTPS(t *testing.T) {
	runRTCoreComposed(t, true)
}

func rtComposedTransportFixture(t *testing.T, remote bool, adjust ...func(map[string]any)) (*control.Client, control.Description, string, string, RTCoreConfig) {
	t.Helper()
	return rtComposedMachineFixture(t, remote, "config/machines/simulation/simulation-program.json", nil, adjust...)
}

// rtComposedMachineFixture starts the simulator on machine (relative to
// rt-core) with simArgs appended to its compiled arguments.
func rtComposedMachineFixture(t *testing.T, remote bool, machine string, simArgs []string, adjust ...func(map[string]any)) (*control.Client, control.Description, string, string, RTCoreConfig) {
	t.Helper()
	if os.Getenv("RTCORE_COMPOSED") != "1" {
		t.Skip("set RTCORE_COMPOSED=1 after make control sim in rt-core")
	}
	root, err := filepath.Abs("../../../../rt-core")
	if err != nil {
		t.Fatal(err)
	}
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "olp-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(build) })
	dir, err := os.MkdirTemp("", "olp-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	config := filepath.Join(root, machine)
	// Every configured axis is commanded; the nine-axis fixtures map all nine.
	axisMask := uint32(511)
	if len(adjust) != 0 {
		raw, err := os.ReadFile(config)
		if err != nil {
			t.Fatal(err)
		}
		var fixture map[string]any
		if err := json.Unmarshal(raw, &fixture); err != nil {
			t.Fatal(err)
		}
		for _, edit := range adjust {
			edit(fixture)
		}
		if axes, ok := fixture["axes"].([]any); ok && len(axes) > 0 && len(axes) <= 9 {
			axisMask = 1<<len(axes) - 1
		}
		raw, err = json.Marshal(fixture)
		if err != nil {
			t.Fatal(err)
		}
		config = filepath.Join(build, "joint-fixture.json")
		if err := os.WriteFile(config, raw, 0600); err != nil {
			t.Fatal(err)
		}
	}
	compiled, args, err := simfixture.Compile(root, config, filepath.Join(build, "config"))
	if err != nil {
		t.Fatal(err)
	}
	spawn := func(name string, args ...string) *exec.Cmd {
		t.Helper()
		log, err := os.Create(filepath.Join(build, name+".log"))
		if err != nil {
			t.Fatal(err)
		}
		cmd := exec.Command(filepath.Join(root, "build", name), args...)
		cmd.Dir = build
		cmd.Stdout, cmd.Stderr = log, log
		if err := cmd.Start(); err != nil {
			log.Close()
			t.Fatal(err)
		}
		t.Cleanup(func() {
			_ = cmd.Process.Signal(syscall.SIGTERM)
			done := make(chan error, 1)
			go func() { done <- cmd.Wait() }()
			select {
			case err := <-done:
				if err != nil {
					t.Errorf("%s exit: %v", name, err)
				}
			case <-time.After(5000 * time.Millisecond):
				_ = cmd.Process.Kill()
				<-done
				t.Errorf("%s did not terminate", name)
			}
			log.Close()
			if t.Failed() {
				data, _ := os.ReadFile(log.Name())
				if len(data) > 1500 {
					data = data[len(data)-1500:]
				}
				t.Logf("%s: %s", name, data)
			}
		})
		return cmd
	}
	coreSocket, socket := filepath.Join(dir, "ipc.sock"), filepath.Join(dir, "control.sock")
	args = append(args, simArgs...)
	spawn("rosie-rt-core-sim", append(args, "--socket-path", coreSocket)...)
	rtAwait(t, func() bool {
		raw, _ := os.ReadFile(filepath.Join(dir, "metrics.json"))
		var m struct {
			ConfigVerifiedMask uint32 `json:"config_verified_mask"`
		}
		return json.Unmarshal(raw, &m) == nil && m.ConfigVerifiedMask == axisMask
	})
	controlArgs := []string{"--core-socket", coreSocket, "--socket", socket, "--compiled-config", compiled, "--configuration-sha256", filepath.Base(compiled), "--backend", "simulation", "--pair-id", "bench", "--pair-revision", "7"}
	var pki, pkiRoot string
	if remote {
		pkiRoot, pki = remotePKI(t, "bench")
		pkiCommand(t, root, pkiRoot, "issue-client", pki, "revoked")
		// A same-CA certificate with the wrong pair must fail SAN authorization.
		if err := os.WriteFile(filepath.Join(pki, "pair-id"), []byte("other-pair\n"), 0600); err != nil {
			t.Fatal(err)
		}
		pkiCommand(t, root, pkiRoot, "issue-client", pki, "wrong-pair")
		controlArgs = append(controlArgs, "--remote-listen", "127.0.0.1:0", "--remote-ca", filepath.Join(pki, "ca.pem"), "--remote-cert", filepath.Join(pki, "server.pem"), "--remote-key", filepath.Join(pki, "server-key.pem"), "--remote-crl", filepath.Join(pki, "crl.pem"))
		// The remote jog clock qualification the 1430 cell's rt-control runs with
		// (RevPi process arguments, read 2026-10-07; the values themselves are
		// unverified there). Without it every remote jog frame is refused
		// jog_clock_unqualified and no remote hold can move.
		controlArgs = append(controlArgs, "--remote-jog-max-uncertainty-ns", "100000000", "--remote-jog-drift-ppb", "2000000", "--remote-jog-calibration-max-age-ns", "10000000000")
	}
	adapter := spawn("rt-control", controlArgs...)
	c, err := control.Dial(control.UnixPath(socket))
	if err != nil {
		t.Fatal(err)
	}
	localObserver := c
	t.Cleanup(func() { localObserver.Close() })
	rtAwait(t, func() bool {
		ctx, cancel := context.WithTimeout(context.Background(), 100*time.Millisecond)
		defer cancel()
		s, err := c.Status(ctx)
		return err == nil && s.Motion.TimeNS > 0
	})
	ctx, cancel := context.WithTimeout(context.Background(), 30000*time.Millisecond)
	defer cancel()
	d, err := c.Describe(ctx)
	if err != nil {
		t.Fatal(err)
	}
	cfg := RTCoreConfig{Socket: socket, Binding: control.Binding{PairID: "bench", Revision: 7, MachineSHA256: d.MachineSHA256, ConfigurationSHA256: d.ConfigurationSHA256}, ExpectedBackend: "simulation", DeploymentSHA256: d.DeploymentSHA256, CapabilitiesDigest: d.CapabilitiesDigest, AxisMask: axisMask}
	if remote {
		cfg.Socket = ""
		rtAwait(t, func() bool { cfg.Address = remoteProcessAddress(adapter.Process.Pid); return cfg.Address != "" })
		cfg.CAFile, cfg.CertificateFile, cfg.KeyFile = filepath.Join(pki, "ca.pem"), filepath.Join(pki, "clients/olp-dev.pem"), filepath.Join(pki, "clients/olp-dev-key.pem")
		for _, principal := range []string{"revoked", "wrong-pair"} {
			t.Run(principal, func(t *testing.T) {
				bad := cfg
				bad.CertificateFile, bad.KeyFile = filepath.Join(pki, "clients/"+principal+".pem"), filepath.Join(pki, "clients/"+principal+"-key.pem")
				r, err := NewRTCore(bad, Config{ControllerID: "offline-programming:refused"}, nil)
				if err != nil {
					t.Fatal(err)
				}
				expected := "session_principal_mismatch"
				if principal == "revoked" {
					if _, err := r.c.Describe(ctx); err != nil {
						t.Fatalf("client before live revocation: %v", err)
					}
					// Closing this pool forces the next attempt onto a new handshake.
					r.Close(ctx)
					pkiCommand(t, root, pkiRoot, "revoke", pki, "revoked")
					r, err = NewRTCore(bad, Config{ControllerID: "offline-programming:refused"}, nil)
					if err != nil {
						t.Fatal(err)
					}
					expected = "rt_core_transport_lost"
				}
				defer r.Close(context.Background())
				_, err = r.Arm(ctx, true)
				var refusal *Error
				matched := errors.As(err, &refusal) && refusal.Code == expected
				if principal == "revoked" && refusal != nil {
					// TLS rejection can arrive before HTTP writes or after them.
					// Both SDK transport categories must retain the certificate
					// refusal; an unrelated connection failure is not a pass.
					matched = (refusal.Code == "rt_core_transport_lost" || refusal.Code == "rt_core_connection_reopened") &&
						strings.Contains(err.Error(), "tls: bad certificate")
				}
				if !matched {
					t.Fatalf("typed TLS refusal: %T %v", err, err)
				}
				if r.c.Grant().Generation != 0 || r.c.Grant().Session != "" {
					t.Fatal("TLS refusal acquired authority")
				}
				s, err := localObserver.Status(ctx)
				if err != nil || s.Core.Armed != 0 || s.Core.AxisEnableMask != 0 {
					t.Fatalf("refusal changed native authority: %+v %v", s.Core, err)
				}
			})
		}
	}
	return c, d, root, socket, cfg
}

func runRTCoreComposed(t *testing.T, remote bool) {
	c, _, root, socket, cfg := rtComposedTransportFixture(t, remote)
	ctx, cancel := context.WithTimeout(context.Background(), 30000*time.Millisecond)
	defer cancel()
	raw, err := os.ReadFile(filepath.Join(root, "adapters/rosie/densejointtraj/testdata/dense_joint_trajectory_reference.rdt"))
	if err != nil {
		t.Fatal(err)
	}
	plan, err := codec.Decode(raw)
	if err != nil {
		t.Fatal(err)
	}
	// A stationary nine-axis program exercises native sample execution without
	// inventing a connecting motion when the same prepared bytes are loaded again.
	for i := range plan.Segments {
		for j := range plan.Segments[i].Samples {
			s := &plan.Segments[i].Samples[j]
			for k := range s.QRad {
				s.QRad[k], s.QdRadS[k] = 0, 0
			}
			s.TorchOn = false
		}
	}
	blob, err := codec.Encode(plan.Header, plan.Segments)
	if err != nil {
		t.Fatal(err)
	}
	encoded, err := codec.Decode(blob)
	if err != nil {
		t.Fatal(err)
	}
	h := encoded.Header
	id := Identity{PlanID: h.PlanID, ProgramID: h.ProgramID, ProgramDigest: h.ProgramDigest, TrajectoryDigest: h.TrajectoryDigest, PlanRevision: h.PlanRevision, ManifestRevision: h.ManifestRevision}
	var injected RTCoreClient = c
	if remote {
		injected = nil
	}
	r, err := NewRTCore(cfg, Config{ControllerID: "offline-programming:composed", FetchBlob: func(context.Context, string) ([]byte, error) { return blob, nil }}, injected)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	c = r.c.(*control.Client)
	for _, externalStop := range []bool{false, true} {
		name := "completion_then_controller_stop"
		if externalStop {
			name = "separate_sdk_client_stop"
		}
		if !t.Run(name, func(t *testing.T) {
			if _, err := r.Arm(ctx, true); err != nil {
				t.Fatalf("arm: %v", err)
			}
			loaded, err := r.Load(ctx, LoadRequest{Identity: id})
			if err != nil {
				t.Fatalf("prepare: %v", err)
			}
			if _, err := r.Play(ctx, loaded.SessionID); err != nil {
				t.Fatalf("start: %v", err)
			}
			if !externalStop {
				rtAwait(t, func() bool { _, _ = r.Heartbeat(loaded.SessionID); return r.Status(ctx).Session.State == "done" })
				s, err := c.Status(ctx)
				if err != nil {
					t.Fatal(err)
				}
				if s.Execution.State != "completed" || s.Execution.NativeSequence == 0 || s.Execution.Identity.SourceDigest == "" || s.Execution.Identity.NormalisedDigest == "" {
					t.Fatalf("completion: %+v", s.Execution)
				}
			} else {
				rtAwait(t, func() bool { s, err := c.Status(ctx); return err == nil && s.Execution.State == "executing" })
			}
			if externalStop {
				operator, err := control.Dial(control.UnixPath(socket))
				if remote {
					operator.Close()
					pem, readErr := os.ReadFile(cfg.CAFile)
					if readErr != nil {
						t.Fatal(readErr)
					}
					roots := x509.NewCertPool()
					if !roots.AppendCertsFromPEM(pem) {
						t.Fatal("CA")
					}
					cert, certErr := tls.LoadX509KeyPair(cfg.CertificateFile, cfg.KeyFile)
					if certErr != nil {
						t.Fatal(certErr)
					}
					operator, err = control.DialTLS(cfg.Address, &tls.Config{RootCAs: roots, Certificates: []tls.Certificate{cert}})
				}
				if err != nil {
					t.Fatal(err)
				}
				defer operator.Close()
				// A separate public client shares the authorized session fence, just
				// as an operator SDK/CLI Stop would; OLP receives no Stop invocation.
				operator.SetFence(c.Fence())
				grant, err := operator.Stop(ctx)
				if err != nil || grant.Stopping {
					t.Fatalf("separate SDK Stop: %+v %v", grant, err)
				}
				rtAwait(t, func() bool {
					status := r.Status(ctx)
					return status.Session.State == "stopped" && !status.Session.Lease.Held &&
						status.RTCore.Reason != "" && c.Grant().Session == ""
				})
				if status := r.Status(ctx); status.Armed == nil || *status.Armed || status.RTCore.Reason == "operator_stop" {
					t.Fatalf("external Stop was not independently detected: %+v", status)
				}
			} else {
				result, err := r.Stop(ctx)
				if err != nil || !result.Stopped {
					t.Fatalf("controller stop: %+v %v", result, err)
				}
			}
			rtAwait(t, func() bool {
				s, err := c.Status(ctx)
				return err == nil && s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Execution.State != "executing"
			})
			if c.Grant().Session != "" {
				t.Fatal("Stop did not release authority")
			}
		}) {
			return
		}
	}
}

// Resolve the kernel-assigned listener by owning PID, preventing a guessed port
// or reserve-and-release race from connecting the test to a different adapter.
func remoteProcessAddress(pid int) string {
	entries, _ := os.ReadDir(fmt.Sprintf("/proc/%d/fd", pid))
	inodes := map[string]bool{}
	for _, entry := range entries {
		link, _ := os.Readlink(fmt.Sprintf("/proc/%d/fd/%s", pid, entry.Name()))
		if strings.HasPrefix(link, "socket:[") {
			inodes[strings.TrimSuffix(strings.TrimPrefix(link, "socket:["), "]")] = true
		}
	}
	raw, _ := os.ReadFile("/proc/net/tcp")
	for _, line := range strings.Split(string(raw), "\n") {
		fields := strings.Fields(line)
		if len(fields) < 10 || fields[3] != "0A" || !inodes[fields[9]] {
			continue
		}
		address := strings.Split(fields[1], ":")
		if len(address) != 2 || address[0] != "0100007F" {
			continue
		}
		port, err := strconv.ParseUint(address[1], 16, 16)
		if err == nil && port != 0 {
			return fmt.Sprintf("https://127.0.0.1:%d", port)
		}
	}
	return ""
}

// Delayed receipts retain separate coverage from HTTPS authorization and joint fixtures.
func TestRTCoreRemoteComposed(t *testing.T) { runRTCoreDelayedComposed(t, true, 300, false) }

// TestReviewOLPExpiredRecovery observes native expiry before Stop/Release and
// requires fresh Acquire, Enable and Arm on the same previously inhibited owner.
func TestReviewOLPExpiredRecovery(t *testing.T) { runRTCoreDelayedComposed(t, false, 0, true) }

// TestReviewOLP250Composed retains the review overlay as a permanent native
// acquire, Home, arm and program-start probe through delayed public receipts.
func TestReviewOLP250Composed(t *testing.T) { runRTCoreDelayedComposed(t, true, 250, false) }

func runRTCoreDelayedComposed(t *testing.T, remote bool, delayMS int, expiredRecovery bool) {
	if os.Getenv("RTCORE_COMPOSED") != "1" {
		t.Skip("set RTCORE_COMPOSED=1 after make control sim in rt-core")
	}
	root, err := filepath.Abs("../../../../rt-core")
	if err != nil {
		t.Fatal(err)
	}
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "olp-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(build) })
	dir, err := os.MkdirTemp("", "olp-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	configuration := "config/machines/simulation/simulation-program.json"
	if remote {
		raw, err := os.ReadFile(filepath.Join(root, "config/machines/bench/nine-motors-1to1.json"))
		if err != nil {
			t.Fatal(err)
		}
		var cell map[string]json.RawMessage
		if err := json.Unmarshal(raw, &cell); err != nil {
			t.Fatal(err)
		}
		// Unverified WSL simulator scheduling allowance: 20000000 ns. All bench
		// axes, Home rules, control ceilings and limits remain compiled unchanged.
		cell["max_cycle_lateness_ns"] = json.RawMessage("20000000")
		raw, err = json.Marshal(cell)
		if err != nil {
			t.Fatal(err)
		}
		configuration = filepath.Join(build, "bench-simulation.json")
		if err := os.WriteFile(configuration, raw, 0600); err != nil {
			t.Fatal(err)
		}
	}
	configPath := configuration
	if !filepath.IsAbs(configPath) {
		configPath = filepath.Join(root, configPath)
	}
	compiled, args, err := simfixture.Compile(root, configPath, filepath.Join(build, "config"))
	if err != nil {
		t.Fatal(err)
	}
	spawn := func(name string, args ...string) {
		t.Helper()
		log, err := os.Create(filepath.Join(build, name+".log"))
		if err != nil {
			t.Fatal(err)
		}
		cmd := exec.Command(filepath.Join(root, "build", name), args...)
		cmd.Stdout, cmd.Stderr = log, log
		if err := cmd.Start(); err != nil {
			log.Close()
			t.Fatal(err)
		}
		t.Cleanup(func() {
			_ = cmd.Process.Signal(syscall.SIGTERM)
			done := make(chan error, 1)
			go func() { done <- cmd.Wait() }()
			select {
			case err := <-done:
				if err != nil {
					t.Errorf("%s exit: %v", name, err)
				}
			case <-time.After(5000 * time.Millisecond):
				_ = cmd.Process.Kill()
				<-done
				t.Errorf("%s did not terminate", name)
			}
			log.Close()
			if t.Failed() {
				data, _ := os.ReadFile(log.Name())
				if len(data) > 1500 {
					data = data[len(data)-1500:]
				}
				t.Logf("%s: %s", name, data)
			}
		})
	}
	coreSocket, socket := filepath.Join(dir, "ipc.sock"), filepath.Join(dir, "control.sock")
	spawn("rosie-rt-core-sim", append(args, "--socket-path", coreSocket)...)
	rtAwait(t, func() bool {
		raw, _ := os.ReadFile(filepath.Join(dir, "metrics.json"))
		var m struct {
			ConfigVerifiedMask uint32 `json:"config_verified_mask"`
		}
		return json.Unmarshal(raw, &m) == nil && m.ConfigVerifiedMask == 511
	})
	spawn("rt-control", "--core-socket", coreSocket, "--socket", socket, "--configuration-sha256", filepath.Base(compiled), "--backend", "simulation", "--pair-id", "bench", "--pair-revision", "7")
	c, err := control.Dial(control.UnixPath(socket))
	if err != nil {
		t.Fatal(err)
	}
	initialClient := c
	t.Cleanup(func() { initialClient.Close() })
	rtAwait(t, func() bool {
		ctx, cancel := context.WithTimeout(context.Background(), 100*time.Millisecond)
		defer cancel()
		s, err := c.Status(ctx)
		return err == nil && s.Motion.TimeNS > 0
	})
	if remote {
		upstream := control.NewUnixHTTPClient(socket)
		delayedSocket := filepath.Join(dir, "delayed.sock")
		listener, err := net.Listen("unix", delayedSocket)
		if err != nil {
			t.Fatal(err)
		}
		server := &http.Server{Handler: http.HandlerFunc(func(w http.ResponseWriter, request *http.Request) {
			request.RequestURI = ""
			request.URL.Scheme = "http"
			request.URL.Host = "localhost"
			response, err := upstream.Do(request)
			if err != nil {
				http.Error(w, err.Error(), 502)
				return
			}
			defer response.Body.Close()
			body, err := io.ReadAll(response.Body)
			if err != nil {
				http.Error(w, err.Error(), 502)
				return
			}
			// Injected transport delay; receipts and native state are the assertions.
			timer := time.NewTimer(time.Duration(delayMS) * time.Millisecond)
			defer timer.Stop()
			select {
			case <-timer.C:
			case <-request.Context().Done():
				return
			}
			for key, values := range response.Header {
				for _, value := range values {
					w.Header().Add(key, value)
				}
			}
			w.WriteHeader(response.StatusCode)
			_, _ = w.Write(body)
		})}
		go server.Serve(listener)
		t.Cleanup(func() { server.Close(); upstream.CloseIdleConnections() })
		socket = delayedSocket
		c, err = control.Dial(control.UnixPath(socket))
		if err != nil {
			t.Fatal(err)
		}
		t.Cleanup(func() { c.Close() })
	}
	ctx, cancel := context.WithTimeout(context.Background(), 30000*time.Millisecond)
	defer cancel()
	d, err := c.Describe(ctx)
	if err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(filepath.Join(root, "adapters/rosie/densejointtraj/testdata/dense_joint_trajectory_reference.rdt"))
	if err != nil {
		t.Fatal(err)
	}
	plan, err := codec.Decode(raw)
	if err != nil {
		t.Fatal(err)
	}
	// A stationary nine-axis program exercises native sample execution without
	// inventing a connecting motion when the same prepared bytes are loaded again.
	for i := range plan.Segments {
		for j := range plan.Segments[i].Samples {
			s := &plan.Segments[i].Samples[j]
			for k := range s.QRad {
				s.QRad[k], s.QdRadS[k] = 0, 0
			}
			s.TorchOn = false
		}
	}
	blob, err := codec.Encode(plan.Header, plan.Segments)
	if err != nil {
		t.Fatal(err)
	}
	encoded, err := codec.Decode(blob)
	if err != nil {
		t.Fatal(err)
	}
	h := encoded.Header
	id := Identity{PlanID: h.PlanID, ProgramID: h.ProgramID, ProgramDigest: h.ProgramDigest, TrajectoryDigest: h.TrajectoryDigest, PlanRevision: h.PlanRevision, ManifestRevision: h.ManifestRevision}
	r, err := NewRTCore(RTCoreConfig{Socket: socket, Binding: control.Binding{PairID: "bench", Revision: 7, MachineSHA256: d.MachineSHA256}, DeploymentSHA256: d.DeploymentSHA256, CapabilitiesDigest: d.CapabilitiesDigest, AxisMask: 511}, Config{ControllerID: "offline-programming:composed", FetchBlob: func(context.Context, string) ([]byte, error) { return blob, nil }}, c)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	if remote {
		heartbeatContext, stopHeartbeat := context.WithCancel(ctx)
		done := make(chan struct{})
		go func() {
			defer close(done)
			// Unverified simulated browser cadence: 100 ms; heartbeat receipts preserve its session.
			ticker := time.NewTicker(100 * time.Millisecond)
			defer ticker.Stop()
			for {
				select {
				case <-heartbeatContext.Done():
					return
				case <-ticker.C:
					if session := r.Status(ctx).Session; session != nil {
						_, _ = r.Heartbeat(session.ID)
					}
				}
			}
		}()
		t.Cleanup(func() { stopHeartbeat(); <-done })
	}

	if expiredRecovery {
		if _, err := r.Arm(ctx, true); err != nil {
			t.Fatal(err)
		}
		// Stop observation and renewal to inject expiry without automatic cleanup.
		r.cancel()
		<-r.done
		c.StopRenewal()
		grant := c.Grant()
		before, err := initialClient.Status(ctx)
		if err != nil || grant.Session == "" || grant.LeaseMS != 500 || before.Grant == nil || before.Grant.GrantActive != 1 || before.Grant.NowHostNs >= grant.DeadlineHostNS {
			t.Fatalf("live expiry fixture: grant=%+v native=%+v err=%v", grant, before.Grant, err)
		}
		rtAwait(t, func() bool {
			state, err := initialClient.Status(ctx)
			return err == nil && state.Grant != nil && state.Grant.GrantActive == 0 && state.Core.Armed == 0 && state.Grant.NowHostNs >= grant.DeadlineHostNS
		})
		// Exercise recovery from the controller's latched failure state.
		r.mu.Lock()
		r.inhibited = true
		r.mu.Unlock()
		result, err := r.Stop(ctx)
		if err != nil || !result.OK || c.Grant().Session != "" {
			t.Fatalf("expired recovery Stop/Release: result=%+v err=%v", result, err)
		}
		r.mu.Lock()
		inhibited := r.inhibited
		r.mu.Unlock()
		if inhibited {
			t.Fatal("acknowledged recovery retained inhibition")
		}
		if _, err := r.Arm(ctx, true); err != nil {
			t.Fatalf("rearm same controller: %v", err)
		}
		fresh := c.Grant()
		if fresh.Session == "" || fresh.Session == grant.Session || fresh.Generation <= grant.Generation {
			t.Fatalf("fresh acquire: %+v", fresh)
		}
		rtAwait(t, func() bool {
			s, err := c.Status(ctx)
			return err == nil && s.Core.Armed == 1 && s.Core.AxisEnableMask == 511
		})
		if _, err := r.Stop(ctx); err != nil {
			t.Fatal(err)
		}
		return
	}
	cases := []bool{false, true}
	if remote {
		cases = []bool{false}
	}
	for _, externalStop := range cases {
		name := "completion_then_controller_stop"
		if externalStop {
			name = "separate_sdk_client_stop"
		}
		if !t.Run(name, func(t *testing.T) {
			if remote {
				if err := r.Home(ctx); err != nil {
					t.Fatalf("native Home: %v", err)
				}
			}
			if _, err := r.Arm(ctx, true); err != nil {
				t.Fatalf("arm: %v", err)
			}
			if remote {
				g := c.Grant()
				if g.LeaseMS < 1125 || g.LeaseMS > 3000 {
					t.Fatalf("remote effective lease=%d", g.LeaseMS)
				}
				if d.MaxGrantLeaseNS != 3000000000 || d.MaxJogInputAgeNS != 750000000 {
					t.Fatalf("remote bench timing=%+v", d)
				}
				t.Logf("%d ms delayed adapter: effective lease %d ms", delayMS, g.LeaseMS)
				observed, err := c.Status(ctx)
				if err != nil {
					t.Fatal(err)
				}
				// Home chooses the bench coordinate frame. Build a stationary program
				// from that observed frame, preventing an invented zero-position bridge.
				for i := range plan.Segments {
					for j := range plan.Segments[i].Samples {
						for k := range plan.Segments[i].Samples[j].QRad {
							plan.Segments[i].Samples[j].QRad[k] = observed.Axes[k].LogicalPosition
						}
					}
				}
				blob, err = codec.Encode(plan.Header, plan.Segments)
				if err != nil {
					t.Fatal(err)
				}
				updated, err := codec.Decode(blob)
				if err != nil {
					t.Fatal(err)
				}
				h := updated.Header
				id = Identity{PlanID: h.PlanID, ProgramID: h.ProgramID, ProgramDigest: h.ProgramDigest, TrajectoryDigest: h.TrajectoryDigest, PlanRevision: h.PlanRevision, ManifestRevision: h.ManifestRevision}

			}
			loaded, err := r.Load(ctx, LoadRequest{Identity: id})
			if err != nil {
				t.Fatalf("prepare: %v", err)
			}
			if _, err := r.Play(ctx, loaded.SessionID); err != nil {
				t.Fatalf("start: %v", err)
			}
			if !externalStop {
				rtAwait(t, func() bool { _, _ = r.Heartbeat(loaded.SessionID); return r.Status(ctx).Session.State == "done" })
				s, err := c.Status(ctx)
				if err != nil {
					t.Fatal(err)
				}
				if s.Execution.State != "completed" || s.Execution.NativeSequence == 0 || s.Execution.Identity.SourceDigest == "" || s.Execution.Identity.NormalisedDigest == "" {
					t.Fatalf("completion: %+v", s.Execution)
				}
			} else {
				rtAwait(t, func() bool { s, err := c.Status(ctx); return err == nil && s.Execution.State == "executing" })
			}
			if externalStop {
				operator, err := control.Dial(control.UnixPath(socket))
				if err != nil {
					t.Fatal(err)
				}
				defer operator.Close()
				// A separate public client shares the authorized session fence, just
				// as an operator SDK/CLI Stop would; OLP receives no Stop invocation.
				operator.SetFence(c.Fence())
				grant, err := operator.Stop(ctx)
				if err != nil || grant.Stopping {
					t.Fatalf("separate SDK Stop: %+v %v", grant, err)
				}
				rtAwait(t, func() bool {
					status := r.Status(ctx)
					return status.Session.State == "stopped" && !status.Session.Lease.Held &&
						status.RTCore.Reason != "" && c.Grant().Session == ""
				})
				if status := r.Status(ctx); status.Armed == nil || *status.Armed || status.RTCore.Reason == "operator_stop" {
					t.Fatalf("external Stop was not independently detected: %+v", status)
				}
			} else {
				result, err := r.Stop(ctx)
				if err != nil || !result.Stopped {
					t.Fatalf("controller stop: %+v %v", result, err)
				}
			}
			rtAwait(t, func() bool {
				s, err := c.Status(ctx)
				return err == nil && s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Execution.State != "executing"
			})
			if c.Grant().Session != "" {
				t.Fatal("Stop did not release authority")
			}
		}) {
			return
		}
	}
}
