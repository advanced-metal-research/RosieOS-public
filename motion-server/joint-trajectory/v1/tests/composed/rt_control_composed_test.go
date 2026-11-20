//go:build linux

package motionserver_test

import (
	"bufio"
	"bytes"
	"context"
	"encoding/binary"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net"
	"net/http"
	"net/http/httputil"
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"syscall"
	"testing"
	"time"

	"golang.org/x/sys/unix"

	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/sdk/control"
	"rosieos/rt-core/simfixture"
)

// Unverified observation ceiling: 10000 ms, independent of motion and lease duration.
const observation = 10000 * time.Millisecond

// Unverified polling cadence: 5 ms; success requires the predicate's observation.
const cadence = 5 * time.Millisecond

var componentRoot string
var composedBuild string

// Direct go test builds the daemon and checks sibling prerequisites before spawn;
// missing binaries name the exact make target rather than failing inside exec.
func TestMain(m *testing.M) {
	if err := simfixture.CheckCPURequirement(); err != nil {
		var requirement *simfixture.CPURequirementError
		if errors.As(err, &requirement) {
			fmt.Fprintf(os.Stdout, "SKIP: composed suite: %v\n", requirement)
			os.Exit(0)
		}
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	_, source, _, ok := runtime.Caller(0)
	if !ok {
		fmt.Fprintln(os.Stderr, "cannot locate composed test source")
		os.Exit(1)
	}
	componentRoot = filepath.Clean(filepath.Join(filepath.Dir(source), "../.."))
	core := filepath.Clean(filepath.Join(componentRoot, "../../../rt-core"))
	composedBuild = filepath.Join(core, "build", "joint-trajectory-composed")
	if err := os.MkdirAll(composedBuild, 0700); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	for _, binary := range []string{"rosie-rt-core-sim", "rt-control", "rtctl"} {
		path := filepath.Join(core, "build", binary)
		if info, err := os.Stat(path); err != nil || info.Mode()&0111 == 0 {
			fmt.Fprintf(os.Stderr, "missing executable %s; run make -C %s control sim\n", path, core)
			os.Exit(1)
		}
	}
	cmd := exec.Command("make", "-C", componentRoot, "all", "BIN_DIR="+filepath.Join(composedBuild, "bin"))
	cmd.Stdout, cmd.Stderr = os.Stdout, os.Stderr
	if err := cmd.Run(); err != nil {
		fmt.Fprintf(os.Stderr, "build joint_trajectory_daemon: %v\n", err)
		os.Exit(1)
	}
	os.Exit(m.Run())
}

func must(t *testing.T, err error) {
	t.Helper()
	if err != nil {
		t.Fatal(err)
	}
}
func read(t *testing.T, path string) []byte {
	t.Helper()
	b, e := os.ReadFile(path)
	must(t, e)
	return b
}
func writeJSON(t *testing.T, path string, value any) {
	t.Helper()
	b, e := json.Marshal(value)
	must(t, e)
	must(t, os.WriteFile(path, b, 0600))
}

// Poll cannot turn elapsed time into success; deadlines only bound missing observations.
func poll(t *testing.T, predicate func() bool) {
	t.Helper()
	deadline := time.NewTimer(observation)
	defer deadline.Stop()
	ticker := time.NewTicker(cadence)
	defer ticker.Stop()
	for {
		if predicate() {
			return
		}
		select {
		case <-ticker.C:
		case <-deadline.C:
			t.Fatal("observation exceeded 10000 ms")
		}
	}
}

// Spawn preserves simfixture's immediate cleanup registration and explicit argv;
// failed readiness cannot leak a simulation, adapter or daemon child.
func spawn(t *testing.T, logPath, binary string, args ...string) *exec.Cmd {
	t.Helper()
	log, e := os.Create(logPath)
	must(t, e)
	cmd := exec.Command(binary, args...)
	cmd.Stdout = log
	cmd.Stderr = log
	must(t, cmd.Start())
	t.Cleanup(func() {
		if cmd.ProcessState != nil {
			_ = log.Close()
			return
		}
		_ = cmd.Process.Signal(syscall.SIGCONT)
		_ = cmd.Process.Signal(syscall.SIGTERM)
		done := make(chan error, 1)
		go func() { done <- cmd.Wait() }()
		select {
		case e := <-done:
			if e != nil {
				t.Errorf("%s exit: %v", binary, e)
			}
		case <-time.After(observation):
			_ = cmd.Process.Kill()
			<-done
			t.Errorf("%s shutdown exceeded 10000 ms", binary)
		}
		_ = log.Close()
		if t.Failed() {
			t.Logf("%s: %s", binary, read(t, logPath))
		}
	})
	return cmd
}

type publication struct {
	subject, reply string
	body           []byte
}
type subscription struct {
	subject, sid string
	peer         *brokerPeer
}
type brokerPeer struct {
	conn net.Conn
	mu   sync.Mutex
}

func (p *brokerPeer) send(s string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	_ = p.conn.SetWriteDeadline(time.Now().Add(observation))
	_, _ = io.WriteString(p.conn, s)
}

type broker struct {
	listener net.Listener
	mu       sync.Mutex
	peers    []*brokerPeer
	subs     []subscription
	messages []publication
	stopped  bool
	wg       sync.WaitGroup
}

// The broker implements the daemon's actual NATS framing and records every PUB,
// preventing an in-process dispatcher test from hiding a missing status publisher.
func newBroker(t *testing.T) *broker {
	t.Helper()
	ln, e := net.Listen("tcp", "127.0.0.1:0")
	must(t, e)
	b := &broker{listener: ln}
	b.wg.Add(1)
	go func() {
		defer b.wg.Done()
		for {
			c, e := ln.Accept()
			if e != nil {
				return
			}
			p := &brokerPeer{conn: c}
			b.mu.Lock()
			if b.stopped {
				_ = c.Close()
				b.mu.Unlock()
				return
			}
			b.peers = append(b.peers, p)
			b.wg.Add(1)
			b.mu.Unlock()
			go func() {
				defer b.wg.Done()
				defer c.Close()
				p.send("INFO {}\r\n")
				r := bufio.NewReader(c)
				for {
					line, e := r.ReadString('\n')
					if e != nil {
						return
					}
					f := strings.Fields(line)
					if len(f) == 0 {
						continue
					}
					switch f[0] {
					case "PING":
						p.send("PONG\r\n")
					case "SUB":
						if len(f) == 3 {
							b.mu.Lock()
							b.subs = append(b.subs, subscription{f[1], f[2], p})
							b.mu.Unlock()
						}
					case "PUB":
						if len(f) != 3 && len(f) != 4 {
							return
						}
						n, e := strconv.Atoi(f[len(f)-1])
						if e != nil || n < 0 || n > 1048576 {
							return
						}
						body := make([]byte, n+2)
						if _, e = io.ReadFull(r, body); e != nil {
							return
						}
						reply := ""
						if len(f) == 4 {
							reply = f[2]
						}
						b.publish(publication{f[1], reply, body[:n]})
					}
				}
			}()
		}
	}()
	t.Cleanup(func() {
		b.mu.Lock()
		b.stopped = true
		_ = ln.Close()
		for _, p := range b.peers {
			_ = p.conn.Close()
		}
		b.mu.Unlock()
		b.wg.Wait()
	})
	return b
}
func (b *broker) publish(message publication) {
	b.mu.Lock()
	b.messages = append(b.messages, message)
	subs := append([]subscription(nil), b.subs...)
	b.mu.Unlock()
	for _, s := range subs {
		if s.subject == message.subject {
			reply := ""
			if message.reply != "" {
				reply = " " + message.reply
			}
			s.peer.send(fmt.Sprintf("MSG %s %s%s %d\r\n%s\r\n", message.subject, s.sid, reply, len(message.body), message.body))
		}
	}
}
func (b *broker) subscribed(subject string) bool {
	b.mu.Lock()
	defer b.mu.Unlock()
	for _, s := range b.subs {
		if s.subject == subject {
			return true
		}
	}
	return false
}
func (b *broker) matching(subject string, predicate func(map[string]any) bool) bool {
	b.mu.Lock()
	defer b.mu.Unlock()
	for _, m := range b.messages {
		if m.subject == subject {
			var v map[string]any
			if json.Unmarshal(m.body, &v) == nil && predicate(v) {
				return true
			}
		}
	}
	return false
}

// Discard stale status samples without losing replies an active renewal is
// still waiting to consume; clearing all publications can strand that request.
func (b *broker) clearStatus() {
	b.mu.Lock()
	defer b.mu.Unlock()
	kept := b.messages[:0]
	for _, message := range b.messages {
		if message.subject != "motion.status" {
			kept = append(kept, message)
		}
	}
	b.messages = kept
}

type fixture struct {
	t                                 *testing.T
	root, core, build, socket, ingest string
	client                            *control.Client
	broker                            *broker
	commandID                         atomic.Int64
	intent                            int
	lease                             map[string]any
	daemon                            *exec.Cmd
	udp                               net.PacketConn
	stopRenew                         func()
	gateEntered                       chan struct{}
	releaseGate                       func()
	stopReceipts                      chan time.Time
	failStatus                        *atomic.Bool
	restartControl                    func()
}

type fixtureOptions struct {
	gate string
}

func setup(t *testing.T, options ...fixtureOptions) *fixture {
	t.Helper()
	var option fixtureOptions
	if len(options) > 0 {
		option = options[0]
	}
	root := componentRoot
	core := filepath.Clean(filepath.Join(root, "../../../rt-core"))
	build, e := os.MkdirTemp(composedBuild, "composed-")
	must(t, e)
	t.Cleanup(func() { _ = os.RemoveAll(build) })
	socketDir, e := os.MkdirTemp("", "m-")
	must(t, e)
	t.Cleanup(func() { _ = os.RemoveAll(socketDir) })
	machinePath := filepath.Join(core, "clients/cpp/examples/testdata/profile-machine.json")
	profiles := filepath.Join(core, "config/drives")
	var machine map[string]any
	must(t, json.Unmarshal(read(t, machinePath), &machine))
	if machine["backend"] != "simulation" {
		t.Fatal("composed fixture requires a simulation configuration")
	}
	// Select before compilation so both argv and deployment identity use an
	// allowed CPU; the checked-in example cannot impose the host's core count.
	var allowed unix.CPUSet
	must(t, unix.SchedGetaffinity(0, &allowed))
	remaining := allowed.Count()
	if remaining == 0 {
		t.Fatal("composed fixture has no allowed CPU")
	}
	selected := 0
	for cpu := 0; remaining > 0; cpu++ {
		if allowed.IsSet(cpu) {
			selected = cpu
			remaining--
		}
	}
	machine["runtime"].(map[string]any)["cpu"] = selected
	t.Logf("simulation fixture: allowed CPUs=%d, runtime CPU=%d", allowed.Count(), selected)
	machinePath = filepath.Join(build, "machine.json")
	writeJSON(t, machinePath, machine)
	compiled, args, e := simfixture.CompileProfiles(core, machinePath, profiles, filepath.Join(build, "config"))
	if e != nil {
		t.Fatal(e)
	}
	coreSocket := filepath.Join(socketDir, "ipc.sock")
	socket := filepath.Join(socketDir, "control.sock")
	spawn(t, filepath.Join(build, "core.log"), filepath.Join(core, "build/rosie-rt-core-sim"), append(args, "--socket-path", coreSocket)...)
	poll(t, func() bool {
		raw, _ := os.ReadFile(filepath.Join(socketDir, "metrics.json"))
		var s struct {
			Mask uint32 `json:"config_verified_mask"`
		}
		return json.Unmarshal(raw, &s) == nil && s.Mask == 511
	})
	controlArgs := []string{"--core-socket", coreSocket, "--socket", socket, "--configuration-sha256", filepath.Base(compiled), "--backend", "simulation", "--pair-id", "bench", "--pair-revision", "7"}
	adapter := spawn(t, filepath.Join(build, "control.log"), filepath.Join(core, "build/rt-control"), controlArgs...)
	restartControl := func() {
		must(t, adapter.Process.Signal(syscall.SIGTERM))
		done := make(chan error, 1)
		go func() { done <- adapter.Wait() }()
		select {
		case err := <-done:
			must(t, err)
		case <-time.After(observation):
			_ = adapter.Process.Kill()
			<-done
			t.Fatal("rt-control restart failed to stop prior process")
		}
		adapter = spawn(t, filepath.Join(build, "control-restarted.log"), filepath.Join(core, "build/rt-control"), controlArgs...)
	}
	c, e := control.Dial(control.UnixPath(socket))
	must(t, e)
	t.Cleanup(func() { _ = c.Close() })
	ctx, cancel := context.WithTimeout(context.Background(), observation)
	defer cancel()
	poll(t, func() bool { s, e := c.Status(ctx); return e == nil && s.Motion.TimeNS != 0 })
	d, e := c.Describe(ctx)
	must(t, e)
	daemonSocket := socket
	var entered, release chan struct{}
	var unblock func()
	stopReceipts := make(chan time.Time, 1)
	failStatus := &atomic.Bool{}
	if option.gate != "" {
		entered, release = make(chan struct{}), make(chan struct{})
		var gated, homeSent atomic.Bool
		var releaseOnce sync.Once
		unblock = func() { releaseOnce.Do(func() { close(release) }) }
		daemonSocket = filepath.Join(socketDir, "gate.sock")
		listener, err := net.Listen("unix", daemonSocket)
		must(t, err)
		transport := &http.Transport{DialContext: func(ctx context.Context, _, _ string) (net.Conn, error) {
			return (&net.Dialer{}).DialContext(ctx, "unix", socket)
		}}
		proxy := &httputil.ReverseProxy{Director: func(r *http.Request) { r.URL.Scheme = "http"; r.URL.Host = "control" }, Transport: transport}
		proxy.ModifyResponse = func(r *http.Response) error {
			if r.Request.URL.Path != "/v1/control" {
				return nil
			}
			body, err := io.ReadAll(r.Body)
			if err != nil {
				return err
			}
			_ = r.Body.Close()
			r.Body = io.NopCloser(bytes.NewReader(body))
			var receipt struct{ Operation, Error string }
			if json.Unmarshal(body, &receipt) == nil && receipt.Operation == "stop" && receipt.Error == "" && r.StatusCode == http.StatusOK {
				select {
				case stopReceipts <- time.Now():
				default:
				}
			}
			return nil
		}
		server := &http.Server{Handler: http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			if r.URL.Path == "/v1/status" && failStatus.Load() {
				http.Error(w, "cleanup status unavailable", http.StatusServiceUnavailable)
				return
			}
			if r.URL.Path == "/v1/control" {
				body, err := io.ReadAll(r.Body)
				if err != nil {
					http.Error(w, err.Error(), http.StatusBadRequest)
					return
				}
				_ = r.Body.Close()
				r.Body = io.NopCloser(bytes.NewReader(body))
				var command struct{ Operation string }
				if json.Unmarshal(body, &command) == nil && command.Operation == "home" {
					homeSent.Store(true)
				}
			}
			if ((option.gate == "describe" && r.URL.Path == "/v1/describe") ||
				(option.gate == "home" && homeSent.Load() && r.URL.Path == "/v1/status")) && gated.CompareAndSwap(false, true) {
				close(entered)
				<-release
			}
			proxy.ServeHTTP(w, r)
		})}
		go func() { _ = server.Serve(listener) }()
		t.Cleanup(func() { unblock(); _ = server.Close(); transport.CloseIdleConnections() })
	}
	b := newBroker(t)
	configPath := filepath.Join(build, "daemon.json")
	writeJSON(t, configPath, map[string]any{"schema": "robot.v4.joint-trajectory-daemon.config.v1", "backend": "rt_core", "rt_core": map[string]any{
		"socket": daemonSocket, "controller": "motion-server-composed", "pair_id": "bench", "pair_revision": 7, "configuration_sha256": d.ConfigurationSHA256, "machine_sha256": d.MachineSHA256, "deployment_sha256": d.DeploymentSHA256, "expected_backend": "simulation", "home_policy": "home", "status_subject": "motion.status"}})
	ready := filepath.Join(build, "ready.json")
	store := filepath.Join(build, "plans")
	must(t, os.Mkdir(store, 0700))
	udp, e := net.ListenPacket("udp", "127.0.0.1:0")
	must(t, e)
	t.Cleanup(func() { _ = udp.Close() })
	daemon := spawn(t, filepath.Join(build, "daemon.log"), filepath.Join(composedBuild, "bin/joint_trajectory_daemon"), "--config", configPath, "--dense-ingest-listen", "127.0.0.1:0", "--ready-file", ready, "--plan-store-dir", store, "--nats-url", "nats://"+b.listener.Addr().String(), "--robot-cell", "test", "--robot-command-subject", "motion.command")
	if unblock != nil {
		t.Cleanup(unblock)
	}
	var address struct {
		Host string
		Port int
	}
	poll(t, func() bool {
		raw, _ := os.ReadFile(ready)
		return json.Unmarshal(raw, &address) == nil && address.Port != 0 && b.subscribed("motion.command")
	})
	return &fixture{t: t, root: root, core: core, build: build, socket: socket, ingest: fmt.Sprintf("%s:%d", address.Host, address.Port), client: c, broker: b, daemon: daemon, udp: udp, gateEntered: entered, releaseGate: unblock, stopReceipts: stopReceipts, failStatus: failStatus, restartControl: restartControl}
}

func (f *fixture) command(command string, fields map[string]any) map[string]any {
	f.t.Helper()
	id := strconv.FormatInt(f.commandID.Add(1), 10)
	request := map[string]any{"schema": "robot.v4.robot-command.v1", "robot": "test", "sender_id": "offline-programming:test-1", "controller_boot_id": "test-boot", "command": command, "command_id": id}
	for k, v := range fields {
		request[k] = v
	}
	raw, e := json.Marshal(request)
	must(f.t, e)
	subject := "reply." + id
	f.broker.publish(publication{"motion.command", subject, raw})
	var result map[string]any
	poll(f.t, func() bool {
		return f.broker.matching(subject, func(v map[string]any) bool { result = v; return true })
	})
	return result
}
func (f *fixture) acquire() {
	if f.stopRenew != nil {
		f.stopRenew()
	}
	f.lease = f.command("leader_acquire", map[string]any{"ttl_ms": 2000})
	if f.lease["ok"] != true {
		f.t.Fatalf("acquire: %v", f.lease)
	}
	stop, done := make(chan struct{}), make(chan struct{})
	var once sync.Once
	f.stopRenew = func() { once.Do(func() { close(stop); <-done }) }
	f.t.Cleanup(f.stopRenew)
	go func() {
		defer close(done)
		// Unverified controller renewal cadence: 100 ms.
		ticker := time.NewTicker(100 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-stop:
				return
			case <-ticker.C:
				r := f.command("leader_renew", map[string]any{"lease_id": f.lease["lease_id"], "leader_fence_epoch": f.lease["leader_fence_epoch"], "ttl_ms": 2000})
				if r["ok"] != true {
					return
				}
			}
		}
	}()
}
func (f *fixture) ingestRequest(request map[string]any, body []byte) map[string]any {
	f.t.Helper()
	raw, e := json.Marshal(request)
	must(f.t, e)
	payload := make([]byte, 4+len(raw)+len(body))
	binary.BigEndian.PutUint32(payload, uint32(len(raw)))
	copy(payload[4:], raw)
	copy(payload[4+len(raw):], body)
	c, e := net.DialTimeout("tcp", f.ingest, observation)
	must(f.t, e)
	defer c.Close()
	must(f.t, c.SetDeadline(time.Now().Add(observation)))
	must(f.t, binary.Write(c, binary.BigEndian, uint64(len(payload))))
	_, e = c.Write(payload)
	must(f.t, e)
	var size uint64
	must(f.t, binary.Read(c, binary.BigEndian, &size))
	if size > 1048576 {
		f.t.Fatal("oversized ingest response")
	}
	response := make([]byte, size)
	_, e = io.ReadFull(c, response)
	must(f.t, e)
	var out map[string]any
	must(f.t, json.Unmarshal(response[4:], &out))
	return out
}
func (f *fixture) upload(path string) map[string]any {
	raw := read(f.t, path)
	var header map[string]any
	must(f.t, json.Unmarshal(raw[4:4+binary.BigEndian.Uint32(raw)], &header))
	response := f.ingestRequest(map[string]any{"kind": "upload"}, raw)
	if response["ok"] != true {
		f.t.Fatalf("upload: %v", response)
	}
	fields := map[string]any{"kind": "preload"}
	for _, k := range []string{"plan_id", "program_id", "program_digest", "trajectory_digest", "manifest_revision", "plan_revision"} {
		fields[k] = header[k]
	}
	return fields
}
func (f *fixture) play(identity map[string]any) map[string]any {
	fields := map[string]any{}
	for k, v := range identity {
		if k != "kind" {
			fields[k] = v
		}
	}
	f.intent++
	fields["motion_intent_seq"] = f.intent
	fields["leader_lease_id"] = f.lease["lease_id"]
	fields["leader_fence_epoch"] = f.lease["leader_fence_epoch"]
	return f.command("play", fields)
}
func (f *fixture) state() control.Status {
	ctx, cancel := context.WithTimeout(context.Background(), observation)
	defer cancel()
	s, e := f.client.Status(ctx)
	must(f.t, e)
	return s
}
func (f *fixture) event(kind string, sequence uint64) bool {
	ctx, cancel := context.WithTimeout(context.Background(), observation)
	defer cancel()
	cursor := uint64(0)
	for {
		batch, e := f.client.EventsContext(ctx, cursor)
		must(f.t, e)
		for _, event := range batch.Events {
			if event.Type == kind && event.Execution != nil && event.Execution.NativeSequence == sequence {
				return true
			}
		}
		cursor = batch.NextSequence
		if cursor == batch.LatestSequence {
			return false
		}
	}
}
func (f *fixture) noLegacyOutput() {
	if f.broker.matching("motion.command", func(m map[string]any) bool { return m["command"] == "io" && m["on"] == true }) {
		f.t.Fatal("native execution emitted torch ON")
	}
	// Absolute negative observation window: 20 ms. Positive cases above require receipts/events.
	must(f.t, f.udp.SetReadDeadline(time.Now().Add(20*time.Millisecond)))
	buf := make([]byte, 2048)
	n, _, e := f.udp.ReadFrom(buf)
	if e == nil || n != 0 {
		f.t.Fatalf("native execution emitted %d UDP bytes", n)
	}
}

func TestNativeCompletion(t *testing.T) {
	f := setup(t)
	f.acquire()
	identity := f.upload(filepath.Join(f.core, "clients/cpp/examples/testdata/valid.rdt"))
	if r := f.ingestRequest(identity, nil); r["ok"] != true {
		t.Fatalf("preload: %v", r)
	}
	digest := strings.TrimPrefix(identity["trajectory_digest"].(string), "sha256:")
	record := filepath.Join(f.build, "plans", digest+".rdt")
	if !bytes.Equal(read(t, record), read(t, filepath.Join(f.core, "clients/cpp/examples/testdata/valid.rdt"))) {
		t.Fatal("stored producer bytes changed")
	}
	var prepared control.Program
	must(t, json.Unmarshal(read(t, record+".rt-core.json"), &prepared))
	if prepared.Identity.SourceDigest != identity["trajectory_digest"] || prepared.Identity.NormalisedDigest != identity["trajectory_digest"] {
		t.Fatalf("persisted identity: %+v", prepared.Identity)
	}
	if r := f.play(identity); r["ok"] != true {
		t.Fatalf("play: %v", r)
	}
	var sequence uint64
	poll(t, func() bool {
		s := f.state()
		sequence = s.Execution.NativeSequence
		return s.Execution.State == "completed"
	})
	poll(t, func() bool { return f.event("execution_completed", sequence) })
	poll(t, func() bool {
		return f.broker.matching("motion.status", func(s map[string]any) bool {
			return s["state"] == "done" && s["native_sequence"] == float64(sequence) && s["execution_generation"] == float64(1) &&
				s["segment_index"] == float64(0) && s["sample_index"] == float64(10) && s["t_s"] == float64(1)
		})
	})
	s := f.state()
	if s.Core.Axes[0].PositionCounts < 19 || s.Core.Axes[0].PositionCounts > 23 {
		t.Fatalf("endpoint counts: %d", s.Core.Axes[0].PositionCounts)
	}
	if r := f.command("stop", nil); r["ok"] != true {
		t.Fatalf("stop: %v", r)
	}
	poll(t, func() bool { s := f.state(); return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 })
	f.noLegacyOutput()
}

// Retained renewal history must not poison either acquisition on one adapter.
func TestNativeAcquisitionAfterRetainedHistory(t *testing.T) {
	f := setup(t)
	ctx, cancel := context.WithTimeout(context.Background(), observation)
	defer cancel()
	d, err := f.client.Describe(ctx)
	must(t, err)
	_, err = f.client.Acquire(ctx, "history-producer", control.Binding{PairID: "bench", Revision: 7, MachineSHA256: d.MachineSHA256})
	must(t, err)
	for i := 0; i < 300; i++ {
		_, err = f.client.Renew(ctx)
		must(t, err)
	}
	batch, err := f.client.EventsContext(ctx, 0)
	must(t, err)
	if batch.LatestSequence <= 256 {
		t.Fatalf("history not populated: %d events", batch.LatestSequence)
	}
	_, err = f.client.Release(ctx)
	must(t, err)
	plan, err := densejointtraj.Decode(read(t, filepath.Join(f.core, "clients/cpp/examples/testdata/valid.rdt")))
	must(t, err)
	for i := range plan.Segments[0].Samples {
		plan.Segments[0].Samples[i] = densejointtraj.Sample{TimeS: float64(i) * .1}
	}
	raw, err := densejointtraj.Encode(plan.Header, plan.Segments)
	must(t, err)
	path := filepath.Join(f.build, "repeatable.rdt")
	must(t, os.WriteFile(path, raw, 0600))
	for acquisition := 0; acquisition < 2; acquisition++ {
		f.acquire()
		identity := f.upload(path)
		if r := f.ingestRequest(identity, nil); r["ok"] != true {
			t.Fatalf("preload %d: %v", acquisition, r)
		}
		if r := f.play(identity); r["ok"] != true {
			t.Fatalf("play %d: %v", acquisition, r)
		}
		poll(t, func() bool { return f.state().Execution.State == "completed" })
		f.broker.clearStatus()
		poll(t, func() bool {
			return f.broker.matching("motion.status", func(s map[string]any) bool { return s["state"] == "done" })
		})
		if r := f.command("stop", nil); r["ok"] != true {
			t.Fatalf("stop %d: %v", acquisition, r)
		}
		poll(t, func() bool {
			return f.broker.matching("motion.status", func(s map[string]any) bool {
				return s["native_state"] == "inactive" && s["consumer_action"] == "acquire_and_preload"
			})
		})
	}
	f.noLegacyOutput()
}

// Gating the public Describe response proves both the subscriber and periodic
// NATS status publisher remain live throughout acquisition, including overflow.
func TestNativeStatusDuringAcquisition(t *testing.T) {
	f := setup(t, fixtureOptions{gate: "describe"})
	done := make(chan map[string]any, 1)
	go func() { done <- f.command("leader_acquire", map[string]any{"ttl_ms": 2000}) }()
	select {
	case <-f.gateEntered:
	case <-time.After(observation):
		t.Fatal("acquire did not reach Describe")
	}
	f.broker.clearStatus()
	poll(t, func() bool {
		f.broker.mu.Lock()
		defer f.broker.mu.Unlock()
		count := 0
		for _, p := range f.broker.messages {
			var s map[string]any
			if p.subject == "motion.status" && json.Unmarshal(p.body, &s) == nil && s["native_state"] == "acquiring" {
				count++
			}
		}
		return count >= 2
	})
	if r := f.command("leader_renew", map[string]any{"ttl_ms": 2000}); r["reason"] != "lease_not_held" {
		t.Fatalf("subscriber blocked or pending lease renewed: %v", r)
	}
	for i := 0; i < 257; i++ {
		body, err := json.Marshal(map[string]any{"schema": "robot.v4.robot-command.v1", "robot": "test", "sender_id": "offline-programming:test-1", "command": "leader_acquire", "command_id": fmt.Sprintf("queued-%d", i)})
		must(t, err)
		f.broker.publish(publication{"motion.command", "queue.reply", body})
	}
	poll(t, func() bool {
		return f.broker.matching("queue.reply", func(s map[string]any) bool {
			return s["command_id"] == "queued-256" && s["reason"] == "command_queue_full" && s["ok"] == false
		})
	})
	select {
	case <-done:
		t.Fatal("acquisition completed while Describe gated")
	default:
	}
	f.releaseGate()
	select {
	case r := <-done:
		if r["ok"] != true {
			t.Fatalf("acquisition: %v", r)
		}
	case <-time.After(observation):
		t.Fatal("acquisition failed to commit")
	}
}

// A real ingest refusal must preserve the native grant and Home observation,
// while repeated NATS publications expose the pending commissioning phase.
func TestNativePreloadDuringHomeObservation(t *testing.T) {
	f := setup(t, fixtureOptions{gate: "home"})
	f.acquire()
	identity := f.upload(filepath.Join(f.core, "clients/cpp/examples/testdata/valid.rdt"))
	if r := f.ingestRequest(identity, nil); r["ok"] != true {
		t.Fatalf("preload: %v", r)
	}
	play := make(chan map[string]any, 1)
	go func() { play <- f.play(identity) }()
	select {
	case <-f.gateEntered:
	case <-time.After(observation):
		t.Fatal("Home observation did not reach gate")
	}
	before := f.state()
	f.broker.clearStatus()
	if r := f.ingestRequest(identity, nil); r["ok"] != false || r["reason"] != "native_commissioning_in_progress" {
		t.Fatalf("preload refusal: %v", r)
	}
	poll(t, func() bool {
		f.broker.mu.Lock()
		defer f.broker.mu.Unlock()
		count := 0
		for _, p := range f.broker.messages {
			var s map[string]any
			if p.subject == "motion.status" && json.Unmarshal(p.body, &s) == nil &&
				s["native_state"] == "owned" && s["commissioning_phase"] == "home" {
				count++
			}
		}
		return count >= 2
	})
	after := f.state()
	if before.Grant == nil || after.Grant == nil || after.Grant.GrantActive != 1 ||
		after.Grant.AppGrantGeneration != before.Grant.AppGrantGeneration || after.Grant.CapabilityId != before.Grant.CapabilityId {
		t.Fatalf("preload changed native authority: before=%+v after=%+v", before.Grant, after.Grant)
	}
	if r := f.command("leader_renew", map[string]any{"lease_id": f.lease["lease_id"], "leader_fence_epoch": f.lease["leader_fence_epoch"], "ttl_ms": 2000}); r["ok"] != true {
		t.Fatalf("preload changed leader lease: %v", r)
	}
	select {
	case <-f.stopReceipts:
		t.Fatal("preload sent Stop during Home")
	default:
	}
	select {
	case r := <-play:
		t.Fatalf("preload terminated gated Home: %v", r)
	default:
	}
	f.releaseGate()
	select {
	case r := <-play:
		if r["ok"] != true {
			t.Fatalf("original play after preload refusal: %v", r)
		}
	case <-time.After(observation):
		t.Fatal("original Home observation did not finish")
	}
	if f.state().Execution.NativeSequence == 0 {
		t.Fatal("original preparation did not Start")
	}
}

// Status reconciliation must recover across a real adapter restart without
// granting motion to persisted preparations or disturbing another controller.
func TestNativeCleanupAfterAdapterRestart(t *testing.T) {
	f := setup(t, fixtureOptions{gate: "cleanup"})
	f.acquire()
	identity := f.upload(filepath.Join(f.core, "clients/cpp/examples/testdata/valid.rdt"))
	if r := f.ingestRequest(identity, nil); r["ok"] != true {
		t.Fatalf("preload: %v", r)
	}
	prior := f.state().AdapterIncarnation
	f.stopRenew()
	f.failStatus.Store(true)
	if r := f.command("stop", nil); r["ok"] != true {
		t.Fatalf("stop: %v", r)
	}
	poll(t, func() bool {
		return f.broker.matching("motion.status", func(s map[string]any) bool { return s["native_state"] == "cleanup_uncertain" })
	})
	f.restartControl()
	ctx, cancel := context.WithTimeout(context.Background(), observation)
	defer cancel()
	poll(t, func() bool {
		s, err := f.client.Status(ctx)
		return err == nil && s.AdapterIncarnation != prior && s.Core.TimeNS != 0 && s.Grant != nil && s.Grant.GrantActive == 0
	})
	f.failStatus.Store(false)
	d, err := f.client.Describe(ctx)
	must(t, err)
	foreign, err := f.client.Acquire(ctx, "foreign-controller", control.Binding{PairID: "bench", Revision: 7, MachineSHA256: d.MachineSHA256})
	must(t, err)
	// Unverified foreign controller renewal cadence: 100 ms.
	_, err = f.client.StartRenewal(ctx, 100*time.Millisecond)
	must(t, err)
	t.Cleanup(f.client.StopRenewal)
	poll(t, func() bool { s := f.state(); return s.Grant != nil && s.Grant.GrantActive == 1 })
	f.broker.clearStatus()
	if r := f.command("leader_acquire", map[string]any{"ttl_ms": 2000}); r["ok"] != false || r["reason"] != "native_grant_active" {
		t.Fatalf("foreign grant refusal: %v", r)
	}
	poll(t, func() bool {
		return f.broker.matching("motion.status", func(s map[string]any) bool { return s["native_state"] == "grant_active" })
	})
	renewed, err := f.client.Renew(ctx)
	must(t, err)
	if renewed.Session != foreign.Session || renewed.Generation != foreign.Generation {
		t.Fatalf("daemon changed foreign grant: %+v", renewed)
	}
	f.client.StopRenewal()
	_, err = f.client.Release(ctx)
	must(t, err)
	poll(t, func() bool { s := f.state(); return s.Grant != nil && s.Grant.GrantActive == 0 })
	f.acquire()
	if r := f.play(identity); r["ok"] != false || r["reason"] != "native_preparation_required" {
		t.Fatalf("retired preparation survived restart: %v", r)
	}
	f.acquire()
	if r := f.ingestRequest(identity, nil); r["ok"] != true {
		t.Fatalf("preload after restart: %v", r)
	}
	if r := f.play(identity); r["ok"] != true {
		t.Fatalf("play after restart reconciliation: %v", r)
	}
	poll(t, func() bool { return f.state().Execution.State == "completed" })
}

// A pending Home status response cannot delay authenticated Stop or its NATS reply.
func TestNativeStopDuringHomeObservation(t *testing.T) {
	f := setup(t, fixtureOptions{gate: "home"})
	f.acquire()
	identity := f.upload(filepath.Join(f.core, "clients/cpp/examples/testdata/valid.rdt"))
	if r := f.ingestRequest(identity, nil); r["ok"] != true {
		t.Fatalf("preload: %v", r)
	}
	play := make(chan map[string]any, 1)
	go func() { play <- f.play(identity) }()
	select {
	case <-f.gateEntered:
	case <-time.After(observation):
		t.Fatal("Home observation never reached gate")
	}
	issued := time.Now()
	stopped := make(chan map[string]any, 1)
	go func() { stopped <- f.command("stop", nil) }()
	// Unverified composed Stop ceiling: 500 ms, independent of the 10000 ms observation bound.
	const stopBound = 500 * time.Millisecond
	select {
	case r := <-stopped:
		if r["ok"] != true || time.Since(issued) > stopBound {
			t.Fatalf("Stop failed 500 ms bound: %v", r)
		}
	case <-time.After(stopBound):
		t.Fatal("Stop reply exceeded 500 ms with Home observation pending")
	}
	select {
	case receipt := <-f.stopReceipts:
		if receipt.Before(issued) || receipt.Sub(issued) > stopBound {
			t.Fatal("native Stop receipt outside 500 ms bound")
		}
	default:
		t.Fatal("rt-core did not acknowledge authenticated Stop")
	}
	select {
	case <-play:
		t.Fatal("Home observation unexpectedly completed before gate release")
	default:
	}
	s := f.state()
	if s.Core.Armed != 0 || s.Core.AxisEnableMask != 0 {
		t.Fatalf("native inhibition missing: %+v", s.Core)
	}
	f.releaseGate()
	select {
	case r := <-play:
		if r["reason"] != "native_operation_cancelled" {
			t.Fatalf("pending play: %v", r)
		}
	case <-time.After(observation):
		t.Fatal("cancelled play did not finish")
	}
	if s := f.state(); s.Execution.NativeSequence != 0 {
		t.Fatalf("cancelled observation launched Start: %+v", s.Execution)
	}
}

func TestNativeLeaseLossCancels(t *testing.T) {
	f := setup(t)
	f.acquire()
	identity := f.upload(filepath.Join(f.core, "clients/cpp/examples/testdata/valid.rdt"))
	if r := f.ingestRequest(identity, nil); r["ok"] != true {
		t.Fatalf("preload: %v", r)
	}
	if r := f.play(identity); r["ok"] != true {
		t.Fatalf("play: %v", r)
	}
	s := f.state()
	if s.Execution.State != "executing" {
		t.Fatalf("expected active execution: %+v", s.Execution)
	}
	sequence := s.Execution.NativeSequence
	f.stopRenew()
	if r := f.command("leader_release", map[string]any{"lease_id": f.lease["lease_id"], "leader_fence_epoch": f.lease["leader_fence_epoch"]}); r["ok"] != true {
		t.Fatalf("release: %v", r)
	}
	poll(t, func() bool { return f.state().Execution.State == "cancelled" })
	poll(t, func() bool { return f.event("execution_aborted", sequence) })
	poll(t, func() bool {
		return f.broker.matching("motion.status", func(s map[string]any) bool { return s["state"] == "aborted" && s["execution_state"] == "cancelled" })
	})
	if r := f.play(identity); r["ok"] != false {
		t.Fatalf("stale play accepted: %v", r)
	}
	f.noLegacyOutput()
}

func TestNativeTorchRefusedAtPreload(t *testing.T) {
	f := setup(t)
	f.acquire()
	plan, e := densejointtraj.Decode(read(t, filepath.Join(f.core, "clients/cpp/examples/testdata/valid.rdt")))
	must(t, e)
	plan.Header.Segments[0].Kind = "weld"
	plan.Segments[0].Kind = "weld"
	plan.Segments[0].Samples[1].TorchOn = true
	raw, e := densejointtraj.Encode(plan.Header, plan.Segments)
	must(t, e)
	path := filepath.Join(f.build, "torch.rdt")
	must(t, os.WriteFile(path, raw, 0600))
	identity := f.upload(path)
	for attempt := 0; attempt < 4; attempt++ {
		if r := f.ingestRequest(identity, nil); r["ok"] != false || r["reason"] != "native_torch_unsupported" {
			t.Fatalf("torch preload: %v", r)
		}
	}
	if handles := f.state().Execution.Handles; len(handles) != 0 {
		t.Fatalf("torch refusal left resident handles: %+v", handles)
	}
	poll(t, func() bool {
		return f.broker.matching("motion.status", func(s map[string]any) bool {
			return s["abort_reason"] == "native_torch_unsupported" && s["torch_on"] == false
		})
	})
	if r := f.play(identity); r["ok"] != false {
		t.Fatalf("torch play: %v", r)
	}
	f.noLegacyOutput()
}

// A deterministic stationary program remains active across either lease expiry;
// cancellation must come from observed authority loss, never the program clock.
func TestNativeLeaseExpiry(t *testing.T) {
	for _, native := range []bool{false, true} {
		t.Run(fmt.Sprintf("native=%v", native), func(t *testing.T) {
			f := setup(t)
			f.acquire()
			plan, e := densejointtraj.Decode(read(t, filepath.Join(f.core, "clients/cpp/examples/testdata/valid.rdt")))
			must(t, e)
			// Unverified fixture size/grid: 101 stationary samples at 100 ms spacing.
			samples := make([]densejointtraj.Sample, 101)
			for i := range samples {
				samples[i].TimeS = float64(i) * .1
			}
			plan.Segments[0].Samples = samples
			plan.Header.Segments[0].SampleCount = 101
			plan.Header.Segments[0].DurationS = 10
			plan.Header.Sampling.TotalSampleCount = 101
			plan.Header.Sampling.TotalDurationS = 10
			raw, e := densejointtraj.Encode(plan.Header, plan.Segments)
			must(t, e)
			path := filepath.Join(f.build, "dwell.rdt")
			must(t, os.WriteFile(path, raw, 0600))
			identity := f.upload(path)
			if r := f.ingestRequest(identity, nil); r["ok"] != true {
				t.Fatalf("preload: %v", r)
			}
			if r := f.play(identity); r["ok"] != true {
				t.Fatalf("play: %v", r)
			}
			s := f.state()
			if s.Execution.State != "executing" {
				t.Fatalf("not executing: %+v", s.Execution)
			}
			sequence := s.Execution.NativeSequence
			f.stopRenew()
			if native {
				must(t, f.daemon.Process.Signal(syscall.SIGSTOP))
			} else {
				r := f.command("leader_renew", map[string]any{"lease_id": f.lease["lease_id"], "leader_fence_epoch": f.lease["leader_fence_epoch"], "ttl_ms": 500})
				if r["ok"] != true {
					t.Fatalf("renew: %v", r)
				}
			}
			poll(t, func() bool {
				s := f.state()
				return s.Execution.State == "cancelled" && s.Core.Armed == 0 && s.Core.AxisEnableMask == 0
			})
			if native {
				must(t, f.daemon.Process.Signal(syscall.SIGCONT))
			}
			poll(t, func() bool { return f.event("execution_aborted", sequence) })
			poll(t, func() bool {
				return f.broker.matching("motion.status", func(s map[string]any) bool { return s["state"] == "aborted" && s["execution_state"] == "cancelled" })
			})
			f.acquire()
			if r := f.play(identity); r["ok"] != false {
				t.Fatalf("expired preparation started: %v", r)
			}
			f.noLegacyOutput()
		})
	}
}
