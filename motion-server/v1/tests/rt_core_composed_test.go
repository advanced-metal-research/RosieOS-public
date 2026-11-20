//go:build linux

package cartesian_test

import (
	"bufio"
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"io"
	"math"
	"net"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/simfixture"
	"runtime"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"testing"
	"time"
)

// This broker implements only the core NATS operations used by cartesiand.
// It forwards actual subscriber traffic so queued acceptance cannot stand in
// for the daemon's application status or the simulator's joint observation.
type broker struct {
	listener     net.Listener
	mu           sync.Mutex
	subscribers  map[net.Conn]map[string]string
	writes       map[net.Conn]*sync.Mutex
	messages     map[string][]byte
	permitSource bool
	sourceEpoch  time.Time
}

func newBroker(t *testing.T) *broker {
	t.Helper()
	listener, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	b := &broker{listener: listener, subscribers: make(map[net.Conn]map[string]string), writes: make(map[net.Conn]*sync.Mutex), messages: make(map[string][]byte)}
	t.Cleanup(func() {
		listener.Close()
		b.mu.Lock()
		defer b.mu.Unlock()
		for c := range b.subscribers {
			c.Close()
		}
	})
	go func() {
		for {
			c, err := listener.Accept()
			if err != nil {
				return
			}
			b.mu.Lock()
			b.subscribers[c] = make(map[string]string)
			b.writes[c] = &sync.Mutex{}
			b.mu.Unlock()
			go b.serve(c)
		}
	}()
	return b
}
func (b *broker) serve(c net.Conn) {
	defer func() { c.Close(); b.mu.Lock(); delete(b.subscribers, c); delete(b.writes, c); b.mu.Unlock() }()
	io.WriteString(c, "INFO {\"max_payload\":1048576}\r\n")
	reader := bufio.NewReader(c)
	for {
		line, err := reader.ReadString('\n')
		if err != nil {
			return
		}
		fields := strings.Fields(line)
		if len(fields) == 0 {
			continue
		}
		switch fields[0] {
		case "PING":
			b.mu.Lock()
			w := b.writes[c]
			b.mu.Unlock()
			w.Lock()
			io.WriteString(c, "PONG\r\n")
			w.Unlock()
		case "SUB":
			if len(fields) != 3 {
				return
			}
			b.mu.Lock()
			b.subscribers[c][fields[1]] = fields[2]
			b.mu.Unlock()
		case "PUB":
			size, err := strconv.Atoi(fields[len(fields)-1])
			if err != nil || size > 1048576 {
				return
			}
			body := make([]byte, size+2)
			if _, err = io.ReadFull(reader, body); err != nil {
				return
			}
			if len(fields) == 4 {
				b.publish(fields[1], body[:size], fields[2])
			} else {
				b.publish(fields[1], body[:size])
			}
		}
	}
}
func (b *broker) publish(subject string, body []byte, reply ...string) {
	b.mu.Lock()
	var permitReply []byte
	if b.permitSource && len(reply) == 1 {
		var request struct {
			Command  string         `json:"command"`
			Position map[string]any `json:"position"`
		}
		decoder := json.NewDecoder(bytes.NewReader(body))
		decoder.UseNumber()
		if decoder.Decode(&request) == nil && request.Command == "position_execution_admit" {
			p := request.Position
			observed := uint64(time.Since(b.sourceEpoch).Milliseconds())
			deadline, _ := strconv.ParseUint(fmt.Sprint(p["deadline_monotonic_ms"]), 10, 64)
			runDeadline, _ := strconv.ParseUint(fmt.Sprint(p["run_deadline_monotonic_ms"]), 10, 64)
			if observed < deadline && deadline <= runDeadline {
				p["schema"] = "robot.v4.bridge-position-permit.v1"
				p["bridge_observed_monotonic_ms"] = observed
				p["remaining_ms"] = deadline - observed
				p["run_remaining_ms"] = runDeadline - observed
				p["runtime_mode_epoch"], p["activation_epoch"], p["home_epoch"], p["pair_revision"] = 1, 1, 1, 1
				p["pair_config_sha256"] = "source-config"
				permitReply, _ = json.Marshal(p)
			}
		}
	}
	defer func() {
		b.mu.Unlock()
		if permitReply != nil {
			b.publish(reply[0], permitReply)
		}
	}()
	b.messages[subject] = append([]byte(nil), body...)
	for c, subjects := range b.subscribers {
		if sid, ok := subjects[subject]; ok {
			b.writes[c].Lock()
			c.SetWriteDeadline(time.Now().Add(time.Second))
			if len(reply) == 1 {
				fmt.Fprintf(c, "MSG %s %s %s %d\r\n%s\r\n", subject, sid, reply[0], len(body), body)
			} else {
				fmt.Fprintf(c, "MSG %s %s %d\r\n%s\r\n", subject, sid, len(body), body)
			}
			b.writes[c].Unlock()
		}
	}
}
func (b *broker) published(subject string, value any) bool {
	b.mu.Lock()
	defer b.mu.Unlock()
	return json.Unmarshal(b.messages[subject], value) == nil
}
func (b *broker) subscribed(subject string) bool {
	b.mu.Lock()
	defer b.mu.Unlock()
	for _, subjects := range b.subscribers {
		if _, ok := subjects[subject]; ok {
			return true
		}
	}
	return false
}

func eventually(t *testing.T, label string, predicate func() bool) {
	t.Helper()
	// Unverified fixture observation budget. Success requires the predicate.
	deadline := time.NewTimer(15 * time.Second)
	defer deadline.Stop()
	ticker := time.NewTicker(10 * time.Millisecond)
	defer ticker.Stop()
	for {
		if predicate() {
			return
		}
		select {
		case <-ticker.C:
		case <-deadline.C:
			t.Fatal("observation unavailable:", label)
		}
	}
}

// startProcess reaps every child and preserves exit status and output, so a
// daemon startup refusal cannot be reported only as a missing observation.
func startProcess(t *testing.T, directory, name, binary string, args ...string) *exec.Cmd {
	t.Helper()
	logfile, err := os.Create(filepath.Join(directory, name+".log"))
	if err != nil {
		t.Fatal(err)
	}
	command := exec.Command(binary, args...)
	command.Dir = directory
	command.Stdout = logfile
	command.Stderr = logfile
	if err = command.Start(); err != nil {
		logfile.Close()
		t.Fatal(err)
	}
	done := make(chan error, 1)
	go func() { done <- command.Wait() }()
	t.Cleanup(func() {
		var waitErr error
		exitedBeforeCleanup := false
		select {
		case waitErr = <-done:
			exitedBeforeCleanup = true
		default:
			command.Process.Signal(syscall.SIGTERM)
			// Unverified shutdown ceiling; process exit is the completion condition.
			select {
			case waitErr = <-done:
			case <-time.After(3 * time.Second):
				command.Process.Kill()
				waitErr = <-done
			}
		}
		logfile.Close()
		if t.Failed() {
			data, _ := os.ReadFile(filepath.Join(directory, name+".log"))
			t.Logf("%s: exited_before_cleanup=%t exit_code=%d wait_error=%v\nstdout/stderr:\n%s",
				name, exitedBeforeCleanup, command.ProcessState.ExitCode(), waitErr, data)
		}
	})
	return command
}

type axisStatus struct {
	Target    float64 `json:"logical_target"`
	Counts    int64   `json:"pos_counts"`
	Readiness string  `json:"readiness"`
}
type coreStatus struct {
	Armed      uint32       `json:"armed"`
	TimeNS     uint64       `json:"time_ns"`
	Axes       []axisStatus `json:"axes"`
	Verified   uint32       `json:"config_verified_mask"`
	HomeActive uint32       `json:"native_home_active_axis_mask"`
}
type observedStatus struct {
	Core  coreStatus `json:"core"`
	Grant struct {
		Active uint32 `json:"GrantActive"`
	} `json:"grant"`
	Jog struct {
		Open                   uint32
		ObservedSourceSequence uint64
		JogGeneration          uint64
	} `json:"jog"`
}
type publishedStatus struct {
	Backend       string          `json:"backend"`
	Armed         bool            `json:"servos_armed_requested"`
	Fresh         bool            `json:"latest_input_fresh"`
	LeaderLease   string          `json:"leader_lease_id"`
	LeaderEpoch   uint64          `json:"leader_fence_epoch"`
	LeaderFresh   bool            `json:"leader_lease_fresh"`
	Refusal       string          `json:"refusal"`
	TypedRefusal  json.RawMessage `json:"typed_refusal"`
	Outputs       uint64          `json:"rtcore_outputs_sent"`
	Rejected      uint64          `json:"rejected_input_count"`
	Sequence      uint32          `json:"latest_sequence"`
	V4Available   bool            `json:"v4_fields_available"`
	StopConfirmed bool            `json:"last_stop_confirmed"`
	Status        observedStatus  `json:"rt_core_status"`
}

func TestCartesianJogThroughRTCoreAndExternalStop(t *testing.T) {
	runCartesianFixture(t, false)
}

func TestNativePositionGoHomeThroughOwner(t *testing.T) {
	runCartesianFixture(t, true)
}

func TestNativePositionRefusesUnresolvedInput(t *testing.T) {
	runCartesianFixture(t, true, "pose")
}

func TestNativeFreshGoHomeNotAdmitted(t *testing.T) {
	runCartesianFixture(t, true, "fresh_home")
}

func TestNativeStopDuringReceipt(t *testing.T) {
	for _, operation := range []string{"prepare_trajectory", "start_trajectory", "home_enable"} {
		t.Run(operation, func(t *testing.T) { runCartesianFixture(t, true, operation) })
	}
}

// The proxy gates one real native receipt while retaining independent Stop traffic.
// Operation ordering exposes starts that overtake an acknowledged ingress Stop.
type nativeReceiptProxy struct {
	mu               sync.Mutex
	operation        string
	entered, release chan struct{}
	operations       []string
}

func newNativeReceiptProxy(t *testing.T, socket, upstream string) *nativeReceiptProxy {
	p := &nativeReceiptProxy{}
	transport := &http.Transport{DialContext: func(ctx context.Context, _, _ string) (net.Conn, error) {
		return (&net.Dialer{}).DialContext(ctx, "unix", upstream)
	}}
	listener, err := net.Listen("unix", socket)
	if err != nil {
		t.Fatal(err)
	}
	server := &http.Server{Handler: http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		body, _ := io.ReadAll(r.Body)
		var command struct {
			Operation string `json:"operation"`
		}
		_ = json.Unmarshal(body, &command)
		p.mu.Lock()
		if command.Operation != "" {
			p.operations = append(p.operations, command.Operation)
		}
		held := command.Operation != "" && command.Operation == p.operation
		entered, release := p.entered, p.release
		if held {
			p.operation = ""
			p.operations = []string{command.Operation}
		}
		p.mu.Unlock()
		request, _ := http.NewRequestWithContext(r.Context(), r.Method, "http://unix"+r.URL.RequestURI(), bytes.NewReader(body))
		request.Header = r.Header.Clone()
		response, err := transport.RoundTrip(request)
		if err != nil {
			http.Error(w, err.Error(), 502)
			return
		}
		defer response.Body.Close()
		responseBody, _ := io.ReadAll(response.Body)
		if held {
			close(entered)
			select {
			case <-release:
			case <-r.Context().Done():
				return
			}
		}
		for k, v := range response.Header {
			w.Header()[k] = v
		}
		w.WriteHeader(response.StatusCode)
		_, _ = w.Write(responseBody)
	})}
	go func() { _ = server.Serve(listener) }()
	t.Cleanup(func() { _ = server.Close(); transport.CloseIdleConnections() })
	return p
}

func (p *nativeReceiptProxy) hold(operation string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.operation = operation
	p.entered = make(chan struct{})
	p.release = make(chan struct{})
	p.operations = nil
}

func (p *nativeReceiptProxy) wait(t *testing.T) {
	t.Helper()
	// Unverified observation ceiling: 5000 ms; the receipt gate is the pass condition.
	select {
	case <-p.entered:
	case <-time.After(5 * time.Second):
		t.Fatal("native receipt gate not reached")
	}
}

func (p *nativeReceiptProxy) snapshot() []string {
	p.mu.Lock()
	defer p.mu.Unlock()
	return append([]string(nil), p.operations...)
}

func runCartesianFixture(t *testing.T, nativePosition bool, cases ...string) {
	scenario := ""
	if len(cases) != 0 {
		scenario = cases[0]
	}

	_, source, _, ok := runtime.Caller(0)
	if !ok {
		t.Fatal("cannot locate Cartesian composed test source")
	}
	root, err := filepath.Abs(filepath.Join(filepath.Dir(source), "../../../rt-core"))
	if err != nil {
		t.Fatal(err)
	}
	cartesiand := os.Getenv("CARTESIAND_BINARY")
	if cartesiand == "" {
		cartesiand = filepath.Join(root, "build/cartesiand/robot-v4-cartesiand")
	}
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "cartesian-composed-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { os.RemoveAll(build) })
	sockets, err := os.MkdirTemp(os.Getenv("TMPDIR"), "cj-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { os.RemoveAll(sockets) })
	for _, binary := range []string{"rtctl", "rt-control", "rosie-rt-core-sim"} {
		if _, err := os.Stat(filepath.Join(root, "build", binary)); err != nil {
			t.Fatal("build dependency missing:", binary, err)
		}
	}
	machine := map[string]any{"schema_version": 1, "backend": "simulation", "cycle_ns": 1000000, "max_motor_rpm": 10,
		"max_cycle_lateness_ns": 20000000, "runtime": map[string]any{"cpu": 2, "priority": 90}}
	axes := []map[string]any{}
	for i := 0; i < 6; i++ {
		profile := "cia402_sim"
		if nativePosition {
			profile = "cia402_sim_home"
		}
		// Unverified synthetic encoder: 131072 motor counts per revolution, matching simulation-program.json.
		axes = append(axes, map[string]any{"name": fmt.Sprintf("J%d", i+1), "slave_position": i, "profile": profile, "type": "rotary",
			"motor_encoder_counts_per_rev": 131072, "coordinate_evidence_mode": "multi_turn",
			"wire_counts_per_rev": 131072, "wire_revs_per_axis_rev": 1, "sign": 1, "require_home": false,
			"limits": map[string]any{"min": -2, "max": 2, "max_velocity": 0.1, "jog_acceleration": 1.0, "max_target_lead": 0.02,
				"following_error": 0.04, "following_error_timeout_ms": 100, "completion_tolerance": 0.0002, "completion_timeout_ms": 500}})
	}
	machine["axes"] = axes
	raw, _ := json.Marshal(machine)
	machineFile := filepath.Join(build, "machine.json")
	if err = os.WriteFile(machineFile, raw, 0600); err != nil {
		t.Fatal(err)
	}
	compiled, args, err := simfixture.Compile(root, machineFile, filepath.Join(build, "config"))
	if err != nil {
		t.Fatal(err)
	}
	hash := filepath.Base(compiled)
	ipc := filepath.Join(sockets, "ipc.sock")
	control := filepath.Join(sockets, "control.sock")
	args = append(args, "--socket-path", ipc)
	startProcess(t, build, "sim", filepath.Join(root, "build/rosie-rt-core-sim"), args...)
	// Unverified diagnostic budget: ten native publisher periods of 100 ms
	// (kDiagnosticsPublisherPeriodNs, rt-core/engine/diagnostics.hpp); change
	// it with that period. It bounds only the first-fault report, never a pass.
	const firstFaultBudget = time.Second
	metrics := filepath.Join(sockets, "metrics.json")
	firstFaultReported := false
	// Registered after the simulator, so it runs while the daemon still
	// publishes: a receipt that says only "context canceled" names its cause.
	t.Cleanup(func() {
		if t.Failed() && !firstFaultReported {
			t.Log(simfixture.FirstFault(metrics, firstFaultBudget))
		}
	})
	eventually(t, "native socket", func() bool { _, e := os.Stat(ipc); return e == nil })
	startProcess(t, build, "control", filepath.Join(root, "build/rt-control"), "--core-socket", ipc, "--socket", control,
		"--configuration-sha256", hash, "--backend", "simulation", "--pair-id", "cartesian-test", "--pair-revision", "1")
	transport := &http.Transport{DialContext: func(ctx context.Context, _, _ string) (net.Conn, error) {
		return (&net.Dialer{}).DialContext(ctx, "unix", control)
	}}
	t.Cleanup(transport.CloseIdleConnections)
	httpClient := &http.Client{Transport: transport, Timeout: time.Second}
	request := func(method, path string, body any, out any) error {
		var encoded []byte
		if body != nil {
			encoded, _ = json.Marshal(body)
		}
		req, _ := http.NewRequest(method, "http://unix"+path, bytes.NewReader(encoded))
		req.Header.Set("Content-Type", "application/json")
		response, e := httpClient.Do(req)
		if e != nil {
			return e
		}
		defer response.Body.Close()
		var envelope struct {
			Data  json.RawMessage `json:"data"`
			Error string          `json:"error"`
		}
		if e = json.NewDecoder(response.Body).Decode(&envelope); e != nil {
			return e
		}
		if envelope.Error != "" {
			return fmt.Errorf("%s", envelope.Error)
		}
		if out != nil {
			return json.Unmarshal(envelope.Data, out)
		}
		return nil
	}
	var status observedStatus
	eventually(t, "public status", func() bool { return request("GET", "/v1/status", nil, &status) == nil })
	binding := map[string]any{"pair_id": "cartesian-test", "revision": 1, "configuration_sha256": hash}
	var oldFence struct {
		Session    string `json:"session"`
		Generation uint64 `json:"generation"`
	}
	if err = request("POST", "/v1/control", map[string]any{"schema": "rosie.rt-control.request.v1", "operation": "acquire", "controller": "external-stop-test", "binding": binding}, &oldFence); err != nil {
		t.Fatal(err)
	}
	if err = request("POST", "/v1/control", map[string]any{"schema": "rosie.rt-control.request.v1", "operation": "release", "session": oldFence.Session, "generation": oldFence.Generation}, nil); err != nil {
		t.Fatal(err)
	}
	// Synthetic six-axis arm is nonsingular at zero and exercises the unchanged
	// Cartesian resolver without requiring a private deployment URDF.
	origins := []string{"0 0 0", "-1 1 0", "1 -1 0", "1 1 1", "0 0 0", "0 0 0"}
	directions := []string{"0 0 1", "0 1 0", "1 0 0", "1 0 0", "0 1 0", "0 0 1"}
	urdf := "<robot name=\"cartesian_fixture\">"
	for i := 0; i < 6; i++ {
		urdf += fmt.Sprintf("<joint name=\"J%d\" type=\"revolute\"><origin xyz=\"%s\" rpy=\"0 0 0\"/><axis xyz=\"%s\"/></joint>", i+1, origins[i], directions[i])
	}
	urdf += "<joint name=\"link_6_to_end_effector\" type=\"fixed\"><origin xyz=\"0 0 0\" rpy=\"0 0 0\"/></joint></robot>"
	urdfFile := filepath.Join(build, "fixture.urdf")
	os.WriteFile(urdfFile, []byte(urdf), 0600)
	urdfHash := fmt.Sprintf("%x", sha256.Sum256([]byte(urdf)))
	b := newBroker(t)
	b.mu.Lock()
	b.permitSource = nativePosition
	b.sourceEpoch = time.Now()
	b.mu.Unlock()
	subject := "robot/v4/robot.jetson-test.command"
	statusFile := filepath.Join(build, "status.json")
	readyFile := filepath.Join(build, "ready")
	proxySocket := filepath.Join(sockets, "proxy.sock")
	review := newNativeReceiptProxy(t, proxySocket, control)
	startProcess(t, build, "cartesiand", cartesiand,
		"--backend", "rt_core", "--rt-control-socket", proxySocket, "--rt-jog-socket", filepath.Join(sockets, "jog.sock"),
		"--rt-pair-id", "cartesian-test", "--rt-pair-revision", "1", "--rt-configuration-sha256", hash,
		"--rt-axis-ids", "J1,J2,J3,J4,J5,J6", "--rt-urdf", urdfFile, "--rt-urdf-sha256", urdfHash,
		"--udp-listen", "127.0.0.1:0", "--udp-ready-file", readyFile, "--status-output", statusFile,
		"--nats-url", "nats://"+b.listener.Addr().String(), "--robot-command-subject", subject,
		"--robot-cell", "jetson-test", "--nats-status-subject", "robot/v4/motion-server.jetson-test.status", "--trusted-lan-leader-authority")
	// Read before process cleanup can replace the original refusal with shutdown.
	t.Cleanup(func() {
		if t.Failed() {
			data, err := os.ReadFile(statusFile)
			t.Logf("cartesiand status: %s (read_error=%v)", data, err)
		}
	})
	var published publishedStatus
	readPublished := func() bool {
		data, e := os.ReadFile(statusFile)
		return e == nil && json.Unmarshal(data, &published) == nil
	}
	eventually(t, "cartesiand command subscription", func() bool { return b.subscribed(subject) })
	sendCold := func(command, id string, extra map[string]any) {
		body := map[string]any{"schema": "robot.v4.robot-command.v1", "robot": "jetson-test", "command": command, "command_id": id,
			"sender_id": "steamdeck:steamdeck-test", "controller_boot_id": "test-boot", "request_id": id, "campaign_generation": 1}
		for k, v := range extra {
			body[k] = v
		}
		payload, _ := json.Marshal(body)
		b.publish(subject, payload, "_INBOX.native."+id)
	}
	sendCold("leader_acquire", "acquire-1", map[string]any{"ttl_ms": 5000})
	eventually(t, "leader acquired", func() bool { return readPublished() && published.LeaderFresh })
	lease, epoch := published.LeaderLease, published.LeaderEpoch
	// Unverified fixture renewal cadence: 250 ms. Terminal checks require a
	// live source lease; extra round trips must not silently let it expire.
	const renewalCadence = 250 * time.Millisecond
	stopRenewals, renewalsDone := make(chan struct{}), make(chan struct{})
	go func() {
		defer close(renewalsDone)
		ticker := time.NewTicker(renewalCadence)
		defer ticker.Stop()
		for sequence := 1; ; sequence++ {
			select {
			case <-stopRenewals:
				return
			case <-ticker.C:
				sendCold("leader_renew", fmt.Sprintf("renew-%d", sequence), map[string]any{"lease_id": lease, "fence_epoch": epoch, "ttl_ms": 2000})
			}
		}
	}()
	defer func() { close(stopRenewals); <-renewalsDone }()
	eventually(t, "verified native configuration", func() bool { return readPublished() && published.Status.Core.Verified == 63 })
	if !nativePosition {
		// Unbound typed IDs must fail and replay without acquiring a native run.
		for _, id := range []any{json.Number("9007199254740993"), "9007199254740993"} {
			extra := map[string]any{"command_id": id, "axis": 0, "target_rad": 0.01,
				"leader_lease_id": lease, "leader_fence_epoch": epoch}
			encodedID, _ := json.Marshal(id)
			var terminal struct {
				CommandID json.RawMessage `json:"command_id"`
				State     string          `json:"state"`
				Accepted  bool            `json:"accepted"`
			}
			sendCold("position", "position-refused", extra)
			eventually(t, "native typed position failure", func() bool {
				return b.published("_INBOX.native.position-refused", &terminal) &&
					bytes.Equal(terminal.CommandID, encodedID) && terminal.State == "failed" && !terminal.Accepted
			})
			b.mu.Lock()
			delete(b.messages, "_INBOX.native.position-refused")
			b.mu.Unlock()
			sendCold("position", "position-refused", extra)
			eventually(t, "native terminal replay", func() bool {
				return b.published("_INBOX.native.position-refused", &terminal) &&
					bytes.Equal(terminal.CommandID, encodedID) && terminal.State == "failed" && !terminal.Accepted
			})
		}
		sendCold("home", "native-home", map[string]any{"leader_lease_id": lease, "leader_fence_epoch": epoch})
		var homeTerminal struct {
			State    string `json:"state"`
			Accepted bool   `json:"accepted"`
			Error    string `json:"error"`
		}
		eventually(t, "native Home terminal receipt", func() bool {
			if readPublished() && published.Armed {
				t.Fatal("native Home implicitly armed")
			}
			return b.published("_INBOX.native.native-home", &homeTerminal) && homeTerminal.State != "accepted" && homeTerminal.State != ""
		})
		// This fixture has no Home recipe. Its exact native refusal must reach the
		// owner terminal instead of treating queue/Enable acceptance as completion.
		if homeTerminal.State != "failed" || homeTerminal.Accepted || homeTerminal.Error != "native_rejected" {
			readPublished()
			t.Fatalf("native Home terminal: %+v; backend refusal: %s %s", homeTerminal, published.Refusal, published.TypedRefusal)
		}
		if readPublished() && published.Armed {
			t.Fatal("native Home implicitly armed")
		}
		eventually(t, "native Home refusal published", func() bool {
			if !readPublished() {
				return false
			}
			if published.Armed {
				t.Fatal("native Home implicitly armed")
			}
			var homeRefusal struct {
				Result struct{ Operation, Reason, AxisMask uint64 } `json:"native_result"`
			}
			return json.Unmarshal(published.TypedRefusal, &homeRefusal) == nil &&
				homeRefusal.Result.Operation == 264 && homeRefusal.Result.Reason == 2 &&
				homeRefusal.Result.AxisMask == 63
		})
	}
	if nativePosition {
		eventually(t, "verified Home configuration", func() bool { return readPublished() && published.Status.Core.Verified == 63 })
		sendCold("home", "initial-home", map[string]any{"leader_lease_id": lease, "leader_fence_epoch": epoch})
		var initial struct {
			State  string          `json:"state"`
			Error  string          `json:"error"`
			Native json.RawMessage `json:"native"`
		}
		eventually(t, "initial native coordinate evidence", func() bool {
			return b.published("_INBOX.native.initial-home", &initial) && initial.State != "" && initial.State != "accepted"
		})
		if initial.State != "completed" {
			firstFaultReported = true
			t.Fatalf("initial Home: %s %s %s; %s", initial.State, initial.Error, initial.Native,
				simfixture.FirstFault(metrics, firstFaultBudget))
		}
	}
	sendCold("arm", "arm-1", map[string]any{"leader_lease_id": lease, "leader_fence_epoch": epoch, "armed": true})
	eventually(t, "operator arm and six ready axes", func() bool {
		if !readPublished() || !published.Armed || len(published.Status.Core.Axes) != 6 {
			return false
		}
		for _, a := range published.Status.Core.Axes {
			if a.Readiness != "ready" {
				return false
			}
		}
		return true
	})
	if nativePosition {
		type nativeTerminal struct {
			CommandID json.RawMessage `json:"command_id"`
			State     string          `json:"state"`
			Error     string          `json:"error"`
			Native    struct {
				Session             string `json:"session"`
				Generation          uint64 `json:"generation"`
				RunID               string `json:"run_id"`
				Handle              uint64 `json:"handle"`
				HomeEpoch           uint64 `json:"completed_home_epoch"`
				ExecutionGeneration uint64 `json:"execution_generation"`
				Identity            struct {
					ProgramID string `json:"program_id"`
				} `json:"identity"`
			} `json:"native"`
		}
		credentials := func(sequence int) map[string]any {
			// Unverified source fixture deadlines: intent 18,000 ms, run 20,000 ms from source boot.
			return map[string]any{"leader_lease_id": lease, "leader_fence_epoch": epoch,
				"bridge_boot_id": "source-boot", "run_id": "native-run", "controller_id": "steamdeck:steamdeck-test",
				"controller_boot_id": "test-boot", "deadline_monotonic_ms": 18000, "run_deadline_monotonic_ms": 20000,
				"motion_intent_seq": sequence, "axis": 0, "target_rad": 0.01, "max_speed_rad_s": 0.05}
		}
		if scenario == "fresh_home" {
			review.hold("")
			for i, extra := range []map[string]any{credentials(1), {"leader_lease_id": lease, "leader_fence_epoch": epoch}} {
				id := fmt.Sprintf("fresh-home-%d", i)
				var result nativeTerminal
				sendCold("go_home", id, extra)
				eventually(t, "fresh GoHome refusal", func() bool {
					return b.published("_INBOX.native."+id, &result) && result.State != "" && result.State != "accepted"
				})
				if result.State != "failed" || result.Error != "rt_core_run_not_admitted" {
					t.Fatalf("fresh GoHome: %+v", result)
				}
			}
			for _, op := range review.snapshot() {
				if op == "acquire" || op == "enable" || op == "home" || op == "prepare_trajectory" || op == "start_trajectory" {
					t.Fatalf("unadmitted GoHome issued %s", op)
				}
			}
			return
		}
		if scenario == "pose" {
			var result nativeTerminal
			awaitTerminal := func(id string) {
				eventually(t, "unresolved input terminal", func() bool {
					return b.published("_INBOX.native."+id, &result) && result.State != "" && result.State != "accepted"
				})
			}
			sendCold("position", "seed", credentials(1))
			awaitTerminal("seed")
			if result.State != "completed" {
				t.Fatalf("seed: %+v", result)
			}
			eventually(t, "held seed target", func() bool {
				return readPublished() && published.Status.Core.Axes[0].Target > .0099 && published.Status.Core.Axes[0].Target < .0101
			})
			held := append([]axisStatus(nil), published.Status.Core.Axes...)
			review.hold("")
			for i, kind := range []string{"pose", "missing", "partial", "vector", "nan-vector", "ambiguous"} {
				extra := credentials(i + 2)
				delete(extra, "axis")
				delete(extra, "target_rad")
				switch kind {
				case "pose":
					extra["tcp_pose"] = []float64{.2, .1, .3, 0, 0, 0}
				case "partial":
					extra["axis"] = 0
				case "vector":
					extra["joint_targets"] = []float64{.02, .01}
				case "nan-vector":
					extra["joint_targets"] = []any{"NaN", 0, 0, 0, 0, 0}
				case "ambiguous":
					extra["axis"] = 0
					extra["target_rad"] = .02
					extra["target_deg"] = 1
				}
				sendCold("position", kind, extra)
				awaitTerminal(kind)
				if result.State != "failed" || result.Error != "rt_core_position_input_unresolved" {
					t.Fatalf("%s: %+v", kind, result)
				}
				if err := request("GET", "/v1/status", nil, &status); err != nil {
					t.Fatal(err)
				}
				for axis, before := range held {
					if status.Core.Axes[axis].Target != before.Target {
						t.Fatalf("%s moved held axis %d", kind, axis)
					}
				}
			}
			for _, op := range review.snapshot() {
				if op == "acquire" || op == "prepare_trajectory" || op == "start_trajectory" {
					t.Fatalf("unresolved input issued %s", op)
				}
			}
			return
		}
		if scenario == "prepare_trajectory" || scenario == "start_trajectory" || scenario == "home_enable" {
			var result nativeTerminal
			awaitTerminal := func(id string) {
				eventually(t, "gated native terminal", func() bool {
					return b.published("_INBOX.native."+id, &result) && result.State != "" && result.State != "accepted"
				})
			}
			operation := scenario
			if scenario == "home_enable" {
				sendCold("position", "seed", credentials(1))
				awaitTerminal("seed")
				if result.State != "completed" {
					var events json.RawMessage
					if err := request("GET", "/v1/events?after=0", nil, &events); err != nil {
						t.Logf("seed native events unavailable: %v", err)
					} else {
						t.Logf("seed native events: %s", events)
					}
					t.Fatalf("seed: %+v", result)
				}
				operation = "enable"
			}
			review.hold(operation)
			if scenario == "home_enable" {
				sendCold("go_home", "gated", credentials(2))
			} else {
				sendCold("position", "gated", credentials(1))
			}
			review.wait(t)
			sendCold("stop", "gated-stop", nil)
			var ack struct {
				Accepted bool `json:"accepted"`
			}
			eventually(t, "Stop admitted by subscriber", func() bool { return b.published("_INBOX.native.gated-stop", &ack) && ack.Accepted })
			// The held receipt is never released until the urgent fence arrives.
			eventually(t, "Stop bypasses held receipt", func() bool {
				for _, op := range review.snapshot() {
					if op == "stop" {
						return true
					}
				}
				return false
			})
			close(review.release)
			awaitTerminal("gated")
			if result.State != "cancelled" || result.Error != "rt_core_execution_cancelled" {
				t.Fatalf("gated terminal: %+v", result)
			}
			eventually(t, "Stop confirmed and grant released", func() bool { return readPublished() && published.StopConfirmed && published.Status.Grant.Active == 0 })
			var native []string
			for _, op := range review.snapshot() {
				if op == "prepare_trajectory" || op == "start_trajectory" || op == "enable" || op == "home" || op == "stop" {
					native = append(native, op)
				}
			}
			if len(native) < 2 || native[0] != operation || native[1] != "stop" {
				t.Fatalf("native call order: %v", native)
			}
			for _, op := range native[1:] {
				if op != "stop" {
					t.Fatalf("native motion after Stop admission: %v", native)
				}
			}
			t.Logf("asserted native call order: %v", native)
			return
		}
		var session string
		var generation uint64
		var previousIdentity string
		for i, id := range []any{json.Number("9007199254740993"), "9007199254740993"} {
			extra := credentials(i + 1)
			extra["command_id"] = id
			encodedID, _ := json.Marshal(id)
			var terminal nativeTerminal
			sendCold("position", "native-position", extra)
			eventually(t, "native position terminal", func() bool {
				return b.published("_INBOX.native.native-position", &terminal) && bytes.Equal(terminal.CommandID, encodedID) && terminal.State != "" && terminal.State != "accepted"
			})
			if terminal.State != "completed" || terminal.Native.Handle == 0 || terminal.Native.RunID != "native-run" {
				t.Fatalf("position terminal: %+v", terminal)
			}
			if terminal.Native.ExecutionGeneration != uint64(i+1) || terminal.Native.Identity.ProgramID == "" || terminal.Native.Identity.ProgramID == previousIdentity {
				t.Fatalf("raw trajectory identity echo: %+v", terminal)
			}
			previousIdentity = terminal.Native.Identity.ProgramID
			if i == 0 {
				session, generation = terminal.Native.Session, terminal.Native.Generation
			}
			if terminal.Native.Session != session || terminal.Native.Generation != generation {
				t.Fatal("one run replaced its grant")
			}
			eventually(t, "absolute joint count target", func() bool {
				return readPublished() && published.Status.Core.Axes[0].Counts >= 205 && published.Status.Core.Axes[0].Counts <= 213
			})
			b.mu.Lock()
			delete(b.messages, "_INBOX.native.native-position")
			b.mu.Unlock()
			sendCold("position", "native-position", extra)
			eventually(t, "typed native completion replay", func() bool {
				var replay nativeTerminal
				return b.published("_INBOX.native.native-position", &replay) && bytes.Equal(replay.CommandID, encodedID) &&
					replay.State == "completed" && replay.Native.Handle == terminal.Native.Handle && replay.Native.Session == session
			})
		}
		sendCold("go_home", "native-go-home", credentials(3))
		var home nativeTerminal
		eventually(t, "native GoHome completion", func() bool {
			return b.published("_INBOX.native.native-go-home", &home) && home.State != "" && home.State != "accepted"
		})
		if home.State != "completed" || home.Native.HomeEpoch == 0 || home.Native.Session == session {
			t.Fatalf("GoHome terminal: %+v", home)
		}
		if !readPublished() || published.Armed {
			t.Fatal("GoHome implicitly armed")
		}
		sendCold("go_home", "preempted-home", credentials(4))
		eventually(t, "committed native Home before Stop", func() bool { return readPublished() && published.Status.Core.HomeActive != 0 })
		sendCold("stop", "native-stop", nil)
		eventually(t, "Stop fence and grant release", func() bool {
			return readPublished() && published.StopConfirmed && published.Status.Core.HomeActive == 0 &&
				published.Status.Grant.Active == 0 && published.Status.Core.Armed == 0
		})
		eventually(t, "committed Home cancellation", func() bool {
			var terminal nativeTerminal
			return b.published("_INBOX.native.preempted-home", &terminal) && terminal.State == "cancelled" && terminal.Error == "rt_core_home_abandoned"
		})
		sendCold("arm", "arm-after-stop", map[string]any{"leader_lease_id": lease, "leader_fence_epoch": epoch, "armed": true})
		eventually(t, "rearm for retired run refusal", func() bool { return readPublished() && published.Armed })
		sendCold("position", "retired-run", credentials(5))
		var retired nativeTerminal
		eventually(t, "retired run refusal", func() bool {
			return b.published("_INBOX.native.retired-run", &retired) && retired.State == "failed"
		})
		if retired.Error != "rt_core_run_retired_or_busy" {
			t.Fatalf("retired run: %+v", retired)
		}
		return
	}
	ready, err := os.ReadFile(readyFile)
	if err != nil {
		t.Fatal(err)
	}
	udp, err := net.Dial("udp", strings.TrimSpace(string(ready)))
	if err != nil {
		t.Fatal(err)
	}
	defer udp.Close()
	sequence := uint32(0)
	sendJog := func() {
		sequence++
		sender := "steamdeck-test"
		packet := make([]byte, 70+len(sender)+9+len(lease))
		binary.BigEndian.PutUint16(packet, 0x4a49)
		packet[2] = 2
		binary.BigEndian.PutUint64(packet[3:], uint64(time.Now().UnixNano()))
		binary.BigEndian.PutUint32(packet[11:], sequence)
		packet[15] = 0
		packet[16] = byte(len(sender))
		binary.BigEndian.PutUint32(packet[17:], math.Float32bits(1))
		binary.BigEndian.PutUint32(packet[41:], math.Float32bits(1))
		packet[45] = 1
		copy(packet[70:], sender)
		offset := 70 + len(sender)
		binary.BigEndian.PutUint64(packet[offset:], epoch)
		packet[offset+8] = byte(len(lease))
		copy(packet[offset+9:], lease)
		if _, err := udp.Write(packet); err != nil {
			t.Fatal(err)
		}
	}
	before := append([]axisStatus(nil), published.Status.Core.Axes...)
	eventually(t, "Cartesian intent produced at least 20 joint counts", func() bool {
		sendJog()
		if !readPublished() {
			return false
		}
		if published.Refusal != "" {
			t.Log("cartesiand refusal:", published.Refusal)
		}
		for i, a := range published.Status.Core.Axes {
			if abs(a.Counts-before[i].Counts) >= 20 {
				return true
			}
		}
		return false
	})
	if published.Backend != "rt_core" || published.V4Available || published.Outputs < 1 {
		t.Fatal("invalid backend status", published)
	}
	var natsStatus publishedStatus
	eventually(t, "NATS carries public backend and logical axes", func() bool {
		return b.published("robot/v4/motion-server.jetson-test.status", &natsStatus) &&
			natsStatus.Backend == "rt_core" && len(natsStatus.Status.Core.Axes) == 6 && !natsStatus.V4Available
	})
	if err = request("POST", "/v1/control", map[string]any{"schema": "rosie.rt-control.request.v1", "operation": "stop", "session": oldFence.Session, "generation": oldFence.Generation}, nil); err != nil {
		t.Fatal(err)
	}
	eventually(t, "external Stop latched", func() bool { return readPublished() && !published.Armed && !published.Fresh && published.Refusal != "" })
	outputs := published.Outputs
	for i := 0; i < 3; i++ {
		sendJog()
	}
	eventually(t, "post-Stop input rejected", func() bool { return readPublished() && published.Rejected >= 1 && published.Sequence == sequence })
	if published.Outputs != outputs {
		t.Fatalf("new frames after Stop: %d -> %d", outputs, published.Outputs)
	}
	if err = request("GET", "/v1/status", nil, &status); err != nil {
		t.Fatal(err)
	}
	stopped := append([]axisStatus(nil), status.Core.Axes...)
	last := status.Core.TimeNS
	observations := 0
	eventually(t, "ten distinct stopped status observations", func() bool {
		if request("GET", "/v1/status", nil, &status) != nil || status.Core.TimeNS == last {
			return false
		}
		last = status.Core.TimeNS
		observations++
		if status.Core.Armed != 0 || status.Jog.Open != 0 {
			t.Fatal("Stop left motion authority")
		}
		for i, a := range status.Core.Axes {
			if abs(a.Counts-stopped[i].Counts) > 2 {
				t.Fatalf("axis %d moved after Stop: %d counts", i, a.Counts-stopped[i].Counts)
			}
		}
		return observations >= 10
	})
}
func abs(x int64) int64 {
	if x < 0 {
		return -x
	}
	return x
}
