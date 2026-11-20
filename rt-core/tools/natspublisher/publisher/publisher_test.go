//go:build linux

package publisher

import (
	"bufio"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/sdk/control"
)

// Unverified observation ceiling: 8000 ms with 5 ms predicate polling; elapsed
// time can only fail the test, never establish a positive observation.
const observation, pollCadence = 8 * time.Second, 5 * time.Millisecond

var must = simfixture.Must

type broker struct {
	listener                              net.Listener
	mu                                    sync.Mutex
	peers                                 map[net.Conn]*brokerPeer
	wg                                    sync.WaitGroup
	closed                                bool
	config                                *streamConfig
	telemetryConfig                       *streamConfig
	telemetryBatches                      [][]byte
	last                                  map[string][]byte
	sequence, creates, updates, publishes uint64
	stall                                 atomic.Bool
	stalled                               atomic.Uint64
	info                                  string
	noResponders                          bool
	rejectUpdate                          bool
	connections                           uint64
	checks                                uint64
	publications                          map[uint64]uint64
}

type brokerPeer struct {
	conn net.Conn
	mu   sync.Mutex
	subs map[string]string // guarded by broker.mu
	id   uint64
}

func (p *brokerPeer) write(raw string) bool {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.conn.SetWriteDeadline(time.Now().Add(observation))
	_, err := io.WriteString(p.conn, raw)
	return err == nil
}

// newBroker models wire-level core NATS plus the JetStream API/ack subset. It
// records configuration and messages independently, preventing a fake transport
// from concealing missing stream provisioning, acknowledgement or subject framing.
func newBroker(t *testing.T, configure ...func(*broker)) *broker {
	t.Helper()
	ln, err := net.Listen("tcp", "127.0.0.1:0")
	must(t, err)
	b := &broker{listener: ln, peers: map[net.Conn]*brokerPeer{}, last: map[string][]byte{}, info: `{"jetstream":true,"max_payload":1048576}`, publications: map[uint64]uint64{}}
	for _, option := range configure {
		option(b)
	}
	b.wg.Add(1)
	go func() {
		defer b.wg.Done()
		for {
			c, err := ln.Accept()
			if err != nil {
				return
			}
			b.mu.Lock()
			if b.closed {
				b.mu.Unlock()
				c.Close()
				return
			}
			peer := &brokerPeer{conn: c, subs: map[string]string{}}
			b.peers[c] = peer
			b.connections++
			peer.id = b.connections
			b.wg.Add(1)
			b.mu.Unlock()
			go func() {
				defer b.wg.Done()
				defer c.Close()
				defer func() { b.mu.Lock(); delete(b.peers, c); b.mu.Unlock() }()
				b.serve(peer)
			}()
		}
	}()
	t.Cleanup(func() {
		b.mu.Lock()
		b.closed = true
		ln.Close()
		for c := range b.peers {
			c.Close()
		}
		b.mu.Unlock()
		b.wg.Wait()
	})
	return b
}
func (b *broker) url() string { return "nats://" + b.listener.Addr().String() }
func (b *broker) serve(peer *brokerPeer) {
	c, write := peer.conn, peer.write
	if !write("INFO " + b.info + "\r\n") {
		return
	}
	r := bufio.NewReader(c)
	inbox, sid := "", ""
	var negotiated struct {
		Headers      bool `json:"headers"`
		NoResponders bool `json:"no_responders"`
	}
	for {
		line, err := r.ReadString('\n')
		if err != nil {
			return
		}
		f := strings.Fields(line)
		if len(f) == 0 {
			return
		}
		switch f[0] {
		case "CONNECT":
			if json.Unmarshal([]byte(strings.TrimPrefix(line, "CONNECT ")), &negotiated) != nil {
				return
			}
		case "PING":
			if !write("PONG\r\n") {
				return
			}
		case "PONG":
		case "SUB":
			if len(f) != 3 {
				return
			}
			b.mu.Lock()
			peer.subs[f[2]] = f[1]
			if f[2] == "2" {
				b.checks++
			}
			b.mu.Unlock()
			if f[2] == "1" {
				inbox, sid = f[1], f[2]
			}
		case "UNSUB":
			if len(f) != 2 {
				return
			}
			b.mu.Lock()
			delete(peer.subs, f[1])
			b.mu.Unlock()
		case "PUB":
			if len(f) != 4 || f[2] != inbox {
				return
			}
			size, err := strconv.Atoi(f[3])
			if err != nil || size < 0 || size > 1048576 {
				return
			}
			raw := make([]byte, size+2)
			if _, err := io.ReadFull(r, raw); err != nil || string(raw[size:]) != "\r\n" {
				return
			}
			if b.noResponders {
				// Real NATS sends 503 only to clients that negotiated headers and
				// no-responder support; missing CONNECT flags must not pass this test.
				if !negotiated.Headers || !negotiated.NoResponders {
					continue
				}
				header := "NATS/1.0 503 No Responders\r\n\r\n"
				if !write(fmt.Sprintf("HMSG %s %s %d %d\r\n%s\r\n", inbox, sid, len(header), len(header), header)) {
					return
				}
				continue
			}
			body, ack := b.handle(f[1], raw[:size])
			if strings.HasPrefix(f[1], "robot/v4/rtcore.") {
				b.mu.Lock()
				b.publications[peer.id]++
				b.mu.Unlock()
			}
			b.broadcast(f[1], raw[:size])
			if ack && !write(fmt.Sprintf("MSG %s %s %d\r\n%s\r\n", inbox, sid, len(body), body)) {
				return
			}
		default:
			return
		}
	}
}

func (b *broker) broadcast(subject string, raw []byte) {
	b.mu.Lock()
	type delivery struct {
		peer *brokerPeer
		sid  string
	}
	var pending []delivery
	for _, peer := range b.peers {
		for sid, sub := range peer.subs {
			if sub == subject {
				pending = append(pending, delivery{peer, sid})
			}
		}
	}
	b.mu.Unlock()
	for _, d := range pending {
		d.peer.write(fmt.Sprintf("MSG %s %s %d\r\n%s\r\n", subject, d.sid, len(raw), raw))
	}
}
func (b *broker) handle(subject string, raw []byte) ([]byte, bool) {
	b.mu.Lock()
	defer b.mu.Unlock()
	var response any
	switch subject {
	case "$JS.API.STREAM.INFO.ROSIE_RT_CORE_TELEMETRY":
		if b.telemetryConfig == nil {
			response = map[string]any{"error": map[string]any{"code": 404}}
		} else {
			response = map[string]any{"config": b.telemetryConfig}
		}
	case "$JS.API.STREAM.CREATE.ROSIE_RT_CORE_TELEMETRY", "$JS.API.STREAM.UPDATE.ROSIE_RT_CORE_TELEMETRY":
		var config streamConfig
		if json.Unmarshal(raw, &config) != nil {
			return nil, false
		}
		if config.Name != "ROSIE_RT_CORE_TELEMETRY" || strings.Contains(subject, "UPDATE") && (b.telemetryConfig == nil || b.rejectUpdate || config.Storage != b.telemetryConfig.Storage || config.Retention != b.telemetryConfig.Retention) {
			return []byte(`{"error":{"code":400}}`), true
		}
		b.telemetryConfig = &config
		response = map[string]any{"config": config}
	case "$JS.API.STREAM.INFO.ROBOT_V4_RTCORE":
		if b.config == nil {
			response = map[string]any{"error": map[string]any{"code": 404}}
		} else {
			response = map[string]any{"config": b.config}
		}
	case "$JS.API.STREAM.CREATE.ROBOT_V4_RTCORE", "$JS.API.STREAM.UPDATE.ROBOT_V4_RTCORE":
		var config streamConfig
		if json.Unmarshal(raw, &config) != nil {
			return nil, false
		}
		// Conservative JetStream contract: storage and retention are not repairable;
		// the stream API name must match. Subjects and limits can be changed.
		if config.Name != "ROBOT_V4_RTCORE" || strings.Contains(subject, "UPDATE") && (b.config == nil || b.rejectUpdate || config.Name != b.config.Name || config.Storage != b.config.Storage || config.Retention != b.config.Retention) {
			return []byte(`{"error":{"code":400,"description":"stream update refused"}}`), true
		}
		b.config = &config
		if strings.Contains(subject, "CREATE") {
			b.creates++
		} else {
			b.updates++
		}
		response = map[string]any{"config": config}
	default:
		if strings.HasPrefix(subject, "rosie/rt-core.") && strings.HasSuffix(subject, ".telemetry.v1") {
			if b.telemetryConfig == nil {
				return []byte(`{"error":{"code":503}}`), true
			}
			b.last[subject] = append([]byte(nil), raw...)
			b.telemetryBatches = append(b.telemetryBatches, append([]byte(nil), raw...))
			b.sequence++
			body, _ := json.Marshal(map[string]any{"stream": "ROSIE_RT_CORE_TELEMETRY", "seq": b.sequence})
			return body, true
		}
		b.publishes++
		if !strings.HasPrefix(subject, "robot/v4/rtcore.") || b.config == nil {
			return []byte(`{"error":{"code":503}}`), true
		}
		b.last[subject] = append([]byte(nil), raw...)
		b.sequence++
		response = map[string]any{"stream": "ROBOT_V4_RTCORE", "seq": b.sequence}
		if b.stall.Load() {
			b.stalled.Add(1)
			return nil, false
		}
	}
	body, _ := json.Marshal(response)
	return body, true
}
func (b *broker) latest(subject string) []byte {
	b.mu.Lock()
	defer b.mu.Unlock()
	return append([]byte(nil), b.last[subject]...)
}
func poll(t *testing.T, fn func() bool) {
	t.Helper()
	simfixture.PollTest(t, observation, pollCadence, fn)
}

func service(t *testing.T) (*control.Client, string, string) {
	t.Helper()
	root, err := filepath.Abs("../../..")
	must(t, err)
	dir, err := os.MkdirTemp("", "np-")
	must(t, err)
	t.Cleanup(func() { os.RemoveAll(dir) })
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "natspublisher-")
	must(t, err)
	t.Cleanup(func() { os.RemoveAll(build) })
	compiled, args, err := simfixture.Compile(root, filepath.Join(root, "config/machines/simulation/simulation-program.json"), filepath.Join(build, "config"))
	must(t, err)
	socket := filepath.Join(dir, "core.sock")
	simfixture.SpawnLogged(t, exec.Command(simfixture.Locate(root, "rosie-rt-core-sim"), append(args, "--socket-path", socket)...), filepath.Join(build, "core.log"), observation, 4000, simfixture.CheckExit(t))
	simfixture.WaitVerified(t, filepath.Join(dir, "metrics.json"), observation, pollCadence, func(s simfixture.Verified) bool { return s.ConfigVerifiedMask == 511 })
	public := filepath.Join(dir, "control.sock")
	simfixture.SpawnLogged(t, simfixture.StartControl(root, socket, public, filepath.Base(compiled), "bench", 7), filepath.Join(build, "control.log"), observation, 4000, simfixture.CheckExit(t))
	c, err := control.Dial(control.UnixPath(public))
	must(t, err)
	t.Cleanup(func() { c.Close() })
	poll(t, func() bool {
		ctx, cancel := context.WithTimeout(context.Background(), 100*time.Millisecond)
		defer cancel()
		s, e := c.Status(ctx)
		return e == nil && s.Core.TimeNS > 0 && len(s.Core.Axes) == 9
	})
	return c, root, dir
}

func startPublisher(t *testing.T, c Observer, b *broker) *Publisher {
	t.Helper()
	p, err := New(c, Config{URL: b.url(), Host: "other-relay"})
	must(t, err)
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- p.Run(ctx) }()
	t.Cleanup(func() {
		cancel()
		select {
		case err := <-done:
			must(t, err)
		case <-time.After(observation):
			t.Error("publisher did not join within 8000 ms")
		}
	})
	return p
}

// The typed projection independently reproduces the Go OLP reader's scalar,
// pointer and nested-readback decoding contract; the map projection reproduces
// system_state.go:578. Neither depends on the publisher's encoding types.
func checkReaders(t *testing.T, raw []byte) {
	t.Helper()
	var reader struct {
		Schema, Subject, Host, Component string
		PublishedAt                      string `json:"published_at"`
		Status                           struct {
			Name, State    string
			UpdatedAt      string `json:"updated_at"`
			Mode           string `json:"rtcore_mode"`
			Simulation     *bool  `json:"simulation_mode"`
			Live           *bool  `json:"ethercat_live"`
			EtherCATOutput *bool  `json:"ethercat_output_enabled"`
			MotionOutput   *bool  `json:"motion_output_enabled"`
			PDOFresh       *bool  `json:"pdo_fresh"`
			Armed          *bool  `json:"servos_armed_requested"`
			AxisCount      int    `json:"configured_axis_count"`
			Enabled        int    `json:"ds402_enabled_axes"`
			EnableMask     uint32 `json:"axis_enable_mask"`
			ErrorMask      uint32 `json:"axis_error_mask"`
			FaultMask      uint32 `json:"axis_fault_mask"`
			Readback       struct {
				Schema      string
				OutputCount int    `json:"output_count"`
				ValuesMask  uint16 `json:"values_mask"`
				ValidMask   uint16 `json:"valid_mask"`
				Fresh       bool
			} `json:"revpi_output_readback"`
		}
	}
	must(t, json.Unmarshal(raw, &reader))
	if reader.Schema != "robot.v4.rtcore.status.v1" || reader.Subject != "robot/v4/rtcore.other-relay.status" || reader.Host != "other-relay" || reader.Component != "rtcore" || reader.Status.Name != "robot-v4-rtcore" || reader.Status.State != "running" {
		t.Fatalf("reader identity: %+v", reader)
	}
	if reader.Status.Simulation == nil || !*reader.Status.Simulation || reader.Status.Mode != "sim" || reader.Status.AxisCount != 9 || reader.Status.Armed == nil || *reader.Status.Armed || reader.Status.Enabled != 0 || reader.Status.EnableMask != 0 || reader.Status.EtherCATOutput != nil || reader.Status.MotionOutput != nil {
		t.Fatalf("reader observations: %+v", reader.Status)
	}
	if reader.Status.Readback.Fresh || reader.Status.Readback.ValidMask != 0 {
		t.Fatal("unavailable readback became valid")
	}
	for _, stamp := range []string{reader.PublishedAt, reader.Status.UpdatedAt} {
		parsed, err := time.Parse(time.RFC3339Nano, stamp)
		must(t, err)
		if time.Since(parsed) > 5*time.Second {
			t.Fatal("reader timestamp exceeds 5000 ms freshness")
		}
	}
	var envelope map[string]any
	must(t, json.Unmarshal(raw, &envelope))
	status, ok := envelope["status"].(map[string]any)
	if !ok || len(status) == 0 {
		t.Fatal("system-state decoder has no status")
	}
	for _, field := range []string{"tour_start_alarm", "virtual_simulation_indicator", "axis_target_counts", "axis_current_ma", "motion_output_enabled", "realtime_core_command_log_complete", "pair_config_drive_ratio_converged"} {
		if value, exists := status[field]; !exists || value != nil {
			t.Fatalf("%s is not explicit null: %v", field, value)
		}
	}
	if status["tour_start_alarm_valid"] != false || status["virtual_simulation_indicator_valid"] != false {
		t.Fatal("absent groups are valid")
	}
	readback := status["revpi_output_readback"].(map[string]any)
	if readback["values_mask"] != nil || readback["sampled_at"] != nil || readback["fresh"] != false || readback["valid_mask"] != float64(0) {
		t.Fatalf("readback: %v", readback)
	}
}

func assertStream(t *testing.T, b *broker) {
	t.Helper()
	b.mu.Lock()
	defer b.mu.Unlock()
	// Measured v4 retention requirements; literals are independent of desiredStream.
	want := streamConfig{"ROBOT_V4_RTCORE", []string{"robot/v4/rtcore.>"}, "limits", "file", "old", 250000, 268435456, 600000000000}
	if b.config == nil || !reflect.DeepEqual(*b.config, want) {
		t.Fatalf("stream configuration: %+v", b.config)
	}
}

func TestComposedStatusBatchStallAndNoGrant(t *testing.T) {
	c, _, _ := service(t)
	b := newBroker(t)
	p := startPublisher(t, c, b)
	poll(t, func() bool {
		return len(b.latest("robot/v4/rtcore.other-relay.status")) > 0 && len(b.latest("robot/v4/rtcore.other-relay.telemetry.batch")) > 0
	})
	checkReaders(t, b.latest("robot/v4/rtcore.other-relay.status"))
	assertStream(t, b)
	var batch struct {
		Schema, Stream, Subject, Host, Component string
		PublishedAt                              string  `json:"published_at"`
		SourceMonotonicMS                        float64 `json:"source_monotonic_ms"`
		Sequence                                 uint64
		IntervalMS                               int `json:"interval_ms"`
		Count                                    int
		Dropped                                  uint64
		Records                                  []map[string]any
		RecentCommands                           []map[string]any `json:"recent_commands"`
	}
	must(t, json.Unmarshal(b.latest("robot/v4/rtcore.other-relay.telemetry.batch"), &batch))
	if batch.Schema != "robot.v4.rtcore.telemetry.batch.v1" || batch.IntervalMS != 500 || batch.Count != 1 || len(batch.Records) != 1 || batch.SourceMonotonicMS <= 0 || batch.Sequence == 0 || batch.Records[0]["event"] != "timeseries" {
		t.Fatalf("batch decoder: %+v", batch)
	}
	beforeStall := p.Dropped()
	b.stall.Store(true)
	// Unverified test bound: require 2 new drops after the broker withholds an ack;
	// any startup drops cannot satisfy this assertion.
	poll(t, func() bool { return b.stalled.Load() >= 1 && p.Dropped()-beforeStall >= 2 })
	b.stall.Store(false)
	poll(t, func() bool {
		var value struct{ Dropped uint64 }
		return json.Unmarshal(b.latest("robot/v4/rtcore.other-relay.telemetry.batch"), &value) == nil && value.Dropped >= beforeStall && value.Dropped-beforeStall >= 2
	})
	poll(t, func() bool {
		var value struct{ Dropped uint64 }
		return json.Unmarshal(b.latest("robot/v4/rtcore.other-relay.status"), &value) == nil && value.Dropped >= beforeStall && value.Dropped-beforeStall >= 2
	})
	ctx, cancel := context.WithTimeout(context.Background(), observation)
	defer cancel()
	s, err := c.Status(ctx)
	must(t, err)
	if s.Grant == nil || s.Grant.GrantActive != 0 || s.Grant.AppGrantGeneration != 0 || s.Grant.CapabilityId != [16]byte{} || s.Core.Armed != 0 || s.Core.AxisEnableMask != 0 {
		t.Fatalf("publisher acquired authority: %+v", s.Grant)
	}
	b.mu.Lock()
	defer b.mu.Unlock()
	if b.creates != 1 || b.updates != 0 || len(b.last) != 3 {
		t.Fatalf("unexpected stream/publications: %d %d %v", b.creates, b.updates, b.last)
	}
}

func TestRepairStreamPreservesArchive(t *testing.T) {
	b := newBroker(t)
	// Unverified fixture limits: one message, 1048576 bytes, 600000000000 ns.
	// Every field is valid for real JetStream; only mutable settings need repair.
	b.config = &streamConfig{Name: "ROBOT_V4_RTCORE", Storage: "file", Retention: "limits", Subjects: []string{"wrong.>"}, Discard: "new", MaxMsgs: 1, MaxBytes: 1048576, MaxAge: 600000000000}
	b.sequence = 41
	b.last["robot/v4/rtcore.other-relay.telemetry.pattern"] = []byte(`{"archived":true}`)
	ctx, cancel := context.WithTimeout(context.Background(), observation)
	defer cancel()
	n, err := dialNATS(ctx, b.url())
	must(t, err)
	defer n.close()
	must(t, n.ensureStream())
	assertStream(t, b)
	b.mu.Lock()
	defer b.mu.Unlock()
	if b.updates != 1 || b.creates != 0 || b.sequence != 41 || string(b.last["robot/v4/rtcore.other-relay.telemetry.pattern"]) != `{"archived":true}` {
		t.Fatal("stream repair replaced archive")
	}
}

type changedObserver struct {
	Observer
	bad atomic.Bool
}

func (o *changedObserver) Describe(ctx context.Context) (control.Description, error) {
	d, err := o.Observer.Describe(ctx)
	if o.bad.Load() {
		d.CapabilitiesDigest = "different"
	}
	return d, err
}

func TestDigestMismatchStopsAndCLIExitsNonzero(t *testing.T) {
	c, root, dir := service(t)
	b := newBroker(t)
	o := &changedObserver{Observer: c}
	p, err := New(o, Config{URL: b.url(), Host: "other-relay"})
	must(t, err)
	ctx, cancel := context.WithTimeout(context.Background(), observation)
	defer cancel()
	done := make(chan error, 1)
	go func() { done <- p.Run(ctx) }()
	t.Cleanup(func() { cancel() })
	poll(t, func() bool { return len(b.latest("robot/v4/rtcore.other-relay.telemetry.batch")) > 0 })
	o.bad.Store(true)
	select {
	case err := <-done:
		if !errors.Is(err, ErrContract) {
			t.Fatalf("digest change: %v", err)
		}
	case <-ctx.Done():
		t.Fatal("digest change did not exit")
	}
	// The executable receives an incompatible describe via a real public HTTP socket.
	ln, err := net.Listen("unix", filepath.Join(dir, "bad.sock"))
	must(t, err)
	server := &http.Server{Handler: http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.Method != "GET" || r.URL.Path != "/v1/describe" {
			t.Errorf("unexpected observer request %s %s", r.Method, r.URL.Path)
			w.WriteHeader(400)
			return
		}
		d, _ := o.Describe(r.Context())
		json.NewEncoder(w).Encode(map[string]any{"schema": "rosie.rt-control.response.v1", "operation": "describe", "data": d})
	})}
	go server.Serve(ln)
	t.Cleanup(func() { server.Close() })
	raw, err := exec.CommandContext(ctx, filepath.Join(root, "build/rt-natspublisher"), "--socket", ln.Addr().String(), "--host", "other-relay", "--nats-url", b.url()).CombinedOutput()
	var exited *exec.ExitError
	if !errors.As(err, &exited) || exited.ExitCode() != 10 || !strings.Contains(string(raw), "generated contract") {
		t.Fatalf("CLI mismatch exit: %v %s", err, raw)
	}
}

func TestHostTokenAndConfiguration(t *testing.T) {
	for input, want := range map[string]string{"other-relay": "other-relay", "  Bench.4.local  ": "Bench-4-local", "__a..b__": "a--b", " !!! ": "unknown"} {
		if got := TopicToken(input); got != want {
			t.Fatalf("token %q: %q != %q", input, got, want)
		}
	}
	for _, c := range []Config{{URL: "nats://localhost:4222", Host: "realtime-core"}, {URL: "nats://user:secret@localhost:4222", Host: "other-relay"}, {URL: "tls://localhost:4222", Host: "other-relay"}, {URL: "nats://localhost:4222", Host: "bad\nHOST=other"}, {URL: "nats://localhost:4222"}} {
		if c.Validate() == nil {
			t.Fatalf("accepted invalid deployment %+v", c)
		}
	}
}

func TestRejectStaleSource(t *testing.T) {
	if _, err := sourceAge(1); err == nil {
		t.Fatal("stale monotonic observation accepted")
	}
	if _, err := sourceAge(^uint64(0)); err == nil {
		t.Fatal("future monotonic observation accepted")
	}
}

func TestRejectMalformedBrokerReplies(t *testing.T) {
	validAck := `{"stream":"ROBOT_V4_RTCORE","seq":1}`
	for _, tc := range []struct{ name, reply string }{
		{"negative-size", "MSG _INBOX.test 1 -1\r\n"},
		{"oversized", "MSG _INBOX.test 1 1048577\r\n"},
		{"oversized-line", strings.Repeat("X", 8192) + "\r\n"},
		{"wrong-inbox", "MSG _INBOX.other 1 2\r\n{}\r\n"},
		{"bad-terminator", "MSG _INBOX.test 1 2\r\n{}XX"},
		{"empty-ack", "MSG _INBOX.test 1 2\r\n{}\r\n"},
		{"negative-header-size", "HMSG _INBOX.test 1 -1 2\r\n{}\r\n"},
		{"header-past-body", "HMSG _INBOX.test 1 13 2\r\n{}\r\n"},
		{"invalid-header-version", fmt.Sprintf("HMSG _INBOX.test 1 13 %d\r\nNATS/1.0X\r\n\r\n%s\r\n", 13+len(validAck), validAck)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			client, server := net.Pipe()
			n := &natsConn{conn: client, reader: bufio.NewReaderSize(client, 8192), inbox: "_INBOX.test", maxPayload: 1048576, stop: func() bool { return true }}
			defer n.close()
			done := make(chan struct{})
			go func() {
				defer close(done)
				defer server.Close()
				r := bufio.NewReader(server)
				if _, err := r.ReadString('\n'); err != nil {
					return
				}
				body := make([]byte, 4)
				if _, err := io.ReadFull(r, body); err != nil {
					return
				}
				io.WriteString(server, tc.reply)
			}()
			if err := n.publish("robot/v4/rtcore.other-relay.status", []byte("{}")); err == nil {
				t.Fatal("invalid broker response accepted")
			}
			client.Close()
			<-done
		})
	}
}
