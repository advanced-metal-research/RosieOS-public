//go:build linux

package cppclient

import (
	"bufio"
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"net"
	"net/http"
	"net/http/httputil"
	"net/url"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"rosieos/rt-core/internal/simfixture"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	adapter "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

var must = simfixture.Must

var poll = func(t *testing.T, predicate func() bool) {
	t.Helper()
	simfixture.PollTest(t, 5*time.Second, time.Millisecond, predicate)
}

type service struct {
	root, build, dir, socket, hash string
	client                         *control.Client
}

func process(t *testing.T, dir, name, binary string, args ...string) *exec.Cmd {
	t.Helper()
	return simfixture.SpawnLogged(t, exec.Command(binary, args...), filepath.Join(dir, name+".log"), 5*time.Second, 0, simfixture.CheckExit(t))
}

func startService(t *testing.T, machinePath ...string) *service {
	t.Helper()
	root, err := filepath.Abs("../..")
	must(t, err)
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "cpp-")
	must(t, err)
	t.Cleanup(func() { _ = os.RemoveAll(build) })
	dir, err := os.MkdirTemp("", "c-")
	must(t, err)
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	machine := filepath.Join(root, "config/machines/simulation/simulation-program.json")
	if len(machinePath) != 0 {
		machine = filepath.Join(root, machinePath[0])
	}
	compiled, args, err := simfixture.Compile(root, machine, filepath.Join(build, "config"))
	simfixture.Must(t, err)
	coreSocket := filepath.Join(dir, "ipc.sock")
	process(t, build, "core", simfixture.Locate(root, "rosie-rt-core-sim"), append(args, "--socket-path", coreSocket)...)
	simfixture.WaitVerified(t, filepath.Join(dir, "metrics.json"), 5*time.Second, time.Millisecond, func(s simfixture.Verified) bool { return s.ConfigVerifiedMask == 511 })
	socket := filepath.Join(dir, "control.sock")
	process(t, build, "control", simfixture.Locate(root, "rt-control"), "--core-socket", coreSocket, "--socket", socket,
		"--configuration-sha256", filepath.Base(compiled), "--backend", "simulation", "--pair-id", "bench", "--pair-revision", "7")
	client, err := control.Dial(control.UnixPath(socket))
	must(t, err)
	t.Cleanup(func() { _ = client.Close() })
	poll(t, func() bool {
		ctx, cancel := context.WithTimeout(context.Background(), 100*time.Millisecond)
		defer cancel()
		s, e := client.Status(ctx)
		return e == nil && s.Motion.TimeNS != 0
	})
	return &service{root, build, dir, socket, filepath.Base(compiled), client}
}

// TestCPPMotionServerIntegrationProfile builds the public-client example against
// the same homing machine and immutable plans as Go, preventing fixture drift.
func TestCPPMotionServerIntegrationProfile(t *testing.T) {
	s := startService(t, "clients/cpp/examples/testdata/profile-machine.json")
	binary := compileCPP(t, s.root, s.build, "clients/cpp/examples/motion_server_sequence.cpp", "motion-server-sequence")
	// Unverified process observation ceiling: 60000 ms; the example asserts receipts.
	ctx, cancel := context.WithTimeout(context.Background(), 60000*time.Millisecond)
	defer cancel()
	argvBytes, err := os.ReadFile(filepath.Join(s.build, "config", s.hash, "argv.json"))
	must(t, err)
	var argv []string
	must(t, json.Unmarshal(argvBytes, &argv))
	identities := map[string]string{}
	for i := 0; i+1 < len(argv); i++ {
		identities[argv[i]] = argv[i+1]
	}
	command := exec.CommandContext(ctx, binary, s.socket, s.hash, filepath.Join(s.root, "clients/cpp/examples/testdata"), identities["--machine-sha256"], identities["--deployment-sha256"])
	// Inject typed busy replies at the initial and next cursor through the real
	// HTTP parser; the complete example must still finish and capture live data.
	proxySocket := filepath.Join(s.dir, "busy.sock")
	listener, err := net.Listen("unix", proxySocket)
	must(t, err)
	target, err := url.Parse("http://control")
	must(t, err)
	proxy := httputil.NewSingleHostReverseProxy(target)
	transport := &http.Transport{DialContext: func(ctx context.Context, _, _ string) (net.Conn, error) {
		return (&net.Dialer{}).DialContext(ctx, "unix", s.socket)
	}}
	defer transport.CloseIdleConnections()
	proxy.Transport = transport
	var polls atomic.Int32
	var refusedCursor atomic.Value
	server := &http.Server{Handler: http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path == "/v1/telemetry" {
			poll := polls.Add(1)
			if poll == 1 || poll == 3 {
				refusedCursor.Store(r.URL.RawQuery)
				w.WriteHeader(http.StatusConflict)
				_ = json.NewEncoder(w).Encode(adapter.Response{Schema: adapter.ResponseSchema, Operation: adapter.OperationTelemetry, Error: adapter.ReasonTelemetryBusy})
				return
			}
			if (poll == 2 || poll == 4) && r.URL.RawQuery != refusedCursor.Load().(string) {
				t.Errorf("busy retry changed cursor: %s to %s", refusedCursor.Load(), r.URL.RawQuery)
			}
		}
		proxy.ServeHTTP(w, r)
	})}
	go server.Serve(listener)
	defer server.Close()
	command.Args[1] = proxySocket
	capturePath := filepath.Join(s.build, "telemetry.bin")
	command.Env = append(os.Environ(), "RTCORE_TELEMETRY_CAPTURE="+capturePath)
	command.Dir = s.build
	raw, err := command.CombinedOutput()
	if err != nil {
		t.Fatalf("motion-server sequence: %v\n%s", err, raw)
	}
	if !bytes.Contains(raw, []byte("sequence complete (polling and SSE verified)")) {
		t.Fatalf("missing conformance result: %s", raw)
	}
	if polls.Load() < 4 {
		t.Fatalf("busy retries observed only %d polls", polls.Load())
	}
	t.Logf("%s", raw)
	capture, err := os.ReadFile(capturePath)
	must(t, err)
	if len(capture) < 10784 || len(capture)%5392 != 0 {
		t.Fatalf("C++ telemetry capture bytes: %d", len(capture))
	}
	ring, err := ipcclient.OpenTelemetry(filepath.Join(s.dir, "telemetry.ring"))
	must(t, err)
	defer ring.Close()
	for offset := 0; offset < len(capture); offset += 5392 {
		record, err := ipcclient.DecodeCycleCaptureRecordV2(capture[offset : offset+5392])
		must(t, err)
		var observed ipcclient.TelemetryBatch
		must(t, simfixture.Poll(ctx, time.Millisecond, func() (bool, error) {
			var err error
			observed, err = ring.ReadFrom(record.Seq, 1)
			if errors.Is(err, ipcclient.ErrTelemetryBusy) {
				return false, nil
			}
			return observed.Drops != 0 || len(observed.Records) == 1, err
		}))
		if observed.Drops != 0 || len(observed.Records) != 1 || !reflect.DeepEqual(observed.Records[0], record) {
			t.Fatalf("C++ telemetry differs from live ring at %d", record.Seq)
		}
	}
}

var compileCPP = simfixture.CompileCPP

// Vendoring only the advertised include directory must compile telemetry; an
// in-tree include search path must not hide repository-relative dependencies.
func TestCPPVendoredHeadersSelfContained(t *testing.T) {
	root, err := filepath.Abs("../..")
	must(t, err)
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "vendored-client-")
	must(t, err)
	t.Cleanup(func() { os.RemoveAll(build) })
	include := filepath.Join(build, "include")
	must(t, os.MkdirAll(filepath.Join(include, "rosie"), 0755))
	headers, err := filepath.Glob(filepath.Join(root, "clients/cpp/include/rosie/*.hpp"))
	must(t, err)
	for _, header := range headers {
		raw, err := os.ReadFile(header)
		must(t, err)
		must(t, os.WriteFile(filepath.Join(include, "rosie", filepath.Base(header)), raw, 0644))
	}
	source := filepath.Join(build, "vendored.cpp")
	must(t, os.WriteFile(source, []byte("#include <rosie/rt_control_client.hpp>\nstatic_assert(sizeof(gradient::ipc::v1::TelemetryBatchHeaderV1) == 312);\nstatic_assert(sizeof(gradient::ipc::v1::CycleCaptureRecordV2) == 5392);\n"), 0644))
	command := exec.Command("g++", "-std=c++17", "-fsyntax-only", "-I"+include, source)
	if raw, err := command.CombinedOutput(); err != nil {
		t.Fatalf("vendored headers: %v\n%s", err, raw)
	}
}

// Regeneration is read-only and byte-exact, preventing stale machine or .rdt
// fixtures from making both consumer examples agree on obsolete producer data.
func TestCPPProfileFixturesCurrent(t *testing.T) {
	command := exec.Command("go", "run", "./clients/cpp/examples/testdata/generate.go", "-check")
	command.Dir = "../.."
	if raw, err := command.CombinedOutput(); err != nil {
		t.Fatalf("fixture regeneration: %v\n%s", err, raw)
	}
}

// referenceProgram uses the same frozen reference transformation as the Python
// oracle: zero-origin 0.05 scale and torch off, preserving eleven samples.
func referenceProgram(t *testing.T, root string) []byte {
	t.Helper()
	raw, err := os.ReadFile(filepath.Join(root, "adapters/rosie/densejointtraj/testdata/corpus/dense_joint_trajectory_reference.rdt"))
	must(t, err)
	plan, err := densejointtraj.Decode(raw)
	must(t, err)
	initial := plan.Segments[0].Samples[0].QRad
	for i := range plan.Segments {
		for j := range plan.Segments[i].Samples {
			s := &plan.Segments[i].Samples[j]
			for axis := range s.QRad {
				s.QRad[axis] = (s.QRad[axis] - initial[axis]) * .05
				s.QdRadS[axis] *= .05
			}
			s.TorchOn = false
		}
	}
	result, err := densejointtraj.Encode(plan.Header, plan.Segments)
	must(t, err)
	return result
}

func TestCPPClientLifecycle(t *testing.T) {
	s := startService(t)
	example := compileCPP(t, s.root, s.build, "clients/cpp/examples/rt_execute.cpp", "rt-execute")
	program := filepath.Join(s.build, "reference.rdt")
	must(t, os.WriteFile(program, referenceProgram(t, s.root), 0600))
	ctx, cancel := context.WithTimeout(context.Background(), 15*time.Second)
	defer cancel()
	cmd := exec.CommandContext(ctx, example, s.socket, "bench", "7", s.hash, program)
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	raw, err := cmd.Output()
	if err != nil {
		t.Fatalf("example: %v %s %s", err, raw, &stderr)
	}
	var state control.Status
	must(t, json.Unmarshal(raw, &state))
	if state.Execution.State != "completed" || state.Execution.Generation != 1 || state.Core.SafetyFaultMask != 0 || len(state.Core.Axes) != 9 {
		t.Fatalf("completion: %+v", state)
	}
	// Oracle endpoint is 20.8608 counts; feedback must be within two counts.
	if got := state.Core.Axes[0].PositionCounts; got < 19 || got > 23 {
		t.Fatalf("endpoint = %d counts, want 19..23", got)
	}
	poll(t, func() bool {
		v, err := s.client.Status(ctx)
		return err == nil && v.Core.Armed == 0 && v.Core.AxisEnableMask == 0
	})
	// A real binding rejection is distinguishable from a transport failure.
	cmd = exec.CommandContext(ctx, example, s.socket, "wrong-pair", "7", s.hash, program)
	raw, err = cmd.CombinedOutput()
	if exit, ok := err.(*exec.ExitError); !ok || exit.ExitCode() != 2 || !bytes.Contains(raw, []byte(adapter.ReasonAuthorityBindingMismatch)) {
		t.Fatalf("rejected exit: %v %s", err, raw)
	}
	cmd = exec.CommandContext(ctx, example, filepath.Join(s.dir, "missing.sock"), "bench", "7", s.hash, program)
	raw, err = cmd.CombinedOutput()
	if exit, ok := err.(*exec.ExitError); !ok || exit.ExitCode() != 3 {
		t.Fatalf("transport exit: %v %s", err, raw)
	}
}

type gateReader struct {
	io.Reader
	entered chan struct{}
	release <-chan struct{}
	ctx     context.Context
	once    sync.Once
}

func (g *gateReader) Read(p []byte) (int, error) {
	g.once.Do(func() { close(g.entered) })
	select {
	case <-g.release:
		return g.Reader.Read(p)
	case <-g.ctx.Done():
		return 0, g.ctx.Err()
	}
}

type proxy struct {
	path      string
	entered   chan struct{}
	release   chan struct{}
	once      sync.Once
	mu        sync.Mutex
	ids       []string
	sequences []uint64
	points    int
	bulkErr   error
}

func (p *proxy) unblock() { p.once.Do(func() { close(p.release) }) }

// The proxy gates after 65,536 body bytes and drops only the first Start receipt.
// All requests still execute against the real adapter; no outcome is fabricated.
func startProxy(t *testing.T, s *service, mode string) *proxy {
	t.Helper()
	p := &proxy{path: filepath.Join(s.dir, "proxy.sock"), entered: make(chan struct{}), release: make(chan struct{})}
	listener, err := net.Listen("unix", p.path)
	must(t, err)
	transport := control.NewUnixHTTPClient(s.socket).Transport.(*http.Transport)
	server := &http.Server{Handler: http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		var prefix []byte
		if r.Method == "POST" && r.URL.Path == "/v1/control" {
			// Unverified wire-progress barrier: 65,536 bytes, independent of upload throughput.
			size := min(r.ContentLength, 65536)
			if size < 0 {
				http.Error(w, "missing length", 400)
				return
			}
			prefix = make([]byte, size)
			if _, err := io.ReadFull(r.Body, prefix); err != nil {
				http.Error(w, err.Error(), 502)
				return
			}
		}
		bulk := mode == "bulk" && bytes.Contains(prefix, []byte(`"operation":"prepare_trajectory"`))
		start := mode == "retry" && bytes.Contains(prefix, []byte(`"operation":"start_trajectory"`))
		var captured bytes.Buffer
		var body io.Reader = io.MultiReader(bytes.NewReader(prefix), r.Body)
		if bulk {
			body = io.TeeReader(io.MultiReader(bytes.NewReader(prefix), &gateReader{Reader: r.Body, entered: p.entered, release: p.release, ctx: r.Context()}), &captured)
		}
		up := r.Clone(r.Context())
		up.RequestURI = ""
		up.URL.Scheme = "http"
		up.URL.Host = "localhost"
		up.Body = io.NopCloser(body)
		up.GetBody = nil
		response, err := transport.RoundTrip(up)
		if err != nil {
			http.Error(w, err.Error(), 502)
			return
		}
		defer response.Body.Close()
		raw, err := io.ReadAll(response.Body)
		if err != nil {
			http.Error(w, err.Error(), 502)
			return
		}
		if bulk {
			var request control.Request
			decodeErr := json.Unmarshal(captured.Bytes(), &request)
			p.mu.Lock()
			p.points, p.bulkErr = len(request.Points), decodeErr
			p.mu.Unlock()
		}
		if start {
			var request control.Request
			var result control.Response
			if json.Unmarshal(prefix, &request) != nil || request.RequestID == nil || json.Unmarshal(raw, &result) != nil {
				http.Error(w, "invalid Start receipt", 502)
				return
			}
			p.mu.Lock()
			p.ids = append(p.ids, *request.RequestID)
			p.sequences = append(p.sequences, result.Sequence)
			first := len(p.ids) == 1
			p.mu.Unlock()
			if first {
				conn, _, err := w.(http.Hijacker).Hijack()
				if err == nil {
					_ = conn.Close()
				}
				return
			}
		}
		for name, values := range response.Header {
			for _, value := range values {
				w.Header().Add(name, value)
			}
		}
		w.WriteHeader(response.StatusCode)
		_, _ = w.Write(raw)
	})}
	done := make(chan struct{})
	go func() { defer close(done); _ = server.Serve(listener) }()
	t.Cleanup(func() { p.unblock(); _ = server.Close(); <-done; transport.CloseIdleConnections() })
	return p
}

func TestCPPClientBulkAndRetry(t *testing.T) {
	root, err := filepath.Abs("../..")
	must(t, err)
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "cpp-driver-")
	must(t, err)
	t.Cleanup(func() { _ = os.RemoveAll(build) })
	driver := compileCPP(t, root, build, "tests/rt_control_client_tests.cpp", "driver")
	for _, mode := range []string{"bulk", "retry"} {
		t.Run(mode, func(t *testing.T) {
			s := startService(t)
			p := startProxy(t, s, mode)
			ctx, cancel := context.WithTimeout(context.Background(), 40000*time.Millisecond)
			defer cancel()
			cmd := exec.CommandContext(ctx, driver, "--"+mode, p.path, s.hash)
			var stderr bytes.Buffer
			cmd.Stderr = &stderr
			stdin, err := cmd.StdinPipe()
			must(t, err)
			stdout, err := cmd.StdoutPipe()
			must(t, err)
			must(t, cmd.Start())
			lines := make(chan string, 8)
			go func() {
				defer close(lines)
				scanner := bufio.NewScanner(stdout)
				for scanner.Scan() {
					lines <- scanner.Text()
				}
			}()
			if mode == "bulk" {
				select {
				case <-p.entered:
				case <-ctx.Done():
					_ = cmd.Wait()
					t.Fatalf("bulk wire progress: %v %s", ctx.Err(), &stderr)
				}
				_, err = io.WriteString(stdin, "s")
				must(t, err)
				select {
				case line := <-lines:
					parts := strings.Fields(line)
					if len(parts) != 2 || parts[0] != "stopped" {
						t.Errorf("Stop receipt: %q", line)
					} else {
						ns, err := strconv.ParseInt(parts[1], 10, 64)
						if err != nil || ns <= 0 || ns >= 1000000000 {
							t.Errorf("Stop receipt = %d ns, require < 1000000000 ns: %v", ns, err)
						}
						t.Logf("Stop acknowledged in %d ns while 250000-point body remained gated", ns)
					}
				case <-ctx.Done():
					t.Error("Stop not observed")
				}
				p.unblock()
			}
			_ = stdin.Close()
			var output []string
			for line := range lines {
				output = append(output, line)
			}
			if err := cmd.Wait(); err != nil {
				t.Fatalf("C++ %s: %v %s %v", mode, err, &stderr, output)
			}
			p.mu.Lock()
			defer p.mu.Unlock()
			if mode == "bulk" {
				if p.bulkErr != nil || p.points != 250000 || len(output) != 1 || output[0] != "upload control_session_stale" {
					t.Fatalf("bulk: %d points, %v, %v", p.points, p.bulkErr, output)
				}
			} else {
				if len(p.ids) != 3 || len(p.ids[0]) != 32 || p.ids[0] != p.ids[1] || p.ids[1] != p.ids[2] || p.sequences[0] == 0 || p.sequences[0] != p.sequences[1] || p.sequences[1] != p.sequences[2] {
					t.Fatalf("retry IDs/sequences: %v %v", p.ids, p.sequences)
				}
				if len(output) != 1 || !strings.HasPrefix(output[0], "generation 1 sequence ") {
					t.Fatalf("execution receipt: %v", output)
				}
			}
		})
	}
}
