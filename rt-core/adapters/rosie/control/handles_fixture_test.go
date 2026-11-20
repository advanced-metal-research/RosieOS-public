//go:build linux

package control

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"net"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/controltest"
	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/internal/unixhttp"
	"sync"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

type handleAPI struct {
	client     *http.Client
	socket     string
	controller *Controller
}

// serveHandleAPI uses the public handler on a real Unix socket; both concurrent
// callers and disconnected callers traverse the production JSON dispatch.
func serveHandleAPI(t *testing.T, c *Controller, wrap func(http.Handler) http.Handler) *handleAPI {
	t.Helper()
	dir, err := os.MkdirTemp("", "h-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	socket := filepath.Join(dir, "h")
	listener, err := net.Listen("unix", socket)
	if err != nil {
		t.Fatal(err)
	}
	var handler http.Handler = NewHandler(c)
	if wrap != nil {
		handler = wrap(handler)
	}
	server := &http.Server{Handler: handler}
	served := make(chan struct{})
	go func() { defer close(served); _ = server.Serve(listener) }()
	transport := unixhttp.NewClient(socket).Transport.(*http.Transport)
	t.Cleanup(func() { transport.CloseIdleConnections(); _ = server.Close(); <-served })
	return &handleAPI{client: &http.Client{Transport: transport, Timeout: 3 * time.Second}, socket: socket, controller: c}
}

func controlRequest(f Fence, operation string) Request {
	return Request{Schema: "rosie.rt-control.request.v1", Operation: operation, Fence: f}
}

func (a *handleAPI) do(ctx context.Context, request Request) (Response, []byte, error) {
	var result Response
	raw, _, err := (controltest.Client{HTTP: a.client, Base: "http://unix"}).JSON(ctx, http.MethodPost, "/v1/control", request, &result)
	return result, raw, err
}

func (a *handleAPI) call(t *testing.T, request Request, reason string) Response {
	t.Helper()
	result, _, err := a.do(context.Background(), request)
	if err != nil || result.Error != reason {
		t.Fatalf("%s: response=%+v transport=%v; want %q", request.Operation, result, err, reason)
	}
	return result
}

func grantFrom(t *testing.T, response Response) Grant {
	t.Helper()
	raw, err := json.Marshal(response.Data)
	if err != nil {
		t.Fatal(err)
	}
	var grant Grant
	if err := json.Unmarshal(raw, &grant); err != nil {
		t.Fatal(err)
	}
	return grant
}

func (a *handleAPI) snapshot(t *testing.T) (ipcclient.Status, ipcclient.MotionState, Execution) {
	t.Helper()
	r, err := a.client.Get("http://unix/v1/status")
	if err != nil {
		t.Fatal(err)
	}
	defer r.Body.Close()
	var response struct {
		Error string `json:"error"`
		Data  struct {
			Core      ipcclient.Status      `json:"core"`
			Motion    ipcclient.MotionState `json:"motion"`
			Execution Execution             `json:"execution"`
		} `json:"data"`
	}
	if err := json.NewDecoder(r.Body).Decode(&response); err != nil {
		t.Fatal(err)
	}
	if response.Error != "" {
		t.Fatal(response.Error)
	}
	return response.Data.Core, response.Data.Motion, response.Data.Execution
}

func requireHandle(t *testing.T, a *handleAPI, id uint64, state HandleState) HandleRecord {
	t.Helper()
	_, _, execution := a.snapshot(t)
	for _, handle := range execution.Handles {
		if handle.Handle == id {
			if handle.State != state {
				t.Fatalf("handle %d: %s, want %s", id, handle.State, state)
			}
			return handle
		}
	}
	t.Fatalf("handle %d absent from %+v", id, execution)
	return HandleRecord{}
}

// awaitHandleCondition bounds failed progress; only the observed predicate can
// pass, so scheduler delay never counts as evidence of successful execution.
func awaitHandleCondition(t *testing.T, condition func() bool) {
	t.Helper()
	simfixture.PollTest(t, 3*time.Second, time.Millisecond, condition)
}

func keepHandleGrant(t *testing.T, a *handleAPI, f Fence) func() {
	t.Helper()
	ctx, cancel := context.WithCancel(context.Background())
	joined := simfixture.Renew(ctx, 25*time.Millisecond, func() bool {
		response, _, err := a.do(ctx, controlRequest(f, "renew"))
		if ctx.Err() != nil {
			return false
		}
		if err != nil || response.Error != "" {
			t.Errorf("keepalive: %+v %v", response, err)
			return false
		}
		return true
	})
	var once sync.Once
	stop := func() { once.Do(func() { cancel(); joined() }) }
	t.Cleanup(stop)
	return stop
}

func handlePoints(duration time.Duration) []ipcclient.Point {
	return []ipcclient.Point{{Time: 0, Position: make([]float64, 9)}, {Time: duration, Position: make([]float64, 9)}}
}

func prepareHandle(t *testing.T, a *handleAPI, f Fence, duration time.Duration) uint64 {
	t.Helper()
	request := controlRequest(f, "prepare_trajectory")
	request.AxisMask, request.Points = 511, handlePoints(duration)
	return a.call(t, request, "").Handle
}

func prepareHandleProgram(t *testing.T, a *handleAPI, f Fence, name string) uint64 {
	t.Helper()
	r, err := http.NewRequest(http.MethodPost, "http://unix/v1/program", bytes.NewReader(stationaryProgram(t, name)))
	if err != nil {
		t.Fatal(err)
	}
	r.Header.Set("X-Control-Session", f.Session)
	r.Header.Set("X-Control-Generation", fmt.Sprint(f.Generation))
	response, err := a.client.Do(r)
	if err != nil {
		t.Fatal(err)
	}
	defer response.Body.Close()
	var result Response
	if err := json.NewDecoder(response.Body).Decode(&result); err != nil {
		t.Fatal(err)
	}
	if result.Error != "" {
		t.Fatal(result.Error)
	}
	_, _, execution := a.snapshot(t)
	if len(execution.Handles) == 0 || execution.Identity.PlanID != name {
		t.Fatalf("missing prepared program: %+v", execution)
	}
	handle := execution.Handles[len(execution.Handles)-1]
	if handle.State != HandlePrepared {
		t.Fatalf("program handle: %+v", handle)
	}
	return handle.Handle
}

func enableHandles(t *testing.T, a *handleAPI, f Fence) {
	t.Helper()
	request := controlRequest(f, "enable")
	request.AxisMask = 511
	a.call(t, request, "")
	a.call(t, controlRequest(f, "arm"), "")
	awaitHandleCondition(t, func() bool {
		state, _, _ := a.snapshot(t)
		if state.Armed != 1 || state.AxisEnableMask != 511 {
			return false
		}
		return ipcclient.AllOperationEnabled(state) && len(state.Axes) == 9
	})
}

// newHandleFixture starts the real daemon without automatic renewal or Enable;
// tests control those events explicitly so expiry and rejected Start are visible.
func newHandleFixture(t *testing.T) lifecycleFixture {
	t.Helper()
	root, err := filepath.Abs("../../..")
	if err != nil {
		t.Fatal(err)
	}
	binaryPath := simfixture.Locate(root, "rosie-rt-core-sim")
	if _, err := os.Stat(binaryPath); err != nil {
		t.Fatal("run make sim first: ", err)
	}
	dir, err := os.MkdirTemp("", "ctl-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	compiled, args, err := simfixture.Compile(root, filepath.Join(root, "config/machines/simulation/simulation-program.json"), filepath.Join(dir, "config"))
	simfixture.Must(t, err)
	socket := filepath.Join(dir, "ipc.sock")

	process := exec.Command(binaryPath, append(args, "--socket-path", socket)...)
	simfixture.SpawnLogged(t, process, filepath.Join(dir, "core.log"), 3*time.Second, 9000, nil)

	var client *ipcclient.Client
	var controller *Controller
	t.Cleanup(func() {
		if controller != nil {
			_ = controller.Close()
		}
		if client != nil {
			_ = client.Close()
		}
	})
	simfixture.WaitVerified(t, filepath.Join(dir, "metrics.json"), 5*time.Second, time.Millisecond, func(state simfixture.Verified) bool { return state.ConfigVerifiedMask == 511 })
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	client, err = ipcclient.Dial(ctx, ipcclient.Options{SocketPath: socket, ConfigurationSHA256: filepath.Base(compiled), Backend: "simulation"})
	if err != nil {
		t.Fatal(err)
	}
	binding := Binding{PairID: "lifecycle-test", Revision: 7, ConfigurationSHA256: filepath.Base(compiled)}
	controller, err = New(client, binding)
	if err != nil {
		t.Fatal(err)
	}
	grant, err := controller.Acquire("lifecycle-test", binding)
	if err != nil {
		t.Fatal(err)
	}
	fence := Fence{grant.Session, grant.Generation}
	return lifecycleFixture{controller, client, fence}
}
