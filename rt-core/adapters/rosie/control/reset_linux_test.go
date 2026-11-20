//go:build linux

package control

import (
	"context"
	"encoding/json"
	"fmt"
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

type publicResetResponse struct {
	Response
	Data json.RawMessage `json:"data"`
}

type publicResetStatus struct {
	Core      ipcclient.Status      `json:"core"`
	Motion    ipcclient.MotionState `json:"motion"`
	Execution Execution             `json:"execution"`
}

type publicResetFixture struct {
	client  *http.Client
	binding Binding
	mu      sync.Mutex
	fence   Fence
	coreLog string
}

// publicResetOptions selects deployed behaviour the reset tests leave off.
type publicResetOptions struct {
	// requestTimeout bounds a complete sequential commissioning transaction.
	requestTimeout time.Duration
	// homeRefresh keeps the startup Home-offset refresh enabled, as a deployed
	// daemon runs it; the public reset tests run with it disabled.
	homeRefresh bool
	// compiledConfig lets the adapter serve the compiled robot description.
	compiledConfig bool
}

// Use real processes and only control.sock for commands and observations, so
// native-client shortcuts cannot conceal a missing public recovery operation.
func startPublicReset(t *testing.T, config string, mask uint32, extra ...string) *publicResetFixture {
	t.Helper()
	return startPublicResetWith(t, publicResetOptions{}, config, mask, extra...)
}

func startPublicResetWith(t *testing.T, options publicResetOptions, config string, mask uint32, extra ...string) *publicResetFixture {
	t.Helper()
	root, err := filepath.Abs("../../..")
	if err != nil {
		t.Fatal(err)
	}
	binaryPath := simfixture.Locate(root, "rosie-rt-core-sim")
	controlPath := simfixture.Locate(root, "rt-control")
	for _, path := range []string{binaryPath, controlPath} {
		if _, err := os.Stat(path); err != nil {
			t.Fatal("run make control and make sim first: ", err)
		}
	}
	artifacts, err := os.MkdirTemp(filepath.Join(root, "build"), "public-reset-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(artifacts) })
	sockets, err := os.MkdirTemp("", "reset-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(sockets) })
	compiled, args, err := simfixture.Compile(root, simfixture.MachineConfigPath(root, config), filepath.Join(artifacts, "config"))
	simfixture.Must(t, err)
	coreSocket := filepath.Join(sockets, "ipc.sock")
	controlSocket := filepath.Join(sockets, "control.sock")
	args = append(args, "--socket-path", coreSocket)
	args = append(args, extra...)
	coreLog := filepath.Join(artifacts, "core.log")
	startPublicResetProcess(t, binaryPath, args, coreLog, options.homeRefresh)
	// Unverified fixture startup budget: 5 seconds; readiness is observed.
	deadline := time.Now().Add(5 * time.Second)
	// Unverified fixture polling interval: 10 ms; never a positive assertion.
	ticker := time.NewTicker(10 * time.Millisecond)
	defer ticker.Stop()
	for {
		raw, _ := os.ReadFile(filepath.Join(sockets, "metrics.json"))
		var state ipcclient.Status
		if json.Unmarshal(raw, &state) == nil && state.ConfigVerifiedMask == mask {
			break
		}
		if time.Now().After(deadline) {
			t.Fatalf("configuration never verified: %+v", state)
		}
		<-ticker.C
	}
	f := &publicResetFixture{binding: Binding{PairID: "public-reset-test", Revision: 7, ConfigurationSHA256: filepath.Base(compiled)}, coreLog: coreLog}
	controlArgs := []string{
		"--core-socket", coreSocket, "--socket", controlSocket,
		"--configuration-sha256", f.binding.ConfigurationSHA256, "--backend", "simulation",
		"--pair-id", f.binding.PairID, "--pair-revision", "7",
	}
	if options.compiledConfig {
		controlArgs = append(controlArgs, "--compiled-config", compiled)
	}
	startPublicResetProcess(t, controlPath, controlArgs, filepath.Join(artifacts, "control.log"), options.homeRefresh)
	transport := unixhttp.NewClient(controlSocket).Transport.(*http.Transport)
	// Unverified HTTP timeout: 5 seconds; bounds failed process interactions.
	requestTimeout := options.requestTimeout
	if requestTimeout == 0 {
		requestTimeout = 5 * time.Second
	}
	f.client = &http.Client{Transport: transport, Timeout: requestTimeout}
	t.Cleanup(transport.CloseIdleConnections)
	deadline = time.Now().Add(5 * time.Second)
	for {
		response, code, err := f.request(http.MethodGet, "/v1/status", nil, nil)
		if err == nil && code == http.StatusOK && response.Error == "" {
			break
		}
		if time.Now().After(deadline) {
			t.Fatalf("public API startup: %+v %v", response, err)
		}
		<-ticker.C
	}
	stopRenew := simfixture.Renew(context.Background(), 100*time.Millisecond, func() bool {
		f.mu.Lock()
		defer f.mu.Unlock()
		if f.fence.Session != "" {
			raw, _ := json.Marshal(Request{Schema: "rosie.rt-control.request.v1", Operation: "renew", Fence: f.fence})
			response, code, err := f.request(http.MethodPost, "/v1/control", raw, nil)
			if err != nil || code != http.StatusOK || response.Error != "" {
				t.Errorf("public renewal: code=%d response=%+v error=%v", code, response, err)
				return false
			}
		}
		return true
	})
	t.Cleanup(stopRenew)
	f.command(t, Request{Operation: "acquire", Controller: "reset-test", Binding: f.binding}, "")
	return f
}

func startPublicResetProcess(t *testing.T, binary string, args []string, logPath string, homeRefresh bool) {
	t.Helper()

	process := exec.Command(binary, args...)
	process.Env = os.Environ()
	if !homeRefresh {
		process.Env = append(process.Env, "GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED=0")
	}
	simfixture.SpawnLogged(t, process, logPath, 3*time.Second, 9000, simfixture.CheckExit(t))

}

func (f *publicResetFixture) request(method, path string, body []byte, headers http.Header) (publicResetResponse, int, error) {
	var response publicResetResponse
	code, err := (controltest.Client{HTTP: f.client, Base: "http://rt-control"}).Decode(context.Background(), method, path, body, headers, &response)
	return response, code, err
}

// Only grant-changing operations exclude renewal; Home can wait for native
// commissioning while the separate public renewal connection keeps it alive.
func (f *publicResetFixture) command(t *testing.T, request Request, wantError string) publicResetResponse {
	t.Helper()
	f.mu.Lock()
	if request.Fence == (Fence{}) {
		request.Fence = f.fence
	}
	grantChange := request.Operation == "acquire" || request.Operation == "release" || request.Operation == "stop" || request.Operation == "reset_fault"
	if grantChange {
		defer f.mu.Unlock()
	} else {
		f.mu.Unlock()
	}
	request.Schema = "rosie.rt-control.request.v1"
	raw, err := json.Marshal(request)
	if err != nil {
		t.Fatal(err)
	}
	response, code, err := f.request(http.MethodPost, "/v1/control", raw, nil)
	wantCode := http.StatusOK
	if wantError != "" {
		wantCode = http.StatusConflict
	}
	if err != nil || code != wantCode || response.Error != wantError || response.Operation != request.Operation {
		t.Fatalf("%s: want HTTP %d error %q, got HTTP %d response=%+v transport=%v", request.Operation, wantCode, wantError, code, response, err)
	}
	if request.Operation == "reset_fault" && wantError != "" && (response.Sequence != 0 || response.NativeResult != nil) {
		t.Fatalf("refused reset reached native: %+v", response)
	}
	if wantError == "" {
		switch request.Operation {
		case "acquire", "stop":
			var grant Grant
			if err := json.Unmarshal(response.Data, &grant); err != nil || grant.Session == "" {
				t.Fatalf("missing public grant: %+v %v", response, err)
			}
			f.fence = Fence{grant.Session, grant.Generation}
		case "release", "reset_fault":
			f.fence = Fence{}
		}
	}
	return response
}

func (f *publicResetFixture) wait(t *testing.T, predicate func(publicResetStatus) bool) publicResetStatus {
	t.Helper()
	// Unverified observation deadline: 8 seconds; success requires public state.
	deadline := time.Now().Add(8 * time.Second)
	// Unverified observation polling interval: 10 ms.
	ticker := time.NewTicker(10 * time.Millisecond)
	defer ticker.Stop()
	for {
		response, code, err := f.request(http.MethodGet, "/v1/status", nil, nil)
		var status publicResetStatus
		if err != nil || code != http.StatusOK || response.Error != "" {
			t.Fatalf("public status: %+v %v", response, err)
		}
		if err := json.Unmarshal(response.Data, &status); err != nil {
			t.Fatal(err)
		}
		if predicate(status) {
			return status
		}
		if time.Now().After(deadline) {
			t.Fatalf("public status deadline: %+v", status)
		}
		<-ticker.C
	}
}

func (f *publicResetFixture) ready(t *testing.T, mask uint32) {
	t.Helper()
	f.command(t, Request{Operation: "enable", AxisMask: mask}, "")
	f.command(t, Request{Operation: "arm"}, "")
	f.wait(t, func(s publicResetStatus) bool {
		if s.Core.Armed != 1 || s.Core.AxisEnableMask != mask {
			return false
		}
		return ipcclient.AllOperationEnabled(s.Core)
	})
}

func requirePublicResetResult(t *testing.T, response publicResetResponse) {
	t.Helper()
	if response.Sequence == 0 || response.NativeResult == nil || response.NativeResult.Sequence != response.Sequence ||
		response.NativeResult.Operation != ipcclient.MsgFaultReset || response.NativeResult.Result != ipcclient.ResultAccepted ||
		response.NativeResult.Reason != ipcclient.ReasonNone || response.NativeResult.Generation == 0 {
		t.Fatalf("reset did not return a correlated native acceptance: %+v", response)
	}
}

func TestPublicResetFaultRecoversIdleBusFault(t *testing.T) {
	f := startPublicReset(t, "simulation-home.json", 1,
		"--sim-send-fail-from-call", "5000", "--sim-send-fail-for-calls", "600")
	f.command(t, Request{Operation: "home", AxisMask: 1}, "")
	f.wait(t, func(s publicResetStatus) bool {
		return s.Core.HomeValidMask == 1 && s.Core.ConfigVerifiedMask == 1 && s.Core.CommissioningPhase == 0 &&
			s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Core.SafetyFaultMask == 0 && s.Core.ExecutionFaultReasons == 0
	})
	// Release before the injection: the fault must latch without motion authority.
	f.command(t, Request{Operation: "release"}, "")
	faulted := f.wait(t, func(s publicResetStatus) bool {
		return s.Motion.State == 6 && s.Motion.Done && s.Core.SafetyFaultMask == 1 && s.Core.ExecutionFaultReasons&256 != 0
	})
	if faulted.Core.Armed != 0 || faulted.Core.AxisEnableMask != 0 || faulted.Core.HomeValidMask != 1 ||
		faulted.Core.LastBusFailureOperation != 5 || faulted.Core.LastBusFailureCode != -5 {
		t.Fatalf("idle send failure did not inhibit and preserve trustworthy Home: %+v", faulted)
	}
	f.wait(t, func(s publicResetStatus) bool {
		return s.Core.TimeNS > faulted.Core.TimeNS && s.Core.ConfigVerifiedMask == 1 && s.Core.SafetyFaultMask == 1 &&
			s.Core.ExecutionFaultReasons&256 != 0 && s.Core.HomeValidMask == 1
	})
	f.command(t, Request{Operation: "acquire", Controller: "reset-test", Binding: f.binding}, "")
	response := f.command(t, Request{Operation: "reset_fault", AxisMask: 0}, "")
	requirePublicResetResult(t, response)
	resetAt := ipcclient.HostMonotonicNS()
	f.wait(t, func(s publicResetStatus) bool {
		return s.Core.TimeNS >= resetAt && s.Core.SafetyFaultMask == 0 && s.Core.ExecutionFaultReasons == 0 &&
			s.Core.HomeValidMask == 1 && s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Core.ConfigVerifiedMask == 1
	})
	f.command(t, Request{Operation: "acquire", Controller: "reset-test", Binding: f.binding}, "")
	f.ready(t, 1)
	plan := f.command(t, Request{Operation: "prepare_trajectory", AxisMask: 1, Points: []ipcclient.Point{
		{Time: 0, Position: []float64{0}},
		{Time: 100 * time.Millisecond, Position: []float64{0.001}},
	}}, "")
	start := f.command(t, Request{Operation: "start_trajectory", Handle: plan.Handle}, "")
	final := f.wait(t, func(s publicResetStatus) bool {
		return s.Motion.TrajectoryID == plan.Handle && s.Motion.CommandSequence == start.Sequence && s.Motion.Done && s.Motion.State == 4
	})
	if final.Core.SafetyFaultMask != 0 || final.Core.ExecutionFaultReasons != 0 || final.Core.HomeValidMask != 1 {
		t.Fatalf("public recovery left a fault/home gate: %+v", final)
	}
}

func TestPublicResetFaultRefusedWhileArmedOrExecuting(t *testing.T) {
	f := startPublicReset(t, "simulation-program.json", 511)
	f.command(t, Request{Operation: "enable", AxisMask: 511}, "")
	f.command(t, Request{Operation: "arm"}, "")
	// Submit immediately after Arm's receipt, before waiting for armed metrics.
	rejected := f.command(t, Request{Operation: "reset_fault"}, "reset_requires_inhibited")
	if rejected.Sequence != 0 || rejected.NativeResult != nil {
		t.Fatalf("armed reset reached native: %+v", rejected)
	}
	f.ready(t, 511)
	for _, program := range []bool{false, true} {
		t.Run(fmt.Sprintf("program=%t", program), func(t *testing.T) {
			var start publicResetResponse
			if program {
				f.mu.Lock()
				fence := f.fence
				f.mu.Unlock()
				response, code, err := f.request(http.MethodPost, "/v1/program", stationaryProgram(t, "reset-refusal"), http.Header{
					"X-Control-Session": {fence.Session}, "X-Control-Generation": {fmt.Sprint(fence.Generation)},
				})
				var prepared Program
				if err != nil || code != http.StatusOK || response.Error != "" || json.Unmarshal(response.Data, &prepared) != nil {
					t.Fatalf("prepare public program: %+v %v", response, err)
				}
				start = f.command(t, Request{Operation: "start_program", Identity: prepared.Identity}, "")
			} else {
				plan := f.command(t, Request{Operation: "prepare_trajectory", AxisMask: 511, Points: []ipcclient.Point{
					{Time: 0, Position: make([]float64, 9)}, {Time: 2 * time.Second, Position: make([]float64, 9)},
				}}, "")
				start = f.command(t, Request{Operation: "start_trajectory", Handle: plan.Handle}, "")
			}
			f.command(t, Request{Operation: "reset_fault"}, "program_already_executing")
			f.mu.Lock()
			stale := f.fence
			f.mu.Unlock()
			stale.Generation++
			f.command(t, Request{Operation: "reset_fault", Fence: stale}, "control_session_stale")
			f.wait(t, func(s publicResetStatus) bool {
				return s.Motion.CommandSequence == start.Sequence && s.Motion.State == 3 && !s.Motion.Done && s.Core.Armed == 1
			})
			f.command(t, Request{Operation: "stop"}, "")
			f.ready(t, 511)
		})
	}
	f.command(t, Request{Operation: "jog", AxisMask: 511, Velocity: make([]float64, 9), TimeoutMS: 250}, "")
	f.wait(t, func(s publicResetStatus) bool { return s.Motion.Mode == 3 && !s.Motion.Done })
	f.command(t, Request{Operation: "reset_fault"}, "reset_requires_inhibited")
}

func TestPublicResetFaultPreservesUnaffectedHomeAndRetiresFence(t *testing.T) {
	f := startPublicReset(t, "simulation-home.json", 1)
	f.command(t, Request{Operation: "home", AxisMask: 1}, "")
	f.wait(t, func(s publicResetStatus) bool { return s.Core.HomeValidMask == 1 && s.Core.CommissioningPhase == 0 })
	f.mu.Lock()
	oldFence := f.fence
	f.mu.Unlock()
	f.command(t, Request{Operation: "reset_fault", AxisMask: 2}, "invalid RTCore axis mask")
	response := f.command(t, Request{Operation: "reset_fault", AxisMask: 1}, "")
	requirePublicResetResult(t, response)
	after := ipcclient.HostMonotonicNS()
	f.wait(t, func(s publicResetStatus) bool {
		return s.Core.TimeNS >= after && s.Core.HomeValidMask == 1 && s.Core.Armed == 0 && s.Core.AxisEnableMask == 0
	})
	f.command(t, Request{Operation: "reset_fault", Fence: oldFence}, "control_session_stale")
	f.command(t, Request{Operation: "acquire", Controller: "reset-test", Binding: f.binding}, "")
	f.mu.Lock()
	freshFence := f.fence
	f.mu.Unlock()
	if freshFence.Generation != oldFence.Generation+1 || freshFence.Session == oldFence.Session {
		t.Fatalf("reset reused authority: old=%+v new=%+v", oldFence, freshFence)
	}
}
