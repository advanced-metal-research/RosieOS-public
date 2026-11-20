//go:build linux

package control

import (
	"bytes"
	"context"
	"encoding/binary"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"reflect"
	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/ipcclient"
	"testing"
	"time"
)

// Exercise every point field, including future additions, so retained retry
// state cannot silently omit fields or alias a caller's sample storage.
func TestCellIORestoreCopiesEveryPointField(t *testing.T) {
	var point ipcclient.Point
	value := reflect.ValueOf(&point).Elem()
	for i := 0; i < value.NumField(); i++ {
		field := value.Field(i)
		switch field.Kind() {
		case reflect.Uint32, reflect.Uint64:
			field.SetUint(1)
		case reflect.Int64:
			field.SetInt(10000000)
		case reflect.Slice:
			if field.Type() != reflect.TypeOf([]float64{}) {
				t.Fatalf("add copy coverage for %s", value.Type().Field(i).Name)
			}
			field.Set(reflect.ValueOf([]float64{1, 2}))
		default:
			t.Fatalf("add copy coverage for %s", value.Type().Field(i).Name)
		}
	}
	source := []ipcclient.Point{point, {Position: []float64{}, Velocity: nil}}
	owned := cloneTrajectoryPoints(source)
	if !reflect.DeepEqual(owned, source) {
		t.Fatalf("point fields changed: got %+v, want %+v", owned, source)
	}
	for i := 0; i < value.NumField(); i++ {
		if field := value.Field(i); field.Kind() == reflect.Slice {
			field.Index(0).SetFloat(9)
			if reflect.ValueOf(owned[0]).Field(i).Index(0).Float() != 1 {
				t.Fatalf("retained %s aliases caller storage", value.Type().Field(i).Name)
			}
		}
	}
	source[0] = ipcclient.Point{}
	if owned[0].IOMask != 1 || owned[0].IOValues != 1 {
		t.Fatal("retained marker aliases caller point")
	}
}

func TestCellIOMarkersSurviveStartRefusal(t *testing.T) {
	f := startLifecycleCoreConfig(t, "cell-io/simulation.json", "--sim-dio-inputs", "1")
	if response := publicIOArm(t, f, true); response.Error != "" {
		t.Fatal(response)
	}
	p, err := f.controller.PrepareProgram(f.fence, ioProgram(t, "gas"))
	if err != nil {
		t.Fatal(err)
	}
	if _, err := f.controller.Enable(f.fence, 1); err != nil {
		t.Fatal(err)
	}
	waitIO(t, f, func(s ipcclient.Status) bool { return s.AxisEnableMask == 1 })
	if _, err := f.controller.StartProgram(f.fence, p.Identity); err == nil || err.Error() != ReasonNotReady {
		t.Fatalf("partial enable must refuse Start: %v", err)
	}
	f.controller.mu.Lock()
	h := f.controller.handles[p.handle]
	state, retained := h.State, cloneTrajectoryPoints(h.points)
	f.controller.mu.Unlock()
	if state != HandlePrepared || len(retained) != 2 || retained[0].IOMask != 1 || retained[0].IOValues != 1 {
		t.Fatalf("restoration lost gas marker: state=%s points=%+v", state, retained)
	}
	armLifecycle(t, f.controller, f.client, f.fence)
	if _, err := f.controller.StartProgram(f.fence, p.Identity); err != nil {
		t.Fatal(err)
	}
	on := waitIO(t, f, func(s ipcclient.Status) bool {
		return len(s.IO.Outputs) == 2 && s.IO.Outputs[0].MarkerSampleNS != 0
	})
	gas := on.IO.Outputs[0]
	if gas.MarkerCycle == 0 || gas.ChangedNS < gas.MarkerSampleNS || gas.ChangedNS-gas.MarkerSampleNS > 1000000 ||
		gas.ExpiryNS-gas.ChangedNS != 200000000 {
		t.Fatalf("restored gas marker was not delivered: %+v", gas)
	}
}

func TestCellIONativeTorchPrepareRefusal(t *testing.T) {
	for _, values := range []uint32{0, 2} {
		t.Run(fmt.Sprint(values), func(t *testing.T) {
			f := startLifecycleCoreConfig(t, "cell-io/simulation.json", "--sim-dio-inputs", "1")
			points := []ipcclient.Point{
				{IOMask: 2, IOValues: values, Position: make([]float64, 9)},
				{Time: time.Second, Position: make([]float64, 9)},
			}
			handle, err := f.controller.Prepare(f.fence, 511, points)
			var point *ipcclient.TrajectoryPointError
			var rejected *ipcclient.RejectedError
			if handle != 0 || err == nil || err.Error() != ReasonIoTorchUnqualified ||
				!errors.As(err, &point) || point.PointIndex != 0 || !errors.As(err, &rejected) ||
				rejected.Result.Reason != ipcclient.ReasonIoTorchUnqualified || rejected.Result.Operation != ipcclient.MSG_CMD_TRAJECTORY_POINT {
				t.Fatalf("native torch refusal lost: handle=%d err=%v", handle, err)
			}
		})
	}
}

func ioProgram(t *testing.T, action string) []byte {
	t.Helper()
	blob := stationaryProgram(t, "cell_io")
	n := int(binary.BigEndian.Uint32(blob))
	var h densejointtraj.Header
	if err := json.Unmarshal(blob[4:4+n], &h); err != nil {
		t.Fatal(err)
	}
	h.ProcessMarkers = []densejointtraj.ProcessMarker{{Segment: 0, Sample: 0, Action: action, Value: true}}
	header, err := json.Marshal(h)
	if err != nil {
		t.Fatal(err)
	}
	out := make([]byte, 4)
	binary.BigEndian.PutUint32(out, uint32(len(header)))
	return append(append(out, header...), blob[4+n:]...)
}

func publicIOArm(t *testing.T, f lifecycleFixture, arm bool) Response {
	t.Helper()
	operation := OperationIoDisarm
	if arm {
		operation = OperationIoArm
	}
	body, err := json.Marshal(Request{Schema: RequestSchema, Operation: operation, Fence: f.fence})
	if err != nil {
		t.Fatal(err)
	}
	r := httptest.NewRequest(http.MethodPost, "/v1/control", bytes.NewReader(body))
	r.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	NewHandler(f.controller).ServeHTTP(w, r)
	var response Response
	if err := json.Unmarshal(w.Body.Bytes(), &response); err != nil {
		t.Fatal(err)
	}
	return response
}

func waitIO(t *testing.T, f lifecycleFixture, predicate func(ipcclient.Status) bool) ipcclient.Status {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	s, err := f.client.Wait(ctx, predicate)
	if err != nil {
		t.Fatalf("I/O did not converge: %v %+v", err, s.IO)
	}
	return s
}

func startIOProgram(t *testing.T, f lifecycleFixture) *Program {
	t.Helper()
	response := publicIOArm(t, f, true)
	if response.Error != "" || response.Sequence == 0 {
		t.Fatal(response)
	}
	p, err := f.controller.PrepareProgram(f.fence, ioProgram(t, "gas"))
	if err != nil {
		t.Fatal(err)
	}
	if _, err = f.controller.StartProgram(f.fence, p.Identity); err != nil {
		t.Fatal(err)
	}
	return p
}

// Expiry must remain observable through ChangedNS after the OFF transition;
// a delayed status consumer must not need to catch the expiry cycle itself.
func TestCellIOMarkerBoundAndIndependentExpiry(t *testing.T) {
	// Unverified simulation fixture: 1000000 ns cycle plus 20000000 ns allowed
	// wake lateness in config/cell-io/simulation.json, not measured poll latency.
	const expiryDispatchBoundNS = 21000000
	// Unverified observation gap: skip 300000000 ns beyond the original deadline
	// to exercise retained state across native cycles and metrics publications.
	const expiryObservationGapNS = 300000000
	f := startLifecycleCoreConfig(t, "cell-io/simulation.json", "--sim-dio-inputs", "1", "--sim-dio-readback-lag-cycles", "1")
	assertCellIODescription(t, f)
	startIOProgram(t, f)
	on := waitIO(t, f, func(s ipcclient.Status) bool {
		return len(s.IO.Outputs) == 2 && s.IO.Outputs[0].Commanded && s.IO.Outputs[0].Readback
	})
	o := on.IO.Outputs[0]
	if !o.Intent || !o.Valid || o.MarkerCycle == 0 || o.ChangedNS < o.MarkerSampleNS || o.ChangedNS-o.MarkerSampleNS > 1000000 || o.ExpiryNS-o.ChangedNS != 200000000 {
		t.Fatalf("delivery/expiry bound: %+v", o)
	}
	// No further output request is issued. Grant renewal cannot extend this lease.
	// Wait only for a later native observation, then assert the retained OFF state.
	// TestCellIOFencingExpiryAndReadback covers the exact deadline with a supplied clock;
	// this process test also admits the fixture's configured scheduler lateness.
	off := waitIO(t, f, func(s ipcclient.Status) bool {
		return s.IO.TimeNS >= o.ExpiryNS+expiryObservationGapNS && len(s.IO.Outputs) == 2
	})
	expired := off.IO.Outputs[0]
	if !off.IO.Armed || off.IO.Reason != ipcclient.ReasonNone || off.ExecutionFaultReasons != 0 {
		t.Fatalf("expiry must be independent of disarm or fault: io=%+v faults=%d", off.IO, off.ExecutionFaultReasons)
	}
	if expired.Intent || expired.Commanded || expired.Readback || !expired.Valid || expired.ExpiryNS != 0 ||
		expired.ChangedNS < o.ExpiryNS || expired.ChangedNS-o.ExpiryNS > expiryDispatchBoundNS ||
		expired.ObservedNS < expired.ChangedNS || expired.MarkerSampleNS != o.MarkerSampleNS || expired.MarkerCycle != o.MarkerCycle {
		t.Fatalf("retained expiry failed: on=%+v later=%+v", o, off.IO)
	}
	if response := publicIOArm(t, f, false); response.Error != "" || response.Sequence == 0 {
		t.Fatal(response)
	}
	waitIO(t, f, func(s ipcclient.Status) bool { return !s.IO.Armed })
}

// Describe exposes immutable policy alongside the admitted live topology, so
// clients cannot substitute output intent for readback or omit the terminal.
func assertCellIODescription(t *testing.T, f lifecycleFixture) {
	t.Helper()
	d := f.controller.Describe()
	if d.CapabilitiesDigest != CapabilitiesDigest {
		t.Fatal("stale capabilities digest", d.CapabilitiesDigest)
	}
	var io struct {
		SlavePosition  uint32 `json:"slave_position"`
		TorchQualified bool   `json:"torch_qualified"`
		Inputs         []struct{ Name, Class, Polarity string }
		Outputs        []struct {
			Name, Class string
			SafeState   *bool  `json:"safe_state"`
			ExpiryNS    uint64 `json:"expiry_ns"`
		}
	}
	if err := json.Unmarshal(d.IO, &io); err != nil {
		t.Fatal(err)
	}
	if io.SlavePosition != 9 || io.TorchQualified || len(io.Inputs) != 1 || len(io.Outputs) != 2 ||
		io.Inputs[0].Name != "consent" || io.Inputs[0].Class != "fast" || io.Inputs[0].Polarity != "active_high" ||
		io.Outputs[0].Name != "gas" || io.Outputs[0].Class != "process" || io.Outputs[1].Class != "torch" {
		t.Fatalf("I/O description: %+v", io)
	}
	for _, output := range io.Outputs {
		if output.SafeState == nil || *output.SafeState || output.ExpiryNS != 200000000 {
			t.Fatalf("output policy: %+v", output)
		}
	}
	state, err := f.client.Status()
	if err != nil {
		t.Fatal(err)
	}
	for _, raw := range []json.RawMessage{d.Bus, state.Bus} {
		var bus struct {
			Admitted           bool
			RespondingExpected uint32 `json:"responding_expected"`
			Slaves             []struct {
				SlavePosition uint32 `json:"slave_position"`
				Disposition   string
				Present       bool
				ALState       uint32 `json:"al_state"`
			} `json:"held_slaves"`
		}
		if err := json.Unmarshal(raw, &bus); err != nil {
			t.Fatal(err)
		}
		if !bus.Admitted || bus.RespondingExpected != 10 || len(bus.Slaves) != 1 ||
			bus.Slaves[0].SlavePosition != 9 || bus.Slaves[0].Disposition != "io_terminal" ||
			!bus.Slaves[0].Present || bus.Slaves[0].ALState != 8 {
			t.Fatalf("terminal topology: %+v", bus)
		}
	}
}

func TestCellIOStopRequiresFreshGrantAndExplicitArm(t *testing.T) {
	f := startLifecycleCoreConfig(t, "cell-io/simulation.json", "--sim-dio-inputs", "1")
	startIOProgram(t, f)
	waitIO(t, f, func(s ipcclient.Status) bool { return len(s.IO.Outputs) == 2 && s.IO.Outputs[0].Commanded })
	grant, err := f.controller.Stop(f.fence, false)
	if err != nil {
		t.Fatal(err)
	}
	waitIO(t, f, func(s ipcclient.Status) bool {
		return !s.IO.Armed && !s.IO.Outputs[0].Commanded && !s.IO.Outputs[0].Readback
	})
	if _, err := f.client.IOArm(context.Background(), f.fence.Generation, true); err == nil {
		t.Fatal("old grant armed I/O")
	}
	f.fence = Fence{grant.Session, grant.Generation}
	if f.fence.Generation < 2 {
		t.Fatal("Stop did not advance grant")
	}
	if _, err = f.controller.Renew(f.fence); err != nil {
		t.Fatal(err)
	}
	armLifecycle(t, f.controller, f.client, f.fence)
	p, err := f.controller.PrepareProgram(f.fence, ioProgram(t, "gas"))
	if err != nil {
		t.Fatal(err)
	}
	if _, err = f.controller.StartProgram(f.fence, p.Identity); err == nil {
		t.Fatal("fresh grant bypassed explicit io_arm")
	} else {
		var rejected *ipcclient.RejectedError
		if !errors.As(err, &rejected) || rejected.Result.Reason != ipcclient.ReasonIoNotArmed {
			t.Fatal(err)
		}
	}
	if _, err = f.controller.Renew(f.fence); err != nil {
		t.Fatal(err)
	}
	startIOProgram(t, f)
	waitIO(t, f, func(s ipcclient.Status) bool { return s.IO.Armed && s.IO.Outputs[0].Commanded })
}

// A PREOP spare changes the master AL bitset but cannot prevent the declared
// terminal and all nine OP drives from reaching configuration verification.
func TestCellIOTopologyWithHeldSpare(t *testing.T) {
	root, err := filepath.Abs("../../..")
	if err != nil {
		t.Fatal(err)
	}
	data, err := os.ReadFile(filepath.Join(root, "config/cell-io/simulation.json"))
	if err != nil {
		t.Fatal(err)
	}
	var config map[string]json.RawMessage
	if err := json.Unmarshal(data, &config); err != nil {
		t.Fatal(err)
	}
	config["bus"] = json.RawMessage(`{"unknown_slaves":"hold","unknown_slaves_note":"unverified simulated spare beside the declared terminal"}`)
	data, err = json.Marshal(config)
	if err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(t.TempDir(), "cell.json")
	if err := os.WriteFile(path, data, 0600); err != nil {
		t.Fatal(err)
	}
	name, err := filepath.Rel(filepath.Join(root, "config"), path)
	if err != nil {
		t.Fatal(err)
	}
	f := startLifecycleCoreConfig(t, name, "--sim-dio-inputs", "1", "--sim-extra-slave", "10|4194304|1813|12024")
	var bus struct {
		Admitted           bool
		RespondingExpected uint32                         `json:"responding_expected"`
		Slaves             []struct{ Disposition string } `json:"held_slaves"`
	}
	if err := json.Unmarshal(f.controller.Describe().Bus, &bus); err != nil {
		t.Fatal(err)
	}
	if !bus.Admitted || bus.RespondingExpected != 11 || len(bus.Slaves) != 2 ||
		bus.Slaves[0].Disposition != "io_terminal" || bus.Slaves[1].Disposition != "held" {
		t.Fatalf("terminal and spare: %+v", bus)
	}
	startIOProgram(t, f)
	waitIO(t, f, func(s ipcclient.Status) bool {
		return s.IO.Armed && len(s.IO.Outputs) == 2 && s.IO.Outputs[0].Commanded && s.IO.Outputs[0].Readback
	})
}

func TestCellIOReadbackDisagreementAndTorchRefusal(t *testing.T) {
	f := startLifecycleCoreConfig(t, "cell-io/simulation.json", "--sim-dio-inputs", "1", "--sim-dio-stuck-readback")
	if _, err := f.controller.PrepareProgram(f.fence, ioProgram(t, "torch")); err == nil || err.Error() != ReasonProcessIoExecutorNotQualified {
		t.Fatal("torch admission", err)
	}
	if response := publicIOArm(t, f, true); response.Error != "" || response.Sequence == 0 {
		t.Fatal(response)
	}
	p, err := f.controller.PrepareProgram(f.fence, ioProgram(t, "gas"))
	if err != nil {
		t.Fatal(err)
	}
	_, _ = f.controller.StartProgram(f.fence, p.Identity)
	s := waitIO(t, f, func(s ipcclient.Status) bool { return s.IO.Reason == ipcclient.ReasonIoReadbackDisagreement })
	if s.IO.Armed || s.IO.Outputs[0].Commanded || s.IO.Outputs[0].Readback || s.ExecutionFaultReasons&8192 == 0 {
		t.Fatalf("readback fault: %+v", s)
	}
}

func TestCellIOHeaderMarkersPreserveDwellAndIdentity(t *testing.T) {
	description, ids := programDescription()
	source := seamProgramTest(t, .01)
	source.Header.ProcessMarkers = []densejointtraj.ProcessMarker{{Segment: 1, Sample: 0, Action: "gas", Value: true}}
	decode := func() *Program {
		t.Helper()
		p, err := DecodeProgram(encodeProgramTest(t, source), description, ids)
		if err != nil {
			t.Fatal(err)
		}
		return p
	}
	p := decode()
	if len(p.points) != 5 || len(p.Markers) != 1 || p.Markers[0].TimeNS != 20000000 || p.Markers[0].Segment != 1 || p.Markers[0].Sample != 0 {
		t.Fatal(p)
	}
	c := &Controller{core: &alarmStatusCore{status: ipcclient.Status{IO: ipcclient.IoState{Outputs: []ipcclient.IoOutput{{Name: "gas"}}}}}}
	if err := c.bindIOMarkers(p); err != nil {
		t.Fatal(err)
	}
	if p.points[2].Time != 20000000 || p.points[2].IOMask != 1 || p.points[2].IOValues != 1 {
		t.Fatal(p.points)
	}
	source.Header.ProcessMarkers[0].Value = false
	off := decode()
	if off.Identity.SourceDigest == p.Identity.SourceDigest || off.Identity.NormalisedDigest == p.Identity.NormalisedDigest {
		t.Fatal("marker intent retained prepared identity")
	}
	source.Header.ProcessMarkers = append(source.Header.ProcessMarkers, source.Header.ProcessMarkers[0])
	if err := c.bindIOMarkers(decode()); err == nil || err.Error() != ReasonIoMarkerInvalid {
		t.Fatal("duplicate marker", err)
	}
	source.Header.ProcessMarkers = source.Header.ProcessMarkers[:1]
	source.Header.ProcessMarkers[0].Action = "unknown"
	if err := c.bindIOMarkers(decode()); err == nil || err.Error() != ReasonIoMarkerInvalid {
		t.Fatal("unknown output", err)
	}
	source.Header.ProcessMarkers[0].Action = "gas"
	c.core.(*alarmStatusCore).status.IO.Outputs[0].Torch = true
	if err := c.bindIOMarkers(decode()); err == nil || err.Error() != ReasonIoTorchUnqualified {
		t.Fatal("torch alias", err)
	}
}

// Manual tests acquire I/O permission without enabling or arming any axis.
func TestCellIOManualTestDisarmedExpiryAndStop(t *testing.T) {
	f := startLifecycleCoreConfigArmed(t, "cell-io/simulation.json", false, "--sim-dio-inputs", "1")
	if _, err := f.controller.IOTest(f.fence, "gas", true, 1, true); err != nil {
		t.Fatal(err)
	}
	state := waitIO(t, f, func(s ipcclient.Status) bool { return len(s.IO.Outputs) > 0 && s.IO.Outputs[0].Commanded })
	if state.Armed != 0 || state.AxisEnableMask != 0 {
		t.Fatal("manual I/O armed motion", state.Armed, state.AxisEnableMask)
	}
	waitIO(t, f, func(s ipcclient.Status) bool { return !s.IO.Outputs[0].Commanded })
	if _, err := f.controller.IOTest(f.fence, "gas", true, 2); err == nil {
		t.Fatal("heartbeat revived expired ON")
	}
	if _, err := f.controller.IOTest(f.fence, "gas", true, 3, true); err != nil {
		t.Fatal(err)
	}
	if _, err := f.controller.IOTest(f.fence, "gas", false, 5); err != nil {
		t.Fatal(err)
	}
	if _, err := f.controller.IOTest(f.fence, "gas", true, 4); err == nil {
		t.Fatal("late ON passed newer OFF")
	}
	waitIO(t, f, func(s ipcclient.Status) bool { return !s.IO.Outputs[0].Commanded })
	if _, err := f.controller.IOTest(f.fence, "torch", true, 5, true); err != nil {
		t.Fatal("manual torch terminal test refused", err)
	}
	state = waitIO(t, f, func(s ipcclient.Status) bool { return s.IO.Outputs[1].Commanded })
	if state.Armed != 0 || state.AxisEnableMask != 0 {
		t.Fatal("terminal test armed motion")
	}
	if _, err := f.controller.IOTest(f.fence, "torch", false, 6); err != nil {
		t.Fatal(err)
	}
	waitIO(t, f, func(s ipcclient.Status) bool { return !s.IO.Outputs[1].Commanded })
	if _, err := f.controller.IOTest(f.fence, "gas", true, 6, true); err != nil {
		t.Fatal(err)
	}
	if _, err := f.controller.Stop(f.fence, false); err != nil {
		t.Fatal(err)
	}
	waitIO(t, f, func(s ipcclient.Status) bool { return !s.IO.Armed && !s.IO.Outputs[0].Commanded })
	if _, err := f.controller.IOTest(f.fence, "gas", true, 7, true); err == nil {
		t.Fatal("stopped test session revived")
	}
}

// Full native upload/run path: the immutable input barrier holds a stationary
// target, succeeds on fresh evidence, and faults on timeout without losing Home.
func TestCellIOInputWaitExecution(t *testing.T) {
	for _, want := range []bool{true, false} {
		t.Run(fmt.Sprint(want), func(t *testing.T) {
			f := startLifecycleCoreConfig(t, "cell-io/simulation.json", "--sim-dio-inputs", "1")
			state, _ := f.client.Status()
			p := []ipcclient.Point{{Position: make([]float64, 9), Velocity: make([]float64, 9), WaitTimeoutNS: 40000000, WaitStableNS: 10000000},
				{Time: 100 * time.Millisecond, Position: make([]float64, 9), Velocity: make([]float64, 9)}}
			if want {
				p[0].WaitValue = 1
			}
			handle, err := f.controller.Prepare(f.fence, 511, p)
			if err != nil {
				t.Fatal(err)
			}
			if _, err = f.controller.Start(f.fence, handle); err != nil {
				t.Fatal(err)
			}
			time.Sleep(20 * time.Millisecond)
			mid, _ := f.client.Motion()
			if mid.Done {
				t.Fatal("wait did not hold the plan", mid)
			}
			final := waitIO(t, f, func(s ipcclient.Status) bool { motion, _ := f.client.Motion(); return motion.Done })
			if want {
				if final.ExecutionFaultReasons != 0 {
					t.Fatal(final.ExecutionFaultReasons)
				}
			} else {
				if final.IO.Reason != ipcclient.ReasonIoWaitTimeout || final.ExecutionFaultReasons&(1<<14) == 0 {
					t.Fatal("missing timeout", final.IO.Reason, final.ExecutionFaultReasons)
				}
				if final.HomeValidMask != state.HomeValidMask {
					t.Fatal("wait timeout invalidated Home")
				}
			}
		})
	}
}

func TestCellIOStopCancelsPendingInputWait(t *testing.T) {
	f := startLifecycleCoreConfig(t, "cell-io/simulation.json", "--sim-dio-inputs", "1")
	p := []ipcclient.Point{{Position: make([]float64, 9), Velocity: make([]float64, 9), WaitTimeoutNS: 1000000000, WaitValue: 0},
		{Time: 100 * time.Millisecond, Position: make([]float64, 9), Velocity: make([]float64, 9)}}
	handle, err := f.controller.Prepare(f.fence, 511, p)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = f.controller.Start(f.fence, handle); err != nil {
		t.Fatal(err)
	}
	time.Sleep(30 * time.Millisecond)
	motion, _ := f.client.Motion()
	if motion.Done {
		t.Fatal("wait did not hold", motion)
	}
	if _, err = f.controller.Stop(f.fence, false); err != nil {
		t.Fatal(err)
	}
	waitIO(t, f, func(s ipcclient.Status) bool { m, _ := f.client.Motion(); return m.Done && !s.IO.Armed })
	if _, err = f.controller.Start(f.fence, handle); err == nil {
		t.Fatal("stopped wait resumed with stale authority")
	}
}

func TestCellIOQualifiedWeldProgramStartsStopsAndCannotBeTestedManually(t *testing.T) {
	// Synthetic wired terminal, never a physical qualification record.
	raw, err := os.ReadFile("../../../config/cell-io/simulation.json")
	if err != nil {
		t.Fatal(err)
	}
	var cfg map[string]any
	if err = json.Unmarshal(raw, &cfg); err != nil {
		t.Fatal(err)
	}
	io := cfg["io"].(map[string]any)
	// A 10 ms synthetic exchange gives this VM a declared scheduling budget;
	// it is not the second 1430 cell timing qualification. Native unit tests cover exact deadlines.
	cfg["cycle_ns"] = 10000000
	io["torch_qualified"] = true
	io["torch_qualification_record"] = "simulation-only-test-fixture"
	raw, _ = json.Marshal(cfg)
	path := filepath.Join(t.TempDir(), "qualified-simulation.json")
	if err = os.WriteFile(path, raw, 0600); err != nil {
		t.Fatal(err)
	}
	f := startLifecycleCoreConfig(t, path, "--sim-dio-inputs", "1")
	plan, err := densejointtraj.Decode(stationaryProgram(t, "qualified_torch"))
	if err != nil {
		t.Fatal(err)
	}
	// Refresh ON every 50 ms, well inside the fixture's 200 ms expiry.
	// The sparse two-second source must be rejected before this valid schedule.
	plan.Header.ProcessMarkers = []densejointtraj.ProcessMarker{{Segment: 0, Sample: 0, Action: "torch", Value: true}, {Segment: 0, Sample: 1, Action: "torch", Value: false}}
	sparse, err := densejointtraj.Encode(plan.Header, plan.Segments)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = f.controller.PrepareProgram(f.fence, sparse); err == nil {
		t.Fatal("missing torch refresh accepted")
	}
	plan.Segments[0].Samples = nil
	plan.Header.ProcessMarkers = nil
	for i := 0; i <= 40; i++ {
		plan.Segments[0].Samples = append(plan.Segments[0].Samples, densejointtraj.Sample{TimeS: float64(i) * .05})
		plan.Header.ProcessMarkers = append(plan.Header.ProcessMarkers, densejointtraj.ProcessMarker{Segment: 0, Sample: i, Action: "torch", Value: i < 40})
	}
	plan.Header.Sampling.DtS = .05
	plan.Header.Sampling.TotalSampleCount = 41
	plan.Header.Segments[0].SampleCount = 41
	out, err := densejointtraj.Encode(plan.Header, plan.Segments)
	if err != nil {
		t.Fatal(err)
	}
	program, err := f.controller.PrepareProgram(f.fence, out)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = f.controller.IOTest(f.fence, "torch", true, 1, true); err == nil {
		t.Fatal("manual torch accepted")
	}
	if _, err = f.controller.IOArm(f.fence, true); err != nil {
		t.Fatal(err)
	}
	sequence, err := f.controller.StartProgram(f.fence, program.Identity)
	if err != nil {
		t.Fatal(err)
	}
	started := waitActiveProgram(t, f.client, program.handle, sequence, 0)
	final := waitIO(t, f, func(s ipcclient.Status) bool {
		m, err := f.client.Motion()
		return err == nil && m.TrajectoryID == program.handle && m.CommandSequence == sequence && m.TimeNS > started.TimeNS && m.Done && !s.IO.Outputs[1].Commanded
	})
	if final.ExecutionFaultReasons != 0 || final.IO.Outputs[1].Commanded {
		t.Fatal("weld did not finish safely", final.ExecutionFaultReasons, final.IO)
	}
	if final.IO.Outputs[1].MarkerCycle == 0 {
		t.Fatal("no qualified weld marker executed")
	}
}

func TestCellIOInputMonitorScopeBinding(t *testing.T) {
	state := ipcclient.IoState{Inputs: []ipcclient.IoInput{{Name: "sensor"}}}
	markers := []ProcessMarker{{TimeNS: 10000000, Action: "sensor", Value: true, Monitor: "start"}, {TimeNS: 30000000, Action: "sensor", Monitor: "end"}}
	makePlan := func(ms []ProcessMarker) *Program {
		p := &Program{Markers: ms}
		for i := 0; i < 5; i++ {
			p.points = append(p.points, ipcclient.Point{Time: time.Duration(i) * 10 * time.Millisecond})
		}
		return p
	}
	p := makePlan(markers)
	if err := bindInputMonitors(p, state); err != nil {
		t.Fatal(err)
	}
	for i, point := range p.points {
		want := uint32(0)
		if i == 1 || i == 2 {
			want = 1
		}
		if point.MonitorMask != want || point.MonitorValues != want {
			t.Fatalf("point %d: %+v", i, point)
		}
	}
	for _, bad := range [][]ProcessMarker{markers[:1], markers[1:], {markers[0], markers[0], markers[1]}, {{TimeNS: 15000000, Action: "sensor", Monitor: "start"}, markers[1]}} {
		if err := bindInputMonitors(makePlan(bad), state); err == nil {
			t.Fatal("invalid monitor scope accepted", bad)
		}
	}
}

func TestCellIOInputMonitorChecksDuringExecution(t *testing.T) {
	for _, expected := range []uint32{0, 1} {
		t.Run(fmt.Sprint(expected), func(t *testing.T) {
			f := startLifecycleCoreConfig(t, "cell-io/simulation.json", "--sim-dio-inputs", "1")
			before, _ := f.client.Status()
			points := []ipcclient.Point{
				{Position: make([]float64, 9), Velocity: make([]float64, 9)},
				{Time: 50 * time.Millisecond, Position: make([]float64, 9), Velocity: make([]float64, 9), MonitorMask: 1, MonitorValues: expected},
				{Time: 150 * time.Millisecond, Position: make([]float64, 9), Velocity: make([]float64, 9)},
				{Time: 200 * time.Millisecond, Position: make([]float64, 9), Velocity: make([]float64, 9)}}
			handle, err := f.controller.Prepare(f.fence, 511, points)
			if err != nil {
				t.Fatal(err)
			}
			seq, err := f.controller.Start(f.fence, handle)
			if err != nil {
				t.Fatal(err)
			}
			started := waitActiveProgram(t, f.client, handle, seq, 0)
			final := waitIO(t, f, func(s ipcclient.Status) bool {
				m, e := f.client.Motion()
				return e == nil && m.TrajectoryID == handle && m.CommandSequence == seq && m.TimeNS > started.TimeNS && m.Done
			})
			if expected == 1 && final.ExecutionFaultReasons != 0 {
				t.Fatal("satisfied monitor faulted", final.ExecutionFaultReasons)
			}
			if expected == 0 && final.ExecutionFaultReasons&(1<<14) == 0 {
				t.Fatal("mismatch did not fault")
			}
			if final.HomeValidMask != before.HomeValidMask {
				t.Fatal("monitor changed Home")
			}
		})
	}
}
