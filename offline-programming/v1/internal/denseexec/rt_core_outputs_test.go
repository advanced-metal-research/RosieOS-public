//go:build linux

package denseexec

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"reflect"
	"strings"
	"sync"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/densejointtraj"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// outputsRTCore is the adapter's preparation contract for header markers
// (rt-core adapters/rosie/control/program.go): with markers the source
// identity is the sha256 of the uploaded envelope and the program requires
// process I/O; torch bits stay refused. io_arm can be refused with a reason.
type outputsRTCore struct {
	*fakeRTCore
	captureMu   sync.Mutex
	blobs       [][]byte
	ioArmRefuse string
}

func (f *outputsRTCore) PrepareProgram(ctx context.Context, blob []byte) (control.Program, error) {
	f.captureMu.Lock()
	f.blobs = append(f.blobs, append([]byte(nil), blob...))
	f.captureMu.Unlock()
	p, err := f.fakeRTCore.PrepareProgram(ctx, blob)
	if err != nil {
		return p, err
	}
	plan, err := densejointtraj.Decode(blob)
	if err != nil {
		return p, err
	}
	for _, segment := range plan.Segments {
		for _, sample := range segment.Samples {
			if sample.TorchOn {
				return p, &control.Rejected{Reason: "process_io_executor_not_qualified"}
			}
		}
	}
	p.Identity.TrajectoryDigest = plan.Header.TrajectoryDigest
	p.Identity.SourceDigest = plan.Header.TrajectoryDigest
	p.RequiresProcessIO = len(plan.Header.ProcessMarkers) != 0
	if p.RequiresProcessIO {
		sum := sha256.Sum256(blob)
		p.Identity.SourceDigest = "sha256:" + hex.EncodeToString(sum[:])
	}
	p.Identity.NormalisedDigest = strings.Repeat("f", 64)
	return p, nil
}

func (f *outputsRTCore) Command(ctx context.Context, request *control.Request) (control.Response, error) {
	if request.Operation == "io_arm" || request.Operation == "io_disarm" {
		f.fakeRTCore.mu.Lock()
		f.fakeRTCore.record(request.Operation)
		refuse := f.ioArmRefuse
		f.fakeRTCore.mu.Unlock()
		if request.Operation == "io_arm" && refuse != "" {
			return control.Response{}, &control.Rejected{Reason: refuse}
		}
		return control.Response{Operation: request.Operation, Sequence: 1}, nil
	}
	return f.fakeRTCore.Command(ctx, request)
}

// The simulation fixture's outputs (rt-core config/cell-io/simulation.json):
// gas (process) and torch (torch class), torch not qualified.
const simulationIO = `{"profile":"dio_terminal","slave_position":9,"torch_qualified":false,
"inputs":[{"name":"consent","bit":0,"polarity":"active_high","class":"fast"}],
"outputs":[{"name":"gas","bit":0,"class":"process","safe_state":false,"expiry_ns":200000000},
{"name":"torch","bit":1,"class":"torch","safe_state":false,"expiry_ns":200000000}]}`

// outputPlan is the motion-only fixture with a header output schedule; the
// header digest is the planner's unless headerDigest overrides it.
func outputPlan(t *testing.T, markers []densejointtraj.ProcessMarker, headerDigest ...string) ([]byte, Identity, string) {
	t.Helper()
	source, _ := fixtureBlob(t)
	motion, _, err := densejointtraj.WithoutProcessOutputs(source)
	if err != nil {
		t.Fatal(err)
	}
	length := binary.BigEndian.Uint32(motion[:4])
	var header map[string]json.RawMessage
	if err := json.Unmarshal(motion[4:4+length], &header); err != nil {
		t.Fatal(err)
	}
	digest := densejointtraj.ProcessMarkersDigest(markers)
	recorded := digest
	if len(headerDigest) != 0 {
		recorded = headerDigest[0]
	}
	header["process_markers"], _ = json.Marshal(markers)
	header["process_markers_sha256"], _ = json.Marshal(recorded)
	raw, _ := json.Marshal(header)
	blob := binary.BigEndian.AppendUint32(nil, uint32(len(raw)))
	blob = append(append(blob, raw...), motion[4+length:]...)
	plan, err := densejointtraj.Decode(blob)
	if err != nil {
		t.Fatal(err)
	}
	return blob, fixtureIdentity(plan), digest
}

var gasOnOff = []densejointtraj.ProcessMarker{{Segment: 0, Sample: 0, Action: "gas", Value: true}, {Segment: 0, Sample: 5, Action: "gas", Value: false}}

func outputsFake(t *testing.T, blob []byte, io string) (*RTCoreController, *outputsRTCore) {
	t.Helper()
	r, f, _ := rtFake(t)
	f.mu.Lock()
	if io != "" {
		f.description.IO = json.RawMessage(io)
	}
	f.mu.Unlock()
	native := &outputsRTCore{fakeRTCore: f}
	r.c = native
	r.source.cfg.FetchBlob = func(context.Context, string) ([]byte, error) { return blob, nil }
	return r, native
}

func callsOf(f *outputsRTCore) []string {
	f.fakeRTCore.mu.Lock()
	defer f.fakeRTCore.mu.Unlock()
	return append([]string(nil), f.fakeRTCore.calls...)
}

// A production plan that switches outputs loads only bound to the plan
// record's schedule digest, with the exact bytes the planner made (markers
// kept), and the prepared identity the adapter derives from them. Play arms
// the outputs after Arm and before Start; the end of the program disarms them.
func TestRTCoreOutputPlanLoadsBoundToItsRecordAndArmsAroundPlay(t *testing.T) {
	blob, id, digest := outputPlan(t, gasOnOff)
	r, f := outputsFake(t, blob, simulationIO)
	ctx := context.Background()
	for _, refused := range []struct{ sent, code string }{
		{"", "process_markers_unbound"},
		{"sha256:" + strings.Repeat("0", 64), "process_markers_mismatch"},
	} {
		_, err := r.Load(ctx, LoadRequest{Identity: id, ProcessMarkersSHA256: refused.sent})
		if errorCode(err) != refused.code || !strings.Contains(err.Error(), "gas") {
			t.Fatalf("record digest %q: %v", refused.sent, err)
		}
	}
	if calls := callsOf(f); len(calls) != 0 {
		t.Fatalf("a refused output plan took authority first: %v", calls)
	}
	loaded, err := r.Load(ctx, LoadRequest{Identity: id, ProcessMarkersSHA256: digest})
	if err != nil {
		t.Fatal(err)
	}
	if s := loaded.Status.Session; s.DryRun || !reflect.DeepEqual(s.ProcessOutputs, []string{"gas"}) {
		t.Fatalf("session does not name its outputs: %+v", s)
	}
	f.captureMu.Lock()
	if len(f.blobs) != 1 || !bytes.Equal(f.blobs[0], blob) {
		t.Fatal("the production upload is not the planner's bytes")
	}
	f.captureMu.Unlock()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	if _, err := r.Play(ctx, loaded.SessionID); err != nil {
		t.Fatal(err)
	}
	sum := sha256.Sum256(blob)
	f.fakeRTCore.mu.Lock()
	started := f.fakeRTCore.started
	f.fakeRTCore.status.Execution.State = "completed"
	f.fakeRTCore.mu.Unlock()
	if started.SourceDigest != "sha256:"+hex.EncodeToString(sum[:]) || started.TrajectoryDigest != id.TrajectoryDigest {
		t.Fatalf("Start did not echo the envelope identity: %+v", started)
	}
	rtAwait(t, func() bool { return r.Status(ctx).Session.State == "done" })
	rtAwait(t, func() bool { c := callsOf(f); return c[len(c)-1] == "io_disarm" })
	want := []string{"acquire", "prepare_program", "enable", "arm", "io_arm", "start_program", "io_disarm"}
	if calls := callsOf(f); !reflect.DeepEqual(calls, want) {
		t.Fatalf("calls: %v", calls)
	}
	rtAwait(t, func() bool { r.mu.Lock(); defer r.mu.Unlock(); return r.ioArmed.Session == "" })
}

// Every output check that needs no robot refuses before any authority is
// taken, naming what is wrong.
func TestRTCoreOutputPlanRefusesWhatTheCellCannotSwitch(t *testing.T) {
	cases := []struct {
		name, io, code, words string
		markers               []densejointtraj.ProcessMarker
		headerDigest          string
	}{
		{"no io block", "", "io_not_configured", "no outputs configured", gasOnOff, ""},
		{"automatic indicator", strings.Replace(simulationIO, `"class":"process"`, `"class":"indicator"`, 1), "io_marker_invalid", "controller-owned indicator", gasOnOff, ""},
		{"unknown output", simulationIO, "io_marker_invalid", `"argon"`,
			[]densejointtraj.ProcessMarker{{Segment: 0, Sample: 0, Action: "argon", Value: true}}, ""},
		{"torch unqualified", simulationIO, "io_torch_unqualified", `torch output "torch"`,
			[]densejointtraj.ProcessMarker{{Segment: 0, Sample: 0, Action: "torch", Value: true}}, ""},
		{"header digest", simulationIO, "process_markers_mismatch", "does not hash", gasOnOff, "sha256:" + strings.Repeat("1", 64)},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			var blob []byte
			var id Identity
			var digest string
			if c.headerDigest != "" {
				blob, id, digest = outputPlan(t, c.markers, c.headerDigest)
			} else {
				blob, id, digest = outputPlan(t, c.markers)
			}
			r, f := outputsFake(t, blob, c.io)
			_, err := r.Load(context.Background(), LoadRequest{Identity: id, ProcessMarkersSHA256: digest})
			if errorCode(err) != c.code || !strings.Contains(err.Error(), c.words) {
				t.Fatalf("%v", err)
			}
			if calls := callsOf(f); len(calls) != 0 {
				t.Fatalf("refusal took authority first: %v", calls)
			}
		})
	}
}

// Run program refuses an unbound output plan in its robot-free preflight,
// before Arm or Go to plan start.
func TestRunProgramRefusesAnUnboundOutputPlanBeforeAnything(t *testing.T) {
	blob, id, _ := outputPlan(t, gasOnOff)
	r, f := outputsFake(t, blob, simulationIO)
	result, err := r.RunProgram(context.Background(), RunProgramRequest{LoadRequest: LoadRequest{Identity: id}, Fraction: 0.5})
	if errorCode(err) != "process_markers_unbound" || result.Refusal == nil || result.Refusal.Step != "load" {
		t.Fatalf("%v %+v", err, result)
	}
	if calls := callsOf(f); len(calls) != 0 {
		t.Fatalf("Run program moved before refusing: %v", calls)
	}
}

// An io_arm refusal the operator can fix (an open interlock) is said in
// words naming the input, and leaves the program loaded and the robot armed:
// nothing started and nothing was stopped. Play again succeeds once fixed.
func TestRTCoreIOArmRefusalKeepsTheProgramLoaded(t *testing.T) {
	blob, id, digest := outputPlan(t, gasOnOff)
	r, f := outputsFake(t, blob, simulationIO)
	ctx := context.Background()
	f.fakeRTCore.mu.Lock()
	f.ioArmRefuse = "io_fast_input_unsatisfied"
	f.fakeRTCore.status.Core.IO.Inputs = []ipcclient.IoInput{{Name: "consent", Fast: true, Valid: true, Value: false}}
	f.fakeRTCore.mu.Unlock()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	loaded, err := r.Load(ctx, LoadRequest{Identity: id, ProcessMarkersSHA256: digest})
	if err != nil {
		t.Fatal(err)
	}
	_, err = r.Play(ctx, loaded.SessionID)
	if errorCode(err) != "io_fast_input_unsatisfied" || !strings.Contains(err.Error(), "fast input consent is not satisfied") ||
		!strings.Contains(err.Error(), "still loaded") {
		t.Fatalf("io_arm refusal: %v", err)
	}
	for _, call := range callsOf(f) {
		if call == "start_program" || call == "stop" || call == "release" {
			t.Fatalf("a refused io_arm %s: %v", call, callsOf(f))
		}
	}
	status := r.statusSnapshot()
	if status.Session.State != "loaded" || status.Armed == nil || !*status.Armed {
		t.Fatalf("session or arm lost: %+v armed=%v", status.Session, status.Armed)
	}
	f.fakeRTCore.mu.Lock()
	f.ioArmRefuse = ""
	f.fakeRTCore.mu.Unlock()
	if _, err := r.Play(ctx, loaded.SessionID); err != nil {
		t.Fatalf("Play after the interlock closed: %v", err)
	}
}

// A Stop fences the outputs natively; the OLP forgets the armed grant and
// sends no io_disarm on a grant it no longer holds.
func TestRTCoreStopForgetsArmedOutputsWithoutDisarming(t *testing.T) {
	blob, id, digest := outputPlan(t, gasOnOff)
	r, f := outputsFake(t, blob, simulationIO)
	ctx := context.Background()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	loaded, err := r.Load(ctx, LoadRequest{Identity: id, ProcessMarkersSHA256: digest})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := r.Play(ctx, loaded.SessionID); err != nil {
		t.Fatal(err)
	}
	if _, err := r.Stop(ctx); err != nil {
		t.Fatal(err)
	}
	time.Sleep(4 * rtPollInterval)
	r.mu.Lock()
	armed := r.ioArmed
	r.mu.Unlock()
	if armed.Session != "" {
		t.Fatal("Stop left outputs recorded armed")
	}
	for _, call := range callsOf(f) {
		if call == "io_disarm" {
			t.Fatalf("io_disarm sent on a released grant: %v", callsOf(f))
		}
	}
}

// The dry run of an output plan strips the markers, needs no record digest
// (an old client may still check the motion) and arms nothing at Play.
func TestRTCoreOutputPlanDryRunArmsNothing(t *testing.T) {
	blob, id, _ := outputPlan(t, gasOnOff)
	r, f := outputsFake(t, blob, simulationIO)
	ctx := context.Background()
	loaded, err := r.Load(ctx, LoadRequest{Identity: id, DryRun: true})
	if err != nil {
		t.Fatalf("dry run of an output plan: %v", err)
	}
	if len(loaded.Status.Session.ProcessOutputs) != 0 {
		t.Fatal("a dry run names outputs")
	}
	f.captureMu.Lock()
	if len(f.blobs) != 1 || bytes.Contains(f.blobs[0], []byte(`"process_markers"`)) {
		t.Fatal("the dry run sent the output schedule to the controller")
	}
	f.captureMu.Unlock()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	if _, err := r.Play(ctx, loaded.SessionID); err != nil {
		t.Fatal(err)
	}
	for _, call := range callsOf(f) {
		if call == "io_arm" {
			t.Fatalf("a dry run armed outputs: %v", callsOf(f))
		}
	}
}

// A record that names a schedule for a plan without one is a different plan.
func TestRTCoreMotionOnlyPlanRefusesARecordWithASchedule(t *testing.T) {
	r, f, id := rtFake(t)
	native := &outputsRTCore{fakeRTCore: f}
	r.c = native
	_, err := r.Load(context.Background(), LoadRequest{Identity: id, ProcessMarkersSHA256: densejointtraj.ProcessMarkersDigest(gasOnOff)})
	if errorCode(err) != "process_markers_mismatch" {
		t.Fatalf("%v", err)
	}
	if calls := callsOf(native); len(calls) != 0 {
		t.Fatalf("refusal took authority first: %v", calls)
	}
}

func TestRTCoreInputOnlyPlanBindsWaitWithoutArmingOutputs(t *testing.T) {
	markers := []densejointtraj.ProcessMarker{{Segment: 0, Sample: 0, Action: "sensor", Value: true, TimeoutMS: 1000, StableMS: 20}}
	blob, id, digest := outputPlan(t, markers)
	r, f := outputsFake(t, blob, `{"torch_qualified":false,"inputs":[{"name":"sensor"}],"outputs":[]}`)
	ctx := context.Background()
	loaded, err := r.Load(ctx, LoadRequest{Identity: id, ProcessMarkersSHA256: digest})
	if err != nil {
		t.Fatal(err)
	}
	if len(r.Status(ctx).Session.ProcessOutputs) != 0 {
		t.Fatal("an input wait was mislabeled as an output")
	}
	if _, err = r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	if _, err = r.Play(ctx, loaded.SessionID); err != nil {
		t.Fatal(err)
	}
	for _, call := range callsOf(f) {
		if call == "io_arm" {
			t.Fatal("input-only plan armed outputs")
		}
	}
}

func TestInputMonitorGateDoesNotRequestOutputAuthority(t *testing.T) {
	markers := []densejointtraj.ProcessMarker{{Segment: 0, Sample: 0, Action: "consent", Value: true, Monitor: "start"}, {Segment: 0, Sample: 5, Action: "consent", Value: true, Monitor: "end"}}
	blob, _, digest := outputPlan(t, markers)
	plan, err := densejointtraj.Decode(blob)
	if err != nil {
		t.Fatal(err)
	}
	var description control.Description
	description.IO = json.RawMessage(simulationIO)
	outputs, err := processOutputsGate(plan.Header, LoadRequest{ProcessMarkersSHA256: digest}, description)
	if err != nil || len(outputs) != 0 {
		t.Fatalf("monitor requested output authority: %v %v", outputs, err)
	}
	plan.Header.ProcessMarkers[0].Action = "missing"
	plan.Header.ProcessMarkersSHA256 = densejointtraj.ProcessMarkersDigest(plan.Header.ProcessMarkers)
	_, err = processOutputsGate(plan.Header, LoadRequest{ProcessMarkersSHA256: plan.Header.ProcessMarkersSHA256}, description)
	if errorCode(err) != "io_wait_invalid" {
		t.Fatalf("unknown input not refused: %v", err)
	}
}
