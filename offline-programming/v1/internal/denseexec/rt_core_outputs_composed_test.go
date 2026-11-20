//go:build linux

package denseexec

import (
	"context"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"os"
	"reflect"
	"strings"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/densejointtraj"
	codec "rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// The composed machine: the 1430 cell's simulation (six A6-EC axes, whose simulated
// HM35 is the only Home the simulator answers; Load requires Home on every
// axis) with the generic cell-I/O fixture's io block after the last drive
// (rt-core config/cell-io/simulation.json: outputs gas, process class, and
// torch, torch class, each with a 200 ms expiry, an unverified fixture value;
// one fast input, consent). --sim-dio-inputs 1 holds consent satisfied; 0
// leaves it open.
func armWithCellIO(t *testing.T) func(map[string]any) {
	t.Helper()
	raw, err := os.ReadFile("../../../../rt-core/config/cell-io/simulation.json")
	if err != nil {
		t.Fatal(err)
	}
	var fixture struct {
		IO map[string]any `json:"io"`
	}
	if err := json.Unmarshal(raw, &fixture); err != nil || fixture.IO == nil {
		t.Fatalf("cell-I/O fixture io block: %v", err)
	}
	return func(config map[string]any) {
		fixture.IO["slave_position"] = len(config["axes"].([]any))
		config["io"] = fixture.IO
		config["_io_unverified"] = "OLP composed test: the cell-I/O simulation fixture's io block after rosie_1420_v1's six simulated drives."
	}
}

// outputScheduleComposed is "P1, gas ON, P2, gas OFF" as the planner exports
// it (weldplan/io_schedule.py): three stationary segments, 41 + 31 + 41
// samples at 10 ms (0.4 s, 0.3 s, 0.4 s); gas ON on segment 1's first sample,
// re-sent every 10 samples (half its 200 ms expiry on the plan's clock), OFF
// on segment 2's first sample, the instant segment 1 ends. The ON therefore
// lasts 300 ms, longer than one expiry: it only lasts that long if the
// refreshes reach the terminal.
var outputScheduleComposed = []densejointtraj.ProcessMarker{
	{Segment: 1, Sample: 0, Action: "gas", Value: true},
	{Segment: 1, Sample: 10, Action: "gas", Value: true},
	{Segment: 1, Sample: 20, Action: "gas", Value: true},
	{Segment: 2, Sample: 0, Action: "gas", Value: false},
}

// outputPlanComposed encodes that plan, held at the arm's joints, from the
// reference fixture's header (identity, limits) with the schedule, its
// planner digest and the machine's robot identity in the header, as the
// planner writes them.
func outputPlanComposed(t *testing.T, joints []float64, robot RobotCell) ([]byte, Identity, string) {
	t.Helper()
	source, _ := fixtureBlob(t)
	reference, err := codec.Decode(source)
	if err != nil {
		t.Fatal(err)
	}
	h := reference.Header
	const dt = 0.01
	counts := []int{41, 31, 41}
	kinds := []string{"freespace", "dwell", "freespace"}
	segments := make([]codec.Segment, len(counts))
	h.Segments = make([]codec.SegmentMeta, len(counts))
	h.Axes.AxisMask = 1<<len(joints) - 1
	total, duration := 0, 0.0
	for i, n := range counts {
		samples := make([]codec.Sample, n)
		for j := range samples {
			samples[j].TimeS = float64(j) * dt
			copy(samples[j].QRad[:], joints)
		}
		segments[i] = codec.Segment{Kind: kinds[i], SourceID: "io-composed-" + kinds[i], Samples: samples}
		h.Segments[i] = codec.SegmentMeta{Index: i, Kind: kinds[i], SourceID: segments[i].SourceID, SampleCount: n, DurationS: float64(n-1) * dt}
		total += n
		duration += float64(n-1) * dt
	}
	h.Sampling.DtS, h.Sampling.TotalSampleCount, h.Sampling.TotalDurationS = dt, total, duration
	h.ProcessMarkers = make([]codec.ProcessMarker, len(outputScheduleComposed))
	for i, m := range outputScheduleComposed {
		h.ProcessMarkers[i] = codec.ProcessMarker{Segment: m.Segment, Sample: m.Sample, Action: m.Action, Value: m.Value}
	}
	encoded, err := codec.Encode(h, segments)
	if err != nil {
		t.Fatal(err)
	}
	// rt-core's codec has no process_markers_sha256; the planner writes it.
	digest := densejointtraj.ProcessMarkersDigest(outputScheduleComposed)
	length := binary.BigEndian.Uint32(encoded[:4])
	var header map[string]json.RawMessage
	if err := json.Unmarshal(encoded[4:4+length], &header); err != nil {
		t.Fatal(err)
	}
	header["process_markers_sha256"], _ = json.Marshal(digest)
	header["robot_cell"], _ = json.Marshal(map[string]string{"model_id": robot.ModelID, "robot_description_sha256": robot.RobotDescriptionSHA256,
		"machine_planning_calibration_sha256": robot.MachinePlanningCalibrationSHA256, "machine_planning_calibration_source": "target"})
	raw, _ := json.Marshal(header)
	blob := binary.BigEndian.AppendUint32(nil, uint32(len(raw)))
	blob = append(append(blob, raw...), encoded[4+length:]...)
	plan, err := densejointtraj.Decode(blob)
	if err != nil {
		t.Fatal(err)
	}
	return blob, fixtureIdentity(plan), digest
}

func gasOf(s control.Status) (ipcclient.IoOutput, bool) {
	for _, o := range s.Core.IO.Outputs {
		if o.Name == "gas" {
			return o, true
		}
	}
	return ipcclient.IoOutput{}, false
}

// outputComposed starts the composed machine, homes it through the OLP (the
// simulator's HM35; Home is the step a loaded host breaks, so its bound is
// the weld-plan composed test's 120 s) and answers the controller, the plan
// held where Home left the arm, and the plan's identity and schedule digest.
func outputComposed(t *testing.T, inputs string) (*control.Client, control.Description, *RTCoreController, []byte, Identity, string) {
	t.Helper()
	c, d, _, _, cfg := rtComposedMachineFixture(t, false, "config/machines/simulation/simulation-rosie1420.json",
		[]string{"--sim-dio-inputs", inputs}, armWithCellIO(t))
	var blob []byte
	r, err := NewRTCore(cfg, Config{ControllerID: "offline-programming:io-composed", FetchBlob: func(context.Context, string) ([]byte, error) { return blob, nil }}, c)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	ctx, cancel := context.WithTimeout(context.Background(), 120*time.Second)
	defer cancel()
	if err := r.Home(ctx); err != nil {
		s, _ := c.Status(ctx)
		faults, _ := json.Marshal(s.Core.Recovery)
		t.Fatalf("simulated Home: %v; safety_fault_mask %d execution_fault_reasons %d bus %s recovery %s", err, s.Core.SafetyFaultMask, s.Core.ExecutionFaultReasons, s.Core.Bus, faults)
	}
	// Arm needs every coordinate trusted; a Home whose qualification failed
	// shows here. Bound: the weld-plan composed test's 30 s.
	for trusted := false; !trusted; time.Sleep(10 * time.Millisecond) {
		if ctx.Err() != nil {
			t.Fatal("axes not trusted after Home")
		}
		s, err := c.Status(ctx)
		trusted = err == nil && s.Core.SafetyFaultMask == 0
		for _, a := range s.Axes {
			trusted = trusted && a.PositionState == "trusted" && a.Readiness != "faulted"
		}
	}
	s, err := c.Status(ctx)
	if err != nil {
		t.Fatal(err)
	}
	joints := make([]float64, len(s.Axes))
	for i, a := range s.Axes {
		joints[i] = a.LogicalPosition
	}
	robot := robotCellOf(d)
	if !robot.Configured || !robot.Valid {
		t.Fatalf("the composed machine names no valid robot: %+v", robot)
	}
	blob, id, digest := outputPlanComposed(t, joints, robot)
	return c, d, r, blob, id, digest
}

// The OLP's whole output execution step against the real adapter and
// simulator: Load binds the plan record's schedule and prepares the exact
// bytes (the adapter's envelope identity), Play arms the outputs and starts,
// the schedule switches gas ON for 300 ms through two refreshes and OFF on
// its own marker, and the end of the program disarms the outputs.
func TestRTCoreOutputProgramComposed(t *testing.T) {
	c, d, r, blob, id, digest := outputComposed(t, "1")
	ctx, cancel := context.WithTimeout(context.Background(), 60*time.Second)
	defer cancel()
	if len(d.IO) == 0 || !strings.Contains(string(d.IO), `"gas"`) {
		t.Fatalf("the composed machine describes no gas output: %s", d.IO)
	}
	if _, err := r.Load(ctx, LoadRequest{Identity: id}); errorCode(err) != "process_markers_unbound" {
		t.Fatalf("an old client's Load: %v", err)
	}
	loaded, err := r.Load(ctx, LoadRequest{Identity: id, ProcessMarkersSHA256: digest})
	if err != nil {
		t.Fatalf("output plan Load: %v", err)
	}
	if !reflect.DeepEqual(loaded.Status.Session.ProcessOutputs, []string{"gas"}) {
		t.Fatalf("session outputs: %+v", loaded.Status.Session)
	}
	before, err := c.Status(ctx)
	if err != nil || before.Core.Armed != 0 || before.Core.IO.Armed {
		t.Fatalf("Load armed something: core armed %d io armed %v %v", before.Core.Armed, before.Core.IO.Armed, err)
	}
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	if _, err := r.Play(ctx, loaded.SessionID); err != nil {
		t.Fatalf("Play: %v", err)
	}
	var on, off ipcclient.IoOutput
	rtAwait(t, func() bool {
		_, _ = r.Heartbeat(loaded.SessionID)
		s, err := c.Status(ctx)
		if err != nil {
			return false
		}
		gas, ok := gasOf(s)
		if ok && gas.Commanded && gas.Readback && on.ChangedNS == 0 {
			on = gas
		}
		if ok && on.ChangedNS != 0 && !gas.Commanded && !gas.Intent {
			off = gas
		}
		return off.ChangedNS != 0
	})
	if !on.Intent || !on.Valid || on.ChangedNS < on.MarkerSampleNS || on.ChangedNS-on.MarkerSampleNS > 1000000 {
		t.Fatalf("gas ON delivery (bound: one 1 ms cycle after its sample): %+v", on)
	}
	// Unverified scheduling allowance: one 1 ms cycle plus the cell-I/O
	// fixture's 20 ms wake lateness, as rt-core's own expiry test allows.
	// Without the refreshes gas would expire 200 ms after ON; with them it
	// stays ON until the OFF marker 300 ms after.
	held := time.Duration(off.ChangedNS - on.ChangedNS)
	if held < 300*time.Millisecond-21*time.Millisecond || held > 300*time.Millisecond+21*time.Millisecond {
		t.Fatalf("gas held %v, the schedule says 300 ms (refreshes lost?): on=%+v off=%+v", held, on, off)
	}
	rtAwait(t, func() bool { _, _ = r.Heartbeat(loaded.SessionID); return r.Status(ctx).Session.State == "done" })
	actual, err := c.Status(ctx)
	sum := sha256.Sum256(blob)
	if err != nil || actual.Execution.Identity.SourceDigest != "sha256:"+hex.EncodeToString(sum[:]) || actual.Execution.Identity.TrajectoryDigest != id.TrajectoryDigest {
		t.Fatalf("native executed another identity: %+v %v", actual.Execution.Identity, err)
	}
	rtAwait(t, func() bool {
		_, _ = r.Heartbeat(loaded.SessionID)
		s, err := c.Status(ctx)
		return err == nil && !s.Core.IO.Armed
	})
	final, err := c.Status(ctx)
	if gas, _ := gasOf(final); err != nil || gas.Commanded || gas.Intent || final.Core.SafetyFaultMask != 0 || final.Core.Armed != 1 {
		t.Fatalf("after the program: io %+v faults %d armed %d %v", final.Core.IO, final.Core.SafetyFaultMask, final.Core.Armed, err)
	}
	if _, err := r.Stop(ctx); err != nil {
		t.Fatal(err)
	}
}

// With the fast input open, Play's io_arm is refused in words naming the
// input; nothing starts, the robot stays armed and the program loaded.
func TestRTCoreOutputProgramComposedRefusesAnOpenInterlock(t *testing.T) {
	c, _, r, _, id, digest := outputComposed(t, "0")
	ctx, cancel := context.WithTimeout(context.Background(), 60*time.Second)
	defer cancel()
	loaded, err := r.Load(ctx, LoadRequest{Identity: id, ProcessMarkersSHA256: digest})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	_, err = r.Play(ctx, loaded.SessionID)
	if errorCode(err) != "io_fast_input_unsatisfied" || !strings.Contains(err.Error(), "consent") {
		t.Fatalf("open interlock: %v", err)
	}
	s, err := c.Status(ctx)
	if err != nil || s.Execution.State == "executing" || s.Core.IO.Armed || s.Core.Armed != 1 {
		t.Fatalf("after the refusal: execution %q io armed %v core armed %d %v", s.Execution.State, s.Core.IO.Armed, s.Core.Armed, err)
	}
	if state := r.Status(ctx).Session.State; state != "loaded" {
		t.Fatalf("session %q, want loaded", state)
	}
	if _, err := r.Stop(ctx); err != nil {
		t.Fatal(err)
	}
}
