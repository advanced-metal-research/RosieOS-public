package denseexec

import (
	"context"
	"encoding/binary"
	"encoding/json"
	"errors"
	"fmt"
	"strings"
	"testing"
	"time"

	codec "rosieos/rt-core/adapters/rosie/densejointtraj"
)

// testProgramRobot is a robot_cell block as tuningreport inertia-plan stamps it.
func testProgramRobot() map[string]string {
	return map[string]string{
		"model_id":                            "cell_a",
		"robot_description_sha256":            "sha256:" + strings.Repeat("1", 64),
		"machine_planning_calibration_sha256": "sha256:" + strings.Repeat("2", 64),
		"machine_planning_calibration_source": "target",
		"machine_configuration_sha256":        "sha256:" + strings.Repeat("9", 64),
	}
}

// testProgramBlob is a stationary 0.19 s program holding start on the axes
// of mask, for binding revision 7 (rtFake's), encoded by rt-core's codec (the generator's) with robot_cell
// stamped into the header the way the generator does; nil robot stamps none.
func testProgramBlob(t *testing.T, mask uint32, start [codec.AxisCount]float64, robot map[string]string, edit func(*codec.Header, []codec.Segment)) []byte {
	t.Helper()
	seg := codec.Segment{Kind: "dwell", SourceID: "lead"}
	for k := 0; k < 20; k++ {
		seg.Samples = append(seg.Samples, codec.Sample{TimeS: float64(k) * 0.01, QRad: start})
	}
	limits := make([]float64, codec.AxisCount)
	for i := range limits {
		limits[i] = 1
	}
	h := codec.Header{Schema: codec.Schema, PlanID: "tuningreport-inertia-J4:0123456789ab", ProgramID: "tuningreport-inertia-J4",
		ProgramDigest: "sha256:" + strings.Repeat("7", 64), ManifestRevision: 7, PlanRevision: 1, CreatedAt: "2026-10-09T23:00:00Z",
		Axes:           codec.HeaderAxes{AxisCount: codec.AxisCount, AxisMask: mask, PositionUnit: "rad", VelocityUnit: "rad_s"},
		Sampling:       codec.Sampling{DtS: 0.01, TotalSampleCount: 20, TotalDurationS: 0.19},
		Limits:         codec.Limits{MaxAbsQdRadS: limits},
		Segments:       []codec.SegmentMeta{{Index: 0, Kind: "dwell", SourceID: "lead", SampleCount: 20, DurationS: 0.19}},
		SampleEncoding: codec.EncodingMeta{Encoding: codec.SampleEncoding, RecordBytes: codec.RecordBytes}}
	segs := []codec.Segment{seg}
	if edit != nil {
		edit(&h, segs)
	}
	blob, err := codec.Encode(h, segs)
	if err != nil {
		t.Fatal(err)
	}
	if robot == nil {
		return blob
	}
	n := int(binary.BigEndian.Uint32(blob[:4]))
	var header map[string]json.RawMessage
	if err := json.Unmarshal(blob[4:4+n], &header); err != nil {
		t.Fatal(err)
	}
	header["robot_cell"], _ = json.Marshal(robot)
	raw, _ := json.Marshal(header)
	out := binary.BigEndian.AppendUint32(nil, uint32(len(raw)))
	return append(append(out, raw...), blob[4+n:]...)
}

func testProgramPlanJSON(t *testing.T, p *TestProgram, edit func(map[string]any)) []byte {
	t.Helper()
	plan := map[string]any{
		"schema": "rosie.tuningreport.inertia-plan.v1", "generated": "2026-10-09T23:00:00Z", "joint": "J4",
		"program_file": "inertia-j4-abc.rdt", "duration_s": 24.4, "window_deg": []float64{-15.9, 15.9}, "peak_speed_deg_s": 60.0,
		"levels":            []map[string]float64{{"accel_deg_s2": 200}, {"accel_deg_s2": 400}},
		"config":            map[string]string{"configuration_sha256": strings.Repeat("9", 64)},
		"trajectory_digest": p.TrajectoryDigest, "plan_id": p.PlanID, "program_id": p.ProgramID, "program_digest": p.ProgramDigest,
		"checks": []string{"format ok"},
	}
	if edit != nil {
		edit(plan)
	}
	raw, err := json.Marshal(plan)
	if err != nil {
		t.Fatal(err)
	}
	return raw
}

func errorCode(err error) string {
	var typed *Error
	if errors.As(err, &typed) {
		return typed.Code
	}
	return fmt.Sprintf("untyped: %v", err)
}

var testProgramNow = time.Date(2026, 10, 9, 23, 30, 0, 0, time.UTC)

// An upload keeps its identity, a digest of the exact file, its start pose in
// the program's own axes, and the generator's description only when that
// description names this program.
func TestTestProgramKeepsIdentityAndProvenance(t *testing.T) {
	start := [codec.AxisCount]float64{0, 0.35, -0.17, 0.1, 0.52, 0}
	blob := testProgramBlob(t, 0x3f, start, testProgramRobot(), nil)
	store := NewTestPrograms()
	bare, err := newTestProgram(blob, "x.rdt", nil, "", "", testProgramNow)
	if err != nil {
		t.Fatal(err)
	}
	p, err := store.Add(blob, "/Users/op/inertia-plans/inertia-j4-abc.rdt", testProgramPlanJSON(t, bare, nil), "plan.json", "Chrome on macOS", testProgramNow)
	if err != nil {
		t.Fatal(err)
	}
	if p.Source != TestProgramSource || p.FileName != "inertia-j4-abc.rdt" || p.Bytes != len(blob) || !strings.HasPrefix(p.FileSHA256, "sha256:") || len(p.FileSHA256) != 71 {
		t.Fatalf("label: %+v", p)
	}
	if p.Robot.MachineConfigurationSHA256 != testProgramRobot()["machine_configuration_sha256"] || p.Robot.ModelID != "cell_a" {
		t.Fatalf("robot: %+v", p.Robot)
	}
	if len(p.StartRad) != 6 || p.StartRad[1] != 0.35 || p.StartRad[4] != 0.52 {
		t.Fatalf("start %v; want the six masked axes of the first sample", p.StartRad)
	}
	if p.Trajectory.TotalSampleCount != 20 || p.Trajectory.TotalDurationS != 0.19 {
		t.Fatalf("summary %+v", p.Trajectory)
	}
	pv := p.Provenance
	if pv == nil || pv.FileName != "plan.json" || pv.Joint != "J4" || pv.DurationS != 24.4 || len(pv.AccelLevelsDegS2) != 2 || pv.AccelLevelsDegS2[1] != 400 ||
		pv.ConfigurationSHA256 != strings.Repeat("9", 64) || len(pv.WindowDeg) != 2 || pv.WindowDeg[1] != 15.9 {
		t.Fatalf("provenance %+v", pv)
	}
	if got := store.List(); len(got) != 1 || got[0].TrajectoryDigest != p.TrajectoryDigest {
		t.Fatalf("list %+v", got)
	}
}

// What needs no cell to refuse is refused at upload, with a typed reason.
func TestTestProgramRefusesBeforeAnyCell(t *testing.T) {
	start := [codec.AxisCount]float64{}
	good := testProgramBlob(t, 0x3f, start, testProgramRobot(), nil)
	ref, err := newTestProgram(good, "p.rdt", nil, "", "", testProgramNow)
	if err != nil {
		t.Fatal(err)
	}
	noConfig := testProgramRobot()
	delete(noConfig, "machine_configuration_sha256")
	torch := func(h *codec.Header, segs []codec.Segment) {
		h.Segments[0].Kind, segs[0].Kind = "weld", "weld"
		segs[0].Samples[5].TorchOn = true
	}
	for _, tc := range []struct {
		name, want string
		blob, plan []byte
	}{
		{"empty", "test_program_invalid", nil, nil},
		{"not a program", "test_program_invalid", []byte("not a dense trajectory at all"), nil},
		{"truncated", "test_program_invalid", good[:len(good)-7], nil},
		{"no robot_cell", "test_program_identity_missing", testProgramBlob(t, 0x3f, start, nil, nil), nil},
		{"robot_cell without configuration", "test_program_identity_missing", testProgramBlob(t, 0x3f, start, noConfig, nil), nil},
		{"process output", "test_program_process_outputs", testProgramBlob(t, 0x3f, start, testProgramRobot(), torch), nil},
		{"plan.json of another program", "test_program_provenance_mismatch", good, testProgramPlanJSON(t, ref, func(m map[string]any) { m["trajectory_digest"] = "sha256:" + strings.Repeat("0", 64) })},
		{"plan.json of another plan id", "test_program_provenance_mismatch", good, testProgramPlanJSON(t, ref, func(m map[string]any) { m["plan_id"] = "other" })},
		{"plan.json without schema", "test_program_provenance_invalid", good, testProgramPlanJSON(t, ref, func(m map[string]any) { delete(m, "schema") })},
		{"plan.json not JSON", "test_program_provenance_invalid", good, []byte("{nope")},
	} {
		t.Run(tc.name, func(t *testing.T) {
			store := NewTestPrograms()
			_, err := store.Add(tc.blob, "p.rdt", tc.plan, "plan.json", "", testProgramNow)
			if code := errorCode(err); code != tc.want {
				t.Fatalf("got %s (%v), want %s", code, err, tc.want)
			}
			if len(store.List()) != 0 {
				t.Fatal("a refused upload was kept")
			}
		})
	}
}

// Off means off: a nil store refuses uploads and Loads that name the source.
func TestTestProgramsOffRefuse(t *testing.T) {
	var store *TestPrograms
	blob := testProgramBlob(t, 0x3f, [codec.AxisCount]float64{}, testProgramRobot(), nil)
	if store.Enabled() || store.List() != nil {
		t.Fatal("nil store reads as enabled")
	}
	if _, err := store.Add(blob, "p.rdt", nil, "", "", testProgramNow); errorCode(err) != "test_programs_disabled" {
		t.Fatalf("add: %v", err)
	}
	source := &blobSource{cfg: Config{}}
	if _, _, _, err := source.loadBlob(context.Background(), LoadRequest{Source: TestProgramSource}); errorCode(err) != "test_programs_disabled" {
		t.Fatalf("load: %v", err)
	}
}

// The store keeps the newest few and never duplicates one program.
func TestTestProgramStoreIsBounded(t *testing.T) {
	store := NewTestPrograms()
	var digests []string
	for i := 0; i < maxTestPrograms+2; i++ {
		start := [codec.AxisCount]float64{float64(i) * 0.01}
		p, err := store.Add(testProgramBlob(t, 0x3f, start, testProgramRobot(), nil), "p.rdt", nil, "", "", testProgramNow)
		if err != nil {
			t.Fatal(err)
		}
		digests = append(digests, p.TrajectoryDigest)
	}
	if _, err := store.Add(testProgramBlob(t, 0x3f, [codec.AxisCount]float64{0.05}, testProgramRobot(), nil), "again.rdt", nil, "", "", testProgramNow); err != nil {
		t.Fatal(err)
	}
	list := store.List()
	if len(list) != maxTestPrograms || list[0].TrajectoryDigest != digests[5] || list[0].FileName != "again.rdt" || list[1].TrajectoryDigest != digests[4] {
		t.Fatalf("kept %d, newest %s %s", len(list), list[0].TrajectoryDigest, list[0].FileName)
	}
	if _, err := store.get(digests[0]); errorCode(err) != "test_program_not_found" {
		t.Fatalf("oldest still kept: %v", err)
	}
	store.Remove(digests[5])
	if _, err := store.get(digests[5]); errorCode(err) != "test_program_not_found" {
		t.Fatal("remove kept the program")
	}
}

// A planner Load reads the planner by digest even when an upload has the
// same digest; a test-program Load reads the upload only, by its identity.
func TestLoadBlobKeepsPlannerAndTestProgramApart(t *testing.T) {
	store := NewTestPrograms()
	blob := testProgramBlob(t, 0x3f, [codec.AxisCount]float64{}, testProgramRobot(), nil)
	p, err := store.Add(blob, "p.rdt", nil, "", "", testProgramNow)
	if err != nil {
		t.Fatal(err)
	}
	planner := []byte("planner bytes")
	fetched := 0
	source := &blobSource{cfg: Config{TestPrograms: store, FetchBlob: func(_ context.Context, digest string) ([]byte, error) {
		fetched++
		return planner, nil
	}}}
	got, program, _, err := source.loadBlob(context.Background(), LoadRequest{Identity: p.Identity})
	if err != nil || string(got) != string(planner) || program != nil || fetched != 1 {
		t.Fatalf("planner load: %q %v %v fetched=%d", got, program, err, fetched)
	}
	got, program, _, err = source.loadBlob(context.Background(), LoadRequest{Identity: p.Identity, Source: TestProgramSource})
	if err != nil || string(got) != string(blob) || program == nil || program.FileSHA256 != p.FileSHA256 || fetched != 1 {
		t.Fatalf("test program load: %v %v fetched=%d", program, err, fetched)
	}
	other := p.Identity
	other.PlanRevision++
	if _, _, _, err := source.loadBlob(context.Background(), LoadRequest{Identity: other, Source: TestProgramSource}); errorCode(err) != "dense_identity_mismatch" {
		t.Fatalf("identity mismatch: %v", err)
	}
	if _, _, _, err := source.loadBlob(context.Background(), LoadRequest{Identity: p.Identity, Source: TestProgramSource, DryRun: true}); errorCode(err) != "test_program_dry_run" {
		t.Fatalf("dry run: %v", err)
	}
	if _, _, _, err := source.loadBlob(context.Background(), LoadRequest{Identity: p.Identity, Source: "file"}); errorCode(err) != "dense_load_source_invalid" {
		t.Fatalf("unknown source: %v", err)
	}
}
