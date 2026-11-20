package main

import (
	"context"
	"encoding/base64"
	"encoding/binary"
	"encoding/json"
	"net/http"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
	codec "rosieos/rt-core/adapters/rosie/densejointtraj"
)

// testProgramBackend is a machine whose Load checks for an upload are
// scripted, and which may be running a program.
type testProgramBackend struct {
	controlBackend
	playing bool
	checked []string
	refuse  error
}

func (b *testProgramBackend) Status(ctx context.Context) *denseexec.StatusBody {
	body := b.controlBackend.Status(ctx)
	if b.playing {
		body.Session = &denseexec.SessionInfo{ID: "ds-planner", Kind: denseexec.SessionKindTrajectory, State: "playing"}
	}
	return body
}

func (b *testProgramBackend) CheckTestProgram(_ context.Context, digest string) ([]string, error) {
	b.checked = append(b.checked, digest)
	if b.refuse != nil {
		return nil, b.refuse
	}
	return []string{"start within completion_tolerance of the measured pose on every axis (worst: J4 off by 0.003 deg of 0.034 deg)"}, nil
}

// testProgramUploadBody is an upload of a stationary six-axis program that
// names its robot, and optionally a plan.json describing it.
func testProgramUploadBody(t *testing.T, withPlan bool, editPlan func(map[string]any)) (string, string) {
	t.Helper()
	seg := codec.Segment{Kind: "dwell", SourceID: "lead"}
	for k := 0; k < 10; k++ {
		seg.Samples = append(seg.Samples, codec.Sample{TimeS: float64(k) * 0.01, QRad: [codec.AxisCount]float64{0, 0.35, -0.17, 0.2, 0.52}})
	}
	limits := []float64{1, 1, 1, 1, 1, 1, 1, 1, 1}
	h := codec.Header{Schema: codec.Schema, PlanID: "tuningreport-inertia-J4:0123456789ab", ProgramID: "tuningreport-inertia-J4",
		ProgramDigest: "sha256:" + strings.Repeat("7", 64), ManifestRevision: 1, PlanRevision: 1,
		Axes:     codec.HeaderAxes{AxisCount: codec.AxisCount, AxisMask: 0x3f, PositionUnit: "rad", VelocityUnit: "rad_s"},
		Sampling: codec.Sampling{DtS: 0.01, TotalSampleCount: 10, TotalDurationS: 0.09}, Limits: codec.Limits{MaxAbsQdRadS: limits},
		Segments:       []codec.SegmentMeta{{Index: 0, Kind: "dwell", SourceID: "lead", SampleCount: 10, DurationS: 0.09}},
		SampleEncoding: codec.EncodingMeta{Encoding: codec.SampleEncoding, RecordBytes: codec.RecordBytes}}
	blob, err := codec.Encode(h, []codec.Segment{seg})
	if err != nil {
		t.Fatal(err)
	}
	n := int(binary.BigEndian.Uint32(blob[:4]))
	var header map[string]json.RawMessage
	if err := json.Unmarshal(blob[4:4+n], &header); err != nil {
		t.Fatal(err)
	}
	header["robot_cell"], _ = json.Marshal(map[string]string{"model_id": "rosie_1420_v1", "robot_description_sha256": "sha256:" + strings.Repeat("1", 64),
		"machine_planning_calibration_sha256": "sha256:" + strings.Repeat("2", 64), "machine_planning_calibration_source": "target",
		"machine_configuration_sha256": "sha256:" + strings.Repeat("9", 64)})
	raw, _ := json.Marshal(header)
	blob = append(append(binary.BigEndian.AppendUint32(nil, uint32(len(raw))), raw...), blob[4+n:]...)
	decoded, err := codec.Decode(blob)
	if err != nil {
		t.Fatal(err)
	}
	upload := map[string]any{"file_name": "inertia-j4-0123456789ab.rdt", "program_base64": base64.StdEncoding.EncodeToString(blob)}
	if withPlan {
		plan := map[string]any{"schema": "rosie.tuningreport.inertia-plan.v1", "joint": "J4", "duration_s": 24.4,
			"trajectory_digest": decoded.Header.TrajectoryDigest, "plan_id": h.PlanID, "program_id": h.ProgramID, "program_digest": h.ProgramDigest}
		if editPlan != nil {
			editPlan(plan)
		}
		upload["plan_json"], upload["plan_file_name"] = plan, "plan.json"
	}
	body, _ := json.Marshal(upload)
	return string(body), decoded.Header.TrajectoryDigest
}

func testProgramServer(t *testing.T, enabled bool) (*Server, *testProgramBackend) {
	t.Helper()
	backend := &testProgramBackend{controlBackend: controlBackend{controllerID: "offline-programming:server-olp001"}}
	options := ServerOptions{DenseExecution: backend}
	if enabled {
		options.TestPrograms = denseexec.NewTestPrograms()
	}
	server := NewServer(options)
	clock := time.Date(2026, 10, 9, 23, 0, 0, 0, time.UTC)
	server.control.now = func() time.Time { return clock }
	server.clients.now = func() time.Time { return clock }
	return server, backend
}

const testProgramPath = denseExecutionPath + "/test-program"

// Off unless the server was started with test programs on: the route says so
// and refuses, and nothing is checked against the cell.
func TestTestProgramRouteIsOffByDefault(t *testing.T) {
	server, backend := testProgramServer(t, false)
	response, body := controlRequest(t, server, browserTab, http.MethodGet, testProgramPath, "")
	if response.Code != http.StatusOK || body["enabled"] != false {
		t.Fatalf("GET: %d %v", response.Code, body)
	}
	upload, _ := testProgramUploadBody(t, false, nil)
	response, body = controlRequest(t, server, browserTab, http.MethodPost, testProgramPath, upload)
	if response.Code != http.StatusForbidden || body["error"] != "test_programs_disabled" || len(backend.checked) != 0 {
		t.Fatalf("POST: %d %v checked=%v", response.Code, body, backend.checked)
	}
}

// An accepted upload answers with the program's identity, its provenance
// and the cell's checks, and is listed for every client.
func TestTestProgramUploadAnswersIdentityProvenanceAndChecks(t *testing.T) {
	server, backend := testProgramServer(t, true)
	upload, digest := testProgramUploadBody(t, true, nil)
	response, body := controlRequest(t, server, browserTab, http.MethodPost, testProgramPath, upload)
	if response.Code != http.StatusOK {
		t.Fatalf("POST: %d %v", response.Code, body)
	}
	program := body["program"].(map[string]any)
	provenance, _ := program["provenance"].(map[string]any)
	if program["trajectory_digest"] != digest || program["source"] != "test_program" || program["file_name"] != "inertia-j4-0123456789ab.rdt" ||
		program["plan_id"] != "tuningreport-inertia-J4:0123456789ab" || provenance["joint"] != "J4" || provenance["file_name"] != "plan.json" {
		t.Fatalf("program %v", program)
	}
	if checks := body["checks"].([]any); len(checks) != 1 || len(backend.checked) != 1 || backend.checked[0] != digest {
		t.Fatalf("checks %v, checked %v", checks, backend.checked)
	}
	_, list := controlRequest(t, server, deckClient, http.MethodGet, testProgramPath, "")
	if programs := list["programs"].([]any); list["enabled"] != true || len(programs) != 1 || programs[0].(map[string]any)["trajectory_digest"] != digest {
		t.Fatalf("list %v", list)
	}
}

// Refusals: a running program, the cell's checks, a plan.json of another
// program, bytes that are not base64. A refused upload is not kept.
func TestTestProgramUploadRefusals(t *testing.T) {
	for _, tc := range []struct {
		name, want string
		code       int
		setup      func(b *testProgramBackend)
		plan       func(map[string]any)
		raw        string
	}{
		{name: "a program is running", want: "program_already_executing", code: http.StatusConflict, setup: func(b *testProgramBackend) { b.playing = true }},
		{name: "cell check refuses", want: "test_program_start_mismatch", code: http.StatusConflict, setup: func(b *testProgramBackend) {
			b.refuse = &denseexec.Error{Status: http.StatusConflict, Code: "test_program_start_mismatch", Detail: "J4 is at 11.959 deg, the program starts at 11.459 deg (tolerance 0.034 deg)"}
		}},
		{name: "plan.json of another program", want: "test_program_provenance_mismatch", code: http.StatusConflict, plan: func(m map[string]any) { m["program_digest"] = "sha256:" + strings.Repeat("0", 64) }},
		{name: "not base64", want: "test_program_invalid", code: http.StatusBadRequest, raw: `{"file_name":"x.rdt","program_base64":"%%%"}`},
	} {
		t.Run(tc.name, func(t *testing.T) {
			server, backend := testProgramServer(t, true)
			if tc.setup != nil {
				tc.setup(backend)
			}
			upload, _ := testProgramUploadBody(t, tc.plan != nil, tc.plan)
			if tc.raw != "" {
				upload = tc.raw
			}
			response, body := controlRequest(t, server, browserTab, http.MethodPost, testProgramPath, upload)
			if response.Code != tc.code || body["error"] != tc.want {
				t.Fatalf("got %d %v, want %d %s", response.Code, body, tc.code, tc.want)
			}
			if programs := server.testPrograms.List(); len(programs) != 0 {
				t.Fatalf("a refused upload was kept: %v", programs)
			}
		})
	}
}

// The upload is a control request like Load: refused while another client
// has the robot armed, accepted from that owner.
func TestTestProgramUploadIsRefusedWhileAnotherClientHasTheRobot(t *testing.T) {
	server, backend := testProgramServer(t, true)
	if response, body := controlRequest(t, server, browserTab, http.MethodPost, denseExecutionPath+"/arm", `{"armed":true}`); response.Code != http.StatusOK {
		t.Fatalf("arm: %d %v", response.Code, body)
	}
	upload, _ := testProgramUploadBody(t, false, nil)
	response, body := controlRequest(t, server, deckClient, http.MethodPost, testProgramPath, upload)
	if response.Code != http.StatusConflict || body["error"] != controlRefusal || len(backend.checked) != 0 {
		t.Fatalf("other client: %d %v checked=%v", response.Code, body, backend.checked)
	}
	if response, body := controlRequest(t, server, browserTab, http.MethodPost, testProgramPath, upload); response.Code != http.StatusOK {
		t.Fatalf("owner: %d %v", response.Code, body)
	}
}

// The maintenance notice refuses the upload like every robot action.
func TestTestProgramUploadIsRefusedDuringMaintenance(t *testing.T) {
	path := filepath.Join(t.TempDir(), "maintenance.json")
	if err := os.WriteFile(path, []byte(activeNotice), 0o644); err != nil {
		t.Fatal(err)
	}
	source := NewMaintenanceSource(path, 2*time.Hour)
	source.now = func() time.Time { return maintenanceTestNow }
	backend := &testProgramBackend{}
	server := NewServer(ServerOptions{DenseExecution: backend, Maintenance: source, TestPrograms: denseexec.NewTestPrograms()})
	upload, _ := testProgramUploadBody(t, false, nil)
	response, body := maintenanceRequest(t, server, http.MethodPost, testProgramPath, upload)
	if response.Code != http.StatusServiceUnavailable || body["error"] != "maintenance" || len(backend.checked) != 0 {
		t.Fatalf("maintenance: %d %v", response.Code, body)
	}
}
