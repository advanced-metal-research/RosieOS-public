package denseexec

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"net/http"
	"strings"
	"sync"
	"time"

	"rosieos/offline-programming/v1/internal/densejointtraj"
)

// TestProgramSource is LoadRequest.Source for a tool-generated dense program
// the operator uploaded (tuningreport inertia-plan), not a planner artifact.
// An empty Source is the planner path, byte for byte as before.
const TestProgramSource = "test_program"

// MaxTestProgramBytes bounds one uploaded program. Unverified sizing: the
// inertia test is capped at 90 s (a Stop dump holds 100 s), 9,000 samples of
// 145 bytes is 1.3 MB; 16 MiB leaves room for a longer test and refuses a
// planner-sized blob that has no business on this route.
const MaxTestProgramBytes = 16 << 20

// maxTestPrograms is how many uploads are kept, newest first: enough for two
// tabs to upload and load in either order, small enough that nothing piles up.
const maxTestPrograms = 4

// TestProgram is an uploaded program as every client sees it: the identity
// Load must name, where it came from, and what it does. The bytes stay here.
type TestProgram struct {
	Identity
	Source     string                 `json:"source"`
	FileName   string                 `json:"file_name"`
	FileSHA256 string                 `json:"file_sha256"`
	Bytes      int                    `json:"bytes"`
	UploadedAt time.Time              `json:"uploaded_at"`
	UploadedBy string                 `json:"uploaded_by,omitempty"`
	Robot      densejointtraj.Robot   `json:"robot_cell"`
	StartRad   []float64              `json:"start_rad"`
	Trajectory TrajectorySummary      `json:"trajectory"`
	Provenance *TestProgramProvenance `json:"provenance,omitempty"`
	blob       []byte
	plan       *densejointtraj.Plan
}

// TestProgramProvenance is what the generator's plan.json says about the
// program, kept only when that file names exactly this program.
type TestProgramProvenance struct {
	FileName            string    `json:"file_name"`
	Schema              string    `json:"schema"`
	Generated           string    `json:"generated,omitempty"`
	Joint               string    `json:"joint,omitempty"`
	ProgramFile         string    `json:"program_file,omitempty"`
	DurationS           float64   `json:"duration_s,omitempty"`
	WindowDeg           []float64 `json:"window_deg,omitempty"`
	PeakSpeedDegS       float64   `json:"peak_speed_deg_s,omitempty"`
	AccelLevelsDegS2    []float64 `json:"accel_levels_deg_s2,omitempty"`
	ConfigurationSHA256 string    `json:"configuration_sha256,omitempty"`
	Checks              []string  `json:"checks,omitempty"`
}

// TestProgramLabel marks a session loaded from an upload, so the status every
// client polls says it is not a planner program and which file it is.
type TestProgramLabel struct {
	FileName   string                 `json:"file_name"`
	FileSHA256 string                 `json:"file_sha256"`
	Provenance *TestProgramProvenance `json:"provenance,omitempty"`
}

// TestPrograms holds uploads by trajectory digest. Nil means test programs
// are off on this server (OFFLINE_PROGRAMMING_TEST_PROGRAMS), and every
// route and Load naming the source refuses.
type TestPrograms struct {
	mu       sync.Mutex
	programs []*TestProgram
}

func NewTestPrograms() *TestPrograms { return &TestPrograms{} }

// Enabled is false for a nil store.
func (s *TestPrograms) Enabled() bool { return s != nil }

// List is the kept programs, newest first.
func (s *TestPrograms) List() []TestProgram {
	if s == nil {
		return nil
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	out := make([]TestProgram, 0, len(s.programs))
	for _, p := range s.programs {
		out = append(out, *p)
	}
	return out
}

func (s *TestPrograms) get(digest string) (*TestProgram, error) {
	if s == nil {
		return nil, httpError(http.StatusForbidden, "test_programs_disabled",
			"test programs are off on this server (OFFLINE_PROGRAMMING_TEST_PROGRAMS)")
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	for _, p := range s.programs {
		if p.TrajectoryDigest == digest {
			return p, nil
		}
	}
	return nil, httpError(http.StatusNotFound, "test_program_not_found",
		"no uploaded test program %s on this server; load the .rdt file again", digest)
}

// Remove drops an upload (one that failed its cell check).
func (s *TestPrograms) Remove(digest string) {
	if s == nil {
		return
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	for i, p := range s.programs {
		if p.TrajectoryDigest == digest {
			s.programs = append(s.programs[:i], s.programs[i+1:]...)
			return
		}
	}
}

// Add validates an upload offline and keeps it. What it refuses here needs
// no cell: a blob that is not a valid dense program, one with process
// outputs, one that does not say which robot and configuration it was made
// for, or a plan.json that describes another program. The cell's own checks
// (robot, configuration, start pose, native limits) run at Load, as for
// planner artifacts; CheckTestProgram runs them early for the upload answer.
func (s *TestPrograms) Add(blob []byte, fileName string, planJSON []byte, planFileName, uploadedBy string, now time.Time) (*TestProgram, error) {
	if s == nil {
		return nil, httpError(http.StatusForbidden, "test_programs_disabled",
			"test programs are off on this server (OFFLINE_PROGRAMMING_TEST_PROGRAMS)")
	}
	p, err := newTestProgram(blob, fileName, planJSON, planFileName, uploadedBy, now)
	if err != nil {
		return nil, err
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	kept := []*TestProgram{p}
	for _, old := range s.programs {
		if old.TrajectoryDigest != p.TrajectoryDigest && len(kept) < maxTestPrograms {
			kept = append(kept, old)
		}
	}
	s.programs = kept
	copied := *p
	return &copied, nil
}

func newTestProgram(blob []byte, fileName string, planJSON []byte, planFileName, uploadedBy string, now time.Time) (*TestProgram, error) {
	if len(blob) == 0 || len(blob) > MaxTestProgramBytes {
		return nil, httpError(http.StatusBadRequest, "test_program_invalid", "the program is %d bytes; it must be 1..%d", len(blob), MaxTestProgramBytes)
	}
	plan, err := densejointtraj.Decode(blob)
	if err != nil {
		return nil, httpError(http.StatusBadRequest, "test_program_invalid", "%v", err)
	}
	h := plan.Header
	if err := validateIdentity(Identity{TrajectoryDigest: h.TrajectoryDigest, PlanID: h.PlanID, ProgramID: h.ProgramID, ProgramDigest: h.ProgramDigest}); err != nil {
		return nil, httpError(http.StatusBadRequest, "test_program_invalid", "%v", err)
	}
	if h.Robot == nil || h.Robot.MachineConfigurationSHA256 == "" {
		return nil, httpError(http.StatusConflict, "test_program_identity_missing",
			"the program does not name the robot and machine configuration it was checked against (header robot_cell with machine_configuration_sha256); regenerate it with a tuningreport that stamps them")
	}
	if len(plan.Segments) == 0 || len(plan.Segments[0].Samples) == 0 {
		return nil, httpError(http.StatusBadRequest, "test_program_invalid", "the program has no samples")
	}
	for si, segment := range plan.Segments {
		for k, sample := range segment.Samples {
			if sample.TorchOn {
				return nil, httpError(http.StatusConflict, "test_program_process_outputs",
					"segment %d sample %d switches a process output; a test program moves joints only", si, k)
			}
		}
	}
	if markers, err := headerHasProcessMarkers(blob); err != nil || markers {
		return nil, httpError(http.StatusConflict, "test_program_process_outputs", "the program carries process markers; a test program moves joints only")
	}
	sum := sha256.Sum256(blob)
	p := &TestProgram{
		Identity:   Identity{TrajectoryDigest: h.TrajectoryDigest, PlanID: h.PlanID, ProgramID: h.ProgramID, ProgramDigest: h.ProgramDigest, ManifestRevision: h.ManifestRevision, PlanRevision: h.PlanRevision},
		Source:     TestProgramSource,
		FileName:   cleanFileName(fileName),
		FileSHA256: "sha256:" + hex.EncodeToString(sum[:]),
		Bytes:      len(blob),
		UploadedAt: now.UTC(),
		UploadedBy: uploadedBy,
		Robot:      *h.Robot,
		Trajectory: summarize(h),
		blob:       append([]byte(nil), blob...),
		plan:       plan,
	}
	first := plan.Segments[0].Samples[0]
	for i := 0; i < densejointtraj.AxisCount; i++ {
		if h.Axes.AxisMask&(1<<i) != 0 {
			p.StartRad = append(p.StartRad, first.QRad[i])
		}
	}
	if len(bytes.TrimSpace(planJSON)) > 0 {
		provenance, err := testProgramProvenance(planJSON, p.Identity)
		if err != nil {
			return nil, err
		}
		provenance.FileName = cleanFileName(planFileName)
		p.Provenance = provenance
	}
	return p, nil
}

// testProgramProvenance reads a generator's plan.json. It must name this
// program's trajectory, plan, program and program digest; anything else is
// another program's description and is refused, never shown beside this one.
func testProgramProvenance(raw []byte, id Identity) (*TestProgramProvenance, error) {
	var plan struct {
		Schema           string    `json:"schema"`
		Generated        string    `json:"generated"`
		Joint            string    `json:"joint"`
		Program          string    `json:"program_file"`
		DurationS        float64   `json:"duration_s"`
		TrajectoryDigest string    `json:"trajectory_digest"`
		ProgramDigest    string    `json:"program_digest"`
		PlanID           string    `json:"plan_id"`
		ProgramID        string    `json:"program_id"`
		Window           []float64 `json:"window_deg"`
		PeakSpeedDegS    float64   `json:"peak_speed_deg_s"`
		Levels           []struct {
			AccelDeg float64 `json:"accel_deg_s2"`
		} `json:"levels"`
		Config *struct {
			ConfigurationSHA256 string `json:"configuration_sha256"`
		} `json:"config"`
		Checks []string `json:"checks"`
	}
	if err := json.Unmarshal(raw, &plan); err != nil {
		return nil, httpError(http.StatusBadRequest, "test_program_provenance_invalid", "plan.json is not a JSON object: %v", err)
	}
	if strings.TrimSpace(plan.Schema) == "" {
		return nil, httpError(http.StatusBadRequest, "test_program_provenance_invalid", "plan.json names no schema")
	}
	var differs []string
	for _, field := range []struct{ name, plan, program string }{
		{"trajectory_digest", plan.TrajectoryDigest, id.TrajectoryDigest},
		{"plan_id", plan.PlanID, id.PlanID},
		{"program_id", plan.ProgramID, id.ProgramID},
		{"program_digest", plan.ProgramDigest, id.ProgramDigest},
	} {
		if field.plan != field.program {
			differs = append(differs, field.name+": plan.json "+field.plan+", program "+field.program)
		}
	}
	if len(differs) > 0 {
		return nil, httpError(http.StatusConflict, "test_program_provenance_mismatch",
			"plan.json describes another program (%s)", strings.Join(differs, "; "))
	}
	out := &TestProgramProvenance{Schema: plan.Schema, Generated: plan.Generated, Joint: plan.Joint, ProgramFile: plan.Program,
		DurationS: plan.DurationS, WindowDeg: plan.Window, PeakSpeedDegS: plan.PeakSpeedDegS, Checks: plan.Checks}
	for _, level := range plan.Levels {
		out.AccelLevelsDegS2 = append(out.AccelLevelsDegS2, level.AccelDeg)
	}
	if plan.Config != nil {
		out.ConfigurationSHA256 = plan.Config.ConfigurationSHA256
	}
	return out, nil
}

func headerHasProcessMarkers(blob []byte) (bool, error) {
	if len(blob) < 4 {
		return false, nil
	}
	n := int(uint32(blob[0])<<24 | uint32(blob[1])<<16 | uint32(blob[2])<<8 | uint32(blob[3]))
	if 4+n > len(blob) {
		return false, nil
	}
	var header struct {
		Markers json.RawMessage `json:"process_markers"`
	}
	if err := json.Unmarshal(blob[4:4+n], &header); err != nil {
		return false, err
	}
	trimmed := strings.TrimSpace(string(header.Markers))
	return trimmed != "" && trimmed != "null" && trimmed != "[]", nil
}

// cleanFileName keeps a display name: the base name, printable, bounded.
func cleanFileName(name string) string {
	if i := strings.LastIndexAny(name, `/\`); i >= 0 {
		name = name[i+1:]
	}
	name = strings.Map(func(r rune) rune {
		if r < 0x20 || r == 0x7f {
			return -1
		}
		return r
	}, name)
	if len(name) > 128 {
		name = name[:128]
	}
	return name
}

func (p *TestProgram) label() *TestProgramLabel {
	return &TestProgramLabel{FileName: p.FileName, FileSHA256: p.FileSHA256, Provenance: p.Provenance}
}
