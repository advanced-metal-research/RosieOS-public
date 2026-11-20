// Package seam provides a bounded adapter around the weld planner's
// `seam_worker` Python package: topology, weld-joint detection with the seams
// they default to, tessellation and torch clearance.
//
// It is the sibling of `internal/cad`, and the split is by what the two
// produce rather than by how they work. `internal/cad` drives
// `extract_weld_edges.py` and produces the classic flow's weld: an edge, its
// sampled polyline, a baked TCP pose per point. This one produces SEAMS --
// a curve, the bisector rail its work angle is measured against, and angle
// profiles with the bands they may be traded inside. Both are CAD, both are
// geometry; only one of them makes a seam.
//
// So it answers only what the other cannot. The topology and the display mesh
// stay with `internal/cad`, which the viewer already reads: a second
// `extract_topology`, on a second worker, returning a different document
// under the same name is worse than not having one.
//
// Like its sibling it contains no HTTP or product-specific server wiring, and
// it holds nothing between calls at all: the STEP travels in the request, one
// process answers it, and that process exits. That is D24 -- the authoring
// server was deleted because a program whose geometry lived in another
// application's workspace could not be compiled without it -- and it is also
// just what the numbers say. Nothing worth caching survives a request here.
package seam

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
)

// Operation is one of the entry points `seam_worker.workers` dispatches.
type Operation string

const (
	OperationDetectJoints     Operation = "detect_joints"
	OperationCheckTorchFits   Operation = "check_torch_fits"
	OperationPlanTorchPath    Operation = "plan_torch_path"
	OperationSampleSeamFrames Operation = "sample_seam_frames"
	OperationBuildPlanRequest Operation = "build_plan_request"
)

// ErrorCode is a stable, transport-independent failure category. The same set
// `internal/cad` uses, because a caller that drives both should classify them
// the same way.
type ErrorCode string

const (
	ErrorInvalidRequest  ErrorCode = "invalid_request"
	ErrorPayloadTooLarge ErrorCode = "payload_too_large"
	ErrorBusy            ErrorCode = "busy"
	ErrorTimeout         ErrorCode = "timeout"
	ErrorCanceled        ErrorCode = "canceled"
	ErrorUnavailable     ErrorCode = "unavailable"
	ErrorWorkerFailed    ErrorCode = "worker_failed"
	ErrorInvalidResponse ErrorCode = "invalid_response"
)

var (
	ErrInvalidRequest  = errors.New("invalid seam request")
	ErrPayloadTooLarge = errors.New("seam payload too large")
	ErrBusy            = errors.New("seam worker busy")
	ErrTimeout         = errors.New("seam worker timeout")
	ErrCanceled        = errors.New("seam request canceled")
	ErrUnavailable     = errors.New("seam worker unavailable")
	ErrWorkerFailed    = errors.New("seam worker failed")
	ErrInvalidResponse = errors.New("invalid seam worker response")
)

// WorkerError carries a stable code while retaining an errors.Is-compatible
// cause for Go callers.
type WorkerError struct {
	Code      ErrorCode
	Operation Operation
	Message   string
	Cause     error
}

func (err *WorkerError) Error() string {
	if err == nil {
		return ""
	}
	if err.Operation == "" {
		return err.Message
	}
	return fmt.Sprintf("seam %s: %s", err.Operation, err.Message)
}

func (err *WorkerError) Unwrap() error {
	if err == nil {
		return nil
	}
	return err.Cause
}

func workerError(code ErrorCode, operation Operation, message string, cause error) error {
	return &WorkerError{Code: code, Operation: operation, Message: message, Cause: cause}
}

// JointsRequest asks where the parts meet, and for the seams each joint
// defaults to. Topology is recomputed unless one is supplied, which is the
// only reason to hand one back: it is the expensive half.
type JointsRequest struct {
	STEP     []byte
	Topology []byte
	// RailOffsetM offsets the bisector rail from the weld line. Zero leaves
	// the worker's own default (20 mm), the same convention as the two
	// fields below -- because zero is not a usable value here, it is a
	// broken one. The offset is the only thing that gives the rail a
	// DIRECTION: `frames.py` reads the work-angle reference as
	// (rail - point), so a rail lying on the seam yields the zero vector,
	// the reference falls back to world up, and every 90 degree fillet is
	// measured 45 degrees off its own bisector.
	RailOffsetM float64
	// MinLengthM drops seams shorter than this; MaxCornerAngleDeg cuts a run
	// apart where the tangent turns harder than this. Zero leaves the
	// worker's own defaults, which are the measured ones.
	MinLengthM        float64
	MaxCornerAngleDeg float64
}

// ClearanceRequest asks whether a torch of this shape can reach along this
// seam, and where it cannot.
type TorchFitRequest struct {
	STEP    []byte
	Joint   []byte
	Seam    []byte
	Torch   []byte
	Options []byte
}

// SearchKnobs is how hard the angle search looks. Values, never a preset id:
// a preset is a starting point and a comparison, and one edited later must not
// reach back and change a weld that was already accepted.
type SearchKnobs struct {
	WorkDeviationDeg     float64 `json:"work_deviation_deg"`
	TravelDeviationDeg   float64 `json:"travel_deviation_deg"`
	StandoffCostDegPerMM float64 `json:"standoff_cost_deg_per_mm"`
	WorkBinDeg           float64 `json:"work_bin_deg,omitempty"`
	TravelBinDeg         float64 `json:"travel_bin_deg,omitempty"`
	KnotSpacingMM        float64 `json:"knot_spacing_mm,omitempty"`
	SampleStepMM         float64 `json:"sample_step_mm,omitempty"`
	RayCount             int     `json:"ray_count,omitempty"`
}

// TorchPathRequest asks where the torch goes along one seam and how it is held
// there. One DIRECTION per request: a caller that wants the seam tried both
// ways asks twice and keeps the better coverage, because the two answers are
// only comparable once both have been measured the same way.
type TorchPathRequest struct {
	STEP  []byte
	Joint []byte
	Seam  []byte
	Torch []byte
	// Direction is "forward" or "reverse"; empty means forward.
	Direction string
	Search    SearchKnobs
	// DefaultStandoffMM is what the weld falls back to when it declares none.
	// Required: a worker that guessed would return a coverage number about
	// hardware nobody has.
	DefaultStandoffMM float64
	Intersector       []byte
}

// SeamFramesRequest asks for the {P, t, r, b} a viewer needs along a seam. The
// only request here that carries no STEP: a curve and the rail its work angle
// is measured from are the whole input, and no ray is cast.
//
// It is answered here rather than in the browser because a sign convention
// implemented twice is a sign convention that can disagree with itself. The
// frames do not depend on the work or travel angle, so dragging a slider needs
// no call: the server sends frames, the client applies angles.
type SeamFramesRequest struct {
	Curve     []byte
	Reference []byte
	// Samples is the uniform grid. Zero leaves the worker's own default, and
	// MaxChordM and MaxAngleRad add positions wherever the seam or the rail
	// bends enough to need them.
	Samples     int
	MaxChordM   float64
	MaxAngleRad float64
}

// PlanRequestBuild asks for the `.weldplan` a motion planner is handed:
// program.v2, the planning context, the torch and the geometry, as bytes that
// hash the same every time.
//
// The robot arrives as a compiled context rather than a name. Naming one
// would resolve differently on another machine, and the placement is
// measured from that cell's work frame, so a different one silently
// relocates the part.
type PlanRequestBuild struct {
	WorkpieceAbsent bool
	STEP            []byte
	STEPFilename    string
	// Program is a `robot.v4.program.v2` document.
	Program []byte
	// PlanningContext is the compiled robot the plan is for -- the corrected
	// URDF, the merged numbers and the description's registered files, as
	// base64 members (docs/PLANNING-CONTEXT.md). The worker packs them and
	// reads the descriptor off the context's own URDF; it is handed no
	// URDF, settings or frames of its own to combine.
	PlanningContext []byte
	// StartState is the server-captured native pose and coordinate identity.
	StartState json.RawMessage
	// IOOutputs are the connected cell's outputs (weldplan/io_schedule.py),
	// read from its Describe when the program switches any. Absent or null
	// is "no outputs packed", never an empty output map.
	IOOutputs json.RawMessage
	Tooling   []byte
	// RequestID names the request in its own manifest. Empty is "plan".
	RequestID string
	Label     string
	// Fixtures are the operator's workcell bodies (a stand under the part), as
	// the OLP sent them; nil means the request says nothing about them and the
	// worker keeps the cell's placeholder. Passed on verbatim.
	Fixtures json.RawMessage
}

// Result is one worker answer, and the content hash of the part it is about.
type Result struct {
	Operation   Operation
	Fingerprint string
	JSON        []byte
}

// Command describes one bounded subprocess invocation.
type Command struct {
	Dir            string
	Name           string
	Args           []string
	Stdin          []byte
	MaxOutputBytes int64
}

// CommandRunner makes process execution replaceable in focused tests.
type CommandRunner interface {
	Run(ctx context.Context, command Command) (stdout []byte, stderr []byte, err error)
}
