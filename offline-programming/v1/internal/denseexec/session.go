package denseexec

import (
	"context"
	"crypto/rand"
	"encoding/base64"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"

	"net/http"
	"net/url"
	"strings"

	"time"

	"rosieos/offline-programming/v1/internal/densejointtraj"
)

// Unverified UI heartbeat budget; native observation and Stop receipts decide state.
const DefaultHeartbeatDeadline = 5000 * time.Millisecond

// The retained ingest frame contract caps immutable planner blobs at 64 MiB.
const maxBlobBytes = MaxFrameBytes

// Config binds immutable planner input to a native controller identity. It has
// no legacy transport addresses, preventing implicit motion backend fallback.
type Config struct {
	ControllerID string
	MotionOrigin string
	HTTPClient   *http.Client
	FetchBlob    func(ctx context.Context, trajectoryDigest string) ([]byte, error)
	// TestPrograms is where Load with Source TestProgramSource reads its
	// bytes; nil keeps that source refused (test_program.go).
	TestPrograms *TestPrograms
}

// ErrUnavailable identifies absent native configuration without acquiring a grant.
var ErrUnavailable = errors.New("dense execution target is not configured")

// Error is an HTTP-shaped failure: the status and code the handler writes
// as {"error": code, "detail": detail}.
type Error struct {
	Status         int
	Code           string
	Detail         string
	LimitViolation *LimitViolation
}

// LimitViolation is the adapter's logical joint-limit evidence. It supplements
// the refusal code and never changes whether the trajectory is admitted.
type LimitViolation struct {
	Kind    string  `json:"kind"`
	Segment int     `json:"segment"`
	Sample  int     `json:"sample"`
	Axis    string  `json:"axis"`
	Value   float64 `json:"value"`
	Limit   float64 `json:"limit"`
	Unit    string  `json:"unit"`
}

func (e *Error) Error() string { return e.Code + ": " + e.Detail }

func httpError(status int, code, format string, args ...any) *Error {
	return &Error{Status: status, Code: code, Detail: fmt.Sprintf(format, args...)}
}

// Session kinds: a trajectory session owns a loaded plan; a go_home
// session exists only so heartbeat, stop and status work for a go-home
// run started with nothing loaded.
const (
	SessionKindTrajectory = "trajectory"
	SessionKindGoHome     = "go_home"
)

// TrajectorySummary is what load answers with: enough for the UI to draw
// the timeline without decoding the blob itself.
type TrajectorySummary struct {
	TrajectoryDigest string           `json:"trajectory_digest"`
	TotalSampleCount int              `json:"total_sample_count"`
	TotalDurationS   float64          `json:"total_duration_s"`
	DtS              float64          `json:"dt_s"`
	Segments         []SegmentSummary `json:"segments"`
}

type SegmentSummary struct {
	Index       int     `json:"index"`
	Kind        string  `json:"kind"`
	SourceID    string  `json:"source_id"`
	SampleCount int     `json:"sample_count"`
	DurationS   float64 `json:"duration_s"`
}

// Capabilities is GET capabilities.
type Capabilities struct {
	RecoverZeroJogSpeeds bool            `json:"recover_zero_jog_speeds,omitempty"`
	ActiveCell           string          `json:"active_cell"`
	Cells                []CellSpec      `json:"cells"`
	Backend              string          `json:"backend,omitempty"`
	RTCore               *RTCoreManifest `json:"rt_core,omitempty"`
	Available            bool            `json:"available"`
	Cell                 string          `json:"cell"`
	IngestAddr           string          `json:"ingest_addr"`
	NATSURL              string          `json:"nats_url"`
	RTCoreHost           string          `json:"rtcore_host"`
	BridgeCell           string          `json:"bridge_cell"`
	ControllerID         string          `json:"controller_id"`
	DaemonReachable      bool            `json:"daemon_reachable"`
	Reason               string          `json:"reason,omitempty"`

	// UnattendedLoopAllowed is the cell manifest's
	// cell_deploy.olp.unattended_loop_allowed. The pendant offers its
	// Run-page program arm only when it is true; arm refuses
	// {"program": true} otherwise (dense_execution.go).
	UnattendedLoopAllowed bool `json:"unattended_loop_allowed,omitempty"`
}

// LoadRequest is POST load.
type LoadRequest struct {
	Identity
	DryRun bool `json:"dry_run,omitempty"`
	// Source is "" for a planner artifact fetched by digest, or
	// TestProgramSource for an uploaded test program.
	Source string `json:"source,omitempty"`
	// DenseBlobBase64 is the planner artifact as the client kept it from
	// /weld-plan's dense_blob_base64: read only when the planner cannot
	// serve the digest (blobSource.loadBlob).
	DenseBlobBase64 string `json:"dense_blob_base64,omitempty"`
	// ProcessMarkersSHA256 is the output schedule's digest from the client's
	// plan record (the planner's dense summary). The schedule sits outside
	// the trajectory digest, so a production Load of a plan that switches
	// outputs is refused without it or when the plan's markers hash to
	// anything else (rt_core_outputs.go).
	ProcessMarkersSHA256 string `json:"process_markers_sha256,omitempty"`
}

// LoadResult is POST load's answer.
type LoadResult struct {
	OK         bool              `json:"ok"`
	SessionID  string            `json:"session_id"`
	Trajectory TrajectorySummary `json:"trajectory"`
	Status     *StatusBody       `json:"status"`
	// Notes are true things about the load that did not stop it, in words:
	// that the machine's configuration moved since the plan was made, or that
	// the client's kept copy stood in for the planner's.
	Notes []string `json:"notes,omitempty"`
	// BlobSource is where the loaded bytes came from: "planner", "client"
	// (the kept copy) or "test_program".
	BlobSource string `json:"blob_source,omitempty"`
}

// StopResult is POST stop's answer.
type StopResult struct {
	Note     string      `json:"note,omitempty"`
	OK       bool        `json:"ok"`
	Stopped  bool        `json:"stopped"`
	Attempts int         `json:"attempts"`
	Status   *StatusBody `json:"status"`
}

// ArmResult is POST arm's answer: OK says the bridge answered, Accepted
// says what it answered.
type ArmResult struct {
	Note     string      `json:"note,omitempty"`
	OK       bool        `json:"ok"`
	Accepted bool        `json:"accepted"`
	Error    string      `json:"error,omitempty"`
	Status   *StatusBody `json:"status"`
	// Steps Arm took before arming (confirm_stop, reset), as journaled.
	Steps []RecoverStep `json:"steps,omitempty"`
}

// GoHomeResult is POST go-home's answer: the status body plus ok.
type GoHomeResult struct {
	OK bool `json:"ok"`
	*StatusBody
}

// HeartbeatResult is POST heartbeat's answer; AgeMS is how long the
// previous beat had been standing when this one landed.
type HeartbeatResult struct {
	OK         bool  `json:"ok"`
	DeadlineMS int64 `json:"deadline_ms"`
	AgeMS      int64 `json:"age_ms"`
}

// StatusBody is GET status and the `status` member of every other answer.
type StatusBody struct {
	LinkLeaseNS int64              `json:"link_lease_ns,omitempty"`
	RoundTripNS int64              `json:"round_trip_ns,omitempty"`
	Target      *TargetIdentity    `json:"target,omitempty"`
	Backend     string             `json:"backend,omitempty"`
	RTCore      *RTCoreObservation `json:"rt_core,omitempty"`
	Available   bool               `json:"available"`
	Session     *SessionInfo       `json:"session"`
	Daemon      *DaemonStatus      `json:"daemon"`
	DaemonError string             `json:"daemon_error,omitempty"`
	Cursor      *Cursor            `json:"cursor"`
	Armed       *bool              `json:"armed"`
	Axes        json.RawMessage    `json:"axes"`
	// Robot is which robot the machine says it is under the description
	// contract, once it has been described: Configured false with the
	// reason when it binds none (rt_core_robot.go).
	Robot *RobotCell `json:"robot_cell,omitempty"`
	// LastJogEnd is the last jog or Cartesian hold end this server recorded,
	// classified for the operator (jog_end.go). Every client polls this body;
	// only the client named by its client_id shows a pop-up.
	LastJogEnd *JogEnd `json:"last_jog_end,omitempty"`
	// JogLive is the running Cartesian path hold's live readout (speed, what
	// limits it, singularity proximity, the way out), or the last one kept
	// after it ended (path_live_api.go).
	JogLive *PathLive `json:"jog_live,omitempty"`
	// Loop is the running repeat of a Run program press, or the last one
	// that ended, with its cycle count and last cycle time (run_loop.go).
	Loop *LoopStatus `json:"loop,omitempty"`
}

// TargetIdentity labels the process-selected public binding, preventing another
// browser's cell switch from leaving a stale machine label beside live status.
type TargetIdentity struct {
	SelectionGeneration uint64 `json:"selection_generation,string"`
	CellID              string `json:"cell_id"`
	ModelID             string `json:"model_id"`
	Label               string `json:"label"`
	Backend             string `json:"backend"`
	Simulation          *bool  `json:"simulation,omitempty"`
	Host                string `json:"host"`
}

type SessionInfo struct {
	DryRun                    bool      `json:"dry_run"`
	ExecutionTrajectoryDigest string    `json:"execution_trajectory_digest,omitempty"`
	ExecutionNormalisedDigest string    `json:"execution_normalised_digest,omitempty"`
	ID                        string    `json:"id"`
	Kind                      string    `json:"kind"`
	TrajectoryDigest          string    `json:"trajectory_digest"`
	PlanID                    string    `json:"plan_id"`
	State                     string    `json:"state"`
	Lease                     LeaseInfo `json:"lease"`
	HeartbeatAgeMS            int64     `json:"heartbeat_age_ms"`
	HeartbeatDeadlineMS       int64     `json:"heartbeat_deadline_ms"`
	StopReason                string    `json:"stop_reason"`
	LastError                 string    `json:"last_error"`
	// Source and TestProgram are set only for an uploaded test program.
	Source      string            `json:"source,omitempty"`
	TestProgram *TestProgramLabel `json:"test_program,omitempty"`
	// ProcessOutputs names the outputs a production session's program
	// switches, sorted; Play arms them (io_arm) and the session's end
	// disarms them. Empty for a dry run and for motion-only programs.
	ProcessOutputs []string `json:"process_outputs,omitempty"`
}

type LeaseInfo struct {
	Held        bool  `json:"held"`
	ExpiresInMS int64 `json:"expires_in_ms"`
}

// DaemonStatus is the daemon's status document passed through verbatim.
type DaemonStatus struct {
	Staged    json.RawMessage `json:"staged"`
	Executor  json.RawMessage `json:"executor"`
	LastError json.RawMessage `json:"last_error"`
}

// Cursor is the executor's segment-local position mapped onto the loaded
// plan's global timeline, from the decoded header's segment list: the
// daemon reports (segment_index, sample_index, t_s) within one segment
// because each segment is its own clock; the UI wants one bar.
type Cursor struct {
	SegmentIndex      int     `json:"segment_index"`
	SegmentCount      int     `json:"segment_count"`
	Kind              string  `json:"kind"`
	SourceID          string  `json:"source_id"`
	SampleIndex       int     `json:"sample_index"`
	GlobalSampleIndex int     `json:"global_sample_index"`
	TotalSampleCount  int     `json:"total_sample_count"`
	TS                float64 `json:"t_s"`
	GlobalTS          float64 `json:"global_t_s"`
	TotalDurationS    float64 `json:"total_duration_s"`
}

// loadBlob is the bytes Load prepares: the planner's by digest, or an
// uploaded test program's. A planner Load never reads the upload store.
// When the planner cannot serve its digest (a restarted planner, an evicted
// cache, a pruned deployment directory) and the client sent the copy it
// kept at plan time, that copy is loaded instead, and `kept` says so; the
// planner's own copy always wins when it answers.
func (m *blobSource) loadBlob(ctx context.Context, request LoadRequest) ([]byte, *TestProgram, string, error) {
	switch request.Source {
	case "":
		blob, err := m.fetchBlob(ctx, request.TrajectoryDigest)
		if err == nil || request.DenseBlobBase64 == "" {
			return blob, nil, "", err
		}
		kept, keptErr := keptPlannerBlob(request)
		if keptErr != nil {
			return nil, nil, "", keptErr
		}
		return kept, nil, fmt.Sprintf("The weld planner could not serve this trajectory (%v); loaded the copy this client kept at plan time, checked against its trajectory digest", err), nil
	case TestProgramSource:
		if request.DryRun {
			return nil, nil, "", httpError(http.StatusBadRequest, "test_program_dry_run", "a test program has no process outputs; load it without dry_run")
		}
		program, err := m.cfg.TestPrograms.get(request.TrajectoryDigest)
		if err != nil {
			return nil, nil, "", err
		}
		if program.Identity != request.Identity {
			return nil, nil, "", httpError(http.StatusConflict, "dense_identity_mismatch", "the load names another identity than the uploaded test program %s", request.TrajectoryDigest)
		}
		return program.blob, program, "", nil
	}
	return nil, nil, "", httpError(http.StatusBadRequest, "dense_load_source_invalid", "source must be empty (planner) or %q", TestProgramSource)
}

// keptPlannerBlob is the trajectory a client kept from /weld-plan's answer,
// admitted only as exactly the plan the Load names: no larger than the
// planner's own fetch cap, decodable, its samples hashing to the header's
// trajectory digest (Decode recomputes it), and the header's six-field
// identity equal to the request's. Header fields outside that digest
// (start_state, robot_cell) are judged by Load's own gates against the live
// machine, as for the planner's copy: validatePlanningCapture, robotGate,
// Home, and native Play's first-sample check.
func keptPlannerBlob(request LoadRequest) ([]byte, error) {
	if base64.StdEncoding.DecodedLen(len(request.DenseBlobBase64)) > maxBlobBytes+2 {
		return nil, httpError(http.StatusRequestEntityTooLarge, "dense_blob_too_large", "the kept trajectory exceeds %d bytes", maxBlobBytes)
	}
	blob, err := base64.StdEncoding.DecodeString(request.DenseBlobBase64)
	if err != nil || len(blob) == 0 {
		return nil, httpError(http.StatusBadRequest, "dense_blob_invalid", "dense_blob_base64 is not base64 trajectory bytes")
	}
	if len(blob) > maxBlobBytes {
		return nil, httpError(http.StatusRequestEntityTooLarge, "dense_blob_too_large", "the kept trajectory is %d bytes; at most %d", len(blob), maxBlobBytes)
	}
	plan, err := densejointtraj.Decode(blob)
	if err != nil {
		return nil, httpError(http.StatusBadRequest, "dense_trajectory_invalid", "the kept trajectory: %v", err)
	}
	h := plan.Header
	if (Identity{PlanID: h.PlanID, ProgramID: h.ProgramID, ProgramDigest: h.ProgramDigest, TrajectoryDigest: h.TrajectoryDigest, ManifestRevision: h.ManifestRevision, PlanRevision: h.PlanRevision}) != request.Identity {
		return nil, httpError(http.StatusConflict, "dense_identity_mismatch", "the kept trajectory is %s, not the %s this Load names", h.TrajectoryDigest, request.TrajectoryDigest)
	}
	return blob, nil
}

func (m *blobSource) fetchBlob(ctx context.Context, digest string) ([]byte, error) {
	if m.cfg.FetchBlob != nil {
		return m.cfg.FetchBlob(ctx, digest)
	}
	origin := strings.TrimSpace(m.cfg.MotionOrigin)
	if origin == "" {
		return nil, httpError(http.StatusServiceUnavailable, "dense_blob_source_unavailable",
			"no weld planner motion origin configured; set OFFLINE_PROGRAMMING_WELD_PLANNER_MOTION_ORIGIN")
	}
	endpoint := strings.TrimRight(origin, "/") + "/api/motion/dense/" + url.PathEscape(digest)
	request, err := http.NewRequestWithContext(ctx, http.MethodGet, endpoint, nil)
	if err != nil {
		return nil, err
	}
	request.Header.Set("Accept", "application/octet-stream")
	response, err := m.cfg.HTTPClient.Do(request)
	if err != nil {
		return nil, fmt.Errorf("fetch %s: %w", endpoint, err)
	}
	defer response.Body.Close()
	if response.StatusCode == http.StatusNotFound {
		return nil, httpError(http.StatusNotFound, "dense_blob_not_found",
			"the weld planner motion server has no dense trajectory %s", digest)
	}
	if response.StatusCode != http.StatusOK {
		snippet, _ := io.ReadAll(io.LimitReader(response.Body, 512))
		return nil, fmt.Errorf("fetch %s: HTTP %d: %s", endpoint, response.StatusCode, strings.TrimSpace(string(snippet)))
	}
	blob, err := io.ReadAll(io.LimitReader(response.Body, maxBlobBytes+1))
	if err != nil {
		return nil, fmt.Errorf("fetch %s: %w", endpoint, err)
	}
	if len(blob) > maxBlobBytes {
		return nil, fmt.Errorf("fetch %s: blob exceeds %d bytes", endpoint, maxBlobBytes)
	}
	return blob, nil
}

func validateIdentity(identity Identity) error {
	switch {
	case !strings.HasPrefix(identity.TrajectoryDigest, "sha256:") || len(identity.TrajectoryDigest) != 71:
		return errors.New("trajectory_digest must be sha256:<64 hex>")
	case strings.TrimSpace(identity.PlanID) == "":
		return errors.New("plan_id is required")
	case strings.TrimSpace(identity.ProgramID) == "":
		return errors.New("program_id is required")
	case strings.TrimSpace(identity.ProgramDigest) == "":
		return errors.New("program_digest is required")
	}
	return nil
}

func summarize(header densejointtraj.Header) TrajectorySummary {
	summary := TrajectorySummary{
		TrajectoryDigest: header.TrajectoryDigest,
		TotalSampleCount: header.Sampling.TotalSampleCount,
		TotalDurationS:   header.Sampling.TotalDurationS,
		DtS:              header.Sampling.DtS,
		Segments:         make([]SegmentSummary, 0, len(header.Segments)),
	}
	for _, meta := range header.Segments {
		summary.Segments = append(summary.Segments, SegmentSummary{
			Index:       meta.Index,
			Kind:        meta.Kind,
			SourceID:    meta.SourceID,
			SampleCount: meta.SampleCount,
			DurationS:   meta.DurationS,
		})
	}
	return summary
}

func newSessionID() string {
	var raw [6]byte
	if _, err := rand.Read(raw[:]); err != nil {
		return fmt.Sprintf("ds-%x", time.Now().UnixNano())
	}
	return "ds-" + hex.EncodeToString(raw[:])
}

// blobSource fetches immutable planner input without opening a motion transport.
type blobSource struct{ cfg Config }
