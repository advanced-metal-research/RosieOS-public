// Package denseexec drives the dense trajectory daemon (motion-server/joint-trajectory/v1)
// on behalf of the OLP UI: the blob goes in over the daemon's TCP ingest,
// motion authority (leases, play/pause/stop) goes over NATS, and one
// session at a time ties the two together with a renew loop, a status
// poll, and a heartbeat watchdog so a vanished browser tab ends in a stop
// rather than in an arm that keeps moving under nobody's supervision.
//
// This file is the TCP ingest client. Framing is the daemon's
// (dense_joint_trajectory_ingest.hpp, mirrored by the Python client in
// weld_planner/v1/python/weldplan/dense_joint_trajectory.py):
//
//	[u64 BE size][payload]         payload = [u32 BE json_len][JSON][blob]
//
// one request, one response, close. Upload is inert on the daemon side;
// preload stages by reference and re-verifies the bytes at rest; play stays
// on NATS, so nothing this socket can carry causes motion by itself.
package denseexec

import (
	"context"
	"encoding/binary"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net"
	"time"
)

const (
	// MaxFrameBytes mirrors the daemon's kMaxFrameBytes: a 250,000-sample
	// plan is ~38 MB, 64 MiB leaves headroom, and a larger frame is refused
	// here before it is allocated rather than trusted from the wire.
	MaxFrameBytes = 64 << 20

	// ExchangeDeadline mirrors the daemon's kSocketTimeoutSeconds: one
	// exchange (a 38 MB upload included) must finish inside it.
	ExchangeDeadline = 240 * time.Second
)

// Identity is the six-field plan identity the daemon matches exactly at
// preload and again at play. manifest_revision and plan_revision are
// integers on the wire (the daemon reads them with json_u64); a string
// would be read as "missing" and the preload refused.
type Identity struct {
	TrajectoryDigest string `json:"trajectory_digest"`
	PlanID           string `json:"plan_id"`
	ProgramID        string `json:"program_id"`
	ProgramDigest    string `json:"program_digest"`
	ManifestRevision uint64 `json:"manifest_revision"`
	PlanRevision     uint64 `json:"plan_revision"`
}

// IngestClient talks to one daemon ingest listener. It is stateless: each
// call opens, exchanges, and closes its own connection, as the daemon
// expects.
type IngestClient struct {
	Addr string
	// Deadline bounds one exchange; zero means ExchangeDeadline. A context
	// deadline that is sooner wins.
	Deadline time.Duration
}

// IngestReply is the control JSON of an upload, validate, or preload
// answer. On a rejection Reason/Detail carry the daemon's shared reason
// vocabulary and SegmentIndex the segment at fault when there is one
// (absent otherwise, hence the pointer).
type IngestReply struct {
	OK               bool    `json:"ok"`
	Kind             string  `json:"kind,omitempty"`
	Reason           string  `json:"reason,omitempty"`
	Detail           string  `json:"detail,omitempty"`
	SegmentIndex     *int    `json:"segment_index,omitempty"`
	TrajectoryDigest string  `json:"trajectory_digest,omitempty"`
	SegmentCount     int     `json:"segment_count,omitempty"`
	TotalSampleCount int     `json:"total_sample_count,omitempty"`
	TotalDurationS   float64 `json:"total_duration_s,omitempty"`
}

// StatusReply is the daemon's `status` answer. staged/executor/last_error
// are kept raw so the HTTP status body can pass the daemon's document
// through verbatim (the UI renders fields this server never has to know
// about); Executor() decodes the executor leniently on demand. The
// executor is the string "absent" when the daemon runs without one.
type StatusReply struct {
	OK          bool            `json:"ok"`
	Kind        string          `json:"kind,omitempty"`
	Schema      string          `json:"schema,omitempty"`
	StoreDir    string          `json:"store_dir,omitempty"`
	Staged      json.RawMessage `json:"staged"`
	ExecutorRaw json.RawMessage `json:"executor"`
	LastError   json.RawMessage `json:"last_error"`
	Reason      string          `json:"reason,omitempty"`
	Detail      string          `json:"detail,omitempty"`
}

// StagedSummary is the staged plan the daemon reports, when any.
type StagedSummary struct {
	TrajectoryDigest string `json:"trajectory_digest"`
	PlanID           string `json:"plan_id"`
	TotalSampleCount int    `json:"total_sample_count"`
}

// ExecutorStatus is the executor document decoded leniently: the fields the
// daemon has always sent, plus armed/authority/feedback that a newer daemon
// adds. Missing fields decode to their zero value or nil, never to an
// error, so this server keeps working against a daemon from either side of
// that change.
type ExecutorStatus struct {
	State            string             `json:"state"`
	AbortReason      string             `json:"abort_reason"`
	TrajectoryDigest string             `json:"trajectory_digest"`
	SegmentIndex     int                `json:"segment_index"`
	TS               float64            `json:"t_s"`
	SampleIndex      int                `json:"sample_index"`
	TorchOn          bool               `json:"torch_on"`
	FeedbackAgeMS    int64              `json:"feedback_age_ms"`
	AuthorityFresh   bool               `json:"authority_fresh"`
	Armed            *bool              `json:"armed"`
	Authority        *ExecutorAuthority `json:"authority"`
	Feedback         *ExecutorFeedback  `json:"feedback"`
	// Transient is "go_home" while a daemon-synthesised go-home plan is
	// active, kept after it ended, and cleared by the next stage/play;
	// state/segment_index/sample_index/t_s then describe that plan, while
	// trajectory_digest stays the staged plan's. Absent on older daemons.
	Transient     *string        `json:"transient"`
	TransientPlan *TransientPlan `json:"transient_plan"`
	// Robot is the description and machine_planning_calibration.json the daemon holds plans to.
	// Absent (nil) on a daemon from before the contract.
	Robot *RobotCell `json:"robot_cell"`
}

// TransientPlan describes the daemon-synthesised plan behind Transient.
type TransientPlan struct {
	Kind        string  `json:"kind"`
	SampleCount int     `json:"sample_count"`
	DurationS   float64 `json:"duration_s"`
}

// TransientGoHome is the Transient value of a go-home run.
const TransientGoHome = "go_home"

// InTransient reports whether the executor's cursor describes a
// daemon-synthesised plan rather than the staged trajectory.
func (s *ExecutorStatus) InTransient() bool {
	return s != nil && s.Transient != nil && *s.Transient != ""
}

// ExecutorAuthority is the RTCore pair-authority document as the executor
// last saw it; runtime_mode and its epoch are what an arm request must
// echo on a real cell.
type ExecutorAuthority struct {
	Valid            bool   `json:"valid"`
	Fresh            bool   `json:"fresh"`
	RuntimeMode      string `json:"runtime_mode"`
	RuntimeModeEpoch uint64 `json:"runtime_mode_epoch"`
	MappedAxisMask   uint32 `json:"mapped_axis_mask"`
	JetsonID         string `json:"jetson_id"`
	RobotModelID     string `json:"robot_model_id"`
}

// ExecutorFeedback is the servo feedback summary; Axes stays raw for
// pass-through to the UI.
type ExecutorFeedback struct {
	Valid         bool            `json:"valid"`
	AgeMS         int64           `json:"age_ms"`
	ArmedMask     uint32          `json:"armed_mask"`
	FaultMask     uint32          `json:"fault_mask"`
	ErrorMask     uint32          `json:"error_mask"`
	HomeValidMask uint32          `json:"home_valid_mask"`
	Axes          json.RawMessage `json:"axes"`
}

// Executor decodes the executor document. ok is false when the daemon
// reports no executor ("absent", null, or missing).
func (r *StatusReply) Executor() (status *ExecutorStatus, ok bool) {
	if r == nil || len(r.ExecutorRaw) == 0 {
		return nil, false
	}
	var decoded ExecutorStatus
	if err := json.Unmarshal(r.ExecutorRaw, &decoded); err != nil {
		// "absent" (a JSON string) lands here, as does anything else that
		// is not an object.
		return nil, false
	}
	return &decoded, true
}

// StagedPlan decodes the staged summary; nil when nothing is staged.
func (r *StatusReply) StagedPlan() *StagedSummary {
	if r == nil || len(r.Staged) == 0 {
		return nil
	}
	var staged StagedSummary
	if err := json.Unmarshal(r.Staged, &staged); err != nil || staged.TrajectoryDigest == "" {
		return nil
	}
	return &staged
}

// Rejection is a daemon answer with ok:false: the request reached the
// daemon and was refused for the stated reason. It is distinct from a
// transport error so a caller can tell "the daemon said no" from "the
// daemon is not there".
type Rejection struct {
	Kind         string
	Reason       string
	Detail       string
	SegmentIndex *int
}

func (r *Rejection) Error() string {
	if r.SegmentIndex != nil {
		return fmt.Sprintf("%s rejected: %s: segment %d: %s",
			r.Kind, r.Reason, *r.SegmentIndex, r.Detail)
	}
	return fmt.Sprintf("%s rejected: %s: %s", r.Kind, r.Reason, r.Detail)
}

// Upload stores a blob in the daemon's plan store. Inert: the daemon never
// moves on an upload. The daemon decodes and validates the blob fully and
// answers with its digest.
func (c *IngestClient) Upload(ctx context.Context, blob []byte) (*IngestReply, error) {
	return c.ingestRequest(ctx, "upload", map[string]any{"kind": "upload"}, blob)
}

// Validate is a dry run of Upload: same validation, nothing persisted.
func (c *IngestClient) Validate(ctx context.Context, blob []byte) (*IngestReply, error) {
	return c.ingestRequest(ctx, "validate", map[string]any{"kind": "validate"}, blob)
}

// Preload stages a stored blob by reference. Every identity field must
// match the stored header exactly; the daemon re-verifies the bytes at
// rest; and a preload during playback is refused with
// playback_in_progress and leaves the in-flight plan alone.
func (c *IngestClient) Preload(ctx context.Context, identity Identity) (*IngestReply, error) {
	control := map[string]any{
		"kind":              "preload",
		"trajectory_digest": identity.TrajectoryDigest,
		"plan_id":           identity.PlanID,
		"program_id":        identity.ProgramID,
		"program_digest":    identity.ProgramDigest,
		"manifest_revision": identity.ManifestRevision,
		"plan_revision":     identity.PlanRevision,
	}
	return c.ingestRequest(ctx, "preload", control, nil)
}

// Status reads the daemon's status document.
func (c *IngestClient) Status(ctx context.Context) (*StatusReply, error) {
	raw, err := c.exchange(ctx, map[string]any{"kind": "status"}, nil)
	if err != nil {
		return nil, err
	}
	var reply StatusReply
	if err := json.Unmarshal(raw, &reply); err != nil {
		return nil, fmt.Errorf("dense ingest: status reply is not JSON: %w", err)
	}
	if !reply.OK {
		return &reply, &Rejection{Kind: "status", Reason: reply.Reason, Detail: reply.Detail}
	}
	return &reply, nil
}

func (c *IngestClient) ingestRequest(
	ctx context.Context, kind string, control map[string]any, body []byte,
) (*IngestReply, error) {
	raw, err := c.exchange(ctx, control, body)
	if err != nil {
		return nil, err
	}
	var reply IngestReply
	if err := json.Unmarshal(raw, &reply); err != nil {
		return nil, fmt.Errorf("dense ingest: %s reply is not JSON: %w", kind, err)
	}
	if !reply.OK {
		return &reply, &Rejection{
			Kind:         kind,
			Reason:       reply.Reason,
			Detail:       reply.Detail,
			SegmentIndex: reply.SegmentIndex,
		}
	}
	return &reply, nil
}

// exchange performs one framed request/response and returns the response's
// control JSON. The frame cap is enforced on both directions: a reply that
// claims more than MaxFrameBytes is refused before any allocation.
func (c *IngestClient) exchange(ctx context.Context, control any, body []byte) ([]byte, error) {
	if c == nil || c.Addr == "" {
		return nil, errors.New("dense ingest: no address configured")
	}
	controlJSON, err := json.Marshal(control)
	if err != nil {
		return nil, err
	}
	payloadLen := 4 + len(controlJSON) + len(body)
	if uint64(payloadLen) > MaxFrameBytes {
		return nil, fmt.Errorf("dense ingest: request frame %d bytes exceeds %d", payloadLen, MaxFrameBytes)
	}
	deadline := c.Deadline
	if deadline <= 0 {
		deadline = ExchangeDeadline
	}
	dialer := net.Dialer{Timeout: deadline}
	conn, err := dialer.DialContext(ctx, "tcp", c.Addr)
	if err != nil {
		return nil, fmt.Errorf("dense ingest: connect %s: %w", c.Addr, err)
	}
	defer conn.Close()
	// A cancelled context must unblock a stalled read, not wait out the
	// full deadline.
	stopWatching := context.AfterFunc(ctx, func() { _ = conn.Close() })
	defer stopWatching()
	limit := time.Now().Add(deadline)
	if ctxDeadline, ok := ctx.Deadline(); ok && ctxDeadline.Before(limit) {
		// A hair past the context deadline: the AfterFunc close is what
		// unblocks the read then, so the error reports the context, not a
		// socket timeout that raced it.
		limit = ctxDeadline.Add(100 * time.Millisecond)
	}
	if err := conn.SetDeadline(limit); err != nil {
		return nil, err
	}

	header := make([]byte, 12)
	binary.BigEndian.PutUint64(header[0:8], uint64(payloadLen))
	binary.BigEndian.PutUint32(header[8:12], uint32(len(controlJSON)))
	for _, chunk := range [][]byte{header, controlJSON, body} {
		if _, err := conn.Write(chunk); err != nil {
			return nil, fmt.Errorf("dense ingest: write: %w", wrapContext(ctx, err))
		}
	}

	var sizeRaw [8]byte
	if _, err := io.ReadFull(conn, sizeRaw[:]); err != nil {
		return nil, fmt.Errorf("dense ingest: read reply size: %w", wrapContext(ctx, err))
	}
	size := binary.BigEndian.Uint64(sizeRaw[:])
	if size < 4 || size > MaxFrameBytes {
		return nil, fmt.Errorf("dense ingest: reply frame %d bytes is outside 4..%d", size, MaxFrameBytes)
	}
	payload := make([]byte, size)
	if _, err := io.ReadFull(conn, payload); err != nil {
		return nil, fmt.Errorf("dense ingest: read reply: %w", wrapContext(ctx, err))
	}
	jsonLen := binary.BigEndian.Uint32(payload[:4])
	if uint64(4+jsonLen) > size {
		return nil, fmt.Errorf("dense ingest: reply json_len %d exceeds payload %d", jsonLen, size)
	}
	return payload[4 : 4+jsonLen], nil
}

func wrapContext(ctx context.Context, err error) error {
	if ctxErr := ctx.Err(); ctxErr != nil {
		return ctxErr
	}
	return err
}
