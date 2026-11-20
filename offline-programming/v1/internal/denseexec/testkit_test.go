package denseexec

import (
	"encoding/binary"
	"encoding/json"
	"errors"

	"io"
	"net"
	"os"

	"path/filepath"
	"sync"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/densejointtraj"
)

func fixtureBlob(t *testing.T) ([]byte, *densejointtraj.Plan) {
	t.Helper()
	path := filepath.Join("..", "..", "..", "..", "motion-server", "joint-trajectory", "v1", "testdata",
		"dense_joint_trajectory_reference.rdt")
	blob, err := os.ReadFile(path)
	if err != nil {
		t.Fatalf("read fixture: %v", err)
	}
	plan, err := densejointtraj.Decode(blob)
	if err != nil {
		t.Fatalf("decode fixture: %v", err)
	}
	return blob, plan
}

func fixtureIdentity(plan *densejointtraj.Plan) Identity {
	return Identity{
		TrajectoryDigest: plan.Header.TrajectoryDigest,
		PlanID:           plan.Header.PlanID,
		ProgramID:        plan.Header.ProgramID,
		ProgramDigest:    plan.Header.ProgramDigest,
		ManifestRevision: plan.Header.ManifestRevision,
		PlanRevision:     plan.Header.PlanRevision,
	}
}

// fakeDaemon is the shared state behind the ingest listener and the NATS
// subscriber, the way the real daemon's executor is shared by both.
type fakeDaemon struct {
	mu sync.Mutex

	store  map[string][]byte // digest -> blob
	staged *densejointtraj.Header

	executorAbsent bool
	state          string // idle | staged | playing | paused | done | aborted
	abortReason    string
	segmentIndex   int
	sampleIndex    int
	tS             float64
	armed          bool
	runtimeMode    string
	runtimeEpoch   uint64

	leaseActive   bool
	leaseSender   string
	leaseBoot     string
	leaseID       string
	fenceEpoch    uint64
	nextEpoch     uint64
	leaseDeadline time.Time
	latestSeq     map[string]uint64

	// statusRequests counts `status` over the ingest socket: a test that
	// asserts the status stream is doing the work watches it stay put.
	statusRequests int

	commands  []map[string]any // every robot command received, in order
	armLog    []map[string]any
	armReply  map[string]any
	lastFrame []byte // raw payload of the last ingest request

	// transient mirrors the daemon's go-home bookkeeping: set while and
	// after a go-home run, cleared by the next preload/play.
	transient     string
	transientPlan map[string]any
}

func newFakeDaemon() *fakeDaemon {
	return &fakeDaemon{
		store:        map[string][]byte{},
		state:        "idle",
		segmentIndex: -1,
		nextEpoch:    1,
		latestSeq:    map[string]uint64{},
		armed:        true,
		runtimeMode:  "csp",
		runtimeEpoch: 7,
		armReply:     map[string]any{"ok": true},
	}
}

func (d *fakeDaemon) setExecutor(state string, segment, sample int, tS float64) {
	d.mu.Lock()
	defer d.mu.Unlock()
	d.state = state
	d.segmentIndex = segment
	d.sampleIndex = sample
	d.tS = tS
}

func startFakeIngest(t *testing.T, d *fakeDaemon) string {
	t.Helper()
	listener, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatalf("listen: %v", err)
	}
	t.Cleanup(func() { listener.Close() })
	go func() {
		for {
			conn, err := listener.Accept()
			if err != nil {
				return
			}
			go d.serveIngest(conn)
		}
	}()
	return listener.Addr().String()
}

func (d *fakeDaemon) serveIngest(conn net.Conn) {
	defer conn.Close()
	var sizeRaw [8]byte
	if _, err := io.ReadFull(conn, sizeRaw[:]); err != nil {
		return
	}
	size := binary.BigEndian.Uint64(sizeRaw[:])
	if size > MaxFrameBytes {
		return
	}
	payload := make([]byte, size)
	if _, err := io.ReadFull(conn, payload); err != nil {
		return
	}
	d.mu.Lock()
	d.lastFrame = append([]byte(nil), payload...)
	d.mu.Unlock()
	response := d.handleIngest(payload)
	out := make([]byte, 12+len(response))
	binary.BigEndian.PutUint64(out[0:8], uint64(4+len(response)))
	binary.BigEndian.PutUint32(out[8:12], uint32(len(response)))
	copy(out[12:], response)
	_, _ = conn.Write(out)
}

func ingestErrorJSON(reason string, segment int, detail string) []byte {
	body := map[string]any{"ok": false, "reason": reason, "detail": detail}
	if segment >= 0 {
		body["segment_index"] = segment
	}
	raw, _ := json.Marshal(body)
	return raw
}

func (d *fakeDaemon) handleIngest(payload []byte) []byte {
	if len(payload) < 4 {
		return ingestErrorJSON("request_invalid", -1, "short")
	}
	jsonLen := binary.BigEndian.Uint32(payload[:4])
	if uint64(4+jsonLen) > uint64(len(payload)) {
		return ingestErrorJSON("request_invalid", -1, "json_len exceeds payload")
	}
	var control map[string]any
	if err := json.Unmarshal(payload[4:4+jsonLen], &control); err != nil {
		return ingestErrorJSON("request_invalid", -1, err.Error())
	}
	body := payload[4+jsonLen:]
	kind, _ := control["kind"].(string)
	switch kind {
	case "upload", "validate":
		plan, err := densejointtraj.Decode(body)
		if err != nil {
			var rejection *densejointtraj.RejectionError
			if errors.As(err, &rejection) {
				return ingestErrorJSON(rejection.Reason, rejection.SegmentIndex, rejection.Detail)
			}
			return ingestErrorJSON("header_json_invalid", -1, err.Error())
		}
		if kind == "upload" {
			d.mu.Lock()
			d.store[plan.Header.TrajectoryDigest] = append([]byte(nil), body...)
			d.mu.Unlock()
		}
		raw, _ := json.Marshal(map[string]any{
			"ok": true, "kind": kind,
			"trajectory_digest":  plan.Header.TrajectoryDigest,
			"segment_count":      len(plan.Segments),
			"total_sample_count": plan.Header.Sampling.TotalSampleCount,
			"total_duration_s":   plan.Header.Sampling.TotalDurationS,
		})
		return raw
	case "preload":
		digest, _ := control["trajectory_digest"].(string)
		d.mu.Lock()
		blob, found := d.store[digest]
		state := d.state
		d.mu.Unlock()
		if !found {
			return ingestErrorJSON("plan_not_found", -1, digest+" not in store")
		}
		plan, err := densejointtraj.Decode(blob)
		if err != nil {
			return ingestErrorJSON("block_sha256_mismatch", 0, err.Error())
		}
		// The daemon reads revisions with json_u64: a string is "missing".
		manifest, okManifest := control["manifest_revision"].(float64)
		planRevision, okPlan := control["plan_revision"].(float64)
		if !okManifest || !okPlan {
			return ingestErrorJSON("identity_mismatch", -1, "missing manifest_revision")
		}
		header := plan.Header
		if control["plan_id"] != header.PlanID || control["program_id"] != header.ProgramID ||
			control["program_digest"] != header.ProgramDigest ||
			uint64(manifest) != header.ManifestRevision || uint64(planRevision) != header.PlanRevision {
			return ingestErrorJSON("identity_mismatch", -1, "plan_id mismatch")
		}
		if state == "playing" || state == "paused" {
			return ingestErrorJSON("playback_in_progress", -1, state+"; stop first")
		}
		d.mu.Lock()
		d.staged = &header
		d.state = "staged"
		d.segmentIndex = -1
		d.sampleIndex = 0
		d.tS = 0
		d.transient = ""
		d.transientPlan = nil
		d.mu.Unlock()
		raw, _ := json.Marshal(map[string]any{
			"ok": true, "kind": "preload", "trajectory_digest": digest,
			"total_sample_count": header.Sampling.TotalSampleCount,
		})
		return raw
	case "status":
		d.mu.Lock()
		d.statusRequests++
		d.mu.Unlock()
		return d.statusJSON()
	}
	return ingestErrorJSON("request_invalid", -1, "unknown kind "+kind)
}

func (d *fakeDaemon) statusJSON() []byte {
	d.mu.Lock()
	defer d.mu.Unlock()
	body := map[string]any{
		"ok": true, "kind": "status", "schema": densejointtraj.Schema,
		"store_dir": "/tmp/fake", "staged": nil, "last_error": nil,
	}
	if d.staged != nil {
		body["staged"] = map[string]any{
			"trajectory_digest":  d.staged.TrajectoryDigest,
			"plan_id":            d.staged.PlanID,
			"total_sample_count": d.staged.Sampling.TotalSampleCount,
		}
	}
	if d.executorAbsent {
		body["executor"] = "absent"
	} else {
		executor := map[string]any{
			"state": d.state, "segment_index": d.segmentIndex, "t_s": d.tS,
			"sample_index": d.sampleIndex, "torch_on": false,
			"feedback_age_ms": 5, "authority_fresh": true,
			"armed": d.armed,
			"authority": map[string]any{
				"valid": true, "fresh": true, "runtime_mode": d.runtimeMode,
				"runtime_mode_epoch": d.runtimeEpoch, "mapped_axis_mask": 511,
				"jetson_id": "jetson-test", "robot_model_id": "rosie_1400_v3",
			},
			"feedback": map[string]any{
				"valid": true, "age_ms": 5, "armed_mask": 511, "fault_mask": 0,
				"error_mask": 0, "home_valid_mask": 511,
				"axes": []map[string]any{{
					"index": 0, "q_rad": 0.1, "q_cmd_rad": nil, "pos_counts": 100,
					"home_counts": 0, "statusword": 0x27, "mode_display": 8,
					"operational": true, "fault": false, "error": false,
				}},
			},
		}
		if d.abortReason != "" {
			executor["abort_reason"] = d.abortReason
		}
		if d.staged != nil {
			executor["trajectory_digest"] = d.staged.TrajectoryDigest
		}
		executor["transient"] = nil
		if d.transient != "" {
			executor["transient"] = d.transient
			executor["transient_plan"] = d.transientPlan
		}
		body["executor"] = executor
	}
	raw, _ := json.Marshal(body)
	return raw
}
