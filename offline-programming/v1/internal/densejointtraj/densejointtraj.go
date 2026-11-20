// Package densejointtraj is the Go twin of the robot.v4.dense-joint-trajectory.v1
// contract: the blob the dense trajectory daemon ingests over TCP.
//
// Layout: [u32 BE header_len][UTF-8 JSON header][block 0]..[block S-1], one
// packed 153-byte-record sample block per motion segment, each with its own
// clock (t_s starts at 0.0 per segment). The canonical byte layout is defined
// once — the packed block bytes are also the digest input — so this package
// must stay byte-for-byte agreed with the Python encoder
// (weld_planner/v1/python/weldplan/dense_joint_trajectory.py) and the
// C++ daemon (motion-server/joint-trajectory/v1/src/dense_joint_trajectory_format.hpp). The
// reference fixture in motion-server/joint-trajectory/v1/testdata is the arbiter: if this
// package and the fixture disagree, this package is wrong.
package densejointtraj

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"math"
	"regexp"
)

const (
	Schema         = "robot.v4.dense-joint-trajectory.v1"
	DigestDomain   = "robot.v4.dense-joint-trajectory.v1\x00"
	SampleEncoding = "f64-be-aos.v1"

	// J1..J6 arm + J7/J8 positioner + J9 side selector; the motion server's
	// kLiveAxisCount. A different count is a different schema version.
	AxisCount = 9

	// f64 t_s + 9x f64 q + 9x f64 qd + u8 flags.
	RecordBytes = 8 + 8*AxisCount + 8*AxisCount + 1

	// Matches the OLP simulator's MaxTrajectorySamples resource ceiling
	// (internal/simulator/contracts.go). ~42 minutes at the 0.01 s grid.
	MaxTotalSamples = 250_000

	// A segment's first and last sample must be numerically at rest: the
	// executor holds between segments with qd = 0, and a nonzero boundary
	// qd steps the commanded velocity at the splice.
	BoundaryQdEpsRadS = 1e-6

	// Consecutive segments must hand over in place; a bigger gap makes the
	// follower slew the jump at the qd clamp (uncommanded motion at up to
	// 3 rad/s), so it is refused, not smoothed over.
	BoundaryQEpsRad = 1e-3

	// Grid tolerance: far below any real retiming, only binary64
	// accumulation error passes.
	TimeGridEpsS = 1e-9

	// A per-sample position step may exceed limit*dt only by interpolation
	// slack.
	QStepSlack = 1.25

	FlagTorchOn = 0x01
)

// Sample is one decoded 153-byte record.
type Sample struct {
	TimeS   float64
	QRad    [AxisCount]float64
	QdRadS  [AxisCount]float64
	TorchOn bool
}

// Segment is one motion segment: its header entry plus its decoded block.
type Segment struct {
	Kind     string `json:"kind"`
	SourceID string `json:"source_id"`
	Samples  []Sample
}

// Header mirrors the JSON header. Unknown keys are ignored on decode by
// construction (encoding/json), which is the forward-compatibility contract.
type Header struct {
	Schema           string        `json:"schema"`
	PlanID           string        `json:"plan_id"`
	ProgramID        string        `json:"program_id"`
	ProgramDigest    string        `json:"program_digest"`
	TrajectoryDigest string        `json:"trajectory_digest"`
	ManifestRevision uint64        `json:"manifest_revision"`
	PlanRevision     uint64        `json:"plan_revision"`
	CreatedAt        string        `json:"created_at"`
	Axes             HeaderAxes    `json:"axes"`
	Sampling         Sampling      `json:"sampling"`
	Limits           Limits        `json:"limits"`
	Segments         []SegmentMeta `json:"segments"`
	SampleEncoding   EncodingMeta  `json:"sample_encoding"`
	// Robot is what the plan was made against (docs/ROBOT-DESCRIPTION-
	// CONTRACT.md). Optional in the format -- a blob from before the
	// contract decodes -- and required by the daemon at preload and play.
	// Identity, like plan_id: not part of the content digest.
	Robot      *Robot         `json:"robot_cell,omitempty"`
	StartState *MeasuredStart `json:"start_state,omitempty"`
	// ProcessMarkers is the program's output schedule (rt-core dense
	// process_markers, weld_planner weldplan/io_schedule.py). Header-only and
	// outside the v1 trajectory digest, so ProcessMarkersSHA256 binds it.
	ProcessMarkers       []ProcessMarker `json:"process_markers,omitempty"`
	ProcessMarkersSHA256 string          `json:"process_markers_sha256,omitempty"`
}

// ProcessMarker switches one configured output at one exact source sample.
type ProcessMarker struct {
	Monitor   string `json:"monitor,omitempty"`
	TimeoutMS uint32 `json:"timeout_ms,omitempty"`
	StableMS  uint32 `json:"stable_ms,omitempty"`
	Segment   int    `json:"segment"`
	Sample    int    `json:"sample"`
	Action    string `json:"action"`
	Value     bool   `json:"value"`
}

// MeasuredStart binds a taught trajectory to the coordinate system observed
// during planning. It is request metadata, like program_digest, not motion data.
type MeasuredStart struct {
	Schema              string    `json:"schema"`
	CoordinateIdentity  string    `json:"coordinate_identity"`
	ObservationIdentity string    `json:"observation_identity"`
	JointNames          []string  `json:"joint_names"`
	JointValues         []float64 `json:"joint_values_rad"`
}

// Robot is the header's `robot` block.
type Robot struct {
	ModelID                          string `json:"model_id"`
	RobotDescriptionSHA256           string `json:"robot_description_sha256"`
	MachinePlanningCalibrationSHA256 string `json:"machine_planning_calibration_sha256"`
	// MachinePlanningCalibrationSource is "target" (the machine_planning_calibration.json pulled off the cell) or "empty"
	// (the model's empty document, the planner could not reach the cell).
	MachinePlanningCalibrationSource string `json:"machine_planning_calibration_source"`
	// MachineConfigurationSHA256 records the machine the plan was made for
	// ("sha256:" + its rt-core configuration_sha256). Optional -- a plan made
	// offline has none -- and never compared to refuse.
	MachineConfigurationSHA256 string `json:"machine_configuration_sha256,omitempty"`
}

var (
	// Both identities are a sha256 of the same shape: the description's is
	// over the files its registry names, the cell's over machine_planning_calibration.json's bytes.
	sha256Pattern = regexp.MustCompile(`^sha256:[0-9a-f]{64}$`)
)

// validate is the format half of the contract: a block that is present must
// be whole. Whether it matches the cell is the daemon's question, at play.
func (r *Robot) validate() error {
	switch {
	case r.ModelID == "":
		return reject("robot_cell_invalid", -1, "robot.model_id is empty")
	case !sha256Pattern.MatchString(r.RobotDescriptionSHA256):
		return reject("robot_cell_invalid", -1,
			"robot.robot_description_sha256 %q is not sha256:<64 hex>", r.RobotDescriptionSHA256)
	case !sha256Pattern.MatchString(r.MachinePlanningCalibrationSHA256):
		return reject("robot_cell_invalid", -1,
			"robot.machine_planning_calibration_sha256 %q is not sha256:<64 hex>", r.MachinePlanningCalibrationSHA256)
	case r.MachineConfigurationSHA256 != "" && !sha256Pattern.MatchString(r.MachineConfigurationSHA256):
		return reject("robot_cell_invalid", -1,
			"robot.machine_configuration_sha256 %q is not sha256:<64 hex>", r.MachineConfigurationSHA256)
	case r.MachinePlanningCalibrationSource != "target" && r.MachinePlanningCalibrationSource != "empty":
		return reject("robot_cell_invalid", -1,
			"robot.machine_planning_calibration_source %q is neither target nor empty", r.MachinePlanningCalibrationSource)
	}
	return nil
}

type HeaderAxes struct {
	AxisCount    int    `json:"axis_count"`
	AxisMask     uint32 `json:"axis_mask"`
	PositionUnit string `json:"position_unit"`
	VelocityUnit string `json:"velocity_unit"`
}

type Sampling struct {
	DtS              float64 `json:"dt_s"`
	TotalSampleCount int     `json:"total_sample_count"`
	TotalDurationS   float64 `json:"total_duration_s"`
}

type Limits struct {
	MaxAbsQdRadS []float64 `json:"max_abs_qd_rad_s"`
}

type SegmentMeta struct {
	ContinuousWeld string    `json:"continuous_weld,omitempty"`
	Index          int       `json:"index"`
	Kind           string    `json:"kind"`
	SourceID       string    `json:"source_id"`
	SampleCount    int       `json:"sample_count"`
	DurationS      float64   `json:"duration_s"`
	Block          BlockMeta `json:"block"`
}

type BlockMeta struct {
	ByteOffset int    `json:"byte_offset"`
	ByteLength int    `json:"byte_length"`
	SHA256     string `json:"sha256"`
}

type EncodingMeta struct {
	Encoding    string `json:"encoding"`
	RecordBytes int    `json:"record_bytes"`
}

// Plan is a fully decoded, validated blob.
type Plan struct {
	Header   Header
	Segments []Segment
}

// RejectionError names the rule a blob broke, in the shared reason
// vocabulary, so a producer-side failure reads exactly like an ingest
// rejection from the daemon.
type RejectionError struct {
	Reason       string
	SegmentIndex int // -1 when no single segment is at fault
	Detail       string
}

func (e *RejectionError) Error() string {
	if e.SegmentIndex >= 0 {
		return fmt.Sprintf("%s: segment %d: %s", e.Reason, e.SegmentIndex, e.Detail)
	}
	return fmt.Sprintf("%s: %s", e.Reason, e.Detail)
}

func reject(reason string, segment int, format string, args ...any) error {
	return &RejectionError{
		Reason:       reason,
		SegmentIndex: segment,
		Detail:       fmt.Sprintf(format, args...),
	}
}

// EncodeBlock packs one segment's samples into canonical 153-byte records.
func EncodeBlock(samples []Sample) []byte {
	out := make([]byte, 0, len(samples)*RecordBytes)
	var raw [8]byte
	putF64 := func(value float64) {
		binary.BigEndian.PutUint64(raw[:], math.Float64bits(value))
		out = append(out, raw[:]...)
	}
	for _, sample := range samples {
		putF64(sample.TimeS)
		for _, v := range sample.QRad {
			putF64(v)
		}
		for _, v := range sample.QdRadS {
			putF64(v)
		}
		if sample.TorchOn {
			out = append(out, FlagTorchOn)
		} else {
			out = append(out, 0)
		}
	}
	return out
}

func decodeBlock(raw []byte) ([]Sample, error) {
	if len(raw)%RecordBytes != 0 {
		return nil, fmt.Errorf("block length %d is not a multiple of %d", len(raw), RecordBytes)
	}
	samples := make([]Sample, len(raw)/RecordBytes)
	for i := range samples {
		record := raw[i*RecordBytes : (i+1)*RecordBytes]
		read := func(offset int) float64 {
			return math.Float64frombits(binary.BigEndian.Uint64(record[offset:]))
		}
		samples[i].TimeS = read(0)
		for a := 0; a < AxisCount; a++ {
			samples[i].QRad[a] = read(8 + 8*a)
			samples[i].QdRadS[a] = read(8 + 8*AxisCount + 8*a)
		}
		flags := record[RecordBytes-1]
		if flags&^byte(FlagTorchOn) != 0 {
			return nil, fmt.Errorf("sample %d: reserved flag bits 0x%02x set", i, flags)
		}
		samples[i].TorchOn = flags&FlagTorchOn != 0
	}
	return samples, nil
}

// TrajectoryDigest is the content digest: what the trajectory is, not what it
// is called. It hashes the versioned big-endian layout with each segment's
// block bytes verbatim, so two runtimes cannot disagree by formatting a float
// differently. Identity strings are excluded; they are matched separately at
// play. After each block there is room for optional tagged sections (u8 tag +
// u64 length + content, hashed only when present); v1 defines none, so a
// future weld-process table can join without moving existing digests.
func TrajectoryDigest(axisMask uint32, dtS float64, limits []float64, segments []Segment) string {
	hash := sha256.New()
	_, _ = hash.Write([]byte(DigestDomain))
	writeU64 := func(value uint64) {
		var raw [8]byte
		binary.BigEndian.PutUint64(raw[:], value)
		_, _ = hash.Write(raw[:])
	}
	writeF64 := func(value float64) { writeU64(math.Float64bits(value)) }
	writeStr := func(value string) {
		writeU64(uint64(len(value)))
		_, _ = hash.Write([]byte(value))
	}
	writeU64(AxisCount)
	writeU64(uint64(axisMask))
	writeF64(dtS)
	writeU64(uint64(len(segments)))
	for _, segment := range segments {
		writeStr(segment.Kind)
		writeStr(segment.SourceID)
		writeU64(uint64(len(segment.Samples)))
		_, _ = hash.Write(EncodeBlock(segment.Samples))
		// v1: no optional sections.
	}
	writeU64(AxisCount)
	for _, v := range limits {
		writeF64(v)
	}
	return "sha256:" + hex.EncodeToString(hash.Sum(nil))
}

func validKind(kind string) bool {
	return kind == "freespace" || kind == "weld" || kind == "dwell"
}

// Decode parses and validates a blob, enforcing every rule the daemon's
// ingest enforces. Block hashes are checked before any sample parsing, so
// transport/storage corruption is reported per segment and distinctly from a
// logical-content mismatch.
func Decode(blob []byte) (*Plan, error) {
	if len(blob) < 4 {
		return nil, reject("header_truncated", -1, "%d bytes", len(blob))
	}
	headerLen := int(binary.BigEndian.Uint32(blob[:4]))
	if 4+headerLen > len(blob) {
		return nil, reject("header_truncated", -1,
			"header_len %d exceeds blob %d", headerLen, len(blob))
	}
	var header Header
	if err := json.Unmarshal(blob[4:4+headerLen], &header); err != nil {
		return nil, reject("header_json_invalid", -1, "%v", err)
	}
	if header.Schema != Schema {
		return nil, reject("schema_mismatch", -1, "schema %q", header.Schema)
	}
	if header.Axes.AxisCount != AxisCount {
		return nil, reject("axis_count_mismatch", -1,
			"axis_count %d != %d", header.Axes.AxisCount, AxisCount)
	}
	if header.SampleEncoding.Encoding != SampleEncoding ||
		header.SampleEncoding.RecordBytes != RecordBytes {
		return nil, reject("sample_encoding_mismatch", -1,
			"%q record_bytes %d", header.SampleEncoding.Encoding,
			header.SampleEncoding.RecordBytes)
	}
	if len(header.Limits.MaxAbsQdRadS) != AxisCount {
		return nil, reject("limits_invalid", -1,
			"max_abs_qd_rad_s has %d values", len(header.Limits.MaxAbsQdRadS))
	}
	for _, v := range header.Limits.MaxAbsQdRadS {
		if !(v > 0) || math.IsInf(v, 0) || math.IsNaN(v) {
			return nil, reject("limits_invalid", -1, "limit %v", v)
		}
	}
	if !(header.Sampling.DtS > 0) {
		return nil, reject("time_grid_invalid", -1, "dt_s %v", header.Sampling.DtS)
	}
	if len(header.Segments) == 0 {
		return nil, reject("segments_empty", -1, "no segments")
	}
	if header.Robot != nil {
		if err := header.Robot.validate(); err != nil {
			return nil, err
		}
	}

	body := blob[4+headerLen:]
	plan := &Plan{Header: header}
	offset := 0
	total := 0
	for i, meta := range header.Segments {
		if meta.Index != i {
			return nil, reject("segment_index_out_of_order", i, "index %d", meta.Index)
		}
		if !validKind(meta.Kind) {
			return nil, reject("kind_invalid", i, "kind %q", meta.Kind)
		}
		if meta.SampleCount < 2 {
			return nil, reject("segment_too_short", i,
				"%d samples; a segment needs at least 2", meta.SampleCount)
		}
		if meta.Block.ByteOffset != offset {
			return nil, reject("block_layout_invalid", i,
				"byte_offset %d, expected %d", meta.Block.ByteOffset, offset)
		}
		if meta.Block.ByteLength != meta.SampleCount*RecordBytes {
			return nil, reject("block_layout_invalid", i,
				"byte_length %d != sample_count %d * %d",
				meta.Block.ByteLength, meta.SampleCount, RecordBytes)
		}
		if offset+meta.Block.ByteLength > len(body) {
			return nil, reject("blob_length_mismatch", i,
				"block ends at %d, body is %d", offset+meta.Block.ByteLength, len(body))
		}
		raw := body[offset : offset+meta.Block.ByteLength]
		sum := sha256.Sum256(raw)
		if got := "sha256:" + hex.EncodeToString(sum[:]); got != meta.Block.SHA256 {
			return nil, reject("block_sha256_mismatch", i,
				"stored %s computed %s", meta.Block.SHA256, got)
		}
		samples, err := decodeBlock(raw)
		if err != nil {
			return nil, reject("reserved_flags_set", i, "%v", err)
		}
		plan.Segments = append(plan.Segments, Segment{
			Kind:     meta.Kind,
			SourceID: meta.SourceID,
			Samples:  samples,
		})
		offset += meta.Block.ByteLength
		total += meta.SampleCount
	}
	if offset != len(body) {
		return nil, reject("blob_length_mismatch", -1,
			"blocks end at %d, body is %d", offset, len(body))
	}
	if header.Sampling.TotalSampleCount != total {
		return nil, reject("total_sample_count_mismatch", -1,
			"header %d, blocks %d", header.Sampling.TotalSampleCount, total)
	}
	if total > MaxTotalSamples {
		return nil, reject("sample_count_overflow", -1,
			"%d samples > %d", total, MaxTotalSamples)
	}
	if err := validateMotion(plan); err != nil {
		return nil, err
	}
	digest := TrajectoryDigest(header.Axes.AxisMask, header.Sampling.DtS,
		header.Limits.MaxAbsQdRadS, plan.Segments)
	if digest != header.TrajectoryDigest {
		return nil, reject("trajectory_digest_mismatch", -1,
			"stored %s computed %s", header.TrajectoryDigest, digest)
	}
	return plan, nil
}

func validateMotion(plan *Plan) error {
	dt := plan.Header.Sampling.DtS
	limits := plan.Header.Limits.MaxAbsQdRadS
	var previousLastQ *[AxisCount]float64
	for i := range plan.Segments {
		segment := &plan.Segments[i]
		samples := segment.Samples
		meta := plan.Header.Segments[i]
		for s, sample := range samples {
			if math.IsNaN(sample.TimeS) || math.IsInf(sample.TimeS, 0) {
				return reject("nonfinite_sample", i, "sample %d time", s)
			}
			for a := 0; a < AxisCount; a++ {
				if math.IsNaN(sample.QRad[a]) || math.IsInf(sample.QRad[a], 0) ||
					math.IsNaN(sample.QdRadS[a]) || math.IsInf(sample.QdRadS[a], 0) {
					return reject("nonfinite_sample", i, "sample %d axis %d", s, a)
				}
				if math.Abs(sample.QdRadS[a]) > limits[a] {
					return reject("qd_limit_exceeded", i,
						"sample %d axis %d: |%.6f| rad/s > limit %.6f rad/s",
						s, a, sample.QdRadS[a], limits[a])
				}
			}
			if sample.TorchOn && segment.Kind != "weld" {
				return reject("torch_outside_weld", i,
					"torch bit set at sample %d of a %s segment", s, segment.Kind)
			}
		}
		if samples[0].TimeS != 0 {
			return reject("time_grid_invalid", i, "t_s[0] is %v, not 0.0", samples[0].TimeS)
		}
		for s := 1; s < len(samples); s++ {
			step := samples[s].TimeS - samples[s-1].TimeS
			if math.Abs(step-dt) > TimeGridEpsS && !(meta.ContinuousWeld != "" && step > 0 && step <= dt+TimeGridEpsS) {
				return reject("time_grid_invalid", i,
					"t step %.12f s at sample %d vs dt_s %v s", step, s, dt)
			}
			for a := 0; a < AxisCount; a++ {
				dq := math.Abs(samples[s].QRad[a] - samples[s-1].QRad[a])
				allowed := QStepSlack * limits[a] * dt
				if dq > allowed {
					return reject("q_step_exceeded", i,
						"sample %d axis %d: |dq| %.6f rad > %.6f rad", s, a, dq, allowed)
				}
			}
		}
		for _, boundary := range []int{0, len(samples) - 1} {
			for a := 0; a < AxisCount; a++ {
				neighbor, other := i+1, 0
				if boundary == 0 {
					neighbor = i - 1
					if neighbor >= 0 {
						other = len(plan.Segments[neighbor].Samples) - 1
					}
				}
				continuous := meta.ContinuousWeld != "" && neighbor >= 0 && neighbor < len(plan.Segments) && plan.Header.Segments[neighbor].ContinuousWeld == meta.ContinuousWeld
				if continuous {
					next := plan.Segments[neighbor].Samples[other]
					if math.Abs(samples[boundary].QdRadS[a]-next.QdRadS[a]) > 1e-12 || math.Abs(samples[boundary].QRad[a]-next.QRad[a]) > 1e-12 {
						return reject("continuous_weld_discontinuity", i, "axis %d position/velocity mismatch", a)
					}
				} else if math.Abs(samples[boundary].QdRadS[a]) > BoundaryQdEpsRadS {
					return reject("boundary_qd_nonzero", i,
						"sample %d axis %d: |%.9f| rad/s > %v rad/s",
						boundary, a, samples[boundary].QdRadS[a], BoundaryQdEpsRadS)
				}
			}
		}
		if previousLastQ != nil {
			for a := 0; a < AxisCount; a++ {
				gap := math.Abs(samples[0].QRad[a] - previousLastQ[a])
				if gap > BoundaryQEpsRad {
					return reject("boundary_q_discontinuity", i,
						"axis %d: %.6f rad gap from previous segment > %v rad",
						a, gap, BoundaryQEpsRad)
				}
			}
		}
		last := samples[len(samples)-1]
		if math.Abs(last.TimeS-meta.DurationS) > TimeGridEpsS {
			return reject("duration_mismatch", i,
				"duration_s %v vs last t_s %v", meta.DurationS, last.TimeS)
		}
		previousLastQ = &last.QRad
	}
	return nil
}
