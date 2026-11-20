package densejointtraj

import (
	"bytes"
	"encoding/binary"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// The reference fixture in motion-server/joint-trajectory/v1/testdata is the cross-language
// arbiter: the Python encoder wrote it, this package and the C++ daemon must
// read it and agree on every byte and the digest. If this test fails, this
// package is wrong (or the format changed and the fixture was regenerated —
// then all three languages fail together, which is the alarm working).

func fixturePath(t *testing.T, name string) string {
	t.Helper()
	return filepath.Join("..", "..", "..", "..", "motion-server", "joint-trajectory", "v1", "testdata", name)
}

type expected struct {
	TrajectoryDigest    string   `json:"trajectory_digest"`
	TotalSampleCount    int      `json:"total_sample_count"`
	SegmentSampleCounts []int    `json:"segment_sample_counts"`
	BlobBytes           int      `json:"blob_bytes"`
	BlockSHA256         []string `json:"block_sha256"`
}

func loadFixture(t *testing.T) ([]byte, expected) {
	t.Helper()
	blob, err := os.ReadFile(fixturePath(t, "dense_joint_trajectory_reference.rdt"))
	if err != nil {
		t.Fatalf("read fixture: %v", err)
	}
	raw, err := os.ReadFile(fixturePath(t, "dense_joint_trajectory_reference.expected.json"))
	if err != nil {
		t.Fatalf("read expected: %v", err)
	}
	var want expected
	if err := json.Unmarshal(raw, &want); err != nil {
		t.Fatalf("parse expected: %v", err)
	}
	return blob, want
}

func TestReferenceFixtureDecodesAndDigestMatches(t *testing.T) {
	blob, want := loadFixture(t)
	if len(blob) != want.BlobBytes {
		t.Fatalf("fixture is %d bytes, expected sidecar says %d", len(blob), want.BlobBytes)
	}
	plan, err := Decode(blob)
	if err != nil {
		t.Fatalf("decode: %v", err)
	}
	if plan.Header.TrajectoryDigest != want.TrajectoryDigest {
		t.Fatalf("header digest %s, expected %s",
			plan.Header.TrajectoryDigest, want.TrajectoryDigest)
	}
	got := TrajectoryDigest(plan.Header.Axes.AxisMask, plan.Header.Sampling.DtS,
		plan.Header.Limits.MaxAbsQdRadS, plan.Segments)
	if got != want.TrajectoryDigest {
		t.Fatalf("recomputed digest %s, expected %s", got, want.TrajectoryDigest)
	}
	if len(plan.Segments) != len(want.SegmentSampleCounts) {
		t.Fatalf("%d segments, expected %d", len(plan.Segments), len(want.SegmentSampleCounts))
	}
	total := 0
	for i, segment := range plan.Segments {
		if len(segment.Samples) != want.SegmentSampleCounts[i] {
			t.Fatalf("segment %d has %d samples, expected %d",
				i, len(segment.Samples), want.SegmentSampleCounts[i])
		}
		total += len(segment.Samples)
	}
	if total != want.TotalSampleCount {
		t.Fatalf("total %d samples, expected %d", total, want.TotalSampleCount)
	}
}

func TestReencodedBlocksAreByteIdentical(t *testing.T) {
	blob, _ := loadFixture(t)
	plan, err := Decode(blob)
	if err != nil {
		t.Fatalf("decode: %v", err)
	}
	// Walk the stored blocks and compare against a re-encode of the decoded
	// samples: proves decode is lossless and encode is canonical.
	headerLen := int(uint32(blob[0])<<24 | uint32(blob[1])<<16 | uint32(blob[2])<<8 | uint32(blob[3]))
	body := blob[4+headerLen:]
	offset := 0
	for i, segment := range plan.Segments {
		stored := body[offset : offset+len(segment.Samples)*RecordBytes]
		reencoded := EncodeBlock(segment.Samples)
		if !bytes.Equal(stored, reencoded) {
			t.Fatalf("segment %d: re-encoded block differs from stored bytes", i)
		}
		offset += len(stored)
	}
}

func TestFixtureCarriesTorchBitsOnlyInWeld(t *testing.T) {
	blob, _ := loadFixture(t)
	plan, err := Decode(blob)
	if err != nil {
		t.Fatalf("decode: %v", err)
	}
	torchSamples := 0
	for _, segment := range plan.Segments {
		for _, sample := range segment.Samples {
			if sample.TorchOn {
				if segment.Kind != "weld" {
					t.Fatalf("torch bit outside weld segment")
				}
				torchSamples++
			}
		}
	}
	if torchSamples != 4 {
		t.Fatalf("fixture has %d torch-on samples, the reference weld has 4", torchSamples)
	}
}

func rejectionReason(t *testing.T, err error) string {
	t.Helper()
	var rejection *RejectionError
	if !errors.As(err, &rejection) {
		t.Fatalf("expected RejectionError, got %v", err)
	}
	return rejection.Reason
}

func TestBitFlippedBlockIsRejectedBySegmentHash(t *testing.T) {
	blob, _ := loadFixture(t)
	corrupted := append([]byte(nil), blob...)
	// Flip one bit in the LAST block so the failure must name segment 1,
	// proving corruption is localized, not just detected.
	corrupted[len(corrupted)-10] ^= 0x01
	_, err := Decode(corrupted)
	if err == nil {
		t.Fatal("corrupted blob decoded")
	}
	var rejection *RejectionError
	if !errors.As(err, &rejection) {
		t.Fatalf("expected RejectionError, got %v", err)
	}
	if rejection.Reason != "block_sha256_mismatch" || rejection.SegmentIndex != 1 {
		t.Fatalf("got %s segment %d, expected block_sha256_mismatch segment 1",
			rejection.Reason, rejection.SegmentIndex)
	}
}

func TestTruncatedBlobIsRejected(t *testing.T) {
	blob, _ := loadFixture(t)
	_, err := Decode(blob[:len(blob)-RecordBytes])
	if err == nil {
		t.Fatal("truncated blob decoded")
	}
	if reason := rejectionReason(t, err); reason != "blob_length_mismatch" {
		t.Fatalf("got %s, expected blob_length_mismatch", reason)
	}
}

func TestValidationRejections(t *testing.T) {
	blob, _ := loadFixture(t)
	decode := func(t *testing.T, mutate func(plan *Plan)) error {
		t.Helper()
		plan, err := Decode(blob)
		if err != nil {
			t.Fatalf("decode: %v", err)
		}
		mutate(plan)
		return validateMotion(plan)
	}

	t.Run("boundary_qd_nonzero", func(t *testing.T) {
		// 1e-5 rad/s: an order of magnitude over the 1e-6 rest tolerance.
		err := decode(t, func(plan *Plan) {
			plan.Segments[0].Samples[0].QdRadS[3] = 1e-5
		})
		if reason := rejectionReason(t, err); reason != "boundary_qd_nonzero" {
			t.Fatalf("got %s", reason)
		}
	})
	t.Run("boundary_q_discontinuity", func(t *testing.T) {
		// 0.002 rad: twice the 1e-3 rad handover tolerance.
		err := decode(t, func(plan *Plan) {
			plan.Segments[1].Samples[0].QRad[0] += 0.002
		})
		if reason := rejectionReason(t, err); reason != "boundary_q_discontinuity" {
			t.Fatalf("got %s", reason)
		}
	})
	t.Run("qd_limit_exceeded", func(t *testing.T) {
		// 3.5 rad/s against the 3.0 rad/s ceiling.
		err := decode(t, func(plan *Plan) {
			plan.Segments[0].Samples[2].QdRadS[0] = 3.5
		})
		if reason := rejectionReason(t, err); reason != "qd_limit_exceeded" {
			t.Fatalf("got %s", reason)
		}
	})
	t.Run("q_step_exceeded", func(t *testing.T) {
		// 0.05 rad in one 0.01 s step: over 1.25 * 3.0 * 0.01 = 0.0375 rad.
		err := decode(t, func(plan *Plan) {
			plan.Segments[0].Samples[2].QRad[0] = plan.Segments[0].Samples[1].QRad[0] + 0.05
		})
		if reason := rejectionReason(t, err); reason != "q_step_exceeded" {
			t.Fatalf("got %s", reason)
		}
	})
	t.Run("torch_outside_weld", func(t *testing.T) {
		err := decode(t, func(plan *Plan) {
			plan.Segments[0].Samples[2].TorchOn = true
		})
		if reason := rejectionReason(t, err); reason != "torch_outside_weld" {
			t.Fatalf("got %s", reason)
		}
	})
	t.Run("nonfinite_sample", func(t *testing.T) {
		err := decode(t, func(plan *Plan) {
			plan.Segments[0].Samples[2].QRad[4] = nan()
		})
		if reason := rejectionReason(t, err); reason != "nonfinite_sample" {
			t.Fatalf("got %s", reason)
		}
	})
	t.Run("time_grid_invalid", func(t *testing.T) {
		// A 1 ms grid slip: 100x the 1e-9 s tolerance.
		err := decode(t, func(plan *Plan) {
			plan.Segments[0].Samples[2].TimeS += 0.001
		})
		if reason := rejectionReason(t, err); reason != "time_grid_invalid" {
			t.Fatalf("got %s", reason)
		}
	})
}

func nan() float64 {
	zero := 0.0
	return zero / zero
}

// withRobot rewrites the fixture's header with a `robot` block. The block is
// identity, outside the content digest, so the blob stays valid otherwise.
func withRobot(t *testing.T, blob []byte, robot any) []byte {
	t.Helper()
	headerLen := int(binary.BigEndian.Uint32(blob[:4]))
	var header map[string]any
	if err := json.Unmarshal(blob[4:4+headerLen], &header); err != nil {
		t.Fatal(err)
	}
	header["robot_cell"] = robot
	rewritten, err := json.Marshal(header)
	if err != nil {
		t.Fatal(err)
	}
	out := binary.BigEndian.AppendUint32(nil, uint32(len(rewritten)))
	out = append(out, rewritten...)
	return append(out, blob[4+headerLen:]...)
}

func TestRobotBlockIsOptionalAndWholeWhenPresent(t *testing.T) {
	blob, want := loadFixture(t)
	plan, err := Decode(blob)
	if err != nil {
		t.Fatal(err)
	}
	if plan.Header.Robot != nil {
		t.Fatal("the reference fixture predates the contract and has no robot block")
	}
	good := map[string]any{
		"model_id":                            "rosie_1420_v1",
		"robot_description_sha256":            "sha256:" + strings.Repeat("a", 64),
		"machine_planning_calibration_sha256": "sha256:" + strings.Repeat("b", 64),
		"machine_planning_calibration_source": "target",
	}
	plan, err = Decode(withRobot(t, blob, good))
	if err != nil {
		t.Fatalf("decode with robot: %v", err)
	}
	if plan.Header.Robot == nil || plan.Header.Robot.ModelID != "rosie_1420_v1" ||
		plan.Header.TrajectoryDigest != want.TrajectoryDigest {
		t.Fatalf("robot = %+v", plan.Header.Robot)
	}
	for field, value := range map[string]any{
		"model_id":                            "",
		"robot_description_sha256":            "abc",
		"machine_planning_calibration_sha256": strings.Repeat("b", 64),
		"machine_planning_calibration_source": "cache",
	} {
		t.Run(field, func(t *testing.T) {
			broken := map[string]any{}
			for k, v := range good {
				broken[k] = v
			}
			broken[field] = value
			_, err := Decode(withRobot(t, blob, broken))
			if reason := rejectionReason(t, err); reason != "robot_cell_invalid" {
				t.Fatalf("got %s", reason)
			}
		})
	}
}
