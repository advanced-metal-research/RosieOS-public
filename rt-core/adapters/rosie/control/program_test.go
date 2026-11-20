//go:build linux

package control

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"net/http"
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/ipcclient"
)

func TestReferenceProgramPreservesAxesClocksAndProcessRequirement(t *testing.T) {
	blob, err := os.ReadFile("../densejointtraj/testdata/corpus/dense_joint_trajectory_reference.rdt")
	if err != nil {
		t.Fatal(err)
	}
	var description ipcclient.Description
	var ids []string
	for i := 0; i < 9; i++ {
		id := fmt.Sprintf("J%d", i+1)
		ids = append(ids, id)
		description.Axes = append(description.Axes,
			ipcclient.AxisDescription{ID: id, PositionUnit: "rad", CountsPerUnit: 100000, Sign: 1, MinPosition: -3, MaxPosition: 3, MaxVelocity: 3})
	}
	plan, err := DecodeProgram(blob, description, ids)
	if err != nil {
		t.Fatal(err)
	}
	if len(plan.points) != 10 || plan.points[9].Time != 90*time.Millisecond || !plan.RequiresProcessIO || plan.Identity.ManifestRevision != 7 {
		t.Fatalf("lost segment clock/process/identity: %+v", plan)
	}
	if plan.points[5].Time != 50*time.Millisecond || plan.points[0].Position[1] != -.25 {
		t.Fatal("axis/time mapping changed")
	}
	swapped := append([]string(nil), ids...)
	swapped[0], swapped[1] = swapped[1], swapped[0]
	remapped, err := DecodeProgram(blob, description, swapped)
	if err != nil || remapped.points[0].Position[0] != -.25 {
		t.Fatal("explicit axis remap ignored", err)
	}
	description.Axes[0].MaxVelocity = .01
	if _, err := DecodeProgram(blob, description, ids); err == nil {
		t.Fatal("native speed limit ignored")
	}
	description.Axes[0].MaxVelocity = 3
	description.Axes[0].PositionUnit = "m"
	if _, err := DecodeProgram(blob, description, ids); err == nil {
		t.Fatal("rad/m mapping accepted")
	}
}

func programDescription() (ipcclient.Description, []string) {
	var description ipcclient.Description
	var ids []string
	for i := 1; i <= 9; i++ {
		id := fmt.Sprintf("J%d", i)
		ids = append(ids, id)
		description.Axes = append(description.Axes, ipcclient.AxisDescription{
			ID: id, PositionUnit: "rad", CountsPerUnit: 100000, Sign: 1, MinPosition: -3, MaxPosition: 3, MaxVelocity: 3})
	}
	return description, ids
}

// encodeProgramTest rebuilds the format's hashes after a deliberate test mutation,
// ensuring rejection exercises motion policy rather than transport corruption.
func encodeProgramTest(t *testing.T, plan *densejointtraj.Plan) []byte {
	t.Helper()
	h := plan.Header
	h.Segments = nil
	h.Sampling.TotalSampleCount, h.Sampling.TotalDurationS = 0, 0
	var body []byte
	for i, segment := range plan.Segments {
		block := densejointtraj.EncodeBlock(segment.Samples)
		sum := sha256.Sum256(block)
		duration := segment.Samples[len(segment.Samples)-1].TimeS
		h.Segments = append(h.Segments, densejointtraj.SegmentMeta{Index: i, Kind: segment.Kind, SourceID: segment.SourceID,
			SampleCount: len(segment.Samples), DurationS: duration,
			Block: densejointtraj.BlockMeta{ByteOffset: len(body), ByteLength: len(block), SHA256: "sha256:" + hex.EncodeToString(sum[:])}})
		h.Sampling.TotalSampleCount += len(segment.Samples)
		h.Sampling.TotalDurationS += duration
		body = append(body, block...)
	}
	h.TrajectoryDigest = densejointtraj.TrajectoryDigest(h.Axes.AxisMask, h.Sampling.DtS, h.Limits.MaxAbsQdRadS, plan.Segments)
	raw, err := json.Marshal(h)
	if err != nil {
		t.Fatal(err)
	}
	blob := make([]byte, 4)
	binary.BigEndian.PutUint32(blob, uint32(len(raw)))
	return append(append(blob, raw...), body...)
}

// seamProgramTest is explicitly synthetic. Two constant-position dwells differ
// by 5e-4 rad before normalisation; the canonical program must command zero counts.
func seamProgramTest(t *testing.T, dt float64) *densejointtraj.Plan {
	t.Helper()
	plan, err := densejointtraj.Decode(stationaryProgram(t, "synthetic-stationary-seam"))
	if err != nil {
		t.Fatal(err)
	}
	plan.Header.Sampling.DtS = dt
	plan.Segments = []densejointtraj.Segment{
		{Kind: "dwell", SourceID: "synthetic-first", Samples: make([]densejointtraj.Sample, 3)},
		{Kind: "dwell", SourceID: "synthetic-second", Samples: make([]densejointtraj.Sample, 3)},
	}
	for i := range plan.Segments {
		for j := range plan.Segments[i].Samples {
			sample := &plan.Segments[i].Samples[j]
			sample.TimeS = float64(j) * dt
			if i == 0 {
				sample.QRad[0] = .0005
			}
		}
	}
	return plan
}

func TestCorpusProgramsPreserveSamplesClocksAndMarkers(t *testing.T) {
	paths, err := filepath.Glob("../densejointtraj/testdata/corpus/*.rdt")
	if err != nil || len(paths) != 1 {
		t.Fatalf("expected the one committed reference, found %d: %v", len(paths), err)
	}
	description, ids := programDescription()
	for _, path := range paths {
		t.Run(filepath.Base(path), func(t *testing.T) {
			blob, err := os.ReadFile(path)
			if err != nil {
				t.Fatal(err)
			}
			source, err := densejointtraj.Decode(blob)
			if err != nil {
				t.Fatal(err)
			}
			program, err := DecodeProgram(blob, description, ids)
			if err != nil {
				t.Fatal(err)
			}
			if program.Segments != 2 || program.Samples != 11 || program.NormalisedSamples != 10 || len(program.points) != 10 || !reflect.DeepEqual(program.boundaries, []int{0, 4}) {
				t.Fatalf("corpus boundary/sample count changed: %+v", program)
			}
			if program.Identity.SourceDigest != source.Header.TrajectoryDigest || program.Identity.NormalisedDigest != source.Header.TrajectoryDigest || program.Identity.ManifestRevision != 7 || program.Identity.PlanRevision != 3 {
				t.Fatalf("reference identity changed: %+v", program.Identity)
			}
			wantMarkers := []ProcessMarker{{TimeNS: 0, Segment: 0, Sample: 0, Action: "torch", Value: false}, {TimeNS: 50000000, Segment: 1, Sample: 1, Action: "torch", Value: true}, {TimeNS: 90000000, Segment: 1, Sample: 5, Action: "torch", Value: false}}
			if !program.RequiresProcessIO || !reflect.DeepEqual(program.Markers, wantMarkers) {
				t.Fatalf("process markers: %+v", program.Markers)
			}
			wantClocks := [][]int64{{0, 10000000, 20000000, 30000000, 40000000}, {0, 10000000, 20000000, 30000000, 40000000, 50000000}}
			for i, segment := range source.Segments {
				start := program.boundaries[i]
				for j, sample := range segment.Samples {
					point := program.points[start+j]
					if int64(point.Time-program.points[start].Time) != wantClocks[i][j] || int64(math.Round(sample.TimeS*1e9)) != wantClocks[i][j] {
						t.Fatalf("local clock changed: segment %d sample %d", i, j)
					}
					for axis := 0; axis < 9; axis++ {
						if point.Position[axis] != sample.QRad[axis] || point.Velocity[axis] != sample.QdRadS[axis] {
							t.Fatalf("source axis %d changed at %d/%d", axis, i, j)
						}
					}
				}
				// Neither reference segment contains a trailing constant-position dwell.
				last := segment.Samples[len(segment.Samples)-1]
				previous := segment.Samples[len(segment.Samples)-2]
				dwellNS := int64(0)
				if last.QRad == previous.QRad && densejointtraj.Stationary(previous) {
					dwellNS = int64(math.Round((last.TimeS - previous.TimeS) * 1e9))
				}
				if dwellNS != 0 {
					t.Fatalf("reference dwell changed: %d ns", dwellNS)
				}
			}
		})
	}
}

func TestStationarySeamCanonicalSamplePreservesDwellAndProcess(t *testing.T) {
	description, ids := programDescription()
	source := seamProgramTest(t, .01)
	source.Segments[1].Kind = "weld"
	source.Segments[1].Samples[0].TorchOn = true
	source.Segments[1].Samples[1].TorchOn = true
	// 1e-6 rad/s is the declared stationary tolerance, and becomes exactly zero.
	source.Segments[0].Samples[2].QdRadS[0] = 1e-6
	source.Segments[1].Samples[0].QdRadS[0] = -1e-6
	blob := encodeProgramTest(t, source)
	program, err := DecodeProgram(blob, description, ids)
	if err != nil {
		t.Fatal(err)
	}
	if program.Samples != 6 || program.NormalisedSamples != 5 || len(program.points) != 5 || !reflect.DeepEqual(program.boundaries, []int{0, 2}) {
		t.Fatalf("shared sample missing: %+v", program)
	}
	if program.Identity.SourceDigest == program.Identity.NormalisedDigest || program.Identity.TrajectoryDigest != program.Identity.SourceDigest {
		t.Fatalf("normalisation identity missing: %+v", program.Identity)
	}
	for i, point := range program.points {
		if int64(point.Time) != int64(i)*10000000 {
			t.Fatalf("clock %d: %d ns", i, point.Time)
		}
		for axis := 0; axis < 9; axis++ {
			if point.Position[axis] != 0 || point.Velocity[axis] != 0 {
				t.Fatalf("added motion at %d/%d: %+v", i, axis, point)
			}
		}
	}
	if dwellNS := int64(program.points[2].Time - program.points[0].Time); dwellNS != 20000000 {
		t.Fatalf("lost first dwell: %d ns", dwellNS)
	}
	want := []ProcessMarker{{TimeNS: 0, Segment: 0, Sample: 0, Action: "torch", Value: false}, {TimeNS: 20000000, Segment: 1, Sample: 0, Action: "torch", Value: true}, {TimeNS: 40000000, Segment: 1, Sample: 2, Action: "torch", Value: false}}
	if !program.RequiresProcessIO || !reflect.DeepEqual(program.Markers, want) {
		t.Fatalf("lost seam process event: %+v", program.Markers)
	}
	if source.Segments[0].Samples[2].QRad[0] != .0005 {
		t.Fatal("source mutated")
	}
}

func TestMovingSegmentBoundaryDiscontinuous(t *testing.T) {
	description, ids := programDescription()
	for _, side := range []int{0, 1} {
		t.Run(fmt.Sprint(side), func(t *testing.T) {
			plan := seamProgramTest(t, .01)
			sample := 0
			if side == 0 {
				sample = 2
			}
			plan.Segments[side].Samples[sample].QdRadS[0] = 1e-5
			_, err := DecodeProgram(encodeProgramTest(t, plan), description, ids)
			var seam *SegmentBoundaryError
			if !errors.As(err, &seam) || seam.Boundary != 0 || len(seam.GapCounts) != 9 || seam.GapCounts[0] != -50 || !strings.HasPrefix(err.Error(), "segment_boundary_discontinuous: boundary=0 gap_counts=[-50 ") {
				t.Fatalf("wrong moving seam rejection: %v", err)
			}
		})
	}
}

func TestMovingBoundaryCountsFollowExplicitAxisMap(t *testing.T) {
	description, ids := programDescription()
	description.Axes[1].CountsPerUnit = 200000
	ids[0], ids[1] = ids[1], ids[0]
	plan := seamProgramTest(t, .01)
	// Motion on a different axis still makes the whole seam a moving handover.
	plan.Segments[1].Samples[0].QdRadS[8] = 1e-5
	_, err := DecodeProgram(encodeProgramTest(t, plan), description, ids)
	var seam *SegmentBoundaryError
	if !errors.As(err, &seam) || seam.GapCounts[0] != -100 || seam.GapCounts[1] != 0 {
		t.Fatalf("count diagnostic ignored source-to-native axis mapping: %v", err)
	}
}

func TestSeamNormalisationRetainsLimitsAndContinuityTolerance(t *testing.T) {
	description, ids := programDescription()
	for _, gap := range []float64{.001, .001001} {
		plan := seamProgramTest(t, .01)
		for i := range plan.Segments[0].Samples {
			plan.Segments[0].Samples[i].QRad[0] = gap
		}
		_, err := DecodeProgram(encodeProgramTest(t, plan), description, ids)
		if (gap == .001) != (err == nil) {
			t.Fatalf("gap %g rad: %v", gap, err)
		}
	}
	for _, gap := range []float64{1e-13, 2e-12} {
		plan := seamProgramTest(t, .01)
		for i := range plan.Segments[0].Samples {
			plan.Segments[0].Samples[i].QRad[0] = gap
		}
		plan.Segments[0].Samples[2].QdRadS[0] = 1e-5
		_, err := DecodeProgram(encodeProgramTest(t, plan), description, ids)
		var seam *SegmentBoundaryError
		if err == nil || errors.As(err, &seam) != (gap == 2e-12) {
			t.Fatalf("continuity threshold changed at %g rad: %v", gap, err)
		}
	}
	plan := seamProgramTest(t, .01)
	plan.Header.Limits.MaxAbsQdRadS[0] = 3
	plan.Segments[0].Samples[0].QRad[0] = 0
	plan.Segments[0].Samples[1].QRad[0], plan.Segments[0].Samples[2].QRad[0] = .0009, .0009
	for i := range plan.Segments[1].Samples {
		plan.Segments[1].Samples[i].QRad[0] = .0014
	}
	description.Axes[0].MaxVelocity = .1
	if _, err := DecodeProgram(encodeProgramTest(t, plan), description, ids); err == nil || err.Error() != "native_segment_rate_exceeded" {
		t.Fatalf("normalised incoming rate must be revalidated: %v", err)
	}
}

func prepareProgramHTTP(t *testing.T, api *handleAPI, fence Fence, blob []byte) *Program {
	t.Helper()
	request, err := http.NewRequest(http.MethodPost, "http://unix/v1/program", bytes.NewReader(blob))
	if err != nil {
		t.Fatal(err)
	}
	request.Header.Set("X-Control-Session", fence.Session)
	request.Header.Set("X-Control-Generation", fmt.Sprint(fence.Generation))
	response, err := api.client.Do(request)
	if err != nil {
		t.Fatal(err)
	}
	defer response.Body.Close()
	var result struct {
		Error     string  `json:"error"`
		Operation string  `json:"operation"`
		Data      Program `json:"data"`
	}
	if err := json.NewDecoder(response.Body).Decode(&result); err != nil {
		t.Fatal(err)
	}
	if result.Error != "" || result.Operation != "prepare_program" {
		t.Fatalf("prepare_program: %+v", result)
	}
	return &result.Data
}

func TestSyntheticStationarySeamCompletesWithoutBridgeMotion(t *testing.T) {
	f := startLifecycleCore(t)
	api := serveHandleAPI(t, f.controller, nil)
	blob := encodeProgramTest(t, seamProgramTest(t, .1))
	normalised, err := DecodeProgram(blob, f.controller.description, f.controller.axes)
	if err != nil {
		t.Fatal(err)
	}
	program := prepareProgramHTTP(t, api, f.fence, blob)
	if program.Identity.SourceDigest == program.Identity.NormalisedDigest {
		t.Fatal("prepared identity lost normalisation")
	}
	metadata := f.controller.DescribeProgram()
	if metadata == nil || metadata.Identity != program.Identity {
		t.Fatalf("metadata identity differs: %+v", metadata)
	}
	metadata.Identity.NormalisedDigest = "changed-by-reader"
	metadata.Markers[0].Value = true
	if f.controller.DescribeProgram().Identity != program.Identity || f.controller.DescribeProgram().Markers[0].Value {
		t.Fatal("metadata aliases prepared identity/markers")
	}
	request := controlRequest(f.fence, "start_program")
	for _, field := range []string{"source", "normalised", "both"} {
		request.Identity = program.Identity
		if field == "source" || field == "both" {
			request.Identity.SourceDigest = ""
		}
		if field == "normalised" || field == "both" {
			request.Identity.NormalisedDigest = ""
		}
		api.call(t, request, "program_identity_mismatch")
	}
	request.Identity = program.Identity
	started := api.call(t, request, "")
	seenBoundary := false
	observations := 0
	awaitHandleCondition(t, func() bool {
		state, motion, execution := api.snapshot(t)
		if motion.CommandSequence != started.Sequence {
			return false
		}
		if len(state.Axes) != 9 {
			t.Fatalf("missing feedback axes: %d", len(state.Axes))
		}
		for axis, feedback := range state.Axes {
			// All canonical dwell samples are exactly zero. This is stricter than
			// simulation-program's four-count completion tolerance.
			if feedback.PositionCounts != 0 {
				t.Fatalf("bridge motion: axis %d feedback %d counts", axis, feedback.PositionCounts)
			}
			for _, point := range normalised.points {
				if point.Position[axis] != 0 || point.Velocity[axis] != 0 {
					t.Fatal("canonical dwell contains motion")
				}
			}
		}
		observations++
		if motion.PointIndex == 2 {
			seenBoundary = true
		}
		if motion.Done && motion.State != 4 {
			t.Fatalf("execution failed: %+v", motion)
		}
		return execution.State == "completed" && motion.Done && motion.State == 4
	})
	if !seenBoundary || observations == 0 {
		t.Fatalf("missing seam observations: seen=%t observations=%d", seenBoundary, observations)
	}
}

// A six-axis cell on the nine-column wire: the cell maps six ids to the first
// six columns and the three spare columns must be uncommanded and still. The
// LA bench (Rosie 1420, dense_axis_ids J1..J6) was refused with
// dense_axis_map_requires_nine_ids on 2026-09-20 before this.
func TestDecodeProgramMapsFewerAxesThanTheWire(t *testing.T) {
	blob, err := os.ReadFile("../densejointtraj/testdata/corpus/dense_joint_trajectory_reference.rdt")
	if err != nil {
		t.Fatal(err)
	}
	plan, err := densejointtraj.Decode(blob)
	if err != nil {
		t.Fatal(err)
	}
	nine, ids := programDescription()
	six := ipcclient.Description{Axes: append([]ipcclient.AxisDescription(nil), nine.Axes[:6]...)}
	still := func() *densejointtraj.Plan {
		copied := *plan
		copied.Header.Axes.AxisMask = 0x3f
		copied.Segments = nil
		for _, segment := range plan.Segments {
			samples := append([]densejointtraj.Sample(nil), segment.Samples...)
			for i := range samples {
				for a := 6; a < densejointtraj.AxisCount; a++ {
					samples[i].QRad[a], samples[i].QdRadS[a] = 0, 0
				}
			}
			copied.Segments = append(copied.Segments, densejointtraj.Segment{Kind: segment.Kind, SourceID: segment.SourceID, Samples: samples})
		}
		return &copied
	}
	program, err := DecodeProgram(encodeProgramTest(t, still()), six, ids[:6])
	if err != nil {
		t.Fatal(err)
	}
	if program.AxisMask != 0x3f || len(program.points) == 0 || len(program.points[0].Position) != 6 {
		t.Fatalf("six-axis program: mask %#x points %d", program.AxisMask, len(program.points))
	}
	// A spare column the header commands is refused, and so is one that moves.
	commanded := still()
	commanded.Header.Axes.AxisMask = 0x7f
	if _, err := DecodeProgram(encodeProgramTest(t, commanded), six, ids[:6]); err == nil || !strings.HasPrefix(err.Error(), ReasonUnmappedDenseAxis) {
		t.Fatalf("commanded spare accepted: %v", err)
	}
	moving := still()
	moving.Segments[0].Samples[len(moving.Segments[0].Samples)-1].QRad[7] = 0.1
	if _, err := DecodeProgram(encodeProgramTest(t, moving), six, ids[:6]); err == nil || !strings.HasPrefix(err.Error(), ReasonUnmappedDenseAxis) {
		t.Fatalf("moving spare accepted: %v", err)
	}
	// Zero ids, or more than the wire holds, keep the catalogue's refusal.
	for _, bad := range [][]string{nil, append(append([]string(nil), ids...), "J10")} {
		if _, err := DecodeProgram(encodeProgramTest(t, still()), nine, bad); err == nil || err.Error() != ReasonDenseAxisMapRequiresNineIds {
			t.Fatalf("%d ids: %v", len(bad), err)
		}
	}
	// Nine ids on a nine-axis cell are unchanged.
	if _, err := DecodeProgram(blob, nine, ids); err != nil {
		t.Fatal(err)
	}
}
