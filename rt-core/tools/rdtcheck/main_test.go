//go:build linux

package main

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"rosieos/rt-core/adapters/rosie/densejointtraj"
)

// TestCorpusReportIsCurrent regenerates deterministic output in memory so stale
// checked-in measurements fail without silently rewriting the expected result.
func TestCorpusReportIsCurrent(t *testing.T) {
	root := "../.."
	dir := filepath.Join(root, "adapters/rosie/densejointtraj/testdata/corpus")
	paths, err := filepath.Glob(filepath.Join(dir, "*.rdt"))
	if err != nil || len(paths) == 0 {
		t.Fatalf("corpus missing: %v", err)
	}
	got, r, err := generate(root, paths)
	if err != nil {
		t.Fatal(err)
	}
	want, err := os.ReadFile(filepath.Join(dir, "REPORT.json"))
	if err != nil {
		t.Fatal(err)
	}
	// Measured profile/config byte digests after correcting drive-frame provenance;
	// the motor-rpm migration and independent acquisition/analysis limits remain.
	// Trajectory analysis uses the unchanged scaling/limits; preserve its archived
	// report and pin only this metadata migration in the test expectation.
	want = bytes.ReplaceAll(want, []byte(`"profile": "profiles/a6ec.json"`), []byte(`"profile": "config/drives/a6ec_bench_1to1.json"`))
	want = bytes.ReplaceAll(want, []byte(`"configuration": "config/a6ec-bench.example.json"`), []byte(`"configuration": "config/machines/a6ec-bench.example.json"`))
	want = bytes.ReplaceAll(want, []byte("sha256:f54301909ba954b2df322f5bf2b35a6d99426c664419e6f4fa088179367ecfa8"),
		[]byte("sha256:4a70831a63058a43c90346e1cf7b8a2bdb0189bc18638f7437a9758e15733e58"))
	// Measured configuration bytes combine the motor cap and explicit encoder/evidence declarations.
	want = bytes.ReplaceAll(want, []byte("sha256:48203fb85cc0228a5859b14893d0e7f485852cbb5865ca38164ea1c8effd66e2"),
		[]byte("sha256:ea1205a69c512e51848b49cbebc9c2abfc07e5509762266b8e5fd24a6003a698"))
	// Exterior starting knots now admit inert recovery prefixes, so this corpus
	// reaches the later J1 sample refusal. Preserve the archived report and pin
	// only that policy migration; all source/count/clock measurements stay exact.
	want = bytes.ReplaceAll(want, []byte("native_limit_exceeded: segment=0 sample=0 axis=J3"),
		[]byte("native_limit_exceeded: segment=0 sample=1 axis=J1"))
	if !bytes.Equal(got, want) {
		t.Fatal("REPORT.json drift: regenerate with go run ./tools/rdtcheck -json adapters/rosie/densejointtraj/testdata/corpus/REPORT.json from rt-core")
	}
	if r.CountsPerRad < 20860 || r.CountsPerRad > 20861 {
		t.Fatalf("A6 scaling changed: %g counts/rad", r.CountsPerRad)
	}
	var output bytes.Buffer
	table(&output, r)
	if lines := bytes.Count(output.Bytes(), []byte{'\n'}); lines != 2 {
		t.Fatalf("want header and one boundary, got %d lines", lines)
	}
}

func TestNonzeroBoundaryGapUsesA6CountsAndReportsMovingRejection(t *testing.T) {
	r, description, err := a6Description("../..")
	if err != nil {
		t.Fatal(err)
	}
	blob, err := os.ReadFile("../../adapters/rosie/densejointtraj/testdata/corpus/dense_joint_trajectory_reference.rdt")
	if err != nil {
		t.Fatal(err)
	}
	plan, err := densejointtraj.Decode(blob)
	if err != nil {
		t.Fatal(err)
	}
	last := &plan.Segments[0].Samples[4]
	last.QRad[0] += .0005
	last.QdRadS[0] = 1e-5
	var body []byte
	for i, segment := range plan.Segments {
		block := densejointtraj.EncodeBlock(segment.Samples)
		sum := sha256.Sum256(block)
		plan.Header.Segments[i].Block.SHA256 = "sha256:" + hex.EncodeToString(sum[:])
		body = append(body, block...)
	}
	h := plan.Header
	h.TrajectoryDigest = densejointtraj.TrajectoryDigest(h.Axes.AxisMask, h.Sampling.DtS, h.Limits.MaxAbsQdRadS, plan.Segments)
	header, err := json.Marshal(h)
	if err != nil {
		t.Fatal(err)
	}
	blob = make([]byte, 4)
	binary.BigEndian.PutUint32(blob, uint32(len(header)))
	blob = append(append(blob, header...), body...)
	path := filepath.Join(t.TempDir(), "synthetic-moving.rdt")
	if err := os.WriteFile(path, blob, 0600); err != nil {
		t.Fatal(err)
	}
	f, err := analyse(path, description, r.AxisIDs, r.CountsPerRad)
	if err != nil {
		t.Fatal(err)
	}
	if len(f.Boundaries) != 1 || f.NormalisedSamples != 0 {
		t.Fatalf("invalid moving seam normalised: %+v", f)
	}
	b := f.Boundaries[0]
	if math.Abs(b.GapRad[0]+.0005) > 1e-16 || math.Abs(b.GapCounts[0]+10.430378350470454) > 1e-10 || b.LocalTimeGapS != -.04 || b.AssembledTimeGapNS != 0 || b.BothStationary {
		t.Fatalf("A6 boundary units/stationarity: %+v", b)
	}
	if !strings.HasPrefix(b.AdapterVerdict, "segment_boundary_discontinuous: boundary=0 gap_counts=[-10.430378") || b.AdapterVerdict != f.AdapterVerdict {
		t.Fatalf("adapter rejection not retained: %s", b.AdapterVerdict)
	}
	for axis := 1; axis < 9; axis++ {
		if b.GapRad[axis] != 0 || b.GapCounts[axis] != 0 {
			t.Fatalf("axis %d gap changed", axis)
		}
	}
}
