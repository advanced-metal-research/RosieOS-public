package densejointtraj

import (
	"encoding/json"
	"reflect"
	"testing"
)

func dwellNormalisationPlan(positions ...float64) *Plan {
	plan := &Plan{Header: Header{Sampling: Sampling{DtS: .01}, Limits: Limits{MaxAbsQdRadS: []float64{3, 3, 3, 3, 3, 3, 3, 3, 3}}}}
	for i, position := range positions {
		samples := []Sample{{TimeS: 0}, {TimeS: .01}, {TimeS: .02}}
		for j := range samples {
			samples[j].QRad[0] = position
			samples[j].QRad[1] = -.25
		}
		plan.Segments = append(plan.Segments, Segment{Kind: "dwell", SourceID: "synthetic", Samples: samples})
		plan.Header.Segments = append(plan.Header.Segments, SegmentMeta{Index: i, Kind: "dwell", DurationS: .02, SampleCount: 3})
	}
	return plan
}

func TestStationaryNormalisationPreservesSourceAndLocalClocks(t *testing.T) {
	source := dwellNormalisationPlan(.0005, .00025, 0)
	before, err := json.Marshal(source)
	if err != nil {
		t.Fatal(err)
	}
	normalised, err := NormaliseStationary(source)
	if err != nil {
		t.Fatal(err)
	}
	after, err := json.Marshal(source)
	if err != nil {
		t.Fatal(err)
	}
	if string(before) != string(after) {
		t.Fatal("normalisation mutated source")
	}
	if len(normalised) != 3 {
		t.Fatalf("lost segments: %d", len(normalised))
	}
	for i, segment := range normalised {
		if segment.Kind != source.Segments[i].Kind || segment.SourceID != source.Segments[i].SourceID || len(segment.Samples) != 3 {
			t.Fatal("segment metadata changed")
		}
		for j, sample := range segment.Samples {
			if sample.TimeS != []float64{0, .01, .02}[j] || sample.QRad[0] != 0 || sample.QRad[1] != -.25 || sample.QdRadS != [AxisCount]float64{} || sample.TorchOn {
				t.Fatalf("changed clock/axes/process: %d/%d %+v", i, j, sample)
			}
		}
	}
	copyPlan := *source
	copyPlan.Segments = normalised
	again, err := NormaliseStationary(&copyPlan)
	if err != nil || !reflect.DeepEqual(again, normalised) {
		t.Fatalf("normalisation is not idempotent: %v", err)
	}
}

func TestStationaryNormalisationRejectsAccumulatedDwellDisplacement(t *testing.T) {
	// Each 7.5e-4 rad seam is legal independently, but shifting the initial
	// dwell by 1.5e-3 rad would exceed the source format's displacement bound.
	plan := dwellNormalisationPlan(.0015, .00075, 0)
	if err := validateMotion(plan); err != nil {
		t.Fatal(err)
	}
	if _, err := NormaliseStationary(plan); err == nil || rejectionReason(t, err) != "boundary_q_discontinuity" {
		t.Fatalf("accumulated correction accepted: %v", err)
	}
}

func TestInspectRetainsInvalidMotionOnlyAfterDigestVerification(t *testing.T) {
	blob, _ := loadFixture(t)
	plan, err := Inspect(blob)
	if err != nil || plan == nil {
		t.Fatal(err)
	}
	blob[len(blob)-2] ^= 1
	if plan, err := Inspect(blob); plan != nil || err == nil {
		t.Fatalf("corrupt bytes exposed as a verified plan: %v", err)
	}
}

func TestContinuousWeldRetainsSharedMovingBoundaryAndPartialClock(t *testing.T) {
	plan := dwellNormalisationPlan(0, 0)
	for i := range plan.Segments {
		plan.Header.Segments[i].ContinuousWeld = "weld-1"
		plan.Segments[i].Kind = "freespace"
	}
	a, b := plan.Segments[0].Samples, plan.Segments[1].Samples
	a[1].QRad[0] = .0001
	a[1].QdRadS[0] = .01
	a[2].TimeS = .015
	a[2].QRad[0] = .00015
	a[2].QdRadS[0] = .01
	b[0].QRad[0] = .00015
	b[0].QdRadS[0] = .01
	b[1].QRad[0] = .00025
	b[1].QdRadS[0] = .01
	b[2].QRad[0] = .0003
	plan.Header.Segments[0].DurationS = .015
	got, err := NormaliseStationary(plan)
	if err != nil || !reflect.DeepEqual(got, plan.Segments) {
		t.Fatalf("changed continuous boundary: %v", err)
	}
	b[0].QdRadS[0] = .009
	if _, err = NormaliseStationary(plan); err == nil || rejectionReason(t, err) != "continuous_weld_discontinuity" {
		t.Fatalf("velocity discontinuity accepted: %v", err)
	}
	b[0].QdRadS[0] = .01
	plan.Header.Segments[1].ContinuousWeld = "other"
	if _, err = NormaliseStationary(plan); err == nil {
		t.Fatal("unrelated moving boundary accepted")
	}
}
