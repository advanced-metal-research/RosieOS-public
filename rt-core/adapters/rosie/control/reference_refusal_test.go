//go:build linux

package control

import (
	"math"
	"os"
	"testing"

	"rosieos/rt-core/adapters/rosie/densejointtraj"
)

// Exterior starting knots are eligible for inert recovery under APPLICATION_API;
// preserve the corpus's later velocity refusal instead of reinstating the old
// unconditional position rejection that prevented every exterior recovery.
func TestReferenceRefusalAfterExteriorRecovery(t *testing.T) {
	blob, err := os.ReadFile("../densejointtraj/testdata/corpus/dense_joint_trajectory_reference.rdt")
	if err != nil {
		t.Fatal(err)
	}
	plan, err := densejointtraj.Decode(blob)
	if err != nil {
		t.Fatal(err)
	}
	if plan.Segments[0].Samples[0].QRad[2] != .5 || plan.Segments[0].Samples[0].QdRadS[2] != 0 ||
		plan.Segments[0].Samples[1].QdRadS[0] != .5625 {
		t.Fatal("reference must start J3 at 0.5 rad, stationary, then request J1 at 0.5625 rad/s")
	}
	description, ids := programDescription()
	for i := range description.Axes {
		description.Axes[i].CountsPerUnit = 131072 / (2 * math.Pi)
		description.Axes[i].MinPosition = -.25
		description.Axes[i].MaxPosition = .25
		description.Axes[i].MaxVelocity = .05
		first := plan.Segments[0].Samples[0]
		if err := densejointtraj.AdmitHermiteKnot(densejointtraj.CommandFrame{CountsPerUnit: description.Axes[i].CountsPerUnit, Sign: 1},
			densejointtraj.HermiteKnot{Position: first.QRad[i], Velocity: first.QdRadS[i]}, -.25, .25); err != nil {
			t.Fatalf("initial recovery knot %s: %v", ids[i], err)
		}
	}
	_, err = DecodeProgram(blob, description, ids)
	if err == nil || err.Error() != "native_limit_exceeded: segment=0 sample=1 axis=J1" {
		t.Fatalf("reference refusal after exterior recovery: %v", err)
	}
}
