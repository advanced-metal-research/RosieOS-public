package densejointtraj

import (
	"math"
	"testing"
)

func TestContinuousHermiteRetainsMultipleTurns(t *testing.T) {
	frame := CommandFrame{CountsPerUnit: 131072 / (2 * math.Pi), Sign: 1}
	bounds, err := BoundHermiteSegment(frame,
		HermiteKnot{TimeNS: 0, Position: -4 * math.Pi},
		HermiteKnot{TimeNS: 1_000_000_000, Position: 4 * math.Pi})
	if err != nil {
		t.Fatal(err)
	}
	if bounds.MinPosition != -262144 || bounds.MaxPosition != 262144 {
		t.Fatalf("multi-turn bounds were folded: %+v", bounds)
	}
}

func TestContinuousHermiteRefusesSigned32Overflow(t *testing.T) {
	frame := CommandFrame{CountsPerUnit: 1, Sign: 1}
	_, err := AdmitHermiteSegment(frame,
		HermiteKnot{TimeNS: 0, Position: 0},
		HermiteKnot{TimeNS: 1_000_000_000, Position: math.MaxInt32 + 1},
		-math.MaxFloat64, math.MaxFloat64)
	if err == nil {
		t.Fatal("signed-32 overflow admitted")
	}
}

func TestContinuousRecoveryDirection(t *testing.T) {
	frame := CommandFrame{CountsPerUnit: 1000, Sign: 1}
	if _, err := AdmitHermiteSegment(frame,
		HermiteKnot{TimeNS: 0, Position: 2, Velocity: -1},
		HermiteKnot{TimeNS: 1_000_000_000, Position: 1, Velocity: 0},
		-1, 1); err != nil {
		t.Fatalf("inward recovery refused: %v", err)
	}
	if _, err := AdmitHermiteSegment(frame,
		HermiteKnot{TimeNS: 0, Position: 2, Velocity: 1},
		HermiteKnot{TimeNS: 1_000_000_000, Position: 3, Velocity: 0},
		-1, 1); err == nil {
		t.Fatal("outward recovery admitted")
	}
}
