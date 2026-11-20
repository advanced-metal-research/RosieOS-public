//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"testing"
	"time"
)

// Native scalar readers preserve U32 bit patterns for generic diagnostics.
// Home must separately recover the declared mathematical value, apply its sign
// in a wider type, and only then check the signed offset's representable range.
func TestHomeTruthScalarSignCheckedBeforeNarrowing(t *testing.T) {
	for _, test := range []struct {
		name       string
		sourceType string
		writeBits  int32
		sign       int
		accepted   bool
		offset     int64
	}{
		{"u32_max_positive", "u32", -1, 1, false, 0},
		{"u32_max_negative", "u32", -1, -1, false, 0},
		{"u32_sign_boundary_positive", "u32", -2147483648, 1, false, 0},
		// Scalar conversion itself accepts this mathematical value (covered by
		// commissioning_tests); Home still refuses the unusable INT32_MIN frame.
		{"u32_sign_boundary_negative", "u32", -2147483648, -1, false, -2147483648},
		{"u32_signed_max_positive", "u32", 2147483647, 1, true, 2147483647},
		{"u32_signed_max_negative", "u32", 2147483647, -1, true, -2147483647},
		{"i32_min_positive", "i32", -2147483648, 1, true, -2147483648},
	} {
		t.Run(test.name, func(t *testing.T) {
			machine, profile := compiledCoordinateInputs(t)
			profile["home_truth_sign"] = test.sign
			home := profile["native_home"].(map[string]any)
			home["truth_source"].(map[string]any)["type"] = test.sourceType
			for _, value := range home["transaction"].([]any) {
				step := value.(map[string]any)
				if step["op"] == "write_sdo" && step["index"] == float64(24700) {
					// Write the same four bytes through I32; the readback type is
					// independently varied above.
					step["value"] = test.writeBits
				}
			}
			limits := machine["axes"].([]any)[0].(map[string]any)["limits"].(map[string]any)
			// This exercises Home scalar conversion, not command-space span.
			// Keep the axis range representable under the linear-mode int32 gate.
			limits["min"], limits["max"] = -1, 1
			compiled, digest, err := compileCoordinateJSON(t, machine, profile)
			if err != nil {
				t.Fatalf("compile Home scalar fixture: %v: %s", err, digest)
			}
			f, _ := coordinateModelDaemon(t, compiled, digest)
			seq, err := f.client.Home(context.Background(), 1)
			mustSubmit(t, seq, err)
			ctx, cancel := context.WithTimeout(context.Background(), 8*time.Second)
			defer cancel()
			result, err := f.client.WaitResult(ctx, seq)
			if test.accepted {
				if err != nil || result.Result != ResultAccepted {
					t.Fatalf("representable signed Home offset rejected: %+v %v", result, err)
				}
			} else {
				var rejected *RejectedError
				if !errors.As(err, &rejected) || result.Reason != ReasonNotReady {
					t.Fatalf("out-of-range mathematical Home offset was narrowed into success: %+v %v", result, err)
				}
			}
			finished := monotonicNS()
			terminal := waitCoordinateObservation(t, f, 1, func(s coordinateObservation) bool {
				if s.Armed != 0 || s.AxisEnableMask != 0 {
					t.Fatalf("Home conversion test enabled motion: %+v", s)
				}
				if !test.accepted && (s.HomeValidMask != 0 || s.Axes[0].HomeState == 2) {
					t.Fatalf("invalid scalar conversion published Home success: %+v", s)
				}
				if s.TimeNS <= finished || s.CommissioningPhase != 0 {
					return false
				}
				wantedMask, wantedState := uint32(0), uint32(3)
				if test.accepted {
					wantedMask, wantedState = 1, 2
				}
				if s.HomeValidMask != wantedMask || s.Axes[0].HomeState != wantedState {
					t.Fatalf("Home scalar result disagrees with terminal state: %+v", s)
				}
				return true
			})
			raw, err := os.ReadFile(filepath.Join(filepath.Dir(f.options.SocketPath), "metrics.json"))
			if err != nil {
				t.Fatal(err)
			}
			var truth struct {
				TimeNS uint64 `json:"time_ns"`
				Axes   []struct {
					Offset *int64 `json:"native_home_position_offset"`
				} `json:"axes"`
			}
			if err := json.Unmarshal(raw, &truth); err != nil {
				t.Fatal(err)
			}
			if truth.TimeNS < terminal.TimeNS || len(truth.Axes) != 1 || truth.Axes[0].Offset == nil || *truth.Axes[0].Offset != test.offset {
				t.Fatalf("Home offset was altered, narrowed, or published on failure: %+v wanted=%d", truth, test.offset)
			}
		})
	}
}
