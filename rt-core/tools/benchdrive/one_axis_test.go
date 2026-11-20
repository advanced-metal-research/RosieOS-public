//go:build linux

package main

import (
	"bytes"
	"context"
	"encoding/json"
	"io"
	"path/filepath"
	"strings"
	"testing"

	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// withoutMask exercises omission through the public CLI; explicitly selecting
// one bit would hide the startup failures on the one-axis bench.
func withoutMask(args []string) []string {
	var result []string
	for i := 0; i < len(args); i++ {
		if args[i] == "--axis-mask" {
			i++
			continue
		}
		result = append(result, args[i])
	}
	return result
}

// TestOneAxisSimulationPhases requires every mini-bench phase to complete using
// the compiled one-axis description and records integer feedback after motion.
func TestOneAxisSimulationPhases(t *testing.T) {
	f := startFixtureAxes(t, true, false, 1)
	// Probe missing anchors before Home; later motion intentionally changes the
	// saved encoder position and must not be mistaken for a restorable anchor.
	for _, phase := range []string{"inhibited", "recovery", "home", "enable", "jog", "move", "discard", "telemetry"} {
		t.Run(phase, func(t *testing.T) {
			args := withoutMask(f.args(phase, phase, true))
			// Unverified fixture observation windows: hold 100000000 ns, jog 300000000 ns.
			args = append(args, "--hold-ns", "100000000", "--jog-duration-ns", "300000000")
			var out bytes.Buffer
			if code := run(context.Background(), args, &out); code != 0 {
				t.Fatalf("%s exit=%d: %s", phase, code, &out)
			}
			records := readRecords(t, filepath.Join(f.build, phase+".jsonl"))
			last := records[len(records)-1]
			if last.ExitCode != 0 || len(last.NotMet) != 0 {
				t.Fatalf("incomplete phase: %+v", last)
			}
			statuses := 0
			for _, record := range records {
				if record.Kind == "request" && (record.Operation == "home" || record.Operation == "enable" || record.Operation == "reset_fault" || record.Operation == "restore_anchor") {
					var request control.Request
					simfixture.Must(t, json.Unmarshal(record.Request, &request))
					if request.AxisMask != 1 {
						t.Fatalf("%s mask=%d", record.Operation, request.AxisMask)
					}
				}
				if record.Kind != "status" {
					continue
				}
				var s struct {
					Core struct {
						Axes []struct {
							Position *int32 `json:"position_counts"`
							Native   *int32 `json:"pos_counts"`
						} `json:"axes"`
					} `json:"core"`
				}
				simfixture.Must(t, json.Unmarshal(record.Response, &s))
				if len(s.Core.Axes) != 1 || s.Core.Axes[0].Position == nil || s.Core.Axes[0].Native == nil || *s.Core.Axes[0].Position != *s.Core.Axes[0].Native {
					t.Fatalf("feedback lost: %s", record.Response)
				}
				statuses++
			}
			if statuses < 2 {
				t.Fatalf("only %d status observations", statuses)
			}
			f.poll(t, func(s control.Status) bool {
				return s.Core.AxisEnableMask == 0 && s.Core.Armed == 0 && s.Core.LeaseValid == 0
			})
		})
		if t.Failed() {
			return
		}
	}
}

// TestOneAxisProgramAdmission derives the enable mask from Describe while
// refusing an unsupported dense format before acquiring authority or enabling.
func TestOneAxisProgramAdmission(t *testing.T) {
	f := startFixtureAxes(t, false, false, 1)
	d, err := f.client.Describe(context.Background())
	simfixture.Must(t, err)
	r := newRunner(options{phase: "program", mask: 1}, nil, nil, io.Discard)
	r.description = d
	if r.enabledMask() != 1 {
		t.Fatalf("program enable mask=%d", r.enabledMask())
	}
	var out bytes.Buffer
	if code := run(context.Background(), withoutMask(f.args("program", "program", true)), &out); code != 1 || !strings.Contains(out.String(), "program requires exactly nine J1..J9 axes") {
		t.Fatalf("program admission=%d: %s", code, &out)
	}
	for _, record := range readRecords(t, filepath.Join(f.build, "program.jsonl")) {
		if record.Operation == "acquire" || record.Operation == "enable" {
			t.Fatalf("unsupported program reached %s", record.Operation)
		}
	}
}

// TestPositionCountsWireName preserves the measured Pi value decoded from the
// native extension, preventing a public spelling from silently decoding zero.
func TestPositionCountsWireName(t *testing.T) {
	var status control.Status
	simfixture.Must(t, json.Unmarshal([]byte(`{"core":{"axes":[{"pos_counts":65532}]},"axes":[{"position_counts":65532}]}`), &status))
	if status.Core.Axes[0].PositionCounts != 65532 || status.Axes[0].PositionCounts != 65532 {
		t.Fatal("position count wire names changed")
	}
	for _, count := range []int{0, 1, 9, 16, 17} {
		d := control.Description{}
		d.Axes = make([]ipcclient.AxisDescription, count)
		want := map[int]uint32{0: 0, 1: 1, 9: 511, 16: 65535, 17: 0}[count]
		if got := describedAxisMask(d); got != want {
			t.Fatalf("count=%d mask=%d want=%d", count, got, want)
		}
	}
}
