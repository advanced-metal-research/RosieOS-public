//go:build linux

package main

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"

	admission "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

func programGeometry() (control.Description, options, control.Status) {
	d := control.Description{NativeDescription: ipcclient.Description{CycleNS: 1000000, MaxTrajectoryPoints: 32768}}
	s := control.Status{}
	for i := 0; i < 9; i++ {
		d.Axes = append(d.Axes, ipcclient.AxisDescription{Index: uint32(i), ID: fmt.Sprintf("J%d", i+1), PositionUnit: "rad",
			CountsPerUnit: 131072 / (2 * math.Pi), Sign: 1,
			MinPosition: -3, MaxPosition: 3, MaxVelocity: 0.5, JogAcceleration: 1, CompletionTolerance: 0.0002})
		s.Core.Axes = append(s.Core.Axes, ipcclient.AxisStatus{LogicalValid: true, LogicalTarget: float64(i) / 10, LogicalPosition: -1})
	}
	return d, options{mask: 257, delta: 0.2, velocityFraction: 0.5, accelFraction: 0.5, binding: control.Binding{Revision: 7}}, s
}

func TestProgramProfileCorpusGridAndBounds(t *testing.T) {
	raw, err := os.ReadFile("../../adapters/rosie/densejointtraj/testdata/corpus/dense_joint_trajectory_reference.rdt")
	simfixture.Must(t, err)
	corpus, err := densejointtraj.Decode(raw)
	simfixture.Must(t, err)
	if corpus.Header.Sampling.DtS != 0.01 {
		t.Fatal("corpus grid changed; review the bench program grid")
	}
	for _, delta := range []float64{0.2, -0.2, 0.001} {
		d, o, s := programGeometry()
		o.delta = delta
		plan, err := programProfile(d, o, s)
		simfixture.Must(t, err)
		blob, err := densejointtraj.Encode(plan.Header, plan.Segments)
		simfixture.Must(t, err)
		decoded, err := densejointtraj.Decode(blob)
		simfixture.Must(t, err)
		if decoded.Header.Sampling.DtS != 0.01 || decoded.Header.Axes.AxisMask != 511 || len(decoded.Segments) != 1 {
			t.Fatal("program did not preserve the nine-axis corpus convention")
		}
		samples := decoded.Segments[0].Samples
		if len(samples) < 3 || len(samples) >= 32768 {
			t.Fatalf("invalid sample count %d", len(samples))
		}
		for axis := 0; axis < 9; axis++ {
			origin, endpoint := float64(axis)/10, float64(axis)/10
			if axis == 0 || axis == 8 {
				endpoint += delta
			}
			if samples[0].QRad[axis] != origin || math.Abs(samples[len(samples)-1].QRad[axis]-endpoint) > 1e-12 ||
				samples[0].QdRadS[axis] != 0 || samples[len(samples)-1].QdRadS[axis] != 0 {
				t.Fatalf("axis %d boundary position or velocity", axis)
			}
			for i, sample := range samples {
				if sample.TorchOn || math.Abs(sample.QdRadS[axis]) > 0.1875 {
					t.Fatal("process bit or velocity exceeds bench limits")
				}
				if axis != 0 && axis != 8 && (sample.QRad[axis] != origin || sample.QdRadS[axis] != 0) {
					t.Fatal("unselected axis moved")
				}
				if i > 0 {
					prev := samples[i-1]
					if math.Abs(sample.TimeS-prev.TimeS-0.01) > 1e-12 || math.Abs(sample.QRad[axis]-prev.QRad[axis])*100 > 0.1875 ||
						math.Abs(sample.QdRadS[axis]-prev.QdRadS[axis])*100 > 0.500001 {
						t.Fatal("grid, position-derived velocity or acceleration bound exceeded")
					}
				}
			}
		}
	}
	for _, name := range []string{"mask", "target", "stationary_target", "soft_limit", "count", "order", "capacity", "velocity", "delta", "counts_per_unit", "sign"} {
		t.Run(name, func(t *testing.T) {
			d, o, s := programGeometry()
			switch name {
			case "mask":
				o.mask = 512
			case "target":
				s.Core.Axes[0].LogicalTarget = math.NaN()
			case "stationary_target":
				s.Core.Axes[1].LogicalTarget = 4
			case "soft_limit":
				s.Core.Axes[0].LogicalTarget = 2.9
			case "count":
				d.Axes = d.Axes[:8]
			case "order":
				d.Axes[0].ID = "J9"
			case "capacity":
				d.MaxTrajectoryPoints = 3
			case "velocity":
				o.velocityFraction = 0.51
			case "delta":
				o.delta = 0
			case "counts_per_unit":
				d.Axes[1].CountsPerUnit = math.NaN()
			case "sign":
				d.Axes[1].Sign = 0
			}
			if _, err := programProfile(d, o, s); err == nil {
				t.Fatal("unsafe program accepted")
			}
		})
	}
}

// TestProgramProfileHermiteBounds admits encoded programs across short/long and
// forward/reverse moves, preventing declarations based only on knot velocities.
func TestProgramProfileHermiteBounds(t *testing.T) {
	for _, delta := range []float64{-.2, -.02, -.001, -.000001, .000001, .001, .02, .2} {
		for _, velocityFraction := range []float64{.01, .1, .5} {
			for _, accelFraction := range []float64{.01, .1, .5} {
				for _, mask := range []uint64{257, 511} {
					t.Run(fmt.Sprintf("delta=%g/velocity=%g/accel=%g/mask=%d", delta, velocityFraction, accelFraction, mask), func(t *testing.T) {
						d, o, s := programGeometry()
						o.delta, o.velocityFraction, o.accelFraction, o.mask = delta, velocityFraction, accelFraction, mask
						ids, offsets := make([]string, 9), make([]int32, 9)
						for axis := range d.Axes {
							ids[axis] = d.Axes[axis].ID
							// Unverified frame fixtures exercise scaling, signs and
							// offset cancellation in the same arithmetic used by admission.
							d.Axes[axis].CountsPerUnit = []float64{1, 4096, 131072 / (2 * math.Pi)}[axis%3]
							if axis%2 == 1 {
								d.Axes[axis].Sign = -1
							}
							offsets[axis] = int32(axis-4) * 100000
							s.Core.Axes[axis].HomeOffset = offsets[axis]
						}
						d.Axes[8].MinPosition, d.Axes[8].MaxPosition = -30, 30
						plan, err := programProfile(d, o, s)
						simfixture.Must(t, err)
						blob, err := densejointtraj.Encode(plan.Header, plan.Segments)
						simfixture.Must(t, err)
						decoded, err := densejointtraj.Decode(blob)
						simfixture.Must(t, err)
						_, err = admission.DecodeProgram(blob, d.NativeDescription, ids, offsets...)
						simfixture.Must(t, err)
						for axis, a := range d.Axes {
							limit := decoded.Header.Limits.MaxAbsQdRadS[axis]
							// Absolute caps are 0.005, 0.05 and 0.25 rad/s, independent
							// of the profile's generated positions and tangents.
							cap := a.MaxVelocity * velocityFraction
							if !finite(limit) || limit <= 0 || limit > cap {
								t.Fatalf("axis %d declaration=%g rad/s exceeds cap=%g rad/s", axis, limit, cap)
							}
							frame := densejointtraj.CommandFrame{CountsPerUnit: a.CountsPerUnit, Sign: a.Sign, HomeOffset: offsets[axis]}
							samples := decoded.Segments[0].Samples
							for i := 1; i < len(samples); i++ {
								first, second := samples[i-1], samples[i]
								bounds, err := densejointtraj.BoundHermiteSegment(frame,
									densejointtraj.HermiteKnot{TimeNS: int64(math.Round(first.TimeS * 1e9)), Position: first.QRad[axis], Velocity: first.QdRadS[axis]},
									densejointtraj.HermiteKnot{TimeNS: int64(math.Round(second.TimeS * 1e9)), Position: second.QRad[axis], Velocity: second.QdRadS[axis]})
								simfixture.Must(t, err)
								if bounds.PeakVelocity > limit*a.CountsPerUnit {
									t.Fatalf("axis %d sample %d peak=%g counts/s exceeds declaration=%g counts/s", axis, i, bounds.PeakVelocity, limit*a.CountsPerUnit)
								}
							}
						}
					})
				}
			}
		}
	}
}

// TestProgramProfileHermiteCapRefusal rejects a command-frame peak above the
// operator cap instead of clamping the declaration and returning an unsafe plan.
func TestProgramProfileHermiteCapRefusal(t *testing.T) {
	d, o, s := programGeometry()
	// Unverified cancellation fixture: coarse scale and large Home offset make
	// 0.000001 rad travel produce count-frame steps above the 0.000005 rad/s cap.
	o.mask, o.delta = 1, .000001
	d.Axes[0].CountsPerUnit, d.Axes[0].MaxVelocity = 1, .00001
	s.Core.Axes[0].HomeOffset = math.MaxInt32
	plan, err := programProfile(d, o, s)
	if plan != nil || err == nil || !strings.Contains(err.Error(), "exceeds program cap") {
		t.Fatalf("over-cap Hermite program returned: plan=%v err=%v", plan, err)
	}
}

// TestProgramProfileBenchReserveAndEncoding checks the recorded bench geometry
// against absolute limits after encoding, preventing a missing reserve, clock
// collapse or numeric encoding loss from increasing the commanded sample rate.
func TestProgramProfileBenchReserveAndEncoding(t *testing.T) {
	d, o, s := programGeometry()
	o.mask, o.velocityFraction, o.accelFraction = 511, .1, .1
	// Measured held targets: bd-program-v16.jsonl:53, J1..J9, rad.
	origins := []float64{-.0026365294791784455, -.000671116594699968, .000047936899621426287,
		.0014860438882642148, .0017257283863713463, -.001006674892049952,
		-.0011025486912928045, -.0012463593901570833, .0012942962897785097}
	for i := range d.Axes {
		// Measured Describe fields: bd-program-v16.jsonl:1, rad/s^2.
		acceleration := .5
		d.Axes[i].JogAcceleration, d.Axes[i].MaxAcceleration = 2, &acceleration
		s.Core.Axes[i].LogicalTarget = origins[i]
	}
	plan, err := programProfile(d, o, s)
	simfixture.Must(t, err)
	blob, err := densejointtraj.Encode(plan.Header, plan.Segments)
	simfixture.Must(t, err)
	decoded, err := densejointtraj.Decode(blob)
	simfixture.Must(t, err)
	if !reflect.DeepEqual(plan.Segments, decoded.Segments) {
		t.Fatal("f64 encoding changed program positions, velocities or sample times")
	}
	if decoded.Header.Sampling.TotalSampleCount != 610 || decoded.Header.Sampling.TotalDurationS != 6.09 || d.CycleNS != 1000000 {
		t.Fatal("bench profile or daemon cycle changed")
	}
	var peak float64
	for i, sample := range decoded.Segments[0].Samples {
		if math.Round(sample.TimeS*1e9) != float64(i)*10000000 {
			t.Fatalf("sample %d is off the 10000000 ns source grid", i)
		}
		for axis := 0; axis < 9; axis++ {
			if limit := decoded.Header.Limits.MaxAbsQdRadS[axis]; limit < .0375 || limit > .0377 || math.Abs(sample.QdRadS[axis]) > .0375 {
				t.Fatalf("axis %d sample %d declaration=%g rad/s or tangent exceeds bench bounds", axis, i, limit)
			}
			if i > 0 {
				previous := decoded.Segments[0].Samples[i-1]
				velocity := math.Abs(sample.QRad[axis]-previous.QRad[axis]) / (sample.TimeS - previous.TimeS)
				if velocity > .0375 {
					t.Fatalf("axis %d sample %d position rate=%g rad/s", axis, i, velocity)
				}
				peak = math.Max(peak, velocity)
			}
		}
	}
	if peak < .0374 {
		t.Fatalf("profile did not exercise reserved cruise: %g rad/s", peak)
	}
	ids := make([]string, 9)
	for axis := range d.Axes {
		ids[axis] = d.Axes[axis].ID
	}
	_, err = admission.DecodeProgram(blob, d.NativeDescription, ids)
	simfixture.Must(t, err)
	for axis := range decoded.Header.Limits.MaxAbsQdRadS {
		decoded.Header.Limits.MaxAbsQdRadS[axis] = .0375
	}
	oldDeclaration, err := densejointtraj.Encode(decoded.Header, decoded.Segments)
	simfixture.Must(t, err)
	_, err = admission.DecodeProgram(oldDeclaration, d.NativeDescription, ids)
	var refused *admission.SegmentVelocityError
	if !errors.As(err, &refused) || refused.Peak <= .0375 || refused.Peak > .0377 || refused.Limit != .0375 {
		t.Fatalf("old declaration did not reproduce the Hermite refusal: %v", err)
	}
	t.Logf("fixture axis 0 declaration=%.17g rad/s; old declaration refused at peak=%.17g rad/s", plan.Header.Limits.MaxAbsQdRadS[0], refused.Peak)
}

func TestProgramFlagsAndConfirmation(t *testing.T) {
	f := &fixture{socket: "missing.sock", hash: strings.Repeat("a", 64), build: testBuildDir(t)}
	args := f.args("program", "dry-program", false)
	o, err := parseOptions(args, io.Discard)
	simfixture.Must(t, err)
	if o.delta != 0.2 || o.mask != 511 {
		t.Fatal("program defaults changed")
	}
	var out bytes.Buffer
	if code := run(context.Background(), args, &out); code != 3 || !strings.Contains(out.String(), "Enable all described axes") {
		t.Fatalf("unconfirmed program: %d %s", code, &out)
	}
	if records := readRecords(t, filepath.Join(f.build, "dry-program.jsonl")); len(records) != 2 || records[0].Kind != "plan" {
		t.Fatal("unconfirmed program opened a connection")
	}
	for _, extra := range [][]string{{"--axis-mask", "0"}, {"--axis-mask", "512"}, {"--axis", "0"}, {"--deltas-rad", "0:0.2"},
		{"--delta-rad", "0"}, {"--delta-rad", "NaN"}, {"--velocity-fraction", "0.51"}, {"--keep-armed"}} {
		if _, err := parseOptions(append(append([]string(nil), args...), extra...), io.Discard); err == nil {
			t.Fatalf("accepted flags %v", extra)
		}
	}
}

func TestProgramRequiresHomeOnStationaryAxes(t *testing.T) {
	d, o, s := programGeometry()
	o.phase, o.mask = "program", 1
	r := newRunner(o, nil, nil, io.Discard)
	r.description = d
	s.Core.HomeValidMask = 1
	s.Core.Axes[0].HomeValid, s.Core.Axes[0].Readiness = true, "not_enabled"
	if err := r.preEnable(s); err == nil || !strings.Contains(err.Error(), "axis 1:") {
		t.Fatalf("stationary unhomed axis admitted: %v", err)
	}
	for i := range s.Core.Axes {
		s.Core.Axes[i].HomeValid, s.Core.Axes[i].Readiness = true, "not_enabled"
	}
	s.Core.HomeValidMask = 511
	simfixture.Must(t, r.preEnable(s))
	if r.enabledMask() != 511 {
		t.Fatal("dense program did not select all nine axes for Enable")
	}
}

// All nine Home completions and in-flight renewals must precede program motion;
// a setup cancellation must expose native diagnostics rather than hide its cause.
// Unverified fixture: nine axes (mask 511), grant generation 1, Home epochs 1/2.
func TestSimulationProgramTravelAndVelocity(t *testing.T) {
	// Measured under -race at 3d538662: wake lateness 115,525,612 ns with
	// cycle work only 581,617 ns exceeded the initial 100,000,000 ns cap.
	// The baseline taskset -c 0-1 maxima were 11,559,836 / 4,298,050 ns.
	// This unverified simulator allowance remains below the 250,000,000 ns
	// native heartbeat lease; live limits and motion assertions are unchanged.
	const simulationCycleLatenessNS uint64 = 200_000_000
	f := startFixtureAxesWithCycleBudget(t, false, false, 9, simulationCycleLatenessNS)
	t.Cleanup(func() {
		if raw, err := os.ReadFile(filepath.Join(filepath.Dir(f.socket), "metrics.json")); err == nil {
			var timing struct {
				JitterNS uint64 `json:"rt_max_abs_jitter_ns"`
				WorkNS   uint64 `json:"rt_max_cycle_work_ns"`
			}
			simfixture.Must(t, json.Unmarshal(raw, &timing))
			t.Logf("simulated cycle measurement: max wake lateness=%d ns, max cycle work=%d ns", timing.JitterNS, timing.WorkNS)
		}
		if t.Failed() {
			raw, err := os.ReadFile(filepath.Join(f.build, "core.log"))
			if err == nil {
				t.Logf("native Home and cancellation diagnostics:\n%s", raw)
			}
		}
	})
	before := f.poll(t, func(s control.Status) bool { return s.Core.HomeValidMask == 0 })
	homeRecords := f.phase(t, "home")
	var homeStart, homeEnd uint64
	var homedMask uint32
	for _, rec := range homeRecords {
		if rec.Operation != "home" {
			continue
		}
		if rec.Kind == "request" {
			homeStart, homeEnd = rec.StartedNS, rec.HostNS
		}
		if rec.Kind == "home_axis" {
			var axis struct {
				Axis      uint32 `json:"axis"`
				HomeValid bool   `json:"home_valid"`
			}
			simfixture.Must(t, json.Unmarshal(rec.Response, &axis))
			if axis.Axis >= 9 || !axis.HomeValid || homedMask&(1<<axis.Axis) != 0 {
				t.Fatalf("invalid or duplicate Home completion: %+v", axis)
			}
			homedMask |= 1 << axis.Axis
		}
	}
	renewals := 0
	for _, rec := range homeRecords {
		if rec.Operation == "renew" && rec.StartedNS > homeStart && rec.HostNS < homeEnd {
			var grant control.Grant
			simfixture.Must(t, json.Unmarshal(rec.Response, &grant))
			if grant.Stopping || grant.Generation != 1 || grant.Session == "" {
				t.Fatalf("Home lost renewal: %+v", grant)
			}
			renewals++
		}
	}
	if homeStart == 0 || homeEnd <= homeStart || homedMask != 511 || renewals < 1 {
		t.Fatalf("Home setup missing completion/renewal: start=%d end=%d mask=%d renewals=%d", homeStart, homeEnd, homedMask, renewals)
	}
	homed := f.poll(t, func(s control.Status) bool { return s.Core.HomeValidMask == 511 && s.Core.CommissioningPhase == 0 })
	if before.Core.HomeEpoch != 1 || homed.Core.HomeEpoch != 2 || homed.Core.AxisEnableMask != 0 || homed.Core.Armed != 0 || homed.Core.SafetyFaultMask != 0 {
		t.Fatalf("Home did not finish inhibited in the next epoch: %+v", homed.Core)
	}
	for _, tc := range []struct{ name, mask, delta string }{{"all", "511", "0.2"}, {"subset_reverse", "257", "-0.02"}} {
		t.Run(tc.name, func(t *testing.T) {
			args := append(f.args("program", tc.name, true), "--axis-mask", tc.mask, "--delta-rad", tc.delta, "--velocity-fraction", "0.5", "--accel-fraction", "0.5")
			var out bytes.Buffer
			if code := run(context.Background(), args, &out); code != 0 {
				t.Fatalf("program exit=%d: %s", code, &out)
			}
			records := readRecords(t, filepath.Join(f.build, tc.name+".jsonl"))
			var prepared control.Program
			var sequence uint64
			travel, completion, uploads, starts, stops, releases, renewals := 0, 0, 0, 0, 0, 0, 0
			type position struct {
				Axis     int
				Time     uint64  `json:"time_ns"`
				Position float64 `json:"logical_position"`
			}
			previous, counts := map[int]position{}, map[int]int{}
			for _, rec := range records {
				switch rec.Operation {
				case "prepare_program":
					simfixture.Must(t, json.Unmarshal(rec.Response, &prepared))
					var fence struct {
						Session    string
						Generation uint64
					}
					simfixture.Must(t, json.Unmarshal(rec.Request, &fence))
					if len(fence.Session) != 8 || fence.Generation == 0 || prepared.AxisMask != 511 || prepared.Identity.PlanRevision != 1 || prepared.Identity.ManifestRevision != 7 ||
						prepared.Identity.SourceDigest == "" || prepared.Identity.NormalisedDigest == "" || prepared.RequiresProcessIO {
						t.Fatalf("incomplete upload fence or metadata: %+v", prepared)
					}
					uploads++
				case "start_program":
					var req control.Request
					var response control.Response
					simfixture.Must(t, json.Unmarshal(rec.Request, &req))
					simfixture.Must(t, json.Unmarshal(rec.Response, &response))
					if uploads != 1 || req.Identity != prepared.Identity || response.Sequence == 0 {
						t.Fatal("Start did not echo the complete uploaded identity")
					}
					sequence = response.Sequence
					starts++
				case "program_before", "program_sample":
					var p position
					simfixture.Must(t, json.Unmarshal(rec.Response, &p))
					if prev, ok := previous[p.Axis]; ok {
						if p.Time <= prev.Time || math.Abs(p.Position-prev.Position)*1e9/float64(p.Time-prev.Time) > 0.05 {
							t.Fatal("recorded program velocity exceeds 0.05 rad/s")
						}
					}
					previous[p.Axis] = p
					counts[p.Axis]++
				case "program_travel":
					var v struct {
						Axis     int
						Start    float64 `json:"start_logical_position"`
						End      float64 `json:"end_logical_position"`
						Measured float64 `json:"measured_travel_rad"`
						Delta    float64 `json:"delta_rad"`
						Identity control.Identity
					}
					simfixture.Must(t, json.Unmarshal(rec.Response, &v))
					want := 0.2
					if tc.name == "subset_reverse" {
						want = 0
						if v.Axis == 0 || v.Axis == 8 {
							want = -0.02
						}
					}
					if v.Delta != want || math.Abs(v.Measured-want) > 0.0002 || math.Abs(v.End-v.Start-want) > 0.0002 || v.Identity != prepared.Identity {
						t.Fatalf("travel exceeds 0.0002 rad tolerance: %+v", v)
					}
					travel++
				case "program":
					if rec.Kind == "completion" {
						var execution control.Execution
						simfixture.Must(t, json.Unmarshal(rec.Response, &execution))
						if execution.State != "completed" || execution.Identity != prepared.Identity || execution.NativeSequence != sequence || execution.Generation == 0 {
							t.Fatal("uncorrelated completion")
						}
						terminal := false
						for _, h := range execution.Handles {
							terminal = terminal || (h.NativeSequence == sequence && (h.State == "consumed" || h.State == "retired"))
						}
						if !terminal {
							t.Fatal("completion without terminal handle")
						}
						completion++
					}
				case "renew":
					if starts == 1 && completion == 0 {
						renewals++
					}
				case "stop":
					if rec.Kind == "request" {
						stops++
					}
				case "release":
					if rec.Kind == "request" {
						if stops != 1 {
							t.Fatal("Release before Stop")
						}
						releases++
					}
				}
			}
			for axis := 0; axis < 9; axis++ {
				if counts[axis] < 3 {
					t.Fatalf("axis %d lacks recorded positions", axis)
				}
			}
			if uploads != 1 || starts != 1 || travel != 9 || completion != 1 || stops != 1 || releases != 1 || renewals < 1 {
				t.Fatalf("missing upload, motion, renewal or cleanup: %d %d %d %d %d %d %d", uploads, starts, travel, completion, stops, releases, renewals)
			}
			f.poll(t, func(s control.Status) bool {
				return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Core.LeaseValid == 0
			})
		})
	}
}

func TestSimulationProgramStartsAtHeldTarget(t *testing.T) {
	f := startFixtureMode(t, false, "--sim-feedback-offset-counts", "4")
	f.phase(t, "home")
	records := f.phase(t, "program", "--delta-rad", "0.02", "--velocity-fraction", "0.5", "--accel-fraction", "0.5")
	var before control.Status
	sources := 0
	for _, rec := range records {
		if rec.Kind == "status" && sources == 0 {
			simfixture.Must(t, json.Unmarshal(rec.Response, &before))
		}
		if rec.Operation == "program_source" {
			var source struct {
				First densejointtraj.Sample `json:"first_sample"`
			}
			simfixture.Must(t, json.Unmarshal(rec.Response, &source))
			for i, a := range before.Core.Axes {
				if source.First.QRad[i] != a.LogicalTarget {
					t.Fatalf("axis %d did not start at held target", i)
				}
				if i == 0 && math.Abs(math.Abs(a.LogicalPosition-a.LogicalTarget)*131072/(2*math.Pi)-4) > 1e-8 {
					t.Fatal("axis 0 lacks the injected 4-count feedback offset")
				}
			}
			sources++
		}
	}
	if sources != 1 {
		t.Fatal("missing source sample record")
	}
}

func TestSimulationProgramRefusals(t *testing.T) {
	f := startFixture(t)
	f.phase(t, "home")
	r := testRunner(t, f, "program", "refusals")
	r.o.delta, r.o.velocityFraction = 0.02, 0.5
	ctx, cancel := context.WithCancelCause(context.Background())
	defer cancel(nil)
	var err error
	r.description, err = r.c.Describe(ctx)
	simfixture.Must(t, err)
	_, err = r.status(ctx)
	simfixture.Must(t, err)
	r.homeEpoch, r.configEpoch = r.last.Core.HomeEpoch, r.last.Core.ConfigurationEpoch
	encode := func() []byte {
		plan, err := programProfile(r.description, r.o, r.last)
		simfixture.Must(t, err)
		blob, err := densejointtraj.Encode(plan.Header, plan.Segments)
		simfixture.Must(t, err)
		return blob
	}
	t.Run("upload_without_grant", func(t *testing.T) {
		_, err := r.c.PrepareProgram(ctx, encode())
		var refused *control.Rejected
		if !errors.As(err, &refused) || refused.Reason != "control_session_stale" {
			t.Fatalf("upload refusal: %v", err)
		}
	})
	_, err = r.command(ctx, "acquire", 0)
	simfixture.Must(t, err)
	defer func() {
		r.stopRenewal()
		simfixture.Must(t, r.stop(context.Background()))
		simfixture.Must(t, r.release(context.Background()))
	}()
	simfixture.Must(t, r.startRenewal(ctx, cancel))
	simfixture.Must(t, r.enable(ctx))
	prepared, err := r.c.PrepareProgram(ctx, encode())
	simfixture.Must(t, err)
	for _, field := range []string{"plan_id", "program_id", "program_digest", "trajectory_digest", "source_digest", "normalised_digest", "manifest_revision", "plan_revision"} {
		t.Run(field, func(t *testing.T) {
			identity := prepared.Identity
			switch field {
			case "plan_id":
				identity.PlanID += "-changed"
			case "program_id":
				identity.ProgramID += "-changed"
			case "program_digest":
				identity.ProgramDigest = "sha256:" + strings.Repeat("0", 64)
			case "trajectory_digest":
				identity.TrajectoryDigest = "sha256:" + strings.Repeat("0", 64)
			case "source_digest":
				identity.SourceDigest = "sha256:" + strings.Repeat("0", 64)
			case "normalised_digest":
				identity.NormalisedDigest = "sha256:" + strings.Repeat("0", 64)
			case "manifest_revision":
				identity.ManifestRevision++
			case "plan_revision":
				identity.PlanRevision++
			}
			_, err := r.c.StartProgram(ctx, identity)
			var refused *control.Rejected
			if !errors.As(err, &refused) || refused.Reason != "program_identity_mismatch" {
				t.Fatalf("identity refusal: %v", err)
			}
		})
	}
	before := r.last
	after := f.poll(t, func(s control.Status) bool {
		return s.Core.TimeNS > before.Core.TimeNS && s.Execution.State == "prepared"
	})
	if after.Motion.CommandSequence != before.Motion.CommandSequence || after.Core.SafetyFaultMask != 0 || after.Core.ControlGeneration != before.Core.ControlGeneration {
		t.Fatal("refusal changed motion, safety or generation")
	}
	for i, a := range after.Core.Axes {
		if a.PositionCounts != before.Core.Axes[i].PositionCounts {
			t.Fatal("refused program moved an axis")
		}
	}
}

// Construct the discontinuous fixture before computing its count-frame Hermite
// declaration. Translating already-bounded knots changes binary64 cancellation
// after scaling/Home subtraction and can exceed that original tight declaration.
func discontinuousProgramFixture(t *testing.T, d control.Description, o options, before control.Status) *densejointtraj.Plan {
	t.Helper()
	shifted := before
	shifted.Core.Axes = append([]ipcclient.AxisStatus(nil), before.Core.Axes...)
	shifted.Core.Axes[0].LogicalTarget += .01
	plan, err := programProfile(d, o, shifted)
	simfixture.Must(t, err)
	return plan
}

// This is a pure source/admission reproduction: no daemon, brake, runtime latch,
// or post-disable cleanup is involved. The command cap remains 0.05 rad/s.
func TestProgramDiscontinuousFixtureReboundsTranslatedOrigin(t *testing.T) {
	staleDeclarations := 0
	for _, origin := range []float64{0, .0001, -.0002, .01} {
		d, o, before := programGeometry()
		o.mask, o.delta, o.velocityFraction, o.accelFraction = 511, .02, .5, .1
		ids := make([]string, len(d.Axes))
		for i := range d.Axes {
			d.Axes[i].MaxVelocity = .1
			before.Core.Axes[i].LogicalTarget = origin
			ids[i] = d.Axes[i].ID
		}
		old, err := programProfile(d, o, before)
		simfixture.Must(t, err)
		blob, err := densejointtraj.Encode(old.Header, old.Segments)
		simfixture.Must(t, err)
		_, err = admission.DecodeProgram(blob, d.NativeDescription, ids)
		simfixture.Must(t, err)
		for i := range old.Segments[0].Samples {
			old.Segments[0].Samples[i].QRad[0] += .01
		}
		blob, err = densejointtraj.Encode(old.Header, old.Segments)
		simfixture.Must(t, err)
		_, err = admission.DecodeProgram(blob, d.NativeDescription, ids)
		if err != nil {
			var rate *admission.SegmentVelocityError
			if !errors.As(err, &rate) || rate.Axis != "J1" {
				t.Fatalf("unexpected stale-declaration failure at origin=%g: %v", origin, err)
			}
			staleDeclarations++
		}
		plan := discontinuousProgramFixture(t, d, o, before)
		blob, err = densejointtraj.Encode(plan.Header, plan.Segments)
		simfixture.Must(t, err)
		_, err = admission.DecodeProgram(blob, d.NativeDescription, ids)
		simfixture.Must(t, err)
		samples := plan.Segments[0].Samples
		gap := (samples[0].QRad[0] - before.Core.Axes[0].LogicalTarget) * d.Axes[0].CountsPerUnit
		if math.Abs(gap) <= 1 || math.Abs(samples[0].QRad[0]-(origin+.01)) > 1e-12 ||
			math.Abs(samples[len(samples)-1].QRad[0]-(origin+.03)) > 1e-12 {
			t.Fatalf("fixture lost its deliberate discontinuity/travel: gap=%g", gap)
		}
		for axis, limit := range plan.Header.Limits.MaxAbsQdRadS {
			if limit <= 0 || limit > .05 || d.Axes[axis].MaxVelocity != .1 {
				t.Fatalf("fixture changed command/axis cap: axis=%d declaration=%g", axis, limit)
			}
			if before.Core.Axes[axis].LogicalTarget != origin {
				t.Fatal("fixture modified observed robot targets")
			}
		}
	}
	if staleDeclarations == 0 {
		t.Fatal("the unchanged post-bound translation no longer reproduces the admission failure")
	}
}

// A discontinuous Start must refuse without motion. Wait for renewal to observe
// native revocation before cleanup, preventing scheduler timing from deciding
// whether this test exercises Release of a session already retired by Stop.
func TestSimulationProgramDiscontinuousStartRefused(t *testing.T) {
	f := startFixture(t)
	f.phase(t, "home")
	r := testRunner(t, f, "program", "discontinuous")
	defer func() {
		if t.Failed() {
			raw, err := os.ReadFile(r.o.evidence)
			t.Logf("cleanup evidence (read error=%v):\n%s", err, raw)
		}
	}()
	r.o.delta, r.o.velocityFraction = 0.02, 0.5
	ctx, cancel := context.WithCancelCause(context.Background())
	defer cancel(nil)
	var err error
	r.description, err = r.c.Describe(ctx)
	simfixture.Must(t, err)
	_, err = r.status(ctx)
	simfixture.Must(t, err)
	r.homeEpoch, r.configEpoch = r.last.Core.HomeEpoch, r.last.Core.ConfigurationEpoch
	_, err = r.command(ctx, "acquire", 0)
	simfixture.Must(t, err)
	defer func() {
		r.stopRenewal()
		if err := r.stop(context.Background()); err != nil {
			t.Fatalf("cleanup Stop: %v", err)
		}
		if r.c.Fence().Session != "" {
			t.Fatal("cleanup Stop retained the revoked session")
		}
		if err := r.release(context.Background()); err != nil {
			t.Fatalf("cleanup Release: %v", err)
		}
	}()
	simfixture.Must(t, r.startRenewal(ctx, cancel))
	simfixture.Must(t, r.enable(ctx))
	before := r.last
	// The 0.01 rad offset deliberately exceeds Start's one-count continuity
	// gate. Recompute the fixture at that origin so Prepare still tests a valid
	// count-frame curve against the unchanged velocity/acceleration limits.
	plan := discontinuousProgramFixture(t, r.description, r.o, before)
	blob, err := densejointtraj.Encode(plan.Header, plan.Segments)
	simfixture.Must(t, err)
	prepared, err := r.c.PrepareProgram(ctx, blob)
	simfixture.Must(t, err)
	_, err = r.request(ctx, control.Request{Operation: "start_program", Identity: prepared.Identity})
	var refused *control.Rejected
	// The discontinuity also revokes authority; its fence can beat the native
	// rejection receipt. Require the exact fault below for either response.
	if !errors.As(err, &refused) || (refused.Reason != "fence" && (refused.Reason != "native_rejected" || refused.NativeResult == nil || refused.NativeResult.Reason != ipcclient.ReasonInvalidTrajectory)) {
		t.Fatalf("continuity refusal: %v", err)
	}
	after := f.poll(t, func(s control.Status) bool {
		return s.Core.TimeNS > before.Core.TimeNS && s.Core.SafetyFaultMask != 0 && r.inhibited(s)
	})
	if after.Core.ExecutionFaultReasons != 1 || after.Core.SafetyFaultMask != 1 {
		t.Fatalf("wrong continuity fault: reasons=%d axes=%d", after.Core.ExecutionFaultReasons, after.Core.SafetyFaultMask)
	}
	for i, a := range after.Core.Axes {
		if a.PositionCounts != before.Core.Axes[i].PositionCounts {
			t.Fatal("discontinuous program moved an axis")
		}
	}
	wait, done := context.WithTimeout(context.Background(), fixtureDeadline)
	defer done()
	select {
	case <-ctx.Done():
		var renewalRefused *control.Rejected
		if !errors.As(context.Cause(ctx), &renewalRefused) {
			t.Fatalf("renewal did not observe native revocation: %v", context.Cause(ctx))
		}
	case <-wait.Done():
		t.Fatal("renewal did not retire the faulted session")
	}
	_, err = r.status(context.Background())
	simfixture.Must(t, err)
}

type cancelProgramSink struct {
	writer    io.Writer
	cancel    context.CancelCauseFunc
	operation string
	sent      bool
}

func (s *cancelProgramSink) Write(raw []byte) (int, error) {
	var rec savedRecord
	if err := json.Unmarshal(raw, &rec); err != nil {
		return 0, err
	}
	if rec.Operation == s.operation && rec.Kind == "request" {
		s.sent = true
		s.cancel(context.Canceled)
	}
	return s.writer.Write(raw)
}

func TestSimulationProgramFailureStopsAndReleases(t *testing.T) {
	f := startFixture(t)
	f.phase(t, "home")
	for _, name := range []string{"soft_limit", "prepare_program", "start_program"} {
		t.Run(name, func(t *testing.T) {
			r := testRunner(t, f, "program", name)
			r.o.delta, r.o.velocityFraction = 0.02, 0.5
			ctx, cancel := context.WithCancelCause(context.Background())
			defer cancel(nil)
			sink := &cancelProgramSink{writer: r.e.file, cancel: cancel, operation: name}
			if name == "soft_limit" {
				r.o.delta = 1
			} else {
				r.e.sink = sink
			}
			err := r.run(ctx)
			if err == nil || (name != "soft_limit" && (!sink.sent || !errors.Is(err, context.Canceled))) {
				t.Fatalf("failure did not reach %s: %v", name, err)
			}
			if name == "soft_limit" && !strings.Contains(err.Error(), "move violates described") {
				t.Fatalf("wrong failure: %v", err)
			}
			r.summary(err)
			stops, releases := 0, 0
			for _, rec := range readRecords(t, r.o.evidence) {
				if rec.Kind != "request" {
					continue
				}
				if rec.Operation == "stop" {
					stops++
				}
				if rec.Operation == "release" {
					if stops != 1 {
						t.Fatal("Release preceded Stop")
					}
					releases++
				}
				if name == "soft_limit" && rec.Operation == "prepare_program" {
					t.Fatal("out-of-limit program reached upload")
				}
			}
			if stops != 1 || releases != 1 {
				t.Fatalf("Stop=%d Release=%d", stops, releases)
			}
			f.poll(t, func(s control.Status) bool {
				return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Core.LeaseValid == 0
			})
		})
	}
}
