//go:build linux

package main

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"io"
	"math"
	"net/http"
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"

	"rosieos/rt-core/adapters/rosie/control/anchors"
	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// anchorProfiles extends the simulation fixture with an independent continuous
// raw source, preventing a fabricated anchor from substituting for Home capture.
func anchorProfiles(t *testing.T, root, build string) string {
	t.Helper()
	read := func(path string) map[string]any {
		raw, err := os.ReadFile(filepath.Join(root, path))
		simfixture.Must(t, err)
		var value map[string]any
		simfixture.Must(t, json.Unmarshal(raw, &value))
		return value
	}
	profile := read("config/drives/cia402_sim_home.json")
	// Unverified synthetic raw-source bounds match the compiled coordinate fixture.
	profile["coordinate_evidence"] = map[string]any{
		"source": "a6ec_sdo", "sample_interval_ns": 100_000,
		"start_alignment_floor_counts": 1,
		"anchor_source": map[string]any{
			"index": 8256, "low_subindex": 33, "high_subindex": 35,
			"name": "synthetic_raw", "identity_name": "raw_multi_turn_encoder", "version": 3,
		},
		"maximum_physical_speed_counts_per_second": 2000000,
		"maximum_acquisition_age_ns":               10000000,
		"maximum_feedback_error_counts":            1,
		"maximum_anchor_error_counts":              1,
	}
	dir := filepath.Join(build, "config/drives")
	simfixture.Must(t, os.MkdirAll(dir, 0700))
	raw, err := json.Marshal(profile)
	simfixture.Must(t, err)
	simfixture.Must(t, os.WriteFile(filepath.Join(dir, "cia402_sim_home.json"), raw, 0600))
	return dir
}

func phaseRecords(t *testing.T, records []savedRecord) []phaseResult {
	t.Helper()
	var rows []phaseResult
	for _, record := range records {
		if record.Kind == "phase" {
			var row phaseResult
			simfixture.Must(t, json.Unmarshal(record.Response, &row))
			rows = append(rows, row)
		}
	}
	return rows
}

func TestSimulationSequenceAllAxes(t *testing.T) {
	f := startFixture(t)
	records := f.phase(t, "sequence", "--axis-mask", "511", "--hold-ns", "100000000")
	rows := phaseRecords(t, records)
	var names []string
	for _, row := range rows {
		names = append(names, row.Phase)
		if !row.Met {
			t.Fatalf("failed phase: %+v", row)
		}
	}
	if !reflect.DeepEqual(names, []string{"inhibited", "home", "enable", "arm", "hold", "stop", "release"}) {
		t.Fatalf("phase order: %v", names)
	}
	if !reflect.DeepEqual(rows[0].Reasons, []string{"control_already_owned", "native:2", "native_rejected"}) {
		t.Fatalf("typed refusals missing: %+v", rows[0])
	}
	transitions, homes := map[int]int{}, map[int]bool{}
	for _, record := range records {
		var value struct {
			Axis      int  `json:"axis"`
			HomeValid bool `json:"home_valid"`
		}
		if record.Kind == "statusword_transition" || record.Kind == "home_axis" {
			simfixture.Must(t, json.Unmarshal(record.Response, &value))
			if record.Kind == "statusword_transition" && record.Operation == "home" {
				transitions[value.Axis]++
			} else if record.Kind == "home_axis" {
				homes[value.Axis] = value.HomeValid
			}
		}
	}
	for axis := 0; axis < 9; axis++ {
		if transitions[axis] < 2 || !homes[axis] {
			t.Fatalf("axis %d transitions=%d home=%t", axis, transitions[axis], homes[axis])
		}
	}
}

type cancelOnStartSink struct {
	writer io.Writer
	cancel context.CancelCauseFunc
	sent   bool
}

func (s *cancelOnStartSink) Write(raw []byte) (int, error) {
	var record savedRecord
	if err := json.Unmarshal(raw, &record); err != nil {
		return 0, err
	}
	if record.Operation == "start_trajectory" && record.Kind == "request" {
		s.sent = true
		s.cancel(context.Canceled)
	}
	return s.writer.Write(raw)
}

func TestSimulationMoveFailureStopsAndReleases(t *testing.T) {
	f := startFixture(t)
	f.phase(t, "home")
	for _, name := range []string{"soft_limit", "canceled"} {
		t.Run(name, func(t *testing.T) {
			r := testRunner(t, f, "move", name)
			r.o.keepArmed = true
			ctx, cancel := context.WithCancelCause(context.Background())
			defer cancel(nil)
			sink := &cancelOnStartSink{writer: r.e.file, cancel: cancel}
			if name == "soft_limit" {
				r.o.moves[0].delta = 1
			} else {
				r.e.sink = sink
			}
			err := r.run(ctx)
			if err == nil {
				t.Fatal("failed move reported success")
			}
			if name == "soft_limit" && !strings.Contains(err.Error(), "move violates described") {
				t.Fatalf("did not reach the soft-limit refusal: %v", err)
			}
			r.summary(err)
			if name == "canceled" && !sink.sent {
				t.Fatal("did not reach trajectory start")
			}
			records := readRecords(t, r.o.evidence)
			stops, releases := 0, 0
			for _, record := range records {
				if record.Kind != "request" {
					continue
				}
				if record.Operation == "stop" {
					stops++
				}
				if record.Operation == "release" {
					releases++
					if stops != 1 {
						t.Fatal("Release preceded Stop")
					}
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

func TestSimulationMoveTravelAndVelocity(t *testing.T) {
	f := startFixture(t)
	f.phase(t, "home")
	for _, tc := range []struct {
		name, delta string
		keep        bool
	}{{"positive", "0.02", false}, {"negative", "-0.02", true}} {
		t.Run(tc.name, func(t *testing.T) {
			args := append(f.args("move", tc.name, true), "--delta-rad", tc.delta, "--accel-fraction", "0.5")
			if tc.keep {
				args = append(args, "--keep-armed")
			}
			var out bytes.Buffer
			if code := run(context.Background(), args, &out); code != 0 {
				t.Fatalf("move exit=%d: %s", code, &out)
			}
			records := readRecords(t, filepath.Join(f.build, tc.name+".jsonl"))
			travel, samples, stops, releases, completion := 0, 0, 0, 0, 0
			var previous struct {
				Time     uint64  `json:"time_ns"`
				Position float64 `json:"logical_position"`
			}
			for _, record := range records {
				switch record.Operation {
				case "move_travel":
					var v struct {
						Start    float64 `json:"start_logical_position"`
						End      float64 `json:"end_logical_position"`
						Delta    float64 `json:"delta_rad"`
						Measured float64 `json:"measured_travel_rad"`
					}
					simfixture.Must(t, json.Unmarshal(record.Response, &v))
					travel++
					if math.Abs(v.End-v.Start-v.Delta) > 0.0002 || math.Abs(v.Measured-v.Delta) > 0.0002 {
						t.Fatalf("travel exceeds 0.0002 rad tolerance: %+v", v)
					}
				case "move_before", "move_sample":
					var v struct {
						Time     uint64  `json:"time_ns"`
						Position float64 `json:"logical_position"`
					}
					simfixture.Must(t, json.Unmarshal(record.Response, &v))
					if previous.Time != 0 {
						if v.Time <= previous.Time {
							t.Fatal("move sample time did not increase")
						}
						velocity := math.Abs(v.Position-previous.Position) * 1e9 / float64(v.Time-previous.Time)
						if velocity > 0.05 {
							t.Fatalf("recorded velocity=%g rad/s exceeds 0.05", velocity)
						}
					}
					previous = v
					samples++
				case "stop":
					if record.Kind == "request" {
						stops++
					}
				case "release":
					if record.Kind == "request" {
						releases++
					}
				case "move":
					if record.Kind == "completion" {
						completion++
					}
				}
			}
			wantStops := 1
			if tc.keep {
				wantStops = 0
			}
			if travel != 1 || samples < 3 || stops != wantStops || releases != 1 || completion != 1 {
				t.Fatalf("travel=%d samples=%d stop=%d release=%d completion=%d", travel, samples, stops, releases, completion)
			}
			f.poll(t, func(s control.Status) bool {
				return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Core.LeaseValid == 0
			})
		})
	}
}

func TestSimulationDiscardUnknownHandle(t *testing.T) {
	f := startFixture(t)
	f.phase(t, "home")
	records := f.phase(t, "discard")
	unknown, discarded := 0, false
	for _, record := range records {
		if record.Reason == "unknown_handle" {
			unknown++
		}
		if record.Kind == "status" {
			var s control.Status
			simfixture.Must(t, json.Unmarshal(record.Response, &s))
			for _, handle := range s.Execution.Handles {
				if handle.State == "discarded" {
					discarded = true
				}
			}
		}
		if record.Kind == "request" && (record.Operation == "enable" || record.Operation == "arm" || record.Operation == "start_trajectory") {
			t.Fatalf("discard sent %s", record.Operation)
		}
	}
	if unknown != 1 || !discarded {
		t.Fatalf("unknown=%d discarded=%t", unknown, discarded)
	}
}

func TestSimulationRecoveryBeforeHomeAndAnchorCapture(t *testing.T) {
	f := startFixtureMode(t, true)
	var out bytes.Buffer
	if code := run(context.Background(), f.args("recovery", "before", false), &out); code != 0 {
		t.Fatalf("recovery before exit=%d: %s", code, &out)
	}
	records := readRecords(t, filepath.Join(f.build, "before.jsonl"))
	acquires, missing, resets := 0, 0, 0
	for _, record := range records {
		if record.Kind != "request" {
			continue
		}
		if record.Operation == "acquire" {
			acquires++
		}
		if record.Reason == "anchor_missing" {
			missing++
		}
		if record.Operation == "reset_fault" {
			resets++
		}
		if record.Operation == "enable" || record.Operation == "arm" || record.Operation == "home" {
			t.Fatalf("recovery energised via %s", record.Operation)
		}
	}
	if acquires != 2 || resets != 1 || missing != 1 {
		t.Fatalf("recovery acquires=%d resets=%d missing=%d", acquires, resets, missing)
	}
	f.phase(t, "home")
	status := f.poll(t, func(s control.Status) bool { return s.Core.HomeValidMask == 511 })
	store := anchors.Store{Directory: filepath.Join(f.build, "anchors")}
	for axis := uint32(0); axis < 9; axis++ {
		saved, err := store.Load(axis)
		simfixture.Must(t, err)
		if saved.CoordinateIdentitySHA256 != status.Core.Axes[axis].CoordinateIdentitySHA256 || !saved.MatchesPair("bench", 7) || saved.HomeEpoch == 0 {
			t.Fatalf("axis %d lacks a qualified persisted Home anchor: %+v", axis, saved)
		}
	}
}

func TestSimulationResetWhileArmedRefused(t *testing.T) {
	f := startFixture(t)
	f.phase(t, "home")
	r := testRunner(t, f, "enable", "armed-reset")
	ctx, cancel := context.WithCancelCause(context.Background())
	defer cancel(nil)
	defer func() {
		r.stopRenewal()
		simfixture.Must(t, r.stop(context.Background()))
		simfixture.Must(t, r.release(context.Background()))
	}()
	var err error
	r.description, err = r.c.Describe(ctx)
	simfixture.Must(t, err)
	s, err := r.status(ctx)
	simfixture.Must(t, err)
	r.homeEpoch, r.configEpoch = s.Core.HomeEpoch, s.Core.ConfigurationEpoch
	_, err = r.command(ctx, "acquire", 0)
	simfixture.Must(t, err)
	simfixture.Must(t, r.startRenewal(ctx, cancel))
	simfixture.Must(t, r.enable(ctx))
	response, err := r.command(ctx, "reset_fault", 511)
	simfixture.Must(t, r.expectRefusal("armed_reset_refused", "reset_requires_inhibited", err))
	if response.Sequence != 0 || response.NativeResult != nil {
		t.Fatal("armed reset reached native")
	}
}

func TestSimulationTelemetryRecordsMarkEvent(t *testing.T) {
	f := startFixture(t)
	var out bytes.Buffer
	if code := run(context.Background(), append(f.args("telemetry", "telemetry", false), "--label", "bench-camera-mark"), &out); code != 0 {
		t.Fatalf("telemetry exit=%d: %s", code, &out)
	}
	records := readRecords(t, filepath.Join(f.build, "telemetry.jsonl"))
	last := records[len(records)-1]
	for _, expected := range []string{"telemetry_mark_submitted", "telemetry_described_axes", "telemetry_increasing_sequence", "telemetry_no_motion", "telemetry_mark_visible_in_events", "stop_inhibited", "release"} {
		if !strings.Contains(strings.Join(last.Met, ","), expected) {
			t.Fatalf("missing %s: %+v", expected, last)
		}
	}
	if len(last.NotMet) != 0 || last.ExitCode != 0 {
		t.Fatalf("telemetry failed: %+v", last)
	}
	var sequence uint64
	marks := 0
	for _, record := range records {
		if record.Kind != "request" {
			continue
		}
		if record.Operation == "mark_telemetry" {
			var response control.Response
			simfixture.Must(t, json.Unmarshal(record.Response, &response))
			sequence = response.Sequence
		}
		if record.Operation == "events" {
			var response control.Response
			simfixture.Must(t, json.Unmarshal(record.Response, &response))
			var batch control.EventBatch
			simfixture.Must(t, json.Unmarshal(response.Data, &batch))
			for _, event := range batch.Events {
				if event.Type == "telemetry_mark" {
					marks++
					if event.Sequence == 0 || event.Mark == nil || event.Mark.Label != "bench-camera-mark" || sequence == 0 || event.Mark.NativeSequence != sequence {
						t.Fatalf("uncorrelated mark: %+v native sequence=%d", event, sequence)
					}
				}
			}
		}
	}
	if marks != 1 {
		t.Fatalf("recorded marks=%d, want 1", marks)
	}
	for _, row := range phaseRecords(t, records) {
		if !row.Met {
			t.Fatalf("phase failed: %+v", row)
		}
	}
}

// A refused mark must not create an event, even when a valid grant exists.
func TestSimulationTelemetryRefusedMarkHasNoEvent(t *testing.T) {
	f := startFixture(t)
	r := testRunner(t, f, "telemetry", "refused-mark")
	// Bypass CLI validation to exercise the adapter's typed label refusal.
	r.o.label = ""
	before, err := f.client.EventsContext(context.Background(), 0)
	simfixture.Must(t, err)
	err = r.summary(r.run(context.Background()))
	if err == nil || !strings.Contains(err.Error(), "telemetry_label_invalid") {
		t.Fatalf("mark did not receive its typed refusal: %v", err)
	}
	batch, err := f.client.EventsContext(context.Background(), before.LatestSequence)
	simfixture.Must(t, err)
	for _, event := range batch.Events {
		if event.Type == "telemetry_mark" || event.Mark != nil {
			t.Fatalf("refused mark published an event: %+v", event)
		}
	}
	records := readRecords(t, r.o.evidence)
	refusals := 0
	for _, record := range records {
		if record.Kind == "request" && record.Operation == "mark_telemetry" && record.Reason == "telemetry_label_invalid" {
			refusals++
		}
	}
	if refusals != 1 {
		t.Fatalf("recorded refusals=%d, want 1", refusals)
	}
	requireSessionCleanup(t, f, records, false)
}

func TestMoveProfileBounds(t *testing.T) {
	d := control.Description{NativeDescription: ipcclient.Description{CycleNS: 1000000, MaxTrajectoryPoints: 32768, Axes: []ipcclient.AxisDescription{{PositionUnit: "rad", MinPosition: -3, MaxPosition: 3, MaxVelocity: 0.5, JogAcceleration: 1, CompletionTolerance: 0.0002}}}}
	o := options{axis: 0, delta: 1.5708, velocityFraction: 0.5, accelFraction: 0.5}
	for _, delta := range []float64{1.5708, -1.5708, 0.001} {
		o.delta = delta
		points, err := moveProfile(d, o, 0)
		simfixture.Must(t, err)
		if points[0].Time != 0 || len(points) >= 32768 || math.Abs(points[len(points)-1].Position[0]-delta) > 1e-12 {
			t.Fatal("profile geometry or endpoint")
		}
		var lastVelocity float64
		for i := 1; i < len(points); i++ {
			if points[i].Time-points[i-1].Time != 1000000 {
				t.Fatal("not sampled at 1000000 ns")
			}
			v := (points[i].Position[0] - points[i-1].Position[0]) * 1000
			if math.Abs(v) > 0.25 {
				t.Fatalf("velocity=%g exceeds 0.25 rad/s", v)
			}
			if math.Abs(v-lastVelocity)*1000 > 0.500001 {
				t.Fatalf("acceleration exceeds 0.500001 rad/s^2")
			}
			lastVelocity = v
		}
	}
	for _, tc := range []struct {
		name string
		edit func(*control.Description, *options)
	}{
		{"travel", func(_ *control.Description, o *options) { o.delta = 3.01 }},
		{"soft_limit", func(_ *control.Description, o *options) { o.delta = 3 }},
		{"points", func(d *control.Description, _ *options) { d.MaxTrajectoryPoints = 3 }},
		{"cycle", func(d *control.Description, _ *options) { d.CycleNS = 0 }},
		{"fraction", func(_ *control.Description, o *options) { o.velocityFraction = 0.51 }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			dd, oo := d, o
			tc.edit(&dd, &oo)
			if _, err := moveProfile(dd, oo, 0.1); err == nil {
				t.Fatal("unsafe profile admitted")
			}
		})
	}
}

func TestNewPhaseFlagsAndConfirmation(t *testing.T) {
	f := &fixture{socket: "missing.sock", hash: strings.Repeat("a", 64), build: testBuildDir(t)}
	for _, phase := range []string{"move", "sequence"} {
		if code := run(context.Background(), f.args(phase, "dry-"+phase, false), io.Discard); code != 3 {
			t.Fatalf("%s confirmation exit=%d", phase, code)
		}
	}
	// Move travel is bounded by the described range at plan time; halt keeps 3 rad.
	halt := []string{"--socket", f.socket, "--pair-id", "bench", "--pair-revision", "7", "--configuration-sha256", f.hash,
		"--phase", "halt", "--evidence", filepath.Join(f.build, "halt-travel.jsonl"), "--confirm-motion",
		"--axis", "0", "--delta-rad", "3.01", "--cruise-velocity-rad-s", "1"}
	if _, err := parseOptions(halt, io.Discard); err == nil || !strings.Contains(err.Error(), "halt requires |delta-rad| <= 3.0") {
		t.Fatalf("halt travel: %v", err)
	}
	for i, flags := range [][]string{{"--delta-rad", "NaN"}, {"--delta-rad", "0"}, {"--velocity-fraction", "0.51"}, {"--accel-fraction", "0"}, {"--axis-mask", "3"}} {
		if code := run(context.Background(), append(f.args("move", fmt.Sprint(i), true), flags...), io.Discard); code != 2 {
			t.Fatalf("flags %v exit=%d", flags, code)
		}
	}
}

func TestTelemetryMarkRequiresFreshPublishedLabel(t *testing.T) {
	for _, tc := range []struct {
		name, field, kind       string
		stamp, sequence, native uint64
		visible, protocol       bool
	}{
		{"visible", "label", "telemetry_mark", 100, 4, 7, true, false},
		{"old", "label", "telemetry_mark", 99, 4, 7, false, false},
		{"unrelated", "controller", "telemetry_mark", 100, 4, 7, false, false},
		{"wrong_type", "label", "grant_revoked", 100, 4, 7, false, false},
		{"wrong_native_sequence", "label", "telemetry_mark", 100, 4, 8, false, false},
		{"gap", "label", "telemetry_mark", 100, 5, 7, false, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			socket := unixServer(t, func(w http.ResponseWriter, req *http.Request) {
				if req.URL.Path != "/v1/events" || req.URL.Query().Get("after") != "3" {
					t.Error("wrong event cursor")
				}
				response := map[string]any{"schema": "rosie.rt-control.response.v1", "data": map[string]any{
					"events":        []any{map[string]any{"sequence": tc.sequence, "time_ns": tc.stamp, "type": tc.kind, "mark": map[string]any{tc.field: "camera", "native_sequence": tc.native}}},
					"next_sequence": tc.sequence, "latest_sequence": tc.sequence}}
				if err := json.NewEncoder(w).Encode(response); err != nil {
					t.Error(err)
				}
			})
			r := runner{o: options{socket: socket, label: "camera"}, e: &evidence{sink: io.Discard}}
			visible, err := r.markEvent(context.Background(), 3, 100, 7)
			if visible != tc.visible || (err != nil) != tc.protocol {
				t.Fatalf("visible=%t error=%v", visible, err)
			}
		})
	}
}
