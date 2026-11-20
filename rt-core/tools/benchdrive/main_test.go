//go:build linux

package main

import (
	"bufio"
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// Unverified fixture observation ceiling: 10,000,000,000 ns. Assertions below
// use fixed count and generation bounds independent of measured trajectories.
const fixtureDeadline = 10 * time.Second

type fixture struct {
	client              *control.Client
	socket, hash, build string
}

// startFixture uses the same daemon/control process helpers and Home-requiring
// machine as the SDK profile, preventing a fake server masking contract gaps.
func startFixture(t *testing.T) *fixture {
	return startFixtureMode(t, false)
}

func startFixtureMode(t *testing.T, anchors bool, extra ...string) *fixture {
	return startFixtureMeasurement(t, anchors, false, extra...)
}

func startFixtureMeasurement(t *testing.T, anchors, measurement bool, extra ...string) *fixture {
	return startFixtureAxes(t, anchors, measurement, 9, extra...)
}

// startFixtureAxes compiles the requested fixture geometry before starting the
// real daemon and adapter, preventing default-mask tests from using a fake description.
func startFixtureAxes(t *testing.T, anchors, measurement bool, axisCount int, extra ...string) *fixture {
	t.Helper()
	return startFixtureAxesWithCycleBudget(t, anchors, measurement, axisCount, 0, extra...)
}

// startFixtureAxesWithCycleBudget compiles an explicit simulator scheduling
// budget into the machine identity, preventing an unhashed CLI override.
func startFixtureAxesWithCycleBudget(t *testing.T, anchors, measurement bool, axisCount int, cycleBudget uint64, extra ...string) *fixture {
	t.Helper()
	root, err := filepath.Abs("../..")
	simfixture.Must(t, err)
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "benchdrive-test-")
	simfixture.Must(t, err)
	t.Cleanup(func() { os.RemoveAll(build) })
	dir, err := os.MkdirTemp("", "bd-")
	simfixture.Must(t, err)
	t.Cleanup(func() { os.RemoveAll(dir) })
	profiles := filepath.Join(root, "config/drives")
	if anchors {
		profiles = anchorProfiles(t, root, build)
	}
	if measurement {
		raw, err := os.ReadFile(filepath.Join(profiles, "cia402_sim_home.json"))
		simfixture.Must(t, err)
		var profile map[string]any
		simfixture.Must(t, json.Unmarshal(raw, &profile))
		drive := profile["ethercat"].(map[string]any)
		// Unverified synthetic PDO fixtures, with the A6 manual's object addresses.
		drive["tx_layout"] = append(drive["tx_layout"].([]any), map[string]any{"semantic": "following_error", "index": 24820, "subindex": 0, "bits": 32}, map[string]any{"semantic": "velocity_actual", "index": 24684, "subindex": 0, "bits": 32})
		drive["tx_layout"] = append(drive["tx_layout"].([]any), map[string]any{"semantic": "torque", "index": 24695, "subindex": 0, "bits": 16})
		raw, err = json.Marshal(profile)
		simfixture.Must(t, err)
		profiles = filepath.Join(build, "config/drives")
		simfixture.Must(t, os.MkdirAll(profiles, 0755))
		simfixture.Must(t, os.WriteFile(filepath.Join(profiles, "cia402_sim_home.json"), raw, 0644))
	}
	machinePath := filepath.Join(root, "clients/cpp/examples/testdata/profile-machine.json")
	if anchors {
		raw, err := os.ReadFile(machinePath)
		simfixture.Must(t, err)
		var machine map[string]any
		simfixture.Must(t, json.Unmarshal(raw, &machine))
		machine["pair"] = map[string]any{"id": "bench", "revision": 7}
		for _, value := range machine["axes"].([]any) {
			axis := value.(map[string]any)
			// Unverified synthetic encoder declaration: 131072 counts/motor revolution, 1:1 gearing.
			axis["motor_encoder_counts_per_rev"] = 131072
			axis["gear_ratio"] = map[string]any{"numerator": 1, "denominator": 1}
			axis["coordinate_evidence_mode"], axis["position_tracking_mode"] = "multi_turn", "continuous"
		}
		raw, err = json.Marshal(machine)
		simfixture.Must(t, err)
		machinePath = filepath.Join(build, "anchor-machine.json")
		simfixture.Must(t, os.WriteFile(machinePath, raw, 0600))
	}
	if axisCount != 9 || cycleBudget != 0 {
		raw, err := os.ReadFile(machinePath)
		simfixture.Must(t, err)
		var machine map[string]any
		simfixture.Must(t, json.Unmarshal(raw, &machine))
		machine["axes"] = machine["axes"].([]any)[:axisCount]
		if cycleBudget != 0 {
			machine["max_cycle_lateness_ns"] = cycleBudget
		}
		raw, err = json.Marshal(machine)
		simfixture.Must(t, err)
		machinePath = filepath.Join(build, "sized-machine.json")
		// Unverified fixture permissions: generated configuration stays in build.
		simfixture.Must(t, os.WriteFile(machinePath, raw, 0600))
	}
	compiled, args, err := simfixture.CompileProfiles(root, machinePath, profiles, filepath.Join(build, "config"))
	simfixture.Must(t, err)
	core, socket := filepath.Join(dir, "ipc.sock"), filepath.Join(dir, "control.sock")
	coreCommand := exec.Command(simfixture.Locate(root, "rosie-rt-core-sim"), append(args, "--socket-path", core)...)
	coreCommand.Args = append(coreCommand.Args, extra...)
	ids := make([]string, axisCount)
	for i := range ids {
		ids[i] = fmt.Sprintf("J%d", i+1)
	}
	controlCommand := simfixture.StartControl(root, core, socket, filepath.Base(compiled), "bench", 7)
	if anchors {
		for _, command := range []*exec.Cmd{coreCommand, controlCommand} {
			command.Env = append(os.Environ(), "ROSIE_RT_ANCHOR_DIR="+filepath.Join(build, "anchors"), "GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED=0")
		}
	}
	simfixture.SpawnLogged(t, coreCommand, filepath.Join(build, "core.log"), fixtureDeadline, 5000, simfixture.CheckExit(t))
	mask := uint32(1<<axisCount) - 1
	simfixture.WaitVerified(t, filepath.Join(dir, "metrics.json"), fixtureDeadline, time.Millisecond, func(v simfixture.Verified) bool { return v.ConfigVerifiedMask == mask })
	simfixture.SpawnLogged(t, controlCommand, filepath.Join(build, "control.log"), fixtureDeadline, 5000, simfixture.CheckExit(t))
	c, err := control.Dial(control.UnixPath(socket))
	simfixture.Must(t, err)
	t.Cleanup(func() { c.Close() })
	f := &fixture{c, socket, filepath.Base(compiled), build}
	f.poll(t, func(s control.Status) bool { return s.Core.TimeNS != 0 && s.Core.ConfigVerifiedMask == mask })
	return f
}

func (f *fixture) poll(t *testing.T, predicate func(control.Status) bool) control.Status {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), fixtureDeadline)
	defer cancel()
	var s control.Status
	simfixture.Must(t, simfixture.Poll(ctx, 5*time.Millisecond, func() (bool, error) {
		var err error
		s, err = f.client.Status(ctx)
		return err == nil && predicate(s), nil
	}))
	return s
}

func (f *fixture) args(phase, name string, confirm bool) []string {
	args := []string{"--socket", f.socket, "--pair-id", "bench", "--pair-revision", "7", "--configuration-sha256", f.hash,
		"--phase", phase, "--evidence", filepath.Join(f.build, name+".jsonl")}
	if phase == "jog" || phase == "move" {
		args = append(args, "--axis", "0")
		if phase == "move" {
			args = append(args, "--delta-rad", "0.02", "--velocity-fraction", "0.5")
		}
	} else {
		args = append(args, "--axis-mask", "511")
	}
	if confirm {
		args = append(args, "--confirm-motion")
	}
	if phase == "session" {
		args = append(args, "--motor-revs-per-axis-rev", "1")
	}
	return args
}

type savedRecord struct {
	Kind            string                  `json:"kind"`
	Class           string                  `json:"class"`
	ExitCode        int                     `json:"exit_code"`
	Operation       string                  `json:"operation"`
	HostNS          uint64                  `json:"host_monotonic_ns"`
	StartedNS       uint64                  `json:"started_host_monotonic_ns"`
	Request         json.RawMessage         `json:"request"`
	Response        json.RawMessage         `json:"response"`
	Reason          string                  `json:"reason"`
	NativeResult    json.RawMessage         `json:"native_result"`
	NativeJogResult *control.JogObservation `json:"native_jog_result"`
	Met             []string                `json:"expectations_met"`
	NotMet          []string                `json:"expectations_not_met"`
}

func readRecords(t *testing.T, path string) []savedRecord {
	t.Helper()
	f, err := os.Open(path)
	simfixture.Must(t, err)
	defer f.Close()
	var records []savedRecord
	scanner := bufio.NewScanner(f)
	// Unverified JSONL allocation ceiling: 16 MiB accommodates bounded profiles.
	scanner.Buffer(make([]byte, 4096), 16*1024*1024)
	for scanner.Scan() {
		assertRedactedSessions(t, scanner.Bytes())
		var r savedRecord
		simfixture.Must(t, json.Unmarshal(scanner.Bytes(), &r))
		var fields map[string]json.RawMessage
		simfixture.Must(t, json.Unmarshal(scanner.Bytes(), &fields))
		if code, present := fields["exit_code"]; present != (r.Kind == "summary") || (present && string(code) == "null") {
			t.Fatalf("exit_code must be present only on summaries: %s", scanner.Bytes())
		}
		if r.HostNS == 0 || (len(records) > 0 && r.HostNS < records[len(records)-1].HostNS) {
			t.Fatal("evidence host monotonic timestamps regressed")
		}
		if r.Operation == "renew" && (r.StartedNS == 0 || r.StartedNS > r.HostNS) {
			t.Fatal("renew receipt lacks request start time")
		}
		records = append(records, r)
	}
	simfixture.Must(t, scanner.Err())
	if len(records) == 0 || records[len(records)-1].Kind != "summary" {
		t.Fatal("missing final summary")
	}
	return records
}

func (f *fixture) phase(t *testing.T, phase string, extra ...string) []savedRecord {
	t.Helper()
	var out bytes.Buffer
	args := append(f.args(phase, phase, true), extra...)
	if code := run(context.Background(), args, &out); code != 0 {
		raw, _ := os.ReadFile(filepath.Join(f.build, phase+".jsonl"))
		t.Fatalf("%s exit=%d: %s\n%s", phase, code, out.String(), raw)
	}
	records := readRecords(t, filepath.Join(f.build, phase+".jsonl"))
	if last := records[len(records)-1]; len(last.NotMet) != 0 || len(last.Met) < 5 || last.Class != "expectations_met" || last.ExitCode != 0 {
		t.Fatalf("incomplete summary: %+v", last)
	}
	f.poll(t, func(s control.Status) bool {
		return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Core.LeaseValid == 0
	})
	return records
}

func TestSimulationPhases(t *testing.T) {
	f := startFixture(t)
	before := f.poll(t, func(s control.Status) bool { return s.Core.HomeValidMask == 0 })
	for _, phase := range []string{"home", "enable", "jog"} {
		var out bytes.Buffer
		if code := run(context.Background(), f.args(phase, "dry-"+phase, false), &out); code != 3 {
			t.Fatalf("unconfirmed %s: exit=%d %s", phase, code, out.String())
		}
		if !strings.Contains(out.String(), "--confirm-motion") || (phase == "home" && !strings.Contains(out.String(), "Home energises the drive")) {
			t.Fatalf("missing confirmation description: %s", out.String())
		}
		dry := readRecords(t, filepath.Join(f.build, "dry-"+phase+".jsonl"))
		if len(dry) != 2 || dry[0].Kind != "plan" || dry[1].Class != "motion_confirmation_required" || dry[1].ExitCode != 3 {
			t.Fatalf("unconfirmed phase performed requests: %+v", dry)
		}
	}
	var rejectedOutput bytes.Buffer
	if code := run(context.Background(), append(f.args("jog", "bad-fraction", true), "--jog-fraction", "0.51"), &rejectedOutput); code != 2 {
		t.Fatalf("over-limit fraction exit=%d: %s", code, rejectedOutput.String())
	}
	invalid := readRecords(t, filepath.Join(f.build, "bad-fraction.jsonl"))
	if len(invalid) != 2 || invalid[0].Kind != "rejection" || invalid[0].Reason != "invalid_flags" || invalid[1].Class != "usage" || invalid[1].ExitCode != 2 {
		t.Fatal("missing flag rejection evidence")
	}
	after := f.poll(t, func(s control.Status) bool { return s.Core.TimeNS > before.Core.TimeNS })
	if before.Core.ControlGeneration != after.Core.ControlGeneration || after.Core.HomeValidMask != 0 || after.Core.Armed != 0 || after.Core.AxisEnableMask != 0 || after.Core.LeaseValid != 0 {
		t.Fatal("unconfirmed phases changed authority or outputs")
	}
	for i := range before.Core.Axes {
		if before.Core.Axes[i].PositionCounts != after.Core.Axes[i].PositionCounts {
			t.Fatal("unconfirmed phase changed position")
		}
	}
	for _, phase := range []string{"enable", "jog"} {
		var out bytes.Buffer
		if code := run(context.Background(), f.args(phase, "unhomed-"+phase, true), &out); code != 1 {
			t.Fatalf("unhomed %s exit=%d", phase, code)
		}
		for _, r := range readRecords(t, filepath.Join(f.build, "unhomed-"+phase+".jsonl")) {
			if r.Operation == "acquire" || r.Operation == "enable" && r.Kind == "request" {
				t.Fatal("unhomed motion acquired/enabled")
			}
		}
	}
	if code := run(context.Background(), f.args("inhibited", "inhibited-unconfirmed", false), io.Discard); code != 0 {
		t.Fatalf("unconfirmed inhibited exit=%d", code)
	}
	unconfirmed := readRecords(t, filepath.Join(f.build, "inhibited-unconfirmed.jsonl"))
	skipped, acquired, renewed, stopped, released := 0, 0, 0, 0, 0
	for _, r := range unconfirmed {
		if r.Kind == "skipped" && r.Operation == "enable" && r.Reason == "motion_confirmation_required" && bytes.Contains(r.Response, []byte("could energise the drive")) {
			skipped++
		}
		if r.Kind == "request" {
			switch r.Operation {
			case "home", "enable", "arm", "begin_jog", "update_jog":
				t.Fatalf("unconfirmed inhibited sent %s", r.Operation)
			case "acquire":
				acquired++
			case "renew":
				renewed++
			case "stop":
				stopped++
			case "release":
				released++
			}
		}
	}
	last := unconfirmed[len(unconfirmed)-1]
	if skipped != 1 || acquired != 2 || renewed < 1 || stopped != 1 || released != 1 || last.Class != "expectations_met" || len(last.NotMet) != 0 || strings.Contains(strings.Join(last.Met, ","), "unhomed_enable_refused") {
		t.Fatalf("inhibited skip or unconditional checks missing: %+v", unconfirmed)
	}
	inhibited := f.phase(t, "inhibited")
	refusals, renewals := 0, 0
	for _, r := range inhibited {
		if r.Kind == "request" && r.Operation == "renew" {
			renewals++
		}
		if r.Reason == "control_already_owned" {
			refusals++
		}
		if r.Reason == "native_rejected" {
			var native struct{ Reason uint32 }
			simfixture.Must(t, json.Unmarshal(r.NativeResult, &native))
			if native.Reason != 2 {
				t.Fatalf("unhomed Enable native reason=%d", native.Reason)
			}
			refusals++
		}
	}
	if refusals != 2 || renewals < 1 {
		t.Fatalf("refusals=%d renewals=%d", refusals, renewals)
	}
	home := f.phase(t, "home")
	transitions := 0
	for _, r := range home {
		if r.Kind == "statusword_transition" {
			transitions++
		}
	}
	if transitions < 1 {
		t.Fatal("Home never recorded a statusword transition")
	}
	f.poll(t, func(s control.Status) bool { return s.Core.HomeValidMask == 511 && s.Core.CommissioningPhase == 0 })
	enabled := f.phase(t, "enable", "--hold-ns", "700000000")
	operationEnabled, armed, drift, renewed := false, false, false, 0
	for _, r := range enabled {
		if r.Operation == "renew" {
			renewed++
		}
		if r.Operation == "hold_drift" {
			drift = true
			var p struct {
				Drift []int64 `json:"drift_counts"`
			}
			simfixture.Must(t, json.Unmarshal(r.Response, &p))
			if len(p.Drift) != 9 {
				t.Fatal("missing per-axis drift")
			}
			for _, counts := range p.Drift {
				if counts < -2 || counts > 2 {
					t.Fatalf("hold drift %d counts exceeds 2", counts)
				}
			}
		}
		if r.Kind == "status" {
			var s control.Status
			simfixture.Must(t, json.Unmarshal(r.Response, &s))
			if s.Core.AxisEnableMask == 511 {
				all := len(s.Core.Axes) == 9
				for _, a := range s.Core.Axes {
					all = all && a.Statusword&0x006f == 0x0027
				}
				operationEnabled = operationEnabled || all
			}
			armed = armed || s.Core.Armed == 1
		}
	}
	if !operationEnabled || !armed || !drift || renewed < 5 {
		t.Fatalf("enabled=%v armed=%v drift=%v renewals=%d", operationEnabled, armed, drift, renewed)
	}
	jog := f.phase(t, "jog", "--jog-duration-ns", "1000000000")
	positions := map[string][]int32{}
	positive, negative, frames := false, false, 0
	for _, r := range jog {
		if r.Kind == "status" {
			var s control.Status
			simfixture.Must(t, json.Unmarshal(r.Response, &s))
			if s.Core.Axes[0].PositionCounts < -15 || s.Core.Axes[0].PositionCounts > 150 {
				t.Fatalf("jog position %d counts outside -15..150", s.Core.Axes[0].PositionCounts)
			}
		}
		if r.Kind == "position" {
			var p struct {
				Counts []int32 `json:"pos_counts"`
			}
			simfixture.Must(t, json.Unmarshal(r.Response, &p))
			positions[r.Operation] = p.Counts
		}
		if r.Operation == "update_jog" {
			var req struct {
				Velocity []float64 `json:"velocity"`
				Deadline uint64    `json:"deadline_host_ns"`
				End      uint64    `json:"jog_end_host_ns"`
			}
			simfixture.Must(t, json.Unmarshal(r.Request, &req))
			if len(req.Velocity) != 9 || req.Velocity[0] < -0.0100000001 || req.Velocity[0] > 0.0100000001 || (req.Velocity[0] != 0 && req.Deadline > req.End) || req.Deadline-r.StartedNS > 100000000 || r.StartedNS >= req.End {
				t.Fatalf("unbounded jog request: %s", r.Request)
			}
			for _, v := range req.Velocity[1:] {
				if v != 0 {
					t.Fatal("jog commanded an unselected axis")
				}
			}
			positive = positive || req.Velocity[0] > 0
			negative = negative || req.Velocity[0] < 0
			frames++
		}
	}
	if !positive || !negative || frames < 2 || len(positions) != 3 {
		t.Fatalf("jog evidence missing: positive=%v negative=%v frames=%d positions=%v", positive, negative, frames, positions)
	}
	peak := positions["jog_reversal"][0] - positions["jog_before"][0]
	returned := positions["jog_after"][0] - positions["jog_before"][0]
	if peak < 50 || peak > 150 || returned < -15 || returned > 15 {
		t.Fatalf("jog excursion=%d counts (50..150), return=%d counts (-15..15)", peak, returned)
	}
	if !reflect.DeepEqual(positions["jog_before"][1:], positions["jog_after"][1:]) {
		t.Fatal("unselected axes moved")
	}
}

func TestMidJogStop(t *testing.T) {
	f := startFixture(t)
	f.phase(t, "home")
	r := testRunner(t, f, "jog", "interrupted")
	done := make(chan int, 1)
	ctx, cancel := context.WithTimeout(context.Background(), fixtureDeadline)
	defer cancel()
	go func() {
		err := r.summary(r.run(ctx))
		code, _ := outcome(err)
		done <- code
	}()
	joined := false
	defer func() {
		cancel()
		if !joined {
			<-done
		}
	}()
	f.poll(t, func(s control.Status) bool {
		select {
		case code := <-done:
			joined = true
			t.Fatalf("jog exited before external Stop: %d", code)
		default:
		}
		return s.Core.Armed == 1 && s.Core.Axes[0].PositionCounts >= 20
	})
	// The test reads the owner's in-memory fence; redacted evidence must never
	// supply usable authority to the independent Stop connection.
	grant := r.c.Grant()
	if grant.Session == "" {
		t.Fatal("missing acquire fence")
	}
	f.client.SetFence(control.Fence{Session: grant.Session, Generation: grant.Generation})
	stopped, err := f.client.Stop(ctx)
	simfixture.Must(t, err)
	if stopped.Generation != grant.Generation+1 {
		t.Fatalf("external Stop generation=%d", stopped.Generation)
	}
	select {
	case code := <-done:
		joined = true
		if code != 1 {
			t.Fatalf("interrupted jog exit=%d", code)
		}
	case <-ctx.Done():
		t.Fatal("interrupted jog failed to terminate")
	}
	records := readRecords(t, filepath.Join(f.build, "interrupted.jsonl"))
	stop, release, refused := false, false, false
	for _, r := range records {
		stop = stop || r.Operation == "stop"
		release = release || r.Operation == "release"
		refused = refused || r.Reason == "control_session_stale" || r.Reason == "jog_session_stale" || r.NativeJogResult != nil
	}
	if !stop || !release || !refused || len(records[len(records)-1].NotMet) == 0 {
		t.Fatalf("interruption evidence stop=%v release=%v refusal=%v", stop, release, refused)
	}
	f.poll(t, func(s control.Status) bool {
		return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Core.LeaseValid == 0
	})
}

func TestFlagBounds(t *testing.T) {
	f := fixture{socket: "control.sock", hash: strings.Repeat("a", 64), build: "."}
	base := f.args("jog", "unused", true)
	for _, bad := range [][]string{
		{"--jog-fraction", "0.500001"}, {"--jog-fraction", "0"}, {"--jog-fraction", "-0.1"}, {"--jog-fraction", "NaN"}, {"--jog-fraction", "Inf"},
		{"--jog-duration-ns", "2000000001"}, {"--jog-duration-ns", "0"}, {"--jog-duration-ns", "-1"},
		{"--deadline-ns", "0"}, {"--hold-ns", "-1"}, {"--axis", "16"}, {"--axis-mask", "3"},
		{"--axis-mask", "4294967297"}, {"--machine-sha256", strings.Repeat("b", 64)}, {"--configuration-sha256", "bad"},
	} {
		t.Run(strings.Join(bad, "="), func(t *testing.T) {
			if _, err := parseOptions(append(append([]string{}, base...), bad...), io.Discard); err == nil {
				t.Fatal("invalid flags accepted")
			}
		})
	}
	if _, err := parseOptions(append(base, "--jog-fraction", "0.5", "--jog-duration-ns", "2000000000"), io.Discard); err != nil {
		t.Fatal(err)
	}
	for _, phase := range []string{"inhibited", "home", "enable", "jog"} {
		o, err := parseOptions(f.args(phase, "unused", true), io.Discard)
		simfixture.Must(t, err)
		if o.deadlineNS != 10000000000 || o.holdNS != 2000000000 || o.fraction != 0.1 || o.controller != "benchdrive" {
			t.Fatalf("defaults: %+v", o)
		}
	}
}

func TestEvidenceRefusesOverwrite(t *testing.T) {
	root, err := filepath.Abs("../..")
	simfixture.Must(t, err)
	f, err := os.CreateTemp(filepath.Join(root, "build"), "benchdrive-existing-")
	simfixture.Must(t, err)
	t.Cleanup(func() { os.Remove(f.Name()) })
	_, err = fmt.Fprint(f, "previous qualification\n")
	simfixture.Must(t, err)
	simfixture.Must(t, f.Close())
	if _, err := openEvidence(f.Name()); err == nil {
		t.Fatal("overwrote existing evidence")
	}
	raw, err := os.ReadFile(f.Name())
	simfixture.Must(t, err)
	if string(raw) != "previous qualification\n" {
		t.Fatal("evidence changed")
	}
}

func TestMissingJogLimitsFailClosed(t *testing.T) {
	validAxis := ipcclient.AxisDescription{MaxVelocity: .1, JogAcceleration: 1, CountsPerUnit: 20000,
		MaxTargetLead: .02, FollowingError: .04, CompletionTolerance: .0002, MinPosition: -.5, MaxPosition: .5, Sign: 1}
	for _, name := range []string{"velocity", "acceleration", "counts", "lead", "following", "tolerance", "position", "home", "logical"} {
		t.Run(name, func(t *testing.T) {
			a := validAxis
			s := control.Status{Core: ipcclient.Status{Axes: []ipcclient.AxisStatus{{HomeValid: true, LogicalValid: true}}}}
			switch name {
			case "velocity":
				a.MaxVelocity = 0
			case "acceleration":
				a.JogAcceleration = 0
			case "counts":
				a.CountsPerUnit = 0
			case "lead":
				a.MaxTargetLead = 0
			case "following":
				a.FollowingError = 0
			case "tolerance":
				a.CompletionTolerance = 0
			case "position":
				a.MinPosition, a.MaxPosition = 0, 0
			case "home":
				s.Core.Axes[0].HomeValid = false
			case "logical":
				s.Core.Axes[0].LogicalValid = false
			}
			r := runner{o: options{axis: 0, fraction: .1, jogNS: 1000000000},
				description:  control.Description{NativeDescription: ipcclient.Description{Axes: []ipcclient.AxisDescription{a}}},
				expectations: make(map[string]bool)}
			if err := r.jogLimits(s); err == nil {
				t.Fatal("missing limit or coordinate evidence admitted jog")
			}
		})
	}
}
