//go:build linux

package ipcclient_test

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
)

// Build only fixture configuration inside this worktree; Unix sockets use TMPDIR.
// Returned Describe metadata is the native authority for adapter parity checks.
func hermiteAdmissionCore(t *testing.T, sign int, velocity float64, ratio ...float64) (*ipcclient.Client, string) {
	t.Helper()
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}
	dir, err := os.MkdirTemp(filepath.Join(root, "build"), "hermite-admission-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	machine := interpolationJSON(t, filepath.Join(root, "config/machines/simulation/simulation-program.json"))
	profile := interpolationJSON(t, filepath.Join(root, "config/drives/cia402_sim.json"))
	for _, item := range machine["axes"].([]any) {
		axis := item.(map[string]any)
		axis["sign"] = sign
		if len(ratio) != 0 {
			axis["wire_revs_per_axis_rev"] = ratio[0]
		}
		// These analytic cases use a synthetic +/-0.5 rad envelope: the
		// .5025 rad Hermite extremum must be outside it, and recovery cases
		// must agree with the same native bounds. Do not inherit robot limits.
		limits := axis["limits"].(map[string]any)
		limits["min"], limits["max"] = -.5, .5
		limits["max_velocity"], limits["jog_acceleration"] = velocity, 1.0
		delete(limits, "max_source")
		delete(limits, "max_velocity_source")
	}
	raw, err := json.Marshal(profile)
	if err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(dir, "cia402_sim.json"), raw, 0600); err != nil {
		t.Fatal(err)
	}
	raw, err = json.Marshal(machine)
	if err != nil {
		t.Fatal(err)
	}
	config := filepath.Join(dir, "machine.json")
	if err := os.WriteFile(config, raw, 0600); err != nil {
		t.Fatal(err)
	}
	compiled, args, err := simfixture.CompileProfiles(root, config, dir, dir)
	if err != nil {
		t.Fatal(err)
	}
	sockets, err := os.MkdirTemp("", "ha-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(sockets) })
	socket, logPath := filepath.Join(sockets, "ipc.sock"), filepath.Join(dir, "core.log")
	process := exec.Command(simfixture.Locate(root, "rosie-rt-core-sim"), append(args, "--socket-path", socket)...)
	simfixture.SpawnLogged(t, process, logPath, 3*time.Second, 0, nil)
	simfixture.WaitVerified(t, filepath.Join(sockets, "metrics.json"), 5*time.Second, 20*time.Millisecond,
		func(s simfixture.Verified) bool { return s.ConfigVerifiedMask == 511 })
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	c, err := ipcclient.Dial(ctx, ipcclient.Options{SocketPath: socket, ConfigurationSHA256: filepath.Base(compiled), Backend: "simulation", SimulationTest: true})
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = c.Close() })
	return c, logPath
}

// Exact position bounds and one-count interior points must agree with native
// admission on the largest described gear ratio; preparation never moves axes.
func TestProgramLargestRatioExactPositionLimit(t *testing.T) {
	c, _ := hermiteAdmissionCore(t, 1, .1, 100)
	description, err := c.Describe()
	if err != nil {
		t.Fatal(err)
	}
	axis := description.Axes[0]
	if math.Abs(axis.CountsPerUnit-2086075.6700940907) > 0.000000001 || axis.MaxPosition != .5 {
		t.Fatalf("largest-ratio description changed: %+v", axis)
	}
	ids := []string{"J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"}
	for index, position := range []float64{.5, .5 - 1/axis.CountsPerUnit, -.5, -.5 + 1/axis.CountsPerUnit} {
		samples := []densejointtraj.Sample{{}, {TimeS: .01}}
		points := []ipcclient.Point{{Position: make([]float64, 9), Velocity: make([]float64, 9)},
			{Time: 10_000_000, Position: make([]float64, 9), Velocity: make([]float64, 9)}}
		for i := range samples {
			samples[i].QRad[0] = position
			points[i].Position[0] = position
		}
		if _, err := control.DecodeProgram(hermiteProgram(t, samples, .01, .1), description, ids); err != nil {
			t.Fatalf("adapter bound %0.17g: %v", position, err)
		}
		id := uint64(8900 + index)
		if err := interpolationPrepare(t, c, id, points); err != nil {
			t.Fatalf("native bound %0.17g: %v", position, err)
		}
		if _, err := c.DiscardTrajectory(context.Background(), id); err != nil {
			t.Fatal(err)
		}
	}
	t.Logf("100:1 scale=%.17g counts/rad; one-count margin=%.17g rad", axis.CountsPerUnit, 1/axis.CountsPerUnit)
}

func TestHermitePositionExtremaAdmissionParity(t *testing.T) {
	for _, frame := range []struct {
		name string
		sign int
	}{{"continuous", 1}, {"negative", -1}} {
		t.Run(frame.name, func(t *testing.T) {
			c, logPath := hermiteAdmissionCore(t, frame.sign, .1)
			description, err := c.Describe()
			if err != nil {
				t.Fatal(err)
			}
			ids := []string{"J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"}
			// Stationary endpoints surround the reviewer's exact interior segment.
			samples := []densejointtraj.Sample{{}, {TimeS: 1}, {TimeS: 2}, {TimeS: 3}}
			samples[0].QRad[0], samples[3].QRad[0] = .465, .465
			samples[1].QRad[0], samples[2].QRad[0] = .49, .49
			samples[1].QdRadS[0], samples[2].QdRadS[0] = .05, -.05
			_, err = control.DecodeProgram(hermiteProgram(t, samples, 1, .1), description, ids)
			var rejected *control.SegmentPositionError
			if !errors.As(err, &rejected) || rejected.Axis != "J1" || rejected.Sample != 2 || rejected.Limit != .5 || math.Abs(rejected.Position-.5025) > 1e-12 {
				t.Fatalf("position extremum: %+v %v", rejected, err)
			}
			points := []ipcclient.Point{
				{Position: []float64{.49, 0, 0, 0, 0, 0, 0, 0, 0}, Velocity: []float64{.05, 0, 0, 0, 0, 0, 0, 0, 0}},
				{Time: time.Second, Position: []float64{.49, 0, 0, 0, 0, 0, 0, 0, 0}, Velocity: []float64{-.05, 0, 0, 0, 0, 0, 0, 0, 0}},
			}
			interpolationRejected(t, interpolationPrepare(t, c, 8501, points), 8501)
			raw, err := os.ReadFile(logPath)
			if err != nil || !strings.Contains(string(raw), "hermite_position_extremum axis=0 (J1) segment=0") || !strings.Contains(string(raw), "limit=0.5") {
				t.Fatalf("missing position/axis/limit diagnostic: %v %s", err, raw)
			}
			points[0].Position[0], points[1].Position[0] = .48, .48
			if err := interpolationPrepare(t, c, 8502, points); err != nil {
				t.Fatalf("legal successor: %v", err)
			}
		})
	}
}

// The independent review's binary64 cap straddles an analytic 0.00135 rad/s
// peak. Exercise real Describe/Commit and adapter admission, without rad slack.
func TestHermiteVelocityBoundaryAdmissionParity(t *testing.T) {
	for _, test := range []struct {
		name     string
		limit    float64
		accepted bool
	}{
		{"review_refusal", .0013499999999999999, false},
		{"above_guard", .001350000000000001, true},
	} {
		for _, sign := range []int{-1, 1} {
			t.Run(fmt.Sprintf("%s/sign_%d", test.name, sign), func(t *testing.T) {
				c, _ := hermiteAdmissionCore(t, sign, test.limit)
				description, err := c.Describe()
				if err != nil {
					t.Fatal(err)
				}
				if description.Axes[0].CountsPerUnit != 131072/(2*math.Pi) || description.Axes[0].MaxVelocity != test.limit {
					t.Fatalf("review fixture changed: %+v", description.Axes[0])
				}
				ids := []string{"J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"}
				samples := []densejointtraj.Sample{{}, {TimeS: .01}}
				samples[1].QRad[0] = .000009
				_, adapterErr := control.DecodeProgram(hermiteProgram(t, samples, .01, test.limit), description, ids)
				points := []ipcclient.Point{
					{Position: make([]float64, 9), Velocity: make([]float64, 9)},
					{Time: 10_000_000, Position: []float64{.000009, 0, 0, 0, 0, 0, 0, 0, 0}, Velocity: make([]float64, 9)},
				}
				nativeErr := interpolationPrepare(t, c, 8601, points)
				if (adapterErr == nil) != test.accepted || (nativeErr == nil) != test.accepted {
					t.Fatalf("accept=%v: adapter=%v native=%v", test.accepted, adapterErr, nativeErr)
				}
				if !test.accepted {
					interpolationRejected(t, nativeErr, 8601)
					var rejected *control.SegmentVelocityError
					if !errors.As(adapterErr, &rejected) || rejected.Axis != "J1" || rejected.Sample != 1 || rejected.Limit != test.limit {
						t.Fatalf("missing typed boundary refusal: %+v %v", rejected, adapterErr)
					}
				}
			})
		}
	}
}

// Encode a single stationary-ended dense segment so admission and execution
// exercise the same public source format without weakening its digest checks.
func hermiteProgram(t *testing.T, samples []densejointtraj.Sample, dt, limit float64) []byte {
	t.Helper()
	limits := make([]float64, 9)
	for i := range limits {
		limits[i] = limit
	}
	duration := samples[len(samples)-1].TimeS
	segments := []densejointtraj.Segment{{Kind: "freespace", SourceID: "hermite", Samples: samples}}
	header := densejointtraj.Header{
		Schema: densejointtraj.Schema, PlanID: "hermite", ProgramID: "hermite", ProgramDigest: "sha256:" + strings.Repeat("0", 64),
		ManifestRevision: 1, PlanRevision: 1,
		Axes:           densejointtraj.HeaderAxes{AxisCount: 9, AxisMask: 511, PositionUnit: "rad", VelocityUnit: "rad/s"},
		Sampling:       densejointtraj.Sampling{DtS: dt, TotalSampleCount: len(samples), TotalDurationS: duration},
		Limits:         densejointtraj.Limits{MaxAbsQdRadS: limits},
		Segments:       []densejointtraj.SegmentMeta{{Index: 0, Kind: "freespace", SourceID: "hermite", SampleCount: len(samples), DurationS: duration}},
		SampleEncoding: densejointtraj.EncodingMeta{Encoding: densejointtraj.SampleEncoding, RecordBytes: densejointtraj.RecordBytes},
	}
	blob, err := densejointtraj.Encode(header, segments)
	if err != nil {
		t.Fatal(err)
	}
	return blob
}

func TestDenseProgramRejectsHermiteVelocityExtremum(t *testing.T) {
	var description ipcclient.Description
	var ids []string
	for i := 0; i < 9; i++ {
		id := fmt.Sprintf("J%d", i+1)
		ids = append(ids, id)
		description.Axes = append(description.Axes, ipcclient.AxisDescription{
			ID: id, PositionUnit: "rad", CountsPerUnit: 4096, Sign: 1, MinPosition: -1, MaxPosition: 1, MaxVelocity: .1})
	}
	samples := []densejointtraj.Sample{{}, {TimeS: .01}}
	samples[1].QRad[0] = .0006 // Measured polynomial peak: 0.09 rad/s, knots zero.
	for _, limits := range [][2]float64{{.08, .1}, {.1, .08}} {
		description.Axes[0].MaxVelocity = limits[1]
		_, err := control.DecodeProgram(hermiteProgram(t, samples, .01, limits[0]), description, ids)
		var rejected *control.SegmentVelocityError
		if !errors.As(err, &rejected) || rejected.Error() != control.ReasonNativeSegmentRateExceeded ||
			rejected.Axis != "J1" || rejected.Segment != 0 || rejected.Sample != 1 ||
			math.Abs(rejected.Peak-.09) > 1e-12 || rejected.Limit != .08 {
			t.Fatalf("missing continuous velocity refusal: %+v %v", rejected, err)
		}
	}
	description.Axes[0].MaxVelocity = .1
	if _, err := control.DecodeProgram(hermiteProgram(t, samples, .01, .1), description, ids); err != nil {
		t.Fatal(err)
	}
	// The raw source rate is legal, but the continuous linear-mode path between
	// these endpoints exceeds 0.7 rad/s.
	samples[0].QRad[0], samples[1].QRad[0], samples[1].TimeS = 3.13, -3.13, 10
	description.Axes[0].MinPosition, description.Axes[0].MaxPosition = -4, 4
	description.Axes[0].MaxVelocity = .7
	_, err := control.DecodeProgram(hermiteProgram(t, samples, 10, .7), description, ids)
	var rejected *control.SegmentVelocityError
	if !errors.As(err, &rejected) || math.Abs(rejected.Peak-.939) > 1e-12 {
		t.Fatalf("continuous command frame peak: %+v %v", rejected, err)
	}
}

// Read-only benchdrive programProfile uses central differences for qd. These
// knots come from its 0.2 rad move, 0.0375 rad/s cap and 0.5 rad/s^2 ramp on
// the 10 ms grid (541 intervals). Preserve the actual overspeed refusal instead
// of adding interpolation slack to make that producer's old tests pass.
func TestDenseProgramRejectsCentralDifferenceVelocityOvershoot(t *testing.T) {
	samples := []densejointtraj.Sample{{}, {TimeS: .01}, {TimeS: .02}, {TimeS: .03}}
	samples[1].QRad[0], samples[1].QdRadS[0] = .0012242453411545147, .034669011440783645
	samples[2].QRad[0], samples[2].QdRadS[0] = .0015928257855822548, .037173245881276094
	// Quadratic approach/departure segments stay below the cap and isolate the
	// offending interior segment while retaining the format's zero endpoint qd.
	samples[0].QRad[0] = samples[1].QRad[0] - samples[1].QdRadS[0]*.005
	samples[3].QRad[0] = samples[2].QRad[0] + samples[2].QdRadS[0]*.005
	var description ipcclient.Description
	var ids []string
	for i := 0; i < 9; i++ {
		id := fmt.Sprintf("J%d", i+1)
		ids = append(ids, id)
		description.Axes = append(description.Axes, ipcclient.AxisDescription{
			ID: id, PositionUnit: "rad", CountsPerUnit: 4096, Sign: 1, MinPosition: -1, MaxPosition: 1, MaxVelocity: .1})
	}
	_, err := control.DecodeProgram(hermiteProgram(t, samples, .01, .0375), description, ids)
	var rejected *control.SegmentVelocityError
	if !errors.As(err, &rejected) || rejected.Sample != 2 ||
		math.Abs(rejected.Peak*4096-154.03170075446442) > 1e-9 || rejected.Limit != .0375 {
		t.Fatalf("central-difference overspeed must fail closed: %+v %v", rejected, err)
	}
}

// Observe every submitted position through the daemon's retained cycle ring.
// Finite differences measure CSP commanded velocity, so linear q with smooth
// feedforward cannot pass. Absolute acceleration allowance is 3 rad/s^2;
// the independent cubic fixture reaches 0.6 rad/s^2 and 0.03 rad/s.
func TestDenseProgramHermitePerCycleCommandVelocity(t *testing.T) {
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}
	dir, err := os.MkdirTemp(filepath.Join(root, "build"), "hermite-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	sockets, err := os.MkdirTemp("", "hm-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(sockets) })
	machine := interpolationJSON(t, filepath.Join(root, "config/machines/simulation/simulation-program.json"))
	// Unverified synthetic high-resolution encoder: 10 rpm yields 104719755
	// counts/s, within U32; the production 3000 rpm cap cannot fit this fixture.
	machine["drive_speed_limit_motor_rpm"] = 10
	for _, item := range machine["axes"].([]any) {
		// Unverified fixture resolution: 628318531 counts/revolution gives
		// 100000000.0444 counts/rad, resolving per-cycle CSP derivatives.
		item.(map[string]any)["wire_counts_per_rev"] = 628318531
	}
	raw, err := json.Marshal(machine)
	if err != nil {
		t.Fatal(err)
	}
	config := filepath.Join(dir, "machine.json")
	if err := os.WriteFile(config, raw, 0600); err != nil {
		t.Fatal(err)
	}
	compiled, args, err := simfixture.Compile(root, config, filepath.Join(dir, "config"))
	if err != nil {
		t.Fatal(err)
	}
	socket := filepath.Join(sockets, "ipc.sock")
	process := exec.Command(simfixture.Locate(root, "rosie-rt-core-sim"), append(args, "--socket-path", socket)...)
	simfixture.SpawnLogged(t, process, filepath.Join(dir, "core.log"), 3*time.Second, 0, nil)
	simfixture.WaitVerified(t, filepath.Join(sockets, "metrics.json"), 5*time.Second, 20*time.Millisecond,
		func(s simfixture.Verified) bool { return s.ConfigVerifiedMask == 511 })
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	c, err := ipcclient.Dial(ctx, ipcclient.Options{SocketPath: socket, ConfigurationSHA256: filepath.Base(compiled), Backend: "simulation"})
	if err != nil {
		t.Fatal(err)
	}
	defer c.Close()
	binding := control.Binding{PairID: "hermite", Revision: 1, ConfigurationSHA256: filepath.Base(compiled)}
	controller, err := control.New(c, binding)
	if err != nil {
		t.Fatal(err)
	}
	defer controller.Close()
	grant, err := controller.Acquire("hermite", binding)
	if err != nil {
		t.Fatal(err)
	}
	fence := control.Fence{Session: grant.Session, Generation: grant.Generation}
	stopRenew := simfixture.Renew(ctx, 100*time.Millisecond, func() bool { _, err := controller.Renew(fence); return err == nil })
	defer stopRenew()
	if _, err := controller.Enable(fence, 511); err != nil {
		t.Fatal(err)
	}
	if _, err := controller.Arm(fence); err != nil {
		t.Fatal(err)
	}
	if _, err := c.Wait(ctx, func(s ipcclient.Status) bool { return s.Armed == 1 && ipcclient.AllOperationEnabled(s) }); err != nil {
		t.Fatal(err)
	}
	description, err := c.Describe()
	if err != nil || description.CycleNS != 1_000_000 || description.Axes[0].MaxAcceleration == nil || *description.Axes[0].MaxAcceleration != 3 {
		t.Fatalf("fixture contract: %+v %v", description, err)
	}
	samples := make([]densejointtraj.Sample, 21)
	for i := range samples {
		u := float64(i) / 20
		samples[i].TimeS = float64(i) / 100
		samples[i].QRad[0] = .004 * (3*u*u - 2*u*u*u)
		samples[i].QdRadS[0] = .12 * u * (1 - u)
	}
	program, err := controller.PrepareProgram(fence, hermiteProgram(t, samples, .01, .1))
	if err != nil {
		t.Fatal(err)
	}
	reader, err := c.OpenTelemetry()
	if err != nil {
		t.Fatal(err)
	}
	defer reader.Close()
	start, err := reader.Written()
	if err != nil {
		t.Fatal(err)
	}
	if _, err := controller.StartProgram(fence, program.Identity); err != nil {
		t.Fatal(err)
	}
	if !simfixture.PollUntil(time.Now().Add(3*time.Second), time.Millisecond, func() bool {
		motion, err := c.Motion()
		return err == nil && motion.Done
	}) {
		t.Fatal("dense program did not complete")
	}
	batch, err := reader.ReadFrom(start, 4096)
	if err != nil || batch.Drops != 0 {
		t.Fatalf("cycle capture: %v %+v", err, batch)
	}
	var previousVelocity, previousMidpoint float64
	moving, checked := 0, 0
	for i := 1; i < len(batch.Records); i++ {
		p, r := batch.Records[i-1], batch.Records[i]
		if r.Cycle != p.Cycle+1 || r.TNs <= p.TNs {
			t.Fatal("missing commanded cycle")
		}
		dt := float64(r.TNs-p.TNs) / 1e9
		velocity := float64(r.Ax[0].Target-p.Ax[0].Target) / dt
		midpoint := float64(p.TNs-batch.Records[0].TNs)/1e9 + dt/2
		if i > 1 {
			// Unverified acceptance bound: 300000000 counts/s^2 (3 rad/s^2).
			if math.Abs(velocity-previousVelocity) > 300000000*(midpoint-previousMidpoint) {
				t.Fatalf("command velocity step at cycle %d: %.9f to %.9f counts/s over %.9f s", r.Cycle, previousVelocity, velocity, midpoint-previousMidpoint)
			}
			checked++
		}
		if r.Ax[0].Target > 0 && r.Ax[0].Target < 400000 {
			moving++
		}
		previousVelocity, previousMidpoint = velocity, midpoint
	}
	if checked < 180 || moving < 180 || len(batch.Records) == 0 || batch.Records[len(batch.Records)-1].Ax[0].Target != 400000 {
		t.Fatalf("incomplete 200 ms command capture: checks=%d moving=%d", checked, moving)
	}
	t.Logf("10 ms source grid; %d consecutive cycle velocity differences satisfy 300000000 counts/s^2", checked)
}
