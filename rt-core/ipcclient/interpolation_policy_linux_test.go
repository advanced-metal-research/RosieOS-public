//go:build linux

package ipcclient_test

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"net/http/httptest"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"rosieos/rt-core/internal/simfixture"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/tools/rtctl"
)

// Unverified simulation fixture: 4096 counts/m makes count boundaries exact;
// the declared acceleration is 40 counts/s^2, below the 409.6 counts/s cap.
const interpolationScale = 4096.0

// Unverified test bound in m/s^2, exactly 40 counts/s^2 at the fixture scale.
const interpolationLimit = 40.0 / 4096.0

func interpolationJSON(t *testing.T, path string) map[string]any {
	t.Helper()
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	var value map[string]any
	if err := json.Unmarshal(raw, &value); err != nil {
		t.Fatal(err)
	}
	return value
}

// Compile profiles against the frozen original output before starting the
// actual daemon, preventing description-only declarations or compiler drift.
func interpolationCore(t *testing.T, omitSecond bool, production ...bool) (*ipcclient.Client, string) {
	t.Helper()
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}
	dir, err := os.MkdirTemp(filepath.Join(root, "build"), "interpolation-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	// Keep the frozen oracle's speed field and identity; migrated cell caps
	// are covered by the compiler's per-axis and native-emission regressions.
	machine := interpolationJSON(t, filepath.Join(root, "tools/rtctl/testdata/parity/frozen/config/simulation.json"))
	for i, item := range machine["axes"].([]any) {
		axis := item.(map[string]any)
		// Preserve the captured synthetic encoder before changing the wire scale;
		// a different historical input must not inherit this coordinate declaration.
		if axis["wire_counts_per_rev"] != float64(131072) || axis["motor_encoder_counts_per_rev"] != nil || axis["coordinate_evidence_mode"] != nil {
			t.Fatal("historical interpolation coordinate inputs changed")
		}
		axis["motor_encoder_counts_per_rev"] = axis["wire_counts_per_rev"]
		axis["motor_encoder_counts_per_rev_unverified"] = "unverified synthetic encoder matching the declared wire revolution"
		axis["coordinate_evidence_mode"] = "multi_turn"
		profile := interpolationJSON(t, filepath.Join(root, "config/drives", axis["profile"].(string)+".json"))
		profile["max_acceleration"] = interpolationLimit
		if omitSecond && i == 1 {
			delete(profile, "max_acceleration")
		}
		axis["profile"] = fmt.Sprintf("interpolation_%d", i)
		axis["type"], axis["wire_counts_per_rev"], axis["lead_m_per_rev"] = "linear", interpolationScale, 1
		raw, err := json.Marshal(profile)
		if err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(filepath.Join(dir, axis["profile"].(string)+".json"), raw, 0600); err != nil {
			t.Fatal(err)
		}
	}
	raw, err := json.Marshal(machine)
	if err != nil {
		t.Fatal(err)
	}
	machinePath := filepath.Join(dir, "machine.json")
	if err := os.WriteFile(machinePath, raw, 0600); err != nil {
		t.Fatal(err)
	}
	compiled, err := simfixture.CompileRaw(raw, dir, rtctl.Compile)
	if err != nil {
		t.Fatal(err)
	}
	// The retired wrap argv/profile golden is intentionally not compared. Keep
	// the current coordinate identities and native startup proof explicit.
	if len(compiled.CoordinateIdentitySHA256) != 2 || len(compiled.CoordinateIdentitySHA256[0]) != 64 || len(compiled.CoordinateIdentitySHA256[1]) != 64 {
		t.Fatal("interpolation axes missing coordinate identities")
	}
	directory, args, err := simfixture.CompileProfiles(root, machinePath, dir, dir)
	if err != nil {
		t.Fatal(err)
	}
	if filepath.Base(directory) != compiled.SHA256 {
		t.Fatal("raw and file fixture compilation selected different identities")
	}
	socketDir, err := os.MkdirTemp("", "in-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(socketDir) })
	socket := filepath.Join(socketDir, "ipc.sock")
	logPath := filepath.Join(dir, "core.log")
	binary := simfixture.Locate(root, "rosie-rt-core-sim")
	args = append(args, "--socket-path", socket)
	process := exec.Command(binary, args...)
	simfixture.SpawnLogged(t, process, logPath, 3*time.Second, 0, nil)
	var client *ipcclient.Client
	t.Cleanup(func() {
		if client != nil {
			_ = client.Close()
		}

	})
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	ticker := time.NewTicker(time.Millisecond)
	defer ticker.Stop()
	for {
		raw, _ := os.ReadFile(filepath.Join(socketDir, "metrics.json"))
		var status ipcclient.Status
		if json.Unmarshal(raw, &status) == nil && status.ConfigVerifiedMask == 3 {
			break
		}
		select {
		case <-ctx.Done():
			t.Fatal("interpolation daemon did not verify configuration")
		case <-ticker.C:
		}
	}
	client, err = ipcclient.Dial(ctx, ipcclient.Options{SocketPath: socket, ConfigurationSHA256: compiled.SHA256, Backend: "simulation", SimulationTest: len(production) == 0})
	if err != nil {
		t.Fatal(err)
	}
	return client, logPath
}

func interpolationPrepare(t *testing.T, c *ipcclient.Client, id uint64, points []ipcclient.Point) error {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	handle, err := c.PrepareTrajectory(ctx, id, 3, points)
	if err == nil {
		if handle != id {
			t.Fatalf("prepared handle %d, want %d", handle, id)
		}
		if _, err := c.DiscardTrajectory(ctx, handle); err != nil {
			t.Fatal(err)
		}
	}
	return err
}

func interpolationRejected(t *testing.T, err error, id uint64) {
	t.Helper()
	var rejected *ipcclient.RejectedError
	if !errors.As(err, &rejected) || rejected.Result.Reason != ipcclient.ReasonInvalidTrajectory || rejected.Result.Handle != id || rejected.Result.Operation != ipcclient.MSG_CMD_TRAJECTORY_COMMIT {
		t.Fatalf("expected invalid_trajectory at Commit for %d, got %v", id, err)
	}
}

func TestPrepareRejectsAccelerationBeyondDeclaredLimit(t *testing.T) {
	c, logPath := interpolationCore(t, false)
	points := []ipcclient.Point{
		{Position: []float64{0, 0}, Velocity: []float64{0, 0}},
		{Time: time.Second, Position: []float64{0, 0}, Velocity: []float64{40 / interpolationScale, 0}},
	}
	if err := interpolationPrepare(t, c, 8101, points); err != nil {
		t.Fatalf("exact bound: %v", err)
	}
	points[1].Velocity[0] = 41 / interpolationScale
	interpolationRejected(t, interpolationPrepare(t, c, 8102, points), 8102)
	raw, err := os.ReadFile(logPath)
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(string(raw), "axis=0 (axis_0) segment=0 acceleration=0.010009765625 m/s^2") {
		t.Fatalf("missing axis/segment/logical acceleration diagnostic: %s", raw)
	}
	points[1].Velocity = []float64{0, 41 / interpolationScale}
	interpolationRejected(t, interpolationPrepare(t, c, 8103, points), 8103)
	points[1].Velocity = nil
	interpolationRejected(t, interpolationPrepare(t, c, 8104, points), 8104)
	points[1].Velocity = []float64{0, 0}
	if err := interpolationPrepare(t, c, 8105, points); err != nil {
		t.Fatalf("valid successor: %v", err)
	}
}

func TestPrepareAcceptsPlanWithoutQdUsingSecondDifferences(t *testing.T) {
	c, _ := interpolationCore(t, false)
	points := []ipcclient.Point{
		{Position: []float64{0, 0}},
		{Time: time.Second, Position: []float64{20 / interpolationScale, 0}},
		{Time: 2 * time.Second, Position: []float64{80 / interpolationScale, 0}},
	}
	if err := interpolationPrepare(t, c, 8201, points); err != nil {
		t.Fatalf("exact second difference bound: %v", err)
	}
	points[2].Position[0] = 81 / interpolationScale
	interpolationRejected(t, interpolationPrepare(t, c, 8202, points), 8202)
	points[2].Time, points[2].Position[0] = 3*time.Second, 180/interpolationScale
	if err := interpolationPrepare(t, c, 8203, points); err != nil {
		t.Fatalf("nonuniform second difference bound: %v", err)
	}
	points[2].Position[0] = 183 / interpolationScale
	interpolationRejected(t, interpolationPrepare(t, c, 8204, points), 8204)
}

func TestDescribeDeclaresInterpolationAndChecks(t *testing.T) {
	c, _ := interpolationCore(t, true, true)
	description, err := c.Describe()
	if err != nil {
		t.Fatal(err)
	}
	if description.ProtocolMajor != 1 || description.ProtocolMinor != 14 || len(description.Axes) != 2 {
		t.Fatalf("description: %+v", description)
	}
	for i, axis := range description.Axes {
		checks := interpolationChecks(t, axis.Checks)
		if axis.Interpolation != "hermite_position_with_velocity, linear_without" || axis.Feedforward != "qd_optional" || axis.Endpoint != "hold_last_sample_then_settle" || checks["position"] != "declared" || checks["velocity"] != "declared" || checks["jerk"] != "unsupported" {
			t.Fatalf("axis %d capabilities: %+v", i, axis)
		}
		if axis.CompletionTolerance != 1/interpolationScale || axis.CompletionTimeoutNS != 500_000_000 || axis.FollowingError != 0.04000000000000001 || axis.FollowingErrorTimeoutNS != 100_000_000 {
			t.Fatalf("axis %d tolerances: %+v", i, axis)
		}
	}
	if interpolationChecks(t, description.Axes[0].Checks)["acceleration"] != "declared" || description.Axes[0].MaxAcceleration == nil || *description.Axes[0].MaxAcceleration != interpolationLimit {
		t.Fatal("missing declared acceleration bound")
	}
	if interpolationChecks(t, description.Axes[1].Checks)["acceleration"] != "not_declared" || description.Axes[1].MaxAcceleration != nil {
		t.Fatal("omitted bound claimed as checked")
	}
	controller, err := control.New(c, control.Binding{PairID: "interpolation", Revision: 1, ConfigurationSHA256: description.ConfigurationSHA256})
	if err != nil {
		t.Fatal(err)
	}
	defer controller.Close()
	response := httptest.NewRecorder()
	control.NewHandler(controller).ServeHTTP(response, httptest.NewRequest(http.MethodGet, "/v1/describe", nil))
	var envelope struct {
		Data control.Description `json:"data"`
	}
	if response.Code != http.StatusOK || json.Unmarshal(response.Body.Bytes(), &envelope) != nil || !reflect.DeepEqual(envelope.Data.Axes, description.Axes) {
		t.Fatalf("HTTP describe lost native declarations: %s", response.Body.String())
	}
}

func interpolationChecks(t *testing.T, raw json.RawMessage) map[string]string {
	t.Helper()
	var checks map[string]string
	if err := json.Unmarshal(raw, &checks); err != nil || len(checks) != 4 {
		t.Fatalf("invalid derivative declaration: %s: %v", raw, err)
	}
	return checks
}

func TestPrepareAcceptsUndeclaredAcceleration(t *testing.T) {
	c, _ := interpolationCore(t, true)
	points := []ipcclient.Point{
		{Position: []float64{0, 0}, Velocity: []float64{0, 0}},
		{Time: time.Second, Position: []float64{0, 0}, Velocity: []float64{0, 41 / interpolationScale}},
	}
	if err := interpolationPrepare(t, c, 8301, points); err != nil {
		t.Fatalf("explicitly undeclared axis: %v", err)
	}
}

// Zero knot velocities hide an interior speed peak from point and sampled
// acceleration checks; Commit must refuse it without poisoning a valid successor.
func TestPrepareRejectsHermiteVelocityExtremum(t *testing.T) {
	c, logPath := interpolationCore(t, true)
	points := []ipcclient.Point{
		{Position: []float64{0, 0}, Velocity: []float64{0, 0}},
		{Time: time.Second, Position: []float64{270 / interpolationScale, 0}, Velocity: []float64{0, 0}},
	}
	if err := interpolationPrepare(t, c, 8401, points); err != nil {
		t.Fatalf("405 counts/s peak below 409.6 counts/s: %v", err)
	}
	points[1].Position[0] = 300 / interpolationScale
	interpolationRejected(t, interpolationPrepare(t, c, 8402, points), 8402)
	raw, err := os.ReadFile(logPath)
	if err != nil || !strings.Contains(string(raw), "hermite_velocity_extremum axis=0 (axis_0) segment=0 peak_counts_per_s=450") {
		t.Fatalf("missing typed peak diagnostic: %v: %s", err, raw)
	}
	points[1].Position = []float64{0, 300 / interpolationScale}
	interpolationRejected(t, interpolationPrepare(t, c, 8403, points), 8403)
	points[1].Position[1] = 270 / interpolationScale
	if err := interpolationPrepare(t, c, 8404, points); err != nil {
		t.Fatalf("valid successor on undeclared acceleration axis: %v", err)
	}
}

// A present but unusable bound must never become an implicit opt-out, in either
// public compiler or the daemon's private profile parser.
func TestInterpolationRejectsMalformedAccelerationProfiles(t *testing.T) {
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}
	dir, err := os.MkdirTemp(filepath.Join(root, "build"), "interpolation-invalid-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	machine := interpolationJSON(t, filepath.Join(root, "config/machines/simulation/simulation.json"))
	machine["axes"] = machine["axes"].([]any)[:1]
	profile := interpolationJSON(t, filepath.Join(root, "config/drives/cia402_sim.json"))
	machineJSON, err := json.Marshal(machine)
	if err != nil {
		t.Fatal(err)
	}
	machinePath := filepath.Join(dir, "machine.json")
	if err := os.WriteFile(machinePath, machineJSON, 0600); err != nil {
		t.Fatal(err)
	}
	writeProfile := func(value any) {
		t.Helper()
		profile["max_acceleration"] = value
		raw, err := json.Marshal(profile)
		if err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(filepath.Join(dir, "cia402_sim.json"), raw, 0600); err != nil {
			t.Fatal(err)
		}
	}
	writeProfile(1)
	valid, err := simfixture.CompileRaw(machineJSON, dir, rtctl.Compile)
	if err != nil {
		t.Fatal(err)
	}
	for _, invalid := range []any{0, -1, nil, "1", true} {
		t.Run(fmt.Sprintf("compiler_%v", invalid), func(t *testing.T) {
			writeProfile(invalid)
			if _, err := simfixture.CompileRaw(machineJSON, dir, rtctl.Compile); err == nil || !strings.Contains(err.Error(), "max_acceleration") {
				t.Fatalf("Go accepted malformed acceleration %v: %v", invalid, err)
			}
			if _, _, err := simfixture.CompileProfiles(root, machinePath, dir, dir); err == nil || !strings.Contains(err.Error(), "max_acceleration") {
				t.Fatalf("file compiler accepted malformed acceleration %v: %v", invalid, err)
			}
		})
	}
	if strings.Count(valid.AxisProfiles, "max_acceleration=1.0\n") != 1 {
		t.Fatal("missing native acceleration fixture")
	}
	profilePath := filepath.Join(dir, "axes.conf")
	binary := simfixture.Locate(root, "rosie-rt-core-sim")
	for _, value := range []string{"0", "-1", "nan", "inf", "", "1junk"} {
		t.Run("native_"+value, func(t *testing.T) {
			raw := strings.Replace(valid.AxisProfiles, "max_acceleration=1.0\n", "max_acceleration="+value+"\n", 1)
			if err := os.WriteFile(profilePath, []byte(raw), 0600); err != nil {
				t.Fatal(err)
			}
			args := append(append([]string(nil), valid.Argv...), "--axis-profiles", profilePath, "--configuration-hash", valid.SHA256, "--validate-config")
			output, err := exec.Command(binary, args...).CombinedOutput()
			var exit *exec.ExitError
			if !errors.As(err, &exit) || exit.ExitCode() != 2 {
				t.Fatalf("native accepted malformed bound %q: %v: %s", value, err, output)
			}
		})
	}
}
