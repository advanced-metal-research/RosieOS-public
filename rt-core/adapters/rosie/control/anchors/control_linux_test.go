//go:build linux

package anchors_test

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"hash/crc32"
	"net/http"
	"net/http/httptest"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"strings"
	"sync"
	"syscall"
	"testing"
	"time"

	"rosieos/robotdesc"
	"rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/adapters/rosie/control/anchors"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/tools/rtctl"
)

// Exercise HM35 anchor capture and the public restoration dispatch against the
// same Go-compiled configuration across two daemon incarnations.
func TestVerifiedDeltaV3PowerCycleRestore(t *testing.T) {
	for _, trial := range []struct {
		name  string
		delta int32
	}{{"positive", 17}, {"negative", -8}, {"wrapped_seam", -123}, {"overcap", 9103}, {"mismatch", 123}, {"opposite", 123}} {
		t.Run(trial.name, func(t *testing.T) { testControlAnchorRestartAxes(t, 1, false, "legacy_delta_"+trial.name, trial.delta) })
	}
}

// A cabinet power cycle clears A6 statusword bit 15 by firmware design (vendor
// reply 2026-04-15, email 4 Q1); each fresh simulator process models it. With
// native Home refresh enabled the restarted daemon must recognise the unchanged
// v3 HM35 anchor at a pose far beyond the explicit restore caps, with no
// restore command, Home, enable, arm or anchor write, and must refuse with a
// named reason when a direct check or the profile policy fails. A v5 file left
// by the removed current-pose prototype is refused, never restored.
func TestAutomaticRetainedHomeAfterPowerCycle(t *testing.T) {
	// 400000 counts = 0.19 rad (11 degrees) at W=13107200, inside +/-0.5 rad;
	// 1200000 counts = 0.58 rad, outside the fixture's hard limit.
	for _, trial := range []struct {
		name, environment string
		delta             int32
	}{
		{"v3", "auto_v3", 400000},
		{"v5_refused", "auto_v5_refused", 400000},
		{"battery_fault", "auto_battery_fault", 400000},
		{"alarm_unreadable", "auto_alarm_unreadable", 400000},
		{"hard_limit", "auto_hard_limit", 1200000},
	} {
		t.Run(trial.name, func(t *testing.T) { testControlAnchorRestartAxes(t, 1, false, trial.environment, trial.delta) })
	}
}

func verifyAutomaticRetainedHome(t *testing.T, environment string, client *ipcclient.Client, daemonLog string, saved anchors.Record, delta int32) {
	t.Helper()
	refusals := map[string]string{"auto_battery_fault": "encoder_alarm",
		"auto_alarm_unreadable": "encoder_alarm_unread", "auto_hard_limit": "hard_limit",
		"auto_v5_refused": "current_reference_anchor_rejected"}
	if reason, refused := refusals[environment]; refused {
		want := "retained_home_refused axis=0 reason=" + reason
		simfixture.PollTest(t, 8*time.Second, 20*time.Millisecond, func() bool {
			raw, _ := os.ReadFile(daemonLog)
			return strings.Contains(string(raw), want)
		})
		s, err := client.Status()
		if err != nil || s.HomeValidMask != 0 || s.Axes[0].RestoredAnchorHomeEpoch != 0 || s.Armed != 0 || s.AxisEnableMask != 0 || s.NativeHomeActiveMask != 0 {
			t.Fatalf("refused automatic restore changed state: %+v %v", s, err)
		}
		return
	}
	var s ipcclient.Status
	simfixture.PollTest(t, 8*time.Second, 10*time.Millisecond, func() bool {
		var err error
		s, err = client.Status()
		return err == nil && s.HomeValidMask == 1
	})
	a := s.Axes[0]
	if s.Armed != 0 || s.AxisEnableMask != 0 || s.CommissioningPhase != 0 || s.NativeHomeActiveMask != 0 || s.HomeEpoch != 1 ||
		a.Statusword&0x8000 != 0 || a.CoordinateValid == 0 || a.RestoredAnchorHomeEpoch != saved.HomeEpoch {
		t.Fatalf("automatic restore state: %+v", s)
	}
}

func TestControlCapturesHomeAndRestoresAnchor(t *testing.T) { testControlAnchorRestart(t, 0) }

func TestControlPresetSwitchKeepsAnchors(t *testing.T) {
	testControlAnchorRestartAxes(t, 1, false, "preset_switch", 0)
}

func TestControlPairMismatchRefusesAnchor(t *testing.T) {
	for _, field := range []string{"pair_id", "pair_revision"} {
		t.Run(field, func(t *testing.T) { testControlAnchorRestartAxes(t, 1, false, field, 0) })
	}
}

func TestNineAxisHomeCapturesAndRestoresAnchors(t *testing.T) {
	testControlAnchorRestartAxes(t, 9, false, "shared", 0)
}

// Cancel the real HTTP transport only after native Home is observed active.
// Its grant-owned worker must finish, persist and admit another Home without restart.
func TestCancelledHTTPHomeAdmitsNextHome(t *testing.T) {
	testControlAnchorRestartAxes(t, 9, true, "shared", 0)
}

func TestHomeAnchorEnvironment(t *testing.T) {
	for _, environment := range []string{"adapter_only", "daemon_only"} {
		t.Run(environment, func(t *testing.T) {
			testControlAnchorRestartAxes(t, 1, false, environment, 0)
		})
	}
}

func TestControlRefusesAnchorAfterPoweredOffMotion(t *testing.T) { testControlAnchorRestart(t, 17) }

// Independent encoder identity can pass Go qualification while the saved
// command disagrees with PDO modulo W; native must refuse that second frame.
func TestNativeRefusesAnchorCommandFrameDisagreement(t *testing.T) {
	testControlAnchorRestart(t, 0, true)
}

func testControlAnchorRestart(t *testing.T, poweredOffDelta int32, nativeDisagreement ...bool) {
	testControlAnchorRestartAxes(t, 1, false, "shared", poweredOffDelta, nativeDisagreement...)
}

func testControlAnchorRestartAxes(t *testing.T, axisCount int, cancelHTTP bool, environment string, poweredOffDelta int32, nativeDisagreement ...bool) {
	root, err := filepath.Abs("../../../..")
	if err != nil {
		t.Fatal(err)
	}
	dir, err := os.MkdirTemp(filepath.Join(root, "build"), "anchor-control-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	read := func(path string) map[string]any {
		raw, err := os.ReadFile(filepath.Join(root, path))
		if err != nil {
			t.Fatal(err)
		}
		var value map[string]any
		if err := json.Unmarshal(raw, &value); err != nil {
			t.Fatal(err)
		}
		return value
	}
	machine := read("config/machines/simulation/simulation-home.json")
	if _, exists := machine["pair"]; exists {
		t.Fatal("configuration must not contain pair metadata")
	}
	// RevPi A6 100:1 fixture: W=13107200 counts matches the physical-speed policy.
	machine["axes"].([]any)[0].(map[string]any)["wire_revs_per_axis_rev"] = 100
	first := machine["axes"].([]any)[0].(map[string]any)
	first["motor_encoder_counts_per_rev"] = 131072
	first["coordinate_evidence_mode"] = "multi_turn"
	var axes []any
	var axisIDs []string
	for axis := 0; axis < axisCount; axis++ {
		entry := make(map[string]any)
		for key, value := range first {
			entry[key] = value
		}
		id := fmt.Sprintf("axis_%d", axis)
		entry["name"], entry["slave_position"] = id, axis
		axes, axisIDs = append(axes, entry), append(axisIDs, id)
	}
	machine["axes"] = axes
	mask := uint32((1 << axisCount) - 1)
	profile := read("config/drives/cia402_sim_home.json")
	a6 := read("config/drives/a6ec_1b02.json")
	profile["startup_defaults"], profile["startup_schema"] = a6["startup_defaults"], a6["startup_schema"]
	policy := a6["coordinate_evidence"].(map[string]any)
	policy["source"] = "a6ec_sdo"
	delete(policy, "validity_mask")
	delete(policy, "validity_value")
	profile["coordinate_evidence"] = policy
	drive := profile["ethercat"].(map[string]any)
	drive["vendor_id"], drive["product_code"] = 4194304, 1813
	drive["rx_pdo"], drive["tx_pdo"] = 5890, 6914
	surveyed := a6["ethercat"].(map[string]any)
	drive["rx_layout"], drive["tx_layout"] = surveyed["rx_layout"], surveyed["tx_layout"]
	machine["bus_bringup"] = map[string]any{"explicit_pdo_config": false}
	if strings.HasPrefix(environment, "legacy_delta_") {
		axes[0].(map[string]any)["reference_verified_delta_restore"] = map[string]any{"method": "independent_absolute_delta_v1"}
		profile["brake"].(map[string]any)["present"] = true
		profile["brake"].(map[string]any)["hold_displacement_tolerance_counts"] = 16
	}
	profilesDir := dir
	if strings.Contains(environment, "_delta_") {
		delete(machine, "drive_speed_limit_motor_rpm")
		profilesDir = filepath.Join(dir, "rt-core/config/drives")
		if err := os.MkdirAll(profilesDir, 0700); err != nil {
			t.Fatal(err)
		}
		descriptionDir := filepath.Join(dir, "robot_description/robots/fixture")
		if err := os.MkdirAll(descriptionDir, 0700); err != nil {
			t.Fatal(err)
		}
		definition := map[string]any{"schema": "rosie.robot-definition.v1", "robot_id": "fixture", "name": "Fixture", "revision": 1, "revision_source": "synthetic process regression", "defaults": map[string]any{"drive_profile": "cia402_sim_home", "home_policy": "required", "coordinate_evidence_mode": "multi_turn"}, "calibration": map[string]any{"identity": "home_calibration", "note": "Native fixture Home"}}
		joint := map[string]any{"name": "axis_0", "urdf_joint": "J1", "kind": "arm", "gear_ratio": map[string]any{"numerator": 100, "denominator": 1, "source": "synthetic", "numerator_source": "synthetic", "denominator_source": "synthetic"}, "motor_encoder_counts_per_rev": 131072, "motor_encoder_counts_per_rev_source": "synthetic", "encoder_battery": "present", "encoder_battery_source": "synthetic test model", "sign": 1, "sign_source": "synthetic", "cartesian_participates": true}
		definition["joints"] = []any{joint}
		raw, _ := json.Marshal(definition)
		if err := os.WriteFile(filepath.Join(descriptionDir, robotdesc.Definition), raw, 0600); err != nil {
			t.Fatal(err)
		}
		for name, data := range map[string]string{"robot.urdf": `<robot name="fixture"><joint name="J1" type="revolute"><limit lower="-0.5" upper="0.5" velocity="0.1"/></joint></robot>`, "config.json": `{"schema":"rosie.robot-config.v1","model_id":"fixture","planning":{"joints":{"J1":{"velocity_rad_s":0.1,"acceleration_rad_s2":1}}}}`, robotdesc.Manifest: `{"schema":"rosie.robot-manifest.v1","model_id":"fixture","files":["config.json","robot.urdf","rtcore_definition.json"]}`} {
			if err := os.WriteFile(filepath.Join(descriptionDir, name), []byte(data), 0600); err != nil {
				t.Fatal(err)
			}
		}
		identity, err := robotdesc.RobotDescriptionSHA256(descriptionDir)
		if err != nil {
			t.Fatal(err)
		}
		machine["robot"] = map[string]any{"robot_description": map[string]any{"path": "robot_description/robots/fixture", "sha256": identity}}
		prior := axes[0].(map[string]any)
		limits := prior["limits"].(map[string]any)
		delete(limits, "min")
		delete(limits, "max")
		delete(limits, "max_velocity")
		delete(limits, "jog_acceleration")
		cell := map[string]any{"robot_joint": "axis_0", "slave_position": 0, "limits": limits}
		if v, ok := prior["reference_verified_delta_restore"]; ok {
			cell["reference_verified_delta_restore"] = v
		}
		machine["axes"] = []any{cell}
	}
	profileBytes, _ := json.Marshal(profile)
	if err := os.WriteFile(filepath.Join(profilesDir, "cia402_sim_home.json"), profileBytes, 0600); err != nil {
		t.Fatal(err)
	}
	machineBytes, _ := json.Marshal(machine)
	compiled, err := simfixture.CompileRaw(machineBytes, profilesDir, rtctl.Compile)
	if err != nil {
		t.Fatal(err)
	}
	profilePath := filepath.Join(dir, "axis.conf")
	if err := os.WriteFile(profilePath, []byte(compiled.AxisProfiles), 0600); err != nil {
		t.Fatal(err)
	}
	store := anchors.Store{Directory: filepath.Join(dir, "anchors")}
	if err := os.MkdirAll(store.Directory, 0700); err != nil {
		t.Fatal(err)
	}
	t.Setenv("ROSIE_RT_ANCHOR_DIR", store.Directory)
	if environment == "daemon_only" {
		t.Setenv("ROSIE_RT_ANCHOR_DIR", "")
	}
	t.Setenv("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "0")
	binding := control.Binding{PairID: "anchor-test", Revision: 1}
	// Automatic environments enable native Home refresh only for the restarted
	// daemon, so its startup path may restore Home before the first observation.
	automatic, restarted, daemonLog := strings.Contains(environment, "auto"), false, ""
	start := func(delta int32) (*control.Controller, control.Fence, *ipcclient.Client, func()) {
		socketDir, err := os.MkdirTemp("", "ac-")
		if err != nil {
			t.Fatal(err)
		}
		t.Cleanup(func() { _ = os.RemoveAll(socketDir) })
		socket := filepath.Join(socketDir, "ipc.sock")
		log, err := os.CreateTemp(dir, "daemon-*.log")
		if err != nil {
			t.Fatal(err)
		}
		args := append(append([]string(nil), compiled.Argv...), "--configuration-hash", compiled.SHA256,
			"--axis-profiles", profilePath, "--socket-path", socket)
		position := delta
		if strings.HasPrefix(environment, "legacy_delta_") {
			position += 16
			args = append(args, "--sim-home-completion-jitter-counts", "16")
		}
		if position != 0 {
			args = append(args, "--sim-initial-position-counts", fmt.Sprint(position))
		}

		if automatic && restarted {
			switch environment {
			case "auto_battery_fault":
				args = append(args, "--sim-encoder-fault-state", "16384") // U41.05 bit 14
			case "auto_alarm_unreadable":
				args = append(args, "--sim-encoder-alarm-unreadable", "1")
			}
		}
		if delta != 0 && (strings.HasSuffix(environment, "mismatch") || strings.HasSuffix(environment, "opposite")) {
			offset := int32(17)
			if strings.HasSuffix(environment, "opposite") {
				offset = -2 * delta
			}
			args = append(args, "--sim-sample-contradiction-from-upload", "1", "--sim-sample-contradiction-counts", fmt.Sprint(offset))
		}
		process := exec.Command(simfixture.Locate(root, "rosie-rt-core-sim"), args...)
		nativeDirectory := store.Directory
		if environment == "adapter_only" {
			nativeDirectory = ""
		}
		process.Env = append(os.Environ(), "ROSIE_RT_ANCHOR_DIR="+nativeDirectory)
		if automatic && restarted {
			process.Env = append(process.Env, "GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED=1")
		}
		daemonLog = log.Name()
		process.Stdout, process.Stderr = log, log
		if err := simfixture.IsolateCommand(process); err != nil {
			t.Fatal(err)
		}
		if err := process.Start(); err != nil {
			t.Fatal(err)
		}
		// Early startup assertions still reap the child before its socket tree
		// is removed; normal shutdown below has already joined Process.Wait.
		t.Cleanup(func() {
			if process.ProcessState == nil {
				_ = process.Process.Kill()
				_ = process.Wait()
			}
			_ = log.Close()
		})
		// Unverified startup observation deadline: 8 seconds; require verified configuration.
		ctx, cancel := context.WithTimeout(context.Background(), 8*time.Second)
		defer cancel()
		var client *ipcclient.Client
		// Unverified observation polling interval: 10 milliseconds.
		ticker := time.NewTicker(10 * time.Millisecond)
		defer ticker.Stop()
		for client == nil {
			raw, _ := os.ReadFile(filepath.Join(socketDir, "metrics.json"))
			var status ipcclient.Status
			ready := json.Unmarshal(raw, &status) == nil && status.ConfigVerifiedMask == mask && len(status.Axes) == axisCount
			for _, axis := range status.Axes {
				proof := axis.IndependentAnchorSource
				ready = ready && proof != nil && proof.Valid && proof.CompletedNS != 0 && proof.MaximumAgeNS != 0 &&
					proof.CompletedNS <= status.TimeNS && status.TimeNS-proof.CompletedNS <= proof.MaximumAgeNS
			}
			if ready {
				for _, axis := range status.Axes {
					if automatic && restarted && status.Armed == 0 && status.AxisEnableMask == 0 {
						continue
					}
					if axis.CoordinateSourceValid != 0 || axis.CoordinateValid != 0 || axis.HomeValid || status.HomeValidMask != 0 || status.Armed != 0 || status.AxisEnableMask != 0 {
						t.Fatal("startup acquired raw evidence must not silently bind coordinates or grant Home", status)
					}
				}
				// Before binding, only independent_anchor_source is usable evidence;
				// sampled_coordinate measures consumed Home/Restore or audit samples.
				client, err = ipcclient.Dial(ctx, ipcclient.Options{SocketPath: socket, ConfigurationSHA256: compiled.SHA256, Backend: "simulation"})
				if err != nil {
					t.Fatal(err)
				}
				break
			}
			select {
			case <-ctx.Done():
				output, _ := os.ReadFile(log.Name())
				t.Logf("startup diagnostics: %s; metrics: %s", output, raw)
				t.Fatal("configuration or independent raw acquisition not ready")
			case <-ticker.C:
			}
		}
		binding.ConfigurationSHA256 = compiled.SHA256
		controller, err := control.New(client, binding)
		if err != nil {
			t.Fatal(err)
		}
		// The adapter's public JSON must carry the fresh independent observation
		// before Home/Restore without misrepresenting it as usable coordinates.
		var public struct {
			Data control.ProcessStatus `json:"data"`
		}
		simfixture.PollTest(t, 3*time.Second, 10*time.Millisecond, func() bool {
			response := httptest.NewRecorder()
			control.NewHandler(controller).ServeHTTP(response, httptest.NewRequest("GET", "/v1/status", nil))
			if response.Code != http.StatusOK || json.Unmarshal(response.Body.Bytes(), &public) != nil || len(public.Data.Core.Axes) != axisCount {
				return false
			}
			for _, axis := range public.Data.Core.Axes {
				if axis.IndependentAnchorSource == nil || !axis.IndependentAnchorSource.Valid {
					return false
				}
				if (axis.CoordinateValid != 0 || axis.CoordinateSourceValid != 0 || axis.HomeValid) && !(automatic && restarted) {
					t.Fatal("public raw observation granted coordinate/Home validity", axis)
				}
			}
			return true
		})
		grant, err := controller.Acquire("anchor-test", binding)
		if err != nil {
			t.Fatal(err)
		}
		fence := control.Fence{Session: grant.Session, Generation: grant.Generation}
		done, renewed := make(chan struct{}), make(chan struct{})
		go func() {
			defer close(renewed)
			// Native grants expire at 250 milliseconds; renew every 50 milliseconds.
			ticks := time.NewTicker(50 * time.Millisecond)
			defer ticks.Stop()
			for {
				select {
				case <-done:
					return
				case <-ticks.C:
					if _, err := controller.Renew(fence); err != nil {
						return
					}
				}
			}
		}()
		var once sync.Once
		stop := func() {
			once.Do(func() {
				close(done)
				<-renewed
				controller.Close()
				_ = client.Close()
				_ = process.Process.Signal(syscall.SIGTERM)
				ended := make(chan error, 1)
				go func() { ended <- process.Wait() }()
				select {
				case <-ended:
				// Unverified shutdown deadline: 3 seconds; completion requires Process.Wait.
				case <-time.After(3 * time.Second):
					_ = process.Process.Kill()
					<-ended
					t.Error("daemon shutdown")
				}
				_ = log.Close()
				if t.Failed() {
					raw, _ := os.ReadFile(log.Name())
					t.Log(string(raw))
				}
			})
		}
		t.Cleanup(stop)
		return controller, fence, client, stop
	}
	c, fence, client, stop := start(0)
	expectedEpoch := uint64(2)
	if cancelHTTP {
		server := httptest.NewServer(control.NewHandler(c))
		t.Cleanup(server.Close)
		// Unverified observation watchdog: 30000000000 ns; require native activity and completion.
		ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
		defer cancel()
		requestCtx, cancelRequest := context.WithCancel(ctx)
		defer cancelRequest()
		body, _ := json.Marshal(control.Request{Schema: control.RequestSchema, Operation: control.OperationHome, Fence: fence, AxisMask: mask})
		request, err := http.NewRequestWithContext(requestCtx, "POST", server.URL+"/v1/control", bytes.NewReader(body))
		if err != nil {
			t.Fatal(err)
		}
		finished := make(chan error, 1)
		go func() {
			response, err := server.Client().Do(request)
			if response != nil {
				_ = response.Body.Close()
			}
			finished <- err
		}()
		if _, err = client.Wait(ctx, func(s ipcclient.Status) bool {
			return s.CommissioningPhase == 1 && s.NativeHomeActiveMask != 0 && s.HomeEpoch == 2
		}); err != nil {
			t.Fatal(err)
		}
		cancelRequest()
		if err = <-finished; !errors.Is(err, context.Canceled) {
			t.Fatalf("HTTP cancellation: %v", err)
		}
		if _, err = client.Wait(ctx, func(s ipcclient.Status) bool {
			return s.CommissioningPhase == 0 && s.HomeValidMask == mask && s.HomeEpoch == 2
		}); err != nil {
			t.Fatal(err)
		}
		expectedEpoch = 3
	}
	// An empty configured store must never turn ordinary Home into restore.
	if _, err := c.Home(fence, mask); err != nil {
		t.Fatal(err)
	}
	expectedAnchor := int32(0)
	if strings.HasPrefix(environment, "legacy_delta_") {
		expectedAnchor = 16
	}

	if environment == "daemon_only" {
		if _, err := store.Load(0); !errors.Is(err, anchors.ErrMissing) {
			t.Fatalf("daemon must not persist Home: %v", err)
		}
		return
	}
	saved, err := store.Load(0)
	if err != nil || saved.HomeEpoch != expectedEpoch || saved.AnchorCounts != expectedAnchor || saved.PairID != binding.PairID || saved.PairRevision != binding.Revision {
		t.Fatalf("capture: %+v %v", saved, err)
	}
	for axis := 0; axis < axisCount; axis++ {
		record, err := store.Load(uint32(axis))
		if err != nil || record.Axis != uint32(axis) || record.HomeEpoch != expectedEpoch || record.AnchorCounts != expectedAnchor {
			t.Fatalf("axis %d capture: %+v %v", axis, record, err)
		}
	}
	files, err := filepath.Glob(filepath.Join(store.Directory, "axis-*.anchor"))
	if err != nil || len(files) != axisCount {
		t.Fatalf("anchor file count: %d %v", len(files), err)
	}
	stop()
	if environment == "preset_switch" {
		previous := compiled
		rich := read("config/drives/a6ec_bench_1to1.json")["ethercat"].(map[string]any)
		drive["tx_pdo"], drive["tx_layout"] = rich["tx_pdo"], rich["tx_layout"]
		// Unverified stationary fixture: 20 motor rpm cap and 1 rad upper limit.
		machine["drive_speed_limit_motor_rpm"] = 20
		first["limits"].(map[string]any)["max"] = 1
		profileBytes, _ = json.Marshal(profile)
		if err := os.WriteFile(filepath.Join(profilesDir, "cia402_sim_home.json"), profileBytes, 0600); err != nil {
			t.Fatal(err)
		}
		machineBytes, _ = json.Marshal(machine)
		compiled, err = simfixture.CompileRaw(machineBytes, profilesDir, rtctl.Compile)
		if err != nil {
			t.Fatal(err)
		}
		if previous.SHA256 == compiled.SHA256 || previous.CoordinateIdentitySHA256[0] != compiled.CoordinateIdentitySHA256[0] {
			t.Fatal("preset identity partition")
		}
		if err := os.WriteFile(profilePath, []byte(compiled.AxisProfiles), 0600); err != nil {
			t.Fatal(err)
		}
	}
	if len(nativeDisagreement) != 0 && nativeDisagreement[0] {
		current := saved
		// Unverified refusal fixture: 17 counts exceeds the profile's 16.
		saved.AnchorCounts += 17
		if err := saved.Qualify(current, true); err != nil {
			t.Fatalf("native fixture refused by Go qualification: %v", err)
		}
		if err := store.Save(saved); err != nil {
			t.Fatal(err)
		}
	}
	if environment == "pair_id" {
		binding.PairID = "replacement"
	}
	if environment == "pair_revision" {
		binding.Revision = 2
	}
	anchorBytes, err := os.ReadFile(store.Path(0))
	if err != nil {
		t.Fatal(err)
	}
	if environment == "auto_v5_refused" {
		// Same fields under the v5 magic left by the removed current-pose
		// prototype, with a valid checksum so only the version is refused.
		payload := append([]byte("rosie-anchor-v5 "), bytes.TrimPrefix(bytes.SplitN(anchorBytes, []byte{'\n'}, 2)[0], []byte("rosie-anchor-v3 "))...)
		payload = append(payload, '\n')
		anchorBytes = fmt.Appendf(payload, "%08x\n", crc32.ChecksumIEEE(payload))
		if err := os.WriteFile(store.Path(0), anchorBytes, 0600); err != nil {
			t.Fatal(err)
		}
		if _, err := store.Load(0); !errors.Is(err, anchors.ErrCurrentReference) {
			t.Fatalf("adapter accepted v5 anchor: %v", err)
		}
	}
	restarted = true
	c, fence, client, _ = start(poweredOffDelta)
	if automatic {
		verifyAutomaticRetainedHome(t, environment, client, daemonLog, saved, poweredOffDelta)
		if after, err := os.ReadFile(store.Path(0)); err != nil || !bytes.Equal(after, anchorBytes) {
			t.Fatalf("automatic restore changed anchor bytes: %v", err)
		}
		return
	}
	skipOrdinaryRestore := strings.HasPrefix(environment, "legacy_delta_") && poweredOffDelta >= -16 && poweredOffDelta <= 16
	if strings.HasPrefix(environment, "legacy_delta_") {
		startup, e := client.Status()
		if e != nil || startup.HomeValidMask != 0 || startup.Axes[0].Statusword&0x9000 != 0 {
			t.Fatalf("power cycle did not clear retained Home: %+v %v", startup, e)
		}
	}

	body, _ := json.Marshal(control.Request{Schema: "rosie.rt-control.request.v1", Operation: "restore_anchor", Fence: fence, AxisMask: mask})
	request := httptest.NewRequest("POST", "/v1/control", bytes.NewReader(body))
	response := httptest.NewRecorder()
	if !skipOrdinaryRestore {
		control.NewHandler(c).ServeHTTP(response, request)
	}
	if environment == "pair_id" || environment == "pair_revision" {
		var envelope control.RawResponse
		if err := json.Unmarshal(response.Body.Bytes(), &envelope); err != nil {
			t.Fatal(err)
		}
		state, err := client.Status()
		if err != nil || response.Code != http.StatusConflict || envelope.Error != control.ReasonAnchorIdentityMismatch || envelope.Sequence != 0 || state.HomeValidMask != 0 || state.Armed != 0 || state.AxisEnableMask != 0 {
			t.Fatalf("pair mismatch refusal: %d %s status=%+v err=%v", response.Code, response.Body.String(), state, err)
		}
		var public struct {
			Data control.ProcessStatus `json:"data"`
		}
		// Unverified observation bound: 3 seconds; startup motion publication may
		// still be stale, so require a successful public snapshot before inspection.
		simfixture.PollTest(t, 3*time.Second, time.Millisecond, func() bool {
			response = httptest.NewRecorder()
			control.NewHandler(c).ServeHTTP(response, httptest.NewRequest("GET", "/v1/status", nil))
			return response.Code == http.StatusOK && json.Unmarshal(response.Body.Bytes(), &public) == nil && len(public.Data.Core.Axes) == axisCount
		})
		field := "pair.id"
		if environment == "pair_revision" {
			field = "pair.revision"
		}
		a := public.Data.Core.Axes[0]
		if a.AnchorReason != control.ReasonAnchorIdentityMismatch || a.AnchorIdentityChangedField != field || a.CoordinateReason != 3 {
			t.Fatalf("public pair refusal missing: %+v", a)
		}
		return
	}
	if environment == "adapter_only" {
		var envelope control.RawResponse
		if err := json.Unmarshal(response.Body.Bytes(), &envelope); err != nil {
			t.Fatal(err)
		}
		if response.Code != http.StatusConflict || envelope.Error != control.ReasonAnchorMissing || envelope.Sequence != 0 {
			t.Fatalf("missing native anchor offer: %d %s", response.Code, response.Body.String())
		}
		return
	}
	if len(nativeDisagreement) != 0 && nativeDisagreement[0] {
		if response.Code != 409 || !bytes.Contains(response.Body.Bytes(), []byte("anchor_disagrees")) {
			t.Fatalf("native PDO disagreement refusal: %d %s", response.Code, response.Body.String())
		}
		// The raw-source offer now checks the command frame before reservation.
		// Native repeats the same check on completion; a rejected offer must not
		// bind coordinates, advance Home or enable any output.
		s, err := client.Status()
		if err != nil || s.HomeEpoch != 1 || s.CommissioningPhase != 0 || s.HomeValidMask != 0 ||
			s.Armed != 0 || s.AxisEnableMask != 0 || s.Axes[0].CoordinateValid != 0 || s.Axes[0].RestoredAnchorHomeEpoch != 0 {
			t.Fatalf("native refusal missing or Home restored: %+v %v", s, err)
		}
		return
	}
	if strings.HasPrefix(environment, "legacy_delta_") {
		// The in-frame negative case intentionally bypasses normal RestoreAnchor:
		// its <=16-count pose is also admissible by that unchanged legacy route.
		if !skipOrdinaryRestore && response.Code == 200 {
			t.Fatal("normal same-pose restore silently broadened")
		}
		original, e := os.ReadFile(store.Path(0))
		if e != nil || !bytes.HasPrefix(original, []byte("rosie-anchor-v3 ")) {
			t.Fatal("missing genuine v3", e)
		}
		body, _ = json.Marshal(control.Request{Schema: control.RequestSchema, Operation: control.OperationRestoreReferenceVerifiedDelta, Fence: fence, AxisMask: mask})
		response = httptest.NewRecorder()
		control.NewHandler(c).ServeHTTP(response, httptest.NewRequest("POST", "/v1/control", bytes.NewReader(body)))
		final, e := client.Status()
		if e != nil {
			t.Fatal(e)
		}
		// No delta cap: any agreeing displacement inside travel restores,
		// including a seam crossing; only encoder/PDO disagreement refuses.
		expect := !strings.HasSuffix(environment, "mismatch") && !strings.HasSuffix(environment, "opposite")
		if expect {
			var env control.RawResponse
			var proof control.VerifiedDeltaRestoreReceipt
			if response.Code != 200 || json.Unmarshal(response.Body.Bytes(), &env) != nil || json.Unmarshal(env.Data, &proof) != nil || proof.AnchorDeltaRestore == nil {
				t.Fatalf("v3 restore %d %s", response.Code, response.Body.String())
			}
			v := proof.AnchorDeltaRestore
			if !v.Valid || v.AnchorVersion != 3 || v.BoundRestoreCommandSequence != env.Sequence || v.SavedCommandCounts != saved.AnchorCounts || v.SavedEncoderCounts != saved.TurnsCounts || v.SavedLogicalOffsetCounts != saved.HomeOffset || v.SavedLogicalOffsetCounts == -saved.AnchorCounts || v.BoundDeltaCommandCounts != int64(poweredOffDelta) || v.BoundDeltaEncoderCounts != int64(poweredOffDelta) || final.HomeValidMask != mask {
				t.Fatalf("wrong v3 provenance %+v", proof)
			}
			// First real public Enable/Arm must hold the present command, not recapture/return to C0.
			if _, e = c.Enable(fence, mask); e != nil {
				t.Fatal(e)
			}
			if _, e = c.Arm(fence); e != nil {
				t.Fatal(e)
			}
			ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
			held, e := client.Wait(ctx, func(x ipcclient.Status) bool { return x.Armed != 0 && x.Axes[0].Statusword&0x6f == 0x27 })
			cancel()
			// Raw PDO counts wrap at the seam; the unwrapped coordinate must hold.
			if e != nil || held.Axes[0].CoordinateCounts != int64(16+poweredOffDelta) || held.Axes[0].LogicalTarget != held.Axes[0].LogicalPosition {
				t.Fatalf("first Arm jumped: %+v %v", held, e)
			}
			if _, e = c.Stop(fence, false); e != nil {
				t.Fatal(e)
			}
		} else if response.Code == 200 || final.HomeValidMask != 0 {
			t.Fatalf("bad v3 delta accepted: %d %s", response.Code, response.Body.String())
		}
		after, e := os.ReadFile(store.Path(0))
		if e != nil || !bytes.Equal(original, after) {
			t.Fatal("v3 anchor rewritten")
		}
		return
	}
	if poweredOffDelta != 0 {
		if response.Code == 200 || !bytes.Contains(response.Body.Bytes(), []byte("anchor_disagrees")) {
			t.Fatalf("powered-off displacement refusal: %d %s", response.Code, response.Body.String())
		}
		return
	}
	if response.Code != 200 {
		t.Fatalf("restore dispatch: %d %s", response.Code, response.Body.String())
	}
	var envelope control.RawResponse
	if err := json.Unmarshal(response.Body.Bytes(), &envelope); err != nil {
		t.Fatal(err)
	}
	var recovery control.RecoveryStatus
	if err := json.Unmarshal(envelope.Data, &recovery); err != nil {
		t.Fatal(err)
	}
	if envelope.Schema != control.ResponseSchema || envelope.Operation != control.OperationRestoreAnchor || envelope.Error != "" || envelope.Sequence == 0 ||
		recovery.HomeValidMask != mask || recovery.ResetSequence != 0 || recovery.Reason != "" ||
		recovery.Faults == nil || len(recovery.Faults) != 0 || recovery.Outcomes == nil || len(recovery.Outcomes) != 0 {
		t.Fatalf("restore recovery: envelope=%+v recovery=%+v", envelope, recovery)
	}
	// Unverified status observation deadline: 3 seconds; require restored Home evidence.
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	s, err := client.Wait(ctx, func(s ipcclient.Status) bool { return s.HomeValidMask == mask && s.Axes[0].LogicalValid })
	if err != nil || s.Axes[0].RestoredAnchorHomeEpoch != expectedEpoch || s.DaemonIncarnation == saved.DaemonIncarnation {
		t.Fatalf("restored status: %+v %v", s, err)
	}
	if s.Armed != 0 || s.AxisEnableMask != 0 || s.Axes[0].PositionCounts != expectedAnchor || s.Axes[0].CoordinateCounts != int64(expectedAnchor) {
		t.Fatal("restore moved or enabled outputs", s)
	}
	for axis, state := range s.Axes {
		if !state.LogicalValid || !state.HomeValid || state.RestoredAnchorHomeEpoch != expectedEpoch {
			t.Fatalf("axis %d restored evidence: %+v", axis, state)
		}
	}
}
