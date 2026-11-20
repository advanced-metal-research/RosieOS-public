//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"net"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/internal/simfixture"
)

// TestCollisionNativeAndPublicStatus runs the compiler, strict native loader,
// cyclic fault fence and public forwarding with fixed 201-count drive feedback.
func TestCollisionNativeAndPublicStatus(t *testing.T) {
	testCollisionNativeAndCapture(t, false)
}

// A simulator move and a forced PDO trip must survive the binary dump and CLI
// conversion. Replaying only captured samples must locate the same trip cycle.
func TestCollisionCaptureSimulatorMove(t *testing.T) {
	testCollisionNativeAndCapture(t, true)
}

func testCollisionNativeAndCapture(t *testing.T, move bool) {
	// Unverified simulator trip windows: 3 cycles for the status contract and
	// 1000 cycles for a move with observed target and feedback displacement.
	sustained := 3
	if move {
		sustained = 1000
	}
	root, err := filepath.Abs("..")
	simfixture.Must(t, err)
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "collision-")
	simfixture.Must(t, err)
	t.Cleanup(func() { os.RemoveAll(build) })
	dir, err := os.MkdirTemp("", "cw-")
	simfixture.Must(t, err)
	t.Cleanup(func() { os.RemoveAll(dir) })
	profiles := filepath.Join(build, "config/drives")
	simfixture.Must(t, os.MkdirAll(profiles, 0755))
	other, err := os.ReadFile(filepath.Join(root, "config/drives/cia402_sim_other.json"))
	simfixture.Must(t, err)
	simfixture.Must(t, os.WriteFile(filepath.Join(profiles, "cia402_sim_other.json"), other, 0644))
	raw, err := os.ReadFile(filepath.Join(root, "config/drives/cia402_sim.json"))
	simfixture.Must(t, err)
	var profile map[string]any
	simfixture.Must(t, json.Unmarshal(raw, &profile))
	drive := profile["ethercat"].(map[string]any)
	drive["tx_layout"] = append(drive["tx_layout"].([]any), map[string]any{"semantic": "following_error", "index": 24820, "subindex": 0, "bits": 32})
	drive["tx_layout"] = append(drive["tx_layout"].([]any), map[string]any{"semantic": "torque", "index": 24695, "subindex": 0, "bits": 16})
	drive["tx_layout"] = append(drive["tx_layout"].([]any), map[string]any{"semantic": "velocity_actual", "index": 24684, "subindex": 0, "bits": 32})
	profile["collision_watchdog"] = map[string]any{"torque_abs_max_raw": 1000, "following_error_counts_max": 200, "sustained_cycles": sustained, "_unverified": "unverified synthetic thresholds", "_source": "TestCollisionNativeAndPublicStatus"}
	raw, err = json.Marshal(profile)
	simfixture.Must(t, err)
	simfixture.Must(t, os.WriteFile(filepath.Join(profiles, "cia402_sim.json"), raw, 0644))
	compiled, args, err := simfixture.CompileProfiles(root, filepath.Join(root, "config/machines/simulation/simulation.json"), profiles, filepath.Join(build, "config"))
	simfixture.Must(t, err)
	socket := filepath.Join(dir, "ipc.sock")
	dumpDirectory := filepath.Join(build, "telemetry")
	args = append(args, "--socket-path", socket, "--sim-following-error-counts", "-201", "--telemetry-directory", dumpDirectory)
	process := exec.Command(simfixture.Locate(root, "rosie-rt-core-sim"), args...)
	logPath := filepath.Join(build, "core.log")
	// Unverified observation/shutdown bounds; success requires exact published evidence.
	simfixture.SpawnLogged(t, process, logPath, 3*time.Second, 9000, nil)
	simfixture.WaitVerified(t, filepath.Join(dir, "metrics.json"), 5*time.Second, time.Millisecond, func(s simfixture.Verified) bool { return s.ConfigVerifiedMask == 3 })
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	c, err := Dial(ctx, Options{SocketPath: socket, ConfigurationSHA256: filepath.Base(compiled), Backend: "simulation", SimulationTest: true})
	simfixture.Must(t, err)
	defer c.Close()
	seq, err := c.Enable(ctx, 1)
	mustSubmit(t, seq, err)
	var handle uint64
	if move {
		handle, err = c.PrepareTrajectory(ctx, 3434, 1, []Point{{Position: []float64{0, 0}}, {Time: time.Second, Position: []float64{0.01, 0}}})
		simfixture.Must(t, err)
	}
	seq, err = c.Arm(ctx)
	mustSubmit(t, seq, err)
	if move {
		wait(t, c, func(s Status) bool { return len(s.Axes) == 2 && s.Axes[0].Statusword&0x6f == 0x27 })
		seq, err = c.StartTrajectory(ctx, handle)
		mustSubmit(t, seq, err)
	}
	s := wait(t, c, func(s Status) bool {
		return s.ExecutionFaultReasons&4096 != 0 && len(s.Axes) == 2 && s.Axes[0].CollisionWatchdog.TripCount == 1
	})
	w := s.Axes[0].CollisionWatchdog
	if s.Armed != 0 || s.AxisEnableMask != 0 || s.SafetyFaultMask != 1 || w.Armed || w.FollowingErrorCycles != uint32(sustained) || w.TripSustainedCycles != uint32(sustained) || w.PeakFollowingErrorCounts != 201 || w.LastTripQuantity != "following_error" || s.Axes[1].CollisionWatchdog.TripCount != 0 {
		t.Fatalf("fault status: %+v", s)
	}
	found := false
	if s.Recovery != nil {
		for _, f := range s.Recovery.Faults {
			if f.Name == "collision_watchdog" && f.Bit == 12 && f.AxisMask == 1 {
				found = true
			}
		}
	}
	if !found {
		t.Fatalf("missing typed reason: %+v", s.Recovery)
	}
	newer := wait(t, c, func(next Status) bool { return next.TimeNS > s.TimeNS })
	if newer.ExecutionFaultReasons&4096 == 0 || newer.Axes[0].CollisionWatchdog.TripCount != 1 {
		t.Fatal("automatic reset or repeated trip")
	}
	if move {
		var dump string
		simfixture.PollTest(t, 5*time.Second, time.Millisecond, func() bool {
			files, err := filepath.Glob(filepath.Join(dumpDirectory, "*.bin"))
			simfixture.Must(t, err)
			if len(files) == 0 {
				return false
			}
			dump = files[0]
			return true
		})
		var samples uint64
		var torqueCycles, errorCycles uint32
		var replayTrip, capturedTrip uint64
		var movedTarget, movedPosition bool
		_, err := ReadTelemetryDump(dump, func(r CycleCaptureRecordV2, _ TelemetryDumpIndex) error {
			a := r.Ax[0]
			if r.ActiveTrajId == handle && r.ActiveMode == MOTION_MODE_TRAJECTORY {
				movedTarget = movedTarget || a.Target >= 100
				movedPosition = movedPosition || a.Position >= 100
			}
			if a.CollisionSamples > samples {
				if a.CollisionSamples != samples+1 || a.TorqueValid != 1 || a.FollowingErrorValid != 1 || a.VelocityActualValid != 1 {
					t.Fatalf("missing captured watchdog sample: %+v", a)
				}
				samples = a.CollisionSamples
				if a.TorqueRaw > 1000 || a.TorqueRaw < -1000 {
					torqueCycles++
				} else {
					torqueCycles = 0
				}
				if a.FollowingErrorCounts > 200 || a.FollowingErrorCounts < -200 {
					errorCycles++
				} else {
					errorCycles = 0
				}
				if a.CollisionTorqueCycles != torqueCycles || a.CollisionFollowingErrorCycles != errorCycles {
					t.Fatal("capture counters differ from replay")
				}
				if errorCycles == 1000 {
					replayTrip = r.Cycle
				}
			}
			if capturedTrip == 0 && a.CollisionTripCount == 1 {
				capturedTrip = r.Cycle
				if a.CollisionLastTripQuantity != 2 || a.CollisionPeakFollowingErrorCounts != 201 || r.ExecutionFaultReasons != 4096 || r.SafetyFaultMask != 1 || r.Armed != 0 || r.AxisEnableMask != 0 || r.GrantActive != 0 {
					t.Fatalf("captured trip: %+v", r)
				}
			}
			return nil
		})
		simfixture.Must(t, err)
		if replayTrip == 0 || replayTrip != capturedTrip || !movedTarget || !movedPosition {
			t.Fatalf("replay=%d captured=%d target_move=%t feedback_move=%t", replayTrip, capturedTrip, movedTarget, movedPosition)
		}
		jsonl, err := exec.Command(simfixture.Locate(root, "rtctl"), "telemetry", "dump-to-jsonl", dump).CombinedOutput()
		simfixture.Must(t, err)
		for _, line := range strings.Split(strings.TrimSpace(string(jsonl)), "\n") {
			var row map[string]any
			simfixture.Must(t, json.Unmarshal([]byte(line), &row))
			if row["cycle"] == float64(capturedTrip) {
				t.Logf("dump-to-jsonl trip: %s", line)
			}
		}
		t.Logf("dump replay cycle=%d quantity=following_error sustained_cycles=1000; target and feedback each moved at least 100 counts", replayTrip)
	}
	simfixture.Must(t, c.Close())
	controlSocket := filepath.Join(dir, "control.sock")
	adapter := simfixture.StartControl(root, socket, controlSocket, filepath.Base(compiled), "collision", 1)
	simfixture.SpawnLogged(t, adapter, filepath.Join(build, "control.log"), 3*time.Second, 9000, nil)
	transport := &http.Transport{DialContext: func(ctx context.Context, _, _ string) (net.Conn, error) {
		return (&net.Dialer{}).DialContext(ctx, "unix", controlSocket)
	}}
	defer transport.CloseIdleConnections()
	httpClient := &http.Client{Transport: transport, Timeout: time.Second}
	simfixture.PollTest(t, 5*time.Second, time.Millisecond, func() bool {
		response, err := httpClient.Get("http://unix/v1/status")
		if err != nil {
			return false
		}
		defer response.Body.Close()
		var envelope struct {
			Data struct{ Axes []LogicalAxisStatus }
		}
		if json.NewDecoder(response.Body).Decode(&envelope) != nil || response.StatusCode != 200 || len(envelope.Data.Axes) != 2 {
			return false
		}
		if envelope.Data.Axes[0].CollisionWatchdog != w {
			t.Fatalf("public trip changed: %+v", envelope.Data.Axes[0])
		}
		return true
	})
	log, err := os.ReadFile(logPath)
	simfixture.Must(t, err)
	if strings.Count(string(log), "collision_watchdog axis=0 quantity=following_error sustained_cycles=") != 1 {
		t.Fatalf("expected one publisher trip log: %s", log)
	}
}
