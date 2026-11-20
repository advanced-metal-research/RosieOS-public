//go:build linux

package ipcclient

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
	"syscall"
	"testing"
	"time"

	"rosieos/rt-core/adapters/rosie/control/anchors"
	"rosieos/rt-core/internal/simfixture"
)

// Unverified observation and shutdown deadline: 3 seconds; success requires a receipt.
const coordinateModelDeadline = 3 * time.Second

// Restart the same compiled machine, preserving only its explicit anchor
// directory. Simulator source state is independent of the saved count frame.
func coordinateModelDaemon(t *testing.T, compiled compiledValidationFixture, digest string, extra ...string) (fixture, func()) {
	t.Helper()
	dir, err := os.MkdirTemp("", "an-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	socket := filepath.Join(dir, "ipc.sock")
	log, err := os.CreateTemp(filepath.Dir(compiled.path), "anchor-daemon-*.log")
	if err != nil {
		t.Fatal(err)
	}
	args := append(append([]string(nil), compiled.args...), "--configuration-hash", digest,
		"--socket-path", socket)
	process := exec.Command(compiled.binary, append(args, extra...)...)
	process.Stdout, process.Stderr = log, log
	if err := simfixture.IsolateCommand(process); err != nil {
		t.Fatal(err)
	}
	if err := process.Start(); err != nil {
		t.Fatal(err)
	}
	f := fixture{options: Options{SocketPath: socket, ConfigurationSHA256: digest, Backend: "simulation", SimulationTest: true}}
	closed := false
	stop := func() {
		if closed {
			return
		}
		closed = true
		if f.client != nil {
			_ = f.client.Close()
		}
		_ = process.Process.Signal(syscall.SIGTERM)
		done := make(chan error, 1)
		go func() { done <- process.Wait() }()
		select {
		case <-done:
		case <-time.After(coordinateModelDeadline):
			tasks, _ := filepath.Glob(fmt.Sprintf("/proc/%d/task/*", process.Process.Pid))
			for _, task := range tasks {
				comm, _ := os.ReadFile(filepath.Join(task, "comm"))
				wait, _ := os.ReadFile(filepath.Join(task, "wchan"))
				call, _ := os.ReadFile(filepath.Join(task, "syscall"))
				t.Logf("shutdown thread %s (%s): wait=%s syscall=%s", filepath.Base(task),
					strings.TrimSpace(string(comm)), strings.TrimSpace(string(wait)), strings.TrimSpace(string(call)))
			}
			_ = process.Process.Kill()
			<-done
			t.Error("anchor daemon did not shut down")
		}
		_ = log.Close()
		if t.Failed() {
			raw, _ := os.ReadFile(log.Name())
			t.Log(string(raw))
		}
	}
	t.Cleanup(stop)
	waitCoordinateObservation(t, f, 1, func(s coordinateObservation) bool { return s.ConfigVerifiedMask == 1 })
	ctx, cancel := context.WithTimeout(context.Background(), coordinateModelDeadline)
	defer cancel()
	f.client, err = Dial(ctx, f.options)
	if err != nil {
		t.Fatal(err)
	}
	return f, stop
}

func coordinateModelRecord(s Status) anchors.Record {
	a := s.Axes[0]
	return anchors.Record{PairID: "identity-test", PairRevision: 1, CoordinateIdentitySHA256: a.CoordinateIdentitySHA256, CoordinateIdentity: a.CoordinateIdentity, Vendor: a.VendorID, Product: a.ProductCode,
		Revision: a.Revision, Serial: a.Serial, SerialValid: a.SerialValid, HomeEpoch: s.HomeEpoch,
		AnchorCounts: int32(a.CoordinateCounts), TurnsCounts: a.AbsoluteSourceCounts, HomeOffset: a.HomeOffset,
		ToleranceCounts:   a.AnchorToleranceCounts,
		DaemonIncarnation: s.DaemonIncarnation, CapturedHostNS: time.Now().UnixNano()}
}

func coordinateModelRestore(t *testing.T, f fixture, accept bool) {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), coordinateModelDeadline)
	defer cancel()
	sequence, err := f.client.RestoreAnchor(ctx, 1)
	mustSubmit(t, sequence, err)
	result, err := f.client.WaitResult(ctx, sequence)
	if accept {
		if err != nil || result.Result != ResultAccepted {
			t.Fatalf("restore: %+v %v", result, err)
		}
	} else {
		var rejected *RejectedError
		if !errors.As(err, &rejected) || result.Result != ResultRejected {
			t.Fatalf("unsafe restore: %+v %v", result, err)
		}
	}
}

func TestCoordinateModelAnchorRestart(t *testing.T) {
	t.Setenv("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "0")
	machine, profile := compiledCoordinateInputs(t)
	compiled, digest, err := compileCoordinateJSON(t, machine, profile)
	if err != nil {
		t.Fatal(err, digest)
	}
	store := anchors.Store{Directory: filepath.Join(filepath.Dir(compiled.path), "anchors")}
	t.Setenv("ROSIE_RT_ANCHOR_DIR", store.Directory)
	f, stop := coordinateModelDaemon(t, compiled, digest)
	before := wait(t, f.client, func(s Status) bool { return s.Axes[0].AnchorReason == "anchor_missing" })
	if before.HomeValidMask != 0 || before.Axes[0].LogicalValid || before.Axes[0].AnchorReason != "anchor_missing" {
		t.Fatalf("startup fabricated Home: %+v", before)
	}
	coordinateHome(t, f, 1)
	homed := wait(t, f.client, func(s Status) bool {
		return s.HomeValidMask == 1 && s.CommissioningPhase == 0 && s.Axes[0].LogicalValid
	})
	if homed.Axes[0].PositionCounts != 0 || homed.Axes[0].LogicalPosition != 0 {
		t.Fatal(homed)
	}
	saved := coordinateModelRecord(homed)
	if err := store.Save(saved); err != nil {
		t.Fatal(err)
	}
	stop()
	f, stop = coordinateModelDaemon(t, compiled, digest)
	restarted := wait(t, f.client, func(s Status) bool { return s.Axes[0].AnchorReason == "anchor_ready" })
	if restarted.HomeValidMask != 0 || restarted.DaemonIncarnation == saved.DaemonIncarnation {
		t.Fatal(restarted)
	}
	coordinateModelRestore(t, f, true)
	wait(t, f.client, func(s Status) bool {
		return s.HomeValidMask == 1 && s.Axes[0].RestoredAnchorHomeEpoch == saved.HomeEpoch && s.Axes[0].LogicalValid
	})
	arm(t, f.client, 1)
	wait(t, f.client, func(s Status) bool { return s.Axes[0].Statusword&0x6f == 0x27 })
	coordinateMove(t, f, 9521, []float64{0}, []float64{-100 * 2 * math.Pi / 131072})
	moved := wait(t, f.client, func(s Status) bool { return s.Axes[0].PositionCounts == -100 })
	if !moved.Axes[0].LogicalValid || math.Abs(moved.Axes[0].LogicalPosition-(-0.004793689962142629)) > 1e-12 {
		t.Fatalf("continuous logical units: %+v", moved.Axes[0])
	}
	// Even a valid anchor cannot restore while motion outputs are armed.
	coordinateModelRestore(t, f, false)
	sequence, err := f.client.Stop(context.Background())
	mustSubmit(t, sequence, err)
	displaced := wait(t, f.client, func(s Status) bool {
		return s.Armed == 0 && s.AxisEnableMask == 0 && s.Axes[0].CoordinateCounts == -100
	})
	displacedAnchor := coordinateModelRecord(displaced)
	stop()

	t.Run("tamper", func(t *testing.T) {
		raw, err := os.ReadFile(store.Path(0))
		if err != nil {
			t.Fatal(err)
		}
		raw[0] ^= 1
		if err := os.WriteFile(store.Path(0), raw, 0600); err != nil {
			t.Fatal(err)
		}
		defer func() {
			if err := store.Save(saved); err != nil {
				t.Error(err)
			}
		}()
		f, _ := coordinateModelDaemon(t, compiled, digest)
		wait(t, f.client, func(s Status) bool { return s.Axes[0].AnchorReason == "anchor_source_invalid" })
		coordinateModelRestore(t, f, false)
		if s, _ := f.client.Status(); s.HomeValidMask != 0 {
			t.Fatal(s)
		}
	})
	t.Run("configuration replacement", func(t *testing.T) {
		f, _ := coordinateModelDaemon(t, compiled, strings.Repeat("e", 64))
		wait(t, f.client, func(s Status) bool { return s.Axes[0].AnchorReason == "anchor_ready" })
		coordinateModelRestore(t, f, true)
	})
	t.Run("observed source disagrees", func(t *testing.T) {
		// Reuse the independently observed negative seam crossing above. The
		// restarted simulator reports zero, 100 counts from this qualified anchor.
		if err := store.Save(displacedAnchor); err != nil {
			t.Fatal(err)
		}
		f, _ := coordinateModelDaemon(t, compiled, digest)
		wait(t, f.client, func(s Status) bool { return s.Axes[0].AnchorReason == "anchor_disagrees" })
		coordinateModelRestore(t, f, false)
	})
}

func TestCoordinateModelMetricsAdditiveDecoding(t *testing.T) {
	var s Status
	if err := json.Unmarshal([]byte(`{"axes":[{"pos_counts":130972,"logical_position":-0.004793689962142629,"logical_valid":true}]}`), &s); err != nil {
		t.Fatal(err)
	}
	if s.Axes[0].PositionCounts != 130972 || !s.Axes[0].LogicalValid || s.Axes[0].LogicalPosition != -0.004793689962142629 {
		t.Fatal(s)
	}
	if err := json.Unmarshal([]byte(`{"axes":[{"pos_counts":42}]}`), &s); err != nil {
		t.Fatal(err)
	}
}
