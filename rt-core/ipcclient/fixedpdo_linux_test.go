//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"errors"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

// PREOP verification distinguishes absence from mismatch and exits before any
// activation on refusal. Successful startup retains the runtime mailbox check.
func TestFixedPDOMappingWorkerReadback(t *testing.T) {
	for _, tc := range []struct {
		name        string
		args        []string
		fault       bool
		retries     int
		siiMismatch bool
	}{
		{"matches", nil, false, 0, false},
		{"transient", []string{"--sim-mapping-upload-failures", "1"}, false, 1, false},
		{"unavailable", []string{"--sim-mapping-upload-failures", "3"}, true, 3, false},
		{"differs", []string{"--sim-pdo-mismatch"}, true, 0, false},
		{"sii_only_mismatch", []string{"--sim-sii-revision", "22033"}, false, 0, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			f := compiledForValidation(t, "simulation-home.json")
			if tc.siiMismatch {
				// RevPi measured identities: SII 12024 differs from CoE 22033.
				machine := compiledCoordinateJSON(t, "config/machines/simulation/simulation-home.json")
				machine["axes"].([]any)[0].(map[string]any)["profile"] = "coordinate_sim"
				profile := compiledCoordinateJSON(t, "config/drives/cia402_sim_home.json")
				profile["ethercat"].(map[string]any)["revision_no"] = 12024
				var err error
				f, _, err = compileCoordinateJSON(t, machine, profile)
				if err != nil {
					t.Fatal(err)
				}
			}
			dir, err := os.MkdirTemp("", "fm-")
			if err != nil {
				t.Fatal(err)
			}
			t.Cleanup(func() { _ = os.RemoveAll(dir) })
			log, err := os.Create(filepath.Join(dir, "daemon.log"))
			if err != nil {
				t.Fatal(err)
			}
			defer log.Close()
			// Unverified startup watchdog: 8000000000 ns, never a success condition.
			ctx, cancel := context.WithTimeout(context.Background(), 8*time.Second)
			defer cancel()
			args := append(append([]string{}, f.args...), "--socket-path", filepath.Join(dir, "ipc.sock"))
			cmd := exec.CommandContext(ctx, f.binary, append(args, tc.args...)...)
			cmd.Stdout, cmd.Stderr = log, log
			if err := cmd.Start(); err != nil {
				t.Fatal(err)
			}
			if tc.fault {
				var exit *exec.ExitError
				if err := cmd.Wait(); !errors.As(err, &exit) || exit.ExitCode() != 1 || ctx.Err() != nil {
					t.Fatalf("expected prompt bring-up refusal, err=%v context=%v", err, ctx.Err())
				}
				output, _ := os.ReadFile(log.Name())
				text := string(output)
				if strings.Contains(text, "IgH libecrt active") || strings.Contains(text, "phase=domain_reg_") ||
					!strings.Contains(text, "activation=blocked") {
					t.Fatalf("refused mapping reached activation: %s", text)
				}
				if strings.Count(text, "result=unavailable_retry") != tc.retries {
					t.Fatalf("wrong retry count: %s", text)
				}
				if tc.name == "differs" && (!strings.Contains(text,
					"result=pdo_mapping_mismatch attempts=1 index=0x1a00 sub=0x01 expected=0x60410010 observed=0x60410018") ||
					!strings.Contains(text, "result=refused attempts=1")) {
					t.Fatalf("mismatch element missing: %s", text)
				}
				return
			}
			defer func() { _ = cmd.Process.Kill(); _ = cmd.Wait() }()
			// Unverified observation poll: 10000000 ns; pass requires exact masks.
			tick := time.NewTicker(10 * time.Millisecond)
			defer tick.Stop()
			var revisionCycle uint64
			for {
				raw, _ := os.ReadFile(filepath.Join(dir, "metrics.json"))
				var status Status
				if json.Unmarshal(raw, &status) == nil {
					if tc.siiMismatch {
						if status.ConfigVerifiedMask != 0 {
							t.Fatalf("SII-only mismatch verified: %+v", status)
						}
						var metrics struct {
							Cycle uint64 `json:"rt_cycle_counter"`
						}
						if err := json.Unmarshal(raw, &metrics); err != nil {
							t.Fatal(err)
						}
						if len(status.Axes) == 1 && status.Axes[0].RevisionValid == 1 && status.Axes[0].Revision == 12024 {
							if revisionCycle == 0 {
								revisionCycle = metrics.Cycle
							}
							// Unverified observation span: 1000 executed cycles after CoE readback.
							if metrics.Cycle >= revisionCycle+1000 {
								break
							}
						}
					}
					if !tc.siiMismatch && status.ConfigVerifiedMask == 1 {
						if status.ExecutionFaultReasons != 0 {
							t.Fatalf("transient fault latched: %+v", status)
						}
						break
					}
				}
				select {
				case <-ctx.Done():
					output, _ := os.ReadFile(log.Name())
					t.Fatalf("no terminal readback: %s", output)
				case <-tick.C:
				}
			}
			output, _ := os.ReadFile(log.Name())
			// Runtime success must be observed independently of PREOP proof;
			// the earlier phase cannot satisfy the worker's positive assertion.
			if !strings.Contains(string(output), "EtherCAT mapping_readback axis=0 slave_pos=0 result=verified reason=0\n") {
				t.Fatalf("mapping worker never verified: %s", output)
			}
			if count := strings.Count(string(output), "result=unavailable_retry"); count != tc.retries {
				t.Fatalf("unavailable attempts = %d, want %d: %s", count, tc.retries, output)
			}
			if strings.Contains(string(output), "result=pdo_mapping_mismatch") {
				t.Fatalf("unavailable or matching mapping became an observed mismatch: %s", output)
			}
		})
	}
}
