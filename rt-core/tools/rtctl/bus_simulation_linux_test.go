//go:build linux

package rtctl

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"golang.org/x/sys/unix"
	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
	control "rosieos/rt-core/sdk/control"
)

// Unverified test watchdogs bound startup and cleanup; observed state alone
// determines success. Poll cadence is 10 ms, never a positive pass condition.
const busTestWatchdog = 10 * time.Second

// busSixAxisFixture pins the six-axis group independently of the three scanned
// extras, preventing admission tests from promoting a held device into an axis.
func busSixAxisFixture(t *testing.T) *fixture {
	t.Helper()
	f := fresh()
	var allowed unix.CPUSet
	if err := unix.SchedGetaffinity(0, &allowed); err != nil {
		t.Fatal(err)
	}
	if allowed.Count() < 2 {
		t.Skip("two CPUs required by daemon")
	}
	for cpu := 0; cpu < 1024; cpu++ {
		if allowed.IsSet(cpu) {
			f.put("machine.runtime.cpu", fmt.Sprint(cpu))
			break
		}
	}
	// Unverified fixture retention: native/compiler minimum is 100 ms.
	f.put("machine.runtime.telemetry", `{"retention_ms":100}`)
	var axes []any
	for i := 0; i < 6; i++ {
		a := asObject(decode([]byte(canonical(f.get("machine.axes.0")))))
		a.set("name", fmt.Sprintf("J%d", i+1))
		a.set("slave_position", json.Number(fmt.Sprint(i)))
		axes = append(axes, a)
	}
	f.put("machine.axes", canonical(axes))
	return f
}

// TestBusSixOfNine exercises production bring-up, PDO registration, convergence
// and publication with three physical extras that have no slave configuration.
func TestBusSixOfNine(t *testing.T) {
	for _, tc := range []struct{ name, policy, revision, refusal string }{
		{"hold", `{"unknown_slaves":"hold","unknown_slaves_note":"unverified six of nine bench"}`, "12024", ""},
		{"refuse_unknown", `{"unknown_slaves":"refuse"}`, "12024", "disposition=refused_unknown"},
		{"refuse_declared", "declared", "12024", ""},
		{"refuse_wrong_identity", "declared", "12025", "disposition=refused_identity"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			f := busSixAxisFixture(t)
			block := tc.policy
			if block == "declared" {
				var entries []string
				for i := 6; i < 9; i++ {
					entries = append(entries, fmt.Sprintf(`{"slave_position":%d,"vendor_id":4194304,"product_code":1813,"revision":12024,"note":"unverified spare"}`, i))
				}
				block = `{"unknown_slaves":"refuse","unused_slaves":[` + strings.Join(entries, ",") + `]}`
			}
			f.put("machine.bus", block)
			compiled, err := f.compile()
			if err != nil {
				t.Fatal(err)
			}
			_, args, err := writeCompiled(compiled, buildTemp(t))
			if err != nil {
				t.Fatal(err)
			}
			dir, err := os.MkdirTemp("", "bt-")
			if err != nil {
				t.Fatal(err)
			}
			t.Cleanup(func() { os.RemoveAll(dir) })
			socket := filepath.Join(dir, "ipc.sock")
			args = append(args, "--socket-path", socket)
			for i := 6; i < 9; i++ {
				args = append(args, "--sim-extra-slave", fmt.Sprintf("%d|4194304|1813|%s", i, tc.revision))
			}
			if tc.refusal != "" {
				out, err := runBringupDaemon(t, args...)
				if err == nil || !strings.Contains(out, tc.refusal) || !strings.Contains(out, "position=6 vendor_id=0x00400000 product_code=0x00000715 revision=") || strings.Contains(out, "IgH libecrt active") {
					t.Fatalf("refusal missing or master activated: %v %s", err, out)
				}
				return
			}
			logPath := filepath.Join(dir, "core.log")
			cmd := exec.Command(filepath.Join(testRoot(t), "build/rosie-rt-core-sim"), args...)
			simfixture.SpawnLogged(t, cmd, logPath, busTestWatchdog, 12000, nil)
			simfixture.WaitVerified(t, filepath.Join(dir, "metrics.json"), busTestWatchdog, 10*time.Millisecond, func(v simfixture.Verified) bool { return v.ConfigVerifiedMask == 63 })
			ctx, cancel := context.WithTimeout(context.Background(), busTestWatchdog)
			defer cancel()
			client, err := ipcclient.Dial(ctx, ipcclient.Options{SocketPath: socket, ConfigurationSHA256: compiled.SHA256, Backend: "simulation", SimulationTest: true})
			if err != nil {
				t.Fatal(err)
			}
			defer client.Close()
			status, err := client.Status()
			if err != nil {
				t.Fatal(err)
			}
			description, err := client.Describe()
			if err != nil {
				t.Fatal(err)
			}
			for _, bus := range []json.RawMessage{status.Bus, description.Bus} {
				var view struct {
					RespondingExpected uint32 `json:"responding_expected"`
					Admitted           bool   `json:"admitted"`
					Held               []struct {
						Position uint32 `json:"slave_position"`
						Vendor   uint32 `json:"vendor_id"`
						Product  uint32 `json:"product_code"`
						Revision uint32 `json:"revision"`
						AL       uint32 `json:"al_state"`
						Present  bool   `json:"present"`
					} `json:"held_slaves"`
				}
				if err := json.Unmarshal(bus, &view); err != nil {
					t.Fatal(err)
				}
				if len(view.Held) != 3 || view.RespondingExpected != 9 || !view.Admitted {
					t.Fatalf("incorrect bus: %s", bus)
				}
				for i, held := range view.Held {
					if held.Position != uint32(6+i) || held.Vendor != 4194304 || held.Product != 1813 || held.Revision != 12024 || held.AL != 2 || !held.Present {
						t.Fatalf("incorrect held identity: %+v", held)
					}
				}
			}
			metrics, err := os.ReadFile(filepath.Join(dir, "metrics.json"))
			if err != nil {
				t.Fatal(err)
			}
			var counters struct {
				Actual      uint32 `json:"wkc_actual"`
				Expected    uint32 `json:"wkc_expected"`
				Responding  uint32 `json:"responding_slaves"`
				Online      uint32 `json:"online_slaves"`
				Operational uint32 `json:"operational_slaves"`
				Ready       uint32 `json:"startup_ready"`
			}
			if err := json.Unmarshal(metrics, &counters); err != nil {
				t.Fatal(err)
			}
			if counters.Actual != 12 || counters.Expected != 12 || counters.Responding != 9 || counters.Online != 6 || counters.Operational != 6 || counters.Ready != 1 {
				t.Fatalf("incorrect axis-only convergence: %+v", counters)
			}
			log, err := os.ReadFile(logPath)
			if err != nil {
				t.Fatal(err)
			}
			if !strings.Contains(string(log), "wkc simulator 12/12: two directions per axis, held devices zero") || strings.Contains(string(log), "wkc live lrw") {
				t.Fatalf("WKC backend/convention missing: %s", log)
			}
			for i := 6; i < 9; i++ {
				if strings.Count(string(log), fmt.Sprintf("bus slave position=%d ", i)) != 1 {
					t.Fatalf("held identity not logged exactly once: %s", log)
				}
				if strings.Contains(string(log), fmt.Sprintf("slave_pos=%d ", i)) {
					t.Fatalf("held slave configured: %s", log)
				}
			}
		})
	}
}

// TestBusDeclaredChangesAfterAdmission preserves the review trace as a running
// daemon regression: refuse must revoke ready/Enable/Arm for declared changes;
// hold must report them while preserving the six axes and zero extra outputs.
// Restoration must require public reset_fault, fresh authority, Enable and Arm,
// preventing a readiness recovery from silently clearing the fault or re-arming.
func TestBusDeclaredChangesAfterAdmission(t *testing.T) {
	t.Parallel()
	binary, out, err := nativeTestBinary(t, "bus-change-daemon", func(binary string) []string {
		return []string{"-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic", "-UNDEBUG", "-Iinclude", "-I.", "tests/bus_change_daemon.cpp", "-o", binary, "-pthread"}
	})
	if err != nil {
		t.Fatalf("compile topology trace daemon: %v %s", err, out)
	}
	for _, policy := range []string{"refuse", "hold"} {
		for _, change := range []struct {
			name, command, reason, refused, held string
			position                             uint32
			responding                           uint32
		}{
			{"identity", "1", "declared_identity_mismatch", "refused_identity", "identity_changed", 6, 9},
			{"missing", "2", "declared_slave_missing", "refused_missing", "disappeared", 8, 8},
		} {
			t.Run(policy+"/"+change.name, func(t *testing.T) {
				f := busSixAxisFixture(t)
				var unused []string
				for position := 6; position < 9; position++ {
					unused = append(unused, fmt.Sprintf(`{"slave_position":%d,"vendor_id":4194304,"product_code":1813,"revision":12024,"note":"unverified review fixture"}`, position))
				}
				f.put("machine.bus", `{"unknown_slaves":"`+policy+`","unknown_slaves_note":"unverified review fixture","unused_slaves":[`+strings.Join(unused, ",")+`]}`)
				compiled, err := f.compile()
				if err != nil {
					t.Fatal(err)
				}
				_, args, err := writeCompiled(compiled, buildTemp(t))
				if err != nil {
					t.Fatal(err)
				}
				dir, err := os.MkdirTemp("", "bc-")
				if err != nil {
					t.Fatal(err)
				}
				t.Cleanup(func() { os.RemoveAll(dir) })
				socket, trigger := filepath.Join(dir, "ipc.sock"), filepath.Join(dir, "change")
				args = append(args, "--socket-path", socket)
				for position := 6; position < 9; position++ {
					args = append(args, "--sim-extra-slave", fmt.Sprintf("%d|4194304|1813|12024", position))
				}
				cmd := exec.Command(binary, args...)
				cmd.Env = append(os.Environ(), "RTCORE_TEST_TOPOLOGY_CHANGE="+trigger)
				simfixture.SpawnLogged(t, cmd, filepath.Join(dir, "core.log"), busTestWatchdog, 12000,
					func(err error) {
						if err != nil {
							t.Errorf("trace daemon: %v", err)
						}
					},
					func(raw []byte) {
						for position := 6; position < 9; position++ {
							if !strings.Contains(string(raw), fmt.Sprintf("TRACE position=%d configs=0 pdos=0 downloads=0 startup=0 registrations=0 output_bytes=0", position)) {
								t.Errorf("missing zero-output trace at position %d", position)
							}
						}
						want := "policy=log_and_continue_axes reason=" + change.held
						if policy == "refuse" {
							want = "disposition=" + change.refused + " reason=" + change.reason + " readiness=blocked"
						}
						if !strings.Contains(string(raw), want) {
							t.Errorf("missing change log %s", want)
						}
					})
				type heldSlave struct {
					Position       uint32  `json:"slave_position"`
					Vendor         uint32  `json:"vendor_id"`
					ObservedVendor *uint32 `json:"observed_vendor_id"`
					ExpectedVendor uint32  `json:"expected_vendor_id"`
					Disposition    string  `json:"disposition"`
					Present        bool    `json:"present"`
				}
				type busView struct {
					Admitted       bool        `json:"admitted"`
					Reason         string      `json:"reason"`
					ReasonPosition *uint32     `json:"reason_position"`
					Expected       uint32      `json:"responding_expected"`
					ChangePolicy   string      `json:"unused_change_policy"`
					Held           []heldSlave `json:"held_slaves"`
				}
				type view struct {
					ipcclient.Status
					Topology    busView `json:"bus"`
					Ready       uint32  `json:"startup_ready"`
					Responding  uint32  `json:"responding_slaves"`
					ActualWKC   uint32  `json:"wkc_actual"`
					ExpectedWKC uint32  `json:"wkc_expected"`
				}
				wait := func(predicate func(view) bool) view {
					t.Helper()
					var latest view
					ctx, cancel := context.WithTimeout(context.Background(), busTestWatchdog)
					defer cancel()
					err := simfixture.Poll(ctx, 10*time.Millisecond, func() (bool, error) {
						raw, err := os.ReadFile(filepath.Join(dir, "metrics.json"))
						return err == nil && json.Unmarshal(raw, &latest) == nil && predicate(latest), nil
					})
					if err != nil {
						t.Fatalf("topology state: %v %+v", err, latest)
					}
					return latest
				}
				wait(func(v view) bool { return v.ConfigVerifiedMask == 63 && v.Ready == 1 && v.Topology.Admitted })
				ctx, cancel := context.WithTimeout(context.Background(), busTestWatchdog)
				defer cancel()
				client, err := ipcclient.Dial(ctx, ipcclient.Options{SocketPath: socket, ConfigurationSHA256: compiled.SHA256, Backend: "simulation", SimulationTest: true})
				if err != nil {
					t.Fatal(err)
				}
				defer client.Close()
				for _, submit := range []func() (uint64, error){func() (uint64, error) { return client.Enable(ctx, 63) }, func() (uint64, error) { return client.Arm(ctx) }} {
					seq, err := submit()
					if err != nil {
						t.Fatal(err)
					}
					if _, err := client.WaitResult(ctx, seq); err != nil {
						t.Fatal(err)
					}
				}
				before := wait(func(v view) bool { return v.Armed == 1 && v.AxisEnableMask == 63 && v.Ready == 1 })
				if err := os.WriteFile(trigger, []byte(change.command), 0600); err != nil {
					t.Fatal(err)
				}
				after := wait(func(v view) bool {
					if len(v.Topology.Held) != 3 || v.Responding != change.responding {
						return false
					}
					disposition := change.refused
					if policy == "hold" {
						disposition = change.held
					}
					if v.Topology.Held[change.position-6].Disposition != disposition {
						return false
					}
					if policy == "hold" {
						return v.Ready == 1 && v.Armed == 1 && v.AxisEnableMask == 63
					}
					return v.Ready == 0 && v.Armed == 0 && v.AxisEnableMask == 0 && v.ConfigVerifiedMask == 0 && v.HomeValidMask == 0
				})
				if after.Responding != change.responding || after.ActualWKC != 12 || after.ExpectedWKC != 12 {
					t.Fatalf("counter conventions changed: %+v", after)
				}
				if policy == "refuse" {
					if after.ExecutionFaultReasons != 128 || after.SafetyFaultMask != 63 {
						t.Fatalf("readiness fault absent: %+v", after)
					}
					if after.Topology.Admitted || after.Topology.Expected != 9 || after.Topology.Reason != change.reason || after.Topology.ReasonPosition == nil || *after.Topology.ReasonPosition != change.position || after.Topology.ChangePolicy != "refuse" || after.ConfigurationEpoch != before.ConfigurationEpoch+1 {
						t.Fatalf("strict admission did not follow loss-of-slave path: %+v", after)
					}
				} else if !after.Topology.Admitted || after.Topology.Expected != change.responding || after.Topology.Reason != "none" || after.Topology.ChangePolicy != "log_and_continue_axes" || after.ConfigVerifiedMask != 63 || after.ConfigurationEpoch != before.ConfigurationEpoch || after.ExecutionFaultReasons != 0 || after.SafetyFaultMask != 0 {
					t.Fatalf("hold lost admission/evidence: %+v", after)
				}
				device := after.Topology.Held[change.position-6]
				if device.Position != change.position || device.Vendor != 4194304 || device.ExpectedVendor != 4194304 {
					t.Fatalf("identity missing: %+v", device)
				}
				if change.command == "1" {
					if !device.Present || device.ObservedVendor == nil || *device.ObservedVendor != 1 {
						t.Fatalf("mismatch missing: %+v", device)
					}
				} else if device.Present || device.ObservedVendor != nil {
					t.Fatalf("absence missing: %+v", device)
				}
				description, err := client.Describe()
				if err != nil {
					t.Fatal(err)
				}
				var described busView
				if err := json.Unmarshal(description.Bus, &described); err != nil {
					t.Fatal(err)
				}
				if described.Reason != after.Topology.Reason || len(described.Held) != 3 || described.Held[change.position-6].Disposition != device.Disposition {
					t.Fatalf("description dropped change: %s", description.Bus)
				}
				if err := os.WriteFile(trigger, []byte("0"), 0600); err != nil {
					t.Fatal(err)
				}
				restored := wait(func(v view) bool {
					return v.Topology.Admitted && v.Topology.Expected == 9 && v.Topology.Reason == "none" && v.Ready == 1
				})
				if policy == "hold" {
					if restored.Armed != 1 || restored.AxisEnableMask != 63 || restored.ExecutionFaultReasons != 0 || restored.SafetyFaultMask != 0 || restored.ConfigurationEpoch != before.ConfigurationEpoch {
						t.Fatalf("held spare restoration changed axis authority or faults: %+v", restored)
					}
					return
				}
				verified := wait(func(v view) bool { return v.TimeNS > restored.TimeNS && v.ConfigVerifiedMask == 63 })
				if verified.Armed != 0 || verified.AxisEnableMask != 0 || verified.ExecutionFaultReasons != 128 || verified.SafetyFaultMask != 63 || verified.HomeValidMask != 0 || verified.ConfigurationEpoch != before.ConfigurationEpoch+1 {
					t.Fatalf("topology restoration cleared the fault or replayed Enable/Arm: %+v", verified)
				}
				// Hand native ownership to the real adapter; all recovery commands
				// use control.sock so a private reset cannot conceal a public failure.
				if err := client.Close(); err != nil {
					t.Fatal(err)
				}
				publicSocket := filepath.Join(dir, "control.sock")
				adapter := exec.Command(filepath.Join(testRoot(t), "build/rt-control"),
					"--core-socket", socket, "--socket", publicSocket,
					"--configuration-sha256", compiled.SHA256, "--backend", "simulation",
					"--pair-id", "bus-restoration", "--pair-revision", "1")
				simfixture.SpawnLogged(t, adapter, filepath.Join(dir, "control.log"), busTestWatchdog, 12000, simfixture.CheckExit(t))
				public, err := control.Dial(control.UnixPath(publicSocket))
				if err != nil {
					t.Fatal(err)
				}
				defer public.Close()
				if err := simfixture.Poll(ctx, 10*time.Millisecond, func() (bool, error) {
					_, err := public.Describe(ctx)
					return err == nil, nil
				}); err != nil {
					t.Fatalf("public adapter startup: %v", err)
				}
				binding := control.Binding{PairID: "bus-restoration", Revision: 1, ConfigurationSHA256: compiled.SHA256}
				if _, err := public.Acquire(ctx, "bus-restoration", binding); err != nil {
					t.Fatal(err)
				}
				result, err := public.Enable(ctx, 63)
				var rejected *control.Rejected
				if !errors.As(err, &rejected) || rejected.NativeResult == nil || rejected.NativeResult.Operation != 0x102 || rejected.NativeResult.Result != ipcclient.ResultRejected || rejected.NativeResult.Reason != ipcclient.ReasonNotReady || rejected.NativeResult.Sequence != result.Sequence {
					t.Fatalf("Enable before reset must reject with native not_ready (2): %+v %v", result, err)
				}
				reset, err := public.ResetFault(ctx, 63)
				if err != nil || reset.NativeResult == nil || reset.NativeResult.Operation != ipcclient.MsgFaultReset || reset.NativeResult.Result != ipcclient.ResultAccepted || reset.NativeResult.Sequence != reset.Sequence {
					t.Fatalf("public reset_fault: %+v %v", reset, err)
				}
				cleared := wait(func(v view) bool {
					return v.TimeNS > verified.TimeNS && v.Recovery != nil && v.Recovery.ResetSequence == reset.Sequence && v.ExecutionFaultReasons == 0 && v.SafetyFaultMask == 0
				})
				if cleared.Armed != 0 || cleared.AxisEnableMask != 0 || cleared.HomeValidMask != 0 || cleared.ConfigurationEpoch != before.ConfigurationEpoch+1 {
					t.Fatalf("reset replayed authority or Home evidence: %+v", cleared)
				}
				// reset_fault retires the session; reacquisition is explicit and
				// must still leave Enable and Arm as separate operator actions.
				public.SetFence(control.Fence{})
				if _, err := public.Acquire(ctx, "bus-restoration", binding); err != nil {
					t.Fatal(err)
				}
				if _, err := public.Enable(ctx, 63); err != nil {
					t.Fatal(err)
				}
				enabled := wait(func(v view) bool { return v.TimeNS > cleared.TimeNS && v.AxisEnableMask == 63 })
				if enabled.Armed != 0 || enabled.ExecutionFaultReasons != 0 || enabled.SafetyFaultMask != 0 {
					t.Fatalf("Enable implicitly armed or retained a fault: %+v", enabled)
				}
				if _, err := public.Arm(ctx); err != nil {
					t.Fatal(err)
				}
				wait(func(v view) bool {
					return v.Armed == 1 && v.AxisEnableMask == 63 && v.Ready == 1 && v.ExecutionFaultReasons == 0 && v.SafetyFaultMask == 0
				})
				t.Log("restoration: Enable refused (2); public reset_fault cleared reason 128 and safety mask 63 while inhibited; reacquire, Enable, Arm accepted")
			})
		}
	}
}
