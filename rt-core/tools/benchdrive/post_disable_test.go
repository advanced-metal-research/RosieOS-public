//go:build linux

package main

import (
	"context"
	"encoding/json"
	"errors"
	"net/http"
	"strings"
	"sync"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// These are controlled public observations, not physical brake qualification.
// No receipt or elapsed sleep is substituted for advancing native feedback.
func TestPostDisableObservation(t *testing.T) {
	for _, name := range []string{"applied", "no_brake", "late_fault", "frozen_core", "frozen_axis", "frozen_other_axis", "selected_second_frozen", "future_axis", "regressed_axis", "stale_pdo", "old_axis", "transport", "deadline", "active_grant", "missing_grant", "active_lease", "unknown_brake", "drive_enabled", "drive_error", "armed", "brake_warning_late", "supervisor_event_late", "warning_before_run"} {
		t.Run(name, func(t *testing.T) {
			f := &fixture{hash: strings.Repeat("a", 64), build: testBuildDir(t)}
			var mu sync.Mutex
			requests := 0
			var first uint64
			f.socket = unixServer(t, func(w http.ResponseWriter, req *http.Request) {
				if req.URL.Path != "/v1/status" {
					t.Errorf("unexpected mutation or endpoint: %s", req.URL.Path)
					http.Error(w, "unexpected", 500)
					return
				}
				mu.Lock()
				requests++
				n := requests
				now := ipcclient.HostMonotonicNS()
				if first == 0 {
					first = now
				}
				stamp := first
				mu.Unlock()
				if name == "transport" {
					http.Error(w, "observation lost", http.StatusServiceUnavailable)
					return
				}
				if name == "deadline" {
					<-req.Context().Done()
					return
				}
				s := control.Status{Core: ipcclient.Status{TimeNS: now, Axes: []ipcclient.AxisStatus{{TimeNS: now, Statusword: 0x240, BrakeState: 2, PDOFresh: 1}}}, Grant: &ipcclient.GrantObservation{}}
				a := &s.Core.Axes[0]
				switch name {
				case "applied":
					if n == 1 {
						a.BrakeState = 4
					}
				case "no_brake":
					a.BrakeState = 1
				case "late_fault":
					if n == 1 {
						a.BrakeState = 4
					} else {
						s.Core.SafetyFaultMask = 1
						s.Core.ExecutionFaultReasons = 128
						s.Core.Recovery = &ipcclient.RecoveryStatus{Faults: []ipcclient.FaultRecovery{{Bit: 7, Name: "drive_readiness", AxisMask: 1}}}
					}
				case "frozen_core":
					s.Core.TimeNS = stamp
					a.TimeNS = stamp
				case "frozen_axis":
					a.TimeNS = stamp
				case "future_axis":
					a.TimeNS = now + 1
				case "regressed_axis":
					if n > 1 {
						a.TimeNS = stamp - 1
					}
				case "frozen_other_axis", "selected_second_frozen":
					s.Core.Axes = append(s.Core.Axes, ipcclient.AxisStatus{TimeNS: stamp, Statusword: 0x240, BrakeState: 2, PDOFresh: 1})
				case "stale_pdo":
					a.PDOFresh = 0
				case "old_axis":
					a.TimeNS = 1
				case "active_grant":
					s.Grant.GrantActive = 1
				case "missing_grant":
					s.Grant = nil
				case "active_lease":
					s.Core.LeaseValid = 1
				case "unknown_brake":
					a.BrakeState = 0
				case "drive_enabled":
					a.Statusword = 0x227
				case "drive_error":
					a.ErrorCode = 0x7121
				case "armed":
					s.Core.Armed = 1
				case "brake_warning_late":
					if n > 1 {
						a.BrakeHoldDiagnostic = ipcclient.BrakeHoldDiagnostic{Severity: "warning", WarningSequence: 1, Transition: "torque_off",
							Cause: "transition_corridor", Phase: "closing",
							Message: "Brake appears to latch after torque off - adjust drive brake hold delay (C05.10)."}
					}
				case "supervisor_event_late":
					if n > 1 {
						a.DriveSupervisor = ipcclient.DriveSupervisorDiagnostic{Severity: "warning", EventSequence: 1, Kind: "disarm_motion",
							Cause: "none", Message: "J1 moved 1.200 deg while disarming (drive still energized); bound 1.146 deg."}
					}
				case "warning_before_run":
					a.BrakeHoldDiagnostic = ipcclient.BrakeHoldDiagnostic{Severity: "warning", WarningSequence: 4}
					a.DriveSupervisor = ipcclient.DriveSupervisorDiagnostic{Severity: "arm_ended", EventSequence: 2, Kind: "enable_timeout"}
				}
				if err := json.NewEncoder(w).Encode(map[string]any{"schema": "rosie.rt-control.response.v1", "data": s}); err != nil && req.Context().Err() == nil {
					t.Error(err)
				}
			})
			r := testRunner(t, f, "move", name)
			r.description = control.Description{NativeDescription: ipcclient.Description{Axes: []ipcclient.AxisDescription{{Index: 0}}}}
			r.o.mask = 1
			if name == "frozen_other_axis" || name == "selected_second_frozen" {
				r.description.Axes = append(r.description.Axes, ipcclient.AxisDescription{Index: 1})
			}
			if name == "selected_second_frozen" {
				r.o.mask = 2
			}
			r.o.deadlineNS = int64(300 * time.Millisecond)
			// Isolate the terminal verdict from unrelated phase expectations.
			r.expectations = make(map[string]bool)
			err := r.postDisable(context.Background())
			success := name == "applied" || name == "no_brake" || name == "frozen_other_axis" || name == "warning_before_run"
			// A warning in the window is observed completion but not a clean pass.
			warned := name == "brake_warning_late" || name == "supervisor_event_late"
			if (err == nil) != success || r.expectations["post_disable_observed"] != (success || warned) {
				t.Fatalf("postDisable=%v expectations=%v; expected success=%v", err, r.expectations, success)
			}
			if warned {
				want := map[string]string{"brake_warning_late": "axis 0 brake warning 1 (transition_corridor during closing): Brake appears to latch",
					"supervisor_event_late": "axis 0 drive supervisor disarm_motion: J1 moved 1.200 deg while disarming"}[name]
				if free, found := r.expectations["post_disable_warning_free"]; !found || free || !strings.Contains(err.Error(), want) {
					t.Fatalf("warning lost from verdict: %v expectations=%v", err, r.expectations)
				}
				if code, _ := outcome(err); code == 0 {
					t.Fatal("warned run produced successful exit")
				}
				if summary := r.summary(err); !strings.Contains(summary.Error(), "post_disable_warning_free") {
					t.Fatalf("summary omits the warned verdict: %v", summary)
				}
			}
			mu.Lock()
			count := requests
			mu.Unlock()
			if success && count < 2 || name == "applied" && count < 3 {
				t.Fatalf("accepted boundary without advancing terminal observations: requests=%d", count)
			}
			if name == "late_fault" && !strings.Contains(err.Error(), "drive_readiness (bit=7 axis_mask=1)") {
				t.Fatalf("lost original native fault identity: %v", err)
			}
			if name == "late_fault" {
				original := errors.New("original command failure")
				err = errors.Join(original, err)
				err = r.summary(err)
				if !errors.Is(err, original) || !strings.Contains(err.Error(), "drive_readiness") {
					t.Fatalf("terminal summary lost one of the failures: %v", err)
				}
				if code, _ := outcome(err); code == 0 {
					t.Fatal("late native fault produced successful exit")
				}
				records := readRecords(t, r.o.evidence)
				last := records[len(records)-1]
				if last.Kind != "summary" || last.ExitCode == 0 || !strings.Contains(strings.Join(last.NotMet, " "), "drive_readiness") {
					t.Fatalf("false-success evidence: %+v", last)
				}
			}
		})
	}
}

// Exercise run's actual deferred Stop/Release path. A successful command cannot
// hide a fault first published after Release; an earlier rejection must survive
// alongside that late fault in both the returned error and persisted verdict.
func TestPostDisableFaultReachesRunVerdict(t *testing.T) {
	for _, originalFailure := range []bool{false, true} {
		name := "completed_home"
		if originalFailure {
			name = "rejected_home"
		}
		t.Run(name, func(t *testing.T) {
			f := &fixture{hash: strings.Repeat("b", 64), build: testBuildDir(t)}
			var mu sync.Mutex
			grant := control.Grant{}
			homed, released := false, false
			afterRelease := 0
			f.socket = unixServer(t, func(w http.ResponseWriter, req *http.Request) {
				mu.Lock()
				defer mu.Unlock()
				response := map[string]any{"schema": "rosie.rt-control.response.v1"}
				switch req.URL.Path {
				case "/v1/describe":
					response["data"] = control.Description{ContractVersion: 2, NativeDescription: ipcclient.Description{ConfigurationSHA256: f.hash, Axes: []ipcclient.AxisDescription{{Index: 0, RequireHome: true, NativeHome: true}}}}
				case "/v1/status":
					now := ipcclient.HostMonotonicNS()
					s := control.Status{Core: ipcclient.Status{TimeNS: now, Axes: []ipcclient.AxisStatus{{TimeNS: now, Statusword: 0x240, BrakeState: 2, PDOFresh: 1}}}, Grant: &ipcclient.GrantObservation{}}
					if grant.Session != "" {
						s.Core.LeaseValid = 1
						s.Grant.GrantActive = 1
					}
					if homed {
						s.Core.HomeValidMask = 1
						s.Core.HomeEpoch = 1
						s.Core.Axes[0].HomeValid = true
					}
					if released {
						afterRelease++
						if afterRelease >= 2 {
							s.Core.SafetyFaultMask = 1
							s.Core.ExecutionFaultReasons = 128
							s.Core.Recovery = &ipcclient.RecoveryStatus{Faults: []ipcclient.FaultRecovery{{Bit: 7, Name: "drive_readiness", AxisMask: 1}}}
						}
					}
					response["data"] = s
				case "/v1/control":
					var command control.Request
					if err := json.NewDecoder(req.Body).Decode(&command); err != nil {
						t.Error(err)
						return
					}
					response["operation"] = command.Operation
					switch command.Operation {
					case "acquire":
						grant = control.Grant{Session: "0123456789abcdef0123456789abcdef", Generation: 1, LeaseMS: 500}
					case "renew":
					case "home":
						response["sequence"] = 1
						if originalFailure {
							response["error"] = "original_home_failure"
							w.WriteHeader(http.StatusConflict)
						} else {
							homed = true
						}
					case "stop":
						grant.Generation++
					case "release":
						grant = control.Grant{}
						released = true
					default:
						t.Errorf("unexpected mutation %s", command.Operation)
					}
					response["data"] = grant
				default:
					t.Errorf("unexpected path %s", req.URL.Path)
				}
				if err := json.NewEncoder(w).Encode(response); err != nil && req.Context().Err() == nil {
					t.Error(err)
				}
			})
			r := testRunner(t, f, "home", name)
			r.o.mask = 1
			r.o.deadlineNS = int64(time.Second)
			err := r.run(context.Background())
			if err == nil || !strings.Contains(err.Error(), "drive_readiness") {
				t.Fatalf("late fault lost from cleanup: %v", err)
			}
			if originalFailure && !strings.Contains(err.Error(), "original_home_failure") {
				t.Fatalf("earlier command error overwritten: %v", err)
			}
			if err = r.summary(err); err == nil {
				t.Fatal("failed run summarized as success")
			}
			records := readRecords(t, r.o.evidence)
			last := records[len(records)-1]
			if last.Kind != "summary" || last.ExitCode == 0 || !strings.Contains(strings.Join(last.NotMet, " "), "drive_readiness") {
				t.Fatalf("false-success summary: %+v", last)
			}
			mu.Lock()
			defer mu.Unlock()
			if !released || afterRelease < 2 {
				t.Fatalf("cleanup did not observe terminal transition: release=%v reads=%d", released, afterRelease)
			}
		})
	}
}
