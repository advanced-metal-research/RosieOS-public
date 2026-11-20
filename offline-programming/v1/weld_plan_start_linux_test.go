//go:build linux

package main

import (
	"context"
	"encoding/json"
	"errors"
	"os"
	"strings"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
)

type startObserver struct {
	fakeCellTarget
	calls     int
	mutate    func(map[string]any)
	transient bool
	// every shapes every sample (mutate only the later ones).
	every func(map[string]any)
}

func (f *startObserver) Status(context.Context) *denseexec.StatusBody {
	raw, _ := startFixture()
	f.calls++
	status := raw["rt_core"].(map[string]any)["status"].(map[string]any)
	if f.every != nil {
		f.every(status)
	}
	status["motion"].(map[string]any)["time_ns"] = 100 + f.calls
	for _, a := range status["axes"].([]any) {
		a.(map[string]any)["time_ns"] = 100 + f.calls
	}
	if f.calls > 1 && f.mutate != nil && (!f.transient || f.calls == 3) {
		f.mutate(status)
	}
	data, _ := json.Marshal(raw)
	var decoded denseexec.StatusBody
	if err := json.Unmarshal(data, &decoded); err != nil {
		panic(err)
	}
	return &decoded
}
func TestMeasuredPlanningCaptureRequiresAdvancingSameIdentity(t *testing.T) {
	_, robot := startFixture()
	for _, name := range []string{"fresh", "stale", "home_changed", "moved_j6", "stale_j6", "intermediate_excursion"} {
		t.Run(name, func(t *testing.T) {
			observer := &startObserver{transient: name == "intermediate_excursion", mutate: func(s map[string]any) {
				switch name {
				case "stale":
					s["motion"].(map[string]any)["time_ns"] = 101
				case "home_changed":
					s["core"].(map[string]any)["home_epoch"] = 33
				case "stale_j6":
					s["axes"].([]any)[5].(map[string]any)["time_ns"] = 101
				case "moved_j6", "intermediate_excursion":
					s["axes"].([]any)[5].(map[string]any)["logical_position"] = .51
				}
			}}
			start, err := captureMeasuredPlanStart(context.Background(), observer, robot)
			if name == "fresh" {
				if err != nil || start == nil || observer.calls < 5 {
					t.Fatalf("fresh capture: %v", err)
				}
			} else if err == nil {
				t.Fatal("accepted stale/torn/moving capture")
			}
		})
	}
}

// A finished jog with the robot still armed and leased is a planning start,
// through the whole stillness window; a joint that drifts more than 1e-5 rad
// inside that window is refused as motion, in the words an operator acts on.
func TestMeasuredPlanningCaptureAfterFinishedJogWhileArmed(t *testing.T) {
	_, robot := startFixture()
	finishedJog := func(s map[string]any) {
		s["motion"].(map[string]any)["mode"], s["motion"].(map[string]any)["state"] = 3, 5
		s["jog"] = map[string]any{"Open": 0, "HasInput": 0}
		s["grant"] = map[string]any{"GrantActive": 1, "StopAckId": 0}
		core := s["core"].(map[string]any)
		core["armed"], core["axis_enable_mask"], core["lease_valid"] = 1, 63, 1
	}
	for _, name := range []string{"still", "drift_j3"} {
		t.Run(name, func(t *testing.T) {
			observer := &startObserver{every: finishedJog, mutate: func(s map[string]any) {
				if name == "drift_j3" {
					// 2e-5 rad: twice the window's bound, far below anything visible.
					s["axes"].([]any)[2].(map[string]any)["logical_position"] = .2 + 2e-5
				}
			}}
			start, err := captureMeasuredPlanStart(context.Background(), observer, robot)
			if name == "still" {
				if err != nil || start == nil || observer.calls < 5 || start.JointValues[2] != .2 {
					t.Fatalf("finished armed jog refused: %v", err)
				}
				return
			}
			var refusal *planStartRefusal
			if !errors.As(err, &refusal) || refusal.code != planStartMoving || !strings.Contains(err.Error(), "robot moved") || !strings.Contains(err.Error(), "J3") {
				t.Fatalf("drift inside the window: %v", err)
			}
		})
	}
}

func TestMeasuredPlanningStartActualNativeSimulatorStatus(t *testing.T) {
	path := os.Getenv("RTCORE_CAPTURE_PUBLIC_STATUS")
	if path == "" {
		t.Skip("requires isolated native public-status export")
	}
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	var captured struct {
		Observations []json.RawMessage `json:"observations"`
	}
	if err = json.Unmarshal(raw, &captured); err != nil || len(captured.Observations) < 5 {
		t.Fatalf("native observations: %v", err)
	}
	// The flat simulator binds no robot. Only this authoring model identity is
	// synthetic; all public native axes, state, clocks and epochs stay untouched.
	var previous *measuredPlanStart
	for _, observation := range captured.Observations {
		var envelope map[string]any
		if err = json.Unmarshal(observation, &envelope); err != nil {
			t.Fatal(err)
		}
		native := envelope["rt_core"].(map[string]any)
		status := native["status"].(map[string]any)
		cfg := status["core"].(map[string]any)["configuration_sha256"].(string)
		robot := denseexec.RobotCell{Valid: true, ModelID: "synthetic-flat-simulation", RobotDescriptionSHA256: "sha256:simulation-only", MachinePlanningCalibrationSHA256: "sha256:simulation-only", MachineConfigurationSHA256: "sha256:" + cfg}
		envelope["robot_cell"] = robot
		data, _ := json.Marshal(envelope)
		start, e := measuredStartObservation(data, robot)
		if e != nil {
			t.Fatalf("actual native status refused: %v; %s", e, data)
		}
		if previous != nil {
			if start.ObservedTimeNS <= previous.ObservedTimeNS {
				t.Fatal("native motion clock did not advance")
			}
			for i := range start.JointNames {
				if start.AxisTimes[i] <= previous.AxisTimes[i] {
					t.Fatal("native axis clock did not advance")
				}
			}
		}
		previous = start
	}
}

func TestTraceCaptureUsesStationaryHeldTargets(t *testing.T) {
	_, robot := startFixture()
	for _, scenario := range []string{"still", "target_changed", "feedback_moved"} {
		t.Run(scenario, func(t *testing.T) {
			observer := &startObserver{every: func(s map[string]any) {
				for _, value := range s["axes"].([]any) {
					a := value.(map[string]any)
					a["logical_target"] = a["logical_position"].(float64) + .0002
				}
			}, mutate: func(s map[string]any) {
				a := s["axes"].([]any)[3].(map[string]any)
				if scenario == "target_changed" {
					a["logical_target"] = .301
				}
				if scenario == "feedback_moved" {
					a["logical_position"] = .301
				}
			}}
			start, err := captureMeasuredPlanStart(context.Background(), observer, robot, true)
			if scenario == "still" {
				if err != nil || start == nil || start.JointValues[0] != .0002 {
					t.Fatalf("%+v %v", start, err)
				}
			} else if err == nil {
				t.Fatal("moving trace start accepted")
			}
		})
	}
}
