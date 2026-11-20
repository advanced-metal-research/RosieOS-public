package main

import (
	"fmt"
	"path/filepath"
)

// MotorSpec is the nameplate data the inertia estimate needs and the runtime
// never sees: rated torque turns 0x6077 (per mille of rated) into N.m, rotor
// inertia is C00.06's 100 %. Both come from a catalog file with a manual
// citation per entry (rt-core/config/motors), never from code.
type MotorSpec struct {
	Model            string  `json:"model"`
	RatedTorqueNm    float64 `json:"rated_torque_nm"`
	RotorInertiaKgM2 float64 `json:"rotor_inertia_kg_m2"`
	Source           string  `json:"source"`
}

// JointMotor is one joint's motor with the evidence for the assignment. An
// unverified assignment is carried into every report that uses it.
type JointMotor struct {
	Joint    string    `json:"joint"`
	Motor    MotorSpec `json:"motor"`
	Verified bool      `json:"verified"`
	Source   string    `json:"source"`
	File     string    `json:"file"`
}

// loadJointMotor reads a rosie.joint-motors.v1 file and its catalog and
// returns the named joint's motor. A joint or model that is missing, or a
// non-positive value, is an error: the estimate must not run on a default.
func loadJointMotor(path, joint string) (JointMotor, error) {
	var jm struct {
		Schema  string `json:"schema"`
		Catalog string `json:"catalog"`
		Joints  []struct {
			Joint    string `json:"joint"`
			Motor    string `json:"motor"`
			Verified bool   `json:"verified"`
			Source   string `json:"source"`
		} `json:"joints"`
	}
	if _, err := readJSON(path, &jm); err != nil {
		return JointMotor{}, err
	}
	if jm.Schema != "rosie.joint-motors.v1" || jm.Catalog == "" {
		return JointMotor{}, fmt.Errorf("%s: want schema rosie.joint-motors.v1 with a catalog", path)
	}
	var cat struct {
		Schema string      `json:"schema"`
		Motors []MotorSpec `json:"motors"`
	}
	catPath := filepath.Join(filepath.Dir(path), jm.Catalog)
	if _, err := readJSON(catPath, &cat); err != nil {
		return JointMotor{}, err
	}
	if cat.Schema != "rosie.motor-catalog.v1" {
		return JointMotor{}, fmt.Errorf("%s: want schema rosie.motor-catalog.v1", catPath)
	}
	for _, j := range jm.Joints {
		if j.Joint != joint {
			continue
		}
		for _, m := range cat.Motors {
			if m.Model != j.Motor {
				continue
			}
			if !(m.RatedTorqueNm > 0) || !(m.RotorInertiaKgM2 > 0) {
				return JointMotor{}, fmt.Errorf("%s: %s has no positive rated torque and rotor inertia", catPath, m.Model)
			}
			return JointMotor{Joint: joint, Motor: m, Verified: j.Verified, Source: j.Source, File: path}, nil
		}
		return JointMotor{}, fmt.Errorf("%s: %s's motor %q is not in %s", path, joint, j.Motor, catPath)
	}
	return JointMotor{}, fmt.Errorf("%s: no motor for joint %s", path, joint)
}
