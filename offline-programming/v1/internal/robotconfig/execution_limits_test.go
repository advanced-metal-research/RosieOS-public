package robotconfig

import (
	"bytes"
	"encoding/json"
	"math"
	"strings"
	"testing"
)

func executionFixture() ExecutionLimits {
	acceleration := 0.5
	return ExecutionLimits{"J1": {PositionUnit: "rad", LowerRad: -2, UpperRad: 2,
		VelocityRadS: 0.04, AccelerationRadS2: &acceleration}}
}

func compileExecution(t *testing.T, model Description, calibration string, limits ExecutionLimits) (Context, error) {
	t.Helper()
	source := SourceTarget
	if calibration == "" {
		source = SourceEmpty
	}
	settings, err := Resolve(model, []byte(calibration), source)
	if err != nil {
		t.Fatal(err)
	}
	return Compile(model, settings, limits)
}

func TestExecutionLimitsNarrowBothPlanningRepresentations(t *testing.T) {
	model := loadedModel(t, nil)
	context, err := compileExecution(t, model, "", executionFixture())
	if err != nil {
		t.Fatal(err)
	}
	planner := plannerOf(t, context)
	if planner.Bounds["J1"] != (JointBounds{-2, 2}) ||
		planner.Limits["J1"] != (PlannerRate{0.04, 0.04, 0.5}) {
		t.Fatalf("native limits not composed: %+v", planner)
	}
	urdf, err := ParseURDF(string(context.Files[ContextURDF]))
	if err != nil {
		t.Fatal(err)
	}
	joint := urdf.Joints["J1"]
	if joint.Lower != -2 || joint.Upper != 2 || joint.Velocity != 0.04 {
		t.Fatalf("corrected URDF differs from planner: %+v", joint)
	}
	if model.URDF.Joints["J1"].Lower != -3 || model.Config.PlanningJoints["J1"].AccelerationRadS2 != 2.1 {
		t.Fatal("compiling mutated the robot description")
	}
}

func TestExecutionLimitsNeverWidenCalibrationOrModel(t *testing.T) {
	limits := executionFixture()
	axis := limits["J1"]
	axis.LowerRad, axis.UpperRad, axis.VelocityRadS = -10, 10, 10
	acceleration := 10.0
	axis.AccelerationRadS2 = &acceleration
	limits["J1"] = axis
	context, err := compileExecution(t, loadedModel(t, nil),
		`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"joints":{"J1":{"lower_rad":-1,"velocity_rad_s":0.03}}}}`, limits)
	if err != nil {
		t.Fatal(err)
	}
	planner := plannerOf(t, context)
	if planner.Bounds["J1"] != (JointBounds{-1, 3}) || planner.Limits["J1"] != (PlannerRate{0.03, 0.03, 2.1}) {
		t.Fatalf("native snapshot widened settled settings: %+v", planner)
	}
}

func TestExecutionLimitsRejectIncompleteInvalidOrUnreachableSnapshots(t *testing.T) {
	for _, tc := range []struct {
		name, want string
		change     func(ExecutionLimits)
	}{
		{"missing driven", "missing driven", func(l ExecutionLimits) { delete(l, "J1") }},
		{"wrong units", "radian", func(l ExecutionLimits) { a := l["J1"]; a.PositionUnit = "deg"; l["J1"] = a }},
		{"nan position", "finite", func(l ExecutionLimits) { a := l["J1"]; a.LowerRad = math.NaN(); l["J1"] = a }},
		{"zero velocity", "finite", func(l ExecutionLimits) { a := l["J1"]; a.VelocityRadS = 0; l["J1"] = a }},
		{"invalid acceleration", "finite", func(l ExecutionLimits) { a := l["J1"]; value := math.Inf(1); a.AccelerationRadS2 = &value; l["J1"] = a }},
		{"unknown joint", "no bounded", func(l ExecutionLimits) { l["unknown"] = l["J1"] }},
		{"disjoint range", "do not overlap", func(l ExecutionLimits) { a := l["J1"]; a.LowerRad = 4; a.UpperRad = 5; l["J1"] = a }},
		{"reset outside", "reset pose", func(l ExecutionLimits) { a := l["J1"]; a.LowerRad = 1; l["J1"] = a }},
		{"held outside", "held pose", func(l ExecutionLimits) {
			l["J2"] = ExecutionAxisLimit{PositionUnit: "rad", LowerRad: 0.1, UpperRad: 0.5, VelocityRadS: 0.1}
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			limits := executionFixture()
			tc.change(limits)
			_, err := compileExecution(t, loadedModel(t, nil), "", limits)
			if err == nil || !strings.Contains(err.Error(), tc.want) {
				t.Fatalf("got %v, want %s", err, tc.want)
			}
		})
	}
}

func TestExecutionSnapshotBindsContextWithoutChangingRobotPins(t *testing.T) {
	model := loadedModel(t, nil)
	first, err := compileExecution(t, model, "", executionFixture())
	if err != nil {
		t.Fatal(err)
	}
	limits := executionFixture()
	axis := limits["J1"]
	axis.UpperRad = 1.5
	limits["J1"] = axis
	second, err := compileExecution(t, model, "", limits)
	if err != nil {
		t.Fatal(err)
	}
	if first.Identity != second.Identity {
		t.Fatal("native limits changed calibration/robot pins")
	}
	for _, name := range []string{ContextIdentity, ContextPlanner, ContextURDF} {
		if bytes.Equal(first.Files[name], second.Files[name]) {
			t.Fatalf("%s did not bind the changed native envelope", name)
		}
	}
	var identity map[string]any
	if err := json.Unmarshal(first.Files[ContextIdentity], &identity); err != nil {
		t.Fatal(err)
	}
	if identity["execution_limits_sha256"] != plannerOf(t, first).ExecutionLimitsSHA256 {
		t.Fatal("identity/planner snapshot mismatch")
	}
	settings, err := Resolve(model, nil, SourceEmpty)
	if err != nil {
		t.Fatal(err)
	}
	offline, err := Compile(model, settings)
	if err != nil {
		t.Fatal(err)
	}
	nilSnapshot, err := Compile(model, settings, nil)
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(offline.Files[ContextPlanner], nilSnapshot.Files[ContextPlanner]) {
		t.Fatal("nil native snapshot changed offline planning")
	}
}

func TestExecutionSnapshotBindsEvenLimitsAbovePlanningPolicy(t *testing.T) {
	model := loadedModel(t, nil)
	limits := executionFixture()
	axis := limits["J1"]
	axis.VelocityRadS, axis.AccelerationRadS2 = 1, nil
	limits["J1"] = axis
	first, err := compileExecution(t, model, "", limits)
	if err != nil {
		t.Fatal(err)
	}
	axis.VelocityRadS = 2
	limits["J1"] = axis
	second, err := compileExecution(t, model, "", limits)
	if err != nil {
		t.Fatal(err)
	}
	a, b := plannerOf(t, first), plannerOf(t, second)
	if a.Limits["J1"] != (PlannerRate{0.09, 0.09, 2.1}) || a.Limits["J1"] != b.Limits["J1"] {
		t.Fatal("an absent native acceleration cap or wider velocity changed planning policy")
	}
	if a.ExecutionLimitsSHA256 == b.ExecutionLimitsSHA256 || bytes.Equal(first.Files[ContextIdentity], second.Files[ContextIdentity]) {
		t.Fatal("distinct native snapshots alias even though planning rates are the same")
	}
}

func TestRosie1420NativeJ6BoundsExcludeRejectedLoadSample(t *testing.T) {
	model, err := At("../../../../robot_description/robots").Load("rosie_1420_v1")
	if err != nil {
		t.Fatal(err)
	}
	limits := ExecutionLimits{}
	for _, name := range model.Config.Driven {
		joint := model.URDF.Joints[name]
		acceleration := 0.5
		limits[name] = ExecutionAxisLimit{PositionUnit: "rad", LowerRad: joint.Lower, UpperRad: joint.Upper,
			VelocityRadS: joint.Velocity, AccelerationRadS2: &acceleration}
	}
	j6 := limits["J6"]
	j6.LowerRad = -2.094395102
	j6.UpperRad = 2.094395102
	limits["J6"] = j6
	settings, err := Resolve(model, nil, SourceEmpty)
	if err != nil {
		t.Fatal(err)
	}
	// The dense wire may carry spare held columns; they are not native joints.
	settings.Held = map[string]float64{"J7": 0, "J8": 0, "J9": 0}
	context, err := Compile(model, settings, limits)
	if err != nil {
		t.Fatal(err)
	}
	planner := plannerOf(t, context)
	const rejectedPosition = -2.094446759543233
	if rejectedPosition >= planner.Bounds["J6"].LowerRad {
		t.Fatal("the rejected sample remains legal to the planner")
	}
	if planner.Limits["J6"].AccelerationRadS2 != 0.5 {
		t.Fatal("native acceleration was dropped")
	}
	urdf, err := ParseURDF(string(context.Files[ContextURDF]))
	if err != nil {
		t.Fatal(err)
	}
	if urdf.Joints["J6"].Lower != -2.094395102 || urdf.Joints["J6"].Upper != 2.094395102 {
		t.Fatal("real URDF kept the wider J6 model range")
	}
}
