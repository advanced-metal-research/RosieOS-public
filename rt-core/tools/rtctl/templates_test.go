package rtctl

import (
	"encoding/json"
	"go/ast"
	"go/parser"
	gotoken "go/token"
	"os"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"
	"testing"
)

// TestConfigurationTemplateFieldCoverage reads compiler declarations rather than
// a copied schema, preventing newly read fields and undocumented template keys
// from silently diverging. Dynamic startup keys are defined by startup_schema.
func TestConfigurationTemplateFieldCoverage(t *testing.T) {
	readFields, known := map[string]bool{}, map[string]bool{}
	for _, name := range []string{"compiler.go", "render.go", "robot_definition.go", "joint_units.go", "cell_io.go", "bus.go", "brake_override.go", "collision_watchdog.go", "link_timing.go", "reference_verified_delta.go", "drive_supervision.go", "tracking_lag.go", "jog_jerk.go", "drive_alarm_reset.go"} {
		file, err := parser.ParseFile(gotoken.NewFileSet(), name, nil, 0)
		if err != nil {
			t.Fatal(err)
		}
		ast.Inspect(file, func(n ast.Node) bool {
			// Table-driven readers declare their JSON key in the first column.
			if table, ok := n.(*ast.CompositeLit); ok {
				if array, ok := table.Type.(*ast.ArrayType); ok {
					if row, ok := array.Elt.(*ast.StructType); ok && len(row.Fields.List) != 0 && len(row.Fields.List[0].Names) != 0 {
						column := row.Fields.List[0].Names[0].Name
						if column == "key" || column == "name" || column == "input" {
							for _, entry := range table.Elts {
								if entry, ok := entry.(*ast.CompositeLit); ok && len(entry.Elts) != 0 {
									if literal, ok := entry.Elts[0].(*ast.BasicLit); ok && literal.Kind == gotoken.STRING {
										key, _ := strconv.Unquote(literal.Value)
										if name == "cell_io.go" {
											key += "_unverified"
										}
										if regexp.MustCompile(`^[a-z][a-z0-9_]+$`).MatchString(key) {
											readFields[key] = true
										}
									}
								}
							}
						}
					}
				}
			}
			// Hand-written field switches/comparisons are readers too. Collect
			// only their field names, never arbitrary words from error messages.
			if binary, ok := n.(*ast.BinaryExpr); ok {
				if selector, ok := binary.X.(*ast.SelectorExpr); ok && selector.Sel.Name == "key" {
					if literal, ok := binary.Y.(*ast.BasicLit); ok && literal.Kind == gotoken.STRING {
						key, _ := strconv.Unquote(literal.Value)
						known[key] = true
					}
				}
			}
			if statement, ok := n.(*ast.SwitchStmt); ok {
				if selector, ok := statement.Tag.(*ast.SelectorExpr); ok && selector.Sel.Name == "key" {
					for _, clause := range statement.Body.List {
						for _, expression := range clause.(*ast.CaseClause).List {
							if literal, ok := expression.(*ast.BasicLit); ok && literal.Kind == gotoken.STRING {
								key, _ := strconv.Unquote(literal.Value)
								known[key] = true
							}
						}
					}
				}
			}
			call, ok := n.(*ast.CallExpr)
			if !ok || len(call.Args) == 0 {
				return true
			}
			if selector, ok := call.Fun.(*ast.SelectorExpr); ok && (selector.Sel.Name == "get" || selector.Sel.Name == "optional") {
				if literal, ok := call.Args[0].(*ast.BasicLit); ok && literal.Kind == gotoken.STRING {
					key, _ := strconv.Unquote(literal.Value)
					readFields[key] = true
				}
			}
			if function, ok := call.Fun.(*ast.Ident); ok {
				switch function.Name {
				case "robotFields", "busFields", "knownIOFields", "knownProfileFields", "knownFields", "brakeValue", "jogValue":
					for i, arg := range call.Args {
						if (function.Name == "brakeValue" || function.Name == "jogValue") && i != 0 {
							continue
						}
						if literal, ok := arg.(*ast.BasicLit); ok && literal.Kind == gotoken.STRING {
							value, _ := strconv.Unquote(literal.Value)
							for _, key := range strings.Fields(value) {
								readFields[key] = true
							}
						}
					}
				}
			}
			return true
		})
	}
	// Cell bindings have a typed reader; every JSON tag is part of the template.
	file, err := parser.ParseFile(gotoken.NewFileSet(), "../../config/cells/compatibility.go", nil, 0)
	if err != nil {
		t.Fatal(err)
	}
	ast.Inspect(file, func(n ast.Node) bool {
		if field, ok := n.(*ast.Field); ok && field.Tag != nil {
			tag, _ := strconv.Unquote(field.Tag.Value)
			if strings.HasPrefix(tag, `json:"`) {
				key := strings.Split(strings.TrimPrefix(tag, `json:"`), `"`)[0]
				readFields[key], known[key] = true, true
			}
		}
		return true
	})
	// robot_motor_rpm is compiler-produced from the URDF, not a user field.
	delete(readFields, "robot_motor_rpm")
	// Unverified metadata convention: these labels are accepted without changing
	// native behavior; the original JSON still participates in configuration identity.
	for _, key := range []string{"id", "label", "name", "runtime", "note", "source", "schema_version", "sim_setting", "maximum_acquisition_duration_ns"} {
		known[key] = true
	}
	for key := range readFields {
		known[key] = true
	}
	// Drive tuning catalogs define the other open dictionary of startup and
	// observe keys.
	catalogs, err := filepath.Glob("../../config/drives/*.tuning.json")
	if err != nil || len(catalogs) == 0 {
		t.Fatal("no drive tuning catalog", err)
	}
	for _, path := range catalogs {
		var catalog struct {
			Parameters map[string]any `json:"parameters"`
		}
		if data, err := os.ReadFile(path); err != nil || json.Unmarshal(data, &catalog) != nil {
			t.Fatal(path, err)
		}
		for key := range catalog.Parameters {
			known[key] = true
		}
	}
	// startup_schema defines an open dictionary of named settings. The same
	// names are accepted in defaults and per-axis overrides.
	for _, layer := range []string{"drive", "machine"} {
		value := readTemplate(t, layer)
		if schema, ok := value["startup_schema"].(map[string]any); ok {
			for key := range schema {
				if !strings.HasSuffix(key, "_doc") {
					known[key] = true
				}
			}
		}
	}
	present := map[string]bool{}
	for _, layer := range []string{"drive", "robot", "machine", "cell"} {
		var visit func(any)
		visit = func(value any) {
			switch value := value.(type) {
			case map[string]any:
				for key, child := range value {
					if strings.HasSuffix(key, "_doc") {
						if _, exists := value[strings.TrimSuffix(key, "_doc")]; !exists {
							t.Errorf("%s: annotation %s has no field", layer, key)
						}
						continue
					}
					present[key] = true
					doc, ok := value[key+"_doc"].(string)
					for _, contract := range []string{"Unit:", "Allowed:", "Default:", "Prevents"} {
						if !ok || !strings.Contains(doc, contract) {
							t.Errorf("%s.%s lacks %s annotation", layer, key, contract)
						}
					}
					annotation := strings.HasPrefix(key, "_") || strings.HasSuffix(key, "_source") || strings.HasSuffix(key, "_unverified") || strings.HasSuffix(key, "_note")
					if !known[key] && !annotation {
						t.Errorf("%s: template field %q is not declared in compiler readers", layer, key)
					}
					visit(child)
				}
			case []any:
				for _, child := range value {
					visit(child)
				}
			}
		}
		visit(readTemplate(t, layer))
	}
	for field := range readFields {
		if !present[field] {
			t.Errorf("compiler field %q missing from templates", field)
		}
	}
}

func readTemplate(t *testing.T, layer string) map[string]any {
	t.Helper()
	data, err := os.ReadFile(filepath.Join(testRoot(t), "config/templates", layer+".json"))
	if err != nil {
		t.Fatal(err)
	}
	var value map[string]any
	if err := json.Unmarshal(data, &value); err != nil {
		t.Fatal(err)
	}
	return value
}

// TestConfigurationTemplateStrictObjects compares each closed object with its
// own field declaration, preventing a valid field from moving to the wrong layer.
func TestConfigurationTemplateStrictObjects(t *testing.T) {
	robot, machine, drive := readTemplate(t, "robot"), readTemplate(t, "machine"), readTemplate(t, "drive")
	at := func(root map[string]any, path ...string) map[string]any {
		var value any = root
		for _, key := range path {
			if values, ok := value.([]any); ok {
				value = values[0]
			}
			value = value.(map[string]any)[key]
		}
		if values, ok := value.([]any); ok {
			value = values[0]
		}
		return value.(map[string]any)
	}
	objects := map[string]map[string]any{
		"definition": robot, "calibration": at(robot, "calibration"), "defaults": at(robot, "defaults"),
		"joint": at(robot, "joints"), "ratio": at(robot, "joints", "gear_ratio"), "l": at(robot, "joints", "mechanical_travel"),
		"deltaRestore": at(machine, "axes", "reference_verified_delta_restore"), "ref": at(machine, "robot"), "pin": at(machine, "robot", "machine_planning_calibration"), "desc": at(machine, "robot", "robot_description"), "cellLimits": at(machine, "axes", "limits"), "bus": at(machine, "bus"), "slave": at(machine, "bus", "unused_slaves"),
		"tolerance": at(drive, "encoder_battery_absent_alarm_tolerance"),
	}
	for _, file := range []string{"robot_definition.go", "cell_io.go", "bus.go", "compiler.go", "reference_verified_delta.go"} {
		source, err := parser.ParseFile(gotoken.NewFileSet(), file, nil, 0)
		if err != nil {
			t.Fatal(err)
		}
		ast.Inspect(source, func(n ast.Node) bool {
			call, ok := n.(*ast.CallExpr)
			if !ok || len(call.Args) < 2 {
				return true
			}
			function, ok := call.Fun.(*ast.Ident)
			if !ok || (function.Name != "robotFields" && function.Name != "knownIOFields" && function.Name != "busFields" && function.Name != "knownFields") {
				return true
			}
			receiver, ok := call.Args[0].(*ast.Ident)
			if !ok {
				t.Fatal("field declaration needs a mapped template object")
			}
			fields := map[string]bool{}
			for _, arg := range call.Args[1:] {
				literal, ok := arg.(*ast.BasicLit)
				if !ok {
					t.Fatal("field declaration is no longer literal")
				}
				text, _ := strconv.Unquote(literal.Value)
				for _, key := range strings.Fields(text) {
					fields[key] = true
				}
			}
			value := objects[receiver.Name]
			if function.Name == "knownFields" && file == "compiler.go" && receiver.Name == "block" {
				// The literal compiler declarations keep each closed guard object
				// distinct; no global key allowlist can hide a misplaced member.
				if fields["target_stationary_ms"] {
					value = at(machine, "axes", "brake_transition_guard")
				} else if fields["max_displacement_counts"] && fields["max_velocity_counts_per_s"] && fields["timeout_ms"] {
					value = at(machine, "axes", "brake_opening_guard")
				}
			}
			if function.Name == "knownIOFields" {
				switch receiver.Name {
				case "block":
					value = at(machine, "io")
				case "indicator":
					value = at(machine, "io")["outputs"].([]any)[2].(map[string]any)["indicator"].(map[string]any)
				case "o":
					if fields["safe_state"] {
						// The indicator example carries the complete output field set.
						value = at(machine, "io")["outputs"].([]any)[2].(map[string]any)
					} else {
						value = at(machine, "io", "inputs")
					}
				}
			}
			// Flat and robot-bound axes are alternate classes in the same template.
			// Their union is covered by the reader scan and both compilation tests.
			if receiver.Name == "cell" {
				return true
			}
			if value == nil {
				t.Fatalf("unmapped %s object %s", file, receiver.Name)
			}
			for key := range fields {
				if _, ok := value[key]; !ok {
					t.Errorf("%s.%s missing %s", file, receiver.Name, key)
				}
			}
			for key := range value {
				if strings.HasSuffix(key, "_doc") || strings.HasPrefix(key, "_") || strings.HasSuffix(key, "_source") || strings.HasSuffix(key, "_unverified") || strings.HasSuffix(key, "_note") {
					continue
				}
				if !fields[key] {
					t.Errorf("%s.%s does not accept %s", file, receiver.Name, key)
				}
			}
			return true
		})
	}
}

// templateValue removes documentation before selecting a concrete branch;
// annotations are class documentation and never deployment authority.
func templateValue(value any) any {
	switch value := value.(type) {
	case map[string]any:
		out := map[string]any{}
		for key, child := range value {
			if !strings.HasSuffix(key, "_doc") {
				out[key] = templateValue(child)
			}
		}
		return out
	case []any:
		out := make([]any, len(value))
		for i, child := range value {
			out[i] = templateValue(child)
		}
		return out
	}
	return value
}

// TestConfigurationTemplateSimulation compiles the flat simulation branch of
// the machine class and its drive class. Robot-only and hardware-only options
// are excluded explicitly so documentation cannot imply hardware qualification.
func TestConfigurationTemplateSimulation(t *testing.T) {
	machine := templateValue(readTemplate(t, "machine")).(map[string]any)
	delete(machine, "robot")
	delete(machine, "io")
	delete(machine, "bus")
	delete(machine, "max_motor_rpm")
	delete(machine, "verify_parameter_storage")
	axes := machine["axes"].([]any)
	axis := axes[0].(map[string]any)
	for _, key := range []string{"robot_joint", "encoder_battery", "brake_override", "diagnostics", "collision_watchdog", "startup", "startup_trial", "observe", "brake", "brake_transition_guard", "brake_opening_guard", "reference_verified_delta_restore"} {
		delete(axis, key)
	}
	axis["profile"], axis["require_home"] = "cia402_sim", false
	axis["coordinate_evidence_mode"] = "multi_turn"
	machine["backend"] = "simulation"
	machine["axes"] = []any{axis}
	raw, err := json.Marshal(machine)
	if err != nil {
		t.Fatal(err)
	}
	compiled, err := Compile(raw, filepath.Join(testRoot(t), "config/drives"))
	if err != nil {
		t.Fatal(err)
	}
	if len(compiled.CoordinateIdentitySHA256) != 1 {
		t.Fatal("simulation template must compile exactly one axis")
	}
	// The drive class also contains mutually exclusive sampled/PDO evidence and
	// terminal descriptors. Compile its CiA402 branch against the same machine.
	drive := templateValue(readTemplate(t, "drive")).(map[string]any)
	for _, key := range []string{"coordinate_evidence", "collision_watchdog", "absolute_feedback", "absolute_pair_field", "startup_defaults", "startup_schema", "native_home"} {
		delete(drive, key)
	}
	drive["brake"] = map[string]any{"present": false}
	directory := t.TempDir()
	data, err := json.Marshal(drive)
	if err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(directory, "cia402_sim.json"), data, 0600); err != nil {
		t.Fatal(err)
	}
	if _, err := Compile(raw, directory); err != nil {
		t.Fatal(err)
	}
}

// The annotated guard objects must compile when explicitly selected on a
// fixture with a brake and velocity feedback; the default flat branch above
// deliberately omits both. This validates example values as well as key parity.
func TestConfigurationTemplateBrakeGuards(t *testing.T) {
	machine := templateValue(readTemplate(t, "machine")).(map[string]any)
	axis := machine["axes"].([]any)[0].(map[string]any)
	f := transitionFixture()
	for _, key := range []string{"brake_transition_guard", "brake_opening_guard"} {
		raw, err := json.Marshal(axis[key])
		if err != nil {
			t.Fatal(err)
		}
		f.put("machine.axes.0."+key, string(raw))
	}
	compiled, err := f.compile()
	if err != nil {
		t.Fatalf("explicit documented guard branch failed: %v", err)
	}
	for _, line := range []string{"brake_transition=327|6553|100|100|500\n", "brake_opening=327|13107|500\n"} {
		if !strings.Contains(compiled.AxisProfiles, line) {
			t.Fatalf("documented guard values changed or were not compiled: %q", line)
		}
	}
}

// TestConfigurationTemplateRobot validates the external-joint simulation branch
// with the real robot reader; model annotations cannot bypass pin verification.
func TestConfigurationTemplateRobot(t *testing.T) {
	f := newRobotFixture(t, false)
	limits := asObject(f.axis().get("limits"))
	limits.set("max_velocity", json.Number("0.25"))
	limits.set("jog_acceleration", json.Number("1"))
	f.setAxis("limits", limits)
	definition := templateValue(readTemplate(t, "robot")).(map[string]any)
	definition["robot_id"], definition["name"] = "fixture", "Template simulation"
	// The template's one joint is external: no geometry to map it to.
	if err := os.Remove(f.urdf); err != nil {
		t.Fatal(err)
	}
	defaults := definition["defaults"].(map[string]any)
	defaults["drive_profile"], defaults["home_policy"], defaults["coordinate_evidence_mode"] = "test", "optional", "multi_turn"
	joint := definition["joints"].([]any)[0].(map[string]any)
	joint["name"], joint["urdf_joint"], joint["kind"], joint["cartesian_participates"] = "axis_0", nil, "external", false
	joint["drive_profile"] = nil
	delete(joint, "encoder_battery")
	delete(joint, "encoder_battery_source")
	data, err := json.Marshal(definition)
	if err != nil {
		t.Fatal(err)
	}
	f.definition = asObject(decode(data))
	f.bind(t)
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	if compiled.Robot == nil || len(compiled.Robot.Axes) != 1 {
		t.Fatal("robot template did not bind exactly one joint")
	}
}
