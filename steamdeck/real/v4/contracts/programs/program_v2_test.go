package programs

// robot.v4.program.v2: the pre-compile weld program shared by Offline
// Programming's classic flow and the weld planner's rails flow, and the
// successor of robot.v4.program.v1. This file is the Go half of its contract:
// a hand-rolled validator in the style of validateProgram (there is no JSON
// Schema library in this module, so every rule the schema states is restated
// here by hand, closed objects included), the fixtures that pin it, and a
// parity test that holds the vendored seam vocabulary equal to the weld
// planner's own schema. The Python twin is weld_planner/v1/python/weldplan/
// program_v2.py and the TypeScript twin is offline-programming/v1/ui/src/
// contracts/robot-v4-program-v2.ts; the three assert the same fixtures.

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"reflect"
	"regexp"
	"strings"
	"testing"
	"time"
)

const programV2Schema = "robot.v4.program.v2"

var sha256HexPattern = regexp.MustCompile(`^[0-9a-f]{64}$`)

// Solved quantities, anywhere in the document. Each is the name of a solved
// thing in another contract: program.v1 rows, the accepted plan, the dense
// trajectory, the motion-plan result. Exact key match, recursive.
var programV2ForbiddenKeys = stringSet(
	"joints", "joint_positions_rad", "target_joints_rad", "q_rad", "qd_rad_s",
	"target_tcp_m", "tcp_pose_m", "trajectory", "planned_path", "accepted_plan",
	"preloaded_plan", "waypoints", "rows",
)

// Legitimate seam or classic vocabulary on a weld node; under a freespace
// node or defaults they can only be a path.
var programV2FreespaceForbiddenKeys = stringSet(
	"knots", "knots_u", "times_s", "control_points_xyz_m", "segments", "samples",
	"points", "poses", "rpy_rad", "path",
)

var programV2PolicyKeys = stringSet("clearance_min_mm", "speed_scale", "torch_policy", "positioner_policy")
var programV2CommonKeys = stringSet("id", "op", "enabled", "label", "comment", "row_hash", "extensions")

var programV2NodeKeys = map[string]map[string]bool{
	"move":     union(programV2CommonKeys, stringSet("motion", "target_space", "target", "via", "speed_scale", "speed_mm_s", "acceleration_scale", "constant_tcp_speed", "capture", "weld_transition")),
	"weld":     union(programV2CommonKeys, stringSet("geometry", "travel_speed_mm_s", "standoff_mm", "weave", "weld_preset_id", "weld_parameters")),
	"approach": union(programV2CommonKeys, stringSet("depart_standoff", "arrive_standoff", "overrides")),
	"transit":  union(programV2CommonKeys, stringSet("depart_standoff", "arrive_standoff", "overrides")),
	"retract":  union(programV2CommonKeys, stringSet("depart_standoff", "arrive_standoff", "overrides")),
	"home":     union(programV2CommonKeys, stringSet("depart_standoff", "arrive_standoff", "overrides")),
	"dwell":    union(programV2CommonKeys, stringSet("duration_s")),
	"io":       union(programV2CommonKeys, stringSet("channel", "state", "action", "duration_s", "timeout_s", "stable_s")),
}

var programV2Fixtures = []string{
	"complete-program-v2.json",
	"invalid-program-v2-freespace-solution.json",
	"invalid-program-v2-dense-joints.json",
	"invalid-program-v2-weld-parameters-filled.json",
	"invalid-program-v2-adjacent-welds.json",
	"invalid-program-v2-dangling-joint-ref.json",
	"invalid-program-v2-search-bin-wider-than-window.json",
}

func TestProgramV2CompleteFixtureIsAcceptedAndCoversEveryOp(t *testing.T) {
	t.Parallel()
	doc := loadProgramV2(t)
	if err := validateProgramV2(doc); err != nil {
		t.Fatal(err)
	}
	ops := map[string]bool{}
	kinds := map[string]bool{}
	for _, raw := range doc["nodes"].([]any) {
		node := raw.(map[string]any)
		ops[node["op"].(string)] = true
		if geometry, ok := node["geometry"].(map[string]any); ok {
			kinds[geometry["kind"].(string)] = true
		}
	}
	for _, op := range []string{"weld", "approach", "transit", "retract", "home", "dwell", "io"} {
		if !ops[op] {
			t.Errorf("complete fixture is missing a %s node", op)
		}
	}
	for _, kind := range []string{"seam", "posed_polyline", "manual_pose_wp"} {
		if !kinds[kind] {
			t.Errorf("complete fixture is missing a %s weld", kind)
		}
	}
}

func TestProgramV2InvalidFixturesFailClosedNamingTheKey(t *testing.T) {
	t.Parallel()
	for name, want := range map[string]string{
		"invalid-program-v2-freespace-solution.json":           "trajectory",
		"invalid-program-v2-dense-joints.json":                 "q_rad",
		"invalid-program-v2-weld-parameters-filled.json":       "weld_parameters",
		"invalid-program-v2-adjacent-welds.json":               "transit is missing",
		"invalid-program-v2-dangling-joint-ref.json":           "not in workpiece.weld_joints",
		"invalid-program-v2-search-bin-wider-than-window.json": "which is one bin",
	} {
		var doc map[string]any
		decodeFixture(t, filepath.Join("fixtures", name), &doc)
		err := validateProgramV2(doc)
		if err == nil || !strings.Contains(err.Error(), want) {
			t.Errorf("%s: want an error naming %q, got %v", name, want, err)
		}
	}
}

func TestProgramV2TheLineMustAlternateAndTheRestIsRefused(t *testing.T) {
	t.Parallel()
	// Node indexes in the complete fixture:
	// 0 io, 1 home, 2 approach, 3 weld(seam), 4 transit, 5 io,
	// 6 weld(posed_polyline), 7 dwell, 8 transit, 9 weld(manual_pose_wp),
	// 10 retract, 11 home, 12 io.
	cases := []struct {
		name   string
		mutate func(doc map[string]any)
		want   string // "" means the mutation must still validate
	}{
		{"two welds with nothing between them", func(d map[string]any) { removeNode(d, 4) }, "transit is missing"},
		{"a weld with no approach", func(d map[string]any) { removeNode(d, 2); removeNode(d, 1) }, "approach is missing"},
		{"home in the middle", func(d map[string]any) { insertNode(d, 5, cloneNode(d, 1, "home_mid")) }, "middle"},
		{"two freespace nodes adjacent", func(d map[string]any) { insertNode(d, 11, cloneNode(d, 10, "retract_again")) }, "only a home or nothing may follow"},
		{"a transit not between welds", func(d map[string]any) { insertNode(d, 2, cloneNode(d, 4, "transit_early")) }, "between two welds"},
		{"events between approach and weld are transparent", func(d map[string]any) {
			insertNode(d, 3, cloneNode(d, 7, "dwell_early"))
			insertNode(d, 3, cloneNode(d, 0, "io_early"))
		}, ""},
		{"a solved path on a transit", func(d map[string]any) { nodeAt(d, 4)["trajectory"] = []any{} }, "trajectory"},
		{"classic points under a transit", func(d map[string]any) { nodeAt(d, 4)["points"] = []any{} }, "points"},
		{"dense joints on a weld", func(d map[string]any) { nodeAt(d, 3)["q_rad"] = []any{} }, "q_rad"},
		{"a field the schema does not name", func(d map[string]any) { nodeAt(d, 3)["speed_hint"] = 1.0 }, "speed_hint"},
		{"freespace policy on a weld", func(d map[string]any) {
			nodeAt(d, 3)["overrides"] = map[string]any{"positioner_policy": "hold"}
		}, "overrides"},
		{"an override that is not a defaults key", func(d map[string]any) {
			nodeAt(d, 4)["overrides"] = map[string]any{"max_qd": 1.0}
		}, "not a defaults key"},
		{"a depart standoff on an approach", func(d map[string]any) {
			nodeAt(d, 2)["depart_standoff"] = nodeAt(d, 10)["depart_standoff"]
		}, "nothing is welded before"},
		{"weld_parameters filled", func(d map[string]any) {
			nodeAt(d, 6)["weld_parameters"] = map[string]any{"voltage_v": 24.0}
		}, "reserved"},
		{"manual poses copied onto each other", func(d map[string]any) {
			points := nodeAt(d, 9)["geometry"].(map[string]any)["points"].([]any)
			points[1] = deepCopy(points[0])
		}, "coincide"},
		{"manual poses outside the workpiece frame", func(d map[string]any) {
			nodeAt(d, 9)["geometry"].(map[string]any)["frame_id"] = "world"
		}, "is not the workpiece frame"},
		{"manual poses as an arc through the L", func(d map[string]any) {
			nodeAt(d, 9)["geometry"].(map[string]any)["shape"] = "arc"
		}, ""},
		{"an arc through points on a line", func(d map[string]any) {
			geometry := nodeAt(d, 9)["geometry"].(map[string]any)
			geometry["shape"] = "arc"
			point := geometry["points"].([]any)[1].(map[string]any)
			point["xyz_m"] = []any{-0.07, 0.0425, 0.153}
		}, "lie on a line"},
		{"an arc of four points", func(d map[string]any) {
			geometry := nodeAt(d, 9)["geometry"].(map[string]any)
			geometry["shape"] = "arc"
			points := geometry["points"].([]any)
			extra := deepCopy(points[2]).(map[string]any)
			extra["xyz_m"] = []any{-0.02, 0.0225, 0.153}
			geometry["points"] = append(points, extra)
		}, "exactly three"},
		{"a manual pose with no continuity", func(d map[string]any) {
			point := nodeAt(d, 9)["geometry"].(map[string]any)["points"].([]any)[1].(map[string]any)
			delete(point, "continuity")
		}, "continuity"},
		{"weld_parameters absent", func(d map[string]any) { delete(nodeAt(d, 6), "weld_parameters") }, "weld_parameters"},
		{"a knot vector one short", func(d map[string]any) {
			segment := seamSegment(d, 3, 0)
			knots := segment["knots_u"].([]any)
			segment["knots_u"] = knots[:len(knots)-1]
		}, "knots_u"},
		{"a wrapping span on an open curve", func(d map[string]any) {
			geometry := nodeAt(d, 3)["geometry"].(map[string]any)
			geometry["curve"].(map[string]any)["closed"] = false
			geometry["span"] = map[string]any{"start_s": 0.8, "end_s": 0.2}
		}, "closed curve"},
		{"a piecewise_linear profile on a weld is seam vocabulary", func(d map[string]any) {
			nodeAt(d, 3)["travel_speed_mm_s"] = map[string]any{
				"kind":  "piecewise_linear",
				"knots": []any{[]any{0.0, 8.0}, []any{1.0, 6.0}},
			}
		}, ""},
		{"a piecewise_linear profile running backwards", func(d map[string]any) {
			nodeAt(d, 3)["travel_speed_mm_s"] = map[string]any{
				"kind":  "piecewise_linear",
				"knots": []any{[]any{0.0, 8.0}, []any{0.5, 6.0}, []any{0.4, 6.0}, []any{1.0, 6.0}},
			}
		}, "strictly increasing"},
		// A standoff is what the planner places the tip from. Absent is not
		// zero, and zero is the tip inside the part -- so both are refused
		// here rather than at plan time, where the answer arrives as a
		// refused seam inside a 200.
		{"a weld with no standoff", func(d map[string]any) { delete(nodeAt(d, 3), "standoff_mm") }, "standoff_mm is required"},
		{"a standoff of zero", func(d map[string]any) {
			nodeAt(d, 3)["standoff_mm"] = map[string]any{"kind": "constant", "value": 0.0}
		}, "positive everywhere"},
		{"a standoff profile that reaches zero", func(d map[string]any) {
			nodeAt(d, 3)["standoff_mm"] = map[string]any{
				"kind":  "piecewise_linear",
				"knots": []any{[]any{0.0, 12.0}, []any{1.0, 0.0}},
			}
		}, "positive everywhere"},
		{"a standoff profile that varies along the weld", func(d map[string]any) {
			nodeAt(d, 3)["standoff_mm"] = map[string]any{
				"kind":  "piecewise_linear",
				"knots": []any{[]any{0.0, 12.0}, []any{1.0, 9.0}},
			}
		}, ""},
		{"a duplicate node id", func(d map[string]any) { nodeAt(d, 7)["id"] = nodeAt(d, 5)["id"] }, "used twice"},
		{"an empty planner_owns", func(d map[string]any) { d["planner_owns"] = []any{} }, "planner_owns"},
		{"a robot without an identity", func(d map[string]any) { d["robot"] = map[string]any{"model_id": "rosie_1400_v3"} }, "robot.robot_description_sha256"},
		{"a robot whose placement names another", func(d map[string]any) {
			d["placement"].(map[string]any)["cell_id"] = "rosie_1420_v1"
		}, "placement.cell_id"},
		{"too many nodes", func(d map[string]any) {
			nodes := d["nodes"].([]any)
			for len(nodes) <= 4096 {
				nodes = append(nodes, nodes[len(nodes)-1])
			}
			d["nodes"] = nodes
		}, "4096"},
	}
	for _, tc := range cases {
		tc := tc
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := loadProgramV2(t)
			tc.mutate(doc)
			err := validateProgramV2(doc)
			switch {
			case tc.want == "" && err != nil:
				t.Fatalf("must still validate, got: %v", err)
			case tc.want != "" && (err == nil || !strings.Contains(err.Error(), tc.want)):
				t.Fatalf("want an error naming %q, got %v", tc.want, err)
			}
		})
	}
}

func TestProgramV2SchemaIsSelfContained(t *testing.T) {
	t.Parallel()
	var schema map[string]any
	decodeFixture(t, "robot.v4.program.v2.schema.json", &schema)
	if schema["$schema"] != "https://json-schema.org/draft/2020-12/schema" {
		t.Fatalf("unexpected $schema: %v", schema["$schema"])
	}
	defs := schema["$defs"].(map[string]any)
	var walk func(value any, path string)
	walk = func(value any, path string) {
		switch typed := value.(type) {
		case map[string]any:
			for key, child := range typed {
				if key == "$ref" {
					ref, _ := child.(string)
					if !strings.HasPrefix(ref, "#/$defs/") {
						t.Errorf("%s: non-local $ref %q; the contracts directory is read by path from four languages and must stay self-contained", path, ref)
					} else if _, ok := defs[strings.TrimPrefix(ref, "#/$defs/")]; !ok {
						t.Errorf("%s: dangling $ref %q", path, ref)
					}
					continue
				}
				walk(child, path+"."+key)
			}
		case []any:
			for index, child := range typed {
				walk(child, fmt.Sprintf("%s[%d]", path, index))
			}
		}
	}
	walk(schema, "$")
}

func TestProgramV2VendoredSeamVocabularyMatchesTheWeldPlanner(t *testing.T) {
	t.Parallel()
	sourcePath := filepath.Join("..", "..", "..", "..", "..", "weld_planner", "v1", "schema",
		"amr-weld-planner-v1.weld-program.v1.schema.json")
	raw, err := os.ReadFile(sourcePath)
	if err != nil {
		t.Skipf("weld planner schema not present at %s: %v", sourcePath, err)
	}
	var source map[string]any
	if err := json.Unmarshal(raw, &source); err != nil {
		t.Fatalf("decode weld planner schema: %v", err)
	}
	var schema map[string]any
	decodeFixture(t, "robot.v4.program.v2.schema.json", &schema)
	defs := schema["$defs"].(map[string]any)
	sourceDefs := source["$defs"].(map[string]any)
	sourceProps := source["properties"].(map[string]any)

	// $defs/seam minus process, which program.v2 splits into the node's
	// travel_speed_mm_s, weave and reserved weld_parameters.
	seam := deepCopy(sourceDefs["seam"]).(map[string]any)
	delete(seam["properties"].(map[string]any), "process")
	var required []any
	for _, key := range seam["required"].([]any) {
		if key != "process" {
			required = append(required, key)
		}
	}
	seam["required"] = required

	// One deliberate difference, D24: v1 carries weld_joints at top
	// level; program.v2 carries it INSIDE the workpiece, because a joint is an
	// objective fact about the part and that is what workpiece already holds.
	workpiece := deepCopy(sourceProps["workpiece"]).(map[string]any)
	workpiece["properties"].(map[string]any)["weld_joints"] = sourceProps["weld_joints"]

	expected := map[string]any{
		"identifier":             sourceDefs["identifier"],
		"vec3":                   sourceDefs["vec3"],
		"unit_vec3":              sourceDefs["unit_vec3"],
		"scalar_function":        sourceDefs["scalar_function"],
		"analytic":               sourceDefs["analytic"],
		"curve_segment":          sourceDefs["curve_segment"],
		"curve":                  sourceDefs["curve"],
		"orientation_limit_axis": sourceDefs["orientation_limit_axis"],
		"seam":                   seam,
		"weld_joint":             sourceDefs["weld_joint"],
		"groove_face":            sourceDefs["groove_face"],
		"weave":                  sourceDefs["seam"].(map[string]any)["properties"].(map[string]any)["process"].(map[string]any)["properties"].(map[string]any)["weave"],
		"source":                 sourceProps["source"],
		"workpiece":              workpiece,
	}
	for name, want := range expected {
		if !reflect.DeepEqual(defs[name], want) {
			t.Errorf("$defs/%s has drifted from the weld planner schema; re-vendor it, do not edit the copy", name)
		}
	}

	// geometry_seam is seam plus a kind and an inline curve, nothing else.
	geometry := defs["geometry_seam"].(map[string]any)["properties"].(map[string]any)
	seamProps := seam["properties"].(map[string]any)
	for key, want := range seamProps {
		if key == "curve" {
			continue
		}
		if !reflect.DeepEqual(geometry[key], want) {
			t.Errorf("geometry_seam.%s differs from seam.%s", key, key)
		}
	}
	for key := range geometry {
		// `search_provenance` is the seam's counterpart to `torch_angles_deg`
		// on the classic geometry: what chose the orientation, beside it. A
		// weld program does not carry it, so it is not a seam property.
		if key != "kind" && key != "curve" && key != "search_provenance" {
			if _, ok := seamProps[key]; !ok {
				t.Errorf("geometry_seam.%s is not a seam property", key)
			}
		}
	}
}

// ---------------------------------------------------------------- validator

func validateProgramV2(doc map[string]any) error {
	if value, present := doc["speed_scale"]; present {
		if number, ok := value.(float64); !ok || !(number >= 0.01 && number <= 1) {
			return fmt.Errorf("speed_scale must be in [0.01, 1]")
		}
	}
	// The joint index a seam's weld_joint_ref resolves against. A duplicated
	// id is reported rather than silently shadowed.
	joints := map[string]map[string]any{}
	if workpiece, ok := doc["workpiece"].(map[string]any); ok {
		entries, _ := workpiece["weld_joints"].([]any)
		for _, raw := range entries {
			entry, ok := raw.(map[string]any)
			if !ok {
				continue
			}
			jid, ok := entry["id"].(string)
			if !ok || jid == "" {
				continue
			}
			if _, seen := joints[jid]; seen {
				return fmt.Errorf("workpiece.weld_joints has two joints with id %q", jid)
			}
			joints[jid] = entry
		}
	}
	if doc["schema"] != programV2Schema {
		return fmt.Errorf("schema must be %s", programV2Schema)
	}
	for _, field := range []string{"program_id", "program_name", "created_utc"} {
		if !nonempty(doc[field]) {
			return fmt.Errorf("%s is required", field)
		}
	}
	if revision, ok := integerValue(doc["program_revision"]); !ok || revision < 1 {
		return fmt.Errorf("program_revision must be positive")
	}
	if base, ok := integerValue(doc["base_program_revision"]); !ok || base < 0 {
		return fmt.Errorf("base_program_revision must be non-negative")
	}
	if digest, present := doc["program_digest"]; present {
		if text, _ := digest.(string); !digestPattern.MatchString(text) {
			return fmt.Errorf("program_digest must be a sha256 digest")
		}
	}
	if !oneOf(doc["program_state"], "draft", "saved_unplanned", "needs_plan", "planning", "planned",
		"accepted_simulation", "accepted_physical", "execution_ready", "running", "completed", "stopped", "faulted") {
		return fmt.Errorf("program_state is invalid")
	}
	if !oneOf(doc["execution_mode"], "simulation", "dry_run", "production") {
		return fmt.Errorf("execution_mode is invalid")
	}
	if _, err := time.Parse(time.RFC3339Nano, doc["created_utc"].(string)); err != nil {
		return fmt.Errorf("created_utc is not RFC 3339: %w", err)
	}
	units, ok := doc["units"].(map[string]any)
	if !ok || units["length"] != "m" || units["angle"] != "rad" || units["time"] != "s" {
		return fmt.Errorf("canonical SI units are required")
	}
	if err := validateJointOrder(units["joint_order"]); err != nil {
		return err
	}
	if err := validateProgramV2Producer(doc["producer"]); err != nil {
		return err
	}
	workpiece, ok := doc["workpiece"].(map[string]any)
	if !ok || !nonempty(workpiece["frame_id"]) {
		return fmt.Errorf("workpiece.frame_id is required")
	}
	placement, ok := doc["placement"].(map[string]any)
	if !ok || !nonempty(placement["cell_id"]) {
		return fmt.Errorf("placement.cell_id is required")
	}
	if robot, _ := doc["robot"].(map[string]any); robot != nil && robot["model_id"] != placement["cell_id"] {
		return fmt.Errorf("placement.cell_id names %v and the program is for %v; the offsets are a record in one description's work frame",
			placement["cell_id"], robot["model_id"])
	}
	for _, key := range []string{"anchor_xyz_m", "offset_xyz_m", "rpy_rad"} {
		if err := validateNumberArray(placement[key], 3); err != nil {
			return fmt.Errorf("placement.%s: %w", key, err)
		}
	}
	if err := validateRobotRef(doc["robot"]); err != nil {
		return err
	}
	if tool, present := doc["tool"]; present {
		if err := validateAssetRef(tool, "tool"); err != nil {
			return err
		}
	}
	defaults, ok := doc["defaults"].(map[string]any)
	if !ok {
		return fmt.Errorf("defaults is required")
	}
	for key := range programV2PolicyKeys {
		if _, present := defaults[key]; !present {
			return fmt.Errorf("defaults.%s is required", key)
		}
	}
	if err := validatePolicy(defaults, "defaults"); err != nil {
		return err
	}
	owns, ok := doc["planner_owns"].([]any)
	if !ok || len(owns) == 0 {
		return fmt.Errorf("planner_owns must state at least one absence")
	}
	for _, sentence := range owns {
		if !nonempty(sentence) {
			return fmt.Errorf("planner_owns entries must be sentences")
		}
	}
	if err := forbiddenKeys(doc, "$", false); err != nil {
		return err
	}
	if err := forbiddenKeys(defaults, "defaults", true); err != nil {
		return err
	}

	nodes, ok := doc["nodes"].([]any)
	if !ok || len(nodes) < 1 || len(nodes) > 4096 {
		return fmt.Errorf("nodes must contain 1-4096 entries")
	}
	ids := map[string]bool{}
	type motionNode struct{ id, op string }
	var motion []motionNode
	for index, raw := range nodes {
		node, ok := raw.(map[string]any)
		if !ok {
			return fmt.Errorf("nodes[%d] must be an object", index)
		}
		id, _ := node["id"].(string)
		if strings.TrimSpace(id) == "" {
			return fmt.Errorf("nodes[%d].id is required", index)
		}
		if ids[id] {
			return fmt.Errorf("node id %q is used twice; one row, one id", id)
		}
		ids[id] = true
		op, _ := node["op"].(string)
		allowed, known := programV2NodeKeys[op]
		if !known {
			return fmt.Errorf("node %q has unknown op %q", id, op)
		}
		for key := range node {
			if !allowed[key] {
				return fmt.Errorf("node %q carries %q, which is not a field of a %s node", id, key, op)
			}
		}
		if _, ok := node["enabled"].(bool); !ok {
			return fmt.Errorf("node %q enabled must be boolean", id)
		}
		for _, key := range []string{"label", "comment"} {
			if _, ok := node[key].(string); !ok {
				return fmt.Errorf("node %q %s must be a string", id, key)
			}
		}
		if !nonempty(node["row_hash"]) {
			return fmt.Errorf("node %q row_hash is required", id)
		}
		var err error
		switch op {
		case "move":
			err = validateTaughtMove(node)
		case "weld":
			err = validateWeldNode(node, id, joints, workpiece["frame_id"])
		case "approach", "transit", "retract", "home":
			err = validateFreespaceNode(node, id, op)
		case "dwell":
			if duration, ok := node["duration_s"].(float64); !ok || duration < 0 {
				err = fmt.Errorf("dwell %q needs a non-negative duration_s", id)
			}
		case "io":
			if !nonempty(node["channel"]) {
				err = fmt.Errorf("io %q channel is required", id)
			} else if _, ok := node["state"].(bool); !ok {
				err = fmt.Errorf("io %q state must be boolean", id)
			}
		}
		if err != nil {
			return err
		}
		if op != "dwell" && op != "io" {
			motion = append(motion, motionNode{id, op})
		}
	}

	// The single line: home? approach weld (transit weld)* retract home?
	if len(motion) == 0 {
		return fmt.Errorf("the program has no motion nodes")
	}
	for k, current := range motion {
		if current.op == "home" && k != 0 && k != len(motion)-1 {
			return fmt.Errorf("home %q sits in the middle of the program; home may only start or end it", current.id)
		}
	}
	for k, current := range motion {
		prev, next := "", ""
		if k > 0 {
			prev = motion[k-1].op
		}
		if k+1 < len(motion) {
			next = motion[k+1].op
		}
		switch current.op {
		case "weld":
			if prev != "approach" && prev != "transit" {
				if prev == "" || prev == "home" {
					return fmt.Errorf("weld %q is not preceded by an approach or transit (an approach is missing)", current.id)
				}
				return fmt.Errorf("weld %q is not preceded by an approach or transit", current.id)
			}
			if next != "retract" && next != "transit" {
				if next == "weld" {
					return fmt.Errorf("weld %q is not followed by a retract or transit (a transit is missing before the next weld)", current.id)
				}
				return fmt.Errorf("weld %q is not followed by a retract or transit", current.id)
			}
		case "approach":
			if next != "weld" {
				return fmt.Errorf("approach %q is not followed by a weld", current.id)
			}
			if prev != "" && prev != "home" && prev != "move" && prev != "retract" {
				return fmt.Errorf("approach %q follows %q; only a home or nothing may precede it", current.id, prev)
			}
		case "retract":
			if prev != "weld" {
				return fmt.Errorf("retract %q is not preceded by a weld", current.id)
			}
			if next != "" && next != "home" && next != "move" && next != "approach" {
				return fmt.Errorf("retract %q is followed by %q; only a home or nothing may follow it", current.id, next)
			}
		case "transit":
			if prev != "weld" || next != "weld" {
				return fmt.Errorf("transit %q must sit between two welds", current.id)
			}
		}
	}
	return nil
}

func validateProgramV2Producer(value any) error {
	producer, ok := value.(map[string]any)
	if !ok {
		return fmt.Errorf("producer is required")
	}
	if !oneOf(producer["component"], "offline-programming-v1", "weld-planner-v1") {
		return fmt.Errorf("producer.component is invalid")
	}
	if !nonempty(producer["source_revision"]) {
		return fmt.Errorf("producer.source_revision is required")
	}
	inputs, ok := producer["inputs"].([]any)
	if !ok || len(inputs) == 0 {
		return fmt.Errorf("producer.inputs must name at least one input")
	}
	for index, raw := range inputs {
		input, ok := raw.(map[string]any)
		if !ok || !oneOf(input["kind"], "project", "weld_program", "cad", "cell", "tool") {
			return fmt.Errorf("producer.inputs[%d].kind is invalid", index)
		}
		if !nonempty(input["ref"]) {
			return fmt.Errorf("producer.inputs[%d].ref is required", index)
		}
		if hash, _ := input["sha256"].(string); !sha256HexPattern.MatchString(hash) {
			return fmt.Errorf("producer.inputs[%d].sha256 must be 64 hex characters", index)
		}
	}
	return nil
}

// The robot the program is for, and that the placement names the same one:
// the offsets are a record in that description's work frame.
func validateRobotRef(value any) error {
	ref, ok := value.(map[string]any)
	if !ok || !nonempty(ref["model_id"]) {
		return fmt.Errorf("robot.model_id is required")
	}
	if hash, _ := ref["robot_description_sha256"].(string); !digestPattern.MatchString(hash) {
		return fmt.Errorf("robot.robot_description_sha256 must be a description identity (sha256:<64 hex>); the description stays out, the identity pins it")
	}
	return nil
}

func validateAssetRef(value any, path string) error {
	ref, ok := value.(map[string]any)
	if !ok || !nonempty(ref["id"]) {
		return fmt.Errorf("%s.id is required", path)
	}
	if hash, _ := ref["sha256"].(string); !sha256HexPattern.MatchString(hash) {
		return fmt.Errorf("%s.sha256 must be 64 hex characters; the contents stay out, the hash pins them", path)
	}
	return nil
}

func validatePolicy(policy map[string]any, path string) error {
	for key := range policy {
		if !programV2PolicyKeys[key] {
			return fmt.Errorf("%s.%s is not a defaults key", path, key)
		}
	}
	if value, present := policy["clearance_min_mm"]; present {
		if number, ok := value.(float64); !ok || number < 0 {
			return fmt.Errorf("%s.clearance_min_mm must be non-negative", path)
		}
	}
	if value, present := policy["speed_scale"]; present {
		if number, ok := value.(float64); !ok || number <= 0 || number > 1 {
			return fmt.Errorf("%s.speed_scale must be in (0, 1]", path)
		}
	}
	if value, present := policy["torch_policy"]; present && !oneOf(value, "free", "hold_last", "torch_down") {
		return fmt.Errorf("%s.torch_policy is invalid", path)
	}
	if value, present := policy["positioner_policy"]; present && !oneOf(value, "free", "hold") {
		return fmt.Errorf("%s.positioner_policy is invalid", path)
	}
	return nil
}

func validateWeldNode(node map[string]any, id string, joints map[string]map[string]any, workpieceFrame any) error {
	params, present := node["weld_parameters"]
	if !present {
		return fmt.Errorf("weld %q has no weld_parameters; the reserved block must be present", id)
	}
	if filled, ok := params.(map[string]any); !ok || len(filled) != 0 {
		return fmt.Errorf("weld %q fills weld_parameters; the block is reserved and must be empty in program.v2", id)
	}
	if speed, ok := node["travel_speed_mm_s"].(map[string]any); !ok {
		return fmt.Errorf("weld %q travel_speed_mm_s is required", id)
	} else if err := validateScalarFunction(speed, fmt.Sprintf("weld %q travel_speed_mm_s", id)); err != nil {
		return err
	}
	// Never defaulted here. An absent standoff is not zero -- zero puts the tip
	// in the part -- so the document has to carry it.
	if standoff, ok := node["standoff_mm"].(map[string]any); !ok {
		return fmt.Errorf("weld %q standoff_mm is required", id)
	} else if err := validateScalarFunction(standoff, fmt.Sprintf("weld %q standoff_mm", id)); err != nil {
		return err
	} else if err := validatePositiveScalar(standoff, fmt.Sprintf("weld %q standoff_mm", id)); err != nil {
		return err
	}
	if weave, ok := node["weave"].(map[string]any); !ok || !nonempty(weave["shape"]) {
		return fmt.Errorf("weld %q weave.shape is required", id)
	}
	geometry, ok := node["geometry"].(map[string]any)
	if !ok {
		return fmt.Errorf("weld %q has no geometry", id)
	}
	switch geometry["kind"] {
	case "seam":
		return validateSeamGeometry(geometry, id, joints)
	case "posed_polyline":
		if !nonempty(geometry["frame_id"]) {
			return fmt.Errorf("weld %q posed_polyline frame_id is required", id)
		}
		points, ok := geometry["points"].([]any)
		if !ok || len(points) < 2 {
			return fmt.Errorf("weld %q posed_polyline needs at least two points", id)
		}
		for index, raw := range points {
			point, ok := raw.(map[string]any)
			if !ok {
				return fmt.Errorf("weld %q point %d must be an object", id, index)
			}
			for _, key := range []string{"xyz_m", "rpy_rad"} {
				if err := validateNumberArray(point[key], 3); err != nil {
					return fmt.Errorf("weld %q point %d %s: %w", id, index, key, err)
				}
			}
		}
		return nil
	case "manual_pose_wp":
		return validateManualPoseGeometry(geometry, id, workpieceFrame)
	default:
		return fmt.Errorf("weld %q geometry kind %v is not seam, posed_polyline or manual_pose_wp", id, geometry["kind"])
	}
}

// manualPoseMinChordM is the shortest chord two consecutive manual_pose_wp
// points may have: a micrometre. Below it the chord has no direction to be
// continuous with and the arc-length parameter collapses (the schema's rule).
const manualPoseMinChordM = 1e-6

func validateManualPoseGeometry(geometry map[string]any, id string, workpieceFrame any) error {
	// The workpiece frame by definition: a hand-placed pose rides with the part.
	if frame, _ := geometry["frame_id"].(string); frame == "" || frame != workpieceFrame {
		return fmt.Errorf("weld %q manual_pose_wp frame_id %v is not the workpiece frame %v", id, geometry["frame_id"], workpieceFrame)
	}
	shape, hasShape := geometry["shape"]
	if hasShape && !oneOf(shape, "spline", "arc") {
		return fmt.Errorf("weld %q manual_pose_wp shape must be spline or arc", id)
	}
	arc := shape == "arc"
	points, ok := geometry["points"].([]any)
	if !ok || len(points) < 2 {
		return fmt.Errorf("weld %q manual_pose_wp needs at least two points", id)
	}
	if arc && len(points) != 3 {
		return fmt.Errorf("weld %q manual_pose_wp arc needs exactly three points", id)
	}
	var previous []float64
	for index, raw := range points {
		point, ok := raw.(map[string]any)
		if !ok {
			return fmt.Errorf("weld %q point %d must be an object", id, index)
		}
		for _, key := range []string{"xyz_m", "rpy_rad"} {
			if err := validateNumberArray(point[key], 3); err != nil {
				return fmt.Errorf("weld %q point %d %s: %w", id, index, key, err)
			}
		}
		if !oneOf(point["continuity"], "C0", "C1", "C2") {
			return fmt.Errorf("weld %q point %d continuity must be C0, C1 or C2", id, index)
		}
		xyz := make([]float64, 3)
		for axis, value := range point["xyz_m"].([]any) {
			xyz[axis] = value.(float64)
		}
		if previous != nil && math.Hypot(math.Hypot(xyz[0]-previous[0], xyz[1]-previous[1]), xyz[2]-previous[2]) <= manualPoseMinChordM {
			return fmt.Errorf("weld %q points %d and %d coincide; consecutive points must be distinct", id, index-1, index)
		}
		previous = xyz
	}
	if arc {
		at := func(i int) []float64 {
			raw := points[i].(map[string]any)["xyz_m"].([]any)
			return []float64{raw[0].(float64), raw[1].(float64), raw[2].(float64)}
		}
		a, b, c := at(0), at(1), at(2)
		ab := []float64{b[0] - a[0], b[1] - a[1], b[2] - a[2]}
		ac := []float64{c[0] - a[0], c[1] - a[1], c[2] - a[2]}
		chord := math.Sqrt(ac[0]*ac[0] + ac[1]*ac[1] + ac[2]*ac[2])
		nx, ny, nz := ab[1]*ac[2]-ab[2]*ac[1], ab[2]*ac[0]-ab[0]*ac[2], ab[0]*ac[1]-ab[1]*ac[0]
		area := math.Sqrt(nx*nx + ny*ny + nz*nz)
		if chord <= manualPoseMinChordM || area/chord <= manualPoseMinChordM {
			return fmt.Errorf("weld %q manual_pose_wp arc points lie on a line; no circle passes through them in order", id)
		}
	}
	return nil
}

func validateSeamGeometry(geometry map[string]any, id string, joints map[string]map[string]any) error {
	for _, key := range []string{"id", "reference", "orientation"} {
		if _, present := geometry[key]; !present {
			return fmt.Errorf("weld %q seam geometry needs %s", id, key)
		}
	}
	// A named joint has to EXIST, curve or no curve. The reference resolves
	// inside this document because workpiece.weld_joints[] is here (D24).
	ref, named := geometry["weld_joint_ref"].(string)
	named = named && ref != ""
	if named {
		if _, found := joints[ref]; !found {
			return fmt.Errorf("weld %q names weld_joint_ref %q, which is not in workpiece.weld_joints", id, ref)
		}
	}
	if err := validateSearchProvenance(geometry["search_provenance"], id); err != nil {
		return err
	}
	curve, ok := geometry["curve"].(map[string]any)
	if !ok {
		// A seam may take its line from its joint instead of carrying one,
		// which is what the planner does: Seam.__init__ reads
		// seam["curve"] or joint["weld_line"], and refuses when neither is there.
		if !named {
			return fmt.Errorf("weld %q seam geometry has no inline curve and no weld_joint_ref to take one from", id)
		}
		if _, has := joints[ref]["weld_line"].(map[string]any); !has {
			return fmt.Errorf("weld %q has no curve and joint %q has no weld_line, so the seam has no line at all", id, ref)
		}
		return nil
	}
	segments, ok := curve["segments"].([]any)
	if !ok || len(segments) == 0 {
		return fmt.Errorf("weld %q curve has no segments", id)
	}
	for index, raw := range segments {
		segment, ok := raw.(map[string]any)
		if !ok {
			return fmt.Errorf("weld %q curve segment %d must be an object", id, index)
		}
		poles, _ := segment["control_points_xyz_m"].([]any)
		knots, _ := segment["knots_u"].([]any)
		degree, okDegree := integerValue(segment["degree"])
		if poles == nil || knots == nil || !okDegree {
			return fmt.Errorf("weld %q curve segment %d needs control_points_xyz_m, degree and knots_u", id, index)
		}
		n, p := int64(len(poles)), degree
		if int64(len(knots)) != n+p+1 {
			return fmt.Errorf("weld %q curve segment %d knots_u has %d entries, needs %d + %d + 1", id, index, len(knots), n, p)
		}
		values := make([]float64, len(knots))
		for k, raw := range knots {
			number, ok := raw.(float64)
			if !ok {
				return fmt.Errorf("weld %q curve segment %d knots_u must be numbers", id, index)
			}
			values[k] = number
			if k > 0 && number < values[k-1] {
				return fmt.Errorf("weld %q curve segment %d knots_u is not non-decreasing", id, index)
			}
		}
		for k := int64(0); k <= p; k++ {
			if values[k] != values[0] || values[int64(len(values))-1-k] != values[len(values)-1] {
				return fmt.Errorf("weld %q curve segment %d knots_u is not clamped", id, index)
			}
		}
		if weights, present := segment["weights"].([]any); present && int64(len(weights)) != n {
			return fmt.Errorf("weld %q curve segment %d has %d weights for %d poles", id, index, len(weights), n)
		}
	}
	closed, _ := curve["closed"].(bool)
	if span, ok := geometry["span"].(map[string]any); ok {
		start, okStart := span["start_s"].(float64)
		end, okEnd := span["end_s"].(float64)
		if okStart && okEnd && start > end && !closed {
			return fmt.Errorf("weld %q span wraps through s=0, which is only valid on a closed curve", id)
		}
	}
	if orientation, ok := geometry["orientation"].(map[string]any); ok {
		for _, axis := range []string{"work_angle_deg", "travel_angle_deg"} {
			if profile, ok := orientation[axis].(map[string]any); ok {
				if err := validateScalarFunction(profile, fmt.Sprintf("weld %q %s", id, axis)); err != nil {
					return err
				}
			}
		}
	}
	return nil
}

// validatePositiveScalar checks a profile is positive everywhere.
//
// Every value a standoff profile can take has to be positive: the number is a distance from the work, and zero or less is the tip inside the part. Checked here rather than left to the planner, because a document that says it is welding at 0 mm is wrong before anything is planned.
func validatePositiveScalar(fn map[string]any, where string) error {
	var values []any
	switch fn["kind"] {
	case "constant":
		values = []any{fn["value"]}
	case "piecewise_linear":
		for _, raw := range asSlice(fn["knots"]) {
			if knot := asSlice(raw); len(knot) == 2 {
				values = append(values, knot[1])
			}
		}
	case "samples":
		values = asSlice(fn["values"])
	case "bspline":
		values = asSlice(fn["control_values"])
	default:
		return nil
	}
	for _, raw := range values {
		number, ok := raw.(float64)
		if !ok || number <= 0 {
			return fmt.Errorf("%s must be positive everywhere, got %v", where, raw)
		}
	}
	return nil
}

func validateScalarFunction(fn map[string]any, where string) error {
	switch fn["kind"] {
	case "constant":
		if _, ok := fn["value"].(float64); !ok {
			return fmt.Errorf("%s constant needs a value", where)
		}
	case "piecewise_linear":
		knots, _ := fn["knots"].([]any)
		if len(knots) < 2 {
			return fmt.Errorf("%s piecewise_linear needs at least two knots", where)
		}
		previous := -1.0
		for index, raw := range knots {
			pair, ok := raw.([]any)
			if !ok || len(pair) != 2 {
				return fmt.Errorf("%s piecewise_linear knots must be [s, value] pairs", where)
			}
			s, _ := pair[0].(float64)
			if (index == 0 && s != 0) || (index == len(knots)-1 && s != 1) {
				return fmt.Errorf("%s piecewise_linear knots must run from s=0 to s=1", where)
			}
			if s <= previous {
				return fmt.Errorf("%s piecewise_linear knots must have strictly increasing s", where)
			}
			previous = s
		}
	case "samples":
		s, _ := fn["s"].([]any)
		values, _ := fn["values"].([]any)
		if len(s) < 2 || len(s) != len(values) {
			return fmt.Errorf("%s samples needs matching s and values", where)
		}
	case "bspline":
		knots, _ := fn["knots_u"].([]any)
		values, _ := fn["control_values"].([]any)
		degree, ok := integerValue(fn["degree"])
		if !ok || int64(len(knots)) != int64(len(values))+degree+1 {
			return fmt.Errorf("%s bspline knot count does not match control_values + degree + 1", where)
		}
	default:
		return fmt.Errorf("%s has unknown kind %v", where, fn["kind"])
	}
	return nil
}

func validateFreespaceNode(node map[string]any, id, op string) error {
	if err := forbiddenKeys(node, fmt.Sprintf("node %q", id), true); err != nil {
		return err
	}
	if overrides, present := node["overrides"]; present {
		policy, ok := overrides.(map[string]any)
		if !ok {
			return fmt.Errorf("node %q overrides must be an object", id)
		}
		if err := validatePolicy(policy, fmt.Sprintf("node %q overrides", id)); err != nil {
			return err
		}
	}
	for _, side := range []string{"depart_standoff", "arrive_standoff"} {
		standoff, present := node[side]
		if !present {
			continue
		}
		if side == "depart_standoff" && (op == "approach" || op == "home") {
			return fmt.Errorf("%s %q carries depart_standoff, but nothing is welded before it", op, id)
		}
		if side == "arrive_standoff" && (op == "retract" || op == "home") {
			return fmt.Errorf("%s %q carries arrive_standoff, but nothing is welded after it", op, id)
		}
		lift, ok := standoff.(map[string]any)
		if !ok {
			return fmt.Errorf("node %q %s must be an object", id, side)
		}
		if distance, ok := lift["distance_mm"].(float64); !ok || distance < 0 {
			return fmt.Errorf("node %q %s distance_mm must be non-negative", id, side)
		}
		switch direction := lift["direction"].(type) {
		case string:
			if direction != "torch_axis" && direction != "workpiece_z" {
				return fmt.Errorf("node %q %s direction %q is not torch_axis, workpiece_z or a unit vector", id, side, direction)
			}
		default:
			if err := validateNumberArray(direction, 3); err != nil {
				return fmt.Errorf("node %q %s direction: %w", id, side, err)
			}
		}
	}
	return nil
}

// forbiddenKeys walks value and refuses any key that names a solved quantity.
// Exact match, recursive: a substring match would trip on weld_joint_ref, a
// top-level-only match would miss a path one object down.
func forbiddenKeys(value any, path string, scoped bool) error {
	switch typed := value.(type) {
	case map[string]any:
		for key, child := range typed {
			if programV2ForbiddenKeys[key] || (scoped && programV2FreespaceForbiddenKeys[key]) {
				return fmt.Errorf("%s carries %q, which is a solved quantity; program.v2 states intent only", path, key)
			}
			if err := forbiddenKeys(child, path+"."+key, scoped); err != nil {
				return err
			}
		}
	case []any:
		for index, child := range typed {
			if err := forbiddenKeys(child, fmt.Sprintf("%s[%d]", path, index), scoped); err != nil {
				return err
			}
		}
	}
	return nil
}

// ---------------------------------------------------------------- helpers

func loadProgramV2(t *testing.T) map[string]any {
	t.Helper()
	var doc map[string]any
	decodeFixture(t, filepath.Join("fixtures", "complete-program-v2.json"), &doc)
	return doc
}

func nodeAt(doc map[string]any, index int) map[string]any {
	return doc["nodes"].([]any)[index].(map[string]any)
}

func cloneNode(doc map[string]any, index int, id string) map[string]any {
	node := deepCopy(nodeAt(doc, index)).(map[string]any)
	node["id"] = id
	return node
}

func removeNode(doc map[string]any, index int) {
	nodes := doc["nodes"].([]any)
	doc["nodes"] = append(append([]any{}, nodes[:index]...), nodes[index+1:]...)
}

func insertNode(doc map[string]any, index int, node map[string]any) {
	nodes := doc["nodes"].([]any)
	next := append([]any{}, nodes[:index]...)
	next = append(next, node)
	doc["nodes"] = append(next, nodes[index:]...)
}

func seamSegment(doc map[string]any, nodeIndex, segmentIndex int) map[string]any {
	geometry := nodeAt(doc, nodeIndex)["geometry"].(map[string]any)
	return geometry["curve"].(map[string]any)["segments"].([]any)[segmentIndex].(map[string]any)
}

func deepCopy(value any) any {
	raw, err := json.Marshal(value)
	if err != nil {
		panic(err)
	}
	var out any
	if err := json.Unmarshal(raw, &out); err != nil {
		panic(err)
	}
	return out
}

func oneOf(value any, options ...string) bool {
	text, ok := value.(string)
	if !ok {
		return false
	}
	for _, option := range options {
		if text == option {
			return true
		}
	}
	return false
}

// asSlice reads a JSON array, or nothing when the value is not one.
func asSlice(value any) []any {
	out, _ := value.([]any)
	return out
}

func stringSet(keys ...string) map[string]bool {
	set := make(map[string]bool, len(keys))
	for _, key := range keys {
		set[key] = true
	}
	return set
}

func union(sets ...map[string]bool) map[string]bool {
	out := map[string]bool{}
	for _, set := range sets {
		for key := range set {
			out[key] = true
		}
	}
	return out
}

func numberValue(value any) (float64, bool) {
	number, ok := value.(float64)
	return number, ok
}

// validateSearchProvenance refuses a search that could not have moved what it
// says it was free to move. The window is the authored profile plus or minus
// the deviation and the bin is how finely it was walked, so a bin wider than
// the window leaves one value in it. Each number is fine alone; only the pair
// is wrong, which is why the schema cannot say it.
func validateSearchProvenance(value any, id string) error {
	if value == nil {
		return nil
	}
	block, ok := value.(map[string]any)
	if !ok {
		return fmt.Errorf("weld %q search_provenance must be an object", id)
	}
	for _, axis := range []string{"work", "travel"} {
		deviation, okDeviation := numberValue(block[axis+"_deviation_deg"])
		binned, okBin := numberValue(block[axis+"_bin_deg"])
		if !okDeviation || !okBin {
			continue
		}
		if deviation > 0 && binned > 2*deviation {
			return fmt.Errorf(
				"weld %q searched %s over +/-%v degrees in %v-degree bins, which is one bin: "+
					"the deviation names a freedom the bin spacing did not allow",
				id, axis, deviation, binned,
			)
		}
	}
	return nil
}

func validateTeachPose(value any) error {
	p, ok := value.(map[string]any)
	if !ok || p["frame_id"] != "world" {
		return fmt.Errorf("teach pose requires world frame")
	}
	for key := range p {
		if !stringSet("frame_id", "xyz_m", "rpy_rad", "joint_names", "joint_values_rad")[key] {
			return fmt.Errorf("unknown teach pose field %s", key)
		}
	}
	for _, key := range []string{"xyz_m", "rpy_rad"} {
		v, ok := p[key].([]any)
		if !ok || len(v) != 3 {
			return fmt.Errorf("teach pose %s needs three numbers", key)
		}
		for _, n := range v {
			if _, ok := n.(float64); !ok {
				return fmt.Errorf("teach pose %s needs numbers", key)
			}
		}
	}
	names, _ := p["joint_names"].([]any)
	values, _ := p["joint_values_rad"].([]any)
	if len(names) == 0 || len(names) > 9 || len(names) != len(values) {
		return fmt.Errorf("teach joint names and values must match")
	}
	seen := map[string]bool{}
	for i, n := range names {
		name, ok := n.(string)
		if !ok || name == "" || seen[name] {
			return fmt.Errorf("teach joint names must be unique")
		}
		seen[name] = true
		if _, ok := values[i].(float64); !ok {
			return fmt.Errorf("teach joint values must be numbers")
		}
	}
	return nil
}

func validateTaughtMove(node map[string]any) error {
	if value := node["target_space"]; value != nil && !stringSet("joint", "cartesian")[fmt.Sprint(value)] {
		return fmt.Errorf("unknown taught target space")
	}
	if !stringSet("joint", "linear", "circular")[fmt.Sprint(node["motion"])] {
		return fmt.Errorf("unknown taught motion")
	}
	if err := validateTeachPose(node["target"]); err != nil {
		return err
	}
	if node["via"] != nil {
		if err := validateTeachPose(node["via"]); err != nil {
			return err
		}
	}
	scale, ok := node["speed_scale"].(float64)
	if !ok || scale <= 0 || scale > 1 {
		return fmt.Errorf("invalid taught speed scale")
	}
	if value, present := node["acceleration_scale"]; present {
		if n, ok := value.(float64); !ok || !(n > 0 && n <= 1) {
			return fmt.Errorf("invalid taught acceleration scale")
		}
	}
	speed, ok := node["speed_mm_s"].(float64)
	if !ok || speed <= 0 {
		return fmt.Errorf("invalid taught Cartesian speed")
	}
	if value, present := node["constant_tcp_speed"]; present {
		if _, ok := value.(bool); !ok {
			return fmt.Errorf("invalid constant TCP speed flag")
		}
	}
	c, ok := node["capture"].(map[string]any)
	if !ok {
		return fmt.Errorf("capture provenance required")
	}
	if c["source"] != "machine" && c["source"] != "preview" {
		return fmt.Errorf("invalid capture source")
	}
	if c["source"] == "machine" && !nonempty(c["cell_id"]) {
		return fmt.Errorf("machine capture needs cell id")
	}
	if _, err := time.Parse(time.RFC3339Nano, fmt.Sprint(c["observed_at"])); err != nil {
		return err
	}
	if !nonempty(c["model_id"]) || !regexp.MustCompile(`^sha256:[0-9a-f]{64}$`).MatchString(fmt.Sprint(c["robot_description_sha256"])) {
		return fmt.Errorf("capture needs robot identity")
	}
	for key := range c {
		if !stringSet("source", "observed_at", "model_id", "robot_description_sha256", "machine_planning_calibration_sha256", "cell_id", "pose")[key] {
			return fmt.Errorf("unknown capture field %s", key)
		}
	}
	return validateTeachPose(c["pose"])
}
