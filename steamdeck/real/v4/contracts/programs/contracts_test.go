package programs

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"regexp"
	"strings"
	"testing"
)

var digestPattern = regexp.MustCompile(`^sha256:[0-9a-f]{64}$`)

func TestSchemasAndFixturesAreValidJSON(t *testing.T) {
	t.Parallel()
	for _, name := range []string{
		"robot.v4.program.v1.schema.json",
		"robot.v4.program-catalog.v1.schema.json",
		"robot.v4.accepted-plan.v1.schema.json",
		"robot.v4.program.v2.schema.json",
		filepath.Join("fixtures", "complete-program.json"),
		filepath.Join("fixtures", "complete-catalog.json"),
		filepath.Join("fixtures", "accepted-simulation-plan.json"),
		filepath.Join("fixtures", "invalid-program-missing-row-id.json"),
		filepath.Join("fixtures", "complete-program-v2.json"),
		filepath.Join("fixtures", "invalid-program-v2-freespace-solution.json"),
		filepath.Join("fixtures", "invalid-program-v2-dense-joints.json"),
		filepath.Join("fixtures", "invalid-program-v2-weld-parameters-filled.json"),
		filepath.Join("fixtures", "invalid-program-v2-adjacent-welds.json"),
	} {
		var value any
		if err := json.Unmarshal(readFixture(t, name), &value); err != nil {
			t.Fatalf("%s is not valid JSON: %v", name, err)
		}
	}
}

func TestCompleteProgramFixtureSatisfiesSharedRuntimeContract(t *testing.T) {
	t.Parallel()
	var program map[string]any
	decodeFixture(t, filepath.Join("fixtures", "complete-program.json"), &program)
	if err := validateProgram(program); err != nil {
		t.Fatal(err)
	}
	rows := program["rows"].([]any)
	rowTypes := make(map[string]bool)
	for _, raw := range rows {
		row := raw.(map[string]any)
		rowTypes[row["row_type"].(string)] = true
	}
	for _, rowType := range []string{"motion", "command", "comment", "capture", "calibration", "io"} {
		if !rowTypes[rowType] {
			t.Errorf("complete fixture is missing %s row coverage", rowType)
		}
	}
}

func TestCatalogAndAcceptedPlanFixturesSatisfyIdentityContract(t *testing.T) {
	t.Parallel()
	var catalog map[string]any
	decodeFixture(t, filepath.Join("fixtures", "complete-catalog.json"), &catalog)
	if catalog["schema"] != "robot.v4.program_catalog.v1" ||
		catalog["owner"] != "motion-server" {
		t.Fatalf("unexpected catalog identity: %#v", catalog)
	}
	if revision, ok := integerValue(catalog["catalog_revision"]); !ok || revision < 0 {
		t.Fatalf("catalog revision is invalid: %#v", catalog["catalog_revision"])
	}
	programs, ok := catalog["programs"].([]any)
	if !ok || len(programs) != 1 {
		t.Fatalf("catalog programs are invalid: %#v", catalog["programs"])
	}
	if err := validateProgram(programs[0].(map[string]any)); err != nil {
		t.Fatal(err)
	}

	var plan map[string]any
	decodeFixture(t, filepath.Join("fixtures", "accepted-simulation-plan.json"), &plan)
	if err := validateAcceptedPlan(plan); err != nil {
		t.Fatal(err)
	}
}

func TestInvalidFixtureFailsClosed(t *testing.T) {
	t.Parallel()
	var program map[string]any
	decodeFixture(t, filepath.Join("fixtures", "invalid-program-missing-row-id.json"), &program)
	if err := validateProgram(program); err == nil ||
		!strings.Contains(err.Error(), "row_id") {
		t.Fatalf("missing row identity was not rejected: %v", err)
	}
}

func validateProgram(program map[string]any) error {
	if version, ok := integerValue(program["schema_version"]); !ok || version != 1 {
		return fmt.Errorf("schema_version must be 1")
	}
	for _, field := range []string{"program_id", "program_name", "robot_model", "program_state", "execution_mode"} {
		if !nonempty(program[field]) {
			return fmt.Errorf("%s is required", field)
		}
	}
	if !nonempty(program["robotic-cell-type"]) && !nonempty(program["robotic_cell_type"]) {
		return fmt.Errorf("robotic cell type is required")
	}
	if revision, ok := integerValue(program["program_revision"]); !ok || revision < 1 {
		return fmt.Errorf("program_revision must be positive")
	}
	if base, ok := integerValue(program["base_program_revision"]); !ok || base < 0 {
		return fmt.Errorf("base_program_revision must be non-negative")
	}
	units, ok := program["units"].(map[string]any)
	if !ok || units["length"] != "m" || units["angle"] != "rad" || units["time"] != "s" {
		return fmt.Errorf("canonical SI units are required")
	}
	if err := validateJointOrder(units["joint_order"]); err != nil {
		return err
	}
	rows, ok := program["rows"].([]any)
	if !ok || len(rows) < 1 || len(rows) > 4096 {
		return fmt.Errorf("rows must contain 1-4096 entries")
	}
	ids := make(map[string]bool, len(rows))
	for index, raw := range rows {
		row, ok := raw.(map[string]any)
		if !ok {
			return fmt.Errorf("row %d must be an object", index)
		}
		rowID, _ := row["row_id"].(string)
		if strings.TrimSpace(rowID) == "" {
			return fmt.Errorf("row %d row_id is required", index)
		}
		if ids[rowID] {
			return fmt.Errorf("duplicate row_id %s", rowID)
		}
		ids[rowID] = true
		if !nonempty(row["row_hash"]) {
			return fmt.Errorf("row %s row_hash is required", rowID)
		}
		if _, ok := row["enabled"].(bool); !ok {
			return fmt.Errorf("row %s enabled must be boolean", rowID)
		}
		if !nonempty(row["row_type"]) {
			return fmt.Errorf("row %s row_type is required", rowID)
		}
		if row["row_type"] == "motion" {
			if err := validateNumberArray(row["target_tcp_m"], 6); err != nil {
				return fmt.Errorf("row %s target_tcp_m: %w", rowID, err)
			}
			if err := validateNumberArray(row["target_joints_rad"], 9); err != nil {
				return fmt.Errorf("row %s target_joints_rad: %w", rowID, err)
			}
		}
	}
	for _, raw := range rows {
		row := raw.(map[string]any)
		if row["row_type"] == "command" && row["command"] == "LOOP" {
			target, _ := row["loop_target_row_id"].(string)
			if !ids[target] {
				return fmt.Errorf("loop targets missing row_id %s", target)
			}
		}
	}
	return nil
}

func validateAcceptedPlan(plan map[string]any) error {
	if plan["schema"] != "robot.v4.accepted-plan.v1" {
		return fmt.Errorf("accepted plan schema is invalid")
	}
	if plan["authority_classification"] != "simulation_only" &&
		plan["authority_classification"] != "physical_target" {
		return fmt.Errorf("accepted plan authority is invalid")
	}
	for _, field := range []string{"plan_id", "program_id", "created_at"} {
		if !nonempty(plan[field]) {
			return fmt.Errorf("accepted plan %s is required", field)
		}
	}
	for _, field := range []string{"trajectory_digest", "program_digest"} {
		value, _ := plan[field].(string)
		if !digestPattern.MatchString(value) {
			return fmt.Errorf("accepted plan %s is invalid", field)
		}
	}
	planner, ok := plan["planner"].(map[string]any)
	if !ok || planner["owner"] != "motion-server" {
		return fmt.Errorf("accepted plan planner owner is invalid")
	}
	buildDigest, _ := planner["build_digest"].(string)
	if !digestPattern.MatchString(buildDigest) {
		return fmt.Errorf("accepted plan planner build digest is invalid")
	}
	target, ok := plan["target"].(map[string]any)
	if !ok {
		return fmt.Errorf("accepted plan target is required")
	}
	if err := validateJointOrder(target["joint_order"]); err != nil {
		return err
	}
	trajectory, ok := plan["trajectory"].([]any)
	if !ok || len(trajectory) < 2 {
		return fmt.Errorf("accepted plan trajectory requires at least two samples")
	}
	prior := -1.0
	for index, raw := range trajectory {
		sample := raw.(map[string]any)
		timeValue, ok := sample["time_s"].(float64)
		if !ok || timeValue < prior {
			return fmt.Errorf("accepted plan sample %d time is not monotonic", index)
		}
		prior = timeValue
		if err := validateNumberArray(sample["tcp_pose_m"], 6); err != nil {
			return err
		}
		if err := validateNumberArray(sample["joint_positions_rad"], 9); err != nil {
			return err
		}
	}
	return nil
}

func validateJointOrder(value any) error {
	values, ok := value.([]any)
	if !ok || len(values) != 9 {
		return fmt.Errorf("joint_order must contain J1-J9")
	}
	for index, raw := range values {
		if raw != fmt.Sprintf("J%d", index+1) {
			return fmt.Errorf("joint_order must contain J1-J9")
		}
	}
	return nil
}

func validateNumberArray(value any, length int) error {
	values, ok := value.([]any)
	if !ok || len(values) != length {
		return fmt.Errorf("must contain %d finite numbers", length)
	}
	for _, value := range values {
		if _, ok := value.(float64); !ok {
			return fmt.Errorf("must contain %d finite numbers", length)
		}
	}
	return nil
}

func nonempty(value any) bool {
	text, ok := value.(string)
	return ok && strings.TrimSpace(text) != ""
}

func integerValue(value any) (int64, bool) {
	number, ok := value.(float64)
	if !ok || number != float64(int64(number)) {
		return 0, false
	}
	return int64(number), true
}

func decodeFixture(t *testing.T, name string, target any) {
	t.Helper()
	if err := json.Unmarshal(readFixture(t, name), target); err != nil {
		t.Fatalf("decode %s: %v", name, err)
	}
}

func readFixture(t *testing.T, name string) []byte {
	t.Helper()
	return readPath(t, name)
}

func readPath(t *testing.T, name string) []byte {
	t.Helper()
	content, err := os.ReadFile(name)
	if err != nil {
		t.Fatalf("read %s: %v", name, err)
	}
	return content
}
