package rtctl

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"encoding/xml"
	"fmt"
	"io"
	"math"
	"math/big"
	"os"
	"path/filepath"
	"strconv"
	"strings"

	"rosieos/robotdesc"
)

// RobotBinding preserves the validated model and axis mapping with the compiled
// configuration, preventing inventory from loading a different robot later.
type RobotBinding struct {
	ModelID  string `json:"model_id"`
	Revision string `json:"revision"`
	// Description is the robot description the machine pins -- the directory
	// the planner's whole picture of the robot comes from, and where rt-core's
	// own definition of it lives (docs/ROBOT-DESCRIPTION-CONTRACT.md) --
	// snapshotted into the compiled resources file for file so the cell
	// serves it.
	Description DescriptionBinding `json:"robot_description"`
	// MachinePlanningCalibrationSHA256 is the identity of this machine's machine_planning_calibration.json, "sha256:<hex>"
	// of the bytes served -- the file the machine pins, or the model's empty
	// document when it pins none.
	MachinePlanningCalibrationSHA256 string             `json:"machine_planning_calibration_sha256"`
	Axes                             []RobotAxisBinding `json:"axes"`
	machinePlanningCalibration       []byte
	profiles                         []any
	// The repository the description path is relative to, for the snapshot.
	repository string
}

// DescriptionBinding names the description directory a machine pins and the
// identity it hashed to: sha256 over the manifest and every file the manifest
// registers, the same number the OLP computes from its own copy.
type DescriptionBinding struct {
	Path                   string `json:"path"`
	RobotDescriptionSHA256 string `json:"sha256"`
}

type RobotAxisBinding struct {
	Name                     string `json:"name"`
	Profile                  string `json:"drive_profile"`
	Kind                     string `json:"kind"`
	URDFJoint                any    `json:"urdf_joint"`
	CartesianParticipates    bool   `json:"cartesian_participates"`
	MotorEncoderCountsPerRev string `json:"motor_encoder_counts_per_rev"`
}

// RobotDefinitionError identifies model admission failures before publication;
// callers can distinguish a bad binding from a drive readback refusal.
type RobotDefinitionError struct{ Reason, Detail string }

func (e *RobotDefinitionError) Error() string { return e.Reason + ": " + e.Detail }

func refuseRobot(reason, format string, args ...any) {
	panic(&RobotDefinitionError{reason, fmt.Sprintf(format, args...)})
}

// uniqueRobotJSON rejects duplicate keys before interpretation. A later sign,
// hash or limit must not silently override an earlier declaration.
func uniqueRobotJSON(data []byte) {
	d := json.NewDecoder(bytes.NewReader(data))
	d.UseNumber()
	var read func()
	read = func() {
		t, err := d.Token()
		if err != nil {
			reject("robot JSON: %v", err)
		}
		switch t {
		case json.Delim('{'):
			seen := map[string]bool{}
			for d.More() {
				key, err := d.Token()
				if err != nil {
					reject("robot JSON: %v", err)
				}
				name := asString(key)
				if seen[name] {
					refuseRobot("robot_duplicate_field", "duplicate JSON field %s", name)
				}
				seen[name] = true
				read()
			}
			if _, err := d.Token(); err != nil {
				reject("robot JSON: %v", err)
			}
		case json.Delim('['):
			for d.More() {
				read()
			}
			if _, err := d.Token(); err != nil {
				reject("robot JSON: %v", err)
			}
		}
	}
	read()
	if _, err := d.Token(); err != io.EOF {
		reject("trailing robot JSON input")
	}
}

// robotFields rejects misspellings and calibration payloads instead of silently
// accepting a second source of mechanics; annotations are string siblings only.
func robotFields(o object, allowed string) {
	keys := map[string]bool{}
	for _, key := range strings.Fields(allowed) {
		keys[key] = true
	}
	for _, m := range o {
		if keys[m.key] {
			continue
		}
		if m.key == "brake_override" {
			panic(&BrakeOverrideError{"brake_override", "robot definition", "only bench cell axes may declare an override"})
		}
		note := m.key == "_source" || m.key == "_unverified" || m.key == "_note"
		for _, suffix := range []string{"_source", "_unverified", "_note"} {
			base, ok := strings.CutSuffix(m.key, suffix)
			note = note || ok && keys[base]
		}
		if !note {
			refuseRobot("robot_definition_field", "unexpected field %q", m.key)
		}
		if strings.TrimSpace(asString(m.value)) == "" {
			reject("empty annotation %s", m.key)
		}
	}
}

// robotNumericSources requires an explicit sibling for each numeric model value,
// so a plausible drive readback cannot silently become verified mechanics.
func robotNumericSources(o object) {
	for _, m := range o {
		switch v := m.value.(type) {
		case json.Number:
			if strings.TrimSpace(asString(o.optional(m.key+"_source", o.optional(m.key+"_unverified", "")))) == "" {
				reject("robot definition numeric field %s needs _source or _unverified", m.key)
			}
		case object:
			robotNumericSources(v)
		case []any:
			for _, item := range v {
				if child, ok := item.(object); ok {
					robotNumericSources(child)
				}
			}
		}
	}
}

// descriptionRoot is what a machine's `robot.description` path is relative
// to: the component itself when it carries `robot_description/` -- a runtime
// package, which stages every description so a cell compiles without a
// checkout (tools/package-runtime.sh) -- and otherwise the repository the
// component sits in. Either way the identity pin decides, so the two can
// never quietly disagree.
func descriptionRoot(componentRoot string) string {
	if info, err := os.Stat(filepath.Join(componentRoot, "robot_description")); err == nil && info.IsDir() {
		return componentRoot
	}
	return filepath.Dir(componentRoot)
}

// boundPath confines a slash-separated reference, including symlinks, to its
// declared root. Absolute paths and traversal cannot substitute another model.
func boundPath(root, reference string) string {
	validateRobotReference(reference)
	root, err := filepath.Abs(root)
	if err != nil {
		reject("reference root: %v", err)
	}
	if resolved, err := filepath.EvalSymlinks(root); err == nil {
		root = resolved
	}
	file := filepath.Join(root, filepath.FromSlash(reference))
	// Check existing ancestors too: a missing target under an escaping symlink
	// must not trigger the sparse-checkout fallback.
	for probe := file; probe != root; probe = filepath.Dir(probe) {
		resolved, err := filepath.EvalSymlinks(probe)
		if os.IsNotExist(err) {
			continue
		}
		if err != nil {
			reject("reference path: %v", err)
		}
		rel, err := filepath.Rel(root, resolved)
		if err != nil || rel == ".." || strings.HasPrefix(rel, ".."+string(filepath.Separator)) {
			refuseRobot("robot_reference_path", "reference escapes root: %s", reference)
		}
		break
	}
	return file
}

func validateRobotReference(reference string) {
	if reference == "" || strings.ContainsAny(reference, "\\:\x00") || strings.HasPrefix(reference, "/") {
		refuseRobot("robot_reference_path", "invalid relative path %q", reference)
	}
	for _, part := range strings.Split(reference, "/") {
		if part == "" || part == "." || part == ".." {
			refuseRobot("robot_reference_path", "invalid path %q", reference)
		}
	}
}

func validateRobotHash(expected, label string) {
	bytes, err := hex.DecodeString(expected)
	if err != nil || len(bytes) != sha256.Size || expected != strings.ToLower(expected) {
		refuseRobot("robot_hash_invalid", "%s requires a lowercase SHA256", label)
	}
}

func verifyRobotHash(data []byte, expected, label string) {
	validateRobotHash(expected, label)
	if actual := fmt.Sprintf("%x", sha256.Sum256(data)); actual != expected {
		refuseRobot("robot_hash_mismatch", "%s expected %s observed %s; update the model revision and binding", label, expected, actual)
	}
}

type factoryJoint struct {
	Name  string    `xml:"name,attr"`
	Type  string    `xml:"type,attr"`
	Mimic *struct{} `xml:"mimic"`
	Limit *struct {
		Lower    string `xml:"lower,attr"`
		Upper    string `xml:"upper,attr"`
		Velocity string `xml:"velocity,attr"`
	} `xml:"limit"`
}

// factoryLimits reads only top-level movable joints of the description's
// URDF, excluding fixed frames and transmission declarations. Unsupported
// scaling and mimic joints are refused. A description without a URDF -- a
// bench robot -- has no factory bounds: the cell's limits then stand alone.
func factoryLimits(data []byte, id string) map[string]object {
	limits := map[string]object{}
	if data == nil {
		return limits
	}
	var document struct {
		XMLName xml.Name       `xml:"robot"`
		Name    string         `xml:"name,attr"`
		Joints  []factoryJoint `xml:"joint"`
	}
	if err := xml.Unmarshal(data, &document); err != nil {
		reject("factory URDF: %v", err)
	}
	if document.Name != id {
		reject("factory URDF robot name differs from robot_id")
	}
	seen := map[string]bool{}
	for _, joint := range document.Joints {
		if joint.Name == "" || seen[joint.Name] {
			reject("empty or duplicate factory URDF joint %q", joint.Name)
		}
		seen[joint.Name] = true
		if joint.Type == "fixed" {
			continue
		}
		if joint.Type != "revolute" || joint.Mimic != nil || joint.Limit == nil {
			reject("factory joint %s requires independent revolute scaling and bounded limits", joint.Name)
		}
		l := joint.Limit
		lower, upper, velocity := json.Number(l.Lower), json.Number(l.Upper), json.Number(l.Velocity)
		if finite(lower, "URDF lower") >= finite(upper, "URDF upper") || finite(velocity, "URDF velocity") <= 0 {
			reject("invalid URDF limits for %s", joint.Name)
		}
		limits[joint.Name] = object{{"min", lower}, {"max", upper}, {"max_velocity", velocity}}
	}
	if len(limits) == 0 {
		reject("the description's robot.urdf has no movable joint")
	}
	return limits
}

// expandRobot derives native axes once from a hash-bound model. Cell inputs can
// narrow geometry and Home policy, set drive tuning-catalog keys per joint and
// list catalog keys to observe, while scaling remains model-owned and the
// fast profile selector is bench-only.
func expandRobot(machine object, profilesDir string) (object, *RobotBinding) {
	if machine.optional("robot", nil) == nil {
		for _, value := range asArray(machine.get("axes")) {
			if asObject(value).optional("encoder_battery", nil) != nil {
				reject("encoder_battery is a robot definition joint fact; migrate the flat configuration")
			}
			if asObject(value).optional("robot_joint", nil) != nil {
				reject("robot_joint requires robot.definition; migrate the flat configuration")
			}
		}
		return machine, nil
	}
	// Resolve before taking parents: filepath.Dir("../..") is "..", which is
	// not the parent of the directory that "../.." names from the current cwd.
	var pathErr error
	profilesDir, pathErr = filepath.Abs(profilesDir)
	if pathErr != nil {
		reject("profile root: %v", pathErr)
	}
	ref := asObject(machine.get("robot"))
	robotFields(ref, "robot_description absent_joints bench_measurement_profile machine_planning_calibration")
	// The robot is a description directory (docs/ROBOT-DESCRIPTION-CONTRACT.md):
	// the manifest says which files it is made of, the identity over them is
	// the pin, and rt-core's definition of the robot -- gearing, encoders,
	// drive profiles -- is one of those files. Nothing about the robot is read
	// from anywhere else.
	componentRoot := filepath.Dir(filepath.Dir(profilesDir))
	repositoryRoot := descriptionRoot(componentRoot)
	desc := asObject(ref.get("robot_description"))
	robotFields(desc, "path sha256 source")
	descriptionPath := asString(desc.get("path"))
	validateRobotReference(descriptionPath)
	id := filepath.Base(descriptionPath)
	dir := boundPath(repositoryRoot, descriptionPath)
	identity, err := robotdesc.RobotDescriptionSHA256(dir)
	if err != nil {
		refuseRobot("robot_description_unavailable", "%v", err)
	}
	if pinned := asString(desc.get("sha256")); identity != pinned {
		refuseRobot("robot_description_mismatch", "%s hashes to %s and the machine pins %s; update the pin", descriptionPath, identity, pinned)
	}
	registered, err := robotdesc.DescriptionFiles(dir)
	if err != nil {
		refuseRobot("robot_description_unavailable", "%v", err)
	}
	files := map[string]bool{}
	for _, rel := range registered {
		files[rel] = true
	}
	if !files[robotdesc.Definition] {
		refuseRobot("robot_definition_unavailable", "%s does not register %s", descriptionPath, robotdesc.Definition)
	}
	data := readBoundedResource(boundPath(repositoryRoot, descriptionPath+"/"+robotdesc.Definition))
	uniqueRobotJSON(data)
	definition := asObject(decode(data))
	robotFields(definition, "schema robot_id name revision defaults joints calibration")
	if definition.get("schema") != "rosie.robot-definition.v1" {
		reject("robot schema must be rosie.robot-definition.v1")
	}
	if token(definition.get("robot_id")) != id {
		refuseRobot("robot_description_mismatch", "%s is the definition of %s, not of the directory %s", robotdesc.Definition, asString(definition.get("robot_id")), id)
	}
	if strings.TrimSpace(asString(definition.get("name"))) == "" {
		reject("robot name is required")
	}
	revision := integer(definition.get("revision"), 1, ^uint64(0), "robot revision")
	robotNumericSources(definition)
	calibration := asObject(definition.get("calibration"))
	robotFields(calibration, "identity note")
	if calibration.get("identity") != "home_calibration" || strings.TrimSpace(asString(calibration.get("note"))) == "" {
		reject("robot calibration must reserve home_calibration with a note; offsets belong in its own record")
	}
	defaults := asObject(definition.get("defaults"))
	robotFields(defaults, "drive_profile home_policy coordinate_evidence_mode")
	profileName := token(defaults.get("drive_profile"))
	homePolicy := asString(defaults.get("home_policy"))
	if homePolicy != "required" && homePolicy != "optional" {
		reject("home_policy must be required or optional")
	}
	mode := asString(defaults.get("coordinate_evidence_mode"))
	if mode != "multi_turn" {
		reject("coordinate_evidence_mode must be multi_turn; single-turn evidence is retired")
	}
	// The geometry, when the description has any: its own robot.urdf, which
	// must be registered -- a URDF the manifest does not name is not part of
	// the robot's identity and cannot bound it.
	var urdf []byte
	if _, err := os.Stat(filepath.Join(dir, "robot.urdf")); err == nil {
		if !files["robot.urdf"] {
			refuseRobot("robot_description_mismatch", "%s has a robot.urdf its manifest does not register", descriptionPath)
		}
		urdf = readBoundedResource(boundPath(repositoryRoot, descriptionPath+"/robot.urdf"))
	}
	limits := factoryLimits(urdf, id)
	binding := &RobotBinding{ModelID: id, Revision: revision, Description: DescriptionBinding{Path: descriptionPath, RobotDescriptionSHA256: identity}, repository: repositoryRoot}
	// This machine's machine_planning_calibration.json: how it deviates from the description, in the
	// planner's terms, judged here by the contract's own rules and served
	// beside the description so the plan is made against exactly it. The
	// path is relative to the component's config tree (config/...), the same
	// in a checkout and in a runtime package. A machine that pins none is a
	// machine with no deviation: it serves the model's empty document, which
	// is what a plan made with no cell to ask hashes to.
	if raw := ref.optional("machine_planning_calibration", nil); raw != nil {
		pin := asObject(raw)
		robotFields(pin, "path sha256 source")
		cellPath := asString(pin.get("path"))
		validateRobotReference(cellPath)
		binding.machinePlanningCalibration = readBoundedResource(boundPath(componentRoot, cellPath))
		verifyRobotHash(binding.machinePlanningCalibration, asString(pin.get("sha256")), "machine_planning_calibration.json")
	}
	description, err := robotdesc.At(filepath.Dir(dir)).Load(id)
	if err != nil {
		refuseRobot("robot_description_unavailable", "%v", err)
	}
	source := robotdesc.SourceTarget
	if binding.machinePlanningCalibration == nil {
		source = robotdesc.SourceEmpty
	}
	if description.HasGeometry() {
		if _, err := robotdesc.Resolve(description, binding.machinePlanningCalibration, source); err != nil {
			refuseRobot("machine_planning_calibration_invalid", "%v", err)
		}
	} else if binding.machinePlanningCalibration != nil {
		refuseRobot("machine_planning_calibration_invalid", "%s has no geometry for a machine_planning_calibration.json to deviate from", descriptionPath)
	}
	if binding.machinePlanningCalibration == nil {
		binding.machinePlanningCalibration = robotdesc.EmptyMachinePlanningCalibrationBytes(id)
	}
	binding.MachinePlanningCalibrationSHA256 = robotdesc.MachinePlanningCalibrationSHA256(binding.machinePlanningCalibration)
	joints, mapped := map[string]object{}, map[string]bool{}
	for _, value := range asArray(definition.get("joints")) {
		joint := asObject(value)
		robotFields(joint, "name urdf_joint kind gear_ratio motor_encoder_counts_per_rev encoder_battery sign drive_profile cartesian_participates mechanical_travel")
		name := token(joint.get("name"))
		if _, exists := joints[name]; exists {
			reject("duplicate definition joint %s", name)
		}
		kind := asString(joint.get("kind"))
		if kind != "arm" && kind != "positioner" && kind != "external" {
			reject("invalid joint kind %s", kind)
		}
		cartesian := asBool(joint.get("cartesian_participates"))
		if u := joint.get("urdf_joint"); u != nil {
			mappedName := asString(u)
			if limits[mappedName] == nil || mapped[mappedName] {
				reject("unknown or duplicate URDF mapping %s", mappedName)
			}
			mapped[mappedName] = true
		} else if cartesian || kind == "arm" {
			reject("arm or Cartesian joint %s requires a URDF mapping", name)
		}
		ratio := asObject(joint.get("gear_ratio"))
		robotFields(ratio, "numerator denominator source")
		integer(ratio.get("numerator"), 1, 0xffffffff, "gear numerator")
		integer(ratio.get("denominator"), 1, 0xffffffff, "gear denominator")
		if strings.TrimSpace(asString(ratio.get("source"))) == "" {
			reject("gear ratio source is required")
		}
		integer(joint.get("motor_encoder_counts_per_rev"), 1, 0x7fffffff, "motor encoder counts/rev")
		if integer(joint.get("sign"), -1, 1, "robot sign") == "0" {
			reject("robot sign must be -1 or +1")
		}
		if p := joint.optional("drive_profile", nil); p != nil {
			token(p)
		}
		// Validate even joints omitted by this cell; absence from the bus cannot
		// hide a missing drive fact in the shared robot definition.
		pname := profileName
		if override := joint.optional("drive_profile", nil); override != nil {
			pname = asString(override)
		}
		profileData, err := os.ReadFile(filepath.Join(profilesDir, pname+".json"))
		if err != nil {
			reject("profile %s: %v", pname, err)
		}
		validateEncoderBattery(joint, asObject(decode(profileData)))
		if travel := joint.optional("mechanical_travel", nil); travel != nil {
			if joint.get("urdf_joint") != nil {
				refuseRobot("robot_urdf_limits_required", "joint %s mechanical_travel must inherit its URDF joint limits", name)
			}
			l := asObject(travel)
			robotFields(l, "min max")
			if finite(l.get("min"), "mechanical minimum") >= finite(l.get("max"), "mechanical maximum") {
				reject("invalid mechanical travel for %s", name)
			}
		}
		joints[name] = joint
	}
	if len(joints) == 0 || len(joints) > 16 || len(mapped) != len(limits) {
		reject("definition must map every movable URDF joint and contain 1..16 joints")
	}
	if urdf != nil {
		// A mesh the URDF names that the manifest does not register is a
		// description that would serve less than the robot. A package label
		// may differ from the robot id (a Rosie 1430 arm uses rosie_1430_v1); the pinned
		// manifest still owns every byte. Never resolve external packages.
		for _, reference := range meshReferences(urdf) {
			rel, valid := robotdesc.MeshResourcePath(reference)
			if !valid || !files[rel] {
				reject("URDF mesh %s is not a file the description registers", reference)
			}
		}
	}
	absent := map[string]bool{}
	for _, value := range asArray(ref.optional("absent_joints", []any{})) {
		name := token(value)
		if joints[name] == nil || absent[name] {
			reject("unknown or duplicate absent joint %s", name)
		}
		absent[name] = true
	}
	var axes []any
	used := map[string]bool{}
	profileCache := map[string]object{}
	catalogs := loadTuningCatalogs(profilesDir)
	for _, value := range asArray(machine.get("axes")) {
		cell := asObject(value)
		robotFields(cell, "robot_joint slave_position limits require_home coordinate_evidence_mode startup startup_trial observe collision_watchdog brake_override brake_transition_guard brake_opening_guard reference_verified_delta_restore")
		name := token(cell.get("robot_joint"))
		// One brake guard value set per robot: per-joint copies drift into
		// per-joint numbers, so robot-bound cells declare them at machine level.
		for _, field := range cell {
			if field.key == "brake_transition_guard" || field.key == "brake_opening_guard" {
				refuseRobot("robot_brake_guard_per_axis", "axis %s %s must be declared once at machine level in joint units for every axis", name, field.key)
			}
		}
		joint := joints[name]
		if joint == nil || used[name] || absent[name] {
			refuseRobot("robot_joint_mapping", "unknown, duplicate or absent cell robot_joint %s", name)
		}
		if joint.get("urdf_joint") != nil {
			for _, field := range machine {
				if field.key == "drive_speed_limit_motor_rpm" || field.key == "max_motor_rpm" {
					refuseRobot("robot_urdf_limits_required", "%s must inherit the robot URDF velocity and gearing", field.key)
				}
			}
			// A robot-bound bare-motor bench exercises the selected robot's travel
			// and evidence policy; it must not define another coordinate envelope.
			if bus, ok := machine.optional("bus", nil).(object); ok && bus.optional("unknown_slaves", nil) == "hold" {
				for _, field := range cell {
					if field.key == "coordinate_evidence_mode" {
						refuseRobot("robot_bench_inheritance", "bench coordinate_evidence_mode must inherit the selected robot definition")
					}
				}
				for _, field := range asObject(cell.optional("limits", object{})) {
					if field.key == "min" || field.key == "max" || field.key == "max_velocity" || field.key == "jog_acceleration" {
						refuseRobot("robot_bench_inheritance", "bench limits.%s must inherit the selected robot definition", field.key)
					}
				}
			}
			for _, field := range asObject(cell.optional("limits", object{})) {
				if field.key == "min" || field.key == "max" || field.key == "max_velocity" || field.key == "jog_acceleration" {
					refuseRobot("robot_urdf_limits_required", "axis %s limits.%s must inherit its URDF joint limits", name, field.key)
				}
			}
		}
		used[name] = true
		pname := profileName
		if override := joint.optional("drive_profile", nil); override != nil {
			pname = asString(override)
		}
		if override := ref.optional("bench_measurement_profile", nil); override != nil {
			if id != "bench_nine_motors_1to1" || pname != "a6ec_bench_1to1" || override != "a6ec_bench_1to1_fast" {
				refuseRobot("robot_definition_field", "bench_measurement_profile is restricted to the bare-motor 1/1 bench")
			}
			pname = asString(override)
		}
		profile := profileCache[pname]
		if profile == nil {
			profileData, err := os.ReadFile(filepath.Join(profilesDir, pname+".json"))
			if err != nil {
				reject("profile %s: %v", pname, err)
			}
			profile = asObject(decode(profileData))
			profileCache[pname] = profile
		}
		binding.profiles = append(binding.profiles, profile)
		a6 := startupSchema(profile).optional("a6ec_encoder_position_tracking_mode", nil) != nil
		encoder := integer(joint.get("motor_encoder_counts_per_rev"), 1, 0x7fffffff, "motor encoder counts/rev")
		ratio := asObject(joint.get("gear_ratio"))
		num, den := ratio.get("numerator").(json.Number), ratio.get("denominator").(json.Number)
		n, _ := new(big.Int).SetString(string(num), 10)
		d, _ := new(big.Int).SetString(string(den), 10)
		wireCounts, wireRatio := json.Number(encoder), json.Number("1")
		startup := append(object{}, asObject(profile.optional("startup_defaults", object{}))...)
		battery := validateEncoderBattery(joint, profile)
		if a6 {
			tracking := startup.optional("a6ec_encoder_position_tracking_mode", nil)
			if !numericEqual(tracking, 1) {
				reject("axis %s A6-EC profile must pin linear tracking mode 1", name)
			}
		}
		if overrides := cell.optional("startup", nil); overrides != nil {
			if len(asObject(overrides)) == 0 {
				refuseRobot("robot_definition_field", "cell startup requires explicit tuning values")
			}
			// Only drive tuning is cell-owned; gearing, direction and tracking
			// mode remain derived from the pinned robot definition.
			catalog := catalogFor(catalogs, profile)
			for _, setting := range asObject(overrides) {
				if startupSchema(profile).optional(setting.key, nil) != nil || catalog == nil || catalog.entries[setting.key] == nil {
					refuseRobot("robot_definition_field", "cell startup %s is not a drive tuning-catalog key; gearing, direction and tracking mode come from the robot definition", setting.key)
				}
				startup.set(setting.key, setting.value)
			}
		}
		// Linear A6 mode and generic software-scaled drives both expose the
		// motor encoder frame. Mechanical gearing comes only from the robot.
		f, _ := new(big.Rat).SetFrac(n, d).Float64()
		wireRatio = json.Number(pythonFloat(f))
		home := homePolicy == "required"
		if v := cell.optional("require_home", nil); v != nil {
			if home && !asBool(v) {
				reject("axis %s cannot weaken required Home policy", name)
			}
			home = asBool(v)
		}
		evidence := asString(cell.optional("coordinate_evidence_mode", mode))
		if evidence != "multi_turn" {
			reject("axis %s coordinate_evidence_mode must be multi_turn", name)
		}
		base := object{}
		var robotAcceleration any
		if u := joint.get("urdf_joint"); u != nil {
			base = append(base, limits[asString(u)]...)
			rate, ok := description.Config.PlanningJoints[asString(u)]
			if !ok {
				refuseRobot("robot_acceleration_required", "axis %s requires config.json planning.joints.%s.acceleration_rad_s2 in its robot description", name, asString(u))
			}
			robotAcceleration = json.Number(pythonFloat(rate.AccelerationRadS2))
			base.set("jog_acceleration", robotAcceleration)
		}
		if travel := joint.optional("mechanical_travel", nil); travel != nil {
			for _, key := range []string{"min", "max"} {
				v := asObject(travel).get(key)
				old := base.optional(key, nil)
				if old == nil || key == "min" && finite(v, key) > finite(old, key) || key == "max" && finite(v, key) < finite(old, key) {
					base.set(key, v)
				}
			}
		}
		cellLimits := asObject(cell.optional("limits", object{}))
		robotFields(cellLimits, "min max max_velocity jog_acceleration max_target_lead following_error following_error_timeout_ms completion_tolerance completion_timeout_ms")
		for _, m := range cellLimits {
			if old := base.optional(m.key, nil); old != nil && (m.key == "min" || m.key == "max" || m.key == "max_velocity") {
				v, bound := finite(m.value, m.key), finite(old, m.key)
				if m.key == "min" && v < bound || m.key != "min" && v > bound {
					unit := "rad"
					if m.key == "max_velocity" {
						unit = "rad/s"
					}
					refuseRobot("robot_limit_widened", "axis %s %s requested %g %s exceeds model bound %g %s", name, m.key, v, unit, bound, unit)
				}
			}
			base.set(m.key, m.value)
		}
		axis := object{{"name", name}, {"profile", pname}, {"slave_position", cell.get("slave_position")}, {"type", "rotary"}, {"wire_counts_per_rev", wireCounts}, {"wire_revs_per_axis_rev", wireRatio}, {"sign", joint.get("sign")}, {"startup", startup}, {"require_home", home}, {"coordinate_evidence_mode", evidence}, {"limits", base}}
		if robotAcceleration != nil {
			axis.set("max_acceleration", robotAcceleration)
			motorRPM := finite(base.get("max_velocity"), "robot velocity") * finite(ratio.get("numerator"), "gear numerator") / finite(ratio.get("denominator"), "gear denominator") * 60 / (2 * math.Pi)
			// Validate/report the actual integer motor-count cap, avoiding false
			// rated-speed warnings from rounded URDF radians near exactly 3000 rpm.
			countsPerSecond, _ := strconv.ParseFloat(driveSpeedLimitCounts(motorRPM, axis, profile), 64)
			axis.set("robot_motor_rpm", json.Number(pythonFloat(countsPerSecond/motorEncoderCountsPerRev(axis, profile)*60)))
		}
		axis.set("motor_encoder_counts_per_rev", json.Number(encoder))
		if battery != "" {
			axis.set("encoder_battery", battery)
		}
		axis.set("gear_ratio", ratio)
		for _, field := range cell {
			if field.key == "collision_watchdog" || field.key == "brake_override" || field.key == "brake_transition_guard" || field.key == "brake_opening_guard" || field.key == "reference_verified_delta_restore" || field.key == "observe" || field.key == "startup_trial" {
				axis.set(field.key, field.value)
			}
		}
		axes = append(axes, axis)
		binding.Axes = append(binding.Axes, RobotAxisBinding{name, pname, asString(joint.get("kind")), joint.get("urdf_joint"), asBool(joint.get("cartesian_participates")), encoder})
	}
	for name := range joints {
		if !used[name] && !absent[name] {
			refuseRobot("robot_joint_missing", "definition joint %s has no cell axis; declare robot.absent_joints explicitly", name)
		}
	}
	expanded := append(object{}, machine...)
	expanded.set("axes", axes)
	return expanded, binding
}

// validateEncoderBattery requires a sourced per-joint A6 fact, preventing a
// profile default or an omitted cell axis from silently claiming a battery.
func validateEncoderBattery(joint, profile object) string {
	a6 := startupSchema(profile).optional("a6ec_encoder_position_tracking_mode", nil) != nil
	value := joint.optional("encoder_battery", nil)
	if value == nil && !a6 {
		return ""
	}
	if value != "present" && value != "absent" {
		reject("joint %s encoder_battery must be present or absent", asString(joint.get("name")))
	}
	if strings.TrimSpace(asString(joint.optional("encoder_battery_source", ""))) == "" {
		reject("joint %s encoder_battery needs _source provenance", asString(joint.get("name")))
	}
	if !a6 {
		return ""
	}
	return asString(value)
}
