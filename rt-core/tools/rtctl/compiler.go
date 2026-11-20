// Package rtctl compiles machine configurations without contacting drives or
// issuing SDOs. The native daemon still validates the resulting private format.
package rtctl

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path"
	"path/filepath"
	"sort"
	"strconv"
	"strings"

	"rosieos/rt-core/ipcclient"
)

// Compiled contains the oracle's identity and native inputs before output paths
// are appended, preventing a deployment directory from changing the identity.
type Compiled struct {
	configurationIdentity     string
	baseConfigurationIdentity string
	resources                 resourceManifest
	resourceData              map[string][]byte
	SHA256                    string
	MachineSHA256             string
	DeploymentSHA256          string
	MachineIdentity           string
	DeploymentIdentity        string
	CoordinateIdentitySHA256  []string
	CoordinateIdentity        []string
	Argv                      []string
	AxisProfiles              string
	CellIO                    string         `json:",omitempty"`
	IOTerminal                *inventoryAxis `json:",omitempty"`
	// JogJerk is the machine-level jog_jerk jerk_rad_s3 compiled onto every
	// axis, zero when the machine omits it. Planners that hold this digest
	// (OLP one-off moves) bound their own jerk by it; it is the same value
	// natively bounding jog and Halt ramps, not a second number.
	JogJerk  float64       `json:",omitempty"`
	Robot    *RobotBinding `json:",omitempty"`
	Warnings []string      `json:",omitempty"`

	// tuningIdentity is the catalog entries a machine uses, bound into the
	// configuration identity; nil keeps a pre-catalog machine's identity.
	// tuningKeys feeds compiled tuning.json and is in no identity.
	tuningIdentity any
	tuningKeys     []string
}

// knownFields validates a closed object and declares its template contract.
func knownFields(value object, allowed ...string) {
	for _, member := range value {
		found := false
		for _, key := range allowed {
			found = found || member.key == key
		}
		if !found {
			reject("unknown field %s", member.key)
		}
	}
}

// Unverified bring-up defaults mirror native Options. Millisecond bounds match
// native unsigned counters, except WaitBeforeSAFEOPms crosses a signed IgH API.
var busBringupSettings = []struct {
	key, flag, identity string
	fallback            any
	maximum             uint64
}{
	{"startup_passive_ms", "--startup-passive-ms", "startup_passive_ms", json.Number("0"), 0xffffffff},
	{"explicit_pdo_config", "--explicit-pdo-config", "explicit_pdo_config", false, 0},
	{"disable_output_watchdog", "--disable-output-watchdog", "disable_output_watchdog", false, 0},
	{"no_dc", "--no-dc", "use_dc", false, 0},
	{"wait_before_safeop_ms", "--wait-before-safeop-ms", "wait_before_safeop_ms", json.Number("250"), 0x7fffffff},
	{"preop_safeop_timeout_ms", "--preop-safeop-timeout-ms", "preop_to_safeop_timeout_ms", json.Number("5000"), 0xffffffff},
	{"safeop_op_timeout_ms", "--safeop-op-timeout-ms", "safeop_to_op_timeout_ms", json.Number("5000"), 0xffffffff},
}

// validateBusBringup admits only the seven machine policies and their string
// measurement notes. Eliding defaults and notes preserves legacy argv/digests;
// rejecting nulls, misspellings and coercions prevents silent policy fallback.
func validateBusBringup(machine object) (args []string, normalized object) {
	block, ok := machine.optional("bus_bringup", object{}).(object)
	if !ok {
		reject("bus_bringup must be an object of machine startup policies")
	}
	known := map[string]bool{}
	for _, setting := range busBringupSettings {
		known[setting.key] = true
	}
	for _, field := range block {
		if known[field.key] {
			continue
		}
		if key, note := strings.CutSuffix(field.key, "_unverified"); note && known[key] {
			if _, ok := field.value.(string); !ok {
				reject("bus_bringup.%s must be a string bench measurement note", field.key)
			}
			continue
		}
		reject("unknown bus_bringup field %q; supported machine policies: startup_passive_ms, explicit_pdo_config, disable_output_watchdog, no_dc, wait_before_safeop_ms, preop_safeop_timeout_ms, safeop_op_timeout_ms (and sibling <key>_unverified notes)", field.key)
	}
	for _, setting := range busBringupSettings {
		value := block.optional(setting.key, setting.fallback)
		label := "bus_bringup." + setting.key
		if setting.maximum == 0 {
			if _, ok := value.(bool); !ok {
				reject("%s must be a boolean (default false; machine semantics)", label)
			}
		} else {
			value = json.Number(integer(value, 0, setting.maximum,
				label+" (milliseconds; default "+string(setting.fallback.(json.Number))+" ms; 0 disables the override)"))
		}
		if value == setting.fallback {
			continue
		}
		normalized = append(normalized, member{setting.key, value})
		args = append(args, setting.flag)
		if setting.maximum != 0 {
			args = append(args, string(value.(json.Number)))
		}
	}
	return args, normalized
}

// Compile validates JSON and profiles and reproduces the frozen Python output.
// Invalid input returns an error before any output files or hardware are touched.
func Compile(machineJSON []byte, profilesDir string) (result Compiled, err error) {
	defer validationBoundary(&err)
	machine := asObject(decode(machineJSON))
	validateBrakeOverrides(machine)
	for _, value := range asArray(machine.get("axes")) {
		for _, field := range asObject(value) {
			if field.key == "brake_override" {
				uniqueRobotJSON(machineJSON)
			}
		}
	}
	if machine.optional("robot", nil) != nil || machine.optional("bus", nil) != nil || machine.optional("control", nil) != nil {
		uniqueRobotJSON(machineJSON)
	}
	expanded, robot := expandRobot(machine, profilesDir)
	if robot != nil {
		result = compile(expanded, robot.profiles, loadTuningCatalogs(profilesDir))
		// Bind the original cell bytes and the pinned definition to planning
		// identity. Native motion identity still describes the effective axes.
		result.configurationIdentity = canonical(configurationInputs(machine, robot.profiles, result.tuningIdentity))
		result.SHA256 = identityDigest(result.configurationIdentity)
		compileResources(&result, expanded, robot.profiles, robot, profilesDir)
		robot.profiles = nil
		// Resource bytes are captured; the source checkout location is no longer
		// part of the returned binding and must not break package relocation.
		robot.repository = ""
		result.Robot = robot
		compileCellIO(expanded, profilesDir, &result)
		return result, nil
	}
	profiles := []any{}
	for _, value := range asArray(machine.get("axes")) {
		axis := asObject(value)
		name := token(axis.get("profile"))
		data, err := os.ReadFile(filepath.Join(profilesDir, name+".json"))
		if err != nil {
			return Compiled{}, fmt.Errorf("profile %s: %w", name, err)
		}
		profiles = append(profiles, asObject(decode(data)))
	}
	result = compile(machine, profiles, loadTuningCatalogs(profilesDir))
	compileResources(&result, machine, profiles, nil, profilesDir)
	compileCellIO(machine, profilesDir, &result)
	return result, nil
}

// configurationInputs is the hashed configuration input: machine JSON, its
// drive profiles and, once a machine uses a non-pre-catalog tuning key or an
// observe list, the tuning-catalog entries it names.
func configurationInputs(machine object, profiles []any, tuning any) object {
	inputs := object{{"machine", machine}, {"profiles", profiles}}
	if tuning != nil {
		inputs = append(inputs, member{"tuning_catalog", tuning})
	}
	return inputs
}

func compile(machine object, profiles []any, catalogs []*tuningCatalog) Compiled {
	validateBrakeOverrides(machine)
	if !numericEqual(machine.optional("schema_version", nil), 1) {
		reject("machine schema_version must be 1")
	}
	axes := asArray(machine.get("axes"))
	if len(axes) < 1 || len(axes) > 16 {
		reject("machine must have 1..16 axes")
	}
	cycle := integer(machine.get("cycle_ns"), 250000, 10000000, "cycle_ns")
	cycleValue, _ := strconv.ParseUint(cycle, 10, 64)
	lateness := integer(machine.get("max_cycle_lateness_ns"), 1, 249999999, "max_cycle_lateness_ns")
	bringupArgs, bringup := validateBusBringup(machine)
	controlArgs := validateControlTiming(machine)
	busArgs := validateBus(machine)
	supervision := validateDriveSupervision(machine)
	lagAllowance := validateTrackingLagAllowance(machine)
	jogJerk, jogJerkValue := validateJogJerk(machine, axes, profiles, cycleValue)
	var positions, counts, ratios, signs, kinds, leads, lines []string
	var warnings []string
	var coordinateIdentities, coordinateDigests []string
	names, occupied := map[string]bool{}, map[string]bool{}
	var tuning tuningUse
	verifyStorage := asBool(machine.optional("verify_parameter_storage", false))
	for i, value := range axes {
		axis := asObject(value)
		name := token(axis.get("name"))
		position := integer(axis.get("slave_position"), 0, 65535, "slave_position")
		if names[name] || occupied[position] {
			reject("axis names and slave positions must be unique")
		}
		names[name], occupied[position] = true, true
		positions = append(positions, position)
		profile := asObject(profiles[i])
		for _, retired := range []string{"feedback_counts_wrap", "command_counts_wrap", "position_semantics"} {
			if profile.optional(retired, nil) != nil {
				reject("axis %s drive profile declares retired field %s", name, retired)
			}
		}
		for _, retired := range []string{"a6ec_rotation_mode_gear_ratio_numerator",
			"a6ec_rotation_mode_gear_ratio_denominator", "a6ec_rotation_mode_reference_running_direction"} {
			if asObject(profile.optional("startup_defaults", object{})).optional(retired, nil) != nil ||
				asObject(profile.optional("startup_schema", object{})).optional(retired, nil) != nil {
				reject("axis %s drive profile declares retired startup key %s", name, retired)
			}
		}
		validateCoordinateInputs(axis, profile)
		drive := asObject(profile.get("ethercat"))
		if truth(profile.optional("simulation_only", false)) && machine.optional("backend", nil) != "simulation" {
			reject("simulation profile cannot be used with live hardware")
		}
		quantum := integer(drive.get("dc_quantum_ns"), 0, 0xffffffff, "DC quantum")
		quantumValue, _ := strconv.ParseUint(quantum, 10, 64)
		if bringup.optional("no_dc", false) != true && quantumValue != 0 && cycleValue%quantumValue != 0 {
			reject("cycle incompatible with axis %s", name)
		}
		rxSync := integer(drive.get("rx_sync"), 0, 15, "RX sync")
		txSync := integer(drive.get("tx_sync"), 0, 15, "TX sync")
		rx, tx := asArray(drive.get("rx_layout")), asArray(drive.get("tx_layout"))
		if rxSync == txSync || len(rx)+len(tx) > 32 {
			reject("invalid PDO layout/sync managers")
		}
		field := func(key, value string) { lines = append(lines, key+"="+value) }
		field("axis", strconv.Itoa(i))
		field("name", name)
		collision, warning := renderCollisionWatchdog(axis, profile, tx)
		if collision != "" {
			field("collision_watchdog", collision)
		}
		if warning != "" {
			warnings = append(warnings, warning)
		}
		field("vendor_id", integer(drive.get("vendor_id"), 1, 0xffffffff, "vendor"))
		field("product_code", integer(drive.get("product_code"), 1, 0xffffffff, "product"))
		field("revision_no", integer(drive.get("revision_no"), 0, 0xffffffff, "revision"))
		field("rx_pdo", integer(drive.get("rx_pdo"), 1, 65535, "RX PDO"))
		field("tx_pdo", integer(drive.get("tx_pdo"), 1, 65535, "TX PDO"))
		field("rx_sync", rxSync)
		field("tx_sync", txSync)
		restoreAssignment, restoreAssignmentOK := drive.optional("restore_assignment_on_exit", false).(bool)
		if !restoreAssignmentOK {
			reject("restore_assignment_on_exit must be boolean")
		}
		if restoreAssignment {
			if bringup.optional("explicit_pdo_config", false) == true {
				reject("restore_assignment_on_exit requires fixed PDO configuration")
			}
			field("restore_assignment_on_exit", "1")
		}
		field("dc_quantum_ns", quantum)
		field("dc_assign_activate", integer(drive.get("dc_assign_activate"), 0, 65535, "DC assign"))
		field("rx_layout", renderPDO(rx, rxWidths, []string{"cw", "target_pos", "mode"}))
		field("tx_layout", renderPDO(tx, txWidths, []string{"sw", "pos", "mode_disp"}))
		homeSign := integer(profile.get("home_truth_sign"), -1, 1, "home sign")
		if homeSign == "0" {
			reject("home sign must be -1 or +1")
		}
		field("home_truth_sign", homeSign)
		field("native_home", renderHome(profile.optional("native_home", nil)))
		rejectCurrentPoseReference(axis, profile)
		if restore := renderReferenceVerifiedDeltaRestore(axis, profile); restore != "" {
			field("reference_verified_delta_restore", restore)
		}
		if retained := renderRetainedHomeRestore(axis, profile); retained != "" {
			field("retained_home_restore", retained)
		}
		feedback := asArray(profile.optional("absolute_feedback", []any{}))
		if len(feedback) > 16 {
			reject("absolute feedback fields must be unique, at most 16")
		}
		keys := map[string]bool{}
		var rendered []string
		for _, value := range feedback {
			f := asObject(value)
			key := token(f.get("key"))
			if keys[key] {
				reject("absolute feedback fields must be unique, at most 16")
			}
			keys[key] = true
			rendered = append(rendered, key+"|"+objectSpec(f))
		}
		field("absolute_feedback", strings.Join(rendered, ";"))
		pair := asString(profile.optional("absolute_pair_field", ""))
		if pair != "" && !keys[pair] {
			reject("paired field must be in absolute_feedback")
		}
		field("absolute_pair_field", pair)
		if monitor := renderMonitor(profile); monitor != "" {
			field("monitor", monitor)
		}
		if assignment := renderExternalEnableInput(profile, axis, tx); assignment != "" {
			field("external_enable_input", assignment)
		}
		catalog := catalogFor(catalogs, profile)
		startupSDOs, written := renderStartup(profile, axis, catalog, &tuning, i)
		startupSDOs, storage, trial := tuning.renderStorage(catalog, profile, axis, verifyStorage, startupSDOs, written)
		field("startup_sdos", startupSDOs)
		if storage != "" {
			field("parameter_storage", storage)
		}
		if trial != "" {
			field("startup_trial", trial)
		}
		if observe := tuning.renderObserve(catalog, profile, axis, machine.optional("observe", nil), written); observe != "" {
			field("startup_observe", observe)
		}
		if axis.optional("encoder_battery", nil) == "absent" {
			field("encoder_battery", "absent")
			tolerance := asObject(profile.get("encoder_battery_absent_alarm_tolerance"))
			knownFields(tolerance, "_source", "error_code", "aux_code")
			field("encoder_battery_absent_alarm", integer(tolerance.get("error_code"), 1, 0xffff, "battery-absent alarm error code")+"|"+
				integer(tolerance.get("aux_code"), 1, 0xffffffff, "battery-absent alarm auxiliary code"))
		}
		coordinateIdentity := renderCoordinateIdentity(machine, axis, profile)
		coordinateIdentities = append(coordinateIdentities, coordinateIdentity)
		coordinateDigest := identityDigest(coordinateIdentity)
		field("coordinate_identity", fmt.Sprintf("%x", coordinateIdentity))
		coordinateDigests = append(coordinateDigests, coordinateDigest)
		limits := asObject(axis.get("limits"))
		kind := asString(axis.get("type"))
		lower, upper := finite(limits.get("min"), "lower limit"), finite(limits.get("max"), "upper limit")
		speed := finite(limits.get("max_velocity"), "velocity limit")
		requireHome := asBool(axis.get("require_home"))
		if lower >= upper || speed <= 0 {
			reject("invalid limits or require_home")
		}
		if requireHome && !truth(profile.optional("native_home", nil)) {
			reject("required homing has no configured procedure")
		}
		home := "0"
		if requireHome {
			home = "1"
		}
		field("limits", strings.Join([]string{pythonFloat(lower), pythonFloat(upper), pythonFloat(speed), home}, "|"))
		var tracking []string
		for _, key := range []string{"jog_acceleration", "max_target_lead", "following_error", "completion_tolerance"} {
			v := finite(limits.get(key), key)
			if v <= 0 {
				reject("tracking limits must be positive")
			}
			tracking = append(tracking, pythonFloat(v))
		}
		tracking = append(tracking, integer(limits.get("following_error_timeout_ms"), 1, 1000, "following error timeout"),
			integer(limits.get("completion_timeout_ms"), 1, 0xffffffff, "completion timeout"))
		field("tracking", strings.Join(tracking, "|"))
		countsPerUnit := finite(axis.get("wire_counts_per_rev"), "wire counts/rev") *
			finite(axis.get("wire_revs_per_axis_rev"), "wire revs/axis rev") / (2 * math.Pi)
		if kind == "linear" {
			countsPerUnit = finite(axis.get("wire_counts_per_rev"), "wire counts/rev") *
				finite(axis.get("wire_revs_per_axis_rev"), "wire revs/axis rev") /
				finite(axis.get("lead_m_per_rev"), "lead")
		}
		marginUnits := math.Max(finite(limits.get("max_target_lead"), "max_target_lead"),
			math.Max(finite(limits.get("following_error"), "following_error"),
				finite(limits.get("completion_tolerance"), "completion_tolerance")))
		// One count covers outward rounding at the signed PDO boundary.
		spanCounts := (math.Max(math.Abs(lower), math.Abs(upper))+marginUnits)*countsPerUnit + 1
		if !(spanCounts > 0) || math.IsInf(spanCounts, 0) || math.IsNaN(spanCounts) ||
			spanCounts > math.MaxInt32 {
			reject("axis %s limits plus tracking margin exceed signed 32-bit continuous count span", name)
		}
		for _, member := range axis {
			if member.key == "tracking_lag_allowance" {
				reject("axis %s declares tracking_lag_allowance; it is one machine-level value for every axis", name)
			}
		}
		if lagAllowance != "" {
			field("tracking_lag_allowance", lagAllowance)
		}
		// Robot-bound axes carry their description's acceleration explicitly.
		// Standalone synthetic fixtures may declare it in their profile instead.
		// An explicit malformed/null axis bound must never fall back.
		accelerationSource := profile
		for _, entry := range axis {
			if entry.key == "max_acceleration" {
				accelerationSource = object{entry}
				break
			}
		}
		for _, entry := range accelerationSource {
			if entry.key == "max_acceleration" {
				acceleration := finite(entry.value, "max_acceleration")
				if acceleration <= 0 {
					reject("max_acceleration must be positive")
				}
				field("max_acceleration", pythonFloat(acceleration))
			}
		}
		// Unverified profile windows mirror the Python compiler and native counters.
		jogValue := func(key, fallback string) any {
			var value any = json.Number(fallback)
			if block := profile.optional("jog", nil); block != nil {
				value = asObject(block).optional(key, value)
			}
			if block := axis.optional("jog", nil); block != nil {
				value = asObject(block).optional(key, value)
			}
			return value
		}
		cycleNS, _ := strconv.ParseUint(cycle, 10, 64)
		field("jog", integer(jogValue("arrest_ns", "200000000"), 1, 0xffff*cycleNS, "jog arrest ns")+"|"+
			integer(jogValue("quick_stop_ns", "300000000"), 1, 0xffff*cycleNS, "jog quick stop ns"))
		if axis.optional("jog_jerk", nil) != nil {
			reject("axis %s declares jog_jerk; it is one machine-level value for every axis", name)
		}
		if jogJerk != "" {
			field("jog_jerk", jogJerk)
		}
		// Brake admission mirrors the frozen compiler: the profile block is overridden per key
		// by the axis block; malformed overrides are rejected rather than silently
		// dropping the drive's hold/release protection.
		brakeValue := func(key string, fallback any) any {
			value := fallback
			if block := profile.optional("brake", nil); block != nil {
				value = asObject(block).optional(key, value)
			}
			if block := axis.optional("brake", nil); block != nil {
				value = asObject(block).optional(key, value)
			}
			return value
		}
		brakePresent := asBool(brakeValue("present", false))
		if axis.optional("brake_override", nil) != nil {
			brakePresent = false
		}
		if axis.optional("reference_verified_delta_restore", nil) != nil && !brakePresent {
			reject("reference_verified_delta_restore requires a present brake")
		}
		brakeGravity := asBool(brakeValue("gravity_axis", false))
		// C05.13/C05.10 defaults are unverified until measured on the drive and relay.
		brakeRelease := integer(brakeValue("release_delay_ms", json.Number("100")), 0, 0xffffffff, "brake release delay")
		brakeHold := integer(brakeValue("hold_delay_ms", json.Number("100")), 0, 0xffffffff, "brake hold delay")
		// Unverified legacy default: one count. A robot machine declares the
		// settled hold tolerance once in joint radians (joint_unit_tolerances),
		// converted per axis by gear ratio, replacing the profile's count. A
		// joint-unit hold tolerance applies brake-present or not, as the count did.
		if block := axis.optional("brake", nil); block != nil && machineJointUnit(machine, "hold_displacement_tolerance_rad") != nil &&
			asObject(block).optional("hold_displacement_tolerance_counts", nil) != nil {
			reject("axis %s declares a hold tolerance and the machine declares one for every axis; declare it once", asString(axis.get("name")))
		}
		brakeTolerance := jointUnitOrCounts(machine, axis, "hold_displacement_tolerance_rad",
			brakeValue("hold_displacement_tolerance_counts", json.Number("1")), 0, 0x7fffffff, "brake hold_displacement_tolerance_counts")
		brakeSignal, brakeBit := "-", "0"
		if released := brakeValue("released_signal", nil); released != nil {
			signal := asObject(released)
			brakeSignal = token(signal.get("semantic"))
			brakeBit = integer(signal.get("bit"), 0, 31, "brake released bit")
			bitValue, _ := strconv.ParseUint(brakeBit, 10, 64)
			mapped := false
			for _, value := range tx {
				entry := asObject(value)
				if asString(entry.get("semantic")) != brakeSignal {
					continue
				}
				bits, _ := strconv.ParseUint(integer(entry.get("bits"), 1, 64, "PDO bits"), 10, 64)
				mapped = bitValue < bits
				break
			}
			if !mapped {
				reject("brake released_signal must name a mapped tx PDO bit")
			}
		}
		if !brakePresent && (brakeGravity || brakeSignal != "-") {
			reject("gravity axis or brake feedback requires a present brake")
		}
		flag := func(b bool) string {
			if b {
				return "1"
			}
			return "0"
		}
		brakeSpec := flag(brakePresent) + "|" + brakeRelease + "|" + brakeHold + "|" + flag(brakeGravity) + "|" + brakeSignal + "|" + brakeBit
		if brakeTolerance != "1" {
			brakeSpec += "|" + brakeTolerance
		}
		field("brake", brakeSpec)
		// Transition limits are a machine-axis commissioning choice, never a
		// drive-profile default or an implicit expansion of the held tolerance.
		if profile.optional("brake_transition_guard", nil) != nil {
			reject("brake_transition_guard must be declared on the machine axis")
		}
		for _, member := range axis {
			if member.key == "brake_transition_guard" && member.value == nil {
				reject("brake_transition_guard must be an explicit object")
			}
		}
		transitionGuard, transitionMachineLevel := axisOrMachineGuard(machine, axis, "brake_transition_guard")
		if transitionGuard != nil {
			if !brakePresent {
				reject("brake_transition_guard requires a present brake")
			}
			block := asObject(transitionGuard)
			if !transitionMachineLevel {
				knownFields(block, "max_displacement_counts", "max_velocity_counts_per_s", "target_stationary_ms", "settling_ms", "timeout_ms")
			}
			keys := []string{"max_displacement_counts", "max_velocity_counts_per_s", "target_stationary_ms", "settling_ms", "timeout_ms"}
			parts := renderBrakeGuardParts(axis, block, "brake_transition_guard", keys, transitionMachineLevel)
			values := make([]uint64, len(keys))
			for i := range keys {
				values[i], _ = strconv.ParseUint(parts[i], 10, 64)
			}
			hold, _ := strconv.ParseUint(brakeHold, 10, 64)
			tolerance, _ := strconv.ParseUint(brakeTolerance, 10, 64)
			if values[0] < tolerance || values[4] <= hold || values[3] > values[4]-hold {
				reject("brake_transition_guard cap or timing inconsistent with brake hold")
			}
			velocityMapped := false
			for _, item := range tx {
				entry := asObject(item)
				velocityMapped = velocityMapped || asString(entry.get("semantic")) == "velocity_actual" && integer(entry.get("bits"), 1, 64, "PDO bits") == "32"
			}
			if !velocityMapped {
				reject("brake_transition_guard requires mapped32-bit velocity_actual")
			}
			field("brake_transition", strings.Join(parts, "|"))
		}
		// Enable acquisition is distinct from settled holding. Opt-in belongs to
		// one machine axis and is included in the compiled identity.
		if profile.optional("brake_opening_guard", nil) != nil {
			reject("brake_opening_guard must be declared on the machine axis")
		}
		for _, member := range axis {
			if member.key == "brake_opening_guard" && member.value == nil {
				reject("brake_opening_guard must be an explicit object")
			}
		}
		if guard, machineLevel := axisOrMachineGuard(machine, axis, "brake_opening_guard"); guard != nil {
			if !brakePresent || transitionGuard == nil {
				reject("brake_opening_guard requires a present brake and explicit closing transition")
			}
			block := asObject(guard)
			if !machineLevel {
				knownFields(block, "max_displacement_counts", "max_velocity_counts_per_s", "timeout_ms")
			}
			keys := []string{"max_displacement_counts", "max_velocity_counts_per_s", "timeout_ms"}
			parts := renderBrakeGuardParts(axis, block, "brake_opening_guard", keys, machineLevel)
			values := make([]uint64, len(keys))
			for i := range keys {
				values[i], _ = strconv.ParseUint(parts[i], 10, 64)
			}
			tolerance, _ := strconv.ParseUint(brakeTolerance, 10, 64)
			if values[0] < tolerance {
				reject("brake_opening_guard displacement is smaller than held tolerance")
			}
			if values[2] > 500 {
				reject("brake_opening_guard enable timeout exceeds 500ms")
			}
			field("brake_opening", strings.Join(parts, "|"))
		}
		if override := axis.optional("brake_override", nil); override != nil {
			// UTF-8 hex preserves arbitrary reason text without injecting private record lines.
			field("brake_override_reason_hex", hex.EncodeToString([]byte(asString(asObject(override).get("reason")))))
		}
		mode := asString(axis.get("coordinate_evidence_mode"))
		if mode != "multi_turn" {
			reject("axis %s coordinate_evidence_mode must be multi_turn; single-turn evidence is retired", name)
		}
		coordinate := profile.optional("coordinate_evidence", nil)
		havePosition, haveValidity := false, false
		for _, value := range tx {
			switch asObject(value).get("semantic") {
			case "continuous_pos":
				havePosition = true
			case "continuous_valid":
				haveValidity = true
			}
		}
		if coordinate != nil {
			c := asObject(coordinate)
			source := asString(c.optional("source", "pdo"))
			sampled := source == "a6ec_sdo"
			var anchor object
			if sampled {
				anchor = asObject(c.get("anchor_source"))
				for _, key := range []string{"name", "identity_name"} {
					token(anchor.get(key))
				}
				anchorVersion := integer(anchor.get("version"), 1, 0xffffffff, "anchor source version")
				if startupSchema(profile).optional("a6ec_encoder_position_tracking_mode", nil) != nil &&
					anchorVersion != "3" {
					reject("axis %s A6-EC linear anchor source version must be 3", name)
				}
				field("coordinate_anchor_source", strings.Join([]string{
					integer(anchor.get("index"), 1, 65535, "anchor source index"),
					integer(anchor.get("low_subindex"), 0, 255, "anchor source low subindex"),
					integer(anchor.get("high_subindex"), 1, 255, "anchor source high subindex"),
					token(anchor.get("name")), token(anchor.get("identity_name")), anchorVersion,
				}, "|"))
				if havePosition || haveValidity || c.optional("validity_mask", nil) != nil || c.optional("validity_value", nil) != nil {
					reject("fixed PDO evidence derives validity from SDO acquisition; continuous PDO validity is unavailable")
				}
				if bringup.optional("explicit_pdo_config", false) == true {
					reject("sampled SDO evidence requires fixed PDO configuration")
				}
			} else if source != "pdo" || !havePosition || !haveValidity {
				reject("coordinate evidence requires continuous position and validity PDOs")
			}
			var values []string
			// Unverified policy limits copied from the oracle; quantities are counts, counts/s, ns and validity bits.
			for _, bound := range []struct {
				key string
				min int64
				max uint64
			}{
				{"maximum_physical_speed_counts_per_second", 1, 0xffffffffffffffff},
				{"maximum_acquisition_age_ns", 1, 0xffffffffffffffff},
			} {
				values = append(values, integer(c.get(bound.key), bound.min, bound.max, bound.key))
			}
			// Encoder/PDO agreement bounds: the profile's counts, or the
			// machine's joint radians converted per axis.
			values = append(values, feedbackErrorCounts(machine, axis, c),
				jointUnitOrCounts(machine, axis, "maximum_anchor_error_rad", c.get("maximum_anchor_error_counts"),
					0, math.MaxUint64, "maximum_anchor_error_counts"))
			if sampled {
				if asString(anchor.get("identity_name")) == "raw_multi_turn_encoder" {
					// Independent-encoder Start allowance floor: the profile's
					// counts or the machine's joint angle converted by this axis's
					// gearing; required because native has no default.
					field("start_alignment_floor", jointUnitOrCounts(machine, axis, "start_alignment_floor_rad",
						c.get("start_alignment_floor_counts"), 1, 0x7fffffff, "start_alignment_floor_counts"))
				}
				cadence := integer(c.get("sample_interval_ns"), 1, 0xffffffffffffffff, "sample interval ns")
				interval, _ := strconv.ParseUint(cadence, 10, 64)
				age, _ := strconv.ParseUint(values[1], 10, 64)
				if interval >= age {
					reject("SDO sample interval must be below the acquisition age bound")
				}
				// Unverified memory cap: ceil(age/cycle)+2 <= 131072 history entries.
				if age/cycleValue > 131070 || (age/cycleValue == 131070 && age%cycleValue != 0) {
					reject("sampled PDO history exceeds 131072 entries (ceil(age_ns/cycle_ns)+2)")
				}
				if interval > ^uint64(0)/2 {
					reject("two sample cadences exceed the evidence-loss grace range")
				}
				values = append(values, "a6ec_sdo", cadence)
			} else {
				values = append(values, integer(c.get("validity_mask"), 1, 0xffff, "validity_mask"), integer(c.get("validity_value"), 0, 0xffff, "validity_value"))
				mask, _ := strconv.ParseUint(values[4], 10, 64)
				valid, _ := strconv.ParseUint(values[5], 10, 64)
				if valid & ^mask != 0 {
					reject("coordinate validity value must be contained in its mask")
				}
			}
			field("coordinate_evidence", strings.Join(values, "|"))
		} else if havePosition || haveValidity {
			reject("continuous PDOs require a coordinate evidence policy")
		}
		counts = append(counts, integer(axis.get("wire_counts_per_rev"), 1, 0x7fffffff, "wire counts/rev"))
		ratio := finite(axis.get("wire_revs_per_axis_rev"), "wire revs/axis rev")
		if ratio <= 0 {
			reject("wire scaling must be positive")
		}
		if kind != "rotary" && kind != "linear" {
			reject("axis type must be rotary or linear")
		}
		lead := finite(axis.optional("lead_m_per_rev", json.Number("0")), "lead")
		if kind == "linear" && lead <= 0 {
			reject("linear axes need a positive lead")
		}
		sign := integer(axis.get("sign"), -1, 1, "axis sign")
		if sign == "0" {
			reject("axis sign must be -1 or +1")
		}
		ratios, signs = append(ratios, pythonFloat(ratio)), append(signs, sign)
		kinds, leads = append(kinds, kind), append(leads, pythonFloat(lead))
		for _, member := range axis {
			if member.key == "drive_supervision" {
				reject("axis %s declares drive_supervision; it is one machine-level block for every axis", name)
			}
		}
		if supervision != nil {
			field("drive_supervision", supervision.render(axis))
		}
		if alarmReset := renderDriveAlarmReset(profile); alarmReset != "" {
			field("drive_alarm_reset", alarmReset)
		}
	}
	identityMachine := object{}
	for _, field := range machine {
		if field.key != "bus_bringup" {
			identityMachine = append(identityMachine, field)
		}
	}
	if len(bringup) != 0 {
		identityMachine = append(identityMachine, member{"bus_bringup", bringup})
	}
	tuningIdentity := tuning.finish(len(axes))
	identity := canonical(configurationInputs(identityMachine, profiles, tuningIdentity))
	digest := fmt.Sprintf("%x", sha256.Sum256([]byte(identity)))
	args := []string{"--num-axes", strconv.Itoa(len(axes)), "--cycle-ns", cycle}
	for _, group := range []struct {
		key    string
		values []string
	}{
		{"slave-positions", positions}, {"counts-per-rev", counts}, {"gear-ratio", ratios},
		{"sign", signs}, {"axis-type", kinds}, {"lead-m-per-rev", leads},
	} {
		args = append(args, "--"+group.key, strings.Join(group.values, ","))
	}
	rpm := driveSpeedLimitMotorRPM(machine)
	allRobotRates, maximumRobotRPM := true, 0.0
	for _, axis := range axes {
		if asObject(axis).optional("robot_motor_rpm", nil) != nil {
			maximumRobotRPM = math.Max(maximumRobotRPM, finite(asObject(axis).get("robot_motor_rpm"), "robot motor rpm"))
		} else {
			allRobotRates = false
		}
	}
	if allRobotRates {
		rpm = maximumRobotRPM
	}
	for i, axis := range axes {
		axisRPM := finite(asObject(axis).optional("robot_motor_rpm", json.Number(pythonFloat(rpm))), "motor rpm")
		if warning := validateDriveMotorRPM(axisRPM, machine.optional("max_motor_rpm", nil) != nil, asObject(axis), asObject(profiles[i])); warning != "" {
			warnings = append(warnings, warning)
		}
	}
	if machine.optional("max_motor_rpm", nil) == nil {
		var caps []string
		for i, axis := range axes {
			axisRPM := finite(asObject(axis).optional("robot_motor_rpm", json.Number(pythonFloat(rpm))), "motor rpm")
			caps = append(caps, driveSpeedLimitCounts(axisRPM, asObject(axis), asObject(profiles[i])))
		}
		args = append(args, "--drive-speed-limit-counts-per-s", strings.Join(caps, ","))
	}
	runtime := asObject(machine.get("runtime"))
	args = append(args, "--rt-cpu", integer(runtime.get("cpu"), 0, 1023, "RT CPU"), "--rt-priority", integer(runtime.get("priority"), 1, 99, "RT FIFO priority"))
	args = append(args, "--max-rpm", pythonFloat(rpm), "--max-cycle-lateness-ns", lateness)
	args = append(args, bringupArgs...)
	args = append(args, controlArgs...)
	// Deployment retention controls host RAM/eMMC use, preserving machine/Home
	// identity when diagnostic history changes. Omitted defaults preserve argv parity.
	telemetry := asObject(runtime.optional("telemetry", object{}))
	for _, field := range telemetry {
		if field.key != "retention_ms" && field.key != "dump_count" && field.key != "directory" && field.key != "dump_bytes" {
			reject("unknown runtime.telemetry field %q", field.key)
		}
	}
	// Capture size is schema-derived; reject retention that exceeds the RAM cap.
	// Unverified bounds mirror native admission: 100..3600000 ms, 2 GiB, 1..100 dumps.
	retention := integer(telemetry.optional("retention_ms", json.Number("100000")), 100, 3600000, "telemetry retention_ms")
	retentionValue, _ := strconv.ParseUint(retention, 10, 64)
	if 4096+((retentionValue*1000000+cycleValue-1)/cycleValue)*uint64(8+ipcclient.CycleCaptureRecordV2AxOffset+ipcclient.CycleCaptureAxisV2Bytes*len(axes)) > 2*1024*1024*1024 {
		reject("telemetry ring exceeds 2 GiB resident ceiling")
	}
	dumps := integer(telemetry.optional("dump_count", json.Number("20")), 1, 100, "telemetry dump_count")
	if len(telemetry) != 0 {
		args = append(args, "--telemetry-retention-ms", retention, "--telemetry-dump-count", dumps)
	}
	for _, field := range telemetry {
		if field.key == "dump_bytes" {
			// Derived minimum: 2*262144 header/index bytes + one 5392-byte
			// maximum-axis record = 529680, matching daemon admission for all axes.
			args = append(args, "--telemetry-dump-bytes", integer(field.value, ipcclient.TelemetryDumpMinBytes, ipcclient.TelemetryDumpMaxBytes, "telemetry dump_bytes (two headers and one 16-axis record)"))
		}
	}
	for _, field := range telemetry {
		if field.key != "directory" {
			continue
		}
		directory, ok := field.value.(string)
		if !ok || !path.IsAbs(directory) || path.Clean(directory) == "/" || strings.ContainsRune(directory, 0) {
			reject("telemetry directory must be an absolute non-root path")
		}
		args = append(args, "--telemetry-directory", path.Clean(directory))
	}
	args = append(args, busArgs...)
	result := Compiled{SHA256: digest, Argv: args, AxisProfiles: strings.Join(lines, "\n") + "\n", Warnings: warnings, JogJerk: jogJerkValue}
	result.configurationIdentity = identity
	result.tuningIdentity, result.tuningKeys = tuningIdentity, tuningKeys(catalogs, profiles)
	result.CoordinateIdentity, result.CoordinateIdentitySHA256 = coordinateIdentities, coordinateDigests
	result.MachineIdentity = machineIdentity(args, result.AxisProfiles)
	result.DeploymentIdentity = deploymentIdentity(machine)
	result.MachineSHA256 = identityDigest(result.MachineIdentity)
	result.DeploymentSHA256 = identityDigest(result.DeploymentIdentity)
	return result
}

// Identity partition (keys in the canonical native field table):
//
// | Input fields | Digest | Reason |
// | axis list/order, names, slave positions, units, scaling, gearing, sign, wrap | machine | Defines the coordinate frame and drive assignment. |
// | profile PDO/SDO/DC/home/absolute/brake/coordinate blocks | machine | Defines drive behaviour and evidence validity; ordered transactions stay ordered. |
// | limits, tracking/completion tolerances, jog durations, motor cap | machine | Defines admitted motion and stopping. |
// | drive_supervision timeouts and per-axis count bounds | machine | Ends Arm attempts, warns on disarm motion and bounds Home dwell. |
// | profile fault_reset classification and timing | machine | Selects which drive alarms an operator Reset sends the drive's fault reset for. |
// | tracking_lag_allowance ms, identical on every axis | machine | Widens the target-lead, following-error and collision-watchdog following-error bounds with commanded speed. |
// | jog_jerk jerk and smoothing window, identical on every axis | machine | Bounds jog and Halt-ramp jerk and lengthens their stops. |
// | cycle_ns, max_cycle_lateness_ns, DC/watchdog/startup policies | machine | Cycle and failure timing affect motion safety. |
// | bus_bringup startup_passive_ms, wait_before_safeop_ms, preop_safeop_timeout_ms, safeop_op_timeout_ms | machine | Drive firmware convergence governs readiness and evidence validity. |
// | bus_bringup explicit_pdo_config, disable_output_watchdog, no_dc | machine | PDO mapping, output failure response and clock synchronization are drive semantics. |
// | runtime CPU/priority/housekeeping, host, NIC, sockets, pair ID, service users | deployment | Selects the host/process placement without changing machine semantics. |
// | backend and other non-motion metadata | deployment | Deployment selection/annotations; profile semantics are already in machine. |
// | original machine JSON and profiles, including metadata | configuration | Frozen compatibility identity; new bus_bringup defaults and measurement notes are elided. |
//
// Canonical identity uses sorted keys, one key=value per LF line, %.17g finite
// doubles, decimal integers and ordered native descriptor blocks. Deployment
// leaves use JSON-escaped strings; array indexes preserve order. The native
// mirror reconstructs machine fields from parsed values, never from file bytes.
func identityText(fields map[string]string) string {
	keys := make([]string, 0, len(fields))
	for key := range fields {
		keys = append(keys, key)
	}
	sort.Strings(keys)
	var out strings.Builder
	for _, key := range keys {
		fmt.Fprintf(&out, "%s=%s\n", key, fields[key])
	}
	return out.String()
}

func identityDigest(text string) string { return fmt.Sprintf("%x", sha256.Sum256([]byte(text))) }

func identityFloat(value float64) string { return strconv.FormatFloat(value, 'g', 17, 64) }

// machineIdentity records effective native inputs and their derived scaling;
// keeping requested and capped velocity prevents a cap from hiding a limit edit.
func machineIdentity(args []string, profiles string) string {
	options := map[string]string{}
	for i := 0; i < len(args); i++ {
		key := args[i]
		switch key {
		case "--explicit-pdo-config", "--disable-output-watchdog", "--no-dc":
			options[key] = "1"
		default:
			i++
			options[key] = args[i]
		}
	}
	// Unverified defaults mirror Options; changing native policy must change this table.
	fields := map[string]string{"split_domains_per_axis": "0", "startup_skip_domain_queue_ms": "0"}
	if options["--bus-unknown-slaves"] == "hold" || options["--bus-unused-slaves"] != "" {
		fields["bus_unknown_slaves"] = "refuse"
		if options["--bus-unknown-slaves"] == "hold" {
			fields["bus_unknown_slaves"] = "hold"
		}
		fields["bus_unused_slaves"] = options["--bus-unused-slaves"]
	}
	for _, key := range []string{"control-max-grant-lease-ns", "control-max-jog-input-age-ns"} {
		if value := options["--"+key]; value != "" {
			fields[strings.ReplaceAll(key, "-", "_")] = value
		}
	}
	for _, setting := range busBringupSettings {
		value, present := options[setting.flag]
		if setting.maximum == 0 {
			value = "0"
			if present != (setting.key == "no_dc") {
				value = "1"
			}
		} else if !present {
			value = string(setting.fallback.(json.Number))
		}
		fields[setting.identity] = value
	}
	for _, key := range []string{"num-axes", "cycle-ns", "max-cycle-lateness-ns"} {
		fields[strings.ReplaceAll(key, "-", "_")] = options["--"+key]
	}
	number := func(text string) float64 {
		value, err := strconv.ParseFloat(text, 64)
		if err != nil {
			reject("invalid identity number")
		}
		return value
	}
	rpm := number(options["--max-rpm"])
	fields["max_rpm"] = identityFloat(rpm)
	axis := ""
	for _, line := range strings.Split(strings.TrimSuffix(profiles, "\n"), "\n") {
		key, value, _ := strings.Cut(line, "=")
		if key == "axis" {
			axis = "axis." + value + "."
			fields[axis+"coordinate_evidence"] = ""
			continue
		}
		if key == "limits" || key == "tracking" {
			parts := strings.Split(value, "|")
			count := 3
			if key == "tracking" {
				count = 4
			}
			for i := 0; i < count; i++ {
				parts[i] = identityFloat(number(parts[i]))
			}
			value = strings.Join(parts, "|")
		}
		// Single numeric profile fields are normalised the same way so the daemon's
		// recomputation from its parsed double reproduces the identical text.
		if key == "max_acceleration" {
			value = identityFloat(number(value))
		}
		if key != "coordinate_identity" {
			fields[axis+key] = value
		}
	}
	n, _ := strconv.Atoi(options["--num-axes"])
	for i := 0; i < n; i++ {
		prefix := fmt.Sprintf("axis.%d.", i)
		for _, key := range []string{"slave-positions", "counts-per-rev", "gear-ratio", "sign", "axis-type", "lead-m-per-rev"} {
			value := strings.Split(options["--"+key], ",")[i]
			if key == "gear-ratio" || key == "lead-m-per-rev" {
				value = identityFloat(number(value))
			}
			name := strings.ReplaceAll(key, "-", "_")
			if key == "slave-positions" {
				name = "slave_position"
			}
			fields[prefix+name] = value
		}
		// Measured mathematical constant: radians per revolution, rounded to binary64.
		divisor := 6.28318530717958647692
		unit := "rad"
		if fields[prefix+"axis_type"] == "linear" {
			divisor = number(fields[prefix+"lead_m_per_rev"])
			unit = "m"
		}
		cpr := number(fields[prefix+"counts_per_rev"])
		cpu := (cpr * number(fields[prefix+"gear_ratio"])) / divisor
		cap := math.Min(4294967295, math.Max(1, math.Round((rpm/60)*cpr)))
		if caps, ok := options["--drive-speed-limit-counts-per-s"]; ok {
			cap = number(strings.Split(caps, ",")[i])
		}
		fields[prefix+"position_unit"] = unit
		fields[prefix+"counts_per_unit"] = identityFloat(cpu)
		fields[prefix+"max_profile_vel_counts_per_s"] = strconv.FormatUint(uint64(cap), 10)
		fields[prefix+"effective_max_velocity"] = identityFloat(math.Min(number(strings.Split(fields[prefix+"limits"], "|")[2]), cap/cpu))
	}
	return identityText(fields)
}

// renderExternalEnableInput requires an explicit diagnostic assignment with
// provenance; startup SDOs cannot silently invent a cyclic enable observation.
func renderExternalEnableInput(profile, axis object, tx []any) string {
	assignment := asObject(axis.optional("diagnostics", object{})).optional("external_enable_input", nil)
	if assignment == nil {
		drive := asObject(profile.get("ethercat"))
		if numericEqual(drive.get("vendor_id"), 4194304) && numericEqual(drive.get("product_code"), 1813) {
			settings := append(object(nil), asObject(profile.optional("startup_defaults", object{}))...)
			for _, m := range asObject(axis.optional("startup", object{})) {
				settings.set(m.key, m.value)
			}
			schema := startupSchema(profile)
			for _, m := range settings {
				spec := schema.optional(m.key, nil)
				if spec == nil {
					continue
				}
				item := asObject(spec)
				obj := asObject(item.get("object"))
				// A6-EC chapter 11, 2004h/C04: DI1..DI5 function 1 is S-ON.
				for _, subindex := range []int{1, 5, 9, 13, 17} {
					if item.get("type") == "u16" && numericEqual(m.value, 1) &&
						numericEqual(obj.get("index"), 8196) && numericEqual(obj.get("subindex"), subindex) {
						reject("S-ON startup assignment requires diagnostics.external_enable_input with bit_index_source and polarity_source")
					}
				}
			}
		}
		return ""
	}
	c := asObject(assignment)
	for _, m := range c {
		switch m.key {
		case "bit_index", "bit_index_source", "polarity", "polarity_source":
		default:
			reject("unknown external_enable_input field %s", m.key)
		}
	}
	for _, key := range []string{"bit_index_source", "polarity_source"} {
		if strings.TrimSpace(asString(c.get(key))) == "" {
			reject("external_enable_input requires %s", key)
		}
	}
	var width int64
	for _, entry := range tx {
		p := asObject(entry)
		if p.get("semantic") == "di" {
			// Native digital-input diagnostics use an unsigned 32-bit PDO.
			width, _ = strconv.ParseInt(integer(p.get("bits"), 1, 32, "digital-input width"), 10, 64)
		}
	}
	if width == 0 {
		reject("external_enable_input requires a mapped digital-input PDO")
	}
	bit := integer(c.get("bit_index"), 0, uint64(width-1), "external_enable_input.bit_index")
	polarity := asString(c.get("polarity"))
	if polarity != "active_high" && polarity != "active_low" {
		reject("external_enable_input.polarity must be active_high or active_low")
	}
	return bit + "|" + polarity
}

// deploymentIdentity includes every non-machine field so an added host setting
// cannot silently disappear from deployment identity. JSON leaf escaping prevents
// a hostname, socket path or metadata value from injecting canonical lines.
func deploymentIdentity(machine object) string {
	fields := map[string]string{}
	var visit func(string, any)
	visit = func(key string, value any) {
		switch v := value.(type) {
		case object:
			fields[key] = "{}"
			for _, m := range v {
				visit(key+"."+asciiQuote(m.key), m.value)
			}
		case []any:
			fields[key] = "[]"
			for i, item := range v {
				visit(fmt.Sprintf("%s.%d", key, i), item)
			}
		case json.Number:
			if strings.ContainsAny(string(v), ".eE") {
				fields[key] = identityFloat(finite(v, "deployment"))
			} else {
				fields[key] = canonical(v)
			}
		default:
			fields[key] = canonical(v)
		}
	}
	for _, m := range machine {
		switch m.key {
		case "schema_version", "axes", "cycle_ns", "max_cycle_lateness_ns", "max_motor_rpm", "drive_speed_limit_motor_rpm", "bus_bringup", "bus", "io",
			"brake_transition_guard", "brake_opening_guard", "joint_unit_tolerances", "drive_supervision",
			"tracking_lag_allowance", "jog_jerk", "observe", "verify_parameter_storage":
			continue
		}
		visit(asciiQuote(m.key), m.value)
	}
	return identityText(fields)
}

// WriteConfig writes the SHA directory and Python-compatible argv bytes. It
// compiles first so rejected input cannot publish a configuration directory.
func WriteConfig(machinePath, profilesDir, outputDir string) (directory string, argv []string, err error) {
	data, err := os.ReadFile(machinePath)
	if err != nil {
		return "", nil, err
	}
	compiled, err := Compile(data, profilesDir)
	if err != nil {
		return "", nil, err
	}
	return writeCompiled(compiled, outputDir)
}

// writeCompiled publishes precisely the validated snapshot, preventing a second
// configuration read from changing the backend or deployment after admission.
func writeCompiled(compiled Compiled, outputDir string) (directory string, argv []string, err error) {
	outputDir, err = filepath.Abs(outputDir)
	if err != nil {
		return "", nil, err
	}
	if err = os.MkdirAll(outputDir, 0o755); err != nil {
		return "", nil, err
	}
	outputDir, err = filepath.EvalSymlinks(outputDir)
	if err != nil {
		return "", nil, err
	}
	directory = filepath.Join(outputDir, compiled.SHA256)
	if err = os.MkdirAll(directory, 0o755); err != nil {
		return "", nil, err
	}
	path := filepath.Join(directory, "axes.conf")
	if err = writeCompiledResources(compiled, directory); err != nil {
		return "", nil, err
	}
	if err = os.WriteFile(path, []byte(compiled.AxisProfiles), 0o644); err != nil {
		return "", nil, err
	}
	if compiled.Robot != nil {
		data, encodeErr := json.MarshalIndent(compiled.Robot, "", "  ")
		if encodeErr != nil {
			return "", nil, encodeErr
		}
		if err = os.WriteFile(filepath.Join(directory, "robot.json"), append(data, '\n'), 0o644); err != nil {
			return "", nil, err
		}
	}
	for name, contents := range map[string]string{"machine.identity": compiled.MachineIdentity, "deployment.identity": compiled.DeploymentIdentity} {
		if err = os.WriteFile(filepath.Join(directory, name), []byte(contents), 0o644); err != nil {
			return "", nil, err
		}
	}
	if len(compiled.tuningKeys) != 0 {
		// The configuration-only deploy reads this to allow per-axis values
		// only for class-tuning keys; it is not part of any identity.
		data, encodeErr := json.MarshalIndent(struct {
			TuningKeys []string `json:"tuning_keys"`
		}{compiled.tuningKeys}, "", "  ")
		if encodeErr != nil {
			return "", nil, encodeErr
		}
		if err = os.WriteFile(filepath.Join(directory, "tuning.json"), append(data, '\n'), 0o644); err != nil {
			return "", nil, err
		}
	}
	coordinate, encodeErr := json.MarshalIndent(struct {
		SHA256 []string `json:"coordinate_identity_sha256"`
		Fields []string `json:"coordinate_identity"`
	}{compiled.CoordinateIdentitySHA256, compiled.CoordinateIdentity}, "", "  ")
	if encodeErr != nil {
		return "", nil, encodeErr
	}
	if err = os.WriteFile(filepath.Join(directory, "coordinate.identity.json"), append(coordinate, '\n'), 0o644); err != nil {
		return "", nil, err
	}
	argv = append(append([]string(nil), compiled.Argv...),
		"--machine-sha256", compiled.MachineSHA256,
		"--deployment-sha256", compiled.DeploymentSHA256,
		"--axis-profiles", path, "--configuration-hash", compiled.SHA256)
	quoted := make([]string, len(argv))
	for i, arg := range argv {
		quoted[i] = "  " + asciiQuote(arg)
	}
	err = os.WriteFile(filepath.Join(directory, "argv.json"), []byte("[\n"+strings.Join(quoted, ",\n")+"\n]\n"), 0o644)
	return directory, argv, err
}
