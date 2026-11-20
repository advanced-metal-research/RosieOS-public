package rtctl

import (
	"encoding/json"
	"fmt"
	"math"
	"strconv"
	"strings"
)

// validateCoordinateInputs rejects omitted declarations before other rendering
// can hide the affected axis/key or synthesize a physical encoder identity.
func validateCoordinateInputs(axis, profile object) {
	name := asString(axis.get("name"))
	require := func(value any, key, input string) {
		if value == nil {
			reject("axis %s coordinate identity missing %s (%s)", name, key, input)
		}
	}
	for _, field := range []struct{ key, input string }{
		{"direction.sign", "sign"}, {"wire.counts_per_rev", "wire_counts_per_rev"},
		{"wire.revs_per_axis_rev", "wire_revs_per_axis_rev"}, {"evidence.mode", "coordinate_evidence_mode"},
		{"encoder.counts_per_motor_rev", "motor_encoder_counts_per_rev"},
	} {
		require(axis.optional(field.input, nil), field.key, field.input)
	}
	drive := asObject(profile.get("ethercat"))
	for _, key := range []string{"vendor_id", "product_code", "revision_no"} {
		require(drive.optional(key, nil), "drive."+key, "ethercat."+key)
	}
	a6 := startupSchema(profile).optional("a6ec_encoder_position_tracking_mode", nil) != nil
	if a6 {
		startup := asObject(profile.optional("startup_defaults", object{}))
		overrides := asObject(axis.optional("startup", object{}))
		require(overrides.optional("a6ec_encoder_position_tracking_mode",
			startup.optional("a6ec_encoder_position_tracking_mode", nil)),
			"tracking.mode", "a6ec_encoder_position_tracking_mode")
	}
	if axis.optional("require_home", false) == true {
		require(profile.optional("native_home", nil), "home.policy", "native_home")
	}
}

// renderCoordinateIdentity binds every axis to resolved count semantics and
// refuses incomplete inputs. Pair authority belongs to the adapter's record.
func renderCoordinateIdentity(machine, axis, profile object) string {
	drive := asObject(profile.get("ethercat"))
	a6 := startupSchema(profile).optional("a6ec_encoder_position_tracking_mode", nil) != nil
	fields := map[string]string{
		"anchor.source":          "none|0",
		"drive.vendor_id":        integer(drive.get("vendor_id"), 1, 0xffffffff, "vendor"),
		"drive.product_code":     integer(drive.get("product_code"), 1, 0xffffffff, "product"),
		"drive.revision_no":      integer(drive.get("revision_no"), 0, 0xffffffff, "revision"),
		"direction.sign":         integer(axis.get("sign"), -1, 1, "sign"),
		"wire.counts_per_rev":    integer(axis.get("wire_counts_per_rev"), 1, 0x7fffffff, "wire counts/rev"),
		"wire.revs_per_axis_rev": identityFloat(finite(axis.get("wire_revs_per_axis_rev"), "wire revs/axis rev")),
		"evidence.mode":          asString(axis.get("coordinate_evidence_mode")),
		"home.policy":            coordinateHome(profile),
	}
	if coordinate := profile.optional("coordinate_evidence", nil); coordinate != nil {
		if configured := asObject(coordinate).optional("anchor_source", nil); configured != nil {
			source := asObject(configured)
			fields["anchor.source"] = token(source.get("identity_name")) + "|" +
				integer(source.get("version"), 1, 0xffffffff, "anchor source version")
		} else {
			fields["anchor.source"] = "none|0"
		}
	}
	startup := asObject(profile.optional("startup_defaults", object{}))
	overrides := asObject(axis.optional("startup", object{}))
	gear := asObject(axis.optional("gear_ratio",
		object{{"numerator", json.Number("1")}, {"denominator", json.Number("1")}}))
	fields["gear.numerator"] = integer(gear.get("numerator"), 1, 0x7fffffff, "gear numerator")
	fields["gear.denominator"] = integer(gear.get("denominator"), 1, 0x7fffffff, "gear denominator")
	if a6 {
		value := overrides.optional("a6ec_encoder_position_tracking_mode",
			startup.optional("a6ec_encoder_position_tracking_mode", nil))
		fields["tracking.mode"] = integer(value, 0, 65535, "a6ec_encoder_position_tracking_mode")
	} else {
		var trackingDefault any
		if machine.optional("backend", nil) == "simulation" {
			// Unverified synthetic plant: no internal gearing; PDO counts track continuously.
			trackingDefault = "continuous"
		}
		fields["tracking.mode"] = token(axis.optional("position_tracking_mode", trackingDefault))
	}
	if fields["gear.numerator"] == "0" || fields["gear.denominator"] == "0" {
		reject("coordinate identity requires positive gearing")
	}
	fields["encoder.counts_per_motor_rev"] = integer(axis.get("motor_encoder_counts_per_rev"), 1, 0x7fffffff, "motor encoder counts/rev")
	return identityText(fields)
}

// coordinateHome retains the native method and offset object semantics, omitting
// sequencing and diagnostic steps which do not define the Home count origin.
func coordinateHome(profile object) string {
	home := profile.optional("native_home", nil)
	if home == nil {
		return "none"
	}
	c := asObject(home)
	text := "truth|" + objectSpec(asObject(c.get("truth_source"))) + ";sign|" + integer(profile.get("home_truth_sign"), -1, 1, "home sign")
	for _, value := range asArray(c.get("transaction")) {
		step := asObject(value)
		op := asString(step.get("op"))
		if op != "write_sdo" {
			continue
		}
		if !numericEqual(step.get("index"), 0x6098) && !numericEqual(step.get("index"), 0x607c) {
			continue
		}
		text += ";" + op + "|" + objectSpec(step)
		min, max := scalarRange(asString(step.get("type")))
		text += "|" + integer(step.get("value"), min, max, "Home value")
	}
	return text
}

// driveSpeedLimitMotorRPM preserves the legacy wire-frame value for one release;
// conflicting fields fail closed instead of silently selecting a powered cap.
func driveSpeedLimitMotorRPM(machine object) float64 {
	legacy, explicit := false, false
	for _, field := range machine {
		legacy = legacy || field.key == "max_motor_rpm"
		explicit = explicit || field.key == "drive_speed_limit_motor_rpm"
	}
	if legacy && explicit {
		reject("max_motor_rpm conflicts with drive_speed_limit_motor_rpm; remove the legacy field")
	}
	key := "drive_speed_limit_motor_rpm"
	if legacy {
		key = "max_motor_rpm"
	}
	// Owner decision 2026-09-15: A6M rated motor speed is 3000 rpm.
	rpm := finite(machine.optional(key, json.Number("3000")), key)
	if rpm <= 0 {
		reject("%s must be a positive motor speed cap", key)
	}
	return rpm
}

// A6-EC manual, ch. 11 C03.27/C03.28: 6000 rpm default speed limits;
// sec. 12.7.1 A6M motor tables: 3000 rpm rated, 6000 rpm maximum.
// https://www.omc-stepperonline.com/download/A6-EC_series_servo_drive_manual.pdf
// Unverified hardware qualification: use the motor maximum as a hard ceiling,
// even though the drive parameter range extends to 8000 rpm.
const (
	a6MotorRatedRPM   = 3000
	a6MotorMaximumRPM = 6000
)

// validateDriveMotorRPM checks the physical motor frame even for legacy fields,
// preventing wire-frame compatibility from bypassing the A6 motor maximum.
func validateDriveMotorRPM(rpm float64, legacy bool, axis, profile object) string {
	drive := asObject(profile.get("ethercat"))
	if !numericEqual(drive.get("vendor_id"), 4194304) || !numericEqual(drive.get("product_code"), 1813) {
		return ""
	}
	if legacy {
		rpm *= finite(axis.get("wire_counts_per_rev"), "wire counts/rev") / motorEncoderCountsPerRev(axis, profile)
	}
	name := asString(axis.get("name"))
	if rpm > a6MotorMaximumRPM {
		reject("axis %s motor speed cap %g rpm exceeds A6-EC maximum 6000 rpm (manual ch. 11 C03.27/C03.28; sec. 12.7.1)", name, rpm)
	}
	if rpm > a6MotorRatedRPM {
		return fmt.Sprintf("axis %s motor speed cap %g rpm exceeds A6M rated 3000 rpm (manual sec. 12.7.1); maximum 6000 rpm", name, rpm)
	}
	return ""
}

// motorEncoderCountsPerRev removes output-shaft gearing from the position scale;
// using that geared scale for a motor speed would overstate the powered cap.
func motorEncoderCountsPerRev(axis, profile object) float64 {
	counts := integer(axis.get("wire_counts_per_rev"), 1, 0x7fffffff, "wire counts/rev")
	encoder, _ := strconv.ParseFloat(counts, 64)
	return encoder
}

// driveSpeedLimitCounts emits motor rpm in the drive's counts/s frame for either
// ratio path; one 17-bit motor revolution is 131072 counts regardless of gearing.
func driveSpeedLimitCounts(rpm float64, axis, profile object) string {
	cap := math.Round(rpm / 60 * motorEncoderCountsPerRev(axis, profile))
	if math.IsInf(cap, 0) || math.IsNaN(cap) || cap < 1 || cap > math.MaxUint32 {
		reject("axis %s drive_speed_limit_motor_rpm is not representable in 0x607F counts/s", asString(axis.get("name")))
	}
	return strconv.FormatUint(uint64(cap), 10)
}

// Unverified compatibility tables: PDO widths in bits, copied from the frozen compiler.
var rxWidths = map[string]int{"cw": 16, "target_pos": 32, "target_vel": 32, "target_torque": 16,
	"mode": 8, "tp_func": 16, "max_profile_vel": 32}
var txWidths = map[string]int{"sw": 16, "pos": 32, "mode_disp": 8, "err": 16, "manufacturer_err": 32,
	"velocity_actual": 32, "following_error": 32, "torque": 16, "di": 32, "tp_status": 16, "tp_pos1": 32, "tp_pos2": 32,
	"bus_voltage": 16, "load_rate": 16, "position_error": 32, "multi_turn_lo": 32,
	"multi_turn_hi": 32, "igbt_temp": 16, "motor_temp": 16,
	"drive_not_ready": 16, "motor_not_rotating": 16, "continuous_pos": 32, "continuous_valid": 16}

func renderPDO(entries []any, allowed map[string]int, required []string) string {
	seen, objects := map[string]bool{}, map[string]bool{}
	var rendered []string
	for _, value := range entries {
		e := asObject(value)
		name := token(e.get("semantic"))
		index := integer(e.get("index"), 1, 65535, "PDO index")
		sub := integer(e.get("subindex"), 0, 255, "PDO subindex")
		width, ok := allowed[name]
		if !ok || !numericEqual(e.get("bits"), width) {
			reject("unsupported PDO semantic/width: %s", name)
		}
		obj := index + "|" + sub
		if seen[name] || objects[obj] {
			reject("duplicate PDO semantic/object")
		}
		seen[name], objects[obj] = true, true
		rendered = append(rendered, name+"|"+obj+"|"+canonical(e.get("bits")))
	}
	for _, key := range required {
		if !seen[key] {
			reject("missing required PDO: %s", key)
		}
	}
	return strings.Join(rendered, ";")
}

func scalarRange(s string) (int64, uint64) {
	switch s {
	case "u8":
		return 0, 255
	case "i8":
		return -128, 127
	case "u16":
		return 0, 65535
	case "i16":
		return -32768, 32767
	case "u32":
		return 0, 4294967295
	case "i32":
		return -2147483648, 2147483647
	default:
		reject("unsupported SDO scalar: %s", s)
	}
	return 0, 0
}

func objectSpec(obj object) string {
	index := integer(obj.get("index"), 1, 65535, "SDO index")
	sub := integer(obj.get("subindex"), 0, 255, "SDO subindex")
	scalar := asString(obj.get("type"))
	scalarRange(scalar)
	return index + "|" + sub + "|" + scalar
}

// maxMonitorEntries bounds a drive profile's monitor list; the daemon keeps
// the same capacity (include/axis_profiles.hpp kMaxMonitorPerAxis).
const maxMonitorEntries = 8

// renderMonitor renders the drive profile's monitor list: drive objects the
// daemon reads over the mailbox while running and publishes per axis, for
// example the A6-EC heatsink temperature (docs/decisions/0001). The value is
// raw / 10^decimals in unit. "key|index|sub|type|decimals|unit|interval_ms;...",
// or "" when the profile has none (no line is emitted).
func renderMonitor(profile object) string {
	entries := asArray(profile.optional("monitor", []any{}))
	if len(entries) > maxMonitorEntries {
		reject("monitor lists at most %d objects", maxMonitorEntries)
	}
	keys, objects := map[string]bool{}, map[string]bool{}
	var rendered []string
	for _, value := range entries {
		m := asObject(value)
		key := token(m.get("key"))
		spec := objectSpec(m)
		if keys[key] || objects[spec] {
			reject("monitor keys and objects must be unique")
		}
		keys[key], objects[spec] = true, true
		decimals := integer(m.get("decimals"), 0, 6, "monitor decimals")
		unit := token(m.get("unit"))
		// Unverified floor: one mailbox read per object per second at most,
		// so monitoring never competes with alarm and startup reads.
		interval := integer(m.get("interval_ms"), 1000, 3600000, "monitor interval_ms")
		rendered = append(rendered, strings.Join([]string{key, spec, decimals, unit, interval}, "|"))
	}
	return strings.Join(rendered, ";")
}

func renderHome(value any) string {
	if value == nil {
		return ""
	}
	c := asObject(value)
	parts := []string{
		"steady_state_mode|" + integer(c.get("steady_state_mode"), 1, 127, "mode"),
		"commissioning_mode|" + integer(c.get("commissioning_mode"), 1, 127, "mode"),
		"truth_source|" + objectSpec(asObject(c.get("truth_source"))),
	}
	for _, value := range asArray(c.get("transaction")) {
		step := asObject(value)
		op := asString(step.get("op"))
		text := "op|" + op
		switch op {
		case "set_mode", "restore_mode":
			text += "|" + integer(step.get("value"), 1, 127, "mode")
		case "write_sdo", "wait_sdo":
			text += "|" + objectSpec(step)
			lo, hi := scalarRange(asString(step.get("type")))
			text += "|" + integer(step.get("value"), lo, hi, "SDO value")
		case "controlword_sequence":
			values := asArray(step.get("values"))
			if len(values) < 1 || len(values) > 8 {
				reject("controlword sequence length")
			}
			var words []string
			for _, value := range values {
				words = append(words, integer(value, 0, 65535, "controlword"))
			}
			text += "|" + strings.Join(words, ",")
		case "wait_statusword":
			text += "|" + integer(step.get("all_set_mask"), 0, 65535, "status mask")
			text += "|" + integer(step.get("all_clear_mask"), 0, 65535, "status mask")
		case "refresh_truth", "release_service_override":
		default:
			reject("unsupported home operation %s", op)
		}
		parts = append(parts, text)
	}
	return strings.Join(parts, ";")
}

// renderStartup emits "key|type|index|sub|value;..." for the profile's own
// startup schema (gearing, tracking mode, direction) and for tuning-catalog
// keys. Omitting a key writes nothing, so the drive keeps whatever it holds
// (A6-EC stores SDO writes); an explicit value is written at every startup
// and verified by readback. The second result names the keys written.
func renderStartup(profile, axis object, catalog *tuningCatalog, use *tuningUse, axisIndex int) (string, map[string]bool) {
	settings := append(object(nil), asObject(profile.optional("startup_defaults", object{}))...)
	for _, m := range asObject(axis.optional("startup", object{})) {
		settings.set(m.key, m.value)
	}
	schema := startupSchema(profile)
	name := asString(axis.get("name"))
	written := map[string]bool{}
	var rendered []string
	for _, m := range settings {
		written[m.key] = true
		spec := schema.optional(m.key, nil)
		if spec == nil {
			e := use.entry(catalog, profile, m.key, name)
			if e.access != "read_write" {
				reject("axis %s %s (%s) is observe-only in its catalog", name, m.key, e.code)
			}
			value := tuningRawValue(e, m.value, name)
			use.record(e, axisIndex, value)
			rendered = append(rendered, fmt.Sprintf("%s|%s|%d|%d|%d", token(m.key), e.scalar, e.index, e.subindex, value))
			continue
		}
		item := asObject(spec)
		if m.value == nil {
			reject("explicitly configure startup setting %s", m.key)
		}
		scalar := asString(item.get("type"))
		lo, hi := scalarRange(scalar)
		value := integer(m.value, lo, hi, m.key)
		v, _ := strconv.ParseInt(value, 10, 64)
		if float64(v) < finite(item.get("min"), "startup minimum") || float64(v) > finite(item.get("max"), "startup maximum") {
			reject("startup setting %s outside schema range", m.key)
		}
		obj := asObject(item.get("object"))
		effective := ""
		if item.optional("effective", nil) != nil {
			if item.get("effective") != "power_cycle" {
				reject("startup setting %s has unsupported effective policy", m.key)
			}
			effective = "|power_cycle"
		}
		rendered = append(rendered, fmt.Sprintf("%s|%s|%s|%s|%s%s", token(m.key), scalar,
			integer(obj.get("index"), 1, 65535, "index"), integer(obj.get("subindex"), 0, 255, "subindex"), value, effective))
	}
	if len(rendered) > startupWritesPerAxisMaximum {
		reject("axis %s has %d startup settings; at most %d per axis", name, len(rendered), startupWritesPerAxisMaximum)
	}
	use.totalWrite += len(rendered)
	return strings.Join(rendered, ";"), written
}

// startupSchema is the profile's own startup objects. Tuning keys, including
// the A6-EC speed feedforward, come from the drive's tuning catalog.
func startupSchema(profile object) object {
	return asObject(profile.optional("startup_schema", object{}))
}
