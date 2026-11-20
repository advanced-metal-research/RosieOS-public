package rtctl

import (
	"path/filepath"
	"sort"
	"strconv"
	"strings"
)

// validateBus binds the non-axis topology to native identity and requires an
// explicit bench rationale, preventing a robot from silently ignoring motors.
func validateBus(machine object) (args []string) {
	raw := machine.optional("bus", nil)
	if raw == nil {
		for _, field := range machine {
			if field.key == "bus" {
				reject("bus must be an object")
			}
		}
		return nil
	}
	bus, ok := raw.(object)
	if !ok {
		reject("bus must be an object")
	}
	busFields(bus, "unknown_slaves unknown_slaves_note unused_slaves hold_on_robot_acknowledged hold_on_robot_note")
	policy := bus.optional("unknown_slaves", "refuse")
	if policy != "refuse" && policy != "hold" {
		reject("bus.unknown_slaves must be refuse or hold")
	}
	ack, ok := bus.optional("hold_on_robot_acknowledged", false).(bool)
	if !ok {
		reject("bus.hold_on_robot_acknowledged must be boolean")
	}
	if policy == "hold" {
		busNote(bus, "unknown_slaves_note")
		if robot, ok := machine.optional("robot", nil).(object); ok {
			model := ""
			if description, ok := robot.optional("robot_description", nil).(object); ok {
				model = filepath.Base(asString(description.get("path")))
			}
			if model != "bench_nine_motors_1to1" && model != "bench_one_motor_1to1" {
				if !ack {
					reject("bus hold on a robot requires bus.hold_on_robot_acknowledged=true and bus.hold_on_robot_note")
				}
				busNote(bus, "hold_on_robot_note")
			}
		}
		args = append(args, "--bus-unknown-slaves", "hold")
	}
	if ack {
		busNote(bus, "hold_on_robot_note")
	}
	occupied := map[string]bool{}
	for _, value := range asArray(machine.get("axes")) {
		occupied[integer(asObject(value).get("slave_position"), 0, 65535, "slave_position")] = true
	}
	unused, ok := bus.optional("unused_slaves", []any{}).([]any)
	if !ok {
		reject("bus.unused_slaves must be an array")
	}
	// Unverified capacity: bound non-axis inventory to 256 entries in native snapshots.
	if len(unused) > 256 {
		reject("bus.unused_slaves exceeds 256 entries")
	}
	type entry struct {
		position int
		text     string
	}
	entries := []entry{}
	for _, value := range unused {
		slave, ok := value.(object)
		if !ok {
			reject("bus.unused_slaves entry must be an object")
		}
		busFields(slave, "slave_position vendor_id product_code revision note")
		busNote(slave, "note")
		position := integer(slave.get("slave_position"), 0, 65535, "bus.unused_slaves.slave_position")
		if occupied[position] {
			reject("bus.unused_slaves duplicate or axis position %s", position)
		}
		occupied[position] = true
		vendor := integer(slave.get("vendor_id"), 1, 0xffffffff, "bus.unused_slaves.vendor_id")
		product := integer(slave.get("product_code"), 1, 0xffffffff, "bus.unused_slaves.product_code")
		revision := "-"
		for _, field := range slave {
			if field.key == "revision" {
				revision = integer(field.value, 0, 0xffffffff, "bus.unused_slaves.revision")
			}
		}
		n, _ := strconv.Atoi(position)
		entries = append(entries, entry{n, strings.Join([]string{position, vendor, product, revision}, "|")})
	}
	sort.Slice(entries, func(i, j int) bool { return entries[i].position < entries[j].position })
	if len(entries) != 0 {
		parts := []string{}
		for _, entry := range entries {
			parts = append(parts, entry.text)
		}
		args = append(args, "--bus-unused-slaves", strings.Join(parts, ";"))
	}
	return args
}

// busFields accepts only documented fields and their textual provenance;
// misspellings must fail instead of selecting a weaker implicit policy.
func busFields(block object, fields string) {
	known := map[string]bool{}
	for _, key := range strings.Fields(fields) {
		known[key] = true
	}
	for _, field := range block {
		if known[field.key] {
			if strings.HasSuffix(field.key, "note") {
				busNote(block, field.key)
			}
			continue
		}
		base, provenance := strings.CutSuffix(field.key, "_source")
		if !provenance {
			base, provenance = strings.CutSuffix(field.key, "_unverified")
		}
		if !provenance || !known[base] {
			reject("unknown bus field %q", field.key)
		}
		busNote(block, field.key)
	}
}

func busNote(block object, key string) {
	note, ok := block.optional(key, nil).(string)
	if !ok || strings.TrimSpace(note) == "" {
		reject("bus.%s requires a nonempty note", key)
	}
}

type inventoryBusSlave struct {
	inventoryIdentity
	Disposition string `json:"disposition"`
}

// inventoryBus classifies only the existing SII scan; no SDO or output is sent
// to non-axis slaves, and unknown or mismatched identities cannot qualify.
func inventoryBus(report *inventoryReport, machine object) int {
	args := validateBus(machine)
	hold := false
	declared := map[uint32]inventoryIdentity{}
	for i := 0; i < len(args); i += 2 {
		if args[i] == "--bus-unknown-slaves" {
			hold = true
			continue
		}
		for _, text := range strings.Split(args[i+1], ";") {
			fields := strings.Split(text, "|")
			values := make([]*uint32, 4)
			for j, value := range fields {
				if value != "-" {
					n, _ := strconv.ParseUint(value, 10, 32)
					v := uint32(n)
					values[j] = &v
				}
			}
			declared[*values[0]] = inventoryIdentity{values[0], values[1], values[2], values[3]}
		}
	}
	positions, err := inventoryPositions(report.Inventory.Stdout)
	if err != nil || report.Inventory.ExitCode != 0 || report.Inventory.Error != "" {
		return 3
	}
	for _, axis := range report.Axes {
		delete(positions, axis.SlavePosition)
	}
	for _, terminal := range report.IO {
		delete(positions, terminal.SlavePosition)
	}
	sii := inventorySIIIdentities(report.Inventory.Stdout)
	status := 0
	for position := range positions {
		identity := sii[position]
		p := position
		identity.SlavePosition = &p
		disposition := "refused_unknown"
		if hold {
			disposition = "held"
		}
		if want, ok := declared[position]; ok {
			disposition = "declared_unused"
			if !sameInventoryNumber(want.VendorID, identity.VendorID) || !sameInventoryNumber(want.ProductCode, identity.ProductCode) || (want.RevisionNo != nil && !sameInventoryNumber(want.RevisionNo, identity.RevisionNo)) {
				disposition = "refused_identity"
			}
			delete(declared, position)
		}
		if strings.HasPrefix(disposition, "refused") && status < 2 {
			status = 2
		}
		// An unknown position already proves refusal. Preserve that mismatch
		// result even when its identity is unavailable; it cannot be admitted.
		if disposition != "refused_unknown" && (identity.VendorID == nil || identity.ProductCode == nil || identity.RevisionNo == nil) {
			disposition = "unreadable"
			status = 3
		}
		report.HeldSlaves = append(report.HeldSlaves, inventoryBusSlave{identity, disposition})
	}
	for _, identity := range declared {
		report.HeldSlaves = append(report.HeldSlaves, inventoryBusSlave{identity, "refused_missing"})
		if status < 2 {
			status = 2
		}
	}
	sort.Slice(report.HeldSlaves, func(i, j int) bool { return *report.HeldSlaves[i].SlavePosition < *report.HeldSlaves[j].SlavePosition })
	return status
}

func sameInventoryNumber(a, b *uint32) bool { return a != nil && b != nil && *a == *b }
