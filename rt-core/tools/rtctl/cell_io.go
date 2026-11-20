package rtctl

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"
)

// Native capacity: two process-image bytes per direction (16 channels), at most
// sixteen configured names per direction (IoStateV1 and the marker mask).
const cellIOMaxChannels = 16

// piControl's process image is 4096 bytes (KUNBUS piControl driver, KB_PI_LEN).
const piControlImageBytes = 4096

var piControlDevicePath = regexp.MustCompile(`^/[A-Za-z0-9_./-]+$`)

// compileCellIO binds the complete terminal policy and profile to configuration,
// leaving coordinate identity unchanged. Missing feedback or expiry fails closed;
// an output without wired feedback must say so (owner decision 2026-10-12).
func compileCellIO(machine object, profilesDir string, result *Compiled) {
	value := machine.optional("io", struct{}{})
	if _, missing := value.(struct{}); missing {
		return
	}
	block := asObject(value)
	knownIOFields(block, "profile", "slave_position", "picontrol_device", "picontrol_output_offset", "picontrol_input_offset", "torch_qualified", "torch_qualification_record", "inputs", "outputs")
	qualified, declared := block.optional("torch_qualified", nil).(bool)
	if !declared {
		reject("P6-05: cell I/O requires explicit torch_qualified")
	}
	if qualified {
		// Human-reviewed hardware qualification is an explicit deployment
		// decision, never inferred from ping, output-image readback or tests.
		record, ok := block.optional("torch_qualification_record", nil).(string)
		if !ok || strings.TrimSpace(record) == "" {
			reject("P6-05: torch execution is not qualified: a reviewed torch_qualification_record is required")
		}
	}
	profileName := token(block.get("profile"))
	data, err := os.ReadFile(filepath.Join(profilesDir, profileName+".json"))
	if err != nil {
		reject("cell I/O profile: %v", err)
	}
	profile := asObject(decode(data))
	var header string
	var outputChannels, inputChannels int
	var terminal *inventoryAxis
	switch {
	case numericEqual(profile.optional("schema_version", nil), 1):
		header, terminal = compileTerminalV1(machine, block, profile, profileName)
		outputChannels, inputChannels = 8, 8
	case numericEqual(profile.optional("schema_version", nil), 2):
		header, outputChannels, inputChannels, terminal = compileIOProfileV2(machine, block, profile, profileName)
	default:
		reject("cell I/O profile %s: unsupported schema_version", profileName)
	}
	parts := []string{header}
	names := map[string]bool{}
	name := func(o object) string {
		n := token(o.get("name"))
		if len(n) > 31 || names[n] {
			reject("duplicate or oversized cell I/O name")
		}
		names[n] = true
		return n
	}
	polarity := func(o object) string {
		switch o.get("polarity") {
		case "active_high":
			return "1"
		case "active_low":
			return "0"
		}
		reject("unknown cell I/O polarity")
		return ""
	}
	channel := func(o object, channels int, label string) string {
		if channels == 0 {
			reject("cell I/O profile %s has no %s channels", profileName, label)
		}
		return integer(o.get("bit"), 0, uint64(channels-1), "cell I/O bit")
	}
	for _, direction := range []string{"inputs", "outputs"} {
		entries := asArray(block.get(direction))
		if len(entries) > cellIOMaxChannels {
			reject("cell I/O exceeds sixteen %s", direction)
		}
		bits := map[string]bool{}
		readbacks := map[string]bool{}
		var rows []string
		for _, value := range entries {
			o := asObject(value)
			if direction == "inputs" {
				knownIOFields(o, "name", "bit", "polarity", "class")
				n := name(o)
				bit := channel(o, inputChannels, "input")
				if bits[bit] {
					reject("duplicate cell I/O %s bit", direction)
				}
				bits[bit] = true
				class := "0"
				switch o.get("class") {
				case "fast":
					class = "1"
				case "supervisory":
				default:
					reject("unknown cell I/O input class")
				}
				rows = append(rows, strings.Join([]string{n, bit, polarity(o), class}, "|"))
				continue
			}
			knownIOFields(o, "name", "bit", "safe_state", "expiry_ns", "class", "readback", "readback_bit", "readback_polarity", "indicator")
			n := name(o)
			bit := channel(o, outputChannels, "output")
			if bits[bit] {
				reject("duplicate cell I/O %s bit", direction)
			}
			bits[bit] = true
			if o.get("safe_state") != false {
				reject("P6-05: output safe_state must be false (OFF)")
			}
			// Unverified maximum independent output lease: 1000000000 ns.
			expiry := integer(o.get("expiry_ns"), 1, 1000000000, "output expiry_ns")
			class := "0"
			switch o.get("class") {
			case "torch":
				class = "1"
			case "process":
			case "indicator":
				class = "2"
			default:
				reject("unknown cell I/O output class")
			}
			rb, rp := "-", "-"
			switch o.optional("readback", "wired") {
			case "wired":
				if inputChannels == 0 {
					reject("cell I/O profile %s has no inputs for readback_bit; declare readback none_unverified", profileName)
				}
				rb = integer(o.get("readback_bit"), 0, uint64(inputChannels-1), "independent input readback_bit")
				if readbacks[rb] {
					reject("duplicate cell I/O readback bit")
				}
				readbacks[rb] = true
				rp = polarity(object{{"polarity", o.get("readback_polarity")}})
			case "none_unverified":
				// Owner decision 2026-10-12: an output may run without wired
				// feedback while no input module is fitted. It is then never
				// observed: no OFF proof before arm, no disagreement fault.
				if o.optional("readback_bit", nil) != nil || o.optional("readback_polarity", nil) != nil {
					reject("cell I/O output %s: readback none_unverified forbids readback_bit and readback_polarity", n)
				}
			default:
				reject("unknown cell I/O output readback")
			}
			fields := []string{n, bit, "0", expiry, class, rb, rp}
			if class == "2" {
				indicator := asObject(o.get("indicator"))
				knownIOFields(indicator, "source", "period_ns", "statusword_mask", "statusword_value")
				source := token(indicator.get("source"))
				period := integer(indicator.get("period_ns"), 1, 60000000000, "indicator period_ns")
				mask := integer(indicator.optional("statusword_mask", json.Number("0")), 0, 65535, "indicator statusword_mask")
				value := integer(indicator.optional("statusword_value", json.Number("0")), 0, 65535, "indicator statusword_value")
				m, _ := strconv.ParseUint(mask, 10, 16)
				v, _ := strconv.ParseUint(value, 10, 16)
				switch source {
				case "drive_power":
					if m == 0 || v == 0 || v & ^m != 0 || period == "1" {
						reject("drive_power indicator needs a nonzero statusword mask/value and flash period >= 2 ns")
					}
				case "arm_pulse":
					if m != 0 || v != 0 {
						reject("arm_pulse indicator cannot declare a statusword mask")
					}
				default:
					reject("unknown indicator source")
				}
				fields = append(fields, source, period, mask, value)
			} else if o.optional("indicator", nil) != nil {
				reject("only indicator outputs may declare indicator policy")
			}
			rows = append(rows, strings.Join(fields, "|"))
		}
		parts = append(parts, strings.Join(rows, ","))
	}
	if qualified {
		torchCount, fastCount := 0, 0
		for _, v := range asArray(block.get("outputs")) {
			o := asObject(v)
			if o.get("class") == "torch" {
				torchCount++
				if o.optional("readback", "wired") != "wired" {
					reject("qualified torch requires independent wired feedback")
				}
			}
		}
		for _, v := range asArray(block.get("inputs")) {
			if asObject(v).get("class") == "fast" {
				fastCount++
			}
		}
		if torchCount != 1 || fastCount == 0 {
			reject("qualified torch requires exactly one torch output and at least one fast permit input")
		}
		parts = append(parts, "torch-qualified")
	}
	result.CellIO = strings.Join(parts, ";")
	result.Argv = append(result.Argv, "--cell-io", result.CellIO)
	// Bind profile bytes semantically as well as the io block; comments remain configuration metadata.
	result.baseConfigurationIdentity = result.configurationIdentity
	result.configurationIdentity = canonical(object{{"base_sha256", result.SHA256}, {"io", block}, {"io_profile", profile}})
	result.SHA256 = identityDigest(result.configurationIdentity)
	result.MachineIdentity += "cell_io=" + result.CellIO + "\n"
	// Canonical ordering matches native reconstruction.
	lines := strings.Split(strings.TrimSuffix(result.MachineIdentity, "\n"), "\n")
	result.MachineIdentity = sortedIOLines(lines)
	result.MachineSHA256 = identityDigest(result.MachineIdentity)
	result.IOTerminal = terminal
}

// compileTerminalV1 keeps the original combined eight-bit CoE terminal header
// byte-identical, so every configuration compiled before schema 2 keeps its pin.
func compileTerminalV1(machine, block, profile object, profileName string) (string, *inventoryAxis) {
	if !numericEqual(profile.get("bits_unverified"), 8) {
		reject("P6-05: cell I/O requires the fixed eight-bit terminal profile")
	}
	rejectPiControlFields(block, profileName)
	position := ioSlavePosition(machine, block)
	fields := []string{position}
	for _, field := range []struct {
		name string
		min  int64
		max  uint64
	}{
		{"vendor_id", 1, 0xffffffff}, {"product_code", 1, 0xffffffff}, {"revision_no", 0, 0xffffffff},
		{"rx_pdo", 1, 65535}, {"tx_pdo", 1, 65535}, {"rx_sync", 0, 15}, {"tx_sync", 0, 15},
		{"output_index", 1, 65535}, {"output_subindex", 0, 255}, {"input_index", 1, 65535}, {"input_subindex", 0, 255},
	} {
		fields = append(fields, integer(profile.get(field.name+"_unverified"), field.min, field.max, field.name+"_unverified"))
	}
	if fields[6] == fields[7] {
		reject("cell I/O sync managers must differ")
	}
	return strings.Join(fields, "|"), ioInventoryTerminal(profileName, position, fields[1], fields[2], fields[3],
		"P6-05: identity and PDO layout unverified; no torch qualification")
}

// compileIOProfileV2 binds a mailbox-free EtherCAT terminal (identity from SII,
// fixed PDOs, one byte each) or a piControl process image (RevPi DIO).
func compileIOProfileV2(machine, block, profile object, profileName string) (string, int, int, *inventoryAxis) {
	knownProfileFields(profile, "schema_version", "id", "io_backend", "vendor_id", "product_code", "revision_no", "mailbox", "output_channels", "input_channels", "output_pdos", "input_pdos")
	outputChannels := int(mustParseUint(integer(profile.get("output_channels"), 0, cellIOMaxChannels, "output_channels")))
	inputChannels := int(mustParseUint(integer(profile.get("input_channels"), 0, cellIOMaxChannels, "input_channels")))
	switch profile.get("io_backend") {
	case "ethercat":
		if profile.get("mailbox") != false {
			reject("cell I/O profile %s: schema 2 EtherCAT terminals are mailbox-free (SII identity); CoE terminals use schema 1", profileName)
		}
		rejectPiControlFields(block, profileName)
		position := ioSlavePosition(machine, block)
		vendor := integer(profile.get("vendor_id"), 1, 0xffffffff, "vendor_id")
		product := integer(profile.get("product_code"), 1, 0xffffffff, "product_code")
		revision := integer(profile.get("revision_no"), 0, 0xffffffff, "revision_no")
		syncs := map[string]bool{}
		pdos := func(key string, channels int) string {
			entries := asArray(profile.get(key))
			if len(entries) > cellIOMaxChannels/8 || channels > 8*len(entries) {
				reject("cell I/O profile %s: %s must carry %d channels in at most two one-byte PDOs", profileName, key, channels)
			}
			var out []string
			for _, value := range entries {
				entry := asObject(value)
				knownProfileFields(entry, "sync", "pdo", "index", "subindex", "bits")
				sync := integer(entry.get("sync"), 0, 15, "pdo sync")
				if syncs[sync] {
					reject("cell I/O profile %s: each PDO needs its own sync manager", profileName)
				}
				syncs[sync] = true
				if !numericEqual(entry.get("bits"), 8) {
					reject("cell I/O profile %s: each PDO is one 8-bit entry", profileName)
				}
				out = append(out, strings.Join([]string{sync, integer(entry.get("pdo"), 1, 65535, "pdo index"),
					integer(entry.get("index"), 1, 65535, "pdo entry index"), integer(entry.get("subindex"), 0, 255, "pdo entry subindex")}, ":"))
			}
			return strings.Join(out, "+")
		}
		outputs, inputs := pdos("output_pdos", outputChannels), pdos("input_pdos", inputChannels)
		header := strings.Join([]string{"v2", "ethercat", position, vendor, product, revision,
			strconv.Itoa(outputChannels), strconv.Itoa(inputChannels), outputs, inputs}, "|")
		return header, outputChannels, inputChannels, ioInventoryTerminal(profileName, position, vendor, product, revision,
			"identity and fixed PDO layout unverified on the bus; no torch qualification")
	case "picontrol":
		for _, key := range []string{"vendor_id", "product_code", "revision_no", "mailbox", "output_pdos", "input_pdos"} {
			if profile.optional(key, nil) != nil {
				reject("cell I/O profile %s: piControl profiles have no %s", profileName, key)
			}
		}
		if block.optional("slave_position", nil) != nil {
			reject("cell I/O profile %s is a piControl process image: slave_position does not apply", profileName)
		}
		device := asString(block.get("picontrol_device"))
		if !piControlDevicePath.MatchString(device) || strings.Contains(device, "..") {
			reject("picontrol_device must be an absolute path")
		}
		offset := func(key string, channels int) (string, uint64) {
			text := integer(block.get(key), 0, piControlImageBytes-uint64((channels+7)/8), key)
			return text, mustParseUint(text)
		}
		outText, out := offset("picontrol_output_offset", outputChannels)
		inText, in := offset("picontrol_input_offset", inputChannels)
		outEnd, inEnd := out+uint64((outputChannels+7)/8), in+uint64((inputChannels+7)/8)
		if inputChannels > 0 && outputChannels > 0 && out < inEnd && in < outEnd {
			reject("picontrol input and output words overlap")
		}
		header := strings.Join([]string{"v2", "picontrol", device, outText, inText,
			strconv.Itoa(outputChannels), strconv.Itoa(inputChannels)}, "|")
		return header, outputChannels, inputChannels, nil
	}
	reject("cell I/O profile %s: io_backend must be ethercat or picontrol", profileName)
	return "", 0, 0, nil
}

func ioSlavePosition(machine, block object) string {
	position := integer(block.get("slave_position"), 0, 65535, "io slave_position")
	if bus, ok := machine.optional("bus", nil).(object); ok {
		for _, value := range asArray(bus.optional("unused_slaves", []any{})) {
			if integer(asObject(value).get("slave_position"), 0, 65535, "unused position") == position {
				reject("cell I/O slave position overlaps bus.unused_slaves")
			}
		}
	}
	for _, a := range asArray(machine.get("axes")) {
		if integer(asObject(a).get("slave_position"), 0, 65535, "axis position") == position {
			reject("duplicate cell I/O slave position")
		}
	}
	return position
}

func rejectPiControlFields(block object, profileName string) {
	for _, key := range []string{"picontrol_device", "picontrol_output_offset", "picontrol_input_offset"} {
		if block.optional(key, nil) != nil {
			reject("cell I/O profile %s is an EtherCAT terminal: %s does not apply", profileName, key)
		}
	}
}

func ioInventoryTerminal(profileName, position, vendor, product, revision, note string) *inventoryAxis {
	ptr := func(s string) *uint32 { v := uint32(mustParseUint(s)); return &v }
	terminal := &inventoryAxis{Name: "cell_io", Profile: profileName, SlavePosition: *ptr(position), Notes: []string{note}}
	terminal.Expected = inventoryIdentity{ptr(position), ptr(vendor), ptr(product), ptr(revision)}
	return terminal
}

func mustParseUint(text string) uint64 {
	n, err := strconv.ParseUint(text, 10, 64)
	if err != nil {
		panic(fmt.Sprintf("validated integer %q", text))
	}
	return n
}

func knownIOFields(o object, names ...string) {
	allowed := map[string]bool{}
	for _, n := range names {
		allowed[n] = true
	}
	for _, f := range o {
		if !allowed[f.key] {
			reject("unknown cell I/O field %s", f.key)
		}
	}
}

// knownProfileFields closes a terminal profile object. Profiles share the drive
// template with drive keys, so template coverage reads their get() calls.
func knownProfileFields(o object, names ...string) {
	allowed := map[string]bool{"_source": true, "_unverified": true}
	for _, n := range names {
		allowed[n] = true
	}
	for _, f := range o {
		if !allowed[f.key] {
			reject("unknown cell I/O profile field %s", f.key)
		}
	}
}

func sortedIOLines(lines []string) string {
	fields := map[string]string{}
	for _, line := range lines {
		k, v, ok := strings.Cut(line, "=")
		if !ok {
			panic(fmt.Sprintf("invalid identity line %q", line))
		}
		fields[k] = v
	}
	return identityText(fields)
}
