package rtctl

import (
	"bytes"
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"
	"text/tabwriter"
	"time"
)

// Unverified hardware budget: retain the retired inventory tool's 15 s timeout for each read.
const inventoryReadTimeout = 15 * time.Second

type inventoryRead struct {
	Command  []string `json:"command"`
	ExitCode int      `json:"exit_code"`
	Stdout   string   `json:"stdout"`
	Stderr   string   `json:"stderr"`
	Error    string   `json:"error,omitempty"`
	TimedOut bool     `json:"timed_out,omitempty"`
}

type inventoryIdentity struct {
	SlavePosition *uint32 `json:"slave_position"`
	VendorID      *uint32 `json:"vendor_id"`
	ProductCode   *uint32 `json:"product_code"`
	RevisionNo    *uint32 `json:"revision_no"`
}

type inventoryDifference struct {
	Field    string  `json:"field"`
	Expected *uint32 `json:"expected"`
	Observed *uint32 `json:"observed"`
}

type inventoryAxis struct {
	Name          string            `json:"name"`
	Profile       string            `json:"profile"`
	SlavePosition uint32            `json:"slave_position"`
	Reads         []inventoryRead   `json:"reads"`
	Status        string            `json:"status"`
	Expected      inventoryIdentity `json:"expected"`
	// Observed is the SII identity from `slaves --verbose`: what the master binds on.
	Observed inventoryIdentity `json:"observed"`
	// CoEIdentity is the drive's 0x1018 object; its revision may differ from the SII
	// revision (bench A6-EC drives: SII 0x2ef8, CoE 0x5611) and is informational.
	CoEIdentity inventoryIdentity      `json:"coe_identity"`
	Differences []inventoryDifference  `json:"differences"`
	Notes       []string               `json:"notes,omitempty"`
	Errors      []string               `json:"errors"`
	DriveChecks []inventoryDriveCheck  `json:"drive_checks,omitempty"`
	Observe     []inventoryObservation `json:"observe,omitempty"`
}

type inventoryReport struct {
	Schema              string              `json:"schema"`
	ConfigurationSHA256 string              `json:"configuration_sha256"`
	Inventory           inventoryRead       `json:"inventory"`
	Axes                []inventoryAxis     `json:"axes"`
	HeldSlaves          []inventoryBusSlave `json:"held_slaves,omitempty"`
	IO                  []inventoryAxis     `json:"io,omitempty"`
}

// RunInventoryCLI compiles before reading live hardware and reports every read.
// Only slaves/upload are issued; failed or ambiguous observations cannot match.
func RunInventoryCLI(args []string, root string, stdout, stderr io.Writer) int {
	flags := flag.NewFlagSet("inventory", flag.ContinueOnError)
	flags.SetOutput(stderr)
	config := flags.String("config", "", "live machine JSON file")
	profiles := flags.String("profiles", filepath.Join(root, "config/drives"), "drive config directory")
	ethercat := flags.String("ethercat", "ethercat", "read-only EtherCAT executable")
	jsonOutput := flags.Bool("json", false, "print the inventory JSON document")
	if err := flags.Parse(args); err != nil {
		return 2
	}
	if flags.NArg() != 0 || *config == "" || *ethercat == "" {
		fmt.Fprintln(stderr, "--config is required; --ethercat must be nonempty; positional arguments are not accepted")
		return 2
	}
	data, err := os.ReadFile(*config)
	if err != nil {
		fmt.Fprintln(stderr, err)
		return 1
	}
	compiled, err := Compile(data, *profiles)
	if err != nil {
		fmt.Fprintln(stderr, err)
		return 1
	}
	machine := asObject(decode(data))
	if machine.optional("backend", nil) != "live" {
		fmt.Fprintln(stderr, "inventory requires a live machine configuration")
		return 2
	}
	axes, err := compiledInventoryAxes(machine, compiled)
	if err != nil {
		fmt.Fprintln(stderr, err)
		return 1
	}
	report, status := collectInventory(context.Background(), *ethercat, compiled.SHA256, axes)
	if compiled.IOTerminal != nil {
		terminal := *compiled.IOTerminal
		terminal.Status = "unverified"
		positions, topologyErr := inventoryPositions(report.Inventory.Stdout)
		terminal.Observed = inventorySIIIdentities(report.Inventory.Stdout)[terminal.SlavePosition]
		if positions[terminal.SlavePosition] {
			p := terminal.SlavePosition
			terminal.Observed.SlavePosition = &p
		}
		if topologyErr != nil || report.Inventory.ExitCode != 0 || report.Inventory.Error != "" || terminal.Observed.VendorID == nil || terminal.Observed.ProductCode == nil || terminal.Observed.RevisionNo == nil {
			terminal.Status = "unreadable"
			terminal.Errors = []string{"cell I/O SII identity unavailable"}
			status = 3
		}
		report.IO = []inventoryAxis{terminal}
	}
	if busStatus := inventoryBus(&report, machine); busStatus > status {
		status = busStatus
	}
	if *jsonOutput {
		encoder := json.NewEncoder(stdout)
		encoder.SetIndent("", "  ")
		err = encoder.Encode(report)
	} else {
		err = printInventory(stdout, report)
		for _, terminal := range report.IO {
			observed, _ := json.Marshal(terminal.Observed)
			if _, printErr := fmt.Fprintf(stdout, "cell I/O %s position=%d status=%s observed=%s (P6-05 unverified)\n", terminal.Profile, terminal.SlavePosition, terminal.Status, observed); err == nil {
				err = printErr
			}
		}
	}
	if err != nil {
		fmt.Fprintln(stderr, err)
		return 1
	}
	return status
}

// compiledInventoryAxes uses native compiler outputs for identity and position,
// preventing a second profile load from disagreeing with the configuration SHA.
func compiledInventoryAxes(machine object, compiled Compiled) (axes []inventoryAxis, err error) {
	defer validationBoundary(&err)
	var positions []string
	for i := 0; i+1 < len(compiled.Argv); i += 2 {
		if compiled.Argv[i] == "--slave-positions" {
			positions = strings.Split(compiled.Argv[i+1], ",")
		}
	}
	metadata := asArray(machine.get("axes"))
	if len(positions) != len(metadata) {
		reject("compiled slave positions do not match axes")
	}
	var fields []map[string]string
	for _, line := range strings.Split(strings.TrimSpace(compiled.AxisProfiles), "\n") {
		key, value, ok := strings.Cut(line, "=")
		if !ok {
			reject("invalid compiled axis field")
		}
		if key == "axis" {
			if value != strconv.Itoa(len(fields)) {
				reject("compiled axes are out of order")
			}
			fields = append(fields, map[string]string{})
		}
		if len(fields) == 0 {
			reject("missing compiled axis index")
		}
		fields[len(fields)-1][key] = value
	}
	if len(fields) != len(metadata) {
		reject("compiled profiles do not match axes")
	}
	value := func(text string, bits int) *uint32 {
		n, e := strconv.ParseUint(text, 10, bits)
		if e != nil {
			reject("invalid compiled inventory value: %q", text)
		}
		v := uint32(n)
		return &v
	}
	for i, f := range fields {
		position := value(positions[i], 16)
		var profile string
		if compiled.Robot != nil {
			profile = compiled.Robot.Axes[i].Profile
		} else {
			profile = asString(asObject(metadata[i]).get("profile"))
		}
		axis := inventoryAxis{
			Name: f["name"], Profile: profile, SlavePosition: *position,
			Expected: inventoryIdentity{position, value(f["vendor_id"], 32), value(f["product_code"], 32), value(f["revision_no"], 32)},
			Reads:    []inventoryRead{}, Differences: []inventoryDifference{}, Errors: []string{},
		}
		if compiled.Robot != nil {
			axis.DriveChecks = compiledDriveChecks(f, compiled.Robot.Axes[i])
		}
		axis.Observe = compiledObservations(f["startup_observe"])
		axes = append(axes, axis)
	}
	return axes, nil
}

// runInventoryRead preserves subprocess output separately from launch/timeout
// errors, so missing tools and failed reads cannot masquerade as empty successes.
func runInventoryRead(parent context.Context, executable string, args ...string) inventoryRead {
	ctx, cancel := context.WithTimeout(parent, inventoryReadTimeout)
	defer cancel()
	read := inventoryRead{Command: append([]string{executable}, args...), ExitCode: -1}
	cmd := exec.CommandContext(ctx, executable, args...)
	// Unverified cleanup bound: close inherited output pipes after 100 ms on exit/cancellation.
	cmd.WaitDelay = 100 * time.Millisecond
	var stdout, stderr bytes.Buffer
	cmd.Stdout, cmd.Stderr = &stdout, &stderr
	err := cmd.Run()
	read.Stdout, read.Stderr = stdout.String(), stderr.String()
	if cmd.ProcessState != nil {
		read.ExitCode = cmd.ProcessState.ExitCode()
	}
	if err != nil {
		read.Error = err.Error()
	}
	if ctx.Err() == context.DeadlineExceeded {
		read.TimedOut = true
		read.Error = "read timed out after 15 s"
	}
	return read
}

func collectInventory(ctx context.Context, executable, sha string, axes []inventoryAxis) (inventoryReport, int) {
	report := inventoryReport{Schema: "rosie.rt-core.inventory.v1", ConfigurationSHA256: sha, Axes: axes}
	report.Inventory = runInventoryRead(ctx, executable, "slaves", "--verbose")
	positions, topologyErr := inventoryPositions(report.Inventory.Stdout)
	sii := inventorySIIIdentities(report.Inventory.Stdout)
	if report.Inventory.ExitCode != 0 || report.Inventory.Error != "" {
		topologyErr = fmt.Errorf("slaves --verbose failed")
	}
	status := 0
	for i := range report.Axes {
		axis := &report.Axes[i]
		axis.Status = "match"
		if topologyErr != nil {
			axis.Errors = append(axis.Errors, topologyErr.Error())
		} else if positions[axis.SlavePosition] {
			position := axis.SlavePosition
			axis.Observed.SlavePosition = &position
			if identity, ok := sii[axis.SlavePosition]; ok {
				axis.Observed.VendorID, axis.Observed.ProductCode, axis.Observed.RevisionNo = identity.VendorID, identity.ProductCode, identity.RevisionNo
			} else {
				axis.Errors = append(axis.Errors, "SII identity absent from slaves --verbose")
			}
		}
		for j, object := range []struct{ index, subindex, kind string }{
			{"0x1018", "1", "uint32"}, {"0x1018", "2", "uint32"}, {"0x1018", "3", "uint32"},
			{"0x6041", "0", "uint16"}, {"0x6061", "0", "int8"}, {"0x6064", "0", "int32"},
		} {
			read := runInventoryRead(ctx, executable, "upload", "--position", strconv.FormatUint(uint64(axis.SlavePosition), 10), "--type", object.kind, object.index, object.subindex)
			axis.Reads = append(axis.Reads, read)
			value, err := inventoryNumber(read.Stdout, object.kind)
			if read.ExitCode != 0 || read.Error != "" {
				axis.Errors = append(axis.Errors, object.index+":"+object.subindex+" read failed")
			} else if err != nil {
				axis.Errors = append(axis.Errors, object.index+":"+object.subindex+": "+err.Error())
			} else if j < 3 {
				v := uint32(value)
				switch j {
				case 0:
					axis.CoEIdentity.VendorID = &v
				case 1:
					axis.CoEIdentity.ProductCode = &v
				case 2:
					axis.CoEIdentity.RevisionNo = &v
				}
			}
		}
		axis.CoEIdentity.SlavePosition = axis.Observed.SlavePosition
		collectDriveChecks(ctx, executable, axis)
		collectObservations(ctx, executable, axis)
		if axis.CoEIdentity.RevisionNo != nil && axis.Observed.RevisionNo != nil && *axis.CoEIdentity.RevisionNo != *axis.Observed.RevisionNo {
			axis.Notes = append(axis.Notes, fmt.Sprintf("CoE 0x1018:3 revision %d differs from the SII revision %d the master binds on; the profile pins the SII value", *axis.CoEIdentity.RevisionNo, *axis.Observed.RevisionNo))
		}
		fields := []struct {
			name               string
			expected, observed *uint32
		}{
			{"slave_position", axis.Expected.SlavePosition, axis.Observed.SlavePosition},
			{"vendor_id", axis.Expected.VendorID, axis.Observed.VendorID},
			{"product_code", axis.Expected.ProductCode, axis.Observed.ProductCode},
			{"revision_no", axis.Expected.RevisionNo, axis.Observed.RevisionNo},
		}
		if axis.Observed.SlavePosition == nil {
			// No slave at the configured position: its identity cannot be observed, and
			// reporting three unobservable fields would only restate the missing slave.
			fields = fields[:1]
		}
		for _, field := range fields {
			if field.observed == nil || *field.expected != *field.observed {
				axis.Differences = append(axis.Differences, inventoryDifference{field.name, field.expected, field.observed})
			}
		}
		if len(axis.Differences) != 0 {
			axis.Status = "mismatch"
			if status == 0 {
				status = 2
			}
		}
		for _, check := range axis.DriveChecks {
			if check.Reason == "drive_parameter_mismatch" {
				axis.Status = "mismatch"
				if status == 0 {
					status = 2
				}
			}
		}
		if len(axis.Errors) != 0 {
			axis.Status, status = "unreadable", 3
		}
	}
	return report, status
}

// inventoryPositions reads absolute ring positions from IgH verbose headers.
// Unknown formats, duplicate positions and multiple masters fail closed because
// the compiled configuration and upload commands do not select another master.
// inventorySIIIdentities parses the Identity block of every `slaves --verbose`
// slave section. Missing or unparsable fields stay nil so the caller reports them
// as differences instead of inventing zeros.
func inventorySIIIdentities(output string) map[uint32]inventoryIdentity {
	identities := map[uint32]inventoryIdentity{}
	header := regexp.MustCompile(`^=== Master 0, Slave ([0-9]+) ===$`)
	field := regexp.MustCompile(`^(Vendor Id|Product code|Revision number):\s+0x([0-9a-fA-F]{1,8})$`)
	var current *inventoryIdentity
	var position uint32
	for _, line := range strings.Split(output, "\n") {
		line = strings.TrimSpace(line)
		if match := header.FindStringSubmatch(line); match != nil {
			if current != nil {
				identities[position] = *current
			}
			n, err := strconv.ParseUint(match[1], 10, 16)
			if err != nil {
				current = nil
				continue
			}
			position = uint32(n)
			current = &inventoryIdentity{}
			continue
		}
		if current == nil {
			continue
		}
		if match := field.FindStringSubmatch(line); match != nil {
			value, err := strconv.ParseUint(match[2], 16, 32)
			if err != nil {
				continue
			}
			v := uint32(value)
			switch match[1] {
			case "Vendor Id":
				current.VendorID = &v
			case "Product code":
				current.ProductCode = &v
			case "Revision number":
				current.RevisionNo = &v
			}
		}
	}
	if current != nil {
		identities[position] = *current
	}
	return identities
}

func inventoryPositions(output string) (map[uint32]bool, error) {
	positions := map[uint32]bool{}
	header := regexp.MustCompile(`^=== Master ([0-9]+), Slave ([0-9]+) ===$`)
	for _, line := range strings.Split(output, "\n") {
		line = strings.TrimSpace(line)
		if !strings.HasPrefix(line, "===") {
			continue
		}
		match := header.FindStringSubmatch(line)
		if match == nil || match[1] != "0" {
			return nil, fmt.Errorf("unreadable or ambiguous slave topology")
		}
		n, err := strconv.ParseUint(match[2], 10, 16)
		if err != nil || positions[uint32(n)] {
			return nil, fmt.Errorf("invalid or duplicate slave position")
		}
		positions[uint32(n)] = true
	}
	if len(positions) == 0 && strings.TrimSpace(output) != "" {
		return nil, fmt.Errorf("unrecognized slaves --verbose output")
	}
	return positions, nil
}

// inventoryNumber accepts decimal, hexadecimal or IgH's consistent hex/decimal
// pair. Width checks and whole-token parsing prevent truncated identities from matching.
func inventoryNumber(output, kind string) (int64, error) {
	bits, signed := 32, false
	switch kind {
	case "uint32":
	case "uint16":
		bits = 16
	case "uint8":
		bits = 8
	case "int16":
		bits, signed = 16, true
	case "int8":
		bits, signed = 8, true
	case "int32":
		signed = true
	default:
		return 0, fmt.Errorf("unsupported upload type")
	}
	parse := func(token string) (int64, error) {
		if strings.HasPrefix(token, "0x") || strings.HasPrefix(token, "0X") {
			hexBits := bits
			if signed {
				// IgH promotes int8 to int before printing signed hexadecimal.
				hexBits = 32
			}
			n, err := strconv.ParseUint(token[2:], 16, hexBits)
			if err != nil {
				return 0, err
			}
			v := int64(n)
			if signed {
				if bits == 16 && n <= 65535 {
					v = int64(int16(n))
				} else if bits == 8 && n <= 255 {
					v = int64(int8(n))
				} else {
					v = int64(int32(n))
				}
				if bits == 8 && (v < -128 || v > 127) {
					return 0, fmt.Errorf("signed upload out of range")
				}
				if bits == 16 && (v < -32768 || v > 32767) {
					return 0, fmt.Errorf("signed 16-bit value out of range")
				}
			}
			return v, nil
		}
		if signed {
			return strconv.ParseInt(token, 10, bits)
		}
		n, err := strconv.ParseUint(token, 10, bits)
		return int64(n), err
	}
	output = strings.TrimSpace(output)
	if strings.ContainsAny(output, "\r\n") {
		return 0, fmt.Errorf("multiple %s upload lines", kind)
	}
	fields := strings.Fields(output)
	if len(fields) < 1 || len(fields) > 2 {
		return 0, fmt.Errorf("invalid %s upload value", kind)
	}
	value, err := parse(fields[0])
	if err != nil {
		return 0, fmt.Errorf("invalid %s upload value", kind)
	}
	if len(fields) == 2 {
		other, err := parse(fields[1])
		if err != nil || other != value || !strings.HasPrefix(strings.ToLower(fields[0]), "0x") || strings.ContainsAny(fields[1], "xX") {
			return 0, fmt.Errorf("inconsistent %s upload values", kind)
		}
	}
	return value, nil
}

func printInventory(output io.Writer, report inventoryReport) error {
	writer := tabwriter.NewWriter(output, 0, 4, 2, ' ', 0)
	if _, err := fmt.Fprintf(writer, "Configuration SHA256: %s\nAXIS\tPOSITION\tPROFILE\tSTATUS\tDETAILS\n", report.ConfigurationSHA256); err != nil {
		return err
	}
	format := func(value *uint32) string {
		if value == nil {
			return "unavailable"
		}
		return strconv.FormatUint(uint64(*value), 10)
	}
	for _, axis := range report.Axes {
		var details []string
		for _, difference := range axis.Differences {
			details = append(details, fmt.Sprintf("%s expected=%s observed=%s", difference.Field, format(difference.Expected), format(difference.Observed)))
		}
		details = append(details, axis.Errors...)
		for _, check := range axis.DriveChecks {
			if check.Reason != "" {
				details = append(details, fmt.Sprintf("%s: %s expected=%d observed=%v", check.Reason, check.Field, check.Expected, observedDriveValue(check.Observed)))
			}
		}
		for _, observation := range axis.Observe {
			if observation.Reason != "" {
				details = append(details, fmt.Sprintf("%s: %s %s:%s", observation.Reason, observation.Key, observation.Index, observation.Subindex))
			}
		}
		if _, err := fmt.Fprintf(writer, "%s\t%d\t%s\t%s\t%s\n", axis.Name, axis.SlavePosition, axis.Profile, axis.Status, strings.Join(details, "; ")); err != nil {
			return err
		}
	}
	for _, slave := range report.HeldSlaves {
		if _, err := fmt.Fprintf(writer, "-\t%s\t-\t%s\tvendor=%s product=%s revision=%s (no outputs)\n", format(slave.SlavePosition), slave.Disposition, format(slave.VendorID), format(slave.ProductCode), format(slave.RevisionNo)); err != nil {
			return err
		}
	}
	if err := writer.Flush(); err != nil {
		return err
	}
	return printObservations(output, report)
}
