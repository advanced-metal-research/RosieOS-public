package rtctl

import (
	"context"
	"fmt"
	"io"
	"strconv"
	"strings"
)

// inventoryObservation is one drive tuning object the compiled configuration
// has rt-core read once per startup epoch (the axis's startup_observe list,
// rendered from the machine's observe keys and the drive's tuning catalog).
// Inventory uploads exactly those descriptors, so a read taken before start
// and the startup journal name the same keys and objects from one source.
// Like rt-core it records the raw drive value, compares it with nothing and
// lets no failed read change the identity status: observations are
// measurements, never qualification.
type inventoryObservation struct {
	Key      string `json:"key"`
	Index    string `json:"index"`
	Subindex string `json:"subindex"`
	Type     string `json:"type"`
	Observed *int64 `json:"observed"`
	Reason   string `json:"reason,omitempty"`
}

// uploadType names a compiled SDO scalar as the IgH upload --type argument.
func uploadType(scalar string) string {
	return map[string]string{"u8": "uint8", "i8": "int8", "u16": "uint16", "i16": "int16", "u32": "uint32", "i32": "int32"}[scalar]
}

// compiledObservations parses an axis's startup_observe field
// ("key|scalar|index|subindex;..."); a malformed descriptor refuses the
// inventory rather than reading an object rt-core would not.
func compiledObservations(field string) []inventoryObservation {
	var observations []inventoryObservation
	for _, descriptor := range strings.Split(field, ";") {
		if descriptor == "" {
			continue
		}
		parts := strings.Split(descriptor, "|")
		if len(parts) != 4 {
			reject("invalid compiled observe descriptor")
		}
		kind := uploadType(parts[1])
		index, indexErr := strconv.ParseUint(parts[2], 10, 16)
		sub, subErr := strconv.ParseUint(parts[3], 10, 8)
		if parts[0] == "" || kind == "" || indexErr != nil || subErr != nil {
			reject("invalid compiled observe object")
		}
		observations = append(observations, inventoryObservation{Key: parts[0], Index: fmt.Sprintf("0x%04x", index), Subindex: strconv.FormatUint(sub, 10), Type: kind})
	}
	return observations
}

// collectObservations issues one upload per observed object and keeps every
// read; an unreadable object is reported with its reason and no value.
func collectObservations(ctx context.Context, executable string, axis *inventoryAxis) {
	for i := range axis.Observe {
		observation := &axis.Observe[i]
		read := runInventoryRead(ctx, executable, "upload", "--position", strconv.FormatUint(uint64(axis.SlavePosition), 10), "--type", observation.Type, observation.Index, observation.Subindex)
		axis.Reads = append(axis.Reads, read)
		value, err := inventoryNumber(read.Stdout, observation.Type)
		if read.ExitCode != 0 || read.Error != "" || err != nil {
			observation.Reason = "observe_unreadable"
			continue
		}
		observation.Observed = &value
	}
}

// printObservations writes one line per observing axis after the summary
// table, raw values as rt-core journals them.
func printObservations(output io.Writer, report inventoryReport) error {
	for _, axis := range report.Axes {
		if len(axis.Observe) == 0 {
			continue
		}
		values := make([]string, 0, len(axis.Observe))
		for _, observation := range axis.Observe {
			values = append(values, fmt.Sprintf("%s=%v", observation.Key, observedDriveValue(observation.Observed)))
		}
		if _, err := fmt.Fprintf(output, "observe %s: %s\n", axis.Name, strings.Join(values, " ")); err != nil {
			return err
		}
	}
	return nil
}
