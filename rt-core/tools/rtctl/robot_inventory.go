package rtctl

import (
	"context"
	"fmt"
	"strconv"
	"strings"
)

// inventoryDriveCheck reports exact compiled object expectations and typed
// refusals; unavailable objects cannot be reported as a matching drive.
type inventoryDriveCheck struct {
	Field    string `json:"field"`
	Index    string `json:"index"`
	Subindex string `json:"subindex"`
	Type     string `json:"type"`
	Expected int64  `json:"expected"`
	Observed *int64 `json:"observed"`
	Reason   string `json:"reason,omitempty"`
	Note     string `json:"note,omitempty"`
}

// compiledDriveChecks reuses emitted startup descriptors, so direction, encoder
// tracking and gearing expectations are the values bound into the daemon input.
func compiledDriveChecks(fields map[string]string, axis RobotAxisBinding) []inventoryDriveCheck {
	checks := []inventoryDriveCheck{}
	for _, descriptor := range strings.Split(fields["startup_sdos"], ";") {
		if descriptor == "" {
			continue
		}
		parts := strings.Split(descriptor, "|")
		if len(parts) != 5 && (len(parts) != 6 || parts[5] != "power_cycle") {
			reject("invalid compiled startup descriptor")
		}
		kind := uploadType(parts[1])
		index, err := strconv.ParseUint(parts[2], 10, 16)
		if err != nil || kind == "" {
			reject("invalid compiled startup object")
		}
		expected, err := strconv.ParseInt(parts[4], 10, 64)
		if err != nil {
			reject("invalid compiled startup value")
		}
		checks = append(checks, inventoryDriveCheck{Field: parts[0], Index: fmt.Sprintf("0x%04x", index), Subindex: parts[3], Type: kind, Expected: expected})
	}
	encoder, err := strconv.ParseInt(axis.MotorEncoderCountsPerRev, 10, 32)
	if err != nil {
		reject("invalid compiled encoder resolution")
	}
	// Unverified A6 availability: CiA402 0x608F:1/:2 encoder increments/motor
	// revolutions must be readable to certify resolution; no inferred Home offset.
	// Object semantics: https://infosys.beckhoff.com/content/1033/ep7211/1877700363.html
	note := "unverified drive support for CiA402 encoder-resolution object; unavailable readback refuses inventory qualification"
	checks = append(checks,
		inventoryDriveCheck{Field: "motor_encoder_counts_per_rev", Index: "0x608f", Subindex: "1", Type: "uint32", Expected: encoder, Note: note},
		inventoryDriveCheck{Field: "encoder_motor_revolutions", Index: "0x608f", Subindex: "2", Type: "uint32", Expected: 1, Note: note})
	return checks
}

// collectDriveChecks issues uploads only, retaining every failed read and exact
// value disagreement as a per-axis refusal before any bring-up qualification.
func collectDriveChecks(ctx context.Context, executable string, axis *inventoryAxis) {
	for i := range axis.DriveChecks {
		check := &axis.DriveChecks[i]
		read := runInventoryRead(ctx, executable, "upload", "--position", strconv.FormatUint(uint64(axis.SlavePosition), 10), "--type", check.Type, check.Index, check.Subindex)
		axis.Reads = append(axis.Reads, read)
		value, err := inventoryNumber(read.Stdout, check.Type)
		if read.ExitCode != 0 || read.Error != "" || err != nil {
			check.Reason = "drive_parameter_unreadable"
			axis.Errors = append(axis.Errors, fmt.Sprintf("%s: %s %s:%s", check.Reason, check.Field, check.Index, check.Subindex))
			continue
		}
		check.Observed = &value
		if value != check.Expected {
			check.Reason = "drive_parameter_mismatch"
		}
	}
	// The two uint32 objects form a ratio, not necessarily a one-revolution
	// encoding. Cross multiplication is exact within uint64 for these bounds.
	if n := len(axis.DriveChecks); n >= 2 {
		increments, revolutions := &axis.DriveChecks[n-2], &axis.DriveChecks[n-1]
		if increments.Field == "motor_encoder_counts_per_rev" && revolutions.Field == "encoder_motor_revolutions" && increments.Observed != nil && revolutions.Observed != nil {
			if *revolutions.Observed > 0 && uint64(*increments.Observed) == uint64(increments.Expected)*uint64(*revolutions.Observed) {
				increments.Reason, revolutions.Reason = "", ""
			} else {
				increments.Reason, revolutions.Reason = "drive_parameter_mismatch", "drive_parameter_mismatch"
			}
		}
	}
}

func observedDriveValue(value *int64) any {
	if value == nil {
		return "unreadable"
	}
	return *value
}
