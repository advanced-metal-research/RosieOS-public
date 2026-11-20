package rtctl

import (
	"fmt"
	"math"
	"strconv"
	"strings"
)

// Native capacity and admission bounds, mirrored from
// include/drive_alarm_reset.hpp (kMaxResettableDriveAlarms,
// kMaxDriveAlarmLeaveFaultMs, kMaxDriveAlarmResetIntervalMs).
const (
	maxResettableDriveAlarms  = 128
	maxDriveAlarmLeaveFaultMS = 500
	maxDriveAlarmIntervalMS   = 3600000
)

// renderDriveAlarmReset compiles the drive profile's fault_reset block: the
// alarms whose drive-side CiA402 fault reset an operator Reset may send, each
// an exact (0x603F, 0x203F) pair, and the attempt timing. It returns "" when
// the profile has no block; native then resets no drive alarm, exactly as
// before. Source classifies nothing; this block is the only authority.
//
// not_reset names the vendor-resettable codes this profile deliberately leaves
// out (for example position-reference alarms) with the reason, so the list is
// auditable against the manual. A code may not appear in both lists.
func renderDriveAlarmReset(profile object) string {
	for _, member := range profile {
		if member.key == "fault_reset" && member.value == nil {
			reject("fault_reset must be an explicit object")
		}
	}
	value := profile.optional("fault_reset", nil)
	if value == nil {
		return ""
	}
	block, ok := value.(object)
	if !ok {
		reject("fault_reset must be an object")
	}
	knownFields(block, "_source", "_unverified", "leave_fault_ms", "min_interval_ms", "resettable_alarms", "not_reset")
	leave := integer(block.get("leave_fault_ms"), 1, maxDriveAlarmLeaveFaultMS, "fault_reset leave_fault_ms")
	interval := integer(block.get("min_interval_ms"), 1, maxDriveAlarmIntervalMS, "fault_reset min_interval_ms")
	alarms := asArray(block.get("resettable_alarms"))
	if len(alarms) == 0 || len(alarms) > maxResettableDriveAlarms {
		reject("fault_reset resettable_alarms must list 1..%d alarms", maxResettableDriveAlarms)
	}
	codes, pairs := map[string]bool{}, map[string]bool{}
	entries := make([]string, 0, len(alarms))
	for _, value := range alarms {
		alarm := asObject(value)
		knownFields(alarm, "code", "error_code", "aux_code")
		code := driveAlarmCode(alarm.get("code"))
		errorCode := driveAlarmWord(alarm.get("error_code"), "fault_reset "+code+" error_code")
		auxCode := driveAlarmWord(alarm.get("aux_code"), "fault_reset "+code+" aux_code")
		if errorCode == 0 {
			reject("fault_reset %s error_code must be nonzero", code)
		}
		pair := fmt.Sprintf("0x%04x:0x%04x", errorCode, auxCode)
		if codes[code] || pairs[pair] {
			reject("fault_reset resettable_alarms must be unique (%s %s)", code, pair)
		}
		codes[code], pairs[pair] = true, true
		entries = append(entries, pair)
	}
	if excluded := block.optional("not_reset", nil); excluded != nil {
		for _, value := range asArray(excluded) {
			entry := asObject(value)
			knownFields(entry, "code", "reason")
			code := driveAlarmCode(entry.get("code"))
			if codes[code] {
				reject("fault_reset %s is both resettable and not_reset", code)
			}
			if strings.TrimSpace(asString(entry.get("reason"))) == "" {
				reject("fault_reset not_reset %s needs a reason", code)
			}
			codes[code] = true
		}
	}
	return leave + "|" + interval + "|" + strings.Join(entries, ";")
}

// driveAlarmCode is the manual's display code (for example Er41.1): a label
// for review and the not_reset cross-check, never compiled.
func driveAlarmCode(value any) string {
	code := asString(value)
	if code == "" || strings.ContainsAny(code, " |;:") {
		reject("fault_reset alarm code must be a nonempty display code, got %q", code)
	}
	return code
}

// driveAlarmWord reads a 16-bit drive code written as 0x-prefixed hex.
func driveAlarmWord(value any, label string) uint64 {
	text := asString(value)
	if !strings.HasPrefix(text, "0x") || len(text) < 3 || len(text) > 6 {
		reject("%s must be 0x-prefixed hex, got %q", label, text)
	}
	word, err := strconv.ParseUint(text[2:], 16, 16)
	if err != nil || word > math.MaxUint16 {
		reject("%s must be a 16-bit code, got %q", label, text)
	}
	return word
}
