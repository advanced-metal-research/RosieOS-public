package rtctl

import (
	"fmt"
	"strings"
)

// renderCollisionWatchdog validates a whole axis override or profile block.
// Missing configuration disables with a named warning; malformed or unprovenanced
// configuration is rejected rather than silently losing collision protection.
func renderCollisionWatchdog(axis, profile object, tx []any) (string, string) {
	name := asString(axis.get("name"))
	var block any
	present := false
	for _, owner := range []object{profile, axis} {
		for _, field := range owner {
			if field.key == "collision_watchdog" {
				block, present = field.value, true
			}
		}
	}
	if !present {
		return "", fmt.Sprintf("axis %s: collision_watchdog missing; disabled", name)
	}
	c := asObject(block)
	for _, f := range c {
		switch f.key {
		case "torque_abs_max_raw", "following_error_counts_max", "sustained_cycles", "disabled", "reason", "_unverified", "_source":
		default:
			reject("axis %s: unknown collision_watchdog field %s", name, f.key)
		}
	}
	for _, key := range []string{"_unverified", "_source"} {
		if strings.TrimSpace(asString(c.get(key))) == "" {
			reject("axis %s: collision_watchdog.%s must document provenance", name, key)
		}
	}
	if asBool(c.optional("disabled", false)) {
		reason := strings.TrimSpace(asString(c.get("reason")))
		if reason == "" {
			reject("axis %s: disabled collision_watchdog requires a reason", name)
		}
		for _, key := range []string{"torque_abs_max_raw", "following_error_counts_max", "sustained_cycles"} {
			for _, field := range c {
				if field.key == key {
					reject("axis %s: disabled collision_watchdog must omit thresholds", name)
				}
			}
		}
		return "", fmt.Sprintf("axis %s: collision_watchdog disabled: %s", name, reason)
	}
	torqueMapped := false
	for _, entry := range tx {
		f := asObject(entry)
		torqueMapped = torqueMapped || asString(f.get("semantic")) == "torque"
	}
	if !torqueMapped {
		reject("axis %s: collision_watchdog requires torque PDO", name)
	}
	return integer(c.get("torque_abs_max_raw"), 1, 1000, "collision_watchdog.torque_abs_max_raw") + "|" +
		integer(c.get("following_error_counts_max"), 1, 0x7fffffff, "collision_watchdog.following_error_counts_max") + "|" +
		integer(c.get("sustained_cycles"), 2, 1000, "collision_watchdog.sustained_cycles"), ""
}
