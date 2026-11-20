package rtctl

import (
	"encoding/json"
	"strings"
)

// validateControlTiming resolves cell ceilings before compilation; unknown keys,
// fractional values and values above the absolute bounds cannot reach native.
// Omitted or explicit LAN defaults preserve existing argv and machine identity.
func validateControlTiming(machine object) []string {
	block, ok := machine.optional("control", object{}).(object)
	if !ok {
		reject("control must be an object")
	}
	for _, f := range block {
		if f.key != "link_profile" && f.key != "max_grant_lease_ns" && f.key != "max_jog_input_age_ns" {
			reject("unknown control field %q", f.key)
		}
	}
	profile := asString(block.optional("link_profile", "lan"))
	// Unverified internet starting ceilings: 3 s lease and 750 ms input age.
	lease, age := "500000000", "250000000"
	switch profile {
	case "lan":
	case "internet":
		lease, age = "3000000000", "750000000"
	default:
		reject("control.link_profile must be lan or internet")
	}
	// Unverified absolute ceilings: 10 s lease and 2 s input age. Whole
	// milliseconds keep the effective public lease_ms exact, never rounded up.
	lease = integer(block.optional("max_grant_lease_ns", json.Number(lease)), 1000000, 10000000000, "control.max_grant_lease_ns")
	age = integer(block.optional("max_jog_input_age_ns", json.Number(age)), 1, 2000000000, "control.max_jog_input_age_ns")
	if !strings.HasSuffix(lease, "000000") {
		reject("control.max_grant_lease_ns must be whole milliseconds")
	}
	var args []string
	if lease != "500000000" {
		args = append(args, "--control-max-grant-lease-ns", lease)
	}
	if age != "250000000" {
		args = append(args, "--control-max-jog-input-age-ns", age)
	}
	return args
}
