package main

import (
	"encoding/json"
	"fmt"
	"math"
)

type sharedConstant struct {
	name  string
	value uint64
}

// This group defines the existing local shared-memory contract, not another
// record codec. Header/payload sizes come from the declared records; every slot
// starts with one aligned 64-bit SharedWords publication sequence.
func parseFastControl(encoded json.RawMessage, records []record) ([]sharedConstant, error) {
	entries, err := object(encoded)
	if err != nil {
		return nil, fmt.Errorf("fast_control must be an object: %w", err)
	}
	keys := []string{"magic", "region_bytes", "grant_offset", "observation_offset", "max_grant_lease_ns"}
	jog, path := false, false
	for _, entry := range entries {
		if entry.name == "jog_control_offset" {
			jog = true
		}
		if entry.name == "path_samples_offset" {
			path = true
		}
	}
	if path && !jog {
		return nil, fmt.Errorf("fast_control path slots require the jog slots")
	}
	if jog {
		keys = append(keys, "jog_control_offset", "jog_update_offset", "jog_observation_offset", "max_jog_input_age_ns", "max_jog_first_input_wait_ns")
	}
	if path {
		keys = append(keys, "path_samples_offset", "path_observation_offset")
	}
	values := make(map[string]uint64, len(keys))
	for _, entry := range entries {
		known := false
		for _, key := range keys {
			known = known || key == entry.name
		}
		if !known {
			return nil, fmt.Errorf("unknown fast_control member %q", entry.name)
		}
		var value uint64
		if err := json.Unmarshal(entry.value, &value); err != nil || value == 0 {
			return nil, fmt.Errorf("fast_control.%s must be a positive unsigned 64-bit integer", entry.name)
		}
		values[entry.name] = value
	}
	for _, key := range keys {
		if _, ok := values[key]; !ok {
			return nil, fmt.Errorf("required fast_control member %q is missing", key)
		}
	}
	for _, key := range keys[1:4] {
		if values[key] > math.MaxUint32 || values[key]%8 != 0 {
			return nil, fmt.Errorf("fast_control.%s must be an aligned 32-bit byte size/offset", key)
		}
	}
	widths := make(map[string]uint64, 3)
	for _, record := range records {
		widths[record.name] = uint64(record.width)
	}
	for _, name := range []string{"fast_header", "grant_request", "grant_observation"} {
		if widths[name] == 0 || widths[name]%8 != 0 {
			return nil, fmt.Errorf("fast_control requires an aligned %s record", name)
		}
	}
	region, grant, observation := values["region_bytes"], values["grant_offset"], values["observation_offset"]
	if widths["fast_header"] > grant || grant > observation || observation > region ||
		widths["grant_request"]+8 > observation-grant || widths["grant_observation"]+8 > region-observation {
		return nil, fmt.Errorf("fast_control records/sequence words overlap or exceed the region")
	}
	if jog {
		previousEnd := observation + widths["grant_observation"] + 8
		slots := []string{"jog_control", "jog_update", "jog_observation"}
		if path {
			// Each path slot is one sequence word plus its record, after the jog
			// slots, inside the same region: the lossless sample handshake and
			// its observation never overlap a latest-value jog slot.
			slots = append(slots, "path_samples", "path_observation")
		}
		for _, name := range slots {
			offset, width := values[name+"_offset"], widths[name]
			if offset > region || offset%8 != 0 || width == 0 || width%8 != 0 ||
				offset < previousEnd || width+8 > region-offset {
				return nil, fmt.Errorf("fast_control %s overlaps or has invalid geometry", name)
			}
			previousEnd = offset + width + 8
		}
		if values["max_jog_input_age_ns"] > values["max_grant_lease_ns"] || values["max_jog_first_input_wait_ns"] > values["max_grant_lease_ns"] {
			return nil, fmt.Errorf("jog timing ceiling exceeds grant lease ceiling")
		}
	}
	constants := make([]sharedConstant, 0, len(keys))
	for _, key := range keys {
		constants = append(constants, sharedConstant{"fast_control_" + key, values[key]})
	}
	return constants, nil
}
