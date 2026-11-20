//go:build linux

package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"reflect"
	"strings"
)

// UnmarshalJSON checks the raw measurement schema before absent scalars can be
// mistaken for measured zero. Derived summaries and saved verdicts are ignored.
func (r *record) UnmarshalJSON(raw []byte) error {
	type plain record
	var decoded plain
	if err := json.Unmarshal(raw, &decoded); err != nil {
		return err
	}
	*r = record(decoded)
	var fields map[string]any
	d := json.NewDecoder(bytes.NewReader(raw))
	d.UseNumber()
	if err := d.Decode(&fields); err != nil {
		return err
	}
	missing := func(path string) {
		if r.Unavailable == nil {
			r.Unavailable = map[string]string{}
		}
		r.Unavailable[path] = "required measurement missing or null"
	}
	for _, field := range []string{"schema", "iterations_completed", "host_loaded", "samples", "after_warmup", "final", "iterations", "jog_observations", "status_publications", "event_batches"} {
		t := reflect.TypeOf(*r)
		for i := 0; i < t.NumField(); i++ {
			f := t.Field(i)
			if f.Tag.Get("json") == field {
				requireMeasurements(fields[field], f.Type, field, missing)
			}
		}
	}
	configuration, ok := fields["configuration"].(map[string]any)
	if !ok {
		missing("configuration")
	} else {
		for _, field := range []string{"iterations", "points_per_plan", "page_bytes", "published_jog_input_age_bound_ns", "cycle_lateness_bound_ns"} {
			requireMeasurements(configuration[field], reflect.TypeOf(uint64(0)), "configuration."+field, missing)
		}
	}
	return nil
}

// requireMeasurements follows the measurement types' JSON fields recursively,
// including zero-valued nested counters. Optional payloads and the C++ daemon's
// inapplicable Go heap are excluded; native occupancy requires all six states.
func requireMeasurements(value any, t reflect.Type, path string, missing func(string)) {
	if strings.HasSuffix(path, ".daemon.heap_inuse_bytes") {
		return
	}
	if value == nil {
		missing(path)
		return
	}
	for t.Kind() == reflect.Pointer {
		t = t.Elem()
	}
	switch t.Kind() {
	case reflect.Struct:
		object, _ := value.(map[string]any)
		for i := 0; i < t.NumField(); i++ {
			f := t.Field(i)
			name, modifiers, _ := strings.Cut(f.Tag.Get("json"), ",")
			if name == "-" {
				continue
			}
			if name == "" {
				name = f.Name
			}
			v, exists := object[name]
			if strings.Contains(modifiers, "omitempty") && (!exists || v == nil) {
				continue
			}
			// An explicit null event list is a response with no new events.
			if exists && v == nil && strings.HasSuffix(path+"."+name, ".batch.events") {
				continue
			}
			requireMeasurements(v, f.Type, path+"."+name, missing)
		}
	case reflect.Slice, reflect.Array:
		values, _ := value.([]any)
		for i, v := range values {
			requireMeasurements(v, t.Elem(), fmt.Sprintf("%s[%d]", path, i), missing)
		}
	case reflect.Map:
		if strings.HasSuffix(path, ".native_slot_occupancy") {
			object, _ := value.(map[string]any)
			for _, name := range []string{"free", "writing", "prepared", "ready", "active", "retired"} {
				requireMeasurements(object[name], t.Elem(), path+"."+name, missing)
			}
		}
	}
}
