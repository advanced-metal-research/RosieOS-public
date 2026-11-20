package rtctl

import (
	"strings"
	"testing"
)

// Unverified synthetic thresholds exercise compiler boundaries, not drive tuning.
const collisionBlock = `{"torque_abs_max_raw":1000,"following_error_counts_max":8000,"sustained_cycles":10,"_unverified":"unverified test values","_source":"synthetic compiler fixture"}`

func collisionFixture() *fixture {
	f := fresh()
	tx := asArray(f.get("profile.ethercat.tx_layout"))
	tx = append(tx, decode([]byte(`{"semantic":"torque","index":24695,"subindex":0,"bits":16}`)))
	f.put("profile.ethercat.tx_layout", canonical(tx))
	f.put("profile.collision_watchdog", collisionBlock)
	return f
}

func TestCollisionWatchdogCompiler(t *testing.T) {
	f := collisionFixture()
	compiled, err := f.compile()
	if err != nil || !strings.Contains(compiled.AxisProfiles, "collision_watchdog=1000|8000|10\n") {
		t.Fatalf("%v\n%s", err, compiled.AxisProfiles)
	}
	f.put("machine.axes.0.collision_watchdog", strings.Replace(collisionBlock, "8000", "7000", 1))
	compiled, err = f.compile()
	if err != nil || !strings.Contains(compiled.AxisProfiles, "collision_watchdog=1000|7000|10\n") {
		t.Fatalf("%v\n%s", err, compiled.AxisProfiles)
	}
	f.put("machine.axes.0.collision_watchdog", `{"disabled":true,"reason":"isolated fixture","_unverified":"unverified","_source":"test"}`)
	compiled, err = f.compile()
	if err != nil || strings.Contains(compiled.AxisProfiles, "collision_watchdog=") || !strings.Contains(strings.Join(compiled.Warnings, " "), "axis axis_0: collision_watchdog disabled: isolated fixture") {
		t.Fatal(compiled, err)
	}
	f = fresh()
	compiled, err = f.compile()
	if err != nil || !strings.Contains(strings.Join(compiled.Warnings, " "), "axis axis_0: collision_watchdog missing; disabled") {
		t.Fatal(compiled, err)
	}
}

func TestCollisionRobotCellOverride(t *testing.T) {
	f := newRobotFixture(t, true)
	f.setAxis("collision_watchdog", decode([]byte(collisionBlock)))
	compiled, err := f.compile()
	if err != nil || !strings.Contains(compiled.AxisProfiles, "collision_watchdog=1000|8000|10\n") {
		t.Fatal(compiled, err)
	}
	f.setAxis("collision_watchdog", nil)
	if _, err := f.compile(); err == nil {
		t.Fatal("null robot cell override accepted")
	}
}

func TestCollisionWatchdogCompilerRejects(t *testing.T) {
	for _, tc := range []struct{ key, value string }{
		{"torque_abs_max_raw", "0"}, {"torque_abs_max_raw", "1001"}, {"torque_abs_max_raw", "1.5"},
		{"following_error_counts_max", "0"}, {"following_error_counts_max", "2147483648"},
		{"sustained_cycles", "1"}, {"sustained_cycles", "1001"}, {"sustained_cycles", "null"},
		{"_source", `""`}, {"_unverified", `false`}, {"typo", "10"},
	} {
		t.Run(tc.key+tc.value, func(t *testing.T) {
			f := collisionFixture()
			f.put("profile.collision_watchdog."+tc.key, tc.value)
			if _, err := f.compile(); err == nil {
				t.Fatal("accepted invalid block")
			}
		})
	}
	for _, key := range []string{"torque_abs_max_raw", "following_error_counts_max", "sustained_cycles", "_source", "_unverified"} {
		f := collisionFixture()
		f.remove("profile.collision_watchdog." + key)
		if _, err := f.compile(); err == nil {
			t.Fatal("accepted missing", key)
		}
	}
	f := collisionFixture()
	f.put("machine.axes.0.collision_watchdog", "null")
	if _, err := f.compile(); err == nil {
		t.Fatal("null override inherited profile")
	}
	f = fresh()
	f.put("profile.collision_watchdog", collisionBlock)
	if _, err := f.compile(); err == nil {
		t.Fatal("accepted absent torque PDO")
	}
	for _, values := range []string{`{"torque_abs_max_raw":1,"following_error_counts_max":1,"sustained_cycles":2,"_unverified":"unverified","_source":"test"}`, `{"torque_abs_max_raw":1000,"following_error_counts_max":2147483647,"sustained_cycles":1000,"_unverified":"unverified","_source":"test"}`} {
		f := collisionFixture()
		f.put("profile.collision_watchdog", values)
		if _, err := f.compile(); err != nil {
			t.Fatal(err)
		}
	}
}
