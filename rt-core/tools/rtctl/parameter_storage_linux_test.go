//go:build linux

package rtctl

import "testing"

// The native parser admits the compiled parameter_storage and startup_trial
// lines and recomputes the same machine identity the compiler published.
func TestParameterStorageNativeIdentity(t *testing.T) {
	f := a6Axes(t, 1)
	f.put("machine.axes.0.startup.a6ec_load_inertia_ratio", "800")
	f.put("machine.verify_parameter_storage", "true")
	f.put("machine.axes.0.startup_trial", `["a6ec_load_inertia_ratio"]`)
	c, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	validatePublishedBringup(t, c)
}
