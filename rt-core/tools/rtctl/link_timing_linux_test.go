//go:build linux

package rtctl

import (
	"strings"
	"testing"
)

// TestControlTimingNativeIdentity rejects changed native ceilings even when a
// caller retains the original configuration hash and axis profiles.
func TestControlTimingNativeIdentity(t *testing.T) {
	f := fresh()
	f.put("machine.control", `{"link_profile":"internet"}`)
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	_, args, err := writeCompiled(compiled, buildTemp(t))
	if err != nil {
		t.Fatal(err)
	}
	if out, err := runBringupDaemon(t, append(args, "--validate-config")...); err != nil {
		t.Fatalf("native: %v %s", err, out)
	}
	for _, flag := range []string{"--control-max-grant-lease-ns", "--control-max-jog-input-age-ns"} {
		tampered := append([]string{}, args...)
		for i := range tampered {
			if tampered[i] == flag {
				tampered[i+1] = "100000000"
			}
		}
		out, err := runBringupDaemon(t, append(tampered, "--validate-config")...)
		if err == nil || !strings.Contains(out, "machine identity mismatch: control_") {
			t.Fatalf("tampered policy: %v %s", err, out)
		}
	}
}
