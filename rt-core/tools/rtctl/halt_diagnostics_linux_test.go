//go:build linux

package rtctl

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// Native parsing must preserve the explicit diagnostic assignment in machine
// identity and refuse malformed compiled mappings even without compiler checks.
func TestExternalEnableInputNativeIdentity(t *testing.T) {
	f := fresh()
	tx := asArray(f.get("profile.ethercat.tx_layout"))
	f.put("profile.ethercat.tx_layout", canonical(append(tx, decode([]byte(`{"semantic":"di","index":24829,"subindex":0,"bits":32}`)))))
	f.put("machine.axes.0.diagnostics", `{"external_enable_input":`+externalEnableJSON+`}`)
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	validatePublishedBringup(t, compiled)
	dir, args, err := writeCompiled(compiled, buildTemp(t))
	if err != nil {
		t.Fatal(err)
	}
	for _, value := range []string{"32|active_high", "-1|active_high", "16|unknown", "16", "16|active_high|extra", "16|active_low"} {
		t.Run(value, func(t *testing.T) {
			changed := strings.Replace(compiled.AxisProfiles, "external_enable_input=16|active_high", "external_enable_input="+value, 1)
			if err := os.WriteFile(filepath.Join(dir, "axes.conf"), []byte(changed), 0o600); err != nil {
				t.Fatal(err)
			}
			output, err := runBringupDaemon(t, append(append([]string{}, args...), "--validate-config")...)
			if err == nil {
				t.Fatalf("native accepted tampered assignment: %s", output)
			}
			if value == "16|active_low" && !strings.Contains(output, "machine identity mismatch: axis.0.external_enable_input") {
				t.Fatalf("polarity escaped identity: %v %s", err, output)
			}
		})
	}
}
