//go:build linux

package ipcclient_test

import (
	"strings"
	"testing"

	"rosieos/rt-core/adapters/rosie/control"
)

// TestAcquireBindsMachineIdentity proves deployment reconfiguration cannot alter
// the requested machine frame, and an explicit bad machine never falls back.
func TestAcquireBindsMachineIdentity(t *testing.T) {
	f := identityCompile(t, nil)
	core := f.start(t, false)
	binding := control.Binding{PairID: "identity-test", Revision: 1,
		ConfigurationSHA256: f.compiled.SHA256, MachineSHA256: f.compiled.MachineSHA256}
	machineOnly := binding
	machineOnly.ConfigurationSHA256 = ""
	controller, err := control.New(core, machineOnly)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = controller.Close() })
	wrong := binding
	wrong.MachineSHA256 = strings.Repeat("0", 64)
	if _, err := controller.Acquire("test", wrong); err == nil || err.Error() != "authority_binding_mismatch" {
		t.Fatalf("accepted wrong machine: %v", err)
	}
	changedDeployment := binding
	changedDeployment.ConfigurationSHA256 = strings.Repeat("a", 64)
	grant, err := controller.Acquire("test", changedDeployment)
	if err != nil {
		t.Fatal(err)
	}
	if grant.Generation != 1 || grant.Binding.MachineSHA256 != binding.MachineSHA256 || grant.Binding.ConfigurationSHA256 != binding.ConfigurationSHA256 {
		t.Fatalf("missing machine/generation binding: %+v", grant)
	}
	if _, err := controller.Stop(control.Fence{Session: grant.Session, Generation: grant.Generation}, true); err != nil {
		t.Fatal(err)
	}
	legacy := binding
	legacy.MachineSHA256 = ""
	grant, err = controller.Acquire("legacy", legacy)
	if err != nil {
		t.Fatal(err)
	}
	if grant.Binding.MachineSHA256 != binding.MachineSHA256 {
		t.Fatal("legacy grant omitted verified machine")
	}
}
