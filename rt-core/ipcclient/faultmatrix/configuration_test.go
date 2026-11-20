//go:build linux

package faultmatrix

import (
	"context"
	"os"
	"os/exec"
	"path/filepath"
	ipc "rosieos/rt-core/ipcclient"
	"strings"
	"testing"
)

func corruptDigest(t *testing.T, r row) {
	f := compileFixture(t, "simulation.json", nil)
	identity := filepath.Join(f.dir, "machine.identity")
	must(t, os.WriteFile(identity, []byte(f.compiled.MachineIdentity), 0600))
	changed := strings.Replace(f.compiled.AxisProfiles, "limits=-0.5|0.5", "limits=-0.4|0.5", 1)
	if changed == f.compiled.AxisProfiles {
		t.Fatal("missing limit fixture")
	}
	must(t, os.WriteFile(f.profile, []byte(changed), 0600))
	ctx, cancel := context.WithTimeout(context.Background(), watchdog)
	defer cancel()
	args := append(append([]string{}, f.args...), "--machine-sha256", f.compiled.MachineSHA256, "--machine-identity", identity, "--validate-config")
	at := ipc.HostMonotonicNS()
	out, err := exec.CommandContext(ctx, f.binary, args...).CombinedOutput()
	if elapsed := ipc.HostMonotonicNS() - at; elapsed > r.boundNS {
		t.Fatalf("digest refusal exceeded bound_ns=%d: elapsed_ns=%d", r.boundNS, elapsed)
	}
	exit, ok := err.(*exec.ExitError)
	if !ok || exit.ExitCode() != 2 || !strings.Contains(string(out), "machine identity mismatch: axis.0.limits") {
		t.Fatalf("want exit 2 naming axis.0.limits: %v %s", err, out)
	}
}
func corruptMissing(t *testing.T, r row) {
	corruptProfile(t, r, func(profile string) string {
		lines := strings.Split(profile, "\n")
		for i, line := range lines {
			if strings.HasPrefix(line, "tracking=") {
				return strings.Join(append(lines[:i], lines[i+1:]...), "\n")
			}
		}
		t.Fatal("missing tracking fixture")
		return ""
	})
}
func corruptUnknown(t *testing.T, r row) {
	corruptProfile(t, r, func(profile string) string { return profile + "fault_matrix_unknown=1\n" })
}
func corruptProfile(t *testing.T, r row, change func(string) string) {
	f := compileFixture(t, "simulation.json", nil)
	must(t, os.WriteFile(f.profile, []byte(change(f.compiled.AxisProfiles)), 0600))
	ctx, cancel := context.WithTimeout(context.Background(), watchdog)
	defer cancel()
	at := ipc.HostMonotonicNS()
	out, err := exec.CommandContext(ctx, f.binary, append(f.args, "--validate-config")...).CombinedOutput()
	if elapsed := ipc.HostMonotonicNS() - at; elapsed > r.boundNS {
		t.Fatalf("invalid configuration exit exceeded bound_ns=%d: elapsed_ns=%d", r.boundNS, elapsed)
	}
	exit, ok := err.(*exec.ExitError)
	if !ok || exit.ExitCode() != 2 || !strings.Contains(string(out), r.reason) {
		t.Fatalf("want exit 2 and %q, got %v %s", r.reason, err, out)
	}
}
