//go:build linux

package rtctl

import (
	"context"
	"encoding/hex"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
	control "rosieos/rt-core/sdk/control"
)

// TestBrakeOverrideDescribe traverses compiler, native identity validation,
// native Describe, public HTTP and the generated Go contract; escaping must not
// lose the reason or label a different axis as overridden.
func TestBrakeOverrideDescribe(t *testing.T) {
	f := busSixAxisFixture(t)
	f.put("profile.brake", `{"present":true}`)
	f.put("machine.bus", `{"unknown_slaves":"hold","unknown_slaves_note":"unverified bench"}`)
	// Unverified synthetic text covers UTF-8, JSON escapes and private line delimiters.
	const reason = "unverified DO3 → relay → BK+; \"quoted\" \\ wiring\nreason=not a record\t\x00"
	f.put("machine.axes.0.brake_override", canonical(object{{"present", false}, {"reason", reason}}))
	c, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	encoded := hex.EncodeToString([]byte(reason))
	if !strings.Contains(c.MachineIdentity, "axis.0.brake_override_reason_hex="+encoded+"\n") ||
		strings.Count(c.AxisProfiles, "brake_override_reason_hex=") != 1 {
		t.Fatal("override metadata missing or leaked to another axis")
	}
	_, args, err := writeCompiled(c, buildTemp(t))
	if err != nil {
		t.Fatal(err)
	}
	dir, err := os.MkdirTemp("", "bo-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { os.RemoveAll(dir) })
	socket := filepath.Join(dir, "ipc.sock")
	args = append(args, "--socket-path", socket)
	core := exec.Command(filepath.Join(testRoot(t), "build/rosie-rt-core-sim"), args...)
	simfixture.SpawnLogged(t, core, filepath.Join(dir, "core.log"), busTestWatchdog, 12000, nil)
	simfixture.WaitVerified(t, filepath.Join(dir, "metrics.json"), busTestWatchdog, 10*time.Millisecond,
		func(v simfixture.Verified) bool { return v.ConfigVerifiedMask == 63 })
	ctx, cancel := context.WithTimeout(context.Background(), busTestWatchdog)
	defer cancel()
	native, err := ipcclient.Dial(ctx, ipcclient.Options{SocketPath: socket, ConfigurationSHA256: c.SHA256, MachineSHA256: c.MachineSHA256, Backend: "simulation", SimulationTest: true})
	if err != nil {
		t.Fatal(err)
	}
	description, err := native.Describe()
	if err != nil {
		t.Fatal(err)
	}
	if err := native.Close(); err != nil {
		t.Fatal(err)
	}
	check := func(axes []ipcclient.AxisDescription) {
		t.Helper()
		if len(axes) != 6 || axes[0].BrakeOverrideReason != reason {
			t.Fatalf("override: %+v", axes)
		}
		for _, axis := range axes[1:] {
			if axis.BrakeOverrideReason != "" {
				t.Fatalf("unexpected override: %+v", axis)
			}
		}
	}
	check(description.Axes)
	publicSocket := filepath.Join(dir, "control.sock")
	adapter := exec.Command(filepath.Join(testRoot(t), "build/rt-control"), "--core-socket", socket,
		"--socket", publicSocket, "--configuration-sha256", c.SHA256, "--backend", "simulation",
		"--pair-id", "brake-override", "--pair-revision", "1")
	simfixture.SpawnLogged(t, adapter, filepath.Join(dir, "control.log"), busTestWatchdog, 12000, simfixture.CheckExit(t))
	public, err := control.Dial(control.UnixPath(publicSocket))
	if err != nil {
		t.Fatal(err)
	}
	defer public.Close()
	if err := simfixture.Poll(ctx, 10*time.Millisecond, func() (bool, error) {
		_, err := public.Describe(ctx)
		return err == nil, nil
	}); err != nil {
		t.Fatal(err)
	}
	view, err := public.Describe(ctx)
	if err != nil {
		t.Fatal(err)
	}
	check(view.Axes)
	f.put("machine.axes.0.brake_override.reason", `"unverified revised wiring note"`)
	revised, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	if revised.MachineSHA256 == c.MachineSHA256 || revised.SHA256 == c.SHA256 || revised.DeploymentSHA256 != c.DeploymentSHA256 {
		t.Fatal("override reason identity partition")
	}
}

// TestBrakeOverrideNativeRefusal prevents malformed metadata or a contradictory
// effective policy from being advertised as an admitted bench override.
func TestBrakeOverrideNativeRefusal(t *testing.T) {
	for _, mutation := range []string{"empty", "odd", "invalid_hex", "present", "non_bench"} {
		t.Run(mutation, func(t *testing.T) {
			f := busSixAxisFixture(t)
			f.put("machine.bus", `{"unknown_slaves":"hold","unknown_slaves_note":"unverified bench"}`)
			f.put("machine.axes.0.brake_override", brakeOverrideJSON)
			c, err := f.compile()
			if err != nil {
				t.Fatal(err)
			}
			original := "brake_override_reason_hex=" + hex.EncodeToString([]byte("unverified bench wiring"))
			switch mutation {
			case "empty":
				c.AxisProfiles = strings.Replace(c.AxisProfiles, original, "brake_override_reason_hex=", 1)
			case "odd":
				c.AxisProfiles = strings.Replace(c.AxisProfiles, original, "brake_override_reason_hex=0", 1)
			case "invalid_hex":
				c.AxisProfiles = strings.Replace(c.AxisProfiles, original, "brake_override_reason_hex=gg", 1)
			case "present":
				c.AxisProfiles = strings.Replace(c.AxisProfiles, "brake=0|", "brake=1|", 1)
			case "non_bench":
				for i, arg := range c.Argv {
					if arg == "--bus-unknown-slaves" {
						c.Argv[i+1] = "refuse"
					}
				}
			}
			_, args, err := writeCompiled(c, buildTemp(t))
			if err != nil {
				t.Fatal(err)
			}
			output, err := runBringupDaemon(t, append(args, "--validate-config")...)
			if err == nil || !strings.Contains(output, "invalid --axis-profiles file") {
				t.Fatalf("missing parser refusal: %v %s", err, output)
			}
		})
	}
}
