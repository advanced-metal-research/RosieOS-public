//go:build linux

package simulator

import (
	"context"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/sdk/control"
)

func TestNativeJogLauncherRequiresCompleteBinding(t *testing.T) {
	values := map[string]string{
		"ROSIE_RT_CONTROL_SOCKET": "/sim/control.sock", "ROSIE_RT_JOG_SOCKET": "/sim/jog.sock",
		"ROSIE_RT_PAIR_ID": "local-dev", "ROSIE_RT_PAIR_REVISION": "1", "ROSIE_RT_CONFIGURATION_SHA256": strings.Repeat("a", 64),
		"ROSIE_RT_URDF": "/sim/robot.urdf", "ROSIE_RT_URDF_SHA256": strings.Repeat("b", 64),
	}
	args, err := nativeJogArguments(func(k string) string { return values[k] })
	if err != nil || len(args) != 19 || args[1] != "rt_core" {
		t.Fatalf("native argv: %v %v", args, err)
	}
	for key := range values {
		if _, err := nativeJogArguments(func(k string) string {
			if k == key {
				return ""
			}
			return values[k]
		}); err == nil {
			t.Fatalf("missing %s accepted", key)
		}
	}
}

// TestLocalRTCoreJog observes a public jog sequence and physical simulation counts
// after an OLP intent; a healthy HTTP process alone cannot pass the cutover smoke.
func TestLocalRTCoreJog(t *testing.T) {
	if os.Getenv("DEV_STACK_SMOKE") != "1" {
		t.Skip("run motion-server/v1/tests/dev-stack-smoke.sh")
	}
	root, err := filepath.Abs("../../../..")
	if err != nil {
		t.Fatal(err)
	}
	manager := NewJogManager(root)
	t.Cleanup(manager.Stop)
	if _, err := manager.Start(); err != nil {
		t.Fatal(err)
	}
	client, err := control.Dial(control.UnixPath(os.Getenv("ROSIE_RT_CONTROL_SOCKET")))
	if err != nil {
		t.Fatal(err)
	}
	defer client.Close()
	ctx, cancel := context.WithTimeout(context.Background(), nativeJogStartup)
	defer cancel()
	before, err := client.Status(ctx)
	if err != nil {
		t.Fatal(err)
	}
	ticker := time.NewTicker(nativeJogPoll)
	defer ticker.Stop()
	for {
		if _, err := manager.Intent(JogIntent{Mode: "base", Axes: [6]float64{0, 0, 0, 0, 0, 0.2}, SpeedScale: 0.2}); err != nil {
			t.Fatal(err)
		}
		status, err := client.Status(ctx)
		if err == nil && status.Jog != nil && status.Jog.ObservedSourceSequence > 0 {
			for i, axis := range status.Core.Axes {
				if axis.PositionCounts != before.Core.Axes[i].PositionCounts {
					manager.Stop()
					return
				}
			}
		}
		select {
		case <-ticker.C:
		case <-ctx.Done():
			t.Fatalf("no public jog motion: %s", manager.session.StatusJSON())
		}
	}
}
