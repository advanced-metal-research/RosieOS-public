//go:build linux

package denseexec

import (
	"context"
	"encoding/json"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"testing"
	"time"

	"rosieos/rt-core/sdk/control"
)

// The production resolver and native simulation must complete the signed step
// sequence and rotation; independent FK prevents false TCP completion claims.
func TestCartesianMoveComposed(t *testing.T) {
	c, d, root, socket := rtComposedFixture(t, func(config map[string]any) {
		for _, axis := range config["axes"].([]any) {
			limits := axis.(map[string]any)["limits"].(map[string]any)
			// Unverified simulation bounds: +/-2 rad and 1 rad/s.
			limits["min"], limits["max"], limits["max_velocity"] = -2.0, 2.0, 1.0
		}
	})
	r, err := NewRTCore(RTCoreConfig{Socket: socket, Binding: control.Binding{PairID: "bench", Revision: 7, MachineSHA256: d.MachineSHA256}, DeploymentSHA256: d.DeploymentSHA256, CapabilitiesDigest: d.CapabilitiesDigest, AxisMask: 511, CartesianModel: "rosie_1420_v1"}, Config{ControllerID: "offline-programming:cartesian-step"}, c)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	// The resolver and its FK oracle default to the composed fixture build under
	// rt-core/build/cartmachine/bin; a checkout that built them elsewhere names
	// them with OLP_CARTESIAN_RESOLVER and OLP_CARTESIAN_RESOLVER_TEST, as the
	// Cartesian hold and picker compositions already allow.
	resolverBinary := os.Getenv("OLP_CARTESIAN_RESOLVER")
	if resolverBinary == "" {
		resolverBinary = filepath.Join(root, "build/cartmachine/bin/robot-v4-cartesiand")
	}
	fkBinary := os.Getenv("OLP_CARTESIAN_RESOLVER_TEST")
	if fkBinary == "" {
		fkBinary = filepath.Join(root, "build/cartmachine/bin/cartesian_resolver_test")
	}
	r.SetCartesianResolverFactory(func(model string) CartesianResolverService {
		return NewCartesianProcess(resolverBinary, filepath.Dir(root), model)
	})
	// Unverified test watchdog: 180000 ms for the complete command sequence.
	ctx, cancel := context.WithTimeout(context.Background(), 180000*time.Millisecond)
	defer cancel()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	session := r.Status(ctx).Session.ID
	heartbeatCtx, heartbeatCancel := context.WithCancel(ctx)
	done := make(chan struct{})
	defer func() { heartbeatCancel(); <-done }()
	go func() {
		defer close(done)
		ticker := time.NewTicker(100 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-heartbeatCtx.Done():
				return
			case <-ticker.C:
				_, _ = r.Heartbeat(session)
			}
		}
	}()
	fk := func(s control.Status) [12]float64 {
		args := []string{"--fk", "rosie_1420_v1"}
		for i := 0; i < 6; i++ {
			args = append(args, strconv.FormatFloat(s.Axes[i].LogicalPosition, 'g', 17, 64))
		}
		command := exec.CommandContext(ctx, fkBinary, args...)
		command.Dir = filepath.Join(root, "../motion-server/v1")
		raw, err := command.Output()
		if err != nil {
			t.Fatal(err)
		}
		var result [12]float64
		if err := json.Unmarshal(raw, &result); err != nil {
			t.Fatal(err)
		}
		return result
	}
	for _, step := range []struct {
		axis     int
		distance float64
	}{{0, 20}, {0, -20}, {0, 100}, {0, -100}, {3, 0}} {
		axis := step.axis
		before, err := c.Status(ctx)
		if err != nil {
			t.Fatal(err)
		}
		origin := fk(before)
		in := CartesianMoveIntent{TargetID: "bench", SessionID: session, Revision: r.JointJogState().Revision, Frame: "base", Axis: axis, Fraction: .05}
		if axis == 0 {
			in.DistanceMM = step.distance
		} else {
			in.AngleDegrees = 15
		}
		state, err := r.CartesianMove(ctx, in)
		if err != nil || state.State != "done" {
			t.Fatalf("axis=%d state=%+v err=%v", axis, state, err)
		}
		after, err := c.Status(ctx)
		if err != nil {
			t.Fatal(err)
		}
		end := fk(after)
		for row := 0; row < 3; row++ {
			want := origin[row*4+3]
			if axis == 0 && row == 0 {
				want += step.distance / 1000
			}
			// Unverified absolute endpoint tolerance: 0.5 mm including encoder counts.
			if math.Abs(end[row*4+3]-want) > .0005 {
				t.Fatalf("TCP row=%d moved=%g m", row, end[row*4+3]-origin[row*4+3])
			}
		}
		angle := 0.0
		if axis == 3 {
			angle = 15 * math.Pi / 180
		}
		rotation := [3][3]float64{{1, 0, 0}, {0, math.Cos(angle), -math.Sin(angle)}, {0, math.Sin(angle), math.Cos(angle)}}
		for row := 0; row < 3; row++ {
			for col := 0; col < 3; col++ {
				want := 0.0
				for k := 0; k < 3; k++ {
					want += rotation[row][k] * origin[k*4+col]
				}
				if math.Abs(end[row*4+col]-want) > .0005 {
					t.Fatalf("TCP rotation row=%d col=%d got=%g want=%g", row, col, end[row*4+col], want)
				}
			}
		}
		if after.Core.Armed != 1 || after.Grant.GrantActive != 1 || after.Grant.AppGrantGeneration != before.Grant.AppGrantGeneration || !after.Motion.Done {
			t.Fatal("move changed armed state or lacked completion")
		}
		for i := 6; i < len(after.Axes); i++ {
			if after.Axes[i].PositionCounts != before.Axes[i].PositionCounts {
				t.Fatalf("axis %d moved", i)
			}
		}
	}
}
