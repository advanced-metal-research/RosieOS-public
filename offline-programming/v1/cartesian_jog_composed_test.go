//go:build linux

package main

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"os"
	"os/exec"
	"path/filepath"
	"syscall"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/targets"
	"rosieos/rt-core/sdk/control"
	"rosieos/rt-core/simfixture"
)

// The production HTTP handler, picker, resolver, adapter and native simulation
// must finish an accepted step after its observer disconnects. Explicit Stop
// and Disarm must still retire motion and expose their cause through state.
func TestCartesianHTTPDisconnectAndStopComposed(t *testing.T) {
	if os.Getenv("RTCORE_COMPOSED") != "1" {
		t.Skip("set RTCORE_COMPOSED=1 after building composed fixtures")
	}
	root, err := filepath.Abs("../../rt-core")
	if err != nil {
		t.Fatal(err)
	}
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "cart-http-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(build) })
	dir, err := os.MkdirTemp("", "ch-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	raw, err := os.ReadFile(filepath.Join(root, "config/machines/simulation/simulation-program.json"))
	if err != nil {
		t.Fatal(err)
	}
	var config map[string]any
	if err := json.Unmarshal(raw, &config); err != nil {
		t.Fatal(err)
	}
	for _, axis := range config["axes"].([]any) {
		limits := axis.(map[string]any)["limits"].(map[string]any)
		// Unverified simulation bounds: +/-2 rad and 1 rad/s.
		limits["min"], limits["max"], limits["max_velocity"] = -2.0, 2.0, 1.0
	}
	raw, err = json.Marshal(config)
	if err != nil {
		t.Fatal(err)
	}
	machine := filepath.Join(build, "machine.json")
	if err := os.WriteFile(machine, raw, 0600); err != nil {
		t.Fatal(err)
	}
	compiled, args, err := simfixture.Compile(root, machine, filepath.Join(build, "config"))
	if err != nil {
		t.Fatal(err)
	}
	spawn := func(name string, args ...string) {
		t.Helper()
		log, err := os.Create(filepath.Join(build, name+".log"))
		if err != nil {
			t.Fatal(err)
		}
		cmd := exec.Command(filepath.Join(root, "build", name), args...)
		cmd.Dir, cmd.Stdout, cmd.Stderr = build, log, log
		if err := cmd.Start(); err != nil {
			log.Close()
			t.Fatal(err)
		}
		t.Cleanup(func() {
			_ = cmd.Process.Signal(syscall.SIGTERM)
			done := make(chan error, 1)
			go func() { done <- cmd.Wait() }()
			select {
			case err := <-done:
				if err != nil {
					t.Errorf("%s: %v", name, err)
				}
			case <-time.After(5000 * time.Millisecond):
				_ = cmd.Process.Kill()
				<-done
				t.Errorf("%s failed to terminate", name)
			}
			log.Close()
			if t.Failed() {
				raw, _ := os.ReadFile(log.Name())
				t.Logf("%s: %s", name, raw)
			}
		})
	}
	// Unverified test watchdog: 180000 ms; only observed state satisfies waits.
	ctx, cancel := context.WithTimeout(context.Background(), 180000*time.Millisecond)
	defer cancel()
	await := func(predicate func() bool) {
		t.Helper()
		ticker := time.NewTicker(10 * time.Millisecond)
		defer ticker.Stop()
		for !predicate() {
			select {
			case <-ctx.Done():
				t.Fatal("Cartesian observation missing", ctx.Err())
			case <-ticker.C:
			}
		}
	}
	coreSocket, socket := filepath.Join(dir, "i.sock"), filepath.Join(dir, "c.sock")
	spawn("rosie-rt-core-sim", append(args, "--socket-path", coreSocket)...)
	await(func() bool {
		raw, _ := os.ReadFile(filepath.Join(dir, "metrics.json"))
		var m struct {
			ConfigVerifiedMask uint32 `json:"config_verified_mask"`
		}
		return json.Unmarshal(raw, &m) == nil && m.ConfigVerifiedMask == 511
	})
	spawn("rt-control", "--core-socket", coreSocket, "--socket", socket, "--configuration-sha256", filepath.Base(compiled), "--backend", "simulation", "--pair-id", "bench", "--pair-revision", "7")
	c, err := control.Dial(control.UnixPath(socket))
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { c.Close() })
	await(func() bool { s, err := c.Status(ctx); return err == nil && s.Motion.TimeNS > 0 })
	binary := os.Getenv("OLP_CARTESIAN_RESOLVER")
	if binary == "" {
		binary = filepath.Join(root, "build/cartmachine/bin/robot-v4-cartesiand")
	}
	p := targets.NewPicker(targets.PickerOptions{
		Catalogue: targets.CellCatalogue{Cells: []targets.Cell{{CellID: "sim", Models: []string{"rosie_1420_v1"}, RequestedMode: "simulation", Execution: targets.CellExecution{Local: true}}}},
		Local:     denseexec.RTCoreConfig{Socket: socket, Binding: control.Binding{PairID: "bench", Revision: 7}, AxisMask: 511},
		Source:    denseexec.Config{ControllerID: "offline-programming:cartesian-http"},
		CartesianResolverFactory: func(model string) denseexec.CartesianResolverService {
			return denseexec.NewCartesianProcess(binary, filepath.Dir(root), model)
		},
	})
	t.Cleanup(func() { p.Close(context.Background()) })
	server := httptest.NewServer(NewServer(ServerOptions{DenseExecution: p}).Handler())
	t.Cleanup(server.Close)
	request := func(ctx context.Context, method, path string, body any, out any) (int, error) {
		raw, err := json.Marshal(body)
		if err != nil {
			return 0, err
		}
		req, err := http.NewRequestWithContext(ctx, method, server.URL+"/api/offline-programming/v1/"+path, bytes.NewReader(raw))
		if err != nil {
			return 0, err
		}
		req.Header.Set("X-RT-Target-Generation", "1")
		req.Header.Set("X-RT-Target-Cell", "sim")
		resp, err := server.Client().Do(req)
		if err != nil {
			return 0, err
		}
		defer resp.Body.Close()
		if out != nil {
			err = json.NewDecoder(resp.Body).Decode(out)
		}
		return resp.StatusCode, err
	}
	ok := func(method, path string, body any, out any) {
		t.Helper()
		if code, err := request(ctx, method, path, body, out); err != nil || code != 200 {
			t.Fatalf("%s: code=%d err=%v", path, code, err)
		}
	}
	state := func() denseexec.JointJogState {
		t.Helper()
		var state denseexec.JointJogState
		ok(http.MethodGet, "dense-execution/cartesian/state", nil, &state)
		return state
	}
	observe := func() control.Status {
		t.Helper()
		s, err := c.Status(ctx)
		if err != nil {
			t.Fatal(err)
		}
		return s
	}
	ok(http.MethodPost, "targets/select", targets.Selection{CellID: "sim", ModelID: "rosie_1420_v1"}, nil)
	for index, action := range []string{"client_disconnected", "stop", "disarm"} {
		t.Run(action, func(t *testing.T) {
			ok(http.MethodPost, "dense-execution/arm", map[string]bool{"armed": true}, nil)
			var status denseexec.StatusBody
			ok(http.MethodGet, "dense-execution/status", nil, &status)
			before := observe()
			in := denseexec.CartesianMoveIntent{TargetID: "bench", SessionID: status.Session.ID, Revision: state().Revision, Frame: "base", Axis: 0, DistanceMM: 100, Fraction: .05}
			if index > 0 {
				in.DistanceMM = -100
			}
			moveCtx, abort := context.WithCancel(ctx)
			defer abort()
			type receipt struct {
				code int
				err  error
				body map[string]any
			}
			done := make(chan receipt, 1)
			go func() {
				body := map[string]any{}
				code, err := request(moveCtx, http.MethodPost, "dense-execution/cartesian/move", in, &body)
				done <- receipt{code, err, body}
			}()
			await(func() bool {
				move := state().Move
				if move.State == "refused" {
					t.Fatalf("move refused: %+v", move)
				}
				if move.State == "moving" {
					feedback := observe()
					for axis := 0; axis < 6; axis++ {
						if feedback.Axes[axis].PositionCounts != before.Axes[axis].PositionCounts {
							return true
						}
					}
				}
				return false
			})
			if action == "client_disconnected" {
				abort()
				if result := <-done; !errors.Is(result.err, context.Canceled) {
					t.Fatalf("abort=%+v", result)
				}
				// No browser heartbeats are sent: the accepted step owns completion.
				await(func() bool {
					move := state().Move
					if move.State == "stopped" || move.State == "refused" {
						t.Fatalf("disconnect retired move: %+v", move)
					}
					return move.State == "done"
				})
				after := observe()
				if after.Core.Armed != 1 || after.Grant.GrantActive != 1 || after.Grant.AppGrantGeneration != before.Grant.AppGrantGeneration || !after.Motion.Done {
					t.Fatalf("disconnect lost authority/completion: core=%+v grant=%+v motion=%+v", after.Core, after.Grant, after.Motion)
				}
				if move := state().Move; move.Revision != in.Revision+1 || move.Reason != "" {
					t.Fatal(move)
				}
			} else {
				// Unverified inhibition ceiling: 3000 ms from request to native off.
				start := time.Now()
				if action == "stop" {
					ok(http.MethodPost, "dense-execution/stop", nil, nil)
				} else {
					ok(http.MethodPost, "dense-execution/arm", map[string]bool{"armed": false}, nil)
				}
				select {
				case result := <-done:
					if result.err != nil || result.code != 409 || result.body["error"] != action {
						t.Fatalf("cancel=%+v", result)
					}
				case <-time.After(3000 * time.Millisecond):
					t.Fatal("inhibition waited for move completion")
				}
				await(func() bool { s := observe(); return s.Core.Armed == 0 && s.Grant.GrantActive == 0 })
				if time.Since(start) > 3000*time.Millisecond {
					t.Fatal("inhibition exceeded 3000 ms")
				}
				if move := state().Move; move.State != "stopped" || move.Reason != action || move.Revision != in.Revision+1 {
					t.Fatal(move)
				}
				stoppedAt := observe().Core.TimeNS
				var held control.Status
				await(func() bool {
					held = observe()
					for axis := 0; axis < 6; axis++ {
						if held.Axes[axis].TimeNS < stoppedAt {
							return false
						}
					}
					return true
				})
				// Unverified sample count: ten distinct native publications must
				// report zero encoder-count change after inhibition.
				for sample := 0; sample < 10; sample++ {
					var next control.Status
					await(func() bool {
						next = observe()
						for axis := 0; axis < 6; axis++ {
							if next.Axes[axis].TimeNS <= held.Axes[axis].TimeNS {
								return false
							}
						}
						return true
					})
					for axis := 0; axis < 6; axis++ {
						if next.Axes[axis].PositionCounts != held.Axes[axis].PositionCounts {
							t.Fatalf("%s did not halt axis %d", action, axis)
						}
					}
					held = next
				}
			}
		})
	}
}
