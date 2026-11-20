//go:build linux

package targets

import (
	"context"
	"encoding/binary"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"syscall"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
	"golang.org/x/sys/unix"
	codec "rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/sdk/control"
	"rosieos/rt-core/simfixture"
)

// TestPickerComposed runs only local simulation processes. The catalogue binds
// both public transports to a compiled robot definition, not a role-derived model.
func TestPickerComposed(t *testing.T) {
	if os.Getenv("RTCORE_COMPOSED") != "1" {
		t.Skip("set RTCORE_COMPOSED=1 after make control sim")
	}
	root, _ := filepath.Abs("../../../../rt-core")
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "picker-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { os.RemoveAll(build) })
	dir, err := os.MkdirTemp("", "olpp-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { os.RemoveAll(dir) })
	write := func(path string, raw []byte) {
		t.Helper()
		if err := os.MkdirAll(filepath.Dir(path), 0700); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(path, raw, 0600); err != nil {
			t.Fatal(err)
		}
	}
	read := func(path string) []byte {
		t.Helper()
		raw, err := os.ReadFile(path)
		if err != nil {
			t.Fatal(err)
		}
		return raw
	}
	marshal := func(value any) []byte {
		t.Helper()
		raw, err := json.Marshal(value)
		if err != nil {
			t.Fatal(err)
		}
		return raw
	}
	fixtureRoot := filepath.Join(build, "repository")
	profiles := filepath.Join(fixtureRoot, "rt-core/config/drives")
	var profile map[string]any
	if err := json.Unmarshal(read(filepath.Join(root, "config/drives/cia402_sim_home.json")), &profile); err != nil {
		t.Fatal(err)
	}
	profile["position_semantics"] = map[string]any{"drive_native_ratio_enabled": false, "position_semantics_source": "motor_shaft"}
	write(filepath.Join(profiles, "cia402_sim_home.json"), marshal(profile))
	var machine map[string]any
	robot := stageDescription(t, filepath.Dir(root), fixtureRoot, "rosie_1400_v3", func(definition map[string]any) {
		definition["defaults"] = map[string]any{"drive_profile": "cia402_sim_home", "home_policy": "optional", "coordinate_evidence_mode": "multi_turn"}
	})
	if err := json.Unmarshal(read(filepath.Join(root, "config/machines/simulation/simulation-program.json")), &machine); err != nil {
		t.Fatal(err)
	}
	var allowed unix.CPUSet
	if err := unix.SchedGetaffinity(0, &allowed); err != nil {
		t.Fatal(err)
	}
	if allowed.Count() < 2 {
		t.Skip("simulation requires two allowed CPUs")
	}
	for cpu := 0; cpu < 1024; cpu++ {
		if allowed.IsSet(cpu) {
			machine["runtime"].(map[string]any)["cpu"] = cpu
			break
		}
	}
	machine["robot"] = robot
	for i, value := range machine["axes"].([]any) {
		axis := value.(map[string]any)
		// Robot-backed fixtures inherit travel from the staged URDF too.
		limits := axis["limits"].(map[string]any)
		delete(limits, "min")
		delete(limits, "max")
		machine["axes"].([]any)[i] = map[string]any{"robot_joint": fmt.Sprintf("J%d", i+1), "slave_position": i, "require_home": false, "limits": axis["limits"]}
	}
	machinePath := filepath.Join(fixtureRoot, "rt-core/config/machine.json")
	write(machinePath, marshal(machine))
	write(filepath.Join(fixtureRoot, "rt-core/config/cells/bench-relay.json"), []byte(`{"name":"bench-relay","nodes":[{"role":"realtime-core","rt_core":{"machine_config":"rt-core/config/machine.json"}}]}`))
	compiled, args, err := simfixture.CompileProfiles(root, machinePath, profiles, filepath.Join(build, "config"))
	if err != nil {
		t.Fatal(err)
	}
	spawn := func(label, binary string, args ...string) *exec.Cmd {
		t.Helper()
		log, err := os.Create(filepath.Join(build, label+".log"))
		if err != nil {
			t.Fatal(err)
		}
		cmd := exec.Command(filepath.Join(root, "build", binary), args...)
		cmd.Dir = build
		cmd.Stdout, cmd.Stderr = log, log
		if err := cmd.Start(); err != nil {
			log.Close()
			t.Fatal(err)
		}
		t.Cleanup(func() {
			_ = cmd.Process.Signal(syscall.SIGTERM)
			done := make(chan error, 1)
			go func() { done <- cmd.Wait() }()
			// Unverified cleanup ceiling; process exit is required.
			select {
			case err := <-done:
				if err != nil {
					t.Errorf("%s exit: %v", label, err)
				}
			case <-time.After(5000 * time.Millisecond):
				cmd.Process.Kill()
				<-done
				t.Errorf("%s did not exit", label)
			}
			log.Close()
			if t.Failed() {
				t.Logf("%s: %s", label, read(log.Name()))
			}
		})
		return cmd
	}
	ctx, cancel := context.WithTimeout(context.Background(), 30000*time.Millisecond)
	defer cancel()
	configs := make([]denseexec.RTCoreConfig, 2)
	observers := make([]*control.Client, 2)
	pkiRoot, err := os.MkdirTemp("", "rosie-rt-pki-test.")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { os.RemoveAll(pkiRoot) })
	pki := filepath.Join(pkiRoot, "pki")
	for _, args := range [][]string{{"init", pki, "picker"}, {"issue-client", pki, "olp-dev"}} {
		cmd := exec.Command("bash", append([]string{filepath.Join(root, "host/remote-pki.sh")}, args...)...)
		cmd.Env = append(os.Environ(), "ROSIE_RT_PKI_TEST_ROOT="+pkiRoot)
		if out, err := cmd.CombinedOutput(); err != nil {
			t.Fatalf("PKI: %v %s", err, out)
		}
	}
	for i := range configs {
		runtime := filepath.Join(dir, strconv.Itoa(i))
		if err := os.Mkdir(runtime, 0700); err != nil {
			t.Fatal(err)
		}
		coreSocket, socket := filepath.Join(runtime, "ipc.sock"), filepath.Join(runtime, "control.sock")
		spawn(fmt.Sprintf("sim%d", i), "rosie-rt-core-sim", append(args, "--socket-path", coreSocket)...)
		pickerAwait(t, func() bool {
			raw, _ := os.ReadFile(filepath.Join(runtime, "metrics.json"))
			var m struct {
				Mask uint32 `json:"config_verified_mask"`
			}
			return json.Unmarshal(raw, &m) == nil && m.Mask == 511
		})
		controlArgs := []string{"--core-socket", coreSocket, "--socket", socket, "--compiled-config", compiled, "--configuration-sha256", filepath.Base(compiled), "--backend", "simulation", "--pair-id", "picker", "--pair-revision", "7"}
		if i == 1 {
			controlArgs = append(controlArgs, "--remote-listen", "127.0.0.1:0", "--remote-ca", filepath.Join(pki, "ca.pem"), "--remote-cert", filepath.Join(pki, "server.pem"), "--remote-key", filepath.Join(pki, "server-key.pem"), "--remote-crl", filepath.Join(pki, "crl.pem"))
		}
		adapter := spawn(fmt.Sprintf("control%d", i), "rt-control", controlArgs...)
		observer, err := control.Dial(control.UnixPath(socket))
		if err != nil {
			t.Fatal(err)
		}
		observers[i] = observer
		t.Cleanup(func() { observer.Close() })
		pickerAwait(t, func() bool {
			d, err := observer.Describe(ctx)
			return err == nil && d.ConfigurationSHA256 == filepath.Base(compiled)
		})
		cfg := denseexec.RTCoreConfig{Socket: socket, Binding: control.Binding{PairID: "picker", Revision: 7, ConfigurationSHA256: filepath.Base(compiled)}, AxisMask: 511, ExpectedBackend: "simulation"}
		if i == 1 {
			cfg.Socket = ""
			pickerAwait(t, func() bool { cfg.Address = pickerAddress(adapter.Process.Pid); return cfg.Address != "" })
			cfg.CAFile = filepath.Join(pki, "ca.pem")
			cfg.CertificateFile = filepath.Join(pki, "clients/olp-dev.pem")
			cfg.KeyFile = filepath.Join(pki, "clients/olp-dev-key.pem")
		}
		configs[i] = cfg
	}
	// Bind the allocated HTTPS listener and exact pair before catalogue admission.
	write(filepath.Join(fixtureRoot, "rt-core/config/cells/bench-relay.json"), marshal(map[string]any{"name": "bench-relay", "nodes": []any{map[string]any{"role": "realtime-core", "rt_core": map[string]any{"machine_config": "rt-core/config/machine.json", "remote_listen": strings.TrimPrefix(configs[1].Address, "https://"), "pair_id": "picker", "pair_revision": 7}}}}))
	cataloguePath := filepath.Join(build, "cells.json")
	write(cataloguePath, marshal(map[string]any{"schema": CellCatalogueSchema, "cells": []any{
		map[string]any{"cell_id": "local-simulation", "label": "Local simulation", "role": "LOCAL SIMULATION", "models": []string{"rosie_1400_v3"}, "execution": map[string]any{"local": true}, "requested_mode": "simulation"},
		map[string]any{"cell_id": "bench-relay", "label": "Detached bench", "role": "TEST BENCH / BENCH-RELAY", "models": []string{"rosie_1400_v3"}, "execution": configs[1], "requested_mode": "simulation"},
	}}))
	catalogue, err := ReadCellCatalogue(cataloguePath, fixtureRoot)
	if err != nil {
		t.Fatal(err)
	}
	plan, err := codec.Decode(read(filepath.Join(root, "adapters/rosie/densejointtraj/testdata/corpus/dense_joint_trajectory_reference.rdt")))
	if err != nil {
		t.Fatal(err)
	}
	for i := range plan.Segments {
		for j := range plan.Segments[i].Samples {
			s := &plan.Segments[i].Samples[j]
			for k := range s.QRad {
				s.QRad[k], s.QdRadS[k] = 0, 0
			}
			s.TorchOn = false
		}
	}
	blob, err := codec.Encode(plan.Header, plan.Segments)
	if err != nil {
		t.Fatal(err)
	}
	// This fixture is bound to a staged robot, so its retained program must
	// carry the exact compiled description and served calibration identity.
	described, err := observers[0].Describe(ctx)
	if err != nil || described.Robot == nil {
		t.Fatalf("compiled robot description: %+v %v", described.Robot, err)
	}
	headerLen := int(binary.BigEndian.Uint32(blob[:4]))
	var header map[string]json.RawMessage
	if err := json.Unmarshal(blob[4:4+headerLen], &header); err != nil {
		t.Fatal(err)
	}
	header["robot_cell"] = marshal(map[string]string{
		"model_id":                            described.Robot.ModelID,
		"robot_description_sha256":            described.Robot.RobotDescriptionSHA256,
		"machine_planning_calibration_sha256": described.Robot.MachinePlanningCalibrationSHA256,
		"machine_planning_calibration_source": "target",
	})
	pinnedHeader := marshal(header)
	pinned := append(binary.BigEndian.AppendUint32(nil, uint32(len(pinnedHeader))), pinnedHeader...)
	blob = append(pinned, blob[4+headerLen:]...)
	encoded, err := codec.Decode(blob)
	if err != nil {
		t.Fatal(err)
	}
	h := encoded.Header
	id := denseexec.Identity{PlanID: h.PlanID, ProgramID: h.ProgramID, ProgramDigest: h.ProgramDigest, TrajectoryDigest: h.TrajectoryDigest, PlanRevision: h.PlanRevision, ManifestRevision: h.ManifestRevision}
	p := NewPicker(PickerOptions{Catalogue: catalogue, Local: configs[0], Source: denseexec.Config{ControllerID: "offline-programming:picker-test", FetchBlob: func(context.Context, string) ([]byte, error) { return blob, nil }}})
	t.Cleanup(func() { p.Close(context.Background()) })
	list := p.List(ctx)
	if len(list.Targets) != 2 || list.Selected != nil {
		t.Fatalf("list: %+v", list)
	}
	for _, view := range list.Targets {
		if view.Backend != "simulation" || view.Reason != "" {
			t.Fatalf("discovery: %+v", view)
		}
	}
	for i, cellID := range []string{"local-simulation", "bench-relay"} {
		if _, err := p.Select(ctx, Selection{cellID, "rosie_1400_v3"}); err != nil {
			t.Fatal(err)
		}
		for _, observer := range observers {
			s, err := observer.Status(ctx)
			if err != nil || s.Core.Armed != 0 || s.Core.AxisEnableMask != 0 {
				t.Fatalf("selection granted authority: %+v %v", s.Core, err)
			}
		}
		if _, err := p.Arm(ctx, true); err != nil {
			t.Fatal(err)
		}
		loaded, err := p.Load(ctx, denseexec.LoadRequest{Identity: id})
		if err != nil {
			t.Fatal(err)
		}
		if _, err := p.Play(ctx, loaded.SessionID); err != nil {
			t.Fatal(err)
		}
		other := "bench-relay"
		if i == 1 {
			other = "local-simulation"
		}
		before, err := observers[i].Status(ctx)
		if err != nil || before.Grant == nil {
			t.Fatalf("grant before switch: %+v %v", before, err)
		}
		beforeSession := p.Status(ctx).Session.ID
		_, err = p.Select(ctx, Selection{other, "rosie_1400_v3"})
		var e *denseexec.Error
		if !errors.As(err, &e) || e.Code != "cell_switch_while_playing" {
			t.Fatalf("mid-play switch: %v", err)
		}
		// Retain the review's native grant and process-session preservation probe.
		after, err := observers[i].Status(ctx)
		if err != nil || after.Grant == nil || after.Grant.GrantActive != 1 || after.Grant.AppGrantGeneration != before.Grant.AppGrantGeneration || after.Grant.CapabilityId != before.Grant.CapabilityId || after.Core.ControlGeneration != before.Core.ControlGeneration {
			t.Fatalf("switch changed live grant: before=%+v after=%+v error=%v", before, after, err)
		}
		session := p.Status(ctx).Session
		if session == nil || session.ID != beforeSession || !session.Lease.Held {
			t.Fatalf("switch lost session grant: %+v", session)
		}
		pickerAwait(t, func() bool { _, _ = p.Heartbeat(loaded.SessionID); return p.Status(ctx).Session.State == "done" })
		s, err := observers[i].Status(ctx)
		if err != nil || s.Execution.State != "completed" || s.Execution.NativeSequence == 0 {
			t.Fatalf("execution: %+v %v", s.Execution, err)
		}
		t.Logf("%s: compiled robot pin accepted; native Load/Play completed; mid-play target switch refused", cellID)
	}
	// A target switch retires the live hold slot (joint and Cartesian path
	// holds share it; this simulation fixture has no jog_jerk, so no path_jog).
	t.Run("jog_target_switch", func(t *testing.T) {
		if _, err := p.Select(ctx, Selection{"local-simulation", "rosie_1400_v3"}); err != nil {
			t.Fatal(err)
		}
		if _, err := p.Arm(ctx, true); err != nil {
			t.Fatal(err)
		}
		in := denseexec.JointJogIntent{TargetID: "picker", SessionID: p.Status(ctx).Session.ID, Revision: p.JointJogState().Revision, Axis: 0, Direction: 1, Fraction: 0.1}
		if _, err := p.JointJog(ctx, "begin", in); err != nil {
			t.Fatal(err)
		}
		if _, err := p.JointJog(ctx, "update", in); err != nil {
			t.Fatal(err)
		}
		if _, err := p.Select(ctx, Selection{"bench-relay", "rosie_1400_v3"}); err != nil {
			t.Fatal(err)
		}
		pickerAwait(t, func() bool {
			s, err := observers[0].Status(ctx)
			return err == nil && s.Jog != nil && s.Jog.Open == 0 && s.Core.Armed == 0 && s.Motion.Done
		})
		if _, err := p.JointJog(ctx, "update", in); err == nil {
			t.Fatal("old hold adopted replacement selection")
		}
		s, err := observers[1].Status(ctx)
		if err != nil || s.Core.Armed != 0 || p.JointJogState().Active {
			t.Fatal("replacement machine acquired motion", err)
		}
	})
	for _, tc := range []struct{ model, mode, reason string }{{"rosie_1420_v1", "simulation", "cell_model_mismatch"}, {"rosie_1400_v3", "real", "cell_mode_mismatch"}} {
		p.options.Catalogue.Cells[1].RequestedMode = tc.mode
		_, err := p.Select(ctx, Selection{"bench-relay", tc.model})
		var e *denseexec.Error
		if !errors.As(err, &e) || e.Code != tc.reason {
			t.Fatalf("%s: %v", tc.reason, err)
		}
	}
	if _, err := p.Stop(ctx); err != nil {
		t.Fatal(err)
	}
}

func pickerAwait(t *testing.T, predicate func() bool) {
	t.Helper()
	// Unverified observation ceiling and polling cadence; only evidence passes.
	ctx, cancel := context.WithTimeout(context.Background(), 5000*time.Millisecond)
	defer cancel()
	tick := time.NewTicker(time.Millisecond)
	defer tick.Stop()
	for !predicate() {
		select {
		case <-ctx.Done():
			t.Fatal("picker observation not reached")
		case <-tick.C:
		}
	}
}

// Resolve the owning listener's assigned port instead of racing to reserve one.
func pickerAddress(pid int) string {
	entries, _ := os.ReadDir(fmt.Sprintf("/proc/%d/fd", pid))
	inodes := map[string]bool{}
	for _, entry := range entries {
		link, _ := os.Readlink(fmt.Sprintf("/proc/%d/fd/%s", pid, entry.Name()))
		if strings.HasPrefix(link, "socket:[") {
			inodes[strings.TrimSuffix(strings.TrimPrefix(link, "socket:["), "]")] = true
		}
	}
	raw, _ := os.ReadFile("/proc/net/tcp")
	for _, line := range strings.Split(string(raw), "\n") {
		f := strings.Fields(line)
		if len(f) < 10 || f[3] != "0A" || !inodes[f[9]] {
			continue
		}
		a := strings.Split(f[1], ":")
		if len(a) != 2 || a[0] != "0100007F" {
			continue
		}
		port, err := strconv.ParseUint(a[1], 16, 16)
		if err == nil && port != 0 {
			return fmt.Sprintf("https://127.0.0.1:%d", port)
		}
	}
	return ""
}
