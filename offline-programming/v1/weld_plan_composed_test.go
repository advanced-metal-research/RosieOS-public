//go:build linux

package main

import (
	"bytes"
	"context"
	"encoding/base64"
	"encoding/json"
	"fmt"
	"io"
	"math"
	"net"
	"net/http"
	"net/http/httptest"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"regexp"
	"strings"
	"sync"
	"syscall"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/densejointtraj"
	"rosieos/offline-programming/v1/internal/seam"
	"rosieos/offline-programming/v1/internal/targets"
	"rosieos/rt-core/sdk/control"
	"rosieos/rt-core/simfixture"
)

// The Steam Deck pendant's teach-and-replay chain on the 1430 cell's simulation
// configuration, end to end off the robot: three waypoints recorded by the
// pendant's own olp-core (ui/test/pendant/deck-teach.mjs) from the measured
// pose, POST /weld-plan with the Deck's planBody through the seam worker and
// the weld planner's CPU motion server (pixi env motion-cpu, as example-relay runs
// it), Load with the kept identity, Arm, Play to completion through the
// production picker, public adapter and native simulator. Then the states an
// operator meets: plan while armed after a Move-to or a held jog, plan after
// STOP, Play off the plan start, Play while disarmed, Load after a re-Home.
//
// It exists because this chain had never completed on the 1430 cell (example-relay's OLP
// journal since 2026-10-06: 0 loads, 0 plays; its planner's dense store
// empty). Every wait is on observed native or server state; refusals are
// recorded with the Deck's operator text and how long they took.
func TestWeldPlanTaughtMovesComposed(t *testing.T) {
	if os.Getenv("RTCORE_COMPOSED") != "1" || os.Getenv("WELD_PLAN_COMPOSED") != "1" {
		t.Skip("set RTCORE_COMPOSED=1 WELD_PLAN_COMPOSED=1 after make control sim in rt-core, npm ci in ui and pixi install -e motion-cpu in weld_planner/v1")
	}
	repo, err := filepath.Abs("../..")
	if err != nil {
		t.Fatal(err)
	}
	root := filepath.Join(repo, "rt-core")
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "olp-weldplan-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(build) })
	dir, err := os.MkdirTemp("", "wp-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })

	report := &weldPlanReport{}
	t.Cleanup(func() { report.flush(t) })

	raw, err := os.ReadFile(filepath.Join(root, "config/machines/simulation/simulation-rosie1420.json"))
	if err != nil {
		t.Fatal(err)
	}
	var machine map[string]json.RawMessage
	if err := json.Unmarshal(raw, &machine); err != nil {
		t.Fatal(err)
	}
	// Unverified container scheduling allowance: 200000000 ns, as
	// TestTestProgramInertiaPlanComposed carries it (Docker on a loaded Mac,
	// 2026-10-09). Every axis, limit and Home rule stays as compiled.
	machine["max_cycle_lateness_ns"] = json.RawMessage("200000000")
	// Move-to's jerk bound, as a cell's catalogue binding carries it
	// (targets/cells.go: the compiled machine's jog_jerk).
	var jogJerk struct {
		JerkRadS3 float64 `json:"jerk_rad_s3"`
	}
	if err := json.Unmarshal(machine["jog_jerk"], &jogJerk); err != nil || jogJerk.JerkRadS3 <= 0 {
		t.Fatalf("simulation-rosie1420.json jog_jerk: %v", err)
	}
	if raw, err = json.Marshal(machine); err != nil {
		t.Fatal(err)
	}
	machinePath := filepath.Join(build, "simulation-rosie1420.json")
	if err := os.WriteFile(machinePath, raw, 0o600); err != nil {
		t.Fatal(err)
	}
	compiled, args, err := simfixture.Compile(root, machinePath, filepath.Join(build, "config"))
	if err != nil {
		t.Fatal(err)
	}
	spawn := func(name, binary, workdir string, env []string, args ...string) *exec.Cmd {
		t.Helper()
		log, err := os.Create(filepath.Join(build, name+".log"))
		if err != nil {
			t.Fatal(err)
		}
		cmd := exec.Command(binary, args...)
		cmd.Dir, cmd.Stdout, cmd.Stderr = workdir, log, log
		cmd.Env = append(os.Environ(), env...)
		// Its own group: pixi run leaves the planner as a child.
		cmd.SysProcAttr = &syscall.SysProcAttr{Setpgid: true}
		if err := cmd.Start(); err != nil {
			log.Close()
			t.Fatal(err)
		}
		t.Cleanup(func() {
			_ = syscall.Kill(-cmd.Process.Pid, syscall.SIGTERM)
			done := make(chan error, 1)
			go func() { done <- cmd.Wait() }()
			select {
			case <-done:
			case <-time.After(10000 * time.Millisecond):
				_ = syscall.Kill(-cmd.Process.Pid, syscall.SIGKILL)
				<-done
				t.Errorf("%s failed to terminate", name)
			}
			log.Close()
			raw, _ := os.ReadFile(log.Name())
			if keep := os.Getenv("WELD_PLAN_LOGS"); keep != "" {
				_ = os.MkdirAll(keep, 0o755)
				_ = os.WriteFile(filepath.Join(keep, name+".log"), raw, 0o644)
			}
			if t.Failed() {
				if len(raw) > 12000 {
					raw = raw[len(raw)-12000:]
				}
				t.Logf("%s: %s", name, raw)
			}
		})
		return cmd
	}

	// Unverified test watchdog: 90 min. A CPU plan of taught moves on arm64
	// had never been timed when this was written; the server allows 30 min
	// per /weld-plan (weldPlanTimeout).
	ctx, cancel := context.WithTimeout(context.Background(), 90*time.Minute)
	defer cancel()

	// helper runs the planner and the pendant driver below everything the
	// servo host would run (the planner is on the OLP host, the Deck is its
	// own machine): on one loaded container host, node's start-up starved
	// OLP's 500 ms grant renewal (grant_expired 1 s after Arm, 2026-10-11).
	helper := func(binary string) []string {
		if nice, err := exec.LookPath("nice"); err == nil {
			return []string{nice, "-n", "19", binary}
		}
		return []string{binary}
	}

	// The weld planner's CPU motion server, as `pixi run -e motion-cpu
	// motion-cpu-serve` on example-relay, unless an origin is given.
	origin := os.Getenv("WELD_PLAN_MOTION_ORIGIN")
	plannerLog := ""
	if origin == "" {
		listener, err := net.Listen("tcp", "127.0.0.1:0")
		if err != nil {
			t.Fatal(err)
		}
		port := listener.Addr().(*net.TCPAddr).Port
		listener.Close()
		pixi := os.Getenv("PIXI")
		if pixi == "" {
			pixi = "pixi"
		}
		began := time.Now()
		// The planner refuses every plan when the pin is not a full commit;
		// unset, its results say the source is unknown (native_result.py).
		var plannerEnv []string
		if revision := os.Getenv("WELD_PLAN_SOURCE_REVISION"); revision != "" {
			plannerEnv = append(plannerEnv, "AMR_WELD_PLANNER_SOURCE_REVISION="+revision)
		}
		plannerArgs := append(helper(pixi), "run", "--frozen", "-e", "motion-cpu", "motion-cpu-serve", "--port", fmt.Sprint(port), "--dense-store-dir", filepath.Join(build, "dense"))
		spawn("motion-cpu", plannerArgs[0], filepath.Join(repo, "weld_planner", "v1"), plannerEnv, plannerArgs[1:]...)
		origin = fmt.Sprintf("http://127.0.0.1:%d", port)
		plannerLog = filepath.Join(build, "motion-cpu.log")
		// Unverified planner start bound: 300 s (imports torch on arm64).
		deadline := time.Now().Add(300 * time.Second)
		for {
			resp, err := http.Get(origin + "/api/motion/health")
			if err == nil {
				resp.Body.Close()
				if resp.StatusCode == http.StatusOK {
					break
				}
			}
			if time.Now().After(deadline) {
				t.Fatalf("motion server did not answer /api/motion/health in 300 s: %v", err)
			}
			time.Sleep(250 * time.Millisecond)
		}
		report.add("planner start to health", "%.1f s", time.Since(began).Seconds())
	}
	if resp, err := http.Get(origin + "/api/motion/health"); err == nil {
		health, _ := io.ReadAll(resp.Body)
		resp.Body.Close()
		report.add("planner health", "%s", strings.TrimSpace(string(health)))
	}

	const axisMask = 0x3f
	// the 1430 cell's pair_revision (rt-core/config/cells/example-arm-1420.json).
	// The Deck and the browser plan manifest_revision 1 (rails/planRequest.ts)
	// and rt-control refuses a Load of any other (manifest_revision_mismatch).
	const pairRevision = 1
	stateDetail := func() string { return "" }
	waitFor := func(tt *testing.T, what string, limit time.Duration, predicate func() bool) {
		tt.Helper()
		deadline := time.Now().Add(limit)
		ticker := time.NewTicker(10 * time.Millisecond)
		defer ticker.Stop()
		for !predicate() {
			if time.Now().After(deadline) {
				tt.Fatalf("%s: not observed in %s; %s", what, limit, stateDetail())
			}
			select {
			case <-ctx.Done():
				tt.Fatalf("%s: not observed (%v); %s", what, ctx.Err(), stateDetail())
			case <-ticker.C:
			}
		}
	}
	// The seam worker as example-relay's OLP unit launches it: SEAM_WORKER_PYTHON
	// is the motion-cpu environment's python (cell_deploy_scripts.go). Its
	// spares and the planner's imports start before the simulator, whose
	// Home is the step a loaded host breaks.
	seams, err := seam.New(seam.Config{WeldPlannerRoot: filepath.Join(repo, "weld_planner", "v1")})
	if err != nil {
		t.Fatal(err)
	}
	seams.Warm()
	t.Cleanup(seams.Close)

	coreSocket, socket := filepath.Join(dir, "i.sock"), filepath.Join(dir, "c.sock")
	spawn("rosie-rt-core-sim", filepath.Join(root, "build", "rosie-rt-core-sim"), build, nil, append(args, "--socket-path", coreSocket)...)
	waitFor(t, "configuration verified", 60*time.Second, func() bool {
		raw, _ := os.ReadFile(filepath.Join(dir, "metrics.json"))
		var m struct {
			ConfigVerifiedMask uint32 `json:"config_verified_mask"`
		}
		return json.Unmarshal(raw, &m) == nil && m.ConfigVerifiedMask&axisMask == axisMask
	})
	spawn("rt-control", filepath.Join(root, "build", "rt-control"), build, nil, "--core-socket", coreSocket, "--socket", socket, "--compiled-config", compiled,
		"--configuration-sha256", filepath.Base(compiled), "--backend", "simulation", "--pair-id", "bench", "--pair-revision", fmt.Sprint(pairRevision))
	c, err := control.Dial(control.UnixPath(socket))
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { c.Close() })
	waitFor(t, "native status", 60*time.Second, func() bool { s, err := c.Status(ctx); return err == nil && s.Motion.TimeNS > 0 })
	d, err := c.Describe(ctx)
	if err != nil {
		t.Fatal(err)
	}
	if d.Robot == nil || d.Robot.ModelID != "rosie_1420_v1" || len(d.Axes) != 6 {
		t.Fatalf("the simulator does not describe rosie_1420_v1's six axes: %+v", d.Robot)
	}
	tolerance := make([]float64, 6)
	for i := range tolerance {
		tolerance[i] = d.Axes[i].CompletionTolerance
	}

	p := targets.NewPicker(targets.PickerOptions{
		Catalogue: targets.CellCatalogue{Cells: []targets.Cell{{CellID: "sim", Models: []string{"rosie_1420_v1"}, RequestedMode: "simulation", Execution: targets.CellExecution{Local: true}}}},
		Local:     denseexec.RTCoreConfig{Socket: socket, Binding: control.Binding{PairID: "bench", Revision: pairRevision}, AxisMask: axisMask, MoveJerk: jogJerk.JerkRadS3},
		Source:    denseexec.Config{ControllerID: "offline-programming:weld-plan-composed", MotionOrigin: origin},
	})
	t.Cleanup(func() { p.Close(context.Background()) })
	server := httptest.NewServer(NewServer(ServerOptions{
		DenseExecution: p, Seams: seams, MotionOrigin: origin,
		RobotDescriptionRoot:  filepath.Join(repo, "robot_description", "robots"),
		RobotDescriptionCache: filepath.Join(build, "description-cache"),
	}).Handler())
	t.Cleanup(server.Close)

	const clientID = "deck-composed"
	cur := t
	request := func(method, path string, body any, out any) (int, map[string]any) {
		cur.Helper()
		raw, err := json.Marshal(body)
		if err != nil {
			cur.Fatal(err)
		}
		req, err := http.NewRequestWithContext(ctx, method, server.URL+"/api/offline-programming/v1/"+path, bytes.NewReader(raw))
		if err != nil {
			cur.Fatal(err)
		}
		req.Header.Set("X-RT-Target-Generation", "1")
		req.Header.Set("X-RT-Target-Cell", "sim")
		req.Header.Set(clientIDHeader, clientID)
		resp, err := server.Client().Do(req)
		if err != nil {
			cur.Fatal(err)
		}
		defer resp.Body.Close()
		payload, _ := readAll(resp)
		decoded := map[string]any{}
		_ = json.Unmarshal(payload, &decoded)
		if out != nil && resp.StatusCode == http.StatusOK {
			if err := json.Unmarshal(payload, out); err != nil {
				cur.Fatal(err)
			}
		}
		return resp.StatusCode, decoded
	}
	ok := func(method, path string, body any, out any) {
		cur.Helper()
		if code, payload := request(method, path, body, out); code != http.StatusOK {
			cur.Fatalf("%s %s: %d %v", method, path, code, payload)
		}
	}
	status := func() denseexec.StatusBody {
		cur.Helper()
		var s denseexec.StatusBody
		ok(http.MethodGet, "dense-execution/status", nil, &s)
		return s
	}
	observe := func() control.Status {
		cur.Helper()
		s, err := c.Status(ctx)
		if err != nil {
			cur.Fatal(err)
		}
		return s
	}
	joints := func() []float64 {
		s := observe()
		q := make([]float64, 6)
		for i := range q {
			q[i] = s.Axes[i].LogicalPosition
		}
		return q
	}
	nativeState := func() string {
		s := observe()
		axes := make([]string, 0, len(s.Axes))
		for i, a := range s.Axes {
			axes = append(axes, fmt.Sprintf("J%d %.4f deg %s/%s %s %s", i+1, a.LogicalPosition*180/math.Pi, a.Readiness, a.PositionState, a.PositionReason, a.DriveAlarm))
		}
		return fmt.Sprintf("execution %+v; core armed %d safety_fault_mask %d execution_fault_reasons %d home_valid %d; axes %s",
			s.Execution, s.Core.Armed, s.Core.SafetyFaultMask, s.Core.ExecutionFaultReasons, s.Core.HomeValidMask, strings.Join(axes, "; "))
	}
	stateDetail = func() string {
		s, err := c.Status(ctx)
		if err != nil {
			return err.Error()
		}
		axes := make([]string, 0, len(s.Axes))
		for i, a := range s.Axes {
			axes = append(axes, fmt.Sprintf("J%d %s/%s %s", i+1, a.Readiness, a.PositionState, a.PositionReason))
		}
		return fmt.Sprintf("armed %d safety_fault_mask %d execution_fault_reasons %d home_valid %d; %s", s.Core.Armed, s.Core.SafetyFaultMask, s.Core.ExecutionFaultReasons, s.Core.HomeValidMask, strings.Join(axes, "; "))
	}
	await := func(what string, limit time.Duration, predicate func() bool) {
		cur.Helper()
		waitFor(cur, what, limit, predicate)
	}
	run := func(name string, body func(t *testing.T)) bool {
		return t.Run(name, func(sub *testing.T) {
			cur = sub
			defer func() { cur = t }()
			body(sub)
		})
	}

	// The Jog speed of every Move-to, and the Deck's Plan speed (program
	// speed scale, 1-100 %). Unverified defaults 0.2 and 100 %: a loaded
	// container host faults the simulator's fastest joint on
	// coordinate_reference (scratchpad 2026-10-09); lower them there.
	moveFraction, planSpeed := 0.2, "100"
	if raw := os.Getenv("WELD_PLAN_MOVE_FRACTION"); raw != "" {
		if _, err := fmt.Sscan(raw, &moveFraction); err != nil {
			t.Fatal(err)
		}
	}
	if raw := os.Getenv("WELD_PLAN_SPEED_PERCENT"); raw != "" {
		planSpeed = raw
	}
	report.add("move-to fraction / plan speed", "%.2f / %s %%", moveFraction, planSpeed)

	// The pendant: olp-core under Node, one step per call, the program kept
	// as the OLP project export the Deck autosaves.
	ui := filepath.Join(repo, "offline-programming", "v1", "ui")
	statePath := filepath.Join(build, "current.olp.json")
	// The packed request is kept beside the logs when they are kept, so the
	// planner can be timed again alone (POST it to /api/motion/plan).
	exportPath := filepath.Join(build, "program.weldplan")
	if keep := os.Getenv("WELD_PLAN_LOGS"); keep != "" {
		_ = os.MkdirAll(keep, 0o755)
		exportPath = filepath.Join(keep, "program.weldplan")
	}
	deck := func(command string, stdin []byte, args ...string) map[string]any {
		cur.Helper()
		argv := append(helper("node"), append([]string{"--experimental-strip-types", "--no-warnings", "test/pendant/deck-teach.mjs", command}, args...)...)
		cmd := exec.CommandContext(ctx, argv[0], argv[1:]...)
		cmd.Dir = ui
		cmd.Env = append(os.Environ(), "OLP="+server.URL, "STATE="+statePath, "CLIENT_ID="+clientID, "BLOB_OUT="+filepath.Join(build, "plan.rdt"), "PLAN_SPEED="+planSpeed, "EXPORT_OUT="+exportPath)
		cmd.Stdin = bytes.NewReader(stdin)
		var stderr bytes.Buffer
		cmd.Stderr = &stderr
		out, err := cmd.Output()
		lines := strings.Split(strings.TrimSpace(string(out)), "\n")
		var outcome map[string]any
		if jsonErr := json.Unmarshal([]byte(lines[len(lines)-1]), &outcome); jsonErr != nil {
			cur.Fatalf("deck %s: %v %v\nstdout %s\nstderr %s", command, err, jsonErr, out, stderr.String())
		}
		return outcome
	}
	type refusal struct {
		State, Code, Detail string
		HTTP                int
		Elapsed             time.Duration
	}
	var refusals []refusal
	refuse := func(state string, code int, payload map[string]any, elapsed time.Duration) {
		refusals = append(refusals, refusal{State: state, HTTP: code, Code: fmt.Sprint(payload["error"]), Detail: fmt.Sprint(payload["detail"]), Elapsed: elapsed})
	}

	ok(http.MethodPost, "targets/select", targets.Selection{CellID: "sim", ModelID: "rosie_1420_v1"}, nil)

	// The Deck's heartbeat, every second while the server holds a session.
	heartbeatCtx, stopHeartbeat := context.WithCancel(ctx)
	heartbeatDone := make(chan struct{})
	go func() {
		defer close(heartbeatDone)
		ticker := time.NewTicker(1000 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-heartbeatCtx.Done():
				return
			case <-ticker.C:
			}
			if s := p.Status(heartbeatCtx); s.Session != nil && s.Session.State != "stopped" {
				raw, _ := json.Marshal(map[string]string{"session_id": s.Session.ID})
				req, err := http.NewRequestWithContext(heartbeatCtx, http.MethodPost, server.URL+"/api/offline-programming/v1/dense-execution/heartbeat", bytes.NewReader(raw))
				if err != nil {
					continue
				}
				req.Header.Set(clientIDHeader, clientID)
				if resp, err := server.Client().Do(req); err == nil {
					resp.Body.Close()
				}
			}
		}
	}()
	t.Cleanup(func() { stopHeartbeat(); <-heartbeatDone })

	home := func() {
		cur.Helper()
		if code, payload := request(http.MethodPost, "dense-execution/home", nil, nil); code != http.StatusOK {
			cur.Fatalf("Home: %d %v", code, payload)
		}
		await("Home on every axis", 120*time.Second, func() bool {
			s := observe()
			return s.Core.HomeValidMask&axisMask == axisMask && s.Core.CommissioningPhase == 0
		})
		// Arm needs every coordinate trusted; a Home whose qualification
		// failed shows here (recovering/invalid_anchor), not at Arm.
		await("every axis trusted after Home", 30*time.Second, func() bool {
			s := observe()
			for _, a := range s.Axes {
				if a.PositionState != "trusted" || a.Readiness == "faulted" {
					return false
				}
			}
			return s.Core.SafetyFaultMask == 0
		})
	}
	home()
	homed := joints()
	arm := func() {
		cur.Helper()
		ok(http.MethodPost, "dense-execution/arm", map[string]bool{"armed": true}, nil)
		await("armed", 30*time.Second, func() bool { return observe().Core.Armed == 1 })
	}
	stop := func() {
		cur.Helper()
		var stopped denseexec.StopResult
		ok(http.MethodPost, "dense-execution/stop", nil, &stopped)
		if !stopped.Stopped {
			cur.Fatalf("stop: %+v", stopped)
		}
		await("native disarm", 10*time.Second, func() bool {
			s := observe()
			return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Execution.State != "executing"
		})
	}
	// moveTo is the pendant's Move-to (Go to plan start): every joint to an
	// absolute position in one synchronised move, done on native completion.
	moveTo := func(q []float64) {
		cur.Helper()
		body := status()
		if body.Session == nil {
			cur.Fatalf("Move-to without a session: %s", nativeState())
		}
		var jog denseexec.JointJogState
		ok(http.MethodGet, "dense-execution/jog/state", nil, &jog)
		axes := make([]denseexec.JointMoveAxis, 6)
		for i := range axes {
			position := q[i]
			axes[i] = denseexec.JointMoveAxis{Axis: i, PositionRad: &position}
		}
		if code, payload := request(http.MethodPost, "dense-execution/move", denseexec.JointMoveIntent{TargetID: "bench", SessionID: body.Session.ID, Revision: jog.Revision, Axis: 0, Fraction: moveFraction, Axes: axes}, nil); code != http.StatusOK {
			cur.Fatalf("Move-to %s: %d %v; %s", degrees(q), code, payload, nativeState())
		}
		await("Move-to done", 120*time.Second, func() bool {
			var state denseexec.JointJogState
			ok(http.MethodGet, "dense-execution/jog/state", nil, &state)
			if state.Move.State != "done" {
				return false
			}
			now := joints()
			for i := range now {
				if math.Abs(now[i]-q[i]) > tolerance[i] {
					return false
				}
			}
			return true
		})
	}
	offset := func(base []float64, deg ...float64) []float64 {
		q := append([]float64(nil), base...)
		for i, v := range deg {
			q[i] += v * math.Pi / 180
		}
		return q
	}
	// P1 where Home left the arm, then reachable poses a few tens of degrees
	// away, every joint well inside its described range. The robot plans
	// from the last one, so the program runs last -> P1 -> ... -> last.
	// WELD_PLAN_WAYPOINTS=2 keeps the first two (one Move-to before the plan).
	poses := [][]float64{
		homed,
		offset(homed, 15, 10, -10, 0, 0, 0),
		offset(homed, -10, 5, 0, 20, 15, -20),
	}
	if os.Getenv("WELD_PLAN_WAYPOINTS") == "2" {
		poses = poses[:2]
	}
	report.add("homed joints (deg)", "%s", degrees(homed))

	// WELD_PLAN_HOME=1: the Deck's program from the 1430 cell at 15:45 2026-10-11, a
	// HOME before the waypoints (added to the empty program, as the Deck
	// does), with a HOME appended after them too. Every motion node plans
	// from the measured pose; the trailing HOME ends the run at the
	// description's reset pose (the 1430 cell: every joint 0).
	withHome := os.Getenv("WELD_PLAN_HOME") == "1"
	addHome := func(t *testing.T, where string) {
		if outcome := deck("home", nil); outcome["ok"] != true {
			t.Fatalf("home %s: %v", where, outcome)
		} else {
			report.add("program after home "+where, "%v", outcome["ops"])
		}
	}
	if withHome && !run("add_home_first", func(t *testing.T) { addHome(t, "first") }) {
		return
	}
	var taught [][]float64
	if !run("teach_waypoints", func(t *testing.T) {
		arm()
		for i, q := range poses {
			if !within(joints(), q, tolerance) {
				moveTo(q)
			}
			began := time.Now()
			outcome := deck("record", nil, "joint")
			if outcome["ok"] != true {
				t.Fatalf("record P%d: %v", i+1, outcome)
			}
			got := floats(outcome["joints"])
			for j := 0; j < 6; j++ {
				if math.Abs(got[j]-q[j]) > tolerance[j] {
					t.Fatalf("P%d J%d recorded %.6f rad, held %.6f", i+1, j+1, got[j], q[j])
				}
			}
			taught = append(taught, got[:6])
			report.add(fmt.Sprintf("record P%d", i+1), "%.2f s, tcp %v, machine-bound %v", time.Since(began).Seconds(), outcome["tcp"], outcome["machineBound"])
		}
	}) {
		return
	}
	start := poses[len(poses)-1]
	if withHome && !run("add_home_last", func(t *testing.T) { addHome(t, "last") }) {
		return
	}

	// planOutcome posts the Deck's plan and records the planner's own times.
	plans := 0
	planOutcome := func(label string) map[string]any {
		cur.Helper()
		began := time.Now()
		outcome := deck("plan", nil)
		plans++
		served := plannerServed(plannerLog, plans)
		report.add("plan "+label, "%.1f s wall (planner: %s), ok %v, %v", time.Since(began).Seconds(), served, outcome["ok"], summary(outcome))
		return outcome
	}
	const armedRefusal = "measured planning start requires fresh stationary verified feedback for every described joint; reconnect or establish Home before planning"
	// Up to 2b56c673 a plan while armed is refused by measuredStartObservation
	// (weld_plan_start.go) unless stopped and disarmed; the Deck teach-workflow
	// fixes accept a finished jog while armed. Either is recorded; nothing else
	// passes.
	armedPlan := func(state string) map[string]any {
		cur.Helper()
		outcome := planOutcome(state)
		if outcome["ok"] == true {
			report.add(state, "ACCEPTED (finished jog while armed is admitted)")
			return outcome
		}
		message := fmt.Sprint(outcome["message"])
		if !strings.Contains(message, armedRefusal) {
			cur.Fatalf("%s: neither a plan nor the known armed refusal: %v", state, outcome)
		}
		report.add(state, "REFUSED: %q", message)
		return nil
	}
	var planned map[string]any
	run("plan_armed_after_move_to", func(t *testing.T) {
		planned = armedPlan("plan armed after Move-to")
	})
	// Run after the main chain: on a loaded host every extra motion is a
	// chance for the simulator to fault before the plan is made.
	heldJogPlan := func(t *testing.T) {
		if observe().Core.Armed == 0 {
			arm()
		}
		var jog denseexec.JointJogState
		ok(http.MethodGet, "dense-execution/jog/state", nil, &jog)
		intent := denseexec.JointJogIntent{TargetID: "bench", SessionID: status().Session.ID, Revision: jog.Revision, Axis: 5, Direction: 1, Fraction: 0.05}
		ok(http.MethodPost, "dense-execution/jog/begin", intent, nil)
		cadence := time.Duration(max(jog.CadenceMS, 50)) * time.Millisecond
		for held := time.Now(); time.Since(held) < 400*time.Millisecond; time.Sleep(cadence) {
			ok(http.MethodPost, "dense-execution/jog/update", intent, nil)
		}
		ok(http.MethodPost, "dense-execution/jog/end", intent, nil)
		await("jog ended", 10*time.Second, func() bool {
			var state denseexec.JointJogState
			ok(http.MethodGet, "dense-execution/jog/state", nil, &state)
			return !state.Active
		})
		if moved := math.Abs(joints()[5] - start[5]); moved < tolerance[5] {
			t.Fatalf("the held jog did not move J6 (%.6f rad)", moved)
		}
		// A plan from the jogged pose is not the one the later Loads name:
		// keep the Deck's copy of that one.
		kept, err := os.ReadFile(filepath.Join(build, "plan.rdt"))
		if err != nil {
			t.Fatal(err)
		}
		armedPlan("plan armed after held jog")
		if err := os.WriteFile(filepath.Join(build, "plan.rdt"), kept, 0o644); err != nil {
			t.Fatal(err)
		}
		moveTo(start)
	}

	var identity map[string]any
	var blob []byte
	if !run("plan_after_stop", func(t *testing.T) {
		stop()
		began := time.Now()
		export := deck("export", nil)
		report.add("pack only (/weld-plan/export)", "%.2f s, ok %v %v", time.Since(began).Seconds(), export["ok"], summary(export))
		outcome := planOutcome("after STOP (cold unless an armed plan succeeded)")
		if outcome["ok"] != true {
			t.Fatalf("plan after STOP: %v", outcome)
		}
		planned = outcome
		identity = planned["load"].(map[string]any)
		digest := fmt.Sprint(planned["digest"])
		fetched := time.Now()
		resp, err := http.Get(origin + "/api/motion/dense/" + digest)
		if err != nil {
			t.Fatal(err)
		}
		blob, _ = io.ReadAll(resp.Body)
		resp.Body.Close()
		report.add("blob fetch (GET /api/motion/dense)", "%.3f s, %d bytes, HTTP %d", time.Since(fetched).Seconds(), len(blob), resp.StatusCode)
		kept, err := os.ReadFile(filepath.Join(build, "plan.rdt"))
		if err != nil || !bytes.Equal(kept, blob) {
			t.Fatalf("the Deck's kept bytes differ from the planner's blob (%d vs %d bytes): %v", len(kept), len(blob), err)
		}
		// The trajectory, as planned: it starts at the measured start and each
		// taught move's segment ends on its waypoint, in absolute units.
		plan, err := densejointtraj.Decode(blob)
		if err != nil {
			t.Fatal(err)
		}
		first := plan.Segments[0].Samples[0]
		for j := 0; j < 6; j++ {
			if math.Abs(first.QRad[j]-start[j]) > tolerance[j] {
				t.Fatalf("trajectory starts J%d at %.6f rad, the measured start is %.6f", j+1, first.QRad[j], start[j])
			}
		}
		var kinds []string
		reached := 0
		// Every sample inside the machine's described position range (the 1430 cell
		// J5 upper 0.5934 rad from 1a4f24ac), in radians.
		margin := make([]float64, 6)
		for j := range margin {
			margin[j] = math.Inf(1)
		}
		for _, segment := range plan.Segments {
			kinds = append(kinds, segment.Kind+":"+segment.SourceID)
			last := segment.Samples[len(segment.Samples)-1]
			if reached < len(taught) && within(last.QRad[:6], taught[reached], tolerance) {
				reached++
			}
			for _, sample := range segment.Samples {
				for j := 0; j < 6; j++ {
					margin[j] = math.Min(margin[j], math.Min(sample.QRad[j]-d.Axes[j].MinPosition, d.Axes[j].MaxPosition-sample.QRad[j]))
				}
			}
		}
		report.add("closest approach to a position limit (rad)", "%.4f", margin)
		for j, m := range margin {
			if m < 0 {
				t.Errorf("J%d leaves its described range by %.6f rad", j+1, -m)
			}
		}
		if withHome {
			home := plan.Segments[0]
			end := plan.Segments[len(plan.Segments)-1]
			if !within(home.Samples[len(home.Samples)-1].QRad[:6], make([]float64, 6), tolerance) ||
				!within(end.Samples[len(end.Samples)-1].QRad[:6], make([]float64, 6), tolerance) {
				t.Fatalf("the HOME segments do not start the plan at the measured pose and end at the reset pose: %s", strings.Join(kinds, ", "))
			}
		}
		report.add("trajectory segments", "%s; start_state %v", strings.Join(kinds, ", "), plan.Header.StartState != nil)
		if reached != len(taught) {
			t.Fatalf("the trajectory's segment ends reach %d of %d taught waypoints in order (segments %s)", reached, len(taught), strings.Join(kinds, ", "))
		}
	}) {
		return
	}
	run("plan_again_warm", func(t *testing.T) {
		if outcome := planOutcome("second (warm)"); outcome["ok"] != true {
			t.Errorf("second plan: %v", outcome)
		} else {
			identity, planned = outcome["load"].(map[string]any), outcome
		}
	})

	// From d271a756 the server takes the copy the client kept at plan time
	// (LoadRequest.dense_blob_base64); the Deck sends it only on a retry
	// after dense_blob_not_found (run_page.cpp). Before, Load fetched the
	// digest from the planner only.
	keptCopy := false
	loadRequest := reflect.TypeOf(denseexec.LoadRequest{})
	for i := 0; i < loadRequest.NumField(); i++ {
		keptCopy = keptCopy || strings.HasPrefix(loadRequest.Field(i).Tag.Get("json"), "dense_blob_base64")
	}
	report.add("server takes the kept copy", "%v", keptCopy)
	load := func(state string) (string, bool) {
		cur.Helper()
		body := map[string]any{}
		for key, value := range identity {
			body[key] = value
		}
		began := time.Now()
		var loaded denseexec.LoadResult
		code, payload := request(http.MethodPost, "dense-execution/load", body, &loaded)
		if code != http.StatusOK && keptCopy && payload["error"] == "dense_blob_not_found" {
			kept, err := os.ReadFile(filepath.Join(build, "plan.rdt"))
			if err != nil {
				cur.Fatal(err)
			}
			body["dense_blob_base64"] = base64.StdEncoding.EncodeToString(kept)
			report.add(state+": load", "dense_blob_not_found after %.2f s; retrying with the kept copy", time.Since(began).Seconds())
			code, payload = request(http.MethodPost, "dense-execution/load", body, &loaded)
		}
		elapsed := time.Since(began)
		if code != http.StatusOK {
			refuse(state+": load", code, payload, elapsed)
			return "", false
		}
		report.add(state+": load", "%.2f s, session %s, notes %q", elapsed.Seconds(), loaded.SessionID, loaded.Notes)
		return loaded.SessionID, true
	}
	play := func(state, session string) bool {
		cur.Helper()
		began := time.Now()
		code, payload := request(http.MethodPost, "dense-execution/play", map[string]string{"session_id": session}, nil)
		if code != http.StatusOK {
			refuse(state+": play", code, payload, time.Since(began))
			return false
		}
		return true
	}

	if !run("load_arm_play_to_completion", func(t *testing.T) {
		arm()
		session, loaded := load("dry run")
		if !loaded {
			t.Fatalf("load refused: %+v", refusals[len(refusals)-1])
		}
		began := time.Now()
		if !play("dry run", session) {
			t.Fatalf("play refused: %+v", refusals[len(refusals)-1])
		}
		// Closest observed approach to each waypoint, in completion tolerances.
		closest := make([]float64, len(taught))
		for k := range closest {
			closest[k] = math.Inf(1)
		}
		await("program completion", 10*time.Minute, func() bool {
			q := joints()
			for k, w := range taught {
				worst := 0.0
				for j := range w {
					worst = math.Max(worst, math.Abs(q[j]-w[j])/tolerance[j])
				}
				closest[k] = math.Min(closest[k], worst)
			}
			body := status()
			if body.Session == nil || body.Session.State == "stopped" {
				t.Fatalf("session ended without completing: %+v; %s", body.Session, nativeState())
			}
			return body.Session.State == "done"
		})
		final := observe()
		if final.Execution.State != "completed" || final.Execution.Identity.SourceDigest != fmt.Sprint(planned["digest"]) {
			t.Fatalf("native execution: %+v (planned %v)", final.Execution, planned["digest"])
		}
		report.add("play to completion", "%.1f s (plan %v s); closest approach to each waypoint in completion tolerances %.2f",
			time.Since(began).Seconds(), planned["dense"].(map[string]any)["durationS"], closest)
		for k, c := range closest {
			if c > 1 {
				t.Errorf("the executed path passed P%d at best %.2f x its completion tolerance", k+1, c)
			}
		}
		last := taught[len(taught)-1]
		if withHome {
			last = make([]float64, 6)
		}
		if !within(joints(), last, tolerance) {
			t.Errorf("ended %s, the program ends at %s", degrees(joints()), degrees(last))
		}
	}) {
		return
	}

	run("plan_armed_after_held_jog", heldJogPlan)

	run("play_refused_off_start", func(t *testing.T) {
		moveTo(offset(start, 3, 0, 0, 0, 0, 0))
		session, loaded := load("off start")
		if loaded && play("off start", session) {
			t.Fatalf("Play from 3 deg off the plan start was accepted; %s", nativeState())
		}
		moveTo(start)
	})

	run("play_while_disarmed", func(t *testing.T) {
		stop()
		session, loaded := load("disarmed")
		if loaded && play("disarmed", session) {
			t.Fatalf("Play while disarmed was accepted; %s", nativeState())
		}
	})

	run("load_after_rehome", func(t *testing.T) {
		home()
		session, loaded := load("after re-Home")
		if loaded {
			arm()
			if play("after re-Home", session) {
				t.Fatalf("a plan made before the re-Home played; %s", nativeState())
			}
		}
		last := refusals[len(refusals)-1]
		if !strings.Contains(last.Code+last.Detail, "planning_start_stale") {
			t.Errorf("after re-Home: refused without planning_start_stale: %+v", last)
		}
		if outcome := planOutcome("after re-Home (stale recovery)"); outcome["ok"] != true {
			t.Errorf("replan after re-Home: %v", outcome)
		} else {
			identity = outcome["load"].(map[string]any)
		}
	})

	// The planner's store is a cache (~/.cache on a cell); Load after it lost
	// the blob is refused unless the kept copy goes along.
	run("load_after_planner_eviction", func(t *testing.T) {
		if plannerLog == "" {
			t.Skip("the planner's dense store is not this test's")
		}
		store := filepath.Join(build, "dense")
		entries, err := os.ReadDir(store)
		if err != nil {
			t.Fatal(err)
		}
		for _, entry := range entries {
			if err := os.RemoveAll(filepath.Join(store, entry.Name())); err != nil {
				t.Fatal(err)
			}
		}
		if observe().Core.Armed == 0 {
			arm()
		}
		_, loaded := load("after planner eviction")
		switch {
		case keptCopy && !loaded:
			t.Errorf("Load with the kept copy refused after eviction: %+v", refusals[len(refusals)-1])
		case !keptCopy && loaded:
			t.Errorf("Load succeeded with the planner's blob evicted and no kept copy")
		case !keptCopy && !strings.Contains(refusals[len(refusals)-1].Code+refusals[len(refusals)-1].Detail, "dense_blob_not_found"):
			t.Errorf("Load after eviction refused without dense_blob_not_found: %+v", refusals[len(refusals)-1])
		}
	})

	if len(refusals) > 0 {
		var input []map[string]string
		for _, r := range refusals {
			input = append(input, map[string]string{"code": r.Code, "detail": r.Detail})
		}
		raw, _ := json.Marshal(input)
		texts, _ := deck("reasons", raw)["texts"].([]any)
		for i, r := range refusals {
			text := ""
			if i < len(texts) {
				text = fmt.Sprint(texts[i])
			}
			report.add("refused "+r.State, "HTTP %d %s (%s) after %.2f s; Deck shows %q", r.HTTP, r.Code, r.Detail, r.Elapsed.Seconds(), text)
		}
	}
}

// weldPlanReport is the run's findings in order, logged at the end and
// written to $WELD_PLAN_REPORT when set.
type weldPlanReport struct {
	mu    sync.Mutex
	lines []string
}

func (r *weldPlanReport) add(key, format string, args ...any) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.lines = append(r.lines, key+": "+fmt.Sprintf(format, args...))
}

func (r *weldPlanReport) flush(t *testing.T) {
	r.mu.Lock()
	defer r.mu.Unlock()
	text := strings.Join(r.lines, "\n")
	t.Logf("weld-plan composed report:\n%s", text)
	if path := os.Getenv("WELD_PLAN_REPORT"); path != "" {
		_ = os.WriteFile(path, []byte(text+"\n"), 0o644)
	}
}

var plannerServedLine = regexp.MustCompile(`served in [0-9.]+ s`)

// plannerServed is the motion server's own "served in" for its nth plan, or
// "-" when the planner's log is not this test's.
func plannerServed(log string, nth int) string {
	if log == "" {
		return "-"
	}
	raw, _ := os.ReadFile(log)
	var lines []string
	for _, line := range strings.Split(string(raw), "\n") {
		if plannerServedLine.MatchString(line) {
			lines = append(lines, strings.TrimSpace(line))
		}
	}
	if nth > len(lines) {
		return "no served line"
	}
	return lines[nth-1]
}

// summary is the outcome without the Load identity it carries.
func summary(outcome map[string]any) string {
	shown := map[string]any{}
	for key, value := range outcome {
		if key != "load" {
			shown[key] = value
		}
	}
	raw, _ := json.Marshal(shown)
	return string(raw)
}

func floats(value any) []float64 {
	list, _ := value.([]any)
	out := make([]float64, len(list))
	for i, v := range list {
		out[i], _ = v.(float64)
	}
	return out
}

func within(q, target, tolerance []float64) bool {
	for j := range target {
		if math.Abs(q[j]-target[j]) > tolerance[j] {
			return false
		}
	}
	return true
}

func degrees(q []float64) string {
	parts := make([]string, len(q))
	for i, v := range q {
		parts[i] = fmt.Sprintf("%.3f", v*180/math.Pi)
	}
	return "[" + strings.Join(parts, " ") + "]"
}
