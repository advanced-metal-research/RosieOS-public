//go:build linux

package denseexec

import (
	"context"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"math"
	"slices"
	"strings"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/densejointtraj"
)

// withClosingSegment re-encodes a trajectory with one more freespace segment,
// named sourceID, from its last sample back to its first: what the planner's
// loop-return Home (or a program that ends where it starts) produces.
func withClosingSegment(t *testing.T, blob []byte, sourceID string) ([]byte, *densejointtraj.Plan) {
	t.Helper()
	plan, err := densejointtraj.Decode(blob)
	if err != nil {
		t.Fatal(err)
	}
	h := plan.Header
	first := plan.Segments[0].Samples[0].QRad
	lastSegment := plan.Segments[len(plan.Segments)-1]
	from := lastSegment.Samples[len(lastSegment.Samples)-1].QRad
	widest := 0.0
	for a := range from {
		if step := math.Abs(first[a]-from[a]) / (0.4 * h.Limits.MaxAbsQdRadS[a] * h.Sampling.DtS); step > widest {
			widest = step
		}
	}
	n := int(math.Ceil(widest*2)) + 3
	samples := make([]densejointtraj.Sample, n)
	for s := range samples {
		u := 0.5 - 0.5*math.Cos(math.Pi*float64(s)/float64(n-1))
		samples[s].TimeS = float64(s) * h.Sampling.DtS
		for a := range from {
			samples[s].QRad[a] = from[a] + u*(first[a]-from[a])
		}
	}
	samples[n-1].QRad = first
	block := densejointtraj.EncodeBlock(samples)
	headerLen := int(binary.BigEndian.Uint32(blob[:4]))
	var header map[string]json.RawMessage
	if err := json.Unmarshal(blob[4:4+headerLen], &header); err != nil {
		t.Fatal(err)
	}
	body := append(append([]byte(nil), blob[4+headerLen:]...), block...)
	var metas []map[string]any
	if err := json.Unmarshal(header["segments"], &metas); err != nil {
		t.Fatal(err)
	}
	sum := sha256.Sum256(block)
	duration := samples[n-1].TimeS
	metas = append(metas, map[string]any{"index": len(metas), "kind": "freespace", "source_id": sourceID, "sample_count": n, "duration_s": duration,
		"block": map[string]any{"byte_offset": len(body) - len(block), "byte_length": len(block), "sha256": "sha256:" + hex.EncodeToString(sum[:])}})
	segments := append(append([]densejointtraj.Segment(nil), plan.Segments...), densejointtraj.Segment{Kind: "freespace", SourceID: sourceID, Samples: samples})
	var sampling map[string]any
	_ = json.Unmarshal(header["sampling"], &sampling)
	sampling["total_sample_count"] = h.Sampling.TotalSampleCount + n
	sampling["total_duration_s"] = h.Sampling.TotalDurationS + duration
	header["sampling"], _ = json.Marshal(sampling)
	header["segments"], _ = json.Marshal(metas)
	header["trajectory_digest"], _ = json.Marshal(densejointtraj.TrajectoryDigest(h.Axes.AxisMask, h.Sampling.DtS, h.Limits.MaxAbsQdRadS, segments))
	raw, _ := json.Marshal(header)
	out := binary.BigEndian.AppendUint32(nil, uint32(len(raw)))
	out = append(append(out, raw...), body...)
	closed, err := densejointtraj.Decode(out)
	if err != nil {
		t.Fatalf("closed trajectory does not decode: %v", err)
	}
	return out, closed
}

var loopOwner = Client{ID: "tab-owner", Label: "Chrome on macOS"}

// loopFixture: runFixture armed at the start, its plan closed by sourceID
// ("" keeps the reference plan, which ends away from its start).
func loopFixture(t *testing.T, sourceID string) (*RTCoreController, *runClient, RunProgramRequest, context.Context) {
	t.Helper()
	r, c, in, _ := runFixture(t)
	if sourceID != "" {
		blob, _ := fixtureBlob(t)
		closed, plan := withClosingSegment(t, blob, sourceID)
		id := fixtureIdentity(plan)
		r.source.cfg.FetchBlob = func(context.Context, string) ([]byte, error) { return closed, nil }
		c.mu.Lock()
		c.program.Identity.TrajectoryDigest, c.program.Identity.SourceDigest = id.TrajectoryDigest, id.TrajectoryDigest
		c.mu.Unlock()
		in.Identity = id
	}
	ctx := WithClient(context.Background(), loopOwner)
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	return r, c, in, ctx
}

// startLoopFor asks, checks the one question, confirms and returns the
// running loop's first answer.
func startLoopFor(t *testing.T, r *RTCoreController, ctx context.Context, in RunProgramRequest) *RunProgramResult {
	t.Helper()
	asked, err := r.RunProgram(ctx, in)
	if err != nil || asked.State != "confirm" || asked.Loop == nil {
		t.Fatalf("question: %+v %v", asked, err)
	}
	in.ConfirmLoop = true
	result, err := r.RunProgram(ctx, in)
	if err != nil || result.State != "playing" || result.Loop == nil || result.Loop.Running == nil {
		t.Fatalf("start: %+v %v", result, err)
	}
	return result
}

func countCalls(c *runClient, name string) int {
	c.mu.Lock()
	defer c.mu.Unlock()
	n := 0
	for _, call := range c.calls {
		if call == name {
			n++
		}
	}
	return n
}

// completeCycle waits for cycle n to be playing, then completes it natively.
func completeCycle(t *testing.T, r *RTCoreController, c *runClient, n int) {
	t.Helper()
	rtAwait(t, func() bool {
		return countCalls(c, "start_program") >= n && r.Status(context.Background()).Session.State == "playing"
	})
	c.mu.Lock()
	c.status.Execution.State = "completed"
	c.mu.Unlock()
}

func loopEnded(t *testing.T, r *RTCoreController) *LoopStatus {
	t.Helper()
	var loop *LoopStatus
	rtAwait(t, func() bool { loop = r.Status(context.Background()).Loop; return loop != nil && loop.State == "ended" })
	return loop
}

// The owner's beat, as a client heartbeating its session every second does.
func keepBeating(t *testing.T, r *RTCoreController, client Client) func() {
	done := make(chan struct{})
	go func() {
		ticker := time.NewTicker(200 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-done:
				return
			case <-ticker.C:
				if s := r.Status(context.Background()).Session; s != nil {
					_, _ = r.HeartbeatFrom(WithClient(context.Background(), client), s.ID)
				}
			}
		}
	}()
	return func() { close(done) }
}

// Run 3 times: one question (with the planned return move), then three
// cycles, each staged again and played with no operator press, and the loop
// ends by itself at the program start.
func TestRunLoopRunsNCyclesAndReloadsBetween(t *testing.T) {
	r, c, in, ctx := loopFixture(t, LoopReturnSourceID)
	defer keepBeating(t, r, loopOwner)()
	in.Repeat, in.Count = "count", 3
	asked, err := r.RunProgram(ctx, in)
	if err != nil || asked.State != "confirm" || asked.Action != "Run 3 times" || asked.Loop.ReturnMove == nil || asked.Loop.ReturnMove.DurationS <= 0 {
		t.Fatalf("question: %+v %v", asked, err)
	}
	if got := countCalls(c, "prepare_program") + countCalls(c, "start_program"); got != 0 {
		t.Fatalf("the question loaded or played (%d)", got)
	}
	moves := 0
	for _, joint := range asked.Loop.ReturnMove.Joints {
		if joint.Moves {
			moves++
		}
	}
	if moves == 0 {
		t.Fatalf("the return move shows no travel: %+v", asked.Loop.ReturnMove)
	}
	in.ConfirmLoop = true
	result, err := r.RunProgram(ctx, in)
	if err != nil || result.State != "playing" || result.Loop.Running.State != "running" || result.Loop.Running.Owner != loopOwner.Label || !result.Loop.Running.Attended {
		t.Fatalf("start: %+v %v", result, err)
	}
	for n := 1; n <= 3; n++ {
		completeCycle(t, r, c, n)
	}
	loop := loopEnded(t, r)
	if loop.Cycles != 3 || loop.EndReason != "ran 3 cycles" || loop.LastCycleS <= 0 {
		t.Fatalf("loop %+v", loop)
	}
	if p, s := countCalls(c, "prepare_program"), countCalls(c, "start_program"); p != 3 || s != 3 {
		t.Fatalf("prepare %d start %d, want 3 and 3", p, s)
	}
	if countCalls(c, "prepare_trajectory") != 0 {
		t.Fatal("a loop at its start moved between cycles")
	}
}

// A plan that ends away from its start never loops: no question, nothing
// sent, and the words say how to get the return move. A plan that already
// ends at its start loops with no return move.
func TestRunLoopClosingMove(t *testing.T) {
	r, c, in, ctx := loopFixture(t, "")
	in.Repeat = "loop"
	result, err := r.RunProgram(ctx, in)
	runRefused(t, result, err, "loop", "loop_not_closed")
	if !strings.Contains(result.Refusal.Detail, "Plan it again with the repeat selected") {
		t.Fatal(result.Refusal.Detail)
	}
	if countCalls(c, "prepare_program") != 0 {
		t.Fatal("an open plan was loaded")
	}

	r, _, in, ctx = loopFixture(t, "move-back-to-start")
	in.Repeat = "loop"
	asked, err := r.RunProgram(ctx, in)
	if err != nil || asked.State != "confirm" || asked.Action != "Loop until stopped" || asked.Loop.ReturnMove != nil {
		t.Fatalf("a program that ends at its start: %+v %v", asked, err)
	}
}

// Finish cycle: only the client that started the loop may ask; the current
// cycle completes and the loop ends at the program start, nothing more played.
func TestRunLoopFinishCycle(t *testing.T) {
	r, c, in, ctx := loopFixture(t, LoopReturnSourceID)
	defer keepBeating(t, r, loopOwner)()
	in.Repeat = "loop"
	startLoopFor(t, r, ctx, in)
	if _, err := r.FinishLoop(WithClient(context.Background(), Client{ID: "deck-3", Label: "rosie-pendant-v5 (steamdeck-pendant)"})); err == nil || !strings.Contains(err.Error(), "loop_owner_only") {
		t.Fatalf("another client finished the loop: %v", err)
	}
	status, err := r.FinishLoop(ctx)
	if err != nil || status.State != "finishing" {
		t.Fatalf("%+v %v", status, err)
	}
	completeCycle(t, r, c, 1)
	loop := loopEnded(t, r)
	if loop.Cycles != 1 || !strings.HasPrefix(loop.EndReason, "Finish cycle: ended at the program start after cycle 1") {
		t.Fatalf("loop %+v", loop)
	}
	time.Sleep(3 * rtPollInterval)
	if countCalls(c, "start_program") != 1 || countCalls(c, "prepare_program") != 1 {
		t.Fatal("Finish cycle started another cycle")
	}
}

// STOP mid-cycle stops at once (the existing Stop) and the loop never starts
// another cycle.
func TestRunLoopStopMidCycle(t *testing.T) {
	r, c, in, ctx := loopFixture(t, LoopReturnSourceID)
	defer keepBeating(t, r, loopOwner)()
	in.Repeat = "loop"
	startLoopFor(t, r, ctx, in)
	completeCycle(t, r, c, 1)
	rtAwait(t, func() bool { return countCalls(c, "start_program") == 2 })
	if _, err := r.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	loop := loopEnded(t, r)
	if loop.Cycles != 1 || loop.EndReason != "stopped at once by STOP" {
		t.Fatalf("loop %+v", loop)
	}
	time.Sleep(3 * rtPollInterval)
	if countCalls(c, "start_program") != 2 {
		t.Fatal("a stopped loop played again")
	}
}

// A reload the machine refuses ends the loop with its words, once: no retry.
// A maintenance notice ends it at the program start, before any reload.
func TestRunLoopReloadRefusalAndNoticeStop(t *testing.T) {
	r, c, in, ctx := loopFixture(t, LoopReturnSourceID)
	defer keepBeating(t, r, loopOwner)()
	in.Repeat = "loop"
	startLoopFor(t, r, ctx, in)
	c.prepareRefusal = "trajectory_limits: J2 velocity"
	completeCycle(t, r, c, 1)
	loop := loopEnded(t, r)
	if loop.Cycles != 1 || !strings.HasPrefix(loop.EndReason, "Load refused before cycle 2: ") || !strings.Contains(loop.EndReason, "trajectory_limits") {
		t.Fatalf("loop %+v", loop)
	}
	time.Sleep(3 * rtPollInterval)
	if countCalls(c, "prepare_program") != 2 || countCalls(c, "start_program") != 1 {
		t.Fatalf("the refused reload was retried or played: prepare %d start %d", countCalls(c, "prepare_program"), countCalls(c, "start_program"))
	}

	r, c, in, ctx = loopFixture(t, LoopReturnSourceID)
	defer keepBeating(t, r, loopOwner)()
	in.Repeat = "loop"
	notice := ""
	in.Continue = func() string { return notice }
	startLoopFor(t, r, ctx, in)
	notice = "Update in progress: OLP-tier release"
	completeCycle(t, r, c, 1)
	loop = loopEnded(t, r)
	if loop.EndReason != "ended at the program start after cycle 1: Update in progress: OLP-tier release" || countCalls(c, "prepare_program") != 1 {
		t.Fatalf("loop %+v prepare %d", loop, countCalls(c, "prepare_program"))
	}
}

// Attended: another client's heartbeat keeps the server's deadman fed, but
// the loop belongs to the client that started it. Its silence past the
// deadline stops the robot at once.
func TestRunLoopStopsWhenItsStarterStopsBeating(t *testing.T) {
	r, c, in, ctx := loopFixture(t, LoopReturnSourceID)
	in.Repeat = "loop"
	stopOwner := keepBeating(t, r, loopOwner)
	defer keepBeating(t, r, Client{ID: "deck-4", Label: "rosie-pendant-v5 (steamdeck-operator)"})()
	startLoopFor(t, r, ctx, in)
	stopOwner()
	started := time.Now()
	var loop *LoopStatus
	deadline := time.Now().Add(DefaultHeartbeatDeadline + 2*time.Second)
	for time.Now().Before(deadline) {
		if loop = r.Status(context.Background()).Loop; loop != nil && loop.State == "ended" {
			break
		}
		time.Sleep(20 * time.Millisecond)
	}
	if loop == nil || loop.State != "ended" || !strings.Contains(loop.EndReason, "Chrome on macOS, which started the loop, stopped sending heartbeats") {
		t.Fatalf("loop %+v", loop)
	}
	if waited := time.Since(started); waited < DefaultHeartbeatDeadline-time.Second {
		t.Fatalf("stopped after %v, before the deadline", waited)
	}
	if !slices.Contains(runCalls(c), "stop") {
		t.Fatalf("the robot was not stopped: %v", runCalls(c))
	}
	if !strings.HasPrefix(r.Status(context.Background()).Session.StopReason, "ui_heartbeat_lost") {
		t.Fatalf("stop reason %q", r.Status(context.Background()).Session.StopReason)
	}
}

// unattendedLoopFixture: loopFixture on a cell whose manifest sets
// cell_deploy.olp.unattended_loop_allowed (owner decision 2026-10-11 23:32
// PDT, the second 1430 cell/the 1430 cell).
func unattendedLoopFixture(t *testing.T) (*RTCoreController, *runClient, RunProgramRequest, context.Context) {
	t.Helper()
	r, c, in, ctx := loopFixture(t, LoopReturnSourceID)
	r.mu.Lock()
	r.cfg.UnattendedLoopAllowed = true
	r.mu.Unlock()
	in.Repeat = "loop"
	return r, c, in, ctx
}

var otherClient = Client{ID: "deck-4", Label: "rosie-pendant-v5 (steamdeck-operator)"}

// Unattended (the machine switch on, not asked for by the client): no client
// beats at all past the deadline -- the starter left and nobody else is
// connected -- and the loop keeps running, the robot never stopped; the next
// cycle starts by itself, and a client that did not start it may Finish cycle.
func TestRunLoopUnattendedOutlivesEveryHeartbeat(t *testing.T) {
	r, c, in, ctx := unattendedLoopFixture(t)
	asked, err := r.RunProgram(ctx, in)
	if err != nil || asked.State != "confirm" || asked.Action != "Loop until stopped" || !asked.Loop.Unattended {
		t.Fatalf("question: %+v %v", asked, err)
	}
	in.ConfirmLoop = true
	result, err := r.RunProgram(ctx, in)
	if err != nil || result.State != "playing" || result.Loop.Running.Attended {
		t.Fatalf("start: %+v %v", result, err)
	}
	time.Sleep(DefaultHeartbeatDeadline + 1500*time.Millisecond)
	if loop := r.Status(context.Background()).Loop; loop == nil || loop.State != "running" || slices.Contains(runCalls(c), "stop") {
		t.Fatalf("an unattended loop stopped on heartbeat silence: %+v %v", loop, runCalls(c))
	}
	completeCycle(t, r, c, 1)
	rtAwait(t, func() bool { return countCalls(c, "start_program") == 2 })
	status, err := r.FinishLoop(WithClient(context.Background(), otherClient))
	if err != nil || status.State != "finishing" {
		t.Fatalf("another client could not finish an unattended loop: %+v %v", status, err)
	}
	completeCycle(t, r, c, 2)
	loop := loopEnded(t, r)
	if loop.Cycles != 2 || !strings.HasPrefix(loop.EndReason, "Finish cycle: ended at the program start after cycle 2") || slices.Contains(runCalls(c), "stop") {
		t.Fatalf("loop %+v calls %v", loop, runCalls(c))
	}
}

// Unattended still ends at once on STOP from a client that did not start it,
// and on a fault, never retried.
func TestRunLoopUnattendedStopsOnStopFromAnyClientAndOnFault(t *testing.T) {
	r, c, in, ctx := unattendedLoopFixture(t)
	startLoopFor(t, r, ctx, in)
	completeCycle(t, r, c, 1)
	rtAwait(t, func() bool { return countCalls(c, "start_program") == 2 })
	if _, err := r.Stop(WithClient(context.Background(), otherClient)); err != nil {
		t.Fatal(err)
	}
	loop := loopEnded(t, r)
	if loop.Cycles != 1 || loop.EndReason != "stopped at once by STOP" {
		t.Fatalf("loop %+v", loop)
	}
	time.Sleep(3 * rtPollInterval)
	if countCalls(c, "start_program") != 2 {
		t.Fatal("a stopped unattended loop played again")
	}

	r, c, in, ctx = unattendedLoopFixture(t)
	startLoopFor(t, r, ctx, in)
	rtAwait(t, func() bool { return r.Status(context.Background()).Session.State == "playing" })
	c.mu.Lock()
	c.status.Core.SafetyFaultMask = 1
	c.mu.Unlock()
	loop = loopEnded(t, r)
	if loop.Cycles != 0 || !strings.Contains(loop.EndReason, "rt_core_fault") || !slices.Contains(runCalls(c), "stop") {
		t.Fatalf("loop %+v calls %v", loop, runCalls(c))
	}
	time.Sleep(3 * rtPollInterval)
	if countCalls(c, "start_program") != 1 || countCalls(c, "prepare_program") != 1 {
		t.Fatal("a faulted unattended loop was retried")
	}
}

// Unattended is refused unless the machine allows it (owner decision).
func TestRunLoopUnattendedNeedsTheMachineSwitch(t *testing.T) {
	r, _, in, ctx := loopFixture(t, LoopReturnSourceID)
	in.Repeat, in.Unattended = "loop", true
	result, err := r.RunProgram(ctx, in)
	runRefused(t, result, err, "start", "unattended_loop_not_allowed")
}
