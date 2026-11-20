import assert from "node:assert/strict";
import test from "node:test";
import { deriveRunControls } from "./runControls.ts";
import { deriveRunProgram, normalizeRunResult, runConfirmation, runOutcomeText, runProgramRequest } from "./runProgram.ts";

const record = { trajectoryDigest: "sha256:t", planId: "plan" };

test("the 409 keeps its steps and the server's sentence", () => {
  const refused = normalizeRunResult({ error: "fault_persists", detail: "J5 drive alarm Er41.1 still active", run: {
    state: "refused", steps: [{ step: "confirm_stop", outcome: "done" }], refusal: { step: "reset", code: "fault_persists", detail: "J5 drive alarm Er41.1 still active — let it cool, then press Run program again" } } });
  assert.equal(runOutcomeText(refused), "Stop confirmed; then J5 drive alarm Er41.1 still active — let it cool, then press Run program again");
  // A gate's 409 (focus, cell, maintenance) has no run member: its detail is the answer.
  const gate = normalizeRunResult({ error: "control_held", detail: "Another pendant controls the robot" });
  assert.equal(runOutcomeText(gate), "Another pendant controls the robot");
});

test("the confirmation names what the press will do and sends back what it showed", () => {
  const both = normalizeRunResult({ state: "confirm", action: "Arm, move to program start and run", needs_arm: true, needs_move: true,
    joints: [{ axis: 0, id: "J1", from_deg: 0, to_deg: 12.5, delta_deg: 12.5, moves: true }, { axis: 1, id: "J2", from_deg: -3, to_deg: -3, delta_deg: 0, moves: false }] });
  const words = runConfirmation(both);
  assert.equal(words.eyebrow, "RUN PROGRAM");
  assert.equal(words.confirm, "Arm, move and run");
  assert.match(words.intro, /disarmed and not at the program's start/);
  assert.deepEqual(words.lines, ["J1  0.00° → 12.50°  Δ +12.50°", "J2  -3.00° → -3.00°  stays"]);
  const body = runProgramRequest({ trajectory_digest: "sha256:t", dry_run: true }, 0.25, both);
  assert.deepEqual(body, { trajectory_digest: "sha256:t", dry_run: true, fraction: 0.25, confirm_arm: true, confirm_move: true,
    confirmed: [{ axis: 0, id: "J1", from_deg: 0, to_deg: 12.5, moves: true }, { axis: 1, id: "J2", from_deg: -3, to_deg: -3, moves: false }] });
  const armOnly = runConfirmation(normalizeRunResult({ state: "confirm", needs_arm: true, joints: [] }));
  assert.equal(armOnly.title, "Arm and run");
  assert.doesNotMatch(armOnly.warning, /sweep/, "no move, no sweep warning");
  assert.deepEqual(runProgramRequest({}, 0.1), { fraction: 0.1, confirm_arm: false, confirm_move: false });
});

test("the one button: Run program, Run again after this plan completed, refused while running", () => {
  const base = { targetSelected: true, cellPending: false, inFlight: false, native: true, record: { trajectoryDigest: "sha256:t", planId: "plan", sourceProjectRevision: 3, totalSampleCount: 100, totalDurationS: 1, createdAt: "", sourceProgramDigest: "" } as never,
    projectRevision: 3, dryRun: true, daemonReachable: true, lastRequestError: null, planStartActive: false };
  const run = (session: Record<string, unknown> | null, extra: Partial<Parameters<typeof deriveRunProgram>[0]> = {}) => {
    const status = { backend: "rt_core", session, rtCore: { axes: [], operatorFeedback: { axes: [] } } } as never;
    const controls = deriveRunControls({ ...base, status });
    return deriveRunProgram({ controls, native: true, targetSelected: true, cellPending: false, running: false, record, session: session as never, ...extra });
  };
  assert.deepEqual([run(null).text, run(null).disabled], ["Run program", false]);
  const again = run({ id: "s", state: "done", trajectoryDigest: "sha256:t", planId: "plan" });
  assert.deepEqual([again.text, again.again, again.disabled], ["Run again", true, false]);
  assert.equal(run({ id: "s", state: "done", trajectoryDigest: "sha256:other", planId: "plan" }).text, "Run program", "another plan finished: not a replay");
  assert.match(run({ id: "s", state: "playing", trajectoryDigest: "sha256:t", planId: "plan" }).title, /running; STOP ends it/);
  assert.equal(run(null, { running: true }).text, "Running…");
  assert.equal(run(null, { native: false }).disabled, true);
});

test("a repeat asks once, shows the planned return move and sends confirm_loop back", async () => {
  const { finishControl, loopText, normalizeLoopView, repeatOf } = await import("./runProgram.ts");
  const asked = normalizeRunResult({ state: "confirm", action: "Loop until stopped", joints: [{ axis: 0, id: "J1", from_deg: 10, to_deg: 10, delta_deg: 0, moves: false }],
    loop: { mode: "loop", return_move: { duration_s: 4.25, joints: [{ axis: 0, id: "J1", from_deg: 80, to_deg: 10, delta_deg: -70, moves: true }] } } });
  const words = runConfirmation(asked);
  assert.equal(words.eyebrow, "LOOP UNTIL STOPPED");
  assert.equal(words.confirm, "Start loop");
  assert.match(words.intro, /reloading and re-checking it before each cycle by itself/);
  assert.match(words.intro, /planned, collision-checked return to the program's start \(4\.3 s/);
  assert.deepEqual(words.lines, ["J1  80.00° → 10.00°  Δ −70.00°"], "the table is the return move");
  const body = runProgramRequest({ trajectory_digest: "sha256:t" }, 0.2, asked, repeatOf("loop", 0));
  assert.equal(body.repeat, "loop");
  assert.equal(body.confirm_loop, true);
  assert.equal(body.count, undefined);
  assert.deepEqual(runProgramRequest({}, 0.2, undefined, repeatOf("count", 4)), { fraction: 0.2, confirm_arm: false, confirm_move: false, repeat: "count", count: 4 });
  assert.deepEqual(repeatOf("count", 1), { mode: "once", count: 1 }, "one cycle is one run");
  const already = runConfirmation(normalizeRunResult({ state: "confirm", needs_arm: true, joints: [], loop: { mode: "count", count: 3 } }));
  assert.equal(already.title, "Run 3 times");
  assert.match(already.intro, /already ends at its start: no return move/);

  const running = normalizeLoopView({ mode: "count", count: 5, cycles: 2, state: "running", last_cycle_s: 12.34, owner: "Chrome", attended: true, return_move: true });
  assert.equal(loopText(running), "Cycle 3 of 5 · last cycle 12.3 s");
  assert.deepEqual([finishControl(running).visible, finishControl(running).disabled], [true, false]);
  const ended = normalizeLoopView({ mode: "loop", cycles: 7, state: "ended", last_cycle_s: 9, end_reason: "stopped at once by STOP" });
  assert.equal(loopText(ended), "Loop ended after 7 cycles · last cycle 9.0 s: stopped at once by STOP");
  assert.equal(finishControl(ended).visible, false);
  assert.equal(normalizeLoopView(null), null);
  assert.match(words.intro, /this client must stay connected/);
});

test("an unattended loop says it outlives this client, and closing the page does not stop it", async () => {
  const { finishControl, normalizeLoopView, unattendedLoopRunning } = await import("./runProgram.ts");
  const asked = normalizeRunResult({ state: "confirm", action: "Loop until stopped", joints: [], loop: { mode: "loop", unattended: true } });
  const words = runConfirmation(asked);
  assert.match(words.intro, /Unattended: it keeps running if this client disconnects\. STOP from any client or the E-stop stops it at once; any client can Finish cycle/);
  assert.doesNotMatch(words.intro, /must stay connected/);
  const running = normalizeLoopView({ mode: "loop", cycles: 4, state: "running", owner: "Chrome", attended: false });
  assert.equal(unattendedLoopRunning(running), true);
  assert.doesNotMatch(finishControl(running).title, /only Chrome/);
  assert.equal(unattendedLoopRunning(normalizeLoopView({ mode: "loop", cycles: 4, state: "running", owner: "Chrome", attended: true })), false, "attended: closing the page stops it");
  assert.equal(unattendedLoopRunning(normalizeLoopView({ mode: "loop", cycles: 4, state: "ended", attended: false })), false);
  assert.equal(unattendedLoopRunning(null), false);
});
