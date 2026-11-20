import type { RecoverConfirmation, RecoverJoint, RecoverRefusal, RecoverStep } from "./recoverToZero.ts";
import type { RunControls } from "./runControls.ts";
import type { PressRefusal } from "./pressNotice.ts";

/** Run program is the one press for a planned program (owner 2026-10-11:
 *  "why do I need to load the plan to the robot if it's already on it?",
 *  "after it's run it makes me click Load to robot again"). The OLP server
 *  runs it (POST .../run-program, `internal/denseexec/run_program_plan.go`):
 *  Load's checks first, one confirmation when the robot must arm or move to
 *  the program's start, then Arm, Go to plan start, Load (unless the plan is
 *  staged) and Play, stopping at the first step that does not clear with the
 *  server's sentence. STOP ends it at any point. After the program completes
 *  the same press is Run again. This file is the request, the words and the
 *  confirmation, shared by the browser and the Deck (olp-core). The separate
 *  Go to plan start, Load to robot and Play stay under Advanced. */
export const RUN_PROGRAM_PATH = "/run-program";
export const RUN_FINISH_PATH = "/run-program/finish";

/** Run once, Run N times, Loop until stopped (owner 2026-10-11: "what if we
 *  want the robot to run the program in a continuous loop?"). A repeat is
 *  planned with the planner's return to the start (rails/planRequest.ts
 *  withLoopReturn) and asked once before its first cycle; the OLP server
 *  then reloads, re-checks and plays every cycle by itself
 *  (`internal/denseexec/run_loop.go`). */
export type RepeatMode = "once" | "count" | "loop";
export interface RunRepeat { mode: RepeatMode; count: number }
/** Unverified bound on Run N times (denseexec maxLoopCount). */
export const MAX_LOOP_COUNT = 10000;

export interface ReturnMoveView { durationS: number; joints: RecoverJoint[] }
/** unattended: the machine allows unattended loops, so this one keeps running without its starter. */
export interface LoopPlanView { mode: string; count: number; returnMove: ReturnMoveView | null; unattended: boolean }
/** A running or ended loop, as every status carries it. */
export interface LoopView {
  mode: string; count: number; cycles: number; state: "running" | "finishing" | "ended";
  cycleElapsedS: number; lastCycleS: number; owner: string; attended: boolean; returnMove: boolean; endReason: string; trajectoryDigest: string;
}

export type RunState = "playing" | "confirm" | "refused";
export interface RunResult {
  state: RunState; action: string; needsArm: boolean; needsMove: boolean; steps: RecoverStep[]; joints: RecoverJoint[];
  refusal: RecoverRefusal | null; sessionId: string; notes: string[];
  /** The repeat this press asks about or started; null for one run. */
  loop: LoopPlanView | null;
  /** The refusal's code (the 409's `error`), "" otherwise. */
  code?: string;
}

const isRecord = (v: unknown): v is Record<string, unknown> => typeof v === "object" && v !== null && !Array.isArray(v);
const num = (v: unknown): number => (typeof v === "number" && Number.isFinite(v) ? v : NaN);
const str = (v: unknown): string => (typeof v === "string" ? v : "");

/** The server's answer, or the `run` member of its 409. */
export function normalizeRunResult(raw: unknown): RunResult {
  const r = isRecord(raw) && isRecord(raw.run) ? raw.run : isRecord(raw) ? raw : {};
  const state = (["playing", "confirm", "refused"] as const).find(s => s === r.state) ?? "refused";
  const joints = Array.isArray(r.joints) ? r.joints.filter(isRecord).map(j => ({
    axis: num(j.axis), id: str(j.id), from_deg: num(j.from_deg), to_deg: num(j.to_deg), delta_deg: num(j.delta_deg), moves: j.moves === true,
  })) : [];
  const steps = Array.isArray(r.steps) ? r.steps.filter(isRecord).map(s => ({ step: str(s.step), outcome: str(s.outcome), detail: str(s.detail) })) : [];
  const refusal = isRecord(r.refusal) ? { step: str(r.refusal.step), code: str(r.refusal.code), detail: str(r.refusal.detail) }
    : state === "refused" && isRecord(raw) && typeof raw.detail === "string" ? { step: "", code: str(raw.error), detail: raw.detail } : null;
  const notes = Array.isArray(r.notes) ? r.notes.filter((n): n is string => typeof n === "string" && n !== "") : [];
  return { state, action: str(r.action), needsArm: r.needs_arm === true, needsMove: r.needs_move === true, steps, joints, refusal, sessionId: str(r.session_id), notes,
    loop: normalizeLoopPlan(r.loop) };
}

const jointsOf = (raw: unknown): RecoverJoint[] => Array.isArray(raw) ? raw.filter(isRecord).map(j => ({
  axis: num(j.axis), id: str(j.id), from_deg: num(j.from_deg), to_deg: num(j.to_deg), delta_deg: num(j.delta_deg), moves: j.moves === true,
})) : [];

function normalizeLoopPlan(raw: unknown): LoopPlanView | null {
  if (!isRecord(raw)) return null;
  const move = isRecord(raw.return_move) ? { durationS: num(raw.return_move.duration_s), joints: jointsOf(raw.return_move.joints) } : null;
  return { mode: str(raw.mode), count: Number.isFinite(num(raw.count)) ? num(raw.count) : 0, returnMove: move, unattended: raw.unattended === true };
}

/** A status body's `loop`, or null. */
export function normalizeLoopView(raw: unknown): LoopView | null {
  if (!isRecord(raw) || (raw.mode !== "loop" && raw.mode !== "count")) return null;
  const state = (["running", "finishing", "ended"] as const).find(s => s === raw.state) ?? "ended";
  const n = (v: unknown) => (typeof v === "number" && Number.isFinite(v) ? v : 0);
  return { mode: raw.mode, count: n(raw.count), cycles: n(raw.cycles), state, cycleElapsedS: n(raw.cycle_elapsed_s), lastCycleS: n(raw.last_cycle_s),
    owner: str(raw.owner), attended: raw.attended === true, returnMove: raw.return_move === true, endReason: str(raw.end_reason), trajectoryDigest: str(raw.trajectory_digest) };
}

/** The counter and last cycle time under the button, or "". */
export function loopText(loop: LoopView | null | undefined): string {
  if (!loop) return "";
  const last = loop.lastCycleS > 0 ? ` · last cycle ${loop.lastCycleS.toFixed(1)} s` : "";
  const of = loop.mode === "count" ? ` of ${loop.count}` : "";
  switch (loop.state) {
    case "running": return `Cycle ${loop.cycles + 1}${of}${last}`;
    case "finishing": return `Finishing cycle ${loop.cycles + 1}${of}: ends at the program start${last}`;
    case "ended": return `Loop ended after ${loop.cycles} cycle${loop.cycles === 1 ? "" : "s"}${last}: ${loop.endReason}`;
  }
}

/** An unattended loop is running (machine unattended_loop_allowed, owner
 *  decision 2026-10-11 23:32 PDT): it outlives this page, so closing the page
 *  must not send STOP. STOP pressed here still stops it. */
export function unattendedLoopRunning(loop: LoopView | null | undefined): boolean {
  return (loop?.state === "running" || loop?.state === "finishing") && loop.attended === false;
}

/** Finish cycle: offered while a loop runs and has not been asked to finish. */
export function finishControl(loop: LoopView | null | undefined): { visible: boolean; disabled: boolean; title: string } {
  const running = loop?.state === "running" || loop?.state === "finishing";
  return {
    visible: running, disabled: loop?.state !== "running",
    title: loop?.state === "finishing" ? "Finishing: the loop ends at the program start after this cycle"
      : `End the loop cleanly at the program start after this cycle${loop?.attended ? ` (only ${loop.owner}, which started it)` : ""}; STOP stops at once`,
  };
}

/** The repeat the operator chose, validated (a count outside 2..MAX is one run). */
export function repeatOf(mode: string, count: number): RunRepeat {
  if (mode === "loop") return { mode: "loop", count: 0 };
  if (mode === "count" && Number.isInteger(count) && count >= 2 && count <= MAX_LOOP_COUNT) return { mode: "count", count };
  return { mode: "once", count: 1 };
}

/** The request: the Load body for the kept plan and the Jog speed; the
 *  confirmation sends back the flags it asked for and the joints the operator
 *  saw, which the server checks against a fresh status. */
export function runProgramRequest(load: Record<string, unknown>, fraction: number, confirmed?: RunResult, repeat?: RunRepeat): Record<string, unknown> {
  const body: Record<string, unknown> = { ...load, fraction, confirm_arm: false, confirm_move: false };
  if (repeat && repeat.mode !== "once") {
    body.repeat = repeat.mode;
    if (repeat.mode === "count") body.count = repeat.count;
  }
  if (confirmed) {
    body.confirm_arm = confirmed.needsArm;
    body.confirm_move = confirmed.needsMove;
    if (confirmed.loop) body.confirm_loop = true;
    body.confirmed = confirmed.joints.map(j => ({ axis: j.axis, id: j.id, from_deg: j.from_deg, to_deg: j.to_deg, moves: j.moves }));
  }
  return body;
}

/** The program this machine last finished is this plan: the button reads
 *  Run again. */
export function runAgain(controls: Pick<RunControls, "state" | "current">, session: { state?: string; trajectoryDigest?: string; planId?: string } | null | undefined,
  record: { trajectoryDigest: string; planId: string } | null | undefined): boolean {
  return controls.current && !!record && session?.state === "done" && session.trajectoryDigest === record.trajectoryDigest && session.planId === record.planId;
}

export interface RunProgramControl { text: string; disabled: boolean; title: string; again: boolean }

/** The one button, for both apps: its words and why it cannot be pressed.
 *  The server checks everything again; this keeps the button from inviting a
 *  press that is certain to be refused. */
export function deriveRunProgram(input: {
  controls: RunControls; native: boolean; targetSelected: boolean; cellPending: boolean; running: boolean;
  record: { trajectoryDigest: string; planId: string } | null | undefined;
  session: { state?: string; trajectoryDigest?: string; planId?: string } | null | undefined;
  loop?: LoopView | null;
  repeat?: RunRepeat;
}): RunProgramControl {
  const { controls } = input;
  const again = runAgain(controls, input.session, input.record);
  const looping = input.loop?.state === "running" || input.loop?.state === "finishing";
  const repeat = input.repeat ?? { mode: "once", count: 1 };
  const verb = repeat.mode === "loop" ? "Loop" : repeat.mode === "count" ? `Run ${repeat.count}×` : again ? "Run again" : "Run program";
  const text = input.running ? "Running…" : looping ? "Looping…" : verb;
  const why = !input.targetSelected || input.cellPending ? "Connect a machine first"
    : !input.native ? "Run program needs an rt-core cell; use Load to robot and Play under Advanced"
    : !input.record ? "Plan the program first"
    : !controls.current ? "The program changed since this was planned; plan it again"
    : looping ? "A loop is running: Finish cycle ends it at the program start, STOP at once"
    : controls.live ? "The program is running; STOP ends it"
    : controls.loadBlocker ? controls.loadBlocker
    : input.running ? "Run program is in progress; STOP ends it"
    : controls.load.disabled ? controls.load.title
    : "";
  return {
    text, again, disabled: why !== "",
    title: why || "One press: if the robot is not at the program's start it asks, then moves there; loads the plan; arms if needed (asks first); plays. STOP ends it at any point.",
  };
}

const deg = (v: number) => `${v.toFixed(2)}°`;
const signed = (v: number) => `${v > 0 ? "+" : v < 0 ? "−" : "±"}${Math.abs(v).toFixed(2)}°`;

const STEP_WORDS: Record<string, string> = {
  confirm_stop: "Stop confirmed", reset: "faults reset", arm: "armed", move: "moved to the program's start", load: "loaded", play: "playing",
};

/** The steps that cleared, in operator words ("armed · moved to the program's start · loaded"). */
export function runStepsText(steps: readonly RecoverStep[]): string {
  return steps.filter(s => s.outcome === "done" && STEP_WORDS[s.step]).map(s => STEP_WORDS[s.step]).join(" · ");
}

/** The confirmation before the press arms or moves: the action by name, each
 *  joint's angle, the program's first position and the travel, and what the
 *  operator must check. Same dialog as All joints to 0° (`recoverDialog.ts`
 *  in the browser, the Deck's askRecover). */
export function runConfirmation(result: RunResult): RecoverConfirmation {
  if (result.loop) return loopConfirmation(result, result.loop);
  const joints = result.joints.map(j => ({ id: j.id, from: deg(j.from_deg), to: deg(j.to_deg), delta: j.moves ? `Δ ${signed(j.delta_deg)}` : "stays", moves: j.moves }));
  const done = runStepsText(result.steps);
  const does = result.needsArm && result.needsMove
    ? "The robot is disarmed and not at the program's start. Confirming arms it, moves every joint to the program's first position at the selected Jog speed, loads the plan and runs it:"
    : result.needsMove
      ? "The robot is not at the program's start. Confirming moves every joint to the program's first position at the selected Jog speed, loads the plan and runs it:"
      : "The robot is disarmed. Confirming arms it, loads the plan and runs it from its first position:";
  return {
    eyebrow: "RUN PROGRAM",
    title: result.action || (result.needsMove ? "Move to program start and run" : "Arm and run"),
    intro: `${done ? `${done}. ` : ""}${does}`,
    joints,
    lines: joints.map(j => `${j.id}  ${j.from} → ${j.to}  ${j.delta}`),
    warning: result.needsMove
      ? "All joints move together along one straight line in joint space, not a straight tool path: the tool may sweep through the cell on the way to the start. Check the whole path is clear. STOP ends the run at any point."
      : "Motors energise and the program starts at once. Check the cell is clear. STOP ends the run at any point.",
    confirm: result.needsArm && result.needsMove ? "Arm, move and run" : result.needsMove ? "Move and run" : "Arm and run",
    cancel: "Cancel",
  };
}

const row = (j: RecoverJoint) => ({ id: j.id, from: deg(j.from_deg), to: deg(j.to_deg), delta: j.moves ? `Δ ${signed(j.delta_deg)}` : "stays", moves: j.moves });

/** The one question before a loop's first cycle: what happens first (arm,
 *  move to the start), the repeat, and the plan's planned return to its
 *  start, which every cycle ends with. Asked once, never per cycle. */
function loopConfirmation(result: RunResult, loop: LoopPlanView): RecoverConfirmation {
  const repeat = loop.mode === "count" ? `${loop.count} times` : "until stopped";
  const first = result.needsArm && result.needsMove ? "arms it, moves every joint to the program's first position, then "
    : result.needsMove ? "moves every joint to the program's first position, then " : result.needsArm ? "arms it, then " : "";
  const move = loop.returnMove;
  const returning = move
    ? `Every cycle ends with the planned, collision-checked return to the program's start (${move.durationS.toFixed(1)} s, shown below).`
    : "The program already ends at its start: no return move.";
  const shown = result.needsMove ? result.joints : move?.joints ?? result.joints;
  const joints = shown.map(row);
  const done = runStepsText(result.steps);
  return {
    eyebrow: loop.mode === "count" ? `RUN ${loop.count} TIMES` : "LOOP UNTIL STOPPED",
    title: result.action || (loop.mode === "count" ? `Run ${loop.count} times` : "Loop until stopped"),
    intro: `${done ? `${done}. ` : ""}Confirming ${first}runs the program ${repeat}, reloading and re-checking it before each cycle by itself. ${returning}` +
      `${result.needsMove ? " The table is the move to the start." : ""} ` +
      (loop.unattended
        ? "Unattended: it keeps running if this client disconnects. STOP from any client or the E-stop stops it at once; any client can Finish cycle (ends at the program start)."
        : "Finish cycle ends at the program start; STOP stops at once; this client must stay connected."),
    joints,
    lines: [...joints.map(j => `${j.id}  ${j.from} → ${j.to}  ${j.delta}`), ...(result.needsMove && move ? move.joints.map(j => `return ${j.id}  ${deg(j.from_deg)} → ${deg(j.to_deg)}`) : [])],
    warning: "The robot keeps cycling without another press. Joint-space moves may sweep the tool through the cell: check the whole path is clear and stay at STOP.",
    confirm: loop.mode === "count" ? `Run ${loop.count} times` : "Start loop",
    cancel: "Cancel",
  };
}

/** The outcome line under the button. A refusal is the server's sentence,
 *  prefixed with what cleared before it. */
export function runOutcomeText(result: RunResult): string {
  const done = runStepsText(result.steps.filter(s => s.step !== "play"));
  switch (result.state) {
    case "playing": return result.loop ? `${result.loop.mode === "count" ? `Running ${result.loop.count} cycles` : "Looping"}${done ? ` (${done})` : ""}` : done ? `Running (${done})` : "Running";
    case "confirm": return "Waiting for the confirmation";
    case "refused": return `${done ? `${done}; then ` : ""}${result.refusal?.detail || "Run program refused"}`;
  }
}

/** What the operator reads when a confirmation is declined: nothing moved. */
export const RUN_CANCELLED = "Run program cancelled: nothing was armed, moved or played";

/** The press's name for the refusal toast: Run 5 times, Loop until stopped, Run program. */
function pressName(repeat: RunRepeat | undefined, loop: LoopPlanView | null): string {
  const mode = loop?.mode || repeat?.mode;
  const count = loop?.count || repeat?.count || 0;
  return mode === "count" ? `RUN ${count} TIMES` : mode === "loop" ? "LOOP UNTIL STOPPED" : "RUN PROGRAM";
}

/** A refused Run program press, for the toast (`pressNotice.ts`): the
 *  server's sentence after what cleared, its code and step, and whether
 *  anything happened before the refusal. */
export function runRefusal(result: RunResult, repeat?: RunRepeat): PressRefusal {
  const code = result.refusal?.code || result.code || "run_refused";
  const cleared = runStepsText(result.steps.filter(s => s.step !== "play"));
  return {
    kicker: `${pressName(repeat, result.loop)} REFUSED`,
    text: runOutcomeText(result),
    details: [
      cleared ? `Before the refusal: ${cleared}.` : "Nothing was armed, moved or played.",
      `${code}${result.refusal?.step ? ` at step ${result.refusal.step}` : ""}`,
    ],
    code,
  };
}

/** A press this client did not send, and why (another request still in
 *  flight, a maintenance notice, the wrong cell). */
export function runNotSent(why: string, repeat?: RunRepeat): PressRefusal {
  return { kicker: `${pressName(repeat, null)} NOT SENT`, text: why, details: ["Nothing was sent to the robot."], code: "not_sent" };
}

/** A refused Finish cycle: the loop keeps running; STOP still ends it at once. */
export function finishRefusal(code: string, detail: string): PressRefusal {
  return {
    kicker: "FINISH CYCLE REFUSED",
    text: detail || code || "Finish cycle refused",
    details: ["The loop keeps running. STOP ends it at once.", code || "finish_refused"],
    code: code || "finish_refused",
  };
}
