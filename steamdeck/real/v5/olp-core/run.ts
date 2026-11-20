// The Run page, decided by OLP's own code: which planner a program uses, the
// weld planner's request body and the record kept from its answer
// (rails/planRequest.ts, shared with the browser's Plan button), the Load /
// Play / Pause / Stop gates and the plan-start checks (execution/runControls.ts,
// shared with the Machine panel), and the executing node (dense-cursor.ts).
// The Qt side posts what this returns and paints what this says.
import { hasWorkpieceGeometry } from "../../../../offline-programming/v1/ui/src/domain/placement.ts";
import { recordPlan } from "../../../../offline-programming/v1/ui/src/domain/planHistory.ts";
import { workcellFixturesWire } from "../../../../offline-programming/v1/ui/src/domain/workcell.ts";
import { environmentFixtures } from "../../../../offline-programming/v1/ui/src/domain/workpieces.ts";
import { denseSegmentNodeId } from "../../../../offline-programming/v1/ui/src/program-v2/dense-cursor.ts";
import type { RailsPlanResult } from "../../../../offline-programming/v1/ui/src/rails/client.ts";
import { denseLoadIdentity, denseTrajectoryRecordOf, measuredStartUnsupported, plannedNodes, planRequestBody, shippedTooling } from "../../../../offline-programming/v1/ui/src/rails/planRequest.ts";
import { flattenForPlayback } from "../../../../offline-programming/v1/ui/src/execution/denseTrajectoryBlob.ts";
import { robotModelFor } from "../../../../offline-programming/v1/ui/src/viewer/models.ts";
import { machineReachable } from "../../../../offline-programming/v1/ui/src/execution/teach.ts";
import {
  atPlanStart, deriveRunControls, describePlan, gatePlayOnPlanStart, PLANNED_FROM_PROGRAM_START, planMotionBlocked, planStartEligible, planStartOf,
} from "../../../../offline-programming/v1/ui/src/execution/runControls.ts";
import { programModelMismatch } from "../../../../offline-programming/v1/ui/src/program-v2/capture.ts";
import {
  deriveRunProgram, finishControl, loopText, MAX_LOOP_COUNT, normalizeRunResult, repeatOf, RUN_CANCELLED, runConfirmation, runOutcomeText, runProgramRequest,
  runRefusal, type RepeatMode,
} from "../../../../offline-programming/v1/ui/src/execution/runProgram.ts";
import { canonicalProgramV2JSON } from "../../../../offline-programming/v1/ui/src/contracts/robot-v4-program-v2.ts";
import { toProgramV2Document } from "../../../../offline-programming/v1/ui/src/program-v2/from-project.ts";
import { planningFailure } from "../../../../offline-programming/v1/ui/src/ui/planning-failure.ts";
import { commitProject, currentProject } from "./session.ts";
import { machineCapabilities, machineStatus } from "./machine.ts";
import { DecodedTrajectoryCache, type TrajectorySource } from "./decoded-trajectory.ts";

let startPositions: number[] | null = null;

// The next PLAN starts from the program's own start rather than the robot's
// measured pose (weld_plan.go PlanFromProgramStart). One plan's choice: set
// by the operator, cleared by a kept plan, so a later PLAN is measured again.
let fromProgramStart = false;

// Run once, Run N times or Loop until stopped: the operator's choice for the
// next PLAN (a repeat plans the planner's return to the start) and RUN.
let repeatMode: RepeatMode = "once";
let repeatCount = 5;

/** The repeat chosen for the next RUN (the refusal toast names the press). */
export function runRepeat(): { mode: RepeatMode; count: number } {
  return { mode: repeatMode, count: repeatCount };
}

/** Choose the repeat for the next PLAN and RUN; the count is clamped to 2..MAX. */
export function setRepeat(mode: string, count?: number): void {
  if (mode === "once" || mode === "count" || mode === "loop") repeatMode = mode;
  if (typeof count === "number" && Number.isFinite(count)) repeatCount = Math.min(MAX_LOOP_COUNT, Math.max(2, Math.round(count)));
}

/** Choose the program's start (true) or the robot's pose (false) for the next PLAN. */
export function setPlanFromProgramStart(on: boolean): void {
  fromProgramStart = on;
}

export interface RunInput {
  connected: boolean;
  cellPending: boolean;
  inFlight: boolean;
  statusAgeMs: number | null;
  lastRequestError: string | null;
  planStartActive: boolean;
  planning: boolean;
  cellModel: string;
  /** The kept trajectory's bytes are on this pendant (planned here). */
  bytesKept: boolean;
  /** This pendant holds a jog (stick or touch). */
  holding: boolean;
  /** This pendant's Move-to, Cartesian step or Go to plan start is in flight. */
  moving: boolean;
  /** This pendant's Run program press is in flight, its question included. */
  running?: boolean;
}

// The run mode is the operator's choice for the next Load, as the Machine
// panel's select holds it: seeded from the program when a program is opened
// (production plays its process outputs, anything else is a dry run) and
// never written back into it.
let runModeProject = "";
let dryRunChosen = true;

function dryRun(): boolean {
  const project = currentProject();
  if (project.identity.id !== runModeProject) {
    runModeProject = project.identity.id;
    dryRunChosen = project.program.executionMode !== "production";
  }
  return dryRunChosen;
}

/** Everything the Run page paints, in OLP's words. */
export function runView(input: RunInput) {
  const project = currentProject();
  const status = machineStatus(), capabilities = machineCapabilities();
  const native = capabilities?.backend === "rt_core";
  const record = project.denseTrajectory;
  const controls = deriveRunControls({
    targetSelected: input.connected, cellPending: input.cellPending, inFlight: input.inFlight, native, status, record,
    projectRevision: project.identity.revision, dryRun: dryRun(),
    daemonReachable: machineReachable(input.connected, status, input.statusAgeMs),
    lastRequestError: input.lastRequestError, planStartActive: input.planStartActive,
  });
  const eligible = planStartEligible({ targetSelected: input.connected, cellPending: input.cellPending, status, capabilities, project });
  if (!eligible) startPositions = null;
  const atStart = !input.planStartActive && !controls.live && atPlanStart(startPositions, status, capabilities, input.statusAgeMs);
  const startGate = !record || !controls.current ? "Select a current plan first" : !input.bytesKept ? "Plan data is unavailable" : "";
  const execution = status?.rtCore?.execution;
  const matches = record && status?.session?.trajectoryDigest === record.trajectoryDigest &&
    status.session.planId === record.planId && record.sourceProjectRevision === project.identity.revision &&
    execution?.identity?.trajectory_digest === status.session.executionTrajectoryDigest;
  const segment = matches ? record.segments.find(candidate => candidate.index === execution?.segment) : null;
  const executing = segment && (controls.state === "playing" || controls.state === "paused")
    ? denseSegmentNodeId(project.nodes, segment.kind, segment.sourceId) : null;
  const nothing = plannedNodes(project).length === 0;
  const geometry = hasWorkpieceGeometry(project);
  const mismatch = input.connected ? programModelMismatch(project.robot.model, input.cellModel) : "";
  const motion = planMotionBlocked({ holding: input.holding, moving: input.moving || controls.live }, "pendant");
  // PLAY waits for the robot at the plan's first sample, as the browser's
  // Machine panel does; native Play would refuse it anyway.
  const play = gatePlayOnPlanStart(controls, native, atStart, machineReachable(input.connected, status, input.statusAgeMs));
  return {
    plan: !record ? "NO PLAN" : controls.current ? "PLANNED" : "STALE",
    planText: record ? `${record.totalSampleCount} samples · ${record.totalDurationS.toFixed(1)} s · ${describePlan(record)}` : "",
    planDigest: record?.trajectoryDigest ?? "",
    planBlocked: input.planning ? "Planning…" : mismatch || (nothing ? "Nothing to plan — record a waypoint or add a weld"
      : geometry ? "This program's part (STEP) is not on the pendant; plan it in offline programming, then open it here"
      : measuredStartUnsupported(project, input.connected) || motion),
    /** The next PLAN starts from the program's start, not the robot's pose
     *  (only meaningful while connected: offline plans always do). */
    planFromProgramStart: fromProgramStart && input.connected,
    /** The program is for another robot than the connected cell's (also shown on PROGRAM). */
    modelMismatch: mismatch,
    dryRun: dryRun(),
    controls: { ...controls, play },
    /** The one press (`runProgram.ts`): Run program, or Run again after this
     *  plan completed. GO TO PLAN START, LOAD and PLAY stay under Advanced. */
    runProgram: deriveRunProgram({
      controls, native, targetSelected: input.connected, cellPending: input.cellPending, running: input.running === true,
      record, session: status?.session, loop: status?.loop ?? null, repeat: repeatOf(repeatMode, repeatCount),
    }),
    /** The repeat chosen, the loop's counter line and Finish cycle (runProgram.ts). */
    repeat: { mode: repeatMode, count: repeatCount },
    loop: loopText(status?.loop ?? null),
    looping: status?.loop?.state === "running" || status?.loop?.state === "finishing",
    finish: finishControl(status?.loop ?? null),
    planStart: {
      disabled: atStart || !native || input.cellPending || input.inFlight || controls.live || !!startGate,
      text: input.planStartActive ? "Moving to plan start…" : atStart ? "✓ At plan start" : "Go to plan start",
      title: atStart ? "All joint positions match this plan's start within the machine's reported completion tolerances. Confirmed from fresh feedback."
        : startGate || "Move all joints to this plan's first position at the selected Jog speed. Arm first; STOP cancels. Process outputs stay off.",
      eligible,
    },
    executing,
    session: status?.session ?? null,
    cursor: status?.cursor ?? null,
  };
}

/** The weld planner's request for the open program: the browser's body,
 *  with the shipped fitted gun and no part (the pendant holds no STEP). */
export async function planBody() {
  const project = currentProject();
  if (plannedNodes(project).length === 0) throw new Error("Nothing to plan — record a waypoint or add a weld");
  if (hasWorkpieceGeometry(project)) throw new Error("This program's part (STEP) is not on the pendant; plan it in offline programming, then open it here");
  return planRequestBody(project, {
    tooling: shippedTooling(), stepBase64: "", stepFilename: "part.step",
    fixtures: workcellFixturesWire(environmentFixtures(project), new Map()), freeSpaceBackend: "bspline",
    planFromProgramStart: fromProgramStart,
    closeLoop: repeatOf(repeatMode, repeatCount).mode !== "once",
  });
}

/** Keep the planner's answer as the browser's applyPlan does: the record on
 *  the program and one history entry; returns the digest the bytes are kept
 *  under, or "" when the planner built no trajectory. */
export function applyPlan(result: RailsPlanResult, planRevision: number, planProjectId: string): string {
  const project = currentProject();
  // A numeric revision is only unique within one program. New/Open during
  // planning must not attach the previous program's result to another one.
  if (project.identity.id !== planProjectId || project.identity.revision !== planRevision)
    throw new Error("The program changed while it was planning; plan it again");
  const record = denseTrajectoryRecordOf(project, result);
  if (!record) {
    const failure = result.dense_error;
    throw new Error(failure ? `no trajectory (${failure.reason}${failure.detail ? `: ${failure.detail}` : ""})` : "the planner built no trajectory");
  }
  commitProject(recordPlan({ ...project, denseTrajectory: record }));
  startPositions = null;
  fromProgramStart = false;
  return record.trajectoryDigest;
}

/** The plan card's note for a kept plan: a plan from the program's start
 *  says what comes before Load and Play. `planned` is what was asked. */
export function plannedNote(samples: number, seconds: number, fromStart: boolean): string {
  return `Planned · ${samples} samples, ${seconds.toFixed(1)} s · ` +
    (fromStart ? PLANNED_FROM_PROGRAM_START.replace(/^planned from/, "from") : "Preview, or Load to robot");
}

/** Dry run or program outputs for the next Load. The caller keeps it locked
 *  while a request is in flight, a cell is pending or the robot moves. */
export function setRunMode(production: boolean): void {
  dryRun();
  dryRunChosen = !production;
}

const decodedTrajectory = new DecodedTrajectoryCache();
function trajectoryOf(source: TrajectorySource) {
  const project = currentProject();
  const status = machineStatus();
  const identity = JSON.stringify([project.identity.id, project.identity.revision, project.robot.model,
    project.robot.description?.robotDescriptionSha256, project.robot.jointOrder, project.denseTrajectory?.trajectoryDigest,
    status?.robotCell, status?.rtCore?.coordinateIdentity, status?.rtCore?.observationIdentity]);
  return decodedTrajectory.read(identity, source);
}

/** The kept plan's first joint positions for Go to plan start, or OLP's reason there are none. */
export function planStart(source: TrajectorySource, cellModel: string): { positions: number[] } | { error: string } {
  const start = planStartOf({
    status: machineStatus(), capabilities: machineCapabilities(), trajectory: trajectoryOf(source),
    project: currentProject(), cellModel,
  });
  if ("positions" in start && start.positions.every(Number.isFinite)) startPositions = start.positions;
  return start;
}

/** The kept plan as a preview path for the 3D view: joint positions by id
 *  every `stepS` seconds of elapsed plan time. A scrubber, never a command. */
export function previewPath(source: TrajectorySource, stepS = 1 / 30) {
  const trajectory = trajectoryOf(source);
  const order = currentProject().robot.jointOrder;
  const samples = flattenForPlayback(trajectory);
  const frames: { t: number; segment: number; q: number[] }[] = [];
  let next = 0;
  samples.forEach((entry, index) => {
    if (entry.elapsedS + 1e-9 < next && index !== samples.length - 1) return;
    frames.push({ t: entry.elapsedS, segment: entry.segmentIndex, q: Array.from(entry.sample.qRad) });
    next = entry.elapsedS + stepS;
  });
  return { jointOrder: [...order], durationS: samples.at(-1)?.elapsedS ?? 0, frames };
}

/** The kept plan's joint samples for the 3D view's planned TCP path, as OLP's
 *  densePlannedPathRuns keeps them: every dense sample, one run per segment so
 *  no line crosses a gap the planner left, weld or travel. The Qt side poses
 *  `frame` (the workstation's descriptor.cameras.tcp.frame) through its own FK. */
export function plannedPathRuns(source: TrajectorySource) {
  const project = currentProject();
  const trajectory = trajectoryOf(source);
  // The pendant needs only coordinates and one colour per segment. Preserve
  // densePlannedPathRuns' exact projection, without allocating its browser
  // SimulationSample metadata and joint-order copy for every dense sample.
  // Explicit indexed reads also avoid Qt 5.15's per-value iterator overhead.
  const runs = trajectory.segments.map(segment => ({
    weld: segment.samples.length > 0 && (segment.kind === "weld" || segment.kind === "dwell"),
    q: segment.samples.map(sample => {
      const q: number[] = [];
      const count = Math.min(project.robot.jointOrder.length, sample.qRad.length);
      for (let axis = 0; axis < count; ++axis) q.push(sample.qRad[axis]!);
      return q;
    }),
  }));
  return {
    jointOrder: [...project.robot.jointOrder], frame: robotModelFor(project.robot.model).cameras.tcp.frame,
    runs,
  };
}

/** The open program's canonical program.v2 JSON: what its authored digest is
 *  the SHA-256 of (core_check compares the engine's hash with Qt's). */
export function authoredProgramJSON(): string {
  return canonicalProgramV2JSON(toProgramV2Document(currentProject()));
}

/** POST /load's body: the kept plan's six-field identity and the run mode. */
export function loadBody() {
  const record = currentProject().denseTrajectory;
  if (!record) throw new Error("Plan a rails program first: the daemon plays the trajectory the weld planner built");
  return { ...denseLoadIdentity(record, dryRun()) };
}

/** One Run program press's body: the kept plan's Load body (`base`, which
 *  may carry the kept bytes; loadBody() when null), the Jog speed, and on the
 *  second press the answer the operator confirmed (the raw `result` runAnswer
 *  returned), whose flags and joints go back for the server to check. */
export function runBody(base: Record<string, unknown> | null, fraction: number, confirmed?: unknown) {
  return runProgramRequest(base ?? loadBody(), fraction, confirmed === undefined || confirmed === null ? undefined : normalizeRunResult(confirmed),
    repeatOf(repeatMode, repeatCount));
}

/** One Run program reply in OLP's words: `confirm` carries the confirmation
 *  to show (the All joints to 0° dialog) and the raw result to send back;
 *  anything else the line to show (a refusal is the server's sentence after
 *  what cleared) and the refusal's code, so the pendant can resend with its
 *  kept bytes when the planner lost them. A refusal also carries `refusal`,
 *  the toast (pressnotice.ts): a press that started nothing says so on top. */
export function runAnswer(payload: unknown, ok: boolean, code = "", detail = "") {
  const result = normalizeRunResult(payload);
  const refusalCode = (payload && typeof payload === "object" && typeof (payload as { error?: unknown }).error === "string" ? (payload as { error: string }).error : "") || code;
  const repeat = repeatOf(repeatMode, repeatCount);
  if (!ok) {
    const text = result.state === "refused" && result.refusal ? runOutcomeText(result) : detail || code || "Run program refused";
    const refused = { ...result, state: "refused" as const, code: result.refusal?.code || refusalCode,
      refusal: result.refusal ?? { step: "", code: refusalCode || "run_refused", detail: text } };
    return { state: "refused", text, code: refused.code, refusal: runRefusal(refused, repeat) };
  }
  if (result.state === "confirm") return { state: result.state, text: runOutcomeText(result), confirmation: runConfirmation(result), result: payload, cancelled: RUN_CANCELLED };
  if (result.state === "refused") return { state: result.state, text: runOutcomeText(result), sessionId: result.sessionId, code: result.refusal?.code ?? "", refusal: runRefusal(result, repeat) };
  return { state: result.state, text: runOutcomeText(result), sessionId: result.sessionId, code: result.refusal?.code ?? "" };
}

/** A refused POST /weld-plan: `message` in the words the browser's plan
 *  dialog uses (for a jog or move still under way, the one instruction to
 *  let go), and `verbatim`, the server's code and sentence as it sent them,
 *  for the status line. */
export function planFailure(code: string, error: string, denseError: unknown, measuredStart = false) {
  const failure = planningFailure(Object.assign(new Error(error), denseError ? { dense_error: denseError } : {}, code ? { code } : {}), "pendant", measuredStart);
  // A failed plan from the robot's pose names the alternative: the pendant's
  // FROM PROGRAM START, then GO TO PLAN START before Load and Play.
  const alternative = failure.action?.key === "plan-from-program-start"
    ? " — or choose FROM PROGRAM START and PLAN again, then GO TO PLAN START" : "";
  return { message: failure.message + alternative, verbatim: code ? `${code}: ${error}` : error };
}

/** The kept plan's trajectory digest, the name its bytes are kept under; "" without one. */
export function planDigest(): string {
  return currentProject().denseTrajectory?.trajectoryDigest ?? "";
}
