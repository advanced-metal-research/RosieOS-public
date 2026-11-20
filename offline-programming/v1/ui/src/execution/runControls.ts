/**
 * Which program-run controls the Machine panel offers, and why not.
 *
 * Load, Play, Halt and Stop are gated on the same facts wherever a program is
 * run from: the browser's Machine panel and the Steam Deck pendant both call
 * this, so a Play that the one refuses the other cannot offer. It decides
 * nothing the server does not check again; it keeps the buttons from
 * inviting a request that is certain to be refused, and says why in words.
 */
import type { DenseTrajectoryRecord, OfflineProgrammingProject } from "../domain/project.ts";
import { denseExecutionState, denseProgramPlayable, type DenseCapabilities, type DenseStatus } from "./denseClient.ts";
import type { DenseTrajectory } from "./denseTrajectoryBlob.ts";
import { STATUS_FRESHNESS_MS } from "./machineFlow.ts";
import { processIOAdmissionMessage } from "../targets/operatorFeedback.ts";

/** Session states in which a program is under way: nothing else loads. */
export const LIVE_STATES: ReadonlySet<string> = new Set(["playing", "paused"]);

/** Play's reason while a loaded program waits for an armed, ready robot. */
export const PLAY_NEEDS_ARM = "Arm the robot to play";

/** The server's code for a plan refused because a jog or move is still under
 *  way, or the joints moved while the start was read (weld_plan_start.go). */
export const PLAN_START_MOVING = "planning_start_moving";

/** What a plan made from the program's own start (no measured capture) asks
 *  of the operator next, shown where the plan is reported. */
export const PLANNED_FROM_PROGRAM_START = "planned from the program's start: Go to plan start, then Load and Play";

/** Play's reason while a loaded plan's first sample is not where the robot
 *  is: native Play refuses it, and nothing approaches the start on its own.
 *  A plan from the measured pose is already there unless the robot moved. */
export const PLAY_NEEDS_PLAN_START = "Go to plan start first: Play starts only with every joint at this plan's first position. No automatic approach is performed.";

/** Play gated on the plan start, for both apps (the browser's Machine panel
 *  and the Deck's RUN page): a loaded native plan plays only with the robot
 *  confirmed at its first sample from fresh feedback. Before 2026-10-11 only
 *  the browser applied this, and the Deck offered PLAY for a program-start
 *  plan that native Play then refused. */
export function gatePlayOnPlanStart(controls: RunControls, native: boolean, atStart: boolean, daemonReachable: boolean): RunControl {
  if (native && controls.loaded && !controls.live && !atStart) {
    return { disabled: true, title: daemonReachable ? PLAY_NEEDS_PLAN_START : controls.play.title };
  }
  return controls.play;
}

/**
 * The one instruction both apps give when planning must wait for motion to
 * end: a plan starts from the robot's measured pose, read while every joint
 * is still. A finished jog or move needs nothing more -- no Stop, no disarm.
 */
export function planMotionInstruction(device: "pendant" | "desktop"): string {
  return device === "pendant"
    ? "Release the sticks and wait for the robot to stop, then PLAN"
    : "Release the jog and wait for the robot to stop, then Plan";
}

/** Why Plan waits for this client's own motion, or "": a held jog, a Move-to
 *  or step in flight, or a program playing. The server refuses all three. */
export function planMotionBlocked(input: { holding: boolean; moving: boolean }, device: "pendant" | "desktop"): string {
  return input.holding || input.moving ? planMotionInstruction(device) : "";
}

export interface RunControlsInput {
  targetSelected: boolean;
  cellPending: boolean;
  inFlight: boolean;
  /** The target is rt-core, which gates Load on Home and fresh status. */
  native: boolean;
  status: DenseStatus | null;
  record: DenseTrajectoryRecord | null;
  projectRevision: number;
  dryRun: boolean;
  /** machineReachable(): selected, status fresh and native time advancing. */
  daemonReachable: boolean;
  lastRequestError: string | null;
  /** Go to plan start is moving: Halt stays offered while that request is in flight. */
  planStartActive: boolean;
}

export interface RunControl { disabled: boolean; title: string }

export interface RunControls {
  state: string;
  live: boolean;
  /** The kept plan is for this revision of the program. */
  current: boolean;
  /** The kept plan is staged on the machine in the chosen run mode, and Play can start it. */
  loaded: boolean;
  homeRequired: boolean;
  /** Why Load cannot run now, shown beside it; "" when nothing blocks it. */
  loadBlocker: string;
  load: RunControl & { text: string };
  play: RunControl;
  halt: RunControl;
  stop: RunControl;
  /** Offer "Use dry run" after a process-output admission refusal. */
  offerDryRun: boolean;
}

/** When this was planned and which program it came out of. Read on the load
 *  button, because loading the wrong plan into a robot is the mistake worth
 *  spending a tooltip on: a trajectory carries no trace of its program once it
 *  is bytes, and "the one that is open" is a guess. */
export function describePlan(record: DenseTrajectoryRecord): string {
  const when = Number.isNaN(Date.parse(record.createdAt))
    ? record.createdAt
    : new Date(record.createdAt).toLocaleString();
  const which = record.sourceProgramDigest
    ? `program ${record.sourceProgramDigest.replace(/^sha256:/, "").slice(0, 12)}`
    : `project revision ${record.sourceProjectRevision}`;
  return `planned ${when}, from ${which}`;
}

export function deriveRunControls(input: RunControlsInput): RunControls {
  const { status, record, native } = input;
  const session = status?.session ?? null;
  const state = denseExecutionState(status);
  const live = session !== null && LIVE_STATES.has(state);
  // Still the project revision. `sourceProgramDigest` says WHICH program was
  // planned and is shown, but comparing it needs the open program's digest,
  // and computing that is async where a render is not. The revision is the
  // coarser answer: it moves for edits that change no motion.
  const current = record !== null && record.sourceProjectRevision === input.projectRevision;
  const loaded = current && record !== null && denseProgramPlayable(status, record.trajectoryDigest, input.dryRun);
  const homeRequired = native && !!status?.rtCore?.axes.some(axis => !axis.homeValid);
  const loadBlocker = native && input.targetSelected
    ? !input.daemonReachable ? "Waiting for fresh machine status before loading."
      : homeRequired ? status?.rtCore?.operatorFeedback.axes.find(axis => axis.homeLoss)?.homeLoss || "Home is required before loading. Review the Home step to restore the robot's position references." : ""
    : "";
  const modeMismatch = !!session && session.trajectoryDigest === record?.trajectoryDigest && session.dryRun !== input.dryRun;
  const loadTitle = loadBlocker || (record === null
    ? "Plan a rails program first: the daemon plays the trajectory the weld planner built"
    : !current
      ? "The program changed since this was planned; plan it again"
      : live
        ? "Playing; STOP before loading another trajectory"
        : `Load ${record.totalSampleCount} samples (${record.totalDurationS.toFixed(1)} s) into `
          + `the joint daemon — ${describePlan(record)}`);
  const processRefusal = [input.lastRequestError, status?.rtCore?.operatorFeedback?.rejection?.code]
    .some(reason => reason && processIOAdmissionMessage(reason));
  const homeLoss = status?.rtCore?.operatorFeedback.axes.find(axis => axis.homeLoss)?.homeLoss;
  // The server's Play waits for Armed and ready until its deadline, then
  // fails. `loaded` already requires both on rt-core (denseProgramPlayable),
  // so a plan staged on a disarmed robot read "Load to robot first" right
  // after a successful Load: say what is missing instead. Same fact as the
  // test-program Play (testProgram.ts).
  const armedReady = status?.armed === true && status.rtCore?.ready === true;
  const stagedUnarmed = native && current && record !== null && state === "loaded" && !armedReady &&
    session?.trajectoryDigest === record.trajectoryDigest && session.dryRun === input.dryRun;
  const play: RunControl = native && !input.daemonReachable
    ? { disabled: true, title: "Waiting for fresh machine status before playing." }
    : native && homeLoss
    ? { disabled: true, title: homeLoss }
    : stagedUnarmed
    ? { disabled: true, title: PLAY_NEEDS_ARM }
    : {
      disabled: !input.targetSelected || input.cellPending || input.inFlight || !loaded || state === "playing",
      title: modeMismatch ? "Reload to robot to apply the selected run mode" : !loaded ? "Load to robot first" : state === "paused" ? "Resume" : "Play from the first sample",
    };
  return {
    state,
    live,
    current,
    loaded,
    homeRequired,
    loadBlocker,
    load: {
      disabled: !input.targetSelected || input.cellPending || input.inFlight || live || record === null || !current || !!loadBlocker,
      title: loadTitle,
      text: modeMismatch ? "Reload to robot" : "Load to robot",
    },
    play,
    halt: { disabled: !input.targetSelected || (input.inFlight && !input.planStartActive) || session === null, title: "Stop the robot and release the lease" },
    // STOP is enabled whenever a target is selected: it needs no session
    // and is answered even when the daemon has nothing to stop.
    stop: { disabled: !input.targetSelected, title: "Software stop" },
    offerDryRun: native && input.targetSelected && processRefusal && !input.dryRun,
  };
}

/** A simulator with no robot description configured: its plan is bound to the
 *  program's own description and an empty calibration instead of the cell's. */
export function unboundSimulation(status: DenseStatus | null, capabilities: DenseCapabilities | null): boolean {
  return capabilities?.rtCore?.simulation === true && status?.robotCell?.configured === false;
}

/** The trajectory was planned for the robot and calibration this machine reports. */
export function planRobotMatches(input: {
  status: DenseStatus | null; capabilities: DenseCapabilities | null; trajectory: DenseTrajectory;
  project: OfflineProgrammingProject; cellModel: string;
}): boolean {
  const robot = input.status?.robotCell, binding = input.trajectory.robot;
  if (unboundSimulation(input.status, input.capabilities)) {
    return !!(binding && binding.modelId === input.cellModel && binding.modelId === input.project.robot.model &&
      binding.robotDescriptionSha256 === input.project.robot.description?.robotDescriptionSha256 &&
      binding.machinePlanningCalibrationSource === "empty" && binding.machinePlanningCalibrationSha256);
  }
  return !!(robot?.valid && binding?.modelId && binding.robotDescriptionSha256 && binding.machinePlanningCalibrationSha256 &&
    robot.modelId === binding.modelId && robot.robotDescriptionSha256 === binding.robotDescriptionSha256 &&
    robot.machinePlanningCalibrationSha256 === binding.machinePlanningCalibrationSha256);
}

/** Whether a plan start can be read for this machine at all: a current plan,
 *  a described robot (or an unbound simulator) and rotary axes in the
 *  program's joint order. */
export function planStartEligible(input: {
  targetSelected: boolean; cellPending: boolean; status: DenseStatus | null; capabilities: DenseCapabilities | null;
  project: OfflineProgrammingProject;
}): boolean {
  const record = input.project.denseTrajectory;
  const axes = input.capabilities?.rtCore?.axes ?? [];
  return input.targetSelected && !input.cellPending && record?.sourceProjectRevision === input.project.identity.revision &&
    (!!input.status?.robotCell?.valid || unboundSimulation(input.status, input.capabilities)) && axes.length > 0 &&
    axes.every((a, i) => a.position_unit === "rad" && a.id === input.project.robot.jointOrder[i]);
}

/** The plan's first joint positions, in the machine's axis order, or the
 *  reason there are none to move to. */
export function planStartOf(input: {
  status: DenseStatus | null; capabilities: DenseCapabilities | null; trajectory: DenseTrajectory;
  project: OfflineProgrammingProject; cellModel: string;
}): { positions: number[] } | { error: string } {
  const record = input.project.denseTrajectory;
  const { trajectory } = input;
  if (!record || trajectory.trajectoryDigest !== record.trajectoryDigest || trajectory.schema !== record.schema) return { error: "Stored trajectory does not match the selected plan" };
  if (!trajectory.robot) return { error: "Selected trajectory has no robot identity; replan before moving to its start" };
  if (!planRobotMatches(input)) return { error: "Plan robot or calibration does not match the connected machine" };
  const first = trajectory.segments[0]?.samples[0];
  const axes = input.capabilities?.rtCore?.axes ?? [];
  if (!first || !axes.length || axes.some((a, i) => a.id !== input.project.robot.jointOrder[i])) return { error: "Plan start joint order does not match the machine" };
  const captured = trajectory.startState;
  if (captured && (trajectory.programDigest !== record.programDigest || captured.coordinate_identity !== input.status?.rtCore?.coordinateIdentity ||
      captured.observation_identity !== input.status?.rtCore?.observationIdentity ||
      !Array.isArray(captured.joint_names) || !Array.isArray(captured.joint_values_rad) ||
      captured.joint_names.length !== axes.length || captured.joint_values_rad.length !== axes.length ||
      axes.some((axis, i) => captured.joint_names[i] !== axis.id || !Number.isFinite(captured.joint_values_rad[i]) ||
        Math.abs(captured.joint_values_rad[i]! - first.qRad[i]!) > 1e-10))) {
    return { error: "Measured planning start identity changed or its first sample does not match; replan from fresh feedback" };
  }
  return { positions: axes.map((_, i) => first.qRad[i]!) };
}

/** Every joint is at the plan start within the machine's own completion
 *  tolerance, from fresh, trusted feedback. */
export function atPlanStart(positions: readonly number[] | null, status: DenseStatus | null, capabilities: DenseCapabilities | null, statusAgeMs: number | null): boolean {
  const axes = capabilities?.rtCore?.axes;
  if (!positions || !axes || (statusAgeMs ?? Infinity) > STATUS_FRESHNESS_MS) return false;
  return positions.length === axes.length && axes.every((axis, i) => {
    const observed = status?.rtCore?.axes[i], tolerance = axis.completion_tolerance;
    return observed?.calibrationValid === true && observed.positionState === "trusted" &&
      observed.homeValid && observed.coordinateValid && observed.position !== null && Number.isFinite(observed.position) &&
      Number.isFinite(tolerance) && tolerance > 0 && Math.abs(observed.position - positions[i]!) <= tolerance;
  });
}
