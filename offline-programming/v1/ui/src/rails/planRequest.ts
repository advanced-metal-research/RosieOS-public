/**
 * The weld planner's request for a program, and the trajectory kept from its
 * answer.
 *
 * Both halves are the program's, not the page's: the browser's Plan button and
 * the Steam Deck pendant pack the same body from the same program and keep the
 * same record of what came back, so a plan made on either is the plan the
 * other would have asked for, and Load on either stages the same bytes.
 */
import { hasWorkpieceGeometry } from "../domain/placement.ts";
import type { DenseTrajectoryRecord, OfflineProgrammingProject } from "../domain/project.ts";
import type { WorkcellFixtureWire } from "../domain/workcell.ts";
import type { DenseLoadIdentity } from "../execution/denseClient.ts";
import { computeProgramV2Digest, type ProgramV2Node } from "../contracts/robot-v4-program-v2.ts";
import { toProgramV2Document } from "../program-v2/from-project.ts";
import { lowerManualPoseWelds } from "../program-v2/manual-pose-seam.ts";
import { effectiveNodes } from "../program-v2/nodes.ts";
import { isRailsWeld } from "../program-v2/rails-node.ts";
import { isManualPoseWeld } from "../waypoints/node.ts";
import type { FreeSpaceBackend, PlanRequestBody, RailsPlanResult } from "./client.ts";
import { SHIPPED_TORCH_PRESETS, torchRequest } from "./parameters.ts";

/** What a plan needs that the program does not carry. */
export interface PlanInputs {
  /** The fitted gun, as the seam worker reads one, with its default standoff. */
  tooling: Record<string, unknown>;
  /** The part, when the program has one; "" when it has none. */
  stepBase64: string;
  stepFilename: string;
  fixtures: WorkcellFixtureWire[];
  freeSpaceBackend: FreeSpaceBackend;
  requireMeasuredStart?: boolean;
  /** Plan from the program's own start (the model reset pose) instead of the
   *  robot's measured pose; Go to plan start then comes before Load and Play.
   *  The operator's explicit choice, never combined with requireMeasuredStart. */
  planFromProgramStart?: boolean;
  /** The operator plans for a repeat (Run N times, Loop until stopped):
   *  the planner adds a collision-checked return from the program's end to
   *  its first configuration, so cycles flow (`withLoopReturn`). */
  closeLoop?: boolean;
}

/** The node id of the loop's planned return; the dense segment carries it
 *  as its source id (OLP denseexec LoopReturnSourceID). */
export const LOOP_RETURN_NODE_ID = "rosie-loop-return";

/** The program sent to the planner for a repeat: its nodes, then one Home
 *  marked `rosie.loop_return`, which the planner plans as a Home to the
 *  plan's first configuration instead of the reset pose (weld_planner
 *  taught.py) and skips when the program already ends there. Only on a
 *  program the planner flies in order (a measured start, or any taught move
 *  or output): the weld-only model-reset planner starts and ends at reset. */
export function withLoopReturn<T extends { nodes: ProgramV2Node[] }>(document: T, inOrder: boolean): T {
  const motion = document.nodes.some(node => node.enabled !== false && ["move", "weld", "approach", "transit", "retract", "home"].includes(node.op));
  if (!inOrder || !motion || document.nodes.some(node => node.id === LOOP_RETURN_NODE_ID)) return document;
  const node = {
    id: LOOP_RETURN_NODE_ID, op: "home", enabled: true, label: "Return to program start", comment: "",
    row_hash: LOOP_RETURN_NODE_ID, extensions: { "rosie.loop_return": true },
  } as unknown as ProgramV2Node;
  return { ...document, nodes: [...document.nodes, node] };
}

/** The nodes a Plan asks for: seam welds, waypoint welds, taught moves and
 *  Home, edited or not. Empty means there is nothing to plan. A Home alone is
 *  a plan from the robot's pose: one transit to the reset pose. */
export function plannedNodes(project: OfflineProgrammingProject): ProgramV2Node[] {
  return effectiveNodes(project.nodes).filter((node) => isRailsWeld(node) || isManualPoseWeld(node) || node.op === "move" || node.op === "home");
}

/** Every enabled node that moves the arm (weldplan/measured_start.py
 *  MOTION_OPS): a connected Plan of a program with any of them starts from
 *  the robot's measured pose (weld_plan_start.go hasMotionNodes). */
export function hasMotionNodes(project: OfflineProgrammingProject): boolean {
  return effectiveNodes(project.nodes).some((node) => node.enabled !== false &&
    ["move", "weld", "approach", "transit", "retract", "home"].includes(node.op));
}

/** Why a Plan of this program would be refused, or "". Every motion node
 *  plans from the robot's measured pose (2026-10-11). An `io` row is
 *  scheduled against the connected cell's outputs (weld_planner
 *  weldplan/io_schedule.py), so with no cell it cannot be planned: the OLP
 *  answers taught_program_unsupported_node with this same sentence
 *  (weld_plan_start.go taughtProgramRefusal). Connected, the cell decides
 *  (io_not_configured, an unknown channel) and the server says which. Said
 *  before posting, so the operator is told which row rather than "Planning
 *  failed". */
export function measuredStartUnsupported(project: OfflineProgrammingProject, connected = false): string {
  if (connected) return "";
  const names = effectiveNodes(project.nodes).filter((node) => node.enabled !== false && node.op === "io").map((node) => {
    const label = (node.label ?? "").trim();
    return !label || label.toLowerCase() === node.op ? node.op.toUpperCase() : `${label} (${node.op})`;
  });
  if (!names.length) return "";
  return `${names.join(", ")} ${names.length === 1 ? "switches an output" : "switch outputs"}: ` +
    "plan with the cell connected, so the plan knows the cell's outputs and their expiry";
}

/** The shipped fitted gun, for a planner with no torch window: the first
 *  shipped preset is the parameter window's own fallback. */
export function shippedTooling(): Record<string, unknown> {
  const preset = SHIPPED_TORCH_PRESETS[0]!;
  return { ...torchRequest(preset.values, preset), default_standoff_mm: Number(preset.values.default_standoff_mm) };
}

export async function planRequestBody(project: OfflineProgrammingProject, inputs: PlanInputs): Promise<PlanRequestBody> {
  if (project.nodes.some(node => node.enabled && node.op === "weld" && node.geometry.kind === "posed_polyline")) throw new Error("Legacy posed welds use the classic planner. Recreate them with Waypoint welds before mixing them with taught moves.");
  // Keep authoring freshness separate from the effective, lowered request.
  const authoredDigest = await computeProgramV2Digest(toProgramV2Document(project));
  const lowered = lowerManualPoseWelds(toProgramV2Document({ ...project, nodes: effectiveNodes(project.nodes) }));
  const inOrder = inputs.planFromProgramStart !== true || effectiveNodes(project.nodes).some(node => node.enabled !== false && (node.op === "move" || node.op === "io"));
  const document = inputs.closeLoop === true ? withLoopReturn(lowered, inOrder) : lowered;
  document.program_digest = await computeProgramV2Digest(document);
  const geometry = hasWorkpieceGeometry(project);
  return {
    program: document,
    require_measured_start: inputs.requireMeasuredStart === true && inputs.planFromProgramStart !== true,
    ...(inputs.planFromProgramStart === true ? { plan_from_program_start: true } : {}),
    tooling: inputs.tooling,
    step_base64: geometry ? inputs.stepBase64 : "",
    workpiece_absent: !geometry,
    step_filename: inputs.stepFilename,
    cell_id: project.robot.model,
    ...(project.robot.cell?.machinePlanningCalibrationBase64 ? { machine_planning_calibration_base64: project.robot.cell.machinePlanningCalibrationBase64 } : {}),
    program_id: project.program.programId,
    fixtures: inputs.fixtures,
    program_digest: authoredDigest,
    free_space_backend: inputs.freeSpaceBackend,
    manifest_revision: 1,
    plan_revision: project.program.programRevision,
    // The cell's own deviations, as pulled off it: the planner applies them
    // and stamps the plan with the hashes the daemon checks at play.
  };
}

/** The planner's trajectory as the program keeps it, stamped with the
 *  revision it was planned from; null when the planner built none. */
export function denseTrajectoryRecordOf(project: OfflineProgrammingProject, result: RailsPlanResult): DenseTrajectoryRecord | null {
  const dense = result.dense;
  if (!dense) return null;
  return {
    schema: dense.schema,
    trajectoryDigest: dense.trajectory_digest,
    requestSha256: dense.request_sha256,
    planId: dense.plan_id,
    programId: dense.program_id,
    programDigest: dense.program_digest,
    manifestRevision: dense.manifest_revision,
    planRevision: dense.plan_revision,
    createdAt: dense.created_at,
    dtS: dense.dt_s,
    totalSampleCount: dense.total_sample_count,
    totalDurationS: dense.total_duration_s,
    bytes: dense.bytes,
    segments: dense.segments.map((segment) => ({
      index: segment.index,
      kind: segment.kind,
      sourceId: segment.source_id,
      sampleCount: segment.sample_count,
      durationS: segment.duration_s,
    })),
    sourceProjectRevision: project.identity.revision,
    sourceProgramDigest: result.program_digest,
    robot: dense.robot_cell ? {
      modelId: dense.robot_cell.model_id,
      robotDescriptionSha256: dense.robot_cell.robot_description_sha256,
      machinePlanningCalibrationSha256: dense.robot_cell.machine_planning_calibration_sha256,
      machinePlanningCalibrationSource: dense.robot_cell.machine_planning_calibration_source,
    } : null,
    ...(dense.process_markers_sha256 ? {
      processMarkersSha256: dense.process_markers_sha256,
      processMarkerCount: dense.process_marker_count ?? 0,
    } : {}),
  };
}

/** The six-field identity a Load or Run program press names, the run mode,
 *  and the output schedule digest when the plan has one. An OLP older than
 *  output execution ignores the digest and loads such a plan only as a dry
 *  run. */
export function denseLoadIdentity(record: DenseTrajectoryRecord, dryRun: boolean): DenseLoadIdentity {
  return {
    dry_run: dryRun,
    trajectory_digest: record.trajectoryDigest,
    plan_id: record.planId,
    program_id: record.programId,
    program_digest: record.programDigest,
    manifest_revision: record.manifestRevision,
    plan_revision: record.planRevision,
    ...(record.processMarkersSha256 ? { process_markers_sha256: record.processMarkersSha256 } : {}),
  };
}
