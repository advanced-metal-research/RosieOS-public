import type {
  CadMotionSample,
  CadSequenceResult,
  CadTopology,
  CadTopologyEdge,
} from "../cad/contracts.ts";
import { ROBOT_V4_PROGRAM_MAX_ROWS } from "../contracts/robot-v4-program.ts";
import {
  type AuthoredProgramRow,
  type AuthoredSeam,
  type OfflineProgrammingProject,
  type PlanTrajectorySample,
  type Rpy,
  type RowPhase,
  type SequenceStep,
  type TcpPose,
  type Vec3,
  installPreviewPlan,
  projectId,
  reviseProject,
  zeroJoints,
} from "../domain/project.ts";
import {
  centreWorkpieceOnWorkFrame,
  transformWorkpiecePoint,
  worldPointToPlanner,
  workpieceWorldRotationRpy,
} from "../domain/placement.ts";
import { previewTrajectory } from "../domain/playback.ts";
import {
  polylineLengthM,
  trimPolylineByOffsetsM,
} from "../domain/seam.ts";
import {
  DEFAULT_MOVE_TOLERANCE_M,
  attachMoveMetadata,
  controlPointIndicesForPath,
  type MoveSegmentInput,
} from "../domain/points-moves.ts";
import {
  processTorchDefaults,
  torchOrientationRpy,
} from "../domain/weld.ts";
import { catalogFromTopology } from "../domain/edge-catalog.ts";
import { canonicalJoints } from "./joints.ts";
import { beginAddedWorkpiece, retainActiveWorkpiece } from "../domain/workpieces.ts";

/** Add a weld part without discarding existing parts, placements or welds. */
export function appendCadQueryTopology(project: OfflineProgrammingProject, topology: CadTopology): OfflineProgrammingProject {
  const draft = structuredClone(project);
  beginAddedWorkpiece(draft);
  const next = adaptCadQueryTopology(draft, topology);
  retainActiveWorkpiece(next);
  return next;
}

type Point3 = Vec3;

/**
 * Map the production CAD client's normalized metre-space topology into the
 * versioned project contract. Raw CadQuery wire types stay inside src/cad.
 */
export function topologyToSeams(
  topology: CadTopology,
  processParameterId = "process-default",
): AuthoredSeam[] {
  const recommendations = topology.recommendedSeams.length > 0
    ? topology.recommendedSeams
    : topology.edges
      .filter((edge) => edge.isSeamCandidate)
      .slice(0, 8)
      .map((edge) => ({
        edgeId: edge.id,
        partId: edge.partId,
        geometryType: edge.geometryType,
        pathClass: edge.pathClass,
        window: { start: 0, end: 1 },
        reversed: false,
        reason: "seam candidate fallback",
      }));
  return recommendations.flatMap((recommendation, index) => {
    const edge = topology.edges.find((candidate) => candidate.id === recommendation.edgeId);
    if (!edge || edge.samplesM.length < 2) return [];
    return [{
      id: `seam-${index + 1}-${safeId(edge.id)}`,
      label: edge.label ?? `SEAM ${String(index + 1).padStart(2, "0")}`,
      enabled: true,
      sourceEdgeId: edge.id,
      partId: edge.partId,
      geometryType: edge.geometryType,
      trim: recommendationTrimM(edge.samplesM, recommendation.window),
      reversed: recommendation.reversed,
      geometry: {
        kind: "polyline",
        pointsM: edge.samplesM.map(copyPoint),
      },
      workFrameId: "world",
      processParameterId,
    }];
  });
}

/**
 * P2.12 import contract (2026-08-04): loading a STEP part creates NO
 * seams and NO program rows. The import records the CAD block, fixture,
 * tolerance seed, and the pickable edge catalog (with extractor
 * recommendations as suggestion metadata only). The program is authored later
 * by the user selecting edges and creating welds.
 */
export function adaptCadQueryTopology(
  project: OfflineProgrammingProject,
  topology: CadTopology,
  timestamp?: string,
): OfflineProgrammingProject {
  return reviseProject(project, "cad", (draft) => {
    draft.cad = {
      kind: topology.filename === "dog_box.step" ? "example" : "step",
      filename: topology.filename,
      fingerprint: topology.fingerprint,
      sourceUnit: topology.sourceUnit,
      topologyModelId: topology.modelId,
    };
    draft.fixtures[0] = {
      ...(draft.fixtures[0] ?? {
        id: "cad-fixture",
        name: topology.filename,
        frameId: "positioner_table_a",
        translationM: [0, 0, 0],
        rotationRpyRad: [0, 0, 0],
      }),
      name: topology.filename,
      frameId: "positioner_table_a",
      sourceCadFingerprint: topology.fingerprint,
    };
    draft.edgeCatalog = catalogFromTopology(topology);
    // Seams and rows belong to the previous part's geometry; a new part starts
    // with an empty program and waits for the user's edge selection.
    draft.seams = [];
    draft.sequence = [];
    draft.rows = [];
    draft.simulation.selectedRowId = null;
    seedImportTolerance(draft);
    centreWorkpieceOnWorkFrame(draft);
    setCadRowBudgetStatus(draft, {
      maxRows: ROBOT_V4_PROGRAM_ROW_BUDGET,
      requiredRows: 0,
      importedRows: 0,
      overBudgetSeamIds: [],
      toleranceM: moveToleranceM(draft),
    });
  }, timestamp);
}

export function adaptCadQueryWeldSequence(
  project: OfflineProgrammingProject,
  payload: CadSequenceResult,
  timestamp?: string,
): OfflineProgrammingProject {
  const tcpPoses = payload.tesseractPlan?.tcpPoses ?? [];
  const poseBySeam = new Map<string, typeof tcpPoses>();
  for (const pose of tcpPoses) {
    const key = pose.seamId ?? "";
    const group = poseBySeam.get(key) ?? [];
    group.push(pose);
    poseBySeam.set(key, group);
  }
  const seams: AuthoredSeam[] = payload.weldSequence.seams.map((seam, index) => {
    const sampled = (poseBySeam.get(seam.id) ?? []).map((pose) => copyPoint(pose.targetM));
    const fallback = [copyPoint(seam.startPointM), copyPoint(seam.stopPointM)];
    return {
      id: seam.id || `seam-${index + 1}`,
      label: `SEAM ${String(index + 1).padStart(2, "0")}`,
      enabled: true,
      sourceEdgeId: seam.edgeId,
      partId: seam.partId,
      geometryType: seam.geometryType,
      trim: { startOffsetM: 0, endOffsetM: 0 },
      reversed: seam.reversed,
      geometry: { kind: "polyline", pointsM: sampled.length >= 2 ? sampled : fallback },
      workFrameId: payload.weldSequence.frame,
      processParameterId: "process-default",
    };
  });
  let next = reviseProject(project, "cad", (draft) => {
    draft.cad = {
      kind: payload.filename === "dog_box.step" ? "example" : "step",
      filename: payload.filename,
      fingerprint: payload.fingerprint,
      sourceUnit: payload.sourceUnit,
      topologyModelId: payload.modelId,
    };
    draft.seams = seams;
    draft.sequence = sequenceFromSeams(seams);
    seedImportTolerance(draft);
    draft.rows = rowsFromSeams(draft);
  }, timestamp);
  next = installPreviewPlan(next, previewTrajectory(next.rows), timestamp);
  return next;
}

// The CAD import budget IS the wire-contract bound: the cap that used to sit
// here (64) was a Steam Deck demo-fixture constant, removed by the L27
// ruling (P2.11). Seams that cannot fit the contract bound are still imported
// disabled with the loud over-budget lint — never silently truncated.
export const ROBOT_V4_PROGRAM_ROW_BUDGET = ROBOT_V4_PROGRAM_MAX_ROWS;

export interface CadRowBudgetStatus {
  maxRows: number;
  requiredRows: number;
  importedRows: number;
  overBudgetSeamIds: string[];
  toleranceM: number;
}

/** Seams the CP-budget import deliberately imported without rows. */
export function deliberatelyRowlessSeamIDs(
  project: OfflineProgrammingProject,
): Set<string> {
  return new Set(cadRowBudgetStatus(project)?.overBudgetSeamIds ?? []);
}

export function cadRowBudgetStatus(project: OfflineProgrammingProject): CadRowBudgetStatus | null {
  const block = project.program.settings.gradient_move_v1;
  if (typeof block !== "object" || block === null || Array.isArray(block)) return null;
  const value = block as Record<string, unknown>;
  const ids = Array.isArray(value.over_budget_seam_ids)
    ? value.over_budget_seam_ids.filter((id): id is string => typeof id === "string")
    : [];
  const required = Number(value.required_row_count);
  const imported = Number(value.imported_row_count);
  return {
    maxRows: ROBOT_V4_PROGRAM_ROW_BUDGET,
    requiredRows: Number.isInteger(required) && required >= 0 ? required : project.rows.length,
    importedRows: Number.isInteger(imported) && imported >= 0 ? imported : project.rows.length,
    overBudgetSeamIds: ids,
    toleranceM: moveToleranceM(project),
  };
}

export function rowsFromSeams(project: OfflineProgrammingProject): AuthoredProgramRow[] {
  return generateBudgetedRows(project, true).rows;
}

/**
 * Recompute the CP-budget bookkeeping (and over-budget seam disabling) from
 * the current seams/sequence without touching the authored rows. Used after a
 * weld deletion, where removing rows must not silently regenerate the rest of
 * the program but the over-budget lint must reflect the freed budget.
 */
export function refreshCadRowBudgetStatus(project: OfflineProgrammingProject): CadRowBudgetStatus {
  // The over-budget marker records exactly the seams the SYSTEM disabled for
  // budget. When a refresh frees one, hand it back enabled — an operator
  // disable was never in the marker, so it stays untouched.
  const systemDisabled = new Set(cadRowBudgetStatus(project)?.overBudgetSeamIds ?? []);
  let result = generateBudgetedRows(project, true);
  const recovered = project.seams.filter((seam) =>
    systemDisabled.has(seam.id) &&
    !result.status.overBudgetSeamIds.includes(seam.id) &&
    !seam.enabled
  );
  if (recovered.length > 0) {
    for (const seam of recovered) seam.enabled = true;
    result = generateBudgetedRows(project, true);
  }
  return result.status;
}

/** Metre trim offsets for a normalized [start, end] window along a polyline. */
export function seamTrimFromWindow(
  points: readonly Point3[],
  window: { start: number; end: number },
): AuthoredSeam["trim"] {
  return recommendationTrimM(points, window);
}

/** Sanitized id fragment shared by seam/move id construction. */
export function safeSeamIdFragment(value: string): string {
  return safeId(value);
}

function generateBudgetedRows(
  project: OfflineProgrammingProject,
  updateSettings: boolean,
): { rows: AuthoredProgramRow[]; status: CadRowBudgetStatus } {
  const toleranceM = moveToleranceM(project);
  const rows: AuthoredProgramRow[] = [];
  const overBudgetSeamIds: string[] = [];
  let requiredRows = 0;
  let previousEmittedOrientation: Rpy | null = null;
  for (const step of [...project.sequence].sort((left, right) => left.order - right.order)) {
    const seam = project.seams.find((candidate) => candidate.id === step.seamId);
    if (!seam || seam.geometry.pointsM.length < 2) continue;
    const generated = rowsForSeam(project, seam, step, toleranceM, previousEmittedOrientation);
    requiredRows += generated.rows.length;
    // A seam that cannot fit is imported disabled and emits nothing: no silent
    // truncation of a seam, and lint raises the over-budget error. A seam the
    // user disabled still emits its (disabled) rows so the P2.2 rule that
    // disabled CAD paths stay editable spreadsheet rows keeps holding — those
    // rows are on the wire, so they keep consuming the budget.
    if (rows.length + generated.rows.length > ROBOT_V4_PROGRAM_ROW_BUDGET) {
      seam.enabled = false;
      overBudgetSeamIds.push(seam.id);
      continue;
    }
    rows.push(...generated.rows);
    previousEmittedOrientation = generated.lastOrientation;
  }
  const status: CadRowBudgetStatus = {
    maxRows: ROBOT_V4_PROGRAM_ROW_BUDGET,
    requiredRows,
    importedRows: rows.length,
    overBudgetSeamIds,
    toleranceM,
  };
  if (updateSettings) setCadRowBudgetStatus(project, status);
  return { rows, status };
}

export function rowsForSeam(
  project: OfflineProgrammingProject,
  seam: AuthoredSeam,
  step: SequenceStep,
  toleranceM: number,
  previousEmittedOrientation: Rpy | null,
): { rows: AuthoredProgramRow[]; lastOrientation: Rpy } {
  const process = project.processParameters.find((candidate) =>
    candidate.id === seam.processParameterId
  ) ?? project.processParameters[0]!;
  const sourcePoints = authoredSeamPoints(seam);
  const controlIndices = controlPointIndicesForPath(
    sourcePoints,
    toleranceM,
    isCurveGeometry(seam.geometryType),
  );
  const controlSourcePoints = controlIndices.map((index) => sourcePoints[index]!);
  const sourceStart = controlSourcePoints[0]!;
  const sourceEnd = controlSourcePoints.at(-1)!;
  const points = controlSourcePoints.map((point) =>
    worldPointToPlanner(project, transformWorkpiecePoint(project, point))
  );
  const approach = worldPointToPlanner(project, transformWorkpiecePoint(project, [
    sourceStart[0],
    sourceStart[1],
    sourceStart[2] + step.approachOffsetM,
  ]));
  const retract = worldPointToPlanner(project, transformWorkpiecePoint(project, [
    sourceEnd[0],
    sourceEnd[1],
    sourceEnd[2] + step.retractOffsetM,
  ]));
  const placeholderOrientation = workpieceWorldRotationRpy(project);
  const torch = processTorchDefaults(process);
  const weldOrientations = points.map((point, pointIndex) => {
    const orientation = torchOrientationRpy(
      seamTravelDirection(points, pointIndex, approach, retract),
      torch,
    ) ?? placeholderOrientation;
    const continuous = unwrapRpy(orientation, previousEmittedOrientation);
    previousEmittedOrientation = continuous;
    return continuous;
  });
  const approachOrientation = weldOrientations[0] ?? placeholderOrientation;
  const retractOrientation = weldOrientations.at(-1) ?? placeholderOrientation;
  const rows = [
    makeRow(project, seam, "approach", approach, false, process.approachSpeedMps, 0, approachOrientation, torch),
    ...points.map((point, pointIndex) =>
      makeRow(
        project,
        seam,
        "weld",
        point,
        true,
        process.travelSpeedMps,
        pointIndex,
        weldOrientations[pointIndex] ?? placeholderOrientation,
        torch,
      )
    ),
    makeRow(project, seam, "retract", retract, false, process.retractSpeedMps, 0, retractOrientation, torch),
  ];
  // A CURVE's via handle must be a source point STRICTLY between its two
  // control points. Adjacent control points have no interior sample to offer,
  // and at the configured tolerance the wire chord between them is straight
  // anyway, so those segments stay LINE rather than carrying a degenerate
  // handle sitting on top of the start control point.
  const curveCenters = isCurveGeometry(seam.geometryType)
    ? controlIndices.slice(0, -1).map((startIndex, segmentIndex) => {
      const endIndex = controlIndices[segmentIndex + 1]!;
      const viaIndex = Math.floor((startIndex + endIndex) / 2);
      return viaIndex > startIndex && viaIndex < endIndex
        ? worldPointToPlanner(
          project,
          transformWorkpiecePoint(project, sourcePoints[viaIndex]!),
        )
        : null;
    })
    : [];
  annotateCadSeamMoves(rows, 0, rows.length - 1, seam.id, toleranceM, curveCenters);
  return { rows, lastOrientation: retractOrientation };
}

export function annotateCadSeamMoves(
  rows: AuthoredProgramRow[],
  startIndex: number,
  endIndex: number,
  seamId: string,
  toleranceM: number,
  curveCenters: readonly (Vec3 | null)[],
): void {
  if (endIndex - startIndex < 2) return;
  const weldStart = startIndex + 1;
  const weldEnd = endIndex - 1;
  const segments: MoveSegmentInput[] = [{ startIndex, endIndex: weldStart, kind: "LINE", toleranceM }];
  // Every emitted weld row is a tolerance-derived control point, so each
  // adjacent pair becomes its own move: a CAD import never hides one of its
  // own control points behind a sample disclosure.
  for (let index = weldStart; index < weldEnd; index += 1) {
    const centerM = curveCenters[index - weldStart] ?? null;
    segments.push(centerM
      ? { startIndex: index, endIndex: index + 1, kind: "CURVE", centerM, toleranceM }
      : { startIndex: index, endIndex: index + 1, kind: "LINE", toleranceM });
  }
  segments.push({ startIndex: weldEnd, endIndex, kind: "LINE", toleranceM });
  for (const segment of segments) {
    for (let index = segment.startIndex + 1; index <= segment.endIndex; index += 1) {
      rows[index]!.moveType = segment.kind;
    }
  }
  attachMoveMetadata(rows, segments, `cad:${safeId(seamId)}`);
}

function setCadRowBudgetStatus(project: OfflineProgrammingProject, status: CadRowBudgetStatus): void {
  const current = project.program.settings.gradient_move_v1;
  project.program.settings.gradient_move_v1 = {
    ...(typeof current === "object" && current !== null && !Array.isArray(current) ? current : {}),
    import_tolerance_m: status.toleranceM,
    max_row_count: status.maxRows,
    required_row_count: status.requiredRows,
    imported_row_count: status.importedRows,
    over_budget_seam_ids: [...status.overBudgetSeamIds],
  };
}

function isCurveGeometry(geometryType: string): boolean {
  return /circle|arc|ellipse|spline|bspline|bezier|curve/i.test(geometryType);
}

/**
 * Seed the chord-deviation tolerance without overwriting one the project
 * already carries. "Coarsen the tolerance" is the first recovery action the
 * over-budget lint error offers, so a re-import must honour the coarser value
 * rather than snapping it back to the 0.1 mm default.
 */
function seedImportTolerance(project: OfflineProgrammingProject): void {
  const current = project.program.settings.gradient_move_v1;
  project.program.settings.gradient_move_v1 = {
    ...(typeof current === "object" && current !== null && !Array.isArray(current) ? current : {}),
    import_tolerance_m: moveToleranceM(project),
  };
}

export function moveToleranceM(project: OfflineProgrammingProject): number {
  const block = project.program.settings.gradient_move_v1;
  const value = typeof block === "object" && block !== null && !Array.isArray(block)
    ? Number((block as Record<string, unknown>).import_tolerance_m)
    : Number.NaN;
  return Number.isFinite(value) && value > 0 ? value : DEFAULT_MOVE_TOLERANCE_M;
}

export function ensureCadPathRows(project: OfflineProgrammingProject): boolean {
  const existingSeamIDs = new Set(
    project.rows.flatMap((row) => row.seamId ? [row.seamId] : []),
  );
  const orderedMissingSeamIDs = [...project.sequence]
    .sort((left, right) => left.order - right.order)
    .map((step) => step.seamId)
    .filter((seamID) =>
      !existingSeamIDs.has(seamID) &&
      // A seam the CP-budget import deliberately left rowless (recorded in
      // over_budget_seam_ids) is a valid saved state, not damage to repair —
      // restoring it at load mutated saved programs and staled their plans
      // (OF-1). Genuinely damaged legacy seams are still restored.
      !deliberatelyRowlessSeamIDs(project).has(seamID) &&
      project.seams.some((seam) =>
        seam.id === seamID && seam.geometry.pointsM.length >= 2
      )
    );
  if (orderedMissingSeamIDs.length === 0) return false;

  const generated = rowsFromSeams(project);
  const onlyRegenerableCadRows = project.rows.every((row) =>
    row.sourceState === "cad_derived" && row.source === "cadquery"
  );
  if (onlyRegenerableCadRows) {
    project.rows = generated;
  } else {
    const orderBySeamID = new Map(
      [...project.sequence]
        .sort((left, right) => left.order - right.order)
        .map((step, index) => [step.seamId, index]),
    );
    for (const seamID of orderedMissingSeamIDs) {
      const additions = generated.filter((row) => row.seamId === seamID);
      if (additions.length === 0) continue;
      const seamOrder = orderBySeamID.get(seamID) ?? Number.MAX_SAFE_INTEGER;
      let insertionIndex = project.rows.findIndex((row) =>
        row.seamId !== null &&
        (orderBySeamID.get(row.seamId) ?? Number.MAX_SAFE_INTEGER) > seamOrder
      );
      if (insertionIndex < 0) {
        let lastEarlierIndex = -1;
        project.rows.forEach((row, index) => {
          if (
            row.seamId !== null &&
            (orderBySeamID.get(row.seamId) ?? Number.MAX_SAFE_INTEGER) < seamOrder
          ) {
            lastEarlierIndex = index;
          }
        });
        insertionIndex = lastEarlierIndex >= 0
          ? lastEarlierIndex + 1
          : project.rows.length;
      }
      project.rows.splice(insertionIndex, 0, ...additions);
    }
  }
  if (!project.rows.some((row) => row.id === project.simulation.selectedRowId)) {
    project.simulation.selectedRowId = project.rows[0]?.id ?? null;
  }
  return true;
}

export function installBrowserFallbackPreview(
  project: OfflineProgrammingProject,
  timestamp?: string,
): OfflineProgrammingProject {
  const authored = project.seams.length > 0 && project.sequence.length > 0
    ? reviseProject(project, "rows", (draft) => {
      draft.rows = rowsFromSeams(draft);
    }, timestamp)
    : project;
  return installPreviewPlan(
    authored,
    previewTrajectory(authored.rows),
    timestamp,
  );
}

export function authoredSeamPoints(seam: AuthoredSeam): Point3[] {
  let points = trimPolylineByOffsetsM(
    seam.geometry.pointsM,
    seam.trim.startOffsetM,
    seam.trim.endOffsetM,
  );
  if (seam.reversed) points.reverse();
  return points;
}

function motionRows(
  project: OfflineProgrammingProject,
  payload: CadSequenceResult,
): AuthoredProgramRow[] {
  const samples = payload.tesseractPlan?.motionPlan?.trajectory ?? [];
  if (samples.length === 0) return rowsFromSeams(project);
  const process = project.processParameters[0]!;
  return samples.map((sample, index) => {
    const phase = normalizePhase(sample.move);
    const seam = project.seams.find((candidate) => candidate.id === sample.seamId) ?? null;
    const pose = samplePose(sample);
    return {
      ...makeRow(
        project,
        seam,
        phase,
        copyPoint(sample.targetM),
        sample.weldOn,
        phase === "weld" ? process.travelSpeedMps : process.approachSpeedMps,
        index,
      ),
      targetTcpPose: pose,
      targetJointsRad: canonicalJoints(sample.jointPositionsRad, sample.jointNames),
    };
  });
}

function motionTrajectory(
  rows: AuthoredProgramRow[],
  payload: CadSequenceResult,
): PlanTrajectorySample[] {
  const samples = payload.tesseractPlan?.motionPlan?.trajectory ?? [];
  if (samples.length === 0) {
    return rows.map((row, index) => ({
      index,
      rowIndex: index,
      seamId: row.seamId,
      phase: row.phase,
      weldOn: row.weldEnabled,
      timeS: index * 0.18,
      tcpPose: [...row.targetTcpPose],
      jointPositionsRad: [...row.targetJointsRad],
    }));
  }
  return samples.map((sample, index) => {
    const row = rows[index] ?? rows.at(-1);
    return {
      index,
      rowIndex: index,
      seamId: sample.seamId ?? row?.seamId ?? null,
      phase: normalizePhase(sample.move),
      weldOn: sample.weldOn,
      timeS: finite(sample.timeS, index * 0.18),
      tcpPose: samplePose(sample),
      jointPositionsRad: canonicalJoints(sample.jointPositionsRad, sample.jointNames),
    };
  });
}

function sequenceFromSeams(seams: AuthoredSeam[]): SequenceStep[] {
  return seams.map((seam, index) => ({
    id: `sequence-${index + 1}`,
    seamId: seam.id,
    order: index,
    approachOffsetM: 0.05,
    retractOffsetM: 0.05,
  }));
}

export function makeRow(
  project: OfflineProgrammingProject,
  seam: AuthoredSeam | null,
  phase: RowPhase,
  xyz: Point3,
  weldEnabled: boolean,
  velocity: number,
  index = 0,
  orientationRpyRad: Rpy = [0, 0, 0],
  inheritedTorch = processTorchDefaults(
    project.processParameters.find((candidate) => candidate.id === seam?.processParameterId)
      ?? project.processParameters[0],
  ),
): AuthoredProgramRow {
  const process = project.processParameters.find((candidate) => candidate.id === seam?.processParameterId)
    ?? project.processParameters[0]!;
  const torchDefaults = inheritedTorch;
  const id = projectId();
  const programRevision = project.program.programRevision + 1;
  const targetTcpPose: TcpPose = [
    xyz[0],
    xyz[1],
    xyz[2],
    orientationRpyRad[0],
    orientationRpyRad[1],
    orientationRpyRad[2],
  ];
  const targetJointsRad = zeroJoints();
  return {
    id,
    enabled: seam?.enabled ?? true,
    label: `${seam?.label ?? "TRAVEL"} · ${phase.toUpperCase()} ${index + 1}`,
    rowType: "motion",
    rowRevision: programRevision,
    rowHash: `offline:${id}:${programRevision}`,
    command: "MOVE",
    comment: "",
    loopTargetRowId: null,
    loopIterations: 0,
    returnHomeBeforeRepeat: false,
    moveType: "LINE",
    phase,
    targetFrameType: "tcp_cartesian",
    fixtureSide: "in_cell",
    calibrationSection: 0,
    harmoniousPhaseDeg: 0,
    captureSource: "cadquery",
    speedSource: "programmed",
    seamId: seam?.id ?? null,
    passId: seam ? `${seam.id}:pass-1` : "",
    passRole: seam ? "root" : "none",
    // Motion Server's current FK/constraint frame chain terminates at
    // end_effector/tool0. This frame owns the local +X torch axis used by RPY.
    toolFrameId: "end_effector",
    workFrameId: "world",
    fixtureId: project.fixtures[0]?.id ?? null,
    targetTcpPose,
    targetJointsRad,
    positionerTargetsRad: [0, 0, 0],
    positionerJointTargetsRad: zeroJoints(),
    nominalTcpM: null,
    currentTcpPose: [...targetTcpPose],
    currentJointsRad: [...targetJointsRad],
    tcpVelocityMps: velocity,
    jointVelocityRadS: 0.25,
    accelerationMps2: process.accelerationMps2,
    jointAccelerationRadS2: 1,
    blendRadiusM: process.blendRadiusM,
    fineStop: process.blendRadiusM === 0,
    weldEnabled,
    arcState: weldEnabled ? "on" : "off",
    wireFeedMPerMin: process.wireFeedMPerMin,
    voltageV: process.voltageV,
    amperageA: process.amperageA,
    weldProfileId: process.weldProfileId,
    weaveProfileId: process.weaveProfileId,
    weldProcess: process.process,
    torchWorkAngleDeg: torchDefaults.workAngleDeg,
    torchTravelAngleDeg: torchDefaults.travelAngleDeg,
    torchSpinAngleDeg: torchDefaults.spinAngleDeg,
    weavePattern: weldEnabled ? torchDefaults.pattern : "none",
    weaveAmplitudeMm: torchDefaults.amplitudeMm,
    weaveFrequencyHz: torchDefaults.frequencyHz,
    weaveDwellMs: torchDefaults.dwellMs,
    revPiIOSlot: process.revPiIOSlot,
    revPiIOAction: weldEnabled ? "weld_on" : "weld_off",
    revPiIOPending: false,
    plannerStatus: "not_planned",
    validationState: "unknown",
    collisionCheckState: "not_run",
    limitCheckState: "not_run",
    plannedPathRevision: 0,
    plannedSampleCount: 0,
    sourceState: seam ? "cad_derived" : "manual",
    source: "cadquery",
    extensions: {},
  };
}

function seamTravelDirection(
  points: readonly Point3[],
  index: number,
  approach?: Point3,
  retract?: Point3,
): Point3 {
  if (points.length === 1 && approach && retract) {
    return [
      retract[0] - approach[0],
      retract[1] - approach[1],
      retract[2] - approach[2],
    ];
  }
  const left = index > 0 ? points[index - 1] : points[index];
  const right = index + 1 < points.length ? points[index + 1] : points[index];
  if (!left || !right) return [0, 0, 0];
  return [
    right[0] - left[0],
    right[1] - left[1],
    right[2] - left[2],
  ];
}

function unwrapRpy(rpy: Rpy, previous: Rpy | null): Rpy {
  if (!previous) return [...rpy];
  return rpy.map((component, index) => {
    const turns = Math.round((previous[index]! - component) / (Math.PI * 2));
    const unwrapped = component + turns * Math.PI * 2;
    return Math.abs(unwrapped) < 1e-12 ? 0 : unwrapped;
  }) as Rpy;
}

function samplePose(sample: CadMotionSample): TcpPose {
  return [
    finite(sample.targetM[0]),
    finite(sample.targetM[1]),
    finite(sample.targetM[2]),
    finite(sample.orientationRpyRad[0]),
    finite(sample.orientationRpyRad[1]),
    finite(sample.orientationRpyRad[2]),
  ];
}

function recommendationTrimM(
  points: readonly Point3[],
  window: { start: number; end: number },
): AuthoredSeam["trim"] {
  const totalLengthM = polylineLengthM(points);
  const start = clamp01(window.start);
  const end = Math.max(start, clamp01(window.end));
  return {
    startOffsetM: start * totalLengthM,
    endOffsetM: (1 - end) * totalLengthM,
  };
}

function copyPoint(point: readonly [number, number, number]): Point3 {
  return [finite(point[0]), finite(point[1]), finite(point[2])];
}

function normalizePhase(value: unknown): RowPhase {
  return ["approach", "weld", "retract", "travel"].includes(String(value))
    ? String(value) as RowPhase
    : "travel";
}

function finite(value: unknown, fallback = 0): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function clamp01(value: number): number {
  return Math.max(0, Math.min(1, finite(value)));
}

function safeId(value: string): string {
  return value.replace(/[^a-zA-Z0-9_-]+/g, "-");
}

export type { CadSequenceResult, CadTopology, CadTopologyEdge };
