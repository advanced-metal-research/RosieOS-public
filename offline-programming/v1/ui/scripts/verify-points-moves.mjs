/**
 * Live points & moves proof against the running offline-programming API server
 * (127.0.0.1:8794, CadQuery wired). Read-only use of the server: one topology
 * extraction per tolerance, no restart, no writes.
 *
 *   node --experimental-strip-types scripts/verify-points-moves.mjs
 *
 * Receipts produced, per the P2.8/P2.9 brief step 4:
 *   - visible spreadsheet entry count vs the dense samples-as-rows baseline
 *   - wire row count against the Robot V4 wire budget (the 4096-row contract
 *     backstop since P2.11), or the loud over-budget lint error engaging
 *   - worst-case chord error between the shipped control points and the dense
 *     CAD polyline the planner is approximating with straight segments
 *   - one curve move's regenerated samples against the three-point arc math
 */
import { HttpCadClient } from "../src/cad/client.ts";
import { DOG_BOX_FIXTURE } from "../src/cad/contracts.ts";
import {
  ROBOT_V4_PROGRAM_ROW_BUDGET,
  adaptCadQueryTopology,
  authoredSeamPoints,
  cadRowBudgetStatus,
} from "../src/adapters/cadquery.ts";
import { toRobotV4Program } from "../src/adapters/robot-v4.ts";
import { createProject } from "../src/domain/project.ts";
import { transformWorkpiecePoint, worldPointToPlanner } from "../src/domain/placement.ts";
import {
  DEFAULT_MOVE_TOLERANCE_M,
  attachMoveMetadata,
  buildProgramView,
  interpolateThreePointArc,
  regenerateCurveMove,
} from "../src/domain/points-moves.ts";
import { lintProgram } from "../src/domain/weld-quality.ts";

const endpoint = "http://127.0.0.1:8794/api/offline-programming/v1/cadquery/topology";
const topology = await new HttpCadClient({ endpoint })
  .extractTopology({ source: DOG_BOX_FIXTURE, sampleCount: 64 });

// What samples-as-rows costs: every dense CAD sample plus approach/retract.
const denseBaselineRows = topology.recommendedSeams.reduce((total, seam) => {
  const edge = topology.edges.find((candidate) => candidate.id === seam.edgeId);
  return total + (edge ? edge.samplesM.length + 2 : 0);
}, 0);

const report = {
  server: endpoint,
  fingerprint: topology.fingerprint,
  topologyEdges: topology.edges.length,
  recommendedSeams: topology.recommendedSeams.length,
  denseSamplesAsRowsBaseline: denseBaselineRows,
  rowBudget: ROBOT_V4_PROGRAM_ROW_BUDGET,
  tolerances: [DEFAULT_MOVE_TOLERANCE_M, 0.001, 0.01, 0.05].map(importAt),
};
// Prove the curve math at the coarsest tolerance that still yields a CAD curve
// move: at 0.1 mm the Dog Box circles are sampled coarser than the tolerance,
// so every sample is its own control point and every move is honestly a LINE.
const curveTolerance = report.tolerances.find((entry) => entry.curveMovesWithCenter > 0);
report.curveArcMath = curveTolerance
  ? curveArcMathProof(curveTolerance.toleranceM)
  : "no tolerance in the sweep produced a CAD-seeded curve move";
console.log(JSON.stringify(report, null, 2));

/** Import the live topology at one tolerance and measure what shipped. */
function importAt(toleranceM) {
  const project = adaptCadQueryTopology(
    withTolerance(createProject({ name: `Dog Box @ ${toleranceM} m` }), toleranceM),
    topology,
  );
  const view = buildProgramView(project);
  const budget = cadRowBudgetStatus(project);
  const wire = toRobotV4Program(project);
  const overBudgetLint = lintProgram(project)
    .filter((finding) => finding.code === "program_row_budget_exceeded")
    .map((finding) => `${finding.severity}: ${finding.message}`);

  let worstChordErrorM = 0;
  let worstChordSeamId = null;
  let worstChordLengthM = 0;
  for (const seam of project.seams) {
    const shipped = project.rows.filter((row) =>
      row.seamId === seam.id && row.phase === "weld"
    );
    if (shipped.length < 2) continue;
    // The planner interpolates linearly between rows, so the geometric error
    // is the distance from each dense CAD sample to the shipped CP polyline.
    const dense = authoredSeamPoints(seam).map((point) =>
      worldPointToPlanner(project, transformWorkpiecePoint(project, point)).slice(0, 3)
    );
    const cps = shipped.map((row) => row.targetTcpPose.slice(0, 3));
    for (const point of dense) {
      let best = Infinity;
      for (let index = 1; index < cps.length; index += 1) {
        best = Math.min(best, pointToSegmentM(point, cps[index - 1], cps[index]));
      }
      if (best > worstChordErrorM) {
        worstChordErrorM = best;
        worstChordSeamId = seam.id;
      }
    }
    for (let index = 1; index < cps.length; index += 1) {
      worstChordLengthM = Math.max(worstChordLengthM, distanceM(cps[index - 1], cps[index]));
    }
  }

  const curves = view.moves.filter((move) => move.kind === "CURVE" && move.centerM);
  return {
    toleranceM,
    wireRows: wire.rows.length,
    withinRowBudget: wire.rows.length <= ROBOT_V4_PROGRAM_ROW_BUDGET,
    requiredRows: budget?.requiredRows ?? null,
    importedRows: budget?.importedRows ?? null,
    seamsEnabled: project.seams.filter((seam) => seam.enabled).length,
    seamsOverBudgetDisabled: budget?.overBudgetSeamIds.length ?? 0,
    overBudgetLint,
    visibleEntries: view.entries.length,
    controlPoints: view.controls.length,
    moves: view.moves.length,
    curveMovesWithCenter: curves.length,
    lineMoves: view.moves.filter((move) => move.kind === "LINE").length,
    hiddenSampleRows: view.moves.reduce((total, move) => total + move.sampleRows.length, 0),
    denseSeamPointsRetained: project.seams.reduce(
      (total, seam) => total + seam.geometry.pointsM.length,
      0,
    ),
    degenerateCurveCenters: curves.filter((move) =>
      distanceM(move.centerM, move.start.row.targetTcpPose.slice(0, 3)) <= 1e-9 ||
      distanceM(move.centerM, move.end.row.targetTcpPose.slice(0, 3)) <= 1e-9
    ).length,
    worstChordErrorM,
    worstChordErrorSeamId: worstChordSeamId,
    worstChordLengthM,
  };
}

/**
 * One live CAD curve move, densified into the sample-bearing shape a catalog
 * program would hold, then re-centred: every regenerated row must equal the
 * pinned three-point arc port, and the CAD-seeded arc must reproduce the dense
 * CadQuery polyline it came from.
 */
function curveArcMathProof(toleranceM) {
  const project = adaptCadQueryTopology(
    withTolerance(createProject({ name: "Dog Box curve proof" }), toleranceM),
    topology,
  );
  const move = buildProgramView(project).moves.find((candidate) =>
    candidate.kind === "CURVE" && candidate.centerM && !candidate.modifiedExternally
  );
  if (!move?.centerM) throw new Error("live import produced no editable curve move");

  const startM = move.start.row.targetTcpPose.slice(0, 3);
  const endM = move.end.row.targetTcpPose.slice(0, 3);
  const seam = project.seams.find((candidate) => candidate.id === move.end.row.seamId);
  const dense = authoredSeamPoints(seam).map((point) =>
    worldPointToPlanner(project, transformWorkpiecePoint(project, point)).slice(0, 3)
  );
  const spanStart = nearestIndex(dense, startM);
  const spanEnd = nearestIndex(dense, endM);
  const span = dense.slice(Math.min(spanStart, spanEnd), Math.max(spanStart, spanEnd) + 1);
  // The CAD-seeded arc versus the dense CadQuery samples it interpolates.
  const seeded = interpolateThreePointArc([startM, move.centerM, endM], span.length);
  const cadFitErrorM = seeded.length === span.length
    ? Math.max(...seeded.map((point, index) => distanceM(point, span[index])))
    : null;

  // Densify the move into the sample-bearing shape a catalog program holds.
  // Kept as a standalone program of just this move's rows so the densified
  // proof program itself still respects the wire row budget.
  const densified = structuredClone(project);
  const sampleCount = Math.min(span.length, 24);
  const interior = Array.from({ length: sampleCount - 2 }, (_, index) =>
    span[Math.round((index + 1) * (span.length - 1) / (sampleCount - 1))]
  );
  const startRow = structuredClone(move.start.row);
  const endRow = structuredClone(move.end.row);
  densified.rows = [
    startRow,
    ...interior.map((point, index) => {
      const row = structuredClone(endRow);
      row.id = `${endRow.id}:sample-${index + 1}`;
      row.rowHash = `offline:${row.id}:${row.rowRevision}`;
      row.targetTcpPose = [...point, ...endRow.targetTcpPose.slice(3)];
      return row;
    }),
    endRow,
  ];
  densified.simulation.selectedRowId = endRow.id;
  attachMoveMetadata(
    densified.rows,
    [{
      startIndex: 0,
      endIndex: densified.rows.length - 1,
      kind: "CURVE",
      centerM: move.centerM,
      moveId: move.id,
      toleranceM,
    }],
    "proof",
  );

  const editedCenter = [move.centerM[0], move.centerM[1], move.centerM[2] + 0.002];
  const after = regenerateCurveMove(densified, move.id, editedCenter);
  const regenerated = buildProgramView(after).moveById.get(move.id);
  const expected = interpolateThreePointArc([startM, editedCenter, endM], sampleCount);
  const rows = regenerated.rows.map((row) => row.targetTcpPose.slice(0, 3));
  const maxArcMathErrorM = rows.length === expected.length
    ? Math.max(...rows.map((point, index) => distanceM(point, expected[index])))
    : null;

  return {
    toleranceM,
    moveId: move.id,
    seamId: seam.id,
    controlPointStartM: startM,
    cadSeededCenterM: move.centerM,
    centerIsDistinctFromControlPoints:
      distanceM(move.centerM, startM) > 1e-9 && distanceM(move.centerM, endM) > 1e-9,
    denseSamplesSpanned: sampleCount,
    cadSeededArcVsDensePolylineM: cadFitErrorM,
    editedCenterM: editedCenter,
    regeneratedRowCount: rows.length,
    maxArcMathErrorM,
    endpointsPinned:
      distanceM(rows[0], startM) <= 1e-12 &&
      distanceM(rows[rows.length - 1], endM) <= 1e-12,
    wireRowsAfterEdit: toRobotV4Program(after).rows.length,
    programStateAfterEdit: after.program.programState,
    torchOrientationsStillFinite: regenerated.rows.every((row) =>
      row.targetTcpPose.slice(3).every((value) => Number.isFinite(value))
    ),
  };
}

function withTolerance(project, toleranceM) {
  project.program.settings.gradient_move_v1 = { import_tolerance_m: toleranceM };
  return project;
}

function distanceM(a, b) {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function nearestIndex(points, target) {
  let best = 0;
  let bestDistance = Infinity;
  points.forEach((point, index) => {
    const distance = distanceM(point, target);
    if (distance < bestDistance) {
      bestDistance = distance;
      best = index;
    }
  });
  return best;
}

function pointToSegmentM(point, a, b) {
  const abx = b[0] - a[0];
  const aby = b[1] - a[1];
  const abz = b[2] - a[2];
  const lengthSquared = abx * abx + aby * aby + abz * abz;
  if (lengthSquared <= 1e-24) return distanceM(point, a);
  const t = Math.max(0, Math.min(1,
    ((point[0] - a[0]) * abx + (point[1] - a[1]) * aby + (point[2] - a[2]) * abz) /
      lengthSquared,
  ));
  return distanceM(point, [a[0] + abx * t, a[1] + aby * t, a[2] + abz * t]);
}
