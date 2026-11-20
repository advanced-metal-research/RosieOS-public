import assert from "node:assert/strict";
import test from "node:test";

import { fromRobotV4Program, toRobotV4Program } from "../adapters/robot-v4.ts";
import {
  ROBOT_V4_PROGRAM_ROW_BUDGET,
  adaptCadQueryTopology,
  cadRowBudgetStatus,
  rowsFromSeams,
} from "../adapters/cadquery.ts";
import { createWeldsFromRecommendations } from "../adapters/weld-authoring.ts";
import type { CadTopology } from "../cad/contracts.ts";
import {
  createProject,
  type OfflineProgrammingProject,
  type Vec3,
} from "./project.ts";
import {
  createBlankSpreadsheetProject,
  createSpreadsheetRow,
  updateSpreadsheetRows,
} from "./spreadsheet.ts";
import {
  DEFAULT_MOVE_TOLERANCE_M,
  attachMoveMetadata,
  buildProgramView,
  canReorderControlPoint,
  controlPointIndicesForPath,
  interpolateThreePointArc,
  maximumChordLengthForSagitta,
  moveMetadata,
  regenerateCurveMove,
  reorderAdjacentControlPoints,
  simplifyPolylineByChordDeviation,
} from "./points-moves.ts";

/**
 * P2.12: importing a topology yields only a pickable edge catalog. These
 * tests exercise the generated program, so they author welds from every
 * extractor recommendation — the explicit equivalent of the old auto-import.
 */
function authorRecommendedWelds(
  project: OfflineProgrammingProject,
  topology: CadTopology,
): OfflineProgrammingProject {
  return createWeldsFromRecommendations(adaptCadQueryTopology(project, topology));
}

test("grouping converts contiguous row metadata into CP and move entries while hiding samples", () => {
  const project = createBlankSpreadsheetProject({ name: "Grouped rows" });
  project.rows = Array.from({ length: 5 }, (_, index) => {
    const row = createSpreadsheetRow(project, "motion", `row-${index}`);
    row.targetTcpPose = [index * 0.01, 0, 0, 0, 0, 0];
    return row;
  });
  attachMoveMetadata(project.rows, [{ startIndex: 0, endIndex: 4, kind: "LINE" }]);

  const view = buildProgramView(project);
  assert.equal(view.moves.length, 1);
  assert.equal(view.moves[0]!.sampleRows.length, 3);
  assert.deepEqual(view.entries.map((entry) => entry.kind), ["control", "move", "control"]);
  assert.deepEqual(view.visibleRowIds, ["row-0", "row-4"]);
  assert.equal(view.moves[0]!.modifiedExternally, false);
});

test("straight simplification collapses collinear samples and honors the inclusive tolerance boundary", () => {
  const tolerance = DEFAULT_MOVE_TOLERANCE_M;
  const straight = Array.from({ length: 65 }, (_, index) =>
    [index / 64, 0, 0] as [number, number, number]
  );
  assert.deepEqual(simplifyPolylineByChordDeviation(straight, tolerance), [0, 64]);

  const onBoundary = [[0, 0, 0], [0.5, tolerance, 0], [1, 0, 0]] as const;
  assert.deepEqual(simplifyPolylineByChordDeviation(onBoundary, tolerance), [0, 2]);
  const outside = [[0, 0, 0], [0.5, tolerance + 1e-9, 0], [1, 0, 0]] as const;
  assert.deepEqual(simplifyPolylineByChordDeviation(outside, tolerance), [0, 1, 2]);
});

test("R50 mm curves derive 6.32 mm maximum chords at 0.1 mm sagitta", () => {
  const chordM = maximumChordLengthForSagitta(0.05, 0.0001);
  assert.ok(Math.abs(chordM - 0.006321392251711664) <= 1e-15);

  const arc = Array.from({ length: 501 }, (_, index) => {
    const angle = Math.PI * index / 500;
    return [0.05 * Math.cos(angle), 0.05 * Math.sin(angle), 0] as [number, number, number];
  });
  const indices = controlPointIndicesForPath(arc, 0.0001, true);
  let worstChordM = 0;
  for (let index = 1; index < indices.length; index += 1) {
    const left = arc[indices[index - 1]!]!;
    const right = arc[indices[index]!]!;
    worstChordM = Math.max(
      worstChordM,
      Math.hypot(
        right[0] - left[0],
        right[1] - left[1],
        right[2] - left[2],
      ),
    );
  }
  assert.ok(worstChordM <= chordM + 1e-12);
  assert.ok(worstChordM >= 0.006 && worstChordM < 0.007);
});

test("three-point arc port matches fixture values computed by the Python original", () => {
  const actual = interpolateThreePointArc(
    [[0.4, -0.2, 0.1], [0.5, -0.1, 0.2], [0.6, -0.2, 0.1]],
    7,
  );
  const python = [
    [0.4, -0.2, 0.09999999999999999],
    [0.3985789533539753, -0.15304810603586924, 0.14695189396413078],
    [0.4369237924991507, -0.1147032668906938, 0.18529673310930622],
    [0.5, -0.1, 0.2],
    [0.5630762075008492, -0.11470326689069377, 0.18529673310930625],
    [0.6014210466460247, -0.15304810603586924, 0.14695189396413078],
    [0.6, -0.2, 0.10000000000000002],
  ];
  assert.equal(actual.length, python.length);
  actual.forEach((point, index) => point.forEach((value, axis) => {
    assert.ok(Math.abs(value - python[index]![axis]!) <= 1e-14);
  }));

  assert.deepEqual(
    interpolateThreePointArc([[0, 0, 0], [0.5, 0, 0], [1, 0, 0]], 5),
    [[0, 0, 0], [0.25, 0, 0], [0.5, 0, 0], [0.75, 0, 0], [1, 0, 0]],
  );
});

test("editing a curve center regenerates existing sample rows and invalidates the plan", () => {
  const project = createBlankSpreadsheetProject({ name: "Curve edit" });
  project.rows = Array.from({ length: 7 }, (_, index) => {
    const row = createSpreadsheetRow(project, "motion", `curve-${index}`);
    row.targetTcpPose = [index / 6, 0, 0, 0, 0, 0];
    row.phase = "weld";
    row.seamId = "curve-seam";
    row.weldEnabled = true;
    return row;
  });
  attachMoveMetadata(project.rows, [{
    startIndex: 0,
    endIndex: 6,
    kind: "CURVE",
    centerM: [0.5, 0.5, 0],
  }]);
  const moveId = moveMetadata(project.rows[6]!)!.moveId!;
  const ids = project.rows.map((row) => row.id);
  const next = regenerateCurveMove(project, moveId, [0.5, 0.25, 0]);
  const expected = interpolateThreePointArc(
    [[0, 0, 0], [0.5, 0.25, 0], [1, 0, 0]],
    7,
  );

  assert.deepEqual(next.rows.map((row) => row.id), ids);
  next.rows.forEach((row, index) => assert.deepEqual(row.targetTcpPose.slice(0, 3), expected[index]));
  assert.equal(next.identity.revision, project.identity.revision + 1);
  assert.equal(next.program.programState, "needs_plan");
  assert.deepEqual(moveMetadata(next.rows[6]!)!.centerM, [0.5, 0.25, 0]);
});

test("curve regeneration refuses a teach_modified member without touching it", () => {
  const project = createBlankSpreadsheetProject({ name: "Taught curve" });
  project.rows = Array.from({ length: 5 }, (_, index) => {
    const row = createSpreadsheetRow(project, "motion", `taught-${index}`);
    row.targetTcpPose = [index / 4, 0, 0, 0, 0, 0];
    return row;
  });
  attachMoveMetadata(project.rows, [{
    startIndex: 0,
    endIndex: 4,
    kind: "CURVE",
    centerM: [0.5, 0.5, 0],
  }]);
  project.rows[2]!.sourceState = "teach_modified";
  const before = structuredClone(project.rows[2]);
  const moveId = moveMetadata(project.rows[4]!)!.moveId!;

  assert.throws(
    () => regenerateCurveMove(project, moveId, [0.5, 0.25, 0]),
    /teach-modified/i,
  );
  assert.deepEqual(project.rows[2], before);
});

test("manual CURVE selection creates a placeable center handle between adjacent CPs", () => {
  let project = createBlankSpreadsheetProject({ name: "Manual curve" });
  project.rows.push(createSpreadsheetRow(project, "motion", "manual-curve-start"));
  const second = createSpreadsheetRow(project, "motion", "manual-curve-end");
  second.targetTcpPose = [0.1, 0, 0, 0, 0, 0];
  project.rows.push(second);
  project = updateSpreadsheetRows(project, [second.id], (row) => {
    row.moveType = "CURVE";
  });
  const curve = buildProgramView(project).moves.find((move) =>
    move.end.row.id === second.id
  );
  assert.equal(curve?.kind, "CURVE");
  assert.ok(curve?.centerM);
  assert.equal(curve?.rows.length, 2);
});

test("catalog round-trip collapses intact moves and exposes externally edited samples", () => {
  const project = createBlankSpreadsheetProject({ name: "Catalog grouping" });
  project.rows = Array.from({ length: 5 }, (_, index) => {
    const row = createSpreadsheetRow(project, "motion", `catalog-${index}`);
    row.targetTcpPose = [index * 0.01, 0, 0, 0, 0, 0];
    return row;
  });
  attachMoveMetadata(project.rows, [{ startIndex: 0, endIndex: 4, kind: "LINE" }]);

  const wire = toRobotV4Program(project);
  const intact = fromRobotV4Program(wire, { timestamp: "2026-08-04T00:00:00Z" });
  assert.equal(buildProgramView(intact).moves[0]!.modifiedExternally, false);
  assert.deepEqual(buildProgramView(intact).entries.map((entry) => entry.kind), [
    "control", "move", "control",
  ]);

  const editedWire = structuredClone(wire);
  editedWire.rows[2]!.target_tcp_m![1] += 0.002;
  editedWire.rows[2]!.source_state = "teach_modified";
  const edited = fromRobotV4Program(editedWire, { timestamp: "2026-08-04T00:00:01Z" });
  const view = buildProgramView(edited);
  assert.equal(view.moves[0]!.modifiedExternally, true);
  assert.equal(view.entries.filter((entry) => entry.kind === "raw").length, 5);
  assert.deepEqual(view.visibleRowIds, edited.rows.map((row) => row.id));
});

test("CAD adapter keeps dense seam geometry but ships only tolerance-derived CP rows", () => {
  const arc = interpolateThreePointArc(
    [[0, 0, 0], [0.05, 0.05, 0], [0.1, 0, 0]],
    64,
  );
  const topology: CadTopology = {
    schemaVersion: 1,
    unit: "m",
    sourceUnit: "mm",
    tool: "test",
    modelId: "points-moves-cad",
    filename: "points.step",
    fingerprint: "a".repeat(64),
    sampleCount: 64,
    parts: [{ id: "part", edgeCount: 2 }],
    edges: [
      {
        id: "line",
        label: "LINE",
        partId: "part",
        geometryType: "LINE",
        sampleCount: 64,
        samplesM: Array.from({ length: 64 }, (_, index) =>
          [index / 630, 0, 0] as [number, number, number]
        ),
        lengthM: 0.1,
        boundsMinM: [0, 0, 0],
        boundsMaxM: [0.1, 0, 0],
        closed: false,
        sourceEdgeIds: [],
        sourceGeometryTypes: [],
        pathClass: "raw_fragment",
        isSeamCandidate: true,
      },
      {
        id: "circle",
        label: "ARC",
        partId: "part",
        geometryType: "CIRCLE",
        sampleCount: 64,
        samplesM: arc,
        lengthM: 0.157,
        boundsMinM: [0, 0, 0],
        boundsMaxM: [0.1, 0.05, 0],
        closed: false,
        sourceEdgeIds: [],
        sourceGeometryTypes: [],
        pathClass: "circle",
        isSeamCandidate: true,
      },
    ],
    rawEdgeCount: 2,
    compositeEdgeCount: 0,
    seamCandidateCount: 2,
    recommendedSeams: [
      {
        edgeId: "line",
        partId: "part",
        geometryType: "LINE",
        pathClass: "raw_fragment",
        window: { start: 0, end: 1 },
        reversed: false,
        reason: "test",
      },
      {
        edgeId: "circle",
        partId: "part",
        geometryType: "CIRCLE",
        pathClass: "circle",
        window: { start: 0, end: 1 },
        reversed: false,
        reason: "test",
      },
    ],
  };
  const project = authorRecommendedWelds(createProject(), topology);
  const view = buildProgramView(project);
  assert.equal(project.seams[0]!.geometry.pointsM.length, 64);
  assert.equal(project.seams[1]!.geometry.pointsM.length, 64);
  assert.ok(project.rows.length <= 64);
  assert.ok(project.rows.length < 132);
  assert.ok(view.moves.some((move) => move.kind === "LINE"));
  assert.ok(view.moves.some((move) => move.kind === "CURVE" && move.centerM !== null));
  assert.equal(
    (project.program.settings.gradient_move_v1 as Record<string, unknown>)
      .import_tolerance_m,
    DEFAULT_MOVE_TOLERANCE_M,
  );
});

test("an unrelated edit never heals an externally modified move", () => {
  const project = createBlankSpreadsheetProject({ name: "External edit" });
  project.rows = Array.from({ length: 6 }, (_, index) => {
    const row = createSpreadsheetRow(project, "motion", `outside-${index}`);
    row.targetTcpPose = [index * 0.01, 0, 0, 0, 0, 0];
    return row;
  });
  attachMoveMetadata(project.rows, [{ startIndex: 0, endIndex: 4, kind: "LINE" }]);

  // A peer that knows nothing about gradient_move_v1 nudges a hidden sample.
  // It does not mark the row teach_modified, so the sample hash is the only
  // thing that can catch it.
  const editedWire = structuredClone(toRobotV4Program(project));
  editedWire.rows[2]!.target_tcp_m![1] += 0.002;
  const edited = fromRobotV4Program(editedWire, { timestamp: "2026-08-04T00:00:00Z" });
  const move = buildProgramView(edited).moves[0]!;
  assert.equal(move.modifiedExternally, true);
  assert.equal(buildProgramView(edited).visibleRowIds.length, edited.rows.length);

  // Editing an unrelated row must not re-pin the flagged move's hash and
  // re-hide the peer's rows behind a collapsed move.
  const outside = edited.rows.at(-1)!;
  const after = updateSpreadsheetRows(edited, [outside.id], (row) => {
    row.label = "unrelated";
  });
  const afterView = buildProgramView(after);
  assert.equal(afterView.moveById.get(move.id)?.modifiedExternally, true);
  assert.equal(afterView.visibleRowIds.length, after.rows.length);
});

test("CAD curve vias sit strictly between control points and no CAD row is hidden", () => {
  // A 0.05 m arc sampled at 64 points is FINER than the 0.1 mm tolerance, so
  // control points span several source samples and each has a real interior
  // via to seed the handle from.
  const fine = authorRecommendedWelds(
    createProject(),
    arcTopology("fine", interpolateThreePointArc(
      [[0, 0, 0], [0.05, 0.05, 0], [0.1, 0, 0]],
      64,
    ) as Vec3[]),
  );
  const fineView = buildProgramView(fine);
  const curves = fineView.moves.filter((move) => move.kind === "CURVE");
  assert.ok(curves.length >= 2);
  for (const move of curves) {
    assert.ok(move.centerM);
    for (const cp of [move.start, move.end]) {
      const pose = cp.row.targetTcpPose;
      assert.ok(
        Math.hypot(...move.centerM.map((value, axis) => value - pose[axis]!)) > 1e-9,
        `move ${move.id} seeded its via on control point ${cp.row.id}`,
      );
    }
  }
  // Every emitted CAD row is a control point: a CAD import hides nothing.
  assert.deepEqual(fineView.visibleRowIds, fine.rows.map((row) => row.id));
  assert.equal(fineView.moves.every((move) => move.sampleRows.length === 0), true);

  // A 0.5 m arc sampled at 30 points is COARSER than the tolerance, so every
  // sample becomes its own control point. There is no interior via, and the
  // wire chord really is straight, so these must be LINE moves rather than
  // CURVEs whose centre collapses onto the start control point.
  const coarse = authorRecommendedWelds(
    createProject(),
    arcTopology("coarse", Array.from({ length: 30 }, (_, index) => {
      const angle = Math.PI * index / 2 / 29;
      return [0.5 * Math.cos(angle), 0.5 * Math.sin(angle), 0] as Vec3;
    })),
  );
  const coarseView = buildProgramView(coarse);
  assert.ok(coarse.rows.length >= 30);
  assert.equal(coarseView.moves.some((move) => move.kind === "CURVE"), false);
  assert.equal(coarseView.moves.every((move) => move.centerM === null), true);
  assert.deepEqual(coarseView.visibleRowIds, coarse.rows.map((row) => row.id));
});

test("a coarsened import tolerance survives re-import and reduces control points", () => {
  const samplesM = Array.from({ length: 40 }, (_, index) => {
    const angle = Math.PI * index / 2 / 39;
    return [0.3 * Math.cos(angle), 0.3 * Math.sin(angle), 0] as Vec3;
  });
  const topology = arcTopology("coarsen", samplesM);

  const atDefault = authorRecommendedWelds(createProject(), topology);
  assert.equal(
    (atDefault.program.settings.gradient_move_v1 as Record<string, unknown>).import_tolerance_m,
    DEFAULT_MOVE_TOLERANCE_M,
  );

  // "Coarsen tolerance" is the first recovery action the over-budget lint
  // offers, so a configured tolerance must not be reset by the import.
  const coarse = createProject();
  coarse.program.settings.gradient_move_v1 = { import_tolerance_m: 0.005 };
  const atCoarse = authorRecommendedWelds(coarse, topology);
  assert.equal(
    (atCoarse.program.settings.gradient_move_v1 as Record<string, unknown>).import_tolerance_m,
    0.005,
  );
  assert.ok(
    atCoarse.rows.length < atDefault.rows.length,
    `coarsening produced ${atCoarse.rows.length} rows, not fewer than ${atDefault.rows.length}`,
  );
  assert.equal(cadRowBudgetStatus(atCoarse)?.toleranceM, 0.005);
});

test("a user-disabled CAD seam keeps disabled rows and still consumes the row budget", () => {
  const project = authorRecommendedWelds(
    createProject(),
    arcTopology("disable-me", Array.from({ length: 8 }, (_, index) =>
      [index / 70, 0, 0] as Vec3), "LINE"),
  );
  const before = project.rows.length;
  assert.ok(before >= 3);

  project.seams[0]!.enabled = false;
  const rows = rowsFromSeams(project);
  // P2.2: a disabled CAD path stays an editable, disabled spreadsheet row set.
  assert.equal(rows.length, before);
  assert.equal(rows.every((row) => row.enabled === false), true);
  assert.ok(rows.some((row) => row.phase === "weld"));
  // Those rows are still on the wire, so they still cost budget, and nothing
  // is reported as over budget.
  const budget = cadRowBudgetStatus(project);
  assert.equal(budget?.requiredRows, before);
  assert.equal(budget?.importedRows, before);
  assert.deepEqual(budget?.overBudgetSeamIds, []);
});

test("CAD import disables whole over-budget seams and never truncates a seam", () => {
  // The fixture's only job is to ask for more rows than the contract
  // backstop. How many rows a seam becomes is chord simplification's
  // business (and the approach/retract rows around it), so this test pins
  // the CONTRACT -- nothing is truncated, whole seams are dropped, and the
  // drop is minimal -- rather than a seam count that happens to divide the
  // bound. Zig-zag samples defeat simplification, so a seam here is roughly
  // as many rows as it has samples.
  const SAMPLES = 126;
  const seamCount = Math.ceil(ROBOT_V4_PROGRAM_ROW_BUDGET / SAMPLES) + 3;
  const seams = Array.from({ length: seamCount }, (_, index) => ({
    edgeId: `line-${index}`,
    partId: "part",
    geometryType: "LINE",
    pathClass: "raw_fragment",
    window: { start: 0, end: 1 },
    reversed: false,
    reason: "budget fixture",
  }));
  const topology: CadTopology = {
    schemaVersion: 1,
    unit: "m",
    sourceUnit: "mm",
    tool: "test",
    modelId: "budget-cad",
    filename: "budget.step",
    fingerprint: "b".repeat(64),
    sampleCount: SAMPLES,
    parts: [{ id: "part", edgeCount: seams.length }],
    edges: seams.map((seam, index) => ({
      id: seam.edgeId,
      label: `LINE ${index + 1}`,
      partId: "part",
      geometryType: "LINE",
      sampleCount: SAMPLES,
      samplesM: Array.from({ length: SAMPLES }, (_, sample) =>
        [sample / 1000, index * 0.05 + (sample % 2 === 0 ? 0 : 0.01), 0] as [number, number, number]
      ),
      lengthM: SAMPLES / 1000,
      boundsMinM: [0, index * 0.05, 0] as [number, number, number],
      boundsMaxM: [SAMPLES / 1000, index * 0.05 + 0.01, 0] as [number, number, number],
      closed: false,
      sourceEdgeIds: [],
      sourceGeometryTypes: [],
      pathClass: "raw_fragment",
      isSeamCandidate: true,
    })),
    rawEdgeCount: seams.length,
    compositeEdgeCount: 0,
    seamCandidateCount: seams.length,
    recommendedSeams: seams,
  };
  const project = authorRecommendedWelds(createProject(), topology);
  const budget = cadRowBudgetStatus(project)!;

  // The import stayed inside the bound while the CAD asked for more.
  assert.ok(budget.requiredRows > ROBOT_V4_PROGRAM_ROW_BUDGET,
    `fixture must exceed the bound, asked ${budget.requiredRows}`);
  assert.ok(project.rows.length <= ROBOT_V4_PROGRAM_ROW_BUDGET);
  assert.equal(budget.importedRows, project.rows.length);

  // What it gave up: whole seams, disabled and rowless, named in the lint.
  const disabled = project.seams.filter((seam) => !seam.enabled);
  assert.ok(disabled.length >= 1);
  assert.deepEqual(budget.overBudgetSeamIds, disabled.map((seam) => seam.id));
  for (const seam of disabled) {
    assert.equal(project.rows.some((row) => row.seamId === seam.id), false);
  }
  // No seam is shortened: geometry is whole whether or not it was imported.
  assert.equal(project.seams.every((seam) => seam.geometry.pointsM.length === SAMPLES), true);

  // And it gave up no more than it had to: the largest seam it did import
  // would not fit a second time, so one more seam was genuinely too many.
  const perSeam = new Map<string, number>();
  for (const row of project.rows) {
    if (row.seamId) perSeam.set(row.seamId, (perSeam.get(row.seamId) ?? 0) + 1);
  }
  const largestImported = Math.max(...perSeam.values());
  assert.ok(project.rows.length + largestImported > ROBOT_V4_PROGRAM_ROW_BUDGET,
    "a dropped seam would have fitted; the import gave up more than the bound required");
});

/** One-edge topology used to drive the real CadQuery import path. */
function arcTopology(
  id: string,
  samplesM: Vec3[],
  geometryType = "CIRCLE",
): CadTopology {
  const axis = (index: 0 | 1 | 2, pick: (values: number[]) => number) =>
    pick(samplesM.map((point) => point[index]));
  return {
    schemaVersion: 1,
    unit: "m",
    sourceUnit: "mm",
    tool: "test",
    modelId: `points-moves-${id}`,
    filename: `${id}.step`,
    fingerprint: id.padEnd(64, "0").slice(0, 64),
    sampleCount: samplesM.length,
    parts: [{ id: "part", edgeCount: 1 }],
    edges: [{
      id,
      label: id.toUpperCase(),
      partId: "part",
      geometryType,
      sampleCount: samplesM.length,
      samplesM,
      lengthM: 0.1,
      boundsMinM: [axis(0, (v) => Math.min(...v)), axis(1, (v) => Math.min(...v)), axis(2, (v) => Math.min(...v))],
      boundsMaxM: [axis(0, (v) => Math.max(...v)), axis(1, (v) => Math.max(...v)), axis(2, (v) => Math.max(...v))],
      closed: false,
      sourceEdgeIds: [],
      sourceGeometryTypes: [],
      pathClass: geometryType === "CIRCLE" ? "circle" : "raw_fragment",
      isSeamCandidate: true,
    }],
    rawEdgeCount: 1,
    compositeEdgeCount: 0,
    seamCandidateCount: 1,
    recommendedSeams: [{
      edgeId: id,
      partId: "part",
      geometryType,
      pathClass: geometryType === "CIRCLE" ? "circle" : "raw_fragment",
      window: { start: 0, end: 1 },
      reversed: false,
      reason: "test",
    }],
  };
}

test("reconciliation catches an external RevPi slot change on a hidden sample (review F1)", () => {
  const project = createBlankSpreadsheetProject({ name: "F1 slot hash" });
  project.rows = Array.from({ length: 3 }, (_, index) => {
    const row = createSpreadsheetRow(project, "motion", `f1-${index}`);
    row.targetTcpPose = [index * 0.01, 0, 0, 0, 0, 0];
    return row;
  });
  attachMoveMetadata(project.rows, [{ startIndex: 0, endIndex: 2, kind: "LINE" }]);
  assert.equal(buildProgramView(project).moves[0]!.modifiedExternally, false);

  // A catalog peer changing WHICH physical IO slot a hidden sample fires must
  // surface — the view may never hide a physical-behaviour change.
  project.rows[1]!.revPiIOSlot = 7;
  const flagged = buildProgramView(project).moves[0]!;
  assert.equal(flagged.modifiedExternally, true);
  assert.match(flagged.externalReason ?? "", /modified externally/);
});

test("null RevPi slots survive the catalog round trip without false modification flags (review F1)", () => {
  const project = createBlankSpreadsheetProject({ name: "F1 round trip" });
  project.rows = Array.from({ length: 3 }, (_, index) => {
    const row = createSpreadsheetRow(project, "motion", `f1rt-${index}`);
    row.targetTcpPose = [index * 0.01, 0, 0, 0, 0, 0];
    row.revPiIOSlot = null; // serializes as 0 on the wire and reads back as 0
    return row;
  });
  attachMoveMetadata(project.rows, [{ startIndex: 0, endIndex: 2, kind: "LINE" }]);
  const reopened = fromRobotV4Program(toRobotV4Program(project));
  assert.equal(reopened.rows[1]!.revPiIOSlot, 0);
  assert.equal(buildProgramView(reopened).moves[0]!.modifiedExternally, false);
});

test("a CP shared with an externally modified move renders exactly once (review F2)", () => {
  const project = createBlankSpreadsheetProject({ name: "F2 shared CP" });
  project.rows = Array.from({ length: 5 }, (_, index) => {
    const row = createSpreadsheetRow(project, "motion", `f2-${index}`);
    row.targetTcpPose = [index * 0.01, 0, 0, 0, 0, 0];
    return row;
  });
  attachMoveMetadata(project.rows, [
    { startIndex: 0, endIndex: 2, kind: "LINE" },
    { startIndex: 2, endIndex: 4, kind: "LINE" },
  ]);
  // Break the second move the way an unaware peer would: strip a hidden
  // sample's metadata. Its rows fall back to raw; the shared CP must not
  // render twice.
  delete project.rows[3]!.extensions?.gradient_move_v1;
  const view = buildProgramView(project);
  const second = view.moves.find((move) => move.rowIndices.includes(3));
  assert.equal(second?.modifiedExternally, true);
  const sharedEntries = view.entries.filter((entry) =>
    (entry.kind === "control" && entry.control.row.id === "f2-2") ||
    (entry.kind === "raw" && entry.row.id === "f2-2")
  );
  assert.equal(sharedEntries.length, 1);
  assert.equal(sharedEntries[0]!.kind, "control");
  assert.equal(new Set(view.visibleRowIds).size, view.visibleRowIds.length);
});

test("duplicating a control point strips its move metadata (review F4)", async () => {
  const { duplicateSpreadsheetRow } = await import("./spreadsheet.ts");
  let project = createBlankSpreadsheetProject({ name: "F4 duplicate" });
  project.rows = Array.from({ length: 3 }, (_, index) => {
    const row = createSpreadsheetRow(project, "motion", `f4-${index}`);
    row.targetTcpPose = [index * 0.01, 0, 0, 0, 0, 0];
    return row;
  });
  attachMoveMetadata(project.rows, [{ startIndex: 0, endIndex: 2, kind: "LINE" }]);
  project = duplicateSpreadsheetRow(project, "f4-2");
  const duplicate = project.rows.find((row) => row.label.endsWith("copy"))!;
  assert.equal(moveMetadata(duplicate), null);
  // The original move stays intact and truthful.
  const view = buildProgramView(project);
  assert.equal(view.moves.find((move) => !move.synthetic)?.modifiedExternally, false);
});

test("Order reorders adjacent CAD control points and keeps shared CPs intact", () => {
  const project = createBlankSpreadsheetProject({ name: "Reorder CPs" });
  project.rows = Array.from({ length: 4 }, (_, index) => {
    const row = createSpreadsheetRow(project, "motion", `cp-${index}`);
    row.label = `P${index}`;
    row.targetTcpPose = [index * 0.1, 0, 0, 0, 0, 0];
    return row;
  });
  attachMoveMetadata(project.rows, [
    { startIndex: 0, endIndex: 1, kind: "LINE" },
    { startIndex: 1, endIndex: 2, kind: "LINE" },
    { startIndex: 2, endIndex: 3, kind: "LINE" },
  ]);

  assert.equal(canReorderControlPoint(project, "cp-0", -1), false);
  assert.equal(canReorderControlPoint(project, "cp-0", 1), true);
  assert.equal(canReorderControlPoint(project, "cp-3", 1), false);

  const next = reorderAdjacentControlPoints(project, "cp-1", 1);
  assert.deepEqual(next.rows.map((row) => row.id), ["cp-0", "cp-2", "cp-1", "cp-3"]);
  const view = buildProgramView(next);
  assert.deepEqual(view.controls.map((control) => control.row.id), [
    "cp-0", "cp-2", "cp-1", "cp-3",
  ]);
  assert.equal(view.moves.every((move) => !move.modifiedExternally), true);
  assert.equal(view.moves.every((move) => move.sampleRows.length === 0), true);
  assert.equal(new Set(view.visibleRowIds).size, view.visibleRowIds.length);
  assert.equal(view.controlByRowId.get("cp-2")?.number, 2);
});

test("Order collapses samples that would lie after a control-point swap", () => {
  const project = createBlankSpreadsheetProject({ name: "Reorder sampled" });
  project.rows = Array.from({ length: 5 }, (_, index) => {
    const row = createSpreadsheetRow(project, "motion", `samp-${index}`);
    row.targetTcpPose = [index * 0.01, 0, 0, 0, 0, 0];
    return row;
  });
  attachMoveMetadata(project.rows, [
    { startIndex: 0, endIndex: 2, kind: "LINE" },
    { startIndex: 2, endIndex: 4, kind: "LINE" },
  ]);
  assert.equal(buildProgramView(project).moves[0]!.sampleRows.length, 1);

  const next = reorderAdjacentControlPoints(project, "samp-2", 1);
  const view = buildProgramView(next);
  assert.deepEqual(view.controls.map((control) => control.row.id), [
    "samp-0", "samp-4", "samp-2",
  ]);
  assert.equal(next.rows.length, 3);
  assert.equal(view.moves.every((move) => !move.modifiedExternally), true);
  assert.equal(view.moves.every((move) => move.sampleRows.length === 0), true);
});

test("Order refuses control points on an externally modified move", () => {
  const project = createBlankSpreadsheetProject({ name: "Reorder blocked" });
  project.rows = Array.from({ length: 5 }, (_, index) => {
    const row = createSpreadsheetRow(project, "motion", `block-${index}`);
    row.targetTcpPose = [index * 0.01, 0, 0, 0, 0, 0];
    return row;
  });
  attachMoveMetadata(project.rows, [
    { startIndex: 0, endIndex: 2, kind: "LINE" },
    { startIndex: 2, endIndex: 4, kind: "LINE" },
  ]);
  delete project.rows[1]!.extensions?.gradient_move_v1;
  assert.equal(canReorderControlPoint(project, "block-2", 1), false);
  assert.throws(
    () => reorderAdjacentControlPoints(project, "block-2", 1),
    /externally modified/,
  );
});
