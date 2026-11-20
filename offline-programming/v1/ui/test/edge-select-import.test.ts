import assert from "node:assert/strict";
import test from "node:test";

import {
  adaptCadQueryTopology,
  cadRowBudgetStatus,
  ROBOT_V4_PROGRAM_ROW_BUDGET,
  rowsFromSeams,
  topologyToSeams,
  type CadTopology,
} from "../src/adapters/cadquery.ts";
import {
  createWeldFromEdges,
  createWeldsFromRecommendations,
  deleteWeld,
  projectWelds,
} from "../src/adapters/weld-authoring.ts";
import { projectEdgeCatalog } from "../src/domain/edge-catalog.ts";
import {
  centreWorkpieceOnWorkFrame,
  hasWorkpieceGeometry,
  transformWorkpiecePoint,
  worldPointToPlanner,
} from "../src/domain/placement.ts";
import {
  createProject,
  reviseProject,
  type AuthoredProgramRow,
  type OfflineProgrammingProject,
  type Vec3,
} from "../src/domain/project.ts";
import { exportProjectJSON, importProjectJSON } from "../src/domain/serialization.ts";
import { appendControlPointRow } from "../src/domain/spreadsheet.ts";
import { createDogBoxProject, DOG_BOX_FALLBACK_TOPOLOGY } from "../src/examples/dog-box.ts";

/**
 * P2.12 contract (2026-08-04): loading a STEP file creates NO welds and
 * NO program rows. The import produces a pickable edge catalog; the program
 * only comes into existence when the user selects edges and creates a weld.
 */

function lineEdge(
  id: string,
  label: string | null,
  origin: Vec3,
  samples = 16,
  lengthM = 0.15,
): CadTopology["edges"][number] {
  const samplesM = Array.from({ length: samples }, (_, index) =>
    [
      origin[0] + (lengthM * index) / (samples - 1),
      origin[1],
      origin[2],
    ] as Vec3);
  return {
    id,
    label,
    partId: "part-1",
    geometryType: "LINE",
    sampleCount: samples,
    samplesM,
    lengthM,
    boundsMinM: origin,
    boundsMaxM: [origin[0] + lengthM, origin[1], origin[2]],
    closed: false,
    sourceEdgeIds: [],
    sourceGeometryTypes: [],
    pathClass: "raw_fragment",
    isSeamCandidate: true,
  };
}

function testTopology(): CadTopology {
  const edges = [
    lineEdge("edge-a", null, [1.0, 0.2, 0.3]),
    lineEdge("edge-b", null, [1.0, 0.3, 0.3]),
    lineEdge("edge-c", null, [1.0, 0.4, 0.3]),
  ];
  return {
    schemaVersion: 1,
    unit: "m",
    sourceUnit: "mm",
    tool: "test",
    modelId: "edge-select-cad",
    filename: "edge_select.step",
    fingerprint: "e".repeat(64),
    sampleCount: 16,
    parts: [{ id: "part-1", edgeCount: edges.length }],
    edges,
    rawEdgeCount: edges.length,
    compositeEdgeCount: 0,
    seamCandidateCount: edges.length,
    // Recommendations cover every edge so the placement anchor of the old
    // seam-derived path and the new catalog-derived path coincide, keeping the
    // row comparison byte-meaningful.
    recommendedSeams: edges.map((edge, index) => ({
      edgeId: edge.id,
      partId: edge.partId,
      geometryType: edge.geometryType,
      pathClass: edge.pathClass,
      window: index === 1 ? { start: 0.1, end: 0.9 } : { start: 0, end: 1 },
      reversed: index === 2,
      reason: "test recommendation",
    })),
  };
}

/**
 * The exact program the pre-P2.12 auto-import produced: every recommendation
 * became an enabled seam and the full row set was generated at import time.
 */
function oldAutoProgram(topology: CadTopology): OfflineProgrammingProject {
  const imported = adaptCadQueryTopology(createProject(), topology);
  return reviseProject(imported, "cad", (draft) => {
    draft.seams = topologyToSeams(topology);
    draft.sequence = draft.seams.map((seam, index) => ({
      id: `sequence-${index + 1}`,
      seamId: seam.id,
      order: index,
      approachOffsetM: 0.05,
      retractOffsetM: 0.05,
    }));
    draft.rows = rowsFromSeams(draft);
  });
}

/** Rows with volatile identity fields normalized so content is comparable. */
function comparableRows(rows: readonly AuthoredProgramRow[]): unknown[] {
  const alias = new Map(rows.map((row, index) => [row.id, `row-${index + 1}`]));
  const aliasControl = (value: unknown): unknown => {
    if (typeof value !== "string" || !value.startsWith("cp:")) return value;
    const mapped = alias.get(value.slice(3));
    return mapped ? `cp:${mapped}` : value;
  };
  return rows.map((row) => {
    const clone = structuredClone(row) as unknown as Record<string, unknown>;
    clone.id = alias.get(row.id);
    delete clone.rowRevision;
    delete clone.rowHash;
    const extensions = clone.extensions as Record<string, unknown> | undefined;
    // A row's node id is as identity-volatile as its own id: the node is
    // minted per weld creation, and the rows are compiled from it.
    delete extensions?.node_id;
    delete extensions?.node_op;
    const meta = extensions?.gradient_move_v1 as Record<string, unknown> | undefined;
    if (meta) {
      for (const key of ["control_id", "start_control_id", "end_control_id"]) {
        meta[key] = aliasControl(meta[key]);
      }
      // The samples hash covers row ids, so it is identity-volatile by design.
      delete meta.samples_hash;
    }
    return roundNumbers(clone);
  });
}

/** Every number to 1 nm / 1 nrad.
 *
 *  These two paths compile the same program by different routes -- one weld
 *  at a time against the batch helper -- and used to land on bit-identical
 *  doubles. The work frame is the server's now, walked out of the URDF at the
 *  zero pose rather than rounded to 0.1 mm in the browser, and the extra
 *  digits make the two orders of the same arithmetic differ in the last bit
 *  (0.22004130000000005 against 0.22004129999999994). That is float
 *  associativity, not a difference between the paths, and a nanometre is
 *  still far finer than anything a row means. */
function roundNumbers(value: unknown): unknown {
  if (typeof value === "number") return Number.isFinite(value) ? Number(value.toFixed(9)) : value;
  if (Array.isArray(value)) return value.map(roundNumbers);
  if (value && typeof value === "object") {
    return Object.fromEntries(
      Object.entries(value as Record<string, unknown>).map(([key, item]) => [key, roundNumbers(item)]),
    );
  }
  return value;
}

test("STEP import produces zero rows, zero seams, and a persisted edge catalog", () => {
  const project = adaptCadQueryTopology(createProject(), testTopology());
  assert.equal(project.rows.length, 0);
  assert.equal(project.seams.length, 0);
  assert.equal(project.sequence.length, 0);
  const catalog = projectEdgeCatalog(project);
  assert.equal(catalog.length, 3);
  assert.deepEqual(catalog.map((edge) => edge.id), ["edge-a", "edge-b", "edge-c"]);
  // Topology polylines survive with their classification metadata.
  assert.equal(catalog[0]!.samplesM.length, 16);
  assert.equal(catalog[0]!.geometryType, "LINE");
  assert.equal(catalog[0]!.pathClass, "raw_fragment");
  assert.equal(catalog[0]!.partId, "part-1");
  // Extractor recommendations become suggestion metadata, never seams.
  assert.deepEqual(
    catalog.map((edge) => edge.recommendation !== null),
    [true, true, true],
  );
  assert.deepEqual(catalog[1]!.recommendation?.window, { start: 0.1, end: 0.9 });
  assert.equal(catalog[2]!.recommendation?.reversed, true);
  // The budget/tolerance seed still happens at import.
  assert.equal(cadRowBudgetStatus(project)?.importedRows, 0);
});

test("a re-import replaces the previous part's seams and rows instead of stacking them", () => {
  let project = adaptCadQueryTopology(createProject(), testTopology());
  project = createWeldsFromRecommendations(project);
  assert.ok(project.rows.length > 0);
  const reimported = adaptCadQueryTopology(project, testTopology());
  assert.equal(reimported.seams.length, 0);
  assert.equal(reimported.sequence.length, 0);
  assert.equal(reimported.rows.length, 0);
  assert.equal(projectEdgeCatalog(reimported).length, 3);
});

test("import centres the part on the table from the edge catalog alone", () => {
  const project = adaptCadQueryTopology(createProject(), testTopology());
  assert.equal(hasWorkpieceGeometry(project), true);
  // Anchor: XY bounds centre, lowest Z. Edges span x 1.0..1.15, y 0.2..0.4, z 0.3.
  const fixture = project.fixtures[0]!;
  // 0.1 mm: the work frame comes from the server's walk of the URDF now, and
  // these numbers were written against the browser's rounded copy of it. They
  // agree to 21 um.
  const expectTuple = (actual: readonly number[], expected: readonly number[]) =>
    expected.forEach((value, axis) =>
      assert.ok(
        Math.abs(actual[axis]! - value) < 1e-4,
        `axis ${axis}: ${actual[axis]} !== ${value}`,
      )
    );
  expectTuple(fixture.translationM, [-1.075, -0.3, -0.3]);
  // transformed anchor lands on the table centre frame origin.
  expectTuple(transformWorkpiecePoint(project, [1.075, 0.3, 0.3]), [
    1.913,
    0.6858,
    0.6509,
  ]);
  // Re-centring a catalog-only project stays valid (no seams involved).
  assert.equal(centreWorkpieceOnWorkFrame(project), true);
});

test("selecting the recommended edges and creating welds reproduces the old auto-import program", () => {
  const topology = testTopology();
  const reference = oldAutoProgram(topology);
  assert.ok(reference.rows.length > 0);

  // The interactive path: import, then one weld per recommended edge in order.
  let project = adaptCadQueryTopology(createProject(), topology);
  for (const edge of projectEdgeCatalog(project)) {
    const recommendation = edge.recommendation!;
    project = createWeldFromEdges(project, {
      weldType: "fillet",
      segments: [{
        edgeId: edge.id,
        startS: recommendation.window.start,
        endS: recommendation.window.end,
        reversed: recommendation.reversed,
      }],
    });
  }

  assert.equal(project.seams.length, reference.seams.length);
  const seamComparable = (seams: OfflineProgrammingProject["seams"]) =>
    seams.map(({ weldId: _weldId, weldType: _weldType, ...seam }) => seam);
  assert.deepEqual(seamComparable(project.seams), seamComparable(reference.seams));
  assert.deepEqual(project.sequence, reference.sequence);
  assert.deepEqual(comparableRows(project.rows), comparableRows(reference.rows));

  // The one-call batch helper produces the same program.
  const batch = createWeldsFromRecommendations(
    adaptCadQueryTopology(createProject(), topology),
  );
  assert.deepEqual(comparableRows(batch.rows), comparableRows(reference.rows));
});

test("a multi-edge weld groups its seams and deleting the weld removes seams, steps, and rows", () => {
  let project = adaptCadQueryTopology(createProject(), testTopology());
  project = createWeldFromEdges(project, {
    name: "Cover weld",
    weldType: "butt",
    segments: [
      { edgeId: "edge-a", startS: 0, endS: 1, reversed: false },
      { edgeId: "edge-b", startS: 0.2, endS: 0.8, reversed: true },
    ],
  });
  project = createWeldFromEdges(project, {
    weldType: "fillet",
    segments: [{ edgeId: "edge-c", startS: 0, endS: 1, reversed: false }],
  });

  const welds = projectWelds(project);
  assert.equal(welds.length, 2);
  const cover = welds[0]!;
  assert.equal(cover.label, "Cover weld");
  assert.equal(cover.weldType, "butt");
  assert.equal(cover.seamIds.length, 2);
  assert.equal(project.seams.length, 3);
  // Segment trims land as metre offsets along the sampled polyline.
  const trimmed = project.seams[1]!;
  assert.ok(Math.abs(trimmed.trim.startOffsetM - 0.03) < 1e-9);
  assert.ok(Math.abs(trimmed.trim.endOffsetM - 0.03) < 1e-9);
  assert.equal(trimmed.reversed, true);

  const coverRowCount = project.rows.filter((row) =>
    row.seamId !== null && cover.seamIds.includes(row.seamId)
  ).length;
  assert.ok(coverRowCount > 0);

  const afterDelete = deleteWeld(project, cover.id);
  assert.equal(projectWelds(afterDelete).length, 1);
  assert.equal(afterDelete.seams.length, 1);
  assert.equal(afterDelete.sequence.length, 1);
  assert.equal(
    afterDelete.rows.some((row) =>
      row.seamId !== null && cover.seamIds.includes(row.seamId)
    ),
    false,
  );
  // The surviving weld keeps its rows.
  assert.ok(afterDelete.rows.length > 0);
});

test("creating a weld preserves manually authored rows", () => {
  let project = adaptCadQueryTopology(createProject(), testTopology());
  const pose = worldPointToPlanner(project, [1.7, 0.6, 0.8]);
  project = appendControlPointRow(project, [pose[0], pose[1], pose[2], 0, 0, 0]);
  const manualRowID = project.rows[0]!.id;
  assert.equal(project.rows.length, 1);

  project = createWeldFromEdges(project, {
    weldType: "fillet",
    segments: [{ edgeId: "edge-a", startS: 0, endS: 1, reversed: false }],
  });
  assert.ok(project.rows.length > 1);
  assert.ok(project.rows.some((row) => row.id === manualRowID));
});

test("shift-click add-point appends a selected motion control point at the picked position", () => {
  const project = createDogBoxProject("2026-08-04T00:00:00.000Z");
  const before = project.rows.length;
  const next = appendControlPointRow(project, [0.5, 0.25, 0.75, 0, 0, 0.5]);
  assert.equal(next.rows.length, before + 1);
  const added = next.rows.at(-1)!;
  assert.deepEqual(added.targetTcpPose, [0.5, 0.25, 0.75, 0, 0, 0.5]);
  assert.equal(added.rowType, "motion");
  assert.equal(added.sourceState, "manual");
  assert.equal(added.weldEnabled, false);
  assert.equal(next.simulation.selectedRowId, added.id);
  // Points & moves: the appended row is a control point, not a hidden sample.
  assert.equal(added.extensions?.gradient_move_v1, undefined);
});

test("the edge catalog round-trips through project JSON export/import", () => {
  const project = adaptCadQueryTopology(createProject(), testTopology());
  const restored = importProjectJSON(exportProjectJSON(project));
  assert.deepEqual(projectEdgeCatalog(restored), projectEdgeCatalog(project));
  assert.equal(restored.rows.length, 0);
  assert.equal(restored.seams.length, 0);
});

test("legacy project JSON without an edge catalog hydrates to an empty catalog", () => {
  const legacy = structuredClone(createDogBoxProject("2026-08-04T00:00:00.000Z")) as unknown as Record<string, unknown>;
  delete legacy.edgeCatalog;
  const restored = importProjectJSON(JSON.stringify(legacy));
  assert.deepEqual(projectEdgeCatalog(restored), []);
  // Legacy seams and rows survive untouched — the boot-repair contract owns them.
  assert.ok(restored.seams.length > 0);
  assert.ok(restored.rows.length > 0);
});

test("weld creation is budget-checked: an over-budget weld imports its seam disabled and rowless", () => {
  // Just over the contract bound, whatever the bound is.
  const samples = ROBOT_V4_PROGRAM_ROW_BUDGET + 104;
  const bigEdge = {
    ...lineEdge("edge-big", "BIG", [0, 0, 0], samples, 0.5),
    // A zig-zag polyline defeats chord simplification so every sample becomes
    // a wire row, guaranteeing the seam cannot fit the contract bound.
    samplesM: Array.from({ length: samples }, (_, index) =>
      [index / 1000, index % 2 === 0 ? 0 : 0.01, 0] as Vec3),
  };
  const topology: CadTopology = {
    ...testTopology(),
    edges: [testTopology().edges[0]!, bigEdge],
    recommendedSeams: [],
  };
  let project = adaptCadQueryTopology(createProject(), topology);
  project = createWeldFromEdges(project, {
    weldType: "fillet",
    segments: [{ edgeId: "edge-a", startS: 0, endS: 1, reversed: false }],
  });
  const fitted = project.rows.length;
  assert.ok(fitted > 0);
  project = createWeldFromEdges(project, {
    weldType: "fillet",
    segments: [{ edgeId: "edge-big", startS: 0, endS: 1, reversed: false }],
  });
  const budget = cadRowBudgetStatus(project);
  const bigSeam = project.seams.find((seam) => seam.sourceEdgeId === "edge-big")!;
  assert.equal(bigSeam.enabled, false);
  assert.deepEqual(budget?.overBudgetSeamIds, [bigSeam.id]);
  assert.ok(budget!.requiredRows > ROBOT_V4_PROGRAM_ROW_BUDGET);
  assert.equal(project.rows.length, fitted);
  assert.equal(project.rows.some((row) => row.seamId === bigSeam.id), false);
});

test("the bundled Dog Box example is explicit weld authoring over its recommendations", () => {
  const example = createDogBoxProject("2026-07-30T00:00:00.000Z");
  // The example ships as a full authored program (it is not a STEP import)…
  assert.ok(example.rows.length > 0);
  assert.equal(example.seams.length, 6);
  assert.equal(projectWelds(example).length, 6);
  // …and carries the full pickable catalog for edge selection.
  assert.equal(projectEdgeCatalog(example).length, DOG_BOX_FALLBACK_TOPOLOGY.edges.length);
  // Byte-comparable with the old auto path over the same fixture.
  const reference = oldAutoProgram(DOG_BOX_FALLBACK_TOPOLOGY);
  assert.deepEqual(comparableRows(example.rows), comparableRows(reference.rows));
});

test("freeing budget re-enables a seam the system disabled for budget, not the operator", () => {
  const zigzag = (id: string, samples: number) => ({
    ...lineEdge(id, id.toUpperCase(), [0, 0, 0], samples, 0.5),
    samplesM: Array.from({ length: samples }, (_, index) =>
      [index / 1000, index % 2 === 0 ? 0 : 0.01, 0] as Vec3),
  });
  const topology: CadTopology = {
    ...testTopology(),
    edges: [zigzag("edge-first", Math.round(ROBOT_V4_PROGRAM_ROW_BUDGET * 0.73)), zigzag("edge-second", Math.round(ROBOT_V4_PROGRAM_ROW_BUDGET * 0.49))],
    recommendedSeams: [],
  };
  let project = adaptCadQueryTopology(createProject(), topology);
  project = createWeldFromEdges(project, {
    weldType: "fillet",
    segments: [{ edgeId: "edge-first", startS: 0, endS: 1, reversed: false }],
  });
  project = createWeldFromEdges(project, {
    weldType: "fillet",
    segments: [{ edgeId: "edge-second", startS: 0, endS: 1, reversed: false }],
  });
  const second = project.seams.find((seam) => seam.sourceEdgeId === "edge-second")!;
  assert.equal(second.enabled, false);
  assert.deepEqual(cadRowBudgetStatus(project)?.overBudgetSeamIds, [second.id]);

  const firstWeld = projectWelds(project)
    .find((weld) => weld.seamIds.some((seamId) =>
      project.seams.find((seam) => seam.id === seamId)?.sourceEdgeId === "edge-first"))!;
  const freed = deleteWeld(project, firstWeld.id);
  const recovered = freed.seams.find((seam) => seam.sourceEdgeId === "edge-second")!;
  assert.equal(recovered.enabled, true, "budget-disabled seam re-enables once the budget frees");
  assert.deepEqual(cadRowBudgetStatus(freed)?.overBudgetSeamIds, []);
  const recoveredRows = freed.rows.filter((row) => row.seamId === recovered.id);
  assert.ok(recoveredRows.length > 0, "rows restore");
  assert.ok(recoveredRows.every((row) => row.enabled), "restored rows are enabled");

  // An operator disable is not budget bookkeeping: deleting an unrelated weld
  // refreshes the budget, and the seam the OPERATOR turned off stays off.
  const smallTopology: CadTopology = {
    ...testTopology(),
    edges: [zigzag("edge-second", Math.round(ROBOT_V4_PROGRAM_ROW_BUDGET * 0.49)), lineEdge("edge-a", null, [1.0, 0.2, 0.3])],
    recommendedSeams: [],
  };
  let operatorProject = adaptCadQueryTopology(createProject(), smallTopology);
  operatorProject = createWeldFromEdges(operatorProject, {
    weldType: "fillet",
    segments: [{ edgeId: "edge-second", startS: 0, endS: 1, reversed: false }],
  });
  operatorProject.seams.find((seam) => seam.sourceEdgeId === "edge-second")!.enabled = false;
  operatorProject = createWeldFromEdges(operatorProject, {
    weldType: "fillet",
    segments: [{ edgeId: "edge-a", startS: 0, endS: 1, reversed: false }],
  });
  const otherWeld = projectWelds(operatorProject)
    .find((weld) => weld.seamIds.some((seamId) =>
      operatorProject.seams.find((seam) => seam.id === seamId)?.sourceEdgeId === "edge-a"))!;
  const refreshed = deleteWeld(operatorProject, otherWeld.id);
  const kept = refreshed.seams.find((seam) => seam.sourceEdgeId === "edge-second")!;
  assert.equal(kept.enabled, false, "operator disable survives budget refresh");
});
