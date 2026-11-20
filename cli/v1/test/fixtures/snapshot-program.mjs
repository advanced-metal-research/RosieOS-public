// Prints one `renderSnapshots` input as JSON on stdout: a program built by
// OLP's OWN domain functions (not a hand-written document that could drift
// from what the browser saves), the the 1430 cell catalogue entry, a CadQuery-shaped
// tessellation of the part and a dense plan. Run by test/snapshot.test.ts:
//
//   node cli/v1/test/fixtures/snapshot-program.mjs > input.json
//
// Plain .mjs importing OLP's .ts through Node's type stripping (Node >= 22.18
// strips by default), so the CLI's tsc never type-checks the OLP UI tree.
// Needs offline-programming/v1/ui/node_modules (three) like the snapshot build.

import { DOG_BOX_FALLBACK_TOPOLOGY } from "../../../../offline-programming/v1/ui/src/examples/dog-box.ts";
import { adaptCadQueryTopology } from "../../../../offline-programming/v1/ui/src/adapters/cadquery.ts";
import { createWeldsFromRecommendations } from "../../../../offline-programming/v1/ui/src/adapters/weld-authoring.ts";
import { setWorkpiecePlacement } from "../../../../offline-programming/v1/ui/src/domain/placement.ts";
import { createProject } from "../../../../offline-programming/v1/ui/src/domain/project.ts";
import { parseRobotCatalogue, setRobotCatalogue } from "../../../../offline-programming/v1/ui/src/domain/robots.ts";
import { exportProjectJSON } from "../../../../offline-programming/v1/ui/src/domain/serialization.ts";
import { manualPoseWeldNode } from "../../../../offline-programming/v1/ui/src/waypoints/node.ts";

// The the 1430 cell entry as `GET /api/offline-programming/v1/robots` answers it
// (robot_description/go Store.Catalogue over robot_description/robots,
// 2026-10-29): the work frame IS the world and the arm starts at J1's child.
// Settings are left out; nothing a snapshot draws reads them.
export const ARM_CATALOGUE = {
  robots: [{
    model_id: "rosie_1420_v1",
    robot_description_sha256: "sha256:2ecc85494ebb6eaf8d9f5caee5fb1037fb5f43de0e3a0272d09b49285442057d",
    frames: { base: "world", tool: "tool0", work: "world", work_world_m: [0, 0, 0], arm_base_world_m: [0.4127, 0.4, 0.893] },
    axes: { driven: ["J1", "J2", "J3", "J4", "J5", "J6"] },
    reset_pose: { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 },
  }],
};

const STAMP = "2026-10-29T00:00:00.000Z";

setRobotCatalogue(parseRobotCatalogue(ARM_CATALOGUE));
// The dog box the way the browser authors it (`createDogBoxProject`), with
// one step between import and welds: the part is placed on the stand in
// front of the arm, so the welds are authored where it sits.
const STAND_TOP_M = 0.6;
const STAND_CENTRE_M = [1.1, 0.4];
let project = adaptCadQueryTopology(createProject({
  name: "Snapshot test: dog box on rosie_1420_v1",
  now: STAMP,
  cad: {
    kind: "example",
    filename: DOG_BOX_FALLBACK_TOPOLOGY.filename,
    fingerprint: DOG_BOX_FALLBACK_TOPOLOGY.fingerprint,
    sourceUnit: DOG_BOX_FALLBACK_TOPOLOGY.sourceUnit,
    topologyModelId: DOG_BOX_FALLBACK_TOPOLOGY.modelId,
  },
}), DOG_BOX_FALLBACK_TOPOLOGY, STAMP);
if (project.robot.model !== "rosie_1420_v1") throw new Error(`dog box authored for ${project.robot.model}, not rosie_1420_v1`);
if (!setWorkpiecePlacement(project, { offsetFromWorkFrameM: [...STAND_CENTRE_M, STAND_TOP_M], rotationRpyRad: [0, 0, 0] })) {
  throw new Error("the dog box has no placement anchor");
}
project = createWeldsFromRecommendations(project, STAMP);

// The part as CadQuery tessellates it, in the CAD frame and in millimetres:
// the dog box fallback topology's bounding box (x 620..950, y -190..190,
// z 610..760 mm), six faces of two triangles each.
const MIN_MM = [620, -190, 610];
const MAX_MM = [950, 190, 760];

/** A box as `POST /cadquery/topology {operation: "tessellate"}` answers:
 *  per-face vertices so each face keeps its own normal. */
function boxTessellation(fingerprint, filename, min, max) {
  const positions = [];
  const normals = [];
  const indices = [];
  // One face per axis and side; winding is chosen so the triangle normal
  // (right-hand rule) points out of the box, as CadQuery's does.
  for (const axis of [0, 1, 2]) {
    for (const plane of [max, min]) {
      const [u, v] = [0, 1, 2].filter((a) => a !== axis);
      const outward = [0, 0, 0];
      outward[axis] = plane === max ? 1 : -1;
      const base = positions.length / 3;
      const corners = [[0, 0], [1, 0], [1, 1], [0, 1]].map(([a, b]) => {
        const p = [0, 0, 0];
        p[axis] = plane[axis];
        p[u] = a ? max[u] : min[u];
        p[v] = b ? max[v] : min[v];
        return p;
      });
      for (const p of corners) positions.push(...p), normals.push(...outward);
      const e1 = corners[1].map((c, i) => c - corners[0][i]);
      const e2 = corners[2].map((c, i) => c - corners[0][i]);
      const n = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
      const ccw = n[0] * outward[0] + n[1] * outward[1] + n[2] * outward[2] > 0;
      indices.push(...(ccw ? [base, base + 1, base + 2, base, base + 2, base + 3] : [base, base + 2, base + 1, base, base + 3, base + 2]));
    }
  }
  return {
    schema_version: 1,
    tool: "cadquery-tessellate",
    model_id: `cadquery-step-${fingerprint.slice(0, 16)}`,
    filename,
    fingerprint,
    unit: "mm",
    mesh: {
      positions, indices, normals,
      vertex_count: positions.length / 3,
      triangle_count: indices.length / 3,
      bounds_min: min, bounds_max: max,
      linear_deflection_mm: 0.5, angular_deflection_rad: 0.5,
      solid_count: 1, truncated: false,
    },
  };
}

// A hand-placed (waypoint) weld across the box top, in the part frame, torch
// pointing straight down (+Z of the torch into the work: 180 degrees about X).
const DOWN = [1, 0, 0, 0];
const weld = manualPoseWeldNode("Top diagonal", [
  { id: "wp-a", positionM: [0.66, -0.15, 0.76], quaternion: DOWN, continuity: "c1", standoffMm: 15 },
  { id: "wp-b", positionM: [0.785, 0.0, 0.76], quaternion: DOWN, continuity: "c1", standoffMm: 15 },
  { id: "wp-c", positionM: [0.91, 0.15, 0.76], quaternion: DOWN, continuity: "c1", standoffMm: 15 },
], { travelSpeedMmS: 8, weave: { shape: "none" } });
project.nodes.push(weld);

// The stand the part sits on: a box footprint centred under it, floor to the
// part's underside.
project.workcellFixtures = [{
  id: "stand", name: "Stand", origin: "authored",
  shape: { kind: "box", sizeM: [0.4, 0.45, STAND_TOP_M] },
  translationM: [...STAND_CENTRE_M, 0], rotationRpyRad: [0, 0, 0],
}];

// A dense plan, robot.v4.dense-joint-trajectory.v1: home -> above the part ->
// a weld sweep -> home, every joint interpolated together. Synthetic joints
// (no planner ran); what it proves is the decode -> FK -> line path the
// browser uses. Sample count per segment and dt are what the header says.
const DT_S = 0.02;
const HOME = [0, 0, 0, 0, 0, 0, 0, 0, 0];
const ABOVE = [-0.35, 0.55, 0.35, 0, 0.75, 0, 0, 0, 0];
const SWEEP_END = [0.25, 0.65, 0.2, 0, 0.8, 0, 0, 0, 0];
const segments = [
  { kind: "freespace", source_id: "home->weld", from: HOME, to: ABOVE, samples: 40 },
  { kind: "weld", source_id: weld.id, from: ABOVE, to: SWEEP_END, samples: 60 },
  { kind: "freespace", source_id: "weld->home", from: SWEEP_END, to: HOME, samples: 40 },
];
function denseBlob() {
  const header = {
    schema: "robot.v4.dense-joint-trajectory.v1",
    plan_id: "snapshot-test-plan",
    program_id: project.identity.id,
    trajectory_digest: `sha256:${"5".repeat(64)}`,
    robot_cell: { model_id: "rosie_1420_v1", robot_description_sha256: ARM_CATALOGUE.robots[0].robot_description_sha256, machine_planning_calibration_sha256: "" },
    sampling: { dt_s: DT_S, total_sample_count: segments.reduce((n, s) => n + s.samples, 0) },
    dt_s: DT_S,
    segments: segments.map((s, index) => ({ index, kind: s.kind, source_id: s.source_id, sample_count: s.samples, duration_s: (s.samples - 1) * DT_S })),
  };
  const head = Buffer.from(JSON.stringify(header));
  const RECORD = 8 + 9 * 8 + 9 * 8 + 1;
  const body = Buffer.alloc(header.sampling.total_sample_count * RECORD);
  let at = 0;
  for (const s of segments) {
    for (let i = 0; i < s.samples; i++) {
      const f = i / (s.samples - 1);
      body.writeDoubleBE(i * DT_S, at);
      for (let axis = 0; axis < 9; axis++) {
        body.writeDoubleBE(s.from[axis] + (s.to[axis] - s.from[axis]) * f, at + 8 + axis * 8);
        body.writeDoubleBE(0, at + 8 + 72 + axis * 8);
      }
      body.writeUInt8(s.kind === "weld" ? 1 : 0, at + RECORD - 1);
      at += RECORD;
    }
  }
  const length = Buffer.alloc(4);
  length.writeUInt32BE(head.length, 0);
  return { bytes: Buffer.concat([length, head, body]), header };
}
const dense = denseBlob();
project.denseTrajectory = {
  schema: dense.header.schema,
  trajectoryDigest: dense.header.trajectory_digest,
  requestSha256: `sha256:${"6".repeat(64)}`,
  planId: dense.header.plan_id,
  programId: project.identity.id,
  programDigest: `sha256:${"7".repeat(64)}`,
  manifestRevision: 1,
  planRevision: 1,
  createdAt: STAMP,
  dtS: DT_S,
  totalSampleCount: dense.header.sampling.total_sample_count,
  totalDurationS: dense.header.segments.reduce((t, s) => t + s.duration_s, 0),
  bytes: dense.bytes.length,
  segments: dense.header.segments.map((s) => ({ index: s.index, kind: s.kind, sourceId: s.source_id, sampleCount: s.sample_count, durationS: s.duration_s })),
  sourceProjectRevision: project.identity.revision,
  robot: { modelId: "rosie_1420_v1", robotDescriptionSha256: dense.header.robot_cell.robot_description_sha256, machinePlanningCalibrationSha256: "" },
};

const fingerprint = DOG_BOX_FALLBACK_TOPOLOGY.fingerprint;
process.stdout.write(JSON.stringify({
  catalogue: ARM_CATALOGUE,
  // The export envelope the browser's "Export" writes; `load` also takes the
  // bare project a cell's program store keeps.
  document: JSON.parse(exportProjectJSON(project, STAMP)),
  meshes: { [fingerprint]: boxTessellation(fingerprint, DOG_BOX_FALLBACK_TOPOLOGY.filename, MIN_MM, MAX_MM) },
  denseBase64: dense.bytes.toString("base64"),
  expect: {
    weldNodeId: weld.id,
    rows: project.rows.length,
    triangles: 12,
    denseSamples: dense.header.sampling.total_sample_count,
  },
}));
