import assert from "node:assert/strict";
import { existsSync } from "node:fs";
import { readFile } from "node:fs/promises";
import test from "node:test";

import {
  ROBOT_V4_PROGRAM_V2_SCHEMA,
  assertRobotV4ProgramV2,
  computeProgramV2Digest,
  computeProgramV2NodeDigest,
  parseRobotV4ProgramV2,
  programV2Problems,
  type RobotV4ProgramV2,
} from "../src/contracts/robot-v4-program-v2.ts";

const contracts = new URL("../../../../steamdeck/real/v4/contracts/programs/", import.meta.url);
const fixtures = new URL("fixtures/", contracts);
const weldPlannerSchema = new URL(
  "../../../../weld_planner/v1/schema/amr-weld-planner-v1.weld-program.v1.schema.json",
  import.meta.url,
);

async function jsonFile(base: URL, name: string): Promise<Record<string, unknown>> {
  return JSON.parse(await readFile(new URL(name, base), "utf8")) as Record<string, unknown>;
}

async function complete(): Promise<RobotV4ProgramV2> {
  return parseRobotV4ProgramV2(await jsonFile(fixtures, "complete-program-v2.json"));
}

type Doc = Record<string, unknown>;
const nodes = (doc: Doc): Doc[] => doc.nodes as Doc[];
const nodeAt = (doc: Doc, index: number): Doc => nodes(doc)[index] as Doc;
const clone = (doc: Doc, index: number, id: string): Doc => ({ ...structuredClone(nodeAt(doc, index)), id });

// Node indexes in the complete fixture:
// 0 io, 1 home, 2 approach, 3 weld(seam), 4 transit, 5 io,
// 6 weld(posed_polyline), 7 dwell, 8 transit, 9 weld(manual_pose_wp),
// 10 retract, 11 home, 12 io.

test("program.v2 schema is draft 2020-12 and self-contained", async () => {
  const schema = await jsonFile(contracts, "robot.v4.program.v2.schema.json");
  assert.equal(schema.$schema, "https://json-schema.org/draft/2020-12/schema");
  assert.equal(typeof schema.$id, "string");
  const defs = schema.$defs as Record<string, unknown>;
  const walk = (value: unknown, path: string): void => {
    if (Array.isArray(value)) {
      value.forEach((child, index) => walk(child, `${path}[${index}]`));
    } else if (value && typeof value === "object") {
      for (const [key, child] of Object.entries(value as Record<string, unknown>)) {
        if (key === "$ref") {
          assert.ok(String(child).startsWith("#/$defs/"), `${path}: non-local $ref ${String(child)}`);
          assert.ok(String(child).slice("#/$defs/".length) in defs, `${path}: dangling $ref ${String(child)}`);
        } else {
          walk(child, `${path}.${key}`);
        }
      }
    }
  };
  walk(schema, "$");
});

test("the complete fixture is accepted and covers every op and every geometry kind", async () => {
  const program = await complete();
  const ops = new Set(program.nodes.map((node) => node.op));
  for (const op of ["weld", "approach", "transit", "retract", "home", "dwell", "io"]) {
    assert.ok(ops.has(op as never), `missing a ${op} node`);
  }
  const kinds = new Set(program.nodes.flatMap((node) => (node.op === "weld" ? [node.geometry.kind] : [])));
  assert.deepEqual([...kinds].sort(), ["manual_pose_wp", "posed_polyline", "seam"]);
  assert.deepEqual(programV2Problems(program), []);
});

test("invalid fixtures fail closed naming the offending key", async () => {
  for (const [name, pattern] of [
    ["invalid-program-v2-freespace-solution.json", /trajectory/],
    ["invalid-program-v2-dense-joints.json", /q_rad/],
    ["invalid-program-v2-weld-parameters-filled.json", /weld_parameters/],
    ["invalid-program-v2-adjacent-welds.json", /transit is missing/],
    ["invalid-program-v2-dangling-joint-ref.json", /not in workpiece\.weld_joints/],
    ["invalid-program-v2-search-bin-wider-than-window.json", /which is one bin/],
  ] as const) {
    const doc = await jsonFile(fixtures, name);
    assert.throws(() => assertRobotV4ProgramV2(doc), pattern, name);
  }
});

test("the line must alternate weld and freespace; events are transparent; solutions are refused", async () => {
  const cases: [string, (doc: Doc) => void, RegExp | null][] = [
    ["two welds with nothing between them", (d) => nodes(d).splice(4, 1), /transit is missing/],
    ["a weld with no approach", (d) => nodes(d).splice(1, 2), /approach is missing/],
    ["home in the middle", (d) => nodes(d).splice(5, 0, clone(d, 1, "home_mid")), /middle/],
    ["two freespace nodes adjacent", (d) => nodes(d).splice(11, 0, clone(d, 10, "retract_again")), /only a home or nothing may follow/],
    ["a transit not between welds", (d) => nodes(d).splice(2, 0, clone(d, 4, "transit_early")), /between two welds/],
    ["events between approach and weld", (d) => nodes(d).splice(3, 0, clone(d, 0, "io_early"), clone(d, 7, "dwell_early")), null],
    ["a solved path on a transit", (d) => { nodeAt(d, 4).trajectory = []; }, /trajectory/],
    ["classic points under a transit", (d) => { nodeAt(d, 4).points = []; }, /points/],
    ["dense joints on a weld", (d) => { nodeAt(d, 3).q_rad = []; }, /q_rad/],
    ["a field the schema does not name", (d) => { nodeAt(d, 3).speed_hint = 1; }, /speed_hint/],
    ["freespace policy on a weld", (d) => { nodeAt(d, 3).overrides = { positioner_policy: "hold" }; }, /overrides/],
    ["an override that is not a defaults key", (d) => { nodeAt(d, 4).overrides = { max_qd: 1 }; }, /not a defaults key/],
    ["a depart standoff on an approach", (d) => { nodeAt(d, 2).depart_standoff = nodeAt(d, 10).depart_standoff; }, /nothing is welded before/],
    ["weld_parameters filled", (d) => { nodeAt(d, 6).weld_parameters = { voltage_v: 24 }; }, /reserved/],
    ["manual poses copied onto each other", (d) => {
      const points = (nodeAt(d, 9).geometry as Doc).points as Doc[];
      points[1] = structuredClone(points[0]!);
    }, /points 0 and 1 coincide/],
    ["manual poses outside the workpiece frame", (d) => {
      (nodeAt(d, 9).geometry as Doc).frame_id = "world";
    }, /is not the workpiece frame/],
    ["manual poses as an arc through the L", (d) => { (nodeAt(d, 9).geometry as Doc).shape = "arc"; }, null],
    ["an arc through points on a line", (d) => {
      const geometry = nodeAt(d, 9).geometry as Doc;
      geometry.shape = "arc";
      ((geometry.points as Doc[])[1] as Doc).xyz_m = [-0.07, 0.0425, 0.153];
    }, /lie on a line/],
    ["an arc of four points", (d) => {
      const geometry = nodeAt(d, 9).geometry as Doc;
      geometry.shape = "arc";
      const points = geometry.points as Doc[];
      points.push({ ...structuredClone(points[2]!), xyz_m: [-0.02, 0.0225, 0.153] });
    }, /exactly three/],
    ["a manual pose with no continuity", (d) => {
      delete ((nodeAt(d, 9).geometry as Doc).points as Doc[])[1]!.continuity;
    }, /continuity must be C0, C1 or C2/],
    ["weld_parameters absent", (d) => { delete nodeAt(d, 6).weld_parameters; }, /weld_parameters/],
    ["a knot vector one short", (d) => {
      const geometry = nodeAt(d, 3).geometry as Doc;
      const segment = ((geometry.curve as Doc).segments as Doc[])[0] as Doc;
      (segment.knots_u as number[]).pop();
    }, /knots_u/],
    ["a wrapping span on an open curve", (d) => {
      const geometry = nodeAt(d, 3).geometry as Doc;
      (geometry.curve as Doc).closed = false;
      geometry.span = { start_s: 0.8, end_s: 0.2 };
    }, /closed curve/],
    ["a piecewise_linear profile on a weld is seam vocabulary", (d) => {
      nodeAt(d, 3).travel_speed_mm_s = { kind: "piecewise_linear", knots: [[0, 8], [1, 6]] };
    }, null],
    ["a piecewise_linear profile running backwards", (d) => {
      nodeAt(d, 3).travel_speed_mm_s = { kind: "piecewise_linear", knots: [[0, 8], [0.5, 6], [0.4, 6], [1, 6]] };
    }, /strictly increasing/],
    ["a duplicate node id", (d) => { nodeAt(d, 7).id = nodeAt(d, 5).id; }, /used twice/],
    ["an empty planner_owns", (d) => { d.planner_owns = []; }, /planner_owns/],
    ["a robot without an identity", (d) => { d.robot = { model_id: "rosie_1400_v3" }; }, /robot\.robot_description_sha256/],
    ["a placement naming another robot", (d) => { (d.placement as Doc).cell_id = "rosie_1420_v1"; }, /placement\.cell_id/],
  ];
  for (const [name, mutate, pattern] of cases) {
    const doc = structuredClone(await complete()) as unknown as Doc;
    mutate(doc);
    if (pattern === null) {
      assert.doesNotThrow(() => assertRobotV4ProgramV2(doc), name);
    } else {
      assert.throws(() => assertRobotV4ProgramV2(doc), pattern, name);
    }
  }
});

test("digests follow the v1 discipline: node hash excludes itself, program digest excludes itself", async () => {
  const program = await complete();
  const first = program.nodes[0]!;
  const hash = await computeProgramV2NodeDigest(first);
  assert.match(hash, /^sha256:[0-9a-f]{64}$/);
  assert.equal(await computeProgramV2NodeDigest({ ...first, row_hash: "sha256:" + "0".repeat(64) }), hash);
  const digest = await computeProgramV2Digest(program);
  assert.equal(await computeProgramV2Digest({ ...program, program_digest: digest }), digest);
  assert.notEqual(await computeProgramV2Digest({ ...program, program_name: "renamed" }), digest);
});

test("the vendored seam vocabulary matches the weld planner schema", async (t) => {
  if (!existsSync(weldPlannerSchema)) {
    t.skip(`weld planner schema not present at ${weldPlannerSchema.pathname}`);
    return;
  }
  const source = JSON.parse(await readFile(weldPlannerSchema, "utf8")) as Doc;
  const schema = await jsonFile(contracts, "robot.v4.program.v2.schema.json");
  const defs = schema.$defs as Doc;
  const sourceDefs = source.$defs as Doc;
  const sourceProps = source.properties as Doc;
  const seam = structuredClone(sourceDefs.seam) as Doc;
  delete (seam.properties as Doc).process;
  seam.required = (seam.required as string[]).filter((key) => key !== "process");
  // One deliberate difference, D24: v1 carries weld_joints at top
  // level; program.v2 carries it INSIDE the workpiece, because a joint is an
  // objective fact about the part and that is what workpiece already holds.
  const workpiece = structuredClone(sourceProps.workpiece) as Doc;
  (workpiece.properties as Doc).weld_joints = sourceProps.weld_joints;
  const expected: Doc = {
    identifier: sourceDefs.identifier,
    vec3: sourceDefs.vec3,
    unit_vec3: sourceDefs.unit_vec3,
    scalar_function: sourceDefs.scalar_function,
    analytic: sourceDefs.analytic,
    curve_segment: sourceDefs.curve_segment,
    curve: sourceDefs.curve,
    orientation_limit_axis: sourceDefs.orientation_limit_axis,
    seam,
    weld_joint: sourceDefs.weld_joint,
    groove_face: sourceDefs.groove_face,
    weave: ((((sourceDefs.seam as Doc).properties as Doc).process as Doc).properties as Doc).weave,
    source: sourceProps.source,
    workpiece,
  };
  for (const [name, want] of Object.entries(expected)) {
    assert.deepEqual(defs[name], want, `$defs/${name} has drifted from the weld planner schema; re-vendor it, do not edit the copy`);
  }
  const geometry = (defs.geometry_seam as Doc).properties as Doc;
  const seamProps = seam.properties as Doc;
  for (const [key, want] of Object.entries(seamProps)) {
    if (key !== "curve") assert.deepEqual(geometry[key], want, `geometry_seam.${key}`);
  }
  for (const key of Object.keys(geometry)) {
    // `search_provenance` is the seam's counterpart to `torch_angles_deg` on
    // the classic geometry. A weld program does not carry it.
    if (key !== "kind" && key !== "curve" && key !== "search_provenance") {
      assert.ok(key in seamProps, `geometry_seam.${key} is not a seam property`);
    }
  }
});

test("the schema name is the one the document declares", () => {
  assert.equal(ROBOT_V4_PROGRAM_V2_SCHEMA, "robot.v4.program.v2");
});

test("a weld with no standoff does not validate", async () => {
  // The planner places the tip from this. Absent is not zero, and zero is the
  // tip inside the part -- so it is refused here, at authoring, rather than
  // arriving as a refused seam inside a 200 from the motion server.
  const program = await complete();
  const weld = program.nodes.find((node) => node.op === "weld")!;
  delete (weld as unknown as Record<string, unknown>).standoff_mm;
  assert.throws(() => assertRobotV4ProgramV2(program), /standoff_mm/);
});

test("a weld welded at zero standoff does not validate", async () => {
  const program = await complete();
  const weld = program.nodes.find((node) => node.op === "weld")!;
  (weld as unknown as Record<string, unknown>).standoff_mm = { kind: "constant", value: 0 };
  assert.throws(() => assertRobotV4ProgramV2(program), /positive everywhere/);
});

test("a standoff profile that reaches zero part-way does not validate", async () => {
  // Every value along the arc has to clear the work, not just the ends.
  const program = await complete();
  const weld = program.nodes.find((node) => node.op === "weld")!;
  (weld as unknown as Record<string, unknown>).standoff_mm = {
    kind: "piecewise_linear", knots: [[0, 12], [1, 0]],
  };
  assert.throws(() => assertRobotV4ProgramV2(program), /positive everywhere/);
});

test("a standoff that varies along the weld validates", async () => {
  const program = await complete();
  const weld = program.nodes.find((node) => node.op === "weld")!;
  (weld as unknown as Record<string, unknown>).standoff_mm = {
    kind: "piecewise_linear", knots: [[0, 12], [1, 9]],
  };
  assert.deepEqual(programV2Problems(program), []);
});
