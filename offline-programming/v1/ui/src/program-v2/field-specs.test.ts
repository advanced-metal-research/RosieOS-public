import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

import type { FreespaceNode, PosedPoint, ProgramV2Node, WeldNode } from "../contracts/robot-v4-program-v2.ts";
import { createDogBoxProject } from "../examples/dog-box.ts";
import { applyEdit, groupsForNode, nodeProblems, type FieldSpec } from "./field-specs.ts";
import { classicProvenance, syncClassicNodes } from "./from-project.ts";
import { dwellNode, freespaceNode, ioNode, weldNodes } from "./nodes.ts";

const schemaUrl = new URL("../../../../../steamdeck/real/v4/contracts/programs/robot.v4.program.v2.schema.json", import.meta.url);
const context = { defaults: { clearance_min_mm: 20, speed_scale: 1, torch_policy: "free" as const, positioner_policy: "free" as const }, classicSeam: true };

type Schema = Record<string, unknown>;

/** Every schema node a dotted path can land on, following $ref and oneOf. */
function resolvePath(schema: Schema, start: Schema, path: string[]): boolean {
  const defs = schema.$defs as Record<string, Schema>;
  const expand = (node: Schema): Schema[] => {
    if (typeof node.$ref === "string") return expand(defs[node.$ref.replace("#/$defs/", "")]!);
    if (Array.isArray(node.oneOf)) return (node.oneOf as Schema[]).flatMap(expand);
    return [node];
  };
  if (path.length === 0) return true;
  const [head, ...rest] = path;
  return expand(start).some((node) => {
    const properties = node.properties as Record<string, Schema> | undefined;
    const child = properties?.[head!];
    return child !== undefined && resolvePath(schema, child, rest);
  });
}

function sampleNodes(): ProgramV2Node[] {
  const project = createDogBoxProject("2026-09-03T00:00:00.000Z");
  syncClassicNodes(project);
  const posed = weldNodes(project.nodes)[0]!;
  const seam: WeldNode = {
    ...structuredClone(posed),
    id: "seam",
    geometry: {
      kind: "seam", id: "seam_0001", reference: { mode: "fixed_vector" },
      curve: { segments: [{ degree: 1, control_points_xyz_m: [[0, 0, 0], [1, 0, 0]], knots_u: [0, 0, 1, 1] }] },
      orientation: { work_angle_deg: { kind: "constant", value: 0 }, travel_angle_deg: { kind: "constant", value: 0 } },
    },
  };
  delete seam.extensions;
  return [
    posed, seam,
    freespaceNode("approach"), freespaceNode("transit"), freespaceNode("retract"), freespaceNode("home"),
    dwellNode(0.5), ioNode("revpi:3", true),
  ];
}

test("every field path exists in the program.v2 schema for its node type", async () => {
  const schema = JSON.parse(await readFile(schemaUrl, "utf8")) as Schema;
  const defs = schema.$defs as Record<string, Schema>;
  const defFor = (node: ProgramV2Node): Schema =>
    node.op === "weld" ? defs.node_weld! : node.op === "dwell" ? defs.node_dwell! : node.op === "io" ? defs.node_io! : defs.node_freespace!;
  for (const node of sampleNodes()) {
    for (const group of groupsForNode(node, context)) {
      for (const spec of group.fields) {
        if (spec.kind === "note" || spec.path.startsWith("_seam.")) continue;
        assert.ok(
          resolvePath(schema, defFor(node), spec.path.split(".")),
          `${node.op}: ${spec.path} is not in the schema`,
        );
      }
    }
  }
});

function spec(node: ProgramV2Node, path: string): FieldSpec {
  const found = groupsForNode(node, context).flatMap((group) => group.fields).find((candidate) => candidate.path === path);
  assert.ok(found, `${node.op} has ${path}`);
  return found;
}

test("a standoff is written whole and cleared whole", () => {
  const node = freespaceNode("transit") as FreespaceNode;
  applyEdit(node, spec(node, "arrive_standoff.distance_mm"), 12);
  assert.deepEqual(node.arrive_standoff, { direction: "workpiece_z", distance_mm: 12 });
  applyEdit(node, spec(node, "arrive_standoff.direction"), "torch_axis");
  assert.deepEqual(node.arrive_standoff, { direction: "torch_axis", distance_mm: 12 });
  applyEdit(node, spec(node, "arrive_standoff.distance_mm"), null);
  assert.equal(node.arrive_standoff, undefined);
  assert.deepEqual(nodeProblems(node), []);
});

test("an override cleared inherits; an empty overrides object is removed", () => {
  const node = freespaceNode("approach") as FreespaceNode;
  applyEdit(node, spec(node, "overrides.clearance_min_mm"), 40);
  assert.deepEqual(node.overrides, { clearance_min_mm: 40 });
  applyEdit(node, spec(node, "overrides.clearance_min_mm"), null);
  assert.equal(node.overrides, undefined);
  assert.throws(() => applyEdit(node, spec(node, "overrides.speed_scale"), 2), /at most 1/);
});

test("a scalar field wraps as a constant; a points edit marks a classic weld hand-modified", () => {
  const posed = sampleNodes()[0] as WeldNode;
  applyEdit(posed, spec(posed, "travel_speed_mm_s"), 12.5);
  assert.deepEqual(posed.travel_speed_mm_s, { kind: "constant", value: 12.5 });
  assert.equal(classicProvenance(posed)!.cad_derived, true);
  assert.equal(posed.geometry.kind, "posed_polyline");
  const points: PosedPoint[] = [
    { xyz_m: [0, 0, 0], rpy_rad: [0, 0, 0] },
    { xyz_m: [0.2, 0, 0], rpy_rad: [0, 0, 0] },
  ];
  applyEdit(posed, spec(posed, "geometry.points"), points);
  assert.equal(classicProvenance(posed)!.cad_derived, false);
  assert.throws(() => applyEdit(posed, spec(posed, "geometry.points"), points.slice(0, 1)), /at least two/);
  assert.deepEqual(nodeProblems(posed), []);
});

test("node problems come from the contract and name the node's own defect", () => {
  const posed = sampleNodes()[0] as WeldNode;
  (posed.weld_parameters as Record<string, unknown>).voltage_v = 24;
  assert.match(nodeProblems(posed).join(" "), /weld_parameters/);
  const approach = freespaceNode("approach") as FreespaceNode;
  approach.depart_standoff = { direction: "workpiece_z", distance_mm: 5 };
  assert.match(nodeProblems(approach).join(" "), /nothing is welded before/);
  for (const node of sampleNodes()) assert.deepEqual(nodeProblems(node), [], node.op);
});


test("taught destination edits preserve captured evidence and convert displayed units", () => {
  const pose = { frame_id: "world" as const, xyz_m: [1, 2, 3] as [number, number, number], rpy_rad: [0, 0, 0] as [number, number, number], joint_names: ["J1"], joint_values_rad: [0] };
  const node: import("../contracts/robot-v4-program-v2.ts").MoveNode = {
    id: "p1", op: "move", enabled: true, label: "P1", comment: "", row_hash: "pending:p1",
    motion: "joint", target: structuredClone(pose), speed_scale: .1, speed_mm_s: 25,
    capture: { source: "preview", observed_at: "2026-09-25T00:00:00Z", model_id: "rosie_1420_v1", robot_description_sha256: "sha256:" + "1".repeat(64), pose: structuredClone(pose) },
  };
  const original = structuredClone(node.capture);
  applyEdit(node, spec(node, "target.joint_values_rad.0"), 90);
  assert.equal(node.target.joint_values_rad[0], Math.PI / 2);
  assert.ok(Array.isArray(node.target.joint_values_rad));
  applyEdit(node, spec(node, "motion"), "linear");
  applyEdit(node, spec(node, "constant_tcp_speed"), false);
  assert.equal(node.constant_tcp_speed, false);
  applyEdit(node, spec(node, "constant_tcp_speed"), true);
  assert.equal(node.constant_tcp_speed, true);
  applyEdit(node, spec(node, "target.xyz_m.0"), 1250);
  assert.equal(node.target.xyz_m[0], 1.25);
  assert.ok(Array.isArray(node.target.xyz_m));
  assert.deepEqual(node.capture, original);
  assert.deepEqual(nodeProblems(node), []);
  applyEdit(node, spec(node, "motion"), "circular");
  assert.deepEqual(nodeProblems(node), []); // Missing via is an authoring state, blocked by planning.
  assert.equal(node.target_space, "cartesian");
  applyEdit(node, spec(node, "motion"), "joint");
  applyEdit(node, spec(node, "target.rpy_rad.2"), 45);
  assert.equal(node.target.rpy_rad[2], Math.PI / 4);
  assert.equal(node.target_space, "cartesian");
  applyEdit(node, spec(node, "target.joint_values_rad.0"), 20);
  assert.equal(node.target_space, "joint");
  assert.throws(() => applyEdit(node, spec(node, "speed_mm_s"), 0), /at least/);
  applyEdit(node, spec(node, "speed_scale"), 50);
  assert.equal(node.speed_scale, 0.5);
  assert.equal(node.acceleration_scale, 0.1, "editing speed preserves a legacy move's acceleration");
  applyEdit(node, spec(node, "acceleration_scale"), 25);
  assert.equal(node.acceleration_scale, 0.25);
  assert.equal(node.speed_scale, 0.5);
  assert.deepEqual(nodeProblems(node), []);
  for (const invalid of [0, 101, NaN]) {
    assert.throws(() => applyEdit(node, spec(node, "acceleration_scale"), invalid));
  }
  assert.deepEqual(node.capture, original);
});
