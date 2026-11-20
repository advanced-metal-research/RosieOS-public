import assert from "node:assert/strict";
import test from "node:test";

import type { CadWorkpieceMesh } from "../cad/contracts.ts";
import { createProject } from "./project.ts";
import { migrateProject } from "./serialization.ts";
import {
  defaultWorkcellFixtures,
  editWorkcellFixture,
  fixtureSublabel,
  newStepFixture,
  WorkcellFixtureUnavailable,
  workcellFixturesWire,
} from "./workcell.ts";

test("repeated fixture imports retain the stand and round-trip with distinct identities", () => {
  const project = createProject({ robotModel: "rosie_1420_v1" });
  const originalStand = structuredClone(project.workcellFixtures[0]);
  const source = { filename: "clamp.step", fingerprint: "clamp" };
  const first = newStepFixture(project.workcellFixtures, source);
  project.workcellFixtures.push(first);
  const second = newStepFixture(project.workcellFixtures, source);
  project.workcellFixtures.push(second);
  assert.notEqual(first.id, second.id);
  assert.equal(first.name, "clamp.step");
  assert.deepEqual(first.translationM, [0, 0, 0]);
  assert.deepEqual(first.rotationRpyRad, [0, 0, 0]);
  assert.equal(first.origin, "authored");
  assert.deepEqual(project.workcellFixtures[0], originalStand);
  assert.deepEqual(migrateProject(JSON.parse(JSON.stringify(project))).workcellFixtures, project.workcellFixtures);
  const mesh = {
    fingerprint: "clamp",
    positionsM: new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0]),
    indices: new Uint32Array([0, 1, 2]),
  } as unknown as CadWorkpieceMesh;
  const wire = workcellFixturesWire(project.workcellFixtures, new Map([["clamp", mesh]]));
  assert.equal(wire.length, 3);
  assert.notEqual(wire[1]!.id, wire[2]!.id);
  assert.deepEqual(wire[1]!.shape, wire[2]!.shape);
});

// The default stand IS the weld planner's placeholder (seam_worker/robot/cell.py
// CELL_FIXTURES): a 0.5 m square box 1 m tall, footprint centred at x 1.0,
// y 1.0 on the floor. A program that never touches it plans as before; the
// two tools only agree because these numbers agree.
test("the 1420 default stand equals the planner's placeholder and the 1400 has none", () => {
  const [stand] = defaultWorkcellFixtures("rosie_1420_v1");
  assert.deepEqual(stand, {
    id: "stand",
    name: "Stand",
    shape: { kind: "box", sizeM: [0.5, 0.5, 1.0] },
    translationM: [1.0, 1.0, 0.0],
    rotationRpyRad: [0, 0, 0],
    origin: "cell-default",
  });
  assert.deepEqual(defaultWorkcellFixtures("rosie_1400_v3"), []);
  assert.equal(fixtureSublabel(stand!), "500 × 500 × 1000 mm · at 1000, 1000, 0 mm · placeholder");
});

test("a new 1420 program carries the stand and an older saved one gains it on load", () => {
  const project = createProject({ robotModel: "rosie_1420_v1" });
  assert.equal(project.workcellFixtures.length, 1);
  const saved = JSON.parse(JSON.stringify(project)) as Record<string, unknown>;
  delete saved.workcellFixtures;
  const loaded = migrateProject(saved);
  assert.deepEqual(loaded.workcellFixtures, defaultWorkcellFixtures("rosie_1420_v1"));
  // An operator's edit survives the load untouched.
  const edited = { ...project, workcellFixtures: [editWorkcellFixture(project.workcellFixtures[0]!, "sz", 700)] };
  assert.deepEqual(migrateProject(JSON.parse(JSON.stringify(edited))).workcellFixtures, edited.workcellFixtures);
});

test("edits are in millimetres and degrees, bounded, and mark the fixture authored", () => {
  const [stand] = defaultWorkcellFixtures("rosie_1420_v1");
  const taller = editWorkcellFixture(stand!, "sz", 850);
  assert.equal(taller.shape.kind === "box" && taller.shape.sizeM[2], 0.85);
  assert.equal(taller.origin, "authored");
  const moved = editWorkcellFixture(taller, "x", 1442.3);
  assert.equal(moved.translationM[0], 1.4423);
  const turned = editWorkcellFixture(moved, "rz", 90);
  assert.ok(Math.abs(turned.rotationRpyRad[2] - Math.PI / 2) < 1e-12);
  // The original is untouched, and a side cannot go to zero or a hall.
  assert.deepEqual(stand!.shape, { kind: "box", sizeM: [0.5, 0.5, 1.0] });
  const side = (fixture: typeof stand, index: number): number => {
    const shape = fixture!.shape;
    return shape.kind === "box" ? shape.sizeM[index]! : Number.NaN;
  };
  assert.equal(side(editWorkcellFixture(stand!, "sx", 0), 0), 0.001);
  assert.equal(side(editWorkcellFixture(stand!, "sy", 99_999), 1), 5);
  assert.equal(editWorkcellFixture(stand!, "z", Number.NaN), stand);
});

test("the wire carries a box as its sides and a STEP as the mesh this session holds", () => {
  const [stand] = defaultWorkcellFixtures("rosie_1420_v1");
  const stepStand = {
    ...stand!,
    shape: { kind: "step" as const, fingerprint: "ABCD", filename: "stand.step" },
    translationM: [1.2, 0.3, 0] as [number, number, number],
  };
  const mesh = {
    fingerprint: "abcd",
    positionsM: new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0]),
    indices: new Uint32Array([0, 1, 2]),
  } as unknown as CadWorkpieceMesh;
  const wire = workcellFixturesWire([stand!, stepStand], new Map([["abcd", mesh]]));
  assert.deepEqual(wire[0], {
    id: "stand", name: "Stand",
    shape: { kind: "box", size_m: [0.5, 0.5, 1.0] },
    xyz_m: [1.0, 1.0, 0.0], rpy_rad: [0, 0, 0],
  });
  assert.equal(wire[1]!.shape.kind, "mesh");
  if (wire[1]!.shape.kind === "mesh") {
    assert.deepEqual(wire[1]!.shape.vertices, [0, 0, 0, 1, 0, 0, 0, 1, 0]);
    assert.deepEqual(wire[1]!.shape.faces, [0, 1, 2]);
    assert.equal(wire[1]!.shape.filename, "stand.step");
  }
  assert.deepEqual(wire[1]!.xyz_m, [1.2, 0.3, 0]);
  // A STEP whose mesh is not here cannot be sent as nothing.
  assert.throws(() => workcellFixturesWire([stepStand], new Map()), WorkcellFixtureUnavailable);
});
