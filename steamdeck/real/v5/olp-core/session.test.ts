import assert from "node:assert/strict";
import test from "node:test";
import { seedRobotCatalogue } from "../../../../offline-programming/v1/ui/test/support/robot-catalogue.ts";
import { robotCatalogue, setRobotCatalogue } from "../../../../offline-programming/v1/ui/src/domain/robots.ts";
import * as session from "./session.ts";

seedRobotCatalogue();

const joints = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"];
const pose = (x: number) => ({
  frame_id: "world" as const, xyz_m: [x, 0.7, 0.9] as [number, number, number], rpy_rad: [Math.PI, 0.6, 3.1] as [number, number, number],
  joint_names: joints, joint_values_rad: joints.map((_, i) => i < 6 ? 0.1 * i : 0),
});
const capture = (x: number, description: string) => ({
  source: "machine" as const, observed_at: "2026-09-27T12:00:00Z", model_id: "rosie_1400_v3",
  robot_description_sha256: description, cell_id: "local-simulation", pose: pose(x),
});

test("a pendant program is OLP's: recorded, edited, undone and exchanged as an OLP project export", () => {
  session.newProject("Deck teach", "rosie_1400_v3");
  const pinned = session.view().description!;
  assert.match(pinned, /^sha256:/);

  const first = session.record("new", pose(1.6), capture(1.6, pinned), "move-1");
  session.record("new", pose(1.7), capture(1.7, pinned), "move-2");
  let view = session.view();
  assert.deepEqual(view.rows.map((row) => [row.label, row.freespace]), [["P1", "MoveJ"], ["P2", "MoveJ"]]);
  assert.equal(view.selected, "move-2");
  assert.equal(view.rows[1]!.cells["target.xyz_m.0"], 1700);

  // Via makes the selection circular; reteach replaces its destination.
  session.record("via", pose(1.65), capture(1.65, pinned), "unused");
  session.record("reteach", pose(1.75), capture(1.75, pinned), "unused");
  view = session.view();
  assert.equal(view.rows[1]!.freespace, "MoveC");
  assert.equal(view.rows[1]!.cells["target.xyz_m.0"], 1750);

  // A dwell goes after the selection; a Home always closes the program.
  session.select(first);
  const dwell = session.addEvent("dwell");
  session.addEvent("home");
  assert.deepEqual(session.view().rows.map((row) => row.op), ["move", "dwell", "move", "home"]);

  // Inline and dialog edits go through OLP's field specs in display units.
  session.edit("move-1", "speed_mm_s", 40);
  session.edit("move-1", "target.xyz_m.2", 950);
  session.edit(dwell, "duration_s", 1.5);
  assert.throws(() => session.edit("move-1", "speed_scale", 250), /100/);
  const fields = session.fields("move-1");
  const tcp = fields.groups.flatMap((group) => group.fields).find((field) => field.path === "speed_mm_s")!;
  assert.equal(tcp.value, 40);

  const beforeUndo = session.view().rows[0]!.cells["target.xyz_m.2"];
  session.undo();
  session.undo();
  assert.equal(session.view().rows[0]!.cells["target.xyz_m.2"], 900);
  session.redo();
  assert.equal(session.view().rows[0]!.cells["target.xyz_m.2"], beforeUndo);

  session.move(dwell, 1);
  assert.deepEqual(session.view().rows.map((row) => row.op), ["move", "move", "dwell", "home"]);
  // Whatever OLP's moveBlocked reports is refused: nothing moves past the end.
  const home = session.view().rows.at(-1)!;
  assert.ok(home.downBlocked);
  assert.throws(() => session.move(home.id, 1));
  session.enable("move-2", false);
  assert.equal(session.view().rows.find((row) => row.id === "move-2")!.enabled, false);

  // What the pendant saves is what desktop OLP opens, node for node.
  const exported = session.exportProject();
  assert.match(exported, /"schema": "offline-programming.project-export.v1"/);
  const nodesBefore = JSON.parse(exported).project.nodes;
  session.openProject(exported);
  assert.deepEqual(JSON.parse(session.exportProject()).project.nodes, nodesBefore);
  assert.equal(session.view().rows.length, 4);
});

test("the program follows the connected cell: an empty one is started again for its robot, a taught one is never converted", () => {
  // Before any cell: the last connected model the server still describes, else the catalogue's first.
  assert.equal(session.startupModel("rosie_1420_v1"), "rosie_1420_v1");
  assert.equal(session.startupModel("retired_model"), "rosie_1400_v3");
  assert.equal(session.startupModel(""), "rosie_1400_v3");

  session.newProject("New pendant program", "rosie_1400_v3");
  assert.deepEqual(session.followCellModel(""), { action: "none", message: "" });
  assert.equal(session.followCellModel("rosie_1400_v3").action, "match");
  const repinned = session.followCellModel("rosie_1420_v1");
  assert.equal(repinned.action, "repinned");
  assert.match(repinned.message, /"New pendant program" is now for rosie_1420_v1.*it was for rosie_1400_v3/);
  assert.equal(session.view().model, "rosie_1420_v1");
  assert.equal(session.view().name, "New pendant program");
  assert.match(session.view().description!, /^sha256:/);
  assert.equal(session.modelMismatch("rosie_1420_v1"), "");

  const pinned = session.view().description!;
  session.record("new", pose(1.6), { ...capture(1.6, pinned), model_id: "rosie_1420_v1" }, "move-1");
  const project = () => JSON.parse(session.exportProject()).project;
  const before = project();
  const mismatch = session.followCellModel("rosie_1400_v3");
  assert.equal(mismatch.action, "mismatch");
  assert.equal(mismatch.message, "This program is for rosie_1420_v1; the connected cell is rosie_1400_v3. New starts a program for it, Open picks another");
  assert.equal(session.modelMismatch("rosie_1400_v3"), mismatch.message);
  assert.deepEqual(project(), before, "a taught program is left exactly as it was");
});

test("a program export names its identity; text OLP cannot open names none", () => {
  session.newProject("Identity", "rosie_1400_v3");
  const id = session.projectIdentity();
  assert.ok(id);
  assert.equal(session.projectIdentityOf(session.exportProject()), id);
  assert.equal(session.projectIdentityOf("{ not a program"), "");
  assert.throws(() => session.newProject("No robot", ""), /listed no robot/);
});

test("a robot-specific description names its meshes by the product package, as OLP's viewer maps them", () => {
  assert.deepEqual(session.meshPackages("rosie_1420_v1"), ["rosie_1420_v1"]);
  // A Rosie 1430 arm's description is its own and its meshes the product's:
  // the server's catalogue says so (urdf_package), and the viewer maps by it.
  const previous = robotCatalogue();
  const template = previous.find(entry => entry.modelId === "rosie_1420_v1")!;
  setRobotCatalogue([...previous, { ...template, modelId: "example_1430_arm", urdfPackage: "rosie_1430_v1" }]);
  try {
    assert.deepEqual(session.meshPackages("example_1430_arm"), ["rosie_1430_v1"]);
  } finally {
    setRobotCatalogue(previous);
  }
});


test("record drafts do not save until accepted, inherit settings but never geometry, and undo once", () => {
  session.newProject("Draft teaching", "rosie_1400_v3");
  const pinned = session.view().description!;
  session.record("new", pose(1), capture(1, pinned), "first");
  session.edit("first", "motion", "linear");
  session.edit("first", "speed_mm_s", 12);
  session.edit("first", "speed_scale", 25);
  const before = JSON.parse(session.exportProject()).project;
  session.prepareRecord(pose(2), capture(2, pinned), "draft");
  session.editPendingRecord("", "speed_mm_s", 17);
  assert.deepEqual(JSON.parse(session.exportProject()).project, before);
  session.cancelPendingRecord();
  assert.deepEqual(JSON.parse(session.exportProject()).project, before);
  session.prepareRecord(pose(3), capture(3, pinned), "saved");
  session.commitPendingRecord();
  let node = JSON.parse(session.exportProject()).project.nodes.at(-1);
  assert.equal(node.motion, "linear"); assert.equal(node.speed_mm_s, 12); assert.equal(node.speed_scale, .25);
  assert.equal(node.target.xyz_m[0], 3); assert.equal(node.via, undefined);
  session.undo(); assert.deepEqual(JSON.parse(session.exportProject()).project.nodes, before.nodes);
  session.prepareRecord(pose(4), capture(4, pinned), "stale");
  session.addEvent("dwell");
  assert.throws(() => session.commitPendingRecord(), /Program changed/);
  session.cancelPendingRecord();
});

test("I/O actions and weld arrival annotations survive save/edit without duplicating a weld start", () => {
  session.newProject("I/O teaching", "rosie_1400_v3");
  const description=session.view().description!;
  const first=session.record("new",pose(1.6),capture(1.6,description),"weld-start");
  session.edit(first,"weld_transition","start");
  assert.match(session.view().rows[0]!.label,/WELD START/);
  const second=session.record("new",pose(1.7),capture(1.7,description),"weld-end");
  const weldField=()=>session.fields(second).groups.flatMap(g=>g.fields).find(f=>f.path==="weld_transition")!;
  assert.equal(weldField().value,"none");
  session.edit(second,"motion","linear"); session.edit(second,"weld_transition","stop");
  const io=session.addEvent("io");
  const action=()=>session.fields(io).groups.flatMap(g=>g.fields).find(f=>f.path==="action")!;
  assert.equal(action().value,"set");
  session.edit(io,"action","wait"); session.edit(io,"channel","I14");
  session.edit(io,"timeout_s",3); session.edit(io,"stable_s",.1);
  const saved=session.exportProject(); session.openProject(saved);
  assert.equal(action().value,"wait");
  assert.equal(weldField().value,"stop");
  session.edit(io,"action","pulse"); session.edit(io,"channel","O14");
  const fields=session.fields(io).groups.flatMap(g=>g.fields);
  assert.ok(fields.some(f=>f.path==="duration_s"));
  assert.ok(!fields.some(f=>f.path==="timeout_s"));
});

test("cell weld defaults become explicit waypoint values and remain editable", () => {
 session.newProject("Weld timing", "rosie_1400_v3");
 const description=session.view().description!;
 session.setWeldDefaults({start_delay_s:.3,stop_delay_s:.4});
 session.prepareRecord(pose(1),capture(1,description),"start-delay");
 session.editPendingRecord("","weld_transition","start");
 assert.equal(session.pendingRecordFields().groups.flatMap(g=>g.fields).find(f=>f.path==="weld_start_delay_s")?.value,.3);
 session.editPendingRecord("","weld_start_delay_s",.6);
 session.commitPendingRecord();
 const node=JSON.parse(session.exportProject()).project.nodes[0];
 assert.equal(node.weld_start_delay_s,.6);assert.equal(node.weld_stop_delay_s,.4);
 session.setWeldDefaults({});
 assert.equal(JSON.parse(session.exportProject()).project.nodes[0].weld_start_delay_s,.6);
});


test("compact teaching groups keep weld actions in motion and default new weld legs to constant TCP", () => {
 session.newProject("Weld defaults", "rosie_1400_v3");
 const description=session.view().description!;
 session.record("new",pose(1),capture(1,description),"start");
 session.edit("start","motion","linear");session.edit("start","constant_tcp_speed",false);
 session.edit("start","weld_transition","start");
 let node=JSON.parse(session.exportProject()).project.nodes[0];
 assert.equal(node.constant_tcp_speed,true);
 // A travel profile is never copied as variable-speed into a new weld leg.
 session.edit("start","constant_tcp_speed",false);
 session.prepareRecord(pose(2),capture(2,description),"end");
 const groups=session.pendingRecordFields().groups;
 assert.equal(groups[0].id,"motion");assert.equal(groups.length,4);
 assert.ok(groups[0].fields.some(f=>f.path==="weld_transition"));
 assert.equal(groups[0].fields.find(f=>f.path==="constant_tcp_speed")?.value,true);
 session.editPendingRecord("","constant_tcp_speed",false);
 session.editPendingRecord("","weld_transition","stop");
 assert.equal(session.pendingRecordFields().groups[0].fields.find(f=>f.path==="constant_tcp_speed")?.value,true);
 assert.ok(session.pendingRecordFields().groups[0].fields.some(f=>f.path==="weld_stop_delay_s"));
 session.commitPendingRecord();
 node=JSON.parse(session.exportProject()).project.nodes[1];assert.equal(node.constant_tcp_speed,true);
 session.prepareRecord(pose(3),capture(3,description),"cancelled");session.cancelPendingRecord();
 assert.equal(JSON.parse(session.exportProject()).project.nodes.length,2);
});
