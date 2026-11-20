import assert from "node:assert/strict";
import test from "node:test";

import type { ProgramV2Node, WeldNode } from "../contracts/robot-v4-program-v2.ts";
import { lineProblems } from "../program-v2/nodes.ts";
import { isManualPoseWeld } from "./node.ts";
import { WaypointSession, type WaypointHost } from "./session.ts";

/** A program that is only its node list, with an edit log. */
function program(): WaypointHost & { list: ProgramV2Node[]; edits: string[] } {
  const host = {
    list: [] as ProgramV2Node[],
    edits: [] as string[],
    nodes: () => host.list,
    editNodes(message: string, change: (nodes: readonly ProgramV2Node[]) => ProgramV2Node[]) {
      host.list = change(host.list);
      host.edits.push(message);
    },
    newWeldProcess: () => ({ travelSpeedMmS: 8, weave: { shape: "none" } }),
  };
  return host;
}

test("each waypoint weld is one weld node in a valid line, and there can be several", () => {
  const host = program();
  const session = new WaypointSession(host);
  session.addWaypoint([0, 0, 0], 17);
  session.addWaypoint([9, 9, 9]);
  session.select({ weldId: null });
  session.addWeld("waypoint", [0.1, 0, 0], 17);
  session.addWaypoint([9, 9, 9]);
  const welds = host.list.filter(isManualPoseWeld);
  assert.equal(welds.length, 2);
  assert.deepEqual(welds.map((node) => node.label), ["Waypoint weld 1", "Waypoint weld 2"]);
  assert.deepEqual(welds.map((node) => node.geometry.points.length), [2, 2]);
  assert.deepEqual(host.list.map((node) => node.op), ["approach", "weld", "transit", "weld", "retract"]);
  assert.deepEqual(lineProblems(host.list), []);
  assert.equal(session.welds.length, 2);
  assert.equal(session.selection.weldId, welds[1]!.id);
});

test("a new waypoint is a copy of the selected one, placed right after it", () => {
  const host = program();
  const session = new WaypointSession(host);
  session.addWaypoint([0.01, 0.02, 0.03], 17);
  const first = session.selectedWaypoint!;
  session.setStandoff(first.id, 21);
  session.addWaypoint([9, 9, 9]);
  const last = session.selectedWaypoint!;
  session.select({ waypointId: first.id });
  session.addWaypoint([9, 9, 9]);
  const middle = session.selectedWaypoint!;
  const ids = session.selectedWeld!.waypoints.map((point) => point.id);
  assert.deepEqual(ids, [first.id, middle.id, last.id]);
  assert.deepEqual(middle.positionM, [0.01, 0.02, 0.03]);
  assert.equal(middle.standoffMm, 21);
});

test("edits are program edits: the node carries the pose, and undoing the node undoes the edit", () => {
  const host = program();
  const session = new WaypointSession(host);
  session.addWaypoint([0, 0, 0], 17);
  session.addWaypoint([0, 0, 0]);
  const moved = session.selectedWaypoint!;
  const before = host.list;
  session.setPose(moved.id, [0.05, 0, 0], [0, 0, 0, 1]);
  const weld = host.list.find(isManualPoseWeld) as WeldNode & { geometry: { points: { xyz_m: number[] }[] } };
  assert.deepEqual(weld.geometry.points[1]!.xyz_m, [0.05, 0, 0]);
  assert.match(host.edits.at(-1)!, /P2 moved/);
  host.list = before;
  assert.deepEqual(session.selectedWeld!.waypoints[1]!.positionM, [0, 0, 0]);
  assert.equal(session.selection.waypointId, moved.id);
});

test("a selection the program no longer has is dropped", () => {
  const host = program();
  const session = new WaypointSession(host);
  session.addWaypoint([0, 0, 0], 17);
  session.addWaypoint([0.02, 0, 0]);
  const weldId = session.selection.weldId!;
  session.removeWeld(weldId);
  assert.deepEqual(session.selection, { weldId: null, waypointId: null });
  assert.deepEqual(host.list, []);
});

test("deleting the selected waypoint hands the selection to its neighbour", () => {
  const host = program();
  const session = new WaypointSession(host);
  session.addWaypoint([0, 0, 0], 17);
  session.addWaypoint([0, 0, 0]);
  const [a, b] = session.selectedWeld!.waypoints;
  session.select({ waypointId: a!.id });
  session.removeWaypoint(a!.id);
  assert.equal(session.selection.waypointId, b!.id);
  session.removeWaypoint(b!.id);
  assert.equal(session.selection.waypointId, null);
  assert.equal(session.welds.length, 1, "the weld stays, empty, until it is deleted");
});

function position(host: ReturnType<typeof program>, weldIndex: number, point: number): number[] {
  const weld = host.list.filter(isManualPoseWeld)[weldIndex]!;
  return weld.geometry.points.at(point)!.xyz_m;
}

test("a line is two points and an arc three, and neither grows or shrinks", () => {
  const host = program();
  const session = new WaypointSession(host);
  session.addWeld("line", [0, 0, 0], 17);
  const line = session.selectedWeld!;
  assert.equal(line.tool, "line");
  assert.equal(line.waypoints.length, 2);
  session.addWaypoint([9, 9, 9]);
  session.removeWaypoint(line.waypoints[0]!.id);
  assert.equal(session.selectedWeld!.waypoints.length, 2);

  session.select({ weldId: null });
  session.addWeld("arc", [0, 0, 0], 17);
  const arc = session.selectedWeld!;
  assert.equal(arc.tool, "arc");
  assert.equal(arc.waypoints.length, 3);
  const node = host.list.filter(isManualPoseWeld).at(-1)!;
  assert.equal(node.geometry.shape, "arc");
  // A usable arc from the start: a quarter circle, not three points in a row.
  const [a, b, c] = arc.waypoints.map((point) => point.positionM);
  const cross = (b![0] - a![0]) * (c![1] - a![1]) - (b![1] - a![1]) * (c![0] - a![0]);
  assert.ok(Math.abs(cross) > 1e-5);
});

test("a new weld starts at the selected weld's end and follows it", () => {
  const host = program();
  const session = new WaypointSession(host);
  session.addWeld("line", [0, 0, 0], 17);
  const first = session.selectedWeld!;
  session.addWeld("arc", [9, 9, 9], 17);
  const second = session.selectedWeld!;
  assert.equal(second.startFrom, first.id);
  assert.deepEqual(position(host, 1, 0), position(host, 0, -1));
  session.addWeld("line", [9, 9, 9], 17);
  const third = session.selectedWeld!;
  assert.equal(third.startFrom, second.id);

  // Move the first line's end: the arc's start follows, and so does the
  // line after the arc once the arc's end moves.
  session.setPose(first.waypoints[1]!.id, [0.05, 0.01, 0], [0, 0, 0, 1]);
  assert.deepEqual(position(host, 1, 0), [0.05, 0.01, 0]);
  const arcEnd = session.welds[1]!.waypoints[2]!;
  session.setPose(arcEnd.id, [0.08, 0.04, 0.01], [0, 0, 0, 1]);
  assert.deepEqual(position(host, 2, 0), [0.08, 0.04, 0.01]);
  // One program edit per move, the followers included.
  assert.match(host.edits.at(-1)!, /P3 moved/);
});

test("a tied start only turns, and untying it frees it", () => {
  const host = program();
  const session = new WaypointSession(host);
  session.addWeld("line", [0, 0, 0], 17);
  session.addWeld("line", [9, 9, 9], 17);
  const tied = session.selectedWeld!;
  const start = tied.waypoints[0]!;
  assert.ok(session.isTied(start.id));
  session.setPose(start.id, [0.3, 0.3, 0.3], [0, 0, 0.6, 0.8]);
  const after = session.selectedWeld!.waypoints[0]!;
  assert.deepEqual(after.positionM, start.positionM);
  assert.ok(Math.abs(after.quaternion[2] - 0.6) < 1e-9, "the orientation still turned");
  session.setStartFrom(tied.id, null);
  assert.ok(!session.isTied(start.id));
  session.setPose(start.id, [0.3, 0.3, 0.3], after.quaternion);
  assert.deepEqual(session.selectedWeld!.waypoints[0]!.positionM, [0.3, 0.3, 0.3]);
});

test("links never close a loop, and a deleted source frees its follower", () => {
  const host = program();
  const session = new WaypointSession(host);
  session.addWeld("line", [0, 0, 0], 17);
  const a = session.selectedWeld!;
  session.addWeld("line", [0, 0, 0], 17);
  const b = session.selectedWeld!;
  // a -> b would close a -> b -> a.
  assert.deepEqual(session.startCandidates(a.id).map((weld) => weld.id), []);
  session.setStartFrom(a.id, b.id);
  assert.equal(session.welds[0]!.startFrom, null);
  session.removeWeld(a.id);
  assert.equal(session.welds[0]!.startFrom, null);
  assert.equal(session.welds[0]!.id, b.id);
});
