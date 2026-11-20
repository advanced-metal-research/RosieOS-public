// Trace follows one immutable program trajectory. Direction selects a range
// in that trajectory. An off-path start uses an explicit native TCP entry to
// its taught endpoint; subsequent legs keep the original program geometry.
import { currentProject, view, select, traceSelectionGap } from "./session.ts";
import { machineStatus } from "./machine.ts";
import { denseLoadIdentity, planRequestBody, shippedTooling } from "../../../../offline-programming/v1/ui/src/rails/planRequest.ts";
import { effectiveNodes } from "../../../../offline-programming/v1/ui/src/program-v2/nodes.ts";
import { denseSegmentNodeId } from "../../../../offline-programming/v1/ui/src/program-v2/dense-cursor.ts";
import { hasWorkpieceGeometry } from "../../../../offline-programming/v1/ui/src/domain/placement.ts";
import { workcellFixturesWire } from "../../../../offline-programming/v1/ui/src/domain/workcell.ts";
import { environmentFixtures } from "../../../../offline-programming/v1/ui/src/domain/workpieces.ts";

import { computeProgramV2Digest } from "../../../../offline-programming/v1/ui/src/contracts/robot-v4-program-v2.ts";
import { toProgramV2Document } from "../../../../offline-programming/v1/ui/src/program-v2/from-project.ts";

type Boundary = { id: string; end: number };
type Leg = { from: string; to: string; first: number; last: number; selection: string; complete: boolean; endpointOnly?: boolean; deleted?: string };
let prepared: { key: string; cursorGeometry: string; digest: string; load: Record<string, unknown>; boundaries: Boundary[]; ready: boolean } | null = null;
let leg: Leg | null = null;
let departedSelection: string | null = null;
let departedKey = "";
let departedGeometry = "";
// Project revisions are immutable; selection/status polls keep the same object.
// Serialize authoring state once per revision, not on every 20 ms hold tick.
let keyProject: ReturnType<typeof currentProject> | null = null;
let programKey = "";

/** A joystick move invalidates the cursor, not the immutable program path. */
export function traceForget(jogged = false): void {
  if (jogged) {
    departedSelection = view().selected ?? null; departedKey = traceKey();
    departedGeometry = cursorGeometry();
    // Keep both endpoints when manually leaving a partially traced leg.
    if (leg && !leg.complete && leg.selection === departedSelection) return;
  }
  leg = null;
  if (!jogged) { departedSelection = null; departedGeometry = ""; }
}
export function traceKey(): string {
  const p = currentProject(), rt = machineStatus()?.rtCore;
  if (p !== keyProject) {
    // This is a local draft-change key, not an executable-program digest.
    // Empty/incomplete teaching drafts must survive background UI refresh;
    // planning and the native load path still validate executable programs.
    programKey = JSON.stringify([p.identity.id, p.identity.revision, p.denseTrajectory?.trajectoryDigest,
      toProgramV2Document(p)]);
    keyProject = p;
  }
  return programKey + JSON.stringify([rt?.observationIdentity, rt?.coordinateIdentity]);
}

/** Preparation runs before a D-pad press and never requests arm or motion. */
export async function tracePlanRequest() {
  const p = currentProject(), key = traceKey(), record = p.denseTrajectory;
  if (!effectiveNodes(p.nodes).some(n => n.enabled !== false && !["io", "dwell"].includes(n.op)))
    throw new Error("Record a waypoint before tracing");
  if (record && record.sourceProgramDigest === await computeProgramV2Digest(toProgramV2Document(p))) {
    return { key, load: { ...denseLoadIdentity(record, true), dry_run: true }, segments: record.segments.map(s => ({
      index: s.index, kind: s.kind, source_id: s.sourceId,
    })) };
  }
  if (hasWorkpieceGeometry(p)) throw new Error("Plan this program with its workpiece in OLP before tracing");
  // Trace-only artifacts never enter the saved production plan record. Keep
  // all authored geometry and delays, but request no outputs or sensor waits.
  const planning = await planRequestBody({ ...p, program: { ...p.program, executionMode: "dry_run" } }, {
    tooling: shippedTooling(), stepBase64: "", stepFilename: "part.step",
    fixtures: workcellFixturesWire(environmentFixtures(p), new Map()), freeSpaceBackend: "bspline",
    planFromProgramStart: true,
  });
  return { key, planning };
}

/** Speed edits require a new verified trajectory, but do not move the TCP
 * to the highlighted row. Compare all other authored inputs conservatively;
 * this is only cursor continuity, never permission to reuse old plan bytes. */
function cursorGeometry(): string {
  const p = currentProject(), rt = machineStatus()?.rtCore;
  const { program_revision, base_program_revision, program_digest, program_state, created_utc, speed_scale, nodes, producer,
    ...geometry } = toProgramV2Document(p);
  const paths = nodes.map(node => {
    if (node.op !== "move") return node;
    const { speed_scale, speed_mm_s, acceleration_scale, constant_tcp_speed, ...path } = node;
    return path;
  });
  // The project input's ref embeds the revision; identity is already explicit.
  const inputs = producer.inputs.filter(input => input.kind !== "project");
  return JSON.stringify([p.identity.id, p.robot.cell, geometry, { ...producer, inputs }, paths, environmentFixtures(p),
    rt?.observationIdentity, rt?.coordinateIdentity]);
}

export function tracePrepared(key: string, result: any): Record<string, unknown> {
  if (key !== traceKey()) throw new Error("Program or robot identity changed during Trace preparation");
  const d = result.dense;
  const load = result.load ?? (d && {
    dry_run: true, trajectory_digest: d.trajectory_digest, plan_id: d.plan_id,
    program_id: d.program_id, program_digest: d.program_digest,
    manifest_revision: d.manifest_revision, plan_revision: d.plan_revision,
    dense_blob_base64: result.dense_blob_base64,
  });
  const segments = result.segments ?? d?.segments;
  if (!load || !Array.isArray(segments) || !segments.length) throw new Error("Trace needs a complete program trajectory");
  const nodes = effectiveNodes(currentProject().nodes).filter(n => n.enabled !== false);
  const boundaries: Boundary[] = [];
  for (const s of segments) {
    const id = denseSegmentNodeId(nodes, s.kind, s.source_id);
    if (!id || s.kind === "dwell") continue;
    const previous = boundaries[boundaries.length - 1];
    if (previous?.id === id) previous.end = s.index;
    else boundaries.push({ id, end: s.index });
  }
  if (!boundaries.length) throw new Error("The planned trajectory has no program waypoints");
  const geometry = cursorGeometry();
  const gap = traceSelectionGap();
  let retained: Leg | null = null;
  if (leg && !leg.complete && leg.selection === view().selected && prepared?.cursorGeometry === geometry &&
      (leg.deleted ?? null) === (gap?.removed ?? null)) {
    const from = boundaries.findIndex(b => b.id === leg!.from), to = boundaries.findIndex(b => b.id === leg!.to);
    if (from >= 0 && to >= from && (from !== to || leg.endpointOnly)) {
      // The new planner may split the same leg differently. Keep endpoints,
      // not old segment indices, and wait for native preparation as usual.
      retained = { ...leg, first: leg.endpointOnly ? boundaries[from]!.end : boundaries[from]!.end + 1,
        last: boundaries[to]!.end };
    }
  }
  prepared = { key, cursorGeometry: geometry, digest: load.trajectory_digest, load, boundaries, ready: false };
  leg = retained;
  if (key !== departedKey && geometry !== departedGeometry) { departedSelection = null; departedGeometry = ""; }
  return load;
}
export function traceReady(key: string): void {
  if (prepared?.key === key && key === traceKey()) prepared.ready = true;
}
export function traceRequest(direction: number, linearMM = 400, angularDeg = 30) {
  if (direction !== 1 && direction !== -1) throw new Error("Invalid trace direction");
  if (!prepared?.ready || prepared.key !== traceKey()) throw new Error("Trace is preparing this program; release the D-pad and try again when ready");
  const selected = view().selected, b = prepared.boundaries;
  const gap = traceSelectionGap();
  if (leg?.deleted && leg.deleted !== gap?.removed) leg = null;
  // The table highlights the successor after deletion. Trace must approach
  // that successor, not skip it; backward approaches the surviving predecessor.
  // At either program edge only the remaining direction is meaningful.
  if (gap && ((direction < 0 && !gap.previous && !gap.restored) || (direction > 0 && !gap.next)))
    throw new Error(direction > 0 ? "Last waypoint reached" : "First waypoint reached");
  // Reversing an unfinished return-to-point resumes the forward leg.
  if (leg?.endpointOnly && direction > 0) leg = null;
  // While stopped between waypoints, either direction stays on the same leg.
  if (!leg || leg.selection !== selected || leg.complete) {
    if (gap) {
      // Undo may restore the deleted source without moving the TCP. Back
      // first returns there; a completed return establishes arrival again.
      const returning = gap.restored && direction < 0;
      const from = b.findIndex(v => v.id === (gap.restored ? gap.removed : gap.previous ?? gap.next));
      const to = returning ? from : b.findIndex(v => v.id === (gap.next ?? gap.previous));
      if (from < 0 || to < from) throw new Error("Deleted waypoint neighbours are not in the planned path");
      leg = { from: b[from]!.id, to: b[to]!.id, first: from === to ? b[from]!.end : b[from]!.end + 1,
        last: b[to]!.end, selection: selected!, complete: false, endpointOnly: from === to, deleted: gap.removed };
    } else {
      const at = b.findIndex(v => v.id === selected);
      if (at < 0) throw new Error("Select a planned motion row before tracing");
      const returning = direction < 0 && departedSelection === selected;
      const next = returning ? at : at + direction;
      if (next < 0 || next >= b.length) throw new Error(direction > 0 ? "Last waypoint reached" : "First waypoint reached");
      const lo = Math.min(at, next), hi = Math.max(at, next);
      leg = { from: b[lo]!.id, to: b[hi]!.id, first: returning ? b[at]!.end : b[lo]!.end + 1, last: b[hi]!.end,
        selection: selected!, complete: false, endpointOnly: returning };
    }
  }
  // Retiming changes no positions or orientation. Native limits may reduce
  // this requested rate; the ordinary Jog percentage scales it while held.
  const incoming = effectiveNodes(currentProject().nodes).find(n => n.id === leg!.to);
  let timeScale = 1;
  if (!Number.isFinite(linearMM) || linearMM <= 0 || linearMM > 1000 ||
      !Number.isFinite(angularDeg) || angularDeg <= 0 || angularDeg > 180) throw new Error("Invalid Jog speed");
  if (incoming?.op === "move") timeScale = incoming.motion === "joint"
    ? 1 / incoming.speed_scale : linearMM / incoming.speed_mm_s;
  if (incoming?.op === "weld") {
    const v = incoming.travel_speed_mm_s;
    const top = v.kind === "constant" ? v.value : v.kind === "piecewise_linear" ? Math.max(...v.knots.map(k => k[1]))
      : Math.max(...(v.kind === "bspline" ? v.control_values : v.values));
    timeScale = linearMM / top;
  }
  if (!Number.isFinite(timeScale) || timeScale <= 0) throw new Error("Invalid program speed");
  return { allow_entry: true, endpoint_only: !!leg.endpointOnly, linear_speed: linearMM / 1000,
    angular_speed: angularDeg * Math.PI / 180, time_scale: timeScale, trajectory_digest: prepared.digest, first_segment: leg.first, last_segment: leg.last,
    reverse: direction < 0 && !leg.endpointOnly, target: direction < 0 ? leg.from : leg.to };
}
export function traceReached(target: string): void {
  if (!leg || (target !== leg.from && target !== leg.to)) return;
  select(target); leg.selection = target; leg.complete = true; departedSelection = null; departedGeometry = "";
}
export function traceCurrent(): boolean {
  return !!prepared?.ready && prepared.key === traceKey() && !!leg && leg.selection === view().selected;
}
