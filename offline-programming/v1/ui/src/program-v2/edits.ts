/**
 * The spreadsheet's edits as transforms of a draft project. Desktop OLP runs
 * them inside `mutate` (one revision, one undo step, rows recompiled); the
 * Steam Deck pendant runs the same functions in its embedded engine, so a
 * program edited on either behaves identically on the other. Nothing here
 * reads a selection or the UI: each caller passes what it has selected.
 */
import type { AuthoredSeam, OfflineProgrammingProject, ProjectChange } from "../domain/project.ts";
import type { MoveNode, ProgramV2Node, TeachPose } from "../contracts/robot-v4-program-v2.ts";
import type { SeamEdits } from "./node-editor.ts";
import { rowsFromSeams } from "../adapters/cadquery.ts";
import { documentProblems, refreshClassicNodes, syncClassicNodes } from "./from-project.ts";
import { RECORDED_MOVE_SPEED_MM_S } from "./move-defaults.ts";
import { freespaceNode, isFreespace, moveBlocked, moveNode, nodeById, removeNode } from "./nodes.ts";
import { rowsFromNodes, rowsOfNode } from "./rows-from-nodes.ts";

export type RecordMode = "new" | "reteach" | "via";


/**
 * Nodes are the authored program and rows are compiled from them. After an
 * authoring change the classic nodes follow their seams (a placement, trim
 * or process change moves CAD-derived points; a hand-edited weld keeps its
 * own), then every row is recompiled. A rails program's seam welds compile
 * on the weld planner, so a node change there empties their rows until the
 * next Compile -- the honest state, and what the status column shows. A
 * legacy program with seams and no nodes keeps the old seam compile.
 */
export function deriveRows(draft: OfflineProgrammingProject, change: ProjectChange, selectedNodeId: string | null): void {
  const authoring = ["fixtures", "seams", "sequence", "process", "nodes"].includes(change);
  if (!authoring) return;
  if (draft.nodes.length > 0 || (draft.seams.length > 0 && !draft.railsAuthoring)) {
    if (!draft.railsAuthoring) {
      syncClassicNodes(draft);
      refreshClassicNodes(draft);
    }
    draft.rows = rowsFromNodes(draft);
    if (!draft.rows.some((row) => row.id === draft.simulation.selectedRowId)) {
      draft.simulation.selectedRowId = rowsOfNode(draft, selectedNodeId ?? "")[0]?.id ?? draft.rows[0]?.id ?? null;
    }
    return;
  }
  if (["fixtures", "seams", "sequence", "process"].includes(change) && draft.seams.length > 0) {
    draft.rows = rowsFromSeams(draft);
  }
}

function assertDocument(draft: OfflineProgrammingProject): void {
  const problems = documentProblems(draft);
  if (problems.length) throw new Error(problems[0]);
}

/**
 * Record, reteach or record-via from one measured (or previewed) pose. A new
 * waypoint goes after the selection -- past the rest of a weld block, never
 * after the closing Home, and a transit it lands on becomes the weld's
 * retract plus an approach to the next weld. Reteach replaces the destination
 * and its capture; via adds the intermediate pose and makes the move circular.
 * Returns the recorded node's id.
 */
export function recordMove(draft: OfflineProgrammingProject, mode: RecordMode, selectedNodeId: string | null,
  target: TeachPose, capture: MoveNode["capture"], newId: string): string {
  const selected = draft.nodes.find((node) => node.id === selectedNodeId);
  if (mode !== "new" && selected?.op !== "move") throw new Error("Select a taught waypoint first");
  if (mode === "new") {
    const node: MoveNode = { id: newId, op: "move", enabled: true, label: `P${draft.nodes.filter(n => n.op === "move").length + 1}`,
      comment: "", row_hash: `pending:${newId}`, motion: "joint", target, speed_scale: 1, acceleration_scale: 1, speed_mm_s: RECORDED_MOVE_SPEED_MM_S, capture };
    let at = draft.nodes.findIndex(node => node.id === selectedNodeId);
    let precedingMotion = at;
    while (precedingMotion >= 0 && ["dwell", "io"].includes(draft.nodes[precedingMotion]!.op)) precedingMotion--;
    if (precedingMotion >= 0 && ["weld", "approach"].includes(draft.nodes[precedingMotion]!.op)) {
      while (at + 1 < draft.nodes.length && !["retract", "transit"].includes(draft.nodes[at]!.op)) at++;
    }
    const transit = draft.nodes[at];
    if (transit?.op === "transit") {
      const approach = freespaceNode("approach", { arrive: transit.arrive_standoff });
      transit.op = "retract"; transit.label = "RETRACT"; delete transit.arrive_standoff;
      draft.nodes.splice(at + 1, 0, node, approach);
    } else {
      let insertAt = at < 0 ? draft.nodes.length : at + 1;
      if (draft.nodes[insertAt - 1]?.op === "home" && insertAt > 1) insertAt--;
      draft.nodes.splice(insertAt, 0, node);
    }
    assertDocument(draft);
    return newId;
  }
  const node = selected as MoveNode;
  if (mode === "via") { node.via = target; node.motion = "circular"; }
  else { node.target = target; node.capture = capture; delete node.target_space; }
  assertDocument(draft);
  return node.id;
}

/** A dwell or IO goes after the selection; a Home always closes the program. */
export function insertEventNode(draft: OfflineProgrammingProject, node: ProgramV2Node, selectedNodeId: string | null): void {
  if (node.op === "home") {
    draft.nodes = [...draft.nodes, node];
  } else {
    const at = draft.nodes.findIndex((candidate) => candidate.id === selectedNodeId);
    draft.nodes = [...draft.nodes];
    draft.nodes.splice(at < 0 ? draft.nodes.length : at + 1, 0, node);
  }
  assertDocument(draft);
}

/** Approach, transit and retract exist because of their welds. */
export function deleteBlocked(nodes: ProgramV2Node[], nodeId: string): string | null {
  const node = nodeById(nodes, nodeId);
  if (!node) return "node not found";
  if (isFreespace(node) && node.op !== "home") return `${node.op} moves with its weld; delete the weld`;
  return null;
}

/** A classic weld's seam goes with it, so the seam list and the nodes keep agreeing. */
export function deleteProgramNode(draft: OfflineProgrammingProject, nodeId: string): void {
  const blocked = deleteBlocked(draft.nodes, nodeId);
  if (blocked) throw new Error(blocked);
  const provenance = nodeById(draft.nodes, nodeId)?.extensions?.classic as { seam_id?: string } | undefined;
  draft.nodes = removeNode(draft.nodes, nodeId);
  if (provenance?.seam_id) {
    draft.seams = draft.seams.filter((seam) => seam.id !== provenance.seam_id);
    draft.sequence = draft.sequence.filter((step) => step.seamId !== provenance.seam_id);
  }
}

/** Seam order follows weld order so a legacy seam compile would agree. */
export function moveProgramNode(draft: OfflineProgrammingProject, nodeId: string, delta: -1 | 1): void {
  const blocked = moveBlocked(draft.nodes, nodeId, delta);
  if (blocked) throw new Error(blocked);
  draft.nodes = moveNode(draft.nodes, nodeId, delta);
  const weldSeams = draft.nodes.flatMap((node) => {
    const provenance = node.op === "weld" ? node.extensions?.classic as { seam_id?: string } | undefined : undefined;
    return provenance?.seam_id ? [provenance.seam_id] : [];
  });
  draft.sequence.forEach((step) => {
    const order = weldSeams.indexOf(step.seamId);
    if (order >= 0) step.order = order;
  });
}

function seamOf(draft: OfflineProgrammingProject, node: ProgramV2Node | undefined): AuthoredSeam | undefined {
  if (node?.op !== "weld") return undefined;
  const provenance = node.extensions?.classic as { seam_id?: string } | undefined;
  return draft.seams.find((candidate) => candidate.id === provenance?.seam_id);
}

/** A weld's enabled flag is also its seam's, so the seam list agrees. */
export function setNodeEnabled(draft: OfflineProgrammingProject, nodeId: string, enabled: boolean): void {
  const node = draft.nodes.find((candidate) => candidate.id === nodeId);
  if (!node) return;
  node.enabled = enabled;
  const seam = seamOf(draft, node);
  if (seam) seam.enabled = enabled;
}

/** Replace one node with its edited copy, and the weld's seam trim with it. */
export function commitProgramNode(draft: OfflineProgrammingProject, node: ProgramV2Node, seam: SeamEdits | null): void {
  if (!nodeById(draft.nodes, node.id)) throw new Error("the node is no longer in the program");
  if (seam) {
    const target = draft.seams.find((candidate) => candidate.id === seam.seamId);
    if (target) {
      target.trim = { startOffsetM: seam.trimStartM, endOffsetM: seam.trimEndM };
      target.reversed = seam.reversed;
    }
  }
  draft.nodes = draft.nodes.map((candidate) => (candidate.id === node.id ? structuredClone(node) : candidate));
  const owner = seamOf(draft, draft.nodes.find((candidate) => candidate.id === node.id));
  if (owner) owner.enabled = node.enabled;
}
