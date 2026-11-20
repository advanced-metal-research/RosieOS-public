/**
 * The node list as an editable thing: constructors and the three edits a
 * spreadsheet makes (insert a weld, remove a node, move a node), each
 * keeping the single line valid.
 *
 * The line is `home? approach weld (transit weld)* retract home?`, motion
 * nodes only; events (`io`, `dwell`) sit on the boundaries and are
 * transparent. The planner never inserts a freespace move, so every edit
 * here has to leave the line whole: inserting a weld turns the trailing
 * retract into a transit, removing a weld merges the freespace nodes around
 * it, and a weld moves by swapping with its neighbouring weld while the
 * freespace nodes stay where they are. A freespace node cannot be removed
 * on its own -- delete the weld instead -- and cannot be moved, because its
 * place IS its meaning.
 *
 * DOM-free and node-importable; `nodes.test.ts` asserts every rule.
 */

import {
  type DwellNode,
  type FreespaceNode,
  type IoNode,
  type ProgramV2Node,
  type ProgramV2Op,
  type Standoff,
  type WeldNode,
  programV2LineProblems,
} from "../contracts/robot-v4-program-v2.ts";
import { projectId } from "../domain/project.ts";

export const MOTION_OPS: ReadonlySet<ProgramV2Op> = new Set(["weld", "approach", "transit", "retract", "home", "move"]);
export const FREESPACE_OPS: ReadonlySet<ProgramV2Op> = new Set(["approach", "transit", "retract", "home"]);
export const EVENT_OPS: ReadonlySet<ProgramV2Op> = new Set(["io", "dwell"]);

export const isMotion = (node: ProgramV2Node): boolean => MOTION_OPS.has(node.op);
export const isFreespace = (node: ProgramV2Node): node is FreespaceNode => FREESPACE_OPS.has(node.op);
export const isWeld = (node: ProgramV2Node): node is WeldNode => node.op === "weld";
export const isEvent = (node: ProgramV2Node): boolean => EVENT_OPS.has(node.op);

export interface StandoffPair {
  arrive?: Standoff;
  depart?: Standoff;
}

function common(op: ProgramV2Op, label: string, comment: string, id = projectId()) {
  return {
    id,
    op,
    enabled: true,
    label,
    comment,
    row_hash: `pending:${id}`,
  };
}

export function freespaceNode(
  op: FreespaceNode["op"],
  input: { id?: string; label?: string; comment?: string } & StandoffPair = {},
): FreespaceNode {
  const node: FreespaceNode = {
    ...common(op, input.label ?? op.toUpperCase(), input.comment ?? "", input.id),
    op,
  };
  if (input.arrive && (op === "approach" || op === "transit")) node.arrive_standoff = structuredClone(input.arrive);
  if (input.depart && (op === "retract" || op === "transit")) node.depart_standoff = structuredClone(input.depart);
  return node;
}

export function dwellNode(durationS: number, input: { id?: string; label?: string; comment?: string } = {}): DwellNode {
  return { ...common("dwell", input.label ?? "DWELL", input.comment ?? "", input.id), op: "dwell", duration_s: durationS };
}

export function ioNode(
  channel: string,
  state: boolean,
  input: { id?: string; label?: string; comment?: string } = {},
): IoNode {
  return { ...common("io", input.label ?? "IO", input.comment ?? "", input.id), op: "io", channel, state };
}

export function nodeById(nodes: readonly ProgramV2Node[], id: string): ProgramV2Node | null {
  return nodes.find((node) => node.id === id) ?? null;
}

/** Every way the motion nodes fail to form the single line. */
export function lineProblems(nodes: readonly ProgramV2Node[]): string[] {
  const motion = nodes.filter(isMotion).map((node) => ({ id: node.id, op: node.op }));
  if (motion.length === 0) return [];
  return programV2LineProblems(motion);
}

/** The index a new weld's approach/transit goes at: before a trailing home
 *  and anything after it, else at the end. */
function insertionIndex(nodes: readonly ProgramV2Node[]): number {
  let lastMotion = -1;
  for (let index = nodes.length - 1; index >= 0; index -= 1) {
    if (isMotion(nodes[index]!)) { lastMotion = index; break; }
  }
  if (lastMotion >= 0 && nodes[lastMotion]!.op === "home") {
    // The home may be preceded by events that belong to it (gas off); keep
    // them with it by inserting before the first event that directly
    // precedes the home.
    let at = lastMotion;
    while (at > 0 && isEvent(nodes[at - 1]!)) at -= 1;
    return at;
  }
  return nodes.length;
}

/**
 * Append a weld and the freespace it needs. An empty line gets
 * `approach weld retract`; a line ending in a retract has that retract
 * turned into a transit (its depart standoff kept, the new weld's arrive
 * standoff added) and `weld retract` appended after it.
 */
export function insertWeld(
  nodes: readonly ProgramV2Node[],
  weld: WeldNode,
  standoff: StandoffPair = {},
): ProgramV2Node[] {
  const out = nodes.map((node) => structuredClone(node));
  const at = insertionIndex(out);
  const before = out.slice(0, at);
  const after = out.slice(at);
  const motionBefore = before.filter(isMotion);
  const last = motionBefore.at(-1);
  const additions: ProgramV2Node[] = [];
  if (!last || last.op === "home" || last.op === "move") {
    additions.push(freespaceNode("approach", { id: `${weld.id}-approach`, arrive: standoff.arrive }));
  } else if (last.op === "retract") {
    const transit = last as FreespaceNode;
    transit.op = "transit";
    transit.label = transit.label === "RETRACT" ? "TRANSIT" : transit.label;
    if (standoff.arrive) transit.arrive_standoff = structuredClone(standoff.arrive);
  } else {
    throw new Error(`cannot add a weld after ${last.op} ${JSON.stringify(last.id)}; the line must end in a retract`);
  }
  additions.push(structuredClone(weld));
  additions.push(freespaceNode("retract", { id: `${weld.id}-retract`, depart: standoff.depart }));
  return [...before, ...additions, ...after];
}

/**
 * Remove a node, keeping the line whole. A weld takes the freespace around
 * it with it: `approach W retract` all go; `approach W transit` becomes an
 * approach; `transit W retract` becomes a retract; `transit W transit`
 * becomes one transit. Events and a home go on their own. A freespace node
 * between welds is refused with the reason.
 */
export function removeNode(nodes: readonly ProgramV2Node[], id: string): ProgramV2Node[] {
  const out = nodes.map((node) => structuredClone(node));
  const index = out.findIndex((node) => node.id === id);
  if (index < 0) throw new Error(`node ${JSON.stringify(id)} was not found`);
  const node = out[index]!;
  if (isEvent(node) || node.op === "home" || node.op === "move") {
    out.splice(index, 1);
    return out;
  }
  if (isFreespace(node)) {
    throw new Error(
      `${node.op} ${JSON.stringify(id)} is the move to or from a weld; delete the weld instead`,
    );
  }
  // A weld. Find the freespace node before and after it (skipping events).
  const prevIndex = neighbourMotion(out, index, -1);
  const nextIndex = neighbourMotion(out, index, 1);
  const prev = prevIndex >= 0 ? out[prevIndex] as FreespaceNode : null;
  const next = nextIndex >= 0 ? out[nextIndex] as FreespaceNode : null;
  const remove = new Set<number>([index]);
  if (prev?.op === "approach" && next?.op === "retract") {
    remove.add(prevIndex); remove.add(nextIndex);
  } else if (prev?.op === "approach" && next?.op === "transit") {
    // The transit carries the next weld's arrival; it becomes the approach.
    remove.add(prevIndex);
    next.op = "approach";
    delete next.depart_standoff;
    if (next.label === "TRANSIT") next.label = "APPROACH";
  } else if (prev?.op === "transit" && next?.op === "retract") {
    remove.add(nextIndex);
    prev.op = "retract";
    delete prev.arrive_standoff;
    if (prev.label === "TRANSIT") prev.label = "RETRACT";
  } else if (prev?.op === "transit" && next?.op === "transit") {
    // One transit from the previous weld to the next: depart from the
    // first, arrive from the second.
    remove.add(nextIndex);
    if (next.arrive_standoff) prev.arrive_standoff = next.arrive_standoff;
    else delete prev.arrive_standoff;
  } else {
    // A line that was already broken around this weld: take the weld out
    // and let lineProblems say what is left.
  }
  return out.filter((_, at) => !remove.has(at));
}

function neighbourMotion(nodes: readonly ProgramV2Node[], from: number, step: -1 | 1): number {
  for (let index = from + step; index >= 0 && index < nodes.length; index += step) {
    if (isMotion(nodes[index]!)) return index;
  }
  return -1;
}

/** Why a node cannot move by `delta`, or null when it can. */
export function moveBlocked(nodes: readonly ProgramV2Node[], id: string, delta: -1 | 1): string | null {
  const index = nodes.findIndex((node) => node.id === id);
  if (index < 0) return "node not found";
  const node = nodes[index]!;
  if (isFreespace(node)) return `${node.op} moves with its weld; move the weld`;
  if (isWeld(node)) {
    const welds = nodes.filter(isWeld);
    const k = welds.findIndex((weld) => weld.id === id);
    if (k + delta < 0) return "already the first weld";
    if (k + delta >= welds.length) return "already the last weld";
    return null;
  }
  if (node.op === "move") return teachMoveIndex(nodes, index, delta) === null ? "No valid position in that direction" : null;
  // An event moves one slot at a time, anywhere.
  if (index + delta < 0) return "already first";
  if (index + delta >= nodes.length) return "already last";
  return null;
}

/**
 * Move a node. A weld swaps places with the neighbouring weld and the
 * freespace nodes stay put, so the line cannot break; an event slides one
 * slot; a freespace node is refused.
 */
export function moveNode(nodes: readonly ProgramV2Node[], id: string, delta: -1 | 1): ProgramV2Node[] {
  const blocked = moveBlocked(nodes, id, delta);
  if (blocked) throw new Error(blocked);
  const out = nodes.map((node) => structuredClone(node));
  const index = out.findIndex((node) => node.id === id);
  const node = out[index]!;
  if (isWeld(node)) {
    const weldIndices = out.flatMap((candidate, at) => (isWeld(candidate) ? [at] : []));
    const k = weldIndices.indexOf(index);
    const other = weldIndices[k + delta]!;
    [out[index], out[other]] = [out[other]!, out[index]!];
    return out;
  }
  if (node.op === "move") {
    const destination = teachMoveIndex(out, index, delta)!;
    out.splice(index, 1); out.splice(destination, 0, node); return out;
  }
  const other = index + delta;
  [out[index], out[other]] = [out[other]!, out[index]!];
  return out;
}

function teachMoveIndex(nodes: readonly ProgramV2Node[], index: number, delta: -1 | 1): number | null {
  for (let at = index + delta; at >= 0 && at < nodes.length; at += delta) {
    const candidate = [...nodes]; const [node] = candidate.splice(index, 1); candidate.splice(at, 0, node!);
    if (!lineProblems(candidate).length) return at;
  }
  return null;
}

/** The weld nodes, in line order. */
export const weldNodes = (nodes: readonly ProgramV2Node[]): WeldNode[] => nodes.filter(isWeld);

/**
 * The line as it compiles: a disabled weld is absent, and the freespace
 * around it merges exactly as if it had been deleted, so the program never
 * travels to a weld it then skips. Disabled events and homes are simply
 * left out. The authored list is untouched; this is what the lowering and
 * the status column read.
 */
export function effectiveNodes(nodes: readonly ProgramV2Node[]): ProgramV2Node[] {
  let out: ProgramV2Node[] = nodes.map((node) => structuredClone(node));
  for (const node of nodes) {
    if (node.enabled) continue;
    if (!out.some((candidate) => candidate.id === node.id)) continue;
    if (isWeld(node) || isEvent(node) || node.op === "home" || node.op === "move") out = removeNode(out, node.id);
  }
  return out;
}
