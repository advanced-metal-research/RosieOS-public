/**
 * Which program node a dense-trajectory segment belongs to.
 *
 * The daemon reports where it is as a segment index; the segment's
 * `source_id` names the motion-result entry it was sampled from, not a
 * node. This is the one place that naming is decoded, so the spreadsheet
 * can light the node the robot is executing:
 *
 * - a weld segment's source is the seam id, the weld node's `geometry.id`;
 * - a dwell's source is `<seam>:dwell-in` or `<seam>:dwell-out`, the same
 *   weld node (the dwell is the torch settling on it);
 * - a freespace segment's source is `<from>-><to>` where each end is a seam
 *   id or `home`. The program's freespace node between those welds is the
 *   one preceding the destination weld in the node line, or, for a move
 *   home, the one following the departed weld. Events (io, dwell) are
 *   transparent to the search, as they are to the alternation rule.
 */

import type { ProgramV2Node } from "../contracts/robot-v4-program-v2.ts";

const FREESPACE_OPS = new Set(["approach", "transit", "retract", "home"]);

function weldIndexBySeam(nodes: ProgramV2Node[], seamId: string): number {
  return nodes.findIndex((node) =>
    node.op === "weld" && node.geometry.kind === "seam" && node.geometry.id === seamId);
}

function isMotion(node: ProgramV2Node): boolean {
  return node.op === "weld" || FREESPACE_OPS.has(node.op);
}

/** The nearest freespace node before (`-1`) or after (`+1`) `index`, skipping
 *  events; null when the neighbour is another weld or the line ends. */
function adjacentFreespace(nodes: ProgramV2Node[], index: number, step: -1 | 1): string | null {
  for (let cursor = index + step; cursor >= 0 && cursor < nodes.length; cursor += step) {
    const node = nodes[cursor];
    if (!node || !isMotion(node)) continue;
    return FREESPACE_OPS.has(node.op) ? node.id : null;
  }
  return null;
}

export function denseSegmentNodeId(
  nodes: ProgramV2Node[],
  kind: "freespace" | "weld" | "dwell",
  sourceId: string,
): string | null {
  if (nodes.some(node => node.id === sourceId)) return sourceId;
  if (kind === "weld" || kind === "dwell") {
    const seamId = kind === "dwell" ? sourceId.replace(/:dwell-(in|out)$/, "") : sourceId;
    const index = weldIndexBySeam(nodes, seamId);
    return index < 0 ? null : nodes[index]?.id ?? null;
  }
  const arrow = sourceId.indexOf("->");
  if (arrow < 0) return null;
  const from = sourceId.slice(0, arrow);
  const to = sourceId.slice(arrow + 2);
  const destination = weldIndexBySeam(nodes, to);
  if (destination >= 0) return adjacentFreespace(nodes, destination, -1);
  const origin = weldIndexBySeam(nodes, from);
  if (origin >= 0) return adjacentFreespace(nodes, origin, 1);
  return null;
}
