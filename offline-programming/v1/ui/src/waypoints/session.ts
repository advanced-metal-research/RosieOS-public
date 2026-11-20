import type { ProgramV2Node } from "../contracts/robot-v4-program-v2.ts";
import { insertWeld, removeNode } from "../program-v2/nodes.ts";
import {
  DEFAULT_WAYPOINT_STANDOFF_MM,
  add,
  clampStandoffMm,
  newId,
  waypointAfter,
  type Continuity,
  type Quat,
  type V3,
  type Waypoint,
} from "./model.ts";
import {
  type ManualWeldProcess,
  type WaypointWeld,
  type WeldTool,
  isManualPoseWeld,
  manualPoseWeldNode,
  nextWeldLabel,
  resolveStartLinks,
  startCandidates,
  waypointWeldOf,
  withWaypoints,
} from "./node.ts";

/** How far a new line's second point, and a new arc's radius, sit from its
 *  first point: 20 mm, along the part's X. Far enough apart to grab either
 *  handle; the operator drags them where the weld goes. */
export const NEW_SHAPE_SIZE_M = 0.02;

/**
 * The Waypoint weld tool's view of the program: its manual_pose_wp weld
 * nodes as waypoint lists, and which weld and waypoint the tool is on.
 *
 * The program is the authority. Every edit goes through the host as one
 * program edit -- undoable, autosaved, in the catalogue -- and the welds are
 * read back from the nodes after it. The session only keeps the selection,
 * which is the tool's and not the program's.
 *
 * Every edit is followed by `resolveStartLinks`, so a weld that starts at
 * another's end moves with it in the same edit, whichever end was touched.
 * A line keeps two points and an arc three: points are moved, never added
 * or removed, on those.
 */
export interface WaypointHost {
  nodes(): readonly ProgramV2Node[];
  /** Replace the node list as one program edit. */
  editNodes(message: string, change: (nodes: readonly ProgramV2Node[]) => ProgramV2Node[]): void;
  /** What a new weld is welded with: the program's process defaults. */
  newWeldProcess(): ManualWeldProcess;
}

export interface WaypointSelection {
  weldId: string | null;
  waypointId: string | null;
}

export class WaypointSession {
  readonly #host: WaypointHost;
  #selection: WaypointSelection = { weldId: null, waypointId: null };
  #cache: { nodes: readonly ProgramV2Node[]; welds: WaypointWeld[] } | null = null;
  readonly #listeners = new Set<() => void>();

  constructor(host: WaypointHost) {
    this.#host = host;
  }

  /** Called after the selection changes. Program edits are the host's to
   *  announce. */
  onSelect(listener: () => void): () => void {
    this.#listeners.add(listener);
    return () => this.#listeners.delete(listener);
  }

  get welds(): readonly WaypointWeld[] {
    const nodes = this.#host.nodes();
    if (this.#cache?.nodes !== nodes) {
      this.#cache = { nodes, welds: nodes.filter(isManualPoseWeld).map(waypointWeldOf) };
    }
    return this.#cache.welds;
  }

  /** The selection, with anything the program no longer has dropped: an
   *  undo or another program can take the selected weld away. */
  get selection(): WaypointSelection {
    const weld = this.welds.find((candidate) => candidate.id === this.#selection.weldId) ?? null;
    const waypoint = weld?.waypoints.find((point) => point.id === this.#selection.waypointId) ?? null;
    return { weldId: weld?.id ?? null, waypointId: waypoint?.id ?? null };
  }

  get selectedWeld(): WaypointWeld | null {
    return this.welds.find((weld) => weld.id === this.selection.weldId) ?? null;
  }

  get selectedWaypoint(): Waypoint | null {
    const { waypointId } = this.selection;
    return this.selectedWeld?.waypoints.find((point) => point.id === waypointId) ?? null;
  }

  select(selection: Partial<WaypointSelection>): void {
    const current = this.selection;
    const next = { ...current, ...selection };
    if (selection.weldId !== undefined && selection.waypointId === undefined && selection.weldId !== current.weldId) {
      next.waypointId = null;
    }
    if (next.weldId === current.weldId && next.waypointId === current.waypointId) return;
    this.#selection = next;
    for (const listener of this.#listeners) listener();
  }

  /**
   * A new weld of `tool`, selected with the handle on its last point. It
   * starts at the selected weld's end, and stays tied there, when a weld is
   * selected -- welds are usually drawn one after another -- and at `origin`
   * otherwise. The first point takes the torch pose it starts from.
   */
  addWeld(tool: WeldTool, origin: V3, standoffMm: number = DEFAULT_WAYPOINT_STANDOFF_MM): void {
    const source = this.selectedWeld;
    const end = source?.waypoints.at(-1);
    // The first point is the template every later waypoint copies, so a
    // waypoint weld starts smooth; a line's or an arc's continuity is unused.
    const first: Waypoint = {
      ...(end ? waypointAfter([end], 0, origin, standoffMm) : waypointAfter([], -1, origin, standoffMm)),
      continuity: tool === "waypoint" ? "c1" : "c0",
    };
    const along = (x: number, y: number) => ({
      ...first,
      id: newId("wp"),
      positionM: add(first.positionM, [x, y, 0]),
    });
    const r = NEW_SHAPE_SIZE_M;
    const points: Waypoint[] = tool === "line"
      ? [first, along(r, 0)]
      : tool === "arc"
        // A quarter circle of radius r about first + (0, r, 0).
        ? [first, along(r * Math.SQRT1_2, r * (1 - Math.SQRT1_2)), along(r, r)]
        : [first];
    this.importWeld(points, { tool, startFrom: source?.id ?? null });
  }

  /** A new weld made of these waypoints, selected. */
  importWeld(
    waypoints: readonly Waypoint[],
    options: { tool?: WeldTool; startFrom?: string | null; message?: string } = {},
  ): void {
    const tool = options.tool ?? "waypoint";
    const nodes = this.#host.nodes();
    const label = nextWeldLabel(nodes, tool);
    const node = manualPoseWeldNode(label, waypoints, this.#host.newWeldProcess(), {
      tool,
      startFrom: options.startFrom ?? null,
    });
    this.#selection = { weldId: node.id, waypointId: waypoints.at(-1)?.id ?? null };
    this.#edit(options.message ?? `${label} added`, (current) => insertWeld(current, node));
  }

  /** The welds `weldId` may start from: none that start, through their own
   *  links, from it. */
  startCandidates(weldId: string): WaypointWeld[] {
    return startCandidates(this.#host.nodes(), weldId)
      .filter(isManualPoseWeld)
      .map(waypointWeldOf);
  }

  /** Tie a weld's first point to another weld's end, or untie it. */
  setStartFrom(weldId: string, sourceId: string | null): void {
    const weld = this.welds.find((candidate) => candidate.id === weldId);
    if (!weld || weld.startFrom === sourceId) return;
    if (sourceId && !this.startCandidates(weldId).some((candidate) => candidate.id === sourceId)) return;
    const source = this.welds.find((candidate) => candidate.id === sourceId);
    this.#edit(
      source ? `${weld.label} starts at the end of ${source.label}` : `${weld.label} starts on its own`,
      (current) => current.map((node) =>
        node.id === weldId && isManualPoseWeld(node)
          ? withWaypoints(node, weld.waypoints, { startFrom: sourceId })
          : node),
    );
  }

  /** Whether a waypoint's position is someone else's to set. */
  isTied(waypointId: string): boolean {
    const weld = this.#weldOf(waypointId);
    return Boolean(weld?.startFrom) && weld?.waypoints[0]?.id === waypointId;
  }

  /** Whether a weld's point count is fixed by its tool. */
  static fixedCount(tool: WeldTool): boolean {
    return tool !== "waypoint";
  }

  removeWeld(weldId: string): void {
    const welds = this.welds;
    const index = welds.findIndex((weld) => weld.id === weldId);
    if (index < 0) return;
    const weld = welds[index]!;
    if (this.#selection.weldId === weldId) {
      const next = welds[index + 1] ?? welds[index - 1];
      this.#selection = { weldId: next?.id ?? null, waypointId: null };
    }
    this.#edit(`${weld.label} deleted`, (current) => removeNode(current, weldId));
  }

  /** A copy of the selected waypoint (or the weld's last) right after it,
   *  selected so the handle lands on it. With no weld selected, a new weld
   *  starting at `origin`. */
  addWaypoint(origin: V3, standoffMm: number = DEFAULT_WAYPOINT_STANDOFF_MM): void {
    const weld = this.selectedWeld;
    if (!weld) {
      this.addWeld("waypoint", origin, standoffMm);
      return;
    }
    if (WaypointSession.fixedCount(weld.tool)) return;
    const points = [...weld.waypoints];
    const selectedIndex = points.findIndex((point) => point.id === this.selection.waypointId);
    const afterIndex = selectedIndex >= 0 ? selectedIndex : points.length - 1;
    const waypoint = waypointAfter(points, afterIndex, origin, standoffMm);
    points.splice(afterIndex + 1, 0, waypoint);
    this.#selection = { weldId: weld.id, waypointId: waypoint.id };
    this.#write(weld, points, `${weld.label}: P${afterIndex + 2} added`);
  }

  removeWaypoint(waypointId: string): void {
    const weld = this.#weldOf(waypointId);
    if (!weld || WaypointSession.fixedCount(weld.tool)) return;
    const points = [...weld.waypoints];
    const index = points.findIndex((point) => point.id === waypointId);
    points.splice(index, 1);
    if (this.#selection.waypointId === waypointId) {
      this.#selection = { weldId: weld.id, waypointId: points[Math.min(index, points.length - 1)]?.id ?? null };
    }
    this.#write(weld, points, `${weld.label}: P${index + 1} deleted`);
  }

  /** Move a waypoint one place earlier (-1) or later (+1) in its path. */
  moveWaypoint(waypointId: string, by: -1 | 1): void {
    const weld = this.#weldOf(waypointId);
    if (!weld) return;
    const points = [...weld.waypoints];
    const index = points.findIndex((point) => point.id === waypointId);
    const target = index + by;
    if (target < 0 || target >= points.length) return;
    const [point] = points.splice(index, 1);
    points.splice(target, 0, point!);
    this.#write(weld, points, `${weld.label}: P${index + 1} moved to P${target + 1}`);
  }

  /** A tied first point takes the orientation only: its position is the
   *  other weld's end, and `resolveStartLinks` would put it back anyway. */
  setPose(waypointId: string, positionM: V3, quaternion: Quat): void {
    const tied = this.isTied(waypointId);
    this.#update(waypointId, tied ? "turned" : "moved", (point) => ({
      ...point,
      positionM: tied ? point.positionM : [...positionM],
      quaternion: [...quaternion],
    }));
  }

  setStandoff(waypointId: string, standoffMm: number): void {
    if (!Number.isFinite(standoffMm)) return;
    this.#update(waypointId, "standoff set", (point) => ({ ...point, standoffMm: clampStandoffMm(standoffMm) }));
  }

  setContinuity(waypointId: string, continuity: Continuity): void {
    this.#update(waypointId, `set to ${continuity.toUpperCase()}`, (point) => ({ ...point, continuity }));
  }

  /** Every waypoint of a weld at once -- the ends included, so a waypoint
   *  that later becomes interior keeps the choice. */
  setWeldContinuity(weldId: string, continuity: Continuity): void {
    const weld = this.welds.find((candidate) => candidate.id === weldId);
    if (!weld) return;
    this.#write(
      weld,
      weld.waypoints.map((point) => ({ ...point, continuity })),
      `${weld.label}: every waypoint ${continuity.toUpperCase()}`,
    );
  }

  #update(waypointId: string, what: string, change: (point: Waypoint) => Waypoint): void {
    const weld = this.#weldOf(waypointId);
    if (!weld) return;
    const index = weld.waypoints.findIndex((point) => point.id === waypointId);
    const points = weld.waypoints.map((point) => (point.id === waypointId ? change(point) : point));
    this.#write(weld, points, `${weld.label}: P${index + 1} ${what}`);
  }

  #write(weld: WaypointWeld, points: readonly Waypoint[], message: string): void {
    this.#edit(message, (current) =>
      current.map((node) => (node.id === weld.id && isManualPoseWeld(node) ? withWaypoints(node, points) : node))
    );
  }

  /** Every program edit, with the start links followed after it. */
  #edit(message: string, change: (nodes: readonly ProgramV2Node[]) => ProgramV2Node[]): void {
    this.#host.editNodes(message, (current) => resolveStartLinks(change(current)));
  }

  #weldOf(waypointId: string): WaypointWeld | undefined {
    return this.welds.find((weld) => weld.waypoints.some((point) => point.id === waypointId));
  }
}

/**
 * Waypoints the tool kept in this browser's storage before they were
 * program nodes, taken out of storage as they are read -- the caller makes a
 * weld of them. Malformed entries are dropped rather than fatal.
 */
export function takeStoredWaypoints(programId: string): Waypoint[] {
  const key = `olp.waypoint-weld.${programId}`;
  let text: string | null = null;
  try {
    text = window.localStorage.getItem(key);
    window.localStorage.removeItem(key);
  } catch {
    return [];
  }
  if (!text) return [];
  let raw: unknown;
  try {
    raw = JSON.parse(text);
  } catch {
    return [];
  }
  if (!Array.isArray(raw)) return [];
  const finite = (value: unknown, length: number): boolean =>
    Array.isArray(value) && value.length === length && value.every((v) => Number.isFinite(v));
  return raw
    .filter((point: Partial<Waypoint> | null) =>
      typeof point?.id === "string"
      && finite(point.positionM, 3)
      && finite(point.quaternion, 4)
      && (point.continuity === "c0" || point.continuity === "c1" || point.continuity === "c2")
    )
    .map((point: Waypoint) => ({
      ...point,
      standoffMm: Number.isFinite(point.standoffMm)
        ? clampStandoffMm(point.standoffMm)
        : DEFAULT_WAYPOINT_STANDOFF_MM,
    }));
}
