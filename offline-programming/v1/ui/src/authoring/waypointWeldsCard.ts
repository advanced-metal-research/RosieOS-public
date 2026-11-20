import { MANUAL_POSE_MIN_CHORD_M } from "../contracts/robot-v4-program-v2.ts";
import {
  CONTINUITIES,
  arcThrough,
  STANDOFF_MAX_MM,
  STANDOFF_MIN_MM,
  sharedContinuity,
  type Continuity,
  type V3,
  type Waypoint,
} from "../waypoints/model.ts";
import { coincidentWaypoints, type WaypointWeld, type WeldTool } from "../waypoints/node.ts";
import { WaypointSession } from "../waypoints/session.ts";
import { NumberField } from "../ui/fields.ts";
import { element } from "../ui/widget.ts";
import { PartFocusCard } from "./partFocusCard.ts";

/**
 * The dialog of the three hand-placed weld tools -- Waypoint welds, Line
 * welds and Arc welds -- each weld one weld node of the program, in the
 * Rails welds card's frame and row style. One card serves all three: the
 * tool it is open for (`setTool`) decides its title, which welds it lists
 * and what "New" makes -- a waypoint weld (as many poses as are placed), a
 * line (two) or an arc (three, a circle from the first through the second to
 * the third). A new weld starts at the selected weld's end, whichever tool
 * made that one, and stays tied there -- "Starts at" unties it or ties it
 * elsewhere -- so a chain of lines and arcs moves together.
 *
 * Waypoints is the selected weld's poses in path order, each with its
 * standoff. Selecting one puts the Move part handle on its work point; a
 * tied first point only turns. On a waypoint weld "Add waypoint" drops an
 * exact copy of the selected pose right after it and selects that, and each
 * pose between the ends says how the path passes through it -- C0, C1 or
 * C2 -- with the list header setting all of them. A line and an arc keep
 * their counts.
 *
 * Two waypoints left on top of each other are refused, here first: the row
 * says so, and ✓ will not close the card until one is moved. (A copy starts
 * on top of its original; that is the moment to drag it.)
 *
 * The card reads and edits the session; it owns no state of its own. ✓
 * keeps and closes, ✕ puts the program back to how it was when the card
 * opened, as every tool card does.
 */
export interface WaypointWeldsCardCallbacks {
  onAccept: () => void;
  onCancel: () => void;
  onCentreCamera: () => void;
  onTransparency: (opacity: number) => void;
  /** Where a new weld's first waypoint goes, in the part's frame. */
  firstWaypointAt: () => V3;
  /** A new weld's first standoff: the program's process default. */
  defaultStandoffMm: () => number;
}

const CONTINUITY_HINTS: Record<Continuity, string> = {
  c0: "C0 · a corner: straight to each neighbour",
  c1: "C1 · a smooth turn: the tangent carries through",
  c2: "C2 · a spline knot: tangent and curvature carry through",
};

export class WaypointWeldsCard extends PartFocusCard {
  readonly #session: WaypointSession;
  readonly #callbacks: WaypointWeldsCardCallbacks;
  readonly #welds: HTMLDivElement;
  readonly #waypoints: HTMLDivElement;
  /** The selected waypoint's standoff field; rebuilt with the list. */
  #standoff: NumberField | null = null;
  #tool: WeldTool = "waypoint";

  constructor(session: WaypointSession, callbacks: WaypointWeldsCardCallbacks) {
    super({
      title: "Waypoint welds",
      className: "waypoint-welds-card",
      onAccept: callbacks.onAccept,
      onCancel: callbacks.onCancel,
      acceptTitle: "Keep these welds and close",
      cancelTitle: "Put the program back to how it was when the card opened",
      onCentreCamera: callbacks.onCentreCamera,
      onTransparency: callbacks.onTransparency,
    });
    this.#session = session;
    this.#callbacks = callbacks;
    this.#welds = this.addSection({ title: "Welds", key: "welds" });
    this.#waypoints = this.addSection({ title: "Waypoints", key: "waypoints" });
  }

  get tool(): WeldTool {
    return this.#tool;
  }

  /** Open the card for another of the three tools. */
  setTool(tool: WeldTool): void {
    this.#tool = tool;
    this.setTitle(TOOL_TITLES[tool]);
  }

  /** A new weld of the card's tool, selected: what picking the tool does. */
  newWeld(): void {
    this.#session.addWeld(this.#tool, this.#callbacks.firstWaypointAt(), this.#callbacks.defaultStandoffMm());
  }

  /** The selected weld, when it is one this tool made. */
  #ownWeld(): WaypointWeld | null {
    const weld = this.#session.selectedWeld;
    return weld && weld.tool === this.#tool ? weld : null;
  }

  render(): void {
    const problems = this.#session.welds.flatMap((weld) => problemsOf(weld).map((text) => `${weld.label}: ${text}`));
    this.setAcceptBlocked(problems.length ? `Fix before keeping: ${problems.join("; ")}` : null);
    this.#renderWelds();
    this.#renderWaypoints();
  }

  #newWaypoint(): void {
    this.#session.addWaypoint(this.#callbacks.firstWaypointAt(), this.#callbacks.defaultStandoffMm());
  }

  #renderWelds(): void {
    const session = this.#session;
    const { weldId } = session.selection;
    const labels = new Map(session.welds.map((weld) => [weld.id, weld.label]));
    const own = session.welds.filter((weld) => weld.tool === this.#tool);
    const rows = own.map((weld) => {
      const selected = weld.id === weldId;
      const problems = problemsOf(weld);
      const tied = weld.startFrom ? ` · from ${labels.get(weld.startFrom) ?? "?"}` : "";
      const row = entry(
        weld.label,
        `${weldNote(weld)}${tied}${problems.length ? ` · ${problems[0]}` : ""}`,
        selected,
        () => session.select({ weldId: weld.id }),
      );
      if (problems.length) row.classList.add("is-failed");
      if (selected) {
        row.append(startControl(session, weld));
        row.append(actions(button("Delete", () => session.removeWeld(weld.id), "Delete this weld and its points")));
      }
      return row;
    });
    const origin = () => this.#callbacks.firstWaypointAt();
    const standoff = () => this.#callbacks.defaultStandoffMm();
    const selected = session.selectedWeld;
    const tool = this.#tool;
    const panel = element("div", "weld-drawer-panel rails-panel");
    panel.append(...(rows.length ? rows : [hint(EMPTY_HINTS[tool])]));
    if (selected && selected.tool !== tool) {
      panel.append(hint(`Selected: ${selected.label}. A new ${TOOL_NOUNS[tool]} starts at its end.`));
    }
    panel.append(actions(button(
      `New ${TOOL_NOUNS[tool]}`,
      () => session.addWeld(tool, origin(), standoff()),
      `${NEW_TITLES[tool]}, starting ${selected ? `at the end of ${selected.label}` : "on the part"}`,
    )));
    this.#welds.replaceChildren(panel);
    const count = own.length;
    this.setSectionSummary(this.#welds, `${count} ${count === 1 ? TOOL_NOUNS[tool] : `${TOOL_NOUNS[tool]}s`}`);
  }

  #renderWaypoints(): void {
    const session = this.#session;
    const weld = this.#ownWeld();
    const panel = element("div", "weld-drawer-panel rails-panel");
    this.#standoff?.dispose();
    this.#standoff = null;
    if (!weld) {
      // Adding a waypoint with no weld selected starts one: the operator
      // came to place a torch, not to name a weld first.
      if (this.#tool === "waypoint") {
        panel.append(
          hint("Select a weld, or add a waypoint to start one."),
          actions(button(
            "Add waypoint",
            () => this.#session.addWeld("waypoint", this.#callbacks.firstWaypointAt(), this.#callbacks.defaultStandoffMm()),
            "Start a weld with a torch on the part",
          )),
        );
      } else {
        panel.append(hint(`Select a ${TOOL_NOUNS[this.#tool]}, or make a new one, to place its points.`));
      }
      this.#waypoints.replaceChildren(panel);
      this.setSectionSummary(this.#waypoints, "");
      return;
    }
    const points = weld.waypoints;
    const fixed = WaypointSession.fixedCount(weld.tool);
    if (!fixed && points.length >= 3) {
      const head = element("div", "rails-panel-head");
      head.append(element("span", "rails-entry-note", "Connect all"));
      head.append(continuityControl(sharedContinuity(points), (continuity) => session.setWeldContinuity(weld.id, continuity)));
      panel.append(head);
    }
    const overlaps = new Set(overlapsOf(weld));
    const source = weld.startFrom ? session.welds.find((candidate) => candidate.id === weld.startFrom) : null;
    const selectedId = session.selection.waypointId;
    points.forEach((point, index) => {
      const selected = point.id === selectedId;
      const end = index === 0 ? "start" : index === points.length - 1 ? "end" : null;
      const overlapping = overlaps.has(index);
      const tied = index === 0 && source ? ` · at the end of ${source.label}` : "";
      const note = overlapping
        ? `on top of P${index} — drag it away`
        : `${formatMm(point.positionM)} mm · ${point.standoffMm.toFixed(1)} mm off${end ? ` · ${end}` : ""}${tied}`;
      const row = entry(
        `P${index + 1}`,
        note,
        selected,
        () => session.select({ waypointId: point.id }),
        end || fixed ? null : continuityControl(point.continuity, (continuity) => session.setContinuity(point.id, continuity)),
      );
      if (overlapping) row.classList.add("is-failed");
      if (selected) {
        const standoff = new NumberField({
          label: "Standoff",
          unit: "mm",
          step: 0.5,
          min: STANDOFF_MIN_MM,
          max: STANDOFF_MAX_MM,
          onChange: (value) => session.setStandoff(point.id, value),
        });
        standoff.setValue(point.standoffMm.toFixed(1));
        // Typing in the field is not a click on the row.
        standoff.el.addEventListener("click", (event) => event.stopPropagation());
        standoff.el.classList.add("waypoint-standoff");
        standoff.el.title = "Contact tip to work point, along the torch axis";
        row.append(standoff.el);
        this.#standoff = standoff;
        const moves = [
          button("↑", () => session.moveWaypoint(point.id, -1), "Earlier in the path", index === 0),
          button("↓", () => session.moveWaypoint(point.id, 1), "Later in the path", index === points.length - 1),
        ];
        if (!fixed) moves.push(button("Delete", () => session.removeWaypoint(point.id), "Delete this waypoint"));
        row.append(actions(...moves));
      }
      panel.append(row);
    });
    if (source) {
      panel.append(hint(`P1 sits at the end of ${source.label} and moves with it; its handle only turns the torch.`));
    }
    if (fixed) {
      panel.append(hint(weld.tool === "arc"
        ? "An arc runs from P1 through P2 to P3. Drag the three off a straight line."
        : "A line runs from P1 to P2."));
    } else {
      if (points.length < 2) {
        panel.append(hint(
          points.length === 0
            ? "Add a waypoint, then drag its torch with the handle."
            : "A weld needs a second waypoint: add one and drag it off the first.",
        ));
      }
      panel.append(actions(button(
        "Add waypoint",
        () => this.#newWaypoint(),
        selectedId ? "Add a copy of the selected waypoint right after it" : "Add a copy of the last waypoint at the end of the path",
      )));
    }
    this.#waypoints.replaceChildren(panel);
    this.setSectionSummary(this.#waypoints, `${weld.label} · ${weldNote(weld)}`);
  }

  override dispose(): void {
    this.#standoff?.dispose();
    super.dispose();
  }
}

const TOOL_TITLES: Record<WeldTool, string> = {
  waypoint: "Waypoint welds",
  line: "Line welds",
  arc: "Arc welds",
};
const TOOL_NOUNS: Record<WeldTool, string> = { waypoint: "waypoint weld", line: "line", arc: "arc" };
const NEW_TITLES: Record<WeldTool, string> = {
  waypoint: "A weld through as many torch poses as you place",
  line: "A straight weld between two torch poses",
  arc: "A circular weld through three torch poses",
};
const EMPTY_HINTS: Record<WeldTool, string> = {
  waypoint: "No waypoint welds yet. Make one, then place its torches.",
  line: "No lines yet. Make one, then drag its two torches where the weld goes.",
  arc: "No arcs yet. Make one, then drag its three torches onto the curve.",
};

/** Indexes (0-based) of waypoints that sit on the one before them. */
function overlapsOf(weld: WaypointWeld): number[] {
  return coincidentWaypoints(weld.waypoints, MANUAL_POSE_MIN_CHORD_M);
}

/** What the program would refuse about a weld, said before it does. */
function problemsOf(weld: WaypointWeld): string[] {
  const out = overlapsOf(weld).map((index) => `P${index} and P${index + 1} on top of each other`);
  if (weld.tool === "arc" && out.length === 0 && !arcThrough(weld.waypoints)) {
    out.push("the three points are on a line");
  }
  return out;
}

function weldNote(weld: WaypointWeld): string {
  const points = weld.waypoints;
  if (weld.tool === "arc") {
    const arc = arcThrough(points);
    return arc ? `r ${(arc.radius * 1000).toFixed(1)} mm · ${(arc.sweep * 180 / Math.PI).toFixed(0)}°` : "3 points";
  }
  if (weld.tool === "line") {
    const [a, b] = points;
    return a && b ? `${(Math.hypot(b.positionM[0] - a.positionM[0], b.positionM[1] - a.positionM[1], b.positionM[2] - a.positionM[2]) * 1000).toFixed(1)} mm` : "2 points";
  }
  const count = `${points.length} waypoint${points.length === 1 ? "" : "s"}`;
  if (points.length < 3) return count;
  return `${count} · ${sharedContinuity(points)?.toUpperCase() ?? "mixed"}`;
}

/** "Starts at: on its own | the end of <weld>", for the selected weld. */
function startControl(session: WaypointSession, weld: WaypointWeld): HTMLElement {
  const row = element("label", "waypoint-start");
  row.append(element("span", "rails-entry-note", "Starts at"));
  const select = element("select");
  const own = element("option", "", "its own P1");
  own.value = "";
  select.append(own);
  for (const candidate of session.startCandidates(weld.id)) {
    const option = element("option", "", `end of ${candidate.label}`);
    option.value = candidate.id;
    select.append(option);
  }
  select.value = weld.startFrom ?? "";
  select.title = "Tie P1 to another weld's last point; it follows that weld when it moves";
  select.addEventListener("click", (event) => event.stopPropagation());
  select.addEventListener("change", () => session.setStartFrom(weld.id, select.value || null));
  row.append(select);
  return row;
}

function formatMm(position: readonly number[]): string {
  return position.map((v) => (v * 1_000).toFixed(1)).join("  ");
}

/** A Rails welds row: label over note, an optional control at the right,
 *  and room under them for the selected row's actions. */
function entry(
  label: string,
  note: string,
  selected: boolean,
  onClick: () => void,
  trailing: HTMLElement | null = null,
): HTMLDivElement {
  const row = element("div", `rails-entry waypoint-entry${selected ? " is-selected" : ""}`);
  const line = element("div", "waypoint-entry-line");
  const text = element("div", "waypoint-entry-text");
  text.append(element("span", "rails-entry-label", label), element("span", "rails-entry-note", note));
  line.append(text);
  if (trailing) line.append(trailing);
  row.append(line);
  row.addEventListener("click", onClick);
  return row;
}

function actions(...buttons: HTMLButtonElement[]): HTMLDivElement {
  const row = element("div", "rails-entry-actions");
  row.append(...buttons);
  return row;
}

function button(label: string, onClick: () => void, title: string, disabled = false): HTMLButtonElement {
  const control = element("button", "", label);
  control.type = "button";
  control.title = title;
  control.disabled = disabled;
  control.addEventListener("click", (event) => {
    // The rows select on click; a row's own button must not also select.
    event.stopPropagation();
    onClick();
  });
  return control;
}

function hint(text: string): HTMLParagraphElement {
  return element("p", "rails-panel-hint", text);
}

/** C0 | C1 | C2, with the current one lit; `null` lights none (mixed). */
function continuityControl(current: Continuity | null, onChoose: (continuity: Continuity) => void): HTMLDivElement {
  const group = element("div", "waypoint-continuity");
  group.setAttribute("role", "radiogroup");
  group.setAttribute("aria-label", "Continuity");
  for (const continuity of CONTINUITIES) {
    const option = element("button", continuity === current ? "is-active" : "", continuity.toUpperCase());
    option.type = "button";
    option.title = CONTINUITY_HINTS[continuity];
    option.setAttribute("role", "radio");
    option.setAttribute("aria-checked", String(continuity === current));
    option.addEventListener("click", (event) => {
      event.stopPropagation();
      onChoose(continuity);
    });
    group.append(option);
  }
  return group;
}
