/** Node rows share the modal's field validation; selection and playback update in place. */

import { el, replace } from "../rails/dom.ts";
import type { FieldSpec, EditValue } from "./field-specs.ts";
import { HOME_WAYPOINT_SPEED_MM_S } from "./move-defaults.ts";
import type { NodeSummary } from "./summaries.ts";

export interface NodeSpreadsheetView {
  summaries: NodeSummary[];
  selectedNodeId: string | null;
  playbackNodeId: string | null;
  /** Editing is locked (an execution is active); actions are disabled. */
  lockedReason: string | null;
  /** A notice above the rows, e.g. for a legacy rails program. */
  banner: string | null;
  moveBlocked(nodeId: string, delta: -1 | 1): string | null;
  deleteBlocked(nodeId: string): string | null;
}

export interface NodeSpreadsheetHandlers {
  onEdit(nodeId: string, spec: FieldSpec, value: EditValue): void;
  onSelect(nodeId: string): void;
  onOpen(nodeId: string): void;
  onToggleEnabled(nodeId: string, enabled: boolean): void;
  onMove(nodeId: string, delta: -1 | 1): void;
  onDelete(nodeId: string): void;
}

export const EMPTY_PROGRAM_TEXT =
  "No nodes yet. Record a waypoint from Teach or add a weld; connecting moves are added around welds.";

/** A Home without welds flies at the planner's default; it has no row field to edit. */
function homeSpeed(): HTMLElement {
  return el("span", { class: "node-speed-default", title: "Default: with no weld in the program, Home is a MoveJ to the reset pose at this TCP speed limit" },
    `${HOME_WAYPOINT_SPEED_MM_S} (default)`);
}

export function renderNodeSpreadsheet(
  host: HTMLElement,
  view: NodeSpreadsheetView,
  handlers: NodeSpreadsheetHandlers,
): void {
  if (view.summaries.length === 0) {
    replace(host, view.banner ? el("div", { class: "spreadsheet-banner" }, view.banner) : null, el("div", { class: "spreadsheet-empty" }, EMPTY_PROGRAM_TEXT));
    return;
  }
  const hasMoves = view.summaries.some(summary => summary.op === "move");
  const hasWelds = view.summaries.some(summary => summary.op === "weld");
  const hasProcess = view.summaries.some(summary => summary.op !== "move" && summary.process !== "");
  const processHeading = hasMoves ? (hasProcess ? "Process / joint limit" : "Joint limit %") : "Process";
  const locked = view.lockedReason;
  const rows = view.summaries.map((summary) => {
    const up = locked ?? view.moveBlocked(summary.id, -1);
    const down = locked ?? view.moveBlocked(summary.id, 1);
    const remove = locked ?? view.deleteBlocked(summary.id);
    const node = summary.node;
    const edit = (spec: FieldSpec, value: string | number, text: string) => inlineField(summary, spec, value, text, locked, handlers);
    const number = (path: string, label: string, value: number, unit: string, scale = 1, min?: number, max?: number) =>
      edit({ path, label, kind: "number", unit, displayScale: scale, min, max }, value * scale, `${Number((value * scale).toFixed(2))}${unit}`);
    const motion = (label = "Move type") => node.op === "move" ? edit({ path: "motion", label, kind: "enum", options: ["joint", "linear", "circular"] }, node.motion, summary.freespace) : summary.freespace;
    const geometry = node.op === "move" ? [
      ...node.target.xyz_m.map((value, i) => el("span", { class: "node-coordinate" }, `${["X", "Y", "Z"][i]} `, number(`target.xyz_m.${i}`, ["X", "Y", "Z"][i]!, value, "", 1000))),
      el("span", {}, " mm · "),
      ...node.target.rpy_rad.map((value, i) => el("span", { class: "node-coordinate" }, `${["Rx", "Ry", "Rz"][i]} `, number(`target.rpy_rad.${i}`, ["Roll", "Pitch", "Yaw"][i]!, value, "°", 180 / Math.PI))),
    ] : summary.geometry;
    const process = node.op === "move" ? number("speed_scale", "Joint speed", node.speed_scale, "%", 100, 1, 100)
      : node.op === "dwell" ? number("duration_s", "Dwell duration", node.duration_s, " s", 1, 0)
      : node.op === "weld" && node.travel_speed_mm_s.kind === "constant" ? edit({ path: "travel_speed_mm_s", label: "Weld speed", kind: "number", scalar: true, min: 0.01 }, node.travel_speed_mm_s.value, summary.process)
      : summary.process;
    const button = (text: string, title: string | null, action: () => void, ok: string): HTMLElement =>
      el("button", {
        type: "button",
        class: "mini-button",
        disabled: title !== null,
        title: title ?? ok,
        on: { click: (event: Event) => { event.stopPropagation(); action(); } },
      }, text);
    return el(
      "tr",
      {
        class: [
          `node-row is-${summary.family}`,
          summary.id === view.selectedNodeId ? "is-selected" : "",
          summary.id === view.playbackNodeId ? "is-playback-current" : "",
          summary.enabled ? "" : "is-disabled",
        ].join(" ").trim(),
        tabindex: 0,
        data: { nodeId: summary.id },
        on: {
          click: () => handlers.onSelect(summary.id),
          dblclick: (event: Event) => {
            if ((event.target as HTMLElement).closest("input, button, select, .node-inline")) return;
            handlers.onOpen(summary.id);
          },
          keydown: (event: Event) => {
            if ((event.target as HTMLElement).closest("input, button, select, .node-inline")) return;
            const key = (event as KeyboardEvent).key;
            if (key === "Enter") { event.preventDefault(); handlers.onOpen(summary.id); }
          },
        },
      },
      // Approach, transit and retract are procedural: they exist because of
      // the welds around them and go where the line puts them, so they carry
      // no controls. A home may be deleted but not moved.
      el("td", { class: "node-order" },
        summary.family === "freespace" && summary.op !== "move"
          ? (summary.op === "home" ? button("✕", remove, () => handlers.onDelete(summary.id), "delete") : null)
          : [
              button("↑", up, () => handlers.onMove(summary.id, -1), "move up"),
              button("↓", down, () => handlers.onMove(summary.id, 1), "move down"),
              button("✕", remove, () => handlers.onDelete(summary.id), "delete"),
            ]),
      el("td", { class: "node-number" }, String(summary.number).padStart(2, "0")),
      // Authored moves and welds can be skipped; procedural connectors stay with their welds.
      el("td", { class: "node-on" }, (summary.family === "weld" || summary.op === "move") ? el("input", {
        type: "checkbox",
        checked: summary.enabled,
        disabled: locked !== null,
        title: locked ?? (summary.op === "move" ? "Include this taught move" : summary.enabled ? "skip this weld and the moves it needs" : "include this weld"),
        "aria-label": `${summary.label} enabled`,
        on: {
          click: (event: Event) => event.stopPropagation(),
          change: (event: Event) => handlers.onToggleEnabled(summary.id, (event.target as HTMLInputElement).checked),
        },
      }) : null),
      el("td", { class: "node-op" }, el("span", { class: "op-badge", data: { op: summary.op } }, node.op === "move" ? motion("Operation") : summary.op)),
      el("td", { class: "node-label" }, summary.label),
      el("td", { class: "node-geometry", title: node.op === "move" ? "World TCP · XYZ in mm, roll/pitch/yaw in degrees" : summary.geometry }, geometry),
      el("td", { class: "node-process", title: node.op === "move" ? "Joint velocity limit; acceleration is set in Motion profile" : summary.process }, process),
      el("td", { class: "node-speed", title: "Requested TCP speed limit; joint and acceleration limits still apply" },
        node.op === "move" ? number("speed_mm_s", "TCP speed", node.speed_mm_s, "", 1, 0.01)
          : node.op === "home" && !hasWelds ? homeSpeed() : ""),
      el("td", { class: "node-freespace" }, motion()),
      el("td", { class: `node-status is-${summary.statusClass}` }, summary.status),
      el("td", { class: "node-comment", title: summary.comment }, summary.comment),
    );
  });
  replace(
    host,
    view.banner ? el("div", { class: "spreadsheet-banner" }, view.banner) : null,
    el(
      "table",
      { class: "node-grid" },
      el("thead", {}, el("tr", {},
        el("th", {}, "Order"), el("th", {}, "Number"), el("th", {}, "Enabled"), el("th", {}, "Operation"),
        el("th", {}, "Label"), el("th", {}, "Geometry"), el("th", { title: hasMoves ? "MOVE rows: percentage of robot joint velocity limits" : undefined }, processHeading), el("th", {}, "Speed · mm/s"), el("th", {}, "Freespace"),
        el("th", {}, "Status"), el("th", {}, "Comment"))),
      el("tbody", {}, rows),
    ),
  );
}

/** Toggle a highlight class on the node rows in place. */
export function applyNodeHighlight(
  host: HTMLElement,
  nodeId: string | null,
  className: "is-selected" | "is-playback-current",
  scrollIntoView: boolean,
): void {
  let match: HTMLElement | null = null;
  for (const row of host.querySelectorAll<HTMLTableRowElement>("tr[data-node-id]")) {
    const hit = nodeId !== null && row.dataset.nodeId === nodeId;
    row.classList.toggle(className, hit);
    if (hit) match = row;
  }
  if (scrollIntoView && match) match.scrollIntoView({ block: "nearest", inline: "nearest" });
}

/** First click selects; a second click edits just that value. */
function inlineField(summary: NodeSummary, spec: FieldSpec, value: string | number, text: string,
  locked: string | null, handlers: NodeSpreadsheetHandlers): HTMLElement {
  const host = el("span", { class: "node-inline" });
  const label = `${summary.label} ${spec.label}`;
  const display = el("button", { type: "button", class: "node-value", disabled: locked !== null,
    title: locked ?? `Select row, then click to edit ${spec.label}`, "aria-label": label,
    on: { click: (event: Event) => {
      event.stopPropagation();
      if (!host.closest("tr")?.classList.contains("is-selected")) { handlers.onSelect(summary.id); return; }
      open();
    } },
  }, text);
  host.append(display);
  function open(): void {
    if (spec.kind === "enum") { openChoices(); return; }
    let finished = false;
    const error = el("span", { class: "node-edit-error", role: "alert" });
    const initial = typeof value === "number" ? String(Number(value.toFixed(2))) : value;
    const input = el("input", { class: "node-edit", type: "text", inputmode: "decimal", autocomplete: "off", "aria-label": label });
    input.value = initial;
    const cancel = () => { finished = true; display.style.visibility = ""; replace(host, display); };
    const commit = () => {
      if (finished) return;
      if (input.value === initial) { cancel(); return; }
      try {
        const next = input.value.trim() === "" ? NaN : Number(input.value);
        finished = true; // Saving rebuilds the row and can synchronously blur this input.
        handlers.onEdit(summary.id, spec, next);
      } catch (reason) {
        finished = false;
        error.textContent = reason instanceof Error ? reason.message : String(reason);
        input.setAttribute("aria-invalid", "true");
      }
    };
    input.addEventListener("click", event => event.stopPropagation());
    input.addEventListener("keydown", event => {
      event.stopPropagation();
      if ((event as KeyboardEvent).key === "Escape") { event.preventDefault(); cancel(); }
      if ((event as KeyboardEvent).key === "Enter") { event.preventDefault(); commit(); }
    });
    input.addEventListener("blur", commit);
    // Keep the original text as the layout measure so editing never shifts the row.
    const suffix = spec.kind === "number" && text.startsWith(initial) ? text.slice(initial.length) : "";
    const editor = el("span", { class: "node-editor-value" }, input, suffix);
    display.style.visibility = "hidden";
    replace(host, display, editor, error);
    input.focus();
    input.select();
  }
  function openChoices(): void {
    const options = spec.options!;
    const cell = host.closest("td")!;
    const bounds = cell.getBoundingClientRect();
    const list = el("div", { class: "node-choice-list", popover: "auto", role: "listbox", "aria-label": label });
    list.style.left = `${bounds.left}px`;
    list.style.top = `${bounds.bottom}px`;
    list.style.minWidth = `${bounds.width}px`;
    const choices = options.map(option => el("button", {
      type: "button", role: "option", "aria-selected": String(option === value), tabindex: -1,
      on: { click: (event: Event) => {
        event.stopPropagation();
        if (option === value) { list.hidePopover(); return; }
        try { handlers.onEdit(summary.id, spec, option); }
        catch (reason) { list.title = reason instanceof Error ? reason.message : String(reason); }
      } },
    }, ({ joint: "MoveJ", linear: "MoveL", circular: "MoveC" } as Record<string, string>)[option] ?? option.toUpperCase()));
    list.append(...choices);
    list.addEventListener("keydown", event => {
      event.stopPropagation();
      const at = choices.indexOf(document.activeElement as HTMLButtonElement);
      if (event.key === "ArrowDown" || event.key === "ArrowUp") {
        event.preventDefault();
        choices[(at + (event.key === "ArrowDown" ? 1 : options.length - 1)) % options.length]!.focus();
      } else if (event.key === "Escape" || event.key === "Tab") {
        if (event.key === "Escape") event.preventDefault();
        list.hidePopover();
      }
    });
    list.addEventListener("toggle", event => {
      if ((event as ToggleEvent).newState !== "closed") return;
      display.classList.remove("is-editing");
      display.setAttribute("aria-expanded", "false");
      list.remove();
    });
    display.classList.add("is-editing");
    display.setAttribute("aria-haspopup", "listbox");
    display.setAttribute("aria-expanded", "true");
    host.append(list);
    list.showPopover();
    choices[Math.max(0, options.indexOf(String(value)))]!.focus();
  }
  return host;
}
