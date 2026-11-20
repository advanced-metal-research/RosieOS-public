/**
 * The parameter table: name, default, value, unit, revert -- one row per
 * field, grouped, with the changed rows marked. Modelled on the weld
 * planner's `panes/prepare/field-form.ts` and extended with the cell kinds
 * a program node needs that a preset does not: an enum, a boolean, a
 * profile drawn by the spline editor, and a table of posed control points.
 *
 * The default column is the point, not decoration. A value on its own
 * cannot be read; a default beside it says which numbers are yours. For a
 * freespace node's overrides the default IS the program default, so a
 * cleared box visibly inherits.
 *
 * Built with `el()`, never `innerHTML`: a form of thirty fields cannot
 * afford to lose focus, scroll and caret on every keystroke, so a number
 * commits on `change` and nothing here re-renders the table on its own.
 */

import type { PosedPoint, ScalarFunction } from "../contracts/robot-v4-program-v2.ts";
import { el } from "../rails/dom.ts";
import { type EditValue, type FieldGroup, type FieldSpec, constantOf, dig, varies } from "./field-specs.ts";

export interface FormModel {
  /** The draft the rows read. */
  readonly resolved: Record<string, unknown>;
  /** What revert restores: the node as opened, or the program defaults. */
  readonly beneath: Record<string, unknown>;
  /** Write one edit; throws with the reason when it cannot. */
  set(spec: FieldSpec, value: EditValue): void;
  /** A virtual `_seam.` field's current value. */
  readVirtual(path: string): unknown;
  /** The spline editor for a profile field. */
  profileEditor(spec: FieldSpec, value: ScalarFunction | undefined, onChange: (fn: ScalarFunction) => void): HTMLElement;
  /** Called after a successful edit, with what to say in the status line. */
  onEdited(spec: FieldSpec): void;
  /** Called when an edit is refused, with the reason. */
  onRefused(spec: FieldSpec, reason: string): void;
}

const isVirtual = (spec: FieldSpec): boolean => spec.path.startsWith("_seam.");

function read(spec: FieldSpec, document: Record<string, unknown>, model: FormModel): unknown {
  return isVirtual(spec) ? model.readVirtual(spec.path) : dig(document, spec.path);
}

/** What a cell says. Absent is drawn as an em dash, never as 0: a field
 *  nobody set and a field set to zero are different welds. */
export function show(spec: FieldSpec, value: unknown): string {
  if (value === undefined || value === null) return "—";
  if (spec.kind === "number") {
    if (spec.scalar && varies(value)) return `varies (${String((value as ScalarFunction).kind)})`;
    const number = spec.scalar ? constantOf(value) : (typeof value === "number" ? value : null);
    return number === null ? "—" : String(Number((number * (spec.displayScale ?? 1)).toFixed(4)));
  }
  if (spec.kind === "bool") return value ? "yes" : "no";
  if (spec.kind === "points") return Array.isArray(value) ? `${value.length} pts` : "—";
  if (spec.kind === "profile") return varies(value) ? `profile (${String((value as ScalarFunction).kind)})` : show({ ...spec, kind: "number", scalar: true }, value);
  if (spec.kind === "enum" && Array.isArray(value)) return `[${value.map((v) => Number(v).toFixed(2)).join(", ")}]`;
  return String(value);
}

function commit(model: FormModel, spec: FieldSpec, value: EditValue): void {
  try {
    model.set(spec, value);
    model.onEdited(spec);
  } catch (error) {
    model.onRefused(spec, error instanceof Error ? error.message : String(error));
  }
}

function valueCell(spec: FieldSpec, model: FormModel): HTMLElement {
  const value = read(spec, model.resolved, model);
  const disabled = Boolean(spec.readOnly);
  const title = spec.readOnly ?? spec.hint ?? "";
  switch (spec.kind) {
    case "number": {
      if (spec.scalar && varies(value)) {
        return el("td", { class: "pvalue" }, el("span", { class: "muted", title: "a profile is shown, not flattened" }, show(spec, value)));
      }
      const number = spec.scalar ? constantOf(value) : (typeof value === "number" ? value : null);
      return el("td", { class: "pvalue" }, el("input", {
        type: "number",
        "aria-label": spec.label,
        step: spec.step ?? 1,
        min: spec.min,
        max: spec.max,
        value: number === null ? "" : String(number * (spec.displayScale ?? 1)),
        disabled,
        title,
        on: {
          change: (event: Event) => {
            const raw = (event.target as HTMLInputElement).value.trim();
            commit(model, spec, raw === "" ? null : Number(raw));
          },
        },
      }));
    }
    case "text":
      return el("td", { class: "pvalue" }, el("input", {
        type: "text",
        value: value === undefined || value === null ? "" : String(value),
        disabled,
        title,
        on: { change: (event: Event) => commit(model, spec, (event.target as HTMLInputElement).value) },
      }));
    case "bool":
      return el("td", { class: "pvalue" }, el("input", {
        type: "checkbox",
        "aria-label": spec.label,
        checked: Boolean(value),
        disabled,
        title,
        on: { change: (event: Event) => commit(model, spec, (event.target as HTMLInputElement).checked) },
      }));
    case "enum": {
      if (Array.isArray(value)) {
        return el("td", { class: "pvalue" }, el("span", { class: "muted", title: "an explicit vector; edit it in the document" }, show(spec, value)));
      }
      const select = el("select", {
        "aria-label": spec.label,
        disabled,
        title,
        on: {
          change: (event: Event) => {
            const raw = (event.target as HTMLSelectElement).value;
            commit(model, spec, raw === "" ? null : raw);
          },
        },
      }, spec.nullable ? el("option", { value: "" }, "— inherit") : null,
      (spec.options ?? []).map((option) => el("option", { value: option }, option)));
      // Set after construction: `selected` on an option is a property the
      // builder assigns before the option is in the tree.
      select.value = value === undefined || value === null ? "" : String(value);
      return el("td", { class: "pvalue" }, select);
    }
    default:
      return el("td", { class: "pvalue" }, show(spec, value));
  }
}

function fieldRow(spec: FieldSpec, model: FormModel): HTMLElement[] {
  // A widget wider than the value column -- a note, a profile, the point
  // table -- takes the whole row, label above, so it is never squeezed into
  // the fixed-width columns to the right of the name.
  const wide = (className: string, body: HTMLElement | string): HTMLElement[] => [
    el("tr", { class: className }, el("td", { colSpan: 5, class: "pwide" },
      el("div", { class: "pwidename", title: spec.hint ?? "" }, spec.label),
      body)),
  ];
  if (spec.kind === "note") return wide("pnote", el("div", { class: "pnotetext" }, spec.text ?? ""));
  if (spec.kind === "profile") {
    const current = read(spec, model.resolved, model) as ScalarFunction | undefined;
    return wide("pprofile", model.profileEditor(spec, current, (fn) => commit(model, spec, fn)));
  }
  if (spec.kind === "points") {
    const points = (read(spec, model.resolved, model) as PosedPoint[] | undefined) ?? [];
    return wide("ppoints", pointsTable(points, spec, model));
  }
  const base = read(spec, model.beneath, model);
  const current = read(spec, model.resolved, model);
  const changed = show(spec, base) !== show(spec, current);
  const revertBlocked = spec.readOnly ?? (changed ? null : "unchanged");
  return [el(
    "tr",
    { data: { changed: String(changed) } },
    el("th", { scope: "row", class: "pname", title: spec.hint ?? "" }, spec.label),
    el("td", { class: "pdefault" }, show(spec, base)),
    valueCell(spec, model),
    el("td", { class: "punit" }, spec.unit ?? ""),
    el("td", { class: "pact" }, el("button", {
      class: "plus",
      type: "button",
      title: revertBlocked ?? `back to ${show(spec, base)}`,
      disabled: Boolean(revertBlocked),
      on: {
        click: () => {
          const value = base === undefined ? null : (base as EditValue);
          // Defaults are already in storage units, not the displayed mm/degrees.
          commit(model, { ...spec, displayScale: undefined }, spec.scalar && value !== null && typeof value === "object" ? (constantOf(value) as number) : value);
        },
      },
    }, "↺")),
  )];
}

const MM = 1000;
const DEG = 180 / Math.PI;

/** The x y z (mm) · roll pitch yaw (°) table of a posed weld. Edits commit
 *  the whole list, so the node is never left with a half-typed point. */
function pointsTable(points: PosedPoint[], spec: FieldSpec, model: FormModel): HTMLElement {
  const draft = structuredClone(points);
  const disabled = Boolean(spec.readOnly);
  const cell = (index: number, axis: number, scale: number, step: number): HTMLElement => {
    const point = draft[index]!;
    const vector = axis < 3 ? point.xyz_m : point.rpy_rad;
    const k = axis < 3 ? axis : axis - 3;
    return el("td", {}, el("input", {
      type: "number",
      step,
      value: String(Number((vector[k]! * scale).toFixed(scale === MM ? 3 : 4))),
      disabled,
      on: {
        change: (event: Event) => {
          const raw = Number((event.target as HTMLInputElement).value);
          if (!Number.isFinite(raw)) return;
          vector[k] = raw / scale;
          commit(model, spec, structuredClone(draft));
        },
      },
    }));
  };
  const rows = draft.map((_, index) => el(
    "tr",
    {},
    el("th", { scope: "row" }, String(index + 1)),
    cell(index, 0, MM, 0.1), cell(index, 1, MM, 0.1), cell(index, 2, MM, 0.1),
    cell(index, 3, DEG, 0.5), cell(index, 4, DEG, 0.5), cell(index, 5, DEG, 0.5),
    el("td", { class: "pact" },
      el("button", {
        class: "plus", type: "button", disabled: disabled || draft.length <= 2,
        title: draft.length <= 2 ? "a weld needs at least two points" : "remove this point",
        on: { click: () => { draft.splice(index, 1); commit(model, spec, structuredClone(draft)); } },
      }, "×"),
      el("button", {
        class: "plus", type: "button", disabled,
        title: "insert a point after this one, halfway to the next",
        on: {
          click: () => {
            const a = draft[index]!;
            const b = draft[index + 1] ?? a;
            const mid: PosedPoint = {
              xyz_m: [0, 1, 2].map((i) => (a.xyz_m[i]! + b.xyz_m[i]!) / 2) as [number, number, number],
              rpy_rad: [...a.rpy_rad] as [number, number, number],
            };
            draft.splice(index + 1, 0, mid);
            commit(model, spec, structuredClone(draft));
          },
        },
      }, "+"),
    ),
  ));
  return el(
    "table",
    { class: "points-table" },
    el("thead", {}, el("tr", {},
      el("th", {}, "#"),
      el("th", {}, "x · mm"), el("th", {}, "y · mm"), el("th", {}, "z · mm"),
      el("th", {}, "roll · °"), el("th", {}, "pitch · °"), el("th", {}, "yaw · °"),
      el("th", {}, ""))),
    el("tbody", {}, rows),
  );
}

function groupHead(group: FieldGroup): HTMLElement {
  return el("tr", { class: "paramgroup" }, el("th", { colSpan: 5 },
    group.group,
    group.note
      ? el("details", { class: "paramwhy" }, el("summary", { title: `what "${group.group}" does` }, "?"), el("p", {}, group.note))
      : null,
  ));
}

/** The table: a head naming the columns once, then a body per group. */
export function fieldForm(groups: FieldGroup[], model: FormModel): HTMLElement {
  return el(
    "table",
    { class: "params" },
    el("thead", {}, el("tr", {},
      el("th", { class: "pname" }, "parameter"),
      el("th", { class: "pdefault" }, "default"),
      el("th", { class: "pvalue" }, "value"),
      el("th", { class: "punit" }, "unit"),
      el("th", { class: "pact" }, ""))),
    groups.map((group) => el("tbody", { data: { group: group.id } }, groupHead(group), group.fields.flatMap((spec) => fieldRow(spec, model)))),
  );
}
