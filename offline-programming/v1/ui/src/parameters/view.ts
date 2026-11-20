/**
 * The parameter window: a spreadsheet of parameters, and the four things an
 * operator does with a set of numbers -- change one, put one back, put them
 * all back, and save what they arrived at under a name.
 *
 * One of these per parameter surface. `seam.search` and `seam.torch` use it
 * now; the weld and free-space planner options are the reason it is a
 * primitive rather than a panel.
 *
 * ## Why a grid and not a form
 *
 * Modelled on the parameter table in an analysis tool -- lettered columns, a
 * numbered row per parameter, an ID, a unit column. The point is not the
 * decoration: it is that a grid puts WHERE THE NUMBER CAME FROM in a column
 * of its own. `Preset` sits beside `Value` on every row, so a set of numbers
 * can be read down two columns and every override is visible at once. A
 * label-and-input form can only mark an override, which tells you that
 * something moved but not from what.
 *
 * The ID column is there for the same reason it is in those tools: a
 * parameter needs a short handle to be talked about, in a note or across a
 * desk, that is not its whole prose label.
 *
 * ## The rows are built once and updated in place
 *
 * Not rebuilt from a template on every change. `rails/dom.ts` gives the
 * reason: a rebuild throws away focus, scroll position and the caret, and a
 * window of thirty parameters cannot afford to lose the one being typed into.
 * `refresh()` writes values into the inputs that exist, and skips the one
 * holding focus so a half-typed number is not overwritten under the cursor.
 */

import { el, replace } from "../rails/dom.ts";
import type { PresetLibrary } from "./library.ts";
import type {
  ParameterConstraints,
  ParameterField,
  ParameterPreset,
  ParameterSchema,
  ParameterSet,
  ParameterValue,
  ParameterValues,
} from "./model.ts";
import { createParameterSet } from "./model.ts";

/** The data columns, in order. The row-number gutter is not one of them --
 *  it is the sheet's own margin, the way it is in a spreadsheet. */
const COLUMNS = ["ID", "Parameter", "Value", "Unit", "Preset"] as const;

export interface ParameterViewOptions {
  schema: ParameterSchema;
  library: PresetLibrary;
  /** Opening values, when the caller has some -- a program's saved
   *  parameters, say. Missing keys come from the preset. */
  initial?: { presetId?: string; values?: ParameterValues };
  /** Rules across fields -- see `ParameterConstraints`. Shown in the window
   *  as a warning, not enforced at the input: refusing the first edit of a
   *  pair would make the second one unreachable. */
  constraints?: ParameterConstraints;
  /** Any change to the values: an edit, a revert, a preset load. Called with
   *  the whole set, because that is what a caller sends. */
  onChange(values: Record<string, ParameterValue>, view: ParameterView): void;
  /** Progress and refusals, in the host's own status line. */
  status?(text: string, isError?: boolean): void;
}

export interface ParameterView {
  readonly root: HTMLElement;
  readonly set: ParameterSet;
  values(): Record<string, ParameterValue>;
  /** Redraw from the model. Call after changing the set from outside. */
  refresh(): void;
  /** Read the operator's saved presets and redraw the picker. */
  ready(): Promise<void>;
}

export function createParameterView(options: ParameterViewOptions): ParameterView {
  const { schema, library, onChange } = options;
  const status = options.status ?? (() => {});
  const opening = (options.initial?.presetId ? library.get(options.initial.presetId) : null)
    ?? library.fallback;
  const set = createParameterSet(schema, opening, {
    initial: options.initial?.values,
    constraints: options.constraints,
  });

  const root = el("div", { class: "parameter-view" });
  const presetSelect = el("select", {
    class: "parameter-preset",
    "aria-label": "Preset",
    on: { change: () => choosePreset(presetSelect.value) },
  }) as HTMLSelectElement;
  const summary = el("span", { class: "parameter-summary" });
  const saveButton = el("button", {
    type: "button", class: "mini-button", on: { click: () => void save() },
  }, "Save");
  const saveAsButton = el("button", {
    type: "button", class: "mini-button", on: { click: () => openSaveAs() },
  }, "Save as…");
  const revertAllButton = el("button", {
    type: "button", class: "mini-button", on: { click: () => revertAll() },
  }, "Revert all");

  const nameInput = el("input", {
    type: "text", class: "parameter-name", placeholder: "Preset name",
    on: {
      keydown: (event: Event) => {
        const key = (event as KeyboardEvent).key;
        if (key === "Enter") void saveAs();
        if (key === "Escape") closeSaveAs();
      },
    },
  }) as HTMLInputElement;
  const saveAsRow = el(
    "div",
    { class: "parameter-saveas", hidden: true },
    nameInput,
    el("button", { type: "button", class: "mini-button primary", on: { click: () => void saveAs() } }, "Save"),
    el("button", { type: "button", class: "mini-button", on: { click: () => closeSaveAs() } }, "Cancel"),
  );
  const problems = el("ul", { class: "parameter-problems", hidden: true });

  interface Row {
    field: ParameterField;
    input: HTMLInputElement | HTMLSelectElement;
    presetCell: HTMLElement;
    revert: HTMLButtonElement;
    element: HTMLElement;
  }
  const rows = new Map<string, Row>();

  root.append(
    el(
      "div",
      { class: "parameter-header" },
      presetSelect,
      saveButton,
      saveAsButton,
      revertAllButton,
      summary,
    ),
    saveAsRow,
    problems,
    el("div", { class: "parameter-sheet" }, buildGrid()),
  );
  refresh();

  function buildGrid(): HTMLElement {
    // `table-layout: fixed` sizes from these, so the numeric columns keep a
    // known width and only the name column takes what is left.
    const columns = el(
      "colgroup",
      {},
      el("col", { class: "col-rownum" }),
      ["col-id", "col-name", "col-value", "col-unit", "col-preset"].map((name) =>
        el("col", { class: name })),
    );
    const head = el(
      "thead",
      {},
      // Column letters, the way a sheet labels them. The corner cell is the
      // row-number gutter's own header and stays empty.
      el(
        "tr",
        { class: "parameter-letters" },
        el("th", { class: "parameter-corner" }),
        COLUMNS.map((_, index) => el("th", {}, String.fromCharCode(65 + index))),
      ),
      el(
        "tr",
        { class: "parameter-head" },
        el("th", { class: "parameter-rownum" }, "1"),
        COLUMNS.map((name) => el("th", {}, name)),
      ),
    );

    const body = el("tbody", {});
    // Row 1 is the header, so the sheet's own numbering starts at 2 -- and a
    // group heading takes a row like everything else, because in a sheet it
    // occupies one.
    let line = 2;
    let group: string | undefined;
    let parameter = 0;
    for (const field of schema) {
      if (field.group && field.group !== group) {
        group = field.group;
        body.append(el(
          "tr",
          { class: "parameter-group-row" },
          el("td", { class: "parameter-rownum" }, String(line)),
          el("td", { colspan: COLUMNS.length }, field.group),
        ));
        line += 1;
      }
      parameter += 1;
      const row = buildRow(field, line, `P${parameter}`);
      rows.set(field.key, row);
      body.append(row.element);
      line += 1;
    }
    return el("table", { class: "parameter-grid" }, columns, head, body);
  }

  function buildRow(field: ParameterField, line: number, id: string): Row {
    const input = fieldInput(field);
    const revert = el("button", {
      type: "button",
      class: "parameter-revert",
      "aria-label": `Revert ${field.label} to the preset`,
      on: {
        click: (event: Event) => {
          event.preventDefault();
          revertField(field.key);
        },
      },
    }, "↺") as HTMLButtonElement;
    const presetCell = el("td", { class: "parameter-preset-cell" });
    const element = el(
      "tr",
      { class: "parameter-row", data: { key: field.key }, title: field.help ?? "" },
      el("td", { class: "parameter-rownum" }, String(line)),
      el("td", { class: "parameter-id" }, id),
      el("td", { class: "parameter-name-cell" }, field.label),
      el("td", { class: "parameter-value-cell" }, input),
      el("td", { class: "parameter-unit-cell" }, field.kind === "number" ? (field.unit ?? "") : ""),
      presetCell,
    );
    return { field, input, presetCell, revert, element };
  }

  function fieldInput(field: ParameterField): HTMLInputElement | HTMLSelectElement {
    if (field.kind === "choice") {
      const select = el("select", {
        class: "parameter-input",
        on: { change: () => commit(field, select.value) },
      }) as HTMLSelectElement;
      for (const option of field.options) {
        select.append(el("option", { value: option.value }, option.label));
      }
      return select;
    }
    if (field.kind === "toggle") {
      const box = el("input", {
        type: "checkbox",
        class: "parameter-input parameter-toggle",
        on: { change: () => commit(field, box.checked) },
      }) as HTMLInputElement;
      return box;
    }
    const number = el("input", {
      type: "number",
      class: "parameter-input",
      inputmode: "decimal",
      min: field.min,
      max: field.max,
      step: field.step ?? "any",
      // `change`, not `input`: a partly typed number is not a number, and
      // committing every keystroke sends "1" on the way to "15".
      on: { change: () => commit(field, Number(number.value)) },
    }) as HTMLInputElement;
    return number;
  }

  function commit(field: ParameterField, value: ParameterValue): void {
    const outcome = set.set(field.key, value);
    if (!outcome.ok) {
      status(`${field.label} ${outcome.reason}`, true);
      // Put the accepted value back on screen, so the cell never shows a
      // number the set does not hold.
      refresh();
      return;
    }
    refresh();
    onChange(set.values(), view);
  }

  function revertField(key: string): void {
    const state = set.state(key);
    if (!state.modified) return;
    set.revert(key);
    refresh();
    status(`${state.field.label} back to ${format(state.field, state.presetValue)}`);
    onChange(set.values(), view);
  }

  function revertAll(): void {
    const count = set.modifiedKeys().length;
    if (count === 0) return;
    set.revertAll();
    refresh();
    status(`${count} parameter${count === 1 ? "" : "s"} back to "${set.preset.label}"`);
    onChange(set.values(), view);
  }

  function choosePreset(id: string): void {
    const preset = library.get(id);
    if (!preset) return;
    closeSaveAs();
    set.loadPreset(preset);
    refresh();
    status(`Loaded "${preset.label}"`);
    onChange(set.values(), view);
  }

  async function save(): Promise<void> {
    const outcome = await library.save(set.preset.id, set.values());
    if (!outcome.ok) {
      status(outcome.reason, true);
      // A shipped preset refuses Save and Save as is the way through, so open
      // it rather than leaving the operator to find the other button.
      if (set.preset.origin === "shipped") openSaveAs(`${set.preset.label} (edited)`);
      return;
    }
    adopt(outcome.preset, `Saved "${outcome.preset.label}"`);
  }

  async function saveAs(): Promise<void> {
    const outcome = await library.saveAs(nameInput.value, set.values());
    if (!outcome.ok) {
      status(outcome.reason, true);
      return;
    }
    closeSaveAs();
    adopt(outcome.preset, `Saved "${outcome.preset.label}"`);
  }

  /** The values were just written into a preset: they have not changed, but
   *  what they are compared against has, so nothing reads as modified now. */
  function adopt(preset: ParameterPreset, message: string): void {
    set.rebase(preset);
    renderPresets();
    refresh();
    status(message);
  }

  function openSaveAs(suggested?: string): void {
    saveAsRow.hidden = false;
    nameInput.value = suggested ?? (set.preset.origin === "user" ? set.preset.label : "");
    nameInput.focus();
    nameInput.select();
  }

  function closeSaveAs(): void {
    saveAsRow.hidden = true;
  }

  function renderPresets(): void {
    replace(
      presetSelect,
      library.all().map((preset) =>
        el(
          "option",
          { value: preset.id, selected: preset.id === set.preset.id },
          preset.origin === "shipped" ? `${preset.label} (shipped)` : preset.label,
        )
      ),
    );
    presetSelect.value = set.preset.id;
  }

  function refresh(): void {
    if (presetSelect.options.length === 0) renderPresets();
    presetSelect.value = set.preset.id;
    for (const state of set.states()) {
      const row = rows.get(state.field.key);
      if (!row) continue;
      // Never overwrite the cell being typed into.
      if (document.activeElement !== row.input) writeInput(row, state.value);
      row.element.classList.toggle("is-modified", state.modified);
      // The preset column carries the value it would go back to, and the
      // control that takes it there. Reading down this column is how an
      // operator sees every override at once.
      replace(
        row.presetCell,
        el("span", { class: "parameter-preset-value" }, format(state.field, state.presetValue, false)),
        state.modified ? row.revert : null,
      );
      row.revert.title = `Back to ${format(state.field, state.presetValue)} — "${set.preset.label}"`;
    }
    const count = set.modifiedKeys().length;
    summary.textContent = count === 0
      ? `matches "${set.preset.label}"`
      : `${count} off "${set.preset.label}"`;
    summary.classList.toggle("is-modified", count > 0);
    revertAllButton.disabled = count === 0;
    // Save stays LIVE on a shipped preset even though it cannot write one.
    // Disabling it would be a dead button whose only explanation is a
    // tooltip; pressing it says why in the status line and opens Save as
    // with a name filled in, which is what the operator wanted anyway.
    saveButton.title = set.preset.origin === "shipped"
      ? `"${set.preset.label}" is shipped — this offers Save as instead`
      : `Write these values into "${set.preset.label}"`;
    const across = set.problems();
    replace(problems, across.map((problem) => el("li", {}, problem)));
    problems.hidden = across.length === 0;
  }

  function writeInput(row: Row, value: ParameterValue): void {
    if (row.field.kind === "toggle") {
      (row.input as HTMLInputElement).checked = Boolean(value);
      return;
    }
    if (row.field.kind === "number") {
      row.input.value = format(row.field, value, false);
      return;
    }
    row.input.value = String(value);
  }

  const view: ParameterView = {
    root,
    set,
    values: () => set.values(),
    refresh,
    async ready() {
      await library.load();
      renderPresets();
      refresh();
    },
  };
  return view;
}

/** A value as the operator reads it. `withUnit` off for a cell whose column
 *  already carries the unit, and for an input's own value, which must parse
 *  back as a number. */
export function format(field: ParameterField, value: ParameterValue, withUnit = true): string {
  if (field.kind === "toggle") return value ? "on" : "off";
  if (field.kind === "choice") {
    return field.options.find((option) => option.value === value)?.label ?? String(value);
  }
  const digits = field.digits ?? decimalsOf(field.step);
  const text = typeof value === "number" ? value.toFixed(digits) : String(value);
  return withUnit && field.unit ? `${text} ${field.unit}` : text;
}

/** How many decimals a step implies: 0.1 -> 1, 0.001 -> 3, 1 -> 0. */
function decimalsOf(step: number | undefined): number {
  if (!step || !Number.isFinite(step)) return 2;
  const text = String(step);
  const point = text.indexOf(".");
  return point < 0 ? 0 : text.length - point - 1;
}
