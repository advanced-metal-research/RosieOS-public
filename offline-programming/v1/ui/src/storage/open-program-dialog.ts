/**
 * Program ▸ Open program…: the cell's saved programs and the programs only
 * this browser keeps, in one list to search and open from.
 *
 * Listing, refusals and the draft question are the shared engine's
 * (cell-sync.ts picker, openCheck, openAfter) -- the Deck's OPEN runs the
 * same three through olp-core -- so the two clients cannot disagree about
 * which program may be opened or what happens to an unsaved edit. This file
 * only draws them:
 * - an edit the cell has not accepted is never replaced silently: the
 *   operator saves it first, keeps it as a draft in this browser, discards
 *   it, or cancels;
 * - a program the robot has loaded, playing or looping is not switched away
 *   from (the row says why);
 * - with no store or no cell, the reason is shown and this browser's own
 *   programs stay openable.
 */
import { ago, type CellOpenCheck, type CellOpenChoice, type CellPicker, type CellPickerRow, type CellSyncView } from "./cell-sync.ts";
import type { ProjectSummary } from "./repository.ts";
import { Dialog } from "../ui/dialog.ts";
import { element } from "../ui/widget.ts";

export interface OpenProgramSync {
  picker(runLock?: string): CellPicker;
  openCheck(programId: string, runLock?: string, local?: boolean): CellOpenCheck;
  openAfter(programId: string, choice: CellOpenChoice, runLock?: string, local?: boolean): string;
  view(): CellSyncView;
}

export interface OpenProgramHost {
  sync: OpenProgramSync;
  /** Every program this browser keeps (IndexedDB), newest first. */
  listLocal(): Promise<ProjectSummary[]>;
  /** Why this browser must not switch programs now (programSwitchLock), or "". */
  runLock(): string;
  notice(message: string): void;
  now?(): number;
}

interface Item {
  programId: string;
  local: boolean;
  blocked: string;
  button: HTMLButtonElement;
}

export class OpenProgramDialog {
  readonly dialog: Dialog;
  readonly #host: OpenProgramHost;
  readonly #search: HTMLInputElement;
  readonly #status: HTMLParagraphElement;
  readonly #cellSection: HTMLDivElement;
  readonly #cellList: HTMLDivElement;
  readonly #localSection: HTMLDivElement;
  readonly #localList: HTMLDivElement;
  readonly #guard: HTMLDivElement;
  #items: Item[] = [];
  #local: ProjectSummary[] = [];
  #active = 0;

  constructor(host: OpenProgramHost) {
    this.#host = host;
    this.dialog = new Dialog({ title: "Open program", className: "open-program-dialog" });
    const top = this.dialog.addSection();
    this.#search = element("input", "open-program-search");
    this.#search.type = "search";
    this.#search.placeholder = "Search by name, robot or who saved it";
    this.#search.setAttribute("aria-label", "Search programs");
    this.#search.autocomplete = "off";
    this.#search.spellcheck = false;
    this.#status = element("p", "open-program-status");
    this.#status.setAttribute("role", "status");
    top.append(this.#search, this.#status);
    this.#cellSection = this.dialog.addSection({ title: "On the cell" });
    this.#cellList = element("div", "open-program-list");
    this.#cellList.setAttribute("role", "listbox");
    this.#cellList.setAttribute("aria-label", "The cell's programs");
    this.#cellSection.append(this.#cellList);
    this.#localSection = this.dialog.addSection({ title: "This browser only" });
    this.#localList = element("div", "open-program-list");
    this.#localList.setAttribute("role", "listbox");
    this.#localList.setAttribute("aria-label", "Programs only in this browser");
    this.#localSection.append(this.#localList);
    this.#guard = element("div", "open-program-guard");
    this.#guard.setAttribute("role", "alertdialog");
    this.#guard.hidden = true;
    this.dialog.panel.append(this.#guard);
    this.#search.addEventListener("input", () => {
      this.#active = 0;
      this.render();
    });
    this.dialog.panel.addEventListener("keydown", (event) => this.#key(event));
    this.dialog.mount(document.body);
  }

  get shown(): boolean {
    return this.dialog.shown;
  }

  /** Read this browser's programs, draw both lists and open the dialog. */
  async open(): Promise<void> {
    this.#search.value = "";
    this.#active = 0;
    this.#guard.hidden = true;
    try {
      this.#local = await this.#host.listLocal();
    } catch (error) {
      this.#local = [];
      this.#host.notice(`This browser's programs could not be read: ${error instanceof Error ? error.message : String(error)}`);
    }
    this.render();
    this.dialog.open();
    this.#search.focus();
  }

  close(): void {
    this.#guard.hidden = true;
    this.dialog.close();
  }

  /** The rows as shown, for checks: [programId, local, text]. */
  rows(): { programId: string; local: boolean; text: string; blocked: string }[] {
    return this.#items.map((i) => ({ programId: i.programId, local: i.local, text: i.button.textContent ?? "", blocked: i.blocked }));
  }

  render(): void {
    const now = this.#host.now?.() ?? Date.now();
    const picker = this.#host.sync.picker(this.#host.runLock());
    const view = this.#host.sync.view();
    const query = this.#search.value.trim().toLowerCase();
    // A redraw (a save on the cell, presence) keeps the keyboard where it was.
    const hadFocus = this.#items.some((i) => i.button === document.activeElement);
    const matches = (...fields: string[]) => !query || fields.some((f) => f.toLowerCase().includes(query));
    this.#items = [];
    this.#cellList.replaceChildren();
    this.#localList.replaceChildren();
    const disabled = view.state === "disabled";
    this.#cellSection.closest(".dialog-section")!.toggleAttribute("hidden", disabled);
    const cellRows = picker.rows.filter((r) => matches(r.name, r.model, r.by, r.openBy));
    this.dialog.setSectionSummary(this.#cellSection, `${picker.host} · ${picker.rows.length} program${picker.rows.length === 1 ? "" : "s"}`);
    for (const row of cellRows) this.#cellList.append(this.#cellRow(row));
    if (!disabled && cellRows.length === 0) {
      this.#cellList.append(element("p", "open-program-empty",
        picker.rows.length === 0 ? (picker.unavailable || `No saved programs on ${picker.host} yet.`) : "No program matches the search."));
    }
    // Programs on the cell are opened from the cell (a draft of one this
    // browser keeps is marked on its row); the rest only live here.
    const onCell = new Set(picker.rows.map((r) => r.programId));
    const localRows = this.#local.filter((p) => !onCell.has(p.id) && matches(p.name, p.robotModel));
    const here = view.programId;
    const localTitle = disabled ? "This browser" : "This browser only";
    const titleEl = this.#localSection.closest(".dialog-section")?.querySelector(".dialog-section-title");
    if (titleEl) titleEl.textContent = localTitle;
    this.dialog.setSectionSummary(this.#localSection, disabled ? "" : "not on the cell: the Deck and other browsers do not see these");
    for (const summary of localRows) this.#localList.append(this.#localRow(summary, summary.id === here, now));
    if (localRows.length === 0) {
      this.#localList.append(element("p", "open-program-empty", query ? "No program matches the search."
        : this.#local.length === 0 ? "No programs in this browser." : `Every program in this browser is on ${picker.host}.`));
    }
    this.#status.textContent = picker.unavailable || (picker.rows.find((r) => r.blocked)?.blocked ?? "") ||
      "↑ ↓ choose · Enter opens · Esc closes";
    this.#status.dataset.tone = picker.unavailable || picker.rows.some((r) => r.blocked) ? "caution" : "";
    this.#active = Math.max(0, Math.min(this.#active, this.#items.length - 1));
    this.#mark();
    if (hadFocus) this.#items[this.#active]?.button.focus();
  }

  #cellRow(row: CellPickerRow): HTMLButtonElement {
    const marks = [
      row.current ? "CURRENT" : "", row.running ? "ON THE ROBOT" : "", row.here ? "OPEN HERE" : "", row.draft ? "DRAFT HERE" : "",
    ].filter(Boolean);
    const button = this.#row(row.programId, false, row.blocked, `${row.current ? "★ " : ""}${row.name}`,
      [row.model, row.openBy ? `open on ${row.openBy}` : ""].filter(Boolean).join(" · "),
      `rev ${row.revision}`, `${row.when || "unknown"}${row.by ? ` · ${row.by}` : ""}`, marks);
    const when = Date.parse(row.updatedAt);
    button.title = row.blocked || [
      `Open it from the cell; it becomes the cell's current program (the Steam Deck follows)`,
      Number.isFinite(when) ? `Last saved ${new Date(when).toLocaleString()}${row.by ? ` by ${row.by}` : ""}` : "",
      row.draft ? "This browser keeps unsaved edits of it: they are opened, then saved by the usual rules" : "",
    ].filter(Boolean).join("\n");
    return button;
  }

  #localRow(summary: ProjectSummary, here: boolean, now: number): HTMLButtonElement {
    const when = ago(summary.updatedAt, now);
    const button = this.#row(summary.id, true, "", summary.name, summary.robotModel, `rev ${summary.revision}`, when || "unknown",
      here ? ["OPEN HERE"] : []);
    const at = Date.parse(summary.updatedAt);
    button.title = `Open this browser's copy${Number.isFinite(at) ? ` · edited ${new Date(at).toLocaleString()}` : ""}`;
    return button;
  }

  #row(programId: string, local: boolean, blocked: string, name: string, meta: string, revision: string, when: string,
    marks: string[]): HTMLButtonElement {
    const button = element("button", "open-program-row");
    button.type = "button";
    button.setAttribute("role", "option");
    button.dataset.programId = programId;
    button.tabIndex = -1;
    if (blocked) {
      button.setAttribute("aria-disabled", "true");
      button.classList.add("is-blocked");
    }
    const title = element("span", "open-program-name");
    title.append(element("span", "open-program-title", name));
    if (meta) title.append(element("span", "open-program-meta", meta));
    const badges = element("span", "open-program-marks");
    for (const mark of marks) {
      const badge = element("span", "open-program-mark", mark);
      badge.dataset.mark = mark.toLowerCase().replace(/ /g, "-");
      badges.append(badge);
    }
    button.append(title, element("span", "open-program-rev", revision), element("span", "open-program-when", when), badges);
    const index = this.#items.length;
    button.addEventListener("click", () => {
      this.#active = index;
      this.#mark();
      this.choose(programId, local);
    });
    this.#items.push({ programId, local, blocked, button });
    return button;
  }

  #mark(): void {
    this.#items.forEach((item, i) => {
      item.button.setAttribute("aria-selected", String(i === this.#active));
      item.button.classList.toggle("is-active", i === this.#active);
    });
  }

  #key(event: KeyboardEvent): void {
    if (!this.#guard.hidden) return;
    if (event.key === "ArrowDown" || event.key === "ArrowUp") {
      event.preventDefault();
      if (this.#items.length === 0) return;
      const fromSearch = document.activeElement === this.#search;
      const step = event.key === "ArrowDown" ? 1 : -1;
      this.#active = fromSearch && step > 0 ? this.#active : Math.max(0, Math.min(this.#items.length - 1, this.#active + step));
      if (fromSearch && step < 0) return;
      this.#mark();
      const button = this.#items[this.#active]!.button;
      button.focus();
      button.scrollIntoView?.({ block: "nearest" });
    } else if (event.key === "Enter") {
      const focused = this.#items.findIndex((i) => i.button === document.activeElement);
      if (focused < 0 && document.activeElement !== this.#search) return;
      event.preventDefault();
      if (focused >= 0) this.#active = focused;
      const item = this.#items[this.#active];
      if (item) this.choose(item.programId, item.local);
    } else if (event.key.length === 1 && document.activeElement !== this.#search && !event.ctrlKey && !event.metaKey && !event.altKey) {
      // Typing anywhere in the list searches.
      this.#search.focus();
    }
  }

  /** Open a row: refused with its reason, already open, the draft question, or opened. */
  choose(programId: string, local: boolean): void {
    const runLock = this.#host.runLock();
    const check = this.#host.sync.openCheck(programId, runLock, local);
    if (check.blocked) {
      this.#say(check.blocked, "danger");
      return;
    }
    if (check.here) {
      this.#host.notice(`${this.#nameOf(programId, local)} is already open`);
      this.close();
      return;
    }
    if (check.draft) {
      this.#ask(programId, local, check);
      return;
    }
    this.#perform(programId, "open", local);
  }

  #nameOf(programId: string, local: boolean): string {
    if (local) return this.#local.find((p) => p.id === programId)?.name ?? programId;
    return this.#host.sync.picker().rows.find((r) => r.programId === programId)?.name ?? programId;
  }

  #say(text: string, tone: string): void {
    this.#status.textContent = text;
    this.#status.dataset.tone = tone;
  }

  #perform(programId: string, choice: CellOpenChoice, local: boolean): void {
    try {
      const message = this.#host.sync.openAfter(programId, choice, this.#host.runLock(), local);
      this.#host.notice(message);
      this.close();
    } catch (error) {
      this.#guard.hidden = true;
      this.#say(error instanceof Error ? error.message : String(error), "danger");
    }
  }

  /** The open program has edits the cell has not accepted: save, keep, discard or cancel. */
  #ask(programId: string, local: boolean, check: CellOpenCheck): void {
    const draft = check.draft!;
    const host = this.#host.sync.view().host;
    const target = this.#nameOf(programId, local);
    this.#guard.replaceChildren();
    const heading = element("h4", "open-program-guard-title", `Unsaved edits in "${draft.name}"`);
    heading.id = "open-program-guard-title";
    this.#guard.setAttribute("aria-labelledby", heading.id);
    this.#guard.append(heading, element("p", "open-program-guard-text", `${draft.text} What happens to them before "${target}" opens?`));
    if (draft.saveRefused) this.#guard.append(element("p", "open-program-guard-why", `Saving to ${host} is not possible now: ${draft.saveRefused}.`));
    const actions = element("div", "open-program-guard-actions");
    const button = (label: string, title: string, run: () => void, primary = false, disabled = false): HTMLButtonElement => {
      const b = element("button", primary ? "mini-button primary" : "mini-button", label);
      b.type = "button";
      b.title = title;
      b.disabled = disabled;
      b.addEventListener("click", run);
      actions.append(b);
      return b;
    };
    const save = button(`Save to ${host}, then open`, draft.saveRefused || `Send the edits to ${host}; "${target}" opens once it accepted them`,
      () => this.#perform(programId, "save", local), !draft.saveRefused, Boolean(draft.saveRefused));
    const keep = draft.keptHere
      ? button("Keep as a draft in this browser", "The edits stay here and are saved to the cell when this program is opened again",
        () => this.#perform(programId, "keep", local), Boolean(draft.saveRefused))
      : null;
    button("Discard my edits", `Drop the edits ${host} has not saved`, () => this.#perform(programId, "discard", local));
    button("Cancel", "Keep this program open", () => {
      this.#guard.hidden = true;
      this.#items[this.#active]?.button.focus();
    });
    this.#guard.append(actions);
    this.#guard.hidden = false;
    (draft.saveRefused ? keep ?? actions.querySelector<HTMLButtonElement>("button:not(:disabled)") : save)?.focus();
  }

  /** The draft question's buttons, for checks. */
  guardButtons(): HTMLButtonElement[] {
    return this.#guard.hidden ? [] : [...this.#guard.querySelectorAll<HTMLButtonElement>("button")];
  }

  status(): string {
    return this.#status.textContent ?? "";
  }
}
