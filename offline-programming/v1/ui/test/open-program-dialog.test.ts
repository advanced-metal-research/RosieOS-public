import assert from "node:assert/strict";
import test from "node:test";
import { setImmediate } from "node:timers/promises";

import { createBlankSpreadsheetProject } from "../src/domain/spreadsheet.ts";
import { reviseProject, type OfflineProgrammingProject } from "../src/domain/project.ts";
import { BrowserCellSync } from "../src/storage/cell-sync-browser.ts";
import { OpenProgramDialog } from "../src/storage/open-program-dialog.ts";
import type { ProjectSummary } from "../src/storage/repository.ts";

import { DECK, FakeCell, MemoryStorage, TAB } from "./support/fake-cell.ts";

async function withDocument(t: test.TestContext) {
  const { Window } = await import("happy-dom");
  const browser = new Window();
  browser.document.body.innerHTML = `<span id="cell-sync" hidden></span>`;
  for (const [name, value] of [["document", browser.document], ["HTMLElement", browser.HTMLElement]] as const) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  return browser;
}

function rename(project: OfflineProgrammingProject, name: string): OfflineProgrammingProject {
  return reviseProject(project, "metadata", (draft) => { draft.identity.name = name; });
}

function summary(project: OfflineProgrammingProject): ProjectSummary {
  return {
    id: project.identity.id, name: project.identity.name, revision: project.identity.revision, updatedAt: project.identity.updatedAt,
    robotModel: project.robot.model, planState: project.plan.state,
  };
}

/** The browser as main.ts wires it: the open program, this browser's own
 *  copies (IndexedDB, here a map the "autosave" writes on every switch), the
 *  sync and the Open program dialog. */
async function browserWith(t: test.TestContext, cell: FakeCell, first: OfflineProgrammingProject, runLock = () => "") {
  const window = await withDocument(t);
  let now = Date.parse("2026-10-11T18:03:00Z");
  const local = new Map<string, OfflineProgrammingProject>();
  const notices: string[] = [];
  const state = { project: first };
  local.set(first.identity.id, first);
  const switchTo = (next: OfflineProgrammingProject) => {
    local.set(state.project.identity.id, state.project);
    state.project = next;
    local.set(next.identity.id, next);
  };
  const sync = new BrowserCellSync({
    clientId: "tab-1", project: () => state.project,
    open: (next) => switchTo(next),
    openLocal: (id) => switchTo(local.get(id)!),
    notice: (text) => notices.push(text), changed: () => undefined,
  }, { fetch: cell.fetch, storage: new MemoryStorage() as unknown as Storage, clock: () => now });
  t.after(() => sync.stop());
  const dialog = new OpenProgramDialog({
    sync, runLock, notice: (text) => notices.push(text), now: () => now,
    listLocal: async () => [...local.values()].map(summary).sort((a, b) => b.updatedAt.localeCompare(a.updatedAt)),
  });
  const settle = async (advance = 1800) => {
    for (let i = 0; i < 6; i++) {
      now += advance / 6;
      sync.tick(now);
      await setImmediate();
      await setImmediate();
    }
  };
  const edit = (name: string) => {
    state.project = rename(state.project, name);
    local.set(state.project.identity.id, state.project);
    sync.tick(now);
  };
  const key = (key: string) => {
    const target = window.document.activeElement ?? dialog.dialog.panel;
    target.dispatchEvent(new window.KeyboardEvent("keydown", { key, bubbles: true, cancelable: true }));
  };
  return { window, state, local, notices, sync, dialog, settle, edit, key, rows: () => dialog.rows() };
}

test("Open program lists the cell's programs and this browser's own; search narrows them; the keyboard opens one", async (t) => {
  const cell = new FakeCell();
  const frame = createBlankSpreadsheetProject({ name: "Frame" });
  const bracket = createBlankSpreadsheetProject({ name: "Bracket" });
  cell.write(frame.identity.id, 0, frame, TAB);
  cell.write(bracket.identity.id, 0, bracket, DECK);
  cell.write(bracket.identity.id, 1, rename(bracket, "Bracket"), DECK);
  cell.current = bracket.identity.id;
  cell.presence = [{ program_id: bracket.identity.id, ...DECK, editing: false }];
  const updated = new Map([[frame.identity.id, "2026-10-11T17:00:00Z"], [bracket.identity.id, "2026-10-11T18:00:00Z"]]);
  const view = cell.view.bind(cell);
  cell.view = (watch) => {
    const body = view(watch) as { programs: { program_id: string; updated_at: string }[] };
    for (const p of body.programs) p.updated_at = updated.get(p.program_id)!;
    return body;
  };
  const scratch = createBlankSpreadsheetProject({ name: "Scratch here" });
  const b = await browserWith(t, cell, scratch);
  b.local.set("old", { ...createBlankSpreadsheetProject({ name: "Old local" }), identity: { ...scratch.identity, id: "old", name: "Old local", updatedAt: "2026-10-10T18:00:00Z" } });
  b.local.set(frame.identity.id, frame); // also on the cell: listed once, from the cell
  await b.settle();

  await b.dialog.open();
  assert.equal(b.dialog.shown, true);
  const rows = b.rows();
  assert.deepEqual(rows.map((r) => [r.programId, r.local]), [
    [bracket.identity.id, false], [frame.identity.id, false], [scratch.identity.id, true], ["old", true],
  ]);
  assert.match(rows[0]!.text, /★ Bracket/);
  assert.match(rows[0]!.text, /open on steamdeck-operator viewing/);
  assert.match(rows[0]!.text, /rev 2/);
  assert.match(rows[0]!.text, /3 min ago · steamdeck-operator/);
  assert.match(rows[0]!.text, /CURRENT/);
  assert.match(rows[1]!.text, /Frame.*rev 1.*1 h ago · Browser \(Chrome on macOS\)/);
  assert.match(rows[2]!.text, /Scratch here.*OPEN HERE/);
  assert.match(b.dialog.dialog.panel.textContent!, /This browser only/);

  // Search narrows both lists.
  const search = b.dialog.dialog.panel.querySelector<HTMLInputElement>(".open-program-search")!;
  search.value = "fra";
  search.dispatchEvent(new b.window.Event("input") as unknown as Event);
  assert.deepEqual(b.rows().map((r) => r.programId), [frame.identity.id]);
  assert.match(b.dialog.dialog.panel.textContent!, /No program matches the search\./);
  search.value = "";
  search.dispatchEvent(new b.window.Event("input") as unknown as Event);

  // ↓ from the search box focuses the first row, ↓ again the second, Enter opens it.
  search.focus();
  b.key("ArrowDown");
  assert.equal(b.window.document.activeElement?.getAttribute("data-program-id"), bracket.identity.id);
  b.key("ArrowDown");
  assert.equal(b.window.document.activeElement?.getAttribute("data-program-id"), frame.identity.id);
  b.key("Enter");
  assert.equal(b.dialog.shown, false);
  await b.settle();
  assert.equal(b.state.project.identity.id, frame.identity.id);
  assert.equal(cell.current, frame.identity.id, "opening makes it the cell's current program: the Deck follows");
  assert.ok(b.local.has(scratch.identity.id), "the browser-only program stays in this browser");
});

test("an unsaved edit is never replaced silently: cancel keeps it, save sends it first, discard drops it", async (t) => {
  const cell = new FakeCell();
  const frame = createBlankSpreadsheetProject({ name: "Frame" });
  cell.write(frame.identity.id, 0, frame, TAB);
  const bracket = createBlankSpreadsheetProject({ name: "Bracket" });
  const b = await browserWith(t, cell, bracket);
  await b.settle();
  b.sync.upload(false);
  await b.settle();
  assert.equal(cell.programs.get(bracket.identity.id)?.revision, 1);

  // An edit the cell has not accepted yet (inside the save debounce).
  b.edit("Bracket, edited");
  await b.dialog.open();
  b.dialog.choose(frame.identity.id, false);
  assert.deepEqual(b.dialog.guardButtons().map((x) => x.textContent), [
    "Save to example-relay, then open", "Keep as a draft in this browser", "Discard my edits", "Cancel",
  ]);
  assert.equal(b.window.document.activeElement?.textContent, "Save to example-relay, then open");
  b.dialog.guardButtons()[3]!.click();
  assert.deepEqual(b.dialog.guardButtons(), []);
  assert.equal(b.dialog.shown, true);
  assert.equal(b.state.project.identity.id, bracket.identity.id);

  // Save, then open: the edit is revision 2 on the cell before Frame opens.
  b.dialog.choose(frame.identity.id, false);
  b.dialog.guardButtons()[0]!.click();
  assert.equal(b.dialog.shown, false);
  assert.match(b.notices.at(-1)!, /^Saving Bracket, edited to example-relay, then opening Frame$/);
  await b.settle();
  assert.equal(cell.programs.get(bracket.identity.id)?.revision, 2);
  assert.equal(cell.programs.get(bracket.identity.id)?.name, "Bracket, edited");
  assert.equal(b.state.project.identity.id, frame.identity.id);

  // Discard: Frame's edit is dropped, Bracket opens at the cell's revision.
  b.edit("Frame, throwaway");
  await b.dialog.open();
  b.dialog.choose(bracket.identity.id, false);
  b.dialog.guardButtons().find((x) => x.textContent === "Discard my edits")!.click();
  await b.settle();
  assert.equal(b.state.project.identity.id, bracket.identity.id);
  assert.equal(b.state.project.identity.name, "Bracket, edited");
  assert.equal(cell.programs.get(frame.identity.id)?.revision, 1);
  assert.equal(cell.programs.get(frame.identity.id)?.name, "Frame");
  await b.dialog.open();
  assert.doesNotMatch(b.rows().find((r) => r.programId === frame.identity.id)!.text, /DRAFT HERE/);
});

test("offline: the cell's rows say why, this browser's programs open, and a kept draft reopens and is saved on reconnect", async (t) => {
  const cell = new FakeCell();
  const bracket = createBlankSpreadsheetProject({ name: "Bracket" });
  const other = createBlankSpreadsheetProject({ name: "Other local" });
  const b = await browserWith(t, cell, bracket);
  b.local.set(other.identity.id, other);
  await b.settle();
  b.sync.upload(false);
  await b.settle();

  cell.offline = true;
  cell.bump(); // the held change feed fails: the cell is gone
  await b.settle();
  b.edit("Bracket, offline edit");
  await b.dialog.open();
  assert.match(b.dialog.status(), /example-relay cannot be reached/);
  const cellRow = b.rows().find((r) => r.programId === bracket.identity.id)!;
  assert.equal(cellRow.local, false);
  assert.match(cellRow.blocked, /example-relay cannot be reached/);

  b.dialog.choose(other.identity.id, true);
  const [save, keep] = b.dialog.guardButtons();
  assert.equal(save!.disabled, true, "nothing can be saved to an unreachable cell");
  assert.match(b.dialog.dialog.panel.textContent!, /Saving to example-relay is not possible now: example-relay cannot be reached\./);
  assert.equal(b.window.document.activeElement, keep);
  keep!.click();
  await b.settle();
  assert.equal(b.state.project.identity.id, other.identity.id);
  assert.equal(b.local.get(bracket.identity.id)?.identity.name, "Bracket, offline edit", "the draft stays in this browser");

  // Back online: Bracket's row says this browser holds a draft of it; opening
  // it opens that draft, which is then saved from revision 1.
  cell.offline = false;
  await b.settle(12000);
  await b.dialog.open();
  const draftRow = b.rows().find((r) => r.programId === bracket.identity.id)!;
  assert.match(draftRow.text, /DRAFT HERE/);
  b.dialog.choose(bracket.identity.id, false);
  assert.deepEqual(b.dialog.guardButtons(), [], "the open program (browser only) loses nothing");
  await b.settle();
  assert.equal(b.state.project.identity.name, "Bracket, offline edit");
  assert.equal(cell.programs.get(bracket.identity.id)?.revision, 2);
  assert.equal(cell.programs.get(bracket.identity.id)?.name, "Bracket, offline edit");
});

test("no store: explained, and every program in this browser opens; a running robot refuses a switch with its reason", async (t) => {
  const cell = new FakeCell();
  cell.disabled = true;
  const first = createBlankSpreadsheetProject({ name: "First" });
  const second = createBlankSpreadsheetProject({ name: "Second" });
  let lock = "";
  const b = await browserWith(t, cell, first, () => lock);
  b.local.set(second.identity.id, second);
  await b.settle();
  await b.dialog.open();
  assert.match(b.dialog.status(), /keeps no cell program store/);
  const sections = [...b.dialog.dialog.panel.querySelectorAll<HTMLElement>(".dialog-section")];
  assert.equal(sections[1]!.hidden, true, "no cell section without a store");
  assert.match(sections[2]!.textContent!, /^This browser/);
  assert.doesNotMatch(sections[2]!.textContent!, /This browser only/);
  assert.deepEqual(b.rows().map((r) => r.local), [true, true]);

  lock = "A loop is running on the robot; Finish cycle or STOP it before opening another program";
  b.dialog.choose(second.identity.id, true);
  assert.equal(b.dialog.status(), lock);
  assert.equal(b.dialog.shown, true);
  assert.equal(b.state.project.identity.id, first.identity.id);

  lock = "";
  b.dialog.choose(second.identity.id, true);
  await b.settle();
  assert.equal(b.state.project.identity.id, second.identity.id);
  assert.equal(b.dialog.shown, false);
});
