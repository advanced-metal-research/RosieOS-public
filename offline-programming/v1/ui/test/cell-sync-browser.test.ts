import assert from "node:assert/strict";
import test from "node:test";
import { setImmediate } from "node:timers/promises";

import { createBlankSpreadsheetProject } from "../src/domain/spreadsheet.ts";
import { reviseProject, type OfflineProgrammingProject } from "../src/domain/project.ts";
import { BrowserCellSync } from "../src/storage/cell-sync-browser.ts";

import { DECK, FakeCell, TAB } from "./support/fake-cell.ts";

async function withDocument(t: test.TestContext) {
  const { Window } = await import("happy-dom");
  const browser = new Window();
  browser.document.body.innerHTML = `<span id="cell-sync" hidden></span>`;
  for (const [name, value] of [["document", browser.document], ["HTMLElement", browser.HTMLElement]] as const) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  return browser.document as unknown as Document;
}

function rename(project: OfflineProgrammingProject, name: string): OfflineProgrammingProject {
  return reviseProject(project, "metadata", (draft) => { draft.identity.name = name; });
}

test("the browser uploads a local program, follows the Deck's edit and offers the copy on a conflict", async (t) => {
  const document = await withDocument(t);
  const cell = new FakeCell();
  let now = 1_000;
  let project = createBlankSpreadsheetProject({ name: "Bracket" });
  const opened: string[] = [];
  const sync = new BrowserCellSync({
    clientId: "tab-1", project: () => project,
    open: (next, message) => { project = next; opened.push(message); },
    notice: () => undefined, changed: () => undefined,
  }, { fetch: cell.fetch, storage: null, clock: () => now });
  t.after(() => sync.stop());
  const settle = async (advance = 1000) => {
    for (let i = 0; i < 6; i++) {
      now += advance / 6;
      sync.tick(now);
      await setImmediate();
      await setImmediate();
    }
  };
  const pill = () => document.querySelector<HTMLElement>("#cell-sync")!;
  const banner = () => document.querySelector<HTMLElement>("#cell-sync-banner")!;
  const buttons = () => [...banner().querySelectorAll("button")].map((b) => b.textContent);

  await settle();
  assert.match(pill().textContent!, /^local only · not on example-relay/);
  assert.equal(banner().hidden, false);
  assert.deepEqual(buttons(), ['Upload "Bracket" to example-relay', "Not now"]);
  assert.match(banner().textContent!, /No saved programs on example-relay yet\./);

  ([...banner().querySelectorAll("button")].find((b) => b.textContent!.startsWith("Upload")) as HTMLButtonElement).click();
  await settle();
  assert.equal(cell.programs.get(project.identity.id)?.revision, 1);
  assert.match(pill().textContent!, /^synced with example-relay · rev 1 · steamdeck-operator editing$/);
  assert.equal(pill().dataset.state, "synced");
  assert.equal(banner().hidden, true);

  // The Deck saves revision 2; the browser, with nothing unsaved, opens it.
  cell.write(project.identity.id, 1, rename(project, "Bracket from the Deck"), DECK);
  await settle();
  assert.equal(project.identity.name, "Bracket from the Deck");
  assert.match(opened.at(-1)!, /rev 2 from example-relay \(saved by steamdeck-operator\)/);
  assert.match(pill().textContent!, /rev 2/);

  // Both edit revision 2; the Deck's lands first.
  project = rename(project, "Bracket, browser edit");
  cell.write(project.identity.id, 2, rename(project, "Bracket, Deck edit"), DECK);
  await settle(2000);
  assert.equal(pill().dataset.state, "conflict");
  assert.match(pill().textContent!, /conflict · example-relay has rev 3, saved by steamdeck-operator/);
  assert.deepEqual(buttons(), ["Reload theirs (rev 3)", "Keep mine as a copy"]);
  assert.equal((cell.programs.get(project.identity.id)!.document as OfflineProgrammingProject).identity.name, "Bracket, Deck edit",
    "the cell's revision was not overwritten");

  const original = project.identity.id;
  ([...banner().querySelectorAll("button")][1] as HTMLButtonElement).click();
  await settle(2000);
  assert.notEqual(project.identity.id, original);
  assert.equal(project.identity.name, "Bracket, browser edit (copy)");
  assert.equal(cell.programs.get(project.identity.id)?.revision, 1);
  assert.equal(pill().dataset.state, "synced");
});

test("a browser-only program is offered the cell's saved programs: newest first, markers, a refusal with its reason", async (t) => {
  const document = await withDocument(t);
  const cell = new FakeCell();
  const bracket = createBlankSpreadsheetProject({ name: "Bracket" });
  const frame = createBlankSpreadsheetProject({ name: "Frame" });
  cell.write(frame.identity.id, 0, frame, TAB);
  cell.write(bracket.identity.id, 0, bracket, DECK);
  cell.current = bracket.identity.id;
  const updated = new Map([[frame.identity.id, "2026-10-11T17:00:00Z"], [bracket.identity.id, "2026-10-11T17:57:00Z"]]);
  const view = cell.view.bind(cell);
  let running: string | null = bracket.identity.id;
  cell.view = (watch: string) => {
    const body = view(watch) as { programs: { program_id: string; updated_at: string }[]; executing: unknown };
    for (const p of body.programs) p.updated_at = updated.get(p.program_id)!;
    body.executing = running ? { program_id: running, program_ref: running, reason: "playing" } : null;
    return body;
  };
  let now = Date.parse("2026-10-11T18:00:00Z");
  let project = createBlankSpreadsheetProject({ name: "Browser draft" });
  let created = 0;
  const sync = new BrowserCellSync({
    clientId: "tab-1", project: () => project, open: (next) => { project = next; },
    notice: () => undefined, changed: () => undefined, newProgram: () => { created++; },
  }, { fetch: cell.fetch, storage: null, clock: () => now });
  t.after(() => sync.stop());
  const settle = async () => {
    for (let i = 0; i < 6; i++) {
      now += 300;
      sync.tick(now);
      await setImmediate();
      await setImmediate();
    }
  };
  await settle();
  const banner = document.querySelector<HTMLElement>("#cell-sync-banner")!;
  const rows = [...banner.querySelectorAll<HTMLButtonElement>(".cell-sync-picker button")];
  assert.deepEqual(rows.map((b) => b.textContent), [
    "★ Bracket · rosie_1400_v3 · rev 1 · saved 3 min ago by steamdeck-operator · CURRENT · ON THE ROBOT",
    "Frame · rosie_1400_v3 · rev 1 · saved 1 h ago by Browser (Chrome on macOS)",
  ]);
  assert.equal(rows[0]!.disabled, false);
  assert.equal(rows[1]!.disabled, true, "the cell refuses to switch away from the program the robot runs");
  assert.match(banner.textContent!, /The robot is running Bracket; Stop it before opening another program/);
  const actions = [...banner.querySelectorAll<HTMLButtonElement>(".cell-sync-picker-actions button")].map((b) => b.textContent);
  assert.deepEqual(actions, ['Upload "Browser draft" to example-relay', "New program", "Not now"]);
  [...banner.querySelectorAll<HTMLButtonElement>(".cell-sync-picker-actions button")][1]!.click();
  assert.equal(created, 1);

  // The robot stops: Frame opens, and becomes the cell's current program.
  running = null;
  cell.bump(); // the server wakes the feed when the robot's session ends
  await settle();
  const frameRow = [...banner.querySelectorAll<HTMLButtonElement>(".cell-sync-picker button")].find((b) => b.dataset.programId === frame.identity.id)!;
  assert.equal(frameRow.disabled, false);
  frameRow.click();
  await settle();
  assert.equal(project.identity.id, frame.identity.id);
  assert.equal(cell.current, frame.identity.id);
  assert.equal(banner.hidden, true);
});
