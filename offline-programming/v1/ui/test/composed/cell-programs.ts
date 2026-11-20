// Two real clients editing one program through a real OLP server's cell
// program store: the browser's BrowserCellSync over fetch and the Steam
// Deck's olp-core cellsync driven the way src/program_sync.cpp drives it.
// Run by offline-programming/v1 cell_programs_composed_test.go, which serves
// the OLP and restarts it between the phases.
// usage: node --experimental-strip-types --import ./test/support/seed.ts \
//          test/composed/cell-programs.ts <edit|restart> <origin> <state.json>
import assert from "node:assert/strict";
import { readFileSync, writeFileSync } from "node:fs";
import { setTimeout as sleep } from "node:timers/promises";

import { createBlankSpreadsheetProject } from "../../src/domain/spreadsheet.ts";
import { reviseProject, type OfflineProgrammingProject } from "../../src/domain/project.ts";
import { BrowserCellSync } from "../../src/storage/cell-sync-browser.ts";
import * as session from "../../../../../steamdeck/real/v5/olp-core/session.ts";
import * as deckSync from "../../../../../steamdeck/real/v5/olp-core/cellsync.ts";

const [phase, origin, statePath] = process.argv.slice(2);
assert.ok(phase && origin && statePath, "usage: <edit|restart> <origin> <state.json>");
const API = "/api/offline-programming/v1/cell-programs";
const DECK_HEADERS = { "X-Rosie-Client": "rosie-pendant-v5 (steamdeck-composed)", "X-Rosie-Client-Id": "deck-composed01" };
const TAB_HEADERS = {
  "X-Rosie-Client-Id": "tab-composed01", Origin: "http://127.0.0.1:5189",
  "User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 14_0) AppleWebKit/537.36 Chrome/130.0 Safari/537.36",
};

const { Window } = await import("happy-dom");
const browserWindow = new Window();
browserWindow.document.body.innerHTML = `<span id="cell-sync" hidden></span>`;
Object.defineProperty(globalThis, "document", { configurable: true, value: browserWindow.document });

async function get(path: string): Promise<any> {
  const response = await fetch(origin + API + path);
  return response.json();
}

async function until(what: string, check: () => boolean | Promise<boolean>, ms = 15000): Promise<void> {
  const deadline = Date.now() + ms;
  while (!(await check())) {
    if (Date.now() > deadline) throw new Error(`timed out waiting for ${what}`);
    await sleep(25);
  }
}

/** The Deck: ProgramSync's loop, over fetch. */
class Deck {
  running = true;
  links: unknown = null;
  offers: deckSync.CellSyncTick["offer"][] = [];
  inflight = 0;
  events: string[] = [];
  constructor(kept: unknown) {
    deckSync.cellSyncStart("deck-composed01", kept);
    void this.loop();
  }
  private async loop(): Promise<void> {
    while (this.running) {
      const tick = deckSync.cellSyncTick(Date.now());
      if (tick.links) this.links = tick.links;
      if (tick.offer) this.offers.push(tick.offer);
      for (const request of tick.requests) void this.send(request);
      await sleep(50);
    }
  }
  private async send(request: deckSync.CellSyncTick["requests"][number]): Promise<void> {
    this.inflight++;
    this.events.push(`send ${request.id} ${request.kind}`);
    let status = 0;
    let body: unknown = null;
    let failure = "";
    try {
      const response = await fetch(origin + request.path, {
        method: request.method, signal: AbortSignal.timeout(request.timeoutMs),
        headers: { ...DECK_HEADERS, "Content-Type": "application/json" },
        body: request.body === undefined ? undefined : JSON.stringify(request.body),
      });
      status = response.status;
      body = await response.json().catch(() => null);
    } catch (error) {
      status = 0;
      failure = error instanceof Error ? error.message : String(error);
    }
    this.inflight--;
    this.events.push(`reply ${request.id} ${request.kind} ${status}${failure ? ` ${failure}` : ""}`);
    if (this.events.length > 20) this.events.splice(0, this.events.length - 20);
    if (!this.running) return;
    const result = deckSync.cellSyncReply(request.id, status, body, Date.now());
    if (result.error) throw new Error(result.error);
  }
  view() {
    return deckSync.cellSyncView(Date.now());
  }
}

async function untilDeck(what: string, deck: Deck, check: () => boolean, ms = 15000): Promise<void> {
  try {
    await until(what, check, ms);
  } catch (error) {
    const view = deck.view();
    const cell = await get("");
    console.error(`${what} sync state`, JSON.stringify({
      deck: { state: view.state, programId: view.programId, revision: view.revision,
        isCurrent: view.isCurrent, current: view.current, dirty: view.dirty, conflict: view.conflict,
        inflight: deck.inflight, events: deck.events },
      cell: { current: cell.current, programs: cell.programs.map((program: { program_id: string; revision: number }) =>
        ({ id: program.program_id, revision: program.revision })) },
    }));
    throw error;
  }
}

function browser(initial: OfflineProgrammingProject) {
  const state = { project: initial };
  const sync = new BrowserCellSync({
    clientId: "tab-composed01", project: () => state.project,
    open: (next) => { state.project = next; }, notice: () => undefined, changed: () => undefined,
  }, {
    storage: null,
    fetch: (input, init) => fetch(origin + String(input), { ...init, headers: { ...(init?.headers as Record<string, string>), ...TAB_HEADERS } }),
  });
  sync.start();
  return { state, sync };
}

if (phase === "edit") {
  // 1. The Deck starts on an empty cell: the picker offers its program and a
  //    new one; the operator uploads it, and it becomes current.
  session.newProject("Deck teach", "rosie_1400_v3");
  const deckProgram = session.projectIdentity();
  const deck = new Deck(null);
  await until("the Deck's picker", () => deck.offers.length > 0);
  assert.deepEqual(deck.offers[0]!.rows, []);
  assert.equal(deck.offers[0]!.local?.name, "Deck teach");
  deckSync.cellSyncPick("upload", "", Date.now());
  await untilDeck("the Deck's program on the cell", deck, () => deck.view().state === "synced");
  let list = await get("");
  assert.equal(list.current.program_id, deckProgram);
  assert.equal(list.programs[0].revision, 1);
  assert.match(list.programs[0].updated_by.label, /steamdeck-composed/);

  // 2. A browser with its own local draft opens the cell's current program.
  const tab = browser(createBlankSpreadsheetProject({ name: "Browser draft" }));
  await until("the browser's upload offer", () => tab.sync.view().offerUpload);
  assert.equal(tab.sync.view().current?.programId, deckProgram);
  tab.sync.openCell(deckProgram);
  await until("the browser on the Deck's program", () => tab.state.project.identity.id === deckProgram && tab.sync.view().state === "synced");
  assert.equal(tab.state.project.identity.name, "Deck teach");
  await until("presence both ways", () => /steamdeck-composed/.test(tab.sync.view().editorsText) && /Browser \(Chrome on macOS\)/.test(deck.view().editorsText));

  // 3. The browser renames it; the Deck, with nothing unsaved, follows.
  tab.state.project = reviseProject(tab.state.project, "metadata", (d) => { d.identity.name = "Bracket (browser)"; });
  await until("the Deck following the browser", () => session.view().name === "Bracket (browser)");
  assert.equal(deck.view().revision, 2);

  // 4. The Deck changes the plan speed; the browser follows.
  session.setSpeedScale(50);
  await until("the browser following the Deck", () => tab.state.project.programV2.speedScale === 0.5);
  assert.equal(tab.sync.view().revision, 3);

  // 5. Both edit revision 3. The browser's lands first; the Deck's is a
  //    conflict and the cell keeps the browser's.
  // The Deck is stopped (its held feed answer is dropped when it comes) while
  // the operator edits on it, as a Deck that went off the network would.
  deck.running = false;
  session.setSpeedScale(25);
  tab.state.project = reviseProject(tab.state.project, "metadata", (d) => { d.identity.name = "Bracket (browser, second)"; });
  await until("the browser's revision 4", () => tab.sync.view().state === "synced" && tab.sync.view().revision === 4);
  const deck2 = new Deck(deck.links);
  await until("the Deck's conflict", () => deck2.view().state === "conflict");
  assert.match(deck2.view().conflict!.text, /example-relay has rev 4, saved by Browser \(Chrome on macOS\)/);
  const stored = await get(`/programs/${deckProgram}`);
  assert.equal(stored.program.revision, 4);
  assert.equal(stored.document.identity.name, "Bracket (browser, second)");
  assert.equal(stored.document.programV2.speedScale, 0.5, "the Deck's stale edit did not reach the cell");

  // 6. The Deck keeps its edit as a copy, which becomes the cell's current program.
  deckSync.cellSyncResolve("copy", Date.now());
  const copy = session.projectIdentity();
  assert.notEqual(copy, deckProgram);
  await untilDeck("the copy on the cell", deck2, () => deck2.view().state === "synced" && deck2.view().isCurrent);
  list = await get("");
  assert.equal(list.current.program_id, copy);
  assert.equal(list.programs.length, 2);
  // The browser stays on the program it opened, synced at rev 4.
  await sleep(300);
  assert.equal(tab.state.project.identity.id, deckProgram);
  assert.equal(tab.sync.view().label, "synced with example-relay · rev 4");

  deck2.running = false;
  tab.sync.stop();
  writeFileSync(statePath, JSON.stringify({ original: deckProgram, copy, links: deck2.links, deckExport: session.exportProject() }));
  console.log(`edit phase ok: ${deckProgram} rev 4 (browser), copy ${copy} current`);
} else if (phase === "restart") {
  // A restarted OLP and a restarted pendant: everything is still there, and
  // the pendant's kept links make its program synced, not a new upload.
  const state = JSON.parse(readFileSync(statePath, "utf8"));
  const list = await get("");
  assert.equal(list.current.program_id, state.copy);
  assert.deepEqual(list.programs.map((p: { program_id: string }) => p.program_id).sort(), [state.original, state.copy].sort());
  const original = await get(`/programs/${state.original}`);
  assert.equal(original.program.revision, 4);
  assert.equal(original.document.identity.name, "Bracket (browser, second)");
  session.openProject(state.deckExport);
  const deck = new Deck(state.links);
  await until("the restarted Deck synced", () => deck.view().state === "synced");
  assert.equal(deck.view().revision, 1);
  assert.equal(deck.view().isCurrent, true);
  const revisions = await get(`/programs/${state.copy}/revisions`);
  assert.deepEqual(revisions.revisions, [1], "a restart is not an edit");
  deck.running = false;
  console.log("restart phase ok");
} else {
  throw new Error(`unknown phase ${phase}`);
}
process.exit(0);
