import assert from "node:assert/strict";
import test from "node:test";
import { seedRobotCatalogue } from "../../../../offline-programming/v1/ui/test/support/robot-catalogue.ts";
import * as session from "./session.ts";
import * as cell from "./cellsync.ts";

seedRobotCatalogue();

const DECK = { id: "deck-1", label: "rosie-pendant-v5 (deck-b)", kind: "pendant", address: "127.0.0.1" };
const TAB = { id: "tab-1", label: "Browser (Chrome on macOS) on http://127.0.0.1:5189", kind: "browser", address: "127.0.0.1" };

function head(id: string, revision: number, name: string, by = TAB) {
  return { program_id: id, revision, name, robot_model: "rosie_1400_v3", updated_at: "2026-10-11T18:00:00Z", updated_by: by };
}

function view(programs: unknown[], current: string | null, watched?: unknown) {
  const body: Record<string, unknown> = {
    schema: "offline-programming.cell-programs.v1", enabled: true, store: { id: "store-example", instance: "i1", label: "example-relay" },
    version: 3, current: current ? { program_id: current, set_by: TAB } : null, executing: null, programs, presence: [],
  };
  if (watched !== undefined) body.watched = watched;
  return body;
}

/** A program as the browser would have saved it to the cell. */
function browserProgram(name: string): { id: string; document: unknown } {
  session.newProject(name, "rosie_1400_v3");
  session.addEvent("home");
  const document = (JSON.parse(session.exportProject()) as { project: { identity: { id: string } } }).project;
  return { id: document.identity.id, document };
}

test("the Deck opens the cell's current program, saves its edits from that revision, and keeps a conflicting edit as the current copy", () => {
  const theirs = browserProgram("Bracket from the browser");
  session.newProject("New pendant program", "rosie_1400_v3");
  cell.cellSyncStart("deck-1", null);

  let tick = cell.cellSyncTick(0);
  assert.equal(tick.view.state, "unknown");
  const feed = tick.requests.find((r) => r.kind === "feed")!;
  cell.cellSyncReply(feed.id, 200, view([head(theirs.id, 7, "Bracket from the browser")], theirs.id), 0);
  tick = cell.cellSyncTick(100);
  assert.deepEqual(tick.offer!.rows.map((r) => [r.name, r.revision, r.current]), [["Bracket from the browser", 7, true]]);
  assert.equal(tick.offer!.local!.name, "New pendant program");

  // The operator opens the cell's program from the picker.
  assert.equal(cell.cellSyncPick("open", theirs.id, 150), "Opening Bracket from the browser from example-relay");
  tick = cell.cellSyncTick(200);
  const current = tick.requests.find((r) => r.kind === "current")!;
  assert.deepEqual(current.body, { program_id: theirs.id });
  cell.cellSyncReply(current.id, 200, { ok: true }, 200);
  tick = cell.cellSyncTick(300);
  const fetch = tick.requests.find((r) => r.kind === "fetch")!;
  const opened = cell.cellSyncReply(fetch.id, 200, { program: head(theirs.id, 7, "Bracket from the browser"), document: theirs.document }, 300);
  assert.equal(opened.opened, "Opened Bracket from the browser · rev 7 from example-relay");
  assert.equal(session.projectIdentity(), theirs.id);
  assert.equal(session.view().name, "Bracket from the browser");
  assert.deepEqual(session.view().rows.map((r) => r.op), ["home"]);
  tick = cell.cellSyncTick(400);
  assert.equal(tick.view.short, "EXAMPLE-RELAY · R7");
  assert.ok(tick.links, "the link is kept for the next start");
  assert.equal(Object.values(tick.links!)[0]!.baseRevision, 7);

  // A Deck edit is saved from revision 7.
  session.setSpeedScale(50);
  cell.cellSyncTick(1000);
  tick = cell.cellSyncTick(2000);
  const save = tick.requests.find((r) => r.kind === "save")!;
  assert.equal((save.body as { base_revision: number }).base_revision, 7);
  assert.equal(((save.body as { document: { identity: { id: string } } }).document).identity.id, theirs.id);
  cell.cellSyncReply(save.id, 200, { ok: true, program: head(theirs.id, 8, "Bracket from the browser", DECK) }, 2000);
  assert.equal(cell.cellSyncTick(2100).view.label, "synced with example-relay · rev 8");

  // The browser saved revision 9 while the Deck edited 8.
  session.setSpeedScale(40);
  cell.cellSyncTick(3000);
  tick = cell.cellSyncTick(4000);
  const stale = tick.requests.find((r) => r.kind === "save")!;
  cell.cellSyncReply(stale.id, 409, { error: "program_conflict", current: head(theirs.id, 9, "Bracket from the browser") }, 4000);
  tick = cell.cellSyncTick(4100);
  assert.equal(tick.view.state, "conflict");
  assert.equal(tick.view.short, "CONFLICT");
  assert.match(tick.view.conflict!.text, /example-relay has rev 9, saved by Browser \(Chrome on macOS\)/);

  assert.match(cell.cellSyncResolve("copy", 4200), /\(copy\)" on the cell$/);
  assert.notEqual(session.projectIdentity(), theirs.id);
  tick = cell.cellSyncTick(5200);
  const copySave = tick.requests.find((r) => r.kind === "save")!;
  assert.match(copySave.path, new RegExp(`programs/${session.projectIdentity()}/save$`));
  assert.deepEqual([(copySave.body as { base_revision: number }).base_revision, (copySave.body as { make_current: boolean }).make_current], [0, true]);
});

test("the links kept in the file bring an offline draft back as a draft", () => {
  session.newProject("Teach", "rosie_1400_v3");
  const id = session.projectIdentity();
  cell.cellSyncStart("deck-1", { [`store-example:${id}`]: { storeId: "store-example", programId: id, baseRevision: 4, dirty: true, syncedKey: "old" } });
  let tick = cell.cellSyncTick(0);
  const feed = tick.requests.find((r) => r.kind === "feed")!;
  cell.cellSyncReply(feed.id, 200, view([head(id, 4, "Teach")], id, head(id, 4, "Teach")), 0);
  tick = cell.cellSyncTick(1000);
  const save = tick.requests.find((r) => r.kind === "save")!;
  assert.equal((save.body as { base_revision: number }).base_revision, 4);
});

test("the picker's actions: open refused while the robot runs another, upload, new, an empty cell", () => {
  session.newProject("Deck teach", "rosie_1400_v3");
  const local = session.projectIdentity();
  cell.cellSyncStart("deck-1", null);
  let tick = cell.cellSyncTick(0);
  let feed = tick.requests.find((r) => r.kind === "feed")!;
  const running = { program_id: "p2", program_ref: "p2", reason: "the robot has this program playing (Stop to release)" };
  cell.cellSyncReply(feed.id, 200, { ...view([head("p1", 3, "Frame"), head("p2", 7, "Bracket", DECK)], "p2"), executing: running }, 0);
  tick = cell.cellSyncTick(100);
  assert.deepEqual(tick.offer!.rows.map((r) => [r.name, r.running, r.blocked !== ""]), [["Frame", false, true], ["Bracket", true, false]]);
  assert.throws(() => cell.cellSyncPick("open", "p1", 100), /The robot is running Bracket; Stop it before opening another program/);
  assert.equal(cell.cellSyncPick("new", "", 100), "");
  assert.match(cell.cellSyncPick("upload", "", 100), /^Putting Deck teach on example-relay as its current program$/);
  tick = cell.cellSyncTick(1200);
  const save = tick.requests.find((r) => r.kind === "save")!;
  assert.match(save.path, new RegExp(`programs/${local}/save$`));
  assert.equal((save.body as { make_current: boolean }).make_current, true);

  // An empty cell: no rows, this Deck's program to upload.
  session.newProject("Other", "rosie_1400_v3");
  cell.cellSyncStart("deck-1", null);
  tick = cell.cellSyncTick(0);
  feed = tick.requests.find((r) => r.kind === "feed")!;
  cell.cellSyncReply(feed.id, 200, view([], null), 0);
  const picker = cell.cellSyncTick(100).offer!;
  assert.deepEqual(picker.rows, []);
  assert.equal(picker.local!.name, "Other");
  assert.equal(cell.cellSyncTick(1500).requests.filter((r) => r.kind === "save").length, 0, "nothing uploaded unasked");
});

test("OPEN on the Deck: who has each program open, an unsaved edit saved first or discarded, never replaced silently", () => {
  const a = browserProgram("Teach A");
  const b = browserProgram("Bracket");
  session.newProject("New pendant program", "rosie_1400_v3");
  cell.cellSyncStart("deck-1", null);
  let tick = cell.cellSyncTick(0);
  let req = tick.requests.find((r) => r.kind === "feed")!;
  const presence = [{ program_id: b.id, ...TAB, editing: true }];
  cell.cellSyncReply(req.id, 200, { ...view([head(a.id, 1, "Teach A"), head(b.id, 3, "Bracket")], a.id), presence }, 0);
  cell.cellSyncTick(50);
  // Open A (the startup question), as every test here starts.
  cell.cellSyncPick("open", a.id, 100);
  const answer = (kind: string, status: number, body: unknown, at: number) => {
    const r = cell.cellSyncTick(at).requests.find((x) => x.kind === kind);
    assert.ok(r, `a ${kind} request at ${at}`);
    return { request: r, reply: cell.cellSyncReply(r.id, status, body, at) };
  };
  answer("current", 200, { ok: true }, 200);
  answer("fetch", 200, { program: head(a.id, 1, "Teach A"), document: a.document }, 300);
  assert.equal(session.projectIdentity(), a.id);

  const rows = cell.cellSyncPicker(400).rows;
  assert.deepEqual(rows.map((r) => [r.name, r.here, r.openBy, r.draft]),
    [["Teach A", true, "", false], ["Bracket", false, "Browser (Chrome on macOS) editing", false]]);
  assert.equal(cell.cellSyncOpenCheck(a.id, 400).here, true);
  assert.equal(cell.cellSyncOpenCheck(b.id, 400).draft, null, "nothing unsaved: Bracket opens at once");

  // An edit the cell has not accepted yet.
  session.setSpeedScale(50);
  cell.cellSyncTick(1000);
  const check = cell.cellSyncOpenCheck(b.id, 1100);
  assert.equal(check.blocked, "");
  assert.deepEqual([check.draft!.keptHere, check.draft!.saveRefused], [false, ""]);
  assert.equal(check.draft!.text, `"Teach A" has edits example-relay has not saved (rev 1 + local edits). Opening another program replaces them.`);
  assert.throws(() => cell.cellSyncOpenAfter(b.id, "open", 1100), /Save, discard or keep it first/);
  assert.equal(session.projectIdentity(), a.id);

  // Save, then open: the save goes first; Bracket only after the cell accepted it.
  assert.equal(cell.cellSyncOpenAfter(b.id, "save", 1200), "Saving Teach A to example-relay, then opening Bracket");
  tick = cell.cellSyncTick(1250);
  assert.deepEqual(tick.requests.map((r) => r.kind), ["save"]);
  req = tick.requests[0]!;
  assert.equal((req.body as { base_revision: number }).base_revision, 1);
  cell.cellSyncReply(req.id, 200, { ok: true, program: head(a.id, 2, "Teach A", DECK) }, 1300);
  const current = answer("current", 200, { ok: true }, 1400);
  assert.deepEqual(current.request.body, { program_id: b.id });
  const opened = answer("fetch", 200, { program: head(b.id, 3, "Bracket"), document: b.document }, 1500);
  assert.equal(opened.reply.opened, "Opened Bracket · rev 3 from example-relay");
  assert.equal(session.projectIdentity(), b.id);

  // Discard: Bracket's edit is dropped, nothing is saved, Teach A opens.
  session.setSpeedScale(30);
  cell.cellSyncTick(2000);
  assert.match(cell.cellSyncOpenAfter(a.id, "discard", 2100), /^Opening Teach A from example-relay; Bracket's edits discarded$/);
  tick = cell.cellSyncTick(2200);
  assert.deepEqual(tick.requests.map((r) => r.kind), ["current"]);
  cell.cellSyncReply(tick.requests[0]!.id, 200, { ok: true }, 2200);
  answer("fetch", 200, { program: head(a.id, 2, "Teach A", DECK), document: a.document }, 2300);
  assert.equal(session.projectIdentity(), a.id);
  assert.equal(cell.cellSyncTick(2400).view.label, "synced with example-relay · rev 2");

  // A save the cell refuses on the way opens nothing and keeps the edit.
  session.setSpeedScale(60);
  cell.cellSyncTick(3000);
  cell.cellSyncOpenAfter(b.id, "save", 3100);
  answer("save", 409, { error: "program_conflict", current: head(a.id, 5, "Teach A", TAB) }, 3200);
  tick = cell.cellSyncTick(3300);
  assert.deepEqual(tick.requests.filter((r) => r.kind !== "feed" && r.kind !== "presence"), []);
  assert.ok(tick.notices.includes("Bracket was not opened: example-relay has a newer revision (conflict). The edits are kept here."));
  assert.equal(tick.view.state, "conflict");
  assert.equal(session.projectIdentity(), a.id);
  assert.match(cell.cellSyncOpenCheck(b.id, 3300).draft!.saveRefused, /conflict/);
});

test("OPEN is refused while the robot holds a program: loaded, playing, looping or the Run press in flight", () => {
  assert.equal(cell.cellSyncRunLock({ controls: { state: "loaded" }, session: { id: "s1" }, looping: false }, false),
    "The robot has a program loaded; STOP it before opening another program");
  assert.equal(cell.cellSyncRunLock({ controls: { state: "playing" }, session: { id: "s1" } }, false),
    "The robot has a program playing; STOP it before opening another program");
  assert.equal(cell.cellSyncRunLock({ controls: { state: "done" }, session: { id: "s1" }, looping: true }, false),
    "A loop is running on the robot; Finish cycle or STOP it before opening another program");
  assert.equal(cell.cellSyncRunLock({ controls: { state: "unknown" }, session: null }, true),
    "Not while the Run program press is still running; STOP it before opening another program");
  assert.equal(cell.cellSyncRunLock({ controls: { state: "done" }, session: { id: "s1" }, looping: false }, false), "");
  assert.equal(cell.cellSyncRunLock(null, false), "");

  const b = browserProgram("Bracket");
  session.newProject("Teach", "rosie_1400_v3");
  cell.cellSyncStart("deck-1", null);
  const feed = cell.cellSyncTick(0).requests.find((r) => r.kind === "feed")!;
  cell.cellSyncReply(feed.id, 200, view([head(b.id, 3, "Bracket")], null), 0);
  const lock = cell.cellSyncRunLock({ controls: { state: "loaded" }, session: { id: "s1" } }, false);
  assert.deepEqual(cell.cellSyncPicker(100, lock).rows.map((r) => r.blocked), [lock]);
  assert.equal(cell.cellSyncOpenCheck(b.id, 100, lock).blocked, lock);
  assert.throws(() => cell.cellSyncOpenAfter(b.id, "discard", 100, lock), new RegExp(lock));
  assert.deepEqual(cell.cellSyncTick(200).requests.filter((r) => r.kind === "current" || r.kind === "fetch"), []);
});
