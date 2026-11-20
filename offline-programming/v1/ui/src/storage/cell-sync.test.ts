import assert from "node:assert/strict";
import test from "node:test";
import {
  adoptRemoteProject, CellProgramSync, clientName, copyProject, MemoryLinks, type CellDocument, type CellRequest,
} from "./cell-sync.ts";
import type { OfflineProgrammingProject } from "../domain/project.ts";

const DECK = "rosie-pendant-v5 (steamdeck-operator)";
const TAB = "Browser (Chrome on macOS) on http://127.0.0.1:5189";

function head(id: string, revision: number, by = DECK, extra: Record<string, unknown> = {}) {
  return { program_id: id, revision, name: `Program ${id}`, robot_model: "rosie_1430_v1", updated_at: "2026-10-11T18:00:00Z",
    updated_by: { id: "deck-1", label: by, kind: "pendant", address: "127.0.0.1" }, ...extra };
}

function storeView(opts: { version?: number; programs?: unknown[]; current?: string | null; presence?: unknown[]; executing?: unknown; watched?: unknown } = {}) {
  const body: Record<string, unknown> = {
    schema: "offline-programming.cell-programs.v1", enabled: true,
    store: { id: "store-rosie_1420_v1", instance: "i1", label: "example-relay", host: "example-relay" },
    version: opts.version ?? 2, current: opts.current ? { program_id: opts.current, set_by: { label: DECK } } : null,
    executing: opts.executing ?? null, programs: opts.programs ?? [], presence: opts.presence ?? [],
  };
  if ("watched" in opts) body.watched = opts.watched;
  return body;
}

function doc(id: string, key: string, name = "Bracket"): CellDocument {
  return { programId: id, name, key, document: { identity: { id, name }, key } };
}

/** Run every request the sync wants now, answering each with `answer`. */
function step(sync: CellProgramSync, now: number, answer: (r: CellRequest) => [number, unknown] | null): CellRequest[] {
  const sent = sync.requests(now);
  for (const r of sent) {
    const reply = answer(r);
    if (reply) sync.reply(r.id, reply[0], reply[1], now);
  }
  return sent;
}

function connected(links = new MemoryLinks(), opts: { followCurrent?: boolean } = {}, view = storeView()) {
  const sync = new CellProgramSync(links, { clientTag: "tab-1", followCurrent: opts.followCurrent, debounceMs: 100 });
  return { sync, links, view };
}

test("a server without a store leaves the program local only", () => {
  const { sync } = connected();
  sync.observe(doc("p1", "k1"), 0);
  assert.equal(sync.view(0).state, "unknown");
  step(sync, 0, () => [503, { error: "program_store_disabled" }]);
  const view = sync.view(0);
  assert.equal(view.state, "disabled");
  assert.equal(view.label, "local only");
  assert.equal(view.short, "LOCAL");
  // Asked again a minute later, not hammered.
  assert.deepEqual(sync.requests(1000), []);
  assert.equal(sync.requests(61_000).length, 1);
});

test("an existing local program is offered for upload, then synced with a revision", () => {
  const { sync, links } = connected();
  sync.observe(doc("p1", "k1"), 0);
  step(sync, 0, (r) => r.kind === "feed" ? [200, storeView()] : null);
  let view = sync.view(0);
  assert.equal(view.state, "local");
  assert.equal(view.offerUpload, true);
  assert.match(view.label, /not on example-relay/);

  sync.upload(false, 0);
  const sent = step(sync, 200, (r) => r.kind === "save" ? [200, { ok: true, program: head("p1", 1, TAB) }] : null);
  const save = sent.find((r) => r.kind === "save")!;
  assert.equal((save.body as { base_revision: number }).base_revision, 0);
  assert.equal(save.path, "/api/offline-programming/v1/cell-programs/programs/p1/save");
  view = sync.view(300);
  assert.equal(view.state, "synced");
  assert.equal(view.label, "synced with example-relay · rev 1");
  assert.equal(view.short, "EXAMPLE-RELAY · R1");
  assert.deepEqual(links.get("store-rosie_1420_v1", "p1"), { storeId: "store-rosie_1420_v1", programId: "p1", baseRevision: 1, dirty: false, syncedKey: "k1" });
});

test("a feed sent before save cannot erase the acknowledged revision", () => {
  const { sync, links } = connected();
  sync.observe(doc("p1", "k1"), 0);
  step(sync, 0, (r) => r.kind === "feed" ? [200, storeView({ version: 2 })] : null);
  sync.upload(false, 0);
  const save = sync.requests(200).find((r) => r.kind === "save")!;
  const feed = sync.requests(300).find((r) => r.kind === "feed")!;
  assert.ok(save && feed);

  sync.reply(save.id, 200, { ok: true, program: head("p1", 1, TAB), version: 3 }, 301);
  sync.reply(feed.id, 200, storeView({ version: 2, watched: null }), 302);

  assert.equal(sync.view(302).state, "synced");
  assert.equal(links.get("store-rosie_1420_v1", "p1")?.baseRevision, 1);

  const restartFeed = sync.requests(600).find((r) => r.kind === "feed")!;
  const restarted = storeView({ version: 1, programs: [head("p1", 1, TAB)], watched: head("p1", 1, TAB) });
  (restarted.store as Record<string, unknown>).instance = "i2";
  sync.reply(restartFeed.id, 200, restarted, 601);
  assert.equal(sync.store?.instance, "i2", "a restarted store may start at a lower version");
  assert.equal(sync.view(601).state, "synced");
});

/** The Deck after "Keep mine as a copy" (composed test step 6): its stale
 *  edit of p1 was a conflict, and the copy p2 has just been handed to resolveCopy. */
function deckAfterCopy() {
  const links = new MemoryLinks();
  links.put({ storeId: "store-rosie_1420_v1", programId: "p1", baseRevision: 3, dirty: true, syncedKey: "k3" });
  const { sync } = connected(links, { followCurrent: true });
  sync.observe(doc("p1", "mine"), 0);
  const p1 = head("p1", 4, TAB);
  step(sync, 0, (r) => r.kind === "feed" ? [200, storeView({ version: 2, current: "p1", programs: [p1], watched: p1 })] : null);
  assert.equal(sync.view(0).conflict?.kind, "changed");
  sync.resolveCopy(doc("p2", "copy", "Bracket (copy)"), true, 10);
  return { sync, links, p1 };
}

test("a presence answer that crosses the copy's first save does not report the copy deleted", () => {
  // CI run 42934666562, the Deck's own trace: send presence, send save,
  // reply save, reply presence. The presence answer is the store before the
  // save: no p2, current still p1.
  const { sync, links, p1 } = deckAfterCopy();
  const sent = sync.requests(10);
  assert.deepEqual(sent.map((r) => r.kind), ["presence", "save"], "one tick sends both, as on the Deck");
  const [presence, save] = sent as [CellRequest, CellRequest];
  assert.equal((save.body as { base_revision: number; make_current: boolean }).base_revision, 0);

  sync.reply(save.id, 200, { ok: true, program: head("p2", 1, DECK), version: 4 }, 20);
  sync.reply(presence.id, 200, storeView({ version: 3, current: "p1", programs: [p1], watched: null }), 21);

  const view = sync.view(21);
  assert.equal(view.conflict, null, "the copy the cell just accepted is not deleted");
  assert.equal(view.state, "synced");
  assert.equal(view.programId, "p2");
  assert.equal(view.revision, 1);
  assert.equal(links.get("store-rosie_1420_v1", "p2")?.baseRevision, 1);
  assert.equal(sync.requests(30).some((r) => r.kind === "fetch"), false, "the Deck does not reopen p1 as the current program");

  // The next feed sees the cell after the save: the copy is current here.
  const feed = sync.requests(300).find((r) => r.kind === "feed")!;
  const p2 = head("p2", 1, DECK);
  sync.reply(feed.id, 200, storeView({ version: 4, current: "p2", programs: [p2, p1], watched: p2 }), 301);
  assert.equal(sync.view(301).state, "synced");
  assert.equal(sync.view(301).isCurrent, true);
});

test("after a crossed answer is ignored, a real delete or another client's save still surfaces", () => {
  const deleted = deckAfterCopy();
  let [presence, save] = deleted.sync.requests(10) as [CellRequest, CellRequest];
  deleted.sync.reply(save.id, 200, { ok: true, program: head("p2", 1, DECK), version: 4 }, 20);
  deleted.sync.reply(presence.id, 200, storeView({ version: 3, current: "p1", programs: [deleted.p1], watched: null }), 21);
  let feed = deleted.sync.requests(300).find((r) => r.kind === "feed")!;
  const tombstone = head("p2", 2, TAB, { deleted: true });
  deleted.sync.reply(feed.id, 200, storeView({ version: 5, current: "p1", programs: [tombstone, deleted.p1], watched: tombstone }), 301);
  assert.equal(deleted.sync.view(301).conflict?.kind, "deleted", "a tombstone newer than the save is a real delete");
  assert.match(deleted.sync.view(301).conflict!.text, /no longer has this program \(deleted by Browser \(Chrome on macOS\)\)/);

  const edited = deckAfterCopy();
  [presence, save] = edited.sync.requests(10) as [CellRequest, CellRequest];
  edited.sync.reply(save.id, 200, { ok: true, program: head("p2", 1, DECK), version: 4 }, 20);
  edited.sync.reply(presence.id, 200, storeView({ version: 3, current: "p1", programs: [edited.p1], watched: null }), 21);
  edited.sync.observe(doc("p2", "copy-edited", "Bracket (copy)"), 250);
  feed = edited.sync.requests(300).find((r) => r.kind === "feed")!;
  assert.equal(edited.sync.requests(300).some((r) => r.kind === "save"), false, "the edit is still settling");
  const theirs = head("p2", 2, TAB);
  edited.sync.reply(feed.id, 200, storeView({ version: 5, current: "p2", programs: [theirs, edited.p1], watched: theirs }), 301);
  const view = edited.sync.view(301);
  assert.equal(view.conflict?.kind, "changed", "another client's save over this edit's base is a real conflict");
  assert.match(view.conflict!.text, /has rev 2, saved by Browser \(Chrome on macOS\)/);
  assert.equal(edited.links.get("store-rosie_1420_v1", "p2")?.dirty, true, "the local edit is kept");
});

test("edits are saved from the revision they were made on, after the debounce", () => {
  const links = new MemoryLinks();
  links.put({ storeId: "store-rosie_1420_v1", programId: "p1", baseRevision: 4, dirty: false, syncedKey: "k1" });
  const { sync } = connected(links);
  sync.observe(doc("p1", "k1"), 0);
  step(sync, 0, () => [200, storeView({ programs: [head("p1", 4)], watched: head("p1", 4) })]);
  assert.equal(sync.view(0).state, "synced");
  sync.observe(doc("p1", "k2"), 1000);
  assert.equal(sync.view(1000).state, "saving");
  assert.equal(links.get("store-rosie_1420_v1", "p1")!.dirty, true, "the draft survives a restart");
  assert.equal(sync.requests(1050).filter((r) => r.kind === "save").length, 0, "not before the edit settles");
  const sent = step(sync, 1200, (r) => r.kind === "save" ? [200, { ok: true, program: head("p1", 5, TAB) }] : null);
  assert.equal((sent.find((r) => r.kind === "save")!.body as { base_revision: number }).base_revision, 4);
  assert.equal(sync.view(1300).label, "synced with example-relay · rev 5");
  // Selecting a row does not change the key: nothing to save.
  sync.observe(doc("p1", "k2"), 1400);
  assert.equal(sync.requests(2000).filter((r) => r.kind === "save").length, 0);
});

test("a stale save is a conflict: nothing is overwritten and the operator chooses", () => {
  const links = new MemoryLinks();
  links.put({ storeId: "store-rosie_1420_v1", programId: "p1", baseRevision: 4, dirty: false, syncedKey: "k1" });
  const { sync } = connected(links);
  sync.observe(doc("p1", "k1"), 0);
  step(sync, 0, () => [200, storeView({ programs: [head("p1", 4)], watched: head("p1", 4) })]);
  sync.observe(doc("p1", "mine"), 1000);
  step(sync, 1200, (r) => r.kind === "save"
    ? [409, { error: "program_conflict", detail: "the cell has revision 5", current: head("p1", 5) }] : null);
  let view = sync.view(1300);
  assert.equal(view.state, "conflict");
  assert.equal(view.short, "CONFLICT");
  assert.match(view.label, /example-relay has rev 5, saved by steamdeck-operator/);
  assert.match(view.conflict!.text, /Reload theirs, or keep yours as a copy/);
  // No further save is attempted while the conflict stands, however long.
  assert.equal(sync.requests(60_000).filter((r) => r.kind === "save").length, 0);

  // Reload theirs: the edit is dropped and revision 5 is fetched and handed over.
  sync.resolveTheirs();
  step(sync, 61_000, (r) => r.kind === "fetch" ? [200, { program: head("p1", 5), document: { theirs: true } }] : null);
  const remote = sync.takeRemote();
  assert.deepEqual(remote!.document, { theirs: true });
  sync.applied(remote!.head, doc("p1", "theirs"));
  view = sync.view(61_100);
  assert.equal(view.state, "synced");
  assert.equal(view.revision, 5);
  assert.equal(links.get("store-rosie_1420_v1", "p1")!.dirty, false);
});

test("keep mine as a copy uploads the local edit as a new program", () => {
  const links = new MemoryLinks();
  links.put({ storeId: "store-rosie_1420_v1", programId: "p1", baseRevision: 4, dirty: true, syncedKey: "k1" });
  const { sync } = connected(links);
  sync.observe(doc("p1", "mine"), 0);
  // The cell moved on while this client was offline with an edit.
  step(sync, 0, () => [200, storeView({ programs: [head("p1", 6)], watched: head("p1", 6) })]);
  assert.equal(sync.view(0).state, "conflict");
  sync.resolveCopy(doc("p1-copy", "mine-copy", "Bracket (copy)"), false, 100);
  const sent = step(sync, 300, (r) => r.kind === "save" ? [200, { ok: true, program: head("p1-copy", 1, TAB) }] : null);
  const save = sent.find((r) => r.kind === "save")!;
  assert.match(save.path, /programs\/p1-copy\/save$/);
  assert.equal((save.body as { base_revision: number }).base_revision, 0);
  assert.equal(sync.view(400).state, "synced");
  assert.equal(links.get("store-rosie_1420_v1", "p1"), null, "the original follows the cell again when reopened");
});

test("an unreachable cell keeps a local draft and reconciles on reconnect", () => {
  const links = new MemoryLinks();
  links.put({ storeId: "store-rosie_1420_v1", programId: "p1", baseRevision: 2, dirty: false, syncedKey: "k1" });
  const { sync } = connected(links);
  sync.observe(doc("p1", "k1"), 0);
  step(sync, 0, () => [200, storeView({ programs: [head("p1", 2)], watched: head("p1", 2) })]);
  sync.observe(doc("p1", "k2"), 100);
  step(sync, 300, () => [0, null]);
  let view = sync.view(300);
  assert.equal(view.state, "offline");
  assert.equal(view.short, "LOCAL DRAFT");
  assert.match(view.label, /example-relay unreachable · rev 2 \+ local edits/);
  // Backed off: nothing for a second.
  assert.deepEqual(sync.requests(500), []);
  // Back: the cell still has revision 2, so the draft is saved from it.
  const sent: CellRequest[] = [];
  for (let t = 1400; t < 4000; t += 200) {
    sent.push(...step(sync, t, (r) => r.kind === "save"
      ? [200, { ok: true, program: head("p1", 3, TAB) }]
      : [200, storeView({ version: 3, programs: [head("p1", 2)], watched: head("p1", 2) })]));
  }
  const save = sent.find((r) => r.kind === "save")!;
  assert.equal((save.body as { base_revision: number }).base_revision, 2);
  view = sync.view(4000);
  assert.equal(view.state, "synced");
  assert.equal(view.revision, 3);
});

test("a save whose answer was lost is retried with the same save id", () => {
  const links = new MemoryLinks();
  links.put({ storeId: "store-rosie_1420_v1", programId: "p1", baseRevision: 2, dirty: false, syncedKey: "k1" });
  const { sync } = connected(links);
  sync.observe(doc("p1", "k1"), 0);
  step(sync, 0, () => [200, storeView({ programs: [head("p1", 2)], watched: head("p1", 2) })]);
  sync.observe(doc("p1", "k2"), 100);
  // The link drops mid-save: every request in flight fails.
  const first = step(sync, 300, () => [0, null]).find((r) => r.kind === "save")!;
  let retry: CellRequest | undefined;
  for (let t = 1400; !retry && t < 20_000; t += 200) {
    retry = step(sync, t, (r) => r.kind === "save" ? [200, { ok: true, program: head("p1", 3, TAB), replayed: true }]
      : [200, storeView({ programs: [head("p1", 2)], watched: head("p1", 2) })]).find((r) => r.kind === "save");
  }
  assert.equal((retry!.body as { save_id: string }).save_id, (first.body as { save_id: string }).save_id);
  assert.equal(sync.view(20_000).state, "synced");
});

test("another client's save of an unedited program is opened here", () => {
  const links = new MemoryLinks();
  links.put({ storeId: "store-rosie_1420_v1", programId: "p1", baseRevision: 2, dirty: false, syncedKey: "k1" });
  const { sync } = connected(links);
  sync.observe(doc("p1", "k1"), 0);
  step(sync, 0, (r) => r.kind === "feed" ? [200, storeView({ programs: [head("p1", 3)], watched: head("p1", 3) })] : null);
  assert.equal(sync.view(0).state, "updating");
  step(sync, 100, (r) => r.kind === "fetch" ? [200, { program: head("p1", 3), document: { rev: 3 } }] : null);
  const remote = sync.takeRemote()!;
  sync.applied(remote.head, doc("p1", "k3"));
  // The host's next observation of what it opened is not an edit.
  sync.observe(doc("p1", "k3"), 200);
  assert.equal(sync.view(200).label, "synced with example-relay · rev 3");
  assert.equal(sync.view(200).dirty, false);
});

test("an edit made while the cell's revision was on its way becomes a conflict", () => {
  const links = new MemoryLinks();
  links.put({ storeId: "store-rosie_1420_v1", programId: "p1", baseRevision: 2, dirty: false, syncedKey: "k1" });
  const { sync } = connected(links);
  sync.observe(doc("p1", "k1"), 0);
  step(sync, 0, (r) => r.kind === "feed" ? [200, storeView({ programs: [head("p1", 3)], watched: head("p1", 3) })] : null);
  const fetch = sync.requests(100).find((r) => r.kind === "fetch")!;
  sync.observe(doc("p1", "mine"), 150);
  sync.reply(fetch.id, 200, { program: head("p1", 3), document: {} }, 200);
  assert.equal(sync.takeRemote(), null);
  assert.equal(sync.view(200).state, "conflict");
});

test("the program on the robot waits as a draft until Stop", () => {
  const links = new MemoryLinks();
  links.put({ storeId: "store-rosie_1420_v1", programId: "p1", baseRevision: 2, dirty: false, syncedKey: "k1" });
  const { sync } = connected(links);
  sync.observe(doc("p1", "k1"), 0);
  const running = { program_id: "p1", program_ref: "p1", reason: "the robot has this program playing (Stop to release)" };
  step(sync, 0, () => [200, storeView({ programs: [head("p1", 2)], watched: head("p1", 2), executing: running })]);
  sync.observe(doc("p1", "k2"), 100);
  const whileRunning = step(sync, 1000, (r) => r.kind === "save" ? null
    : [200, storeView({ version: 3, programs: [head("p1", 2)], watched: head("p1", 2), executing: running })]);
  assert.equal(whileRunning.filter((r) => r.kind === "save").length, 0, "no save is sent while the robot runs it");
  let view = sync.view(1000);
  assert.equal(view.state, "locked");
  assert.match(view.label, /local draft · the robot has this program playing/);
  // A 423 from a race says the same.
  const links2 = new MemoryLinks();
  links2.put({ storeId: "store-rosie_1420_v1", programId: "p1", baseRevision: 2, dirty: false, syncedKey: "k1" });
  const other = connected(links2).sync;
  other.observe(doc("p1", "k1"), 0);
  step(other, 0, () => [200, storeView({ programs: [head("p1", 2)], watched: head("p1", 2) })]);
  other.observe(doc("p1", "k2"), 100);
  step(other, 300, (r) => r.kind === "save" ? [423, { error: "program_executing", detail: "the robot has this program loaded (Stop to release)" }] : null);
  assert.equal(other.view(300).state, "locked");
  // Stop: the next view has nothing executing and the draft goes.
  const sent = step(sync, 2000, (r) => r.kind === "save" ? [200, { ok: true, program: head("p1", 3, TAB) }]
    : [200, storeView({ version: 4, programs: [head("p1", 2)], watched: head("p1", 2) })]);
  step(sync, 2100, (r) => r.kind === "save" ? [200, { ok: true, program: head("p1", 3, TAB) }] : null);
  view = sync.view(2200);
  assert.ok(sent.length > 0);
  assert.equal(view.state, "synced");
  assert.equal(view.revision, 3);
});

test("who else has the program open is named, and this client is not", () => {
  const links = new MemoryLinks();
  links.put({ storeId: "store-rosie_1420_v1", programId: "p1", baseRevision: 2, dirty: false, syncedKey: "k1" });
  const { sync } = connected(links);
  sync.observe(doc("p1", "k1"), 0);
  step(sync, 0, () => [200, storeView({
    programs: [head("p1", 2)], watched: head("p1", 2),
    presence: [
      { program_id: "p1", id: "deck-1", label: DECK, editing: true },
      { program_id: "p1", id: "tab-1", label: TAB, editing: false, you: true },
      { program_id: "p2", id: "tab-2", label: TAB, editing: true },
    ],
  })]);
  assert.equal(sync.view(0).editorsText, "steamdeck-operator editing");
  // An edit here tells the cell at once, not at the next long-poll.
  sync.observe(doc("p1", "k2"), 100);
  const presence = sync.requests(100).find((r) => r.kind === "presence")!;
  assert.match(presence.path, /editing=1/);
  assert.match(presence.path, /timeout_ms=0/);
});

test("names: the Deck by host, the browser without its origin", () => {
  assert.equal(clientName(DECK), "steamdeck-operator");
  assert.equal(clientName(TAB), "Browser (Chrome on macOS)");
  assert.equal(clientName("curl (deploy tool or a script)"), "curl (deploy tool or a script)");
  assert.equal(clientName(""), "another client");
});

test("the Deck follows the cell's current program when its own has nothing unsaved", () => {
  const links = new MemoryLinks();
  links.put({ storeId: "store-rosie_1420_v1", programId: "p1", baseRevision: 2, dirty: false, syncedKey: "k1" });
  const { sync } = connected(links, { followCurrent: true });
  sync.observe(doc("p1", "k1"), 0);
  step(sync, 0, (r) => r.kind === "feed" ? [200, storeView({ current: "p2", programs: [head("p1", 2), head("p2", 7)], watched: head("p1", 2) })] : null);
  assert.match(sync.takeNotices()[0] ?? "", /Opening Program p2: example-relay's current program \(set by steamdeck-operator\)/);
  step(sync, 100, (r) => r.kind === "fetch" ? [200, { program: head("p2", 7), document: { p2: true } }] : null);
  const remote = sync.takeRemote()!;
  assert.equal(remote.head.programId, "p2");
  sync.applied(remote.head, doc("p2", "k7"));
  sync.observe(doc("p2", "k7"), 200);
  assert.equal(sync.view(200).isCurrent, true);
});

test("the Deck asks once, with the cell's saved programs, before replacing a program the cell does not have", () => {
  const { sync } = connected(new MemoryLinks(), { followCurrent: true });
  sync.observe({ ...doc("local", "k1", "Pendant teach"), document: { identity: { id: "local", updatedAt: "2026-10-11T17:58:00Z" } } }, 0);
  const now = Date.parse("2026-10-11T18:00:00Z");
  step(sync, now, (r) => r.kind === "feed" ? [200, storeView({
    current: "p2",
    programs: [
      head("p1", 3, TAB, { updated_at: "2026-10-11T17:00:00Z", name: "Frame" }),
      head("p2", 7, DECK, { updated_at: "2026-10-11T17:57:00Z", name: "Bracket" }),
    ],
  })] : null);
  const offer = sync.takeOffer(now)!;
  assert.equal(offer.host, "example-relay");
  assert.deepEqual(offer.rows.map((r) => [r.name, r.revision, r.current, r.running, r.saved]), [
    ["Bracket", 7, true, false, "saved 3 min ago by steamdeck-operator"],
    ["Frame", 3, false, false, "saved 1 h ago by Browser (Chrome on macOS)"],
  ]);
  assert.deepEqual(offer.local, { programId: "local", name: "Pendant teach", edited: "edited 2 min ago" });
  assert.equal(sync.takeOffer(now), null, "asked once per cell and program");
  assert.equal(sync.requests(now + 100).filter((r) => r.kind === "fetch" || r.kind === "save").length, 0, "nothing is replaced or uploaded unasked");
});

test("the picker says why a program cannot be opened while the robot runs another", () => {
  const { sync } = connected(new MemoryLinks(), { followCurrent: true });
  sync.observe(doc("local", "k1"), 0);
  step(sync, 0, (r) => r.kind === "feed" ? [200, storeView({
    current: "p2", programs: [head("p1", 3, TAB, { name: "Frame" }), head("p2", 7, DECK, { name: "Bracket" })],
    executing: { program_id: "p2", program_ref: "p2", reason: "the robot has this program playing (Stop to release)" },
  })] : null);
  const rows = sync.takeOffer(0)!.rows;
  const bracket = rows.find((r) => r.programId === "p2")!;
  const frame = rows.find((r) => r.programId === "p1")!;
  assert.equal(bracket.running, true);
  assert.equal(bracket.blocked, "");
  assert.equal(frame.blocked, "The robot is running Bracket; Stop it before opening another program");
});

test("an empty cell offers only this program and a new one: nothing is uploaded unasked", () => {
  const { sync } = connected(new MemoryLinks(), { followCurrent: true });
  sync.observe(doc("local", "k1", "Pendant teach"), 0);
  step(sync, 0, (r) => r.kind === "feed" ? [200, storeView()] : null);
  const offer = sync.takeOffer(0)!;
  assert.deepEqual(offer.rows, []);
  assert.equal(offer.local?.name, "Pendant teach");
  assert.equal(sync.requests(1000).filter((r) => r.kind === "save").length, 0);
  sync.upload(true, 1000);
  const save = step(sync, 2000, (r) => r.kind === "save" ? [200, { ok: true, program: head("local", 1) }] : null).find((r) => r.kind === "save")!;
  assert.equal((save.body as { make_current: boolean }).make_current, true);
});

test("a program the cell has but this client never synced is a conflict, not an upload", () => {
  const { sync } = connected();
  sync.observe(doc("p1", "k1"), 0);
  step(sync, 0, (r) => r.kind === "feed" ? [200, storeView({ programs: [head("p1", 3)] })] : null);
  const view = sync.view(0);
  assert.equal(view.state, "conflict");
  assert.equal(view.offerUpload, false);
});

test("a revision number this client already used is moved past it on open", () => {
  const local = { identity: { id: "p1", revision: 9 } } as unknown as OfflineProgrammingProject;
  const remote = { identity: { id: "p1", revision: 7, name: "x" } } as unknown as OfflineProgrammingProject;
  assert.equal(adoptRemoteProject(local, remote).identity.revision, 10);
  assert.equal(adoptRemoteProject(null, remote).identity.revision, 7);
  const ahead = { identity: { id: "p1", revision: 12 } } as unknown as OfflineProgrammingProject;
  assert.equal(adoptRemoteProject(local, ahead), ahead);
  const copy = copyProject({ identity: { id: "p1", name: "Bracket", revision: 3 }, program: { programId: "p1", baseProgramRevision: 2 } } as unknown as OfflineProgrammingProject);
  assert.notEqual(copy.identity.id, "p1");
  assert.equal(copy.program.programId, copy.identity.id);
  assert.equal(copy.identity.name, "Bracket (copy)");
});

test("a feed answer for the program open before a switch is not read as the new one's", () => {
  const links = new MemoryLinks();
  links.put({ storeId: "store-rosie_1420_v1", programId: "p1", baseRevision: 2, dirty: false, syncedKey: "k1" });
  links.put({ storeId: "store-rosie_1420_v1", programId: "p2", baseRevision: 1, dirty: false, syncedKey: "q1" });
  const { sync } = connected(links);
  sync.observe(doc("p1", "k1"), 0);
  step(sync, 0, () => [200, storeView({ programs: [head("p1", 2), head("p2", 1)], watched: head("p1", 2) })]);
  const feed = sync.requests(300).find((r) => r.kind === "feed")!;
  assert.match(feed.path, /watch=p1/);
  sync.observe(doc("p2", "q1"), 400);
  // The answer watched p1, which moved to rev 5; p2 is still at rev 1.
  sync.reply(feed.id, 200, storeView({ version: 5, programs: [head("p1", 5), head("p2", 1)], watched: head("p1", 5) }), 500);
  assert.equal(sync.requests(1000).filter((r) => r.kind === "fetch").length, 0);
  assert.equal(sync.view(1000).label, "synced with example-relay · rev 1");
});

test("an upload asked before the cell answered waits for it", () => {
  const { sync } = connected(new MemoryLinks(), { followCurrent: true });
  sync.observe(doc("new", "k1"), 0);
  sync.upload(true, 0);
  step(sync, 0, (r) => r.kind === "feed" ? [200, storeView({ current: "p2", programs: [head("p2", 7)] })] : null);
  assert.equal(sync.takeOffer(0), null, "no question: the operator already chose this program");
  const save = step(sync, 1000, (r) => r.kind === "save" ? [200, { ok: true, program: head("new", 1) }] : null).find((r) => r.kind === "save")!;
  assert.equal((save.body as { make_current: boolean }).make_current, true);
});
