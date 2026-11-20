import assert from "node:assert/strict";
import test from "node:test";
import { MaintenanceTracker, formatMaintenanceDuration, formatMaintenanceMinutes, normalizeMaintenance, normalizeMaintenanceProgress } from "./maintenance.ts";
import { MaintenanceWatch } from "./maintenanceWatch.ts";

const T0 = Date.parse("2026-10-07T20:00:00Z");

function notice(overrides: Record<string, unknown> = {}) {
  return {
    schema: "offline-programming.maintenance.v1", active: true,
    reason: "rt-core full deploy fd9af899", phase: "installing package on the servo host",
    started_at: "2026-10-07T19:50:00Z", eta: "2026-10-07T20:30:00Z",
    set_by: "realtime-core deploy (operator@example-relay)", release: "fd9af899", run_id: "run-1",
    now: "2026-10-07T20:00:00Z", overrun: false, stale: false, malformed: false,
    server: { build_id: "sha256:aaaaaaaaaaaa", started_at: "2026-10-07T08:00:00Z" },
    ...overrides,
  };
}
const inactive = (build = "sha256:aaaaaaaaaaaa") => notice({ active: false, reason: "", server: { build_id: build } });

test("durations read as m:ss under an hour and h:mm:ss over", () => {
  assert.equal(formatMaintenanceDuration(0), "0:00");
  assert.equal(formatMaintenanceDuration(754_000), "12:34");
  assert.equal(formatMaintenanceDuration(3_723_900), "1:02:03");
  assert.equal(formatMaintenanceDuration(-5_000), "0:00");
});

test("an estimate reads in whole minutes, rounded up", () => {
  assert.equal(formatMaintenanceMinutes(754_000), "13 min");
  assert.equal(formatMaintenanceMinutes(60_000), "1 min");
  assert.equal(formatMaintenanceMinutes(3_600_000), "1 h");
  assert.equal(formatMaintenanceMinutes(3_900_000), "1 h 5 min");
  assert.equal(formatMaintenanceMinutes(0), "1 min");
});

test("a notice that is not one is ignored, never read as cleared", () => {
  assert.equal(normalizeMaintenance(null), null);
  assert.equal(normalizeMaintenance({ reason: "x" }), null);
  const tracker = new MaintenanceTracker();
  tracker.observe({ kind: "notice", payload: notice() }, T0);
  tracker.observe({ kind: "notice", payload: "<html>proxy error</html>" }, T0 + 2000);
  assert.equal(tracker.display(T0 + 2000).blocking, true);
});

test("an active notice blocks with the minutes left on the server's clock", () => {
  const tracker = new MaintenanceTracker();
  // This client's clock is 90 s fast; the estimate follows the server's.
  tracker.observe({ kind: "notice", payload: notice() }, T0 + 90_000);
  const view = tracker.display(T0 + 90_000 + 1_000);
  assert.equal(view.blocking, true);
  assert.equal(view.title, "Update in progress");
  assert.equal(view.reason, "rt-core full deploy fd9af899");
  assert.equal(view.phase, "installing package on the servo host");
  assert.equal(view.countdownLabel, "Expected finish");
  assert.equal(view.countdown, "About 30 min left");
  assert.equal(view.elapsed, "Running for 10:01");
  assert.equal(view.source, "Started by realtime-core deploy (operator@example-relay) · release fd9af899");
  assert.equal(view.progress, null, "a notice without progress (an older deploy tool) has no bar");
  assert.equal(view.stepLine, "");
  assert.deepEqual(view.steps, []);
  assert.equal(view.overrun, false);
  assert.match(view.notes.at(-1)!, /STOP stays available/);
});

test("past the ETA it says it is taking longer, never a negative clock; a stale notice says how to clear it", () => {
  const tracker = new MaintenanceTracker();
  tracker.observe({ kind: "notice", payload: notice({ eta: "2026-10-07T19:59:40Z" }) }, T0);
  let view = tracker.display(T0);
  assert.equal(view.overrun, true);
  assert.equal(view.countdownLabel, "Past the expected finish");
  assert.equal(view.countdown, "Taking longer than expected");
  tracker.observe({ kind: "notice", payload: notice({ eta: "2026-10-07T19:58:30Z" }) }, T0);
  view = tracker.display(T0);
  assert.equal(view.countdownLabel, "Past the expected finish");
  assert.equal(view.countdown, "Taking longer than expected (+1 min)");
  // The deploy re-estimates as its steps finish: a later ETA is simply shown.
  tracker.observe({ kind: "notice", payload: notice({ eta: "2026-10-07T20:04:00Z", now: "2026-10-07T20:00:02Z" }) }, T0 + 2_000);
  view = tracker.display(T0 + 2_000);
  assert.equal(view.overrun, false);
  assert.equal(view.countdown, "About 4 min left");
  assert.equal(view.elapsed, "Running for 10:02", "the start stays");
  tracker.observe({ kind: "notice", payload: notice({ eta: "2026-10-07T20:00:30Z" }) }, T0);
  assert.equal(tracker.display(T0).countdown, "Less than a minute left");
  tracker.observe({ kind: "notice", payload: notice({ eta: "2026-10-07T17:00:00Z", stale: true, overrun: true }) }, T0);
  view = tracker.display(T0);
  assert.equal(view.blocking, true, "a stale notice still blocks: nothing clears it silently");
  assert.equal(view.stale, true);
  assert.equal(view.countdownLabel, "Past the expected finish");
  assert.equal(view.countdown, "Taking longer than expected (+3 h)");
  assert.ok(view.notes.some((note) => note.includes("offline-programming maintenance clear")));
  assert.ok(!view.notes.some((note) => /site data/i.test(note)));
});

// A full release mid-gate, as the deploy tool writes it (notice_progress.go).
const midGate = {
  steps: [
    { name: "build+test", label: "Build and test gate", state: "running", estimate_s: 1077, elapsed_s: 420, remaining_s: 600 },
    { name: "copy-dumps", label: "Copy stop dumps off the servo host", state: "pending", estimate_s: 4, elapsed_s: 0, remaining_s: 4 },
    { name: "deploy", label: "Install rt-core on the servo host", state: "pending", estimate_s: 36, elapsed_s: 0, remaining_s: 36 },
    { name: "olp install", label: "Install the OLP server", state: "pending", estimate_s: 7, elapsed_s: 0, remaining_s: 7 },
  ],
  current: 1, total: 4, fraction: 0.41,
  suite: { name: "ui_test", index: 5, total: 12, done: ["cells", "pendant_olp_core", "sdk_path_jog"], running: ["make_test", "ui_test"] },
  updated_at: "2026-10-07T20:00:00Z",
};

test("a run's progress shows its step, the gate's suite, a time-weighted bar and the step list", () => {
  const tracker = new MaintenanceTracker();
  tracker.observe({ kind: "notice", payload: notice({ phase: "Step 1/4 · Build and test gate · suite 5/12 ui_test", eta: "2026-10-07T20:12:00Z", progress: midGate }) }, T0);
  let view = tracker.display(T0);
  assert.equal(view.stepLine, "Step 1/4 · Build and test gate");
  assert.equal(view.suiteLine, "Suite 5/12: ui_test · 3 done · 2 running");
  assert.equal(view.phase, "", "the run's own step line is not repeated as Now:");
  // (1077 - 600) / (1077 + 4 + 36 + 7): weighted by estimated time, not 0 of 4 steps.
  assert.ok(Math.abs(view.progress! - 477 / 1124) < 1e-9, String(view.progress));
  assert.equal(view.progressText, "42%");
  assert.equal(view.countdown, "About 12 min left");
  assert.deepEqual(view.steps.map((s) => `${s.mark} ${s.label} ${s.time}`), [
    "▸ Build and test gate 7:00 of about 18 min",
    "· Copy stop dumps off the servo host about 1 min",
    "· Install rt-core on the servo host about 1 min",
    "· Install the OLP server about 1 min",
  ]);
  // Between updates the running step advances on the server's clock.
  view = tracker.display(T0 + 60_000);
  assert.ok(Math.abs(view.progress! - 537 / 1124) < 1e-9, String(view.progress));
  assert.equal(view.steps[0]!.time, "8:00 of about 18 min");
  // ...but never past 99 % of the step however long it overruns.
  view = tracker.display(T0 + 3_600_000);
  assert.ok(view.progress! < 1);
  assert.equal(view.countdown, "Taking longer than expected (+48 min)");
  // An inner step's own phase is shown under the step line.
  const done = { ...midGate, current: 3, suite: null, steps: midGate.steps.map((s, i) => ({ ...s, state: i < 2 ? "done" : i === 2 ? "running" : "pending", elapsed_s: i === 0 ? 1100 : i === 1 ? 4 : 10, remaining_s: i === 2 ? 26 : s.remaining_s })) };
  tracker.observe({ kind: "notice", payload: notice({ phase: "copying the package", progress: done }) }, T0);
  view = tracker.display(T0);
  assert.equal(view.stepLine, "Step 3/4 · Install rt-core on the servo host");
  assert.equal(view.suiteLine, "");
  assert.equal(view.phase, "copying the package");
  assert.deepEqual(view.steps.map((s) => s.mark), ["✓", "✓", "▸", "·"]);
  assert.equal(view.steps[0]!.time, "18:20");
});

test("a malformed progress is ignored; the notice still blocks with its phase and ETA", () => {
  const tracker = new MaintenanceTracker();
  tracker.observe({ kind: "notice", payload: notice({ progress: { steps: "build" } }) }, T0);
  const view = tracker.display(T0);
  assert.equal(view.blocking, true);
  assert.equal(view.progress, null);
  assert.equal(view.phase, "installing package on the servo host");
  assert.equal(view.countdown, "About 30 min left");
  assert.equal(normalizeMaintenanceProgress({ steps: [{ name: "x", state: "bogus", estimate_s: -4 }] })!.steps[0]!.state, "pending");
});

test("without an ETA there is no countdown; an unreadable notice still blocks", () => {
  const tracker = new MaintenanceTracker();
  tracker.observe({ kind: "notice", payload: notice({ eta: undefined, malformed: true, error: "eta is not RFC 3339" }) }, T0);
  const view = tracker.display(T0);
  assert.equal(view.blocking, true);
  assert.equal(view.countdownLabel, "No expected finish time");
  assert.equal(view.countdown, "");
  assert.ok(view.notes.some((note) => note.includes("eta is not RFC 3339")));
});

test("an unreachable server keeps an active notice up, and only then", () => {
  const fresh = new MaintenanceTracker();
  fresh.observe({ kind: "unreachable", error: "HTTP 502" }, T0);
  assert.equal(fresh.display(T0).blocking, false, "a server that is simply down is not maintenance");

  const tracker = new MaintenanceTracker();
  tracker.observe({ kind: "notice", payload: notice() }, T0);
  tracker.observe({ kind: "unreachable", error: "Failed to fetch" }, T0 + 2000);
  const view = tracker.display(T0 + 4000);
  assert.equal(view.blocking, true);
  assert.equal(view.unreachable, true);
  assert.equal(view.countdown, "About 30 min left", "the estimate keeps running from the last answer");
  assert.equal(tracker.display(T0 + 1_741_000).countdown, "Less than a minute left");
  assert.match(view.notes[0]!, /not answering/);
  assert.equal(tracker.needsHealth, false);
});

test("a cleared notice unblocks only after health, and suggests a reload when the build changed", () => {
  const tracker = new MaintenanceTracker();
  tracker.observe({ kind: "notice", payload: inactive("sha256:old000000000") }, T0 - 10_000);
  tracker.observe({ kind: "notice", payload: notice({ server: { build_id: "sha256:old000000000" } }) }, T0);
  tracker.observe({ kind: "unreachable", error: "restarting" }, T0 + 2000);
  tracker.observe({ kind: "notice", payload: inactive("sha256:new000000000") }, T0 + 4000);
  assert.equal(tracker.needsHealth, true);
  assert.equal(tracker.display(T0 + 4000).blocking, true, "still blocked until health answers");
  assert.ok(tracker.display(T0 + 4000).notes.some((note) => note.includes("checking that the OLP server is healthy")));
  tracker.confirmHealth(false);
  assert.equal(tracker.display(T0 + 4000).blocking, true, "unhealthy keeps blocking");
  tracker.confirmHealth(true, "sha256:new000000000");
  const view = tracker.display(T0 + 4000);
  assert.equal(view.blocking, false);
  assert.equal(view.reloadSuggested, true);
  assert.equal(view.buildBefore, "sha256:old000000000");
  assert.equal(view.buildAfter, "sha256:new000000000");
  tracker.dismissReload();
  assert.equal(tracker.display(T0 + 5000).reloadSuggested, false);
});

test("a notice that clears on the same build unblocks without a reload", () => {
  const tracker = new MaintenanceTracker();
  tracker.observe({ kind: "notice", payload: inactive() }, T0);
  tracker.observe({ kind: "notice", payload: notice() }, T0 + 2000);
  tracker.observe({ kind: "notice", payload: inactive() }, T0 + 4000);
  tracker.confirmHealth(true, "sha256:aaaaaaaaaaaa");
  const view = tracker.display(T0 + 4000);
  assert.equal(view.blocking, false);
  assert.equal(view.reloadSuggested, false);
});

test("a server without the notice route is no notice; after one it confirms health", () => {
  const tracker = new MaintenanceTracker();
  tracker.observe({ kind: "absent" }, T0);
  assert.equal(tracker.display(T0).blocking, false);
  tracker.observe({ kind: "notice", payload: notice() }, T0);
  tracker.observe({ kind: "absent" }, T0 + 2000);
  assert.equal(tracker.needsHealth, true);
});

test("the watch polls the notice, then health once it clears", async () => {
  const replies: Array<{ path: string; status: number; body?: unknown } | Error> = [
    { path: "/api/offline-programming/v1/maintenance", status: 200, body: notice() },
    new Error("Failed to fetch"),
    { path: "/api/offline-programming/v1/maintenance", status: 502 },
    { path: "/api/offline-programming/v1/maintenance", status: 200, body: inactive("sha256:new000000000") },
    { path: "/api/offline-programming/v1/health", status: 200, body: { ok: true, build_id: "sha256:new000000000" } },
  ];
  const asked: string[] = [];
  const shown: boolean[] = [];
  const watch = new MaintenanceWatch({
    now: () => T0,
    render: (display) => shown.push(display.blocking),
    fetch: async (path, init) => {
      asked.push(path);
      assert.equal(init?.cache, "no-store");
      const reply = replies.shift()!;
      if (reply instanceof Error) throw reply;
      assert.equal(reply.path, path);
      return new Response(reply.body === undefined ? "" : JSON.stringify(reply.body), { status: reply.status });
    },
  });
  assert.equal((await watch.poll()).blocking, true);
  assert.equal((await watch.poll()).unreachable, true);
  assert.equal((await watch.poll()).blocking, true, "a proxy's 502 is unreachable, not cleared");
  const done = await watch.poll();
  assert.equal(done.blocking, false);
  assert.deepEqual(asked, [
    "/api/offline-programming/v1/maintenance", "/api/offline-programming/v1/maintenance",
    "/api/offline-programming/v1/maintenance", "/api/offline-programming/v1/maintenance",
    "/api/offline-programming/v1/health",
  ]);
  assert.deepEqual(shown, [true, true, true, false]);
  watch.stop();
});

test("the watch hands the renderer who is connected from the last answer", async () => {
  let sessions: unknown[] = [];
  const watch = new MaintenanceWatch({
    now: () => T0,
    render: (_display, lines) => { sessions = lines; },
    fetch: async () => new Response(JSON.stringify(notice({ clients: [
      { kind: "pendant", label: "rosie-pendant-v5 (steamdeck-pendant)", address: "192.168.1.113", last_seen: "2026-10-07T19:59:58Z", session_holder: true },
    ] })), { status: 200 }),
  });
  await watch.poll();
  assert.deepEqual(sessions, [{ kind: "pendant", text: "rosie-pendant-v5 (steamdeck-pendant) · from 192.168.1.113 · seen 2 s ago · holds the execution session", holds: true }]);
  watch.stop();
});

test("the cell's own notice blocks a client whose OLP host has none (a Deck running OLP locally)", () => {
  // The server merged the servo host's notice into the top-level fields.
  const cell = { cell: "example-arm-1420", reachable: true, supported: true, active: true, reason: "rt-core full deploy on example-arm-1420",
    holder: { controller: "offline-programming:server-1a2b3c", remote: true, identity: "steamdeck-pendant-olp", generation: 4, since: "2026-10-07T19:30:00Z" } };
  const tracker = new MaintenanceTracker();
  tracker.observe({ kind: "notice", payload: notice({ reason: cell.reason, set_by: "realtime-core deploy (operator@mac)", run_id: undefined, cell }) }, T0);
  let view = tracker.display(T0);
  assert.equal(view.blocking, true);
  assert.equal(view.reason, "rt-core full deploy on example-arm-1420");
  assert.equal(normalizeMaintenance(notice({ cell }))!.cell!.cell, "example-arm-1420");
  assert.ok(!view.notes.some((note) => note.includes("rt-control")), "a reachable rt-control adds no note");

  // A full deploy restarts rt-control: the server keeps its last notice up.
  tracker.observe({ kind: "notice", payload: notice({ reason: cell.reason, cell: { ...cell, reachable: false, holder: undefined } }) }, T0 + 2000);
  view = tracker.display(T0 + 2000);
  assert.equal(view.blocking, true);
  assert.ok(view.notes.some((note) => note.includes("rt-control on the servo host (example-arm-1420) is not answering")));

  tracker.observe({ kind: "notice", payload: notice({ reason: cell.reason, stale: true, overrun: true, eta: "2026-10-07T17:00:00Z", cell }) }, T0 + 4000);
  assert.ok(tracker.display(T0 + 4000).notes.some((note) => note.includes("`realtime-core maintenance clear` (both hosts)")));

  // An inactive cell notice is the ordinary case and changes nothing.
  const quiet = new MaintenanceTracker();
  quiet.observe({ kind: "notice", payload: inactive() }, T0);
  quiet.observe({ kind: "notice", payload: { ...inactive(), cell: { ...cell, active: false, reason: "" } } }, T0 + 2000);
  assert.equal(quiet.display(T0 + 2000).blocking, false);
});
