import assert from "node:assert/strict";
import test from "node:test";
import * as maintenance from "./maintenance.ts";

const T0 = Date.parse("2026-10-07T20:00:00Z");
const active = {
  active: true, reason: "rt-core configuration deploy", phase: "restarting rosie-rt-core",
  started_at: "2026-10-07T19:59:00Z", eta: "2026-10-07T20:01:00Z", now: "2026-10-07T20:00:00Z",
  set_by: "realtime-core config (operator@example-relay)", server: { build_id: "git:aaaaaaaaaaaa" },
};

test("the pendant blocks on a notice and through the server going away during it", () => {
  maintenance.maintenanceForget();
  assert.equal(maintenance.maintenanceObserve(0, null, "olp_server_unreachable", T0), false);
  assert.equal(maintenance.maintenanceView(T0).blocking, false, "an unreachable server alone is not maintenance");
  maintenance.maintenanceObserve(200, active, "", T0);
  let view = maintenance.maintenanceView(T0 + 30_000);
  assert.equal(view.blocking, true);
  assert.equal(view.countdown, "Less than a minute left");
  assert.equal(view.phase, "restarting rosie-rt-core");
  maintenance.maintenanceObserve(0, null, "olp_server_unreachable", T0 + 2_000);
  view = maintenance.maintenanceView(T0 + 90_000);
  assert.equal(view.blocking, true);
  assert.equal(view.unreachable, true);
  assert.equal(view.countdownLabel, "Past the expected finish");
  assert.equal(view.countdown, "Taking longer than expected");
});

test("the pendant shows a run's progress: step, gate suite, bar, steps; its clock offset applies", () => {
  maintenance.maintenanceForget();
  const progress = {
    steps: [
      { name: "build+test", label: "Build and test gate", state: "running", estimate_s: 1077, elapsed_s: 420, remaining_s: 600 },
      { name: "deploy", label: "Install rt-core on the servo host", state: "pending", estimate_s: 36, elapsed_s: 0, remaining_s: 36 },
    ],
    current: 1, total: 2, fraction: 0.43, updated_at: "2026-10-07T20:00:00Z",
    suite: { name: "ui_test", index: 5, total: 12, done: ["cells", "olp"], running: ["ui_test"] },
  };
  // This Deck's clock runs 60 s behind the server's.
  maintenance.maintenanceObserve(200, { ...active, eta: "2026-10-07T20:12:00Z", phase: "Step 1/2 · Build and test gate · suite 5/12 ui_test", progress }, "", T0 - 60_000);
  const view = maintenance.maintenanceView(T0 - 60_000);
  assert.equal(view.stepLine, "Step 1/2 · Build and test gate");
  assert.equal(view.suiteLine, "Suite 5/12: ui_test · 2 done");
  assert.equal(view.phase, "");
  assert.ok(Math.abs(view.progress! - 477 / 1113) < 1e-9, String(view.progress));
  assert.equal(view.progressText, "42%");
  assert.equal(view.countdown, "About 12 min left");
  assert.deepEqual(view.steps.map(s => `${s.mark} ${s.label} ${s.time}`), [
    "▸ Build and test gate 7:00 of about 18 min", "· Install rt-core on the servo host about 1 min",
  ]);
});

test("the pendant lists who is connected on the server's clock", () => {
  maintenance.maintenanceForget();
  assert.deepEqual(maintenance.maintenanceSessions(T0), []);
  // This Deck's clock runs 60 s behind the server's.
  maintenance.maintenanceObserve(200, { ...active, clients: [
    { kind: "browser", label: "Browser (Chrome on macOS) on http://127.0.0.1:5189", address: "127.0.0.1", loopback: true,
      last_seen: "2026-10-07T19:59:55Z", session_holder: true },
  ] }, "", T0 - 60_000);
  assert.deepEqual(maintenance.maintenanceSessions(T0 - 60_000).map(line => [line.text, line.holds]), [
    ["Browser (Chrome on macOS) on http://127.0.0.1:5189 · from 127.0.0.1 (this host or a tunnel) · seen 5 s ago · holds the execution session", true],
  ]);
});

test("the pendant unblocks only after the cleared server answers health", () => {
  maintenance.maintenanceForget();
  maintenance.maintenanceObserve(200, active, "", T0);
  assert.equal(maintenance.maintenanceObserve(200, { active: false, server: { build_id: "git:bbbbbbbbbbbb" } }, "", T0 + 2_000), true);
  assert.equal(maintenance.maintenanceView(T0 + 2_000).blocking, true);
  maintenance.maintenanceHealth(true, "git:bbbbbbbbbbbb");
  const view = maintenance.maintenanceView(T0 + 2_000);
  assert.equal(view.blocking, false);
  assert.equal(view.reloadSuggested, true);
  maintenance.maintenanceDismissReload();
  assert.equal(maintenance.maintenanceView(T0 + 2_000).reloadSuggested, false);
});

test("the pendant names who set the notice and its run, and flags it STALE 2 h past its ETA", () => {
  maintenance.maintenanceForget();
  maintenance.maintenanceObserve(200, { ...active, run_id: "20261007T195900Z-realtime-core-config-example-arm-1420" }, "", T0);
  let view = maintenance.maintenanceView(T0);
  assert.equal(view.source, "Started by realtime-core config (operator@example-relay) · run 20261007T195900Z-realtime-core-config-example-arm-1420");
  assert.equal(view.stale, false);
  assert.equal(view.staleLine, "");
  // The ETA is 20:01:00. The server stops answering; 2 h after the ETA the
  // notice is still overrunning but not stale, one second later it is.
  maintenance.maintenanceObserve(0, null, "olp_server_unreachable", T0 + 2_000);
  view = maintenance.maintenanceView(T0 + 60_000 + 7_200_000);
  assert.equal(view.blocking, true);
  assert.equal(view.overrun, true);
  assert.equal(view.stale, false);
  view = maintenance.maintenanceView(T0 + 60_000 + 7_201_000);
  assert.equal(view.blocking, true, "a stale notice still blocks; nothing on the pendant clears it");
  assert.equal(view.stale, true);
  assert.equal(view.staleLine, "STALE: 2:00:01 past its expected finish");
});

test("the pendant keeps the server's stale flag and judges an ETA-less notice by its age", () => {
  maintenance.maintenanceForget();
  // A server configured with OFFLINE_PROGRAMMING_MAINTENANCE_STALE_AFTER=30m,
  // 31 min past the 20:01:00 ETA.
  maintenance.maintenanceObserve(200, { ...active, now: "2026-10-07T20:32:00Z", stale: true }, "", T0 + 1_920_000);
  assert.equal(maintenance.maintenanceView(T0 + 1_920_000).staleLine, "STALE: 31:00 past its expected finish");
  maintenance.maintenanceForget();
  maintenance.maintenanceObserve(200, { ...active, eta: undefined }, "", T0);
  assert.equal(maintenance.maintenanceView(T0 + 7_140_000).stale, false);
  assert.equal(maintenance.maintenanceView(T0 + 7_141_000).staleLine, "STALE: up for 2:00:01 with no expected finish");
  // Cleared and healthy: nothing left to flag.
  maintenance.maintenanceObserve(200, { active: false, server: { build_id: "git:aaaaaaaaaaaa" } }, "", T0 + 7_200_000);
  maintenance.maintenanceHealth(true, "git:aaaaaaaaaaaa");
  const view = maintenance.maintenanceView(T0 + 7_200_000);
  assert.equal(view.blocking, false);
  assert.equal(view.staleLine, "");
});

test("a Deck running OLP locally blocks on the cell's own notice and names the servo host's grant holder", () => {
  maintenance.maintenanceForget();
  // The Deck's OLP host has no notice; its server merged the servo host's.
  const cell = { cell: "example-arm-1420", reachable: true, supported: true, active: true, reason: "rt-core full deploy on example-arm-1420",
    holder: { controller: "offline-programming:server-1a2b3c", remote: true, identity: "example-relay-olp", generation: 7, since: "2026-10-07T19:57:00Z" } };
  maintenance.maintenanceObserve(200, { ...active, reason: cell.reason, phase: "", set_by: "realtime-core deploy (operator@mac)", cell }, "", T0);
  let view = maintenance.maintenanceView(T0);
  assert.equal(view.blocking, true);
  assert.equal(view.reason, "rt-core full deploy on example-arm-1420");
  assert.deepEqual(maintenance.maintenanceSessions(T0).map(line => [line.kind, line.text, line.holds]), [
    ["grant", "Servo host grant held by offline-programming:server-1a2b3c · remote example-relay-olp · since 3 min ago", true],
  ]);
  // rt-control restarts during the deploy: the server keeps the last notice.
  maintenance.maintenanceObserve(200, { ...active, reason: cell.reason, cell: { ...cell, reachable: false, holder: undefined } }, "", T0 + 2_000);
  view = maintenance.maintenanceView(T0 + 2_000);
  assert.equal(view.blocking, true);
  assert.ok(view.notes.some((note: string) => note.includes("is not answering")));
});
