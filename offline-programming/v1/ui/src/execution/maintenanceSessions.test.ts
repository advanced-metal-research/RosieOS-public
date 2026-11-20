import assert from "node:assert/strict";
import test from "node:test";
import { maintenanceAgo, maintenanceSessionLines } from "./maintenanceSessions.ts";

const NOW = Date.parse("2026-10-07T21:00:30Z");

test("the machine comes first, then each client in the server's words", () => {
  const lines = maintenanceSessionLines({
    machine: { cell: "example-arm-1420", label: "Rosie 1430 rosie_1420_v1", armed: true, lease_valid: true, session_state: "loaded" },
    clients: [
      { kind: "browser", label: "Browser (Chrome on macOS) on http://127.0.0.1:5189", address: "127.0.0.1", loopback: true,
        last_seen: "2026-10-07T21:00:28Z", last_action: "arm", last_action_at: "2026-10-07T20:58:30Z", session_holder: true },
      { kind: "pendant", label: "rosie-pendant-v5 (steamdeck-pendant)", address: "192.168.1.113", loopback: false, last_seen: "2026-10-07T21:00:30Z" },
      { kind: "unknown", address: "10.0.0.9", last_seen: "garbage" },
    ],
  }, NOW);
  assert.deepEqual(lines, [
    { kind: "cell", text: "Cell Rosie 1430 rosie_1420_v1 · ARMED · control lease held · program loaded", holds: true },
    { kind: "browser", text: "Browser (Chrome on macOS) on http://127.0.0.1:5189 · from 127.0.0.1 (this host or a tunnel) · seen 2 s ago · last robot request arm 2 min ago · holds the execution session", holds: true },
    { kind: "pendant", text: "rosie-pendant-v5 (steamdeck-pendant) · from 192.168.1.113 · seen 0 s ago", holds: false },
    { kind: "unknown", text: "Unknown client · from 10.0.0.9", holds: false },
  ]);
});

test("an older server without the lists, or garbage, lists nobody", () => {
  assert.deepEqual(maintenanceSessionLines({ active: true }, NOW), []);
  assert.deepEqual(maintenanceSessionLines(null, NOW), []);
  assert.deepEqual(maintenanceSessionLines({ clients: "x", machine: 3 }, NOW), []);
  assert.equal(maintenanceAgo("2026-10-07T19:00:30Z", NOW), "2 h ago");
  assert.equal(maintenanceAgo(undefined, NOW), "");
});

test("the cell's rt-control grant holder follows the machine, by name and principal", () => {
  const lines = maintenanceSessionLines({
    machine: { cell: "example-arm-1420", armed: false, lease_valid: false },
    cell: { active: true, reachable: true, holder: { controller: "offline-programming:server-1a2b3c", remote: true, identity: "steamdeck-pendant-olp", generation: 4, since: "2026-10-07T20:57:30Z" } },
    clients: [],
  }, NOW);
  assert.deepEqual(lines, [
    { kind: "cell", text: "Cell example-arm-1420 · disarmed", holds: false },
    { kind: "grant", text: "Servo host grant held by offline-programming:server-1a2b3c · remote steamdeck-pendant-olp · since 3 min ago", holds: true },
  ]);
  assert.deepEqual(maintenanceSessionLines({ cell: { holder: { controller: "rtctl", remote: false } } }, NOW),
    [{ kind: "grant", text: "Servo host grant held by rtctl · local socket", holds: true }]);
  assert.deepEqual(maintenanceSessionLines({ cell: { active: false } }, NOW), []);
});
