import assert from "node:assert/strict";
import test from "node:test";
import { CapabilitiesDigest } from "../../../../../rt-core/clients/ts/rt_control_api_generated.ts";
import {
  DISCOVERED_CELLS_PATH, DiscoveredCellsClient, DiscoveryModel, discoveredRows, discoveryStatusText, validateDiscoveryResult,
  type DiscoveryResult,
} from "./discoveredCells.ts";

const OTHER_DIGEST = "b1e28f06".padEnd(64, "0");
const PIN = "87183b30".padEnd(64, "a");

function result(): DiscoveryResult {
  return {
    schema: "offline-programming.discovered-cells.v1",
    enabled: true,
    searched_at: "2026-10-08T19:00:00Z",
    sources: [{ name: "mdns", interfaces: ["wlP1p1s0"], found: 2 }],
    servers: [{
      instance: "steamdeck-pendant-olp-abcdef", hostname: "steamdeck-pendant", address: "192.168.50.77", port: 8794, reach: "loopback",
      build_id: "git:6d1fd9e4", started_at: "2026-10-08T07:00:00Z", maintenance: false, source: "mdns", health: "not_probed",
      cells: [
        { cell_id: "example-arm-1420", label: "rosie_1420_v1", model: "rosie_1420_v1", mode: "real", configuration_sha256: PIN, capabilities_digest: CapabilitiesDigest, selected: true,
          here: { configured: true, pin: "match", known_manifest: true, commissioned_address: "https://127.0.0.1:18443", commissioned_host: "127.0.0.1" } },
      ],
    }, {
      instance: "bench-relay-olp-123456", hostname: "bench-relay", address: "192.168.50.140", port: 8794, reach: "network",
      build_id: "git:0123abcd", started_at: "2026-10-08T07:00:00Z", maintenance: true, source: "mdns", health: "ok",
      cells: [
        { cell_id: "example-bench-1400", label: "Bench", model: "bench_nine_motors_1to1", mode: "real", configuration_sha256: PIN, capabilities_digest: OTHER_DIGEST,
          here: { configured: false, pin: "differs", known_manifest: true, commissioned_address: "https://127.0.0.1:28443", commissioned_host: "127.0.0.1", reason: "Not in this server's cell catalogue" } },
        { cell_id: "made-up", label: "?", model: "rosie_1420_v1", mode: "simulation",
          here: { configured: false, pin: "unknown", known_manifest: false, reason: "No manifest for this cell id in rt-core/config/cells" } },
      ],
    }],
  };
}

test("the client asks for a new search only on refresh and validates every field", async () => {
  const urls: string[] = [];
  const client = new DiscoveredCellsClient(async (url, init) => {
    urls.push(String(url));
    assert.equal(init?.redirect, "error");
    return Response.json(result());
  });
  await client.search(false);
  await client.search(true);
  assert.deepEqual(urls, [DISCOVERED_CELLS_PATH, `${DISCOVERED_CELLS_PATH}?refresh=1`]);

  const bad = result();
  (bad.servers[1]!.cells[0] as { capabilities_digest: string }).capabilities_digest = "not-a-digest";
  assert.throws(() => validateDiscoveryResult(bad), /servers\[1\]\.cells\[0\]\.digest/);
  const noHere = result() as unknown as { servers: { cells: { here?: unknown }[] }[] };
  delete noHere.servers[0]!.cells[0]!.here;
  assert.throws(() => validateDiscoveryResult(noHere), /cells\[0\]\.here/);

  const deadProxy = new DiscoveredCellsClient(async () => new Response("", { status: 500 }));
  await assert.rejects(deadProxy.search(false), /OLP API unreachable: HTTP 500/);
  const olderServer = new DiscoveredCellsClient(async () => new Response("404 page not found\n", { status: 404 }));
  await assert.rejects(olderServer.search(false), /no network search: it runs a build from before discovered-cells/);
});

test("rows flag incompatible and unverified cells, and Add is offered only for a commissioned binding known here", () => {
  const rows = discoveredRows(result());
  assert.deepEqual(rows.map((r) => r.cellId), ["example-arm-1420", "example-bench-1400", "made-up"]);
  for (const row of rows) assert.ok(row.badges.some((b) => b.text === "Unverified"), `${row.cellId} must say it is unverified`);

  const [configured, bench, unknown] = rows;
  assert.equal(configured!.compatibility, "compatible");
  assert.equal(configured!.addable, false);
  assert.ok(configured!.badges.some((b) => b.text === "Configured here"));
  assert.ok(configured!.badges.some((b) => b.text === "In use on steamdeck-pendant"));

  assert.equal(bench!.compatibility, "incompatible");
  assert.ok(bench!.badges.some((b) => b.kind === "bad" && b.text === "Incompatible"));
  assert.ok(bench!.badges.some((b) => b.kind === "bad" && b.text === "Pin differs"));
  assert.ok(bench!.badges.some((b) => b.text === "Maintenance"));
  assert.equal(bench!.addable, true);
  // The form is filled from this server's commissioning, not the advertising host's address.
  assert.deepEqual(bench!.prefill, { id: "example-bench-1400", host: "127.0.0.1", address: "https://127.0.0.1:28443", credentialRef: "example-bench-1400" });
  assert.ok(!JSON.stringify(bench!.prefill).includes("192.168.50.140"));

  assert.equal(unknown!.compatibility, "unknown");
  assert.equal(unknown!.addable, false);
  assert.equal(unknown!.prefill, null);
  assert.match(unknown!.addReason, /No manifest/);
});

test("status text says when search is off, what was searched and what failed", () => {
  assert.match(discoveryStatusText({ result: { schema: "offline-programming.discovered-cells.v1", enabled: false, reason: "Network search is off on this server: set OFFLINE_PROGRAMMING_DISCOVERY_INTERFACES", sources: [], servers: [] }, error: null, searching: false }),
    /OFFLINE_PROGRAMMING_DISCOVERY_INTERFACES/);
  const text = discoveryStatusText({ result: result(), error: null, searching: false });
  assert.match(text, /^3 cells on 2 OLP servers via mDNS on wlP1p1s0/);
  const failed = result();
  failed.servers = [];
  failed.sources = [{ name: "mdns", interfaces: ["wlan0"], found: 0, error: "browse socket: permission denied" }];
  assert.match(discoveryStatusText({ result: failed, error: null, searching: false }), /No OLP servers found .* mdns: browse socket: permission denied/);
});

test("the model coalesces overlapping refreshes and keeps the last good result on an error", async () => {
  let calls = 0;
  let fail = false;
  const model = new DiscoveryModel(new DiscoveredCellsClient(async () => {
    calls++;
    if (fail) return new Response("", { status: 502 });
    return Response.json(result());
  }));
  let changes = 0;
  model.onChange(() => changes++);
  await Promise.all([model.refresh(true), model.refresh(true)]);
  assert.equal(calls, 1);
  assert.equal(model.rows().length, 3);
  assert.ok(changes >= 2);
  fail = true;
  await model.refresh(true);
  assert.equal(model.rows().length, 3);
  assert.match(model.statusText(), /OLP API unreachable/);
});
