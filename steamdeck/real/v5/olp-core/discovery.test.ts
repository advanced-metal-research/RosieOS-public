import assert from "node:assert/strict";
import test from "node:test";
import { CapabilitiesDigest } from "../../../../rt-core/clients/ts/rt_control_api_generated.ts";
import { discoveredCells } from "./discovery.ts";

const here = { configured: true, pin: "match", known_manifest: true, commissioned_address: "https://127.0.0.1:18443", commissioned_host: "127.0.0.1" };
const body = {
  schema: "offline-programming.discovered-cells.v1", enabled: true, sources: [{ name: "mdns", interfaces: ["wlan0"], found: 1 }],
  servers: [{
    instance: "example-relay-olp-3f2a1c", hostname: "example-relay", address: "192.168.50.121", port: 8794, reach: "loopback",
    build_id: "git:9dcf3158", started_at: "", maintenance: false, source: "mdns", health: "not_probed",
    cells: [
      { cell_id: "example-arm-1420", label: "rosie_1420_v1", model: "rosie_1420_v1", mode: "real", capabilities_digest: CapabilitiesDigest, here },
      { cell_id: "example-bench-1400", label: "Bench", model: "bench_nine_motors_1to1", mode: "real", capabilities_digest: "c".repeat(64),
        here: { ...here, configured: false, pin: "unknown" } },
    ],
  }],
};

test("the pendant lists discovered cells in OLP's words and flags an incompatible one", () => {
  const view = discoveredCells(body, "");
  assert.match(view.status, /^2 cells on 1 OLP server via mDNS on wlan0/);
  assert.deepEqual(view.lines.map((line) => [line.title, line.flagged]), [["rosie_1420_v1", false], ["Bench", true]]);
  assert.match(view.lines[0]!.badges, /Unverified · Compatible · Configured here · Pin matches/);
  assert.match(view.lines[1]!.badges, /Incompatible/);
});

test("the pendant says why there is no list", () => {
  assert.match(discoveredCells(null, "http_404").status, /no network search/);
  assert.match(discoveredCells(null, "olp_server_unreachable").status, /olp_server_unreachable/);
  assert.match(discoveredCells({ schema: "other" }, "").status, /Invalid discovered-cells answer: schema/);
  assert.deepEqual(discoveredCells(null, ""), { status: "Not searched yet.", lines: [] });
});
