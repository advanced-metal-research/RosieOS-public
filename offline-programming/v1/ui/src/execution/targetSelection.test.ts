import assert from "node:assert/strict";
import test from "node:test";
import { DenseExecutionClient, type DenseStatus } from "./denseClient.ts";
import { cartesianJogAPI } from "./cartesianJog.ts";
import { jointJogAPI, JointJogHold } from "./jointJog.ts";

test("First target_changed refreshes identity, retires the owned hold and never retries a move", async () => {
  const calls: { url: string; init?: RequestInit }[] = [];
  const old = { cell_id: "first", selection_generation: "1" } as DenseStatus["target"];
  const next = { cell_id: "second", selection_generation: "2" } as DenseStatus["target"];
  let changed = false, stops = 0;
  const client = new DenseExecutionClient((async (url, init) => {
    calls.push({ url: String(url), init });
    if (String(url).endsWith("/status")) return Response.json({ backend: "rt_core", target: next });
    if (changed && !String(url).endsWith("/stop") && new Headers(init?.headers).get("X-RT-Target-Generation") === "1") return Response.json({ error: "target_changed" }, { status: 409 });
    return Response.json({ revision: 0, active: false });
  }) as typeof fetch);
  client.observeTarget(old);
  const api = cartesianJogAPI(() => "base", client.fetcher);
  const hold = new JointJogHold(api, () => {}, { every: () => 1 as unknown as ReturnType<typeof setInterval>, clear: () => { stops++; } });
  client.onSelectionChanged(status => { if (!status) void hold.end(); });
  await hold.begin("first", "s", 0, 1, .1, old);
  assert.equal(hold.active, true);
  changed = true;
  await assert.rejects(client.arm(true), /target_changed|arm failed/);
  assert.equal(hold.active, false); assert.equal(stops, 1);
  assert.equal(calls.filter(c => c.url.endsWith("/status")).length, 1);
  assert.equal(calls.filter(c => c.url.endsWith("/arm")).length, 1);
  const network = calls.length;
  for (let i = 0; i < 5; i++) {
    await assert.rejects(api.state(old));
    await assert.rejects(jointJogAPI(client.fetcher).state(old));
  }
  assert.equal(calls.length, network);
  await client.heartbeat("new-session");
  assert.equal(new Headers(calls.at(-1)!.init?.headers).get("X-RT-Target-Generation"), "2");
  // Explicit inhibition remains reachable even if its browser identity is old.
  client.observeTarget(old); await client.stop();
  assert.ok(calls.at(-1)!.url.endsWith("/stop"));
});

test("A failed selection refresh blocks obsolete traffic until normal status polling recovers", async () => {
  const calls: string[] = [];
  const client = new DenseExecutionClient((async url => {
    calls.push(String(url));
    return String(url).endsWith("/status") ? new Response("", { status: 503 }) : Response.json({ error: "target_changed" }, { status: 409 });
  }) as typeof fetch);
  client.observeTarget({ cell_id: "old", selection_generation: "3" } as DenseStatus["target"]);
  await assert.rejects(client.arm(true));
  for (let i = 0; i < 5; i++) await assert.rejects(client.arm(true));
  assert.equal(calls.length, 2);
});

test("A preview tab also resynchronizes once when another window connects a machine", async () => {
  const calls: string[] = [];
  const client = new DenseExecutionClient((async url => {
    calls.push(String(url));
    if (String(url).endsWith("/status")) return Response.json({ backend: "rt_core", target: { cell_id: "bench", selection_generation: "9" } });
    return Response.json({ error: "target_changed" }, { status: 409 });
  }) as typeof fetch);
  let target = "";
  client.onSelectionChanged(status => { if (status?.target) target = status.target.cell_id; });
  const preview = "/api/offline-programming/v1/local-simulator/jog/start";
  await client.fetcher(preview, { method: "POST" });
  await client.fetcher(preview, { method: "POST" });
  assert.equal(target, "bench"); assert.equal(calls.length, 2);
});

// The machine catalogue and other identity-less reads are never refused
// locally: after a preview tab's identity-less POST was refused target_changed,
// the next catalogue GET still reaches the server, so the tab recovers instead
// of showing "catalogue could not be read" (2026-09-16, LA bench).
test("Identity-less reads pass the stale-tab filter after an identity-less refusal", async () => {
  const calls: string[] = [];
  const client = new DenseExecutionClient((async url => {
    calls.push(String(url));
    if (String(url).endsWith("/status")) return Response.json({ backend: "rt_core", target: { cell_id: "bench", selection_generation: "9" } });
    if (String(url).endsWith("/targets")) return Response.json({ targets: [{ cell_id: "bench" }], selected: null });
    return Response.json({ error: "target_changed" }, { status: 409 });
  }) as typeof fetch);
  await client.fetcher("/api/offline-programming/v1/local-simulator/jog/start", { method: "POST" });
  const catalogue = await client.fetcher("/api/offline-programming/v1/targets", { method: "GET" });
  assert.equal(catalogue.status, 200);
  assert.equal(calls.filter(c => c.endsWith("/targets")).length, 1, "the catalogue read reached the server");
  const again = await client.fetcher("/api/offline-programming/v1/local-simulator/jog/start", { method: "POST" });
  assert.equal(again.status, 409, "the identity-less mutation stays refused locally");
  assert.equal(calls.filter(c => c.endsWith("/jog/start")).length, 1);
});
