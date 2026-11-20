import assert from "node:assert/strict";
import test from "node:test";
import { DenseExecutionClient } from "./denseClient.ts";

test("Clear faults posts only a target-bound reset and retains a policy refusal", async () => {
  const calls: { url: string; init?: RequestInit }[] = [];
  const client = new DenseExecutionClient(async (url, init) => {
    calls.push({ url: String(url), init });
    return Response.json({ accepted: false, error: "restart_required: pdo_mapping_mismatch", status: { backend: "rt_core", available: false } });
  });
  client.observeTarget({ selection_generation: "7", cell_id: "rosie_1420_v1", model_id: "rosie_1420_v1", label: "Robot", backend: "ethercat", host: "cell" });
  const result = await client.clearFaults();
  assert.equal(calls.length, 1);
  assert.ok(calls[0]!.url.endsWith("/dense-execution/reset-fault"));
  assert.equal(calls[0]!.init?.method, "POST");
  assert.deepEqual(JSON.parse(String(calls[0]!.init?.body)), {});
  const headers = new Headers(calls[0]!.init?.headers);
  assert.equal(headers.get("X-RT-Target-Cell"), "rosie_1420_v1");
  assert.equal(headers.get("X-RT-Target-Generation"), "7");
  assert.equal(result.accepted, false);
  assert.equal(result.error, "restart_required: pdo_mapping_mismatch");
});
