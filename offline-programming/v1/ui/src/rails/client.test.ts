import assert from "node:assert/strict";
import test from "node:test";
import { planWeldTrajectory, type PlanRequestBody } from "./client.ts";

test("weld planning cancellation aborts its HTTP request", async () => {
  const original = globalThis.fetch;
  const cancellation = new AbortController();
  globalThis.fetch = async (url, options) => {
    assert.equal(url, "/api/offline-programming/v1/weld-plan");
    assert.equal(options?.signal, cancellation.signal);
    return new Promise<Response>((_resolve, reject) => {
      options!.signal!.addEventListener("abort", () => reject(options!.signal!.reason), { once: true });
    });
  };
  try {
    const pending = planWeldTrajectory({ program: {} } as PlanRequestBody, cancellation.signal);
    cancellation.abort();
    await assert.rejects(pending, { name: "AbortError" });
  } finally { globalThis.fetch = original; }
});
