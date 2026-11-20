import assert from "node:assert/strict";
import test from "node:test";
import { CLIENT_ID_STORAGE_KEY, controlFetch, tabClientId } from "./controlFetch.ts";
import { CLIENT_ID_HEADER, CONTROL_REFUSAL } from "./controlNotice.ts";

const ORIGIN = "http://127.0.0.1:5189";

function recorder(reply: () => Response) {
  const sent: { url: string; headers: Headers }[] = [];
  const inner = (async (input: RequestInfo | URL, init?: RequestInit) => {
    sent.push({ url: String(input), headers: new Headers(init?.headers) });
    return reply();
  }) as typeof fetch;
  return { sent, inner };
}

test("same-origin API requests carry the tab's id; others are untouched", async () => {
  const { sent, inner } = recorder(() => new Response("{}"));
  const wrapped = controlFetch(inner, { clientId: "tab-0123456789abcdef", origin: ORIGIN, onRefusal: () => {} });
  await wrapped("/api/offline-programming/v1/dense-execution/arm", { method: "POST", headers: { "X-RT-Target-Cell": "c" } });
  await wrapped(`${ORIGIN}/api/offline-programming/v1/maintenance`);
  await wrapped("/offline-programming/v1/ui/robots/rosie_1420_v1.urdf");
  await wrapped("https://elsewhere.example/api/x");
  assert.equal(sent[0]!.headers.get(CLIENT_ID_HEADER), "tab-0123456789abcdef");
  assert.equal(sent[0]!.headers.get("X-RT-Target-Cell"), "c", "the request's own headers are kept");
  assert.equal(sent[1]!.headers.get(CLIENT_ID_HEADER), "tab-0123456789abcdef");
  assert.equal(sent[2]!.headers.get(CLIENT_ID_HEADER), null);
  assert.equal(sent[3]!.headers.get(CLIENT_ID_HEADER), null);
});

test("a control refusal reaches the pop-up and the caller still gets the response", async () => {
  const refusal = { error: CONTROL_REFUSAL, detail: "in use", control: { state: "other" } };
  const { inner } = recorder(() => new Response(JSON.stringify(refusal), { status: 409 }));
  const heard: unknown[] = [];
  const wrapped = controlFetch(inner, { clientId: "tab-0123456789abcdef", origin: ORIGIN, onRefusal: (body) => heard.push(body) });
  const response = await wrapped("/api/offline-programming/v1/dense-execution/jog/begin", { method: "POST" });
  assert.equal(response.status, 409);
  assert.deepEqual(await response.json(), refusal);
  await new Promise((resolve) => setTimeout(resolve, 0));
  assert.deepEqual(heard, [refusal]);
  const other = recorder(() => new Response(JSON.stringify({ error: "target_changed" }), { status: 409 }));
  const quiet: unknown[] = [];
  await controlFetch(other.inner, { clientId: "tab-0123456789abcdef", origin: ORIGIN, onRefusal: (body) => quiet.push(body) })("/api/x", { method: "POST" });
  await new Promise((resolve) => setTimeout(resolve, 0));
  assert.deepEqual(quiet, []);
});

test("the tab keeps its id across reloads (sessionStorage) and makes one when storage is blocked", () => {
  const store = new Map<string, string>();
  const storage = { getItem: (k: string) => store.get(k) ?? null, setItem: (k: string, v: string) => void store.set(k, v) };
  const first = tabClientId(storage, () => "aaaaaaaaaaaaaaaa");
  assert.equal(first, "tab-aaaaaaaaaaaaaaaa");
  assert.equal(store.get(CLIENT_ID_STORAGE_KEY), first);
  assert.equal(tabClientId(storage, () => "bbbbbbbbbbbbbbbb"), first);
  const blocked = { getItem: () => { throw new Error("blocked"); }, setItem: () => { throw new Error("blocked"); } };
  assert.equal(tabClientId(blocked, () => "cccccccccccccccc"), "tab-cccccccccccccccc");
  assert.match(tabClientId(null), /^tab-[0-9a-f]{16}$/);
});
