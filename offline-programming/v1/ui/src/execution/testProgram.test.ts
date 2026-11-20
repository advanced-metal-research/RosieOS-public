import assert from "node:assert/strict";
import test from "node:test";
import { setImmediate } from "node:timers/promises";
import { createDogBoxProject } from "../examples/dog-box.ts";
import { DenseExecutionClient, normalizeDenseCapabilities, normalizeDenseStatus } from "./denseClient.ts";
import { DensePanel } from "./densePanel.ts";
import { CellTargetsClient, type CellTarget } from "../targets/cells.ts";
import { CapabilitiesDigest, ContractVersion } from "../../../../../rt-core/clients/ts/rt_control_api_generated.ts";
import {
  deriveTestProgramControls, encodeBase64, normalizeTestProgram, pickTestProgramFiles,
  testProgramLabel, testProgramLoadIdentity, type DenseTestProgram,
} from "./testProgram.ts";

const DIGEST = "sha256:" + "fa26306fa5bf".padEnd(64, "0");
const serverProgram = {
  trajectory_digest: DIGEST, plan_id: "tuningreport-inertia-J4:1a4af196baf0", program_id: "tuningreport-inertia-J4",
  program_digest: "sha256:" + "5".repeat(64), manifest_revision: 1, plan_revision: 1, source: "test_program",
  file_name: "inertia-j4-fa26306fa5bf.rdt", file_sha256: "sha256:" + "54fa2657b295".padEnd(64, "0"), bytes: 141998,
  uploaded_at: "2026-10-09T23:30:00Z", robot_cell: { model_id: "rosie_1420_v1", machine_configuration_sha256: "sha256:" + "6df512948230".padEnd(64, "0") },
  start_rad: [0, 0.35, -0.17, 0.2, 0.52, -0.3], trajectory: { total_duration_s: 24.4, total_sample_count: 2441 },
  provenance: { file_name: "plan.json", schema: "rosie.tuningreport.inertia-plan.v1", generated: "2026-10-09T23:00:00Z", joint: "J4",
    duration_s: 24.4, window_deg: [-15.9, 15.9], accel_levels_deg_s2: [200, 400], configuration_sha256: "6df512948230".padEnd(64, "0") },
};

function program(overrides: Record<string, unknown> = {}): DenseTestProgram {
  const p = normalizeTestProgram({ ...serverProgram, ...overrides });
  assert.ok(p);
  return p;
}

test("a server program normalises, and only a test_program source does", () => {
  const p = program();
  assert.equal(p.trajectoryDigest, DIGEST);
  assert.deepEqual(p.startRad, [0, 0.35, -0.17, 0.2, 0.52, -0.3]);
  assert.equal(p.provenance?.joint, "J4");
  assert.deepEqual(p.provenance?.accelLevelsDegS2, [200, 400]);
  assert.equal(normalizeTestProgram({ ...serverProgram, source: "" }), null, "an unlabelled program is not shown as a test program");
  assert.deepEqual(normalizeDenseCapabilities({ backend: "rt_core", test_programs: { enabled: true, max_bytes: 16 << 20 } }).testPrograms, { enabled: true, maxBytes: 16 << 20 });
  assert.equal(normalizeDenseCapabilities({ backend: "rt_core" }).testPrograms, null, "an older server, or one with test programs off, reads as off");
});

test("the label cannot be read as a planner program and says where the program came from", () => {
  const label = testProgramLabel(program());
  assert.match(label, /^TEST PROGRAM, not from the planner · inertia-j4-fa26306fa5bf\.rdt · J4 only, 24\.4 s, 200\/400 deg\/s², -15\.9\.\.15\.9 deg/);
  assert.match(label, /trajectory fa26306fa5bf · file 54fa2657b295 · configuration 6df512948230 · rosie\.tuningreport\.inertia-plan\.v1, generated 2026-10-09T23:00:00Z$/);
  assert.match(testProgramLabel(program({ provenance: null })), /no plan\.json: provenance unknown$/);
});

test("Load names the uploaded identity and the test-program source", () => {
  assert.deepEqual(testProgramLoadIdentity(program()), {
    trajectory_digest: DIGEST, plan_id: serverProgram.plan_id, program_id: serverProgram.program_id, program_digest: serverProgram.program_digest,
    manifest_revision: 1, plan_revision: 1, source: "test_program",
  });
});

test("file choice: one .rdt, at most one plan.json, nothing else, within the server's bound", () => {
  const f = (name: string, size = 10) => ({ name, size });
  assert.deepEqual(pickTestProgramFiles([f("a.rdt"), f("plan.json")], 100), { rdt: f("a.rdt"), plan: f("plan.json") });
  assert.deepEqual(pickTestProgramFiles([f("A.RDT")], 100), { rdt: f("A.RDT"), plan: null });
  for (const [files, error] of [
    [[f("plan.json")], /Choose the program's \.rdt/],
    [[f("a.rdt"), f("b.rdt")], /one \.rdt program at a time/],
    [[f("a.rdt"), f("p.json"), f("q.json")], /at most one plan\.json/],
    [[f("a.rdt"), f("notes.txt")], /Only the \.rdt program and its plan\.json/],
    [[f("a.rdt", 101)], /101 bytes; this server takes at most 100/],
  ] as const) {
    const picked = pickTestProgramFiles(files, 100);
    assert.ok("error" in picked && error.test(picked.error), JSON.stringify(picked));
  }
});

test("base64 matches Node's for every byte value and a multi-chunk program", () => {
  const bytes = new Uint8Array(0x8000 * 2 + 7).map((_, i) => (i * 37) & 0xff);
  assert.equal(encodeBase64(bytes), Buffer.from(bytes).toString("base64"));
});

function nativeStatus(session: Record<string, unknown> | null, armed: boolean, positions = serverProgram.start_rad) {
  return normalizeDenseStatus({
    backend: "rt_core", available: true, session,
    rt_core: { description: { axes: positions.map((_, i) => ({ index: i, id: `J${i + 1}`, position_unit: "rad", completion_tolerance: 0.0006 })) }, status: {
      time_ns: 1, adapter_incarnation: "a", daemon_incarnation: "d", execution: { state: "idle" }, core: { armed: armed ? 1 : 0 },
      axes: positions.map(q => ({ home_valid: true, logical_valid: true, readiness: "ready", logical_position: q })),
    } },
  });
}

test("controls: hidden unless on for a native cell; Load refused while running; Play only when loaded, armed and at the start", () => {
  const p = program();
  const base = { enabled: true, targetSelected: true, cellPending: false, native: true, inFlight: false, daemonReachable: true, program: p, atStart: true };
  const loaded = nativeStatus({ id: "s", state: "loaded", trajectory_digest: DIGEST, source: "test_program", lease: { held: true } }, true);
  assert.equal(deriveTestProgramControls({ ...base, enabled: false, status: loaded }).visible, false);
  assert.equal(deriveTestProgramControls({ ...base, enabled: null, status: loaded }).visible, false, "not asked yet reads as off");
  assert.equal(deriveTestProgramControls({ ...base, native: false, status: loaded }).visible, false);
  const ok = deriveTestProgramControls({ ...base, status: loaded });
  assert.equal(ok.visible, true);
  assert.equal(ok.load.disabled, false);
  assert.equal(ok.play.disabled, false, ok.play.title);
  assert.match(ok.label, /^TEST PROGRAM, not from the planner .* · loaded$/);
  const playing = nativeStatus({ id: "s", state: "playing", trajectory_digest: DIGEST, source: "test_program", lease: { held: true } }, true);
  assert.deepEqual([deriveTestProgramControls({ ...base, status: playing }).load.disabled, deriveTestProgramControls({ ...base, status: playing }).load.title],
    [true, "A program is running; STOP it before loading another"]);
  const planner = nativeStatus({ id: "s", state: "loaded", trajectory_digest: DIGEST, lease: { held: true } }, true);
  assert.equal(deriveTestProgramControls({ ...base, status: planner }).play.title, "Load the test program first", "a planner session with the same digest is not this upload");
  assert.equal(deriveTestProgramControls({ ...base, status: nativeStatus({ id: "s", state: "loaded", trajectory_digest: DIGEST, source: "test_program", lease: { held: true } }, false) }).play.title, "Arm the robot first");
  assert.match(deriveTestProgramControls({ ...base, atStart: false, status: loaded }).play.title, /not at the program's start/);
  assert.equal(deriveTestProgramControls({ ...base, daemonReachable: false, status: loaded }).play.disabled, true);
});

test("the client uploads the bytes and plan, loads with the source, and reads the session label", async () => {
  const calls: { path: string; body: Record<string, unknown> }[] = [];
  const client = new DenseExecutionClient(async (url, init) => {
    const path = String(url);
    calls.push({ path, body: init?.body ? JSON.parse(String(init.body)) : {} });
    if (path.endsWith("/test-program")) return Response.json({ ok: true, program: serverProgram, checks: ["start within completion_tolerance"] });
    return Response.json({ ok: true, session_id: "ds-1", status: { backend: "rt_core", available: true, session: { id: "ds-1", state: "loaded", trajectory_digest: DIGEST, source: "test_program", test_program: { file_name: "inertia-j4-fa26306fa5bf.rdt", file_sha256: serverProgram.file_sha256 } } } });
  });
  const bytes = new Uint8Array([0, 1, 2, 250, 255]);
  const uploaded = await client.uploadTestProgram({ fileName: "inertia-j4-fa26306fa5bf.rdt", bytes, planJSON: { schema: "s" }, planFileName: "plan.json" });
  assert.equal(uploaded.program.fileName, "inertia-j4-fa26306fa5bf.rdt");
  assert.deepEqual(calls[0]!.body, { file_name: "inertia-j4-fa26306fa5bf.rdt", program_base64: "AAEC+v8=", plan_json: { schema: "s" }, plan_file_name: "plan.json" });
  const loaded = await client.load(testProgramLoadIdentity(uploaded.program));
  assert.equal(calls[1]!.path.endsWith("/load"), true);
  assert.equal(calls[1]!.body.source, "test_program");
  assert.equal(loaded.status.session?.source, "test_program");
  assert.equal(loaded.status.session?.testProgram?.fileName, "inertia-j4-fa26306fa5bf.rdt");
  await client.uploadTestProgram({ fileName: "a.rdt", bytes, planJSON: null, planFileName: null });
  assert.equal("plan_json" in calls[2]!.body, false, "no plan.json is sent as none, not as null");
});

// The panel's DOM stand-in, as densePanel.test.ts uses: every selector
// resolves, events dispatch, rendered properties are observable.
class PanelElement extends EventTarget {
  innerHTML = ""; className = ""; textContent = ""; title = ""; value = ""; hidden = false; disabled = false;
  dataset: Record<string, string> = {}; style = { width: "" }; files: unknown[] = [];
  classList = { add: (..._n: string[]) => {}, remove: (..._n: string[]) => {}, toggle: (_n: string, _f?: boolean) => false };
  options: PanelElement[] = []; nodes = new Map<string, PanelElement>();
  querySelector(selector: string): PanelElement {
    let node = this.nodes.get(selector);
    if (!node) { node = new PanelElement(); this.nodes.set(selector, node); }
    return node;
  }
  append(...options: PanelElement[]): void { this.options.push(...options); }
  replaceChildren(): void { this.options = []; this.nodes.clear(); this.textContent = ""; }
  setAttribute(): void {}
  scrollIntoView(): void {}
  focus(): void {}
  remove(): void {}
  click(): void { assert.equal(this.disabled, false, "control must be enabled"); this.dispatchEvent(new Event("click")); }
}

async function testProgramPanel(t: import("node:test").TestContext, enabled: boolean) {
  t.mock.timers.enable({ apis: ["setTimeout", "setInterval", "Date"], now: 100_000 });
  for (const [name, value] of Object.entries({ window: new EventTarget(), document: Object.assign(new EventTarget(), { createElement: () => new PanelElement() }), sessionStorage: { getItem: () => "", setItem: () => {} } })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  Object.assign(window, { setInterval, clearInterval, setTimeout, clearTimeout });
  const cell: CellTarget = { cell_id: "machine", label: "Machine", role: "bench", models: ["rosie_1420_v1"], requested_mode: "simulation", backend: "simulation", host: "localhost" };
  const description = { contract_version: ContractVersion, capabilities_digest: CapabilitiesDigest, machine_sha256: "a".repeat(64), deployment_sha256: "b".repeat(64),
    axes: serverProgram.start_rad.map((_, index) => ({ index, id: `J${index + 1}`, position_unit: "rad", min_position: -3, max_position: 3, completion_tolerance: 0.0006 })), capabilities: [] };
  const status = { backend: "rt_core", available: true, target: { ...cell, model_id: "rosie_1420_v1", selection_generation: "1" },
    session: { id: "authority", state: "idle", lease: { held: true } } as Record<string, unknown>, rt_core: { description, status: {
      time_ns: 1_000_000, adapter_incarnation: "adapter", daemon_incarnation: "daemon", execution: { state: "idle" }, core: { armed: 1 },
      axes: serverProgram.start_rad.map(q => ({ home_valid: true, logical_valid: true, readiness: "ready", logical_position: q, calibration_valid: true, position_state: "trusted" })),
    } } };
  const posts: { path: string; body: Record<string, unknown> }[] = [];
  const client = new DenseExecutionClient(async (url, init) => {
    const path = String(url);
    if (init?.method === "POST") posts.push({ path, body: JSON.parse(String(init.body)) });
    if (path.endsWith("/test-program")) {
      assert.equal(init?.method, "POST", "the panel reads the switch from capabilities, never with its own request");
      return Response.json({ ok: true, program: serverProgram, checks: ["start within completion_tolerance of the measured pose on every axis"] });
    }
    if (path.endsWith("/load")) {
      status.session = { id: "ds-1", state: "loaded", trajectory_digest: DIGEST, source: "test_program", lease: { held: true } };
      return Response.json({ ok: true, session_id: "ds-1", status });
    }
    if (path.endsWith("/play")) { status.session = { ...status.session, state: "playing" }; return Response.json(status); }
    if (path.endsWith("/capabilities")) return Response.json({ backend: "rt_core", available: true, daemon_reachable: true, cell: cell.cell_id, rt_core: { description },
      ...(enabled ? { test_programs: { enabled: true, max_bytes: 16 << 20 } } : {}) });
    if (path.endsWith("/state") || path.includes("/jog/")) return Response.json({ active: false, revision: 0 });
    return Response.json(status);
  });
  const cells = new CellTargetsClient(async () => Response.json({ targets: [cell], selected: { cell_id: cell.cell_id, model_id: "rosie_1420_v1" } }));
  const messages: string[] = [];
  const panel = new DensePanel({ client, cells, getProject: () => createDogBoxProject(), highlightNode: () => {}, status: text => { messages.push(text); },
    onLockChange: () => {}, onTargetChange: () => {}, onRobotPose: () => {}, onRobotMotion: () => {}, wantsFastPoll: () => false });
  const host = new PanelElement(); panel.mount(host as unknown as HTMLElement, host as unknown as HTMLElement);
  await setImmediate(); t.mock.timers.tick(0); await setImmediate();
  const poll = async () => { status.rt_core.status.time_ns += 1_000_000; t.mock.timers.tick(250); await setImmediate(); await setImmediate(); };
  await poll();
  return { panel, host, posts, status, messages, poll };
}

test("the Machine panel shows no test-program action when the server has them off", async t => {
  const h = await testProgramPanel(t, false);
  assert.equal(h.host.querySelector("#dense-test-program").hidden, true);
});

test("Load test program uploads the files, then the ordinary Load; Play waits for the loaded session and plays it", async t => {
  const h = await testProgramPanel(t, true);
  assert.equal(h.host.querySelector("#dense-test-program").hidden, false);
  assert.equal(h.host.querySelector("#dense-test-load").disabled, false, h.host.querySelector("#dense-test-load").title);
  assert.equal(h.host.querySelector("#dense-test-play").disabled, true, "nothing loaded yet");
  const rdt = new Uint8Array([1, 2, 3, 4]);
  const file = (name: string, body: Uint8Array | string) => ({ name, size: typeof body === "string" ? body.length : body.length,
    arrayBuffer: async () => (typeof body === "string" ? new TextEncoder().encode(body) : body).buffer, text: async () => String(body) }) as unknown as File;
  await h.panel.loadTestProgram([file("inertia-j4-fa26306fa5bf.rdt", rdt), file("plan.json", JSON.stringify({ schema: "rosie.tuningreport.inertia-plan.v1" }))]);
  assert.deepEqual(h.posts.map(p => p.path.split("/").at(-1)), ["test-program", "load"]);
  assert.equal(h.posts[0]!.body.program_base64, "AQIDBA==");
  assert.deepEqual(h.posts[0]!.body.plan_json, { schema: "rosie.tuningreport.inertia-plan.v1" });
  assert.equal(h.posts[1]!.body.source, "test_program");
  assert.equal(h.posts[1]!.body.trajectory_digest, DIGEST);
  assert.match(h.messages.at(-1)!, /^Robot: loaded TEST PROGRAM, not from the planner · inertia-j4-fa26306fa5bf\.rdt .* · start within completion_tolerance/);
  await h.poll();
  assert.match(h.host.querySelector("#dense-test-label").textContent, /^TEST PROGRAM, not from the planner .* · loaded$/);
  const play = h.host.querySelector("#dense-test-play");
  assert.equal(play.disabled, false, play.title);
  assert.equal(h.host.querySelector("#dense-play").disabled, true, "the planner's Play does not play a test program");
  // Off the start by more than the axis tolerance: Play is withheld.
  h.status.rt_core.status.axes[3]!.logical_position += 0.001;
  await h.poll();
  assert.match(play.title, /not at the program's start/);
  h.status.rt_core.status.axes[3]!.logical_position -= 0.001;
  await h.poll();
  play.click(); await setImmediate();
  assert.deepEqual(h.posts.at(-1), { path: "/api/offline-programming/v1/dense-execution/play", body: { session_id: "ds-1" } });
  await h.poll();
  assert.equal(h.host.querySelector("#dense-test-load").disabled, true, "no other Load while it plays");
  assert.equal(h.host.querySelector("#dense-stop").disabled, false, "STOP stays reachable");
});

test("a bad file choice is refused in the panel without a request", async t => {
  const h = await testProgramPanel(t, true);
  await h.panel.loadTestProgram([{ name: "plan.json", size: 2 } as unknown as File]);
  assert.equal(h.posts.length, 0);
  assert.match(h.messages.at(-1)!, /Load test program: Choose the program's \.rdt file/);
});
