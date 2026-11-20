import assert from "node:assert/strict";
import test from "node:test";
import { setImmediate } from "node:timers/promises";
import { createDogBoxProject } from "../examples/dog-box.ts";
import { DenseExecutionClient, DenseExecutionError, normalizeDenseStatus, type DenseLoadIdentity } from "./denseClient.ts";
import { DensePanel } from "./densePanel.ts";
import { machineStatePill } from "./machineFlow.ts";
import { CellTargetsClient, cellModelPresentation, renderCellModel, type CellTarget } from "../targets/cells.ts";
import { sessionModeStatusPill } from "../sessionStatusPills.ts";
import type { RecoverConfirmation } from "./recoverToZero.ts";
import { normalizeRunResult, type RunResult } from "./runProgram.ts";
import type { PressRefusal } from "./pressNotice.ts";

const noLegacy = { executionMode: "simulation" as const, robotLive: false, connectedArmed: false };
import { parseJogState } from "../simulation/live-jog.ts";
import { renderOperatorFeedback } from "../targets/operatorFeedback.ts";
import { CapabilitiesDigest, ContractVersion } from "../../../../../rt-core/clients/ts/rt_control_api_generated.ts";

// This DOM boundary keeps event dispatch and rendered controls observable in
// the existing Node harness without substituting the panel's action methods.
class PanelElement extends EventTarget {
  innerHTML = "";
  className = "";
  textContent = "";
  title = "";
  value = "";
  hidden = false;
  disabled = false;
  focused = false;
  scrolled = false;
  dataset: Record<string, string> = {};
  style = { width: "" };
  classList = { add: (..._names: string[]) => {}, remove: (..._names: string[]) => {}, toggle: (_name: string, _force?: boolean) => false };
  options: PanelElement[] = [];
  nodes = new Map<string, PanelElement>();

  querySelector(selector: string): PanelElement {
    let node = this.nodes.get(selector);
    if (!node) {
      node = new PanelElement();
      this.nodes.set(selector, node);
    }
    return node;
  }

  append(...options: PanelElement[]): void { this.options.push(...options); }
  replaceChildren(): void { this.options = []; this.nodes.clear(); this.textContent = ""; }
  setAttribute(_name: string, _value: string): void {}
  scrollIntoView(): void { this.scrolled = true; }
  focus(): void { this.focused = true; }
  remove(): void { /* No initial target options need removal in this fixture. */ }
  click(): void {
    assert.equal(this.disabled, false, "control must be enabled");
    this.dispatchEvent(new Event("click"));
  }
}

test("Stale selection recovery displays the re-check wording in the Machine panel", async t => {
  t.mock.timers.enable({ apis: ["setTimeout"] });
  for (const [name, value] of Object.entries({ window: new EventTarget(), document: Object.assign(new EventTarget(), { createElement: () => new PanelElement() }), sessionStorage: { getItem: () => "", setItem: () => undefined } })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  const target = { cell_id: "next", selection_generation: "2", model_id: "rosie_1420_v1", backend: "simulation", label: "Next", host: "localhost" };
  const client = new DenseExecutionClient(async url => String(url).endsWith("/arm")
    ? Response.json({ error: "target_changed" }, { status: 409 }) : Response.json({ backend: "rt_core", target }));
  const panel = new DensePanel({ client, getProject: () => createDogBoxProject(), highlightNode: () => {}, status: () => {}, onLockChange: () => {}, onTargetChange: () => {}, onRobotPose: () => {}, onRobotMotion: () => {}, wantsFastPoll: () => false });
  const host = new PanelElement(); panel.mount(host as unknown as HTMLElement, host as unknown as HTMLElement);
  await setImmediate();
  await assert.rejects(client.arm(true));
  const notice = host.querySelector("#dense-selection-notice");
  assert.equal(notice.hidden, false);
  assert.equal(notice.textContent, "Machine selection changed in another window; re-check before arming");
});

test("operator conditions render for dense and jog; Home blocks Run and Pause is absent", async (t) => {
  t.mock.timers.enable({ apis: ["setTimeout"] });
  for (const [name, value] of Object.entries({ window: new EventTarget(), document: Object.assign(new EventTarget(), { createElement: () => new PanelElement() }), sessionStorage: { getItem: () => "", setItem: () => undefined } })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  const description = { contract_version: ContractVersion, capabilities_digest: CapabilitiesDigest, machine_sha256: "a".repeat(64), deployment_sha256: "b".repeat(64),
    axes: [{ index: 0, id: "J1", position_unit: "rad", min_position: -5, max_position: 5 }], capabilities: [] };
  const observation = { description, reason: "soft_limit", status: {
    time_ns: 1000000, adapter_incarnation: "adapter", daemon_incarnation: "daemon", motion: { time_ns: 1000000, mode: 0, done: true },
    grant: { GrantActive: 1 }, execution: { state: "idle" },
    core: { armed: 0, commissioning_phase: 0, native_home_active_axis_mask: 0, safety_fault_mask: 4,
      axes: [{ anchor_reason: "anchor_identity_mismatch", anchor_identity_changed_field: "serial_no" }],
      recovery: { reason: "fault_persists", reset_sequence: 7, faults: [{ bit: 4, name: "drive_fault", axis_mask: 1, recovery: "reset" }] } },
    axes: [{ home_valid: false, logical_valid: true, readiness: "home_required", logical_position: 4, logical_target: 4, statusword: 8, position_counts: 100 }],
  } };
  const client = new DenseExecutionClient(async (url) => Response.json(String(url).endsWith("capabilities")
    ? { backend: "rt_core", available: true, rt_core: { description } }
    : { backend: "rt_core", available: true, rt_core: observation }));
  const refusals: string[] = [];
  const panel = new DensePanel({ client, refusal: text => refusals.push(text), getProject: () => createDogBoxProject(), highlightNode: () => undefined, status: () => undefined,
    onLockChange: () => undefined, onTargetChange: () => undefined, onRobotPose: () => undefined, onRobotMotion: () => undefined, wantsFastPoll: () => false });
  const host = new PanelElement(); panel.mount(host as unknown as HTMLElement, host as unknown as HTMLElement);
  await setImmediate();
  const target = host.querySelector("#dense-target"); target.value = "joint-daemon"; target.dispatchEvent(new Event("change"));
  t.mock.timers.tick(0); await setImmediate();
  const text = (element: PanelElement) => element.options.map(row => row.textContent).join("\n");
  const dense = text(host.querySelector("#dense-operator-feedback"));
  assert.match(dense, /Soft limit; correct the trajectory \[soft_limit\]/);
  assert.match(dense, /Approaching soft limit; lower 90.0%, upper 10.0%/);
  assert.match(dense, /anchor identity changed: serial_no/);
  assert.match(dense, /Reset fault available.*Fault persists; drive_fault \(reset\)/);
  assert.equal(host.querySelector("#dense-play").disabled, true);
  assert.equal(host.querySelector("#dense-play").title, "Home required: anchor identity changed: serial_no");
  assert.doesNotMatch(host.innerHTML, /id="dense-pause"/);
  // J1 needs Home but this description offers no native Home: the path says
  // so at the Home step and holds Arm and Drive behind it; STOP stays live.
  assert.equal(host.querySelector('[data-step="home"]').dataset.state, "blocked");
  assert.match(host.querySelector('[data-step-summary="home"]').textContent, /J1 report Home required, and this machine does not offer native Home/);
  assert.equal(host.querySelector('[data-step="arm"]').dataset.state, "pending");
  assert.equal(host.querySelector('[data-step-summary="arm"]').textContent, "Home first.");
  assert.equal(host.querySelector('[data-step="drive"]').dataset.state, "pending");
  assert.equal(host.querySelector("#dense-arm-on").disabled, true);
  assert.equal(host.querySelector("#dense-home").disabled, true);
  assert.equal(host.querySelector("#dense-stop").disabled, false, "STOP stays reachable");
  const jog = parseJogState({ running: true, rtCore: observation });
  const jogHost = new PanelElement(); renderOperatorFeedback(jogHost as unknown as HTMLElement, jog?.operatorFeedback ?? null);
  assert.equal(text(jogHost), dense);
  assert.equal(refusals.length, 1);
  assert.match(refusals[0]!, /Soft limit.*soft_limit/);
  assert.match(refusals[0]!, /Controller check: drive_fault; axes mask 0x1; recovery reset; outcome not reported/);
  panel.render();
  assert.equal(refusals.length, 1, "unchanged observations do not flood Logs");
});

function deferred<T>() {
  let resolve!: (value: T) => void;
  const promise = new Promise<T>((done) => { resolve = done; });
  return { promise, resolve };
}

/** Exercise actual panel events with a fixed status and controlled clock, so
 * an old armed response cannot hide an enabled motion control. */
async function modelPanel(t: import("node:test").TestContext, model: string, malformed = false, production = false, readTrajectory?: () => Promise<Uint8Array>, axisCount = 1, confirmRecover?: (confirmation: RecoverConfirmation) => Promise<boolean>) {
  t.mock.timers.enable({ apis: ["setTimeout", "setInterval", "Date"], now: 100_000 });
  for (const [name, value] of Object.entries({ window: new EventTarget(), document: Object.assign(new EventTarget(), { createElement: () => new PanelElement() }), sessionStorage: { getItem: () => "", setItem: () => {} } })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  const project = createDogBoxProject();
  if (production) project.program.executionMode = "production";
  Object.assign(window, { setInterval, clearInterval, setTimeout, clearTimeout });
  const cell: CellTarget = { cell_id: "machine", label: "Machine", role: "bench", models: [model], requested_mode: "simulation", backend: "simulation", host: "localhost" };
  const description = { contract_version: ContractVersion, capabilities_digest: CapabilitiesDigest, machine_sha256: "a".repeat(64), deployment_sha256: "b".repeat(64),
    axes: Array.from({ length: axisCount }, (_, index) => ({ index, id: `J${index + 1}`, position_unit: "rad", min_position: -3, max_position: 3, completion_tolerance: .000002 })), capabilities: [] };
  const status = { backend: "rt_core", available: true, target: { ...cell, model_id: model, selection_generation: "1" },
    session: { id: "s", state: "idle", lease: { held: true } }, rt_core: { description, status: {
      time_ns: 1_000_000, adapter_incarnation: "adapter", daemon_incarnation: "daemon", execution: { state: "idle" }, core: { armed: 1, configuration_sha256: "a".repeat(64), configuration_epoch: 1, home_epoch: 1 },
      axes: [{ home_valid: true, logical_valid: true, readiness: "ready", logical_position: 0 }],
    } } };
  const requests: string[] = [];
  const moves: unknown[] = [];
  const client = new DenseExecutionClient(async (url, init) => {
    const path = String(url); requests.push(path);
    if (path.endsWith("/move")) { moves.push(JSON.parse(String(init?.body))); return Response.json({ state: "done", axis: 0, reason: "", revision: 1 }); }
    if (path.endsWith("/capabilities")) return Response.json({ backend: "rt_core", available: true, daemon_reachable: true, cell: cell.cell_id, rt_core: { description } });
    if (path.endsWith("/state") || path.includes("/jog/")) return Response.json({ active: false, revision: 0 });
    return Response.json(status);
  });
  const cells = new CellTargetsClient(async () => Response.json(malformed ? { targets: [{ ...cell, models: [] }], selected: null }
    : { targets: [cell], selected: { cell_id: cell.cell_id, model_id: model } }));
  const presentations: string[] = [];
  const highlights: (string | null)[] = [];
  const notices: (PressRefusal | null)[] = [];
  const panel = new DensePanel({ client, cells, readTrajectory: readTrajectory ?? (async () => startTestBlob(model)), getProject: () => project, onCellModels: (_models, selected) => { presentations.push(selected); },
    highlightNode: id => { highlights.push(id); }, status: () => {}, onLockChange: () => {}, onTargetChange: () => {}, onRobotPose: () => {}, onRobotMotion: () => {}, wantsFastPoll: () => false, confirmRecover,
    onPressNotice: refusal => { notices.push(refusal); } });
  const host = new PanelElement(); panel.mount(host as unknown as HTMLElement, host as unknown as HTMLElement);
  await setImmediate(); t.mock.timers.tick(0); await setImmediate();
  return { panel, host, requests, moves, presentations, highlights, project, status, client, notices };
}

test("Go to plan start uses the first absolute sample and refuses a changed plan before motion", async t => {
  const header = new TextEncoder().encode(JSON.stringify({ schema: "robot.v4.dense-joint-trajectory.v1", trajectory_digest: "sha256:start", robot_cell: { model_id: "rosie_1420_v1", robot_description_sha256: "sha256:robot", machine_planning_calibration_sha256: "sha256:calibration" }, dt_s: .01, segments: [{ sample_count: 1 }] }));
  const bytes = new Uint8Array(4 + header.length + 153), view = new DataView(bytes.buffer);
  view.setUint32(0, header.length); bytes.set(header, 4); view.setFloat64(4 + header.length + 8, .00001);
  let pending: Promise<Uint8Array> = Promise.resolve(bytes);
  const h = await modelPanel(t, "rosie_1420_v1", false, false, () => pending);
  const robot = { modelId: "rosie_1420_v1", robotDescriptionSha256: "sha256:robot", machinePlanningCalibrationSha256: "sha256:calibration", machinePlanningCalibrationSource: "target" as const };
  Object.assign(h.status, { robot_cell: { configured: true, valid: true, model_id: robot.modelId, robot_description_sha256: robot.robotDescriptionSha256, machine_planning_calibration_sha256: robot.machinePlanningCalibrationSha256 } });
  h.project.denseTrajectory = { schema: "robot.v4.dense-joint-trajectory.v1", trajectoryDigest: "sha256:start", requestSha256: "", planId: "plan", programId: h.project.identity.id, programDigest: "", manifestRevision: 1, planRevision: 1, createdAt: "", dtS: .01, totalSampleCount: 1, totalDurationS: .01, bytes: bytes.length, segments: [], sourceProjectRevision: h.project.identity.revision, robot };
  t.mock.timers.tick(250); await setImmediate(); h.panel.render();
  delete h.project.denseTrajectory.robot; // Older saved records omitted this duplicate; the blob still has the binding.
  const button = h.host.querySelector("#dense-plan-start");
  button.click(); await setImmediate();
  assert.equal(h.moves.length, 1);
  assert.deepEqual((h.moves[0] as { axes: unknown[] }).axes, [{ axis: 0, degrees: 0, position_rad: .00001 }]);
  assert.match(h.host.querySelector("#dense-plan-start-result").textContent, /Move completed/);
  assert.equal(h.requests.some(p => p.endsWith("/load") || p.endsWith("/play") || p.endsWith("/home")), false);
  const delayed = deferred<Uint8Array>(); pending = delayed.promise;
  button.click(); h.project.identity.revision++;
  delayed.resolve(bytes); await setImmediate();
  assert.equal(h.moves.length, 1, "changed plan must not move");
  h.project.identity.revision--;
  pending = Promise.resolve(bytes);
  const axis = h.status.rt_core.status.axes[0]!;
  Object.assign(axis, { logical_position: .000011, calibration_valid: true, position_state: "trusted" });
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(button.textContent, "✓ At plan start", "fresh measured position is inside the machine's tolerance");
  assert.equal(button.disabled, true);
  for (const [calibration_valid, position_state] of [
    [false, "trusted"], [true, "recovering"], [true, "lost"], [true, "unverified"],
    [undefined, "trusted"], [true, undefined], [undefined, undefined],
  ] as const) {
    Object.assign(axis, { calibration_valid, position_state });
    h.status.rt_core.status.time_ns += 1_000_000;
    t.mock.timers.tick(250); await setImmediate();
    assert.equal(button.textContent, "Go to plan start", `unverified calibration/position cannot confirm plan start: ${calibration_valid}/${position_state}`);
  }
  Object.assign(axis, { calibration_valid: true, position_state: "trusted" });
  h.status.rt_core.status.axes[0]!.logical_position = .001;
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(button.textContent, "Go to plan start", "moving away clears confirmation");
  h.status.rt_core.status.axes[0]!.logical_position = .00001;
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(button.textContent, "✓ At plan start");
  t.mock.timers.tick(5000); await setImmediate();
  assert.equal(button.textContent, "Go to plan start", "unchanged native time cannot keep the position confirmed");
  h.status.rt_core.status.time_ns += 1_000_000;
  Object.assign(h.status, { robot_cell: { configured: true, valid: true, model_id: robot.modelId, robot_description_sha256: robot.robotDescriptionSha256, machine_planning_calibration_sha256: "sha256:changed" } });
  t.mock.timers.tick(250); await setImmediate();
  button.click(); await setImmediate();
  assert.equal(h.moves.length, 1, "a genuine calibration mismatch still prevents motion");
  const record = h.project.denseTrajectory!;
  record.segments = h.project.nodes.slice(0, 2).map((node, index) => ({ index, kind: "freespace", sourceId: node.id, sampleCount: 1, durationS: .01 }));
  Object.assign(h.status.session, { state: "playing", plan_id: record.planId, trajectory_digest: record.trajectoryDigest, execution_trajectory_digest: "native-run" });
  Object.assign(h.status.rt_core.status.execution, { segment: 0, identity: { trajectory_digest: "native-run" } });
  const poll = async () => { h.status.rt_core.status.time_ns += 1_000_000; t.mock.timers.tick(250); await setImmediate(); };
  await poll(); assert.equal(h.highlights.at(-1), h.project.nodes[0]!.id);
  Object.assign(h.status.rt_core.status.execution, { segment: 1 });
  await poll(); assert.equal(h.highlights.at(-1), h.project.nodes[1]!.id);
  Object.assign(h.status.session, { execution_trajectory_digest: "different-run" });
  await poll(); assert.equal(h.highlights.at(-1), null, "another native execution must not light this program");
  Object.assign(h.status.session, { execution_trajectory_digest: "native-run" });
  await poll(); assert.equal(h.highlights.at(-1), h.project.nodes[1]!.id);
  Object.assign(h.status.session, { state: "done" });
  await poll(); assert.equal(h.highlights.at(-1), null, "completion clears the execution highlight");

});

function jointRow(host: PanelElement): PanelElement[] {
  // The table sits in its sideways scroller; the first body row's cells.
  const scroller = host.querySelector("#dense-joint-jog").options.find(node => node.className === "joint-jog-table-scroll")!;
  return scroller.options[0]!.options[0]!.options[0]!.options;
}

test("process-output load refusal keeps the machine armed and renders only relevant feedback", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  Object.assign(h.status.rt_core, { reason: "process_io_executor_not_qualified" });
  t.mock.timers.tick(250); await setImmediate();
  h.panel.render();
  const host = h.host.querySelector("#dense-operator-feedback");
  const text = host.options.map(row => row.textContent).join("\n");
  assert.equal(host.hidden, false);
  assert.match(text, /Load refused:.*plan requests welding or process outputs/);
  assert.doesNotMatch(text, /Reset unavailable|Home valid|Limits clear|of range/);
  assert.equal(h.panel.flowView()!.arm.state, "armed");
  assert.match(machineStatePill(h.panel.flowView()!, noLegacy).label, /ARMED/);
  assert.doesNotMatch(machineStatePill(h.panel.flowView()!, noLegacy).label, /FAULT/);
  assert.equal(h.host.querySelector("#dense-stop").disabled, false);
});

test("run mode is available before planning and never changes the authored project", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  const selector = h.host.querySelector("#dense-run-mode");
  assert.equal(h.project.denseTrajectory, null);
  assert.equal(h.host.hidden, false);
  assert.equal(selector.value, "dry_run");
  assert.equal(selector.disabled, false);
  assert.match(h.host.innerHTML, /Dry run · process outputs off/);
  assert.match(h.host.innerHTML, /Program outputs · as planned/);
  const before = JSON.stringify(h.project);
  h.requests.length = 0;
  selector.value = "production"; selector.dispatchEvent(new Event("change"));
  h.panel.render();
  assert.equal(selector.value, "production");
  assert.equal(JSON.stringify(h.project), before);
  assert.deepEqual(h.requests, [], "selecting a mode never loads, moves, or replans");
});

test("process-output refusal offers Use dry run without planning or robot requests", async t => {
  const h = await modelPanel(t, "rosie_1420_v1", false, true);
  const selector = h.host.querySelector("#dense-run-mode");
  assert.equal(selector.value, "production", "explicit production projects retain their default");
  const action = h.host.querySelector("#dense-use-dry-run");
  assert.equal(action.hidden, true);
  Object.assign(h.status.rt_core, { reason: "process_io_executor_not_qualified" });
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(action.hidden, false);
  const before = JSON.stringify(h.project);
  h.requests.length = 0;
  action.click();
  assert.equal(selector.value, "dry_run");
  assert.equal(action.hidden, true);
  assert.equal(JSON.stringify(h.project), before);
  assert.deepEqual(h.requests, [], "recovery never calls Load, Arm, Home, Play, or the planner");
  h.status.session.state = "playing";
  Object.assign(h.status.rt_core.status.execution, { state: "running" });
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(selector.disabled, true);
  selector.value = "production"; selector.dispatchEvent(new Event("change"));
  assert.equal(selector.value, "dry_run", "synthetic changes cannot switch a running program");
});

function installDenseRecord(h: Awaited<ReturnType<typeof modelPanel>>) {
  Object.assign(h.status, { robot_cell: { configured: true, valid: true, model_id: h.status.target.model_id, robot_description_sha256: "sha256:robot", machine_planning_calibration_sha256: "sha256:calibration" } });
  for (const axis of h.status.rt_core.status.axes) Object.assign(axis, { calibration_valid: true, position_state: "trusted", logical_velocity: 0 });
  h.project.denseTrajectory = { schema: "robot.v4.dense-joint-trajectory.v1", trajectoryDigest: "sha256:source", requestSha256: "sha256:request", planId: "plan", programId: "program",
    programDigest: "sha256:program", manifestRevision: 1, planRevision: 1, createdAt: "", dtS: 0.01, totalSampleCount: 101, totalDurationS: 1,
    bytes: 1024, segments: [], sourceProjectRevision: h.project.identity.revision };
  h.panel.render();
}

function startTestBlob(model: string, positions = [0, 0, 0, 0, 0, 0, 0, 0, 0], startState?: object): Uint8Array {
  const header = new TextEncoder().encode(JSON.stringify({ schema: "robot.v4.dense-joint-trajectory.v1", trajectory_digest: "sha256:source",
    robot_cell: { model_id: model, robot_description_sha256: "sha256:robot", machine_planning_calibration_sha256: "sha256:calibration" },
    dt_s: .01, segments: [{ sample_count: 1 }], start_state: startState, program_digest: "sha256:program" }));
  const bytes = new Uint8Array(4 + header.length + 153), view = new DataView(bytes.buffer);
  view.setUint32(0, header.length); bytes.set(header, 4);
  positions.forEach((position, index) => view.setFloat64(4 + header.length + 8 + index * 8, position));
  return bytes;
}

test("Load requires Home on every axis and recovery only navigates to Home", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  installDenseRecord(h);
  const load = h.host.querySelector("#dense-load");
  assert.equal(load.disabled, false);
  h.status.rt_core.status.axes = Array.from({ length: 6 }, () => ({ ...h.status.rt_core.status.axes[0]!, home_valid: false, logical_valid: false, calibration_valid: undefined, position_state: undefined }));
  Object.assign(h.status.rt_core.status.core, { armed: 0, axes: Array.from({ length: 6 }, () => ({ anchor_reason: "anchor_identity_mismatch", anchor_identity_changed_field: "anchor.source" })) });
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(load.disabled, true);
  assert.match(load.title, /encoder reference source changed.*saved position reference cannot be reused; establish Home again/);
  assert.equal(h.host.querySelector("#dense-load-blocker").hidden, false);
  const review = h.host.querySelector("#dense-review-home");
  assert.equal(review.hidden, false);
  h.requests.length = 0;
  load.dispatchEvent(new Event("click"));
  review.click();
  await setImmediate();
  assert.deepEqual(h.requests, [], "no upload, Home, Arm or other machine command");
  assert.equal(h.host.querySelector('[data-step="home"]').focused, true);
  assert.equal(h.host.querySelector('[data-step="home"]').scrolled, true);
  h.status.rt_core.status.axes.forEach(axis => { axis.home_valid = true; axis.logical_valid = true; });
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(load.disabled, false, "disarmed machines with valid Home can load");
  assert.equal(review.hidden, true);
  assert.equal(h.host.querySelector("#dense-load-blocker").hidden, true);
});

test("Load carries run mode; switching requires Reload before Play without mutating the plan", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  installDenseRecord(h);
  const before = JSON.stringify(h.project);
  const calls: DenseLoadIdentity[] = [];
  t.mock.method(h.client, "load", async (identity: DenseLoadIdentity) => {
    calls.push(identity);
    Object.assign(h.status.session, { trajectory_digest: identity.trajectory_digest, dry_run: identity.dry_run, execution_trajectory_digest: "sha256:execution", state: "loaded" });
    return { sessionId: "s", status: normalizeDenseStatus(h.status), notes: [] };
  });
  const selector = h.host.querySelector("#dense-run-mode");
  h.host.querySelector("#dense-load").click();
  assert.equal(selector.disabled, true, "mode cannot change during Load");
  selector.value = "production"; selector.dispatchEvent(new Event("change"));
  assert.equal(selector.value, "dry_run");
  await setImmediate();
  assert.equal(calls[0]!.dry_run, true);
  assert.equal(calls[0]!.trajectory_digest, "sha256:source");
  assert.equal(h.host.querySelector("#dense-play").disabled, false);
  selector.value = "production"; selector.dispatchEvent(new Event("change"));
  assert.equal(h.host.querySelector("#dense-play").disabled, true);
  assert.equal(h.host.querySelector("#dense-load").textContent, "Reload to robot");
  h.requests.length = 0;
  h.host.querySelector("#dense-play").dispatchEvent(new Event("click"));
  assert.deepEqual(h.requests, [], "a stale loaded variant cannot Play with a synthetic click");
  h.host.querySelector("#dense-load").click(); await setImmediate();
  assert.equal(calls[1]!.dry_run, false);
  assert.equal(h.host.querySelector("#dense-play").disabled, false);
  assert.equal(JSON.stringify(h.project), before);
  Object.assign(h.status.session, { dry_run: undefined });
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(h.host.querySelector("#dense-play").disabled, true, "unreported session mode requires a fresh Load");
});

test("Load sends the kept trajectory only after the planner no longer holds the digest", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  installDenseRecord(h);
  const calls: DenseLoadIdentity[] = [];
  let refusal = "dense_blob_not_found";
  t.mock.method(h.client, "load", async (identity: DenseLoadIdentity) => {
    calls.push(identity);
    if (calls.length === 1) throw new DenseExecutionError(refusal, "the weld planner motion server has no dense trajectory", 404);
    Object.assign(h.status.session, { trajectory_digest: identity.trajectory_digest, dry_run: identity.dry_run, execution_trajectory_digest: "sha256:execution", state: "loaded" });
    return { sessionId: "s", status: normalizeDenseStatus(h.status), notes: [] };
  });
  h.host.querySelector("#dense-load").click();
  for (let i = 0; i < 5; i++) await setImmediate();
  assert.equal(calls.length, 2);
  assert.equal(calls[0]!.dense_blob_base64, undefined, "the first Load carries no bytes: an older server refuses the body");
  const kept = startTestBlob("rosie_1420_v1");
  assert.equal(calls[1]!.dense_blob_base64, btoa(String.fromCharCode(...kept)));
  assert.equal(calls[1]!.trajectory_digest, calls[0]!.trajectory_digest);

  // Any other refusal is the answer: no second request.
  calls.length = 0;
  refusal = "home_required";
  Object.assign(h.status.session, { state: "idle" });
  t.mock.timers.tick(250); await setImmediate();
  h.host.querySelector("#dense-load").click();
  for (let i = 0; i < 5; i++) await setImmediate();
  assert.equal(calls.length, 1);
});

test("loaded Play disables when native time stops, ignores synthetic clicks and recovers on fresh feedback", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  installDenseRecord(h);
  Object.assign(h.status.session, { trajectory_digest: "sha256:source", dry_run: true, state: "loaded" });
  h.status.rt_core.status.time_ns++;
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(h.host.querySelector("#dense-play").disabled, false);
  for (let i = 0; i < 11; i++) { t.mock.timers.tick(250); await setImmediate(); }
  const play = h.host.querySelector("#dense-play");
  assert.equal(play.disabled, true);
  assert.equal(play.title, "Waiting for fresh machine status before playing.");
  assert.equal(h.host.querySelector("#dense-stop").disabled, false);
  h.requests.length = 0;
  play.dispatchEvent(new Event("click")); await setImmediate();
  assert.deepEqual(h.requests, [], "no Play request may use the stale loaded/armed snapshot");
  play.disabled = false; // The status can age between rendering the button and clicking it.
  play.dispatchEvent(new Event("click")); await setImmediate();
  assert.deepEqual(h.requests, [], "the click handler rechecks native freshness independently of its last render");
  h.status.rt_core.status.time_ns++;
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(play.disabled, false);
});

for (const first of [0, 0.2]) test(`Play compares all measured joints to actual first sample ${first}, without an implicit approach`, async t => {
  const positions = [first, -.1, .1, .05, -.05, .03, 0, 0, 0];
  const blob = startTestBlob("rosie_1420_v1", positions);
  let pending: Promise<Uint8Array> = Promise.resolve(blob);
  const h = await modelPanel(t, "rosie_1420_v1", false, false, () => pending, 6);
  const axis = h.status.rt_core.status.axes[0]!, description = h.status.rt_core.description.axes[0]!;
  h.status.rt_core.description.axes = positions.slice(0, 6).map((_, index) => ({ ...description, index, id: `J${index + 1}` }));
  h.status.rt_core.status.axes = positions.slice(0, 6).map(position => ({ ...axis, logical_position: position }));
  installDenseRecord(h);
  Object.assign(h.status.session, { trajectory_digest: "sha256:source", dry_run: true, state: "loaded" });
  const observe = async () => { h.status.rt_core.status.time_ns++; t.mock.timers.tick(250); await setImmediate(); };
  h.status.rt_core.status.axes[5]!.logical_position += .001;
  await observe();
  const play = h.host.querySelector("#dense-play");
  assert.equal(play.disabled, true, "even J6 mismatch prevents using the planner's implicit start");
  assert.match(play.title, /^Go to plan start first: Play starts only with every joint at this plan's first position/);
  h.requests.length = 0;
  play.disabled = false; play.dispatchEvent(new Event("click")); await setImmediate();
  assert.equal(h.requests.some(path => path.endsWith("/play")), false);
  assert.deepEqual(h.moves, [], "a mismatch never inserts a move to model zero or plan start");
  h.status.rt_core.status.axes[5]!.logical_position = positions[5]!;
  await observe();
  assert.equal(play.disabled, false, "the actual plan start, including nonzero starts, can be confirmed");
  const delayed = deferred<Uint8Array>(); pending = delayed.promise;
  h.status.rt_core.status.core.home_epoch++;
  await observe();
  assert.equal(play.disabled, true, "a new Home epoch cannot reuse the old cached start proof");
  h.status.rt_core.status.core.configuration_epoch++;
  await observe();
  assert.equal(play.disabled, true);
  Object.assign(h.status, { robot_cell: { configured: true, valid: true, model_id: "rosie_1420_v1", robot_description_sha256: "sha256:robot", machine_planning_calibration_sha256: "sha256:changed" } });
  await observe(); delayed.resolve(blob); await setImmediate();
  assert.equal(play.disabled, true, "late bytes bound to the old calibration cannot restore confirmation");
  assert.equal(h.host.querySelector("#dense-stop").disabled, false);
});

test("a superseded project cannot accept a late Load response", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  installDenseRecord(h);
  const pending = deferred<Awaited<ReturnType<DenseExecutionClient["load"]>>>();
  t.mock.method(h.client, "load", () => pending.promise);
  h.host.querySelector("#dense-load").click();
  h.project.identity.id = "another-project";
  h.project.denseTrajectory = null;
  h.panel.render();
  pending.resolve({ sessionId: "old", status: normalizeDenseStatus({ ...h.status, session: { id: "old", trajectory_digest: "sha256:source", dry_run: true, state: "loaded" } }), notes: [] });
  await setImmediate();
  assert.equal(h.panel.loadedTrajectoryDigest(), null);
  assert.equal(h.host.querySelector("#dense-play").disabled, true);
});

test("bench_unknown with ready armed status disables Cartesian, joint holds, angle GO and Arm with zero requests", async t => {
  const h = await modelPanel(t, "bench_unknown");
  const reason = "Unknown robot model bench_unknown: this app does not know how to drive it";
  assert.equal(h.panel.flowView()!.arm.state, "armed");
  assert.equal(h.panel.cellModel(), "bench_unknown");
  assert.equal(cellModelPresentation(h.presentations.at(-1)!).viewportNote, reason);
  assert.equal(h.panel.cartesianJog()!.disabled, reason);
  assert.equal(h.host.querySelector('[data-step-summary="arm"]').textContent, reason);
  assert.equal(h.host.querySelector('[data-step-summary="drive"]').textContent, reason);
  const row = jointRow(h.host), hold = row[3]!.options[1]!, go = row[4]!.options[0]!.options[1]!;
  assert.equal(hold.disabled, true); assert.equal(go.disabled, true);
  assert.equal(hold.title, reason); assert.equal(go.title, reason);
  assert.equal(h.host.querySelector("#dense-arm-on").disabled, true);
  assert.equal(h.host.querySelector("#dense-arm-confirm").disabled, true);
  h.requests.length = 0;
  h.panel.cartesianJog()!.begin("base", [1, 0, 0, 0, 0, 0], .1);
  hold.dispatchEvent(Object.assign(new Event("pointerdown"), { button: 0, isPrimary: true }));
  row[4]!.options[0]!.options[0]!.value = "15"; go.dispatchEvent(new Event("click"));
  h.host.querySelector("#dense-arm-on").dispatchEvent(new Event("click"));
  h.host.querySelector("#dense-arm-confirm").dispatchEvent(new Event("click"));
  await setImmediate();
  assert.equal(h.requests.length, 0);
});

test("invalid catalogue restores no arm capability and displays an unknown model", async t => {
  const h = await modelPanel(t, "rosie_1420_v1", true);
  assert.equal(h.panel.cellModel(), "unknown");
  assert.equal(h.presentations.at(-1), "unknown");
  assert.equal(cellModelPresentation(h.panel.cellModel()).viewportNote, "Unknown robot model unknown: this app does not know how to drive it");
  assert.equal(h.panel.flowView()!.canArm, false);
  assert.equal(h.panel.flowView()!.canDrive, false);
  assert.equal(h.panel.cartesianJog()!.disabled, "Unknown robot model unknown: this app does not know how to drive it");
  h.host.querySelector("#dense-arm-on").dispatchEvent(new Event("click"));
  h.panel.cartesianJog()!.begin("base", [1, 0, 0, 0, 0, 0], .1);
  await setImmediate();
  assert.equal(h.requests.length, 0);
});

for (const model of ["rosie_1400_v3", "rosie_1420_v1", "bench_one_motor_1to1", "bench_nine_motors_1to1"]) {
  test(`${model}: stale ARM fact disables hold and angle GO and ends an active hold once`, async t => {
    const h = await modelPanel(t, model);
    const row = jointRow(h.host), hold = row[3]!.options[1]!, go = row[4]!.options[0]!.options[1]!;
    assert.equal(hold.disabled, false); assert.equal(go.disabled, false);
    hold.dispatchEvent(Object.assign(new Event("pointerdown"), { button: 0, isPrimary: true }));
    await setImmediate();
    assert.equal(h.requests.filter(path => path.endsWith("/jog/begin")).length, 1);
    h.requests.length = 0;
    t.mock.timers.setTime(110_000);
    h.panel.render(); h.panel.render();
    await setImmediate();
    assert.equal(h.panel.statusAgeMs(), 10_000);
    assert.match(machineStatePill(h.panel.flowView()!, noLegacy).label, /STATUS STALE/);
    assert.equal(hold.disabled, true); assert.equal(go.disabled, true);
    assert.equal(hold.title, "Arm state UNKNOWN: no fresh status from the machine.");
    assert.equal(h.requests.filter(path => path.endsWith("/jog/end")).length, 1);
    assert.equal(h.requests.length, 1);
    h.requests.length = 0;
    hold.dispatchEvent(Object.assign(new Event("pointerdown"), { button: 0, isPrimary: true }));
    row[4]!.options[0]!.options[0]!.value = "15"; go.dispatchEvent(new Event("click"));
    hold.dispatchEvent(new Event("pointerup")); h.panel.render();
    await setImmediate();
    assert.equal(h.requests.length, 0);
  });
}

test("clearing a bench choice restores the arm model, viewport and preview presentation", async t => {
  t.mock.timers.enable({ apis: ["setTimeout"] });
  const note = { textContent: "", hidden: true }, panes = [{ style: { visibility: "" } }];
  let option: unknown = null;
  const modelSelect = { disabled: false, value: "", querySelector: () => option ? { remove: () => { option = null; } } : null, append: (value: unknown) => { option = value; } };
  for (const [name, value] of Object.entries({ window: new EventTarget(), document: Object.assign(new EventTarget(), { createElement: () => new PanelElement() }), sessionStorage: { getItem: () => "", setItem: () => {} } })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  const cell = { cell_id: "bench", label: "Mini bench", role: "bench", models: ["bench_one_motor_1to1"], requested_mode: "simulation", backend: "simulation", host: "localhost" };
  const project = createDogBoxProject(), callbacks: string[] = [];
  const panel = new DensePanel({ client: new DenseExecutionClient(async () => { assert.fail("clearing a choice sends nothing"); }),
    cells: new CellTargetsClient(async () => Response.json({ targets: [cell], selected: null })), getProject: () => project,
    onCellModels: (models, selected) => {
      callbacks.push(selected);
      assert.ok(models.includes(selected));
      renderCellModel(modelSelect as unknown as HTMLSelectElement, note as HTMLElement, panes as HTMLElement[], selected);
    }, highlightNode: () => {}, status: () => {}, onLockChange: () => {}, onTargetChange: () => {}, onRobotPose: () => {}, onRobotMotion: () => {}, wantsFastPoll: () => false });
  const host = new PanelElement(); panel.mount(host as unknown as HTMLElement, host as unknown as HTMLElement); await setImmediate();
  const picker = host.querySelector("#dense-target"); picker.value = cell.cell_id; picker.dispatchEvent(new Event("change"));
  assert.equal(panel.cellModel(), "bench_one_motor_1to1"); assert.equal(note.hidden, false); assert.equal(modelSelect.disabled, true);
  picker.value = ""; picker.dispatchEvent(new Event("change"));
  assert.deepEqual(callbacks, ["bench_one_motor_1to1", project.robot.model]);
  assert.equal(panel.cellModel(), project.robot.model); assert.equal(modelSelect.value, project.robot.model);
  assert.equal(modelSelect.disabled, false); assert.equal(note.hidden, true); assert.equal(option, null);
  assert.equal(panes[0]!.style.visibility, "");
});

for (const action of ["stop", "disarm", "selection_changed"] as const) {
  test(`Machine panel preserves a Cartesian fetch through failed polling and names ${action}`, async t => {
    t.mock.timers.enable({ apis: ["setTimeout"] });
    for (const [name, value] of Object.entries({ window: new EventTarget(), document: Object.assign(new EventTarget(), { createElement: () => new PanelElement() }), sessionStorage: { getItem: () => "", setItem: () => undefined } })) {
      const previous = Object.getOwnPropertyDescriptor(globalThis, name);
      Object.defineProperty(globalThis, name, { configurable: true, value });
      t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
    }
    const description = { contract_version: ContractVersion, capabilities_digest: CapabilitiesDigest, machine_sha256: "a".repeat(64), deployment_sha256: "b".repeat(64),
      axes: [{ index: 0, id: "J1", position_unit: "rad", min_position: -5, max_position: 5 }], capabilities: [] };
    const target = { cell_id: "bench", selection_generation: "1", model_id: "rosie_1420_v1", label: "Bench", host: "localhost", backend: "simulation" };
    const good = { backend: "rt_core", available: true, round_trip_ns: 240000000, target, session: { id: "s", state: "idle", lease: { held: true } },
      rt_core: { description, status: { time_ns: 1, adapter_incarnation: "a", daemon_incarnation: "d", core: { armed: 1 }, motion: { done: true }, execution: { state: "idle" },
        axes: [{ home_valid: true, logical_valid: true, readiness: "ready", logical_position: 0 }] } } };
    let pollStatus: unknown = good, pollFails = false, signal: AbortSignal | null | undefined;
    const entered = deferred<void>(), move = deferred<Response>();
    const inhibition: string[] = [];
    const client = new DenseExecutionClient(async (url, init) => {
      const path = String(url);
      if (path.endsWith("/capabilities")) return Response.json({ backend: "rt_core", available: true, daemon_reachable: true, cell: "bench", rt_core: { description } });
      if (path.endsWith("/cartesian/move")) { signal = init?.signal; entered.resolve(); return move.promise; }
      if (path.endsWith("/state")) return Response.json({ active: false, revision: 0 });
      if (path.endsWith("/status")) { if (pollFails) throw new TypeError("Failed to fetch"); return Response.json(pollStatus); }
      if (path.endsWith("/heartbeat")) return Response.json({ ok: true });
      inhibition.push(path.endsWith("/arm") ? JSON.parse(String(init?.body)).armed === false ? "disarm" : "arm" : "stop");
      return Response.json({ accepted: true, stopped: true, attempts: 1, status: good });
    });
    const panel = new DensePanel({ client, getProject: () => createDogBoxProject(), highlightNode: () => {}, status: () => {}, onLockChange: () => {}, onTargetChange: () => {}, onRobotPose: () => {}, onRobotMotion: () => {}, wantsFastPoll: () => false });
    const host = new PanelElement(); panel.mount(host as unknown as HTMLElement, host as unknown as HTMLElement);
    await setImmediate();
    const select = host.querySelector("#dense-target"); select.value = "joint-daemon"; select.dispatchEvent(new Event("change"));
    t.mock.timers.tick(0); await setImmediate();
    const machine = panel.cartesianJog()!;
    assert.equal(machine.disabled, "");
    machine.begin("base", [1, 0, 0, 0, 0, 0], .05);
    await entered.promise;
    for (const next of [{ ...good, target: undefined }, { ...good, session: null }, "failed", { ...good, rt_core: { ...good.rt_core, stale: true, reason: "rt_core_status_slow" } }]) {
      pollFails = next === "failed"; pollStatus = next;
      t.mock.timers.tick(1000); await setImmediate();
      assert.equal(signal?.aborted, false);
      assert.equal(machine.busy, true);
    }
    assert.equal(machine.reason, "waiting for the machine");
    window.dispatchEvent(new Event("pagehide"));
    assert.deepEqual(inhibition, []);
    if (action === "selection_changed") {
      pollFails = false; pollStatus = { ...good, target: { ...target, selection_generation: "2" } };
      t.mock.timers.tick(1000); await setImmediate();
    } else {
      host.querySelector(action === "stop" ? "#dense-stop" : "#dense-arm-off").click();
      await setImmediate();
      assert.deepEqual(inhibition, [action]);
    }
    assert.equal(signal?.aborted, true);
    assert.equal(signal?.reason, action);
    assert.equal(machine.reason, action === "stop" ? "Move cancelled: STOP pressed" : action === "disarm" ? "Move cancelled: DISARM pressed" : "Move cancelled: selection changed in another window");
    move.resolve(Response.json({ state: "done", axis: 0, revision: 1 }));
    await setImmediate();
    assert.match(machine.reason, /^Move cancelled:/);
  });
}

function rtStatus(armed: number, reason = "") {
  return {
    backend: "rt_core", available: true,
    rt_core: {
      reason,
      status: {
        time_ns: 1, adapter_incarnation: "adapter", daemon_incarnation: "daemon",
        axes: [], core: { armed, safety_fault_mask: 0 }, execution: { state: "idle", error: "" },
      },
    },
  };
}

test("mini bench connects and arms independently of the arm project, with one live J1 jog and angle row", async t => {
  t.mock.timers.enable({ apis: ["setTimeout"] });
  for (const [name, value] of Object.entries({ window: new EventTarget(), document: Object.assign(new EventTarget(), { createElement: () => new PanelElement() }), sessionStorage: { getItem: () => "", setItem: () => undefined } })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  const project = createDogBoxProject(), original = JSON.stringify(project);
  const cell: CellTarget = { cell_id: "example-minibench", label: "Mini bench", role: "bench", models: ["bench_one_motor_1to1"], requested_mode: "simulation", backend: "simulation", simulation: true, host: "localhost" };
  const target = { ...cell, model_id: "bench_one_motor_1to1", selection_generation: "1" };
  const description = { contract_version: ContractVersion, capabilities_digest: CapabilitiesDigest, machine_sha256: "a".repeat(64), deployment_sha256: "b".repeat(64),
    axes: [{ index: 0, id: "J1", position_unit: "rad", min_position: -3, max_position: 3, max_velocity: 314.159 }], capabilities: [] };
  let armed = 0;
  const status = () => ({ backend: "rt_core", available: true, target, session: { id: "s", state: "idle", lease: { held: armed === 1 } }, rt_core: { description, status: {
    time_ns: 1_000_000, adapter_incarnation: "adapter", daemon_incarnation: "daemon", execution: { state: "idle" }, core: { armed, safety_fault_mask: 0 },
    axes: [{ home_valid: true, logical_valid: true, readiness: "ready", logical_position: .125, logical_target: .125, statusword: 0, position_counts: 2608 }],
  } } });
  const calls: string[] = [];
  const cells = new CellTargetsClient(async (url, init) => {
    calls.push(String(url));
    if (String(url).endsWith("/select")) {
      assert.deepEqual(JSON.parse(String(init!.body)), { cell_id: cell.cell_id, model_id: "bench_one_motor_1to1" });
      return Response.json({ selected: { cell_id: cell.cell_id, model_id: "bench_one_motor_1to1" }, target: cell });
    }
    return Response.json({ targets: [cell], selected: null });
  });
  const client = new DenseExecutionClient(async (url, init) => {
    calls.push(String(url));
    if (String(url).endsWith("capabilities")) return Response.json({ backend: "rt_core", cell: cell.cell_id, available: true, daemon_reachable: true, rt_core: { description } });
    if (String(url).endsWith("/arm")) { armed = 1; return Response.json({ accepted: true, status: status() }); }
    if (String(url).endsWith("/move")) {
      const move = JSON.parse(String(init!.body)); assert.equal(move.axis, 0); assert.equal(move.degrees, 15);
      return Response.json({ state: "done", axis: 0, revision: 1 });
    }
    if (String(url).includes("/jog/")) return Response.json({ active: false, revision: 0 });
    return Response.json(status());
  });
  const panel = new DensePanel({ client, cells, getProject: () => project, onCellModels: (_models, selected) => { assert.equal(cellModelPresentation(selected).armModel, null); },
    highlightNode: () => {}, status: () => {}, onLockChange: () => {}, onTargetChange: () => {}, onRobotPose: () => assert.fail("bench has no viewer pose"), onRobotMotion: () => {}, wantsFastPoll: () => false });
  const host = new PanelElement(); panel.mount(host as unknown as HTMLElement, host as unknown as HTMLElement);
  await setImmediate();
  const picker = host.querySelector("#dense-target"); picker.value = cell.cell_id; picker.dispatchEvent(new Event("change"));
  assert.equal(panel.cellModel(), "bench_one_motor_1to1");
  assert.equal(calls.filter(path => path.endsWith("/arm")).length, 0);
  host.querySelector("#dense-cell-connect").click(); await setImmediate();
  assert.equal(panel.flowView()!.modelId, "bench_one_motor_1to1", "Connect names the bench before the first status");
  t.mock.timers.tick(0); await setImmediate();
  assert.equal(panel.targetSelected(), true);
  assert.equal(panel.flowView()!.modelId, "bench_one_motor_1to1");
  host.querySelector("#dense-arm-on").click(); await setImmediate();
  assert.equal(panel.flowView()!.arm.state, "armed");
  assert.equal(panel.cartesianJog()!.disabled, "Not available on a bench without an arm model");
  const table = host.querySelector("#dense-joint-jog").options.find(node => node.className === "joint-jog-table-scroll")!.options[0]!;
  assert.equal(table.options[0]!.options.length, 1);
  const row = table.options[0]!.options[0]!;
  assert.equal(row.options[0]!.textContent, "J1");
  assert.equal(row.options[1]!.textContent, "7.16°");
  assert.equal(row.options[2]!.textContent, "—");
  const hold = row.options[3]!.options[1]!;
  assert.equal(hold.disabled, false);
  hold.dispatchEvent(Object.assign(new Event("pointerdown", { cancelable: true }), { button: 0, isPrimary: true }));
  await setImmediate(); hold.dispatchEvent(new Event("pointerup")); await setImmediate();
  assert.equal(calls.filter(path => path.endsWith("/jog/begin")).length, 1);
  const move = row.options[4]!.options[0]!; move.options[0]!.value = "15"; move.options[1]!.click(); await setImmediate();
  assert.equal(calls.filter(path => path.endsWith("/move")).length, 1);
  assert.equal(JSON.stringify(project), original);
});

// 2026-11-04 on the mini bench: after Cells > None (offline) the Robot model
// picker still held the bench's state, every arm option disabled, so the
// program could not be moved to another robot until the page was reloaded.
test("None (offline) after a connected bench offers every arm model again", async t => {
  t.mock.timers.enable({ apis: ["setTimeout"] });
  for (const [name, value] of Object.entries({ window: new EventTarget(), document: Object.assign(new EventTarget(), { createElement: () => new PanelElement() }), sessionStorage: { getItem: () => "", setItem: () => undefined } })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  const project = createDogBoxProject();
  const cell: CellTarget = { cell_id: "example-minibench", label: "Mini bench", role: "bench", models: ["bench_one_motor_1to1"], requested_mode: "simulation", backend: "simulation", simulation: true, host: "localhost" };
  const target = { ...cell, model_id: "bench_one_motor_1to1", selection_generation: "1" };
  const description = { contract_version: ContractVersion, capabilities_digest: CapabilitiesDigest, machine_sha256: "a".repeat(64), deployment_sha256: "b".repeat(64),
    axes: [{ index: 0, id: "J1", position_unit: "rad", min_position: -3, max_position: 3, max_velocity: 314.159 }], capabilities: [] };
  let selected = true;
  const status = () => ({ backend: "rt_core", available: true, ...(selected ? { target } : {}), session: { id: "s", state: "idle", lease: { held: false } }, rt_core: { description, status: {
    time_ns: 1_000_000, adapter_incarnation: "adapter", daemon_incarnation: "daemon", execution: { state: "idle" }, core: { armed: 0, safety_fault_mask: 0 },
    axes: [{ home_valid: true, logical_valid: true, readiness: "ready", logical_position: .125, logical_target: .125, statusword: 0, position_counts: 2608 }],
  } } });
  const cells = new CellTargetsClient(async url => String(url).endsWith("/select")
    ? Response.json({ selected: { cell_id: cell.cell_id, model_id: "bench_one_motor_1to1" }, target: cell })
    : Response.json({ targets: [cell], selected: null }));
  const client = new DenseExecutionClient(async url => {
    if (String(url).endsWith("/target")) { selected = false; return Response.json({ backend: "rt_core", cell: "", available: false, daemon_reachable: false }); }
    if (String(url).endsWith("capabilities")) return Response.json({ backend: "rt_core", cell: cell.cell_id, available: true, daemon_reachable: true, rt_core: { description } });
    return Response.json(status());
  });
  const offered: [readonly string[], string][] = [];
  const panel = new DensePanel({ client, cells, getProject: () => project, onCellModels: (models, model) => { offered.push([models, model]); },
    highlightNode: () => {}, status: () => {}, onLockChange: () => {}, onTargetChange: () => {}, onRobotPose: () => {}, onRobotMotion: () => {}, wantsFastPoll: () => false });
  const host = new PanelElement(); panel.mount(host as unknown as HTMLElement, host as unknown as HTMLElement);
  await setImmediate();
  const picker = host.querySelector("#dense-target"); picker.value = cell.cell_id; picker.dispatchEvent(new Event("change"));
  host.querySelector("#dense-cell-connect").click(); await setImmediate();
  t.mock.timers.tick(0); await setImmediate();
  assert.equal(panel.targetSelected(), true);
  assert.equal(panel.cellModel(), "bench_one_motor_1to1");
  panel.selectTarget(""); await setImmediate(); await setImmediate();
  assert.equal(panel.targetSelected(), false);
  assert.equal(panel.cellModel(), project.robot.model);
  assert.deepEqual(offered.at(-1), [["rosie_1400_v3", "rosie_1420_v1"], project.robot.model]);
});

// A tab open across a server restart reads an empty catalogue reply once. The
// panel retries by itself and comes back with the server's selection, so the
// operator never sees a permanent "blocked" step with no machine and no joint
// rows (2026-09-16 on the LA bench: one empty reply during a restart stranded
// the owner's tab until a manual reload).
test("machine catalogue read failure retries on its own and recovers the selection", async (t) => {
  t.mock.timers.enable({ apis: ["setTimeout"] });
  for (const [name, value] of Object.entries({
    window: new EventTarget(), document: Object.assign(new EventTarget(), { createElement: () => new PanelElement() }),
    sessionStorage: { getItem: () => "joint-daemon", setItem: () => assert.fail("catalogue selection is process-wide") },
  })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => { if (previous) Object.defineProperty(globalThis, name, previous); else Reflect.deleteProperty(globalThis, name); });
  }
  const project = createDogBoxProject();
  const cell: CellTarget = { cell_id: "bench-relay", label: "Detached bench", role: "TEST BENCH / BENCH-RELAY", models: ["rosie_1420_v1"], requested_mode: "simulation", backend: "simulation", host: "127.0.0.1:8443" };
  cell.simulation = true;
  const selection = { cell_id: cell.cell_id, model_id: cell.models[0] };
  let listCalls = 0;
  const cells = new CellTargetsClient(async (url) => {
    assert.doesNotMatch(String(url), /select$/);
    listCalls++;
    // The first read is the empty body a restarting server (or its proxy) returns.
    if (listCalls === 1) return new Response("", { status: 200, headers: { "Content-Type": "application/json" } });
    return Response.json({ targets: [cell], selected: selection });
  });
  const client = new DenseExecutionClient(async (url) => {
    if (String(url).endsWith("/status")) return Response.json({ ...rtStatus(0), target: { ...cell, model_id: "rosie_1420_v1" } });
    assert.match(String(url), /capabilities$/);
    return Response.json({ backend: "rt_core", available: true, daemon_reachable: true });
  });
  const panel = new DensePanel({ client, cells, getProject: () => project,
    onCellModels: (models, selected) => { project.robot.model = cellModelPresentation(selected).armModel!; },
    highlightNode: () => undefined, status: () => undefined, onLockChange: () => undefined,
    onTargetChange: () => undefined, onRobotPose: () => undefined, onRobotMotion: () => undefined, wantsFastPoll: () => false,
  });
  const host = new PanelElement();
  panel.mount(host as unknown as HTMLElement, host as unknown as HTMLElement);
  await setImmediate();
  assert.equal(listCalls, 1);
  assert.match(host.querySelector("#dense-target-state").textContent, /Machine catalogue unavailable .*retrying/);
  assert.equal(host.querySelector('[data-step="choose"]').dataset.state, "blocked");
  assert.match(host.querySelector('[data-step-summary="choose"]').textContent, /could not be read .*retrying/);
  // Back-off table, first entry: the retry fires without any operator action.
  t.mock.timers.tick(DensePanel.CATALOGUE_RETRY_MS[0]!);
  await setImmediate();
  await setImmediate();
  assert.equal(listCalls, 2);
  assert.equal(host.querySelector("#dense-target-state").textContent, `Detached bench · rosie_1420_v1 · ${cell.backend} · 127.0.0.1:8443`);
  assert.notEqual(host.querySelector('[data-step="choose"]').dataset.state, "blocked");
  assert.equal(panel.targetSelected(), true);
});

for (const outcome of ["success", "refused", "remembered", "review-real"] as const) {
  test(outcome === "review-real" ? "TestReviewRound2RealPosture" : `machine picker narrows models without authority and handles ${outcome}`, async (t) => {
    t.mock.timers.enable({ apis: ["setTimeout"] });
    for (const [name, value] of Object.entries({
      window: new EventTarget(), document: Object.assign(new EventTarget(), { createElement: () => new PanelElement() }),
      sessionStorage: { getItem: () => "joint-daemon", setItem: () => assert.fail("catalogue selection is process-wide") },
    })) {
      const previous = Object.getOwnPropertyDescriptor(globalThis, name);
      Object.defineProperty(globalThis, name, { configurable: true, value });
      t.after(() => { if (previous) Object.defineProperty(globalThis, name, previous); else Reflect.deleteProperty(globalThis, name); });
    }
    const project = createDogBoxProject();
    const cell: CellTarget = { cell_id: "bench-relay", label: "Detached bench", role: "TEST BENCH / BENCH-RELAY", models: ["rosie_1420_v1"], requested_mode: "simulation", backend: "simulation", host: "127.0.0.1:8443" };
    cell.simulation = true;
    if (outcome === "review-real") {
      cell.requested_mode = "real";
      cell.backend = "ethercat";
      cell.simulation = false;
    }
    const selection = { cell_id: cell.cell_id, model_id: cell.models[0] };
    const calls: string[] = [];
    let polledTarget: CellTarget | null = null;
    let narrowed: readonly string[] = [];
    const cells = new CellTargetsClient(async (url) => {
      calls.push(String(url));
      if (String(url).endsWith("/select")) return outcome === "refused"
        ? Response.json({ error: "cell_mode_mismatch" }, { status: 409 })
        : Response.json({ selected: selection, target: cell });
      return Response.json({ targets: [cell], selected: outcome === "remembered" ? selection : null });
    });
    const client = new DenseExecutionClient(async (url) => {
      calls.push(String(url));
      if (String(url).endsWith("/arm")) return Response.json({ accepted: true });
      if (String(url).endsWith("/status")) return Response.json({ ...rtStatus(0), target: { ...polledTarget, model_id: "rosie_1420_v1" } });
      assert.match(String(url), /capabilities$/);
      return Response.json({ backend: "rt_core", available: true, daemon_reachable: true });
    });
    const panel = new DensePanel({ client, cells, getProject: () => project,
      onCellModels: (models, selected) => { narrowed = models; project.robot.model = cellModelPresentation(selected).armModel!; },
      highlightNode: () => undefined, status: () => undefined, onLockChange: () => undefined,
      onTargetChange: () => undefined, onRobotPose: () => undefined, onRobotMotion: () => undefined, wantsFastPoll: () => false,
    });
    const host = new PanelElement();
    panel.mount(host as unknown as HTMLElement, host as unknown as HTMLElement);
    await setImmediate();
    const target = host.querySelector("#dense-target");
    target.value = cell.cell_id;
    target.dispatchEvent(new Event("change"));
    assert.deepEqual(narrowed, ["rosie_1420_v1"]);
    assert.equal(project.robot.model, "rosie_1420_v1");
    assert.equal(calls.filter(call => call.endsWith("/select")).length, 0, "highlight must not select or arm");
    if (outcome !== "remembered") {
      assert.equal(panel.targetSelected(), false);
      host.querySelector("#dense-cell-connect").click();
      await setImmediate();
      assert.equal(calls.filter(call => call.endsWith("/select")).length, 1);
    }
    assert.equal(panel.targetSelected(), outcome !== "refused");
    if (outcome === "refused") {
      assert.equal(panel.connectionSummary(), "bench-relay · identity mismatch");
      assert.equal(host.querySelector("#dense-cell-connection").textContent, "cell_mode_mismatch");
      assert.equal(host.querySelector('[data-step="connect"]').dataset.state, "blocked");
      assert.match(host.querySelector('[data-step-summary="connect"]').textContent, /live backend is not what the catalogue requests.*\[cell_mode_mismatch\]/);
    } else {
      assert.equal(host.querySelector("#dense-target-state").textContent, `Detached bench · rosie_1420_v1 · ${cell.backend} · 127.0.0.1:8443`);
      assert.equal(host.querySelector('[data-step="connect"]').dataset.state, "done");
    }
    assert.equal(calls.filter(call => /arm|play|acquire/.test(call)).length, 0);
    if (outcome === "review-real") {
      // Review probe: an explicit real ARM must never leave the main badge SIM.
      project.program.executionMode = "simulation";
      const badge = () => sessionModeStatusPill(project.program.executionMode, false, panel.executionPosture());
      assert.equal(badge().label, "REAL · PHYSICAL OUTPUT");
      assert.equal(badge().className, "is-real");
      // A real cell arms in two deliberate clicks: the first only asks.
      host.querySelector("#dense-arm-on").click();
      await setImmediate();
      assert.equal(calls.filter(call => call.endsWith("/arm")).length, 0, "the first click on a REAL target must not arm");
      assert.equal(host.querySelector("#dense-arm-confirm").hidden, false);
      assert.match(host.querySelector('[data-step-summary="arm"]').textContent, /Confirm to energise the motors on Detached bench/);
      host.querySelector("#dense-arm-confirm").click();
      await setImmediate();
      assert.equal(calls.filter(call => call.endsWith("/arm")).length, 1);
      assert.equal(badge().label, "REAL · PHYSICAL OUTPUT");
      assert.equal(host.querySelector("#dense-runtime").dataset.mode, "live");
      assert.equal(panel.flowView()?.posture, "real");
      target.value = "";
      target.dispatchEvent(new Event("change"));
      assert.equal(badge().label, "REAL · PHYSICAL OUTPUT", "pending choice cannot clear selected posture");
      polledTarget = { ...cell, cell_id: "local-simulation", backend: "simulation", simulation: true };
      t.mock.timers.tick(1);
      await setImmediate();
      assert.equal(badge().label, "SIMULATION", "status follows process-wide selection");
      polledTarget = cell;
      t.mock.timers.tick(250);
      await setImmediate();
      assert.equal(badge().label, "REAL · PHYSICAL OUTPUT", "another tab selecting a real cell cannot leave this tab SIM");
    }
  });
}

for (const reply of ["success", "failure"] as const) {
  for (const armFirst of [false, true]) {
    test(`public STOP bypasses pending Arm and discards ${reply}, Arm first=${armFirst}`, async (t) => {
      t.mock.timers.enable({ apis: ["setTimeout"] });
      const globals = {
        window: new EventTarget(),
        document: Object.assign(new EventTarget(), { createElement: () => new PanelElement() }),
        sessionStorage: { getItem: () => "", setItem: () => undefined },
      };
      for (const [name, value] of Object.entries(globals)) {
        const previous = Object.getOwnPropertyDescriptor(globalThis, name);
        Object.defineProperty(globalThis, name, { configurable: true, value });
        t.after(() => {
          if (previous) Object.defineProperty(globalThis, name, previous);
          else Reflect.deleteProperty(globalThis, name);
        });
      }
      const arm = deferred<Response>();
      const stop = deferred<Response>();
      const requests: string[] = [];
      const messages: string[] = [];
      const client = new DenseExecutionClient(async (url) => {
        const operation = String(url).split("/").at(-1)!;
        requests.push(operation);
        switch (operation) {
          case "capabilities":
            return Response.json({ backend: "rt_core", available: true, daemon_reachable: true });
          case "arm": return arm.promise;
          case "stop": return stop.promise;
          default: throw new Error(`unexpected request ${operation}`);
        }
      });
      const panel = new DensePanel({
        client, getProject: () => createDogBoxProject(),
        highlightNode: () => undefined, status: (message) => { messages.push(message); },
        onLockChange: () => undefined, onTargetChange: () => undefined,
        onRobotPose: () => undefined, onRobotMotion: () => undefined, wantsFastPoll: () => false,
      });
      const host = new PanelElement();
      panel.mount(host as unknown as HTMLElement, host as unknown as HTMLElement);
      await setImmediate();
      const target = host.querySelector("#dense-target");
      target.value = "joint-daemon";
      target.dispatchEvent(new Event("change"));
      const armButton = host.querySelector("#dense-arm-on");
      armButton.click();
      // Posture unknown is treated as real: the confirming click sends Arm.
      assert.equal(requests.filter((op) => op === "arm").length, 0);
      host.querySelector("#dense-arm-confirm").click();
      assert.equal(requests.filter((op) => op === "arm").length, 1);
      assert.equal(requests.filter((op) => op === "stop").length, 0);
      host.querySelector("#dense-stop").click();
      assert.equal(requests.filter((op) => op === "stop").length, 1);

      const finishArm = async () => {
        arm.resolve(reply === "success"
          ? Response.json({ accepted: true, status: rtStatus(1) })
          : Response.json({ error: "control_session_stale", detail: "cancelled Arm" }, { status: 409 }));
        await setImmediate();
      };
      if (armFirst) {
        await finishArm();
        assert.equal(armButton.disabled, true, "late Arm must not unlock a pending Stop");
        assert.equal(messages.length, 0, "superseded Arm must not publish a result");
      }
      stop.resolve(Response.json({
        stopped: true, attempts: 1,
        status: rtStatus(0, "operator_stop"),
      }));
      await setImmediate();
      if (!armFirst) await finishArm();
      // The Stop result leaves the drives' own word fresh and off: DISARMED,
      // on the Arm step and in the run row alike; the reason stays a note.
      assert.equal(host.querySelector("#dense-arm-state").textContent, "DISARMED");
      assert.equal(host.querySelector("#dense-arm-state").dataset.armed, "disarmed");
      assert.match(host.querySelector("#dense-readout").textContent, /^rt-core · DISARMED · operator_stop/);
      assert.equal(machineStatePill(panel.flowView()!, noLegacy).label, "OUTPUT UNKNOWN · selected target · DISARMED");
      assert.equal(armButton.disabled, false);
      // The Stop result is a note on Arm, not a lock; the operator arms again.
      assert.match(host.querySelector('[data-step-summary="arm"]').textContent, /Stopped by the operator\. \[operator_stop\]/);
      assert.equal(host.querySelector('[data-step="arm"]').dataset.state, "current");
      assert.equal(messages.length, 1);
      assert.match(messages[0]!, /stopped \(acknowledged/);
      assert.deepEqual(requests, ["capabilities", "arm", "stop"]);
    });
  }
}

function rejectable<T>() {
  let resolve!: (value: T) => void;
  let reject!: (error: Error) => void;
  const promise = new Promise<T>((done, fail) => { resolve = done; reject = fail; });
  return { promise, resolve, reject };
}

// The LA bench split, at the DOM: the header pill read ARMED while the Arm
// step read "arm state is unknown", the Drive step "Arm first" and the run
// row "busy". Every rendered surface now carries the one fact, through
// DISARMED, ARMING, ARMED, STOPPING, a transport failure that never reached
// the machine, and STATUS STALE when the server stops answering.
test("Arm headline, header pill, run row, Drive step and Jog caption agree in every arm state", async (t) => {
  t.mock.timers.enable({ apis: ["setTimeout", "Date"], now: 100_000 });
  for (const [name, value] of Object.entries({ window: new EventTarget(), document: Object.assign(new EventTarget(), { createElement: () => new PanelElement() }), sessionStorage: { getItem: () => "", setItem: () => undefined } })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  const held = (armed: number, reason = "") => ({ ...rtStatus(armed, reason), session: { id: "s", state: "idle", lease: { held: armed === 1 } } });
  const pollStatus: unknown = held(0);
  let pollFails = false;
  let arm = rejectable<Response>();
  const stop = deferred<Response>();
  const messages: string[] = [];
  const client = new DenseExecutionClient(async (url) => {
    switch (String(url).split("/").at(-1)!) {
      case "capabilities": return Response.json({ backend: "rt_core", available: true, daemon_reachable: true });
      case "status": if (pollFails) throw new Error("fetch failed: connection refused"); return Response.json(pollStatus);
      case "arm": return arm.promise;
      case "stop": return stop.promise;
      default: throw new Error(`unexpected request ${url}`);
    }
  });
  const panel = new DensePanel({ client, getProject: () => createDogBoxProject(), highlightNode: () => undefined, status: (message) => { messages.push(message); },
    onLockChange: () => undefined, onTargetChange: () => undefined, onRobotPose: () => undefined, onRobotMotion: () => undefined, wantsFastPoll: () => false });
  const host = new PanelElement();
  const jogHost = new PanelElement();
  panel.mount(host as unknown as HTMLElement, host as unknown as HTMLElement, jogHost as unknown as HTMLElement);
  await setImmediate();
  const target = host.querySelector("#dense-target"); target.value = "joint-daemon"; target.dispatchEvent(new Event("change"));
  t.mock.timers.tick(0); await setImmediate();
  const surfaces = () => ({
    headline: host.querySelector("#dense-arm-state").textContent,
    armed: host.querySelector("#dense-arm-state").dataset.armed,
    pill: machineStatePill(panel.flowView()!, noLegacy),
    readout: host.querySelector("#dense-readout").textContent,
    arm: host.querySelector('[data-step-summary="arm"]').textContent,
    drive: host.querySelector('[data-step-summary="drive"]').textContent,
    driveState: host.querySelector('[data-step="drive"]').dataset.state,
    jog: jogHost.querySelector("#jog-joints-target").textContent,
  });
  const agree = (token: string, headline: string) => {
    const s = surfaces();
    assert.equal(s.headline, headline, "Arm headline");
    assert.equal(s.pill.label, `OUTPUT UNKNOWN · selected target · ${token}`, "header pill");
    assert.match(s.readout, new RegExp(`^rt-core · ${token} · `), "run row");
    return s;
  };

  // DISARMED: the drives' word, fresh.
  let s = agree("DISARMED", "DISARMED");
  assert.equal(s.armed, "disarmed");
  assert.equal(s.drive, "Arm first.");
  assert.match(s.jog, /Arm it first \(step 4\)\.$/);
  assert.equal(host.querySelector("#dense-arm-state").hidden, true, "main title-bar pill owns the visible armed fact");

  // ARMING: this panel's request in flight (the second, confirming click sent it).
  host.querySelector("#dense-arm-on").click();
  host.querySelector("#dense-arm-confirm").click();
  s = agree("ARMING", "ARMING…");
  assert.equal(s.armed, "arming");
  assert.match(s.arm, /Arm request in flight/);
  assert.doesNotMatch(s.arm, /unknown|Last result/i);
  assert.match(s.drive, /Arming; Drive opens at ARMED/);
  assert.match(s.jog, /Arming; wait for ARMED/);

  // ARMED, with this server's grant: Drive is no longer waiting on Arm.
  arm.resolve(Response.json({ accepted: true, status: held(1) }));
  await setImmediate();
  s = agree("ARMED", "ARMED");
  assert.equal(s.armed, "armed");
  assert.equal(s.pill.className, "is-real");
  assert.notEqual(s.driveState, "pending");
  assert.doesNotMatch(s.drive, /Arm first/);
  assert.doesNotMatch(s.jog, /Arm it first/);

  // STOPPING: the public STOP in flight while the drives are still on.
  host.querySelector("#dense-stop").click();
  s = agree("STOPPING", "STOPPING…");
  assert.equal(s.armed, "disarming");
  assert.match(s.drive, /Stopping; Arm again afterwards/);
  assert.match(s.jog, /Stopping; wait for DISARMED/);

  // Back to DISARMED; the Stop result is a note, not a state.
  stop.resolve(Response.json({ stopped: true, attempts: 1, status: held(0, "operator_stop") }));
  await setImmediate();
  s = agree("DISARMED", "DISARMED");
  assert.match(s.readout, /^rt-core · DISARMED · operator_stop/);
  assert.match(s.arm, /Last result: Stopped by the operator\. \[operator_stop\]/);

  // A transport failure that never reached the machine: the state and the
  // pill stay DISARMED; the Arm step says to retry.
  arm = rejectable<Response>();
  host.querySelector("#dense-arm-on").click();
  host.querySelector("#dense-arm-confirm").click();
  arm.reject(new Error("write tcp 127.0.0.1:34492->127.0.0.1:8443: write: broken pipe"));
  await setImmediate();
  s = agree("DISARMED", "DISARMED");
  assert.match(s.arm, /Request did not reach the machine; retry Arm\./);
  assert.doesNotMatch(s.arm, /unknown|broken pipe/i);
  assert.equal(host.querySelector("#dense-arm-on").disabled, false, "retry Arm is offered");
  assert.match(messages.at(-1)!, /^ARM: write tcp .*broken pipe$/);
  assert.equal(panel.armReason(), "Request did not reach the machine; retry Arm.");

  arm = rejectable<Response>();
  host.querySelector("#dense-arm-on").click();
  host.querySelector("#dense-arm-confirm").click();
  arm.reject(new Error("rt_core_inhibited"));
  await setImmediate();
  assert.equal(panel.armRefused(), true);
  assert.equal(panel.armReason(), "New work is refused until the last Stop is acknowledged and control is released. [rt_core_inhibited]");

  // STATUS STALE: the server stops answering the poll. The pill picks no
  // side, the headline says UNKNOWN, and STOP stays available.
  pollFails = true;
  t.mock.timers.tick(0); await setImmediate();
  s = agree("STATUS STALE", "UNKNOWN: no fresh status from the machine");
  assert.equal(s.armed, "unknown");
  assert.match(s.pill.className, /is-stale/);
  assert.doesNotMatch(s.pill.label, /\bARMED\b|\bDISARMED\b/);
  assert.match(s.pill.title, /STOP stays available/);
  assert.match(s.drive, /UNKNOWN: no fresh status from the machine/);
  assert.match(s.jog, /Arm state UNKNOWN: no fresh status from the machine/);
  assert.equal(host.querySelector("#dense-stop").disabled, false, "STOP stays reachable while stale");
});

test("rt-core status positions reach the host as a pose for the reported model; silence turns the live view STALE", async (t) => {
  t.mock.timers.enable({ apis: ["setTimeout", "Date"], now: 100_000 });
  for (const [name, value] of Object.entries({ window: new EventTarget(), document: Object.assign(new EventTarget(), { createElement: () => new PanelElement() }), sessionStorage: { getItem: () => "", setItem: () => undefined } })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  // Six measured joints; J7-J9 are wire padding, not robot joints.
  const description = { contract_version: ContractVersion, capabilities_digest: CapabilitiesDigest, machine_sha256: "a".repeat(64), deployment_sha256: "b".repeat(64),
    axes: ["J1", "J2", "J3", "J4", "J5", "J6"].map((id, index) => ({ index, id, position_unit: "rad", min_position: -5, max_position: 5 })), capabilities: [] };
  const positions = [0.0498, 0.1937, 0, 0, 0, 0];
  const status = {
    backend: "rt_core", available: true,
    robot_cell: { valid: true, model_id: "rosie_1420_v1", robot_description_sha256: "sha256:" + "1".repeat(64) },
    target: { selection_generation: "g1", cell_id: "la-bench", model_id: "rosie_1420_v1", label: "Rosie 1420 bench", backend: "ethercat", host: "bench-relay" },
    rt_core: { description, status: {
      time_ns: 1_000_000, adapter_incarnation: "adapter", daemon_incarnation: "daemon", execution: { state: "idle" },
      core: { armed: 1, safety_fault_mask: 0 },
      axes: positions.map(position => ({ home_valid: true, logical_valid: true, calibration_valid: true, position_state: "trusted", logical_velocity: 0, readiness: "ready", logical_position: position, logical_target: position, statusword: 0, position_counts: 0 })),
    } },
  };
  let hang = false;
  const client = new DenseExecutionClient(async (url) => {
    if (String(url).endsWith("capabilities")) return Response.json({ backend: "rt_core", available: true, daemon_reachable: true, rt_core: { description } });
    if (hang) return new Promise<Response>(() => undefined);
    return Response.json(status);
  });
  const poses: { modelId: string; jointOrder: readonly string[]; jointsRad: readonly number[] }[] = [];
  const project = createDogBoxProject();
  const panel = new DensePanel({ client, getProject: () => project, highlightNode: () => undefined, status: () => undefined,
    onLockChange: () => undefined, onTargetChange: () => undefined, onRobotPose: pose => poses.push(pose), onRobotMotion: () => undefined, wantsFastPoll: () => false });
  const host = new PanelElement(); panel.mount(host as unknown as HTMLElement, host as unknown as HTMLElement);
  await setImmediate();
  assert.deepEqual(panel.livePoseView(), { mode: "none", pose: null, caption: null }, "no machine: the preview owns the viewport");
  const target = host.querySelector("#dense-target"); target.value = "joint-daemon"; target.dispatchEvent(new Event("change"));
  t.mock.timers.tick(0); await setImmediate();
  assert.equal(poses.length, 1, "one status, one pose");
  assert.equal(poses[0]!.modelId, "rosie_1420_v1", "the model the machine reports, not the open project's");
  assert.deepEqual(poses[0]!.jointOrder, ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]);
  assert.deepEqual(poses[0]!.jointsRad, [0.0498, 0.1937, 0, 0, 0, 0, 0, 0, 0]);
  const live = panel.livePoseView();
  assert.equal(live.mode, "live");
  assert.equal(live.caption, "LIVE · Rosie 1420 bench · rosie_1420_v1 · status 0 ms");
  assert.deepEqual(panel.teachObservation().pose.jointsRad.slice(0, 6), positions);
  // Home31: a raw fallback zero is not a measured J6, in either display or teaching.
  Object.assign(status.rt_core.status.axes[5]!, { home_valid: false, logical_valid: false, calibration_valid: false, position_state: "unverified", logical_position: 0 });
  status.rt_core.status.time_ns += 250_000_000;
  t.mock.timers.tick(250); await setImmediate();
  assert.match(panel.livePoseView().caption!, /^LIVE \(PARTIAL\)/);
  assert.match(panel.livePoseView().jointCaption!, /J6 unavailable \(Home required\)/);
  assert.doesNotMatch(panel.livePoseView().jointCaption!, /J6 0.0°/);
  assert.throws(() => panel.teachObservation(), /verified position/);
  Object.assign(status.rt_core.status.axes[5]!, { home_valid: true, logical_valid: true, calibration_valid: true, position_state: "trusted" });
  status.rt_core.status.time_ns += 250_000_000;
  t.mock.timers.tick(250); await setImmediate();
  assert.deepEqual(panel.teachObservation().pose.jointsRad.slice(0, 6), positions);
  // Real targets may omit optional velocity PDOs: successive advancing native
  // position observations still allow teaching, but motion restarts the wait.
  for (const axis of status.rt_core.status.axes) Reflect.deleteProperty(axis, "logical_velocity");
  status.rt_core.status.time_ns += 250_000_000;
  t.mock.timers.tick(250); await setImmediate();
  assert.deepEqual(panel.teachObservation().pose.jointsRad.slice(0, 6), positions);
  status.rt_core.status.axes[0]!.logical_position += .01;
  status.rt_core.status.time_ns += 250_000_000;
  t.mock.timers.tick(250); await setImmediate();
  assert.throws(() => panel.teachObservation(), /stationary position samples/);
  t.mock.timers.tick(250); await setImmediate();
  assert.throws(() => panel.teachObservation(), /stationary position samples/, "repeated native frame is not another position observation");
  status.rt_core.status.time_ns += 250_000_000;
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(panel.teachObservation().pose.jointsRad[0], positions[0]! + .01);
  status.rt_core.status.axes[0]!.logical_velocity = .1;
  status.rt_core.status.time_ns += 250_000_000;
  t.mock.timers.tick(250); await setImmediate();
  assert.throws(() => panel.teachObservation(), /stationary/);
  status.rt_core.status.axes[0]!.logical_velocity = 0;
  status.rt_core.status.axes[0]!.calibration_valid = false;
  status.rt_core.status.time_ns += 250_000_000;
  t.mock.timers.tick(250); await setImmediate();
  assert.throws(() => panel.teachObservation(), /verified position/);
  const observed = poses.length;
  // The next poll hangs; the clock runs past the freshness bound with no answer.
  hang = true;
  t.mock.timers.tick(250); await setImmediate();
  t.mock.timers.tick(2300); await setImmediate();
  assert.equal(poses.length, observed, "no fresh status, no new pose");
  assert.throws(() => panel.teachObservation(), /fresh stationary feedback/);
  const stale = panel.livePoseView();
  assert.equal(stale.mode, "stale");
  assert.equal(stale.pose, null);
  assert.equal(stale.caption, "LIVE (STALE) · Rosie 1420 bench · rosie_1420_v1 · status 2550 ms");
});

test("RUN reports native progress and terminal state; NETWORK reports SDK timing", async t => {
  const h = await modelPanel(t, "rosie_1400_v3");
  Object.assign(h.status, { round_trip_ns: 250000000, link_lease_ns: 3000000000 });
  Object.assign(h.status.rt_core.status.execution, { state: "running", segment: 1, identity: { trajectory_digest: "c".repeat(64) } });
  Object.assign(h.status.rt_core.status, { plan_cursor: { active_handle: 5, sample_index: 17, native_clock_ns: 340000000 } });
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(h.panel.networkSummary(), "SDK lease 3000 ms · RTT 250.0 ms");
  assert.match(h.host.querySelector("#dense-readout").textContent, /segment 2 · sample 17/);
  assert.doesNotMatch(h.host.querySelector("#dense-readout").textContent, /clock|340 ms/);
  Object.assign(h.status.rt_core.status.execution, { state: "done" });
  h.status.session.state = "done";
  t.mock.timers.tick(250); await setImmediate();
  assert.match(h.host.querySelector("#dense-readout").textContent, /done/);
});

test("start-position refusal explains recovery without showing an inactive sample or host uptime", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  Object.assign(h.status.rt_core, { reason: "native_rejected" });
  Object.assign(h.status.rt_core.status.execution, { state: "faulted", error: "native_rejected", segment: 0, identity: { trajectory_digest: "c".repeat(64) } });
  Object.assign(h.status.rt_core.status, { plan_cursor: { active_handle: 0, sample_index: 0xffffffff, native_clock_ns: 981234000000 },
    motion: { time_ns: 1, done: true, mode: 0 }, grant: { GrantActive: 1 } });
  Object.assign(h.status.rt_core.status.core, { armed: 0, safety_fault_mask: 1, execution_fault_reasons: 1,
    commissioning_phase: 0, native_home_active_axis_mask: 0,
    recovery: { reason: "none", faults: [{ name: "start_discontinuity", axis_mask: 1, recovery: "reset_clears" }] } });
  t.mock.timers.tick(250); await setImmediate();
  const readout = h.host.querySelector("#dense-readout").textContent;
  assert.match(readout, /Start position does not match plan/);
  assert.doesNotMatch(readout, /Reset clears|reviewed start position|motion was refused/);
  assert.doesNotMatch(readout, /segment 1|sample|4294967295|clock|981234/);
  const feedback = h.host.querySelector("#dense-operator-feedback");
  assert.equal(feedback.hidden, false);
  const text = feedback.options.map(row => row.textContent).join("\n");
  assert.match(text, /Arm the robot to recover/);
  assert.match(text, /Go to plan start, Load to robot and Play/);
  assert.match(text, /Reset fault available/);
  assert.match(text, /start_discontinuity/);
  assert.equal(h.host.querySelector("#dense-play").disabled, true);
  assert.equal(h.host.querySelector("#dense-stop").disabled, false);
});


test("Home confirmation ages with native time even when HTTP keeps answering, then recovers", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  const heading = h.host.querySelector("#dense-home-state");
  assert.equal(heading.textContent, "Homed · 1/1 homed");
  // Ordinary successful polls returning an unchanged native snapshot are not fresh proof.
  for (let i = 0; i < 11; i++) { t.mock.timers.tick(250); await setImmediate(); }
  assert.equal(heading.textContent, "Status stale");
  assert.equal(h.host.querySelector('[data-step="home"]').dataset.state, "pending");
  assert.equal(h.host.querySelector("#dense-home-joints").options[0]!.dataset.homeState, "stale");
  assert.ok(h.panel.statusAgeMs()! >= 2750);
  h.status.rt_core.status.time_ns += 1_000_000;
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(heading.textContent, "Homed · 1/1 homed");
  // A native restart gets its own clock domain, even when its clock is lower.
  h.status.rt_core.status.daemon_incarnation = "restarted";
  h.status.rt_core.status.time_ns = 1;
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(heading.textContent, "Homed · 1/1 homed");
  assert.equal(h.panel.statusAgeMs(), 0);
  h.status.rt_core.status.axes[0]!.home_valid = false;
  h.status.rt_core.status.time_ns++;
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(heading.textContent, "Home required · 0/1 homed");
  assert.equal(h.host.querySelector("#dense-home-details").options.length, 1);
});


test("delayed same-incarnation status cannot replace a newer Home failure with old success", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  h.status.rt_core.status.time_ns = 3_000_000;
  h.status.rt_core.status.axes[0]!.home_valid = false;
  t.mock.timers.tick(250); await setImmediate();
  const heading = h.host.querySelector("#dense-home-state");
  assert.equal(heading.textContent, "Home required · 0/1 homed");
  h.status.rt_core.status.time_ns = 2_000_000;
  h.status.rt_core.status.axes[0]!.home_valid = true;
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(heading.textContent, "Home required · 0/1 homed");
  h.panel.render();
  assert.equal(heading.textContent, "Home required · 0/1 homed", "retained facts also remain current");
  h.status.rt_core.status.time_ns = 4_000_000;
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(heading.textContent, "Homed · 1/1 homed");
});


test("measured-start Play is fenced by capture epoch and exact request identity", async t => {
  const captured = { coordinate_identity: `${"a".repeat(64)}:1:1`, observation_identity: "adapter:daemon", joint_names: ["J1"], joint_values_rad: [0] };
  const h = await modelPanel(t, "rosie_1420_v1", false, false, async () => startTestBlob("rosie_1420_v1", [0,0,0,0,0,0,0,0,0], captured));
  installDenseRecord(h);
  Object.assign(h.status.session, { trajectory_digest: "sha256:source", dry_run: true, state: "loaded" });
  const observe = async () => { h.status.rt_core.status.time_ns++; t.mock.timers.tick(250); await setImmediate(); };
  await observe();
  const play = h.host.querySelector("#dense-play");
  assert.equal(play.disabled, false);
  h.status.rt_core.status.core.home_epoch++;
  await observe();
  assert.equal(play.disabled, true, "same position in another Home generation cannot reuse the captured start");
  h.status.rt_core.status.core.home_epoch--;
  h.project.denseTrajectory!.programDigest = "sha256:other-request";
  await observe();
  assert.equal(play.disabled, true, "equal trajectory content is insufficient when the request identity differs");
  h.requests.length = 0;
  play.disabled = false; play.dispatchEvent(new Event("click")); await setImmediate();
  assert.equal(h.requests.some(path => path.endsWith("/play")), false);
});


test("Home button refuses a restart-required native fault without sending Home", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  Object.assign(h.status.rt_core.description, { capabilities: [{ name: "home", state: "implemented" }] });
  Object.assign(h.status.rt_core.status.core, { armed: 0, safety_fault_mask: 8, commissioning_phase: 0, native_home_active_axis_mask: 0,
    recovery: { reason: "fault_persists", faults: [{ bit: 11, name: "pdo_mapping_mismatch", axis_mask: 8, recovery: "restart_required" }] } });
  h.status.rt_core.status.axes[0]!.home_valid = false;
  h.status.rt_core.status.time_ns++;
  t.mock.timers.tick(250); await setImmediate();
  const home = h.host.querySelector("#dense-home");
  assert.equal(home.disabled, true);
  assert.match(h.host.querySelector('[data-step-summary="home"]').textContent, /pdo_mapping_mismatch.*restart/);
  h.requests.length = 0;
  home.dispatchEvent(new Event("click")); await setImmediate();
  assert.equal(h.requests.some(path => path.endsWith("/home")), false);
});

// Recorded on a Rosie 1430 arm: the J4 closing class. A brake-hold warning is visible but changes
// no Arm, Home or fault gate and offers no fault recovery.
test("a brake-hold warning is shown without blocking Arm or reporting a fault", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  Object.assign(h.status.rt_core.status.core, { armed: 0, axis_enable_mask: 0, safety_fault_mask: 0, commissioning_phase: 0, native_home_active_axis_mask: 0 });
  h.status.session.lease.held = false;
  h.status.rt_core.status.time_ns++;
  t.mock.timers.tick(250); await setImmediate();
  const before = { canArm: h.panel.flowView()?.canArm, canHome: h.panel.flowView()?.canHome, reason: h.panel.armReason() };
  Object.assign(h.status.rt_core.status.axes[0]!, { brake_hold_diagnostic: { severity: "warning", warning_sequence: 1,
    transition: "torque_off", message: "Brake appears to latch after torque off - adjust drive brake hold delay (C05.10).",
    cause: "velocity_cap", phase: "closing", displacement_counts: 24, peak_displacement_counts: 24,
    peak_velocity_counts_per_s: 20479, settled: true, settled_displacement_counts: 24 } });
  h.status.rt_core.status.time_ns++;
  t.mock.timers.tick(250); await setImmediate();
  const feedback = h.host.querySelector("#dense-operator-feedback");
  assert.equal(feedback.hidden, false);
  const text = feedback.options.map(row => row.textContent).join("\n");
  assert.match(text, /Warning: J1 brake warning: Brake appears to latch after torque off - adjust drive brake hold delay \(C05\.10\)\./);
  assert.match(text, /Warning only: nothing to reset, and Arm is not blocked\./);
  assert.doesNotMatch(text, /Arm ended/);
  assert.equal(h.panel.flowView()?.fault, null);
  assert.deepEqual({ canArm: h.panel.flowView()?.canArm, canHome: h.panel.flowView()?.canHome, reason: h.panel.armReason() }, before);
  assert.equal(h.panel.clearFaultsMessage(), "");
});

// A drive-state supervisor refusal ends the Arm attempt after Arm was accepted
// (here J1 never reached Operation Enabled). The reason leads the strip and
// the Arm explanation; it is not a fault, offers no reset, and Arm stays
// available for a retry. Once armed again it is no longer shown.
test("a drive-supervisor refusal explains the ended Arm attempt and leaves Arm retryable", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  Object.assign(h.status.rt_core.status.core, { armed: 0, axis_enable_mask: 0, safety_fault_mask: 0, commissioning_phase: 0, native_home_active_axis_mask: 0 });
  h.status.session.lease.held = false;
  h.status.rt_core.status.time_ns++;
  t.mock.timers.tick(250); await setImmediate();
  const before = { canArm: h.panel.flowView()?.canArm, canHome: h.panel.flowView()?.canHome };
  const message = "J1 did not reach Operation Enabled within 500 ms (statusword 0x0233). Arm ended; retry Arm.";
  Object.assign(h.status.rt_core.status.axes[0]!, { drive_supervisor: { severity: "arm_ended", event_sequence: 1,
    kind: "enable_timeout", cause: "none", message, statusword: 0x0233, arm_ended: true, current: true,
    elapsed_ns: 500_000_000, bound_ns: 500_000_000, displacement_counts: 0, bound_counts: 0 } });
  h.status.rt_core.status.time_ns++;
  t.mock.timers.tick(250); await setImmediate();
  const feedback = h.host.querySelector("#dense-operator-feedback");
  assert.equal(feedback.hidden, false);
  const rows = feedback.options.map(row => row.textContent);
  assert.equal(rows[0], `Arm ended: ${message}`);
  assert.doesNotMatch(rows.join("\n"), /Not a fault; Arm remains available|brake/i);
  assert.equal(h.panel.flowView()?.fault, null);
  assert.deepEqual({ canArm: h.panel.flowView()?.canArm, canHome: h.panel.flowView()?.canHome }, before);
  assert.equal(before.canArm, true);
  assert.equal(h.panel.armReason(), `Arm ended: ${message}`);
  assert.equal(h.panel.clearFaultsMessage(), "");
  // Retried: armed again, so the ended attempt is history.
  Object.assign(h.status.rt_core.status.core, { armed: 1, axis_enable_mask: 1 });
  h.status.rt_core.status.time_ns++;
  t.mock.timers.tick(250); await setImmediate();
  assert.doesNotMatch(h.host.querySelector("#dense-operator-feedback").options.map(row => row.textContent).join("\n"), /Arm ended/);
});

test("Arm after faulting Stop explains restart refusal without sending Arm", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  Object.assign(h.status.rt_core, { reason: "operator_stop" });
  Object.assign(h.status.rt_core.status.core, { armed: 0, safety_fault_mask: 8, execution_fault_reasons: 2048,
    recovery: { faults: [{ name: "pdo_mapping_mismatch", axis_mask: 8, recovery: "restart_required" }] } });
  h.status.session.lease.held = false;
  h.status.rt_core.status.time_ns++;
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(h.panel.flowView()?.fault, "rt_core_fault: 8");
  assert.match(h.panel.armReason(), /pdo_mapping_mismatch.*requires controller restart/);
  h.requests.length = 0;
  h.panel.arm(); await setImmediate();
  assert.equal(h.panel.armConfirmation(), false);
  assert.equal(h.requests.some(path => path.endsWith("/arm")), false);
  assert.match(h.panel.armReason(), /pdo_mapping_mismatch.*requires controller restart/);
});

// Owner 2026-10-11: "re-arming the robot should clear faults too if they
// exist". A resettable drive alarm and an unconfirmed Stop leave the Arm
// button enabled; the Arm step says what Arm will do first, and Arm is sent.
test("a resettable drive alarm and an unconfirmed Stop keep Arm enabled and say Arm resets first", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  Object.assign(h.status.rt_core, { reason: "operator_stop: control_session_stale", stop_uncertain: true });
  Object.assign(h.status.rt_core.status.core, { armed: 0, axis_enable_mask: 0, safety_fault_mask: 16, execution_fault_reasons: 128,
    recovery: { faults: [{ bit: 7, name: "drive_readiness", axis_mask: 1, recovery: "reset_after_condition_clears", outcome: "persists", rehome_axis_mask: 0 }] } });
  Object.assign(h.status.rt_core.status.axes[0]!, { readiness: "faulted", drive_alarm: "Er41.1", drive_alarm_text: "Motor over-temperature due to locked- rotor" });
  h.status.session.lease.held = false;
  h.status.rt_core.status.time_ns++;
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(h.panel.flowView()?.canArm, true);
  assert.equal(h.host.querySelector("#dense-arm-on").disabled, false);
  const summary = h.panel.flowView()!.steps[3]!.summary;
  assert.match(summary, /The last Stop is unconfirmed\. Arm confirms it from the robot's state first/);
  assert.match(summary, /J1 drive alarm Er41\.1 \(Motor over-temperature due to locked- rotor\)\. Arm will reset J1 Er41\.1 first/);
  h.requests.length = 0;
  h.host.querySelector("#dense-arm-on").click();
  h.host.querySelector("#dense-arm-confirm").click();
  await setImmediate();
  assert.equal(h.requests.some(path => path.endsWith("/arm")), true);
});


test("Clear faults sends only reset and reports disarmed success", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  Object.assign(h.status.rt_core.status.core, { armed: 0, axis_enable_mask: 0, safety_fault_mask: 0 });
  h.status.session.lease.held = false;
  h.status.rt_core.status.time_ns++;
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(h.panel.clearFaultsDisabledReason(), null);
  const normalized = await h.client.status();
  const reset = t.mock.method(h.client, "clearFaults", async () => ({ accepted: true, error: null, status: normalized }));
  h.requests.length = 0;
  h.panel.clearFaults(); await setImmediate();
  assert.equal(reset.mock.callCount(), 1);
  assert.equal(h.panel.clearFaultsMessage(), "Faults cleared. Robot remains disarmed.");
  assert.equal(h.requests.some(p => /\/(arm|home|play|move)$/.test(p)), false);
  assert.equal(h.panel.clearingFaults(), false);
});

test("Clear faults preserves restart-required diagnosis instead of claiming success", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  Object.assign(h.status.rt_core.status.core, { armed: 0, axis_enable_mask: 0, safety_fault_mask: 8,
    recovery: { faults: [{ name: "pdo_mapping_mismatch", axis_mask: 8, recovery: "restart_required" }] } });
  h.status.session.lease.held = false;
  h.status.rt_core.status.time_ns++;
  t.mock.timers.tick(250); await setImmediate();
  const normalized = await h.client.status();
  t.mock.method(h.client, "clearFaults", async () => ({ accepted: false, error: "fault_persists", status: normalized }));
  h.panel.clearFaults(); await setImmediate();
  assert.match(h.panel.clearFaultsMessage(), /Not cleared: pdo_mapping_mismatch: controller restart and recovery required/);
  assert.equal(h.panel.armConfirmation(), false);
  h.panel.dismissClearFaults();
  assert.equal(h.panel.clearFaultsMessage(), "");
});

test("Clear faults refuses armed, enabled or stale status and suppresses duplicate requests", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  const pending = deferred<import("./denseClient.ts").DenseArmResult>();
  const reset = t.mock.method(h.client, "clearFaults", () => pending.promise);
  h.panel.clearFaults();
  assert.equal(reset.mock.callCount(), 0);
  Object.assign(h.status.rt_core.status.core, { armed: 0, axis_enable_mask: 1, safety_fault_mask: 0 });
  h.status.rt_core.status.time_ns++;
  t.mock.timers.tick(250); await setImmediate();
  h.panel.clearFaults(); assert.equal(reset.mock.callCount(), 0);
  Object.assign(h.status.rt_core.status.core, { axis_enable_mask: 0 });
  h.status.rt_core.status.time_ns++;
  t.mock.timers.tick(250); await setImmediate();
  const normalized = await h.client.status();
  h.panel.clearFaults(); h.panel.clearFaults();
  assert.equal(reset.mock.callCount(), 1);
  pending.resolve({ accepted: true, error: null, status: normalized }); await setImmediate();
  t.mock.timers.tick(5000); await setImmediate();
  h.panel.clearFaults(); assert.equal(reset.mock.callCount(), 1);
});

// Owner 2026-10-11: Run program is the default, one press. The panel posts
// the kept plan's Load body with the Jog speed, asks the server's question
// once in the All joints to 0° dialog, sends the confirmation back with the
// joints it showed, and after the program completed the same button reads
// Run again. The separate Go to plan start, Load and Play stay under Advanced.
test("Run program: one press, one question, Run again after completion, STOP during the question sends nothing", async t => {
  const asked = [deferred<boolean>(), deferred<boolean>()];
  const confirmations: RecoverConfirmation[] = [];
  const h = await modelPanel(t, "rosie_1420_v1", false, false, undefined, 1, async confirmation => {
    confirmations.push(confirmation);
    return asked[confirmations.length - 1]!.promise;
  });
  installDenseRecord(h);
  const button = h.host.querySelector("#dense-run-program");
  assert.equal(button.textContent, "▶ Run program");
  assert.equal(button.disabled, false);
  assert.match(h.host.innerHTML, /<details id="dense-run-advanced"[\s\S]*id="dense-plan-start"[\s\S]*id="dense-load"[\s\S]*id="dense-play"[\s\S]*<\/details>/, "the separate steps stay, under Advanced");
  const bodies: Record<string, unknown>[] = [];
  const question: RunResult = normalizeRunResult({ state: "confirm", action: "Move to program start and run", needs_move: true, steps: [{ step: "start", outcome: "done" }],
    joints: [{ axis: 0, id: "J1", from_deg: 40, to_deg: 10, delta_deg: -30, moves: true }] });
  t.mock.method(h.client, "runProgram", async (body: Record<string, unknown>) => {
    bodies.push(body);
    if (body.confirm_move !== true) return { result: question, status: null };
    Object.assign(h.status.session, { id: "run-1", trajectory_digest: "sha256:source", plan_id: "plan", dry_run: true, state: "playing" });
    return { result: normalizeRunResult({ state: "playing", session_id: "run-1", steps: [{ step: "move", outcome: "done" }, { step: "load", outcome: "done" }, { step: "play", outcome: "done" }] }), status: normalizeDenseStatus(h.status) };
  });
  button.click();
  for (let i = 0; i < 3; i++) await setImmediate();
  assert.equal(bodies.length, 1);
  assert.deepEqual({ ...bodies[0], fraction: undefined }, { dry_run: true, trajectory_digest: "sha256:source", plan_id: "plan", program_id: "program", program_digest: "sha256:program",
    manifest_revision: 1, plan_revision: 1, fraction: undefined, confirm_arm: false, confirm_move: false });
  assert.ok(Number(bodies[0]!.fraction) > 0 && Number(bodies[0]!.fraction) <= 1, "Go to plan start runs at the Jog speed");
  assert.equal(confirmations[0]!.title, "Move to program start and run");
  assert.deepEqual(confirmations[0]!.lines, ["J1  40.00° → 10.00°  Δ −30.00°"]);
  assert.match(confirmations[0]!.warning, /sweep through the cell/);
  asked[0]!.resolve(true);
  for (let i = 0; i < 5; i++) await setImmediate();
  assert.equal(bodies.length, 2);
  assert.equal(bodies[1]!.confirm_move, true);
  assert.equal(bodies[1]!.confirm_arm, false);
  assert.deepEqual(bodies[1]!.confirmed, [{ axis: 0, id: "J1", from_deg: 40, to_deg: 10, moves: true }]);
  assert.equal(h.host.querySelector("#dense-run-result").textContent, "Running (moved to the program's start · loaded)");

  // Completed: native retired the program; the button is Run again.
  Object.assign(h.status.session, { state: "done" });
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(button.textContent, "▶ Run again");
  assert.equal(button.disabled, false);

  // Run again asks again (the arm is at the program's end); STOP while the
  // question is open ends the press and the late "yes" sends nothing.
  button.click();
  for (let i = 0; i < 3; i++) await setImmediate();
  assert.equal(bodies.length, 3);
  h.host.querySelector("#dense-stop").click();
  asked[1]!.resolve(true);
  for (let i = 0; i < 5; i++) await setImmediate();
  assert.equal(bodies.length, 3, "a confirmation answered after STOP is never sent");
  assert.equal(h.host.querySelector("#dense-run-result").textContent, "Run program ended by STOP");
});

test("Run program shows the server's refusal sentence and declines without a second press", async t => {
  const h = await modelPanel(t, "rosie_1420_v1", false, false, undefined, 1, async () => false);
  installDenseRecord(h);
  const bodies: Record<string, unknown>[] = [];
  let answer = normalizeRunResult({ state: "confirm", action: "Arm and run", needs_arm: true, joints: [{ axis: 0, id: "J1", from_deg: 10, to_deg: 10, delta_deg: 0, moves: false }] });
  t.mock.method(h.client, "runProgram", async (body: Record<string, unknown>) => { bodies.push(body); return { result: answer, status: null }; });
  h.host.querySelector("#dense-run-program").click();
  for (let i = 0; i < 5; i++) await setImmediate();
  assert.equal(bodies.length, 1, "Cancel sends nothing more");
  assert.equal(h.host.querySelector("#dense-run-result").textContent, "Run program cancelled: nothing was armed, moved or played");
  answer = { ...normalizeRunResult({ run: { state: "refused", steps: [{ step: "reset", outcome: "done" }], refusal: { step: "load", code: "trajectory_limits", detail: "Load refused: trajectory_limits: J3 velocity" } } }), code: "trajectory_limits" };
  h.host.querySelector("#dense-run-program").click();
  for (let i = 0; i < 5; i++) await setImmediate();
  assert.equal(h.host.querySelector("#dense-run-result").textContent, "faults reset; then Load refused: trajectory_limits: J3 velocity");
  // ...and in the toast, not only under the button (2026-10-12 steamdeck-pendant).
  const refusal = h.notices.at(-1)!;
  assert.equal(refusal.kicker, "RUN PROGRAM REFUSED");
  assert.equal(refusal.text, "faults reset; then Load refused: trajectory_limits: J3 velocity");
  assert.deepEqual(refusal.details, ["Before the refusal: faults reset.", "trajectory_limits at step load"]);
  // A gate's 409 without a run body (focus, cell, maintenance) throws in the
  // client; it reaches the toast with its code and sentence too.
  t.mock.method(h.client, "runProgram", async () => { throw new DenseExecutionError("client_cell_mismatch", "This browser was opened for cell-a, but this OLP server drives cell-b: nothing was sent.", 409); });
  h.host.querySelector("#dense-run-program").click();
  for (let i = 0; i < 5; i++) await setImmediate();
  assert.equal(h.notices.at(-2), null, "the press clears the last toast first");
  assert.equal(h.notices.at(-1)!.text, "This browser was opened for cell-a, but this OLP server drives cell-b: nothing was sent.");
  assert.deepEqual(h.notices.at(-1)!.details, ["Nothing was armed, moved or played.", "client_cell_mismatch"]);
});

// Owner 2026-10-11: loop mode. The repeat chosen beside the button goes in
// the press, the one question shows the planned return move, every status
// shows the cycle counter and the last cycle time, and Finish cycle asks the
// server to end at the program start.
test("Loop until stopped: one question, cycle counter and last cycle time, Finish cycle", async t => {
  const confirmations: RecoverConfirmation[] = [];
  const h = await modelPanel(t, "rosie_1420_v1", false, false, undefined, 1, async confirmation => { confirmations.push(confirmation); return true; });
  installDenseRecord(h);
  const repeat = h.host.querySelector("#dense-run-repeat");
  repeat.value = "loop"; repeat.dispatchEvent(new Event("change"));
  assert.equal(h.panel.repeatMode(), "loop", "Plan asks the planner for the return move");
  assert.equal(h.host.querySelector("#dense-run-program").textContent, "▶ Loop");
  const bodies: Record<string, unknown>[] = [];
  t.mock.method(h.client, "runProgram", async (body: Record<string, unknown>) => {
    bodies.push(body);
    if (body.confirm_loop !== true) {
      return { result: normalizeRunResult({ state: "confirm", action: "Loop until stopped", joints: [], loop: { mode: "loop", return_move: { duration_s: 3, joints: [{ axis: 0, id: "J1", from_deg: 50, to_deg: 0, delta_deg: -50, moves: true }] } } }), status: null };
    }
    Object.assign(h.status, { loop: { mode: "loop", cycles: 0, state: "running", owner: "Chrome", attended: true, return_move: true } });
    Object.assign(h.status.session, { state: "playing" });
    return { result: normalizeRunResult({ state: "playing", session_id: "s", loop: { mode: "loop" }, steps: [{ step: "load", outcome: "done" }, { step: "play", outcome: "done" }] }), status: normalizeDenseStatus(h.status) };
  });
  h.host.querySelector("#dense-run-program").click();
  for (let i = 0; i < 6; i++) await setImmediate();
  assert.deepEqual(bodies.map(b => [b.repeat, b.confirm_loop ?? false]), [["loop", false], ["loop", true]]);
  assert.equal(confirmations.length, 1, "asked once, not per cycle");
  assert.equal(confirmations[0]!.eyebrow, "LOOP UNTIL STOPPED");
  assert.deepEqual(confirmations[0]!.lines, ["J1  50.00° → 0.00°  Δ −50.00°"]);
  Object.assign(h.status, { loop: { mode: "loop", cycles: 4, state: "running", last_cycle_s: 11.25, owner: "Chrome", attended: true, return_move: true } });
  t.mock.timers.tick(250); await setImmediate();
  assert.equal(h.host.querySelector("#dense-run-loop").textContent, "Cycle 5 · last cycle 11.3 s");
  assert.equal(h.host.querySelector("#dense-run-program").disabled, true, "no second press while a loop runs");
  assert.equal(h.host.querySelector("#dense-run-repeat").disabled, true);
  const finish = h.host.querySelector("#dense-run-finish");
  assert.equal(finish.hidden, false);
  let finished = 0;
  t.mock.method(h.client, "finishLoop", async () => { finished++; return null; });
  finish.click();
  await setImmediate();
  assert.equal(finished, 1);
  // A refused Finish cycle says so in the toast; the loop keeps running.
  t.mock.method(h.client, "finishLoop", async () => { throw new DenseExecutionError("loop_owner_only", "Only Chrome, which started the loop, can finish it; STOP stops it from anywhere", 409); });
  finish.click();
  for (let i = 0; i < 3; i++) await setImmediate();
  assert.equal(h.notices.at(-1)!.kicker, "FINISH CYCLE REFUSED");
  assert.equal(h.notices.at(-1)!.text, "Only Chrome, which started the loop, can finish it; STOP stops it from anywhere");
});

test("the outputs block under Run follows the polled status: hidden without an io block, then gas ON", async t => {
  const h = await modelPanel(t, "rosie_1420_v1");
  const outputs = h.host.querySelector("#dense-outputs");
  assert.equal(outputs.hidden, true, "a cell with no io block shows no outputs");
  const observation = h.status.rt_core as { description: Record<string, unknown>; status: { core: Record<string, unknown> } };
  observation.description.io = { torch_qualified: false, outputs: [{ name: "gas", class: "process", expiry_ns: 200_000_000 }] };
  observation.status.core.io = { time_ns: 1_000_000, cycle: 1, armed: true, reason: 0, inputs: [],
    outputs: [{ name: "gas", intent: true, commanded: true, readback: true, valid: true, expiry_ns: 150_000_000, observed_ns: 1, changed_ns: 1, marker_sample_ns: 1, marker_cycle: 1, bit: 0, torch: false }] };
  t.mock.timers.tick(250); await setImmediate(); await setImmediate();
  assert.equal(outputs.hidden, false);
  assert.equal(outputs.dataset.tone, "on");
});
