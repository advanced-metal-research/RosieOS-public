import test from "node:test";
import assert from "node:assert/strict";
import { setImmediate } from "node:timers/promises";
import { CapabilitiesDigest, ContractVersion, type Description, type Event, type ProcessStatus } from "../../../../../rt-core/clients/ts/rt_control_api_generated.ts";
import { manifestFromRTCore, rtCoreTargetText } from "./manifest.ts";
import { createProject } from "../domain/project.ts";
import { DensePanel } from "../execution/densePanel.ts";
import { mapRTCoreStatus } from "./rtCoreStatus.ts";
import { DenseExecutionClient, denseExecutionState, denseProgramPlayable, normalizeDenseCapabilities, normalizeDenseStatus } from "../execution/denseClient.ts";

function description(): Description {
  return { backend: "simulation", contract_version: ContractVersion, capabilities_digest: CapabilitiesDigest,
    machine_sha256: "a".repeat(64), deployment_sha256: "b".repeat(64), axes: [{ index: 0, id: "J1", position_unit: "rad" }],
    capabilities: ["acquire", "renew", "release", "enable", "arm", "home", "stop", "prepare_program", "start_program", "status", "subscribe_events"].map(name => ({ name, state: "implemented", transport: "HTTP" })),
  } as Description;
}

test("TestReviewRound2MergedCapabilitiesIdentity", () => {
  // Generated schema identity includes the uncapped HM35 (v3) verified-delta
  // restore policy (no maximum_delta_counts), the per-axis brake-hold warning
  // with slip and energized fields, the drive-state supervisor event, and the
  // path_jog capability (rt-core protocol 1.14, Cartesian path holds), and the
  // read-only GET /v1/maintenance route, manual I/O, input waits and scoped
  // monitors, and per-axis drive health (load and monitor, docs/decisions/0001).
  // Linear coordinates retire the feedback/command-wrap and anchor-period fields.
  // Fault records carry the optional reset_preserves_home advisory.
  const mergedDigest = "0948a57f2ce6c5928e9ecb94cb8725046ec03f29684deb96d3e76a4bd8240662";
  // Previous accepted identity, before reset_preserves_home: refused like every
  // other digest; the OLP server, UI and Deck ship with rt-core from one tree.
  const beforeResetPreservesHomeDigest = "c607bcc67cb5a6cc49eb05b64cecc80492e4799c6a1baa77c24159b1b3b64d46";
  // Previous accepted identities: main before linear coordinates, the linear-mode
  // branch before drive health, and each side before that. All are refused like
  // every other digest; the OLP server and UI ship with rt-core from one tree.
  const beforeLinearCoordinatesOnMainDigest = "9812b73dd20bfa761ba3185d77ced535cd42ceae42c747ce03418a9cabe70edf";
  const beforeDriveHealthOnLinearDigest = "f02e9f057e6ae580ecfc2737e6a683bae4cfffe5c7935540684f28722eac084d";
  const beforeLinearCoordinatesDigest = "725c5bbb8cf6c99deacda861976f5a01415ab04c8bd633a4d7ee200f302fa1ce";
  const beforeDriveHealthDigest = "2a3b54dac880e1007d20ec690f9ca60450bb9e07b33e3fc19a7a8bff95c44c91";
  // Previous accepted identity, before the maintenance route: refused like
  // every other digest; the OLP UI and the Deck are re-pinned with the deploy.
  const beforeMaintenanceDigest = "b1e28f06dfc586c7971e93a7524df6a1830719939d1591f0caec9a1008e599ad";
  // Previous accepted identity, before path_jog: such a native cannot run a
  // Cartesian path hold and must be refused, not silently lose holds.
  const beforePathJogDigest = "201fbe819365b7310c7f2a949e300c413eb77fd8f1ad53cef0360db008a4bdb2";
  // Previous accepted identity, before drive-state supervisor events; a native
  // without them must be refused rather than silently lose the Arm-ended reason.
  const beforeSupervisorDigest = "64dad2af2ad1f3bb721eb28be72c538b0248756a6e2c2f66456e3a57dfc00b8e";
  // Previous accepted identities: with the per-joint verified-delta count cap,
  // before brake-hold warnings and before their slip/energized fields; such a
  // native must be refused rather than silently misreport its restore policy
  // or lose the warning display.
  const cappedVerifiedDeltaDigest = "22bda50c7d61a911c7fc8428cf904414109630c467b4de6e3668475c4a774fd3";
  const beforeBrakeWarningDigest = "39f6a2c641593ef41aa56d77ffe259e57a4d6939570ccad5f63b6b82732db46b";
  const beforeBrakeSlipDigest = "164365c684c6a9119d5ea2fdeaf0453898cc774a9b0b0b60edbd0263f6d8acd1";
  const currentReferenceOnlyDigest = "26b50b6704a03bc72efb2469cf4ab52489dbe6986ff5fd7427e2d5fab272b1ab";
  const v5OnlyDeltaDigest = "c70cb667cdfc8f3ba553ac611b312dd72634a42f9ff6fdebf5d186fc26c95878";
  const beforeReferenceDigest = "d63b2aee7bbcb7e4f11eca7148ce9ffda0e5d8db421f165d20918ccb4f119158";
  // Measured previous accepted identity at contract_version 1, before the description merge.
  const previousDigest = "b85f235c2dcfdb4a1eb5ee03c94ccc1289537aa6f671c6f84e96913cbcbf1859";
  assert.equal(CapabilitiesDigest, mergedDigest);
  for (const digest of [beforeResetPreservesHomeDigest, beforeLinearCoordinatesOnMainDigest, beforeDriveHealthOnLinearDigest, beforeLinearCoordinatesDigest, beforeDriveHealthDigest, beforeMaintenanceDigest, beforePathJogDigest, cappedVerifiedDeltaDigest, currentReferenceOnlyDigest, v5OnlyDeltaDigest, beforeBrakeWarningDigest, beforeBrakeSlipDigest, beforeSupervisorDigest]) {
    assert.throws(() => manifestFromRTCore({ ...description(), capabilities_digest: digest }), /incompatible/);
  }
  assert.throws(() => manifestFromRTCore({ ...description(), capabilities_digest: beforeReferenceDigest }), /incompatible/);
  assert.equal(manifestFromRTCore({ ...description(), capabilities_digest: mergedDigest }).programExecution, true);
  assert.throws(() => manifestFromRTCore({ ...description(), capabilities_digest: previousDigest }), /incompatible/);
  assert.throws(() => normalizeDenseCapabilities({ backend: "rt_core", available: true, rt_core: { description: { ...description(), capabilities_digest: previousDigest } } }), /incompatible/);
});

function status(): ProcessStatus {
  return { time_ns: 1000000, adapter_incarnation: "adapter", daemon_incarnation: "daemon", core: { armed: 1, safety_fault_mask: 0 },
    execution: { state: "completed", generation: 2 }, axes: [{ time_ns: 1000000, logical_position: 0.25, logical_target: 0.25,
      logical_valid: true, home_valid: true, readiness: "ready", statusword: 39, position_counts: 42 }],
  } as ProcessStatus;
}

test("rt_core manifest uses generated contract and leaves v4 evidence unavailable", () => {
  const d = description();
  const manifest = manifestFromRTCore(d);
  assert.equal(manifest.programExecution, true);
  assert.equal(manifest.nativeBackend, "simulation");
  assert.equal(manifest.axes, d.axes);
  for (const field of [manifest.pairRevision, manifest.factoryHash, manifest.urdf, manifest.homeProfile]) assert.equal(field, null);
  const caps = normalizeDenseCapabilities({ backend: "rt_core", available: true, rt_core: { description: d } });
  assert.equal(caps.rtCore?.nativeHome, true);
  assert.throws(() => manifestFromRTCore({ ...d, capabilities_digest: "wrong" }), /incompatible/);
  assert.equal(manifestFromRTCore({ ...d, capabilities: [] }).programExecution, false);
});

test("rt_core status preserves logical validity, Home, faults and typed refusal", () => {
  const s = status();
  assert.equal(mapRTCoreStatus(s).axes[0]?.position, 0.25);
  assert.equal(mapRTCoreStatus(s).ready, true);
  s.axes[0]!.logical_valid = false;
  s.axes[0]!.home_valid = false;
  s.axes[0]!.readiness = "home_required";
  assert.equal(mapRTCoreStatus(s).axes[0]?.position, null);
  assert.equal(mapRTCoreStatus(s).axes[0]?.homeValid, false);
  assert.equal(mapRTCoreStatus(s).ready, false);
  s.core.safety_fault_mask = 4;
  assert.equal(mapRTCoreStatus(s).reason, "rt_core_fault: 4");
  assert.equal(mapRTCoreStatus(s, [], "control_session_stale").reason, "control_session_stale");
  const mapped = normalizeDenseStatus({ backend: "rt_core", available: true, rt_core: { status: s, reason: "control_session_stale" } });
  assert.equal(mapped.daemonError, "control_session_stale");
  assert.equal(mapped.axes?.[0]?.qRad, null);
  assert.equal(mapped.axes?.[0]?.homeCounts, null);
  assert.equal(mapped.armed, null);
});

test("rt_core lost events and malformed observations fail closed", () => {
  assert.equal(mapRTCoreStatus(status(), [{ type: "events_dropped" } as Event]).ready, false);
  assert.equal(normalizeDenseStatus({ backend: "rt_core", rt_core: {} }).armed, null);
  assert.equal(normalizeDenseStatus({ backend: "rt_core", rt_core: {} }).daemonError, "rt_core status unavailable");
});

test("coordinate identity requires an exact configuration and Home generation", () => {
  const s = status();
  assert.equal(mapRTCoreStatus(s).coordinateIdentity, null);
  Object.assign(s.core, { configuration_sha256: "config", configuration_epoch: 4, home_epoch: 31 });
  assert.equal(mapRTCoreStatus(s).coordinateIdentity, "config:4:31");
  s.core.home_epoch++;
  assert.equal(mapRTCoreStatus(s).coordinateIdentity, "config:4:32");
  s.core.configuration_epoch++;
  assert.equal(mapRTCoreStatus(s).coordinateIdentity, "config:5:32");
  s.core.home_epoch = Number.MAX_SAFE_INTEGER + 1;
  assert.equal(mapRTCoreStatus(s).coordinateIdentity, null);
});

test("public controller state enables prepared Play and fences replay after completion or Stop", () => {
  const current = normalizeDenseStatus({ backend: "rt_core", available: true,
    session: { id: "session", state: "loaded", trajectory_digest: "digest" }, rt_core: { status: status() } });
  assert.equal(denseExecutionState(current), "loaded");
  assert.equal(denseProgramPlayable(current, "digest"), true);
  assert.equal(denseProgramPlayable(current, "other"), false);
  assert.equal(denseProgramPlayable({ ...current, armed: false }, "digest"), false);
  assert.equal(denseProgramPlayable({ ...current, daemonError: "transport lost" }, "digest"), false);
  for (const state of ["playing", "done", "stopped"]) {
    current.session!.state = state;
    assert.equal(denseExecutionState(current), state);
    assert.equal(denseProgramPlayable(current, "digest"), false);
  }
});

test("remote target display rejects credentials and paths", () => {
  assert.equal(manifestFromRTCore(description(), "BENCH.example:443").addressHost, "bench.example");
  assert.equal(manifestFromRTCore(description(), "[::1]:8443").addressHost, "[::1]:8443");
  for (const host of ["https://user:secret@host", "user:secret@host", "/private/key.pem", "host/private/key", "host?key=secret", "host#secret"]) {
    const manifest = manifestFromRTCore(description(), host);
    assert.equal(manifest.addressHost, null);
    assert.doesNotMatch(rtCoreTargetText(manifest), /secret|private/);
  }
});

// A DOM boundary exposes the actual panel's selection, text and Play control;
// it prevents a pure manifest test from overlooking a fixed nine-axis UI gate.
class TargetElement extends EventTarget {
  innerHTML = ""; textContent = ""; title = ""; value = "";
  hidden = false; disabled = false;
  dataset: Record<string, string> = {};
  style: Record<string, string> = {};
  private classes = new Set<string>();
  classList = {
    add: (...tokens: string[]) => { for (const token of tokens) this.classes.add(token); },
    remove: (...tokens: string[]) => { for (const token of tokens) this.classes.delete(token); },
    contains: (token: string) => this.classes.has(token),
    toggle: (token: string, force?: boolean) => {
      const present = force ?? !this.classes.has(token);
      if (present) this.classes.add(token); else this.classes.delete(token);
      return present;
    },
  };
  options: TargetElement[] = [];
  nodes = new Map<string, TargetElement>();
  querySelector(selector: string): TargetElement {
    if (!this.nodes.has(selector)) this.nodes.set(selector, new TargetElement());
    return this.nodes.get(selector)!;
  }
  append(...options: TargetElement[]): void { this.options.push(...options); }
  setAttribute(_name: string, _value: string): void {}
  replaceChildren(): void { this.options = []; this.textContent = ""; }
  remove(): void { /* The fixture has no prior options. */ }
  click(): void {
    assert.equal(this.disabled, false);
    this.dispatchEvent(new Event("click"));
  }
}

test("six-axis remote Rosie 1420 renders and enables prepared Play", async (t) => {
  t.mock.timers.enable({ apis: ["setTimeout", "setInterval"] });
  for (const [name, value] of Object.entries({
    window: Object.assign(new EventTarget(), { setTimeout, clearTimeout, setInterval, clearInterval }),
    document: Object.assign(new EventTarget(), { createElement: () => new TargetElement() }),
    sessionStorage: { getItem: () => "", setItem: () => undefined },
  })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => { if (previous) Object.defineProperty(globalThis, name, previous); else Reflect.deleteProperty(globalThis, name); });
  }
  const d = description();
  d.backend = "ethercat";
  d.axes = Array.from({ length: 6 }, (_, index) => ({ ...d.axes[0]!, index, id: `J${index + 1}`, completion_tolerance: .000002 }));
  const s = status();
  s.axes = Array.from({ length: 6 }, (_, index) => ({ ...s.axes[0]!, index, calibration_valid: true, position_state: "trusted", logical_velocity: 0 }));
  Object.assign(s.core, { configuration_sha256: "a".repeat(64), configuration_epoch: 1, home_epoch: 1 });
  s.execution.state = "prepared";
  const project = createProject({ robotModel: "rosie_1420_v1" });
  project.denseTrajectory = {
    schema: "robot.v4.dense-joint-trajectory.v1", trajectoryDigest: "sha256:trajectory", requestSha256: "request", planId: "plan", programId: "program",
    programDigest: "sha256:program", manifestRevision: 1, planRevision: 1, createdAt: "fixture", dtS: 0.01,
    totalSampleCount: 2, totalDurationS: 0.01, bytes: 1, segments: [], sourceProjectRevision: 1,
  };
  const binding = { model_id: "rosie_1420_v1", robot_description_sha256: "sha256:robot", machine_planning_calibration_sha256: "sha256:calibration" };
  const header = new TextEncoder().encode(JSON.stringify({ schema: project.denseTrajectory.schema, trajectory_digest: "sha256:trajectory", robot_cell: binding, dt_s: .01, segments: [{ sample_count: 1 }] }));
  const bytes = new Uint8Array(4 + header.length + 153), view = new DataView(bytes.buffer);
  view.setUint32(0, header.length); bytes.set(header, 4);
  for (let index = 0; index < 6; index++) view.setFloat64(4 + header.length + 8 + index * 8, .25);
  const loaded = { backend: "rt_core", available: true, robot_cell: { ...binding, configured: true, valid: true },
    session: { id: "session", state: "loaded", trajectory_digest: "sha256:trajectory", dry_run: true }, rt_core: { status: s } };
  const requests: string[] = [];
  const client = new DenseExecutionClient(async (url, init) => {
    const op = String(url).split("/").at(-1)!;
    requests.push(op);
    if (op === "capabilities") return Response.json({ backend: "rt_core", available: true, daemon_reachable: true,
      rt_core: { description: d, address_host: "127.0.0.1:8443", key_file: "/private/never-render.pem" } });
    if (op === "status") return Response.json(loaded);
    if (op === "load") return Response.json({ session_id: "session", status: loaded });
    if (op === "play") {
      assert.deepEqual(JSON.parse(String(init?.body)), { session_id: "session" });
      return Response.json({ ...loaded, session: { ...loaded.session, state: "playing" } });
    }
    throw new Error(`unexpected ${op}`);
  });
  const panel = new DensePanel({ client, getProject: () => project, readTrajectory: async () => bytes,
    highlightNode: () => undefined, status: () => undefined, onLockChange: () => undefined,
    onTargetChange: () => undefined, onRobotPose: () => undefined, onRobotMotion: () => undefined, wantsFastPoll: () => false });
  const host = new TargetElement();
  panel.mount(host as unknown as HTMLElement, host as unknown as HTMLElement);
  await setImmediate();
  const target = host.querySelector("#dense-target");
  assert.equal(target.options.length, 1);
  assert.equal(target.options[0]!.textContent, "rt-core remote 127.0.0.1:8443 · ethercat · 6 axes (J1, J2, J3, J4, J5, J6)");
  target.value = "joint-daemon";
  target.dispatchEvent(new Event("change"));
  assert.match(host.querySelector("#dense-target-state").textContent, /remote 127\.0\.0\.1:8443.*6 axes/);
  assert.doesNotMatch(host.querySelector("#dense-target-state").textContent, /private|pem|J7|J8|J9/);
  assert.equal(host.querySelector("#dense-play").disabled, true);
  t.mock.timers.tick(300);
  await setImmediate();
  host.querySelector("#dense-load").click();
  await setImmediate();
  assert.equal(project.robot.model, "rosie_1420_v1");
  assert.equal(host.querySelector("#dense-load-progress").classList.contains("is-done"), true);
  assert.equal(host.querySelector("#dense-play").disabled, false);
  host.querySelector("#dense-play").click();
  await setImmediate();
  assert.deepEqual(requests, ["capabilities", "status", "state", "state", "load", "play"]);
});
