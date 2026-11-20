import assert from "node:assert/strict";
import test from "node:test";
import type { DenseCapabilities, DenseStatus } from "./denseClient.ts";
import { deriveLivePose, mapLivePose } from "./livePose.ts";
import { STATUS_FRESHNESS_MS } from "./machineFlow.ts";
import { CANONICAL_JOINT_ORDER } from "../viewer/models.ts";

const target = (model_id: "rosie_1420_v1" | "rosie_1400_v3", label = "Rosie 1420 bench") =>
  ({ selection_generation: "g1", cell_id: "la-bench", model_id, label, backend: "ethercat", host: "bench-relay" }) as NonNullable<DenseStatus["target"]>;

function rtCoreCaps(ids: readonly string[], unit = "rad"): DenseCapabilities {
  return { backend: "rt_core", cell: "la-bench", daemonReachable: true, rtCore: { axes: ids.map((id, index) => ({ id, index, position_unit: unit })) } } as unknown as DenseCapabilities;
}

function rtCoreStatus(model: "rosie_1420_v1" | "rosie_1400_v3", positions: readonly (number | null)[]): DenseStatus {
  return {
    backend: "rt_core", target: target(model), available: true, session: null, daemon: null, daemonError: null, cursor: null, armed: true, axes: null,
    rtCore: { ready: true, coreArmed: true, armed: true, reason: null, axes: positions.map(position => ({ position, target: position, velocity: 0, homeValid: true, coordinateValid: true, readiness: "ready", statusword: 0, positionCounts: 0 })) },
  } as unknown as DenseStatus;
}

test("live pose maps rosie_1420_v1's six described axes onto J1..J6 and leaves J7..J9 at home", () => {
  // The owner's screenshot: J1 0.0498 rad, J2 0.1937 rad while jogging the LA bench.
  const pose = mapLivePose(rtCoreStatus("rosie_1420_v1", [0.0498, 0.1937, -0.25, 0.5, 1, -1.5]), rtCoreCaps(["J1", "J2", "J3", "J4", "J5", "J6"]), null, 1000, "rosie_1400_v3");
  assert.ok(pose);
  assert.equal(pose.modelId, "rosie_1420_v1");
  assert.deepEqual(pose.jointOrder, [...CANONICAL_JOINT_ORDER]);
  assert.deepEqual(pose.jointsRad, [0.0498, 0.1937, -0.25, 0.5, 1, -1.5, 0, 0, 0]);
  assert.equal(pose.source, "live");
  assert.equal(pose.observedAtMs, 1000);
});

test("live pose maps rosie_1400_v3's nine described axes onto J1..J9 by axis id, not by wire index", () => {
  const ids = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"];
  const positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9];
  const pose = mapLivePose(rtCoreStatus("rosie_1400_v3", positions), rtCoreCaps(ids), null, 5, "rosie_1420_v1");
  assert.ok(pose);
  assert.equal(pose.modelId, "rosie_1400_v3");
  assert.deepEqual(pose.jointsRad, positions);
  // A description that lists the positioner first still lands each id on its joint.
  const shuffled = mapLivePose(rtCoreStatus("rosie_1400_v3", [0.7, 0.1]), rtCoreCaps(["J7", "J1"]), null, 5, "rosie_1420_v1");
  assert.deepEqual(shuffled?.jointsRad, [0.1, 0, 0, 0, 0, 0, 0.7, 0, 0]);
  // Ids the model does not know fall back to the wire index.
  const unnamed = mapLivePose(rtCoreStatus("rosie_1400_v3", [0.11, 0.22]), rtCoreCaps(["axis_a", "axis_b"]), null, 5, "rosie_1420_v1");
  assert.deepEqual(unnamed?.jointsRad, [0.11, 0.22, 0, 0, 0, 0, 0, 0, 0]);
});

test("live pose keeps the last shown value for an axis without a valid position and ignores non-radian axes", () => {
  const caps = rtCoreCaps(["J1", "J2", "J3", "J4", "J5", "J6"]);
  const first = mapLivePose(rtCoreStatus("rosie_1420_v1", [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]), caps, null, 1, "rosie_1420_v1")!;
  const second = mapLivePose(rtCoreStatus("rosie_1420_v1", [0.15, null, 0.3, 0.4, 0.5, 0.6]), caps, first, 2, "rosie_1420_v1")!;
  assert.deepEqual(second.jointsRad, [0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0, 0, 0]);
  // A previous pose for another model is not carried across.
  const other = mapLivePose(rtCoreStatus("rosie_1400_v3", [0.9]), rtCoreCaps(["J1"]), first, 3, "rosie_1420_v1")!;
  assert.deepEqual(other.jointsRad, [0.9, 0, 0, 0, 0, 0, 0, 0, 0]);
  // Counts or millimetres are not joint angles; without a description the unit is unknown.
  assert.equal(mapLivePose(rtCoreStatus("rosie_1420_v1", [1, 2]), rtCoreCaps(["J1", "J2"], "counts"), null, 4, "rosie_1420_v1"), null);
  assert.equal(mapLivePose(rtCoreStatus("rosie_1420_v1", [1, 2]), null, null, 4, "rosie_1420_v1"), null);
  // Every axis invalid: nothing to draw.
  assert.equal(mapLivePose(rtCoreStatus("rosie_1420_v1", [null, null]), caps, first, 5, "rosie_1420_v1"), null);
  // An unknown model is refused rather than guessed.
  const unknown = { ...rtCoreStatus("rosie_1420_v1", [0.1]), target: { ...target("rosie_1420_v1"), model_id: "rosie_9000" } } as unknown as DenseStatus;
  assert.equal(mapLivePose(unknown, caps, null, 6, "rosie_1420_v1"), null);
});

test("live pose on the legacy joint daemon takes only mapped, homed radians and needs feedback", () => {
  const axes = (homed: boolean) => Array.from({ length: 9 }, (_, index) => ({ index, qRad: index / 10, qCmdRad: null, posCounts: 0, homeCounts: null, statusword: 0, modeDisplay: null, operational: true, fault: false, error: false, homed }));
  const legacy = (mask: number | null, feedbackAgeMs: number | null, homed = true) => ({
    available: true, session: null, daemonError: null, cursor: null, armed: true, axes: axes(homed),
    daemon: { executorState: "idle", mappedAxisMask: mask, feedbackAgeMs, runtimeMode: "real" },
  }) as unknown as DenseStatus;
  // No target in the status: the open project's model names the joints.
  const pose = mapLivePose(legacy(0b111111, 4), null, null, 7, "rosie_1420_v1")!;
  assert.equal(pose.modelId, "rosie_1420_v1");
  assert.deepEqual(pose.jointsRad, [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0, 0, 0]);
  assert.equal(mapLivePose(legacy(0b111111, null), null, null, 8, "rosie_1420_v1"), null);
  assert.equal(mapLivePose(legacy(0b111111, 4, false), null, null, 9, "rosie_1420_v1"), null);
});

test("Home31 labels J6 unavailable instead of claiming the viewer's retained or default angle is live", () => {
  const capabilities = rtCoreCaps(["J1", "J2", "J3", "J4", "J5", "J6"]);
  const status = rtCoreStatus("rosie_1420_v1", [0.1, 0, 0, 0, 0, null]);
  Object.assign(status.rtCore!.axes[5]!, { homeValid: false, coordinateValid: false, calibrationValid: false, positionState: "unverified" });
  const previous = mapLivePose(rtCoreStatus("rosie_1420_v1", [0, 0, 0, 0, 0, 1]), capabilities, null, 1, "rosie_1420_v1")!;
  const input = { connected: true, status, capabilities, statusAgeMs: 20, nowMs: 100, previous, cellLabel: "rosie_1420_v1", fallbackModelId: "rosie_1420_v1" };
  const held = deriveLivePose(input);
  assert.equal(held.pose!.jointsRad[5], 1, "the renderer may retain geometry without claiming it was measured");
  assert.match(held.caption!, /^LIVE \(PARTIAL\)/);
  assert.match(held.jointCaption!, /J1 5.7°/);
  assert.match(held.jointCaption!, /J6 unavailable \(Home required\)/);
  assert.doesNotMatch(held.jointCaption!, /J6 (0\.0|57\.3)°/);
  assert.equal(deriveLivePose({ ...input, previous: null }).pose!.jointsRad[5], 0);
  assert.match(deriveLivePose({ ...input, previous: null }).jointCaption!, /J6 unavailable/);
  assert.equal(deriveLivePose({ ...input, statusAgeMs: STATUS_FRESHNESS_MS + 1 }).jointCaption, undefined, "stale samples are not fresh readouts");
  // Even a numeric fallback must not override explicit invalidity.
  status.rtCore!.axes[5]!.position = 0;
  assert.equal(deriveLivePose(input).pose!.jointsRad[5], 1);
  Object.assign(status.rtCore!.axes[5]!, { homeValid: true, coordinateValid: true, calibrationValid: true, positionState: "trusted" });
  assert.match(deriveLivePose(input).jointCaption!, /J6 0.0°/);
  assert.doesNotMatch(deriveLivePose(input).caption!, /PARTIAL/);
});

test("live view is LIVE with the status age while fresh, STALE past the Machine panel's bound, none without a machine", () => {
  const status = rtCoreStatus("rosie_1420_v1", [0.0498, 0.1937, 0, 0, 0, 0]);
  const caps = rtCoreCaps(["J1", "J2", "J3", "J4", "J5", "J6"]);
  const base = { connected: true, status, capabilities: caps, nowMs: 10_000, previous: null, cellLabel: "Rosie 1420 bench", fallbackModelId: "rosie_1400_v3" };

  const fresh = deriveLivePose({ ...base, statusAgeMs: 180 });
  assert.equal(fresh.mode, "live");
  assert.deepEqual(fresh.pose?.jointsRad.slice(0, 2), [0.0498, 0.1937]);
  assert.equal(fresh.caption, "LIVE · Rosie 1420 bench · rosie_1420_v1 · status 180 ms");

  const edge = deriveLivePose({ ...base, statusAgeMs: STATUS_FRESHNESS_MS });
  assert.equal(edge.mode, "live");

  // Past the bound the pose stops updating and the caption says so.
  const stale = deriveLivePose({ ...base, statusAgeMs: STATUS_FRESHNESS_MS + 417 });
  assert.equal(stale.mode, "stale");
  assert.equal(stale.pose, null);
  assert.equal(stale.caption, "LIVE (STALE) · Rosie 1420 bench · rosie_1420_v1 · status 2417 ms");

  // A retained status without a servo observation (the poll failed) is stale too.
  const unobserved = deriveLivePose({ ...base, statusAgeMs: 100, status: { ...status, rtCore: null } });
  assert.equal(unobserved.mode, "stale");
  assert.match(unobserved.caption!, /^LIVE \(STALE\) · Rosie 1420 bench · rosie_1420_v1/);

  // Before the first answer there is no age to vouch for anything.
  assert.equal(deriveLivePose({ ...base, statusAgeMs: null }).mode, "stale");
  assert.equal(deriveLivePose({ ...base, statusAgeMs: null }).caption, "LIVE (STALE) · Rosie 1420 bench · rosie_1420_v1 · no status yet");

  // Fresh but positionless: LIVE, nothing to draw, and the caption says why.
  const empty = deriveLivePose({ ...base, statusAgeMs: 50, capabilities: null });
  assert.equal(empty.mode, "live"); assert.equal(empty.pose, null);
  assert.equal(empty.caption, "LIVE · Rosie 1420 bench · rosie_1420_v1 · no axis positions · status 50 ms");

  // No machine: the preview owns the viewport and its caption.
  assert.deepEqual(deriveLivePose({ ...base, connected: false, statusAgeMs: 10 }), { mode: "none", pose: null, caption: null });
  assert.deepEqual(deriveLivePose({ ...base, status: null, statusAgeMs: 10 }), { mode: "none", pose: null, caption: null });
});
