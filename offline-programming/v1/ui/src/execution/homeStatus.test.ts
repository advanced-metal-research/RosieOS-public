import assert from "node:assert/strict";
import test from "node:test";
import { deriveHomeStatus } from "./homeStatus.ts";
import { normalizeDenseCapabilities, normalizeDenseStatus } from "./denseClient.ts";
import { STATUS_FRESHNESS_MS } from "./machineFlow.ts";
import { CapabilitiesDigest, ContractVersion } from "../../../../../rt-core/clients/ts/rt_control_api_generated.ts";

function fixture() {
  const description = { contract_version: ContractVersion, capabilities_digest: CapabilitiesDigest,
    machine_sha256: "a".repeat(64), deployment_sha256: "b".repeat(64), capabilities: [],
    axes: ["J1", "J2"].map((id, index) => ({ id, index, position_unit: "rad", min_position: -3, max_position: 3 })) };
  const capabilities = normalizeDenseCapabilities({ backend: "rt_core", available: true, rt_core: { description } });
  const status = normalizeDenseStatus({ backend: "rt_core", available: true, rt_core: { description, status: {
    time_ns: 1000, adapter_incarnation: "a", daemon_incarnation: "d", execution: { state: "idle" },
    core: { armed: 0, native_home_active_axis_mask: 0 },
    axes: [{ home_valid: true, logical_valid: true }, { home_valid: false, logical_valid: false }],
  } } });
  return { status, capabilities, connected: true, statusAgeMs: 0 };
}

test("Home labels come from each reported joint, with native invalid reasons", () => {
  const input = fixture();
  input.status.rtCore!.operatorFeedback.axes[1]!.homeLoss = "Encoder reference changed";
  const home = deriveHomeStatus(input);
  assert.equal(home.state, "required");
  assert.deepEqual(home.axes.map(axis => axis.label), ["Homed", "Home required"]);
  assert.equal(home.axes[1]!.detail, "Encoder reference changed");
});

test("pending or native Home stays Homing until fresh confirmation", () => {
  const input = fixture();
  assert.equal(deriveHomeStatus({ ...input, pendingAxes: [1] }).axes[1]!.state, "homing");
  input.status.rtCore!.axes[1]!.homeActive = true;
  assert.equal(deriveHomeStatus(input).state, "homing");
  assert.equal(deriveHomeStatus({ ...input, statusAgeMs: STATUS_FRESHNESS_MS + 1 }).state, "stale");
});

test("stale, missing and disconnected observations never confirm retained Home", () => {
  const input = fixture();
  assert.ok(deriveHomeStatus({ ...input, statusAgeMs: STATUS_FRESHNESS_MS + 1 }).axes.every(axis => axis.state === "stale"));
  input.status.rtCore!.axes.pop();
  assert.equal(deriveHomeStatus(input).axes[1]!.state, "unavailable");
  assert.equal(deriveHomeStatus({ ...input, status: null }).state, "unavailable");
  assert.ok(deriveHomeStatus({ ...input, connected: false }).axes.every(axis => axis.state === "unavailable"));
});

test("sticky historical slow note does not override fresh status; position validity is separate", () => {
  const input = fixture();
  input.status.rtCoreFlags!.note = "rt_core_status_slow";
  input.status.rtCore!.axes[1]!.homeValid = true;
  const home = deriveHomeStatus(input);
  assert.equal(home.state, "homed");
  assert.match(home.axes[1]!.detail, /live joint position is unavailable/);
});


test("calibration and position recovery remain distinct; stale observations cannot confirm either", () => {
  const input = fixture();
  const axes = input.status.rtCore!.axes;
  for (const axis of axes) { axis.calibrationValid = true; axis.positionState = "trusted"; axis.homeValid = true; axis.coordinateValid = true; }
  assert.equal(deriveHomeStatus(input).label, "Calibrated · position verified");
  axes[1]!.homeValid = false; axes[1]!.coordinateValid = false; axes[1]!.positionState = "recovering";
  assert.equal(deriveHomeStatus(input).state, "recovering");
  axes[1]!.positionState = "lost";
  assert.equal(deriveHomeStatus(input).state, "lost");
  axes[1]!.calibrationValid = false;
  assert.equal(deriveHomeStatus(input).state, "calibration");
  assert.equal(deriveHomeStatus({ ...input, statusAgeMs: STATUS_FRESHNESS_MS + 1 }).state, "stale");
});
