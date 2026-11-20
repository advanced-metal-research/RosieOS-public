import assert from "node:assert/strict";
import test from "node:test";
import type { PlanCursor, ProcessStatus } from "../../../../../rt-core/clients/ts/rt_control_api_generated.ts";
import { activeRTCoreCursor, mapRTCoreStatus } from "./rtCoreStatus.ts";

test("latched reference fault distinguishes unqualified evidence from recovered Home without accepting Reset", () => {
  const status = {
    time_ns: 1, adapter_incarnation: "a", daemon_incarnation: "d", execution: { state: "cancelled" },
    core: { armed: 0, safety_fault_mask: 4, home_valid_mask: 0,
      recovery: { faults: [{ name: "coordinate_reference", bit: 6, axis_mask: 4, recovery: "rehome_required", outcome: "persists" }] } },
    axes: Array.from({ length: 6 }, () => ({ home_valid: false, logical_valid: false, readiness: "home_required" })),
  } as unknown as ProcessStatus;
  const mapped = mapRTCoreStatus(status, [], "native_rejected");
  assert.match(mapped.operatorFeedback.note!, /J3 position-reference evidence is not yet qualified/);
  assert.match(mapped.operatorFeedback.note!, /Keep the drives disarmed and inspect recovery status/);
  assert.match(mapped.operatorFeedback.note!, /do not establish a new Home just to clear this fault/);
  assert.equal(mapped.ready, false);
  assert.equal(mapped.operatorFeedback.reset.admissible, false);
  assert.equal(mapped.reason, "native_rejected");
  // Native may subsequently revalidate the existing saved reference. This
  // changes the guidance, not the latched fault or control authority.
  status.core.home_valid_mask = 63;
  status.axes.forEach(axis => { axis.home_valid = true; axis.logical_valid = true; axis.readiness = "ready"; });
  Object.assign(status.core.recovery!.faults[0]!, { reset_preserves_home:true, rehome_axis_mask:0 });
  const recovered = mapRTCoreStatus(status, [], "native_rejected");
  assert.match(recovered.operatorFeedback.note!, /J3: saved Home has recovered.*Clear fault while disarmed.*No new Home is needed/);
  assert.equal(recovered.ready, false, "the fault remains latched until an explicit native Reset succeeds");
  assert.equal(recovered.operatorFeedback.reset.admissible, false, "guidance grants no control authority");
  assert.equal(recovered.reason, "native_rejected");
});

test("only an active native handle with a real sample produces a cursor", () => {
  const cursor: PlanCursor = { active_handle: 3, sample_index: 0, time_ns: 1, native_clock_ns: 981234000000 };
  assert.equal(activeRTCoreCursor(cursor), cursor, "sample zero is valid while a handle is active");
  for (const active_handle of [0, -1, NaN, Infinity]) {
    assert.equal(activeRTCoreCursor({ ...cursor, active_handle }), null);
  }
  for (const sample_index of [0xffffffff, -1, NaN, Infinity, 2.5]) {
    assert.equal(activeRTCoreCursor({ ...cursor, sample_index }), null);
  }
  assert.equal(activeRTCoreCursor(null), null);
  assert.equal(activeRTCoreCursor(undefined), null);
});

test("start mismatch explanation retains rejection, Home and Reset gates", () => {
  const status = {
    time_ns: 1, adapter_incarnation: "a", daemon_incarnation: "d",
    execution: { state: "faulted", error: "native_rejected", segment: 0 },
    core: { armed: 0, safety_fault_mask: 1, execution_fault_reasons: 1, commissioning_phase: 0, native_home_active_axis_mask: 0,
      recovery: { reason: "none", faults: [{ name: "start_discontinuity", axis_mask: 1, recovery: "reset_clears" }] } },
    motion: { time_ns: 1, done: true, mode: 0 }, grant: { GrantActive: 1 },
    axes: [{ home_valid: true, logical_valid: true, readiness: "ready", logical_position: 0 }],
    plan_cursor: { active_handle: 0, sample_index: 0xffffffff, native_clock_ns: 981234000000 },
  } as unknown as ProcessStatus;
  const mapped = mapRTCoreStatus(status, [], "native_rejected");
  assert.equal(mapped.reason, "native_rejected", "display must not replace the raw gate reason");
  assert.equal(mapped.ready, false);
  assert.equal(mapped.armed, false);
  assert.equal(mapped.cursor, null);
  assert.match(mapped.executionDiagnostic!, /not at this plan’s starting position/);
  assert.match(mapped.operatorFeedback.note!, /Home does not move the robot/);
  assert.match(mapped.operatorFeedback.note!, /Go to plan start, Load to robot and Play/);
  assert.equal(mapped.operatorFeedback.reset.admissible, true);
  assert.equal(mapped.operatorFeedback.axes[0]!.homeLoss, "");
  status.grant!.GrantActive = 0;
  assert.equal(mapRTCoreStatus(status).operatorFeedback.reset.admissible, false);
  status.core.execution_fault_reasons = 0;
  status.core.recovery!.faults = [];
  assert.equal(mapRTCoreStatus(status).executionDiagnostic, null, "generic rejection alone must not invent a pose mismatch");
});
