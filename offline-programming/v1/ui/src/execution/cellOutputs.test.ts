import assert from "node:assert/strict";
import test from "node:test";
import { cellOutputsView, normalizeCellOutputs } from "./cellOutputs.ts";
import { normalizeDenseStatus } from "./denseClient.ts";
import { denseLoadIdentity } from "../rails/planRequest.ts";
import type { DenseTrajectoryRecord } from "../domain/project.ts";

// The cell-I/O simulation's policy (rt-core config/cell-io/simulation.json)
// as Describe publishes it, and one cyclic status.core.io.
const description = {
  io: {
    torch_qualified: false,
    outputs: [
      { name: "gas", class: "process", expiry_ns: 200_000_000 },
      { name: "torch", class: "torch", expiry_ns: 200_000_000 },
      { name: "weld", class: "process", expiry_ns: 200_000_000, readback: "none_unverified" },
    ],
  },
};
const output = (name: string, fields: Record<string, unknown> = {}) => ({
  name, intent: false, commanded: false, readback: false, valid: true, expiry_ns: 0, observed_ns: 0,
  changed_ns: 0, marker_sample_ns: 0, marker_cycle: 0, bit: 0, torch: name === "torch", ...fields,
});
const body = (io: Record<string, unknown>, session: Record<string, unknown> | null = null) => ({
  backend: "rt_core",
  rt_core: { description, status: { core: { io } } },
  session,
});

test("gas ON mid-program: commanded, read back, the time left on its expiry, and what the program switches", () => {
  const io = {
    time_ns: 5_000_000_000, cycle: 5000, armed: true, reason: 0, inputs: [],
    outputs: [
      output("gas", { intent: true, commanded: true, readback: true, expiry_ns: 5_120_000_000 }),
      output("torch"),
      output("weld", { valid: false }),
    ],
  };
  const view = cellOutputsView(normalizeCellOutputs(body(io, { kind: "trajectory", dry_run: false, process_outputs: ["gas"] })));
  assert.equal(view.visible, true);
  assert.equal(view.tone, "on");
  assert.equal(view.headline, "Outputs armed · this program switches gas");
  assert.deepEqual(view.rows.map((row) => row.text), [
    "gas: ON · readback ON · expires in 120 ms unless refreshed",
    "torch (torch): off · readback off",
    "weld: off · no wired readback",
  ]);
});

test("a readback that disagrees with the command is a fault row; an invalid one is unknown, never OFF", () => {
  const io = {
    time_ns: 1, cycle: 1, armed: false, reason: 15, inputs: [],
    outputs: [output("gas", { commanded: false, readback: true }), output("torch", { valid: false })],
  };
  const view = cellOutputsView(normalizeCellOutputs(body(io)));
  assert.equal(view.tone, "fault");
  assert.equal(view.headline, "Outputs disarmed: I/O readback disagreement");
  assert.equal(view.rows[0]!.tone, "fault");
  assert.equal(view.rows[0]!.text, "gas: off · readback ON");
  assert.equal(view.rows[1]!.text, "torch (torch): off · readback unknown");
});

test("the program's intent is shown apart from the command when they differ (expired or not yet armed)", () => {
  const io = { time_ns: 1, cycle: 1, armed: false, reason: 13, inputs: [], outputs: [output("gas", { intent: true })] };
  const view = cellOutputsView(normalizeCellOutputs(body(io)));
  assert.equal(view.rows[0]!.text, "gas: off · program asks ON · readback off");
  assert.equal(view.tone, "idle");
  assert.match(view.headline, /^Outputs disarmed: I\/O is not armed$/);
});

test("a dry run says its outputs stay off; a cell with no io block shows nothing", () => {
  const io = { time_ns: 1, cycle: 1, armed: false, reason: 0, inputs: [], outputs: [output("gas")] };
  const dry = cellOutputsView(normalizeCellOutputs(body(io, { kind: "trajectory", dry_run: true })));
  assert.equal(dry.headline, "Outputs disarmed · dry run: the program's outputs stay off");
  const none = cellOutputsView(normalizeCellOutputs({ backend: "rt_core", rt_core: { description: {}, status: { core: {} } } }));
  assert.equal(none.visible, false);
});

test("the dense status carries the outputs and the session's process outputs", () => {
  const status = normalizeDenseStatus({
    ...body({ time_ns: 1, cycle: 1, armed: true, reason: 0, inputs: [], outputs: [output("gas")] }),
    session: { id: "ds-1", kind: "trajectory", state: "loaded", process_outputs: ["gas"], lease: {} },
  });
  assert.equal(status.outputs?.armed, true);
  assert.deepEqual(status.outputs?.programOutputs, ["gas"]);
  assert.deepEqual(status.session?.processOutputs, ["gas"]);
});

test("Load and Run program send the plan record's output schedule digest only for a plan that has one", () => {
  const record = {
    schema: "robot.v4.dense-joint-trajectory.v1", trajectoryDigest: "sha256:" + "a".repeat(64), requestSha256: "", planId: "p",
    programId: "g", programDigest: "sha256:" + "b".repeat(64), manifestRevision: 7, planRevision: 2, createdAt: "", dtS: 0.01,
    totalSampleCount: 11, totalDurationS: 0.1, bytes: 1, segments: [], sourceProjectRevision: 1,
  } satisfies DenseTrajectoryRecord;
  assert.equal("process_markers_sha256" in denseLoadIdentity(record, false), false);
  const digest = "sha256:" + "c".repeat(64);
  assert.deepEqual(denseLoadIdentity({ ...record, processMarkersSha256: digest, processMarkerCount: 4 }, false), {
    dry_run: false, trajectory_digest: record.trajectoryDigest, plan_id: "p", program_id: "g", program_digest: record.programDigest,
    manifest_revision: 7, plan_revision: 2, process_markers_sha256: digest,
  });
});
