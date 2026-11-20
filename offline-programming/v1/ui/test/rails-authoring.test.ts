// Rails authoring without a window (src/rails/authoring.ts): what the browser's
// rails mode and the RosieOS MCP server both run, so a weld accepted in either
// is the same node.
import assert from "node:assert/strict";
import test from "node:test";

import { decideWeld, measureRunPiece, optimisedPayload, posedSeam, runIndexAt, searchSeam, shippedSearchKnobs, type SeamMeasurer } from "../src/rails/authoring.ts";
import { SHIPPED_SEARCH_PRESETS } from "../src/rails/parameters.ts";
import type { RailsOptimisePlan } from "../src/rails/client.ts";

const seam = {
  id: "seam_0001", label: "fillet", weld_joint_ref: "weld_joint_0001", span: { start_s: 0, end_s: 1 }, direction: "forward",
  reference: { mode: "rail_curve" }, process: {}, orientation: { work_angle_deg: { kind: "constant", value: 45 }, spin: { mode: "free" } },
  curve: { from_weld_joint: true },
};
const joint = { id: "weld_joint_0001", weld_line: { segments: [], closed: false, length_m: 0.2 } };
const constant = (value: number) => ({ kind: "constant", value });
const plan = (clear: number, direction: "forward" | "reverse", turned: boolean): RailsOptimisePlan => ({
  work_angle_deg: constant(40), travel_angle_deg: constant(12), direction, direction_changed: turned,
  standoff_mm: 19, standoff_departure_mm: 2, clear_fraction: clear, authored_clear_fraction: 0.5,
  seam_length_mm: 200, knot_fractions: [0, 1], work_path_deg: [40, 40], travel_path_deg: [12, 12],
});

function measurer(clearRuns: number[][], blocked: number[][] = []) {
  const calls: { op: string; body: Record<string, unknown> }[] = [];
  const m: SeamMeasurer = {
    async planTorchPath(_step, body) {
      calls.push({ op: "plan", body });
      return body.direction === "reverse" ? { direction: "reverse", plan: plan(0.9, "reverse", true) } : { direction: "forward", plan: plan(0.6, "forward", false) };
    },
    async checkTorchFits(_step, body) {
      calls.push({ op: "check", body });
      return { clearance: { seam_length_m: 0.2, clear_runs: clearRuns, blocked_spans: blocked } };
    },
  };
  return { m, calls };
}

test("the search asks both directions, keeps the better coverage and checks clearance at the chosen angles", async () => {
  const { m, calls } = measurer([[0, 1]]);
  const searched = await searchSeam(m, new Uint8Array([1]), seam, joint, { torch: { id: "gun" }, search: shippedSearchKnobs(), defaultStandoffMm: 17 });
  assert.ok(searched);
  assert.deepEqual(calls.filter((c) => c.op === "plan").map((c) => c.body.direction).sort(), ["forward", "reverse"]);
  assert.equal(searched.plan.direction, "reverse");
  assert.deepEqual(searched.payload.overrides, optimisedPayload(seam, plan(0.9, "reverse", true)).overrides);
  const check = calls.find((c) => c.op === "check")!;
  assert.deepEqual((check.body.seam as { orientation: unknown }).orientation, { work_angle_deg: constant(40), travel_angle_deg: constant(12), spin: { mode: "free" } });
  assert.equal((check.body.seam as { direction: string }).direction, "reverse");
  assert.deepEqual(check.body.options, { standoff_mm: 19 });
  assert.equal(searched.report.state, "clear");
});

test("a checkpoint answering false drops the search before the clearance check", async () => {
  const { m, calls } = measurer([[0, 1]]);
  assert.equal(await searchSeam(m, new Uint8Array([1]), seam, joint, { torch: {}, search: shippedSearchKnobs(), defaultStandoffMm: 17 }, () => false), null);
  assert.equal(calls.some((c) => c.op === "check"), false);
});

test("a blocked seam narrows to one run with the profiles restricted onto it", async () => {
  const { m } = measurer([[0, 0.4], [0.6, 1]], [[0.4, 0.6]]);
  const searched = (await searchSeam(m, new Uint8Array([1]), seam, joint, { torch: {}, search: shippedSearchKnobs(), defaultStandoffMm: 17 }))!;
  assert.equal(searched.report.state, "partial");
  assert.equal(runIndexAt(searched.report, 0.8), 1);
  const piece = (await measureRunPiece(m, new Uint8Array([1]), seam, joint, searched, 1, {}))!;
  assert.equal(piece.id, "rails-seam_0001-run2");
  assert.deepEqual(piece.span, searched.report.split[1]);
  assert.equal(piece.payload.span?.start_s, piece.span.start_s);
  assert.match(piece.note, /^run 2\/2 · \d+ mm · \d+% clear$/);
});

test("accept writes the weld node; reaccept keeps what the operator authored on it", () => {
  const entry = { id: "rails-seam_0001", seam, joint, span: { start_s: 0, end_s: 1 }, plan: plan(0.9, "reverse", true), summary: null, payload: optimisedPayload(seam, plan(0.9, "reverse", true)) };
  const accepted = decideWeld(entry, "accept", "a".repeat(64))!;
  assert.equal(accepted.state, "accepted");
  assert.equal(accepted.note, "accepted · 90% weldable");
  assert.equal(accepted.node.op, "weld");
  assert.equal(accepted.node.geometry.kind, "seam");
  assert.deepEqual(accepted.node.geometry.span, { start_s: 0, end_s: 1 });
  assert.equal((accepted.node.extensions?.rails as { sha256: string }).sha256, "a".repeat(64));
  const authored = { ...accepted.node, enabled: false, standoff_mm: constant(25) } as typeof accepted.node;
  const again = decideWeld(entry, "accept", "a".repeat(64), authored)!;
  assert.equal(again.node.enabled, false);
  assert.deepEqual(again.node.standoff_mm, constant(25));
  assert.equal(decideWeld(entry, "trim", "a".repeat(64)), null, "nothing to trim to without a coverage summary");
  assert.deepEqual(posedSeam(seam, accepted.payload!).span, { start_s: 0, end_s: 1 });
});

test("the shipped search knobs are the first shipped preset's values", () => {
  const values = SHIPPED_SEARCH_PRESETS[0]!.values;
  assert.deepEqual(shippedSearchKnobs(), {
    work_deviation_deg: values.work_deviation_deg, travel_deviation_deg: values.travel_deviation_deg, standoff_cost_deg_per_mm: values.standoff_cost_deg_per_mm,
  });
});
