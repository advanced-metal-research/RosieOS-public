import assert from "node:assert/strict";
import test from "node:test";
import { DenseExecutionClient, DenseExecutionError } from "./denseClient.ts";
import { isMachineFault, machineReasonText } from "./machineFlow.ts";
import { parseUploadLimitViolation, uploadLimitText } from "./uploadLimit.ts";
import { rtCoreReasonText } from "../targets/rtCoreStatus.ts";

const violation = {
  kind: "position" as const, segment: 0, sample: 2385, axis: "J6",
  value: -2.094446759543233, limit: -2.094395102, unit: "rad" as const,
};
const identity = { trajectory_digest: "sha256:a", plan_id: "p", program_id: "program", program_digest: "sha256:b", manifest_revision: 1, plan_revision: 1 };

test("a native upload position violation names the joint and tiny overshoot without suggesting a reset", async () => {
  const client = new DenseExecutionClient(async () => new Response(JSON.stringify({
    error: "native_limit_exceeded", detail: "segment 0 sample 2385 axis 5", limit_violation: violation,
  }), { status: 409, headers: { "Content-Type": "application/json" } }));
  await assert.rejects(client.load(identity), error => {
    assert.ok(error instanceof DenseExecutionError);
    assert.deepEqual(error.limitViolation, violation);
    assert.equal(error.detail, "segment 0 sample 2385 axis 5");
    assert.match(error.message, /J6 planned position -120\.00296°/);
    assert.match(error.message, /machine's -120° limit/);
    assert.match(error.message, /Replan for the connected machine/);
    assert.doesNotMatch(error.message, /Reset|fault/);
    assert.equal(isMachineFault(error.message), false);
    assert.match(machineReasonText(error.message), /J6 planned position -120\.00296°/);
    return true;
  });
});

test("velocity violations display angular speed magnitude and preserve diagnostic sample indices", () => {
  const parsed = parseUploadLimitViolation({ ...violation, kind: "velocity", unit: "rad/s", value: -Math.PI, limit: Math.PI / 2 })!;
  assert.equal(parsed.segment, 0);
  assert.equal(parsed.sample, 2385);
  assert.match(uploadLimitText(parsed), /J6 planned speed 180°\/s/);
  assert.match(uploadLimitText(parsed), /90°\/s limit/);
});

test("Hermite segment velocity refusals preserve measured speed through HTTP and operator wording", async () => {
  const rate = { ...violation, kind: "velocity", unit: "rad/s", value: Math.PI, limit: Math.PI / 2 };
  const client = new DenseExecutionClient(async () => Response.json({
    error: "native_segment_rate_exceeded", detail: "segment=0 sample=2385 axis=J6", limit_violation: rate,
  }, { status: 409 }));
  await assert.rejects(client.load(identity), error => {
    assert.ok(error instanceof DenseExecutionError);
    assert.equal(error.code, "native_segment_rate_exceeded");
    assert.equal(error.httpStatus, 409);
    assert.equal(error.detail, "segment=0 sample=2385 axis=J6");
    assert.deepEqual(error.limitViolation, rate);
    assert.match(error.message, /J6 planned speed 180°\/s/);
    assert.match(error.message, /90°\/s limit/);
    const detail = uploadLimitText(error.limitViolation!);
    assert.equal(rtCoreReasonText(error.code, detail), error.message);
    assert.equal(machineReasonText(error.message), error.message);
    assert.equal(isMachineFault(error.message), false);
    return true;
  });
});

test("malformed limit diagnostics are ignored rather than inventing a joint or bound", () => {
  for (const bad of [null, [], {}, { ...violation, value: NaN }, { ...violation, limit: Infinity },
    { ...violation, value: "-2" }, { ...violation, segment: -1 }, { ...violation, sample: 0.2 },
    { ...violation, unit: "mm" }, { ...violation, axis: "<script>" },
    { ...violation, kind: "velocity", unit: "rad" },
    { ...violation, kind: "velocity", unit: "rad/s", limit: -1 }]) {
    assert.equal(parseUploadLimitViolation(bad), null);
  }
});

test("old servers and unrelated refusals retain their original error text and code", async () => {
  for (const body of [
    { error: "native_limit_exceeded", detail: "native_limit_exceeded" },
    { error: "native_limit_exceeded", detail: "original", limit_violation: { ...violation, value: "bad" } },
    { error: "native_segment_rate_exceeded", detail: "original", limit_violation: { ...violation, value: "bad" } },
    { error: "native_segment_rate_exceeded_other", detail: "original", limit_violation: violation },
    { error: "no_grant", detail: "acquire control", limit_violation: violation },
  ]) {
    const client = new DenseExecutionClient(async () => new Response(JSON.stringify(body), { status: 409 }));
    await assert.rejects(client.load(identity), error => {
      assert.ok(error instanceof DenseExecutionError);
      assert.equal(error.message, `${body.error}: ${body.detail}`);
      assert.equal(error.code, body.error);
      return true;
    });
  }
});
