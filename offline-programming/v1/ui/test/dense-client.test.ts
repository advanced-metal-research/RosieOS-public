import assert from "node:assert/strict";
import test from "node:test";

import {
  DENSE_EXECUTION_PATH,
  DenseExecutionClient,
  DenseExecutionError,
  normalizeDenseStatus,
} from "../src/execution/denseClient.ts";

const jsonResponse = (status: number, body: unknown): Response =>
  new Response(JSON.stringify(body), { status, headers: { "Content-Type": "application/json" } });

test("a status with the daemon's full shape normalises into the UI's naming", () => {
  const status = normalizeDenseStatus({
    available: true,
    session: { id: "s-1", trajectory_digest: "sha256:ab", plan_id: "p:1", state: "playing",
      lease: { held: true, expires_in_ms: 1500 }, heartbeat_age_ms: 120, heartbeat_deadline_ms: 5000 },
    daemon: { staged: { plan_id: "p:1" }, executor: { state: "playing", segment_index: 2, sample_index: 40,
      t_s: 0.4, torch_on: true, feedback_age_ms: 3, authority_fresh: true,
      authority: { runtime_mode: "sim", jetson_id: "bench-relay", robot_model_id: "rosie_1420_v1" } }, last_error: null },
    cursor: { segment_index: 2, segment_count: 5, kind: "weld", source_id: "seam-1", sample_index: 40,
      global_sample_index: 1240, total_sample_count: 9820, t_s: 0.4, global_t_s: 12.4, total_duration_s: 98.2 },
    armed: true,
    axes: [{ index: 0, q_rad: 0.5, q_cmd_rad: 0.5001, pos_counts: 10, home_counts: 0, statusword: 0x27,
      mode_display: 8, operational: true, fault: false, error: false, homed: false }],
  });
  assert.equal(status.session?.state, "playing");
  assert.equal(status.session?.leaseHeld, true);
  assert.equal(status.daemon?.executorState, "playing");
  assert.equal(status.daemon?.staged, true);
  assert.equal(status.daemon?.runtimeMode, "sim");
  assert.equal(status.daemon?.jetsonId, "bench-relay");
  assert.equal(status.daemon?.robotModelId, "rosie_1420_v1");
  assert.equal(status.cursor?.globalSampleIndex, 1240);
  assert.equal(status.armed, true);
  assert.equal(status.axes?.[0]?.qCmdRad, 0.5001);
  assert.equal(status.axes?.[0]?.homed, false);
});

test("a daemon without an executor, and fields the daemon has not learned yet, read as absent", () => {
  const status = normalizeDenseStatus({ available: true, daemon: { executor: "absent" } });
  assert.equal(status.daemon?.executorState, "absent");
  assert.equal(status.daemon?.feedbackAgeMs, null);
  assert.equal(status.cursor, null);
  assert.equal(status.armed, null);
  assert.equal(status.axes, null);
  assert.equal(status.session, null);
});

test("requests carry the identity and the server's error code survives", async () => {
  const calls: { url: string; init: RequestInit }[] = [];
  const client = new DenseExecutionClient(async (input, init) => {
    calls.push({ url: String(input), init: init ?? {} });
    if (String(input).endsWith("/load")) {
      return jsonResponse(409, { error: "dense_identity_mismatch", detail: "header plan_id differs" });
    }
    return jsonResponse(200, { ok: true, stopped: true, attempts: 1, status: { available: true } });
  });
  await assert.rejects(
    client.load({ trajectory_digest: "sha256:ab", plan_id: "p:1", program_id: "p",
      program_digest: "sha256:cd", manifest_revision: 1, plan_revision: 3, dry_run: true }),
    (error: unknown) => error instanceof DenseExecutionError && error.code === "dense_identity_mismatch"
      && error.httpStatus === 409,
  );
  assert.equal(calls[0]!.url, `${DENSE_EXECUTION_PATH}/load`);
  assert.equal(JSON.parse(String(calls[0]!.init.body)).plan_revision, 3);
  assert.equal(calls[0]!.init.cache, "no-store");
  assert.equal(JSON.parse(String(calls[0]!.init.body)).dry_run, true);
  const stop = await client.stop();
  assert.equal(stop.stopped, true);
  assert.equal(JSON.parse(String(calls[1]!.init.body ?? "{}")) instanceof Object, true);
  await client.heartbeat("s-1");
  assert.equal(JSON.parse(String(calls[2]!.init.body)).session_id, "s-1");
  await client.goHome();
  assert.equal(calls[3]!.url, `${DENSE_EXECUTION_PATH}/go-home`);
});

test("an rt-core status carries which robot the machine says it is", () => {
  const described = normalizeDenseStatus({
    backend: "rt_core", available: true, session: null, rt_core: { status: null, events: [], reason: "" },
    robot_cell: { configured: true, valid: true, model_id: "rosie_1400_v3",
      robot_description_sha256: "sha256:" + "a".repeat(64), machine_planning_calibration_sha256: "sha256:" + "b".repeat(64), machine_planning_calibration_present: true, error: "" },
  });
  assert.deepEqual(described.robotCell, { configured: true, valid: true, modelId: "rosie_1400_v3",
    robotDescriptionSha256: "sha256:" + "a".repeat(64), machinePlanningCalibrationSha256: "sha256:" + "b".repeat(64), machinePlanningCalibrationPresent: true, machineConfigurationSha256: "", error: null });
  // A machine that binds no robot says so, and one not yet described says nothing.
  const flat = normalizeDenseStatus({ backend: "rt_core", available: true, session: null, rt_core: { status: null, events: [], reason: "" },
    robot_cell: { configured: false, valid: false, model_id: "", robot_description_sha256: "", machine_planning_calibration_sha256: "", machine_planning_calibration_present: false,
      error: "the machine binds no robot description" } });
  assert.equal(flat.robotCell?.configured, false);
  assert.equal(flat.robotCell?.error, "the machine binds no robot description");
  const silent = normalizeDenseStatus({ backend: "rt_core", available: false, session: null, rt_core: { status: null, events: [], reason: "" } });
  assert.equal(silent.robotCell, null);
});


test("session retains source identity, execution identity and explicit run mode", () => {
  const session = normalizeDenseStatus({ session: { id: "session", trajectory_digest: "sha256:source", execution_trajectory_digest: "sha256:motion-only", dry_run: true } }).session;
  assert.equal(session?.trajectoryDigest, "sha256:source");
  assert.equal(session?.executionTrajectoryDigest, "sha256:motion-only");
  assert.equal(session?.dryRun, true);
  assert.equal(normalizeDenseStatus({ session: { dry_run: false } }).session?.dryRun, false);
  assert.equal(normalizeDenseStatus({ session: {} }).session?.dryRun, null, "older sessions never imply outputs are off");
});
