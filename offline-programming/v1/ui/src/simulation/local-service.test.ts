import assert from "node:assert/strict";
import test from "node:test";

import {
  createProject,
  zeroJoints,
  type AuthoredProgramRow,
  type OfflineProgrammingProject,
} from "../domain/project.ts";
import type { NormalizedPlanRequest } from "../execution/contracts.ts";
import {
  LocalPlannerAdapter,
  LocalSimulationAdapter,
} from "./local-adapter.ts";
import {
  LocalClientError,
  LocalPlanRejectedError,
  LocalSimulatorClient,
} from "./local-client.ts";
import {
  assertLocalPlanResult,
  localSafetyReasons,
  localSimulatorReady,
  parseLocalPlanResult,
  parseLocalSimulatorStatus,
} from "./local-contracts.ts";
import {
  buildLocalPlanRequest,
  localPlanRequestWire,
} from "./local-plan.ts";

const NOW = Date.UTC(2026, 6, 30, 16, 0, 0);

function statusWire(overrides: Record<string, unknown> = {}): Record<string, unknown> {
  const safety = {
    schema: "offline-programming.local-simulator-safety-proof.v1",
    profile_id: "local-simulator",
    runtime_boot_id: "pc-runtime-1",
    status_sequence: 7,
    observed_at: new Date(NOW - 100).toISOString(),
    fresh_until: new Date(NOW + 1_000).toISOString(),
    simulation: true,
    no_real_movement: true,
    loopback_only: true,
    ethercat_output_enabled: false,
    motion_output_enabled: false,
    hardware_access: false,
    physical_services_started: false,
    command_submission_enabled: false,
    ...(overrides.safety as Record<string, unknown> | undefined),
  };
  return {
    schema: "offline-programming.local-simulator-status.v1",
    service_id: "offline-programming-local",
    status_sequence: 8,
    phase: "ready",
    observed_at: new Date(NOW - 50).toISOString(),
    availability: {
      schema: "offline-programming.local-simulator-availability.v1",
      available: true,
      code: "available",
      reason: "",
      canonical_planner: true,
      process_bounded: true,
      persistent_process: true,
      explicit_opt_in: true,
      ...(overrides.availability as Record<string, unknown> | undefined),
    },
    topology: {
      platform: "pc_native",
      profile_id: "local-simulator",
      robot_model: "rosie_1400_v3",
      loopback_host: "127.0.0.1",
      plan_transport: "in_process",
      physical_service_start: false,
      command_transport: false,
      ...(overrides.topology as Record<string, unknown> | undefined),
    },
    readiness: {
      ready: true,
      reasons: [],
      ...(overrides.readiness as Record<string, unknown> | undefined),
    },
    safety,
    plan_in_flight: false,
    last_plan: {},
    last_error_code: "",
    last_error: "",
    ...Object.fromEntries(
      Object.entries(overrides).filter(([key]) => key !== "safety" &&
        key !== "topology" &&
        key !== "readiness" &&
        key !== "availability"),
    ),
  };
}

function row(id: string, x: number, weldOn = false): AuthoredProgramRow {
  return {
    id,
    enabled: true,
    label: id,
    command: "MOVE",
    moveType: "LINE",
    phase: weldOn ? "weld" : "travel",
    seamId: null,
    passId: "",
    passRole: "none",
    toolFrameId: "torch_tcp",
    workFrameId: "world",
    fixtureId: null,
    targetTcpPose: [x, 0, 0, 0, 0, 0],
    targetJointsRad: zeroJoints([x]),
    positionerTargetsRad: [0, 0, 0],
    tcpVelocityMps: 0.05,
    jointVelocityRadS: 0.25,
    accelerationMps2: 0.75,
    jointAccelerationRadS2: 1,
    blendRadiusM: 0,
    fineStop: true,
    weldEnabled: weldOn,
    arcState: weldOn ? "on" : "off",
    wireFeedMPerMin: 0,
    voltageV: 0,
    weldProfileId: 0,
    weaveProfileId: 0,
    weldProcess: "none",
    revPiIOSlot: null,
    revPiIOAction: "none",
    source: "offline_programming",
  };
}

function localProject(): OfflineProgrammingProject {
  const project = createProject({
    id: "project-1",
    now: new Date(NOW - 10_000).toISOString(),
  });
  project.identity.revision = 3;
  project.program.programRevision = 3;
  project.program.baseProgramRevision = 2;
  project.program.sourceProjectRevision = 3;
  project.target = {
    mode: "local_simulator",
    profileId: "local-simulator",
    manifestRef: null,
  };
  project.rows = [row("row-1", 0), row("row-2", 0.1, true)];
  return project;
}

function requestFixture(): NormalizedPlanRequest {
  return buildLocalPlanRequest(localProject(), {
    requestId: "request-local-1",
    submittedAt: new Date(NOW - 100).toISOString(),
  });
}

function planResultWire(
  request: NormalizedPlanRequest,
  overrides: Record<string, unknown> = {},
): Record<string, unknown> {
  const trajectory = [
    {
      row_index: 0,
      waypoint_index: 0,
      time_s: 0,
      move_type: "line_tcp",
      motion_phase: "travel",
      weld_on: false,
      tcp_pose: [0, 0, 0, 0, 0, 0],
      positions_rad: [0, 0, 0, 0, 0, 0, 0, 0, 0],
      applied_velocity_scale: 1,
    },
    {
      row_index: 1,
      waypoint_index: 1,
      time_s: 1,
      move_type: "line_tcp",
      motion_phase: "weld",
      weld_on: true,
      tcp_pose: [0.1, 0, 0, 0, 0, 0],
      positions_rad: [0.1, 0, 0, 0, 0, 0, 0, 0, 0],
      applied_velocity_scale: 1,
    },
  ];
  const evidence = {
    schema: "offline-programming.normalized-plan-evidence.v1",
    request_id: request.requestId,
    identity: {
      project_id: request.projectId,
      project_revision: request.projectRevision,
      program_id: request.programId,
      program_revision: request.programRevision,
      plan_id: "offline-programming-local/request-local-1/1",
      plan_revision: 1,
      program_hash: "fnv1a64:1234",
      robot_model: request.robotModel,
      target: {
        profile_id: request.target.profileId,
        manifest_ref: request.target.manifestRef,
      },
    },
    generated_at: new Date(NOW).toISOString(),
    planner_owner: "motion-server",
    planner_backend: "motion_server_tesseract_trajectory_v1",
    state: "server_validated",
    accepted: true,
    validation_state: "valid",
    sample_count: 2,
    collision: {
      state: "clear",
      check_state: "tesseract_discrete_contact_checked",
      points: [],
    },
    singularity: {
      state: "clear",
      check_state: "jacobian_svd_checked",
      points: [],
    },
    limits: { state: "clear", check_state: "clear", points: [] },
    runtime: { state: "clear", playable: true },
    correlation_reasons: [],
    ...(overrides.evidence as Record<string, unknown> | undefined),
  };
  const acceptedPlan = {
    schema: "robot.v4.accepted-plan.v1",
    authority_classification: "simulation_only",
    plan_id: "offline-programming-local/request-local-1/1",
    plan_revision: 1,
    trajectory_digest: "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
    program_id: request.programId,
    program_revision: request.programRevision,
    program_digest: "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
    planner: {
      owner: "motion-server",
      backend: "motion_server_tesseract_trajectory_v1",
      trajectory_method: "linear-tcp-dense-csp",
      build_digest: "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
    },
    target: {
      target_id: "local-simulator",
      robot_model: request.robotModel,
      joint_order: ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"],
      manifest_revision: "local-simulator:test",
      cell_config_revision: "local-cell-test",
      calibration_digest: "sha256:dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd",
      urdf_digest: "sha256:eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee",
      mesh_digest: "sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff",
      collision_world_digest:
        "sha256:1111111111111111111111111111111111111111111111111111111111111111",
      settings_revision: 1,
    },
    evidence: {
      planning_contract_accepted: true,
      collision: "clear",
      singularity: "clear",
      limits: "clear",
      start_pose: "unavailable",
      runtime_playable: false,
      collision_points: [],
      singularity_points: [],
    },
    request_id: request.requestId,
    trajectory: trajectory.map((sample, sampleIndex) => ({
      sample_index: sampleIndex,
      row_id: request.rows[sample.row_index]!.rowId,
      row_index: sample.row_index,
      time_s: sample.time_s,
      tcp_pose_m: sample.tcp_pose,
      joint_positions_rad: sample.positions_rad,
      weld_on: sample.weld_on,
    })),
    created_at: new Date(NOW).toISOString(),
  };
  return {
    schema: "offline-programming.normalized-plan-result.v1",
    evidence,
    trajectory,
    report_reasons: [],
    accepted_plan: acceptedPlan,
    ...Object.fromEntries(
      Object.entries(overrides).filter(([key]) => key !== "evidence"),
    ),
  };
}

function jsonResponse(body: unknown, status = 200): Response {
  const encoded = JSON.stringify(body);
  return new Response(encoded, {
    status,
    headers: {
      "content-type": "application/json",
      "content-length": String(new TextEncoder().encode(encoded).byteLength),
    },
  });
}

test("local status requires fresh explicit proof for every disabled output", () => {
  const ready = parseLocalSimulatorStatus(statusWire());
  assert.equal(localSimulatorReady(ready, NOW), true);
  assert.deepEqual(localSafetyReasons(ready, NOW), []);

  const unsafeCases: Array<[string, Record<string, unknown>, string]> = [
    ["simulation false", { safety: { simulation: false } }, "simulation_not_proven"],
    [
      "real movement possible",
      { safety: { no_real_movement: false } },
      "no_real_movement_not_proven",
    ],
    [
      "EtherCAT enabled",
      { safety: { ethercat_output_enabled: true } },
      "ethercat_output_enabled",
    ],
    [
      "motion enabled",
      { safety: { motion_output_enabled: true } },
      "motion_output_enabled",
    ],
    [
      "physical service",
      { safety: { physical_services_started: true } },
      "physical_services_started",
    ],
    [
      "command transport",
      { topology: { command_transport: true } },
      "command_transport_enabled",
    ],
    [
      "external host",
      { topology: { loopback_host: "192.0.2.5" } },
      "host_not_loopback",
    ],
    [
      "stale",
      { safety: { fresh_until: new Date(NOW).toISOString() } },
      "safety_proof_stale",
    ],
    [
      "contradictory readiness",
      { readiness: { reasons: ["backend_warming"] } },
      "readiness_reasons_present",
    ],
    [
      "typed backend unavailable",
      {
        availability: {
          available: false,
          code: "collision_runtime_unavailable",
        },
      },
      "collision_runtime_unavailable",
    ],
    [
      "unbounded proof window",
      {
        safety: {
          observed_at: new Date(NOW - 100).toISOString(),
          fresh_until: new Date(NOW + 31_000).toISOString(),
        },
      },
      "safety_proof_window_unbounded",
    ],
  ];
  for (const [name, override, reason] of unsafeCases) {
    const status = parseLocalSimulatorStatus(statusWire(override));
    assert.equal(localSimulatorReady(status, NOW), false, name);
    assert.ok(localSafetyReasons(status, NOW).includes(reason), name);
  }

  const boundedClockSkew = parseLocalSimulatorStatus(statusWire({
    safety: {
      observed_at: new Date(NOW + 250).toISOString(),
      fresh_until: new Date(NOW + 1_000).toISOString(),
    },
  }));
  assert.equal(localSimulatorReady(boundedClockSkew, NOW), true);

  const excessiveClockSkew = parseLocalSimulatorStatus(statusWire({
    safety: {
      observed_at: new Date(NOW + 1_001).toISOString(),
      fresh_until: new Date(NOW + 2_000).toISOString(),
    },
  }));
  assert.equal(localSimulatorReady(excessiveClockSkew, NOW), false);
  assert.ok(localSafetyReasons(excessiveClockSkew, NOW).includes("safety_proof_stale"));
});

test("local project conversion is revision-bound and emits snake-case wire data", () => {
  const request = requestFixture();
  assert.equal(request.projectRevision, 3);
  assert.equal(request.programRevision, 3);
  assert.equal(request.baseProgramRevision, 2);
  assert.equal(request.target.profileId, "local-simulator");
  assert.equal(request.rows.length, 2);
  const wire = localPlanRequestWire(request) as Record<string, unknown>;
  assert.equal(wire.request_id, "request-local-1");
  assert.equal(wire.project_revision, 3);
  assert.equal("planned_path" in wire, false);

  const wrongTarget = localProject();
  wrongTarget.target = { mode: "offline_preview", profileId: null, manifestRef: null };
  assert.throws(
    () => buildLocalPlanRequest(wrongTarget, { requestId: "request-2" }),
    /local_simulator/,
  );
  const tooShort = localProject();
  tooShort.rows[1]!.enabled = false;
  assert.throws(
    () => buildLocalPlanRequest(tooShort, { requestId: "request-3" }),
    /two enabled/,
  );
});

test("local result parser enforces exact revision and canonical safety evidence", () => {
  const request = requestFixture();
  const result = parseLocalPlanResult(planResultWire(request), request);
  assert.equal(result.trajectory.length, 2);
  assert.equal(result.trajectory[1]?.jointPositionsRad[0], 0.1);
  assert.doesNotThrow(() => assertLocalPlanResult(result, request));

  const stale = planResultWire(request);
  const staleEvidence = stale.evidence as Record<string, unknown>;
  const staleIdentity = staleEvidence.identity as Record<string, unknown>;
  staleIdentity.project_revision = 2;
  assert.throws(
    () => parseLocalPlanResult(stale, request),
    /project_revision_mismatch/,
  );

  const missingRuntime = planResultWire(request, {
    evidence: { runtime: { state: "unavailable", playable: false } },
  });
  const blocked = parseLocalPlanResult(missingRuntime, request);
  assert.throws(
    () => assertLocalPlanResult(blocked, request),
    /runtime_not_playable/,
  );
});

test("local client refuses non-loopback endpoints and gates plan on status", async () => {
  assert.throws(
    () => new LocalSimulatorClient({ baseUrl: "http://192.0.2.20:8794" }),
    (error: unknown) => error instanceof LocalClientError &&
      error.code === "invalid_endpoint",
  );
  const request = requestFixture();
  let calls = 0;
  const fetchMock: typeof fetch = async (input, init) => {
    calls += 1;
    const url = String(input);
    if (url.endsWith("/status")) {
      return jsonResponse({ status: statusWire() });
    }
    assert.ok(url.endsWith("/plans"));
    assert.equal(init?.method, "POST");
    const wire = JSON.parse(String(init?.body)) as Record<string, unknown>;
    assert.equal(wire.request_id, request.requestId);
    return jsonResponse({ result: planResultWire(request) });
  };
  const client = new LocalSimulatorClient({
    baseUrl: "http://127.0.0.1:8794",
    fetch: fetchMock,
    now: () => NOW,
  });
  assert.equal(client.timeoutMs, 10_000);
  assert.equal(client.operationTimeoutMs, 270_000);
  const result = await client.plan(request);
  assert.equal(result.evidence.state, "server_validated");
  assert.equal(calls, 3);

  const unsafeClient = new LocalSimulatorClient({
    fetch: async () => jsonResponse({
      status: statusWire({
        safety: { motion_output_enabled: true },
      }),
    }),
    now: () => NOW,
  });
  await assert.rejects(
    unsafeClient.plan(request),
    (error: unknown) => error instanceof LocalClientError &&
      error.code === "not_ready",
  );
});

test("local client fails closed on HTTP, oversized, and malformed evidence", async () => {
  const httpClient = new LocalSimulatorClient({
    fetch: async () => jsonResponse({ error: "planner unavailable" }, 503),
    now: () => NOW,
  });
  await assert.rejects(
    httpClient.status(),
    (error: unknown) => error instanceof LocalClientError &&
      error.code === "http" &&
      error.status === 503,
  );

  const htmlClient = new LocalSimulatorClient({
    fetch: async () => new Response("Bad Gateway", {
      status: 502,
      headers: { "content-type": "text/plain" },
    }),
    now: () => NOW,
  });
  await assert.rejects(
    htmlClient.status(),
    (error: unknown) => error instanceof LocalClientError &&
      error.code === "http" &&
      error.status === 502 &&
      error.message === "local simulator HTTP 502",
  );

  const oversized = new LocalSimulatorClient({
    maxResponseBytes: 1_024,
    fetch: async () => new Response("{}", {
      status: 200,
      headers: { "content-length": "2048" },
    }),
  });
  await assert.rejects(
    oversized.status(),
    (error: unknown) => error instanceof LocalClientError &&
      error.code === "response_too_large",
  );

  const request = requestFixture();
  const fetchMock: typeof fetch = async (input) => {
    if (String(input).endsWith("/status")) {
      return jsonResponse({ status: statusWire() });
    }
    return jsonResponse({
      result: planResultWire(request, {
        evidence: { planner_backend: "browser-preview" },
      }),
    });
  };
  const invalid = new LocalSimulatorClient({ fetch: fetchMock, now: () => NOW });
  await assert.rejects(
    invalid.plan(request),
    (error: unknown) => error instanceof LocalClientError &&
      error.code === "invalid_response",
  );

  const collisionFetch: typeof fetch = async (input) => {
    if (String(input).endsWith("/status")) {
      return jsonResponse({ status: statusWire() });
    }
    return jsonResponse({
      result: planResultWire(request, {
        evidence: {
          state: "rejected",
          accepted: false,
          validation_state: "invalid",
          collision: {
            state: "blocked",
            check_state: "tesseract_discrete_contact_checked",
            points: [{
              row_index: 1,
              waypoint_index: 1,
              link_a: "link_1",
              link_b: "fixture",
              depth_m: 0.01,
              affected_joints: ["J1"],
              positions_rad: [0, 0, 0, 0, 0, 0, 0, 0, 0],
            }],
          },
          runtime: { state: "blocked", playable: false },
        },
      }),
    });
  };
  const rejected = new LocalSimulatorClient({
    fetch: collisionFetch,
    now: () => NOW,
  });
  await assert.rejects(
    rejected.plan(request),
    (error: unknown) => error instanceof LocalPlanRejectedError &&
      error.code === "plan_rejected" &&
      error.result.evidence.collision.points.length === 1,
  );

  const mismatchedWire = planResultWire(request);
  const mismatchedAccepted = structuredClone(mismatchedWire.accepted_plan) as {
    trajectory: Array<{ joint_positions_rad: number[] }>;
  };
  mismatchedAccepted.trajectory[1]!.joint_positions_rad[0] = 1.234;
  mismatchedWire.accepted_plan = mismatchedAccepted;
  const mismatched = new LocalSimulatorClient({
    now: () => NOW,
    fetch: async (input) => String(input).endsWith("/status")
      ? jsonResponse({ status: statusWire() })
      : jsonResponse({ result: mismatchedWire }),
  });
  await assert.rejects(
    mismatched.plan(request),
    (error: unknown) => error instanceof LocalPlanRejectedError &&
      error.reasons.includes("accepted_trajectory_mismatch"),
  );
});

test("local adapters load only validated data into browser playback and expose no commands", async () => {
  const project = localProject();
  let latestRequestId = "";
  const fetchMock: typeof fetch = async (input, init) => {
    const url = String(input);
    if (url.endsWith("/status")) {
      return jsonResponse({ status: statusWire() });
    }
    if (url.endsWith("/plans")) {
      const wire = JSON.parse(String(init?.body)) as Record<string, unknown>;
      const latestRequest: NormalizedPlanRequest = {
        ...requestFixture(),
        requestId: String(wire.request_id),
      };
      latestRequestId = latestRequest.requestId;
      return jsonResponse({ result: planResultWire(latestRequest) });
    }
    throw new Error(`unexpected route ${url}`);
  };
  const planner = new LocalPlannerAdapter({
    fetch: fetchMock,
    now: () => NOW,
  });
  assert.equal(planner.enabled, false);
  await planner.refresh();
  assert.equal(planner.enabled, true);
  const evidence = await planner.plan(project);
  assert.equal(evidence.state, "server_validated");
  assert.ok(latestRequestId.includes(":local:3:1"));

  let loaded: unknown = null;
  let stopped = 0;
  const simulator = new LocalSimulationAdapter(planner, {
    load: (trajectory) => {
      loaded = trajectory;
    },
    stop: () => {
      stopped += 1;
    },
  });
  await simulator.load(planner.lastResult!.trajectory);
  assert.notEqual(loaded, planner.lastResult!.trajectory);
  await simulator.stop();
  assert.equal(stopped, 1);
  for (const forbidden of ["arm", "jog", "play", "weld", "execute", "teleop"]) {
    assert.equal(forbidden in simulator, false);
    assert.equal(forbidden in planner, false);
  }
});
