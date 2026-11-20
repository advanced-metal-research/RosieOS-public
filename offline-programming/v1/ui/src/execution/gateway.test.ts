import assert from "node:assert/strict";
import test from "node:test";
import * as gatewayModule from "./gateway.ts";
import {
  GATEWAY_BROWSER_BOUNDARY,
  normalizeGatewayCapabilities,
  normalizeGatewayReview,
  unavailableGatewayCapabilities,
  type GatewayExecutionIdentity,
  type GatewayReviewExpectation,
} from "./gateway.ts";
import { ExecutionGatewayClient } from "./gatewayClient.ts";

const nowMs = Date.parse("2026-07-30T08:00:00.000Z");

test("gateway UI defaults unavailable and exposes no command sender", () => {
  const unavailable = unavailableGatewayCapabilities(nowMs);
  assert.equal(unavailable.infrastructureReady, false);
  assert.equal(unavailable.planPreload, false);
  assert.equal(unavailable.planPlay, false);
  assert.equal(unavailable.protectiveStop, false);
  assert.equal(unavailable.arm, false);
  assert.equal(unavailable.weldOn, false);
  assert.equal(unavailable.jog, false);
  assert.equal(unavailable.teleop, false);
  assert.equal(unavailable.rawBrowserTransport, false);
  assert.equal(GATEWAY_BROWSER_BOUNDARY.canMintReviewToken, false);
  assert.equal(GATEWAY_BROWSER_BOUNDARY.canAcquireLease, false);
  assert.equal(GATEWAY_BROWSER_BOUNDARY.canSubmitCommand, false);
  assert.equal(GATEWAY_BROWSER_BOUNDARY.canExecuteInBrowser, false);
  for (const unsafeExport of ["send", "submit", "execute", "connect", "transport"]) {
    assert.equal(
      Object.prototype.hasOwnProperty.call(gatewayModule, unsafeExport),
      false,
      `unexpected browser execution export ${unsafeExport}`,
    );
  }
});

test("gateway client reads same-origin capabilities without exposing a sender", async () => {
  const client = new ExecutionGatewayClient(async (input, init) => {
    assert.equal(input, "/api/offline-programming/v1/execution-gateway/capabilities");
    assert.equal(init?.method, "GET");
    return new Response(JSON.stringify({
      ...capabilityWire(),
      gateway_enabled: false,
      connected_execution_enabled: false,
      server_controller_authority: false,
      manifest_allowlist_injected: false,
      authoritative_evidence_injected: false,
      server_transport_injected: false,
      server_owned_transport: false,
      lease_authority_injected: false,
      fence_validation_injected: false,
      audit_sink_injected: false,
      idempotency_store_injected: false,
      review_tokens: false,
      plan_preload: false,
      plan_play: false,
      protective_stop: false,
      protective_disarm: false,
      protective_weld_off: false,
      infrastructure_ready: false,
      controller: { controller_id: "", controller_boot_id: "" },
      reasons: ["gateway_disabled"],
    }), {
      status: 200,
      headers: { "Content-Type": "application/json" },
    });
  });
  const capabilities = await client.capabilities();
  assert.equal(capabilities.infrastructureReady, false);
  assert.equal(capabilities.rawBrowserTransport, false);
  assert.deepEqual(capabilities.reasons, ["gateway_disabled"]);
});

test("exact server review can request a server start but never execute in browser", () => {
  const expected = expectation();
  const review = normalizeGatewayReview(reviewWire(), expected, nowMs);
  assert.equal(review.ready, true);
  assert.equal(review.canRequestServerStart, true);
  assert.equal(review.canExecuteInBrowser, false);
  assert.ok(review.token);
  assert.deepEqual(review.identity, expected.identity);
  assert.equal(review.capabilities.infrastructureReady, true);
  assert.equal(review.capabilities.rawBrowserTransport, false);
});

test("review token rejects project, plan, target revision, sequence, and expiry tampering", () => {
  const cases: Array<{
    name: string;
    mutate: (wire: Record<string, unknown>) => void;
    reason: string;
  }> = [
    {
      name: "project revision",
      mutate: (wire) => {
        tokenIdentity(wire).project_revision = 12;
      },
      reason: "review_token_binding_invalid",
    },
    {
      name: "plan revision",
      mutate: (wire) => {
        tokenIdentity(wire).plan_revision = 43;
      },
      reason: "review_token_binding_invalid",
    },
    {
      name: "manifest revision",
      mutate: (wire) => {
        target(tokenIdentity(wire)).manifest_revision = `sha256:${"b".repeat(64)}`;
      },
      reason: "review_token_binding_invalid",
    },
    {
      name: "status sequence",
      mutate: (wire) => {
        token(wire).status_sequence = 9;
      },
      reason: "review_token_binding_invalid",
    },
    {
      name: "expired",
      mutate: (wire) => {
        token(wire).expires_at = "2026-07-30T08:00:00.000Z";
      },
      reason: "review_token_binding_invalid",
    },
  ];

  for (const entry of cases) {
    const wire = reviewWire();
    entry.mutate(wire);
    const review = normalizeGatewayReview(wire, expectation(), nowMs);
    assert.equal(review.ready, false, entry.name);
    assert.equal(review.canRequestServerStart, false, entry.name);
    assert.equal(review.token, null, entry.name);
    assert.ok(review.reasons.includes(entry.reason), entry.name);
  }
});

test("unsafe or internally inconsistent capability claims fail closed", () => {
  const unsafe = capabilityWire();
  unsafe.raw_browser_transport = true;
  unsafe.arm = true;
  const normalizedUnsafe = normalizeGatewayCapabilities(unsafe);
  assert.equal(normalizedUnsafe.rawBrowserTransport, false);
  assert.equal(normalizedUnsafe.arm, false);
  assert.equal(normalizedUnsafe.infrastructureReady, false);
  assert.ok(
    normalizedUnsafe.reasons.includes(
      "unsafe_browser_or_energizing_capability_claim",
    ),
  );

  const inconsistent = capabilityWire();
  inconsistent.server_owned_transport = false;
  const normalizedInconsistent = normalizeGatewayCapabilities(inconsistent);
  assert.equal(normalizedInconsistent.infrastructureReady, false);
  assert.ok(
    normalizedInconsistent.reasons.includes("capability_claim_inconsistent"),
  );

  const review = reviewWire();
  review.capabilities = inconsistent;
  const denied = normalizeGatewayReview(review, expectation(), nowMs);
  assert.equal(denied.ready, false);
  assert.equal(denied.canRequestServerStart, false);
  assert.ok(denied.reasons.includes("gateway_infrastructure_unavailable"));
});

test("malformed manifest digest and oversized reasons are rejected", () => {
  const malformed = reviewWire();
  target(identity(malformed)).manifest_revision = "sha256:not-a-digest";
  assert.throws(
    () => normalizeGatewayReview(malformed, expectation(), nowMs),
    /execution identity is incomplete/,
  );

  const unbounded = reviewWire();
  unbounded.reasons = Array.from({ length: 65 }, () => "reason");
  assert.throws(
    () => normalizeGatewayReview(unbounded, expectation(), nowMs),
    /gateway reasons are invalid/,
  );
});

function expectation(): GatewayReviewExpectation {
  return {
    identity: expectedIdentity(),
    statusSequence: 10,
    runtimePlanSequence: 20,
    evidenceRequestId: "plan-request-1",
  };
}

function expectedIdentity(): GatewayExecutionIdentity {
  return {
    projectId: "project-1",
    projectRevision: 11,
    programId: "program-7",
    programRevision: 7,
    planId: "plan-42",
    planRevision: 42,
    programHash: "fnv1a64:1234",
    programDigest: `sha256:${"c".repeat(64)}`,
    trajectoryDigest: `sha256:${"d".repeat(64)}`,
    robotModel: "rosie_1400_v3",
    target: {
      profileId: "bench-relay",
      manifestRef: "rt-core/config/cells/bench-relay.json",
      cellId: "bench-relay",
      manifestRevision: `sha256:${"a".repeat(64)}`,
    },
  };
}

function reviewWire(): Record<string, unknown> {
  return {
    schema: "offline-programming.execution-gateway-review.v1",
    reviewed_at: "2026-07-30T07:59:59.900Z",
    ready: true,
    checks: [
      { code: "gateway_infrastructure_ready", passed: true, detail: "" },
      { code: "exact_status_plan_target_correlation", passed: true, detail: "" },
    ],
    reasons: [],
    identity: identityWire(),
    capabilities: capabilityWire(),
    token: {
      schema: "offline-programming.execution-review-token.v1",
      token_id: "opg-review-1",
      issued_at: "2026-07-30T07:59:59.900Z",
      expires_at: "2026-07-30T08:00:04.900Z",
      controller: controllerWire(),
      identity: identityWire(),
      status_sequence: 10,
      runtime_plan_sequence: 20,
      evidence_request_id: "plan-request-1",
    },
  };
}

function capabilityWire(): Record<string, unknown> {
  return {
    schema: "offline-programming.execution-gateway-capabilities.v1",
    evaluated_at: "2026-07-30T07:59:59.900Z",
    controller: controllerWire(),
    gateway_enabled: true,
    connected_execution_enabled: true,
    server_controller_authority: true,
    manifest_allowlist_injected: true,
    authoritative_evidence_injected: true,
    session_observer_injected: true,
    session_monitoring: true,
    server_transport_injected: true,
    server_owned_transport: true,
    lease_authority_injected: true,
    fence_validation_injected: true,
    audit_sink_injected: true,
    idempotency_store_injected: true,
    review_tokens: true,
    plan_preload: true,
    controlled_start_arm: true,
    plan_play: true,
    protective_stop: true,
    protective_disarm: true,
    protective_weld_off: true,
    arm: false,
    weld_on: false,
    jog: false,
    teleop: false,
    raw_browser_transport: false,
    infrastructure_ready: true,
    reasons: [],
  };
}

function identityWire(): Record<string, unknown> {
  const value = expectedIdentity();
  return {
    project_id: value.projectId,
    project_revision: value.projectRevision,
    program_id: value.programId,
    program_revision: value.programRevision,
    plan_id: value.planId,
    plan_revision: value.planRevision,
    program_hash: value.programHash,
    program_digest: value.programDigest,
    trajectory_digest: value.trajectoryDigest,
    robot_model: value.robotModel,
    target: {
      profile_id: value.target.profileId,
      manifest_ref: value.target.manifestRef,
      cell_id: value.target.cellId,
      manifest_revision: value.target.manifestRevision,
    },
  };
}

function controllerWire(): Record<string, unknown> {
  return {
    controller_id: "offline-programming:server-1",
    controller_boot_id: "offline-programming-boot-1",
  };
}

function identity(review: Record<string, unknown>): Record<string, unknown> {
  return review.identity as Record<string, unknown>;
}

function token(review: Record<string, unknown>): Record<string, unknown> {
  return review.token as Record<string, unknown>;
}

function tokenIdentity(review: Record<string, unknown>): Record<string, unknown> {
  return token(review).identity as Record<string, unknown>;
}

function target(value: Record<string, unknown>): Record<string, unknown> {
  return value.target as Record<string, unknown>;
}
