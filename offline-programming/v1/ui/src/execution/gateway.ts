export const EXECUTION_GATEWAY_CAPABILITY_SCHEMA =
  "offline-programming.execution-gateway-capabilities.v1" as const;
export const EXECUTION_GATEWAY_REVIEW_SCHEMA =
  "offline-programming.execution-gateway-review.v1" as const;
export const EXECUTION_REVIEW_TOKEN_SCHEMA =
  "offline-programming.execution-review-token.v1" as const;

export const GATEWAY_BROWSER_BOUNDARY = Object.freeze({
  canMintReviewToken: false as const,
  canAcquireLease: false as const,
  canSubmitCommand: false as const,
  canExecuteInBrowser: false as const,
  rawBrowserTransport: false as const,
  reason: "server_owned_execution_gateway_required" as const,
});

export interface GatewayControllerIdentity {
  controllerId: string;
  controllerBootId: string;
}

export interface GatewayTargetRevisionIdentity {
  profileId: string;
  manifestRef: string;
  cellId: string;
  manifestRevision: string;
}

export interface GatewayExecutionIdentity {
  projectId: string;
  projectRevision: number;
  programId: string;
  programRevision: number;
  planId: string;
  planRevision: number;
  programHash: string;
  programDigest: string;
  trajectoryDigest: string;
  /** The robot description the plan was made against, by directory name.
   *  Checked as a NAME: which robots exist is the server's answer
   *  (`domain/robots.ts`), and a gateway that carried the list would refuse a
   *  robot the moment one was added. */
  robotModel: string;
  target: GatewayTargetRevisionIdentity;
}

export interface GatewayCapabilityReport {
  schema: typeof EXECUTION_GATEWAY_CAPABILITY_SCHEMA;
  evaluatedAtMs: number;
  controller: GatewayControllerIdentity;
  gatewayEnabled: boolean;
  connectedExecutionEnabled: boolean;
  serverControllerAuthority: boolean;
  manifestAllowlistInjected: boolean;
  authoritativeEvidenceInjected: boolean;
  sessionObserverInjected: boolean;
  sessionMonitoring: boolean;
  serverTransportInjected: boolean;
  serverOwnedTransport: boolean;
  leaseAuthorityInjected: boolean;
  fenceValidationInjected: boolean;
  auditSinkInjected: boolean;
  idempotencyStoreInjected: boolean;
  reviewTokens: boolean;
  planPreload: boolean;
  controlledStartArm: boolean;
  planPlay: boolean;
  protectiveStop: boolean;
  protectiveDisarm: boolean;
  protectiveWeldOff: boolean;
  arm: false;
  weldOn: false;
  jog: false;
  teleop: false;
  rawBrowserTransport: false;
  infrastructureReady: boolean;
  reasons: string[];
}

export interface GatewayReviewToken {
  schema: typeof EXECUTION_REVIEW_TOKEN_SCHEMA;
  tokenId: string;
  issuedAtMs: number;
  expiresAtMs: number;
  controller: GatewayControllerIdentity;
  identity: GatewayExecutionIdentity;
  statusSequence: number;
  runtimePlanSequence: number;
  evidenceRequestId: string;
}

export interface GatewayReviewExpectation {
  identity: GatewayExecutionIdentity;
  statusSequence: number;
  runtimePlanSequence: number;
  evidenceRequestId: string;
}

export interface GatewayReviewResult {
  schema: typeof EXECUTION_GATEWAY_REVIEW_SCHEMA;
  reviewedAtMs: number;
  ready: boolean;
  checks: Array<{ code: string; passed: boolean; detail: string }>;
  reasons: string[];
  identity: GatewayExecutionIdentity;
  capabilities: GatewayCapabilityReport;
  token: GatewayReviewToken | null;
  canRequestServerStart: boolean;
  canExecuteInBrowser: false;
}

export function unavailableGatewayCapabilities(
  evaluatedAtMs: number,
): GatewayCapabilityReport {
  if (!validTime(evaluatedAtMs)) {
    throw new Error("gateway capability time is invalid");
  }
  return {
    schema: EXECUTION_GATEWAY_CAPABILITY_SCHEMA,
    evaluatedAtMs,
    controller: { controllerId: "", controllerBootId: "" },
    gatewayEnabled: false,
    connectedExecutionEnabled: false,
    serverControllerAuthority: false,
    manifestAllowlistInjected: false,
    authoritativeEvidenceInjected: false,
    sessionObserverInjected: false,
    sessionMonitoring: false,
    serverTransportInjected: false,
    serverOwnedTransport: false,
    leaseAuthorityInjected: false,
    fenceValidationInjected: false,
    auditSinkInjected: false,
    idempotencyStoreInjected: false,
    reviewTokens: false,
    planPreload: false,
    controlledStartArm: false,
    planPlay: false,
    protectiveStop: false,
    protectiveDisarm: false,
    protectiveWeldOff: false,
    arm: false,
    weldOn: false,
    jog: false,
    teleop: false,
    rawBrowserTransport: false,
    infrastructureReady: false,
    reasons: [
      "manifest_allowlist_not_injected",
      "fresh_authoritative_status_unavailable",
      "authoritative_runtime_playable_evidence_unavailable",
      "server_transport_not_injected",
    ],
  };
}

export function normalizeGatewayCapabilities(
  input: unknown,
): GatewayCapabilityReport {
  const object = record(input, "gateway capabilities");
  if (object.schema !== EXECUTION_GATEWAY_CAPABILITY_SCHEMA) {
    throw new Error("gateway capability schema is unsupported");
  }
  const reasons = boundedReasons(object.reasons);
  const controller = controllerIdentity(object.controller);
  const booleans = {
    gatewayEnabled: boolean(object.gateway_enabled, "gateway_enabled"),
    connectedExecutionEnabled: boolean(
      object.connected_execution_enabled,
      "connected_execution_enabled",
    ),
    serverControllerAuthority: boolean(
      object.server_controller_authority,
      "server_controller_authority",
    ),
    manifestAllowlistInjected: boolean(
      object.manifest_allowlist_injected,
      "manifest_allowlist_injected",
    ),
    authoritativeEvidenceInjected: boolean(
      object.authoritative_evidence_injected,
      "authoritative_evidence_injected",
    ),
    sessionObserverInjected: boolean(
      object.session_observer_injected,
      "session_observer_injected",
    ),
    sessionMonitoring: boolean(
      object.session_monitoring,
      "session_monitoring",
    ),
    serverTransportInjected: boolean(
      object.server_transport_injected,
      "server_transport_injected",
    ),
    serverOwnedTransport: boolean(
      object.server_owned_transport,
      "server_owned_transport",
    ),
    leaseAuthorityInjected: boolean(
      object.lease_authority_injected,
      "lease_authority_injected",
    ),
    fenceValidationInjected: boolean(
      object.fence_validation_injected,
      "fence_validation_injected",
    ),
    auditSinkInjected: boolean(object.audit_sink_injected, "audit_sink_injected"),
    idempotencyStoreInjected: boolean(
      object.idempotency_store_injected,
      "idempotency_store_injected",
    ),
    reviewTokens: boolean(object.review_tokens, "review_tokens"),
    planPreload: boolean(object.plan_preload, "plan_preload"),
    controlledStartArm: boolean(
      object.controlled_start_arm,
      "controlled_start_arm",
    ),
    planPlay: boolean(object.plan_play, "plan_play"),
    protectiveStop: boolean(object.protective_stop, "protective_stop"),
    protectiveDisarm: boolean(object.protective_disarm, "protective_disarm"),
    protectiveWeldOff: boolean(object.protective_weld_off, "protective_weld_off"),
  };
  const unsafeClaim = boolean(object.arm, "arm") ||
    boolean(object.weld_on, "weld_on") ||
    boolean(object.jog, "jog") ||
    boolean(object.teleop, "teleop") ||
    boolean(object.raw_browser_transport, "raw_browser_transport");
  const claimedReady = boolean(object.infrastructure_ready, "infrastructure_ready");
  const recomputedReady = Object.values(booleans).every(Boolean) &&
    validController(controller) &&
    !unsafeClaim;
  if (unsafeClaim) addReason(reasons, "unsafe_browser_or_energizing_capability_claim");
  if (claimedReady !== recomputedReady) {
    addReason(reasons, "capability_claim_inconsistent");
  }
  return {
    schema: EXECUTION_GATEWAY_CAPABILITY_SCHEMA,
    evaluatedAtMs: timestamp(object.evaluated_at, "evaluated_at"),
    controller,
    ...booleans,
    arm: false,
    weldOn: false,
    jog: false,
    teleop: false,
    rawBrowserTransport: false,
    infrastructureReady: claimedReady && recomputedReady && reasons.length === 0,
    reasons,
  };
}

export function normalizeGatewayReview(
  input: unknown,
  expected: GatewayReviewExpectation,
  nowMs: number,
): GatewayReviewResult {
  if (!validTime(nowMs) ||
      !completeIdentity(expected.identity) ||
      !positiveInteger(expected.statusSequence) ||
      !positiveInteger(expected.runtimePlanSequence) ||
      !boundedText(expected.evidenceRequestId, 128)) {
    throw new Error("gateway review expectation is incomplete");
  }
  const object = record(input, "gateway review");
  if (object.schema !== EXECUTION_GATEWAY_REVIEW_SCHEMA) {
    throw new Error("gateway review schema is unsupported");
  }
  const reviewedAtMs = timestamp(object.reviewed_at, "reviewed_at");
  const identity = executionIdentity(object.identity);
  const capabilities = normalizeGatewayCapabilities(object.capabilities);
  const checks = reviewChecks(object.checks);
  const reasons = boundedReasons(object.reasons);
  const claimedReady = boolean(object.ready, "ready");
  let token: GatewayReviewToken | null = null;
  let tokenFailure = "";
  if (object.token !== undefined && object.token !== null) {
    try {
      token = reviewToken(object.token);
      if (!sameIdentity(token.identity, expected.identity) ||
          !sameIdentity(identity, expected.identity) ||
          token.controller.controllerId !== capabilities.controller.controllerId ||
          token.controller.controllerBootId !== capabilities.controller.controllerBootId ||
          token.statusSequence < expected.statusSequence ||
          token.runtimePlanSequence < expected.runtimePlanSequence ||
          token.evidenceRequestId !== expected.evidenceRequestId ||
          reviewedAtMs > nowMs ||
          token.issuedAtMs < reviewedAtMs ||
          token.issuedAtMs > nowMs ||
          token.expiresAtMs <= nowMs ||
          token.expiresAtMs - token.issuedAtMs > 30_000) {
        tokenFailure = "review_token_binding_invalid";
      }
    } catch {
      tokenFailure = "review_token_malformed";
    }
  } else if (claimedReady) {
    tokenFailure = "review_token_missing";
  }
  if (tokenFailure) {
    token = null;
    addReason(reasons, tokenFailure);
  }
  if (!sameIdentity(identity, expected.identity)) {
    addReason(reasons, "review_identity_mismatch");
  }
  if (checks.some(({ passed }) => !passed)) {
    addReason(reasons, "review_check_failed");
  }
  if (!capabilities.infrastructureReady) {
    addReason(reasons, "gateway_infrastructure_unavailable");
  }
  const ready = claimedReady &&
    reviewedAtMs <= nowMs &&
    sameIdentity(identity, expected.identity) &&
    checks.every(({ passed }) => passed) &&
    capabilities.infrastructureReady &&
    token !== null &&
    reasons.length === 0;
  return {
    schema: EXECUTION_GATEWAY_REVIEW_SCHEMA,
    reviewedAtMs,
    ready,
    checks,
    reasons,
    identity,
    capabilities,
    token,
    canRequestServerStart: ready,
    canExecuteInBrowser: false,
  };
}

export function sameGatewayIdentity(
  left: GatewayExecutionIdentity,
  right: GatewayExecutionIdentity,
): boolean {
  return sameIdentity(left, right);
}

export function normalizeGatewayExecutionIdentity(
  input: unknown,
): GatewayExecutionIdentity {
  return executionIdentity(input);
}

function reviewToken(input: unknown): GatewayReviewToken {
  const object = record(input, "review token");
  if (object.schema !== EXECUTION_REVIEW_TOKEN_SCHEMA) {
    throw new Error("review token schema is unsupported");
  }
  const token = {
    schema: EXECUTION_REVIEW_TOKEN_SCHEMA,
    tokenId: text(object.token_id, "token_id", 128),
    issuedAtMs: timestamp(object.issued_at, "issued_at"),
    expiresAtMs: timestamp(object.expires_at, "expires_at"),
    controller: controllerIdentity(object.controller),
    identity: executionIdentity(object.identity),
    statusSequence: positive(object.status_sequence, "status_sequence"),
    runtimePlanSequence: positive(
      object.runtime_plan_sequence,
      "runtime_plan_sequence",
    ),
    evidenceRequestId: text(
      object.evidence_request_id,
      "evidence_request_id",
      128,
    ),
  };
  if (token.expiresAtMs <= token.issuedAtMs) {
    throw new Error("review token lifetime is invalid");
  }
  return token;
}

/** A robot description directory name, which is all this layer can judge. */
function isRobotModelName(value: unknown): value is string {
  return typeof value === "string" && /^[A-Za-z0-9][A-Za-z0-9_-]{0,127}$/.test(value);
}

function executionIdentity(input: unknown): GatewayExecutionIdentity {
  const object = record(input, "execution identity");
  const targetObject = record(object.target, "target revision identity");
  const robotModel = object.robot_model;
  if (!isRobotModelName(robotModel)) {
    throw new Error("execution robot model is not a robot description name");
  }
  const identity: GatewayExecutionIdentity = {
    projectId: text(object.project_id, "project_id", 128),
    projectRevision: positive(object.project_revision, "project_revision"),
    programId: text(object.program_id, "program_id", 128),
    programRevision: positive(object.program_revision, "program_revision"),
    planId: text(object.plan_id, "plan_id", 128),
    planRevision: positive(object.plan_revision, "plan_revision"),
    programHash: text(object.program_hash, "program_hash", 256),
    programDigest: text(object.program_digest, "program_digest", 71),
    trajectoryDigest: text(
      object.trajectory_digest,
      "trajectory_digest",
      71,
    ),
    robotModel,
    target: {
      profileId: text(targetObject.profile_id, "profile_id", 128),
      manifestRef: text(targetObject.manifest_ref, "manifest_ref", 512),
      cellId: text(targetObject.cell_id, "cell_id", 128),
      manifestRevision: text(
        targetObject.manifest_revision,
        "manifest_revision",
        71,
      ),
    },
  };
  if (!completeIdentity(identity)) {
    throw new Error("execution identity is incomplete");
  }
  return identity;
}

function controllerIdentity(input: unknown): GatewayControllerIdentity {
  const object = record(input, "controller identity");
  return {
    controllerId: optionalText(object.controller_id, 63),
    controllerBootId: optionalText(object.controller_boot_id, 63),
  };
}

function reviewChecks(
  input: unknown,
): Array<{ code: string; passed: boolean; detail: string }> {
  if (!Array.isArray(input) || input.length > 64) {
    throw new Error("gateway review checks are invalid");
  }
  return input.map((value) => {
    const object = record(value, "review check");
    return {
      code: text(object.code, "review check code", 160),
      passed: boolean(object.passed, "review check passed"),
      detail: optionalText(object.detail, 512),
    };
  });
}

function completeIdentity(identity: GatewayExecutionIdentity): boolean {
  return boundedText(identity.projectId, 128) &&
    positiveInteger(identity.projectRevision) &&
    boundedText(identity.programId, 128) &&
    positiveInteger(identity.programRevision) &&
    boundedText(identity.planId, 128) &&
    positiveInteger(identity.planRevision) &&
    boundedText(identity.programHash, 256) &&
    /^sha256:[0-9a-f]{64}$/.test(identity.programDigest) &&
    /^sha256:[0-9a-f]{64}$/.test(identity.trajectoryDigest) &&
    isRobotModelName(identity.robotModel) &&
    boundedText(identity.target.profileId, 128) &&
    boundedText(identity.target.manifestRef, 512) &&
    boundedText(identity.target.cellId, 128) &&
    /^sha256:[0-9a-f]{64}$/.test(identity.target.manifestRevision);
}

function sameIdentity(
  left: GatewayExecutionIdentity,
  right: GatewayExecutionIdentity,
): boolean {
  return left.projectId === right.projectId &&
    left.projectRevision === right.projectRevision &&
    left.programId === right.programId &&
    left.programRevision === right.programRevision &&
    left.planId === right.planId &&
    left.planRevision === right.planRevision &&
    left.programHash === right.programHash &&
    left.programDigest === right.programDigest &&
    left.trajectoryDigest === right.trajectoryDigest &&
    left.robotModel === right.robotModel &&
    left.target.profileId === right.target.profileId &&
    left.target.manifestRef === right.target.manifestRef &&
    left.target.cellId === right.target.cellId &&
    left.target.manifestRevision === right.target.manifestRevision;
}

function validController(controller: GatewayControllerIdentity): boolean {
  return printable(controller.controllerId, 63) &&
    printable(controller.controllerBootId, 63);
}

function boundedReasons(input: unknown): string[] {
  if (!Array.isArray(input) || input.length > 64) {
    throw new Error("gateway reasons are invalid");
  }
  return input.map((value) => text(value, "gateway reason", 160));
}

function addReason(reasons: string[], reason: string): void {
  if (!reasons.includes(reason)) reasons.push(reason);
}

function record(value: unknown, label: string): Record<string, unknown> {
  if (typeof value !== "object" || value === null || Array.isArray(value)) {
    throw new Error(`${label} must be an object`);
  }
  return value as Record<string, unknown>;
}

function boolean(value: unknown, label: string): boolean {
  if (typeof value !== "boolean") throw new Error(`${label} must be boolean`);
  return value;
}

function text(value: unknown, label: string, maximum: number): string {
  if (!boundedText(value, maximum)) throw new Error(`${label} is invalid`);
  return value.trim();
}

function optionalText(value: unknown, maximum: number): string {
  if (typeof value !== "string" || value.length > maximum) {
    throw new Error("optional text is invalid");
  }
  return value.trim();
}

function printable(value: string, maximum: number): boolean {
  return value.length > 0 &&
    value.length <= maximum &&
    [...value].every((char) => {
      const code = char.charCodeAt(0);
      return code >= 0x21 && code <= 0x7e;
    });
}

function boundedText(value: unknown, maximum: number): value is string {
  return typeof value === "string" &&
    value.trim().length > 0 &&
    value.length <= maximum;
}

function positive(value: unknown, label: string): number {
  if (!positiveInteger(value)) throw new Error(`${label} is invalid`);
  return value;
}

function positiveInteger(value: unknown): value is number {
  return typeof value === "number" &&
    Number.isSafeInteger(value) &&
    value > 0;
}

function timestamp(value: unknown, label: string): number {
  if (typeof value !== "string") throw new Error(`${label} is invalid`);
  const parsed = Date.parse(value);
  if (!validTime(parsed)) throw new Error(`${label} is invalid`);
  return parsed;
}

function validTime(value: number): boolean {
  return Number.isFinite(value) && value >= 0;
}
