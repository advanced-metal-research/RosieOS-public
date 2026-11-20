import { isRecord } from "./contracts.ts";

export const CONNECTED_STATUS_SCHEMA = "offline-programming.connected-status.v1" as const;
export const CONNECTED_PLAN_SCHEMA = "offline-programming.connected-plan.v1" as const;
export const MAX_CONNECTED_CORRELATION_REASONS = 32;

export interface ConnectedTargetIdentity {
  profileId: string;
  manifestRef: string;
  cellId: string;
}

export interface ConnectedBooleanFact {
  known: boolean;
  value: boolean;
}

export interface ConnectedRuntimePlanFacts {
  preloaded: ConnectedBooleanFact;
  loadedProgramAvailable: ConnectedBooleanFact;
  loadedProgramPlayable: ConnectedBooleanFact;
  serverValidated: ConnectedBooleanFact;
  playAuthorized: ConnectedBooleanFact;
  playbackActive: ConnectedBooleanFact;
  loadedProgramRevision: number;
  planRevision: number;
  planSessionId: string;
}

export interface ConnectedRuntimeOutputFacts {
  armRequested: ConnectedBooleanFact;
  currentZeroHold: ConnectedBooleanFact;
  staleZeroHold: ConnectedBooleanFact;
  actualServoArmed: ConnectedBooleanFact;
  ethercatEnabled: ConnectedBooleanFact;
  motionOutputEnabled: ConnectedBooleanFact;
  weldOutputEnabled: ConnectedBooleanFact;
}

export interface ConnectedAuthorityObservation {
  known: boolean;
  controllerId: string;
  controllerBootId: string;
  leaseId: string;
  fenceEpoch: number;
  campaignGeneration: number;
  leaseFresh: boolean;
  transitionPending: boolean;
  expiresInNs: number;
  freshUntilMs: number;
}

export interface ConnectedStatusObservation {
  schema: typeof CONNECTED_STATUS_SCHEMA;
  target: ConnectedTargetIdentity;
  sourcePeer: string;
  sourceSubject: string;
  sequence: number;
  receivedAtMs: number;
  sourceUpdatedAtMs: number;
  freshUntilMs: number;
  connected: boolean;
  identityCorrelated: boolean;
  serviceName: string;
  serviceState: string;
  plan: ConnectedRuntimePlanFacts;
  output: ConnectedRuntimeOutputFacts;
  authority: ConnectedAuthorityObservation;
  correlationReasons: string[];
}

export interface ConnectedPlanIdentity {
  planId: string;
  planRevision: number;
  programId: string;
  programRevision: number;
  programHash: string;
  robotModel: string;
}

export interface ConnectedSafetyEvidence {
  available: boolean;
  clear: boolean;
  state: string;
  pointCount: number;
}

export interface ConnectedPlanObservation {
  schema: typeof CONNECTED_PLAN_SCHEMA;
  target: ConnectedTargetIdentity;
  sourcePeer: string;
  sourceSubject: string;
  sequence: number;
  receivedAtMs: number;
  freshUntilMs: number;
  identity: ConnectedPlanIdentity;
  plannerOwner: string;
  plannerBackend: string;
  validationState: string;
  accepted: ConnectedBooleanFact;
  playEnabled: ConnectedBooleanFact;
  runtimePreloadEnabled: ConnectedBooleanFact;
  sampleCount: number;
  safety: {
    collision: ConnectedSafetyEvidence;
    singularity: ConnectedSafetyEvidence;
    limits: ConnectedSafetyEvidence;
    runtime: ConnectedSafetyEvidence;
  };
  identityCorrelated: boolean;
  correlationReasons: string[];
}

export interface ConnectedStreamState {
  status: ConnectedStatusObservation | null;
  plan: ConnectedPlanObservation | null;
  statusSequence: number;
  planSequence: number;
  blockedBy: string[];
}

export type ConnectedStreamEvent =
  | { type: "status"; value: unknown }
  | { type: "plan"; value: unknown }
  | { type: "disconnect"; reason?: string };

export function initialConnectedStreamState(): ConnectedStreamState {
  return { status: null, plan: null, statusSequence: 0, planSequence: 0, blockedBy: [] };
}

// reduceConnectedStreams retains one status and one plan only. Repeated or
// older frames are rejected rather than accumulated in browser memory.
export function reduceConnectedStreams(
  state: ConnectedStreamState,
  event: ConnectedStreamEvent,
): ConnectedStreamState {
  if (event.type === "disconnect") {
    return {
      status: null,
      plan: null,
      statusSequence: state.statusSequence,
      planSequence: state.planSequence,
      blockedBy: [event.reason?.trim() || "connected_target_disconnected"],
    };
  }
  try {
    if (event.type === "status") {
      const status = normalizeConnectedStatus(event.value);
      if (status.sequence <= state.statusSequence) {
        return { ...state, blockedBy: ["status_sequence_stale_or_repeated"] };
      }
      return {
        ...state,
        status,
        statusSequence: status.sequence,
        blockedBy: [...status.correlationReasons],
      };
    }
    const plan = normalizeConnectedPlan(event.value);
    if (plan.sequence <= state.planSequence) {
      return { ...state, blockedBy: ["plan_sequence_stale_or_repeated"] };
    }
    return {
      ...state,
      plan,
      planSequence: plan.sequence,
      blockedBy: [...plan.correlationReasons],
    };
  } catch {
    return { ...state, blockedBy: ["connected_stream_frame_invalid"] };
  }
}

export function normalizeConnectedStatus(value: unknown): ConnectedStatusObservation {
  const record = schemaRecord(value, CONNECTED_STATUS_SCHEMA);
  const receivedAtMs = timestamp(record.received_at);
  const sourceUpdatedAtMs = timestamp(record.source_updated_at);
  const freshUntilMs = timestamp(record.fresh_until);
  const authority = record.authority;
  if (!isRecord(authority)) throw new Error("connected status authority is required");
  return {
    schema: CONNECTED_STATUS_SCHEMA,
    target: normalizeTargetIdentity(record.target),
    sourcePeer: requiredText(record.source_peer),
    sourceSubject: requiredText(record.source_subject),
    sequence: positiveInteger(record.sequence),
    receivedAtMs,
    sourceUpdatedAtMs,
    freshUntilMs,
    connected: requiredBoolean(record.connected),
    identityCorrelated: requiredBoolean(record.identity_correlated),
    serviceName: requiredText(record.service_name),
    serviceState: requiredText(record.service_state),
    plan: normalizeRuntimePlan(record.plan),
    output: normalizeRuntimeOutput(record.output),
    authority: {
      known: requiredBoolean(authority.known),
      controllerId: text(authority.controller_id),
      controllerBootId: text(authority.controller_boot_id),
      leaseId: text(authority.lease_id),
      fenceEpoch: nonnegativeInteger(authority.fence_epoch),
      campaignGeneration: nonnegativeInteger(authority.campaign_generation),
      leaseFresh: requiredBoolean(authority.lease_fresh),
      transitionPending: requiredBoolean(authority.transition_pending),
      expiresInNs: nonnegativeInteger(authority.expires_in),
      freshUntilMs: optionalTimestamp(authority.fresh_until),
    },
    correlationReasons: reasons(record.correlation_reasons),
  };
}

export function normalizeConnectedPlan(value: unknown): ConnectedPlanObservation {
  const record = schemaRecord(value, CONNECTED_PLAN_SCHEMA);
  if (!isRecord(record.identity) || !isRecord(record.safety)) {
    throw new Error("connected plan identity and safety evidence are required");
  }
  const safety = record.safety;
  return {
    schema: CONNECTED_PLAN_SCHEMA,
    target: normalizeTargetIdentity(record.target),
    sourcePeer: requiredText(record.source_peer),
    sourceSubject: requiredText(record.source_subject),
    sequence: positiveInteger(record.sequence),
    receivedAtMs: timestamp(record.received_at),
    freshUntilMs: timestamp(record.fresh_until),
    identity: {
      planId: text(record.identity.plan_id),
      planRevision: nonnegativeInteger(record.identity.plan_revision),
      programId: text(record.identity.program_id),
      programRevision: nonnegativeInteger(record.identity.program_revision),
      programHash: text(record.identity.program_hash),
      robotModel: text(record.identity.robot_model),
    },
    plannerOwner: text(record.planner_owner),
    plannerBackend: text(record.planner_backend),
    validationState: text(record.validation_state),
    accepted: fact(record.accepted),
    playEnabled: fact(record.play_enabled),
    runtimePreloadEnabled: fact(record.runtime_preload_enabled),
    sampleCount: nonnegativeInteger(record.sample_count),
    safety: {
      collision: evidence(safety.collision),
      singularity: evidence(safety.singularity),
      limits: evidence(safety.limits),
      runtime: evidence(safety.runtime),
    },
    identityCorrelated: requiredBoolean(record.identity_correlated),
    correlationReasons: reasons(record.correlation_reasons),
  };
}

export function connectedStatusIsFresh(
  status: ConnectedStatusObservation,
  nowMs: number,
): boolean {
  return status.connected &&
    status.identityCorrelated &&
    validTime(nowMs) &&
    status.sourceUpdatedAtMs <= nowMs &&
    nowMs < status.freshUntilMs;
}

export function connectedPlanIsFresh(
  plan: ConnectedPlanObservation,
  nowMs: number,
): boolean {
  return validTime(nowMs) &&
    plan.receivedAtMs <= nowMs &&
    nowMs < plan.freshUntilMs;
}

function normalizeRuntimePlan(value: unknown): ConnectedRuntimePlanFacts {
  if (!isRecord(value)) throw new Error("connected runtime plan facts are required");
  return {
    preloaded: fact(value.preloaded),
    loadedProgramAvailable: fact(value.loaded_program_available),
    loadedProgramPlayable: fact(value.loaded_program_playable),
    serverValidated: fact(value.server_validated),
    playAuthorized: fact(value.play_authorized),
    playbackActive: fact(value.playback_active),
    loadedProgramRevision: nonnegativeInteger(value.loaded_program_revision),
    planRevision: nonnegativeInteger(value.plan_revision),
    planSessionId: text(value.plan_session_id),
  };
}

function normalizeRuntimeOutput(value: unknown): ConnectedRuntimeOutputFacts {
  if (!isRecord(value)) throw new Error("connected output facts are required");
  return {
    armRequested: fact(value.arm_requested),
    currentZeroHold: fact(value.current_zero_hold),
    staleZeroHold: fact(value.stale_zero_hold),
    actualServoArmed: fact(value.actual_servo_armed),
    ethercatEnabled: fact(value.ethercat_enabled),
    motionOutputEnabled: fact(value.motion_output_enabled),
    weldOutputEnabled: fact(value.weld_output_enabled),
  };
}

function normalizeTargetIdentity(value: unknown): ConnectedTargetIdentity {
  if (!isRecord(value)) throw new Error("connected target identity is required");
  return {
    profileId: requiredText(value.profile_id),
    manifestRef: requiredText(value.manifest_ref),
    cellId: requiredText(value.cell_id),
  };
}

function fact(value: unknown): ConnectedBooleanFact {
  if (!isRecord(value)) throw new Error("connected boolean fact is required");
  return {
    known: requiredBoolean(value.known),
    value: requiredBoolean(value.value),
  };
}

function evidence(value: unknown): ConnectedSafetyEvidence {
  if (!isRecord(value)) throw new Error("connected safety evidence is required");
  return {
    available: requiredBoolean(value.available),
    clear: requiredBoolean(value.clear),
    state: text(value.state),
    pointCount: nonnegativeInteger(value.point_count),
  };
}

function schemaRecord(value: unknown, schema: string): Record<string, unknown> {
  if (!isRecord(value) || value.schema !== schema) throw new Error(`connected schema must be ${schema}`);
  return value;
}

function reasons(value: unknown): string[] {
  if (!Array.isArray(value) || value.length > MAX_CONNECTED_CORRELATION_REASONS) {
    throw new Error("connected correlation reasons exceed bound");
  }
  return value.map((entry) => {
    const reason = requiredText(entry);
    if (reason.length > 160) throw new Error("connected correlation reason exceeds bound");
    return reason;
  });
}

function requiredText(value: unknown): string {
  if (typeof value !== "string" || !value.trim() || value.length > 512) {
    throw new Error("nonempty bounded string required");
  }
  return value.trim();
}

function text(value: unknown): string {
  return typeof value === "string" ? value.trim() : "";
}

function requiredBoolean(value: unknown): boolean {
  if (typeof value !== "boolean") throw new Error("boolean required");
  return value;
}

function positiveInteger(value: unknown): number {
  if (!Number.isSafeInteger(value) || Number(value) <= 0) throw new Error("positive integer required");
  return Number(value);
}

function nonnegativeInteger(value: unknown): number {
  if (!Number.isSafeInteger(value) || Number(value) < 0) throw new Error("nonnegative integer required");
  return Number(value);
}

function timestamp(value: unknown): number {
  if (typeof value !== "string") throw new Error("timestamp required");
  const parsed = Date.parse(value);
  if (!validTime(parsed)) throw new Error("valid timestamp required");
  return parsed;
}

function optionalTimestamp(value: unknown): number {
  if (typeof value !== "string" || value === "0001-01-01T00:00:00Z") return 0;
  return timestamp(value);
}

function validTime(value: number): boolean {
  return Number.isFinite(value) && value >= 0;
}
