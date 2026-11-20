import { isRecord } from "./contracts.ts";
import {
  normalizeConnectedTargetDiscovery,
  type ConnectedTargetDiscoveryDescriptor,
} from "./connectedDiscovery.ts";
import {
  normalizeConnectedPlan,
  normalizeConnectedStatus,
  type ConnectedPlanObservation,
  type ConnectedStatusObservation,
} from "./connectedStreams.ts";

export const CONNECTED_TARGETS_PATH =
  "/api/offline-programming/v1/connected-targets" as const;
export const CONNECTED_TARGETS_SCHEMA =
  "offline-programming.connected-targets.v1" as const;
export const CONNECTED_TARGET_STATUS_SCHEMA =
  "offline-programming.connected-target-status.v1" as const;

export interface ConnectedManifestIssue {
  manifestRef: string;
  code: string;
  reason: string;
}

export interface ConnectedDiscoveryResult {
  targets: ConnectedTargetDiscoveryDescriptor[];
  issues: ConnectedManifestIssue[];
}

export interface ConnectedReadonlyStatus {
  schema: typeof CONNECTED_TARGET_STATUS_SCHEMA;
  target: ConnectedTargetDiscoveryDescriptor;
  state: "unavailable" | "read_only";
  observedAtMs: number;
  statusAvailable: boolean;
  planAvailable: boolean;
  connected: boolean;
  serviceName: string;
  serviceState: string;
  statusObservation: ConnectedStatusObservation | null;
  planObservation: ConnectedPlanObservation | null;
  reasons: string[];
}

export interface ConnectedReadonlyCapabilities {
  schema: "offline-programming.connected-capabilities.v1";
  manifestBackedDiscovery: boolean;
  readOnlyStatus: boolean;
  readOnlyPlan: boolean;
  identityCorrelated: boolean;
  authorityObserved: boolean;
  commandSubmission: false;
  arm: false;
  weldOn: false;
  play: false;
  jog: false;
  teleop: false;
  rawBrowserTransport: false;
  physicalExecution: false;
  reasons: string[];
}

export interface ConnectedTargetsClientOptions {
  fetch?: typeof fetch;
  timeoutMs?: number;
}

// This client has fixed same-origin GET routes only. It cannot accept a host,
// URL, subject, WebSocket, command body, or transport supplied by the browser.
export class ConnectedTargetsClient {
  #fetch: typeof fetch;
  #timeoutMs: number;

  constructor(options: ConnectedTargetsClientOptions = {}) {
    this.#fetch = options.fetch ?? globalThis.fetch.bind(globalThis);
    this.#timeoutMs = boundedInteger(options.timeoutMs ?? 5_000, 100, 30_000);
  }

  async discover(signal?: AbortSignal): Promise<ConnectedDiscoveryResult> {
    const value = await this.get(CONNECTED_TARGETS_PATH, signal);
    if (!isRecord(value) || value.schema !== CONNECTED_TARGETS_SCHEMA ||
        !Array.isArray(value.targets) || !Array.isArray(value.issues)) {
      throw new Error("connected target discovery response is invalid");
    }
    return {
      targets: value.targets.map(normalizeConnectedTargetDiscovery),
      issues: value.issues.map(normalizeIssue),
    };
  }

  async status(
    profileId: string,
    signal?: AbortSignal,
  ): Promise<ConnectedReadonlyStatus> {
    const value = await this.get(`${targetPath(profileId)}/status`, signal);
    if (!isRecord(value) || value.schema !== CONNECTED_TARGET_STATUS_SCHEMA) {
      throw new Error("connected target status response is invalid");
    }
    const state = value.state;
    if (state !== "unavailable" && state !== "read_only") {
      throw new Error("connected target status state is invalid");
    }
    return {
      schema: CONNECTED_TARGET_STATUS_SCHEMA,
      target: normalizeConnectedTargetDiscovery(value.target),
      state,
      observedAtMs: timestamp(value.observed_at),
      statusAvailable: boolean(value.status_available),
      planAvailable: boolean(value.plan_available),
      connected: boolean(value.connected),
      serviceName: text(value.service_name),
      serviceState: text(value.service_state),
      statusObservation: value.status_observation === undefined
        ? null
        : normalizeConnectedStatus(value.status_observation),
      planObservation: value.plan_observation === undefined
        ? null
        : normalizeConnectedPlan(value.plan_observation),
      reasons: reasonList(value.reasons),
    };
  }

  async capabilities(
    profileId: string,
    signal?: AbortSignal,
  ): Promise<ConnectedReadonlyCapabilities> {
    const value = await this.get(`${targetPath(profileId)}/capabilities`, signal);
    if (!isRecord(value) ||
        value.schema !== "offline-programming.connected-capabilities.v1") {
      throw new Error("connected capability response is invalid");
    }
    const physicalFalse = [
      "command_submission",
      "arm",
      "weld_on",
      "play",
      "jog",
      "teleop",
      "raw_browser_transport",
      "physical_execution",
    ] as const;
    if (physicalFalse.some((key) => value[key] !== false)) {
      throw new Error("connected capability response enabled a physical path");
    }
    return {
      schema: "offline-programming.connected-capabilities.v1",
      manifestBackedDiscovery: boolean(value.manifest_backed_discovery),
      readOnlyStatus: boolean(value.read_only_status),
      readOnlyPlan: boolean(value.read_only_plan),
      identityCorrelated: boolean(value.identity_correlated),
      authorityObserved: boolean(value.authority_observed),
      commandSubmission: false,
      arm: false,
      weldOn: false,
      play: false,
      jog: false,
      teleop: false,
      rawBrowserTransport: false,
      physicalExecution: false,
      reasons: reasonList(value.reasons),
    };
  }

  private async get(path: string, callerSignal?: AbortSignal): Promise<unknown> {
    const controller = new AbortController();
    const abort = (): void => controller.abort(callerSignal?.reason);
    callerSignal?.addEventListener("abort", abort, { once: true });
    const timeout = setTimeout(() => controller.abort("connected read timeout"), this.#timeoutMs);
    try {
      const response = await this.#fetch(path, {
        method: "GET",
        cache: "no-store",
        credentials: "same-origin",
        redirect: "error",
        signal: controller.signal,
      });
      const text = await response.text();
      const value = parseJsonPayload(text);
      if (!response.ok) {
        const detail = isRecord(value) && typeof value.error === "string"
          ? value.error
          : `connected read HTTP ${response.status}`;
        throw new Error(detail);
      }
      if (value === undefined) {
        throw new Error("connected read returned non-JSON");
      }
      return value;
    } finally {
      clearTimeout(timeout);
      callerSignal?.removeEventListener("abort", abort);
    }
  }
}

function targetPath(profileId: string): string {
  if (!/^[A-Za-z0-9._-]{1,96}$/.test(profileId)) {
    throw new Error("connected target profile identity is invalid");
  }
  return `${CONNECTED_TARGETS_PATH}/${encodeURIComponent(profileId)}`;
}

function normalizeIssue(value: unknown): ConnectedManifestIssue {
  if (!isRecord(value)) throw new Error("connected manifest issue is invalid");
  return {
    manifestRef: requiredText(value.manifest_ref),
    code: requiredText(value.code),
    reason: requiredText(value.reason),
  };
}

function reasonList(value: unknown): string[] {
  if (!Array.isArray(value) || value.length > 32) {
    throw new Error("connected status reasons exceed the bound");
  }
  return value.map(requiredText);
}

function timestamp(value: unknown): number {
  if (typeof value !== "string") throw new Error("connected timestamp is required");
  const parsed = Date.parse(value);
  if (!Number.isFinite(parsed)) throw new Error("connected timestamp is invalid");
  return parsed;
}

function requiredText(value: unknown): string {
  if (typeof value !== "string" || !value.trim() || value.length > 512) {
    throw new Error("connected bounded text is required");
  }
  return value.trim();
}

function text(value: unknown): string {
  return typeof value === "string" ? value.trim() : "";
}

function boolean(value: unknown): boolean {
  if (typeof value !== "boolean") throw new Error("connected boolean is required");
  return value;
}

function parseJsonPayload(text: string): unknown {
  const trimmed = text.trim();
  if (trimmed === "") return undefined;
  try {
    return JSON.parse(trimmed) as unknown;
  } catch {
    return undefined;
  }
}

function boundedInteger(value: number, minimum: number, maximum: number): number {
  if (!Number.isSafeInteger(value) || value < minimum || value > maximum) {
    throw new Error("connected timeout is out of bounds");
  }
  return value;
}
