import type { NormalizedPlanRequest } from "../execution/contracts.ts";
import {
  localPlanPlayableReasons,
  localSafetyReasons,
  localSimulatorReady,
  parseLocalPlanResult,
  parseLocalSimulatorStatus,
  type LocalPlanResult,
  type LocalSimulatorStatus,
} from "./local-contracts.ts";
import { localPlanRequestWire } from "./local-plan.ts";

export const LOCAL_SIMULATOR_PATHS = Object.freeze({
  status: "/api/offline-programming/v1/local-simulator/status",
  start: "/api/offline-programming/v1/local-simulator/start",
  stop: "/api/offline-programming/v1/local-simulator/stop",
  plans: "/api/offline-programming/v1/local-simulator/plans",
});

export type LocalClientErrorCode =
  | "invalid_endpoint"
  | "timeout"
  | "network"
  | "http"
  | "response_too_large"
  | "invalid_response"
  | "unsafe_status"
  | "not_ready"
  | "plan_rejected";

export class LocalClientError extends Error {
  readonly code: LocalClientErrorCode;
  readonly status: number | null;

  constructor(code: LocalClientErrorCode, message: string, status: number | null = null) {
    super(message);
    this.name = "LocalClientError";
    this.code = code;
    this.status = status;
  }
}

export class LocalPlanRejectedError extends LocalClientError {
  readonly result: LocalPlanResult;
  readonly reasons: string[];

  constructor(result: LocalPlanResult, reasons: string[]) {
    super(
      "plan_rejected",
      `canonical local plan is not runtime-playable: ${reasons.join(",")}`,
    );
    this.name = "LocalPlanRejectedError";
    this.result = result;
    this.reasons = [...reasons];
  }
}

export interface LocalSimulatorClientOptions {
  baseUrl?: string;
  fetch?: typeof fetch;
  timeoutMs?: number;
  operationTimeoutMs?: number;
  maxResponseBytes?: number;
  now?: () => number;
}

export class LocalSimulatorClient {
  readonly baseUrl: string;
  readonly timeoutMs: number;
  readonly operationTimeoutMs: number;
  readonly maxResponseBytes: number;
  #fetch: typeof fetch;
  #now: () => number;

  constructor(options: LocalSimulatorClientOptions = {}) {
    this.baseUrl = normalizeBaseUrl(options.baseUrl ?? "");
    this.timeoutMs = boundedInteger(options.timeoutMs ?? 10_000, 100, 120_000, "timeoutMs");
    this.operationTimeoutMs = boundedInteger(
      options.operationTimeoutMs ?? 270_000,
      1_000,
      600_000,
      "operationTimeoutMs",
    );
    this.maxResponseBytes = boundedInteger(
      options.maxResponseBytes ?? 64 << 20,
      1_024,
      256 << 20,
      "maxResponseBytes",
    );
    this.#fetch = options.fetch ?? globalThis.fetch.bind(globalThis);
    this.#now = options.now ?? Date.now;
  }

  async status(signal?: AbortSignal): Promise<LocalSimulatorStatus> {
    const raw = await this.request(LOCAL_SIMULATOR_PATHS.status, { method: "GET" }, signal);
    return parseStatusResponse(raw);
  }

  async start(signal?: AbortSignal): Promise<LocalSimulatorStatus> {
    const raw = await this.request(
      LOCAL_SIMULATOR_PATHS.start,
      { method: "POST" },
      signal,
      this.operationTimeoutMs,
    );
    const status = parseStatusResponse(raw);
    if (!localSimulatorReady(status, this.#now())) {
      throw new LocalClientError(
        "unsafe_status",
        `local simulator start failed closed: ${localSafetyReasons(status, this.#now()).join(",")}`,
      );
    }
    return status;
  }

  async stop(signal?: AbortSignal): Promise<LocalSimulatorStatus> {
    const raw = await this.request(
      LOCAL_SIMULATOR_PATHS.stop,
      { method: "POST" },
      signal,
      this.operationTimeoutMs,
    );
    return parseStatusResponse(raw);
  }

  async plan(
    request: NormalizedPlanRequest,
    signal?: AbortSignal,
  ): Promise<LocalPlanResult> {
    const status = await this.status(signal);
    if (!localSimulatorReady(status, this.#now())) {
      throw new LocalClientError(
        "not_ready",
        "local simulator is not ready with a fresh no-motion proof",
      );
    }
    if (
      status.profileId !== request.target.profileId ||
      status.robotModel !== request.robotModel
    ) {
      throw new LocalClientError(
        "not_ready",
        "local simulator status does not match the requested target and robot model",
      );
    }
    const raw = await this.request(
      LOCAL_SIMULATOR_PATHS.plans,
      {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(localPlanRequestWire(request)),
      },
      signal,
      this.operationTimeoutMs,
    );
    try {
      const result = parseLocalPlanResult(unwrap(raw, "result"), request);
      const rejectionReasons = localPlanPlayableReasons(result, request);
      if (rejectionReasons.length > 0) {
        throw new LocalPlanRejectedError(result, rejectionReasons);
      }
      const postStatus = await this.status(signal);
      if (
        !localSimulatorReady(postStatus, this.#now()) ||
        postStatus.profileId !== request.target.profileId ||
        postStatus.robotModel !== request.robotModel
      ) {
        throw new LocalClientError(
          "unsafe_status",
          "local simulator post-plan safety proof is missing, stale, or mismatched",
        );
      }
      return result;
    } catch (error) {
      if (error instanceof LocalClientError) throw error;
      throw invalidResponse(error);
    }
  }

  private async request(
    path: string,
    init: RequestInit,
    callerSignal?: AbortSignal,
    timeoutMs = this.timeoutMs,
  ): Promise<unknown> {
    const controller = new AbortController();
    let callerAborted = false;
    const onAbort = (): void => {
      callerAborted = true;
      controller.abort(callerSignal?.reason);
    };
    callerSignal?.addEventListener("abort", onAbort, { once: true });
    const timeout = setTimeout(() => controller.abort("local simulator timeout"), timeoutMs);
    let response: Response;
    try {
      response = await this.#fetch(`${this.baseUrl}${path}`, {
        ...init,
        signal: controller.signal,
        credentials: "same-origin",
        cache: "no-store",
        redirect: "error",
      });
    } catch (error) {
      if (controller.signal.aborted && !callerAborted) {
        throw new LocalClientError("timeout", "local simulator request timed out");
      }
      if (callerAborted) throw error;
      throw new LocalClientError(
        "network",
        `local simulator request failed: ${errorMessage(error)}`,
      );
    } finally {
      clearTimeout(timeout);
      callerSignal?.removeEventListener("abort", onAbort);
    }
    const declaredLength = Number(response.headers.get("content-length"));
    if (Number.isFinite(declaredLength) && declaredLength > this.maxResponseBytes) {
      throw new LocalClientError("response_too_large", "local simulator response is too large");
    }
    const text = await response.text();
    if (new TextEncoder().encode(text).byteLength > this.maxResponseBytes) {
      throw new LocalClientError("response_too_large", "local simulator response is too large");
    }
    const payload = parseJsonPayload(text);
    if (!response.ok) {
      const root = typeof payload === "object" && payload !== null
        ? payload as Record<string, unknown>
        : {};
      throw new LocalClientError(
        "http",
        typeof root.error === "string" ? root.error : `local simulator HTTP ${response.status}`,
        response.status,
      );
    }
    if (payload === undefined) {
      throw new LocalClientError("invalid_response", "local simulator returned invalid JSON");
    }
    return payload;
  }
}

function parseStatusResponse(value: unknown): LocalSimulatorStatus {
  try {
    return parseLocalSimulatorStatus(unwrap(value, "status"));
  } catch (error) {
    throw invalidResponse(error);
  }
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

function unwrap(value: unknown, key: string): unknown {
  if (typeof value !== "object" || value === null || Array.isArray(value)) return value;
  const root = value as Record<string, unknown>;
  return root[key] ?? value;
}

function normalizeBaseUrl(value: string): string {
  const base = value.trim().replace(/\/+$/, "");
  if (base === "") return "";
  let parsed: URL;
  try {
    parsed = new URL(base);
  } catch {
    throw new LocalClientError("invalid_endpoint", "local simulator base URL must be absolute");
  }
  const host = parsed.hostname.toLowerCase().replace(/^\[|\]$/g, "");
  const loopback = host === "localhost" || host === "::1" || host.startsWith("127.");
  if (
    (parsed.protocol !== "http:" && parsed.protocol !== "https:") ||
    !loopback ||
    parsed.username !== "" ||
    parsed.password !== "" ||
    parsed.search !== "" ||
    parsed.hash !== ""
  ) {
    throw new LocalClientError(
      "invalid_endpoint",
      "local simulator endpoint must be an unauthenticated loopback HTTP URL",
    );
  }
  return base;
}

function boundedInteger(
  value: number,
  minimum: number,
  maximum: number,
  name: string,
): number {
  if (!Number.isSafeInteger(value) || value < minimum || value > maximum) {
    throw new Error(`${name} is out of bounds`);
  }
  return value;
}

function invalidResponse(error: unknown): LocalClientError {
  return new LocalClientError(
    "invalid_response",
    `invalid local simulator response: ${errorMessage(error)}`,
  );
}

function errorMessage(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}
