import {
  normalizeGatewayCapabilities,
  normalizeGatewayExecutionIdentity,
  normalizeGatewayReview,
  sameGatewayIdentity,
  type GatewayCapabilityReport,
  type GatewayExecutionIdentity,
  type GatewayReviewExpectation,
  type GatewayReviewResult,
} from "./gateway.ts";

export const EXECUTION_GATEWAY_CAPABILITIES_PATH =
  "/api/offline-programming/v1/execution-gateway/capabilities" as const;
export const EXECUTION_PATH =
  "/api/offline-programming/v1/execution" as const;

export interface GatewayExecutionSession {
  schema: "offline-programming.execution-session.v1";
  sessionId: string;
  startedAtMs: number;
  identity: GatewayExecutionIdentity;
  controllerId: string;
  campaignGeneration: number;
}

export class ExecutionGatewayClient {
  readonly #fetch: typeof fetch;

  constructor(fetcher: typeof fetch = globalThis.fetch.bind(globalThis)) {
    this.#fetch = fetcher;
  }

  async capabilities(signal?: AbortSignal): Promise<GatewayCapabilityReport> {
    const response = await this.#fetch(EXECUTION_GATEWAY_CAPABILITIES_PATH, {
      method: "GET",
      cache: "no-store",
      credentials: "same-origin",
      redirect: "error",
      signal,
    });
    const payload: unknown = await response.json();
    if (!response.ok) {
      throw new Error(`execution gateway capabilities HTTP ${response.status}`);
    }
    return normalizeGatewayCapabilities(payload);
  }

  async review(
    wire: unknown,
    expected: GatewayReviewExpectation,
    signal?: AbortSignal,
  ): Promise<GatewayReviewResult> {
    const response = await this.#fetch(`${EXECUTION_PATH}/review`, {
      method: "POST",
      headers: {
        "Accept": "application/json",
        "Content-Type": "application/json",
      },
      credentials: "same-origin",
      redirect: "error",
      body: JSON.stringify(wire),
      signal,
    });
    const payload: unknown = await response.json();
    if (!response.ok) {
      throw new Error(apiError(payload, `execution review HTTP ${response.status}`));
    }
    return normalizeGatewayReview(payload, expected, Date.now());
  }

  async start(
    requestId: string,
    reviewToken: string,
    reviewWire: unknown,
    expected: GatewayExecutionIdentity,
    signal?: AbortSignal,
  ): Promise<GatewayExecutionSession> {
    boundedID(requestId);
    boundedID(reviewToken);
    const response = await this.#fetch(`${EXECUTION_PATH}/start`, {
      method: "POST",
      headers: {
        "Accept": "application/json",
        "Content-Type": "application/json",
      },
      credentials: "same-origin",
      redirect: "error",
      body: JSON.stringify({
        request_id: requestId,
        review_token: reviewToken,
        review: reviewWire,
      }),
      signal,
    });
    const payload: unknown = await response.json();
    if (!response.ok) {
      throw new Error(apiError(payload, `execution start HTTP ${response.status}`));
    }
    return normalizeSession(payload, expected);
  }

  async stop(
    sessionId: string,
    requestId: string,
    reason = "operator_stop",
    signal?: AbortSignal,
  ): Promise<void> {
    boundedID(sessionId);
    boundedID(requestId);
    const response = await this.#fetch(
      `${EXECUTION_PATH}/${encodeURIComponent(sessionId)}/stop`,
      {
        method: "POST",
        headers: {
          "Accept": "application/json",
          "Content-Type": "application/json",
        },
        credentials: "same-origin",
        redirect: "error",
        body: JSON.stringify({ request_id: requestId, reason }),
        signal,
      },
    );
    const payload: unknown = await response.json();
    if (!response.ok) {
      throw new Error(apiError(payload, `execution stop HTTP ${response.status}`));
    }
    const result = record(payload);
    if (!result ||
        result.schema !== "offline-programming.execution-stop.v1" ||
        result.execution_id !== sessionId ||
        result.stopped !== true) {
      throw new Error("execution stop response is invalid");
    }
  }

  async session(
    sessionId: string,
    expected: GatewayExecutionIdentity,
    signal?: AbortSignal,
  ): Promise<GatewayExecutionSession> {
    boundedID(sessionId);
    const response = await this.#fetch(
      `${EXECUTION_PATH}/${encodeURIComponent(sessionId)}`,
      {
        method: "GET",
        cache: "no-store",
        credentials: "same-origin",
        redirect: "error",
        signal,
      },
    );
    const payload: unknown = await response.json();
    if (!response.ok) {
      throw new Error(apiError(payload, `execution session HTTP ${response.status}`));
    }
    return normalizeSession(payload, expected);
  }
}

function normalizeSession(
  input: unknown,
  expected: GatewayExecutionIdentity,
): GatewayExecutionSession {
  const value = record(input);
  if (!value || value.schema !== "offline-programming.execution-session.v1") {
    throw new Error("execution session response is invalid");
  }
  const identity = normalizeGatewayExecutionIdentity(value.identity);
  const controller = record(value.controller);
  const startedAtMs = typeof value.started_at === "string"
    ? Date.parse(value.started_at)
    : Number.NaN;
  if (!sameGatewayIdentity(identity, expected) ||
      !controller ||
      typeof controller.controller_id !== "string" ||
      !controller.controller_id ||
      typeof value.session_id !== "string" ||
      !value.session_id ||
      !Number.isFinite(startedAtMs) ||
      !Number.isSafeInteger(value.campaign_generation) ||
      Number(value.campaign_generation) < 1) {
    throw new Error("execution session identity is invalid");
  }
  return {
    schema: "offline-programming.execution-session.v1",
    sessionId: value.session_id,
    startedAtMs,
    identity,
    controllerId: controller.controller_id,
    campaignGeneration: Number(value.campaign_generation),
  };
}

function apiError(payload: unknown, fallback: string): string {
  const value = record(payload);
  return value && typeof value.error === "string" && value.error.trim()
    ? value.error
    : fallback;
}

function boundedID(value: string): void {
  if (!value || value.length > 128 || !/^[\x21-\x7e]+$/.test(value)) {
    throw new Error("execution request identity is invalid");
  }
}

function record(value: unknown): Record<string, unknown> | null {
  return typeof value === "object" &&
      value !== null &&
      !Array.isArray(value)
    ? value as Record<string, unknown>
    : null;
}
