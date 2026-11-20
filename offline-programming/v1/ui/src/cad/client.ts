import {
  CAD_API_PATH,
  CadClientError,
  type CadClient,
  type CadErrorCode,
  type CadMeshRequest,
  type CadOperation,
  type CadRequestState,
  type CadSeamRequest,
  type CadSeamResult,
  type CadSequenceRequest,
  type CadSequenceResult,
  type CadSTEPSource,
  type CadTopology,
  type CadTopologyRequest,
  type CadWorkpieceMesh,
} from "./contracts.ts";
import {
  normalizeSeamResult,
  normalizeSequenceResult,
  normalizeTopology,
  normalizeWorkpieceMesh,
} from "./normalize.ts";

const DEFAULT_TIMEOUT_MS = 95_000;
const MAX_STEP_BYTES = 8 << 20;
const MAX_SEQUENCE_SEAMS = 64;

// Mirrors the bounds enforced by internal/cad. Tessellating a large assembly
// runs far longer than sampling its edges, so mesh requests get their own
// client timeout to match the worker's longer mesh ceiling.
const DEFAULT_MESH_TIMEOUT_MS = 245_000;
const DEFAULT_LINEAR_DEFLECTION_MM = 0.5;
const DEFAULT_ANGULAR_DEFLECTION_RAD = 0.5;
const MIN_LINEAR_DEFLECTION_MM = 0.01;
const MAX_LINEAR_DEFLECTION_MM = 10;
const MIN_ANGULAR_DEFLECTION_RAD = 0.05;
const MAX_ANGULAR_DEFLECTION_RAD = 1.5;
const DEFAULT_MAX_TRIANGLES = 400_000;
const MIN_TRIANGLES = 1_000;
const MAX_TRIANGLES = 2_000_000;

export interface HttpCadClientOptions {
  endpoint?: string;
  timeoutMS?: number;
  meshTimeoutMS?: number;
  fetch?: typeof fetch;
}

export class HttpCadClient implements CadClient {
  readonly endpoint: string;
  readonly timeoutMS: number;
  readonly meshTimeoutMS: number;
  readonly fetch: typeof fetch;

  constructor(options: HttpCadClientOptions = {}) {
    this.endpoint = options.endpoint ?? CAD_API_PATH;
    this.timeoutMS = Math.max(1, Math.floor(options.timeoutMS ?? DEFAULT_TIMEOUT_MS));
    this.meshTimeoutMS = Math.max(
      1,
      Math.floor(options.meshTimeoutMS ?? DEFAULT_MESH_TIMEOUT_MS),
    );
    this.fetch = options.fetch ?? globalThis.fetch.bind(globalThis);
  }

  async extractTopology(request: CadTopologyRequest, signal?: AbortSignal): Promise<CadTopology> {
    const body = {
      operation: "extract_topology",
      ...sourceBody(request.source),
      sample_count: boundedInteger(request.sampleCount, 64, 8, 256, "sampleCount"),
    };
    return this.post("extract_topology", body, normalizeTopology, signal);
  }

  async buildSeam(request: CadSeamRequest, signal?: AbortSignal): Promise<CadSeamResult> {
    const window = request.window ?? { start: 0, end: 1 };
    validateWindow(window.start, window.end);
    const body = {
      operation: "build_seam",
      ...sourceBody(request.source),
      sample_count: boundedInteger(request.sampleCount, 64, 8, 256, "sampleCount"),
      seam_sample_count: boundedInteger(request.seamSampleCount, 40, 2, 512, "seamSampleCount"),
      edge_id: request.edgeId?.trim() || undefined,
      start_s: window.start,
      end_s: window.end,
      reversed: request.reversed ?? false,
      geometry_method: request.geometryMethod?.trim() || undefined,
      path_method: request.pathMethod?.trim() || undefined,
    };
    return this.post("build_seam", body, normalizeSeamResult, signal);
  }

  async buildSequence(request: CadSequenceRequest, signal?: AbortSignal): Promise<CadSequenceResult> {
    if (request.segments.length > MAX_SEQUENCE_SEAMS) {
      throw clientError("invalid_request", `segments exceeds the ${MAX_SEQUENCE_SEAMS} seam limit`, "build_sequence");
    }
    const segments = request.segments.map((segment, index) => {
      const edgeId = segment.edgeId.trim();
      if (!edgeId) {
        throw clientError("invalid_request", `segments[${index}].edgeId is required`, "build_sequence");
      }
      validateWindow(segment.window.start, segment.window.end, `segments[${index}].window`);
      return {
        id: segment.id?.trim() || undefined,
        edge_id: edgeId,
        start_s: segment.window.start,
        end_s: segment.window.end,
        reversed: segment.reversed ?? false,
      };
    });
    const body = {
      operation: "build_sequence",
      ...sourceBody(request.source),
      sample_count: boundedInteger(request.sampleCount, 64, 8, 256, "sampleCount"),
      seam_sample_count: boundedInteger(request.seamSampleCount, 40, 2, 512, "seamSampleCount"),
      segments,
    };
    return this.post("build_sequence", body, normalizeSequenceResult, signal);
  }

  /**
   * Tessellate the STEP into surface triangles. The response is keyed by the
   * STEP fingerprint and cached by the local CAD service, so callers should
   * request it once per fingerprint rather than per render.
   */
  async tessellate(request: CadMeshRequest, signal?: AbortSignal): Promise<CadWorkpieceMesh> {
    const body = {
      operation: "tessellate",
      ...sourceBody(request.source),
      linear_deflection_mm: boundedNumber(
        request.linearDeflectionMM,
        DEFAULT_LINEAR_DEFLECTION_MM,
        MIN_LINEAR_DEFLECTION_MM,
        MAX_LINEAR_DEFLECTION_MM,
        "linearDeflectionMM",
      ),
      angular_deflection_rad: boundedNumber(
        request.angularDeflectionRad,
        DEFAULT_ANGULAR_DEFLECTION_RAD,
        MIN_ANGULAR_DEFLECTION_RAD,
        MAX_ANGULAR_DEFLECTION_RAD,
        "angularDeflectionRad",
      ),
      max_triangles: boundedInteger(
        request.maxTriangles,
        DEFAULT_MAX_TRIANGLES,
        MIN_TRIANGLES,
        MAX_TRIANGLES,
        "maxTriangles",
      ),
    };
    return this.post("tessellate", body, normalizeWorkpieceMesh, signal);
  }

  private async post<T>(
    operation: CadOperation,
    body: Record<string, unknown>,
    normalize: (value: unknown) => T,
    parentSignal?: AbortSignal,
  ): Promise<T> {
    const timeoutMS = operation === "tessellate" ? this.meshTimeoutMS : this.timeoutMS;
    const controller = new AbortController();
    let timedOut = false;
    const onParentAbort = () => controller.abort(parentSignal?.reason);
    if (parentSignal?.aborted) {
      onParentAbort();
    } else {
      parentSignal?.addEventListener("abort", onParentAbort, { once: true });
    }
    const timer = setTimeout(() => {
      timedOut = true;
      controller.abort();
    }, timeoutMS);
    try {
      const response = await this.fetch(this.endpoint, {
        method: "POST",
        headers: {
          "Accept": "application/json",
          "Content-Type": "application/json",
        },
        body: JSON.stringify(body),
        cache: "no-store",
        credentials: "same-origin",
        signal: controller.signal,
      });
      const raw = await readJSON(response, operation);
      if (!response.ok) {
        throw httpError(response.status, raw, operation);
      }
      return normalize(raw);
    } catch (error) {
      if (error instanceof CadClientError) throw error;
      if (controller.signal.aborted) {
        throw clientError(
          timedOut ? "timeout" : "aborted",
          timedOut ? `CAD request timed out after ${timeoutMS} ms` : "CAD request was canceled",
          operation,
          null,
          timedOut,
          error,
        );
      }
      throw clientError("network", message(error, "CAD request failed"), operation, null, true, error);
    } finally {
      clearTimeout(timer);
      parentSignal?.removeEventListener("abort", onParentAbort);
    }
  }
}

export function idleCadState<T>(): CadRequestState<T> {
  return { status: "idle", data: null, error: null };
}

export function loadingCadState<T>(data: T | null = null): CadRequestState<T> {
  return { status: "loading", data, error: null };
}

export function readyCadState<T>(data: T): CadRequestState<T> {
  return { status: "ready", data, error: null };
}

export function failedCadState<T>(error: unknown, data: T | null = null): CadRequestState<T> {
  return {
    status: "error",
    data,
    error: error instanceof CadClientError
      ? error
      : clientError("worker_failed", message(error, "CAD request failed"), null, null, false, error),
  };
}

function sourceBody(source: CadSTEPSource): Record<string, unknown> {
  if (source.kind === "fixture") {
    const fixtureId = source.fixtureId.trim();
    if (!fixtureId) throw clientError("invalid_request", "fixtureId is required", null);
    return {
      fixture_id: fixtureId,
      filename: source.filename?.trim() || undefined,
    };
  }
  const filename = source.filename.trim();
  if (!filename || !/\.st(e)?p$/i.test(filename)) {
    throw clientError("invalid_request", "filename must end in .step or .stp", null);
  }
  if (source.bytes.byteLength === 0) {
    throw clientError("invalid_request", "STEP payload is empty", null);
  }
  if (source.bytes.byteLength > MAX_STEP_BYTES) {
    throw clientError("payload_too_large", `STEP payload exceeds the ${MAX_STEP_BYTES} byte limit`, null);
  }
  return {
    filename,
    step_base64: encodeBase64(source.bytes),
  };
}

function encodeBase64(bytes: Uint8Array): string {
  const alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  let result = "";
  for (let index = 0; index < bytes.length; index += 3) {
    const first = bytes[index]!;
    const hasSecond = index + 1 < bytes.length;
    const hasThird = index + 2 < bytes.length;
    const second = hasSecond ? bytes[index + 1]! : 0;
    const third = hasThird ? bytes[index + 2]! : 0;
    const packed = (first << 16) | (second << 8) | third;
    result += alphabet[(packed >> 18) & 63]!;
    result += alphabet[(packed >> 12) & 63]!;
    result += hasSecond ? alphabet[(packed >> 6) & 63]! : "=";
    result += hasThird ? alphabet[packed & 63]! : "=";
  }
  return result;
}

function boundedInteger(
  value: number | undefined,
  fallback: number,
  minimum: number,
  maximum: number,
  name: string,
): number {
  const result = value ?? fallback;
  if (!Number.isInteger(result) || result < minimum || result > maximum) {
    throw clientError("invalid_request", `${name} must be an integer from ${minimum} to ${maximum}`, null);
  }
  return result;
}

function boundedNumber(
  value: number | undefined,
  fallback: number,
  minimum: number,
  maximum: number,
  name: string,
): number {
  const result = value ?? fallback;
  if (!Number.isFinite(result) || result < minimum || result > maximum) {
    throw clientError("invalid_request", `${name} must be a number from ${minimum} to ${maximum}`, null);
  }
  return result;
}

function validateWindow(start: number, end: number, name = "window"): void {
  if (!Number.isFinite(start) || !Number.isFinite(end) || start < 0 || start > 1 || end < 0 || end > 1) {
    throw clientError("invalid_request", `${name} must be within [0, 1]`, null);
  }
  if (start === end) {
    throw clientError("invalid_request", `${name} must have non-zero length`, null);
  }
}

async function readJSON(response: Response, operation: CadOperation): Promise<unknown> {
  try {
    return await response.json() as unknown;
  } catch (error) {
    throw clientError(
      "invalid_response",
      `CAD service returned non-JSON HTTP ${response.status}`,
      operation,
      response.status,
      false,
      error,
    );
  }
}

function httpError(status: number, value: unknown, operation: CadOperation): CadClientError {
  const response = typeof value === "object" && value !== null
    ? value as Record<string, unknown>
    : {};
  const serverCode = typeof response.code === "string" ? response.code : "";
  const detail = typeof response.error === "string"
    ? response.error
    : typeof response.message === "string"
      ? response.message
      : `CAD request failed with HTTP ${status}`;
  let code: CadErrorCode;
  switch (status) {
    case 400:
    case 422:
      code = detail.toLowerCase().includes("timed out") ? "timeout" : "invalid_request";
      break;
    case 413:
      code = "payload_too_large";
      break;
    case 429:
      code = "busy";
      break;
    case 503:
      code = "unavailable";
      break;
    case 504:
      code = "timeout";
      break;
    default:
      code = isErrorCode(serverCode) ? serverCode : "worker_failed";
  }
  return clientError(
    code,
    detail,
    operation,
    status,
    code === "busy" || code === "timeout" || code === "unavailable" || status >= 500,
  );
}

function isErrorCode(value: string): value is CadErrorCode {
  return [
    "aborted",
    "invalid_request",
    "payload_too_large",
    "busy",
    "timeout",
    "unavailable",
    "worker_failed",
    "network",
    "invalid_response",
  ].includes(value);
}

function clientError(
  code: CadErrorCode,
  detail: string,
  operation: CadOperation | null,
  status: number | null = null,
  retryable = false,
  cause?: unknown,
): CadClientError {
  return new CadClientError({
    code,
    message: detail,
    operation,
    status,
    retryable,
    cause,
  });
}

function message(error: unknown, fallback: string): string {
  return error instanceof Error && error.message ? error.message : fallback;
}
