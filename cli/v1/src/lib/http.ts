// HTTP to the RosieOS services. OLP and the planner are plain HTTP on
// loopback; rt-control is HTTP over its public Unix socket or over the
// mutual-TLS listener (rt-core/docs/APPLICATION_API.md, "Transport and
// response semantics"). Nothing here retries: a robot service's refusal is
// an answer, not a transient fault.
import { readFileSync } from "node:fs";
import http from "node:http";
import https from "node:https";

import type { RtControlConfig } from "./config.js";

export interface HttpReply {
  status: number;
  ok: boolean;
  /** Parsed JSON when the body was JSON, otherwise the text. */
  body: unknown;
  contentType: string;
}

export class ServiceUnreachable extends Error {
  constructor(
    readonly service: string,
    readonly target: string,
    cause: unknown,
  ) {
    super(`${service} at ${target} is unreachable: ${cause instanceof Error ? cause.message : String(cause)}`);
  }
}

function parseBody(raw: string, contentType: string): unknown {
  if (contentType.includes("json") || /^\s*[[{]/.test(raw)) {
    try {
      return JSON.parse(raw) as unknown;
    } catch {
      return raw;
    }
  }
  return raw;
}

export interface RequestOptions {
  method?: "GET" | "POST" | "PUT" | "DELETE";
  body?: unknown;
  rawBody?: Uint8Array;
  headers?: Record<string, string>;
  timeoutMs: number;
}

export async function fetchJSON(service: string, url: string, options: RequestOptions): Promise<HttpReply> {
  const headers: Record<string, string> = { Accept: "application/json", ...options.headers };
  let body: RequestInit["body"];
  if (options.rawBody) {
    body = options.rawBody;
    headers["Content-Type"] ??= "application/octet-stream";
  } else if (options.body !== undefined) {
    body = JSON.stringify(options.body);
    headers["Content-Type"] = "application/json";
  }
  let response: Response;
  try {
    response = await fetch(url, {
      method: options.method ?? (body === undefined ? "GET" : "POST"),
      headers,
      body,
      redirect: "error",
      signal: AbortSignal.timeout(options.timeoutMs),
    });
  } catch (error) {
    throw new ServiceUnreachable(service, url, error);
  }
  const contentType = response.headers.get("content-type") ?? "";
  const raw = await response.text();
  return { status: response.status, ok: response.ok, body: parseBody(raw, contentType), contentType };
}

/** {"error", "detail"} is the refusal shape OLP and rt-control both answer with. */
export function refusalText(reply: HttpReply): string {
  const body = reply.body;
  if (body && typeof body === "object") {
    const record = body as Record<string, unknown>;
    const code = record.error ?? record.code ?? record.reason;
    const detail = record.detail ?? record.message;
    if (code || detail) return [code, detail].filter(Boolean).join(": ");
  }
  return typeof body === "string" && body.trim() ? body.trim().slice(0, 500) : `HTTP ${reply.status}`;
}

const RESPONSE_SCHEMA = "rosie.rt-control.response.v1";

export class RtControlClient {
  readonly target: string;
  private readonly agent: http.Agent | https.Agent | undefined;

  constructor(
    private readonly config: RtControlConfig,
    private readonly timeoutMs: number,
  ) {
    if (config.url) {
      this.target = config.url;
      this.agent = new https.Agent({
        ca: config.caFile ? readFileSync(config.caFile) : undefined,
        cert: config.certFile ? readFileSync(config.certFile) : undefined,
        key: config.keyFile ? readFileSync(config.keyFile) : undefined,
        keepAlive: false,
      });
    } else if (config.socketPath) {
      this.target = `unix:${config.socketPath}`;
    } else {
      this.target = "";
    }
  }

  get configured(): boolean {
    return this.target !== "";
  }

  /**
   * GET one observation route. Ordinary replies are the Response envelope
   * (`rosie.rt-control.response.v1`, result under `data`); the envelope is
   * unwrapped so callers see the named result type.
   */
  async get(path: string): Promise<HttpReply> {
    if (!this.configured) {
      throw new Error(
        "rt-control is not configured: set ROSIE_RT_CONTROL_SOCKET (local public control socket) or ROSIE_RT_CONTROL_URL with ROSIE_RT_CONTROL_CA/CERT/KEY (mutual TLS)",
      );
    }
    const reply = await this.request(path);
    const body = reply.body;
    if (body && typeof body === "object" && (body as Record<string, unknown>).schema === RESPONSE_SCHEMA && "data" in body) {
      return { ...reply, body: (body as Record<string, unknown>).data };
    }
    return reply;
  }

  private request(path: string): Promise<HttpReply> {
    const options: http.RequestOptions = {
      method: "GET",
      path,
      headers: { Accept: "application/json" },
      timeout: this.timeoutMs,
    };
    let transport: typeof http | typeof https = http;
    if (this.config.url) {
      const url = new URL(this.config.url);
      Object.assign(options, { protocol: url.protocol, hostname: url.hostname, port: url.port, agent: this.agent });
      transport = url.protocol === "https:" ? https : http;
    } else {
      options.socketPath = this.config.socketPath;
    }
    return new Promise((resolve, reject) => {
      const request = transport.request(options, (response) => {
        const chunks: Buffer[] = [];
        response.on("data", (chunk: Buffer) => chunks.push(chunk));
        response.on("end", () => {
          const contentType = String(response.headers["content-type"] ?? "");
          const status = response.statusCode ?? 0;
          resolve({
            status,
            ok: status >= 200 && status < 300,
            body: parseBody(Buffer.concat(chunks).toString("utf8"), contentType),
            contentType,
          });
        });
        response.on("error", (error) => reject(new ServiceUnreachable("rt-control", this.target, error)));
      });
      request.on("timeout", () => request.destroy(new Error(`no answer within ${this.timeoutMs} ms`)));
      request.on("error", (error) => reject(new ServiceUnreachable("rt-control", this.target, error)));
      request.end();
    });
  }
}
