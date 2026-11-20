// The OLP API client. OLP is the controller of record for a cell: it holds
// rt-control's one grant, runs the 5 s heartbeat deadman, and applies the
// maintenance, control and focus gates (offline-programming/v1/README.md
// "Maintenance notice", "Another controller"). This server acts on the robot
// only through it; a second rt-control grant beside OLP would sit where OLP's
// STOP cannot reach (README "Test programs").
import { randomBytes } from "node:crypto";

import { fetchJSON, refusalText, type HttpReply } from "./http.js";

export const OLP_API = "/api/offline-programming/v1";
export const DENSE = `${OLP_API}/dense-execution`;

/** The selected target as GET dense-execution/status reports it (denseexec.TargetIdentity). */
export interface TargetIdentity {
  selection_generation: string;
  cell_id: string;
  model_id?: string;
  label?: string;
  backend?: string;
  simulation?: boolean;
  host?: string;
}

export interface SessionInfo {
  id: string;
  kind: string;
  state: string;
  dry_run?: boolean;
  trajectory_digest?: string;
  plan_id?: string;
  lease?: { held: boolean; expires_in_ms: number };
  heartbeat_age_ms?: number;
  heartbeat_deadline_ms?: number;
  stop_reason?: string;
  last_error?: string;
  source?: string;
}

/** GET dense-execution/status (denseexec.StatusBody); fields this server reads. */
export interface DenseStatus {
  available: boolean;
  backend?: string;
  target?: TargetIdentity | null;
  armed?: boolean | null;
  session?: SessionInfo | null;
  cursor?: Record<string, unknown> | null;
  axes?: unknown;
  robot_cell?: Record<string, unknown> | null;
  last_jog_end?: Record<string, unknown> | null;
  loop?: Record<string, unknown> | null;
  daemon_error?: string;
  rt_core?: Record<string, unknown> | null;
}

export class OlpRefusal extends Error {
  constructor(
    readonly action: string,
    readonly reply: HttpReply,
  ) {
    super(`OLP refused ${action} (HTTP ${reply.status}): ${refusalText(reply)}`);
  }
}

export class OlpClient {
  /**
   * One controller identity per server process (OLP control.go
   * validClientID: 8-64 of [A-Za-z0-9-_.:]). OLP journals it on every
   * request, so a robot action from this server is never "Unknown client".
   */
  readonly clientId: string;

  /** `clientId` defaults to a fresh `mcp-<16 hex>`; the CLI passes its persisted one, so a save's idempotency key and OLP's journal name one client across commands. */
  constructor(
    readonly baseUrl: string,
    private readonly label: string,
    private readonly timeoutMs: number,
    clientId?: string,
  ) {
    this.clientId = clientId ?? `mcp-${randomBytes(8).toString("hex")}`;
  }

  private headers(target?: TargetIdentity | null): Record<string, string> {
    const headers: Record<string, string> = { "X-Rosie-Client": this.label, "X-Rosie-Client-Id": this.clientId };
    if (target) {
      // The selection fence (dense_execution.go denseTargetRequests): a
      // request naming a generation other than the live one is refused, so
      // a cell switched by another client after our status read cannot
      // receive this action.
      headers["X-RT-Target-Generation"] = target.selection_generation;
      headers["X-RT-Target-Cell"] = target.cell_id;
    }
    return headers;
  }

  async get(path: string, timeoutMs = this.timeoutMs): Promise<HttpReply> {
    return fetchJSON("OLP", `${this.baseUrl}${path}`, { headers: this.headers(), timeoutMs });
  }

  async post(path: string, body: unknown, options: { target?: TargetIdentity | null; timeoutMs?: number } = {}): Promise<HttpReply> {
    return fetchJSON("OLP", `${this.baseUrl}${path}`, {
      method: "POST",
      body,
      headers: this.headers(options.target),
      timeoutMs: options.timeoutMs ?? this.timeoutMs,
    });
  }

  /** GET and require 2xx; a refusal throws with OLP's own code and sentence. */
  async getOk(path: string, action: string, timeoutMs = this.timeoutMs): Promise<unknown> {
    const reply = await this.get(path, timeoutMs);
    if (!reply.ok) throw new OlpRefusal(action, reply);
    return reply.body;
  }

  async status(timeoutMs = this.timeoutMs): Promise<DenseStatus> {
    return (await this.getOk(`${DENSE}/status`, "status", timeoutMs)) as DenseStatus;
  }
}
