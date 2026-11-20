import { CapabilitiesDigest } from "../../../../../rt-core/clients/ts/rt_control_api_generated.ts";
import type { DenseCellInput } from "../execution/denseClient.ts";

/**
 * Cells other OLP servers advertise on this server's LAN
 * (GET /api/offline-programming/v1/discovered-cells, internal/discovery).
 *
 * Everything a row says about the remote server and its cell is advertised,
 * so unverified. The only facts that are this server's own are under `here`
 * (its catalogue and the repository manifests). The Add form is filled from
 * `here` only: a discovered network address is shown, never dialled, and Add
 * still goes through the commissioned-only registry and a mutual-TLS
 * Describe on Connect. Nothing here connects, selects or arms.
 */

export const DISCOVERED_CELLS_PATH = "/api/offline-programming/v1/discovered-cells";
const SCHEMA = "offline-programming.discovered-cells.v1";

export interface DiscoveredHere {
  configured: boolean;
  pin: "match" | "differs" | "unknown";
  known_manifest: boolean;
  commissioned_address?: string;
  commissioned_host?: string;
  reason?: string;
}

export interface DiscoveredCell {
  cell_id: string;
  label: string;
  model: string;
  mode: "simulation" | "real";
  configuration_sha256?: string;
  capabilities_digest?: string;
  selected?: boolean;
  here: DiscoveredHere;
}

export interface DiscoveredServer {
  instance: string;
  hostname: string;
  address: string;
  port: number;
  reach: "loopback" | "network" | "unknown";
  build_id: string;
  started_at: string;
  maintenance: boolean;
  truncated?: boolean;
  dropped_cells?: number;
  source: "mdns" | "tailscale";
  health: "ok" | "unhealthy" | "unreachable" | "not_probed";
  health_detail?: string;
  cells: DiscoveredCell[];
}

export interface DiscoverySource {
  name: string;
  interfaces?: string[];
  found: number;
  error?: string;
}

export interface DiscoveryResult {
  schema: typeof SCHEMA;
  enabled: boolean;
  reason?: string;
  searched_at?: string;
  sources: DiscoverySource[];
  servers: DiscoveredServer[];
}

export class DiscoveryError extends Error {
  constructor(message: string) { super(message); this.name = "DiscoveryError"; }
}

const DIGEST = /^[0-9a-f]{64}$/;
const isObject = (v: unknown): v is Record<string, unknown> => typeof v === "object" && v !== null && !Array.isArray(v);
const isString = (v: unknown): v is string => typeof v === "string";
const optional = (v: unknown, check: (x: unknown) => boolean): boolean => v === undefined || check(v);
const isDigest = (v: unknown): boolean => v === "" || (isString(v) && DIGEST.test(v));

/** Checks the route's body field by field and names the first bad one. */
export function validateDiscoveryResult(body: unknown): DiscoveryResult {
  const fail = (field: string): never => { throw new DiscoveryError(`Invalid discovered-cells answer: ${field}`); };
  if (!isObject(body) || body.schema !== SCHEMA) return fail("schema");
  if (typeof body.enabled !== "boolean") return fail("enabled");
  if (!optional(body.reason, isString) || !optional(body.searched_at, isString)) return fail("reason");
  if (!Array.isArray(body.sources) || !Array.isArray(body.servers)) return fail("sources/servers");
  body.sources.forEach((s, i) => {
    if (!isObject(s) || !isString(s.name) || typeof s.found !== "number" || !optional(s.error, isString)
      || !optional(s.interfaces, (v) => Array.isArray(v) && v.every(isString))) fail(`sources[${i}]`);
  });
  body.servers.forEach((s, i) => {
    const at = `servers[${i}]`;
    if (!isObject(s)) return fail(at);
    for (const key of ["instance", "hostname", "address", "build_id", "started_at"] as const) if (!isString(s[key])) fail(`${at}.${key}`);
    if (typeof s.port !== "number" || typeof s.maintenance !== "boolean") fail(`${at}.port`);
    if (!["loopback", "network", "unknown"].includes(s.reach as string)) fail(`${at}.reach`);
    if (!["mdns", "tailscale"].includes(s.source as string)) fail(`${at}.source`);
    if (!["ok", "unhealthy", "unreachable", "not_probed"].includes(s.health as string)) fail(`${at}.health`);
    if (!Array.isArray(s.cells)) return fail(`${at}.cells`);
    s.cells.forEach((c, j) => {
      const cellAt = `${at}.cells[${j}]`;
      if (!isObject(c) || !isString(c.cell_id) || !isString(c.label) || !isString(c.model)) return fail(cellAt);
      if (c.mode !== "simulation" && c.mode !== "real") fail(`${cellAt}.mode`);
      if (!optional(c.configuration_sha256, isDigest) || !optional(c.capabilities_digest, isDigest)) fail(`${cellAt}.digest`);
      const here = c.here;
      if (!isObject(here) || typeof here.configured !== "boolean" || typeof here.known_manifest !== "boolean"
        || !["match", "differs", "unknown"].includes(here.pin as string)
        || !optional(here.commissioned_address, isString) || !optional(here.commissioned_host, isString)
        || !optional(here.reason, isString)) fail(`${cellAt}.here`);
    });
  });
  return body as unknown as DiscoveryResult;
}

export class DiscoveredCellsClient {
  readonly #fetch: typeof fetch;
  constructor(fetcher: typeof fetch = globalThis.fetch.bind(globalThis)) { this.#fetch = fetcher; }

  /** `refresh` asks the server for a new search (it rate-limits to one per 2 s). */
  async search(refresh: boolean, signal?: AbortSignal): Promise<DiscoveryResult> {
    const response = await this.#fetch(`${DISCOVERED_CELLS_PATH}${refresh ? "?refresh=1" : ""}`, {
      headers: { Accept: "application/json" }, redirect: "error", signal,
    });
    if (response.status === 404) {
      throw new DiscoveryError("This OLP server has no network search: it runs a build from before discovered-cells (install an OLP built from this commit)");
    }
    let body: unknown;
    try { body = await response.json(); } catch {
      throw new DiscoveryError(`OLP API unreachable: HTTP ${response.status} with no JSON body`);
    }
    if (!response.ok) throw new DiscoveryError(`Network search failed: HTTP ${response.status}`);
    return validateDiscoveryResult(body);
  }
}

export type BadgeKind = "ok" | "warn" | "bad" | "info";
export interface DiscoveryBadge { kind: BadgeKind; text: string; title: string }

/** One discovered cell, as the dialog and the menu show it. */
export interface DiscoveredRow {
  key: string;
  cellId: string;
  /** The advertising server's host name (or address). */
  where: string;
  title: string;
  subtitle: string;
  badges: DiscoveryBadge[];
  compatibility: "compatible" | "incompatible" | "unknown";
  configured: boolean;
  /** Add may be offered: a commissioned binding for this id is known here. */
  addable: boolean;
  addReason: string;
  /** What the Add form is filled with: this server's commissioning, never the network. */
  prefill: DenseCellInput | null;
}

const short = (digest: string | undefined): string => digest ? digest.slice(0, 8) : "—";

/** Compatibility is this UI's generated rt-control client against the
 *  digest the cell gave its own server in an authenticated Describe; the
 *  same comparison Connect makes, here only as a warning. */
export function discoveredRows(result: DiscoveryResult, uiDigest: string = CapabilitiesDigest): DiscoveredRow[] {
  const rows: DiscoveredRow[] = [];
  for (const server of result.servers) {
    const where = server.hostname || server.address;
    for (const cell of server.cells) {
      const badges: DiscoveryBadge[] = [{
        kind: "info", text: "Unverified",
        title: "Advertised by another OLP server on the network; nothing here is verified until Connect describes the cell over mutual TLS",
      }];
      let compatibility: DiscoveredRow["compatibility"] = "unknown";
      if (!cell.capabilities_digest) {
        badges.push({ kind: "info", text: "Compatibility unknown", title: `${where} has not described this cell yet (it is not selected there)` });
      } else if (cell.capabilities_digest === uiDigest) {
        compatibility = "compatible";
        badges.push({ kind: "ok", text: "Compatible", title: `rt-control capabilities ${short(cell.capabilities_digest)} match this UI's client` });
      } else {
        compatibility = "incompatible";
        badges.push({ kind: "bad", text: "Incompatible", title: `The cell runs rt-control capabilities ${short(cell.capabilities_digest)}; this UI's client is ${short(uiDigest)}. Connect would be refused until both run one commit.` });
      }
      const here = cell.here;
      if (here.configured) badges.push({ kind: "ok", text: "Configured here", title: "Already in this server's cell list: connect to it under Connect to" });
      else if (!here.known_manifest) badges.push({ kind: "bad", text: "Unknown cell id", title: here.reason ?? "No manifest for this cell id in rt-core/config/cells" });
      else badges.push({ kind: "warn", text: "Not configured here", title: here.reason ?? "Not in this server's cell catalogue" });
      if (here.pin === "match") badges.push({ kind: "ok", text: "Pin matches", title: `Configuration pin ${short(cell.configuration_sha256)} equals this server's ${here.configured ? "catalogue" : "manifest"} pin` });
      else if (here.pin === "differs") badges.push({ kind: "bad", text: "Pin differs", title: `Advertised configuration pin ${short(cell.configuration_sha256)} differs from this server's ${here.configured ? "catalogue" : "manifest"} pin: the two servers run different configurations` });
      if (server.maintenance) badges.push({ kind: "warn", text: "Maintenance", title: `${where} has a maintenance notice up` });
      if (cell.selected) badges.push({ kind: "info", text: `In use on ${where}`, title: `${where} has this cell selected` });

      const health = server.health === "ok" ? "API ok"
        : server.health === "not_probed" ? "via its tunnel only"
          : server.health === "unhealthy" ? "API unhealthy" : "API unreachable";
      const subtitle = [
        cell.model, cell.mode,
        `${where}${server.address && server.address !== where ? ` (${server.address})` : ""}`,
        server.build_id ? `OLP ${server.build_id}` : "",
        health,
        server.source === "tailscale" ? "Tailscale" : "",
      ].filter(Boolean).join(" · ");

      let addable = false;
      let addReason: string;
      let prefill: DenseCellInput | null = null;
      if (here.configured) {
        addReason = "Already configured on this server";
      } else if (!here.known_manifest || !here.commissioned_address || !here.commissioned_host) {
        addReason = here.reason ?? "No commissioned binding for this cell id is known here";
      } else {
        addable = true;
        addReason = here.reason ?? "";
        prefill = { id: cell.cell_id, host: here.commissioned_host, address: here.commissioned_address, credentialRef: cell.cell_id };
      }
      rows.push({
        key: `${server.instance}/${cell.cell_id}`,
        cellId: cell.cell_id,
        where,
        title: cell.label || cell.cell_id,
        subtitle,
        badges,
        compatibility,
        configured: here.configured,
        addable,
        addReason,
        prefill,
      });
    }
  }
  return rows;
}

/** One line for the dialog section and the menu. */
export function discoveryStatusText(state: { result: DiscoveryResult | null; error: string | null; searching: boolean }): string {
  if (state.error) return state.error;
  if (!state.result) return state.searching ? "Searching the local network…" : "Not searched yet";
  if (!state.result.enabled) return state.result.reason ?? "Network search is off on this server";
  const where = state.result.sources.map((s) => s.name === "mdns" ? `mDNS on ${(s.interfaces ?? []).join(", ")}` : "Tailscale peers").join(" and ");
  const failed = state.result.sources.filter((s) => s.error).map((s) => `${s.name}: ${s.error}`);
  const servers = state.result.servers.length;
  const cells = state.result.servers.reduce((n, s) => n + s.cells.length, 0);
  const found = servers === 0 ? "No OLP servers found" : `${cells} cell${cells === 1 ? "" : "s"} on ${servers} OLP server${servers === 1 ? "" : "s"}`;
  const when = state.result.searched_at ? ` · searched ${new Date(state.result.searched_at).toLocaleTimeString()}` : "";
  return `${state.searching ? "Searching… " : ""}${found} via ${where}${when}${failed.length ? ` · ${failed.join("; ")}` : ""}`;
}

/**
 * The last search and its live refresh. Polling runs only while someone is
 * looking (the Cells dialog open); the server caches a search for 10 s, so a
 * 5 s poll costs one LAN query about every 10 s.
 */
export class DiscoveryModel {
  readonly #client: DiscoveredCellsClient;
  readonly #listeners = new Set<() => void>();
  #result: DiscoveryResult | null = null;
  #error: string | null = null;
  #searching = false;
  #timer: ReturnType<typeof setInterval> | null = null;
  #inflight: Promise<void> | null = null;

  constructor(client: DiscoveredCellsClient = new DiscoveredCellsClient()) { this.#client = client; }

  get result(): DiscoveryResult | null { return this.#result; }
  get error(): string | null { return this.#error; }
  get searching(): boolean { return this.#searching; }
  rows(): DiscoveredRow[] { return this.#result ? discoveredRows(this.#result) : []; }
  statusText(): string { return discoveryStatusText({ result: this.#result, error: this.#error, searching: this.#searching }); }

  onChange(listener: () => void): () => void {
    this.#listeners.add(listener);
    return () => this.#listeners.delete(listener);
  }

  refresh(force = false): Promise<void> {
    if (this.#inflight) return this.#inflight;
    this.#searching = true;
    this.#emit();
    this.#inflight = this.#client.search(force).then(
      (result) => { this.#result = result; this.#error = null; },
      (error: unknown) => { this.#error = error instanceof Error ? error.message : String(error); },
    ).finally(() => {
      this.#searching = false;
      this.#inflight = null;
      this.#emit();
    });
    return this.#inflight;
  }

  startPolling(intervalMs = 5000): void {
    if (this.#timer !== null) return;
    void this.refresh(false);
    this.#timer = setInterval(() => void this.refresh(false), intervalMs);
  }

  stopPolling(): void {
    if (this.#timer !== null) clearInterval(this.#timer);
    this.#timer = null;
  }

  #emit(): void { for (const listener of this.#listeners) listener(); }
}
