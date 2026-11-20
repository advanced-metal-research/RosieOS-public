/**
 * Who is connected, as the maintenance overlay lists it (browser and Steam
 * Deck alike), from the OLP server's GET .../maintenance `clients` and
 * `machine`. The server builds the list only from what requests already carry
 * (address, Origin/Referer, User-Agent, the Deck's X-Rosie-Client label), so
 * the words here never claim more: a loopback address is "this host or a
 * tunnel", an unnamed client stays unknown. The cell's rt-control adds who
 * holds its grant (`cell.holder`: the acquire name and principal, never a
 * session), which can be a client of another OLP server.
 *
 * Bundled for Qt 5.15's engine (ES2016): no padStart, no BigInt.
 */

export interface MaintenanceSessionLine {
  /** "browser", "pendant", "tool", "unknown", "cell" for the machine line, or
   *  "grant" for the cell rt-control's grant holder. */
  kind: string;
  text: string;
  /** Holds the execution session or the machine is armed: the one to wait for. */
  holds: boolean;
}

function record(value: unknown): Record<string, unknown> {
  return value && typeof value === "object" && !Array.isArray(value) ? value as Record<string, unknown> : {};
}

function str(value: unknown): string {
  return typeof value === "string" ? value.trim() : "";
}

/** "4 s ago", "3 min ago"; empty when the time is unreadable. */
export function maintenanceAgo(iso: unknown, nowMs: number): string {
  const ms = typeof iso === "string" ? Date.parse(iso) : Number.NaN;
  if (!Number.isFinite(ms)) return "";
  const seconds = Math.max(0, Math.round((nowMs - ms) / 1000));
  if (seconds < 90) return `${seconds} s ago`;
  const minutes = Math.round(seconds / 60);
  return minutes < 90 ? `${minutes} min ago` : `${Math.round(minutes / 60)} h ago`;
}

/**
 * The machine first (armed, lease, session), then the cell's grant holder,
 * then every client, most recent first. `nowMs` is the server's clock as the
 * client estimates it.
 */
export function maintenanceSessionLines(payload: unknown, nowMs: number): MaintenanceSessionLine[] {
  const body = record(payload);
  const lines: MaintenanceSessionLine[] = [];
  const machine = record(body.machine);
  const cell = str(machine.label) || str(machine.cell);
  if (cell) {
    const armed = machine.armed === true;
    const parts = [`Cell ${cell}`, armed ? "ARMED" : machine.armed === false ? "disarmed" : "arm state unknown"];
    if (machine.lease_valid === true) parts.push("control lease held");
    const session = str(machine.session_state);
    if (session) parts.push(`program ${session}`);
    lines.push({ kind: "cell", text: parts.join(" · "), holds: armed || machine.lease_valid === true });
  }
  const holder = record(record(body.cell).holder);
  const controller = str(holder.controller);
  if (controller) {
    const parts = [`Servo host grant held by ${controller}`];
    const identity = str(holder.identity);
    parts.push(holder.remote === true ? `remote${identity ? ` ${identity}` : ""}` : "local socket");
    const since = maintenanceAgo(holder.since, nowMs);
    if (since) parts.push(`since ${since}`);
    lines.push({ kind: "grant", text: parts.join(" · "), holds: true });
  }
  const clients = Array.isArray(body.clients) ? body.clients : [];
  for (const entry of clients) {
    const client = record(entry);
    const parts = [str(client.label) || "Unknown client"];
    const address = str(client.address);
    if (address) parts.push(client.loopback === true ? `from ${address} (this host or a tunnel)` : `from ${address}`);
    const seen = maintenanceAgo(client.last_seen, nowMs);
    if (seen) parts.push(`seen ${seen}`);
    const action = str(client.last_action);
    if (action) {
      const at = maintenanceAgo(client.last_action_at, nowMs);
      parts.push(`last robot request ${action}${at ? ` ${at}` : ""}`);
    }
    const holds = client.session_holder === true;
    if (holds) parts.push("holds the execution session");
    lines.push({ kind: str(client.kind) || "unknown", text: parts.join(" · "), holds });
  }
  return lines;
}
