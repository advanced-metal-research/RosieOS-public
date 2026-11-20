/**
 * The cell's switched outputs as both apps show them: for every output the
 * cell describes, what the program asks for (intent), what rt-core writes to
 * the terminal (commanded) and what the terminal's wired feedback reads back
 * (readback), with the time left on an ON before it expires on its own.
 *
 * Read from the dense status body as sent: rt-core's cyclic `status.core.io`
 * (rt-core docs/CELL_IO.md) beside Describe's `io` policy (class, expiry,
 * `readback: "none_unverified"` for an output with no wired feedback) and the
 * loaded session's `process_outputs`. Intent, command and readback are three
 * separate facts and stay three columns: a command is never shown as proof
 * that the wire switched, and an unwired output never reads OFF.
 *
 * Display only. Nothing here decides whether an output may switch; rt-core
 * gates every output on permit, grant and its own expiry.
 */
import { operatorRejection } from "../targets/operatorFeedback.ts";

export interface CellOutput {
  name: string;
  /** "process" or "torch", from Describe; "" when the description has none. */
  outputClass: string;
  intent: boolean;
  commanded: boolean;
  /** The wired feedback, or null when the output has none or it is invalid. */
  readback: boolean | null;
  /** Describe says this output has no wired feedback (owner decision 2026-10-12). */
  unwired: boolean;
  /** Milliseconds until an ON expires on its own, at the observation; null when OFF. */
  expiresInMs: number | null;
}

export interface CellOutputs {
  armed: boolean;
  /** rt-core's I/O reason as its public code, "" for none. */
  reason: string;
  outputs: CellOutput[];
  /** What the loaded production program switches (session.process_outputs). */
  programOutputs: string[];
  /** A loaded dry run: the program's outputs stay off. */
  dryRun: boolean;
}

export interface CellOutputRow {
  name: string;
  state: "on" | "off";
  text: string;
  tone: "idle" | "on" | "fault";
}

export interface CellOutputsView {
  visible: boolean;
  tone: "idle" | "armed" | "on" | "fault";
  headline: string;
  rows: CellOutputRow[];
  /** The headline and every row, one line each, for the Deck's label and the checks. */
  lines: string[];
}

// rt-core's native I/O reasons (protocol reason numbers, ipcclient
// ReasonIoNotConfigured..ReasonIoExchangeLost and the grant reasons an I/O
// state can carry), as the public codes the reason labels are keyed by.
const IO_REASON_CODES: Record<number, string> = {
  1: "no_grant", 2: "wrong_generation", 3: "inhibited",
  12: "io_not_configured", 13: "io_not_armed", 14: "io_fast_input_unsatisfied",
  15: "io_readback_disagreement", 16: "io_torch_unqualified", 17: "io_marker_late", 18: "io_exchange_lost", 19: "io_wait_timeout", 20: "io_wait_input_invalid",
};

const isRecord = (value: unknown): value is Record<string, unknown> => typeof value === "object" && value !== null && !Array.isArray(value);

/** The outputs from one dense status body; null when the cell has none. */
export function normalizeCellOutputs(body: unknown): CellOutputs | null {
  const record = isRecord(body) ? body : {};
  const observation = isRecord(record.rt_core) ? record.rt_core : {};
  const status = isRecord(observation.status) ? observation.status : {};
  const core = isRecord(status.core) ? status.core : {};
  const io = isRecord(core.io) ? core.io : null;
  const description = isRecord(observation.description) && isRecord(observation.description.io) ? observation.description.io : {};
  const described = new Map<string, Record<string, unknown>>();
  for (const output of Array.isArray(description.outputs) ? description.outputs : []) {
    if (isRecord(output) && typeof output.name === "string") described.set(output.name, output);
  }
  const live = io && Array.isArray(io.outputs) ? io.outputs.filter(isRecord) : [];
  if (!live.length && !described.size) return null;
  const timeNs = typeof io?.time_ns === "number" ? io.time_ns : 0;
  const names = live.length ? live.map((o) => String(o.name ?? "")) : [...described.keys()];
  const outputs = names.map((name): CellOutput => {
    const o = live.find((candidate) => candidate.name === name) ?? {};
    const policy = described.get(name) ?? {};
    const commanded = o.commanded === true;
    const expiry = typeof o.expiry_ns === "number" ? o.expiry_ns : 0;
    return {
      name,
      outputClass: typeof policy.class === "string" ? policy.class : o.torch === true ? "torch" : "",
      intent: o.intent === true,
      commanded,
      readback: o.valid === true ? o.readback === true : null,
      unwired: policy.readback === "none_unverified",
      expiresInMs: commanded && expiry > timeNs && timeNs > 0 ? Math.round((expiry - timeNs) / 1e6) : null,
    };
  });
  const session = isRecord(record.session) ? record.session : {};
  const reason = typeof io?.reason === "number" ? IO_REASON_CODES[io.reason] ?? `io_reason_${io.reason}` : "";
  return {
    armed: io?.armed === true,
    reason: io?.reason === 0 ? "" : reason,
    outputs,
    programOutputs: Array.isArray(session.process_outputs) ? session.process_outputs.filter((n): n is string => typeof n === "string") : [],
    dryRun: session.dry_run === true && session.kind === "trajectory",
  };
}

function rowOf(o: CellOutput): CellOutputRow {
  const label = o.outputClass === "torch" ? `${o.name} (torch)` : o.name;
  const readback = o.unwired ? "no wired readback"
    : o.readback === null ? "readback unknown" : `readback ${o.readback ? "ON" : "off"}`;
  const disagrees = o.readback !== null && o.readback !== o.commanded;
  const parts = [o.commanded ? "ON" : "off"];
  if (o.intent !== o.commanded) parts.push(`program asks ${o.intent ? "ON" : "off"}`);
  parts.push(readback);
  if (o.expiresInMs !== null) parts.push(`expires in ${o.expiresInMs} ms unless refreshed`);
  return {
    name: o.name,
    state: o.commanded ? "on" : "off",
    text: `${label}: ${parts.join(" · ")}`,
    tone: disagrees ? "fault" : o.commanded ? "on" : "idle",
  };
}

/** What the operator sees about the outputs; hidden for a cell with none. */
export function cellOutputsView(io: CellOutputs | null): CellOutputsView {
  if (!io) return { visible: false, tone: "idle", headline: "", rows: [], lines: [] };
  const rows = io.outputs.map(rowOf);
  let headline = io.armed ? "Outputs armed" : "Outputs disarmed";
  if (io.reason) headline += `: ${operatorRejection(io.reason).text}`;
  if (io.programOutputs.length) headline += ` · this program switches ${io.programOutputs.join(", ")}`;
  else if (io.dryRun) headline += " · dry run: the program's outputs stay off";
  const tone = io.reason && io.reason !== "io_not_armed" || rows.some((row) => row.tone === "fault") ? "fault"
    : rows.some((row) => row.state === "on") ? "on" : io.armed ? "armed" : "idle";
  return { visible: true, tone, headline, rows, lines: [headline, ...rows.map((row) => row.text)] };
}
