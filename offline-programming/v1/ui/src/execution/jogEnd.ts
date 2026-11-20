/**
 * "Jog stopped": the warning the operator sees when a jog or Cartesian hold
 * ends without them releasing it -- the robot stops, stays armed (or not),
 * and the trigger must be pulled again. The OLP server classifies every hold
 * end (internal/denseexec/jog_end.go) and publishes the last one as
 * `last_jog_end` on the status body every client polls; this decides what a
 * client shows of it, for the browser toast and the Steam Deck overlay alike.
 *
 * The server words it and says whether to pop up (never for a release, nor
 * for a Stop or Disarm the owner pressed itself). Only the client whose
 * press began the hold (its X-Rosie-Client-Id) gets the toast; every other
 * client gets a quiet note. Each end shows once; an end already old when a
 * client first sees it (a reload, a reconnect) is not shown.
 *
 * Bundled for Qt 5.15's engine (ES2016): no padStart, no Object.entries.
 */

/** The toast auto-dismisses after this (owner's ask: ~6 s). Unverified. */
export const JOG_END_TOAST_MS = 6000;
/** A note for a client that did not own the hold: shorter, quieter. */
export const JOG_END_NOTE_MS = 4000;
/** An end older than this when a client first sees it is history, not news:
 *  covers one status poll on a slow link plus a reload. Unverified. */
export const JOG_END_FRESH_MS = 10000;

export interface JogEnd {
  seq: number;
  code: string;
  cause: string;
  text: string;
  detail: string;
  repull: string;
  popup: boolean;
  disarmed: boolean;
  atStart: boolean;
  kind: string;
  axis: string;
  frame: string;
  clientId: string;
  clientLabel: string;
  endedBy: string;
  at: string;
  ageMs: number;
  durationMs: number;
  speedUnit: string;
  requestedSpeed: number;
  achievedSpeed: number;
  distanceUnit: string;
  distance: number;
  governing: string;
  samplesAhead: number | null;
  lastIntentAgeMs: number;
  lifetimeMs: number;
}

export type JogEndTone = "toast" | "note";

export interface JogEndNotice {
  active: boolean;
  tone: JogEndTone;
  seq: number;
  code: string;
  /** JOG STOPPED, JOG STOPPED · DISARMED, JOG COULD NOT START, JOG NOTE. */
  kicker: string;
  /** The server's operator text. */
  text: string;
  /** Technical detail, one fact per line (the details expander), starting
   *  with what pulling the trigger again will do. */
  details: string[];
  expanded: boolean;
  remainingMs: number;
}

const INACTIVE: JogEndNotice = {
  active: false, tone: "toast", seq: 0, code: "", kicker: "", text: "", details: [], expanded: false, remainingMs: 0,
};

function record(value: unknown): Record<string, unknown> {
  return value && typeof value === "object" && !Array.isArray(value) ? value as Record<string, unknown> : {};
}

function str(value: unknown): string {
  return typeof value === "string" ? value.trim() : "";
}

function num(value: unknown): number {
  return typeof value === "number" && isFinite(value) ? value : 0;
}

/** The server's `last_jog_end` (or a status body carrying it), or null. */
export function normalizeJogEnd(value: unknown): JogEnd | null {
  let body = record(value);
  if (body.last_jog_end !== undefined) body = record(body.last_jog_end);
  const code = str(body.code);
  const seq = num(body.seq);
  if (!code || seq <= 0) return null;
  return {
    seq, code, cause: str(body.cause), text: str(body.text) || `Jog stopped (${code}).`, detail: str(body.detail),
    repull: str(body.repull), popup: body.popup === true, disarmed: body.disarmed === true, atStart: body.at_start === true,
    kind: str(body.kind), axis: str(body.axis), frame: str(body.frame), clientId: str(body.client_id),
    clientLabel: str(body.client_label), endedBy: str(body.ended_by), at: str(body.at), ageMs: num(body.age_ms),
    durationMs: num(body.duration_ms), speedUnit: str(body.speed_unit), requestedSpeed: num(body.requested_speed),
    achievedSpeed: num(body.achieved_speed), distanceUnit: str(body.distance_unit), distance: num(body.distance),
    governing: str(body.governing), samplesAhead: typeof body.samples_ahead === "number" ? body.samples_ahead : null,
    lastIntentAgeMs: num(body.last_intent_age_ms), lifetimeMs: num(body.lifetime_ms),
  };
}

const REPULL_WORDS: Record<string, string> = {
  yes: "Pulling the trigger again should work.",
  no: "Pulling the trigger again in the same direction stops again.",
  after_rearm: "The robot is disarmed: arm it before jogging.",
};

function kickerOf(end: JogEnd, tone: JogEndTone): string {
  if (tone === "note") return "JOG NOTE";
  if (end.code === "speed_governed_to_zero") return "JOG STALLED";
  const head = end.atStart ? "JOG COULD NOT START" : "JOG STOPPED";
  return end.disarmed ? `${head} · DISARMED` : head;
}

/** The expander's lines: code and cause, the server's detail, the evidence. */
export function jogEndDetails(end: JogEnd): string[] {
  const lines: string[] = [];
  const repull = REPULL_WORDS[end.repull];
  if (repull) lines.push(repull);
  lines.push(`${end.code} · cause ${end.cause || "release"}`);
  if (end.detail) lines.push(end.detail);
  const hold = `${end.kind || "jog"} ${end.axis}${end.frame ? ` (${end.frame})` : ""}`.trim();
  if (end.speedUnit) {
    lines.push(`${hold} · asked ${end.requestedSpeed.toFixed(1)} ${end.speedUnit}, reached ${end.achievedSpeed.toFixed(1)} ${end.speedUnit} · travelled ${end.distance.toFixed(1)} ${end.distanceUnit} in ${Math.round(end.durationMs)} ms`);
  } else if (hold) {
    lines.push(hold);
  }
  const timing = [`last intent ${Math.round(end.lastIntentAgeMs)} ms before the end (limit ${Math.round(end.lifetimeMs)} ms)`];
  if (end.samplesAhead !== null) timing.push(`${end.samplesAhead} samples ahead`);
  if (end.governing) timing.push(`limited by ${end.governing}`);
  lines.push(timing.join(" · "));
  const who = [end.clientLabel ? `jog from ${end.clientLabel}` : "", end.endedBy ? `ended by ${end.endedBy}` : ""].filter(Boolean);
  if (who.length) lines.push(who.join(" · "));
  return lines;
}

interface Shown {
  end: JogEnd;
  tone: JogEndTone;
  untilMs: number;
}

/**
 * Feed it every status body; draw `notice(now)`. A client without an id (an
 * older pendant) never owns an end: it only ever gets notes.
 */
export class JogEndTracker {
  readonly #clientId: string;
  #lastKey: string | null = null;
  #shown: Shown | null = null;
  #expanded = false;

  constructor(clientId: string) {
    this.#clientId = clientId;
  }

  /** One status body (or its last_jog_end). True when the notice changed. */
  observe(value: unknown, nowMs: number): boolean {
    const end = normalizeJogEnd(value);
    if (!end) return false;
    const key = `${end.seq}|${end.at}`;
    const shown = this.#shown;
    if (key === this.#lastKey) {
      // The same end, revised (an uncertain End that the Stop fence then
      // disarmed): update what is on screen; never re-raise a dismissed one.
      if (shown && shown.end.seq === end.seq && (shown.end.text !== end.text || shown.end.disarmed !== end.disarmed)) {
        shown.end = end;
        return true;
      }
      return false;
    }
    this.#lastKey = key;
    if (!end.popup || end.ageMs > JOG_END_FRESH_MS) return false;
    const tone: JogEndTone = this.#clientId !== "" && end.clientId === this.#clientId ? "toast" : "note";
    this.#shown = { end, tone, untilMs: nowMs + (tone === "toast" ? JOG_END_TOAST_MS : JOG_END_NOTE_MS) };
    this.#expanded = false;
    return true;
  }

  /** The × button. */
  dismiss(): void {
    this.#shown = null;
    this.#expanded = false;
  }

  /** The Details button: opens or closes the technical detail. While it is
   *  open the toast does not time out; closing it restarts the timer. */
  toggleDetails(nowMs: number): void {
    if (!this.#shown) return;
    this.#expanded = !this.#expanded;
    if (!this.#expanded) this.#shown.untilMs = nowMs + JOG_END_TOAST_MS;
  }

  notice(nowMs: number): JogEndNotice {
    const shown = this.#shown;
    if (!shown) return INACTIVE;
    if (!this.#expanded && nowMs >= shown.untilMs) {
      this.#shown = null;
      return INACTIVE;
    }
    const { end, tone } = shown;
    const text = tone === "toast" ? end.text
      : `${end.clientLabel || "Another client"}: ${end.text}`;
    return {
      active: true, tone, seq: end.seq, code: end.code, kicker: kickerOf(end, tone), text,
      details: jogEndDetails(end), expanded: this.#expanded,
      remainingMs: this.#expanded ? JOG_END_TOAST_MS : Math.max(0, shown.untilMs - nowMs),
    };
  }
}
