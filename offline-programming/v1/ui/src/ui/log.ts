import { Signal } from "./signal.ts";

/**
 * The app's log: one bus every module writes to through its own logger,
 * read by the status bar (worst level not yet seen) and the log panel
 * (every entry, with its time, module, level and text).
 *
 * A status line that only shows the latest text loses the error that
 * scrolled by while the operator was looking at the part. Here nothing
 * is lost until the ring turns over: an error stays counted until the
 * panel is opened, and the panel shows what happened, when, and where.
 *
 * Levels: `info` is what happened; `warn` is something the operator
 * should know about but that did not stop anything; `error` stopped
 * something. `debug` is for the panel's filter and never colours the bar.
 */
export type LogLevel = "debug" | "info" | "warn" | "error";

export interface LogEntry {
  id: number;
  /** Milliseconds since the epoch. */
  timeMs: number;
  level: LogLevel;
  module: string;
  message: string;
  /** Anything more: an error's stack, a payload summary. */
  detail?: string;
}

export interface Logger {
  readonly module: string;
  debug(message: string, detail?: unknown): void;
  info(message: string, detail?: unknown): void;
  warn(message: string, detail?: unknown): void;
  error(message: string, detail?: unknown): void;
  /** Log at a level chosen by the caller. */
  at(level: LogLevel, message: string, detail?: unknown): void;
}

const LEVEL_RANK: Readonly<Record<LogLevel, number>> = { debug: 0, info: 1, warn: 2, error: 3 };

export class LogBus {
  readonly changed = new Signal<LogEntry | null>();
  readonly #entries: LogEntry[] = [];
  readonly #capacity: number;
  #nextId = 1;
  /** The id up to which the operator has looked at the panel. */
  #seenThrough = 0;

  constructor(capacity = 500) {
    this.#capacity = capacity;
  }

  get entries(): readonly LogEntry[] {
    return this.#entries;
  }

  push(level: LogLevel, module: string, message: string, detail?: unknown): LogEntry {
    const entry: LogEntry = {
      id: this.#nextId++,
      timeMs: Date.now(),
      level,
      module,
      message,
      ...(detail === undefined ? {} : { detail: describe(detail) }),
    };
    this.#entries.push(entry);
    if (this.#entries.length > this.#capacity) this.#entries.splice(0, this.#entries.length - this.#capacity);
    this.changed.emit(entry);
    return entry;
  }

  logger(module: string): Logger {
    const at = (level: LogLevel, message: string, detail?: unknown): void => {
      this.push(level, module, message, detail);
    };
    return {
      module,
      at,
      debug: (message, detail) => at("debug", message, detail),
      info: (message, detail) => at("info", message, detail),
      warn: (message, detail) => at("warn", message, detail),
      error: (message, detail) => at("error", message, detail),
    };
  }

  /** The worst level among entries the operator has not looked at, or
   *  null when every unseen entry is info or there is none. */
  unseenWorst(): "warn" | "error" | null {
    let worst: "warn" | "error" | null = null;
    for (const entry of this.#entries) {
      if (entry.id <= this.#seenThrough) continue;
      if (entry.level === "error") return "error";
      if (entry.level === "warn") worst = "warn";
    }
    return worst;
  }

  /** How many unseen entries at or above a level. */
  unseenCount(level: LogLevel = "warn"): number {
    let count = 0;
    for (const entry of this.#entries) {
      if (entry.id > this.#seenThrough && LEVEL_RANK[entry.level] >= LEVEL_RANK[level]) count += 1;
    }
    return count;
  }

  /** The operator has looked: nothing so far colours the bar any more. */
  markSeen(): void {
    const last = this.#entries.at(-1);
    this.#seenThrough = last ? last.id : this.#seenThrough;
    this.changed.emit(null);
  }

  clear(): void {
    this.#entries.length = 0;
    this.#seenThrough = this.#nextId - 1;
    this.changed.emit(null);
  }
}

/** A detail as one string: an Error keeps its stack, an object its JSON. */
export function describe(detail: unknown): string {
  if (detail instanceof Error) return detail.stack ?? `${detail.name}: ${detail.message}`;
  if (typeof detail === "string") return detail;
  try {
    return JSON.stringify(detail, null, 1);
  } catch {
    return String(detail);
  }
}

/** Guess a level from a status line's wording, for the lines the app
 *  already writes as prose: a "blocked", "failed" or "refused" line is a
 *  warning; the rest is information. */
export function levelOfStatus(text: string): LogLevel {
  return /\b(blocked|failed|fail|refus|cannot|could not|not available|unavailable|error|locked|missing|did not)\b/i.test(text)
    ? "warn"
    : "info";
}

/** The one bus for the app. Modules take loggers from it. */
export const appLog = new LogBus();
