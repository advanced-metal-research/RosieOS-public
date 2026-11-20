import {
  parseRobotV4Catalog,
  parseRobotV4Program,
  type RobotV4Program,
  type RobotV4ProgramCatalog,
} from "../contracts/robot-v4-program.ts";

export const PROGRAMS_PATH = "/api/offline-programming/v1/programs" as const;
export const PROGRAM_SAVE_SCHEMA = "offline-programming.program-save.v1" as const;

export interface ProgramCatalogClientOptions {
  fetch?: typeof fetch;
  timeoutMs?: number;
}

export interface ProgramEntry {
  catalogRevision: number;
  program: RobotV4Program;
}

export interface ProgramRevisions {
  programId: string;
  catalogRevision: number;
  revisions: RobotV4Program[];
}

export class ProgramCatalogConflictError extends Error {
  readonly catalogRevision: number | null;
  readonly programId: string | null;
  readonly currentProgramRevision: number | null;

  constructor(input: {
    message: string;
    catalogRevision?: number | null;
    programId?: string | null;
    currentProgramRevision?: number | null;
  }) {
    super(input.message);
    this.name = "ProgramCatalogConflictError";
    this.catalogRevision = input.catalogRevision ?? null;
    this.programId = input.programId ?? null;
    this.currentProgramRevision = input.currentProgramRevision ?? null;
  }
}

export class ProgramCatalogClient {
  readonly #fetch: typeof fetch;
  readonly #timeoutMs: number;

  constructor(options: ProgramCatalogClientOptions = {}) {
    this.#fetch = options.fetch ?? globalThis.fetch.bind(globalThis);
    this.#timeoutMs = boundedInteger(options.timeoutMs ?? 8_000, 100, 30_000);
  }

  async list(signal?: AbortSignal): Promise<RobotV4ProgramCatalog> {
    return parseRobotV4Catalog(await this.request(PROGRAMS_PATH, { method: "GET" }, signal));
  }

  async get(
    programId: string,
    revision?: number,
    signal?: AbortSignal,
  ): Promise<ProgramEntry> {
    const path = programPath(programId) +
      (revision === undefined ? "" : `?revision=${positiveInteger(revision, "revision")}`);
    const value = record(await this.request(path, { method: "GET" }, signal), "program entry");
    if (value.schema !== "robot.v4.program-entry.v1" || value.owner !== "motion-server") {
      throw new Error("Motion Server program entry response is invalid");
    }
    return {
      catalogRevision: nonNegativeInteger(value.catalog_revision, "catalog revision"),
      program: parseRobotV4Program(value.program),
    };
  }

  async revisions(programId: string, signal?: AbortSignal): Promise<ProgramRevisions> {
    const value = record(
      await this.request(`${programPath(programId)}/revisions`, { method: "GET" }, signal),
      "program revisions",
    );
    if (value.schema !== "robot.v4.program-revisions.v1" ||
        value.owner !== "motion-server" ||
        value.program_id !== programId ||
        !Array.isArray(value.revisions)) {
      throw new Error("Motion Server program revisions response is invalid");
    }
    return {
      programId,
      catalogRevision: nonNegativeInteger(value.catalog_revision, "catalog revision"),
      revisions: value.revisions.map(parseRobotV4Program),
    };
  }

  async save(
    programValue: RobotV4Program,
    catalogRevision: number,
    signal?: AbortSignal,
  ): Promise<RobotV4ProgramCatalog> {
    const program = parseRobotV4Program(programValue);
    if (!program.program_digest) {
      throw new Error("program digest is required before catalog save");
    }
    const create = program.base_program_revision === 0;
    if (!create && program.program_revision <= program.base_program_revision) {
      throw new Error("program revision must be greater than its explicit base revision");
    }
    const requestId = requestIdentity("request");
    const correlationId = requestIdentity("catalog-save");
    const value = record(await this.request(
      create ? PROGRAMS_PATH : programPath(program.program_id),
      {
        method: create ? "POST" : "PUT",
        headers: {
          "Accept": "application/json",
          "Content-Type": "application/json",
          "X-Request-ID": requestId,
          "X-Correlation-ID": correlationId,
          "Idempotency-Key": [
            "save",
            program.program_id,
            program.program_revision,
            program.program_digest,
          ].join(":"),
        },
        body: JSON.stringify({
          schema: PROGRAM_SAVE_SCHEMA,
          catalog_revision: nonNegativeInteger(catalogRevision, "catalog revision"),
          program,
        }),
      },
      signal,
    ), "program save");
    if (value.ok !== true || !("catalog" in value)) {
      throw new Error("Motion Server program save response is invalid");
    }
    return parseRobotV4Catalog(value.catalog);
  }

  private async request(
    path: string,
    init: RequestInit,
    callerSignal?: AbortSignal,
  ): Promise<unknown> {
    const controller = new AbortController();
    let timedOut = false;
    const abort = (): void => controller.abort(callerSignal?.reason);
    if (callerSignal?.aborted) abort();
    else callerSignal?.addEventListener("abort", abort, { once: true });
    const timer = setTimeout(() => {
      timedOut = true;
      controller.abort("program catalog timeout");
    }, this.#timeoutMs);
    try {
      const response = await this.#fetch(path, {
        ...init,
        cache: "no-store",
        credentials: "same-origin",
        redirect: "error",
        signal: controller.signal,
      });
      let value: unknown;
      try {
        value = await response.json() as unknown;
      } catch {
        throw new Error(`Motion Server program catalog returned non-JSON HTTP ${response.status}`);
      }
      if (response.status === 409) throw conflict(value);
      if (!response.ok) {
        const payload = isRecord(value) ? value : {};
        const message = typeof payload.error === "string"
          ? payload.error
          : `Motion Server program catalog HTTP ${response.status}`;
        throw new Error(message);
      }
      return value;
    } catch (error) {
      if (error instanceof ProgramCatalogConflictError) throw error;
      if (controller.signal.aborted) {
        throw new Error(timedOut
          ? `Motion Server program catalog timed out after ${this.#timeoutMs} ms`
          : "Motion Server program catalog request was canceled");
      }
      throw error;
    } finally {
      clearTimeout(timer);
      callerSignal?.removeEventListener("abort", abort);
    }
  }
}

function conflict(value: unknown): ProgramCatalogConflictError {
  const payload = isRecord(value) ? value : {};
  return new ProgramCatalogConflictError({
    message: typeof payload.error === "string" ? payload.error : "program was edited elsewhere",
    catalogRevision: optionalNonNegativeInteger(payload.catalog_revision),
    programId: typeof payload.program_id === "string" ? payload.program_id : null,
    currentProgramRevision: optionalNonNegativeInteger(payload.current_program_revision),
  });
}

function programPath(programId: string): string {
  if (!/^[A-Za-z0-9._-]{1,128}$/.test(programId)) {
    throw new Error("program identity is invalid");
  }
  return `${PROGRAMS_PATH}/${encodeURIComponent(programId)}`;
}

function requestIdentity(prefix: string): string {
  const uuid = globalThis.crypto?.randomUUID?.();
  return `${prefix}-${uuid ?? `${Date.now()}-${Math.random().toString(16).slice(2)}`}`;
}

function record(value: unknown, name: string): Record<string, unknown> {
  if (!isRecord(value)) throw new Error(`${name} response is invalid`);
  return value;
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function positiveInteger(value: unknown, name: string): number {
  if (!Number.isSafeInteger(value) || Number(value) < 1) {
    throw new Error(`${name} must be a positive integer`);
  }
  return Number(value);
}

function nonNegativeInteger(value: unknown, name: string): number {
  if (!Number.isSafeInteger(value) || Number(value) < 0) {
    throw new Error(`${name} must be a non-negative integer`);
  }
  return Number(value);
}

function optionalNonNegativeInteger(value: unknown): number | null {
  return Number.isSafeInteger(value) && Number(value) >= 0 ? Number(value) : null;
}

function boundedInteger(value: number, minimum: number, maximum: number): number {
  if (!Number.isSafeInteger(value) || value < minimum || value > maximum) {
    throw new Error("program catalog timeout is out of bounds");
  }
  return value;
}
