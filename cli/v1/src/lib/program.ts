// The one program this server has open, in OLP's own engine, and what the
// browser keeps beside a program that is not in it: the part's STEP bytes
// (IndexedDB `sources`, keyed by fingerprint), the detected weld joints and
// searched seams of this session, and the planned trajectory's bytes.
//
// The engine holds one program in module state, as the Deck's olp-core does,
// so every program call runs through `run` one at a time.
import { createHash, randomUUID } from "node:crypto";
import { mkdir, readFile, writeFile } from "node:fs/promises";
import path from "node:path";

import type { Config } from "./config.js";
import {
  loadEngine, type Engine, type RailsJoint, type RailsSeam, type RailsSettings, type RailsSettingsChange, type RailsSettingsState,
  type SeamMeasurer, type SeamPlan, type WorkpieceMesh,
} from "./engine.js";
import { refusalText } from "./http.js";
import { DENSE, OLP_API, type OlpClient } from "./olp.js";

export interface ProgramLink {
  /** Where the open program came from, and so where Save goes by default. */
  source: "new" | "cell" | "file";
  /** The cell store revision the open program was edited from; null when the cell has never had it. */
  cellRevision: number | null;
  path: string | null;
}

export interface SessionState {
  link: ProgramLink | null;
  joints: { fingerprint: string; doc: JointsDoc } | null;
  seamPlans: [string, SeamPlan][];
  railsState: RailsSettingsState | null;
}

export interface JointsDoc {
  weld_joints: RailsJoint[];
  seams: RailsSeam[];
  [key: string]: unknown;
}

/** Seam-worker operations take seconds to minutes (plan_torch_path searches a lattice). */
const SEAM_TIMEOUT_MS = 600_000;
/** The CAD worker's mesh ceiling is ~240 s (cad/client.ts DEFAULT_MESH_TIMEOUT_MS 245 s). */
const TESSELLATE_TIMEOUT_MS = 300_000;

export class ProgramSession {
  link: ProgramLink | null = null;
  /** Detected joints of the part this session analysed, by fingerprint. */
  joints: { fingerprint: string; doc: JointsDoc } | null = null;
  /** Searched seams, as rails mode caches them (`seamPlans`), keyed by fingerprint and seam. */
  readonly seamPlans = new Map<string, SeamPlan>();
  /**
   * The fitted torch and angle search, as the browser's two parameter
   * windows hold them: session state, never the program's. Null is the
   * windows' fallback (each scope's first shipped preset).
   */
  railsState: RailsSettingsState | null = null;
  /** Tessellated STEP fixtures and inactive parts by lowercase fingerprint (main.ts workcellFixtureMeshes). */
  readonly meshes = new Map<string, WorkpieceMesh>();
  /** The last GET /robots answer installed: the CLI keeps it so a program still opens with OLP down. */
  catalogue: unknown = null;
  private queue: Promise<unknown> = Promise.resolve();

  constructor(
    private readonly config: Config,
    private readonly olp: OlpClient,
  ) {}

  /** One program operation at a time: the engine's open program is shared state. */
  run<T>(operation: (engine: Engine) => Promise<T>): Promise<T> {
    const next = this.queue.then(async () => operation(await loadEngine()));
    this.queue = next.catch(() => undefined);
    return next;
  }

  /** Throws unless a program is open. */
  requireOpen(engine: Engine): void {
    if (!this.link) throw new Error("No program is open: rosie_program_new or rosie_program_open first");
    engine.projectIdentity();
  }

  /**
   * GET /robots and the selected cell's robot, as the browser installs them at
   * boot: placement hangs off the model's work frame, and pinning reads the
   * catalogue's description identity.
   */
  async installRobots(engine: Engine): Promise<{ cellModel: string; connected: boolean }> {
    const catalogue = await this.olp.getOk(`${OLP_API}/robots`, "robot catalogue");
    this.catalogue = catalogue;
    let connected: { modelId: string; robotDescriptionSha256: string } | null = null;
    let cellModel = "";
    let selected = false;
    try {
      const status = await this.olp.status();
      selected = Boolean(status.available && status.target);
      const robot = (status.robot_cell ?? {}) as Record<string, unknown>;
      // A machine that binds no description answers empty strings here (valid: false).
      cellModel = String(robot.model_id || status.target?.model_id || "");
      if (robot.valid === true && typeof robot.model_id === "string" && robot.model_id && typeof robot.robot_description_sha256 === "string" && robot.robot_description_sha256) {
        connected = { modelId: robot.model_id, robotDescriptionSha256: robot.robot_description_sha256 };
      }
    } catch {
      // No execution status: plan and author offline, as the browser does with no cell.
    }
    engine.setRobots(catalogue, connected);
    return { cellModel, connected: selected };
  }

  /**
   * Feed the engine's machine view (olp-core machine.ts) the selected cell's
   * capabilities and a fresh status, as the Deck's poll does. `statusAgeMs` is
   * the request's own round trip: OLP's freshness rules judge it.
   */
  async observeMachine(engine: Engine, withCapabilities = true): Promise<{ selected: boolean; statusAgeMs: number; cellModel: string; cellId: string }> {
    if (withCapabilities) {
      const capabilities = await this.olp.getOk(`${DENSE}/capabilities`, "capabilities");
      try {
        engine.observeCapabilities(capabilities);
      } catch (error) {
        throw new Error(`${error instanceof Error ? error.message : String(error)}: OLP's client code in this checkout refuses the selected cell's rt-core contract; this server must be built from the commit the cell runs (capabilities digest)`);
      }
    }
    const started = Date.now();
    const status = await this.olp.status();
    engine.observeStatus(status, false);
    return {
      selected: Boolean(status.available && status.target),
      statusAgeMs: Date.now() - started,
      cellModel: status.target?.model_id ?? "",
      cellId: status.target?.cell_id ?? "",
    };
  }

  forgetPart(): void {
    this.joints = null;
    this.seamPlans.clear();
  }

  /** The session's torch and search, validated by OLP's parameter model. */
  rails(engine: Engine): RailsSettings {
    return engine.railsSettings(this.railsState);
  }

  /**
   * Change the torch or search (or `reset` to the shipped fallbacks). As the
   * browser's parametersChanged: a change drops every cached search, since
   * it was measured with numbers that are no longer the numbers; accepted
   * welds keep what they were searched with. A refused change keeps the old.
   */
  setRails(engine: Engine, change: RailsSettingsChange | "reset"): { settings: RailsSettings; dropped: number } {
    const before = JSON.stringify(this.rails(engine).state);
    const settings = change === "reset" ? engine.railsSettings(null) : engine.railsSettings(this.railsState, change);
    this.railsState = change === "reset" ? null : settings.state;
    let dropped = 0;
    if (JSON.stringify(settings.state) !== before) {
      dropped = this.seamPlans.size;
      this.seamPlans.clear();
    }
    return { settings, dropped };
  }

  /**
   * A STEP's surface through OLP's CAD client, kept under its fingerprint:
   * in this process, and CadQuery's wire reply on disk
   * (`<stateDir>/meshes/<fingerprint>.json`), so a later process -- the next
   * CLI command, a snapshot render -- normalises the same reply through OLP's
   * own client instead of waiting up to minutes for CadQuery again.
   */
  async tessellate(engine: Engine, filename: string, bytes: Uint8Array): Promise<WorkpieceMesh> {
    const fingerprint = createHash("sha256").update(bytes).digest("hex");
    const cached = await this.meshWire(fingerprint);
    let wire: unknown = null;
    const mesh = await engine.tessellateStep(async (body) => {
      if (cached) return { status: 200, body: cached };
      const reply = await this.olp.post(`${OLP_API}/cadquery/topology`, body, { timeoutMs: TESSELLATE_TIMEOUT_MS });
      if (reply.ok) wire = reply.body;
      return { status: reply.status, body: reply.body };
    }, filename, bytes);
    if (wire && mesh.fingerprint.toLowerCase() === fingerprint) {
      const file = this.meshPath(fingerprint);
      await mkdir(path.dirname(file), { recursive: true });
      await writeFile(file, JSON.stringify(wire));
    }
    this.meshes.set(mesh.fingerprint.toLowerCase(), mesh);
    return mesh;
  }

  private meshPath(fingerprint: string): string {
    if (!/^[0-9a-f]{64}$/.test(fingerprint)) throw new Error(`not a STEP fingerprint: ${fingerprint}`);
    return path.join(this.config.stateDir, "meshes", `${fingerprint}.json`);
  }

  /** CadQuery's tessellate reply for a fingerprint, if one is kept on disk. */
  async meshWire(fingerprint: string): Promise<unknown | null> {
    try {
      return JSON.parse(await readFile(this.meshPath(fingerprint.toLowerCase()), "utf8")) as unknown;
    } catch {
      return null;
    }
  }

  /** The tessellate reply for a kept STEP: from disk, else CadQuery now (kept for next time). */
  async meshWireFor(engine: Engine, fingerprint: string, stepPath?: string): Promise<unknown> {
    const kept = await this.meshWire(fingerprint);
    if (kept) return kept;
    const source = await this.getSource(fingerprint.toLowerCase(), stepPath);
    await this.tessellate(engine, source.filename, source.bytes);
    const fresh = await this.meshWire(fingerprint);
    if (!fresh) throw new Error(`CadQuery tessellated ${source.filename} but its reply could not be kept`);
    return fresh;
  }

  /**
   * What the browser keeps beside a program but not in it, as JSON: where it
   * came from (and so the cell revision a save is checked against), the
   * detected joints, the searched seams and the torch/search settings. The
   * CLI persists it per workspace so each command, a fresh process, resumes
   * exactly where the last one stopped; meshes live in the shared state dir.
   */
  exportState(): SessionState {
    return { link: this.link, joints: this.joints, seamPlans: [...this.seamPlans.entries()], railsState: this.railsState };
  }

  importState(state: Partial<SessionState> | null): void {
    this.link = state?.link ?? null;
    this.joints = state?.joints ?? null;
    this.seamPlans.clear();
    for (const [key, plan] of state?.seamPlans ?? []) this.seamPlans.set(key, plan);
    this.railsState = state?.railsState ?? null;
  }

  /** Keep STEP files the caller names (a program opened from the cell whose fixtures this server never imported). */
  async keepSources(paths: readonly string[]): Promise<string[]> {
    const kept: string[] = [];
    for (const file of paths) kept.push(await this.putSource(path.basename(file), await readFile(file)));
    return kept;
  }

  /**
   * Every STEP fixture and inactive part the open program's plan must carry
   * as a mesh: tessellated from the STEP this server kept, as the browser's
   * loadWorkcellFixtureMeshes does from its IndexedDB sources. Answers the
   * map planRequest takes and which fingerprints were tessellated now.
   */
  async fixtureMeshes(engine: Engine): Promise<{ meshes: Map<string, WorkpieceMesh>; tessellated: string[] }> {
    const tessellated: string[] = [];
    for (const need of engine.fixtureMeshNeeds()) {
      if (this.meshes.has(need.fingerprint)) continue;
      // tessellate() answers from the kept wire reply when there is one.
      let source: { filename: string; bytes: Uint8Array };
      try {
        source = await this.getSource(need.fingerprint);
      } catch {
        throw new Error(`The STEP for ${need.name} (${need.filename}, ${need.fingerprint.slice(0, 12)}…) is not kept by this server, so it cannot be sent to the planner as an obstacle: pass it in fixture_step_paths, or replace or remove that fixture`);
      }
      const mesh = await this.tessellate(engine, source.filename, source.bytes);
      if (mesh.fingerprint.toLowerCase() !== need.fingerprint) {
        throw new Error(`CadQuery fingerprinted ${source.filename} as ${mesh.fingerprint.slice(0, 12)}…, not ${need.fingerprint.slice(0, 12)}…`);
      }
      tessellated.push(need.fingerprint);
    }
    return { meshes: this.meshes, tessellated };
  }

  private sourcePath(fingerprint: string): string {
    if (!/^[0-9a-f]{64}$/.test(fingerprint)) throw new Error(`not a STEP fingerprint: ${fingerprint}`);
    return path.join(this.config.stateDir, "sources", `${fingerprint}.step`);
  }

  /** Keep a part's bytes under its fingerprint (the lowercase SHA-256 of the STEP, as CadQuery reports it). */
  async putSource(filename: string, bytes: Uint8Array): Promise<string> {
    const fingerprint = createHash("sha256").update(bytes).digest("hex");
    const file = this.sourcePath(fingerprint);
    await mkdir(path.dirname(file), { recursive: true });
    await writeFile(file, bytes);
    await writeFile(`${file}.json`, JSON.stringify({ filename }));
    return fingerprint;
  }

  /** The part's STEP, from this server's keep or from a file the caller names (checked against the fingerprint). */
  async getSource(fingerprint: string, fallbackPath?: string): Promise<{ filename: string; bytes: Uint8Array }> {
    if (fallbackPath) {
      const bytes = await readFile(fallbackPath);
      const sum = createHash("sha256").update(bytes).digest("hex");
      if (sum !== fingerprint) throw new Error(`${fallbackPath} is not this program's part (sha256 ${sum.slice(0, 12)}…, the program's part is ${fingerprint.slice(0, 12)}…)`);
      await this.putSource(path.basename(fallbackPath), bytes);
      return { filename: path.basename(fallbackPath), bytes };
    }
    const file = this.sourcePath(fingerprint);
    try {
      const bytes = await readFile(file);
      const meta = JSON.parse(await readFile(`${file}.json`, "utf8").catch(() => "{}")) as { filename?: string };
      return { filename: meta.filename ?? "part.step", bytes };
    } catch {
      throw new Error(`This server has no copy of the part's STEP (fingerprint ${fingerprint.slice(0, 12)}…). STEP bytes are never shared between clients: pass step_path pointing at the same file, or import it again`);
    }
  }

  /** Keep a planned trajectory's bytes, as the browser keeps them beside the plan record. */
  async keepTrajectory(digest: string, base64: string): Promise<string> {
    const file = path.join(this.config.stateDir, "dense", `${digest.replace(/^sha256:/, "")}.rdt`);
    await mkdir(path.dirname(file), { recursive: true });
    await writeFile(file, Buffer.from(base64, "base64"));
    return file;
  }

  /** A planned trajectory's bytes: a file named, this server's kept copy, or the planner's store by digest. */
  async trajectoryBytes(digest: string, file?: string): Promise<Uint8Array> {
    if (file) return readFile(file);
    try {
      return await readFile(path.join(this.config.stateDir, "dense", `${digest.replace(/^sha256:/, "")}.rdt`));
    } catch {
      const response = await fetch(`${this.config.plannerUrl}/api/motion/dense/${digest}`, { signal: AbortSignal.timeout(60_000) }).catch(() => null);
      if (!response?.ok) throw new Error(`No copy of trajectory ${digest} here and the weld planner (${this.config.plannerUrl}) does not serve it: plan again or pass trajectory_path`);
      const bytes = new Uint8Array(await response.arrayBuffer());
      await this.keepTrajectory(digest, Buffer.from(bytes).toString("base64"));
      return bytes;
    }
  }

  /** POST /seam: the seam worker, one subprocess per request inside OLP. */
  async seam<T>(body: Record<string, unknown>): Promise<T> {
    const reply = await this.olp.post(`${OLP_API}/seam`, body, { timeoutMs: SEAM_TIMEOUT_MS });
    if (!reply.ok) throw new Error(`the seam worker refused ${String(body.operation)} (HTTP ${reply.status}): ${refusalText(reply)}`);
    return reply.body as T;
  }

  measurer(): SeamMeasurer {
    const base64 = (bytes: Uint8Array) => Buffer.from(bytes).toString("base64");
    return {
      planTorchPath: (step, body) => this.seam({ operation: "plan_torch_path", step_base64: base64(step), ...body }),
      checkTorchFits: (step, body) => this.seam({ operation: "check_torch_fits", step_base64: base64(step), ...body }),
    };
  }

  /** A fresh id for a recorded waypoint. */
  newId(): string {
    return randomUUID();
  }
}
