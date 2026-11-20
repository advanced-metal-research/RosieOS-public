// Program orchestration over OLP's own engine, shared by the CLI and the MCP
// server: the multi-step flows (open, save with its revision check, capture,
// record from the robot, STEP import, seam search, accept a weld, plan,
// preview, export) that hold no OLP rule of their own but must run in OLP's
// order with OLP's requests. Every rule (fields, capture, placement, weld
// nodes, plan body) is a call into the engine; nothing here re-implements one.
// Front ends render the results; a refusal is thrown as an Error carrying
// OLP's own words.
import { readFile, stat, writeFile } from "node:fs/promises";
import path from "node:path";

import type { Context } from "./context.js";
import type {
  Engine, FixtureEdit, FixtureField, FixtureView, PartRecordView, PlanRecordView, RailsJoint, RailsSeam, SeamPlan, SegmentPath,
} from "./engine.js";
import { refusalText } from "./http.js";
import { DENSE, OLP_API } from "./olp.js";
import type { JointsDoc, ProgramSession } from "./program.js";
import { json, table } from "./text.js";

/** The browser's STEP import limit (cad/client.ts). */
export const MAX_STEP_BYTES = 8 * 1024 * 1024;
/** POST /weld-plan's own budget (weld_plan.go weldPlanTimeout). */
export const PLAN_TIMEOUT_MS = 30 * 60 * 1000;
// Unverified bound: 20 polls 75 ms apart (1.5 s) covers OLP's 100 ms
// stillness window several times over when the drives report no velocity.
const STILL_POLLS = 20;
const STILL_POLL_MS = 75;

/** An OLP refusal with the HTTP status, so a front end can tell a 409 conflict from a 423 lock. */
export class ProgramRefusal extends Error {
  constructor(message: string, readonly status?: number, readonly body?: unknown) {
    super(message);
  }
}

// ---------------------------------------------------------------------------
// Rendering shared by both front ends: the spreadsheet as OLP shows it.

export const mm = (values: number[]) => values.map((v) => Math.round(v * 10) / 10).join(", ");

export function describeProgram(engine: Engine, session: ProgramSession): Record<string, unknown> {
  const view = engine.view();
  return {
    link: session.link, ...view, part: engine.partView(), parts: engine.partsView(), workcell: engine.fixturesView().fixtures,
    plan: engine.planRecord(), joint_names: engine.jointNames(),
  };
}

export function fixtureLine(fixture: FixtureView, session: ProgramSession): string {
  const shape = fixture.shape.kind === "box"
    ? `box ${mm(fixture.shape.sizeMm)} mm`
    : `STEP ${fixture.shape.filename}${session.meshes.has(fixture.shape.fingerprint.toLowerCase()) ? "" : " (mesh not yet tessellated here)"}`;
  return `\`${fixture.id}\` ${fixture.name}: ${shape} at [${mm(fixture.positionMm)}] mm, rpy [${mm(fixture.rotationDeg)}]°${fixture.origin === "cell-default" ? " (cell placeholder)" : ""}`;
}

export function spreadsheet(engine: Engine, session: ProgramSession): string {
  const view = engine.view();
  const part = engine.partView();
  const parts = engine.partsView();
  const workcell = engine.fixturesView().fixtures;
  const plan = engine.planRecord();
  const link = session.link;
  const cell = (row: (typeof view.rows)[number]) => {
    const c = row.cells;
    if (row.op === "move") {
      const xyz = [0, 1, 2].map((i) => Number(c[`target.xyz_m.${i}`]).toFixed(1)).join(", ");
      const rpy = [0, 1, 2].map((i) => Number(c[`target.rpy_rad.${i}`]).toFixed(1)).join(", ");
      return `${c.motion} · xyz [${xyz}] mm · rpy [${rpy}]° · ${c.speed_mm_s} mm/s · ${Number(c.speed_scale).toFixed(0)}%`;
    }
    return [row.geometry, row.process, row.freespace].filter(Boolean).join(" · ");
  };
  return [
    `# ${view.name} (${view.model}, revision ${view.revision}, ${view.programState})`,
    `- **Source**: ${link?.source ?? "?"}${link?.cellRevision ? ` · cell store rev ${link.cellRevision}` : link?.source === "cell" || link?.source === "new" ? " · not yet on the cell" : ""}${link?.path ? ` · ${link.path}` : ""}`,
    `- **Authoring**: ${view.railsAuthoring ? "rails (welds from the part's detected seams)" : "classic"} · plan speed ${(view.speedScale * 100).toFixed(0)}% · undo ${view.canUndo ? "yes" : "no"} / redo ${view.canRedo ? "yes" : "no"}`,
    part.cad
      ? `- **Part**: ${part.cad.filename} (\`${part.cad.fingerprint.slice(0, 12)}…\`, ${part.edges} edges)${part.placement ? ` at [${part.placement.offsetFromWorkFrameMm.map((v) => v.toFixed(1)).join(", ")}] mm, rpy [${part.placement.rotationRpyDeg.map((v) => v.toFixed(1)).join(", ")}]° in the work frame` : ""}`
      : "- **Part**: none",
    parts.length > 1 || (parts.length === 1 && !parts[0]!.active)
      ? `- **Parts**: ${parts.map((p) => `\`${p.id}\` ${p.filename} (${p.welds} weld${p.welds === 1 ? "" : "s"})${p.active ? " ACTIVE" : ""}`).join(" · ")}; only the active part's welds plan, the others go as obstacles`
      : "",
    `- **Workcell**: ${workcell.length ? workcell.map((f) => fixtureLine(f, session)).join("; ") : "no fixtures"}`,
    plan
      ? `- **Plan**: ${plan.current ? "CURRENT" : "STALE (edited since)"} · ${plan.totalSampleCount} samples, ${plan.totalDurationS.toFixed(1)} s · \`${plan.trajectoryDigest}\``
      : "- **Plan**: none",
    view.problems.length ? `- **Problems**: ${view.problems.join("; ")}` : "",
    "",
    view.rows.length
      ? table(["#", "Row id", "Op", "Label", "What", "Status", "On"], view.rows.map((row) => [row.number, row.id, row.op, row.label, cell(row), row.status, row.enabled ? "yes" : "no"]))
      : "The program has no rows yet.",
  ].filter((line) => line !== "").join("\n");
}

// ---------------------------------------------------------------------------
// Open, new, save

export async function newProgram(ctx: Context, engine: Engine, name: string, model?: string): Promise<string> {
  const session = ctx.program;
  const { cellModel } = await session.installRobots(engine);
  const chosen = model || engine.startupModel(cellModel);
  if (!chosen) throw new ProgramRefusal("OLP listed no robot to write a program for");
  engine.newProject(name, chosen);
  session.link = { source: "new", cellRevision: null, path: null };
  session.forgetPart();
  return chosen;
}

export async function openProgram(ctx: Context, engine: Engine, from: { source: "cell"; programId: string; revision?: number } | { source: "file"; path: string }): Promise<void> {
  const session = ctx.program;
  await session.installRobots(engine);
  if (from.source === "cell") {
    const reply = await ctx.olp.get(`${OLP_API}/cell-programs/programs/${encodeURIComponent(from.programId)}${from.revision ? `?revision=${from.revision}` : ""}`);
    if (!reply.ok) throw new ProgramRefusal(`the cell refused program ${from.programId}: ${refusalText(reply)}`, reply.status, reply.body);
    const body = reply.body as { program?: { revision?: number }; document?: unknown };
    engine.openCellProject(body.document);
    session.link = { source: "cell", cellRevision: Number(body.program?.revision ?? 0) || null, path: null };
  } else {
    engine.openProject(await readFile(from.path, "utf8"));
    session.link = { source: "file", cellRevision: null, path: from.path };
  }
  session.forgetPart();
}

export type SaveResult = { kind: "file"; path: string } | { kind: "cell"; id: string; revision: number; refusedCurrent: unknown };

/**
 * Save to the cell store (revision-checked: the save names the revision this
 * program was opened from and the cell refuses a stale one with 409
 * program_conflict instead of overwriting; 423 while the robot holds it) or
 * to a file (OLP's Export JSON envelope).
 */
export async function saveProgram(ctx: Context, engine: Engine, options: { target: "cell" | "file"; path?: string; saveAsCopy?: boolean; makeCurrent?: boolean }): Promise<SaveResult> {
  const session = ctx.program;
  if (options.saveAsCopy) {
    engine.copyOpenProject();
    session.link = { source: session.link?.source === "file" ? "file" : "new", cellRevision: null, path: null };
  }
  const document = engine.exportProject();
  if (options.target === "file") {
    const destination = options.path ?? session.link?.path;
    if (!destination) throw new ProgramRefusal("a file save needs a path");
    await writeFile(destination, document);
    session.link = { source: "file", cellRevision: session.link?.cellRevision ?? null, path: destination };
    return { kind: "file", path: destination };
  }
  const id = engine.projectIdentity();
  const base = session.link?.cellRevision ?? 0;
  // The store keeps the bare project, as cell-sync.ts cellDocument sends it; the file keeps the export envelope.
  const project = (JSON.parse(document) as { project: unknown }).project;
  const reply = await ctx.olp.post(`${OLP_API}/cell-programs/programs/${encodeURIComponent(id)}/save`, {
    base_revision: base, save_id: `${ctx.olp.clientId}-${id}-${base}-${engine.view().revision}`, make_current: options.makeCurrent ?? false, document: project,
  });
  if (!reply.ok) {
    const conflict = reply.status === 409 ? " Someone saved this program on the cell since it was opened here: reopen theirs, or save as a copy." : "";
    throw new ProgramRefusal(`the cell refused the save (HTTP ${reply.status}): ${refusalText(reply)}.${conflict}`, reply.status, reply.body);
  }
  const head = (reply.body as { program?: { revision?: number } }).program;
  session.link = { source: "cell", cellRevision: Number(head?.revision ?? base + 1), path: null };
  return { kind: "cell", id, revision: session.link.cellRevision!, refusedCurrent: (reply.body as { current_refused?: unknown }).current_refused ?? null };
}

// ---------------------------------------------------------------------------
// Taught moves

/** Apply optional field edits to a freshly recorded row; the first refusal names the row kept. */
function applyEdits(engine: Engine, id: string, edits: [string, unknown][], what: string): void {
  for (const [field, value] of edits) {
    try {
      engine.edit(id, field, value);
    } catch (error) {
      throw new ProgramRefusal(`${what} ${id}, but ${field} was refused: ${error instanceof Error ? error.message : String(error)}. The row stays as recorded; fix it with an edit or remove it.`);
    }
  }
}

export interface MoveOptions { motion?: string; xyzMm?: number[]; rpyDeg?: number[]; speedMmS?: number; speedPercent?: number; label?: string }

function moveEdits(options: MoveOptions): [string, unknown][] {
  const edits: [string, unknown][] = [];
  if (options.motion && options.motion !== "joint") edits.push(["motion", options.motion]);
  options.xyzMm?.forEach((v, i) => edits.push([`target.xyz_m.${i}`, v]));
  options.rpyDeg?.forEach((v, i) => edits.push([`target.rpy_rad.${i}`, v]));
  if (options.speedMmS !== undefined) edits.push(["speed_mm_s", options.speedMmS]);
  if (options.speedPercent !== undefined) edits.push(["speed_scale", options.speedPercent]);
  if (options.label) edits.push(["label", options.label]);
  return edits;
}

/** POST /robots/capture-model, then OLP's preview capture of these joints (degrees, URDF order). */
export async function capture(ctx: Context, engine: Engine, mode: "new" | "reteach" | "via", jointsDeg: number[], anchor: string | null): Promise<string> {
  await ctx.program.installRobots(engine);
  const jointsRad = jointsDeg.map((deg) => (deg * Math.PI) / 180);
  const reply = await ctx.olp.post(`${OLP_API}/robots/capture-model`, engine.captureModelRequest(jointsRad));
  if (!reply.ok) throw new ProgramRefusal(`OLP could not resolve the capture model: ${refusalText(reply)}`, reply.status);
  return engine.recordPreview(mode, jointsRad, anchor, reply.body as { urdf: string; identity: unknown }, ctx.program.newId());
}

export async function addWaypoint(ctx: Context, engine: Engine, jointsDeg: number[], afterRowId: string | null, options: MoveOptions): Promise<string> {
  const id = await capture(ctx, engine, "new", jointsDeg, afterRowId);
  applyEdits(engine, id, moveEdits(options), "Recorded");
  return id;
}

/**
 * A waypoint from the selected cell's MEASURED pose, by OLP's teach rules:
 * every described joint Home-valid, coordinates trusted, stationary (held
 * within OLP's stillness window), the cell's calibration for a machine that
 * binds a description; refused under the editing lock (loaded, staged,
 * playing or paused). Reads the robot; moves nothing.
 */
export async function recordRobotPose(ctx: Context, engine: Engine, mode: "new" | "reteach" | "via", rowId: string | null, afterRowId: string | null, options: MoveOptions): Promise<{ id: string; cellId: string; joints: string }> {
  const session = ctx.program;
  if (mode !== "new" && !rowId) throw new ProgramRefusal(`mode '${mode}' needs the row to ${mode === "via" ? "give a via pose" : "reteach"}`);
  await session.installRobots(engine);
  let machine = await session.observeMachine(engine);
  if (!machine.selected) throw new ProgramRefusal("OLP has no cell selected: the measured pose comes from the selected cell");
  const locked = () => engine.editingLockReason(machine.selected);
  if (locked()) throw new ProgramRefusal(`OLP will not record from the robot: ${locked()}`);
  let measured: ReturnType<Engine["teachObservation"]> | null = null;
  let refusal = "";
  for (let poll = 0; poll < STILL_POLLS && !measured; poll++) {
    if (poll > 0) {
      await new Promise((resolve) => setTimeout(resolve, STILL_POLL_MS));
      machine = await session.observeMachine(engine, false);
    }
    try {
      measured = engine.teachObservation({ selected: machine.selected, busy: false, statusAgeMs: machine.statusAgeMs, cellModel: machine.cellModel, cellId: machine.cellId });
    } catch (error) {
      refusal = error instanceof Error ? error.message : String(error);
    }
  }
  if (!measured) throw new ProgramRefusal(`OLP will not record from the robot: ${refusal}`);
  if (locked()) throw new ProgramRefusal(`OLP will not record from the robot: ${locked()}`);
  engine.select(mode === "new" ? afterRowId : rowId);
  const start = engine.captureStart(mode, measured);
  let calibration: Record<string, unknown> | null = null;
  if (start.machineBound) {
    const cell = await ctx.olp.get(`${DENSE}/cell`);
    if (!cell.ok) throw new ProgramRefusal(`OLP could not read the cell's calibration: ${refusalText(cell)}`, cell.status);
    calibration = engine.fetchedCellSettings(machine.selected, machine.statusAgeMs, cell.body);
  }
  const model = await ctx.olp.post(`${OLP_API}/robots/capture-model`, {
    model_id: start.modelId, robot_description_sha256: start.description,
    machine_planning_calibration_base64: String(calibration?.machinePlanningCalibrationBase64 ?? engine.programModelRequest().machine_planning_calibration_base64 ?? ""),
  });
  if (!model.ok) throw new ProgramRefusal(`OLP could not resolve the capture model: ${refusalText(model)}`, model.status);
  const resolved = model.body as { urdf: string; identity: unknown };
  const id = engine.captureRecord(mode, measured, start.revision, start.description, calibration, resolved.identity, engine.tcpPose(resolved.urdf, measured.pose), session.newId());
  const edits = moveEdits({ ...options, motion: options.motion });
  if (options.motion === "joint") edits.unshift(["motion", "joint"]);
  applyEdits(engine, id, edits, "Recorded from the robot");
  const joints = measured.pose.jointOrder.map((name, i) => `${name} ${((Number(measured!.pose.jointsRad[i]) * 180) / Math.PI).toFixed(3)}°`).join(", ");
  return { id, cellId: machine.cellId, joints };
}

/** show | pull the selected cell's settings and calibration | adopt the current robot description. */
export async function robotSettings(ctx: Context, engine: Engine, action: "show" | "pull" | "adopt"): Promise<{ headline: string; settings: ReturnType<Engine["robotSettings"]> }> {
  const session = ctx.program;
  await session.installRobots(engine);
  let headline = "";
  if (action === "pull") {
    const machine = await session.observeMachine(engine);
    if (!machine.selected) throw new ProgramRefusal("OLP has no cell selected to pull settings from");
    const cell = await ctx.olp.get(`${DENSE}/cell`);
    if (!cell.ok) throw new ProgramRefusal(`OLP could not read the cell's settings: ${refusalText(cell)}`, cell.status);
    const settings = engine.fetchedCellSettings(machine.selected, machine.statusAgeMs, cell.body);
    if (!settings) throw new ProgramRefusal("The selected cell is not reachable with fresh rt-core status; its settings cannot be pulled");
    headline = engine.pullCellSettings(settings) ? `Pulled ${machine.cellId}'s settings into the program.` : `The program already has ${machine.cellId}'s settings.`;
  } else if (action === "adopt") {
    engine.adoptDescription();
    headline = "Adopted the current robot description; plan again before running.";
  }
  return { headline, settings: engine.robotSettings() };
}

// ---------------------------------------------------------------------------
// STEP parts and rails welds

/** A STEP within OLP's import limit, read. */
export async function readStep(file: string): Promise<{ filename: string; bytes: Buffer }> {
  if (!/\.(step|stp)$/i.test(file)) throw new ProgramRefusal("the file must end in .step or .stp");
  const info = await stat(file);
  if (info.size > MAX_STEP_BYTES) throw new ProgramRefusal(`${file} is ${info.size} bytes; OLP imports at most ${MAX_STEP_BYTES}`);
  return { filename: path.basename(file), bytes: await readFile(file) };
}

export interface ImportedPart { filename: string; fingerprint: string; edges: number; railsAuthoring: boolean; joints: number; seams: number; parts: number; added: boolean }

/** OLP's Import STEP (main.ts importStepBytes, kind "weld"): every import after the first adds a part and makes it active. */
export async function importStep(ctx: Context, engine: Engine, file: string): Promise<ImportedPart> {
  const session = ctx.program;
  const { filename, bytes } = await readStep(file);
  await session.installRobots(engine);
  const partsBefore = engine.partsView().length;
  const reply = await ctx.olp.post(`${OLP_API}/cadquery/topology`, { operation: "extract_topology", filename, step_base64: bytes.toString("base64"), sample_count: 64 }, { timeoutMs: 300_000 });
  if (!reply.ok) throw new ProgramRefusal(`CadQuery refused ${filename} (HTTP ${reply.status}): ${refusalText(reply)}`, reply.status);
  await session.putSource(filename, bytes);
  const imported = engine.importPart(reply.body, filename);
  session.forgetPart();
  const rails = engine.view().railsAuthoring;
  let joints = 0;
  let seams = 0;
  if (rails) {
    const answered = await session.seam<{ joints: JointsDoc }>({ operation: "detect_joints", step_base64: bytes.toString("base64") });
    session.joints = { fingerprint: imported.fingerprint, doc: answered.joints };
    joints = answered.joints.weld_joints.length;
    seams = answered.joints.seams.length;
  }
  return { filename, fingerprint: imported.fingerprint, edges: imported.edges, railsAuthoring: rails, joints, seams, parts: engine.partsView().length, added: partsBefore > 0 };
}

/** The detected joints of the open program's part: from this session, or detected now from the kept STEP. */
export async function jointsOf(ctx: Context, engine: Engine, stepPath?: string): Promise<{ doc: JointsDoc; fingerprint: string; step: Uint8Array }> {
  const session = ctx.program;
  const part = engine.partView();
  if (!part.cad || !part.hasGeometry) throw new ProgramRefusal("The program has no part: import a STEP first");
  if (!part.railsAuthoring) throw new ProgramRefusal("This is a classic program; rails welds need a program started new (rails authoring is chosen at New program)");
  const { bytes } = await session.getSource(part.cad.fingerprint, stepPath);
  if (session.joints?.fingerprint !== part.cad.fingerprint) {
    const answered = await session.seam<{ joints: JointsDoc }>({ operation: "detect_joints", step_base64: Buffer.from(bytes).toString("base64") });
    session.joints = { fingerprint: part.cad.fingerprint, doc: answered.joints };
  }
  return { doc: session.joints.doc, fingerprint: part.cad.fingerprint, step: bytes };
}

function seamAndJoint(doc: JointsDoc, seamId: string): { seam: RailsSeam; joint: RailsJoint } {
  const seam = doc.seams.find((candidate) => candidate.id === seamId);
  if (!seam) throw new ProgramRefusal(`no seam ${seamId}; seams: ${doc.seams.map((s) => s.id).join(", ")}`);
  const joint = doc.weld_joints.find((candidate) => candidate.id === seam.weld_joint_ref);
  if (!joint) throw new ProgramRefusal(`seam ${seamId} names joint ${seam.weld_joint_ref}, which detection did not return`);
  return { seam, joint };
}

export interface SeamRow {
  seam_id: string; label: string; joint_id: string; type: string | null; weld_symbol: string | null; length_mm: number; closed: boolean | null;
  span: { start_s: number; end_s: number }; weld_rows: string[]; searched: { state: string; clear_percent: number; runs: number } | null;
}

export async function listSeams(ctx: Context, engine: Engine, stepPath?: string): Promise<SeamRow[]> {
  const { doc, fingerprint } = await jointsOf(ctx, engine, stepPath);
  const rows = engine.view().rows;
  return doc.seams.map((seam) => {
    const joint = doc.weld_joints.find((candidate) => candidate.id === seam.weld_joint_ref);
    const searched = ctx.program.seamPlans.get(`${fingerprint}:${seam.id}`);
    const welds = rows.filter((row) => row.id === `rails-${seam.id}` || row.id.startsWith(`rails-${seam.id}-run`)).map((row) => row.id);
    return {
      seam_id: seam.id, label: seam.label, joint_id: seam.weld_joint_ref, type: (joint?.type as string | undefined) ?? null, weld_symbol: (joint?.weld_symbol as string | undefined) ?? null,
      length_mm: Math.round((joint?.weld_line.length_m ?? 0) * 1000), closed: joint?.weld_line.closed ?? null, span: seam.span,
      weld_rows: welds,
      searched: searched ? { state: searched.report.state, clear_percent: Math.round(searched.plan.clear_fraction * 100), runs: searched.report.split.length } : null,
    };
  });
}

/** One seam's search, cached as rails mode caches it. */
async function searched(ctx: Context, engine: Engine, seamId: string, stepPath?: string, fresh = false): Promise<{ plan: SeamPlan; seam: RailsSeam; joint: RailsJoint; step: Uint8Array; fingerprint: string }> {
  const session = ctx.program;
  const { doc, fingerprint, step } = await jointsOf(ctx, engine, stepPath);
  const { seam, joint } = seamAndJoint(doc, seamId);
  const key = `${fingerprint}:${seamId}`;
  let plan = fresh ? undefined : session.seamPlans.get(key);
  if (!plan) {
    const rails = session.rails(engine);
    const answered = await engine.searchSeam(session.measurer(), step, seam, joint, { torch: rails.torchRequest, search: rails.searchKnobs, defaultStandoffMm: rails.defaultStandoffMm });
    if (!answered) throw new ProgramRefusal("the search was abandoned");
    plan = answered;
    session.seamPlans.set(key, plan);
  }
  return { plan, seam, joint, step, fingerprint };
}

export interface Coverage {
  seam_id: string; state: "clear" | "partial" | "blocked"; direction: string | null; clear_percent: number; standoff_mm: number;
  seam_length_mm: number; usable_mm: number; blocked_mm: number; runs: { run: number; span: { start_s: number; end_s: number }; fraction: number[] | undefined }[];
}

function coverage(seamId: string, plan: SeamPlan): Coverage {
  return {
    seam_id: seamId,
    state: plan.report.state,
    direction: plan.plan.direction ?? null,
    clear_percent: Math.round(plan.plan.clear_fraction * 100),
    standoff_mm: plan.plan.standoff_mm,
    seam_length_mm: Math.round(plan.report.seamLengthM * 1000),
    usable_mm: Math.round(plan.report.usableLengthM * 1000),
    blocked_mm: Math.round(plan.report.blockedLengthM * 1000),
    runs: plan.report.split.map((span, k) => ({ run: k, span, fraction: plan.report.usableRuns[k] })),
  };
}

/** Rails mode's angle search on one seam with the session's torch and search: clear, partial (clear runs) or blocked. */
export async function searchSeam(ctx: Context, engine: Engine, seamId: string, options: { fresh?: boolean; stepPath?: string } = {}): Promise<Coverage> {
  const { plan } = await searched(ctx, engine, seamId, options.stepPath, options.fresh ?? false);
  return coverage(seamId, plan);
}

/**
 * Accept a searched seam as weld rows (OLP's seamWeldNodeFor via decideWeld):
 * 'accept' as searched, or 'trim' to the longest clear run; a partial seam's
 * named runs (default all) each become their own weld, re-checked.
 */
export async function addWeld(ctx: Context, engine: Engine, seamId: string, options: { choice?: "accept" | "trim"; runs?: number[]; stepPath?: string } = {}): Promise<{ added: string[]; notes: string[] }> {
  const session = ctx.program;
  const choice = options.choice ?? "accept";
  const { plan, seam, joint, step, fingerprint } = await searched(ctx, engine, seamId, options.stepPath);
  const added: string[] = [];
  const notes: string[] = [];
  const write = (entry: Parameters<Engine["decideWeld"]>[0]) => {
    const decision = engine.decideWeld(entry, choice, fingerprint, engine.railsWeld(entry.id));
    if (!decision) {
      notes.push(`${entry.id}: nothing to trim to`);
      return;
    }
    engine.upsertWeld(decision.node);
    added.push(`${entry.id} (${decision.note})`);
  };
  if (engine.narrowsToRun(plan.report)) {
    const wanted = options.runs ?? plan.report.split.map((_, k) => k);
    for (const k of wanted) {
      if (k >= plan.report.split.length) throw new ProgramRefusal(`${seamId} has ${plan.report.split.length} clear run(s); run ${k} does not exist`);
      const piece = await engine.measureRunPiece(session.measurer(), step, seam, joint, plan, k, session.rails(engine).torchRequest);
      if (!piece) continue;
      write({ id: piece.id, seam, joint, span: piece.span, plan: piece.plan, summary: piece.summary, payload: piece.payload });
    }
  } else {
    if (options.runs) notes.push(`${seamId} is ${plan.report.state}, not split into runs: 'runs' ignored`);
    write({ id: `rails-${seam.id}`, seam, joint, span: { ...seam.span }, plan: plan.plan, summary: engine.summarise(plan.clearance, seam.span), payload: plan.payload });
  }
  if (!added.length) throw new ProgramRefusal(`no weld added for ${seamId}${notes.length ? `: ${notes.join("; ")}` : ""}`);
  return { added, notes };
}

/** A part by id becomes the active one (its STEP kept here or at stepPath). */
export async function switchPart(ctx: Context, engine: Engine, partId: string, stepPath?: string): Promise<{ part: PartRecordView; switched: boolean }> {
  const part = engine.partsView().find((candidate) => candidate.id === partId);
  if (!part) throw new ProgramRefusal(`no part ${partId}; parts: ${engine.partsView().map((p) => `${p.id} (${p.filename})`).join(", ") || "none"}`);
  await ctx.program.getSource(part.fingerprint, stepPath);
  await ctx.program.installRobots(engine);
  const switched = engine.switchPart(partId);
  if (switched) ctx.program.forgetPart();
  return { part, switched };
}

export interface FixtureArgs { action: FixtureEdit["action"]; fixtureId?: string; fields?: Partial<Record<FixtureField, number>>; path?: string }

/** The Stand card and workspace fixture actions, each one program edit. */
export async function editFixture(ctx: Context, engine: Engine, args: FixtureArgs): Promise<{ changed: boolean; id: string; note: string }> {
  const session = ctx.program;
  const needId = () => {
    if (!args.fixtureId) throw new ProgramRefusal(`'${args.action}' needs a fixture id (fixture list)`);
    return args.fixtureId;
  };
  /** main.ts importStandStep / importStepBytes kind "fixture": tessellate, keep the bytes, then the edit. */
  const stepFixture = async () => {
    if (!args.path) throw new ProgramRefusal(`'${args.action}' needs a .step/.stp path`);
    const { filename, bytes } = await readStep(args.path);
    const mesh = await session.tessellate(engine, filename, bytes);
    const kept = await session.putSource(filename, bytes);
    if (kept !== mesh.fingerprint.toLowerCase()) throw new ProgramRefusal(`CadQuery fingerprinted ${filename} as ${mesh.fingerprint.slice(0, 12)}…, not its sha256 ${kept.slice(0, 12)}…`);
    return { filename, fingerprint: kept, triangles: mesh.triangleCount };
  };
  let edit: FixtureEdit;
  let note = "";
  switch (args.action) {
    case "add_stand": edit = { action: "add_stand" }; break;
    case "edit": edit = { action: "edit", id: needId(), fields: args.fields ?? {} }; break;
    case "cell_default": edit = { action: "cell_default", id: needId() }; break;
    case "remove": edit = { action: "remove", id: needId() }; break;
    case "replace_with_step": {
      const id = needId();
      const step = await stepFixture();
      edit = { action: "replace_with_step", id, filename: step.filename, fingerprint: step.fingerprint };
      note = `${step.triangles.toLocaleString("en-US")} triangles`;
      break;
    }
    case "add_step_fixture": {
      const step = await stepFixture();
      edit = { action: "add_step_fixture", filename: step.filename, fingerprint: step.fingerprint };
      note = `${step.triangles.toLocaleString("en-US")} triangles; placed by its own origin, move it with 'edit'`;
      break;
    }
  }
  const done = engine.editFixture(edit);
  return { changed: done.changed, id: done.id, note };
}

// ---------------------------------------------------------------------------
// Plan, preview, export

/** Everything a plan needs that the program does not carry: the active part's STEP, the session's fitted gun, a mesh for every STEP fixture and inactive part. */
async function planInputs(ctx: Context, engine: Engine, stepPath?: string, fixturePaths?: string[]) {
  const session = ctx.program;
  if (fixturePaths?.length) await session.keepSources(fixturePaths);
  const part = engine.partView();
  let step = "";
  let filename = "part.step";
  if (part.hasGeometry && part.cad) {
    const source = await session.getSource(part.cad.fingerprint, stepPath);
    step = Buffer.from(source.bytes).toString("base64");
    filename = source.filename;
  }
  const { meshes } = await session.fixtureMeshes(engine);
  return { step, filename, tooling: session.rails(engine).tooling, meshes };
}

export interface PlanOptions { fromProgramStart?: boolean; repeat?: "once" | "count" | "loop"; freeSpaceBackend?: "bspline" | "curobo" | "legacy"; stepPath?: string; fixtureStepPaths?: string[] }

export interface PlanResult { elapsedS: number; identity: Record<string, unknown>; plan: PlanRecordView | null; trajectoryFile: string | null; startedFrom: "program start" | "measured pose" | "offline" }

/** POST /weld-plan as the Plan button sends it; the plan record is kept on the program and the bytes in the state dir. */
export async function planProgram(ctx: Context, engine: Engine, options: PlanOptions = {}): Promise<PlanResult> {
  const session = ctx.program;
  const { connected, cellModel } = await session.installRobots(engine);
  const mismatch = connected ? engine.modelMismatch(cellModel) : "";
  if (mismatch) throw new ProgramRefusal(mismatch);
  const inputs = await planInputs(ctx, engine, options.stepPath, options.fixtureStepPaths);
  const { revision, projectId, body } = await engine.planRequest(inputs.step, inputs.filename, {
    connected, fromProgramStart: options.fromProgramStart ?? false, closeLoop: (options.repeat ?? "once") !== "once", freeSpaceBackend: options.freeSpaceBackend ?? "bspline", tooling: inputs.tooling, meshes: inputs.meshes,
  });
  const started = Date.now();
  const reply = await ctx.olp.post(`${OLP_API}/weld-plan`, body, { timeoutMs: PLAN_TIMEOUT_MS });
  const elapsedS = (Date.now() - started) / 1000;
  if (!reply.ok) {
    const refusal = reply.body as { code?: string; error?: string; dense_error?: unknown };
    throw new ProgramRefusal(`planning refused after ${elapsedS.toFixed(1)} s (HTTP ${reply.status}${refusal.code ? `, ${refusal.code}` : ""}): ${refusal.error ?? refusalText(reply)}${refusal.dense_error ? ` · ${json(refusal.dense_error)}` : ""}`, reply.status, reply.body);
  }
  const answered = reply.body as { dense_blob_base64?: string; dense?: unknown };
  const digest = engine.applyPlan(answered, revision, projectId);
  const trajectoryFile = answered.dense_blob_base64 ? await session.keepTrajectory(digest, answered.dense_blob_base64) : null;
  return {
    elapsedS, identity: engine.loadBody(), plan: engine.planRecord(), trajectoryFile,
    startedFrom: options.fromProgramStart ? "program start" : connected ? "measured pose" : "offline",
  };
}

/** The kept plan's TCP path per segment through OLP's FK on the program's capture model. Moves nothing. */
export async function previewPath(ctx: Context, engine: Engine, options: { maxPoints?: number; trajectoryPath?: string } = {}): Promise<{ record: PlanRecordView; frame: string; segments: SegmentPath[]; [key: string]: unknown }> {
  const record = engine.planRecord();
  if (!record) throw new ProgramRefusal("The open program has no plan: plan it first");
  const blob = await ctx.program.trajectoryBytes(record.trajectoryDigest, options.trajectoryPath);
  const model = await ctx.olp.post(`${OLP_API}/robots/capture-model`, engine.programModelRequest());
  if (!model.ok) throw new ProgramRefusal(`OLP could not compile the program's robot model: ${refusalText(model)}`, model.status);
  const planned = engine.plannedTcpPath(blob, (model.body as { urdf: string }).urdf, options.maxPoints ?? 20);
  return { record, ...planned };
}

/** POST /weld-plan/export: the .weldplan container exactly as a plan would pack it. */
export async function exportWeldplan(ctx: Context, engine: Engine, destination: string, options: { fromProgramStart?: boolean; stepPath?: string; fixtureStepPaths?: string[] } = {}): Promise<{ path: string; sha256: string | null }> {
  const { connected } = await ctx.program.installRobots(engine);
  const inputs = await planInputs(ctx, engine, options.stepPath, options.fixtureStepPaths);
  const { body } = await engine.planRequest(inputs.step, inputs.filename, {
    connected, fromProgramStart: options.fromProgramStart ?? false, closeLoop: false, freeSpaceBackend: "bspline", tooling: inputs.tooling, meshes: inputs.meshes,
  });
  const response = await fetch(`${ctx.olp.baseUrl}${OLP_API}/weld-plan/export`, {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body), signal: AbortSignal.timeout(PLAN_TIMEOUT_MS),
  });
  if (!response.ok) throw new ProgramRefusal(`OLP refused the export (HTTP ${response.status}): ${(await response.text()).slice(0, 500)}`, response.status);
  await writeFile(destination, new Uint8Array(await response.arrayBuffer()));
  return { path: destination, sha256: response.headers.get("X-Weldplan-SHA256") };
}
