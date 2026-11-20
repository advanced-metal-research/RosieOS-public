// Verify and look: `rosie program check` (one pass/fail report on the plan
// and program), `rosie view scene` (OLP's own 3D renderer, several angles,
// PNG an agent opens itself) and `rosie report` (a self-contained HTML page
// with the snapshots, the spreadsheet and the checks). Agents make geometric
// mistakes they would see at a glance -- a part never moved from the work
// frame origin, a torch through the stand, a path through the table -- so
// every authoring recipe ends here. Nothing here moves the robot.
import { execFile } from "node:child_process";
import { mkdir, readFile, writeFile } from "node:fs/promises";
import path from "node:path";

import type { Engine } from "../../lib/engine.js";
import * as programs from "../../lib/programs.js";
import { summarize } from "../../lib/rdt.js";
import { renderSnapshots, type RenderResult, type ShotSpec, type SnapshotInput } from "../../lib/snapshot/index.js";
import { describeStatus } from "../../lib/status.js";
import { errorMessage, table } from "../../lib/text.js";
import { ok, refused, UsageError, type Command, type Outcome } from "../command.js";
import type { Env } from "../env.js";
import { onProgram, resolveRow, type Program } from "./common.js";

// ---------------------------------------------------------------- check

// Unverified bound: a MoveL's planned TCP may leave the straight chord by
// this much (sampling and the planner's blend tolerance, not a path error).
// The owner rule is that the TCP never leaves its commanded path; anything
// over half a millimetre on a MoveL is reported as a failure to look at.
const MOVEL_CHORD_TOLERANCE_MM = 0.5;
// Unverified: a segment's peak TCP speed may exceed its row's commanded TCP
// speed by 5 % (+1 mm/s) before it is reported.
const SPEED_TOLERANCE = 1.05;
const SPEED_SLACK_MM_S = 1;

type Level = "pass" | "warn" | "fail" | "skip";
export interface CheckItem { check: string; level: Level; detail: string; next?: string }
const MARK: Record<Level, string> = { pass: "ok  ", warn: "WARN", fail: "FAIL", skip: "--  " };

export async function checkProgram(p: Program, options: { trajectory?: string } = {}): Promise<CheckItem[]> {
  const items: CheckItem[] = [];
  const { engine, ctx } = p;
  const view = engine.view();
  items.push(view.problems.length
    ? { check: "program problems", level: "fail", detail: view.problems.join("; "), next: "rosie program show" }
    : { check: "program problems", level: "pass", detail: "OLP's document check finds none" });
  const errors = view.rows.filter((row) => row.statusClass === "error");
  const stale = view.rows.filter((row) => row.statusClass === "stale" && row.enabled);
  items.push(errors.length
    ? { check: "rows", level: "fail", detail: errors.map((r) => `#${r.number} ${r.id}: ${r.status}`).join("; ") }
    : stale.length
      ? { check: "rows", level: "warn", detail: `${stale.length} not compiled: ${stale.map((r) => `#${r.number} ${r.id}`).join(", ")}`, next: "rosie program plan" }
      : { check: "rows", level: "pass", detail: `${view.rows.length} rows, none in error` });
  let cellModel = "";
  let connected = false;
  try {
    ({ cellModel, connected } = await ctx.program.installRobots(engine));
  } catch {
    // offline: model checks skipped
  }
  const mismatch = connected ? engine.modelMismatch(cellModel) : "";
  items.push(mismatch
    ? { check: "robot model", level: "fail", detail: mismatch }
    : { check: "robot model", level: connected ? "pass" : "skip", detail: connected ? `program ${view.model} matches the selected cell` : "no cell selected: not compared" });
  const part = engine.partView();
  if (part.cad && part.placement) {
    const atOrigin = part.placement.offsetFromWorkFrameMm.every((v) => Math.abs(v) < 1e-6) && part.placement.rotationRpyDeg.every((v) => Math.abs(v) < 1e-6);
    items.push(atOrigin
      ? { check: "part placement", level: "warn", detail: `${part.cad.filename} is still where import put it (centred on the work frame origin): was it placed?`, next: "rosie part place --offset <x,y,z mm>; rosie view scene" }
      : { check: "part placement", level: "pass", detail: `${part.cad.filename} at [${programs.mm(part.placement.offsetFromWorkFrameMm)}] mm, rpy [${programs.mm(part.placement.rotationRpyDeg)}]°` });
  }
  const record = engine.planRecord();
  if (!record) {
    items.push({ check: "plan", level: "fail", detail: "no plan", next: "rosie program plan" });
    return items;
  }
  items.push(record.current
    ? { check: "plan", level: "pass", detail: `current: ${record.totalSampleCount} samples, ${record.totalDurationS.toFixed(1)} s, ${record.segments.length} segments` }
    : { check: "plan", level: "fail", detail: "STALE: the program was edited after it was planned", next: "rosie program plan" });
  let bytes: Uint8Array | null = null;
  try {
    bytes = await ctx.program.trajectoryBytes(record.trajectoryDigest, options.trajectory);
  } catch (error) {
    items.push({ check: "trajectory", level: "fail", detail: errorMessage(error), next: "rosie program plan" });
  }
  if (bytes) {
    const summary = summarize(bytes);
    if (!summary.valid) {
      items.push({ check: "trajectory ingest rules", level: "fail", detail: `${summary.rejection!.reason}: ${summary.rejection!.detail} (the daemon would refuse it)` });
    } else {
      items.push({ check: "trajectory ingest rules", level: "pass", detail: `every dense-daemon rule holds; digest ${summary.identity!.trajectory_digest.slice(0, 19)}…` });
      const over = summary.axes.filter((a) => a.in_mask && a.qd_utilisation > 1);
      const peak = summary.axes.filter((a) => a.in_mask).reduce((best, a) => (a.qd_utilisation > best.qd_utilisation ? a : best), summary.axes[0]!);
      items.push(over.length
        ? { check: "joint speed ceilings", level: "fail", detail: over.map((a) => `axis ${a.axis} at ${(a.qd_utilisation * 100).toFixed(0)} % of its ceiling`).join("; ") }
        : { check: "joint speed ceilings", level: "pass", detail: `peak ${(peak.qd_utilisation * 100).toFixed(0)} % of ceiling (axis ${peak.axis})` });
    }
  }
  try {
    const preview = await programs.previewPath(ctx, engine, { maxPoints: 2, trajectoryPath: options.trajectory });
    const rows = new Map(view.rows.map((row) => [row.id, row]));
    const bent: string[] = [];
    const fast: string[] = [];
    for (const segment of preview.segments) {
      const row = segment.nodeId ? rows.get(segment.nodeId) : undefined;
      if (row?.op === "move" && row.cells.motion === "linear" && segment.maxChordDeviationMm > MOVEL_CHORD_TOLERANCE_MM) {
        bent.push(`#${row.number} ${row.id} leaves its line by ${segment.maxChordDeviationMm.toFixed(2)} mm`);
      }
      const commanded = row?.op === "move" ? Number(row.cells.speed_mm_s) : NaN;
      if (Number.isFinite(commanded) && commanded > 0 && segment.peakTcpSpeedMmS > commanded * SPEED_TOLERANCE + SPEED_SLACK_MM_S) {
        fast.push(`#${row!.number} ${row!.id} peaks at ${segment.peakTcpSpeedMmS.toFixed(0)} mm/s, commanded ${commanded} mm/s`);
      }
    }
    items.push(bent.length
      ? { check: "MoveL straightness", level: "fail", detail: bent.join("; "), next: "rosie view scene; rosie program preview" }
      : { check: "MoveL straightness", level: "pass", detail: `every MoveL within ${MOVEL_CHORD_TOLERANCE_MM} mm of its line` });
    items.push(fast.length
      ? { check: "TCP speed", level: "warn", detail: fast.join("; ") }
      : { check: "TCP speed", level: "pass", detail: `peak ${Math.max(0, ...preview.segments.map((s) => s.peakTcpSpeedMmS)).toFixed(0)} mm/s; no move over its commanded speed` });
  } catch (error) {
    items.push({ check: "TCP path", level: "skip", detail: `preview unavailable: ${errorMessage(error)}` });
  }
  if (p.ws.isUnsaved) items.push({ check: "saved", level: "warn", detail: "the workspace has unsaved edits", next: "rosie program save (when the user asks)" });
  return items;
}

function checkTable(items: CheckItem[]): string {
  return table(["", "Check", "Detail"], items.map((c) => [MARK[c.level], c.check, c.detail]));
}

const check: Command<Env> = {
  path: ["program", "check"],
  summary: "Is this program right? One pass/fail report: problems, plan current, model, trajectory rules, MoveL straightness, speeds, placement",
  description: "Absolute checks, no motion: OLP's document problems and row errors; the plan is current; the program's robot matches the selected cell; the part was placed (not left at the import origin); the kept trajectory passes every dense-daemon ingest rule and no joint exceeds its speed ceiling; every MoveL's planned TCP stays within 0.5 mm of its line (unverified bound); no move's peak TCP speed exceeds its commanded speed by more than 5 %. Then look at it: rosie view scene.",
  options: [{ name: "trajectory", type: "string", description: ".rdt file if neither this machine nor the planner has the plan's bytes" }],
  examples: ["rosie program check", "rosie program check --json"],
  run: (env, args) => onProgram(env, {}, async (p) => {
    const items = await checkProgram(p, { trajectory: args.trajectory as string | undefined });
    const fails = items.filter((i) => i.level === "fail");
    const warns = items.filter((i) => i.level === "warn");
    const nexts = [...new Set([...fails, ...warns].map((i) => i.next).filter((n): n is string => Boolean(n)))];
    return (fails.length ? refused : ok)(fails.length ? `${fails.length} failing: ${fails.map((i) => i.check).join(", ")}` : `passes${warns.length ? ` with ${warns.length} warning(s): ${warns.map((i) => i.check).join(", ")}` : ""}`, {
      body: checkTable(items), data: items, next: nexts.length ? nexts : ["rosie view scene", "rosie run --sim"],
    });
  }),
};

// ---------------------------------------------------------------- scene

const CAMERAS = ["home", "part", "path", "front", "back", "left", "right", "top", "iso"] as const;

/** camera[@pose]: pose = start | end | 0..1 | row:<id or number>. */
function parseShots(engine: Engine, spec: string[]): ShotSpec[] {
  return spec.map((token) => {
    const [camera, pose] = token.split("@") as [string, string | undefined];
    if (!(CAMERAS as readonly string[]).includes(camera)) throw new UsageError(`camera "${camera}": one of ${CAMERAS.join(", ")}`);
    const shot: ShotSpec = { camera: camera as ShotSpec["camera"] };
    if (pose === undefined) return shot;
    if (pose === "start" || pose === "end") shot.pose = { sample: pose };
    else if (/^(0(\.\d+)?|1(\.0+)?)$/.test(pose)) shot.pose = { sample: Number(pose) };
    else if (pose.startsWith("row:")) shot.pose = { row: resolveRow(engine, pose.slice(4)) };
    else throw new UsageError(`pose "${pose}": start, end, a fraction 0..1, or row:<id or number>`);
    return shot;
  });
}

function defaultShots(engine: Engine): string[] {
  return engine.planRecord() ? ["home@start", "iso@start", "front@start", "top@start", "right@0.5", "iso@end"] : ["home", "iso", "front", "top"];
}

/** Everything OLP's renderer needs for the workspace's program: the catalogue, the export, the meshes kept (or tessellated now) and the plan's bytes. */
async function sceneInput(p: Program, options: { step?: string; plan?: boolean }): Promise<{ input: SnapshotInput; warnings: string[] }> {
  const { engine, ctx } = p;
  const warnings: string[] = [];
  const meshes: Record<string, unknown> = {};
  const fingerprints = new Map<string, string | undefined>();
  const part = engine.partView();
  if (part.cad && part.hasGeometry) fingerprints.set(part.cad.fingerprint.toLowerCase(), options.step);
  for (const need of engine.fixtureMeshNeeds()) fingerprints.set(need.fingerprint.toLowerCase(), undefined);
  for (const [fingerprint, step] of fingerprints) {
    try {
      meshes[fingerprint] = await ctx.program.meshWireFor(engine, fingerprint, step);
    } catch (error) {
      warnings.push(`no surface for ${fingerprint.slice(0, 12)}…: ${errorMessage(error)}`);
    }
  }
  let denseBase64: string | undefined;
  const record = engine.planRecord();
  if (record && options.plan !== false) {
    try {
      denseBase64 = Buffer.from(await ctx.program.trajectoryBytes(record.trajectoryDigest)).toString("base64");
    } catch (error) {
      warnings.push(`planned path not drawn: ${errorMessage(error)}`);
    }
  }
  return { input: { catalogue: ctx.program.catalogue, document: JSON.parse(engine.exportProject()), meshes, denseBase64 }, warnings };
}

function stamp(): string {
  return new Date().toISOString().replace(/[-:]/g, "").replace(/\..*$/, "").replace("T", "-");
}

function size(value: unknown): { width: number; height: number } {
  const match = /^(\d{3,4})x(\d{3,4})$/.exec(String(value ?? "1280x800"));
  if (!match) throw new UsageError("--size is WIDTHxHEIGHT, e.g. 1280x800");
  return { width: Number(match[1]), height: Number(match[2]) };
}

async function render(env: Env, p: Program, args: Record<string, unknown>, outDir: string): Promise<{ result: RenderResult; warnings: string[]; shots: string[] }> {
  const shots = (args.shots as string[] | undefined) ?? defaultShots(p.engine);
  const specs = parseShots(p.engine, shots);
  env.progress(`gathering meshes and plan for ${shots.length} view(s)…`);
  const { input, warnings } = await sceneInput(p, { step: args.step as string | undefined, plan: args.plan !== false });
  env.progress("rendering with OLP's renderer in headless Chrome…");
  const result = await renderSnapshots({ ...input, showLabels: args.labels !== false }, specs, { outDir, ...size(args.size), prefix: "scene" });
  return { result, warnings: [...warnings, ...result.warnings], shots };
}

const SCENE_OPTIONS = [
  { name: "shots", type: "strings", description: `camera[@pose] list; cameras ${CAMERAS.join(", ")}; pose start | end | 0..1 | row:<id>` },
  { name: "out", type: "string", description: "Output directory (default ~/.rosie/views/<workspace>-<time>)", short: "o" },
  { name: "size", type: "string", description: "WIDTHxHEIGHT", default: "1280x800" },
  { name: "step", type: "string", description: "The active part's STEP, if this machine never imported it" },
  { name: "plan", type: "boolean", description: "Draw the planned path (--no-plan hides it)", default: true },
  { name: "labels", type: "boolean", description: "Row labels on the path", default: true },
] as const;

const scene: Command<Env> = {
  path: ["view", "scene"],
  summary: "PNG snapshots of the program's 3D scene from OLP's own renderer: robot, part, fixtures, welds, waypoints, planned path",
  description: "Default views: home (the whole cell), iso, front and top at the plan's start, right at mid-plan and iso at the end (without a plan: home, iso, front, top). Open the PNGs (an agent reads them as images) and check: is the part on the table/stand where it should be, does the torch reach every weld, does any path pass through a fixture, is the robot pose sane? ~10-20 s cold.",
  options: [...SCENE_OPTIONS],
  examples: ["rosie view scene", "rosie view scene --shots iso@start,front@row:rails-seam_0001,top@end", "rosie view scene --shots part --size 1600x1000"],
  run: (env, args) => onProgram(env, {}, async (p) => {
    const outDir = path.resolve(String(args.out ?? path.join(env.home, "views", `${p.ws.name}-${stamp()}`)));
    const { result, warnings, shots } = await render(env, p, args, outDir);
    const s = result.scene;
    return ok(`${result.files.length} view(s) of ${p.engine.view().name} written — open them and look before running anything`, {
      body: [
        `scene: ${s.modelId} · part ${s.partTriangles ? `${s.partTriangles} triangles` : "NOT DRAWN"} · ${s.fixtures} fixture(s) · ${s.welds} weld(s) · path ${s.pathPoints} points · planned ${s.plannedPathPoints} points${s.plannedStale ? " (STALE plan)" : ""}`,
        s.boundsMm.part ? `part bounds (world, mm): [${programs.mm(s.boundsMm.part[0])}] .. [${programs.mm(s.boundsMm.part[1])}]` : "",
        ...result.files.map((f, i) => `${shots[i]}: ${f.path}${f.notes?.length ? ` (${f.notes.join("; ")})` : ""}`),
        ...warnings.map((w) => `warning: ${w}`),
      ].filter(Boolean).join("\n"),
      data: { scene: s, files: result.files, warnings, timings_ms: result.timingsMs },
      artifacts: result.files.map((f) => f.path),
      next: ["rosie program check", "rosie report program"],
    });
  }),
};

// ---------------------------------------------------------------- report

function escape(text: unknown): string {
  return String(text ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]!);
}

function htmlTable(headers: string[], rows: unknown[][]): string {
  return `<table><thead><tr>${headers.map((h) => `<th>${escape(h)}</th>`).join("")}</tr></thead><tbody>${rows.map((row) => `<tr>${row.map((cell) => `<td>${escape(cell)}</td>`).join("")}</tr>`).join("")}</tbody></table>`;
}

function page(title: string, sections: string[]): string {
  return `<!doctype html><html lang="en"><head><meta charset="utf-8"><title>${escape(title)}</title><style>
body{font:14px/1.45 -apple-system,system-ui,sans-serif;margin:24px auto;max-width:1400px;padding:0 20px;color:#1b1f24;background:#fafbfc}
h1{font-size:22px;margin:0 0 4px}h2{font-size:16px;margin:28px 0 8px;border-bottom:1px solid #d8dee4;padding-bottom:4px}
.meta{color:#57606a}table{border-collapse:collapse;width:100%;font-size:13px;background:#fff}th,td{border:1px solid #d8dee4;padding:4px 8px;text-align:left;vertical-align:top}
th{background:#f0f3f6}.views{display:grid;grid-template-columns:repeat(auto-fill,minmax(420px,1fr));gap:12px}figure{margin:0;background:#fff;border:1px solid #d8dee4}
figure img{width:100%;display:block}figcaption{padding:4px 8px;font-size:12px;color:#57606a}.pass{color:#1a7f37}.warn{color:#9a6700}.fail{color:#cf222e;font-weight:600}
.verdict{font-size:16px;padding:8px 12px;border-radius:6px;background:#fff;border:1px solid #d8dee4}pre{background:#fff;border:1px solid #d8dee4;padding:8px;overflow:auto}
</style></head><body>${sections.join("\n")}</body></html>`;
}

function openFile(file: string): void {
  const opener = process.platform === "darwin" ? "open" : process.platform === "win32" ? "explorer" : "xdg-open";
  execFile(opener, [file], () => undefined);
}

const reportProgram: Command<Env> = {
  path: ["report", "program"],
  summary: "A self-contained HTML report: verdict, 3D views, checks, spreadsheet, planned path — for a person to review",
  options: [...SCENE_OPTIONS, { name: "open", type: "boolean", description: "Open it in the browser" }, { name: "no_scene", type: "boolean", description: "Skip the 3D views" }],
  examples: ["rosie report program --open", "rosie report program --out ~/reports/bracket"],
  run: (env, args) => onProgram(env, {}, async (p) => {
    const outDir = path.resolve(String(args.out ?? path.join(env.home, "views", `${p.ws.name}-${stamp()}`)));
    await mkdir(outDir, { recursive: true });
    env.progress("checking…");
    const items = await checkProgram(p);
    let views = "";
    let warnings: string[] = [];
    if (!args.no_scene) {
      try {
        const rendered = await render(env, p, args, outDir);
        warnings = rendered.warnings;
        const figures = await Promise.all(rendered.result.files.map(async (f, i) => `<figure><img alt="${escape(rendered.shots[i])}" src="data:image/png;base64,${(await readFile(f.path)).toString("base64")}"><figcaption>${escape(rendered.shots[i])}${f.notes?.length ? ` · ${escape(f.notes.join("; "))}` : ""}</figcaption></figure>`));
        views = `<h2>3D views (OLP's renderer)</h2><div class="views">${figures.join("")}</div>${warnings.length ? `<p class="warn">${warnings.map(escape).join("<br>")}</p>` : ""}`;
      } catch (error) {
        views = `<h2>3D views</h2><p class="fail">not rendered: ${escape(errorMessage(error))}</p>`;
      }
    }
    let preview = "";
    try {
      const planned = await programs.previewPath(p.ctx, p.engine, { maxPoints: 2 });
      preview = `<h2>Planned TCP path</h2>${htmlTable(["#", "Kind", "Row", "Duration s", "Path mm", "Off chord mm", "Peak mm/s"], planned.segments.map((s) => [s.index, s.kind, s.nodeId ?? s.sourceId, s.durationS.toFixed(2), s.lengthMm.toFixed(1), s.maxChordDeviationMm.toFixed(2), s.peakTcpSpeedMmS.toFixed(0)]))}`;
    } catch {
      preview = "";
    }
    const view = p.engine.view();
    const fails = items.filter((i) => i.level === "fail");
    const verdict = fails.length ? `${fails.length} failing check(s): ${fails.map((i) => i.check).join(", ")}` : "all checks pass";
    const html = page(`${view.name} — rosie report`, [
      `<h1>${escape(view.name)}</h1><p class="meta">${escape(view.model)} · revision ${view.revision} · workspace ${escape(p.ws.name)} · ${new Date().toISOString()}</p>`,
      `<p class="verdict ${fails.length ? "fail" : "pass"}">${escape(verdict)}</p>`,
      views,
      `<h2>Checks</h2>${htmlTable(["", "Check", "Detail"], items.map((i) => [i.level.toUpperCase(), i.check, i.detail]))}`.replace(/<td>(PASS|WARN|FAIL)<\/td>/g, (_, level: string) => `<td class="${level.toLowerCase()}">${level}</td>`),
      `<h2>Program</h2>${htmlTable(["#", "Row", "Op", "Label", "Status", "On"], view.rows.map((r) => [r.number, r.id, r.op, r.label, r.status, r.enabled ? "yes" : "no"]))}`,
      preview,
      `<h2>Spreadsheet (text)</h2><pre>${escape(programs.spreadsheet(p.engine, p.ctx.program))}</pre>`,
    ]);
    const file = path.join(outDir, "report.html");
    await writeFile(file, html);
    if (args.open) openFile(file);
    return (fails.length ? refused : ok)(`report written: ${verdict}`, { artifacts: [file], data: { checks: items, warnings }, body: checkTable(items) });
  }),
};

const reportCell: Command<Env> = {
  path: ["report", "cell"],
  summary: "A self-contained HTML page of the cell's state: control, maintenance, machine status, rt-core faults",
  options: [{ name: "out", type: "string", description: "Output file (default ~/.rosie/views/cell-<time>/report.html)", short: "o" }, { name: "open", type: "boolean", description: "Open it" }],
  async run(env, args): Promise<Outcome> {
    const ctx = await env.ctx();
    const status = await ctx.olp.status();
    const control = await ctx.olp.get("/api/offline-programming/v1/control").then((r) => r.body).catch(() => null);
    const maintenance = await ctx.olp.get("/api/offline-programming/v1/maintenance").then((r) => r.body).catch(() => null);
    const file = path.resolve(String(args.out ?? path.join(env.home, "views", `cell-${stamp()}`, "report.html")));
    await mkdir(path.dirname(file), { recursive: true });
    const t = status.target;
    await writeFile(file, page("cell — rosie report", [
      `<h1>${escape(t ? `${t.cell_id} (${t.simulation ? "SIMULATION" : t.simulation === false ? "REAL ROBOT" : t.backend})` : "no cell selected")}</h1><p class="meta">${escape(ctx.olp.baseUrl)} · ${new Date().toISOString()}</p>`,
      `<h2>Machine status</h2><pre>${escape(describeStatus(status))}</pre>`,
      `<h2>Control</h2><pre>${escape(JSON.stringify(control, null, 2))}</pre>`,
      `<h2>Maintenance</h2><pre>${escape(JSON.stringify(maintenance, null, 2))}</pre>`,
    ]));
    if (args.open) openFile(file);
    return ok(`cell report written`, { artifacts: [file], next: ["rosie cell diagnose"] });
  },
};

export const commands: Command<Env>[] = [check, scene, reportProgram, reportCell];
