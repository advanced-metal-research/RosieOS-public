// Render PNG snapshots of one OLP program with OLP's own three.js renderer:
// serve the built snapshot page (scripts/build-snapshot.mjs), open it in
// headless Chrome, `olpSnapshot.load()` the program, `olpSnapshot.shot()`
// each view, write each PNG.
//
// A snapshot is evidence an agent acts on, so it fails loudly rather than
// return a picture of something else: a page exception, a console error (a
// robot mesh that 404s is one), or a load the page refuses (`ok: false`)
// throws with the page's own words. Things drawn incompletely but honestly
// (a part with no mesh, a stale plan) come back as warnings.
import { existsSync } from "node:fs";
import { mkdir, writeFile } from "node:fs/promises";
import path from "node:path";
import { setTimeout as sleep } from "node:timers/promises";
import { fileURLToPath } from "node:url";

import { withChrome, type CdpClient } from "./chrome.js";
import { startStaticServer } from "./server.js";

/** What `olpSnapshot.load()` draws. */
export interface SnapshotInput {
  /** `GET /api/offline-programming/v1/robots`, as the server answered. */
  catalogue: unknown;
  /** The program: a bare project (a cell store document) or an export
   *  envelope `{schema: "offline-programming.project-export.v1", exportedAt, project}`. */
  document: unknown;
  /** Lowercase STEP fingerprint -> CadQuery tessellate response body (the
   *  wire `POST /cadquery/topology {operation: "tessellate"}` answers). */
  meshes?: Record<string, unknown>;
  /** robot.v4.dense-joint-trajectory.v1 bytes of the program's plan, base64. */
  denseBase64?: string;
  /** Row-number labels on the path (default true). */
  showLabels?: boolean;
}

export type NamedCamera = "home" | "part" | "path" | "front" | "back" | "left" | "right" | "top" | "iso";
export type SnapshotCamera = NamedCamera | { positionM: number[]; targetM: number[]; fovDeg?: number };

/** Joints in the model's pose order (radians), a point along the plan
 *  (fraction of its duration), or a row / node id: a taught move's joints or
 *  the first planned sample of that weld or row. */
export type SnapshotPose = number[] | { sample: "start" | "end" | number } | { row: string };

export interface ShotSpec {
  pose?: SnapshotPose;
  /** home: OLP's orbit home; part / path: OLP's Frame part / Frame path;
   *  front back left right top iso: the union of part, path, plan and welds
   *  framed from +X, -X, +Y, -Y, +Z and (1,-1,0.75), in the cell's world. */
  camera: SnapshotCamera;
  /** The TCP frame triads (default true). */
  tcpFrame?: boolean;
  /** Everything but the part translucent (default false). */
  partFocus?: boolean;
  /** The planned TCP path (default true). */
  plannedPath?: boolean;
}

export interface SceneInfo {
  modelId: string;
  /** Triangles of the drawn part surface; 0 when no mesh matched the part. */
  partTriangles: number;
  /** Workcell fixtures and inactive parts drawn around the part. */
  fixtures: number;
  /** Authored path points (the spreadsheet rows' targets). */
  pathPoints: number;
  /** Planned TCP points traced from the plan. */
  plannedPathPoints: number;
  /** The plan drawn is not this revision's (or this robot's). */
  plannedStale: boolean;
  /** Weld lines drawn (seam and hand-placed welds). */
  welds: number;
  /** World-frame bounds in millimetres, [min, max]. */
  boundsMm: { part?: [number[], number[]]; path?: [number[], number[]] };
}

export interface RenderOptions {
  /** Where the PNGs are written (created if missing). */
  outDir: string;
  width?: number;
  height?: number;
  /** File name prefix (default "snapshot-"). */
  prefix?: string;
  /** Budget for the whole run, browser start to last PNG (default 120 s). */
  timeoutMs?: number;
  /** The built page (default ROSIE_SNAPSHOT_OUT or cli/v1/dist/snapshot). */
  snapshotDir?: string;
  /** Browser executable (default ROSIE_CHROME, else an installed Chrome/Edge). */
  chrome?: string;
}

export interface RenderedFile {
  path: string;
  /** The camera's name, or "custom". */
  camera: string;
  /** "current", "joints", "start", "end", "t0.50" or "row-<id>". */
  pose: string;
  jointOrder: string[];
  jointsRad: number[];
  cameraPositionM: number[];
  cameraTargetM: number[];
  /** How the page resolved the shot (which plan sample a row pose came from,
   *  a camera that fell back). */
  notes: string[];
}

export interface RenderResult {
  files: RenderedFile[];
  scene: SceneInfo;
  warnings: string[];
  /** Wall time per phase: browser start, page open, program load (robot
   *  meshes, part, plan trace), all shots. */
  timingsMs: { browser: number; page: number; load: number; shots: number };
}

/** `promise`, or a rejection after `ms`; the timer never outlives the race
 *  (a dangling one holds the process open for the whole budget). */
async function withTimeout<T>(promise: Promise<T>, ms: number, message: string): Promise<T> {
  let timer: NodeJS.Timeout | undefined;
  try {
    return await Promise.race([promise, new Promise<never>((_, reject) => {
      timer = setTimeout(() => reject(new Error(message)), ms);
    })]);
  } finally {
    clearTimeout(timer);
  }
}

/** The CLI package root: the nearest ancestor holding package.json, whether
 *  this runs compiled (dist/src/lib/snapshot) or from source. */
function cliRoot(): string {
  let dir = path.dirname(fileURLToPath(import.meta.url));
  while (!existsSync(path.join(dir, "package.json"))) {
    const parent = path.dirname(dir);
    if (parent === dir) throw new Error("cannot find the rosie CLI package root");
    dir = parent;
  }
  return dir;
}

export function defaultSnapshotDir(env: NodeJS.ProcessEnv = process.env): string {
  return path.resolve(env.ROSIE_SNAPSHOT_OUT ?? path.join(cliRoot(), "dist", "snapshot"));
}

function poseLabel(pose: SnapshotPose | undefined): string {
  if (pose === undefined) return "current";
  if (Array.isArray(pose)) return "joints";
  if ("sample" in pose) return typeof pose.sample === "number" ? `t${pose.sample.toFixed(2)}` : pose.sample;
  return `row-${pose.row.replace(/[^A-Za-z0-9_-]/g, "").slice(0, 12)}`;
}

/** Collects everything the page says went wrong, from any of the three
 *  places Chrome reports it. */
function watchPage(cdp: CdpClient, sessionId: string): { take(): string[] } {
  const problems: string[] = [];
  const mine = (id: string | undefined) => id === sessionId;
  cdp.on("Runtime.exceptionThrown", (params, id) => {
    if (!mine(id)) return;
    const details = params.exceptionDetails as { text?: string; exception?: { description?: string } } | undefined;
    problems.push(`page exception: ${details?.exception?.description ?? details?.text ?? "unknown"}`);
  });
  cdp.on("Runtime.consoleAPICalled", (params, id) => {
    if (!mine(id) || (params.type !== "error" && params.type !== "assert")) return;
    const args = (params.args as { value?: unknown; description?: string }[] | undefined) ?? [];
    problems.push(`console.${String(params.type)}: ${args.map((arg) => arg.description ?? (typeof arg.value === "string" ? arg.value : JSON.stringify(arg.value))).join(" ")}`);
  });
  cdp.on("Log.entryAdded", (params, id) => {
    if (!mine(id)) return;
    const entry = params.entry as { level?: string; text?: string; url?: string } | undefined;
    if (entry?.level === "error") problems.push(`browser: ${entry.text ?? ""}${entry.url ? ` (${entry.url})` : ""}`);
  });
  return { take: () => problems.splice(0) };
}

async function evaluate<T>(cdp: CdpClient, sessionId: string, expression: string, timeoutMs: number): Promise<T> {
  const answer = await cdp.send<{ result?: { value?: unknown }; exceptionDetails?: { text?: string; exception?: { description?: string } } }>(
    "Runtime.evaluate",
    { expression, awaitPromise: true, returnByValue: true },
    sessionId,
    timeoutMs,
  );
  if (answer.exceptionDetails) {
    // The page's message, without its minified stack.
    const description = answer.exceptionDetails.exception?.description ?? answer.exceptionDetails.text ?? "evaluation failed";
    throw new Error(description.split("\n    at ")[0]!.replace(/^Error: /, ""));
  }
  return answer.result?.value as T;
}

/**
 * Draw `input` once and write one PNG per shot into `options.outDir`, in
 * order. Throws on any page error, a refused load, or the time budget.
 */
export async function renderSnapshots(input: SnapshotInput, shots: readonly ShotSpec[], options: RenderOptions): Promise<RenderResult> {
  const snapshotDir = path.resolve(options.snapshotDir ?? defaultSnapshotDir());
  if (!existsSync(path.join(snapshotDir, "snapshot.html"))) {
    throw new Error(`the snapshot page is not built (${snapshotDir}/snapshot.html): run \`node cli/v1/scripts/build-snapshot.mjs\``);
  }
  if (shots.length === 0) throw new Error("renderSnapshots: no shots requested");
  const width = Math.round(options.width ?? 1280);
  const height = Math.round(options.height ?? 800);
  if (!(width >= 64 && width <= 4096 && height >= 64 && height <= 4096)) throw new Error(`snapshot size ${width}x${height} is outside 64..4096`);
  const prefix = options.prefix ?? "snapshot-";
  const deadline = Date.now() + (options.timeoutMs ?? 120_000);
  const remaining = () => {
    const left = deadline - Date.now();
    if (left <= 0) throw new Error(`snapshot render exceeded ${options.timeoutMs ?? 120_000} ms`);
    return left;
  };
  await mkdir(options.outDir, { recursive: true });

  const server = await startStaticServer(snapshotDir);
  const started = Date.now();
  let mark = started;
  const lap = () => {
    const now = Date.now();
    const elapsed = now - mark;
    mark = now;
    return elapsed;
  };
  try {
    return await withChrome(async ({ cdp }) => {
      const browserMs = lap();
      const { targetId } = await cdp.send<{ targetId: string }>("Target.createTarget", { url: "about:blank" }, undefined, remaining());
      const { sessionId } = await cdp.send<{ sessionId: string }>("Target.attachToTarget", { targetId, flatten: true }, undefined, remaining());
      const page = watchPage(cdp, sessionId);
      const fail = (step: string) => {
        const problems = page.take();
        if (problems.length > 0) throw new Error(`snapshot ${step} failed:\n  ${problems.join("\n  ")}`);
      };
      await cdp.send("Runtime.enable", {}, sessionId, remaining());
      await cdp.send("Log.enable", {}, sessionId, remaining());
      await cdp.send("Page.enable", {}, sessionId, remaining());
      await cdp.send("Emulation.setDeviceMetricsOverride", { width, height, deviceScaleFactor: 1, mobile: false }, sessionId, remaining());

      const loaded = new Promise<void>((resolve) => {
        const off = cdp.on("Page.loadEventFired", (_, id) => {
          if (id === sessionId) {
            off();
            resolve();
          }
        });
      });
      const navigation = await cdp.send<{ errorText?: string }>("Page.navigate", { url: `${server.url}snapshot.html` }, sessionId, remaining());
      if (navigation.errorText) throw new Error(`cannot open the snapshot page: ${navigation.errorText}`);
      await withTimeout(loaded, remaining(), "the snapshot page did not load");
      // The module script runs after load; it marks the stage once
      // window.olpSnapshot exists.
      for (;;) {
        const state = await evaluate<string>(cdp, sessionId, `document.getElementById("workstation-stage")?.dataset.snapshotState ?? ""`, remaining());
        if (state === "idle") break;
        fail("page start");
        if (state !== "booting") throw new Error(`the snapshot page is in state "${state}"`);
        await sleep(25);
        remaining();
      }

      const pageMs = lap();
      const load = await evaluate<{ ok: boolean; errors: string[]; warnings: string[]; scene: SceneInfo }>(
        cdp,
        sessionId,
        `window.olpSnapshot.load(${JSON.stringify({ ...input, width, height })})`,
        remaining(),
      );
      fail("load");
      if (!load.ok) throw new Error(`OLP refused to draw the program:\n  ${load.errors.join("\n  ")}`);
      const warnings = [...load.warnings];
      const loadMs = lap();

      const files: RenderedFile[] = [];
      for (const [index, spec] of shots.entries()) {
        const shot = await evaluate<{ dataUrl: string; pose: { jointOrder: string[]; jointsRad: number[] }; camera: { positionM: number[]; targetM: number[] }; notes: string[] }>(
          cdp,
          sessionId,
          `window.olpSnapshot.shot(${JSON.stringify(spec)})`,
          remaining(),
        );
        fail(`shot ${index + 1}`);
        const camera = typeof spec.camera === "string" ? spec.camera : "custom";
        const pose = poseLabel(spec.pose);
        const file = path.join(options.outDir, `${prefix}${String(index + 1).padStart(2, "0")}-${camera}-${pose}.png`);
        const comma = shot.dataUrl.indexOf(",");
        if (!shot.dataUrl.startsWith("data:image/png") || comma < 0) throw new Error(`shot ${index + 1} returned no PNG`);
        await writeFile(file, Buffer.from(shot.dataUrl.slice(comma + 1), "base64"));
        files.push({
          path: file, camera, pose,
          jointOrder: shot.pose.jointOrder, jointsRad: shot.pose.jointsRad,
          cameraPositionM: shot.camera.positionM, cameraTargetM: shot.camera.targetM, notes: shot.notes,
        });
      }
      return { files, scene: load.scene, warnings, timingsMs: { browser: browserMs, page: pageMs, load: loadMs, shots: lap() } };
    }, { executable: options.chrome, timeoutMs: Math.min(30_000, remaining()) });
  } finally {
    await server.close();
  }
}
