import assert from "node:assert/strict";
import { existsSync } from "node:fs";
import { mkdir, writeFile } from "node:fs/promises";
import { spawn, type ChildProcess } from "node:child_process";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { setTimeout as delay } from "node:timers/promises";

import puppeteer, { type Page } from "puppeteer-core";
import { PuppeteerScreenRecorder } from "puppeteer-screen-recorder";

type SeamTourState = {
  modelId: string | null;
  edgeCount: number;
  selectedEdgeId: string | null;
  savedSeams: Array<{
    id: string;
    edgeId: string;
    startS: number;
    endS: number;
    reversed: boolean;
    points: unknown[];
  }>;
  tourStep: number;
  status: string;
  seamJSON: string;
  tourCurrentTitle: string;
  tourLogItems: string[];
  robotStatus: string;
  pathStatus: string;
  weldStatus: string;
  weldProgress: number;
  weldTorchDistanceToPathMm: number | null;
};

type MotionPlanSample = {
  progress: number | null;
  torchPos: number[] | null;
  torchDistToPathMm: number | null;
  motionGroupPresent: boolean;
  weldRuns: number;
  travelRuns: number;
  gizmos: number;
  robotJointsAnimating: boolean;
  robotJointAnglesRad: number[] | null;
  collisionSampleCount: number;
  collisionMarkers: number;
  collisionStatusText: string;
  collisionOutlinedLinks: string[];
  collisionOutlineObjects: number;
};

type EdgeGeometryTourState = {
  selectedEdgeId: string | null;
  caption: string;
  result: string;
  modeText: string;
  billboards: string[];
  sceneCounts: {
    frameTourGroup: number;
    torch: number;
    trace: number;
    liveGizmo: number;
  };
  torchMoveMm: number | null;
  tourLogTextPresent: boolean;
};

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const uiRoot = findUIRoot(scriptDir);
const artifactDir = path.join(uiRoot, "test", "screenshots");
const baseURL = process.env.CADQUERY_SEAM_TEST_URL || "http://127.0.0.1:5284/cadquery/v1/ui/seam";
const targetURL = new URL(baseURL);
const testPort = targetURL.port || "5284";
const videoPath = path.join(artifactDir, "cadquery-seam-tour.mp4");

const consoleErrors: string[] = [];
const pageErrors: string[] = [];
const failedRequests: string[] = [];
let viteProcess: ChildProcess | null = null;

await mkdir(artifactDir, { recursive: true });
if (!(await isHTTPReady(baseURL))) {
  viteProcess = startVite();
}
await waitForHTTP(baseURL, 45_000);

const browser = await puppeteer.launch({
  executablePath: await findChrome(),
  headless: process.env.CADQUERY_SEAM_TEST_HEADED !== "1",
  args: ["--no-sandbox", "--disable-dev-shm-usage"],
  defaultViewport: { width: 1440, height: 980, deviceScaleFactor: 1 },
});

try {
  const page = await browser.newPage();
  hookBrowserLogs(page);

  const recorder = new PuppeteerScreenRecorder(page as never, {
    followNewTab: false,
    fps: 24,
    videoFrame: { width: 1440, height: 980 },
    aspectRatio: "16:9",
    videoCrf: 22,
    videoCodec: "libx264",
    videoPreset: "ultrafast",
  });

  await page.goto(targetURL.toString(), { waitUntil: "networkidle0", timeout: 90_000 });
  await recorder.start(videoPath);
  await page.waitForSelector("#quick-tour-button", { timeout: 30_000 });
  await page.evaluate(() => document.querySelector<HTMLButtonElement>("#quick-tour-button")?.click());

  // Motion-plan playback: sample while the tour is still on this page (the
  // visible tour button always ends by navigating to Edge Geometry, so
  // sampling after completion would race the hand-off).
  await page.waitForFunction(
    () => {
      const runtime = (window as typeof window & {
        __cadquerySeamRuntime?: {
          selectedEdgeId: string | null;
          scene?: { getObjectByName?: (name: string) => unknown };
        };
      }).__cadquerySeamRuntime;
      return Boolean(
        runtime?.selectedEdgeId === "part_3:wire_00034" &&
          runtime.scene?.getObjectByName?.("cadquery-tesseract-motion-plan") &&
          runtime.scene?.getObjectByName?.("weld-45deg-torch"),
      );
    },
    { timeout: 150_000 },
  );
  const motionSampleA = await sampleMotionPlan(page);
  await delay(2_500);
  const motionSampleB = await sampleMotionPlan(page);
  assert.ok(
    motionSampleB.motionGroupPresent && motionSampleB.weldRuns >= 1 && motionSampleB.travelRuns >= 1 && motionSampleB.gizmos > 0,
    `Tesseract motion plan overlay should render weld runs, travel runs, and gizmos: ${JSON.stringify(motionSampleB)}`,
  );
  assert.ok(
    motionSampleA.progress !== null && motionSampleB.progress !== null && motionSampleB.progress !== motionSampleA.progress,
    `Motion-plan playback progress should advance, got ${motionSampleA.progress} -> ${motionSampleB.progress}`,
  );
  const torchMovedMm =
    motionSampleA.torchPos && motionSampleB.torchPos
      ? Math.hypot(
          motionSampleB.torchPos[0] - motionSampleA.torchPos[0],
          motionSampleB.torchPos[1] - motionSampleA.torchPos[1],
          motionSampleB.torchPos[2] - motionSampleA.torchPos[2],
        )
      : 0;
  assert.ok(torchMovedMm > 1, `Weld torch should move through the motion plan, moved ${torchMovedMm.toFixed(2)}mm`);
  assert.ok(
    typeof motionSampleB.torchDistToPathMm === "number" && motionSampleB.torchDistToPathMm < 12,
    `Weld torch should stay on the planned path during playback, got ${motionSampleB.torchDistToPathMm}`,
  );
  // URDF robot animation: the backend IK joint trajectory must be driving
  // the rosie_1400_v3 arm, and the joint angles must change over time.
  assert.ok(
    motionSampleA.robotJointsAnimating && motionSampleB.robotJointsAnimating,
    `URDF robot joints should animate the plan: ${JSON.stringify({ a: motionSampleA.robotJointsAnimating, b: motionSampleB.robotJointsAnimating })}`,
  );
  assert.ok(
    Array.isArray(motionSampleA.robotJointAnglesRad) &&
      Array.isArray(motionSampleB.robotJointAnglesRad) &&
      motionSampleA.robotJointAnglesRad.length === 6 &&
      motionSampleB.robotJointAnglesRad.length === 6,
    `Runtime should expose 6 joint angles: ${JSON.stringify({ a: motionSampleA.robotJointAnglesRad, b: motionSampleB.robotJointAnglesRad })}`,
  );
  const jointDeltaRad = Math.max(
    ...motionSampleA.robotJointAnglesRad!.map((value, index) => Math.abs(motionSampleB.robotJointAnglesRad![index] - value)),
  );
  assert.ok(jointDeltaRad > 1e-4, `URDF joint angles should change during playback, max delta ${jointDeltaRad}`);
  // Collision detection (robot/v4 pattern): the HUD reports the plan's
  // collision state and contact waypoints render red overlay markers.
  assert.ok(
    /collision: (clear|\d+\/\d+ contact waypoints|unavailable|skipped|not checked)|COLLISION .+ depth=[\d.]+mm/.test(
      motionSampleB.collisionStatusText,
    ),
    `Collision HUD should report plan collision state, got ${JSON.stringify(motionSampleB.collisionStatusText)}`,
  );
  assert.equal(
    motionSampleB.collisionMarkers,
    motionSampleB.collisionSampleCount,
    `One red marker per contact waypoint: markers=${motionSampleB.collisionMarkers} samples=${motionSampleB.collisionSampleCount}`,
  );
  if (motionSampleB.collisionSampleCount > 0) {
    // Both sides of the colliding geometry must be outlined in red for the
    // whole tour (arm links AND the environment link they touch).
    assert.ok(
      motionSampleB.collisionOutlinedLinks.length >= 2,
      `Both collision pair members should be outlined, got ${JSON.stringify(motionSampleB.collisionOutlinedLinks)}`,
    );
    assert.ok(
      motionSampleB.collisionOutlineObjects > 0,
      `Red collision outline lines should exist in the scene, got ${motionSampleB.collisionOutlineObjects}`,
    );
  }

  await page.waitForFunction(
    () => {
      const runtime = (window as typeof window & {
        __cadquerySeamRuntime?: {
          model: { model_id: string; edges: unknown[] } | null;
          selectedEdgeId: string | null;
          savedSeams: Array<{ edgeId: string; points: unknown[] }>;
          weldTorchDistanceToPathMm?: number | null;
        };
      }).__cadquerySeamRuntime;
      const status = document.querySelector<HTMLElement>("#status")?.innerText ?? "";
      return Boolean(
        runtime?.model?.edges?.length &&
          runtime.selectedEdgeId === "part_3:wire_00034" &&
          runtime.savedSeams.some((seam) => seam.edgeId === "part_3:wire_00034") &&
          /selected, captured, and animating/i.test(status),
      );
    },
    { timeout: 120_000 },
  );
  await delay(750);
  await captureStep(page, "01-weld-sequence-final");

  const state = await page.evaluate((): SeamTourState => {
    const runtime = (window as typeof window & {
      __cadquerySeamRuntime?: {
        model: { model_id: string; edges: unknown[] } | null;
        selectedEdgeId: string | null;
        savedSeams: SeamTourState["savedSeams"];
          tourStep?: number;
          weldAnimationProgress?: number;
          weldTorchDistanceToPathMm?: number | null;
      };
    }).__cadquerySeamRuntime;
    return {
      modelId: runtime?.model?.model_id ?? null,
      edgeCount: runtime?.model?.edges?.length ?? 0,
      selectedEdgeId: runtime?.selectedEdgeId ?? null,
      savedSeams: runtime?.savedSeams ?? [],
      tourStep: runtime?.tourStep ?? 5,
      status: document.querySelector<HTMLElement>("#status")?.innerText ?? "",
      seamJSON: document.querySelector<HTMLTextAreaElement>("#seam-json")?.value ?? "",
      tourCurrentTitle: "",
      tourLogItems: [],
      robotStatus: document.querySelector<HTMLElement>("#robot-status")?.innerText ?? "",
      pathStatus: document.querySelector<HTMLElement>("#path-status")?.innerText ?? "",
      weldStatus: document.querySelector<HTMLElement>("#weld-status")?.innerText ?? "",
      weldProgress: runtime?.weldAnimationProgress ?? 0,
      weldTorchDistanceToPathMm: runtime?.weldTorchDistanceToPathMm ?? null,
    };
  });

  assert.match(state.modelId ?? "", /^cadquery-step-/, "Dog Box tour should load a Geometry topology model");
  assert.ok(state.edgeCount > 0, `Dog Box tour should extract edges, got ${state.edgeCount}`);
  assert.equal(state.selectedEdgeId, "part_3:wire_00034", "Weld Sequence should select canonical Dog Box seam");
  assert.ok(
    state.savedSeams.some((seam) => seam.edgeId === "part_3:wire_00034"),
    `Weld Sequence should capture canonical seam: ${JSON.stringify(state.savedSeams)}`,
  );
  assert.ok(state.savedSeams[0].points.length >= 2, "Captured seam should contain sampled points");
  assert.equal(state.savedSeams[0].startS, 0, "Canonical seam should start at 0%");
  assert.equal(state.savedSeams[0].endS, 1, "Canonical seam should end at 100%");
  assert.match(state.status, /selected, captured, and animating/i, `Tour status should report completion: ${state.status}`);
  assert.match(state.seamJSON, /"workflow"/, "Seam JSON should include workflow metadata");
  assert.match(state.seamJSON, /"computed_by": "python\/cadquery"/, "Seam JSON should be computed by the backend Geometry path");
  assert.match(state.seamJSON, /"edge_id"/, "Seam JSON should include selected edge id");
  assert.match(state.seamJSON, /"seam_plane_normal"/, "Tesseract TCP poses should include seam plane normals");
  assert.match(state.seamJSON, /"angle_from_seam_plane_deg": 45/, "Tesseract TCP poses should constrain torch 45 degrees out of the seam plane");
  assert.match(state.seamJSON, /"tool_x_tangent_alignment": 1/, "Tool +X should stay tangent/aligned to the seam direction");
  assert.match(state.seamJSON, /"geometry_method"/, "Backend should include selected weld geometry method diagnostics");
  assert.match(state.seamJSON, /"candidates"/, "Backend should expose geometry method candidates for review");
  assert.match(state.seamJSON, /"dog_box_workpiece"/, "Tesseract environment should include Dog Box as a collision object");
  assert.match(state.seamJSON, /"tool_x_axis": "locked_to_seam_tangent"/, "Tesseract constraints should document tool +X tangent locking");
  assert.match(
    state.robotStatus,
    /rosie_1400_v3 (URDF loaded|animating backend joint trajectory)/i,
    `Robot overlay should load and animate: ${state.robotStatus}`,
  );
  assert.match(state.pathStatus, /Tesseract TCP poses.*\+X tangent.*45deg plane/i, `Tesseract TCP path should load: ${state.pathStatus}`);
  assert.match(state.weldStatus, /45deg weld torch/i, `Weld animation should be active: ${state.weldStatus}`);
  assert.ok(state.weldProgress >= 0 && state.weldProgress <= 1, `Weld animation progress should be normalized: ${state.weldProgress}`);
  assert.ok(
    typeof state.weldTorchDistanceToPathMm === "number" && state.weldTorchDistanceToPathMm < 12,
    `Weld torch should stay on the active seam path, got distance ${state.weldTorchDistanceToPathMm}`,
  );

  assert.match(state.seamJSON, /"motion_plan"/, "Seam JSON should include the Tesseract motion plan");
  assert.match(state.seamJSON, /"collision"/, "Seam JSON should include the Tesseract collision summary");

  const edgeURL = new URL(targetURL.toString());
  edgeURL.pathname = edgeURL.pathname.replace(/\/seam$/, "/edge-geometry");
  edgeURL.search = "?proof=multi-app";
  await page.goto(edgeURL.toString(), { waitUntil: "networkidle0", timeout: 90_000 });
  await page.waitForSelector("#frame-tour", { timeout: 30_000 });
  // The visible Tour Test button always runs the full multi-app loop; drive the
  // single-page frame tour through the runtime hook for deterministic asserts.
  await page.evaluate(() => {
    void (window as typeof window & {
      __edgeGeometryRuntime?: { runFrameTour?: () => Promise<void> };
    }).__edgeGeometryRuntime?.runFrameTour?.();
  });
  const backendFrameState = await waitForEdgeFrameTour(page, /Frame tour complete/i);
  await captureStep(page, "02-edge-geometry-backend-frame");
  assert.equal(backendFrameState.selectedEdgeId, "part_2:wire_00024", "Edge Geometry should select first dog-bone seam");
  assert.equal(backendFrameState.result, "FRAME TOUR PASS");
  assert.equal(backendFrameState.billboards.length, 6, `Backend frame should show 6 billboard labels: ${backendFrameState.billboards}`);
  assert.ok(backendFrameState.sceneCounts.torch === 1 && backendFrameState.sceneCounts.trace === 1 && backendFrameState.sceneCounts.liveGizmo === 1);
  assert.ok(
    typeof backendFrameState.torchMoveMm === "number" && backendFrameState.torchMoveMm > 1,
    `Edge Geometry torch should move, got ${backendFrameState.torchMoveMm}`,
  );
  assert.equal(backendFrameState.tourLogTextPresent, false, "Frame tour should use closed captions, not side tour log text");
  assert.match(backendFrameState.caption, /red \+X follows travel/i);

  await page.evaluate(() => document.querySelector<HTMLButtonElement>("#frame-mode-toggle")?.click());
  await page.evaluate(() => {
    void (window as typeof window & {
      __edgeGeometryRuntime?: { runFrameTour?: () => Promise<void> };
    }).__edgeGeometryRuntime?.runFrameTour?.();
  });
  const alternateFrameState = await waitForEdgeFrameTour(page, /\+X mode tour complete/i);
  await captureStep(page, "03-edge-geometry-x-mode-frame");
  assert.equal(alternateFrameState.modeText, "Frame: +X Mode");
  assert.equal(alternateFrameState.result, "ALT FRAME TOUR PASS");
  assert.equal(alternateFrameState.billboards.length, 3, `Alternate frame should show 3 billboard labels: ${alternateFrameState.billboards}`);
  assert.ok(alternateFrameState.billboards.some((name) => name.includes("x-torch")));
  assert.match(alternateFrameState.caption, /red \+X is the into-box torch axis/i);

  await writeFile(
    path.join(artifactDir, "cadquery-seam-tour-state.json"),
    `${JSON.stringify({ weldSequence: state, edgeGeometryBackend: backendFrameState, edgeGeometryAlternate: alternateFrameState }, null, 2)}\n`,
    "utf8",
  );

  await recorder.stop();
  await captureStep(page, "04-multi-app-tour-final");

  assert.deepEqual(consoleErrors, [], `browser console errors:\n${consoleErrors.join("\n")}`);
  assert.deepEqual(pageErrors, [], `browser page errors:\n${pageErrors.join("\n")}`);
  assert.deepEqual(failedRequests, [], `browser failed requests:\n${failedRequests.join("\n")}`);
  assert.ok(existsSync(videoPath), `expected MP4 video at ${videoPath}`);
  console.log(`cadquery multi-app tour ok: ${state.edgeCount} edges, ${backendFrameState.billboards.length} backend labels, ${alternateFrameState.billboards.length} alt labels`);
  console.log(`artifacts: ${artifactDir}`);
} finally {
  await browser.close();
  if (viteProcess) {
    viteProcess.kill();
  }
}

async function sampleMotionPlan(page: Page): Promise<MotionPlanSample> {
  return page.evaluate((): MotionPlanSample => {
    const runtime = (window as typeof window & {
      __cadquerySeamRuntime?: {
        weldAnimationProgress?: number;
        weldTorchDistanceToPathMm?: number | null;
        robotJointsAnimating?: boolean;
        robotJointAnglesRad?: number[] | null;
        collisionSampleCount?: number;
        collisionOutlinedLinks?: string[];
        scene?: {
          getObjectByName?: (name: string) => {
            position: { toArray: () => number[] };
            traverse: (callback: (object: { name?: string }) => void) => void;
          } | undefined;
          traverse?: (callback: (object: { name?: string }) => void) => void;
        };
      };
    }).__cadquerySeamRuntime;
    const scene = runtime?.scene;
    const torch = scene?.getObjectByName?.("weld-45deg-torch");
    const motionGroup = scene?.getObjectByName?.("cadquery-tesseract-motion-plan");
    let weldRuns = 0;
    let travelRuns = 0;
    let gizmos = 0;
    let collisionMarkers = 0;
    motionGroup?.traverse((object) => {
      if (object.name === "motion-weld-run") weldRuns += 1;
      if (object.name === "motion-travel-run") travelRuns += 1;
      if (object.name?.startsWith("motion-gizmo-")) gizmos += 1;
      if (object.name?.startsWith("motion-collision-contact-")) collisionMarkers += 1;
    });
    let collisionOutlineObjects = 0;
    runtime?.scene?.traverse?.((object) => {
      if (object.name?.startsWith("collision-outline-")) collisionOutlineObjects += 1;
    });
    return {
      progress: runtime?.weldAnimationProgress ?? null,
      torchPos: torch ? torch.position.toArray() : null,
      torchDistToPathMm: runtime?.weldTorchDistanceToPathMm ?? null,
      motionGroupPresent: Boolean(motionGroup),
      weldRuns,
      travelRuns,
      gizmos,
      robotJointsAnimating: runtime?.robotJointsAnimating ?? false,
      robotJointAnglesRad: runtime?.robotJointAnglesRad ?? null,
      collisionSampleCount: runtime?.collisionSampleCount ?? 0,
      collisionMarkers,
      collisionStatusText: document.querySelector<HTMLElement>("#collision-status")?.innerText ?? "",
      collisionOutlinedLinks: runtime?.collisionOutlinedLinks ?? [],
      collisionOutlineObjects,
    };
  });
}

async function captureStep(page: Page, name: string): Promise<void> {
  await page.screenshot({
    path: path.join(artifactDir, `cadquery-seam-tour-step-${name}.png`),
    fullPage: true,
  });
}

async function waitForEdgeFrameTour(page: Page, captionPattern: RegExp): Promise<EdgeGeometryTourState> {
  await page.waitForFunction(
    (patternSource) => {
      const caption = document.querySelector<HTMLElement>("#frame-tour-caption")?.innerText ?? "";
      const frameTourButton = document.querySelector<HTMLButtonElement>("#frame-tour");
      return Boolean(frameTourButton && !frameTourButton.disabled && new RegExp(patternSource, "i").test(caption));
    },
    { timeout: 120_000 },
    captionPattern.source,
  );
  return page.evaluate(async (): Promise<EdgeGeometryTourState> => {
    const runtime = (window as typeof window & {
      __edgeGeometryRuntime?: {
        scene: {
          traverse?: (callback: (object: { name?: string; type?: string; position?: { x: number; y: number; z: number } }) => void) => void;
          getObjectByName?: (name: string) => { position: { x: number; y: number; z: number } } | undefined;
        };
      };
    }).__edgeGeometryRuntime;
    const countScene = () => {
      const billboards: string[] = [];
      const sceneCounts = { frameTourGroup: 0, torch: 0, trace: 0, liveGizmo: 0 };
      runtime?.scene?.traverse?.((object) => {
        if (object.name === "edge-geometry-frame-tour") sceneCounts.frameTourGroup += 1;
        if (object.name === "edge-geometry-animated-weld-torch") sceneCounts.torch += 1;
        if (object.name === "edge-geometry-weld-trace") sceneCounts.trace += 1;
        if (object.name === "edge-geometry-live-weld-tcp-gizmo") sceneCounts.liveGizmo += 1;
        if (object.type === "Sprite" && object.name?.startsWith("frame-billboard-")) billboards.push(object.name);
      });
      return { billboards, sceneCounts };
    };
    const torchBefore = runtime?.scene?.getObjectByName?.("edge-geometry-animated-weld-torch")?.position;
    const before = torchBefore ? { x: torchBefore.x, y: torchBefore.y, z: torchBefore.z } : null;
    await new Promise((resolve) => setTimeout(resolve, 900));
    const torchAfter = runtime?.scene?.getObjectByName?.("edge-geometry-animated-weld-torch")?.position;
    const after = torchAfter ? { x: torchAfter.x, y: torchAfter.y, z: torchAfter.z } : null;
    const { billboards, sceneCounts } = countScene();
    const torchMoveMm = before && after
      ? Math.hypot(after.x - before.x, after.y - before.y, after.z - before.z)
      : null;
    return {
      selectedEdgeId: document.querySelector<HTMLElement>("#selected-title")?.innerText ?? null,
      caption: document.querySelector<HTMLElement>("#frame-tour-caption")?.innerText ?? "",
      result: document.querySelector<HTMLElement>("#result-state")?.innerText ?? "",
      modeText: document.querySelector<HTMLButtonElement>("#frame-mode-toggle")?.innerText ?? "",
      billboards,
      sceneCounts,
      torchMoveMm,
      tourLogTextPresent: document.body.textContent?.includes("Tour Log") ?? false,
    };
  });
}

function startVite(): ChildProcess {
  const child = spawn(process.execPath, [
    path.join(uiRoot, "node_modules", "vite", "bin", "vite.js"),
    "--host",
    "127.0.0.1",
    "--port",
    testPort,
    "--strictPort",
    "--base",
    "/cadquery/v1/ui/",
  ], {
    cwd: uiRoot,
    stdio: "inherit",
    env: process.env,
  });
  return child;
}

function findUIRoot(start: string): string {
  let dir = path.resolve(start);
  for (;;) {
    if (existsSync(path.join(dir, "package.json")) && existsSync(path.join(dir, "index.html"))) {
      return dir;
    }
    const parent = path.dirname(dir);
    if (parent === dir) {
      throw new Error(`Could not locate cadquery/v1/ui from ${start}`);
    }
    dir = parent;
  }
}

function hookBrowserLogs(page: Page): void {
  page.on("console", (msg) => {
    if (msg.type() === "error" && !msg.text().includes("Failed to load resource")) {
      consoleErrors.push(msg.text());
    }
  });
  page.on("pageerror", (err) => {
    pageErrors.push(err instanceof Error ? err.stack || err.message : String(err));
  });
  page.on("requestfailed", (request) => {
    failedRequests.push(`${request.method()} ${request.url()} ${request.failure()?.errorText ?? "failed"}`);
  });
  page.on("response", (response) => {
    if (response.status() >= 400 && !response.url().endsWith("/favicon.ico")) {
      failedRequests.push(`${response.status()} ${response.url()}`);
    }
  });
}

async function waitForHTTP(url: string, timeoutMs: number): Promise<void> {
  const start = Date.now();
  let lastError = "";
  while (Date.now() - start < timeoutMs) {
    try {
      const response = await fetch(url, { method: "GET" });
      if (response.ok) {
        return;
      }
      lastError = `HTTP ${response.status}`;
    } catch (error) {
      lastError = error instanceof Error ? error.message : String(error);
    }
    await new Promise((resolve) => setTimeout(resolve, 500));
  }
  throw new Error(`Timed out waiting for ${url}: ${lastError}`);
}

async function isHTTPReady(url: string): Promise<boolean> {
  try {
    const response = await fetch(url, { method: "GET" });
    return response.ok;
  } catch {
    return false;
  }
}

async function findChrome(): Promise<string> {
  const explicit = process.env.PUPPETEER_EXECUTABLE_PATH || process.env.CHROME_PATH;
  if (explicit && existsSync(explicit)) {
    return explicit;
  }
  const candidates =
    process.platform === "win32"
      ? [
          path.join(process.env.PROGRAMFILES || "C:\\Program Files", "Google", "Chrome", "Application", "chrome.exe"),
          path.join(process.env["PROGRAMFILES(X86)"] || "C:\\Program Files (x86)", "Google", "Chrome", "Application", "chrome.exe"),
          path.join(process.env.LOCALAPPDATA || "", "Google", "Chrome", "Application", "chrome.exe"),
          path.join(process.env.PROGRAMFILES || "C:\\Program Files", "Microsoft", "Edge", "Application", "msedge.exe"),
          path.join(process.env["PROGRAMFILES(X86)"] || "C:\\Program Files (x86)", "Microsoft", "Edge", "Application", "msedge.exe"),
        ]
      : process.platform === "darwin"
        ? [
            "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
            "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge",
            "/Applications/Chromium.app/Contents/MacOS/Chromium",
          ]
        : ["/usr/bin/google-chrome", "/usr/bin/chromium-browser", "/usr/bin/chromium", "/opt/google/chrome/chrome"];
  const found = candidates.find((candidate) => candidate && existsSync(candidate));
  if (!found) {
    throw new Error("Chrome/Edge executable not found. Set PUPPETEER_EXECUTABLE_PATH.");
  }
  return found;
}
