// One-shot check that the Weld Sequence torch animation is playing the
// backend Tesseract motion plan: motion group present, progress advancing,
// torch moving through space while staying on the planned path.
import puppeteer from "puppeteer-core";

const browser = await puppeteer.connect({ browserURL: "http://127.0.0.1:9223", defaultViewport: null });
const pages = await browser.pages();
const page = pages.find((p) => p.url().includes("/cadquery/v1/ui/")) ?? pages[0];

await page.goto("http://127.0.0.1:5184/cadquery/v1/ui/seam", { waitUntil: "networkidle0", timeout: 90_000 });
await page.waitForSelector("#quick-tour-button", { timeout: 30_000 });
await page.evaluate(() => document.querySelector("#quick-tour-button")?.click());

// Wait until the canonical seam is selected and the motion plan is in the
// scene; the animation runs from that point on, well before the tour's final
// step (the visible Tour Test button always ends by navigating to the next
// app, so sampling must happen while the tour is still on this page).
await page.waitForFunction(
  () => {
    const runtime = window.__cadquerySeamRuntime;
    return Boolean(
      runtime?.selectedEdgeId === "part_3:wire_00034" &&
        runtime?.scene?.getObjectByName("cadquery-tesseract-motion-plan") &&
        runtime?.scene?.getObjectByName("weld-45deg-torch"),
    );
  },
  { timeout: 150_000 },
);

const snapshot = () =>
  page.evaluate(() => {
    const runtime = window.__cadquerySeamRuntime;
    const scene = runtime?.scene;
    const torch = scene?.getObjectByName("weld-45deg-torch");
    const motionGroup = scene?.getObjectByName("cadquery-tesseract-motion-plan");
    let weldRuns = 0;
    let travelRuns = 0;
    let gizmos = 0;
    motionGroup?.traverse((o) => {
      if (o.name === "motion-weld-run") weldRuns += 1;
      if (o.name === "motion-travel-run") travelRuns += 1;
      if (o.name?.startsWith("motion-gizmo-")) gizmos += 1;
    });
    return {
      progress: runtime?.weldAnimationProgress ?? null,
      torchPos: torch ? torch.position.toArray().map((v) => Math.round(v * 100) / 100) : null,
      torchDistToPathMm: runtime?.weldTorchDistanceToPathMm ?? null,
      motionGroupPresent: Boolean(motionGroup),
      weldRuns,
      travelRuns,
      gizmos,
      robotJointsAnimating: runtime?.robotJointsAnimating ?? false,
      robotJointAnglesRad: runtime?.robotJointAnglesRad ?? null,
      collisionActive: runtime?.collisionActive ?? false,
      collisionLinks: runtime?.collisionLinks ?? [],
      collisionSampleCount: runtime?.collisionSampleCount ?? 0,
      collisionOutlinedLinks: runtime?.collisionOutlinedLinks ?? [],
      weldStatus: document.querySelector("#weld-status")?.innerText ?? "",
      robotStatus: document.querySelector("#robot-status")?.innerText ?? "",
      collisionStatus: document.querySelector("#collision-status")?.innerText ?? "",
    };
  });

const a = await snapshot();
await new Promise((r) => setTimeout(r, 2500));
const b = await snapshot();

const moved =
  a.torchPos && b.torchPos ? Math.hypot(...a.torchPos.map((v, i) => v - b.torchPos[i])) : 0;
const jointDelta =
  Array.isArray(a.robotJointAnglesRad) && Array.isArray(b.robotJointAnglesRad)
    ? Math.max(...a.robotJointAnglesRad.map((v, i) => Math.abs(b.robotJointAnglesRad[i] - v)))
    : 0;
const verdict = {
  motionPlanRendered: b.motionGroupPresent && b.weldRuns >= 1 && b.travelRuns >= 1 && b.gizmos > 0,
  progressAdvancing: a.progress !== null && b.progress !== null && b.progress !== a.progress,
  torchMovedMm: Math.round(moved * 100) / 100,
  torchOnPlannedPath: typeof b.torchDistToPathMm === "number" && b.torchDistToPathMm < 12,
  robotJointsAnimating: a.robotJointsAnimating && b.robotJointsAnimating,
  robotJointDeltaRad: Math.round(jointDelta * 1e6) / 1e6,
  collisionReported: /collision: (clear|\d+\/\d+ contact waypoints)|COLLISION .+ depth=[\d.]+mm/.test(b.collisionStatus),
  collisionSampleCount: b.collisionSampleCount,
  collisionOutlinedLinks: b.collisionOutlinedLinks,
  collisionBothSidesOutlined: b.collisionSampleCount === 0 || b.collisionOutlinedLinks.length >= 2,
  sampleA: a,
  sampleB: b,
};
verdict.PASS =
  verdict.motionPlanRendered &&
  verdict.progressAdvancing &&
  verdict.torchMovedMm > 1 &&
  verdict.torchOnPlannedPath &&
  verdict.robotJointsAnimating &&
  verdict.robotJointDeltaRad > 1e-4 &&
  verdict.collisionReported &&
  verdict.collisionBothSidesOutlined;
console.log(JSON.stringify(verdict, null, 2));

await page.screenshot({ path: process.argv[2] ?? "chrome-animation-verify.png" });
browser.disconnect();
process.exit(verdict.PASS ? 0 : 1);
