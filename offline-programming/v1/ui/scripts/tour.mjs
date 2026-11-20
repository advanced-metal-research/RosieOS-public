import { readFile } from "node:fs/promises";
import { resolve } from "node:path";

const steps = [
  "Open the bundled Dog Box project",
  "Confirm the shared Orbit / J6 / TCP workstation renders one model",
  "Select, trim, reverse, and reorder seams",
  "Generate an orange PREVIEW plan",
  "Play and scrub the browser-only trajectory",
  "Select Local simulator and confirm it never auto-starts",
  "Explicitly start, stop, and plan only when canonical dependencies are available",
  "Inspect collision, singularity, limit, runtime, and revision evidence",
  "Select a manifest-backed Robot V4 target and create its immutable accepted plan",
  "Request server review of the exact stored plan and fresh RTCore evidence",
  "Explicitly confirm Start, monitor the supervised session, and Stop safely",
  "Save, reload, export, and edit the same shared spreadsheet program",
  "Confirm raw browser robot transport and tele-op remain disabled",
];

if (process.argv.includes("--check")) {
  const uiRoot = resolve(import.meta.dirname, "..");
  const productRoot = resolve(uiRoot, "..");
  const main = await readFile(resolve(uiRoot, "src", "main.ts"), "utf8");
  const server = await readFile(resolve(productRoot, "server.go"), "utf8");
  const simulatorContracts = await readFile(
    resolve(productRoot, "internal", "simulator", "contracts.go"),
    "utf8",
  );
  const requiredMainMarkers = [
    "new WorkstationRenderer",
    "HttpCadClient",
    "syncProjectToWorkstation",
    "reviewPhysicalStart",
    "startReviewedExecution",
    "stopActiveExecution",
    "LocalPlannerAdapter",
    "ConnectedTargetsClient",
    "Plan with local simulator",
    "Review physical start",
  ];
  for (const marker of requiredMainMarkers) {
    if (!main.includes(marker)) throw new Error(`tour integration marker is missing: ${marker}`);
  }
  if (main.includes("WorkstationViewer") || main.includes("./api/cadquery")) {
    throw new Error("tour found a duplicate scaffold renderer or CAD boundary");
  }
  for (const endpoint of ["/execute", "/teleop"]) {
    if (!server.includes(`POST /api/offline-programming/v1${endpoint}`)) {
      throw new Error(`fail-closed server endpoint is missing: ${endpoint}`);
    }
  }
  for (const endpoint of [
    "local-simulator/status",
    "local-simulator/start",
    "local-simulator/stop",
    "local-simulator/plans",
    "connected-targets",
    'targets/{profile}/plans',
    'plans/{plan_id}',
    'executionPath+"/review',
    'executionPath+"/start',
    'executionPath+"/{execution_id}/stop',
  ]) {
    if (!`${server}\n${simulatorContracts}`.includes(endpoint)) {
      throw new Error(`integrated server endpoint is missing: ${endpoint}`);
    }
  }
  if (steps.length !== 13 || !steps.at(-1)?.includes("disabled")) {
    throw new Error("offline programming integrated milestone tour contract is incomplete");
  }
  process.stdout.write("offline-programming v1 integrated tour contract: ok\n");
} else {
  process.stdout.write(`${steps.map((step, index) => `${index + 1}. ${step}`).join("\n")}\n`);
}
