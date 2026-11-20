// Build OLP's scene-snapshot page into cli/v1/dist/snapshot (or
// ROSIE_SNAPSHOT_OUT): stage the robot assets with the UI's own
// scripts/stage-assets.mjs (what `npm run dev` runs; it writes the UI's
// gitignored .staged-public), then run the UI's local Vite on
// vite.snapshot.config.ts. Never the UI's `npm run build`: that writes
// offline-programming/v1/ui/dist, the :8794 redirect stub.
//
//   node cli/v1/scripts/build-snapshot.mjs
import { spawnSync } from "node:child_process";
import { existsSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const cliRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const repoRoot = path.resolve(cliRoot, "../..");
const ui = path.join(repoRoot, "offline-programming", "v1", "ui");
const vite = path.join(ui, "node_modules", "vite", "bin", "vite.js");
const outDir = path.resolve(process.env.ROSIE_SNAPSHOT_OUT ?? path.join(cliRoot, "dist", "snapshot"));

if (!existsSync(vite)) {
  console.error(`OLP UI packages are missing (${vite}): run \`npm ci\` in offline-programming/v1/ui first`);
  process.exit(1);
}

function run(args) {
  const started = Date.now();
  const result = spawnSync(process.execPath, args, {
    cwd: ui,
    stdio: "inherit",
    env: { ...process.env, ROSIE_SNAPSHOT_OUT: outDir },
  });
  if (result.status !== 0) {
    console.error(`build-snapshot: \`node ${args.join(" ")}\` failed (${result.status ?? result.signal})`);
    process.exit(1);
  }
  return Date.now() - started;
}

const stageMs = run([path.join("scripts", "stage-assets.mjs")]);
const buildMs = run([vite, "build", "--config", "vite.snapshot.config.ts"]);
for (const required of ["snapshot.html", path.join("robot", "manifest.json")]) {
  if (!existsSync(path.join(outDir, required))) {
    console.error(`build-snapshot: ${required} is missing from ${outDir}`);
    process.exit(1);
  }
}
console.error(`built ${path.relative(process.cwd(), outDir) || outDir} (stage ${stageMs} ms, vite ${buildMs} ms)`);
