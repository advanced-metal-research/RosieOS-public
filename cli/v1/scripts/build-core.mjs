// Bundle OLP's own program engine (core/entry.ts -> dist/core/olp-core.mjs),
// as steamdeck/real/v5's Makefile bundles olp-core for the Deck. The engine's
// packages (three, urdf-loader, happy-dom) are the OLP UI's: run `npm ci` in
// offline-programming/v1/ui first, so the CLI, the MCP server and the browser run the
// same versions.
import { existsSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

import { build } from "esbuild";

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const uiModules = path.resolve(root, "../../offline-programming/v1/ui/node_modules");
if (!existsSync(path.join(uiModules, "three"))) {
  console.error(`OLP UI packages are missing (${uiModules}): run \`npm ci\` in offline-programming/v1/ui first`);
  process.exit(1);
}
await build({
  entryPoints: [path.join(root, "core", "entry.ts")],
  outfile: path.join(root, "dist", "core", "olp-core.mjs"),
  bundle: true,
  format: "esm",
  platform: "node",
  target: "node20",
  nodePaths: [uiModules],
  logLevel: "warning",
  // happy-dom reaches for optional native packages it never loads here.
  external: ["canvas", "bufferutil", "utf-8-validate"],
  banner: { js: "import { createRequire as __createRequire } from 'node:module'; const require = __createRequire(import.meta.url);" },
});
console.error("bundled dist/core/olp-core.mjs");
