/**
 * The scene-snapshot page (snapshot.html + src/snapshot.ts) and the robot
 * assets it loads, built for `cli/v1/src/lib/snapshot` to serve to headless
 * Chrome. Run by `cli/v1/scripts/build-snapshot.mjs`, which stages the
 * assets first (scripts/stage-assets.mjs, the step `npm run dev` runs).
 *
 * The output goes to ROSIE_SNAPSHOT_OUT (default <repo>/cli/v1/dist/snapshot),
 * NEVER to this UI's dist/: that is the deliberate redirect stub :8794 serves
 * (CLAUDE.md, "Three traps"), and it is gitignored, so overwriting it would
 * break the API's browser bounce without anything showing in git status.
 */
import { resolve } from "node:path";
import { fileURLToPath } from "node:url";

import { defineConfig } from "vite";

const uiRoot = fileURLToPath(new URL(".", import.meta.url));
const repoRoot = resolve(uiRoot, "../../..");
const outDir = resolve(process.env.ROSIE_SNAPSHOT_OUT ?? resolve(repoRoot, "cli/v1/dist/snapshot"));
if (outDir === resolve(uiRoot, "dist") || outDir.startsWith(resolve(uiRoot, "dist") + "/")) {
  throw new Error(`ROSIE_SNAPSHOT_OUT must not be the OLP UI's dist/ (the :8794 redirect stub): ${outDir}`);
}

export default defineConfig({
  root: uiRoot,
  // Relative URLs: the page is served from whatever directory and port the
  // CLI's static server picks.
  base: "./",
  publicDir: resolve(uiRoot, ".staged-public"),
  // Its own dependency cache, so a build never churns the dev server's.
  cacheDir: resolve(uiRoot, "node_modules/.vite-snapshot"),
  logLevel: "warn",
  build: {
    outDir,
    emptyOutDir: true,
    chunkSizeWarningLimit: 2000,
    rollupOptions: {
      input: { snapshot: resolve(uiRoot, "snapshot.html") },
    },
  },
});
