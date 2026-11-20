import { defineConfig } from "vite";

// The committed sphere model is part of the robot's DESCRIPTION now
// (`robot_description/<cell>/spheres.json`, beside the meshes it was fitted to), so the
// page reads it through `/cell-assets` like every other file in that
// directory and there is no separate route for it. This used to be a
// `/cell-data` middleware pointing at `weld_planner/v1/data/motion/cells/`.
// A copy under `public/` was the first idea and it is a trap: the page then
// shows whatever was copied last, which after one re-authoring is a collision
// model that exists nowhere else.

export default defineConfig({
  plugins: [],
  server: {
    // Bound wide, not to localhost. This machine is reached over Tailscale,
    // and Vite's default of 127.0.0.1 means the page is unreachable from
    // anywhere but the box itself -- which looks like a broken dev server
    // rather than a bind address.
    host: true,
    port: 5179,
    // `serve.py` owns the cell: `/api/cells` for the descriptor,
    // `/cell-assets` for the .dae meshes it names, `/cell-urdf` for the file
    // itself. `/cell-assets` is deliberately NOT under `/api`, and it was once
    // left out of a list like this one: Vite then answered every mesh request
    // with its SPA fallback, `ColladaLoader` failed on `<!doctype html>` with
    // "StartTag: invalid element name", and every link loaded with no geometry
    // at all. A 404 would have been visible; the fallback made a missing file
    // look like an empty part.
    proxy: {
      "/api": { target: "http://127.0.0.1:8795", changeOrigin: true },
      "/cell-assets": { target: "http://127.0.0.1:8795", changeOrigin: true },
      "/cell-urdf": { target: "http://127.0.0.1:8795", changeOrigin: true },
    },
  },
  build: { outDir: "dist", emptyOutDir: true },
});
