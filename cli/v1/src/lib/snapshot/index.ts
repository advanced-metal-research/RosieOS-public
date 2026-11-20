// 3D scene snapshots of an OLP program, drawn by OLP's own renderer in
// headless Chrome. Build the page first: `node cli/v1/scripts/build-snapshot.mjs`.
export { renderSnapshots, defaultSnapshotDir } from "./render.js";
export type {
  NamedCamera, RenderOptions, RenderResult, RenderedFile, SceneInfo, ShotSpec, SnapshotCamera, SnapshotInput, SnapshotPose,
} from "./render.js";
export { CdpClient, findChrome, withChrome, type ChromeSession } from "./chrome.js";
export { startStaticServer, contentType, type StaticServer } from "./server.js";
