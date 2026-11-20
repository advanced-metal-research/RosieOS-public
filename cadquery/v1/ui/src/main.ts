import "./style.css";

import * as monaco from "monaco-editor/esm/vs/editor/editor.api.js";
import editorWorker from "monaco-editor/esm/vs/editor/editor.worker?worker";
import "monaco-editor/esm/vs/language/typescript/monaco.contribution";
import tsWorker from "monaco-editor/esm/vs/language/typescript/ts.worker?worker";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

type LibraryItem = {
  id: string;
  name: string;
  manifest: string;
  step?: string;
  source?: string;
};

type CadQueryLink = {
  name: string;
  mesh: string;
  source: string;
  mass_properties?: {
    volume?: number;
    center?: number[];
    bounding_box?: Record<"xmin" | "ymin" | "zmin" | "xmax" | "ymax" | "zmax", number>;
  };
};

type CadQueryManifest = {
  robot_name: string;
  step: string;
  output_dir: string;
  split: boolean;
  solid_count: number;
  links: CadQueryLink[];
  warnings?: string[];
};

type CadQueryRuntime = {
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  renderer: THREE.WebGLRenderer;
  controls: OrbitControls;
  codeEditor: monaco.editor.IStandaloneCodeEditor;
  manifest: CadQueryManifest | null;
  linkCount: number;
};

declare global {
  interface Window {
    __cadqueryLibraryRuntime?: CadQueryRuntime;
  }
}

const monacoEnvironmentTarget = self as typeof self & {
  MonacoEnvironment: {
    getWorker(_workerID: string, label: string): Worker;
  };
};

monacoEnvironmentTarget.MonacoEnvironment = {
  getWorker(_workerID: string, label: string) {
    if (label === "typescript" || label === "javascript") {
      return new tsWorker();
    }
    return new editorWorker();
  },
};

function uiBasePath(): string {
  const viteBase = import.meta.env.BASE_URL || "/";
  if (viteBase !== "/") {
    return viteBase;
  }
  const pathname = window.location.pathname;
  return pathname.endsWith("/") ? pathname : `${pathname}/`;
}

const uiBaseURL = new URL(uiBasePath(), window.location.origin);

function assetURL(value: string): string {
  return new URL(value.replace(/^\/+/, ""), uiBaseURL).toString();
}

const fallbackItems: LibraryItem[] = [
  {
    id: "dog_box",
    name: "dog box",
    manifest: assetURL("library/dog_box/cadquery_manifest.json"),
    source: "Fusion active document",
  },
];

const palette = [0xe8e8e8, 0xd2d2d2, 0xbcbcbc, 0xa6a6a6, 0x909090, 0x787878, 0x626262, 0x4c4c4c];
const editorStorageKey = "gradient.cadquery.v1.editorDraft";
const fallbackEditorText = `type GeometryLibraryItem = {
  robotName: string;
  solids: number;
  meshes: string[];
};

export const activeItem: GeometryLibraryItem = {
  robotName: "loading",
  solids: 0,
  meshes: [],
};

export function label(item: GeometryLibraryItem): string {
  return \`\${item.robotName}: \${item.solids} solids / \${item.meshes.length} meshes\`;
}
`;

const app = document.querySelector<HTMLElement>("#app");
if (!app) {
  throw new Error("missing #app");
}

document.documentElement.dataset.cadqueryState = "loading";

app.innerHTML = `
  <div class="workspace">
    <header class="topbar">
      <div>
        <p class="eyebrow">Rosie Geometry / v1</p>
        <h1>Geometry Object Library</h1>
      </div>
      <div class="status-strip" aria-live="polite">
        <span id="status" class="status">Loading library</span>
      </div>
    </header>
    <section class="control-grid" aria-label="Geometry library controls">
      <div class="library-control">
        <label for="item-picker">Library</label>
        <select id="item-picker" aria-label="Library item"></select>
      </div>
      <div class="stats" aria-label="Loaded object statistics">
        <span id="solid-count">0 solids</span>
        <span id="mesh-count">0 meshes</span>
      </div>
      <dl class="item-meta">
        <div><dt>Robot</dt><dd id="robot-name">-</dd></div>
        <div><dt>STEP</dt><dd id="step-path">-</dd></div>
        <div><dt>Mode</dt><dd id="split-mode">-</dd></div>
        <div><dt>Subparts</dt><dd id="subpart-total">0</dd></div>
      </dl>
    </section>
    <main class="main">
      <section class="viewer-section" aria-label="3D viewer">
        <div id="viewer" class="viewer"></div>
      </section>
      <section class="code-section" aria-label="Code editor">
        <header class="code-topbar">
          <span class="code-file">geometry-library.ts</span>
          <span id="editor-status" class="editor-status">Ready</span>
        </header>
        <div id="code-editor" class="code-editor"></div>
      </section>
    </main>
    <button id="menu-button" class="menu-button" type="button" aria-haspopup="dialog" aria-controls="subpart-menu" aria-expanded="false" disabled>Menu</button>
    <div id="subpart-menu" class="subpart-menu" role="dialog" aria-modal="true" aria-labelledby="subpart-menu-title" hidden>
      <div class="subpart-menu-shell">
        <header class="subpart-menu-top">
          <div>
            <p class="eyebrow">Subparts</p>
            <h2 id="subpart-menu-title">Object Links</h2>
          </div>
          <button id="menu-close" class="menu-close" type="button">Close</button>
        </header>
        <div id="subpart-list" class="subpart-list"></div>
      </div>
    </div>
  </div>
`;

const picker = mustQuery<HTMLSelectElement>("#item-picker");
const viewer = mustQuery<HTMLElement>("#viewer");
const codeEditorHost = mustQuery<HTMLElement>("#code-editor");
const editorStatusEl = mustQuery<HTMLElement>("#editor-status");
const statusEl = mustQuery<HTMLElement>("#status");
const robotNameEl = mustQuery<HTMLElement>("#robot-name");
const stepPathEl = mustQuery<HTMLElement>("#step-path");
const splitModeEl = mustQuery<HTMLElement>("#split-mode");
const solidCountEl = mustQuery<HTMLElement>("#solid-count");
const meshCountEl = mustQuery<HTMLElement>("#mesh-count");
const subpartTotalEl = mustQuery<HTMLElement>("#subpart-total");
const subpartListEl = mustQuery<HTMLElement>("#subpart-list");
const subpartMenuEl = mustQuery<HTMLElement>("#subpart-menu");
const subpartMenuTitleEl = mustQuery<HTMLElement>("#subpart-menu-title");
const menuButton = mustQuery<HTMLButtonElement>("#menu-button");
const menuCloseButton = mustQuery<HTMLButtonElement>("#menu-close");
let suppressEditorChange = false;
let editorHasUserDraft = Boolean(readStoredEditor());

monaco.editor.defineTheme("cadquery-dark", {
  base: "vs-dark",
  inherit: true,
  rules: [],
  colors: {
    "editor.background": "#050608",
    "editor.foreground": "#e8e8e8",
    "editorLineNumber.foreground": "#6e747c",
    "editorLineNumber.activeForeground": "#f1f1f1",
    "editorCursor.foreground": "#ffffff",
    "editor.selectionBackground": "#264f78",
    "editor.inactiveSelectionBackground": "#1f3448",
  },
});

const codeEditor = monaco.editor.create(codeEditorHost, {
  value: readStoredEditor() ?? fallbackEditorText,
  language: "typescript",
  theme: "cadquery-dark",
  automaticLayout: true,
  minimap: { enabled: false },
  fontFamily: "Cascadia Code, SFMono-Regular, Consolas, Liberation Mono, monospace",
  fontSize: 13,
  lineHeight: 20,
  tabSize: 2,
  padding: { top: 14, bottom: 14 },
  scrollBeyondLastLine: false,
  wordWrap: "on",
});

codeEditor.onDidChangeModelContent(() => {
  if (suppressEditorChange) {
    return;
  }
  editorHasUserDraft = true;
  writeStoredEditor(codeEditor.getValue());
  editorStatusEl.textContent = "Edited";
});

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x000000);

const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 10000);
camera.position.set(460, 330, 420);

const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
viewer.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 0, 70);
controls.update();

const grid = new THREE.GridHelper(420, 14, 0xffffff, 0x343434);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

scene.add(new THREE.HemisphereLight(0xffffff, 0x2a2a2a, 2.1));

const key = new THREE.DirectionalLight(0xffffff, 2.6);
key.position.set(300, 460, 520);
key.castShadow = true;
scene.add(key);

const fill = new THREE.DirectionalLight(0xffffff, 0.7);
fill.position.set(-360, -180, 260);
scene.add(fill);

let activeGroup: THREE.Group | null = null;
let activeManifest: CadQueryManifest | null = null;
let selectedLinkName = "";

window.__cadqueryLibraryRuntime = {
  scene,
  camera,
  renderer,
  controls,
  codeEditor,
  manifest: null,
  linkCount: 0,
};

function mustQuery<T extends Element>(selector: string): T {
  const el = document.querySelector<T>(selector);
  if (!el) {
    throw new Error(`missing ${selector}`);
  }
  return el;
}

function readStoredEditor(): string | null {
  try {
    return localStorage.getItem(editorStorageKey);
  } catch {
    return null;
  }
}

function writeStoredEditor(value: string): void {
  try {
    localStorage.setItem(editorStorageKey, value);
  } catch {
    editorStatusEl.textContent = "Unsaved";
  }
}

async function fetchJSON<T>(url: string): Promise<T> {
  const response = await fetch(url, { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`${url}: HTTP ${response.status}`);
  }
  return (await response.json()) as T;
}

async function loadLibraryIndex(): Promise<LibraryItem[]> {
  try {
    const index = await fetchJSON<{ items: LibraryItem[] }>(assetURL("library/index.json"));
    if (Array.isArray(index.items) && index.items.length > 0) {
      return index.items;
    }
  } catch {
    // Generated libraries are local artifacts; the dog_box bundle is the default smoke item.
  }
  return fallbackItems;
}

function manifestBase(manifestURL: string): string {
  return manifestURL.slice(0, manifestURL.lastIndexOf("/") + 1);
}

async function loadItem(item: LibraryItem): Promise<void> {
  document.documentElement.dataset.cadqueryState = "loading";
  statusEl.textContent = `Loading ${item.name}`;
  menuButton.disabled = true;
  closeSubpartMenu({ restoreFocus: false });
  clearActiveGroup();

  const manifestURL = assetURL(item.manifest);
  const manifest = await fetchJSON<CadQueryManifest>(manifestURL);
  const base = manifestBase(manifestURL);
  const group = new THREE.Group();
  group.name = manifest.robot_name;

  await Promise.all(
    manifest.links.map(async (link, index) => {
      const geometry = await loadSTL(`${base}meshes/${encodeURIComponent(link.mesh)}`);
      geometry.computeVertexNormals();
      const material = new THREE.MeshStandardMaterial({
        color: palette[index % palette.length],
        metalness: 0.42,
        roughness: 0.38,
        side: THREE.DoubleSide,
      });
      const mesh = new THREE.Mesh(geometry, material);
      mesh.name = link.name;
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      group.add(mesh);
    }),
  );

  scene.add(group);
  activeGroup = group;
  activeManifest = manifest;
  window.__cadqueryLibraryRuntime!.manifest = manifest;
  window.__cadqueryLibraryRuntime!.linkCount = manifest.links.length;

  centerGroup(group);
  frameGroup(group);
  renderMetadata(item, manifest);
  syncEditorTemplate(item, manifest);
  statusEl.textContent = `${item.name} loaded`;
  document.documentElement.dataset.cadqueryState = "ready";
  document.documentElement.dataset.cadqueryItem = item.id;
  document.documentElement.dataset.cadqueryLinkCount = String(manifest.links.length);
  document.documentElement.dataset.cadquerySolidCount = String(manifest.solid_count);
}

function syncEditorTemplate(item: LibraryItem, manifest: CadQueryManifest): void {
  if (editorHasUserDraft) {
    return;
  }

  suppressEditorChange = true;
  codeEditor.setValue(renderEditorTemplate(item, manifest));
  suppressEditorChange = false;
  editorStatusEl.textContent = "Synced";
}

function renderEditorTemplate(item: LibraryItem, manifest: CadQueryManifest): string {
  const meshes = manifest.links.map((link) => `    "${link.mesh}",`).join("\n");
  return `type GeometryLibraryItem = {
  id: string;
  name: string;
  robotName: string;
  stepPath: string;
  solids: number;
  meshes: string[];
};

export const activeItem: GeometryLibraryItem = {
  id: ${JSON.stringify(item.id)},
  name: ${JSON.stringify(item.name)},
  robotName: ${JSON.stringify(manifest.robot_name || item.name)},
  stepPath: ${JSON.stringify(manifest.step || item.step || "")},
  solids: ${manifest.solid_count},
  meshes: [
${meshes}
  ],
};

export function label(item: GeometryLibraryItem): string {
  return \`\${item.robotName}: \${item.solids} solids / \${item.meshes.length} meshes\`;
}
`;
}

function clearActiveGroup(): void {
  if (!activeGroup) {
    return;
  }
  scene.remove(activeGroup);
  activeGroup.traverse((obj) => {
    const mesh = obj as THREE.Mesh;
    if (mesh.isMesh) {
      mesh.geometry.dispose();
      const material = mesh.material;
      if (Array.isArray(material)) {
        material.forEach((m) => m.dispose());
      } else {
        material.dispose();
      }
    }
  });
  activeGroup = null;
  activeManifest = null;
  selectedLinkName = "";
  delete document.documentElement.dataset.cadquerySelectedLink;
}

function loadSTL(url: string): Promise<THREE.BufferGeometry> {
  const loader = new STLLoader();
  return new Promise((resolve, reject) => {
    loader.load(url, resolve, undefined, (err) => reject(err));
  });
}

function centerGroup(group: THREE.Group): void {
  group.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(group);
  const center = new THREE.Vector3();
  box.getCenter(center);
  group.position.sub(center);
  group.updateMatrixWorld(true);
}

function frameGroup(group: THREE.Group): void {
  const box = new THREE.Box3().setFromObject(group);
  const center = new THREE.Vector3();
  const size = new THREE.Vector3();
  box.getCenter(center);
  box.getSize(size);
  const maxDim = Math.max(size.x, size.y, size.z, 10);
  controls.target.copy(center);
  camera.near = Math.max(maxDim / 1000, 0.1);
  camera.far = Math.max(maxDim * 12, 2000);
  camera.position.copy(center).add(new THREE.Vector3(maxDim * 1.25, maxDim * 0.9, maxDim * 1.15));
  camera.updateProjectionMatrix();
  controls.update();
}

function frameObject(target: THREE.Object3D): void {
  const box = new THREE.Box3().setFromObject(target);
  const center = new THREE.Vector3();
  const size = new THREE.Vector3();
  box.getCenter(center);
  box.getSize(size);
  const maxDim = Math.max(size.x, size.y, size.z, 35);
  const viewDirection = new THREE.Vector3().subVectors(camera.position, controls.target);
  if (viewDirection.lengthSq() < 0.001) {
    viewDirection.set(1.25, 0.9, 1.15);
  }
  viewDirection.normalize();
  controls.target.copy(center);
  camera.near = Math.max(maxDim / 1000, 0.1);
  camera.far = Math.max(maxDim * 20, 2000);
  camera.position.copy(center).add(viewDirection.multiplyScalar(maxDim * 4.2));
  camera.updateProjectionMatrix();
  controls.update();
}

function renderMetadata(item: LibraryItem, manifest: CadQueryManifest): void {
  robotNameEl.textContent = manifest.robot_name || item.name;
  stepPathEl.textContent = manifest.step || item.step || "-";
  stepPathEl.title = manifest.step || item.step || "";
  splitModeEl.textContent = manifest.split ? "split solids" : "compound";
  solidCountEl.textContent = `${manifest.solid_count} solids`;
  meshCountEl.textContent = `${manifest.links.length} meshes`;
  subpartTotalEl.textContent = String(manifest.links.length);
  subpartMenuTitleEl.textContent = `${manifest.robot_name || item.name} links`;
  menuButton.disabled = manifest.links.length === 0;
  selectedLinkName = "";
  delete document.documentElement.dataset.cadquerySelectedLink;

  subpartListEl.replaceChildren(
    ...manifest.links.map((link, index) => {
      const row = document.createElement("button");
      row.type = "button";
      row.className = "subpart-row";
      row.dataset.link = link.name;
      row.setAttribute("aria-pressed", "false");
      row.style.setProperty("--part-color", `#${palette[index % palette.length].toString(16).padStart(6, "0")}`);
      row.innerHTML = `
        <span>${link.name}</span>
        <strong>${link.mesh}</strong>
        <em>${formatVolume(link.mass_properties?.volume)}</em>
      `;
      row.addEventListener("click", () => {
        focusLink(link.name);
        closeSubpartMenu();
      });
      return row;
    }),
  );
}

function formatVolume(value: number | undefined): string {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    return "-";
  }
  if (value >= 1_000_000) {
    return `${(value / 1_000_000).toFixed(2)}e6 mm3`;
  }
  return `${Math.round(value).toLocaleString()} mm3`;
}

function focusLink(linkName: string): void {
  if (!activeGroup) {
    return;
  }
  selectedLinkName = linkName;
  activeGroup.traverse((obj) => {
    const mesh = obj as THREE.Mesh;
    if (!mesh.isMesh) {
      return;
    }
    const material = mesh.material as THREE.MeshStandardMaterial;
    const isSelected = !selectedLinkName || mesh.name === selectedLinkName;
    material.opacity = isSelected ? 1 : 0.22;
    material.transparent = !isSelected;
  });
  subpartListEl.querySelectorAll<HTMLButtonElement>(".subpart-row").forEach((row) => {
    const isSelected = row.dataset.link === selectedLinkName;
    row.classList.toggle("is-selected", isSelected);
    row.setAttribute("aria-pressed", String(isSelected));
  });
  if (!selectedLinkName) {
    frameGroup(activeGroup);
    return;
  }
  const target = activeGroup.getObjectByName(selectedLinkName);
  if (target) {
    document.documentElement.dataset.cadquerySelectedLink = selectedLinkName;
    frameObject(target);
  }
}

function openSubpartMenu(): void {
  if (!activeManifest || activeManifest.links.length === 0) {
    return;
  }
  subpartMenuEl.hidden = false;
  menuButton.setAttribute("aria-expanded", "true");
  document.documentElement.dataset.cadqueryMenu = "open";
  const selected = subpartListEl.querySelector<HTMLButtonElement>(".subpart-row.is-selected");
  const first = selected ?? subpartListEl.querySelector<HTMLButtonElement>(".subpart-row") ?? menuCloseButton;
  first.focus();
}

function closeSubpartMenu(opts: { restoreFocus?: boolean } = {}): void {
  if (subpartMenuEl.hidden) {
    return;
  }
  subpartMenuEl.hidden = true;
  menuButton.setAttribute("aria-expanded", "false");
  delete document.documentElement.dataset.cadqueryMenu;
  if (opts.restoreFocus !== false) {
    menuButton.focus();
  }
}

function resize(): void {
  const rect = viewer.getBoundingClientRect();
  const width = Math.max(rect.width, 320);
  const height = Math.max(rect.height, 320);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height);
  codeEditor.layout();
}

function tick(): void {
  requestAnimationFrame(tick);
  controls.update();
  renderer.render(scene, camera);
}

async function init(): Promise<void> {
  const items = await loadLibraryIndex();
  picker.replaceChildren(
    ...items.map((item) => {
      const option = document.createElement("option");
      option.value = item.id;
      option.textContent = item.name;
      return option;
    }),
  );
  const itemByID = new Map(items.map((item) => [item.id, item]));
  picker.addEventListener("change", () => {
    const selected = itemByID.get(picker.value) ?? items[0];
    void loadItem(selected).catch(showError);
  });
  await loadItem(items[0]);
}

function showError(err: unknown): void {
  console.error(err);
  const message = err instanceof Error ? err.message : String(err);
  statusEl.textContent = message;
  document.documentElement.dataset.cadqueryState = "error";
}

window.addEventListener("resize", resize);
const layoutObserver = new ResizeObserver(resize);
layoutObserver.observe(viewer);
layoutObserver.observe(codeEditorHost);
menuButton.addEventListener("click", openSubpartMenu);
menuCloseButton.addEventListener("click", () => closeSubpartMenu());
document.addEventListener("keydown", (event) => {
  if (event.key === "Escape" && !subpartMenuEl.hidden) {
    closeSubpartMenu();
  }
});
resize();
tick();

void init().catch(showError);
