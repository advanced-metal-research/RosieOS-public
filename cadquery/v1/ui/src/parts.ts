import "./parts.css";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

type PartNode = {
  id: string;
  name: string;
  kind: "assembly" | "group" | "part";
  image?: string;
  children?: PartNode[];
};

type PartItem = {
  id: string;
  name: string;
  group: string;
  assembly: string;
  step_file: string;
  image: string;
  images?: PartImage[];
  vertex_count?: number;
  triangle_count?: number;
  metadata_source?: string;
  origin_m?: number[];
  origin_mm?: number[];
  center_of_gravity_m?: number[];
  center_of_gravity_mm?: number[];
  center_of_gravity_source?: string;
  mass_kg?: number;
  mass_source?: string;
  density_kg_m3?: number;
  density_source?: string;
  volume_cm3?: number;
  area_cm2?: number;
  body_count?: number;
  material_names?: string[];
  joints?: PartJoint[];
  bbox?: {
    min?: number[];
    max?: number[];
  };
  external_shell?: ExternalShellReport;
};

type ExternalShellReport = {
  method?: string;
  mesh?: string;
  mesh_triangle_count?: number;
  triangle_count?: number;
  total_triangle_count?: number;
  triangle_ratio?: number;
  sample_view_count?: number;
  raster_size?: number;
  bbox_error_mm?: number | null;
  bbox?: {
    min?: number[];
    max?: number[];
  };
  notes?: string[];
};

type PartImage = {
  id: string;
  name: string;
  image: string;
  theme_images?: Partial<Record<ThemeName, string>>;
  triangle_count?: number;
  gizmo?: {
    origin_mm?: number[];
    length_mm?: number;
    joints?: PartJoint[];
  };
};

type PartJoint = {
  name: string;
  type?: string;
  parent?: string;
  child?: string;
  origin_m?: number[];
  origin_mm?: number[];
  axis?: number[];
  limit?: string;
  source?: string;
};

type PartsIndex = {
  schema_version: number;
  generated_at: string;
  source_dir: string;
  assembly: string;
  themes?: ThemeName[];
  default_theme?: ThemeName;
  tree: PartNode;
  parts: PartItem[];
};

type ThemeName = "light" | "dark";

const app = document.querySelector<HTMLElement>("#parts-app");
if (!app) {
  throw new Error("missing #parts-app");
}

app.innerHTML = `
  <div class="parts-shell">
    <header class="topbar">
      <div>
        <p class="eyebrow">Rosie Geometry / v1</p>
        <h1>Assembly Parts Images</h1>
      </div>
      <div class="top-actions">
        <button id="theme-toggle" class="theme-toggle" type="button" aria-label="Switch drawing theme">Dark mode</button>
        <p id="status" class="status">Loading parts index</p>
      </div>
    </header>
    <main class="parts-layout">
      <aside class="tree-panel" aria-labelledby="tree-title">
        <p class="eyebrow">Fusion-style tree</p>
        <h2 id="tree-title">Assembly</h2>
        <div id="tree" class="tree"></div>
      </aside>
      <section class="image-panel" aria-labelledby="part-title">
        <div class="image-header">
          <div>
            <p id="part-group" class="eyebrow">Part</p>
            <h2 id="part-title">No part selected</h2>
          </div>
          <a id="image-link" class="image-link" href="#" target="_blank" rel="noreferrer" hidden>Open PNG</a>
        </div>
        <div id="image-wrap" class="image-wrap">
          <p>Select a part from the tree.</p>
        </div>
        <div id="view-strip" class="view-strip" aria-label="Part image views"></div>
        <dl id="metrics" class="metrics"></dl>
        <section id="joints-panel" class="joints-panel" aria-label="Fusion joints"></section>
      </section>
    </main>
  </div>
`;

const statusEl = mustQuery<HTMLElement>("#status");
const treeEl = mustQuery<HTMLElement>("#tree");
const partTitleEl = mustQuery<HTMLElement>("#part-title");
const partGroupEl = mustQuery<HTMLElement>("#part-group");
const imageWrapEl = mustQuery<HTMLElement>("#image-wrap");
const imageLinkEl = mustQuery<HTMLAnchorElement>("#image-link");
const viewStripEl = mustQuery<HTMLElement>("#view-strip");
const metricsEl = mustQuery<HTMLElement>("#metrics");
const jointsPanelEl = mustQuery<HTMLElement>("#joints-panel");
const themeToggleEl = mustQuery<HTMLButtonElement>("#theme-toggle");

let indexData: PartsIndex | null = null;
let selectedPartID = "";
let currentTheme: ThemeName = preferredTheme();
const selectedViewByPart = new Map<string, string>();
let shellViewer: ShellViewerState | null = null;

type ShellViewerState = {
  renderer: THREE.WebGLRenderer;
  controls: OrbitControls;
  animationID: number;
  resizeObserver?: ResizeObserver;
  mesh?: THREE.Object3D;
};

applyTheme();
themeToggleEl.addEventListener("click", () => {
  currentTheme = currentTheme === "dark" ? "light" : "dark";
  window.localStorage.setItem("cadquery-parts-theme", currentTheme);
  applyTheme();
  if (indexData) {
    render();
  }
});
void loadParts();

async function loadParts() {
  try {
    const response = await fetch(assetURL("parts/index.json"), { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`${response.status} ${response.statusText}`);
    }
    indexData = (await response.json()) as PartsIndex;
    selectedPartID = indexData.parts[0]?.id ?? "";
    if (!indexData.themes?.includes(currentTheme) && indexData.default_theme) {
      currentTheme = indexData.default_theme;
      applyTheme();
    }
    statusEl.textContent = `${indexData.parts.length} parts from ${indexData.assembly}`;
    render();
  } catch (error) {
    statusEl.textContent = `Missing parts index: ${error instanceof Error ? error.message : String(error)}`;
    treeEl.innerHTML = `<p class="empty">Run the parts-image command first for a STEP source directory.</p>`;
  }
}

function render() {
  if (!indexData) {
    return;
  }
  treeEl.innerHTML = renderNode(indexData.tree, 0);
  for (const button of treeEl.querySelectorAll<HTMLButtonElement>("[data-part-id]")) {
    button.addEventListener("click", () => {
      selectedPartID = button.dataset.partId ?? selectedPartID;
      render();
    });
  }
  const selected = indexData.parts.find((part) => part.id === selectedPartID) ?? indexData.parts[0] ?? null;
  renderPart(selected);
}

function renderNode(node: PartNode, depth: number): string {
  const children = node.children?.map((child) => renderNode(child, depth + 1)).join("") ?? "";
  if (node.kind === "part") {
    const selected = node.id === selectedPartID;
    return `
      <div class="tree-row depth-${depth}">
        <button type="button" class="part-button${selected ? " selected" : ""}" data-part-id="${escapeAttr(node.id)}">
          <span class="tree-icon">part</span>
          <span>${escapeHTML(node.name)}</span>
        </button>
      </div>
    `;
  }
  return `
    <section class="tree-group depth-${depth}">
      <h3><span class="tree-icon">${node.kind}</span>${escapeHTML(node.name)}</h3>
      ${children}
    </section>
  `;
}

function renderPart(part: PartItem | null) {
  if (!part) {
    disposeShellViewer();
    partTitleEl.textContent = "No part selected";
    imageWrapEl.innerHTML = `<p>No parts in index.</p>`;
    viewStripEl.innerHTML = "";
    metricsEl.innerHTML = "";
    jointsPanelEl.innerHTML = "";
    imageLinkEl.hidden = true;
    return;
  }
  const views = part.images?.length ? part.images : [{ id: "iso", name: "Isometric", image: part.image }];
  const selectedViewID = selectedViewByPart.get(part.id) ?? views[0].id;
  const selectedView = views.find((view) => view.id === selectedViewID) ?? views[0];
  selectedViewByPart.set(part.id, selectedView.id);
  const image = assetURL(`parts/${imageForTheme(selectedView)}`);
  partTitleEl.textContent = part.name;
  partGroupEl.textContent = part.group;
  imageWrapEl.innerHTML = `
    <div class="view-grid">
      ${views
        .map(
          (view) => `
            <button type="button" class="view-tile${view.id === selectedView.id ? " selected" : ""}" data-view-id="${escapeAttr(view.id)}">
              <img src="${escapeAttr(assetURL(`parts/${imageForTheme(view)}`))}" alt="${escapeAttr(part.name)} ${escapeAttr(view.name)} Geometry render">
              <span>${escapeHTML(view.name)}</span>
            </button>
          `,
        )
        .join("")}
    </div>
    <section class="shell-viewer" aria-label="Reduced external shell mesh viewer">
      <div class="shell-viewer-header">
        <div>
          <p class="eyebrow">Reduced shell</p>
          <h3>Orbit external shell mesh</h3>
        </div>
        <p id="shell-viewer-status" class="shell-viewer-status">Loading shell mesh</p>
      </div>
      <div id="shell-viewer-canvas" class="shell-viewer-canvas"></div>
    </section>
  `;
  imageLinkEl.href = image;
  imageLinkEl.hidden = false;
  viewStripEl.innerHTML = "";
  for (const button of imageWrapEl.querySelectorAll<HTMLButtonElement>("[data-view-id]")) {
    button.addEventListener("click", () => {
      selectedViewByPart.set(part.id, button.dataset.viewId ?? selectedView.id);
      renderPart(part);
    });
  }
  metricsEl.innerHTML = `
    ${metric("Assembly", part.assembly)}
    ${metric("View", selectedView.name)}
    ${metric("STEP", part.step_file)}
    ${metric("Source", part.metadata_source ?? "-")}
    ${metric("Origin", formatVector(part.origin_mm, "mm"))}
    ${metric("Center of gravity", formatVector(part.center_of_gravity_mm, "mm"))}
    ${metric("Mass", formatMass(part))}
    ${metric("Density", formatDensity(part))}
    ${metric("Volume", formatNumber(part.volume_cm3, "cm^3"))}
    ${metric("Area", formatNumber(part.area_cm2, "cm^2"))}
    ${metric("Bodies", String(part.body_count ?? "-"))}
    ${metric("Materials", part.material_names?.length ? part.material_names.join(", ") : "-")}
    ${metric("Dimensions", bboxSize(part))}
    ${metric("External shell", formatShell(part.external_shell))}
    ${metric("Shell bbox error", formatShellError(part.external_shell))}
    ${metric("Shell method", part.external_shell?.method ?? "-")}
    ${metric("Vertices", String(part.vertex_count ?? "-"))}
    ${metric("Triangles", String(selectedView.triangle_count ?? part.triangle_count ?? "-"))}
    ${metric("BBox min", formatVector(part.bbox?.min))}
    ${metric("BBox max", formatVector(part.bbox?.max))}
  `;
  jointsPanelEl.innerHTML = renderJoints(part, selectedView);
  renderShellViewer(part);
}

function disposeShellViewer(): void {
  if (!shellViewer) {
    return;
  }
  window.cancelAnimationFrame(shellViewer.animationID);
  shellViewer.resizeObserver?.disconnect();
  shellViewer.controls.dispose();
  shellViewer.mesh?.traverse((obj) => {
    const mesh = obj as THREE.Mesh;
    mesh.geometry?.dispose?.();
    const material = mesh.material as THREE.Material | THREE.Material[] | undefined;
    if (Array.isArray(material)) {
      for (const item of material) {
        item.dispose();
      }
    } else {
      material?.dispose?.();
    }
  });
  shellViewer.renderer.dispose();
  shellViewer.renderer.domElement.remove();
  shellViewer = null;
}

function shellMeshURL(part: PartItem): string | null {
  const mesh = part.external_shell?.mesh;
  if (!mesh) {
    return null;
  }
  const baseDir = part.image.split("/").slice(0, -1).join("/");
  return assetURL(`parts/${baseDir ? `${baseDir}/` : ""}${mesh}`);
}

function renderShellViewer(part: PartItem): void {
  disposeShellViewer();
  const host = document.querySelector<HTMLElement>("#shell-viewer-canvas");
  const status = document.querySelector<HTMLElement>("#shell-viewer-status");
  if (!host || !status) {
    return;
  }
  const url = shellMeshURL(part);
  if (!url) {
    status.textContent = "No reduced shell mesh generated";
    host.innerHTML = `<p class="empty">Regenerate this part with the current <code>parts-image</code> task to create <code>external_shell.stl</code>.</p>`;
    return;
  }
  host.innerHTML = "";
  status.textContent = "Loading shell mesh";

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(currentTheme === "dark" ? 0x070a10 : 0xf5f7fb);
  const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100000);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  host.append(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;

  scene.add(new THREE.HemisphereLight(0xdde8ff, 0x1c2430, 1.7));
  const key = new THREE.DirectionalLight(0xffffff, 2.1);
  key.position.set(3, 5, 4);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0x9bbcff, 0.9);
  fill.position.set(-4, 2, -3);
  scene.add(fill);

  const material = new THREE.MeshStandardMaterial({
    color: currentTheme === "dark" ? 0xc8d0da : 0x6c7888,
    metalness: 0.2,
    roughness: 0.58,
    side: THREE.DoubleSide,
  });

  const resize = () => {
    const rect = host.getBoundingClientRect();
    const width = Math.max(260, Math.floor(rect.width));
    const height = Math.max(260, Math.floor(rect.height));
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  };
  const resizeObserver = new ResizeObserver(resize);
  resizeObserver.observe(host);
  resize();

  const animate = () => {
    controls.update();
    renderer.render(scene, camera);
    if (shellViewer) {
      shellViewer.animationID = window.requestAnimationFrame(animate);
    }
  };

  shellViewer = {
    renderer,
    controls,
    resizeObserver,
    animationID: window.requestAnimationFrame(animate),
  };

  new STLLoader().load(
    url,
    (geometry) => {
      if (!shellViewer || shellViewer.renderer !== renderer) {
        geometry.dispose();
        return;
      }
      geometry.computeBoundingBox();
      geometry.computeVertexNormals();
      const box = geometry.boundingBox;
      if (!box) {
        status.textContent = "Shell mesh has no bounds";
        return;
      }
      const center = box.getCenter(new THREE.Vector3());
      const size = box.getSize(new THREE.Vector3());
      geometry.translate(-center.x, -center.y, -center.z);
      const mesh = new THREE.Mesh(geometry, material);
      scene.add(mesh);
      shellViewer.mesh = mesh;
      const maxDim = Math.max(size.x, size.y, size.z, 1);
      camera.near = Math.max(0.01, maxDim / 10000);
      camera.far = Math.max(1000, maxDim * 12);
      camera.position.set(maxDim * 1.25, -maxDim * 1.6, maxDim * 1.05);
      controls.target.set(0, 0, 0);
      controls.update();
      status.textContent = `${part.external_shell?.triangle_count?.toLocaleString() ?? "-"} shell triangles`;
    },
    undefined,
    (error) => {
      status.textContent = `Shell mesh failed to load: ${error instanceof Error ? error.message : String(error)}`;
    },
  );
}

function preferredTheme(): ThemeName {
  const saved = window.localStorage.getItem("cadquery-parts-theme");
  if (saved === "light" || saved === "dark") {
    return saved;
  }
  return window.matchMedia?.("(prefers-color-scheme: light)").matches ? "light" : "dark";
}

function applyTheme() {
  document.documentElement.dataset.theme = currentTheme;
  themeToggleEl.textContent = currentTheme === "dark" ? "Light mode" : "Dark mode";
  themeToggleEl.setAttribute("aria-label", `Switch to ${currentTheme === "dark" ? "light" : "dark"} mode`);
}

function imageForTheme(view: PartImage): string {
  return view.theme_images?.[currentTheme] ?? view.image;
}

function metric(label: string, value: string): string {
  return `<div><dt>${escapeHTML(label)}</dt><dd>${escapeHTML(value)}</dd></div>`;
}

function renderJoints(part: PartItem, selectedView: PartImage): string {
  const joints = part.joints?.length ? part.joints : selectedView.gizmo?.joints ?? [];
  if (!joints.length) {
    return `<p class="empty">No Fusion joints matched this part in the metadata sidecar.</p>`;
  }
  return `
    <div class="joints-header">
      <p class="eyebrow">Fusion joints</p>
      <h3>${joints.length} matched joint${joints.length === 1 ? "" : "s"}</h3>
    </div>
    <div class="joint-list">
      ${joints.map(renderJoint).join("")}
    </div>
  `;
}

function renderJoint(joint: PartJoint): string {
  return `
    <article class="joint-card">
      <div>
        <p class="joint-type">${escapeHTML(joint.type ?? "unknown")}</p>
        <h4>${escapeHTML(joint.name)}</h4>
      </div>
      <dl>
        ${metric("Origin", formatVector(joint.origin_mm ?? metersToMillimeters(joint.origin_m), "mm"))}
        ${metric("Axis", formatVector(joint.axis))}
        ${metric("Parent", joint.parent ?? "-")}
        ${metric("Child", joint.child ?? "-")}
        ${metric("Limit", joint.limit || "-")}
      </dl>
    </article>
  `;
}

function uiBasePath(): string {
  const viteBase = import.meta.env.BASE_URL || "/";
  if (viteBase !== "/") {
    return viteBase;
  }
  const pathname = window.location.pathname;
  return pathname.endsWith("/") ? pathname : `${pathname}/`;
}

function assetURL(value: string): string {
  const base = uiBasePath().replace(/parts\/?$/, "");
  return new URL(value.replace(/^\/+/, ""), new URL(base, window.location.origin)).toString();
}

function formatVector(value?: number[], unit = ""): string {
  if (!value?.length) {
    return "-";
  }
  const text = value.map((n) => Number(n).toFixed(2)).join(", ");
  return unit ? `${text} ${unit}` : text;
}

function formatNumber(value: number | undefined, unit: string, digits = 3): string {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    return "-";
  }
  return `${value.toLocaleString(undefined, { maximumFractionDigits: digits })} ${unit}`;
}

function formatMass(part: PartItem): string {
  const value = formatNumber(part.mass_kg, "kg", 4);
  return part.mass_source && value !== "-" ? `${value} (${part.mass_source})` : value;
}

function formatDensity(part: PartItem): string {
  const value = formatNumber(part.density_kg_m3, "kg/m^3", 2);
  return part.density_source && value !== "-" ? `${value} (${part.density_source})` : value;
}

function formatShell(shell?: ExternalShellReport): string {
  if (!shell) {
    return "-";
  }
  const count = shell.triangle_count ?? 0;
  const total = shell.total_triangle_count ?? 0;
  const ratio = typeof shell.triangle_ratio === "number" ? `${(shell.triangle_ratio * 100).toFixed(1)}%` : "-";
  const views = shell.sample_view_count ? `, ${shell.sample_view_count} views` : "";
  return `${count.toLocaleString()} / ${total.toLocaleString()} triangles (${ratio}${views})`;
}

function formatShellError(shell?: ExternalShellReport): string {
  if (!shell || typeof shell.bbox_error_mm !== "number" || !Number.isFinite(shell.bbox_error_mm)) {
    return "-";
  }
  return `${shell.bbox_error_mm.toFixed(2)} mm`;
}

function bboxSize(part: PartItem): string {
  const min = part.bbox?.min;
  const max = part.bbox?.max;
  if (!min || !max || min.length < 3 || max.length < 3) {
    return "-";
  }
  return [
    `X ${(max[0] - min[0]).toFixed(1)} mm`,
    `Y ${(max[1] - min[1]).toFixed(1)} mm`,
    `Z ${(max[2] - min[2]).toFixed(1)} mm`,
  ].join(" / ");
}

function metersToMillimeters(value?: number[]): number[] | undefined {
  if (!value?.length) {
    return undefined;
  }
  return value.map((n) => n * 1000);
}

function mustQuery<T extends Element>(selector: string): T {
  const element = document.querySelector<T>(selector);
  if (!element) {
    throw new Error(`missing ${selector}`);
  }
  return element;
}

function escapeHTML(value: string): string {
  return value.replace(/[&<>"']/g, (ch) => {
    switch (ch) {
      case "&":
        return "&amp;";
      case "<":
        return "&lt;";
      case ">":
        return "&gt;";
      case '"':
        return "&quot;";
      default:
        return "&#39;";
    }
  });
}

function escapeAttr(value: string): string {
  return escapeHTML(value);
}
