/**
 * The sphere-model authoring tool.
 *
 * The planner puts spheres on the moving body -- the arm chain and the
 * torch -- and queries them against distance fields. That cover is a
 * per-cell asset nobody has written yet, and `weld_motion_planner.robot_cell.robot.profile` has been
 * reporting it missing since M0. This is where it gets made.
 *
 * Reads the cell through the authoring server's existing routes rather than
 * inventing new ones: `/api/cells` for the descriptor, `/cell-assets/<id>/`
 * for the meshes it names. Both are already proxied by the dev server, and
 * the asset route serves only paths a descriptor actually references.
 *
 * The fit lives in `sphere-fit.ts`; this file is the instrument around it --
 * pick a link, fit it, see what the fit missed, fix it by hand, export.
 * Editing matters as much as fitting: the automatic pass is good at bulk
 * and blunt at the places that decide a plan, which for this robot is the
 * torch and the wrist.
 */

import * as THREE from "three";
import { ColladaLoader } from "three/examples/jsm/loaders/ColladaLoader.js";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { TransformControls } from "three/examples/jsm/controls/TransformControls.js";

import type { FitReport, Sphere } from "./sphere-fit";
import { DEFAULT_FIT, fitSpheres, measure, refineSpheres, type FitOptions } from "./sphere-fit";
import { cellWithBounds } from "./limits";
import {
  linkTransforms as computeLinkTransforms,
  multiply,
  toThreeArray,
  type Cell as KinCell,
  type Mat4,
} from "./kinematics";

interface CellLink {
  name: string;
  visuals: { mesh: string; xyz_m: number[]; rpy_rad: number[]; rgba: number[] }[];
}
interface CellJoint {
  name: string;
  type: string;
  parent: string;
  child: string;
  limits_rad?: { lower: number; upper: number } | null;
}
interface Cell {
  id: string;
  frames: Record<string, string>;
  links: CellLink[];
  joints: CellJoint[];
  problems: string[];
}

const SCHEMA = "amr-weld-planner-v1.sphere-model.v1";
// Readability first. The first cut drew a dark grey mesh at half opacity on
// a near-black field, which on a flat plate like `floor` is invisible: no
// silhouette, no shading, no scale. What reads is a light body against a
// dark ground, with its edges drawn.
const BACKGROUND = 0x171b22;
const MESH = 0xcdd6e3;
const EDGE = 0x5a6577;
const SPHERE = 0x6fb3d2;
const SELECTED = 0xf07167;
const GRID = 0x2a313c;

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;

function fail(message: string): void {
  const box = $("error");
  box.textContent = message;
  box.classList.add("on");
}

/** Link name to its spheres, in that link's own frame. */
const model = new Map<string, Sphere[]>();
/** Position limits as EDITED here, seeded from the cell on load. Kept apart
 *  from `cell.joints[].limits_rad`, which stays the URDF's answer, so the
 *  export can diff the two and rewrite only what actually moved. */
const limits = new Map<string, { lower: number; upper: number }>();
const reports = new Map<string, Omit<FitReport, "spheres">>();
const geometries = new Map<string, Float32Array>();

let cell: Cell | null = null;
let active = "";
let selected = -1;
/** Links on the chain from base to tool: the body that needs covering. */
let armLinks = new Set<string>();

/**
 * The arm chain, derived here the same way `weld_motion_planner.robot_cell.robot.cell.axis_roles`
 * derives it in Python: walk parents from the tool frame to the base.
 *
 * It decides two things on this page. Which links the tool OPENS on --
 * the first version opened on `floor`, a three-metre plate seen edge-on,
 * which is both the least useful link and the worst possible first
 * impression. And which links are marked as needing spheres at all: the
 * fixtures and the second bay are distance fields by design, so their
 * having no spheres is correct rather than an omission.
 */
function deriveArmChain(cell: Cell): Set<string> {
  const parents = new Map(cell.joints.map((j) => [j.child, j]));
  const path: { link: string; joint: CellJoint }[] = [];
  let node = cell.frames.tool;
  const base = cell.frames.base;
  while (node && node !== base && parents.has(node)) {
    const joint = parents.get(node)!;
    path.push({ link: node, joint });
    node = joint.parent;
  }
  // A link is on the arm when at least one REVOLUTE joint lies between it
  // and the base -- not merely when it is on the path. `floor` is bolted to
  // `world` by a fixed joint and came out "arm" until this distinction was
  // made: a three-metre base plate is a distance field, and covering it in
  // spheres would be both wrong and enormous. Walking from the base end,
  // everything at or above the first revolute moves.
  const chain = new Set<string>();
  let moving = false;
  for (let i = path.length - 1; i >= 0; i -= 1) {
    if (path[i].joint.type === "revolute") moving = true;
    if (moving) chain.add(path[i].link);
  }
  return chain;
}

// --------------------------------------------------------------------------

async function main(): Promise<void> {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(BACKGROUND);
  const host = $("viewport");
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.setSize(host.clientWidth, host.clientHeight);
  host.appendChild(renderer.domElement);
  const camera = new THREE.PerspectiveCamera(45, host.clientWidth / host.clientHeight, 0.005, 50);
  const controls = new OrbitControls(camera, renderer.domElement);
  // Three lights, not one. A single directional light leaves every surface
  // facing away from it the same flat tone, which is what made the first
  // version of this page unreadable: a plate lit edge-on had no shading to
  // separate it from the background at all. Key, fill and rim give a body
  // three distinguishable tones from any orbit angle.
  // The first two attempts were too dim — "light colour on dark background"
  // sounds right and still looks black when the hemisphere ground colour is
  // near-black too, because every surface facing AWAY from the key light
  // gets only that ground fill. The fix: a bright hemisphere ground (not
  // the scene background — that is the void) plus strong fill, so the
  // dimmest face on the body is still clearly lighter than the viewport.
  scene.add(new THREE.HemisphereLight(0xf0f4f8, 0x3a4555, 1.2));
  for (const [x, y, z, intensity] of [
    [1.0, 1.4, 0.8, 1.6],   // key
    [-1.2, 0.4, -0.6, 0.7], // fill
    [0.2, -0.8, -1.2, 0.5], // rim
  ] as const) {
    const light = new THREE.DirectionalLight(0xffffff, intensity);
    light.position.set(x, y, z);
    scene.add(light);
  }
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.0;

  const meshGroup = new THREE.Group();
  const sphereGroup = new THREE.Group();
  const helperGroup = new THREE.Group();
  scene.add(meshGroup, sphereGroup, helperGroup);

  /**
   * Ground grid and an origin triad, sized to the link being looked at.
   *
   * Both are scale references, and the triad is more than that: spheres are
   * authored in the LINK's frame, so where that frame's origin sits is the
   * one thing an operator has to keep in view. Without it a link is a shape
   * floating in nothing and a nudge of 2 mm looks the same as a nudge of
   * 20 cm.
   */
  function drawHelpers(size: number): void {
    helperGroup.clear();
    const step = Math.pow(10, Math.round(Math.log10(size / 6)));
    const extent = Math.ceil(size / step) * step * 2;
    const grid = new THREE.GridHelper(extent, Math.round(extent / step), GRID, GRID);
    // The URDF is Z-up and three's GridHelper lies in XZ, so it is turned
    // to sit in the link frame's XY plane -- the floor the operator expects.
    grid.rotation.x = Math.PI / 2;
    (grid.material as THREE.Material).opacity = 0.5;
    (grid.material as THREE.Material).transparent = true;
    helperGroup.add(grid);
    helperGroup.add(new THREE.AxesHelper(Math.max(step, size * 0.35)));
  }

  let cells: { id: string; placeable: boolean }[] = [];
  try {
    const response = await fetch("/api/cells");
    if (!response.ok) throw new Error(`${response.status}`);
    cells = (await response.json()).cells;
  } catch (error) {
    fail(
      `Cannot reach the authoring server for /api/cells (${error}). Start it with ` +
        `\`pixi run serve\`; this tool reads the cell through it.`,
    );
    return;
  }

  const picker = $<HTMLSelectElement>("cell-picker");
  picker.innerHTML = cells.map((c) => `<option value="${c.id}">${c.id}</option>`).join("");
  picker.addEventListener("change", () => void loadCell(picker.value));

  // ---- rendering -------------------------------------------------------

  let drawSpheres = function (): void {
    sphereGroup.clear();
    const showAll = $<HTMLInputElement>("show-all").checked;
    for (const [link, spheres] of model) {
      if (!showAll && link !== active) continue;
      spheres.forEach((sphere, index) => {
        const isSelected = link === active && index === selected;
        const geometry = new THREE.SphereGeometry(sphere.radius, 24, 16);
        const colour = isSelected ? SELECTED : SPHERE;
        const mesh = new THREE.Mesh(
          geometry,
          new THREE.MeshStandardMaterial({
            color: colour,
            roughness: 0.4,
            transparent: true,
            opacity: link === active ? (isSelected ? 0.5 : 0.3) : 0.12,
            depthWrite: false,
          }),
        );
        mesh.position.set(...sphere.center);
        mesh.userData = { link, index };
        sphereGroup.add(mesh);
        // A wireframe shell over the fill. Overlapping translucent spheres
        // otherwise merge into one cloud, and the count -- which is the
        // number being tuned -- becomes impossible to read off the picture.
        const shell = new THREE.LineSegments(
          new THREE.WireframeGeometry(new THREE.SphereGeometry(sphere.radius, 12, 8)),
          new THREE.LineBasicMaterial({
            color: colour,
            transparent: true,
            opacity: isSelected ? 0.8 : 0.28,
          }),
        );
        shell.position.set(...sphere.center);
        sphereGroup.add(shell);
      });
    }
    renderSphereList();
    renderTotals();
  }

  function renderSphereList(): void {
    const host = $("sphere-list");
    const spheres = model.get(active) ?? [];
    $("sphere-count").textContent = String(spheres.length);
    host.innerHTML = "";
    spheres.forEach((sphere, index) => {
      const row = document.createElement("div");
      row.className = `seam${index === selected ? " on" : ""}`;
      const [cx, cy, cz] = sphere.center;
      row.innerHTML =
        `<i class="dot" style="background:${index === selected ? "var(--seam)" : "var(--accent)"}"></i>` +
        `<b>#${index + 1}</b>` +
        `<span class="len" style="font-variant-numeric:tabular-nums">` +
        `r ${(sphere.radius * 1000).toFixed(1)}  ` +
        `(${(cx * 1000).toFixed(0)}, ${(cy * 1000).toFixed(0)}, ${(cz * 1000).toFixed(0)})` +
        `</span>`;
      row.addEventListener("click", () => {
        selected = index;
        $<HTMLInputElement>("radius").value = String(Math.round(sphere.radius * 1000));
        $("radius-readout").textContent = String(Math.round(sphere.radius * 1000));
        drawSpheres();
      });
      host.appendChild(row);
    });
  }

  function renderTotals(): void {
    const total = [...model.values()].reduce((sum, list) => sum + list.length, 0);
    const worst = Math.max(0, ...[...reports.values()].map((r) => r.worstUncoveredM));
    $("totals").textContent =
      `${total} spheres over ${model.size} links · worst uncovered ${(worst * 1000).toFixed(1)} mm`;
    const report = reports.get(active);
    $("fit-report").innerHTML = report
      ? `<div class="fact"><b>method</b><span>${report.method} · ` +
        `${model.get(active)?.length ?? 0} spheres</span></div>` +
        `<div class="fact"><b>uncovered</b><span>${(report.uncoveredFraction * 100).toFixed(1)}%` +
        `${report.worstUncoveredM > 0.001 ? ` · worst ${(report.worstUncoveredM * 1000).toFixed(1)} mm` : ""}</span></div>` +
        `<div class="fact"><b>protrusion</b><span>` +
        `mean ${((report as any).meanProtrusionM * 1000).toFixed(1)} · ` +
        `max ${((report as any).maxProtrusionM * 1000).toFixed(1)} mm</span></div>` +
        `<div class="fact"><b>outside vol</b><span>+${(report.overCoverage * 100).toFixed(1)}% of part</span></div>` +
        `<div class="fact"><b>voxel</b><span>${(report.voxelM * 1000).toFixed(1)} mm · ` +
        `${report.surfaceVoxels} surf / ${report.interiorVoxels} int</span></div>` +
        // Without an interior the waste is measured against a hollow shell
        // and reads near 100% however good the fit is -- a number the
        // operator would otherwise spend the afternoon trying to tune down.
        (report.sealed
          ? ""
          : `<div class="fact warn"><b>hollow</b><span>no interior at this grid — ` +
            `outside-volume % is meaningless. Lower the surface grid, or the mesh does not close.</span></div>`)
      : "";
  }

  function renderLinks(): void {
    const host = $("link-list");
    host.innerHTML = "";
    for (const link of cell?.links ?? []) {
      const spheres = model.get(link.name)?.length ?? 0;
      const report = reports.get(link.name);
      const row = document.createElement("div");
      row.className = `seam${link.name === active ? " on" : ""}`;
      const badges = [
        armLinks.has(link.name) ? `<i class="badge arm">arm</i>` : `<i class="badge">field</i>`,
        !link.visuals.length ? `<i class="badge">no mesh</i>` : "",
        report && report.worstUncoveredM > 0.005
          ? `<i class="badge gap">gap ${(report.worstUncoveredM * 1000).toFixed(0)}mm</i>`
          : "",
      ].join("");
      // Arm links with no spheres are the actual to-do list, so they are
      // the ones the dot marks. A fixture with no spheres is finished work.
      const wanting = armLinks.has(link.name) && !spheres;
      row.innerHTML =
        `<i class="dot" style="background:${wanting ? "var(--seam)" : "var(--torch)"}"></i>` +
        `<b>${link.name}</b>${badges}<span class="len">${spheres}</span>`;
      row.addEventListener("click", () => void selectLink(link.name));
      host.appendChild(row);
    }
  }

  // ---- loading ---------------------------------------------------------

  /**
   * Take a sphere asset into the editor, or say why not.
   *
   * Shared by the committed file this page opens with and by the file picker,
   * so the two cannot drift on what they accept -- a picker that took a
   * document the auto-load rejected would put the operator in a state the
   * page cannot reach again after a reload.
   */
  function adopt(parsed: { schema?: string; links?: unknown }, where: string): boolean {
    if (parsed.schema !== SCHEMA) {
      fail(`${where} says schema ${parsed.schema}, not ${SCHEMA}`);
      return false;
    }
    for (const [link, spheres] of Object.entries(parsed.links ?? {})) {
      model.set(
        link,
        (spheres as { center_m: number[]; radius_m: number }[]).map((s) => ({
          center: [s.center_m[0], s.center_m[1], s.center_m[2]],
          radius: s.radius_m,
        })),
      );
    }
    $("asset-note").textContent = `loaded ${where}`;
    return true;
  }

  async function loadCell(id: string): Promise<void> {
    const response = await fetch(`/api/cells/${id}`);
    cell = (await response.json()) as Cell;
    limits.clear();
    for (const joint of cell.joints) {
      if (joint.type !== "revolute" || !joint.limits_rad) continue;
      limits.set(joint.name, { ...joint.limits_rad });
    }
    model.clear();
    reports.clear();
    geometries.clear();
    for (const link of cell.links) model.set(link.name, []);
    // The committed model for this cell, if there is one. Opening empty on a
    // cell the repo already has spheres for is how a re-authoring session
    // silently drops the links it did not touch: export writes only what is
    // in the editor, so what was never loaded is what gets deleted.
    try {
      const asset = await fetch(`/cell-assets/${id}/spheres.json`);
      if (asset.ok) adopt(await asset.json(), `${id}/spheres.json`);
    } catch {
      /* no server route for it; the file picker still works */
    }
    armLinks = deriveArmChain(cell);
    $("cell-note").textContent = `${cell.links.length} links · ${armLinks.size} on the arm`;
    renderLinks();
    const opening =
      cell.links.find((l) => l.visuals.length && armLinks.has(l.name)) ??
      cell.links.find((l) => l.visuals.length);
    await selectLink(opening?.name ?? cell.links[0].name);
  }

  async function selectLink(name: string): Promise<void> {
    active = name;
    selected = -1;
    gizmo.detach();
    meshGroup.clear();
    const link = cell?.links.find((l) => l.name === name);
    if (!link) return;

    const collected: number[] = [];
    for (const visual of link.visuals) {
      const object = await loadMesh(`/cell-assets/${cell!.id}/${visual.mesh}`);
      if (!object) continue;
      // The visual's own origin, in the link frame. Baked into the vertices
      // rather than kept as a transform: the fit works on one array of
      // points in link coordinates, and a link with two visuals has to be
      // covered as one body.
      const offset = new THREE.Matrix4()
        .makeRotationFromEuler(
          new THREE.Euler(visual.rpy_rad[0], visual.rpy_rad[1], visual.rpy_rad[2], "ZYX"),
        )
        .setPosition(visual.xyz_m[0], visual.xyz_m[1], visual.xyz_m[2]);
      object.updateMatrixWorld(true);
      object.traverse((node) => {
        const mesh = node as THREE.Mesh;
        if (!mesh.isMesh) return;
        const geometry = mesh.geometry.clone().applyMatrix4(mesh.matrixWorld).applyMatrix4(offset);
        const nonIndexed = geometry.index ? geometry.toNonIndexed() : geometry;
        const position = nonIndexed.getAttribute("position");
        for (let i = 0; i < position.count; i += 1) {
          collected.push(position.getX(i), position.getY(i), position.getZ(i));
        }
        // Collada meshes from URDF exporters often arrive without normals,
        // or with normals that are all zero, or with normals that face
        // inward on half the faces. MeshStandardMaterial needs correct
        // normals to shade at all -- which is why every previous attempt
        // came out black despite the lights being fine. Computing them
        // here from the triangles is the only reliable fix.
        nonIndexed.computeVertexNormals();
        meshGroup.add(
          new THREE.Mesh(
            nonIndexed,
            new THREE.MeshPhongMaterial({
              color: MESH,
              specular: 0x444444,
              shininess: 30,
              side: THREE.DoubleSide,
              flatShading: true,
            }),
          ),
        );
        // Feature edges at 1 px. This is what makes a machined part read as
        // a shape rather than a smooth blob, and it is the difference
        // between seeing a flange and seeing a grey mass.
        meshGroup.add(
          new THREE.LineSegments(
            new THREE.EdgesGeometry(nonIndexed, 24),
            new THREE.LineBasicMaterial({ color: EDGE, transparent: true, opacity: 0.75 }),
          ),
        );
      });
    }
    const positions = new Float32Array(collected);
    geometries.set(name, positions);
    meshGroup.visible = $<HTMLInputElement>("show-mesh").checked;

    if (positions.length) {
      const box = new THREE.Box3().setFromBufferAttribute(new THREE.BufferAttribute(positions, 3));
      const centre = box.getCenter(new THREE.Vector3());
      const size = box.getSize(new THREE.Vector3());
      const span = size.length();
      // Frame from the field of view rather than by a guessed multiple of
      // the diagonal: a long thin link and a compact one then both fill the
      // viewport instead of one of them sitting in a corner.
      const distance = (span / 2) / Math.tan((camera.fov * Math.PI) / 360) * 1.5;
      const direction = new THREE.Vector3(0.9, -1.0, 0.65).normalize();
      camera.position.copy(centre).addScaledVector(direction, distance);
      camera.near = Math.max(span / 1000, 1e-4);
      camera.far = distance * 20;
      camera.updateProjectionMatrix();
      controls.target.copy(centre);
      controls.update();
      drawHelpers(span);
      $("cell-note").textContent =
        `${cell!.links.length} links · ${name} is ${size.toArray().map((v) => (v * 1000).toFixed(0)).join(" x ")} mm`;
    } else {
      helperGroup.clear();
    }
    drawAxes();
    renderLinks();
    drawSpheres();
  }

  const meshCache = new Map<string, Promise<THREE.Object3D | null>>();
  function loadMesh(url: string): Promise<THREE.Object3D | null> {
    const cached = meshCache.get(url);
    if (cached) return cached;
    const loading = new Promise<THREE.Object3D | null>((resolve) => {
      new ColladaLoader().load(
        url,
        (result) => {
          // UNDO the loader's up-axis conversion, as `viewer.ts` does and
          // for the same reason: every mesh in this bundle declares
          // Z_UP, the loader rotates for it, and the URDF's own frames are
          // already Z-up. Leaving it on turns each link about its own
          // origin -- which here would fit spheres to a rotated body and
          // look almost right.
          result.scene.rotation.set(0, 0, 0);
          resolve(result.scene);
        },
        undefined,
        () => resolve(null),
      );
    });
    meshCache.set(url, loading);
    return loading;
  }

  // ---- fitting ---------------------------------------------------------

  // Slider readouts — kept live so dragging shows the number. `aspect` is
  // stored as a percent because range inputs cannot step by 0.1 reliably
  // across browsers; it reads back as the ratio the algorithm wants.
  const READOUTS: [string, (v: number) => string][] = [
    ["resolution", (v) => String(v)],
    ["sphere-budget", (v) => String(v)],
    ["max-spheres", (v) => String(v)],
    ["box-resolution", (v) => String(v)],
    ["base-resolution", (v) => String(v)],
    ["levels", (v) => String(v)],
    ["aspect", (v) => (v / 100).toFixed(1)],
  ];
  for (const [id, format] of READOUTS) {
    const slider = $<HTMLInputElement>(id);
    const readout = $(id + "-val");
    const sync = () => {
      readout.textContent = format(Number(slider.value));
      renderGridNote();
    };
    slider.addEventListener("input", sync);
    sync();
  }

  /**
   * Only show the knobs the chosen method reads.
   *
   * The previous panel had three sliders of which one was dead and one
   * only reached candidates that usually lost — so the panel was telling
   * the operator they had control they did not have. Each method now
   * exposes exactly its own parameters.
   */
  function renderModeOptions(): void {
    const method = $<HTMLSelectElement>("method").value;
    for (const row of document.querySelectorAll<HTMLElement>(".mode-opt")) {
      const applies = (row.dataset.for ?? "").split(/\s+/);
      row.style.display = method === "auto" || applies.includes(method) ? "" : "none";
    }
    renderGridNote();
  }

  /**
   * The hierarchy's finest grid is base * 2^(levels-1), which is the number
   * that actually decides how many spheres come out. Deriving it silently
   * is how "base 12, 4 levels" turns into a 96-voxel grid and six thousand
   * spheres without anyone having asked for that.
   */
  function renderGridNote(): void {
    const method = $<HTMLSelectElement>("method").value;
    if (method !== "hierarchy" && method !== "auto") { $("grid-note").textContent = ""; return; }
    const base = Number($<HTMLInputElement>("base-resolution").value);
    const levels = Number($<HTMLInputElement>("levels").value);
    $("grid-note").textContent =
      `hierarchy finest grid: ${base} × 2^${levels - 1} = ${base * 2 ** (levels - 1)} voxels`;
  }

  $("method").addEventListener("change", renderModeOptions);
  renderModeOptions();

  function options(): FitOptions {
    return {
      ...DEFAULT_FIT,
      method: $<HTMLSelectElement>("method").value as FitOptions["method"],
      convexHull: $<HTMLInputElement>("convex-hull").checked,
      resolution: Number($<HTMLInputElement>("resolution").value),
      sphereBudget: Number($<HTMLInputElement>("sphere-budget").value),
      maxSpheres: Number($<HTMLInputElement>("max-spheres").value),
      boxResolution: Number($<HTMLInputElement>("box-resolution").value),
      baseResolution: Number($<HTMLInputElement>("base-resolution").value),
      levels: Number($<HTMLInputElement>("levels").value),
      aspect: Number($<HTMLInputElement>("aspect").value) / 100,
    };
  }

  function fitOne(name: string): void {
    const positions = geometries.get(name);
    if (!positions?.length) return;
    const report = fitSpheres(positions, options());
    model.set(name, report.spheres);
    reports.set(name, report);
  }

  $("fit-link").addEventListener("click", () => {
    fitOne(active);
    selected = -1;
    renderLinks();
    drawSpheres();
  });

  $("fit-all").addEventListener("click", async () => {
    for (const link of cell?.links ?? []) {
      if (!link.visuals.length) continue;
      if (!geometries.has(link.name)) await selectLink(link.name);
      fitOne(link.name);
    }
    renderLinks();
    drawSpheres();
  });

  /**
   * Refine rather than refit: descend from the spheres already on the link.
   *
   * A no-op when the link has none -- refining nothing would silently do
   * nothing, and the operator would reasonably read that as the button
   * being broken rather than as "there is nothing here to refine yet".
   */
  function refineOne(name: string): boolean {
    const positions = geometries.get(name);
    const spheres = model.get(name);
    if (!positions?.length || !spheres?.length) return false;
    const report = refineSpheres(positions, spheres, options());
    model.set(name, report.spheres);
    reports.set(name, report);
    return true;
  }

  $("refine-link").addEventListener("click", () => {
    if (!refineOne(active)) {
      $("fit-report").innerHTML =
        `<div class="fact warn"><b>nothing to refine</b><span>fit the link first, ` +
        `or add a sphere by hand</span></div>`;
      return;
    }
    selected = -1;
    renderLinks();
    drawSpheres();
  });

  $("refine-all").addEventListener("click", async () => {
    for (const link of cell?.links ?? []) {
      if (!model.get(link.name)?.length) continue;
      if (!geometries.has(link.name)) await selectLink(link.name);
      refineOne(link.name);
    }
    selected = -1;
    renderLinks();
    drawSpheres();
  });

  function remeasure(): void {
    const positions = geometries.get(active);
    const spheres = model.get(active);
    if (positions?.length && spheres) {
      reports.set(active, measure(positions, spheres, Number($<HTMLInputElement>("resolution").value)));
    }
  }

  // ---- 3D gizmo ---------------------------------------------------------
  //
  // A translate handle on the selected sphere, exactly like the placement
  // tool in the main app. Dragging moves the sphere and the arrow keys are
  // a fallback for fine adjustment. Without this, positioning a sphere is
  // "type a coordinate and see if it landed right", which is not a tool.

  const gizmoTarget = new THREE.Object3D();
  scene.add(gizmoTarget);
  const gizmo = new TransformControls(camera, renderer.domElement);
  gizmo.setMode("translate");
  gizmo.setSize(0.6);
  let gizmoMode: "translate" | "scale" = "translate";
  /** The radius when the scale drag started, so scale is relative. */
  let scaleBaseRadius = 0;

  gizmo.addEventListener("dragging-changed", (event) => {
    controls.enabled = !(event as unknown as { value: boolean }).value;
    const dragging = (event as unknown as { value: boolean }).value;
    if (dragging && gizmoMode === "scale") {
      // Capture the starting radius so that the uniform scale is applied
      // relative to it, not compounding on every change event.
      const spheres = model.get(active);
      if (spheres && selected >= 0) scaleBaseRadius = spheres[selected].radius;
      gizmoTarget.scale.set(1, 1, 1);
    }
    if (!dragging && gizmoMode === "scale") {
      // Reset scale so next drag starts from 1.
      gizmoTarget.scale.set(1, 1, 1);
    }
  });
  gizmo.addEventListener("objectChange", () => {
    const spheres = model.get(active);
    if (!spheres || selected < 0) return;
    if (gizmoMode === "translate") {
      spheres[selected].center = [
        gizmoTarget.position.x,
        gizmoTarget.position.y,
        gizmoTarget.position.z,
      ];
    } else {
      // Uniform scale → radius. The gizmo scale is relative to 1, so
      // newRadius = baseRadius * uniformScale. We use the average of the
      // three axes to handle any axis the user drags.
      const s = gizmoTarget.scale;
      const uniform = (s.x + s.y + s.z) / 3;
      const newRadius = Math.max(0.001, scaleBaseRadius * uniform);
      spheres[selected].radius = newRadius;
      const mm = Math.round(newRadius * 1000);
      $<HTMLInputElement>("radius").value = String(mm);
      $("radius-readout").textContent = String(mm);
    }
    remeasure();
    drawSpheres();
  });
  const gizmoHelper = gizmo.getHelper?.() ?? (gizmo as unknown as THREE.Object3D);
  scene.add(gizmoHelper);

  function setGizmoMode(mode: "translate" | "scale"): void {
    gizmoMode = mode;
    if (mode === "translate") {
      gizmo.setMode("translate");
      gizmo.setSize(0.6);
    } else {
      gizmo.setMode("scale");
      gizmo.setSize(0.4);
    }
    $("gizmo-move").classList.toggle("active", mode === "translate");
    $("gizmo-scale").classList.toggle("active", mode === "scale");
  }

  function attachGizmo(): void {
    const spheres = model.get(active);
    if (!spheres || selected < 0) {
      gizmo.detach();
      return;
    }
    const sphere = spheres[selected];
    gizmoTarget.position.set(...sphere.center);
    gizmoTarget.scale.set(1, 1, 1);
    gizmo.attach(gizmoTarget);
  }

  $("gizmo-move").addEventListener("click", () => setGizmoMode("translate"));
  $("gizmo-scale").addEventListener("click", () => setGizmoMode("scale"));

  // Patch drawSpheres to also update the gizmo position.
  const _origDrawSpheres = drawSpheres;
  drawSpheres = function () {
    _origDrawSpheres();
    attachGizmo();
  };

  // ---- editing ---------------------------------------------------------

  const raycaster = new THREE.Raycaster();
  renderer.domElement.addEventListener("pointerdown", (event) => {
    // Don't pick a sphere when the user is dragging the gizmo.
    if (gizmo.dragging) return;
    const rect = renderer.domElement.getBoundingClientRect();
    raycaster.setFromCamera(
      new THREE.Vector2(
        ((event.clientX - rect.left) / rect.width) * 2 - 1,
        -((event.clientY - rect.top) / rect.height) * 2 + 1,
      ),
      camera,
    );
    const hit = raycaster.intersectObjects(sphereGroup.children, false)[0];
    if (!hit) return;
    const { link, index } = hit.object.userData as { link: string; index: number };
    if (link !== active) return;
    selected = index;
    const mm = Math.round(model.get(link)![index].radius * 1000);
    $<HTMLInputElement>("radius").value = String(mm);
    $("radius-readout").textContent = String(mm);
    drawSpheres();
  });

  $<HTMLInputElement>("radius").addEventListener("input", () => {
    const spheres = model.get(active);
    if (!spheres || selected < 0) return;
    const mm = Number($<HTMLInputElement>("radius").value);
    spheres[selected].radius = mm / 1000;
    $("radius-readout").textContent = String(mm);
    remeasure();
    drawSpheres();
  });

  $("add-sphere").addEventListener("click", () => {
    const spheres = model.get(active) ?? [];
    spheres.push({ center: [controls.target.x, controls.target.y, controls.target.z], radius: 0.03 });
    model.set(active, spheres);
    selected = spheres.length - 1;
    remeasure();
    renderLinks();
    drawSpheres();
  });

  $("delete-sphere").addEventListener("click", () => {
    const spheres = model.get(active);
    if (!spheres || selected < 0) return;
    spheres.splice(selected, 1);
    selected = -1;
    gizmo.detach();
    remeasure();
    renderLinks();
    drawSpheres();
  });

  addEventListener("keydown", (event) => {
    // T/S to switch gizmo mode without leaving the mouse.
    if (event.key === "t" || event.key === "T") { setGizmoMode("translate"); return; }
    if (event.key === "s" || event.key === "S") { setGizmoMode("scale"); return; }

    const spheres = model.get(active);
    if (!spheres || selected < 0) return;
    const step = event.altKey ? 0.0005 : 0.002;
    const axis = event.shiftKey ? 2 : event.key === "ArrowUp" || event.key === "ArrowDown" ? 1 : 0;
    const sign = event.key === "ArrowRight" || event.key === "ArrowUp" ? 1 : -1;
    if (!["ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown"].includes(event.key)) return;
    event.preventDefault();
    spheres[selected].center[axis] += sign * step;
    gizmoTarget.position.set(...spheres[selected].center);
    remeasure();
    drawSpheres();
  });

  $<HTMLInputElement>("show-mesh").addEventListener("input", () => {
    meshGroup.visible = $<HTMLInputElement>("show-mesh").checked;
  });
  $<HTMLInputElement>("show-all").addEventListener("input", drawSpheres);

  // ---- the asset -------------------------------------------------------

  $("export").addEventListener("click", () => {
    const document_ = {
      schema: SCHEMA,
      cell_id: cell?.id,
      note: "Spheres are in each link's own frame, metres. Authored with tools/sphere_tool.",
      links: Object.fromEntries(
        [...model].filter(([, s]) => s.length).map(([link, spheres]) => [
          link,
          spheres.map((s) => ({
            center_m: s.center.map((v) => Number(v.toFixed(6))),
            radius_m: Number(s.radius.toFixed(6)),
          })),
        ]),
      ),
      coverage: Object.fromEntries(
        [...reports].map(([link, r]) => [
          link,
          {
            uncovered_fraction: Number(r.uncoveredFraction.toFixed(4)),
            worst_uncovered_m: Number(r.worstUncoveredM.toFixed(5)),
            over_coverage: Number(r.overCoverage.toFixed(4)),
            voxel_m: Number(r.voxelM.toFixed(5)),
          },
        ]),
      ),
    };
    const blob = new Blob([JSON.stringify(document_, null, 2)], { type: "application/json" });
    const link = Object.assign(document.createElement("a"), {
      href: URL.createObjectURL(blob),
      download: `${cell?.id ?? "cell"}.spheres.json`,
    });
    link.click();
    URL.revokeObjectURL(link.href);
    $("asset-note").textContent =
      `exported ${link.download} — it belongs in robot_description/${cell?.id ?? "<cell>"}/spheres.json, ` +
      `and its provenance hashes need re-stamping (tools/fit_spheres.py --restamp)`;
  });

  $("import").addEventListener("click", () => $<HTMLInputElement>("import-file").click());
  $<HTMLInputElement>("import-file").addEventListener("change", async (event) => {
    const file = (event.target as HTMLInputElement).files?.[0];
    if (!file) return;
    if (adopt(JSON.parse(await file.text()), file.name)) {
      renderLinks();
      drawSpheres();
    }
  });

  // ---- robot mode --------------------------------------------------------
  //
  // Shows ALL links posed by FK, with spheres on each in world space,
  // and joint sliders to move them. This is where the sphere model gets
  // reality-checked: a cover that looks fine on a single link may overlap
  // a neighbour or leave a gap between two links that the per-link view
  // could never show.

  const robotGroup = new THREE.Group();
  scene.add(robotGroup);
  robotGroup.visible = false;
  let mode: "link" | "robot" = "link";
  const jointValues: Record<string, number> = {};

  /** Cache of loaded THREE meshes per visual URL, as flat position arrays per link. */
  const linkMeshes = new Map<string, THREE.Object3D[]>();

  async function loadAllMeshes(): Promise<void> {
    if (!cell || linkMeshes.size) return;
    for (const link of cell.links) {
      const objects: THREE.Object3D[] = [];
      for (const visual of link.visuals) {
        const obj = await loadMesh(`/cell-assets/${cell.id}/${visual.mesh}`);
        if (!obj) continue;
        const clone = obj.clone(true);
        clone.rotation.set(0, 0, 0); // undo ColladaLoader Z-up again on the clone
        // Bake the visual's own offset into the geometry
        const offset = new THREE.Matrix4()
          .makeRotationFromEuler(
            new THREE.Euler(visual.rpy_rad[0], visual.rpy_rad[1], visual.rpy_rad[2], "ZYX"),
          )
          .setPosition(visual.xyz_m[0], visual.xyz_m[1], visual.xyz_m[2]);
        clone.applyMatrix4(offset);
        clone.traverse((node) => {
          const m = node as THREE.Mesh;
          if (m.isMesh) {
            m.geometry.computeVertexNormals();
            m.material = new THREE.MeshPhongMaterial({
              color: MESH,
              specular: 0x444444,
              shininess: 30,
              side: THREE.DoubleSide,
              flatShading: true,
            });
          }
        });
        objects.push(clone);
      }
      linkMeshes.set(link.name, objects);
    }
  }

  function buildJointSliders(): void {
    const host = $("joint-sliders");
    host.innerHTML = "";
    if (!cell) return;
    for (const joint of cell.joints) {
      if (joint.type !== "revolute") continue;
      jointValues[joint.name] ??= 0;
      const bound = limits.get(joint.name)
        ?? { lower: -Math.PI, upper: Math.PI };
      // Three lines. Name and angle across the top, then the slider with the
      // panel's full width, then the two bounds beneath the ends they belong
      // to. Everything on one line fit the 259 px panel only until a joint was
      // named something longer than "J3", and putting the bounds beside the
      // slider left the slider itself too narrow to drive a joint with any
      // precision -- which is the control this panel is built around.
      const row = document.createElement("div");
      row.className = "joint";
      const head = document.createElement("div");
      head.className = "row joint-head";
      const track = document.createElement("div");
      track.className = "row joint-track";
      const body = document.createElement("div");
      body.className = "row joint-body";
      row.append(head, track, body);
      const readout = document.createElement("span");
      readout.className = "muted";
      readout.style.marginLeft = "auto";
      const slider = document.createElement("input");
      slider.type = "range";
      slider.step = "0.5";

      /** A bound, in degrees, as a number the operator can type over. */
      const field = (side: "lower" | "upper"): HTMLInputElement => {
        const input = document.createElement("input");
        input.type = "number";
        input.step = "0.5";
        input.className = "limit-field";
        input.title = `${side} limit, degrees`;
        input.value = deg(bound[side]).toFixed(1);
        input.addEventListener("change", () => {
          const next = (Number(input.value) * Math.PI) / 180;
          if (!Number.isFinite(next)) return;
          bound[side] = next;
          // A bound that crosses its partner is a joint with no range at all,
          // and the slider silently reports the wrong end of it. Refuse.
          if (bound.lower > bound.upper) {
            [bound.lower, bound.upper] = [bound.upper, bound.lower];
          }
          limits.set(joint.name, bound);
          buildJointSliders();
          clampPose();
          drawRobot();
          reportLimits();
        });
        return input;
      };

      /** Take the bound from where the slider IS. The reason this panel
       *  exists: a limit is found by driving the joint until the arm stops,
       *  not by typing a number at it. */
      const grab = (side: "lower" | "upper"): HTMLButtonElement => {
        const button = document.createElement("button");
        button.textContent = side === "lower" ? "⇤" : "⇥";
        button.className = "limit-grab";
        button.title = `set the ${side} limit to the current angle`;
        button.addEventListener("click", () => {
          bound[side] = jointValues[joint.name];
          limits.set(joint.name, bound);
          buildJointSliders();
          drawRobot();
          reportLimits();
        });
        return button;
      };

      slider.min = String(deg(bound.lower));
      slider.max = String(deg(bound.upper));
      slider.value = String(deg(jointValues[joint.name]));
      readout.textContent = `${deg(jointValues[joint.name]).toFixed(0)}°`;
      slider.addEventListener("input", () => {
        jointValues[joint.name] = (Number(slider.value) * Math.PI) / 180;
        readout.textContent = `${Number(slider.value).toFixed(0)}°`;
        drawRobot();
      });

      const label = document.createElement("span");
      label.textContent = joint.name;
      if (moved(joint)) label.classList.add("edited");
      const lower = document.createElement("span");
      lower.className = "limit-pair";
      lower.append(grab("lower"), field("lower"));
      const upper = document.createElement("span");
      upper.className = "limit-pair";
      upper.append(field("upper"), grab("upper"));
      head.append(label, readout);
      track.append(slider);
      body.append(lower, upper);
      host.appendChild(row);
    }
    reportLimits();
  }

  const deg = (rad: number): number => (rad * 180) / Math.PI;

  /** Whether this joint's bounds differ from the ones the URDF states. A
   *  tenth of a degree is not an edit: the fields round to that, so typing
   *  nothing and tabbing through must not mark the joint changed. */
  function moved(joint: CellJoint): boolean {
    const edited = limits.get(joint.name);
    const urdf = joint.limits_rad;
    if (!edited || !urdf) return false;
    return Math.abs(deg(edited.lower - urdf.lower)) > 0.05
      || Math.abs(deg(edited.upper - urdf.upper)) > 0.05;
  }

  /** Hold every joint inside its own bounds. Narrowing a limit past where the
   *  joint is posed would otherwise leave the robot drawn somewhere the robot
   *  is no longer allowed to be, which is the one picture this panel must not
   *  produce. */
  function clampPose(): void {
    for (const [name, bound] of limits) {
      jointValues[name] = Math.min(Math.max(jointValues[name] ?? 0, bound.lower),
                                   bound.upper);
    }
  }

  function reportLimits(): void {
    const changed = (cell?.joints ?? []).filter(
      (j) => j.type === "revolute" && moved(j));
    $("limit-note").textContent = changed.length
      ? `${changed.length} joint${changed.length > 1 ? "s differ" : " differs"} `
        + `from the URDF: ${changed.map((j) => j.name).join(", ")}`
      : "limits match the URDF";
  }

  /**
   * The cell's own URDF with the edited bounds written into it.
   *
   * A REWRITE of the real file, not a document built from the descriptor: the
   * URDF is the source of truth for position limits and it carries far more
   * than limits -- meshes, materials, transmissions, the comments that record
   * why a number is what it is. Emitting a fresh one from what this page
   * happens to model would silently drop all of it. So the text comes back
   * whole and exactly two attributes move, on exactly the joints that changed.
   */
  function patchedURDF(text: string): { urdf: string; changed: string[] } {
    const changed: string[] = [];
    let urdf = text;
    for (const joint of cell?.joints ?? []) {
      if (joint.type !== "revolute" || !moved(joint)) continue;
      const bound = limits.get(joint.name)!;
      // Anchored to THIS joint's block. A bare `<limit ...>` search would take
      // the first one in the file for every joint.
      const block = new RegExp(
        `(<joint name="${joint.name}"[^>]*>[\\s\\S]*?</joint>)`);
      const found = block.exec(urdf);
      if (!found) continue;
      const patched = found[1].replace(
        /<limit([^>]*?)lower="[^"]*"([^>]*?)upper="[^"]*"/,
        (_whole, before: string, between: string) =>
          `<limit${before}lower="${round(bound.lower)}"${between}` +
          `upper="${round(bound.upper)}"`);
      if (patched === found[1]) continue;
      urdf = urdf.replace(found[1], patched);
      changed.push(joint.name);
    }
    return { urdf, changed };
  }

  /** Radians, at the precision the existing URDFs use. Trailing zeros trimmed
   *  so an unchanged-looking value does not churn the diff. */
  const round = (rad: number): string =>
    String(Number(rad.toFixed(9)));

  $("export-limits").addEventListener("click", () => {
    void (async () => {
      if (!cell) return;
      const response = await fetch(`/cell-urdf/${cell.id}`);
      if (!response.ok) {
        fail(`no /cell-urdf route for ${cell.id} (${response.status}); this ` +
             `needs serve.py, which reads the bundle the descriptor came from`);
        return;
      }
      const { urdf, changed } = patchedURDF(await response.text());
      if (!changed.length) {
        $("limit-note").textContent = "nothing to export: limits match the URDF";
        return;
      }
      const blob = new Blob([urdf], { type: "application/xml" });
      const link = Object.assign(document.createElement("a"), {
        href: URL.createObjectURL(blob),
        download: "robot.urdf",
      });
      link.click();
      URL.revokeObjectURL(link.href);
      // Said here rather than in a README nobody has open: this file is
      // embedded in the pair owner and hashed, and the home profile key is
      // derived from that hash. Editing it is a cell-wide change.
      $("limit-note").textContent =
        `exported robot.urdf with ${changed.join(", ")} changed. It replaces ` +
        `robot_description/${cell.id}/robot.urdf AND the pair-owner copies beside it; ` +
        `pairConfigFactoryURDFSHA256 and the home profile key both move with ` +
        `it, and a cell running this pair needs the revision re-applied.`;
    })();
  });

  $("revert-limits").addEventListener("click", () => {
    if (!cell) return;
    for (const joint of cell.joints) {
      if (joint.type !== "revolute" || !joint.limits_rad) continue;
      limits.set(joint.name, { ...joint.limits_rad });
    }
    clampPose();
    buildJointSliders();
    drawRobot();
  });

  function drawRobot(): void {
    robotGroup.clear();
    drawAxes();
    if (!cell) return;
    const transforms = computeLinkTransforms(cellWithBounds(cell as unknown as KinCell, limits), jointValues);
    for (const link of cell.links) {
      const objects = linkMeshes.get(link.name) ?? [];
      const mat = transforms.get(link.name);
      if (!mat) continue;
      const matrix = new THREE.Matrix4().fromArray(toThreeArray(mat));
      for (const obj of objects) {
        const clone = obj.clone(true);
        clone.applyMatrix4(matrix);
        robotGroup.add(clone);
      }
      // Spheres in world space: transform each sphere's centre by the link transform.
      const spheres = model.get(link.name) ?? [];
      for (const sphere of spheres) {
        const centre = new THREE.Vector3(...sphere.center).applyMatrix4(matrix);
        const geometry = new THREE.SphereGeometry(sphere.radius, 16, 10);
        const mesh = new THREE.Mesh(
          geometry,
          new THREE.MeshPhongMaterial({
            color: armLinks.has(link.name) ? SPHERE : 0x555555,
            transparent: true,
            opacity: 0.3,
            depthWrite: false,
            side: THREE.DoubleSide,
          }),
        );
        mesh.position.copy(centre);
        robotGroup.add(mesh);
        const shell = new THREE.LineSegments(
          new THREE.WireframeGeometry(new THREE.SphereGeometry(sphere.radius, 8, 6)),
          new THREE.LineBasicMaterial({
            color: armLinks.has(link.name) ? SPHERE : 0x555555,
            transparent: true,
            opacity: 0.2,
          }),
        );
        shell.position.copy(centre);
        robotGroup.add(shell);
      }
    }
  }


  // --------------------------------------------------------------------
  // frame axes
  // --------------------------------------------------------------------
  //
  // A cell declares more frames than it draws. `tool0` and `end_effector`
  // carry no mesh at all -- they are the frames a planner aims -- and a link
  // with geometry still has an origin that the spheres beside it are measured
  // from. None of that is visible in a shaded render, so a tool frame can sit
  // 56 mm off the gun it is supposed to be the tip of and every picture of the
  // robot still looks right. That is not hypothetical: rosie_1420_v1 shipped
  // that way until 2026-09-12.
  //
  // So: a triad per frame, in whichever space the current mode works in.

  const axesGroup = new THREE.Group();
  scene.add(axesGroup);

  /** A label that always faces the camera, drawn to a canvas because three
   *  has no text and a whole font loader is not worth one link name. */
  function frameLabel(text: string, height: number): THREE.Sprite {
    const pad = 8;
    const canvas = document.createElement("canvas");
    const context = canvas.getContext("2d")!;
    context.font = "600 42px ui-monospace, monospace";
    canvas.width = Math.ceil(context.measureText(text).width) + pad * 2;
    canvas.height = 42 + pad * 2;
    const ctx = canvas.getContext("2d")!;
    ctx.font = "600 42px ui-monospace, monospace";
    ctx.fillStyle = "rgba(12,14,18,0.72)";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "#e8ecf1";
    ctx.textBaseline = "middle";
    ctx.fillText(text, pad, canvas.height / 2);
    const sprite = new THREE.Sprite(
      new THREE.SpriteMaterial({
        map: new THREE.CanvasTexture(canvas),
        depthTest: false,
        transparent: true,
      }),
    );
    sprite.scale.set((height * canvas.width) / canvas.height, height, 1);
    sprite.renderOrder = 999;
    return sprite;
  }

  /** One triad, plus its name. `depthTest` off: a frame inside a casting is
   *  exactly the one you need to see, and a hidden triad reads as no triad. */
  function frameTriad(
    matrix: Mat4, name: string, length: number, labelSlot = 0,
  ): THREE.Object3D {
    const node = new THREE.Group();
    node.applyMatrix4(new THREE.Matrix4().fromArray(toThreeArray(matrix)));
    for (const [axis, colour] of [
      [new THREE.Vector3(1, 0, 0), 0xff5d5d],
      [new THREE.Vector3(0, 1, 0), 0x6ee787],
      [new THREE.Vector3(0, 0, 1), 0x6ab7ff],
    ] as const) {
      const line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(0, 0, 0),
          axis.clone().multiplyScalar(length),
        ]),
        new THREE.LineBasicMaterial({ color: colour, depthTest: false, transparent: true }),
      );
      line.renderOrder = 998;
      node.add(line);
      const tip = new THREE.Mesh(
        new THREE.ConeGeometry(length * 0.055, length * 0.18, 12),
        new THREE.MeshBasicMaterial({ color: colour, depthTest: false, transparent: true }),
      );
      tip.position.copy(axis.clone().multiplyScalar(length));
      tip.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis);
      tip.renderOrder = 998;
      node.add(tip);
    }
    if ($<HTMLInputElement>("show-axis-labels").checked) {
      const label = frameLabel(name, length * 0.33);
      // Coincident frames stack their labels UP THE WORLD, not up their own
      // +Z. `tool0` sits exactly on `end_effector` by design on both Rosies,
      // and two names drawn at one point render as one unreadable smear
      // ("end_tool0tor"). The lift is built in world axes and rotated back
      // into this node so frames at any orientation still stack the same way.
      const lift = new THREE.Vector3(0, 0, length * (0.22 + 0.42 * labelSlot));
      lift.applyQuaternion(node.quaternion.clone().invert());
      label.position.copy(lift);
      node.add(label);
    }
    return node;
  }

  /** Frames that are RIGIDLY attached to `link`: itself, and anything reached
   *  from it through fixed joints only. In the single-link view those are the
   *  only other frames whose position is knowable, because every revolute
   *  joint below would need a pose this view does not have. */
  function framesFixedTo(link: string): { name: string; matrix: Mat4 }[] {
    const out = [{ name: link, matrix: identityMat() }];
    if (!cell) return out;
    const joints = (cell as unknown as KinCell).joints;
    for (let head = 0; head < out.length; head += 1) {
      for (const joint of joints) {
        if (joint.type !== "fixed" || joint.parent !== out[head].name) continue;
        if (out.some((f) => f.name === joint.child)) continue;
        out.push({
          name: joint.child,
          matrix: multiply(out[head].matrix, jointOrigin(joint)),
        });
      }
    }
    return out;
  }

  const identityMat = (): Mat4 => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

  function jointOrigin(joint: KinCell["joints"][number]): Mat4 {
    const [r, p, y] = joint.rpy_rad;
    const [cr, sr, cp, sp, cy, sy] =
      [Math.cos(r), Math.sin(r), Math.cos(p), Math.sin(p), Math.cos(y), Math.sin(y)];
    return [
      cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, joint.xyz_m[0],
      sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, joint.xyz_m[1],
      -sp, cp * sr, cp * cr, joint.xyz_m[2],
      0, 0, 0, 1,
    ];
  }

  function drawAxes(): void {
    axesGroup.clear();
    axesGroup.visible = $<HTMLInputElement>("show-axes").checked;
    if (!axesGroup.visible || !cell) return;
    const length = Number($<HTMLInputElement>("axis-length").value) / 1000;
    // How many frames already sit at this exact point, so their labels can be
    // stacked instead of overprinted. A tenth of a millimetre is the same
    // place: `tool0` is coincident with `end_effector`, not merely near it.
    const taken = new Map<string, number>();
    const slot = (matrix: Mat4): number => {
      const key = [matrix[3], matrix[7], matrix[11]]
        .map((v) => Math.round(v * 1e4)).join(",");
      const used = taken.get(key) ?? 0;
      taken.set(key, used + 1);
      return used;
    };
    const frames: { name: string; matrix: Mat4 }[] =
      mode === "robot"
        ? [...computeLinkTransforms(cellWithBounds(cell as unknown as KinCell, limits), jointValues)]
            .map(([name, matrix]) => ({ name, matrix }))
        : framesFixedTo(active);
    for (const frame of frames) {
      axesGroup.add(frameTriad(frame.matrix, frame.name, length, slot(frame.matrix)));
    }
  }

  function setMode(m: "link" | "robot"): void {
    mode = m;
    meshGroup.visible = mode === "link" && $<HTMLInputElement>("show-mesh").checked;
    sphereGroup.visible = mode === "link";
    helperGroup.visible = mode === "link";
    robotGroup.visible = mode === "robot";
    $("link-section").style.display = mode === "link" ? "" : "none";
    $("joint-section").style.display = mode === "robot" ? "" : "none";
    $("mode-link").classList.toggle("active", mode === "link");
    $("mode-robot").classList.toggle("active", mode === "robot");
    drawAxes();
    if (mode === "robot") {
      void loadAllMeshes().then(() => {
        buildJointSliders();
        drawRobot();
        // Frame the whole robot
        const box = new THREE.Box3().setFromObject(robotGroup);
        if (!box.isEmpty()) {
          const centre = box.getCenter(new THREE.Vector3());
          const span = box.getSize(new THREE.Vector3()).length();
          const distance = (span / 2) / Math.tan((camera.fov * Math.PI) / 360) * 1.6;
          camera.position.copy(centre).addScaledVector(new THREE.Vector3(0.9, -1.0, 0.65).normalize(), distance);
          camera.near = Math.max(span / 1000, 1e-4);
          camera.far = distance * 20;
          camera.updateProjectionMatrix();
          controls.target.copy(centre);
          controls.update();
        }
      });
    }
  }

  $<HTMLInputElement>("show-axes").addEventListener("input", drawAxes);
  $<HTMLInputElement>("show-axis-labels").addEventListener("input", drawAxes);
  $<HTMLInputElement>("axis-length").addEventListener("input", () => {
    $("axis-length-val").textContent = `${$<HTMLInputElement>("axis-length").value} mm`;
    drawAxes();
  });

  $("mode-link").addEventListener("click", () => setMode("link"));
  $("mode-robot").addEventListener("click", () => setMode("robot"));
  $("reset-joints").addEventListener("click", () => {
    for (const key of Object.keys(jointValues)) jointValues[key] = 0;
    buildJointSliders();
    drawRobot();
  });

  addEventListener("resize", () => {
    camera.aspect = host.clientWidth / host.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(host.clientWidth, host.clientHeight);
  });
  renderer.setAnimationLoop(() => {
    controls.update();
    renderer.render(scene, camera);
  });

  await loadCell(cells[0]?.id ?? "");
}

main().catch((error) => fail(String(error)));
