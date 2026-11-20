#!/usr/bin/env node
// Rebuild the rosie_1420_v1 URDF bundle from fusion_part_export/export_manifest.json.
//
// Mirrors rosie_1400_v3's builder and exists for the same reason: the Fusion
// export is a flat list of world-placed parts joined by fixed joints, which is
// geometry without kinematics. Nothing in it says which pair of links a joint
// separates or which way that joint turns. This file is where that is written
// down, once, so the bundle can be rebuilt from committed files without a CAD
// seat -- rewriting an axis is a `node` run, not a Fusion session.
//
// Reads:  fusion_part_export/export_manifest.json + meshes/*.stl
// Writes: robot.urdf, meshes/*.dae
//
// The bundle ships COLLADA, not the raw STL, for the same reasons v3 does. STL
// carries no material, so every link renders in one flat default colour and the
// arm reads as a single extrusion; and STL is millimetres here, so a consumer
// that misses the URDF's scale attribute gets a robot a thousand times too big
// rather than an obviously broken one. The DAE bakes metres into the
// coordinates and names a per-link colour, so the mesh is correct on its own
// and the URDF needs no scale at all.
import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const bundleDir = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const exportDir = resolve(bundleDir, "fusion_part_export");
const meshOutDir = resolve(bundleDir, "meshes");
const meshScale = 0.001; // Fusion STL millimetres -> URDF metres.

const manifest = JSON.parse(readFileSync(resolve(exportDir, "export_manifest.json"), "utf8"));
const partsByName = new Map(manifest.parts.map((p) => [p.name, p]));

// Bundle link name -> Fusion export part name.
//
// old_base is the pedestal the cell stands on. It is kept as the world-fixed
// root rather than dropped, because the arm's world Z starts at 864 mm and a
// bundle that begins at the shoulder would put the robot on the floor.
// Colours alternate along the chain so neighbouring links read as separate
// bodies at card-art size, where the whole robot is 84x64 and a single grey
// silhouette is unreadable. Same palette as rosie_1400_v3.
const MATERIALS = {
  grey_floor_dark: "0.23 0.25 0.28 1",
  grey_body_light: "0.89 0.90 0.91 1",
  grey_body_mid: "0.60 0.63 0.67 1",
  grey_fixture: "0.65 0.68 0.72 1",
};
const LINKS = [
  { link: "pedestal", part: "old_base_1", material: "grey_floor_dark" },
  { link: "base", part: "base_1", material: "grey_fixture" },
  { link: "link_1", part: "l1_1", material: "grey_body_light" },
  { link: "link_2", part: "l2_1", material: "grey_body_mid" },
  { link: "link_3", part: "l3_1", material: "grey_body_light" },
  { link: "link_4", part: "l4_1", material: "grey_body_mid" },
  { link: "link_5", part: "l5_1", material: "grey_body_light" },
  { link: "link_6", part: "mig_torch_v1_1", material: "grey_body_mid" },
];

// The kinematic chain.
//
// AXES COME FROM FUSION, not from this file. `export-model` now exports the
// design's own joints -- a revolute joint's rotationAxisVector is exactly the
// URDF <axis> -- and `axisFor()` below looks each one up by the pair of
// occurrences it connects. The chain order, the limits and the effort/velocity
// figures are authored here; the direction each joint turns is not.
//
// This replaced six inferred axes, two of which were wrong: J2's sign was
// flipped and J6 was given X when the design says -Z. Both render identically to
// a correct robot in a still image and only diverge once something rotates, which
// is exactly why guessing them was the wrong call.
//
// Limits are still authored: every joint in this design has both ends disabled,
// so Fusion reports no range at all and a URDF needs one.
const JOINTS = [
  { name: "world_to_pedestal", type: "fixed", parent: "world", child: "pedestal" },
  { name: "pedestal_to_base", type: "fixed", parent: "pedestal", child: "base" },
  { name: "J1", type: "revolute", parent: "base", child: "link_1", lower: -3.14159265, upper: 3.14159265, effort: 100, velocity: 2 },
  { name: "J2", type: "revolute", parent: "link_1", child: "link_2", lower: -1.9, upper: 1.9, effort: 100, velocity: 2 },
  { name: "J3", type: "revolute", parent: "link_2", child: "link_3", lower: -2.6, upper: 2.6, effort: 100, velocity: 2 },
  { name: "J4", type: "revolute", parent: "link_3", child: "link_4", lower: -3.14159265, upper: 3.14159265, effort: 50, velocity: 2 },
  { name: "J5", type: "revolute", parent: "link_4", child: "link_5", lower: -2.2, upper: 2.2, effort: 50, velocity: 2 },
  { name: "J6", type: "revolute", parent: "link_5", child: "link_6", lower: -6.283, upper: 6.283, effort: 25, velocity: 3 },
];

// ---------- Fusion's joints ----------

// occurrenceForLink maps a bundle link back to the occurrence path the export
// recorded, so a Fusion joint naming "l2:1" can be resolved to "link_2".
function occurrenceForLink(link) {
  const entry = LINKS.find((l) => l.link === link);
  if (!entry) return "";
  const part = partsByName.get(entry.part);
  return part ? (part.full_path || part.occurrence_name || "") : "";
}

const linkByOccurrence = new Map();
for (const { link } of LINKS) {
  const occurrence = occurrenceForLink(link);
  if (occurrence) linkByOccurrence.set(occurrence, link);
}

// snap cleans up the floating-point dust Fusion returns: an axis reads as
// [1.67e-16, -0.9999999999999998, -9.0e-17], which is -Y with noise. Rounding to
// the nearest integer is safe only because every joint in this design is
// axis-aligned; a skew axis would round to nonsense, so it is checked.
function snap(component) {
  const nearest = Math.round(component);
  if (Math.abs(component - nearest) > 1e-6) {
    throw new Error(`joint axis component ${component} is not axis-aligned; this builder assumes it is`);
  }
  return nearest === 0 ? 0 : nearest;
}

// axisFor finds the Fusion joint connecting a parent/child pair and returns its
// axis. Fusion's occurrenceOne moves relative to occurrenceTwo, so one is the
// URDF child and two is the parent.
//
// A missing joint is fatal rather than defaulted. A default would be a guess
// wearing the costume of exported data, which is the failure this whole change
// exists to remove.
function axisFor(joint) {
  const all = manifest.joints || [];
  if (all.length === 0) {
    throw new Error(
      "export_manifest.json carries no joints. Re-export with a build of " +
      "`rosie autodesk v1 export-model` that captures them, or the axes are unknown.",
    );
  }
  const wantChild = occurrenceForLink(joint.child);
  const wantParent = occurrenceForLink(joint.parent);
  const match = all.find(
    (f) => f.kind === "revolute" && f.occurrence_one === wantChild && f.occurrence_two === wantParent,
  );
  if (!match || !Array.isArray(match.axis)) {
    throw new Error(
      `no Fusion revolute joint connects ${joint.parent} (${wantParent}) to ${joint.child} (${wantChild}); ` +
      `the design has: ${all.map((f) => `${f.name} [${f.occurrence_one} -> ${f.occurrence_two}]`).join(", ")}`,
    );
  }
  const axis = match.axis.map(snap);
  if (axis.every((v) => v === 0)) {
    throw new Error(`Fusion joint ${match.name} reports a zero axis`);
  }
  // The URDF <axis> is in the child link's frame. Every joint origin in this
  // bundle comes out with zero rotation, so child frames are world-aligned and
  // Fusion's world-space axis is directly usable. Verified rather than assumed.
  const rotation = relative(joint.parent, joint.child).r;
  const identity = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  for (let i = 0; i < 9; i++) {
    if (Math.abs(rotation[i] - identity[i]) > 1e-6) {
      throw new Error(
        `${joint.name} has a rotated frame, so Fusion's world-space axis cannot be copied ` +
        `into the URDF unrotated. Transform it into the child frame first.`,
      );
    }
  }
  return { axis, source: match.name };
}

// tool0 is the frame everything downstream plans against. It sits at the torch
// tip, which is the lowest point of the torch mesh in Z.
const TOOL0_PARENT = "link_6";

// ---------- STL in, COLLADA out ----------

function parseStl(path) {
  const buf = readFileSync(path);
  const triCount = buf.readUInt32LE(80);
  if (buf.length === 84 + triCount * 50) {
    const verts = new Array(triCount * 9);
    for (let i = 0; i < triCount; i++) {
      const base = 84 + i * 50 + 12; // skip the per-facet normal
      for (let k = 0; k < 9; k++) verts[i * 9 + k] = buf.readFloatLE(base + k * 4);
    }
    return verts;
  }
  // ASCII fallback. Fusion writes binary, but a hand-edited mesh may not.
  const verts = [];
  for (const line of buf.toString("utf8").split(/\r?\n/)) {
    const m = line.trim().match(/^vertex\s+([\d.eE+-]+)\s+([\d.eE+-]+)\s+([\d.eE+-]+)$/);
    if (m) verts.push(Number(m[1]), Number(m[2]), Number(m[3]));
  }
  return verts;
}

function fmtCoord(x) {
  const v = Number((x * meshScale).toFixed(9));
  return Object.is(v, -0) ? "0" : String(v);
}

function buildDae(name, verts, rgba, stamp) {
  const positionCount = verts.length / 3;
  const triangles = positionCount / 3;
  const coords = new Array(verts.length);
  for (let i = 0; i < verts.length; i++) coords[i] = fmtCoord(verts[i]);
  const indices = Array.from({ length: positionCount }, (_, i) => i).join(" ");
  return `<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor>
      <authoring_tool>gradient-cli urdf/v1 rosie_1420_v1 STL-to-DAE</authoring_tool>
    </contributor>
    <created>${stamp}</created>
    <modified>${stamp}</modified>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_effects>
    <effect id="default-effect">
      <profile_COMMON>
        <technique sid="common">
          <phong>
            <diffuse><color>${rgba}</color></diffuse>
            <specular><color>0.35 0.38 0.42 1</color></specular>
            <shininess><float>18</float></shininess>
          </phong>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_materials>
    <material id="default-material" name="Default Material">
      <instance_effect url="#default-effect"/>
    </material>
  </library_materials>
  <library_geometries>
    <geometry id="${name}-geometry" name="${name}">
      <mesh>
        <source id="${name}-positions">
          <float_array id="${name}-positions-array" count="${verts.length}">${coords.join(" ")}</float_array>
          <technique_common>
            <accessor source="#${name}-positions-array" count="${positionCount}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="${name}-vertices">
          <input semantic="POSITION" source="#${name}-positions"/>
        </vertices>
        <triangles material="default-material" count="${triangles}">
          <input semantic="VERTEX" source="#${name}-vertices" offset="0"/>
          <p>${indices}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="${name}-node" name="${name}">
        <instance_geometry url="#${name}-geometry">
          <bind_material>
            <technique_common>
              <instance_material symbol="default-material" target="#default-material"/>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>
`;
}

// ---------- transform math ----------

function partWorld(linkName) {
  if (linkName === "world") return { r: [1, 0, 0, 0, 1, 0, 0, 0, 1], t: [0, 0, 0] };
  const entry = LINKS.find((l) => l.link === linkName);
  if (!entry) throw new Error(`no LINKS entry for ${linkName}`);
  const part = partsByName.get(entry.part);
  if (!part) throw new Error(`export has no part named ${entry.part} (for link ${linkName})`);
  const m = part.transform;
  // Fusion hands back column axes; the URDF wants a row-major rotation.
  return {
    r: [m.x_axis[0], m.y_axis[0], m.z_axis[0],
        m.x_axis[1], m.y_axis[1], m.z_axis[1],
        m.x_axis[2], m.y_axis[2], m.z_axis[2]],
    t: [...m.origin_m],
  };
}

function transposeMul(a, b) {
  const out = new Array(9).fill(0);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      for (let k = 0; k < 3; k++) out[i * 3 + j] += a[k * 3 + i] * b[k * 3 + j];
  return out;
}

function transposeApply(r, v) {
  return [0, 1, 2].map((i) => r[0 * 3 + i] * v[0] + r[1 * 3 + i] * v[1] + r[2 * 3 + i] * v[2]);
}

// Parent-relative transform: inverse(parent) * child.
function relative(parentName, childName) {
  const p = partWorld(parentName);
  const c = partWorld(childName);
  const d = [c.t[0] - p.t[0], c.t[1] - p.t[1], c.t[2] - p.t[2]];
  return { r: transposeMul(p.r, c.r), t: transposeApply(p.r, d) };
}

function rpyFrom(r) {
  const sy = Math.hypot(r[0], r[3]);
  if (sy > 1e-9) return [Math.atan2(r[7], r[8]), Math.atan2(-r[6], sy), Math.atan2(r[3], r[0])];
  return [Math.atan2(-r[5], r[4]), Math.atan2(-r[6], sy), 0];
}

const n = (v) => (Math.abs(v) < 1e-12 ? 0 : Number(v.toFixed(9)));
const xyz = (v) => v.map(n).join(" ");

// ---------- emit ----------

mkdirSync(meshOutDir, { recursive: true });
const lines = ['<?xml version="1.0"?>', '<robot name="rosie_1420_v1">', '  <link name="world"/>'];

// Stamped from the export rather than from the clock. v3 uses `new Date()`,
// which rewrites every DAE on every rebuild and puts eight meaningless file
// changes in front of whoever reviews the real one. Keying off the newest
// exported_at makes a rebuild byte-identical unless the CAD export actually
// moved.
const stamp = manifest.parts
  .map((p) => p.exported_at)
  .filter(Boolean)
  .sort()
  .pop() || "1970-01-01T00:00:00Z";

let triangleTotal = 0;
const axisReport = [];
for (const { link, part, material } of LINKS) {
  const p = partsByName.get(part);
  if (!p) throw new Error(`export has no part named ${part}`);
  const verts = parseStl(resolve(exportDir, "meshes", p.mesh));
  if (verts.length === 0 || verts.length % 9 !== 0) {
    throw new Error(`${p.mesh} parsed to ${verts.length} coordinates, which is not whole triangles`);
  }
  triangleTotal += verts.length / 9;
  const mesh = `${link}.dae`;
  writeFileSync(resolve(meshOutDir, mesh), buildDae(link, verts, MATERIALS[material], stamp));
  // No scale attribute: the DAE is already metres.
  const g = `<mesh filename="package://rosie_1420_v1/meshes/${mesh}"/>`;
  lines.push(
    `  <link name="${link}">`,
    `    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry>${g}</geometry></visual>`,
    `    <collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry>${g}</geometry></collision>`,
    `  </link>`,
  );
}

for (const j of JOINTS) {
  const rel = relative(j.parent, j.child);
  lines.push(
    `  <joint name="${j.name}" type="${j.type}">`,
    `    <parent link="${j.parent}"/>`,
    `    <child link="${j.child}"/>`,
    `    <origin xyz="${xyz(rel.t)}" rpy="${xyz(rpyFrom(rel.r))}"/>`,
  );
  if (j.type === "revolute") {
    const { axis, source } = axisFor(j);
    axisReport.push(`${j.name}=[${axis.join(",")}] from ${source}`);
    lines.push(
      `    <axis xyz="${axis.join(" ")}"/>`,
      `    <limit lower="${j.lower}" upper="${j.upper}" effort="${j.effort}" velocity="${j.velocity}"/>`,
    );
  }
  lines.push(`  </joint>`);
}

// tool0 at the torch tip, measured from the torch mesh's own bounding box.
const torch = partsByName.get(LINKS.find((l) => l.link === TOOL0_PARENT).part);
const torchWorld = partWorld(TOOL0_PARENT);
const tipWorld = [
  (torch.bounding_box.min_m[0] + torch.bounding_box.max_m[0]) / 2,
  (torch.bounding_box.min_m[1] + torch.bounding_box.max_m[1]) / 2,
  torch.bounding_box.min_m[2],
];
const tipLocal = transposeApply(torchWorld.r, [
  tipWorld[0] - torchWorld.t[0],
  tipWorld[1] - torchWorld.t[1],
  tipWorld[2] - torchWorld.t[2],
]);
lines.push(
  '  <link name="tool0"/>',
  `  <joint name="${TOOL0_PARENT}_to_tool0" type="fixed">`,
  `    <parent link="${TOOL0_PARENT}"/>`,
  `    <child link="tool0"/>`,
  `    <origin xyz="${xyz(tipLocal)}" rpy="0 0 0"/>`,
  `  </joint>`,
  "</robot>",
);

writeFileSync(resolve(bundleDir, "robot.urdf"), lines.join("\n") + "\n");
const driven = JOINTS.filter((j) => j.type === "revolute").length;
console.log(`rosie_1420_v1: ${LINKS.length} links, ${driven} driven joints, ${JOINTS.length + 1} joints total`);
console.log(`  meshes: ${LINKS.length} COLLADA, ${triangleTotal} triangles, metres baked in`);
console.log(`  axes:   ${axisReport.join(", ")}`);
console.log(`  source: ${manifest.active_document} (${manifest.parts.length} exported parts, stamped ${stamp})`);
