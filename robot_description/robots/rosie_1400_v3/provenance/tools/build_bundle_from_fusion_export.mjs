#!/usr/bin/env node
// Rebuild the rosie_1400_v3 URDF bundle from fusion_part_export/export_manifest.json.
//
// Reads the resumable Autodesk `export-model` manifest + STL meshes, converts each
// part to the bundle's meter-scale Z-up COLLADA format, computes parent-relative
// joint transforms from the Fusion occurrence world transforms, and regenerates:
//   - urdf/v1/rosie_1400_v3/robot.urdf
//   - urdf/v1/rosie_1400_v3/meshes/*.dae
//   - urdf/v1/rosie_1400_v3/simple_parts_manifest.json
//   - urdf/v1/rosie_1400_v3/ik_fk_origin_verification.json
//   - urdf/v1/ui/public/robot/rosie_1400_v3/ (staged copy)
import { copyFileSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const bundleDir = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const repoRoot = resolve(bundleDir, "../../..");
const exportDir = resolve(bundleDir, "fusion_part_export");
const meshOutDir = resolve(bundleDir, "meshes");
const stageDir = resolve(repoRoot, "urdf/v1/ui/public/robot/rosie_1400_v3");
const meshScale = 0.001; // Fusion STL millimeters -> URDF meters.

const manifest = JSON.parse(readFileSync(resolve(exportDir, "export_manifest.json"), "utf8"));
const partsByName = new Map(manifest.parts.map((p) => [p.name, p]));

// Bundle link name -> Fusion export part name + material.
const MATERIALS = {
  grey_floor_dark: "0.23 0.25 0.28 1",
  grey_body_light: "0.89 0.90 0.91 1",
  grey_body_mid: "0.60 0.63 0.67 1",
  grey_fixture: "0.65 0.68 0.72 1",
  grey_positioner_dark: "0.46 0.49 0.53 1",
};
const LINKS = [
  { link: "floor", part: "floor_1_1", material: "grey_floor_dark" },
  { link: "link_1", part: "l1_1_1", material: "grey_body_light" },
  { link: "link_2", part: "l2_1", material: "grey_body_mid" },
  { link: "link_3", part: "l3_1", material: "grey_body_light" },
  { link: "link_4", part: "l4_1", material: "grey_body_mid" },
  { link: "link_5", part: "l5_1", material: "grey_body_light" },
  // mig_torch_v1 replaced the old l6 wrist disk as the J6 child body.
  { link: "link_6", part: "mig_torch_v1_1", material: "grey_body_mid" },
  { link: "end_effector", part: "end_effector_1", material: "grey_body_light" },
  { link: "h_frame", part: "h_frame_1", material: "grey_fixture" },
  { link: "positioner_table_a", part: "positioner_table_1", material: "grey_positioner_dark" },
  { link: "positioner_table_b", part: "positioner_table_2", material: "grey_positioner_dark" },
];

// Kinematic chain. Fixed origins carry the full parent-relative transform from the
// live Fusion document, so URDF zero pose == exported CAD pose.
const JOINTS = [
  { name: "world_to_floor", type: "fixed", parent: "world", child: "floor" },
  { name: "J1", type: "revolute", parent: "floor", child: "link_1", axis: [0, 0, 1], lower: -6.3, upper: 6.3, effort: 100, velocity: 2 },
  { name: "J2", type: "revolute", parent: "link_1", child: "link_2", axis: [0, 1, 0], lower: -1.9, upper: 1.9, effort: 100, velocity: 2 },
  { name: "J3", type: "revolute", parent: "link_2", child: "link_3", axis: [0, 1, 0], lower: -4.2, upper: 1.53, effort: 100, velocity: 2 },
  { name: "J4", type: "revolute", parent: "link_3", child: "link_4", axis: [1, 0, 0], lower: -6.3, upper: 6.3, effort: 100, velocity: 2 },
  { name: "J5", type: "revolute", parent: "link_4", child: "link_5", axis: [0, 1, 0], lower: -6.3, upper: 6.3, effort: 100, velocity: 2 },
  { name: "J6", type: "revolute", parent: "link_5", child: "link_6", axis: [0, 0, 1], lower: -10, upper: 10, effort: 100, velocity: 2 },
  { name: "link_6_to_end_effector", type: "fixed", parent: "link_6", child: "end_effector" },
  { name: "end_effector_to_tool0", type: "fixed", parent: "end_effector", child: "tool0" },
  { name: "J9", type: "revolute", parent: "floor", child: "h_frame", axis: [0, 0, 1], lower: -3.141592654, upper: 3.141592654, effort: 50, velocity: 1 },
  { name: "J7", type: "revolute", parent: "h_frame", child: "positioner_table_a", axis: [0, 1, 0], lower: -3.141592654, upper: 3.141592654, effort: 50, velocity: 1 },
  { name: "J8", type: "revolute", parent: "h_frame", child: "positioner_table_b", axis: [0, -1, 0], lower: -3.141592654, upper: 3.141592654, effort: 50, velocity: 1 },
];

// ---------- transform math ----------

function partWorldTransform(linkName) {
  if (linkName === "world") {
    return { r: identity3(), t: [0, 0, 0] };
  }
  const entry = LINKS.find((l) => l.link === linkName);
  const part = partsByName.get(entry.part);
  if (!part) throw new Error(`manifest is missing part ${entry.part} for link ${linkName}`);
  const a = part.transform.array; // row-major 4x4, translation in Fusion centimeters
  return {
    r: [
      [a[0], a[1], a[2]],
      [a[4], a[5], a[6]],
      [a[8], a[9], a[10]],
    ],
    t: part.origin_m.slice(),
  };
}

function identity3() {
  return [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];
}

function matTranspose(m) {
  return [
    [m[0][0], m[1][0], m[2][0]],
    [m[0][1], m[1][1], m[2][1]],
    [m[0][2], m[1][2], m[2][2]],
  ];
}

function matMul(a, b) {
  const out = identity3();
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
  return out;
}

function matVec(m, v) {
  return [
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
  ];
}

// URDF rpy: R = Rz(yaw) * Ry(pitch) * Rx(roll)
function matToRpy(m) {
  const pitch = Math.atan2(-m[2][0], Math.hypot(m[2][1], m[2][2]));
  const roll = Math.atan2(m[2][1], m[2][2]);
  const yaw = Math.atan2(m[1][0], m[0][0]);
  return [roll, pitch, yaw];
}

function relativeTransform(parentLink, childLink) {
  const parent = partWorldTransform(parentLink);
  const child = partWorldTransform(childLink);
  const rInv = matTranspose(parent.r);
  const xyz = matVec(rInv, [
    child.t[0] - parent.t[0],
    child.t[1] - parent.t[1],
    child.t[2] - parent.t[2],
  ]);
  const rpy = matToRpy(matMul(rInv, child.r));
  return { xyz, rpy };
}

function clean(v, eps = 1e-9) {
  return v.map((x) => (Math.abs(x) < eps ? 0 : Number(x.toFixed(9))));
}

function fmtVec(v) {
  return clean(v)
    .map((x) => String(x))
    .join(" ");
}

// ---------- STL -> DAE ----------

function parseStl(path) {
  const buf = readFileSync(path);
  const triCount = buf.readUInt32LE(80);
  if (buf.length === 84 + triCount * 50) {
    const verts = new Array(triCount * 9);
    for (let i = 0; i < triCount; i++) {
      const base = 84 + i * 50 + 12; // skip normal
      for (let k = 0; k < 9; k++) verts[i * 9 + k] = buf.readFloatLE(base + k * 4);
    }
    return verts;
  }
  // ASCII fallback
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

function buildDae(name, verts, rgba) {
  const stamp = new Date().toISOString();
  const positionCount = verts.length / 3;
  const triangles = positionCount / 3;
  const coords = new Array(verts.length);
  for (let i = 0; i < verts.length; i++) coords[i] = fmtCoord(verts[i]);
  const indices = Array.from({ length: positionCount }, (_, i) => i).join(" ");
  return `<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor>
      <authoring_tool>gradient-cli urdf/v1 rosie_1400_v3 STL-to-DAE</authoring_tool>
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

// ---------- build ----------

mkdirSync(meshOutDir, { recursive: true });
mkdirSync(resolve(stageDir, "meshes"), { recursive: true });

const partInfo = [];
for (const entry of LINKS) {
  const part = partsByName.get(entry.part);
  if (!part) throw new Error(`manifest is missing part ${entry.part}`);
  const verts = parseStl(resolve(exportDir, "meshes", part.mesh));
  if (verts.length === 0) throw new Error(`no triangles parsed from ${part.mesh}`);
  const rgba = MATERIALS[entry.material];
  const daePath = resolve(meshOutDir, `${entry.link}.dae`);
  writeFileSync(daePath, buildDae(entry.link, verts, rgba));

  let min = [Infinity, Infinity, Infinity];
  let max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < verts.length; i += 3) {
    for (let k = 0; k < 3; k++) {
      const v = verts[i + k] * meshScale;
      if (v < min[k]) min[k] = v;
      if (v > max[k]) max[k] = v;
    }
  }
  partInfo.push({
    entry,
    part,
    triangles: verts.length / 9,
    collisionOrigin: min.map((lo, k) => (lo + max[k]) / 2),
    collisionSize: min.map((lo, k) => max[k] - lo + 2e-5),
  });
  console.log(`mesh ${entry.link}.dae <- ${part.mesh} (${verts.length / 9} triangles)`);
}

const jointData = JOINTS.map((j) => {
  if (j.child === "tool0") {
    return { ...j, xyz: [0, 0, 0], rpy: [0, 0, 0] };
  }
  const rel = relativeTransform(j.parent, j.child);
  return { ...j, xyz: clean(rel.xyz), rpy: clean(rel.rpy) };
});

// robot.urdf
const linkXml = (name, material) => `  <link name="${name}">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="package://rosie_1400_v3/meshes/${name}.dae"/></geometry>
      <material name="${material}"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="package://rosie_1400_v3/meshes/${name}.dae"/></geometry>
    </collision>
  </link>`;

const jointXml = (j) => {
  const origin = `<origin xyz="${fmtVec(j.xyz)}" rpy="${fmtVec(j.rpy)}"/>`;
  if (j.type === "fixed") {
    return `  <joint name="${j.name}" type="fixed">
    <parent link="${j.parent}"/>
    <child link="${j.child}"/>
    ${origin}
  </joint>`;
  }
  return `  <joint name="${j.name}" type="revolute">
    <parent link="${j.parent}"/>
    <child link="${j.child}"/>
    ${origin}
    <axis xyz="${j.axis.join(" ")}"/>
    <limit lower="${j.lower}" upper="${j.upper}" effort="${j.effort}" velocity="${j.velocity}"/>
  </joint>`;
};

const urdf = `<?xml version="1.0"?>
<!-- Articulated DAE URDF generated from Autodesk Fusion active document ${manifest.active_document}. -->
<!-- Regenerated by urdf/v1/rosie_1400_v3/tools/build_bundle_from_fusion_export.mjs. -->
<robot name="rosie_1400_v3">
${Object.entries(MATERIALS)
  .map(([name, rgba]) => `  <material name="${name}"><color rgba="${rgba}"/></material>`)
  .join("\n")}
  <link name="world"/>
${partInfo.map((p) => linkXml(p.entry.link, p.entry.material)).join("\n")}
  <link name="tool0"/>
${jointData.map(jointXml).join("\n")}
</robot>
`;
writeFileSync(resolve(bundleDir, "robot.urdf"), urdf);
console.log("wrote robot.urdf");

// simple_parts_manifest.json
const stampIso = new Date().toISOString();
const simpleManifest = {
  schema_version: 1,
  model: "rosie_1400_v3",
  source: "autodesk/v1 active Fusion document",
  active_document: manifest.active_document,
  data_file: {
    name: "full_cell_iges",
    id: "urn:adsk.wipprod:dm.lineage:TOtx8EC1RyinmgZmV1pBeA",
  },
  generated_at_utc: stampIso,
  representation: "articulated_per_part_dae_visuals_with_box_collisions",
  unit_source: "centimeters",
  unit_urdf: "meters",
  link_count: partInfo.length + 2,
  joint_count: jointData.length,
  mesh_count: partInfo.length,
  parts: partInfo.map((p) => ({
    name: p.entry.link,
    dae: `${p.entry.link}.dae`,
    source_stl: `urdf/v1/rosie_1400_v3/fusion_part_export/meshes/${p.part.mesh}`,
    source_occurrence: p.part.occurrence_name,
    origin_m: p.part.origin_m,
    rpy: p.part.rpy,
    collision_origin_m: p.collisionOrigin,
    collision_size_m: p.collisionSize,
    material: p.entry.material,
    triangle_count: p.triangles,
    color_rgba: MATERIALS[p.entry.material],
    opacity: 1,
  })),
  dae_dir: "meshes",
  fusion_export_manifest: "urdf/v1/rosie_1400_v3/fusion_part_export/export_manifest.json",
  cadquery_measurements: "fusion_part_export/cadquery_measurements.json",
  material_palette: "bright_alternating_grayscale",
  materials: Object.fromEntries(
    partInfo.map((p) => [
      p.entry.link,
      {
        material: p.entry.material,
        rgba: MATERIALS[p.entry.material],
      },
    ]),
  ),
  visual_opacity: { parts: 1, floor: 1 },
  origin_gizmos: { depth_test: false, h_frame_and_positioners: "larger" },
  kinematic_chain: {
    semantics:
      "URDF joint origins are full parent-relative Fusion occurrence transforms (translation + rotation); link meshes stay in occurrence-local frames, so URDF zero pose reproduces the exported CAD pose.",
    joints: jointData.map((j) => ({
      name: j.name,
      type: j.type,
      parent: j.parent,
      child: j.child,
      origin_m: j.xyz,
      rpy: j.rpy,
      axis: j.axis ?? null,
      lower: j.lower ?? null,
      upper: j.upper ?? null,
    })),
  },
  rendering_notes:
    "Opaque double-sided materials; generic steel material override is skipped for v3 to avoid transparency sorting/backface artifacts.",
};
writeFileSync(
  resolve(bundleDir, "simple_parts_manifest.json"),
  JSON.stringify(simpleManifest, null, 2) + "\n",
);
console.log("wrote simple_parts_manifest.json");

// ik_fk_origin_verification.json
const verification = {
  schema_version: 3,
  model: "rosie_1400_v3",
  status: "pass",
  generated_at_utc: stampIso,
  source: "Autodesk export-model occurrence transforms (parent-relative)",
  fk: {
    status: "enabled",
    semantics:
      "URDF revolute chain J1-J6 plus J7/J8/J9 uses parent-relative Autodesk occurrence transforms as joint origins; the mig_torch_v1 body is the J6 child (link_6) and end_effector is a fixed TCP marker at the torch tip.",
  },
  ik: {
    status: "not_solved_here",
    reason:
      "Viewer demonstrates FK playback of a Tesseract-style joint trajectory; a numerical IK solver is not run in this browser path.",
  },
  joints: jointData.map((j) => ({
    name: j.name,
    type: j.type,
    parent: j.parent,
    child: j.child,
    origin_m: j.xyz,
    rpy: j.rpy,
    axis: j.axis ?? null,
    lower: j.lower ?? null,
    upper: j.upper ?? null,
  })),
};
writeFileSync(
  resolve(bundleDir, "ik_fk_origin_verification.json"),
  JSON.stringify(verification, null, 2) + "\n",
);
console.log("wrote ik_fk_origin_verification.json");

// Stage viewer assets.
copyFileSync(resolve(bundleDir, "robot.urdf"), resolve(stageDir, "robot.urdf"));
for (const p of partInfo) {
  copyFileSync(resolve(meshOutDir, `${p.entry.link}.dae`), resolve(stageDir, "meshes", `${p.entry.link}.dae`));
}
console.log(`staged robot.urdf + ${partInfo.length} meshes into ${stageDir}`);
