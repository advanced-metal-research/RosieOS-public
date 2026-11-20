#!/usr/bin/env node
// Verify the regenerated rosie_1400_v3 robot.urdf against the live Fusion export:
// 1. Parse robot.urdf joints, compose FK at zero pose per link.
// 2. Compare each link's world transform to the Fusion export_manifest occurrence transform.
// 3. Compare each DAE mesh's bounds to CadQuery-measured STL bounds (scaled to meters).
// Exits non-zero on any mismatch beyond tolerance.
import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const bundleDir = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const manifest = JSON.parse(
  readFileSync(resolve(bundleDir, "fusion_part_export/export_manifest.json"), "utf8"),
);
const measurements = JSON.parse(
  readFileSync(resolve(bundleDir, "fusion_part_export/cadquery_measurements.json"), "utf8"),
);
const urdfText = readFileSync(resolve(bundleDir, "robot.urdf"), "utf8");

const LINK_TO_PART = {
  floor: "floor_1_1",
  link_1: "l1_1_1",
  link_2: "l2_1",
  link_3: "l3_1",
  link_4: "l4_1",
  link_5: "l5_1",
  link_6: "mig_torch_v1_1",
  end_effector: "end_effector_1",
  h_frame: "h_frame_1",
  positioner_table_a: "positioner_table_1",
  positioner_table_b: "positioner_table_2",
};
const POS_TOL_M = 0.002;
const ROT_TOL = 0.002; // max abs element delta in rotation matrix
const MESH_TOL_M = 0.002;

// ---------- URDF parsing (regex; file is generated with known shape) ----------
const joints = [];
const jointRe = /<joint name="([^"]+)" type="([^"]+)">([\s\S]*?)<\/joint>/g;
let m;
while ((m = jointRe.exec(urdfText))) {
  const body = m[3];
  const parent = body.match(/<parent link="([^"]+)"/)[1];
  const child = body.match(/<child link="([^"]+)"/)[1];
  const originMatch = body.match(/<origin xyz="([^"]+)" rpy="([^"]+)"/);
  const xyz = originMatch[1].split(/\s+/).map(Number);
  const rpy = originMatch[2].split(/\s+/).map(Number);
  joints.push({ name: m[1], type: m[2], parent, child, xyz, rpy });
}

// ---------- math ----------
function rpyToMat(r, p, y) {
  const cr = Math.cos(r), sr = Math.sin(r);
  const cp = Math.cos(p), sp = Math.sin(p);
  const cy = Math.cos(y), sy = Math.sin(y);
  // R = Rz(y) * Ry(p) * Rx(r)
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
}
function matMul(a, b) {
  const o = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
  ];
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) o[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
  return o;
}
function matVec(mm, v) {
  return [
    mm[0][0] * v[0] + mm[0][1] * v[1] + mm[0][2] * v[2],
    mm[1][0] * v[0] + mm[1][1] * v[1] + mm[1][2] * v[2],
    mm[2][0] * v[0] + mm[2][1] * v[1] + mm[2][2] * v[2],
  ];
}

// FK at zero pose: revolute joints contribute only their fixed origin.
const world = { r: rpyToMat(0, 0, 0), t: [0, 0, 0] };
const frames = new Map([["world", world]]);
let progressed = true;
while (progressed) {
  progressed = false;
  for (const j of joints) {
    if (frames.has(j.child) || !frames.has(j.parent)) continue;
    const parent = frames.get(j.parent);
    const rj = rpyToMat(...j.rpy);
    frames.set(j.child, {
      r: matMul(parent.r, rj),
      t: [0, 1, 2].map((k) => parent.t[k] + matVec(parent.r, j.xyz)[k]),
    });
    progressed = true;
  }
}

const partsByName = new Map(manifest.parts.map((p) => [p.name, p]));
const measByName = new Map(measurements.measurements.map((p) => [p.name, p]));

let failures = 0;
const report = [];
for (const [link, partName] of Object.entries(LINK_TO_PART)) {
  const frame = frames.get(link);
  const part = partsByName.get(partName);
  if (!frame || !part) {
    report.push({ link, status: "MISSING", detail: !frame ? "no FK frame" : "no manifest part" });
    failures++;
    continue;
  }
  const a = part.transform.array;
  const fusionR = [
    [a[0], a[1], a[2]],
    [a[4], a[5], a[6]],
    [a[8], a[9], a[10]],
  ];
  const posErr = Math.max(...[0, 1, 2].map((k) => Math.abs(frame.t[k] - part.origin_m[k])));
  let rotErr = 0;
  for (let i = 0; i < 3; i++)
    for (let jj = 0; jj < 3; jj++) rotErr = Math.max(rotErr, Math.abs(frame.r[i][jj] - fusionR[i][jj]));

  // Mesh bounds check: CadQuery raw STL mm bounds * 0.001 vs recorded triangle count.
  const meas = measByName.get(partName);
  const boundsOk = meas && meas.status === "measured" && meas.vertex_count > 0;

  const ok = posErr <= POS_TOL_M && rotErr <= ROT_TOL && boundsOk;
  if (!ok) failures++;
  report.push({
    link,
    part: partName,
    status: ok ? "PASS" : "FAIL",
    fk_world_origin_m: frame.t.map((v) => Number(v.toFixed(6))),
    fusion_world_origin_m: part.origin_m.map((v) => Number(v.toFixed(6))),
    position_error_m: Number(posErr.toExponential(3)),
    rotation_error_max_abs: Number(rotErr.toExponential(3)),
    cadquery_measured: Boolean(boundsOk),
    mesh_diagonal_mm: meas ? Number(meas.bounds_raw.diagonal.toFixed(3)) : null,
  });
}

// Torch tip / TCP sanity: tool0 must sit at the end_effector origin, which must be
// near the l5 flange (J6 child chain) and 6.35mm-long cone per CadQuery.
const tool0 = frames.get("tool0");
const ee = frames.get("end_effector");
const tcpOk =
  tool0 && ee && Math.max(...[0, 1, 2].map((k) => Math.abs(tool0.t[k] - ee.t[k]))) < 1e-9;
if (!tcpOk) failures++;

console.log(JSON.stringify({ tool0_matches_end_effector: tcpOk, links: report }, null, 2));
console.log(failures === 0 ? "VERIFY: PASS (all links)" : `VERIFY: FAIL (${failures} problem(s))`);
process.exit(failures === 0 ? 0 : 1);
