// Scene snapshots against real headless Chrome and OLP's own renderer: a dog
// box program on the 1430 cell (built by OLP's domain functions in
// fixtures/snapshot-program.mjs), its part mesh, a stand and a dense plan.
//
// What it holds the snapshot to, in absolute units: the PNGs are real PNGs of
// the requested pixel size with a picture in them (not one flat colour, the
// failure a lost WebGL context or an empty scene produces), the welds are lit
// in the views that face them, the scene the page reports is the program's
// (12 part triangles, 24 path points, 140 planned samples, one weld), the
// part sits where the placement put it to within 0.5 mm, and a pose taken
// from the plan is the plan's joints to 1e-9 rad.
//
// Skipped, not failed, on a machine with no Chrome/Edge (set ROSIE_CHROME).
import assert from "node:assert/strict";
import { spawnSync } from "node:child_process";
import { existsSync, readFileSync } from "node:fs";
import { mkdtemp, rm } from "node:fs/promises";
import os from "node:os";
import path from "node:path";
import { test } from "node:test";
import { fileURLToPath } from "node:url";
import { inflateSync } from "node:zlib";

import { findChrome, renderSnapshots, type ShotSpec, type SnapshotInput } from "../src/lib/snapshot/index.js";

function repoRoot(): string {
  let dir = path.dirname(fileURLToPath(import.meta.url));
  while (!(existsSync(path.join(dir, "CLAUDE.md")) && existsSync(path.join(dir, "cli", "v1")))) {
    const parent = path.dirname(dir);
    if (parent === dir) throw new Error("cannot find the repository root");
    dir = parent;
  }
  return dir;
}

interface Png { width: number; height: number; rgba: Uint8Array }

/** Enough of PNG to read what Chrome's canvas writes: 8-bit RGB/RGBA,
 *  non-interlaced, every filter type. */
function decodePng(bytes: Buffer): Png {
  assert.deepEqual([...bytes.subarray(0, 8)], [0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a], "PNG signature");
  let offset = 8;
  let width = 0, height = 0, colorType = 0;
  const idat: Buffer[] = [];
  while (offset < bytes.length) {
    const length = bytes.readUInt32BE(offset);
    const type = bytes.toString("latin1", offset + 4, offset + 8);
    const data = bytes.subarray(offset + 8, offset + 8 + length);
    if (type === "IHDR") {
      width = data.readUInt32BE(0);
      height = data.readUInt32BE(4);
      assert.equal(data[8], 8, "bit depth");
      colorType = data[9]!;
      assert.ok(colorType === 6 || colorType === 2, `colour type ${colorType}`);
      assert.equal(data[12], 0, "interlace");
    } else if (type === "IDAT") {
      idat.push(data);
    }
    offset += 12 + length;
  }
  const channels = colorType === 6 ? 4 : 3;
  const raw = inflateSync(Buffer.concat(idat));
  const stride = width * channels;
  const out = new Uint8Array(width * height * 4);
  const previous = new Uint8Array(stride);
  const line = new Uint8Array(stride);
  for (let y = 0; y < height; y++) {
    const filter = raw[y * (stride + 1)]!;
    const source = raw.subarray(y * (stride + 1) + 1, (y + 1) * (stride + 1));
    for (let x = 0; x < stride; x++) {
      const a = x >= channels ? line[x - channels]! : 0;
      const b = previous[x]!;
      const c = x >= channels ? previous[x - channels]! : 0;
      const p = a + b - c;
      const paeth = Math.abs(p - a) <= Math.abs(p - b) && Math.abs(p - a) <= Math.abs(p - c) ? a : Math.abs(p - b) <= Math.abs(p - c) ? b : c;
      const predictor = [0, a, b, (a + b) >> 1, paeth][filter];
      assert.notEqual(predictor, undefined, `filter ${filter}`);
      line[x] = (source[x]! + predictor!) & 0xff;
    }
    for (let x = 0; x < width; x++) {
      out.set([line[x * channels]!, line[x * channels + 1]!, line[x * channels + 2]!, channels === 4 ? line[x * channels + 3]! : 255], (y * width + x) * 4);
    }
    previous.set(line);
  }
  return { width, height, rgba: out };
}

/** Standard deviation of luma, 0..255. */
function lumaSpread(png: Png): number {
  let sum = 0, squares = 0;
  const n = png.width * png.height;
  for (let i = 0; i < n; i++) {
    const luma = 0.2126 * png.rgba[i * 4]! + 0.7152 * png.rgba[i * 4 + 1]! + 0.0722 * png.rgba[i * 4 + 2]!;
    sum += luma;
    squares += luma * luma;
  }
  return Math.sqrt(squares / n - (sum / n) ** 2);
}

/** Pixels in the robot's dark link greys (urdf.ts meshColor: 0x3f474e ..
 *  0x6b737a, lit); the part, stand, grid and background are all lighter. */
function robotPixels(png: Png): number {
  let count = 0;
  for (let i = 0; i < png.width * png.height; i++) {
    if (png.rgba[i * 4]! < 110 && png.rgba[i * 4 + 1]! < 120 && png.rgba[i * 4 + 2]! < 130) count++;
  }
  return count;
}

/** Pixels in the weld highlight's signal yellow (0xffd400 at 0.75 over the
 *  grey viewport): strong red, mid-high green, little blue. */
function weldYellowPixels(png: Png): number {
  let count = 0;
  for (let i = 0; i < png.width * png.height; i++) {
    const [r, g, b] = [png.rgba[i * 4]!, png.rgba[i * 4 + 1]!, png.rgba[i * 4 + 2]!];
    if (r > 200 && g > 150 && b < 110 && r - b > 120) count++;
  }
  return count;
}

const chrome = (() => {
  try {
    return findChrome();
  } catch {
    return null;
  }
})();

// The scene is the 1430 cell's: a real robot's description, present in the private
// repository and left out of the public mirror.
const armDescription = existsSync(path.join(repoRoot(), "robot_description", "robots", "rosie_1420_v1", "robot.urdf"));

test("snapshots draw the program's robot, part, weld and plan from each camera", {
  skip: !chrome ? "no Chrome or Edge found (set ROSIE_CHROME)" : !armDescription ? "private robot description rosie_1420_v1 not present" : false,
  timeout: 240_000,
}, async () => {
  const root = repoRoot();
  const work = await mkdtemp(path.join(os.tmpdir(), "rosie-snapshot-test-"));
  try {
    const built = spawnSync(process.execPath, [path.join(root, "cli", "v1", "scripts", "build-snapshot.mjs")], {
      env: { ...process.env, ROSIE_SNAPSHOT_OUT: path.join(work, "page") }, encoding: "utf8",
    });
    assert.equal(built.status, 0, `build-snapshot failed:\n${built.stderr}`);
    const generated = spawnSync(process.execPath, [path.join(root, "cli", "v1", "test", "fixtures", "snapshot-program.mjs")], {
      encoding: "utf8", maxBuffer: 64 << 20,
    });
    assert.equal(generated.status, 0, `the program generator failed:\n${generated.stderr}`);
    const fixture = JSON.parse(generated.stdout) as SnapshotInput & {
      expect: { weldNodeId: string; rows: number; triangles: number; denseSamples: number };
    };
    const { expect, ...input } = fixture;

    const width = 960;
    const height = 600;
    const shots: ShotSpec[] = [
      { camera: "iso", pose: { sample: "start" } },
      { camera: "front", pose: { row: expect.weldNodeId } },
      { camera: "top", pose: { sample: "end" } },
      { camera: "left", pose: { sample: 0.5 } },
      { camera: "home" },
      { camera: "part", partFocus: true },
    ];
    const started = Date.now();
    const result = await renderSnapshots(input, shots, {
      outDir: path.join(work, "png"), width, height, snapshotDir: path.join(work, "page"), chrome: chrome!,
    });
    const elapsedMs = Date.now() - started;
    console.log(`rendered ${shots.length} shots in ${elapsedMs} ms (${JSON.stringify(result.timingsMs)}) -> ${path.join(work, "png")}`);
    if (process.env.ROSIE_SNAPSHOT_KEEP) console.log(JSON.stringify(result, null, 2));

    // The scene is the program's.
    assert.equal(result.scene.modelId, "rosie_1420_v1");
    assert.equal(result.scene.partTriangles, expect.triangles);
    assert.equal(result.scene.fixtures, 1, "the stand");
    assert.equal(result.scene.pathPoints, expect.rows);
    assert.equal(result.scene.plannedPathPoints, expect.denseSamples);
    assert.equal(result.scene.plannedStale, false);
    assert.equal(result.scene.welds, 1);
    // Dog box CAD bounds (620..950, -190..190, 610..760 mm) placed with its
    // base centre at the stand top, world (1100, 400, 600) mm.
    const [partMin, partMax] = result.scene.boundsMm.part!;
    for (const [axis, expected] of [935, 210, 600].entries()) assert.ok(Math.abs(partMin[axis]! - expected) < 0.5, `part min[${axis}] ${partMin[axis]} mm`);
    for (const [axis, expected] of [1265, 590, 750].entries()) assert.ok(Math.abs(partMax[axis]! - expected) < 0.5, `part max[${axis}] ${partMax[axis]} mm`);
    assert.deepEqual(result.warnings, []);

    // Poses: the plan starts at zero joints; the weld's row pose is its
    // segment's first sample (fixture ABOVE: -0.35, 0.55, 0.35, 0, 0.75, 0).
    const [start, weldRow, end] = result.files;
    assert.deepEqual(start!.jointOrder, ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]);
    for (const value of start!.jointsRad) assert.ok(Math.abs(value) < 1e-9);
    [-0.35, 0.55, 0.35, 0, 0.75, 0, 0, 0, 0].forEach((expected, axis) => assert.ok(Math.abs(weldRow!.jointsRad[axis]! - expected) < 1e-9, `weld pose J${axis + 1}`));
    for (const value of end!.jointsRad) assert.ok(Math.abs(value) < 1e-9, "the plan ends at zero joints");

    // Every PNG: the size asked for, a picture in it.
    assert.equal(result.files.length, shots.length);
    for (const file of result.files) {
      const png = decodePng(readFileSync(file.path));
      assert.equal(png.width, width, file.path);
      assert.equal(png.height, height, file.path);
      assert.ok(lumaSpread(png) > 8, `${path.basename(file.path)} is nearly one flat colour (luma sd ${lumaSpread(png).toFixed(1)})`);
    }
    // The robot is drawn in the first shot, not only once its meshes trickle
    // in: ~31,800 px at 960x600 measured; a robot missing its links (the
    // double-load race in snapshot.ts) left 1,480.
    const first = decodePng(readFileSync(result.files[0]!.path));
    assert.ok(robotPixels(first) > 10_000, `first shot shows no robot (${robotPixels(first)} px)`);
    // The weld is lit in the views that face the part.
    for (const index of [0, 1, 2]) {
      const png = decodePng(readFileSync(result.files[index]!.path));
      assert.ok(weldYellowPixels(png) > 50, `${path.basename(result.files[index]!.path)}: weld not visible (${weldYellowPixels(png)} px)`);
    }
    // Named views frame the subject: the camera looks at the union's centre.
    const iso = result.files[0]!;
    assert.ok(iso.cameraPositionM[0]! > iso.cameraTargetM[0]! && iso.cameraPositionM[1]! < iso.cameraTargetM[1]!, "iso looks from +X, -Y");
  } finally {
    if (!process.env.ROSIE_SNAPSHOT_KEEP) await rm(work, { recursive: true, force: true });
    else console.log(`kept ${work}`);
  }
});
