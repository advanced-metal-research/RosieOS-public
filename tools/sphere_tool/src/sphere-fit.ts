/**
 * Fitting spheres to a link mesh by covering its SURFACE voxels.
 *
 * ## Why surface, not interior
 *
 * The first version voxelized, flood-filled the outside, distance-
 * transformed the inside, and placed spheres at the deepest interior
 * points. That fails on every mesh that is not watertight -- the flood
 * leaks and the interior vanishes -- and on thin plates, whose interior is
 * zero voxels deep at any practical resolution.
 *
 * CAD link meshes from URDF exporters are routinely not watertight, and
 * half the links on a welding robot are thin plates or tubes. An algorithm
 * that requires closure to work is one that silently fails on the majority
 * of inputs and produces a fit that looks plausible and covers nothing.
 *
 * Surface voxelization needs no inside/outside: mark every voxel a
 * triangle passes through, then cover THOSE. The coverage metric becomes
 * "what fraction of the surface is within some sphere's radius", which is
 * directly what the collision check cares about.
 *
 * ## The algorithm
 *
 * 1. Voxelize: mark surface voxels from the triangle soup.
 * 2. Place: greedily pick the centre that covers the most uncovered surface
 *    voxels within a candidate radius. The candidate radius is the distance
 *    from that centre to the farthest surface voxel it covers, shrunk
 *    slightly so the sphere does not bulge past the surface everywhere.
 * 3. Adjust: for each placed sphere, set its radius to the distance from
 *    its centre to the farthest surface voxel it should cover — which is
 *    the voxels assigned to it after all spheres are placed.
 *
 * ## Conservative is the direction that matters
 *
 * Uncovered surface = the planner's search cannot see that part of the
 * robot. Over-coverage = plans are conservative. The CCD verifier re-checks
 * the winner against exact meshes, so neither is a safety argument.
 */

import { Vector3 } from "three";
import { ConvexGeometry } from "three/examples/jsm/geometries/ConvexGeometry.js";

export interface Sphere {
  center: [number, number, number];
  radius: number;
}

/**
 * How far past the farthest voxel CENTRE a sphere must reach to have
 * genuinely covered that voxel, in units of the voxel size.
 *
 * Coverage is evaluated at voxel centres, so a radius that just touches the
 * farthest centre it owns reports 100% at the grid it was fitted on -- and
 * misses the surface everywhere the real geometry sits toward a corner of
 * that voxel. The slack used to be 0.5, which covers only the axis
 * directions; the corner of a cube is sqrt(3)/2 away.
 *
 * Measured before the fix, fitting link_2 at grid 48 and re-checking:
 * 0.00% uncovered at 48, 9.7% at 80. The fit was not complete, it was
 * complete AT ITS OWN RESOLUTION, which is not the same claim and is not
 * the one the operator reads.
 */
const COVER_SLACK = Math.sqrt(3) / 2;

/**
 * Which strategy to run. They do not dominate each other and they do not
 * have the same knobs, which is why the method is chosen rather than
 * inferred: a fit that quietly switched families would also quietly switch
 * which sliders mean anything.
 *
 * - `cluster`   k-means on surface and interior. The count is exactly what
 *               you ask for. Wastes more on bodies with real depth.
 * - `boxes`     single-resolution box decomposition, one sphere per box.
 *               Wastes least per sphere; the count is emergent and large.
 * - `hierarchy` coarse-to-fine box decomposition. The bulk is claimed by a
 *               few big boxes and only the skin descends to the fine grid,
 *               so it lands between the two.
 * - `auto`      run all three and keep whichever wastes least at 100%
 *               coverage, after each has been cut to the budget.
 */
export type FitMethod = "auto" | "cluster" | "boxes" | "hierarchy";

export interface FitOptions {
  method: FitMethod;
  /**
   * Longest-axis voxel count of the MEASUREMENT grid: the surface and
   * occupancy every method is scored against. Not the decomposition grid --
   * raising it makes the numbers finer, not the fit denser.
   */
  resolution: number;
  /**
   * Hard cap on the sphere count, for every method. Box decompositions
   * produce however many boxes the body needs -- 6045 on link_1 at
   * resolution 48 -- which is a fine answer to "waste least" and a useless
   * one to ship. Enforced by merging, so coverage survives it.
   */
  sphereBudget: number;
  /** `cluster`: how many clusters to start from (still capped by the budget). */
  maxSpheres: number;
  /**
   * `boxes` / `hierarchy`: how far from cubic a merged box may grow. The
   * sphere around a box has radius = half its DIAGONAL, so a 1x1x20 run
   * becomes a ball of radius 10 covering almost nothing. 1.0 is cubic and
   * wastes least; higher merges harder and uses fewer spheres.
   */
  aspect: number;
  /** `boxes`: voxel resolution of the decomposition grid. */
  boxResolution: number;
  /** `hierarchy`: the COARSEST voxel resolution -- where the first, biggest boxes come from. */
  baseResolution: number;
  /** `hierarchy`: octaves above the base. The finest grid is base * 2^(levels-1). */
  levels: number;
  /**
   * Fit the mesh's CONVEX HULL instead of the mesh.
   *
   * The hull fills every concavity, so it is a strict over-approximation:
   * a sphere set covering the hull covers the part, always. That buys a far
   * simpler body to cover -- no pockets, no thin webs, no interior the
   * flood fill can leak out of -- which usually means fewer spheres and a
   * faster, more stable fit.
   *
   * The price is paid in exactly the currency the fit is trying to save:
   * everything the hull added is volume outside the part, and on a link
   * shaped like a fork or a C that is most of it. Which is why the report
   * still measures against the REAL mesh with this on. The two settings
   * are directly comparable, and the number tells you whether the link is
   * convex enough to be worth it.
   */
  convexHull: boolean;
}

export const DEFAULT_FIT: FitOptions = {
  method: "auto",
  resolution: 48,
  sphereBudget: 50,
  maxSpheres: 30,
  aspect: 1.2,
  boxResolution: 24,
  baseResolution: 12,
  levels: 3,
  convexHull: false,
};

/**
 * The mesh's convex hull, as a triangle soup in the same layout.
 *
 * Returns the input unchanged if the hull cannot be built -- a degenerate
 * or coplanar visual should fall back to fitting the mesh, not throw in the
 * middle of a fit the operator asked for.
 */
export function convexHullPositions(positions: Float32Array): Float32Array {
  if (positions.length < 12) return positions;
  const points: Vector3[] = [];
  for (let i = 0; i < positions.length; i += 3)
    points.push(new Vector3(positions[i], positions[i + 1], positions[i + 2]));
  try {
    const geometry = new ConvexGeometry(points);
    const attribute = geometry.getAttribute("position");
    if (!attribute || attribute.count < 3) return positions;
    const out = new Float32Array(attribute.count * 3);
    for (let i = 0; i < attribute.count; i += 1) {
      out[i * 3] = attribute.getX(i);
      out[i * 3 + 1] = attribute.getY(i);
      out[i * 3 + 2] = attribute.getZ(i);
    }
    return out;
  } catch {
    return positions;
  }
}

export interface FitReport {
  spheres: Sphere[];
  /** Which strategy produced this -- the point of `auto` is seeing what won. */
  method: string;
  /** Surface voxels not covered by any sphere. */
  uncoveredFraction: number;
  /** Distance (m) from the worst uncovered surface voxel to the nearest sphere. */
  worstUncoveredM: number;
  /**
   * The quantity to minimise: how much volume the sphere model adds outside
   * the part, as a fraction of the part's own volume.
   *
   * Measured on the UNION of the spheres. An earlier version summed each
   * sphere's volume separately and reported outside/total across that sum,
   * which double-counted every overlap -- and since overlaps are mostly
   * INSIDE the body, they padded the denominator and flattered exactly the
   * fits that used the most spheres. It also meant that dropping a
   * redundant sphere buried inside the part, which cannot change the model's
   * shape at all, made the number get worse. Both are gone: the union does
   * not care how many spheres draw it.
   */
  overCoverage: number;
  /** The farthest any sphere surface extends past the nearest mesh surface, metres. */
  maxProtrusionM: number;
  /** Mean protrusion across all spheres. */
  meanProtrusionM: number;
  voxelM: number;
  surfaceVoxels: number;
  interiorVoxels: number;
  /**
   * Whether the body enclosed an interior at this resolution.
   *
   * When it did not, `overCoverage` is measured against a hollow shell and
   * reads near 100% no matter how good the fit is. That is worth saying out
   * loud rather than leaving as a number the operator would reasonably try
   * to tune down. It happens when the grid is too coarse to resolve any
   * thickness, or when the mesh genuinely does not close.
   */
  sealed: boolean;
}

// --------------------------------------------------------------------------

interface Grid {
  nx: number;
  ny: number;
  nz: number;
  voxel: number;
  origin: [number, number, number];
  /** 1 = surface voxel, 0 = empty. */
  surface: Uint8Array;
  /** Centres of surface voxels, for distance queries. */
  surfaceCentres: Float32Array; // [n, 3] flat
  surfaceCount: number;
  /**
   * Interior voxels and how deep each one is. Needed to sink sphere
   * centres into the body: a centre on the surface protrudes by its whole
   * radius, one at depth d protrudes by radius - d.
   *
   * Computed by flood-filling the outside, which is not reliable on a
   * non-watertight mesh -- but unlike the first version of this file, a
   * leak here only means the deep-move finds nothing and the fit falls
   * back to surface-centred spheres. It degrades rather than failing.
   */
  interiorCentres: Float32Array;
  interiorDepth: Float32Array;
  interiorCount: number;
}

function voxelize(positions: Float32Array, resolution: number): Grid {
  // Bounding box with one voxel of padding.
  let lo: [number, number, number] = [Infinity, Infinity, Infinity];
  let hi: [number, number, number] = [-Infinity, -Infinity, -Infinity];
  for (let v = 0; v < positions.length; v += 3) {
    for (let a = 0; a < 3; a += 1) {
      lo[a] = Math.min(lo[a], positions[v + a]);
      hi[a] = Math.max(hi[a], positions[v + a]);
    }
  }
  const span = Math.max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2], 1e-9);
  const voxel = span / Math.max(8, resolution);
  lo = [lo[0] - voxel, lo[1] - voxel, lo[2] - voxel];
  hi = [hi[0] + voxel, hi[1] + voxel, hi[2] + voxel];

  const nx = Math.max(3, Math.ceil((hi[0] - lo[0]) / voxel));
  const ny = Math.max(3, Math.ceil((hi[1] - lo[1]) / voxel));
  const nz = Math.max(3, Math.ceil((hi[2] - lo[2]) / voxel));
  const surface = new Uint8Array(nx * ny * nz);

  const mark = (x: number, y: number, z: number) => {
    const i = Math.floor((x - lo[0]) / voxel);
    const j = Math.floor((y - lo[1]) / voxel);
    const k = Math.floor((z - lo[2]) / voxel);
    if (i >= 0 && j >= 0 && k >= 0 && i < nx && j < ny && k < nz)
      surface[(i * ny + j) * nz + k] = 1;
  };

  // Rasterize each triangle densely enough that no voxel it passes through
  // is missed.
  //
  // The step count must FOLLOW the voxel size. It used to be capped at 64,
  // which meant that past about resolution 64 a triangle spanning the part
  // could no longer reach every voxel it crosses, and the shell it painted
  // came out full of pinholes. The outside flood then poured through them
  // and the interior went to exactly zero -- on link_2 and link_5 at
  // resolution 80, measured. Nothing about the fit looked wrong; the
  // OCCUPANCY did, so every sphere measured as ~90% outside the part and
  // the waste number the operator tunes against was noise.
  //
  // `STEP_GUARD` is only a runaway guard for a degenerate mesh, not a
  // budget: sealing link_1 at resolution 96 costs 9 ms of rasterization.
  const STEP_GUARD = 2048;
  for (let t = 0; t < positions.length; t += 9) {
    const ax = positions[t],     ay = positions[t + 1], az = positions[t + 2];
    const bx = positions[t + 3], by = positions[t + 4], bz = positions[t + 5];
    const cx = positions[t + 6], cy = positions[t + 7], cz = positions[t + 8];
    const longest = Math.max(
      Math.hypot(bx - ax, by - ay, bz - az),
      Math.hypot(cx - bx, cy - by, cz - bz),
      Math.hypot(ax - cx, ay - cy, az - cz),
    );
    const steps = Math.min(STEP_GUARD, Math.max(2, Math.ceil((longest / voxel) * 2)));
    for (let u = 0; u <= steps; u += 1) {
      for (let v = 0; v + u <= steps; v += 1) {
        const fu = u / steps;
        const fv = v / steps;
        mark(
          ax + fu * (bx - ax) + fv * (cx - ax),
          ay + fu * (by - ay) + fv * (cy - ay),
          az + fu * (bz - az) + fv * (cz - az),
        );
      }
    }
  }

  // ---- interior, by flooding the outside -------------------------------
  const at = (i: number, j: number, k: number) => (i * ny + j) * nz + k;
  const outside = new Uint8Array(nx * ny * nz);
  const stack = [0];
  outside[0] = 1;
  while (stack.length) {
    const index = stack.pop()!;
    const k = index % nz;
    const j = Math.floor(index / nz) % ny;
    const i = Math.floor(index / (ny * nz));
    for (const [di, dj, dk] of [[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]] as const) {
      const x = i + di, y = j + dj, z = k + dk;
      if (x < 0 || y < 0 || z < 0 || x >= nx || y >= ny || z >= nz) continue;
      const nn = at(x, y, z);
      if (outside[nn] || surface[nn]) continue;
      outside[nn] = 1;
      stack.push(nn);
    }
  }
  const inside = new Uint8Array(nx * ny * nz);
  for (let i = 0; i < inside.length; i += 1) inside[i] = outside[i] || surface[i] ? 0 : 1;

  // Depth by two chamfer sweeps.
  const depthGrid = new Float32Array(nx * ny * nz);
  for (let i = 0; i < depthGrid.length; i += 1) depthGrid[i] = inside[i] ? 1e9 : 0;
  const sweep = (forward: boolean) => {
    const range = (count: number) =>
      forward ? [...Array(count).keys()] : [...Array(count).keys()].reverse();
    for (const i of range(nx)) for (const j of range(ny)) for (const k of range(nz)) {
      const nn = at(i, j, k);
      if (!inside[nn]) continue;
      let best = depthGrid[nn];
      for (let di = -1; di <= 1; di += 1)
        for (let dj = -1; dj <= 1; dj += 1)
          for (let dk = -1; dk <= 1; dk += 1) {
            if (!di && !dj && !dk) continue;
            if (forward && (di > 0 || (di === 0 && (dj > 0 || (dj === 0 && dk > 0))))) continue;
            if (!forward && (di < 0 || (di === 0 && (dj < 0 || (dj === 0 && dk < 0))))) continue;
            const x = i + di, y = j + dj, z = k + dk;
            if (x < 0 || y < 0 || z < 0 || x >= nx || y >= ny || z >= nz) continue;
            best = Math.min(best, depthGrid[at(x, y, z)] + Math.hypot(di, dj, dk) * voxel);
          }
      depthGrid[nn] = best;
    }
  };
  sweep(true);
  sweep(false);

  // ---- collect both sets ------------------------------------------------
  let count = 0;
  let interiorCount = 0;
  for (let n = 0; n < surface.length; n += 1) {
    if (surface[n]) count += 1;
    if (inside[n]) interiorCount += 1;
  }
  const centres = new Float32Array(count * 3);
  const interiorCentres = new Float32Array(interiorCount * 3);
  const interiorDepth = new Float32Array(interiorCount);
  let idx = 0;
  let iidx = 0;
  for (let i = 0; i < nx; i += 1) {
    for (let j = 0; j < ny; j += 1) {
      for (let k = 0; k < nz; k += 1) {
        const nn = at(i, j, k);
        const px = lo[0] + (i + 0.5) * voxel;
        const py = lo[1] + (j + 0.5) * voxel;
        const pz = lo[2] + (k + 0.5) * voxel;
        if (surface[nn]) { centres[idx++] = px; centres[idx++] = py; centres[idx++] = pz; }
        if (inside[nn]) {
          interiorCentres[iidx * 3] = px;
          interiorCentres[iidx * 3 + 1] = py;
          interiorCentres[iidx * 3 + 2] = pz;
          interiorDepth[iidx] = depthGrid[nn];
          iidx += 1;
        }
      }
    }
  }

  return {
    nx, ny, nz, voxel, origin: lo, surface,
    surfaceCentres: centres, surfaceCount: count,
    interiorCentres, interiorDepth, interiorCount,
  };
}

// --------------------------------------------------------------------------

/**
 * Stride for loops that SEARCH over interior voxels.
 *
 * Interior count goes as the cube of the grid resolution -- link_5 has 15k
 * interior voxels at resolution 48 and 130k at 80 -- and the clustering
 * passes are linear in it, so raising the grid one notch made the fit take
 * ten seconds. The search does not need every voxel: it needs a
 * representative sample of where the body is thick, and a stride gives that
 * at a bounded cost.
 *
 * Only the SEARCH is strided. `buildOccupancy` still consumes every voxel,
 * because the waste measurement is the one thing that must not get coarser
 * when the grid gets finer.
 */
function interiorStride(grid: Grid, cap = 20000): number {
  return Math.max(1, Math.ceil(grid.interiorCount / cap));
}

/**
 * Cluster the INTERIOR, weighted by depth, rather than the surface.
 *
 * A centre chosen on the surface wastes its whole radius; a centre chosen
 * deep inside wastes only radius minus depth. Clustering interior voxels
 * with depth as the weight puts centroids in the thick parts of the body by
 * construction, instead of moving them there afterwards and hoping the
 * radius does not have to grow to compensate.
 *
 * Benchmarked against six alternatives in `scripts/bench-fit.mts`: this
 * wins on the compact links (link_5 22.9%, link_6 34.9% wasted volume)
 * while the surface-clustered variant wins on the long ones (link_2
 * 25.4%). Neither dominates, which is why `fitSpheres` runs both.
 */
function fitInteriorKMeans(grid: Grid, k: number): Sphere[] {
  const cx = grid.surfaceCentres;
  const n = grid.surfaceCount;
  if (!grid.interiorCount) return [];

  const stride = interiorStride(grid);
  const centres: [number, number, number][] = [];
  let deepest = 0;
  for (let i = 1; i < grid.interiorCount; i += stride)
    if (grid.interiorDepth[i] > grid.interiorDepth[deepest]) deepest = i;
  centres.push([
    grid.interiorCentres[deepest * 3],
    grid.interiorCentres[deepest * 3 + 1],
    grid.interiorCentres[deepest * 3 + 2],
  ]);

  // Farthest-point seeding, biased toward depth: a candidate far from the
  // existing centres AND deep scores highest.
  while (centres.length < k) {
    let bestIdx = -1;
    let bestScore = -1;
    for (let i = 0; i < grid.interiorCount; i += stride) {
      let nearest = Infinity;
      for (const c of centres) {
        const dx = grid.interiorCentres[i * 3] - c[0];
        const dy = grid.interiorCentres[i * 3 + 1] - c[1];
        const dz = grid.interiorCentres[i * 3 + 2] - c[2];
        nearest = Math.min(nearest, dx * dx + dy * dy + dz * dz);
      }
      const score = nearest * (grid.interiorDepth[i] + grid.voxel);
      if (score > bestScore) { bestScore = score; bestIdx = i; }
    }
    if (bestIdx < 0) break;
    centres.push([
      grid.interiorCentres[bestIdx * 3],
      grid.interiorCentres[bestIdx * 3 + 1],
      grid.interiorCentres[bestIdx * 3 + 2],
    ]);
  }

  // Lloyd iterations with depth-weighted centroids.
  const assignment = new Int32Array(grid.interiorCount);
  for (let iter = 0; iter < 20; iter += 1) {
    for (let i = 0; i < grid.interiorCount; i += stride) {
      let bestDist = Infinity;
      let bestJ = 0;
      for (let j = 0; j < centres.length; j += 1) {
        const dx = grid.interiorCentres[i * 3] - centres[j][0];
        const dy = grid.interiorCentres[i * 3 + 1] - centres[j][1];
        const dz = grid.interiorCentres[i * 3 + 2] - centres[j][2];
        const d = dx * dx + dy * dy + dz * dz;
        if (d < bestDist) { bestDist = d; bestJ = j; }
      }
      assignment[i] = bestJ;
    }
    for (let j = 0; j < centres.length; j += 1) {
      let sx = 0, sy = 0, sz = 0, weight = 0;
      for (let i = 0; i < grid.interiorCount; i += stride) {
        if (assignment[i] !== j) continue;
        const w = grid.interiorDepth[i] + grid.voxel;
        sx += grid.interiorCentres[i * 3] * w;
        sy += grid.interiorCentres[i * 3 + 1] * w;
        sz += grid.interiorCentres[i * 3 + 2] * w;
        weight += w;
      }
      if (weight > 0) centres[j] = [sx / weight, sy / weight, sz / weight];
    }
  }

  // Radius: reach the surface voxels nearest to each centre.
  const spheres: Sphere[] = centres.map((c) => ({ center: c, radius: grid.voxel }));
  const owner = new Int32Array(n);
  for (let vi = 0; vi < n; vi += 1) {
    let bestDist = Infinity;
    for (let j = 0; j < spheres.length; j += 1) {
      const dx = cx[vi * 3] - spheres[j].center[0];
      const dy = cx[vi * 3 + 1] - spheres[j].center[1];
      const dz = cx[vi * 3 + 2] - spheres[j].center[2];
      const d = dx * dx + dy * dy + dz * dz;
      if (d < bestDist) { bestDist = d; owner[vi] = j; }
    }
  }
  for (let j = 0; j < spheres.length; j += 1) {
    let maxD = 0;
    for (let vi = 0; vi < n; vi += 1) {
      if (owner[vi] !== j) continue;
      const dx = cx[vi * 3] - spheres[j].center[0];
      const dy = cx[vi * 3 + 1] - spheres[j].center[1];
      const dz = cx[vi * 3 + 2] - spheres[j].center[2];
      maxD = Math.max(maxD, Math.sqrt(dx * dx + dy * dy + dz * dz));
    }
    spheres[j].radius = maxD + grid.voxel * COVER_SLACK;
  }
  return spheres;
}

/** Grow the nearest sphere until every surface voxel is covered. */
function repairCoverage(grid: Grid, spheres: Sphere[]): void {
  const cx = grid.surfaceCentres;
  for (let vi = 0; vi < grid.surfaceCount; vi += 1) {
    let covered = false;
    let bestJ = 0;
    let bestGap = Infinity;
    for (let j = 0; j < spheres.length; j += 1) {
      const dx = cx[vi * 3] - spheres[j].center[0];
      const dy = cx[vi * 3 + 1] - spheres[j].center[1];
      const dz = cx[vi * 3 + 2] - spheres[j].center[2];
      const d = Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (d <= spheres[j].radius) { covered = true; break; }
      if (d - spheres[j].radius < bestGap) { bestGap = d - spheres[j].radius; bestJ = j; }
    }
    if (covered || !spheres.length) continue;
    const dx = cx[vi * 3] - spheres[bestJ].center[0];
    const dy = cx[vi * 3 + 1] - spheres[bestJ].center[1];
    const dz = cx[vi * 3 + 2] - spheres[bestJ].center[2];
    spheres[bestJ].radius = Math.sqrt(dx * dx + dy * dy + dz * dz) + grid.voxel * COVER_SLACK;
  }
}

/**
 * k-means on the surface, then push each centre to the deepest interior
 * point it contains, then retighten.
 *
 * Chosen by benchmark against five alternatives (`scripts/bench-fit.mts`).
 * On the five arm links of rosie_1400_v3 it halves the protrusion of the
 * previous greedy-on-surface method -- 23-39 mm against 51-82 mm -- at the
 * same sphere count and 100% coverage, and runs in a few milliseconds.
 *
 * Why it works: a sphere centred ON the surface protrudes by its full
 * radius, because the body ends right there. Moving the centre to the
 * deepest interior voxel inside it buys back that depth: protrusion
 * becomes radius minus depth. k-means gives well-spread clusters to start
 * from, which greedy placement does not -- greedy's first sphere always
 * takes the densest region and the rest inherit whatever is left.
 *
 * The remaining protrusion is irreducible: a ball covering a patch of a
 * plate thinner than the ball must stick out. What the operator tunes is
 * the sphere count, which trades protrusion for query cost.
 */
function fitSurfaceKMeans(grid: Grid, requested: number): Sphere[] {
  if (!grid.surfaceCount) return [];

  const n = grid.surfaceCount;
  const cx = grid.surfaceCentres;
  const k = Math.max(1, Math.min(requested, n));

  // ---- k-means with farthest-point seeding ------------------------------
  // Farthest-point rather than random: it spreads the initial centres over
  // the whole body, so a long link gets clusters along its length instead
  // of several in whichever end happened to be sampled.
  const centres: [number, number, number][] = [[cx[0], cx[1], cx[2]]];
  while (centres.length < k) {
    let bestIdx = -1;
    let bestDist = -1;
    for (let vi = 0; vi < n; vi += 1) {
      let nearest = Infinity;
      for (const c of centres) {
        const dx = cx[vi * 3] - c[0], dy = cx[vi * 3 + 1] - c[1], dz = cx[vi * 3 + 2] - c[2];
        nearest = Math.min(nearest, dx * dx + dy * dy + dz * dz);
      }
      if (nearest > bestDist) { bestDist = nearest; bestIdx = vi; }
    }
    if (bestIdx < 0) break;
    centres.push([cx[bestIdx * 3], cx[bestIdx * 3 + 1], cx[bestIdx * 3 + 2]]);
  }

  const assignment = new Int32Array(n);
  for (let iter = 0; iter < 25; iter += 1) {
    for (let vi = 0; vi < n; vi += 1) {
      let bestDist = Infinity;
      let bestJ = 0;
      for (let j = 0; j < centres.length; j += 1) {
        const dx = cx[vi * 3] - centres[j][0];
        const dy = cx[vi * 3 + 1] - centres[j][1];
        const dz = cx[vi * 3 + 2] - centres[j][2];
        const d = dx * dx + dy * dy + dz * dz;
        if (d < bestDist) { bestDist = d; bestJ = j; }
      }
      assignment[vi] = bestJ;
    }
    for (let j = 0; j < centres.length; j += 1) {
      let sx = 0, sy = 0, sz = 0, count = 0;
      for (let vi = 0; vi < n; vi += 1) {
        if (assignment[vi] !== j) continue;
        sx += cx[vi * 3]; sy += cx[vi * 3 + 1]; sz += cx[vi * 3 + 2];
        count += 1;
      }
      if (count) centres[j] = [sx / count, sy / count, sz / count];
    }
  }

  // ---- build spheres, then sink each centre into the body ---------------
  const spheres: Sphere[] = [];
  for (let j = 0; j < centres.length; j += 1) {
    let maxD = 0;
    let any = false;
    for (let vi = 0; vi < n; vi += 1) {
      if (assignment[vi] !== j) continue;
      any = true;
      const dx = cx[vi * 3] - centres[j][0];
      const dy = cx[vi * 3 + 1] - centres[j][1];
      const dz = cx[vi * 3 + 2] - centres[j][2];
      maxD = Math.max(maxD, Math.sqrt(dx * dx + dy * dy + dz * dz));
    }
    if (any) spheres.push({ center: centres[j], radius: maxD + grid.voxel * COVER_SLACK });
  }

  // The deep move: for each sphere, find the deepest interior voxel it
  // contains and move there. This is what turns protrusion = radius into
  // protrusion = radius - depth.
  const stride = interiorStride(grid);
  if (grid.interiorCount) {
    for (const sphere of spheres) {
      let bestDepth = 0;
      let bestIdx = -1;
      for (let i = 0; i < grid.interiorCount; i += stride) {
        const dx = grid.interiorCentres[i * 3] - sphere.center[0];
        const dy = grid.interiorCentres[i * 3 + 1] - sphere.center[1];
        const dz = grid.interiorCentres[i * 3 + 2] - sphere.center[2];
        if (dx * dx + dy * dy + dz * dz > sphere.radius * sphere.radius) continue;
        if (grid.interiorDepth[i] > bestDepth) {
          bestDepth = grid.interiorDepth[i];
          bestIdx = i;
        }
      }
      if (bestIdx >= 0 && bestDepth > grid.voxel) {
        sphere.center = [
          grid.interiorCentres[bestIdx * 3],
          grid.interiorCentres[bestIdx * 3 + 1],
          grid.interiorCentres[bestIdx * 3 + 2],
        ];
      }
    }

    // Reassign to the moved centres and retighten, or the move would have
    // left gaps where a cluster's voxels are now nearer another sphere.
    for (let vi = 0; vi < n; vi += 1) {
      let bestDist = Infinity;
      for (let j = 0; j < spheres.length; j += 1) {
        const dx = cx[vi * 3] - spheres[j].center[0];
        const dy = cx[vi * 3 + 1] - spheres[j].center[1];
        const dz = cx[vi * 3 + 2] - spheres[j].center[2];
        const d = dx * dx + dy * dy + dz * dz;
        if (d < bestDist) { bestDist = d; assignment[vi] = j; }
      }
    }
    for (let j = 0; j < spheres.length; j += 1) {
      let maxD = 0;
      for (let vi = 0; vi < n; vi += 1) {
        if (assignment[vi] !== j) continue;
        const dx = cx[vi * 3] - spheres[j].center[0];
        const dy = cx[vi * 3 + 1] - spheres[j].center[1];
        const dz = cx[vi * 3 + 2] - spheres[j].center[2];
        maxD = Math.max(maxD, Math.sqrt(dx * dx + dy * dy + dz * dz));
      }
      spheres[j].radius = maxD + grid.voxel * COVER_SLACK;
    }
  }

  descendWaste(grid, spheres);
  return spheres;
}

/**
 * The objective, descended: minimise outside volume, coverage held at 100%.
 *
 * Coverage is a HARD constraint, not a term to be weighed. A cover that
 * wastes less volume by leaving surface uncovered is not a better cover; it
 * is a blind spot in collision search that the CCD verifier only finds at the
 * very end, after the search has already committed to a basin.
 *
 * So the loop below never accepts a move that uncovers anything. Within
 * that constraint it descends on wasted volume by two moves per sphere:
 * sink the centre deeper into the body (which lets the radius shrink for
 * the same reach), and shrink the radius to the minimum that still covers
 * the sphere's own assigned voxels.
 *
 * Takes the spheres as they are and improves them IN PLACE, which is what
 * makes it usable both as the tail of a fresh fit and on its own, against
 * whatever is currently on the link -- a hand-edited cover, or the result
 * of a previous fit the operator does not want to throw away.
 */
function descendWaste(grid: Grid, spheres: Sphere[], rounds = 6): void {
  const n = grid.surfaceCount;
  const cx = grid.surfaceCentres;
  const stride = interiorStride(grid);
  if (!n || !spheres.length) return;

  const occupancy = buildOccupancy(grid);
  const at = (i: number, j: number, k: number) => (i * grid.ny + j) * grid.nz + k;
  const insideBody = (x: number, y: number, z: number): boolean => {
    const i = Math.floor((x - grid.origin[0]) / grid.voxel);
    const j = Math.floor((y - grid.origin[1]) / grid.voxel);
    const k = Math.floor((z - grid.origin[2]) / grid.voxel);
    if (i < 0 || j < 0 || k < 0 || i >= grid.nx || j >= grid.ny || k >= grid.nz) return false;
    return occupancy[at(i, j, k)] === 1;
  };

  /** Wasted volume of one sphere, by the same lattice the report uses. */
  const wasted = (sphere: Sphere): number => {
    const STEPS = 7;
    const r = sphere.radius;
    const cell = (2 * r) / STEPS;
    let out = 0;
    for (let a = 0; a < STEPS; a += 1)
      for (let b = 0; b < STEPS; b += 1)
        for (let c = 0; c < STEPS; c += 1) {
          const px = sphere.center[0] - r + (a + 0.5) * cell;
          const py = sphere.center[1] - r + (b + 0.5) * cell;
          const pz = sphere.center[2] - r + (c + 0.5) * cell;
          const dx = px - sphere.center[0], dy = py - sphere.center[1], dz = pz - sphere.center[2];
          if (dx * dx + dy * dy + dz * dz > r * r) continue;
          if (!insideBody(px, py, pz)) out += 1;
        }
    return out * cell * cell * cell;
  };

  /** The smallest radius at `centre` that still covers `owned`. */
  const radiusFor = (centre: [number, number, number], owned: number[]): number => {
    let maxD = 0;
    for (const vi of owned) {
      const dx = cx[vi * 3] - centre[0];
      const dy = cx[vi * 3 + 1] - centre[1];
      const dz = cx[vi * 3 + 2] - centre[2];
      maxD = Math.max(maxD, Math.sqrt(dx * dx + dy * dy + dz * dz));
    }
    return maxD + grid.voxel * COVER_SLACK;
  };

  for (let round = 0; round < rounds; round += 1) {
    // Ownership: each surface voxel belongs to the sphere covering it whose
    // surface is nearest. A voxel covered by only one sphere pins that
    // sphere -- shrinking it would uncover the voxel, so it cannot shrink.
    const owners: number[][] = spheres.map(() => []);
    let anyUncovered = false;
    for (let vi = 0; vi < n; vi += 1) {
      let bestJ = -1;
      let bestGap = Infinity;
      for (let j = 0; j < spheres.length; j += 1) {
        const dx = cx[vi * 3] - spheres[j].center[0];
        const dy = cx[vi * 3 + 1] - spheres[j].center[1];
        const dz = cx[vi * 3 + 2] - spheres[j].center[2];
        const gap = Math.sqrt(dx * dx + dy * dy + dz * dz) - spheres[j].radius;
        if (gap < bestGap) { bestGap = gap; bestJ = j; }
      }
      if (bestGap > 0) anyUncovered = true;
      if (bestJ >= 0) owners[bestJ].push(vi);
    }

    let improved = false;
    for (let j = 0; j < spheres.length; j += 1) {
      if (!owners[j].length) continue;
      const before = spheres[j];
      const beforeWaste = wasted(before);

      // Candidate centres: the current one, and the interior voxels near it
      // ordered by depth. Deeper is better only if the radius needed from
      // there does not grow more than the depth gained.
      let best = before;
      let bestWaste = beforeWaste;
      const reach = before.radius * 1.2;
      for (let i = 0; i < grid.interiorCount; i += stride) {
        const dx = grid.interiorCentres[i * 3] - before.center[0];
        const dy = grid.interiorCentres[i * 3 + 1] - before.center[1];
        const dz = grid.interiorCentres[i * 3 + 2] - before.center[2];
        if (dx * dx + dy * dy + dz * dz > reach * reach) continue;
        const centre: [number, number, number] = [
          grid.interiorCentres[i * 3],
          grid.interiorCentres[i * 3 + 1],
          grid.interiorCentres[i * 3 + 2],
        ];
        const candidate = { center: centre, radius: radiusFor(centre, owners[j]) };
        // Never accept a candidate that would uncover this sphere's own
        // voxels -- `radiusFor` guarantees it covers them, so the only
        // check left is that it does not grow absurdly.
        if (candidate.radius > before.radius * 1.5) continue;
        const waste = wasted(candidate);
        if (waste < bestWaste * 0.98) { bestWaste = waste; best = candidate; }
      }

      if (best !== before) { spheres[j] = best; improved = true; }
      else {
        // No better centre: at least shrink to the minimum that still
        // covers what this sphere owns.
        const tight = radiusFor(before.center, owners[j]);
        if (tight < before.radius - grid.voxel * 0.01) {
          spheres[j] = { center: before.center, radius: tight };
          improved = true;
        }
      }
    }
    if (!improved && !anyUncovered) break;

    // Coverage repair after every round. This is the hard constraint: any
    // voxel left outside every sphere grows its nearest one until covered.
    const covered = new Uint8Array(n);
    for (let vi = 0; vi < n; vi += 1)
      for (const sphere of spheres) {
        const dx = cx[vi * 3] - sphere.center[0];
        const dy = cx[vi * 3 + 1] - sphere.center[1];
        const dz = cx[vi * 3 + 2] - sphere.center[2];
        if (dx * dx + dy * dy + dz * dz <= sphere.radius * sphere.radius) { covered[vi] = 1; break; }
      }
    for (let vi = 0; vi < n; vi += 1) {
      if (covered[vi]) continue;
      let bestJ = 0;
      let bestGap = Infinity;
      for (let j = 0; j < spheres.length; j += 1) {
        const dx = cx[vi * 3] - spheres[j].center[0];
        const dy = cx[vi * 3 + 1] - spheres[j].center[1];
        const dz = cx[vi * 3 + 2] - spheres[j].center[2];
        const gap = Math.sqrt(dx * dx + dy * dy + dz * dz) - spheres[j].radius;
        if (gap < bestGap) { bestGap = gap; bestJ = j; }
      }
      const dx = cx[vi * 3] - spheres[bestJ].center[0];
      const dy = cx[vi * 3 + 1] - spheres[bestJ].center[1];
      const dz = cx[vi * 3 + 2] - spheres[bestJ].center[2];
      spheres[bestJ].radius = Math.max(
        spheres[bestJ].radius,
        Math.sqrt(dx * dx + dy * dy + dz * dz) + grid.voxel * COVER_SLACK,
      );
    }
  }
}

/**
 * Box decomposition: carve the solid into near-cubic boxes, one sphere each.
 *
 * A sphere covering a FILLED box wastes only the corners -- the ratio of a
 * ball to its inscribed cube is fixed and modest. A sphere covering a
 * scattered cluster of surface voxels wastes everything between them. So
 * decomposing the solid into boxes and taking one sphere per box attacks
 * the objective structurally rather than by local search.
 *
 * Three parts:
 *
 * 1. **Greedy maximal boxes.** Sweep the solid, and from each unused voxel
 *    grow a box along x, then y, then z, as far as the region stays solid.
 *    (An octree's power-of-two blocks would produce a staircase on any
 *    surface not aligned to the grid; unconstrained growth does not.)
 *
 * 2. **Aspect cap.** Growth is trimmed so boxes stay near-cubic, because
 *    the sphere around a box has radius = half its DIAGONAL. A 1x1x20 run
 *    becomes a ball of radius 10 covering almost nothing -- the first
 *    version of this without a cap merged link_2 into five boxes and
 *    wasted 93% of its volume. `aspect` is the knob: 1.0 is cubic and
 *    wastes least, higher merges more aggressively and uses fewer spheres.
 *
 * 3. **Offset search.** The decomposition is sensitive to where the grid
 *    happens to sit: a slab half a voxel out of phase splits into twice as
 *    many boxes. The whole thing is repeated over four sub-voxel offsets
 *    and the phase yielding the fewest boxes is kept.
 *
 * Then each sphere is shrunk to the minimum covering the surface voxels it
 * owns, and redundant spheres -- ones whose voxels are all covered by a
 * neighbour -- are dropped.
 */
function fitBoxes(positions: Float32Array, grid: Grid, resolution: number, aspect: number): Sphere[] {
  let best: Box[] | null = null;
  for (const offset of [0, 0.25, 0.5, 0.75]) {
    const boxes = decomposeSolid(positions, resolution, offset, aspect);
    if (!best || boxes.length < best.length) best = boxes;
  }
  const spheres: Sphere[] = (best ?? []).map((b) => ({
    center: [(b.lo[0] + b.hi[0]) / 2, (b.lo[1] + b.hi[1]) / 2, (b.lo[2] + b.hi[2]) / 2],
    radius: Math.hypot(b.hi[0] - b.lo[0], b.hi[1] - b.lo[1], b.hi[2] - b.lo[2]) / 2,
  }));
  if (!spheres.length) return [];

  const cx = grid.surfaceCentres;
  const n = grid.surfaceCount;
  for (let round = 0; round < 3; round += 1) {
    const owners: number[][] = spheres.map(() => []);
    for (let vi = 0; vi < n; vi += 1) {
      let bestJ = 0;
      let bestGap = Infinity;
      for (let j = 0; j < spheres.length; j += 1) {
        const dx = cx[vi * 3] - spheres[j].center[0];
        const dy = cx[vi * 3 + 1] - spheres[j].center[1];
        const dz = cx[vi * 3 + 2] - spheres[j].center[2];
        const gap = Math.sqrt(dx * dx + dy * dy + dz * dz) - spheres[j].radius;
        if (gap < bestGap) { bestGap = gap; bestJ = j; }
      }
      owners[bestJ].push(vi);
    }
    for (let j = 0; j < spheres.length; j += 1) {
      if (!owners[j].length) continue;
      let maxD = 0;
      for (const vi of owners[j]) {
        const dx = cx[vi * 3] - spheres[j].center[0];
        const dy = cx[vi * 3 + 1] - spheres[j].center[1];
        const dz = cx[vi * 3 + 2] - spheres[j].center[2];
        maxD = Math.max(maxD, Math.sqrt(dx * dx + dy * dy + dz * dz));
      }
      const tight = maxD + grid.voxel * COVER_SLACK;
      if (tight < spheres[j].radius) spheres[j].radius = tight;
    }
    repairCoverage(grid, spheres);
  }
  return spheres;
}

interface Box { lo: [number, number, number]; hi: [number, number, number] }

function decomposeSolid(
  positions: Float32Array, resolution: number, offsetFraction: number, maxAspect: number,
): Box[] {
  let lo: [number, number, number] = [Infinity, Infinity, Infinity];
  let hi: [number, number, number] = [-Infinity, -Infinity, -Infinity];
  for (let v = 0; v < positions.length; v += 3)
    for (let a = 0; a < 3; a += 1) {
      lo[a] = Math.min(lo[a], positions[v + a]);
      hi[a] = Math.max(hi[a], positions[v + a]);
    }
  const span = Math.max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2], 1e-9);
  const voxel = span / Math.max(8, resolution);
  lo = [
    lo[0] - voxel * (2 + offsetFraction),
    lo[1] - voxel * (2 + offsetFraction),
    lo[2] - voxel * (2 + offsetFraction),
  ];
  hi = [hi[0] + voxel * 2, hi[1] + voxel * 2, hi[2] + voxel * 2];
  const nx = Math.max(3, Math.ceil((hi[0] - lo[0]) / voxel));
  const ny = Math.max(3, Math.ceil((hi[1] - lo[1]) / voxel));
  const nz = Math.max(3, Math.ceil((hi[2] - lo[2]) / voxel));
  const at = (i: number, j: number, k: number) => (i * ny + j) * nz + k;

  const surface = new Uint8Array(nx * ny * nz);
  const mark = (x: number, y: number, z: number) => {
    const i = Math.floor((x - lo[0]) / voxel);
    const j = Math.floor((y - lo[1]) / voxel);
    const k = Math.floor((z - lo[2]) / voxel);
    if (i >= 0 && j >= 0 && k >= 0 && i < nx && j < ny && k < nz) surface[at(i, j, k)] = 1;
  };
  for (let t = 0; t < positions.length; t += 9) {
    const ax = positions[t], ay = positions[t+1], az = positions[t+2];
    const bx = positions[t+3], by = positions[t+4], bz = positions[t+5];
    const ccx = positions[t+6], ccy = positions[t+7], ccz = positions[t+8];
    const longest = Math.max(
      Math.hypot(bx-ax, by-ay, bz-az),
      Math.hypot(ccx-bx, ccy-by, ccz-bz),
      Math.hypot(ax-ccx, ay-ccy, az-ccz),
    );
    const steps = Math.min(2048, Math.max(2, Math.ceil((longest / voxel) * 2)));
    for (let u = 0; u <= steps; u += 1)
      for (let v = 0; v + u <= steps; v += 1) {
        const fu = u / steps, fv = v / steps;
        mark(ax + fu*(bx-ax) + fv*(ccx-ax), ay + fu*(by-ay) + fv*(ccy-ay), az + fu*(bz-az) + fv*(ccz-az));
      }
  }

  const outside = new Uint8Array(nx * ny * nz);
  const stack = [0];
  outside[0] = 1;
  while (stack.length) {
    const index = stack.pop()!;
    const k = index % nz, j = Math.floor(index / nz) % ny, i = Math.floor(index / (ny * nz));
    for (const [di, dj, dk] of [[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]] as const) {
      const x = i+di, y = j+dj, z = k+dk;
      if (x < 0 || y < 0 || z < 0 || x >= nx || y >= ny || z >= nz) continue;
      const nn = at(x, y, z);
      if (outside[nn] || surface[nn]) continue;
      outside[nn] = 1;
      stack.push(nn);
    }
  }
  const solid = new Uint8Array(nx * ny * nz);
  for (let i = 0; i < solid.length; i += 1) solid[i] = !outside[i] || surface[i] ? 1 : 0;

  const used = new Uint8Array(nx * ny * nz);
  const free = (i: number, j: number, k: number) =>
    i >= 0 && j >= 0 && k >= 0 && i < nx && j < ny && k < nz &&
    solid[at(i, j, k)] === 1 && used[at(i, j, k)] === 0;

  const boxes: Box[] = [];
  for (let i = 0; i < nx; i += 1)
    for (let j = 0; j < ny; j += 1)
      for (let k = 0; k < nz; k += 1) {
        if (!free(i, j, k)) continue;
        let ei = i;
        while (free(ei + 1, j, k)) ei += 1;
        let ej = j;
        growY: while (true) {
          for (let x = i; x <= ei; x += 1) if (!free(x, ej + 1, k)) break growY;
          ej += 1;
        }
        let ek = k;
        growZ: while (true) {
          for (let x = i; x <= ei; x += 1)
            for (let y = j; y <= ej; y += 1) if (!free(x, y, ek + 1)) break growZ;
          ek += 1;
        }
        // Trim toward cubic: the sphere's radius is half the box DIAGONAL,
        // so an elongated box is a wasteful sphere.
        const shortest = Math.min(ei - i + 1, ej - j + 1, ek - k + 1);
        const cap = Math.max(1, Math.ceil(shortest * maxAspect));
        ei = Math.min(ei, i + cap - 1);
        ej = Math.min(ej, j + cap - 1);
        ek = Math.min(ek, k + cap - 1);
        for (let x = i; x <= ei; x += 1)
          for (let y = j; y <= ej; y += 1)
            for (let z = k; z <= ek; z += 1) used[at(x, y, z)] = 1;
        boxes.push({
          lo: [lo[0] + i*voxel, lo[1] + j*voxel, lo[2] + k*voxel],
          hi: [lo[0] + (ei+1)*voxel, lo[1] + (ej+1)*voxel, lo[2] + (ek+1)*voxel],
        });
      }
  return boxes;
}

/**
 * Hierarchy: coarse-to-fine box decomposition.
 *
 * `fitBoxes` decomposes at ONE resolution, which forces a single trade.
 * Coarse voxels merge the bulk into few boxes but cannot represent a thin
 * flange; fine voxels represent everything and produce thousands of boxes --
 * 6045 on link_1.
 *
 * The hierarchy does both. Start at the coarsest level; whatever is fully
 * solid there becomes a big box and is struck off. Refine, decompose only
 * what is left, repeat. The bulk goes to a handful of large boxes and only
 * the skin and the thin features descend to the fine grid, where they are
 * cheap because there is little of them left.
 *
 * Each level searches its own sub-voxel phase, since the phase that suits a
 * coarse grid is not the one that suits a fine one.
 */
function fitHierarchy(
  positions: Float32Array, baseResolution: number, levels: number, aspect: number,
): Sphere[] {
  const fineRes = baseResolution * Math.pow(2, levels - 1);
  const fine = buildSolidGrid(positions, fineRes);

  // `claimed` marks fine voxels already inside some box.
  const claimed = new Uint8Array(fine.solid.length);
  const boxes: Box[] = [];

  for (let level = 0; level < levels; level += 1) {
    const ratio = Math.round(fineRes / (baseResolution * Math.pow(2, level)));
    let bestLevel: Box[] | null = null;
    for (const offset of [0, 0.5]) {
      const found = decomposeLevel(fine, claimed, ratio, offset, aspect);
      if (!bestLevel || found.length < bestLevel.length) bestLevel = found;
    }
    for (const box of bestLevel ?? []) {
      boxes.push(box);
      markClaimed(fine, claimed, box);
    }
  }

  // Anything still unclaimed becomes a single fine-voxel box.
  for (let i = 0; i < fine.nx; i += 1)
    for (let j = 0; j < fine.ny; j += 1)
      for (let k = 0; k < fine.nz; k += 1) {
        const n = (i * fine.ny + j) * fine.nz + k;
        if (!fine.solid[n] || claimed[n]) continue;
        boxes.push({
          lo: [fine.lo[0] + i * fine.voxel, fine.lo[1] + j * fine.voxel, fine.lo[2] + k * fine.voxel],
          hi: [fine.lo[0] + (i + 1) * fine.voxel, fine.lo[1] + (j + 1) * fine.voxel, fine.lo[2] + (k + 1) * fine.voxel],
        });
        claimed[n] = 1;
      }

  return boxes.map(boxSphere);
}

function boxSphere(b: Box): Sphere {
  return {
    center: [(b.lo[0] + b.hi[0]) / 2, (b.lo[1] + b.hi[1]) / 2, (b.lo[2] + b.hi[2]) / 2],
    radius: Math.hypot(b.hi[0] - b.lo[0], b.hi[1] - b.lo[1], b.hi[2] - b.lo[2]) / 2,
  };
}

interface SolidGrid {
  nx: number; ny: number; nz: number; voxel: number;
  lo: [number, number, number];
  solid: Uint8Array;
}

/** Surface-voxelize, flood the outside, call the rest solid. */
function buildSolidGrid(positions: Float32Array, resolution: number): SolidGrid {
  let lo: [number, number, number] = [Infinity, Infinity, Infinity];
  let hi: [number, number, number] = [-Infinity, -Infinity, -Infinity];
  for (let v = 0; v < positions.length; v += 3)
    for (let a = 0; a < 3; a += 1) {
      lo[a] = Math.min(lo[a], positions[v + a]);
      hi[a] = Math.max(hi[a], positions[v + a]);
    }
  const span = Math.max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2], 1e-9);
  const voxel = span / Math.max(8, resolution);
  lo = [lo[0] - voxel * 2, lo[1] - voxel * 2, lo[2] - voxel * 2];
  hi = [hi[0] + voxel * 2, hi[1] + voxel * 2, hi[2] + voxel * 2];
  const nx = Math.max(3, Math.ceil((hi[0] - lo[0]) / voxel));
  const ny = Math.max(3, Math.ceil((hi[1] - lo[1]) / voxel));
  const nz = Math.max(3, Math.ceil((hi[2] - lo[2]) / voxel));
  const at = (i: number, j: number, k: number) => (i * ny + j) * nz + k;

  const surface = new Uint8Array(nx * ny * nz);
  const mark = (x: number, y: number, z: number) => {
    const i = Math.floor((x - lo[0]) / voxel);
    const j = Math.floor((y - lo[1]) / voxel);
    const k = Math.floor((z - lo[2]) / voxel);
    if (i >= 0 && j >= 0 && k >= 0 && i < nx && j < ny && k < nz) surface[at(i, j, k)] = 1;
  };
  for (let t = 0; t < positions.length; t += 9) {
    const ax = positions[t], ay = positions[t + 1], az = positions[t + 2];
    const bx = positions[t + 3], by = positions[t + 4], bz = positions[t + 5];
    const ccx = positions[t + 6], ccy = positions[t + 7], ccz = positions[t + 8];
    const longest = Math.max(
      Math.hypot(bx - ax, by - ay, bz - az),
      Math.hypot(ccx - bx, ccy - by, ccz - bz),
      Math.hypot(ax - ccx, ay - ccy, az - ccz),
    );
    const steps = Math.min(2048, Math.max(2, Math.ceil((longest / voxel) * 2)));
    for (let u = 0; u <= steps; u += 1)
      for (let v = 0; v + u <= steps; v += 1) {
        const fu = u / steps, fv = v / steps;
        mark(
          ax + fu * (bx - ax) + fv * (ccx - ax),
          ay + fu * (by - ay) + fv * (ccy - ay),
          az + fu * (bz - az) + fv * (ccz - az),
        );
      }
  }

  const outside = new Uint8Array(nx * ny * nz);
  const stack = [0];
  outside[0] = 1;
  while (stack.length) {
    const index = stack.pop()!;
    const k = index % nz, j = Math.floor(index / nz) % ny, i = Math.floor(index / (ny * nz));
    for (const [di, dj, dk] of [[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]] as const) {
      const x = i + di, y = j + dj, z = k + dk;
      if (x < 0 || y < 0 || z < 0 || x >= nx || y >= ny || z >= nz) continue;
      const nn = at(x, y, z);
      if (outside[nn] || surface[nn]) continue;
      outside[nn] = 1;
      stack.push(nn);
    }
  }
  const solid = new Uint8Array(nx * ny * nz);
  for (let i = 0; i < solid.length; i += 1) solid[i] = !outside[i] || surface[i] ? 1 : 0;
  return { nx, ny, nz, voxel, lo, solid };
}

/**
 * One level: coarse cells of `ratio`^3 fine voxels that are ENTIRELY solid
 * and unclaimed, merged greedily into boxes under the aspect cap.
 */
function decomposeLevel(
  fine: SolidGrid, claimed: Uint8Array, ratio: number, offsetFraction: number, maxAspect: number,
): Box[] {
  const at = (i: number, j: number, k: number) => (i * fine.ny + j) * fine.nz + k;
  const shift = Math.floor(ratio * offsetFraction);
  const cnx = Math.floor((fine.nx - shift) / ratio);
  const cny = Math.floor((fine.ny - shift) / ratio);
  const cnz = Math.floor((fine.nz - shift) / ratio);
  if (cnx < 1 || cny < 1 || cnz < 1) return [];
  const cAt = (i: number, j: number, k: number) => (i * cny + j) * cnz + k;

  const avail = new Uint8Array(cnx * cny * cnz);
  for (let ci = 0; ci < cnx; ci += 1)
    for (let cj = 0; cj < cny; cj += 1)
      for (let ck = 0; ck < cnz; ck += 1) {
        let ok = true;
        outer: for (let a = 0; a < ratio; a += 1)
          for (let b = 0; b < ratio; b += 1)
            for (let c = 0; c < ratio; c += 1) {
              const i = shift + ci * ratio + a, j = shift + cj * ratio + b, k = shift + ck * ratio + c;
              if (i >= fine.nx || j >= fine.ny || k >= fine.nz) { ok = false; break outer; }
              const n = at(i, j, k);
              if (!fine.solid[n] || claimed[n]) { ok = false; break outer; }
            }
        if (ok) avail[cAt(ci, cj, ck)] = 1;
      }

  const used = new Uint8Array(cnx * cny * cnz);
  const free = (i: number, j: number, k: number) =>
    i >= 0 && j >= 0 && k >= 0 && i < cnx && j < cny && k < cnz &&
    avail[cAt(i, j, k)] === 1 && used[cAt(i, j, k)] === 0;

  const boxes: Box[] = [];
  for (let i = 0; i < cnx; i += 1)
    for (let j = 0; j < cny; j += 1)
      for (let k = 0; k < cnz; k += 1) {
        if (!free(i, j, k)) continue;
        let ei = i;
        while (free(ei + 1, j, k)) ei += 1;
        let ej = j;
        growY: while (true) {
          for (let x = i; x <= ei; x += 1) if (!free(x, ej + 1, k)) break growY;
          ej += 1;
        }
        let ek = k;
        growZ: while (true) {
          for (let x = i; x <= ei; x += 1)
            for (let y = j; y <= ej; y += 1) if (!free(x, y, ek + 1)) break growZ;
          ek += 1;
        }
        const shortest = Math.min(ei - i + 1, ej - j + 1, ek - k + 1);
        const cap = Math.max(1, Math.ceil(shortest * maxAspect));
        ei = Math.min(ei, i + cap - 1);
        ej = Math.min(ej, j + cap - 1);
        ek = Math.min(ek, k + cap - 1);
        for (let x = i; x <= ei; x += 1)
          for (let y = j; y <= ej; y += 1)
            for (let z = k; z <= ek; z += 1) used[cAt(x, y, z)] = 1;
        boxes.push({
          lo: [
            fine.lo[0] + (shift + i * ratio) * fine.voxel,
            fine.lo[1] + (shift + j * ratio) * fine.voxel,
            fine.lo[2] + (shift + k * ratio) * fine.voxel,
          ],
          hi: [
            fine.lo[0] + (shift + (ei + 1) * ratio) * fine.voxel,
            fine.lo[1] + (shift + (ej + 1) * ratio) * fine.voxel,
            fine.lo[2] + (shift + (ek + 1) * ratio) * fine.voxel,
          ],
        });
      }
  return boxes;
}

function markClaimed(fine: SolidGrid, claimed: Uint8Array, box: Box): void {
  const i0 = Math.max(0, Math.floor((box.lo[0] - fine.lo[0]) / fine.voxel));
  const j0 = Math.max(0, Math.floor((box.lo[1] - fine.lo[1]) / fine.voxel));
  const k0 = Math.max(0, Math.floor((box.lo[2] - fine.lo[2]) / fine.voxel));
  const i1 = Math.min(fine.nx - 1, Math.ceil((box.hi[0] - fine.lo[0]) / fine.voxel) - 1);
  const j1 = Math.min(fine.ny - 1, Math.ceil((box.hi[1] - fine.lo[1]) / fine.voxel) - 1);
  const k1 = Math.min(fine.nz - 1, Math.ceil((box.hi[2] - fine.lo[2]) / fine.voxel) - 1);
  for (let i = i0; i <= i1; i += 1)
    for (let j = j0; j <= j1; j += 1)
      for (let k = k0; k <= k1; k += 1) claimed[(i * fine.ny + j) * fine.nz + k] = 1;
}

// --------------------------------------------------------------------------

/**
 * Cut a sphere set down to `budget`, without uncovering anything.
 *
 * The box methods answer "waste least" with however many boxes the body
 * needs, which on link_1 is six thousand. That is a real answer to the
 * wrong question: a sphere model is also a thing a person has to look at,
 * edit, and reason about, and the query cost is not actually free once it
 * is multiplied by trajectory samples and optimizer iterations.
 *
 * The merge is k-means over the sphere CENTRES, weighted by volume so a big
 * sphere pulls a group toward itself, and each group collapses to the
 * sphere that ENCLOSES its members: centre at the weighted centroid, radius
 * = max over members of (distance to member centre + member radius). That
 * containment is what makes the operation safe -- whatever the members
 * covered, the enclosing sphere still covers. It is strictly worse on
 * wasted volume, which is the honest price of the cap, and the caller
 * re-tightens against the surface afterwards to give some of it back.
 */
function enforceBudget(spheres: Sphere[], budget: number): Sphere[] {
  if (budget < 1 || spheres.length <= budget) return spheres;

  const weight = spheres.map((s) => Math.max(s.radius * s.radius * s.radius, 1e-18));

  // Farthest-point seeding: a body that is mostly one dense blob would
  // otherwise put every seed in the blob and merge the outriggers into one
  // enormous sphere.
  const seeds: number[] = [0];
  while (seeds.length < budget) {
    let bestIdx = -1;
    let bestDist = -1;
    for (let i = 0; i < spheres.length; i += 1) {
      let nearest = Infinity;
      for (const s of seeds) {
        const dx = spheres[i].center[0] - spheres[s].center[0];
        const dy = spheres[i].center[1] - spheres[s].center[1];
        const dz = spheres[i].center[2] - spheres[s].center[2];
        nearest = Math.min(nearest, dx * dx + dy * dy + dz * dz);
      }
      if (nearest > bestDist) { bestDist = nearest; bestIdx = i; }
    }
    if (bestIdx < 0) break;
    seeds.push(bestIdx);
  }
  const centres = seeds.map((s) => [...spheres[s].center] as [number, number, number]);

  const assignment = new Int32Array(spheres.length);
  for (let iter = 0; iter < 15; iter += 1) {
    for (let i = 0; i < spheres.length; i += 1) {
      let bestDist = Infinity;
      let bestJ = 0;
      for (let j = 0; j < centres.length; j += 1) {
        const dx = spheres[i].center[0] - centres[j][0];
        const dy = spheres[i].center[1] - centres[j][1];
        const dz = spheres[i].center[2] - centres[j][2];
        const d = dx * dx + dy * dy + dz * dz;
        if (d < bestDist) { bestDist = d; bestJ = j; }
      }
      assignment[i] = bestJ;
    }
    for (let j = 0; j < centres.length; j += 1) {
      let sx = 0, sy = 0, sz = 0, w = 0;
      for (let i = 0; i < spheres.length; i += 1) {
        if (assignment[i] !== j) continue;
        sx += spheres[i].center[0] * weight[i];
        sy += spheres[i].center[1] * weight[i];
        sz += spheres[i].center[2] * weight[i];
        w += weight[i];
      }
      if (w > 0) centres[j] = [sx / w, sy / w, sz / w];
    }
  }

  const merged: Sphere[] = [];
  for (let j = 0; j < centres.length; j += 1) {
    let radius = 0;
    let any = false;
    for (let i = 0; i < spheres.length; i += 1) {
      if (assignment[i] !== j) continue;
      any = true;
      const dx = spheres[i].center[0] - centres[j][0];
      const dy = spheres[i].center[1] - centres[j][1];
      const dz = spheres[i].center[2] - centres[j][2];
      radius = Math.max(radius, Math.hypot(dx, dy, dz) + spheres[i].radius);
    }
    if (any) merged.push({ center: centres[j], radius });
  }
  return merged;
}

/**
 * Shrink each sphere to the smallest radius still covering the surface it
 * owns, then restore coverage. Run after any operation that inflated radii.
 */
function tighten(grid: Grid, spheres: Sphere[], rounds = 2): void {
  const cx = grid.surfaceCentres;
  const n = grid.surfaceCount;
  for (let round = 0; round < rounds; round += 1) {
    const owners: number[][] = spheres.map(() => []);
    for (let vi = 0; vi < n; vi += 1) {
      let bestJ = 0;
      let bestGap = Infinity;
      for (let j = 0; j < spheres.length; j += 1) {
        const dx = cx[vi * 3] - spheres[j].center[0];
        const dy = cx[vi * 3 + 1] - spheres[j].center[1];
        const dz = cx[vi * 3 + 2] - spheres[j].center[2];
        const gap = Math.sqrt(dx * dx + dy * dy + dz * dz) - spheres[j].radius;
        if (gap < bestGap) { bestGap = gap; bestJ = j; }
      }
      owners[bestJ].push(vi);
    }
    for (let j = 0; j < spheres.length; j += 1) {
      if (!owners[j].length) continue;
      let maxD = 0;
      for (const vi of owners[j]) {
        const dx = cx[vi * 3] - spheres[j].center[0];
        const dy = cx[vi * 3 + 1] - spheres[j].center[1];
        const dz = cx[vi * 3 + 2] - spheres[j].center[2];
        maxD = Math.max(maxD, Math.sqrt(dx * dx + dy * dy + dz * dz));
      }
      const tight = maxD + grid.voxel * COVER_SLACK;
      if (tight < spheres[j].radius) spheres[j].radius = tight;
    }
    repairCoverage(grid, spheres);
  }
}

/**
 * Drop spheres that cover nothing another sphere does not already cover.
 *
 * Done by a coverage COUNT rather than by re-measuring the whole set once
 * per sphere: the latter is quadratic in the sphere count, which on a
 * six-thousand-sphere decomposition is the entire cost of the fit. Counts
 * are decremented as spheres are dropped, so two mutually-redundant spheres
 * cannot both be removed and leave a hole.
 */
function pruneRedundant(grid: Grid, spheres: Sphere[]): Sphere[] {
  const cx = grid.surfaceCentres;
  const n = grid.surfaceCount;
  if (!n || spheres.length < 2) return spheres;

  // Two relations per (voxel, sphere): the sphere COVERS the voxel centre,
  // and the sphere covers it STRONGLY -- with the same slack a radius gets,
  // so the claim survives being re-checked on a finer grid.
  //
  // Redundancy has to be judged on the strong relation. Two spheres whose
  // surfaces both merely touch a voxel's centre do not between them cover
  // the voxel, so dropping one on that evidence opens a hole that only a
  // finer grid can see -- which is exactly what `hierarchy` was doing:
  // 0.00% uncovered at grid 48, 3.3% at grid 80, after the radius slack had
  // already been fixed.
  const slack = grid.voxel * COVER_SLACK;
  const strongCount = new Int32Array(n);
  const owned: number[][] = spheres.map(() => []);
  const strong: boolean[][] = spheres.map(() => []);
  for (let vi = 0; vi < n; vi += 1)
    for (let j = 0; j < spheres.length; j += 1) {
      const dx = cx[vi * 3] - spheres[j].center[0];
      const dy = cx[vi * 3 + 1] - spheres[j].center[1];
      const dz = cx[vi * 3 + 2] - spheres[j].center[2];
      const d2 = dx * dx + dy * dy + dz * dz;
      if (d2 > spheres[j].radius * spheres[j].radius) continue;
      owned[j].push(vi);
      const inner = spheres[j].radius - slack;
      const isStrong = inner > 0 && d2 <= inner * inner;
      strong[j].push(isStrong);
      if (isStrong) strongCount[vi] += 1;
    }

  // Biggest first: dropping a large redundant sphere buys the most volume.
  const order = spheres.map((_, j) => j).sort((a, b) => spheres[b].radius - spheres[a].radius);
  const dropped = new Uint8Array(spheres.length);
  for (const j of order) {
    let droppable = true;
    for (let t = 0; t < owned[j].length; t += 1) {
      const others = strongCount[owned[j][t]] - (strong[j][t] ? 1 : 0);
      if (others < 1) { droppable = false; break; }
    }
    if (!droppable) continue;
    dropped[j] = 1;
    for (let t = 0; t < owned[j].length; t += 1)
      if (strong[j][t]) strongCount[owned[j][t]] -= 1;
  }
  const kept = spheres.filter((_, j) => !dropped[j]);
  return kept.length ? kept : spheres;
}

/**
 * The fit: build the requested strategy (or all three), cap each at the
 * budget, keep whichever wastes least at 100% coverage.
 *
 * Selection is on the SAME lattice measurement the report shows -- an
 * earlier version scored candidates by analytic sphere volume while the
 * report sampled, and the ensemble picked the loser on one link because the
 * two disagreed.
 */
export function fitSpheres(positions: Float32Array, options: FitOptions = DEFAULT_FIT): FitReport {
  const grid = voxelize(positions, options.resolution);
  if (!grid.surfaceCount) {
    return {
      spheres: [],
      method: options.method,
      uncoveredFraction: 0,
      worstUncoveredM: 0,
      overCoverage: 0,
      maxProtrusionM: 0,
      meanProtrusionM: 0,
      voxelM: grid.voxel,
      surfaceVoxels: 0,
      interiorVoxels: 0,
      sealed: false,
    };
  }

  const budget = Math.max(1, Math.round(options.sphereBudget));
  const candidates: { label: string; spheres: Sphere[] }[] = [];
  const wants = (m: FitMethod) => options.method === m || options.method === "auto";

  // The body the fit COVERS may not be the body it is SCORED against. With
  // `convexHull` on, every method works from the hull -- a solid with no
  // pockets and no leaks -- while the report keeps measuring waste and
  // coverage against the real mesh, so the two settings stay comparable
  // and the hull's cost shows up as the number it actually is.
  const hull = options.convexHull;
  const fitPositions = hull ? convexHullPositions(positions) : positions;
  const fitGrid = hull ? voxelize(fitPositions, options.resolution) : grid;
  const suffix = hull ? "+hull" : "";

  if (wants("cluster")) {
    // In `auto` the budget IS the cluster count. The `clusters` slider is
    // the knob for `cluster` mode, and letting it also cap the auto run
    // meant a 50-sphere budget quietly produced a 30-sphere clustering fit
    // and then won the comparison with it -- measured, on four of six arm
    // links.
    const requested = options.method === "auto" ? budget : options.maxSpheres;
    const k = Math.max(1, Math.min(requested, budget, fitGrid.surfaceCount));
    candidates.push({ label: `cluster/surface${suffix}`, spheres: fitSurfaceKMeans(fitGrid, k) });
    const interior = fitInteriorKMeans(fitGrid, k);
    if (interior.length) {
      repairCoverage(fitGrid, interior);
      candidates.push({ label: `cluster/interior${suffix}`, spheres: interior });
    }
  }
  if (wants("boxes")) {
    const boxes = fitBoxes(fitPositions, fitGrid, options.boxResolution, options.aspect);
    if (boxes.length) candidates.push({ label: `boxes${suffix}`, spheres: boxes });
  }
  if (wants("hierarchy")) {
    const levels = Math.max(1, Math.round(options.levels));
    const raw = fitHierarchy(fitPositions, options.baseResolution, levels, options.aspect);
    if (raw.length) {
      // Cap BEFORE tightening: the tighten and prune passes are linear in
      // the sphere count times the voxel count, and paying them on six
      // thousand spheres that are about to be merged down to forty is the
      // difference between a fit that feels instant and one that hangs.
      const capped = enforceBudget(raw, budget);
      tighten(fitGrid, capped);
      candidates.push({ label: `hierarchy/${levels}${suffix}`, spheres: pruneRedundant(fitGrid, capped) });
    }
  }

  const occupancy = buildOccupancy(grid);
  let best = candidates[0]?.spheres ?? [];
  let bestLabel = candidates[0]?.label ?? options.method;
  let bestWaste = Infinity;
  for (const candidate of candidates) {
    if (!candidate.spheres.length) continue;
    // Every family goes through the same cap, so `auto` compares fits of
    // comparable size instead of handing it to whoever spent the most
    // spheres.
    if (candidate.spheres.length > budget) {
      candidate.spheres = enforceBudget(candidate.spheres, budget);
      tighten(fitGrid, candidate.spheres);
    }
    // Covering the hull's SURFACE does not by itself cover a concave mesh's
    // surface -- a pocket floor sits well inside the hull. Coverage is the
    // hard constraint, so it is repaired against the real mesh regardless
    // of what the fit was built from.
    if (hull) repairCoverage(grid, candidate.spheres);
    const report = measureFromGrid(grid, coverageOf(grid, candidate.spheres), candidate.spheres, occupancy);
    // Coverage is the hard constraint: a candidate that leaves surface
    // uncovered is not in the running however little volume it wastes.
    if (report.uncoveredFraction > 0) continue;
    if (report.overCoverage < bestWaste) {
      bestWaste = report.overCoverage;
      best = candidate.spheres;
      bestLabel = candidate.label;
    }
  }

  return {
    ...measureFromGrid(grid, coverageOf(grid, best), best, occupancy),
    spheres: best,
    method: bestLabel,
  };
}

/**
 * Improve the spheres that are already on the link, instead of re-seeding.
 *
 * Every method in `fitSpheres` starts from its own construction -- k-means
 * seeds, a box decomposition -- and throws away whatever was there. That is
 * right for a first fit and wrong for everything after it: once the cover
 * is roughly where you want it, either because a fit put it there or
 * because you dragged it there by hand, the next step is to descend from
 * THAT, not to jump to a different basin and start over.
 *
 * So this runs only the improving half: the wasted-volume descent, the
 * radius tightening, coverage repair, and a redundancy prune. Sphere count
 * only ever goes down (via the prune, or the budget if the set is over it);
 * it never re-seeds, so hand placement survives.
 *
 * `convexHull` and `resolution` still apply, since they define the body
 * being descended against.
 */
export function refineSpheres(
  positions: Float32Array,
  existing: Sphere[],
  options: FitOptions = DEFAULT_FIT,
): FitReport {
  const grid = voxelize(positions, options.resolution);
  const spheres = existing.map((s) => ({ center: [...s.center] as [number, number, number], radius: s.radius }));
  const occupancy = buildOccupancy(grid);

  if (!grid.surfaceCount || !spheres.length) {
    return {
      ...measureFromGrid(grid, coverageOf(grid, spheres), spheres, occupancy),
      spheres,
      method: "refine",
    };
  }

  const budget = Math.max(1, Math.round(options.sphereBudget));
  const fitGrid = options.convexHull
    ? voxelize(convexHullPositions(positions), options.resolution)
    : grid;

  if (spheres.length > budget) {
    const merged = enforceBudget(spheres, budget);
    spheres.length = 0;
    spheres.push(...merged);
  }

  const before = measureFromGrid(grid, coverageOf(grid, existing), existing, occupancy);

  descendWaste(fitGrid, spheres);
  tighten(fitGrid, spheres);
  // Coverage is repaired against the REAL mesh whatever the descent used,
  // for the same reason `fitSpheres` does: the hull's surface is not the
  // part's surface.
  repairCoverage(grid, spheres);
  const kept = pruneRedundant(grid, spheres);
  repairCoverage(grid, kept);
  const after = measureFromGrid(grid, coverageOf(grid, kept), kept, occupancy);

  // Refine must never hand back something worse than it was given.
  //
  // The descent minimises each sphere's OWN outside volume, which is only a
  // proxy for the union's -- so on a cover that is already near a local
  // optimum it can trade a little of the real objective for a lot of the
  // proxy. Measured: link_4 went 62.0% to 63.1% that way. Keeping the
  // better of the two costs one measurement and makes the button safe to
  // press repeatedly, which is how anyone will actually use it.
  //
  // A cover with holes is not in the running whatever it wastes: closing
  // them is the whole point, and it is paid for in volume.
  const wasValid = before.uncoveredFraction === 0;
  const keepOld = wasValid && after.uncoveredFraction === 0 && before.overCoverage <= after.overCoverage;
  return keepOld
    ? { ...before, spheres: existing.map((s) => ({ center: [...s.center] as [number, number, number], radius: s.radius })), method: "refine/no-change" }
    : { ...after, spheres: kept, method: `refine${options.convexHull ? "+hull" : ""}` };
}

/** Which surface voxels this sphere set covers. */
function coverageOf(grid: Grid, spheres: Sphere[]): Uint8Array {
  const cx = grid.surfaceCentres;
  const covered = new Uint8Array(grid.surfaceCount);
  for (let vi = 0; vi < grid.surfaceCount; vi += 1)
    for (const sphere of spheres) {
      const dx = cx[vi * 3] - sphere.center[0];
      const dy = cx[vi * 3 + 1] - sphere.center[1];
      const dz = cx[vi * 3 + 2] - sphere.center[2];
      if (dx * dx + dy * dy + dz * dz <= sphere.radius * sphere.radius) { covered[vi] = 1; break; }
    }
  return covered;
}

/**
 * Occupancy lookup: is this point inside the body (interior or surface)?
 *
 * Built as a flat grid rather than a distance query because the volume
 * measurement below samples millions of points and a per-sample scan over
 * voxel lists would dominate the fit.
 */
function buildOccupancy(grid: Grid): Uint8Array {
  const occ = new Uint8Array(grid.nx * grid.ny * grid.nz);
  const at = (i: number, j: number, k: number) => (i * grid.ny + j) * grid.nz + k;
  for (let n = 0; n < grid.surface.length; n += 1) if (grid.surface[n]) occ[n] = 1;
  for (let i = 0; i < grid.interiorCount; i += 1) {
    const gi = Math.floor((grid.interiorCentres[i * 3] - grid.origin[0]) / grid.voxel);
    const gj = Math.floor((grid.interiorCentres[i * 3 + 1] - grid.origin[1]) / grid.voxel);
    const gk = Math.floor((grid.interiorCentres[i * 3 + 2] - grid.origin[2]) / grid.voxel);
    if (gi >= 0 && gj >= 0 && gk >= 0 && gi < grid.nx && gj < grid.ny && gk < grid.nz)
      occ[at(gi, gj, gk)] = 1;
  }
  return occ;
}

/**
 * The objective, measured: coverage first, then wasted volume.
 *
 * `outsideVolumeFraction` is the quantity to minimise -- the share of the
 * sphere model's volume that lies outside the part. It is measured by
 * stratified sampling inside each sphere against the occupancy grid, not
 * inferred from radii, because two covers with identical radii can waste
 * very different amounts depending on where their centres sit.
 *
 * Coverage is reported alongside and is NOT traded against it: a cover
 * that wastes less volume by leaving surface uncovered is not better, it
 * is broken, because an uncovered patch is a blind spot in collision
 * search that the CCD verifier will only discover at the end.
 */
function measureFromGrid(
  grid: Grid,
  covered: Uint8Array,
  spheres: Sphere[],
  occupancy?: Uint8Array,
): Omit<FitReport, "spheres" | "method"> {
  const n = grid.surfaceCount;
  const cx = grid.surfaceCentres;
  let uncovered = 0;
  let worstUncovered = 0;
  for (let vi = 0; vi < n; vi += 1) {
    if (covered[vi]) continue;
    uncovered += 1;
    let nearest = Infinity;
    for (const sphere of spheres) {
      const dx = cx[vi * 3] - sphere.center[0];
      const dy = cx[vi * 3 + 1] - sphere.center[1];
      const dz = cx[vi * 3 + 2] - sphere.center[2];
      nearest = Math.min(nearest, Math.sqrt(dx * dx + dy * dy + dz * dz) - sphere.radius);
    }
    worstUncovered = Math.max(worstUncovered, nearest);
  }

  const occ = occupancy ?? buildOccupancy(grid);
  const at = (i: number, j: number, k: number) => (i * grid.ny + j) * grid.nz + k;

  // The union, painted onto the same grid the occupancy lives on. Each
  // sphere only visits the cells in its own bounding box, so the cost is
  // the model's volume rather than the grid's -- which is what makes this
  // affordable at the resolutions the panel offers.
  const union = new Uint8Array(grid.nx * grid.ny * grid.nz);
  for (const sphere of spheres) {
    const r = sphere.radius;
    const r2 = r * r;
    const i0 = Math.max(0, Math.floor((sphere.center[0] - r - grid.origin[0]) / grid.voxel));
    const j0 = Math.max(0, Math.floor((sphere.center[1] - r - grid.origin[1]) / grid.voxel));
    const k0 = Math.max(0, Math.floor((sphere.center[2] - r - grid.origin[2]) / grid.voxel));
    const i1 = Math.min(grid.nx - 1, Math.ceil((sphere.center[0] + r - grid.origin[0]) / grid.voxel));
    const j1 = Math.min(grid.ny - 1, Math.ceil((sphere.center[1] + r - grid.origin[1]) / grid.voxel));
    const k1 = Math.min(grid.nz - 1, Math.ceil((sphere.center[2] + r - grid.origin[2]) / grid.voxel));
    for (let i = i0; i <= i1; i += 1) {
      const px = grid.origin[0] + (i + 0.5) * grid.voxel - sphere.center[0];
      for (let j = j0; j <= j1; j += 1) {
        const py = grid.origin[1] + (j + 0.5) * grid.voxel - sphere.center[1];
        const partial = px * px + py * py;
        if (partial > r2) continue;
        for (let k = k0; k <= k1; k += 1) {
          const pz = grid.origin[2] + (k + 0.5) * grid.voxel - sphere.center[2];
          if (partial + pz * pz <= r2) union[at(i, j, k)] = 1;
        }
      }
    }
  }

  const cellVol = grid.voxel * grid.voxel * grid.voxel;
  let outsideCells = 0;
  let bodyCells = 0;
  for (let idx = 0; idx < union.length; idx += 1) {
    if (occ[idx]) bodyCells += 1;
    else if (union[idx]) outsideCells += 1;
  }
  const outsideVol = outsideCells * cellVol;
  const bodyVol = bodyCells * cellVol;

  let maxProtrusion = 0;
  let sumProtrusion = 0;
  for (const sphere of spheres) {
    let nearestSurf = Infinity;
    for (let vi = 0; vi < n; vi += 1) {
      const dx = cx[vi * 3] - sphere.center[0];
      const dy = cx[vi * 3 + 1] - sphere.center[1];
      const dz = cx[vi * 3 + 2] - sphere.center[2];
      nearestSurf = Math.min(nearestSurf, Math.sqrt(dx * dx + dy * dy + dz * dz));
    }
    const protrusion = Math.max(0, sphere.radius - nearestSurf);
    maxProtrusion = Math.max(maxProtrusion, protrusion);
    sumProtrusion += protrusion;
  }

  return {
    uncoveredFraction: n ? uncovered / n : 0,
    worstUncoveredM: worstUncovered,
    overCoverage: bodyVol > 0 ? outsideVol / bodyVol : 0,
    maxProtrusionM: maxProtrusion,
    meanProtrusionM: spheres.length ? sumProtrusion / spheres.length : 0,
    voxelM: grid.voxel,
    surfaceVoxels: n,
    interiorVoxels: grid.interiorCount,
    sealed: grid.interiorCount > 0,
  };
}

/**
 * Measure coverage of an arbitrary sphere set against a mesh.
 *
 * Used for hand-edited covers, so that EDITING gets the same numbers as
 * fitting.
 */
export function measure(
  positions: Float32Array,
  spheres: Sphere[],
  resolution = DEFAULT_FIT.resolution,
): Omit<FitReport, "spheres"> {
  const grid = voxelize(positions, resolution);
  const n = grid.surfaceCount;
  const cx = grid.surfaceCentres;
  const covered = new Uint8Array(n);
  for (let vi = 0; vi < n; vi += 1) {
    for (const sphere of spheres) {
      const dx = cx[vi * 3] - sphere.center[0];
      const dy = cx[vi * 3 + 1] - sphere.center[1];
      const dz = cx[vi * 3 + 2] - sphere.center[2];
      if (dx * dx + dy * dy + dz * dz <= sphere.radius * sphere.radius) {
        covered[vi] = 1;
        break;
      }
    }
  }
  return { ...measureFromGrid(grid, covered, spheres), method: "manual" };
}
