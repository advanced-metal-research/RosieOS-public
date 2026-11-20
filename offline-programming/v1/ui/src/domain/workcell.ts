/**
 * Workcell fixtures: the bodies in the cell that are neither the robot nor the
 * part -- the stand the part sits on, a clamp, a table -- drawn in the
 * viewport, listed in the workspace tree, edited by the operator, and sent
 * to the weld planner with every plan so it avoids what the operator sees.
 *
 * Until 2026-09-20 the only such body was a placeholder box the weld
 * planner's seam worker added on its own (`seam_worker/robot/cell.py`,
 * CELL_FIXTURES): nothing drew it, nothing could change it, and the OLP
 * placed parts in clear air beside a stand the planner alone believed in.
 * The default here IS that placeholder, so a program that never touched its
 * stand plans exactly as before; the difference is that the operator can now
 * see it, move it, resize it, or replace it with the real fixture's STEP.
 *
 * Poses are in the cell's WORLD frame (the 1420's floor origin, the pedestal
 * corner), the frame the part's "from the cell origin" offset also uses. A
 * box stands on its bottom-centre: `translationM` is the middle of its
 * footprint and it rises `sizeM[2]` from there. A STEP is placed by its own
 * origin, as an imported part is.
 */
import type { CadWorkpieceMesh } from "../cad/contracts.ts";
import type { OfflineProgrammingProject, RobotModel, Rpy, Vec3 } from "./project.ts";

export type WorkcellFixtureShape =
  | { kind: "box"; sizeM: Vec3 }
  | { kind: "step"; fingerprint: string; filename: string };

export interface WorkcellFixture {
  id: string;
  name: string;
  shape: WorkcellFixtureShape;
  translationM: Vec3;
  rotationRpyRad: Rpy;
  /** The cell's own placeholder, or something the operator made or changed. */
  origin: "cell-default" | "authored";
}

/** The placeholder the weld planner used to add on its own for the 1420: a
 *  0.5 m square box 1 m tall, its footprint centred at x 1.0, y 1.0 on the
 *  floor (`seam_worker/robot/cell.py` CELL_FIXTURES, 2026-09-13, unmeasured).
 *  Kept equal to that table until the real stand is measured. The 1400 has
 *  its table in the URDF and needs nothing here. */
export const STAND_ID = "stand";

export function defaultWorkcellFixtures(model: RobotModel): WorkcellFixture[] {
  if (model !== "rosie_1420_v1") return [];
  return [{
    id: STAND_ID,
    name: "Stand",
    shape: { kind: "box", sizeM: [0.5, 0.5, 1.0] },
    translationM: [1.0, 1.0, 0.0],
    rotationRpyRad: [0, 0, 0],
    origin: "cell-default",
  }];
}

/** A new box stand the operator adds where the cell has none. */
export function newBoxFixture(id = STAND_ID, name = "Stand"): WorkcellFixture {
  return {
    id,
    name,
    shape: { kind: "box", sizeM: [0.5, 0.5, 1.0] },
    translationM: [1.0, 1.0, 0.0],
    rotationRpyRad: [0, 0, 0],
    origin: "authored",
  };
}

/** Add another imported fixture without replacing the stand or any prior import.
 *  STEP geometry keeps its own origin in the cell's world frame. */
export function newStepFixture(
  fixtures: readonly WorkcellFixture[],
  source: { filename: string; fingerprint: string },
): WorkcellFixture {
  const ids = new Set(fixtures.map((fixture) => fixture.id));
  let index = 1;
  while (ids.has(`fixture-${index}`)) index++;
  return {
    id: `fixture-${index}`,
    name: source.filename,
    shape: { kind: "step", filename: source.filename, fingerprint: source.fingerprint },
    translationM: [0, 0, 0],
    rotationRpyRad: [0, 0, 0],
    origin: "authored",
  };
}

export function workcellFixtures(project: OfflineProgrammingProject): WorkcellFixture[] {
  return project.workcellFixtures ?? [];
}

export function fixtureById(project: OfflineProgrammingProject, id: string): WorkcellFixture | null {
  return workcellFixtures(project).find((fixture) => fixture.id === id) ?? null;
}

const mm = (v: number): string => `${Math.round(v * 1000)}`;

/** "500 × 500 × 1000 mm" for a box; the file name for a STEP. */
export function fixtureShapeLabel(fixture: WorkcellFixture): string {
  if (fixture.shape.kind === "box") return `${fixture.shape.sizeM.map(mm).join(" × ")} mm`;
  return fixture.shape.filename || "STEP";
}

/** "at 1000, 1000, 0 mm" in the cell's world frame. */
export function fixturePoseLabel(fixture: WorkcellFixture): string {
  return `at ${fixture.translationM.map(mm).join(", ")} mm`;
}

/** The one line the tree shows under the name. */
export function fixtureSublabel(fixture: WorkcellFixture): string {
  const parts = [fixtureShapeLabel(fixture), fixturePoseLabel(fixture)];
  if (fixture.origin === "cell-default") parts.push("placeholder");
  return parts.join(" · ");
}

/** What the weld planner receives for one fixture: the box as its sides, a
 *  STEP as the triangles the CAD service made of it (CAD-local metres), and
 *  the pose in the cell's world frame. Snake case: it is the planner's word. */
export interface WorkcellFixtureWire {
  id: string;
  name: string;
  shape:
    | { kind: "box"; size_m: Vec3 }
    | { kind: "mesh"; filename: string; fingerprint: string; vertices: number[]; faces: number[] };
  xyz_m: Vec3;
  rpy_rad: Rpy;
}

/** A STEP fixture whose mesh this session does not hold cannot be sent; the
 *  planner would otherwise clear a body it cannot see. */
export class WorkcellFixtureUnavailable extends Error {
  readonly fixture: WorkcellFixture;
  constructor(fixture: WorkcellFixture) {
    super(`the fixture "${fixture.name}" is a STEP this session has not tessellated yet; wait for it or replace it`);
    this.fixture = fixture;
  }
}

export function workcellFixturesWire(
  fixtures: readonly WorkcellFixture[],
  meshes: ReadonlyMap<string, CadWorkpieceMesh>,
): WorkcellFixtureWire[] {
  return fixtures.map((fixture) => {
    if (fixture.shape.kind === "box") {
      return {
        id: fixture.id,
        name: fixture.name,
        shape: { kind: "box", size_m: [...fixture.shape.sizeM] as Vec3 },
        xyz_m: [...fixture.translationM] as Vec3,
        rpy_rad: [...fixture.rotationRpyRad] as Rpy,
      };
    }
    const mesh = meshes.get(fixture.shape.fingerprint.toLowerCase());
    if (!mesh) throw new WorkcellFixtureUnavailable(fixture);
    return {
      id: fixture.id,
      name: fixture.name,
      shape: {
        kind: "mesh",
        filename: fixture.shape.filename,
        fingerprint: fixture.shape.fingerprint,
        vertices: Array.from(mesh.positionsM),
        faces: Array.from(mesh.indices),
      },
      xyz_m: [...fixture.translationM] as Vec3,
      rpy_rad: [...fixture.rotationRpyRad] as Rpy,
    };
  });
}

/** One field of a fixture edited from the Stand card; values in mm and degrees. */
export type WorkcellFixtureField = "sx" | "sy" | "sz" | "x" | "y" | "z" | "rx" | "ry" | "rz";

export function editWorkcellFixture(fixture: WorkcellFixture, field: WorkcellFixtureField, value: number): WorkcellFixture {
  if (!Number.isFinite(value)) return fixture;
  const next: WorkcellFixture = {
    ...fixture,
    shape: fixture.shape.kind === "box" ? { kind: "box", sizeM: [...fixture.shape.sizeM] as Vec3 } : { ...fixture.shape },
    translationM: [...fixture.translationM] as Vec3,
    rotationRpyRad: [...fixture.rotationRpyRad] as Rpy,
    origin: "authored",
  };
  const clamp = (v: number, lo: number, hi: number): number => Math.min(hi, Math.max(lo, v));
  switch (field) {
    case "sx": case "sy": case "sz": {
      if (next.shape.kind !== "box") return fixture;
      // Sides from a millimetre to five metres: a stand, not a hall.
      next.shape.sizeM[field === "sx" ? 0 : field === "sy" ? 1 : 2] = clamp(value, 1, 5_000) / 1_000;
      break;
    }
    case "x": case "y": case "z":
      next.translationM[field === "x" ? 0 : field === "y" ? 1 : 2] = clamp(value, -5_000, 5_000) / 1_000;
      break;
    case "rx": case "ry": case "rz":
      next.rotationRpyRad[field === "rx" ? 0 : field === "ry" ? 1 : 2] = clamp(value, -360, 360) * Math.PI / 180;
      break;
  }
  return next;
}
