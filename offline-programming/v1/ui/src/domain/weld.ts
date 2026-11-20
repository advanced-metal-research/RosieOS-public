import type {
  AuthoredProgramRow,
  Rpy,
  Vec3,
  WeavePattern,
  WeldProcessParameters,
} from "./project.ts";

export const WEAVE_PATTERNS: readonly WeavePattern[] = [
  "none",
  "zigzag",
  "circular",
  "trapezoidal",
];

/** Resolved per-row torch orientation and weave strategy with safe defaults. */
export interface ResolvedTorchSettings {
  workAngleDeg: number;
  travelAngleDeg: number;
  spinAngleDeg: number;
}

export interface ResolvedWeaveSettings {
  pattern: WeavePattern;
  amplitudeMm: number;
  frequencyHz: number;
  dwellMs: number;
}

export function rowTorchSettings(
  row: Pick<
    AuthoredProgramRow,
    "torchWorkAngleDeg" | "torchTravelAngleDeg" | "torchSpinAngleDeg"
  >,
): ResolvedTorchSettings {
  return {
    workAngleDeg: clampAngle(finite(row.torchWorkAngleDeg, 0)),
    travelAngleDeg: clampAngle(finite(row.torchTravelAngleDeg, 0)),
    spinAngleDeg: clampAngle(finite(row.torchSpinAngleDeg, 0), 180),
  };
}

export function rowWeaveSettings(
  row: Pick<
    AuthoredProgramRow,
    "weavePattern" | "weaveAmplitudeMm" | "weaveFrequencyHz" | "weaveDwellMs"
  >,
): ResolvedWeaveSettings {
  return {
    pattern: normalizeWeavePattern(row.weavePattern),
    amplitudeMm: Math.max(0, finite(row.weaveAmplitudeMm, 0)),
    frequencyHz: Math.max(0, finite(row.weaveFrequencyHz, 0)),
    dwellMs: Math.max(0, finite(row.weaveDwellMs, 0)),
  };
}

export function processTorchDefaults(
  process: WeldProcessParameters | undefined,
): ResolvedTorchSettings & ResolvedWeaveSettings {
  return {
    workAngleDeg: clampAngle(finite(process?.torchWorkAngleDeg, 45)),
    travelAngleDeg: clampAngle(finite(process?.torchTravelAngleDeg, 10)),
    spinAngleDeg: clampAngle(finite(process?.torchSpinAngleDeg, 0), 180),
    pattern: normalizeWeavePattern(process?.weavePattern),
    amplitudeMm: Math.max(0, finite(process?.weaveAmplitudeMm, 2)),
    frequencyHz: Math.max(0, finite(process?.weaveFrequencyHz, 1)),
    dwellMs: Math.max(0, finite(process?.weaveDwellMs, 0)),
  };
}

export function normalizeWeavePattern(value: unknown): WeavePattern {
  return (WEAVE_PATTERNS as readonly string[]).includes(String(value))
    ? String(value) as WeavePattern
    : "none";
}

/**
 * Set arc/weld output consistently on a row. The spreadsheet keeps
 * `weldEnabled`, `arcState`, and the RevPi weld I/O action as one authored
 * intent rather than three independently editable flags.
 */
export function setRowArc(row: AuthoredProgramRow, on: boolean): void {
  row.weldEnabled = on;
  row.arcState = on ? "on" : "off";
  row.revPiIOAction = on ? "weld_on" : "weld_off";
}

/**
 * Torch guide vectors for the 3D stage. All math is plain arrays so it stays
 * testable without Three.js.
 *
 * The rotation sequence matches the GradientOS controller's
 * `_build_weld_orientations` and its web-ui preview exactly:
 * - the torch direction starts pointing straight down (-Z, work angle 0);
 * - work angle rotates about the horizontal-projected travel axis, tilting
 *   the torch across the seam toward the normal side;
 * - travel angle rotates about the normal axis (up x travel); positive tips
 *   the torch direction backward so the torch body leans toward travel;
 * - spin swings the tilted direction azimuthally about world up.
 */
export interface TorchGuideVectors {
  travel: Vec3;
  side: Vec3;
  torchUp: Vec3;
}

export function torchGuideVectors(
  travelDirection: Vec3,
  torch: ResolvedTorchSettings,
): TorchGuideVectors | null {
  const travel = normalize(travelDirection);
  if (!travel) return null;
  const worldUp: Vec3 = [0, 0, 1];
  const travelDotUp = dot(travel, worldUp);
  const horizontalProjection: Vec3 = [
    travel[0] - worldUp[0] * travelDotUp,
    travel[1] - worldUp[1] * travelDotUp,
    travel[2] - worldUp[2] * travelDotUp,
  ];
  // Reject near-vertical CAD tangents before normalization can amplify tiny
  // horizontal rounding noise into a fully trusted arbitrary azimuth.
  if (length(horizontalProjection) <= 1e-6) return null;
  const horizontalForward = normalize(horizontalProjection);
  if (!horizontalForward) return null;
  const side = normalize(cross(worldUp, horizontalForward));
  if (!side) return null;
  let torchDir: Vec3 = [-worldUp[0], -worldUp[1], -worldUp[2]];
  torchDir = rotateAboutAxis(torchDir, horizontalForward, degToRad(torch.workAngleDeg));
  torchDir = rotateAboutAxis(torchDir, side, degToRad(torch.travelAngleDeg));
  torchDir = rotateAboutAxis(torchDir, worldUp, degToRad(torch.spinAngleDeg));
  return {
    travel,
    side,
    torchUp: [-torchDir[0], -torchDir[1], -torchDir[2]],
  };
}

/**
 * Build the authored torch TCP orientation as planner-frame XYZ-order RPY.
 *
 * The Motion Server/URDF constraint identifies local tool +X as the torch
 * direction. Local +Y is the seam-travel secondary axis at zero angles; after
 * the torch direction tilts, +Y is projected onto the plane perpendicular to
 * +X. This keeps a deterministic right-handed frame while making spin observable
 * as a full TCP orientation rather than only as an under-specified tool axis.
 *
 * The planner frame differs from workstation world by translation only, so the
 * world-space guide vectors are valid planner-frame orientation components.
 * Returns null when a horizontal seam azimuth or complete orthonormal frame
 * cannot be defined; callers must retain their existing placeholder orientation.
 */
export function torchOrientationRpy(
  travelDirection: Vec3,
  torch: ResolvedTorchSettings,
): Rpy | null {
  const guide = torchGuideVectors(travelDirection, torch);
  if (!guide) return null;

  const worldUp: Vec3 = [0, 0, 1];
  const travelDotUp = dot(guide.travel, worldUp);
  const horizontalTravel = normalize([
    guide.travel[0] - worldUp[0] * travelDotUp,
    guide.travel[1] - worldUp[1] * travelDotUp,
    guide.travel[2] - worldUp[2] * travelDotUp,
  ]);
  if (!horizontalTravel) return null;

  const toolX = normalize([
    -guide.torchUp[0],
    -guide.torchUp[1],
    -guide.torchUp[2],
  ]);
  if (!toolX) return null;

  // Spin is a world-up rotation of the whole torch frame. Rotating the
  // secondary reference as well keeps spin observable even when tool +X is
  // exactly vertical (where rotating the torch direction alone is a no-op).
  const spunTravel = rotateAboutAxis(
    horizontalTravel,
    worldUp,
    degToRad(torch.spinAngleDeg),
  );
  const projectedTravel = subtractProjection(spunTravel, toolX);
  let toolY = normalize(projectedTravel);
  if (!toolY) {
    toolY = normalize(cross(worldUp, toolX));
  }
  if (!toolY) return null;

  let toolZ = normalize(cross(toolX, toolY));
  if (!toolZ) return null;
  toolY = normalize(cross(toolZ, toolX));
  if (!toolY) return null;
  toolZ = normalize(cross(toolX, toolY));
  if (!toolZ) return null;

  // Matrix columns are the local +X/+Y/+Z axes expressed in planner/world.
  return rotationMatrixToXyzRpy([
    [toolX[0], toolY[0], toolZ[0]],
    [toolX[1], toolY[1], toolZ[1]],
    [toolX[2], toolY[2], toolZ[2]],
  ]);
}

/**
 * Recompute seam-associated row orientations after spreadsheet position or
 * torch-angle edits. Weld rows own the authored angles. Approach and retract
 * inherit the first and last valid weld frames respectively. A row whose seam
 * tangent is vertical/degenerate retains its existing RPY (fail closed).
 */
export function synchronizeTorchOrientations(
  rows: AuthoredProgramRow[],
  seamIDs?: ReadonlySet<string>,
): void {
  const selectedSeamIDs = seamIDs ?? new Set(
    rows.flatMap((row) => row.seamId ? [row.seamId] : []),
  );
  for (const seamID of selectedSeamIDs) {
    const seamRows = rows.filter((row) =>
      row.seamId === seamID &&
      (row.rowType ?? "motion") === "motion"
    );
    const weldRows = rows.filter((row) =>
      row.seamId === seamID &&
      (row.rowType ?? "motion") === "motion" &&
      row.phase === "weld"
    );
    if (weldRows.length === 0) continue;

    let previousRpy = previousMotionRpy(rows, seamRows[0]!);
    const resolved = weldRows.map((row, index) => {
      if (row.sourceState === "teach_modified") {
        previousRpy = rowRpy(row);
        return null;
      }
      const orientation = torchOrientationRpy(
        rowTravelDirection(weldRows, index, seamRows),
        rowTorchSettings(row),
      );
      if (!orientation) {
        previousRpy = rowRpy(row);
        return null;
      }
      const continuous = unwrapRpy(orientation, previousRpy);
      setRowRpy(row, continuous);
      previousRpy = continuous;
      return continuous;
    });
    let firstIndex = -1;
    let lastIndex = -1;
    for (let index = 0; index < resolved.length; index += 1) {
      if (resolved[index] !== null) {
        firstIndex = index;
        break;
      }
    }
    for (let index = resolved.length - 1; index >= 0; index -= 1) {
      if (resolved[index] !== null) {
        lastIndex = index;
        break;
      }
    }
    if (firstIndex >= 0) {
      const first = resolved[firstIndex]!;
      const source = weldRows[firstIndex]!;
      for (const row of rows) {
        if (row.seamId === seamID && row.phase === "approach") {
          if (row.sourceState === "teach_modified") continue;
          setRowRpy(row, first);
          copyRowTorchSettings(source, row);
        }
      }
    }
    if (lastIndex >= 0) {
      const last = resolved[lastIndex]!;
      const source = weldRows[lastIndex]!;
      for (const row of rows) {
        if (row.seamId === seamID && row.phase === "retract") {
          if (row.sourceState === "teach_modified") continue;
          setRowRpy(row, last);
          copyRowTorchSettings(source, row);
        }
      }
    }
  }
}

/**
 * Sample a lateral weave ribbon along a path polyline. The wavelength comes
 * from travel speed and weave frequency; amplitude offsets alternate across
 * the seam using the pattern's lateral profile. Returns [] when the weave is
 * disabled or the geometry cannot support it.
 */
export function weavePreviewPoints(input: {
  pathPointsM: readonly Vec3[];
  travelSpeedMps: number;
  weave: ResolvedWeaveSettings;
  maxPoints?: number;
}): Vec3[] {
  const { pathPointsM, travelSpeedMps, weave } = input;
  if (weave.pattern === "none" || weave.amplitudeMm <= 0 || weave.frequencyHz <= 0) {
    return [];
  }
  if (pathPointsM.length < 2 || travelSpeedMps <= 0) return [];
  const wavelengthM = travelSpeedMps / weave.frequencyHz;
  if (!Number.isFinite(wavelengthM) || wavelengthM <= 0) return [];
  const amplitudeM = weave.amplitudeMm / 1000;

  const cumulativeM = [0];
  for (let index = 1; index < pathPointsM.length; index += 1) {
    cumulativeM.push(
      cumulativeM[index - 1]! +
        distance(pathPointsM[index - 1]!, pathPointsM[index]!),
    );
  }
  const totalLengthM = cumulativeM.at(-1)!;
  if (totalLengthM <= 0) return [];

  const maxPoints = Math.max(16, input.maxPoints ?? 1200);
  const stepM = Math.max(wavelengthM / 12, totalLengthM / maxPoints, 0.0005);
  const points: Vec3[] = [];
  for (let distanceM = 0; distanceM <= totalLengthM + 1e-9; distanceM += stepM) {
    const clamped = Math.min(distanceM, totalLengthM);
    const { point, tangent } = samplePolyline(pathPointsM, cumulativeM, clamped);
    const worldUp: Vec3 = [0, 0, 1];
    let side = normalize(cross(tangent, worldUp));
    if (!side) side = normalize(cross(tangent, [1, 0, 0]));
    if (!side) {
      points.push(point);
      continue;
    }
    const phase = clamped / wavelengthM;
    const offset = amplitudeM * lateralProfile(weave.pattern, phase);
    points.push([
      point[0] + side[0] * offset,
      point[1] + side[1] * offset,
      point[2] + side[2] * offset,
    ]);
  }
  return points;
}

/** Lateral offset factor in [-1, 1] for one weave cycle at the given phase. */
export function lateralProfile(pattern: WeavePattern, phase: number): number {
  const cycle = phase - Math.floor(phase);
  switch (pattern) {
    case "zigzag": {
      // Triangle wave: 0 → +1 → -1 → 0 across one cycle.
      const value = cycle < 0.25
        ? cycle * 4
        : cycle < 0.75
          ? 2 - cycle * 4
          : cycle * 4 - 4;
      return value;
    }
    case "circular":
      return Math.sin(cycle * Math.PI * 2);
    case "trapezoidal": {
      const triangle = cycle < 0.25
        ? cycle * 4
        : cycle < 0.75
          ? 2 - cycle * 4
          : cycle * 4 - 4;
      // Flatten the crests so the torch dwells at each sidewall.
      return Math.max(-1, Math.min(1, triangle * 1.6));
    }
    default:
      return 0;
  }
}

function samplePolyline(
  points: readonly Vec3[],
  cumulativeM: readonly number[],
  distanceM: number,
): { point: Vec3; tangent: Vec3 } {
  for (let index = 1; index < points.length; index += 1) {
    if (distanceM > cumulativeM[index]! && index < points.length - 1) continue;
    const left = points[index - 1]!;
    const right = points[index]!;
    const segmentM = cumulativeM[index]! - cumulativeM[index - 1]!;
    const ratio = segmentM > 0
      ? Math.max(0, Math.min(1, (distanceM - cumulativeM[index - 1]!) / segmentM))
      : 0;
    const tangent = normalize([
      right[0] - left[0],
      right[1] - left[1],
      right[2] - left[2],
    ]) ?? [1, 0, 0];
    return {
      point: [
        left[0] + (right[0] - left[0]) * ratio,
        left[1] + (right[1] - left[1]) * ratio,
        left[2] + (right[2] - left[2]) * ratio,
      ],
      tangent,
    };
  }
  return { point: [...points.at(-1)!] as Vec3, tangent: [1, 0, 0] };
}

function rowTravelDirection(
  rows: readonly AuthoredProgramRow[],
  index: number,
  seamRows: readonly AuthoredProgramRow[] = rows,
): Vec3 {
  const current = rows[index]?.targetTcpPose;
  if (!current) return [0, 0, 0];
  if (rows.length === 1) {
    const approach = seamRows.find((row) => row.phase === "approach")?.targetTcpPose;
    const retract = [...seamRows].reverse()
      .find((row) => row.phase === "retract")?.targetTcpPose;
    if (approach && retract) {
      return [
        retract[0] - approach[0],
        retract[1] - approach[1],
        retract[2] - approach[2],
      ];
    }
  }
  const previous = index > 0 ? rows[index - 1]?.targetTcpPose : current;
  const next = index + 1 < rows.length ? rows[index + 1]?.targetTcpPose : current;
  if (!previous || !next) return [0, 0, 0];
  return [
    next[0] - previous[0],
    next[1] - previous[1],
    next[2] - previous[2],
  ];
}

function setRowRpy(row: AuthoredProgramRow, rpy: Rpy): void {
  if (row.sourceState === "teach_modified") return;
  row.targetTcpPose[3] = rpy[0];
  row.targetTcpPose[4] = rpy[1];
  row.targetTcpPose[5] = rpy[2];
  if (row.currentTcpPose) {
    row.currentTcpPose[3] = rpy[0];
    row.currentTcpPose[4] = rpy[1];
    row.currentTcpPose[5] = rpy[2];
  }
}

function copyRowTorchSettings(
  source: AuthoredProgramRow,
  target: AuthoredProgramRow,
): void {
  const torch = rowTorchSettings(source);
  target.torchWorkAngleDeg = torch.workAngleDeg;
  target.torchTravelAngleDeg = torch.travelAngleDeg;
  target.torchSpinAngleDeg = torch.spinAngleDeg;
}

function rotateAboutAxis(vector: Vec3, axis: Vec3, angleRad: number): Vec3 {
  // Rodrigues rotation; axis must be unit length.
  const cos = Math.cos(angleRad);
  const sin = Math.sin(angleRad);
  const crossed = cross(axis, vector);
  const dotted = dot(axis, vector);
  return [
    vector[0] * cos + crossed[0] * sin + axis[0] * dotted * (1 - cos),
    vector[1] * cos + crossed[1] * sin + axis[1] * dotted * (1 - cos),
    vector[2] * cos + crossed[2] * sin + axis[2] * dotted * (1 - cos),
  ];
}

function subtractProjection(vector: Vec3, axis: Vec3): Vec3 {
  const projection = dot(vector, axis);
  return [
    vector[0] - axis[0] * projection,
    vector[1] - axis[1] * projection,
    vector[2] - axis[2] * projection,
  ];
}

function rotationMatrixToXyzRpy(
  matrix: readonly [
    readonly [number, number, number],
    readonly [number, number, number],
    readonly [number, number, number],
  ],
): Rpy | null {
  if (matrix.flat().some((value) => !Number.isFinite(value))) return null;
  const pitch = Math.asin(Math.max(-1, Math.min(1, -matrix[2][0])));
  const cosPitch = Math.cos(pitch);
  const roll = Math.abs(cosPitch) > 1e-9
    ? Math.atan2(matrix[2][1], matrix[2][2])
    : Math.atan2(-matrix[1][2], matrix[1][1]);
  const yaw = Math.abs(cosPitch) > 1e-9
    ? Math.atan2(matrix[1][0], matrix[0][0])
    : 0;
  return [cleanNumber(roll), cleanNumber(pitch), cleanNumber(yaw)];
}

function cross(left: Vec3, right: Vec3): Vec3 {
  return [
    left[1] * right[2] - left[2] * right[1],
    left[2] * right[0] - left[0] * right[2],
    left[0] * right[1] - left[1] * right[0],
  ];
}

function dot(left: Vec3, right: Vec3): number {
  return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

function normalize(vector: Vec3): Vec3 | null {
  const magnitude = length(vector);
  if (!Number.isFinite(magnitude) || magnitude < 1e-9) return null;
  return [vector[0] / magnitude, vector[1] / magnitude, vector[2] / magnitude];
}

function length(vector: Vec3): number {
  return Math.hypot(vector[0], vector[1], vector[2]);
}

function rowRpy(row: AuthoredProgramRow): Rpy {
  return [
    row.targetTcpPose[3],
    row.targetTcpPose[4],
    row.targetTcpPose[5],
  ];
}

function previousMotionRpy(
  rows: readonly AuthoredProgramRow[],
  row: AuthoredProgramRow,
): Rpy | null {
  const rowIndex = rows.indexOf(row);
  for (let index = rowIndex - 1; index >= 0; index -= 1) {
    const previous = rows[index]!;
    if (
      previous.enabled &&
      (previous.rowType ?? "motion") === "motion"
    ) {
      return rowRpy(previous);
    }
  }
  return null;
}

function unwrapRpy(rpy: Rpy, previous: Rpy | null): Rpy {
  if (!previous) return [...rpy];
  return rpy.map((component, index) =>
    unwrapAngle(component, previous[index]!)
  ) as Rpy;
}

function unwrapAngle(angle: number, previous: number): number {
  const turns = Math.round((previous - angle) / (Math.PI * 2));
  return cleanNumber(angle + turns * Math.PI * 2);
}

function distance(left: Vec3, right: Vec3): number {
  return Math.hypot(
    right[0] - left[0],
    right[1] - left[1],
    right[2] - left[2],
  );
}

function degToRad(value: number): number {
  return value * Math.PI / 180;
}

function clampAngle(value: number, limit = 89): number {
  return Math.max(-limit, Math.min(limit, value));
}

function finite(value: unknown, fallback: number): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function cleanNumber(value: number): number {
  return Math.abs(value) < 1e-12 ? 0 : value;
}
