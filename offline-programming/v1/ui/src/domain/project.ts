import type { RobotV4AcceptedPlan } from "../contracts/robot-v4-program.ts";
import { cellWorkFrame, currentDescriptionSha256, defaultRobotModel, robotDescriptor, robotSettingsSnapshot, type RobotSettingsSnapshot, type CellWorkFrame } from "./robots.ts";
import { defaultWorkcellFixtures, type WorkcellFixture } from "./workcell.ts";
import type {
  AssetRef,
  FreespacePolicy,
  ProgramV2Node,
} from "../contracts/robot-v4-program-v2.ts";

export const PROJECT_SCHEMA = "offline-programming.project.v1" as const;
export const JOINT_ORDER = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"] as const;
/**
 * The frame a part is placed against comes from the CELL, and the cell comes
 * from the server: `domain/robots.ts`, filled at boot from
 * `GET /robots`. It was a constant per model here, which meant the browser
 * carried a list of robots and a table centre for each, and a project on a
 * cell with no positioner was placed 1.913 m from where the planner read it.
 */

/** Does the program link still name a different robot from the one the
 *  viewer draws? */
export function needsCellPinReconcile(project: OfflineProgrammingProject): boolean {
  return project.program.roboticCellType !== project.robot.model;
}

/**
 * Point the program link at the robot the project actually names. Returns
 * whether anything moved.
 *
 * `robot.model` is the authority: it is what the operator picked, what the
 * viewer draws, and -- with its description pin -- what the program.v2
 * document is written for (`toProgramV2Document` reads both off `robot`, so
 * there is no second copy of the model name to fall behind). The Robot model
 * selector writes `roboticCellType` with it, but only on `change`: a project
 * that already held `rosie_1420_v1` renders the select at 1420, so picking
 * 1420 again fires nothing. A stale link once meant a 1400 `.weldplan` whose
 * placement was read from the 1400's table, 2.4 to 2.9 m from the arm; the
 * document no longer carries a pin of its own, and this keeps the link
 * honest on load as well.
 */
export function reconcileCellPin(project: OfflineProgrammingProject): boolean {
  if (!needsCellPinReconcile(project)) return false;
  project.program.roboticCellType = project.robot.model;
  return true;
}

export type { CellWorkFrame };
export { cellWorkFrame };

export type ProjectSchema = typeof PROJECT_SCHEMA;
/** A robot the server holds a description for, by directory name. What
 *  exists is `domain/robots.ts`'s to say, not this type's. */
export type RobotModel = string;
export type TargetMode = "offline_preview" | "local_simulator" | "real_connected";
export type Vec3 = [number, number, number];
/**
 * Roll, pitch, yaw radians. TCP/row poses use the URDF fixed-axis convention
 * `R = Rz(yaw) · Ry(pitch) · Rx(roll)`; fixture/frame placement fields retain
 * their documented Three.js XYZ authoring convention.
 */
export type Rpy = [number, number, number];
export type TcpPose = [number, number, number, number, number, number];
export type JointVector = [number, number, number, number, number, number, number, number, number];
export type UUID = string;

export interface ProjectIdentity {
  id: UUID;
  name: string;
  description: string;
  createdAt: string;
  updatedAt: string;
  revision: number;
}

export interface CadSource {
  kind: "example" | "step";
  filename: string;
  fingerprint: string;
  sourceUnit: "mm" | "m";
  topologyModelId: string;
}

export interface FrameDefinition {
  id: string;
  parentId: string | null;
  translationM: Vec3;
  rotationRpyRad: Rpy;
}

export interface FixturePlacement {
  id: string;
  name: string;
  frameId: string;
  sourceCadFingerprint: string;
  translationM: Vec3;
  rotationRpyRad: Rpy;
}

export interface SeamGeometry {
  kind: "polyline";
  pointsM: Vec3[];
}

export type WeldType = "fillet" | "butt" | "lap" | "tack" | "custom";

export interface AuthoredSeam {
  id: UUID;
  label: string;
  enabled: boolean;
  sourceEdgeId: string;
  partId: string | null;
  geometryType: string;
  trim: { startOffsetM: number; endOffsetM: number };
  reversed: boolean;
  geometry: SeamGeometry;
  workFrameId: string;
  processParameterId: string;
  /** The user-created weld this seam belongs to; absent on legacy seams. */
  weldId?: string;
  weldType?: WeldType;
}

/**
 * Extractor recommendation carried as suggestion metadata on a catalog edge.
 * A recommendation never becomes a seam by itself — it only highlights the
 * edge until the user selects it into a weld (P2.12).
 */
export interface CadEdgeRecommendation {
  window: { start: number; end: number };
  reversed: boolean;
  reason: string;
}

/**
 * One pickable topology edge from the CAD import, persisted with the project
 * so edge selection works offline after save/load.
 */
export interface CadCatalogEdge {
  id: string;
  label: string | null;
  partId: string | null;
  geometryType: string;
  pathClass: string;
  closed: boolean;
  lengthM: number;
  samplesM: Vec3[];
  isSeamCandidate: boolean;
  recommendation: CadEdgeRecommendation | null;
}

export type WeavePattern = "none" | "zigzag" | "circular" | "trapezoidal";

/** Where a weld sits when nothing else states a stickout: the cell's one
 *  default, `seam_worker/seam/seams.py` `DEFAULT_STANDOFF_MM`, set by the
 *  owner (2026-09-11). On bracket_b_2x through the motion planner 12 mm crossed
 *  4 of 22 seams and 17 mm crossed 8 -- at 12 the Ø30 gun baked into link_6
 *  sat 0.2 mm inside the part. Inside both fitted guns' held ranges (Ø16
 *  nozzle 6–25 mm, rosie_link6_gun 15–30 mm). */
export const DEFAULT_TORCH_STANDOFF_MM = 17;

/** How fast a weld walks when nobody has said: 5 mm/s, owner decision
 *  2026-09-21. It was 0.05 m/s -- 50 mm/s, ten times a real travel speed --
 *  and a hand-placed weld COPIES the project's process speed when it is made,
 *  so every such weld left for the planner at 50. At the planner's 250 ms
 *  lattice step that is 12.5 mm between knots; one 12 mm rounded corner fell
 *  between two of them and the tracking certificate refused the plan
 *  (0.65 mm witnessed against 0.5, and 2.1 and 2.5 mm on its neighbours).
 *  One number, in metres a second because `WeldProcessParameters` is. */
export const DEFAULT_TRAVEL_SPEED_MPS = 0.005;

export interface WeldProcessParameters {
  id: string;
  name: string;
  process: "none" | "gmaw" | "gtaw" | "laser" | "custom";
  travelSpeedMps: number;
  approachSpeedMps: number;
  retractSpeedMps: number;
  accelerationMps2: number;
  blendRadiusM: number;
  wireFeedMPerMin: number;
  voltageV: number;
  amperageA: number;
  weldProfileId: number;
  weaveProfileId: number;
  revPiIOSlot: number | null;
  torchWorkAngleDeg?: number;
  torchTravelAngleDeg?: number;
  torchSpinAngleDeg?: number;
  /** How far the tip is held off the work, in millimetres.
   *
   *  Required on the weld node this becomes, and never defaulted there: an
   *  absent standoff is not zero, and zero is the tip inside the part. The
   *  planner refuses a weld that states none. */
  torchStandoffMm?: number;
  weavePattern?: WeavePattern;
  weaveAmplitudeMm?: number;
  weaveFrequencyHz?: number;
  weaveDwellMs?: number;
}

export interface SequenceStep {
  id: UUID;
  seamId: UUID;
  order: number;
  approachOffsetM: number;
  retractOffsetM: number;
}

export type RowMove = "LINE" | "CURVE" | "JOINT";
export type RowPhase = "approach" | "weld" | "retract" | "travel" | "home";
export type RowType = "motion" | "command" | "comment" | "capture" | "calibration" | "io";
export type RowSourceState = "cad_derived" | "teach_modified" | "manual" | "imported";

export interface AuthoredProgramRow {
  id: UUID;
  enabled: boolean;
  label: string;
  rowType?: RowType;
  rowRevision?: number;
  rowHash?: string;
  command: "MOVE" | "WELD_ON" | "WELD_OFF" | "HOME" | "LOOP" | "COMMENT" | "CAPTURE" | "CALIBRATE" | "IO";
  comment?: string;
  loopTargetRowId?: string | null;
  loopIterations?: number;
  returnHomeBeforeRepeat?: boolean;
  moveType: RowMove;
  phase: RowPhase;
  targetFrameType?: string;
  fixtureSide?: string;
  calibrationSection?: number;
  harmoniousPhaseDeg?: number;
  captureSource?: string;
  speedSource?: string;
  seamId: UUID | null;
  passId: string;
  passRole: string;
  toolFrameId: string;
  workFrameId: string;
  fixtureId: string | null;
  targetTcpPose: TcpPose;
  targetJointsRad: JointVector;
  positionerTargetsRad: [number, number, number];
  positionerJointTargetsRad?: JointVector;
  nominalTcpM?: Vec3 | null;
  currentTcpPose?: TcpPose;
  currentJointsRad?: JointVector;
  tcpVelocityMps: number;
  jointVelocityRadS: number;
  accelerationMps2: number;
  jointAccelerationRadS2: number;
  blendRadiusM: number;
  fineStop: boolean;
  /** The segment clock departing this row, in ms. The Motion Server row
   * struct defaults an ABSENT `playback_step_ms` to 10 000 ms and then never
   * consults the velocities, so dropping this field on a round-trip retimes
   * an imported program to a flat 10 s per segment. Preserved verbatim; the
   * editor does not set it. */
  playbackStepMs?: number;
  weldEnabled: boolean;
  arcState: "on" | "off";
  wireFeedMPerMin: number;
  voltageV: number;
  amperageA?: number;
  weldProfileId: number;
  weaveProfileId: number;
  weldProcess: string;
  torchWorkAngleDeg?: number;
  torchTravelAngleDeg?: number;
  torchSpinAngleDeg?: number;
  weavePattern?: WeavePattern;
  weaveAmplitudeMm?: number;
  weaveFrequencyHz?: number;
  weaveDwellMs?: number;
  revPiIOSlot: number | null;
  revPiIOAction: "none" | "weld_on" | "weld_off";
  revPiIOPending?: boolean;
  plannerStatus?: string;
  validationState?: string;
  collisionCheckState?: string;
  limitCheckState?: string;
  plannedPathRevision?: number;
  plannedSampleCount?: number;
  sourceState?: RowSourceState;
  source: "offline_programming" | "cadquery" | "path_planner" | "robot_v4";
  extensions?: Record<string, unknown>;
}

export interface RobotProgramLink {
  schema: "robot.v4.program.v1";
  programId: string;
  programRevision: number;
  baseProgramRevision: number;
  programDigest: string | null;
  catalogRevision: number | null;
  partName: string;
  partNumber: string;
  partRevision: string;
  roboticCellType: string;
  cellConfigRevision: string;
  programState: "draft" | "saved_unplanned" | "needs_plan" | "planning" | "planned" |
    "accepted_simulation" | "accepted_physical" | "execution_ready" | "running" |
    "completed" | "stopped" | "faulted";
  executionMode: "simulation" | "dry_run" | "production";
  editSource: "offline_programming" | "steamdeck" | "motion_server" | "import";
  sourceProjectRevision: number;
  defaults: Record<string, unknown>;
  settings: Record<string, unknown>;
  constraints: Record<string, unknown>;
  extensions: Record<string, unknown>;
}

export interface PlanTrajectorySample {
  index: number;
  rowIndex: number;
  seamId: string | null;
  phase: RowPhase;
  weldOn: boolean;
  timeS: number;
  tcpPose: TcpPose;
  jointPositionsRad: JointVector;
}

export interface PlanEvidence {
  collision: "unavailable" | "clear" | "collision";
  singularity: "unavailable" | "clear" | "warning" | "hard_stop";
  limits: "unavailable" | "clear" | "outside_limits";
  startPose: "unavailable" | "converged" | "mismatch";
  runtimePlayable: boolean | null;
  collisionPoints: Array<{
    waypointIndex: number;
    linkA: string;
    linkB: string;
    depthM: number;
    affectedJoints: string[];
  }>;
  singularityPoints: Array<{
    waypointIndex: number;
    state: string;
    appliedVelocityScale: number;
    affectedJoints: string[];
  }>;
}

export type PlanState = "missing" | "preview" | "server_validated" | "stale" | "rejected";

export interface PlanRecord {
  state: PlanState;
  id: string | null;
  revision: number;
  sourceProjectRevision: number | null;
  sourceRobotModel: RobotModel | null;
  /** `cellSettingsKey` of the settings the program held when planned;
   *  absent on plans made before the Cells menu (read as null). */
  sourceCell?: string | null;
  planner: "browser_preview" | "motion_server_tesseract" | null;
  createdAt: string | null;
  staleReason: string | null;
  evidence: PlanEvidence;
  trajectory: PlanTrajectorySample[];
  acceptedPlan: RobotV4AcceptedPlan | null;
}

export interface SimulationState {
  cursorS: number;
  playbackRate: number;
  loop: boolean;
  selectedRowId: string | null;
  camera: "orbit" | "table" | "tcp";
}

export interface TargetProfileReference {
  mode: TargetMode;
  profileId: string | null;
  manifestRef: string | null;
}

/**
 * What a connected cell said about itself, copied into the program by
 * "Pull cell settings". A program is written against a cell, not a robot
 * model alone: the pair authority names the model, the Jetson and which
 * of the nine wire axes are motors. The copy is what the plan is checked
 * against (`planIsFresh`), so a plan made for one cell reads STALE when
 * the program is pointed at another.
 *
 * The cell's own deviations from its robot model -- narrowed limits,
 * measured kinematic corrections -- arrive as its `machine_planning_calibration.json`, kept here
 * byte for byte and sent with every plan request; the planner applies them
 * and stamps the plan with the hashes the daemon compares at play
 * (docs/ROBOT-DESCRIPTION-CONTRACT.md). Settings pulled before that
 * contract, or from a daemon that predates it, have none of those fields.
 */
export interface CellSettings {
  configurationDigest?: string;
  backend?: string;
  axes?: unknown;
  io?: unknown;
  /** The deployment cell's name, for example a bench cell's. */
  cell: string;
  jetsonId: string | null;
  /** The pair authority's robot model id. */
  robotModelId: string | null;
  /** RTCore's runtime mode when pulled: `sim` or `live`. */
  runtimeMode: string | null;
  /** Which wire axes (J1..J9) the pair config maps to motors. */
  mappedAxisMask: number | null;
  rtcoreHost: string | null;
  pulledAt: string;
  /** The daemon's description model id (the directory it was given). */
  modelId?: string | null;
  /** Identity of that description: sha256 over the files its registry names. */
  robotDescriptionSha256?: string | null;
  /** `sha256:<hex>` of the target's machine_planning_calibration.json, for display and staleness. */
  machinePlanningCalibrationSha256?: string | null;
  /** When the identity was fetched; null when it could not be. */
  cellFetchedAt?: string | null;
  /** The machine_planning_calibration.json itself, base64: pulled from the cell, or loaded from a
   *  file when there is no cell to ask (docs/PLANNING-CONTEXT.md). Kept so a
   *  plan can be made OFFLINE against the same deviation; while a cell is
   *  connected the server fetches its own and these bytes are not read,
   *  because the cell is the source of truth for its settings and a tab
   *  left open is not. */
  machinePlanningCalibrationBase64?: string | null;
  /** Where the bytes came from: the cell, or a file somebody loaded. The
   *  plan's header says `target` either way -- the gate is the hash -- so
   *  this is what lets the UI say which. */
  machinePlanningCalibrationSource?: "target" | "upload" | null;
}

/** The part of the settings a plan depends on, as one comparable string.
 *  The robot identity joins it only when pulled, so settings from before the
 *  contract keep the key their plans were recorded under. */
export function cellSettingsKey(settings: CellSettings | null | undefined): string | null {
  if (!settings) return null;
  const key = [settings.cell, settings.robotModelId ?? "", settings.mappedAxisMask ?? "", settings.configurationDigest ?? "", settings.backend ?? ""];
  if (settings.machinePlanningCalibrationSha256 || settings.robotDescriptionSha256) {
    key.push(settings.robotDescriptionSha256 ?? "", settings.machinePlanningCalibrationSha256 ?? "");
  }
  return key.join(":");
}

/**
 * The description this program was authored against.
 *
 * `robot.model` says WHICH robot; this says which version of it. The name is
 * what a person picked and what everything downstream resolves by, so it is
 * the identity that is stored; the hash is what tells a program authored
 * against last month's description from one authored against today's, since
 * a robot's geometry, limits or torch can change under a name that does not
 * (`docs/ROBOT-DESCRIPTION-CONTRACT.md`).
 *
 * It is provenance, not content: taking a pin never bumps the revision or
 * touches a plan. What it buys is the ability to SAY that the description
 * moved, which is a thing only this program knows -- the cell knows the
 * hashes disagree and refuses, but it cannot know whether the person meant
 * it.
 */
export interface RobotDescriptionPin {
  /** `sha256:<hex>`, as the catalogue reported it when the pin was taken. */
  robotDescriptionSha256: string;
  pinnedAt: string;
  /** Exact-identity settings retained for a later human-readable comparison. */
  settingsSnapshot?: RobotSettingsSnapshot;
  /** Each adoption of a changed description, oldest first. Absent until one
   *  happens, so an untouched program carries no empty list. */
  adopted?: RobotDescriptionAdoption[];
}

/** One adoption: the description this program left, the one it took, and
 *  when. Kept because "somebody decided this was fine" is the only record of
 *  a decision nothing else can check. */
export interface RobotDescriptionAdoption {
  from: string;
  to: string;
  at: string;
}

/** The description a plan for this program's robot would be made against
 *  now -- the connected cell's while one is connected, the server's own
 *  otherwise -- or "" when neither has one, which the Robot card says in its
 *  own words rather than guessing at. */
function currentRobotSha256(project: OfflineProgrammingProject): string {
  return currentDescriptionSha256(project.robot.model);
}

/**
 * The description this program was pinned to and the one the server holds
 * now, when they differ. Null when they agree, when this program has no pin
 * (nothing to compare against), or when the server has no description for
 * this robot at all -- that is a different sentence, and the card says it.
 */
export function robotDescriptionDrift(
  project: OfflineProgrammingProject,
): { from: string; to: string } | null {
  const from = project.robot.description?.robotDescriptionSha256 ?? "";
  const to = currentRobotSha256(project);
  return from && to && from !== to ? { from, to } : null;
}

/**
 * A program with no pin takes the description the server holds now.
 *
 * Silently, and only once: a program saved before the contract was authored
 * against SOMETHING, and there is no record of what, so the honest first
 * answer is "this one". Returns the project unchanged when it already has a
 * pin or the server has no description to pin to, and never bumps the
 * revision -- a program is not edited by being opened.
 */
export function pinRobotDescription(
  project: OfflineProgrammingProject,
  at = nowISO(),
): OfflineProgrammingProject {
  if (project.robot.description?.robotDescriptionSha256) return project;
  // A pin stored under the field's earlier name (`robotSha256`, before
  // 2026-09-20) is the same pin: carry it over rather than silently take
  // the current description as if the program had never been pinned.
  const stored = project.robot.description as (RobotDescriptionPin & { robotSha256?: unknown }) | null | undefined;
  if (stored && typeof stored.robotSha256 === "string" && stored.robotSha256) {
    const next = cloneProject(project);
    const { robotSha256: legacy, ...rest } = stored;
    next.robot.description = { ...rest, robotDescriptionSha256: legacy };
    return next;
  }
  const robotDescriptionSha256 = currentRobotSha256(project);
  if (!robotDescriptionSha256) return project;
  const next = cloneProject(project);
  const settingsSnapshot = robotSettingsSnapshot(project.robot.model, robotDescriptionSha256);
  next.robot.description = { robotDescriptionSha256, pinnedAt: at, ...(settingsSnapshot ? { settingsSnapshot } : {}) };
  return next;
}

/** The description the server holds for `model`, as a fresh pin; null when it
 *  holds none, which the Robot card says rather than inventing a hash. */
export function robotDescriptionPinFor(
  model: string,
  at = nowISO(),
): RobotDescriptionPin | null {
  const robotDescriptionSha256 = currentDescriptionSha256(model);
  const settingsSnapshot = robotSettingsSnapshot(model, robotDescriptionSha256);
  return robotDescriptionSha256 ? { robotDescriptionSha256, pinnedAt: at, ...(settingsSnapshot ? { settingsSnapshot } : {}) } : null;
}

/** Pin to the current description of whatever robot the draft now names,
 *  keeping no history: adoptions belong to the robot they were decided for,
 *  and this is a different robot. */
export function setRobotDescriptionPin(
  draft: OfflineProgrammingProject,
  at = nowISO(),
): void {
  draft.robot.description = robotDescriptionPinFor(draft.robot.model, at);
}

/**
 * Take the description the server holds now as the one this program is
 * authored against, and record that somebody decided so.
 *
 * It re-checks NOTHING. The geometry, the limits and the torch may all have
 * moved; the program is not re-planned, no stored plan becomes playable, and
 * the cell still refuses every plan made before the change. What this
 * changes is one line in this file and who is answerable for it.
 */
export function adoptRobotDescription(
  draft: OfflineProgrammingProject,
  at = nowISO(),
): void {
  const to = currentRobotSha256(draft);
  if (!to) return;
  const from = draft.robot.description?.robotDescriptionSha256 ?? "";
  const adopted = [...(draft.robot.description?.adopted ?? [])];
  if (from && from !== to) adopted.push({ from, to, at });
  const settingsSnapshot = robotSettingsSnapshot(draft.robot.model, to);
  draft.robot.description = {
    robotDescriptionSha256: to,
    pinnedAt: at,
    ...(settingsSnapshot ? { settingsSnapshot } : {}),
    ...(adopted.length > 0 ? { adopted } : {}),
  };
}

export interface OfflineProgrammingProject {
  schema: ProjectSchema;
  identity: ProjectIdentity;
  cad: CadSource;
  /** Weld parts in this workspace. Top-level CAD/weld fields are the active
   * part's editable working copy; older projects contain that one part. */
  workpieces?: WorkpieceRecord[];
  activeWorkpieceId?: string;
  robot: {
    model: RobotModel;
    jointOrder: typeof JOINT_ORDER;
    /** The connected cell's settings, once pulled; absent on programs
     *  authored offline or saved before the Cells menu. */
    cell?: CellSettings | null;
    /** Which version of that robot this program was authored against; absent
     *  on programs saved before the description contract, which take the
     *  current one the first time they are opened. */
    description?: RobotDescriptionPin | null;
  };
  target: TargetProfileReference;
  frames: FrameDefinition[];
  fixtures: FixturePlacement[];
  /** The stand and the other bodies in the cell that are neither the robot
   *  nor the part: drawn, editable, and sent to the weld planner as what it
   *  must clear. See `domain/workcell.ts`. */
  workcellFixtures: WorkcellFixture[];
  /** Pickable CAD topology edges; empty until a part is imported. */
  edgeCatalog: CadCatalogEdge[];
  seams: AuthoredSeam[];
  /** Rails authoring (weld planner seams), chosen at program creation. A
   *  program authors welds ONE way — rails-mode seam clicks or the classic
   *  edge-draft flow — because a seam owned by both would be welded twice.
   *  Program state, not a session toggle; switching is allowed only while
   *  the other flow has authored nothing. */
  railsAuthoring?: boolean;
  processParameters: WeldProcessParameters[];
  sequence: SequenceStep[];
  program: RobotProgramLink;
  /**
   * The authored program: `robot.v4.program.v2` nodes, one spreadsheet row
   * each. In both authoring modes this is what the operator edits; `rows`
   * is compiled from it (in the browser for classic welds, on the weld
   * planner for rails welds) and is never edited directly. Empty on
   * projects saved before nodes existed; hydration builds classic nodes
   * from `seams` + `sequence` + `rows`, and a legacy rails program stays
   * rows-only until its welds are re-authored.
   */
  nodes: ProgramV2Node[];
  /** The program.v2 header the project cannot derive from anything else. */
  programV2: ProgramV2Header;
  rows: AuthoredProgramRow[];
  plan: PlanRecord;
  /**
   * The joint-space daemon's trajectory for this program, as the weld
   * planner's motion server summarised it when the rails compile refined.
   * Only the summary lives here; the bytes stay on the motion server by
   * digest and the OLP server fetches them when the program is loaded to the
   * robot. Cleared by any authoring change, the way the plan is staled: a
   * trajectory whose program moved on is not this program's trajectory.
   */
  denseTrajectory: DenseTrajectoryRecord | null;
  /** The plans this program has had, newest first (see planHistory.ts);
   *  absent on programs saved before it. */
  planHistory?: PlanHistoryEntry[];
  simulation: SimulationState;
}

export interface WorkpieceRecord {
  id: string;
  cad: CadSource;
  fixtures: FixturePlacement[];
  edgeCatalog: CadCatalogEdge[];
  seams: AuthoredSeam[];
  nodes: ProgramV2Node[];
  sequence: SequenceStep[];
  rows: AuthoredProgramRow[];
}

/** One plan the program had. Rails entries hold the dense trajectory
 *  record (the bytes stay on the motion server by digest); classic
 *  entries hold the plan record with its sampled trajectory. */
export interface PlanHistoryEntry {
  id: string;
  createdAt: string;
  kind: "rails" | "classic";
  planner: string;
  sourceProjectRevision: number;
  /** `cellSettingsKey` of the settings held when planned. */
  sourceCell: string | null;
  robotModel: RobotModel;
  plan: PlanRecord | null;
  denseTrajectory: DenseTrajectoryRecord | null;
  summary: { durationS: number; sampleCount: number; segments: number } | null;
}

export interface DenseTrajectorySegment {
  index: number;
  kind: "freespace" | "weld" | "dwell";
  /** The motion result entry the segment was sampled from: a seam id for a
   *  weld, `<from>-><to>` for a connecting move, `<seam>:dwell-in|out`. */
  sourceId: string;
  sampleCount: number;
  durationS: number;
}

/** The compile answer's `dense` block, in the project's own naming. The six
 *  identity fields are what the daemon fences preload and play on; they
 *  travel to the OLP server unchanged. */
export interface DenseTrajectoryRecord {
  schema: string;
  trajectoryDigest: string;
  requestSha256: string;
  planId: string;
  programId: string;
  programDigest: string;
  manifestRevision: number;
  planRevision: number;
  createdAt: string;
  dtS: number;
  totalSampleCount: number;
  totalDurationS: number;
  bytes: number;
  segments: DenseTrajectorySegment[];
  /** The project revision the compile ran against. */
  sourceProjectRevision: number;
  /** The digest of the program.v2 document that was planned.
   *
   *  Not `programDigest` above -- that is `sha256:<plan request>`, what the
   *  daemon fences on at play. This one answers a different question: is this
   *  still the plan for the program in front of me? A revision comparison
   *  cannot, because a revision moves for edits that change no motion and
   *  stands still for some that do. */
  sourceProgramDigest?: string;
  /** What the plan was made against, as its header says: the daemon refuses
   *  it on any other description or machine_planning_calibration.json. Absent on plans from before
   *  the robot description contract. */
  robot?: DenseRobotRef | null;
  /** The output schedule the plan switches, when it switches any: its
   *  digest (sha256 of the canonical marker list) and marker count. The
   *  schedule is outside the trajectory digest, so Load sends the digest
   *  and the OLP refuses a production Load of a schedule that differs, or
   *  of an output plan without it. Absent for motion-only plans. */
  processMarkersSha256?: string;
  processMarkerCount?: number;
}

/** The trajectory header's `robot` block. */
export interface DenseRobotRef {
  modelId: string;
  robotDescriptionSha256: string;
  machinePlanningCalibrationSha256: string;
  machinePlanningCalibrationSource: "target" | "empty";
  /** The machine the plan was made for ("sha256:" + its rt-core
   *  configuration digest), when there was one. A record, never the fence. */
  machineConfigurationSha256?: string;
}

export interface ProgramV2Header {
  /** The robot the program is for is NOT here: `robot.model` and its
   *  description pin are, and the document is written from them
   *  (docs/PLANNING-CONTEXT.md). Two copies of which robot could only
   *  disagree, and once did. */
  tool: AssetRef | null;
  /** Uniform motion timing multiplier; absent means full planned speed. */
  speedScale?: number;
  defaults: FreespacePolicy;
  plannerOwns: string[];
}

export const PLACEHOLDER_SHA256 = "0".repeat(64);
/** The identity a program carries when it was authored with no description
 *  to pin to; the packer resolves it to the one it packs. */
export const PLACEHOLDER_ROBOT_DESCRIPTION_SHA256 = "sha256:" + "0".repeat(64);

export function defaultProgramV2Header(): ProgramV2Header {
  return {
    tool: null,
    defaults: {
      clearance_min_mm: 20,
      speed_scale: 1,
      torch_policy: "free",
      positioner_policy: "free",
    },
    plannerOwns: [
      "positioner joint values -- prescribed by the planner, not by this document",
      "the joint-space solution of every freespace node -- this document states constraints, never a path",
      "sampling density and where waypoints land on a weld",
      "time parametrization and dynamics",
      "the cell descriptor digest -- the catalog does not publish one yet, so cell.sha256 is a placeholder",
    ],
  };
}

export type AuthoringChange =
  | "cad"
  | "robot"
  | "workcell"
  | "target"
  | "program"
  | "frames"
  | "fixtures"
  | "seams"
  | "process"
  | "sequence"
  | "nodes"
  | "rows";

export type NonPlanningChange = "metadata" | "simulation";
export type ProjectChange = AuthoringChange | NonPlanningChange;

export interface ExecutionReadiness {
  ready: boolean;
  reasons: string[];
}

export function nowISO(): string {
  return new Date().toISOString();
}

export function projectId(): string {
  const randomUUID = globalThis.crypto?.randomUUID?.bind(globalThis.crypto);
  return randomUUID ? randomUUID() : `offline-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

export function emptyEvidence(): PlanEvidence {
  return {
    collision: "unavailable",
    singularity: "unavailable",
    limits: "unavailable",
    startPose: "unavailable",
    runtimePlayable: null,
    collisionPoints: [],
    singularityPoints: [],
  };
}

export function missingPlan(): PlanRecord {
  return {
    state: "missing",
    id: null,
    revision: 0,
    sourceProjectRevision: null,
    sourceRobotModel: null,
    planner: null,
    createdAt: null,
    staleReason: null,
    evidence: emptyEvidence(),
    trajectory: [],
    acceptedPlan: null,
  };
}

export function zeroJoints(values: readonly number[] = []): JointVector {
  return JOINT_ORDER.map((_, index) => finite(values[index], 0)) as JointVector;
}

export function createProject(input: {
  name?: string;
  robotModel?: RobotModel;
  cad?: Partial<CadSource>;
  now?: string;
  id?: string;
} = {}): OfflineProgrammingProject {
  const timestamp = input.now ?? nowISO();
  const id = input.id ?? projectId();
  // One reading of the cell for the whole project: the model, the frame the
  // part hangs off, and the cell the program is pinned to all have to be the
  // same cell, and three separate `input.robotModel ??` defaults is how they
  // come apart.
  const model: RobotModel = input.robotModel ?? defaultRobotModel();
  const work = cellWorkFrame(model);
  const process: WeldProcessParameters = {
    id: "process-default",
    name: "Preview weld",
    process: "gmaw",
    travelSpeedMps: DEFAULT_TRAVEL_SPEED_MPS,
    approachSpeedMps: 0.1,
    retractSpeedMps: 0.1,
    accelerationMps2: 0.75,
    blendRadiusM: 0,
    wireFeedMPerMin: 0,
    voltageV: 0,
    amperageA: 0,
    weldProfileId: 0,
    weaveProfileId: 0,
    revPiIOSlot: 5,
    torchWorkAngleDeg: 45,
    torchTravelAngleDeg: 10,
    torchSpinAngleDeg: 0,
    torchStandoffMm: DEFAULT_TORCH_STANDOFF_MM,
    weavePattern: "none",
    weaveAmplitudeMm: 2,
    weaveFrequencyHz: 1,
    weaveDwellMs: 0,
  };
  return {
    schema: PROJECT_SCHEMA,
    identity: {
      id,
      name: input.name ?? "Untitled offline program",
      description: "",
      createdAt: timestamp,
      updatedAt: timestamp,
      revision: 1,
    },
    cad: {
      kind: input.cad?.kind ?? "example",
      filename: input.cad?.filename ?? "dog_box.step",
      fingerprint: input.cad?.fingerprint ?? "example:dog-box-v1",
      sourceUnit: input.cad?.sourceUnit ?? "mm",
      topologyModelId: input.cad?.topologyModelId ?? "dog-box-offline-fixture",
    },
    robot: {
      model: model,
      jointOrder: JOINT_ORDER,
      // Which version of it, so a program authored today can tell when the
      // robot moves under it.
      description: robotDescriptionPinFor(model, timestamp),
    },
    target: { mode: "local_simulator", profileId: "local-simulator", manifestRef: null },
    frames: [
      { id: "world", parentId: null, translationM: [0, 0, 0], rotationRpyRad: [0, 0, 0] },
      // The tool frame every row references. In the canonical URDF this is
      // the torch assembly link, and on rosie_1400_v3 `tool0` is its child
      // with a zero transform, so end_effector == tool0 == the torch frame.
      // The transform here is a name registration, not a kinematic claim —
      // nothing in this app resolves toolFrameId against this table; the
      // Motion Server resolves frames in its own URDF.
      { id: "end_effector", parentId: "world", translationM: [0, 0, 0], rotationRpyRad: [0, 0, 0] },
      // Legacy alias retained for programs authored before 2026-08-04.
      { id: "torch_tcp", parentId: "world", translationM: [0, 0, 0], rotationRpyRad: [0, 0, 0] },
      // The cell's own work frame -- ON A CELL THAT HAS ONE. `rosie_1420_v1`
      // fixtures to the floor, so its work frame IS `world`, already the first
      // entry above; pushing a second `world` whose parent is `world` would be
      // a cycle, and `resolveFrameTransform` would answer with the identity
      // rather than say so.
      ...(work.id === "world" ? [] : [{
        id: work.id,
        parentId: "world",
        translationM: [...work.worldM] as Vec3,
        rotationRpyRad: [0, 0, 0] as Rpy,
      }]),
    ],
    fixtures: [{
      id: "dog-box-fixture",
      name: "Dog Box",
      frameId: work.id,
      sourceCadFingerprint: input.cad?.fingerprint ?? "example:dog-box-v1",
      translationM: [0, 0, 0],
      rotationRpyRad: [0, 0, 0],
    }],
    workcellFixtures: defaultWorkcellFixtures(model),
    edgeCatalog: [],
    seams: [],
    railsAuthoring: false,
    processParameters: [process],
    sequence: [],
    program: {
      schema: "robot.v4.program.v1",
      programId: id,
      programRevision: 1,
      baseProgramRevision: 0,
      programDigest: null,
      catalogRevision: null,
      partName: input.cad?.filename ?? "dog_box.step",
      partNumber: "",
      partRevision: "A",
      roboticCellType: model,
      cellConfigRevision: "",
      programState: "draft",
      executionMode: "simulation",
      editSource: "offline_programming",
      sourceProjectRevision: 1,
      defaults: {
        default_tool_frame: "end_effector",
        default_work_frame: "world",
        default_move_type: "line_tcp",
        default_tcp_velocity_m_s: 0.05,
        default_joint_velocity_rad_s: 0.25,
        default_accel_m_s2: 0.75,
        default_joint_accel_rad_s2: 1,
        default_blend_radius_m: 0,
      },
      settings: {
        singularity_protection_enabled: true,
        singularity_behavior: "avoid",
        singularity_warning_threshold: 0.18,
        singularity_hard_stop_threshold: 0.08,
        singularity_speed_limit_rad_s: 0.25,
        revision: 1,
      },
      constraints: {
        tip_link: "tool0",
        tool_axis_constraint: {
          tool_frame: "end_effector",
          tip_link: "tool0",
          world_frame: "world",
          tool_axis_xyz: [1, 0, 0],
          world_axis_xyz: [0, 0, -1],
          offset_reference_axis_xyz: [1, 0, 0],
          offset_deg: 0,
          // Work/travel are each clamped to +/-89 degrees. Their composed
          // world-down dot is cos(work)*cos(travel) > 0, and spin preserves Z,
          // so every authored torch remains inside the downward hemisphere.
          // +/-90 admits that complete authored domain without fighting the
          // per-row full RPY while still rejecting an upward-pointing tool.
          tolerance_minus_deg: 90,
          tolerance_plus_deg: 90,
          weight: 0.5,
        },
      },
      extensions: {},
    },
    nodes: [],
    programV2: defaultProgramV2Header(),
    rows: [],
    plan: missingPlan(),
    denseTrajectory: null,
    planHistory: [],
    simulation: {
      cursorS: 0,
      playbackRate: 1,
      loop: false,
      selectedRowId: null,
      camera: "orbit",
    },
  };
}

export function cloneProject(project: OfflineProgrammingProject): OfflineProgrammingProject {
  return structuredClone(project);
}

export function reviseProject(
  current: OfflineProgrammingProject,
  change: ProjectChange,
  mutate: (draft: OfflineProgrammingProject) => void,
  timestamp = nowISO(),
): OfflineProgrammingProject {
  const next = cloneProject(current);
  mutate(next);
  next.identity.revision = current.identity.revision + 1;
  next.identity.updatedAt = timestamp;
  if (affectsPlan(change)) {
    next.plan = invalidatePlan(current.plan, change);
    next.denseTrajectory = null;
    next.program.programRevision = current.program.programRevision + 1;
    next.program.programDigest = null;
    next.program.programState = "needs_plan";
    next.program.editSource = "offline_programming";
    next.program.sourceProjectRevision = next.identity.revision;
    const priorRows = new Map(current.rows.map((row) => [
      row.id,
      JSON.stringify(editableRowRevisionValue(row)),
    ]));
    for (const row of next.rows) {
      if (priorRows.get(row.id) === JSON.stringify(editableRowRevisionValue(row))) continue;
      row.rowRevision = next.program.programRevision;
      row.rowHash = `pending:${row.id}:${next.program.programRevision}`;
      row.plannerStatus = "stale";
      row.validationState = "unknown";
      row.collisionCheckState = "not_run";
      row.limitCheckState = "not_run";
      row.plannedPathRevision = 0;
      row.plannedSampleCount = 0;
    }
  } else if (
    current.plan.sourceProjectRevision === current.identity.revision &&
    current.plan.state !== "stale"
  ) {
    // Metadata/simulation do not alter plan inputs. Carry the exact project
    // association forward instead of making an unchanged plan appear stale.
    next.plan.sourceProjectRevision = next.identity.revision;
  }
  assertProject(next);
  return next;
}

export function restoreAsRevision(
  current: OfflineProgrammingProject,
  snapshot: OfflineProgrammingProject,
  reason: "undo" | "redo",
  timestamp = nowISO(),
): OfflineProgrammingProject {
  const restored = cloneProject(snapshot);
  restored.identity.id = current.identity.id;
  restored.identity.createdAt = current.identity.createdAt;
  restored.identity.revision = current.identity.revision + 1;
  restored.identity.updatedAt = timestamp;
  restored.plan = invalidatePlan(restored.plan, reason);
  restored.denseTrajectory = null;
  restored.program.programId = current.program.programId;
  restored.program.baseProgramRevision = current.program.baseProgramRevision;
  restored.program.programRevision = current.program.programRevision + 1;
  restored.program.programDigest = null;
  restored.program.programState = "needs_plan";
  restored.program.editSource = "offline_programming";
  restored.program.sourceProjectRevision = restored.identity.revision;
  assertProject(restored);
  return restored;
}

export function invalidatePlan(plan: PlanRecord, reason: string): PlanRecord {
  if (plan.state === "missing" && plan.trajectory.length === 0) {
    return missingPlan();
  }
  return {
    ...cloneValue(plan),
    state: "stale",
    staleReason: `project changed: ${reason}`,
    evidence: {
      ...cloneValue(plan.evidence),
      runtimePlayable: false,
    },
  };
}

export function installPreviewPlan(
  project: OfflineProgrammingProject,
  trajectory: PlanTrajectorySample[],
  timestamp = nowISO(),
): OfflineProgrammingProject {
  const next = cloneProject(project);
  next.plan = {
    state: "preview",
    id: `preview:${project.identity.id}:${project.identity.revision}`,
    revision: Math.max(1, project.plan.revision + 1),
    sourceProjectRevision: project.identity.revision,
    sourceRobotModel: project.robot.model,
    sourceCell: cellSettingsKey(project.robot.cell),
    planner: "browser_preview",
    createdAt: timestamp,
    staleReason: null,
    evidence: emptyEvidence(),
    trajectory: cloneValue(trajectory),
    acceptedPlan: null,
  };
  return next;
}

export function installServerValidatedPlan(
  project: OfflineProgrammingProject,
  input: Omit<
    PlanRecord,
    "state" | "sourceProjectRevision" | "sourceRobotModel" | "sourceCell" | "planner" |
      "staleReason" | "acceptedPlan"
  > & { acceptedPlan?: RobotV4AcceptedPlan | null },
): OfflineProgrammingProject {
  const next = cloneProject(project);
  next.plan = {
    ...cloneValue(input),
    state: "server_validated",
    sourceProjectRevision: project.identity.revision,
    sourceRobotModel: project.robot.model,
    sourceCell: cellSettingsKey(project.robot.cell),
    planner: "motion_server_tesseract",
    staleReason: null,
    acceptedPlan: cloneValue(input.acceptedPlan ?? null),
  };
  return next;
}

export function planIsFresh(project: OfflineProgrammingProject): boolean {
  return project.plan.sourceProjectRevision === project.identity.revision &&
    project.plan.sourceRobotModel === project.robot.model &&
    (project.plan.sourceCell ?? null) === cellSettingsKey(project.robot.cell) &&
    project.plan.state !== "stale";
}

export function executionReadiness(project: OfflineProgrammingProject): ExecutionReadiness {
  const reasons: string[] = [];
  if (project.target.mode !== "real_connected") reasons.push("real connected target not selected");
  if (project.plan.state !== "server_validated") reasons.push("server-validated plan required");
  if (!planIsFresh(project)) reasons.push("plan revision is stale");
  if (project.plan.evidence.collision !== "clear") reasons.push("collision evidence is not clear");
  if (project.plan.evidence.singularity !== "clear") reasons.push("singularity evidence is not clear");
  if (project.plan.evidence.limits !== "clear") reasons.push("joint-limit evidence is not clear");
  if (project.plan.evidence.startPose !== "converged") reasons.push("start pose is not converged");
  if (project.plan.evidence.runtimePlayable !== true) reasons.push("runtime playable evidence is missing");
  if (project.plan.acceptedPlan?.authority_classification !== "physical_target") {
    reasons.push("physical-target accepted plan is missing");
  }
  if (!project.target.profileId || !project.target.manifestRef) reasons.push("manifest-backed target identity is missing");
  return { ready: reasons.length === 0, reasons };
}

export function affectsPlan(change: ProjectChange): change is AuthoringChange {
  return change !== "metadata" && change !== "simulation";
}

export function assertProject(value: unknown): asserts value is OfflineProgrammingProject {
  if (!isRecord(value) || value.schema !== PROJECT_SCHEMA) {
    throw new Error(`project schema must be ${PROJECT_SCHEMA}`);
  }
  if (!isRecord(value.identity) || typeof value.identity.id !== "string" || !value.identity.id.trim()) {
    throw new Error("project identity.id is required");
  }
  if (!Number.isInteger(value.identity.revision) || Number(value.identity.revision) < 1) {
    throw new Error("project identity.revision must be a positive integer");
  }
  // A NAME, not a list. Which robots exist is the server's answer
  // (`domain/robots.ts`); a stored project that names one this server does not
  // have still loads, and the Robot card says so -- refusing to open it here
  // would strand the program on the browser that could not describe its robot.
  if (!isRecord(value.robot) || !/^[A-Za-z0-9][A-Za-z0-9_-]*$/.test(String(value.robot.model))) {
    throw new Error("project robot.model is not a robot description name");
  }
  const arrays = ["frames", "fixtures", "seams", "processParameters", "sequence", "rows"] as const;
  for (const key of arrays) {
    if (!Array.isArray(value[key])) throw new Error(`project ${key} must be an array`);
  }
  // Absent on projects saved before P2.12; serialization hydrates it to [].
  if (value.edgeCatalog !== undefined && !Array.isArray(value.edgeCatalog)) {
    throw new Error("project edgeCatalog must be an array when present");
  }
  // Absent on projects saved before nodes; serialization hydrates both.
  if (value.nodes !== undefined && !Array.isArray(value.nodes)) {
    throw new Error("project nodes must be an array when present");
  }
  if (value.programV2 !== undefined && !isRecord(value.programV2)) {
    throw new Error("project programV2 must be an object when present");
  }
  if (value.planHistory !== undefined && !Array.isArray(value.planHistory)) {
    throw new Error("project planHistory must be an array when present");
  }
  if (value.workpieces !== undefined) {
    if (!Array.isArray(value.workpieces)) throw new Error("project workpieces must be an array");
    const ids = new Set<string>();
    for (const part of value.workpieces) {
      if (!isRecord(part) || typeof part.id !== "string" || !part.id || ids.has(part.id) ||
          !isRecord(part.cad) || typeof part.cad.filename !== "string" || typeof part.cad.fingerprint !== "string") {
        throw new Error("every weld part must have a unique id and CAD source");
      }
      ids.add(part.id);
      for (const field of ["fixtures", "edgeCatalog", "seams", "nodes", "sequence", "rows"]) {
        if (!Array.isArray(part[field])) throw new Error(`weld part ${part.id}.${field} must be an array`);
      }
    }
  }
  if (!isRecord(value.plan) || !isRecord(value.simulation) || !isRecord(value.target) ||
      !isRecord(value.program)) {
    throw new Error("project program, plan, simulation, and target are required");
  }
  if (value.program.schema !== "robot.v4.program.v1" ||
      typeof value.program.programId !== "string" || !value.program.programId.trim() ||
      !Number.isInteger(value.program.programRevision) || Number(value.program.programRevision) < 1 ||
      !Number.isInteger(value.program.baseProgramRevision) || Number(value.program.baseProgramRevision) < 0) {
    throw new Error("project Robot V4 program link is invalid");
  }
  const targetMode = String(value.target.mode);
  if (!["offline_preview", "local_simulator", "real_connected"].includes(targetMode)) {
    throw new Error(`unsupported target mode ${targetMode}`);
  }
  const rows = value.rows as unknown[];
  for (const row of rows) {
    if (!isRecord(row) || !Array.isArray(row.targetJointsRad) || row.targetJointsRad.length !== JOINT_ORDER.length) {
      throw new Error("every authored row must carry J1-J9 in canonical order");
    }
  }
  const seams = value.seams as unknown[];
  for (const seam of seams) {
    if (!isRecord(seam) || !isRecord(seam.trim) ||
        !Number.isFinite(Number(seam.trim.startOffsetM)) ||
        !Number.isFinite(Number(seam.trim.endOffsetM)) ||
        Number(seam.trim.startOffsetM) < 0 ||
        Number(seam.trim.endOffsetM) < 0) {
      throw new Error("every CAD weld path trim must use non-negative metre offsets");
    }
  }
}

export function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function finite(value: unknown, fallback: number): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

export function cloneValue<T>(value: T): T {
  return structuredClone(value);
}

function editableRowRevisionValue(row: AuthoredProgramRow): Record<string, unknown> {
  const value = structuredClone(row) as unknown as Record<string, unknown>;
  for (const key of [
    "rowRevision",
    "rowHash",
    "plannerStatus",
    "validationState",
    "collisionCheckState",
    "limitCheckState",
    "plannedPathRevision",
    "plannedSampleCount",
    "currentTcpPose",
    "currentJointsRad",
    "revPiIOPending",
  ]) delete value[key];
  return value;
}
