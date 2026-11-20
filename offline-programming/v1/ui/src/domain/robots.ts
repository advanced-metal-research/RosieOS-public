/**
 * Which robots there are, read from the server once per session.
 *
 * This browser used to know: `RobotModel` was the union
 * `"rosie_1400_v3" | "rosie_1420_v1"`, the work frame of each was a constant
 * beside it, and adding a robot meant editing five TypeScript files after the
 * description directory already existed. The descriptions live in
 * `robot_description/<model>/` and the server is the one thing that reads
 * them (`docs/ROBOT-DESCRIPTION-CONTRACT.md`), so it is the one thing that can
 * say what exists; `GET /api/offline-programming/v1/robots` is that answer and
 * this module is where the session keeps it.
 *
 * It is a module-level store on purpose. `createProject` and the placement
 * maths are synchronous and are called from everywhere, and threading a
 * catalogue through them would put an await in front of arithmetic. `main.ts`
 * fills it once, before the first project is loaded.
 */

import type { Vec3 } from "./project.ts";

/** Where a workpiece is fixtured on one cell, and what that frame is called. */
export interface CellWorkFrame {
  /** The project frame the fixture hangs off. */
  id: string;
  /** That frame in the viewer's world, at the robot's zero pose. */
  worldM: Vec3;
}

/** The named frames a description declares, as the planner reads them. */
export interface RobotFrames {
  base: string;
  tool: string;
  work: string;
  /** The work frame in the robot's base frame, at the zero pose. */
  workWorldM: Vec3;
  /** Where the arm starts: the first driven joint's child link, at the zero
   *  pose. Motion Server's TCP contract is expressed in that frame and the
   *  viewer in the world, so rows convert between the two. */
  armBaseWorldM: Vec3;
}

/** One robot the server holds a description for. */
export interface RobotDescriptorSummary {
  modelId: string;
  /** The package the URDF names its meshes under, when it differs from the
   *  model id the viewer needs it (and the family) to draw the robot. */
  urdfPackage?: string;
  /** The description's identity, `sha256:<hex>`; "" when the server omits it. */
  robotDescriptionSha256: string;
  frames: RobotFrames;
  /** The axes the weld planner searches, in order. */
  driven: readonly string[];
  /** The axes it holds, and at what value. Held is about PLANNING: the cell
   *  still maps, homes and jogs them. */
  held: Readonly<Record<string, number>>;
  resetPose: Readonly<Record<string, number>>;
  /** Optional description-owned settings for reviewing a changed identity. */
  settings?: RobotSettings;
}

export interface RobotSettings {
  revision?: number;
  joints: { name: string; type: string; lower?: number; upper?: number; rated_velocity?: number; gear_ratio?: number }[];
  planning?: { speed_ceiling_rad_s?: number; joints: Record<string, { velocity_rad_s?: number; acceleration_rad_s2?: number }> };
  torch?: { tool_frame: string; electrode_axis?: Vec3 };
  resources?: Record<string, string>;
}

export interface RobotSettingsSnapshot {
  modelId: string;
  robotDescriptionSha256: string;
  settings: RobotSettings;
  frames: RobotFrames;
}

/** Never substitute the local revision for an unavailable connected revision. */
export function robotSettingsSnapshot(modelId: string, identity: string): RobotSettingsSnapshot | undefined {
  const entry = catalogue.find(item => item.modelId === modelId && item.robotDescriptionSha256 === identity);
  if (!entry?.settings) return undefined;
  return structuredClone({ modelId, robotDescriptionSha256: identity, settings: entry.settings, frames: entry.frames });
}

/**
 * The frame an unknown robot's parts hang off: the robot's own base, no
 * offset. Deliberately not another robot's table -- the bug this whole
 * contract exists to end is a part placed 1.913 m from where the planner read
 * it, which is what defaulting to the 1400's positioner did. A project naming
 * a robot the server does not have is SAID OUT LOUD by the caller
 * (`knownRobotModel`); this only keeps the arithmetic defined until it is.
 */
const BASE_ORIGIN: CellWorkFrame = { id: "world", worldM: [0, 0, 0] };

let catalogue: readonly RobotDescriptorSummary[] = [];

/** The description the connected cell advertises: which model, and which
 *  identity of it. While a cell is connected the plan is made against ITS
 *  description (docs/PLANNING-CONTEXT.md), so this is what "the current
 *  description" means for drift and for adopting; the workstation's own copy
 *  is what it means otherwise. */
export interface ConnectedRobot {
  modelId: string;
  robotDescriptionSha256: string;
}

let connected: ConnectedRobot | null = null;

/** What the server answered. Call once, before the first project is read. */
export function setRobotCatalogue(entries: readonly RobotDescriptorSummary[]): void {
  catalogue = [...entries];
}

export function robotCatalogue(): readonly RobotDescriptorSummary[] {
  return catalogue;
}

/** The connected cell's description, or null when no cell says. */
export function setConnectedRobot(robot: ConnectedRobot | null): void {
  connected = robot && robot.modelId && robot.robotDescriptionSha256 ? { ...robot } : null;
}

export function connectedRobot(): ConnectedRobot | null {
  return connected;
}

/** The entry for `modelId`: the connected cell's revision of it when the
 *  catalogue lists that one (a fetched description is listed beside the
 *  workstation's), else the workstation's. */
export function robotDescriptor(modelId: string): RobotDescriptorSummary | null {
  const matching = catalogue.filter((entry) => entry.modelId === modelId);
  if (connected && connected.modelId === modelId) {
    const theirs = matching.find((entry) => entry.robotDescriptionSha256 === connected!.robotDescriptionSha256);
    if (theirs) return theirs;
  }
  return matching[0] ?? null;
}

/** The identity of the description a plan for `modelId` would be made
 *  against right now: the connected cell's when one is connected and is
 *  this model, else the workstation's; "" when neither has one. */
export function currentDescriptionSha256(modelId: string): string {
  if (connected && connected.modelId === modelId) return connected.robotDescriptionSha256;
  return robotDescriptor(modelId)?.robotDescriptionSha256 ?? "";
}

/** Does the server hold a description for this model? */
export function knownRobotModel(modelId: string): boolean {
  return robotDescriptor(modelId) !== null;
}

/** What a new program is for when nobody said: the first robot the server
 *  listed. Empty when the catalogue could not be read, which the caller shows
 *  rather than papering over. */
export function defaultRobotModel(): string {
  return catalogue[0]?.modelId ?? "";
}

/** The work frame of the cell a project is authored against. */
export function cellWorkFrame(modelId: string): CellWorkFrame {
  const entry = robotDescriptor(modelId);
  if (!entry) return BASE_ORIGIN;
  return { id: entry.frames.work, worldM: [...entry.frames.workWorldM] };
}

/** The route's payload, as this session uses it. Anything malformed is left
 *  out rather than guessed at: a robot the browser cannot describe is a robot
 *  it must not offer. */
export function parseRobotCatalogue(payload: unknown): RobotDescriptorSummary[] {
  const robots = isRecord(payload) ? payload.robots : payload;
  if (!Array.isArray(robots)) return [];
  const out: RobotDescriptorSummary[] = [];
  for (const entry of robots) {
    const parsed = parseRobot(entry);
    if (parsed) out.push(parsed);
  }
  return out;
}

function parseRobot(value: unknown): RobotDescriptorSummary | null {
  if (!isRecord(value)) return null;
  const modelId = text(value.model_id);
  const frames = isRecord(value.frames) ? value.frames : null;
  const work = text(frames?.work);
  const worldM = vec3(frames?.work_world_m);
  if (!modelId || !work || !worldM) return null;
  const axes = isRecord(value.axes) ? value.axes : {};
  const settings = parseRobotSettings(value.settings);
  const urdfPackage = text(value.urdf_package);
  return {
    modelId,
    ...(urdfPackage ? { urdfPackage } : {}),
    robotDescriptionSha256: text(value.robot_description_sha256),
    frames: {
      base: text(frames?.base) || "world",
      tool: text(frames?.tool) || "tool0",
      work,
      workWorldM: worldM,
      armBaseWorldM: vec3(frames?.arm_base_world_m) ?? [0, 0, 0],
    },
    driven: Array.isArray(axes.driven) ? axes.driven.map((axis) => String(axis)) : [],
    held: numbers(axes.held),
    resetPose: numbers(value.reset_pose),
    ...(settings ? { settings } : {}),
  };
}

/** Missing/malformed snapshots do not make an otherwise valid old catalogue unusable. */
export function parseRobotSettings(value: unknown): RobotSettings | undefined {
  if (!isRecord(value) || !Array.isArray(value.joints)) return undefined;
  const finite = (raw: unknown): raw is number => typeof raw === "number" && Number.isFinite(raw);
  const fields = (raw: Record<string, unknown>, keys: readonly string[]) => Object.fromEntries(
    keys.filter(key => finite(raw[key])).map(key => [key, raw[key] as number]),
  );
  const settings: RobotSettings = {
    ...fields(value, ["revision"]),
    joints: value.joints.flatMap(raw => isRecord(raw) && text(raw.name) && text(raw.type)
      ? [{ name: text(raw.name), type: text(raw.type), ...fields(raw, ["lower", "upper", "rated_velocity", "gear_ratio"]) }] : []),
  };
  if (isRecord(value.planning)) {
    const planning = value.planning;
    settings.planning = { ...fields(planning, ["speed_ceiling_rad_s"]), joints: {} };
    if (isRecord(planning.joints)) for (const [name, joint] of Object.entries(planning.joints)) {
      if (isRecord(joint)) settings.planning.joints[name] = fields(joint, ["velocity_rad_s", "acceleration_rad_s2"]);
    }
  }
  if (isRecord(value.torch)) {
    const axis = Array.isArray(value.torch.electrode_axis) && value.torch.electrode_axis.every(finite)
      ? vec3(value.torch.electrode_axis) : null;
    settings.torch = { tool_frame: text(value.torch.tool_frame), ...(axis ? { electrode_axis: axis } : {}) };
  }
  if (isRecord(value.resources)) settings.resources = Object.fromEntries(Object.entries(value.resources)
    .filter((entry): entry is [string, string] => typeof entry[1] === "string" && /^sha256:[a-f0-9]{64}$/i.test(entry[1])));
  return settings;
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function text(value: unknown): string {
  return typeof value === "string" ? value.trim() : "";
}

function vec3(value: unknown): Vec3 | null {
  if (!Array.isArray(value) || value.length !== 3) return null;
  const out = value.map((component) => Number(component));
  return out.every((component) => Number.isFinite(component))
    ? [out[0]!, out[1]!, out[2]!]
    : null;
}

function numbers(value: unknown): Record<string, number> {
  if (!isRecord(value)) return {};
  const out: Record<string, number> = {};
  for (const [key, raw] of Object.entries(value)) {
    const parsed = Number(raw);
    if (Number.isFinite(parsed)) out[key] = parsed;
  }
  return out;
}

/** Where `model`'s arm starts, in the world the viewer draws. The origin for
 *  an unknown robot: a row converted against another robot's offset would be
 *  metres out and look plausible. */
export function armBaseWorldM(model: string): Vec3 {
  const entry = robotDescriptor(model);
  return entry ? [...entry.frames.armBaseWorldM] : [0, 0, 0];
}
