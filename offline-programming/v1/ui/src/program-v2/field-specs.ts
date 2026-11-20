/**
 * Which fields a node has, where they live, and how an edit lands.
 *
 * DOM-free and node-importable on purpose. The dialog renders these rows
 * through `field-form.ts`; `field-specs.test.ts` walks every path against
 * `robot.v4.program.v2.schema.json` and fails if one points at a field the
 * schema does not have -- a dead row in a dialog is otherwise invisible
 * until someone types into it and nothing happens.
 *
 * `applyEdit` is the one place an edit is written, and it knows the shapes
 * the schema makes awkward for a flat form: a standoff is `{direction,
 * distance_mm}` or absent, never half; an override key cleared means
 * "inherit"; a scalar field is stored as `{kind: "constant", value}`; a
 * points edit on a CAD-derived classic weld marks it teach-modified so a
 * placement change no longer overwrites it.
 */

import {
  type FreespacePolicy,
  type PosedPoint,
  type ProgramV2Node,
  type ScalarFunction,
  type WeldNode,
  assertRobotV4ProgramV2,
} from "../contracts/robot-v4-program-v2.ts";
import { CLASSIC_EXTENSION, classicProvenance, setClassicProvenance } from "./from-project.ts";
import { isFreespace, isWeld } from "./nodes.ts";

export type FieldKind = "number" | "text" | "bool" | "enum" | "profile" | "points" | "note";

export interface FieldSpec {
  /** Dotted, rooted at the node. Paths under `_seam.` are virtual: the
   *  editor reads and writes them through the project's classic seam. */
  path: string;
  label: string;
  kind: FieldKind;
  unit?: string;
  displayScale?: number;
  step?: number;
  min?: number;
  max?: number;
  /** Stored as `{kind: "constant", value}`; a non-constant profile is
   *  shown as `varies` and left alone. */
  scalar?: boolean;
  options?: readonly string[];
  /** Clearing the box removes the key: an override reverts to the default,
   *  a standoff goes back to the planner. */
  nullable?: boolean;
  /** Why the field cannot be typed into, when it cannot. */
  readOnly?: string;
  hint?: string;
  /** The `note` kind's text. */
  text?: string;
}

export interface FieldGroup {
  id: string;
  group: string;
  note?: string;
  fields: FieldSpec[];
}

export interface SpecContext {
  defaults: FreespacePolicy;
  /** The node is a classic weld whose seam is still in the project. */
  classicSeam: boolean;
}

export const POLICY_KEYS = ["clearance_min_mm", "speed_scale", "torch_policy", "positioner_policy"] as const;
export const STANDOFF_DIRECTIONS = ["workpiece_z", "torch_axis"] as const;
export const WEAVE_SHAPES = ["none", "sine", "zigzag", "trapezoidal", "circular", "crescent", "triangular"] as const;

const identity = (node: ProgramV2Node): FieldGroup => ({
  id: "identity",
  group: "identity",
  fields: [
    { path: "label", label: "label", kind: "text" },
    { path: "comment", label: "comment", kind: "text", hint: "free text; a v1 comment row becomes this" },
    { path: "enabled", label: "enabled", kind: "bool", hint: `a disabled ${node.op} stays in the program and is skipped` },
  ],
});

const process = (): FieldGroup => ({
  id: "process",
  group: "process",
  note: "Travel speed is along the weld's arc length, never tip speed; the planner times the weld from it. Weave reserves transverse mobility.",
  fields: [
    { path: "travel_speed_mm_s", label: "travel speed", kind: "number", unit: "mm/s", step: 0.5, min: 0, scalar: true },
    { path: "weave.shape", label: "weave shape", kind: "enum", options: WEAVE_SHAPES },
    { path: "weave.amplitude_mm", label: "weave amplitude", kind: "number", unit: "mm", step: 0.1, min: 0, scalar: true, nullable: true },
    { path: "weave.frequency_hz", label: "weave frequency", kind: "number", unit: "Hz", step: 0.1, min: 0, scalar: true, nullable: true },
  ],
});

const weldParameters = (): FieldGroup => ({
  id: "weld_parameters",
  group: "weld parameters",
  fields: [
    {
      path: "weld_parameters",
      label: "reserved",
      kind: "note",
      text: "Arc voltage, current, wire feed, gas, ignition and crater fill will live here once designed. Until then the block must be empty; the power source keeps its own settings.",
    },
  ],
});

function standoffFields(side: "arrive" | "depart"): FieldSpec[] {
  const what = side === "arrive" ? "arrive at the weld after this move" : "leave the weld before this move";
  return [
    {
      path: `${side}_standoff.direction`,
      label: `${side} direction`,
      kind: "enum",
      options: STANDOFF_DIRECTIONS,
      nullable: true,
      hint: `how to ${what}: straight along the workpiece +Z, or back along the electrode`,
    },
    {
      path: `${side}_standoff.distance_mm`,
      label: `${side} distance`,
      kind: "number",
      unit: "mm",
      step: 1,
      min: 0,
      nullable: true,
      hint: "clear both boxes to leave the whole move to the planner",
    },
  ];
}

export function groupsForNode(node: ProgramV2Node, context: SpecContext): FieldGroup[] {
  const groups: FieldGroup[] = [identity(node)];
  if (node.op === "move") {
    groups.push({ id: "weld", group: "Weld", note: "Starts/stops after arrival at this waypoint. Set process parameters on the welder. Following weld moves use their programmed TCP speed; ramps and limits still apply. The cell must have a qualified assigned weld output before a production run.", fields: [
      { path: "weld_transition", label: "Weld action", kind: "enum", options: ["none","start","stop"] },
      ...(node.weld_transition === "start" ? [{ path: "weld_start_delay_s", label: "Start delay · ON before moving", kind: "number" as const, unit: "s", min: 0, max: 30, step: .1 }] : []),
      ...(node.weld_transition === "stop" ? [{ path: "weld_stop_delay_s", label: "Stop delay · ON after arrival", kind: "number" as const, unit: "s", min: 0, max: 30, step: .1 }] : []),
    ] });
    groups.push({ id: "motion", group: "Motion profile", note: "TCP speed is in mm/s. Joint speed and acceleration limits are below. Save & plan updates the graph; plan speed scales the resulting motion.", fields: [
      { path: "motion", label: "Move type", kind: "enum", options: ["joint", "linear", "circular"] },
      { path: "speed_mm_s", label: "TCP speed limit", kind: "number", unit: "mm/s", min: 0.01, step: 1 },
      ...(node.motion === "joint" ? [] : [{ path: "constant_tcp_speed", label: "Constant TCP speed", kind: "bool" as const,
        hint: "Cruise at the requested TCP speed between acceleration/deceleration ramps. Short moves and joint limits can prevent reaching that speed. Disable for smooth speed variation over the whole move." }]),
    ] });
    groups.push({ id: "joint-limits", group: "Joint limits", fields: [
      { path: "speed_scale", label: "Joint speed limit", kind: "number", unit: "%", displayScale: 100, min: 1, max: 100, step: 1 },
      { path: "acceleration_scale", label: "Joint acceleration limit", kind: "number", unit: "%", displayScale: 100, min: 1, max: 100, step: 1 },
    ] });
    for (const key of ["target", ...(node.via ? ["via"] : [])] as ("target" | "via")[]) {
      const pose = node[key]!;
      groups.push({ id: key, group: key === "target" ? "Destination" : "Circular via pose", note: "Editing XYZ/RPY defines a world TCP destination; recorded joints seed inverse kinematics. Editing joint angles selects a joint destination.", fields: [
        ...["x", "y", "z"].map((label, i): FieldSpec => ({ path: `${key}.xyz_m.${i}`, label, kind: "number", unit: "mm", displayScale: 1000, step: 1 })),
        ...["roll", "pitch", "yaw"].map((label, i): FieldSpec => ({ path: `${key}.rpy_rad.${i}`, label, kind: "number", unit: "°", displayScale: 180 / Math.PI, step: 0.1 })),
        ...pose.joint_names.map((label, i): FieldSpec => ({ path: `${key}.joint_values_rad.${i}`, label, kind: "number", unit: "°", displayScale: 180 / Math.PI, step: 0.1, ...(node.motion !== "joint" ? { readOnly: "recorded IK seed" } : {}) })),
      ] });
    }
    groups.push({ id: "capture", group: "Original observation", note: "Editing the destination preserves this original observation. Reteach replaces it with a fresh capture.", fields: [
      { path: "capture.source", label: "Source", kind: "text", readOnly: "capture provenance" },
      { path: "capture.observed_at", label: "Observed", kind: "text", readOnly: "capture provenance" },
    ] });
    return groups;
  }
  if (isWeld(node)) {
    if (node.geometry.kind === "seam") {
      groups.push({
        id: "geometry",
        group: "seam geometry",
        note: "The weld planner's seam: an analytic curve, welded over a span of its normalised arc length, with the torch angles as profiles the planner samples and may trade inside their band.",
        fields: [
          { path: "geometry.id", label: "seam", kind: "text", readOnly: "the seam id names the weld planner's seam" },
          { path: "geometry.weld_joint_ref", label: "joint", kind: "text", readOnly: "detected from the CAD" },
          { path: "geometry.span.start_s", label: "span start", kind: "number", unit: "s", step: 0.01, min: 0, max: 1, nullable: true, hint: "0 is the curve's start; start > end wraps through s=0 on a closed curve" },
          { path: "geometry.span.end_s", label: "span end", kind: "number", unit: "s", step: 0.01, min: 0, max: 1, nullable: true },
          { path: "geometry.direction", label: "direction", kind: "enum", options: ["forward", "reverse"], nullable: true },
          { path: "geometry.orientation.work_angle_deg", label: "work angle", kind: "profile", unit: "°" },
          { path: "geometry.orientation.travel_angle_deg", label: "travel angle", kind: "profile", unit: "°" },
        ],
      });
    } else if (node.geometry.kind === "manual_pose_wp") {
      groups.push({
        id: "geometry",
        group: "hand-placed poses",
        note: "Torch poses placed by hand in the Waypoint weld tool, and how the path passes through each. They are edited there, where the handle and the standoffs are; the weld planner receives them lowered to a seam.",
        fields: [
          { path: "geometry.frame_id", label: "frame", kind: "text", readOnly: "hand-placed poses are in the workpiece frame" },
          { path: "geometry.points", label: "poses", kind: "note", text: `${node.geometry.points.length} poses · edit them in the Waypoint weld tool` },
        ],
      });
    } else {
      if (context.classicSeam) {
        groups.push({
          id: "source",
          group: "source seam",
          note: "The CAD edge this weld was drawn from. Trim and reverse regenerate the points below unless a point has been edited by hand.",
          fields: [
            { path: "_seam.trimStartMm", label: "trim start", kind: "number", unit: "mm", step: 0.5, min: 0 },
            { path: "_seam.trimEndMm", label: "trim end", kind: "number", unit: "mm", step: 0.5, min: 0 },
            { path: "_seam.reversed", label: "reversed", kind: "bool" },
          ],
        });
      }
      groups.push({
        id: "geometry",
        group: "points",
        note: "Every control point carries the TCP orientation as it was baked, and that orientation is the authority: the planner tracks it and never re-derives it from the torch angles. Editing a point marks the weld hand-modified.",
        fields: [
          { path: "geometry.frame_id", label: "frame", kind: "text", readOnly: "the frame the points are expressed in" },
          { path: "geometry.closed", label: "closed", kind: "bool" },
          { path: "geometry.points", label: "control points", kind: "points", unit: "mm · °" },
          { path: "geometry.torch_angles_deg.work", label: "work angle (provenance)", kind: "number", unit: "°", readOnly: "what the points were built from; the points are the authority" },
          { path: "geometry.torch_angles_deg.travel", label: "travel angle (provenance)", kind: "number", unit: "°", readOnly: "what the points were built from; the points are the authority" },
          { path: "geometry.torch_angles_deg.spin", label: "spin (provenance)", kind: "number", unit: "°", readOnly: "what the points were built from; the points are the authority" },
        ],
      });
    }
    groups.push(process(), weldParameters());
    return groups;
  }
  if (isFreespace(node)) {
    const fields: FieldSpec[] = [];
    if (node.op === "retract" || node.op === "transit") fields.push(...standoffFields("depart"));
    if (node.op === "approach" || node.op === "transit") fields.push(...standoffFields("arrive"));
    if (node.op === "home") {
      fields.push({ path: "home", label: "home", kind: "note", text: "Goes to, or comes from, the cell's reset pose. The pose itself is the cell's, not the program's." });
    }
    groups.push({
      id: "standoff",
      group: "standoff",
      note: "A straight lift the torch makes leaving a weld or the last straight stretch into one. The planner solves everything between; this only says how the move begins and ends.",
      fields,
    });
    groups.push({
      id: "overrides",
      group: "overrides",
      note: "The program's freespace defaults, and this move's exceptions. A cleared box inherits the default shown beside it. Joint speed and acceleration limits are the cell's and are not here.",
      fields: [
        { path: "overrides.clearance_min_mm", label: "clearance floor", kind: "number", unit: "mm", step: 1, min: 0, nullable: true },
        { path: "overrides.speed_scale", label: "speed scale", kind: "number", unit: "×", step: 0.05, min: 0.05, max: 1, nullable: true },
        { path: "overrides.torch_policy", label: "torch policy", kind: "enum", options: ["free", "hold_last", "torch_down"], nullable: true },
        { path: "overrides.positioner_policy", label: "positioner", kind: "enum", options: ["free", "hold"], nullable: true },
      ],
    });
    return groups;
  }
  if (node.op === "dwell") {
    groups.push({
      id: "event",
      group: "dwell",
      fields: [{ path: "duration_s", label: "duration", kind: "number", unit: "s", step: 0.1, min: 0 }],
    });
    return groups;
  }
  groups.push({
    id: "event",
    group: "I/O action",
    note: "Runs at rest in program order. Wait reads an input; set and pulse command an output. Trace and dry run skip I/O.",
    fields: [
      { path: "action", label: "action", kind: "enum", options: ["set", "pulse", "wait", "monitor", "end_monitor"] },
      { path: "channel", label: ["wait", "monitor", "end_monitor"].includes(node.action ?? "set") ? "input" : "output", kind: "text", hint: "Configured channel name from I/O setup" },
      { path: "state", label: node.action === "wait" ? "wait for ON" : node.action === "monitor" ? "required ON" : "ON", kind: "bool" },
      ...(node.action === "pulse" ? [{ path: "duration_s", label: "pulse duration", kind: "number" as const, unit: "s", step: 0.1, min: 0.001, max: 3600 }] : []),
      ...(node.action === "wait" ? [
        { path: "timeout_s", label: "timeout", kind: "number" as const, unit: "s", step: 0.1, min: 0.001, max: 3600 },
        { path: "stable_s", label: "stable for", kind: "number" as const, unit: "s", step: 0.01, min: 0, max: 3600 },
      ] : []),
    ],
  });
  return groups;
}

// ------------------------------------------------------------------- paths

export const dig = (root: unknown, path: string): unknown =>
  path.split(".").reduce<unknown>((node, key) => {
    if (node && typeof node === "object") return (node as Record<string, unknown>)[key];
    return undefined;
  }, root);

export function setPath(root: Record<string, unknown>, path: string, value: unknown): void {
  const keys = path.split(".");
  let node = root;
  for (const key of keys.slice(0, -1)) {
    const next = node[key];
    if (!next || typeof next !== "object") node[key] = {};
    node = node[key] as Record<string, unknown>;
  }
  node[keys.at(-1)!] = value;
}

export function deletePath(root: Record<string, unknown>, path: string): void {
  const keys = path.split(".");
  let node: Record<string, unknown> | undefined = root;
  const chain: Record<string, unknown>[] = [];
  for (const key of keys.slice(0, -1)) {
    chain.push(node);
    const next: unknown = node[key];
    if (!next || typeof next !== "object" || Array.isArray(next)) return;
    node = next as Record<string, unknown>;
  }
  delete node[keys.at(-1)!];
  // Prune the containers the deletion emptied, innermost first.
  for (let index = chain.length - 1; index >= 0; index -= 1) {
    const parent = chain[index]!;
    const key = keys[index]!;
    const child = parent[key];
    if (child && typeof child === "object" && !Array.isArray(child) && Object.keys(child).length === 0) {
      delete parent[key];
    } else {
      break;
    }
  }
}

export const constantOf = (value: unknown): number | null => {
  if (typeof value === "number") return value;
  if (value && typeof value === "object" && (value as ScalarFunction).kind === "constant") {
    return (value as { value: number }).value;
  }
  return null;
};

export const constant = (value: number): ScalarFunction => ({ kind: "constant", value });

/** True when a scalar field holds a profile the form must not flatten. */
export const varies = (value: unknown): boolean =>
  Boolean(value && typeof value === "object" && (value as ScalarFunction).kind !== "constant");

// ------------------------------------------------------------------- edits

export type EditValue = number | string | boolean | PosedPoint[] | ScalarFunction | null;

/**
 * Write one edit into a node draft. Returns the draft for chaining. Throws
 * when the edit is impossible for the node (a standoff on a home, a points
 * edit on a seam weld), which the form turns into a message.
 */
export function applyEdit(draft: ProgramV2Node, spec: FieldSpec, value: EditValue): ProgramV2Node {
  const root = draft as unknown as Record<string, unknown>;
  const [head] = spec.path.split(".");
  if (head === "_seam") throw new Error("source seam fields are written through the project, not the node");
  if (value === null) {
    if (!spec.nullable) throw new Error(`${spec.label} cannot be empty`);
    deletePath(root, spec.path);
    if (head === "arrive_standoff" || head === "depart_standoff") delete root[head];
    return draft;
  }
  if (spec.kind === "points") {
    if (!isWeld(draft) || draft.geometry.kind !== "posed_polyline") throw new Error("only a posed weld has control points");
    if (!Array.isArray(value) || value.length < 2) throw new Error("a weld needs at least two control points");
    draft.geometry.points = structuredClone(value as PosedPoint[]);
    const provenance = classicProvenance(draft);
    if (provenance) {
      const { curve_centers_m: _dropped, ...rest } = provenance;
      setClassicProvenance(draft, { ...rest, cad_derived: false });
    }
    return draft;
  }
  if (spec.kind === "profile") {
    if (!value || typeof value !== "object" || Array.isArray(value)) throw new Error(`${spec.label} needs a profile`);
    setPath(root, spec.path, structuredClone(value));
    return draft;
  }
  if (head === "arrive_standoff" || head === "depart_standoff") {
    const current = (root[head] as Record<string, unknown> | undefined) ?? {};
    const standoff = { direction: "workpiece_z", distance_mm: 0, ...current };
    const key = spec.path.split(".")[1]!;
    (standoff as Record<string, unknown>)[key] = value;
    root[head] = standoff;
    return draft;
  }
  if (spec.kind === "number") {
    if (typeof value !== "number" || !Number.isFinite(value)) throw new Error(`${spec.label} must be a number`);
    if (spec.min !== undefined && value < spec.min) throw new Error(`${spec.label} must be at least ${spec.min}`);
    if (spec.max !== undefined && value > spec.max) throw new Error(`${spec.label} must be at most ${spec.max}`);
    if (draft.op === "move" && spec.path === "speed_scale" && draft.acceleration_scale === undefined) {
      draft.acceleration_scale = draft.speed_scale;
    }
    setPath(root, spec.path, spec.scalar ? constant(value / (spec.displayScale ?? 1)) : value / (spec.displayScale ?? 1));
    if (draft.op === "move" && /^target\.(xyz_m|rpy_rad)\./.test(spec.path)) draft.target_space = "cartesian";
    if (draft.op === "move" && spec.path.startsWith("target.joint_values_rad.")) draft.target_space = "joint";
    return draft;
  }
  if (spec.kind === "enum") {
    if (typeof value !== "string" || !spec.options?.includes(value)) throw new Error(`${spec.label} must be one of ${spec.options?.join(", ")}`);
    setPath(root, spec.path, value);
    if (draft.op === "io" && spec.path === "action") {
      delete draft.duration_s; delete draft.timeout_s; delete draft.stable_s;
      if (value === "pulse") { draft.state=true; draft.duration_s=0.5; }
      if (value === "wait") { draft.timeout_s=5; draft.stable_s=0.05; }
    }
    return draft;
  }
  if (spec.kind === "bool") {
    setPath(root, spec.path, Boolean(value));
    return draft;
  }
  setPath(root, spec.path, String(value));
  return draft;
}

/** The document the DEFAULT column reads: for a freespace node's overrides
 *  the program defaults, for everything else the node as it was opened. */
export function beneathOf(opened: ProgramV2Node, context: SpecContext): Record<string, unknown> {
  const beneath = structuredClone(opened) as unknown as Record<string, unknown>;
  if (isFreespace(opened)) beneath.overrides = { ...context.defaults };
  return beneath;
}

// ------------------------------------------------------------ validation

const PADDING_POINTS: PosedPoint[] = [
  { xyz_m: [0, 0, 0], rpy_rad: [0, 0, 0] },
  { xyz_m: [0.1, 0, 0], rpy_rad: [0, 0, 0] },
];

function paddingWeld(id: string): WeldNode {
  return {
    id, op: "weld", enabled: true, label: id, comment: "", row_hash: `pending:${id}`,
    geometry: { kind: "posed_polyline", frame_id: "world", points: PADDING_POINTS },
    travel_speed_mm_s: { kind: "constant", value: 4 },
    standoff_mm: { kind: "constant", value: 12 },
    weave: { shape: "none" },
    weld_parameters: {},
  };
}

function padding(op: "approach" | "retract"): ProgramV2Node {
  return { id: `_${op}`, op, enabled: true, label: op, comment: "", row_hash: `pending:_${op}` };
}

/**
 * What is wrong with one node, judged by the contract. The node is placed
 * in the smallest line that is valid around it, so every message is about
 * the node and not about its neighbours; the line itself is judged by the
 * document on Save.
 */
export function nodeProblems(node: ProgramV2Node): string[] {
  let nodes: ProgramV2Node[];
  switch (node.op) {
    case "weld": nodes = [padding("approach"), node, padding("retract")]; break;
    case "approach": nodes = [node, paddingWeld("_w1"), padding("retract")]; break;
    case "retract": nodes = [padding("approach"), paddingWeld("_w1"), node]; break;
    case "transit": nodes = [padding("approach"), paddingWeld("_w1"), node, paddingWeld("_w2"), padding("retract")]; break;
    case "home": nodes = [node, padding("approach"), paddingWeld("_w1"), padding("retract")]; break;
    default: nodes = [padding("approach"), paddingWeld("_w1"), padding("retract"), node];
  }
  const document = {
    schema: "robot.v4.program.v2",
    program_id: "draft",
    program_name: "draft",
    program_revision: 1,
    base_program_revision: 0,
    program_state: "draft",
    execution_mode: "simulation",
    created_utc: "2026-01-01T00:00:00Z",
    units: { length: "m", angle: "rad", time: "s", joint_order: ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"] },
    producer: { component: "offline-programming-v1", source_revision: "draft", inputs: [{ kind: "project", ref: "draft", sha256: "0".repeat(64) }] },
    workpiece: { frame_id: "world" },
    placement: { cell_id: "draft", anchor_xyz_m: [0, 0, 0], offset_xyz_m: [0, 0, 0], rpy_rad: [0, 0, 0] },
    robot: { model_id: "draft", robot_description_sha256: "sha256:" + "0".repeat(64) },
    defaults: { clearance_min_mm: 20, speed_scale: 1, torch_policy: "free", positioner_policy: "free" },
    nodes,
    planner_owns: ["everything a plan is"],
  };
  try {
    assertRobotV4ProgramV2(document);
    return [];
  } catch (error) {
    return [error instanceof Error ? error.message : String(error)];
  }
}

export { CLASSIC_EXTENSION };
