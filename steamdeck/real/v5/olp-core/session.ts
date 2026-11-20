// The pendant's program session: OLP's own project, edits and history, with
// the selection a desktop tab would hold. It stands where main.ts's `mutate`
// stands -- one revision, one undo step, rows recompiled by OLP's deriveRows --
// and exchanges only JSON-able values with the Qt side.
import { importProjectJSON, exportProjectJSON } from "../../../../offline-programming/v1/ui/src/domain/serialization.ts";
import { adoptRemoteProject, copyProject } from "../../../../offline-programming/v1/ui/src/storage/cell-sync.ts";
import {
  adoptRobotDescription, pinRobotDescription, reviseProject, robotDescriptionDrift,
  type CellSettings, type OfflineProgrammingProject, type ProjectChange,
} from "../../../../offline-programming/v1/ui/src/domain/project.ts";
import { cellSettingsPull } from "../../../../offline-programming/v1/ui/src/domain/robot-model.ts";
import { robotSettingsReview } from "../../../../offline-programming/v1/ui/src/domain/robot-settings-review.ts";
import { robotModelFor } from "../../../../offline-programming/v1/ui/src/viewer/models.ts";
import { projectPath, projectRowLabels } from "../../../../offline-programming/v1/ui/src/viewer/workstation.ts";
import { buildMotionRuns } from "../../../../offline-programming/v1/ui/src/viewer/geometry.ts";
import {
  captureDescription, checkCaptureCalibration, checkCaptureModel, machineBoundCapture, programModelMismatch, teachCapture,
  type CaptureModelIdentity, type MeasuredPose,
} from "../../../../offline-programming/v1/ui/src/program-v2/capture.ts";
import { createBlankSpreadsheetProject } from "../../../../offline-programming/v1/ui/src/domain/spreadsheet.ts";
import { ProjectHistory } from "../../../../offline-programming/v1/ui/src/domain/history.ts";
import {
  defaultRobotModel, knownRobotModel, parseRobotCatalogue, setConnectedRobot, setRobotCatalogue,
} from "../../../../offline-programming/v1/ui/src/domain/robots.ts";
import type { MoveNode, ProgramV2Node, TeachPose } from "../../../../offline-programming/v1/ui/src/contracts/robot-v4-program-v2.ts";
import { nodeSummary } from "../../../../offline-programming/v1/ui/src/program-v2/summaries.ts";
import { HOME_WAYPOINT_SPEED_MM_S } from "../../../../offline-programming/v1/ui/src/program-v2/move-defaults.ts";
import { applyEdit, dig, groupsForNode, type EditValue, type FieldSpec, type FieldGroup } from "../../../../offline-programming/v1/ui/src/program-v2/field-specs.ts";
import { dwellNode, effectiveNodes, freespaceNode, ioNode, moveBlocked, nodeById } from "../../../../offline-programming/v1/ui/src/program-v2/nodes.ts";
import { rowNodeId, rowsOfNode } from "../../../../offline-programming/v1/ui/src/program-v2/rows-from-nodes.ts";
import { documentProblems } from "../../../../offline-programming/v1/ui/src/program-v2/from-project.ts";
import {
  commitProgramNode, deleteBlocked, deleteProgramNode, deriveRows, insertEventNode,
  moveProgramNode, recordMove, setNodeEnabled, type RecordMode,
} from "../../../../offline-programming/v1/ui/src/program-v2/edits.ts";

let project: OfflineProgrammingProject | null = null;
let selected: string | null = null;
// Deleting the current taught point leaves the TCP between its surviving
// neighbours. The successor highlighted by the table is not an arrival there.
// This cursor is local to the current edit, never persisted as program data.
let deletedTraceSelection: {
  program: string; removed: string; selection: string | null; before: string[]; after: string[];
} | null = null;
const history = new ProjectHistory();
let pendingRecord: { base: OfflineProgrammingProject; draft: OfflineProgrammingProject; id: string } | null = null;
let lastRecorded: { program: string; id: string } | null = null;

function current(): OfflineProgrammingProject {
  if (!project) throw new Error("No program is open");
  return project;
}

function mutate(change: ProjectChange, edit: (draft: OfflineProgrammingProject) => void): void {
  const previous = current();
  project = reviseProject(previous, change, (draft) => {
    edit(draft);
    deriveRows(draft, change, selected);
  });
  history.push(previous);
}

/** GET /robots, and the connected cell's robot: what pinning and adoption read. */
export function setRobots(catalogue: unknown, connected: { modelId: string; robotDescriptionSha256: string } | null): void {
  setRobotCatalogue(parseRobotCatalogue(catalogue));
  setConnectedRobot(connected);
  cachedProgramView = null;
}

export function openProject(text: string): void {
  project = importProjectJSON(text);
  history.clear();
  deletedTraceSelection = null;
  selected = project.nodes[0]?.id ?? null;
}

/** As desktop OLP's New program: a blank rails program pinned to the current description. */
export function newProject(name: string, robotModel: string): void {
  if (!robotModel) throw new Error("The OLP server listed no robot to write a program for");
  project = pinRobotDescription(createBlankSpreadsheetProject({ name, robotModel, railsAuthoring: true }));
  history.clear();
  deletedTraceSelection = null;
  selected = null;
}

/** The model a program starts for before any cell has said which robot it
 *  is: the last connected cell's when the server still describes it, else
 *  the catalogue's first (OLP's defaultRobotModel); "" without a catalogue.
 *  Never a model name written into the pendant. */
export function startupModel(lastConnected: string): string {
  return lastConnected && knownRobotModel(lastConnected) ? lastConnected : defaultRobotModel();
}

/** The open program's identity: what a remembered Save As file must still hold. */
export function projectIdentity(): string {
  return current().identity.id;
}

/** A program export's identity, or "" when the text is not one OLP opens. */
export function projectIdentityOf(text: string): string {
  try {
    return importProjectJSON(text).identity.id;
  } catch {
    return "";
  }
}

/** Why this program cannot be taught or planned on the connected cell, in
 *  OLP's words (program-v2/capture.ts), or "". */
export function modelMismatch(cellModel: string): string {
  return programModelMismatch(current().robot.model, cellModel);
}

/**
 * The open program follows the connected cell's robot. A program with
 * nothing in it -- no node, no plan -- is started again for the cell's model
 * under the same name; one with nodes is left exactly as it is and says why
 * it cannot be used here (`modelMismatch`): its waypoints were taught on
 * another robot and are never converted silently.
 */
export function followCellModel(cellModel: string): { action: "none" | "match" | "repinned" | "mismatch"; message: string } {
  const p = current();
  if (!cellModel) return { action: "none", message: "" };
  if (p.robot.model === cellModel) return { action: "match", message: "" };
  if (p.nodes.length === 0 && !p.denseTrajectory && knownRobotModel(cellModel)) {
    const was = p.robot.model;
    newProject(p.identity.name, cellModel);
    return { action: "repinned", message: `The empty program "${p.identity.name}" is now for ${cellModel}, the connected cell (it was for ${was})` };
  }
  return { action: "mismatch", message: programModelMismatch(p.robot.model, cellModel) };
}

/** The open project, for the pendant's other OLP modules (run.ts). Never mutate it: use commitProject. */
export function currentProject(): OfflineProgrammingProject {
  return current();
}

/** Replace the project with a revision OLP produced from it (a recorded plan), as one undo step. */
export function commitProject(next: OfflineProgrammingProject): void {
  history.push(current());
  project = next;
}

export function exportProject(): string {
  return exportProjectJSON(current());
}

/**
 * A program from the cell's program store replaces the open one (cellsync.ts):
 * the same program's newer revision is one undo step and keeps the selection
 * while its node exists; another program starts its own history. The
 * document is OLP's project as the cell holds it.
 */
export function openCellProject(document: unknown): OfflineProgrammingProject {
  const opened = importProjectJSON(JSON.stringify(document));
  const previous = project;
  const next = adoptRemoteProject(previous, opened);
  if (previous && previous.identity.id === next.identity.id) history.push(previous);
  else history.clear();
  project = next;
  deletedTraceSelection = null;
  if (!selected || !nodeById(next.nodes, selected)) selected = next.nodes[0]?.id ?? null;
  return next;
}

/** "Keep mine as a copy": the open program under a new identity, opened. */
export function copyOpenProject(): OfflineProgrammingProject {
  project = copyProject(current());
  history.clear();
  return project;
}

export function select(id: string | null): void {
  selected = id && nodeById(current().nodes, id) ? id : null;
  deletedTraceSelection = null;
}

/** The surviving endpoints of a deleted selected motion row, until deliberate
 * navigation/teaching changes the cursor. Editing a speed or keeping a plan
 * cannot imply TCP arrival. Panasonic OM0105045E29, section 6.15. */
export function traceSelectionGap() {
  const gap = deletedTraceSelection, p = current();
  if (!gap || gap.program !== p.identity.id || gap.selection !== selected) return null;
  const motion = new Set(effectiveNodes(p.nodes).filter(n => n.enabled !== false && !["io", "dwell"].includes(n.op)).map(n => n.id));
  return { program: gap.program, removed: gap.removed, selection: gap.selection,
    previous: gap.before.slice().reverse().find(id => motion.has(id)) ?? null,
    next: gap.after.find(id => motion.has(id)) ?? null, restored: motion.has(gap.removed) };
}

function deletionCursor(p: OfflineProgrammingProject, id: string) {
  const motion = effectiveNodes(p.nodes).filter(n => n.enabled !== false && !["io", "dwell"].includes(n.op)).map(n => n.id);
  const at = motion.indexOf(id);
  return at < 0 ? null : { program: p.identity.id, removed: id, selection: selected,
    before: motion.slice(0, at), after: motion.slice(at + 1) };
}

/** Undo changes the document, not the physical TCP. Retain the deletion
 * relationship; if its source point reappears, approach it before claiming
 * arrival. No robot position or cursor metadata enters the saved project. */
function restoreHistoryCursor(before: OfflineProgrammingProject, gap: typeof deletedTraceSelection): void {
  const p = current();
  if (!gap && selected && nodeById(before.nodes, selected) && !nodeById(p.nodes, selected))
    gap = deletionCursor(before, selected);
  deletedTraceSelection = gap;
  if (gap && gap.program === p.identity.id) {
    const ids = new Set(p.nodes.map(n => n.id));
    if (ids.has(gap.removed)) selected = gap.removed;
    else selected = gap.after.find(id => ids.has(id)) ?? gap.before.slice().reverse().find(id => ids.has(id)) ?? p.nodes[0]?.id ?? null;
    gap.selection = selected;
  } else if (selected && !nodeById(p.nodes, selected)) selected = p.nodes[0]?.id ?? null;
}

export interface Row {
  id: string; number: number; op: string; family: string; enabled: boolean; toggleable: boolean;
  label: string; geometry: string; process: string; freespace: string; status: string; statusClass: string;
  comment: string; upBlocked: string | null; downBlocked: string | null; deleteBlocked: string | null;
  /** The move's inline cells, by OLP field path, in display units. */
  cells: Record<string, number | string>;
}

// Native status refreshes ask for this table several times without editing.
// Keep only the current immutable project's derived rows/validation, never a
// history-sized cache. Selection and undo/redo availability remain live below.
let cachedProgramView: { project: OfflineProgrammingProject; rows: Row[]; problems: string[] } | null = null;
function programView(p: OfflineProgrammingProject) {
  if (cachedProgramView?.project === p) return cachedProgramView;
  const effective = new Set(effectiveNodes(p.nodes).map((node) => node.id));
  const planned = new Set(p.denseTrajectory?.segments.map((segment) => segment.sourceId) ?? []);
  const hasWeld = p.nodes.some((node) => node.op === "weld");
  const context = {
    defaults: p.programV2.defaults,
    rowsOf: (id: string) => rowsOfNode(p, id),
    skipped: (id: string) => !effective.has(id),
    // reviseProject drops the trajectory on every planning change, so a
    // trajectory still here was planned from these nodes.
    planned: (id: string) => planned.has(id),
  };
  const rows = p.nodes.map((node, index): Row => {
    const summary = nodeSummary(node, index + 1, context);
    const cells: Record<string, number | string> = {};
    if (node.op === "move") {
      node.target.xyz_m.forEach((v, i) => { cells[`target.xyz_m.${i}`] = v * 1000; });
      node.target.rpy_rad.forEach((v, i) => { cells[`target.rpy_rad.${i}`] = v * 180 / Math.PI; });
      cells.speed_scale = node.speed_scale * 100;
      cells.speed_mm_s = node.speed_mm_s;
      cells.motion = node.motion;
    } else if (node.op === "dwell") {
      cells.duration_s = node.duration_s;
    } else if (node.op === "home" && !hasWeld) {
      cells.default_speed_mm_s = HOME_WAYPOINT_SPEED_MM_S;
    }
    return {
      id: node.id, number: summary.number, op: node.op, family: summary.family, enabled: node.enabled,
      toggleable: summary.family === "weld" || node.op === "move",
      label: summary.label + (node.op === "move" && node.weld_transition && node.weld_transition !== "none" ? ` · WELD ${node.weld_transition.toUpperCase()}` : ""), geometry: summary.geometry, process: summary.process, freespace: summary.freespace,
      status: summary.status, statusClass: summary.statusClass, comment: summary.comment,
      upBlocked: moveBlocked(p.nodes, node.id, -1), downBlocked: moveBlocked(p.nodes, node.id, 1),
      deleteBlocked: deleteBlocked(p.nodes, node.id), cells,
    };
  });
  return cachedProgramView = { project: p, rows, problems: p.nodes.length ? documentProblems(p) : [] };
}

export function view(): {
  name: string; revision: number; programState: string; railsAuthoring: boolean; selected: string | null;
  canUndo: boolean; canRedo: boolean; speedScale: number; rows: Row[]; problems: string[];
  model: string; description: string | null;
} {
  const p = current();
  const { rows, problems } = programView(p);
  return {
    name: p.identity.name, revision: p.identity.revision, programState: p.program.programState,
    railsAuthoring: Boolean(p.railsAuthoring), selected, canUndo: history.canUndo, canRedo: history.canRedo,
    speedScale: p.programV2.speedScale ?? 1, rows, problems,
    model: p.robot.model, description: p.robot.description?.robotDescriptionSha256 ?? null,
  };
}

/**
 * A capture's first step, under OLP's rules: which description it is for and
 * whether the machine's own calibration must be fetched before the capture
 * model is asked for. `measured` is machine.teachObservation().
 */
export function captureStart(mode: RecordMode, measured: MeasuredPose): { description: string; machineBound: boolean; modelId: string; revision: number } {
  const p = current();
  const selectedNode = p.nodes.find((node) => node.id === selected);
  if (mode !== "new" && selectedNode?.op !== "move") throw new Error("Select a taught waypoint first");
  const description = captureDescription(p, measured, measured.pose);
  return { description, machineBound: machineBoundCapture(measured), modelId: measured.pose.modelId, revision: p.identity.revision };
}

/**
 * The capture itself, once the calibration and capture model are in hand and
 * the TCP (xyz m, URDF rpy rad) has come out of FK on the capture model's URDF.
 * Refused when the program moved on while the capture was being resolved.
 */
export function captureRecord(mode: RecordMode, measured: MeasuredPose, revision: number, description: string,
  calibration: CellSettings | null, identity: CaptureModelIdentity, tcp: number[], newId: string, draftOnly = false): string {
  const p = current();
  if (p.identity.revision !== revision) throw new Error("Program changed during capture; record again");
  checkCaptureCalibration(measured, description, calibration ?? p.robot.cell);
  checkCaptureModel(identity, description, calibration ?? p.robot.cell, mode, p.nodes.find((node) => node.id === selected));
  const { target, capture } = teachCapture(p, measured.pose, tcp, description, identity, measured);
  return draftOnly && mode === "new" ? prepareRecord(target, capture, newId) : record(mode, target, capture, newId);
}

/** Record, reteach or record-via from a destination and its capture record. */
export function record(mode: RecordMode, target: TeachPose, capture: MoveNode["capture"], newId: string): string {
  let recorded = "";
  mutate("nodes", (draft) => { recorded = recordMove(draft, mode, selected, target, capture, newId); });
  selected = recorded;
  deletedTraceSelection = null;
  return recorded;
}

/** Keep a newly measured point outside the program until the operator saves it. */
let weldDefaults = { start_delay_s: 0, stop_delay_s: 0 };
export function setWeldDefaults(value: {start_delay_s?:number; stop_delay_s?:number}): void {
  const valid=(v:unknown)=>typeof v==="number"&&Number.isFinite(v)&&v>=0&&v<=30?v:0;
  weldDefaults={start_delay_s:valid(value.start_delay_s),stop_delay_s:valid(value.stop_delay_s)};
}
export function prepareRecord(target: TeachPose, capture: MoveNode["capture"], newId: string): string {
  const base = current(), draft = structuredClone(base);
  const remembered = lastRecorded?.program === base.identity.id ? base.nodes.find(n => n.id === lastRecorded!.id) : undefined;
  const previous = remembered?.op === "move" ? remembered : [...base.nodes].reverse().find(n => n.op === "move") as MoveNode | undefined;
  recordMove(draft, "new", selected, target, capture, newId);
  const node = draft.nodes.find(n => n.id === newId) as MoveNode;
  if (previous) {
    node.motion = previous.motion;
    node.speed_mm_s = previous.speed_mm_s;
    node.speed_scale = previous.speed_scale;
    node.acceleration_scale = previous.acceleration_scale;
    node.constant_tcp_speed = previous.constant_tcp_speed;
    // A via is geometry, never a reusable setting. Record a new via for MoveC.
  }
  // A new leg inside an active weld defaults to constant TCP cruise, even
  // when the preceding travel point used a variable-speed profile.
  let welding = false;
  for (const step of draft.nodes) {
    if (step.id === newId) break;
    if (step.enabled && step.op === "move" && step.weld_transition === "start") welding = true;
    if (step.enabled && step.op === "move" && step.weld_transition === "stop") welding = false;
  }
  if (welding) node.constant_tcp_speed = true;
  node.weld_start_delay_s = weldDefaults.start_delay_s;
  node.weld_stop_delay_s = weldDefaults.stop_delay_s;
  pendingRecord = { base, draft, id: newId };
  return newId;
}
function recordDraft() {
  if (!pendingRecord || pendingRecord.base !== current()) throw new Error("Program changed during recording; cancel and record again");
  return pendingRecord;
}
export function pendingRecordFields() { const p = recordDraft(); return fieldsIn(p.draft, p.id); }
export function editPendingRecord(_id: string, path: string, value: EditValue): void {
  const p = recordDraft();
  const node = nodeById(p.draft.nodes, p.id)!;
  const spec = groupsForNode(node, { defaults: p.draft.programV2.defaults, classicSeam: false }).flatMap(g => g.fields).find(s => s.path === path);
  if (!spec || spec.readOnly) throw new Error("This captured field cannot be edited");
  const edited = applyEdit(structuredClone(node), spec, value);
  if (edited.op === "move" && path === "weld_transition" && (value === "start" || value === "stop"))
    edited.constant_tcp_speed = true;
  commitProgramNode(p.draft, edited, null);
}
export function cancelPendingRecord(): void { pendingRecord = null; }
export function commitPendingRecord(): string {
  const p = recordDraft();
  mutate("nodes", draft => { draft.nodes = structuredClone(p.draft.nodes); });
  selected = p.id;
  deletedTraceSelection = null;
  lastRecorded = { program: current().identity.id, id: selected };
  pendingRecord = null;
  return selected;
}

/** + Dwell / + IO / + Home with OLP's defaults; returns the new node's id. */
export function addEvent(kind: "dwell" | "io" | "home"): string {
  const node = kind === "dwell" ? dwellNode(0.5) : kind === "io" ? ioNode("revpi:1", true) : freespaceNode("home");
  mutate("nodes", (draft) => { insertEventNode(draft, node, selected); });
  selected = node.id;
  deletedTraceSelection = null;
  return node.id;
}

export function remove(id: string): void {
  const before = current();
  const index = before.nodes.findIndex((node) => node.id === id);
  const wasSelected = selected === id;
  // Repeated deletions widen the same gap. Deleting another row cannot
  // imply that the TCP arrived at the highlighted successor.
  const gap = traceSelectionGap() ? deletedTraceSelection : wasSelected ? deletionCursor(before, id) : null;
  mutate("nodes", (draft) => { deleteProgramNode(draft, id); });
  if (wasSelected) selected = current().nodes[Math.min(index, current().nodes.length - 1)]?.id ?? null;
  deletedTraceSelection = gap;
  if (gap) gap.selection = selected;
}

export function move(id: string, delta: -1 | 1): void {
  mutate("nodes", (draft) => { moveProgramNode(draft, id, delta); });
  selected = id;
  deletedTraceSelection = null;
}

export function enable(id: string, enabled: boolean): void {
  mutate("nodes", (draft) => { setNodeEnabled(draft, id, enabled); });
  selected = id;
  deletedTraceSelection = null;
}

/** The program's robot settings against the connected machine's, in OLP's words (Workspace › Cell). */
export function robotSettings() {
  const p = current();
  const drift = robotDescriptionDrift(p);
  const review = drift ? robotSettingsReview(p) : null;
  return {
    model: p.robot.model,
    pinned: p.robot.description?.robotDescriptionSha256 ?? null,
    cell: p.robot.cell ? { cell: p.robot.cell.cell, pulledAt: p.robot.cell.pulledAt,
      robotDescriptionSha256: p.robot.cell.robotDescriptionSha256 ?? null,
      machinePlanningCalibrationSha256: p.robot.cell.machinePlanningCalibrationSha256 ?? null,
      runtimeMode: p.robot.cell.runtimeMode } : null,
    drift, review: review ? { title: review.title, explanation: review.explanation, rows: review.rows } : null,
    adopted: p.robot.description?.adopted?.at(-1) ?? null,
  };
}

/** Pull the connected cell's settings into the program (machine.fetchedCellSettings); false when they already match. */
export function pullCellSettings(settings: CellSettings): boolean {
  const pull = cellSettingsPull(current(), settings);
  if (!pull) return false;
  mutate("robot", pull.apply);
  return true;
}

/** Accept the current robot description after reviewing its changes; plan again before running. */
export function adoptDescription(): void {
  if (!robotDescriptionDrift(current())) throw new Error("Robot settings are already up to date");
  mutate("metadata", (draft) => adoptRobotDescription(draft));
}

/** OLP's J6 and TCP camera mounts for the model (viewer/models.ts), as the workstation hangs them. */
export function cameras(modelId: string) {
  return robotModelFor(modelId).cameras;
}

/** The URDF mesh packages that resolve to the model's own description
 *  (viewer/models.ts packageMap). A robot-specific description keeps its
 *  product's package name: one Rosie 1430 arm's says
 *  package://rosie_1430_v1/. */
export function meshPackages(modelId: string): string[] {
  return Object.keys(robotModelFor(modelId).packageMap);
}

/** The authored path as OLP's workstation draws it (viewer/workstation.ts):
 *  every target in world metres, weld or travel, and the sparse row labels,
 *  focused on the pendant's selected node the way a selected row focuses the
 *  desktop stage. World is the robot's URDF frame, which RobotView draws in. */
export function pathOverlay(): {
  runs: { weld: boolean; pointsM: number[][] }[];
  points: { positionM: number[]; weld: boolean }[];
  labels: { text: string; positionM: number[]; weld: boolean; emphasis: boolean }[];
  focused: boolean;
} {
  const p = current();
  const row = selected ? p.rows.find((candidate) => rowNodeId(candidate) === selected) : undefined;
  const shown = { ...p, simulation: { ...p.simulation, selectedRowId: row?.id ?? null } };
  const path = projectPath(shown);
  return {
    runs: buildMotionRuns(path).map(({ motion, pointsM }) => ({ weld: motion === "weld", pointsM: pointsM.map((point) => [...point]) })),
    points: path.map(({ positionM, motion }) => ({ positionM: [...positionM], weld: motion === "weld" })),
    labels: projectRowLabels(shown).map(({ text, positionM, motion, emphasis }) => ({
      text, positionM: [...positionM], weld: motion === "weld", emphasis: Boolean(emphasis),
    })),
    // OLP fades the path to 30 % while a row is selected (syncAuthoringOverlays).
    focused: Boolean(row),
  };
}

/** Which frame a capture's TCP is taken at, and the joints it names, for the model. */
export function captureFrame(modelId: string): { frame: string; jointNames: string[] } {
  const descriptor = robotModelFor(modelId);
  return { frame: descriptor.cameras.tcp.frame, jointNames: [...descriptor.urdfJointNames] };
}

export function setSpeedScale(percent: number): void {
  if (!(percent > 0 && percent <= 100)) throw new Error("Plan speed must be between 1 and 100 %");
  mutate("nodes", (draft) => { draft.programV2.speedScale = percent / 100; });
}

export function undo(): void {
  if (!history.canUndo) return;
  const before = current(), gap = traceSelectionGap() ? deletedTraceSelection : null;
  project = history.undo(before);
  restoreHistoryCursor(before, gap);
}

export function redo(): void {
  if (!history.canRedo) return;
  const before = current(), gap = traceSelectionGap() ? deletedTraceSelection : null;
  project = history.redo(before);
  restoreHistoryCursor(before, gap);
}

/** OLP's parameter dialog for a node: its groups, each field with its value in display units. */
export function fields(id: string): { op: string; label: string; groups: { id: string; group: string; note?: string; fields: (FieldSpec & { value: unknown })[] }[] } {
  return fieldsIn(current(), id);
}
function fieldsIn(p: OfflineProgrammingProject, id: string) {
  const node = nodeById(p.nodes, id);
  if (!node) throw new Error("The node is no longer in the program");
  const classicSeam = Boolean((node.extensions?.classic as { seam_id?: string } | undefined)?.seam_id);
  let groups = groupsForNode(node, { defaults: p.programV2.defaults, classicSeam });
  if (node.op === "move") {
    const byId = (id: string) => groups.find(g => g.id === id)!;
    const motion = byId("motion"), weld = byId("weld");
    const pose = groups.filter(g => g.id === "target" || g.id === "via");
    const advanced = groups.filter(g => g.id === "joint-limits" || g.id === "capture");
    const merged: FieldGroup[] = [
      { ...motion, note: "Weld actions apply after arrival. TCP speed is the requested cruise speed; acceleration and limits still apply.", fields: [...motion.fields.map(f => f.path === "speed_mm_s" ? {...f, label: "TCP speed"} : f), ...weld.fields] },
      { ...byId("identity"), group: "Waypoint" },
      { id: "target", group: "Pose", note: pose[0]?.note, fields: pose.flatMap(g => g.fields.map(f => ({...f, label: g.id === "via" ? `Via ${f.label}` : f.label}))) },
      { id: "advanced", group: "Advanced", fields: advanced.flatMap(g => g.fields) },
    ];
    groups = merged;
  }
  return {
    op: node.op, label: node.label,
    groups: groups.map((group) => ({ ...group, fields: group.fields.map((spec) => ({ ...spec, value: displayValue(node, spec) })) })),
  };
}

function displayValue(node: ProgramV2Node, spec: FieldSpec): unknown {
  if (spec.path.startsWith("_seam.")) return null;
  let value = dig(node as unknown as Record<string, unknown>, spec.path);
  if (value === undefined && node.op === "io" && spec.path === "action") value = "set";
  if (value === undefined && node.op === "move" && spec.path === "weld_transition") value = "none";
  if (value === undefined && node.op === "move" && ["weld_start_delay_s", "weld_stop_delay_s"].includes(spec.path)) value = 0;
  if (spec.scalar && value && typeof value === "object" && (value as { kind?: string }).kind === "constant") value = (value as { value: number }).value;
  return typeof value === "number" && spec.displayScale ? value * spec.displayScale : value ?? null;
}

/**
 * One edit, by OLP field path, in display units: the spec comes from OLP's
 * own groups for the node (or the inline spreadsheet cells, which name the
 * same paths), applyEdit validates and writes it, and the whole document is
 * checked before it becomes a revision.
 */
export function edit(id: string, path: string, value: EditValue): void {
  const p = current();
  const node = nodeById(p.nodes, id);
  if (!node) throw new Error("The node is no longer in the program");
  const classicSeam = Boolean((node.extensions?.classic as { seam_id?: string } | undefined)?.seam_id);
  const spec = groupsForNode(node, { defaults: p.programV2.defaults, classicSeam })
    .flatMap((group) => group.fields).find((candidate) => candidate.path === path);
  if (!spec) throw new Error(`${path} is not editable on a ${node.op}`);
  if (spec.readOnly) throw new Error(`${spec.label}: ${spec.readOnly}`);
  // applyEdit takes the display value: it checks min/max, scales and wraps scalars itself.
  const edited = applyEdit(structuredClone(node), spec, value);
  if (edited.op === "move" && path === "weld_transition" && (value === "start" || value === "stop"))
    edited.constant_tcp_speed = true;
  mutate("nodes", (draft) => {
    commitProgramNode(draft, edited, null);
    const problems = documentProblems(draft);
    if (problems.length) throw new Error(problems[0]);
  });
  selected = id;
}
