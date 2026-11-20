// rosie's program engine (the CLI and the MCP server): OLP's own TypeScript, bundled for Node the
// way steamdeck/real/v5 bundles olp-core for Qt (`make build/olp-core.js`).
// Every edit, weld, capture and plan request goes through the functions the
// browser and the Deck call, so a program edited here is the program they
// open. Nothing in this file decides a rule; it only adapts OLP's calls to a
// client with no window: no viewer, no IndexedDB, one open program.
import { Window } from "happy-dom";
import * as THREE from "three";
import URDFLoader from "urdf-loader";

import { appendCadQueryTopology } from "../../../offline-programming/v1/ui/src/adapters/cadquery.ts";
import { HttpCadClient } from "../../../offline-programming/v1/ui/src/cad/client.ts";
import { CAD_API_PATH, type CadWorkpieceMesh } from "../../../offline-programming/v1/ui/src/cad/contracts.ts";
import { normalizeTopology } from "../../../offline-programming/v1/ui/src/cad/normalize.ts";
import type { ProgramV2Node, WeldNode } from "../../../offline-programming/v1/ui/src/contracts/robot-v4-program-v2.ts";
import { hasWorkpieceGeometry, setWorkpiecePlacement, workpiecePlacement } from "../../../offline-programming/v1/ui/src/domain/placement.ts";
import { DEFAULT_TRAVEL_SPEED_MPS, restoreAsRevision, reviseProject, type OfflineProgrammingProject } from "../../../offline-programming/v1/ui/src/domain/project.ts";
import { importProjectJSON } from "../../../offline-programming/v1/ui/src/domain/serialization.ts";
import { processTorchDefaults } from "../../../offline-programming/v1/ui/src/domain/weld.ts";
import {
  STAND_ID, defaultWorkcellFixtures, editWorkcellFixture, fixtureById, fixtureSublabel, newBoxFixture, newStepFixture,
  workcellFixtures, workcellFixturesWire, type WorkcellFixture, type WorkcellFixtureField,
} from "../../../offline-programming/v1/ui/src/domain/workcell.ts";
import {
  activateWorkpiece, activeWorkpieceId, environmentFixtures, forgetActiveWorkpiece, projectWorkpieces, workpieceProject,
} from "../../../offline-programming/v1/ui/src/domain/workpieces.ts";
import { denseExecutionState } from "../../../offline-programming/v1/ui/src/execution/denseClient.ts";
import { createPresetLibrary, memoryPresetStore } from "../../../offline-programming/v1/ui/src/parameters/library.ts";
import { createParameterSet, type ParameterPreset, type ParameterSchema, type ParameterValue } from "../../../offline-programming/v1/ui/src/parameters/model.ts";
import { captureDescription, checkCaptureModel, teachCapture, type CaptureModelIdentity } from "../../../offline-programming/v1/ui/src/program-v2/capture.ts";
import { deriveRows, type RecordMode } from "../../../offline-programming/v1/ui/src/program-v2/edits.ts";
import { dig } from "../../../offline-programming/v1/ui/src/program-v2/field-specs.ts";
import { documentProblems } from "../../../offline-programming/v1/ui/src/program-v2/from-project.ts";
import { insertWeld, isWeld, nodeById } from "../../../offline-programming/v1/ui/src/program-v2/nodes.ts";
import { isRailsWeld } from "../../../offline-programming/v1/ui/src/program-v2/rails-node.ts";
import { narrowsToRun } from "../../../offline-programming/v1/ui/src/rails/decisions.ts";
import {
  SEARCH_SCHEMA, SEARCH_SCOPE, SHIPPED_SEARCH_PRESETS, SHIPPED_TORCH_PRESETS, TORCH_SCHEMA, TORCH_SCOPE, torchProblems, torchRequest,
} from "../../../offline-programming/v1/ui/src/rails/parameters.ts";
import { hasMotionNodes, measuredStartUnsupported, plannedNodes, planRequestBody, shippedTooling } from "../../../offline-programming/v1/ui/src/rails/planRequest.ts";
import type { FreeSpaceBackend, PlanRequestBody } from "../../../offline-programming/v1/ui/src/rails/client.ts";
import { fromPoints, isEditable, type ScalarFunction } from "../../../offline-programming/v1/ui/src/rails/scalar-function.ts";
import { fractionAt, summarise } from "../../../offline-programming/v1/ui/src/rails/span-math.ts";
import type { RobotPoseSnapshot } from "../../../offline-programming/v1/ui/src/simulation/contracts.ts";
import { decodeDenseTrajectory } from "../../../offline-programming/v1/ui/src/execution/denseTrajectoryBlob.ts";
import { denseSegmentNodeId } from "../../../offline-programming/v1/ui/src/program-v2/dense-cursor.ts";
import { robotModelFor } from "../../../offline-programming/v1/ui/src/viewer/models.ts";
import {
  DEFAULT_WAYPOINT_STANDOFF_MM, STANDOFF_MAX_MM, STANDOFF_MIN_MM, newId, waypointArcFractions, type Continuity, type V3, type Waypoint,
} from "../../../offline-programming/v1/ui/src/waypoints/model.ts";
import {
  isManualPoseWeld, quatToRpy, resolveStartLinks, rpyToQuat, shapeOf, startCandidates, waypointWeldOf, withWaypoints, type WeldTool,
} from "../../../offline-programming/v1/ui/src/waypoints/node.ts";
import { WaypointSession, type WaypointHost } from "../../../offline-programming/v1/ui/src/waypoints/session.ts";
import { machineStatus } from "../../../steamdeck/real/v5/olp-core/machine.ts";
import { commitProject, currentProject, record, select } from "../../../steamdeck/real/v5/olp-core/session.ts";

export * from "../../../steamdeck/real/v5/olp-core/session.ts";
// The machine as the Deck sees it: replies in, OLP's teach and cell-settings rules out.
export { fetchedCellSettings, observeCapabilities, observeStatus, teachObservation } from "../../../steamdeck/real/v5/olp-core/machine.ts";
export { applyPlan, loadBody, planDigest, setPlanFromProgramStart, setRepeat } from "../../../steamdeck/real/v5/olp-core/run.ts";
export { decideWeld, measureRunPiece, posedSeam, runIndexAt, searchSeam, shippedSearchKnobs } from "../../../offline-programming/v1/ui/src/rails/authoring.ts";
export { narrowsToRun, fractionAt, summarise, shippedTooling, hasWorkpieceGeometry };

/**
 * Undo or redo across processes: the CLI keeps each revision of the open
 * program on disk (an engine's ProjectHistory lives only as long as its
 * process). The open program becomes `snapshotText` through OLP's own
 * restoreAsRevision -- revision bumped, kept plan dropped -- exactly as the
 * browser's ProjectHistory.undo/redo does, never a bare reload of the old
 * revision (which would revive a plan for a program that has moved on).
 */
export function restoreSnapshot(snapshotText: string, kind: "undo" | "redo"): void {
  commitProject(restoreAsRevision(currentProject(), importProjectJSON(snapshotText), kind));
}

/** The plan kept on the program, and whether it was planned from this revision. */
export function planRecord() {
  const project = currentProject();
  const record = project.denseTrajectory;
  if (!record) return null;
  return {
    trajectoryDigest: record.trajectoryDigest, planId: record.planId, programId: record.programId,
    programDigest: record.programDigest, manifestRevision: record.manifestRevision, planRevision: record.planRevision,
    createdAt: record.createdAt, totalSampleCount: record.totalSampleCount, totalDurationS: record.totalDurationS,
    segments: record.segments, robot: record.robot,
    current: record.sourceProjectRevision === project.identity.revision,
  };
}

/** The fitted gun the browser's torch window falls back to: its first shipped preset. */
export function shippedTorch(): Record<string, unknown> {
  const preset = SHIPPED_TORCH_PRESETS[0]!;
  return torchRequest(preset.values, preset);
}

/** The program's parts and where the active one sits in the cell's work frame. */
export function partView() {
  const project = currentProject();
  const placement = workpiecePlacement(project);
  const geometry = hasWorkpieceGeometry(project);
  return {
    railsAuthoring: Boolean(project.railsAuthoring),
    // A blank program carries a placeholder CAD reference with no geometry: no part.
    cad: geometry && project.cad ? { filename: project.cad.filename, fingerprint: project.cad.fingerprint } : null,
    parts: projectWorkpieces(project).map((part) => ({ id: part.id, filename: part.cad?.filename ?? null, fingerprint: part.cad?.fingerprint ?? null })),
    activeWorkpieceId: project.activeWorkpieceId ?? null,
    placement: placement ? {
      offsetFromWorkFrameMm: placement.offsetFromWorkFrameM.map((v) => v * 1000),
      rotationRpyDeg: placement.rotationRpyRad.map((v) => (v * 180) / Math.PI),
    } : null,
    hasGeometry: geometry,
    edges: Array.isArray(project.edgeCatalog) ? project.edgeCatalog.length : 0,
  };
}

/**
 * A CadQuery topology answer becomes the program's part, as the browser's
 * importStepBytes does: OLP's appendCadQueryTopology centres it on the cell's
 * work frame, and the first part names the program after its file.
 */
export function importPart(topologyWire: unknown, filename: string): { fingerprint: string; edges: number } {
  const topology = normalizeTopology(topologyWire);
  const project = currentProject();
  const firstPart = projectWorkpieces(project).length === 0;
  const next = appendCadQueryTopology(project, topology);
  if (firstPart) next.identity.name = filename.replace(/\.(step|stp)$/i, "");
  commitProject(next);
  return { fingerprint: topology.fingerprint, edges: topology.edges.length };
}

/** Move the part: its pose in the cell's work frame (mm, degrees), as the Move part fields write it. */
export function placePart(offsetMm: number[], rotationRpyDeg: number[]): void {
  commitProject(reviseProject(currentProject(), "fixtures", (draft) => {
    if (!setWorkpiecePlacement(draft, {
      offsetFromWorkFrameM: [offsetMm[0]! / 1000, offsetMm[1]! / 1000, offsetMm[2]! / 1000],
      rotationRpyRad: [rotationRpyDeg[0]! * Math.PI / 180, rotationRpyDeg[1]! * Math.PI / 180, rotationRpyDeg[2]! * Math.PI / 180],
    })) throw new Error("The program has no part to place; import a STEP first");
    deriveRows(draft, "fixtures", null);
  }));
}

/** The rails weld already in the program under this id, which a reaccept merges into. */
export function railsWeld(id: string): WeldNode | undefined {
  const node = nodeById(currentProject().nodes, id);
  return node && isRailsWeld(node) ? node : undefined;
}

/** Write an accepted rails weld, as the browser's upsertWeldNode does: replace
 *  it in place, or insert it with its approach and retract. */
export function upsertWeld(node: WeldNode): void {
  const existing = nodeById(currentProject().nodes, node.id);
  commitProject(reviseProject(currentProject(), "nodes", (draft) => {
    if (existing) draft.nodes = draft.nodes.map((candidate) => (candidate.id === node.id ? structuredClone(node) : candidate));
    else draft.nodes = insertWeld(draft.nodes, node);
    deriveRows(draft, "nodes", node.id);
  }));
  select(node.id);
}

export interface PlanOptions {
  /** A cell is selected on the OLP server: a motion program then plans from its measured pose. */
  connected: boolean;
  fromProgramStart: boolean;
  closeLoop: boolean;
  freeSpaceBackend: FreeSpaceBackend;
  /** The fitted gun with its default standoff (railsSettings().tooling). */
  tooling: Record<string, unknown>;
  /** Tessellated STEP fixtures and inactive parts, by lowercase fingerprint. */
  meshes: ReadonlyMap<string, CadWorkpieceMesh>;
}

/**
 * POST /weld-plan's body for the open program, as rails mode's Plan packs it
 * (rails/index.ts planRequestBody): the session's fitted gun, the active
 * part's STEP when the program has one, the workcell fixtures and the other
 * parts as meshes, and the program id and revision the answer must be
 * applied to (applyPlan refuses an answer for another program or revision).
 */
export async function planRequest(stepBase64: string, stepFilename: string, options: PlanOptions): Promise<{ revision: number; projectId: string; body: PlanRequestBody }> {
  const project = currentProject();
  if (plannedNodes(project).length === 0) throw new Error("Nothing to plan — record a waypoint, add a Home or accept a weld");
  const geometry = hasWorkpieceGeometry(project);
  if (geometry && !stepBase64) throw new Error("This program has a part: its STEP must travel with the plan request");
  const unsupported = measuredStartUnsupported(project, options.connected);
  if (unsupported) throw new Error(unsupported);
  const body = await planRequestBody(project, {
    tooling: options.tooling,
    stepBase64: geometry ? stepBase64 : "",
    stepFilename,
    fixtures: workcellFixturesWire(environmentFixtures(project), options.meshes),
    freeSpaceBackend: options.freeSpaceBackend,
    requireMeasuredStart: hasMotionNodes(project) && options.connected,
    planFromProgramStart: options.fromProgramStart,
    closeLoop: options.closeLoop,
  });
  return { revision: project.identity.revision, projectId: project.identity.id, body };
}

function snapshotOf(jointsRad: readonly number[], observedAtMs: number): RobotPoseSnapshot {
  const project = currentProject();
  const names = robotModelFor(project.robot.model).urdfJointNames;
  if (jointsRad.length !== names.length) {
    throw new Error(`${project.robot.model} has ${names.length} joints (${names.join(", ")}); ${jointsRad.length} values were given`);
  }
  if (!jointsRad.every(Number.isFinite)) throw new Error("Every joint value must be a finite number of radians");
  return { modelId: project.robot.model, jointOrder: [...names], jointsRad: [...jointsRad], source: "preview", observedAtMs };
}

/** POST /robots/capture-model's body for a preview capture of these joints. */
export function captureModelRequest(jointsRad: readonly number[]): Record<string, string> {
  const project = currentProject();
  const snapshot = snapshotOf(jointsRad, Date.now());
  return {
    model_id: snapshot.modelId,
    robot_description_sha256: captureDescription(project, null, snapshot),
    machine_planning_calibration_base64: project.robot.cell?.machinePlanningCalibrationBase64 ?? "",
  };
}

let domReady = false;

/** urdf-loader parses with DOMParser and checks against Document, as in the
 *  Deck's capture fixture (steamdeck/real/v5/tests/capture_fk_fixture.mjs). */
function ensureDom(): void {
  if (domReady) return;
  const window = new Window() as unknown as Record<string, unknown>;
  const globals = globalThis as Record<string, unknown>;
  // The window's own classes: a bare happy-dom DOMParser has no window to build documents in.
  for (const name of ["DOMParser", "Document", "XMLDocument", "Element", "Node"] as const) globals[name] ??= window[name];
  domReady = true;
}

/**
 * urdf-loader on a URDF string. happy-dom (20.x) parses a document that opens
 * with an XML declaration as HTML and finds no <robot>, where a browser's
 * DOMParser does not; the declaration carries nothing the robot needs, so it
 * is dropped first. The compiled capture models of two Rosie 1430 arms carry one.
 */
function parseUrdf(urdf: string) {
  ensureDom();
  const loader = new URDFLoader();
  loader.parseVisual = false;
  loader.parseCollision = false;
  return loader.parse(urdf.replace(/^\s*<\?xml[^>]*\?>\s*/, ""));
}

/** The TCP of a pose on the capture model's URDF: workstation-renderer.ts tcpPoseFor, without the renderer. */
export function tcpPose(urdf: string, snapshot: RobotPoseSnapshot): [number, number, number, number, number, number] {
  const descriptor = robotModelFor(snapshot.modelId);
  const copy = parseUrdf(urdf);
  for (const name of descriptor.urdfJointNames) {
    const index = snapshot.jointOrder.indexOf(name), value = snapshot.jointsRad[index];
    if (index < 0 || value === undefined || !Number.isFinite(value) || !copy.joints[name]) throw new Error(`Capture is missing ${name}`);
    copy.joints[name]!.ignoreLimits = true;
    copy.setJointValue(name, value);
  }
  const frame = copy.links[descriptor.cameras.tcp.frame] ?? copy.joints[descriptor.cameras.tcp.frame];
  if (!frame) throw new Error("The robot description has no TCP frame");
  frame.updateWorldMatrix(true, false);
  const position = new THREE.Vector3().setFromMatrixPosition(frame.matrixWorld);
  const rotation = new THREE.Euler().setFromRotationMatrix(frame.matrixWorld, "ZYX");
  return [position.x, position.y, position.z, rotation.x, rotation.y, rotation.z];
}

/**
 * Record a waypoint from joint values, as desktop OLP records one in
 * simulation (main.ts recordWaypoint with no measured pose): the capture
 * model OLP served, its FK at the TCP frame, OLP's teachCapture and
 * recordMove. `anchorId` is the row it goes after ("new") or the waypoint it
 * reteaches or gives a via ("reteach", "via").
 */
export function recordPreview(mode: RecordMode, jointsRad: readonly number[], anchorId: string | null,
  model: { urdf: string; identity: CaptureModelIdentity }, newId: string): string {
  const project = currentProject();
  select(anchorId);
  const snapshot = snapshotOf(jointsRad, Date.now());
  const description = captureDescription(project, null, snapshot);
  const anchor = anchorId ? nodeById(project.nodes, anchorId) ?? undefined : undefined;
  checkCaptureModel(model.identity, description, project.robot.cell, mode, anchor);
  const { target, capture } = teachCapture(project, snapshot, tcpPose(model.urdf, snapshot), description, model.identity, null);
  return record(mode, target, capture, newId);
}

/** POST /robots/capture-model's body for the program's own model, pin and pulled calibration. */
export function programModelRequest(): Record<string, string> {
  const project = currentProject();
  const description = project.robot.description?.robotDescriptionSha256;
  if (!description) throw new Error("The program is not pinned to a robot description");
  return {
    model_id: project.robot.model,
    robot_description_sha256: description,
    machine_planning_calibration_base64: project.robot.cell?.machinePlanningCalibrationBase64 ?? "",
  };
}

export interface SegmentPath {
  index: number;
  kind: string;
  sourceId: string;
  nodeId: string | null;
  samples: number;
  durationS: number;
  startMm: number[];
  endMm: number[];
  lengthMm: number;
  chordMm: number;
  /** Largest distance of the TCP from the straight start-end chord: ~0 for a MoveL. */
  maxChordDeviationMm: number;
  peakTcpSpeedMmS: number;
  boundsMinMm: number[];
  boundsMaxMm: number[];
  pointsMm: number[][];
}

/**
 * The planned TCP path: every sample of the kept trajectory (OLP's own
 * decoder) through FK at the model's TCP frame on the capture model's URDF.
 * Joints are matched by name, so the trajectory's nine axes drive whichever
 * of them the URDF has. A preview, never a command.
 */
export function plannedTcpPath(blob: Uint8Array, urdf: string, maxPointsPerSegment: number): { frame: string; dtS: number; segments: SegmentPath[] } {
  const project = currentProject();
  const descriptor = robotModelFor(project.robot.model);
  const trajectory = decodeDenseTrajectory(blob);
  const robot = parseUrdf(urdf);
  const frame = robot.links[descriptor.cameras.tcp.frame] ?? robot.joints[descriptor.cameras.tcp.frame];
  if (!frame) throw new Error("The robot description has no TCP frame");
  const order = project.robot.jointOrder;
  const driven = order.map((name, axis) => ({ name, axis })).filter(({ name }) => robot.joints[name]);
  for (const { name } of driven) robot.joints[name]!.ignoreLimits = true;
  const position = new THREE.Vector3();
  const segments = trajectory.segments.map((segment): SegmentPath => {
    const points = segment.samples.map((sample) => {
      for (const { name, axis } of driven) robot.setJointValue(name, sample.qRad[axis] ?? 0);
      frame.updateWorldMatrix(true, false);
      position.setFromMatrixPosition(frame.matrixWorld);
      return [position.x * 1000, position.y * 1000, position.z * 1000];
    });
    const start = points[0]!, end = points.at(-1)!;
    const chord = end.map((v, i) => v - start[i]!);
    const chordLength = Math.hypot(...chord);
    let length = 0, peak = 0, deviation = 0;
    points.forEach((point, i) => {
      if (i > 0) {
        const step = Math.hypot(...point.map((v, k) => v - points[i - 1]![k]!));
        length += step;
        const dt = segment.samples[i]!.tS - segment.samples[i - 1]!.tS;
        if (dt > 0) peak = Math.max(peak, step / dt);
      }
      const rel = point.map((v, k) => v - start[k]!);
      const along = chordLength > 0 ? rel.reduce((sum, v, k) => sum + v * chord[k]!, 0) / chordLength : 0;
      deviation = Math.max(deviation, Math.sqrt(Math.max(0, rel.reduce((sum, v) => sum + v * v, 0) - along * along)));
    });
    const stride = Math.max(1, Math.ceil(points.length / Math.max(2, maxPointsPerSegment)));
    const kept = points.filter((_, i) => i % stride === 0 || i === points.length - 1);
    return {
      index: segment.index, kind: segment.kind, sourceId: segment.sourceId,
      nodeId: denseSegmentNodeId(project.nodes, segment.kind as "freespace" | "weld" | "dwell", segment.sourceId),
      samples: points.length, durationS: segment.samples.at(-1)?.tS ?? 0,
      startMm: start, endMm: end, lengthMm: length, chordMm: chordLength, maxChordDeviationMm: deviation, peakTcpSpeedMmS: peak,
      boundsMinMm: [0, 1, 2].map((k) => Math.min(...points.map((p) => p[k]!))),
      boundsMaxMm: [0, 1, 2].map((k) => Math.max(...points.map((p) => p[k]!))),
      pointsMm: kept,
    };
  });
  return { frame: descriptor.cameras.tcp.frame, dtS: trajectory.dtS, segments };
}

/** The joint names a waypoint of the open program takes, in order. */
export function jointNames(): string[] {
  return [...robotModelFor(currentProject().robot.model).urdfJointNames];
}

/**
 * The editing lock's machine half, as the Dense panel's lockReason words it
 * (execution/densePanel.ts; a method of a DOM class, so its rule is restated
 * here): while the selected cell's session is loaded, staged, playing or
 * paused, main.ts editingLockReason refuses recording and every edit. Read
 * from the status olp-core's machine view last applied (observeStatus).
 */
export function editingLockReason(selected: boolean): string | null {
  const status = machineStatus();
  if (!selected || !status?.session) return null;
  const state = denseExecutionState(status);
  if (state === "playing" || state === "paused" || state === "staged" || state === "loaded") {
    return `Editing is locked while the joint daemon is ${state} on this program (STOP to release).`;
  }
  return null;
}

// ---------------------------------------------------------- rails parameters

/** One of the browser's two rails parameter windows as session state: the
 *  preset it compares against and its values. Never program state
 *  (rails/index.ts): an accepted weld keeps what it was searched with. */
export interface RailsParameterState { preset: string; values: Record<string, ParameterValue> }
export interface RailsSettingsState { torch: RailsParameterState; search: RailsParameterState }
export interface RailsSettingsChange { torchPreset?: string; torch?: Record<string, number>; searchPreset?: string; search?: Record<string, number> }

function parameterSide(
  scope: string, schema: ParameterSchema, shipped: readonly ParameterPreset[], what: string,
  current: RailsParameterState | undefined, presetId: string | undefined, values: Record<string, number> | undefined,
  constraints?: (values: Record<string, ParameterValue>) => string[],
) {
  // The shipped presets only: saved presets live in a browser's own store.
  const library = createPresetLibrary({ scope, schema, shipped, store: memoryPresetStore() });
  const wanted = presetId ?? current?.preset;
  const preset = wanted ? library.get(wanted) : library.fallback;
  if (!preset) throw new Error(`there is no ${what} preset ${wanted}; shipped: ${library.all().map((p) => p.id).join(", ")}`);
  const set = createParameterSet(schema, preset, { constraints });
  const apply = (entries: Record<string, ParameterValue>) => {
    for (const [key, value] of Object.entries(entries)) {
      const field = schema.find((candidate) => candidate.key === key);
      if (!field) throw new Error(`the ${what} has no parameter ${key}; it has ${schema.map((f) => f.key).join(", ")}`);
      const outcome = set.set(key, value);
      if (!outcome.ok) throw new Error(`${field.label} ${outcome.reason}`);
    }
  };
  if (!presetId && current) apply(current.values);
  if (values) apply(values);
  return set;
}

/**
 * The fitted torch and the angle search, as the rails parameter windows
 * hold them: a shipped preset, then values the parameter model validates
 * one by one (refused, never clamped). The torch's cross-field problems
 * (torchProblems) are advisory in the window; a request carrying them is
 * refused by the seam worker's tooling rules, so here they refuse at once.
 * `current` null is the windows' fallback: each scope's first shipped preset.
 */
export function railsSettings(current: RailsSettingsState | null, change: RailsSettingsChange = {}) {
  const torch = parameterSide(TORCH_SCOPE, TORCH_SCHEMA, SHIPPED_TORCH_PRESETS, "fitted torch", current?.torch, change.torchPreset, change.torch, torchProblems);
  const search = parameterSide(SEARCH_SCOPE, SEARCH_SCHEMA, SHIPPED_SEARCH_PRESETS, "angle search", current?.search, change.searchPreset, change.search);
  const problems = torch.problems();
  if (problems.length) throw new Error(`the fitted torch is not a gun the seam worker accepts: ${problems.join("; ")}`);
  const torchValues = torch.values();
  const searchValues = search.values();
  const describe = (set: typeof torch) => ({
    preset: { id: set.preset.id, label: set.preset.label },
    modified: set.modifiedKeys(),
    fields: set.states().map((state) => ({
      key: state.field.key, label: state.field.label, group: state.field.group ?? "",
      unit: state.field.kind === "number" ? state.field.unit ?? "" : "",
      min: state.field.kind === "number" ? state.field.min ?? null : null,
      max: state.field.kind === "number" ? state.field.max ?? null : null,
      value: state.value, presetValue: state.presetValue, modified: state.modified, help: state.field.help ?? "",
    })),
  });
  const request = torchRequest(torchValues, torch.preset);
  const defaultStandoffMm = Number(torchValues.default_standoff_mm);
  return {
    state: { torch: { preset: torch.preset.id, values: torchValues }, search: { preset: search.preset.id, values: searchValues } } as RailsSettingsState,
    torch: describe(torch),
    search: describe(search),
    presets: {
      torch: SHIPPED_TORCH_PRESETS.map((preset) => ({ id: preset.id, label: preset.label, values: { ...preset.values } })),
      search: SHIPPED_SEARCH_PRESETS.map((preset) => ({ id: preset.id, label: preset.label, values: { ...preset.values } })),
    },
    torchRequest: request,
    // rails/index.ts search(): values, never the preset's id.
    searchKnobs: {
      work_deviation_deg: Number(searchValues.work_deviation_deg),
      travel_deviation_deg: Number(searchValues.travel_deviation_deg),
      standoff_cost_deg_per_mm: Number(searchValues.standoff_cost_deg_per_mm),
    },
    defaultStandoffMm,
    // rails/index.ts planRequestBody: tooling = {...torch(), default_standoff_mm}.
    tooling: { ...request, default_standoff_mm: defaultStandoffMm },
  };
}

// ---------------------------------------------------------- angle profiles

/** The parameter dialog's profile editor range for torch angles
 *  (node-editor.ts profileEditor: createSplineEditor min -90, max 90). */
const PROFILE_MIN_DEG = -90;
const PROFILE_MAX_DEG = 90;
/** scalar-function.ts normalize's minimum knot spacing. */
const PROFILE_MIN_GAP_S = 1e-4;

/**
 * A rails weld's work or travel angle as the dialog's spline editor writes
 * it: control points [s, degrees] spanning s=0..1, through OLP's fromPoints
 * (a flat profile is written back as a constant). The editor keeps its ends
 * at s=0 and s=1 and its points apart, and edits only constant and
 * piecewise-linear profiles; a bspline or samples profile is shown and left
 * alone. The caller writes the answer through olp-core edit (applyEdit's
 * profile kind, then documentProblems).
 */
export function weldProfile(id: string, axis: "work" | "travel", points: { s: number; deg: number }[]): { path: string; fn: ScalarFunction } {
  const node = nodeById(currentProject().nodes, id);
  if (!node) throw new Error("The node is no longer in the program");
  if (!isWeld(node) || node.geometry.kind !== "seam") {
    throw new Error(`${id} is not a rails (seam) weld: only a seam weld's torch angles are profiles; a waypoint weld's angles are its poses`);
  }
  const path = `geometry.orientation.${axis}_angle_deg`;
  const current = dig(node, path) as ScalarFunction | undefined;
  if (current && !isEditable(current)) throw new Error(`${axis} angle is a ${current.kind} profile · not editable here`);
  if (points.length < 2) throw new Error("a profile needs at least two points, at s=0 and s=1");
  if (points[0]!.s !== 0 || points.at(-1)!.s !== 1) throw new Error("a profile runs from s=0 to s=1: the first point is at s=0 and the last at s=1");
  points.forEach((point, index) => {
    if (!Number.isFinite(point.deg) || point.deg < PROFILE_MIN_DEG || point.deg > PROFILE_MAX_DEG) {
      throw new Error(`point ${index} is ${point.deg}°; the profile editor's range is ${PROFILE_MIN_DEG}° to ${PROFILE_MAX_DEG}°`);
    }
    if (index > 0 && !(point.s - points[index - 1]!.s >= PROFILE_MIN_GAP_S)) {
      throw new Error(`point ${index} (s=${point.s}) must come after point ${index - 1} (s=${points[index - 1]!.s}) by at least ${PROFILE_MIN_GAP_S}`);
    }
  });
  return { path, fn: fromPoints(points.map((point) => ({ s: point.s, value: point.deg }))) };
}

// ---------------------------------------------------------- workcell fixtures

/** editWorkcellFixture's bounds (mm, degrees): the Stand card clamps to
 *  them; a value outside is refused here instead of silently clamped. */
const FIXTURE_BOUNDS: Record<WorkcellFixtureField, [number, number, string]> = {
  sx: [1, 5000, "mm"], sy: [1, 5000, "mm"], sz: [1, 5000, "mm"],
  x: [-5000, 5000, "mm"], y: [-5000, 5000, "mm"], z: [-5000, 5000, "mm"],
  rx: [-360, 360, "°"], ry: [-360, 360, "°"], rz: [-360, 360, "°"],
};

const deg = (rad: number) => (rad * 180) / Math.PI;

function fixtureView(fixture: WorkcellFixture, kind: "fixture" | "part") {
  return {
    id: fixture.id, name: fixture.name, kind, origin: fixture.origin,
    shape: fixture.shape.kind === "box"
      ? { kind: "box" as const, sizeMm: fixture.shape.sizeM.map((v) => v * 1000) }
      : { kind: "step" as const, filename: fixture.shape.filename, fingerprint: fixture.shape.fingerprint },
    positionMm: fixture.translationM.map((v) => v * 1000),
    rotationDeg: fixture.rotationRpyRad.map(deg),
    label: fixtureSublabel(fixture),
  };
}

/** Everything a plan clears besides the robot and the active part: the
 *  program's workcell fixtures and its other parts (environmentFixtures),
 *  in the cell's world frame; and the cell's own default for comparison. */
export function fixturesView() {
  const project = currentProject();
  const own = new Set(workcellFixtures(project).map((fixture) => fixture.id));
  return {
    fixtures: environmentFixtures(project).map((fixture) => fixtureView(fixture, own.has(fixture.id) ? "fixture" : "part")),
    cellDefault: defaultWorkcellFixtures(project.robot.model).map((fixture) => fixtureView(fixture, "fixture")),
  };
}

/** The STEP bodies a plan must carry as meshes (workcellFixturesWire refuses
 *  any it is not given), by lowercase fingerprint. */
export function fixtureMeshNeeds(): { id: string; name: string; filename: string; fingerprint: string }[] {
  return environmentFixtures(currentProject()).flatMap((fixture) => fixture.shape.kind === "step"
    ? [{ id: fixture.id, name: fixture.name, filename: fixture.shape.filename, fingerprint: fixture.shape.fingerprint.toLowerCase() }]
    : []);
}

export type FixtureEdit =
  | { action: "add_stand" }
  | { action: "edit"; id: string; fields: Partial<Record<WorkcellFixtureField, number>> }
  | { action: "replace_with_step"; id: string; filename: string; fingerprint: string }
  | { action: "add_step_fixture"; filename: string; fingerprint: string }
  | { action: "cell_default"; id: string }
  | { action: "remove"; id: string };

/**
 * The Stand card's and the workspace menu's fixture actions (main.ts
 * updateStandField, importStandStep, resetStandToCellDefault, removeStand,
 * addStand, importStepBytes kind "fixture"), each one "workcell" revision.
 * A STEP fixture's mesh is the caller's: tessellated before it is named here.
 */
export function editFixture(edit: FixtureEdit): { id: string; changed: boolean } {
  const project = currentProject();
  const commit = (apply: (draft: OfflineProgrammingProject) => void) => {
    commitProject(reviseProject(project, "workcell", (draft) => {
      apply(draft);
      deriveRows(draft, "workcell", null);
    }));
  };
  const existing = (id: string): WorkcellFixture => {
    const fixture = fixtureById(project, id);
    if (!fixture) throw new Error(`no workcell fixture ${id}; fixtures: ${workcellFixtures(project).map((f) => f.id).join(", ") || "none"}`);
    return fixture;
  };
  const replace = (id: string, next: WorkcellFixture) => commit((draft) => {
    const index = draft.workcellFixtures.findIndex((fixture) => fixture.id === id);
    if (index >= 0) draft.workcellFixtures[index] = next;
  });
  switch (edit.action) {
    case "add_stand": {
      if (fixtureById(project, STAND_ID)) return { id: STAND_ID, changed: false };
      const fixture = newBoxFixture();
      commit((draft) => { draft.workcellFixtures.push(fixture); });
      return { id: fixture.id, changed: true };
    }
    case "edit": {
      const current = existing(edit.id);
      let next = current;
      const fields = Object.entries(edit.fields) as [WorkcellFixtureField, number][];
      if (!fields.length) throw new Error("name at least one field: sx, sy, sz (mm), x, y, z (mm), rx, ry, rz (degrees)");
      for (const [field, value] of fields) {
        const bounds = FIXTURE_BOUNDS[field];
        if (!bounds) throw new Error(`${field} is not a fixture field: sx, sy, sz, x, y, z, rx, ry, rz`);
        const [lo, hi, unit] = bounds;
        if (!Number.isFinite(value) || value < lo || value > hi) throw new Error(`${field} ${value} ${unit} is outside the fixture card's ${lo} to ${hi} ${unit}`);
        const edited = editWorkcellFixture(next, field, value);
        if (edited === next) throw new Error(`${current.name} is a STEP fixture: it has no box size to set (${field})`);
        next = edited;
      }
      replace(edit.id, next);
      return { id: edit.id, changed: true };
    }
    case "replace_with_step": {
      const current = existing(edit.id);
      replace(edit.id, {
        ...current,
        name: current.origin === "cell-default" ? "Stand" : current.name,
        shape: { kind: "step", fingerprint: edit.fingerprint.toLowerCase(), filename: edit.filename },
        origin: "authored",
      });
      return { id: edit.id, changed: true };
    }
    case "add_step_fixture": {
      const fixture = newStepFixture(workcellFixtures(project), { filename: edit.filename, fingerprint: edit.fingerprint.toLowerCase() });
      commit((draft) => { draft.workcellFixtures.push(fixture); });
      return { id: fixture.id, changed: true };
    }
    case "cell_default": {
      existing(edit.id);
      const fallback = defaultWorkcellFixtures(project.robot.model).find((fixture) => fixture.id === edit.id)
        ?? { ...newBoxFixture(edit.id), origin: "cell-default" as const };
      replace(edit.id, fallback);
      return { id: edit.id, changed: true };
    }
    case "remove": {
      existing(edit.id);
      commit((draft) => { draft.workcellFixtures = draft.workcellFixtures.filter((fixture) => fixture.id !== edit.id); });
      return { id: edit.id, changed: true };
    }
  }
}

/** What the CAD client posts through: the OLP route, answered as HTTP. */
export type CadPost = (body: Record<string, unknown>) => Promise<{ status: number; body: unknown }>;

/**
 * A STEP's surface triangles through OLP's own CAD client (cad/client.ts
 * tessellate: its deflection and triangle defaults, its refusals, and
 * normalizeWorkpieceMesh), posted by the caller so the request carries
 * this server's client identity.
 */
export function tessellateStep(post: CadPost, filename: string, bytes: Uint8Array): Promise<CadWorkpieceMesh> {
  const client = new HttpCadClient({
    endpoint: CAD_API_PATH,
    fetch: (async (_input: unknown, init?: { body?: unknown }) => {
      const reply = await post(JSON.parse(String(init?.body ?? "{}")) as Record<string, unknown>);
      return new Response(JSON.stringify(reply.body ?? null), { status: reply.status, headers: { "Content-Type": "application/json" } });
    }) as unknown as typeof fetch,
  });
  return client.tessellate({ source: { kind: "bytes", filename, bytes } });
}

// ---------------------------------------------------------- parts

/** The program's parts (projectWorkpieces), which one is active for editing
 *  and planning, and where each sits in the work frame. */
export function partsView() {
  const project = currentProject();
  const active = hasWorkpieceGeometry(project) ? activeWorkpieceId(project) : null;
  return projectWorkpieces(project).map((part) => {
    const placement = workpiecePlacement(workpieceProject(project, part));
    return {
      id: part.id, filename: part.cad.filename, fingerprint: part.cad.fingerprint, active: part.id === active,
      welds: part.nodes.filter((node) => node.op === "weld").length, rows: part.nodes.length,
      placement: placement ? {
        offsetFromWorkFrameMm: placement.offsetFromWorkFrameM.map((v) => v * 1000),
        rotationRpyDeg: placement.rotationRpyRad.map(deg),
      } : null,
    };
  });
}

/** The tree's part selection (main.ts selectWorkpiece): another part
 *  becomes the one edited and planned; the active one keeps its welds in
 *  its own record. One "cad" revision. The caller holds the part's STEP. */
export function switchPart(id: string): boolean {
  const project = currentProject();
  const parts = projectWorkpieces(project);
  if (!parts.some((part) => part.id === id)) throw new Error(`no part ${id}; parts: ${parts.map((part) => `${part.id} (${part.cad.filename})`).join(", ") || "none"}`);
  if (hasWorkpieceGeometry(project) && activeWorkpieceId(project) === id) return false;
  commitProject(reviseProject(project, "cad", (draft) => {
    activateWorkpiece(draft, id);
    deriveRows(draft, "cad", null);
  }));
  select(null);
  return true;
}

/** Delete part (main.ts deletePart): the active part, its edges, seams,
 *  welds and rows leave the program; the other parts stay. */
export function removePart(): string {
  const project = currentProject();
  if (!hasWorkpieceGeometry(project)) throw new Error("The program has no active part to remove");
  const filename = project.cad.filename || "part";
  commitProject(reviseProject(project, "cad", (draft) => {
    forgetActiveWorkpiece(draft);
    draft.cad = { kind: "step", filename: "", fingerprint: "", sourceUnit: "mm", topologyModelId: "" };
    draft.edgeCatalog = [];
    draft.seams = [];
    draft.nodes = [];
    draft.sequence = [];
    draft.rows = [];
    draft.fixtures = [];
    draft.simulation.selectedRowId = null;
    deriveRows(draft, "cad", null);
  }));
  select(null);
  return filename;
}

// ---------------------------------------------------------- waypoint welds

/** A hand-placed torch pose: work point in the part frame (mm), URDF roll,
 *  pitch, yaw (degrees, tool +Z into the work), continuity, standoff. */
export interface WaypointInput { xyzMm: number[]; rpyDeg: number[]; continuity?: Continuity; standoffMm?: number }

/** main.ts waypointSession's newWeldProcess: the program's process defaults. */
function newWeldProcess() {
  const process = currentProject().processParameters[0];
  const torch = processTorchDefaults(process);
  return {
    travelSpeedMmS: (process?.travelSpeedMps ?? DEFAULT_TRAVEL_SPEED_MPS) * 1000,
    weave: torch.pattern === "none"
      ? { shape: "none" }
      : { shape: torch.pattern, amplitude_mm: { kind: "constant" as const, value: torch.amplitudeMm }, frequency_hz: { kind: "constant" as const, value: torch.frequencyHz } },
  } as ReturnType<WaypointHost["newWeldProcess"]>;
}

/** main.ts WaypointWeldsCard defaultStandoffMm. */
function defaultStandoffMm(): number {
  return currentProject().processParameters[0]?.torchStandoffMm ?? DEFAULT_WAYPOINT_STANDOFF_MM;
}

function waypointOf(input: WaypointInput, fallback: { continuity: Continuity; standoffMm: number }, id = newId("wp")): Waypoint {
  const standoffMm = input.standoffMm ?? fallback.standoffMm;
  // The tool clamps to this range (clampStandoffMm); a value outside is refused here.
  if (!(standoffMm >= STANDOFF_MIN_MM && standoffMm <= STANDOFF_MAX_MM)) {
    throw new Error(`standoff ${standoffMm} mm is outside ${STANDOFF_MIN_MM}–${STANDOFF_MAX_MM} mm, the range a waypoint accepts`);
  }
  if (input.xyzMm.length !== 3 || input.rpyDeg.length !== 3 || ![...input.xyzMm, ...input.rpyDeg].every(Number.isFinite)) {
    throw new Error("a waypoint needs three finite xyz_mm and three finite rpy_deg");
  }
  return {
    id,
    positionM: input.xyzMm.map((v) => v / 1000) as V3,
    quaternion: rpyToQuat(input.rpyDeg.map((v) => (v * Math.PI) / 180) as V3),
    continuity: input.continuity ?? fallback.continuity,
    standoffMm,
  };
}

/** The Waypoint weld tool's host (main.ts), staging its edits: committed as
 *  one undo step only when OLP's document validator finds nothing new
 *  wrong, so a weld the contract refuses never becomes a revision. */
function stagedWaypoints() {
  let staged: OfflineProgrammingProject | null = null;
  const host: WaypointHost = {
    nodes: () => (staged ?? currentProject()).nodes,
    editNodes: (_message, change) => {
      staged = reviseProject(staged ?? currentProject(), "nodes", (draft) => {
        draft.nodes = change(draft.nodes);
        deriveRows(draft, "nodes", null);
      });
    },
    newWeldProcess,
  };
  return {
    session: new WaypointSession(host),
    host,
    commit(unchanged: string): void {
      if (!staged) throw new Error(unchanged);
      const before = new Set(documentProblems(currentProject()));
      const added = documentProblems(staged).filter((problem) => !before.has(problem));
      if (added.length) throw new Error(added[0]);
      commitProject(staged);
    },
  };
}

function requireWaypointWeldTools(): void {
  const project = currentProject();
  // main.ts renderWeldToolActions: the three hand-placed tools' gate.
  if (!hasWorkpieceGeometry(project) || workpiecePlacement(project) === null) throw new Error("Import a part to place welds on");
  if (!project.railsAuthoring) throw new Error("Rails authoring only");
}

/** A new waypoint, line or arc weld from these poses (WaypointSession
 *  importWeld: nextWeldLabel, manualPoseWeldNode with the program's process
 *  defaults, insertWeld's approach and retract, the start link followed). */
export function addWaypointWeld(tool: WeldTool, points: WaypointInput[], startFrom: string | null): string {
  requireWaypointWeldTools();
  if (tool === "line" && points.length !== 2) throw new Error("a line weld has exactly two points");
  if (tool === "arc" && points.length !== 3) throw new Error("an arc weld has exactly three points");
  if (points.length < 2) throw new Error("a waypoint weld needs at least two points");
  if (startFrom && !startCandidates(currentProject().nodes, "").some((node) => node.id === startFrom)) {
    throw new Error(`${startFrom} is not a waypoint, line or arc weld this weld could start from`);
  }
  const fallback = { continuity: (tool === "waypoint" ? "c1" : "c0") as Continuity, standoffMm: defaultStandoffMm() };
  const waypoints = points.map((point) => waypointOf(point, fallback));
  const staged = stagedWaypoints();
  staged.session.importWeld(waypoints, { tool, startFrom });
  const id = staged.session.selection.weldId;
  staged.commit("the weld was not added");
  select(id);
  return id!;
}

export type WaypointEdit =
  | { op: "set_points"; points: WaypointInput[] }
  | { op: "add_point"; afterIndex?: number; point: WaypointInput }
  | { op: "move_point"; index: number; xyzMm?: number[]; rpyDeg?: number[] }
  | { op: "reorder_point"; index: number; by: -1 | 1 }
  | { op: "remove_point"; index: number }
  | { op: "set_standoff"; index?: number; standoffMm: number }
  | { op: "set_continuity"; index?: number; continuity: Continuity }
  | { op: "set_start_from"; startFrom: string | null };

function manualWeld(id: string) {
  const node = nodeById(currentProject().nodes, id);
  if (!node) throw new Error("The node is no longer in the program");
  if (!isManualPoseWeld(node)) throw new Error(`${id} is not a waypoint, line or arc weld (manual_pose_wp)`);
  return node;
}

/** One Waypoint weld card edit, through OLP's WaypointSession. */
export function editWaypointWeld(id: string, edit: WaypointEdit): void {
  requireWaypointWeldTools();
  const weld = waypointWeldOf(manualWeld(id));
  const staged = stagedWaypoints();
  const session = staged.session;
  const count = weld.waypoints.length;
  const at = (index: number) => {
    const point = weld.waypoints[index];
    if (!point) throw new Error(`${weld.label} has ${count} points (index 0 to ${count - 1}); there is no point ${index}`);
    return point;
  };
  const fixed = WaypointSession.fixedCount(weld.tool)
    ? `${weld.label} is ${weld.tool === "line" ? "a line" : "an arc"}: a line keeps two points and an arc three; its points are moved, never added or removed`
    : "";
  const standoffOk = (value: number) => waypointOf({ xyzMm: [0, 0, 0], rpyDeg: [0, 0, 0], standoffMm: value }, { continuity: "c0", standoffMm: value });
  switch (edit.op) {
    case "set_points": {
      if (fixed && edit.points.length !== count) throw new Error(fixed);
      const points = edit.points.map((point, index) => waypointOf(point, {
        continuity: weld.waypoints[index]?.continuity ?? (weld.tool === "waypoint" ? "c1" : "c0"),
        standoffMm: weld.waypoints[index]?.standoffMm ?? defaultStandoffMm(),
      }, weld.waypoints[index]?.id ?? newId("wp")));
      // WaypointSession #write, for every point at once.
      staged.host.editNodes(`${weld.label}: points set`, (nodes) => resolveStartLinks(nodes.map((node) =>
        node.id === id && isManualPoseWeld(node) ? withWaypoints(node, points) : node)));
      break;
    }
    case "add_point": {
      if (fixed) throw new Error(fixed);
      const after = edit.afterIndex ?? count - 1;
      const base = at(after);
      const point = waypointOf(edit.point, { continuity: base.continuity, standoffMm: base.standoffMm });
      session.select({ weldId: id, waypointId: base.id });
      session.addWaypoint([0, 0, 0], point.standoffMm);
      const added = session.selection.waypointId!;
      session.setPose(added, point.positionM, point.quaternion);
      session.setStandoff(added, point.standoffMm);
      session.setContinuity(added, point.continuity);
      break;
    }
    case "move_point": {
      const point = at(edit.index);
      if (session.isTied(point.id) && edit.xyzMm) {
        throw new Error(`point 0 of ${weld.label} starts at the end of ${weld.startFrom}: only its orientation can change (set_start_from null unties it)`);
      }
      const moved = waypointOf({ xyzMm: edit.xyzMm ?? point.positionM.map((v) => v * 1000), rpyDeg: edit.rpyDeg ?? quatToRpy(point.quaternion).map(deg) },
        { continuity: point.continuity, standoffMm: point.standoffMm });
      session.setPose(point.id, edit.xyzMm ? moved.positionM : point.positionM, edit.rpyDeg ? moved.quaternion : point.quaternion);
      break;
    }
    case "reorder_point": {
      const point = at(edit.index);
      if (edit.index + edit.by < 0 || edit.index + edit.by >= count) throw new Error(`point ${edit.index} cannot move ${edit.by < 0 ? "earlier" : "later"}: it is already ${edit.by < 0 ? "first" : "last"}`);
      session.moveWaypoint(point.id, edit.by);
      break;
    }
    case "remove_point": {
      if (fixed) throw new Error(fixed);
      session.removeWaypoint(at(edit.index).id);
      break;
    }
    case "set_standoff": {
      standoffOk(edit.standoffMm);
      for (const point of edit.index === undefined ? weld.waypoints : [at(edit.index)]) session.setStandoff(point.id, edit.standoffMm);
      break;
    }
    case "set_continuity": {
      if (edit.index === undefined) session.setWeldContinuity(id, edit.continuity);
      else session.setContinuity(at(edit.index).id, edit.continuity);
      break;
    }
    case "set_start_from": {
      if (edit.startFrom && !session.startCandidates(id).some((candidate) => candidate.id === edit.startFrom)) {
        throw new Error(`${edit.startFrom} cannot be where ${weld.label} starts: it is not another waypoint, line or arc weld, or it starts from ${weld.label} itself`);
      }
      session.setStartFrom(id, edit.startFrom);
      break;
    }
  }
  staged.commit(`nothing to change on ${weld.label}`);
  select(id);
}

/** A waypoint weld as its card shows it: each point in the part frame (mm,
 *  degrees), its continuity, standoff and normalised arc length. */
export function waypointWeldView(id: string) {
  const node = manualWeld(id);
  const weld = waypointWeldOf(node);
  const fractions = waypointArcFractions(weld.waypoints, shapeOf(weld.tool));
  const speed = node.travel_speed_mm_s;
  return {
    id: weld.id, label: weld.label, tool: weld.tool, enabled: weld.enabled, startFrom: weld.startFrom,
    travelSpeedMmS: speed.kind === "constant" ? speed.value : null,
    points: weld.waypoints.map((point, index) => ({
      index, id: point.id, xyzMm: point.positionM.map((v) => v * 1000), rpyDeg: quatToRpy(point.quaternion).map(deg),
      continuity: point.continuity, standoffMm: point.standoffMm, s: fractions[index] ?? 0,
      tied: index === 0 && Boolean(weld.startFrom),
    })),
  };
}

/** Whether a node id names a waypoint weld (for the spreadsheet's rendering). */
export function isWaypointWeld(id: string): boolean {
  const node = nodeById(currentProject().nodes, id);
  return Boolean(node && isManualPoseWeld(node as ProgramV2Node));
}
