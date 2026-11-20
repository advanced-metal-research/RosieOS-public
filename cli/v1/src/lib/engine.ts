// The bundled OLP program engine (core/entry.ts -> dist/core/olp-core.mjs),
// typed here for the calls this server makes. The shapes are OLP's; the
// engine itself is typechecked against OLP's sources (`npm run typecheck`).
import { existsSync } from "node:fs";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

export interface Row {
  id: string; number: number; op: string; family: string; enabled: boolean; toggleable: boolean;
  label: string; geometry: string; process: string; freespace: string; status: string; statusClass: string;
  comment: string; upBlocked: string | null; downBlocked: string | null; deleteBlocked: string | null;
  cells: Record<string, number | string>;
}

export interface ProgramView {
  name: string; revision: number; programState: string; railsAuthoring: boolean; selected: string | null;
  canUndo: boolean; canRedo: boolean; speedScale: number; rows: Row[]; problems: string[];
  model: string; description: string | null;
}

export interface FieldSpec {
  path: string; label: string; kind: string; unit?: string; hint?: string; min?: number; max?: number; step?: number;
  options?: unknown; readOnly?: string; value: unknown;
}

export interface FieldGroups {
  op: string; label: string;
  groups: { id: string; group: string; note?: string; fields: FieldSpec[] }[];
}

export interface PartView {
  railsAuthoring: boolean;
  cad: { filename: string; fingerprint: string } | null;
  parts: { id: string; filename: string | null; fingerprint: string | null }[];
  activeWorkpieceId: string | null;
  placement: { offsetFromWorkFrameMm: number[]; rotationRpyDeg: number[] } | null;
  hasGeometry: boolean;
  edges: number;
}

export interface PlanRecordView {
  trajectoryDigest: string; planId: string; programId: string; programDigest: string;
  manifestRevision: number; planRevision: number; createdAt: string; totalSampleCount: number; totalDurationS: number;
  segments: { index: number; kind: string; sourceId: string; sampleCount: number; durationS: number }[];
  robot: unknown; current: boolean;
}

/** program-v2/capture.ts MeasuredPose: the machine's pose and the identity it was observed under. */
export interface MeasuredPose {
  pose: { modelId: string; jointOrder: readonly string[]; jointsRad: readonly number[]; source: string; observedAtMs?: number };
  settings: Record<string, unknown>;
  cellId: string;
}

export interface SegmentPath {
  index: number; kind: string; sourceId: string; nodeId: string | null; samples: number; durationS: number;
  startMm: number[]; endMm: number[]; lengthMm: number; chordMm: number; maxChordDeviationMm: number; peakTcpSpeedMmS: number;
  boundsMinMm: number[]; boundsMaxMm: number[]; pointsMm: number[][];
}

/** rails/client.ts shapes, as the seam worker answers them. */
export interface RailsSeam { id: string; label: string; weld_joint_ref: string; span: { start_s: number; end_s: number }; direction: string; [key: string]: unknown }
export interface RailsJoint { id: string; weld_line: { closed: boolean; length_m: number; [key: string]: unknown }; [key: string]: unknown }
export interface RailsOptimisePlan { clear_fraction: number; direction?: string; direction_changed?: boolean; standoff_mm: number; seam_length_mm: number; [key: string]: unknown }
export interface RailsClearance { seam_length_m: number; [key: string]: unknown }
export interface CoverageOffer { state: "clear" | "partial" | "blocked"; seamLengthM: number; usableLengthM: number; blockedLengthM: number; usableRuns: number[][]; split: { start_s: number; end_s: number }[]; trim: unknown }
export interface CoverageSummary { seamMm: number; clearMm: number; trimMm: number; trimSpan: unknown; [key: string]: unknown }
export interface SeamPlan { plan: RailsOptimisePlan; payload: Record<string, unknown>; clearance: RailsClearance; report: CoverageOffer }
export interface RunPiece { id: string; span: { start_s: number; end_s: number }; payload: Record<string, unknown>; clearance: RailsClearance; summary: CoverageSummary; plan: RailsOptimisePlan; note: string }
export interface Decision { state: string; span: unknown; note: string; payload: unknown; node: { id: string; [key: string]: unknown } }
export interface SeamMeasurer {
  planTorchPath(step: Uint8Array, body: Record<string, unknown>): Promise<{ direction: string; plan: RailsOptimisePlan }>;
  checkTorchFits(step: Uint8Array, body: Record<string, unknown>): Promise<{ clearance: RailsClearance }>;
}
export interface DecidableEntry { id: string; seam: RailsSeam; joint: RailsJoint; span: { start_s: number; end_s: number }; plan: RailsOptimisePlan | null; summary: CoverageSummary | null; payload: Record<string, unknown> | null }

/** rails/parameters.ts as session state, and what the seam worker and the planner are sent. */
export interface RailsParameterState { preset: string; values: Record<string, number | string | boolean> }
export interface RailsSettingsState { torch: RailsParameterState; search: RailsParameterState }
export interface RailsSettingsChange { torchPreset?: string; torch?: Record<string, number>; searchPreset?: string; search?: Record<string, number> }
export interface RailsParameterView {
  preset: { id: string; label: string };
  modified: string[];
  fields: { key: string; label: string; group: string; unit: string; min: number | null; max: number | null; value: unknown; presetValue: unknown; modified: boolean; help: string }[];
}
export interface RailsSettings {
  state: RailsSettingsState;
  torch: RailsParameterView;
  search: RailsParameterView;
  presets: { torch: { id: string; label: string; values: Record<string, unknown> }[]; search: { id: string; label: string; values: Record<string, unknown> }[] };
  torchRequest: Record<string, unknown>;
  searchKnobs: Record<string, number>;
  defaultStandoffMm: number;
  tooling: Record<string, unknown>;
}

/** cad/contracts.ts CadWorkpieceMesh, the fields this server reads. */
export interface WorkpieceMesh { fingerprint: string; filename: string; triangleCount: number; vertexCount: number; truncated: boolean }

export interface FixtureView {
  id: string; name: string; kind: "fixture" | "part"; origin: string;
  shape: { kind: "box"; sizeMm: number[] } | { kind: "step"; filename: string; fingerprint: string };
  positionMm: number[]; rotationDeg: number[]; label: string;
}
export type FixtureField = "sx" | "sy" | "sz" | "x" | "y" | "z" | "rx" | "ry" | "rz";
export type FixtureEdit =
  | { action: "add_stand" }
  | { action: "edit"; id: string; fields: Partial<Record<FixtureField, number>> }
  | { action: "replace_with_step"; id: string; filename: string; fingerprint: string }
  | { action: "add_step_fixture"; filename: string; fingerprint: string }
  | { action: "cell_default"; id: string }
  | { action: "remove"; id: string };

export interface PartRecordView {
  id: string; filename: string; fingerprint: string; active: boolean; welds: number; rows: number;
  placement: { offsetFromWorkFrameMm: number[]; rotationRpyDeg: number[] } | null;
}

export type Continuity = "c0" | "c1" | "c2";
export type WeldTool = "waypoint" | "line" | "arc";
export interface WaypointInput { xyzMm: number[]; rpyDeg: number[]; continuity?: Continuity; standoffMm?: number }
export type WaypointEdit =
  | { op: "set_points"; points: WaypointInput[] }
  | { op: "add_point"; afterIndex?: number; point: WaypointInput }
  | { op: "move_point"; index: number; xyzMm?: number[]; rpyDeg?: number[] }
  | { op: "reorder_point"; index: number; by: -1 | 1 }
  | { op: "remove_point"; index: number }
  | { op: "set_standoff"; index?: number; standoffMm: number }
  | { op: "set_continuity"; index?: number; continuity: Continuity }
  | { op: "set_start_from"; startFrom: string | null };
export interface WaypointWeldView {
  id: string; label: string; tool: WeldTool; enabled: boolean; startFrom: string | null; travelSpeedMmS: number | null;
  points: { index: number; id: string; xyzMm: number[]; rpyDeg: number[]; continuity: Continuity; standoffMm: number; s: number; tied: boolean }[];
}

export interface Engine {
  setRobots(catalogue: unknown, connected: { modelId: string; robotDescriptionSha256: string } | null): void;
  startupModel(lastConnected: string): string;
  newProject(name: string, robotModel: string): void;
  openProject(text: string): void;
  openCellProject(document: unknown): { identity: { id: string; name: string; revision: number } };
  copyOpenProject(): { identity: { id: string; name: string } };
  exportProject(): string;
  projectIdentity(): string;
  modelMismatch(cellModel: string): string;
  view(): ProgramView;
  fields(id: string): FieldGroups;
  edit(id: string, path: string, value: unknown): void;
  select(id: string | null): void;
  addEvent(kind: "dwell" | "io" | "home"): string;
  remove(id: string): void;
  move(id: string, delta: -1 | 1): void;
  enable(id: string, enabled: boolean): void;
  setSpeedScale(percent: number): void;
  undo(): void;
  redo(): void;
  partView(): PartView;
  importPart(topologyWire: unknown, filename: string): { fingerprint: string; edges: number };
  placePart(offsetMm: number[], rotationRpyDeg: number[]): void;
  railsWeld(id: string): unknown;
  upsertWeld(node: unknown): void;
  planRequest(stepBase64: string, stepFilename: string, options: {
    connected: boolean; fromProgramStart: boolean; closeLoop: boolean; freeSpaceBackend: string;
    tooling: Record<string, unknown>; meshes: ReadonlyMap<string, WorkpieceMesh>;
  }): Promise<{ revision: number; projectId: string; body: Record<string, unknown> }>;
  applyPlan(result: unknown, planRevision: number, planProjectId: string): string;
  planRecord(): PlanRecordView | null;
  loadBody(): Record<string, unknown>;
  captureModelRequest(jointsRad: readonly number[]): Record<string, string>;
  recordPreview(mode: "new" | "reteach" | "via", jointsRad: readonly number[], anchorId: string | null, model: { urdf: string; identity: unknown }, newId: string): string;
  jointNames(): string[];
  /** olp-core machine.ts: OLP replies in, the Deck's teach and settings rules out. */
  observeCapabilities(payload: unknown): void;
  observeStatus(payload: unknown, busy: boolean): void;
  teachObservation(input: { selected: boolean; busy: boolean; statusAgeMs: number | null; cellModel: string; cellId: string }): MeasuredPose;
  fetchedCellSettings(selected: boolean, statusAgeMs: number | null, payload: unknown): Record<string, unknown> | null;
  /** olp-core session.ts: the measured capture and the program's robot settings. */
  captureStart(mode: "new" | "reteach" | "via", measured: MeasuredPose): { description: string; machineBound: boolean; modelId: string; revision: number };
  captureRecord(mode: "new" | "reteach" | "via", measured: MeasuredPose, revision: number, description: string, calibration: Record<string, unknown> | null, identity: unknown, tcp: number[], newId: string): string;
  tcpPose(urdf: string, snapshot: MeasuredPose["pose"]): number[];
  robotSettings(): { model: string; pinned: string | null; cell: Record<string, unknown> | null; drift: unknown; review: { title: string; explanation: string; rows: unknown[] } | null; adopted: unknown };
  pullCellSettings(settings: Record<string, unknown>): boolean;
  adoptDescription(): void;
  programModelRequest(): Record<string, string>;
  plannedTcpPath(blob: Uint8Array, urdf: string, maxPointsPerSegment: number): { frame: string; dtS: number; segments: SegmentPath[] };
  searchSeam(measurer: SeamMeasurer, step: Uint8Array, seam: RailsSeam, joint: RailsJoint, inputs: { torch: unknown; search: unknown; defaultStandoffMm: number }): Promise<SeamPlan | null>;
  measureRunPiece(measurer: SeamMeasurer, step: Uint8Array, seam: RailsSeam, joint: RailsJoint, cached: SeamPlan, k: number, torch: unknown): Promise<RunPiece | null>;
  decideWeld(entry: DecidableEntry, choice: "trim" | "accept", sha256: string, previous?: unknown): Decision | null;
  narrowsToRun(report: CoverageOffer): boolean;
  summarise(clearance: RailsClearance, span: { start_s: number; end_s: number }): CoverageSummary;
  shippedTorch(): Record<string, unknown>;
  shippedSearchKnobs(): Record<string, number>;
  shippedTooling(): Record<string, unknown>;
  /** The selected cell's editing lock in OLP's words, from the status last observed; null when unlocked. */
  editingLockReason(selected: boolean): string | null;
  railsSettings(current: RailsSettingsState | null, change?: RailsSettingsChange): RailsSettings;
  weldProfile(id: string, axis: "work" | "travel", points: { s: number; deg: number }[]): { path: string; fn: Record<string, unknown> };
  fixturesView(): { fixtures: FixtureView[]; cellDefault: FixtureView[] };
  fixtureMeshNeeds(): { id: string; name: string; filename: string; fingerprint: string }[];
  editFixture(edit: FixtureEdit): { id: string; changed: boolean };
  tessellateStep(post: (body: Record<string, unknown>) => Promise<{ status: number; body: unknown }>, filename: string, bytes: Uint8Array): Promise<WorkpieceMesh>;
  partsView(): PartRecordView[];
  switchPart(id: string): boolean;
  removePart(): string;
  addWaypointWeld(tool: WeldTool, points: WaypointInput[], startFrom: string | null): string;
  editWaypointWeld(id: string, edit: WaypointEdit): void;
  waypointWeldView(id: string): WaypointWeldView;
  isWaypointWeld(id: string): boolean;
  /** Undo/redo across processes through OLP's restoreAsRevision (core/entry.ts). */
  restoreSnapshot(snapshotText: string, kind: "undo" | "redo"): void;
}

let loaded: Promise<Engine> | null = null;

/** dist/src/lib/engine.js -> dist/core/olp-core.mjs, built by cli/v1 scripts/build-core.mjs. */
export function loadEngine(): Promise<Engine> {
  loaded ??= (async () => {
    const bundle = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "..", "core", "olp-core.mjs");
    if (!existsSync(bundle)) {
      throw new Error(`OLP's program engine is not built (${bundle}): run \`npm ci\` in offline-programming/v1/ui, then \`npm run build\` in cli/v1`);
    }
    return (await import(pathToFileURL(bundle).href)) as Engine;
  })();
  return loaded;
}
