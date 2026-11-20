/**
 * The browser's half of dense execution: the OLP server's
 * `/dense-execution/` routes, normalised into the UI's own naming.
 *
 * The browser never sees the trajectory bytes, the TCP ingest, NATS or the
 * lease: it names the trajectory by digest and the identity the compile
 * stamped, and the server does the rest. Every reply is normalised
 * leniently -- a field the daemon has not learned to send yet reads as
 * null, never as a thrown error -- because the status poll runs four times
 * a second while a robot moves and must keep painting whatever it has.
 */

import type { Description, Event, ProcessStatus } from "../../../../../rt-core/clients/ts/rt_control_api_generated.ts";
import { manifestFromRTCore } from "../targets/manifest.ts";
import { mapRTCoreStatus } from "../targets/rtCoreStatus.ts";
import type { RobotModelID } from "../targets/cells.ts";
import { createOperatorFence } from "../targets/operatorFeedback.ts";
import { TargetSelectionSync } from "./targetSelection.ts";
import { parseUploadLimitViolation, uploadLimitText, type UploadLimitViolation } from "./uploadLimit.ts";
import { encodeBase64, normalizeTestProgram, type DenseTestProgram } from "./testProgram.ts";
import { normalizeLoopView, normalizeRunResult, RUN_PROGRAM_PATH, RUN_FINISH_PATH, type LoopView, type RunResult } from "./runProgram.ts";
import { normalizeCellOutputs, type CellOutputs } from "./cellOutputs.ts";

export const DENSE_EXECUTION_PATH = "/api/offline-programming/v1/dense-execution" as const;

export interface DenseCapabilities {
	backend?: "unavailable" | "rt_core";
	rtCore?: ReturnType<typeof manifestFromRTCore> | null;
  available: boolean;
  cell: string;
  ingestAddr: string;
  natsUrl: string;
  rtcoreHost: string;
  bridgeCell?: string;
  controllerId: string;
  daemonReachable: boolean;
  reason: string | null;
  /** OLP enforces the explicit TCP jog rates on return-to-zero requests. */
  recoverZeroJogSpeeds?: boolean;
  /** The fleet: the selected cell (blank for none) and the registry. */
  activeCell?: string;
  cells?: DenseCellSpec[];
  registryPath?: string;
  /** Present only while the server takes test programs (testProgram.ts). */
  testPrograms?: { enabled: boolean; maxBytes: number } | null;
}

/** One registered deployment cell, as the server keeps it. */
export interface DenseCellSpec {
  id: string;
  label: string;
  address: string;
  host: string;
  credentialRef: string;
  modelId: string;
  backend: string;
  configurationDigest: string;
  source: "commissioned";
}

/** A registered cell with whether its daemon answered a dial just now. */
export interface DenseCellProbe extends DenseCellSpec {
  reachable: boolean;
  error: string | null;
}

export interface DenseCellsBody {
  cells: DenseCellProbe[];
  activeCell: string;
  registryPath: string;
}

/** What the Robot card sends to add a cell. */
export interface DenseCellInput {
  id: string;
  address: string;
  host: string;
  credentialRef: string;
}

export interface DenseLoadIdentity {
  /** Select motion-only execution without changing the source plan. */
  dry_run?: boolean;
  trajectory_digest: string;
  plan_id: string;
  program_id: string;
  program_digest: string;
  manifest_revision: number;
  plan_revision: number;
  /** "test_program": an uploaded tool-generated program (testProgram.ts). */
  source?: "test_program";
  /** The planner artifact as kept at plan time; the server loads it only when
   *  the planner cannot serve the digest, and only as exactly this plan. */
  dense_blob_base64?: string;
  /** The plan record's output schedule digest, sent only for a plan that
   *  switches outputs; a production Load of such a plan is refused without it. */
  process_markers_sha256?: string;
}

/** Whether `axis` is one the active pair maps. RTCore reports all nine
 *  wire axes, and an unmapped one (J7–J9 on a cell with no positioner)
 *  is never homed; a mask of 0 or null means the authority has not said,
 *  and every axis counts. */
export function isMappedAxis(mask: number | null, index: number): boolean {
  return mask === null || mask === 0 || (mask & (1 << index)) !== 0;
}

/** The mapped axes that report no valid home. This is the daemon's own
 *  go_home refusal ("a mapped axis has no valid home"); counting unmapped
 *  axes too kept GO HOME disabled on rosie_1420_v1 (J7–J9 unhomed, unmapped). */
export function unhomedMappedAxes(axes: readonly DenseAxisStatus[] | null, mask: number | null): DenseAxisStatus[] {
  return (axes ?? []).filter((axis) => isMappedAxis(mask, axis.index) && axis.homed === false);
}

export interface DenseAxisStatus {
  index: number;
  qRad: number | null;
  qCmdRad: number | null;
  posCounts: number;
  homeCounts: number | null;
  statusword: number;
  modeDisplay: number | null;
  operational: boolean;
  fault: boolean;
  error: boolean;
  /** RTCore's home-valid bit; null when the daemon does not report it. An
   *  unhomed axis has a position in counts but no meaningful angle. */
  homed: boolean | null;
}

export interface DenseCursor {
  segmentIndex: number;
  segmentCount: number;
  kind: "freespace" | "weld" | "dwell" | string;
  sourceId: string;
  sampleIndex: number;
  globalSampleIndex: number;
  totalSampleCount: number;
  tS: number;
  globalTS: number;
  totalDurationS: number;
}

export interface DenseSessionStatus {
  /** Unknown on older servers; never assume an unknown session is motion-only. */
  dryRun?: boolean | null;
  executionTrajectoryDigest?: string;
  id: string;
  trajectoryDigest: string;
  planId: string;
  state: string;
  leaseHeld: boolean;
  leaseExpiresInMs: number | null;
  heartbeatAgeMs: number | null;
  heartbeatDeadlineMs: number | null;
  stopReason: string | null;
  lastError: string | null;
  /** "test_program" for an uploaded test program; null for a planner one. */
  source?: string | null;
  testProgram?: { fileName: string; fileSha256: string } | null;
  /** The outputs a loaded production program switches; empty otherwise. */
  processOutputs?: string[];
}

/** The robot description and machine_planning_calibration.json a daemon holds plans to
 *  (docs/ROBOT-DESCRIPTION-CONTRACT.md). */
export interface DenseRobotCell {
  /** False: the daemon was started without a description, and refuses every plan. */
  configured: boolean;
  /** False: it was given one, but it or its machine_planning_calibration.json does not read; `error` says why. */
  valid: boolean;
  modelId: string;
  /** The description's identity, "sha256:" + 64 hex. */
  robotDescriptionSha256: string;
  /** `sha256:<hex>` of the machine_planning_calibration.json bytes (the empty document when there is none). */
  machinePlanningCalibrationSha256: string;
  /** The machine's whole compiled configuration ("sha256:" + rt-core's
   *  configuration_sha256), or "". Recorded in plans; never a refusal. */
  machineConfigurationSha256: string;
  /** Whether the cell has a machine_planning_calibration.json file at all. */
  machinePlanningCalibrationPresent: boolean;
  error: string | null;
}

/** What the target says about itself, and the bytes a plan request carries. */
export interface DenseCellBody {
  robotCell: DenseRobotCell;
  machinePlanningCalibrationBase64: string;
}

/** The daemon's last refusal, kept in its status document. */
export interface DenseDaemonError {
  reason: string;
  detail: string;
}

export interface DenseDaemonStatus {
  /** idle|staged|playing|paused|done|aborted, or "absent" when the daemon
   *  has no executor (it is not attached to a cell). */
  executorState: string;
  abortReason: string | null;
  trajectoryDigest: string | null;
  segmentIndex: number | null;
  sampleIndex: number | null;
  tS: number | null;
  torchOn: boolean;
  feedbackAgeMs: number | null;
  authorityFresh: boolean;
  runtimeMode: string | null;
  /** RTCore's pair-config mapped axes; axes outside it are not this cell's. */
  mappedAxisMask: number | null;
  /** The daemon's own plan under the cursor ("go_home"), or null. */
  transient: string | null;
  /** The cell behind RTCore, as its pair authority names it. */
  jetsonId: string | null;
  robotModelId: string | null;
  /** Null on a daemon from before the robot description contract. */
  robotCell: DenseRobotCell | null;
  staged: boolean;
  lastError: DenseDaemonError | null;
}

/** The server's own flags beside the rt-core observation: its local Stop
 *  latch, an unconfirmed Stop, the recovery it asks for, and the note it
 *  leaves when a request was retried without reaching the adapter. */
export interface DenseRTCoreFlags {
  inhibited: boolean;
  stopUncertain: boolean;
  recoveryAction: string | null;
  note: string | null;
}

/** One record of the ring window as the server thins it: raw drive units
 *  (counts, counts/s, per-mille) with their validity; the strip converts them
 *  with the axis description exactly as it converts the live status. */
export interface DenseTelemetryWindowAxis {
  coordinate_counts?: number; coordinate_ok?: boolean; home_offset?: number;
  p: number; tp: number; v: number; v_ok: boolean; tv: number; tv_ok: boolean; fe: number; fe_ok: boolean; tq: number; tq_ok: boolean;
}
export interface DenseTelemetryWindowRecord {
  configuration_epoch?: number; home_epoch?: number; trajectory_id?: number; submitted_mask?: number;
  seq: number; t_ns: number; work_ns: number; jitter_ns: number; armed: boolean; jog_open: boolean; state: number; mode: number; fault_mask: number; exec_fault: number;
  ax: DenseTelemetryWindowAxis[];
}
export interface DenseTelemetryWindow {
  daemon_incarnation?: string; configuration_sha256?: string;
  cycle_period_ns: number; axis_count: number; stride: number; seconds: number;
  first_sequence: number; last_sequence: number; head_estimate: number; fetched_records: number; fetch_ms: number;
  records: DenseTelemetryWindowRecord[];
}

export function normalizeTelemetryWindow(payload: unknown): DenseTelemetryWindow {
  const record = isRecord(payload) ? payload : {};
  const num = (v: unknown): number => (typeof v === "number" && Number.isFinite(v) ? v : 0);
  const records = Array.isArray(record.records) ? record.records : [];
  return {
    daemon_incarnation: typeof record.daemon_incarnation === "string" ? record.daemon_incarnation : undefined,
    configuration_sha256: typeof record.configuration_sha256 === "string" ? record.configuration_sha256 : undefined,
    cycle_period_ns: num(record.cycle_period_ns) || 1_000_000,
    axis_count: num(record.axis_count),
    stride: num(record.stride) || 1,
    seconds: num(record.seconds),
    first_sequence: num(record.first_sequence),
    last_sequence: num(record.last_sequence),
    head_estimate: num(record.head_estimate),
    fetched_records: num(record.fetched_records),
    fetch_ms: num(record.fetch_ms),
    records: records.filter(isRecord).map(r => ({
      configuration_epoch: num(r.configuration_epoch), home_epoch: num(r.home_epoch), trajectory_id: num(r.trajectory_id), submitted_mask: num(r.submitted_mask),
      seq: num(r.seq), t_ns: num(r.t_ns), work_ns: num(r.work_ns), jitter_ns: num(r.jitter_ns),
      armed: r.armed === true, jog_open: r.jog_open === true, state: num(r.state), mode: num(r.mode), fault_mask: num(r.fault_mask), exec_fault: num(r.exec_fault),
      ax: (Array.isArray(r.ax) ? r.ax : []).filter(isRecord).map(a => ({
        coordinate_counts: typeof a.coordinate_counts === "number" ? a.coordinate_counts : undefined,
        coordinate_ok: a.coordinate_ok === true, home_offset: typeof a.home_offset === "number" ? a.home_offset : undefined,
        p: num(a.p), tp: num(a.tp), v: num(a.v), v_ok: a.v_ok === true, tv: num(a.tv), tv_ok: a.tv_ok === true,
        fe: num(a.fe), fe_ok: a.fe_ok === true, tq: num(a.tq), tq_ok: a.tq_ok === true,
      })),
    })),
  };
}

export interface DenseStatus {
  roundTripMS?: number;
  linkLeaseMS?: number;
  target?: { selection_generation: string; cell_id: string; model_id: RobotModelID; label: string; backend: string; simulation?: boolean; host: string };
  backend?: "unavailable" | "rt_core";
  rtCore?: ReturnType<typeof mapRTCoreStatus> | null;
  rtCoreFlags?: DenseRTCoreFlags | null;
  available: boolean;
  session: DenseSessionStatus | null;
  daemon: DenseDaemonStatus | null;
  daemonError: string | null;
  cursor: DenseCursor | null;
  armed: boolean | null;
  axes: DenseAxisStatus[] | null;
  /** Which robot the machine says it is under the description contract,
   *  once it has been described; `configured` false with the reason when it
   *  binds none (docs/ROBOT-DESCRIPTION-CONTRACT.md). */
  robotCell?: DenseRobotCell | null;
  /** The server's last classified jog end, as sent (execution/jogEnd.ts
   *  reads it); absent from an older server. */
  lastJogEnd?: unknown;
  /** The running Cartesian path hold's live readout, as sent (jogLive.ts). */
  jogLive?: unknown;
  /** The running or last loop of a repeating Run program (runProgram.ts). */
  loop?: LoopView | null;
  /** The cell's outputs: intent, command, readback (cellOutputs.ts); null
   *  for a cell with no io block. */
  outputs?: CellOutputs | null;
}

export interface DenseLoadResult {
  sessionId: string;
  status: DenseStatus;
  /** True things about the load that did not stop it, in the server's words. */
  notes: string[];
}

/** Resolve the selected controller's observed state, preventing a public session
 * from inheriting the missing legacy daemon's unknown state. */
export function denseExecutionState(status: DenseStatus | null): string {
  return status?.backend === "rt_core" ? status.session?.state ?? "unknown" : status?.daemon?.executorState ?? "unknown";
}

/** Public programs are single-use prepared identities; completion and Stop
 * require a new Load rather than enabling the legacy replay button. */
export function denseProgramPlayable(status: DenseStatus | null, digest: string, dryRun?: boolean): boolean {
  if (dryRun !== undefined && status?.session?.dryRun !== dryRun) return false;
  if (!status?.session || status.session.trajectoryDigest !== digest) return false;
  const state = denseExecutionState(status);
  return status.backend === "rt_core"
    ? state === "loaded" && status.available && !status.daemonError && status.armed === true && status.rtCore?.ready === true
    : ["staged", "playing", "paused", "done", "aborted"].includes(state);
}

export interface DenseStopResult {
  stopped: boolean;
  attempts: number;
  status: DenseStatus;
}

export interface DenseArmResult {
  accepted: boolean;
  error: string | null;
  status: DenseStatus;
}

export class DenseExecutionError extends Error {
  readonly code: string;
  readonly httpStatus: number;
  readonly detail: string;
  readonly limitViolation: UploadLimitViolation | null;

  constructor(code: string, detail: string, httpStatus: number, limitViolation: UploadLimitViolation | null = null) {
    const limitRefusal = code === "native_limit_exceeded" || code === "native_segment_rate_exceeded";
    const diagnostic = limitRefusal && limitViolation ? uploadLimitText(limitViolation) : detail;
    super(diagnostic ? `${code}: ${diagnostic}` : code);
    this.code = code;
    this.httpStatus = httpStatus;
    this.detail = detail;
    this.limitViolation = limitViolation;
  }
}

const isRecord = (value: unknown): value is Record<string, unknown> =>
  typeof value === "object" && value !== null && !Array.isArray(value);

const str = (value: unknown, fallback = ""): string => (typeof value === "string" ? value : fallback);
const strOrNull = (value: unknown): string | null => (typeof value === "string" && value !== "" ? value : null);
const num = (value: unknown, fallback = 0): number => (typeof value === "number" && Number.isFinite(value) ? value : fallback);
const numOrNull = (value: unknown): number | null => (typeof value === "number" && Number.isFinite(value) ? value : null);
const bool = (value: unknown): boolean => value === true;

export function normalizeDenseCapabilities(payload: unknown): DenseCapabilities {
  const record = isRecord(payload) ? payload : {};
  const rtCore = record.backend === "rt_core" && isRecord(record.rt_core)
    ? manifestFromRTCore(record.rt_core.description as Description, record.rt_core.address_host) : null;
  return {
    backend: record.backend === "rt_core" ? "rt_core" : "unavailable",
    rtCore,
    available: record.backend === "rt_core" && bool(record.available),
    cell: str(record.cell),
    ingestAddr: str(record.ingest_addr),
    natsUrl: str(record.nats_url),
    rtcoreHost: str(record.rtcore_host),
    bridgeCell: str(record.bridge_cell),
    controllerId: str(record.controller_id),
    daemonReachable: bool(record.daemon_reachable),
    reason: record.backend === "rt_core" ? strOrNull(record.reason) : "backend_retired",
    recoverZeroJogSpeeds: record.recover_zero_jog_speeds === true,
    activeCell: str(record.active_cell),
    cells: Array.isArray(record.cells) ? record.cells.map(normalizeCellSpec) : [],
    registryPath: str(record.registry_path),
    testPrograms: isRecord(record.test_programs) && record.test_programs.enabled === true
      ? { enabled: true, maxBytes: num(record.test_programs.max_bytes) } : null,
  };
}

export function normalizeCellSpec(value: unknown): DenseCellSpec {
  const record = isRecord(value) ? value : {};
  return {
    id: str(record.id),
    label: str(record.label),
    address: str(record.address),
    host: str(record.host),
    credentialRef: str(record.credential_ref),
    modelId: str(record.model_id),
    backend: str(record.backend),
    configurationDigest: str(record.configuration_digest),
    source: "commissioned",
  };
}

export function normalizeDenseCells(payload: unknown): DenseCellsBody {
  const record = isRecord(payload) ? payload : {};
  return {
    cells: Array.isArray(record.cells)
      ? record.cells.map((value) => {
        const cell = isRecord(value) ? value : {};
        return { ...normalizeCellSpec(cell), reachable: bool(cell.reachable), error: strOrNull(cell.error) };
      })
      : [],
    activeCell: str(record.active_cell),
    registryPath: str(record.registry_path),
  };
}

function normalizeAxis(value: unknown, index: number): DenseAxisStatus {
  const record = isRecord(value) ? value : {};
  return {
    index: num(record.index, index),
    qRad: numOrNull(record.q_rad),
    qCmdRad: numOrNull(record.q_cmd_rad),
    posCounts: num(record.pos_counts),
    homeCounts: num(record.home_counts),
    statusword: num(record.statusword),
    modeDisplay: num(record.mode_display),
    operational: bool(record.operational),
    fault: bool(record.fault),
    error: bool(record.error),
    homed: typeof record.homed === "boolean" ? record.homed : null,
  };
}

export function normalizeRobotCell(value: unknown): DenseRobotCell | null {
  if (!isRecord(value)) return null;
  return {
    configured: bool(value.configured),
    valid: bool(value.valid),
    modelId: str(value.model_id),
    robotDescriptionSha256: str(value.robot_description_sha256),
    machinePlanningCalibrationSha256: str(value.machine_planning_calibration_sha256),
    machinePlanningCalibrationPresent: bool(value.machine_planning_calibration_present),
    machineConfigurationSha256: str(value.machine_configuration_sha256),
    error: strOrNull(value.error),
  };
}

/** `last_error` is `{reason, detail}`; an older server sent a bare string. */
function normalizeDaemonError(value: unknown): DenseDaemonError | null {
  if (typeof value === "string") return value ? { reason: value, detail: "" } : null;
  if (!isRecord(value)) return null;
  const reason = str(value.reason);
  const detail = str(value.detail);
  return reason || detail ? { reason, detail } : null;
}

function normalizeDaemon(value: unknown): DenseDaemonStatus | null {
  if (!isRecord(value)) return null;
  const executor = isRecord(value.executor) ? value.executor : null;
  const authority = executor && isRecord(executor.authority) ? executor.authority : null;
  return {
    executorState: executor ? str(executor.state, "unknown") : str(value.executor, "absent"),
    abortReason: executor ? strOrNull(executor.abort_reason) : null,
    trajectoryDigest: executor ? strOrNull(executor.trajectory_digest) : null,
    segmentIndex: executor ? numOrNull(executor.segment_index) : null,
    sampleIndex: executor ? numOrNull(executor.sample_index) : null,
    tS: executor ? numOrNull(executor.t_s) : null,
    torchOn: executor ? bool(executor.torch_on) : false,
    feedbackAgeMs: executor ? numOrNull(executor.feedback_age_ms) : null,
    authorityFresh: executor ? bool(executor.authority_fresh) : false,
    runtimeMode: authority ? strOrNull(authority.runtime_mode) : null,
    mappedAxisMask: authority ? numOrNull(authority.mapped_axis_mask) : null,
    transient: executor ? strOrNull(executor.transient) : null,
    jetsonId: authority ? strOrNull(authority.jetson_id) : null,
    robotModelId: authority ? strOrNull(authority.robot_model_id) : null,
    robotCell: executor ? normalizeRobotCell(executor.robot_cell) : null,
    staged: isRecord(value.staged),
    lastError: normalizeDaemonError(value.last_error),
  };
}

function normalizeCursor(value: unknown): DenseCursor | null {
  if (!isRecord(value)) return null;
  return {
    segmentIndex: num(value.segment_index, -1),
    segmentCount: num(value.segment_count),
    kind: str(value.kind),
    sourceId: str(value.source_id),
    sampleIndex: num(value.sample_index),
    globalSampleIndex: num(value.global_sample_index),
    totalSampleCount: num(value.total_sample_count),
    tS: num(value.t_s),
    globalTS: num(value.global_t_s),
    totalDurationS: num(value.total_duration_s),
  };
}

function normalizeSession(value: unknown): DenseSessionStatus | null {
  if (!isRecord(value)) return null;
  const lease = isRecord(value.lease) ? value.lease : {};
  return {
    id: str(value.id),
    trajectoryDigest: str(value.trajectory_digest),
    executionTrajectoryDigest: str(value.execution_trajectory_digest),
    dryRun: typeof value.dry_run === "boolean" ? value.dry_run : null,
    planId: str(value.plan_id),
    state: str(value.state, "unknown"),
    leaseHeld: bool(lease.held),
    leaseExpiresInMs: numOrNull(lease.expires_in_ms),
    heartbeatAgeMs: numOrNull(value.heartbeat_age_ms),
    heartbeatDeadlineMs: numOrNull(value.heartbeat_deadline_ms),
    stopReason: strOrNull(value.stop_reason),
    lastError: strOrNull(value.last_error),
    source: strOrNull(value.source),
    testProgram: isRecord(value.test_program)
      ? { fileName: str(value.test_program.file_name), fileSha256: str(value.test_program.file_sha256) } : null,
    processOutputs: Array.isArray(value.process_outputs) ? value.process_outputs.filter((name): name is string => typeof name === "string") : [],
  };
}

export function normalizeDenseStatus(payload: unknown, fence = createOperatorFence()): DenseStatus {
  const record = isRecord(payload) ? payload : {};
  if (record.backend === "rt_core") {
    const observation = isRecord(record.rt_core) ? record.rt_core : {};
    let rtCore: ReturnType<typeof mapRTCoreStatus> | null = null;
    let reason = strOrNull(observation.reason);
    try {
      rtCore = mapRTCoreStatus(observation.status as ProcessStatus, (observation.events ?? []) as Event[], reason ?? "", (observation.description ?? { axes: [] }) as Description, typeof observation.refusal === "string" ? observation.refusal : undefined, typeof observation.refusal_after_ns === "number" ? observation.refusal_after_ns : undefined, typeof observation.refusal_kind === "string" ? observation.refusal_kind : "unknown", fence);
      reason = rtCore.reason;
    } catch { reason ||= "rt_core status unavailable"; }
    return {
      backend: "rt_core", rtCore, available: bool(record.available),
      roundTripMS: typeof record.round_trip_ns === "number" && record.round_trip_ns > 0 ? record.round_trip_ns / 1e6 : undefined,
      linkLeaseMS: typeof record.link_lease_ns === "number" && record.link_lease_ns > 0 ? record.link_lease_ns / 1e6 : undefined,
      rtCoreFlags: {
        inhibited: bool(observation.inhibited), stopUncertain: bool(observation.stop_uncertain),
        recoveryAction: strOrNull(observation.recovery_action), note: strOrNull(observation.note),
      },
      ...(isRecord(record.target) ? { target: record.target as DenseStatus["target"] } : {}),
      session: normalizeSession(record.session), daemon: null, daemonError: reason,
      cursor: null, armed: reason ? null : rtCore?.armed ?? null,
      robotCell: normalizeRobotCell(record.robot_cell),
      lastJogEnd: record.last_jog_end ?? null,
      jogLive: record.jog_live ?? null,
      loop: normalizeLoopView(record.loop),
      outputs: normalizeCellOutputs(record),
      // Logical units belong to Describe. The public view retains them in
      // rtCore; the legacy radian viewer cannot infer units or URDF mapping.
      axes: rtCore?.axes.map((axis, index) => ({ index, qRad: null, qCmdRad: null,
        posCounts: axis.positionCounts, homeCounts: null, statusword: axis.statusword,
        modeDisplay: null, operational: axis.readiness === "ready", fault: axis.readiness === "faulted",
        error: axis.readiness === "faulted", homed: axis.homeValid })) ?? null,
    };
  }
  const axes = Array.isArray(record.axes)
    ? record.axes.map((axis, index) => normalizeAxis(axis, index))
    : null;
  return {
    backend: "unavailable",
    available: false,
    session: normalizeSession(record.session),
    daemon: normalizeDaemon(record.daemon),
    daemonError: "backend_retired",
    cursor: normalizeCursor(record.cursor),
    armed: typeof record.armed === "boolean" ? record.armed : null,
    axes,
  };
}

async function readJSON(response: Response): Promise<unknown> {
  try {
    return await response.json();
  } catch {
    return null;
  }
}

function raiseFor(response: Response, payload: unknown, what: string): never {
  const record = isRecord(payload) ? payload : {};
  throw new DenseExecutionError(
    str(record.error, `dense_${what}_http_${response.status}`),
    str(record.detail, `${what} failed with HTTP ${response.status}`),
    response.status,
    parseUploadLimitViolation(record.limit_violation ?? (isRecord(record.details) ? record.details.limit_violation : null)),
  );
}

export class DenseExecutionClient {
  readonly #fetch: typeof fetch;
  #selectionChanged: ((status?: DenseStatus) => void) | null = null;
  /** Share selection recovery with jog request builders, preventing independent
   * state and heartbeat loops from retaining a rejected generation. */
  get fetcher(): typeof fetch { return this.#fetch; }
  onSelectionChanged(listener: (status?: DenseStatus) => void): void { this.#selectionChanged = listener; }
  #target: DenseStatus["target"];

  /** Bind requests to the rendered status; a background refresh cannot silently
   * retarget an operator action before its machine badge changes. */
  observeTarget(target: DenseStatus["target"]): void {
    this.#target = target ? { ...target } : undefined;
  }
  readonly #feedbackFence = createOperatorFence();

  constructor(fetcher: typeof fetch = globalThis.fetch.bind(globalThis)) {
    this.#fetch = new TargetSelectionSync(fetcher, DENSE_EXECUTION_PATH, payload => {
      if (payload === undefined) { this.#selectionChanged?.(); return; }
      const status = normalizeDenseStatus(payload, this.#feedbackFence);
      if (!status.target) return;
      this.observeTarget(status.target);
      this.#selectionChanged?.(status);
    }).fetch;
  }

  async #get(path: string, signal?: AbortSignal): Promise<unknown> {
    const response = await this.#fetch(`${DENSE_EXECUTION_PATH}${path}`, {
      method: "GET",
      cache: "no-store",
      credentials: "same-origin",
      redirect: "error",
      signal,
    });
    const payload = await readJSON(response);
    if (!response.ok) raiseFor(response, payload, path.slice(1));
    return payload;
  }

  async #post(path: string, body: unknown, signal?: AbortSignal): Promise<unknown> {
    const { response, payload } = await this.#exchange(path, body, signal);
    if (!response.ok) raiseFor(response, payload, path.slice(1));
    return payload;
  }

  async #exchange(path: string, body: unknown, signal?: AbortSignal): Promise<{ response: Response; payload: unknown }> {
    const response = await this.#fetch(`${DENSE_EXECUTION_PATH}${path}`, {
      method: "POST",
      headers: {
        "Accept": "application/json", "Content-Type": "application/json",
        ...(this.#target ? {
          "X-RT-Target-Generation": this.#target.selection_generation,
          "X-RT-Target-Cell": this.#target.cell_id,
        } : {}),
      },
      cache: "no-store",
      credentials: "same-origin",
      redirect: "error",
      body: JSON.stringify(body ?? {}),
      signal,
    });
    return { response, payload: await readJSON(response) };
  }

  /** One Run program press (`runProgram.ts`). A refusal the server explains
   *  answers as a refused result with its steps; anything else throws. */
  async runProgram(body: Record<string, unknown>, signal?: AbortSignal): Promise<{ result: RunResult; status: DenseStatus | null }> {
    const { response, payload } = await this.#exchange(RUN_PROGRAM_PATH, body, signal);
    const record = isRecord(payload) ? payload : {};
    if (!response.ok && !isRecord(record.run)) raiseFor(response, payload, "run-program");
    const result = normalizeRunResult(payload);
    if (!response.ok && result.state !== "refused") raiseFor(response, payload, "run-program");
    const run = isRecord(record.run) ? record.run : record;
    return { result: { ...result, code: str(record.error) || result.refusal?.code || "" }, status: isRecord(run.status) ? normalizeDenseStatus(run.status, this.#feedbackFence) : null };
  }

  /** Finish cycle: the running loop ends at the program start after the
   *  current cycle (only the client that started it may ask). */
  async finishLoop(signal?: AbortSignal): Promise<LoopView | null> {
    const payload = await this.#post(RUN_FINISH_PATH, {}, signal);
    return normalizeLoopView(isRecord(payload) ? payload.loop : null);
  }

  async capabilities(signal?: AbortSignal): Promise<DenseCapabilities> {
    return normalizeDenseCapabilities(await this.#get("/capabilities", signal));
  }

  /** The registry, each cell dialled once by the server. */
  async cells(signal?: AbortSignal): Promise<DenseCellsBody> {
    return normalizeDenseCells(await this.#get("/cells", signal));
  }

  /** Select a cell (blank disconnects); answers the new capabilities. */
  async selectCell(cell: string, signal?: AbortSignal): Promise<DenseCapabilities> {
    return normalizeDenseCapabilities(await this.#post("/target", { cell }, signal));
  }

  async addCell(input: DenseCellInput, signal?: AbortSignal): Promise<DenseCellsBody> {
    return normalizeDenseCells(await this.#post("/cells", {
      id: input.id,
      address: input.address,
      host: input.host,
      credential_ref: input.credentialRef,
    }, signal));
  }

  async removeCell(id: string, signal?: AbortSignal): Promise<DenseCellsBody> {
    const response = await this.#fetch(`${DENSE_EXECUTION_PATH}/cells/${encodeURIComponent(id)}`, {
      method: "DELETE",
      cache: "no-store",
      credentials: "same-origin",
      redirect: "error",
      signal,
    });
    const payload = await readJSON(response);
    if (!response.ok) raiseFor(response, payload, "cells");
    return normalizeDenseCells(payload);
  }

  /** The selected target's machine_planning_calibration.json and robot identity. */
  async cell(signal?: AbortSignal): Promise<DenseCellBody> {
    const payload = await this.#get("/cell", signal);
    const record = isRecord(payload) ? payload : {};
    const robot = normalizeRobotCell(record.robot_cell);
    if (!robot) throw new DenseExecutionError("dense_cell_invalid", "the target named no robot identity", 502);
    return { robotCell: robot, machinePlanningCalibrationBase64: str(record.machine_planning_calibration_base64) };
  }

  async status(signal?: AbortSignal): Promise<DenseStatus> {
    return normalizeDenseStatus(await this.#get("/status", signal), this.#feedbackFence);
  }

  /** The newest `seconds` of the machine's telemetry ring, every `stride`-th
   *  record (`telemetryStrip.ts` plots it). The server reads the ring from
   *  the cell, so over a slow link this takes seconds; the signal aborts it. */
  async telemetryWindow(seconds: number, stride: number, signal?: AbortSignal): Promise<DenseTelemetryWindow> {
    const payload = await this.#get(`/telemetry/window?seconds=${encodeURIComponent(String(seconds))}&stride=${encodeURIComponent(String(stride))}`, signal);
    return normalizeTelemetryWindow(payload);
  }

  async load(identity: DenseLoadIdentity, signal?: AbortSignal): Promise<DenseLoadResult> {
    const payload = await this.#post("/load", identity, signal);
    const record = isRecord(payload) ? payload : {};
    const notes = Array.isArray(record.notes) ? record.notes.filter((note): note is string => typeof note === "string" && note !== "") : [];
    return { sessionId: str(record.session_id), status: normalizeDenseStatus(record.status, this.#feedbackFence), notes };
  }

  /** Upload a test program (and its plan.json) for the cell's checks; Load
   *  then names it with source "test_program". Moves nothing. */
  async uploadTestProgram(input: { fileName: string; bytes: Uint8Array; planJSON: unknown | null; planFileName: string | null }, signal?: AbortSignal): Promise<{ program: DenseTestProgram; checks: string[] }> {
    const payload = await this.#post("/test-program", {
      file_name: input.fileName, program_base64: encodeBase64(input.bytes),
      ...(input.planJSON !== null ? { plan_json: input.planJSON, plan_file_name: input.planFileName ?? "plan.json" } : {}),
    }, signal);
    const record = isRecord(payload) ? payload : {};
    const program = normalizeTestProgram(record.program);
    if (!program) throw new DenseExecutionError("test_program_invalid", "the server answered without the program", 502);
    const checks = Array.isArray(record.checks) ? record.checks.filter((c): c is string => typeof c === "string") : [];
    return { program, checks };
  }

  async play(sessionId: string, signal?: AbortSignal): Promise<DenseStatus> {
    return normalizeDenseStatus(await this.#post("/play", { session_id: sessionId }, signal), this.#feedbackFence);
  }

  async pause(sessionId: string, signal?: AbortSignal): Promise<DenseStatus> {
    return normalizeDenseStatus(await this.#post("/pause", { session_id: sessionId }, signal), this.#feedbackFence);
  }

  /** The software stop: always allowed, session or not. The server resends
   *  the daemon's stop until it is acknowledged and releases the lease. */
  async stop(signal?: AbortSignal): Promise<DenseStopResult> {
    const payload = await this.#post("/stop", {}, signal);
    const record = isRecord(payload) ? payload : {};
    return {
      stopped: bool(record.stopped),
      attempts: num(record.attempts),
      status: normalizeDenseStatus(record.status, this.#feedbackFence),
    };
  }

  /** Clear eligible native faults without enabling, arming or homing. */
  async clearFaults(signal?: AbortSignal): Promise<DenseArmResult> {
    const payload = await this.#post("/reset-fault", {}, signal);
    const record = isRecord(payload) ? payload : {};
    return { accepted: bool(record.accepted), error: strOrNull(record.error),
      status: normalizeDenseStatus(record.status, this.#feedbackFence) };
  }

  async arm(armed: boolean, signal?: AbortSignal): Promise<DenseArmResult> {
    const payload = await this.#post("/arm", { armed }, signal);
    const record = isRecord(payload) ? payload : {};
    return {
      accepted: bool(record.accepted) || bool(record.ok),
      error: strOrNull(record.error),
      status: normalizeDenseStatus(record.status, this.#feedbackFence),
    };
  }

  /** Go home: the daemon interpolates from the pose the servos report to
   *  its configured reset pose, at rest at both ends, under the same lease
   *  and heartbeat as a play. A motion command, fenced like play. */
  async goHome(signal?: AbortSignal): Promise<DenseStatus> {
    return normalizeDenseStatus(await this.#post("/go-home", {}, signal), this.#feedbackFence);
  }

  /** The native Home on every configured axis, or on the axes given (indices). */
  async home(axes?: readonly number[], signal?: AbortSignal): Promise<DenseStatus> {
    return normalizeDenseStatus(await this.#post("/home", axes && axes.length ? { axes: [...axes] } : {}, signal), this.#feedbackFence);
  }

  async heartbeat(sessionId: string, signal?: AbortSignal): Promise<void> {
    await this.#post("/heartbeat", { session_id: sessionId }, signal);
  }
}
