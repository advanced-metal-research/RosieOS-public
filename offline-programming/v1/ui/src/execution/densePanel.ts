/**
 * The machine panel: one path from choosing a cell to driving it. Choose →
 * Connect → Home → Arm → Drive, each step done, current, blocked (with the
 * server's typed reason in words) or not yet possible, one primary action
 * at a time, and STOP / DISARM in reach the whole way. It owns the status
 * poll and the heartbeat, the arm and run controls, the per-motor readout,
 * and lends the joint jog its host inside the Jog panel.
 *
 * What this panel exists to prevent: a robot that keeps welding after the
 * browser that started it has gone. While a session is live the panel posts
 * a heartbeat every second; the OLP server stops the daemon when it has
 * heard nothing for five seconds (`ui_heartbeat_lost`). The panel shows the
 * same clock to the operator, so a stalled tab reads as "connection lost,
 * stopping in 3 s" rather than as a frozen number.
 *
 * And the second thing: arming a real cell while believing it was the
 * simulator. A REAL or unknown posture needs a second, deliberate click;
 * a described simulation does not.
 *
 * The browser owns no lease, sequence or socket. Every button is one HTTP
 * request to the OLP server, which is the single controller of the daemon;
 * the panel renders what the server reports and nothing it merely hopes.
 */

import type {
  CellSettings, OfflineProgrammingProject } from "../domain/project.ts";
import { Signal } from "../ui/signal.ts";
import type { DenseCellInput, DenseCellProbe, DenseCellSpec } from "./denseClient.ts";
import { DenseExecutionError } from "./denseClient.ts";
import { denseSegmentNodeId } from "../program-v2/dense-cursor.ts";
import { JointJogPanel, jointJogAPI } from "./jointJog.ts";
import type { RecoverConfirmation } from "./recoverToZero.ts";
import type { JogEndCause } from "./jogEndCause.ts";
import { TARGET_SELECTION_CHANGED } from "./targetSelection.ts";
import { MachineCartesianJog } from "./cartesianJog.ts";
import { rtCoreTargetText } from "../targets/manifest.ts";
import { executionPosturePill, type ExecutionPosture } from "../sessionStatusPills.ts";
import { armModels, CellCatalogueError, cellTargetText, robotModelClass, unknownRobotModelReason, type CellTarget, type CellTargetsClient, type RobotModelID } from "../targets/cells.ts";
import { processIOAdmissionMessage, renderOperatorFeedback } from "../targets/operatorFeedback.ts";
import { armRefusalText, deriveMachineFlow, nativeHomeRecoveryRefusal, machineReasonText, STATUS_FRESHNESS_MS, type MachineFlowView, type MachineRequest, type MachineStepID } from "./machineFlow.ts";
import {
  DenseExecutionClient,
  denseExecutionState,
  denseProgramPlayable,
  unhomedMappedAxes,
  type DenseCapabilities,
  type DenseStatus,
} from "./denseClient.ts";
import { deriveLivePose, mapLivePose, type LivePoseView } from "./livePose.ts";
import { loadDoneText, loadFailedText, loadProgressFraction, loadProgressOverdue, loadProgressText } from "./loadProgress.ts";
import { deriveHomeStatus } from "./homeStatus.ts";
import { decodeDenseTrajectory } from "./denseTrajectoryBlob.ts";
import { atPlanStart, deriveRunControls, LIVE_STATES, PLAY_NEEDS_PLAN_START, planMotionBlocked, planStartEligible, planStartOf } from "./runControls.ts";
import { deriveRunProgram, finishControl, finishRefusal, loopText, MAX_LOOP_COUNT, repeatOf, RUN_CANCELLED, runConfirmation, runNotSent, runOutcomeText, runProgramRequest, runRefusal, unattendedLoopRunning, type RepeatMode, type RunResult } from "./runProgram.ts";
import type { PressRefusal } from "./pressNotice.ts";
import { encodeSTEP as base64Of } from "../rails/client.ts";
import { denseLoadIdentity } from "../rails/planRequest.ts";
import { cellOutputsView, type CellOutputs } from "./cellOutputs.ts";
import { cellSettingsOf, machineReachable, nextTeachStill, teachObservationOf, type TeachStill } from "./teach.ts";
import { deriveTestProgramControls, pickTestProgramFiles, testProgramLabel, testProgramLoadIdentity, testProgramSession, type DenseTestProgram } from "./testProgram.ts";
import type { RobotPoseSnapshot, SimulationSample } from "../simulation/contracts.ts";

/** Status poll period. 250 ms: fast enough that a 100 Hz sample counter
 *  reads as moving, slow enough that four tabs do not load the daemon's
 *  single TCP ingest thread (each poll is one TCP round trip there). */
export const DENSE_STATUS_POLL_MS = 250;
/** How often the panel re-requests capabilities it lacks for a target that
 *  keeps answering status: eight status polls apart, so a server whose probe
 *  is still failing is not hammered while the operator waits. */
export const CAPABILITIES_RETRY_MS = 2_000;
/** The poll while the viewer shows the real robot. The pose itself reaches
 *  the OLP on the daemon's status stream (20 Hz; internal/denseexec/
 *  statusstream.go says why it is a stream and not a poll), so this request
 *  reads a cache on the same machine and costs ~0.2 ms. It deliberately
 *  runs at twice the stream's rate: polling at exactly 20 Hz beats against
 *  the stream and lands frames at an uneven 50/100/50 ms, which the viewer
 *  shows as stutter; sampling twice as often puts every frame on screen
 *  within half a period of its arrival. */
export const DENSE_STATUS_POLL_FAST_MS = 25;
/** Heartbeat period. One fifth of the server's 5 s deadline, so four
 *  consecutive lost posts are survivable and the fifth is the stop. */
export const DENSE_HEARTBEAT_MS = 1000;
/** The server's deadline, restated here only for the countdown the operator
 *  sees; the server's value is authoritative and arrives in every status. */
const HEARTBEAT_DEADLINE_MS = 5000;

const STOP_TITLE =
  "Software stop: stops the daemon's playback, drops the torch, releases the lease. " +
  "This is not the hardware emergency stop.";
const STEP_IDS: readonly MachineStepID[] = ["choose", "connect", "home", "arm", "drive"];

export interface DensePanelDeps {
  client: DenseExecutionClient;
  cells?: CellTargetsClient;
  onCellModels?(models: readonly RobotModelID[], selected: RobotModelID): void;
  getProject(): OfflineProgrammingProject;
  readTrajectory?(digest: string): Promise<Uint8Array | ArrayBuffer | null>;
  /** Light the node the robot is executing (null clears). */
  highlightNode(nodeId: string | null): void;
  status(text: string, isError?: boolean): void;
  /** Copy visible native refusals to Logs without replacing their local feedback. */
  refusal?(text: string): void;
  /** All joints to 0° needs Arm: ask the operator (`recoverDialog.ts`). */
  confirmRecover?(confirmation: RecoverConfirmation): Promise<boolean>;
  /** The lock state changed; the host re-renders what it gates. */
  onLockChange(): void;
  /** The target changed; the host swaps its transport for this panel's. */
  onTargetChange(): void;
  /** Every render: the derived path, for the header pill and STOP / DISARM. */
  onFlowChange?(view: MachineFlowView): void;
  /** Every status that carries axis positions: the machine's pose for the
   *  model it reports (`livePose.ts`); an axis without a position keeps the
   *  value last shown. rt-core and the legacy daemon both arrive here. */
  onRobotPose(pose: RobotPoseSnapshot): void;
  /** The daemon was asked to move (play, go home): the viewer follows it. */
  onRobotMotion(): void;
  /** Poll faster while the viewer shows this robot's pose. */
  wantsFastPoll(): boolean;
  /** Every status, with the capabilities in hand: the telemetry strip
   *  samples the drives from it (`telemetryStrip.ts`). */
  onTelemetry?(status: DenseStatus, capabilities: DenseCapabilities | null): void;
  /** What the machine says about itself -- its robot identity above all --
   *  changed, or the target went away. */
  onStatus?(): void;
  /** Every status poll: the server's last classified jog end
   *  (`jogEnd.ts`), for the "Jog stopped" toast. */
  onJogEnd?(lastJogEnd: unknown): void;
  /** Every status poll: the live path-hold readout (`jogLive.ts`). */
  onJogLive?(jogLive: unknown): void;
  /** A Run program or Finish cycle press that started nothing: the toast
   *  (`pressNotice.ts`); null clears it when a later press goes through. */
  onPressNotice?(refusal: PressRefusal | null): void;
}

const TARGET_STORAGE_KEY = "offline-programming.dense-target";
const RAD_TO_DEG = 180 / Math.PI;

const escapeHTML = (value: string): string =>
  value.replace(/[&<>"']/g, (char) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", "\"": "&quot;", "'": "&#39;" })[char]!);

export class DensePanel {
  readonly changed = new Signal<void>();
  #probes: DenseCellProbe[] | null = null;
  #selecting = false;
  #cellError: string | null = null;
  #loggedFeedback = "";
  #loggedWarnings = "";
  #selectionChanged = false;
  #jointJog: JointJogPanel | null = null;
  #cartesianJog: MachineCartesianJog | null = null;
  readonly #deps: DensePanelDeps;
  #capabilities: DenseCapabilities | null = null;
  /** When capabilities were last (re)requested because the panel held none
   *  for a native target that keeps answering status: a probe that failed
   *  once at load must not disable the joint table and the live pose for
   *  the whole session. */
  #capabilitiesRequestedAtMs = 0;
  #targetSelected = false;
  #status: DenseStatus | null = null;
  #sessionId: string | null = null;
  #lastContactMs = 0;
  #nativeObservation = "";
  #lastNativeTimeNs = 0;
  #lastNativeAdvanceMs = 0;
  #pendingHomeAxes: readonly number[] | null = null;
  #pollTimer: ReturnType<typeof setTimeout> | null = null;
  #heartbeatTimer: ReturnType<typeof setTimeout> | null = null;
  #inFlight = false;
  #planStartPhase: "reading" | "moving" | null = null;
  #planStartResult = "";
  #planStartKey = "";
  #planStartPositions: readonly number[] | null = null;
  /** The Run program press in flight, its question included; STOP clears it. */
  #runPress: AbortController | null = null;
  /** The last Run program outcome, in the server's words. */
  #runNote = "";
  /** Run once, Run N times or Loop until stopped, for the next Plan and Run. */
  #repeatMode: RepeatMode = "once";
  #repeatCount = 5;
  #dryRun = true;
  /** The test program this tab uploaded and loaded (testProgram.ts). */
  #testProgram: DenseTestProgram | null = null;
  #runModeProject = "";
  #actionEpoch = 0;
  #highlightedNode: string | null = null;
  #lastPose: RobotPoseSnapshot | null = null;
  #teachStill: TeachStill | null = null;
  #robotHost: HTMLElement | null = null;
  #playbackHost: HTMLElement | null = null;
  #jogHost: HTMLElement | null = null;
  #wasLocked = false;
  #cells: CellTarget[] | null = null;
  #catalogueError: string | null = null;
  #catalogueInvalid = false;
  #connectedCell = "";
  #connectedPosture: ExecutionPosture | null = null;
  #cellPending = false;
  #cellModel: RobotModelID | null = null;
  #connectError: string | null = null;
  /** A REAL or unknown target's Arm awaits the operator's confirming click. */
  #armConfirm = false;
  #clearFaultsMessage = "";
  #clearingFaults = false;
  /** The operator has dismissed the last shown Arm refusal. The refusal
   *  stays in Logs and in armReason(); only the popover hides. */
  #armRefusalAck = false;
  /** The request in flight, by what it does to the arm state; the fact
   *  renders it as ARMING / DISARMING / STOPPING instead of unknown. */
  #pendingRequest: MachineRequest | null = null;
  /** The last request this panel completed and how it ended, so a transport
   *  failure that never reached the machine reads as a retry, not a state. */
  #lastRequest: { request: MachineRequest; error: string | null } | null = null;
  #view: MachineFlowView | null = null;

  constructor(deps: DensePanelDeps) {
    this.#deps = deps;
  }

  /** Dialog getters read native selection facts; they own no second selection. */
  cells(): DenseCellSpec[] {
    return this.#probes ?? this.#capabilities?.cells ?? (this.#cells ?? []).map(cell => ({
      id: cell.cell_id, label: cell.label, address: "", host: cell.host,
      credentialRef: "", modelId: cell.models[0] ?? "", backend: cell.backend,
      configurationDigest: "", source: "commissioned",
    }));
  }
  probes(): DenseCellProbe[] | null { return this.#probes; }
  activeCell(): string { return this.#connectedCell; }
  connectedCell(): string | null { return this.#targetSelected ? this.#connectedCell : null; }
  selecting(): boolean { return this.#selecting; }
  lastCellError(): string | null { return this.#cellError ?? this.#connectError; }
  targetOptions() {
    return [
      { value: "", label: "None (offline)", selected: !this.#targetSelected, disabled: false },
      ...this.cells().map(cell => ({ value: cell.id, label: cell.label || cell.id,
        selected: cell.id === this.#connectedCell, disabled: false })),
    ];
  }
  targetStatusText(): string {
    return this.#connectError ?? this.#robotHost?.querySelector("#dense-target-state")?.textContent ?? "";
  }
  connectionSummary(): string {
    if (this.#selecting) return "connecting…";
    if (this.#connectError?.includes("mismatch")) return `${this.#select().value} · identity mismatch`;
    if (!this.#targetSelected) return "Not connected";
    return `${this.#connectedCell} · ${this.daemonReachable() ? "verified" : "unreachable"}`;
  }
  daemonReachable(): boolean {
    return machineReachable(this.#targetSelected, this.#status, this.statusAgeMs());
  }
  /** Pull copies authenticated Describe and the Picker model, never daemon status. */
  cellSettings(now = new Date()): CellSettings | null {
    return cellSettingsOf(this.#status, this.#capabilities, this.daemonReachable(), now);
  }

  /**
   * The settings plus the machine's machine_planning_calibration.json, fetched now. The bytes come
   * with the identity the machine hashed them under, so the pair cannot be
   * torn by a machine_planning_calibration.json that changed between the status and the fetch.
   * Throws when the machine cannot serve them: one that binds no robot
   * description (the caller keeps the status-only settings), or one whose
   * identity is invalid (the plan would be refused, so the pull is too).
   */
  async fetchCellSettings(now = new Date()): Promise<CellSettings | null> {
    const settings = this.cellSettings(now);
    if (!settings) return null;
    const body = await this.#deps.client.cell();
    if (!body.robotCell.valid) {
      throw new DenseExecutionError("robot_cell_unavailable", body.robotCell.error ?? "", 409);
    }
    return {
      ...settings,
      modelId: body.robotCell.modelId,
      robotDescriptionSha256: body.robotCell.robotDescriptionSha256,
      machinePlanningCalibrationSha256: body.robotCell.machinePlanningCalibrationSha256,
      cellFetchedAt: now.toISOString(),
      // The bytes themselves, for a plan made later with no cell to ask.
      machinePlanningCalibrationBase64: body.machinePlanningCalibrationBase64 || null,
      machinePlanningCalibrationSource: "target",
    };
  }
  loadedTrajectoryDigest(): string | null { return this.#status?.session?.trajectoryDigest || null; }
  networkSummary(): string {
    if (!this.#targetSelected) return "offline";
    const ms = this.#status?.roundTripMS;
    const lease = this.#status?.linkLeaseMS;
    return [lease === undefined ? "SDK lease unknown" : "SDK lease " + lease.toFixed(0) + " ms",
      ms === undefined ? "RTT measuring" : "RTT " + ms.toFixed(1) + " ms"].join(" · ");
  }
  clearFaultsAvailable(): boolean { return this.#targetSelected && this.#capabilities?.backend === "rt_core"; }
  clearingFaults(): boolean { return this.#clearingFaults; }
  clearFaultsMessage(): string { return this.#clearFaultsMessage; }
  dismissClearFaults(): void { this.#clearFaultsMessage = ""; this.render(); }
  clearFaultsDisabledReason(): string | null {
    if (!this.clearFaultsAvailable() || this.#cellPending) return "Connect to a RosieOS cell first";
    if (this.#inFlight) return "Another request is in progress";
    const age = this.statusAgeMs();
    if (!this.#status?.available || !this.#status.rtCore || age === null || age > STATUS_FRESHNESS_MS)
      return "Waiting for fresh controller status";
    if (this.#status.rtCore.coreArmed || this.#status.rtCore.coreEnableMask !== 0)
      return "Disarm the robot before clearing faults";
    if (this.robotMoving()) return "Stop motion before clearing faults";
    return null;
  }
  clearFaults(): void {
    if (this.clearFaultsDisabledReason()) return;
    this.#armConfirm = false;
    this.#clearingFaults = true;
    this.#clearFaultsMessage = "Checking fault recovery…";
    void this.#run("Clear faults", async (current) => {
      try {
        const result = await this.#deps.client.clearFaults();
        if (!current()) return null;
        const faults = result.status.rtCore?.operatorFeedback?.reset.faults ?? [];
        const remaining = faults.map(f => `${f.name}: ${f.recovery === "restart_required" ? "controller restart and recovery required" : f.recovery.replaceAll("_", " ")}`).join("; ");
        const inhibited = result.status.rtCore?.coreArmed === false && result.status.rtCore.coreEnableMask === 0;
        const posture = inhibited ? "Robot remains disarmed." : "Check controller status; disarmed state is not confirmed.";
        const cleared = result.accepted && !remaining && result.status.rtCore?.coreFaultMask === 0 && inhibited;
        this.#clearFaultsMessage = cleared
          ? "Faults cleared. Robot remains disarmed."
          : `Not cleared: ${remaining || result.error || result.status.daemonError || "controller did not confirm recovery"}. ${posture}`;
        this.#deps.status(this.#clearFaultsMessage, !cleared);
        return result.status;
      } catch (error) {
        if (current()) this.#clearFaultsMessage = `Clear faults failed: ${error instanceof Error ? error.message : String(error)}. Check controller status.`;
        throw error;
      } finally {
        if (current()) this.#clearingFaults = false;
      }
    });
  }
  arm(): void { void this.#arm(true); }
  /** Leave the confirm step and acknowledge a shown Arm refusal. Nothing is
   *  sent; the reason stays on the badge tooltip and in Logs. */
  cancelArm(): void {
    this.#armConfirm = false;
    this.#armRefusalAck = true;
    this.render();
  }
  armConfirmation(): boolean { return this.#armConfirm; }
  armRefused(): boolean {
    return this.#lastRequest?.request === "arm" && !!this.#lastRequest.error && !this.#armRefusalAck;
  }
  /** The header arm popover is open for the confirm step or an unacknowledged refusal. */
  armPopoverOpen(): boolean {
    return this.armConfirmation() || (this.#view?.connected === true && this.armRefused());
  }
  armReason(): string {
    if (this.#view?.connected && !this.#view.canArm) return this.#view.steps[3]!.summary;
    // A drive-state refusal ends the attempt after Arm was accepted; say why.
    const ended = this.#status?.rtCore?.operatorFeedback?.armEnded ?? [];
    return this.#view?.arm.transportRetry ?? (this.#lastRequest?.error
      ? armRefusalText(this.#view?.armBlockers ?? [], this.#lastRequest.error)
      : ended.length ? `Arm ended: ${ended.join("; ")}` : this.#view?.arm.detail ?? "Connect to a cell first");
  }

  /** Selection preserves action epochs and retires jog; failure keeps the old cell. */
  selectTarget(id: string): void { void this.#selectCell(id); }
  async #selectCell(id: string): Promise<void> {
    if (this.#selecting || this.#inFlight) return;
    this.#selecting = true;
    this.#cellError = null;
    this.#connectError = null;
    this.#armConfirm = false;
    this.#jointJog?.stop("cell_change");
    this.#cartesianJog?.end("cell_change");
    this.render();
    try {
      if (id) {
        this.#select().value = id;
        this.#highlightCell();
        await this.#connectCell();
      } else {
        this.#inFlight = true;
        const epoch = ++this.#actionEpoch;
        const caps = await this.#deps.client.selectCell("");
        if (epoch !== this.#actionEpoch) return;
        this.#capabilities = caps;
        this.#connectedCell = "";
        this.#connectedPosture = null;
        this.#cellPending = false;
        this.#deps.client.observeTarget(undefined);
        this.#setTarget(false);
        this.#select().value = "";
      }
    } catch (error) {
      this.#cellError = error instanceof Error ? error.message : String(error);
      this.#connectError = this.#cellError;
      this.#deps.status(this.#cellError, true);
    } finally {
      this.#selecting = false;
      this.#inFlight = false;
      this.render();
    }
  }
  async refreshCells(): Promise<void> {
    try {
      const body = await this.#deps.client.cells();
      this.#probes = body.cells;
      if (body.activeCell !== this.#connectedCell) {
        if (!body.activeCell) {
          this.#actionEpoch++;
          this.#connectedCell = "";
          this.#connectedPosture = null;
          this.#setTarget(false);
        } else await this.#discover();
      }
    } catch (error) {
      this.#cellError = error instanceof Error ? error.message : String(error);
      this.#deps.status(this.#cellError, true);
    }
    this.changed.emit();
  }
  async addCell(input: DenseCellInput): Promise<boolean> {
    try {
      this.#probes = (await this.#deps.client.addCell(input)).cells;
      this.#cellError = null;
      this.changed.emit();
      return true;
    } catch (error) {
      this.#cellError = error instanceof Error ? error.message : String(error);
      this.#deps.status(this.#cellError, true);
      this.changed.emit();
      return false;
    }
  }
  async removeCell(id: string): Promise<void> {
    try { await this.#deps.client.removeCell(id); await this.refreshCells(); }
    catch (error) {
      this.#cellError = error instanceof Error ? error.message : String(error);
      this.#deps.status(this.#cellError, true);
      this.changed.emit();
    }
  }

  /** Editing is locked while the daemon is staged, playing or paused on
   *  this program; the reason is the message the host shows. */
  lockReason(): string | null {
    if (this.#planStartPhase) return "moving to the selected plan start (STOP to cancel)";
    if (this.#runPress) return "running the program (STOP to cancel)";
    const state = denseExecutionState(this.#status);
    if (!this.#targetSelected || !this.#status?.session) return null;
    if (state === "playing" || state === "paused" || state === "staged" || state === "loaded") {
      return `the joint daemon is ${state} on this program (STOP to release)`;
    }
    return null;
  }

  /** What cell-sync.ts programSwitchLock reads: the robot's session, a loop
   *  between its cycles, and this panel's own press still running. */
  switchLockInput(): { sessionState: string; looping: boolean; busy: string } {
    const loop = this.#targetSelected ? this.#status?.loop?.state : undefined;
    return {
      sessionState: this.#targetSelected && this.#status?.session ? denseExecutionState(this.#status) : "",
      looping: loop === "running" || loop === "finishing",
      busy: this.#planStartPhase ? "moving to the plan start" : this.#runPress ? "running the program" : "",
    };
  }

  targetSelected(): boolean {
    return this.#targetSelected;
  }

  /** The last derived path, for hosts that paint the header from it. */
  flowView(): MachineFlowView | null {
    return this.#view;
  }

  cartesianJog(): MachineCartesianJog | null { return this.#cartesianJog; }
  /** Hiding Jog releases both hold lanes, preventing hidden input from continuing. */
  endJogHolds(cause: JogEndCause = "ui_navigation"): void { this.#jointJog?.end(cause); this.#cartesianJog?.end(cause); }
  /** The chosen catalogue model stays independent of the arm authoring project,
   * preventing Connect from substituting the last previewed arm for a bench. */
  /** The connected machine's axis ids in axis order (J1…), or none. */
  axisIds(): string[] { return this.#capabilities?.rtCore?.axes.map(axis => axis.id) ?? []; }
  /** The previewed plan at the playback cursor, or null when stopped.
   *  Display only: hold, GO and zero-all keep using the machine. */
  previewSample(sample: SimulationSample | null): void { this.#jointJog?.previewSample(sample); }
  /** The shared joint-space move; the calling pane supplies its displayed Speed %. */
  zeroAllJoints(fraction?: number): Promise<void> { return this.#jointJog?.zeroAll(fraction) ?? Promise.resolve(); }
  zeroJointsGate(): string | null { return this.#jointJog ? this.#jointJog.zeroGate() : "Connect a machine first"; }
  zeroJointsLabel(): string { return this.#jointJog?.zeroLabel() ?? "All joints to 0°"; }
  /** The capabilities in hand, for consumers that convert drive units. */
  capabilities(): DenseCapabilities | null { return this.#capabilities; }
  /** Capture uses complete native evidence, never the viewer's held values. */
  teachObservation(): { pose: RobotPoseSnapshot; settings: CellSettings; cellId: string } {
    return teachObservationOf({
      reachable: this.daemonReachable(), busy: this.#teachBusy(), status: this.#status, capabilities: this.#capabilities,
      still: this.#teachStill, settings: this.cellSettings(), observedAtMs: Date.now() - (this.statusAgeMs() ?? 0),
      cellModel: this.cellModel(), cellId: this.#connectedCell,
    });
  }
  /** A request in flight, a held jog or a playing program: no capture now. */
  #teachBusy(): boolean {
    return this.#inFlight || Boolean(this.#jointJog?.active) || Boolean(this.#cartesianJog?.busy) || this.robotMoving();
  }
  cellModel(): RobotModelID { return this.#catalogueInvalid ? "unknown" : this.#status?.target?.model_id ?? this.#cellModel ?? this.#deps.getProject().robot.model; }
  get fetcher(): typeof fetch { return this.#deps.client.fetcher; }

  /** A fresh status from a native target while the panel holds no axis
   *  description: the capabilities answer was taken while the server's
   *  probe had failed (the cell list said "unreachable"), so ask again, at
   *  most every two seconds, and let the status say what the target is. */
  #recoverCapabilities(status: DenseStatus): void {
    const target = status.target;
    if (!target || status.backend !== "rt_core" || !status.rtCore) return;
    if (this.#capabilities?.rtCore?.axes.length) return;
    const now = Date.now();
    if (now - this.#capabilitiesRequestedAtMs < CAPABILITIES_RETRY_MS) return;
    this.#capabilitiesRequestedAtMs = now;
    if (this.#connectedPosture && this.#connectedPosture.backend !== target.backend) {
      this.#connectedPosture = target;
      this.#robotHost?.querySelector<HTMLElement>("#dense-target-state")?.replaceChildren(`${target.label} · ${target.model_id} · ${target.backend} · ${target.host}`);
    }
    this.#deps.status("Machine answered status without an axis description in hand; requesting its capabilities again", false);
    void this.#deps.client.capabilities().then(caps => {
      if (this.#connectedCell !== target.cell_id) return;
      this.#capabilities = caps;
      this.render();
    }).catch(error => this.#deps.status(`Capabilities request failed: ${error instanceof Error ? error.message : String(error)}`, true));
  }

  /** Whether the viewport should follow this machine now, and its caption.
   *  Derived from the same status and freshness bound as the armed fact. */
  livePoseView(): LivePoseView {
    return deriveLivePose({
      connected: this.#targetSelected,
      status: this.#status,
      capabilities: this.#capabilities,
      statusAgeMs: this.statusAgeMs(),
      nowMs: Date.now(),
      previous: this.#lastPose,
      cellLabel: this.#view?.cellLabel ?? this.#status?.target?.label ?? this.#capabilities?.cell ?? "",
      fallbackModelId: this.#deps.getProject().robot.model,
    });
  }

  /** Retain selected posture across pending switches and failed requests so
   * local preview mode cannot hide an already selected real machine. */
  executionPosture(): ExecutionPosture | null {
    if (this.#connectedPosture) return this.#connectedPosture;
    if (!this.#targetSelected) return null;
    const manifest = this.#capabilities?.rtCore;
    return manifest ? { backend: manifest.nativeBackend, simulation: manifest.simulation } : { backend: "unknown" };
  }

  /** Why Plan must wait for this tab's own motion, or "": a held jog, a GO,
   *  Cartesian step or Go to plan start in flight, or a program playing. */
  planMotionBlocked(): string {
    return planMotionBlocked({
      holding: Boolean(this.#jointJog?.active) || Boolean(this.#cartesianJog?.busy),
      moving: Boolean(this.#jointJog?.moveBusy) || this.#planStartPhase === "moving" || this.#runPress !== null || this.robotMoving(),
    }, "desktop");
  }

  /** The daemon is playing or paused, a program or a go-home. */
  robotMoving(): boolean {
    const state = denseExecutionState(this.#status);
    return this.#targetSelected && (state === "playing" || state === "paused");
  }

  /** The software STOP from anywhere in the app (the header bar mirrors the panel's). */
  stop(): void { void this.#stop("STOP"); }
  /** DISARM from anywhere in the app. */
  disarm(): void { void this.#arm(false); }

  /**
   * @param robotHost the Machine panel: the stepper, arm controls, STOP.
   * @param playbackHost the dock RUN controls, kept in their supplied host.
   * @param jogHost the Jog panel, which carries the joint jog table
   *   (`#dense-joint-jog`); defaults to the robot host.
   */
  mount(robotHost: HTMLElement, playbackHost: HTMLElement, jogHost: HTMLElement | null = null): void {
    this.#robotHost = robotHost;
    this.#playbackHost = playbackHost;
    this.#jogHost = jogHost ?? robotHost;
    // `lead` sits between the head and the summary: the Arm step's state in
    // large words comes before its explanation.
    const step = (id: MachineStepID, index: number, title: string, body: string, lead = ""): string => `
      <li class="machine-step" data-step="${id}" data-state="pending">
        <span class="machine-step-mark" data-step-mark="${id}" aria-hidden="true">${index}</span>
        <div class="machine-step-body">
          <div class="machine-step-head">
            <span class="machine-step-title">${title}</span>
            <span class="machine-step-state" data-step-state="${id}"></span>
          </div>
          ${lead}
          <p class="machine-step-summary" data-step-summary="${id}" aria-live="polite"></p>
          ${body}
        </div>
      </li>`;
    robotHost.innerHTML = `
      <div class="machine-flow" data-posture="none" data-connected="false">
        <p id="dense-selection-notice" class="field-help" aria-live="polite" hidden></p>
        <div id="dense-runtime" class="machine-posture" data-mode="unknown" hidden title="What the selected machine is: a described simulation drives no motor, a real cell does">RTCore ?</div>
        <ol class="machine-steps">
          ${step("choose", 1, "Choose a machine", `
            <label class="field machine-field" data-setup-item="robot.target">
              <span>Machine</span>
              <select id="dense-target" aria-describedby="dense-target-state">
                <option value="">None (browser preview only)</option>
              </select>
            </label>
            <p id="dense-cell-mode" class="field-help"></p>`)}
          ${step("connect", 2, "Connect", `
            <p id="dense-target-state" class="field-help machine-detail" aria-live="polite">Joint daemon · checking</p>
            <div id="dense-cell-controls" class="machine-step-actions" hidden>
              <button id="dense-cell-connect" class="mini-button" type="button" title="Describe the machine over mutual TLS and verify its identity. Grants no control; moves nothing.">Connect to machine</button>
              <p id="dense-cell-connection" class="machine-code" aria-live="polite"></p>
            </div>`)}
          ${step("home", 3, "Home", `
            <strong id="dense-home-state" aria-live="polite"></strong>
            <div id="dense-home-joints" class="machine-detail dense-home-joints" aria-live="polite"></div>
            <div id="dense-home-details" class="field-help" aria-live="polite"></div>
            <div class="machine-step-actions">
              <button id="dense-home" class="mini-button" type="button" title="Start disarmed. Home briefly energises the drives; settling movement may occur.">Home</button>
            </div>`)}
          ${step("arm", 4, "Arm", `
            <div id="dense-arm" class="dense-arm machine-step-actions" hidden>
              <button id="dense-arm-on" class="mini-button" type="button" title="Arm the servos through RTCore">ARM</button>
              <button id="dense-arm-confirm" class="mini-button is-warn" type="button" hidden title="Second click: acquire control, enable and arm the drives. Motors energise.">Confirm: energise motors</button>
              <button id="dense-arm-cancel" class="mini-button" type="button" hidden>Cancel</button>
            </div>`,
            `<strong id="dense-arm-state" class="dense-arm-state" data-armed="unknown" aria-live="polite" hidden>UNKNOWN: no fresh status from the machine</strong>`)}
          ${step("drive", 5, "Drive", `
            <div id="dense-run-slot" class="machine-run-slot"></div>
            <div id="dense-motors" class="dense-motors" hidden></div>
            <div id="dense-operator-feedback" class="inspector-note machine-feedback" aria-live="polite" hidden></div>`)}
        </ol>
        <div id="dense-guard" class="machine-guard" hidden>
          <button id="dense-stop" class="dense-stop" type="button" title="${STOP_TITLE}">STOP</button>
          <button id="dense-arm-off" class="mini-button" type="button" title="Disarm: Stop, then release control of the machine">DISARM</button>
        </div>
      </div>
    `;
    playbackHost.innerHTML = `
      <label class="machine-run-label" for="dense-run-mode">Run</label>
      <select id="dense-run-mode" aria-label="Run mode" title="Choose process outputs for the next Load; no replanning required">
        <option value="dry_run">Dry run · process outputs off</option>
        <option value="production">Program outputs · as planned</option>
      </select>
      <select id="dense-run-repeat" aria-label="Repeat" title="Run once, a number of times, or until stopped. A repeat is planned with the planner's return to the program's start: plan again after changing it">
        <option value="once">Run once</option>
        <option value="count">Run N times</option>
        <option value="loop">Loop until stopped</option>
      </select>
      <input id="dense-run-count" type="number" min="2" max="${MAX_LOOP_COUNT}" step="1" value="5" aria-label="Cycles" title="Cycles to run" hidden>
      <button id="dense-run-program" class="mini-button primary" type="button">▶ Run program</button>
      <button id="dense-halt" class="icon-button" type="button" aria-label="Stop the robot" title="Stop the robot and release the lease">■</button>
      <p id="dense-run-result" class="field-help" aria-live="polite" hidden></p>
      <p id="dense-run-loop" class="field-help dense-run-loop" aria-live="polite" hidden></p>
      <button id="dense-run-finish" class="mini-button" type="button" hidden>Finish cycle</button>
      <div id="dense-outputs" class="dense-outputs" data-tone="idle" aria-live="polite" hidden></div>
      <details id="dense-run-advanced" class="dense-run-advanced">
        <summary title="Go to plan start, Load to robot and Play as separate presses">Advanced · step by step</summary>
        <div class="dense-run-advanced-body">
          <button id="dense-plan-start" class="mini-button" type="button">Go to plan start</button>
          <p id="dense-plan-start-result" class="field-help" aria-live="polite" hidden></p>
          <button id="dense-load" class="mini-button" type="button" title="Fetch the compiled trajectory into the joint daemon">Load to robot</button>
          <button id="dense-play" class="play-button" type="button" aria-label="Play on the robot" title="Play on the robot">▶</button>
        </div>
      </details>
      <span id="dense-readout" class="dense-readout" aria-live="polite">no target</span>
      <p id="dense-load-blocker" class="field-help" aria-live="polite" hidden></p>
      <button id="dense-review-home" class="mini-button" type="button" hidden>Review Home requirement</button>
      <button id="dense-use-dry-run" class="mini-button" type="button" hidden title="Select process outputs off, then Load and Play separately. No replanning required.">Use dry run</button>
      <div id="dense-load-progress" class="dense-load-progress" hidden>
        <div class="dense-load-progress-bar" role="progressbar" aria-valuemin="0" aria-valuemax="100" aria-valuenow="0"><div class="dense-load-progress-fill"></div></div>
        <span class="dense-load-progress-text" aria-live="polite"></span>
      </div>
      <div id="dense-test-program" class="dense-test-program" hidden>
        <button id="dense-test-load" class="mini-button" type="button">Load test program (.rdt)…</button>
        <input id="dense-test-file" type="file" accept=".rdt,.json,application/json" multiple hidden>
        <button id="dense-test-play" class="play-button" type="button" aria-label="Play test program">▶ Play test program</button>
        <p id="dense-test-label" class="field-help dense-test-label" aria-live="polite"></p>
      </div>
    `;
    // The dock owns RUN placement; keep its host separate from the Machine steps.

    const select = this.#select();
    let jogHostElement = this.#jogHost.querySelector<HTMLElement>("#dense-joint-jog");
    if (!jogHostElement) {
      jogHostElement = document.createElement("div");
      jogHostElement.id = "dense-joint-jog";
      jogHostElement.className = "dense-joint-jog";
      this.#jogHost.append(jogHostElement);
    }
    this.#jointJog = new JointJogPanel(jogHostElement, jointJogAPI(this.#deps.client.fetcher), text => this.#deps.refusal?.(text), {
      home: () => void this.#home(),
      homeAxis: index => void this.#home([index]),
      homeDisabled: () => this.#homeDisabledReason(),
      confirmRecover: confirmation => this.#deps.confirmRecover?.(confirmation) ?? Promise.resolve(false),
    });
    this.#cartesianJog = new MachineCartesianJog(() => { this.#deps.onFlowChange?.(this.#view!); }, this.#deps.client.fetcher, text => this.#deps.refusal?.(text));
    this.#deps.client.onSelectionChanged(status => {
      this.#selectionChanged = true;
      this.#armConfirm = false;
      if (!status) { this.#actionEpoch++; this.#inFlight = false; this.#pendingRequest = null; }
      this.#jointJog?.stop("selection_changed");
      this.#cartesianJog?.end("selection_changed");
      if (status) {
        this.#apply(status);
        if (!this.#targetSelected) this.#setTarget(true);
      } else this.render();
    });
    select.addEventListener("change", () => {
      if (this.#cells) this.#highlightCell();
      else this.#setTarget(select.value !== "");
    });
    this.#robotHost.querySelector<HTMLButtonElement>("#dense-cell-connect")!.addEventListener("click", () => void this.#connectCell());
    this.#button("#dense-load").addEventListener("click", () => void this.#load());
    this.#button("#dense-plan-start").addEventListener("click", () => void this.#goToPlanStart());
    this.#button("#dense-review-home").addEventListener("click", () => {
      const step = this.#element('[data-step="home"]');
      step.setAttribute("tabindex", "-1");
      step.scrollIntoView({ block: "center" });
      step.focus({ preventScroll: true });
    });
    const runMode = this.#element("#dense-run-mode") as HTMLSelectElement;
    runMode.addEventListener("change", () => {
      if (!this.#runModeLocked()) this.#dryRun = runMode.value !== "production";
      this.render();
    });
    this.#button("#dense-use-dry-run").addEventListener("click", () => {
      const button = this.#button("#dense-use-dry-run");
      if (button.hidden || button.disabled || this.#runModeLocked()) return;
      this.#dryRun = true;
      this.render();
    });
    this.#button("#dense-play").addEventListener("click", () => void this.#play());
    this.#button("#dense-run-program").addEventListener("click", () => void this.runProgram());
    this.#button("#dense-run-finish").addEventListener("click", () => void this.finishLoop());
    const repeat = this.#element("#dense-run-repeat") as HTMLSelectElement;
    repeat.addEventListener("change", () => {
      if (!this.#runModeLocked() && (repeat.value === "once" || repeat.value === "count" || repeat.value === "loop")) this.#repeatMode = repeat.value;
      this.render();
    });
    const count = this.#element("#dense-run-count") as HTMLInputElement;
    count.addEventListener("change", () => {
      const value = Math.round(Number(count.value));
      if (!this.#runModeLocked() && Number.isFinite(value)) this.#repeatCount = Math.min(MAX_LOOP_COUNT, Math.max(2, value));
      this.render();
    });
    this.#button("#dense-test-load").addEventListener("click", () => {
      if (!this.#button("#dense-test-load").disabled) (this.#element("#dense-test-file") as HTMLInputElement).click();
    });
    this.#element("#dense-test-file").addEventListener("change", () => {
      const input = this.#element("#dense-test-file") as HTMLInputElement;
      const files = Array.from(input.files ?? []);
      input.value = "";
      void this.loadTestProgram(files);
    });
    this.#button("#dense-test-play").addEventListener("click", () => void this.#playTestProgram());
    this.#button("#dense-halt").addEventListener("click", () => void this.#stop("stop"));
    this.#button("#dense-stop").addEventListener("click", () => void this.#stop("STOP"));
    this.#button("#dense-arm-on").addEventListener("click", () => void this.#arm(true));
    this.#button("#dense-arm-confirm").addEventListener("click", () => void this.#arm(true));
    this.#button("#dense-arm-cancel").addEventListener("click", () => { this.#armConfirm = false; this.render(); });
    this.#button("#dense-arm-off").addEventListener("click", () => void this.#arm(false));
    this.#button("#dense-home").addEventListener("click", () => void this.#home());
    // Accepted Cartesian steps belong to the server even when this tab exits.
    // Only held input ends on pagehide; bounded steps retain their completion.
    // So does an unattended loop.
    window.addEventListener("pagehide", () => {
      if (!this.#cartesianJog?.stepBusy && !unattendedLoopRunning(this.#status?.loop) && this.#status?.session && LIVE_STATES.has(denseExecutionState(this.#status))) {
        void this.#deps.client.stop().catch(() => undefined);
      }
    });
    void this.#discover();
    this.render();
  }

  render(): void {
    if (!this.daemonReachable()) this.#setHighlight(null);
    if (!this.#robotHost || !this.#playbackHost) return;
    const notice = this.#robotHost.querySelector<HTMLElement>("#dense-selection-notice");
    if (notice) { notice.hidden = !this.#selectionChanged; notice.textContent = this.#selectionChanged ? TARGET_SELECTION_CHANGED : ""; }
    const project = this.#deps.getProject();
    if (this.#runModeProject !== project.identity.id) {
      this.#runModeProject = project.identity.id;
      this.#dryRun = project.program.executionMode !== "production";
    }
    const runMode = this.#element("#dense-run-mode") as HTMLSelectElement;
    runMode.value = this.#dryRun ? "dry_run" : "production";
    runMode.disabled = this.#runModeLocked();
    const status = this.#status;
    const session = status?.session ?? null;
    const record = project.denseTrajectory;
    const native = this.#capabilities?.backend === "rt_core";
    const run = deriveRunControls({
      targetSelected: this.#targetSelected, cellPending: this.#cellPending, inFlight: this.#inFlight, native, status, record,
      projectRevision: project.identity.revision, dryRun: this.#dryRun, daemonReachable: this.daemonReachable(),
      lastRequestError: this.#lastRequest?.error ?? null, planStartActive: !!this.#planStartPhase || this.#runPress !== null,
    });
    const { state, live, current } = run;
    const highlighted = this.#cells?.find(cell => cell.cell_id === this.#select().value) ?? null;
    const connectedCell = this.#cells?.find(cell => cell.cell_id === this.#connectedCell) ?? null;
    // A confirmation is only pending while Arm is still the next action.
    if (this.#armConfirm && (status?.armed === true || live || !this.#targetSelected || this.#cellPending)) this.#armConfirm = false;

    const view = deriveMachineFlow({
      catalogue: this.#cells,
      catalogueError: this.#catalogueError,
      highlighted,
      connectedCell: this.#connectedCell,
      connectedLabel: status?.target?.label || connectedCell?.label || "",
      // A pending switch names the highlighted cell's model, not the
      // connected machine's; a bench never changes the arm project.
      modelId: unknownRobotModelReason(this.cellModel()) ? this.cellModel() : this.#cells && this.#cellPending && highlighted
        ? (highlighted.models.includes(project.robot.model) ? project.robot.model : highlighted.models[0]!)
        : status?.target?.model_id ?? this.cellModel(),
      connected: this.#targetSelected,
      cellPending: this.#cells !== null && this.#cellPending,
      connectError: this.#connectError,
      inFlight: this.#inFlight,
      posture: this.executionPosture(),
      capabilities: this.#capabilities,
      status,
      armConfirmPending: this.#armConfirm,
      statusAgeMs: this.statusAgeMs(),
      pendingRequest: this.#pendingRequest,
      lastRequest: this.#lastRequest,
    });
    this.#view = view;
    if (!view.canArm) this.#armConfirm = false;
    // `armed` here is the fact's boolean: true ARMED, false DISARMED, null in
    // transition or unknown. Every gate below reads it or the fact itself.
    const armed = view.armed;
    this.#paintSteps(view);
    this.#paintHomeStatus();

    this.#playbackHost.hidden = false;
    this.#element("#dense-arm").hidden = !this.#targetSelected;
    this.#element("#dense-guard").hidden = !this.#targetSelected;
    this.#element("#dense-motors").hidden = !this.#targetSelected;

    const load = this.#button("#dense-load");
    const loadNotice = this.#element("#dense-load-blocker");
    loadNotice.textContent = run.loadBlocker;
    loadNotice.hidden = !run.loadBlocker;
    this.#button("#dense-review-home").hidden = !this.#targetSelected || !run.homeRequired;
    load.disabled = run.load.disabled;
    load.title = run.load.title;
    load.textContent = run.load.text;
    const dryRun = this.#button("#dense-use-dry-run");
    dryRun.hidden = !run.offerDryRun;
    dryRun.disabled = this.#runModeLocked();
    const play = this.#button("#dense-play");
    const feedbackHost = this.#element("#dense-operator-feedback");
    feedbackHost.hidden = !native || !this.#targetSelected;
    const feedback = status?.rtCore?.operatorFeedback;
    const refusal = this.#connectError ?? this.#lastRequest?.error;
    if (native && feedback) renderOperatorFeedback(feedbackHost, feedback, refusal ?? "");
    const feedbackReason = [
      ...(native ? feedback?.armEnded?.map(reason => `Arm ended: ${reason}`) ?? [] : []),
      feedback?.note ?? "",
      feedback?.rejection ? feedback.rejection.text + " [" + feedback.rejection.code + "]" : "",
      feedback?.stale ? feedback.stale.text + " [" + feedback.stale.code + "]" : "",
      ...(feedback?.reset.faults.map(fault => `Controller check: ${fault.name}; axes mask 0x${fault.axis_mask.toString(16)}; recovery ${fault.recovery}; outcome ${fault.outcome || "not reported"}`) ?? []),
      ...(feedback?.axes.flatMap(axis => [axis.homeLoss, axis.alarmText]) ?? []),
    ].filter(Boolean).join("; ");
    // Brake-hold and disarm-motion warnings stay visible but are not refusals:
    // they gate no step and are logged as ordinary status, once per change.
    const warnings = native ? (feedback?.warnings ?? []).join("; ") : "";
    feedbackHost.hidden = !this.#targetSelected || (!feedbackReason && !warnings);
    if (feedbackReason && feedbackReason !== this.#loggedFeedback) this.#deps.refusal?.(feedbackReason);
    this.#loggedFeedback = feedbackReason;
    if (warnings && warnings !== this.#loggedWarnings) this.#deps.status(`Warning: ${warnings}`, false);
    this.#loggedWarnings = warnings;
    play.disabled = run.play.disabled;
    play.title = run.play.title;
    this.#button("#dense-halt").disabled = run.halt.disabled;
    this.#button("#dense-stop").disabled = run.stop.disabled;
    this.#renderTestProgram(native);

    this.#element("#dense-readout").textContent = this.#readoutText();
    this.#element("#dense-readout").title = this.#readoutTitle();
    this.#renderRuntime();

    const cell = view.cellLabel;
    const physical = view.posture === "real" || view.posture === "unknown";
    // The Arm step's large words: the fact's headline, the fact's detail as
    // the tooltip. No wording is composed here; the step summary, the pill,
    // the run row and the Jog caption draw on the same `view.arm`.
    const armState = this.#element("#dense-arm-state");
    armState.hidden = !this.#targetSelected;
    armState.textContent = view.arm.headline;
    armState.title = `${cell}: ${view.arm.detail}${view.arm.transportRetry ? ` ${view.arm.transportRetry}` : ""}${physical ? " REAL target: physical motion is possible." : " Selected simulation target."}`;
    armState.dataset.armed = view.arm.state;
    armState.dataset.posture = physical ? "real" : "simulation";
    this.#element("#dense-arm").dataset.posture = armState.dataset.posture;
    const armOn = this.#button("#dense-arm-on");
    armOn.textContent = physical ? `Arm ${cell}` : `Arm ${cell} (simulation)`;
    armOn.title = unknownRobotModelReason(this.cellModel(), status?.target?.model_id) || `Arm ${cell}${physical ? ": REAL target, physical motion is possible. A second click confirms." : ": selected simulation target, no physical output"}`;
    armOn.hidden = this.#armConfirm;
    armOn.disabled = !this.#targetSelected || this.#cellPending || this.#inFlight || armed === true || live || !view.canArm;
    armOn.className = "mini-button";
    const confirm = this.#button("#dense-arm-confirm");
    confirm.hidden = !this.#armConfirm;
    confirm.disabled = !this.#armConfirm || this.#inFlight || !view.canArm;
    confirm.textContent = `Confirm: energise motors on ${cell}`;
    this.#button("#dense-arm-cancel").hidden = !this.#armConfirm;
    this.#button("#dense-arm-off").disabled = !this.#targetSelected || this.#inFlight || armed === false || live;
    const home = this.#button("#dense-home");
    const unhomed = unhomedMappedAxes(status?.axes ?? null, status?.daemon?.mappedAxisMask ?? null).length;
    // Legacy GO HOME is a motion: it needs the drives observed ARMED.
    home.disabled = !this.#targetSelected || this.#cellPending || this.#inFlight || live || armed !== true || unhomed > 0;
    home.textContent = "GO HOME";
    if (this.#cells) {
      this.#select().disabled = live || this.#inFlight;
      this.#element("#dense-cell-controls").hidden = false;
      const connect = this.#robotHost.querySelector<HTMLButtonElement>("#dense-cell-connect")!;
      connect.disabled = live || this.#inFlight || !highlighted || highlighted.reason === "cell_configuration_missing";
      connect.className = "mini-button";
      connect.textContent = this.#targetSelected && this.#cellPending && highlighted ? `Connect to ${highlighted.label}` : "Connect to machine";
    }
    home.title = armed !== true
      ? "ARM first: go home is a motion"
      : unhomed
        ? `${unhomed} axis${unhomed === 1 ? "" : "es"} unhomed: the daemon has no start pose for them`
        : "Move every mapped axis to the reset pose along a smooth profile, under the lease";
    if (this.#capabilities?.backend === "rt_core") {
      this.#button("#dense-arm-off").disabled = !this.#targetSelected;
      home.textContent = "Home";
      // Native Home is offered only while the path says it is the step to
      // take: drives observed DISARMED, a described axis without valid Home,
      // no blocking fault, nothing in flight.
      home.disabled = !this.#targetSelected || this.#cellPending || this.#inFlight || live || view.arm.state !== "disarmed" || !this.#capabilities.rtCore?.nativeHome || !view.canHome;
      home.title = view.canHome ? "Start disarmed. Home briefly energises the drives; settling movement may occur." : view.steps[2]!.summary;
    }
    home.hidden = !this.#targetSelected;
    home.className = "mini-button";
    this.#renderMotors();
    this.#jointJog?.update(this.#targetSelected, this.#capabilities, status, this.#inFlight && this.#planStartPhase !== "moving", view.arm, this.cellModel(), this.statusAgeMs(), this.#pendingHomeAxes);
    this.#cartesianJog?.update(this.#targetSelected, this.#capabilities, status, this.#inFlight || view.arm.state !== "armed", (this.statusAgeMs() ?? Infinity) <= STATUS_FRESHNESS_MS, this.cellModel());
    const planStart = this.#button("#dense-plan-start");
    this.#syncPlanStartPosition();
    const atStart = !this.#planStartPhase && !live && this.#atPlanStart();
    if (native && run.loaded && !atStart) {
      play.disabled = true;
      if (this.daemonReachable()) play.title = PLAY_NEEDS_PLAN_START;
    }
    const startGate = !record || !current ? "Select a current plan first" : !this.#deps.readTrajectory ? "Plan data is unavailable" : this.#jointJog?.zeroGate();
    planStart.disabled = atStart || !native || this.#cellPending || this.#inFlight || live || !!startGate;
    planStart.classList.toggle("is-at-plan-start", atStart);
    planStart.textContent = this.#planStartPhase === "reading" ? "Reading plan start…" : this.#planStartPhase === "moving" ? "Moving to plan start…" : atStart ? "✓ At plan start" : "Go to plan start";
    planStart.title = atStart ? "All joint positions match this plan's start within the machine's reported completion tolerances. Confirmed from fresh feedback." : startGate || "Move all joints to this plan's first position at the selected Jog speed. Arm first; STOP cancels. Process outputs stay off.";
    const startResult = this.#element("#dense-plan-start-result");
    startResult.textContent = this.#planStartResult;
    startResult.hidden = !this.#planStartResult;
    const loop = status?.loop ?? null;
    const runControl = deriveRunProgram({
      controls: run, native, targetSelected: this.#targetSelected, cellPending: this.#cellPending, running: this.#runPress !== null,
      record, session, loop, repeat: this.repeat(),
    });
    const repeatSelect = this.#element("#dense-run-repeat") as HTMLSelectElement;
    repeatSelect.value = this.#repeatMode;
    repeatSelect.disabled = this.#runModeLocked() || loop?.state === "running" || loop?.state === "finishing";
    const countInput = this.#element("#dense-run-count") as HTMLInputElement;
    countInput.hidden = this.#repeatMode !== "count";
    countInput.value = String(this.#repeatCount);
    countInput.disabled = repeatSelect.disabled;
    const loopLine = this.#element("#dense-run-loop");
    loopLine.textContent = loopText(loop);
    loopLine.hidden = !loop;
    this.#paintOutputs(this.#targetSelected ? status?.outputs ?? null : null);
    const finish = finishControl(loop);
    const finishButton = this.#button("#dense-run-finish");
    finishButton.hidden = !finish.visible;
    finishButton.disabled = finish.disabled;
    finishButton.title = finish.title;
    const runButton = this.#button("#dense-run-program");
    runButton.textContent = `▶ ${runControl.text}`;
    runButton.title = runControl.title;
    runButton.disabled = runControl.disabled || (this.#inFlight && this.#runPress === null);
    runButton.dataset.again = String(runControl.again);
    const runResult = this.#element("#dense-run-result");
    runResult.textContent = this.#runNote;
    runResult.hidden = !this.#runNote;
    this.#paintJogHost(view);

    const locked = this.lockReason() !== null;
    if (locked !== this.#wasLocked) {
      this.#wasLocked = locked;
      this.#deps.onLockChange();
    }
    for (const selector of ['[data-step="choose"]', "#dense-cell-controls", "#dense-runtime", "#dense-arm", "#dense-guard", "#dense-arm-state"]) {
      this.#element(selector).hidden = true;
    }
    if (refusal) {
      const admission = processIOAdmissionMessage(refusal);
      // The scoped feedback already contains this admission refusal.
      if (!(admission && native && feedback)) {
        if (feedbackHost.hidden) feedbackHost.textContent = machineReasonText(refusal);
        else { const line = document.createElement("div"); line.textContent = machineReasonText(refusal); feedbackHost.append(line); }
      }
      feedbackHost.hidden = false;
    }
    this.#deps.onFlowChange?.(view);
    this.changed.emit();
  }

  /** Marks, states and summaries of the five steps; a checkmark is the only
   *  ornament, and only a step the server confirmed done gets it. */
  #paintSteps(view: MachineFlowView): void {
    const flow = this.#element(".machine-flow");
    flow.dataset.posture = view.posture;
    flow.dataset.connected = String(view.connected);
    flow.dataset.current = view.current ?? "";
    for (const [index, id] of STEP_IDS.entries()) {
      const step = view.steps[index]!;
      const item = this.#element(`[data-step="${id}"]`);
      item.dataset.state = step.state;
      const mark = this.#element(`[data-step-mark="${id}"]`);
      mark.textContent = step.state === "done" ? "✓" : step.state === "blocked" ? "!" : String(index + 1);
      const stateLabel = this.#element(`[data-step-state="${id}"]`);
      stateLabel.textContent = step.state === "done" ? "done" : step.state === "current" ? "next" : step.state === "blocked" ? "blocked" : "";
      const summary = this.#element(`[data-step-summary="${id}"]`);
      summary.textContent = step.summary;
      summary.title = step.code ?? "";
    }
  }

  #paintHomeStatus(): void {
    const native = this.#capabilities?.backend === "rt_core";
    const heading = this.#element("#dense-home-state"), rows = this.#element("#dense-home-joints"), details = this.#element("#dense-home-details");
    heading.hidden = rows.hidden = details.hidden = !native;
    if (!native) return;
    const home = deriveHomeStatus({ status: this.#status, capabilities: this.#capabilities,
      connected: this.#targetSelected, statusAgeMs: this.statusAgeMs(), pendingAxes: this.#pendingHomeAxes });
    const known = home.state !== "stale" && home.state !== "unavailable";
    heading.textContent = home.label + (known ? ` · ${home.axes.filter(axis => axis.state === "homed").length}/${home.axes.length} homed` : "");
    heading.dataset.state = home.state; heading.title = home.detail;
    rows.replaceChildren();
    details.replaceChildren();
    const reasons = new Map<string, string[]>();
    for (const axis of home.axes) {
      const row = document.createElement("span"); row.dataset.homeState = axis.state;
      row.textContent = `${axis.id}: ${axis.label}`;
      row.title = axis.detail; rows.append(row);
      if (known && (axis.state !== "homed" || axis.detail.includes("unavailable"))) {
        reasons.set(axis.detail, [...(reasons.get(axis.detail) ?? []), axis.id]);
      }
    }
    for (const [reason, ids] of reasons) {
      const line = document.createElement("div"); line.textContent = `${ids.join(", ")}: ${reason}`; details.append(line);
    }
    if (this.#element('[data-step="home"]').dataset.state !== "blocked") {
      this.#element('[data-step-summary="home"]').textContent = home.state === "required"
        ? "Home establishes the joint position references. Start disarmed; the drives briefly energise and settling movement may occur." : home.detail;
    }
    if (home.state === "stale" || home.state === "unavailable" || home.state === "homing" || home.state === "recovering") {
      this.#element('[data-step="home"]').dataset.state = "pending";
      this.#element('[data-step-mark="home"]').textContent = "3";
      this.#element('[data-step-state="home"]').textContent = home.label;
      this.#element('[data-step-summary="home"]').textContent = home.detail;
    }
  }

  /** What a request does to the arm state, from the label it is performed under. */
  static #requestOf(label: string): MachineRequest {
    const upper = label.toUpperCase();
    return upper === "ARM" ? "arm" : upper === "DISARM" ? "disarm" : upper === "STOP" ? "stop" : "other";
  }

  /** The Jog panel's heading line about the joints: which machine, which posture. */
  #paintJogHost(view: MachineFlowView): void {
    const target = this.#jogHost?.querySelector<HTMLElement>("#jog-joints-target");
    if (!target) return;
    if (!view.connected) {
      target.textContent = "No machine connected. Joint jog needs a connected, armed machine; choose one under Cells.";
      target.dataset.posture = "none";
      return;
    }
    const posture = view.posture === "simulation" ? "simulation, no physical output" : view.posture === "real" ? "REAL, motion is physical" : "output unknown, treated as real";
    target.textContent = `Drives the selected machine, ${view.cellLabel} (${posture}).${view.arm.jogNote ? ` ${view.arm.jogNote}` : ""}`;
    target.dataset.posture = view.posture;
    target.dataset.armed = view.arm.state;
  }

  /** Age of contact and native-clock progress; repeated HTTP answers cannot
   *  keep an unchanged native observation fresh. Null before the first answer. */
  statusAgeMs(): number | null {
    if (!this.#lastContactMs) return null;
    const contact = Math.max(0, Date.now() - this.#lastContactMs);
    return this.#status?.rtCore && this.#lastNativeAdvanceMs
      ? Math.max(contact, Date.now() - this.#lastNativeAdvanceMs) : contact;
  }

  /** The posture banner across the panel: the one word that says whether a
   *  play moves metal. A described simulation is the only green. */
  #renderRuntime(): void {
    const tag = this.#robotHost?.querySelector<HTMLElement>("#dense-runtime");
    const state = this.#robotHost?.querySelector<HTMLElement>("#dense-target-state");
    if (!tag || !state) return;
    const daemon = this.#status?.daemon ?? null;
    const mode = daemon?.runtimeMode ?? null;
    tag.hidden = !this.#targetSelected;
    tag.dataset.mode = mode ?? "unknown";
    tag.textContent = `RTCore ${mode ? mode.toUpperCase() : "?"}`;
    if (!this.#targetSelected || !this.#capabilities) return;
    const caps = this.#capabilities;
    if (caps.backend === "rt_core") {
      const posture = this.executionPosture()!;
      const pill = executionPosturePill(posture);
      tag.textContent = pill.label;
      tag.title = pill.title;
      tag.dataset.mode = pill.className === "is-offline" ? "sim" : "live";
      if (!this.#cells) state.textContent = `${rtCoreTargetText(caps.rtCore)} · ${caps.daemonReachable ? "reachable" : "unavailable"}`;
      return;
    }
    const cell = daemon?.jetsonId && daemon?.robotModelId
      ? ` · ${daemon.robotModelId} on ${daemon.jetsonId}`
      : "";
    state.textContent = `Joint daemon · ${caps.cell} via ${caps.ingestAddr} · RTCore ${mode ? mode.toUpperCase() : "mode unknown"}` +
      `${mode === "sim" ? " (simulated servos, no motion output)" : mode === "live" ? " (LIVE DRIVES)" : ""}${cell}`;
  }

  #readoutText(): string {
    if (!this.#targetSelected) return "no target";
    const status = this.#status;
    if (!status) return "joint daemon · waiting for status";
    // The run row carries the same arm token as the pill, then the server's
    // reason or the execution state.
    if (status.backend === "rt_core") {
      const parts = ["rt-core", this.#view?.arm.token ?? "STATUS STALE", status.rtCore?.executionDiagnostic ? "Start position does not match plan" : status.daemonError || denseExecutionState(status)];
      const execution = status.rtCore?.execution;
      const cursor = status.rtCore?.cursor;
      if (execution?.identity?.trajectory_digest && cursor) {
        if (Number.isInteger(execution.segment) && execution.segment >= 0 && execution.segment < 0xffffffff) parts.push("segment " + (execution.segment + 1));
        parts.push("sample " + cursor.sample_index);
      }
      if (status.session?.stopReason) parts.push(status.session.stopReason);
      return parts.join(" · ");
    }
    if (status.daemonError) return `joint daemon unreachable · ${status.daemonError}`;
    const daemon = status.daemon;
    const cursor = status.cursor;
    const parts: string[] = [];
    const state = daemon?.executorState ?? "unknown";
    const homing = daemon?.transient === "go_home";
    parts.push(state === "aborted" && daemon?.abortReason
      ? `aborted · ${daemon.abortReason}`
      : homing && state === "playing" ? "going home" : homing && state === "done" ? "home" : state);
    if (cursor && cursor.totalSampleCount > 0) {
      parts.push(`sample ${cursor.globalSampleIndex}/${cursor.totalSampleCount}`);
      parts.push(`motion ${Math.max(0, cursor.segmentIndex) + 1}/${cursor.segmentCount}`);
      parts.push(`${cursor.globalTS.toFixed(1)}/${cursor.totalDurationS.toFixed(1)} s`);
    }
    if (daemon) parts.push(`torch ${daemon.torchOn ? "on" : "off"}`);
    const session = status.session;
    // The server's reason, when it adds something the daemon's does not
    // (a heartbeat loss reads as an operator stop on the daemon).
    if (session?.stopReason && session.stopReason !== daemon?.abortReason) {
      parts.push(`stopped: ${session.stopReason}`);
    }
    const silence = this.#lastContactMs ? Date.now() - this.#lastContactMs : 0;
    if (session && LIVE_STATES.has(state) && silence > DENSE_HEARTBEAT_MS * 1.5) {
      const deadline = session.heartbeatDeadlineMs ?? HEARTBEAT_DEADLINE_MS;
      parts.push(`CONNECTION LOST · server stops in ${Math.max(0, (deadline - silence) / 1000).toFixed(0)} s`);
    }
    return parts.join(" · ");
  }

  #readoutTitle(): string {
    const status = this.#status;
    if (!status?.daemon) return "";
    const daemon = status.daemon;
    const session = status.session;
    return [
      `servo feedback age ${daemon.feedbackAgeMs === null ? "none" : `${daemon.feedbackAgeMs} ms`}`,
      `pair authority ${daemon.authorityFresh ? "fresh" : "STALE"}`,
      daemon.runtimeMode ? `runtime mode ${daemon.runtimeMode}` : "",
      session ? `lease ${session.leaseHeld ? `held, ${session.leaseExpiresInMs ?? "?"} ms left` : "not held"}` : "",
      session?.heartbeatAgeMs !== null && session?.heartbeatAgeMs !== undefined
        ? `heartbeat ${session.heartbeatAgeMs} ms ago`
        : "",
      daemon.lastError ? `daemon: ${daemon.lastError}` : "",
    ].filter(Boolean).join("\n");
  }

  #renderMotors(): void {
    const host = this.#robotHost?.querySelector<HTMLElement>("#dense-motors");
    if (!host || host.hidden) return;
    if (this.#capabilities?.backend === "rt_core") { host.hidden = true; return; }
    // Only the cell's mapped axes: on the nine-motor bench the pair config maps six of
    // the nine wire slots, and the other three are not motors to watch.
    const mask = this.#status?.daemon?.mappedAxisMask ?? null;
    const axes = (this.#status?.axes ?? null)?.filter((axis) =>
      mask === null || mask === 0 || (mask & (1 << axis.index)) !== 0) ?? null;
    if (!axes || axes.length === 0) {
      host.innerHTML = `<p class="field-help">No per-motor feedback yet.</p>`;
      return;
    }
    const rows = axes.map((axis) => {
      // An unhomed axis has counts but no angle: RTCore's home is zero and the
      // conversion wraps to ±π, which read as a real position on the live
      // cell until the flag was shown.
      const unhomed = axis.homed === false;
      const fb = axis.qRad === null || unhomed ? "—" : (axis.qRad * RAD_TO_DEG).toFixed(2);
      const cmd = axis.qCmdRad === null ? "—" : (axis.qCmdRad * RAD_TO_DEG).toFixed(2);
      const delta = axis.qRad === null || axis.qCmdRad === null || unhomed
        ? "—"
        : ((axis.qCmdRad - axis.qRad) * RAD_TO_DEG).toFixed(3);
      const flags = [
        axis.operational ? "OP" : "off",
        unhomed ? "UNHOMED" : "",
        axis.fault ? "FAULT" : "",
        axis.error ? "ERR" : "",
      ].filter(Boolean).join(" ");
      const tone = axis.fault || axis.error ? "is-fault" : unhomed ? "is-unhomed" : axis.operational ? "is-operational" : "";
      const detail = `statusword 0x${axis.statusword.toString(16).padStart(4, "0")} · mode ${axis.modeDisplay}` +
        ` · ${axis.posCounts} counts (home ${axis.homeCounts})`;
      return `<tr class="${tone}" title="${escapeHTML(detail)}"><th>J${axis.index + 1}</th><td>${fb}</td>` +
        `<td>${cmd}</td><td>${delta}</td><td>${escapeHTML(flags)}</td></tr>`;
    });
    // Position, command and error in degrees; the raw statusword, mode and
    // counts sit in each row's tooltip so the table fits the sidebar.
    host.innerHTML = `
      <table class="dense-motor-table">
        <thead><tr><th>Axis</th><th>Pos °</th><th>Cmd °</th><th>Err °</th><th>State</th></tr></thead>
        <tbody>${rows.join("")}</tbody>
      </table>`;
  }

  #select(): HTMLSelectElement {
    return this.#robotHost!.querySelector<HTMLSelectElement>("#dense-target")!;
  }

  #element(selector: string): HTMLElement {
    return (this.#robotHost!.querySelector<HTMLElement>(selector)
      ?? this.#playbackHost!.querySelector<HTMLElement>(selector))!;
  }

  #button(selector: string): HTMLButtonElement {
    return (this.#playbackHost!.querySelector<HTMLButtonElement>(selector)
      ?? this.#robotHost!.querySelector<HTMLButtonElement>(selector))!;
  }

  // A failed catalogue read (an empty reply while the server restarts, a
  // proxy 5xx) is retried on its own with a bounded back-off, so a tab that
  // was open across a restart comes back by itself instead of staying on
  // "blocked" with no machine and no joint rows (seen 2026-09-16).
  // Mutable so a test can shorten the back-off; the panel reads it per attempt.
  static CATALOGUE_RETRY_MS: readonly number[] = [2000, 4000, 8000, 10000];
  #catalogueRetry: ReturnType<typeof setTimeout> | null = null;
  #catalogueAttempt = 0;

  #scheduleCatalogueRetry(): void {
    if (this.#catalogueRetry !== null) return;
    const delays = DensePanel.CATALOGUE_RETRY_MS;
    const delay = delays[Math.min(this.#catalogueAttempt, delays.length - 1)]!;
    this.#catalogueAttempt++;
    this.#catalogueRetry = setTimeout(() => {
      this.#catalogueRetry = null;
      void this.#discover();
    }, delay);
  }

  async #discover(): Promise<void> {
    const state = this.#robotHost!.querySelector<HTMLElement>("#dense-target-state")!;
    if (this.#deps.cells) {
      try {
        const list = await this.#deps.cells.list();
        this.#catalogueInvalid = false;
        this.#catalogueError = null;
        this.#catalogueAttempt = 0;
        if (list.targets.length) {
          this.#cells = list.targets;
          const select = this.#select();
          select.innerHTML = '<option value="">Choose a machine</option>';
          for (const cell of list.targets) {
            const option = document.createElement("option");
            option.value = cell.cell_id;
            option.textContent = `${cell.label} · ${cell.role}`;
            select.append(option);
          }
          this.#robotHost!.querySelector<HTMLElement>("#dense-cell-controls")!.hidden = false;
          state.textContent = "No machine connected";
          const connected = list.targets.find(cell => cell.cell_id === list.selected?.cell_id);
          if (connected && list.selected) {
            this.#connectedCell = connected.cell_id;
            this.#connectedPosture = connected;
            this.#deps.onTargetChange();
            select.value = connected.cell_id;
            state.textContent = cellTargetText(connected, list.selected.model_id);
            this.#capabilities = await this.#deps.client.capabilities();
            this.#setTarget(true);
          }
          this.#highlightCell();
          return;
        }
      } catch (error) {
        this.#rejectCatalogue(error);
        this.#catalogueError = error instanceof Error ? error.message : String(error);
        state.textContent = `Machine catalogue unavailable (${this.#catalogueError}); retrying`;
        this.render();
        this.#scheduleCatalogueRetry();
        return;
      }
    }
    try {
      this.#capabilities = await this.#deps.client.capabilities();
    } catch (error) {
      this.#capabilities = null;
      state.textContent = `Joint daemon · OLP server unreachable (${error instanceof Error ? error.message : String(error)})`;
      this.render();
      return;
    }
    const select = this.#select();
    for (const option of [...select.options]) if (option.value) option.remove();
    const caps = this.#capabilities;
    if (!caps.available) {
      state.textContent = `Joint daemon · not configured on the OLP server${caps.reason ? ` (${caps.reason})` : ""}`;
      this.render();
      return;
    }
    const option = document.createElement("option");
    option.value = "joint-daemon";
    option.textContent = `Joint daemon · ${caps.cell}${caps.daemonReachable ? "" : " (unreachable)"}`;
    select.append(option);
    if (caps.backend === "rt_core") option.textContent = rtCoreTargetText(caps.rtCore);
    state.textContent = `Joint daemon · ${caps.cell} via ${caps.ingestAddr} · ${caps.daemonReachable ? "reachable" : "NOT reachable"}` +
      ` · controller ${caps.controllerId}`;
    if (caps.backend === "rt_core") state.textContent = `${rtCoreTargetText(caps.rtCore)} · ${caps.daemonReachable ? "reachable" : "unavailable"}`;
    let remembered = "";
    try {
      remembered = sessionStorage.getItem(TARGET_STORAGE_KEY) ?? "";
    } catch {
      remembered = "";
    }
    if (remembered === "joint-daemon") {
      select.value = "joint-daemon";
      this.#setTarget(true);
    } else {
      this.render();
    }
  }

  #setTarget(selected: boolean): void {
    this.#clearFaultsMessage = "";
    this.#clearingFaults = false;
    this.#jointJog?.stop("cell_change");
    this.#cartesianJog?.end("cell_change");
    this.#targetSelected = selected;
    this.#armConfirm = false;
    try {
      if (!this.#cells) sessionStorage.setItem(TARGET_STORAGE_KEY, selected ? "joint-daemon" : "");
    } catch {
      // Per-tab convenience only.
    }
    if (selected) {
      this.#schedulePoll(0);
      this.#scheduleHeartbeat(DENSE_HEARTBEAT_MS);
    } else {
      this.#clearTimers();
      this.#status = null;
      this.#sessionId = null;
      this.#lastContactMs = 0;
      this.#nativeObservation = ""; this.#lastNativeTimeNs = 0; this.#lastNativeAdvanceMs = 0;
      this.#lastRequest = null;
      this.#setHighlight(null);
      this.#releaseCellModel();
    }
    this.render();
    this.#deps.onTargetChange();
  }

  /** No machine decides the robot any more: hand the picker every arm and
   * the program's own model. None (offline) once kept the last cell's model,
   * so after the mini bench every arm stayed disabled and the program could
   * not be moved to another robot until the page was reloaded. */
  #releaseCellModel(): void {
    if (this.#cellModel === null) return;
    this.#cellModel = null;
    this.#deps.onCellModels?.(armModels(), this.cellModel());
  }

  #highlightCell(): void {
    const cell = this.#cells?.find(candidate => candidate.cell_id === this.#select().value);
    this.#cellPending = !cell || cell.cell_id !== this.#connectedCell;
    this.#connectError = null;
    this.#robotHost!.querySelector<HTMLElement>("#dense-cell-mode")!.textContent = cell
      ? `Robot ${cell.models.join(", ")} · catalogue requests ${cell.requested_mode} · live backend ${cell.backend}` : "Choose a cell and its configured model";
    this.#robotHost!.querySelector<HTMLElement>("#dense-cell-connection")!.textContent = cell
      ? cell.reason || (cell.backend === "unreachable" ? "unreachable" : "reachable") : "";
    if (cell) {
      this.#cellModel = cell.models[0]!;
      this.#deps.onCellModels?.(cell.models, this.cellModel());
    } else this.#releaseCellModel();
    this.render();
  }

  async #connectCell(): Promise<void> {
    const cell = this.#cells?.find(candidate => candidate.cell_id === this.#select().value);
    if (!cell || cell.reason === "cell_configuration_missing" || !this.#deps.cells || this.#inFlight || unknownRobotModelReason(this.cellModel())) return;
    this.#inFlight = true;
    this.#connectError = null;
    const epoch = ++this.#actionEpoch;
    this.render();
    try {
      const result = await this.#deps.cells.select({ cell_id: cell.cell_id, model_id: cell.models[0]! });
      if (epoch !== this.#actionEpoch) return;
      this.#connectedCell = result.selected.cell_id;
      this.#selectionChanged = false;
      this.#connectedPosture = result.target;
      this.#deps.onTargetChange();
      this.#cellPending = false;
      this.#status = null;
      this.#sessionId = null;
      this.#lastContactMs = 0;
      this.#nativeObservation = ""; this.#lastNativeTimeNs = 0; this.#lastNativeAdvanceMs = 0;
      this.#lastRequest = null;
      this.#setHighlight(null);
      this.#robotHost!.querySelector<HTMLElement>("#dense-target-state")!.textContent = cellTargetText(result.target, result.selected.model_id);
      this.#robotHost!.querySelector<HTMLElement>("#dense-cell-connection")!.textContent = "connected · control and arming require explicit actions";
      this.#capabilities = await this.#deps.client.capabilities();
      this.#setTarget(true);
    } catch (error) {
      this.#rejectCatalogue(error);
      this.#connectError = error instanceof Error ? error.message : "cell_unreachable";
      this.#deps.status(this.#connectError, true);
      this.#robotHost!.querySelector<HTMLElement>("#dense-cell-connection")!.textContent = this.#connectError;
    } finally {
      if (epoch === this.#actionEpoch) this.#inFlight = false;
      this.render();
    }
  }

  #clearTimers(): void {
    if (this.#catalogueRetry !== null) clearTimeout(this.#catalogueRetry);
    this.#catalogueRetry = null;
    if (this.#pollTimer !== null) clearTimeout(this.#pollTimer);
    if (this.#heartbeatTimer !== null) clearTimeout(this.#heartbeatTimer);
    this.#pollTimer = this.#heartbeatTimer = null;
  }

  #schedulePoll(delay: number): void {
    if (this.#pollTimer !== null) clearTimeout(this.#pollTimer);
    this.#pollTimer = setTimeout(() => void this.#poll(), delay);
  }

  #scheduleHeartbeat(delay: number): void {
    if (this.#heartbeatTimer !== null) clearTimeout(this.#heartbeatTimer);
    this.#heartbeatTimer = setTimeout(() => void this.#heartbeat(), delay);
  }

  async #poll(): Promise<void> {
    if (!this.#targetSelected) return;
    const epoch = this.#actionEpoch;
    try {
      const status = await this.#deps.client.status();
      if (epoch === this.#actionEpoch) this.#apply(status);
      this.#deps.onJogEnd?.(status.lastJogEnd);
      this.#deps.onJogLive?.(status.jogLive);
      void this.#jointJog?.poll();
      void this.#cartesianJog?.poll();
    } catch (error) {
      if (epoch !== this.#actionEpoch) return;
      // The server did not answer: the machine is unobserved from here, so
      // the retained status keeps its identity but no servo fact. The last
      // contact time is left alone and ages into STATUS STALE.
      this.#status = this.#status
        ? { ...this.#status, rtCore: null, armed: null, daemonError: error instanceof Error ? error.message : String(error) }
        : null;
      this.render();
    } finally {
      if (this.#targetSelected) {
        this.#schedulePoll(this.#deps.wantsFastPoll() ? DENSE_STATUS_POLL_FAST_MS : DENSE_STATUS_POLL_MS);
      }
    }
  }

  async #heartbeat(): Promise<void> {
    if (!this.#targetSelected) return;
    try {
      const session = this.#status?.session;
      // A stopped session is over: the server answers its heartbeat with
      // 409 dense_session_mismatch, so the beat stops with the session.
      if (session && session.state !== "stopped") await this.#deps.client.heartbeat(session.id);
    } catch {
      // The readout shows the silence; the server enforces the deadline.
    } finally {
      if (this.#targetSelected) {
        // A poll that hangs never renders. The heartbeat's clock repaints the
        // fact once the last answer is past the freshness bound, so a stalled
        // status reads as STATUS STALE and not as the last word it said.
        const age = this.statusAgeMs();
        if (age !== null && age > STATUS_FRESHNESS_MS && this.#view?.arm.state !== "unknown") this.render();
        this.#scheduleHeartbeat(DENSE_HEARTBEAT_MS);
      }
    }
  }

  #apply(status: DenseStatus): void {
    // A delayed request must not replace a newer invalid Home/arm observation
    // with an earlier success from the same native clock domain.
    const observedTarget = status.target ?? this.#status?.target;
    const observationIdentity = `${observedTarget?.cell_id ?? ""}:${observedTarget?.selection_generation ?? ""}:${status.rtCore?.observationIdentity}`;
    if (status.rtCore && observationIdentity === this.#nativeObservation && status.rtCore.timeNs < this.#lastNativeTimeNs) return;
    const robotBefore = this.#status?.robotCell;
    queueMicrotask(() => {
      const robotAfter = this.#status?.robotCell;
      if (robotBefore?.robotDescriptionSha256 !== robotAfter?.robotDescriptionSha256 || robotBefore?.machinePlanningCalibrationSha256 !== robotAfter?.machinePlanningCalibrationSha256 ||
          robotBefore?.modelId !== robotAfter?.modelId || robotBefore?.valid !== robotAfter?.valid) {
        this.#deps.onStatus?.();
      }
    });
    const before = this.#status?.target?.selection_generation, after = status.target?.selection_generation;
    if (before && after && /^\d+$/.test(before) && /^\d+$/.test(after) && BigInt(after) < BigInt(before)) return;
    if (before && after && before !== after) {
      this.#selectionChanged = true; this.#armConfirm = false;
      this.#jointJog?.stop("selection_changed"); this.#cartesianJog?.stop("selection_changed");
    }
    if (this.#status?.target && !status.target && !status.available) {
      this.#connectedCell = "";
      this.#connectedPosture = null;
      this.#deps.client.observeTarget(undefined);
      this.#setTarget(false);
      return;
    }
    status = { ...status, target: status.target ?? this.#status?.target };
    this.#deps.client.observeTarget(status.target);
    if (this.#cells && status.target) {
      const previous = this.#connectedPosture;
      this.#connectedPosture = status.target;
      if (previous?.backend !== status.target.backend || previous?.simulation !== status.target.simulation || this.#connectedCell !== status.target.cell_id) {
        this.#deps.onTargetChange();
      }
    }
    if (this.#cells && status.target && (status.target.cell_id !== this.#connectedCell || (before && after && before !== after))) {
      const target = status.target;
      this.#connectedCell = target.cell_id;
      this.#select().value = target.cell_id;
      this.#cellPending = false;
      this.#armConfirm = false;
      this.#status = null;
      this.#lastRequest = null;
      const cell = this.#cells.find(candidate => candidate.cell_id === target.cell_id);
      if (cell) {
        this.#cellModel = target.model_id;
        this.#deps.onCellModels?.(cell.models, this.cellModel());
      }
      this.#robotHost!.querySelector<HTMLElement>("#dense-target-state")!.textContent = `${target.label} · ${target.model_id} · ${target.backend} · ${target.host}`;
      this.#capabilities = null;
      this.#testProgram = null;
      void this.#deps.client.capabilities().then(caps => {
        if (this.#connectedCell === target.cell_id) { this.#capabilities = caps; this.render(); }
      }).catch(() => undefined);
    }
    this.#status = status;
    this.#teachStill = nextTeachStill(this.#teachStill, status, observationIdentity, this.#teachBusy());
    if (status.rtCore) {
      const identity = `${status.target?.cell_id ?? ""}:${status.target?.selection_generation ?? ""}:${status.rtCore.observationIdentity}`;
      if (identity !== this.#nativeObservation || status.rtCore.timeNs > this.#lastNativeTimeNs) {
        this.#nativeObservation = identity;
        this.#lastNativeTimeNs = status.rtCore.timeNs;
        this.#lastNativeAdvanceMs = Date.now();
      }
    }
    this.#lastContactMs = Date.now();
    this.#recoverCapabilities(status);
    this.#deps.onTelemetry?.(status, this.#capabilities);
    this.#sessionId = status.session?.id ?? null;
    // Contact and native clock progress are checked together by livePoseView.
    // A successful HTTP response alone does not prove that native status advanced.
    if (status.target && !this.#cellPending && !this.#catalogueInvalid && robotModelClass(status.target.model_id) === "unknown" && status.target.model_id !== this.#cellModel) {
      this.#cellModel = status.target.model_id;
      this.#deps.onCellModels?.(this.#cells?.find(cell => cell.cell_id === status.target!.cell_id)?.models ?? [this.#cellModel], this.#cellModel);
    }
    const pose = robotModelClass(status.target?.model_id ?? this.cellModel()) !== "arm" ? null
      : mapLivePose(status, this.#capabilities, this.#lastPose, Date.now(), this.#deps.getProject().robot.model);
    if (pose) {
      this.#lastPose = pose;
      this.#deps.onRobotPose(pose);
    }
    const project = this.#deps.getProject(), record = project.denseTrajectory;
    const execution = status.rtCore?.execution;
    const state = denseExecutionState(status);
    const matches = record && status.session?.trajectoryDigest === record.trajectoryDigest &&
      status.session.planId === record.planId && record.sourceProjectRevision === project.identity.revision &&
      execution?.identity?.trajectory_digest === status.session.executionTrajectoryDigest;
    const segment = matches ? record.segments.find(segment => segment.index === execution?.segment) : null;
    this.#setHighlight(segment && (state === "playing" || state === "paused") && this.daemonReachable()
      ? denseSegmentNodeId(project.nodes, segment.kind, segment.sourceId) : null);
    this.render();
  }

  #setHighlight(nodeId: string | null): void {
    if (nodeId === this.#highlightedNode) return;
    this.#highlightedNode = nodeId;
    this.#deps.highlightNode(nodeId);
  }

  async #run(label: string, action: (current: () => boolean) => Promise<DenseStatus | null>): Promise<void> {
    if (this.#inFlight) return;
    await this.#perform(label, action);
  }

  /** Only the current action may publish a result or unlock controls, preventing
   * a superseded motion response from overwriting the public Stop result. */
  async #perform(label: string, action: (current: () => boolean) => Promise<DenseStatus | null>): Promise<void> {
    const epoch = ++this.#actionEpoch;
    const current = () => epoch === this.#actionEpoch;
    const request = DensePanel.#requestOf(label);
    this.#inFlight = true;
    this.#pendingRequest = request;
    this.#lastRequest = null;
    this.render();
    let failure: string | null = null;
    try {
      const status = await action(current);
      if (current() && status) this.#apply(status);
    } catch (error) {
      failure = error instanceof Error ? error.message : String(error);
      const message = processIOAdmissionMessage(failure) ? machineReasonText(failure) : failure;
      if (current()) this.#deps.status(`${label}: ${message}`, true);
    } finally {
      if (current()) {
        this.#inFlight = false;
        this.#pendingRequest = null;
        this.#lastRequest = { request, error: failure };
        // A new Arm result is not the one the operator already dismissed.
        if (request === "arm") this.#armRefusalAck = false;
        this.render();
        this.#schedulePoll(0);
      }
    }
  }

  /** The cell's outputs under Run: intent, command and wired readback per
   *  output (cellOutputs.ts), hidden for a cell with no io block. */
  #paintOutputs(outputs: CellOutputs | null): void {
    const view = cellOutputsView(outputs);
    const host = this.#element("#dense-outputs");
    host.hidden = !view.visible;
    host.dataset.tone = view.tone;
    const headline = document.createElement("strong");
    headline.textContent = view.headline;
    const rows = view.rows.map((row) => {
      const line = document.createElement("div");
      line.className = "dense-output-row";
      line.dataset.state = row.state;
      line.dataset.tone = row.tone;
      line.textContent = row.text;
      return line;
    });
    host.replaceChildren(headline, ...rows);
  }

  #runModeLocked(): boolean {
    return this.#inFlight || this.#cellPending || this.robotMoving();
  }

  async #load(): Promise<void> {
    const project = this.#deps.getProject();
    const record = project.denseTrajectory;
    if (!record || this.#button("#dense-load").disabled) return;
    const dryRun = this.#dryRun;
    const projectId = project.identity.id;
    const revision = project.identity.revision;
    this.#beginLoadProgress(record.bytes);
    let loaded = false;
    try {
      await this.#run("Load to robot", async (current) => {
        const identity = denseLoadIdentity(record, dryRun);
        // Only when the planner no longer holds the digest does the copy kept
        // at plan time go along (the server checks it is this plan). An OLP
        // server older than that fallback refuses a body that large.
        const result = await this.#deps.client.load(identity).catch(async (error: unknown) => {
          const kept = error instanceof DenseExecutionError && error.code === "dense_blob_not_found"
            ? await this.#deps.readTrajectory?.(record.trajectoryDigest).catch(() => null) ?? null : null;
          if (!kept) throw error;
          return this.#deps.client.load({ ...identity, dense_blob_base64: base64Of(kept instanceof Uint8Array ? kept : new Uint8Array(kept)) });
        });
        const latest = this.#deps.getProject();
        if (!current() || latest.identity.id !== projectId || latest.identity.revision !== revision ||
          latest.denseTrajectory?.trajectoryDigest !== record.trajectoryDigest || this.#dryRun !== dryRun) return null;
        loaded = true;
        this.#sessionId = result.sessionId;
        this.#deps.status([`Robot: trajectory ${record.trajectoryDigest.slice(7, 19)} staged on the joint daemon`, ...result.notes].join(" · "));
        return result.status;
      });
    } finally {
      this.#endLoadProgress(loaded);
    }
  }

  async #goToPlanStart(): Promise<void> {
    if (this.#button("#dense-plan-start").disabled) return;
    const project = this.#deps.getProject(), record = project.denseTrajectory!;
    const target = this.#status?.target;
    const revision = project.identity.revision;
    const actionEpoch = this.#actionEpoch + 1;
    this.#planStartPhase = "reading"; this.#planStartResult = "";
    try {
      await this.#run("Go to plan start", async current => {
        const bytes = await this.#deps.readTrajectory!(record.trajectoryDigest);
        const latest = this.#deps.getProject();
        if (!current()) return null;
        if (latest.identity.id !== project.identity.id || latest.identity.revision !== revision || latest.denseTrajectory?.trajectoryDigest !== record.trajectoryDigest ||
          target?.cell_id !== this.#status?.target?.cell_id || target?.selection_generation !== this.#status?.target?.selection_generation) throw new Error("Plan or machine changed; select the plan again");
        if (!bytes) throw new Error("Selected plan data is unavailable");
        const trajectory = decodeDenseTrajectory(bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes));
        const start = planStartOf({ status: this.#status, capabilities: this.#capabilities, trajectory, project, cellModel: this.cellModel() });
        if ("error" in start) throw new Error(start.error);
        this.#planStartPhase = "moving"; this.render();
        this.#deps.onRobotMotion();
        try {
          await this.#jointJog!.goToPositions(start.positions);
          if (current()) this.#planStartResult = "Move completed. Load to robot, then Play.";
        } catch (error) {
          if (current()) this.#planStartResult = error instanceof Error ? error.message : String(error);
          throw error;
        }
        return null;
      });
    } finally { if (this.#actionEpoch === actionEpoch) { this.#planStartPhase = null; this.render(); } }
  }

  #syncPlanStartPosition(): void {
    const project = this.#deps.getProject(), record = project.denseTrajectory;
    const robot = this.#status?.robotCell;
    const axes = this.#capabilities?.rtCore?.axes ?? [];
    const eligible = planStartEligible({ targetSelected: this.#targetSelected, cellPending: this.#cellPending, status: this.#status, capabilities: this.#capabilities, project });
    const key = eligible ? JSON.stringify([project.identity.id, project.identity.revision, record, this.#status?.target, robot,
      this.#status?.rtCore?.observationIdentity, this.#status?.rtCore?.coordinateIdentity, axes]) : "";
    if (key === this.#planStartKey) return;
    this.#planStartKey = key; this.#planStartPositions = null; this.#planStartResult = "";
    if (!key || !record || !this.#deps.readTrajectory) return;
    void this.#deps.readTrajectory(record.trajectoryDigest).then(bytes => {
      if (this.#planStartKey !== key || !bytes) return;
      const trajectory = decodeDenseTrajectory(bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes));
      const start = planStartOf({ status: this.#status, capabilities: this.#capabilities, trajectory, project: this.#deps.getProject(), cellModel: this.cellModel() });
      if ("error" in start) return;
      if (start.positions.every(Number.isFinite)) this.#planStartPositions = start.positions;
      this.render();
    }).catch(() => { /* Unavailable plan bytes never imply a confirmed position. */ });
  }

  #atPlanStart(): boolean {
    return !!this.#status?.rtCore?.coordinateIdentity &&
      atPlanStart(this.#planStartPositions, this.#status, this.#capabilities, this.statusAgeMs());
  }


  /** The upload's progress bar under the run row. The wire reports nothing
   *  about how far an upload is, so the bar is an estimate from the
   *  trajectory's bytes and the link's measured rate, said as such, and it
   *  only reaches the end on the cell's answer (owner, 2026-09-21: no way to
   *  tell whether Load was still going). */
  #loadProgress: { startedMs: number; bytes: number; timer: number } | null = null;
  #loadProgressHide: number | null = null;

  #beginLoadProgress(bytes: number): void {
    if (this.#loadProgressHide !== null) { window.clearTimeout(this.#loadProgressHide); this.#loadProgressHide = null; }
    if (this.#loadProgress) window.clearInterval(this.#loadProgress.timer);
    const host = this.#element("#dense-load-progress");
    host.hidden = false;
    host.classList.remove("is-done", "is-failed", "is-overdue");
    host.classList.add("is-running");
    this.#loadProgress = { startedMs: performance.now(), bytes, timer: window.setInterval(() => this.#paintLoadProgress(), 200) };
    this.#paintLoadProgress();
  }

  #paintLoadProgress(): void {
    const progress = this.#loadProgress;
    if (!progress) return;
    const elapsedS = (performance.now() - progress.startedMs) / 1000;
    const host = this.#element("#dense-load-progress");
    const bar = host.querySelector<HTMLElement>(".dense-load-progress-bar")!;
    const fill = host.querySelector<HTMLElement>(".dense-load-progress-fill")!;
    const fraction = loadProgressFraction(progress.bytes, elapsedS);
    fill.style.width = `${(fraction * 100).toFixed(1)}%`;
    bar.setAttribute("aria-valuenow", String(Math.round(fraction * 100)));
    host.classList.toggle("is-overdue", loadProgressOverdue(progress.bytes, elapsedS));
    host.querySelector<HTMLElement>(".dense-load-progress-text")!.textContent = loadProgressText(progress.bytes, elapsedS);
  }

  #endLoadProgress(loaded: boolean): void {
    const progress = this.#loadProgress;
    if (!progress) return;
    window.clearInterval(progress.timer);
    this.#loadProgress = null;
    const elapsedS = (performance.now() - progress.startedMs) / 1000;
    const host = this.#element("#dense-load-progress");
    host.classList.remove("is-running", "is-overdue");
    host.classList.add(loaded ? "is-done" : "is-failed");
    const fill = host.querySelector<HTMLElement>(".dense-load-progress-fill")!;
    fill.style.width = loaded ? "100%" : `${(loadProgressFraction(progress.bytes, elapsedS) * 100).toFixed(1)}%`;
    host.querySelector<HTMLElement>(".dense-load-progress-bar")!.setAttribute("aria-valuenow", loaded ? "100" : String(Math.round(loadProgressFraction(progress.bytes, elapsedS) * 100)));
    host.querySelector<HTMLElement>(".dense-load-progress-text")!.textContent = loaded ? loadDoneText(progress.bytes, elapsedS) : loadFailedText(progress.bytes, elapsedS);
    // The outcome stays readable for a while, then the row goes back to the readout alone.
    this.#loadProgressHide = window.setTimeout(() => { host.hidden = true; this.#loadProgressHide = null; }, loaded ? 8_000 : 15_000);
  }

  /** Run program, or Run again after the program completed (`runProgram.ts`):
   *  one press the OLP server sequences -- Go to plan start when the arm is
   *  elsewhere, Load unless staged, Arm when disarmed, Play -- asking once
   *  before it arms or moves (the All joints to 0° dialog). STOP ends it at
   *  any point; a question answered after STOP sends nothing. */
  /** The repeat chosen for the next Plan and Run (rails plans the return
   *  move when it is not "once"). */
  repeatMode(): RepeatMode { return this.repeat().mode; }
  repeat() { return repeatOf(this.#repeatMode, this.#repeatCount); }

  /** Finish cycle: the running loop ends at the program start after the
   *  current cycle. The server admits only the client that started it. */
  async finishLoop(): Promise<void> {
    try {
      const loop = await this.#deps.client.finishLoop();
      this.#runNote = loop ? loopText(loop) : this.#runNote;
      this.#deps.onPressNotice?.(null);
    } catch (error) {
      this.#runNote = error instanceof DenseExecutionError ? error.detail || error.message : String(error);
      this.#deps.status(`Finish cycle: ${this.#runNote}`, true);
      this.#deps.onPressNotice?.(finishRefusal(error instanceof DenseExecutionError ? error.code : "", this.#runNote));
    }
    this.render();
  }

  async runProgram(): Promise<void> {
    const project = this.#deps.getProject(), record = project.denseTrajectory;
    if (!record || this.#runPress || this.#button("#dense-run-program").disabled) return;
    const load: Record<string, unknown> = { ...denseLoadIdentity(record, this.#dryRun) };
    const fraction = this.#jointJog?.speedFraction ?? 0.1;
    const repeat = this.repeat();
    if (this.#inFlight) {
      // #run would drop the press without a word.
      this.#deps.onPressNotice?.(runNotSent(`${!this.#pendingRequest || this.#pendingRequest === "other" ? "An earlier request" : this.#pendingRequest.toUpperCase()} is still in progress; press again when it has finished`, repeat));
      return;
    }
    this.#deps.onPressNotice?.(null);
    const press = new AbortController();
    this.#runPress = press;
    this.#runNote = "";
    let kept: string | null = null;
    // Only when the planner no longer holds the digest does the copy kept at
    // plan time go along, as Load sends it.
    const send = async (confirmed?: RunResult) => {
      const body = () => runProgramRequest(kept ? { ...load, dense_blob_base64: kept } : load, fraction, confirmed, repeat);
      let answer = await this.#deps.client.runProgram(body(), press.signal);
      if (answer.result.state === "refused" && answer.result.code === "dense_blob_not_found" && !kept) {
        const bytes = await this.#deps.readTrajectory?.(record.trajectoryDigest).catch(() => null) ?? null;
        if (bytes) {
          kept = base64Of(bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes));
          answer = await this.#deps.client.runProgram(body(), press.signal);
        }
      }
      return answer;
    };
    try {
      await this.#run("Run program", async current => {
        const live = () => current() && this.#runPress === press;
        this.#deps.onRobotMotion();
        // A refusal without a `run` body (a guard's 409, a lost server) throws.
        const refused = (error: unknown): never => {
          if (live()) {
            const code = error instanceof DenseExecutionError ? error.code : "run_failed";
            const detail = error instanceof DenseExecutionError ? error.detail || error.message : error instanceof Error ? error.message : String(error);
            this.#deps.onPressNotice?.(runRefusal({ state: "refused", action: "", needsArm: false, needsMove: false, steps: [], joints: [],
              refusal: { step: "", code, detail }, sessionId: "", notes: [], loop: null, code }, repeat));
          }
          throw error;
        };
        let { result, status } = await send().catch(refused);
        if (result.state === "confirm") {
          if (status && live()) this.#apply(status);
          this.#runNote = runOutcomeText(result);
          this.render();
          const yes = this.#deps.confirmRecover ? await this.#deps.confirmRecover(runConfirmation(result)) : false;
          if (!live()) return null;
          if (!yes) {
            this.#runNote = RUN_CANCELLED;
            return null;
          }
          ({ result, status } = await send(result).catch(refused));
        }
        if (!live()) return null;
        if (result.sessionId) this.#sessionId = result.sessionId;
        this.#runNote = runOutcomeText(result);
        if (result.state === "refused") {
          this.#deps.onPressNotice?.(runRefusal(result, repeat));
          throw new DenseExecutionError(result.code || result.refusal?.code || "run_refused", this.#runNote, 409);
        }
        this.#deps.status(`Robot: ${this.#runNote}`);
        return status;
      });
    } finally {
      if (this.#runPress === press) this.#runPress = null;
      this.render();
    }
  }

  async #play(): Promise<void> {
    this.#syncPlanStartPosition();
    const session = this.#sessionId;
    const record = this.#deps.getProject().denseTrajectory;
    if (!session || !record || record.sourceProjectRevision !== this.#deps.getProject().identity.revision || this.#button("#dense-play").disabled ||
      (this.#capabilities?.backend === "rt_core" && !this.daemonReachable()) ||
      (this.#capabilities?.backend === "rt_core" && !this.#atPlanStart()) ||
      !denseProgramPlayable(this.#status, record.trajectoryDigest, this.#dryRun)) return;
    this.#deps.onRobotMotion();
    await this.#run("Play", () => this.#deps.client.play(session));
  }

  #renderTestProgram(native: boolean): void {
    const program = this.#testProgram;
    const controls = deriveTestProgramControls({
      enabled: this.#capabilities ? this.#capabilities.testPrograms?.enabled === true : null, targetSelected: this.#targetSelected, cellPending: this.#cellPending, native,
      inFlight: this.#inFlight, daemonReachable: this.daemonReachable(), status: this.#status, program,
      atStart: program !== null && atPlanStart(program.startRad, this.#status, this.#capabilities, this.statusAgeMs()),
    });
    this.#element("#dense-test-program").hidden = !controls.visible;
    const load = this.#button("#dense-test-load"), play = this.#button("#dense-test-play");
    load.disabled = controls.load.disabled; load.title = controls.load.title;
    play.disabled = controls.play.disabled; play.title = controls.play.title;
    this.#element("#dense-test-label").textContent = controls.label;
  }

  /** Upload the chosen .rdt and plan.json for the server's cell checks, then
   *  the ordinary Load naming it as a test program. Moves nothing. */
  async loadTestProgram(files: readonly File[]): Promise<void> {
    if (this.#button("#dense-test-load").disabled) return;
    const picked = pickTestProgramFiles(files, this.#capabilities?.testPrograms?.maxBytes ?? 0);
    if ("error" in picked) { this.#deps.status(`Load test program: ${picked.error}`, true); return; }
    await this.#run("Load test program", async (current) => {
      const bytes = new Uint8Array(await picked.rdt.arrayBuffer());
      let planJSON: unknown = null;
      if (picked.plan) {
        try { planJSON = JSON.parse(await picked.plan.text()); } catch { throw new Error(`${picked.plan.name} is not JSON`); }
      }
      const uploaded = await this.#deps.client.uploadTestProgram({ fileName: picked.rdt.name, bytes, planJSON, planFileName: picked.plan?.name ?? null });
      if (!current()) return null;
      this.#testProgram = uploaded.program;
      const result = await this.#deps.client.load(testProgramLoadIdentity(uploaded.program));
      if (!current()) return null;
      this.#sessionId = result.sessionId;
      this.#deps.status([`Robot: loaded ${testProgramLabel(uploaded.program)}`, ...uploaded.checks, ...result.notes].join(" · "));
      return result.status;
    });
  }

  async #playTestProgram(): Promise<void> {
    const session = this.#status?.session;
    if (this.#button("#dense-test-play").disabled || !session || !testProgramSession(this.#status, this.#testProgram)) return;
    this.#deps.onRobotMotion();
    await this.#run("Play test program", () => this.#deps.client.play(session.id));
  }

  async #pause(): Promise<void> {
    const session = this.#sessionId;
    if (!session) return;
    await this.#run("Pause", () => this.#deps.client.pause(session));
  }

  async #stop(label: string): Promise<void> {
    this.#clearingFaults = false;
    this.#clearFaultsMessage = "";
    this.#planStartPhase = null;
    if (this.#runPress) {
      // The server fences the rest of the press by STOP's epoch; aborting
      // here also keeps an open question from sending its confirmation.
      this.#runPress.abort();
      this.#runPress = null;
      this.#runNote = `Run program ended by ${label === "DISARM" ? "DISARM" : "STOP"}`;
    }
    this.#jointJog?.stop(label === "DISARM" ? "operator_disarm" : "operator_stop");
    this.#cartesianJog?.stop(label === "DISARM" ? "disarm" : "stop");
    this.#armConfirm = false;
    const stop = async (current: () => boolean) => {
      if (label === "DISARM" && this.#capabilities?.backend === "rt_core") {
        const result = await this.#deps.client.arm(false);
        if (!current()) return null;
        this.#deps.status(result.accepted ? "Robot: disarm accepted by RTCore" : "Robot: DISARM NOT ACKNOWLEDGED; use the hardware stop", !result.accepted);
        return result.status;
      }
      const result = await this.#deps.client.stop();
      if (!current()) return null;
      this.#deps.status(
        result.stopped
          ? `Robot: stopped (acknowledged after ${result.attempts} attempt${result.attempts === 1 ? "" : "s"}), lease released`
          : `Robot: STOP NOT ACKNOWLEDGED after ${result.attempts} attempts — use the hardware stop`,
        !result.stopped,
      );
      return result.status;
    };
    if (this.#capabilities?.backend === "rt_core") {
      await this.#perform(label, stop);
    } else {
      await this.#run(label, stop);
    }
  }

  /** Why the native Home cannot run now, or null: the same gate as the
   *  Deploy pane's HOME, in words the Joints section can show. */
  #homeDisabledReason(): string | null {
    if (!this.#targetSelected || this.#cellPending) return "Connect a machine first";
    if (this.#inFlight) return "Another request is in flight";
    if (this.#status?.session && LIVE_STATES.has(this.#status.session.state)) return "The program is running";
    if (this.#capabilities?.backend === "rt_core") {
      if (!this.#capabilities.rtCore?.nativeHome) return "This machine has no native Home";
      const recoveryRefusal = nativeHomeRecoveryRefusal(this.#status);
      if (recoveryRefusal) return recoveryRefusal;
      if (this.#view && this.#view.arm.state !== "disarmed") return "Disarm first: Home runs with the drives off";
      // Unlike the Deploy pane's HOME step, which retires once Home is valid, the
      // Jog view re-homes a joint that already has a Home: that is its point.
      return null;
    }
    return this.#view?.arm.state === "armed" ? null : "Arm first: the legacy Go Home is a motion";
  }

  async #home(axes?: readonly number[]): Promise<void> {
    const refusal = this.#homeDisabledReason();
    if (refusal) { this.#deps.status(refusal, true); return; }
    this.#pendingHomeAxes = axes ?? this.#capabilities?.rtCore?.axes.map(axis => axis.index) ?? [];
    this.#deps.onRobotMotion();
    try { await this.#run("GO HOME", async (current) => {
      const status = this.#capabilities?.backend === "rt_core"
        ? await this.#deps.client.home(axes)
        : await this.#deps.client.goHome();
      if (!current()) return null;
      const which = axes?.length ? axes.map(index => this.#capabilities?.rtCore?.axes[index]?.id ?? `axis ${index}`).join(", ") : "every axis";
      this.#deps.status(this.#capabilities?.backend === "rt_core" ? `Robot: Home request completed for ${which}; check the reported Home status` : "Robot: going home under the lease");
      return status;
    }); } finally { this.#pendingHomeAxes = null; this.render(); }
  }

  /** Arm on a REAL or unknown posture is two clicks: the first shows what
   *  will happen and the second sends it. A described simulation arms on
   *  the first. Disarm is never gated. */
  async #arm(armed: boolean): Promise<void> {
    if (!armed && this.#capabilities?.backend === "rt_core") { await this.#stop("DISARM"); return; }
    if (armed) {
      this.render();
      if (!this.#view?.canArm || unknownRobotModelReason(this.cellModel(), this.#status?.target?.model_id)) return;
    }
    if (armed && this.#view?.armConfirmRequired && !this.#armConfirm) {
      this.#armConfirm = true;
      this.render();
      return;
    }
    this.#armConfirm = false;
    await this.#run(armed ? "ARM" : "DISARM", async (current) => {
      const result = await this.#deps.client.arm(armed);
      if (!current()) return null;
      if (!result.accepted) throw new Error(result.error || "Arm refused");
      this.#deps.status(
        result.accepted
          ? `Robot: ${armed ? "arm" : "disarm"} accepted by RTCore`
          : `Robot: ${armed ? "arm" : "disarm"} refused${result.error ? ` — ${result.error}` : ""}`,
        !result.accepted,
      );
      return result.status;
    });
  }

  /** Discard invalid catalogue claims so a parser failure cannot retain an
   * arm presentation or enable motion on an unvalidated target. */
  #rejectCatalogue(error: unknown): void {
    if (!(error instanceof CellCatalogueError)) return;
    this.#catalogueInvalid = true;
    this.#armConfirm = false;
    this.#deps.onCellModels?.(["unknown"], "unknown");
  }
}
