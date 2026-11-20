import { bindProgramNameInput } from "./ui/programName.ts";
import { isMachineFault } from "./execution/machineFlow.ts";
import { cartesianStepLabels, readCartesianSteps, writeCartesianSteps } from "./execution/cartesianMove.ts";
import type { RobotPoseSnapshot, SimulationSample } from "./simulation/contracts.ts";
import { createOperatorFence, previewFeedbackLine } from "./targets/operatorFeedback.ts";
import { armModels, CellTargetsClient, renderCellModel, robotModelClass } from "./targets/cells.ts";
import { DiscoveryModel, type DiscoveredRow } from "./targets/discoveredCells.ts";
import { JOINT_JOG_FRACTIONS, ZERO_BUTTON_TITLE } from "./execution/jointJog.ts";
import { TelemetryStrip, sampleFromStatus, samplesFromWindow } from "./execution/telemetryStrip.ts";
import { executionPathSamples, executionTcpErrors } from "./execution/executedPath.ts";
import { ExecutionPathPanel, type ExecutionCapture } from "./execution/executionPathPanel.ts";
import { robotModelFor } from "./viewer/models.ts";
import { isBusyText, markBusy, setBusyText } from "./ui/busy.ts";
import { cartesianPadPresentation } from "./execution/cartesianJog.ts";
import "./style.css";

import {
  appendCadQueryTopology,
  deliberatelyRowlessSeamIDs,
  ensureCadPathRows,
  rowsFromSeams,
} from "./adapters/cadquery.ts";
import {
  computeProgramV2Digest,
  programV2Problems,
  type ProgramV2Node,
  type WeldNode,
} from "./contracts/robot-v4-program-v2.ts";
import { toProgramV2Document } from "./program-v2/from-project.ts";
import {
  dwellNode,
  effectiveNodes,
  freespaceNode,
  insertWeld,
  ioNode,
  moveBlocked,
  nodeById,
  removeNode,
} from "./program-v2/nodes.ts";
import { applyEdit, type FieldSpec, type EditValue } from "./program-v2/field-specs.ts";
import {
  commitProgramNode,
  deleteBlocked,
  deleteProgramNode,
  deriveRows,
  insertEventNode,
  moveProgramNode,
  recordMove,
  setNodeEnabled,
} from "./program-v2/edits.ts";
import { createNodeEditor, type SeamEdits } from "./program-v2/node-editor.ts";
import { CaptureProgramMismatch, captureDescription, checkCaptureCalibration, checkCaptureModel, machineBoundCapture, teachCapture } from "./program-v2/capture.ts";
import { decodeDenseTrajectory } from "./execution/denseTrajectoryBlob.ts";
import { jointMotionProfile, tcpMotionProfile } from "./domain/motion-profile.ts";
import { rowNodeId, rowsOfNode, stampRowNodeIds } from "./program-v2/rows-from-nodes.ts";
import { applyNodeHighlight, renderNodeSpreadsheet } from "./program-v2/spreadsheet.ts";
import { DenseExecutionClient } from "./execution/denseClient.ts";
import { DensePanel } from "./execution/densePanel.ts";
import { confirmRecoverDialog } from "./execution/recoverDialog.ts";
import { createJogFeedGate } from "./viewer/jog-feed.ts";
import { nodeSummary } from "./program-v2/summaries.ts";
import {
  createWeldFromEdges,
  deleteWeld,
  projectWelds,
} from "./adapters/weld-authoring.ts";
import {
  draftWithEdge,
  draftWithoutEdge,
  renderWeldDrawer,
  updateWeldTrimReadout,
  type WeldDraftUIState,
} from "./authoring/weld-drawer.ts";
import {
  acceptedPlanTrajectory,
  fromRobotV4Program,
  toRobotV4Program,
} from "./adapters/robot-v4.ts";
import {
  DOG_BOX_FIXTURE,
  HttpCadClient,
  type CadSTEPSource,
  type CadWorkpieceMesh,
} from "./cad/index.ts";
import { ProjectHistory } from "./domain/history.ts";
import {
  canStepPlayback,
  rowIdForTrajectorySample,
  sampleAtTime,
  stepPlaybackTime,
  trajectoryMotionSummary,
  type PlaybackStepDirection,
} from "./domain/playback.ts";
import {
  alignWorkpieceToCellWorkFrame,
  centreWorkpieceOnWorkFrame,
  hasWorkpieceGeometry,
  needsCellWorkFrameRehang,
  needsWorkFrameMigration,
  placementFromWorldAnchorPose,
  setWorkpiecePlacement,
  workpiecePlacement,
  type WorkpiecePlacement,
  worldPointToPlanner,
} from "./domain/placement.ts";
import {
  finalizeRobotV4Program,
  type RobotV4ProgramCatalog,
} from "./contracts/robot-v4-program.ts";
import {
  migrateLegacySeamTrims,
  needsPhysicalSeamTrimMigration,
  seamTrimLimits,
} from "./domain/seam.ts";
import {
  cellSettingsKey,
  cellWorkFrame,
  needsCellPinReconcile,
  reconcileCellPin,
  adoptRobotDescription,
  pinRobotDescription,
  robotDescriptionDrift,
  type CellSettings,
  type RobotModel,
  type AuthoredProgramRow,
  type JointVector,
  type OfflineProgrammingProject,
  type PlanEvidence,
  type PlanTrajectorySample,
  type ProjectChange,
  type RowType,
  type Vec3,
  DEFAULT_TRAVEL_SPEED_MPS,
  installServerValidatedPlan,
  reviseProject,
} from "./domain/project.ts";
import { createBlankSpreadsheetProject } from "./domain/spreadsheet.ts";
import { connectedRobot, robotCatalogue, setConnectedRobot, setRobotCatalogue } from "./domain/robots.ts";
import { applyRobotModel, cellSettingsPull, isRobotModel } from "./domain/robot-model.ts";
import { robotSettingsReview } from "./domain/robot-settings-review.ts";
import { fetchRobotCatalogue } from "./targets/robotsClient.ts";
import { exportProjectJSON, importProjectJSON } from "./domain/serialization.ts";
import {
  WEAVE_PATTERNS,
  normalizeWeavePattern,
  processTorchDefaults,
  rowTorchSettings,
  rowWeaveSettings,
} from "./domain/weld.ts";
import { lintProgram } from "./domain/weld-quality.ts";
import {
  buildProgramView,
  canReorderControlPoint,
  type ProgramControlPoint,
  type ProgramMove,
} from "./domain/points-moves.ts";
import {
  migrateDogBoxPlannerDefaults,
  needsDogBoxPlannerDefaultsMigration,
} from "./examples/dog-box.ts";
import {
  unavailableGatewayCapabilities,
  type GatewayCapabilityReport,
  type GatewayExecutionIdentity,
  type GatewayReviewResult,
} from "./execution/gateway.ts";
import {
  ExecutionGatewayClient,
  type GatewayExecutionSession,
} from "./execution/gatewayClient.ts";
import { TELEOP_CAPABILITY } from "./execution/teleop.ts";
import { installResizableLayout } from "./layout/resizable.ts";
import { installViewportShellSizing } from "./layout/viewport-shell.ts";
import {
  ProgramCatalogClient,
  ProgramCatalogConflictError,
} from "./programs/catalog-client.ts";
import {
  LocalPlannerAdapter,
  LocalSimulationAdapter,
} from "./simulation/local-adapter.ts";
import {
  LocalPlanRejectedError,
} from "./simulation/local-client.ts";
import {
  CARTESIAN_JOG_DISABLED_TITLE,
  JOG_HOLD_MS,
  cartesianJogView,
  jogButtonAxes,
  jogSessionIsLive,
  liveJogUrl,
  nextJogRetryDelayMs,
  parseJogResponse,
  readJogPadVisible,
  shouldKeepTryingJogStart,
  writeJogPadVisible,
  type JogResponse,
  type LiveJogPath,
} from "./simulation/live-jog.ts";
import {
  createViewerJointMoveProject,
  homeJointTarget,
  jointsMatch,
  validateViewerJointMoveTrajectory,
  type ViewerJointMovePurpose,
} from "./simulation/viewer-joint-move.ts";
import {
  localSafetyReasons,
  localSimulatorReady,
  type LocalPlanResult,
  type LocalSimulatorStatus,
} from "./simulation/local-contracts.ts";
import { AutosaveController } from "./storage/autosave.ts";
import { BrowserCellSync } from "./storage/cell-sync-browser.ts";
import { programSwitchLock } from "./storage/cell-sync.ts";
import { OpenProgramDialog } from "./storage/open-program-dialog.ts";
import { IndexedDbProjectRepository } from "./storage/repository.ts";
import {
  ConnectedTargetsClient,
  type ConnectedReadonlyCapabilities,
  type ConnectedReadonlyStatus,
} from "./targets/connectedClient.ts";
import { TargetPlanningClient } from "./targets/planningClient.ts";
import type {
  ConnectedTargetDiscoveryDescriptor,
} from "./targets/connectedDiscovery.ts";
import {
  TARGET_DESCRIPTOR_SCHEMA,
  type TargetDescriptor,
} from "./targets/contracts.ts";
import {
  initialTargetState,
  reduceTargetState,
  type TargetState,
} from "./targets/stateMachine.ts";
import { EdgeSelectionController } from "./viewer/edge-picking.ts";
import { createRailsMode, type RailsSnapshot } from "./rails/index.ts";
import { StandCard } from "./authoring/standCard.ts";
import {
  defaultWorkcellFixtures,
  editWorkcellFixture,
  fixtureById,
  fixturePoseLabel,
  fixtureShapeLabel,
  newBoxFixture,
  newStepFixture,
  workcellFixtures,
  workcellFixturesWire,
  type WorkcellFixture,
  type WorkcellFixtureField,
} from "./domain/workcell.ts";
import { activeWorkpieceId, activateWorkpiece, environmentFixtures, forgetActiveWorkpiece, projectWorkpieces } from "./domain/workpieces.ts";
import { createRailsSimulation, type RailsLoadOutcome } from "./execution/railsSimulation.ts";
import { PLAYBACK_RATES, type PlaybackRate } from "./execution/denseSimulation.ts";
import { repositoryPresetStore } from "./parameters/repositoryStore.ts";
import type { FreeSpaceBackend } from "./rails/client.ts";
import { denseTrajectoryRecordOf } from "./rails/planRequest.ts";
import { WorkstationRenderer } from "./viewer/workstation-renderer.ts";
import { Action, ActionGroup } from "./ui/action.ts";
import { ToolBar, ToolButton } from "./ui/toolbar.ts";
import { Menu, MenuButton, armDismiss } from "./ui/menu.ts";
import { applyResolved, setThemePreference, themeChanged, themePreference, type ThemePreference } from "./ui/theme.ts";
import { PlacementCard, type PlacementField, type PlacementValues } from "./authoring/placementCard.ts";
import { RailsWeldsCard } from "./authoring/railsWeldsCard.ts";
import { WaypointWeldsCard } from "./authoring/waypointWeldsCard.ts";
import { WaypointSession, takeStoredWaypoints } from "./waypoints/session.ts";
import { WaypointOverlay } from "./waypoints/overlay.ts";
import { WeldHighlight } from "./viewer/weld-highlight.ts";
import { weldPathPoints } from "./program-v2/weld-path.ts";
import { DEFAULT_WAYPOINT_STANDOFF_MM } from "./waypoints/model.ts";
import { isManualPoseWeld, waypointWeldOf, type WeldTool } from "./waypoints/node.ts";
import { RobotDialog } from "./authoring/robotDialog.ts";
import { activatePlan, activePlanEntryId, deletePlan, recordPlan } from "./domain/planHistory.ts";
import { WeldPresetsCard } from "./authoring/weldPresetsCard.ts";
import { PlanPresetsCard } from "./authoring/planPresetsCard.ts";
import { Inspector, summaryRow } from "./authoring/inspector.ts";
import { TREE_FIXTURE_PREFIX, TREE_PART, TREE_PROGRAM, TREE_WORKSPACE, TREE_WORKPIECE_PREFIX, TREE_WORKPIECE_WELD_PREFIX, programTreeNodes } from "./authoring/programTree.ts";
import { TreeView } from "./ui/tree.ts";
import { appLog, levelOfStatus } from "./ui/log.ts";
import { LogIndicator, LogPanel } from "./ui/log-panel.ts";
import { ProgressDialog } from "./ui/progress-dialog.ts";
import { MaintenanceOverlay } from "./ui/maintenance-overlay.ts";
import { MaintenanceWatch } from "./execution/maintenanceWatch.ts";
import { ControlPopup } from "./ui/control-popup.ts";
import { CONTROL_TAKE_API, ControlTracker } from "./execution/controlNotice.ts";
import { JogEndToast } from "./ui/jog-end-toast.ts";
import { JogEndTracker } from "./execution/jogEnd.ts";
import { PressNoticeTracker } from "./execution/pressNotice.ts";
import type { JogEndCause } from "./execution/jogEndCause.ts";
import { jogLiveView, normalizeJogLive } from "./execution/jogLive.ts";
import { JogLiveReadout } from "./ui/jog-live.ts";
import { installControlFetch, tabClientId } from "./execution/controlFetch.ts";
import { planningFailure } from "./ui/planning-failure.ts";
import { planMotionInstruction, PLANNED_FROM_PROGRAM_START } from "./execution/runControls.ts";
import { SideDock } from "./ui/dock.ts";
import { element } from "./ui/widget.ts";
import { Shortcuts, isTypingTarget } from "./ui/shortcuts.ts";
import { Activity, Axis3d, Camera, Gamepad2, House, Menu as MenuIcon, ListChecks, Move3d, Redo2, RotateCcw, Settings, Slash, SlidersHorizontal, Spline, Table2, Tags, Trash2, Undo2, Upload, Video, Waypoints } from "lucide";
import { workpieceMeshSummary } from "./viewer/workpiece.ts";
import { createPlannedPathController } from "./viewer/planned-path-controller.ts";
import { densePlannedPathRuns, projectPlannedPath } from "./viewer/planned-path.ts";
import {
  applyDenseSample,
  applyProjectSample,
  projectCadEdgeOverlays,
  projectFramePoints,
  syncAuthoringOverlays,
  syncProjectToWorkstation,
} from "./viewer/workstation.ts";

declare global {
  interface Window {
    __olpBootProgress?: () => void;
    __olpBootDone?: () => void;
  }
}

// Before any API client is built (they bind fetch): every request names this
// tab to the server, and a refusal because another controller is using the
// robot raises the control pop-up (execution/controlFetch.ts).
const controlTracker = new ControlTracker();
let controlPopup: ControlPopup | null = null;
function renderControl(): void {
  controlPopup?.render(controlTracker.notice(Date.now()));
}
const clientId = tabClientId(sessionStorageOrNull());
installControlFetch(window, clientId, (body) => {
  if (controlTracker.refused(body, Date.now())) renderControl();
});
// "Jog stopped" (execution/jogEnd.ts): fed by every dense status poll; this
// tab's own jogs get the toast, other clients' a quiet note. A timer redraws
// it while shown so it goes by itself.
const jogEndTracker = new JogEndTracker(clientId);
let jogEndToast: JogEndToast | null = null;
let jogLiveReadout: JogLiveReadout | null = null;
let jogEndTimer: ReturnType<typeof setTimeout> | null = null;
function renderJogEnd(): void {
  const notice = jogEndTracker.notice(Date.now());
  jogEndToast?.render(notice);
  if (jogEndTimer !== null) clearTimeout(jogEndTimer);
  jogEndTimer = notice.active ? setTimeout(renderJogEnd, Math.min(Math.max(notice.remainingMs, 50), 1000)) : null;
}
// "Run program refused" (execution/pressNotice.ts): a Run or Finish cycle
// press that started nothing, drawn by the Jog stopped toast one row lower.
const pressNoticeTracker = new PressNoticeTracker();
let pressNoticeToast: JogEndToast | null = null;
let pressNoticeTimer: ReturnType<typeof setTimeout> | null = null;
function renderPressNotice(): void {
  const notice = pressNoticeTracker.notice(Date.now());
  pressNoticeToast?.render(notice);
  if (pressNoticeTimer !== null) clearTimeout(pressNoticeTimer);
  pressNoticeTimer = notice.active ? setTimeout(renderPressNotice, Math.min(Math.max(notice.remainingMs, 50), 1000)) : null;
}

function sessionStorageOrNull(): Storage | null {
  try {
    return window.sessionStorage;
  } catch {
    return null;
  }
}

// Re-arms the boot watchdog in `public/boot-recovery.js`. Boot is judged by
// progress rather than total elapsed time, so a slow mesh load on a cold cache
// never trips the recovery reload.
function bootProgress(): void {
  window.__olpBootProgress?.();
}

/**
 * Live Motion Server jog state. Cartesian BASE/TOOL hold buttons in the
 * spreadsheet header enable only when a live jog session is running
 * (`POST .../local-simulator/jog/start` → isolated robot-v4-cartesiand) AND
 * the user has explicitly armed the simulation drives via the header pill.
 * Tesseract planning is a separate path.
 *
 * These `let` declarations MUST stay ahead of ALL top-level executable code:
 * module-scope `let` is in the temporal dead zone until evaluated, and a
 * ReferenceError from `render()` aborts the
 * rest of the module's top-level wiring (this killed jog boot once already).
 */
let jogLive = false;
/** Writes the live jog pose only when the jogged robot moves; see jog-feed.ts. */
const jogFeed = createJogFeedGate();
let simArmed = false;
/** Drops the header arm popover's Escape / outside-click listeners. */
let armPopoverRelease: (() => void) | null = null;
let jogLastError = "";
let jogPadVisible = readJogPadVisible();
/** Whether the telemetry panel is shown beside the Jog column; a toolbar
 *  toggle next to Jog, remembered per browser (owner, 2026-09-20: the panel
 *  should be hideable like the Jog pane is). */
const TELEMETRY_VISIBLE_KEY = "olp.telemetry.shown";
function readTelemetryVisible(): boolean {
  try { return localStorage.getItem(TELEMETRY_VISIBLE_KEY) !== "0"; } catch { return true; }
}
function writeTelemetryVisible(shown: boolean): void {
  try { localStorage.setItem(TELEMETRY_VISIBLE_KEY, shown ? "1" : "0"); } catch { /* the choice lasts the session */ }
}
let telemetryVisible = readTelemetryVisible();
/** The dock height the Jog panel needs for its switch, the joint table and the
 *  telemetry strip's tabs, plot and values table; measured on the served app. */
const JOG_DOCK_MIN_HEIGHT_PX = 440;
/** Without the strip the joints table alone sets the floor: heading, caption,
 *  toolbar, header row and six 32 px rows; the column scrolls below that. */
const JOG_DOCK_MIN_HEIGHT_JOINTS_PX = 320;
let jogHoldTimer: number | null = null;
// Unverified display policy; feedback retains one native command fence.
const jogOperatorFence = createOperatorFence();
let jogStatePoll: number | null = null;
let jogRestartTimer: number | null = null;
let jogStartInFlight = false;

// Module loggers: every status line the app writes also lands in the log,
// with its module, so the status bar can say "something went wrong" and
// the panel can say what and when. See src/ui/log.ts.
const appLogger = appLog.logger("app");
const cadLogger = appLog.logger("cad");
const viewerLogger = appLog.logger("viewer");
const railsLogger = appLog.logger("rails");
const denseLogger = appLog.logger("deploy");
const storageLogger = appLog.logger("storage");
const consoleLogger = appLog.logger("console");
{
  // Console warnings and errors, and anything uncaught, are log entries
  // too: a stack in the devtools nobody has open is a stack nobody saw.
  const warn = console.warn.bind(console);
  const error = console.error.bind(console);
  // Library chatter that is not a problem stays in the log as debug, so the
  // bar is not orange before the operator has done anything.
  const noise = [
    /ColladaLoader: You are loading an asset with a Z-UP coordinate system/,
  ];
  console.warn = (...args: unknown[]) => {
    warn(...args);
    const text = args.map(String).join(" ");
    consoleLogger.at(noise.some((pattern) => pattern.test(text)) ? "debug" : "warn", text);
  };
  console.error = (...args: unknown[]) => {
    error(...args);
    consoleLogger.error(args.map(String).join(" "), args.find((arg) => arg instanceof Error));
  };
  window.addEventListener("error", (event) => {
    appLogger.error(event.message || "Uncaught error", event.error ?? undefined);
  });
  window.addEventListener("unhandledrejection", (event) => {
    const reason = event.reason;
    appLogger.error(reason instanceof Error ? reason.message : String(reason), reason);
  });
}

function setStorageStatus(text: string): void {
  setBusyText(query("#storage-status"), text);
  appLogger.at(levelOfStatus(text), text);
}

const app = query<HTMLDivElement>("#app");
app.innerHTML = `
  <main class="app-shell">
    <header class="topbar">
      <div class="brand">
        <svg class="brand-mark" viewBox="0 0 975 370" role="img" aria-label="Advanced Metal Research">
          <!-- The company mark, amr-website/assets/img/amr-mark.svg, filled with currentColor. -->
          <path d="M459.958 337.987L255.11 0H131.807L112.273 30.0028L319.11 370H439.116L459.958 337.987Z"/>
          <path d="M660.958 337.987L456.11 0H332.807L313.273 30.0028L520.11 370H640.116L660.958 337.987Z"/>
          <path d="M860.958 337.987L656.11 0H532.807L513.273 30.0028L720.11 370H840.116L860.958 337.987Z"/>
          <path d="M135.359 132L279.109 370H19.1094L0 337.93L135.359 132Z"/>
          <path d="M838.859 238L695.109 6.11097e-06L955.109 1.35821e-05L974.288 31.9951L838.859 238Z"/>
        </svg>
        <div>
          <p class="eyebrow">ROSIE / OFFLINE PROGRAMMING V1</p>
          <h1>Weld workstation</h1>
        </div>
        <span class="brand-divider" aria-hidden="true"></span>
        <div class="top-title">
          <span id="program-menu-host" class="program-menu-host"></span>
          <button id="program-title" class="program-title" type="button" title="Rename the program"></button>
          <span id="save-state" class="save-state">LOCAL</span>
          <span id="cell-sync" class="cell-sync" hidden></span>
        </div>
        <nav id="app-menubar" class="menubar" aria-label="Application menus"></nav>
      </div>
      <div class="mode-cluster">
        <span id="target-badge" class="mode-badge is-offline">OFFLINE</span>
        <span
          id="session-mode-badge"
          class="mode-badge is-offline"
          title="This session is simulation. No physical robot output."
        >SIMULATION</span>
        <button id="cell-stop-pill" class="mode-badge is-real" type="button" hidden>STOP</button>
        <button
          type="button"
          id="drive-arm-badge"
          class="mode-badge is-disarmed"
          title="Connect to a cell before arming."
        >DISARMED</button>
        <button id="cell-clear-faults" class="mode-badge" type="button" hidden title="Clear eligible faults while disarmed">Clear faults</button>
        <div id="cell-clear-faults-result" class="menu" hidden>
          <p id="cell-clear-faults-message" class="inspector-note" role="status" aria-live="polite"></p>
          <button id="cell-clear-faults-dismiss" class="mini-button" type="button">Close</button>
        </div>
        <div id="cell-arm-popover" class="menu" hidden>
          <p id="cell-arm-question" class="inspector-note"></p>
          <button id="cell-arm-confirm" class="mini-button" type="button">Confirm</button>
          <button id="cell-arm-cancel" class="mini-button" type="button">Cancel</button>
          <p id="cell-arm-reason" class="inspector-note is-warn" aria-live="polite"></p>
        </div>
        <span id="plan-badge" class="mode-badge">NO PLAN</span>
        <span id="revision-label" class="revision-label">REV —</span>
      </div>
    </header>

    <div id="app-toolbar-host" class="app-toolbar-host"></div>

    <dialog
      id="new-program-dialog"
      class="new-program-dialog"
      aria-labelledby="new-program-dialog-title"
      aria-describedby="new-program-dialog-description"
    >
      <form id="new-program-form" class="new-program-form" method="dialog">
        <div class="new-program-dialog-accent" aria-hidden="true"></div>
        <div class="new-program-dialog-heading">
          <div>
            <p class="eyebrow">NEW PROGRAM</p>
            <h2 id="new-program-dialog-title">Create a blank robot program</h2>
          </div>
          <button
            id="new-program-close"
            class="new-program-dialog-close"
            type="button"
            aria-label="Close new program dialog"
          >×</button>
        </div>
        <p id="new-program-dialog-description" class="new-program-dialog-copy">
          Start fresh with one motion row using the current robot model.
        </p>
        <label class="field new-program-name-field">
          <span>Program name</span>
          <input
            id="new-program-name"
            type="text"
            maxlength="80"
            autocomplete="off"
            spellcheck="false"
          />
        </label>
        <div class="new-program-dialog-note">
          <span aria-hidden="true">↓</span>
          <p>Your current program is saved automatically before this one opens.</p>
        </div>
        <div class="new-program-dialog-actions">
          <button id="new-program-cancel" class="button secondary" type="button">Cancel</button>
          <button class="button primary" type="submit">Create program</button>
        </div>
      </form>
    </dialog>

    <dialog
      id="record-blocked-dialog"
      class="new-program-dialog record-blocked-dialog"
      aria-labelledby="record-blocked-title"
      aria-describedby="record-blocked-copy"
    >
      <div class="new-program-form">
        <div class="new-program-dialog-heading">
          <div>
            <p class="eyebrow">RECORD BLOCKED</p>
            <h2 id="record-blocked-title">This program uses older robot settings</h2>
          </div>
          <button id="record-blocked-close" class="new-program-dialog-close" type="button" aria-label="Close">×</button>
        </div>
        <p id="record-blocked-copy" class="new-program-dialog-copy"></p>
        <div id="record-blocked-changes"></div>
        <div class="new-program-dialog-note">
          <span aria-hidden="true">↓</span>
          <p id="record-blocked-note"></p>
        </div>
        <div class="new-program-dialog-actions">
          <button id="record-blocked-review" class="button secondary" type="button">Review in Workspace</button>
          <button id="record-blocked-new" class="button secondary" type="button">New program</button>
          <button id="record-blocked-accept" class="button primary" type="button">Accept current settings</button>
        </div>
      </div>
    </dialog>

    <dialog
      id="node-editor"
      class="node-editor-dialog"
      aria-label="Node parameters"
    ></dialog>

    <section class="workspace">
      <aside class="authoring-panel panel">
        <div class="panel-kicker"><p class="eyebrow">WORKSPACE TREE</p></div>
        <div id="program-tree-host" class="program-tree-host"></div>
        <div
          id="inspector-resizer"
          class="inspector-resizer"
          role="separator"
          aria-label="Resize the selection panel"
          aria-orientation="horizontal"
          tabindex="0"
        ></div>
        <div id="inspector-host" class="inspector-host"></div>
        <div id="setup-sections" hidden>
        <input id="project-file" type="file" accept=".json,application/json" hidden />
        <section data-setup-section="project">
        <label class="field" data-setup-item="project.name">
          <span>Name</span>
          <input id="project-name" type="text" autocomplete="off" />
        </label>

        <div class="source-actions" data-setup-item="project.import">
          <button id="import-step" class="button secondary" type="button">Import STEP</button>
          <input id="step-file" type="file" accept=".step,.stp" multiple hidden />
        </div>
        <label class="field step-examples" data-setup-item="project.examples">
          <span>Examples</span>
          <div class="source-actions">
            <select id="step-example" aria-label="Example part" disabled>
              <option value="">Loading examples…</option>
            </select>
            <button id="open-example" class="button secondary" type="button" disabled title="Import the selected example part through local CadQuery">Open example</button>
          </div>
          <small id="step-example-note" class="step-example-note"></small>
        </label>
        </section>

        <section data-setup-section="robot" hidden>
        <label class="field" data-setup-item="robot.model">
          <span>Robot model</span>
          <select id="robot-model">
            <option value="rosie_1400_v3">Rosie 1400 V3</option>
            <option value="rosie_1420_v1">Rosie 1420 V1</option>
          </select>
        </label>
        <div id="cell-settings" class="cell-settings"></div>
        <div id="dense-robot-host" class="dense-robot-host"></div>
        </section>

        <section data-setup-section="planner" hidden>
        <label class="field" data-setup-item="planner.mode">
          <span>Planner</span>
          <select id="target-mode" aria-describedby="target-state">
            <option value="offline_preview">None (browser-only, no planner)</option>
            <option value="local_simulator" selected>Tesseract · checking</option>
          </select>
        </label>
        <div data-setup-item="planner.status">
        <p id="target-state" class="field-help" aria-live="polite">Tesseract · checking</p>
        <div class="target-readout" aria-live="polite">
          <p id="local-simulator-status">Tesseract status not loaded.</p>
          <p id="connected-target-status">Connected targets are manifest-only and read-only.</p>
        </div>
        </div>
        </section>

        <section data-setup-section="weld" hidden>
        <div class="section-heading">
          <div>
            <p class="eyebrow">WELD PROCESS</p>
            <h3>Pass defaults</h3>
          </div>
        </div>
        <p class="section-note">
          Used when CAD seams generate their approach / weld / retract rows
        </p>
        <div class="process-grid">
          <label class="field" data-setup-item="weld.process">
            <span>Process</span>
            <select data-process-field="process">
              <option value="none">none</option>
              <option value="gmaw">GMAW / MIG</option>
              <option value="gtaw">GTAW / TIG</option>
              <option value="laser">Laser</option>
              <option value="custom">Custom</option>
            </select>
          </label>
          <label class="field" data-setup-item="weld.speeds">
            <span>Travel · m/s</span>
            <input data-process-field="travelSpeedMps" type="number" min="0" step="0.001" inputmode="decimal" />
          </label>
          <label class="field" data-setup-item="weld.speeds">
            <span>Approach · m/s</span>
            <input data-process-field="approachSpeedMps" type="number" min="0" step="0.001" inputmode="decimal" />
          </label>
          <label class="field" data-setup-item="weld.speeds">
            <span>Retract · m/s</span>
            <input data-process-field="retractSpeedMps" type="number" min="0" step="0.001" inputmode="decimal" />
          </label>
          <label class="field" data-setup-item="weld.electrical">
            <span>Voltage · V</span>
            <input data-process-field="voltageV" type="number" min="0" step="0.1" inputmode="decimal" />
          </label>
          <label class="field" data-setup-item="weld.electrical">
            <span>Current · A</span>
            <input data-process-field="amperageA" type="number" min="0" step="1" inputmode="decimal" />
          </label>
          <label class="field" data-setup-item="weld.electrical">
            <span>Wire · m/min</span>
            <input data-process-field="wireFeedMPerMin" type="number" min="0" step="0.1" inputmode="decimal" />
          </label>
          <label class="field" data-setup-item="weld.torch">
            <span>Work angle · °</span>
            <input data-process-field="torchWorkAngleDeg" type="number" min="-89" max="89" step="1" inputmode="decimal" />
          </label>
          <label class="field" data-setup-item="weld.torch">
            <span>Travel angle · °</span>
            <input data-process-field="torchTravelAngleDeg" type="number" min="-89" max="89" step="1" inputmode="decimal" />
          </label>
          <label class="field" data-setup-item="weld.torch">
            <span>Spin · °</span>
            <input data-process-field="torchSpinAngleDeg" type="number" min="-180" max="180" step="1" inputmode="decimal" />
          </label>
          <label class="field" data-setup-item="weld.weave">
            <span>Weave</span>
            <select data-process-field="weavePattern">
              <option value="none">none</option>
              <option value="zigzag">zigzag</option>
              <option value="circular">circular</option>
              <option value="trapezoidal">trapezoidal</option>
            </select>
          </label>
          <label class="field" data-setup-item="weld.weave">
            <span>Amplitude · mm</span>
            <input data-process-field="weaveAmplitudeMm" type="number" min="0" step="0.1" inputmode="decimal" />
          </label>
          <label class="field" data-setup-item="weld.weave">
            <span>Frequency · Hz</span>
            <input data-process-field="weaveFrequencyHz" type="number" min="0" step="0.1" inputmode="decimal" />
          </label>
          <label class="field" data-setup-item="weld.weave">
            <span>Dwell · ms</span>
            <input data-process-field="weaveDwellMs" type="number" min="0" step="10" inputmode="decimal" />
          </label>
        </div>
        </section>
        </div>
      </aside>

      <div
        id="authoring-resizer"
        class="workspace-resizer"
        role="separator"
        aria-label="Resize authoring panel"
        aria-orientation="vertical"
        tabindex="0"
      ></div>

      <section class="stage-panel panel" aria-label="Offline 3D workstation">
        <div id="workstation-stage" class="workstation-stage">
          <canvas
            id="workstation-canvas"
            role="img"
            aria-label="Interactive Rosie workstation showing the loaded spreadsheet path"
          >Rosie offline workstation preview</canvas>
          <div id="orbit-pane" class="workstation-pane orbit-pane" aria-label="Orbit camera">
            <span class="pane-label">ORBIT</span>
            <canvas
              id="world-axis-indicator"
              class="world-axis-indicator"
              width="184"
              height="184"
              role="img"
              aria-label="World coordinate orientation: X is red, Y is green, and Z is blue"
            ></canvas>
            <div id="view-menu-host" class="view-menu-host"></div>
          </div>
          <div id="j6-pane" class="workstation-pane j6-pane" aria-label="J6 camera">
            <span class="pane-label">J6 CAM</span>
          </div>
          <div id="tcp-pane" class="workstation-pane tcp-pane" aria-label="TCP camera">
            <span class="pane-label">TCP CAM</span>
            <span class="tcp-crosshair" aria-hidden="true"></span>
          </div>
          <div
            id="camera-column-resizer"
            class="workstation-resizer camera-column-resizer"
            role="separator"
            aria-label="Resize camera column"
            aria-orientation="vertical"
            tabindex="0"
          ></div>
          <div
            id="camera-row-resizer"
            class="workstation-resizer camera-row-resizer"
            role="separator"
            aria-label="Resize J6 and TCP camera panes"
            aria-orientation="horizontal"
            tabindex="0"
          ></div>
          <div
            id="weld-drawer"
            class="weld-drawer"
            aria-label="Weld draft from selected edges"
            hidden
          ></div>
          <div id="placement-card-host"></div>
        </div>
        <div class="stage-state">
          <span class="safety-dot" aria-hidden="true"></span>
          <span id="viewer-status" aria-live="polite">Loading offline workstation…</span>
        </div>
        <div class="preview-watermark">PREVIEW · NO MOTION OUTPUT</div>
      </section>

      <div id="side-dock-host" class="side-dock-host"></div>

      <div
        id="spreadsheet-resizer"
        class="workspace-resizer spreadsheet-resizer"
        role="separator"
        aria-label="Resize program spreadsheet height"
        aria-orientation="horizontal"
        tabindex="0"
      ></div>

      <div id="bottom-workspace" class="spreadsheet-panel panel bottom-workspace">
      <section id="program-sheet" class="program-sheet" aria-label="Robot program spreadsheet">
        <div id="program-lint" class="program-lint" hidden></div>
        <div class="spreadsheet-toolbar">
          <div class="spreadsheet-toolbar-edit">
            <button id="edit-node" class="mini-button primary" type="button" title="Open the selected node's parameters">EDIT</button>
            <button id="record-waypoint" class="mini-button" type="button">RECORD WAYPOINT</button>
            <button id="reteach-waypoint" class="mini-button" type="button">RETEACH</button>
            <button id="record-via" class="mini-button" type="button">RECORD VIA</button>
            <button id="add-dwell" class="mini-button" type="button" title="Add a dwell after the selected node">+ DWELL</button>
            <button id="add-io" class="mini-button" type="button" title="Add a digital output after the selected node">+ IO</button>
            <button id="add-home" class="mini-button" type="button" title="Add a home move at the end of the program">+ HOME</button>
            <button id="delete-node" class="mini-button" type="button" title="Delete the selected node">DELETE</button>
            <button id="toggle-advanced" class="mini-button" type="button" aria-pressed="false" title="Show the compiled robot rows (read-only)">ADVANCED</button>
            <span id="row-count" class="spreadsheet-status">0 nodes</span>
          </div>
          <div class="spreadsheet-toolbar-ops">
            <div class="spreadsheet-ops-group spreadsheet-catalog">
              <select id="catalog-program" aria-label="Motion Server catalogue" title="Motion Server catalogue">
                <option value="">Catalogue not loaded</option>
              </select>
              <button id="catalog-refresh" class="mini-button" type="button">REFRESH</button>
              <button id="catalog-open" class="mini-button" type="button" disabled>Open</button>
              <button id="catalog-save" class="mini-button primary" type="button" title="Save shared program">Save</button>
              <span id="catalog-status" class="spreadsheet-status" aria-live="polite">
                Local draft · checking Motion Server catalogue
              </span>
            </div>
            <label class="spreadsheet-ops-group spreadsheet-mode">
              <span>Mode</span>
              <select id="program-execution-mode" aria-label="Program execution mode">
                <option value="simulation">Simulation</option>
                <option value="dry_run">Dry run</option>
                <option value="production">Production · target plan required</option>
              </select>
            </label>
            <div class="spreadsheet-ops-group spreadsheet-plan">
              <strong id="plan-state">MISSING</strong>
              <span id="plan-source">not generated</span>
              <button id="generate-preview" class="mini-button primary" type="button">Plan &amp; simulate</button>
              <button id="target-plan" class="mini-button" type="button" hidden disabled>Plan on selected robot</button>
              <p id="plan-detail" hidden>Authoring changes require a new plan from the selected canonical planner.</p>
              <details class="spreadsheet-plan-pop">
                <summary>Evidence</summary>
                <div id="plan-evidence" class="plan-evidence" aria-live="polite">
                  <span>Planner evidence unavailable</span>
                </div>
              </details>
            </div>
            <div class="spreadsheet-ops-group playback">
              <button id="play" class="play-button" type="button" aria-label="Play preview">▶</button>
              <button id="stop" class="icon-button" type="button" aria-label="Stop preview">■</button>
              <button
                id="step-back"
                class="icon-button playback-step"
                type="button"
                title="Step back one move"
                aria-label="Step back one move"
              >◀|</button>
              <button
                id="step-forward"
                class="icon-button playback-step"
                type="button"
                title="Step forward one move"
                aria-label="Step forward one move"
              >|▶</button>
              <button
                id="home-robot"
                class="button secondary playback-home"
                type="button"
                title="Return the simulated robot to J1–J9 = 0"
                aria-label="Home simulated robot"
              >Home</button>
              <input id="timeline" type="range" min="0" max="1000" value="0" aria-label="Preview timeline" />
              <span id="playback-time">0.0 s</span>
              <select
                id="playback-rate"
                class="playback-rate"
                title="Playback speed: trajectory seconds per real second. The trajectory itself is unchanged."
                aria-label="Playback speed"
              >
                <option value="1" selected>1×</option>
                <option value="2">2×</option>
                <option value="4">4×</option>
                <option value="8">8×</option>
                <option value="16">16×</option>
              </select>
            </div>
            <div id="dense-playback" class="spreadsheet-ops-group dense-playback" hidden></div>
          </div>
        </div>
        <div id="execution-chrome" class="execution-chrome" hidden>
          <div class="execution-lock">
            <span class="lock-icon" aria-hidden="true">×</span>
            <div>
              <strong id="execution-review-title">Execution review</strong>
              <p id="execution-review-summary"></p>
              <ul id="execution-review" class="execution-review" aria-label="Execution review blockers"></ul>
            </div>
          </div>
          <button
            id="review-execute"
            class="button danger wide"
            type="button"
            disabled
            title="Review the exact robot-stored plan and live safety evidence"
          >Review physical start</button>
          <button
            id="start-execution"
            class="button danger wide"
            type="button"
            disabled
          >Start reviewed plan</button>
          <button
            id="stop-execution"
            class="button secondary wide"
            type="button"
            hidden
          >Stop robot safely</button>
        </div>
        <div id="program-spreadsheet" class="program-spreadsheet"></div>
      </section>
      <div id="jog-resizer" class="workspace-resizer" role="separator" aria-label="Resize jog commands" aria-orientation="vertical" tabindex="0" hidden></div>
          <section id="jog-panel" class="jog-panel dock-section" aria-label="Jog" hidden>
            <div class="jog-panel-heading">
              <h3>Jog</h3>
              <div class="run-switch jog-mode-switch" role="tablist" aria-label="Jog mode">
                <button type="button" class="jog-mode-tab is-active" role="tab" data-jog-mode="joints" aria-selected="true" title="Joint hold and move by angle">Joints</button>
                <button type="button" class="jog-mode-tab" role="tab" data-jog-mode="cartesian" aria-selected="false" title="Cartesian hold and step in BASE or TOOL">Cartesian</button>
              </div>
              <button id="jog-panel-close" class="mini-button jog-panel-close" type="button" title="Hide Jog; the toolbar Jog toggle opens it again" aria-label="Hide the Jog panel">×</button>
            </div>
            <div class="jog-column">
            <div class="jog-section jog-section-joints">
              <div class="jog-section-head">
                <span class="jog-section-title">Joints</span>
                <span class="jog-section-drives">selected machine</span>
              </div>
              <p id="jog-joints-target" class="jog-section-note" data-posture="none" aria-live="polite">No machine connected. Joint jog needs a connected, armed machine; choose one under Cells.</p>
              <div id="dense-joint-jog" class="dense-joint-jog"></div>
            </div>
            <div id="spreadsheet-jog" class="jog-section jog-section-cartesian spreadsheet-jog" hidden>
              <div class="jog-section-head">
                <span class="jog-section-title">Cartesian</span>
                <span id="cartesian-jog-tag" class="jog-section-drives is-preview">BROWSER PREVIEW</span>
              </div>
              <span id="jog-cartesian-caption" class="jog-caption" hidden>Drives the browser preview only, not the selected machine.</span>
              <p id="cartesian-jog-target" class="jog-section-note" hidden>Drives the browser preview. Arm preview to jog.</p>
              <div class="jog-cartesian-controls">
                <button id="sim-arm-toggle" class="mini-button primary sim-arm-toggle" type="button" aria-pressed="false" title="Arm the browser simulator for Cartesian jog. Never a physical robot.">Arm preview</button>
                <select id="jog-frame" title="${CARTESIAN_JOG_DISABLED_TITLE}" aria-label="Jog frame" disabled>
                  <option value="base">BASE</option>
                  <option value="tool">TOOL</option>
                </select>
                <button id="cartesian-zero" class="mini-button cartesian-zero" type="button" title="Move all joints together to 0° at the selected speed; this is not a collision-planned path">All joints to 0°</button>
                <label id="cartesian-speed-label" hidden>Speed <select id="cartesian-speed" aria-label="Cartesian speed fraction">
                  ${JOINT_JOG_FRACTIONS.map(f => `<option value="${f}" ${f === 0.1 ? "selected" : ""}>${f * 100}%</option>`).join("")}
                </select></label>
              </div>
              <p id="cartesian-machine-reason" class="field-help" aria-live="polite" hidden></p>
              <div id="cartesian-step-controls" hidden>
                <label>Mode <select id="cartesian-mode" aria-label="Cartesian Hold or Step"><option value="hold">Hold</option><option value="step">Step</option></select></label>
                <label><span id="cartesian-distance-label">Distance (mm)</span> <input id="cartesian-distance" type="number" min="0" max="1000" step="any" value="100"></label>
                <label><span id="cartesian-angle-label">Angle (degrees)</span> <input id="cartesian-angle" type="number" min="0" max="180" step="any" value="15"></label>
                <p id="cartesian-step-help" class="field-help">Step sends one move of this distance or angle per press; Stop or Disarm cancels it. Hold moves while a button is held and suits a local link. Step is chosen for you on a slow link.</p>
              </div>
              <!-- The pad reads like a controller: a cross for X and Y with X forward at
                   the top and Y to the left where the base frame puts it, a column for Z,
                   a cross for the wrist rotations RX and RY, a column for RZ. Every key
                   keeps its axis name under the arrow, so the frame is never guessed. -->
              <div id="spreadsheet-cartesian-grid" class="jog-cartesian-grid is-disabled" aria-disabled="true">
                <div class="jog-pad jog-pad-cross" role="group" aria-label="Move X and Y">
                  <span class="jog-pad-title">Move</span>
                  <button type="button" class="mini-button jog-hold jog-key" style="grid-area: up" data-jog-axes="1,0,0,0,0,0" disabled title="${CARTESIAN_JOG_DISABLED_TITLE}"><span class="jog-key-glyph">▲</span><span class="jog-key-name">X+</span></button>
                  <button type="button" class="mini-button jog-hold jog-key" style="grid-area: left" data-jog-axes="0,1,0,0,0,0" disabled title="${CARTESIAN_JOG_DISABLED_TITLE}"><span class="jog-key-glyph">◀</span><span class="jog-key-name">Y+</span></button>
                  <span class="jog-pad-hub" style="grid-area: hub" aria-hidden="true"></span>
                  <button type="button" class="mini-button jog-hold jog-key" style="grid-area: right" data-jog-axes="0,-1,0,0,0,0" disabled title="${CARTESIAN_JOG_DISABLED_TITLE}"><span class="jog-key-glyph">▶</span><span class="jog-key-name">Y−</span></button>
                  <button type="button" class="mini-button jog-hold jog-key" style="grid-area: down" data-jog-axes="-1,0,0,0,0,0" disabled title="${CARTESIAN_JOG_DISABLED_TITLE}"><span class="jog-key-glyph">▼</span><span class="jog-key-name">X−</span></button>
                </div>
                <div class="jog-pad jog-pad-column" role="group" aria-label="Move Z">
                  <span class="jog-pad-title">Lift</span>
                  <button type="button" class="mini-button jog-hold jog-key" style="grid-area: up" data-jog-axes="0,0,1,0,0,0" disabled title="${CARTESIAN_JOG_DISABLED_TITLE}"><span class="jog-key-glyph">▲</span><span class="jog-key-name">Z+</span></button>
                  <button type="button" class="mini-button jog-hold jog-key" style="grid-area: down" data-jog-axes="0,0,-1,0,0,0" disabled title="${CARTESIAN_JOG_DISABLED_TITLE}"><span class="jog-key-glyph">▼</span><span class="jog-key-name">Z−</span></button>
                </div>
                <div class="jog-pad jog-pad-cross" role="group" aria-label="Tilt RX and RY">
                  <span class="jog-pad-title">Tilt</span>
                  <button type="button" class="mini-button jog-hold jog-key" style="grid-area: up" data-jog-axes="0,0,0,0,1,0" disabled title="${CARTESIAN_JOG_DISABLED_TITLE}"><span class="jog-key-glyph">⤒</span><span class="jog-key-name">RY+</span></button>
                  <button type="button" class="mini-button jog-hold jog-key" style="grid-area: left" data-jog-axes="0,0,0,-1,0,0" disabled title="${CARTESIAN_JOG_DISABLED_TITLE}"><span class="jog-key-glyph">↶</span><span class="jog-key-name">RX−</span></button>
                  <span class="jog-pad-hub" style="grid-area: hub" aria-hidden="true"></span>
                  <button type="button" class="mini-button jog-hold jog-key" style="grid-area: right" data-jog-axes="0,0,0,1,0,0" disabled title="${CARTESIAN_JOG_DISABLED_TITLE}"><span class="jog-key-glyph">↷</span><span class="jog-key-name">RX+</span></button>
                  <button type="button" class="mini-button jog-hold jog-key" style="grid-area: down" data-jog-axes="0,0,0,0,-1,0" disabled title="${CARTESIAN_JOG_DISABLED_TITLE}"><span class="jog-key-glyph">⤓</span><span class="jog-key-name">RY−</span></button>
                </div>
                <div class="jog-pad jog-pad-column" role="group" aria-label="Turn RZ">
                  <span class="jog-pad-title">Turn</span>
                  <button type="button" class="mini-button jog-hold jog-key" style="grid-area: up" data-jog-axes="0,0,0,0,0,1" disabled title="${CARTESIAN_JOG_DISABLED_TITLE}"><span class="jog-key-glyph">↺</span><span class="jog-key-name">RZ+</span></button>
                  <button type="button" class="mini-button jog-hold jog-key" style="grid-area: down" data-jog-axes="0,0,0,0,0,-1" disabled title="${CARTESIAN_JOG_DISABLED_TITLE}"><span class="jog-key-glyph">↻</span><span class="jog-key-name">RZ−</span></button>
                </div>
              </div>
              <div id="jog-live-host"></div>
              <div id="jog-operator-feedback" class="operator-feedback-strip" aria-live="polite" hidden></div>
            </div>
            </div>
            <div id="jog-column-resizer" class="workspace-resizer jog-column-resizer" role="separator" aria-label="Resize the Jog column against the telemetry plot" aria-orientation="vertical" tabindex="0"></div>
            <section id="telemetry-strip" class="telemetry-strip" aria-label="Telemetry"></section>
          </section>
      </div>
    </section>

    <footer class="statusbar" title="Click for the log">
      <span class="statusbar-label"><span class="statusbar-chevron" aria-hidden="true">^</span>LOGS</span>
      <span id="log-status-host"></span>
      <span id="storage-status">Opening local project store…</span>
      <span id="cad-status">Dog Box fallback ready</span>
      <span id="planner-status">PLANNER: DISABLED</span>
      <span id="cell-network">NETWORK: OFFLINE</span>
      <span id="cell-physical-play">PHYSICAL PLAY: DISABLED</span>
      <span id="cell-real-output">REAL OUTPUT: DISABLED</span>
    </footer>
  </main>
`;

const disposeViewportShellSizing = installViewportShellSizing({
  shell: query(".app-shell"),
});
window.addEventListener("beforeunload", disposeViewportShellSizing, { once: true });

const resizableLayout = installResizableLayout({
  dockMinHeight: () => (dockPaneShown() ? (telemetryVisible ? JOG_DOCK_MIN_HEIGHT_PX : JOG_DOCK_MIN_HEIGHT_JOINTS_PX) : 0),
  workspace: query(".workspace"),
  stage: query("#workstation-stage"),
  authoringResizer: query("#authoring-resizer"),
  spreadsheetResizer: query("#spreadsheet-resizer"),
  cameraColumnResizer: query("#camera-column-resizer"),
  cameraRowResizer: query("#camera-row-resizer"),
  jogResizer: query<HTMLElement>("#jog-resizer"),
  bottomWorkspace: query<HTMLElement>("#bottom-workspace"),
  jogColumnResizer: query<HTMLElement>("#jog-column-resizer"),
  jogPanel: query<HTMLElement>("#jog-panel"),
});
window.addEventListener("beforeunload", () => resizableLayout.dispose(), { once: true });

// The right dock: one handle on the window's right edge. Simulate &
// deploy is one pane because both
// act on the same plan -- the preview replays it, the robot runs it --
// and the plan list that is coming sits above both: SIMULATE (plan state,
// transport) over DEPLOY (mode, target, load, review). The elements move
// in from the bottom bar and the Workspace inspector with their handlers.
const sideDock = new SideDock({
  tabs: [{ id: "run", label: "Simulate & deploy" }],
  storageKey: "olp.dock.open",
}).mount(query<HTMLElement>("#side-dock-host"));
// The catalogue is the Project menu's (open, save, refresh); its old row
// keeps its controls, out of sight, because the catalogue code reads them.
query<HTMLElement>("#setup-sections").append(query<HTMLElement>(".spreadsheet-ops-group.spreadsheet-catalog"));
// Two panels behind one handle, Simulate and Deploy, with a switch at
// the top; which one is up decides what the viewer shows (see poseSource).
{
  const pane = sideDock.panes.get("run")!;
  const runSwitch = element("div", "run-switch");
  runSwitch.setAttribute("role", "tablist");
  for (const [panel, label] of [["simulate", "Simulate"], ["deploy", "Deploy"]] as const) {
    const button = element("button", "run-switch-tab", label);
    button.type = "button";
    button.setAttribute("role", "tab");
    button.dataset.runPanel = panel;
    button.addEventListener("click", () => setRunPanel(panel));
    runSwitch.append(button);
  }
  // A fact row, the guide's: a kicker naming the fact, then what holds it.
  const fact = (label: string, ...content: HTMLElement[]): HTMLElement => {
    const row = element("div", "fact");
    row.append(element("span", "fact-label", label), ...content);
    return row;
  };
  const simulate = element("div", "run-panel");
  simulate.dataset.runPanel = "simulate";
  simulate.setAttribute("role", "tabpanel");
  simulate.append(
    fact("Plan", query<HTMLElement>(".spreadsheet-ops-group.spreadsheet-plan")),

  );
  const playback = query<HTMLElement>(".spreadsheet-ops-group.playback");
  playback.setAttribute("role", "group");
  playback.setAttribute("aria-label", "Program preview controls");
  playback.prepend(element("span", "playback-label", "PREVIEW"));
  const planSpeed = element("label", "plan-speed", "PLAN SPEED ");
  planSpeed.title = "Whole-plan motion speed. Change before planning; replan and reload to apply. Dwell times stay unchanged.";
  const speedInput = document.createElement("input");
  speedInput.id = "plan-speed";
  speedInput.type = "text";
  speedInput.inputMode = "decimal";
  speedInput.setAttribute("aria-label", "Plan speed percent");
  speedInput.addEventListener("change", () => {
    const percent = Number(speedInput.value);
    if (!Number.isFinite(percent) || percent < 1 || percent > 100) {
      setStorageStatus("Plan speed must be between 1% and 100%.");
      syncNodeToolbar();
      return;
    }
    if (percent / 100 === (project.programV2.speedScale ?? 1)) return;
    mutate("nodes", (draft) => { draft.programV2.speedScale = percent / 100; });
    setStorageStatus(`Plan speed ${percent}%. Plan and reload to apply.`);
  });
  speedInput.addEventListener("keydown", (event) => {
    if (event.key === "Enter") speedInput.blur();
    if (event.key === "Escape") { syncNodeToolbar(); speedInput.blur(); }
  });
  planSpeed.append(speedInput, "%");
  query<HTMLElement>(".spreadsheet-toolbar").append(planSpeed);
  query<HTMLElement>(".spreadsheet-toolbar").append(playback);
  const deploy = element("div", "run-panel");
  deploy.dataset.runPanel = "deploy";
  deploy.setAttribute("role", "tabpanel");
  // The target is the cell chosen from Cells; the row names it and
  // opens the Cells dialog to change it. The execution mode select
  // (simulation / dry run / production) gates the manifest-backed Robot
  // V4 path, so it stays with the other hidden setup fields. Native dry run
  // is selected in DensePanel's Run row and never changes the authored plan.
  const target = element("div", "deploy-target");
  const targetName = element("span", "fact-value deploy-target-name");
  targetName.id = "deploy-target-name";
  const changeTarget = element("button", "mini-button deploy-target-change", "CELL…");
  changeTarget.type = "button";
  changeTarget.title = "Choose the cell, pull its settings, manage the registry";
  changeTarget.addEventListener("click", () => setRobotOpen(true));
  target.append(targetName, changeTarget);
  query<HTMLElement>("#setup-sections").append(query<HTMLElement>(".spreadsheet-ops-group.spreadsheet-mode"));
  deploy.append(
    fact("Target", target),
    fact("Machine", query<HTMLElement>("#dense-robot-host")),
    fact("Run", query<HTMLElement>("#dense-playback")),
    query<HTMLElement>("#execution-chrome"),
  );
  // Two sections behind the handle: the run panels, and the plans the
  // program has had (what Simulate replays and Deploy loads is the active
  // one, chosen here).
  const run = element("section", "dock-section run-section");
  run.setAttribute("aria-label", "Simulate and deploy");
  run.append(runSwitch, simulate, deploy);
  const plans = element("section", "dock-section plan-history");
  plans.setAttribute("aria-label", "Plans");
  plans.append(element("p", "eyebrow dock-section-heading", "PLANS"), element("div", "plan-history-list"));
  pane.append(plans, run);
}

// The left panel: the program as a tree above, the selection below. The
// old setup sections stay in the DOM (hidden) as the homes of fields whose
// handlers are bound by id; the inspector and the Weld presets card adopt
// them from there.
const programTree = new TreeView("Program").mount(query<HTMLElement>("#program-tree-host"));
const inspector = new Inspector().mount(query<HTMLElement>("#inspector-host"));
const setupSections = query<HTMLElement>("#setup-sections");
// The bar between tree and inspector: drag to give one more room. The
// height is a CSS variable on the panel so both halves scroll on their own.
{
  const panel = query<HTMLElement>(".authoring-panel");
  const resizer = query<HTMLElement>("#inspector-resizer");
  let dragStartY = 0;
  let dragStartHeight = 0;
  const onMove = (event: PointerEvent): void => {
    const next = Math.max(120, Math.min(panel.clientHeight - 120, dragStartHeight + (dragStartY - event.clientY)));
    panel.style.setProperty("--inspector-height", `${next}px`);
    try {
      localStorage.setItem("olp.inspector.height", String(next));
    } catch {
      // Storage blocked: the height lasts the session.
    }
  };
  const onUp = (): void => {
    window.removeEventListener("pointermove", onMove);
    window.removeEventListener("pointerup", onUp);
  };
  resizer.addEventListener("pointerdown", (event) => {
    dragStartY = event.clientY;
    dragStartHeight = query<HTMLElement>("#inspector-host").getBoundingClientRect().height;
    window.addEventListener("pointermove", onMove);
    window.addEventListener("pointerup", onUp);
    event.preventDefault();
  });
  try {
    const stored = Number(localStorage.getItem("olp.inspector.height"));
    if (Number.isFinite(stored) && stored > 0) panel.style.setProperty("--inspector-height", `${stored}px`);
  } catch {
    // Storage blocked: the default height stands.
  }
}
inspector.panes.program.append(query<HTMLElement>('[data-setup-item="project.name"]'));
for (const section of ["robot", "planner"]) {
  const el = query<HTMLElement>(`[data-setup-section="${section}"]`);
  el.hidden = false;
  inspector.panes.workspace.append(el);
}

// An update in progress (the server's maintenance notice) covers the whole
// app. Up before the first await below: boot waits on the server, and the
// notice must not wait for boot.
const maintenanceOverlay = new MaintenanceOverlay({
  onReload: () => window.location.reload(),
  onDismissReload: () => maintenanceWatch.dismissReload(),
}).mount(document.body);
// Another controller using the robot: a non-blocking card at the bottom, fed
// by the same poll (its `control`) and by refusals. The title bar's STOP stays
// above it; its own Stop is the same densePanel.stop().
controlPopup = new ControlPopup({
  onWait: () => { controlTracker.wait(Date.now()); renderControl(); },
  onExpand: () => { controlTracker.expand(); renderControl(); },
  onStop: () => densePanel.stop(),
  onTake: () => void takeControl(),
}).mount(document.body);
// The Cartesian pad's live path-hold line (execution/jogLive.ts).
jogLiveReadout = new JogLiveReadout().mount(query<HTMLElement>("#jog-live-host"));
jogEndToast = new JogEndToast({
  onDismiss: () => { jogEndTracker.dismiss(); renderJogEnd(); },
  onDetails: () => { jogEndTracker.toggleDetails(Date.now()); renderJogEnd(); },
}).mount(document.body);
pressNoticeToast = new JogEndToast({
  onDismiss: () => { pressNoticeTracker.dismiss(); renderPressNotice(); },
  onDetails: () => { pressNoticeTracker.toggleDetails(Date.now()); renderPressNotice(); },
}).mount(document.body);
pressNoticeToast.el.classList.add("press-notice-toast");
async function takeControl(): Promise<void> {
  if (!controlPopup || controlPopup.taking) return;
  controlPopup.taking = true;
  try {
    const response = await fetch(CONTROL_TAKE_API, { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" });
    if (response.ok) controlTracker.took(await response.json(), Date.now());
  } catch {
    // The next poll shows the state as it is.
  } finally {
    controlPopup.taking = false;
    renderControl();
  }
}
const maintenanceWatch = new MaintenanceWatch({
  render: (display, sessions) => maintenanceOverlay.render(display, sessions),
  onBody: (body) => { controlTracker.observe(body, Date.now()); renderControl(); },
}).start();
bootProgress();
// WHICH DESCRIPTION EACH ROBOT HAS IS THE SERVER'S ANSWER, and it has to
// arrive before the first project does: a program pins the description it
// was authored against, and drift is judged against what the server holds
// now (docs/ROBOT-DESCRIPTION-CONTRACT.md). Read once, here, and kept in
// `domain/robots.ts` for the session.
let robotCatalogueError: string | null = null;

/** The picker lists the public arms in markup and every other arm the
 *  catalogue describes here, so no robot id is written into the page. */
function addCatalogueArmOptions(): void {
  const select = query<HTMLSelectElement>("#robot-model");
  const listed = new Set([...select.options].map(option => option.value));
  for (const model of armModels()) {
    if (!listed.has(model)) select.add(new Option(robotModelFor(model).displayName, model));
  }
}
try {
  setRobotCatalogue(await fetchRobotCatalogue());
  addCatalogueArmOptions();
  if (robotCatalogue().length === 0) {
    robotCatalogueError = "the server lists no robot descriptions";
  }
} catch (error) {
  robotCatalogueError = error instanceof Error ? error.message : String(error);
}
const repository = new IndexedDbProjectRepository();
await repository.init();
bootProgress();
const autosave = new AutosaveController(repository, 450, (error) => {
  query("#save-state").textContent = "ERROR";
  query("#storage-status").textContent = `Autosave blocked: ${message(error)}`;
  storageLogger.error(`Autosave blocked: ${message(error)}`, error);
}, () => {
  query("#save-state").textContent = repository.usingMemoryFallback ? "MEMORY" : "LOCAL";
});
const history = new ProjectHistory();
const cadClient = new HttpCadClient();
const planner = new LocalPlannerAdapter();
const connectedClient = new ConnectedTargetsClient();
const targetPlanner = new TargetPlanningClient();
const executionGatewayClient = new ExecutionGatewayClient();
const programCatalogClient = new ProgramCatalogClient();
let project = await loadInitialProject();
bootProgress();
let targetState: TargetState = initialTargetState(Date.now());
let localStatus: LocalSimulatorStatus | null = null;
let localStatusError: string | null = null;
let localPlanResult: LocalPlanResult | null = null;
let localPlannerError: string | null = null;
let connectedTargets: ConnectedTargetDiscoveryDescriptor[] = [];
let connectedStatus: ConnectedReadonlyStatus | null = null;
let connectedCapabilities: ConnectedReadonlyCapabilities | null = null;
let connectedDiscoveryError: string | null = null;
let executionGatewayCapabilities: GatewayCapabilityReport =
  unavailableGatewayCapabilities(Date.now());
let executionGatewayError: string | null = null;
let serverExecutionReview: GatewayReviewResult | null = null;
let serverExecutionReviewWire: unknown = null;
let activeExecution: GatewayExecutionSession | null = null;
/**
 * The joint-daemon controls. The panel owns its DOM, its status poll and
 * the heartbeat; the host lends it the project, the node highlight and the
 * status line, and asks it whether editing is locked.
 */
const denseTelemetryClient = new DenseExecutionClient();
const densePanel = new DensePanel({
  client: new DenseExecutionClient(),
  confirmRecover: confirmation => confirmRecoverDialog(confirmation),
  readTrajectory: async digest => (await repository.getTrajectory(digest))?.bytes ?? null,
  cells: new CellTargetsClient(),
  // The connected machine decides the robot: the program is written against
  // the cell's model and axis map, and the viewer can only follow a machine
  // whose model it has loaded (a pose for another model is rejected on
  // identity and the arm sits at home). This handler was lost in the main
  // merge (676431bb); the LA bench found it on 2026-09-19.
  onCellModels: (models, selected) => {
    const select = query<HTMLSelectElement>("#robot-model");
    const note = benchModelNote(), wasBench = !note.hidden;
    renderCellModel(select, note, ["#workstation-canvas", "#orbit-pane", "#j6-pane", "#tcp-pane", "#camera-column-resizer", "#camera-row-resizer"].map(id => query(id)), selected);
    if (wasBench || !note.hidden) setViewerStatus(note.hidden ? `Canonical ${selected} · home/last-known offline pose` : note.textContent ?? "");
    for (const option of select.options) option.disabled = !models.includes(option.value as typeof selected);
    if (robotModelClass(selected) !== "arm") return;
    if (project.robot.model !== selected) {
      const from = project.robot.model;
      select.value = selected;
      select.dispatchEvent(new Event("change"));
      if (project.robot.model === selected) denseLogger.info(`Program robot switched from ${from} to ${selected}, the connected machine's`);
      else denseLogger.warn(`Program robot stays ${from} while the machine is ${selected}: ${editingLockReason() ?? "the switch was refused"}; the viewer cannot follow the machine`);
    }
  },
  onFlowChange: () => { renderSessionStatusPills(); renderLivePoseCaption(); syncSpreadsheetCartesianJog(); },
  getProject: () => project,
  highlightNode: (nodeId) => {
    executedHighlightNodeId = nodeId;
    applyPlaybackNodeHighlight(true);
  },
  refusal: text => denseLogger.info(text),
  status: (text, isError) => {
    query("#cad-status").textContent = text;
    denseLogger.at(isError ? "info" : levelOfStatus(text), text);
  },
  onLockChange: () => {
    syncNodeToolbar();
    renderAuthoringMode();
  },
  onTargetChange: () => {
    // Connecting defaults to live feedback, regardless of the remembered dock
    // panel. With no cell, setPoseSource falls back to the offline preview.
    setPoseSource("real", "auto");
    syncConnectedRobot();
    render();
  },
  onStatus: () => syncConnectedRobot(),
  onRobotPose: (jointsRad) => {
    applyRealPose(jointsRad);
    renderPoseSourceToggle();
  },
  onRobotMotion: () => setRunPanel("deploy", "auto"),
  wantsFastPoll: () => poseSource === "real",
  onTelemetry: (status, capabilities) => {
    const sample = sampleFromStatus(status, capabilities);
    if (sample) telemetryStrip().push(sample);
  },
  onJogEnd: (lastJogEnd) => {
    if (jogEndTracker.observe(lastJogEnd, Date.now())) renderJogEnd();
  },
  onJogLive: (jogLive) => jogLiveReadout?.render(jogLiveView(normalizeJogLive(jogLive))),
  onPressNotice: (refusal) => {
    if (refusal) pressNoticeTracker.show(refusal, Date.now());
    else pressNoticeTracker.dismiss();
    renderPressNotice();
  },
});

/**
 * What "the current description" means right now: the connected machine's
 * while one is connected -- the plan is made against ITS description
 * (docs/PLANNING-CONTEXT.md) -- and the server's own otherwise. Drift and
 * the typed adopt read this, so a program authored against the
 * workstation's revision is told about the machine's, not the reverse.
 */
function syncConnectedRobot(): void {
  const settings = densePanel.cellSettings();
  const robot = settings?.robotModelId && settings.robotDescriptionSha256
    ? { modelId: settings.robotModelId, robotDescriptionSha256: settings.robotDescriptionSha256 }
    : null;
  const before = connectedRobot();
  setConnectedRobot(robot);
  if (before?.robotDescriptionSha256 !== robot?.robotDescriptionSha256 || before?.modelId !== robot?.modelId) {
    renderCellSettings();
  }
}

/** The drives' motion over time beside the Jog controls; built when the panel
 *  is first shown, fed from the first sample. */
let telemetryStripInstance: TelemetryStrip | null = null;
let executionPathPanel: ExecutionPathPanel | null = null;
let executionTraceGeneration = 0;

async function showExecutionCapture(capture: ExecutionCapture): Promise<void> {
  const generation = ++executionTraceGeneration;
  viewer.setExecutionPaths([], []);
  executionPathPanel?.clear();
  const snapshot = project;
  if (capture.modelId !== snapshot.robot.model || !capture.robotDescriptionSha256 ||
      capture.robotDescriptionSha256 !== snapshot.robot.description?.robotDescriptionSha256) {
    throw new Error("Open a workspace with the trace's robot and robot-description revision before comparing TCP paths");
  }
  const samples = executionPathSamples(capture.window, capture.capabilities, capture.modelId, robotModelFor(capture.modelId).urdfJointNames);
  const isCurrent = () => generation === executionTraceGeneration && snapshot === project;
  await viewerProjectReady;
  if (!isCurrent()) return;
  const actual = await viewer.tracePlannedPath(samples.actual, isCurrent);
  const commanded = await viewer.tracePlannedPath(samples.commanded, isCurrent);
  if (!isCurrent()) return;
  const errors = executionTcpErrors(samples, actual, commanded);
  viewer.setExecutionPaths(actual, commanded);
  executionPathPanel?.show(capture, errors, samples.omitted);
}

function telemetryStrip(): TelemetryStrip {
  telemetryStripInstance ??= new TelemetryStrip(query("#telemetry-strip"), {
    axisLabels: () => densePanel.axisIds(),
    // The newest seconds of the machine's 1 kHz ring, read by the server from
    // the cell; preserve every cycle so command/feedback pairs stay adjacent.
    capture: async (seconds, signal) => {
      executionTraceGeneration++;
      viewer.setExecutionPaths([], []);
      executionPathPanel?.clear();
      const stride = 1;
      const robot = connectedRobot();
      const capabilities = densePanel.capabilities();
      const window = await denseTelemetryClient.telemetryWindow(seconds, stride, signal);
      const samples = samplesFromWindow(window, densePanel.capabilities());
      const note = `1 kHz capture · ${window.records.length} records over ${(window.records.length ? ((window.records.at(-1)!.t_ns - window.records[0]!.t_ns) / 1e9) : 0).toFixed(2)} s · sequences ${window.first_sequence}–${window.last_sequence} · fetched in ${(window.fetch_ms / 1000).toFixed(1)} s`;
      denseLogger.info(note);
      if (robot && capabilities && !signal.aborted) {
        try {
          if (connectedRobot()?.robotDescriptionSha256 !== robot.robotDescriptionSha256 ||
              densePanel.capabilities()?.rtCore?.configurationSHA256 !== capabilities.rtCore?.configurationSHA256) {
            throw new Error("Connected robot changed during capture; capture again");
          }
          await showExecutionCapture({ schema: "olp-execution-capture-v1", modelId: robot.modelId,
            robotDescriptionSha256: robot.robotDescriptionSha256, capturedAt: new Date().toISOString(), capabilities, window });
        } catch (error) { executionPathPanel?.message(`TCP comparison unavailable: ${message(error)}`); }
      }
      return { samples, note };
    },
  });
  executionPathPanel ??= new ExecutionPathPanel(query("#telemetry-strip"), showExecutionCapture);
  return telemetryStripInstance;
}

/** Joints or Cartesian: one at a time in the Jog column, remembered per browser. */
type JogMode = "joints" | "cartesian";
let jogMode: JogMode = readJogMode();
function readJogMode(): JogMode {
  try { return localStorage.getItem("olp.jog.mode") === "cartesian" ? "cartesian" : "joints"; } catch { return "joints"; }
}
function applyJogMode(mode: JogMode): void {
  // Leaving Cartesian ends any hold before the pad disappears.
  if (jogMode === "cartesian" && mode !== "cartesian") { stopJogHold("mode_switch"); densePanel.endJogHolds("mode_switch"); }
  jogMode = mode;
  query("#jog-panel").dataset.jogMode = mode;
  // Its own class: the dock's run-switch repaint walks every `.run-switch-tab`
  // and would clear a mode tab it does not know.
  for (const tab of document.querySelectorAll<HTMLButtonElement>(".jog-mode-tab")) {
    const active = tab.dataset.jogMode === mode;
    tab.classList.toggle("is-active", active);
    tab.setAttribute("aria-selected", String(active));
  }
  try { localStorage.setItem("olp.jog.mode", mode); } catch { /* the choice lasts the session */ }
}

/**
 * Which robot the viewer shows. "sim" is the browser's own pose (Tesseract
 * playback, the simulated jog); "real" is the joint daemon's servo feedback.
 * Connecting defaults to live feedback, independently of the remembered dock
 * panel. Choosing Simulate shows the preview; choosing Deploy shows the robot
 * (when a cell is selected; otherwise the preview stands in).
 * Browser playback brings Simulate up, a play or go-home on the robot
 * brings Deploy up, and while either is moving the switch is locked, so
 * the viewer never shows one robot while the other one moves. Session
 * state, not project state; the panel is remembered per browser.
 */
type RunPanel = "simulate" | "deploy";
const RUN_PANEL_STORAGE_KEY = "olp.dock.panel";
let runPanel: RunPanel = readRunPanel();
let poseSource: "sim" | "real" = "sim";
let lastRealPose: RobotPoseSnapshot | null = null;

function readRunPanel(): RunPanel {
  try {
    return localStorage.getItem(RUN_PANEL_STORAGE_KEY) === "deploy" ? "deploy" : "simulate";
  } catch {
    return "simulate";
  }
}

/** Bring a panel up. `auto` is the app's doing (a playback started); the
 *  operator's click is the default. */
function setRunPanel(panel: RunPanel, why: "auto" | "operator" = "operator"): void {
  if (runPanel !== panel) {
    runPanel = panel;
    try {
      localStorage.setItem(RUN_PANEL_STORAGE_KEY, panel);
    } catch {
      // Storage blocked: the panel resets on reload.
    }
  }
  renderRunPanel();
  setPoseSource(panel === "deploy" ? "real" : "sim", why);
}

function renderRunPanel(): void {
  for (const button of document.querySelectorAll<HTMLButtonElement>(".run-switch-tab")) {
    const active = button.dataset.runPanel === runPanel;
    button.classList.toggle("is-active", active);
    button.setAttribute("aria-selected", String(active));
  }
  for (const panel of document.querySelectorAll<HTMLElement>(".run-panel")) {
    panel.hidden = panel.dataset.runPanel !== runPanel;
  }
}

/** The note a bench model (no arm to draw) is described in, beside the viewer
 *  status line; created on first use because the merged layout has none. */
function benchModelNote(): HTMLElement {
  let note = document.querySelector<HTMLElement>("#bench-model-note");
  if (!note) {
    note = document.createElement("p");
    note.id = "bench-model-note";
    note.className = "bench-model-note";
    note.hidden = true;
    query("#viewer-status").insertAdjacentElement("afterend", note);
  }
  return note;
}

function setPoseSource(next: "sim" | "real", why: "auto" | "operator" = "operator"): void {
  if (next === "real" && !densePanel.targetSelected()) next = "sim";
  const changed = poseSource !== next;
  poseSource = next;
  syncNodeToolbar();
  applyPlaybackNodeHighlight(false);
  renderPoseSourceToggle();
  if (!changed) return;
  if (next === "real") {
    if (lastRealPose) applyRealPose(lastRealPose);
    renderLivePoseCaption();
  } else {
    // Asked for by the operator: show the jogged robot again on the next poll,
    // even if it has not moved since the feed last wrote it.
    if (why === "operator") jogFeed.forget();
    setViewerStatus(densePanel.targetSelected() ? "Pose: SIM · browser preview" : `Canonical ${project.robot.model} · home/last-known offline pose`);
    renderLivePoseCaption();
  }
}

/** The daemon's feedback as a viewer pose. Axes the cell does not map or has
 *  not homed keep the joint's current viewer value: a wrapped ±π from an
 *  unhomed encoder is not a position. */
function applyRealPose(pose: RobotPoseSnapshot): void {
  lastRealPose = pose;
  if (poseSource !== "real") return;
  viewer.setPose(pose, true);
  renderLivePoseCaption();
}

/** Native freshness controls both live captions; preview cannot imply live feedback. */
function renderLivePoseCaption(): void {
  const live = densePanel.livePoseView();
  const connected = densePanel.targetSelected();
  if (connected && poseSource === "real" && live.caption) {
    // The joint angles beside the LIVE label: the model and the numbers are
    // read together, so a model that does not follow is seen at once.
    const joints = live.jointCaption ? " · " + live.jointCaption : "";
    const mismatch = lastRealPose && lastRealPose.modelId !== project.robot.model
      ? ` · the viewer shows the program's ${project.robot.model} at home; the machine is a ${lastRealPose.modelId}. Switch the program's robot to follow it`
      : "";
    query("#viewer-status").textContent = live.caption + joints + mismatch;
  }
  query(".preview-watermark").textContent = connected && poseSource === "real"
    ? "LIVE · " + densePanel.activeCell() : "PREVIEW · NO MOTION OUTPUT";
}

/** The dock's switch follows the pose: locked while either robot moves,
 *  so the viewer cannot be flipped away from the one in motion. */
function renderPoseSourceToggle(): void {
  const moving = playing || railsPlaying || densePanel.robotMoving();
  for (const button of document.querySelectorAll<HTMLButtonElement>(".run-switch-tab")) {
    const active = button.dataset.runPanel === runPanel;
    button.disabled = moving && !active;
    button.title = moving && !active
      ? "Locked while a robot is moving"
      : button.dataset.runPanel === "deploy"
        ? "The robot: the cell's servo feedback, ARM, load, play"
        : "The browser preview: plan state and playback";
  }
  renderRunPanel();
}

/** The STEP a project was cut from, for a session that did not import it.
 *
 *  The example is a fixture the worker knows by name; anything else was
 *  imported once and kept under its fingerprint. A miss is not an error --
 *  a project saved before this was kept has nothing to find -- and rails says
 *  what to do about it when it needs the bytes. */
async function restoreCADSource(
  restored: OfflineProgrammingProject,
): Promise<CadSTEPSource | null> {
  if (restored.cad.fingerprint === "example:dog-box-v1") return DOG_BOX_FIXTURE;
  try {
    const stored = await repository.getSource(restored.cad.fingerprint);
    if (stored) return { kind: "bytes", filename: stored.filename, bytes: stored.bytes };
  } catch (error) {
    console.warn("reading the kept STEP failed:", error);
  }
  return null;
}

/** Why editing is refused right now, or null. Two locks share it: the
 *  reviewed v1 execution and a live joint-daemon session on this program. */
function editingLockReason(): string | null {
  if (activeExecution) return "Editing is locked while the reviewed robot execution is active.";
  const dense = densePanel.lockReason();
  return dense ? `Editing is locked while ${dense}.` : null;
}
let executionOperationInFlight = false;
let programCatalog: RobotV4ProgramCatalog | null = null;
let programCatalogError: string | null = null;
let programCatalogInFlight = false;
let showAdvancedProgramFields = false;
let edgeSelectionMode = false;
let weldDraft: WeldDraftUIState | null = null;
/** Cache key of the world-space edge overlays last pushed to the viewer. */
let edgeOverlaySyncKey = "";
const disclosedMoveIDs = new Set<string>();
let showPathLabels = true;
let showTcpFrame = true;
let targetOperationInFlight = false;
let offlineSimulationInFlight = false;
let stepImportInFlight = false;
let importingPartFilename: string | null = null;
let importingWorkpieceId: string | null = null;
let stepImportKind: "weld" | "fixture" = "weld";
let importingKind: "weld" | "fixture" = "weld";
let workpieceSwitchInFlight = false;
let railsStartup: Promise<void> = Promise.resolve();
let j6CamVisible = true;
let tcpCamVisible = true;
// The kept STEP comes back with the project, not only for the example part:
// without it a reloaded program draws its pickable edges and no surface (the
// part is an outline), and rails reports "no copy of the part's STEP" while
// the bytes sit in the sources store under the project's own fingerprint.
let activeCADSource: CadSTEPSource | null = await restoreCADSource(project);
/** The placement the Move part tool opened with; ✕ puts it back. */
let placementBeforeMove: WorkpiecePlacement | null = null;
/** The one selection (see setSelection); declared here because the first
 *  render pass draws the tree and inspector from it. */
let selection: Selection = { kind: "program" };
let selectionSyncing = false;
/** The bundled demo parts (examples/manifest.json), read at boot. Declared
 *  here because the tree marks demo parts by these names on every render. */
interface StepExample {
  id: string;
  filename: string;
  label: string;
  description: string;
  bytes: number;
}

let stepExamples: StepExample[] = [];
/** The kept STEPs, for the Import menu (see refreshRecentParts). */
let recentParts: Array<{ fingerprint: string; filename: string; storedAt: string }> = [];

/** The Move part tool's dialog; shown while the placement triad is on.
 *  Declared with the other view state because the first render pass, which
 *  runs before the toolbar is built, already reports into it. */
const planPresetsCard = new PlanPresetsCard(() => setPlanPresetsOpen(false))
  .mount(query<HTMLElement>("#placement-card-host"));
const weldPresetsCard = new WeldPresetsCard(() => setWeldPresetsOpen(false))
  .mount(query<HTMLElement>("#placement-card-host"));
/** Cells other OLP servers advertise on this server's network
 *  (discovered-cells). Searched while the Cells dialog shows them and when
 *  the Cells menu opens; it never connects, selects or arms. */
const cellDiscovery = new DiscoveryModel();
/** The Cells dialog: the cell this session drives, its settings pulled
 *  into the program, and the registry. A modal over the page. */
const robotDialog = new RobotDialog({
  onClose: () => cellDiscovery.stopPolling(),
  onSelect: (id) => densePanel.selectTarget(id),
  onRefresh: () => void densePanel.refreshCells(),
  onRemove: (id) => void densePanel.removeCell(id),
  onAdd: async (input) => { const added = await densePanel.addCell(input); robotDialog.setAddError(densePanel.lastCellError()); return added; },
  onPull: () => pullCellSettings(),
  onNetworkSearch: (force) => void cellDiscovery.refresh(force),
  onNetworkSectionOpen: (open) => { if (open) cellDiscovery.startPolling(); else cellDiscovery.stopPolling(); },
}).mount(document.body);
cellDiscovery.onChange(() => { if (robotDialog.shown) renderDiscovery(); });
weldPresetsCard.adopt(setupSections);
const railsWeldsCard = new RailsWeldsCard({
  onAccept: () => setRailsWeldsOpen(false),
  onCancel: cancelRailsWelds,
  onCentreCamera: () => viewer.framePart(),
  onTransparency: (opacity) => viewer.setPartDim(opacity),
}).mount(query<HTMLElement>("#placement-card-host"));
/** The program and the rails session as the Rails welds card found them;
 *  ✕ puts both back. */
let railsWeldsOpenedWith: { project: OfflineProgrammingProject; rails: RailsSnapshot } | null = null;
/** Waypoint welds: torch poses placed by hand, each a manual_pose_wp weld
 *  node of the program. The session is the tool's view of those nodes; its
 *  edits are program edits. */
const waypointSession = new WaypointSession({
  nodes: () => project.nodes,
  editNodes: (message, change) => {
    const before = project;
    mutate("nodes", (draft) => {
      draft.nodes = change(draft.nodes);
    });
    // A locked program refuses the edit and has already said why.
    if (project !== before) setStorageStatus(message);
  },
  newWeldProcess: () => {
    const process = project.processParameters[0];
    const torch = processTorchDefaults(process);
    return {
      travelSpeedMmS: (process?.travelSpeedMps ?? DEFAULT_TRAVEL_SPEED_MPS) * 1000,
      weave: torch.pattern === "none"
        ? { shape: "none" }
        : {
          shape: torch.pattern,
          amplitude_mm: { kind: "constant", value: torch.amplitudeMm },
          frequency_hz: { kind: "constant", value: torch.frequencyHz },
        },
    };
  },
});
const waypointOverlay = new WaypointOverlay();
const waypointWeldsCard = new WaypointWeldsCard(waypointSession, {
  onAccept: () => setWaypointWeldsOpen(false),
  onCancel: cancelWaypointWelds,
  onCentreCamera: () => viewer.framePart(),
  onTransparency: (opacity) => viewer.setPartDim(opacity),
  firstWaypointAt: () => viewer.partTopCentre() ?? [0, 0, 0],
  defaultStandoffMm: () => project.processParameters[0]?.torchStandoffMm ?? DEFAULT_WAYPOINT_STANDOFF_MM,
}).mount(query<HTMLElement>("#placement-card-host"));
/** How often the path follows a held waypoint handle: 10 Hz. The path is a
 *  fresh fat line per redraw; at pointer rate that is a geometry upload per
 *  mouse event for a line nobody reads mid-flick. The pose itself is kept
 *  once, on release. */
const WAYPOINT_PREVIEW_INTERVAL_MS = 100;
let waypointPreviewAtMs = Number.NEGATIVE_INFINITY;
/** The selected weld, lit in the viewport; `weldHighlighted` is the node
 *  it was drawn from, so a render that did not change it leaves it (and its
 *  blink) alone. */
const weldHighlight = new WeldHighlight();
let weldHighlighted: ProgramV2Node | null = null;
/** The program as the Waypoint welds card found it; ✕ puts it back. */
let waypointWeldsOpenedWith: OfflineProgrammingProject | null = null;
/** Programs whose browser-kept waypoints (from before they were nodes) have
 *  been looked for, so the look happens once per program per session. */
const storedWaypointsChecked = new Set<string>();
const planProgress = new ProgressDialog().mount(document.body);
const logPanel = new LogPanel(appLog).mount(document.body);
new LogIndicator(appLog, () => logPanel.toggle()).mount(query<HTMLElement>("#log-status-host"));
query<HTMLElement>(".statusbar").addEventListener("click", () => logPanel.toggle());
const placementCard = new PlacementCard({
  onChange: (field, value) => updateWorkpiecePlacementField(field, value),
  onCentre: centreCurrentWorkpiece,
  onAccept: () => setPartGizmo(false),
  onCancel: cancelPartMove,
}).mount(query<HTMLElement>("#placement-card-host"));

/** The Stand card: which fixture it edits, and the fixture as it was when the
 *  card opened, for ✕. */
let standEditing: string | null = null;
let standBeforeEdit: WorkcellFixture | null = null;
const standCard = new StandCard({
  onChange: (field, value) => updateStandField(field, value),
  onReplaceWithStep: () => replaceStandWithStep(),
  onCellDefault: () => resetStandToCellDefault(),
  onRemove: () => removeStand(),
  onAccept: () => closeStandCard(false),
  onCancel: () => closeStandCard(true),
}).mount(query<HTMLElement>("#placement-card-host"));
/** The STEP picker for a stand; the part's own picker imports a part. */
const standStepInput = document.createElement("input");
standStepInput.type = "file";
standStepInput.accept = ".step,.stp";
standStepInput.hidden = true;
standStepInput.setAttribute("aria-label", "STEP file for the stand");
document.body.append(standStepInput);
standStepInput.addEventListener("change", () => void importStandStep());
/** Tessellated STEP fixtures by lowercase fingerprint, held like the workpiece
 *  mesh: the project holds the fingerprint, this session the triangles. */
const workcellFixtureMeshes = new Map<string, CadWorkpieceMesh>();
const workcellFixtureMeshesInFlight = new Set<string>();

/** The stage toolbar's commands. Declared once; the toolbar buttons, their
 *  tooltips and the keyboard shortcuts are views of these. */
const stageActions = {
  frame: new Action({
    id: "frame-program",
    label: "Home view",
    icon: House,
    tooltip: "Fit the view to the program, or to the part when there is no path yet",
    shortcut: "F",
  }),
  labels: new Action({
    id: "path-labels",
    label: "Labels",
    icon: Tags,
    tooltip: "Show or hide row labels on the path",
    shortcut: "L",
    checkable: true,
    checked: true,
  }),
  tcpFrame: new Action({
    id: "tcp-frame",
    label: "TCP",
    icon: Axis3d,
    tooltip: "Show or hide the TCP position marker and its axes",
    checkable: true,
    checked: true,
  }),
  tcpPath: new Action({
    id: "planned-tcp-path",
    label: "TCP path",
    icon: Spline,
    tooltip: "Show the full planned TCP route: blue travel and green welds, including approach and retract",
    checkable: true,
    checked: true,
  }),
  executedPath: new Action({ id: "executed-tcp-path", label: "Executed TCP path", icon: Spline,
    tooltip: "Show the encoder-derived and commanded paths from the captured run", checkable: true, checked: true }),
  railsWelds: new Action({
    id: "rails-welds",
    label: "Rails welds",
    icon: ListChecks,
    tooltip: "The detected seams: search their torch angles, then accept or trim each weld",
    shortcut: "W",
    checkable: true,
  }),
  waypointWelds: new Action({
    id: "waypoint-welds",
    label: "Waypoint welds",
    icon: Spline,
    tooltip: "Place torch poses by hand and choose how the path joins them: C0, C1 or C2",
    checkable: true,
  }),
  lineWelds: new Action({
    id: "line-welds",
    label: "Line welds",
    icon: Slash,
    tooltip: "A straight weld between two torch poses you place, from the end of the selected weld",
    checkable: true,
  }),
  arcWelds: new Action({
    id: "arc-welds",
    label: "Arc welds",
    icon: RotateCcw,
    tooltip: "A circular weld through three torch poses you place, from the end of the selected weld",
    checkable: true,
  }),
  weldPresets: new Action({
    id: "weld-presets",
    label: "Weld presets",
    icon: SlidersHorizontal,
    tooltip: "The process defaults a seam's approach, weld and retract rows are generated with",
    checkable: true,
  }),
  planPresets: new Action({
    id: "plan-presets",
    label: "Plan presets",
    icon: SlidersHorizontal,
    tooltip: "The angle search and fitted torch the weld planner plans with",
    checkable: true,
  }),
  plan: new Action({
    id: "plan",
    label: "Plan",
    icon: Waypoints,
    tooltip: "Plan the program on the selected planner",
    shortcut: "P",
  }),
  undo: new Action({ id: "undo", label: "Undo", icon: Undo2, tooltip: "Undo", shortcut: "Ctrl+Z" }),
  redo: new Action({ id: "redo", label: "Redo", icon: Redo2, tooltip: "Redo", shortcut: "Ctrl+Shift+Z" }),
  spreadsheet: new Action({
    id: "spreadsheet",
    label: "Spreadsheet",
    icon: Table2,
    tooltip: "Show the program's rows as a spreadsheet under the viewport",
    checkable: true,
    checked: readSpreadsheetShown(),
  }),
  movePart: new Action({
    id: "move-part",
    label: "Move part",
    icon: Move3d,
    tooltip: "Drag an arrow to move the part along one of its own axes, or an arc knob to turn it about one; the placement updates when you release",
    shortcut: "M",
    checkable: true,
  }),
  j6Cam: new Action({
    id: "j6-cam",
    label: "J6 cam",
    icon: Video,
    tooltip: "Show or hide the J6 camera pane",
    checkable: true,
    checked: true,
  }),
  tcpCam: new Action({
    id: "tcp-cam",
    label: "TCP cam",
    icon: Camera,
    tooltip: "Show or hide the TCP camera pane",
    checkable: true,
    checked: true,
  }),
};
/** The three hand-placed weld tools, which share one card. */
const weldToolActions: Record<WeldTool, Action> = {
  waypoint: stageActions.waypointWelds,
  line: stageActions.lineWelds,
  arc: stageActions.arcWelds,
};

/**
 * Tessellated workpiece surfaces by STEP fingerprint. Deliberately module state
 * rather than project state: a production part is megabytes of triangles and the
 * project is autosaved to IndexedDB on every edit. Bounded to a couple of parts
 * because each entry holds its own typed arrays.
 */
const workpieceMeshes = new Map<string, CadWorkpieceMesh>();
const MAX_CACHED_WORKPIECE_MESHES = 3;
/** Fingerprints already tried and known to have no mesh — do not re-request. */
const workpieceMeshMisses = new Set<string>();
let workpieceMeshInFlight: string | null = null;
let playing = false;
/** Whether the rails dense replay is playing. A flag kept from the player's
 *  onChange rather than `railsSimulation.playing` read where it is needed:
 *  several of those readers can run before that const is initialised. */
let railsPlaying = false;
let playStartedAt = 0;
let playStartCursor = 0;
let playbackFrame = 0;
let playbackCursorS = project.simulation.cursorS;
/** Trajectory seconds per wall second for both previews. A preview
 *  convenience only: nothing planned, stored or sent to a robot reads it. */
let playbackRate: PlaybackRate = 1;
let playbackHighlightRowId: string | null = null;
let previewHighlightNodeId: string | null = null;
let executedHighlightNodeId: string | null = null;
/** When set, playback is a single-move step along the accepted trajectory. */
let stepPlayback: {
  direction: PlaybackStepDirection;
  targetTimeS: number;
} | null = null;
let jointMovePlanning = false;
let viewerMotion: {
  kind: ViewerJointMovePurpose;
  label: string;
  project: OfflineProgrammingProject;
  trajectory: PlanTrajectorySample[];
  startedAtMs: number;
  onComplete: () => void;
} | null = null;

const viewer = new WorkstationRenderer({
  elements: {
    stage: query("#workstation-stage"),
    canvas: query<HTMLCanvasElement>("#workstation-canvas"),
    orbitPane: query("#orbit-pane"),
    worldAxisCanvas: query<HTMLCanvasElement>("#world-axis-indicator"),
    j6Pane: query("#j6-pane"),
    tcpPane: query("#tcp-pane"),
  },
  initialModelId: project.robot.model,
  splitView: true,
  onEvent: (event) => {
    if (event.type === "model-loading") {
      setViewerStatus(`Loading canonical ${event.modelId} assets…`);
    } else if (event.type === "model-loaded") {
      const suffix = event.assetErrors.length > 0 ? ` · ${event.assetErrors.length} mesh warnings` : "";
      setViewerStatus(`Canonical ${event.modelId} · home/last-known offline pose${suffix}`);
    } else if (event.type === "model-error") {
      setViewerStatus(`Model blocked: ${event.message}`);
    } else if (event.type === "pose-rejected") {
      // The live caption repaints on every poll, so the caption alone hides
      // a rejected pose; the log keeps it.
      const codes = event.validation.issues.map((issue) => issue.code).join(", ");
      viewerLogger.error(`Pose rejected for ${event.modelId}: ${codes}`);
      setViewerStatus(`Pose rejected: ${codes}`);
    } else if (event.type === "layout-warning") {
      setViewerStatus(`Viewport layout blocked: ${event.validation.issues.join(", ")}`);
    } else if (event.type === "workpiece-drag") {
      // A handle drag is not an edge click, however short.
      edgeSelection.suppressed = event.dragging;
    } else if (event.type === "workpiece-placed") {
      commitGizmoPlacement(event.anchorM, event.rotationXyzw);
    } else if (event.type === "waypoint-drag") {
      edgeSelection.suppressed = event.dragging;
      waypointPreviewAtMs = Number.NEGATIVE_INFINITY;
    } else if (event.type === "waypoint-moving") {
      const { weldId, waypointId: id } = waypointSession.selection;
      const now = performance.now();
      if (!id || now - waypointPreviewAtMs < WAYPOINT_PREVIEW_INTERVAL_MS) return;
      waypointPreviewAtMs = now;
      waypointOverlay.previewPaths(waypointSession.welds, weldId, {
        id,
        positionM: event.positionM,
        quaternion: event.quaternionXyzw,
      });
    } else if (event.type === "waypoint-moved") {
      const id = waypointSession.selection.waypointId;
      if (id && waypointOverlay.torchFor(id) === event.target) {
        waypointSession.setPose(id, event.positionM, event.quaternionXyzw);
      }
    }
  },
});

let viewerProjectReady = viewer.ready;
const plannedPathController = createPlannedPathController({
  readTrajectory: async digest => (await repository.getTrajectory(digest))?.bytes ?? null,
  clear: () => viewer.setPlannedPath([]),
  show: async (snapshot, dense, isCurrent) => {
    let ready = viewerProjectReady;
    let outcome: Awaited<typeof viewerProjectReady> | undefined;
    for (;;) {
      try { outcome = await ready; }
      catch (error) {
        if (!isCurrent()) return;
        if (ready === viewerProjectReady) throw error;
      }
      if (!isCurrent()) return;
      if (ready === viewerProjectReady) break;
      ready = viewerProjectReady;
    }
    if (!outcome?.ok || viewer.modelId !== snapshot.robot.model) {
      throw new Error("The plan's robot geometry could not be loaded");
    }
    const segments = dense
      ? await viewer.tracePlannedPath(densePlannedPathRuns(snapshot, dense), isCurrent)
      : [projectPlannedPath(snapshot)];
    if (isCurrent()) viewer.setPlannedPath(segments);
  },
  onError: error => {
    const detail = `TCP path unavailable: ${message(error)}. Plan again to rebuild it.`;
    viewerLogger.warn(detail);
    setViewerStatus(detail);
  },
});

const localSimulation = new LocalSimulationAdapter(planner, {
  load: (trajectory) => {
    playbackCursorS = 0;
    updateTimeline(trajectory.at(-1)?.timeS ?? 0);
  },
  stop: () => stopPlayback(),
});

/** The trajectory digest Simulate currently holds, so a project switch
 *  reloads and an edit does not. */
let railsTrajectoryLoaded: string | null = null;

/** The same preview sample the viewer shows, or null when Simulate stops.
 *  Jog and the strip never read the viewer or each other. */
function publishPreviewSample(sample: SimulationSample | null): void {
  densePanel.previewSample(sample);
  telemetryStrip().previewSample(sample);
}

function stopRailsPreview(): void {
  railsSimulation.stop();
  previewHighlightNodeId = null;
  applyPlaybackNodeHighlight(false);
  publishPreviewSample(null);
}

/** Rails Simulate: the planned dense trajectory, replayed in the viewer.
 *
 *  A rails program has no rows, so none of the classic playback above
 *  applies to it -- `hasExactProgramPlayback()` asks for an accepted
 *  Tesseract plan and a rails program will never have one. The toolbar's
 *  transport routes here instead while `project.railsAuthoring` is set. */
const railsSimulation = createRailsSimulation({
  show: (sample) => {
    // Planning/edits also load and rewind this player. Only preview owns
    // these poses; the live view must remain on measured feedback.
    if (poseSource === "sim") applyDenseSample(viewer, sample);
    publishPreviewSample(sample);
    const nodeId = sample?.nodeId ?? null;
    const changed = previewHighlightNodeId !== nodeId;
    previewHighlightNodeId = nodeId;
    if (changed) applyPlaybackNodeHighlight(true);
  },
  readTrajectory: async (digest) => (await repository.getTrajectory(digest))?.bytes ?? null,
  programDigest: (open) => computeProgramV2Digest(toProgramV2Document(open)),
  onChange: (snapshot) => {
    playbackCursorS = snapshot.cursorS;
    if (snapshot.phase === "idle" || snapshot.phase === "ended") {
      previewHighlightNodeId = null;
      applyPlaybackNodeHighlight(false);
    }
    const nowPlaying = snapshot.phase === "playing";
    if (nowPlaying !== railsPlaying) {
      railsPlaying = nowPlaying;
      renderPoseSourceToggle();
    }
    updateTimeline(snapshot.durationS);
    const play = query<HTMLButtonElement>("#play");
    play.textContent = snapshot.phase === "playing" ? "Ⅱ" : "▶";
    play.setAttribute(
      "aria-label",
      snapshot.phase === "playing" ? "Pause simulation" : "Play simulation",
    );
    // Stop is live whenever there is something loaded, playing or not: it
    // rewinds to the start, which is what stop means on a paused preview.
    query<HTMLButtonElement>("#stop").disabled = snapshot.phase === "idle";
  },
});

/** Load the open program's planned trajectory into Simulate.
 *
 *  Every outcome but `loaded` is reported rather than swallowed: a Simulate
 *  button that does nothing is indistinguishable from a broken one, and the
 *  three ways it can have nothing to play are three different problems. */
async function loadRailsSimulation(): Promise<RailsLoadOutcome> {
  const outcome = await railsSimulation.load(project);
  if (outcome.state === "missing") {
    setViewerStatus(
      `This program was planned, but its trajectory is not in this browser's store ` +
        `(${outcome.digest.slice(0, 12)}\u2026). Plan again to rebuild it.`,
    );
  } else if (outcome.state === "unreadable") {
    setViewerStatus(`The stored trajectory could not be read: ${outcome.reason}`);
  }
  return outcome;
}

// Pickable CAD topology edges in the orbit view (P2.12). Selection is
// additive into the weld draft; shift-click adds a program control point.
const edgeSelection = new EdgeSelectionController({
  scene: viewer.workFrameLayer,
  camera: viewer.orbitCamera,
  element: query("#orbit-pane"),
  // The ground the edge picker snaps to: this robot's work frame, not a
  // table centre written into the browser (domain/robots.ts).
  groundZ: cellWorkFrame(project.robot.model).worldM[2],
  callbacks: {
    onEdgeClick: (edgeId, pointM) => {
      // The document chooses the workflow. Rails loading or declining a
      // click must never create a draft in the older edge workflow.
      if (project.railsAuthoring) {
        railsMode.onEdgeClicked(edgeId, pointM);
        return;
      }
      weldDraft = draftWithEdge(weldDraft, edgeId);
      syncEdgeSelectionState();
      renderWeldDrawerPanel();
    },
    onScenePointClick: (pointM) => {
      if (project.railsAuthoring) {
        railsMode.onScenePointClicked(pointM);
        return;
      }
      addControlPointAtWorld(pointM);
    },
    // A plain click that missed every edge tube: in rails mode, snap to the
    // nearest seam instead of doing nothing — the tubes are 4 mm wide and
    // silence reads as a broken button.
    onEdgeMissClick: (pointM) => {
      if (pointM) void railsMode.onScenePointClicked(pointM);
    },
    // A click on the part selects it, as a click on its tree row does.
    onPartClick: () => {
      if (selection.kind !== "part") setSelection({ kind: "part" });
    },
  },
});

// Rails mode (src/rails): the weld planner's seam model behind a toggle.
// Off, nothing below runs and every flow above is exactly as it was.
const railsMode = createRailsMode({
  requiresMeasuredStart: () => densePanel.connectedCell() !== null,
  closeLoop: () => densePanel.repeatMode() !== "once",
  parent: viewer.workFrameLayer,
  presetStore: repositoryPresetStore(repository),
  getProject: () => project,
  getSource: () => activeCADSource,
  getFixtures: () => workcellFixturesWire(environmentFixtures(project), workcellFixtureMeshes),
  status: (text, isError, detail) => {
    query("#cad-status").textContent = text;
    railsLogger.at(isError ? "error" : levelOfStatus(text), text, detail);
  },
  requestPanelRender: () => renderWeldDrawerPanel(),
  applyPlan: async (result) => {
    // No rows to install. `export/robot_v4.py` compiled the planner's result
    // into `robot.v4.program.v1` rows and went with the authoring server; the
    // program is program.v2 and the motion is the planner's own document,
    // which is a better thing to hold than a flattened copy of it.
    const previous = project;
    const dense = result.dense;
    const plannedProject = { ...project };
    plannedProject.denseTrajectory = denseTrajectoryRecordOf(project, result);
    // The bytes, kept here. The motion server files them under `~/.cache` and
    // owes nobody a copy later, and the summary above names the digest they
    // are found by -- the same digest the daemon fences on, so there is no
    // second id to keep in step.
    if (dense && result.dense_blob_base64) {
      const binary = atob(result.dense_blob_base64);
      const bytes = new Uint8Array(binary.length);
      for (let index = 0; index < binary.length; index += 1) bytes[index] = binary.charCodeAt(index);
      try {
        await repository.putTrajectory({
          digest: dense.trajectory_digest,
          bytes,
          programDigest: result.program_digest,
          programId: dense.program_id,
          plannedAt: dense.created_at,
        });
      } catch (error) {
        console.warn("keeping the trajectory failed:", error);
      }
    }
    if (result.dense_error) {
      console.warn(`dense trajectory not built: ${result.dense_error.reason} ${result.dense_error.detail}`);
    }
    if (project !== previous) return;
    project = recordPlan(plannedProject);
    history.push(previous);
    changed("Program planned");
    await autosave.flush();
    render();
  },
  upsertWeldNode: (node) => upsertWeldNode(node),
  removeWeldNode: (nodeId) => {
    if (!nodeById(project.nodes, nodeId)) return;
    mutate("nodes", (draft) => {
      draft.nodes = removeNode(draft.nodes, nodeId);
    });
  },
});

// ---------------------------------------------------------------------------
// The node program: selection, the parameter dialog, and the edits the
// spreadsheet makes. Every write goes through mutate("nodes", …), so it is
// one revision, one undo step, locked during an execution, and rows are
// recompiled by deriveRows.
// ---------------------------------------------------------------------------

let selectedNodeId: string | null = null;

const nodeEditor = createNodeEditor(query<HTMLDialogElement>("#node-editor"), {
  getProject: () => project,
  plan: () => planProgram(),
  motionProfile: async (nodeId) => {
    const opened = project;
    const record = opened.denseTrajectory;
    if (!record) return null;
    const stored = await repository.getTrajectory(record.trajectoryDigest);
    if (!stored) return null;
    const decoded = decodeDenseTrajectory(stored.bytes);
    if (decoded.trajectoryDigest !== record.trajectoryDigest) throw new Error("Trajectory identity does not match the selected plan");
    const segment = decoded.segments.find(segment => segment.sourceId === nodeId);
    const profile = segment ? jointMotionProfile(segment.samples) : null;
    const node = nodeById(opened.nodes, nodeId);
    if (!profile || node?.op !== "move") return null;
    await viewerProjectReady;
    if (project !== opened) return null;
    const runs = densePlannedPathRuns(opened, { ...decoded, segments: [segment!] });
    const [path] = await viewer.tracePlannedPath(runs, () => project === opened);
    if (!path) return null;
    const tcp = tcpMotionProfile(segment!.samples, path.map(point => point.positionM));
    if (!tcp) throw new Error("Planned TCP samples are unavailable");
    const current = record.sourceProgramDigest === await computeProgramV2Digest(toProgramV2Document(opened));
    return { profile, tcp, current, planRevision: record.planRevision,
      planSpeed: current ? opened.programV2.speedScale ?? 1 : null, jointNames: node.target.joint_names };
  },
  documentProblems: (node) => {
    const nodes = project.nodes.map((candidate) => (candidate.id === node.id ? node : candidate));
    return nodes.length === 0 ? [] : programV2Problems(toProgramV2Document({ ...project, nodes }));
  },
  commit: (node, seam) => commitNode(node, seam),
  status: (text) => {
    setStorageStatus(text);
  },
});

function editNodeInline(id: string, spec: FieldSpec, value: EditValue): void {
  const node = nodeById(project.nodes, id);
  if (!node) throw new Error("The node is no longer in the program");
  const edited = applyEdit(structuredClone(node), spec, value);
  const nodes = project.nodes.map(candidate => candidate.id === id ? edited : candidate);
  const problems = programV2Problems(toProgramV2Document({ ...project, nodes }));
  if (problems.length) throw new Error(problems[0]);
  commitNode(edited, null);
}

function commitNode(node: ProgramV2Node, seam: SeamEdits | null): void {
  const lock = editingLockReason();
  if (lock) throw new Error(lock);
  if (!nodeById(project.nodes, node.id)) throw new Error("the node is no longer in the program");
  mutate("nodes", (draft) => { commitProgramNode(draft, node, seam); });
  selectNode(node.id);
  setStorageStatus(`${node.op} ${node.label || node.id} saved · rows recompiled`);
}

function upsertWeldNode(node: WeldNode): void {
  const existing = nodeById(project.nodes, node.id);
  mutate("nodes", (draft) => {
    if (existing) {
      draft.nodes = draft.nodes.map((candidate) => (candidate.id === node.id ? structuredClone(node) : candidate));
    } else {
      draft.nodes = insertWeld(draft.nodes, node);
    }
  });
  selectNode(node.id);
}

function selectNode(nodeId: string | null): void {
  selectedNodeId = nodeId;
  if (nodeId && !selectionSyncing) {
    const node = nodeById(project.nodes, nodeId);
    if (node && node.op === "weld") {
      selectionSyncing = true;
      try {
        setSelection({ kind: "weld", id: nodeId }, "sheet");
      } finally {
        selectionSyncing = false;
      }
    }
  }
  const firstRow = nodeId ? rowsOfNode(project, nodeId)[0] : undefined;
  if (firstRow) project.simulation.selectedRowId = firstRow.id;
  onRowSelectionChanged();
  applyNodeHighlight(query("#program-spreadsheet"), nodeId, "is-selected", false);
  syncNodeToolbar();
}

function syncNodeToolbar(): void {
  const node = selectedNodeId ? nodeById(project.nodes, selectedNodeId) : null;
  const locked = editingLockReason();
  const planSpeed = query<HTMLInputElement>("#plan-speed");
  planSpeed.value = String(Number(((project.programV2.speedScale ?? 1) * 100).toFixed(2)));
  planSpeed.disabled = locked !== null || !usesDensePlanner();
  planSpeed.title = locked ?? (usesDensePlanner() ? "Replan and reload after changing speed" : "Available for weld-planner and taught-move plans");
  const edit = query<HTMLButtonElement>("#edit-node");
  edit.disabled = !node || locked !== null;
  edit.title = "Edit the selected move's destination, speed and motion profile, or another row's settings."
    + (locked ? `\n${locked}` : !node ? "\nSelect a row first." : "");
  const remove = query<HTMLButtonElement>("#delete-node");
  const removeBlocked = locked ?? (node ? deleteBlocked(project.nodes, node.id) : "Select a node to delete");
  remove.disabled = removeBlocked !== null;
  remove.title = "Remove the selected row from the program. Undo restores it. This does not move the robot."
    + (removeBlocked ? `\n${removeBlocked}` : "");
  const pose = poseSource === "real" ? "connected robot's measured TCP position and orientation" : "preview robot's current TCP position and orientation";
  const tips: Record<string, string> = {
    "#record-waypoint": `Record the ${pose} as a new waypoint in the program. Does not move the robot.`,
    "#reteach-waypoint": `Replace the selected waypoint's destination with the ${pose}. Keeps its move type and speed. Does not move the robot.`,
    "#record-via": `Record the ${pose} as the selected move's intermediate arc point and change it to MoveC. Keeps the destination. Does not move the robot.`,
    "#add-dwell": "Insert a timed pause after the selected row and set its duration. The pause happens when the program runs.",
    "#add-io": "Insert an output on/off instruction and choose its channel and value. The channel must name one of the connected cell's outputs; plan with the cell connected. Plans that switch outputs load as dry runs until output execution is enabled; adding it does not switch an output.",
    "#add-home": "Append a move to the cell's configured home pose. Runs only with the program; does not move the robot now or recalibrate its joints.",
  };
  for (const [id, tip] of Object.entries(tips)) {
    const button = query<HTMLButtonElement>(id);
    const reason = locked ?? ((id === "#reteach-waypoint" || id === "#record-via") && node?.op !== "move" ? "Select a waypoint row first." : null);
    button.disabled = reason !== null;
    button.title = tip + (reason ? `\n${reason}` : "");
  }
  query<HTMLButtonElement>("#record-waypoint").textContent = poseSource === "real" ? "RECORD ROBOT WAYPOINT" : "RECORD PREVIEW WAYPOINT";
  query<HTMLButtonElement>("#toggle-advanced").title = "Switch between program rows and read-only compiled robot instructions. Return to program rows to edit.";
}

function openSelectedNodeEditor(): void {
  const nodeId = selectedNodeId ?? project.nodes[0]?.id;
  if (!nodeId) {
    setStorageStatus("No node to edit yet.");
    return;
  }
  const lock = editingLockReason();
  if (lock) {
    setStorageStatus(lock);
    return;
  }
  selectNode(nodeId);
  nodeEditor.open(nodeId);
}

async function recordWaypoint(mode: "new" | "reteach" | "via" = "new"): Promise<void> {
  try {
    const lock = editingLockReason();
    if (lock) throw new Error(lock);
    if (playing || railsPlaying) throw new Error("Stop preview playback before recording");
    const selected = project.nodes.find(node => node.id === selectedNodeId);
    if (mode !== "new" && selected?.op !== "move") throw new Error("Select a taught waypoint first");
    const measured = poseSource === "real" ? densePanel.teachObservation() : null;
    const snapshot = measured?.pose ?? viewer.currentPoseSnapshot();
    const description = captureDescription(project, measured, snapshot);
    const originalProject = project;
    const calibration = machineBoundCapture(measured) ? await densePanel.fetchCellSettings() : project.robot.cell;
    checkCaptureCalibration(measured, description, calibration);
    const response = await fetch("/api/offline-programming/v1/robots/capture-model", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ model_id: snapshot.modelId, robot_description_sha256: description,
        machine_planning_calibration_base64: calibration?.machinePlanningCalibrationBase64 ?? "" }),
    });
    const model = await response.json();
    if (!response.ok) throw new Error(model.error ?? "Could not resolve the capture model");
    if (project !== originalProject || editingLockReason()) throw new Error("Program changed during capture; record again");
    checkCaptureModel(model.identity, description, calibration, mode, selected);
    const { target, capture } = teachCapture(project, snapshot, viewer.tcpPoseFor(snapshot, model.urdf), description, model.identity, measured);
    const id = mode === "new" ? crypto.randomUUID() : selected!.id;
    mutate("nodes", draft => { recordMove(draft, mode, selectedNodeId, target, capture, id); });
    stageActions.spreadsheet.checked = true;
    applySpreadsheetShown(true);
    selectNode(id);
    setStorageStatus(`${measured ? "Measured" : "Preview"} ${mode === "via" ? "via pose" : "waypoint"} recorded`);
  } catch (error) {
    setStorageStatus(`Record blocked: ${message(error)}`);
    if (error instanceof CaptureProgramMismatch) openRecordBlockedDialog(error);
  }
}

/**
 * Says why a waypoint was not recorded when the program is not written for
 * the connected robot, and offers the remedy in place: accept the current
 * robot settings (shown here, so this is the review), or start a program for
 * this robot. The status line alone was missed on a Rosie 1430 arm (2026-10-11): a
 * program saved before J6's 50:1 -> 100:1 change refused every capture.
 */
function openRecordBlockedDialog(error: CaptureProgramMismatch): void {
  const dialog = query<HTMLDialogElement>("#record-blocked-dialog");
  const copy = query("#record-blocked-copy");
  const note = query("#record-blocked-note");
  const changes = query("#record-blocked-changes");
  const accept = query<HTMLButtonElement>("#record-blocked-accept");
  const review = query<HTMLButtonElement>("#record-blocked-review");
  const fresh = query<HTMLButtonElement>("#record-blocked-new");
  changes.replaceChildren();
  const drift = error.kind === "description" ? robotDescriptionDrift(project) : null;
  if (error.kind === "model") {
    query("#record-blocked-title").textContent = "This program is for another robot";
    copy.textContent = `${error.message}. Waypoints are recorded for one robot model; this program's waypoints are not converted.`;
    note.textContent = "New program saves this one first, then starts a program for the connected robot.";
  } else {
    query("#record-blocked-title").textContent = "This program uses older robot settings";
    copy.textContent = `This program was saved against an earlier ${project.robot.model} description. `
      + "The connected robot's settings have changed since, so waypoints can't be recorded until you accept them.";
    note.textContent = drift
      ? "Accepting keeps your parts, welds and waypoints. Plan again and review the simulation before running."
      : "The server's robot description matches this program but the robot's does not; pull the cell's settings from Cells, then try again.";
    if (drift) {
      const settings = robotSettingsReview(project);
      if (settings.rows.length) {
        const table = element("table", "robot-settings-changes");
        const head = element("tr");
        for (const label of ["Setting", "In this program", "Robot now"]) head.append(element("th", "", label));
        table.append(head);
        for (const change of settings.rows.slice(0, 6)) {
          const row = element("tr");
          row.append(element("th", "", change.label), element("td", "", change.before), element("td", "", change.after));
          table.append(row);
        }
        changes.append(table);
      }
    }
  }
  accept.hidden = !drift;
  review.hidden = error.kind === "model";
  fresh.classList.toggle("primary", !drift);
  fresh.classList.toggle("secondary", Boolean(drift));
  if (!dialog.open) dialog.showModal();
  (drift ? accept : fresh).focus();
}

function closeRecordBlockedDialog(): void {
  const dialog = query<HTMLDialogElement>("#record-blocked-dialog");
  if (dialog.open) dialog.close();
}

function addEventNode(kind: "dwell" | "io" | "home"): void {
  const lock = editingLockReason();
  if (lock) {
    setStorageStatus(lock);
    return;
  }
  const node = kind === "dwell"
    ? dwellNode(0.5)
    : kind === "io"
      ? ioNode("revpi:1", true)
      : freespaceNode("home");
  try {
    mutate("nodes", (draft) => { insertEventNode(draft, node, selectedNodeId); });
    selectNode(node.id);
    if (kind !== "home") nodeEditor.open(node.id);
  } catch (error) {
    setStorageStatus(`Add blocked: ${message(error)}`);
  }
}

function deleteNode(nodeId: string): void {
  const blocked = deleteBlocked(project.nodes, nodeId);
  if (blocked) {
    setStorageStatus(`Delete blocked: ${blocked}`);
    return;
  }
  try {
    const index = project.nodes.findIndex((candidate) => candidate.id === nodeId);
    mutate("nodes", (draft) => { deleteProgramNode(draft, nodeId); });
    selectNode(project.nodes[Math.min(index, project.nodes.length - 1)]?.id ?? null);
  } catch (error) {
    setStorageStatus(`Delete blocked: ${message(error)}`);
  }
}

function moveNodeInProgram(nodeId: string, delta: -1 | 1): void {
  const blocked = moveBlocked(project.nodes, nodeId, delta);
  if (blocked) {
    setStorageStatus(`Move blocked: ${blocked}`);
    return;
  }
  mutate("nodes", (draft) => { moveProgramNode(draft, nodeId, delta); });
  selectNode(nodeId);
}

function toggleNodeEnabled(nodeId: string, enabled: boolean): void {
  mutate("nodes", (draft) => { setNodeEnabled(draft, nodeId, enabled); });
  selectNode(nodeId);
}

// Dev-only console handle — drives rails clicks from scripted browser
// sessions, where hitting a millimetre-wide seam line by pixel is luck.
if (import.meta.env.DEV) {
  (window as unknown as { __olpDev?: unknown }).__olpDev = { viewer, railsMode };
}
/** The toolbar's weld tools follow the program's mode: Edges for classic,
 *  Rails welds for rails, each disabled with the reason in the other. */
/**
 * Waypoint, Line and Arc welds: a part to place on, nothing locking the
 * program, and a rails program whose mode is up -- the weld planner plans
 * these, and reads the workpiece frame a rails program is written in.
 * Called from both renders that own an ingredient: the rails mode comes up
 * asynchronously after the part's seams load, and a program opened before
 * that kept these grey until its next edit.
 */
function renderWeldToolActions(): void {
  const lock = editingLockReason();
  const part = hasWorkpieceGeometry(project) && workpiecePlacement(project) !== null;
  const allowed = part && lock === null && railsMode.enabled;
  const reason = lock
    ?? (!part
      ? "Import a part to place welds on"
      : project.railsAuthoring ? "Rails is loading the part's seams" : "Rails authoring only");
  for (const action of Object.values(weldToolActions)) action.setEnabled(allowed, reason);
  if (!allowed && waypointWeldsCard.shown) setWaypointWeldsOpen(false);
}

function renderAuthoringMode(): void {
  const rails = Boolean(project.railsAuthoring);
  stageActions.railsWelds.setEnabled(railsMode.enabled, rails
    ? "Rails is loading the part's seams"
    : "Rails authoring only");
  stageActions.planPresets.setEnabled(railsMode.enabled, rails
    ? "Rails is loading the part's seams"
    : "Rails authoring only");
  if (!railsMode.enabled && planPresetsCard.shown) setPlanPresetsOpen(false);
  if (!railsMode.enabled && railsWeldsCard.shown) setRailsWeldsOpen(false);
  renderWeldToolActions();
}

/** The Project switch is the one control for the mode; it also shows the
 *  runtime arming (rails needs the part's STEP at the weld planner). */
function syncRailsButton(): void {
  renderAuthoringMode();
}

/** Rails runtime follows the loaded program: a classic program turns the
 *  mode off; a rails program re-arms it as soon as its STEP is at hand. */
function syncRailsModeToProject(): void {
  // The analysed part must be this program's part. A New program, another
  // program, or a program without a part discards the previous part's seams,
  // guides, searches and overlays; disable() alone only hid them, and a rails
  // program without a part was never even disabled, so the old part's orange
  // seam guides stayed drawn in the viewport (owner, 2026-09-20).
  const analysed = railsMode.fingerprint;
  if (analysed !== null && (!hasWorkpieceGeometry(project) || analysed !== project.cad.fingerprint)) railsMode.reset();
  if (!project.railsAuthoring && railsMode.enabled) railsMode.disable();
  // A rails program with no part has nothing to detect seams on; it
  // re-arms at the next import rather than reporting a missing STEP.
  if (project.railsAuthoring && !railsMode.enabled && hasWorkpieceGeometry(project)) {
    const owner = `${project.identity.id}/${activeWorkpieceId(project)}/${project.cad.fingerprint}`;
    railsStartup = railsMode.enable().then(() => {
      if (owner !== `${project.identity.id}/${activeWorkpieceId(project)}/${project.cad.fingerprint}`) return;
      syncRailsButton();
      setRailsWeldsOpen(true);
      syncEdgeSelectionState();
      if (railsMode.enabled && targetState.mode !== "local_simulator") {
        void selectTarget("weld_planner");
      }
      updateTargetOptions();
      renderTargetAndReview();
    });
  }
  syncRailsButton();
  syncEdgeSelectionState();
  updateTargetOptions();
  renderTargetAndReview();
}

applyDefaultPlannerSelection();
render();
bootProgress();
await viewer.ready;
bootProgress();
await syncViewerProject();
// A rails program reopened from the store has its trajectory in the store
// too. Load it now, so Simulate works on a program nobody has re-planned.
await syncRailsSimulation();
registerServiceWorker();
await Promise.allSettled([
  refreshLocalSimulatorStatus(),
  discoverConnectedTargets(),
  refreshExecutionGatewayCapabilities(),
  refreshProgramCatalog(),
]);
renderTargetAndReview();
if (connectedTargets.length === 0) {
  window.setTimeout(() => void discoverConnectedTargets(), 1_000);
}

// Deep link into the catalog: `?program=<id>` opens that shared program on
// arrival — the weld planner's export receipt links here, so "Export to OLP"
// lands the operator in front of the exported rows in one click. The
// parameter is consumed (removed from the URL) so a reload afterwards does
// not re-open over whatever they navigated to since.
{
  const requested = new URLSearchParams(window.location.search).get("program");
  if (requested) {
    const url = new URL(window.location.href);
    url.searchParams.delete("program");
    window.history.replaceState(null, "", url);
    const selector = query<HTMLSelectElement>("#catalog-program");
    if ([...selector.options].some((option) => option.value === requested)) {
      selector.value = requested;
      void openSelectedCatalogProgram();
    } else {
      programCatalogError = `Deep link: catalogue has no program "${requested}"`;
      renderProgramCatalog();
    }
  }
}

setStorageStatus(repository.usingMemoryFallback
  ? "IndexedDB unavailable · protected in-memory fallback"
  : "IndexedDB project revisions active");
// The restored project may be a rails-authoring one — arm the mode the same
// way opening it any other way would.
syncRailsModeToProject();
window.__olpBootDone?.();
// A restored project keeps its CAD fingerprint, and the kept STEP under that
// fingerprint (or the registered Dog Box fixture) brings its part surface back
// after a reload. Deliberately after boot completes: the mesh must never gate
// first paint.
void loadWorkpieceMesh(activeCADSource);
void refreshRecentParts();

stageActions.undo.triggered.connect(() => {
  if (!history.canUndo) return;
  project = history.undo(project);
  changed("Undo restored as a new revision");
  void resyncPartSource();
});

/** After an undo or redo across a part's removal or import, the session's
 *  copy of the STEP follows the project: the kept bytes come back under
 *  the project's fingerprint, the surface is drawn again, and rails
 *  re-arms -- or, with no part, stands down. */
async function resyncPartSource(): Promise<void> {
  const wanted = workpieceMeshKey(project) !== "" || project.cad.fingerprint === "example:dog-box-v1";
  if (!wanted) {
    if (activeCADSource !== null) {
      activeCADSource = null;
      if (railsMode.enabled) railsMode.disable();
      syncRailsButton();
    }
    return;
  }
  {
    const owner = `${project.identity.id}/${activeWorkpieceId(project)}/${project.cad.fingerprint}`;
    const restored = await restoreCADSource(project);
    if (owner !== `${project.identity.id}/${activeWorkpieceId(project)}/${project.cad.fingerprint}`) return;
    railsMode.reset();
    activeCADSource = restored;
    void loadWorkpieceMesh(activeCADSource);
    syncRailsModeToProject();
  }
}
query("#new-program-close").addEventListener("click", closeNewProgramDialog);
query("#new-program-cancel").addEventListener("click", closeNewProgramDialog);
query<HTMLFormElement>("#new-program-form").addEventListener("submit", (event) => {
  event.preventDefault();
  const input = query<HTMLInputElement>("#new-program-name");
  const name = input.value.trim() || "Untitled robot program";
  closeNewProgramDialog();
  void createNewProgram(name);
});
query<HTMLDialogElement>("#new-program-dialog").addEventListener("click", (event) => {
  if (event.target === event.currentTarget) closeNewProgramDialog();
});
query("#record-blocked-close").addEventListener("click", closeRecordBlockedDialog);
query<HTMLDialogElement>("#record-blocked-dialog").addEventListener("click", (event) => {
  if (event.target === event.currentTarget) closeRecordBlockedDialog();
});
query("#record-blocked-review").addEventListener("click", () => {
  closeRecordBlockedDialog();
  setSelection({ kind: "workspace" });
});
query("#record-blocked-new").addEventListener("click", () => {
  closeRecordBlockedDialog();
  openNewProgramDialog();
});
query("#record-blocked-accept").addEventListener("click", () => {
  closeRecordBlockedDialog();
  adoptCurrentRobotDescription();
  setSelection({ kind: "workspace" });
});
stageActions.redo.triggered.connect(() => {
  if (!history.canRedo) return;
  project = history.redo(project);
  changed("Redo restored as a new revision");
  void resyncPartSource();
});
query("#catalog-refresh").addEventListener("click", () => void refreshProgramCatalog());
query("#catalog-open").addEventListener("click", () => void openSelectedCatalogProgram());
query("#catalog-save").addEventListener("click", () => void saveSharedProgram());
query("#edit-node").addEventListener("click", () => openSelectedNodeEditor());
query("#record-waypoint").addEventListener("click", () => recordWaypoint());
query("#reteach-waypoint").addEventListener("click", () => recordWaypoint("reteach"));
query("#record-via").addEventListener("click", () => recordWaypoint("via"));
query("#add-dwell").addEventListener("click", () => addEventNode("dwell"));
query("#add-io").addEventListener("click", () => addEventNode("io"));
query("#add-home").addEventListener("click", () => addEventNode("home"));
query("#delete-node").addEventListener("click", () => {
  if (selectedNodeId) deleteNode(selectedNodeId);
});
query("#toggle-advanced").addEventListener("click", () => {
  showAdvancedProgramFields = !showAdvancedProgramFields;
  const button = query<HTMLButtonElement>("#toggle-advanced");
  button.setAttribute("aria-pressed", String(showAdvancedProgramFields));
  button.classList.toggle("is-active", showAdvancedProgramFields);
  renderProgramSpreadsheet();
});
// `startLiveJog` retries at boot so header cartesian enables as soon as the
// isolated cartesiand is up. Jog state lives near the top of this module —
// see the TDZ note above the declarations.
void startLiveJog();
query("#drive-arm-badge").addEventListener("click", () => {
  // The pill's word is the drives' state: DISARMED arms, ARMED (or ARMING)
  // disarms. Holding control with the drives off (after Home) is not a
  // reason to release control; on the LA bench that turned the first Arm
  // press into a Disarm and the real target's confirm never appeared.
  const arm = densePanel.flowView()?.arm;
  if (arm?.state === "armed" || arm?.state === "arming") densePanel.disarm();
  else densePanel.arm();
});
query("#cell-stop-pill").addEventListener("click", () => densePanel.stop());
// The maintenance overlay's STOP: the title bar's is underneath it.
maintenanceOverlay.bindStop(() => densePanel.stop(), () => densePanel.flowView()?.connected === true);
query("#cell-clear-faults").addEventListener("click", () => densePanel.clearFaults());
query("#cell-clear-faults-dismiss").addEventListener("click", () => densePanel.dismissClearFaults());
query("#cell-arm-confirm").addEventListener("click", () => densePanel.arm());
query("#cell-arm-cancel").addEventListener("click", () => densePanel.cancelArm());
query("#sim-arm-toggle").addEventListener("click", toggleSimArm);
document.querySelectorAll<HTMLElement>("[data-process-field]").forEach((input) => {
  input.addEventListener("change", updateProcessDefaults);
});
query<HTMLInputElement>("#project-file").addEventListener("change", (event) => void importProject(event));
query("#import-step").addEventListener("click", () => {
  chooseStepFiles("weld");
});
query<HTMLInputElement>("#step-file").addEventListener("change", (event) => void importSTEP(event));
query("#open-example").addEventListener("click", () => void openStepExample());
query("#cartesian-zero").addEventListener("click", () => {
  const fraction = Number(query<HTMLSelectElement>("#cartesian-speed").value);
  void densePanel.zeroAllJoints(fraction).then(() => syncSpreadsheetCartesianJog());
  syncSpreadsheetCartesianJog();
});
query<HTMLSelectElement>("#step-example").addEventListener("change", renderStepExampleNote);
void loadStepExamples();
query("#generate-preview").addEventListener("click", () => void planProgram());
query("#target-plan").addEventListener("click", () => void planOnSelectedRobot());
query("#review-execute").addEventListener("click", () => void reviewPhysicalStart());
query("#start-execution").addEventListener("click", () => void startReviewedExecution());
query("#stop-execution").addEventListener("click", () => void stopActiveExecution());
// The toolbar transport drives whichever preview the program HAS. A rails
// program's is the planned dense trajectory; a classic program's is the
// accepted Tesseract plan. Never both -- `railsAuthoring` is fixed at New
// program, so there is no moment where two previews are loaded.
query("#play").addEventListener("click", () => {
  if (!usesDensePlanner()) {
    void startPlayback();
    return;
  }
  if (railsSimulation.playing) {
    railsSimulation.pause();
    return;
  }
  // The viewer shows the preview while it runs, not the daemon's feedback.
  setRunPanel("simulate", "auto");
  railsSimulation.play();
});
densePanel.mount(query("#dense-robot-host"), query("#dense-playback"), query<HTMLElement>("#jog-panel"));
query<HTMLElement>('.run-panel[data-run-panel="simulate"]').append(query("#sim-arm-toggle"));
// The machine's last word sits in the heading row after the mode switch,
// before the close button: one line for the dock's identity and its status.
query(".jog-panel-heading").insertBefore(query("#jog-operator-feedback"), query("#jog-panel-close"));
query("#dense-playback").append(query("#dense-operator-feedback"));
wireSpreadsheetCartesianJog();
query("#jog-panel-close").addEventListener("click", toggleJogPad);
// The target is chosen from Cells now; the select stays in the pane's
// DOM, out of sight, because the panel reads it and writes its status line.
query<HTMLElement>('[data-setup-item="robot.target"]').hidden = true;
query<HTMLElement>("#dense-target-state").hidden = true;
densePanel.changed.connect(() => {
  renderCellBadge();
  renderCellSettings();
  renderProgramTree();
  if (robotDialog.shown) renderRobotDialog();
  renderSessionStatusPills();
  renderLivePoseCaption();
});
query("#stop").addEventListener("click", () => {
  if (usesDensePlanner()) stopRailsPreview();
  else stopPlayback();
});
query<HTMLSelectElement>("#playback-rate").addEventListener("change", (event) => {
  const chosen = Number((event.currentTarget as HTMLSelectElement).value);
  const rate = PLAYBACK_RATES.find((candidate) => candidate === chosen) ?? 1;
  playbackRate = rate;
  // The dense player keeps its own clock; the classic loop re-bases here so
  // a change mid-play applies from the current cursor rather than re-timing
  // the part already shown.
  railsSimulation.setRate(rate);
  if (playing && !stepPlayback) {
    playStartCursor = playbackCursorS;
    playStartedAt = performance.now();
  }
});
query("#step-back").addEventListener("click", () => void stepProgramPlayback("backward"));
query("#step-forward").addEventListener("click", () => void stepProgramPlayback("forward"));
query("#home-robot").addEventListener("click", () => void homeRobot());
query<HTMLInputElement>("#timeline").addEventListener("input", (event) => {
  if (!usesDensePlanner()) {
    previewTimelineSelection(event);
    return;
  }
  // Scrubbing a dense trajectory is a seek, not a plan: the samples are
  // already here and the viewer poses from them directly.
  railsSimulation.seekFraction(Number((event.currentTarget as HTMLInputElement).value) / 1000);
});
query<HTMLInputElement>("#timeline").addEventListener("change", (event) => {
  if (!usesDensePlanner()) void seekPlayback(event);
});
bindProgramNameInput(query<HTMLInputElement>("#project-name"), renameProgram);

function renameProgram(raw: string): void {
  const name = raw.trim();
  if (!name || name === project.identity.name) {
    render();
    return;
  }
  mutate("metadata", (draft) => {
    draft.identity.name = name;
  });
}

/** The program's name in the title bar is a button: a click turns it
 *  into a field in place. Enter or leaving it keeps the name; Escape
 *  puts the old one back. */
function beginRenameProgram(): void {
  const title = query<HTMLButtonElement>("#program-title");
  if (title.hidden) return;
  const input = document.createElement("input");
  input.type = "text";
  input.className = "program-title-input";
  input.value = project.identity.name;
  input.setAttribute("aria-label", "Program name");
  input.autocomplete = "off";
  input.spellcheck = false;
  let done = false;
  const finish = (commit: boolean): void => {
    if (done) return;
    done = true;
    input.remove();
    title.hidden = false;
    if (commit) renameProgram(input.value);
    else render();
    title.focus();
  };
  input.addEventListener("keydown", (event) => {
    if (event.key === "Enter") {
      event.preventDefault();
      finish(true);
    } else if (event.key === "Escape") {
      event.preventDefault();
      finish(false);
    }
  });
  input.addEventListener("blur", () => finish(true));
  title.hidden = true;
  title.after(input);
  input.focus();
  input.select();
}
query<HTMLButtonElement>("#program-title").addEventListener("click", beginRenameProgram);
query<HTMLSelectElement>("#robot-model").addEventListener("change", (event) => {
  const model = (event.currentTarget as HTMLSelectElement).value;
  if (!isRobotModel(model)) return;
  if (model === project.robot.model) return;
  mutate("robot", (draft) => applyRobotModel(draft, model));
});

/** A robot this server holds a description for. Asked of the catalogue, not
 *  of a list written here: which robots exist is the server's answer
 *  (`domain/robots.ts`), and this was the last place in the browser that
 *  still spelled the two names out. */

/**
 * Copy the connected cell's settings into the program. The order on a real
 * cell is connect, pull, plan: the program is then written against the
 * cell's robot model and axis map, and the plan records which settings it
 * was made for (`planIsFresh`). The weld planner still plans with the repo
 * URDF and limits; what the pull changes today is the model the program
 * is pinned to and what the Workspace says it was written for.
 */
async function pullCellSettings(): Promise<void> {
  let settings = densePanel.cellSettings();
  if (!settings) {
    setStorageStatus("Pull cell settings blocked: the cell has not reported its status yet");
    return;
  }
  // The machine's own machine_planning_calibration.json, which the next plan request carries to the
  // planner. A machine that binds no robot description has none to give;
  // the pull keeps the rest and says the plan will use the model's empty
  // document.
  try {
    settings = await densePanel.fetchCellSettings() ?? settings;
  } catch (error) {
    if (settings.machinePlanningCalibrationSha256) {
      setStorageStatus(`Pull cell settings blocked: ${error instanceof Error ? error.message : String(error)}`);
      return;
    }
    appLogger.warn(`${settings.cell} served no machine_planning_calibration.json (${error instanceof Error ? error.message : String(error)}); plans use the model's empty document`);
  }
  const pulled = settings;
  const pull = cellSettingsPull(project, pulled);
  if (pulled.robotModelId && !isRobotModel(pulled.robotModelId)) {
    appLogger.warn(`${pulled.cell} reports robot model ${pulled.robotModelId}, which this OLP has no cell descriptor for; the program keeps ${project.robot.model}`);
  }
  if (!pull) {
    setStorageStatus(`Cell settings already match ${pulled.cell}`);
    return;
  }
  mutate("robot", pull.apply);
  appLogger.info(`Pulled cell settings from ${settings.cell}: ${describeCellSettings(settings)}`);
}

/**
 * A machine_planning_calibration.json from a file, as the cell settings of a program with no cell
 * to ask. Checked here only as far as a file can be: JSON, the cell schema,
 * the program's own model. Whether the deviation is one the description
 * admits is the server's judgement when it packs, and whether it is the
 * cell's is the cell's when it plays -- the plan's header says `target`, and
 * the gate is the hash (docs/PLANNING-CONTEXT.md, Offline).
 */
async function loadCellJsonFromFile(): Promise<void> {
  const picker = document.createElement("input");
  picker.type = "file";
  picker.accept = ".json,application/json";
  const file = await new Promise<File | null>((resolve) => {
    picker.addEventListener("change", () => resolve(picker.files?.[0] ?? null), { once: true });
    picker.addEventListener("cancel", () => resolve(null), { once: true });
    picker.click();
  });
  if (!file) return;
  const bytes = new Uint8Array(await file.arrayBuffer());
  let document_: unknown;
  try {
    document_ = JSON.parse(new TextDecoder().decode(bytes));
  } catch (error) {
    setStorageStatus(`${file.name} is not JSON: ${error instanceof Error ? error.message : String(error)}`);
    return;
  }
  const record = document_ as { schema?: unknown; model_id?: unknown };
  if (record?.schema !== "rosie.machine-planning-calibration.v1") {
    setStorageStatus(`${file.name} is not a machine_planning_calibration.json (schema ${String(record?.schema)})`);
    return;
  }
  if (record.model_id !== project.robot.model) {
    setStorageStatus(`${file.name} is for ${String(record.model_id)}; this program is for ${project.robot.model}`);
    return;
  }
  const digest = await crypto.subtle.digest("SHA-256", bytes);
  const machinePlanningCalibrationSha256 = "sha256:" + [...new Uint8Array(digest)].map((b) => b.toString(16).padStart(2, "0")).join("");
  let binary = "";
  for (const byte of bytes) binary += String.fromCharCode(byte);
  const at = new Date().toISOString();
  const settings: CellSettings = {
    cell: `file:${file.name}`,
    jetsonId: null,
    robotModelId: project.robot.model,
    runtimeMode: null,
    mappedAxisMask: null,
    rtcoreHost: null,
    pulledAt: at,
    modelId: project.robot.model,
    robotDescriptionSha256: project.robot.description?.robotDescriptionSha256 ?? null,
    machinePlanningCalibrationSha256,
    cellFetchedAt: at,
    machinePlanningCalibrationBase64: btoa(binary),
    machinePlanningCalibrationSource: "upload",
  };
  mutate("robot", (draft) => {
    draft.robot.cell = settings;
  });
  appLogger.info(`Loaded ${file.name} as ${project.robot.model}'s machine_planning_calibration.json (${shortHash(machinePlanningCalibrationSha256)}); `
    + "plans made without a cell use it, and the cell that plays them checks the hash");
}

/** The first 12 hex of an identity, prefix dropped: enough to tell two
 *  descriptions or two machine_planning_calibration.json apart by eye, short enough for one line. */
function shortHash(value: string): string {
  return value.replace(/^sha256:/, "").slice(0, 12);
}

function describeCellSettings(settings: CellSettings): string {
  return [
    settings.robotModelId ?? "robot model unknown",
    settings.jetsonId ? `on ${settings.jetsonId}` : "",
    settings.runtimeMode ? `RTCore ${settings.runtimeMode.toUpperCase()}` : "",
    settings.mappedAxisMask !== null ? `axes ${mappedAxesLabel(settings.mappedAxisMask)}` : "",
    settings.robotDescriptionSha256 ? `description ${shortHash(settings.robotDescriptionSha256)}` : "",
    settings.machinePlanningCalibrationSha256
      ? `machine_planning_calibration.json ${shortHash(settings.machinePlanningCalibrationSha256)}${settings.machinePlanningCalibrationSource === "upload" ? " (from a file)" : ""}`
      : "",
  ].filter(Boolean).join(" · ");
}

/**
 * The robot's description pin under the model: the drift note when the
 * description this program was authored against is not the one a plan
 * would be made against now, and an explicit acceptance after reviewing changes.
 *
 * The name is the same robot; what moved is the description behind it -- new
 * geometry, a narrowed limit, another torch. Show the recorded differences
 * before accepting the new identity; acceptance still requires a fresh plan.
 */
function renderRobotDescriptionPin(host: HTMLElement): void {
  const authoredAgainst = project.robot.description?.robotDescriptionSha256 ?? "";
  if (robotCatalogueError) {
    host.append(element("p", "inspector-note is-warn",
      `Robot descriptions unavailable: ${robotCatalogueError}. Reload once the server can list them.`));
    return;
  }
  const drift = robotDescriptionDrift(project);
  if (drift) {
    const review = robotSettingsReview(project);
    host.append(element("p", "inspector-note is-warn",
      "Robot settings have changed. Review the changes below, accept the current settings, then plan again."));
    host.append(element("h4", "robot-settings-heading", review.title));
    host.append(element("p", "field-help robot-settings-date", review.date
      ? `Change recorded: ${new Date(review.date).toLocaleString(undefined, { dateStyle: "medium", timeStyle: "short" })}`
      : "The change date was not recorded."));
    if (review.rows.length) {
      const table = element("table", "robot-settings-changes");
      const header = element("tr");
      for (const label of ["Setting", "Previous", "Current"]) {
        const cell = element("th", "", label);
        cell.setAttribute("scope", "col");
        header.append(cell);
      }
      const head = element("thead");
      head.append(header);
      const body = element("tbody");
      for (const change of review.rows) {
        const row = element("tr");
        const label = element("th", "", change.label);
        label.setAttribute("scope", "row");
        row.append(label, element("td", "", change.before), element("td", "", change.after));
        body.append(row);
      }
      table.append(head, body);
      host.append(table);
    }
    if (review.unchanged.length) host.append(element("p", "field-help", `Unchanged: ${review.unchanged.join(", ")}.`));
    if (review.explanation) host.append(element("p", "field-help", review.explanation));
    host.append(element("p", "field-help",
      "Accepting keeps your parts and welds. Plan again and review the simulation before running."));
    const versions = element("details", "robot-settings-details");
    versions.append(element("summary", "", "Technical details"),
      summaryRow("Robot", project.robot.model),
      summaryRow("Program version", shortHash(drift.from)),
      summaryRow("Current version", shortHash(drift.to)));
    if (review.provenance) versions.append(element("p", "field-help", review.provenance));
    if (review.technical.commit) versions.append(summaryRow("Recorded change", review.technical.commit.slice(0, 8)));
    host.append(versions);
    const programId = project.identity.id;
    const modelId = project.robot.model;
    const cell = densePanel.connectedCell();
    const adopt = element("button", "button secondary accept-robot-settings", "Accept current settings");
    adopt.type = "button";
    adopt.addEventListener("click", () => {
      if (adopt.disabled) return;
      const current = robotDescriptionDrift(project);
      if (!adopt.isConnected || project.identity.id !== programId || project.robot.model !== modelId ||
          densePanel.connectedCell() !== cell || current?.from !== drift.from || current?.to !== drift.to) {
        renderCellSettings();
        return;
      }
      adopt.disabled = true;
      adoptCurrentRobotDescription();
    });
    const row = element("div", "adopt-row");
    row.append(adopt);
    host.append(row);
    return;
  }
  const adopted = project.robot.description?.adopted?.at(-1);
  if (authoredAgainst) host.append(summaryRow("Robot settings", "Up to date"));
  if (adopted) {
    host.append(element("p", "inspector-note",
      `Accepted ${adopted.at.slice(0, 10)}; plan and review the simulation before running.`));
  }
}

/** The Robot card's one override, and the whole of it: this program is now
 *  authored against the description the server (or the connected cell)
 *  holds. It asks the server nothing, re-plans nothing, and makes no stored
 *  plan playable -- it records that somebody took responsibility for the
 *  difference. */
function adoptCurrentRobotDescription(): void {
  const drift = robotDescriptionDrift(project);
  if (!drift) return;
  mutate("metadata", (draft) => adoptRobotDescription(draft));
  appLogger.warn(
    `Adopted ${project.robot.model} description ${shortHash(drift.from)} -> ${shortHash(drift.to)}; `
    + "nothing has been re-checked -- plan again and verify before running",
  );
  setStorageStatus(
    `Adopted the current ${project.robot.model} description; plan again and verify before running`,
  );
}

/** "J1–J6" for a contiguous mask, "J1 J2 J4" otherwise. */
function mappedAxesLabel(mask: number): string {
  const axes: number[] = [];
  for (let axis = 0; axis < 9; axis += 1) if (mask & (1 << axis)) axes.push(axis + 1);
  if (axes.length === 0) return "none";
  const contiguous = axes.every((axis, index) => index === 0 || axis === axes[index - 1]! + 1);
  return contiguous && axes.length > 1 ? `J${axes[0]}–J${axes.at(-1)}` : axes.map((axis) => `J${axis}`).join(" ");
}

/** The Workspace inspector's cell block: what the program was written
 *  for, from which cell, and whether the connected cell is that one. */
function renderCellSettings(): void {
  const host = query<HTMLElement>("#cell-settings");
  const settings = project.robot.cell ?? null;
  const connected = densePanel.connectedCell();
  const live = connected ? densePanel.cellSettings() : null;
  // Status polling must not replace a pressed summary/button or discard an
  // open disclosure or focused acceptance button.
  // Invalidate on program/settings changes, not live motion or fetch times.
  const renderKey = JSON.stringify([
    project.identity.id, project.robot.model, project.robot.description,
    robotCatalogueError, robotDescriptionDrift(project), robotSettingsReview(project), connected,
    settings && [settings.cell, settings.robotModelId, settings.jetsonId,
      settings.runtimeMode, settings.mappedAxisMask, settings.robotDescriptionSha256,
      settings.machinePlanningCalibrationSha256, settings.machinePlanningCalibrationSource,
      settings.pulledAt, cellSettingsKey(settings)],
    live && [cellSettingsKey(live), describeCellSettings(live)],
  ]);
  if (host.dataset.renderKey === renderKey && host.childElementCount > 0) return;
  host.dataset.renderKey = renderKey;
  host.replaceChildren();
  renderRobotDescriptionPin(host);
  if (!settings) {
    host.append(element("p", "inspector-empty", connected
      ? `Connected to ${connected}. Pull its settings from Cells before planning.`
      : "No cell settings. Connect to a cell from Cells and pull its settings."));
    return;
  }
  host.append(element("p", "eyebrow cell-settings-source", `FROM ${settings.cell.toUpperCase()}`));
  host.append(summaryRow("Robot model", settings.robotModelId ?? "unknown"));
  if (settings.jetsonId) host.append(summaryRow("Jetson", settings.jetsonId));
  host.append(summaryRow("RTCore mode", settings.runtimeMode ? settings.runtimeMode.toUpperCase() : "unknown"));
  if (settings.mappedAxisMask !== null) host.append(summaryRow("Mapped axes", mappedAxesLabel(settings.mappedAxisMask)));
  if (settings.robotDescriptionSha256) host.append(summaryRow("Description", shortHash(settings.robotDescriptionSha256)));
  if (settings.machinePlanningCalibrationSha256) {
    host.append(summaryRow("machine_planning_calibration.json", shortHash(settings.machinePlanningCalibrationSha256) + (settings.machinePlanningCalibrationSource === "upload" ? " (from a file)" : "")));
  }
  host.append(summaryRow("Pulled", plannedAtLabel(settings.pulledAt)));
  if (connected && connected !== settings.cell) {
    host.append(element("p", "inspector-note is-warn", `Connected to ${connected}, not ${settings.cell}. Pull again before planning.`));
  } else if (connected) {
    if (live && cellSettingsKey(live) !== cellSettingsKey(settings)) {
      host.append(element("p", "inspector-note is-warn", `${connected} now reports ${describeCellSettings(live)}. Pull again before planning.`));
    }
  }
}

/** The header pill: which cell the session is connected to. */
function renderCellBadge(): void {
  const cell = densePanel.connectedCell();
  const summary = densePanel.connectionSummary();
  const tone = cell === null ? "off" : densePanel.daemonReachable() ? "ok" : "down";
  const host = document.querySelector<HTMLElement>("#robot-status");
  if (host) {
    host.dataset.tone = tone;
    setBusyText(host.querySelector<HTMLElement>(".menubar-status-text")!, summary);
    host.title = densePanel.targetStatusText() || summary;
  }
  // The Deploy panel's target row says the same thing, where the run is.
  const target = document.querySelector<HTMLElement>("#deploy-target-name");
  if (target) {
    const settings = densePanel.cellSettings();
    target.textContent = cell === null ? "No cell selected" : [cell, settings?.robotModelId, settings?.backend, settings?.configurationDigest?.slice(0, 12)].filter(Boolean).join(" · ");
    target.dataset.tone = tone;
    target.title = cell === null ? "Choose a cell from Cells" : summary;
  }
}
query<HTMLSelectElement>("#program-execution-mode").addEventListener(
  "change",
  (event) => {
    const mode = (event.currentTarget as HTMLSelectElement).value;
    if (mode !== "simulation" && mode !== "dry_run" && mode !== "production") {
      return;
    }
    if (mode === project.program.executionMode) return;
    mutate("program", (draft) => {
      draft.program.executionMode = mode;
    });
    // Fail closed: changing execution mode always drops the sim arm state.
    setSimArmed(false);
  },
);
query<HTMLSelectElement>("#target-mode").addEventListener("change", (event) => {
  void selectTarget((event.currentTarget as HTMLSelectElement).value);
});

async function selectTarget(value: string): Promise<void> {
  if (activeExecution) {
    query("#target-state").textContent =
      "Stop the active robot execution before changing targets.";
    return;
  }
  const now = Date.now();
  serverExecutionReview = null;
  serverExecutionReviewWire = null;
  connectedStatus = null;
  connectedCapabilities = null;
  if (value.startsWith("weld_planner")) {
    // The weld planner authors motion at Compile to rows; what this target
    // arms is the CHECKER — the canonical local simulator that replays the
    // compiled joints for the evidence. The suffix names which free-space
    // solver the next compile hands its connecting motions to; it is session
    // state, so it is set here and never written to the project.
    const backend = value.split(":")[1];
    if (backend === "bspline" || backend === "curobo" || backend === "legacy") {
      railsMode.freeSpaceBackend = backend;
    }
    if (!usesDensePlanner()) {
      query("#target-state").textContent =
        "Weld planner motion is for rails programs — every New program is one; import a part and it comes on.";
      updateTargetOptions();
      return;
    }
    const target = localSimulatorTarget();
    // Refresh FIRST. The safety proof is fresh for five seconds and
    // `selectTarget` runs whenever a human touches the selector -- always
    // longer than that after the last poll -- so applying the cached status
    // blocks the target on a proof that is merely OLD, not unsafe. This was
    // invisible while rails mode offered a single enabled option that was
    // already selected at startup: nothing ever called this path.
    await refreshLocalSimulatorStatus();
    applyLocalTargetState(target, Date.now());
    setProjectTarget("local_simulator", target.profileId, null);
  } else if (value === "offline_preview") {
    targetState = reduceTargetState(targetState, { type: "select_offline", atMs: now });
    setProjectTarget("offline_preview", null, null);
  } else if (value === "local_simulator") {
    const target = localSimulatorTarget();
    await refreshLocalSimulatorStatus();  // same five-second proof
    applyLocalTargetState(target, Date.now());
    setProjectTarget("local_simulator", target.profileId, null);
  } else {
    const profileId = value.startsWith("connected:") ? value.slice("connected:".length) : "";
    const discovery = connectedTargets.find((candidate) => candidate.profileId === profileId);
    if (!discovery) {
      targetState = reduceTargetState(targetState, {
        type: "disconnect",
        atMs: now,
        reason: "manifest_backed_target_unavailable",
      });
      renderTargetAndReview();
      return;
    }
    targetState = reduceTargetState(targetState, {
      type: "select_real_target",
      atMs: now,
      target: discovery.target,
    });
    setProjectTarget("real_connected", discovery.profileId, discovery.manifestRef);
    renderTargetAndReview();
    await readConnectedTarget(discovery);
  }
  query<HTMLSelectElement>("#target-mode").value = value;
  renderTargetAndReview();
}
// The stage toolbar. Every button is a view of one Action: the command's
// enabled/checked state is set once on the action (by the render passes
// and the handlers below) and the button, its tooltip and its shortcut all
// follow. See src/ui/action.ts.
const appToolbar = new ToolBar("Toolbar").mount(query<HTMLElement>("#app-toolbar-host"));
// Project ▾: the file commands, named by where they put the program. Three
// stores exist -- this browser's autosave, the shared catalogue, and files
// -- and "save" alone has lost programs to the wrong one.
const projectMenu = new Menu("Project").mount(document.body);
const projectActions = {
  newProgram: new Action({ id: "project-new", label: "New program…", tooltip: "Start a blank program" }),
  openProgram: new Action({ id: "project-open-program", label: "Open program…", tooltip: "Open one of the cell's saved programs, or one kept only in this browser" }),
  openFile: new Action({ id: "project-open-file", label: "Open file…", tooltip: "Open a program exported as .json" }),
  saveCatalogue: new Action({ id: "project-save-catalogue", label: "Save to catalogue", tooltip: "Save this program to the shared Motion Server catalogue" }),
  refreshCatalogue: new Action({ id: "project-refresh-catalogue", label: "Refresh catalogue", tooltip: "Read the catalogue again" }),
  exportWeldplan: new Action({ id: "project-export", label: "Export .weldplan…", tooltip: "Download this program as a .weldplan file" }),
};
projectActions.newProgram.triggered.connect(openNewProgramDialog);
projectActions.openProgram.triggered.connect(() => void openProgramDialog.open());
projectActions.openFile.triggered.connect(() => query<HTMLInputElement>("#project-file").click());
projectActions.saveCatalogue.triggered.connect(() => void saveNow());
projectActions.refreshCatalogue.triggered.connect(() => void refreshProgramCatalog());
projectActions.exportWeldplan.triggered.connect(() => void exportProject());
// The app's settings, the ≡ menu's last section: one submenu per setting,
// so the section can grow without lengthening the menu (owner, 2026-09-26).
// Theme is an app setting, not a view of the model, so it is here rather
// than in the viewport's view menu; its radios keep the submenu open so
// light and dark can be compared from it.
const themeMenu = new Menu("Theme");
const themeGroup = new ActionGroup();
for (const [value, label] of [["system", "Match system"], ["light", "Light"], ["dark", "Dark"]] as const) {
  const action = new Action({ id: "theme-" + value, label, group: themeGroup, checked: themePreference() === value });
  action.triggered.connect(() => setThemePreference(value as ThemePreference));
  themeMenu.addAction(action, { keepOpen: true });
}
themeMenu.beforeOpen = () => {
  for (const action of themeGroup.actions) action.checked = action.id === "theme-" + themePreference();
};
projectMenu.beforeOpen = () => {
  projectMenu.clear();
  projectMenu.addActions([projectActions.newProgram, projectActions.openProgram, projectActions.openFile]);
  projectMenu.addSeparator();
  projectMenu.addAction(projectActions.exportWeldplan);
  projectMenu.addSeparator();
  projectMenu.addSubmenu("Theme", themeMenu);
};
// Catalogue ▾: the shared Motion Server program store -- what the pendant
// and the production planner see. Its programs to open, and save/refresh.
const catalogueMenu = new Menu("Catalogue").mount(document.body);
catalogueMenu.beforeOpen = () => {
  catalogueMenu.clear();
  // The cell's program store first: what this robot's Deck and every other
  // browser open (storage/cell-sync.ts). The Motion Server catalogue below is
  // only listed where a server is configured with one.
  const cell = cellSync.view();
  if (cell.state !== "disabled" && cell.state !== "unknown") {
    catalogueMenu.addHeading(`On ${cell.host} · ${cell.programs.length} program${cell.programs.length === 1 ? "" : "s"}`);
    catalogueMenu.addNote(cell.editorsText ? `${cell.label} · ${cell.editorsText}` : cell.label);
    // One way to open a cell program, with its search and draft question.
    catalogueMenu.addAction(projectActions.openProgram);
    if (cell.offerUpload) {
      const upload = new Action({ id: "cell-upload", label: `Upload this program to ${cell.host}`, tooltip: "Save it in the cell's program store" });
      upload.triggered.connect(() => cellSync.upload(false));
      catalogueMenu.addAction(upload);
    } else if (cell.programId && !cell.isCurrent && cell.state !== "conflict") {
      const current = new Action({ id: "cell-current", label: `Make this ${cell.host}'s current program`, tooltip: "The Steam Deck opens the cell's current program" });
      current.triggered.connect(() => cellSync.makeCurrent());
      catalogueMenu.addAction(current);
    }
    if (!programCatalog) return;
    catalogueMenu.addSeparator();
    catalogueMenu.addHeading("Motion Server catalogue");
  }
  const programs = programCatalog?.programs ?? [];
  const status = query("#catalog-status").textContent?.trim() ?? "";
  catalogueMenu.addHeading(programs.length > 0 ? `Programs · ${programs.length}` : "Programs");
  if (programs.length === 0) catalogueMenu.addNote(programCatalogError ?? (status || "Catalogue not loaded"));
  for (const program of programs.slice(0, 12)) {
    const open = new Action({
      id: `catalogue-open:${program.program_id}`,
      label: program.program_name,
      tooltip: `Open from the catalogue · r${program.program_revision}`,
    });
    open.triggered.connect(() => void openCatalogProgram(program.program_id));
    catalogueMenu.addAction(open);
  }
  catalogueMenu.addSeparator();
  catalogueMenu.addActions([projectActions.saveCatalogue, projectActions.refreshCatalogue]);
  if (status && programs.length > 0) catalogueMenu.addNote(status);
};
// The cell's program store (storage/cell-sync-browser.ts): the open program
// is kept in step with the cell it is connected to, IndexedDB staying this
// browser's copy and draft. Started here, after every module binding the
// redraw reads exists; its answers arrive after boot.
const cellSync = new BrowserCellSync({
  clientId,
  project: () => project,
  open: openFromCell,
  notice: (text) => setStorageStatus(text),
  newProgram: () => openNewProgramDialog(),
  openLocal: (id) => void openLocalProgram(id),
  changed: () => { if (openProgramDialog.shown) openProgramDialog.render(); },
});
// Program ▸ Open program… (storage/open-program-dialog.ts): the cell's
// programs and this browser's own, through the shared engine's refusals and
// draft question.
const openProgramDialog = new OpenProgramDialog({
  sync: cellSync,
  listLocal: () => repository.list(),
  runLock: programSwitchLockReason,
  notice: (text) => setStorageStatus(text),
});
cellSync.start();

/** Why this browser must not switch programs now (cell-sync.ts programSwitchLock). */
function programSwitchLockReason(): string {
  const dense = densePanel.switchLockInput();
  return programSwitchLock({ ...dense, busy: dense.busy || (activeExecution ? "the reviewed robot execution is active" : "") });
}

/** This browser's own copy of a program (a local-only one, or its draft of a
 *  cell program), opened as a program from the cell is. */
async function openLocalProgram(id: string): Promise<void> {
  try {
    await autosave.flush();
    const stored = await repository.get(id);
    if (!stored) throw new Error("it is no longer in this browser");
    const next = pinRobotDescription(migrateLegacyTablePlacement(stored));
    openFromCell(next, `Opened ${next.identity.name} from this browser`);
  } catch (error) {
    setStorageStatus(`Open blocked: ${message(error)}`);
  }
}

/** A program from the cell replaces the open one: an edit of the same
 *  program is one undo step, another program starts its own history. */
function openFromCell(next: OfflineProgrammingProject, text: string): void {
  const previous = project;
  const same = previous.identity.id === next.identity.id;
  project = next;
  if (same) history.push(previous);
  else history.clear();
  rememberCurrentProject(project.identity.id);
  if (!same) {
    resetWeldAuthoringUiState();
    syncRailsModeToProject();
  }
  if (!same || previous.cad.fingerprint !== next.cad.fingerprint) {
    void restoreCADSource(project).then((source) => {
      if (project === next) activeCADSource = source;
    });
  }
  changed(text);
}

// The application menus -- Project, Catalogue, Cells -- are the header's,
// as a menu bar beside the app's name: they are about the document and
// the session (which program, which store, which cell), not about the
// model, which is what the toolbar under them is for. Undo and Redo
// stay on the toolbar because they act on the model.
const menubar = query<HTMLElement>("#app-menubar");
// The program's own menu (new, open, export) hangs off the ≡ before its
// name, where a document's menu sits; Catalogue and Cells follow the name.
new MenuButton({ label: "Program", icon: MenuIcon, showLabel: false, caret: false, tooltip: "New, open, export; theme" }, projectMenu)
  .mount(query<HTMLElement>("#program-menu-host"));
new MenuButton({ label: "Catalogue", caret: false, tooltip: "The shared program catalogue: open, save, refresh" }, catalogueMenu).mount(menubar);
// No names under the groups: the separators divide them and the buttons
// say what they are.
const projectGroup = appToolbar.addGroup();
// Cells ▾: which cell the session drives and the pull of its settings,
// as a menu like Project and Plan; the registry (add, remove, dial) is
// more than a menu holds, so "Manage cells…" opens Cells. On a
// real cell the order is connect, pull, plan; the target used to be a
// select at the bottom of Deploy, after the plan it should have preceded.
const robotActions = {
  pullCellSettings: new Action({
    id: "robot-pull-cell",
    label: "Pull cell settings",
    tooltip: "Copy the connected cell's robot model and axis map into this program's Workspace",
  }),
  loadCellJson: new Action({
    id: "robot-load-machine-planning-calibration",
    label: "Load machine_planning_calibration.json…",
    tooltip: "A cell's machine_planning_calibration.json from a file, for planning with no cell connected; the cell that plays the plan checks the hash",
  }),
  manageCells: new Action({
    id: "robot-manage-cells",
    label: "Manage cells…",
    tooltip: "The cell registry: dial, add and remove cells",
  }),
  searchNetwork: new Action({
    id: "robot-search-network",
    label: "Search local network…",
    tooltip: "OLP servers advertising on this server's network and the cells they present; Add fills the form for you to confirm",
  }),
};
robotActions.pullCellSettings.triggered.connect(() => void pullCellSettings());
robotActions.loadCellJson.triggered.connect(() => void loadCellJsonFromFile());
robotActions.manageCells.triggered.connect(() => setRobotOpen(true));
robotActions.searchNetwork.triggered.connect(() => openDiscoveredCell(null));
const robotMenu = new Menu("Cells").mount(document.body);
robotMenu.beforeOpen = () => {
  robotMenu.clear();
  robotMenu.addHeading("Connect to");
  const targets = new ActionGroup();
  for (const option of densePanel.targetOptions()) {
    const choice = new Action({
      id: `robot-target:${option.value || "none"}`,
      label: option.label,
      group: targets,
      checked: option.selected,
    });
    choice.setEnabled(!option.disabled && !densePanel.selecting(), "Not available");
    choice.triggered.connect(() => densePanel.selectTarget(option.value));
    robotMenu.addAction(choice);
  }
  robotMenu.addNote(densePanel.targetStatusText() || "Joint daemon · checking");
  robotMenu.addSeparator();
  const live = densePanel.cellSettings();
  const lock = editingLockReason();
  robotActions.pullCellSettings.setEnabled(live !== null && lock === null,
    lock ?? (densePanel.targetSelected() ? "Waiting for the cell's status" : "Connect to a cell first"));
  robotMenu.addAction(robotActions.pullCellSettings);
  // With no cell to ask, a file stands in (docs/PLANNING-CONTEXT.md,
  // Offline). Connected, the cell's own file is the truth and this is off.
  robotActions.loadCellJson.setEnabled(!densePanel.targetSelected() && lock === null,
    lock ?? "Connected: the cell's own machine_planning_calibration.json is what a plan is made against");
  robotMenu.addAction(robotActions.loadCellJson);
  const held = project.robot.cell ?? null;
  robotMenu.addNote(held
    ? `Workspace is from ${held.cell}: ${describeCellSettings(held)}`
    : "Workspace has no cell settings yet");
  robotMenu.addSeparator();
  // What the last search found; opening the menu refreshes it in the
  // background (the server answers from its 10 s cache when it can).
  robotMenu.addHeading("On the local network");
  void cellDiscovery.refresh(false);
  const found = cellDiscovery.rows();
  for (const row of found.slice(0, 6)) {
    const flag = row.compatibility === "incompatible" ? " · incompatible" : row.configured ? " · configured" : "";
    const pick = new Action({
      id: `robot-discovered:${row.key}`,
      label: `${row.title} · ${row.where}${flag}`,
      tooltip: `${row.subtitle}. ${row.badges.map((badge) => badge.text).join(", ")}`,
    });
    pick.triggered.connect(() => openDiscoveredCell(row));
    robotMenu.addAction(pick);
  }
  if (found.length === 0 || found.length > 6) robotMenu.addNote(cellDiscovery.statusText());
  robotMenu.addAction(robotActions.searchNetwork);
  robotMenu.addSeparator();
  robotMenu.addAction(robotActions.manageCells);
};
new MenuButton({ label: "Cells", caret: false, tooltip: "Choose the cell, pull its settings, manage the registry" }, robotMenu).mount(menubar);
// Beside Robot: which cell, and what it is doing -- the session's state
// where the choice is made, so no one opens the menu to find out.
{
  const status = element("span", "menubar-status");
  status.id = "robot-status";
  status.append(element("span", "menubar-status-dot"), element("span", "menubar-status-text"));
  menubar.append(status);
}
projectGroup.addActions([stageActions.undo, stageActions.redo], { showLabel: false });
// Import ▾: the kept STEPs first (no file browser), then a file. The demo
// parts are under Project, as files to open.
const importMenu = new Menu("Import part").mount(document.body);
importMenu.beforeOpen = () => {
  importMenu.clear();
  for (const source of recentParts.slice(0, 6)) {
    const open = new Action({
      id: `import-recent:${source.fingerprint}`,
      label: source.filename,
      tooltip: `Import again from this browser's kept copy · ${source.storedAt.slice(0, 10)}`,
    });
    open.triggered.connect(() => void importRecentPart(source.fingerprint));
    importMenu.addAction(open);
  }
  if (recentParts.length > 0) importMenu.addSeparator();
  const fromFile = new Action({ id: "import-file", label: "Weld parts from files…", tooltip: "Add STEP files, each with its own placement and welds" });
  fromFile.triggered.connect(() => chooseStepFiles("weld"));
  importMenu.addAction(fromFile);
  const fixtureFile = new Action({ id: "import-fixture", label: "Fixtures / obstacles from files…", tooltip: "Add STEP geometry using the existing stand and fixture controls" });
  fixtureFile.triggered.connect(() => chooseStepFiles("fixture"));
  importMenu.addAction(fixtureFile);
};
const teachMenu = new Menu("Teach").mount(document.body);
teachMenu.beforeOpen = () => {
  teachMenu.clear();
  for (const [mode, label] of [["new", poseSource === "real" ? "Record robot waypoint" : "Record preview waypoint"], ["reteach", "Reteach selected waypoint"], ["via", "Record circular via pose"]] as const) {
    const action = new Action({ id: `teach-${mode}`, label });
    action.setEnabled(!editingLockReason() && (mode === "new" || project.nodes.find(n => n.id === selectedNodeId)?.op === "move"), "Select a taught waypoint");
    action.triggered.connect(() => recordWaypoint(mode));
    teachMenu.addAction(action);
  }
};
const teachGroup = appToolbar.addGroup();
teachGroup.add(new MenuButton({ label: "Teach", icon: Spline }, teachMenu));
const partGroup = appToolbar.addGroup();
partGroup.add(new MenuButton({ label: "Import", icon: Upload, caret: false, tooltip: "Import a part: a kept STEP or a file" }, importMenu));
partGroup.addActions([stageActions.movePart]);
appToolbar.addGroup().addActions([
  stageActions.railsWelds,
  stageActions.waypointWelds,
  stageActions.lineWelds,
  stageActions.arcWelds,
  stageActions.weldPresets,
]);
// Plan ▾: the command, then the planner it runs on -- the same choice the
// Workspace inspector's Planner select offers, as radio items, and the
// planner's status line under them. Choosing a planner keeps the menu open,
// so its status line can be read and Plan run from the same menu; the radios
// and the line resync once the switch settles, since `selectTarget` can
// refuse it (an active execution) and leave the select where it was.
const planMenu = new Menu("Plan").mount(document.body);
planMenu.beforeOpen = () => {
  planMenu.clear();
  planMenu.addAction(stageActions.plan);
  planMenu.addSeparator();
  planMenu.addHeading("Planner");
  const select = query<HTMLSelectElement>("#target-mode");
  const planners = new ActionGroup();
  // By value: updateTargetOptions rebuilds the <option>s, so the ones read
  // here may be detached by the time a switch settles.
  const choices = new Map<string, Action>();
  const sync = (): void => {
    const current = choices.get(select.value);
    if (current) current.checked = true;
    note.textContent = query("#target-state").textContent?.trim() || "";
  };
  for (const option of select.options) {
    const choice = new Action({
      id: `planner:${option.value}`,
      label: option.textContent?.trim() || option.value,
      tooltip: option.title || option.textContent?.trim() || option.value,
      group: planners,
      checked: option.selected,
    });
    choice.setEnabled(!option.disabled, option.title || "Not available for this program");
    choice.triggered.connect(() => void selectTarget(option.value).finally(sync));
    planMenu.addAction(choice, { keepOpen: true });
    choices.set(option.value, choice);
  }
  planMenu.addSeparator();
  const note = planMenu.addNote("");
  sync();
};
const planGroup = appToolbar.addGroup();
planGroup.add(new MenuButton({ label: "Plan", icon: Waypoints, caret: false, tooltip: "Plan the program, or choose the planner" }, planMenu));
planGroup.addActions([stageActions.planPresets]);
// Unverified visibility preference; enabling the view never acquires control.
const jogAction = new Action({ id: "toggle-jog", label: "Jog", icon: Gamepad2, checkable: true, checked: jogPadVisible, tooltip: "Jog the selected armed cell and watch its drives' telemetry" });
/** The telemetry panel: beside the Jog column when Jog is open, the whole
 *  pane when it is not. */
const telemetryAction = new Action({ id: "toggle-telemetry", label: "Telemetry", icon: Activity, checkable: true, checked: telemetryVisible, tooltip: "Show the drives' telemetry, with or without the jog controls" });
for (const tab of document.querySelectorAll<HTMLButtonElement>(".jog-mode-tab")) {
  tab.addEventListener("click", () => applyJogMode(tab.dataset.jogMode === "cartesian" ? "cartesian" : "joints"));
}
applyJogMode(jogMode);
jogAction.triggered.connect(toggleJogPad);
telemetryAction.triggered.connect(toggleTelemetryPanel);
// The bottom dock's views, in the order they stack: the spreadsheet, then
// the Jog column and its telemetry beside it.
appToolbar.addGroup().addActions([stageActions.spreadsheet, jogAction, telemetryAction]);
applyTelemetryVisible();

// The view menu: display settings live under the axis gizmo, the way a
// CAD view cube carries its view menu, not on the command toolbar. The
// menu is a popup over the whole page (fixed), so it lives on <body>; its
// button sits in the orbit pane and must not be a click on the pane.
const viewMenu = new Menu("View settings").mount(document.body);
viewMenu.addActions([stageActions.labels, stageActions.tcpFrame, stageActions.tcpPath, stageActions.executedPath]);
viewMenu.addSeparator();
viewMenu.addActions([stageActions.j6Cam, stageActions.tcpCam]);
const poseGroup = new ActionGroup();
for (const [value, label] of [["sim", "Preview pose"], ["real", "Live pose"]] as const) {
  const action = new Action({ id: "pose-source-" + value, label, group: poseGroup, checked: poseSource === value });
  action.triggered.connect(() => setPoseSource(value));
  viewMenu.addAction(action);
}
themeChanged.connect(() => {
  viewer.applyTheme();
  telemetryStripInstance?.render();
});
applyResolved();
viewMenu.beforeOpen = () => {
  for (const action of poseGroup.actions) {
    action.checked = action.id === "pose-source-" + poseSource;
    action.setEnabled(!densePanel.robotMoving() && !playing && !railsPlaying && (action.id.endsWith("sim") || densePanel.targetSelected()), "No cell connected or motion is active");
  }
};
const viewMenuHost = query<HTMLElement>("#view-menu-host");
// Home view (zoom to fit) sits with the view menu, as a view cube's home
// button does, not among the editing commands.
new ToolButton(stageActions.frame, { showLabel: false, iconSize: 16 }).mount(viewMenuHost);
new MenuButton({
  label: "View settings",
  icon: Settings,
  showLabel: false,
  caret: false,
  align: "end",
  iconSize: 16,
  tooltip: "View settings: overlays and camera panes",
}, viewMenu).mount(viewMenuHost);
for (const type of ["pointerdown", "pointerup", "click"] as const) {
  viewMenuHost.addEventListener(type, (event) => event.stopPropagation());
}
stageActions.frame.triggered.connect(() => viewer.framePath(projectFramePoints(project)));
stageActions.tcpPath.triggered.connect(action => viewer.setPlannedPathVisible(action.checked));
stageActions.executedPath.triggered.connect(action => viewer.setExecutionPathVisible(action.checked));
stageActions.labels.triggered.connect((action) => {
  showPathLabels = action.checked;
  syncAuthoringOverlays(viewer, project, { showLabels: showPathLabels });
});
stageActions.movePart.triggered.connect((action) => setPartGizmo(action.checked));
stageActions.railsWelds.triggered.connect((action) => setRailsWeldsOpen(action.checked));
// Picking a hand-placed weld tool DRAWS one, as a CAD sketch tool does: a
// new weld of that kind, from the end of the one edited last. Editing an
// existing weld is opening it (tree, inspector, right-click).
for (const [tool, action] of Object.entries(weldToolActions) as [WeldTool, Action][]) {
  action.triggered.connect(() => setWaypointWeldsOpen(action.checked, tool, { create: true }));
}
viewer.waypointLayer.add(waypointOverlay.root);
viewer.waypointLayer.add(weldHighlight.root);
waypointOverlay.torchesVisible = false;
waypointSession.onSelect(renderWaypointWelds);
stageActions.weldPresets.triggered.connect((action) => setWeldPresetsOpen(action.checked));
stageActions.plan.triggered.connect(() => void planProgram());
stageActions.planPresets.triggered.connect((action) => setPlanPresetsOpen(action.checked));
stageActions.spreadsheet.triggered.connect((action) => applySpreadsheetShown(action.checked));
applySpreadsheetShown(stageActions.spreadsheet.checked);
programTree.selected.connect((id) => void selectTreeItem(id));
// Right-click on a tree row: select it and offer what can be done to it.
const contextMenu = new Menu("Context").mount(document.body);
query<HTMLElement>("#program-tree-host").addEventListener("contextmenu", async (event) => {
  const row = (event.target as HTMLElement).closest<HTMLElement>(".tree-row");
  if (!row?.dataset.nodeId) return;
  event.preventDefault();
  if (!await selectTreeItem(row.dataset.nodeId)) return;
  contextMenu.clear();
  const items = contextActionsFor(selection);
  if (items.length === 0) return;
  contextMenu.addActions(items);
  contextMenu.openAt(event.clientX, event.clientY);
});
programTree.activated.connect(async (id) => {
  if (!await selectTreeItem(id)) return;
  const sel = selection;
  if (sel.kind === "part") launchPartMove();
  if (sel.kind === "fixture") openStandCard(sel.id);
  if (sel.kind === "weld") openWeld(sel.id);
});
stageActions.tcpFrame.triggered.connect((action) => {
  showTcpFrame = action.checked;
  viewer.setTcpFrameEnabled(showTcpFrame);
});
stageActions.j6Cam.triggered.connect((action) => {
  j6CamVisible = action.checked;
  applyCameraPaneVisibility();
});
stageActions.tcpCam.triggered.connect((action) => {
  tcpCamVisible = action.checked;
  applyCameraPaneVisibility();
});
window.addEventListener("keydown", (event) => {
  if (event.key !== "Delete" || isTypingTarget(event.target)) return;
  if (selection.kind !== "weld" && selection.kind !== "part") return;
  if (!document.activeElement || !query<HTMLElement>("#program-tree-host").contains(document.activeElement)) return;
  event.preventDefault();
  if (selection.kind === "part") {
    deletePart();
    return;
  }
  if (deleteBlocked(project.nodes, selection.id) === null && editingLockReason() === null) deleteNode(selection.id);
});
// Right-click on the part in the viewport: the same menu as its tree row.
// A right-drag is an orbit pan and opens nothing.
{
  const pane = query<HTMLElement>("#orbit-pane");
  let rightDown: { x: number; y: number } | null = null;
  pane.addEventListener("pointerdown", (event) => {
    rightDown = event.button === 2 ? { x: event.clientX, y: event.clientY } : null;
  });
  pane.addEventListener("contextmenu", (event) => {
    const started = rightDown;
    rightDown = null;
    if (!started || Math.hypot(event.clientX - started.x, event.clientY - started.y) > 4) return;
    if (!edgeSelection.pickPartPoint(event.clientX, event.clientY)) return;
    event.preventDefault();
    setSelection({ kind: "part" });
    contextMenu.clear();
    const items = contextActionsFor(selection);
    if (items.length === 0) return;
    contextMenu.addActions(items);
    contextMenu.openAt(event.clientX, event.clientY);
  });
}
new Shortcuts(window).bind(
  stageActions.frame,
  stageActions.railsWelds,
  stageActions.movePart,
  stageActions.labels,
  stageActions.plan,
  stageActions.undo,
  stageActions.redo,
);
window.addEventListener("beforeunload", () => {
  void autosave.flush().catch(() => {
    // The page is closing; the queued immutable revision remains best-effort.
  });
  edgeSelection.dispose();
  plannedPathController.dispose();
  viewer.dispose();
});

/** The program this browser has open, remembered so a reload comes back to
 *  it. Booting into "the most recently saved program" let another tab's
 *  autosave of an older program win over a New program made here. */
const CURRENT_PROJECT_KEY = "olp.project.current";

function rememberCurrentProject(id: string): void {
  try { if (localStorage.getItem(CURRENT_PROJECT_KEY) !== id) localStorage.setItem(CURRENT_PROJECT_KEY, id); } catch { /* the choice lasts the session */ }
}

async function loadInitialProject(): Promise<OfflineProgrammingProject> {
  let remembered: string | null = null;
  try { remembered = localStorage.getItem(CURRENT_PROJECT_KEY); } catch { remembered = null; }
  const summaries = await repository.list();
  const chosen = remembered && summaries.some(summary => summary.id === remembered) ? remembered : summaries[0]?.id ?? null;
  const stored = chosen ? await repository.get(chosen) : null;
  if (stored) {
    // A program saved before the description contract takes the
    // description the server holds now, once (`pinRobotDescription`).
    const migrated = pinRobotDescription(migrateLegacyTablePlacement(stored));
    if (migrated !== stored) await repository.save(migrated);
    return migrated;
  }
  // First boot gets an EMPTY program — never a demo full of welds.
  const created = pinRobotDescription(createBlankSpreadsheetProject({ name: "New weld program" }));
  await repository.save(created);
  return created;
}

function mutate(change: ProjectChange, edit: (draft: OfflineProgrammingProject) => void): void {
  const lock = editingLockReason();
  if (lock) {
    setStorageStatus(lock);
    return;
  }
  const previous = project;
  serverExecutionReview = null;
  serverExecutionReviewWire = null;
  project = reviseProject(project, change, (draft) => {
    edit(draft);
    deriveRows(draft, change, selectedNodeId);
  });
  history.push(previous);
  changed("Autosave queued");
}

function changed(message: string): void {
  stopPlayback();
  stopRailsPreview();
  playbackCursorS = 0;
  autosave.schedule(project);
  query("#save-state").textContent = "SAVING";
  setStorageStatus(message);
  render();
  void syncViewerProject();
  void syncRailsSimulation();
}

/** Keep Simulate pointed at the open program's trajectory.
 *
 *  Called from `changed()`, which is the single funnel every project switch,
 *  undo, import and plan passes through -- so this does not have to be
 *  threaded onto each of them, and cannot be forgotten on the next one. */
async function syncRailsSimulation(): Promise<void> {
  if (!usesDensePlanner()) {
    if (railsTrajectoryLoaded !== null) {
      railsSimulation.clear();
      railsTrajectoryLoaded = null;
    }
    return;
  }
  const digest = project.denseTrajectory?.trajectoryDigest ?? null;
  if (digest === railsTrajectoryLoaded) {
    // The same bytes, possibly a different program: ask again whether they
    // are still this program's motion.
    await railsSimulation.recheck(project);
    render();
    return;
  }
  railsTrajectoryLoaded = digest;
  await loadRailsSimulation();
  render();
}

/** The classic plan panel: what the Motion Server accepted, and when it
 *  stopped being this program's. */
function renderClassicPlanState(): void {
  query("#plan-state").textContent = project.plan.state.replace("_", " ").toUpperCase();
  query("#plan-state").dataset.tone = project.plan.state === "server_validated" || project.plan.state === "preview"
    ? "ok"
    : project.plan.state === "stale" ? "caution" : project.plan.state === "rejected" ? "danger" : "neutral";
  query("#plan-source").textContent = project.plan.planner === "browser_preview"
    ? "browser preview"
    : project.plan.state === "server_validated"
      ? project.plan.acceptedPlan?.authority_classification === "physical_target"
        ? "selected robot"
        : "local canonical simulation"
      : project.plan.state === "rejected"
        ? "canonical rejected"
        : "not generated";
  const planDetail = project.plan.state === "stale"
    ? project.plan.staleReason ?? "Authoring revision changed."
    : project.plan.state === "server_validated"
      ? `${project.plan.trajectory.length} accepted Tesseract J1–J9 samples · ${
        project.plan.acceptedPlan?.authority_classification === "physical_target"
          ? `the exact robot-stored trajectory from ${project.plan.acceptedPlan.target.target_id}`
          : "the exact simulation accepted-plan trajectory"
      }`
      : project.plan.state === "rejected"
        ? "Canonical planner evidence rejected this plan. Physical execution remains disabled."
    : project.plan.state === "preview"
      ? "Legacy path-only preview · robot playback disabled · create a Tesseract plan."
      : "Create a timed Tesseract J1–J9 trajectory before robot playback.";
  query("#plan-detail").textContent = planDetail;
  query("#plan-state").title = planDetail;
}

/** The rails plan panel. There is no plan STATE to report -- rails does not
 *  install a plan record -- so it reports the trajectory instead: how long,
 *  when it was planned, and whether the program has moved since.
 *
 *  That last one is the whole reason the digest is kept. A revision
 *  comparison cannot answer it: a revision moves for edits that change no
 *  motion and stands still for some that do. */
function renderRailsPlanState(): void {
  const provenance = railsSimulation.provenance;
  const record = project.denseTrajectory;
  const state = !record
    ? "NOT PLANNED"
    : provenance?.current === false
      ? "STALE"
      : provenance
        ? "PLANNED"
        : "NOT LOADED";
  query("#plan-state").textContent = state;
  query("#plan-state").dataset.tone = state === "PLANNED" ? "ok" : state === "STALE" ? "caution" : "neutral";
  query("#plan-source").textContent = record ? "weld planner" : "not generated";
  const detail = !record
    ? "Record waypoints or add welds, then Plan to generate and verify the motion."
    : provenance
      ? `${provenance.totalSampleCount} joint samples · ${provenance.totalDurationS.toFixed(1)} s · ` +
        `planned ${plannedAtLabel(provenance.plannedAt)} from program ${provenance.programDigest.slice(0, 12)}` +
        (provenance.current
          ? " · still the open program"
          : " · the program has been edited since; plan again before running it")
      : `Planned ${plannedAtLabel(record.createdAt)}, but its bytes are not in this browser's store. Plan again to rebuild it.`;
  query("#plan-detail").textContent = detail;
  query("#plan-state").title = detail;
}

/** "3 minutes ago", falling back to the stamp itself. The operator's question
 *  at the viewer is how old this is, not what o'clock it was. */
function plannedAtLabel(iso: string): string {
  const at = Date.parse(iso);
  if (!Number.isFinite(at)) return iso;
  const seconds = Math.max(0, Math.round((Date.now() - at) / 1000));
  if (seconds < 90) return `${seconds}s ago`;
  const minutes = Math.round(seconds / 60);
  if (minutes < 90) return `${minutes} min ago`;
  const hours = Math.round(minutes / 60);
  if (hours < 36) return `${hours} h ago`;
  return new Date(at).toLocaleString();
}

function render(): void {
  railsMode.syncProgramWelds();
  rememberCurrentProject(project.identity.id);
  query<HTMLInputElement>("#project-name").value = project.identity.name;
  query("#program-title").textContent = project.identity.name;
  const modelSelect = query<HTMLSelectElement>("#robot-model");
  modelSelect.value = project.robot.model;
  // Pinned by the cell: while connected to the cell the settings came
  // from, the model is the cell's to say. Pull again to refresh it.
  const pinnedBy = project.robot.cell && densePanel.connectedCell() === project.robot.cell.cell ? project.robot.cell.cell : null;
  modelSelect.disabled = pinnedBy !== null;
  modelSelect.title = pinnedBy ? `Set by ${pinnedBy}; pull the cell settings again to refresh` : "";
  renderCellSettings();
  renderCellBadge();
  renderPlanHistory();
  if (robotDialog.shown) renderRobotDialog();
  query<HTMLSelectElement>("#program-execution-mode").value =
    project.program.executionMode;
  query("#revision-label").textContent = `REV ${project.identity.revision}`;
  query("#row-count").textContent = `${project.nodes.length} nodes · ${project.rows.length} rows`;
  stageActions.undo.setEnabled(history.canUndo, "Nothing to undo");
  stageActions.redo.setEnabled(history.canRedo, "Nothing to redo");
  renderProgramTree();
  renderInspector();

  const planBadge = query("#plan-badge");
  planBadge.textContent = usesDensePlanner() ? (!project.denseTrajectory ? "NO PLAN" : railsSimulation.provenance?.current ? "PLANNED" : "STALE") : planLabel(project.plan.state);
  planBadge.className = `mode-badge plan-${project.plan.state}`;
  renderSessionStatusPills();
  if (usesDensePlanner()) {
    renderRailsPlanState();
  } else {
    renderClassicPlanState();
  }
  // What the transport can play. A rails program is never `server_validated`
  // -- that state means an accepted Tesseract plan, which rails no longer
  // produces -- so it answers the same question about its own trajectory.
  const canPlay = usesDensePlanner()
    ? railsSimulation.loaded
    : project.plan.state === "server_validated" &&
      project.plan.planner === "motion_server_tesseract" &&
      project.plan.acceptedPlan !== null &&
      project.plan.sourceProjectRevision === project.identity.revision;
  query<HTMLInputElement>("#timeline").disabled = !canPlay;
  query<HTMLButtonElement>("#play").disabled = !canPlay || jointMovePlanning;
  query<HTMLButtonElement>("#stop").disabled = usesDensePlanner()
    ? !railsSimulation.loaded
    : !playing;
  query<HTMLButtonElement>("#home-robot").disabled =
    jointMovePlanning || activeExecution !== null;
  updateStepControls(canPlay);
  // Each panel has its own transport: the preview's under Simulate, the
  // daemon's under Deploy. Only the panel that is up is on screen.
  densePanel.render();
  renderPoseSourceToggle();
  renderAuthoringMode();
  renderWorkpiecePlacement();
  renderWaypointWelds();
  adoptStoredWaypoints();
  renderProcessDefaults();
  renderProgramCatalog();
  renderProgramLint();
  renderProgramSpreadsheet();
  renderTargetAndReview();
  renderWeldDrawerPanel();
  updateTimeline(usesDensePlanner() ? railsSimulation.provenance?.totalDurationS ?? 0 : project.plan.trajectory.at(-1)?.timeS ?? 0);
}

async function syncViewerProject(): Promise<void> {
  ++executionTraceGeneration;
  viewer.setExecutionPaths([], []);
  executionPathPanel?.clear();
  void loadWorkcellFixtureMeshes();
  try {
    const snapshot = project;
    viewerProjectReady = syncProjectToWorkstation(viewer, snapshot, {
      showLabels: showPathLabels,
      workpieceMesh: currentWorkpieceMesh(),
      fixtureMeshes: workcellFixtureMeshes,
    });
    void plannedPathController.sync(snapshot);
    const outcome = await viewerProjectReady;
    if (snapshot !== project) return;
    syncEdgeOverlays();
    railsMode.syncPlacement();
    if (!outcome.ok) {
      setViewerStatus(`Model blocked: ${outcome.error ?? "unknown load error"}`);
    }
  } catch (error) {
    setViewerStatus(`Workstation update blocked: ${message(error)}`);
  }
}

/**
 * Push the catalog's world-space edges to the pickable layer. Rebuilding tube
 * geometry is the expensive part, so it only happens when the part or its
 * placement changes; selection state is cheap and always re-applied.
 */
function syncEdgeOverlays(): void {
  const fixture = project.fixtures[0];
  const key = [
    project.cad.fingerprint,
    project.edgeCatalog?.length ?? 0,
    fixture ? fixture.translationM.join(",") : "",
    fixture ? fixture.rotationRpyRad.join(",") : "",
  ].join("|");
  if (key !== edgeOverlaySyncKey) {
    edgeOverlaySyncKey = key;
    edgeSelection.setEdges(projectCadEdgeOverlays(project));
    pruneWeldDraft();
  }
  syncEdgeSelectionState();
}

/** Drop draft segments whose edge left the catalog (e.g. a new part import). */
function pruneWeldDraft(): void {
  if (!weldDraft) return;
  const known = new Set((project.edgeCatalog ?? []).map((edge) => edge.id));
  const segments = weldDraft.segments.filter((segment) => known.has(segment.edgeId));
  if (segments.length === weldDraft.segments.length) return;
  weldDraft = segments.length === 0 ? null : {
    ...weldDraft,
    segments,
    activeEdgeId: weldDraft.activeEdgeId && known.has(weldDraft.activeEdgeId)
      ? weldDraft.activeEdgeId
      : segments[0]!.edgeId,
  };
  renderWeldDrawerPanel();
}

function syncEdgeSelectionState(): void {
  const active = weldDraft?.segments.find(
    (segment) => segment.edgeId === weldDraft?.activeEdgeId,
  ) ?? null;
  const state = {
    // Rails authoring is a picking mode in itself: plain clicks must land on
    // seams without arming Select edges (or holding shift) first.
    mode: project.railsAuthoring ? railsMode.enabled : edgeSelectionMode,
    selectedIds: new Set(weldDraft?.segments.map((segment) => segment.edgeId) ?? []),
    activeId: weldDraft?.activeEdgeId ?? null,
    activeWindow: active
      ? { startS: active.startS, endS: active.endS }
      : null,
  };
  edgeSelection.setState(state);
}

/** The cached mesh for the part currently loaded, if it has been fetched. */
function currentWorkpieceMesh(): CadWorkpieceMesh | null {
  return workpieceMeshes.get(workpieceMeshKey(project)) ?? null;
}

/**
 * A tessellated mesh only exists for a real STEP fingerprint. The browser
 * fallback example project carries the literal fingerprint
 * "example:dog-box-v1", so this returns "" for it and no request is ever made.
 */
function workpieceMeshKey(candidate: OfflineProgrammingProject): string {
  const fingerprint = candidate.cad.fingerprint.trim().toLowerCase();
  return /^[a-f0-9]{64}$/.test(fingerprint) ? fingerprint : "";
}

/**
 * Fetch the workpiece surface for the loaded part, once per fingerprint.
 *
 * Every failure path is a one-line status and a viewport without a part — never
 * a console error and never a blocked import. Weld paths, placement, planning,
 * and playback do not depend on the mesh.
 */
async function loadWorkpieceMesh(source: CadSTEPSource | null): Promise<void> {
  const fingerprint = workpieceMeshKey(project);
  if (!source || fingerprint === "") return;
  if (workpieceMeshes.has(fingerprint) || workpieceMeshMisses.has(fingerprint)) return;
  if (workpieceMeshInFlight === fingerprint) return;
  workpieceMeshInFlight = fingerprint;
  try {
    const mesh = await cadClient.tessellate({ source });
    if (mesh.fingerprint !== fingerprint) {
      // The service answered for a different part than the one loaded.
      workpieceMeshMisses.add(fingerprint);
      return;
    }
    rememberWorkpieceMesh(fingerprint, mesh);
    if (workpieceMeshKey(project) === fingerprint) {
      setCADStatus(workpieceMeshSummary(mesh));
      void syncViewerProject();
    }
  } catch (error) {
    // Not an error state for the program: the part is simply not drawn.
    workpieceMeshMisses.add(fingerprint);
    setCADStatus(`Weld paths loaded · no 3D part surface (${message(error)})`);
  } finally {
    if (workpieceMeshInFlight === fingerprint) workpieceMeshInFlight = null;
  }
}

function rememberWorkpieceMesh(fingerprint: string, mesh: CadWorkpieceMesh): void {
  workpieceMeshes.set(fingerprint, mesh);
  while (workpieceMeshes.size > MAX_CACHED_WORKPIECE_MESHES) {
    const oldest = workpieceMeshes.keys().next();
    if (oldest.done || oldest.value === fingerprint) break;
    workpieceMeshes.delete(oldest.value);
  }
}

function renderSessionStatusPills(): void {
  const view = densePanel.flowView();
  const connected = view?.connected === true;
  const mode = query("#session-mode-badge");
  mode.textContent = !connected || view.posture === "simulation" ? "SIMULATION" : "LIVE";
  mode.className = "mode-badge " + (connected && view.posture !== "simulation" ? "is-real" : "is-offline");
  mode.title = connected ? view.postureLabel : "Browser preview only";
  const pill = query<HTMLButtonElement>("#drive-arm-badge");
  pill.dataset.armed = view?.arm.state ?? "disarmed";
  // FAULT is a latched rt-core fault or a blocking reason. A Stop or Disarm
  // result is the operator's own action, so the pill keeps the drives' word;
  // with no cell there is no drive to describe.
  const faulted = connected && !!view?.fault && isMachineFault(view.fault);
  const armLabel = !connected ? "NO CELL" : faulted ? "FAULT" : view?.arm.token ?? "DISARMED";
  // Preserve the mouse-down target through status polling. Replacing an
  // unchanged text node can cancel Safari's click on the lettering while
  // clicks on the button's padding (and accessibility activation) still work.
  if (pill.textContent !== armLabel) pill.textContent = armLabel;
  pill.className = "mode-badge " + (
    !connected || view?.arm.state === "unknown" ? "is-offline" : faulted ? "is-fault" : view?.arm.state === "armed" ? "is-armed" : "is-disarmed"
  );
  pill.disabled = !connected;
  pill.title = densePanel.armReason();
  const stop = query<HTMLButtonElement>("#cell-stop-pill");
  stop.hidden = !connected;
  stop.disabled = !view?.stopEnabled;
  const clear = query<HTMLButtonElement>("#cell-clear-faults");
  clear.hidden = !densePanel.clearFaultsAvailable();
  clear.disabled = densePanel.clearFaultsDisabledReason() !== null;
  const clearLabel = densePanel.clearingFaults() ? "Clearing…" : "Clear faults";
  if (clear.textContent !== clearLabel) clear.textContent = clearLabel;
  clear.title = densePanel.clearFaultsDisabledReason() ?? "Clear eligible faults. Does not arm, home or move the robot.";
  const clearResult = query<HTMLElement>("#cell-clear-faults-result");
  clearResult.hidden = !connected || !densePanel.clearFaultsMessage();
  const clearAnchor = clear.getBoundingClientRect();
  clearResult.style.top = clearAnchor.bottom + "px";
  clearResult.style.right = Math.max(0, window.innerWidth - clearAnchor.right) + "px";
  query("#cell-clear-faults-message").textContent = densePanel.clearFaultsMessage();
  const confirm = densePanel.armConfirmation();
  const popover = query<HTMLElement>("#cell-arm-popover");
  const anchor = pill.getBoundingClientRect();
  popover.style.top = anchor.bottom + "px";
  popover.style.right = Math.max(0, window.innerWidth - anchor.right) + "px";
  popover.hidden = !densePanel.armPopoverOpen();
  query("#cell-arm-question").textContent = "Arm " + (densePanel.activeCell() || "cell") + "?";
  query<HTMLButtonElement>("#cell-arm-confirm").hidden = !confirm;
  query("#cell-arm-reason").textContent = densePanel.armReason();
  if (popover.hidden) {
    armPopoverRelease?.();
    armPopoverRelease = null;
  } else if (!armPopoverRelease) {
    armPopoverRelease = armDismiss(popover, pill, () => densePanel.cancelArm());
  }
  const physical = connected && view.posture === "real" && view.arm.state === "armed" && view.canDrive;
  query("#cell-physical-play").textContent = "PHYSICAL PLAY: " + (physical ? "ENABLED" : "DISABLED");
  query("#cell-real-output").textContent = "REAL OUTPUT: " + (physical ? "ENABLED" : "DISABLED");
  query("#cell-network").textContent = "NETWORK: " + densePanel.networkSummary();
  const preview = query<HTMLButtonElement>("#sim-arm-toggle");
  const previewLabel = simArmed ? "Disarm preview" : "Arm preview";
  if (preview.textContent !== previewLabel) preview.textContent = previewLabel;
  preview.setAttribute("aria-pressed", String(simArmed));
  preview.hidden = connected;
}

function renderTargetAndReview(): void {
  renderSessionStatusPills();
  const blocked = targetState.blockedBy.length > 0
    ? ` · blocked: ${targetState.blockedBy.join(", ")}`
    : "";
  query("#target-state").textContent = usesDensePlanner() && targetState.mode === "local_simulator"
    ? `Native planner · continuous verification · ${
      targetState.phase.replaceAll("_", " ")}${blocked}`
    : `${targetState.target.label} · ${targetState.phase.replaceAll("_", " ")}${blocked}`;
  const badge = query("#target-badge");
  badge.textContent = targetState.mode === "offline_preview"
    ? "OFFLINE"
    : targetState.mode === "local_simulator"
      ? usesDensePlanner() ? "WELD PLANNER" : "TESSERACT"
      : "ROBOT TARGET";
  badge.className = `mode-badge ${
    targetState.mode === "offline_preview" ? "is-offline" : ""
  }`;
  const offlineSimulationButton = query<HTMLButtonElement>("#generate-preview");
  const physicalPlanningSelected =
    targetState.mode === "real_connected" &&
    project.program.executionMode === "production";
  offlineSimulationButton.hidden = physicalPlanningSelected;
  // A connected plan starts from the measured pose, read while the robot is still.
  const planMotion = usesDensePlanner() ? densePanel.planMotionBlocked() : "";
  offlineSimulationButton.disabled =
    offlineSimulationInFlight ||
    targetOperationInFlight ||
    activeExecution !== null ||
    planMotion !== "";
  // "Plan" alone in rails: planning and simulating are two buttons now,
  // because they were two trips to two different planners pretending to be
  // one action.
  //
  // The label names the action and never the state. While a plan runs the
  // button is only disabled; the status line says what is happening. A
  // button that read "Planning…" for the whole plan was a status display in
  // the wrong place.
  offlineSimulationButton.textContent = usesDensePlanner()
    ? projectWorkpieces(project).length > 1 ? "Plan active part" : "Plan"
    : "Plan & simulate";
  // The toolbar's Plan is a view of the same state as this button.
  stageActions.plan.label = offlineSimulationButton.textContent;
  stageActions.plan.setEnabled(
    !offlineSimulationButton.hidden && !offlineSimulationButton.disabled,
    physicalPlanningSelected
      ? "Physical planning goes through the selected robot (bottom bar)"
      : offlineSimulationInFlight
        ? "Planning"
        : activeExecution !== null
          ? "Locked while a robot execution is active"
          : planMotion || "Busy",
  );
  if (planMotion) offlineSimulationButton.title = planMotion;
  else if (offlineSimulationButton.title === planMotionInstruction("desktop")) offlineSimulationButton.title = "";
  const selectedConnectedTarget = targetState.mode === "real_connected"
    ? connectedTargets.find((candidate) =>
      candidate.profileId === targetState.target.profileId
    ) ?? null
    : null;
  const targetPlanButton = query<HTMLButtonElement>("#target-plan");
  targetPlanButton.hidden = !physicalPlanningSelected;
  targetPlanButton.disabled =
    targetOperationInFlight ||
    selectedConnectedTarget === null ||
    !selectedConnectedTarget.target.capabilities.planning ||
    project.program.executionMode !== "production" ||
    project.program.programState !== "saved_unplanned" ||
    !project.program.programDigest;
  targetPlanButton.title = selectedConnectedTarget === null
    ? "Select a manifest-backed Robot V4 target"
    : project.program.executionMode !== "production"
      ? "Select Production program mode"
      : project.program.programState !== "saved_unplanned" ||
          !project.program.programDigest
        ? "Save this exact shared program revision first"
        : "Ask the selected Robot V4 Motion Server to create the physical accepted plan";
  query("#local-simulator-status").textContent = localStatusLabel(localStatus);
  query("#connected-target-status").textContent = connectedStatusLabel();
  if (localPlannerError && project.plan.state !== "server_validated") {
    query("#plan-detail").textContent = `Simulation failed: ${localPlannerError}`;
    query("#plan-state").title = query("#plan-detail").textContent;
  }
  renderPlanEvidence();
  const chrome = query<HTMLElement>("#execution-chrome");
  const showExecutionChrome = executionChromeVisible();
  chrome.hidden = !showExecutionChrome;
  if (showExecutionChrome) {
    const localExecutionReasons = physicalExecutionPrerequisites();
    const failedServerChecks = serverExecutionReview?.checks
      .filter(({ passed }) => !passed)
      .map(({ code }) => code) ?? [];
    const reasons = [...new Set([
      ...localExecutionReasons,
      ...executionGatewayCapabilities.reasons,
      ...(serverExecutionReview?.reasons ?? []),
      ...failedServerChecks,
    ])];
    query("#execution-review-title").textContent =
      activeExecution
        ? "Robot execution active"
        : serverExecutionReview?.ready
          ? "Review approved · confirm Start"
          : localExecutionReasons.length === 0 &&
              executionGatewayCapabilities.infrastructureReady
            ? "Ready for server review"
            : "Execution review blocked";
    query("#execution-review-summary").textContent =
      activeExecution
        ? `${activeExecution.identity.planId} · controller lease and RTCore output monitored by the server`
        : `Robot-stored plan ${
          project.plan.acceptedPlan?.authority_classification === "physical_target"
            ? "selected"
            : "unavailable"
        } · live Motion Server/RTCore evidence ${
          connectedStatus?.statusObservation && connectedStatus.planObservation
            ? "available"
            : "unavailable"
        } · server execution ${
          executionGatewayCapabilities.infrastructureReady ? "ready" : "disabled"
        }${executionGatewayError ? " (gateway unavailable)" : ""} · ` +
          `tele-op ${TELEOP_CAPABILITY.enabled ? "available" : "disabled"}`;
    query("#execution-review").innerHTML = reasons.slice(0, 6)
      .map((reason) => `<li>${escapeHTML(reviewReason(reason))}</li>`)
      .join("");
    const reviewButton = query<HTMLButtonElement>("#review-execute");
    reviewButton.disabled =
      executionOperationInFlight ||
      activeExecution !== null ||
      localExecutionReasons.length > 0 ||
      !executionGatewayCapabilities.infrastructureReady;
    reviewButton.textContent = executionOperationInFlight
      ? "Checking live robot evidence…"
      : "Review physical start";
    const startButton = query<HTMLButtonElement>("#start-execution");
    startButton.disabled =
      executionOperationInFlight ||
      activeExecution !== null ||
      serverExecutionReview?.ready !== true ||
      serverExecutionReview.token === null ||
      serverExecutionReview.token.expiresAtMs <= Date.now();
    const stopButton = query<HTMLButtonElement>("#stop-execution");
    stopButton.hidden = activeExecution === null;
    stopButton.disabled = executionOperationInFlight || activeExecution === null;
  }
  query<HTMLSelectElement>("#target-mode").disabled =
    executionOperationInFlight || activeExecution !== null;
  query("#planner-status").textContent =
    `PLANNER: ${
      localPlannerError
        ? `BLOCKED · ${localPlannerError}`
        : planner.enabled
          ? "LOCAL READY"
          : localStatus?.availability.code.toUpperCase() ?? "UNAVAILABLE"
    } · ` +
    `TARGET PLAN: ${
      project.plan.acceptedPlan?.authority_classification === "physical_target"
        ? "ACCEPTED"
        : selectedConnectedTarget ? "AVAILABLE WHEN SAVED" : "NO TARGET"
    } · PHYSICAL PLAY: ${executionGatewayCapabilities.infrastructureReady ? "GATED" : "DISABLED"}`;
}

function executionChromeVisible(): boolean {
  return activeExecution !== null || (
    project.program.executionMode === "production" &&
    targetState.mode === "real_connected" &&
    executionGatewayCapabilities.infrastructureReady
  );
}

function physicalExecutionPrerequisites(): string[] {
  const reasons: string[] = [];
  const accepted = project.plan.acceptedPlan;
  const status = connectedStatus?.statusObservation;
  const runtimePlan = connectedStatus?.planObservation;
  if (targetState.mode !== "real_connected") {
    reasons.push("real_mode_required");
  }
  if (!accepted || accepted.authority_classification !== "physical_target") {
    reasons.push("physical_target_accepted_plan_required");
  }
  if (!project.plan.sourceProjectRevision ||
      project.plan.sourceProjectRevision !== project.identity.revision) {
    reasons.push("current_project_plan_required");
  }
  if (!status || !runtimePlan) {
    reasons.push("fresh_authoritative_status_required");
  } else if (accepted &&
      (runtimePlan.identity.planId !== accepted.plan_id ||
        runtimePlan.identity.planRevision !== accepted.plan_revision ||
        runtimePlan.identity.programId !== accepted.program_id ||
        runtimePlan.identity.programRevision !== accepted.program_revision ||
        runtimePlan.identity.robotModel !== accepted.target.robot_model)) {
    reasons.push("target_runtime_plan_identity_mismatch");
  }
  if (!accepted?.request_id) {
    reasons.push("accepted_plan_request_identity_missing");
  }
  return [...new Set(reasons)];
}

function executionReviewInputs(): {
  identity: GatewayExecutionIdentity;
  wire: unknown;
  statusSequence: number;
  runtimePlanSequence: number;
  evidenceRequestId: string;
} {
  const accepted = project.plan.acceptedPlan;
  const selected = targetState.mode === "real_connected"
    ? connectedTargets.find(({ profileId }) =>
      profileId === targetState.target.profileId
    ) ?? null
    : null;
  const status = connectedStatus?.statusObservation ?? null;
  const runtimePlan = connectedStatus?.planObservation ?? null;
  if (!accepted || !selected || !status || !runtimePlan ||
      accepted.authority_classification !== "physical_target" ||
      !accepted.request_id) {
    throw new Error("exact robot plan and live observations are required");
  }
  const identity: GatewayExecutionIdentity = {
    projectId: project.identity.id,
    projectRevision: project.identity.revision,
    programId: accepted.program_id,
    programRevision: accepted.program_revision,
    planId: accepted.plan_id,
    planRevision: accepted.plan_revision,
    programHash: runtimePlan.identity.programHash,
    programDigest: accepted.program_digest,
    trajectoryDigest: accepted.trajectory_digest,
    robotModel: accepted.target.robot_model as GatewayExecutionIdentity["robotModel"],
    target: {
      profileId: selected.profileId,
      manifestRef: selected.manifestRef,
      cellId: selected.cellId,
      manifestRevision: accepted.target.manifest_revision,
    },
  };
  const wire = {
    identity: executionIdentityWire(identity),
    accepted_plan: structuredClone(accepted),
  };
  return {
    identity,
    wire,
    statusSequence: status.sequence,
    runtimePlanSequence: runtimePlan.sequence,
    evidenceRequestId: accepted.request_id,
  };
}

function executionIdentityWire(
  identity: GatewayExecutionIdentity,
): Record<string, unknown> {
  return {
    project_id: identity.projectId,
    project_revision: identity.projectRevision,
    program_id: identity.programId,
    program_revision: identity.programRevision,
    plan_id: identity.planId,
    plan_revision: identity.planRevision,
    program_hash: identity.programHash,
    program_digest: identity.programDigest,
    trajectory_digest: identity.trajectoryDigest,
    robot_model: identity.robotModel,
    target: {
      profile_id: identity.target.profileId,
      manifest_ref: identity.target.manifestRef,
      cell_id: identity.target.cellId,
      manifest_revision: identity.target.manifestRevision,
    },
  };
}

async function reviewPhysicalStart(): Promise<void> {
  if (executionOperationInFlight || activeExecution) return;
  executionOperationInFlight = true;
  serverExecutionReview = null;
  serverExecutionReviewWire = null;
  renderTargetAndReview();
  try {
    const selected = targetState.mode === "real_connected"
      ? connectedTargets.find(({ profileId }) =>
        profileId === targetState.target.profileId
      ) ?? null
      : null;
    if (!selected) throw new Error("select a Robot V4 target first");
    [connectedStatus, connectedCapabilities, executionGatewayCapabilities] =
      await Promise.all([
        connectedClient.status(selected.profileId),
        connectedClient.capabilities(selected.profileId),
        executionGatewayClient.capabilities(),
      ]);
    const input = executionReviewInputs();
    const review = await executionGatewayClient.review(input.wire, {
      identity: input.identity,
      statusSequence: input.statusSequence,
      runtimePlanSequence: input.runtimePlanSequence,
      evidenceRequestId: input.evidenceRequestId,
    });
    if (!review.ready || !review.token) {
      throw new Error(review.reasons.join(", ") || "execution review denied");
    }
    serverExecutionReview = review;
    serverExecutionReviewWire = input.wire;
  } catch (error) {
    executionGatewayError = message(error);
    const summary = document.querySelector("#execution-review-summary");
    if (summary) {
      summary.textContent = `Execution review blocked: ${executionGatewayError}`;
    }
  } finally {
    executionOperationInFlight = false;
    renderTargetAndReview();
  }
}

async function startReviewedExecution(): Promise<void> {
  const review = serverExecutionReview;
  if (executionOperationInFlight || activeExecution ||
      !review?.ready || !review.token || !serverExecutionReviewWire) {
    return;
  }
  executionOperationInFlight = true;
  renderTargetAndReview();
  try {
    activeExecution = await executionGatewayClient.start(
      `start-${crypto.randomUUID()}`,
      review.token.tokenId,
      serverExecutionReviewWire,
      review.identity,
    );
    project.program.programState = "running";
    autosave.schedule(project);
    executionGatewayError = null;
    scheduleExecutionSessionCheck();
  } catch (error) {
    executionGatewayError = message(error);
    serverExecutionReview = null;
    serverExecutionReviewWire = null;
  } finally {
    executionOperationInFlight = false;
    renderTargetAndReview();
  }
}

async function stopActiveExecution(): Promise<void> {
  if (executionOperationInFlight || !activeExecution) return;
  executionOperationInFlight = true;
  const session = activeExecution;
  renderTargetAndReview();
  try {
    await executionGatewayClient.stop(
      session.sessionId,
      `stop-${crypto.randomUUID()}`,
    );
    activeExecution = null;
    serverExecutionReview = null;
    serverExecutionReviewWire = null;
    project.program.programState = "stopped";
    autosave.schedule(project);
  } catch (error) {
    executionGatewayError = message(error);
  } finally {
    executionOperationInFlight = false;
    renderTargetAndReview();
  }
}

function scheduleExecutionSessionCheck(): void {
  const sessionID = activeExecution?.sessionId;
  if (!sessionID) return;
  window.setTimeout(async () => {
    if (activeExecution?.sessionId !== sessionID) return;
    try {
      await executionGatewayClient.session(
        sessionID,
        activeExecution.identity,
      );
      scheduleExecutionSessionCheck();
    } catch (error) {
      activeExecution = null;
      serverExecutionReview = null;
      serverExecutionReviewWire = null;
      project.program.programState = "faulted";
      executionGatewayError =
        `Robot execution ended or lost supervision: ${message(error)}`;
      autosave.schedule(project);
      renderTargetAndReview();
    }
  }, 750);
}

async function refreshExecutionGatewayCapabilities(): Promise<void> {
  try {
    executionGatewayCapabilities = await executionGatewayClient.capabilities();
    executionGatewayError = null;
  } catch (error) {
    executionGatewayCapabilities = unavailableGatewayCapabilities(Date.now());
    executionGatewayError = message(error);
  }
  renderSessionStatusPills();
}

async function refreshLocalSimulatorStatus(): Promise<void> {
  try {
    localStatus = await planner.refresh();
    localStatusError = null;
  } catch (error) {
    localStatus = null;
    localStatusError = plannerStatusError(error);
    if (targetState.mode === "local_simulator") {
      applyLocalTargetState(localSimulatorTarget(), Date.now());
    }
    updateTargetOptions();
    renderTargetAndReview();
    return;
  }
  if (targetState.mode === "local_simulator") {
    applyLocalTargetState(localSimulatorTarget(), Date.now());
  }
  updateTargetOptions();
  renderTargetAndReview();
}

/** The Plan button. Which planner it reaches is program state, not a
 *  session choice: `project.railsAuthoring` is fixed at New program and a
 *  program authors welds ONE way, so the two paths never interleave. */
function usesDensePlanner(): boolean {
  return Boolean(project.railsAuthoring) || project.nodes.some(node => node.op === "move");
}

async function planProgram(): Promise<void> {
  if (offlineSimulationInFlight || targetOperationInFlight || activeExecution || stepImportInFlight || workpieceSwitchInFlight) return;
  if (usesDensePlanner()) {
    const motion = densePanel.planMotionBlocked();
    if (motion) {
      setStorageStatus(motion);
      return;
    }
    await planRailsProgram();
    return;
  }
  await planAndSimulateOffline();
}

/**
 * Plan a rails program: the weld planner's seam search, both trajopt stages
 * and the verifier, and the dense trajectory kept here.
 *
 * It stops there. It used to go on to compile rows and hand them to the
 * Motion Server's spreadsheet planner "for the evidence", which planned the
 * program a second time -- and worse than the first, because a `line_tcp`
 * row's supplied J1..J6 are discarded and IK re-solved from the interpolated
 * TCP pose, so the trajectory that got watched was not the trajectory that
 * would run. Simulate replays the planner's own bytes instead.
 *
 * The Tesseract discrete contact check went with it and is not missed: it
 * samples waypoints, and every segment here has already passed the weld
 * planner's T2 verifier, which proves clearance across whole intervals.
 */
async function planRailsProgram(fromProgramStart = false): Promise<void> {
  const cancellation = new AbortController();
  // A connected plan starts from the robot's measured pose unless the operator
  // chose the program's start; a failure then offers that alternative.
  const measuredStart = !fromProgramStart && densePanel.connectedCell() !== null;
  offlineSimulationInFlight = true;
  stopRailsPreview();
  // The status says a plan is running; the Plan button only greys out. The
  // progress dialog says where the wait is and for how long.
  setStorageStatus("Planning…");
  planProgress.start(projectWorkpieces(project).length > 1 ? `Planning ${project.cad.filename}` : "Planning", () => {
    setStorageStatus("Cancelling planning…");
    cancellation.abort();
  });
  renderTargetAndReview();
  try {
    const planned = await railsMode.planWeldTrajectory((stage) => {
      // Once the result is being committed, closing a request cannot undo it.
      if (stage.index >= 3) planProgress.setCancelAction(null);
      planProgress.stage(stage);
    }, cancellation.signal, { fromProgramStart });
    cancellation.signal.throwIfAborted();
    if (!planned) {
      showPlanningFailure(query("#cad-status").textContent ?? "The planner refused", measuredStart);
      return;
    }
    planProgress.stage({ index: 4, count: 4, label: "Loading the simulation" });
    const outcome = await loadRailsSimulation();
    if (outcome.state === "loaded") {
      const note = `${outcome.provenance.totalSampleCount} samples, ` +
        `${outcome.provenance.totalDurationS.toFixed(1)} s · ` +
        (fromProgramStart ? PLANNED_FROM_PROGRAM_START : "press play to simulate");
      setStorageStatus(`Planned · ${note}`);
      planProgress.finish(note);
    } else {
      planProgress.finish("");
    }
  } catch (error) {
    if (cancellation.signal.aborted) {
      setStorageStatus("Planning cancelled");
      planProgress.cancelled();
      return;
    }
    // `planWeldTrajectory` puts the reason in the rails status line, which is
    // in the viewport. Repeat it in the toolbar, next to the button that was
    // pressed -- that is where someone who just pressed Plan is looking.
    showPlanningFailure(error, measuredStart);
  } finally {
    offlineSimulationInFlight = false;
    render();
  }
}

function showPlanningFailure(error: unknown, measuredStart = false): void {
  const failure = planningFailure(error, "desktop", measuredStart);
  setStorageStatus(`Planning failed: ${failure.message}`);
  planProgress.fail(failure.message, {
    details: failure.details,
    action: failure.action?.key === "plan-from-program-start" ? { label: failure.action.label, run: () => {
      void planRailsProgram(true);
    } } : failure.action ? { label: failure.action.label, run: () => {
      setSelection({ kind: "workspace" });
      renderCellSettings();
      const settings = query<HTMLElement>("#cell-settings");
      settings.scrollIntoView({ behavior: "smooth", block: "start" });
      settings.querySelector<HTMLButtonElement>(".accept-robot-settings")?.focus({ preventScroll: true });
    } } : undefined,
  });
}

async function planAndSimulateOffline(): Promise<void> {
  const motionRows = project.rows.filter(
    (row) => row.enabled && (row.rowType ?? "motion") === "motion",
  );
  if (motionRows.length < 2) {
    setStorageStatus("Offline simulation needs at least two enabled motion rows.");
    query("#plan-detail").textContent =
      "Add another motion row, set the target poses, then plan again.";
    query("#plan-state").title = query("#plan-detail").textContent;
    return;
  }

  offlineSimulationInFlight = true;
  localPlanResult = null;
  localPlannerError = null;
  // The status says a plan is running; the Plan button only greys out.
  setStorageStatus("Planning & preparing simulation…");
  renderTargetAndReview();
  let previous: OfflineProgrammingProject | null = null;
  let startAcceptedPlayback = false;
  try {
    if (targetState.mode !== "local_simulator") {
      await selectTarget("local_simulator");
    }
    previous = project;
    localStatus = await planner.refresh();
    if (!localStatus.availability.available) {
      throw new Error(
        `canonical local planner unavailable: ${localStatus.availability.reason}`,
      );
    }
    if (!localSimulatorReady(localStatus)) {
      setStorageStatus("Starting the contained Robot V4 Motion Server/Tesseract planner…");
      localStatus = await planner.start();
      applyLocalTargetState(localSimulatorTarget(), Date.now());
    }
    if (!localSimulatorReady(localStatus)) {
      throw new Error(
        `safe local simulator readiness missing: ${localSafetyReasons(localStatus).join(",")}`,
      );
    }
    try {
      await planner.plan(project);
    } catch (error) {
      // "ready" is only the service's MEMORY of a past start: `Status()`
      // returns cached state without probing, so when the planner subprocess
      // dies the phase stays `ready` and the service wedges -- `plan` fails
      // with "persistent canonical planner is not started" and `start` is
      // refused with "cannot start from ready". Nothing recovers it, so every
      // Plan & simulate failed until the whole service was restarted by hand.
      //
      // A stop first is what clears it: measured on the wedged service,
      // stop -> start took 0.42 s and came back ready with a live runner.
      // Retried ONCE, and only for this error -- a second failure is a real
      // one and must reach the user.
      if (!/persistent canonical planner is not started/i.test(message(error))) {
        throw error;
      }
      setStorageStatus("The contained planner had exited — restarting it and planning again…");
      await planner.stop();
      localStatus = await planner.start();
      applyLocalTargetState(localSimulatorTarget(), Date.now());
      await planner.plan(project);
    }
    const result = planner.lastResult;
    if (!result) throw new Error("canonical planner returned no result");
    if (!result.acceptedPlan) {
      throw new Error("canonical planner returned no accepted trajectory artifact");
    }
    const trajectory = acceptedPlanTrajectory(project, result.acceptedPlan);
    localPlanResult = result;
    project = installServerValidatedPlan(project, {
      id: result.evidence.identity.planId,
      revision: result.evidence.identity.planRevision,
      createdAt: result.evidence.generatedAt,
      evidence: projectPlanEvidence(result),
      trajectory,
      acceptedPlan: result.acceptedPlan,
    });
    project = recordPlan(project);
    history.push(previous);
    changed("Accepted Tesseract joint trajectory installed · starting simulation");
    await localSimulation.load(trajectory);
    await refreshLocalSimulatorStatus();
    startAcceptedPlayback = true;
  } catch (error) {
    if (error instanceof LocalPlanRejectedError) {
      localPlanResult = error.result;
      project = installRejectedLocalPlan(project, error.result);
      if (previous) history.push(previous);
      changed(`Canonical plan rejected: ${error.reasons.join(", ")}`);
    } else {
      localPlannerError = message(error);
      setStorageStatus(`Simulation failed: ${localPlannerError}`);
    }
  } finally {
    offlineSimulationInFlight = false;
    renderTargetAndReview();
  }
  if (startAcceptedPlayback) {
    setStorageStatus("Accepted trajectory ready · moving safely to the first planned joint position");
    await startPlayback();
  }
}

async function planOnSelectedRobot(): Promise<void> {
  if (targetOperationInFlight || targetState.mode !== "real_connected") return;
  const selected = connectedTargets.find(
    (candidate) => candidate.profileId === targetState.target.profileId,
  );
  if (!selected) return;
  targetOperationInFlight = true;
  renderTargetAndReview();
  const previous = project;
  try {
    const accepted = await targetPlanner.plan(project, selected);
    const trajectory = acceptedPlanTrajectory(project, accepted);
    project = installServerValidatedPlan(project, {
      id: accepted.plan_id,
      revision: accepted.plan_revision,
      createdAt: accepted.created_at,
      evidence: {
        collision: accepted.evidence.collision,
        singularity: accepted.evidence.singularity,
        limits: accepted.evidence.limits,
        startPose: accepted.evidence.start_pose ?? "unavailable",
        runtimePlayable: accepted.evidence.runtime_playable,
        collisionPoints: [],
        singularityPoints: [],
      },
      trajectory,
      acceptedPlan: accepted,
    });
    project.program.programState = "accepted_physical";
    project = recordPlan(project);
    localPlanResult = null;
    history.push(previous);
    changed(
      `Physical plan accepted by ${accepted.target.target_id} · ${accepted.plan_id}`,
    );
    playbackCursorS = 0;
    updateTimeline(trajectory.at(-1)?.timeS ?? 0);
  } catch (error) {
    query("#planner-status").textContent =
      `TARGET PLAN: BLOCKED · ${message(error)}`;
  } finally {
    targetOperationInFlight = false;
    renderTargetAndReview();
  }
}

async function discoverConnectedTargets(): Promise<void> {
  try {
    const discovery = await connectedClient.discover();
    connectedTargets = discovery.targets;
    connectedDiscoveryError = null;
    if (
      connectedStatus &&
      !connectedTargets.some((target) =>
        target.profileId === connectedStatus?.target.profileId
      )
    ) {
      connectedStatus = null;
      connectedCapabilities = null;
    }
    updateTargetOptions();
    query("#connected-target-status").textContent = connectedTargets.length > 0
      ? `${connectedTargets.length} manifest-backed target${
        connectedTargets.length === 1 ? "" : "s"
      } · read-only status · server-mediated target planning`
      : "No valid manifest-backed connected targets.";
  } catch (error) {
    connectedTargets = [];
    connectedDiscoveryError = message(error);
    connectedStatus = null;
    connectedCapabilities = null;
    updateTargetOptions();
    query("#connected-target-status").textContent =
      `Connected discovery unavailable: ${connectedDiscoveryError}`;
  }
  renderSessionStatusPills();
}

async function readConnectedTarget(
  discovery: ConnectedTargetDiscoveryDescriptor,
): Promise<void> {
  if (targetOperationInFlight) return;
  targetOperationInFlight = true;
  renderTargetAndReview();
  try {
    [connectedStatus, connectedCapabilities] = await Promise.all([
      connectedClient.status(discovery.profileId),
      connectedClient.capabilities(discovery.profileId),
    ]);
    targetState = reduceTargetState(targetState, {
      type: "disconnect",
      atMs: Date.now(),
      reason: connectedStatus.reasons[0] ?? "connected_read_only",
    });
  } catch (error) {
    connectedStatus = null;
    connectedCapabilities = null;
    targetState = reduceTargetState(targetState, {
      type: "disconnect",
      atMs: Date.now(),
      reason: "connected_status_unavailable",
    });
    query("#connected-target-status").textContent =
      `Connected read blocked: ${message(error)}`;
  } finally {
    targetOperationInFlight = false;
    renderTargetAndReview();
  }
}

function applyLocalTargetState(target: TargetDescriptor, now: number): void {
  if (localSimulatorReady(localStatus, now) && localStatus) {
    targetState = reduceTargetState(targetState, {
      type: "select_local_simulator",
      atMs: now,
      target,
      proof: {
        profileId: localStatus.profileId,
        statusSequence: localStatus.statusSequence,
        observedAtMs: localStatus.safety.observedAtMs,
        freshUntilMs: localStatus.safety.freshUntilMs,
        noRealMovement: localStatus.safety.noRealMovement,
        ethercatEnabled: localStatus.safety.ethercatOutputEnabled,
        motionOutputEnabled: localStatus.safety.motionOutputEnabled,
      },
    });
    return;
  }
  targetState = reduceTargetState(targetState, {
    type: "select_local_simulator_unavailable",
    atMs: now,
    target,
    reason: localStatus?.availability.code ?? "local_simulator_status_unavailable",
  });
}

function setProjectTarget(
  mode: OfflineProgrammingProject["target"]["mode"],
  profileId: string | null,
  manifestRef: string | null,
): void {
  if (
    project.target.mode === mode &&
    project.target.profileId === profileId &&
    project.target.manifestRef === manifestRef
  ) {
    return;
  }
  mutate("target", (draft) => {
    draft.target = { mode, profileId, manifestRef };
  });
}

function applyDefaultPlannerSelection(): void {
  if (project.target.mode === "real_connected") return;
  const target = localSimulatorTarget();
  applyLocalTargetState(target, Date.now());
  project.target = {
    mode: "local_simulator",
    profileId: target.profileId,
    manifestRef: null,
  };
}

function updateTargetOptions(): void {
  const select = query<HTMLSelectElement>("#target-mode");
  // In a rails program the weld planner IS the motion planner — its search
  // and trajopt author the joints at Compile to rows, and the canonical
  // local simulator replays them as the safety check. The selector says so,
  // and says it exclusively: the classic planners would re-plan rows the
  // rails compile owns.
  //
  // The weld planner appears once per FREE-SPACE SOLVER, because that is the
  // only thing left to choose about a rails program's motion: the weld passes
  // themselves are the search's, identical whichever of these is picked, and
  // what differs is the solver that plans the approach, transits and retract.
  // Three planner entries and not a planner plus a second dropdown — a rails
  // program's planner IS the weld planner, so the choice belongs where
  // planners are chosen.
  const rails = usesDensePlanner();
  const selected = rails
    ? `weld_planner:${railsMode.freeSpaceBackend}`
    : targetState.mode === "real_connected"
      ? `connected:${targetState.target.profileId}`
      : targetState.mode;
  const weldPlannerHint = rails
    ? "The native planner verifies the continuous motion; Simulate replays the verified trajectory"
    : "rails programs only — every New program is one; import a part and the weld planner takes over";
  const freeSpaceSolvers: readonly (readonly [FreeSpaceBackend, string])[] = [
    ["bspline", "B-spline"],
    ["curobo", "cuRobo"],
    ["legacy", "Catmull-Rom (deprecated)"],
  ];
  select.innerHTML = `
    <option value="offline_preview" ${rails ? "disabled" : ""}>None (browser-only, no planner)</option>
    <option value="local_simulator" ${rails ? "disabled" : ""}>${escapeHTML(tesseractPlannerOptionLabel())}</option>
    ${freeSpaceSolvers.map(([name, label]) => `
      <option value="weld_planner:${name}" ${rails ? "" : "disabled"} title="${
        escapeHTML(weldPlannerHint)
      }">Weld planner · ${escapeHTML(label)} free space</option>
    `).join("")}
    ${connectedTargets.map((target) => `
      <option value="connected:${escapeHTML(target.profileId)}" ${rails ? "disabled" : ""}>${escapeHTML(target.cellId)} · target planning</option>
    `).join("")}
  `;
  if ([...select.options].some((option) => option.value === selected && !option.disabled)) {
    select.value = selected;
  }
}

function tesseractPlannerOptionLabel(): string {
  if (localStatus?.availability.available) return "Tesseract";
  if (!localStatus && localStatusError === null) return "Tesseract · checking";
  return "Tesseract · unavailable";
}

function plannerStatusError(error: unknown): string {
  const text = message(error);
  if (/Failed to execute 'json' on 'Response'/i.test(text) || /invalid JSON/i.test(text)) {
    return "planner status is unavailable";
  }
  return text;
}

function localStatusLabel(status: LocalSimulatorStatus | null): string {
  if (!status) {
    return localStatusError
      ? `Tesseract unavailable · ${localStatusError}`
      : "Tesseract status not loaded.";
  }
  if (!status.availability.available) {
    return `Tesseract unavailable · ${status.availability.code.replaceAll("_", " ")} · ${
      status.availability.reason
    }`;
  }
  if (localSimulatorReady(status)) {
    return "Ready · PC-native · loopback only · simulation · no real movement · EtherCAT/motion outputs off";
  }
  return status.phase === "starting"
    ? "Starting the local Motion Server/Tesseract simulator…"
    : `${status.phase} · Plan & simulate will recover and start it automatically · ${
      status.readinessReasons.join(", ") || "runtime restart required"
    }`;
}

function connectedStatusLabel(): string {
  if (!connectedStatus) {
    if (connectedDiscoveryError) {
      return `Connected discovery unavailable: ${connectedDiscoveryError}`;
    }
    return connectedTargets.length > 0
      ? `${connectedTargets.length} manifest-backed targets · status is read-only; planning is server-mediated`
      : "No manifest-backed Robot V4 targets are available.";
  }
  return `${connectedStatus.target.cellId} · ${connectedStatus.state.replaceAll("_", " ")} · ` +
    `status ${connectedStatus.statusAvailable ? "fresh" : "unavailable"} · ` +
    `plan ${connectedStatus.planAvailable ? "fresh" : "unavailable"} · ` +
      `physical start ${
        executionGatewayCapabilities.infrastructureReady
          ? "available through reviewed server flow"
          : "disabled"
      } · tele-op disabled`;
}

function renderPlanEvidence(): void {
  const host = query("#plan-evidence");
  if (localPlannerError) {
    host.innerHTML = `<span class="is-error"><strong>Simulation failed</strong>${
      escapeHTML(localPlannerError)
    }</span>`;
    return;
  }
  if (!localPlanResult) {
    const accepted = project.plan.acceptedPlan;
    host.innerHTML = accepted
      ? [
        `<span><strong>Authority</strong>${escapeHTML(
          accepted.authority_classification.replaceAll("_", " "),
        )}</span>`,
        `<span><strong>Collision</strong>${escapeHTML(accepted.evidence.collision)}</span>`,
        `<span><strong>Singularity</strong>${escapeHTML(accepted.evidence.singularity)}</span>`,
        `<span><strong>Limits</strong>${escapeHTML(accepted.evidence.limits)}</span>`,
        `<span><strong>Accepted plan</strong>${escapeHTML(accepted.plan_id)} · r${
          accepted.plan_revision
        }</span>`,
        `<span><strong>Trajectory digest</strong>${escapeHTML(
          accepted.trajectory_digest.slice(0, 30),
        )}…</span>`,
        `<span><strong>Planner build</strong>${escapeHTML(
          accepted.planner.build_digest.slice(0, 30),
        )}…</span>`,
      ].join("")
      : "<span>Planner evidence unavailable</span>";
    return;
  }
  const evidence = localPlanResult.evidence;
  const accepted = localPlanResult.acceptedPlan;
  const evidenceItem = (label: string, state: string, count: number): string =>
    `<span><strong>${escapeHTML(label)}</strong>${escapeHTML(state)} · ${count}</span>`;
  host.innerHTML = [
    evidenceItem("Collision", evidence.collision.state, evidence.collision.points.length),
    evidenceItem("Singularity", evidence.singularity.state, evidence.singularity.points.length),
    evidenceItem("Limits", evidence.limits.state, evidence.limits.points.length),
    evidenceItem("Runtime", evidence.runtime.playable ? "playable in simulation" : "blocked", evidence.sampleCount),
    `<span><strong>Correlation</strong>${
      evidence.correlationReasons.length === 0
        ? "revision matched"
        : escapeHTML(evidence.correlationReasons.join(", "))
    }</span>`,
    `<span><strong>Accepted artifact</strong>${
      accepted
        ? `${escapeHTML(accepted.authority_classification.replaceAll("_", " "))} · ${
          escapeHTML(accepted.trajectory_digest.slice(0, 23))
        }…`
        : "not minted"
    }</span>`,
    `<span><strong>Planner build</strong>${
      accepted
        ? `${escapeHTML(accepted.planner.backend)} · ${
          escapeHTML(accepted.planner.build_digest.slice(0, 23))
        }…`
        : "unavailable"
    }</span>`,
  ].join("");
}

function projectPlanEvidence(result: LocalPlanResult): PlanEvidence {
  const evidence = result.evidence;
  return {
    collision: evidence.collision.state === "clear"
      ? "clear"
      : evidence.collision.points.length > 0 ? "collision" : "unavailable",
    singularity: evidence.singularity.state === "clear"
      ? "clear"
      : evidence.singularity.points.some((point) => point.state === "hard_stop")
        ? "hard_stop"
        : evidence.singularity.points.length > 0 ? "warning" : "unavailable",
    limits: evidence.limits.state === "clear"
      ? "clear"
      : evidence.limits.points.length > 0 ? "outside_limits" : "unavailable",
    startPose: "unavailable",
    runtimePlayable: evidence.runtime.playable,
    collisionPoints: evidence.collision.points.map((point) => ({
      waypointIndex: point.waypointIndex,
      linkA: point.linkA,
      linkB: point.linkB,
      depthM: point.depthM,
      affectedJoints: [...point.affectedJoints],
    })),
    singularityPoints: evidence.singularity.points.map((point) => ({
      waypointIndex: point.waypointIndex,
      state: point.state,
      appliedVelocityScale: point.appliedVelocityScale,
      affectedJoints: [...point.affectedJoints],
    })),
  };
}

function installRejectedLocalPlan(
  current: OfflineProgrammingProject,
  result: LocalPlanResult,
): OfflineProgrammingProject {
  const next = installServerValidatedPlan(current, {
    id: result.evidence.identity.planId,
    revision: result.evidence.identity.planRevision,
    createdAt: result.evidence.generatedAt,
    evidence: {
      ...projectPlanEvidence(result),
      runtimePlayable: false,
    },
    trajectory: [],
  });
  next.plan.state = "rejected";
  return next;
}

function setViewerStatus(value: string): void {
  setBusyText(query("#viewer-status"), value);
  renderLivePoseCaption();
  viewerLogger.at(levelOfStatus(value), value);
}

function applyCameraPaneVisibility(): void {
  const stage = query("#workstation-stage");
  stage.dataset.j6Cam = j6CamVisible ? "visible" : "hidden";
  stage.dataset.tcpCam = tcpCamVisible ? "visible" : "hidden";
  viewer.setPaneVisibility({ j6: j6CamVisible, tcp: tcpCamVisible });
  viewer.resize();
}

function localSimulatorTarget(): TargetDescriptor {
  return {
    schema: TARGET_DESCRIPTOR_SCHEMA,
    profileId: "local-simulator",
    label: "Tesseract",
    kind: "local_simulator",
    manifestRef: null,
    manifestSchema: null,
    cellId: null,
    axes: [],
    capabilities: {
      monitoring: true,
      planning: true,
      simulation: true,
      realExecution: false,
      teleop: false,
      rawBrowserTransport: false,
    },
  };
}

function reviewReason(value: string): string {
  const labels: Record<string, string> = {
    real_mode_required: "Real connected mode is not selected",
    manifest_backed_target_required: "No manifest-backed target is available",
    server_validated_plan_required: "Authoritative motion-server plan evidence is missing",
    collision_evidence_clear: "Collision evidence is unavailable",
    singularity_evidence_clear: "Singularity evidence is unavailable",
    limit_evidence_clear: "Joint-limit evidence is unavailable",
    fresh_runtime_status_required: "Fresh server runtime status is missing",
    fresh_server_authority_required: "Server lease/fence authority is missing",
    physical_target_accepted_plan_required: "Plan on the selected robot first",
    current_project_plan_required: "The plan does not match the current project revision",
    target_runtime_plan_identity_mismatch: "The robot reports a different accepted plan",
    accepted_plan_request_identity_missing: "The robot plan is missing its planning request identity",
    active_session_monitoring_unavailable: "Continuous robot monitoring is unavailable",
    fresh_authoritative_status_required: "Fresh Motion Server and RTCore evidence is unavailable",
  };
  return labels[value] ?? value.replaceAll("_", " ");
}

/** What the placement offsets are measured from, in the operator's words.
 *  A cell with no positioner has no table and saying otherwise is how the
 *  offsets came to be typed against a frame 1.913 m from the one that read
 *  them. */
function workFrameLabel(): string {
  return cellWorkFrame(project.robot.model).id === "world"
    ? "the cell origin"
    : "table centre";
}

/** The part gizmo: the placement triad in the viewport, arrows to move and
 *  rings to turn. Clicking the active button turns it off. Its result
 *  arrives as a viewer event once per drag, so the placement fields and the
 *  derived rows follow the release, not the pointer. */
function setPartGizmo(enabled: boolean): void {
  if (enabled && !viewer.workpieceGizmoEnabled) {
    const current = workpiecePlacement(project);
    placementBeforeMove = current
      ? {
        offsetFromWorkFrameM: [...current.offsetFromWorkFrameM] as [number, number, number],
        rotationRpyRad: [...current.rotationRpyRad] as [number, number, number],
      }
      : null;
  }
  if (!enabled) placementBeforeMove = null;
  viewer.setWorkpieceGizmo(enabled);
  stageActions.movePart.checked = enabled;
  // The tool's dialog takes the weld drawer's corner while the tool is on;
  // one tool card at a time, like tabs.
  if (enabled) {
    if (railsWeldsCard.shown) setRailsWeldsOpen(false);
    if (waypointWeldsCard.shown) setWaypointWeldsOpen(false);
    if (weldPresetsCard.shown) setWeldPresetsOpen(false);
    if (planPresetsCard.shown) setPlanPresetsOpen(false);
    setSelection({ kind: "part" });
    placementCard.show();
  } else {
    placementCard.hide();
  }
  query("#weld-drawer").classList.toggle("is-yielded", enabled);
  if (enabled) renderWorkpiecePlacement();
}

/**
 * One selection for the whole app: the tree, the inspector, the spreadsheet
 * and the tool cards are views of it. A weld selected in the tree is the
 * node selected in the spreadsheet and the reverse; the part selected is
 * what Move part moves.
 */
type Selection =
  | { kind: "program" }
  | { kind: "workspace" }
  | { kind: "part" }
  | { kind: "fixture"; id: string }
  | { kind: "weld"; id: string };

function treeIdOf(sel: Selection): string {
  switch (sel.kind) {
    case "program": return TREE_PROGRAM;
    case "workspace": return TREE_WORKSPACE;
    case "part": return TREE_PART;
    case "fixture": return TREE_FIXTURE_PREFIX + sel.id;
    case "weld": return sel.id;
  }
}

function selectionForTreeId(id: string): Selection {
  if (id === TREE_PROGRAM) return { kind: "program" };
  if (id === TREE_WORKSPACE) return { kind: "workspace" };
  if (id === TREE_PART) return { kind: "part" };
  if (id.startsWith(TREE_FIXTURE_PREFIX)) return { kind: "fixture", id: id.slice(TREE_FIXTURE_PREFIX.length) };
  return { kind: "weld", id };
}

/** Tree ids for inactive parts carry their owner, since different STEP
 * files can legitimately contain welds with the same local seam id. */
async function selectTreeItem(id: string): Promise<boolean> {
  let partId: string | null = null;
  let weldId: string | null = null;
  if (id.startsWith(TREE_WORKPIECE_PREFIX)) partId = id.slice(TREE_WORKPIECE_PREFIX.length);
  if (id.startsWith(TREE_WORKPIECE_WELD_PREFIX)) {
    const [owner, weld] = id.slice(TREE_WORKPIECE_WELD_PREFIX.length).split("/");
    partId = decodeURIComponent(owner!);
    weldId = decodeURIComponent(weld!);
  }
  if (partId && partId !== activeWorkpieceId(project)) {
    if (!await selectWorkpiece(partId)) return false;
  }
  setSelection(partId ? weldId ? { kind: "weld", id: weldId } : { kind: "part" } : selectionForTreeId(id), "tree");
  return true;
}

async function selectWorkpiece(id: string): Promise<boolean> {
  const lock = editingLockReason();
  if (lock || stepImportInFlight || workpieceSwitchInFlight) {
    setStorageStatus(lock ?? "Wait for the current part to finish loading");
    programTree.select(treeIdOf(selection));
    return false;
  }
  const part = projectWorkpieces(project).find(candidate => candidate.id === id);
  if (!part) return false;
  workpieceSwitchInFlight = true;
  const projectIdBeforeRead = project.identity.id;
  setStorageStatus(`Opening ${part.cad.filename}…`);
  try {
    const source = await repository.getSource(part.cad.fingerprint);
    if (project.identity.id !== projectIdBeforeRead) return false;
    if (!source) throw new Error(`The STEP for ${part.cad.filename} is missing from this browser`);
    if (viewer.workpieceGizmoEnabled) setPartGizmo(false);
    if (railsWeldsCard.shown) setRailsWeldsOpen(false);
    if (waypointWeldsCard.shown) setWaypointWeldsOpen(false);
    railsMode.reset();
    activeCADSource = { kind: "bytes", filename: source.filename, bytes: source.bytes };
    mutate("cad", draft => { activateWorkpiece(draft, id); });
    resetWeldAuthoringUiState();
    syncRailsModeToProject();
    await Promise.all([loadWorkpieceMesh(activeCADSource), railsStartup]);
    setStorageStatus(`${part.cad.filename} is active for editing and planning`);
    return true;
  } catch (error) {
    setStorageStatus(`Could not open ${part.cad.filename}: ${message(error)}`);
    return false;
  } finally {
    workpieceSwitchInFlight = false;
  }
}

function setSelection(next: Selection, source: "tree" | "sheet" | "app" = "app"): void {
  const before = selection;
  selection = next;
  const landed = next.kind === "weld" && !(before.kind === "weld" && before.id === next.id);
  syncWeldHighlight(landed);
  programTree.select(treeIdOf(next));
  renderInspector();
  if (next.kind === "weld" && source !== "sheet" && !selectionSyncing) {
    selectionSyncing = true;
    try {
      selectNode(next.id);
    } finally {
      selectionSyncing = false;
    }
  }
}

/** Light the selected weld; `flash` blinks it first (the selection just
 *  landed on it). Called on every selection change and render. */
function syncWeldHighlight(flash = false): void {
  const node = selection.kind === "weld" ? nodeById(project.nodes, selection.id) : null;
  if (!flash && node === weldHighlighted) return;
  weldHighlighted = node;
  weldHighlight.show(node ? weldPathPoints(node) : null, flash);
}

function renderProgramTree(): void {
  const treeProject = densePanel.targetSelected() ? { ...project, robot: { ...project.robot, model: densePanel.cellModel() as RobotModel } } : project;
  programTree.render(programTreeNodes(treeProject, {
    // The link says "Import a part": it opens the file picker, as From file
    // does, rather than the menu the toolbar button opens.
    onImportPart: () => chooseStepFiles("weld"),
    importingFilename: importingPartFilename,
    importingWorkpieceId,
    importingKind,
    onAddStand: addStand,
    demoFilenames: new Set(stepExamples.map((example) => example.filename)),
  }));
  // A weld that left the program (undo, delete) cannot stay selected.
  if (selection.kind === "weld" && !nodeById(project.nodes, selection.id)) selection = { kind: "part" };
  // Nor a stand that was removed.
  if (selection.kind === "fixture" && !fixtureById(project, selection.id)) selection = { kind: "workspace" };
  programTree.select(treeIdOf(selection));
  renderStandCard();
  syncWeldHighlight();
}

function renderInspector(): void {
  const sel = selection;
  if (sel.kind === "program") {
    inspector.show("program", project.identity.name || "Untitled program", "PROGRAM");
    return;
  }
  if (sel.kind === "workspace") {
    inspector.show("workspace", "Workspace", "CELL");
    return;
  }
  if (sel.kind === "fixture") {
    const fixture = fixtureById(project, sel.id);
    const pane = inspector.panes.fixture;
    pane.replaceChildren();
    if (!fixture) {
      inspector.show("fixture", "No stand", "STAND");
      return;
    }
    inspector.show("fixture", fixture.name, fixture.origin === "cell-default" ? "STAND · PLACEHOLDER" : "STAND");
    pane.append(summaryRow("Shape", fixture.shape.kind === "box" ? "Box" : "STEP"));
    pane.append(summaryRow(fixture.shape.kind === "box" ? "Size" : "File", fixtureShapeLabel(fixture)));
    pane.append(summaryRow("Position", fixturePoseLabel(fixture).replace(/^at /, "")));
    pane.append(summaryRow("Rotation", fixture.rotationRpyRad.map((v) => `${(v * 180 / Math.PI).toFixed(1)}°`).join("  ")));
    pane.append(summaryRow("From", "the cell origin"));
    pane.append(summaryRow("Source", fixture.origin === "cell-default"
      ? "The cell's placeholder: nobody has measured the real stand"
      : "Set here"));
    const note = element("p", "inspector-empty", "The weld planner clears this body with every plan, so what is drawn here is what the arm avoids.");
    pane.append(note);
    const edit = element("button", "mini-button primary inspector-action", "EDIT STAND");
    edit.type = "button";
    edit.addEventListener("click", () => openStandCard(fixture.id));
    pane.append(edit);
    return;
  }
  if (sel.kind === "part") {
    const pane = inspector.panes.part;
    pane.replaceChildren();
    if (!hasWorkpieceGeometry(project)) {
      inspector.show("part", "No part", "PART");
      pane.append(element("p", "inspector-empty", "Import a STEP file to place its weld paths in the cell."));
      return;
    }
    const isDemo = project.cad.kind === "example" ||
      stepExamples.some((example) => example.filename === project.cad.filename);
    inspector.show("part", project.cad.filename || "Part", isDemo ? "DEMO WELD PART" : "WELD PART");
    pane.append(element("p", "inspector-note", "Active for weld editing and planning. Other parts remain in the scene and are checked for collisions."));
    const placement = workpiecePlacement(project);
    pane.append(summaryRow("Source", isDemo ? "Bundled demo file" : "Imported STEP"));
    pane.append(summaryRow("Edges", String(project.edgeCatalog?.length ?? 0)));
    if (placement) {
      const mm = (v: number) => `${(v * 1_000).toFixed(1)} mm`;
      const deg = (v: number) => `${(v * 180 / Math.PI).toFixed(1)}°`;
      pane.append(summaryRow("Offset", placement.offsetFromWorkFrameM.map(mm).join("  ")));
      pane.append(summaryRow("Rotation", placement.rotationRpyRad.map(deg).join("  ")));
      pane.append(summaryRow("From", workFrameLabel()));
    }
    const move = element("button", "mini-button primary inspector-action", "MOVE PART");
    move.type = "button";
    move.disabled = !stageActions.movePart.enabled;
    move.title = stageActions.movePart.title;
    move.addEventListener("click", launchPartMove);
    pane.append(move);
    return;
  }
  const node = nodeById(project.nodes, sel.id);
  const pane = inspector.panes.weld;
  pane.replaceChildren();
  if (!node || node.op !== "weld") {
    inspector.show("none", "Nothing selected");
    return;
  }
  inspector.show("weld", node.label || node.id, "WELD");
  pane.append(summaryRow("Seam", node.id));
  pane.append(summaryRow("Enabled", node.enabled ? "yes" : "no"));
  if (node.comment) pane.append(summaryRow("Comment", node.comment));
  const rows = rowsOfNode(project, node.id);
  pane.append(summaryRow("Rows", rows.length ? `${rows.length} compiled` : "not compiled"));
  if (isManualPoseWeld(node)) {
    const open = element("button", "mini-button primary inspector-action", "OPEN IN WAYPOINT WELDS");
    open.type = "button";
    open.disabled = !stageActions.waypointWelds.enabled;
    open.title = stageActions.waypointWelds.disabledReason ?? "";
    open.addEventListener("click", () => openWeldInWaypoints(node.id));
    open.textContent = `OPEN IN ${weldToolActions[waypointWeldOf(node).tool].label.toUpperCase()}`;
    pane.append(open);
  } else if (railsMode.enabled) {
    const open = element("button", "mini-button primary inspector-action", "OPEN IN RAILS WELDS");
    open.type = "button";
    open.addEventListener("click", () => openWeldInRails(node.id));
    pane.append(open);
  }
}

/** The commands a right-click on the selection offers. Built fresh each
 *  time: they read the lock and the node's state at that moment. */
function contextActionsFor(sel: Selection): Action[] {
  const lock = editingLockReason();
  if (sel.kind === "fixture") {
    const edit = new Action({ id: "ctx-edit-fixture", label: "Edit fixture", icon: Move3d });
    edit.setEnabled(lock === null, lock ?? "");
    edit.triggered.connect(() => openStandCard(sel.id));
    const remove = new Action({ id: "ctx-remove-fixture", label: "Remove fixture", icon: Trash2 });
    remove.setEnabled(lock === null, lock ?? "");
    remove.triggered.connect(() => {
      if (standEditing === sel.id) closeStandCard(false);
      mutate("workcell", draft => { draft.workcellFixtures = draft.workcellFixtures.filter(fixture => fixture.id !== sel.id); });
      setSelection({ kind: "workspace" });
    });
    return [edit, remove];
  }
  if (sel.kind === "part") {
    if (!hasWorkpieceGeometry(project)) return [];
    const move = new Action({ id: "ctx-move-part", label: "Move part", icon: Move3d, shortcut: "M" });
    move.setEnabled(stageActions.movePart.enabled, stageActions.movePart.disabledReason ?? "");
    move.triggered.connect(launchPartMove);
    const rails = new Action({ id: "ctx-rails-welds", label: "Rails welds", icon: ListChecks, shortcut: "W" });
    rails.setEnabled(stageActions.railsWelds.enabled, stageActions.railsWelds.disabledReason ?? "");
    rails.triggered.connect(() => setRailsWeldsOpen(true));
    const waypoints = new Action({ id: "ctx-waypoint-welds", label: "Waypoint welds", icon: Spline });
    waypoints.setEnabled(stageActions.waypointWelds.enabled, stageActions.waypointWelds.disabledReason ?? "");
    waypoints.triggered.connect(() => setWaypointWeldsOpen(true));
    const remove = new Action({ id: "ctx-delete-part", label: "Delete part", icon: Trash2, shortcut: "Del" });
    remove.setEnabled(lock === null, lock ?? "");
    remove.triggered.connect(deletePart);
    return [move, rails, waypoints, remove];
  }
  if (sel.kind === "weld") {
    const node = nodeById(project.nodes, sel.id);
    if (!node) return [];
    const manual = isManualPoseWeld(node);
    const open = manual
      ? new Action({
        id: "ctx-open-weld",
        label: `Open in ${weldToolActions[waypointWeldOf(node).tool].label}`,
        icon: weldToolActions[waypointWeldOf(node).tool].icon ?? Spline,
      })
      : new Action({ id: "ctx-open-weld", label: "Open in Rails welds", icon: ListChecks });
    if (manual) {
      open.setEnabled(stageActions.waypointWelds.enabled, stageActions.waypointWelds.disabledReason ?? "");
      open.triggered.connect(() => openWeldInWaypoints(sel.id));
    } else {
      open.setEnabled(railsMode.enabled, "Rails authoring only");
      open.triggered.connect(() => openWeldInRails(sel.id));
    }
    const enable = new Action({ id: "ctx-toggle-weld", label: node.enabled ? "Disable weld" : "Enable weld" });
    enable.setEnabled(lock === null, lock ?? "");
    enable.triggered.connect(() => toggleNodeEnabled(sel.id, !node.enabled));
    const remove = new Action({ id: "ctx-delete-weld", label: "Delete weld", icon: Trash2, shortcut: "Del" });
    const blocked = lock ?? deleteBlocked(project.nodes, sel.id);
    remove.setEnabled(blocked === null, blocked ?? "");
    remove.triggered.connect(() => deleteNode(sel.id));
    return [open, enable, remove];
  }
  if (sel.kind === "workspace") {
    const presets = new Action({ id: "ctx-weld-presets", label: "Weld presets", icon: SlidersHorizontal });
    presets.triggered.connect(() => setWeldPresetsOpen(true));
    const fixture = new Action({ id: "ctx-import-fixture", label: "Import fixtures / obstacles…", icon: Upload });
    fixture.triggered.connect(() => chooseStepFiles("fixture"));
    const stand = new Action({ id: "ctx-add-stand", label: "Add a stand", icon: Move3d });
    stand.triggered.connect(addStand);
    return [presets, fixture, stand];
  }
  const rename = new Action({ id: "ctx-rename", label: "Rename program" });
  rename.triggered.connect(() => {
    setSelection({ kind: "program" });
    beginRenameProgram();
  });
  return [rename];
}

/**
 * Take the part out of the program: its geometry, the edge catalog, the
 * seams and welds authored on it, the rows compiled from them, and its
 * placement. The program keeps its name, cell and presets, and reads "No
 * part" until the next import. One revision, one undo step.
 */
function deletePart(): void {
  const lock = editingLockReason();
  if (lock) {
    setStorageStatus(lock);
    return;
  }
  if (!hasWorkpieceGeometry(project)) return;
  const filename = project.cad.filename || "part";
  if (viewer.workpieceGizmoEnabled) setPartGizmo(false);
  if (railsWeldsCard.shown) setRailsWeldsOpen(false);
  if (waypointWeldsCard.shown) setWaypointWeldsOpen(false);
  mutate("cad", (draft) => {
    forgetActiveWorkpiece(draft);
    draft.cad = { kind: "step", filename: "", fingerprint: "", sourceUnit: "mm", topologyModelId: "" };
    draft.edgeCatalog = [];
    draft.seams = [];
    draft.nodes = [];
    draft.sequence = [];
    draft.rows = [];
    draft.fixtures = [];
    draft.simulation.selectedRowId = null;
  });
  activeCADSource = null;
  // No part, no seams to detect: rails waits for the next import, which
  // re-arms it (syncRailsModeToProject) with the new STEP at hand.
  railsMode.reset();
  resetWeldAuthoringUiState();
  setSelection({ kind: "part" });
  syncRailsButton();
  appLogger.info(`Removed ${filename} from the program`);
  setStorageStatus(`Removed ${filename} · import a part to author welds`);
}

function openWeldInRails(nodeId: string): void {
  setRailsWeldsOpen(true);
  if (!railsMode.openEntryForSeam(nodeId)) {
    setStorageStatus("This weld's seam is not in the current rails session; re-import the part to search it again.");
  }
}

/** Open a weld in the tool that made it: a hand-placed one in its own
 *  tool, a seam in Rails welds. */
function openWeld(nodeId: string): void {
  const node = nodeById(project.nodes, nodeId);
  if (node && isManualPoseWeld(node)) openWeldInWaypoints(nodeId);
  else openWeldInRails(nodeId);
}

function openWeldInWaypoints(nodeId: string): void {
  const node = nodeById(project.nodes, nodeId);
  const tool = node && isManualPoseWeld(node) ? waypointWeldOf(node).tool : "waypoint";
  waypointSession.select({ weldId: nodeId });
  setWaypointWeldsOpen(true, tool);
}

/** The Weld presets card: the process defaults, as a tool. */
function setWeldPresetsOpen(open: boolean): void {
  stageActions.weldPresets.checked = open;
  if (open) {
    if (viewer.workpieceGizmoEnabled) setPartGizmo(false);
    if (railsWeldsCard.shown) setRailsWeldsOpen(false);
    if (waypointWeldsCard.shown) setWaypointWeldsOpen(false);
    if (planPresetsCard.shown) setPlanPresetsOpen(false);
    weldPresetsCard.show();
  } else {
    weldPresetsCard.hide();
  }
}

/** The plan list under the dock's panels: every plan the program has
 *  had, newest first, the active one marked, the one on the robot too.
 *  A row's Select makes it the active plan; × forgets it. */
function renderPlanHistory(): void {
  const list = document.querySelector<HTMLElement>(".plan-history-list");
  if (!list) return;
  list.replaceChildren();
  const entries = project.planHistory ?? [];
  const activeId = activePlanEntryId(project);
  const onRobot = densePanel.loadedTrajectoryDigest();
  const lock = editingLockReason();
  if (entries.length === 0) {
    list.append(element("p", "inspector-empty", "No plans yet. Plan the program and its plans collect here."));
    return;
  }
  for (const entry of entries) {
    const row = element("div", "plan-row");
    row.dataset.planId = entry.id;
    const active = entry.id === activeId;
    const deployed = entry.denseTrajectory !== null && entry.denseTrajectory.trajectoryDigest === onRobot;
    const stale = entry.sourceProjectRevision !== project.identity.revision;
    row.classList.toggle("is-active", active);
    row.classList.toggle("is-stale", stale);
    const pick = element("button", "plan-row-pick");
    pick.type = "button";
    pick.setAttribute("role", "radio");
    pick.setAttribute("aria-checked", String(active));
    pick.title = active ? "The active plan" : lock ?? "Make this the active plan";
    pick.disabled = active || lock !== null;
    const marks = element("span", "plan-row-marks");
    marks.append(element("span", `plan-row-mark is-active-mark${active ? "" : " is-off"}`, "●"));
    marks.append(element("span", `plan-row-mark is-robot-mark${deployed ? "" : " is-off"}`, "▲"));
    marks.title = `${active ? "active" : "not active"}${deployed ? " · on the robot" : ""}`;
    const text = element("span", "plan-row-text");
    const title = element("span", "plan-row-title", `${entry.planner} · rev ${entry.sourceProjectRevision}`);
    const detailParts = [plannedAtLabel(entry.createdAt)];
    if (entry.summary) {
      detailParts.push(`${entry.summary.durationS.toFixed(1)} s`);
      if (entry.summary.segments > 0) detailParts.push(`${entry.summary.segments} motions`);
      else detailParts.push(`${entry.summary.sampleCount} samples`);
    }
    if (stale) detailParts.push("program changed since");
    text.append(title, element("span", "plan-row-detail", detailParts.join(" · ")));
    pick.append(marks, text);
    pick.addEventListener("click", () => selectPlanEntry(entry.id));
    const remove = element("button", "plan-row-remove", "×");
    remove.type = "button";
    remove.title = deployed ? "Loaded on the robot; STOP before forgetting it" : lock ?? "Forget this plan";
    remove.setAttribute("aria-label", "Forget this plan");
    remove.disabled = deployed || lock !== null;
    remove.addEventListener("click", () => deletePlanEntry(entry.id));
    row.append(pick, remove);
    list.append(row);
  }
}

function selectPlanEntry(entryId: string): void {
  const lock = editingLockReason();
  if (lock) {
    setStorageStatus(lock);
    return;
  }
  const previous = project;
  const next = activatePlan(project, entryId);
  if (next === project) return;
  project = next;
  history.push(previous);
  changed("Plan activated");
}

function deletePlanEntry(entryId: string): void {
  const lock = editingLockReason();
  if (lock) {
    setStorageStatus(lock);
    return;
  }
  const previous = project;
  const wasActive = activePlanEntryId(project) === entryId;
  const next = deletePlan(project, entryId);
  if (next === project) return;
  project = next;
  history.push(previous);
  changed(wasActive ? "Plan forgotten; the program has no active plan" : "Plan forgotten");
}

/** Cells ▾ → a discovered cell (or Search local network…): the dialog with
 *  the network section open, and an addable cell's binding in the Add form
 *  waiting for the operator's ADD TO REGISTRY. */
function openDiscoveredCell(row: DiscoveredRow | null): void {
  setRobotOpen(true);
  robotDialog.openNetworkSection();
  if (row) robotDialog.prefillFromDiscovery(row);
}

function renderDiscovery(): void {
  robotDialog.renderDiscovery({ rows: cellDiscovery.rows(), status: cellDiscovery.statusText(), searching: cellDiscovery.searching });
}

/** The Cells dialog describes every cell so identity failures remain visible. */
function setRobotOpen(open: boolean): void {
  if (open) {
    renderRobotDialog();
    // Rebuilt on open and on a new search only, not with every status poll.
    renderDiscovery();
    robotDialog.open();
    void densePanel.refreshCells();
  } else {
    robotDialog.close();
  }
}

function renderRobotDialog(): void {
  robotDialog.setAddError(densePanel.lastCellError());
  const activeId = densePanel.activeCell();
  const connected = densePanel.connectedCell();
  robotDialog.renderCells({
    cells: densePanel.cells(),
    probes: densePanel.probes(),
    activeId,
    daemonReachable: densePanel.daemonReachable(),
    selecting: densePanel.selecting(),
    status: densePanel.selecting()
      ? `Selecting ${activeId ? "another cell" : "a cell"}…`
      : densePanel.targetStatusText() || "Joint daemon · checking",
  });
  const held = project.robot.cell ?? null;
  const live = densePanel.cellSettings();
  let warning: string | null = null;
  if (held && connected && connected !== held.cell) {
    warning = `Connected to ${connected}, not ${held.cell}. Pull again before planning.`;
  } else if (held && live && cellSettingsKey(live) !== cellSettingsKey(held)) {
    warning = `${connected} now reports ${describeCellSettings(live)}. Pull again before planning.`;
  }
  const lock = editingLockReason();
  robotDialog.renderSettings({
    held: held ? describeCellSettings(held) : null,
    heldCell: held?.cell ?? null,
    warning,
    canPull: live !== null && lock === null,
    pullReason: lock ?? (densePanel.targetSelected() ? "Waiting for the cell's status" : "Connect to a cell first"),
  });
}

/** The Plan presets card: the planner's parameter sets, as a tool. */
function setPlanPresetsOpen(open: boolean): void {
  const wanted = open && railsMode.enabled;
  stageActions.planPresets.checked = wanted;
  if (wanted) {
    if (viewer.workpieceGizmoEnabled) setPartGizmo(false);
    if (railsWeldsCard.shown) setRailsWeldsOpen(false);
    if (waypointWeldsCard.shown) setWaypointWeldsOpen(false);
    if (weldPresetsCard.shown) setWeldPresetsOpen(false);
    planPresetsCard.show();
    renderPlanPresets();
  } else {
    planPresetsCard.hide();
  }
}

function renderPlanPresets(): void {
  if (!planPresetsCard.shown || !railsMode.enabled) return;
  planPresetsCard.setSummaries(railsMode.renderParameterSections({
    search: planPresetsCard.search,
    torch: planPresetsCard.torch,
  }));
}

/** The spreadsheet is an auxiliary view of the program under the viewport,
 *  off by default; the tree is the primary list. Remembered per browser. */
function readSpreadsheetShown(): boolean {
  try {
    return localStorage.getItem("olp.spreadsheet.shown") === "1";
  } catch {
    return false;
  }
}

function applySpreadsheetShown(shown: boolean): void {
  query(".workspace").classList.toggle("is-sheet-collapsed", !shown && !dockPaneShown());
  // The telemetry strip needs the room a bare spreadsheet does not: the dock
  // grows while the Jog panel is open (the resizer applies the floor; a later
  // drag takes over as usual).
  query(".workspace").classList.toggle("has-jog", dockPaneShown());
  resizableLayout.apply();
  query("#program-sheet").hidden = !shown;
  query("#jog-resizer").hidden = !shown || !dockPaneShown();
  query("#bottom-workspace").classList.toggle("has-jog", dockPaneShown());
  query("#bottom-workspace").classList.toggle("has-sheet", shown);
  try {
    localStorage.setItem("olp.spreadsheet.shown", shown ? "1" : "0");
  } catch {
    // Storage blocked: the choice lasts the session.
  }
  viewer.resize();
}

/** The kept STEPs, for the Import menu. Refreshed at boot and after every
 *  import; the menu reads the cache because it opens synchronously. */
async function refreshRecentParts(): Promise<void> {
  try {
    recentParts = await repository.listSources();
  } catch {
    recentParts = [];
  }
}

async function importRecentPart(fingerprint: string): Promise<void> {
  const recent = recentParts.find(part => part.fingerprint === fingerprint);
  if (!recent) return;
  await importStepBytes(recent.filename, async () => {
    const source = await repository.getSource(fingerprint);
    if (!source) {
      await refreshRecentParts();
      throw new Error("That part's kept copy is gone from this browser; import the file again.");
    }
    return source.bytes;
  });
}

/** The Rails welds card: open with the tool, closed by ✕ or by another
 *  tool taking the corner. Only meaningful in rails authoring. */
function setRailsWeldsOpen(open: boolean): void {
  const wanted = open && railsMode.enabled;
  stageActions.railsWelds.checked = wanted;
  if (wanted) {
    if (viewer.workpieceGizmoEnabled) setPartGizmo(false);
    if (waypointWeldsCard.shown) setWaypointWeldsOpen(false);
    if (weldPresetsCard.shown) setWeldPresetsOpen(false);
    if (planPresetsCard.shown) setPlanPresetsOpen(false);
    if (!railsWeldsCard.shown) {
      railsWeldsOpenedWith = { project, rails: railsMode.snapshot() };
    }
    // Part focus: the cell fades, the camera turns about the part, and the
    // seams are the only bright lines in the view.
    viewer.setPartFocus(true);
    railsWeldsCard.setTransparency(viewer.partDim);
    railsWeldsCard.show();
    renderWeldDrawerPanel();
  } else {
    viewer.setPartFocus(false);
    railsWeldsCard.hide();
    railsWeldsOpenedWith = null;
  }
}

/** ✕ on the Rails welds card: the program's weld nodes go back to the
 *  revision the card opened on (kept in history, so it is itself undoable),
 *  and the rails session's decisions and parameters follow. */
function cancelRailsWelds(): void {
  const opened = railsWeldsOpenedWith;
  if (opened) {
    railsMode.restore(opened.rails);
    if (project !== opened.project) {
      history.push(project);
      project = opened.project;
      changed("Rails welds: put back to how they were when the card opened");
    }
  }
  setRailsWeldsOpen(false);
}

/** The hand-placed weld card: open for one of its three tools, closed by
 *  ✓/✕ or by another tool taking the corner. Picking another of the three
 *  while it is open switches the card and keeps the selection, so a line
 *  drawn next starts where the arc just drawn ended. Rails programs only. */
function setWaypointWeldsOpen(
  open: boolean,
  tool: WeldTool = waypointWeldsCard.tool,
  options: { create?: boolean } = {},
): void {
  const wanted = open && stageActions.waypointWelds.enabled;
  for (const [each, action] of Object.entries(weldToolActions) as [WeldTool, Action][]) {
    action.checked = wanted && each === tool;
  }
  if (wanted) {
    waypointWeldsCard.setTool(tool);
    if (viewer.workpieceGizmoEnabled) setPartGizmo(false);
    if (railsWeldsCard.shown) setRailsWeldsOpen(false);
    if (weldPresetsCard.shown) setWeldPresetsOpen(false);
    if (planPresetsCard.shown) setPlanPresetsOpen(false);
    // Recorded before the new weld exists, so ✕ takes it back too.
    if (!waypointWeldsCard.shown) waypointWeldsOpenedWith = project;
    viewer.setPartFocus(true);
    waypointWeldsCard.setTransparency(viewer.partDim);
    waypointWeldsCard.show();
    if (options.create) waypointWeldsCard.newWeld();
  } else {
    if (waypointWeldsCard.shown) viewer.setPartFocus(false);
    waypointWeldsCard.hide();
    waypointWeldsOpenedWith = null;
  }
  renderWaypointWelds();
}

/** ✕ on the Waypoint welds card: the program goes back to the revision the
 *  card opened on (kept in history, so it is itself undoable). */
function cancelWaypointWelds(): void {
  const opened = waypointWeldsOpenedWith;
  if (opened && project !== opened) {
    history.push(project);
    project = opened;
    changed("Waypoint welds: put back to how they were when the card opened");
  }
  setWaypointWeldsOpen(false);
}

/** The waypoint paths follow the program; the torches and the handle are
 *  there only while the tool is open. */
function renderWaypointWelds(): void {
  const open = waypointWeldsCard.shown;
  const selection = waypointSession.selection;
  waypointOverlay.sync(waypointSession.welds, selection);
  waypointOverlay.torchesVisible = open;
  viewer.setWaypointGizmoTarget(open ? waypointOverlay.torchFor(selection.waypointId) : null);
  if (open) waypointWeldsCard.render();
}

/**
 * Before waypoint welds were program nodes the tool kept one weld per
 * program in this browser's storage. A rails program that still has one
 * there, and no waypoint weld of its own, takes it in as a weld node -- once,
 * after this render, as its own undoable edit.
 */
function adoptStoredWaypoints(): void {
  const id = project.identity.id;
  if (!project.railsAuthoring || storedWaypointsChecked.has(id)) return;
  storedWaypointsChecked.add(id);
  if (project.nodes.some(isManualPoseWeld)) return;
  const stored = takeStoredWaypoints(id);
  if (stored.length === 0) return;
  queueMicrotask(() => {
    if (project.identity.id !== id) return;
    waypointSession.importWeld(stored, {
      message: `Waypoint weld kept in this browser added to the program (${stored.length} waypoints)`,
    });
  });
}

function cancelPartMove(): void {
  const restore = placementBeforeMove;
  if (restore && hasWorkpieceGeometry(project)) {
    mutate("fixtures", (draft) => {
      setWorkpiecePlacement(draft, restore);
    });
  }
  setPartGizmo(false);
}


function commitGizmoPlacement(
  anchorM: [number, number, number],
  rotationXyzw: [number, number, number, number],
): void {
  const lock = editingLockReason();
  if (lock || !hasWorkpieceGeometry(project)) {
    // The mesh followed the handle; put it back where the project says.
    setStorageStatus(lock ?? "Import a part before placing it");
    void syncViewerProject();
    return;
  }
  const next = placementFromWorldAnchorPose(project, anchorM, rotationXyzw);
  mutate("fixtures", (draft) => {
    setWorkpiecePlacement(draft, next);
  });
}

function renderWorkpiecePlacement(): void {
  const placement = workpiecePlacement(project);
  const available = hasWorkpieceGeometry(project) && placement !== null;
  const lock = editingLockReason();
  const gizmoAllowed = available && lock === null;
  stageActions.movePart.setEnabled(gizmoAllowed, lock ?? "Import a part before placing it");
  if (!gizmoAllowed && viewer.workpieceGizmoEnabled) setPartGizmo(false);
  renderWeldToolActions();
  const values: Record<string, number> = placement
    ? {
      x: placement.offsetFromWorkFrameM[0] * 1_000,
      y: placement.offsetFromWorkFrameM[1] * 1_000,
      z: placement.offsetFromWorkFrameM[2] * 1_000,
      rx: placement.rotationRpyRad[0] * 180 / Math.PI,
      ry: placement.rotationRpyRad[1] * 180 / Math.PI,
      rz: placement.rotationRpyRad[2] * 180 / Math.PI,
    }
    : { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 };
  placementCard.setValues(values as PlacementValues, numberCell);
  placementCard.setNote(`Offsets from ${workFrameLabel()} · in the part's own axes`);
}

/** Open the Move part tool from the inspector; selects the part first,
 *  so the tool acts on the selection (one part today, any part later). */
/* ---- The stand: the workcell fixture the part sits on. ---------------- */

/** Open the Stand card on a fixture; the card floats where Move part's does. */
function openStandCard(id: string): void {
  const fixture = fixtureById(project, id);
  if (!fixture) return;
  const lock = editingLockReason();
  if (lock) {
    setStorageStatus(lock);
    return;
  }
  setSelection({ kind: "fixture", id });
  if (placementCard.shown) setPartGizmo(false);
  standEditing = id;
  standBeforeEdit = JSON.parse(JSON.stringify(fixture)) as WorkcellFixture;
  standCard.setFixture(fixture);
  standCard.show();
}

/** ✓ keeps the fixture; ✕ puts back the one the card opened with. */
function closeStandCard(restore: boolean): void {
  const id = standEditing;
  const before = standBeforeEdit;
  standEditing = null;
  standBeforeEdit = null;
  standCard.hide();
  if (restore && id && before) {
    const current = fixtureById(project, id);
    if (JSON.stringify(current) !== JSON.stringify(before)) {
      mutate("workcell", (draft) => {
        const index = draft.workcellFixtures.findIndex((fixture) => fixture.id === id);
        if (index >= 0) draft.workcellFixtures[index] = before;
        else draft.workcellFixtures.push(before);
      });
      void syncViewerProject();
    }
  }
}

function renderStandCard(): void {
  if (!standEditing) return;
  const fixture = fixtureById(project, standEditing);
  if (!fixture) {
    standEditing = null;
    standBeforeEdit = null;
    standCard.hide();
    return;
  }
  standCard.setFixture(fixture);
}

/** One number changed in the Stand card: size, position or rotation. */
function updateStandField(field: WorkcellFixtureField, value: number): void {
  const id = standEditing;
  if (!id || !Number.isFinite(value)) return;
  const current = fixtureById(project, id);
  if (!current) return;
  const next = editWorkcellFixture(current, field, value);
  if (next === current) return;
  mutate("workcell", (draft) => {
    const index = draft.workcellFixtures.findIndex((fixture) => fixture.id === id);
    if (index >= 0) draft.workcellFixtures[index] = next;
  });
  void syncViewerProject();
}

function replaceStandWithStep(): void {
  if (!standEditing) return;
  standStepInput.value = "";
  standStepInput.click();
}

/** The picked STEP becomes the stand: tessellated by the CAD service for the
 *  viewport and the planner, its bytes kept under their fingerprint so the
 *  program reopens with it. */
async function importStandStep(): Promise<void> {
  const id = standEditing;
  const file = standStepInput.files?.[0];
  standStepInput.value = "";
  if (!id || !file) return;
  const filename = file.name;
  setStorageStatus(`Reading ${filename} for the stand…`);
  try {
    const bytes = new Uint8Array(await file.arrayBuffer());
    const mesh = await cadClient.tessellate({ source: { kind: "bytes", filename, bytes } });
    const fingerprint = mesh.fingerprint.toLowerCase();
    workcellFixtureMeshes.set(fingerprint, mesh);
    void repository
      .putSource({ fingerprint, filename, bytes })
      .catch((error) => console.warn("keeping the stand's STEP failed:", error));
    mutate("workcell", (draft) => {
      const index = draft.workcellFixtures.findIndex((fixture) => fixture.id === id);
      if (index < 0) return;
      const fixture = draft.workcellFixtures[index]!;
      draft.workcellFixtures[index] = {
        ...fixture,
        name: fixture.origin === "cell-default" ? "Stand" : fixture.name,
        shape: { kind: "step", fingerprint, filename },
        origin: "authored",
      };
    });
    setStorageStatus(`Stand is now ${filename} · ${mesh.triangleCount.toLocaleString()} triangles`);
    void syncViewerProject();
  } catch (error) {
    setStorageStatus(`Stand STEP blocked: ${message(error)}`);
  }
}

function resetStandToCellDefault(): void {
  const id = standEditing;
  if (!id) return;
  const fallback = defaultWorkcellFixtures(project.robot.model).find((fixture) => fixture.id === id)
    ?? { ...newBoxFixture(id), origin: "cell-default" as const };
  mutate("workcell", (draft) => {
    const index = draft.workcellFixtures.findIndex((fixture) => fixture.id === id);
    if (index >= 0) draft.workcellFixtures[index] = fallback;
  });
  void syncViewerProject();
}

function removeStand(): void {
  const id = standEditing;
  if (!id) return;
  standEditing = null;
  standBeforeEdit = null;
  standCard.hide();
  mutate("workcell", (draft) => {
    draft.workcellFixtures = draft.workcellFixtures.filter((fixture) => fixture.id !== id);
  });
  setSelection({ kind: "workspace" });
  setStorageStatus("Fixture removed · it is no longer included in collision checking");
  void syncViewerProject();
}

/** A cell with no stand gets one to edit: the tree's "Add a stand" hint. */
function addStand(): void {
  const lock = editingLockReason();
  if (lock) {
    setStorageStatus(lock);
    return;
  }
  const stand = workcellFixtures(project).find(fixture => fixture.id === "stand");
  if (stand) {
    openStandCard(stand.id);
    return;
  }
  const fixture = newBoxFixture();
  mutate("workcell", (draft) => {
    draft.workcellFixtures.push(fixture);
  });
  void syncViewerProject();
  openStandCard(fixture.id);
}

/** STEP fixtures the project names but this session has not tessellated:
 *  their bytes are kept under the fingerprint at import, so a reopened
 *  program gets its stand back without the file. */
async function loadWorkcellFixtureMeshes(): Promise<void> {
  for (const fixture of environmentFixtures(project)) {
    if (fixture.shape.kind !== "step") continue;
    const fingerprint = fixture.shape.fingerprint.toLowerCase();
    if (workcellFixtureMeshes.has(fingerprint) || workcellFixtureMeshesInFlight.has(fingerprint)) continue;
    workcellFixtureMeshesInFlight.add(fingerprint);
    try {
      const stored = await repository.getSource(fingerprint);
      if (!stored) {
        setStorageStatus(`The STEP for ${fixture.name} is not kept in this browser; restore its source before planning`);
        continue;
      }
      const mesh = await cadClient.tessellate({ source: { kind: "bytes", filename: stored.filename, bytes: stored.bytes } });
      workcellFixtureMeshes.set(fingerprint, mesh);
      void syncViewerProject();
    } catch (error) {
      console.warn(`The STEP for ${fixture.name} could not be tessellated:`, error);
    } finally {
      workcellFixtureMeshesInFlight.delete(fingerprint);
    }
  }
}

function launchPartMove(): void {
  setSelection({ kind: "part" });
  const action = stageActions.movePart;
  if (!action.enabled) {
    setStorageStatus(action.disabledReason ?? "Move part is unavailable");
    return;
  }
  setPartGizmo(true);
}

/** One placement number changed in the Move part card. */
function updateWorkpiecePlacementField(field: string, value: number): void {
  const current = workpiecePlacement(project);
  if (!current || !field || !Number.isFinite(value)) {
    renderWorkpiecePlacement();
    return;
  }
  const next = {
    offsetFromWorkFrameM: [...current.offsetFromWorkFrameM] as [number, number, number],
    rotationRpyRad: [...current.rotationRpyRad] as [number, number, number],
  };
  if (field === "x" || field === "y" || field === "z") {
    const index = field === "x" ? 0 : field === "y" ? 1 : 2;
    next.offsetFromWorkFrameM[index] = clampNumber(value, -5_000, 5_000) / 1_000;
  } else if (field === "rx" || field === "ry" || field === "rz") {
    const index = field === "rx" ? 0 : field === "ry" ? 1 : 2;
    next.rotationRpyRad[index] = clampNumber(value, -360, 360) * Math.PI / 180;
  } else {
    renderWorkpiecePlacement();
    return;
  }
  mutate("fixtures", (draft) => {
    setWorkpiecePlacement(draft, next);
  });
}

function centreCurrentWorkpiece(): void {
  if (!hasWorkpieceGeometry(project)) return;
  mutate("fixtures", (draft) => {
    centreWorkpieceOnWorkFrame(draft);
  });
}

/**
 * A project identity change (new program, JSON import, catalog open, STEP
 * load) invalidates any in-flight edge selection: a draft could otherwise
 * bind to a different part's edges that happen to reuse the same ids.
 */
function resetWeldAuthoringUiState(): void {
  weldDraft = null;
  setEdgeSelectionMode(false);
}

/** Single owner of edge-selection mode: toolbar and drawer both route here. */
function setEdgeSelectionMode(next: boolean): void {
  edgeSelectionMode = next;
  syncEdgeSelectionState();
  renderWeldDrawerPanel();
}

function renderWeldDrawerPanel(): void {
  if (project.railsAuthoring) {
    // Rails authoring's welds live in the Rails welds tool card, framed by
    // it; the classic drawer stays empty and hidden.
    query("#weld-drawer").hidden = true;
    // enabled becomes true only after seam detection. A loading or failed
    // rails import is still a rails program, not a classic edge program.
    if (!railsMode.enabled) return;
    const welds = railsMode.renderPanel(railsWeldsCard.welds, { head: false, parameters: false });
    const parameters = railsMode.renderParameterSections({
      search: railsWeldsCard.search,
      torch: railsWeldsCard.torch,
    });
    railsWeldsCard.setSummaries({ welds, ...parameters });
    renderPlanPresets();
    return;
  }
  renderWeldDrawer(query("#weld-drawer"), {
    project,
    draft: weldDraft,
    edgeSelectionMode,
    welds: projectWelds(project),
  }, {
    onToggleMode: () => setEdgeSelectionMode(!edgeSelectionMode),
    onNameChange: (name) => {
      if (weldDraft) weldDraft = { ...weldDraft, name };
    },
    onTypeChange: (weldType) => {
      if (!weldDraft) return;
      weldDraft = { ...weldDraft, weldType };
      renderWeldDrawerPanel();
    },
    onAngleChange: (field, valueDeg) => {
      if (!weldDraft) return;
      weldDraft = { ...weldDraft, [field]: valueDeg };
      renderWeldDrawerPanel();
    },
    onSegmentTrim: (edgeId, side, value) => {
      if (!weldDraft) return;
      weldDraft = {
        ...weldDraft,
        segments: weldDraft.segments.map((segment) =>
          segment.edgeId === edgeId
            ? { ...segment, [side]: Math.max(0, Math.min(1, value)) }
            : segment
        ),
      };
      syncEdgeSelectionState();
      if (weldDraft) {
        updateWeldTrimReadout(query("#weld-drawer"), project, weldDraft, edgeId);
      }
    },
    onSegmentReverse: (edgeId) => {
      if (!weldDraft) return;
      weldDraft = {
        ...weldDraft,
        segments: weldDraft.segments.map((segment) =>
          segment.edgeId === edgeId
            ? { ...segment, reversed: !segment.reversed }
            : segment
        ),
      };
      renderWeldDrawerPanel();
    },
    onSegmentActivate: (edgeId) => {
      if (!weldDraft) return;
      weldDraft = { ...weldDraft, activeEdgeId: edgeId };
      syncEdgeSelectionState();
      renderWeldDrawerPanel();
    },
    onSegmentRemove: (edgeId) => {
      if (!weldDraft) return;
      weldDraft = draftWithoutEdge(weldDraft, edgeId);
      syncEdgeSelectionState();
      renderWeldDrawerPanel();
    },
    onCreateWeld: createWeldFromDraft,
    onClearDraft: () => {
      weldDraft = null;
      syncEdgeSelectionState();
      renderWeldDrawerPanel();
    },
    onDeleteWeld: deleteWeldFromProgram,
  });
}

/** "Create weld": the only path from selected edges to seams and rows. */
function createWeldFromDraft(): void {
  if (!weldDraft || weldDraft.segments.length === 0) return;
  if (activeExecution) {
    setStorageStatus("Editing is locked while the reviewed robot execution is active.");
    return;
  }
  try {
    const previous = project;
    project = createWeldFromEdges(project, {
      name: weldDraft.name,
      weldType: weldDraft.weldType,
      segments: weldDraft.segments,
      workAngleDeg: weldDraft.workAngleDeg,
      travelAngleDeg: weldDraft.travelAngleDeg,
      spinAngleDeg: weldDraft.spinAngleDeg,
    });
    history.push(previous);
    const created = weldDraft.segments.length;
    weldDraft = null;
    serverExecutionReview = null;
    serverExecutionReviewWire = null;
    changed(`Weld created from ${created} selected edge${created === 1 ? "" : "s"}`);
  } catch (error) {
    setStorageStatus(`Weld creation blocked: ${message(error)}`);
  }
}

function deleteWeldFromProgram(weldId: string): void {
  if (!weldId) return;
  if (activeExecution) {
    setStorageStatus("Editing is locked while the reviewed robot execution is active.");
    return;
  }
  try {
    const previous = project;
    project = deleteWeld(project, weldId);
    history.push(previous);
    serverExecutionReview = null;
    serverExecutionReviewWire = null;
    changed("Weld deleted with its seam and rows");
  } catch (error) {
    setStorageStatus(`Weld deletion blocked: ${message(error)}`);
  }
}

/** Shift-click in the viewport used to append a raw control point row. Rows
 *  are compiled from nodes now, so a point is added in the weld's dialog. */
function addControlPointAtWorld(pointM: Vec3): void {
  const [x, y, z] = pointM.map((value) => (value * 1000).toFixed(1));
  setStorageStatus(`Picked [${x}, ${y}, ${z}] mm · add points in the weld node's parameters (EDIT), rows are compiled from nodes.`);
}

function migrateLegacyTablePlacement(
  candidate: OfflineProgrammingProject,
): OfflineProgrammingProject {
  const migrateTrimUnits = needsPhysicalSeamTrimMigration(candidate);
  // THE CELL THE PROGRAM IS PINNED TO HAS TO BE THE ROBOT THE VIEWER DRAWS.
  // The selector reconciles them, but only on `change` -- a project that
  // already held `robot.model: rosie_1420_v1` renders the select at 1420, so
  // picking 1420 again fires nothing and `programV2.cell` keeps the 1400 it
  // was created with. Every such project exported a 1400 `.weldplan` whose
  // placement was then read from the 1400's table: the part authored at world
  // [1, 1, 1] arrived at [2.9, 1.7, 1.7], 2.4 to 2.9 m from the arm.
  //
  // `robot.model` wins because it is what the operator picked and what the
  // viewer draws; the pinned digest goes back to the placeholder because it
  // named the other cell's descriptor.
  const reconcileCell = needsCellPinReconcile(candidate);
  // A project saved before the work frame was per-cell hangs its part off the
  // 1400's table whatever robot it names. Re-hang it on its own cell's frame,
  // keeping the part where it was drawn -- BEFORE the centre-migration test
  // below, which would otherwise see a mismatched frame and answer by throwing
  // the authored placement away.
  const rehangWorkFrame = needsCellWorkFrameRehang(candidate);
  const migrateWorkFrame = !rehangWorkFrame && needsWorkFrameMigration(candidate);
  const migrateDogBoxPlanner = needsDogBoxPlannerDefaultsMigration(candidate);
  // Seams the CP-budget import deliberately left rowless (over-budget list)
  // are a valid saved state — "repairing" them at boot mutated saved
  // programs and invalidated their plans (OF-1). Genuine legacy damage
  // (a seam that should have rows but lost them) is still restored.
  const deliberatelyRowless = deliberatelyRowlessSeamIDs(candidate);
  const restoreMissingCadPaths = candidate.seams.some((seam) =>
    seam.geometry.pointsM.length >= 2 &&
    !deliberatelyRowless.has(seam.id) &&
    !candidate.rows.some((row) => row.seamId === seam.id)
  );
  if (
    !migrateTrimUnits &&
    !reconcileCell &&
    !rehangWorkFrame &&
    !migrateWorkFrame &&
    !migrateDogBoxPlanner &&
    !restoreMissingCadPaths
  ) {
    return candidate;
  }
  return reviseProject(
    candidate,
    migrateTrimUnits
      ? "seams"
      : rehangWorkFrame || migrateWorkFrame
        ? "fixtures"
        : reconcileCell || migrateDogBoxPlanner ? "program" : "rows",
    (draft) => {
      if (migrateTrimUnits) migrateLegacySeamTrims(draft);
      if (reconcileCell) reconcileCellPin(draft);
      if (rehangWorkFrame) alignWorkpieceToCellWorkFrame(draft);
      if (migrateWorkFrame) centreWorkpieceOnWorkFrame(draft);
      if (migrateDogBoxPlanner) migrateDogBoxPlannerDefaults(draft);
      if (migrateTrimUnits || rehangWorkFrame || migrateWorkFrame) {
        draft.rows = rowsFromSeams(draft);
      } else {
        ensureCadPathRows(draft);
      }
    },
  );
}

function renderProgramCatalog(): void {
  const select = query<HTMLSelectElement>("#catalog-program");
  const priorSelection = select.value;
  const programs = programCatalog?.programs ?? [];
  select.innerHTML = programs.length === 0
    ? `<option value="">${programCatalog ? "No shared programs" : "Catalogue unavailable"}</option>`
    : programs.map((program) => `
      <option value="${escapeHTML(program.program_id)}">${
        escapeHTML(program.program_name)
      } · r${program.program_revision} · ${escapeHTML(program.edit_source ?? "unknown")}</option>
    `).join("");
  const preferred = programs.some((candidate) => candidate.program_id === priorSelection)
    ? priorSelection
    : programs.some((candidate) => candidate.program_id === project.program.programId)
      ? project.program.programId
      : programs[0]?.program_id ?? "";
  select.value = preferred;

  query<HTMLButtonElement>("#catalog-refresh").disabled = programCatalogInFlight;
  query<HTMLButtonElement>("#catalog-open").disabled =
    programCatalogInFlight || !select.value;
  const stored = programs.find((candidate) => candidate.program_id === project.program.programId);
  const unchangedStoredProgram = stored !== undefined &&
    project.program.programRevision <= project.program.baseProgramRevision;
  query<HTMLButtonElement>("#catalog-save").disabled =
    programCatalogInFlight ||
    !programCatalog ||
    project.rows.length === 0 ||
    unchangedStoredProgram;
  const catalogStatus = query("#catalog-status");
  catalogStatus.textContent = programCatalogInFlight
    ? "Contacting Motion Server catalogue…"
    : programCatalogError
      ? programCatalogError
      : programCatalog
        ? `Catalogue r${programCatalog.catalog_revision} · ${programs.length} program${
          programs.length === 1 ? "" : "s"
        } · local base r${project.program.baseProgramRevision}`
        : "Local draft only · Motion Server catalogue not configured";
  catalogStatus.title = catalogStatus.textContent;
}

async function refreshProgramCatalog(): Promise<void> {
  if (programCatalogInFlight) return;
  programCatalogInFlight = true;
  programCatalogError = null;
  renderProgramCatalog();
  try {
    programCatalog = await programCatalogClient.list();
  } catch (error) {
    programCatalog = null;
    programCatalogError = `Catalogue unavailable: ${message(error)}`;
  } finally {
    programCatalogInFlight = false;
    renderProgramCatalog();
  }
}

async function openSelectedCatalogProgram(): Promise<void> {
  await openCatalogProgram(query<HTMLSelectElement>("#catalog-program").value);
}

async function openCatalogProgram(programID: string): Promise<void> {
  if (!programID || programCatalogInFlight) return;
  programCatalogInFlight = true;
  programCatalogError = null;
  renderProgramCatalog();
  try {
    const entry = await programCatalogClient.get(programID);
    const previous = project;
    project = fromRobotV4Program(entry.program, {
      catalogRevision: entry.catalogRevision,
      timestamp: new Date().toISOString(),
    });
    history.push(previous);
    activeCADSource = await restoreCADSource(project);
    resetWeldAuthoringUiState();
    syncRailsModeToProject();
    targetState = reduceTargetState(targetState, {
      type: "select_offline",
      atMs: Date.now(),
    });
    changed(`Opened shared program ${entry.program.program_name} revision ${entry.program.program_revision}`);
  } catch (error) {
    programCatalogError = `Open blocked: ${message(error)}`;
  } finally {
    programCatalogInFlight = false;
    renderProgramCatalog();
  }
}

async function saveSharedProgram(): Promise<void> {
  if (!programCatalog || programCatalogInFlight || project.rows.length === 0) return;
  programCatalogInFlight = true;
  programCatalogError = null;
  renderProgramCatalog();
  try {
    const wire = toRobotV4Program(project);
    wire.program_state = "saved_unplanned";
    wire.planner_status = "stale";
    wire.validation_state = "unknown";
    wire.collision_check_state = "not_run";
    wire.limit_check_state = "not_run";
    wire.planned_path_revision = 0;
    wire.planned_sample_count = 0;
    const finalized = await finalizeRobotV4Program(wire);
    const savedCatalog = await programCatalogClient.save(
      finalized,
      programCatalog.catalog_revision,
    );
    const saved = savedCatalog.programs.find(
      (candidate) => candidate.program_id === finalized.program_id,
    );
    if (!saved) throw new Error("saved catalogue omitted the accepted program");
    const previous = project;
    project = fromRobotV4Program(saved, {
      project,
      catalogRevision: savedCatalog.catalog_revision,
      timestamp: new Date().toISOString(),
    });
    project.program.baseProgramRevision = saved.program_revision;
    project.program.programState = "saved_unplanned";
    history.push(previous);
    programCatalog = savedCatalog;
    changed(`Shared program saved · catalogue r${savedCatalog.catalog_revision} · program r${saved.program_revision}`);
  } catch (error) {
    if (error instanceof ProgramCatalogConflictError) {
      try {
        programCatalog = await programCatalogClient.list();
      } catch {
        // Keep the local program untouched; the typed conflict remains visible.
      }
      programCatalogError =
        `Conflict · Motion Server has catalogue r${error.catalogRevision ?? "?"}, ` +
        `program r${error.currentProgramRevision ?? "?"}. Open that revision before merging.`;
    } else {
      programCatalogError = `Shared save blocked: ${message(error)}`;
    }
  } finally {
    programCatalogInFlight = false;
    renderProgramCatalog();
  }
}


// Collapse the flat compiled program into one segment per semantic motion:
// a welding segment is a contiguous run that arcs on and shares a seam, a
// free-space segment is a contiguous travel run, and every non-motion row
// (comment, loop, I/O, ...) stays on its own. This is the spreadsheet the
// operator reads -- one row per weld seam, one per free move -- instead of the
// hundreds of exact-replay joint samples underneath.
const LEGACY_RAILS_BANNER =
  "This rails program predates nodes: its compiled rows are shown read-only. Re-author its welds in rails (click seams, accept) to edit them as nodes.";

function renderProgramSpreadsheet(): void {
  const host = query<HTMLElement>("#program-spreadsheet");
  const legacyRails = Boolean(project.railsAuthoring) && project.nodes.length === 0 && project.rows.length > 0;
  if (showAdvancedProgramFields || legacyRails) {
    renderCompiledRows(host, legacyRails ? LEGACY_RAILS_BANNER : null);
    syncNodeToolbar();
    return;
  }
  if (selectedNodeId && !nodeById(project.nodes, selectedNodeId)) selectedNodeId = null;
  if (!selectedNodeId) {
    const selectedRow = project.rows.find((row) => row.id === project.simulation.selectedRowId);
    selectedNodeId = (selectedRow && rowNodeId(selectedRow)) ?? project.nodes[0]?.id ?? null;
  }
  const locked = editingLockReason();
  const effective = new Set(effectiveNodes(project.nodes).map((node) => node.id));
  const context = {
    defaults: project.programV2.defaults,
    rowsOf: (id: string) => rowsOfNode(project, id),
    skipped: (id: string) => !effective.has(id),
    planned: (id: string) => railsSimulation.provenance?.current === true && Boolean(project.denseTrajectory?.segments.some(segment => segment.sourceId === id)),
  };
  renderNodeSpreadsheet(host, {
    summaries: project.nodes.map((node, index) => nodeSummary(node, index + 1, context)),
    selectedNodeId,
    playbackNodeId: activePlaybackNodeId(),
    lockedReason: locked,
    banner: null,
    moveBlocked: (id, delta) => moveBlocked(project.nodes, id, delta),
    deleteBlocked: (id) => deleteBlocked(project.nodes, id),
  }, {
    onSelect: (id) => selectNode(id),
    onOpen: (id) => {
      selectNode(id);
      if (!activeExecution) nodeEditor.open(id);
    },
    onEdit: editNodeInline,
    onToggleEnabled: toggleNodeEnabled,
    onMove: moveNodeInProgram,
    onDelete: deleteNode,
  });
  syncNodeToolbar();
}

/**
 * ADVANCED: the compiled robot.v4 rows, read-only. Nodes are the authored
 * program, so every control here is disabled and says which node to edit;
 * clicking a row selects its node. A legacy rails program with rows and no
 * nodes shows this view with a banner.
 */
function renderCompiledRows(host: HTMLElement, banner: string | null): void {
  if (project.rows.length === 0) {
    host.innerHTML = `${banner ? `<div class="spreadsheet-banner">${escapeHTML(banner)}</div>` : ""}<div class="spreadsheet-empty">No compiled rows. Rows are compiled from the program's nodes.</div>`;
    return;
  }
  const programView = buildProgramView(project);
  const selectedID = project.simulation.selectedRowId ?? project.rows[0]!.id;
  const seamControlRowIDs = spreadsheetSeamControlRowIDs();
  host.innerHTML = `
    ${banner ? `<div class="spreadsheet-banner">${escapeHTML(banner)}</div>` : ""}
    <table class="program-grid is-readonly">
      <thead>
        <tr>
          <th>#</th><th>On</th><th>Type</th><th>Command</th><th>Label</th>
          <th>Move</th><th>Target XYZ · m</th><th>CAD path · mm</th>
          <th>Trim · mm</th><th>Direction</th>
          <th class="speed-col">Speed · m/s</th><th>Arc</th><th>Source</th><th>Validation</th><th>Order</th>
        </tr>
      </thead>
      <tbody>
        ${programView.entries.map((entry) => {
          if (entry.kind === "control") {
            return controlPointSpreadsheetRow(
              entry.control,
              entry.control.row.id === selectedID,
              seamControlRowIDs.has(entry.control.row.id),
            );
          }
          if (entry.kind === "move") return moveProgramSpreadsheetRow(entry.move, selectedID);
          return spreadsheetRows(
            entry.row,
            entry.rowIndex,
            entry.row.id === selectedID,
            seamControlRowIDs.has(entry.row.id),
          );
        }).join("")}
      </tbody>
    </table>
  `;
  for (const control of host.querySelectorAll<HTMLInputElement | HTMLSelectElement | HTMLButtonElement>("input, select, button")) {
    const rowID = control.closest<HTMLElement>("[data-program-row]")?.dataset.programRow;
    const row = project.rows.find((candidate) => candidate.id === rowID);
    const node = row ? nodeById(project.nodes, rowNodeId(row) ?? "") : null;
    control.disabled = true;
    control.title = node
      ? `Compiled from ${node.op} ${node.label || node.id}; edit the node`
      : "Compiled rows are read-only";
  }
  applyPlaybackRowHighlight(false);
  host.onclick = (event) => {
    const target = event.target as HTMLElement;
    const rowID = target.closest<HTMLElement>("[data-program-row]")?.dataset.programRow;
    if (!rowID) return;
    const row = project.rows.find((candidate) => candidate.id === rowID);
    if (!row) return;
    project.simulation.selectedRowId = rowID;
    selectedNodeId = rowNodeId(row) ?? selectedNodeId;
    onRowSelectionChanged();
    renderProgramSpreadsheet();
  };
}

function spreadsheetSeamControlRowIDs(): Set<string> {
  const rowIDBySeam = new Map<string, string>();
  for (const row of project.rows) {
    if (row.seamId && row.phase === "weld" && !rowIDBySeam.has(row.seamId)) {
      rowIDBySeam.set(row.seamId, row.id);
    }
  }
  for (const row of project.rows) {
    if (row.seamId && !rowIDBySeam.has(row.seamId)) {
      rowIDBySeam.set(row.seamId, row.id);
    }
  }
  return new Set(rowIDBySeam.values());
}

function seamSpreadsheetCells(
  row: AuthoredProgramRow,
  isControlRow: boolean,
): string {
  const seam = row.seamId
    ? project.seams.find((candidate) => candidate.id === row.seamId)
    : null;
  if (!seam) {
    return `
      <td class="cad-path-cell is-empty">—</td>
      <td class="trim-cell is-empty">—</td>
      <td class="direction-cell is-empty">—</td>
    `;
  }
  if (!isControlRow) {
    return `
      <td class="cad-path-cell is-reference" title="${escapeHTML(seam.label)}">${escapeHTML(seam.label)}</td>
      <td class="trim-cell is-empty">—</td>
      <td class="direction-cell is-empty">—</td>
    `;
  }
  const limits = seamTrimLimits(seam);
  return `
    <td class="cad-path-cell ${seam.enabled ? "" : "is-disabled"}" data-seam-control="${escapeHTML(seam.id)}">
      <label class="cad-path-control">
        <input data-seam-field="enabled" type="checkbox" ${seam.enabled ? "checked" : ""} aria-label="Use ${escapeHTML(seam.label)}" />
        <span>
          <strong title="${escapeHTML(seam.label)}">${escapeHTML(seam.label)}</strong>
          <small>${millimetreText(limits.remainingLengthM)} / ${millimetreText(limits.totalLengthM)} mm</small>
        </span>
      </label>
    </td>
    <td class="trim-cell" data-seam-control="${escapeHTML(seam.id)}">
      ${trimDualControl(seam.label, limits)}
    </td>
    <td class="direction-cell" data-seam-control="${escapeHTML(seam.id)}">
      <button
        class="weld-direction"
        data-seam-action="reverse"
        type="button"
        aria-label="Reverse ${escapeHTML(seam.label)} direction"
      >${seam.reversed ? "REVERSED" : "FORWARD"}</button>
    </td>
  `;
}

/**
 * One dual-handle trim control per seam: the left handle trims from the
 * start, the right handle trims from the end (it slides the KEPT span's end
 * position), and the orange fill between them is the weld that remains.
 * Exact values live in one compact "start – end mm" group beside it.
 */
function trimDualControl(
  seamLabel: string,
  limits: ReturnType<typeof seamTrimLimits>,
): string {
  const totalMm = limits.totalLengthM * 1000;
  const startMm = limits.startOffsetM * 1000;
  const endMm = limits.endOffsetM * 1000;
  const endPosMm = totalMm - endMm;
  const lo = totalMm > 0 ? startMm / totalMm * 100 : 0;
  const hi = totalMm > 0 ? endPosMm / totalMm * 100 : 100;
  return `
    <div class="trim-compact">
    <div class="trim-dual" style="--lo: ${numberCell(lo)}%; --hi: ${numberCell(hi)}%;">
      <div class="trim-dual-track" aria-hidden="true"></div>
      <input
        class="trim-dual-range"
        data-seam-field="trimStartMm"
        type="range"
        min="0"
        max="${numberCell(totalMm)}"
        step="0.1"
        value="${numberCell(startMm)}"
        aria-label="Trim from the start of ${escapeHTML(seamLabel)} in millimetres"
      />
      <input
        class="trim-dual-range"
        data-seam-field="trimEndPosMm"
        type="range"
        min="0"
        max="${numberCell(totalMm)}"
        step="0.1"
        value="${numberCell(endPosMm)}"
        aria-label="Keep ${escapeHTML(seamLabel)} up to this distance in millimetres"
      />
    </div>
    <span class="trim-values">
      <input
        data-seam-field="trimStartMm"
        type="number"
        min="0"
        max="${numberCell(limits.maxStartOffsetM * 1000)}"
        step="0.1"
        inputmode="decimal"
        value="${numberCell(startMm)}"
        aria-label="Trim from the start of ${escapeHTML(seamLabel)}, exact millimetres"
      />
      <span aria-hidden="true">–</span>
      <input
        data-seam-field="trimEndMm"
        type="number"
        min="0"
        max="${numberCell(limits.maxEndOffsetM * 1000)}"
        step="0.1"
        inputmode="decimal"
        value="${numberCell(endMm)}"
        aria-label="Trim from the end of ${escapeHTML(seamLabel)}, exact millimetres"
      />
      <span>mm</span>
    </span>
    </div>
  `;
}

function setPlaybackRowHighlight(rowId: string | null): void {
  const changed = playbackHighlightRowId !== rowId;
  playbackHighlightRowId = rowId;
  const row = rowId ? project.rows.find((candidate) => candidate.id === rowId) : undefined;
  previewHighlightNodeId = row ? rowNodeId(row) : null;
  applyPlaybackNodeHighlight(changed);
}

function activePlaybackNodeId(): string | null {
  return executedHighlightNodeId ?? (poseSource === "sim" ? previewHighlightNodeId : null);
}

function applyPlaybackNodeHighlight(scrollIntoView: boolean): void {
  applyNodeHighlight(query("#program-spreadsheet"), activePlaybackNodeId(), "is-playback-current", scrollIntoView);
  applyPlaybackRowHighlight(scrollIntoView);
}

function applyPlaybackRowHighlight(scrollIntoView: boolean): void {
  const host = document.querySelector<HTMLElement>("#program-spreadsheet");
  if (!host) return;
  let primaryMatch: HTMLElement | null = null;
  const nodeId = activePlaybackNodeId();
  const rowIds = new Set(project.rows.filter(row => nodeId !== null && rowNodeId(row) === nodeId).map(row => row.id));
  for (const row of host.querySelectorAll<HTMLTableRowElement>("tr[data-program-row]")) {
    const match = rowIds.has(row.dataset.programRow ?? "");
    row.classList.toggle("is-playback-current", match);
    if (match && !primaryMatch && !row.classList.contains("advanced-row")) {
      primaryMatch = row;
    }
  }
  if (scrollIntoView && primaryMatch) {
    primaryMatch.scrollIntoView({ block: "nearest", inline: "nearest" });
  }
}

function controlPointSpreadsheetRow(
  control: ProgramControlPoint,
  selected: boolean,
  isSeamControlRow: boolean,
): string {
  const row = control.row;
  const tcp = row.targetTcpPose;
  const rowClass = [
    "control-point-row",
    selected ? "is-selected" : "",
    playbackHighlightRowId === row.id ? "is-playback-current" : "",
  ].filter(Boolean).join(" ");
  const canUp = canReorderControlPoint(project, row.id, -1);
  const canDown = canReorderControlPoint(project, row.id, 1);
  const orderCell = `<td class="row-order" title="Reorder this control point in the path">
        <button data-row-action="up" type="button" ${canUp ? "" : "disabled"}>↑</button>
        <button data-row-action="down" type="button" ${canDown ? "" : "disabled"}>↓</button>
      </td>`;
  return `
    <tr class="${rowClass}" data-program-row="${escapeHTML(row.id)}">
      <td class="row-number">CP${control.number}</td>
      <td><input data-field="enabled" type="checkbox" ${row.enabled ? "checked" : ""} aria-label="Enable control point ${control.number}" /></td>
      <td><span class="program-kind">POINT</span></td>
      <td>${escapeHTML(row.phase.toUpperCase())}</td>
      <td><input data-field="label" class="cell-label" value="${escapeHTML(row.label)}" /></td>
      <td>${escapeHTML(row.moveType)}</td>
      <td class="vector-cell"><div class="vector-inputs">${[0, 1, 2].map((axis) =>
        `<input data-tcp-index="${axis}" type="number" step="0.001" value="${numberCell(tcp[axis]!)}" aria-label="${["X", "Y", "Z"][axis]} metres" />`
      ).join("")}</div></td>
      ${seamSpreadsheetCells(row, isSeamControlRow)}
      <td class="speed-cell"><input data-field="tcpVelocityMps" type="number" min="0" step="0.001" value="${numberCell(row.tcpVelocityMps)}" /></td>
      <td><span class="arc-state-label ${row.weldEnabled ? "is-on" : ""}">${row.weldEnabled ? "ARC ON" : "ARC OFF"}</span></td>
      <td><span class="source-state source-${escapeHTML(row.sourceState ?? "manual")}">${escapeHTML(row.sourceState ?? "manual")}</span></td>
      <td><span class="validation-state">${escapeHTML(row.validationState ?? "unknown")}</span></td>
      ${orderCell}
    </tr>
    ${showAdvancedProgramFields
      ? advancedSpreadsheetRow(row, control.rowIndex, selected)
      : ""}
  `;
}

function moveProgramSpreadsheetRow(move: ProgramMove, selectedRowID: string): string {
  const disclosed = showAdvancedProgramFields || disclosedMoveIDs.has(move.id);
  const selected = move.end.row.id === selectedRowID;
  const sampleCount = Math.max(0, move.rows.length - 2);
  const center = move.centerM;
  return `
    <tr
      class="program-move-row ${selected ? "is-selected" : ""}"
      data-program-move="${escapeHTML(move.id)}"
      data-program-row="${escapeHTML(move.end.row.id)}"
    >
      <td class="row-number">M${move.number}</td>
      <td>${move.end.row.enabled ? "●" : "○"}</td>
      <td><span class="program-kind is-move">MOVE</span></td>
      <td>${escapeHTML(move.end.row.command)}</td>
      <td><strong>${escapeHTML(move.start.row.label)} → ${escapeHTML(move.end.row.label)}</strong></td>
      <td><span class="move-kind move-${move.kind.toLowerCase()}">${escapeHTML(move.kind)}</span></td>
      <td class="move-summary">${move.rows.length} wire rows · ${sampleCount} hidden sample${sampleCount === 1 ? "" : "s"}</td>
      <td colspan="3">${center ? `CENTER ${center.map((value) => numberCell(value)).join(", ")} m` : "—"}</td>
      <td class="speed-cell">${numberCell(move.end.row.tcpVelocityMps)}</td>
      <td>${move.rows.some((row) => row.weldEnabled) ? "ARC ON" : "ARC OFF"}</td>
      <td>${move.modifiedExternally
        ? `<span class="source-state source-teach_modified">modified externally</span>`
        : "grouped"}</td>
      <td>${move.modifiedExternally
        ? escapeHTML(move.externalReason ?? "modified externally")
        : "valid"}</td>
      <td class="row-order"><button class="move-disclose-button" data-move-action="disclose" type="button" aria-expanded="${disclosed}">${disclosed ? "HIDE" : "SAMPLES"}</button></td>
    </tr>
    ${disclosed
      ? move.sampleRows.map((row) => sampleDisclosureRow(move, row, selectedRowID)).join("")
      : ""}
  `;
}

function sampleDisclosureRow(
  move: ProgramMove,
  row: AuthoredProgramRow,
  selectedRowID: string,
): string {
  const rowIndex = project.rows.indexOf(row);
  const sampleNumber = rowIndex - move.start.rowIndex;
  return `
    <tr
      class="move-sample-row ${row.id === selectedRowID ? "is-selected" : ""}"
      data-program-row="${escapeHTML(row.id)}"
    >
      <td class="row-number">↳${sampleNumber}</td>
      <td>${row.enabled ? "●" : "○"}</td>
      <td><span class="program-kind is-sample">SAMPLE</span></td>
      <td>${escapeHTML(row.phase.toUpperCase())}</td>
      <td>${escapeHTML(row.label)}</td>
      <td>${escapeHTML(row.moveType)}</td>
      <td class="vector-cell"><div class="vector-inputs">${[0, 1, 2].map((axis) =>
        `<input data-tcp-index="${axis}" type="number" step="0.001" value="${numberCell(row.targetTcpPose[axis]!)}" />`
      ).join("")}</div></td>
      <td colspan="3">M${move.number} internal row ${sampleNumber} / ${move.rows.length - 1}</td>
      <td class="speed-cell">${numberCell(row.tcpVelocityMps)}</td>
      <td>${row.weldEnabled ? "ARC ON" : "ARC OFF"}</td>
      <td>${escapeHTML(row.sourceState ?? "manual")}</td>
      <td>${escapeHTML(row.validationState ?? "unknown")}</td>
      <td>—</td>
    </tr>
    ${showAdvancedProgramFields ? advancedSpreadsheetRow(row, rowIndex, row.id === selectedRowID) : ""}
  `;
}

function spreadsheetRows(
  row: AuthoredProgramRow,
  index: number,
  selected: boolean,
  isSeamControlRow: boolean,
): string {
  const tcp = row.targetTcpPose;
  const playbackCurrent = playbackHighlightRowId === row.id;
  const rowClass = [
    selected ? "is-selected" : "",
    playbackCurrent ? "is-playback-current" : "",
  ].filter(Boolean).join(" ");
  return `
    <tr class="${rowClass}" data-program-row="${escapeHTML(row.id)}">
      <td class="row-number">${String(index + 1).padStart(2, "0")}</td>
      <td><input data-field="enabled" type="checkbox" ${row.enabled ? "checked" : ""} aria-label="Enable row ${index + 1}" /></td>
      <td><select data-field="rowType">${rowTypeOptions(row.rowType ?? "motion")}</select></td>
      <td><select data-field="command">${commandOptions(row.command)}</select></td>
      <td><input data-field="label" class="cell-label" value="${escapeHTML(row.label)}" /></td>
      <td><select data-field="moveType">${optionList(["LINE", "CURVE", "JOINT"], row.moveType)}</select></td>
      <td class="vector-cell"><div class="vector-inputs">${[0, 1, 2].map((axis) =>
        `<input data-tcp-index="${axis}" type="number" step="0.001" value="${numberCell(tcp[axis]!)}" aria-label="${["X", "Y", "Z"][axis]} metres" />`
      ).join("")}</div></td>
      ${seamSpreadsheetCells(row, isSeamControlRow)}
      <td class="speed-cell"><input data-field="tcpVelocityMps" type="number" min="0" step="0.001" value="${numberCell(row.tcpVelocityMps)}" /></td>
      <td class="arc-cell">
        <button
          class="arc-toggle ${row.weldEnabled ? "is-on" : ""}"
          data-row-action="toggle-arc"
          type="button"
          aria-pressed="${row.weldEnabled}"
          title="Toggle weld arc output for this move"
        >${row.weldEnabled ? "ARC ON" : "ARC OFF"}</button>
      </td>
      <td><span class="source-state source-${escapeHTML(row.sourceState ?? "manual")}">${escapeHTML(row.sourceState ?? "manual")}</span></td>
      <td><span class="validation-state">${escapeHTML(row.validationState ?? "unknown")}</span></td>
      <td class="row-order">
        <button data-row-action="up" type="button" ${index === 0 ? "disabled" : ""}>↑</button>
        <button data-row-action="down" type="button" ${index === project.rows.length - 1 ? "disabled" : ""}>↓</button>
      </td>
    </tr>
    ${showAdvancedProgramFields ? advancedSpreadsheetRow(row, index, selected) : ""}
  `;
}

function advancedSpreadsheetRow(
  row: AuthoredProgramRow,
  index: number,
  selected: boolean,
): string {
  const playbackCurrent = playbackHighlightRowId === row.id;
  const rowClass = [
    "advanced-row",
    selected ? "is-selected" : "",
    playbackCurrent ? "is-playback-current" : "",
  ].filter(Boolean).join(" ");
  const torchDisabled = row.phase === "weld"
    ? ""
    : `disabled title="Torch angles are authored on weld rows; approach and retract inherit the adjacent weld orientation."`;
  return `
    <tr class="${rowClass}" data-program-row="${escapeHTML(row.id)}">
      <td colspan="15">
        <div class="advanced-grid">
          <label>TCP RPY · rad<div class="vector-cell">${[3, 4, 5].map((axis) =>
            `<input data-tcp-index="${axis}" type="number" step="0.001" value="${numberCell(row.targetTcpPose[axis]!)}" />`
          ).join("")}</div></label>
          <label>J1–J9 · rad<div class="joint-vector">${row.targetJointsRad.map((value, joint) =>
            `<input data-joint-index="${joint}" title="J${joint + 1}" aria-label="J${joint + 1} radians" type="number" step="0.001" value="${numberCell(value)}" />`
          ).join("")}</div></label>
          <label>Tool frame<input data-field="toolFrameId" value="${escapeHTML(row.toolFrameId)}" /></label>
          <label>Work frame<input data-field="workFrameId" value="${escapeHTML(row.workFrameId)}" /></label>
          <label>Joint speed · rad/s<input data-field="jointVelocityRadS" type="number" min="0" step="0.001" value="${numberCell(row.jointVelocityRadS)}" /></label>
          <label>TCP accel · m/s²<input data-field="accelerationMps2" type="number" min="0" step="0.001" value="${numberCell(row.accelerationMps2)}" /></label>
          <label>Joint accel · rad/s²<input data-field="jointAccelerationRadS2" type="number" min="0" step="0.001" value="${numberCell(row.jointAccelerationRadS2)}" /></label>
          <label>Blend · m<input data-field="blendRadiusM" type="number" min="0" step="0.001" value="${numberCell(row.blendRadiusM)}" /></label>
          <label class="inline-check">Fine stop<input data-field="fineStop" type="checkbox" ${row.fineStop ? "checked" : ""} /></label>
          <label>Wire · m/min<input data-field="wireFeedMPerMin" type="number" min="0" step="0.1" value="${numberCell(row.wireFeedMPerMin)}" /></label>
          <label>Voltage · V<input data-field="voltageV" type="number" min="0" step="0.1" value="${numberCell(row.voltageV)}" /></label>
          <label>Current · A<input data-field="amperageA" type="number" min="0" step="0.1" value="${numberCell(row.amperageA ?? 0)}" /></label>
          <label>Weld profile<input data-field="weldProfileId" type="number" min="0" step="1" value="${row.weldProfileId}" /></label>
          <label>Weave profile<input data-field="weaveProfileId" type="number" min="0" step="1" value="${row.weaveProfileId}" /></label>
          <label>Work · °<input data-field="torchWorkAngleDeg" type="number" min="-89" max="89" step="1" value="${numberCell(rowTorchSettings(row).workAngleDeg)}" ${torchDisabled} /></label>
          <label title="Positive leans the torch body along the travel direction — the torch tip points back at the finished weld (drag/backhand). Negative = push.">Travel · ° (+ drag)<input data-field="torchTravelAngleDeg" type="number" min="-89" max="89" step="1" value="${numberCell(rowTorchSettings(row).travelAngleDeg)}" ${torchDisabled} /></label>
          <label>Spin · °<input data-field="torchSpinAngleDeg" type="number" min="-180" max="180" step="1" value="${numberCell(rowTorchSettings(row).spinAngleDeg)}" ${torchDisabled} /></label>
          <label>Weave<select data-field="weavePattern">${optionList(WEAVE_PATTERNS, rowWeaveSettings(row).pattern)}</select></label>
          <label>Weave amp · mm<input data-field="weaveAmplitudeMm" type="number" min="0" step="0.1" value="${numberCell(rowWeaveSettings(row).amplitudeMm)}" /></label>
          <label>Weave freq · Hz<input data-field="weaveFrequencyHz" type="number" min="0" step="0.1" value="${numberCell(rowWeaveSettings(row).frequencyHz)}" /></label>
          <label>Weave dwell · ms<input data-field="weaveDwellMs" type="number" min="0" step="10" value="${numberCell(rowWeaveSettings(row).dwellMs)}" /></label>
          <label>Positioner J7–J9<div class="vector-cell">${[6, 7, 8].map((joint, axis) =>
            `<input data-positioner-index="${joint}" title="J${joint + 1}" type="number" step="0.001" value="${numberCell(row.positionerTargetsRad[axis]!)}" />`
          ).join("")}</div></label>
          <label>Loop target<select data-field="loopTargetRowId"><option value="">—</option>${
            project.rows.map((candidate, targetIndex) => `<option value="${escapeHTML(candidate.id)}" ${
              row.loopTargetRowId === candidate.id ? "selected" : ""
            }>${String(targetIndex + 1).padStart(2, "0")} · ${escapeHTML(candidate.label)}</option>`).join("")
          }</select></label>
          <label>Loop count<input data-field="loopIterations" type="number" min="0" step="1" value="${row.loopIterations ?? 0}" /></label>
          <label>RevPi slot<input data-field="revPiIOSlot" type="number" min="0" step="1" value="${row.revPiIOSlot ?? ""}" /></label>
          <label class="advanced-comment">Comment<input data-field="comment" value="${escapeHTML(row.comment ?? "")}" /></label>
          <div class="row-identity">
            <span>row ${escapeHTML(row.id)}</span>
            <span>r${row.rowRevision ?? project.program.programRevision}</span>
            <span>${escapeHTML(row.rowHash ?? "hash pending")}</span>
            <span>${escapeHTML(row.seamId ?? "no CAD seam")} / ${escapeHTML(row.passId || "no pass")}</span>
          </div>
          ${row.sourceState === "teach_modified" ? `
            <div class="teach-resolution">
              <strong>Taught on SteamDeck</strong>
              <span>Keep this taught override or explicitly regenerate its CAD seam/pass row.</span>
              <button data-row-action="keep-taught" type="button">Keep taught</button>
              <button data-row-action="regenerate-cad" type="button">Regenerate from CAD</button>
            </div>
          ` : ""}
        </div>
      </td>
    </tr>
  `;
}

function renderProgramLint(): void {
  const host = query<HTMLElement>("#program-lint");
  const findings = lintProgram(project);
  host.hidden = findings.length === 0;
  if (findings.length === 0) {
    host.innerHTML = "";
    return;
  }
  const errors = findings.filter((finding) => finding.severity === "error").length;
  host.innerHTML = `
    <span class="lint-count ${errors > 0 ? "has-errors" : ""}">
      ${findings.length} check${findings.length === 1 ? "" : "s"}${errors > 0 ? ` · ${errors} error${errors === 1 ? "" : "s"}` : ""}
    </span>
    ${findings.slice(0, 4).map((finding) => `
      <button
        type="button"
        class="lint-finding is-${finding.severity}"
        data-lint-row="${escapeHTML(finding.rowId ?? "")}"
        title="${escapeHTML(finding.message)}"
      >${finding.rowNumber !== null ? `${String(finding.rowNumber).padStart(2, "0")} · ` : ""}${escapeHTML(finding.message)}</button>
    `).join("")}
    ${findings.length > 4 ? `<span class="lint-more">+${findings.length - 4} more</span>` : ""}
  `;
  host.onclick = (event) => {
    const rowID = (event.target as HTMLElement)
      .closest<HTMLButtonElement>("[data-lint-row]")?.dataset.lintRow;
    const row = project.rows.find((candidate) => candidate.id === rowID);
    if (!row) return;
    project.simulation.selectedRowId = row.id;
    onRowSelectionChanged();
    renderProgramSpreadsheet();
    const nodeId = rowNodeId(row);
    if (nodeId && !activeExecution) {
      selectNode(nodeId);
      nodeEditor.open(nodeId);
    }
  };
}

async function jogRequest(path: LiveJogPath, body?: unknown): Promise<JogResponse> {
  const response = await densePanel.fetcher(liveJogUrl(path), {
    method: path === "state" ? "GET" : "POST",
    headers: body ? { "Content-Type": "application/json" } : undefined,
    body: body ? JSON.stringify(body) : undefined,
  });
  let payload: unknown = null;
  try {
    payload = await response.json();
  } catch {
    payload = { ok: false, error: `jog ${path} HTTP ${response.status}` };
  }
  const parsed = parseJogResponse(payload, jogOperatorFence);
  if (!parsed.ok && parsed.error === "") {
    parsed.error = `jog ${path} HTTP ${response.status}`;
  }
  return parsed;
}

function markJogLive(live: boolean, error = ""): void {
  jogLive = live;
  jogLastError = live ? "" : error;
  if (!live && !densePanel.targetSelected()) {
    // The preview's strip has nothing to say once its jog session is gone;
    // a connected machine paints its own reason from the machine view.
    const host = document.querySelector<HTMLElement>("#jog-operator-feedback")!;
    host.textContent = ""; host.hidden = true;
  }
  if (live && jogStatePoll === null) {
    jogStatePoll = window.setInterval(() => void pollJogState(), 100);
  }
  if (!live && jogStatePoll !== null) {
    window.clearInterval(jogStatePoll);
    jogStatePoll = null;
  }
  syncSpreadsheetCartesianJog();
}

function scheduleJogRestart(delayMs = 2500): void {
  if (densePanel.targetSelected() || jogLive || jogRestartTimer !== null || jogStartInFlight) return;
  jogRestartTimer = window.setTimeout(() => {
    jogRestartTimer = null;
    void startLiveJog();
  }, delayMs);
}

async function startLiveJog(): Promise<void> {
  if (densePanel.targetSelected() || jogStartInFlight || jogLive) return;
  jogStartInFlight = true;
  try {
    for (let attempt = 0; shouldKeepTryingJogStart(jogLive, attempt); attempt += 1) {
      if (densePanel.targetSelected()) return;
      try {
        const current = await jogRequest("state");
        if (jogSessionIsLive(current)) {
          markJogLive(true);
          return;
        }
        const started = await jogRequest("start");
        if (jogSessionIsLive(started)) {
          markJogLive(true);
          return;
        }
        jogLastError = started.error || current.error || "Motion Server jog session is not running";
      } catch (error) {
        jogLastError = error instanceof Error ? error.message : "Motion Server jog session is not running";
      }
      syncSpreadsheetCartesianJog();
      if (!shouldKeepTryingJogStart(false, attempt + 1)) break;
      await new Promise((resolve) => window.setTimeout(resolve, nextJogRetryDelayMs(attempt)));
    }
    markJogLive(false, jogLastError);
  } finally {
    jogStartInFlight = false;
  }
  if (!jogLive) scheduleJogRestart(5000);
}

async function pollJogState(): Promise<void> {
  if (!jogLive || densePanel.targetSelected()) return;
  try {
    const result = await jogRequest("state");
    const state = result.state;
    const feedbackHost = document.querySelector<HTMLElement>("#jog-operator-feedback")!;
    // The preview shows one line for a refusal it made, nothing while idle;
    // the per-axis fence dump is a real cell's, not the browser's.
    const feedbackLine = previewFeedbackLine(state?.operatorFeedback ?? null);
    feedbackHost.textContent = feedbackLine;
    feedbackHost.hidden = !feedbackLine;
    if (!jogSessionIsLive(result) || !state) {
      markJogLive(false, result.error || "Motion Server jog session stopped");
      scheduleJogRestart();
      return;
    }
    // With the viewer on the real robot, the daemon's feedback owns the pose.
    if (poseSource === "real") return;
    const names: string[] = state.axisNames ?? [];
    const joints: number[] = state.jointsRad ?? [];
    const byName = new Map(names.map((name, index) => [name, joints[index] ?? 0]));
    const jointOrder = [...project.robot.jointOrder];
    const jointsRad = jointOrder.map((name) => byName.get(name) ?? 0);
    // A preview owns the pose while it plays and keeps it after: this 10 Hz
    // feed writes only when the jogged robot moves. A rails replay does not
    // drive the jog session, and its idle reset pose written on every poll
    // flashed the robot back between playback frames.
    if (!jogFeed.admit(jointsRad, playing || railsPlaying)) return;
    viewer.setPose({
      modelId: project.robot.model,
      jointOrder,
      jointsRad,
      source: "live",
    }, true);
  } catch {
    markJogLive(false, "Motion Server jog state poll failed");
    scheduleJogRestart();
  }
}

/**
 * Explicit ARM/DISARM for simulated jog drive. Arming never touches physical
 * output — the backend serve runs with execution disabled and the jog daemon
 * is an isolated SIMULATION_MODE cartesiand. Disarming is the dominant stop:
 * it halts any held intent immediately via POST jog/halt (release edge).
 */
function toggleSimArm(): void {
  // In production the pill mirrors the real robot's servo fact; this UI
  // never arms physical drives.
  if (densePanel.targetSelected() || project.program.executionMode === "production") return;
  setSimArmed(!simArmed);
}

function setSimArmed(next: boolean): void {
  if (simArmed === next) return;
  simArmed = next;
  if (!next) {
    stopJogHold();
  }
  syncSpreadsheetCartesianJog();
  renderSessionStatusPills();
}

/** Route a hold to the selected controller, or the explicitly armed preview. */
function holdJogIntent(intent: { mode: string; axes: number[]; selectedJoint?: number }): void {
  if (jogHoldTimer !== null) stopJogHold();
  if (robotModelClass(densePanel.cellModel()) !== "arm") return;
  if (densePanel.targetSelected()) {
    if (jogPadVisible) densePanel.cartesianJog()?.begin(intent.mode, intent.axes, Number(query<HTMLSelectElement>("#cartesian-speed").value), {
      distanceMM: Number(query<HTMLInputElement>("#cartesian-distance").value), angleDegrees: Number(query<HTMLInputElement>("#cartesian-angle").value),
    });
    return;
  }
  if (!jogPadVisible || !jogLive || !simArmed) return;
  const send = () => {
    if (densePanel.targetSelected() || !jogPadVisible || !jogLive || !simArmed) {
      stopJogHold();
      return;
    }
    let mode = intent.mode;
    let axes = intent.axes;
    if (mode === "tool") {
      // The server consumes base-frame twists. Reuse the live tool orientation
      // for each send so the held axes follow the tool as it rotates.
      const linear = viewer.rotateTcpVectorToWorld([axes[0]!, axes[1]!, axes[2]!]);
      const angular = viewer.rotateTcpVectorToWorld([axes[3]!, axes[4]!, axes[5]!]);
      axes = [...linear, ...angular];
      mode = "base";
    }
    void jogRequest("intent", {
      mode,
      axes,
      selectedJoint: intent.selectedJoint ?? 0,
      speedScale: 1,
    }).then((result) => {
      if (!jogSessionIsLive(result)) {
        markJogLive(false, result.error || "Jog intent rejected: simulator session is not running");
        scheduleJogRestart();
      }
    }).catch((error: unknown) => {
      markJogLive(false, error instanceof Error ? error.message : "Simulator jog intent failed");
      scheduleJogRestart();
    });
  };
  send();
  jogHoldTimer = window.setInterval(send, JOG_HOLD_MS);
}

/** End each hold once and surface halt failures, preventing retries and a falsely live pad. */
function stopJogHold(cause: JogEndCause = "pointer_up"): void {
  densePanel.cartesianJog()?.end(cause);
  if (jogHoldTimer === null) return;
  window.clearInterval(jogHoldTimer);
  jogHoldTimer = null;
  void jogRequest("halt").then((result) => {
    if (!result.ok) {
      markJogLive(false, result.error || "Simulator jog halt rejected");
      scheduleJogRestart();
    }
  }).catch((error: unknown) => {
    markJogLive(false, error instanceof Error ? error.message : "Simulator jog halt failed");
    scheduleJogRestart();
  });
}

/** Halt before hiding, so persistence failures cannot leave an active hold. */
function toggleJogPad(): void {
  if (jogPadVisible) { stopJogHold("ui_navigation"); densePanel.endJogHolds("ui_navigation"); }
  jogPadVisible = !jogPadVisible;
  writeJogPadVisible(jogPadVisible);
  jogAction.checked = jogPadVisible;
  applyTelemetryVisible();
  applySpreadsheetShown(stageActions.spreadsheet.checked);
  syncSpreadsheetCartesianJog();
}

/** Show or hide the telemetry panel; sampling continues either way, so the
 *  history is there when it comes back. */
function toggleTelemetryPanel(): void {
  telemetryVisible = !telemetryVisible;
  writeTelemetryVisible(telemetryVisible);
  telemetryAction.checked = telemetryVisible;
  applyTelemetryVisible();
  applySpreadsheetShown(stageActions.spreadsheet.checked);
}

/** The dock pane holds two views, the Jog controls and the telemetry; it shows
 *  while either is on. Telemetry alone is watching the drives without being
 *  one tap from jogging them (owner, 2026-09-27). */
function dockPaneShown(): boolean {
  return jogPadVisible || telemetryVisible;
}

/** The strip, its boundary and the pane's grid follow the toggle; the dock's
 *  floor drops to the joints table's when the strip is away, and the Jog
 *  column steps aside when only the strip is on. */
function applyTelemetryVisible(): void {
  const shown = telemetryVisible;
  query("#telemetry-strip").hidden = !shown;
  query("#jog-column-resizer").hidden = !shown || !jogPadVisible;
  query("#jog-panel").classList.toggle("is-strip-hidden", !shown);
  query("#jog-panel").classList.toggle("is-controls-hidden", !jogPadVisible);
  query("#jog-panel").hidden = !dockPaneShown();
  query("#bottom-workspace").classList.toggle("strip-hidden", !shown);
  resizableLayout.apply();
  // Built here, not on the first sample: an operator with no cell connected
  // still sees the panel's tabs and empty plot, not a blank half-pane (owner,
  // 2026-09-20: "telemetry panel active but still hidden").
  if (shown) telemetryStrip().render();
}

/** Keep the pad tied to the session and ARM gates; disablement ends a hold. */
function syncSpreadsheetCartesianJog(): void {
  const pad = query("#spreadsheet-jog");

  const frame = query<HTMLSelectElement>("#jog-frame");
  const grid = query("#spreadsheet-cartesian-grid");
  const connected = densePanel.targetSelected();
  const machine = densePanel.cartesianJog();
  const bench = robotModelClass(densePanel.cellModel()) !== "arm";
  const presentation = cartesianPadPresentation(densePanel.flowView(), densePanel.cellModel());
  const zero = query<HTMLButtonElement>("#cartesian-zero");
  const zeroGate = connected ? densePanel.zeroJointsGate() : "Connect a machine first";
  zero.disabled = zeroGate !== null;
  zero.textContent = densePanel.zeroJointsLabel();
  zero.title = zeroGate ?? ZERO_BUTTON_TITLE;
  query("#cartesian-jog-tag").textContent = presentation.tag;
  query("#cartesian-jog-tag").classList.toggle("is-preview", !connected);
  query("#cartesian-jog-target").textContent = presentation.heading;
  query("#cartesian-jog-target").dataset.posture = densePanel.flowView()?.posture ?? "none";
  query("#sim-arm-toggle").hidden = !presentation.previewArmVisible;
  query("#cartesian-speed-label").hidden = !connected;
  // A path hold changes speed within the hold; a Step keeps its speed.
  query<HTMLSelectElement>("#cartesian-speed").disabled = bench || !!machine?.stepBusy;
  query("#cartesian-step-controls").hidden = !connected;
  query<HTMLSelectElement>("#cartesian-mode").value = machine?.mode ?? "step";
  query<HTMLSelectElement>("#cartesian-mode").disabled = bench || !!machine?.busy;
  const labels = cartesianStepLabels(machine?.mode ?? "step");
  query("#cartesian-distance-label").textContent = labels.linear;
  query("#cartesian-angle-label").textContent = labels.angular;
  query("#cartesian-step-help").textContent = labels.help;
  for (const id of ["#cartesian-distance", "#cartesian-angle"]) query<HTMLInputElement>(id).disabled = bench || !!machine?.busy;
  const reason = query("#cartesian-machine-reason");
  reason.hidden = !connected;
  reason.textContent = machine?.reason ?? "Connect and arm the machine";
  if (connected) {
    const feedback = query<HTMLElement>("#jog-operator-feedback");
    feedback.textContent = machine?.reason || "";
    feedback.hidden = !feedback.textContent;
    if (jogHoldTimer !== null) stopJogHold("cell_change");
  }
  const disabled = !machine || !!machine.disabled || (!!machine.busy && machine.mode === "step");
  const view = bench
    ? { buttonDisabled: true, frameDisabled: true, gridDisabled: true, title: presentation.heading }
    : connected
    ? { buttonDisabled: disabled, frameDisabled: disabled || !!machine?.busy, gridDisabled: disabled, title: machine?.disabled || (machine?.mode === "step" ? "Press once to move the selected machine by the entered distance or angle" : "Hold to jog the selected machine; release to stop") }
    : { buttonDisabled: true, frameDisabled: true, gridDisabled: true, title: "Connect and arm a cell to jog" };
  if ((!jogPadVisible || !!machine?.disabled) && machine?.busy) machine.end(machine.disabled ? "jog_disabled" : "ui_navigation");
  if ((!jogPadVisible || view.buttonDisabled) && jogHoldTimer !== null) stopJogHold(view.buttonDisabled ? "jog_disabled" : "ui_navigation");
  pad.hidden = !jogPadVisible;
  pad.title = view.title;


  // The toolbar toggles joint and Cartesian controls as one pane; telemetry
  // keeps the pane open without them.
  const panel = document.querySelector<HTMLElement>("#jog-panel");
  if (panel) panel.hidden = !dockPaneShown();
  frame.disabled = view.frameDisabled;
  frame.title = view.title;
  grid.title = view.title;
  grid.classList.toggle("is-disabled", view.gridDisabled);
  grid.setAttribute("aria-disabled", String(view.gridDisabled));
  grid.querySelectorAll<HTMLButtonElement>(".jog-hold").forEach((button) => {
    button.disabled = view.buttonDisabled;
    button.title = view.title;
  });
}

/** Wire release/cancel edges with the hold, preventing a stuck repeat timer. */
function wireSpreadsheetCartesianJog(): void {
  const steps = readCartesianSteps();
  query<HTMLInputElement>("#cartesian-distance").value = String(steps.distanceMM);
  query<HTMLInputElement>("#cartesian-angle").value = String(steps.angleDegrees);
  for (const id of ["#cartesian-distance", "#cartesian-angle"]) query<HTMLInputElement>(id).addEventListener("change", () => {
    writeCartesianSteps({ distanceMM: Number(query<HTMLInputElement>("#cartesian-distance").value), angleDegrees: Number(query<HTMLInputElement>("#cartesian-angle").value) });
  });
  query<HTMLSelectElement>("#cartesian-speed").addEventListener("change", () => densePanel.cartesianJog()?.setFraction(Number(query<HTMLSelectElement>("#cartesian-speed").value)));
  query<HTMLSelectElement>("#cartesian-mode").addEventListener("change", event => {
    densePanel.cartesianJog()?.setMode((event.target as HTMLSelectElement).value === "hold" ? "hold" : "step");
  });
  const grid = query("#spreadsheet-cartesian-grid");
  grid.onpointerdown = (event: PointerEvent) => {
    if (!jogPadVisible || (!densePanel.targetSelected() && (!jogLive || !simArmed)) || event.button !== 0) return;
    const button = (event.target as HTMLElement).closest<HTMLButtonElement>(".jog-hold");
    if (!button || button.disabled) return;
    event.preventDefault();
    holdJogIntent({
      mode: query<HTMLSelectElement>("#jog-frame").value,
      axes: jogButtonAxes(button.dataset.jogAxes!.split(",").map(Number)),
    });
  };
  grid.onpointerup = () => stopJogHold("pointer_up");
  grid.onpointerleave = () => stopJogHold("pointer_leave");
  grid.onpointercancel = () => stopJogHold("pointer_cancel");

  syncSpreadsheetCartesianJog();
}

function onRowSelectionChanged(): void {
  // Focus mode: fade the rest of the program so the selected move reads
  // clearly, and relabel around the selection.
  syncAuthoringOverlays(viewer, project, { showLabels: showPathLabels });
}

function renderProcessDefaults(): void {
  const process = project.processParameters[0];
  const resolved = processTorchDefaults(process);
  const values: Record<string, string> = {
    process: process?.process ?? "gmaw",
    travelSpeedMps: numberCell(process?.travelSpeedMps ?? DEFAULT_TRAVEL_SPEED_MPS),
    approachSpeedMps: numberCell(process?.approachSpeedMps ?? 0.1),
    retractSpeedMps: numberCell(process?.retractSpeedMps ?? 0.1),
    voltageV: numberCell(process?.voltageV ?? 0),
    amperageA: numberCell(process?.amperageA ?? 0),
    wireFeedMPerMin: numberCell(process?.wireFeedMPerMin ?? 0),
    torchWorkAngleDeg: numberCell(resolved.workAngleDeg),
    torchTravelAngleDeg: numberCell(resolved.travelAngleDeg),
    torchSpinAngleDeg: numberCell(resolved.spinAngleDeg),
    weavePattern: resolved.pattern,
    weaveAmplitudeMm: numberCell(resolved.amplitudeMm),
    weaveFrequencyHz: numberCell(resolved.frequencyHz),
    weaveDwellMs: numberCell(resolved.dwellMs),
  };
  document.querySelectorAll<HTMLInputElement | HTMLSelectElement>("[data-process-field]")
    .forEach((input) => {
      const field = input.dataset.processField ?? "";
      if (field in values) input.value = values[field]!;
      input.disabled = !process;
    });
}

function updateProcessDefaults(event: Event): void {
  const input = event.currentTarget as HTMLInputElement | HTMLSelectElement;
  const field = input.dataset.processField;
  if (!field || !project.processParameters[0]) return;
  try {
    mutate("process", (draft) => {
      const process = draft.processParameters[0]!;
      if (field === "process") {
        if (!["none", "gmaw", "gtaw", "laser", "custom"].includes(input.value)) {
          throw new Error("weld process is unsupported");
        }
        process.process = input.value as typeof process.process;
        return;
      }
      if (field === "weavePattern") {
        process.weavePattern = normalizeWeavePattern(input.value);
        return;
      }
      const numeric = finiteCell(input.value);
      if (field === "torchWorkAngleDeg" || field === "torchTravelAngleDeg") {
        (process as unknown as Record<string, unknown>)[field] = clampNumber(numeric, -89, 89);
      } else if (field === "torchSpinAngleDeg") {
        process.torchSpinAngleDeg = clampNumber(numeric, -180, 180);
      } else if ([
        "travelSpeedMps",
        "approachSpeedMps",
        "retractSpeedMps",
        "voltageV",
        "amperageA",
        "wireFeedMPerMin",
        "weaveAmplitudeMm",
        "weaveFrequencyHz",
        "weaveDwellMs",
      ].includes(field)) {
        (process as unknown as Record<string, unknown>)[field] = Math.max(0, numeric);
      } else {
        throw new Error(`weld process field ${field} is unsupported`);
      }
    });
  } catch (error) {
    setStorageStatus(`Process edit blocked: ${message(error)}`);
    renderProcessDefaults();
  }
}

function rowTypeOptions(selected: RowType): string {
  return optionList(
    ["motion", "command", "comment", "capture", "calibration", "io"],
    selected,
  );
}

function commandOptions(selected: AuthoredProgramRow["command"]): string {
  return optionList(
    ["MOVE", "WELD_ON", "WELD_OFF", "HOME", "LOOP", "COMMENT", "CAPTURE", "CALIBRATE", "IO"],
    selected,
  );
}

function optionList(values: readonly string[], selected: string): string {
  return values.map((value) =>
    `<option value="${escapeHTML(value)}" ${value === selected ? "selected" : ""}>${escapeHTML(value)}</option>`
  ).join("");
}

function finiteCell(value: string): number {
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) throw new Error("numeric cell must be finite");
  return parsed;
}

function clampNumber(value: number, minimum: number, maximum: number): number {
  return Math.min(Math.max(value, minimum), maximum);
}

function numberCell(value: number): string {
  return Number.isFinite(value) ? String(Number(value.toFixed(6))) : "0";
}

function millimetreText(valueM: number): string {
  const millimetres = valueM * 1000;
  return Number.isInteger(Number(millimetres.toFixed(1)))
    ? String(Math.round(millimetres))
    : millimetres.toFixed(1);
}

function chooseStepFiles(kind: "weld" | "fixture"): void {
  if (stepImportInFlight || workpieceSwitchInFlight) return;
  stepImportKind = kind;
  query<HTMLInputElement>("#step-file").click();
}

async function importSTEP(event: Event): Promise<void> {
  const input = event.currentTarget as HTMLInputElement;
  const files = Array.from(input.files ?? []);
  const kind = stepImportKind;
  const batchProjectId = project.identity.id;
  input.value = "";
  const failed: string[] = [];
  for (const file of files) {
    if (project.identity.id !== batchProjectId) {
      setCADStatus("Remaining imports cancelled because the open program changed");
      return;
    }
    if (!await importStepBytes(file.name, async () => new Uint8Array(await file.arrayBuffer()), kind)) failed.push(file.name);
  }
  if (files.length > 1 && failed.length) setCADStatus(`Imported ${files.length - failed.length} of ${files.length} files. Could not import: ${failed.join(", ")}. See Logs for details.`);
}

/** One STEP import path for a picked file and an example: through local
 *  CadQuery, into the open project, with the rails mode told the source. */
async function importStepBytes(filename: string, readBytes: Uint8Array | (() => Promise<Uint8Array>), kind: "weld" | "fixture" = "weld"): Promise<boolean> {
  if (stepImportInFlight || workpieceSwitchInFlight) return false;
  const lock = editingLockReason();
  if (lock) { setCADStatus(lock); return false; }
  stepImportInFlight = true;
  const importProjectId = project.identity.id;
  importingPartFilename = filename;
  importingKind = kind;
  importingWorkpieceId = null;
  renderProgramTree();
  const buttons = [query<HTMLButtonElement>("#import-step"), query<HTMLButtonElement>("#open-example")];
  for (const button of buttons) {
    button.disabled = true;
    button.classList.add("is-loading");
  }
  query("#import-step").textContent = "Importing…";
  markBusy(query("#import-step"), true);
  setCADStatus(`Importing ${filename} through local CadQuery…`);
  try {
    const bytes = typeof readBytes === "function" ? await readBytes() : readBytes;
    const source: CadSTEPSource = { kind: "bytes", filename, bytes };
    if (kind === "fixture") {
      const mesh = await cadClient.tessellate({ source });
      const fingerprint = mesh.fingerprint.toLowerCase();
      await repository.putSource({ fingerprint, filename, bytes });
      if (project.identity.id !== importProjectId) throw new Error("The open program changed during import; choose the files again in this program");
      workcellFixtureMeshes.set(fingerprint, mesh);
      const fixture = newStepFixture(workcellFixtures(project), { filename, fingerprint });
      mutate("workcell", draft => { draft.workcellFixtures.push(fixture); });
      setSelection({ kind: "fixture", id: fixture.id });
      setCADStatus(`Added fixture ${filename} · use Edit fixture to position it`);
      void refreshRecentParts();
      return true;
    }
    const topology = await cadClient.extractTopology({ source, sampleCount: 64 });
    // Keep the source before committing the new part, so every part in a
    // saved workspace can be selected again without losing its geometry.
    await repository.putSource({ fingerprint: topology.fingerprint, filename, bytes });
    if (project.identity.id !== importProjectId) throw new Error("The open program changed during import; choose the files again in this program");
    if (viewer.workpieceGizmoEnabled) setPartGizmo(false);
    if (railsWeldsCard.shown) setRailsWeldsOpen(false);
    if (waypointWeldsCard.shown) setWaypointWeldsOpen(false);
    const previous = project;
    const firstPart = projectWorkpieces(project).length === 0;
    project = appendCadQueryTopology(project, topology);
    importingWorkpieceId = activeWorkpieceId(project);
    if (firstPart) project.identity.name = filename.replace(/\.(step|stp)$/i, "");
    activeCADSource = source;
    void refreshRecentParts();
    railsMode.reset();
    if (project.railsAuthoring) syncRailsModeToProject();
    history.push(previous);
    resetWeldAuthoringUiState();
    const nextStep = project.railsAuthoring ? "loading rail welds" : "select edges to define welds";
    changed(`Imported ${filename} · ${nextStep}`);
    setCADStatus(`${topology.edges.length} topology edges · ${nextStep}`);
    await Promise.all([loadWorkpieceMesh(source), project.railsAuthoring ? railsStartup : Promise.resolve()]);
    setSelection({ kind: "part" });
    return true;
  } catch (error) {
    setCADStatus(`STEP import blocked: ${message(error)}`);
    return false;
  } finally {
    stepImportInFlight = false;
    importingPartFilename = null;
    importingWorkpieceId = null;
    renderProgramTree();
    for (const button of buttons) {
      button.disabled = false;
      button.classList.remove("is-loading");
    }
    query("#import-step").textContent = "Import STEP";
    markBusy(query("#import-step"), false);
    query<HTMLButtonElement>("#open-example").disabled = stepExamples.length === 0;
  }
}

// ---------------------------------------------------------------------------
// Example parts: staged by scripts/stage-assets.mjs from where they live in
// the repository, listed in examples/manifest.json, fetched on demand and
// imported exactly as a picked file would be.
// ---------------------------------------------------------------------------


function examplesUrl(name: string): string {
  return `${import.meta.env.BASE_URL}examples/${name}`;
}

async function loadStepExamples(): Promise<void> {
  const select = query<HTMLSelectElement>("#step-example");
  try {
    const response = await fetch(examplesUrl("manifest.json"));
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const manifest = await response.json() as { examples?: StepExample[] };
    stepExamples = (manifest.examples ?? []).filter((example) =>
      typeof example.filename === "string" && typeof example.label === "string"
    );
  } catch (error) {
    stepExamples = [];
    select.innerHTML = `<option value="">Examples unavailable</option>`;
    select.title = `examples/manifest.json did not load: ${message(error)} (run npm run stage:assets)`;
    return;
  }
  select.innerHTML = stepExamples.map((example) =>
    `<option value="${escapeHTML(example.id)}">${escapeHTML(example.label)}</option>`
  ).join("");
  select.disabled = stepExamples.length === 0;
  query<HTMLButtonElement>("#open-example").disabled = stepExamples.length === 0;
  renderStepExampleNote();
  // The tree marks demo parts by these names; it drew before they arrived.
  renderProgramTree();
  renderInspector();
}

function renderStepExampleNote(): void {
  const select = query<HTMLSelectElement>("#step-example");
  const example = stepExamples.find((candidate) => candidate.id === select.value);
  query("#step-example-note").textContent = example
    ? `${example.filename} · ${(example.bytes / 1024).toFixed(0)} KB. ${example.description}`
    : "";
}

async function openStepExample(): Promise<void> {
  await openStepExampleById(query<HTMLSelectElement>("#step-example").value);
}

async function openStepExampleById(id: string): Promise<void> {
  const example = stepExamples.find((candidate) => candidate.id === id);
  if (!example) return;
  await importStepBytes(example.filename, async () => {
    setCADStatus(`Fetching example ${example.filename}…`);
    const response = await fetch(examplesUrl(example.filename));
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    return new Uint8Array(await response.arrayBuffer());
  });
}

async function startPlayback(): Promise<void> {
  if (playing) {
    // Pause keeps the program cursor — same semantics a teach pendant expects.
    pausePlaybackKeepCursor();
    return;
  }
  if (
    project.plan.state !== "server_validated" ||
    project.plan.planner !== "motion_server_tesseract" ||
    !project.plan.acceptedPlan ||
    project.plan.sourceProjectRevision !== project.identity.revision
  ) {
    setStorageStatus("Robot playback needs a current accepted Tesseract trajectory. Use Plan & simulate offline.");
    return;
  }
  const motion = trajectoryMotionSummary(project.plan.trajectory);
  if (project.plan.trajectory.length < 2 || motion.durationS <= 0) {
    setStorageStatus("The accepted trajectory has no timed motion to play.");
    return;
  }
  if (!motion.robotMoves) {
    setStorageStatus("The accepted trajectory contains no joint-position change, so the robot cannot move.");
    return;
  }
  if (playbackCursorS >= motion.durationS - 1e-9) {
    playbackCursorS = 0;
    updateTimeline(motion.durationS);
  }
  const startSample = sampleAtTime(
    project.plan.trajectory,
    playbackCursorS,
  ).sample;
  if (!startSample) return;
  const currentJoints = currentViewerJoints();
  if (!jointsMatch(currentJoints, startSample.jointPositionsRad)) {
    await planAndRunViewerJointMove(
      [...startSample.jointPositionsRad] as JointVector,
      "program_start",
      "MOVEJ TO PROGRAM",
      beginProgramPlayback,
    );
    return;
  }
  beginProgramPlayback();
}

function beginProgramPlayback(): void {
  setRunPanel("simulate", "auto");
  playing = true;
  viewerMotion = null;
  stepPlayback = null;
  playStartedAt = performance.now();
  playStartCursor = playbackCursorS;
  setPlaybackRowHighlight(
    rowIdAtProgramCursor(playbackCursorS),
  );
  setMotionControls("program");
  setViewerStatus(
    `Running accepted Tesseract trajectory · ${project.plan.trajectory.length} J1–J9 samples`,
  );
  setStorageStatus("Simulating the exact accepted Robot V4 trajectory · no physical robot output sent");
  playbackFrame = requestAnimationFrame(tickPlayback);
}

function rowIdAtProgramCursor(timeS: number): string | null {
  return rowIdForTrajectorySample(
    project.rows,
    sampleAtTime(project.plan.trajectory, timeS).sample,
  );
}

function tickPlayback(now: number): void {
  if (!playing || viewerMotion) return;
  const elapsed = ((now - playStartedAt) / 1000) * playbackRate;
  const duration = project.plan.trajectory.at(-1)?.timeS ?? 0;
  const step = stepPlayback;

  let time: number;
  if (step) {
    time = step.direction === "forward"
      ? playStartCursor + elapsed
      : playStartCursor - elapsed;
    const reached = step.direction === "forward"
      ? time >= step.targetTimeS - 1e-9
      : time <= step.targetTimeS + 1e-9;
    time = step.direction === "forward"
      ? Math.min(time, step.targetTimeS)
      : Math.max(time, step.targetTimeS);
    const resolved = sampleAtTime(project.plan.trajectory, time);
    playbackCursorS = time;
    applyProjectSample(viewer, project, resolved.sample);
    setPlaybackRowHighlight(rowIdForTrajectorySample(project.rows, resolved.sample));
    updateTimeline(duration);
    if (reached) {
      finishStepPlayback(resolved.sample);
      return;
    }
    playbackFrame = requestAnimationFrame(tickPlayback);
    return;
  }

  time = playStartCursor + elapsed;
  const resolved = sampleAtTime(project.plan.trajectory, time);
  playbackCursorS = resolved.done ? duration : time;
  applyProjectSample(viewer, project, resolved.sample);
  setPlaybackRowHighlight(rowIdForTrajectorySample(project.rows, resolved.sample));
  updateTimeline(duration);
  if (resolved.done) {
    if (project.simulation.loop) {
      stopPlayback();
      playbackCursorS = 0;
      updateTimeline(duration);
      void startPlayback();
      return;
    } else {
      stopPlayback();
      return;
    }
  }
  playbackFrame = requestAnimationFrame(tickPlayback);
}

function finishStepPlayback(sample: PlanTrajectorySample | null): void {
  const direction = stepPlayback?.direction ?? "forward";
  playing = false;
  stepPlayback = null;
  cancelAnimationFrame(playbackFrame);
  setMotionControls("idle");
  updateStepControls(true);
  const rowLabel = sample ? `row ${sample.rowIndex + 1}` : "move";
  setViewerStatus(
    `Step ${direction} complete · ${rowLabel} · ${playbackCursorS.toFixed(1)} s`,
  );
  setStorageStatus(`Executed one ${direction} trajectory move along the accepted plan · no physical robot output sent`);
}

function stopPlayback(): void {
  const stoppedViewerMotion = viewerMotion;
  const wasStepping = stepPlayback !== null;
  playing = false;
  viewerMotion = null;
  stepPlayback = null;
  cancelAnimationFrame(playbackFrame);
  renderPoseSourceToggle();
  setPlaybackRowHighlight(null);
  setMotionControls("idle");
  updateTimeline(project.plan.trajectory.at(-1)?.timeS ?? 0);
  if (stoppedViewerMotion) {
    setViewerStatus(
      `${stoppedViewerMotion.label} stopped · holding the current J1–J9 position`,
    );
  } else if (wasStepping) {
    setViewerStatus(
      `Step stopped · holding J1–J9 at ${playbackCursorS.toFixed(1)} s on the accepted trajectory`,
    );
  }
}

function hasExactProgramPlayback(): boolean {
  return (
    project.plan.state === "server_validated" &&
    project.plan.planner === "motion_server_tesseract" &&
    project.plan.acceptedPlan !== null &&
    project.plan.sourceProjectRevision === project.identity.revision
  );
}

function pausePlaybackKeepCursor(): void {
  playing = false;
  stepPlayback = null;
  cancelAnimationFrame(playbackFrame);
  setMotionControls("idle");
  updateStepControls(hasExactProgramPlayback());
}

/**
 * Execute one program move along the accepted trajectory at planned timing.
 * Forward advances through the move; backward reverses through it. Never
 * teleports — same motion a robot would run for step ±.
 */
async function stepProgramPlayback(direction: PlaybackStepDirection): Promise<void> {
  if (jointMovePlanning || viewerMotion) return;
  if (usesDensePlanner()) {
    if (!railsSimulation.canStep(direction)) return;
    setRunPanel("simulate", "auto");
    railsSimulation.step(direction);
    return;
  }
  if (!hasExactProgramPlayback()) return;
  const trajectory = project.plan.trajectory;
  if (trajectory.length === 0) return;

  if (playing) pausePlaybackKeepCursor();

  const targetTime = stepPlaybackTime(trajectory, playbackCursorS, direction);
  if (targetTime === null) {
    updateStepControls(true);
    return;
  }
  if (Math.abs(targetTime - playbackCursorS) <= 1e-9) {
    updateStepControls(true);
    return;
  }

  const startSample = sampleAtTime(trajectory, playbackCursorS).sample;
  if (!startSample) return;
  const currentJoints = currentViewerJoints();
  if (!jointsMatch(currentJoints, startSample.jointPositionsRad)) {
    await planAndRunViewerJointMove(
      [...startSample.jointPositionsRad] as JointVector,
      "program_start",
      "MOVEJ TO PROGRAM",
      () => beginStepPlayback(direction, targetTime),
    );
    return;
  }
  beginStepPlayback(direction, targetTime);
}

function beginStepPlayback(
  direction: PlaybackStepDirection,
  targetTimeS: number,
): void {
  setRunPanel("simulate", "auto");
  playing = true;
  viewerMotion = null;
  stepPlayback = { direction, targetTimeS };
  playStartedAt = performance.now();
  playStartCursor = playbackCursorS;
  setPlaybackRowHighlight(rowIdAtProgramCursor(playbackCursorS));
  setMotionControls("program");
  const verb = direction === "forward" ? "forward" : "reverse";
  setViewerStatus(
    `Step ${verb} · executing one accepted-trajectory move · ${playbackCursorS.toFixed(1)} → ${targetTimeS.toFixed(1)} s`,
  );
  setStorageStatus(`Stepping ${verb} along the exact accepted Robot V4 trajectory · no physical robot output sent`);
  playbackFrame = requestAnimationFrame(tickPlayback);
}

function updateStepControls(exactPlayback: boolean): void {
  if (usesDensePlanner()) {
    const busy = jointMovePlanning || viewerMotion !== null || railsPlaying;
    query<HTMLButtonElement>("#step-back").disabled = busy || !railsSimulation.canStep("backward");
    query<HTMLButtonElement>("#step-forward").disabled = busy || !railsSimulation.canStep("forward");
    return;
  }
  const trajectory = project.plan.trajectory;
  const busy = jointMovePlanning || viewerMotion !== null || stepPlayback !== null;
  const enabled = exactPlayback && !busy && trajectory.length > 0;
  query<HTMLButtonElement>("#step-back").disabled =
    !enabled || !canStepPlayback(trajectory, playbackCursorS, "backward");
  query<HTMLButtonElement>("#step-forward").disabled =
    !enabled || !canStepPlayback(trajectory, playbackCursorS, "forward");
}

async function homeRobot(): Promise<void> {
  await planAndRunViewerJointMove(
    homeJointTarget(),
    "home",
    "HOME MOVEJ",
    () => {
      playbackCursorS = 0;
      updateTimeline(project.plan.trajectory.at(-1)?.timeS ?? 0);
      setViewerStatus(
        `Canonical ${project.robot.model} · HOME reached by timed Tesseract MOVEJ · J1–J9 = 0 rad`,
      );
      setStorageStatus("Home MOVEJ complete · accepted program unchanged · no physical robot output sent");
    },
  );
}

function previewTimelineSelection(event: Event): void {
  const input = event.currentTarget as HTMLInputElement;
  const duration = project.plan.trajectory.at(-1)?.timeS ?? 0;
  const requested = Number(input.value) / 1000 * duration;
  query("#playback-time").textContent =
    `MOVEJ ${requested.toFixed(1)} / ${duration.toFixed(1)} s`;
}

async function seekPlayback(event: Event): Promise<void> {
  const input = event.currentTarget as HTMLInputElement;
  const duration = project.plan.trajectory.at(-1)?.timeS ?? 0;
  const requestedCursorS = Number(input.value) / 1000 * duration;
  const target = sampleAtTime(project.plan.trajectory, requestedCursorS).sample;
  if (!target) {
    updateTimeline(duration);
    return;
  }
  await planAndRunViewerJointMove(
    [...target.jointPositionsRad] as JointVector,
    "timeline_seek",
    "TIMELINE MOVEJ",
    () => {
      playbackCursorS = requestedCursorS;
      updateTimeline(duration);
      setViewerStatus(
        `Timeline position reached by timed Tesseract MOVEJ · ${requestedCursorS.toFixed(1)} s`,
      );
    },
  );
}

async function planAndRunViewerJointMove(
  targetJointsRad: JointVector,
  purpose: ViewerJointMovePurpose,
  label: string,
  onComplete: () => void,
): Promise<void> {
  if (jointMovePlanning || activeExecution) return;
  stopPlayback();
  const startJointsRad = currentViewerJoints();
  if (jointsMatch(startJointsRad, targetJointsRad)) {
    onComplete();
    setMotionControls("idle");
    return;
  }

  jointMovePlanning = true;
  setMotionControls("planning", purpose);
  setViewerStatus(`${label} · requesting a canonical Motion Server/Tesseract joint plan…`);
  setStorageStatus(`${label} planning · current J1–J9 to requested target · no browser-generated trajectory`);
  try {
    localStatus = await planner.refresh();
    if (!localStatus.availability.available) {
      throw new Error(
        `canonical local planner unavailable: ${localStatus.availability.reason}`,
      );
    }
    if (!localSimulatorReady(localStatus)) {
      localStatus = await planner.start();
    }
    if (!localSimulatorReady(localStatus)) {
      throw new Error(
        `safe local planner readiness missing: ${localSafetyReasons(localStatus).join(",")}`,
      );
    }
    const motionProject = createViewerJointMoveProject(project, {
      purpose,
      startJointsRad,
      targetJointsRad,
      localProfileId: localStatus.profileId,
    });
    const result = await planner.planTransient(motionProject);
    localStatus = planner.status;
    if (!result.acceptedPlan) {
      throw new Error("canonical planner returned no accepted MOVEJ trajectory");
    }
    const trajectory = acceptedPlanTrajectory(
      motionProject,
      result.acceptedPlan,
    );
    validateViewerJointMoveTrajectory(
      trajectory,
      startJointsRad,
      targetJointsRad,
    );
    startViewerMotion({
      kind: purpose,
      label,
      project: motionProject,
      trajectory,
      onComplete,
    });
  } catch (error) {
    setViewerStatus(`${label} blocked · ${message(error)}`);
    setStorageStatus(`${label} blocked: ${message(error)} · robot held at its current position`);
    updateTimeline(project.plan.trajectory.at(-1)?.timeS ?? 0);
  } finally {
    jointMovePlanning = false;
    if (!viewerMotion) setMotionControls("idle");
  }
}

function startViewerMotion(input: {
  kind: ViewerJointMovePurpose;
  label: string;
  project: OfflineProgrammingProject;
  trajectory: PlanTrajectorySample[];
  onComplete: () => void;
}): void {
  viewerMotion = {
    ...input,
    startedAtMs: performance.now(),
  };
  setRunPanel("simulate", "auto");
  playing = true;
  viewer.setCurrentTcpPose(null);
  setMotionControls("joint_move", input.kind);
  playbackFrame = requestAnimationFrame(tickViewerMotion);
}

function tickViewerMotion(now: number): void {
  const motion = viewerMotion;
  if (!playing || !motion) return;
  const duration = motion.trajectory.at(-1)?.timeS ?? 0;
  const timeS = Math.min(duration, (now - motion.startedAtMs) / 1000);
  const resolved = sampleAtTime(motion.trajectory, timeS);
  applyProjectSample(viewer, motion.project, resolved.sample);
  const fraction = duration > 0 ? timeS / duration : 1;
  query<HTMLInputElement>("#timeline").value =
    String(Math.round(Math.max(0, Math.min(1, fraction)) * 1000));
  query("#playback-time").textContent =
    `${motion.label} ${timeS.toFixed(1)} / ${duration.toFixed(1)} s`;
  if (resolved.done) {
    const complete = motion.onComplete;
    viewerMotion = null;
    playing = false;
    cancelAnimationFrame(playbackFrame);
    setMotionControls("idle");
    complete();
    return;
  }
  playbackFrame = requestAnimationFrame(tickViewerMotion);
}

function currentViewerJoints(): JointVector {
  const joints = viewer.currentPoseSnapshot().jointsRad;
  if (joints.length !== 9 || joints.some((value) => !Number.isFinite(value))) {
    throw new Error("rendered robot pose is not a finite J1–J9 vector");
  }
  return [...joints] as JointVector;
}

function setMotionControls(
  state: "idle" | "planning" | "program" | "joint_move",
  purpose?: ViewerJointMovePurpose,
): void {
  const exactPlayback = hasExactProgramPlayback();
  const play = query<HTMLButtonElement>("#play");
  const stop = query<HTMLButtonElement>("#stop");
  const home = query<HTMLButtonElement>("#home-robot");
  const moving = state === "program" || state === "joint_move";
  play.textContent = moving ? "Ⅱ" : "▶";
  play.setAttribute(
    "aria-label",
    moving ? "Pause robot motion" : "Play preview",
  );
  play.disabled = !exactPlayback || state === "planning";
  stop.disabled = state === "idle" || state === "planning";
  home.disabled = state === "planning" || activeExecution !== null;
  home.textContent = purpose === "home" && state === "planning"
    ? "Planning…"
    : purpose === "home" && state === "joint_move"
      ? "Homing…"
      : "Home";
  updateStepControls(exactPlayback && state !== "planning");
}

function updateTimeline(duration: number): void {
  const fraction = duration > 0 ? playbackCursorS / duration : 0;
  query<HTMLInputElement>("#timeline").value = String(Math.round(Math.max(0, Math.min(1, fraction)) * 1000));
  query("#playback-time").textContent = `${playbackCursorS.toFixed(1)} / ${duration.toFixed(1)} s`;
  updateStepControls(hasExactProgramPlayback() && !jointMovePlanning);
}

async function saveNow(): Promise<void> {
  try {
    await autosave.flush();
    await repository.save(project);
    query("#save-state").textContent = repository.usingMemoryFallback ? "MEMORY" : "SAVED";
    setStorageStatus(`Saved project revision ${project.identity.revision}`);
  } catch (error) {
    query("#save-state").textContent = "ERROR";
    setStorageStatus(`Save blocked: ${message(error)}`);
  }
}

function openNewProgramDialog(): void {
  if (activeExecution) {
    setStorageStatus("Stop the active robot execution before creating another program.");
    return;
  }
  const dialog = query<HTMLDialogElement>("#new-program-dialog");
  const input = query<HTMLInputElement>("#new-program-name");
  input.value = "Untitled robot program";
  if (!dialog.open) dialog.showModal();
  window.requestAnimationFrame(() => input.select());
}

function closeNewProgramDialog(): void {
  const dialog = query<HTMLDialogElement>("#new-program-dialog");
  if (dialog.open) dialog.close();
}

async function createNewProgram(name: string): Promise<void> {
  try {
    await autosave.flush();
    await repository.save(project);
    // Every new program is a rails program: seams are detected, angles
    // searched. Classic edge drafting stays only for programs that have it.
    // A connected machine names the robot; otherwise the previous program's.
    const cellModel = densePanel.targetSelected() ? densePanel.cellModel() : null;
    project = pinRobotDescription(createBlankSpreadsheetProject({
      name,
      robotModel: cellModel && isRobotModel(cellModel) ? cellModel : project.robot.model,
      railsAuthoring: true,
    }));
    await repository.save(project);
    history.clear();
    targetState = initialTargetState(Date.now());
    applyDefaultPlannerSelection();
    localPlanResult = null;
    localPlannerError = null;
    serverExecutionReview = null;
    serverExecutionReviewWire = null;
    activeCADSource = null;
    resetWeldAuthoringUiState();
    playbackCursorS = 0;
    stopPlayback();
    syncRailsModeToProject();
    render();
    await syncViewerProject();
    query("#save-state").textContent =
      repository.usingMemoryFallback ? "MEMORY" : "SAVED";
    setStorageStatus("New program created · empty until you add rows or define welds");
    // Made while connected to a cell: it is the cell's program from the start.
    if (cellSync.sync.enabled && cellSync.sync.store) cellSync.upload(false);
  } catch (error) {
    query("#save-state").textContent = "ERROR";
    setStorageStatus(`New program blocked: ${message(error)}`);
  }
}

function download(bytes: BlobPart, filename: string, type: string): void {
  const url = URL.createObjectURL(new Blob([bytes], { type }));
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = filename;
  anchor.click();
  URL.revokeObjectURL(url);
}

function projectFileStem(): string {
  return project.identity.name.replace(/[^a-z0-9]+/gi, "-").toLowerCase() || "offline-program";
}

/**
 * Export what a planner is handed, not what the OLP holds.
 *
 * A rails program exports its `.weldplan`: program.v2, the placement inside
 * it, the cell descriptor, the tooling and the STEP, hashed together into
 * deterministic bytes. That container is the only artifact that reproduces a
 * plan somewhere else — it is what the motion server is posted, and it can be
 * replayed against it directly. The project JSON cannot: it is this
 * application's own document, and no planner reads it.
 *
 * Packed by the same code path Plan uses, so what is exported is what would
 * be planned rather than a second answer built for the occasion.
 *
 * A classic program has no `.weldplan` — it plans through the Motion Server
 * from rows — so it still exports its project JSON.
 */
async function exportProject(): Promise<void> {
  if (!usesDensePlanner()) {
    download(exportProjectJSON(project), `${projectFileStem()}.json`, "application/json");
    return;
  }
  // The export is a menu action, not a toolbar button (there is no `#export`
  // element any more); disabling the action keeps a second click from
  // packing the same request twice while the seam worker is still on the first.
  const action = projectActions.exportWeldplan;
  action.enabled = false;
  try {
    const packed = await railsMode.exportWeldPlan();
    download(
      // A copy into a plain ArrayBuffer: the fetched view may be backed by a
      // SharedArrayBuffer, which Blob does not take.
      packed.bytes.slice().buffer as ArrayBuffer,
      packed.filename || `${projectFileStem()}.weldplan`,
      "application/vnd.gradient.amr-weld-planner-v1.plan-request",
    );
    setStorageStatus(`Exported ${packed.filename} · sha256 ${packed.sha256.slice(0, 12)} (first 12)`);
  } catch (error) {
    setStorageStatus(`Export failed: ${message(error)}`);
  } finally {
    action.enabled = true;
  }
}

async function importProject(event: Event): Promise<void> {
  const input = event.currentTarget as HTMLInputElement;
  const file = input.files?.[0];
  input.value = "";
  if (!file) return;
  try {
    const imported = pinRobotDescription(migrateLegacyTablePlacement(importProjectJSON(await file.text())));
    history.push(project);
    project = imported;
    activeCADSource = await restoreCADSource(imported);
    syncRailsModeToProject();
    resetWeldAuthoringUiState();
    changed(`Imported project ${file.name}`);
  } catch (error) {
    setStorageStatus(`Import rejected: ${message(error)}`);
  }
}

function setCADStatus(value: string): void {
  setBusyText(query("#cad-status"), value);
  cadLogger.at(levelOfStatus(value), value);
}

function registerServiceWorker(): void {
  if (!("serviceWorker" in navigator)) return;
  // In dev, EVICT rather than merely skip. Skipping leaves a worker that a
  // built-mode session registered still in charge, and it serves `/src/*.ts`
  // through stale-while-revalidate -- so every reload runs the PREVIOUS
  // edit's module and the current one only lands on the reload after that.
  // `dev-stack.sh` picks its UI mode from whether Tailscale is up, so a
  // profile crosses this line without anyone choosing to, and the symptom is
  // a fix that does not arrive.
  if (import.meta.env.DEV) {
    void evictStaleServiceWorker();
    return;
  }
  void navigator.serviceWorker.register(`${import.meta.env.BASE_URL}service-worker.js`, {
    scope: import.meta.env.BASE_URL,
  });
}

/** Dev only: throw out a worker a built-mode session registered, then reload.
 *
 *  The reload is the point. By the time this runs, the modules already
 *  executing came from that worker's cache, so the tab is running the
 *  PREVIOUS edit -- unregistering alone would leave stale code in charge
 *  until someone happened to reload twice. Once per profile: after the
 *  reload there is no registration and this does nothing, and the session
 *  flag stops a loop if a profile somehow keeps handing one back. */
async function evictStaleServiceWorker(): Promise<void> {
  const RELOADED = "olp:dev-service-worker-evicted";
  try {
    const registrations = await navigator.serviceWorker.getRegistrations();
    if (registrations.length === 0) return;
    await Promise.all(registrations.map((registration) => registration.unregister()));
    if (self.caches) {
      const keys = await caches.keys();
      await Promise.all(keys.map((key) => caches.delete(key)));
    }
    if (sessionStorage.getItem(RELOADED)) return;
    sessionStorage.setItem(RELOADED, String(Date.now()));
    location.reload();
  } catch {
    // A profile that cannot enumerate workers has none to evict.
  }
}

function planLabel(state: OfflineProgrammingProject["plan"]["state"]): string {
  if (state === "server_validated") return "SERVER VALIDATED";
  if (state === "preview") return "PREVIEW";
  if (state === "stale") return "STALE PLAN";
  if (state === "rejected") return "REJECTED";
  return "NO PLAN";
}

function escapeHTML(value: string): string {
  return value.replace(/[&<>"']/g, (character) => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    '"': "&quot;",
    "'": "&#39;",
  })[character]!);
}

function message(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}

function query<T extends Element = HTMLElement>(selector: string): T {
  const element = document.querySelector<T>(selector);
  if (!element) throw new Error(`missing ${selector}`);
  return element;
}
