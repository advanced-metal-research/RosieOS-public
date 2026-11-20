import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

const source = await readFile(new URL("../src/main.ts", import.meta.url), "utf8");
const edits = await readFile(new URL("../src/program-v2/edits.ts", import.meta.url), "utf8");
const robotModel = await readFile(new URL("../src/domain/robot-model.ts", import.meta.url), "utf8");

test("compiled rows are read-only and torch inputs keep their explanatory title", () => {
  assert.match(source, /row\.phase === "weld"[\s\S]*disabled title="Torch angles are authored on weld rows;/);
  assert.match(source, /class="program-grid is-readonly"/);
  assert.match(source, /control\.disabled = true;/);
  assert.doesNotMatch(source, /function applySpreadsheetInput/);
});

test("planner dropdown defaults to Tesseract, not TARGET / Offline preview", () => {
  assert.match(source, /<span>Planner<\/span>/);
  assert.doesNotMatch(source, /<span>Target<\/span>/);
  assert.match(
    source,
    /value="offline_preview">None \(browser-only, no planner\)</,
  );
  assert.match(
    source,
    /value="local_simulator" selected>Tesseract/,
  );
  assert.match(source, /applyDefaultPlannerSelection\(\)/);
  assert.match(source, /function tesseractPlannerOptionLabel\(\)/);
  assert.match(source, /Tesseract · unavailable/);
  assert.doesNotMatch(source, />Offline preview</);
  assert.doesNotMatch(source, />Local simulator/);
});

test("the toolbar Jog action opens native controls in the bottom spreadsheet split", () => {
  assert.match(source, /id: "toggle-jog", label: "Jog", icon: Gamepad2, checkable: true/);
  assert.match(source, /id="bottom-workspace"/);
  assert.match(source, /id="jog-resizer"[^>]*role="separator"/);
  assert.match(source, /id="dense-joint-jog"/);
  assert.match(source, /id="spreadsheet-cartesian-grid"/);
  assert.match(source, /wireSpreadsheetCartesianJog/);
  assert.match(source, /densePanel\.cartesianJog\(\)\?\.begin/);
  assert.doesNotMatch(source, /Show jog panel/);
});

test("the row inspector is gone; nodes are edited in the parameter dialog", () => {
  assert.doesNotMatch(source, /id="tab-setup"/);
  assert.doesNotMatch(source, /id="tab-row"/);
  assert.doesNotMatch(source, /class="program-panel panel"/);
  assert.doesNotMatch(source, /id="row-tab-panel"/);
  assert.doesNotMatch(source, /id="program-resizer"/);
  assert.doesNotMatch(source, /function renderRowInspector/);
  assert.doesNotMatch(source, /function focusRowInspector/);
  assert.match(source, /id="setup-sections"/);
  assert.match(source, /id="node-editor"/);
  assert.match(source, /createNodeEditor\(query<HTMLDialogElement>\("#node-editor"\)/);
  assert.match(source, /renderNodeSpreadsheet\(host, \{/);
  assert.match(source, /id="edit-node"[\s\S]*id="toggle-advanced"/);
  // Execution review and start/stop moved into the spreadsheet panel.
  assert.match(source, /class="spreadsheet-toolbar">[\s\S]*id="execution-chrome"[\s\S]*id="program-spreadsheet"/);
});

test("left SETUP panel keeps a segmented nav and the original field ids", () => {
  // The left panel is the program tree over the selection inspector.
  assert.match(source, /id="program-tree-host"/);
  assert.match(source, /new TreeView\("Program"\)/);
  assert.match(source, /new Inspector\(\)/);
  assert.match(source, /data-setup-section="project"/);
  assert.match(source, /data-setup-section="robot"/);
  assert.match(source, /data-setup-section="planner"/);
  assert.match(source, /data-setup-section="weld"/);
  assert.match(source, /id="project-name"/);
  assert.match(source, /id="import-step"/);
  assert.match(source, /id="step-example"[\s\S]*id="open-example"/);
  assert.match(source, /function importStepBytes\(/);
  // Every new program is a rails program; there is no mode switch anywhere.
  assert.doesNotMatch(source, /data-new-program-mode/);
  assert.doesNotMatch(source, /data-setup-item="project.mode"/);
  assert.match(source, /railsAuthoring: true,/);
  assert.doesNotMatch(source, /id="toggle-rails-mode"/);
  assert.match(source, /examples\/manifest\.json/);
  assert.match(source, /id="robot-model"/);
  assert.match(source, /id="target-mode"/);
  // Placing the part is the Move part tool; the inspector only launches it.
  assert.match(source, /function launchPartMove\(\)/);
  assert.doesNotMatch(source, /data-workpiece-field=/);
  assert.match(source, /onCentre: centreCurrentWorkpiece/);
  assert.match(source, /data-process-field="process"/);
  assert.match(source, /data-process-field="torchSpinAngleDeg"/);
});

test("topbar session pills fail closed to simulation and disarmed", () => {
  assert.match(source, /id="session-mode-badge"/);
  assert.match(source, /id="drive-arm-badge"/);
  assert.match(
    source,
    /id="session-mode-badge"[\s\S]*?>SIMULATION<\/span>/,
  );
  assert.match(
    source,
    /id="drive-arm-badge"[\s\S]*?>DISARMED<\/button>/,
  );
  assert.match(source, /renderSessionStatusPills\(\)/);
});

test("jog module state is declared before the top-level boot calls (TDZ guard)", () => {
  // Regression: `let jogLive` after the top-level jog boot threw a
  // temporal-dead-zone ReferenceError at module evaluation, which also killed
  // `startLiveJog()` and every listener wired after it — jog never started.
  const declaration = source.indexOf("let jogLive = false;");
  const armDeclaration = source.indexOf("let simArmed = false;");
  const popoverRelease = source.indexOf("let armPopoverRelease:");
  const firstTopLevel = source.indexOf('const app = query<HTMLDivElement>("#app");');
  const boot = source.indexOf("void startLiveJog();");
  assert.ok(declaration > 0, "jogLive declaration exists");
  assert.ok(armDeclaration > 0, "simArmed declaration exists");
  assert.ok(popoverRelease > 0, "armPopoverRelease declaration exists");
  assert.ok(firstTopLevel > 0, "app shell statement exists");
  assert.ok(boot > 0, "boot call exists");
  // render() runs during boot and reads simArmed, so the declarations must
  // precede every top-level executable statement, not just the jog boot calls.
  assert.ok(declaration < firstTopLevel, "jogLive is declared before the app shell renders");
  assert.ok(armDeclaration < firstTopLevel, "simArmed is declared before the app shell renders");
  assert.ok(popoverRelease < firstTopLevel, "armPopoverRelease is declared before the app shell renders");
  assert.ok(declaration < boot, "jogLive is declared before startLiveJog()");
});

test("the maintenance overlay is up before boot first waits on the server", () => {
  const mount = source.indexOf("const maintenanceOverlay = new MaintenanceOverlay(");
  const watch = source.indexOf("new MaintenanceWatch(");
  const firstAwait = source.search(/^(?:[^\s/].*)?\bawait\b/m);
  assert.ok(mount > 0 && watch > mount, "overlay and watch exist");
  assert.ok(firstAwait > 0 && watch < firstAwait, "the notice does not wait for boot");
  assert.ok(watch < source.indexOf("await fetchRobotCatalogue()"), "the notice does not wait for the robot catalogue");
  assert.match(source, /maintenanceOverlay\.bindStop\(\(\) => densePanel\.stop\(\)/);
});

test("native Arm and Stop remain reachable while preview arming stays in Simulate", () => {
  assert.match(source, /id="cell-stop-pill"/);
  assert.match(source, /id="drive-arm-badge"/);
  assert.match(source, /densePanel\.disarm\(\)/);
  // The pill follows the drives' word: a held control session with the drives
  // off (the state after Home) must arm, not release control.
  assert.match(source, /arm\?\.state === "armed" \|\| arm\?\.state === "arming"\) densePanel\.disarm\(\)/);
  assert.doesNotMatch(source, /arm\?\.controlled \|\| arm\?\.state === "armed"/);
  assert.match(source, /densePanel\.arm\(\)/);
  assert.match(source, /id="cell-arm-popover"/);
  // The browser preview's jog strip: one line for a refusal, never the per-axis fence dump.
  assert.match(source, /previewFeedbackLine\(state\?\.operatorFeedback \?\? null\)/);
  assert.doesNotMatch(source, /renderOperatorFeedback\(/);
  assert.match(source, /query\("#cell-arm-cancel"\)\.addEventListener\("click", \(\) => densePanel\.cancelArm\(\)\)/);
  assert.match(source, /popover\.hidden = !densePanel\.armPopoverOpen\(\)/);
  assert.match(source, /armDismiss\(popover, pill, \(\) => densePanel\.cancelArm\(\)\)/);
  assert.match(source, /query\("#sim-arm-toggle"\)\.addEventListener\("click", toggleSimArm\)/);
  assert.match(source, /densePanel\.targetSelected\(\) \|\| project\.program\.executionMode === "production"/);
  assert.doesNotMatch(source, /armControlPill\(/);
});

test("jog state feedback drives the 3D robot pose while the session is live", () => {
  assert.match(source, /function pollJogState\(\)/);
  // The 100ms state poll maps jointsRad by axis name and applies it to the
  // viewer -- through the edge-triggered gate, so an idle jog session does not
  // overwrite a preview's pose on every poll (the flashing rails replay).
  assert.match(
    source,
    /pollJogState[\s\S]*?const jointsRad = jointOrder\.map\(\(name\) => byName\.get\(name\) \?\? 0\);[\s\S]*?if \(!jogFeed\.admit\(jointsRad, playing \|\| railsPlaying\)\) return;\s*viewer\.setPose\(\{\s*modelId: project\.robot\.model,\s*jointOrder,\s*jointsRad,\s*source: "live",/,
  );
  assert.match(source, /jogStatePoll = window\.setInterval\(\(\) => void pollJogState\(\), 100\)/);
});

test("every node write goes through mutate, so it is locked during an execution", () => {
  for (const name of ["commitNode", "upsertWeldNode", "deleteNode", "moveNodeInProgram", "toggleNodeEnabled"]) {
    const start = source.indexOf(`function ${name}(`);
    assert.ok(start > 0, `${name} exists`);
    const body = source.slice(start, source.indexOf("\n}\n", start));
    assert.match(body, /mutate\("nodes"/, `${name} writes through mutate("nodes")`);
  }
  // The edits and the row compile are the shared module the Deck pendant runs too.
  assert.match(source, /edit\(draft\);\n\s*deriveRows\(draft, change, selectedNodeId\);/);
  assert.match(edits, /export function deriveRows\(/);
  assert.match(edits, /draft\.rows = rowsFromNodes\(draft\);/);
});

test("the robot transport lives in Deploy, preview controls in the spreadsheet toolbar", () => {
  assert.match(source, /id="dense-playback"/);
  assert.match(source, /id="dense-robot-host"/);
  assert.match(source, /densePanel\.mount\(query\("#dense-robot-host"\), query\("#dense-playback"\), query<HTMLElement>\("#jog-panel"\)\)/);
  assert.match(source, /simulate\.append\(\s*fact\("Plan", query<HTMLElement>\("\.spreadsheet-ops-group\.spreadsheet-plan"\)\)/);
  assert.match(source, /const playback = query<HTMLElement>\("\.spreadsheet-ops-group\.playback"\)/);
  assert.match(source, /query<HTMLElement>\("\.spreadsheet-toolbar"\)\.append\(playback\)/);
  assert.match(source, /deploy\.append\(\s*fact\("Target", target\),[\s\S]*?fact\("Run", query<HTMLElement>\("#dense-playback"\)\)/);
  // The execution mode select is not the Deploy panel's; the target is the cell from Robot.
  assert.match(source, /query<HTMLElement>\("#setup-sections"\)\.append\(query<HTMLElement>\("\.spreadsheet-ops-group\.spreadsheet-mode"\)\)/);
  assert.match(source, /targetName\.id = "deploy-target-name"/);
  assert.doesNotMatch(source, /\.playback"\)\.hidden = densePanel\.targetSelected\(\)/);
  // Every editing gate goes through the one lock that knows both sessions.
  assert.doesNotMatch(source, /if \(activeExecution\) \{\n\s*query\("#storage-status"\)\.textContent = "Editing is locked/);
  assert.match(source, /function editingLockReason\(\)/);
});

test("the viewer's pose source follows the dock panel, which follows whichever robot moves", () => {
  // The real robot's feedback owns the pose when it is shown; otherwise a
  // playing preview does, classic or rails, and the jog feed yields to it.
  assert.match(source, /pollJogState[\s\S]*?if \(poseSource === "real"\) return;/);
  assert.match(source, /jogFeed\.admit\(jointsRad, playing \|\| railsPlaying\)/);
  assert.match(source, /railsPlaying = nowPlaying;/);
  // With no cell selected the Deploy panel still shows the preview.
  assert.match(source, /if \(next === "real" && !densePanel\.targetSelected\(\)\) next = "sim";/);
  // The switch is locked away from the moving robot.
  assert.match(source, /button\.disabled = moving && !active;/);
});

test("the Jog panel is a Joints | Cartesian switch beside the telemetry strip fed by every machine status", () => {
  assert.match(source, /class="run-switch jog-mode-switch" role="tablist"/);
  // The mode tabs carry their own class: the run-switch repaint clears `is-active` on every `.run-switch-tab` it finds.
  assert.match(source, /class="jog-mode-tab is-active" role="tab" data-jog-mode="joints"/);
  assert.match(source, /class="jog-mode-tab" role="tab" data-jog-mode="cartesian"/);
  assert.doesNotMatch(source, /class="run-switch-tab[^"]*" role="tab" data-jog-mode/);
  assert.match(source, /<section id="telemetry-strip" class="telemetry-strip" aria-label="Telemetry"><\/section>/);
  assert.match(source, /onTelemetry: \(status, capabilities\) => \{\s*const sample = sampleFromStatus\(status, capabilities\);\s*if \(sample\) telemetryStrip\(\)\.push\(sample\);/);
  // Every status line marks busy from its own ellipsis; the cell pill and the import button too.
  assert.match(source, /function setStorageStatus\(text: string\): void \{\s*setBusyText\(query\("#storage-status"\), text\);/);
  assert.match(source, /function setViewerStatus\(value: string\): void \{\s*setBusyText\(query\("#viewer-status"\), value\);/);
  assert.match(source, /function setCADStatus\(value: string\): void \{\s*setBusyText\(query\("#cad-status"\), value\);/);
  assert.match(source, /setBusyText\(host\.querySelector<HTMLElement>\("\.menubar-status-text"\)!, summary\);/);
  // The Cartesian view shares the zero-all sequence and no longer repeats the machine notes above its controls.
  assert.match(source, /<button id="cartesian-zero" class="mini-button cartesian-zero" type="button"/);
  assert.match(source, /<span id="jog-cartesian-caption" class="jog-caption" hidden>/);
  assert.match(source, /<p id="cartesian-jog-target" class="jog-section-note" hidden>/);
  // The Cartesian pad is four clusters: a cross for X and Y, a column for Z, a cross for RX and RY, a column for RZ.
  assert.equal((source.match(/class="jog-pad jog-pad-cross"/g) ?? []).length, 2);
  assert.equal((source.match(/class="jog-pad jog-pad-column"/g) ?? []).length, 2);
  assert.equal((source.match(/class="mini-button jog-hold jog-key"/g) ?? []).length, 12);
  // X forward is the top key of the first cross; Y+ is its left key, where the base frame puts it.
  assert.match(source, /style="grid-area: up" data-jog-axes="1,0,0,0,0,0"/);
  assert.match(source, /style="grid-area: left" data-jog-axes="0,1,0,0,0,0"/);
  // The browser boots into the program it had open, not the most recently saved one.
  assert.match(source, /const CURRENT_PROJECT_KEY = "olp\.project\.current";/);
  assert.match(source, /const chosen = remembered && summaries\.some\(summary => summary\.id === remembered\) \? remembered : summaries\[0\]\?\.id \?\? null;/);
  assert.match(source, /function render\(\): void \{[^}]*rememberCurrentProject\(project\.identity\.id\);/);
  // The ring capture reads through the dense client with the panel's capabilities for its unit conversion.
  assert.match(source, /const window = await denseTelemetryClient\.telemetryWindow\(seconds, stride, signal\);\s*const samples = samplesFromWindow\(window, densePanel\.capabilities\(\)\);/);
  // Leaving Cartesian ends a hold before the pad disappears.
  assert.match(source, /if \(jogMode === "cartesian" && mode !== "cartesian"\) \{ stopJogHold\("mode_switch"\); densePanel\.endJogHolds\("mode_switch"\); \}/);
});

test("the connected machine's model becomes the program's robot, so the viewer can follow its live pose", () => {
  // Lost once in the main merge (676431bb): a 1400 program on the 1420 bench showed LIVE while every pose was rejected.
  assert.match(source, /onCellModels: \(models, selected\) => \{/);
  assert.match(source, /renderCellModel\(select, note, \[/);
  assert.match(source, /if \(project\.robot\.model !== selected\) \{\s*const from = project\.robot\.model;\s*select\.value = selected;\s*select\.dispatchEvent\(new Event\("change"\)\);/);
  assert.match(source, /robotModel: cellModel && isRobotModel\(cellModel\) \? cellModel : project\.robot\.model/);
  assert.match(source, /the viewer shows the program's \$\{project\.robot\.model\} at home; the machine is a \$\{lastRealPose\.modelId\}/);
});

test("the deployment target is chosen from Cells ▾ and its dialog, and the cell's settings are pulled into the Workspace", () => {
  assert.match(source, /new MenuButton\(\{ label: "Cells", caret: false[^}]*\}, robotMenu\)\.mount\(menubar\)/);
  assert.match(source, /id="app-menubar"/);
  assert.match(source, /robotMenu\.addHeading\("Connect to"\)/);
  assert.match(source, /label: "Pull cell settings"/);
  assert.match(source, /label: "Manage cells…"/);
  assert.match(source, /manageCells\.triggered\.connect\(\(\) => setRobotOpen\(true\)\)/);
  // Search local network: discovered cells open the dialog and only fill the Add form.
  assert.match(source, /robotMenu\.addHeading\("On the local network"\)/);
  assert.match(source, /label: "Search local network…"/);
  assert.match(source, /function openDiscoveredCell[\s\S]*?robotDialog\.openNetworkSection\(\);\s*if \(row\) robotDialog\.prefillFromDiscovery\(row\);/);
  assert.doesNotMatch(source, /prefillFromDiscovery[^\n]*densePanel\.addCell/);
  assert.match(source, /new RobotDialog\(\{[\s\S]*?\}\)\.mount\(document\.body\)/);
  assert.match(source, /onSelect: \(id\) => densePanel\.selectTarget\(id\)/);
  assert.match(source, /onAdd: async \(input\)[\s\S]*densePanel\.addCell\(input\)/);
  assert.match(source, /draft\.robot\.cell = settings;/);
  // A modal, not a tool card: it does not take part in the one-card-at-a-time rule.
  assert.doesNotMatch(source, /robotCard/);
  assert.match(source, /function setRobotOpen[\s\S]*?robotDialog\.open\(\)/);
  // The select stays for the panel that reads it, hidden from the pane.
  assert.match(source, /'\[data-setup-item="robot\.target"\]'\)\.hidden = true/);
  // The header names the connected cell; the Workspace says where its settings came from.
  assert.match(source, /status\.id = "robot-status";/);
  assert.match(source, /densePanel\.connectionSummary\(\)/);
  assert.match(source, /id="cell-settings"/);
  assert.match(source, /modelSelect\.disabled = pinnedBy !== null/);
});

test("the dock is two panels, Simulate and Deploy, and the panel that is up decides the viewer's pose", () => {
  assert.match(source, /element\("div", "run-switch"\)/);
  assert.match(source, /\[\["simulate", "Simulate"\], \["deploy", "Deploy"\]\]/);
  assert.match(source, /setPoseSource\(panel === "deploy" \? "real" : "sim", why\)/);
  // Playback brings Simulate up; motion on the robot brings Deploy up.
  assert.match(source, /onRobotMotion: \(\) => setRunPanel\("deploy", "auto"\)/);
  assert.match(source, /function beginProgramPlayback\(\): void \{\s*setRunPanel\("simulate", "auto"\)/);
  // No Pose group on the toolbar any more.
  assert.doesNotMatch(source, /addGroup\("Pose"\)/);
  assert.doesNotMatch(source, /poseSim|poseReal/);
});

test("the plan history sits under both dock panels, with select and forget", () => {
  assert.match(source, /element\("section", "dock-section plan-history"\)/);
  assert.match(source, /run\.append\(runSwitch, simulate, deploy\);[\s\S]*?pane\.append\(plans, run\)/);
  // Every plan install records itself.
  assert.equal((source.match(/project = recordPlan\((?:project|plannedProject)\);/g) ?? []).length, 3);
  assert.match(source, /function selectPlanEntry[\s\S]*?activatePlan\(project, entryId\)/);
  assert.match(source, /function deletePlanEntry[\s\S]*?deletePlan\(project, entryId\)/);
  // A plan on the robot cannot be forgotten from under it.
  assert.match(source, /remove\.disabled = deployed \|\| lock !== null;/);
});

test("the part is selectable in the viewport, and its menu is Move part / Rails welds / Waypoint welds / Delete part", () => {
  assert.match(source, /onPartClick: \(\) => \{\s*if \(selection\.kind !== "part"\) setSelection\(\{ kind: "part" \}\);/);
  assert.match(source, /label: "Move part", icon: Move3d, shortcut: "M"/);
  assert.match(source, /label: "Rails welds", icon: ListChecks, shortcut: "W"/);
  assert.match(source, /label: "Delete part", icon: Trash2, shortcut: "Del"/);
  assert.match(source, /label: "Waypoint welds", icon: Spline \}/);
  assert.match(source, /return \[move, rails, waypoints, remove\];/);
  // Delete takes everything the part brought, in one revision.
  assert.match(source, /function deletePart[\s\S]*?mutate\("cad", \(draft\) => \{[\s\S]*?draft\.edgeCatalog = \[\];[\s\S]*?draft\.nodes = \[\];[\s\S]*?draft\.fixtures = \[\];/);
  // The tree's Delete key and the viewport's right-click reach the same menu.
  assert.match(source, /if \(selection\.kind === "part"\) \{\s*deletePart\(\);/);
  assert.match(source, /pane\.addEventListener\("contextmenu"[\s\S]*?edgeSelection\.pickPartPoint\(event\.clientX, event\.clientY\)/);
});

test("the program's name in the title bar renames in place", () => {
  assert.match(source, /<button id="program-title" class="program-title" type="button"/);
  // The program's menu is the ≡ before its name; no Project button in the menu bar.
  assert.match(source, /icon: MenuIcon, showLabel: false, caret: false[^}]*\}, projectMenu\)\s*\.mount\(query<HTMLElement>\("#program-menu-host"\)\)/);
  assert.doesNotMatch(source, /label: "Project", caret: false/);
  assert.match(source, /function beginRenameProgram[\s\S]*?input\.className = "program-title-input"/);
  assert.match(source, /if \(event\.key === "Enter"\) \{\s*event\.preventDefault\(\);\s*finish\(true\);/);
  assert.match(source, /if \(event\.key === "Escape"\) \{\s*event\.preventDefault\(\);\s*finish\(false\);/);
  // One rename path for the title, the inspector field and the context menu.
  assert.match(source, /bindProgramNameInput\(query<HTMLInputElement>\("#project-name"\), renameProgram\)/);
  assert.match(source, /rename\.triggered\.connect\(\(\) => \{\s*setSelection\(\{ kind: "program" \}\);\s*beginRenameProgram\(\);/);
});

test("a program pins the description it was authored against, and the cell's identity is read through rt-core", () => {
  // The catalogue is read once at boot; the pin is taken when a program is
  // created, opened, imported or moved to another robot.
  assert.match(source, /setRobotCatalogue\(await fetchRobotCatalogue\(\)\)/);
  assert.match(source, /pinRobotDescription\(migrateLegacyTablePlacement\(stored\)\)/);
  assert.match(source, /pinRobotDescription\(createBlankSpreadsheetProject\(\{ name: "New weld program" \}\)\)/);
  assert.match(source, /pinRobotDescription\(migrateLegacyTablePlacement\(importProjectJSON\(await file\.text\(\)\)\)\)/);
  // The model change is the shared module the Deck pendant runs too.
  assert.match(robotModel, /function applyRobotModel[\s\S]*?draft\.robot\.model = model;\s*setRobotDescriptionPin\(draft\);/);
  assert.match(source, /import \{ applyRobotModel, cellSettingsPull, isRobotModel \} from "\.\/domain\/robot-model\.ts";/);
  // Drift is against the connected machine's description while one is
  // connected, and accepting is an explicit action after reviewing changes.
  assert.match(source, /onStatus: \(\) => syncConnectedRobot\(\)/);
  assert.match(source, /function syncConnectedRobot[\s\S]*?setConnectedRobot\(robot\)/);
  assert.match(source, /function renderRobotDescriptionPin[\s\S]*?accept-robot-settings/);
  assert.doesNotMatch(source, /adopt-hash|robot-confirmation-code|navigator\.clipboard\.writeText\(wanted\)/);
  assert.match(source, /mutate\("metadata", \(draft\) => adoptRobotDescription\(draft\)\)/);
  assert.match(source, /renderRobotDescriptionPin\(host\);\s*if \(!settings\)/);
  // Pull fetches the machine's machine_planning_calibration.json bytes with its identity; with no
  // machine, a file stands in, and it is off while one is connected.
  assert.match(source, /settings = await densePanel\.fetchCellSettings\(\) \?\? settings;/);
  assert.match(source, /label: "Load machine_planning_calibration\.json…"/);
  assert.match(source, /loadCellJson\.setEnabled\(!densePanel\.targetSelected\(\) && lock === null/);
  assert.match(source, /if \(record\?\.schema !== "rosie\.machine-planning-calibration\.v1"\)/);
  assert.match(source, /machinePlanningCalibrationSource: "upload",/);
});
