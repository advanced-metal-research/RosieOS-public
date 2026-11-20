import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { createContext, runInContext } from "node:vm";
import { Window } from "happy-dom";
import ts from "typescript";
import { cellSettingsKey, type CellSettings } from "../src/domain/project.ts";
import { element } from "../src/ui/widget.ts";

const source = await readFile(new URL("../src/main.ts", import.meta.url), "utf8");
const ast = ts.createSourceFile("main.ts", source, ts.ScriptTarget.ES2022, true);
const names = ["renderCellSettings", "renderRobotDescriptionPin", "describeCellSettings", "shortHash", "mappedAxesLabel"];
const handlers = ast.statements.filter(node => ts.isFunctionDeclaration(node) && names.includes(node.name?.text ?? ""));
assert.equal(handlers.length, names.length);
const script = ts.transpileModule(handlers.map(node => node.getText(ast)).join("\n"), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.None },
}).outputText;

function harness() {
  const browser = new Window();
  const original = Object.getOwnPropertyDescriptor(globalThis, "document");
  Object.defineProperty(globalThis, "document", { configurable: true, value: browser.document });
  const host = document.createElement("div");
  host.id = "cell-settings";
  document.body.append(host);
  const stored: CellSettings = {
    cell: "simulation", jetsonId: "sim", robotModelId: "rosie_1400_v3", runtimeMode: "sim",
    mappedAxisMask: 511, rtcoreHost: "localhost", pulledAt: "2026-09-22T00:00:00Z",
    robotDescriptionSha256: "sha256:123456789abc", machinePlanningCalibrationSha256: "sha256:calibration",
  };
  const state = {
    project: { identity: { id: "program-one" }, robot: { model: "rosie_1400_v3", description: {
      robotDescriptionSha256: "sha256:123456789abc",
    }, cell: stored } },
    connected: "simulation", live: { ...stored }, currentHash: "sha256:abcdef123456", robotCatalogueError: "",
    element, cellSettingsKey,
    robotSettingsReview: () => ({
      title: "Joint speed settings changed", date: "2026-09-22", provenance: "Verified robot revision",
      rows: [{ label: "J7 gear ratio", before: "10:1", after: "100:1" }],
      unchanged: ["Robot geometry"], explanation: null,
      technical: { modelId: "rosie_1400_v3", from: "sha256:old", to: "sha256:new" },
    }),
    query: (selector: string) => document.querySelector(selector),
    robotDescriptionDrift: () => ({ from: state.project.robot.description.robotDescriptionSha256, to: state.currentHash }),
    densePanel: { connectedCell: () => state.connected, cellSettings: () => state.live },
    summaryRow: (label: string, value: string) => element("div", "", `${label}: ${value}`),
    plannedAtLabel: (value: string) => value,
    accepted: 0,
    adoptCurrentRobotDescription: () => { state.accepted++; },
  };
  const context = createContext(state);
  runInContext(script, context);
  return {
    host, browser, state,
    render: () => runInContext("renderCellSettings()", context),
    close() {
      if (original) Object.defineProperty(globalThis, "document", original);
      else Reflect.deleteProperty(globalThis, "document");
      void browser.happyDOM.close();
    },
  };
}

test("status refresh preserves expanded robot versions and focused explicit Accept button", () => {
  const h = harness();
  try {
    h.render();
    const details = h.host.querySelector<HTMLDetailsElement>(".robot-settings-details")!;
    const accept = h.host.querySelector<HTMLButtonElement>(".accept-robot-settings")!;
    details.open = true;
    accept.focus();
    for (let poll = 0; poll < 15; poll++) {
      h.state.live = { ...h.state.live, pulledAt: `poll-${poll}`, cellFetchedAt: `poll-${poll}` };
      h.render();
    }
    assert.ok(h.host.querySelector(".robot-settings-details") === details, "status refresh replaced the expanded versions");
    assert.equal(details.open, true);
    assert.ok(h.host.querySelector(".accept-robot-settings") === accept, "status refresh replaced the Accept button");
    assert.equal(document.activeElement, accept);
    assert.equal(accept.disabled, false);
    assert.equal(h.state.accepted, 0, "rendering and refreshing never accepts settings");
    assert.equal(h.host.querySelector("input, .robot-confirmation-code-row"), null);
    assert.doesNotMatch(h.host.textContent!, /Copy code|Enter confirmation code/);
    const changes = h.host.querySelector(".robot-settings-changes")!;
    assert.match(changes.textContent!, /J7 gear ratio/);
    assert.match(changes.textContent!, /10:1/);
    assert.match(changes.textContent!, /100:1/);
    assert.doesNotMatch(changes.textContent!, /sha256/);
    accept.click();
    assert.equal(h.state.accepted, 1, "one explicit click accepts settings");
    accept.click();
    assert.equal(h.state.accepted, 1, "repeat activation cannot accept twice");
  } finally { h.close(); }
});

for (const change of ["robot version", "program", "connection", "live settings", "stored settings", "catalogue error"] as const) {
  test(`${change} change refreshes Workspace and replaces the prior review`, () => {
    const h = harness();
    try {
      h.render();
      const details = h.host.querySelector<HTMLDetailsElement>(".robot-settings-details")!;
      const accept = h.host.querySelector<HTMLButtonElement>(".accept-robot-settings")!;
      details.open = true;
      assert.equal(accept.disabled, false);
      switch (change) {
        case "robot version": h.state.currentHash = "sha256:fedcba654321"; break;
        case "program": h.state.project.identity.id = "program-two"; break;
        case "connection": h.state.connected = "other-cell"; break;
        case "live settings": h.state.live.robotDescriptionSha256 = "sha256:live-update"; break;
        case "stored settings": h.state.project.robot.cell.pulledAt = "2026-09-23T00:00:00Z"; break;
        case "catalogue error": h.state.robotCatalogueError = "Description service unavailable"; break;
      }
      h.render();
      assert.ok(h.host.querySelector(".robot-settings-details") !== details, "changed settings kept the obsolete disclosure");
      assert.equal(accept.isConnected, false);
      assert.equal(h.state.accepted, 0);
      if (change === "catalogue error") {
        assert.match(h.host.textContent!, /Description service unavailable/);
      } else {
        assert.equal(h.host.querySelector<HTMLButtonElement>(".accept-robot-settings")!.disabled, false);
        assert.equal(h.host.querySelector<HTMLDetailsElement>(".robot-settings-details")!.open, false);
      }
      if (change === "connection") assert.match(h.host.textContent!, /Connected to other-cell/);
      if (change === "live settings") assert.match(h.host.textContent!, /simulation now reports/);
    } finally { h.close(); }
  });
}

for (const change of ["robot version", "authored version", "program", "model", "connection"] as const) {
  test(`an old Accept callback cannot accept after the ${change} changes`, () => {
    const h = harness();
    try {
      h.render();
      const oldAccept = h.host.querySelector<HTMLButtonElement>(".accept-robot-settings")!;
      switch (change) {
        case "robot version": h.state.currentHash = "sha256:fedcba654321"; break;
        case "authored version": h.state.project.robot.description.robotDescriptionSha256 = "sha256:other"; break;
        case "program": h.state.project.identity.id = "program-two"; break;
        case "model": h.state.project.robot.model = "rosie_1420_v1"; break;
        case "connection": h.state.connected = "other-cell"; break;
      }
      oldAccept.click();
      assert.equal(h.state.accepted, 0, "the callback must recheck identity before accepting");
      h.render();
      assert.equal(oldAccept.isConnected, false);
      oldAccept.click();
      assert.equal(h.state.accepted, 0, "detached obsolete buttons cannot accept a different review");
      h.host.querySelector<HTMLButtonElement>(".accept-robot-settings")!.click();
      assert.equal(h.state.accepted, 1, "the newly reviewed version can be explicitly accepted");
    } finally { h.close(); }
  });
}
