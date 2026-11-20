import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { createContext, runInContext } from "node:vm";
import { Window } from "happy-dom";
import ts from "typescript";
import { ProgressDialog } from "../src/ui/progress-dialog.ts";
import { planningFailure } from "../src/ui/planning-failure.ts";
import { element } from "../src/ui/widget.ts";

const source = await readFile(new URL("../src/main.ts", import.meta.url), "utf8");
const ast = ts.createSourceFile("main.ts", source, ts.ScriptTarget.ES2022, true);
const names = ["showPlanningFailure", "renderRobotDescriptionPin"];
const handlers = ast.statements.filter(node => ts.isFunctionDeclaration(node) && names.includes(node.name?.text ?? ""));
assert.equal(handlers.length, names.length);
const script = ts.transpileModule(handlers.map(node => node.getText(ast)).join("\n"), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.None },
}).outputText;

const driftError = "seam.build_plan_request: the program is written for 'rosie_1400_v3' at description sha256:old... and this request packs sha256:new...; the description moved under the program -- adopt it in the authoring tool (typed) and re-plan";

function harness() {
  const browser = new Window();
  const originals = ["window", "document"].map(key => [key, Object.getOwnPropertyDescriptor(globalThis, key)] as const);
  Object.assign(globalThis, { window: browser, document: browser.document });
  const host = document.createElement("div");
  host.id = "cell-settings";
  document.body.append(host);
  const calls: string[] = [];
  host.scrollIntoView = options => {
    assert.equal((options as ScrollIntoViewOptions).block, "start");
    calls.push("scroll");
  };
  const project = { identity: { id: "test-program" }, robot: { model: "rosie_1400_v3", description: { robotDescriptionSha256: "sha256:old" } } };
  const before = JSON.stringify(project);
  const dialog = new ProgressDialog().mount(document.body);
  const statuses: string[] = [];
  const state = {
    planningFailure, planProgress: dialog, project, robotCatalogueError: "", element,
    densePanel: { connectedCell: () => "simulation" },
    robotSettingsReview: () => ({
      title: "Robot settings changed", date: null, provenance: "Historical values unavailable",
      rows: [], unchanged: [], explanation: "The previous robot settings are unavailable for comparison.",
      technical: { modelId: "rosie_1400_v3", from: "sha256:old", to: "sha256:new" },
    }),
    shortHash: (hash: string) => hash.replace("sha256:", ""),
    robotDescriptionDrift: () => ({ from: "sha256:old", to: "sha256:new" }),
    summaryRow: (label: string, text: string) => element("div", "", `${label}: ${text}`),
    setStorageStatus: (text: string) => statuses.push(text),
    setSelection: (selection: { kind: string }) => {
      assert.equal(selection.kind, "workspace");
      assert.equal(dialog.shown, false);
      calls.push("workspace");
    },
    renderCellSettings: () => {
      calls.push("render");
      host.replaceChildren();
      runInContext("renderRobotDescriptionPin(host)", context);
    },
    query: (selector: string) => document.querySelector(selector),
    adoptCurrentRobotDescription: () => calls.push("adopt"),
    plan: () => { throw new Error("review must not plan automatically"); },
    mutate: () => { throw new Error("review must not change settings"); },
    host,
  };
  const context = createContext(state);
  runInContext(script, context);
  return {
    browser, host, calls, statuses, dialog, project, before,
    show(error: string) {
      Object.assign(state, { error });
      dialog.start("Planning");
      runInContext("showPlanningFailure(error)", context);
    },
    close() {
      dialog.dispose();
      for (const [key, descriptor] of originals) {
        if (descriptor) Object.defineProperty(globalThis, key, descriptor);
        else Reflect.deleteProperty(globalThis, key);
      }
      void browser.happyDOM.close();
    },
  };
}

test("planning warning action opens Workspace and focuses explicit acceptance without adopting or planning", () => {
  const h = harness();
  try {
    h.show(driftError);
    assert.deepEqual(h.calls, []);
    assert.match(h.statuses.at(-1)!, /Robot settings have changed/);
    assert.doesNotMatch(h.statuses.at(-1)!, /sha256/);
    const action = h.dialog.el.querySelector<HTMLButtonElement>(".progress-dialog-action")!;
    assert.equal(action.textContent, "Review robot settings");
    action.click();
    assert.deepEqual(h.calls, ["workspace", "render", "scroll"]);
    const accept = h.host.querySelector<HTMLButtonElement>(".accept-robot-settings")!;
    assert.equal(document.activeElement, accept);
    assert.equal(accept.disabled, false);
    assert.equal(h.host.querySelector("input, .robot-confirmation-code-row"), null);
    assert.match(h.host.textContent!, /previous robot settings are unavailable for comparison/);
    assert.match(h.host.textContent!, /change date was not recorded/);
    assert.equal(JSON.stringify(h.project), h.before);
    assert.match(accept.textContent!, /Accept current (?:robot )?settings/);
    accept.click();
    assert.deepEqual(h.calls, ["workspace", "render", "scroll", "adopt"]);
  } finally { h.close(); }
});

test("an unrelated planning failure has no repair action and leaves Workspace and robot settings untouched", () => {
  const h = harness();
  try {
    h.show("Collision on seam 3");
    const action = h.dialog.el.querySelector<HTMLButtonElement>(".progress-dialog-action")!;
    assert.equal(action.hidden, true);
    action.click();
    assert.deepEqual(h.calls, []);
    assert.equal(JSON.stringify(h.project), h.before);
    assert.equal(h.dialog.el.querySelector(".progress-dialog-note")!.textContent, "Collision on seam 3");
    assert.equal(h.host.childElementCount, 0);
  } finally { h.close(); }
});
