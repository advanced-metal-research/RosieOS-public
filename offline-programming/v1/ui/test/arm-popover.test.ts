import assert from "node:assert/strict";
import test from "node:test";
import { setImmediate } from "node:timers/promises";

import { createDogBoxProject } from "../src/examples/dog-box.ts";
import { DenseExecutionClient } from "../src/execution/denseClient.ts";
import { DensePanel } from "../src/execution/densePanel.ts";
import { machineReasonText } from "../src/execution/machineFlow.ts";
import { armDismiss } from "../src/ui/menu.ts";

class PanelElement extends EventTarget {
  innerHTML = "";
  textContent = "";
  title = "";
  value = "";
  hidden = false;
  disabled = false;
  classList = { toggle: (_name: string, force?: boolean) => force ?? false };
  dataset: Record<string, string> = {};
  options: PanelElement[] = [];
  nodes = new Map<string, PanelElement>();

  querySelector(selector: string): PanelElement {
    let node = this.nodes.get(selector);
    if (!node) {
      node = new PanelElement();
      this.nodes.set(selector, node);
    }
    return node;
  }

  append(...options: PanelElement[]): void { this.options.push(...options); }
  replaceChildren(): void { this.options = []; this.nodes.clear(); this.textContent = ""; }
  setAttribute(_name: string, _value: string): void {}
  remove(): void {}
  click(): void {
    assert.equal(this.disabled, false, "control must be enabled");
    this.dispatchEvent(new Event("click"));
  }
}

function installPanelDom(t: import("node:test").TestContext): void {
  t.mock.timers.enable({ apis: ["setTimeout"] });
  for (const [name, value] of Object.entries({
    window: new EventTarget(),
    document: Object.assign(new EventTarget(), { createElement: () => new PanelElement() }),
    sessionStorage: { getItem: () => "", setItem: () => undefined },
  })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
}

const simulationStatus = {
  backend: "rt_core", available: true,
  target: {
    selection_generation: "1", cell_id: "local-simulation", model_id: "rosie_1420_v1",
    label: "local-simulation", backend: "simulation", simulation: true, host: "localhost",
  },
  session: { id: "s", state: "idle", lease: { held: false } },
  rt_core: {
    status: {
      time_ns: 1_000_000, adapter_incarnation: "adapter", daemon_incarnation: "daemon",
      execution: { state: "idle" }, core: { armed: 0, safety_fault_mask: 0 },
      axes: [{ home_valid: true, logical_valid: true, readiness: "ready", logical_position: 0, logical_target: 0, statusword: 0, position_counts: 0 }],
    },
  },
};

test("machineReasonText prints an rt_core_event once, even when the HTTP pair already carries the code", () => {
  assert.equal(machineReasonText("rt_core_event: rt_core_event: fault_cleared"), "rt_core_event: fault_cleared");
  assert.equal(machineReasonText("rt_core_event: fault_cleared"), "rt_core_event: fault_cleared");
});

test("Cancel, Escape and an outside click dismiss a refused Arm; a later refusal reopens it", async (t) => {
  installPanelDom(t);
  const refusals = [
    { error: "rt_core_event", detail: "rt_core_event: fault_cleared" },
    { error: "rt_core_event", detail: "rt_core_event: grant_expired" },
  ];
  const logs: string[] = [];
  const client = new DenseExecutionClient(async (url) => {
    switch (String(url).split("/").at(-1)!) {
      case "capabilities":
        return Response.json({ backend: "rt_core", available: true, daemon_reachable: true });
      case "status":
        return Response.json(simulationStatus);
      case "arm": {
        const body = refusals.shift();
        assert.ok(body, "unexpected extra Arm");
        return Response.json(body, { status: 409 });
      }
      default:
        throw new Error(`unexpected request ${url}`);
    }
  });
  const popover = { hidden: true, contains: (node: Node | null) => node === inside };
  const inside = {} as Node;
  const badge = { title: "", contains: () => false };
  const reason = { textContent: "" };
  let release: (() => void) | null = null;
  const panel = new DensePanel({
    client, getProject: () => createDogBoxProject(), highlightNode: () => undefined,
    status: (text, isError) => { if (isError) logs.push(text); },
    onLockChange: () => undefined, onTargetChange: () => undefined, onRobotPose: () => undefined,
    onRobotMotion: () => undefined, wantsFastPoll: () => false,
    onFlowChange: () => paint(),
  });
  function paint(): void {
    popover.hidden = !panel.armPopoverOpen();
    reason.textContent = panel.armReason();
    badge.title = panel.armReason();
    if (popover.hidden) {
      release?.();
      release = null;
    } else if (!release) {
      release = armDismiss(popover as unknown as HTMLElement, badge as unknown as HTMLElement, () => panel.cancelArm());
    }
  }
  const host = new PanelElement();
  panel.mount(host as unknown as HTMLElement, host as unknown as HTMLElement);
  await setImmediate();
  const target = host.querySelector("#dense-target");
  target.value = "joint-daemon";
  target.dispatchEvent(new Event("change"));
  t.mock.timers.tick(0); await setImmediate();

  host.querySelector("#dense-arm-on").click();
  host.querySelector("#dense-arm-confirm").click();
  await setImmediate();
  assert.equal(popover.hidden, false, "a refused Arm opens the popover");
  assert.equal(reason.textContent, "rt_core_event: fault_cleared");
  assert.equal(badge.title, "rt_core_event: fault_cleared");
  assert.match(logs.at(-1)!, /rt_core_event/);

  panel.cancelArm();
  assert.equal(popover.hidden, true, "Cancel acknowledges the refusal");
  assert.equal(badge.title, "rt_core_event: fault_cleared", "the badge tooltip still carries the reason");
  assert.equal(panel.armReason(), "rt_core_event: fault_cleared");
  assert.equal(panel.armRefused(), false);

  t.mock.timers.tick(250); await setImmediate();
  assert.equal(popover.hidden, true, "an acknowledged refusal stays closed across a status poll");

  host.querySelector("#dense-arm-on").click();
  host.querySelector("#dense-arm-confirm").click();
  await setImmediate();
  assert.equal(popover.hidden, false, "a later refusal opens the popover again");
  assert.equal(reason.textContent, "rt_core_event: grant_expired");
  assert.equal(badge.title, "rt_core_event: grant_expired");

  await setImmediate();
  document.dispatchEvent(Object.assign(new Event("keydown"), { key: "Escape" }));
  assert.equal(popover.hidden, true, "Escape dismisses like Cancel");
  assert.equal(badge.title, "rt_core_event: grant_expired");
});

test("armDismiss closes on Escape and a pointerdown outside, and ignores one inside", async (t) => {
  const { Window } = await import("happy-dom");
  const browser = new Window();
  const bindings: Record<string, unknown> = {
    window: browser,
    document: browser.document,
    HTMLElement: browser.HTMLElement,
    Node: browser.Node,
    KeyboardEvent: browser.KeyboardEvent,
    PointerEvent: browser.PointerEvent,
  };
  for (const [name, value] of Object.entries(bindings)) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }

  const popover = document.createElement("div");
  const badge = document.createElement("button");
  const outside = document.createElement("div");
  document.body.append(badge, popover, outside);
  let closed = 0;
  const release = armDismiss(popover, badge, () => {
    closed += 1;
    release();
  });
  await setImmediate();

  popover.dispatchEvent(new browser.PointerEvent("pointerdown", { bubbles: true }) as unknown as PointerEvent);
  assert.equal(closed, 0, "a pointerdown inside the popover is not a dismiss");
  badge.dispatchEvent(new browser.PointerEvent("pointerdown", { bubbles: true }) as unknown as PointerEvent);
  assert.equal(closed, 0, "a pointerdown on the arm badge is not a dismiss");

  outside.dispatchEvent(new browser.PointerEvent("pointerdown", { bubbles: true }) as unknown as PointerEvent);
  assert.equal(closed, 1, "a pointerdown outside dismisses");

  document.dispatchEvent(new browser.KeyboardEvent("keydown", { key: "Escape", bubbles: true }) as unknown as KeyboardEvent);
  assert.equal(closed, 1, "release drops the listeners");
});
