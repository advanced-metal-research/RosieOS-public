import assert from "node:assert/strict";
import test from "node:test";
import { Window } from "happy-dom";

const browser = new Window();
Object.assign(globalThis, {
  window: browser,
  document: browser.document,
  HTMLElement: browser.HTMLElement,
  Node: browser.Node,
  MouseEvent: browser.MouseEvent,
  KeyboardEvent: browser.KeyboardEvent,
  Event: browser.Event,
  MutationObserver: browser.MutationObserver,
});

const { MaintenanceOverlay } = await import("./maintenance-overlay.ts");
const { MaintenanceTracker } = await import("../execution/maintenance.ts");

const T0 = Date.parse("2026-10-07T20:00:00Z");
const active = {
  active: true, reason: "rt-core full deploy fd9af899", phase: "restarting the controller",
  started_at: "2026-10-07T19:50:00Z", eta: "2026-10-07T20:30:00Z", set_by: "realtime-core deploy (operator@example-relay)",
  release: "fd9af899", now: "2026-10-07T20:00:00Z", server: { build_id: "sha256:old000000000" },
};

function setup(connected = true) {
  document.body.replaceChildren();
  const app = document.createElement("div");
  app.id = "app";
  const arm = document.createElement("button");
  arm.id = "arm";
  app.append(arm);
  document.body.append(app);
  const calls: string[] = [];
  const overlay = new MaintenanceOverlay({
    onStop: () => calls.push("stop"),
    stopAvailable: () => connected,
    onReload: () => calls.push("reload"),
    onDismissReload: () => calls.push("later"),
  }).mount(document.body);
  const tracker = new MaintenanceTracker();
  return { app, arm, overlay, tracker, calls };
}

function dispatch(target: EventTarget, event: Event): boolean {
  return target.dispatchEvent(event);
}

test("an active notice covers the app as a modal alert with the countdown and no dismiss", () => {
  const { app, overlay, tracker } = setup();
  tracker.observe({ kind: "notice", payload: active }, T0);
  overlay.render(tracker.display(T0 + 1000));
  const el = overlay.el;
  assert.equal(el.hidden, false);
  assert.equal(el.getAttribute("role"), "alertdialog");
  assert.equal(el.getAttribute("aria-modal"), "true");
  assert.equal(document.getElementById(el.getAttribute("aria-labelledby")!)!.textContent, "Update in progress");
  assert.equal(document.getElementById(el.getAttribute("aria-describedby")!)!.textContent, "rt-core full deploy fd9af899");
  assert.match(el.textContent!, /Now: restarting the controller/);
  assert.equal(el.querySelector(".maintenance-countdown-label")!.textContent, "Expected finish");
  assert.equal(el.querySelector(".maintenance-countdown")!.textContent, "About 30 min left");
  assert.match(el.querySelector(".maintenance-elapsed")!.textContent!, /Running for 10:01/);
  assert.match(el.textContent!, /release fd9af899/);
  assert.equal(app.hasAttribute("inert"), true, "the app under the notice is inert");
  assert.equal(app.getAttribute("aria-hidden"), "true");
  const visibleButtons = [...el.querySelectorAll<HTMLButtonElement>("button")].filter((b) => !b.hidden).map((b) => b.textContent);
  assert.deepEqual(visibleButtons, ["■ STOP"], "STOP is the only control while blocking: there is no dismiss");
  overlay.dispose();
});

test("a run's progress draws the step, the gate's suite, a bar and the step list; without it none show", () => {
  const { overlay, tracker } = setup();
  tracker.observe({ kind: "notice", payload: active }, T0);
  overlay.render(tracker.display(T0));
  const el = overlay.el;
  for (const part of [".maintenance-step", ".maintenance-suite", ".maintenance-progress", ".maintenance-steps"]) {
    assert.equal(el.querySelector<HTMLElement>(part)!.hidden, true, `${part} without progress`);
  }
  const progress = {
    steps: [
      { name: "build+test", label: "Build and test gate", state: "done", estimate_s: 1077, elapsed_s: 1100, remaining_s: 0 },
      { name: "deploy", label: "Install rt-core on the servo host", state: "running", estimate_s: 36, elapsed_s: 10, remaining_s: 26 },
      { name: "verify", label: "Verify every axis", state: "pending", estimate_s: 8, elapsed_s: 0, remaining_s: 8 },
    ],
    current: 2, total: 3, fraction: 0.97, updated_at: "2026-10-07T20:00:00Z",
    suite: null,
  };
  tracker.observe({ kind: "notice", payload: { ...active, phase: "copying the package", progress } }, T0);
  overlay.render(tracker.display(T0));
  assert.equal(el.querySelector(".maintenance-step")!.textContent, "Step 2/3 · Install rt-core on the servo host");
  assert.equal(el.querySelector<HTMLElement>(".maintenance-suite")!.hidden, true, "no gate running");
  const bar = el.querySelector<HTMLElement>(".maintenance-progress")!;
  assert.equal(bar.hidden, false);
  assert.equal(bar.getAttribute("role"), "progressbar");
  // (1077 + 36 x 10/36) / 1121 = 0.9697: weighted by estimated time.
  assert.equal(bar.getAttribute("aria-valuenow"), "96");
  assert.equal(el.querySelector<HTMLElement>(".maintenance-progress-fill")!.style.width, "97%");
  assert.equal(el.querySelector(".maintenance-progress-text")!.textContent, "96%");
  assert.match(el.textContent!, /Now: copying the package/);
  const rows = [...el.querySelectorAll(".maintenance-step-row")].map((row) => row.textContent);
  assert.deepEqual(rows, ["✓Build and test gate18:20", "▸Install rt-core on the servo host0:10 of about 1 min", "·Verify every axisabout 1 min"]);
  assert.equal(el.querySelector(".maintenance-step-row.is-running")!.getAttribute("aria-current"), "step");
  assert.match(el.querySelector(".maintenance-source")!.textContent!, /^Started by realtime-core deploy \(operator@example-relay\)/);
  overlay.dispose();
});

test("presses, clicks, keys and the wheel never reach the app; releases always do", () => {
  const { app, arm, overlay, tracker } = setup();
  let armed = 0, shortcuts = 0, releases = 0;
  arm.addEventListener("click", () => armed++);
  document.addEventListener("keydown", () => shortcuts++);
  window.addEventListener("pointerup", () => releases++);
  window.addEventListener("pointercancel", () => releases++);
  window.addEventListener("keyup", () => releases++);
  tracker.observe({ kind: "notice", payload: active }, T0);
  overlay.render(tracker.display(T0));

  const click = new browser.MouseEvent("click", { bubbles: true, cancelable: true }) as unknown as Event;
  assert.equal(dispatch(arm, click), false, "the click is cancelled");
  assert.equal(armed, 0);
  for (const type of ["pointerdown", "mousedown", "contextmenu", "dblclick"]) {
    dispatch(app, new browser.MouseEvent(type, { bubbles: true, cancelable: true }) as unknown as Event);
  }
  dispatch(document.body, new browser.KeyboardEvent("keydown", { key: " ", bubbles: true, cancelable: true }) as unknown as Event);
  dispatch(overlay.el, new browser.KeyboardEvent("keydown", { key: "Escape", bubbles: true, cancelable: true }) as unknown as Event);
  assert.equal(shortcuts, 0, "no app shortcut runs, from the app or the overlay");

  // A jog held when the notice appeared still ends on release.
  dispatch(app, new browser.Event("pointerup", { bubbles: true }) as unknown as Event);
  dispatch(overlay.el, new browser.Event("pointercancel", { bubbles: true }) as unknown as Event);
  dispatch(document.body, new browser.KeyboardEvent("keyup", { key: "ArrowUp", bubbles: true }) as unknown as Event);
  assert.equal(releases, 3);
  overlay.dispose();
});

test("STOP on the overlay stops the connected robot; it is hidden with no cell", () => {
  const { overlay, tracker, calls } = setup(true);
  tracker.observe({ kind: "notice", payload: active }, T0);
  overlay.render(tracker.display(T0));
  const stop = overlay.el.querySelector<HTMLButtonElement>(".maintenance-stop")!;
  stop.click();
  assert.deepEqual(calls, ["stop"]);
  overlay.dispose();
  const offline = setup(false);
  offline.tracker.observe({ kind: "notice", payload: active }, T0);
  offline.overlay.render(offline.tracker.display(T0));
  assert.equal(offline.overlay.el.querySelector<HTMLButtonElement>(".maintenance-stop")!.hidden, true);
  offline.overlay.dispose();
});

test("unreachable during a notice keeps blocking and says so; overrun turns the clock red", () => {
  const { overlay, tracker } = setup();
  tracker.observe({ kind: "notice", payload: { ...active, eta: "2026-10-07T19:59:00Z" } }, T0);
  tracker.observe({ kind: "unreachable", error: "Failed to fetch" }, T0 + 2000);
  overlay.render(tracker.display(T0 + 2000));
  assert.equal(overlay.blocking, true);
  assert.ok(overlay.el.classList.contains("is-unreachable"));
  assert.ok(overlay.el.classList.contains("is-overrun"));
  assert.equal(overlay.el.querySelector(".maintenance-countdown-label")!.textContent, "Past the expected finish");
  assert.equal(overlay.el.querySelector(".maintenance-countdown")!.textContent, "Taking longer than expected (+1 min)");
  assert.match(overlay.el.querySelector(".maintenance-kicker")!.textContent!, /SERVER NOT ANSWERING/);
  overlay.dispose();
});

test("when the notice clears the app is restored, with a reload offer after a server update", async () => {
  const { app, arm, overlay, tracker, calls } = setup();
  let armed = 0;
  arm.addEventListener("click", () => armed++);
  tracker.observe({ kind: "notice", payload: active }, T0);
  overlay.render(tracker.display(T0));
  // A dialog opened during the notice goes under it too.
  const late = document.createElement("section");
  document.body.append(late);
  await new Promise((resolve) => setTimeout(resolve, 0));
  assert.equal(late.hasAttribute("inert"), true);

  tracker.observe({ kind: "notice", payload: { active: false, now: active.now, server: { build_id: "sha256:new000000000" } } }, T0 + 2000);
  tracker.confirmHealth(true, "sha256:new000000000");
  overlay.render(tracker.display(T0 + 2000));
  assert.equal(overlay.blocking, false);
  assert.equal(app.hasAttribute("inert"), false);
  assert.equal(late.hasAttribute("inert"), false);
  arm.click();
  assert.equal(armed, 1, "the app takes input again");
  assert.equal(overlay.el.hidden, false);
  assert.equal(overlay.el.querySelector(".maintenance-title")!.textContent, "Update finished");
  assert.match(overlay.el.textContent!, /sha256:old000000000 → sha256:new000000000/);
  overlay.el.querySelector<HTMLButtonElement>(".maintenance-reload")!.click();
  overlay.el.querySelector<HTMLButtonElement>(".maintenance-later")!.click();
  assert.deepEqual(calls, ["reload", "later"]);
  assert.equal(overlay.el.hidden, true);

  tracker.dismissReload();
  overlay.render(tracker.display(T0 + 3000));
  assert.equal(overlay.el.hidden, true);
  overlay.dispose();
});

test("the notice names who is connected, the holder first, and hides the list when nobody is", () => {
  const { overlay, tracker } = setup();
  tracker.observe({ kind: "notice", payload: active }, T0);
  overlay.render(tracker.display(T0), [
    { kind: "pendant", text: "rosie-pendant-v5 (steamdeck-pendant) · from 192.168.1.113 · seen 1 s ago", holds: false },
    { kind: "browser", text: "Browser (Chrome on macOS) on http://127.0.0.1:5189 · holds the execution session", holds: true },
  ]);
  const items = [...overlay.el.querySelectorAll<HTMLLIElement>(".maintenance-connected-list li")];
  assert.deepEqual(items.map((item) => item.textContent), [
    "Browser (Chrome on macOS) on http://127.0.0.1:5189 · holds the execution session",
    "rosie-pendant-v5 (steamdeck-pendant) · from 192.168.1.113 · seen 1 s ago",
  ]);
  assert.ok(items[0]!.classList.contains("is-holder"));
  assert.match(items[0]!.getAttribute("aria-label")!, /^Holds the machine:/);
  assert.equal(overlay.el.querySelector<HTMLElement>(".maintenance-connected")!.hidden, false);
  overlay.render(tracker.display(T0), []);
  assert.equal(overlay.el.querySelector<HTMLElement>(".maintenance-connected")!.hidden, true);
  overlay.dispose();
});
