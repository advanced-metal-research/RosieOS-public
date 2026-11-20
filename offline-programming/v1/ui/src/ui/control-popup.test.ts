import assert from "node:assert/strict";
import test from "node:test";
import { readFileSync } from "node:fs";
import { Window } from "happy-dom";

const browser = new Window();
Object.assign(globalThis, {
  window: browser,
  document: browser.document,
  HTMLElement: browser.HTMLElement,
  Node: browser.Node,
  MouseEvent: browser.MouseEvent,
  Event: browser.Event,
});

const { ControlPopup } = await import("./control-popup.ts");
const { ControlTracker } = await import("../execution/controlNotice.ts");

const NOW = Date.parse("2026-10-07T22:05:00Z");
const otherArmed = {
  state: "other", now: "2026-10-07T22:05:00Z", armed: true, lease_held: true, take_control: false, stop_available: true,
  take_control_note: "It is armed: control is never taken from an armed controller.",
  owner: { kind: "pendant", label: "rosie-pendant-v5 (steamdeck-pendant)", address: "100.64.0.15", loopback: false,
    since: "2026-10-07T22:01:00Z", last_action: "jog/begin", last_action_at: "2026-10-07T22:04:56Z" },
};

function setup() {
  document.body.replaceChildren();
  // The title bar's STOP, which the pop-up must never cover or block.
  const titleStop = document.createElement("button");
  titleStop.id = "cell-stop-pill";
  document.body.append(titleStop);
  const calls: string[] = [];
  const tracker = new ControlTracker();
  const popup = new ControlPopup({
    onWait: () => { calls.push("wait"); tracker.wait(NOW); popup.render(tracker.notice(NOW)); },
    onExpand: () => { calls.push("expand"); tracker.expand(); popup.render(tracker.notice(NOW)); },
    onStop: () => calls.push("stop"),
    onTake: () => calls.push("take"),
  }).mount(document.body);
  return { titleStop, calls, tracker, popup };
}

function button(root: HTMLElement, selector: string): HTMLButtonElement {
  return root.querySelector<HTMLButtonElement>(selector)!;
}

test("shows who holds the robot, with Stop and Wait, and clears when free", () => {
  const { tracker, popup, calls } = setup();
  assert.equal(popup.el.hidden, true);
  tracker.observe({ control: otherArmed }, NOW);
  popup.render(tracker.notice(NOW));
  assert.equal(popup.el.hidden, false);
  assert.equal(popup.el.getAttribute("role"), "dialog");
  assert.equal(popup.el.getAttribute("aria-modal"), "false");
  assert.match(popup.el.textContent!, /Another controller is using the robot/);
  assert.match(popup.el.textContent!, /rosie-pendant-v5 \(steamdeck-pendant\) · from 100\.64\.0\.15/);
  assert.match(popup.el.textContent!, /ARMED · last request jog\/begin/);
  assert.ok(popup.el.classList.contains("is-armed"));
  assert.equal(button(popup.el, ".control-stop").hidden, false);
  assert.equal(button(popup.el, ".control-take").hidden, true, "never offered against an armed holder");
  button(popup.el, ".control-stop").click();
  assert.deepEqual(calls, ["stop"]);
  tracker.observe({ control: { state: "free" } }, NOW);
  popup.render(tracker.notice(NOW));
  assert.equal(popup.el.hidden, true);
});

test("Wait collapses to a slim bar; Details expands it again", () => {
  const { tracker, popup, calls } = setup();
  tracker.observe({ control: otherArmed }, NOW);
  popup.render(tracker.notice(NOW));
  button(popup.el, ".control-wait").click();
  assert.equal(popup.el.hidden, false);
  assert.equal(popup.el.querySelector<HTMLElement>(".control-card")!.hidden, true);
  assert.equal(popup.el.querySelector<HTMLElement>(".control-bar")!.hidden, false);
  assert.match(popup.el.querySelector(".control-summary")!.textContent!, /Robot in use by rosie-pendant-v5 \(steamdeck-pendant\) · ARMED/);
  button(popup.el, ".control-details").click();
  assert.equal(popup.el.querySelector<HTMLElement>(".control-card")!.hidden, false);
  assert.deepEqual(calls, ["wait", "expand"]);
});

test("Take control appears only when allowed and waits for its answer", () => {
  const { tracker, popup, calls } = setup();
  tracker.observe({ control: { ...otherArmed, armed: false, take_control: true, take_control_note: "It is disarmed and has been idle for 45s." } }, NOW);
  popup.render(tracker.notice(NOW));
  const take = button(popup.el, ".control-take");
  assert.equal(take.hidden, false);
  take.click();
  popup.taking = true;
  take.click();
  assert.deepEqual(calls, ["take"], "a second press while the first is in flight is ignored");
  assert.equal(take.textContent, "Taking control…");
});

test("another server's grant: no Stop button, the hardware stop named", () => {
  const { tracker, popup } = setup();
  tracker.observe({ control: { state: "other_server", stop_available: false, servo_holder: { controller: "offline-programming:server-deck01", remote: true, identity: "steamdeck-pendant-olp", since: "2026-10-07T21:58:00Z" } } }, NOW);
  popup.render(tracker.notice(NOW));
  assert.equal(button(popup.el, ".control-stop").hidden, true);
  assert.match(popup.el.textContent!, /hardware stop/);
});

test("the title bar's STOP stays visible and pressable while the pop-up is up", () => {
  const { titleStop, tracker, popup } = setup();
  tracker.observe({ control: otherArmed }, NOW);
  popup.render(tracker.notice(NOW));
  assert.equal(titleStop.hasAttribute("inert"), false);
  assert.equal(titleStop.getAttribute("aria-hidden"), null);
  let pressed = 0;
  titleStop.addEventListener("click", () => pressed++);
  titleStop.click();
  assert.equal(pressed, 1);
  // No scrim and no full-window box: anchored to the bottom, and the box
  // itself lets presses through to what is under its margins.
  const css = readFileSync(new URL("../style.css", import.meta.url), "utf8");
  const rule = css.match(/\.control-popup \{([^}]*)\}/)![1]!;
  assert.match(rule, /bottom: 16px/);
  assert.match(rule, /pointer-events: none/);
  assert.doesNotMatch(rule, /inset: 0|top: 0|background/);
  const maintenance = css.match(/\.maintenance-overlay \{([^}]*)\}/)![1]!;
  assert.ok(Number(rule.match(/z-index: (\d+)/)![1]) < Number(maintenance.match(/z-index: (\d+)/)![1]), "under the maintenance overlay");
});
