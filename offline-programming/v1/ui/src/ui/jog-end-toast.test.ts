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
  Event: browser.Event,
});

const { JogEndToast } = await import("./jog-end-toast.ts");
const { JogEndTracker, JOG_END_TOAST_MS } = await import("../execution/jogEnd.ts");

const ME = "tab-0123456789abcdef";
const NOW = Date.parse("2026-10-10T22:40:00Z");
const singularity = {
  seq: 3, code: "singularity_gate", cause: "jacobian_gate", repull: "no", popup: true, disarmed: false,
  text: "Jog stopped: the robot reached a singularity (J5 = -88.3°, J3 = 40.1°); jog back the way you came, or reposition with joint jog.",
  detail: "cause=jacobian_gate: the resolver's Jacobian gate ended the line native_stop=path_end governing=path_end samples_ahead=0 underruns=0",
  kind: "cartesian", axis: "-Z", frame: "tool", client_id: ME, client_label: "Browser", at: "2026-10-10T22:39:59.9Z", age_ms: 100,
  speed_unit: "mm/s", requested_speed: 120, achieved_speed: 118, distance_unit: "mm", distance: 41.5, duration_ms: 610,
  governing: "path_end", samples_ahead: 0, last_intent_age_ms: 22, lifetime_ms: 250,
};

function setup() {
  document.body.replaceChildren();
  const jogPad = document.createElement("button");
  jogPad.id = "jog-pad";
  document.body.append(jogPad);
  const tracker = new JogEndTracker(ME);
  const calls: string[] = [];
  let now = NOW;
  const toast = new JogEndToast({
    onDismiss: () => { calls.push("dismiss"); tracker.dismiss(); toast.render(tracker.notice(now)); },
    onDetails: () => { calls.push("details"); tracker.toggleDetails(now); toast.render(tracker.notice(now)); },
  }).mount(document.body);
  return { tracker, toast, calls, jogPad, at: (t: number) => { now = t; } };
}

test("shows the operator text for the owner's drop-out, non-blocking", () => {
  const { tracker, toast } = setup();
  assert.equal(toast.el.hidden, true);
  tracker.observe({ last_jog_end: singularity }, NOW);
  toast.render(tracker.notice(NOW));
  assert.equal(toast.el.hidden, false);
  assert.equal(toast.el.getAttribute("role"), "status");
  assert.equal(toast.el.getAttribute("aria-live"), "polite");
  assert.equal(toast.el.querySelector(".jog-end-kicker")!.textContent, "JOG STOPPED");
  assert.match(toast.el.querySelector(".jog-end-text")!.textContent!, /reached a singularity \(J5 = -88\.3°, J3 = 40\.1°\)/);
  assert.equal(toast.el.dataset.code, "singularity_gate");
  assert.equal(toast.el.classList.contains("is-note"), false);
  assert.equal(toast.el.querySelector<HTMLElement>(".jog-end-details")!.hidden, true);
  // Showing it never takes focus from what the operator is jogging with.
  assert.notEqual(document.activeElement, toast.el.querySelector(".jog-end-dismiss"));
});

test("Details expands the technical detail; × dismisses", () => {
  const { tracker, toast, calls } = setup();
  tracker.observe(singularity, NOW);
  toast.render(tracker.notice(NOW));
  toast.el.querySelector<HTMLButtonElement>(".jog-end-toggle")!.click();
  const details = toast.el.querySelector<HTMLElement>(".jog-end-details")!;
  assert.equal(details.hidden, false);
  assert.equal(toast.el.querySelector(".jog-end-toggle")!.getAttribute("aria-expanded"), "true");
  assert.match(details.textContent!, /Pulling the trigger again in the same direction stops again\./);
  assert.match(details.textContent!, /singularity_gate · cause jacobian_gate/);
  assert.match(details.textContent!, /cartesian -Z \(tool\) · asked 120\.0 mm\/s/);
  toast.el.querySelector<HTMLButtonElement>(".jog-end-dismiss")!.click();
  assert.equal(toast.el.hidden, true);
  assert.deepEqual(calls, ["details", "dismiss"]);
});

test("not shown for a release or the owner's own Stop; gone after the timeout", () => {
  const { tracker, toast } = setup();
  tracker.observe({ ...singularity, seq: 4, code: "released", popup: false, text: "Jog released." }, NOW);
  toast.render(tracker.notice(NOW));
  assert.equal(toast.el.hidden, true);
  tracker.observe({ ...singularity, seq: 5, code: "halted", cause: "operator_stop", popup: false, disarmed: true }, NOW);
  toast.render(tracker.notice(NOW));
  assert.equal(toast.el.hidden, true);
  tracker.observe({ ...singularity, seq: 6 }, NOW);
  toast.render(tracker.notice(NOW));
  assert.equal(toast.el.hidden, false);
  toast.render(tracker.notice(NOW + JOG_END_TOAST_MS));
  assert.equal(toast.el.hidden, true);
});

test("another client's drop-out is a quiet note", () => {
  const { tracker, toast } = setup();
  tracker.observe({ ...singularity, client_id: "deck-00112233aabbccdd", client_label: "rosie-pendant-v5 steamdeck-pendant" }, NOW);
  toast.render(tracker.notice(NOW));
  assert.equal(toast.el.classList.contains("is-note"), true);
  assert.equal(toast.el.querySelector(".jog-end-kicker")!.textContent, "JOG NOTE");
  assert.match(toast.el.querySelector(".jog-end-text")!.textContent!, /^rosie-pendant-v5 steamdeck-pendant: Jog stopped/);
});
