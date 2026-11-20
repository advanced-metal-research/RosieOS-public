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
  performance: globalThis.performance,
});

const {
  ProgressDialog,
  planRefusalText,
} = await import("./progress-dialog.ts");

const OWNER_REFUSAL = {
  reason: "motion_not_verified",
  detail: "seam seam_0011: FAIL (1 collisions; 2 limit violations); requires native PASS",
  segment_index: null as number | null,
};

const OWNER_SENTENCE =
  "seam seam_0011: FAIL (1 collisions; 2 limit violations); requires native PASS · motion_not_verified";

function barOf(dialog: InstanceType<typeof ProgressDialog>): HTMLElement {
  return dialog.el.querySelector(".progress-dialog-bar")!;
}

function fillOf(dialog: InstanceType<typeof ProgressDialog>): HTMLElement {
  return dialog.el.querySelector<HTMLElement>(".progress-dialog-fill")!;
}

test("Cancel requests cancellation once and waits for the job to settle", () => {
  const dialog = new ProgressDialog();
  let cancelled = 0;
  dialog.start("Planning", () => cancelled++);
  const button = dialog.el.querySelector<HTMLButtonElement>(".progress-dialog-cancel")!;
  assert.equal(button.hidden, false);
  button.click(); button.click();
  assert.equal(cancelled, 1);
  assert.equal(button.disabled, true);
  assert.equal(dialog.shown, true);
  assert.match(dialog.el.textContent!, /Cancelling planning/);
  dialog.cancelled();
  assert.equal(button.hidden, true);
  assert.equal(dialog.el.classList.contains("is-failed"), false);
  assert.match(dialog.el.textContent!, /No new plan was saved/);
  dialog.start("Next plan", () => cancelled++);
  dialog.setCancelAction(null);
  button.click();
  assert.equal(cancelled, 1, "committing a result has no stale cancel action");
  dialog.start("Uncancellable job");
  assert.equal(button.hidden, true);
  dialog.dispose();
});

test("a progress dialog counts stages and reports elapsed time", async () => {
  const dialog = new ProgressDialog();
  document.body.append(dialog.el);
  dialog.start("Planning");
  assert.equal(dialog.shown, true);
  assert.ok(barOf(dialog).classList.contains("is-indeterminate"));
  assert.equal(barOf(dialog).getAttribute("aria-valuenow"), "0");
  dialog.stage({ index: 2, count: 4, label: "Planning on the weld planner" });
  const stage = dialog.el.querySelector(".progress-dialog-stage")!.textContent;
  assert.equal(stage, "Planning on the weld planner (2/4)");
  await new Promise((resolve) => setTimeout(resolve, 150));
  assert.match(dialog.el.querySelector(".progress-dialog-elapsed")!.textContent ?? "", /^\d+\.\d s$/);
  dialog.finish("12 samples", 10);
  assert.equal(fillOf(dialog).style.width, "100%");
  await new Promise((resolve) => setTimeout(resolve, 40));
  assert.equal(dialog.shown, false, "a finished job closes on its own");
  dialog.dispose();
});

test("the bar fills by completed stage fraction", () => {
  const dialog = new ProgressDialog();
  document.body.append(dialog.el);
  dialog.start("Planning");
  assert.ok(barOf(dialog).classList.contains("is-indeterminate"));
  dialog.stage({ index: 1, count: 4, label: "pack" });
  dialog.stage({ index: 2, count: 4, label: "plan" });
  assert.equal(barOf(dialog).getAttribute("aria-valuenow"), "50");
  assert.equal(fillOf(dialog).style.width, "50%");
  assert.equal(barOf(dialog).classList.contains("is-indeterminate"), false);
  dialog.stage({ index: 4, count: 4, label: "load" });
  assert.equal(barOf(dialog).getAttribute("aria-valuenow"), "100");
  assert.equal(fillOf(dialog).style.width, "100%");
  dialog.dispose();
});

test("a failed plan keeps the fill it reached", () => {
  const dialog = new ProgressDialog();
  document.body.append(dialog.el);
  dialog.start("Planning");
  dialog.stage({ index: 2, count: 4, label: "plan" });
  dialog.fail("the planner refused");
  assert.equal(barOf(dialog).getAttribute("aria-valuenow"), "50");
  assert.equal(fillOf(dialog).style.width, "50%");
  assert.ok(dialog.el.classList.contains("is-failed"));
  assert.equal(barOf(dialog).classList.contains("is-indeterminate"), false);
  assert.equal(dialog.el.querySelector<HTMLElement>(".progress-dialog-close")!.hidden, false);
  dialog.dispose();
});

test("a refused plan is a sentence, not the JSON object", () => {
  const dialog = new ProgressDialog();
  document.body.append(dialog.el);
  dialog.start("Planning");
  dialog.fail(OWNER_REFUSAL);
  assert.equal(dialog.el.querySelector(".progress-dialog-note")!.textContent, OWNER_SENTENCE);
  assert.equal(planRefusalText(OWNER_REFUSAL), OWNER_SENTENCE);
  dialog.fail(
    `the plan produced no trajectory: ${JSON.stringify(OWNER_REFUSAL)}`,
  );
  assert.equal(dialog.el.querySelector(".progress-dialog-note")!.textContent, OWNER_SENTENCE);
  dialog.dispose();
});

test("finer seam progress fills inside the current stage", () => {
  const dialog = new ProgressDialog();
  document.body.append(dialog.el);
  dialog.start("Planning");
  dialog.stage({ index: 2, count: 4, label: "plan", done: 1, total: 2 });
  // Stage 2 of 4, one of two seams: (1 + 1/2) / 4 = 37.5% → 38.
  assert.equal(barOf(dialog).getAttribute("aria-valuenow"), "38");
  assert.equal(fillOf(dialog).style.width, "38%");
  assert.equal(barOf(dialog).classList.contains("is-indeterminate"), false);
  dialog.dispose();
});

test("a refused plan names the segment when the refusal does", () => {
  const dialog = new ProgressDialog();
  document.body.append(dialog.el);
  dialog.start("Planning");
  dialog.fail({ ...OWNER_REFUSAL, segment_index: 3 });
  assert.equal(
    dialog.el.querySelector(".progress-dialog-note")!.textContent,
    `${OWNER_SENTENCE} · segment 3`,
  );
  dialog.dispose();
});

test("failure actions open the repair destination once and diagnostics stay secondary", () => {
  const dialog = new ProgressDialog();
  document.body.append(dialog.el);
  let opened = 0;
  const options = {
    details: "sha256:old differs from sha256:new",
    action: { label: "Review robot settings", run: () => {
      assert.equal(dialog.shown, false, "close the failure before showing its repair destination");
      opened++;
    } },
  };
  dialog.start("Planning");
  dialog.fail("Robot settings changed", options);
  const details = dialog.el.querySelector<HTMLDetailsElement>(".progress-dialog-details")!;
  const action = dialog.el.querySelector<HTMLButtonElement>(".progress-dialog-action")!;
  assert.equal(details.hidden, false);
  assert.equal(details.open, false);
  assert.match(details.textContent!, /sha256:old/);
  assert.equal(action.hidden, false);
  assert.equal(action.textContent, "Review robot settings");
  action.click();
  action.click();
  assert.equal(opened, 1);

  dialog.start("Planning again");
  assert.equal(details.hidden, true);
  assert.equal(action.hidden, true);
  dialog.fail("Other failure");
  action.click();
  assert.equal(opened, 1, "a later failure cannot invoke the previous repair action");
  assert.equal(details.hidden, true);
  dialog.fail("Robot settings changed", options);
  dialog.start("New job");
  action.click();
  assert.equal(opened, 1, "starting a new job clears the repair callback");
  dialog.dispose();
});
