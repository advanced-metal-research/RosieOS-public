import assert from "node:assert/strict";
import test from "node:test";
import { planningFailure } from "./planning-failure.ts";

const drift = "seam.build_plan_request: the program is written for 'rosie_1400_v3' at description sha256:803ab172c4e2... and this request packs sha256:686029493a22...; the description moved under the program -- adopt it in the authoring tool (typed) and re-plan";

test("description drift gets a plain explanation and robot settings action, with hashes only in details", () => {
  for (const error of [drift, new Error(drift), { reason: "description_mismatch", detail: drift, segment_index: null }]) {
    const failure = planningFailure(error);
    assert.match(failure.message, /Robot settings have changed/);
    assert.match(failure.message, /Review and accept/);
    assert.doesNotMatch(failure.message, /sha256|seam\.build_plan_request/);
    assert.match(failure.details!, /sha256:803ab/);
    assert.deepEqual(failure.action, { key: "review-robot-settings", label: "Review robot settings" });
  }
});

// Only an unverified measured start supports this alternative. An internal
// path refusal remains a refusal regardless of the plan's entry position.
test("an unverified measured start offers program start without replacing motion or settings instructions", () => {
  const unverified = Object.assign(new Error("Home is running (axis mask 0x1); plan after it finishes"), { code: "planning_start_unverified" });
  assert.deepEqual(planningFailure(unverified, "desktop", true).action, { key: "plan-from-program-start", label: "Plan from program start" });
  assert.equal(planningFailure(unverified, "desktop", false).action, undefined, "an offline plan already starts at the program's start");
  const moving = Object.assign(new Error("Release the jog and wait for the robot to stop before planning"), { code: "planning_start_moving" });
  assert.equal(planningFailure(moving, "desktop", true).action, undefined);
  assert.equal(planningFailure(drift, "desktop", true).action?.key, "review-robot-settings");
});

test("internal MoveL refusal preserves its exact program location and cause without entry advice", () => {
  const detail = "MoveL to P2 (program step 2, node e1c4fa37-9ca0-4eb5-8633-395fd1021913): " +
    "Cartesian tracking tolerance exceeded (0.13063 mm / 0.000262 rad)";
  const error = Object.assign(new Error("Planning failed"), {
    code: "motion_plan_failed", dense_error: { reason: "motion_join_failed", detail, segment_index: null },
  });
  for (const measuredStart of [true, false]) {
    const failure = planningFailure(error, "pendant", measuredStart);
    assert.ok(failure.message.includes(detail));
    assert.match(failure.message, /motion_join_failed/);
    assert.equal(failure.action, undefined);
    assert.doesNotMatch(failure.message, /choose.*program.*start|go to plan start/i);
  }
});

test("tracking, collision and transport failures cannot recommend a different plan start", () => {
  for (const error of [
    Object.assign(new Error("Motion server is unreachable"), { code: "motion_plan_failed" }),
    Object.assign(new Error("target_lead: tracking limit exceeded"), { code: "target_lead" }),
    Object.assign(new Error("Cartesian tracking tolerance exceeded"), { code: "motion_join_failed" }),
    Object.assign(new Error("requires native PASS: 1 collision"), { code: "motion_not_verified" }),
    new Error("The CAD service did not respond"),
  ]) {
    const failure = planningFailure(error, "pendant", true);
    assert.ok(failure.message.length > 0);
    assert.equal(failure.action, undefined);
  }
});

test("unrelated refusals retain their actual reason and have no settings action", () => {
  assert.deepEqual(planningFailure(new Error("The CAD service did not respond")), {
    message: "The CAD service did not respond",
  });
  assert.equal(planningFailure("the program is written for robot 'other' but cell is 'rosie'").action, undefined);
});


test("native FAIL explains the block without interpreting aggregate checks as six joint limits", () => {
  const detail = "seam seam_0011: FAIL (1 collisions; 1 penetrating; 6 limit violations); requires native PASS";
  for (const error of [{ reason: "motion_not_verified", detail, segment_index: null },
    new Error(`No trajectory: ${JSON.stringify({ reason: "motion_not_verified", detail, segment_index: null })}`),
    `${detail} · motion_not_verified`]) {
    const failure = planningFailure(error);
    assert.match(failure.message, /no verified motion path.*seam_0011/);
    assert.match(failure.message, /1 collision \(1 overlap\).*6 limit-check failures/);
    assert.match(failure.message, /seam, torch pose and part placement/);
    assert.match(failure.message, /do not count distinct joints/);
    assert.doesNotMatch(failure.message, /\bFAIL\b|native PASS|motion_not_verified|six joints|6 joints|lower.*speed/i);
    assert.match(failure.details!, /requires native PASS.*motion_not_verified/);
    assert.equal(failure.action, undefined);
  }
});

test("detailed mixed checks retain pair, axis, measured value, bound and location", () => {
  const detail = "seam seam_0011: FAIL (1 collisions; 1 penetrating; 6 limit violations); Contact or overlap between link_4 and workpiece (segment 8); J2 position 110° outside [-109°, 109°] (waypoint 3); J3 velocity magnitude 40°/s exceeds 30°/s (segment 4, 0.125 s); requires native PASS";
  const failure = planningFailure({ reason: "motion_not_verified", detail, segment_index: 2 });
  assert.match(failure.message, /part placement for collisions/);
  assert.match(failure.message, /requested weld speed.*speed or acceleration limits/);
  assert.match(failure.message, /reported joint positions and limits/);
  assert.equal(failure.details, `${detail} · motion_not_verified · segment 2`);
  assert.doesNotMatch(failure.message, /lower.*speed|reduce.*speed/i);
});

test("rate-only and singular checks do not invent collisions or joint-position violations", () => {
  const rate = planningFailure({ reason: "motion_not_verified", detail: "seam s1: FAIL (0 collisions; 2 limit violations); J3 acceleration magnitude 80°/s² exceeds 60°/s²; travel_speed 14 mm/s exceeds 12 mm/s", segment_index: null });
  assert.match(rate.message, /2 limit-check failures/);
  assert.match(rate.message, /requested weld speed/);
  assert.doesNotMatch(rate.message, /for collisions|joint positions/);
  const singular = planningFailure({ reason: "motion_not_verified", detail: "seam s1: FAIL; singularity at waypoint 2", segment_index: null });
  assert.match(singular.message, /robot posture near the reported singularity/);
  assert.doesNotMatch(singular.message, /speed|joint positions|for collisions/);
});

test("missing verification explains the block without fabricating a cause", () => {
  const failure = planningFailure({ reason: "motion_not_verified", detail: "seam s1: missing continuous trajectory; generate and verify motion", segment_index: null });
  assert.match(failure.message, /no verified motion path/);
  assert.doesNotMatch(failure.message, /collisions|speed|joint positions/);
  assert.match(failure.details!, /missing continuous trajectory.*motion_not_verified/);
});


test("unplanned seams explain that no candidate was found without inventing collision or limit causes", () => {
  for (const error of [{ reason: "seam_not_planned", detail: "seam seam_0011", segment_index: null },
    { dense_error: { reason: "seam_not_planned", detail: "seam_0011", segment_index: null } },
    new Error('Planner refused: {"reason":"seam_not_planned","detail":"seam seam_0011","segment_index":null}'),
    "seam seam_0011 · seam_not_planned"]) {
    const failure = planningFailure(error);
    assert.match(failure.message, /no candidate motion path was found for seam_0011/);
    assert.match(failure.message, /did not report why/);
    assert.doesNotMatch(failure.message, /collision|limit|speed|singular|unreachable/i);
    assert.match(failure.details!, /seam_0011.*seam_not_planned/);
    assert.equal(failure.action, undefined);
  }
});

test("unplanned seams preserve supplied search failures without replacing them with a guessed diagnosis", () => {
  const detail = "seam seam_0042: no continuous candidate sequence at waypoint 17; search exhausted 80 candidates";
  const failure = planningFailure({ reason: "seam_not_planned", detail, segment_index: 2 });
  assert.match(failure.message, /no candidate motion path was found for seam_0042/);
  assert.match(failure.message, /planner's explanation in Technical details/);
  assert.doesNotMatch(failure.message, /did not report why|collision|limit|speed/i);
  assert.equal(failure.details, `${detail} · seam_not_planned · segment 2`);
  const missing = planningFailure({ reason: "seam_not_planned", detail: "", segment_index: null });
  assert.match(missing.message, /for this seam.*did not report why/);
  assert.equal(missing.details, "seam_not_planned");
});

test("native human-readable direction and travel-speed findings get relevant guidance", () => {
  const direction = planningFailure({ reason: "motion_not_verified", detail: "seam s1: FAIL; The robot cannot move along the weld direction at this pose (waypoint 2)", segment_index: null });
  assert.match(direction.message, /robot posture near the reported singularity/);
  const speed = planningFailure({ reason: "motion_not_verified", detail: "seam s1: FAIL; The requested weld travel speed exceeds the available joint speeds at this pose", segment_index: null });
  assert.match(speed.message, /requested weld speed/);
  assert.doesNotMatch(speed.message, /collision|joint positions/);
});
