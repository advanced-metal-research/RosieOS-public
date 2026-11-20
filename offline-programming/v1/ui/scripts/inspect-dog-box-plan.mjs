import { createDogBoxProject } from "../src/examples/dog-box.ts";
import { buildLocalPlanRequest, localPlanRequestWire } from "../src/simulation/local-plan.ts";

const project = createDogBoxProject("2026-07-31T13:30:00.000Z");
project.target = {
  mode: "local_simulator",
  profileId: "local-simulator",
  manifestRef: null,
};
const request = buildLocalPlanRequest(project, {
  requestId: `inspect-dog-box-${Date.now()}`,
  submittedAt: new Date().toISOString(),
});
const startedAt = performance.now();
const response = await fetch("http://127.0.0.1:8796/api/offline-programming/v1/local-simulator/plans", {
  method: "POST",
  headers: { "Content-Type": "application/json" },
  body: JSON.stringify(localPlanRequestWire(request)),
});
const body = await response.json();
const wallTimeS = (performance.now() - startedAt) / 1000;
const result = body.result ?? body;
const collisionIndexes = new Set(
  (result.evidence?.collision?.points ?? []).map((point) => point.waypoint_index),
);
const nearby = (result.trajectory ?? []).filter((point) =>
  [...collisionIndexes].some((index) => Math.abs(point.waypoint_index - index) <= 2)
);
console.log(JSON.stringify({
  status: response.status,
  planState: result.evidence?.state,
  wallTimeS,
  timing: result.evidence?.timing,
  reportReasons: result.report_reasons,
  collisions: result.evidence?.collision,
  nearbyTrajectory: nearby,
}, null, 2));
