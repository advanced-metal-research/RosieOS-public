// The Steam Deck pendant's teach-and-plan half, run by Node instead of Qt:
// the same olp-core modules (steamdeck/real/v5/olp-core) make every decision,
// the requests are the ones window.cpp and run_page.cpp send, and the TCP of
// a capture comes from OLP's urdf-loader chain (tests/capture_fk_fixture.mjs,
// the reference the Deck's C++ FK must reproduce). Driven step by step by
// offline-programming/v1/weld_plan_composed_test.go, which owns the machine.
//
// usage, from offline-programming/v1/ui so OLP's node_modules resolve:
//   OLP=<origin> STATE=<project.json> node --experimental-strip-types --no-warnings test/pendant/deck-teach.mjs <command>
// commands:
//   record [joint|linear]  record a new waypoint from the measured pose
//   plan                   POST /weld-plan with planBody(); keep the plan as applyPlan does
//   export                 POST /weld-plan/export with the same body (pack only, timed)
//   reasons                stdin [{code, detail}] -> the Deck's operator text for each
//   home                   append the Deck's HOME event (first row of an empty program)
// The last stdout line is one JSON object with the outcome.
import { readFileSync, writeFileSync, existsSync } from "node:fs";
import { randomUUID } from "node:crypto";
import { Window } from "happy-dom";
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import * as session from "../../../../../steamdeck/real/v5/olp-core/session.ts";
import * as machine from "../../../../../steamdeck/real/v5/olp-core/machine.ts";
import * as run from "../../../../../steamdeck/real/v5/olp-core/run.ts";

const origin = process.env.OLP ?? "";
const statePath = process.env.STATE ?? "";
const cellId = process.env.CELL_ID ?? "sim";
const cellModel = process.env.CELL_MODEL ?? "rosie_1420_v1";
const clientId = process.env.CLIENT_ID ?? "deck-composed";
const dense = "/api/offline-programming/v1/dense-execution";

const window = new Window();
for (const name of ["DOMParser", "Document", "XMLDocument", "Element", "Node"]) globalThis[name] = window[name];

/** OlpClient::send: the Deck's headers; a refusal is `error` (else http_<status>) and `detail`. */
async function call(method, path, body, timeoutMs = 30000) {
  const headers = { Accept: "application/json", "X-Rosie-Client": "rosie-pendant-v5 (composed)", "X-Rosie-Client-Id": clientId };
  if (method === "POST") headers["Content-Type"] = "application/json";
  const started = performance.now();
  const response = await fetch(origin + path, {
    method, headers, body: method === "POST" ? JSON.stringify(body ?? {}) : undefined, signal: AbortSignal.timeout(timeoutMs),
  });
  const text = await response.text();
  let json = null;
  try { json = JSON.parse(text); } catch { /* not JSON */ }
  const ms = performance.now() - started;
  if (response.ok) return { ok: true, status: response.status, body: json, ms };
  const object = json && typeof json === "object" && !Array.isArray(json) ? json : {};
  return {
    ok: false, status: response.status, body: json, ms,
    error: typeof object.error === "string" ? object.error : `http_${response.status}`,
    detail: typeof object.detail === "string" ? object.detail : typeof object.message === "string" ? object.message : "",
  };
}

async function must(method, path, body) {
  const reply = await call(method, path, body);
  if (!reply.ok) throw new Error(`${method} ${path}: ${reply.status} ${reply.error} ${reply.detail}`);
  return reply.body;
}

/** window.cpp: GET /robots, then the open program (the state file) or a new one on the cell's model. */
async function openSession() {
  session.setRobots(await must("GET", "/api/offline-programming/v1/robots"), null);
  if (statePath && existsSync(statePath)) {
    session.openProject(readFileSync(statePath, "utf8"));
    // openProject selects the first node; the Deck's session still has the
    // last record selected, and "new" inserts after the selection.
    if (existsSync(`${statePath}.selected`)) session.select(readFileSync(`${statePath}.selected`, "utf8"));
  } else session.newProject("Deck teach composed", cellModel);
}

function keep() {
  if (!statePath) return;
  writeFileSync(statePath, session.exportProject());
  writeFileSync(`${statePath}.selected`, session.view().selected ?? "");
}

/** The Deck's poll: capabilities once, then statuses until OLP's teach rules admit a capture. */
async function teachObservation(limitMs) {
  machine.observeCapabilities(await must("GET", `${dense}/capabilities`));
  const deadline = Date.now() + limitMs;
  let last = "no status";
  for (;;) {
    const reply = await call("GET", `${dense}/status`);
    if (reply.ok) {
      machine.observeStatus(reply.body, false);
      try {
        return machine.teachObservation({ selected: true, busy: false, statusAgeMs: 0, cellModel, cellId });
      } catch (error) {
        last = error.message;
      }
    } else {
      last = `${reply.error} ${reply.detail}`;
    }
    if (Date.now() > deadline) {
      const axes = (machine.machineStatus()?.rtCore?.axes ?? []).map((axis, index) =>
        `J${index + 1} home ${axis.homeValid} coordinate ${axis.coordinateValid} calibration ${axis.calibrationValid} ${axis.positionState} v ${axis.velocity}`);
      throw new Error(`Record blocked: ${last} (${axes.join("; ")})`);
    }
    await new Promise((resolve) => setTimeout(resolve, 100));
  }
}

/** capture_fk_fixture.mjs: tool frame pose from the capture model's URDF, xyz m and URDF rpy rad. */
function forwardKinematics(urdf, frameName, pose) {
  const loader = new URDFLoader();
  loader.parseVisual = false; loader.parseCollision = false;
  // happy-dom's DOMParser loses the document behind an XML declaration that
  // names an encoding (the 1430 cell's `<?xml version='1.0' encoding='utf-8'?>`);
  // the declaration carries no geometry.
  const robot = loader.parse(urdf.replace(/^<\?xml[^>]*\?>\s*/, ""));
  pose.jointOrder.forEach((name, index) => {
    const joint = robot.joints[name];
    if (!joint) return;
    joint.ignoreLimits = true;
    joint.setJointValue(pose.jointsRad[index]);
  });
  const frame = robot.links[frameName];
  if (!frame) throw new Error(`the capture model has no ${frameName}`);
  frame.updateWorldMatrix(true, false);
  const position = new THREE.Vector3().setFromMatrixPosition(frame.matrixWorld);
  const rotation = new THREE.Euler().setFromRotationMatrix(frame.matrixWorld, "ZYX");
  return [position.x, position.y, position.z, rotation.x, rotation.y, rotation.z];
}

/** Window::record("new"), then the Move type cell when a linear move is asked for. */
async function record(motion) {
  await openSession();
  const measured = await teachObservation(Number(process.env.TEACH_WAIT_MS ?? 10000));
  const start = session.captureStart("new", measured);
  let calibration = null;
  if (start.machineBound) {
    calibration = machine.fetchedCellSettings(true, 0, await must("GET", `${dense}/cell`));
  }
  const model = await must("POST", "/api/offline-programming/v1/robots/capture-model", {
    model_id: start.modelId, robot_description_sha256: start.description,
    machine_planning_calibration_base64: calibration?.machinePlanningCalibrationBase64 ?? "",
  });
  const frame = session.captureFrame(start.modelId);
  const tcp = forwardKinematics(model.urdf, frame.frame, measured.pose);
  const id = session.captureRecord("new", measured, start.revision, start.description, calibration, model.identity, tcp, randomUUID());
  if (motion && motion !== "joint") session.edit(id, "motion", motion);
  keep();
  const node = JSON.parse(session.exportProject()).project.nodes.find((candidate) => candidate.id === id);
  return { ok: true, id, label: node.label, motion: node.motion, joints: node.target.joint_values_rad, jointNames: node.target.joint_names, tcp, machineBound: start.machineBound };
}

/** RunPage::plan: planBody, POST /weld-plan, planFailure on a refusal, applyPlan on success. */
async function plan() {
  await openSession();
  // The Run page's Plan speed, when asked for (session.setSpeedScale).
  const speed = Number(process.env.PLAN_SPEED ?? 100);
  if (Math.abs(session.view().speedScale * 100 - speed) > 1e-9) {
    session.setSpeedScale(speed);
    keep();
  }
  const revision = session.view().revision;
  let body;
  try {
    body = await run.planBody();
  } catch (error) {
    return { ok: false, stage: "planBody", message: `Planning failed: ${error.message}` };
  }
  const timeoutMs = Number(process.env.PLAN_TIMEOUT_MS ?? 1800000);
  const reply = await call("POST", "/api/offline-programming/v1/weld-plan", body, timeoutMs);
  if (!reply.ok) {
    const code = reply.body?.code ?? "";
    // run.ts planFailure took (error, dense_error) up to 2b56c673 and
    // (code, error, dense_error) from 74b01589.
    const failure = run.planFailure.length >= 3 ? run.planFailure(code, reply.error, reply.body?.dense_error ?? null)
      : run.planFailure(reply.error, reply.body?.dense_error ?? null);
    return { ok: false, stage: "weld-plan", status: reply.status, code, error: reply.error, ms: reply.ms,
      message: `Planning failed: ${failure.message}`, verbatim: failure.verbatim ?? "", details: failure.details ?? "" };
  }
  const digest = run.applyPlan(reply.body, revision);
  keep();
  if (process.env.BLOB_OUT && reply.body.dense_blob_base64) writeFileSync(process.env.BLOB_OUT, Buffer.from(reply.body.dense_blob_base64, "base64"));
  return {
    ok: true, ms: reply.ms, digest, load: run.loadBody(), weldplanSha256: reply.body.weldplan_sha256,
    dense: { samples: reply.body.dense?.total_sample_count, durationS: reply.body.dense?.total_duration_s, segments: reply.body.dense?.segments?.length },
    blobBytes: reply.body.dense_blob_base64 ? Buffer.from(reply.body.dense_blob_base64, "base64").length : 0,
  };
}

/** The same body packed only (POST /weld-plan/export): the seam worker's share of a plan. */
async function exportRequest() {
  await openSession();
  const headers = { "X-Rosie-Client": "rosie-pendant-v5 (composed)", "X-Rosie-Client-Id": clientId, "Content-Type": "application/json" };
  const started = performance.now();
  const response = await fetch(`${origin}/api/offline-programming/v1/weld-plan/export`, {
    method: "POST", headers, body: JSON.stringify(await run.planBody()), signal: AbortSignal.timeout(300000),
  });
  const bytes = Buffer.from(await response.arrayBuffer());
  const ms = performance.now() - started;
  if (!response.ok) return { ok: false, status: response.status, ms, error: bytes.toString("utf8").slice(0, 600) };
  // The packed request, for replaying the planner alone (cold and warm timings).
  if (process.env.EXPORT_OUT) writeFileSync(process.env.EXPORT_OUT, bytes);
  return { ok: true, ms, bytes: bytes.length, sha256: response.headers.get("x-weldplan-sha256") };
}

/** The Deck's HOME event (session.addEvent): always appended, and selected,
 *  so waypoints recorded next follow it. Added to an empty program it is the
 *  first row, as on the 1430 cell's Deck at 15:45 2026-10-11 (HOME, then P1-P4). */
async function addHome() {
  await openSession();
  const id = session.addEvent("home");
  keep();
  return { ok: true, id, ops: session.view().rows.map((row) => row.op) };
}

/** Machine::text: OLP's operator words for a refused /load, /play or /arm. */
function reasons() {
  const input = JSON.parse(readFileSync(0, "utf8"));
  return { ok: true, texts: input.map(({ code, detail }) => machine.reasonText(code, detail ?? "")) };
}

const [command, argument] = process.argv.slice(2);
let outcome;
try {
  if (command === "record") outcome = await record(argument ?? "joint");
  else if (command === "plan") outcome = await plan();
  else if (command === "export") outcome = await exportRequest();
  else if (command === "reasons") outcome = reasons();
  else if (command === "home") outcome = await addHome();
  else throw new Error(`unknown command ${command}`);
} catch (error) {
  outcome = { ok: false, message: error.message };
}
process.stdout.write(`${JSON.stringify(outcome)}\n`);
