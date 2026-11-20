// Teaching from the robot's measured pose and pulling the cell's settings,
// against the Steam Deck's recorded cell A replies
// (steamdeck/real/v5/tests/fixtures/example-live). As in the Deck's teach_check,
// the fake advances rt-core's clock on every status so OLP's stillness window
// can elapse; the pose itself never changes.
import assert from "node:assert/strict";
import { once } from "node:events";
import { mkdtempSync, readFileSync } from "node:fs";
import http from "node:http";
import { tmpdir } from "node:os";
import path from "node:path";
import { after, before, test } from "node:test";

import { connect, REPO_ROOT, textOf, urdfTcpMmOf } from "./helpers.js";

const live = path.join(REPO_ROOT, "steamdeck", "real", "v5", "tests", "fixtures", "example-live");
const read = (name: string) => JSON.parse(readFileSync(path.join(live, name), "utf8"));
const status = read("status.json");
const captureModel = read("capture-model.json");
// The capture predates later rt-core contract changes, so the fake answers
// with this checkout's capabilities digest, as a cell running this commit
// would; a cell on another contract is refused (src/program.ts observeMachine).
const generated = readFileSync(path.join(REPO_ROOT, "rt-core", "clients", "ts", "rt_control_api_generated.ts"), "utf8");
const capabilitiesDigest = /CapabilitiesDigest = "([0-9a-f]{64})"/.exec(generated)![1]!;
let servedDigest = capabilitiesDigest;

function capabilitiesNow() {
  const body = read("caps.json");
  body.rt_core.description.capabilities_digest = servedDigest;
  return body;
}
const started = process.hrtime.bigint();
let olpUrl = "";
let server: http.Server;
const posts: { route: string; body: any }[] = [];

function statusNow() {
  const body = structuredClone(status);
  const ns = Number(status.rt_core.status.time_ns) + Number(process.hrtime.bigint() - started);
  body.rt_core.status.time_ns = ns;
  for (const axis of body.rt_core.status.axes) axis.time_ns = ns;
  return body;
}

before(async () => {
  server = http.createServer((request, response) => {
    const chunks: Buffer[] = [];
    request.on("data", (chunk: Buffer) => chunks.push(chunk));
    request.on("end", () => {
      const route = (request.url ?? "").replace("/api/offline-programming/v1", "");
      const raw = Buffer.concat(chunks).toString("utf8");
      if (request.method === "POST") posts.push({ route, body: raw ? JSON.parse(raw) : undefined });
      const send = (value: unknown) => {
        response.writeHead(200, { "Content-Type": "application/json" });
        response.end(JSON.stringify(value));
      };
      if (route === "/robots") return send(read("robots.json"));
      if (route === "/dense-execution/capabilities") return send(capabilitiesNow());
      if (route === "/dense-execution/status") return send(statusNow());
      if (route === "/dense-execution/cell") return send(read("cell.json"));
      if (route === "/robots/capture-model") return send(captureModel);
      response.writeHead(404, { "Content-Type": "application/json" });
      response.end(JSON.stringify({ error: "unknown_route", detail: route }));
    });
  });
  server.listen(0, "127.0.0.1");
  await once(server, "listening");
  olpUrl = `http://127.0.0.1:${(server.address() as { port: number }).port}`;
});

after(() => new Promise<void>((resolve) => server.close(() => resolve())));

test("pull the cell's settings, then teach from cellA's measured pose: the machine's joints, its calibration, OLP's TCP", async () => {
  const session = await connect({ ROSIE_OLP_URL: olpUrl, ROSIE_MCP_STATE_DIR: mkdtempSync(path.join(tmpdir(), "rosieos-mcp-teach-")) });
  try {
    const call = (name: string, args: Record<string, unknown>) => session.client.callTool({ name, arguments: { ...args, response_format: "json" } });
    const created = await call("rosie_program_new", { name: "cellA teach", model: "rosie_1420_v1" });
    assert.notEqual(created.isError, true, textOf(created));

    const pulled = await call("rosie_program_robot_settings", { action: "pull" });
    assert.notEqual(pulled.isError, true, textOf(pulled));
    const cell = (pulled.structuredContent as { cell: { cell: string; machinePlanningCalibrationSha256: string } }).cell;
    assert.equal(cell.cell, "example-arm-1420");
    assert.equal(cell.machinePlanningCalibrationSha256, read("cell.json").robot_cell.machine_planning_calibration_sha256);

    const recorded = await call("rosie_program_record_robot_pose", { motion: "linear", speed_mm_s: 100 });
    assert.notEqual(recorded.isError, true, textOf(recorded));
    const row = (recorded.structuredContent as { rows: { id: string; op: string; cells: Record<string, number | string> }[] }).rows.find((r) => r.op === "move")!;
    assert.deepEqual([row.cells.motion, row.cells.speed_mm_s], ["linear", 100]);

    const capture = posts.findLast((p) => p.route === "/robots/capture-model")!.body;
    assert.equal(capture.machine_planning_calibration_base64, read("cell.json").machine_planning_calibration_base64, "the cell's own calibration, as fetched");
    const fields = (await call("rosie_program_get_row", { row_id: row.id })).structuredContent as { groups: { fields: { path: string; value: unknown }[] }[] };
    const value = (field: string) => fields.groups.flatMap((g) => g.fields).find((f) => f.path === field)?.value;
    assert.equal(value("capture.source"), "machine");
    const measured = status.rt_core.status.axes.slice(0, 6).map((axis: { logical_position: number }) => axis.logical_position);
    measured.forEach((rad: number, i: number) => assert.ok(Math.abs(Number(value(`target.joint_values_rad.${i}`)) - (rad * 180) / Math.PI) < 1e-9, `J${i + 1}`));
    const expected = urdfTcpMmOf(captureModel.urdf, Object.fromEntries(measured.map((rad: number, i: number) => [`J${i + 1}`, rad])));
    for (let k = 0; k < 3; k++) assert.ok(Math.abs(Number(row.cells[`target.xyz_m.${k}`]) - expected[k]!) < 1e-6, `TCP ${"xyz"[k]} ${row.cells[`target.xyz_m.${k}`]} vs ${expected[k]} mm`);
  } finally {
    await session.close();
  }
});

test("recording from the robot is refused while the cell's session holds a program loaded, as the browser's editing lock", async () => {
  const original = status.session;
  status.session = { ...original, state: "loaded", trajectory_digest: `sha256:${"c".repeat(64)}`, lease: { held: true, expires_in_ms: 4000 } };
  const session = await connect({ ROSIE_OLP_URL: olpUrl, ROSIE_MCP_STATE_DIR: mkdtempSync(path.join(tmpdir(), "rosieos-mcp-teach-")) });
  try {
    await session.client.callTool({ name: "rosie_program_new", arguments: { name: "locked", model: "rosie_1420_v1" } });
    const before = posts.filter((p) => p.route === "/robots/capture-model").length;
    const refused = await session.client.callTool({ name: "rosie_program_record_robot_pose", arguments: { response_format: "json" } });
    assert.equal(refused.isError, true);
    // main.ts editingLockReason around densePanel.lockReason, word for word.
    assert.match(textOf(refused), /OLP will not record from the robot: Editing is locked while the joint daemon is loaded on this program \(STOP to release\)\./);
    assert.equal(posts.filter((p) => p.route === "/robots/capture-model").length, before, "no capture was asked for");
    const view = await session.client.callTool({ name: "rosie_program_view", arguments: { response_format: "json" } });
    assert.equal((view.structuredContent as { rows: unknown[] }).rows.length, 0);

    for (const state of ["staged", "playing", "paused"]) {
      status.session = { ...original, state };
      const again = await session.client.callTool({ name: "rosie_program_record_robot_pose", arguments: {} });
      assert.match(textOf(again), new RegExp(`joint daemon is ${state} on this program`));
    }
  } finally {
    status.session = original;
    await session.close();
  }
});

// The guard itself, independent of how old the recording is: a cell whose
// capabilities digest differs from this checkout's by one character is
// refused before any capture, and nothing is recorded.
test("a cell on another rt-core contract is refused, and nothing is recorded", async () => {
  servedDigest = (capabilitiesDigest[0] === "0" ? "1" : "0") + capabilitiesDigest.slice(1);
  const session = await connect({ ROSIE_OLP_URL: olpUrl, ROSIE_MCP_STATE_DIR: mkdtempSync(path.join(tmpdir(), "rosieos-mcp-teach-")) });
  try {
    await session.client.callTool({ name: "rosie_program_new", arguments: { name: "mismatched", model: "rosie_1420_v1" } });
    const before = posts.filter((p) => p.route === "/robots/capture-model").length;
    const refused = await session.client.callTool({ name: "rosie_program_record_robot_pose", arguments: { response_format: "json" } });
    assert.equal(refused.isError, true);
    assert.match(textOf(refused), /rt_core description identity is incompatible/);
    assert.equal(posts.filter((p) => p.route === "/robots/capture-model").length, before, "no capture was asked for");
    const view = await session.client.callTool({ name: "rosie_program_view", arguments: { response_format: "json" } });
    assert.equal((view.structuredContent as { rows: unknown[] }).rows.length, 0);
  } finally {
    servedDigest = capabilitiesDigest;
    await session.close();
  }
});

test("a robot that is not verified and still is refused in OLP's words, and nothing is recorded", async () => {
  const moving = structuredClone(status);
  for (const axis of moving.rt_core.status.axes) axis.home_valid = false;
  const original = status.rt_core.status.axes.map((axis: { home_valid: boolean }) => axis.home_valid);
  status.rt_core.status.axes.forEach((axis: { home_valid: boolean }) => { axis.home_valid = false; });
  const session = await connect({ ROSIE_OLP_URL: olpUrl, ROSIE_MCP_STATE_DIR: mkdtempSync(path.join(tmpdir(), "rosieos-mcp-teach-")) });
  try {
    await session.client.callTool({ name: "rosie_program_new", arguments: { name: "refused", model: "rosie_1420_v1" } });
    const refused = await session.client.callTool({ name: "rosie_program_record_robot_pose", arguments: { response_format: "json" } });
    assert.equal(refused.isError, true);
    assert.match(textOf(refused), /OLP will not record from the robot: .*(verified|stationary)/);
    const view = await session.client.callTool({ name: "rosie_program_view", arguments: { response_format: "json" } });
    assert.equal((view.structuredContent as { rows: unknown[] }).rows.length, 0);
  } finally {
    status.rt_core.status.axes.forEach((axis: { home_valid: boolean }, i: number) => { axis.home_valid = original[i]; });
    await session.close();
  }
});
