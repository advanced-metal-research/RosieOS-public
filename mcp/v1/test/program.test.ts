// Programs through the MCP client, edited by OLP's own engine (the bundled
// core), against a fake OLP at the HTTP boundary: robots, capture model, the
// CAD and seam workers, the planner route and the cell program store.
import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { once } from "node:events";
import { mkdtempSync, readFileSync, writeFileSync } from "node:fs";
import http from "node:http";
import { tmpdir } from "node:os";
import path from "node:path";
import { after, before, test } from "node:test";

import { decode } from "rosie-cli/lib/rdt.js";
import { connect, REPO_ROOT, textOf, urdfTcpMmOf } from "./helpers.js";

const MODEL = "rosie_1420_v1";
const DESCRIPTION = `sha256:${"2".repeat(64)}`;
const urdf = readFileSync(path.join(REPO_ROOT, "robot_description", "robots", MODEL, "robot.urdf"), "utf8");
const fixture = readFileSync(path.join(REPO_ROOT, "motion-server", "joint-trajectory", "v1", "testdata", "dense_joint_trajectory_reference.rdt"));
const state = mkdtempSync(path.join(tmpdir(), "rosieos-mcp-program-"));
const stepFile = path.join(state, "plate.step");
writeFileSync(stepFile, "ISO-10303-21;\nHEADER;ENDSEC;DATA;ENDSEC;END-ISO-10303-21;\n");
const stepBytes = readFileSync(stepFile);
const fingerprint = createHash("sha256").update(stepBytes).digest("hex");

/** The reference trajectory's header, as the planner's `dense` summary reports it. */
function denseSummary() {
  const header = JSON.parse(fixture.subarray(4, 4 + fixture.readUInt32BE(0)).toString("utf8"));
  return {
    schema: header.schema, trajectory_digest: header.trajectory_digest, request_sha256: "f".repeat(64), plan_id: header.plan_id,
    program_id: header.program_id, program_digest: header.program_digest, manifest_revision: header.manifest_revision,
    plan_revision: header.plan_revision, created_at: header.created_at, dt_s: header.sampling.dt_s,
    total_sample_count: header.sampling.total_sample_count, total_duration_s: header.sampling.total_duration_s, bytes: fixture.byteLength,
    segments: header.segments.map((s: Record<string, unknown>) => ({ index: s.index, kind: s.kind, source_id: s.source_id, sample_count: s.sample_count, duration_s: s.duration_s })),
    robot_cell: null,
  };
}

interface Seen { method: string; route: string; body: any; headers: http.IncomingHttpHeaders }
const seen: Seen[] = [];
let storeRevision = 0;
let olpUrl = "";
let server: http.Server;

before(async () => {
  server = http.createServer((request, response) => {
    const chunks: Buffer[] = [];
    request.on("data", (chunk: Buffer) => chunks.push(chunk));
    request.on("end", () => {
      const raw = Buffer.concat(chunks).toString("utf8");
      const route = (request.url ?? "").replace("/api/offline-programming/v1", "");
      const body = raw ? JSON.parse(raw) : undefined;
      seen.push({ method: request.method ?? "", route, body, headers: request.headers });
      const send = (code: number, value: unknown) => {
        response.writeHead(code, { "Content-Type": "application/json" });
        response.end(JSON.stringify(value));
      };
      if (route === "/robots") {
        return send(200, { problems: [], robots: [{ model_id: MODEL, robot_description_sha256: DESCRIPTION,
          frames: { base: "world", tool: "tool0", work: "world", work_world_m: [0, 0, 0], arm_base_world_m: [0.4127, 0.4, 0.893] },
          axes: { driven: ["J1", "J2", "J3", "J4", "J5", "J6"] }, reset_pose: { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 } }] });
      }
      if (route === "/dense-execution/status") {
        return send(200, { available: true, target: { selection_generation: "3", cell_id: "sim", model_id: MODEL, backend: "simulation", simulation: true, host: "localhost" }, armed: false, session: null });
      }
      if (route === "/robots/capture-model") {
        return send(200, { urdf, identity: { model_id: MODEL, robot_description_sha256: body.robot_description_sha256, machine_planning_calibration_sha256: `sha256:${"e".repeat(64)}`, machine_planning_calibration_source: "empty" } });
      }
      if (route === "/cadquery/topology") {
        const samples = [[0, 0, 0], [100, 0, 0], [200, 0, 0]];
        return send(200, { schema_version: 1, unit: "mm", tool: "cadquery", model_id: "plate", filename: body.filename, fingerprint, sample_count: 64,
          parts: [{ id: "p1", edge_count: 2 }],
          edges: [{ id: "e1", part_id: "p1", geom_type: "LINE", samples, length_mm: 200, closed: false, is_seam_candidate: true },
            { id: "e2", part_id: "p1", geom_type: "LINE", samples: [[0, 0, 0], [0, 80, 50]], length_mm: 94, closed: false }] });
      }
      if (route === "/seam") {
        if (body.operation === "detect_joints") {
          return send(200, { joints: { weld_joints: [{ id: "weld_joint_0001", type: "tee", weld_symbol: "fillet", weld_line: { segments: [{ source_edge_id: "e1", degree: 1, control_points_xyz_m: [[0, 0, 0], [0.2, 0, 0]], knots_u: [0, 0, 1, 1] }], closed: false, length_m: 0.2 }, bisector_rail: { curve: {} } }],
            seams: [{ id: "seam_0001", label: "fillet 200 mm", weld_joint_ref: "weld_joint_0001", span: { start_s: 0, end_s: 1 }, direction: "forward", reference: { mode: "rail_curve" }, process: {},
              orientation: { work_angle_deg: { kind: "constant", value: 45 }, travel_angle_deg: { kind: "constant", value: 10 } }, curve: { from_weld_joint: true } }] } });
        }
        if (body.operation === "plan_torch_path") {
          const clear = body.direction === "reverse" ? 0.95 : 0.8;
          return send(200, { direction: body.direction, plan: { work_angle_deg: { kind: "constant", value: 42 }, travel_angle_deg: { kind: "constant", value: 12 }, direction: body.direction,
            direction_changed: body.direction === "reverse", standoff_mm: 17, standoff_departure_mm: 0, clear_fraction: clear, authored_clear_fraction: 0.7, seam_length_mm: 200,
            knot_fractions: [0, 1], work_path_deg: [42, 42], travel_path_deg: [12, 12] } });
        }
        if (body.operation === "check_torch_fits") return send(200, { clearance: { seam_length_m: 0.2, clear_runs: [[0, 1]], blocked_spans: [] } });
      }
      if (route === "/weld-plan") {
        return send(200, { result: { seams: [] }, weldplan_sha256: "a".repeat(64), program_digest: body.program_digest, dense: denseSummary(), dense_blob_base64: fixture.toString("base64") });
      }
      if (route === "/dense-execution/load") return send(200, { ok: true, session_id: "sess-load", trajectory: { trajectory_digest: body.trajectory_digest }, status: null });
      const save = /^\/cell-programs\/programs\/([^/]+)\/save$/.exec(route);
      if (save) {
        if (body.base_revision !== storeRevision) return send(409, { error: "program_conflict", detail: `the cell has revision ${storeRevision}, saved by steamdeck-operator` });
        storeRevision += 1;
        return send(200, { ok: true, program: { program_id: save[1], revision: storeRevision }, replayed: false });
      }
      send(404, { error: "unknown_route", detail: route });
    });
  });
  server.listen(0, "127.0.0.1");
  await once(server, "listening");
  olpUrl = `http://127.0.0.1:${(server.address() as { port: number }).port}`;
});

after(() => new Promise<void>((resolve) => server.close(() => resolve())));

const urdfTcpMm = (jointsRad: Record<string, number>) => urdfTcpMmOf(urdf, jointsRad);

test("a waypoint recorded from joint angles carries OLP's TCP for them, in absolute millimetres", async () => {
  const session = await connect({ ROSIE_OLP_URL: olpUrl, ROSIE_MCP_STATE_DIR: state });
  try {
    const created = await session.client.callTool({ name: "rosie_program_new", arguments: { name: "teach", response_format: "json" } });
    assert.notEqual(created.isError, true, textOf(created));
    assert.equal((created.structuredContent as { model: string }).model, MODEL);
    const deg = [30, -20, 40, 10, 30, -15];
    const recorded = await session.client.callTool({ name: "rosie_program_add_waypoint", arguments: { joints_deg: deg, response_format: "json" } });
    assert.notEqual(recorded.isError, true, textOf(recorded));
    const rows = (recorded.structuredContent as { rows: { id: string; op: string; cells: Record<string, number> }[] }).rows;
    assert.equal(rows.length, 1);
    const expected = urdfTcpMm(Object.fromEntries(deg.map((v, i) => [`J${i + 1}`, (v * Math.PI) / 180])));
    for (let i = 0; i < 3; i++) assert.ok(Math.abs(rows[0]!.cells[`target.xyz_m.${i}`]! - expected[i]!) < 1e-6, `TCP ${"xyz"[i]} ${rows[0]!.cells[`target.xyz_m.${i}`]} vs ${expected[i]} mm`);
    const fields = await session.client.callTool({ name: "rosie_program_get_row", arguments: { row_id: rows[0]!.id, response_format: "json" } });
    const values = (fields.structuredContent as { groups: { fields: { path: string; value: unknown }[] }[] }).groups.flatMap((g) => g.fields);
    assert.ok(Math.abs(Number(values.find((f) => f.path === "target.joint_values_rad.0")!.value) - 30) < 1e-9);
    assert.equal(values.find((f) => f.path === "capture.source")!.value, "preview");
    const capture = seen.findLast((s) => s.route === "/robots/capture-model")!;
    assert.deepEqual(capture.body, { model_id: MODEL, robot_description_sha256: DESCRIPTION, machine_planning_calibration_base64: "" });
    assert.match(String(capture.headers["x-rosie-client-id"]), /^mcp-[0-9a-f]{16}$/);
  } finally {
    await session.close();
  }
});

test("edits go through OLP's field specs: units, refusals, order and undo", async () => {
  const session = await connect({ ROSIE_OLP_URL: olpUrl, ROSIE_MCP_STATE_DIR: state });
  try {
    const call = (name: string, args: Record<string, unknown>) => session.client.callTool({ name, arguments: { ...args, response_format: "json" } });
    type View = { rows: { id: string; op: string; label: string; cells: Record<string, number | string> }[]; canUndo: boolean };
    await call("rosie_program_new", { name: "edits" });
    await call("rosie_program_add_waypoint", { joints_deg: [0, -10, 20, 0, 0, 0], motion: "linear", speed_mm_s: 120 });
    const dwell = ((await call("rosie_program_add_event", { kind: "dwell" })).structuredContent as View).rows.find((r) => r.op === "dwell")!;
    await call("rosie_program_add_event", { kind: "home" });
    const edited = await call("rosie_program_edit_row", { row_id: dwell.id, edits: [{ path: "duration_s", value: 2.5 }, { path: "label", value: "settle" }] });
    const after = (edited.structuredContent as View).rows.find((r) => r.id === dwell.id)!;
    assert.deepEqual([after.cells.duration_s, after.label], [2.5, "settle"]);
    const move = (edited.structuredContent as View).rows.find((r) => r.op === "move")!;
    assert.deepEqual([move.cells.motion, move.cells.speed_mm_s], ["linear", 120]);
    const refused = await call("rosie_program_edit_row", { row_id: dwell.id, edits: [{ path: "duration_s", value: -1 }] });
    assert.equal(refused.isError, true);
    assert.match(textOf(refused), /duration_s refused: .*nothing applied/);
    const wrongField = await call("rosie_program_edit_row", { row_id: dwell.id, edits: [{ path: "speed_mm_s", value: 10 }] });
    assert.match(textOf(wrongField), /not editable on a dwell/);
    const undone = await call("rosie_program_undo", { steps: 1 });
    assert.equal((undone.structuredContent as View).rows.find((r) => r.id === dwell.id)!.label !== "settle", true);
    const removed = await call("rosie_program_remove_row", { row_id: dwell.id });
    assert.equal((removed.structuredContent as View).rows.some((r) => r.id === dwell.id), false);
  } finally {
    await session.close();
  }
});

test("STEP to weld to plan to Load: the part travels with the plan and Load takes the kept plan's identity", async () => {
  const session = await connect({ ROSIE_OLP_URL: olpUrl, ROSIE_MCP_STATE_DIR: state, ROSIE_MCP_ROBOT_ACTIONS: "simulation" });
  try {
    const call = (name: string, args: Record<string, unknown>) => session.client.callTool({ name, arguments: { ...args, response_format: "json" } });
    await call("rosie_program_new", { name: "plate" });
    const imported = await call("rosie_program_import_step", { path: stepFile });
    assert.notEqual(imported.isError, true, textOf(imported));
    const part = (imported.structuredContent as { name: string; part: { cad: { fingerprint: string }; placement: { offsetFromWorkFrameMm: number[] } } });
    assert.equal(part.part.cad.fingerprint, fingerprint);
    assert.equal(part.name, "plate", "the first part names the program after its file");
    assert.deepEqual(part.part.placement.offsetFromWorkFrameMm.map((v) => Math.round(v * 1e6) / 1e6), [0, 0, 0], "import centres the part on the work frame");
    const seams = await call("rosie_program_list_seams", {});
    assert.deepEqual((seams.structuredContent as { seams: { seam_id: string }[] }).seams.map((s) => s.seam_id), ["seam_0001"]);
    const searched = await call("rosie_program_search_seam", { seam_id: "seam_0001" });
    assert.deepEqual([(searched.structuredContent as { state: string }).state, (searched.structuredContent as { direction: string }).direction], ["clear", "reverse"]);
    const welded = await call("rosie_program_add_weld", { seam_id: "seam_0001" });
    assert.notEqual(welded.isError, true, textOf(welded));
    const ops = (welded.structuredContent as { rows: { id: string; op: string }[] }).rows.map((r) => `${r.op}:${r.id}`);
    assert.deepEqual(ops, ["approach:rails-seam_0001-approach", "weld:rails-seam_0001", "retract:rails-seam_0001-retract"]);
    const placed = await call("rosie_program_place_part", { offset_mm: [100, -50, 0], rotation_deg: [0, 0, 90] });
    assert.deepEqual((placed.structuredContent as { part: { placement: { offsetFromWorkFrameMm: number[] } } }).part.placement.offsetFromWorkFrameMm.map((v) => Math.round(v * 1e6) / 1e6), [100, -50, 0]);

    const planned = await call("rosie_program_plan", { from_program_start: true });
    assert.notEqual(planned.isError, true, textOf(planned));
    const request = seen.findLast((s) => s.route === "/weld-plan")!.body;
    assert.equal(request.step_base64, stepBytes.toString("base64"));
    assert.deepEqual([request.workpiece_absent, request.plan_from_program_start, request.cell_id], [false, true, MODEL]);
    const weld = request.program.nodes.find((n: { op: string }) => n.op === "weld");
    assert.equal(weld.extensions.rails.sha256, fingerprint);
    assert.deepEqual(weld.geometry.orientation.work_angle_deg, { kind: "constant", value: 42 });
    assert.equal(weld.geometry.direction, "reverse");
    const kept = (planned.structuredContent as { trajectory_file: string }).trajectory_file;
    assert.deepEqual(readFileSync(kept), fixture);

    const preview = await call("rosie_program_preview_path", { max_points: 5 });
    assert.notEqual(preview.isError, true, textOf(preview));
    const previewed = (preview.structuredContent as { segments: { startMm: number[]; endMm: number[]; samples: number; pointsMm: number[][] }[] }).segments;
    const decoded = decode(fixture).segments;
    assert.equal(previewed.length, decoded.length);
    decoded.forEach((segment, i) => {
      for (const [at, mm] of [[0, previewed[i]!.startMm], [segment.samples.length - 1, previewed[i]!.endMm]] as const) {
        const q = segment.samples[at]!.q;
        const expected = urdfTcpMm(Object.fromEntries(q.slice(0, 6).map((v, k) => [`J${k + 1}`, v])));
        for (let k = 0; k < 3; k++) assert.ok(Math.abs(mm[k]! - expected[k]!) < 1e-6, `segment ${i} sample ${at} ${"xyz"[k]}: ${mm[k]} vs ${expected[k]} mm`);
      }
      assert.equal(previewed[i]!.samples, segment.samples.length);
      assert.ok(previewed[i]!.pointsMm.length <= 6);
    });

    const loaded = await session.client.callTool({ name: "rosie_olp_load", arguments: { from_open_program: true } });
    assert.notEqual(loaded.isError, true, textOf(loaded));
    const load = seen.findLast((s) => s.route === "/dense-execution/load")!;
    const summary = denseSummary();
    assert.deepEqual(
      [load.body.trajectory_digest, load.body.plan_id, load.body.program_id, load.body.program_digest, load.body.manifest_revision, load.body.plan_revision],
      [summary.trajectory_digest, summary.plan_id, summary.program_id, summary.program_digest, summary.manifest_revision, summary.plan_revision],
    );
    assert.equal(load.body.dry_run, true, "a simulation-mode program loads as a dry run, as the browser seeds it");
    assert.equal(load.headers["x-rt-target-generation"], "3");

    await call("rosie_program_edit_row", { row_id: "rails-seam_0001", edits: [{ path: "label", value: "edited" }] });
    const stale = await session.client.callTool({ name: "rosie_olp_load", arguments: { from_open_program: true } });
    // OLP's reviseProject drops the kept plan on a planning change: an edited program has no plan to load.
    assert.equal(stale.isError, true);
    assert.match(textOf(stale), /has no plan: plan it first/);
    assert.equal(seen.filter((s) => s.route === "/dense-execution/load").length, 1, "the refused Load sent nothing");
  } finally {
    await session.close();
  }
});

test("the cell store save is revision-checked and a conflict never overwrites", async () => {
  const session = await connect({ ROSIE_OLP_URL: olpUrl, ROSIE_MCP_STATE_DIR: state });
  try {
    const call = (name: string, args: Record<string, unknown>) => session.client.callTool({ name, arguments: { ...args, response_format: "json" } });
    await call("rosie_program_new", { name: "stored" });
    storeRevision = 0;
    const first = await call("rosie_program_save", { target: "cell" });
    assert.notEqual(first.isError, true, textOf(first));
    assert.equal(seen.findLast((s) => s.route.endsWith("/save"))!.body.base_revision, 0);
    await call("rosie_program_add_event", { kind: "home" });
    await call("rosie_program_save", { target: "cell" });
    assert.equal(seen.findLast((s) => s.route.endsWith("/save"))!.body.base_revision, 1);
    storeRevision = 5;
    await call("rosie_program_add_event", { kind: "home" });
    const conflict = await call("rosie_program_save", { target: "cell" });
    assert.equal(conflict.isError, true);
    assert.match(textOf(conflict), /HTTP 409.*program_conflict.*reopen it/s);
    assert.equal(storeRevision, 5, "nothing was written over the cell's revision");
    const document = seen.findLast((s) => s.route.endsWith("/save"))!.body.document;
    assert.equal(document.identity.name, "stored");
  } finally {
    await session.close();
  }
});
