// Rails parameters, angle profiles, workcell fixtures, several parts and
// waypoint welds through the MCP client, edited by OLP's own engine (the
// bundled core), against a fake OLP at the HTTP boundary. Expectations are
// absolute: millimetres, degrees and metres on the wire, OLP's documented
// presets and defaults, and OLP's own refusal sentences.
import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { once } from "node:events";
import { mkdtempSync, rmSync, readFileSync, writeFileSync } from "node:fs";
import http from "node:http";
import { tmpdir } from "node:os";
import path from "node:path";
import { after, before, test } from "node:test";

import { connect, REPO_ROOT, textOf } from "./helpers.js";

const MODEL = "rosie_1420_v1";
const DESCRIPTION = `sha256:${"2".repeat(64)}`;
const fixture = readFileSync(path.join(REPO_ROOT, "motion-server", "joint-trajectory", "v1", "testdata", "dense_joint_trajectory_reference.rdt"));
const state = mkdtempSync(path.join(tmpdir(), "rosieos-mcp-authoring-"));
const sha = (bytes: Uint8Array) => createHash("sha256").update(bytes).digest("hex");

function stepFile(name: string): { file: string; bytes: Buffer; fingerprint: string } {
  const file = path.join(state, name);
  writeFileSync(file, `ISO-10303-21;\nHEADER;/* ${name} */ENDSEC;DATA;ENDSEC;END-ISO-10303-21;\n`);
  const bytes = readFileSync(file);
  return { file, bytes, fingerprint: sha(bytes) };
}
const plate = stepFile("plate.step");
const bracket = stepFile("bracket.step");
const obstacle = stepFile("obstacle.step");
const column = stepFile("column.step");

/** The fake CAD worker's triangle for every tessellation, in millimetres (CadQuery's unit). */
const MESH_MM = [0, 0, 0, 100, 0, 0, 0, 200, 0];

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

interface Seen { method: string; route: string; body: any }
const seen: Seen[] = [];
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
      seen.push({ method: request.method ?? "", route, body });
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
      if (route === "/cadquery/topology") {
        const fingerprint = sha(Buffer.from(body.step_base64, "base64"));
        if (body.operation === "tessellate") {
          return send(200, { schema_version: 1, unit: "mm", tool: "cadquery", model_id: "mesh", filename: body.filename, fingerprint,
            mesh: { positions: MESH_MM, indices: [0, 1, 2], bounds_min: [0, 0, 0], bounds_max: [100, 200, 0], linear_deflection_mm: body.linear_deflection_mm } });
        }
        // Edge samples span x 0..200, y 0..80, z 0..50 mm: the part's base centre (its anchor) is [100, 40, 0] mm.
        return send(200, { schema_version: 1, unit: "mm", tool: "cadquery", model_id: "part", filename: body.filename, fingerprint, sample_count: 64,
          parts: [{ id: "p1", edge_count: 2 }],
          edges: [{ id: "e1", part_id: "p1", geom_type: "LINE", samples: [[0, 0, 0], [100, 0, 0], [200, 0, 0]], length_mm: 200, closed: false, is_seam_candidate: true },
            { id: "e2", part_id: "p1", geom_type: "LINE", samples: [[0, 0, 0], [0, 80, 50]], length_mm: 94, closed: false }] });
      }
      if (route === "/seam") {
        if (body.operation === "detect_joints") {
          return send(200, { joints: { weld_joints: [{ id: "weld_joint_0001", type: "tee", weld_symbol: "fillet", weld_line: { segments: [{ source_edge_id: "e1", degree: 1, control_points_xyz_m: [[0, 0, 0], [0.2, 0, 0]], knots_u: [0, 0, 1, 1] }], closed: false, length_m: 0.2 }, bisector_rail: { curve: {} } }],
            seams: [{ id: "seam_0001", label: "fillet 200 mm", weld_joint_ref: "weld_joint_0001", span: { start_s: 0, end_s: 1 }, direction: "forward", reference: { mode: "rail_curve" }, process: {},
              orientation: { work_angle_deg: { kind: "constant", value: 45 }, travel_angle_deg: { kind: "constant", value: 10 } }, curve: { from_weld_joint: true } }] } });
        }
        if (body.operation === "plan_torch_path") {
          return send(200, { direction: body.direction, plan: { work_angle_deg: { kind: "constant", value: 42 }, travel_angle_deg: { kind: "constant", value: 12 }, direction: body.direction,
            direction_changed: false, standoff_mm: 17, standoff_departure_mm: 0, clear_fraction: body.direction === "forward" ? 0.9 : 0.8, authored_clear_fraction: 0.7, seam_length_mm: 200,
            knot_fractions: [0, 1], work_path_deg: [42, 42], travel_path_deg: [12, 12] } });
        }
        if (body.operation === "check_torch_fits") return send(200, { clearance: { seam_length_m: 0.2, clear_runs: [[0, 1]], blocked_spans: [] } });
      }
      if (route === "/weld-plan") {
        return send(200, { result: { seams: [] }, weldplan_sha256: "a".repeat(64), program_digest: body.program_digest, dense: denseSummary(), dense_blob_base64: fixture.toString("base64") });
      }
      send(404, { error: "unknown_route", detail: route });
    });
  });
  server.listen(0, "127.0.0.1");
  await once(server, "listening");
  olpUrl = `http://127.0.0.1:${(server.address() as { port: number }).port}`;
});

after(() => new Promise<void>((resolve) => server.close(() => resolve())));

type Session = Awaited<ReturnType<typeof connect>>;
const caller = (session: Session) => async (name: string, args: Record<string, unknown> = {}) =>
  session.client.callTool({ name, arguments: { ...args, response_format: "json" } });
async function ok(call: ReturnType<typeof caller>, name: string, args: Record<string, unknown> = {}) {
  const answer = await call(name, args);
  assert.notEqual(answer.isError, true, `${name}: ${textOf(answer)}`);
  return answer.structuredContent as any;
}
async function refused(call: ReturnType<typeof caller>, name: string, args: Record<string, unknown>, pattern: RegExp) {
  const answer = await call(name, args);
  assert.equal(answer.isError, true, `${name} should be refused`);
  assert.match(textOf(answer), pattern);
}
const fieldValue = (row: any, fieldPath: string) => row.groups.flatMap((g: any) => g.fields).find((f: any) => f.path === fieldPath)?.value;
const near = (actual: number[], expected: number[], tolerance: number, what: string) =>
  expected.forEach((v, i) => assert.ok(Math.abs(actual[i]! - v) <= tolerance, `${what}[${i}]: ${actual[i]} vs ${v}`));
const lastPlan = () => seen.findLast((s) => s.route === "/weld-plan")!.body;
const since = (mark: number, predicate: (s: Seen) => boolean) => seen.slice(mark).filter(predicate);

test("the fitted torch and the angle search: OLP's presets and parameter model, sent to every search and the plan", async () => {
  const session = await connect({ ROSIE_OLP_URL: olpUrl, ROSIE_MCP_STATE_DIR: state });
  const call = caller(session);
  try {
    await ok(call, "rosie_program_new", { name: "rails settings" });
    await ok(call, "rosie_program_import_step", { path: plate.file });
    const shown = await ok(call, "rosie_program_rails_settings");
    const values = (side: any) => Object.fromEntries(side.fields.map((f: any) => [f.key, f.value]));
    // rails/presets.ts: ROSIE_LINK6_GUN (Ø32 mm, 150 mm barrel, held 15-30 mm), DEFAULT_TORCH_STANDOFF_MM 17, STEADY_SEARCH ±15°/±40°, 3°/mm.
    assert.equal(shown.torch.preset.id, "rosie_link6_gun");
    assert.deepEqual(values(shown.torch), { body_radius_mm: 16, body_length_mm: 150, standoff_min_mm: 15, standoff_max_mm: 30, default_standoff_mm: 17 });
    assert.deepEqual(values(shown.search), { work_deviation_deg: 15, travel_deviation_deg: 40, standoff_cost_deg_per_mm: 3 });

    let mark = seen.length;
    await ok(call, "rosie_program_search_seam", { seam_id: "seam_0001" });
    const searches = since(mark, (s) => s.body?.operation === "plan_torch_path");
    assert.deepEqual(searches.map((s) => s.body.direction).sort(), ["forward", "reverse"]);
    for (const s of searches) {
      assert.deepEqual(s.body.torch, { id: "rosie_link6_gun", label: "Rosie 1400, link_6 gun", body_radius_mm: 16, body_length_mm: 150, standoff_range_mm: { min: 15, max: 30 } });
      assert.deepEqual(s.body.search, { work_deviation_deg: 15, travel_deviation_deg: 40, standoff_cost_deg_per_mm: 3 });
      assert.equal(s.body.default_standoff_mm, 17);
    }
    await ok(call, "rosie_program_add_weld", { seam_id: "seam_0001" });

    const changed = await ok(call, "rosie_program_rails_settings", { action: "set", torch_preset: "mig_gas_nozzle_16", torch: { body_length_mm: 200 }, search: { work_deviation_deg: 20 } });
    // MIG_GAS_NOZZLE_16: Ø16 mm, held 6-25 mm; the barrel edited to 200 mm.
    assert.deepEqual(values(changed.torch), { body_radius_mm: 8, body_length_mm: 200, standoff_min_mm: 6, standoff_max_mm: 25, default_standoff_mm: 17 });
    assert.deepEqual(changed.torch.modified, ["body_length_mm"]);
    assert.deepEqual(values(changed.search), { work_deviation_deg: 20, travel_deviation_deg: 40, standoff_cost_deg_per_mm: 3 });
    assert.equal(changed.dropped_searches, true, "a change drops the searched seams, as the browser's parametersChanged");

    const weld = await ok(call, "rosie_program_get_row", { row_id: "rails-seam_0001" });
    assert.deepEqual(fieldValue(weld, "geometry.orientation.work_angle_deg"), { kind: "constant", value: 42 }, "an accepted weld keeps what it was searched with");

    mark = seen.length;
    await ok(call, "rosie_program_search_seam", { seam_id: "seam_0001" });
    const again = since(mark, (s) => s.body?.operation === "plan_torch_path" || s.body?.operation === "check_torch_fits");
    assert.equal(again.filter((s) => s.body.operation === "plan_torch_path").length, 2, "searched again, not answered from the dropped cache");
    for (const s of again) assert.deepEqual(s.body.torch, { id: "mig_gas_nozzle_16", label: "MIG, Ø16 gas nozzle", body_radius_mm: 8, body_length_mm: 200, standoff_range_mm: { min: 6, max: 25 } });
    assert.deepEqual(again.find((s) => s.body.operation === "plan_torch_path")!.body.search, { work_deviation_deg: 20, travel_deviation_deg: 40, standoff_cost_deg_per_mm: 3 });

    await refused(call, "rosie_program_rails_settings", { action: "set", torch: { body_radius_mm: 50 } }, /Nozzle radius above the maximum, 40 mm/);
    await refused(call, "rosie_program_rails_settings", { action: "set", search: { standoff_cost_deg_per_mm: 0 } }, /Standoff cost below the minimum, 0.1 °\/mm/);
    await refused(call, "rosie_program_rails_settings", { action: "set", torch: { standoff_min_mm: 30 } }, /the standoff range crosses: closest 30 mm is past furthest 25 mm/);
    await refused(call, "rosie_program_rails_settings", { action: "set", torch: { standoff_max_mm: 90, body_length_mm: 80 } }, /the barrel is 80 mm and the gun is held out to 90 mm/);
    await refused(call, "rosie_program_rails_settings", { action: "set", torch_preset: "tig" }, /no fitted torch preset tig; shipped: rosie_link6_gun, mig_gas_nozzle_16/);
    assert.deepEqual(values((await ok(call, "rosie_program_rails_settings")).torch).body_radius_mm, 8, "a refused change keeps the settings");

    await ok(call, "rosie_program_plan", { from_program_start: true });
    assert.deepEqual(lastPlan().tooling, { id: "mig_gas_nozzle_16", label: "MIG, Ø16 gas nozzle", body_radius_mm: 8, body_length_mm: 200, standoff_range_mm: { min: 6, max: 25 }, default_standoff_mm: 17 });

    const reset = await ok(call, "rosie_program_rails_settings", { action: "reset" });
    assert.equal(reset.torch.preset.id, "rosie_link6_gun");
    assert.deepEqual(reset.torch.modified, []);
  } finally {
    await session.close();
  }
});

test("a rails weld's work and travel angle profiles: OLP's fromPoints, written through the dialog's edit and planned", async () => {
  const session = await connect({ ROSIE_OLP_URL: olpUrl, ROSIE_MCP_STATE_DIR: state });
  const call = caller(session);
  try {
    await ok(call, "rosie_program_new", { name: "profiles" });
    await ok(call, "rosie_program_import_step", { path: plate.file });
    await ok(call, "rosie_program_add_weld", { seam_id: "seam_0001" });
    const row = "rails-seam_0001";
    await ok(call, "rosie_program_set_weld_profile", { row_id: row, angle: "work", points: [{ s: 0, deg: 30 }, { s: 0.5, deg: 45 }, { s: 1, deg: 40 }] });
    await ok(call, "rosie_program_set_weld_profile", { row_id: row, angle: "travel", constant_deg: 12.5 });
    const weld = await ok(call, "rosie_program_get_row", { row_id: row });
    assert.deepEqual(fieldValue(weld, "geometry.orientation.work_angle_deg"), { kind: "piecewise_linear", knots: [[0, 30], [0.5, 45], [1, 40]] });
    assert.deepEqual(fieldValue(weld, "geometry.orientation.travel_angle_deg"), { kind: "constant", value: 12.5 });

    await refused(call, "rosie_program_set_weld_profile", { row_id: row, angle: "work", points: [{ s: 0, deg: 30 }, { s: 1, deg: 95 }] }, /95°; the profile editor's range is -90° to 90°/);
    await refused(call, "rosie_program_set_weld_profile", { row_id: row, angle: "work", points: [{ s: 0.2, deg: 30 }, { s: 1, deg: 40 }] }, /first point is at s=0/);
    await refused(call, "rosie_program_set_weld_profile", { row_id: row, angle: "work", points: [{ s: 0, deg: 30 }, { s: 0.6, deg: 31 }, { s: 0.6, deg: 32 }, { s: 1, deg: 40 }] }, /must come after point 1/);
    await refused(call, "rosie_program_set_weld_profile", { row_id: `${row}-approach`, angle: "work", constant_deg: 30 }, /not a rails \(seam\) weld/);
    await refused(call, "rosie_program_set_weld_profile", { row_id: row, angle: "work" }, /exactly one of constant_deg or points/);

    // A flat set of points is a constant, as the editor writes it back.
    await ok(call, "rosie_program_set_weld_profile", { row_id: row, angle: "travel", points: [{ s: 0, deg: 8 }, { s: 0.3, deg: 8 }, { s: 1, deg: 8 }] });
    assert.deepEqual(fieldValue(await ok(call, "rosie_program_get_row", { row_id: row }), "geometry.orientation.travel_angle_deg"), { kind: "constant", value: 8 });

    const undone = await ok(call, "rosie_program_undo", { steps: 1 });
    assert.equal(undone.rows.some((r: any) => r.id === row), true);
    assert.deepEqual(fieldValue(await ok(call, "rosie_program_get_row", { row_id: row }), "geometry.orientation.travel_angle_deg"), { kind: "constant", value: 12.5 }, "each profile edit is one undo step");

    await ok(call, "rosie_program_plan", { from_program_start: true });
    const planned = lastPlan().program.nodes.find((n: any) => n.id === row);
    assert.deepEqual(planned.geometry.orientation.work_angle_deg, { kind: "piecewise_linear", knots: [[0, 30], [0.5, 45], [1, 40]] });
    assert.deepEqual(planned.geometry.orientation.travel_angle_deg, { kind: "constant", value: 12.5 });
  } finally {
    await session.close();
  }
});

test("workcell fixtures: the stand edited in mm and degrees, STEP obstacles as meshes on the plan wire, a missing mesh rebuilt from the kept STEP", async () => {
  const session = await connect({ ROSIE_OLP_URL: olpUrl, ROSIE_MCP_STATE_DIR: state });
  const call = caller(session);
  const saved = path.join(state, "fixtures.json");
  try {
    await ok(call, "rosie_program_new", { name: "fixtures" });
    // workcell.ts defaultWorkcellFixtures(rosie_1420_v1): a 0.5 x 0.5 x 1.0 m box, footprint centred at 1.0, 1.0 m.
    const listed = await ok(call, "rosie_program_list_fixtures");
    assert.equal(listed.fixtures.length, 1);
    assert.deepEqual([listed.fixtures[0].id, listed.fixtures[0].origin, listed.fixtures[0].shape], ["stand", "cell-default", { kind: "box", sizeMm: [500, 500, 1000] }]);
    assert.deepEqual(listed.fixtures[0].positionMm, [1000, 1000, 0]);

    await ok(call, "rosie_program_edit_fixture", { action: "edit", fixture_id: "stand", fields: { sx: 600, z: 10, rz: 90 } });
    await refused(call, "rosie_program_edit_fixture", { action: "edit", fixture_id: "stand", fields: { sx: 6000 } }, /sx 6000 mm is outside the fixture card's 1 to 5000 mm/);
    await refused(call, "rosie_program_edit_fixture", { action: "edit", fixture_id: "stand", fields: { rx: 400 } }, /rx 400 ° is outside the fixture card's -360 to 360 °/);

    let mark = seen.length;
    await ok(call, "rosie_program_edit_fixture", { action: "add_step_fixture", path: obstacle.file });
    const tessellation = since(mark, (s) => s.route === "/cadquery/topology")[0]!.body;
    // cad/client.ts tessellate: OLP's own deflection and triangle budget.
    assert.deepEqual({ ...tessellation, step_base64: undefined }, { operation: "tessellate", filename: "obstacle.step", step_base64: undefined, linear_deflection_mm: 0.5, angular_deflection_rad: 0.5, max_triangles: 400000 });
    assert.equal(tessellation.step_base64, obstacle.bytes.toString("base64"));
    await ok(call, "rosie_program_edit_fixture", { action: "edit", fixture_id: "fixture-1", fields: { x: -250, y: 300 } });
    await refused(call, "rosie_program_edit_fixture", { action: "edit", fixture_id: "fixture-1", fields: { sx: 10 } }, /STEP fixture: it has no box size/);
    const already = await ok(call, "rosie_program_edit_fixture", { action: "add_stand" });
    assert.ok(already.workcell.some((f: any) => f.id === "stand"));

    await ok(call, "rosie_program_add_event", { kind: "home" });
    await ok(call, "rosie_program_plan", { from_program_start: true });
    const wire = lastPlan().fixtures;
    assert.deepEqual(wire.map((f: any) => f.id), ["stand", "fixture-1"]);
    assert.deepEqual(wire[0].shape, { kind: "box", size_m: [0.6, 0.5, 1.0] });
    near(wire[0].xyz_m, [1.0, 1.0, 0.01], 1e-12, "stand xyz_m");
    near(wire[0].rpy_rad, [0, 0, Math.PI / 2], 1e-12, "stand rpy_rad");
    assert.deepEqual([wire[1].shape.kind, wire[1].shape.fingerprint, wire[1].shape.filename], ["mesh", obstacle.fingerprint, "obstacle.step"]);
    near(wire[1].shape.vertices, MESH_MM.map((v) => v / 1000), 1e-7, "mesh vertices (m)");
    assert.deepEqual(wire[1].shape.faces, [0, 1, 2]);
    near(wire[1].xyz_m, [-0.25, 0.3, 0], 1e-12, "obstacle xyz_m");
    await ok(call, "rosie_program_save", { target: "file", path: saved });
  } finally {
    await session.close();
  }

  // A new server process keeps no meshes in memory: the plan's mesh is rebuilt from CadQuery's reply
  // kept in the state dir, through OLP's own client, without running CadQuery again.
  const isTessellate = (s: { route: string; body: any }) => s.route === "/cadquery/topology" && s.body.operation === "tessellate";
  const fromReply = await connect({ ROSIE_OLP_URL: olpUrl, ROSIE_MCP_STATE_DIR: state });
  try {
    await ok(caller(fromReply), "rosie_program_open", { source: "file", path: saved });
    const mark = seen.length;
    await ok(caller(fromReply), "rosie_program_plan", { from_program_start: true });
    assert.equal(seen.slice(mark).some(isTessellate), false, "rebuilt from the kept CadQuery reply");
    near(lastPlan().fixtures[1].shape.vertices, MESH_MM.map((v) => v / 1000), 1e-7, "mesh vertices rebuilt from the kept reply (m)");
  } finally {
    await fromReply.close();
  }

  // With the kept reply gone, the kept STEP is tessellated again before the plan.
  rmSync(path.join(state, "meshes"), { recursive: true, force: true });
  const reopened = await connect({ ROSIE_OLP_URL: olpUrl, ROSIE_MCP_STATE_DIR: state });
  const again = caller(reopened);
  try {
    await ok(again, "rosie_program_open", { source: "file", path: saved });
    const mark = seen.length;
    await ok(again, "rosie_program_plan", { from_program_start: true });
    const after = seen.slice(mark);
    const tessellated = after.findIndex(isTessellate);
    assert.ok(tessellated >= 0 && tessellated < after.findIndex((s) => s.route === "/weld-plan"), "tessellated before the plan");
    assert.equal(after[tessellated]!.body.step_base64, obstacle.bytes.toString("base64"));
    near(lastPlan().fixtures[1].shape.vertices, MESH_MM.map((v) => v / 1000), 1e-7, "rebuilt mesh vertices (m)");

    await ok(again, "rosie_program_edit_fixture", { action: "cell_default", fixture_id: "stand" });
    await ok(again, "rosie_program_edit_fixture", { action: "replace_with_step", fixture_id: "stand", path: column.file });
    const replaced = (await ok(again, "rosie_program_list_fixtures")).fixtures.find((f: any) => f.id === "stand");
    // The cell placeholder becomes "Stand", keeping its pose (cell default: 1000, 1000, 0 mm).
    assert.deepEqual([replaced.name, replaced.origin, replaced.shape], ["Stand", "authored", { kind: "step", filename: "column.step", fingerprint: column.fingerprint }]);
    assert.deepEqual(replaced.positionMm, [1000, 1000, 0]);
    const removed = await ok(again, "rosie_program_edit_fixture", { action: "remove", fixture_id: "fixture-1" });
    assert.deepEqual(removed.workcell.map((f: any) => f.id), ["stand"]);
  } finally {
    await reopened.close();
  }

  // A server that never kept the obstacle's STEP says so, posts no plan, and takes the file when given.
  const elsewhere = await connect({ ROSIE_OLP_URL: olpUrl, ROSIE_MCP_STATE_DIR: mkdtempSync(path.join(tmpdir(), "rosieos-mcp-elsewhere-")) });
  const third = caller(elsewhere);
  try {
    await ok(third, "rosie_program_open", { source: "file", path: saved });
    const mark = seen.length;
    await refused(third, "rosie_program_plan", { from_program_start: true }, /STEP for obstacle\.step .* is not kept by this server.*fixture_step_paths/);
    assert.equal(since(mark, (s) => s.route === "/weld-plan").length, 0);
    await ok(third, "rosie_program_plan", { from_program_start: true, fixture_step_paths: [obstacle.file] });
    assert.equal(lastPlan().fixtures[1].shape.fingerprint, obstacle.fingerprint);
  } finally {
    await elsewhere.close();
  }
});

test("several parts: only the active part's welds and STEP plan, the others go as mesh obstacles where they are placed", async () => {
  const session = await connect({ ROSIE_OLP_URL: olpUrl, ROSIE_MCP_STATE_DIR: state });
  const call = caller(session);
  try {
    await ok(call, "rosie_program_new", { name: "parts" });
    await ok(call, "rosie_program_import_step", { path: plate.file });
    await ok(call, "rosie_program_add_weld", { seam_id: "seam_0001" });
    await ok(call, "rosie_program_place_part", { offset_mm: [300, -50, 0] });
    const added = await ok(call, "rosie_program_add_part", { path: bracket.file });
    assert.equal(added.rows.length, 0, "the new part starts with no welds");
    let parts = (await ok(call, "rosie_program_list_parts")).parts;
    assert.deepEqual(parts.map((p: any) => [p.filename, p.active, p.welds]), [["plate.step", false, 1], ["bracket.step", true, 0]]);
    const plateId = parts[0].id, bracketId = parts[1].id;

    await ok(call, "rosie_program_add_event", { kind: "home" });
    await ok(call, "rosie_program_plan", { from_program_start: true });
    let body = lastPlan();
    assert.equal(body.step_base64, bracket.bytes.toString("base64"));
    assert.equal(body.program.nodes.some((n: any) => n.op === "weld"), false, "the inactive part's weld is not planned");
    let other = body.fixtures.find((f: any) => f.id === `workpiece:${plateId}`);
    assert.deepEqual([other.shape.kind, other.shape.fingerprint, other.shape.filename], ["mesh", plate.fingerprint, "plate.step"]);
    // The plate's CAD origin in the work frame (world here): its placement offset less its anchor, the edge
    // samples' base centre [100, 40, 0] mm, with no rotation: [300 - 100, -50 - 40, 0] mm.
    near(other.xyz_m, [0.2, -0.09, 0], 1e-9, "inactive part xyz_m");

    await refused(call, "rosie_program_switch_part", { part_id: "nope" }, /no part nope; parts: .*plate\.step/);
    const switched = await ok(call, "rosie_program_switch_part", { part_id: plateId });
    assert.deepEqual(switched.rows.map((r: any) => r.id), ["rails-seam_0001-approach", "rails-seam_0001", "rails-seam_0001-retract"]);
    await ok(call, "rosie_program_plan", { from_program_start: true });
    body = lastPlan();
    assert.equal(body.step_base64, plate.bytes.toString("base64"));
    assert.equal(body.program.nodes.filter((n: any) => n.op === "weld").length, 1);
    other = body.fixtures.find((f: any) => f.id === `workpiece:${bracketId}`);
    assert.equal(other.shape.fingerprint, bracket.fingerprint);

    const removed = await ok(call, "rosie_program_remove_part");
    assert.equal(removed.rows.length, 0);
    parts = (await ok(call, "rosie_program_list_parts")).parts;
    assert.deepEqual(parts.map((p: any) => [p.filename, p.active]), [["bracket.step", false]]);
    await ok(call, "rosie_program_undo", { steps: 1 });
    assert.equal((await ok(call, "rosie_program_list_parts")).parts.length, 2, "Delete part is one undo step");
  } finally {
    await session.close();
  }
});

test("waypoint, line and arc welds: OLP's WaypointSession in the part frame, the contract's refusals, lowered for the planner", async () => {
  const session = await connect({ ROSIE_OLP_URL: olpUrl, ROSIE_MCP_STATE_DIR: state });
  const call = caller(session);
  const file = path.join(state, "waypoints.json");
  const down = [180, 0, 0];
  try {
    await ok(call, "rosie_program_new", { name: "waypoints" });
    await refused(call, "rosie_program_add_waypoint_weld", { tool: "line", points: [{ xyz_mm: [0, 0, 0], rpy_deg: down }, { xyz_mm: [100, 0, 0], rpy_deg: down }] }, /Import a part to place welds on/);
    await ok(call, "rosie_program_import_step", { path: plate.file });

    const line = await ok(call, "rosie_program_add_waypoint_weld", { tool: "line", points: [{ xyz_mm: [0, 0, 10], rpy_deg: down }, { xyz_mm: [100, 0, 10], rpy_deg: down, standoff_mm: 20 }] });
    const lineId = line.row_id;
    assert.equal(line.waypoint_weld.label, "Line 1");
    assert.deepEqual(line.program.rows.map((r: any) => r.op), ["approach", "weld", "retract"]);

    await refused(call, "rosie_program_add_waypoint_weld", { tool: "arc", points: [{ xyz_mm: [0, 0, 0], rpy_deg: down }, { xyz_mm: [50, 0, 0], rpy_deg: down }, { xyz_mm: [100, 0, 0], rpy_deg: down }] }, /arc points lie on a line/);
    await refused(call, "rosie_program_add_waypoint_weld", { tool: "line", points: [{ xyz_mm: [0, 0, 0], rpy_deg: down }, { xyz_mm: [50, 0, 0], rpy_deg: down }, { xyz_mm: [100, 0, 0], rpy_deg: down }] }, /a line weld has exactly two points/);
    await refused(call, "rosie_program_add_waypoint_weld", { tool: "waypoint", points: [{ xyz_mm: [0, 0, 0], rpy_deg: down }, { xyz_mm: [0, 0, 0.0005], rpy_deg: down }] }, /coincide/);
    await refused(call, "rosie_program_add_waypoint_weld", { tool: "line", points: [{ xyz_mm: [0, 0, 0], rpy_deg: down, standoff_mm: 150 }, { xyz_mm: [100, 0, 0], rpy_deg: down }] }, /standoff 150 mm is outside 0.5–100 mm/);
    assert.equal((await ok(call, "rosie_program_view")).rows.length, 3, "nothing refused became a row");

    const curve = await ok(call, "rosie_program_add_waypoint_weld", { tool: "waypoint", label: "S bend", travel_speed_mm_s: 6, points: [{ xyz_mm: [0, 50, 10], rpy_deg: down }, { xyz_mm: [50, 50, 10], rpy_deg: down }, { xyz_mm: [100, 60, 10], rpy_deg: [170, 5, 30] }] });
    const curveId = curve.row_id;
    assert.deepEqual(curve.waypoint_weld.points.map((p: any) => p.continuity), ["c1", "c1", "c1"], "a waypoint weld starts smooth");
    near(curve.waypoint_weld.points[2].rpyDeg, [170, 5, 30], 1e-9, "rpy round trip");
    const edit = (args: Record<string, unknown>) => ok(call, "rosie_program_edit_waypoint_weld", { row_id: curveId, ...args });
    await edit({ op: "add_point", after_index: 0, point: { xyz_mm: [25, 40, 10], rpy_deg: down } });
    await edit({ op: "move_point", index: 3, xyz_mm: [120, 60, 10] });
    await edit({ op: "set_standoff", index: 2, standoff_mm: 25 });
    const smooth = await edit({ op: "set_continuity", continuity: "c2" });
    assert.deepEqual(smooth.waypoint_weld.points.map((p: any) => [p.xyzMm, p.standoffMm, p.continuity]), [
      [[0, 50, 10], 17, "c2"], [[25, 40, 10], 17, "c2"], [[50, 50, 10], 25, "c2"], [[120, 60, 10], 17, "c2"],
    ]);
    const tied = await edit({ op: "set_start_from", start_from_row_id: lineId });
    near(tied.waypoint_weld.points[0].xyzMm, [100, 0, 10], 1e-9, "a tied first point sits on the line's end");
    await refused(call, "rosie_program_edit_waypoint_weld", { row_id: curveId, op: "move_point", index: 0, xyz_mm: [1, 2, 3] }, /only its orientation can change/);
    await refused(call, "rosie_program_edit_waypoint_weld", { row_id: lineId, op: "add_point", point: { xyz_mm: [50, 0, 10], rpy_deg: down } }, /a line keeps two points and an arc three/);
    await refused(call, "rosie_program_edit_waypoint_weld", { row_id: curveId, op: "set_standoff", standoff_mm: 0.2 }, /outside 0.5–100 mm/);
    await refused(call, "rosie_program_edit_waypoint_weld", { row_id: curveId, op: "move_point", index: 9, xyz_mm: [0, 0, 0] }, /has 4 points .* there is no point 9/);
    await refused(call, "rosie_program_edit_waypoint_weld", { row_id: "rails-nothing", op: "remove_point", index: 0 }, /no longer in the program/);

    await ok(call, "rosie_program_edit_waypoint_weld", { row_id: lineId, op: "move_point", index: 1, xyz_mm: [150, 0, 10] });
    const followed = await ok(call, "rosie_program_get_row", { row_id: curveId });
    near(followed.waypoint_weld.points[0].xyzMm, [150, 0, 10], 1e-9, "the tied start follows the line's moved end");

    await ok(call, "rosie_program_save", { target: "file", path: file });
    const nodes = JSON.parse(readFileSync(file, "utf8")).project.nodes;
    const lineNode = nodes.find((n: any) => n.id === lineId);
    assert.equal(lineNode.geometry.kind, "manual_pose_wp");
    assert.equal(lineNode.geometry.frame_id, "workpiece");
    near(lineNode.geometry.points[0].xyz_m, [0, 0, 0.01], 1e-15, "line P0 xyz_m");
    near(lineNode.geometry.points[1].xyz_m, [0.15, 0, 0.01], 1e-15, "line P1 xyz_m");
    near(lineNode.geometry.points[0].rpy_rad, [Math.PI, 0, 0], 1e-12, "line P0 rpy_rad");
    assert.deepEqual(lineNode.geometry.points.map((p: any) => p.continuity), ["C0", "C0"]);
    // DEFAULT_TORCH_STANDOFF_MM 17 at s=0, 20 at s=1; DEFAULT_TRAVEL_SPEED_MPS 0.005 = 5 mm/s.
    assert.deepEqual(lineNode.standoff_mm, { kind: "piecewise_linear", knots: [[0, 17], [1, 20]] });
    assert.deepEqual(lineNode.travel_speed_mm_s, { kind: "constant", value: 5 });
    assert.equal(lineNode.extensions.manual_pose_wp.tool, "line");
    const curveNode = nodes.find((n: any) => n.id === curveId);
    assert.deepEqual([curveNode.label, curveNode.travel_speed_mm_s], ["S bend", { kind: "constant", value: 6 }]);
    assert.equal(curveNode.extensions.manual_pose_wp.start_from, lineId);

    await ok(call, "rosie_program_plan", { from_program_start: true });
    const planned = lastPlan().program.nodes.filter((n: any) => n.op === "weld");
    assert.deepEqual(planned.map((n: any) => [n.id, n.geometry.kind]), [[lineId, "seam"], [curveId, "seam"]], "lowered to seams for the planner");

    const before = (await ok(call, "rosie_program_get_row", { row_id: curveId })).waypoint_weld.points.length;
    await ok(call, "rosie_program_edit_waypoint_weld", { row_id: curveId, op: "remove_point", index: 1 });
    assert.equal((await ok(call, "rosie_program_get_row", { row_id: curveId })).waypoint_weld.points.length, before - 1);
    await ok(call, "rosie_program_undo", { steps: 1 });
    assert.equal((await ok(call, "rosie_program_get_row", { row_id: curveId })).waypoint_weld.points.length, before, "each edit is one undo step");
  } finally {
    await session.close();
  }
});
