// Fakes at the process boundary (OLP over HTTP, rt-control over its socket),
// shared by the CLI's tests and the MCP server's (`rosie-cli/testing`), so
// both front ends are proved against the same OLP behaviour.
import { once } from "node:events";
import http from "node:http";
import { tmpdir } from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";

/** cli/v1/dist/test -> the repository root. */
export const REPO_ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../../../..");

export interface Recorded {
  method: string;
  path: string;
  headers: http.IncomingHttpHeaders;
  body: unknown;
}

/** A scripted answer for one route: [HTTP code, JSON body], possibly after a delay. */
export type FakeRoute = (request: Recorded) => [number, unknown] | Promise<[number, unknown]>;

export interface FakeOlp {
  url: string;
  requests: Recorded[];
  status: Record<string, unknown>;
  /** Routes (without the /api/offline-programming/v1 prefix) answered before the defaults. */
  routes: Record<string, FakeRoute>;
  close(): Promise<void>;
}

export function simTarget(simulation: boolean) {
  return {
    selection_generation: "7",
    cell_id: simulation ? "local-sim" : "example-arm-1420",
    model_id: "rosie_1420_v1",
    label: simulation ? "Local simulation" : "rosie_1420_v1",
    backend: simulation ? "simulation" : "ethercat",
    simulation,
    host: "localhost",
  };
}

/** An OLP that answers status from `status` and records every request. */
export async function startFakeOlp(status: Record<string, unknown>): Promise<FakeOlp> {
  const requests: Recorded[] = [];
  const fake: FakeOlp = { url: "", requests, status, routes: {}, close: async () => {} };
  const server = http.createServer((request, response) => {
    const chunks: Buffer[] = [];
    request.on("data", (chunk: Buffer) => chunks.push(chunk));
    request.on("end", () => {
      const raw = Buffer.concat(chunks).toString("utf8");
      const route = (request.url ?? "").replace("/api/offline-programming/v1", "");
      const recorded: Recorded = { method: request.method ?? "", path: route, headers: request.headers, body: raw ? JSON.parse(raw) : undefined };
      requests.push(recorded);
      const send = (code: number, value: unknown) => {
        response.writeHead(code, { "Content-Type": "application/json" });
        response.end(JSON.stringify(value));
      };
      const scripted = fake.routes[route];
      if (scripted) {
        void Promise.resolve(scripted(recorded)).then(([code, value]) => send(code, value));
        return;
      }
      if (route === "/dense-execution/status") return send(200, fake.status);
      if (route === "/dense-execution/arm") return send(200, { ok: true, accepted: true, status: fake.status });
      if (route === "/dense-execution/stop") return send(200, { ok: true, stopped: true, attempts: 1, status: fake.status });
      if (route === "/dense-execution/heartbeat") return send(200, { ok: true, deadline_ms: 5000, age_ms: 10 });
      if (route === "/dense-execution/play") return send(200, fake.status);
      if (route === "/maintenance") return send(200, { active: false, clients: [] });
      if (route === "/control") return send(200, { state: "free", armed: false, lease_held: false, stop_available: true });
      send(404, { error: "unknown_route", detail: route });
    });
  });
  server.listen(0, "127.0.0.1");
  await once(server, "listening");
  const address = server.address() as { port: number };
  fake.url = `http://127.0.0.1:${address.port}`;
  fake.close = () => new Promise((resolve) => server.close(() => resolve()));
  return fake;
}

/** rt-control's public socket: a Unix socket, or a named pipe on Windows. */
export async function startFakeRtControl(routes: Record<string, unknown>): Promise<{ socketPath: string; close(): Promise<void> }> {
  const socketPath = process.platform === "win32" ? `\\\\.\\pipe\\rosieos-mcp-test-${process.pid}-${Date.now()}` : path.join(tmpdir(), `rosieos-mcp-${process.pid}-${Date.now()}.sock`);
  const server = http.createServer((request, response) => {
    const body = routes[(request.url ?? "").split("?")[0]!];
    response.writeHead(body === undefined ? 404 : 200, { "Content-Type": "application/json" });
    // Ordinary replies are the rosie.rt-control.response.v1 envelope.
    response.end(JSON.stringify(body === undefined ? { schema: "rosie.rt-control.response.v1", error: "unknown_endpoint" } : { schema: "rosie.rt-control.response.v1", operation: "get", data: body }));
  });
  server.listen(socketPath);
  await once(server, "listening");
  return { socketPath, close: () => new Promise((resolve) => server.close(() => resolve())) };
}

type Mat = number[][];
const mul = (a: Mat, b: Mat): Mat => a.map((row) => b[0]!.map((_, j) => row.reduce((sum, v, k) => sum + v * b[k]![j]!, 0)));
function originOf(xyz: number[], rpy: number[]): Mat {
  const [r, p, y] = rpy as [number, number, number];
  const [cr, sr, cp, sp, cy, sy] = [Math.cos(r), Math.sin(r), Math.cos(p), Math.sin(p), Math.cos(y), Math.sin(y)];
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]!],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]!],
    [-sp, cp * sr, cp * cr, xyz[2]!],
    [0, 0, 0, 1],
  ];
}
function aboutAxis(axis: number[], q: number): Mat {
  const n = Math.hypot(...axis), [x, y, z] = axis.map((v) => v / n) as [number, number, number];
  const c = Math.cos(q), s = Math.sin(q), t = 1 - c;
  return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0], [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0], [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0], [0, 0, 0, 1]];
}

/** FK composed from the URDF's joint origins and axes, independent of three.js and urdf-loader: the absolute check on OLP's capture FK. */
export function urdfTcpMmOf(urdf: string, jointsRad: Record<string, number>, tcp = "tool0"): number[] {
  const joints = [...urdf.matchAll(/<joint\b[^>]*name="([^"]+)"[^>]*type="([^"]+)"[^>]*>([\s\S]*?)<\/joint>/g)].map(([, name, type, inner]) => {
    const attr = (tag: string, key: string) => new RegExp(`<${tag}\\b[^>]*\\b${key}="([^"]+)"`).exec(inner!)?.[1];
    const nums = (value: string | undefined, fallback: number[]) => (value ? value.trim().split(/\s+/).map(Number) : fallback);
    return { name: name!, type: type!, parent: attr("parent", "link")!, child: attr("child", "link")!, xyz: nums(attr("origin", "xyz"), [0, 0, 0]), rpy: nums(attr("origin", "rpy"), [0, 0, 0]), axis: nums(attr("axis", "xyz"), [1, 0, 0]) };
  });
  const chain: typeof joints = [];
  for (let link = tcp; ;) {
    const joint = joints.find((candidate) => candidate.child === link);
    if (!joint) break;
    chain.unshift(joint);
    link = joint.parent;
  }
  let transform: Mat = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
  for (const joint of chain) {
    transform = mul(transform, originOf(joint.xyz, joint.rpy));
    if (joint.type !== "fixed") transform = mul(transform, aboutAxis(joint.axis, jointsRad[joint.name] ?? 0));
  }
  return [transform[0]![3]! * 1000, transform[1]![3]! * 1000, transform[2]![3]! * 1000];
}

/**
 * An OLP that authors and plans: robots, capture model, CadQuery topology and
 * tessellation, the seam worker (one detected seam; reverse travel searches
 * 95 % clear), /weld-plan answering the reference trajectory, and a
 * revision-checked cell program store. Program flows of both front ends are
 * proved against it.
 */
export interface FakeProgramOlp {
  url: string;
  model: string;
  description: string;
  urdf: string;
  stepFile: string;
  fingerprint: string;
  seen: { method: string; route: string; body: any }[];
  storeRevision: number;
  close(): Promise<void>;
}

export async function startFakeProgramOlp(dir: string): Promise<FakeProgramOlp> {
  const { createHash } = await import("node:crypto");
  const { readFileSync, writeFileSync } = await import("node:fs");
  const model = "rosie_1420_v1";
  const description = `sha256:${"2".repeat(64)}`;
  const urdf = readFileSync(path.join(REPO_ROOT, "robot_description", "robots", model, "robot.urdf"), "utf8");
  const reference = readFileSync(path.join(REPO_ROOT, "motion-server", "joint-trajectory", "v1", "testdata", "dense_joint_trajectory_reference.rdt"));
  const stepFile = path.join(dir, "plate.step");
  writeFileSync(stepFile, "ISO-10303-21;\nHEADER;ENDSEC;DATA;ENDSEC;END-ISO-10303-21;\n");
  const fingerprint = createHash("sha256").update(readFileSync(stepFile)).digest("hex");
  const header = JSON.parse(reference.subarray(4, 4 + reference.readUInt32BE(0)).toString("utf8"));
  const dense = {
    schema: header.schema, trajectory_digest: header.trajectory_digest, request_sha256: "f".repeat(64), plan_id: header.plan_id,
    program_id: header.program_id, program_digest: header.program_digest, manifest_revision: header.manifest_revision,
    plan_revision: header.plan_revision, created_at: header.created_at, dt_s: header.sampling.dt_s,
    total_sample_count: header.sampling.total_sample_count, total_duration_s: header.sampling.total_duration_s, bytes: reference.byteLength,
    segments: header.segments.map((s: Record<string, unknown>) => ({ index: s.index, kind: s.kind, source_id: s.source_id, sample_count: s.sample_count, duration_s: s.duration_s })),
    robot_cell: null,
  };
  const fake: FakeProgramOlp = { url: "", model, description, urdf, stepFile, fingerprint, seen: [], storeRevision: 0, close: async () => {} };
  const server = http.createServer((request, response) => {
    const chunks: Buffer[] = [];
    request.on("data", (chunk: Buffer) => chunks.push(chunk));
    request.on("end", () => {
      const raw = Buffer.concat(chunks).toString("utf8");
      const route = (request.url ?? "").replace("/api/offline-programming/v1", "").split("?")[0]!;
      let body: any;
      try {
        body = raw ? JSON.parse(raw) : undefined;
      } catch {
        body = raw;
      }
      fake.seen.push({ method: request.method ?? "", route, body });
      const send = (code: number, value: unknown) => {
        response.writeHead(code, { "Content-Type": "application/json" });
        response.end(JSON.stringify(value));
      };
      if (route === "/robots") {
        return send(200, { problems: [], robots: [{ model_id: model, robot_description_sha256: description,
          frames: { base: "world", tool: "tool0", work: "world", work_world_m: [0, 0, 0], arm_base_world_m: [0.4127, 0.4, 0.893] },
          axes: { driven: ["J1", "J2", "J3", "J4", "J5", "J6"] }, reset_pose: { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 } }] });
      }
      if (route === "/dense-execution/status") return send(200, { available: false, target: null, armed: false, session: null });
      if (route === "/health") return send(200, { ok: true, build_id: "fake", maintenance: { active: false }, cell_programs: { enabled: true } });
      if (route === "/capabilities") return send(200, { weld_planner: { enabled: true } });
      if (route === "/robots/capture-model") {
        return send(200, { urdf, identity: { model_id: model, robot_description_sha256: body.robot_description_sha256, machine_planning_calibration_sha256: `sha256:${"e".repeat(64)}`, machine_planning_calibration_source: "empty" } });
      }
      if (route === "/cadquery/topology" && body.operation === "tessellate") {
        // A 200 x 100 x 10 mm plate, two triangles per face (CadQuery's wire, mm).
        const [x, y, z] = [200, 100, 10];
        const positions = [0, 0, 0, x, 0, 0, x, y, 0, 0, y, 0, 0, 0, z, x, 0, z, x, y, z, 0, y, z];
        const indices = [0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7];
        return send(200, { schema_version: 1, tool: "cadquery", model_id: "plate", filename: body.filename, fingerprint, unit: "mm",
          mesh: { positions, indices, vertex_count: 8, triangle_count: 12, bounds_min: [0, 0, 0], bounds_max: [x, y, z], linear_deflection_mm: 0.1, angular_deflection_rad: 0.5, solid_count: 1, truncated: false } });
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
        if (typeof body !== "object") return send(400, { ok: false, code: "weld_plan_invalid", error: "request is not valid JSON" });
        return send(200, { result: { seams: [] }, weldplan_sha256: "a".repeat(64), program_digest: body.program_digest, dense, dense_blob_base64: reference.toString("base64") });
      }
      const save = /^\/cell-programs\/programs\/([^/]+)\/save$/.exec(route);
      if (save) {
        if (body.base_revision !== fake.storeRevision) return send(409, { error: "program_conflict", detail: `the cell has revision ${fake.storeRevision}, saved by steamdeck-operator` });
        fake.storeRevision += 1;
        return send(200, { ok: true, program: { program_id: save[1], revision: fake.storeRevision }, replayed: false });
      }
      if (route === "/cell-programs") return send(200, { programs: [] });
      send(404, { error: "unknown_route", detail: route });
    });
  });
  server.listen(0, "127.0.0.1");
  await once(server, "listening");
  fake.url = `http://127.0.0.1:${(server.address() as { port: number }).port}`;
  fake.close = () => new Promise((resolve) => server.close(() => resolve()));
  return fake;
}
