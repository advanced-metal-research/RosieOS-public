import assert from "node:assert/strict";
import test from "node:test";

import {
  DOG_BOX_FIXTURE,
  CadClientError,
  type CadSequenceResultWire,
  type CadTopologyWire,
  type CadWorkpieceMeshWire,
} from "./contracts.ts";
import {
  HttpCadClient,
  failedCadState,
  readyCadState,
} from "./client.ts";
import {
  normalizeSeamResult,
  normalizeSequenceResult,
  normalizeTopology,
  normalizeWorkpieceMesh,
} from "./normalize.ts";

const FINGERPRINT = "a".repeat(64);

test("normalizes topology millimeters to explicit meter fields", () => {
  const wire: CadTopologyWire = {
    schema_version: 1,
    tool: "rosie cadquery v1 seam topology",
    model_id: "cadquery-step-a",
    filename: "part.step",
    fingerprint: FINGERPRINT,
    unit: "mm",
    sample_count: 32,
    parts: [{ id: "part_0", edge_count: 1 }],
    edges: [{
      id: "part_0:edge_00000",
      label: "A",
      part_id: "part_0",
      geom_type: "LINE",
      samples: [[0, 0, 0], [1000, 2000, 3000]],
      length_mm: 2000,
      bounds_min: [0, 0, 0],
      bounds_max: [1000, 2000, 3000],
      is_seam_candidate: true,
    }],
    seam_candidate_count: 1,
    recommended_seams: [{
      edge_id: "part_0:edge_00000",
      start_s: 0.1,
      end_s: 0.9,
    }],
  };
  const topology = normalizeTopology(wire);
  assert.equal(topology.unit, "m");
  assert.equal(topology.sourceUnit, "mm");
  assert.deepEqual(topology.edges[0]?.samplesM[1], [1, 2, 3]);
  assert.equal(topology.edges[0]?.lengthM, 2);
  assert.equal(topology.seamCandidateCount, 1);
});

test("normalizes seam, plan, collision, and feed dimensions", () => {
  const seam = normalizeSeamResult(seamWire());
  assert.deepEqual(seam.seam.localPointsM[1], [1, 0, 0]);
  assert.deepEqual(seam.seam.startPointM, [2, 0, 0]);
  assert.equal(seam.seam.lengthM, 1);
  assert.equal(seam.environment?.collisionObjects[0]?.sizeM[0], 0.5);
  assert.equal(seam.environment?.constraints.collisionMarginM, 0.025);
  assert.deepEqual(seam.tesseractPlan?.tcpPoses[0]?.targetM, [2, 0, 0]);
  assert.equal(seam.tesseractPlan?.motionPlan?.weldLengthM, 1);
  assert.equal(seam.tesseractPlan?.motionPlan?.weldFeedMps, 0.012);
  assert.equal(seam.tesseractPlan?.motionPlan?.trajectory[0]?.collisionContacts[0]?.depthM, 0.002);
});

test("normalizes multi-seam sequence contracts", () => {
  const wire: CadSequenceResultWire = {
    ...commonWire(),
    weld_sequence: {
      schema: "cadquery.v1.weld-sequence",
      robot: "rosie_1400_v3",
      frame: "world",
      seams: [{
        id: "seam_01",
        edge_id: "edge-a",
        part_id: "part_0",
        geom_type: "WIRE",
        start_s: 0.2,
        end_s: 0.8,
        start_point_mm: [100, 200, 300],
        stop_point_mm: [400, 500, 600],
        point_count: 24,
      }],
    },
  };
  const sequence = normalizeSequenceResult(wire);
  assert.deepEqual(sequence.weldSequence.seams[0]?.startPointM, [0.1, 0.2, 0.3]);
  assert.deepEqual(sequence.weldSequence.seams[0]?.window, { start: 0.2, end: 0.8 });
});

test("HTTP client emits worker operation and generic fixture ID", async () => {
  const bodies: Array<Record<string, unknown>> = [];
  const client = new HttpCadClient({
    fetch: async (_input, init) => {
      bodies.push(JSON.parse(String(init?.body)) as Record<string, unknown>);
      return Response.json(topologyWire());
    },
  });
  const topology = await client.extractTopology({ source: DOG_BOX_FIXTURE, sampleCount: 32 });
  assert.equal(bodies[0]?.operation, "extract_topology");
  assert.equal(bodies[0]?.fixture_id, "dog_box");
  assert.equal(bodies[0]?.step_base64, undefined);
  assert.equal(topology.fingerprint, FINGERPRINT);
});

test("HTTP client base64-encodes uploaded STEP bytes", async () => {
  const bodies: Array<Record<string, unknown>> = [];
  const client = new HttpCadClient({
    fetch: async (_input, init) => {
      bodies.push(JSON.parse(String(init?.body)) as Record<string, unknown>);
      return Response.json(topologyWire());
    },
  });
  await client.extractTopology({
    source: {
      kind: "bytes",
      filename: "part.step",
      bytes: new Uint8Array([0x53, 0x54, 0x45, 0x50]),
    },
  });
  assert.equal(bodies[0]?.step_base64, "U1RFUA==");
});

test("HTTP client exposes retryable busy state and preserves typed state errors", async () => {
  const client = new HttpCadClient({
    fetch: async () => Response.json(
      { ok: false, error: "CadQuery worker is busy" },
      { status: 429 },
    ),
  });
  await assert.rejects(
    client.extractTopology({ source: DOG_BOX_FIXTURE }),
    (error: unknown) => {
      assert.ok(error instanceof CadClientError);
      assert.equal(error.code, "busy");
      assert.equal(error.status, 429);
      assert.equal(error.retryable, true);
      const state = failedCadState(error);
      assert.equal(state.status, "error");
      assert.equal(state.error, error);
      return true;
    },
  );
  assert.equal(readyCadState({ ok: true }).status, "ready");
});

test("HTTP client distinguishes timeout from caller cancellation", async () => {
  const fetchUntilAbort: typeof fetch = async (_input, init) =>
    new Promise<Response>((_resolve, reject) => {
      init?.signal?.addEventListener("abort", () => reject(new DOMException("aborted", "AbortError")), { once: true });
    });
  const timeoutClient = new HttpCadClient({ fetch: fetchUntilAbort, timeoutMS: 5 });
  await assert.rejects(
    timeoutClient.extractTopology({ source: DOG_BOX_FIXTURE }),
    (error: unknown) => error instanceof CadClientError && error.code === "timeout",
  );

  const controller = new AbortController();
  const canceled = new HttpCadClient({ fetch: fetchUntilAbort, timeoutMS: 1_000 })
    .extractTopology({ source: DOG_BOX_FIXTURE }, controller.signal);
  controller.abort();
  await assert.rejects(
    canceled,
    (error: unknown) => error instanceof CadClientError && error.code === "aborted",
  );
});

test("normalizers fail closed on unsupported units", () => {
  assert.throws(
    () => normalizeTopology({ ...topologyWire(), unit: "inch" }),
    (error: unknown) => error instanceof CadClientError && error.code === "invalid_response",
  );
  assert.throws(
    () => normalizeWorkpieceMesh({ ...meshWire(), unit: "inch" }),
    (error: unknown) => error instanceof CadClientError && error.code === "invalid_response",
  );
});

test("normalizes the workpiece mesh into metre typed arrays", () => {
  const mesh = normalizeWorkpieceMesh(meshWire());
  assert.equal(mesh.unit, "m");
  assert.equal(mesh.sourceUnit, "mm");
  assert.ok(mesh.positionsM instanceof Float32Array);
  assert.ok(mesh.indices instanceof Uint32Array);
  // 1000 mm on the wire is 1 m in the app, like every other CAD dimension.
  assert.equal(mesh.positionsM[3], 1);
  assert.equal(mesh.vertexCount, 4);
  assert.equal(mesh.triangleCount, 2);
  assert.deepEqual(mesh.boundsMinM, [0, 0, 0]);
  assert.deepEqual(mesh.boundsMaxM, [1, 1, 0]);
  assert.equal(mesh.linearDeflectionM, 0.0005);
  assert.equal(mesh.angularDeflectionRad, 0.5);
  assert.equal(mesh.solidCount, 1);
  assert.equal(mesh.truncated, false);
  // Absent normals are reported as absent so the viewer derives them.
  assert.equal(mesh.normals, null);
});

test("workpiece mesh normals are kept unscaled when the worker supplies them", () => {
  const mesh = normalizeWorkpieceMesh({
    ...meshWire(),
    mesh: { ...meshWire().mesh, normals: [0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1] },
  });
  assert.ok(mesh.normals);
  // Unit vectors must not be multiplied by the millimetre scale factor.
  assert.equal(mesh.normals[2], 1);
});

test("workpiece mesh rejects structurally broken triangle data", () => {
  const invalidResponse = (value: unknown) => {
    assert.throws(
      () => normalizeWorkpieceMesh(value),
      (error: unknown) => error instanceof CadClientError && error.code === "invalid_response",
    );
  };
  const base = meshWire();
  invalidResponse({ ...base, mesh: { ...base.mesh, positions: [] } });
  invalidResponse({ ...base, mesh: { ...base.mesh, positions: [0, 0] } });
  invalidResponse({ ...base, mesh: { ...base.mesh, indices: [] } });
  invalidResponse({ ...base, mesh: { ...base.mesh, indices: [0, 1] } });
  // An index past the last vertex silently corrupts the whole draw call.
  invalidResponse({ ...base, mesh: { ...base.mesh, indices: [0, 1, 9] } });
  invalidResponse({ ...base, mesh: { ...base.mesh, indices: [0, 1, -1] } });
  invalidResponse({ ...base, mesh: { ...base.mesh, indices: [0, 1, 1.5] } });
  invalidResponse({ ...base, mesh: { ...base.mesh, normals: [0, 0, 1] } });
  invalidResponse({ ...base, mesh: { ...base.mesh, positions: [0, 0, 0, "x", 0, 0, 0, 0, 0, 0, 0, 0] } });
});

test("HTTP client sends bounded tessellation options and its own longer timeout", async () => {
  const bodies: Array<Record<string, unknown>> = [];
  const client = new HttpCadClient({
    fetch: async (_input, init) => {
      bodies.push(JSON.parse(String(init?.body)) as Record<string, unknown>);
      return Response.json(meshWire());
    },
  });
  const mesh = await client.tessellate({ source: DOG_BOX_FIXTURE });
  assert.equal(bodies[0]?.operation, "tessellate");
  assert.equal(bodies[0]?.fixture_id, "dog_box");
  assert.equal(bodies[0]?.linear_deflection_mm, 0.5);
  assert.equal(bodies[0]?.angular_deflection_rad, 0.5);
  assert.equal(bodies[0]?.max_triangles, 400_000);
  assert.equal(mesh.fingerprint, FINGERPRINT);
  // Tessellation outlasts the 95 s sampled-topology ceiling on real assemblies.
  assert.ok(client.meshTimeoutMS > client.timeoutMS);

  for (
    const request of [
      { source: DOG_BOX_FIXTURE, linearDeflectionMM: 0 },
      { source: DOG_BOX_FIXTURE, linearDeflectionMM: 50 },
      { source: DOG_BOX_FIXTURE, angularDeflectionRad: 0 },
      { source: DOG_BOX_FIXTURE, angularDeflectionRad: 5 },
      { source: DOG_BOX_FIXTURE, maxTriangles: 10 },
      { source: DOG_BOX_FIXTURE, maxTriangles: 5_000_000 },
      { source: DOG_BOX_FIXTURE, maxTriangles: 1.5 },
    ]
  ) {
    await assert.rejects(
      client.tessellate(request),
      (error: unknown) => error instanceof CadClientError && error.code === "invalid_request",
    );
  }
});

test("HTTP client maps an unavailable CAD service to a typed mesh failure", async () => {
  const client = new HttpCadClient({
    fetch: async () => Response.json(
      { ok: false, code: "unavailable", error: "CadQuery worker unavailable" },
      { status: 503 },
    ),
  });
  await assert.rejects(
    client.tessellate({ source: DOG_BOX_FIXTURE }),
    (error: unknown) => {
      assert.ok(error instanceof CadClientError);
      assert.equal(error.code, "unavailable");
      assert.equal(error.operation, "tessellate");
      return true;
    },
  );
});

function meshWire(): CadWorkpieceMeshWire {
  return {
    ...commonWire(),
    mesh: {
      positions: [0, 0, 0, 1000, 0, 0, 1000, 1000, 0, 0, 1000, 0],
      indices: [0, 1, 2, 0, 2, 3],
      vertex_count: 4,
      triangle_count: 2,
      bounds_min: [0, 0, 0],
      bounds_max: [1000, 1000, 0],
      linear_deflection_mm: 0.5,
      angular_deflection_rad: 0.5,
      solid_count: 1,
      truncated: false,
    },
  };
}

function commonWire() {
  return {
    schema_version: 1,
    tool: "test",
    model_id: "cadquery-step-a",
    filename: "part.step",
    fingerprint: FINGERPRINT,
    unit: "mm",
  };
}

function topologyWire(): CadTopologyWire {
  return {
    ...commonWire(),
    sample_count: 32,
    parts: [],
    edges: [],
  };
}

function seamWire() {
  return {
    ...commonWire(),
    placement: {
      frame: "world",
      origin_mm: [1000, 2000, 3000],
    },
    environment: {
      urdf: "robot.urdf",
      workcell_frame: "world",
      manipulator: "rosie_1400_v3",
      tcp_frame: "weld_torch_tcp",
      tcp_offset_mm: [0, 0, 10],
      positioner_axis: "J7a",
      collision_objects: [{
        name: "workpiece",
        type: "box",
        frame: "world",
        parent_link: "table",
        world_pose_xyz_mm: [2000, 0, 0],
        size_mm: [500, 600, 700],
      }],
      constraints: { collision_margin_mm: 25 },
    },
    seam: {
      edge_id: "edge-a",
      part_id: "part_0",
      geom_type: "LINE",
      start_s: 0,
      end_s: 1,
      reversed: false,
      length_mm: 1000,
      point_count: 2,
      points: [[0, 0, 0], [1000, 0, 0]],
      world_points_mm: [[2000, 0, 0], [3000, 0, 0]],
      start_point_mm: [2000, 0, 0],
      stop_point_mm: [3000, 0, 0],
    },
    weld_sequence: {
      schema: "cadquery.v1.weld-sequence",
      robot: "rosie_1400_v3",
      frame: "world",
      seams: [{
        id: "seam_01",
        edge_id: "edge-a",
        start_point_mm: [2000, 0, 0],
        stop_point_mm: [3000, 0, 0],
        point_count: 2,
      }],
    },
    tesseract_plan: {
      schema: "cadquery.v1.tesseract-weld-plan",
      planner: "tesseract",
      robot: "rosie_1400_v3",
      working_frame: "world",
      tcp_poses: [{
        index: 0,
        time_s: 0,
        target_xyz_mm: [2000, 0, 0],
        seam_tangent: [1, 0, 0],
        seam_plane_normal: [0, 0, 1],
        seam_side_axis: [0, 1, 0],
        tool_x_axis: [1, 0, 0],
        tool_y_axis: [0, 1, 0],
        tool_z_axis: [0, 0, 1],
        orientation_rpy_rad: [0, 0, 0],
        work_angle_deg: 45,
        angle_from_seam_plane_deg: 45,
        tool_x_tangent_alignment: 1,
      }],
      motion_plan: {
        trajectory: [{
          index: 0,
          seam_id: "seam_01",
          move: "weld",
          weld_on: true,
          time_s: 0,
          target_xyz_mm: [2000, 0, 0],
          collision_contacts: [{ link_a: "tool", link_b: "part", depth_mm: 2 }],
        }],
        sample_count: 1,
        weld_sample_count: 1,
        duration_s: 1,
        tool_path_length_mm: 1000,
        weld_length_mm: 1000,
        travel_length_mm: 0,
        max_tcp_step_mm: 10,
        max_frame_rotation_deg: 2,
        weld_feed_mm_s: 12,
        travel_feed_mm_s: 90,
      },
    },
  };
}
