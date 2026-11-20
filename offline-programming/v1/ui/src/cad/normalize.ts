import {
  CadClientError,
  type CadOperation,
  type CadCollisionSummary,
  type CadEnvironment,
  type CadGeometryMethod,
  type CadIKSummary,
  type CadMotionPlan,
  type CadMotionSample,
  type CadPathMethod,
  type CadPlacement,
  type CadPlanStep,
  type CadRecommendedSeam,
  type CadSeamResult,
  type CadSequenceResult,
  type CadSequenceSeam,
  type CadTCPFrame,
  type CadTesseractPlan,
  type CadTopology,
  type CadTopologyEdge,
  type CadWeldSequence,
  type CadWorkpieceMesh,
  type Quaternion,
  type Vec3,
} from "./contracts.ts";

const MM_TO_M = 0.001;

export function normalizeTopology(value: unknown): CadTopology {
  const root = commonRoot(value, "extract_topology");
  const edges = array(root.edges, "edges").map((edge, index) => topologyEdge(edge, `edges[${index}]`));
  return {
    schemaVersion: 1,
    unit: "m",
    sourceUnit: "mm",
    tool: text(root.tool, "tool"),
    modelId: text(root.model_id, "model_id"),
    filename: text(root.filename, "filename"),
    fingerprint: fingerprint(root.fingerprint),
    sampleCount: integer(root.sample_count, "sample_count"),
    parts: array(root.parts, "parts").map((part, index) => {
      const item = record(part, `parts[${index}]`);
      return {
        id: text(item.id, `parts[${index}].id`),
        edgeCount: integer(item.edge_count, `parts[${index}].edge_count`),
      };
    }),
    edges,
    rawEdgeCount: optionalInteger(root.raw_edge_count, edges.filter((edge) => edge.sourceEdgeIds.length === 0).length),
    compositeEdgeCount: optionalInteger(root.composite_edge_count, edges.filter((edge) => edge.sourceEdgeIds.length > 0).length),
    seamCandidateCount: optionalInteger(root.seam_candidate_count, edges.filter((edge) => edge.isSeamCandidate).length),
    recommendedSeams: optionalArray(root.recommended_seams).map((item, index) =>
      recommendedSeam(item, `recommended_seams[${index}]`)),
  };
}

/**
 * Tessellated surface, millimetres to metres. Positions and normals become
 * typed arrays so the viewer can hand them straight to a BufferGeometry without
 * a second copy; indices are validated against the vertex count because an
 * out-of-range index silently corrupts the whole draw call.
 */
export function normalizeWorkpieceMesh(value: unknown): CadWorkpieceMesh {
  const root = commonRoot(value, "tessellate");
  const mesh = record(root.mesh, "mesh");
  const rawPositions = array(mesh.positions, "mesh.positions");
  const rawIndices = array(mesh.indices, "mesh.indices");
  if (rawPositions.length === 0 || rawPositions.length % 3 !== 0) {
    invalid("mesh.positions must be a non-empty flat XYZ array", "tessellate");
  }
  if (rawIndices.length === 0 || rawIndices.length % 3 !== 0) {
    invalid("mesh.indices must be a non-empty flat triangle array", "tessellate");
  }
  const vertexCount = rawPositions.length / 3;
  const positionsM = new Float32Array(rawPositions.length);
  for (let index = 0; index < rawPositions.length; index += 1) {
    positionsM[index] = finite(rawPositions[index], `mesh.positions[${index}]`) * MM_TO_M;
  }
  const indices = new Uint32Array(rawIndices.length);
  for (let index = 0; index < rawIndices.length; index += 1) {
    const vertex = finite(rawIndices[index], `mesh.indices[${index}]`);
    if (!Number.isInteger(vertex) || vertex < 0 || vertex >= vertexCount) {
      invalid(`mesh.indices[${index}] is outside the vertex range`, "tessellate");
    }
    indices[index] = vertex;
  }
  const rawNormals = optionalArray(mesh.normals);
  let normals: Float32Array | null = null;
  if (rawNormals.length > 0) {
    if (rawNormals.length !== rawPositions.length) {
      invalid("mesh.normals must match mesh.positions in length", "tessellate");
    }
    normals = new Float32Array(rawNormals.length);
    for (let index = 0; index < rawNormals.length; index += 1) {
      normals[index] = finite(rawNormals[index], `mesh.normals[${index}]`);
    }
  }
  return {
    schemaVersion: 1,
    unit: "m",
    sourceUnit: "mm",
    tool: text(root.tool, "tool"),
    modelId: text(root.model_id, "model_id"),
    filename: text(root.filename, "filename"),
    fingerprint: fingerprint(root.fingerprint),
    positionsM,
    indices,
    normals,
    vertexCount,
    triangleCount: rawIndices.length / 3,
    boundsMinM: optionalVec3(mesh.bounds_min, [0, 0, 0], MM_TO_M),
    boundsMaxM: optionalVec3(mesh.bounds_max, [0, 0, 0], MM_TO_M),
    linearDeflectionM: optionalFinite(mesh.linear_deflection_mm, 0, MM_TO_M) ?? 0,
    angularDeflectionRad: optionalFinite(mesh.angular_deflection_rad, 0) ?? 0,
    solidCount: optionalInteger(mesh.solid_count, 0),
    truncated: boolean(mesh.truncated, false),
  };
}

export function normalizeSeamResult(value: unknown): CadSeamResult {
  const root = commonRoot(value, "build_seam");
  const seam = record(root.seam, "seam");
  const localPoints = vec3Array(seam.points, "seam.points", MM_TO_M);
  const worldPoints = vec3Array(seam.world_points_mm ?? seam.points, "seam.world_points_mm", MM_TO_M);
  return {
    schemaVersion: 1,
    unit: "m",
    sourceUnit: "mm",
    tool: text(root.tool, "tool"),
    modelId: text(root.model_id, "model_id"),
    filename: text(root.filename, "filename"),
    fingerprint: fingerprint(root.fingerprint),
    placement: placement(root.placement),
    environment: environment(root.environment),
    seam: {
      edgeId: text(seam.edge_id, "seam.edge_id"),
      partId: optionalText(seam.part_id),
      geometryType: optionalText(seam.geom_type) ?? "EDGE",
      window: {
        start: finite(seam.start_s, "seam.start_s"),
        end: finite(seam.end_s, "seam.end_s"),
      },
      reversed: boolean(seam.reversed, false),
      lengthM: optionalFinite(seam.length_mm, null, MM_TO_M),
      pointCount: optionalInteger(seam.point_count, localPoints.length),
      localPointsM: localPoints,
      localControlPointsM: vec3Array(seam.control_points ?? [], "seam.control_points", MM_TO_M),
      worldPointsM: worldPoints,
      worldControlPointsM: vec3Array(
        seam.world_control_points_mm ?? seam.control_points ?? [],
        "seam.world_control_points_mm",
        MM_TO_M,
      ),
      startPointM: optionalVec3(seam.start_point_mm, worldPoints[0] ?? localPoints[0] ?? [0, 0, 0], MM_TO_M),
      stopPointM: optionalVec3(
        seam.stop_point_mm,
        worldPoints.at(-1) ?? localPoints.at(-1) ?? [0, 0, 0],
        MM_TO_M,
      ),
      positionerAxis: optionalText(seam.positioner_axis),
    },
    weldSequence: weldSequence(root.weld_sequence),
    tesseractPlan: tesseractPlan(root.tesseract_plan),
  };
}

export function normalizeSequenceResult(value: unknown): CadSequenceResult {
  const root = commonRoot(value, "build_sequence");
  return {
    schemaVersion: 1,
    unit: "m",
    sourceUnit: "mm",
    tool: text(root.tool, "tool"),
    modelId: text(root.model_id, "model_id"),
    filename: text(root.filename, "filename"),
    fingerprint: fingerprint(root.fingerprint),
    placement: placement(root.placement),
    environment: environment(root.environment),
    weldSequence: weldSequence(root.weld_sequence),
    tesseractPlan: tesseractPlan(root.tesseract_plan),
  };
}

function commonRoot(
  value: unknown,
  operation: CadOperation,
): Record<string, unknown> {
  const root = record(value, "response");
  if (integer(root.schema_version, "schema_version") !== 1) {
    invalid("unsupported CAD schema_version", operation);
  }
  if (root.unit !== "mm") {
    invalid(`unsupported CAD unit ${String(root.unit)}`, operation);
  }
  return root;
}

function topologyEdge(value: unknown, path: string): CadTopologyEdge {
  const edge = record(value, path);
  const samples = vec3Array(edge.samples, `${path}.samples`, MM_TO_M);
  const fallback = samples[0] ?? [0, 0, 0];
  return {
    id: text(edge.id, `${path}.id`),
    label: optionalText(edge.label),
    partId: optionalText(edge.part_id),
    geometryType: optionalText(edge.geom_type) ?? "EDGE",
    sampleCount: optionalInteger(edge.sample_count, samples.length),
    samplesM: samples,
    lengthM: optionalFinite(edge.length_mm, 0, MM_TO_M) ?? 0,
    boundsMinM: optionalVec3(edge.bounds_min, fallback, MM_TO_M),
    boundsMaxM: optionalVec3(edge.bounds_max, samples.at(-1) ?? fallback, MM_TO_M),
    closed: boolean(edge.closed, false),
    sourceEdgeIds: stringArray(edge.source_edge_ids),
    sourceGeometryTypes: stringArray(edge.source_geom_types),
    pathClass: optionalText(edge.path_class) ?? "raw_fragment",
    isSeamCandidate: boolean(edge.is_seam_candidate, false),
  };
}

function recommendedSeam(value: unknown, path: string): CadRecommendedSeam {
  const seam = record(value, path);
  return {
    edgeId: text(seam.edge_id, `${path}.edge_id`),
    partId: optionalText(seam.part_id),
    geometryType: optionalText(seam.geom_type) ?? "EDGE",
    pathClass: optionalText(seam.path_class) ?? "raw_fragment",
    window: {
      start: finite(seam.start_s, `${path}.start_s`),
      end: finite(seam.end_s, `${path}.end_s`),
    },
    reversed: boolean(seam.reversed, false),
    reason: optionalText(seam.reason) ?? "",
  };
}

function placement(value: unknown): CadPlacement | null {
  if (value === undefined || value === null) return null;
  const item = record(value, "placement");
  return {
    robot: optionalText(item.robot),
    axis: optionalText(item.axis),
    table: optionalText(item.table),
    frame: optionalText(item.frame) ?? "world",
    axisOriginM: nullableVec3(item.axis_origin_mm, MM_TO_M),
    originM: nullableVec3(item.origin_mm, MM_TO_M),
    placementNote: optionalText(item.placement_note),
  };
}

function environment(value: unknown): CadEnvironment | null {
  if (value === undefined || value === null) return null;
  const item = record(value, "environment");
  const constraintsRecord = optionalRecord(item.constraints);
  const additional: Record<string, string | boolean> = {};
  for (const [key, raw] of Object.entries(constraintsRecord)) {
    if (["tool_x_axis", "tool_y_axis", "tool_z_axis", "tcp_position", "collision_margin_mm"].includes(key)) {
      continue;
    }
    if (typeof raw === "string" || typeof raw === "boolean") {
      additional[key] = raw;
    }
  }
  return {
    urdf: optionalText(item.urdf) ?? "",
    workcellFrame: optionalText(item.workcell_frame) ?? "world",
    manipulator: optionalText(item.manipulator) ?? "",
    tcpFrame: optionalText(item.tcp_frame) ?? "",
    tcpOffsetM: optionalVec3(item.tcp_offset_mm, [0, 0, 0], MM_TO_M),
    positionerAxis: optionalText(item.positioner_axis) ?? "",
    collisionObjects: optionalArray(item.collision_objects).map((raw, index) => {
      const object = record(raw, `environment.collision_objects[${index}]`);
      return {
        name: text(object.name, `environment.collision_objects[${index}].name`),
        type: optionalText(object.type) ?? "",
        frame: optionalText(object.frame) ?? "world",
        parentLink: optionalText(object.parent_link) ?? "",
        worldPositionM: optionalVec3(object.world_pose_xyz_mm, [0, 0, 0], MM_TO_M),
        sizeM: optionalVec3(object.size_mm, [0, 0, 0], MM_TO_M),
        note: optionalText(object.note),
      };
    }),
    constraints: {
      toolXAxis: optionalText(constraintsRecord.tool_x_axis),
      toolYAxis: optionalText(constraintsRecord.tool_y_axis),
      toolZAxis: optionalText(constraintsRecord.tool_z_axis),
      tcpPosition: optionalText(constraintsRecord.tcp_position),
      collisionMarginM: optionalFinite(constraintsRecord.collision_margin_mm, null, MM_TO_M),
      additional,
    },
  };
}

function geometryMethod(value: unknown): CadGeometryMethod | null {
  if (value === undefined || value === null) return null;
  const method = record(value, "geometry_method");
  return {
    schema: optionalText(method.schema) ?? "",
    selected: optionalText(method.selected) ?? "",
    selectionReason: optionalText(method.selection_reason) ?? "",
    seamCenterM: optionalVec3(method.seam_center_mm, [0, 0, 0], MM_TO_M),
    partCenterM: optionalVec3(method.part_center_mm, [0, 0, 0], MM_TO_M),
    candidates: optionalArray(method.candidates).map((raw, index) => {
      const candidate = record(raw, `geometry_method.candidates[${index}]`);
      return {
        id: text(candidate.id, `geometry_method.candidates[${index}].id`),
        score: optionalFinite(candidate.score, 0) ?? 0,
        awayFromPart: optionalFinite(candidate.away_from_part, 0) ?? 0,
        upward: optionalFinite(candidate.upward, 0) ?? 0,
        toolXTangentAlignment: optionalFinite(candidate.tool_x_tangent_alignment, 0) ?? 0,
        angleFromSeamPlaneDeg: optionalFinite(candidate.angle_from_seam_plane_deg, 0) ?? 0,
        maxFrameRotationDeg: optionalFinite(candidate.max_frame_rotation_deg, 0) ?? 0,
        seamPlaneNormal: optionalVec3(candidate.seam_plane_normal, [0, 0, 1]),
        toolZAxis: optionalVec3(candidate.tool_z_axis, [0, 0, 1]),
      };
    }),
  };
}

function weldSequence(value: unknown): CadWeldSequence {
  const sequence = record(value, "weld_sequence");
  return {
    schema: optionalText(sequence.schema) ?? "",
    robot: optionalText(sequence.robot) ?? "",
    frame: optionalText(sequence.frame) ?? "world",
    positionerAxis: optionalText(sequence.positioner_axis),
    seams: optionalArray(sequence.seams).map((raw, index) => sequenceSeam(raw, index)),
  };
}

function sequenceSeam(value: unknown, index: number): CadSequenceSeam {
  const path = `weld_sequence.seams[${index}]`;
  const seam = record(value, path);
  return {
    id: text(seam.id, `${path}.id`),
    edgeId: text(seam.edge_id, `${path}.edge_id`),
    partId: optionalText(seam.part_id),
    geometryType: optionalText(seam.geom_type) ?? "EDGE",
    window: {
      start: optionalFinite(seam.start_s, 0) ?? 0,
      end: optionalFinite(seam.end_s, 1) ?? 1,
    },
    reversed: boolean(seam.reversed, false),
    startPointM: optionalVec3(seam.start_point_mm, [0, 0, 0], MM_TO_M),
    stopPointM: optionalVec3(seam.stop_point_mm, [0, 0, 0], MM_TO_M),
    pointCount: optionalInteger(seam.point_count, 0),
    geometryMethod: geometryMethod(seam.geometry_method),
  };
}

function tesseractPlan(value: unknown): CadTesseractPlan | null {
  if (value === undefined || value === null) return null;
  const plan = record(value, "tesseract_plan");
  const singleMethod = geometryMethod(plan.geometry_method);
  return {
    schema: optionalText(plan.schema) ?? "",
    planner: optionalText(plan.planner) ?? "tesseract",
    robot: optionalText(plan.robot) ?? "",
    workingFrame: optionalText(plan.working_frame) ?? "world",
    edgeId: optionalText(plan.edge_id),
    environment: environment(plan.environment),
    geometryMethod: singleMethod,
    geometryMethods: optionalArray(plan.geometry_methods)
      .map((method) => geometryMethod(method))
      .filter((method): method is CadGeometryMethod => method !== null),
    pathMethod: pathMethod(plan.path_method),
    sequence: optionalArray(plan.sequence).map((step, index) => planStep(step, index)),
    tcpPoses: optionalArray(plan.tcp_poses).map((pose, index) => tcpFrame(pose, `tesseract_plan.tcp_poses[${index}]`)),
    motionPlan: motionPlan(plan.motion_plan),
  };
}

function pathMethod(value: unknown): CadPathMethod | null {
  if (value === undefined || value === null) return null;
  const method = record(value, "tesseract_plan.path_method");
  return {
    selected: optionalText(method.selected) ?? "",
    supported: stringArray(method.supported),
    fallbackReason: optionalText(method.fallback_reason),
    controlPoseCount: optionalInteger(method.control_pose_count, 0),
    controlPointsM: vec3Array(method.control_points ?? [], "path_method.control_points", MM_TO_M),
  };
}

function planStep(value: unknown, index: number): CadPlanStep {
  const path = `tesseract_plan.sequence[${index}]`;
  const step = record(value, path);
  return {
    kind: text(step.kind, `${path}.kind`),
    seamId: optionalText(step.seam_id),
    targetM: position(step, path),
  };
}

function tcpFrame(value: unknown, path: string): CadTCPFrame {
  const pose = record(value, path);
  return {
    index: optionalInteger(pose.index, 0),
    seamId: optionalText(pose.seam_id),
    timeS: optionalFinite(pose.time_s, 0) ?? 0,
    targetM: position(pose, path),
    seamTangent: optionalVec3(pose.seam_tangent, [1, 0, 0]),
    seamPlaneNormal: optionalVec3(pose.seam_plane_normal, [0, 0, 1]),
    seamSideAxis: optionalVec3(pose.seam_side_axis, [0, 1, 0]),
    toolXAxis: optionalVec3(pose.tool_x_axis, [1, 0, 0]),
    toolYAxis: optionalVec3(pose.tool_y_axis, [0, 1, 0]),
    toolZAxis: optionalVec3(pose.tool_z_axis, [0, 0, 1]),
    toolQuaternionXYZW: nullableQuaternion(pose.tool_quat_xyzw),
    orientationRpyRad: optionalVec3(pose.orientation_rpy_rad, [0, 0, 0]),
    workAngleDeg: optionalFinite(pose.work_angle_deg, 0) ?? 0,
    angleFromSeamPlaneDeg: optionalFinite(pose.angle_from_seam_plane_deg, 0) ?? 0,
    toolXTangentAlignment: optionalFinite(pose.tool_x_tangent_alignment, 0) ?? 0,
  };
}

function motionPlan(value: unknown): CadMotionPlan | null {
  if (value === undefined || value === null) return null;
  const plan = record(value, "motion_plan");
  return {
    trajectory: optionalArray(plan.trajectory).map((sample, index) => motionSample(sample, index)),
    sampleCount: optionalInteger(plan.sample_count, 0),
    weldSampleCount: optionalInteger(plan.weld_sample_count, 0),
    durationS: optionalFinite(plan.duration_s, 0) ?? 0,
    toolPathLengthM: optionalFinite(plan.tool_path_length_mm, 0, MM_TO_M) ?? 0,
    weldLengthM: optionalFinite(plan.weld_length_mm, 0, MM_TO_M) ?? 0,
    travelLengthM: optionalFinite(plan.travel_length_mm, 0, MM_TO_M) ?? 0,
    maxTCPStepM: optionalFinite(plan.max_tcp_step_mm, 0, MM_TO_M) ?? 0,
    maxFrameRotationDeg: optionalFinite(plan.max_frame_rotation_deg, 0) ?? 0,
    perSeamFrameRotationDeg: numberArray(plan.per_seam_frame_rotation_deg),
    weldFeedMps: optionalFinite(plan.weld_feed_mm_s, 0, MM_TO_M) ?? 0,
    travelFeedMps: optionalFinite(plan.travel_feed_mm_s, 0, MM_TO_M) ?? 0,
    ik: ikSummary(plan.ik),
    collision: collisionSummary(plan.collision),
  };
}

function motionSample(value: unknown, index: number): CadMotionSample {
  const path = `motion_plan.trajectory[${index}]`;
  const sample = record(value, path);
  const frame = tcpFrame(sample, path);
  const rawMove = optionalText(sample.move) ?? "travel";
  const move = ["approach", "weld", "retract", "travel"].includes(rawMove)
    ? rawMove as CadMotionSample["move"]
    : "travel";
  return {
    ...frame,
    move,
    weldOn: boolean(sample.weld_on, false),
    jointNames: stringArray(sample.joint_names),
    jointPositionsRad: numberArray(sample.joint_positions_rad),
    ikPositionErrorM: optionalFinite(sample.ik_position_error_mm, null, MM_TO_M),
    ikRotationErrorDeg: optionalFinite(sample.ik_rotation_error_deg, null),
    collisionContacts: optionalArray(sample.collision_contacts).map((raw, contactIndex) => {
      const contact = record(raw, `${path}.collision_contacts[${contactIndex}]`);
      return {
        linkA: text(contact.link_a, `${path}.collision_contacts[${contactIndex}].link_a`),
        linkB: text(contact.link_b, `${path}.collision_contacts[${contactIndex}].link_b`),
        depthM: optionalFinite(contact.depth_mm, 0, MM_TO_M) ?? 0,
      };
    }),
  };
}

function ikSummary(value: unknown): CadIKSummary | null {
  if (value === undefined || value === null) return null;
  const summary = record(value, "motion_plan.ik");
  return {
    solver: optionalText(summary.solver) ?? "",
    robot: optionalText(summary.robot) ?? "",
    tcpLink: optionalText(summary.tcp_link) ?? "",
    jointNames: stringArray(summary.joint_names),
    maxPositionErrorM: optionalFinite(summary.max_position_error_mm, 0, MM_TO_M) ?? 0,
    meanPositionErrorM: optionalFinite(summary.mean_position_error_mm, 0, MM_TO_M) ?? 0,
    maxRotationErrorDeg: optionalFinite(summary.max_rotation_error_deg, 0) ?? 0,
    maxJointStepRad: optionalFinite(summary.max_joint_step_rad, 0) ?? 0,
  };
}

function collisionSummary(value: unknown): CadCollisionSummary | null {
  if (value === undefined || value === null) return null;
  const summary = record(value, "motion_plan.collision");
  const rawStatus = optionalText(summary.status) ?? "unavailable";
  const status = ["checked", "unavailable", "skipped"].includes(rawStatus)
    ? rawStatus as CadCollisionSummary["status"]
    : "unavailable";
  return {
    status,
    reason: optionalText(summary.reason),
    checker: optionalText(summary.checker),
    checkedWaypointCount: optionalInteger(summary.checked_waypoint_count, 0),
    contactCount: optionalInteger(summary.contact_count, 0),
    contactSampleCount: optionalInteger(summary.contact_sample_count, 0),
    maxDepthM: optionalFinite(summary.max_depth_mm, 0, MM_TO_M) ?? 0,
    contactPairs: stringArray(summary.contact_pairs),
    checkDurationS: optionalFinite(summary.check_duration_s, 0) ?? 0,
  };
}

function position(value: Record<string, unknown>, path: string): Vec3 {
  if (value.target_xyz_m !== undefined) {
    return optionalVec3(value.target_xyz_m, [0, 0, 0]);
  }
  return optionalVec3(value.target_xyz_mm, [0, 0, 0], MM_TO_M);
}

function record(value: unknown, path: string): Record<string, unknown> {
  if (typeof value !== "object" || value === null || Array.isArray(value)) {
    invalid(`${path} must be an object`);
  }
  return value as Record<string, unknown>;
}

function optionalRecord(value: unknown): Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value)
    ? value as Record<string, unknown>
    : {};
}

function array(value: unknown, path: string): unknown[] {
  if (!Array.isArray(value)) invalid(`${path} must be an array`);
  return value as unknown[];
}

function optionalArray(value: unknown): unknown[] {
  return Array.isArray(value) ? value : [];
}

function vec3Array(value: unknown, path: string, scale = 1): Vec3[] {
  return array(value, path).map((point, index) => vec3(point, `${path}[${index}]`, scale));
}

function vec3(value: unknown, path: string, scale = 1): Vec3 {
  if (!Array.isArray(value) || value.length !== 3) invalid(`${path} must be a three-number tuple`);
  const tuple = value as unknown[];
  return [
    finite(tuple[0], `${path}[0]`) * scale,
    finite(tuple[1], `${path}[1]`) * scale,
    finite(tuple[2], `${path}[2]`) * scale,
  ];
}

function optionalVec3(value: unknown, fallback: Vec3, scale = 1): Vec3 {
  return value === undefined || value === null
    ? [...fallback]
    : vec3(value, "vector", scale);
}

function nullableVec3(value: unknown, scale = 1): Vec3 | null {
  return value === undefined || value === null ? null : vec3(value, "vector", scale);
}

function nullableQuaternion(value: unknown): Quaternion | null {
  if (value === undefined || value === null) return null;
  if (!Array.isArray(value) || value.length !== 4) invalid("quaternion must be a four-number tuple");
  return [
    finite(value[0], "quaternion[0]"),
    finite(value[1], "quaternion[1]"),
    finite(value[2], "quaternion[2]"),
    finite(value[3], "quaternion[3]"),
  ];
}

function text(value: unknown, path: string): string {
  if (typeof value !== "string" || value.trim() === "") invalid(`${path} must be a non-empty string`);
  return (value as string).trim();
}

function optionalText(value: unknown): string | null {
  return typeof value === "string" && value.trim() !== "" ? value.trim() : null;
}

function fingerprint(value: unknown): string {
  const result = text(value, "fingerprint").toLowerCase();
  if (!/^[a-f0-9]{64}$/.test(result)) invalid("fingerprint must be a SHA-256 hex digest");
  return result;
}

function finite(value: unknown, path: string): number {
  if (typeof value !== "number" || !Number.isFinite(value)) invalid(`${path} must be finite`);
  return value as number;
}

function optionalFinite<T extends number | null>(value: unknown, fallback: T, scale = 1): number | T {
  if (value === undefined || value === null) return fallback;
  return finite(value, "number") * scale;
}

function integer(value: unknown, path: string): number {
  const result = finite(value, path);
  if (!Number.isInteger(result) || result < 0) invalid(`${path} must be a non-negative integer`);
  return result;
}

function optionalInteger(value: unknown, fallback: number): number {
  return value === undefined || value === null ? fallback : integer(value, "integer");
}

function boolean(value: unknown, fallback: boolean): boolean {
  return typeof value === "boolean" ? value : fallback;
}

function stringArray(value: unknown): string[] {
  return optionalArray(value).flatMap((item) => typeof item === "string" ? [item] : []);
}

function numberArray(value: unknown): number[] {
  return optionalArray(value).map((item, index) => finite(item, `numbers[${index}]`));
}

function invalid(message: string, operation: CadOperation | null = null): never {
  throw new CadClientError({
    code: "invalid_response",
    message,
    operation,
  });
}
