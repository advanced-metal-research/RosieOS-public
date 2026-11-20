export const CAD_API_PATH = "/api/offline-programming/v1/cadquery/topology" as const;
export const DOG_BOX_FIXTURE_ID = "dog_box" as const;

export type CadOperation =
  | "extract_topology"
  | "build_seam"
  | "build_sequence"
  | "tessellate";
export type Vec3 = [number, number, number];
export type Quaternion = [number, number, number, number];
export type CadMove = "approach" | "weld" | "retract" | "travel";

export type CadSTEPSource =
  | { kind: "bytes"; filename: string; bytes: Uint8Array }
  | { kind: "fixture"; fixtureId: string; filename?: string };

export const DOG_BOX_FIXTURE: CadSTEPSource = {
  kind: "fixture",
  fixtureId: DOG_BOX_FIXTURE_ID,
  filename: "dog_box.step",
};

export interface CadTopologyRequest {
  source: CadSTEPSource;
  sampleCount?: number;
}

export interface CadMeshRequest {
  source: CadSTEPSource;
  /** OCCT linear meshing tolerance in millimetres. */
  linearDeflectionMM?: number;
  /** OCCT angular meshing tolerance in radians. */
  angularDeflectionRad?: number;
  maxTriangles?: number;
}

/**
 * Tessellated workpiece surface, keyed by the STEP SHA-256 fingerprint.
 *
 * Deliberately NOT part of OfflineProgrammingProject: a production part is
 * megabytes of triangles and the project is autosaved on every edit. The mesh
 * is fetched by fingerprint and held only in memory.
 */
export interface CadWorkpieceMesh {
  schemaVersion: 1;
  unit: "m";
  sourceUnit: "mm";
  tool: string;
  modelId: string;
  filename: string;
  fingerprint: string;
  /** Flat XYZ triples in metres, in CAD-local coordinates. */
  positionsM: Float32Array;
  /** Flat triangle vertex indices into positionsM. */
  indices: Uint32Array;
  /** Flat unit-length XYZ triples, or null to compute them in the browser. */
  normals: Float32Array | null;
  vertexCount: number;
  triangleCount: number;
  boundsMinM: Vec3;
  boundsMaxM: Vec3;
  linearDeflectionM: number;
  angularDeflectionRad: number;
  solidCount: number;
  /** True when the worker hit its triangle cap; never a silent truncation. */
  truncated: boolean;
}

export interface CadSeamWindow {
  start: number;
  end: number;
}

export interface CadSeamRequest extends CadTopologyRequest {
  edgeId?: string;
  window?: CadSeamWindow;
  seamSampleCount?: number;
  reversed?: boolean;
  geometryMethod?: string;
  pathMethod?: "sampled_edge" | "three_point_arc" | string;
}

export interface CadSequenceSegment {
  id?: string;
  edgeId: string;
  window: CadSeamWindow;
  reversed?: boolean;
}

export interface CadSequenceRequest extends CadTopologyRequest {
  seamSampleCount?: number;
  segments: CadSequenceSegment[];
}

export interface CadTopologyPart {
  id: string;
  edgeCount: number;
}

export interface CadRecommendedSeam {
  edgeId: string;
  partId: string | null;
  geometryType: string;
  pathClass: string;
  window: CadSeamWindow;
  reversed: boolean;
  reason: string;
}

export interface CadTopologyEdge {
  id: string;
  label: string | null;
  partId: string | null;
  geometryType: string;
  sampleCount: number;
  samplesM: Vec3[];
  lengthM: number;
  boundsMinM: Vec3;
  boundsMaxM: Vec3;
  closed: boolean;
  sourceEdgeIds: string[];
  sourceGeometryTypes: string[];
  pathClass: string;
  isSeamCandidate: boolean;
}

export interface CadTopology {
  schemaVersion: 1;
  unit: "m";
  sourceUnit: "mm";
  tool: string;
  modelId: string;
  filename: string;
  fingerprint: string;
  sampleCount: number;
  parts: CadTopologyPart[];
  edges: CadTopologyEdge[];
  rawEdgeCount: number;
  compositeEdgeCount: number;
  seamCandidateCount: number;
  recommendedSeams: CadRecommendedSeam[];
}

export interface CadPlacement {
  robot: string | null;
  axis: string | null;
  table: string | null;
  frame: string;
  axisOriginM: Vec3 | null;
  originM: Vec3 | null;
  placementNote: string | null;
}

export interface CadCollisionObject {
  name: string;
  type: string;
  frame: string;
  parentLink: string;
  worldPositionM: Vec3;
  sizeM: Vec3;
  note: string | null;
}

export interface CadEnvironment {
  urdf: string;
  workcellFrame: string;
  manipulator: string;
  tcpFrame: string;
  tcpOffsetM: Vec3;
  positionerAxis: string;
  collisionObjects: CadCollisionObject[];
  constraints: {
    toolXAxis: string | null;
    toolYAxis: string | null;
    toolZAxis: string | null;
    tcpPosition: string | null;
    collisionMarginM: number | null;
    additional: Readonly<Record<string, string | boolean>>;
  };
}

export interface CadGeometryCandidate {
  id: string;
  score: number;
  awayFromPart: number;
  upward: number;
  toolXTangentAlignment: number;
  angleFromSeamPlaneDeg: number;
  maxFrameRotationDeg: number;
  seamPlaneNormal: Vec3;
  toolZAxis: Vec3;
}

export interface CadGeometryMethod {
  schema: string;
  selected: string;
  selectionReason: string;
  seamCenterM: Vec3;
  partCenterM: Vec3;
  candidates: CadGeometryCandidate[];
}

export interface CadTCPFrame {
  index: number;
  seamId: string | null;
  timeS: number;
  targetM: Vec3;
  seamTangent: Vec3;
  seamPlaneNormal: Vec3;
  seamSideAxis: Vec3;
  toolXAxis: Vec3;
  toolYAxis: Vec3;
  toolZAxis: Vec3;
  toolQuaternionXYZW: Quaternion | null;
  orientationRpyRad: Vec3;
  workAngleDeg: number;
  angleFromSeamPlaneDeg: number;
  toolXTangentAlignment: number;
}

export interface CadCollisionContact {
  linkA: string;
  linkB: string;
  depthM: number;
}

export interface CadMotionSample extends CadTCPFrame {
  move: CadMove;
  weldOn: boolean;
  jointNames: string[];
  jointPositionsRad: number[];
  ikPositionErrorM: number | null;
  ikRotationErrorDeg: number | null;
  collisionContacts: CadCollisionContact[];
}

export interface CadIKSummary {
  solver: string;
  robot: string;
  tcpLink: string;
  jointNames: string[];
  maxPositionErrorM: number;
  meanPositionErrorM: number;
  maxRotationErrorDeg: number;
  maxJointStepRad: number;
}

export interface CadCollisionSummary {
  status: "checked" | "unavailable" | "skipped";
  reason: string | null;
  checker: string | null;
  checkedWaypointCount: number;
  contactCount: number;
  contactSampleCount: number;
  maxDepthM: number;
  contactPairs: string[];
  checkDurationS: number;
}

export interface CadMotionPlan {
  trajectory: CadMotionSample[];
  sampleCount: number;
  weldSampleCount: number;
  durationS: number;
  toolPathLengthM: number;
  weldLengthM: number;
  travelLengthM: number;
  maxTCPStepM: number;
  maxFrameRotationDeg: number;
  perSeamFrameRotationDeg: number[];
  weldFeedMps: number;
  travelFeedMps: number;
  ik: CadIKSummary | null;
  collision: CadCollisionSummary | null;
}

export interface CadPlanStep {
  kind: "approach" | "weld_start" | "weld_stop" | "retract" | string;
  seamId: string | null;
  targetM: Vec3;
}

export interface CadPathMethod {
  selected: string;
  supported: string[];
  fallbackReason: string | null;
  controlPoseCount: number;
  controlPointsM: Vec3[];
}

export interface CadTesseractPlan {
  schema: string;
  planner: string;
  robot: string;
  workingFrame: string;
  edgeId: string | null;
  environment: CadEnvironment | null;
  geometryMethod: CadGeometryMethod | null;
  geometryMethods: CadGeometryMethod[];
  pathMethod: CadPathMethod | null;
  sequence: CadPlanStep[];
  tcpPoses: CadTCPFrame[];
  motionPlan: CadMotionPlan | null;
}

export interface CadWeldSeam {
  edgeId: string;
  partId: string | null;
  geometryType: string;
  window: CadSeamWindow;
  reversed: boolean;
  lengthM: number | null;
  pointCount: number;
  localPointsM: Vec3[];
  localControlPointsM: Vec3[];
  worldPointsM: Vec3[];
  worldControlPointsM: Vec3[];
  startPointM: Vec3;
  stopPointM: Vec3;
  positionerAxis: string | null;
}

export interface CadSequenceSeam {
  id: string;
  edgeId: string;
  partId: string | null;
  geometryType: string;
  window: CadSeamWindow;
  reversed: boolean;
  startPointM: Vec3;
  stopPointM: Vec3;
  pointCount: number;
  geometryMethod: CadGeometryMethod | null;
}

export interface CadWeldSequence {
  schema: string;
  robot: string;
  frame: string;
  positionerAxis: string | null;
  seams: CadSequenceSeam[];
}

export interface CadSeamResult {
  schemaVersion: 1;
  unit: "m";
  sourceUnit: "mm";
  tool: string;
  modelId: string;
  filename: string;
  fingerprint: string;
  placement: CadPlacement | null;
  environment: CadEnvironment | null;
  seam: CadWeldSeam;
  weldSequence: CadWeldSequence;
  tesseractPlan: CadTesseractPlan | null;
}

export interface CadSequenceResult {
  schemaVersion: 1;
  unit: "m";
  sourceUnit: "mm";
  tool: string;
  modelId: string;
  filename: string;
  fingerprint: string;
  placement: CadPlacement | null;
  environment: CadEnvironment | null;
  weldSequence: CadWeldSequence;
  tesseractPlan: CadTesseractPlan | null;
}

export type CadErrorCode =
  | "aborted"
  | "invalid_request"
  | "payload_too_large"
  | "busy"
  | "timeout"
  | "unavailable"
  | "worker_failed"
  | "network"
  | "invalid_response";

export class CadClientError extends Error {
  readonly code: CadErrorCode;
  readonly operation: CadOperation | null;
  readonly status: number | null;
  readonly retryable: boolean;
  readonly cause: unknown;

  constructor(options: {
    code: CadErrorCode;
    message: string;
    operation?: CadOperation | null;
    status?: number | null;
    retryable?: boolean;
    cause?: unknown;
  }) {
    super(options.message);
    this.name = "CadClientError";
    this.code = options.code;
    this.operation = options.operation ?? null;
    this.status = options.status ?? null;
    this.retryable = options.retryable ?? false;
    this.cause = options.cause;
  }
}

export type CadRequestState<T> =
  | { status: "idle"; data: null; error: null }
  | { status: "loading"; data: T | null; error: null }
  | { status: "ready"; data: T; error: null }
  | { status: "error"; data: T | null; error: CadClientError };

export interface CadClient {
  extractTopology(request: CadTopologyRequest, signal?: AbortSignal): Promise<CadTopology>;
  buildSeam(request: CadSeamRequest, signal?: AbortSignal): Promise<CadSeamResult>;
  buildSequence(request: CadSequenceRequest, signal?: AbortSignal): Promise<CadSequenceResult>;
  tessellate(request: CadMeshRequest, signal?: AbortSignal): Promise<CadWorkpieceMesh>;
}

// Wire contracts mirror the existing Python worker. Application code should
// consume the normalized contracts above.
export interface CadTopologyWire {
  schema_version: number;
  tool: string;
  model_id: string;
  filename: string;
  fingerprint: string;
  unit: string;
  sample_count: number;
  parts: Array<{ id: string; edge_count: number }>;
  edges: CadTopologyEdgeWire[];
  raw_edge_count?: number;
  composite_edge_count?: number;
  seam_candidate_count?: number;
  recommended_seams?: CadRecommendedSeamWire[];
}

export interface CadTopologyEdgeWire {
  id: string;
  label?: string;
  part_id?: string;
  geom_type?: string;
  sample_count?: number;
  samples: Vec3[];
  length_mm?: number;
  bounds_min?: Vec3;
  bounds_max?: Vec3;
  closed?: boolean;
  source_edge_ids?: string[];
  source_geom_types?: string[];
  path_class?: string;
  is_seam_candidate?: boolean;
}

export interface CadRecommendedSeamWire {
  edge_id: string;
  part_id?: string;
  geom_type?: string;
  path_class?: string;
  start_s: number;
  end_s: number;
  reversed?: boolean;
  reason?: string;
}

export interface CadWorkpieceMeshWire {
  schema_version: number;
  tool: string;
  model_id: string;
  filename: string;
  fingerprint: string;
  unit: string;
  mesh: {
    positions: number[];
    indices: number[];
    normals?: number[];
    vertex_count?: number;
    triangle_count?: number;
    bounds_min?: Vec3;
    bounds_max?: Vec3;
    linear_deflection_mm?: number;
    angular_deflection_rad?: number;
    solid_count?: number;
    truncated?: boolean;
  };
}

export interface CadSeamResultWire {
  schema_version: number;
  tool: string;
  model_id: string;
  filename: string;
  fingerprint: string;
  unit: string;
  placement?: unknown;
  environment?: unknown;
  seam: unknown;
  weld_sequence: unknown;
  tesseract_plan?: unknown;
}

export interface CadSequenceResultWire {
  schema_version: number;
  tool: string;
  model_id: string;
  filename: string;
  fingerprint: string;
  unit: string;
  placement?: unknown;
  environment?: unknown;
  weld_sequence: unknown;
  tesseract_plan?: unknown;
}
