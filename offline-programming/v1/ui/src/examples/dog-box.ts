import {
  adaptCadQueryTopology,
  type CadTopology,
  type CadTopologyEdge,
} from "../adapters/cadquery.ts";
import { createWeldsFromRecommendations } from "../adapters/weld-authoring.ts";
import { createProject, type OfflineProgrammingProject, type Vec3 } from "../domain/project.ts";

// Browser-resident, already-normalized fallback for first-run PWA use. The
// production HttpCadClient replaces it with exact STEP topology when the local
// Go/CadQuery service is available.
export const DOG_BOX_FALLBACK_TOPOLOGY: CadTopology = {
  schemaVersion: 1,
  unit: "m",
  sourceUnit: "mm",
  tool: "offline-programming browser fixture",
  modelId: "dog-box-offline-fixture",
  filename: "dog_box.step",
  fingerprint: "d06b0fd3a9b47f361b1b0e57b14dfcbdbe83c8e58d22b3399699f9fc8b45b86a",
  sampleCount: 4,
  parts: [{ id: "dog-box", edgeCount: 8 }],
  edges: [
    edge("dog-box:top-front", "TOP FRONT", [[0.62, -0.19, 0.76], [0.73, -0.19, 0.76], [0.84, -0.19, 0.76], [0.95, -0.19, 0.76]]),
    edge("dog-box:top-right", "TOP RIGHT", [[0.95, -0.19, 0.76], [0.95, -0.065, 0.76], [0.95, 0.065, 0.76], [0.95, 0.19, 0.76]]),
    edge("dog-box:top-back", "TOP BACK", [[0.95, 0.19, 0.76], [0.84, 0.19, 0.76], [0.73, 0.19, 0.76], [0.62, 0.19, 0.76]]),
    edge("dog-box:top-left", "TOP LEFT", [[0.62, 0.19, 0.76], [0.62, 0.065, 0.76], [0.62, -0.065, 0.76], [0.62, -0.19, 0.76]]),
    edge("dog-box:front-left", "FRONT LEFT", [[0.62, -0.19, 0.61], [0.62, -0.19, 0.66], [0.62, -0.19, 0.71], [0.62, -0.19, 0.76]]),
    edge("dog-box:front-right", "FRONT RIGHT", [[0.95, -0.19, 0.61], [0.95, -0.19, 0.66], [0.95, -0.19, 0.71], [0.95, -0.19, 0.76]]),
    edge("dog-box:back-right", "BACK RIGHT", [[0.95, 0.19, 0.61], [0.95, 0.19, 0.66], [0.95, 0.19, 0.71], [0.95, 0.19, 0.76]]),
    edge("dog-box:back-left", "BACK LEFT", [[0.62, 0.19, 0.61], [0.62, 0.19, 0.66], [0.62, 0.19, 0.71], [0.62, 0.19, 0.76]]),
  ],
  rawEdgeCount: 8,
  compositeEdgeCount: 0,
  seamCandidateCount: 8,
  recommendedSeams: [
    recommendation("dog-box:top-front", 0.05, 0.95),
    recommendation("dog-box:top-right", 0.05, 0.95),
    recommendation("dog-box:top-back", 0.05, 0.95),
    recommendation("dog-box:top-left", 0.05, 0.95),
    recommendation("dog-box:front-left", 0.1, 0.9),
    recommendation("dog-box:front-right", 0.1, 0.9),
  ],
};

/**
 * The bundled, fully-authored example program. This is deliberate example
 * authoring over the fixture's recommendations — NOT an import-time default:
 * a real STEP import (and "Open Dog Box") stops at the pickable edge catalog
 * and waits for the user to select edges (P2.12).
 */
export function createDogBoxProject(timestamp?: string): OfflineProgrammingProject {
  const project = createProject({
    name: "Dog Box weld program",
    now: timestamp,
    cad: {
      kind: "example",
      filename: DOG_BOX_FALLBACK_TOPOLOGY.filename,
      fingerprint: DOG_BOX_FALLBACK_TOPOLOGY.fingerprint,
      sourceUnit: DOG_BOX_FALLBACK_TOPOLOGY.sourceUnit,
      topologyModelId: DOG_BOX_FALLBACK_TOPOLOGY.modelId,
    },
  });
  return createWeldsFromRecommendations(
    adaptCadQueryTopology(project, DOG_BOX_FALLBACK_TOPOLOGY, timestamp),
    timestamp,
  );
}

/** Catalog-only Dog Box load for the browser-fallback "Open Dog Box" path. */
export function createDogBoxEdgeCatalogProject(timestamp?: string): OfflineProgrammingProject {
  const project = createProject({
    name: "Dog Box weld program",
    now: timestamp,
    cad: {
      kind: "example",
      filename: DOG_BOX_FALLBACK_TOPOLOGY.filename,
      fingerprint: DOG_BOX_FALLBACK_TOPOLOGY.fingerprint,
      sourceUnit: DOG_BOX_FALLBACK_TOPOLOGY.sourceUnit,
      topologyModelId: DOG_BOX_FALLBACK_TOPOLOGY.modelId,
    },
  });
  return adaptCadQueryTopology(project, DOG_BOX_FALLBACK_TOPOLOGY, timestamp);
}

/**
 * Upgrade only bundled-example constraints that predate authored per-row RPY:
 * the rejected 45-degree fixture and the one-degree world-down gate.
 * Imported/customer programs retain their authored torch policy unchanged.
 */
export function needsDogBoxPlannerDefaultsMigration(
  project: OfflineProgrammingProject,
): boolean {
  if (
    project.cad.kind !== "example" ||
    project.cad.topologyModelId !== DOG_BOX_FALLBACK_TOPOLOGY.modelId
  ) return false;
  const constraint = project.program.constraints.tool_axis_constraint;
  if (!constraint || typeof constraint !== "object" || Array.isArray(constraint)) return false;
  const value = constraint as Record<string, unknown>;
  const rejectedFixture = value.offset_deg === 45 &&
    value.tolerance_minus_deg === 20 &&
    value.tolerance_plus_deg === 20;
  const obsoleteWorldDownGate = value.offset_deg === 0 &&
    value.tolerance_minus_deg === 1 &&
    value.tolerance_plus_deg === 1;
  return rejectedFixture || obsoleteWorldDownGate;
}

export function migrateDogBoxPlannerDefaults(
  project: OfflineProgrammingProject,
): boolean {
  if (!needsDogBoxPlannerDefaultsMigration(project)) return false;
  const constraint = project.program.constraints.tool_axis_constraint as Record<string, unknown>;
  project.program.constraints.tool_axis_constraint = {
    ...constraint,
    offset_deg: 0,
    tolerance_minus_deg: 90,
    tolerance_plus_deg: 90,
  };
  return true;
}

function edge(id: string, label: string, samplesM: Vec3[]): CadTopologyEdge {
  const xs = samplesM.map((point) => point[0]);
  const ys = samplesM.map((point) => point[1]);
  const zs = samplesM.map((point) => point[2]);
  let lengthM = 0;
  for (let index = 1; index < samplesM.length; index += 1) {
    const left = samplesM[index - 1]!;
    const right = samplesM[index]!;
    lengthM += Math.hypot(right[0] - left[0], right[1] - left[1], right[2] - left[2]);
  }
  return {
    id,
    label,
    partId: "dog-box",
    geometryType: "LINE",
    sampleCount: samplesM.length,
    samplesM,
    lengthM,
    boundsMinM: [Math.min(...xs), Math.min(...ys), Math.min(...zs)],
    boundsMaxM: [Math.max(...xs), Math.max(...ys), Math.max(...zs)],
    closed: false,
    sourceEdgeIds: [],
    sourceGeometryTypes: [],
    pathClass: "raw_fragment",
    isSeamCandidate: true,
  };
}

function recommendation(edgeId: string, start: number, end: number) {
  return {
    edgeId,
    partId: "dog-box",
    geometryType: "LINE",
    pathClass: "raw_fragment",
    window: { start, end },
    reversed: false,
    reason: "bundled Dog Box seam",
  };
}
