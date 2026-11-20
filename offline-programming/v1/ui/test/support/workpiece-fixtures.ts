import type { CadTopology } from "../../src/cad/contracts.ts";

export function workpieceTopology(filename = "bracket.step", fingerprint = "a".repeat(64)): CadTopology {
  return {
    schemaVersion: 1, unit: "m", sourceUnit: "mm", tool: "test", modelId: fingerprint,
    filename, fingerprint, sampleCount: 2,
    parts: [{ id: "solid", edgeCount: 1 }], rawEdgeCount: 1, compositeEdgeCount: 0,
    seamCandidateCount: 1,
    edges: [{
      id: "edge-1", label: "Straight seam", partId: "solid", geometryType: "LINE",
      sampleCount: 2, samplesM: [[0, 0, 0], [0.2, 0, 0]], lengthM: 0.2,
      boundsMinM: [0, 0, 0], boundsMaxM: [0.2, 0, 0], closed: false,
      sourceEdgeIds: [], sourceGeometryTypes: [], pathClass: "line", isSeamCandidate: true,
    }],
    recommendedSeams: [{ edgeId: "edge-1", partId: "solid", geometryType: "LINE",
      pathClass: "line", window: { start: 0, end: 1 }, reversed: false, reason: "test" }],
  };
}
