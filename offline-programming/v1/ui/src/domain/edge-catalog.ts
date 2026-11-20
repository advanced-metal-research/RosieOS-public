import type { CadTopology } from "../cad/contracts.ts";
import type {
  CadCatalogEdge,
  OfflineProgrammingProject,
  Vec3,
} from "./project.ts";
import { isRecord } from "./project.ts";

/**
 * The persisted pickable-edge catalog (P2.12). Import stores every sampled
 * topology edge here so the viewer can offer selection offline after a
 * save/load; extractor recommendations ride along as suggestion metadata and
 * are never turned into seams without the user creating a weld.
 */
export function catalogFromTopology(topology: CadTopology): CadCatalogEdge[] {
  const recommendations = new Map(
    topology.recommendedSeams.map((seam) => [seam.edgeId, seam]),
  );
  return topology.edges
    .filter((edge) => edge.samplesM.length >= 2)
    .map((edge) => {
      const recommendation = recommendations.get(edge.id);
      return {
        id: edge.id,
        label: edge.label,
        partId: edge.partId,
        geometryType: edge.geometryType,
        pathClass: edge.pathClass,
        closed: edge.closed,
        lengthM: edge.lengthM,
        samplesM: edge.samplesM.map(copyPoint),
        isSeamCandidate: edge.isSeamCandidate,
        recommendation: recommendation
          ? {
            window: {
              start: clamp01(recommendation.window.start),
              end: clamp01(recommendation.window.end),
            },
            reversed: recommendation.reversed === true,
            reason: recommendation.reason,
          }
          : null,
      };
    });
}

/** Catalog accessor tolerant of projects saved before the catalog existed. */
export function projectEdgeCatalog(
  project: OfflineProgrammingProject,
): CadCatalogEdge[] {
  return Array.isArray(project.edgeCatalog) ? project.edgeCatalog : [];
}

/**
 * GradientOS weld picking is one B-rep edge (a single line). The extractor
 * also emits face-wire composites — every edge on a surface joined into one
 * `WIRE` — which must not be hoverable or clickable.
 */
export function isSelectableTopologyEdge(edge: {
  geometryType: string;
  pathClass: string;
}): boolean {
  if (edge.geometryType.toUpperCase() === "WIRE") return false;
  return edge.pathClass !== "wire_non_seam" &&
    edge.pathClass !== "dogbone_4arc_4line";
}

export function catalogEdgeById(
  project: OfflineProgrammingProject,
  edgeId: string,
): CadCatalogEdge | null {
  return projectEdgeCatalog(project).find((edge) => edge.id === edgeId) ?? null;
}

/** Normalize an untyped stored catalog (save-file hydration). */
export function hydrateEdgeCatalog(value: unknown): CadCatalogEdge[] {
  if (!Array.isArray(value)) return [];
  return value.flatMap((entry) => {
    if (!isRecord(entry) || typeof entry.id !== "string" || !entry.id.trim()) return [];
    const samplesM = Array.isArray(entry.samplesM)
      ? entry.samplesM.flatMap((point) => {
        const copied = finitePoint(point);
        return copied ? [copied] : [];
      })
      : [];
    if (samplesM.length < 2) return [];
    const recommendation = isRecord(entry.recommendation) &&
        isRecord(entry.recommendation.window)
      ? {
        window: {
          start: clamp01(Number(entry.recommendation.window.start)),
          end: clamp01(Number(entry.recommendation.window.end)),
        },
        reversed: entry.recommendation.reversed === true,
        reason: typeof entry.recommendation.reason === "string"
          ? entry.recommendation.reason
          : "",
      }
      : null;
    return [{
      id: entry.id,
      label: typeof entry.label === "string" && entry.label.trim() ? entry.label : null,
      partId: typeof entry.partId === "string" && entry.partId.trim() ? entry.partId : null,
      geometryType: typeof entry.geometryType === "string" ? entry.geometryType : "LINE",
      pathClass: typeof entry.pathClass === "string" ? entry.pathClass : "raw_fragment",
      closed: entry.closed === true,
      lengthM: finite(entry.lengthM),
      samplesM,
      isSeamCandidate: entry.isSeamCandidate === true,
      recommendation,
    }];
  });
}

function copyPoint(point: readonly [number, number, number]): Vec3 {
  return [finite(point[0]), finite(point[1]), finite(point[2])];
}

function finitePoint(value: unknown): Vec3 | null {
  if (!Array.isArray(value) || value.length < 3) return null;
  const point: Vec3 = [finite(value[0]), finite(value[1]), finite(value[2])];
  return point;
}

function finite(value: unknown, fallback = 0): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function clamp01(value: number): number {
  return Math.max(0, Math.min(1, finite(value)));
}
