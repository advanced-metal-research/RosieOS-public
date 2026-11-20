import { syncClassicNodes } from "../program-v2/from-project.ts";
import { rowsFromNodes } from "../program-v2/rows-from-nodes.ts";
import {
  ensureCadPathRows,
  refreshCadRowBudgetStatus,
  safeSeamIdFragment,
  seamTrimFromWindow,
} from "./cadquery.ts";
import {
  catalogEdgeById,
  projectEdgeCatalog,
} from "../domain/edge-catalog.ts";
import {
  reviseProject,
  type AuthoredSeam,
  type CadCatalogEdge,
  type OfflineProgrammingProject,
  type WeldProcessParameters,
  type WeldType,
} from "../domain/project.ts";
import { processTorchDefaults } from "../domain/weld.ts";

export const WELD_TYPES: readonly WeldType[] = [
  "fillet",
  "butt",
  "lap",
  "tack",
  "custom",
];

/** One selected edge span inside a weld draft; trims are normalized 0..1. */
export interface WeldDraftSegment {
  edgeId: string;
  startS: number;
  endS: number;
  reversed: boolean;
}

export interface WeldDraftInput {
  name?: string;
  weldType?: WeldType;
  segments: readonly WeldDraftSegment[];
  /** Torch defaults for this weld; omitted values keep the process defaults. */
  workAngleDeg?: number;
  travelAngleDeg?: number;
  spinAngleDeg?: number;
  processParameterId?: string;
}

export interface ProjectWeld {
  id: string;
  label: string;
  weldType: WeldType | null;
  seamIds: string[];
}

/**
 * User-created welds, grouped from the seams they own. Legacy seams authored
 * before P2.12 have no weld id and surface as single-seam welds keyed by the
 * seam id, so they remain manageable from the same UI.
 */
export function projectWelds(project: OfflineProgrammingProject): ProjectWeld[] {
  const welds: ProjectWeld[] = [];
  const byId = new Map<string, ProjectWeld>();
  for (const seam of project.seams) {
    const id = seam.weldId ?? seam.id;
    const existing = byId.get(id);
    if (existing) {
      existing.seamIds.push(seam.id);
      continue;
    }
    const weld: ProjectWeld = {
      id,
      label: seam.label.replace(/ · \d+$/, ""),
      weldType: seam.weldType ?? null,
      seamIds: [seam.id],
    };
    byId.set(id, weld);
    welds.push(weld);
  }
  return welds;
}

/**
 * Turn an additive edge-selection draft into program state: one seam per
 * selected edge span, sequence steps in draft order, and rows generated
 * through the budgeted CAD machinery (`rowsFromSeams` via
 * `ensureCadPathRows`), so the over-budget lint and the
 * `deliberatelyRowlessSeamIDs` boot contract keep holding. Manually authored
 * rows are preserved — only the new seams' rows are inserted.
 */
export function createWeldFromEdges(
  project: OfflineProgrammingProject,
  input: WeldDraftInput,
  timestamp?: string,
): OfflineProgrammingProject {
  const segments = input.segments.filter((segment) =>
    catalogEdgeById(project, segment.edgeId) !== null
  );
  if (segments.length === 0) {
    throw new Error("a weld needs at least one selected catalog edge");
  }
  return reviseProject(project, "seams", (draft) => {
    appendWeldSeams(draft, { ...input, segments });
    // Nodes are the authored program. The seam compile runs so the new
    // seam's points can be read off its rows (and the budget bookkeeping
    // holds), then the prior rows are put back so a recompile from the
    // nodes keeps every untouched row's planner status, and every row is
    // compiled from the nodes.
    const priorRows = draft.rows;
    ensureCadPathRows(draft);
    syncClassicNodes(draft);
    draft.rows = priorRows;
    draft.rows = rowsFromNodes(draft);
    if (!draft.rows.some((row) => row.id === draft.simulation.selectedRowId)) {
      draft.simulation.selectedRowId = draft.rows[0]?.id ?? null;
    }
  }, timestamp);
}

/**
 * Author one weld per extractor recommendation (or per seam-candidate edge
 * when the extractor recommended nothing, mirroring the pre-P2.12 fallback).
 * This is the explicit, single-revision path used by the bundled Dog Box
 * example and by tests that need the historical auto-import program; the
 * interactive UI never calls it.
 */
export function createWeldsFromRecommendations(
  project: OfflineProgrammingProject,
  timestamp?: string,
): OfflineProgrammingProject {
  const catalog = projectEdgeCatalog(project);
  const recommended = catalog.filter((edge) => edge.recommendation !== null);
  const selected = recommended.length > 0
    ? recommended
    : catalog.filter((edge) => edge.isSeamCandidate).slice(0, 8);
  if (selected.length === 0) return project;
  return reviseProject(project, "seams", (draft) => {
    for (const edge of selected) {
      appendWeldSeams(draft, {
        weldType: "fillet",
        segments: [{
          edgeId: edge.id,
          startS: edge.recommendation?.window.start ?? 0,
          endS: edge.recommendation?.window.end ?? 1,
          reversed: edge.recommendation?.reversed ?? false,
        }],
      });
    }
    // Nodes are the authored program. The seam compile runs so the new
    // seam's points can be read off its rows (and the budget bookkeeping
    // holds), then the prior rows are put back so a recompile from the
    // nodes keeps every untouched row's planner status, and every row is
    // compiled from the nodes.
    const priorRows = draft.rows;
    ensureCadPathRows(draft);
    syncClassicNodes(draft);
    draft.rows = priorRows;
    draft.rows = rowsFromNodes(draft);
    if (!draft.rows.some((row) => row.id === draft.simulation.selectedRowId)) {
      draft.simulation.selectedRowId = draft.rows[0]?.id ?? null;
    }
  }, timestamp);
}

/** Remove a weld and exactly its seams, sequence steps, and program rows. */
export function deleteWeld(
  project: OfflineProgrammingProject,
  weldId: string,
  timestamp?: string,
): OfflineProgrammingProject {
  const target = projectWelds(project).find((weld) => weld.id === weldId);
  if (!target) throw new Error(`weld ${weldId} was not found`);
  const seamIds = new Set(target.seamIds);
  return reviseProject(project, "seams", (draft) => {
    draft.seams = draft.seams.filter((seam) => !seamIds.has(seam.id));
    draft.sequence = draft.sequence.filter((step) => !seamIds.has(step.seamId));
    draft.rows = draft.rows.filter((row) =>
      row.seamId === null || !seamIds.has(row.seamId)
    );
    // Freed budget first (over-budget list shrinks), then restore rows for any
    // seam the budget no longer excludes — as disabled rows, per P2.2.
    refreshCadRowBudgetStatus(draft);
    // Nodes are the authored program. The seam compile runs so the new
    // seam's points can be read off its rows (and the budget bookkeeping
    // holds), then the prior rows are put back so a recompile from the
    // nodes keeps every untouched row's planner status, and every row is
    // compiled from the nodes.
    const priorRows = draft.rows;
    ensureCadPathRows(draft);
    syncClassicNodes(draft);
    draft.rows = priorRows;
    draft.rows = rowsFromNodes(draft);
    if (!draft.rows.some((row) => row.id === draft.simulation.selectedRowId)) {
      draft.simulation.selectedRowId = draft.rows[0]?.id ?? null;
    }
  }, timestamp);
}

/**
 * Append the draft's seams and sequence steps to the project draft. Ids and
 * labels intentionally follow the historical auto-import scheme
 * (`seam-N-<edge>` / `SEAM NN` / `sequence-N`) so a user selecting the
 * recommended edges produces the same program the auto path used to.
 */
function appendWeldSeams(
  draft: OfflineProgrammingProject,
  input: WeldDraftInput & { segments: readonly WeldDraftSegment[] },
): void {
  const weldId = nextWeldId(draft);
  const weldType = input.weldType ?? "fillet";
  const name = input.name?.trim() ?? "";
  const processParameterId = weldProcessParameterId(draft, input, weldId);
  input.segments.forEach((segment, segmentIndex) => {
    const edge = catalogEdgeById(draft, segment.edgeId);
    if (!edge || edge.samplesM.length < 2) return;
    const seamIndex = draft.seams.length;
    const seam: AuthoredSeam = {
      id: uniqueSeamId(draft, seamIndex, edge),
      label: seamLabel(name, input.segments.length, segmentIndex, edge, seamIndex),
      enabled: true,
      sourceEdgeId: edge.id,
      partId: edge.partId,
      geometryType: edge.geometryType,
      trim: seamTrimFromWindow(edge.samplesM, {
        start: Math.min(segment.startS, segment.endS),
        end: Math.max(segment.startS, segment.endS),
      }),
      reversed: segment.reversed,
      geometry: {
        kind: "polyline",
        pointsM: edge.samplesM.map((point) => [...point]),
      },
      workFrameId: "world",
      processParameterId,
      weldId,
      weldType,
    };
    draft.seams.push(seam);
    draft.sequence.push({
      id: `sequence-${draft.sequence.length + 1}`,
      seamId: seam.id,
      order: draft.sequence.length,
      approachOffsetM: 0.05,
      retractOffsetM: 0.05,
    });
  });
}

/**
 * The weld's process parameters. Torch angles equal to the referenced process
 * defaults reuse it unchanged (keeping select-then-create byte-comparable
 * with the old auto path); custom angles clone the process for this weld.
 */
function weldProcessParameterId(
  draft: OfflineProgrammingProject,
  input: WeldDraftInput,
  weldId: string,
): string {
  const baseId = input.processParameterId ?? "process-default";
  const base = draft.processParameters.find((candidate) => candidate.id === baseId) ??
    draft.processParameters[0];
  if (!base) return baseId;
  const defaults = processTorchDefaults(base);
  const workAngleDeg = finiteOr(input.workAngleDeg, defaults.workAngleDeg);
  const travelAngleDeg = finiteOr(input.travelAngleDeg, defaults.travelAngleDeg);
  const spinAngleDeg = finiteOr(input.spinAngleDeg, defaults.spinAngleDeg);
  if (
    workAngleDeg === defaults.workAngleDeg &&
    travelAngleDeg === defaults.travelAngleDeg &&
    spinAngleDeg === defaults.spinAngleDeg
  ) {
    return base.id;
  }
  const cloned: WeldProcessParameters = {
    ...structuredClone(base),
    id: `process-${weldId}`,
    name: `${base.name} · ${weldId}`,
    torchWorkAngleDeg: workAngleDeg,
    torchTravelAngleDeg: travelAngleDeg,
    torchSpinAngleDeg: spinAngleDeg,
  };
  draft.processParameters.push(cloned);
  return cloned.id;
}

function seamLabel(
  name: string,
  segmentCount: number,
  segmentIndex: number,
  edge: CadCatalogEdge,
  seamIndex: number,
): string {
  if (name) return segmentCount > 1 ? `${name} · ${segmentIndex + 1}` : name;
  return edge.label ?? `SEAM ${String(seamIndex + 1).padStart(2, "0")}`;
}

function uniqueSeamId(
  draft: OfflineProgrammingProject,
  seamIndex: number,
  edge: CadCatalogEdge,
): string {
  const base = `seam-${seamIndex + 1}-${safeSeamIdFragment(edge.id)}`;
  if (!draft.seams.some((seam) => seam.id === base)) return base;
  let suffix = 2;
  while (draft.seams.some((seam) => seam.id === `${base}-${suffix}`)) suffix += 1;
  return `${base}-${suffix}`;
}

function nextWeldId(draft: OfflineProgrammingProject): string {
  let highest = 0;
  for (const seam of draft.seams) {
    const match = seam.weldId?.match(/^weld-(\d+)$/);
    if (match) highest = Math.max(highest, Number(match[1]));
  }
  return `weld-${highest + 1}`;
}

function finiteOr(value: number | undefined, fallback: number): number {
  return typeof value === "number" && Number.isFinite(value) ? value : fallback;
}
