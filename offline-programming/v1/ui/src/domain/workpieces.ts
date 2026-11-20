import { hasWorkpieceGeometry, transformWorkpiecePoint, workpieceWorldRotationRpy } from "./placement.ts";
import { type OfflineProgrammingProject, type WorkpieceRecord, projectId } from "./project.ts";
import { type WorkcellFixture, workcellFixtures } from "./workcell.ts";

export const LEGACY_WORKPIECE_ID = "part";

export function activeWorkpieceId(project: OfflineProgrammingProject): string {
  return project.activeWorkpieceId ?? LEGACY_WORKPIECE_ID;
}

function capture(project: OfflineProgrammingProject): WorkpieceRecord {
  return { id: activeWorkpieceId(project), cad: project.cad,
    fixtures: project.fixtures, edgeCatalog: project.edgeCatalog, seams: project.seams,
    nodes: project.nodes, sequence: project.sequence, rows: project.rows };
}

/** Read the active working copy, never its older saved snapshot. */
export function projectWorkpieces(project: OfflineProgrammingProject): WorkpieceRecord[] {
  const id = activeWorkpieceId(project);
  const records = (project.workpieces ?? []).filter(part => part.id !== id);
  const active = hasWorkpieceGeometry(project) ? capture(project) : null;
  const order = (project.workpieces ?? []).map(part => part.id);
  if (active) records.push(active);
  return records.sort((a, b) => {
    const index = (key: string) => { const i = order.indexOf(key); return i < 0 ? order.length : i; };
    return index(a.id) - index(b.id);
  });
}

export function workpieceProject(project: OfflineProgrammingProject, part: WorkpieceRecord): OfflineProgrammingProject {
  return { ...project, ...part, activeWorkpieceId: part.id };
}

/** Called before a new part is installed. Existing geometry and welds stay
 * in their own part; the new working copy starts empty. */
export function beginAddedWorkpiece(draft: OfflineProgrammingProject): void {
  draft.workpieces = projectWorkpieces(draft);
  draft.activeWorkpieceId = projectId();
  draft.cad = { kind: "step", filename: "", fingerprint: "", sourceUnit: "mm", topologyModelId: "" };
  draft.fixtures = [];
  draft.edgeCatalog = [];
  draft.seams = [];
  draft.nodes = [];
  draft.sequence = [];
  draft.rows = [];
  draft.simulation.selectedRowId = null;
}

export function retainActiveWorkpiece(draft: OfflineProgrammingProject): void {
  draft.workpieces = projectWorkpieces(draft);
}

/** A selection changes the editing/planning part, not the workspace. */
export function activateWorkpiece(draft: OfflineProgrammingProject, id: string): boolean {
  const parts = projectWorkpieces(draft);
  const selected = parts.find(part => part.id === id);
  if (!selected) return false;
  draft.workpieces = parts;
  const { id: selectedId, ...fields } = structuredClone(selected);
  Object.assign(draft, fields);
  draft.activeWorkpieceId = selectedId;
  draft.simulation.selectedRowId = null;
  return true;
}

export function forgetActiveWorkpiece(draft: OfflineProgrammingProject): void {
  const id = activeWorkpieceId(draft);
  draft.workpieces = projectWorkpieces(draft).filter(part => part.id !== id);
}

/** Inactive weld parts remain visible and are collision obstacles while the
 * selected part is planned. Use the same CAD-to-world transform as its mesh. */
export function environmentFixtures(project: OfflineProgrammingProject): WorkcellFixture[] {
  return [...workcellFixtures(project), ...projectWorkpieces(project)
    .filter(part => part.id !== activeWorkpieceId(project))
    .map(part => {
      const view = workpieceProject(project, part);
      return {
        id: `workpiece:${part.id}`, name: part.cad.filename,
        shape: { kind: "step" as const, fingerprint: part.cad.fingerprint, filename: part.cad.filename },
        translationM: transformWorkpiecePoint(view, [0, 0, 0]),
        rotationRpyRad: workpieceWorldRotationRpy(view), origin: "authored" as const,
      };
    })];
}
