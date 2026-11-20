import { Box, Factory, FileBox, Flame, Square } from "lucide";

import { activeWorkpieceId, projectWorkpieces } from "../domain/workpieces.ts";
import type { OfflineProgrammingProject } from "../domain/project.ts";
import { fixtureSublabel, workcellFixtures } from "../domain/workcell.ts";
import { weldNodes } from "../program-v2/nodes.ts";
import type { TreeNode } from "../ui/tree.ts";

/**
 * The program as a tree: what the left panel shows.
 *
 *   Program                 the file: name
 *   ├─ Workspace            the cell and the settings every program shares
 *   │  └─ Stand             the fixtures in the cell: what the part sits on
 *   └─ Part                 the imported STEP, or the empty slot
 *      └─ Welds             the weld nodes, one per authored weld
 *
 * Node ids double as selection ids: "program", "workspace", "part",
 * "fixture:<id>", and a weld's node id. Welds sit under the part because a
 * weld is a feature of that part; fixtures sit under the workspace because
 * they are the cell's, whatever part is loaded.
 */
export const TREE_PROGRAM = "program";
export const TREE_WORKSPACE = "workspace";
export const TREE_PART = "part";
export const TREE_FIXTURE_PREFIX = "fixture:";
export const TREE_WORKPIECE_PREFIX = "workpiece:";
export const TREE_WORKPIECE_WELD_PREFIX = "workpiece-weld:";

export interface ProgramTreeOptions {
  /** Runs the part import; shown as the empty slot's hint. */
  onImportPart: () => void;
  /** Transient filename while reading and processing a new part. */
  importingFilename?: string | null;
  importingWorkpieceId?: string | null;
  importingKind?: "weld" | "fixture";
  /** Adds a stand to a cell that has none; shown as that slot's hint. */
  onAddStand?: () => void;
  /** The bundled demo files' names: a part with one of these was opened
   *  from "Open demo file", and says so. */
  demoFilenames?: ReadonlySet<string>;
}

export function programTreeNodes(
  project: OfflineProgrammingProject,
  options: ProgramTreeOptions,
): TreeNode[] {
  const parts: TreeNode[] = projectWorkpieces(project).map(part => {
    const active = part.id === activeWorkpieceId(project);
    const welds = weldNodes(part.nodes);
    const weldChildren: TreeNode[] = welds.map((weld) => ({
    id: active ? weld.id : `${TREE_WORKPIECE_WELD_PREFIX}${encodeURIComponent(part.id)}/${encodeURIComponent(weld.id)}`,
    label: weld.label || weld.id,
    sublabel: weld.enabled ? "" : "off",
    icon: Flame,
  }));
  const isDemo = part.cad.kind === "example" || (options.demoFilenames?.has(part.cad.filename) ?? false);
  const weldCount = welds.length ? `${welds.length} weld${welds.length === 1 ? "" : "s"}` : "";
    return {
      id: active ? TREE_PART : TREE_WORKPIECE_PREFIX + part.id,
      label: part.cad.filename || "Part",
      // A demo part says so where it sits, not only in the menu it came from.
      sublabel: [active ? "active weld part" : "weld part", isDemo ? "demo" : "", weldCount].filter(Boolean).join(" · "),
      icon: Box,
      children: weldChildren,
      busy: options.importingWorkpieceId === part.id,
    };
  });
  if (parts.length === 0 && !options.importingFilename) parts.push({
      id: TREE_PART,
      label: "No part",
      icon: Box,
      muted: true,
      hint: { label: "Import a part", run: options.onImportPart },
    });
  const fixtures = workcellFixtures(project);
  const fixtureChildren: TreeNode[] = fixtures.length
    ? fixtures.map((fixture) => ({
      id: TREE_FIXTURE_PREFIX + fixture.id,
      label: fixture.name,
      sublabel: fixture.shape.kind === "step" ? "fixture" : `fixture · ${fixtureSublabel(fixture)}`,
      icon: Square,
    }))
    : [{
      id: TREE_FIXTURE_PREFIX,
      label: "No stand",
      icon: Square,
      muted: true,
      selectable: false,
      ...(options.onAddStand ? { hint: { label: "Add a stand", run: options.onAddStand } } : {}),
    }];
  if (options.importingFilename && !options.importingWorkpieceId) {
    const pending: TreeNode = { id: "importing-part", label: options.importingFilename,
      icon: Box, busy: true, selectable: false };
    (options.importingKind === "fixture" ? fixtureChildren : parts).push(pending);
  }
  return [
    {
      id: TREE_PROGRAM,
      label: project.identity.name || "Untitled program",
      // The root of the tree says so, the way the cell and the part say
      // what they are beside their names.
      sublabel: "root",
      icon: FileBox,
      children: [
        {
          id: TREE_WORKSPACE,
          label: "Workspace",
          sublabel: project.robot.cell ? `${project.robot.model} · ${project.robot.cell.cell}` : project.robot.model,
          icon: Factory,
          children: fixtureChildren,
        },
        ...parts,
      ],
    },
  ];
}
