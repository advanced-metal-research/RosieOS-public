/**
 * The program's robot model, and the connected cell's settings pulled into it.
 * Desktop OLP's Robot and Cells menus and the Steam Deck pendant's Cell page
 * make these edits the same way.
 */
import { rowsFromSeams } from "../adapters/cadquery.ts";
import { alignWorkpieceToCellWorkFrame } from "./placement.ts";
import {
  cellSettingsKey, reconcileCellPin, setRobotDescriptionPin,
  type CellSettings, type OfflineProgrammingProject, type RobotModel,
} from "./project.ts";
import { knownRobotModel } from "./robots.ts";

export function isRobotModel(value: string | null): value is RobotModel {
  return typeof value === "string" && knownRobotModel(value);
}

/** Change the program's robot model, and with it THE WHOLE CELL, not just
 *  the robot the viewer draws. This once moved `robot.model` alone, so
 *  picking the 1420 redrew the arm and left the program pinned to the 1400:
 *  the rails compile sent `cell_id: rosie_1400_v3`, the server read
 *  `urdf/v1/rosie_1400_v3/robot.urdf`, and the exported `.weldplan` carried
 *  the 1400's cell descriptor. The plan came back posed for an arm standing
 *  0.375 m away from the one on screen. */
export function applyRobotModel(draft: OfflineProgrammingProject, model: RobotModel): void {
  draft.robot.model = model;
  setRobotDescriptionPin(draft);
  reconcileCellPin(draft);
  // The part keeps its place in the world; what changes is the frame the
  // offset is measured from, which is the cell's and not the 1400's table.
  alignWorkpieceToCellWorkFrame(draft);
  draft.rows = rowsFromSeams(draft);
}

/**
 * Pulled settings as one program edit: the cell's settings, and its robot
 * model when this OLP has a descriptor for it. Null when the program already
 * holds exactly these settings. `unknownModel` names a model the cell reports
 * that this OLP cannot describe; the program keeps its own.
 */
export function cellSettingsPull(project: OfflineProgrammingProject, settings: CellSettings): {
  apply: (draft: OfflineProgrammingProject) => void; unknownModel: string | null;
} | null {
  const model = settings.robotModelId;
  const known = isRobotModel(model);
  const modelChange = known && model !== project.robot.model;
  if (!modelChange && cellSettingsKey(project.robot.cell) === cellSettingsKey(settings)) return null;
  return {
    unknownModel: model && !known ? model : null,
    apply: (draft) => {
      draft.robot.cell = settings;
      if (known && model !== draft.robot.model) applyRobotModel(draft, model);
    },
  };
}
