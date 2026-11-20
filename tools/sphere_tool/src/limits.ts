/**
 * Joint limits as the panel has them, against the cell as the URDF states it.
 *
 * Two answers to the same question have to coexist while limits are being
 * authored: what the committed URDF says a joint's range is, and what the
 * operator has just measured it to be. The editor keeps them apart so the
 * export can diff one against the other and rewrite only what moved.
 *
 * Everything that POSES the robot has to be handed the edited one. `kinematics`
 * clamps to `cell.joints[].limits_rad` before it solves, so forward kinematics
 * given the raw cell silently holds a joint at the committed stop while the
 * slider, the readout and the export all use the widened one. On J3 at 100
 * degrees, with the committed stop at 80, that is 271 mm of TCP -- drawn
 * confidently, and wrong.
 */

import type { Cell } from "./kinematics";

export interface JointBound {
  lower: number;
  upper: number;
}

/** The cell's own limits, as the map the editor mutates. */
export function boundsOf(cell: Cell): Map<string, JointBound> {
  const out = new Map<string, JointBound>();
  for (const joint of cell.joints) {
    if (joint.type !== "revolute" || !joint.limits_rad) continue;
    out.set(joint.name, { ...joint.limits_rad });
  }
  return out;
}

/**
 * A VIEW of the cell carrying `bounds`, for anything that poses it.
 *
 * Never a mutation: `cell.joints[].limits_rad` has to survive as the URDF's
 * answer, because that is what the export diffs against to decide which joints
 * it rewrites.
 */
export function cellWithBounds(cell: Cell, bounds: Map<string, JointBound>): Cell {
  return {
    ...cell,
    joints: cell.joints.map((joint) => {
      const bound = bounds.get(joint.name);
      return bound ? { ...joint, limits_rad: { ...bound } } : joint;
    }),
  };
}

/** Whether a joint's bounds differ from the ones the cell declares. A tenth of
 *  a degree is not an edit: the fields round to that, so tabbing through a
 *  field without typing must not mark the joint changed. */
export function boundMoved(cell: Cell, name: string, bounds: Map<string, JointBound>): boolean {
  const edited = bounds.get(name);
  const declared = cell.joints.find((j) => j.name === name)?.limits_rad;
  if (!edited || !declared) return false;
  const deg = (rad: number) => (rad * 180) / Math.PI;
  return Math.abs(deg(edited.lower - declared.lower)) > 0.05
    || Math.abs(deg(edited.upper - declared.upper)) > 0.05;
}
