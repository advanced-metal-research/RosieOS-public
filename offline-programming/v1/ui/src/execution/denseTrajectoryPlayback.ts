/**
 * Replaying a planned rails program: the bytes the robot will play, in the
 * viewer.
 *
 * The classic flow previews by interpolating its rows, which carry a TCP pose
 * per waypoint. A rails plan has no rows -- `export/robot_v4.py` compiled them
 * and went with the authoring server -- and what it has instead is better for
 * this: the dense joint trajectory, the same bytes the daemon ingests and the
 * arm executes. Previewing anything else would be previewing a derivation and
 * hoping the two agree.
 *
 * So this is a separate path from `domain/playback.ts` rather than an
 * adaptation of it, because the two are genuinely different documents. A row
 * is a waypoint with a baked pose; a dense sample is nine joints, their rates
 * and whether the arc is lit, at 100 Hz. The robot is posed from the joints
 * either way, and no pose is derived here to sit beside them -- the viewer's
 * own kinematics already knows it, and a second answer computed here could
 * only disagree with the first.
 */

import { denseSegmentNodeId } from "../program-v2/dense-cursor.ts";
import type { SimulationSample } from "../simulation/contracts.ts";
import type { OfflineProgrammingProject } from "../domain/project.ts";
import type { DenseTrajectory } from "./denseTrajectoryBlob.ts";
import { flattenForPlayback } from "./denseTrajectoryBlob.ts";

/** A segment kind, as the motion result named it, mapped to what the viewer
 *  colours motion by. A dwell is the torch settling on a weld and is drawn as
 *  one. */
function motionOf(kind: string): SimulationSample["motion"] {
  return kind === "weld" || kind === "dwell" ? "weld" : "travel";
}

/**
 * The trajectory as viewer samples, in segment order.
 *
 * `timeS` runs across the whole playback so a scrubber has something
 * monotonic to seek in. That is this function's convenience and not the
 * contract: every segment keeps its own clock, a boundary is a stop rather
 * than a join, and the daemon honours the per-segment times. Nothing that
 * commands motion reads what is written here.
 */
export function denseTrajectorySamples(
  project: OfflineProgrammingProject,
  trajectory: DenseTrajectory,
): SimulationSample[] {
  const jointOrder = [...project.robot.jointOrder];
  const byIndex = new Map(trajectory.segments.map((segment) => [segment.index, segment]));
  const nodeBySegment = new Map(trajectory.segments.map(segment =>
    [segment.index, denseSegmentNodeId(project.nodes, segment.kind as "freespace" | "weld" | "dwell", segment.sourceId)]));
  return flattenForPlayback(trajectory).map(({ elapsedS, segmentIndex, sample }) => {
    const segment = byIndex.get(segmentIndex);
    return {
      modelId: project.robot.model,
      jointOrder,
      // The trajectory carries nine axes -- the arm and the positioner -- and
      // a cell with fewer named joints takes the ones it named.
      jointsRad: [...sample.qRad].slice(0, jointOrder.length),
      source: "preview" as const,
      nodeId: nodeBySegment.get(segmentIndex) ?? null,
      timeS: elapsedS,
      motion: motionOf(segment?.kind ?? "freespace"),
      // Which weld, when the segment came out of one. A freespace segment's
      // source names both ends and belongs to no single seam.
      seamId: segment?.kind === "weld" ? segment.sourceId : null,
    };
  });
}

/** Whether the arc is lit at each sample, in the same order.
 *
 *  Kept apart from the samples because it is the one thing here the viewer
 *  does not already have a field for, and because it is worth being able to
 *  say the trajectory carries it: the torch flag rides in the blob, per
 *  sample, so the preview switches the arc exactly where the robot will. */
export function torchOnFlags(trajectory: DenseTrajectory): boolean[] {
  return flattenForPlayback(trajectory).map(({ sample }) => sample.torchOn);
}

/** How long the whole thing takes, as the sum of the segments' own clocks.
 *
 *  Not the last sample's `t_s`: that is the last SEGMENT's, and reading it as
 *  the total is exactly the mistake of gluing the clocks together. */
export function totalDurationS(trajectory: DenseTrajectory): number {
  return trajectory.segments.reduce((total, segment) => {
    const last = segment.samples[segment.samples.length - 1];
    return total + (last?.tS ?? 0);
  }, 0);
}
