import * as THREE from "three";
import type { URDFRobot } from "urdf-loader";
import type { OfflineProgrammingProject } from "../domain/project.ts";
import { plannerPoseToWorld } from "../domain/placement.ts";
import type { DenseTrajectory } from "../execution/denseTrajectoryBlob.ts";
import type { SimulationSample } from "../simulation/contracts.ts";
import type { MotionPathPoint } from "./geometry.ts";

/** Keep each planned segment separate: no synthetic line crosses a boundary
 * where the planner supplied no connecting motion. Every dense sample is kept. */
export function densePlannedPathRuns(
  project: OfflineProgrammingProject,
  trajectory: DenseTrajectory,
): SimulationSample[][] {
  let elapsed = 0;
  return trajectory.segments.map(segment => {
    const samples = segment.samples.map(sample => ({
      modelId: project.robot.model,
      jointOrder: [...project.robot.jointOrder],
      jointsRad: Array.from(sample.qRad).slice(0, project.robot.jointOrder.length),
      source: "preview" as const,
      timeS: elapsed + sample.tS,
      motion: segment.kind === "weld" || segment.kind === "dwell" ? "weld" as const : "travel" as const,
      seamId: segment.kind === "weld" ? segment.sourceId : null,
    }));
    elapsed += segment.samples.at(-1)?.tS ?? 0;
    return samples;
  });
}

/** Classic plans already carry the planner's TCP result for every sample. */
export function projectPlannedPath(project: OfflineProgrammingProject): MotionPathPoint[] {
  return project.plan.trajectory.map(sample => {
    if (!sample.tcpPose.every(Number.isFinite)) throw new Error("Planned TCP path contains an invalid pose");
    const world = plannerPoseToWorld(project, sample.tcpPose);
    return { positionM: [world[0], world[1], world[2]], motion: sample.weldOn ? "weld" : "travel" };
  });
}

export interface PlannedPathTraceOptions {
  /** Async callers use this to cancel an obsolete project/model/plan. */
  isCurrent?: () => boolean;
  /** Injected for deterministic cancellation tests; default yields to the UI. */
  yieldToUI?: () => Promise<void>;
}

/** Evaluate the viewer's URDF kinematics on a private copy. The visible robot,
 * pose latch and moving workpiece never change. Returned TCP points are in
 * scene world coordinates and must be drawn outside the moving work frame. */
export async function tracePlannedTcpPath(
  robot: URDFRobot,
  tcpFrameName: string,
  jointOrder: readonly string[],
  sampleRuns: readonly (readonly SimulationSample[])[],
  options: PlannedPathTraceOptions = {},
): Promise<MotionPathPoint[][]> {
  const isCurrent = options.isCurrent ?? (() => true);
  if (!isCurrent()) return [];
  const copy = robot.clone() as URDFRobot;
  robot.updateWorldMatrix(true, false);
  copy.matrix.copy(robot.matrixWorld);
  copy.matrixAutoUpdate = false;
  // Mesh resources are shared by Object3D.clone: detach, never dispose them.
  const visuals: THREE.Object3D[] = [];
  copy.traverse(node => {
    if (["URDFVisual", "URDFCollider", "Mesh"].includes(node.type)) visuals.push(node);
  });
  for (const visual of visuals) visual.removeFromParent();
  const tcp = copy.links[tcpFrameName] ?? copy.joints[tcpFrameName];
  if (!tcp) throw new Error(`Planned TCP path cannot find robot frame ${tcpFrameName}`);
  if (new Set(jointOrder).size !== jointOrder.length || jointOrder.length === 0) {
    throw new Error("Planned TCP path has an invalid joint order");
  }
  for (const name of jointOrder) {
    const joint = copy.joints[name];
    if (!joint) throw new Error(`Planned TCP path cannot find robot joint ${name}`);
    // Draw the actual planned coordinates, including any out-of-limit evidence;
    // silently clamping would describe a different motion than the plan.
    joint.ignoreLimits = true;
  }
  const runs: MotionPathPoint[][] = [];
  const position = new THREE.Vector3();
  const yieldToUI = options.yieldToUI ?? (() => new Promise<void>(resolve => setTimeout(resolve, 0)));
  let processed = 0;
  for (const samples of sampleRuns) {
    const run: MotionPathPoint[] = [];
    for (const sample of samples) {
      if (processed > 0 && processed % 1024 === 0) {
        await yieldToUI();
        if (!isCurrent()) return [];
      }
      const indices = jointOrder.map(name => sample.jointOrder.indexOf(name));
      if (sample.jointsRad.length !== sample.jointOrder.length ||
          new Set(sample.jointOrder).size !== sample.jointOrder.length ||
          indices.some(index => index < 0) || !sample.jointsRad.every(Number.isFinite)) {
        throw new Error("Planned TCP path sample does not match the robot joint order");
      }
      // The dense wire always has nine axes; a six-axis URDF uses its named
      // joints and leaves the unused wire columns out, exactly as playback does.
      for (let index = 0; index < jointOrder.length; index++) copy.setJointValue(jointOrder[index]!, sample.jointsRad[indices[index]!]!);
      // Only the TCP ancestry needs updating, not the detached visual hierarchy.
      tcp.updateWorldMatrix(true, false);
      position.setFromMatrixPosition(tcp.matrixWorld);
      if (![position.x, position.y, position.z].every(Number.isFinite)) throw new Error("Planned TCP path has non-finite kinematics");
      run.push({ positionM: [position.x, position.y, position.z], motion: sample.motion });
      processed++;
    }
    runs.push(run);
  }
  return isCurrent() ? runs : [];
}
