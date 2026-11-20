import {
  cellSettingsKey,
  planIsFresh,
  type OfflineProgrammingProject,
} from "../domain/project.ts";
import { decodeDenseTrajectory, type DenseTrajectory } from "../execution/denseTrajectoryBlob.ts";

export interface PlannedPathControllerDeps {
  readTrajectory(digest: string): Promise<Uint8Array | ArrayBuffer | null>;
  clear(): void;
  /** Async viewers must check isCurrent after awaiting model/FK preparation,
   * before changing the scene. Null dense means the classic plan's samples. */
  show(project: OfflineProgrammingProject, dense: DenseTrajectory | null, isCurrent: () => boolean): void | Promise<void>;
  onError?(error: unknown): void;
}

/** Loads only the active, current plan. Display preferences and playback pose
 * are deliberately absent: toggling a layer or jogging never reloads a plan. */
export function createPlannedPathController(deps: PlannedPathControllerDeps): {
  sync(project: OfflineProgrammingProject): Promise<void>;
  dispose(): void;
} {
  let generation = 0;
  let key: string | null = null;
  let pending = Promise.resolve();
  let disposed = false;

  function sync(project: OfflineProgrammingProject): Promise<void> {
    if (disposed) return Promise.resolve();
    const dense = project.denseTrajectory;
    const nextKey = JSON.stringify([
      project.identity.id, project.identity.revision, project.activeWorkpieceId,
      project.robot.model, project.robot.jointOrder,
      project.robot.description?.robotDescriptionSha256, cellSettingsKey(project.robot.cell),
      project.frames,
      dense ? [dense.trajectoryDigest, dense.sourceProjectRevision, dense.robot] : [
        project.plan.id, project.plan.revision, project.plan.state,
        project.plan.sourceProjectRevision, project.plan.sourceRobotModel,
        project.plan.sourceCell, project.plan.trajectory.length,
      ],
    ]);
    if (nextKey === key) return pending;
    key = nextKey;
    const started = ++generation;
    const isCurrent = (): boolean => !disposed && started === generation;
    deps.clear();
    pending = (async () => {
      try {
        if (dense) {
          if (dense.sourceProjectRevision !== project.identity.revision) return;
          const robot = dense.robot;
          if (robot && (robot.modelId !== project.robot.model ||
            robot.robotDescriptionSha256 !== project.robot.description?.robotDescriptionSha256)) return;
          const bytes = await deps.readTrajectory(dense.trajectoryDigest);
          if (!isCurrent() || !bytes) return;
          const decoded = decodeDenseTrajectory(bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes));
          if (decoded.trajectoryDigest !== dense.trajectoryDigest || decoded.schema !== dense.schema) {
            throw new Error("The stored trajectory does not match the selected plan.");
          }
          if (isCurrent()) await deps.show(project, decoded, isCurrent);
        } else if (planIsFresh(project) &&
          (project.plan.state === "preview" || project.plan.state === "server_validated") &&
          project.plan.trajectory.length > 0) {
          await deps.show(project, null, isCurrent);
        }
      } catch (error) {
        if (!isCurrent()) return;
        deps.clear();
        deps.onError?.(error);
      }
    })();
    return pending;
  }

  return {
    sync,
    dispose() {
      if (disposed) return;
      disposed = true;
      generation++;
      deps.clear();
    },
  };
}
