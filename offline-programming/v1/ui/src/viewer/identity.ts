import type { RobotPoseSnapshot } from "../simulation/contracts.ts";
import type { RobotModelDescriptor } from "./models.ts";

export type IdentityIssueCode =
  | "model-mismatch"
  | "robot-name-mismatch"
  | "joint-order-mismatch"
  | "joint-count-mismatch"
  | "non-finite-joint"
  | "missing-joint"
  | "missing-frame";

export interface IdentityIssue {
  code: IdentityIssueCode;
  message: string;
}

export interface IdentityValidation {
  ok: boolean;
  issues: IdentityIssue[];
}

export interface LoadedUrdfIdentity {
  robotName?: unknown;
  joints?: Readonly<Record<string, unknown>>;
  frames?: Readonly<Record<string, unknown>>;
  links?: Readonly<Record<string, unknown>>;
}

export type ResolvedPoseMode = "active" | "last-known" | "home";

export interface ResolvedPose<TModelId extends string = string> {
  mode: ResolvedPoseMode;
  pose: RobotPoseSnapshot<TModelId>;
  reason: string;
}

export function validateLoadedUrdfIdentity<TModelId extends string>(
  descriptor: RobotModelDescriptor<TModelId>,
  loaded: LoadedUrdfIdentity,
): IdentityValidation {
  const issues: IdentityIssue[] = [];
  if (loaded.robotName !== descriptor.urdfRobotName) {
    issues.push({
      code: "robot-name-mismatch",
      message: `URDF robot name ${String(loaded.robotName)} does not match ${descriptor.urdfRobotName}`,
    });
  }
  for (const jointName of descriptor.urdfJointNames) {
    if (!loaded.joints || !Object.hasOwn(loaded.joints, jointName)) {
      issues.push({ code: "missing-joint", message: `URDF is missing joint ${jointName}` });
    }
  }
  for (const frameName of descriptor.requiredFrames) {
    const inFrames = loaded.frames && Object.hasOwn(loaded.frames, frameName);
    const inLinks = loaded.links && Object.hasOwn(loaded.links, frameName);
    if (!inFrames && !inLinks) {
      issues.push({ code: "missing-frame", message: `URDF is missing frame ${frameName}` });
    }
  }
  return { ok: issues.length === 0, issues };
}

export function validatePoseIdentity<TModelId extends string>(
  descriptor: RobotModelDescriptor<TModelId>,
  pose: RobotPoseSnapshot<string>,
): IdentityValidation {
  const issues: IdentityIssue[] = [];
  if (pose.modelId !== descriptor.id) {
    issues.push({
      code: "model-mismatch",
      message: `pose model ${pose.modelId} does not match loaded model ${descriptor.id}`,
    });
  }
  if (!sameStrings(pose.jointOrder, descriptor.poseJointOrder)) {
    issues.push({
      code: "joint-order-mismatch",
      message: `pose joint order must be ${descriptor.poseJointOrder.join(",")}`,
    });
  }
  if (pose.jointsRad.length !== descriptor.poseJointOrder.length) {
    issues.push({
      code: "joint-count-mismatch",
      message: `pose has ${pose.jointsRad.length} joints; expected ${descriptor.poseJointOrder.length}`,
    });
  }
  pose.jointsRad.forEach((value, index) => {
    if (!Number.isFinite(value)) {
      issues.push({
        code: "non-finite-joint",
        message: `pose joint ${pose.jointOrder[index] ?? index} is not finite`,
      });
    }
  });
  return { ok: issues.length === 0, issues };
}

/**
 * Holds only identity-valid poses. Going offline keeps the last accepted pose;
 * a model with no accepted pose resolves to its canonical home pose.
 */
export class PoseLatch<TModelId extends string = string> {
  readonly descriptor: RobotModelDescriptor<TModelId>;
  #last: RobotPoseSnapshot<TModelId> | null = null;
  #online = false;

  constructor(descriptor: RobotModelDescriptor<TModelId>) {
    this.descriptor = descriptor;
  }

  accept(
    pose: RobotPoseSnapshot<string>,
    online = true,
  ): IdentityValidation {
    const validation = validatePoseIdentity(this.descriptor, pose);
    if (!validation.ok) return validation;
    this.#last = clonePose(pose as RobotPoseSnapshot<TModelId>);
    this.#online = online;
    return validation;
  }

  markOffline(): void {
    this.#online = false;
  }

  clear(): void {
    this.#last = null;
    this.#online = false;
  }

  resolve(nowMs?: number, staleAfterMs = 1500): ResolvedPose<TModelId> {
    if (!this.#last) {
      return {
        mode: "home",
        pose: {
          modelId: this.descriptor.id,
          jointOrder: [...this.descriptor.poseJointOrder],
          jointsRad: [...this.descriptor.homePoseRad],
          source: "home",
        },
        reason: "no accepted pose; using canonical home pose",
      };
    }
    const observedAt = this.#last.observedAtMs;
    const stale = nowMs !== undefined &&
      observedAt !== undefined &&
      nowMs - observedAt >= staleAfterMs;
    const active = this.#online && !stale;
    return {
      mode: active ? "active" : "last-known",
      pose: clonePose(this.#last),
      reason: active ? "accepted identity-valid pose" : "offline or stale; holding last accepted pose",
    };
  }
}

function clonePose<TModelId extends string>(
  pose: RobotPoseSnapshot<TModelId>,
): RobotPoseSnapshot<TModelId> {
  return {
    ...pose,
    jointOrder: [...pose.jointOrder],
    jointsRad: [...pose.jointsRad],
  };
}

function sameStrings(left: readonly string[], right: readonly string[]): boolean {
  return left.length === right.length && left.every((value, index) => value === right[index]);
}
