import type {
  Rpy as ProjectRpy,
  TcpPose as ProjectTcpPose,
  Vec3 as ProjectVec3,
} from "../domain/project.ts";

export type Rpy = Readonly<ProjectRpy>;
export type TcpPose = Readonly<ProjectTcpPose>;
export type Vec3 = Readonly<ProjectVec3>;

export type MotionClass = "travel" | "weld";
export type PoseSource = "home" | "offline" | "playback" | "preview" | "live";

/**
 * Model identity travels with every pose. Integrators should map project/runtime
 * contracts into this shape instead of letting array position imply identity.
 */
export interface RobotPoseSnapshot<TModelId extends string = string> {
  modelId: TModelId;
  jointOrder: readonly string[];
  jointsRad: readonly number[];
  source: PoseSource;
  observedAtMs?: number;
}

export interface SimulationSample<TModelId extends string = string>
  extends RobotPoseSnapshot<TModelId> {
  timeS: number;
  tcpPose?: TcpPose;
  motion: MotionClass;
  seamId?: string | null;
  rowIndex?: number;
  /** Authored node owning this planned sample, when the plan supplies one. */
  nodeId?: string | null;
}

export type PlaybackStatus = "empty" | "paused" | "playing" | "ended";

export interface PlaybackSnapshot<TModelId extends string = string> {
  status: PlaybackStatus;
  cursorS: number;
  durationS: number;
  playbackRate: number;
  loop: boolean;
  sample: SimulationSample<TModelId> | null;
  sampleIndex: number;
}
