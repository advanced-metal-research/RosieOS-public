import type { Vec3 } from "../simulation/contracts.ts";
import { robotDescriptor } from "../domain/robots.ts";

export const CANONICAL_JOINT_ORDER = [
  "J1",
  "J2",
  "J3",
  "J4",
  "J5",
  "J6",
  "J7",
  "J8",
  "J9",
] as const;

export type CanonicalJointName = typeof CANONICAL_JOINT_ORDER[number];
/** The models whose viewer chrome is authored here: the public descriptions. */
export type CanonicalRobotModelId = "rosie_1400_v3" | "rosie_1420_v1";

export interface OrbitHomeDescriptor {
  readonly positionM: Vec3;
  readonly targetM: Vec3;
}

export interface TargetCameraMount {
  readonly kind: "target";
  readonly frame: string;
  readonly targetFrame: string;
  readonly lensOffsetLocalM: Vec3;
  readonly upLocal: Vec3;
  readonly fovDeg: number;
  readonly nearM: number;
  readonly farM: number;
}

export interface ForwardCameraMount {
  readonly kind: "forward";
  readonly frame: string;
  readonly lensOffsetLocalM: Vec3;
  readonly forwardLocal: Vec3;
  readonly upLocal: Vec3;
  readonly fovDeg: number;
  readonly nearM: number;
  readonly farM: number;
  readonly hideFrames?: readonly string[];
}

export interface RobotModelDescriptor<TModelId extends string = string> {
  readonly id: TModelId;
  readonly descriptorVersion: string;
  readonly displayName: string;
  readonly urdfRobotName: string;
  readonly urdfURL: string;
  readonly packageMap: Readonly<Record<string, string>>;
  readonly poseJointOrder: readonly string[];
  readonly urdfJointNames: readonly string[];
  readonly requiredFrames: readonly string[];
  readonly homePoseRad: readonly number[];
  /** The URDF link the workpiece is fixtured to — the positioner's table.
   *  Everything authored on the part (mesh, seams, rails) rides this link as
   *  the positioner turns; absent, the part sits still in the world. */
  readonly workFrameLink?: string;
  readonly worldUp: Vec3;
  readonly orbitHome: OrbitHomeDescriptor;
  readonly cameras: {
    readonly j6: TargetCameraMount;
    readonly tcp: ForwardCameraMount;
  };
}

const HOME_POSE = Object.freeze([0, 0, 0, 0, 0, 0, 0, 0, 0]);
const WORLD_UP = Object.freeze([0, 0, 1]) as Vec3;

const ROSIE_1400_V3: RobotModelDescriptor<"rosie_1400_v3"> = {
  id: "rosie_1400_v3",
  descriptorVersion: "robot_description",
  displayName: "Rosie 1400 V3",
  urdfRobotName: "rosie_1400_v3",
  urdfURL: "robot/rosie_1400_v3/robot.urdf",
  packageMap: { rosie_1400_v3: "robot/rosie_1400_v3" },
  poseJointOrder: CANONICAL_JOINT_ORDER,
  urdfJointNames: CANONICAL_JOINT_ORDER,
  requiredFrames: ["link_6", "tool0"],
  homePoseRad: HOME_POSE,
  workFrameLink: "positioner_table_a",
  worldUp: WORLD_UP,
  orbitHome: {
    positionM: [4.5, -5.8, 3.4],
    targetM: [0.9, 0.55, 0.65],
  },
  cameras: {
    // Guardrail: lens is 120 mm down link_6 local -Z, at the small shaft tip.
    j6: {
      kind: "target",
      frame: "link_6",
      targetFrame: "tool0",
      lensOffsetLocalM: [0, 0, -0.12],
      upLocal: [1, 0, 0],
      fovDeg: 62,
      nearM: 0.02,
      farM: 40,
    },
    tcp: {
      kind: "forward",
      frame: "tool0",
      // 55 mm tip offset minus 40 mm camera back-off.
      lensOffsetLocalM: [0.015, 0, 0],
      forwardLocal: [1, 0, 0],
      upLocal: [0, 0, 1],
      fovDeg: 68,
      nearM: 0.012,
      farM: 40,
      hideFrames: ["link_6", "end_effector", "tool0"],
    },
  },
};

const ROSIE_1420_V1: RobotModelDescriptor<"rosie_1420_v1"> = {
  id: "rosie_1420_v1",
  descriptorVersion: "robot_description",
  displayName: "Rosie 1420 V1",
  urdfRobotName: "rosie_1420_v1",
  urdfURL: "robot/rosie_1420_v1/robot.urdf",
  packageMap: { rosie_1420_v1: "robot/rosie_1420_v1" },
  // Project/playback contracts stay J1-J9 even though this URDF articulates J1-J6.
  poseJointOrder: CANONICAL_JOINT_ORDER,
  urdfJointNames: CANONICAL_JOINT_ORDER.slice(0, 6),
  requiredFrames: ["link_6", "tool0"],
  homePoseRad: HOME_POSE,
  worldUp: WORLD_UP,
  orbitHome: {
    positionM: [3.1, -3.9, 2.7],
    targetM: [0.7, 0.2, 1.05],
  },
  cameras: {
    j6: {
      kind: "target",
      frame: "link_6",
      targetFrame: "tool0",
      lensOffsetLocalM: [0, 0, -0.12],
      upLocal: [1, 0, 0],
      fovDeg: 62,
      nearM: 0.02,
      farM: 40,
    },
    tcp: {
      kind: "forward",
      frame: "tool0",
      lensOffsetLocalM: [0.015, 0, 0],
      forwardLocal: [1, 0, 0],
      upLocal: [0, 0, 1],
      fovDeg: 68,
      nearM: 0.012,
      farM: 40,
      hideFrames: ["link_6", "end_effector", "tool0"],
    },
  },
};

export const CANONICAL_ROBOT_MODELS: Readonly<
  Record<CanonicalRobotModelId, RobotModelDescriptor<CanonicalRobotModelId>>
> = Object.freeze({
  rosie_1400_v3: ROSIE_1400_V3,
  rosie_1420_v1: ROSIE_1420_V1,
});

export function isCanonicalRobotModelId(value: string): value is CanonicalRobotModelId {
  return Object.hasOwn(CANONICAL_ROBOT_MODELS, value);
}

export function canonicalRobotModel(
  modelId: CanonicalRobotModelId,
): RobotModelDescriptor<CanonicalRobotModelId> {
  return CANONICAL_ROBOT_MODELS[modelId];
}

/**
 * Viewer chrome for a robot family, keyed by the package its URDF names its
 * meshes under (the server's catalogue states it, from the description). A
 * Rosie 1430 is drawn the way the 1420 is; only its files are its own. The
 * robots of a family are their descriptions, so none is named here.
 */
const ROBOT_FAMILIES: Readonly<Record<string, (modelId: string) => RobotModelDescriptor>> = {
  rosie_1430_v1: (modelId) => ({
    ...ROSIE_1420_V1,
    id: modelId,
    displayName: `Rosie 1430 \u00b7 ${modelId}`,
    urdfRobotName: modelId,
    urdfURL: `robot/${modelId}/robot.urdf`,
    packageMap: { rosie_1430_v1: `robot/${modelId}` },
  }),
};

function familyModel(modelId: string): RobotModelDescriptor | null {
  const urdfPackage = robotDescriptor(modelId)?.urdfPackage;
  const make = urdfPackage ? ROBOT_FAMILIES[urdfPackage] : undefined;
  return make ? make(modelId) : null;
}

/**
 * A robot this viewer has no chrome for, drawn anyway.
 *
 * What a descriptor holds is two different things: where the model's files
 * are, which follows from its name and the way assets are staged, and how a
 * camera is hung on it, which somebody chose by looking at it. Only the
 * second needs authoring, so a model the server offers and this file has
 * never heard of renders with the arm's own conventions -- tool0 forward,
 * Z up -- and an orbit that frames a few metres around the base. The
 * alternative was refusing to draw a robot that exists.
 */
export function robotModelFor(modelId: string): RobotModelDescriptor {
  if (isCanonicalRobotModelId(modelId)) return canonicalRobotModel(modelId);
  const family = familyModel(modelId);
  if (family) return family;
  return {
    id: modelId,
    descriptorVersion: "robot_description",
    displayName: modelId,
    urdfRobotName: modelId,
    urdfURL: `robot/${modelId}/robot.urdf`,
    packageMap: { [modelId]: `robot/${modelId}` },
    poseJointOrder: CANONICAL_JOINT_ORDER,
    urdfJointNames: CANONICAL_JOINT_ORDER,
    requiredFrames: ["tool0"],
    homePoseRad: HOME_POSE,
    worldUp: WORLD_UP,
    orbitHome: { positionM: [3.1, -3.9, 2.7], targetM: [0.7, 0.2, 1.05] },
    cameras: {
      j6: {
        kind: "target", frame: "tool0", targetFrame: "tool0",
        lensOffsetLocalM: [0, 0, -0.12], upLocal: [1, 0, 0],
        fovDeg: 62, nearM: 0.02, farM: 40,
      },
      tcp: {
        kind: "forward", frame: "tool0",
        lensOffsetLocalM: [0.015, 0, 0], forwardLocal: [1, 0, 0], upLocal: [0, 0, 1],
        fovDeg: 68, nearM: 0.012, farM: 40, hideFrames: ["tool0"],
      },
    },
  };
}
