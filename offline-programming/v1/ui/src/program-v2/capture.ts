/**
 * The rules a recorded waypoint is made under, apart from the fetches and the
 * forward kinematics each app does its own way. Desktop OLP's recordWaypoint
 * and the Steam Deck pendant both apply them, so a waypoint captured on
 * either names the same description and calibration and is refused for the
 * same reasons.
 */
import type { CellSettings, OfflineProgrammingProject } from "../domain/project.ts";
import type { MoveNode, ProgramV2Node, TeachPose } from "../contracts/robot-v4-program-v2.ts";
import type { RobotPoseSnapshot } from "../simulation/contracts.ts";
import { robotModelFor } from "../viewer/models.ts";

/** A measured machine pose and the identity it was observed under (teach.ts). */
export interface MeasuredPose { pose: RobotPoseSnapshot; settings: CellSettings; cellId: string }

/** The capture model's identity, as POST /robots/capture-model answers it. */
export interface CaptureModelIdentity { robot_description_sha256: string; machine_planning_calibration_sha256: string }

/**
 * A capture refused because the program is not written for the robot it is
 * recording from: another model ("model"), or the same model under a robot
 * description the program has not accepted ("description"). Clients offer
 * the remedy (accept the current settings, or start another program) instead
 * of a status line the operator never sees.
 */
export class CaptureProgramMismatch extends Error {
  readonly kind: "model" | "description";
  constructor(kind: "model" | "description", text: string) {
    super(text);
    this.name = "CaptureProgramMismatch";
    this.kind = kind;
  }
}

/**
 * The description a capture is for: the machine's when it binds one, else the
 * program's pin. Either way it must be the program's, on the program's model.
 */
/**
 * Why a program cannot be taught or planned on the connected cell's robot,
 * or "" when the models agree (or no cell says which robot it is). A program
 * is written for one robot model; plans do not cross robots, so the remedy
 * is another program, never a silent conversion of this one's waypoints.
 */
export function programModelMismatch(programModel: string, cellModel: string): string {
  if (!cellModel || !programModel || programModel === cellModel) return "";
  return `This program is for ${programModel}; the connected cell is ${cellModel}. New starts a program for it, Open picks another`;
}

export function captureDescription(project: OfflineProgrammingProject, measured: MeasuredPose | null, snapshot: RobotPoseSnapshot): string {
  const mismatch = programModelMismatch(project.robot.model, snapshot.modelId);
  if (mismatch) throw new CaptureProgramMismatch("model", mismatch);
  const description = measured?.settings.robotDescriptionSha256 ?? project.robot.description?.robotDescriptionSha256;
  if (!description || snapshot.modelId !== project.robot.model || description !== project.robot.description?.robotDescriptionSha256) {
    throw new CaptureProgramMismatch("description", "Adopt the matching robot description before recording a waypoint");
  }
  return description;
}

/**
 * An unbound simulator uses the authored model, just like simulation
 * planning; a machine that binds a description must supply its own
 * calibration, fetched now and still the one it was observed under.
 */
export function machineBoundCapture(measured: MeasuredPose | null): boolean {
  return measured?.settings.robotDescriptionSha256 != null;
}

export function checkCaptureCalibration(measured: MeasuredPose | null, description: string, calibration: CellSettings | null | undefined): void {
  if (!machineBoundCapture(measured) || !measured) return;
  if (calibration?.cell !== measured.cellId || calibration.robotDescriptionSha256 !== description ||
      calibration.machinePlanningCalibrationSha256 !== measured.settings.machinePlanningCalibrationSha256) {
    throw new Error("Machine identity changed during capture; record again");
  }
  if (!calibration?.machinePlanningCalibrationBase64) throw new Error("Pull machine calibration before recording");
}

/** The capture model must be the one asked for; a via is recorded against its destination's calibration. */
export function checkCaptureModel(identity: CaptureModelIdentity, description: string, calibration: CellSettings | null | undefined,
  mode: "new" | "reteach" | "via", selected: ProgramV2Node | undefined): void {
  if (identity.robot_description_sha256 !== description || (calibration?.machinePlanningCalibrationSha256 && identity.machine_planning_calibration_sha256 !== calibration.machinePlanningCalibrationSha256)) {
    throw new Error("Capture calibration identity does not match; pull machine settings again");
  }
  if (mode === "via" && selected?.op === "move" && selected.capture.machine_planning_calibration_sha256 !== identity.machine_planning_calibration_sha256) {
    throw new Error("Reteach the destination against this calibration before recording its via pose");
  }
}

/** The destination and its capture record, from the snapshot and the TCP its FK gave (xyz m, URDF rpy rad). */
export function teachCapture(project: OfflineProgrammingProject, snapshot: RobotPoseSnapshot, tcp: readonly number[],
  description: string, identity: CaptureModelIdentity, measured: MeasuredPose | null, nowMs = Date.now()): { target: TeachPose; capture: MoveNode["capture"] } {
  const names = robotModelFor(project.robot.model).urdfJointNames;
  const target: TeachPose = { frame_id: "world", xyz_m: [tcp[0]!, tcp[1]!, tcp[2]!], rpy_rad: [tcp[3]!, tcp[4]!, tcp[5]!],
    joint_names: [...names], joint_values_rad: names.map(name => snapshot.jointsRad[snapshot.jointOrder.indexOf(name)]!) };
  const capture: MoveNode["capture"] = { source: measured ? "machine" : "preview", observed_at: new Date(snapshot.observedAtMs ?? nowMs).toISOString(),
    model_id: snapshot.modelId, robot_description_sha256: description, machine_planning_calibration_sha256: identity.machine_planning_calibration_sha256,
    ...(measured ? { cell_id: measured.cellId } : {}), pose: structuredClone(target) };
  return { target, capture };
}
