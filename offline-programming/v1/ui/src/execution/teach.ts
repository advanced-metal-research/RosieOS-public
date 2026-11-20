/**
 * What a recorded waypoint may be made from: the machine's own measured pose,
 * never a preview, and only while it is verified and still. The Dense panel
 * and the Steam Deck pendant both record through these two functions, so a
 * pendant capture is admitted exactly when a desktop capture would be.
 */
import type { CellSettings } from "../domain/project.ts";
import type { RobotPoseSnapshot } from "../simulation/contracts.ts";
import { robotModelFor } from "../viewer/models.ts";
import type { DenseCapabilities, DenseStatus } from "./denseClient.ts";
import { jointIndexForAxis, mapLivePose } from "./livePose.ts";
import { isNoObservationReason, STATUS_FRESHNESS_MS } from "./machineFlow.ts";

/**
 * A Stop or Disarm result is the machine answering, not the machine gone:
 * only a no-observation reason (transport lost, adapter closed) or a stale
 * status reads as unreachable, so DISARM does not flip the pill to "unreachable".
 */
export function machineReachable(selected: boolean, status: DenseStatus | null, statusAgeMs: number | null): boolean {
  return selected && status?.backend === "rt_core" && !!status.rtCore &&
    !(status.daemonError && isNoObservationReason(status.daemonError)) &&
    (statusAgeMs ?? Infinity) <= STATUS_FRESHNESS_MS;
}

/** The machine's settings from its authenticated Describe and status, never daemon status alone. */
export function cellSettingsOf(status: DenseStatus | null, capabilities: DenseCapabilities | null, reachable: boolean,
  now = new Date()): CellSettings | null {
  const target = status?.target, manifest = capabilities?.rtCore;
  if (!target || !manifest || !reachable) return null;
  // The machine's identity under the description contract, when it has
  // one: which description it was compiled with and which machine_planning_calibration.json.
  const robot = status?.robotCell?.valid ? status.robotCell : null;
  return {
    cell: target.cell_id, robotModelId: target.model_id, jetsonId: null,
    runtimeMode: manifest.simulation ? "sim" : "live", backend: manifest.nativeBackend,
    mappedAxisMask: manifest.axes.reduce((mask, _, index) => mask | (1 << index), 0),
    rtcoreHost: target.host, configurationDigest: manifest.configurationSHA256,
    axes: manifest.axes, io: manifest.io, pulledAt: now.toISOString(),
    ...(robot ? { modelId: robot.modelId, robotDescriptionSha256: robot.robotDescriptionSha256, machinePlanningCalibrationSha256: robot.machinePlanningCalibrationSha256 } : {}),
  };
}

/** Positions held within 0.00002 rad since `sinceNs`, in one observation identity. */
export interface TeachStill { identity: string; positions: number[]; sinceNs: number; timeNs: number; receivedMs: number }

type NativeAxis = NonNullable<NonNullable<DenseStatus["rtCore"]>["axes"]>[number];

function verifiedStationary(axis: NativeAxis): boolean {
  return axis.homeValid && axis.coordinateValid && axis.calibrationValid === true && axis.positionState === "trusted" &&
    axis.position !== null && Number.isFinite(axis.position) &&
    (axis.velocity === null || (Number.isFinite(axis.velocity) && Math.abs(axis.velocity) <= 0.001));
}

/**
 * The stillness window after one status. Motion, a request in flight, an
 * unverified axis or a changed identity resets it; positions that stay within
 * 0.00002 rad extend it to the newer native time.
 */
export function nextTeachStill(previous: TeachStill | null, status: DenseStatus, observationIdentity: string,
  busy: boolean, nowMs = Date.now()): TeachStill | null {
  const native = status.rtCore;
  const identity = `${observationIdentity}:${status.robotCell?.robotDescriptionSha256}:${status.robotCell?.machinePlanningCalibrationSha256}`;
  const positions = native?.axes.map(axis => axis.position);
  if (!native || !positions?.length || busy || native.axes.some(axis => !verifiedStationary(axis))) return null;
  if (!previous || previous.identity !== identity || nowMs - previous.receivedMs > STATUS_FRESHNESS_MS ||
      positions.length !== previous.positions.length || positions.some((position, i) => Math.abs(position! - previous.positions[i]!) > 0.00002)) {
    return { identity, positions: positions as number[], sinceNs: native.timeNs, timeNs: native.timeNs, receivedMs: nowMs };
  }
  if (native.timeNs > previous.timeNs) return { ...previous, timeNs: native.timeNs, receivedMs: nowMs };
  return previous;
}

/**
 * The measured pose a recording is made from, with the identity it was
 * observed under, or the reason there is none. `busy` covers a request in
 * flight, a held jog and a playing program; `settings` is the machine's
 * status-only settings (Dense panel `cellSettings`).
 */
export function teachObservationOf(input: {
  reachable: boolean; busy: boolean; status: DenseStatus | null; capabilities: DenseCapabilities | null;
  still: TeachStill | null; settings: CellSettings | null; observedAtMs: number; cellModel: string; cellId: string;
}): { pose: RobotPoseSnapshot; settings: CellSettings; cellId: string } {
  const { status, capabilities, settings } = input;
  if (!input.reachable || input.busy) throw new Error("Release jog and wait for fresh stationary feedback before recording");
  const axes = status?.rtCore?.axes;
  const descriptions = capabilities?.rtCore?.axes;
  if (!axes?.length || !descriptions?.length || axes.length !== descriptions.length ||
      axes.some((axis, i) => !verifiedStationary(axis) || descriptions[i]?.position_unit !== "rad")) {
    throw new Error("Record needs verified position and stationary feedback for every described joint");
  }
  // Velocity PDOs are optional. When absent, require fresh positions that have
  // stayed within 0.00002 rad for 100 ms after releasing the controls.
  if (axes.some(axis => axis.velocity === null) && (!input.still || input.still.timeNs - input.still.sinceNs < 100_000_000)) {
    throw new Error("Release jog and wait for fresh stationary position samples before recording");
  }
  const pose = mapLivePose(status!, capabilities, null, input.observedAtMs, input.cellModel);
  if (!pose || !settings) throw new Error("Machine pose is unavailable");
  if ((!settings.robotDescriptionSha256 && settings.runtimeMode !== "sim") || (status?.robotCell?.configured && !status.robotCell.valid)) throw new Error("Machine robot description is unavailable or invalid");
  const mapped = new Set(descriptions.map((axis, i) => jointIndexForAxis(pose.jointOrder, axis.id, i)));
  if (mapped.size !== descriptions.length || robotModelFor(pose.modelId).urdfJointNames.some(name => !mapped.has(pose.jointOrder.indexOf(name)))) throw new Error("Record needs measured feedback for every model joint");
  return { pose, settings, cellId: input.cellId };
}
