/**
 * The viewport follows the connected machine.
 *
 * The Machine panel polls `/dense-execution/status` and the Jog panel already
 * prints the per-axis logical positions it carries; this module turns the
 * same observation into the one thing the 3D model can consume, a
 * `RobotPoseSnapshot` in the loaded model's joint order. It is pure: the
 * panel hands it the last status, the described axes and the status age, and
 * it answers with a pose to draw (or none) and the caption that says where
 * the pose came from.
 *
 * What it exists to prevent: the owner jogging the Los Angeles bench,
 * watching the joints turn on camera and in the Jog panel's radians, while
 * the model beside them sat at the program preview pose. The rt-core
 * backend published positions on every poll and nothing forwarded them.
 *
 * Mapping is by the model the machine reports (`status.target.model_id`),
 * never by assuming nine wire axes: rosie_1420_v1 describes six axes and
 * its URDF articulates J1..J6, rosie_1400_v3 describes nine. A described
 * axis is matched to a model joint by its id (J1..J9); an id the model does
 * not know falls back to its index. Only axes described in radians are
 * taken as joint angles; an axis without a valid position keeps the value
 * the viewer last showed for it, because a wrapped or invalid encoder
 * reading is not a position.
 */

import type { RobotPoseSnapshot } from "../simulation/contracts.ts";
import { robotModelFor } from "../viewer/models.ts";
import { robotModelClass } from "../targets/cells.ts";
import type { DenseCapabilities, DenseStatus } from "./denseClient.ts";
import { isMappedAxis } from "./denseClient.ts";
import { STATUS_FRESHNESS_MS } from "./machineFlow.ts";

export type LivePoseMode = "live" | "stale" | "none";

export interface LivePoseView {
  /** `live`: fresh status, the pose is the machine's. `stale`: connected but
   *  the status is past the freshness bound or carries no observation; the
   *  pose is not updated. `none`: no machine connected; the preview owns the viewport. */
  mode: LivePoseMode;
  /** The pose to draw now, or null when nothing fresh arrived. */
  pose: RobotPoseSnapshot | null;
  /** The viewport caption for this state, or null when the preview owns it. */
  caption: string | null;
  /** Fresh measured joint values only; held/default viewer angles are never measurements. */
  jointCaption?: string;
}

export interface LivePoseInput {
  /** A target is selected on the Machine panel (its status poll runs). */
  connected: boolean;
  status: DenseStatus | null;
  capabilities: DenseCapabilities | null;
  /** Milliseconds since the server last answered the status poll; null before the first answer. */
  statusAgeMs: number | null;
  nowMs: number;
  /** The live pose last drawn, so axes without a position keep their value. */
  previous: RobotPoseSnapshot | null;
  /** The connected cell's label for the caption. */
  cellLabel: string;
  /** The model to pose when the status names none (the single joint-daemon target). */
  fallbackModelId: string;
}

/** Radians are the only unit the URDF joints take. Unverified spelling set:
 *  rt-core's described `position_unit` for the bench's rotary axes is `rad`. */
const RADIAN_UNITS = new Set(["rad", "radian", "radians"]);

type NativeAxis = NonNullable<DenseStatus["rtCore"]>["axes"][number];

function nativePositionValid(axis: NativeAxis | undefined): boolean {
  return !!axis && axis.homeValid && axis.coordinateValid && axis.calibrationValid !== false &&
    (axis.positionState === undefined || axis.positionState === null || axis.positionState === "trusted") &&
    axis.position !== null && Number.isFinite(axis.position);
}

function nativeJointReadout(status: DenseStatus, capabilities: DenseCapabilities | null): { text: string; partial: boolean } | null {
  if (status.backend !== "rt_core") return null;
  const described = capabilities?.rtCore?.axes ?? [];
  if (!described.length) return null;
  let partial = false;
  const text = described.map((description, index) => {
    const axis = status.rtCore?.axes[index];
    if (RADIAN_UNITS.has(description.position_unit) && nativePositionValid(axis)) {
      return `${description.id} ${(axis!.position! * 180 / Math.PI).toFixed(1)}°`;
    }
    partial = true;
    const reason = axis?.homeValid === false ? "Home required" : "position unverified";
    return `${description.id} unavailable (${reason})`;
  }).join(" ");
  return { text, partial };
}

/**
 * Which model joint a described machine axis maps onto: the joint whose name
 * is the axis id, else the axis index. `mapLivePose` writes that way, and the
 * preview column reads the reverse — a sample is in `jointOrder`, the jog
 * row is an axis. An id the model does not know falls back to index so a
 * bench that names axes `axis_a` still lines up by wire order rather than
 * going blank.
 */
export function jointIndexForAxis(
  jointOrder: readonly string[],
  axisId: string | undefined,
  axisIndex: number,
): number {
  const byId = axisId ? jointOrder.indexOf(axisId) : -1;
  return byId >= 0 ? byId : axisIndex;
}

/**
 * The preview sample's angle for one machine-axis row, in radians. Null when
 * there is no sample, or that row has no joint (index past the sample).
 */
export function previewJointRadForAxis(
  sample: { jointOrder: readonly string[]; jointsRad: readonly number[] } | null,
  axisId: string | undefined,
  axisIndex: number,
): number | null {
  if (!sample) return null;
  const joint = jointIndexForAxis(sample.jointOrder, axisId, axisIndex);
  const value = sample.jointsRad[joint];
  return value !== undefined && Number.isFinite(value) ? value : null;
}

/**
 * The status's axis positions as a pose for the model the machine reports.
 * Null when the status carries no usable position (no observation, an
 * unknown model, every axis invalid) so the caller draws nothing new.
 */
export function mapLivePose(
  status: DenseStatus | null,
  capabilities: DenseCapabilities | null,
  previous: RobotPoseSnapshot | null,
  nowMs: number,
  fallbackModelId: string,
): RobotPoseSnapshot | null {
  if (!status) return null;
  const modelId = status.target?.model_id ?? fallbackModelId;
  if (robotModelClass(modelId) !== "arm") return null;
  const descriptor = robotModelFor(modelId);
  const jointOrder = [...descriptor.poseJointOrder];
  const jointsRad = previous && previous.modelId === modelId && previous.jointsRad.length === jointOrder.length
    ? [...previous.jointsRad]
    : [...descriptor.homePoseRad];
  let observed = 0;

  if (status.backend === "rt_core") {
    const axes = status.rtCore?.axes ?? [];
    const described = capabilities?.backend === "rt_core" ? capabilities.rtCore?.axes ?? [] : [];
    axes.forEach((axis, index) => {
      const description = described[index];
      // Without a description the unit is unknown; the Jog panel prints the
      // raw value with no unit in that case, and the model must not guess.
      if (!description || !RADIAN_UNITS.has(description.position_unit)) return;
      if (!nativePositionValid(axis)) return;
      const joint = jointIndexForAxis(jointOrder, description.id, index);
      if (joint >= jointOrder.length) return;
      jointsRad[joint] = axis.position!;
      observed++;
    });
  } else {
    // The legacy joint daemon reports all nine wire axes in radians; only the
    // cell's mapped, homed axes are positions (a wrapped ±π from an unhomed
    // encoder is not one). No feedback age means no feedback at all.
    if (!status.axes || status.daemon?.feedbackAgeMs === null) return null;
    const mask = status.daemon?.mappedAxisMask ?? null;
    status.axes.forEach((axis, index) => {
      const joint = axis.index >= 0 && axis.index < jointOrder.length ? axis.index : index;
      if (joint >= jointOrder.length) return;
      if (!isMappedAxis(mask, axis.index) || axis.homed === false || axis.qRad === null) return;
      jointsRad[joint] = axis.qRad;
      observed++;
    });
  }
  if (observed === 0) return null;
  return { modelId, jointOrder, jointsRad, source: "live", observedAtMs: nowMs };
}

/**
 * Whether the viewport should follow the machine now, with the pose and
 * caption for that state. Freshness is the Machine panel's own bound,
 * `STATUS_FRESHNESS_MS`: past it the caption says STALE and the pose stops
 * moving rather than holding a claim the server can no longer back.
 */
export function deriveLivePose(input: LivePoseInput): LivePoseView {
  if (!input.connected || !input.status) return { mode: "none", pose: null, caption: null };
  const modelId = input.status.target?.model_id ?? input.fallbackModelId;
  const age = input.statusAgeMs;
  const ageText = age === null ? "no status yet" : `status ${Math.round(age)} ms`;
  const who = `${input.cellLabel || "machine"} · ${modelId}`;
  const observed = input.status.backend === "rt_core"
    ? input.status.rtCore !== null && input.status.rtCore !== undefined
    : input.status.axes !== null;
  if (age === null || age > STATUS_FRESHNESS_MS || !observed) {
    return { mode: "stale", pose: null, caption: `LIVE (STALE) · ${who} · ${ageText}` };
  }
  const pose = mapLivePose(input.status, input.capabilities, input.previous, input.nowMs, input.fallbackModelId);
  const readout = nativeJointReadout(input.status, input.capabilities);
  return {
    mode: "live",
    pose,
    caption: readout?.partial ? `LIVE (PARTIAL) · ${who} · unverified joints held in viewer · ${ageText}`
      : pose ? `LIVE · ${who} · ${ageText}` : `LIVE · ${who} · no axis positions · ${ageText}`,
    ...(readout ? { jointCaption: readout.text } : {}),
  };
}
