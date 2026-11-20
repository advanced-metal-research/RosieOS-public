import type { DenseCapabilities, DenseStatus } from "./denseClient.ts";
import { STATUS_FRESHNESS_MS } from "./machineFlow.ts";

export type HomeState = "homed" | "required" | "homing" | "recovering" | "lost" | "calibration" | "stale" | "unavailable";
export interface JointHomeStatus { index: number; id: string; state: HomeState; label: string; detail: string }
export interface HomeStatusView { state: HomeState; label: string; detail: string; axes: JointHomeStatus[] }
const labels: Record<HomeState, string> = {
  recovering: "Calibrated · recovering position", lost: "Calibrated · position recovery required", calibration: "Calibration missing/mismatched",
  homed: "Homed", required: "Home required", homing: "Homing", stale: "Status stale", unavailable: "Status unavailable",
};

/** Presentation only: a request or old success never establishes Home validity. */
export function deriveHomeStatus(input: {
  status: DenseStatus | null; capabilities: DenseCapabilities | null; connected: boolean;
  statusAgeMs: number | null; pendingAxes?: readonly number[] | null;
}): HomeStatusView {
  const { status, capabilities, connected, statusAgeMs, pendingAxes } = input;
  const observation = status?.rtCore;
  const stale = connected && statusAgeMs !== null && statusAgeMs > STATUS_FRESHNESS_MS;
  const unavailable = !connected || !observation || statusAgeMs === null;
  const described = capabilities?.rtCore?.axes ?? [];
  const indexes = new Set([...described.map((axis, index) => axis.index ?? index), ...(observation?.axes.map((_, index) => index) ?? [])]);
  const axes = [...indexes].sort((a, b) => a - b).map(index => {
    const axis = observation?.axes[index];
    const id = described.find((axis, position) => (axis.index ?? position) === index)?.id ?? `J${index + 1}`;
    let state: HomeState;
    let detail = "";
    if (stale) { state = "stale"; detail = "No fresh machine observation; Home validity is unconfirmed."; }
    else if (unavailable || !axis || typeof axis.homeValid !== "boolean") {
      state = "unavailable"; detail = connected ? "The machine has not reported this joint's Home status." : "Connect to a machine to read Home status.";
    } else if (axis.homeActive || pendingAxes?.includes(index)) {
      state = "homing"; detail = "Home is in progress; waiting for the machine to confirm validity.";
    } else if (typeof axis.calibrationValid === "boolean") {
      const reason = observation?.operatorFeedback?.axes?.find(axis => axis.index === index)?.homeLoss;
      state = !axis.calibrationValid ? "calibration" : axis.positionState === "trusted" && axis.coordinateValid && axis.homeValid ? "homed"
        : axis.positionState === "recovering" ? "recovering" : axis.positionState === "lost" || axis.positionState === "unverified" ? "lost" : "unavailable";
      detail = reason || (state === "homed" ? "Calibration and current position verified by the machine." : `Position check ${axis.positionReason ?? "unknown"}; audit check ${axis.auditReason ?? "unknown"}.`);
    } else if (axis.homeValid) {
      state = "homed"; detail = axis.coordinateValid === false ? "Home is valid; live joint position is unavailable." : "Legacy Home status: separate calibration and recovery status is unavailable.";
    } else {
      state = "required";
      detail = observation?.operatorFeedback?.axes?.find(axis => axis.index === index)?.homeLoss || "The machine reports no valid Home reference.";
    }
    return { index, id, state, label: state === "homed" && axis?.calibrationValid === true ? "Calibrated · position verified" : labels[state], detail };
  });
  const state: HomeState = stale ? "stale" : unavailable || !axes.length || axes.some(axis => axis.state === "unavailable") ? "unavailable"
    : axes.some(axis => axis.state === "homing") ? "homing" : axes.some(axis => axis.state === "calibration") ? "calibration" : axes.some(axis => axis.state === "lost") ? "lost" : axes.some(axis => axis.state === "recovering") ? "recovering" : axes.some(axis => axis.state === "required") ? "required" : "homed";
  const detail = state === "homed" ? `Home confirmed on all ${axes.length} joints.`
    : state === "required" ? `${axes.filter(axis => axis.state === "required").map(axis => axis.id).join(", ")} need a valid Home reference.`
    : state === "calibration" ? "Saved calibration is unavailable or does not match the drive. Review the per-joint reason."
    : state === "recovering" ? "Recovering current position from saved calibration while disarmed and stationary. No motion or Home offset write."
    : state === "lost" ? "Calibration is retained. Position recovery is required before motion; reset and Arm remain explicit."
    : state === "homing" ? "Home is in progress. Completion must be confirmed by fresh machine status."
    : state === "stale" ? "Machine status stopped updating. Previous Home results are not current confirmation."
    : !connected ? "Connect to a machine to read Home status." : status?.daemonError ? `Cannot confirm Home status: ${status.daemonError}` : "Waiting for a complete machine status; Home validity is unknown.";
  return { state, label: state === "homed" && observation?.axes.every(axis => axis.calibrationValid === true) ? "Calibrated · position verified" : labels[state], detail, axes };
}
