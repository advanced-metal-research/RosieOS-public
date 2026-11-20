/**
 * The live readout of a Cartesian path hold: what the robot is doing while
 * the trigger is held -- its speed against the speed asked for, what limits
 * it, how close the resolver's singularity gate is and the way out. On a Rosie 1430 arm
 * (2026-10-10) the speed collapsed near the wrist singularity over the last
 * millimetres and read as a jam; nothing on screen said why
 * (rt-core/docs/history/2026-10-10-singularity-and-jog-dropouts.md, A3/A5).
 *
 * The OLP server words it and publishes it as `jog_live` on the status body
 * (internal/denseexec/path_live.go); this only decides what to draw, for the
 * browser's jog pad and the Steam Deck jog page alike. The proximity meter
 * shows when the resolver's own singularity scale is below 1 (its slowing
 * band); it uses no threshold of its own. It stays up at the stop after a
 * hold that ended near a singularity, until the next hold or move.
 *
 * Bundled for Qt 5.15's engine (ES2016): no padStart, no Object.entries.
 */

export interface JogLive {
  active: boolean;
  axis: string;
  frame: string;
  speedUnit: string;
  requestedSpeed: number;
  speed: number;
  governing: string;
  limitKind: string;
  limit: string;
  gateScale: number | null;
  minGateScaleAhead: number | null;
  near: boolean;
  j5Deg: number | null;
  wristSingularDeg: number | null;
  wristDistanceDeg: number | null;
  lineEnd: string;
  distanceUnit: string;
  distanceToEnd: number | null;
  escape: string;
  jointHint: string;
  text: string;
}

export type JogLiveTone = "normal" | "caution" | "danger";

export interface JogLiveView {
  visible: boolean;
  active: boolean;
  /** PATH HOLD · +X, or LAST STOP · -X. */
  kicker: string;
  /** "21 / 200 mm/s" (empty at the stop). */
  speed: string;
  /** Speed as a fraction of the speed asked for, 0..1. */
  speedFraction: number;
  /** The server's line: "limited by: singularity (J5 −88.3°, …) — slowing, …". */
  text: string;
  meter: { visible: boolean; closeness: number; label: string };
  tone: JogLiveTone;
}

const HIDDEN: JogLiveView = {
  visible: false, active: false, kicker: "", speed: "", speedFraction: 0, text: "",
  meter: { visible: false, closeness: 0, label: "" }, tone: "normal",
};

function record(value: unknown): Record<string, unknown> {
  return value && typeof value === "object" && !Array.isArray(value) ? value as Record<string, unknown> : {};
}
function str(value: unknown): string {
  return typeof value === "string" ? value.trim() : "";
}
function num(value: unknown): number {
  return typeof value === "number" && isFinite(value) ? value : 0;
}
function opt(value: unknown): number | null {
  return typeof value === "number" && isFinite(value) ? value : null;
}

/** The server's `jog_live` (or a status body carrying it), or null. */
export function normalizeJogLive(value: unknown): JogLive | null {
  let body = record(value);
  if (body.jog_live !== undefined || body.path_live !== undefined) body = record(body.jog_live ?? body.path_live);
  if (typeof body.active !== "boolean" || !str(body.axis)) return null;
  return {
    active: body.active, axis: str(body.axis), frame: str(body.frame), speedUnit: str(body.speed_unit),
    requestedSpeed: num(body.requested_speed), speed: num(body.speed), governing: str(body.governing),
    limitKind: str(body.limit_kind), limit: str(body.limit), gateScale: opt(body.gate_scale),
    minGateScaleAhead: opt(body.min_gate_scale_ahead), near: body.near === true, j5Deg: opt(body.j5_deg),
    wristSingularDeg: opt(body.wrist_singular_deg), wristDistanceDeg: opt(body.wrist_distance_deg),
    lineEnd: str(body.line_end), distanceUnit: str(body.distance_unit), distanceToEnd: opt(body.distance_to_end),
    escape: str(body.escape), jointHint: str(body.joint_hint), text: str(body.text),
  };
}

/** What a client draws of the readout. */
export function jogLiveView(live: JogLive | null): JogLiveView {
  if (!live || !(live.active || live.near)) return HIDDEN;
  const scale = live.minGateScaleAhead ?? live.gateScale ?? 1;
  const closeness = Math.max(0, Math.min(1, 1 - scale));
  const label = live.wristDistanceDeg !== null && live.wristSingularDeg !== null
    ? `J5 ${Math.abs(live.wristDistanceDeg).toFixed(1)}° from ${live.wristSingularDeg.toFixed(0)}°`
    : `singularity gate ${Math.round(closeness * 100)} %`;
  const tone: JogLiveTone = live.near && closeness >= 1 ? "danger" : live.near || live.limitKind === "singularity" ? "caution" : "normal";
  return {
    visible: true, active: live.active,
    kicker: `${live.active ? "PATH HOLD" : "LAST STOP"} · ${live.axis}`,
    speed: live.active ? `${live.speed.toFixed(0)} / ${live.requestedSpeed.toFixed(0)} ${live.speedUnit}` : "",
    speedFraction: live.active && live.requestedSpeed > 0 ? Math.max(0, Math.min(1, live.speed / live.requestedSpeed)) : 0,
    text: live.text,
    meter: { visible: live.near, closeness, label },
    tone,
  };
}
