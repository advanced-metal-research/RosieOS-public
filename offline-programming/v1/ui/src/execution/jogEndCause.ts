/**
 * Why a client ended a jog or Cartesian hold: the End body's `end_cause`.
 * The 2026-10-10 analysis of a Rosie 1430 arm (fix B2) found 107 of 112 holds ended by
 * clients and every one logged "release", so the stops the operator noticed
 * could not be told from releases. The server journals the cause and words
 * it (internal/denseexec/jog_end.go: a real release stays released, a stick
 * direction change or lost focus pops up, a page or mode change the operator
 * chose does not). The Steam Deck pendant sends the same words
 * (steamdeck/real/v5/src/machine.cpp).
 */
export type JogEndCause =
  | "trigger_released" | "pointer_up" | "stick_neutral" | "key_up"
  | "stick_direction_change"
  | "pointer_leave" | "pointer_cancel" | "pointer_capture_lost"
  | "focus_lost" | "visibility_hidden" | "page_hidden"
  | "controller_disconnected"
  | "mode_switch" | "speed_change" | "frame_change" | "ui_navigation" | "cell_change" | "app_closed" | "maintenance"
  | "operator_stop" | "operator_disarm" | "selection_changed" | "session_changed"
  | "jog_disabled" | "state_poll_failed"
  | `intent_refused:${string}` | `server_reason:${string}`;

/** A jog button's release event. */
export function pointerEndCause(event: string): JogEndCause {
  switch (event) {
    case "pointerleave": return "pointer_leave";
    case "pointercancel": return "pointer_cancel";
    case "lostpointercapture": return "pointer_capture_lost";
    default: return "pointer_up";
  }
}

/** A window-level event that ends a held jog. */
export function windowEndCause(event: string): JogEndCause {
  switch (event) {
    case "blur": return "focus_lost";
    case "pagehide": return "page_hidden";
    case "visibilitychange": return "visibility_hidden";
    case "pointercancel": return "pointer_cancel";
    default: return "pointer_up";
  }
}

/** A request this hold made was refused: the refusal's code, which the
 *  server classifies like its own (a client error has none). */
export function refusalEndCause(error: unknown): JogEndCause {
  const code = error && typeof error === "object" && typeof (error as { code?: unknown }).code === "string" ? (error as { code: string }).code : "";
  return /^[a-z0-9_.:-]{1,64}$/.test(code) ? `intent_refused:${code}` : "intent_refused:client_error";
}
