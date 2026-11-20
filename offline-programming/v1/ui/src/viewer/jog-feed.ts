/**
 * When the Motion Server's live jog pose may be written into the viewer.
 *
 * The jog state is polled at 10 Hz and the viewer shows ONE robot. A preview
 * -- a classic Tesseract playback or a rails dense trajectory -- poses the
 * same robot from its own samples, and a rails replay does not drive the jog
 * session at all, so the session sits at the reset pose while the preview
 * plays. Written on every poll, that idle pose put the robot and positioner
 * back at the reset pose between playback frames: the replay flashed.
 *
 * So the feed is edge-triggered. It writes a pose only when the jogged robot
 * has actually moved since the last one it wrote, and never while a preview
 * is playing. A paused or scrubbed preview therefore keeps the pose it drew,
 * and a real jog still shows the instant it moves.
 */

export interface JogFeedGate {
  /** Whether this jog pose should be written now. */
  admit(jointsRad: readonly number[], previewPlaying: boolean): boolean;
  /** Write the next jog pose whatever it is -- the operator asked to see the
   *  jogged robot again. */
  forget(): void;
}

/** Joints closer than this are the same pose: well under a hundredth of a
 *  degree, so encoder-free simulated state reads as still. */
export const JOG_POSE_TOLERANCE_RAD = 1e-5;

export function createJogFeedGate(toleranceRad = JOG_POSE_TOLERANCE_RAD): JogFeedGate {
  let last: number[] | null = null;
  return {
    admit(jointsRad, previewPlaying) {
      if (previewPlaying) return false;
      const unchanged = last !== null
        && last.length === jointsRad.length
        && jointsRad.every((value, index) => Math.abs(value - last![index]!) <= toleranceRad);
      if (unchanged) return false;
      last = [...jointsRad];
      return true;
    },
    forget() {
      last = null;
    },
  };
}
