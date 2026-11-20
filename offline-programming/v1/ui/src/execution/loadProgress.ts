/**
 * What LOAD TO ROBOT is doing while it runs: the trajectory's bytes are
 * uploaded to the cell and decoded there, and nothing on the wire reports how
 * far along the upload is. The OLP knows the size, so it shows an estimate
 * against the link's measured throughput, and says it is an estimate.
 *
 * The owner pressed Load on the LA bench and had no way to tell whether the
 * multi-megabyte upload was still moving (2026-09-21).
 */

/** Unverified as a floor: the LA bench link moved a 4.9 MB program in about
 *  6 s on 2026-09-20 (P8-37), about 0.8 MB/s, and the SDK budgets against a
 *  256 KiB/s floor. The estimate uses the measured rate; the bar never claims
 *  the last few percent until the cell answers. */
export const UPLOAD_ESTIMATE_BYTES_PER_S = 800_000;

/** The bar's fill for an upload of `bytes` after `elapsedS`: in [0, 0.96]
 *  while the estimate holds, so a finished upload is the cell's word, not
 *  the clock's. */
export function loadProgressFraction(bytes: number, elapsedS: number): number {
  const expected = expectedUploadSeconds(bytes);
  if (!(expected > 0) || !(elapsedS >= 0)) return 0;
  return Math.min(0.96, elapsedS / expected);
}

export function expectedUploadSeconds(bytes: number): number {
  if (!Number.isFinite(bytes) || bytes <= 0) return 0;
  return bytes / UPLOAD_ESTIMATE_BYTES_PER_S;
}

/** Whether the upload has outlived its estimate: the bar then sweeps instead
 *  of standing at 96%, because a stuck bar reads as a stuck upload. */
export function loadProgressOverdue(bytes: number, elapsedS: number): boolean {
  const expected = expectedUploadSeconds(bytes);
  return expected > 0 && elapsedS > expected;
}

export function formatMegabytes(bytes: number): string {
  return `${(bytes / 1_000_000).toFixed(1)} MB`;
}

/** The line under the bar while loading, when done, and when refused. */
export function loadProgressText(bytes: number, elapsedS: number): string {
  const expected = expectedUploadSeconds(bytes);
  const size = formatMegabytes(bytes);
  if (!(expected > 0)) return `Uploading to the robot · ${elapsedS.toFixed(0)} s`;
  if (elapsedS > expected) {
    return `Uploading ${size} to the robot · ${elapsedS.toFixed(0)} s, past the ${expected.toFixed(0)} s estimate; the cell has not answered yet`;
  }
  return `Uploading ${size} to the robot · ${elapsedS.toFixed(0)} s of about ${Math.max(1, expected).toFixed(0)} s`;
}

export function loadDoneText(bytes: number, elapsedS: number): string {
  return `Loaded ${formatMegabytes(bytes)} in ${elapsedS.toFixed(1)} s`;
}

export function loadFailedText(bytes: number, elapsedS: number): string {
  return `Load of ${formatMegabytes(bytes)} refused after ${elapsedS.toFixed(1)} s`;
}
