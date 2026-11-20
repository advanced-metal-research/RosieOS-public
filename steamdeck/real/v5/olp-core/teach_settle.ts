// Pendant-only capture settling. This does not grant or change motion.
import type { DenseCapabilities, DenseStatus } from "../../../../offline-programming/v1/ui/src/execution/denseClient.ts";
import type { TeachStill } from "../../../../offline-programming/v1/ui/src/execution/teach.ts";
import { STATUS_FRESHNESS_MS } from "../../../../offline-programming/v1/ui/src/execution/machineFlow.ts";
export class TeachSettle {
  private samples: { ns: number; q: number[]; target: (number | null)[] }[] = [];
  private identity = "";
  private received = 0;
  diagnostics: Record<string, unknown> = {};
  reset() { this.samples = []; this.identity = ""; this.diagnostics = {}; }
  observe(status: DenseStatus, capabilities: DenseCapabilities | null, identity: string, busy: boolean, now = Date.now()): TeachStill | null {
    const native = status.rtCore, axes = native?.axes;
    identity += `:${native?.coordinateIdentity}:${native?.coreArmed}:${status.robotCell?.robotDescriptionSha256}:${status.robotCell?.machinePlanningCalibrationSha256}`;
    const valid = !!axes?.length && axes.every(a => a.homeValid && a.coordinateValid && a.calibrationValid === true && a.positionState === "trusted" && a.position !== null && Number.isFinite(a.position) && (a.velocity === null || Number.isFinite(a.velocity) && Math.abs(a.velocity) <= .001));
    if (busy || !valid || !native || !axes) {
      this.samples = []; this.diagnostics = { busy, valid, reason: busy ? "motion_busy" : "unverified_feedback" }; return null;
    }
    if (identity !== this.identity || now - this.received > STATUS_FRESHNESS_MS) this.samples = [];
    this.identity = identity;
    const last = this.samples[this.samples.length - 1];
    if (!last || native.timeNs > last.ns) {
      this.samples.push({ ns: native.timeNs, q: axes.map(a => a.position!), target: axes.map(a => a.target) });
      this.received = now;
    }
    // Keep one frame just before the 250 ms boundary, at most 32 frames.
    while (this.samples.length > 2 && this.samples[1]!.ns <= native.timeNs - 250_000_000) this.samples.shift();
    if (this.samples.length > 32) this.samples.shift();
    const first = this.samples[0]!;
    const durationMs = (native.timeNs - first.ns) / 1e6;
    const joints = axes.map((a, i) => {
      const declared = capabilities?.rtCore?.axes[i]?.completion_tolerance;
      // No capability means the original strict fallback. Never expand beyond
      // the machine's completion precision or 0.0344 degrees.
      const toleranceRad = Number.isFinite(declared) && declared! > 0 ? Math.min(.0006, declared!) : .00002;
      const values = this.samples.map(s => s.q[i]!);
      const spanRad = Math.max(...values) - Math.min(...values);
      const targets = this.samples.map(s => s.target[i]);
      const targetsValid = targets.every(t => t !== null && Number.isFinite(t));
      const targetSpanRad = targetsValid ? Math.max(...targets as number[]) - Math.min(...targets as number[]) : null;
      const targetErrorRad = a.target === null ? null : Math.abs(a.position! - a.target);
      // A static following offset is diagnostic, not a reason to substitute
      // the commanded pose or refuse teaching the actual measured pose.
      const stable = spanRad <= toleranceRad && (!native.coreArmed || targetsValid && targetSpanRad! <= .00002);
      return { joint: capabilities?.rtCore?.axes[i]?.id ?? `J${i+1}`, spanRad, toleranceRad, targetSpanRad, targetErrorRad, stable };
    });
    const ready = this.samples.length >= 2 && durationMs >= 250 && joints.every(j => j.stable) && now - this.received <= STATUS_FRESHNESS_MS;
    this.diagnostics = { ready, busy, valid, durationMs, samples: this.samples.length, joints };
    return ready ? { identity, positions: axes.map(a => a.position!), sinceNs: first.ns, timeNs: native.timeNs, receivedMs: this.received } : null;
  }
}
