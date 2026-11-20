import assert from "node:assert/strict";
import test from "node:test";
import {
  driveArmStatusPill,
  drivesAreArmed,
  robotTargetIsLive,
  sessionModeStatusPill,
  type RobotLiveEvidence,
} from "./sessionStatusPills.ts";
import type { ConnectedStatusObservation } from "./targets/connectedStreams.ts";

const nowMs = Date.parse("2026-07-30T08:00:00.000Z");

test("TestReviewRound2PostureRequiresDescribedSimulation", () => {
  for (const mode of ["simulation", "dry_run", "production"] as const) {
    assert.equal(sessionModeStatusPill(mode, false, { backend: "ethercat", simulation: false }).label, "REAL · PHYSICAL OUTPUT");
    assert.equal(sessionModeStatusPill(mode, false, { backend: "ethercat", simulation: true }).label, "REAL · PHYSICAL OUTPUT");
    assert.equal(sessionModeStatusPill(mode, false, { backend: "simulation", simulation: true }).label, "SIMULATION");
    assert.equal(sessionModeStatusPill(mode, false, { backend: "simulation" }).label, "OUTPUT UNKNOWN");
    assert.equal(sessionModeStatusPill(mode, false, { backend: "unreachable" }).label, "OUTPUT UNKNOWN");
  }
});

test("session mode pills stay simulation or dry-run even if a robot is live", () => {
  assert.equal(sessionModeStatusPill("simulation", true).label, "SIMULATION");
  assert.equal(sessionModeStatusPill("simulation", false).className, "is-offline");
  assert.equal(sessionModeStatusPill("dry_run", true).label, "DRY RUN");
  assert.equal(sessionModeStatusPill("dry_run", false).className, "");
});

test("production never says CONNECTED without live robot evidence", () => {
  const disconnected = sessionModeStatusPill("production", false);
  assert.equal(disconnected.label, "NO ROBOT");
  assert.equal(disconnected.className, "");
  const live = sessionModeStatusPill("production", true);
  assert.equal(live.label, "CONNECTED");
  assert.equal(live.className, "is-connected");
});

test("robotTargetIsLive fails closed on missing, unavailable, or stale status", () => {
  assert.equal(robotTargetIsLive(liveEvidence({ connected: false })), false);
  assert.equal(robotTargetIsLive(liveEvidence({ state: "unavailable" })), false);
  assert.equal(robotTargetIsLive(liveEvidence({ observation: null })), false);
  assert.equal(robotTargetIsLive(liveEvidence({ nowMs: nowMs + 10_000 })), false);
  assert.equal(robotTargetIsLive(liveEvidence({})), true);
});

test("drive arm pills fail closed unless actual servo armed is known true", () => {
  assert.equal(drivesAreArmed(null), false);
  assert.equal(drivesAreArmed({ known: false, value: true }), false);
  assert.equal(drivesAreArmed({ known: true, value: false }), false);
  assert.equal(drivesAreArmed({ known: true, value: true }), true);
  assert.equal(driveArmStatusPill(false).label, "DISARMED");
  assert.equal(driveArmStatusPill(false).className, "is-disarmed");
  assert.equal(driveArmStatusPill(true).label, "ARMED");
  assert.equal(driveArmStatusPill(true).className, "is-armed");
});

function liveEvidence(overrides: Partial<RobotLiveEvidence>): RobotLiveEvidence {
  return {
    connected: true,
    state: "read_only",
    observation: {
      connected: true,
      identityCorrelated: true,
      sourceUpdatedAtMs: nowMs,
      freshUntilMs: nowMs + 5_000,
    } as ConnectedStatusObservation,
    nowMs,
    ...overrides,
  };
}
