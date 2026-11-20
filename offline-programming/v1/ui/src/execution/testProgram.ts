/**
 * Test programs: a dense joint program a tool generated (`tuningreport
 * inertia-plan`), loaded from a file the operator picks and run through the
 * same Load and Play as a planner trajectory, so the OLP server keeps the
 * grant, the heartbeat deadman and STOP.
 *
 * What this exists to prevent: a test program mistaken for a planned one, or
 * played from somewhere it was not made for. The upload is checked by the
 * server against the connected cell (robot, configuration, binding, start
 * pose); everything here is labelling and gating that the server checks
 * again. Off unless the server says test programs are on.
 */

import type { DenseLoadIdentity, DenseStatus } from "./denseClient.ts";
import type { RunControl } from "./runControls.ts";

export const TEST_PROGRAM_SOURCE = "test_program" as const;

export interface DenseTestProgramProvenance {
  fileName: string;
  schema: string;
  generated: string;
  joint: string;
  durationS: number | null;
  windowDeg: number[];
  accelLevelsDegS2: number[];
  configurationSha256: string;
}

/** An uploaded program as the server answers it. */
export interface DenseTestProgram {
  trajectoryDigest: string;
  planId: string;
  programId: string;
  programDigest: string;
  manifestRevision: number;
  planRevision: number;
  fileName: string;
  fileSha256: string;
  bytes: number;
  uploadedAt: string;
  totalDurationS: number;
  totalSampleCount: number;
  robotModelId: string;
  machineConfigurationSha256: string;
  /** First sample of every program axis, rad, in the machine's axis order. */
  startRad: number[];
  provenance: DenseTestProgramProvenance | null;
}

const isRecord = (value: unknown): value is Record<string, unknown> =>
  typeof value === "object" && value !== null && !Array.isArray(value);
const str = (value: unknown): string => (typeof value === "string" ? value : "");
const num = (value: unknown): number => (typeof value === "number" && Number.isFinite(value) ? value : 0);
const nums = (value: unknown): number[] => (Array.isArray(value) ? value.filter((v): v is number => typeof v === "number" && Number.isFinite(v)) : []);

export function normalizeTestProgram(value: unknown): DenseTestProgram | null {
  if (!isRecord(value) || typeof value.trajectory_digest !== "string" || value.source !== TEST_PROGRAM_SOURCE) return null;
  const robot = isRecord(value.robot_cell) ? value.robot_cell : {};
  const trajectory = isRecord(value.trajectory) ? value.trajectory : {};
  const p = isRecord(value.provenance) ? value.provenance : null;
  return {
    trajectoryDigest: value.trajectory_digest,
    planId: str(value.plan_id), programId: str(value.program_id), programDigest: str(value.program_digest),
    manifestRevision: num(value.manifest_revision), planRevision: num(value.plan_revision),
    fileName: str(value.file_name), fileSha256: str(value.file_sha256), bytes: num(value.bytes), uploadedAt: str(value.uploaded_at),
    totalDurationS: num(trajectory.total_duration_s), totalSampleCount: num(trajectory.total_sample_count),
    robotModelId: str(robot.model_id), machineConfigurationSha256: str(robot.machine_configuration_sha256),
    startRad: nums(value.start_rad),
    provenance: p ? {
      fileName: str(p.file_name), schema: str(p.schema), generated: str(p.generated), joint: str(p.joint),
      durationS: typeof p.duration_s === "number" ? p.duration_s : null, windowDeg: nums(p.window_deg),
      accelLevelsDegS2: nums(p.accel_levels_deg_s2), configurationSha256: str(p.configuration_sha256),
    } : null,
  };
}

/** The ordinary Load request, naming the uploaded program and its source. */
export function testProgramLoadIdentity(program: DenseTestProgram): DenseLoadIdentity {
  return {
    trajectory_digest: program.trajectoryDigest, plan_id: program.planId, program_id: program.programId,
    program_digest: program.programDigest, manifest_revision: program.manifestRevision, plan_revision: program.planRevision,
    source: TEST_PROGRAM_SOURCE,
  };
}

const short = (digest: string): string => digest.replace(/^sha256:/, "").slice(0, 12);

/** One line that cannot be read as a planner program: the label, the file,
 *  what it moves and for how long, its digest and where it came from. */
export function testProgramLabel(program: DenseTestProgram): string {
  const p = program.provenance;
  const what = [
    p?.joint ? `${p.joint} only` : "",
    `${program.totalDurationS.toFixed(1)} s`,
    p && p.accelLevelsDegS2.length ? `${p.accelLevelsDegS2.map(a => a.toFixed(0)).join("/")} deg/s²` : "",
    p && p.windowDeg.length === 2 ? `${p.windowDeg[0]!.toFixed(1)}..${p.windowDeg[1]!.toFixed(1)} deg` : "",
  ].filter(Boolean).join(", ");
  const origin = p
    ? `${p.schema}${p.generated ? `, generated ${p.generated}` : ""}`
    : "no plan.json: provenance unknown";
  return `TEST PROGRAM, not from the planner · ${program.fileName || "unnamed .rdt"} · ${what} · trajectory ${short(program.trajectoryDigest)} · file ${short(program.fileSha256)} · configuration ${short(program.machineConfigurationSha256)} · ${origin}`;
}

/** The chosen files: exactly one .rdt and at most one plan .json. */
export function pickTestProgramFiles<F extends { name: string; size: number }>(files: readonly F[], maxBytes: number): { rdt: F; plan: F | null } | { error: string } {
  const rdts = files.filter(f => f.name.toLowerCase().endsWith(".rdt"));
  const plans = files.filter(f => f.name.toLowerCase().endsWith(".json"));
  if (rdts.length !== 1) return { error: rdts.length ? "Choose one .rdt program at a time" : "Choose the program's .rdt file (and its plan.json)" };
  if (plans.length > 1) return { error: "Choose at most one plan.json, the one generated with this program" };
  if (files.length !== rdts.length + plans.length) return { error: "Only the .rdt program and its plan.json can be loaded" };
  if (maxBytes > 0 && rdts[0]!.size > maxBytes) return { error: `The program is ${rdts[0]!.size} bytes; this server takes at most ${maxBytes}` };
  return { rdt: rdts[0]!, plan: plans[0] ?? null };
}

export function encodeBase64(bytes: Uint8Array): string {
  let binary = "";
  for (let i = 0; i < bytes.length; i += 0x8000) binary += String.fromCharCode(...bytes.subarray(i, i + 0x8000));
  return btoa(binary);
}

/** Whether the server's session is this uploaded program, loaded. */
export function testProgramSession(status: DenseStatus | null, program: DenseTestProgram | null): boolean {
  const session = status?.session;
  return !!program && !!session && session.source === TEST_PROGRAM_SOURCE && session.trajectoryDigest === program.trajectoryDigest;
}

export interface TestProgramControlsInput {
  /** The server says test programs are on (null: not asked yet). */
  enabled: boolean | null;
  targetSelected: boolean;
  cellPending: boolean;
  native: boolean;
  inFlight: boolean;
  daemonReachable: boolean;
  status: DenseStatus | null;
  program: DenseTestProgram | null;
  /** Every joint is at the program's first sample within its tolerance. */
  atStart: boolean;
}

export interface TestProgramControls {
  /** The block is shown only where test programs are on for a native cell. */
  visible: boolean;
  load: RunControl;
  play: RunControl;
  label: string;
}

export function deriveTestProgramControls(input: TestProgramControlsInput): TestProgramControls {
  const visible = input.enabled === true && input.targetSelected && input.native;
  const state = input.status?.session?.state ?? "";
  const running = state === "playing" || state === "paused";
  const base = !visible ? "Test programs are off on this server"
    : input.cellPending ? "Connecting to the machine"
    : input.inFlight ? "Another request is in progress"
    : !input.daemonReachable ? "Waiting for fresh machine status"
    : running ? "A program is running; STOP it before loading another"
    : "";
  const load: RunControl = { disabled: !!base, title: base || "Choose a tool-generated .rdt program and its plan.json; the server checks robot, configuration and start pose against this cell, then loads it" };
  const loaded = testProgramSession(input.status, input.program) && state === "loaded";
  const ready = input.status?.armed === true && input.status.rtCore?.ready === true;
  const playBlocker = !visible ? "Test programs are off on this server"
    : input.inFlight ? "Another request is in progress"
    : !input.daemonReachable ? "Waiting for fresh machine status"
    : !loaded ? "Load the test program first"
    : !ready ? "Arm the robot first"
    : !input.atStart ? "The arm is not at the program's start; load it again from the held pose"
    : "";
  const play: RunControl = { disabled: !!playBlocker, title: playBlocker || "Play the loaded test program; STOP and the heartbeat deadman work as for any program" };
  const label = input.program && testProgramSession(input.status, input.program)
    ? `${testProgramLabel(input.program)} · ${state}`
    : input.program ? testProgramLabel(input.program) : "";
  return { visible, load, play, label };
}
