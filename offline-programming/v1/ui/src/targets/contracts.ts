import type { TargetMode } from "../domain/project.ts";

export type { TargetMode } from "../domain/project.ts";

export const TARGET_DESCRIPTOR_SCHEMA = "offline-programming.target-descriptor.v1" as const;

export type TargetKind = "offline_preview" | "local_simulator" | "real_cell";

export interface TargetCapabilities {
  monitoring: boolean;
  planning: boolean;
  simulation: boolean;
  realExecution: boolean;
  teleop: boolean;
  rawBrowserTransport: boolean;
}

export interface AxisDescriptor {
  index: number;
  name: string;
  urdfJoint: string;
  kind: "arm" | "external";
  /** Motor-to-joint ratio when the manifest declares one; null when the cell does not publish it (a Rosie 1430 cell's manifest and the bench manifests omit it and the server treats it as optional). */
  gearRatio: number | null;
  sign: -1 | 1;
  cartesianParticipates: boolean;
}

export interface TargetDescriptor {
  schema: typeof TARGET_DESCRIPTOR_SCHEMA;
  profileId: string;
  label: string;
  kind: TargetKind;
  manifestRef: string | null;
  manifestSchema: string | null;
  cellId: string | null;
  axes: AxisDescriptor[];
  capabilities: TargetCapabilities;
}

export interface SimulatorSafetyProof {
  profileId: string;
  statusSequence: number;
  observedAtMs: number;
  freshUntilMs: number;
  noRealMovement: boolean;
  ethercatEnabled: boolean;
  motionOutputEnabled: boolean;
}

export interface TargetObservation {
  profileId: string;
  manifestRef: string;
  robotModel: string;
  statusSequence: number;
  observedAtMs: number;
  freshUntilMs: number;
  connected: boolean;
}

export interface ReadonlyAuthorityStatus {
  targetProfileId: string;
  controllerId: string;
  controllerBootId: string;
  leaseId: string;
  fenceEpoch: number;
  campaignGeneration: number;
  statusSequence: number;
  freshUntilMs: number;
  transitionPending: boolean;
}

export function offlineTargetDescriptor(): TargetDescriptor {
  return {
    schema: TARGET_DESCRIPTOR_SCHEMA,
    profileId: "offline-preview",
    label: "None",
    kind: "offline_preview",
    manifestRef: null,
    manifestSchema: null,
    cellId: null,
    axes: [],
    capabilities: {
      monitoring: false,
      planning: false,
      simulation: true,
      realExecution: false,
      teleop: false,
      rawBrowserTransport: false,
    },
  };
}

export function isTargetDescriptor(value: unknown): value is TargetDescriptor {
  if (!isRecord(value) || value.schema !== TARGET_DESCRIPTOR_SCHEMA) return false;
  if (!nonempty(value.profileId) || !nonempty(value.label)) return false;
  if (!["offline_preview", "local_simulator", "real_cell"].includes(String(value.kind))) return false;
  if (!Array.isArray(value.axes) || !isRecord(value.capabilities)) return false;
  return value.axes.every(isAxisDescriptor) &&
    boolean(value.capabilities.monitoring) &&
    boolean(value.capabilities.planning) &&
    boolean(value.capabilities.simulation) &&
    boolean(value.capabilities.realExecution) &&
    boolean(value.capabilities.teleop) &&
    boolean(value.capabilities.rawBrowserTransport);
}

function isAxisDescriptor(value: unknown): value is AxisDescriptor {
  return isRecord(value) &&
    Number.isInteger(value.index) &&
    nonempty(value.name) &&
    nonempty(value.urdfJoint) &&
    (value.kind === "arm" || value.kind === "external") &&
    (value.gearRatio === null || (Number.isFinite(value.gearRatio) && Number(value.gearRatio) > 0)) &&
    (value.sign === -1 || value.sign === 1) &&
    boolean(value.cartesianParticipates);
}

function boolean(value: unknown): value is boolean {
  return typeof value === "boolean";
}

function nonempty(value: unknown): value is string {
  return typeof value === "string" && value.trim().length > 0;
}

export function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}
