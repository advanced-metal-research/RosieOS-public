import {
  TARGET_DESCRIPTOR_SCHEMA,
  type AxisDescriptor,
  type TargetCapabilities,
  type TargetDescriptor,
  isRecord,
  isTargetDescriptor,
} from "./contracts.ts";

export const CONNECTED_TARGET_DISCOVERY_SCHEMA =
  "offline-programming.connected-target-discovery.v1" as const;

export interface ConnectedTargetServiceDescriptor {
  role: "motion-server" | "realtime-core";
  statusStream: boolean;
  planStream: boolean;
  readOnly: true;
  serverMediated: true;
}

export interface ConnectedTargetDiscoveryDescriptor {
  schema: typeof CONNECTED_TARGET_DISCOVERY_SCHEMA;
  profileId: string;
  manifestRef: string;
  manifestSchema: "robot_v4_deployment_manifest_v1";
  cellId: string;
  peerStaleAfterS: number;
  robotModelBound: boolean;
  services: ConnectedTargetServiceDescriptor[];
  target: TargetDescriptor;
}

// normalizeConnectedTargetDiscovery accepts the safe JSON emitted by the Go
// connected discovery descriptor. It intentionally has no binding/URL fields.
export function normalizeConnectedTargetDiscovery(
  value: unknown,
): ConnectedTargetDiscoveryDescriptor {
  if (!isRecord(value) || value.schema !== CONNECTED_TARGET_DISCOVERY_SCHEMA) {
    throw new Error(`connected discovery schema must be ${CONNECTED_TARGET_DISCOVERY_SCHEMA}`);
  }
  rejectUnknownKeys(value, [
    "schema",
    "profile_id",
    "manifest_ref",
    "manifest_schema",
    "cell_id",
    "peer_stale_after_s",
    "robot_model_bound",
    "services",
    "target",
  ]);
  const profileId = requiredText(value.profile_id, "profile_id");
  const manifestRef = requiredText(value.manifest_ref, "manifest_ref");
  const cellId = requiredText(value.cell_id, "cell_id");
  if (value.manifest_schema !== "robot_v4_deployment_manifest_v1") {
    throw new Error("connected discovery manifest schema is unsupported");
  }
  const peerStaleAfterS = positiveInteger(value.peer_stale_after_s);
  if (peerStaleAfterS < 1 || peerStaleAfterS > 300) {
    throw new Error("connected discovery peer freshness is out of bounds");
  }
  if (typeof value.robot_model_bound !== "boolean") {
    throw new Error("connected discovery robot_model_bound must be boolean");
  }
  if (!Array.isArray(value.services) || value.services.length !== 2) {
    throw new Error("connected discovery requires exactly two read-only services");
  }
  const services = value.services.map(normalizeService);
  if (services.filter(({ role }) => role === "motion-server").length !== 1 ||
      services.filter(({ role }) => role === "realtime-core").length !== 1) {
    throw new Error("connected discovery service roles are ambiguous");
  }
  const target = normalizeWireTarget(value.target);
  if (target.profileId !== profileId ||
      target.cellId !== cellId ||
      target.manifestRef !== manifestRef ||
      target.manifestSchema !== value.manifest_schema ||
      target.kind !== "real_cell") {
    throw new Error("connected discovery target identity mismatch");
  }
  if (target.capabilities.realExecution ||
      target.capabilities.teleop ||
      target.capabilities.rawBrowserTransport) {
    throw new Error("connected discovery enables an unavailable physical capability");
  }
  return {
    schema: CONNECTED_TARGET_DISCOVERY_SCHEMA,
    profileId,
    manifestRef,
    manifestSchema: value.manifest_schema,
    cellId,
    peerStaleAfterS,
    robotModelBound: value.robot_model_bound,
    services,
    target,
  };
}

function normalizeService(value: unknown): ConnectedTargetServiceDescriptor {
  if (!isRecord(value)) throw new Error("connected discovery service must be an object");
  rejectUnknownKeys(value, [
    "role",
    "status_stream",
    "plan_stream",
    "read_only",
    "server_mediated",
  ]);
  const role = value.role;
  if (role !== "motion-server" && role !== "realtime-core") {
    throw new Error("connected discovery service role is unsupported");
  }
  if (value.read_only !== true || value.server_mediated !== true ||
      typeof value.status_stream !== "boolean" ||
      typeof value.plan_stream !== "boolean") {
    throw new Error("connected discovery service is not read-only and server-mediated");
  }
  if (!value.status_stream ||
      (role === "motion-server" ? !value.plan_stream : value.plan_stream)) {
    throw new Error("connected discovery service stream contract is invalid");
  }
  return {
    role,
    statusStream: value.status_stream,
    planStream: value.plan_stream,
    readOnly: true,
    serverMediated: true,
  };
}

function normalizeWireTarget(value: unknown): TargetDescriptor {
  if (isTargetDescriptor(value)) return structuredClone(value);
  if (!isRecord(value) || value.schema !== TARGET_DESCRIPTOR_SCHEMA ||
      !Array.isArray(value.axes) || !isRecord(value.capabilities)) {
    throw new Error("connected discovery target descriptor is invalid");
  }
  const axes = value.axes.map(normalizeWireAxis);
  const capabilities = normalizeWireCapabilities(value.capabilities);
  const descriptor: TargetDescriptor = {
    schema: TARGET_DESCRIPTOR_SCHEMA,
    profileId: requiredText(value.profile_id, "target.profile_id"),
    label: requiredText(value.label, "target.label"),
    kind: value.kind === "real_cell" ? "real_cell" :
      value.kind === "local_simulator" ? "local_simulator" :
        value.kind === "offline_preview" ? "offline_preview" :
          fail("target.kind is unsupported"),
    manifestRef: nullableText(value.manifest_ref),
    manifestSchema: nullableText(value.manifest_schema),
    cellId: nullableText(value.cell_id),
    axes,
    capabilities,
  };
  if (!isTargetDescriptor(descriptor)) throw new Error("connected target descriptor did not normalize");
  return descriptor;
}

function normalizeWireAxis(value: unknown): AxisDescriptor {
  if (!isRecord(value)) throw new Error("connected target axis must be an object");
  const kind = value.kind;
  if (kind !== "arm" && kind !== "external") throw new Error("connected target axis kind is invalid");
  const sign = value.sign;
  if (sign !== -1 && sign !== 1) throw new Error("connected target axis sign is invalid");
  return {
    index: nonnegativeInteger(value.index),
    name: requiredText(value.name, "axis.name"),
    urdfJoint: requiredText(value.urdf_joint, "axis.urdf_joint"),
    kind,
    gearRatio: optionalPositiveFinite(value.gear_ratio),
    sign,
    cartesianParticipates: value.cartesian_participates === true,
  };
}

function normalizeWireCapabilities(value: Record<string, unknown>): TargetCapabilities {
  return {
    monitoring: requiredBoolean(value.monitoring, "monitoring"),
    planning: requiredBoolean(value.planning, "planning"),
    simulation: requiredBoolean(value.simulation, "simulation"),
    realExecution: requiredBoolean(value.real_execution, "real_execution"),
    teleop: requiredBoolean(value.teleop, "teleop"),
    rawBrowserTransport: requiredBoolean(value.raw_browser_transport, "raw_browser_transport"),
  };
}

function rejectUnknownKeys(value: Record<string, unknown>, allowed: string[]): void {
  const allowlist = new Set(allowed);
  const unknown = Object.keys(value).filter((key) => !allowlist.has(key));
  if (unknown.length > 0) throw new Error(`connected discovery contains unsupported field ${unknown[0]}`);
}

function requiredText(value: unknown, label: string): string {
  if (typeof value !== "string" || !value.trim() || value.length > 256) {
    throw new Error(`${label} is required`);
  }
  return value.trim();
}

function nullableText(value: unknown): string | null {
  return typeof value === "string" && value.trim() ? value.trim() : null;
}

function requiredBoolean(value: unknown, label: string): boolean {
  if (typeof value !== "boolean") throw new Error(`${label} must be boolean`);
  return value;
}

function positiveInteger(value: unknown): number {
  if (!Number.isSafeInteger(value) || Number(value) <= 0) throw new Error("positive integer required");
  return Number(value);
}

function nonnegativeInteger(value: unknown): number {
  if (!Number.isSafeInteger(value) || Number(value) < 0) throw new Error("nonnegative integer required");
  return Number(value);
}

function optionalPositiveFinite(value: unknown): number | null {
  if (value === undefined || value === null) return null;
  return positiveFinite(value);
}

function positiveFinite(value: unknown): number {
  if (typeof value !== "number" || !Number.isFinite(value) || value <= 0) {
    throw new Error("positive finite number required");
  }
  return value;
}

function fail(message: string): never {
  throw new Error(message);
}
