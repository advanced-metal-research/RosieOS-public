import {
  TARGET_DESCRIPTOR_SCHEMA,
  type AxisDescriptor,
  type TargetDescriptor,
  isRecord,
} from "./contracts.ts";
import { CapabilitiesDigest, ContractVersion, type Description } from "../../../../../rt-core/clients/ts/rt_control_api_generated.ts";

/** The public description is the target manifest for dense execution. It cannot
 * supply URDF, factory or pair-service evidence, so those fields stay unavailable. */
export function manifestFromRTCore(description: Description, addressHost?: unknown) {
  if (description.contract_version !== ContractVersion || description.capabilities_digest !== CapabilitiesDigest ||
      !/^[a-f0-9]{64}$/.test(description.machine_sha256) || !/^[a-f0-9]{64}$/.test(description.deployment_sha256) ||
      !Array.isArray(description.axes) || description.axes.length < 1 || description.axes.length > 9) {
    throw new Error("rt_core description identity is incompatible");
  }
  const supports = (name: string) => description.capabilities?.some(c => c.name === name && c.state === "implemented") === true;
  return {
    backend: "rt_core" as const,
    nativeBackend: description.backend,
    simulation: description.backend === "simulation",
    addressHost: rtCoreAddressHost(addressHost),
    configurationSHA256: description.configuration_sha256,
    io: description.io ?? null,
    machineSHA256: description.machine_sha256,
    deploymentSHA256: description.deployment_sha256,
    axes: description.axes,
    monitoring: supports("status") && supports("subscribe_events"),
    programExecution: ["acquire", "renew", "release", "enable", "arm", "stop", "prepare_program", "start_program"].every(supports),
    nativeHome: supports("home"),
    pairRevision: null,
    factoryHash: null,
    urdf: null,
    homeProfile: null,
  };
}

/** Accept only a host for display, preventing paths or userinfo from entering the panel. */
function rtCoreAddressHost(value: unknown): string | null {
  if (typeof value !== "string" || !value || /[\s/\\?#@]/.test(value)) return null;
  try {
    const url = new URL(`https://${value}`);
    return url.hostname && !url.username && !url.password && url.pathname === "/" && !url.search && !url.hash
      ? url.host : null;
  } catch { return null; }
}

/** Render the actual described axes; no nine-axis model or local target is inferred. */
export function rtCoreTargetText(manifest: ReturnType<typeof manifestFromRTCore> | null | undefined): string {
  if (!manifest) return "rt-core unavailable";
  return `rt-core ${manifest.addressHost ? `remote ${manifest.addressHost}` : "local"} · ${manifest.nativeBackend} · ` +
    `${manifest.axes.length} axes (${manifest.axes.map(axis => axis.id).join(", ")})`;
}

const MANIFEST_SCHEMA = "robot_v4_deployment_manifest_v1";

// descriptorFromRobotV4Manifest consumes only identity-bearing manifest
// fields. Hosts, NATS subjects, UDP endpoints, and the command route are
// validated for exact cell binding, then intentionally omitted.
export function descriptorFromRobotV4Manifest(
  manifestRef: string,
  value: unknown,
): TargetDescriptor {
  if (!nonempty(manifestRef)) throw new Error("manifest reference is required");
  if (!isRecord(value) || value.schema_version !== MANIFEST_SCHEMA) {
    throw new Error(`manifest schema must be ${MANIFEST_SCHEMA}`);
  }
  const cellId = text(value.name);
  if (!safeIdentity(cellId)) throw new Error("manifest name must be a concrete cell identity");
  if (value.robot_command_subject !== `robot/v4/robot.${cellId}.command`) {
    throw new Error("robot command subject does not match manifest cell");
  }
  if (!isRecord(value.robot_axes) || !Array.isArray(value.robot_axes.axes)) {
    throw new Error("manifest robot axes are required");
  }
  const axes = value.robot_axes.axes.map(normalizeAxis);
  if (axes.length < 1 || axes.length > 9) {
    throw new Error("manifest must define between one and nine axes");
  }
  axes.forEach((axis, index) => {
    const name = `J${index + 1}`;
    if (axis.index !== index || axis.name !== name || axis.urdfJoint !== name) {
      throw new Error(`axis ${index} is not in canonical J1-J9 order`);
    }
  });

  if (!Array.isArray(value.nodes)) throw new Error("manifest nodes are required");
  const motion = uniqueRole(value.nodes, "motion-server");
  const realtime = uniqueRole(value.nodes, "realtime-core");
  const motionPeer = concreteNodeIdentity(motion);
  const realtimePeer = concreteNodeIdentity(realtime);
  const realtimeHost = text(realtime.host);
  requireSubject(motion, `robot/v4/motion-server.${motionPeer}.status`);
  requireSubject(motion, `robot/v4/motion-server.${motionPeer}.plan`);
  requireSubject(realtime, `robot/v4/rtcore.${realtimeHost}.status`);
  if (!realtimePeer) throw new Error("realtime-core peer identity is required");

  return {
    schema: TARGET_DESCRIPTOR_SCHEMA,
    profileId: cellId,
    label: cellId,
    kind: "real_cell",
    manifestRef: manifestRef.trim(),
    manifestSchema: MANIFEST_SCHEMA,
    cellId,
    axes,
    capabilities: {
      monitoring: true,
      planning: true,
      simulation: false,
      realExecution: false,
      teleop: false,
      rawBrowserTransport: false,
    },
  };
}

function normalizeAxis(value: unknown, index: number): AxisDescriptor {
  if (!isRecord(value)) throw new Error(`axis ${index} must be an object`);
  const kind = value.kind;
  if (kind !== "arm" && kind !== "external") {
    throw new Error(`axis ${index} has unsupported kind`);
  }
  const sign = Number(value.sign);
  const gearRatio = value.gear_ratio === undefined || value.gear_ratio === null ? null : Number(value.gear_ratio);
  if ((sign !== -1 && sign !== 1) || (gearRatio !== null && (!Number.isFinite(gearRatio) || gearRatio <= 0))) {
    throw new Error(`axis ${index} has invalid ratio or sign`);
  }
  return {
    index: integer(value.index, -1),
    name: text(value.name),
    urdfJoint: text(value.urdf_joint),
    kind,
    gearRatio,
    sign,
    cartesianParticipates: value.cartesian_participates === true,
  };
}

function uniqueRole(values: unknown[], role: string): Record<string, unknown> {
  const matches = values.filter(
    (entry): entry is Record<string, unknown> => isRecord(entry) && text(entry.role) === role,
  );
  if (matches.length !== 1) throw new Error(`manifest requires exactly one ${role} node`);
  return matches[0]!;
}

function concreteNodeIdentity(node: Record<string, unknown>): string {
  const peer = text(node.peer);
  const host = text(node.host);
  const role = text(node.role);
  const selector = text(node.node);
  if (!safeIdentity(peer) || !safeIdentity(host) || peer === role || host === role || peer === selector) {
    throw new Error(`${role || "manifest"} node requires concrete peer and host identities`);
  }
  return peer;
}

function requireSubject(node: Record<string, unknown>, expected: string): void {
  if (!Array.isArray(node.cold_path_subjects) || !node.cold_path_subjects.includes(expected)) {
    throw new Error(`manifest is missing exact subject ${expected}`);
  }
}

function safeIdentity(value: string): boolean {
  return value.length > 0 && value.length <= 96 && /^[A-Za-z0-9_.-]+$/.test(value);
}

function nonempty(value: unknown): value is string {
  return typeof value === "string" && value.trim().length > 0;
}

function text(value: unknown): string {
  return typeof value === "string" ? value.trim() : "";
}

function integer(value: unknown, fallback: number): number {
  return Number.isInteger(value) ? Number(value) : fallback;
}
