import {
  finalizeRobotV4Program,
  parseRobotV4AcceptedPlan,
  type RobotV4AcceptedPlan,
} from "../contracts/robot-v4-program.ts";
import type { OfflineProgrammingProject } from "../domain/project.ts";
import { toRobotV4Program } from "../adapters/robot-v4.ts";
import type { ConnectedTargetDiscoveryDescriptor } from "./connectedDiscovery.ts";

export const TARGET_PLAN_REQUEST_SCHEMA =
  "offline-programming.target-plan-request.v1" as const;
export const TARGET_PLAN_ENVELOPE_SCHEMA =
  "offline-programming.target-plan-envelope.v1" as const;

export interface TargetPlanClientOptions {
  fetch?: typeof fetch;
  now?: () => Date;
  createID?: () => string;
}

export class TargetPlanningClient {
  readonly #fetch: typeof fetch;
  readonly #now: () => Date;
  readonly #createID: () => string;

  constructor(options: TargetPlanClientOptions = {}) {
    this.#fetch = options.fetch ?? fetch;
    this.#now = options.now ?? (() => new Date());
    this.#createID = options.createID ?? (() => crypto.randomUUID());
  }

  async plan(
    project: OfflineProgrammingProject,
    target: ConnectedTargetDiscoveryDescriptor,
  ): Promise<RobotV4AcceptedPlan> {
    if (project.target.mode !== "real_connected" ||
        project.target.profileId !== target.profileId ||
        project.target.manifestRef !== target.manifestRef) {
      throw new Error("selected project target does not match the planning target");
    }
    if (project.program.executionMode !== "production") {
      throw new Error("physical target planning requires Production execution mode");
    }
    if (project.program.programState !== "saved_unplanned" ||
        !project.program.programDigest) {
      throw new Error("save this exact shared program revision before target planning");
    }
    const program = await finalizeRobotV4Program(toRobotV4Program(project));
    if (program.program_digest !== project.program.programDigest) {
      throw new Error("saved program digest changed; save the shared program again");
    }
    const requestID = `target-plan-${this.#createID()}`;
    const correlationID = `target-correlation-${this.#createID()}`;
    const idempotencyKey =
      `target-${target.profileId}-${program.program_revision}-${program.program_digest.slice(7)}`;
    const response = await this.#fetch(
      `/api/offline-programming/v1/targets/${encodeURIComponent(target.profileId)}/plans`,
      {
        method: "POST",
        headers: {
          "Accept": "application/json",
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          schema: TARGET_PLAN_REQUEST_SCHEMA,
          request_id: requestID,
          correlation_id: correlationID,
          idempotency_key: idempotencyKey,
          project_id: project.identity.id,
          project_revision: project.identity.revision,
          submitted_at: this.#now().toISOString(),
          program,
        }),
      },
    );
    const payload = await response.json() as unknown;
    if (!response.ok) {
      throw new Error(targetPlanError(payload, response.status));
    }
    if (!record(payload) ||
        payload.schema !== TARGET_PLAN_ENVELOPE_SCHEMA) {
      throw new Error("target planning returned an unsupported envelope");
    }
    const plan = parseRobotV4AcceptedPlan(payload.plan);
    if (plan.authority_classification !== "physical_target" ||
        !plan.evidence.runtime_playable ||
        plan.program_id !== program.program_id ||
        plan.program_revision !== program.program_revision ||
        plan.program_digest !== program.program_digest ||
        plan.target.target_id !== target.profileId ||
        plan.target.robot_model !== project.robot.model) {
      throw new Error("target planning returned a mismatched accepted plan");
    }
    return plan;
  }

  async get(planID: string): Promise<RobotV4AcceptedPlan> {
    const response = await this.#fetch(
      `/api/offline-programming/v1/plans/${encodeURIComponent(planID)}`,
      { headers: { "Accept": "application/json" } },
    );
    const payload = await response.json() as unknown;
    if (!response.ok || !record(payload) ||
        payload.schema !== TARGET_PLAN_ENVELOPE_SCHEMA) {
      throw new Error(targetPlanError(payload, response.status));
    }
    return parseRobotV4AcceptedPlan(payload.plan);
  }
}

function targetPlanError(payload: unknown, status: number): string {
  if (record(payload) && typeof payload.error === "string" &&
      payload.error.trim()) {
    return payload.error;
  }
  return `target planning HTTP ${status}`;
}

function record(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}
