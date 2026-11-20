import type {
  PlannerAdapter,
  SimulationAdapter,
} from "../adapters/interfaces.ts";
import type {
  OfflineProgrammingProject,
  PlanTrajectorySample,
} from "../domain/project.ts";
import type { NormalizedPlanEvidence } from "../execution/contracts.ts";
import {
  LocalSimulatorClient,
  type LocalSimulatorClientOptions,
} from "./local-client.ts";
import {
  localSimulatorReady,
  type LocalPlanResult,
  type LocalSimulatorStatus,
} from "./local-contracts.ts";
import { buildLocalPlanRequest } from "./local-plan.ts";

export interface LocalPlaybackSink {
  load(trajectory: readonly PlanTrajectorySample[]): void | Promise<void>;
  stop(): void | Promise<void>;
}

export class LocalPlannerAdapter implements PlannerAdapter {
  readonly client: LocalSimulatorClient;
  status: LocalSimulatorStatus | null = null;
  lastResult: LocalPlanResult | null = null;
  #requestSequence = 0;
  #now: () => number;

  constructor(
    clientOptions: LocalSimulatorClientOptions = {},
    client = new LocalSimulatorClient(clientOptions),
  ) {
    this.client = client;
    this.#now = clientOptions.now ?? Date.now;
  }

  get enabled(): boolean {
    return localSimulatorReady(this.status, this.#now());
  }

  async refresh(signal?: AbortSignal): Promise<LocalSimulatorStatus> {
    this.status = await this.client.status(signal);
    return this.status;
  }

  async start(signal?: AbortSignal): Promise<LocalSimulatorStatus> {
    this.status = await this.client.start(signal);
    return this.status;
  }

  async stop(signal?: AbortSignal): Promise<LocalSimulatorStatus> {
    this.lastResult = null;
    this.status = await this.client.stop(signal);
    return this.status;
  }

  async plan(
    project: OfflineProgrammingProject,
    signal?: AbortSignal,
  ): Promise<NormalizedPlanEvidence> {
    const result = await this.planTransient(project, signal);
    this.lastResult = result;
    return result.evidence;
  }

  async planTransient(
    project: OfflineProgrammingProject,
    signal?: AbortSignal,
  ): Promise<LocalPlanResult> {
    if (!this.enabled) {
      throw new Error("local planner is not ready with a fresh no-motion proof");
    }
    this.#requestSequence += 1;
    const request = buildLocalPlanRequest(project, {
      requestId: `${project.identity.id}:local:${project.identity.revision}:${this.#requestSequence}`,
      submittedAt: new Date(this.#now()).toISOString(),
    });
    const result = await this.client.plan(request, signal);
    this.status = await this.client.status(signal);
    if (!localSimulatorReady(this.status, this.#now())) {
      throw new Error(
        "local planner post-plan status is not ready with a fresh no-motion proof",
      );
    }
    return result;
  }
}

// LocalSimulationAdapter feeds only the normalized local trajectory into the
// existing browser playback sink. It has no network command, arm, jog, play,
// weld, or physical-stop path.
export class LocalSimulationAdapter implements SimulationAdapter {
  readonly kind = "loopback" as const;
  #planner: LocalPlannerAdapter;
  #sink: LocalPlaybackSink;

  constructor(planner: LocalPlannerAdapter, sink: LocalPlaybackSink) {
    this.#planner = planner;
    this.#sink = sink;
  }

  get enabled(): boolean {
    return this.#planner.enabled;
  }

  async load(trajectory: PlanTrajectorySample[]): Promise<void> {
    if (!this.enabled) {
      throw new Error("local simulator is not ready with a fresh no-motion proof");
    }
    validateTrajectory(trajectory);
    await this.#sink.load(structuredClone(trajectory));
  }

  async stop(): Promise<void> {
    await this.#sink.stop();
  }
}

function validateTrajectory(trajectory: readonly PlanTrajectorySample[]): void {
  if (trajectory.length < 1) throw new Error("local trajectory is empty");
  let previousTime = -1;
  for (const [index, point] of trajectory.entries()) {
    if (
      point.index !== index ||
      !Number.isSafeInteger(point.rowIndex) ||
      point.rowIndex < 0 ||
      !Number.isFinite(point.timeS) ||
      point.timeS < previousTime ||
      point.tcpPose.some((value) => !Number.isFinite(value)) ||
      point.jointPositionsRad.some((value) => !Number.isFinite(value))
    ) {
      throw new Error(`local trajectory sample ${index} is invalid`);
    }
    previousTime = point.timeS;
  }
}
