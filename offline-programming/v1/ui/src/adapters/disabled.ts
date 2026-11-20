import type {
  PlannerAdapter,
  SimulationAdapter,
} from "./interfaces.ts";
import type {
  OfflineProgrammingProject,
  PlanTrajectorySample,
} from "../domain/project.ts";
import type { NormalizedPlanEvidence } from "../execution/contracts.ts";

export class DisabledPlannerAdapter implements PlannerAdapter {
  readonly enabled = false;
  async plan(
    _project: OfflineProgrammingProject,
    _signal?: AbortSignal,
  ): Promise<NormalizedPlanEvidence> {
    throw new Error("Canonical server planning is disabled until milestone 2.");
  }
}

export class DisabledLocalSimulatorAdapter implements SimulationAdapter {
  readonly kind = "loopback" as const;
  readonly enabled = false;
  readonly reason =
    "Local loopback simulation is disabled until server-owned no-motion, EtherCAT-off, and motion-output-off proof is available.";

  async load(_trajectory: PlanTrajectorySample[]): Promise<void> {
    throw new Error(this.reason);
  }

  async stop(): Promise<void> {
    // No loopback process exists in milestone 1.
  }
}
