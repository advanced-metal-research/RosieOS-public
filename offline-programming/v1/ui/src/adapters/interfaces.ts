import type {
  OfflineProgrammingProject,
  PlanTrajectorySample,
} from "../domain/project.ts";
import type { NormalizedPlanEvidence } from "../execution/contracts.ts";

export interface PlannerAdapter {
  readonly enabled: boolean;
  plan(project: OfflineProgrammingProject, signal?: AbortSignal): Promise<NormalizedPlanEvidence>;
}

export interface SimulationAdapter {
  readonly kind: "browser_preview" | "loopback";
  readonly enabled: boolean;
  load(trajectory: PlanTrajectorySample[]): Promise<void>;
  stop(): Promise<void>;
}
