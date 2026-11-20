import {
  cellSettingsKey,
  cloneProject,
  cloneValue,
  missingPlan,
  type OfflineProgrammingProject,
  type PlanHistoryEntry,
} from "./project.ts";

/**
 * The plans a program has had, newest first, so a planner run that came
 * out worse than the last one is not the end of the last one. The
 * program still has ONE active plan -- `project.plan` and
 * `project.denseTrajectory`, what Simulate replays and Deploy loads --
 * and the history is where the others wait. Activating an entry copies
 * it back into those slots; the freshness checks (`planIsFresh`, the
 * dense record's source revision) then say whether it is still this
 * program's motion, which an older entry usually is not.
 *
 * Capped: a rails entry is a record (the bytes stay on the motion server
 * by digest), a classic entry carries its sampled trajectory, and the
 * project is autosaved whole.
 */
export const PLAN_HISTORY_LIMIT = 8;

export type { PlanHistoryEntry };

/** The entry the active plan is, or null when the active plan is not in
 *  the history (a program from before the history, or none). */
export function activePlanEntryId(project: OfflineProgrammingProject): string | null {
  const history = project.planHistory ?? [];
  const digest = project.denseTrajectory?.trajectoryDigest ?? null;
  if (digest) {
    return history.find((entry) => entry.denseTrajectory?.trajectoryDigest === digest)?.id ?? null;
  }
  if (project.plan.state !== "missing" && project.plan.id) {
    const planId = project.plan.id;
    const revision = project.plan.revision;
    return history.find((entry) => entry.plan?.id === planId && entry.plan?.revision === revision)?.id ?? null;
  }
  return null;
}

/**
 * Record the plan the project holds now as the newest entry. A plan
 * already recorded (same digest, or same plan id and revision) moves to
 * the top rather than doubling. Nothing to record is nothing done.
 */
export function recordPlan(project: OfflineProgrammingProject, timestamp?: string): OfflineProgrammingProject {
  const dense = project.denseTrajectory;
  const classic = project.plan.state !== "missing" && project.plan.state !== "rejected" && project.plan.trajectory.length > 0
    ? project.plan
    : null;
  if (!dense && !classic) return project;
  const next = cloneProject(project);
  const history = next.planHistory ?? [];
  const entry: PlanHistoryEntry = dense
    ? {
      id: `plan:${dense.trajectoryDigest}`,
      createdAt: timestamp ?? dense.createdAt,
      kind: "rails",
      planner: "weld planner",
      sourceProjectRevision: dense.sourceProjectRevision,
      sourceCell: cellSettingsKey(project.robot.cell),
      robotModel: project.robot.model,
      plan: null,
      denseTrajectory: cloneValue(dense),
      summary: { durationS: dense.totalDurationS, sampleCount: dense.totalSampleCount, segments: dense.segments.length },
    }
    : {
      id: `plan:${classic!.id}:${classic!.revision}`,
      createdAt: timestamp ?? classic!.createdAt ?? new Date().toISOString(),
      kind: "classic",
      planner: classic!.planner === "browser_preview" ? "browser preview" : "Tesseract",
      sourceProjectRevision: classic!.sourceProjectRevision ?? project.identity.revision,
      sourceCell: classic!.sourceCell ?? null,
      robotModel: project.robot.model,
      plan: cloneValue(classic!),
      denseTrajectory: null,
      summary: {
        durationS: classic!.trajectory.at(-1)?.timeS ?? 0,
        sampleCount: classic!.trajectory.length,
        segments: 0,
      },
    };
  next.planHistory = [entry, ...history.filter((other) => other.id !== entry.id)].slice(0, PLAN_HISTORY_LIMIT);
  return next;
}

/** Make an entry the active plan. Unknown id: the project unchanged. */
export function activatePlan(project: OfflineProgrammingProject, entryId: string): OfflineProgrammingProject {
  const entry = (project.planHistory ?? []).find((candidate) => candidate.id === entryId);
  if (!entry) return project;
  const next = cloneProject(project);
  if (entry.kind === "rails") {
    next.denseTrajectory = cloneValue(entry.denseTrajectory);
    // A rails program's classic slot is the missing plan; leave it.
  } else if (entry.plan) {
    next.plan = cloneValue(entry.plan);
    next.denseTrajectory = null;
  }
  return next;
}

/** Forget an entry. Deleting the active plan leaves the program with
 *  none: Simulate has nothing to replay and Deploy nothing to load until
 *  another is activated or planned. */
export function deletePlan(project: OfflineProgrammingProject, entryId: string): OfflineProgrammingProject {
  const history = project.planHistory ?? [];
  if (!history.some((entry) => entry.id === entryId)) return project;
  const next = cloneProject(project);
  const active = activePlanEntryId(project) === entryId;
  next.planHistory = history.filter((entry) => entry.id !== entryId).map((entry) => cloneValue(entry));
  if (active) {
    next.plan = missingPlan();
    next.denseTrajectory = null;
  }
  return next;
}
