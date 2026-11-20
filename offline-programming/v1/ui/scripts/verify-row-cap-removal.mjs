// P2.11 live proof: the exact Dog Box at 0.1 mm weld tolerance imports FULLY
// ENABLED (~730 rows) and plans on the canonical local simulator.
//
//   node --experimental-strip-types scripts/verify-row-cap-removal.mjs <plan-port> [red-port]
//
// Topology is extracted read-only from the long-running 127.0.0.1:8794 server.
// The plan request goes to <plan-port> (a scratch server built from this
// branch); if [red-port] is given, the same request is first shown REJECTED
// there (the pre-change binary still enforcing 1-64 rows).
import { HttpCadClient } from "../src/cad/client.ts";
import { DOG_BOX_FIXTURE } from "../src/cad/contracts.ts";
import {
  ROBOT_V4_PROGRAM_ROW_BUDGET,
  adaptCadQueryTopology,
  cadRowBudgetStatus,
} from "../src/adapters/cadquery.ts";
import { toRobotV4Program } from "../src/adapters/robot-v4.ts";
import { createProject } from "../src/domain/project.ts";
import { DEFAULT_MOVE_TOLERANCE_M } from "../src/domain/points-moves.ts";
import { lintProgram } from "../src/domain/weld-quality.ts";
import { buildLocalPlanRequest, localPlanRequestWire } from "../src/simulation/local-plan.ts";

const planPort = Number(process.argv[2] ?? 8795);
const redPort = process.argv[3] ? Number(process.argv[3]) : null;

const endpoint = "http://127.0.0.1:8794/api/offline-programming/v1/cadquery/topology";
const topology = await new HttpCadClient({ endpoint })
  .extractTopology({ source: DOG_BOX_FIXTURE, sampleCount: 64 });

const project = adaptCadQueryTopology(
  createProject({ name: "Dog Box P2.11 live proof" }),
  topology,
);
project.target = { mode: "local_simulator", profileId: "local-simulator", manifestRef: null };

const budget = cadRowBudgetStatus(project);
const overBudgetLint = lintProgram(project)
  .filter((finding) => finding.code === "program_row_budget_exceeded");
const wire = toRobotV4Program(project);
const importProof = {
  toleranceM: budget?.toleranceM,
  defaultToleranceM: DEFAULT_MOVE_TOLERANCE_M,
  rowBudget: ROBOT_V4_PROGRAM_ROW_BUDGET,
  requiredRows: budget?.requiredRows,
  importedRows: budget?.importedRows,
  wireRows: wire.rows.length,
  seams: project.seams.length,
  seamsEnabled: project.seams.filter((seam) => seam.enabled).length,
  overBudgetSeamIds: budget?.overBudgetSeamIds,
  overBudgetLintFindings: overBudgetLint.length,
  fullyEnabled: project.seams.every((seam) => seam.enabled) &&
    (budget?.overBudgetSeamIds.length ?? -1) === 0 &&
    budget?.requiredRows === budget?.importedRows,
  programBytes: JSON.stringify(wire).length,
};

async function submitPlan(port, label) {
  const request = buildLocalPlanRequest(project, {
    requestId: `p211-live-proof-${label}-${Date.now()}`,
    submittedAt: new Date().toISOString(),
  });
  const startedAt = performance.now();
  const response = await fetch(
    `http://127.0.0.1:${port}/api/offline-programming/v1/local-simulator/plans`,
    {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(localPlanRequestWire(request)),
    },
  );
  const wallTimeS = (performance.now() - startedAt) / 1000;
  const body = await response.json();
  const result = body.result ?? body;
  return {
    port,
    status: response.status,
    wallTimeS: Number(wallTimeS.toFixed(3)),
    error: body.error ?? null,
    planState: result?.evidence?.state ?? null,
    trajectorySamples: Array.isArray(result?.trajectory) ? result.trajectory.length : null,
    timing: result?.evidence?.timing ?? null,
    reportReasons: result?.report_reasons ?? null,
  };
}

const report = { importProof };
if (redPort) report.redPlan = await submitPlan(redPort, "red");
report.greenPlan = await submitPlan(planPort, "green");
console.log(JSON.stringify(report, null, 2));
