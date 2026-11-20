import type {
  AuthoredProgramRow,
  OfflineProgrammingProject,
} from "./project.ts";
import { cadRowBudgetStatus } from "../adapters/cadquery.ts";
import { buildProgramView } from "./points-moves.ts";
import { rowWeaveSettings } from "./weld.ts";

/**
 * Arc energy per length, the number checked against a WPS. Uses the standard
 * HI = (V x I) / travel speed with no process efficiency factor applied —
 * callers must label it as arc energy, not net heat input.
 */
export function heatInputKJPerMm(
  voltageV: number,
  amperageA: number,
  travelSpeedMps: number,
): number | null {
  if (!isPositive(voltageV) || !isPositive(amperageA) || !isPositive(travelSpeedMps)) {
    return null;
  }
  const wattS = voltageV * amperageA;
  const speedMmPerS = travelSpeedMps * 1000;
  return wattS / speedMmPerS / 1000;
}

export interface ProgramLintFinding {
  severity: "error" | "warning";
  code: string;
  rowId: string | null;
  rowNumber: number | null;
  message: string;
}

/**
 * Pre-plan authoring checks. These are advisory: they never block Plan &
 * simulate, they exist so obvious weld-schedule mistakes surface before a
 * planning round-trip (or worse, a reviewed physical start).
 */
export function lintProgram(
  project: OfflineProgrammingProject,
): ProgramLintFinding[] {
  const findings: ProgramLintFinding[] = [];
  const view = buildProgramView(project);
  const add = (
    severity: ProgramLintFinding["severity"],
    code: string,
    row: AuthoredProgramRow | null,
    rowNumber: number | null,
    message: string,
  ): void => {
    findings.push({ severity, code, rowId: row?.id ?? null, rowNumber, message });
  };
  const budget = cadRowBudgetStatus(project);
  if (budget && budget.overBudgetSeamIds.length > 0) {
    add(
      "error",
      "program_row_budget_exceeded",
      null,
      null,
      `Program is over row budget (${budget.requiredRows} required, ${budget.maxRows} allowed) — coarsen tolerance, disable seams, or split the program. ${budget.overBudgetSeamIds.length} over-budget seam${budget.overBudgetSeamIds.length === 1 ? " was" : "s were"} imported disabled.`,
    );
  }

  const lintRows = new Map<string, { row: AuthoredProgramRow; index: number }>();
  for (const entry of view.entries) {
    if (entry.kind === "control") {
      lintRows.set(entry.control.row.id, {
        row: entry.control.row,
        index: entry.control.rowIndex,
      });
    } else if (entry.kind === "raw") {
      lintRows.set(entry.row.id, { row: entry.row, index: entry.rowIndex });
    }
  }
  for (const move of view.moves) {
    if (move.modifiedExternally) continue;
    const representative = move.rows.find((row) => row.weldEnabled) ?? move.end.row;
    lintRows.set(representative.id, {
      row: representative,
      index: project.rows.indexOf(representative),
    });
  }

  [...lintRows.values()]
    .sort((left, right) => left.index - right.index)
    .forEach(({ row, index }) => {
    const rowNumber = index + 1;
    const rowType = row.rowType ?? "motion";
    if (rowType === "command" && row.command === "LOOP" && !row.loopTargetRowId) {
      add("error", "loop_without_target", row, rowNumber, "Loop row has no target row.");
    }
    if (rowType !== "motion" || !row.enabled) return;

    if (!isPositive(row.tcpVelocityMps) && row.moveType !== "JOINT") {
      add("error", "zero_speed", row, rowNumber, "Enabled move has no TCP speed.");
    }
    if (row.blendRadiusM > 0 && row.fineStop) {
      add(
        "warning",
        "blend_with_fine_stop",
        row,
        rowNumber,
        "Blend radius is set but fine stop forces a full stop — the blend never happens.",
      );
    }

    if (!row.weldEnabled) return;
    if (row.moveType === "JOINT") {
      add(
        "warning",
        "arc_on_joint_move",
        row,
        rowNumber,
        "Arc is on during a JOINT move — the TCP path through the seam is not defined.",
      );
    }
    if (row.weldProcess === "none") {
      add("warning", "arc_without_process", row, rowNumber, "Arc is on but no weld process is selected.");
    }
    if (!isPositive(row.voltageV) || !isPositive(row.amperageA ?? 0)) {
      add(
        "warning",
        "arc_without_power",
        row,
        rowNumber,
        "Arc is on with zero voltage or current — no usable weld schedule.",
      );
    }
    if (row.weldProcess === "gmaw" && !isPositive(row.wireFeedMPerMin)) {
      add("warning", "gmaw_without_wire", row, rowNumber, "GMAW arc row has no wire feed speed.");
    }
    const weave = rowWeaveSettings(row);
    if (weave.pattern !== "none" && weave.amplitudeMm > 0 && weave.frequencyHz > 0) {
      const wavelengthMm = isPositive(row.tcpVelocityMps)
        ? row.tcpVelocityMps / weave.frequencyHz * 1000
        : null;
      if (wavelengthMm !== null && wavelengthMm < 2) {
        add(
          "warning",
          "weave_wavelength_too_short",
          row,
          rowNumber,
          `Weave wavelength is ${wavelengthMm.toFixed(2)} mm at this speed — too fast to execute; lower the frequency or raise the speed.`,
        );
      }
    }
    });

  const enabledMotion = project.rows.filter(
    (row) => row.enabled && (row.rowType ?? "motion") === "motion",
  );
  if ((!project.nodes.some(node => node.op === "move") || project.nodes.some(node => node.op === "weld" && node.enabled)) && enabledMotion.length > 0 && !enabledMotion.some((row) => row.weldEnabled)) {
    add(
      "warning",
      "no_arc_rows",
      null,
      null,
      "No enabled motion row turns the arc on — this program never welds.",
    );
  }
  return findings;
}

function isPositive(value: unknown): boolean {
  return typeof value === "number" && Number.isFinite(value) && value > 0;
}
