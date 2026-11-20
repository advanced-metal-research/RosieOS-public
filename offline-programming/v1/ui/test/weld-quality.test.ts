import assert from "node:assert/strict";
import test from "node:test";
import {
  createBlankSpreadsheetProject,
  createSpreadsheetRow,
} from "../src/domain/spreadsheet.ts";
import { setRowArc } from "../src/domain/weld.ts";
import { heatInputKJPerMm, lintProgram } from "../src/domain/weld-quality.ts";
import { attachMoveMetadata } from "../src/domain/points-moves.ts";

test("heat input follows V*I over travel speed and fails closed on zeros", () => {
  // 20 V x 150 A at 5 mm/s = 3000 W / 5 mm/s = 600 J/mm = 0.6 kJ/mm.
  const value = heatInputKJPerMm(20, 150, 0.005);
  assert.ok(value !== null && Math.abs(value - 0.6) < 1e-12);
  assert.equal(heatInputKJPerMm(0, 150, 0.005), null);
  assert.equal(heatInputKJPerMm(20, 0, 0.005), null);
  assert.equal(heatInputKJPerMm(20, 150, 0), null);
});

test("program lint flags weld-schedule mistakes without blocking", () => {
  const project = createBlankSpreadsheetProject({ name: "Lint" });
  project.rows.push(createSpreadsheetRow(project, "motion"));
  const row = project.rows[0]!;
  setRowArc(row, true);
  row.weldProcess = "gmaw";
  row.voltageV = 0;
  row.amperageA = 0;
  row.wireFeedMPerMin = 0;
  row.blendRadiusM = 0.002;
  row.fineStop = true;
  row.weavePattern = "zigzag";
  row.weaveAmplitudeMm = 2;
  row.weaveFrequencyHz = 60;
  row.tcpVelocityMps = 0.005; // 5 mm/s at 60 Hz -> 0.083 mm wavelength

  const codes = lintProgram(project).map((finding) => finding.code);
  assert.ok(codes.includes("arc_without_power"));
  assert.ok(codes.includes("gmaw_without_wire"));
  assert.ok(codes.includes("blend_with_fine_stop"));
  assert.ok(codes.includes("weave_wavelength_too_short"));
  assert.ok(!codes.includes("no_arc_rows"));
});

test("program lint reports zero-speed errors and arc-free programs", () => {
  const project = createBlankSpreadsheetProject({ name: "Lint 2" });
  project.rows.push(createSpreadsheetRow(project, "motion"));
  const first = project.rows[0]!;
  first.tcpVelocityMps = 0;
  const loop = createSpreadsheetRow(project, "command");
  loop.loopTargetRowId = null;
  project.rows.push(loop);

  const findings = lintProgram(project);
  const byCode = new Map(findings.map((finding) => [finding.code, finding]));
  assert.equal(byCode.get("zero_speed")?.severity, "error");
  assert.equal(byCode.get("zero_speed")?.rowNumber, 1);
  assert.equal(byCode.get("loop_without_target")?.severity, "error");
  assert.equal(byCode.get("no_arc_rows")?.rowId, null);

  // Disabled rows are not linted.
  first.enabled = false;
  const after = lintProgram(project).map((finding) => finding.code);
  assert.ok(!after.includes("zero_speed"));
});

test("program lint reports one move-level finding instead of sample spam", () => {
  const project = createBlankSpreadsheetProject({ name: "Move lint" });
  project.rows = Array.from({ length: 8 }, (_, index) => {
    const row = createSpreadsheetRow(project, "motion", `lint-sample-${index}`);
    row.targetTcpPose = [index * 0.01, 0, 0, 0, 0, 0];
    row.tcpVelocityMps = 0;
    return row;
  });
  attachMoveMetadata(project.rows, [{ startIndex: 0, endIndex: 7, kind: "LINE" }]);
  const zeroSpeed = lintProgram(project).filter((finding) =>
    finding.code === "zero_speed"
  );
  assert.equal(zeroSpeed.length, 2); // CP1 plus the single M1 representative.
  assert.ok(zeroSpeed.every((finding) => finding.rowId !== "lint-sample-3"));
});

test("program lint reports the corrected row-budget error with recovery actions", () => {
  const project = createBlankSpreadsheetProject({ name: "Budget lint" });
  project.program.settings.gradient_move_v1 = {
    import_tolerance_m: 0.0001,
    max_row_count: 64,
    required_row_count: 70,
    imported_row_count: 63,
    over_budget_seam_ids: ["seam-3"],
  };
  const finding = lintProgram(project).find((candidate) =>
    candidate.code === "program_row_budget_exceeded"
  );
  assert.equal(finding?.severity, "error");
  assert.match(finding?.message ?? "", /over row budget/i);
  assert.match(finding?.message ?? "", /coarsen tolerance, disable seams, or split the program/i);
});
