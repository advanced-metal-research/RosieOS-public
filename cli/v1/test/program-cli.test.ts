// The program workflow through the real `rosie` binary, one process per
// command as an agent runs it, against a fake OLP at the HTTP boundary. What
// it proves is the workspace: each command is a fresh process, so the
// program, its cell revision, the searched seams and the undo history must
// come back from disk exactly.
import assert from "node:assert/strict";
import { execFile } from "node:child_process";
import { mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { after, before, test } from "node:test";

import { startFakeProgramOlp, urdfTcpMmOf, type FakeProgramOlp } from "./fakes.js";

const MAIN = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "src", "main.js");
const home = mkdtempSync(path.join(tmpdir(), "rosie-cli-home-"));
let olp: FakeProgramOlp;

interface Envelope { ok: boolean; verdict: string; exit: number; data: any; artifacts: string[]; next: string[] }

function rosie(...args: string[]): Promise<Envelope & { code: number }> {
  return new Promise((resolve, reject) => {
    execFile(process.execPath, [MAIN, ...args, "--json", "--quiet"], {
      env: { ...process.env, ROSIE_HOME: home, ROSIE_STATE_DIR: path.join(home, "state"), ROSIE_OLP_URL: olp.url, ROSIE_PLANNER_URL: "http://127.0.0.1:9", ROSIE_ROBOT_ACTIONS: "" },
      timeout: 60_000,
    }, (error, stdout, stderr) => {
      try {
        const envelope = JSON.parse(stdout) as Envelope;
        resolve({ ...envelope, code: error ? Number((error as { code?: number }).code ?? 1) : 0 });
      } catch {
        reject(new Error(`rosie ${args.join(" ")} printed no envelope (exit ${String((error as { code?: number } | null)?.code)}):\n${stdout}\n${stderr}`));
      }
    });
  });
}

before(async () => {
  olp = await startFakeProgramOlp(home);
});

after(() => olp.close());

test("a program authored command by command: waypoint TCP in mm, edits, refusals, undo/redo, STEP, seams, weld, plan, save, conflict", async () => {
  const created = await rosie("program", "new", "teach");
  assert.equal(created.ok, true, created.verdict);
  assert.equal(created.data.program.model, olp.model);

  // A waypoint from joint angles: OLP's FK, checked against an independent URDF chain in absolute millimetres.
  const deg = [30, -20, 40, 10, 30, -15];
  const added = await rosie("program", "add-waypoint", "--joints", deg.join(","));
  assert.equal(added.ok, true, added.verdict);
  const row = added.data.program.rows[0];
  const expected = urdfTcpMmOf(olp.urdf, Object.fromEntries(deg.map((v, i) => [`J${i + 1}`, (v * Math.PI) / 180])));
  for (let i = 0; i < 3; i++) assert.ok(Math.abs(row.cells[`target.xyz_m.${i}`] - expected[i]!) < 1e-6, `TCP ${"xyz"[i]} ${row.cells[`target.xyz_m.${i}`]} vs ${expected[i]} mm`);

  // Edits by row number, in display units; OLP refuses out of range, nothing applied.
  const edited = await rosie("program", "edit", "1", "speed_mm_s=40", "label=slow approach");
  assert.equal(edited.ok, true, edited.verdict);
  const bad = await rosie("program", "edit", "1", "speed_scale=500");
  assert.equal(bad.ok, false);
  assert.equal(bad.code, 1);
  assert.match(bad.verdict, /speed_scale refused/);

  // Undo and redo across processes, through OLP's restoreAsRevision.
  const shown = await rosie("program", "show");
  assert.equal(shown.data.rows[0].label, "slow approach");
  const revision = shown.data.revision;
  const undone = await rosie("program", "undo");
  assert.equal(undone.ok, true, undone.verdict);
  const afterUndo = await rosie("program", "show");
  assert.notEqual(afterUndo.data.rows[0].label, "slow approach");
  assert.ok(afterUndo.data.revision > revision, "an undo is a new revision (restoreAsRevision), never a reload of the old one");
  await rosie("program", "redo");
  assert.equal((await rosie("program", "show")).data.rows[0].label, "slow approach");

  // STEP part, seam detection, search kept across processes, weld accepted.
  const imported = await rosie("part", "import", olp.stepFile);
  assert.equal(imported.ok, true, imported.verdict);
  assert.match(imported.verdict, /1 seams/);
  const searched = await rosie("seam", "search", "--all");
  assert.equal(searched.ok, true, searched.verdict);
  assert.equal(searched.data[0].clear_percent, 95, "the reverse direction's 95 % coverage is kept");
  const listed = await rosie("seam", "list");
  assert.ok(listed.data[0].searched, "the search survived into the next process");
  const welded = await rosie("seam", "weld", "seam_0001");
  assert.equal(welded.ok, true, welded.verdict);
  const rows = (await rosie("program", "show")).data.rows.map((r: { id: string }) => r.id);
  assert.ok(rows.includes("rails-seam_0001"), rows.join(", "));

  // Plan: the trajectory is kept, and the plan is current until the next edit.
  const planned = await rosie("program", "plan");
  assert.equal(planned.ok, true, planned.verdict);
  assert.ok(planned.artifacts.some((file) => file.endsWith(".rdt")));
  assert.equal((await rosie("program", "show")).data.plan.current, true);

  // Save to the cell twice (the revision is carried between processes), then a conflict is refused, never overwritten.
  const first = await rosie("program", "save");
  assert.equal(first.ok, true, first.verdict);
  assert.match(first.verdict, /revision 1/);
  await rosie("program", "edit", "1", "label=again");
  const second = await rosie("program", "save");
  assert.match(second.verdict, /revision 2/);
  olp.storeRevision = 7;
  await rosie("program", "edit", "1", "label=conflicting");
  const conflict = await rosie("program", "save");
  assert.equal(conflict.ok, false);
  assert.match(conflict.verdict, /409/);

  // The unsaved program is not replaced by accident.
  const replaced = await rosie("program", "new", "other");
  assert.equal(replaced.ok, false);
  assert.match(replaced.verdict, /unsaved/);
  assert.equal((await rosie("program", "new", "other", "--force")).ok, true);
});

test("usage errors exit 2 and name the command's help; an unreachable OLP exits 3", async () => {
  const usage = await rosie("program", "add-waypoint");
  assert.equal(usage.code, 2);
  assert.ok(usage.next.some((n) => n.includes("--help")));
  const unreachable = await new Promise<{ code: number; stdout: string }>((resolve) => {
    execFile(process.execPath, [MAIN, "status", "--json"], { env: { ...process.env, ROSIE_HOME: home, ROSIE_OLP_URL: "http://127.0.0.1:9" } }, (error, stdout) => resolve({ code: Number((error as { code?: number } | null)?.code ?? 0), stdout }));
  });
  assert.equal(unreachable.code, 3);
  assert.equal(JSON.parse(unreachable.stdout).ok, false);
});
