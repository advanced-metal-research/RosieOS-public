// Robot actions through the real `rosie` binary against a fake OLP. What
// must hold whatever an agent types: with robot actions off, or a real
// robot under `simulation`, or --sim on a real robot, nothing is POSTed;
// after a motion command the session keeps being heartbeaten by the
// keepalive process (the command itself has exited), and Stop ends it; a
// run is a preview that moves nothing until --confirm carries the very
// joints OLP listed.
import assert from "node:assert/strict";
import { execFile } from "node:child_process";
import { mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { after, before, test } from "node:test";

import { REPO_ROOT, simTarget, startFakeOlp, type FakeOlp } from "./fakes.js";

const MAIN = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "src", "main.js");
const REFERENCE = path.join(REPO_ROOT, "motion-server", "joint-trajectory", "v1", "testdata", "dense_joint_trajectory_reference.rdt");
const home = mkdtempSync(path.join(tmpdir(), "rosie-robot-home-"));
let olp: FakeOlp;

interface Envelope { ok: boolean; verdict: string; data: any; next: string[]; code: number }

function rosie(mode: string, ...args: string[]): Promise<Envelope> {
  return new Promise((resolve, reject) => {
    execFile(process.execPath, [MAIN, ...args, "--json", "--quiet"], {
      env: { ...process.env, ROSIE_HOME: home, ROSIE_STATE_DIR: path.join(home, "state"), ROSIE_OLP_URL: olp.url, ROSIE_ROBOT_ACTIONS: mode, ROSIE_HEARTBEAT_WINDOW_S: "5" },
      timeout: 60_000,
    }, (error, stdout, stderr) => {
      try {
        resolve({ ...(JSON.parse(stdout) as Envelope), code: error ? Number((error as { code?: number }).code ?? 1) : 0 });
      } catch {
        reject(new Error(`rosie ${args.join(" ")}: no envelope\n${stdout}\n${stderr}`));
      }
    });
  });
}

const posts = (route: string) => olp.requests.filter((r) => r.method === "POST" && r.path === route).length;
const sleep = (ms: number) => new Promise((resolve) => setTimeout(resolve, ms));

before(async () => {
  olp = await startFakeOlp({ available: true, armed: true, target: simTarget(true), session: { id: "sess-1", kind: "dense", state: "idle", lease: { held: true, expires_in_ms: 4000 } } });
});

after(async () => {
  await rosie("simulation", "robot", "keepalive", "--stop").catch(() => undefined);
  await olp.close();
});

test("refused before anything is sent: robot actions off, a real robot under 'simulation', --sim on a real robot", async () => {
  const off = await rosie("off", "robot", "arm");
  assert.equal(off.code, 1);
  assert.match(off.verdict, /robot actions are off \(ROSIE_ROBOT_ACTIONS=off\)/);
  olp.status.target = simTarget(false);
  const real = await rosie("simulation", "robot", "arm");
  assert.equal(real.code, 1);
  assert.match(real.verdict, /not a simulation/);
  const sim = await rosie("cell", "robot", "arm", "--sim");
  assert.equal(sim.code, 1);
  assert.match(sim.verdict, /--sim: the selected cell is .*REAL ROBOT/);
  assert.equal(posts("/dense-execution/arm"), 0, "no Arm reached OLP");
  olp.status.target = simTarget(true);
});

test("after Arm the keepalive heartbeats the session from its own process; Stop ends it", async () => {
  const armed = await rosie("simulation", "robot", "arm", "--sim");
  assert.equal(armed.ok, true, armed.verdict);
  assert.match(armed.verdict, /keepalive heartbeats the session/);
  const beatsAtExit = posts("/dense-execution/heartbeat");
  await sleep(2500);
  assert.ok(posts("/dense-execution/heartbeat") >= beatsAtExit + 2, "beats continue after the command exited");
  const shown = await rosie("simulation", "robot", "keepalive");
  assert.match(shown.verdict, /heartbeating session sess-1/);
  const stopped = await rosie("simulation", "robot", "stop");
  assert.equal(stopped.ok, true, stopped.verdict);
  assert.equal(posts("/dense-execution/stop"), 1);
  await sleep(1200);
  const afterStop = posts("/dense-execution/heartbeat");
  await sleep(2000);
  assert.equal(posts("/dense-execution/heartbeat"), afterStop, "no beat after Stop");
  assert.match((await rosie("simulation", "robot", "keepalive")).verdict, /no keepalive running/);
});

test("run is a preview that moves nothing, then --confirm sends OLP's own joints list unchanged", async () => {
  const joints = [{ axis: 0, id: "J1", from_deg: 10, to_deg: 0, delta_deg: -10, moves: true }, { axis: 1, id: "J2", from_deg: 0, to_deg: 0, delta_deg: 0, moves: false }];
  olp.routes["/dense-execution/run-program"] = (request) => {
    const body = request.body as Record<string, unknown>;
    if (!body.confirm_move) return [200, { state: "confirm", action: "Move to program start and run", needs_move: true, needs_arm: false, joints }];
    return [200, { state: "playing", session_id: "sess-1", steps: ["move", "play"] }];
  };
  const preview = await rosie("simulation", "run", "--sim", "--trajectory", REFERENCE, "--no-wait");
  assert.equal(preview.code, 1, "a preview is not a success: nothing ran");
  assert.match(preview.verdict, /needs the user's confirmation: Move to program start and run .* nothing moved/);
  const token = preview.data.token as string;
  assert.match(token, /^[0-9a-f]{6}$/);
  const wrong = await rosie("simulation", "run", "--sim", "--confirm", "000000", "--no-wait");
  assert.equal(wrong.code, 1);
  const confirmed = await rosie("simulation", "run", "--sim", "--confirm", token, "--no-wait");
  assert.equal(confirmed.ok, true, confirmed.verdict);
  const sent = olp.requests.filter((r) => r.path === "/dense-execution/run-program").map((r) => r.body as Record<string, unknown>);
  assert.equal(sent.length, 2, "the wrong token sent nothing");
  assert.equal(sent[0]!.confirm_move, false);
  assert.equal(sent[1]!.confirm_move, true);
  assert.deepEqual(sent[1]!.confirmed, joints, "the joints OLP listed, unchanged");
  assert.equal(sent[1]!.trajectory_digest, sent[0]!.trajectory_digest);
  assert.equal(sent[1]!.unattended, false, "runs from the CLI are attended");
  await rosie("simulation", "robot", "keepalive", "--stop");
});
