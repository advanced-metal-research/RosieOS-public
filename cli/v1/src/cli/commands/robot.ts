// rosie robot / rosie run: robot actions through OLP only (lib/robot.ts,
// lib/jog.ts — the MCP server's rules, one implementation). Ending motion
// (stop, disarm, pause, finish) is always available. Starting motion needs
// the target's robot_actions `simulation` (simulated cells only) or `cell`
// (real robots: the owner's setting). Home, fault reset and recover-to-zero
// are never offered. After a successful motion command a detached keepalive
// heartbeats the session for the target's window, then OLP's 5 s deadman
// stops the robot unless an operator's client keeps it alive.
import { randomBytes } from "node:crypto";
import { mkdir, readFile, rm, writeFile } from "node:fs/promises";
import path from "node:path";

import type { Context } from "../../lib/context.js";
import type { Engine } from "../../lib/engine.js";
import { CARTESIAN_AXES, fmt, jogHold, moveJoints, cartesianStep, toDeg, type CartesianAxis } from "../../lib/jog.js";
import type { DenseStatus } from "../../lib/olp.js";
import { summarize } from "../../lib/rdt.js";
import * as robot from "../../lib/robot.js";
import { describeStatus } from "../../lib/status.js";
import { errorMessage, record, table } from "../../lib/text.js";
import { ok, refused, UsageError, type Command, type Outcome } from "../command.js";
import type { Env } from "../env.js";
import { extendKeepalive, keepaliveState, serveKeepalive, stopKeepalive } from "../keepalive.js";
import { onProgram } from "./common.js";

/** A library refusal as an outcome (exit 1, OLP's words); anything else propagates. */
async function refusable(body: () => Promise<Outcome>): Promise<Outcome> {
  try {
    return await body();
  } catch (error) {
    if (error instanceof robot.RobotRefusal) return refused(error.message, { data: error.data, next: ["rosie status", "rosie cell diagnose"] });
    throw error;
  }
}

function modeLine(status: DenseStatus): string {
  const t = status.target;
  return t ? `${t.cell_id} · ${t.simulation ? "SIMULATION" : t.simulation === false ? "REAL ROBOT" : `backend ${t.backend ?? "?"}`}` : "no cell";
}

/** --sim: refuse unless the selected target is a simulation, whatever robot_actions allows. */
async function requireSimulation(ctx: Context, wanted: boolean): Promise<void> {
  if (!wanted) return;
  const status = await ctx.olp.status();
  if (status.target?.simulation !== true) throw new robot.RobotRefusal(`--sim: the selected cell is ${modeLine(status)}, not a simulation; nothing was sent`);
}

async function keepAfter(env: Env, ctx: Context): Promise<string> {
  const status = await ctx.olp.status().catch(() => null);
  const view = await extendKeepalive(env, ctx, status?.session?.id ?? null);
  return view.running ? `keepalive heartbeats the session until ${view.until} (rosie robot stop ends it)` : "keepalive not running";
}

/** The workspace program's current plan identity, read straight from the engine (inside the workspace runner). */
function programIdentity(engine: Engine): { identity: robot.PlanIdentity; dryRun: boolean } {
  const kept = engine.planRecord();
  if (!kept) throw new robot.RobotRefusal("the workspace program has no plan: rosie program plan");
  if (!kept.current) throw new robot.RobotRefusal("the workspace program was edited after it was planned: rosie program plan");
  const { dry_run: dryRun, ...fields } = engine.loadBody();
  return { identity: fields as robot.PlanIdentity, dryRun: dryRun === true };
}

/** The plan to load or run: --trajectory <.rdt> (its header's identity), else the workspace program's plan. */
async function resolveIdentity(env: Env, trajectory: unknown): Promise<{ identity: robot.PlanIdentity; dryRun?: boolean; source: string }> {
  if (trajectory) {
    const summary = summarize(await readFile(String(trajectory)));
    if (!summary.valid) throw new robot.RobotRefusal(`${String(trajectory)} is rejected by the ingest rules: ${summary.rejection!.reason}: ${summary.rejection!.detail}`);
    return { identity: summary.identity as unknown as robot.PlanIdentity, source: String(trajectory) };
  }
  let found: { identity: robot.PlanIdentity; dryRun: boolean } | null = null;
  const outcome = await onProgram(env, {}, async (p) => {
    found = programIdentity(p.engine);
    return ok("");
  });
  if (!found) throw new robot.RobotRefusal(outcome.verdict);
  const resolved = found as { identity: robot.PlanIdentity; dryRun: boolean };
  return { ...resolved, source: `workspace ${env.globals.workspace}` };
}

// ---------------------------------------------------------------- ending motion

const stop: Command<Env> = {
  path: ["robot", "stop"],
  summary: "STOP the selected cell through OLP (always available, any client, any gate); ends the keepalive",
  description: "Native Stop, outputs inhibited, session and grant released, any loop ended. A receipt is not proof of standstill: if it says NOT confirmed, the hardware E-stop is next — tell the operator.",
  examples: ["rosie robot stop"],
  async run(env) {
    const ctx = await env.ctx();
    await stopKeepalive(env.home, "STOP pressed");
    const stopped = await robot.stop(ctx);
    if (!stopped.ok || !stopped.stopped) {
      return refused(`STOP ${stopped.ok ? "sent, NOT confirmed" : `failed (HTTP ${stopped.reply.status})`} — use the hardware E-stop if the robot may still be moving, and tell the operator`, { data: robot.replyBody(stopped.reply) });
    }
    return ok(`STOP confirmed after ${String(stopped.attempts ?? "?")} attempt(s)${stopped.note ? ` · ${String(stopped.note)}` : ""}`, { body: stopped.status ? describeStatus(stopped.status) : "", data: robot.replyBody(stopped.reply) });
  },
};

const disarm: Command<Env> = {
  path: ["robot", "disarm"],
  summary: "Disarm (passes every gate, any client); ends the keepalive",
  async run(env) {
    const ctx = await env.ctx();
    await stopKeepalive(env.home, "disarmed");
    return refusable(async () => {
      const data = await robot.disarm(ctx);
      return data.accepted === false ? refused("Disarm was NOT accepted", { data }) : ok("disarmed", { data });
    });
  },
};

const pause: Command<Env> = {
  path: ["robot", "pause"],
  summary: "Pause the playing program (a hold on the path, not Stop)",
  options: [{ name: "session", type: "string", description: "Session id (default the current)" }],
  run: async (env, args) => refusable(async () => {
    const paused = await robot.pause(await env.ctx(), args.session as string | undefined);
    return ok(`paused session ${paused.sessionId}`, { body: describeStatus(paused.status) });
  }),
};

const finish: Command<Env> = {
  path: ["robot", "finish"],
  summary: "Finish cycle: a running loop ends at the program start after this cycle",
  run: async (env) => refusable(async () => {
    const data = await robot.finishLoop(await env.ctx());
    return ok("the loop will finish at the end of this cycle", { data });
  }),
};

// ---------------------------------------------------------------- starting motion

const select: Command<Env> = {
  path: ["robot", "select"],
  summary: "Connect OLP to a commissioned cell ('' disconnects); stops the previous cell's session",
  options: [{ name: "cell", type: "string", description: "Cell id from rosie cell list (OLP's list)", positional: 0, required: true }],
  examples: ["rosie robot select local-simulation"],
  run: async (env, args) => refusable(async () => {
    const selected = await robot.selectCell(await env.ctx(), String(args.cell));
    return ok(`selected ${String(args.cell) || "no cell"}: ${modeLine(selected.status)}`, { body: describeStatus(selected.status), data: selected });
  }),
};

const arm: Command<Env> = {
  path: ["robot", "arm"],
  summary: "Arm the selected cell (resets only drive-profile-resettable faults; never homes; moves nothing)",
  description: "Refusals (maintenance 503, in_use_by_other_controller 409, home_invalid, fault_persists, ...) come back in OLP's words: report them, never work around them. Home is an operator action at the mechanical home pose.",
  options: [{ name: "sim", type: "boolean", description: "Refuse unless the selected cell is a simulation" }],
  examples: ["rosie robot arm --sim"],
  run: async (env, args) => refusable(async () => {
    const ctx = await env.ctx();
    await requireSimulation(ctx, Boolean(args.sim));
    const data = await robot.arm(ctx);
    if (!data.accepted) return refused(`Arm NOT accepted${data.error ? `: ${String(data.error)}` : ""}`, { data });
    return ok(`armed · ${await keepAfter(env, ctx)}`, { body: data.status ? describeStatus(data.status as DenseStatus) : "", data });
  }),
};

const load: Command<Env> = {
  path: ["robot", "load"],
  summary: "Load the workspace program's plan (or --trajectory <.rdt>) onto the selected cell; moves nothing",
  options: [
    { name: "trajectory", type: "string", description: "An .rdt file whose identity to load (default: the workspace program's plan)" },
    { name: "dry_run", type: "boolean", description: "Motion-only check, no process outputs (default: the program's run mode)" },
    { name: "sim", type: "boolean", description: "Refuse unless the selected cell is a simulation" },
  ],
  run: async (env, args) => refusable(async () => {
    const plan = await resolveIdentity(env, args.trajectory);
    const ctx = await env.ctx();
    await requireSimulation(ctx, Boolean(args.sim));
    const data = await robot.load(ctx, { fromOpenProgram: false, ...plan.identity, dryRun: (args.dry_run as boolean | undefined) ?? plan.dryRun });
    return ok(`loaded ${plan.source} as session ${String(data.session_id)} · ${await keepAfter(env, ctx)}`, { data, next: ["rosie robot play --sim", "rosie run --sim"] });
  }),
};

const play: Command<Env> = {
  path: ["robot", "play"],
  summary: "Play the loaded session. THE ROBOT MOVES. Needs every joint at the program's first sample (else use rosie run)",
  options: [{ name: "session", type: "string", description: "Session id (default the current)" }, { name: "sim", type: "boolean", description: "Refuse unless the selected cell is a simulation" }],
  run: async (env, args) => refusable(async () => {
    const ctx = await env.ctx();
    await requireSimulation(ctx, Boolean(args.sim));
    const played = await robot.play(ctx, args.session as string | undefined);
    return ok(`playing session ${played.sessionId} · ${await keepAfter(env, ctx)}`, { body: describeStatus(played.status), next: ["rosie robot watch", "rosie robot stop"] });
  }),
};

interface PendingRun {
  token: string;
  created_at: number;
  target: string;
  identity: robot.PlanIdentity;
  fraction: number;
  repeat: "once" | "count" | "loop";
  count?: number;
  action: string;
  needs_arm: boolean;
  needs_move: boolean;
  loop: unknown;
  joints: robot.ConfirmedJoint[];
}

// Unverified: a confirmation older than this is refused here (OLP itself refuses one whose joints moved > 0.1°).
const CONFIRM_TTL_MS = 10 * 60 * 1000;

async function watchUntilDone(ctx: Context, env: Env, timeoutS: number): Promise<{ status: DenseStatus; finished: boolean }> {
  const deadline = Date.now() + timeoutS * 1000;
  let status = await ctx.olp.status();
  let lastLine = "";
  while (Date.now() < deadline) {
    const s = status.session;
    const loop = record(status.loop);
    const cursor = record(status.cursor);
    const line = `${s ? `${s.kind}/${s.state}` : "no session"}${cursor.total_sample_count ? ` · segment ${String(cursor.segment_index)}/${String(cursor.segment_count)} · ${Number(cursor.global_t_s).toFixed(1)}/${Number(cursor.total_duration_s).toFixed(1)} s` : ""}${loop.mode ? ` · loop ${String(loop.state)} ${String(loop.cycles)} cycles` : ""}`;
    if (line !== lastLine) env.progress(line);
    lastLine = line;
    const playing = s?.state === "playing" || s?.state === "staging" || s?.state === "moving" || loop.state === "running" || loop.state === "finishing";
    if (!playing && Date.now() > deadline - timeoutS * 1000 + 1500) return { status, finished: true };
    await new Promise((resolve) => setTimeout(resolve, 500));
    status = await ctx.olp.status();
  }
  return { status, finished: false };
}

const run: Command<Env> = {
  path: ["run"],
  summary: "Run the planned program (Go to start, Load, Arm, Play) — two steps: preview the moves, then --confirm <token>",
  description: `Like the operator dialog. Without --confirm, OLP answers what the press needs (e.g. "Move to program start and run") and each joint's from/to; NOTHING MOVES, and rosie prints a token. Show that to the user; only on their agreement run again with --confirm <token> (same plan, joints unchanged; OLP refuses confirmation_stale if a joint moved > 0.1°). --sim refuses unless the selected cell is a simulation. Waits for the program to finish (--no-wait to return at once); the keepalive heartbeats the session. Loops started here are attended (they end with the keepalive window).`,
  options: [
    { name: "confirm", type: "string", description: "The token from the preview: the user agreed" },
    { name: "sim", type: "boolean", description: "Refuse unless the selected cell is a simulation" },
    { name: "fraction", type: "number", description: "Speed of the move to the start, fraction of each joint's max (0.01-1)", default: 0.1 },
    { name: "repeat", type: "string", description: "once | count | loop (count/loop need a plan made with --repeat)", choices: ["once", "count", "loop"], default: "once" },
    { name: "count", type: "number", description: "Runs, with --repeat count" },
    { name: "trajectory", type: "string", description: "An .rdt file to run instead of the workspace plan" },
    { name: "wait", type: "boolean", description: "Watch until the program ends", default: true },
    { name: "timeout_s", type: "number", description: "Longest wait", default: 600 },
  ],
  examples: ["rosie run --sim", "rosie run --sim --confirm 7f3a9c", "rosie run --sim --repeat count --count 3"],
  run: async (env, args) => refusable(async () => {
    const pendingFile = path.join(env.home, "cli", "run-confirm.json");
    const resolved = await env.resolved();
    let pending: PendingRun | null = null;
    if (args.confirm) {
      pending = JSON.parse(await readFile(pendingFile, "utf8").catch(() => "null")) as PendingRun | null;
      if (!pending || pending.token !== args.confirm) return refused(`no pending run with token ${String(args.confirm)}: preview again with rosie run`);
      if (Date.now() - pending.created_at > CONFIRM_TTL_MS) return refused("that confirmation is older than 10 minutes: preview again with rosie run");
      if (pending.target !== resolved.name) return refused(`that confirmation was for target ${pending.target}, not ${resolved.name}`);
    }
    const plan = pending ? { identity: pending.identity, source: "the confirmed plan" } : await resolveIdentity(env, args.trajectory);
    const ctx = await env.ctx();
    await requireSimulation(ctx, Boolean(args.sim));
    const repeat = (pending?.repeat ?? args.repeat) as "once" | "count" | "loop";
    const { data } = await robot.runProgram(ctx, {
      fromOpenProgram: false, ...plan.identity, fraction: pending?.fraction ?? Number(args.fraction),
      confirmArm: Boolean(pending?.needs_arm), confirmMove: Boolean(pending?.needs_move), confirmed: pending?.joints,
      repeat, count: (pending?.count ?? args.count) as number | undefined, confirmLoop: Boolean(pending && repeat !== "once"),
    });
    if (data.state === "confirm") {
      const joints = (Array.isArray(data.joints) ? data.joints : []) as robot.ConfirmedJoint[];
      const token = randomBytes(3).toString("hex");
      const next: PendingRun = {
        token, created_at: Date.now(), target: resolved.name, identity: plan.identity, fraction: Number(args.fraction), repeat, count: args.count as number | undefined,
        action: String(data.action), needs_arm: data.needs_arm === true, needs_move: data.needs_move === true, loop: data.loop ?? null, joints,
      };
      await mkdir(path.dirname(pendingFile), { recursive: true });
      await writeFile(pendingFile, JSON.stringify(next, null, 2));
      const status = await ctx.olp.status();
      return refused(`needs the user's confirmation: ${String(data.action)} on ${modeLine(status)} — nothing moved`, {
        body: [
          table(["Joint", "From", "To", "Δ", ""], joints.map((j) => [j.id, fmt(j.from_deg), fmt(j.to_deg), fmt(j.delta_deg), j.moves ? "moves" : "stays"])),
          data.loop ? `loop: ${JSON.stringify(data.loop)}` : "",
          `Show the user this action and every joint's move. Only if they agree: rosie run --confirm ${token}${args.sim ? " --sim" : ""}`,
        ].filter(Boolean).join("\n"),
        data: { ...data, token },
        next: [`rosie run --confirm ${token}${args.sim ? " --sim" : ""}   (only after the user agrees)`],
      });
    }
    await rm(pendingFile, { force: true });
    const keep = await keepAfter(env, ctx);
    if (!args.wait) return ok(`run program: ${String(data.state)}${data.session_id ? ` (session ${String(data.session_id)})` : ""} · ${keep}`, { data, next: ["rosie robot watch", "rosie robot stop"] });
    const watched = await watchUntilDone(ctx, env, Number(args.timeout_s));
    const s = watched.status.session;
    return (watched.finished && !s?.last_error ? ok : refused)(`${watched.finished ? "finished" : "still running after the timeout"}: ${s ? `${s.kind}/${s.state}` : "no session"}${s?.stop_reason ? ` · stop reason ${s.stop_reason}` : ""}${s?.last_error ? ` · last error ${s.last_error}` : ""}`, {
      body: describeStatus(watched.status), data: { run: data, status: watched.status }, next: watched.finished ? ["rosie status"] : ["rosie robot watch", "rosie robot stop"],
    });
  }),
};

const watch: Command<Env> = {
  path: ["robot", "watch"],
  summary: "Follow the session until it stops playing (progress on stderr), then print the status",
  options: [{ name: "timeout_s", type: "number", description: "Longest wait", default: 600 }],
  async run(env, args) {
    const ctx = await env.ctx();
    const watched = await watchUntilDone(ctx, env, Number(args.timeout_s));
    return (watched.finished ? ok : refused)(`${watched.finished ? "not playing" : "still running after the timeout"}: ${modeLine(watched.status)}`, { body: describeStatus(watched.status) });
  },
};

// ---------------------------------------------------------------- jogs

const jogMove: Command<Env> = {
  path: ["robot", "move"],
  summary: "Move joints BY (relative) or TO (absolute) angles in one synchronised move. THE ROBOT MOVES. Bounded per call",
  description: "Joints as J1=10,J3=-5 (degrees). Bounds before anything is sent: the target's jog limit per joint per call (default 30°), measured from the live position for absolute targets, and the described joint range; fraction ≤ 0.5 (default 0.1). Needs an armed session.",
  options: [
    { name: "joints", type: "strings", description: "J1=10,J3=-5 (degrees)", positional: 0, required: true },
    { name: "to", type: "boolean", description: "Absolute targets instead of relative travel" },
    { name: "fraction", type: "number", description: "Of each joint's described max velocity", default: 0.1 },
    { name: "sim", type: "boolean", description: "Refuse unless the selected cell is a simulation" },
  ],
  examples: ["rosie robot move J1=10 --sim", "rosie robot move J2=-20,J3=35 --to --sim"],
  run: async (env, args) => refusable(async () => {
    const joints = (args.joints as string[]).map((item) => {
      const [joint, deg] = item.split("=");
      if (!joint || deg === undefined || !Number.isFinite(Number(deg))) throw new UsageError(`"${item}" is not J<n>=<degrees>`);
      return { joint, deg: Number(deg) };
    });
    const ctx = await env.ctx();
    await requireSimulation(ctx, Boolean(args.sim));
    const moved = await moveJoints(ctx, { mode: args.to ? "absolute" : "relative", joints, fraction: Number(args.fraction) });
    if (moved.nothingToMove) return ok("nothing to move: every listed joint is already at its target");
    return ok(`moved ${moved.planned.length} joint(s) · ${await keepAfter(env, ctx)}`, {
      body: moved.planned.map((p) => `${p.id}: ${p.from === null ? "?" : fmt(p.from)} → ${p.toDeg === null ? `Δ ${fmt(p.deltaDeg)}` : fmt(p.toDeg)}${moved.now[p.index] != null ? ` (now ${fmt(toDeg(moved.now[p.index]!))})` : ""}`).join("\n"),
      data: moved,
    });
  }),
};

const jogStep: Command<Env> = {
  path: ["robot", "step"],
  summary: "One straight TCP step (x y z mm, rx ry rz °) from the held pose. THE ROBOT MOVES. Bounded per call (100 mm / 15°)",
  options: [
    { name: "axis", type: "string", description: "x, y, z, rx, ry, rz", positional: 0, required: true, choices: CARTESIAN_AXES },
    { name: "amount", type: "number", description: "Signed: mm or degrees", positional: 1, required: true },
    { name: "frame", type: "string", description: "base or tool", choices: ["base", "tool"], default: "base" },
    { name: "fraction", type: "number", description: "Speed fraction", default: 0.1 },
    { name: "sim", type: "boolean", description: "Refuse unless the selected cell is a simulation" },
  ],
  examples: ["rosie robot step z -20 --sim", "rosie robot step rx 5 --frame tool --sim"],
  run: async (env, args) => refusable(async () => {
    const ctx = await env.ctx();
    await requireSimulation(ctx, Boolean(args.sim));
    const amount = Number(args.amount);
    const stepped = await cartesianStep(ctx, { frame: args.frame as "base" | "tool", axis: args.axis as CartesianAxis, amount, fraction: Number(args.fraction) });
    return ok(`stepped ${String(args.axis)} ${amount > 0 ? "+" : ""}${amount} ${stepped.unit} (${String(args.frame)} frame) · ${await keepAfter(env, ctx)}`, { data: stepped });
  }),
};

const jogHoldCommand: Command<Env> = {
  path: ["robot", "hold"],
  summary: "A timed hold-to-run jog (joint J3+ or Cartesian z-), ≤ 3 s. THE ROBOT MOVES. End always sent",
  options: [
    { name: "what", type: "string", description: "J1..Jn or x/y/z/rx/ry/rz", positional: 0, required: true },
    { name: "direction", type: "string", description: "+ or -", positional: 1, required: true, choices: ["+", "-"] },
    { name: "seconds", type: "number", description: "Duration (≤ 3 s by default)", default: 1 },
    { name: "fraction", type: "number", description: "Speed fraction (≤ 0.25)", default: 0.1 },
    { name: "frame", type: "string", description: "base or tool (Cartesian)", choices: ["base", "tool"], default: "base" },
    { name: "sim", type: "boolean", description: "Refuse unless the selected cell is a simulation" },
  ],
  examples: ["rosie robot hold J3 + --seconds 1 --sim", "rosie robot hold z - --seconds 0.5 --sim"],
  run: async (env, args) => refusable(async () => {
    const what = String(args.what);
    const cartesian = (CARTESIAN_AXES as readonly string[]).includes(what.toLowerCase());
    const ctx = await env.ctx();
    await requireSimulation(ctx, Boolean(args.sim));
    const outcome = await jogHold(ctx, {
      kind: cartesian ? "cartesian" : "joint", joint: cartesian ? undefined : what, axis: cartesian ? (what.toLowerCase() as CartesianAxis) : undefined,
      frame: args.frame as "base" | "tool", direction: args.direction === "+" ? "positive" : "negative", durationS: Number(args.seconds), fraction: Number(args.fraction), speedMmS: 100, speedDegS: 15,
    });
    const end = outcome.lastJogEnd;
    const body = [
      `intents ${outcome.updates}, held ${(outcome.heldMs / 1000).toFixed(2)} s · end (${outcome.routes.end}, ${outcome.endCause}): ${outcome.endText}`,
      outcome.stopText ? `STOP: ${outcome.stopText}` : "",
      end ? `OLP's jog end: ${String(end.code ?? "?")}: ${String(end.text ?? "")}${end.repull ? ` (pull again: ${String(end.repull)})` : ""}` : "",
      outcome.hint,
    ].filter(Boolean).join("\n");
    if (!outcome.ok) return refused(`hold ${outcome.label}: ${outcome.summary}`, { body, data: outcome });
    return ok(`hold ${outcome.label}: ${outcome.summary} · ${await keepAfter(env, ctx)}`, { body, data: outcome });
  }),
};

const keepalive: Command<Env> = {
  path: ["robot", "keepalive"],
  summary: "Show (or --stop) the heartbeat keepalive that keeps a session alive between commands",
  options: [{ name: "stop", type: "boolean", description: "End it now (OLP's deadman then stops the robot in 5 s unless another client beats)" }, { name: "serve", type: "boolean", description: "(internal) run the keepalive process" }],
  async run(env, args) {
    if (args.serve) {
      await serveKeepalive(env);
      return ok("keepalive ended");
    }
    if (args.stop) return ok((await stopKeepalive(env.home, "stopped by rosie robot keepalive --stop")) ? "keepalive stopped" : "no keepalive was running");
    const view = await keepaliveState(env.home);
    return ok(view.running ? `heartbeating session ${view.session ?? "?"} on ${view.target} until ${view.until} (${view.beats ?? 0} beats${view.last_error ? `, last error ${view.last_error}` : ""})` : "no keepalive running", { data: view });
  },
};

export const commands: Command<Env>[] = [stop, disarm, pause, finish, select, arm, load, play, run, watch, jogMove, jogStep, jogHoldCommand, keepalive];
