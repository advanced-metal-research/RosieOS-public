// Robot actions through OLP only, shared by the CLI and the MCP server. OLP
// holds rt-control's single grant for every client, runs the 5 s heartbeat
// deadman and the maintenance/control gates; a second grant from here would
// sit where OLP's STOP cannot reach. So every action is an OLP request.
//
// Ending motion (Stop, Disarm, Pause, Finish cycle) passes every gate from
// any client and is never refused here. Starting motion (select, arm, load,
// play, run program, jogs) is admitted only through `admit`: a fresh status
// read, then the mode's refusal (robot actions `simulation` act only on a
// target rt-core itself describes as backend `simulation`; `cell` is the
// owner's choice), and the action carries that read's selection generation
// so OLP refuses it if another client switched cells in between.
//
// Never offered, by design: Home (HM35 is run only by an operator at the
// joint's mechanical home pose, never as recovery), fault reset (Arm already
// resets what the drive profile classes resettable; anything else is the
// owner's call), recover-to-zero, open-ended jogging.
import type { Context } from "./context.js";
import { refusalText, type HttpReply } from "./http.js";
import { DENSE, type DenseStatus, type TargetIdentity } from "./olp.js";
import { record } from "./text.js";

export class RobotRefusal extends Error {
  constructor(message: string, readonly data?: unknown) {
    super(message);
  }
}

export function replyBody(reply: HttpReply): Record<string, unknown> {
  return reply.body && typeof reply.body === "object" ? (reply.body as Record<string, unknown>) : { body: reply.body };
}

export function refusalOf(action: string, reply: HttpReply): RobotRefusal {
  return new RobotRefusal(`OLP refused ${action} (HTTP ${reply.status}): ${refusalText(reply)}`, { status: reply.status, ...replyBody(reply) });
}

/** Fresh status, then the mode's refusal. Every action that starts motion passes through here; none reads the target elsewhere. */
export async function admit(ctx: Context, action: string): Promise<{ target: TargetIdentity; status: DenseStatus }> {
  const { olp, config } = ctx;
  if (config.robotActions === "off") {
    throw new RobotRefusal(`${action} refused: robot actions are off (${config.variablePrefix}ROBOT_ACTIONS=off). Only Stop, Disarm, Pause and Finish cycle are available.`);
  }
  const status = await olp.status();
  const target = status.target;
  if (!status.available || !target) throw new RobotRefusal(`${action} refused: OLP has no cell selected. List cells and select one first.`);
  if (config.robotActions !== "cell" && target.simulation !== true) {
    throw new RobotRefusal(
      `${action} refused: the selected cell ${target.cell_id} is backend '${target.backend ?? "unknown"}', not a simulation, and ${config.variablePrefix}ROBOT_ACTIONS=${config.robotActions} allows robot actions on simulated cells only. A real cell needs ${config.variablePrefix}ROBOT_ACTIONS=cell, set by the owner.`,
      { target },
    );
  }
  return { target, status };
}

/** Admit, then one request with heartbeats around it (keeper.around: beats before the send, the window restarted on success). */
export async function act(ctx: Context, action: string, path: string, payload: unknown, timeoutMs?: number): Promise<{ reply: HttpReply; target: TargetIdentity }> {
  const { target } = await admit(ctx, action);
  const reply = await ctx.keeper.around(() => ctx.olp.post(path, payload, { target, timeoutMs }), (answer) => answer.ok);
  return { reply, target };
}

export const IDENTITY_FIELDS = ["trajectory_digest", "plan_id", "program_id", "program_digest", "manifest_revision", "plan_revision"] as const;
export type PlanIdentity = Partial<Record<(typeof IDENTITY_FIELDS)[number], string | number>>;

/**
 * The six-field plan identity: given, or the open program's current plan
 * (olp-core loadBody), with the program's own run mode (production plays
 * its process outputs; anything else is a dry run).
 */
export async function planIdentity(ctx: Context, input: { fromOpenProgram: boolean } & PlanIdentity): Promise<{ identity: Record<string, unknown>; dryRun?: boolean }> {
  if (input.fromOpenProgram) {
    const identity = await ctx.program.run(async (engine) => {
      ctx.program.requireOpen(engine);
      const kept = engine.planRecord();
      if (!kept) throw new RobotRefusal("The open program has no plan: plan it first");
      if (!kept.current) throw new RobotRefusal("The open program was edited after it was planned: plan it again");
      return engine.loadBody();
    });
    const { dry_run: dryRun, ...fields } = identity;
    return { identity: fields, dryRun: dryRun === true };
  }
  const missing = IDENTITY_FIELDS.filter((field) => input[field] === undefined);
  if (missing.length) throw new RobotRefusal(`give all six plan identity fields (missing: ${missing.join(", ")}) or the open program's plan`);
  return { identity: Object.fromEntries(IDENTITY_FIELDS.map((field) => [field, input[field]])) };
}

export async function currentSession(ctx: Context, given?: string): Promise<string> {
  if (given) return given;
  const status = await ctx.olp.status();
  if (!status.session?.id) throw new RobotRefusal("OLP has no execution session; Load a plan or Arm first");
  return status.session.id;
}

// ---------------------------------------------------------------- ending motion: every mode, every gate

/** STOP: native Stop, outputs inhibited, session and grant released, any loop ended. A receipt is not proof of standstill: read `stopped`. */
export async function stop(ctx: Context): Promise<{ ok: boolean; stopped: boolean; attempts: unknown; note: unknown; status: DenseStatus | undefined; reply: HttpReply }> {
  ctx.keeper.stop("STOP pressed");
  const reply = await ctx.olp.post(`${DENSE}/stop`, {});
  const data = replyBody(reply);
  return { ok: reply.ok, stopped: data.stopped === true, attempts: data.attempts, note: data.note, status: data.status as DenseStatus | undefined, reply };
}

export async function disarm(ctx: Context): Promise<Record<string, unknown>> {
  ctx.keeper.stop("disarmed");
  const reply = await ctx.olp.post(`${DENSE}/arm`, { armed: false });
  if (!reply.ok) throw refusalOf("disarm", reply);
  return replyBody(reply);
}

export async function pause(ctx: Context, sessionId?: string): Promise<{ sessionId: string; status: DenseStatus }> {
  const id = await currentSession(ctx, sessionId);
  const reply = await ctx.olp.post(`${DENSE}/pause`, { session_id: id });
  if (!reply.ok) throw refusalOf("pause", reply);
  return { sessionId: id, status: reply.body as DenseStatus };
}

export async function finishLoop(ctx: Context): Promise<Record<string, unknown>> {
  const reply = await ctx.olp.post(`${DENSE}/run-program/finish`, {});
  if (!reply.ok) throw refusalOf("finish cycle", reply);
  return replyBody(reply);
}

// ---------------------------------------------------------------- starting motion: admitted

/** Select (connect) a commissioned cell; '' disconnects. Stops whatever the previous cell's session was doing. Moves nothing itself. */
export async function selectCell(ctx: Context, cell: string): Promise<{ capabilities: unknown; status: DenseStatus }> {
  if (ctx.config.robotActions === "off") throw new RobotRefusal(`select cell refused: robot actions are off (${ctx.config.variablePrefix}ROBOT_ACTIONS=off)`);
  const reply = await ctx.olp.post(`${DENSE}/target`, { cell });
  if (!reply.ok) throw refusalOf(`select cell ${cell}`, reply);
  return { capabilities: reply.body, status: await ctx.olp.status() };
}

/** Arm: settles an unconfirmed Stop, resets only faults the drive profile classes resettable; never homes; moves nothing. */
export async function arm(ctx: Context): Promise<Record<string, unknown>> {
  const { reply } = await act(ctx, "Arm", `${DENSE}/arm`, { armed: true });
  if (!reply.ok) throw refusalOf("arm", reply);
  return replyBody(reply);
}

/** Load a planned trajectory (OLP fetches its bytes by digest and checks model, description, calibration, Home and the start). Moves nothing. */
export async function load(ctx: Context, input: { fromOpenProgram: boolean; dryRun?: boolean } & PlanIdentity): Promise<Record<string, unknown>> {
  const plan = await planIdentity(ctx, input);
  const { reply } = await act(ctx, "Load", `${DENSE}/load`, { ...plan.identity, dry_run: input.dryRun ?? plan.dryRun ?? false }, 120_000);
  if (!reply.ok) throw refusalOf("load", reply);
  return replyBody(reply);
}

/** Play the loaded session. THE ROBOT MOVES. Native Play requires every joint at the program's first sample. */
export async function play(ctx: Context, sessionId?: string): Promise<{ sessionId: string; status: DenseStatus }> {
  const id = await currentSession(ctx, sessionId);
  const { reply } = await act(ctx, "Play", `${DENSE}/play`, { session_id: id });
  if (!reply.ok) throw refusalOf("play", reply);
  return { sessionId: id, status: reply.body as DenseStatus };
}

export interface ConfirmedJoint { axis: number; id: string; from_deg: number; to_deg: number; delta_deg: number; moves: boolean }

export interface RunInput extends PlanIdentity {
  fromOpenProgram: boolean;
  fraction: number;
  confirmArm: boolean;
  confirmMove: boolean;
  confirmed?: ConfirmedJoint[];
  repeat: "once" | "count" | "loop";
  count?: number;
  confirmLoop: boolean;
}

/**
 * The Run program press: Go to plan start when the arm is elsewhere, Load
 * unless staged, Arm when disarmed, then Play. A two-step conversation like
 * the operator dialog: without consent OLP answers state 'confirm' with the
 * action and each joint's from/to, and nothing moves; the second press
 * carries the confirm flags and that joints list unchanged (OLP refuses
 * confirmation_stale if a joint moved more than 0.1° since). Loops started
 * here are attended (they end with the heartbeat window).
 */
export async function runProgram(ctx: Context, input: RunInput): Promise<{ ok: boolean; data: Record<string, unknown>; reply: HttpReply }> {
  if (input.repeat === "count" && !input.count) throw new RobotRefusal("repeat 'count' needs a count");
  const plan = await planIdentity(ctx, input);
  const payload = {
    ...plan.identity, fraction: input.fraction, confirm_arm: input.confirmArm, confirm_move: input.confirmMove,
    ...(input.confirmed ? { confirmed: input.confirmed } : {}), repeat: input.repeat, ...(input.count ? { count: input.count } : {}), confirm_loop: input.confirmLoop,
    unattended: false,
  };
  const { reply } = await act(ctx, "Run program", `${DENSE}/run-program`, payload, 300_000);
  const data = replyBody(reply);
  if (!reply.ok) {
    const run = record(data.run);
    throw new RobotRefusal(`Run program stopped at '${String(record(run.refusal).step ?? "?")}': ${refusalText(reply)}`, data);
  }
  return { ok: true, data, reply };
}
