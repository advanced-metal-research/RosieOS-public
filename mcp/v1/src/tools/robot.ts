// Robot actions, through OLP only (it holds the grant, the deadman and the
// maintenance/control gates). Two groups:
//
// - Ending motion: Stop, Disarm, Pause, Finish cycle. Always registered: OLP
//   passes them through every gate from any client, and refusing a Stop is
//   never safer than sending it.
// - Starting motion: Select cell, Arm, Load, Play, Run program, and the
//   bounded jogs in jog.ts. Registered only when ROSIE_MCP_ROBOT_ACTIONS is
//   `simulation` or `cell`; under `simulation` each one re-reads OLP's
//   selected target and refuses unless rt-core itself described it as
//   backend `simulation`. The action carries that read's selection
//   generation, so OLP refuses it if another client switched cells in
//   between.
//
// Never offered, by design: Home (HM35 is run only by an operator at the
// joint's mechanical home pose, never as recovery), fault reset (Arm already
// resets what the drive profile classes resettable; anything else is the
// owner's call), recover-to-zero, open-ended hold-to-run jogging (jog.ts
// offers only moves, steps and holds bounded per call), test-program upload
// and cell catalogue edits.
import type { CallToolResult, McpServer } from "@modelcontextprotocol/server";
import * as z from "zod/v4";

import type { Context } from "rosie-cli/lib/context.js";
import { refusalText } from "rosie-cli/lib/http.js";
import type { DenseStatus } from "rosie-cli/lib/olp.js";
import * as robot from "rosie-cli/lib/robot.js";
import { describeStatus } from "rosie-cli/lib/status.js";
import { failure, guarded, json, text } from "../format.js";

const ENDS_MOTION = { readOnlyHint: false, destructiveHint: false, idempotentHint: true, openWorldHint: true } as const;
const STARTS_MOTION = { readOnlyHint: false, destructiveHint: true, idempotentHint: false, openWorldHint: true } as const;

const identityShape = {
  from_open_program: z.boolean().default(false).describe("Use the plan of the program open in this server (rosie_program_plan) instead of the six identity fields"),
  trajectory_digest: z.string().regex(/^sha256:[0-9a-f]{64}$/).optional().describe("Dense trajectory digest from the plan (sha256:<64 hex>)"),
  plan_id: z.string().min(1).max(200).optional(),
  program_id: z.string().min(1).max(200).optional(),
  program_digest: z.string().min(1).max(200).optional(),
  manifest_revision: z.number().int().min(1).optional(),
  plan_revision: z.number().int().min(1).optional(),
};

const confirmedJoint = z
  .object({
    axis: z.number().int().min(0).max(15),
    id: z.string(),
    from_deg: z.number(),
    to_deg: z.number(),
    delta_deg: z.number(),
    moves: z.boolean(),
  })
  .describe("One joint exactly as the previous 'confirm' answer listed it");

/** A library refusal as the tool's error result; anything else propagates to guarded(). */
async function refusable(body: () => Promise<CallToolResult>): Promise<CallToolResult> {
  try {
    return await body();
  } catch (error) {
    if (error instanceof robot.RobotRefusal) return failure(error.message, error.data);
    throw error;
  }
}

/** The tools' identity arguments as the library's. */
function identityOf(input: Record<string, unknown>): { fromOpenProgram: boolean } & robot.PlanIdentity {
  return { fromOpenProgram: input.from_open_program === true, ...Object.fromEntries(robot.IDENTITY_FIELDS.filter((f) => input[f] !== undefined).map((f) => [f, input[f] as string | number])) };
}

export function registerRobotTools(server: McpServer, ctx: Context): void {
  const { config } = ctx;

  server.registerTool(
    "rosie_olp_stop",
    {
      title: "STOP the robot",
      description: `Press STOP on the selected cell through OLP (POST dense-execution/stop): native Stop, outputs inhibited, the session and grant released, any loop ended at once. OLP accepts Stop from any client, during a maintenance notice and while another client controls the robot. Also ends this server's heartbeats.

A Stop receipt is not proof of standstill: the answer says whether it was confirmed (stopped, attempts). If it is unconfirmed, use the hardware E-stop and tell the operator. Safe to repeat.`,
      inputSchema: z.object({}).strict(),
      annotations: ENDS_MOTION,
    },
    guarded(async () => {
      const stopped = await robot.stop(ctx);
      const data = robot.replyBody(stopped.reply);
      if (!stopped.ok) return failure(`Stop was not confirmed (HTTP ${stopped.reply.status}): ${refusalText(stopped.reply)}. Use the hardware E-stop if the robot may still be moving.`, data);
      return text(`STOP ${stopped.stopped ? "confirmed" : "sent, NOT confirmed"} after ${String(stopped.attempts ?? "?")} attempt(s).${stopped.note ? ` ${String(stopped.note)}` : ""}${stopped.status ? `\n${describeStatus(stopped.status)}` : ""}`, data);
    }),
  );

  server.registerTool(
    "rosie_olp_disarm",
    {
      title: "Disarm the robot",
      description: `Disarm through OLP (POST dense-execution/arm {"armed": false}). Like Stop it passes every gate from any client. Also ends this server's heartbeats. Safe to repeat.`,
      inputSchema: z.object({}).strict(),
      annotations: ENDS_MOTION,
    },
    guarded(() => refusable(async () => {
      const data = await robot.disarm(ctx);
      return text(`Disarm ${data.accepted === false ? "was NOT accepted" : "accepted"}.`, data);
    })),
  );

  server.registerTool(
    "rosie_olp_pause",
    {
      title: "Pause the playing program",
      description: `Pause playback of the loaded program (POST dense-execution/pause {session_id}). session_id defaults to the current session. Pause only slows to a hold on the path; it is not Stop. If no client keeps heartbeating the paused session, OLP's 5 s deadman stops it.`,
      inputSchema: z.object({ session_id: z.string().min(1).max(200).optional() }).strict(),
      annotations: ENDS_MOTION,
    },
    guarded(({ session_id }) => refusable(async () => {
      const paused = await robot.pause(ctx, session_id);
      return text(`Paused session ${paused.sessionId}.\n${describeStatus(paused.status)}`, paused.status);
    })),
  );

  server.registerTool(
    "rosie_olp_finish_loop",
    {
      title: "Finish the running loop after this cycle",
      description: `Finish cycle (POST dense-execution/run-program/finish): the running Run N times / Loop until stopped ends at the program start after the current cycle. It only ends motion sooner. To stop at once, use rosie_olp_stop.`,
      inputSchema: z.object({}).strict(),
      annotations: ENDS_MOTION,
    },
    guarded(() => refusable(async () => {
      const data = await robot.finishLoop(ctx);
      return text(`Loop will finish at the end of this cycle: ${json(data.loop)}`, data);
    })),
  );

  if (config.robotActions === "off") return;

  const mode = config.robotActions === "simulation" ? "simulated cells only (ROSIE_MCP_ROBOT_ACTIONS=simulation)" : "any selected cell, REAL ROBOTS INCLUDED (ROSIE_MCP_ROBOT_ACTIONS=cell)";
  const deadman = `This server heartbeats the session while the request is in flight and, after it succeeds, for up to ${config.heartbeatWindowS} s from the last robot action; then OLP's 5 s deadman stops the robot unless an operator's client keeps it alive.`;

  server.registerTool(
    "rosie_olp_select_cell",
    {
      title: "Connect OLP to a cell",
      description: `Select (connect) a commissioned cell in OLP (POST dense-execution/target {"cell": id}); an empty id disconnects. Selecting Stops whatever the previously selected cell's session was doing and invalidates every client's displayed selection. Refused while another client controls the robot or under a maintenance notice.

Allowed on: ${mode} for the later actions; selecting itself moves nothing. Get ids from rosie_olp_list_cells.`,
      inputSchema: z.object({ cell: z.string().max(200).describe("Cell id to connect, or '' to disconnect") }).strict(),
      annotations: { readOnlyHint: false, destructiveHint: false, idempotentHint: true, openWorldHint: true },
    },
    guarded(({ cell }) => refusable(async () => {
      const selected = await robot.selectCell(ctx, cell);
      return text(`Selected ${cell || "no cell"}.\n${describeStatus(selected.status)}`, selected);
    })),
  );

  server.registerTool(
    "rosie_olp_arm",
    {
      title: "Arm the robot",
      description: `Arm the selected cell through OLP (POST dense-execution/arm {"armed": true}). Arm first settles an unconfirmed Stop and resets only faults whose native recovery is a reset (drive alarms the drive profile classes resettable); it never homes. Arming moves nothing by itself.

Allowed on: ${mode}. ${deadman} Refusals (maintenance 503, in_use_by_other_controller 409, home_invalid, fault_persists, ...) come back in OLP's own words; report them, do not work around them.`,
      inputSchema: z.object({}).strict(),
      annotations: STARTS_MOTION,
    },
    guarded(() => refusable(async () => {
      const data = await robot.arm(ctx);
      return text(`Arm ${data.accepted ? "accepted" : "NOT accepted"}${data.error ? `: ${String(data.error)}` : ""}.${Array.isArray(data.steps) && data.steps.length ? ` Steps: ${json(data.steps)}` : ""}${data.status ? `\n${describeStatus(data.status as DenseStatus)}` : ""}`, data);
    })),
  );

  server.registerTool(
    "rosie_olp_load",
    {
      title: "Load a planned trajectory",
      description: `Load a planned dense trajectory onto the selected cell (POST dense-execution/load) by its six-field identity, as the weld planner filed it (the 'dense' block of a plan result; rosie_inspect_dense_trajectory prints it from an .rdt), or from_open_program=true for the plan rosie_program_plan kept on the open program (refused if it is stale). OLP fetches the bytes from the planner by digest, then checks robot model, description, calibration, Home and the measured start before native preparation. dry_run loads a motion-only check with no process outputs. Loading moves nothing.

Allowed on: ${mode}. ${deadman} Returns the session id and the trajectory summary; play it with rosie_olp_play.`,
      inputSchema: z.object({ ...identityShape, dry_run: z.boolean().optional().describe("Default: the open program's run mode with from_open_program, else false") }).strict(),
      annotations: STARTS_MOTION,
    },
    guarded((input) => refusable(async () => {
      const data = await robot.load(ctx, { ...identityOf(input), dryRun: input.dry_run });
      return text(`Loaded as session ${String(data.session_id)} (bytes from ${String(data.blob_source ?? "planner")}).${Array.isArray(data.notes) && data.notes.length ? `\nNotes: ${data.notes.join(" ")}` : ""}\nTrajectory: ${json(data.trajectory)}`, data);
    })),
  );

  server.registerTool(
    "rosie_olp_play",
    {
      title: "Play the loaded program",
      description: `Play the loaded session (POST dense-execution/play {session_id}); session_id defaults to the current session. THE ROBOT MOVES. Native Play checks that every joint is within its completion tolerance of the program's first sample; if the arm is elsewhere use rosie_olp_run_program, which moves to the program start after a confirmation.

Allowed on: ${mode}. ${deadman} Watch it with rosie_olp_get_machine_status; stop it with rosie_olp_stop.`,
      inputSchema: z.object({ session_id: z.string().min(1).max(200).optional() }).strict(),
      annotations: STARTS_MOTION,
    },
    guarded(({ session_id }) => refusable(async () => {
      const played = await robot.play(ctx, session_id);
      return text(`Playing session ${played.sessionId}.\n${describeStatus(played.status)}`, played.status);
    })),
  );

  server.registerTool(
    "rosie_olp_run_program",
    {
      title: "Run a planned program (one press)",
      description: `The Run program press (POST dense-execution/run-program): Go to plan start when the arm is elsewhere, Load unless staged, Arm when disarmed, then Play. THE ROBOT MOVES.

It is a two-step conversation, exactly like the operator dialog:
  1. Call with the plan identity (or from_open_program=true), fraction and confirm_* false. If something needs consent the answer is state 'confirm' with the action ("Arm and run", "Move to program start and run", ...), each joint's from/to/delta in degrees, and for a repeat the loop question.
  2. Show the user that list. Only if they agree, call again with the same identity, confirm_arm / confirm_move / confirm_loop set as asked, and 'confirmed' = the joints list from step 1 unchanged. OLP refuses 'confirmation_stale' if any joint moved more than 0.1° since.

Args:
  - fraction (0.01-1): speed for the move to plan start, as a fraction of each joint's maximum (default 0.1).
  - repeat: 'once' (default), 'count' with count, or 'loop' (until stopped or finished). Loops from here are always attended: they end when this server's heartbeat window (${config.heartbeatWindowS} s) runs out.
Allowed on: ${mode}. ${deadman}`,
      inputSchema: z
        .object({
          ...identityShape,
          fraction: z.number().min(0.01).max(1).default(0.1),
          confirm_arm: z.boolean().default(false),
          confirm_move: z.boolean().default(false),
          confirmed: z.array(confirmedJoint).max(16).optional(),
          repeat: z.enum(["once", "count", "loop"]).default("once"),
          count: z.number().int().min(1).max(1000).optional(),
          confirm_loop: z.boolean().default(false),
        })
        .strict(),
      annotations: STARTS_MOTION,
    },
    guarded((input) => refusable(async () => {
      const { data } = await robot.runProgram(ctx, {
        ...identityOf(input), fraction: input.fraction, confirmArm: input.confirm_arm, confirmMove: input.confirm_move, confirmed: input.confirmed,
        repeat: input.repeat, count: input.count, confirmLoop: input.confirm_loop,
      });
      if (data.state === "confirm") {
        const joints = Array.isArray(data.joints) ? (data.joints as Array<Record<string, unknown>>) : [];
        return text(
          [
            `Run program needs the operator's confirmation: **${String(data.action)}**${data.needs_arm ? " (arm)" : ""}${data.needs_move ? " (move to program start)" : ""}.`,
            ...joints.map((j) => `- ${String(j.id)}: ${Number(j.from_deg).toFixed(2)}° → ${Number(j.to_deg).toFixed(2)}° (Δ ${Number(j.delta_deg).toFixed(2)}°)${j.moves ? "" : " stays"}`),
            data.loop ? `Loop: ${json(data.loop)}` : "",
            "Nothing moved. Ask the user; if they agree, call again with the confirm flags set and confirmed = these joints.",
          ]
            .filter(Boolean)
            .join("\n"),
          data,
        );
      }
      return text(`Run program: ${String(data.state)}${data.session_id ? ` (session ${String(data.session_id)})` : ""}. Steps: ${json(data.steps)}${data.status ? `\n${describeStatus(data.status as DenseStatus)}` : ""}`, data);
    })),
  );
}
