// Bounded jogs as MCP tools. The logic -- the per-call bounds, the browser's
// own requests, End/Halt always sent, Stop only after a hold that began and
// then failed, heartbeats around every request -- is rosie-cli/lib/jog.ts,
// shared with the CLI; this file is the tools' schemas and their text.
// Registered with the other motion-starting tools: only when
// ROSIE_MCP_ROBOT_ACTIONS is `simulation` or `cell`.
import type { CallToolResult, McpServer } from "@modelcontextprotocol/server";
import * as z from "zod/v4";

import type { Context } from "rosie-cli/lib/context.js";
import { AT_TARGET_DEG, CADENCE_MS, CARTESIAN_AXES, cartesianStep, fmt, jogHold, moveJoints, toDeg } from "rosie-cli/lib/jog.js";
import { RobotRefusal } from "rosie-cli/lib/robot.js";
import { failure, guarded, text } from "../format.js";

const STARTS_MOTION = { readOnlyHint: false, destructiveHint: true, idempotentHint: false, openWorldHint: true } as const;

/** A library refusal as the tool's error result; anything else propagates to guarded(). */
async function refusable(body: () => Promise<CallToolResult>): Promise<CallToolResult> {
  try {
    return await body();
  } catch (error) {
    if (error instanceof RobotRefusal) return failure(error.message, error.data);
    throw error;
  }
}

export function registerJogTools(server: McpServer, ctx: Context): void {
  const { config } = ctx;
  if (config.robotActions === "off") return;
  const limits = config.jog;
  const mode = config.robotActions === "simulation" ? "simulated cells only (ROSIE_MCP_ROBOT_ACTIONS=simulation)" : "any selected cell, REAL ROBOTS INCLUDED (ROSIE_MCP_ROBOT_ACTIONS=cell)";
  const never = "Home, fault reset and recover-to-zero are never offered; OLP refuses jogging while a program plays or a loop runs, and that refusal is the answer.";

  server.registerTool(
    "rosie_olp_move_joints",
    {
      title: "Move joints by or to an angle",
      description: `One joint-space move on the selected cell through OLP (POST dense-execution/move), the browser's Move by angle / All joints to 0° request. THE ROBOT MOVES. Every listed joint moves in ONE synchronised move: one shared progress along the straight joint segment, all start and finish together. Returns when OLP reports correlated native completion ('done'), or its refusal.

Args:
  - mode: 'relative' (deg = signed travel) or 'absolute' (deg = target angle; OLP plans it from the fresh held position, as Go to plan start does). Absolute joints already within ${AT_TARGET_DEG}° are left out.
  - joints: [{joint: 'J1', deg}], each joint once, ids as the cell describes them.
  - fraction (0.01-${limits.maxMoveFraction}, default ${Math.min(0.1, limits.maxMoveFraction)}): of each joint's described maximum velocity; OLP runs it at 0.75 of that.
Bounds here, before anything is sent: at most ${limits.maxJointDeg}° of travel per joint per call (ROSIE_MCP_JOG_MAX_DEG), computed from the live position for absolute targets, and the described joint range. OLP and rt-core apply their own limits after.
Needs an armed session (rosie_olp_arm). This server heartbeats while the move is in flight (OLP's 5 s deadman applies during it) and restarts its window when it completes. Allowed on: ${mode}. ${never}`,
      inputSchema: z
        .object({
          mode: z.enum(["relative", "absolute"]),
          joints: z.array(z.object({ joint: z.string().min(1).max(32), deg: z.number() }).strict()).min(1).max(16),
          fraction: z.number().min(0.01).max(limits.maxMoveFraction).default(Math.min(0.1, limits.maxMoveFraction)),
        })
        .strict(),
      annotations: STARTS_MOTION,
    },
    guarded(({ mode: kind, joints, fraction }) => refusable(async () => {
      const moved = await moveJoints(ctx, { mode: kind, joints, fraction });
      if (moved.nothingToMove) return text(`Nothing to move: every listed joint is within ${AT_TARGET_DEG}° of its target.`, { planned: moved.planned });
      return text(
        [
          `Move done (${moved.planned.length} joint${moved.planned.length > 1 ? "s" : ""}, fraction ${fraction}, revision ${String(moved.revision)}):`,
          ...moved.planned.map((p) => `- ${p.id}: ${p.from === null ? "?" : fmt(p.from)} → ${p.toDeg === null ? `Δ ${fmt(p.deltaDeg)}` : fmt(p.toDeg)}${moved.now[p.index] != null ? ` (now ${fmt(toDeg(moved.now[p.index]!))})` : ""}`),
        ].join("\n"),
        { request: moved.request, reply: moved.reply, planned: moved.planned },
      );
    })),
  );

  server.registerTool(
    "rosie_olp_cartesian_step",
    {
      title: "Step the TCP in a straight line",
      description: `One straight TCP step on the selected cell through OLP (POST dense-execution/cartesian/move), the browser's Cartesian Step press. THE ROBOT MOVES. OLP resolves the step from the machine's held pose (never a pose sent from here) and returns when native completion is correlated, or its refusal (cartesian_resolver_unavailable, singularity and reach refusals, ...).

Args:
  - frame: 'base' (default) or 'tool'.
  - axis: x, y, z (amount in mm) or rx, ry, rz (amount in degrees), one axis per step.
  - amount: signed, nonzero; at most ${limits.maxCartesianMm} mm / ${limits.maxCartesianDeg}° per call here (ROSIE_MCP_CARTESIAN_MAX_MM / _DEG).
  - fraction (0.01-${limits.maxMoveFraction}, default ${Math.min(0.1, limits.maxMoveFraction)}).
Needs an armed session. Allowed on: ${mode}. ${never}`,
      inputSchema: z
        .object({
          frame: z.enum(["base", "tool"]).default("base"),
          axis: z.enum(CARTESIAN_AXES),
          amount: z.number().describe("Signed step: mm for x/y/z, degrees for rx/ry/rz"),
          fraction: z.number().min(0.01).max(limits.maxMoveFraction).default(Math.min(0.1, limits.maxMoveFraction)),
        })
        .strict(),
      annotations: STARTS_MOTION,
    },
    guarded(({ frame, axis, amount, fraction }) => refusable(async () => {
      const stepped = await cartesianStep(ctx, { frame, axis, amount, fraction });
      return text(`Cartesian step done: ${axis} ${amount > 0 ? "+" : ""}${amount} ${stepped.unit} in the ${frame} frame at fraction ${fraction}.`, { request: stepped.request, reply: stepped.reply });
    })),
  );

  server.registerTool(
    "rosie_olp_jog_hold",
    {
      title: "Jog for a fixed time (timed hold)",
      description: `A hold-to-run jog with a timer for a hand, through OLP. THE ROBOT MOVES for duration_s. Joint: POST dense-execution/jog/begin, update every ${CADENCE_MS} ms, end (the browser's +/- buttons). Cartesian: cartesian/start, intent, halt (the path hold: stops along the line). End/Halt is ALWAYS sent when the time is up or anything fails; once OLP accepted the press, if End fails or an intent fails mid-hold, rosie_olp_stop's Stop follows (which also releases the grant). A refused press (jog_mode_conflict while a program plays or a loop runs, ...) gets End only, never Stop. Reports OLP's jog end reason (last_jog_end).

Args:
  - kind: 'joint' (with joint, e.g. 'J3') or 'cartesian' (with axis x..rz and frame).
  - direction: 'positive' or 'negative'.
  - duration_s (default ${Math.min(1, limits.maxHoldS)}, max ${limits.maxHoldS}, ROSIE_MCP_JOG_HOLD_MAX_S), counted from OLP's answer to the press.
  - fraction (0.01-${limits.maxHoldFraction}, default ${Math.min(0.1, limits.maxHoldFraction)}). Joint: of the joint's described maximum velocity (x0.75 in OLP). Cartesian: of speed_mm_s / speed_deg_s (the browser's boxes, default 100 mm/s and 15°/s).
Bounds here: the commanded travel (speed x duration) at most ${limits.maxJointDeg}° per joint, ${limits.maxCartesianMm} mm / ${limits.maxCartesianDeg}° for the TCP. Allowed on: ${mode}. ${never}`,
      inputSchema: z
        .object({
          kind: z.enum(["joint", "cartesian"]),
          joint: z.string().min(1).max(32).optional().describe("kind 'joint': the joint id, e.g. 'J3'"),
          axis: z.enum(CARTESIAN_AXES).optional().describe("kind 'cartesian': x, y, z, rx, ry or rz"),
          frame: z.enum(["base", "tool"]).default("base"),
          direction: z.enum(["positive", "negative"]),
          duration_s: z.number().min(0.05).max(limits.maxHoldS).default(Math.min(1, limits.maxHoldS)),
          fraction: z.number().min(0.01).max(limits.maxHoldFraction).default(Math.min(0.1, limits.maxHoldFraction)),
          speed_mm_s: z.number().gt(0).max(1000).default(100).describe("Cartesian linear speed at 100% (mm/s)"),
          speed_deg_s: z.number().gt(0).max(180).default(15).describe("Cartesian angular rate at 100% (deg/s)"),
        })
        .strict(),
      annotations: STARTS_MOTION,
    },
    guarded((input) => refusable(async () => {
      const outcome = await jogHold(ctx, {
        kind: input.kind, joint: input.joint, axis: input.axis, frame: input.frame, direction: input.direction,
        durationS: input.duration_s, fraction: input.fraction, speedMmS: input.speed_mm_s, speedDegS: input.speed_deg_s,
      });
      const end = outcome.lastJogEnd;
      const lines = [
        `Jog hold ${outcome.label} at fraction ${input.fraction}: ${outcome.summary}`,
        `- intents sent: ${outcome.updates}, held for ${(outcome.heldMs / 1000).toFixed(2)} s of ${input.duration_s} s`,
        `- end (${outcome.routes.end}, end_cause ${outcome.endCause}): ${outcome.endText}`,
        outcome.stopText ? `- STOP: ${outcome.stopText}` : "",
        end ? `- OLP's jog end: ${String(end.code ?? "?")}: ${String(end.text ?? "")}${end.repull ? ` (pull again: ${String(end.repull)})` : ""}` : "- OLP reported no last_jog_end",
        outcome.hint ? `- ${outcome.hint}` : "",
      ].filter(Boolean);
      const data = { request: outcome.request, routes: outcome.routes, ok: outcome.ok, summary: outcome.summary, updates: outcome.updates, heldMs: outcome.heldMs, endCause: outcome.endCause, endText: outcome.endText, stopText: outcome.stopText, last_jog_end: end };
      return outcome.ok ? text(lines.join("\n"), data) : failure(lines.join("\n"), data);
    })),
  );
}
