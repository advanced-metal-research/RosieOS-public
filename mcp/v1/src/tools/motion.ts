// Motion servers: the dense trajectory daemon (joint_trajectory_daemon, TCP
// ingest) and the Cartesian daemon's resolve-only kinematics service. Both
// are used here only in their inert forms: daemon `status` and `validate`
// (nothing stored), and resolver computation (no control client exists in
// that mode).
import { readFile, stat } from "node:fs/promises";

import type { McpServer } from "@modelcontextprotocol/server";
import * as z from "zod/v4";

import type { Context } from "rosie-cli/lib/context.js";
import { deg, failure, guarded, json, responseFormat, result, table, yesNo } from "../format.js";
import { ingestExchange, MAX_FRAME_BYTES } from "rosie-cli/lib/ingest.js";
import { resolve, resolverModels, resolverRequest, transformToPose } from "rosie-cli/lib/kinematics.js";

const READ_ONLY = { readOnlyHint: true, destructiveHint: false, idempotentHint: true, openWorldHint: true } as const;
const COMPUTE = { readOnlyHint: true, destructiveHint: false, idempotentHint: true, openWorldHint: false } as const;

const sixRad = z.array(z.number().finite()).length(6).describe("Six joint angles J1..J6 in radians");

export function registerMotionTools(server: McpServer, ctx: Context): void {
  const { config } = ctx;
  const daemon = `${config.denseDaemonAddr.host}:${config.denseDaemonAddr.port}`;
  // The arms the resolver can serve are the descriptions in its repository:
  // every robot_description/robots directory with a robot.urdf.
  const models = resolverModels(config.cartesiandRepoRoot);
  const model = models.length ? z.enum(models as [string, ...string[]]) : z.string().regex(/^[a-z0-9]+(?:_[a-z0-9]+)*$/);

  server.registerTool(
    "rosie_dense_daemon_status",
    {
      title: "Get the dense trajectory daemon's status",
      description: `Read joint_trajectory_daemon's status over its TCP ingest (${daemon}, kind 'status'): executor state, staged plan identity, cursor (segment, sample, t_s), armed, authority freshness and the native feedback view per axis.

Read-only: the TCP ingest carries no play or stop; those are NATS commands owned by the authority holder.`,
      inputSchema: z.object({ response_format: responseFormat }).strict(),
      annotations: READ_ONLY,
    },
    guarded(async ({ response_format }) => {
      const reply = await ingestExchange(config.denseDaemonAddr, "status", undefined, config.requestTimeoutMs);
      return result(response_format, reply, () => `# Dense daemon status (${daemon})\n\n\`\`\`json\n${json(reply)}\n\`\`\``);
    }),
  );

  server.registerTool(
    "rosie_dense_daemon_validate",
    {
      title: "Validate a dense trajectory with the daemon",
      description: `Send a local .rdt file (robot.v4.dense-joint-trajectory.v1) to the dense daemon's 'validate' ingest (${daemon}): the daemon's own decoder runs every format rule and answers with the digest or the rejection reason and segment. Nothing is stored or staged; it is upload's dry run.

For an offline check that needs no daemon, use rosie_inspect_dense_trajectory (same rules, ported).`,
      inputSchema: z.object({ path: z.string().min(1).describe("Absolute path to a .rdt file") }).strict(),
      annotations: READ_ONLY,
    },
    guarded(async ({ path }) => {
      const info = await stat(path);
      if (info.size > MAX_FRAME_BYTES) return failure(`${path} is ${info.size} bytes; the daemon's frame limit is ${MAX_FRAME_BYTES}`);
      const reply = await ingestExchange(config.denseDaemonAddr, "validate", await readFile(path), Math.max(config.requestTimeoutMs, 60_000));
      const ok = (reply as Record<string, unknown> | null)?.ok === true;
      return { ...result("markdown", reply, () => `# Daemon validate: ${ok ? "ACCEPTED" : "REJECTED"}\n\n\`\`\`json\n${json(reply)}\n\`\`\``), isError: !ok };
    }),
  );

  const resolverCommand = (): string[] => {
    if (!config.cartesiandCommand) {
      throw new Error(
        "robot-v4-cartesiand is not available: build it (make -C motion-server/v1 BIN_DIR=$PWD/motion-server/v1/bin all) or set ROSIE_CARTESIAND_COMMAND (a path, or a JSON argv prefix such as [\"wsl\",\"-e\",\"/home/me/RosieOS/motion-server/v1/bin/robot-v4-cartesiand\"] with ROSIE_CARTESIAND_REPO_ROOT as that process sees the repository)",
      );
    }
    return config.cartesiandCommand;
  };

  server.registerTool(
    "rosie_kinematics_forward",
    {
      title: "Forward kinematics of a joint pose",
      description: `Compute the TCP pose of a robot model at six joint angles, with the same URDF and kinematics the Cartesian jog uses (robot-v4-cartesiand --resolve-only, operation 'fk'). Pure computation: no controller, no grant, no motion.

Args:
  - model: ${models.join(", ") || "a robot_description model id"} (robot_description/robots/<model>/robot.urdf).
  - joints_rad: J1..J6 in radians.
Returns the row-major 3x4 transform, position in mm and roll/pitch/yaw in degrees (ZYX). Joint limits are not checked here.`,
      inputSchema: z.object({ model, joints_rad: sixRad, response_format: responseFormat }).strict(),
      annotations: COMPUTE,
    },
    guarded(async ({ model, joints_rad, response_format }) => {
      const verdict = await resolve(resolverCommand(), config.cartesiandRepoRoot, model, resolverRequest(model, "fk", joints_rad), config.requestTimeoutMs);
      if (!verdict.accepted || !verdict.transform?.length) return failure(`forward kinematics refused: ${verdict.reason || "no transform"}`, verdict);
      const pose = transformToPose(verdict.transform);
      const data = { model, joints_rad, transform: verdict.transform, position_mm: pose.position_m.map((v) => v * 1000), rpy_deg: pose.rpy_rad.map((v) => (v * 180) / Math.PI) };
      return result(response_format, data, () =>
        [
          `# FK ${model} at [${joints_rad.map((q) => deg(q)).join(", ")}]`,
          `- **Position**: X ${data.position_mm[0]!.toFixed(3)} mm, Y ${data.position_mm[1]!.toFixed(3)} mm, Z ${data.position_mm[2]!.toFixed(3)} mm`,
          `- **Orientation (RPY, ZYX)**: ${data.rpy_deg.map((v) => `${v.toFixed(3)}°`).join(", ")}`,
        ].join("\n"),
      );
    }),
  );

  server.registerTool(
    "rosie_kinematics_cartesian_line",
    {
      title: "Resolve a straight Cartesian move into joint waypoints",
      description: `Resolve a straight TCP move from a joint pose into joint waypoints (robot-v4-cartesiand --resolve-only, operation 'move'): the same line construction, IK, joint-limit and Jacobian gates as OLP's Cartesian steps. The whole segment is accepted or refused; nothing moves.

Args:
  - model, joints_rad: the starting pose.
  - frame: 'base' or 'tool' (axes of the TCP frame at the start pose).
  - axis: x, y, z (translation, distance in mm) or rx, ry, rz (rotation, distance in degrees). Exactly one axis per move.
  - distance: signed amount.
Returns accepted, refusal reason (joint_limit, jacobian_gate, ik_no_solution, cartesian_input_invalid), the number of waypoints, the end pose and the largest joint travel.`,
      inputSchema: z
        .object({
          model,
          joints_rad: sixRad,
          frame: z.enum(["base", "tool"]).default("base"),
          axis: z.enum(["x", "y", "z", "rx", "ry", "rz"]),
          distance: z.number().finite().refine((v) => v !== 0, "distance must be nonzero"),
          response_format: responseFormat,
        })
        .strict(),
      annotations: COMPUTE,
    },
    guarded(async ({ model, joints_rad, frame, axis, distance, response_format }) => {
      const index = ["x", "y", "z", "rx", "ry", "rz"].indexOf(axis);
      const delta = [0, 0, 0, 0, 0, 0];
      delta[index] = index < 3 ? distance / 1000 : (distance * Math.PI) / 180;
      const verdict = await resolve(resolverCommand(), config.cartesiandRepoRoot, model, resolverRequest(model, "move", joints_rad, { frame, delta }), config.requestTimeoutMs);
      const waypoints = verdict.waypoints ?? [];
      const end = waypoints.at(-1) ?? null;
      const travel = end ? end.map((q, j) => Math.abs(q - joints_rad[j]!)) : [];
      const data = {
        accepted: verdict.accepted,
        reason: verdict.reason,
        waypoint_count: waypoints.length,
        end_joints_rad: end,
        joint_travel_rad: travel,
        singularity_scale: verdict.singularity_scale,
        joint_limit_scale: verdict.joint_limit_scale,
      };
      return result(response_format, data, () =>
        [
          `# ${frame.toUpperCase()} ${axis} ${distance > 0 ? "+" : ""}${distance}${index < 3 ? " mm" : "°"} from [${joints_rad.map((q) => deg(q)).join(", ")}] on ${model}`,
          `- **Accepted**: ${yesNo(verdict.accepted)}${verdict.reason ? ` (${verdict.reason})` : ""} · ${waypoints.length} waypoints`,
          end ? table(["Joint", "Start", "End", "Travel"], end.map((q, j) => [`J${j + 1}`, deg(joints_rad[j]), deg(q), deg(travel[j])])) : "",
        ].join("\n"),
      );
    }),
  );
}
