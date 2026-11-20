// traj, kin, planner: dense trajectories (.rdt), .weldplan files, the weld
// planner and the Cartesian resolver. Computation and inspection only.
import { readFile, stat } from "node:fs/promises";

import { fetchJSON, refusalText } from "../../lib/http.js";
import { ingestExchange } from "../../lib/ingest.js";
import { resolve, resolverModels, resolverRequest, transformToPose, type ResolverModel } from "../../lib/kinematics.js";
import { fetchDense, planWeldplan } from "../../lib/planner.js";
import { summarize, type TrajectorySummary } from "../../lib/rdt.js";
import { deg, json, record, table, yesNo } from "../../lib/text.js";
import { ok, refused, UsageError, type Command, type Outcome } from "../command.js";
import type { Env } from "../env.js";

function trajectoryOutcome(summary: TrajectorySummary & { saved_to?: string }, artifacts: string[] = []): Outcome {
  if (!summary.valid) {
    const r = summary.rejection!;
    return refused(`REJECTED: ${r.reason}${r.segment_index !== null ? ` (segment ${r.segment_index})` : ""} — the dense daemon would refuse it`, { body: r.detail, data: summary, artifacts });
  }
  const id = summary.identity!;
  return ok(`valid: ${summary.total_samples} samples, ${summary.total_duration_s?.toFixed(3)} s, ${summary.segments.length} segments`, {
    body: [
      `identity: trajectory_digest ${id.trajectory_digest} · plan_id ${id.plan_id} · program_id ${id.program_id} · program_digest ${id.program_digest} · manifest_revision ${id.manifest_revision} · plan_revision ${id.plan_revision}`,
      `robot: ${summary.robot_cell ? JSON.stringify(summary.robot_cell) : "none (the daemon requires one at preload)"} · measured start ${summary.start_state ? "present" : "none"} · dt ${summary.dt_s} s`,
      "",
      table(["#", "Kind", "Source", "Samples", "Duration s", "Torch samples"], summary.segments.map((s) => [s.index, s.kind, s.source_id, s.samples, s.duration_s.toFixed(3), s.torch_samples])),
      "",
      table(["Axis", "Min", "Max", "Peak speed", "Ceiling", "Use"], summary.axes.filter((a) => a.in_mask).map((a) => [a.axis, deg(a.min_q_rad), deg(a.max_q_rad), `${deg(a.max_abs_qd_rad_s)}/s`, `${deg(a.limit_qd_rad_s)}/s`, `${(a.qd_utilisation * 100).toFixed(1)} %`])),
    ].join("\n"),
    data: summary, artifacts,
    next: ["rosie robot load --trajectory <file> --sim", "rosie run --trajectory <file> --sim"],
  });
}

const inspect: Command<Env> = {
  path: ["traj", "inspect"],
  summary: "Validate a .rdt offline with every dense-daemon ingest rule; print its identity, segments, joint ranges and peak speeds",
  options: [{ name: "file", type: "string", description: ".rdt file", positional: 0, required: true }],
  examples: ["rosie traj inspect ~/.rosie/mcp/dense/ea9641e1….rdt"],
  async run(_env, args) {
    const file = String(args.file);
    if ((await stat(file)).size > 64 * 1024 * 1024) return refused(`${file} is beyond the 64 MiB frame limit`);
    return trajectoryOutcome(summarize(await readFile(file)));
  },
};

const validate: Command<Env> = {
  path: ["traj", "validate"],
  summary: "Send a .rdt to the dense daemon's own validate ingest (its decoder; nothing stored)",
  options: [{ name: "file", type: "string", description: ".rdt file", positional: 0, required: true }],
  async run(env, args) {
    const ctx = await env.ctx();
    const reply = record(await ingestExchange(ctx.config.denseDaemonAddr, "validate", await readFile(String(args.file)), Math.max(ctx.config.requestTimeoutMs, 60_000)));
    const accepted = reply.ok === true || reply.accepted === true || (typeof reply.digest === "string" && !reply.error);
    return (accepted ? ok : refused)(accepted ? `the daemon accepts it: ${String(reply.digest ?? reply.trajectory_digest ?? "")}` : `the daemon rejects it: ${String(reply.reason ?? reply.error ?? "?")}`, { body: json(reply), data: reply });
  },
};

const fetchCommand: Command<Env> = {
  path: ["traj", "fetch"],
  summary: "Download a planned trajectory from the weld planner by digest, save it and inspect it",
  options: [
    { name: "digest", type: "string", description: "sha256:<64 hex>", positional: 0, required: true },
    { name: "out", type: "string", description: "Where to save the .rdt", short: "o" },
  ],
  async run(env, args) {
    if (!/^sha256:[0-9a-f]{64}$/.test(String(args.digest))) throw new UsageError("the digest is sha256:<64 hex>");
    const summary = await fetchDense((await env.ctx()).config, String(args.digest), args.out as string | undefined);
    return trajectoryOutcome(summary, [summary.saved_to]);
  },
};

const planFile: Command<Env> = {
  path: ["traj", "plan-weldplan"],
  summary: "Plan a .weldplan on the weld planner (seconds to minutes); full result to a file, summary printed",
  options: [
    { name: "file", type: "string", description: ".weldplan (rosie program export)", positional: 0, required: true },
    { name: "out", type: "string", description: "Where to write the full JSON result", short: "o" },
    { name: "no_dense", type: "boolean", description: "Skip the dense export" },
    { name: "program_id", type: "string", description: "Identity field the daemon fences on" },
    { name: "timeout_s", type: "number", description: "Give up after", default: 600 },
  ],
  async run(env, args) {
    env.progress("planning on the weld planner…");
    const summary = await planWeldplan((await env.ctx()).config, String(args.file), { dense: !args.no_dense, programId: args.program_id as string | undefined, timeoutS: Number(args.timeout_s), outputPath: args.out as string | undefined });
    const failed = Boolean(summary.connecting_error || summary.dense_error);
    return (failed ? refused : ok)(`planned in ${summary.elapsed_s.toFixed(1)} s: ${summary.seams} seams (${summary.weld_trajectories} weld trajectories), ${summary.connecting_moves ?? "?"} connecting moves${summary.dense_error ? " · DENSE EXPORT REFUSED" : ""}${summary.connecting_error ? " · CONNECTING MOVES FAILED" : ""}`, {
      body: [summary.dense ? `dense: ${json(summary.dense)}` : "", summary.dense_error ? `dense refused: ${json(summary.dense_error)}` : "", summary.connecting_error ? `connecting failed: ${String(summary.connecting_error)}` : "", summary.timings_ms ? `timings (ms): ${json(summary.timings_ms)}` : ""].filter(Boolean).join("\n"),
      data: summary, artifacts: [summary.result_path],
    });
  },
};

const plannerStatus: Command<Env> = {
  path: ["planner", "status"],
  summary: "The weld planner: CUDA or CPU-only, installed revision, warm worker, the plan in flight",
  async run(env) {
    const ctx = await env.ctx();
    const base = ctx.config.plannerUrl;
    const [health, progress] = await Promise.all([fetchJSON("weld planner", `${base}/api/motion/health`, { timeoutMs: ctx.config.requestTimeoutMs }), fetchJSON("weld planner", `${base}/api/motion/progress`, { timeoutMs: ctx.config.requestTimeoutMs })]);
    if (!health.ok) return refused(`planner health: ${refusalText(health)}`);
    const h = record(health.body);
    return ok(`${base}: ${String(h.device ?? (h.cuda ? "CUDA" : "CPU"))}${h.source_revision ? ` · revision ${String(h.source_revision)}` : ""} · ${record(progress.body).stage ? `planning (${String(record(progress.body).stage)})` : "idle"}`, { body: json({ health: h, progress: progress.body }), data: { health: h, progress: progress.body } });
  },
};

function radians(value: unknown): number[] {
  const list = value as number[];
  if (!Array.isArray(list) || list.length !== 6) throw new UsageError("--joints takes J1..J6 in degrees, e.g. 0,-30,45,0,60,0");
  return list.map((d) => (d * Math.PI) / 180);
}

// The model must be an arm this tree describes; the resolver would refuse
// anything else, so say which ones exist instead.
function arm(repoRoot: string, value: unknown): ResolverModel {
  const models = resolverModels(repoRoot);
  const model = String(value);
  if (!models.includes(model)) throw new UsageError(`--model ${model} is not an arm in ${repoRoot}/robot_description/robots; known: ${models.join(", ") || "none"}`);
  return model;
}

function cartesiand(env: Env, command: string[] | null): string[] {
  if (!command) throw new UsageError("robot-v4-cartesiand is not built: make -C motion-server/v1 (or set ROSIE_CARTESIAND_COMMAND)");
  return command;
}

const fk: Command<Env> = {
  path: ["kin", "fk"],
  summary: "TCP pose of a model at six joint angles (degrees), with the Cartesian jog's own kinematics",
  options: [
    { name: "model", type: "string", description: "Arm model: a robot_description/robots directory with a robot.urdf", required: true },
    { name: "joints", type: "numbers", description: "J1..J6 degrees", required: true },
  ],
  examples: ["rosie kin fk --model rosie_1420_v1 --joints 0,-30,45,0,60,0"],
  async run(env, args) {
    const ctx = await env.ctx();
    const model = arm(ctx.config.cartesiandRepoRoot, args.model);
    const verdict = await resolve(cartesiand(env, ctx.config.cartesiandCommand), ctx.config.cartesiandRepoRoot, model, resolverRequest(model, "fk", radians(args.joints)), ctx.config.requestTimeoutMs);
    if (!verdict.accepted || !verdict.transform?.length) return refused(`forward kinematics refused: ${verdict.reason || "no transform"}`, { data: verdict });
    const pose = transformToPose(verdict.transform);
    const mm = pose.position_m.map((v) => v * 1000);
    const rpy = pose.rpy_rad.map((v) => (v * 180) / Math.PI);
    return ok(`TCP at [${mm.map((v) => v.toFixed(2)).join(", ")}] mm, rpy [${rpy.map((v) => v.toFixed(2)).join(", ")}]°`, { data: { model, transform: verdict.transform, position_mm: mm, rpy_deg: rpy } });
  },
};

const line: Command<Env> = {
  path: ["kin", "line"],
  summary: "Would OLP's Cartesian step accept this straight line? (same IK, joint-limit and Jacobian gates; nothing moves)",
  options: [
    { name: "model", type: "string", description: "Arm model: a robot_description/robots directory with a robot.urdf", required: true },
    { name: "joints", type: "numbers", description: "Start pose J1..J6 degrees", required: true },
    { name: "axis", type: "string", description: "x y z (mm) | rx ry rz (°)", required: true, choices: ["x", "y", "z", "rx", "ry", "rz"] },
    { name: "distance", type: "number", description: "Signed mm or degrees", required: true },
    { name: "frame", type: "string", description: "base | tool", choices: ["base", "tool"], default: "base" },
  ],
  examples: ["rosie kin line --model rosie_1420_v1 --joints 0,-30,45,0,60,0 --axis z --distance -50"],
  async run(env, args) {
    const ctx = await env.ctx();
    const model = arm(ctx.config.cartesiandRepoRoot, args.model);
    const axis = String(args.axis);
    const distance = Number(args.distance);
    if (!distance) throw new UsageError("--distance must be nonzero");
    const index = ["x", "y", "z", "rx", "ry", "rz"].indexOf(axis);
    const delta = [0, 0, 0, 0, 0, 0];
    delta[index] = index < 3 ? distance / 1000 : (distance * Math.PI) / 180;
    const verdict = await resolve(cartesiand(env, ctx.config.cartesiandCommand), ctx.config.cartesiandRepoRoot, model, resolverRequest(model, "move", radians(args.joints), { frame: args.frame, delta }), ctx.config.requestTimeoutMs);
    return (verdict.accepted ? ok : refused)(verdict.accepted ? `accepted: ${verdict.waypoints?.length ?? "?"} waypoints` : `refused: ${verdict.reason}`, { data: verdict, body: yesNo(verdict.accepted) });
  },
};

export const commands: Command<Env>[] = [inspect, validate, fetchCommand, planFile, plannerStatus, fk, line];
