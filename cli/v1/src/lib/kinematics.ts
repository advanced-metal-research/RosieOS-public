// `robot-v4-cartesiand --resolve-only REPO_ROOT MODEL` (motion-server/v1
// README, "Resolve-only Cartesian service"): JSON request lines in, one JSON
// verdict line out. It constructs no control client and sends no motion;
// its answers are computation, never permission to move.
import { spawn } from "node:child_process";
import { existsSync, readdirSync } from "node:fs";
import path from "node:path";

/** A model id: the name of its description directory under robot_description/robots. */
export type ResolverModel = string;

/**
 * The arms the resolver can serve from this tree: every description
 * directory under robot_description/robots that holds a robot.urdf. The
 * description is the one record of a model, so no model id is written here.
 */
export function resolverModels(repoRoot: string): ResolverModel[] {
  const robots = path.join(repoRoot, "robot_description", "robots");
  if (!existsSync(robots)) return [];
  return readdirSync(robots, { withFileTypes: true })
    .filter((entry) => entry.isDirectory() && /^[a-z0-9]+(?:_[a-z0-9]+)*$/.test(entry.name) && existsSync(path.join(robots, entry.name, "robot.urdf")))
    .map((entry) => entry.name)
    .sort();
}

export interface ResolverVerdict {
  accepted: boolean;
  reason: string;
  velocities: number[];
  joint_limit_scale: number;
  singularity_scale: number;
  waypoints: number[][];
  transform?: number[];
}

// The service narrows these to the model's URDF limits, so wide bounds ask
// only for the robot's own; they never widen anything.
const OPEN_LOWER = [-1e3, -1e3, -1e3, -1e3, -1e3, -1e3];
const OPEN_UPPER = [1e3, 1e3, 1e3, 1e3, 1e3, 1e3];

export function resolverRequest(model: ResolverModel, operation: "fk" | "move", pose: number[], extra: Record<string, unknown> = {}): Record<string, unknown> {
  return {
    model,
    frame: "base",
    twist: [0, 0, 0, 0, 0, 0],
    pose,
    lower: OPEN_LOWER,
    upper: OPEN_UPPER,
    fraction: 1,
    input_age_ns: 0,
    operation,
    ...extra,
  };
}

export async function resolve(
  command: string[],
  repoRoot: string,
  model: ResolverModel,
  request: Record<string, unknown>,
  timeoutMs: number,
): Promise<ResolverVerdict> {
  const [program, ...prefix] = command;
  if (!program) throw new Error("no cartesiand command configured");
  return new Promise((resolvePromise, reject) => {
    const child = spawn(program, [...prefix, "--resolve-only", repoRoot, model], { stdio: ["pipe", "pipe", "pipe"], windowsHide: true });
    let stdout = "";
    let stderr = "";
    const timer = setTimeout(() => {
      child.kill();
      reject(new Error(`cartesiand did not answer within ${timeoutMs} ms`));
    }, timeoutMs);
    child.stdout.on("data", (chunk: Buffer) => {
      stdout += chunk.toString("utf8");
      const newline = stdout.indexOf("\n");
      if (newline < 0) return;
      clearTimeout(timer);
      child.stdin.end();
      try {
        resolvePromise(JSON.parse(stdout.slice(0, newline)) as ResolverVerdict);
      } catch {
        reject(new Error(`cartesiand answered with a line that is not JSON: ${stdout.slice(0, 200)}`));
      }
    });
    child.stderr.on("data", (chunk: Buffer) => {
      stderr = (stderr + chunk.toString("utf8")).slice(-2000);
    });
    child.on("error", (error) => {
      clearTimeout(timer);
      reject(new Error(`could not start cartesiand (${command.join(" ")}): ${error.message}`));
    });
    child.on("close", (code) => {
      clearTimeout(timer);
      if (!stdout.includes("\n")) reject(new Error(`cartesiand exited ${code} without an answer${stderr ? `: ${stderr.trim()}` : ""}`));
    });
    child.stdin.write(`${JSON.stringify(request)}\n`);
  });
}

/** Row-major 3x4 TCP transform -> position (m) and roll/pitch/yaw (rad, ZYX). */
export function transformToPose(transform: number[]): { position_m: number[]; rpy_rad: number[] } {
  const m = (index: number) => transform[index] ?? NaN;
  const pitch = Math.asin(Math.max(-1, Math.min(1, -m(8))));
  return {
    position_m: [m(3), m(7), m(11)],
    rpy_rad: [Math.atan2(m(9), m(10)), pitch, Math.atan2(m(4), m(0))],
  };
}
