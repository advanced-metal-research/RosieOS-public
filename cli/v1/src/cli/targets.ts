// Where `rosie` points: named targets in ~/.rosie/rosie.json, so switching
// from the local stack to a cell's tunnel is `--target <cell>`, never a
// re-export of six variables (or a restart that loses state, as the MCP
// server's environment did).
//
// Precedence: an explicitly named target's own values win over the
// environment, so `--target <cell>` can never silently reach a local service
// because ROSIE_OLP_URL is still exported; without --target the environment
// overrides the default target (dev-stack and CI set variables).
import { randomBytes } from "node:crypto";
import { existsSync } from "node:fs";
import { mkdir, readFile, rename, writeFile } from "node:fs/promises";
import { homedir, userInfo } from "node:os";
import path from "node:path";

import { loadConfig, type Config, type RobotActionMode } from "../lib/config.js";

export interface Target {
  /** OLP API, e.g. http://127.0.0.1:8794 (local) or http://127.0.0.1:18894 (`olp ui start` tunnel to a cell). */
  olp_url?: string;
  planner_url?: string;
  /** host:port of the dense daemon's TCP ingest. */
  dense_daemon?: string;
  rt_control_socket?: string;
  rt_control_url?: string;
  rt_control_ca?: string;
  rt_control_cert?: string;
  rt_control_key?: string;
  /** off (default) | simulation | cell. `cell` (real robots) is the owner's setting only. */
  robot_actions?: RobotActionMode;
  heartbeat_window_s?: number;
  note?: string;
}

export interface Profile {
  default_target: string;
  targets: Record<string, Target>;
}

export function rosieHome(env: NodeJS.ProcessEnv = process.env): string {
  return path.resolve(env.ROSIE_HOME || path.join(homedir(), ".rosie"));
}

export function profilePath(env: NodeJS.ProcessEnv = process.env): string {
  return path.join(rosieHome(env), "rosie.json");
}

/** dev-stack.sh's public control socket: $ROSIE_LOCAL_RT_DIR or ${XDG_RUNTIME_DIR:-${TMPDIR:-/tmp}}/rosie-dev-rt-$UID. */
export function devStackControlSocket(env: NodeJS.ProcessEnv = process.env): string | undefined {
  const runBase = env.XDG_RUNTIME_DIR || env.TMPDIR || "/tmp";
  const dir = env.ROSIE_LOCAL_RT_DIR || path.join(runBase, `rosie-dev-rt-${userInfo().uid}`);
  const socket = path.join(dir, "control.sock");
  return existsSync(socket) ? socket : undefined;
}

/** The built-in local target: the documented local ports (offline-programming/v1/README.md "Ports, all of them"). */
export function builtinLocal(env: NodeJS.ProcessEnv = process.env): Target {
  return {
    olp_url: "http://127.0.0.1:8794",
    planner_url: "http://127.0.0.1:8796",
    dense_daemon: "127.0.0.1:8797",
    rt_control_socket: devStackControlSocket(env),
    robot_actions: "off",
    note: "the local stack (dev-stack.sh / start-offline-programming.sh)",
  };
}

export async function readProfile(env: NodeJS.ProcessEnv = process.env): Promise<Profile> {
  let stored: Partial<Profile> = {};
  try {
    stored = JSON.parse(await readFile(profilePath(env), "utf8")) as Partial<Profile>;
  } catch (error) {
    if ((error as NodeJS.ErrnoException).code !== "ENOENT") throw new Error(`${profilePath(env)} is not valid JSON: ${(error as Error).message}`);
  }
  return {
    default_target: stored.default_target || "local",
    targets: { local: builtinLocal(env), ...(stored.targets ?? {}) },
  };
}

export async function writeProfile(profile: Profile, env: NodeJS.ProcessEnv = process.env): Promise<void> {
  const file = profilePath(env);
  await mkdir(path.dirname(file), { recursive: true });
  // The built-in local target is recomputed every run (its socket comes and goes); only overrides are stored.
  const { local, ...rest } = profile.targets;
  const builtin = builtinLocal(env);
  const localOverrides = Object.fromEntries(Object.entries(local ?? {}).filter(([key, value]) => value !== undefined && builtin[key as keyof Target] !== value));
  const stored = { default_target: profile.default_target, targets: { ...(Object.keys(localOverrides).length ? { local: localOverrides } : {}), ...rest } };
  const temporary = `${file}.${process.pid}.tmp`;
  await writeFile(temporary, `${JSON.stringify(stored, null, 2)}\n`);
  await rename(temporary, file);
}

export interface Resolved {
  name: string;
  target: Target;
  config: Config;
  /** Which environment variables overrode the target (empty when the target was named). */
  overrides: string[];
}

const ENV_OF: Record<string, keyof Target> = {
  ROSIE_OLP_URL: "olp_url",
  ROSIE_PLANNER_URL: "planner_url",
  ROSIE_DENSE_DAEMON_ADDR: "dense_daemon",
  ROSIE_RT_CONTROL_SOCKET: "rt_control_socket",
  ROSIE_RT_CONTROL_URL: "rt_control_url",
  ROSIE_ROBOT_ACTIONS: "robot_actions",
};

export async function resolveTarget(named: string | undefined, env: NodeJS.ProcessEnv = process.env): Promise<Resolved> {
  const profile = await readProfile(env);
  const name = named || profile.default_target;
  const stored = profile.targets[name];
  if (!stored) throw new Error(`no target "${name}"; targets: ${Object.keys(profile.targets).join(", ")} (add one with \`rosie target add\`)`);
  const target: Target = name === "local" ? { ...builtinLocal(env), ...stored } : { ...stored };
  const overrides: string[] = [];
  if (!named) {
    for (const [variable, key] of Object.entries(ENV_OF)) {
      if (env[variable]) {
        (target as Record<string, unknown>)[key] = env[variable];
        overrides.push(variable);
      }
    }
  }
  const configEnv: NodeJS.ProcessEnv = {
    ROSIE_REPO_ROOT: env.ROSIE_REPO_ROOT,
    ROSIE_STATE_DIR: env.ROSIE_STATE_DIR || env.ROSIE_MCP_STATE_DIR,
    ROSIE_GO: env.ROSIE_GO,
    ROSIE_CARTESIAND_COMMAND: env.ROSIE_CARTESIAND_COMMAND,
    ROSIE_CARTESIAND_REPO_ROOT: env.ROSIE_CARTESIAND_REPO_ROOT,
    ROSIE_MCP_REQUEST_TIMEOUT_MS: env.ROSIE_REQUEST_TIMEOUT_MS,
    ROSIE_MCP_CLIENT_LABEL: env.ROSIE_CLIENT_LABEL || `rosie CLI (${userInfo().username})`,
    ROSIE_MCP_JOG_MAX_DEG: env.ROSIE_JOG_MAX_DEG,
    ROSIE_MCP_CARTESIAN_MAX_MM: env.ROSIE_CARTESIAN_MAX_MM,
    ROSIE_MCP_CARTESIAN_MAX_DEG: env.ROSIE_CARTESIAN_MAX_DEG,
    ROSIE_MCP_JOG_MAX_FRACTION: env.ROSIE_JOG_MAX_FRACTION,
    ROSIE_MCP_JOG_HOLD_MAX_S: env.ROSIE_JOG_HOLD_MAX_S,
    ROSIE_MCP_JOG_HOLD_MAX_FRACTION: env.ROSIE_JOG_HOLD_MAX_FRACTION,
    ROSIE_MCP_HEARTBEAT_WINDOW_S: target.heartbeat_window_s !== undefined ? String(target.heartbeat_window_s) : env.ROSIE_HEARTBEAT_WINDOW_S,
    ROSIE_OLP_URL: target.olp_url,
    ROSIE_PLANNER_URL: target.planner_url,
    ROSIE_DENSE_DAEMON_ADDR: target.dense_daemon,
    ROSIE_RT_CONTROL_SOCKET: target.rt_control_socket,
    ROSIE_RT_CONTROL_URL: target.rt_control_url,
    ROSIE_RT_CONTROL_CA: target.rt_control_ca,
    ROSIE_RT_CONTROL_CERT: target.rt_control_cert,
    ROSIE_RT_CONTROL_KEY: target.rt_control_key,
    ROSIE_ROBOT_ACTIONS: target.robot_actions,
  };
  for (const key of Object.keys(configEnv)) if (configEnv[key] === undefined || configEnv[key] === "") delete configEnv[key];
  return { name, target, overrides, config: loadConfig(configEnv, { variablePrefix: "ROSIE_" }) };
}

/**
 * One client id for every `rosie` process of this user (OLP control.go
 * validClientID: 8-64 of [A-Za-z0-9-_.:]): OLP's journal and control view
 * name the CLI as one client across commands, and a cell-store save's
 * idempotency key stays stable when a command is retried.
 */
export async function clientId(env: NodeJS.ProcessEnv = process.env): Promise<string> {
  const file = path.join(rosieHome(env), "cli", "client-id");
  try {
    const stored = (await readFile(file, "utf8")).trim();
    if (/^[A-Za-z0-9._:-]{8,64}$/.test(stored)) return stored;
  } catch {
    // first run
  }
  const id = `cli-${randomBytes(8).toString("hex")}`;
  await mkdir(path.dirname(file), { recursive: true });
  await writeFile(file, `${id}\n`);
  return id;
}
