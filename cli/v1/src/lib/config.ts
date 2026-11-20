// Every endpoint this server talks to, from the environment. Defaults are the
// documented local ports (README "Ports, all of them" in offline-programming/v1):
// OLP API 8794, weld planner motion 8796, dense daemon ingest 8797.
import { existsSync } from "node:fs";
import { homedir, hostname } from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";

/**
 * How far the robot-action tools may go. `off` registers only the tools that
 * end motion (Stop, Pause, Disarm, Finish cycle). `simulation` adds Arm, Load,
 * Play and Run program, refused unless the OLP's selected target describes
 * itself as rt-core backend `simulation`. `cell` lifts that refusal: an
 * explicit owner choice, never a default.
 */
export type RobotActionMode = "off" | "simulation" | "cell";

export interface RtControlConfig {
  /** Local public control socket (`/run/rosie-rt-core/control.sock` on a cell). */
  socketPath?: string;
  /** Remote mutual-TLS listener, e.g. `https://127.0.0.1:18443` through the cell tunnel. */
  url?: string;
  caFile?: string;
  certFile?: string;
  keyFile?: string;
}

/**
 * This server's own bounds on the jog tools, applied before any request
 * reaches OLP and in addition to OLP's and rt-core's limits. Travel is per
 * call: a joint move or joint hold may turn each joint at most
 * `maxJointDeg`, a Cartesian step or hold may move the TCP at most
 * `maxCartesianMm` / `maxCartesianDeg`.
 */
export interface JogLimits {
  maxJointDeg: number;
  maxCartesianMm: number;
  maxCartesianDeg: number;
  /** Largest speed fraction of a joint move or Cartesian step. */
  maxMoveFraction: number;
  /** Longest timed hold, and its largest speed fraction. */
  maxHoldS: number;
  maxHoldFraction: number;
}

export interface Config {
  repoRoot: string;
  olpUrl: string;
  plannerUrl: string;
  denseDaemonAddr: { host: string; port: number };
  rtControl: RtControlConfig;
  /** argv prefix that runs `robot-v4-cartesiand`; `--resolve-only <root> <model>` is appended. */
  cartesiandCommand: string[] | null;
  /** The repository root as the cartesiand process sees it (differs under WSL). */
  cartesiandRepoRoot: string;
  robotActions: RobotActionMode;
  /** ROSIE_MCP_ (the MCP server) or ROSIE_ (the CLI): the names refusals give for the settings below. */
  variablePrefix: "ROSIE_MCP_" | "ROSIE_";
  /** Longest this server keeps heartbeating a session after its last robot action. */
  heartbeatWindowS: number;
  jog: JogLimits;
  clientLabel: string;
  goCommand: string;
  requestTimeoutMs: number;
  /** Where imported STEP parts and planned trajectories are kept between
   *  sessions: this server's equivalent of the browser's IndexedDB stores. */
  stateDir: string;
}

const ROBOT_ACTION_MODES: readonly RobotActionMode[] = ["off", "simulation", "cell"];

function defaultRepoRoot(): string {
  // The checkout this module was built in: walk up from dist/src/lib (or an
  // npm link of it, which Node resolves to the real path) to the root that
  // holds CLAUDE.md and rt-core.
  let dir = path.dirname(fileURLToPath(import.meta.url));
  for (;;) {
    if (existsSync(path.join(dir, "CLAUDE.md")) && existsSync(path.join(dir, "rt-core"))) return dir;
    const parent = path.dirname(dir);
    if (parent === dir) return process.cwd();
    dir = parent;
  }
}

function parseAddr(raw: string, name: string): { host: string; port: number } {
  const match = /^\[?([^\]]+?)\]?:(\d{1,5})$/.exec(raw.trim());
  const port = match ? Number(match[2]) : NaN;
  if (!match || !(port > 0 && port < 65536)) {
    throw new Error(`${name} must be host:port, got ${JSON.stringify(raw)}`);
  }
  return { host: match[1]!, port };
}

function parseCommand(raw: string | undefined, fallback: string): string[] | null {
  if (raw && raw.trim().startsWith("[")) {
    const parsed: unknown = JSON.parse(raw);
    if (!Array.isArray(parsed) || parsed.length === 0 || !parsed.every((part) => typeof part === "string")) {
      throw new Error("ROSIE_CARTESIAND_COMMAND must be a JSON array of strings or a path");
    }
    return parsed as string[];
  }
  if (raw && raw.trim()) return [raw.trim()];
  return existsSync(fallback) ? [fallback] : null;
}

function bounded(raw: string | undefined, name: string, fallback: number, min: number, max: number): number {
  if (raw === undefined || raw.trim() === "") return fallback;
  const value = Number(raw);
  if (!Number.isFinite(value) || value < min || value > max) {
    throw new Error(`${name} must be a number in [${min}, ${max}], got ${JSON.stringify(raw)}`);
  }
  return value;
}

function trimSlash(url: string): string {
  return url.replace(/\/+$/, "");
}

export interface ConfigOptions {
  /** The front end's variable prefix: the MCP server reads ROSIE_MCP_ROBOT_ACTIONS,
   *  the CLI ROSIE_ROBOT_ACTIONS, so a mode set for one front end never widens
   *  the other; refusals name the variable of the front end that refused. */
  variablePrefix?: "ROSIE_MCP_" | "ROSIE_";
}

export function loadConfig(env: NodeJS.ProcessEnv = process.env, options: ConfigOptions = {}): Config {
  const repoRoot = path.resolve(env.ROSIE_REPO_ROOT || defaultRepoRoot());
  const variablePrefix = options.variablePrefix ?? "ROSIE_MCP_";
  const modeVariable = `${variablePrefix}ROBOT_ACTIONS`;
  const mode = (env[modeVariable] || "off").trim().toLowerCase();
  if (!ROBOT_ACTION_MODES.includes(mode as RobotActionMode)) {
    throw new Error(`${modeVariable} must be one of ${ROBOT_ACTION_MODES.join(", ")}, got ${JSON.stringify(mode)}`);
  }
  const rtControl: RtControlConfig = {};
  if (env.ROSIE_RT_CONTROL_SOCKET) rtControl.socketPath = env.ROSIE_RT_CONTROL_SOCKET;
  if (env.ROSIE_RT_CONTROL_URL) {
    rtControl.url = trimSlash(env.ROSIE_RT_CONTROL_URL);
    rtControl.caFile = env.ROSIE_RT_CONTROL_CA;
    rtControl.certFile = env.ROSIE_RT_CONTROL_CERT;
    rtControl.keyFile = env.ROSIE_RT_CONTROL_KEY;
  }
  return {
    repoRoot,
    olpUrl: trimSlash(env.ROSIE_OLP_URL || "http://127.0.0.1:8794"),
    plannerUrl: trimSlash(env.ROSIE_PLANNER_URL || "http://127.0.0.1:8796"),
    denseDaemonAddr: parseAddr(env.ROSIE_DENSE_DAEMON_ADDR || "127.0.0.1:8797", "ROSIE_DENSE_DAEMON_ADDR"),
    rtControl,
    cartesiandCommand: parseCommand(
      env.ROSIE_CARTESIAND_COMMAND,
      path.join(repoRoot, "motion-server", "v1", "bin", "robot-v4-cartesiand"),
    ),
    cartesiandRepoRoot: env.ROSIE_CARTESIAND_REPO_ROOT || repoRoot,
    robotActions: mode as RobotActionMode,
    variablePrefix,
    // Unverified window: two minutes covers a taught program and a loop cycle
    // watched from a chat; past it the OLP's own 5 s deadman stops the robot.
    heartbeatWindowS: bounded(env.ROSIE_MCP_HEARTBEAT_WINDOW_S, "ROSIE_MCP_HEARTBEAT_WINDOW_S", 120, 5, 3600),
    // Unverified per-call bounds for an agent with no hand on the trigger:
    // 30° per joint, and the browser's default Cartesian step (100 mm, 15°,
    // CARTESIAN_STEP_DEFAULTS) as the most one step or hold may travel. The
    // upper ends are OLP's own input bounds (1000 mm, 180°). A hold lasts at
    // most 3 s at a quarter of each joint's maximum.
    jog: {
      maxJointDeg: bounded(env.ROSIE_MCP_JOG_MAX_DEG, "ROSIE_MCP_JOG_MAX_DEG", 30, 0.1, 180),
      maxCartesianMm: bounded(env.ROSIE_MCP_CARTESIAN_MAX_MM, "ROSIE_MCP_CARTESIAN_MAX_MM", 100, 0.1, 1000),
      maxCartesianDeg: bounded(env.ROSIE_MCP_CARTESIAN_MAX_DEG, "ROSIE_MCP_CARTESIAN_MAX_DEG", 15, 0.1, 180),
      maxMoveFraction: bounded(env.ROSIE_MCP_JOG_MAX_FRACTION, "ROSIE_MCP_JOG_MAX_FRACTION", 0.5, 0.01, 1),
      maxHoldS: bounded(env.ROSIE_MCP_JOG_HOLD_MAX_S, "ROSIE_MCP_JOG_HOLD_MAX_S", 3, 0.1, 10),
      maxHoldFraction: bounded(env.ROSIE_MCP_JOG_HOLD_MAX_FRACTION, "ROSIE_MCP_JOG_HOLD_MAX_FRACTION", 0.25, 0.01, 1),
    },
    clientLabel: (env.ROSIE_MCP_CLIENT_LABEL || `rosieos-mcp-server (${hostname()})`).slice(0, 80),
    goCommand: env.ROSIE_GO || "go",
    requestTimeoutMs: bounded(env.ROSIE_MCP_REQUEST_TIMEOUT_MS, "ROSIE_MCP_REQUEST_TIMEOUT_MS", 15_000, 500, 600_000),
    // One store for the CLI and the MCP server: a STEP either imported is kept for both.
    stateDir: path.resolve(env.ROSIE_STATE_DIR || env.ROSIE_MCP_STATE_DIR || path.join(homedir(), ".rosie", "mcp")),
  };
}
