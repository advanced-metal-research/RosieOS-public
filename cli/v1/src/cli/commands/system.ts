// doctor, status and target: what is up, who has the robot, where rosie points.
import { existsSync } from "node:fs";
import { readdir, stat } from "node:fs/promises";
import path from "node:path";

import type { Context } from "../../lib/context.js";
import { fetchJSON, refusalText, ServiceUnreachable } from "../../lib/http.js";
import { ingestExchange } from "../../lib/ingest.js";
import { DENSE, OLP_API, type DenseStatus } from "../../lib/olp.js";
import { describeStatus } from "../../lib/status.js";
import { errorMessage, record, table } from "../../lib/text.js";
import { ok, refused, type Command, type Outcome } from "../command.js";
import type { Env } from "../env.js";
import { defaultSnapshotDir, findChrome } from "../../lib/snapshot/index.js";
import { cliRoot } from "../paths.js";
import { builtinLocal, profilePath, readProfile, writeProfile, type Target } from "../targets.js";
import { keepaliveState } from "../keepalive.js";

type Level = "pass" | "warn" | "fail" | "skip";
interface Check { check: string; level: Level; detail: string; fix?: string }

const MARK: Record<Level, string> = { pass: "ok  ", warn: "WARN", fail: "FAIL", skip: "--  " };

async function newestMtime(dir: string, depth = 6): Promise<number> {
  let newest = 0;
  let entries: import("node:fs").Dirent[];
  try {
    entries = await readdir(dir, { withFileTypes: true });
  } catch {
    return 0;
  }
  for (const entry of entries) {
    if (entry.name === "node_modules" || entry.name.startsWith(".")) continue;
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) {
      if (depth > 0) newest = Math.max(newest, await newestMtime(full, depth - 1));
    } else if (/\.(ts|mjs|json)$/.test(entry.name) && !entry.name.endsWith(".test.ts")) {
      newest = Math.max(newest, (await stat(full)).mtimeMs);
    }
  }
  return newest;
}

async function engineCheck(repoRoot: string): Promise<Check> {
  const bundle = path.join(cliRoot(), "dist", "core", "olp-core.mjs");
  if (!existsSync(bundle)) return { check: "engine bundle", level: "fail", detail: `${bundle} missing`, fix: "(cd offline-programming/v1/ui && npm ci) && (cd cli/v1 && npm ci && npm run build)" };
  const built = (await stat(bundle)).mtimeMs;
  const sources = Math.max(
    await newestMtime(path.join(repoRoot, "offline-programming", "v1", "ui", "src")),
    await newestMtime(path.join(repoRoot, "steamdeck", "real", "v5", "olp-core")),
    await newestMtime(path.join(cliRoot(), "core")),
  );
  return sources > built
    ? { check: "engine bundle", level: "warn", detail: "OLP sources changed since the bundle was built: rosie may run old OLP rules", fix: "(cd cli/v1 && npm run build)" }
    : { check: "engine bundle", level: "pass", detail: "built from the current OLP sources" };
}

async function olpChecks(ctx: Context, local: boolean): Promise<{ checks: Check[]; status: DenseStatus | null }> {
  const checks: Check[] = [];
  const health = await ctx.olp.get(`${OLP_API}/health`).catch((error: unknown) => error);
  if (health instanceof ServiceUnreachable || health instanceof Error) {
    checks.push({ check: "OLP API", level: "fail", detail: errorMessage(health), fix: local ? "bash offline-programming/v1/start-offline-programming.sh (or ./dev-stack.sh start)" : "rosie target show; for a cell from the Mac: go run . olp ui start --manifest <cell> (steamdeck/real/v4)" });
    return { checks, status: null };
  }
  const h = record((health as { body: unknown }).body);
  checks.push({ check: "OLP API", level: "pass", detail: `${ctx.olp.baseUrl} · build ${String(h.build_id ?? "?")}${record(h.maintenance).active ? " · MAINTENANCE NOTICE ACTIVE" : ""}` });
  if (record(h.maintenance).active) checks.push({ check: "maintenance", level: "warn", detail: "a deploy/build/test holds the cell: arming is refused from every client", fix: "rosie status (reason, ETA)" });
  checks.push(record(h.cell_programs).enabled
    ? { check: "cell program store", level: "pass", detail: "enabled (save/open on the cell)" }
    : { check: "cell program store", level: "warn", detail: "this OLP has no --program-store: save programs to a file", fix: "rosie program save --file <path>" });
  const capabilities = await ctx.olp.get(`${OLP_API}/capabilities`).catch(() => null);
  const seamWired = record(record(capabilities?.body).weld_planner).enabled === true;
  checks.push(seamWired
    ? { check: "seam worker", level: "pass", detail: "wired in OLP (rails seam detection, search, welds)" }
    : { check: "seam worker", level: "fail", detail: "OLP reports no seam worker: rails authoring and every plan are refused", fix: "start OLP with the weld planner checkout (start-offline-programming.sh)" });
  if (local) {
    const env = path.join(ctx.config.repoRoot, "weld_planner", "v1", ".pixi", "envs", "default");
    checks.push(existsSync(env)
      ? { check: "seam worker env", level: "pass", detail: "weld_planner/v1 pixi default env installed" }
      : { check: "seam worker env", level: "fail", detail: "weld_planner/v1 has no pixi default env: OLP health says ok but every rails action fails", fix: "cd weld_planner/v1 && pixi install -e default" });
  }
  // A body that is not an object: OLP checks the motion origin before it parses, so this plans nothing.
  const probe = await ctx.olp.post(`${OLP_API}/weld-plan`, "probe").catch(() => null);
  const code = String(record(probe?.body).code ?? "");
  checks.push(code === "motion_origin_unavailable"
    ? { check: "planning origin", level: "fail", detail: "OLP has no weld planner origin: every plan is refused", fix: "export OFFLINE_PROGRAMMING_WELD_PLANNER_MOTION_ORIGIN=http://127.0.0.1:8796 and restart OLP" }
    : code === "seam_worker_unavailable"
      ? { check: "planning origin", level: "skip", detail: "not reachable before the seam worker" }
      : { check: "planning origin", level: "pass", detail: `configured (probe answered ${probe?.status ?? "?"} ${code || ""})`.trim() });
  let status: DenseStatus | null = null;
  try {
    status = await ctx.olp.status();
    const t = status.target;
    checks.push(t
      ? { check: "selected cell", level: "pass", detail: `${t.cell_id} (${t.model_id ?? "?"}) · ${t.simulation ? "SIMULATION" : t.simulation === false ? "REAL ROBOT" : "backend " + (t.backend ?? "?")}${status.armed ? " · ARMED" : ""}` }
      : { check: "selected cell", level: "warn", detail: "no cell selected: plans are offline (no measured start), robot actions refused", fix: "rosie cell list; rosie robot select <cell> (robot actions on)" });
  } catch (error) {
    checks.push({ check: "selected cell", level: "warn", detail: `dense-execution/status: ${errorMessage(error)}` });
  }
  return { checks, status };
}

async function plannerCheck(ctx: Context): Promise<Check> {
  try {
    const reply = await fetchJSON("weld planner", `${ctx.config.plannerUrl}/api/motion/health`, { timeoutMs: 5000 });
    if (!reply.ok) return { check: "weld planner", level: "fail", detail: `HTTP ${reply.status}: ${refusalText(reply)}` };
    const h = record(reply.body);
    const cuda = h.cuda === true || record(h.cuda).available === true || (typeof h.device === "string" && /cuda/i.test(h.device));
    return cuda
      ? { check: "weld planner", level: "pass", detail: `${ctx.config.plannerUrl} · CUDA ${String(h.device ?? "")}`.trim() }
      : { check: "weld planner", level: "warn", detail: `${ctx.config.plannerUrl} · no CUDA: plans MoveJ, dwell and Home only; welds and MoveL are refused`, fix: "plan welds on a CUDA planner (pixi run -e motion motion-serve on a GPU host)" };
  } catch (error) {
    return { check: "weld planner", level: "fail", detail: errorMessage(error), fix: "cd weld_planner/v1 && pixi run -e motion motion-serve" };
  }
}

async function rtCheck(ctx: Context): Promise<Check> {
  if (!ctx.rt.configured) return { check: "rt-control", level: "skip", detail: "not configured for this target (rt-core reads come through OLP's status)", fix: "local: ./dev-stack.sh start (socket found automatically); else rosie target add --rt-control-socket" };
  try {
    const reply = await ctx.rt.get("/v1/describe");
    if (!reply.ok) return { check: "rt-control", level: "fail", detail: refusalText(reply) };
    const d = record(reply.body);
    return { check: "rt-control", level: "pass", detail: `${ctx.rt.target} · backend ${String(d.backend)} · configuration ${String(d.configuration_sha256 ?? "").slice(0, 12)}…` };
  } catch (error) {
    return { check: "rt-control", level: "fail", detail: errorMessage(error) };
  }
}

async function daemonCheck(ctx: Context): Promise<Check> {
  try {
    await ingestExchange(ctx.config.denseDaemonAddr, "status", undefined, 3000);
    return { check: "dense daemon", level: "pass", detail: `${ctx.config.denseDaemonAddr.host}:${ctx.config.denseDaemonAddr.port}` };
  } catch (error) {
    return { check: "dense daemon", level: "skip", detail: `not reachable (${errorMessage(error).slice(0, 80)}); only needed for daemon validate/status` };
  }
}

/** Are the rosie skills in the user's Claude Code skills directory (so sessions outside this repo find them)? */
async function skillsCheck(env: Env): Promise<Check> {
  const { commands } = await import("./skills.js");
  const listed = await commands.find((c) => c.path[1] === "list")!.run(env, {});
  const skills = ((listed.data as { skills?: { state: string }[] } | undefined)?.skills ?? []);
  const installed = skills.filter((s) => s.state === "linked" || s.state === "copied").length;
  return installed === skills.length && skills.length
    ? { check: "agent skills", level: "pass", detail: listed.verdict }
    : { check: "agent skills", level: "warn", detail: `${listed.verdict}: Claude Code sessions outside this repo won't find them`, fix: "rosie skills install" };
}

function toolChecks(ctx: Context): Check[] {
  const checks: Check[] = [];
  checks.push(ctx.config.cartesiandCommand
    ? { check: "cartesiand", level: "pass", detail: ctx.config.cartesiandCommand.join(" ") }
    : { check: "cartesiand", level: "skip", detail: "robot-v4-cartesiand not built: `rosie kin` unavailable", fix: "make -C motion-server/v1" });
  const chrome = findChrome();
  checks.push(chrome
    ? { check: "chrome (3D snapshots)", level: "pass", detail: chrome }
    : { check: "chrome (3D snapshots)", level: "warn", detail: "no Chrome or Edge found: rosie view scene cannot render", fix: "install Google Chrome or set ROSIE_CHROME" });
  checks.push(existsSync(path.join(defaultSnapshotDir(), "snapshot.html"))
    ? { check: "snapshot page", level: "pass", detail: defaultSnapshotDir() }
    : { check: "snapshot page", level: "warn", detail: "OLP's snapshot page is not built: rosie view scene cannot render", fix: "(cd cli/v1 && npm run build:snapshot)" });
  return checks;
}

const doctor: Command<Env> = {
  path: ["doctor"],
  summary: "Check every service and the known setup traps in one report",
  description: `Probes the selected target: the engine bundle, OLP (health, program store, seam worker and its pixi env, the planning origin — the "health ok but rails/planning fails" traps), the selected cell (SIMULATION or REAL ROBOT), the weld planner (CUDA or CPU-only), rt-control, the dense daemon, cartesiand and the 3D snapshot renderer. Each failing check names its fix. Moves nothing.`,
  examples: ["rosie doctor", "rosie doctor --target my-cell --json"],
  async run(env) {
    const resolved = await env.resolved();
    const ctx = await env.ctx();
    const local = resolved.name === "local";
    const [engine, olp, planner, rt, daemon] = await Promise.all([engineCheck(ctx.config.repoRoot), olpChecks(ctx, local), plannerCheck(ctx), rtCheck(ctx), daemonCheck(ctx)]);
    const checks: Check[] = [
      { check: "target", level: "pass", detail: `${resolved.name} · robot actions ${ctx.config.robotActions}${resolved.overrides.length ? ` · env overrides ${resolved.overrides.join(", ")}` : ""}` },
      engine, ...olp.checks, planner, rt, daemon, ...toolChecks(ctx), await skillsCheck(env),
    ];
    const fails = checks.filter((c) => c.level === "fail");
    const warns = checks.filter((c) => c.level === "warn");
    const body = [
      table(["", "Check", "Detail"], checks.map((c) => [MARK[c.level], c.check, c.detail])),
      ...(fails.length || warns.length ? ["", "fixes:", ...[...fails, ...warns].filter((c) => c.fix).map((c) => `  ${c.check}: ${c.fix}`)] : []),
    ].join("\n");
    const verdict = fails.length ? `${fails.length} failing, ${warns.length} warning: ${fails.map((c) => c.check).join(", ")}` : `ready${warns.length ? ` (${warns.length} warning${warns.length > 1 ? "s" : ""}: ${warns.map((c) => c.check).join(", ")})` : ""}`;
    return (fails.length ? refused : ok)(verdict, { body, data: { target: resolved.name, checks }, next: fails.length ? [] : ["rosie status", "rosie program --help"] });
  },
};

const status: Command<Env> = {
  path: ["status"],
  summary: "Who has the robot, SIMULATION or REAL, armed, session, maintenance — one screen",
  description: "OLP's control view and maintenance notice, the selected cell's machine status (session, cursor, loop, axes, drive alarms), this CLI's heartbeat keepalive, and rt-core's own status when rt-control is configured. Read it before any robot action. Moves nothing.",
  examples: ["rosie status", "rosie status --json"],
  async run(env) {
    const ctx = await env.ctx();
    const [control, maintenance, machine] = await Promise.all([
      ctx.olp.get(`${OLP_API}/control`).catch((e: unknown) => e),
      ctx.olp.get(`${OLP_API}/maintenance`).catch((e: unknown) => e),
      ctx.olp.status(),
    ]);
    const c = record((control as { body?: unknown })?.body);
    const m = record((maintenance as { body?: unknown })?.body);
    const owner = record(c.owner);
    let rt: Record<string, unknown> | null = null;
    if (ctx.rt.configured) {
      const reply = await ctx.rt.get("/v1/status").catch(() => null);
      rt = reply?.ok ? record(reply.body) : null;
    }
    const core = record(rt?.core);
    const faults = Array.isArray(record(core.recovery).faults) ? (record(core.recovery).faults as unknown[]).map(record) : [];
    const keep = await keepaliveState(env.home);
    const t = machine.target;
    const mode = t ? (t.simulation ? "SIMULATION" : t.simulation === false ? "REAL ROBOT" : `backend ${t.backend ?? "?"}`) : "no cell";
    const body = [
      `- **Control**: ${String(c.state ?? "?")}${owner.label ? ` — ${String(owner.label)} (${String(owner.address ?? "")}), last action ${String(owner.last_action ?? "?")}` : ""} · this CLI is \`${ctx.olp.clientId}\``,
      `- **Maintenance**: ${m.active ? `ACTIVE — ${String(m.reason ?? "")} (${String(m.phase ?? "")}, ETA ${String(m.eta ?? "?")})` : "none"}`,
      describeStatus(machine),
      rt ? `- **rt-core**: armed ${String(core.armed)} · latched faults ${faults.length ? faults.map((f) => `${String(f.name)} → ${String(f.recovery)}`).join("; ") : "none"}` : "",
      `- **Keepalive**: ${keep.running ? `heartbeating session ${keep.session ?? "?"} until ${keep.until ?? "?"}` : "none"}`,
    ].filter(Boolean).join("\n");
    const verdict = `${t ? `${t.cell_id} · ${mode}` : "no cell selected"} · ${machine.armed ? "ARMED" : "disarmed"} · control ${String(c.state ?? "?")}${m.active ? " · MAINTENANCE" : ""}${machine.session ? ` · session ${machine.session.kind}/${machine.session.state}` : ""}`;
    return ok(verdict, { body, data: { control: c, maintenance: m, machine, rt_core: rt, keepalive: keep } });
  },
};

const TARGET_KEYS: ReadonlyArray<keyof Target> = ["olp_url", "planner_url", "dense_daemon", "rt_control_socket", "rt_control_url", "rt_control_ca", "rt_control_cert", "rt_control_key", "robot_actions", "heartbeat_window_s", "note"];

const targetList: Command<Env> = {
  path: ["target", "list"],
  summary: "List the named targets (local stack, cell tunnels) and the default",
  examples: ["rosie target list"],
  async run(env) {
    const profile = await readProfile(env.processEnv);
    const rows = Object.entries(profile.targets).map(([name, t]) => [name === profile.default_target ? `${name} *` : name, t.olp_url ?? "", t.planner_url ?? "", t.robot_actions ?? "off", t.note ?? ""]);
    return ok(`${rows.length} target(s); default ${profile.default_target}`, { body: `${table(["Target", "OLP", "Planner", "Robot actions", "Note"], rows)}\n\nprofile: ${profilePath(env.processEnv)}`, data: profile });
  },
};

const targetShow: Command<Env> = {
  path: ["target", "show"],
  summary: "Show the resolved target: every endpoint and the robot-action mode in effect",
  options: [{ name: "name", type: "string", description: "Target (default: the --target or default one)", positional: 0 }],
  examples: ["rosie target show", "rosie target show my-cell"],
  async run(env, args) {
    if (args.name) env.globals.target = String(args.name);
    const resolved = await env.resolved();
    const c = resolved.config;
    const data = {
      name: resolved.name, olp_url: c.olpUrl, planner_url: c.plannerUrl, dense_daemon: `${c.denseDaemonAddr.host}:${c.denseDaemonAddr.port}`,
      rt_control: c.rtControl.socketPath ?? c.rtControl.url ?? null, robot_actions: c.robotActions, heartbeat_window_s: c.heartbeatWindowS,
      jog_limits: c.jog, state_dir: c.stateDir, repo_root: c.repoRoot, env_overrides: resolved.overrides,
    };
    return ok(`${resolved.name}: OLP ${c.olpUrl} · robot actions ${c.robotActions}`, { body: table(["Key", "Value"], Object.entries(data).map(([k, v]) => [k, typeof v === "object" ? JSON.stringify(v) : v])), data });
  },
};

const targetAdd: Command<Env> = {
  path: ["target", "add"],
  summary: "Add or update a named target",
  description: `Stored in ~/.rosie/rosie.json. A cell's OLP from the Mac is the \`olp ui start\` tunnel (http://127.0.0.1:18894). robot_actions 'cell' lets motion commands act on REAL ROBOTS: it is the owner's decision and needs --owner-approved; never set it to get past a refusal.`,
  options: [
    { name: "name", type: "string", description: "Target name, for example my-cell", positional: 0, required: true },
    { name: "olp_url", type: "string", description: "OLP API URL" },
    { name: "planner_url", type: "string", description: "Weld planner URL" },
    { name: "dense_daemon", type: "string", description: "Dense daemon host:port" },
    { name: "rt_control_socket", type: "string", description: "rt-control public socket path" },
    { name: "rt_control_url", type: "string", description: "rt-control mTLS URL (with --rt-control-ca/-cert/-key)" },
    { name: "rt_control_ca", type: "string", description: "CA file" },
    { name: "rt_control_cert", type: "string", description: "client certificate" },
    { name: "rt_control_key", type: "string", description: "client key" },
    { name: "robot_actions", type: "string", description: "off | simulation | cell", choices: ["off", "simulation", "cell"] },
    { name: "heartbeat_window_s", type: "number", description: "Keepalive window after a robot action (5-3600 s)" },
    { name: "note", type: "string", description: "Free text shown in target list" },
    { name: "owner_approved", type: "boolean", description: "Required with --robot-actions cell (the owner's decision)" },
    { name: "default", type: "boolean", description: "Make it the default target" },
  ],
  examples: ["rosie target add my-cell --olp-url http://127.0.0.1:18894 --note 'my-cell via olp ui tunnel'", "rosie target add local --robot-actions simulation"],
  async run(env, args) {
    if (args.robot_actions === "cell" && !args.owner_approved) {
      return refused("robot_actions cell lets rosie move REAL ROBOTS: only the owner sets it, with --owner-approved");
    }
    const profile = await readProfile(env.processEnv);
    const name = String(args.name);
    const current: Target = name === "local" ? {} : { ...(profile.targets[name] ?? {}) };
    if (name === "local") {
      const builtin = builtinLocal(env.processEnv);
      for (const key of TARGET_KEYS) if (profile.targets.local?.[key] !== builtin[key]) (current as Record<string, unknown>)[key] = profile.targets.local?.[key];
    }
    for (const key of TARGET_KEYS) if (args[key] !== undefined) (current as Record<string, unknown>)[key] = args[key];
    profile.targets[name] = name === "local" ? { ...builtinLocal(env.processEnv), ...current } : current;
    if (args.default) profile.default_target = name;
    await writeProfile(profile, env.processEnv);
    return ok(`target ${name} saved${args.default ? " (default)" : ""}`, { data: profile.targets[name], next: [`rosie doctor --target ${name}`] });
  },
};

const targetUse: Command<Env> = {
  path: ["target", "use"],
  summary: "Make a target the default",
  options: [{ name: "name", type: "string", description: "Target name", positional: 0, required: true }],
  examples: ["rosie target use my-cell", "rosie target use local"],
  async run(env, args) {
    const profile = await readProfile(env.processEnv);
    const name = String(args.name);
    if (!profile.targets[name]) return refused(`no target ${name}; targets: ${Object.keys(profile.targets).join(", ")}`);
    profile.default_target = name;
    await writeProfile(profile, env.processEnv);
    return ok(`default target is ${name}`, { next: ["rosie doctor"] });
  },
};

const targetRemove: Command<Env> = {
  path: ["target", "remove"],
  summary: "Remove a named target",
  options: [{ name: "name", type: "string", description: "Target name", positional: 0, required: true }],
  async run(env, args): Promise<Outcome> {
    const profile = await readProfile(env.processEnv);
    const name = String(args.name);
    if (name === "local") return refused("the local target is built in; override its values with rosie target add local");
    if (!profile.targets[name]) return refused(`no target ${name}`);
    delete profile.targets[name];
    if (profile.default_target === name) profile.default_target = "local";
    await writeProfile(profile, env.processEnv);
    return ok(`removed target ${name}`);
  },
};

export const commands: Command<Env>[] = [doctor, status, targetList, targetShow, targetAdd, targetUse, targetRemove];
