// cell, docs, deploy: read-only answers about cells, from this checkout,
// OLP, rt-control and the deploy module. Nothing here acts on a robot.
import { execFile } from "node:child_process";
import { existsSync } from "node:fs";
import { readFile } from "node:fs/promises";
import path from "node:path";

import type { Context } from "../../lib/context.js";
import { refusalText } from "../../lib/http.js";
import { DENSE, OLP_API, type DenseStatus } from "../../lib/olp.js";
import { cellSummary, DOCS, docsFor, listCellNames, lookupAlarm, readCell, resolveCellName, searchDocs, type AlarmMatch } from "../../lib/repo.js";
import { describeStatus } from "../../lib/status.js";
import { deg, errorMessage, json, maskAxes, record, table, yesNo } from "../../lib/text.js";
import { ok, refused, type Command } from "../command.js";
import type { Env } from "../env.js";

function run(command: string, args: string[], cwd: string, timeoutMs: number): Promise<{ code: number | null; stdout: string; stderr: string; timedOut: boolean }> {
  return new Promise((resolve) => {
    execFile(command, args, { cwd, timeout: timeoutMs, maxBuffer: 32 * 1024 * 1024 }, (error, stdout, stderr) => {
      const failed = error as (NodeJS.ErrnoException & { killed?: boolean; code?: number | string }) | null;
      if (failed && typeof failed.code === "string") {
        resolve({ code: null, stdout, stderr: `${stderr}\ncould not run ${command}: ${failed.message}`, timedOut: false });
        return;
      }
      resolve({ code: failed ? (typeof failed.code === "number" ? failed.code : 1) : 0, stdout, stderr, timedOut: Boolean(failed?.killed) });
    });
  });
}

function alarmRows(matches: AlarmMatch[]): string {
  return table(
    ["Code", "Name", "Manual: resettable", "Arm/Reset clears it", "0x603F"],
    matches.map((m) => [m.code ?? m.key, m.name, yesNo(m.resettable), m.rt_core_profile_resettable === null ? "n/a" : yesNo(m.rt_core_profile_resettable), m.bus_fault_code_603f ?? ""]),
  );
}

const cellList: Command<Env> = {
  path: ["cell", "list"],
  summary: "The committed cell manifests (this checkout) and the cells OLP can select",
  examples: ["rosie cell list"],
  async run(env) {
    const root = await env.repoRoot();
    const names = await listCellNames(root);
    const cells = await Promise.all(names.map(async (name) => cellSummary(name, await readCell(root, name))));
    let olpCells: Record<string, unknown> | null = null;
    let olpError = "";
    try {
      const ctx = await env.ctx();
      olpCells = record(await ctx.olp.getOk(`${DENSE}/cells`, "cells"));
    } catch (error) {
      olpError = errorMessage(error);
    }
    const selectable = (Array.isArray(olpCells?.cells) ? (olpCells!.cells as unknown[]) : []).map(record);
    const body = [
      "committed manifests (rt-core/config/cells):",
      table(["Cell", "Servo host", "Pin", "OLP", "Unattended loops"], cells.map((c) => [c.file, c.servo_host ?? "", c.configuration_sha256 ? `${String(c.configuration_sha256).slice(0, 12)}…` : "", c.olp_listen ?? "", yesNo(c.unattended_loop_allowed)])),
      "",
      olpCells
        ? `selectable in OLP (active: ${String(olpCells.active_cell ?? olpCells.active ?? "none")}):\n${selectable.length ? table(["Cell", "Label", "Model", "Host", "Backend"], selectable.map((x) => [x.cell_id ?? x.id, x.label, x.model_id, x.host, x.backend])) : "none"}`
        : `OLP cells unavailable: ${olpError}`,
    ].join("\n");
    return ok(`${cells.length} committed cells; ${olpCells ? `${selectable.length} selectable in OLP` : "OLP not reachable"}`, { body, data: { committed: cells, olp: olpCells } });
  },
};

const cellShow: Command<Env> = {
  path: ["cell", "show"],
  summary: "One committed manifest: summary, or a section in full",
  options: [
    { name: "cell", type: "string", description: "Manifest name or short robot name (for example example-arm-1420)", positional: 0, required: true },
    { name: "section", type: "string", description: "Top-level key: cell_deploy, nodes, robot_axes, connections, pendant_deploy, ...", positional: 1 },
  ],
  examples: ["rosie cell show example-arm-1420", "rosie cell show example-arm-1420 cell_deploy"],
  async run(env, args) {
    const root = await env.repoRoot();
    const name = await resolveCellName(root, String(args.cell));
    const manifest = await readCell(root, name);
    if (args.section) {
      const section = String(args.section);
      if (!(section in manifest)) return refused(`${name} has no section ${section}; sections: ${Object.keys(manifest).join(", ")}`);
      return ok(`${name} · ${section}`, { body: json(manifest[section]), data: manifest[section] });
    }
    const summary = cellSummary(name, manifest);
    return ok(`${name}: servo host ${String(summary.servo_host)}, pin ${String(summary.configuration_sha256 ?? "").slice(0, 12)}…`, {
      body: `${table(["Key", "Value"], Object.entries(summary).map(([k, v]) => [k, v]))}\n\nsections: ${Object.keys(manifest).join(", ")}`,
      data: { ...summary, sections: Object.keys(manifest) },
    });
  },
};

const alarm: Command<Env> = {
  path: ["cell", "alarm"],
  summary: "Explain an A6-EC drive code and whether Arm/Reset may clear it",
  description: "From the vendor manual table and rt-core's drive profile (fault_reset). A code the profile does not list is not resettable by software: fix the drive or hardware; Home is never re-run to clear a fault.",
  options: [{ name: "query", type: "string", description: "Er41.1, ALF1.3, 0x3230 or words from the name", positional: 0, required: true }],
  examples: ["rosie cell alarm Er41.1", "rosie cell alarm over-temperature"],
  async run(env, args) {
    const matches = await lookupAlarm(await env.repoRoot(), String(args.query));
    if (!matches.length) return refused(`no A6-EC code or name matches "${String(args.query)}" (codes look like Er41.1, ALF1.3, 0x3230)`);
    const first = matches[0]!;
    const verdict = first.exact
      ? `${first.code ?? first.key} ${first.name}: ${first.rt_core_profile_resettable ? "Arm resets it (drive profile lists it)" : first.rt_core_profile_resettable === false ? "NOT resettable by software — the owner's call; fix at the drive" : "bus fault code"}`
      : `${matches.length} codes match "${String(args.query)}"`;
    return ok(verdict, { body: alarmRows(matches.slice(0, 25)), data: { matches: matches.slice(0, 25), total: matches.length } });
  },
};

const events: Command<Env> = {
  path: ["cell", "events"],
  summary: "rt-core's event log (grant, faults, jog ends, Home epochs), oldest first",
  options: [
    { name: "after", type: "number", description: "Events after this sequence", default: 0 },
    { name: "limit", type: "number", description: "At most this many (1-200)", default: 50 },
  ],
  examples: ["rosie cell events", "rosie cell events --after 1200 --limit 20"],
  async run(env, args) {
    const ctx = await env.ctx();
    if (!ctx.rt.configured) return refused("rt-control is not configured for this target", { next: ["rosie target show", "rosie doctor"] });
    const reply = await ctx.rt.get(`/v1/events?after=${Number(args.after)}`);
    if (!reply.ok) return refused(`rt-control refused events: ${refusalText(reply)}`);
    const batch = record(reply.body);
    const list = (Array.isArray(batch.events) ? batch.events : []).map(record).slice(0, Math.min(200, Math.max(1, Number(args.limit))));
    const last = list.at(-1);
    const detail = (e: Record<string, unknown>) => JSON.stringify(e[String(e.type)] ?? Object.fromEntries(Object.entries(e).filter(([k]) => !["sequence", "time_ns", "type", "daemon_incarnation", "adapter_incarnation"].includes(k)))).slice(0, 200);
    const dropped = list.some((e) => e.type === "events_dropped");
    return ok(`${list.length} events after ${Number(args.after)}${last ? `; continue with --after ${String(last.sequence)}` : ""}${dropped ? " · EVENTS DROPPED: reconcile with rosie status" : ""}`, {
      body: list.length ? table(["Seq", "Type", "Detail"], list.map((e) => [e.sequence, e.type, detail(e)])) : "",
      data: { events: list, next_sequence: last ? Number(last.sequence) : batch.next_sequence ?? Number(args.after), latest_sequence: batch.latest_sequence ?? null },
    });
  },
};

const describe: Command<Env> = {
  path: ["cell", "describe"],
  summary: "rt-core Describe: backend, configuration pin, axis limits (compare with the manifest)",
  examples: ["rosie cell describe"],
  async run(env) {
    const ctx = await env.ctx();
    if (!ctx.rt.configured) return refused("rt-control is not configured for this target", { next: ["rosie target show"] });
    const reply = await ctx.rt.get("/v1/describe");
    if (!reply.ok) return refused(`rt-control refused describe: ${refusalText(reply)}`);
    const d = record(reply.body);
    const axes = (Array.isArray(d.axes) ? d.axes : []).map(record);
    const rad = (unit: unknown) => typeof unit === "string" && /rad/.test(unit);
    return ok(`backend ${String(d.backend)} · configuration ${String(d.configuration_sha256)}`, {
      body: table(["Axis", "Min", "Max", "Max velocity", "Home required"], axes.map((a) => [a.id, rad(a.position_unit) ? deg(Number(a.min_position)) : a.min_position, rad(a.position_unit) ? deg(Number(a.max_position)) : a.max_position, rad(a.position_unit) ? `${deg(Number(a.max_velocity))}/s` : a.max_velocity, yesNo(a.require_home)])),
      data: d,
    });
  },
};

async function diagnoseRead(ctx: Context) {
  const [control, maintenance, machine] = await Promise.all([
    ctx.olp.get(`${OLP_API}/control`).then((r) => (r.ok ? record(r.body) : null)).catch(() => null),
    ctx.olp.get(`${OLP_API}/maintenance`).then((r) => (r.ok ? record(r.body) : null)).catch(() => null),
    ctx.olp.status().catch((error: unknown) => error as Error),
  ]);
  let rt: Record<string, unknown> | null = null;
  let rtEvents: Record<string, unknown>[] = [];
  if (ctx.rt.configured) {
    rt = await ctx.rt.get("/v1/status").then((r) => (r.ok ? record(r.body) : null)).catch(() => null);
    const latest = await ctx.rt.get("/v1/events?after=0").then((r) => (r.ok ? record(r.body) : null)).catch(() => null);
    rtEvents = (Array.isArray(latest?.events) ? (latest!.events as unknown[]) : []).map(record).slice(-15);
  }
  return { control, maintenance, machine, rt, rtEvents };
}

const diagnose: Command<Env> = {
  path: ["cell", "diagnose"],
  summary: "Read every layer and say what state the cell is in, why, and what the operator does next",
  description: "OLP control and maintenance, machine status (stop reason, last error, loop, jog end, axes), rt-core status and recent events when configured, every drive alarm looked up against the drive profile. Conclusions follow the owner rules: never re-home to clear a fault, never route around a drive failure, non-resettable alarms are the owner's call. Moves nothing.",
  options: [{ name: "symptom", type: "string", description: "What the operator saw, e.g. 'Arm refused'", positional: 0 }],
  examples: ["rosie cell diagnose", "rosie cell diagnose 'J5 alarm after stop' --json"],
  async run(env, args) {
    const ctx = await env.ctx();
    const root = ctx.config.repoRoot;
    const read = await diagnoseRead(ctx);
    if (read.machine instanceof Error) return refused(`OLP status unavailable: ${read.machine.message}`, { exit: 3, next: ["rosie doctor"] });
    const machine = read.machine as DenseStatus;
    const findings: { level: "stop" | "warn" | "info"; what: string; next: string }[] = [];
    const m = read.maintenance ?? {};
    if (m.active) findings.push({ level: "warn", what: `maintenance notice: ${String(m.reason ?? "")} (${String(m.phase ?? "")}, ETA ${String(m.eta ?? "?")}) — arming is refused from every client`, next: "wait for the job to end; Stop and Disarm still pass" });
    const c = read.control ?? {};
    if (c.state === "other" || c.state === "other_server") findings.push({ level: "warn", what: `another client controls the robot: ${String(record(c.owner).label ?? "?")}`, next: "that client releases or presses Stop; never take over" });
    const t = machine.target;
    if (!t) findings.push({ level: "info", what: "no cell selected in OLP", next: "rosie cell list; select one from the browser/Deck or rosie robot select" });
    const s = machine.session;
    if (s?.stop_reason) findings.push({ level: "info", what: `last stop reason: ${s.stop_reason}`, next: s.stop_reason.includes("heartbeat") ? "the grant holder's heartbeats stopped >5 s (closed tab, lapsed keepalive)" : "rosie docs search " + JSON.stringify(s.stop_reason) });
    if (s?.last_error) findings.push({ level: "warn", what: `last error: ${s.last_error}`, next: `rosie docs search ${JSON.stringify(s.last_error.split(/[:\s]/)[0])}` });
    if (machine.daemon_error) findings.push({ level: "warn", what: `dense daemon error: ${machine.daemon_error}`, next: "rosie doctor" });
    const jogEnd = record(machine.last_jog_end);
    if (jogEnd.code && jogEnd.code !== "released") findings.push({ level: "info", what: `last jog ended: ${String(jogEnd.code)} — ${String(jogEnd.text ?? "")}`, next: "OLP README 'Jog end reasons' (rosie docs search \"Jog end reasons\")" });
    const alarms = new Map<string, string[]>();
    for (const axis of (Array.isArray(machine.axes) ? machine.axes : []).map(record)) {
      if (axis.drive_alarm) alarms.set(String(axis.drive_alarm), [...(alarms.get(String(axis.drive_alarm)) ?? []), String(axis.id ?? axis.name ?? "?")]);
      if (axis.home_valid === false) findings.push({ level: "warn", what: `${String(axis.id ?? "axis")} Home not valid`, next: "only genuine reference loss (encoder/battery alarm, coordinate_reference) needs HM35, run by an operator at the mechanical home pose — never to clear a fault" });
    }
    const rtAxes = (Array.isArray(read.rt?.axes) ? (read.rt!.axes as unknown[]) : []).map(record);
    rtAxes.forEach((axis, i) => {
      if (axis.drive_alarm) alarms.set(String(axis.drive_alarm), [...new Set([...(alarms.get(String(axis.drive_alarm)) ?? []), `J${i + 1}`])]);
    });
    const alarmInfo: Record<string, AlarmMatch | null> = {};
    for (const [code, axes] of alarms) {
      const match = (await lookupAlarm(root, code)).find((x) => x.exact) ?? null;
      alarmInfo[code] = match;
      findings.push(match?.rt_core_profile_resettable
        ? { level: "warn", what: `${code} ${match.name ?? ""} on ${axes.join(", ")} — the drive profile classes it resettable`, next: "an operator Arms after checking the cause (Arm resets it)" }
        : { level: "stop", what: `${code} ${match?.name ?? "(not in the manual table)"} on ${axes.join(", ")} — NOT resettable by software`, next: "the owner's call: inspect and fix at the drive; never re-home, never widen a tolerance" });
    }
    const core = record(read.rt?.core);
    for (const fault of (Array.isArray(record(core.recovery).faults) ? (record(core.recovery).faults as unknown[]) : []).map(record)) {
      findings.push({ level: "warn", what: `rt-core latched fault ${String(fault.name)} on ${maskAxes(Number(fault.axis_mask))} → recovery ${String(fault.recovery)}${fault.rehome_axis_mask ? ` (Home lost on ${maskAxes(Number(fault.rehome_axis_mask))})` : ""}`, next: String(fault.recovery) === "reset" ? "Arm resets it" : "rosie docs search " + JSON.stringify(String(fault.name)) });
    }
    if (read.rtEvents.some((e) => e.type === "events_dropped")) findings.push({ level: "info", what: "rt-control's event ring dropped events", next: "conclusions from events are incomplete; status is authoritative" });
    const stops = findings.filter((f) => f.level === "stop");
    const state = `${t ? `${t.cell_id} (${t.simulation ? "SIMULATION" : t.simulation === false ? "REAL ROBOT" : t.backend ?? "?"})` : "no cell"} · ${machine.armed ? "armed" : "disarmed"}${s ? ` · session ${s.kind}/${s.state}` : ""}`;
    const verdict = stops.length ? `${state} · ${stops.length} blocking: ${stops.map((f) => f.what.split(" — ")[0]).join("; ")}` : findings.some((f) => f.level === "warn") ? `${state} · ${findings.filter((f) => f.level === "warn").length} warning(s)` : `${state} · nothing wrong found`;
    const body = [
      args.symptom ? `symptom: ${String(args.symptom)}\n` : "",
      describeStatus(machine),
      "",
      findings.length ? table(["", "Finding", "Next"], findings.map((f) => [f.level.toUpperCase(), f.what, f.next])) : "no findings: no alarm, fault, notice, stop reason or error is reported",
      read.rtEvents.length ? `\nrecent rt-core events:\n${table(["Seq", "Type"], read.rtEvents.map((e) => [e.sequence, e.type]))}` : "",
    ].filter(Boolean).join("\n");
    return (stops.length ? refused : ok)(verdict, { body, data: { findings, alarms: alarmInfo, machine, control: read.control, maintenance: read.maintenance, rt_core: read.rt, rt_events: read.rtEvents } });
  },
};

const discover: Command<Env> = {
  path: ["cell", "discover"],
  summary: "Find OLP servers on the network (mDNS and the tailnet), as the browser's Search local network",
  options: [
    { name: "interface", type: "string", description: "Network interface for mDNS", default: "en0" },
    { name: "tailscale", type: "boolean", description: "Also ask tailnet peers", default: true },
  ],
  examples: ["rosie cell discover", "rosie cell discover --interface en1 --no-tailscale"],
  async run(env, args) {
    const root = await env.repoRoot();
    const dir = path.join(root, "offline-programming", "v1");
    const go = (await env.resolved()).config.goCommand;
    env.progress("searching (go run ./tools/olp-discover)…");
    const result = await run(go, ["run", "./tools/olp-discover", "--interface", String(args.interface), ...(args.tailscale ? ["--tailscale"] : []), "--json"], dir, 120_000);
    if (result.code !== 0) return refused(`olp-discover failed (exit ${result.code ?? "none"}): ${result.stderr.trim().slice(-500)}`);
    let found: unknown;
    try {
      found = JSON.parse(result.stdout);
    } catch {
      return refused(`olp-discover answered non-JSON: ${result.stdout.slice(0, 300)}`);
    }
    const list = (Array.isArray(found) ? found : Array.isArray(record(found).servers) ? (record(found).servers as unknown[]) : []).map(record);
    return ok(`${list.length} OLP server(s) found`, { body: list.length ? table(["Name", "Address", "Cell", "Source"], list.map((x) => [x.name ?? x.instance, x.address ?? x.url ?? `${String(x.host)}:${String(x.port)}`, x.cell ?? x.cell_id ?? "", x.source ?? ""])) : "", data: found });
  },
};

const docsSearch: Command<Env> = {
  path: ["docs", "search"],
  summary: "Search the component READMEs and contracts (measured numbers, owner rules)",
  options: [
    { name: "query", type: "string", description: "Plain text, case-insensitive", positional: 0, required: true },
    { name: "docs", type: "strings", description: `Restrict to ids: ${DOCS.map((d) => d.id).join(", ")}` },
    { name: "limit", type: "number", description: "Hits shown", default: 20 },
  ],
  examples: ["rosie docs search HM35", "rosie docs search 'loop_not_closed' --docs offline-programming"],
  async run(env, args) {
    const hits = await searchDocs(await env.repoRoot(), String(args.query), args.docs as string[] | undefined);
    const shown = hits.slice(0, Number(args.limit));
    return (hits.length ? ok : refused)(hits.length ? `${hits.length} hits for "${String(args.query)}"${hits.length > shown.length ? ` (first ${shown.length})` : ""}` : `no documentation line contains "${String(args.query)}"`, {
      body: shown.map((h) => `${h.file}:${h.line} (${h.heading || "top"}): ${h.text}`).join("\n"),
      data: { total: hits.length, hits: shown },
    });
  },
};

const docsShow: Command<Env> = {
  path: ["docs", "show"],
  summary: "Print one document, or the section under a heading",
  options: [
    { name: "doc", type: "string", description: `Document id: ${DOCS.map((d) => d.id).join(", ")}`, positional: 0, required: true },
    { name: "section", type: "string", description: "Heading text to print (its section only)", positional: 1 },
  ],
  examples: ["rosie docs show offline-programming 'Loop mode'", "rosie docs show repo-rules"],
  async run(env, args) {
    const repoRoot = await env.repoRoot();
    const docs = docsFor(repoRoot);
    const doc = docs.find((d) => d.id === args.doc);
    if (!doc) return refused(`unknown document ${String(args.doc)}; ids: ${docs.map((d) => d.id).join(", ")}`);
    const file = path.join(repoRoot, doc.path);
    if (!existsSync(file)) return refused(`${doc.path} is not in this checkout`);
    const lines = (await readFile(file, "utf8")).split(/\r?\n/);
    if (!args.section) return ok(`${doc.path} (${lines.length} lines)`, { body: lines.join("\n"), data: { file: doc.path } });
    const needle = String(args.section).toLowerCase();
    const start = lines.findIndex((line) => /^#{1,6} /.test(line) && line.toLowerCase().includes(needle));
    if (start < 0) return refused(`no heading in ${doc.path} contains "${String(args.section)}"`);
    const level = /^#+/.exec(lines[start]!)![0].length;
    let end = lines.findIndex((line, i) => i > start && /^#{1,6} /.test(line) && /^#+/.exec(line)![0].length <= level);
    if (end < 0) end = lines.length;
    return ok(`${doc.path}:${start + 1} ${lines[start]!.replace(/^#+ /, "")}`, { body: lines.slice(start, end).join("\n"), data: { file: doc.path, line: start + 1 } });
  },
};

const READ_ONLY_DEPLOY = {
  release_plan: { argv: ["realtime-core", "release", "--plan"], needsCommit: true, what: "the tier and steps a release of <commit> would run" },
  olp_status: { argv: ["olp", "status"], needsCommit: false, what: "the OLP and planner units" },
  precheck: { argv: ["realtime-core", "precheck"], needsCommit: false, what: "disarmed, no lease, OLP idle, Stop dumps copied, drive alarms" },
  verify: { argv: ["realtime-core", "verify"], needsCommit: false, what: "the running controller against the manifest" },
  config_dry_run: { argv: ["realtime-core", "config", "--dry-run"], needsCommit: false, what: "the pin an apply would install and every drive value it changes" },
  maintenance_show: { argv: ["realtime-core", "maintenance", "show"], needsCommit: false, what: "the maintenance notice on the cell's hosts" },
} as const;
type DeployCommand = keyof typeof READ_ONLY_DEPLOY;

async function deployRun(env: Env, cell: string, command: DeployCommand, commit: string | undefined, timeoutS: number) {
  const root = await env.repoRoot();
  const spec = READ_ONLY_DEPLOY[command];
  const args = ["run", ".", ...spec.argv, "--manifest", `rt-core/config/cells/${cell}.json`, ...(spec.needsCommit ? ["--commit", commit!] : [])];
  const go = (await env.resolved()).config.goCommand;
  env.progress(`${command}: go ${args.join(" ")}`);
  const started = Date.now();
  const out = await run(go, args, path.join(root, "steamdeck", "real", "v4"), timeoutS * 1000);
  return { command, argv: `${go} ${args.join(" ")}`, exit_code: out.code, timed_out: out.timedOut, elapsed_s: (Date.now() - started) / 1000, stdout: out.stdout, stderr: out.stderr };
}

const deployInspect: Command<Env> = {
  path: ["deploy", "inspect"],
  summary: "Run one read-only deploy command for a cell (plan, precheck, verify, olp status, config dry-run, maintenance)",
  description: "From steamdeck/real/v4 over this Mac's SSH/Tailscale. Releases, installs, restarts, locks, rollbacks and reboots are never run from rosie: they are the owner's repo commands (docs/DEPLOY_AND_TEST_WORKFLOW.md).",
  options: [
    { name: "cell", type: "string", description: "Manifest or short name (for example example-arm-1420)", positional: 0, required: true },
    { name: "command", type: "string", description: "Read-only command", positional: 1, required: true, choices: Object.keys(READ_ONLY_DEPLOY) },
    { name: "commit", type: "string", description: "Commit (release_plan)" },
    { name: "timeout_s", type: "number", description: "Give up after", default: 300 },
  ],
  examples: ["rosie deploy inspect <cell> precheck", "rosie deploy inspect <cell> release_plan --commit 7ff69b1a"],
  async run(env, args) {
    const cell = await resolveCellName(await env.repoRoot(), String(args.cell));
    const command = args.command as DeployCommand;
    if (READ_ONLY_DEPLOY[command].needsCommit && !/^[0-9a-f]{7,40}$/.test(String(args.commit ?? ""))) return refused(`${command} needs --commit <7-40 hex>`);
    const out = await deployRun(env, cell, command, args.commit as string | undefined, Number(args.timeout_s));
    const body = `${out.argv}\n\n${out.stdout.trim().slice(-20_000)}${out.stderr.trim() ? `\n\nstderr:\n${out.stderr.trim().slice(-5000)}` : ""}`;
    return (out.exit_code === 0 ? ok : refused)(`${command} for ${cell}: exit ${out.exit_code ?? "none"}${out.timed_out ? " (TIMED OUT)" : ""} in ${out.elapsed_s.toFixed(1)} s`, { body, data: out });
  },
};

const deployReview: Command<Env> = {
  path: ["deploy", "review"],
  summary: "What a release of <commit> would do to a cell, and what the owner must approve (read-only)",
  options: [
    { name: "cell", type: "string", description: "Manifest or short name (for example example-arm-1420)", positional: 0, required: true },
    { name: "commit", type: "string", description: "Commit to release", required: true },
    { name: "timeout_s", type: "number", description: "Per command", default: 300 },
  ],
  examples: ["rosie deploy review <cell> --commit 7ff69b1a"],
  async run(env, args) {
    const root = await env.repoRoot();
    const cell = await resolveCellName(root, String(args.cell));
    const commit = String(args.commit);
    if (!/^[0-9a-f]{7,40}$/.test(commit)) return refused("--commit must be 7-40 hex");
    const manifest = await readCell(root, cell);
    const [plan, precheck, olpStatus] = await Promise.all([
      deployRun(env, cell, "release_plan", commit, Number(args.timeout_s)),
      deployRun(env, cell, "precheck", undefined, Number(args.timeout_s)),
      deployRun(env, cell, "olp_status", undefined, Number(args.timeout_s)),
    ]);
    const tier = /tier[:=\s]+(config|olp|full)/i.exec(plan.stdout)?.[1] ?? "unknown";
    const servoHost = String(cellSummary(cell, manifest).servo_host ?? "");
    const approvals = [
      tier === "full" && /hostcheck/i.test(plan.stdout + plan.stderr) ? "--accept-hostcheck <check>: only for the failing hostcheck the plan names, accepted by the owner for this run; accept no other" : "",
      /config.drift|accept-config-drift/i.test(plan.stdout + plan.stderr) ? "--accept-config-drift <running pin>: the cell runs a pin no commit holds (owner's call; prefer committing the live values)" : "",
      precheck.exit_code !== 0 ? "precheck failed: the robot must be disarmed, unleased and idle (Finish cycle or Stop a loop first)" : "",
    ].filter(Boolean);
    const body = [
      `cell ${cell} (servo host ${servoHost}) · commit ${commit} · tier ${tier}`,
      "",
      "## release --plan", plan.stdout.trim().slice(-8000) || plan.stderr.trim().slice(-2000),
      "", `## precheck (exit ${precheck.exit_code})`, precheck.stdout.trim().slice(-3000) || precheck.stderr.trim().slice(-1000),
      "", `## olp status (exit ${olpStatus.exit_code})`, olpStatus.stdout.trim().slice(-3000) || olpStatus.stderr.trim().slice(-1000),
      "", "## owner approvals", approvals.length ? approvals.map((a) => `- ${a}`).join("\n") : "- none detected beyond running the release itself",
      "", "## the command (the owner runs it; rosie never does)",
      `cd steamdeck/real/v4 && go run . realtime-core release --commit ${commit} --manifest rt-core/config/cells/${cell}.json${approvals.some((a) => a.includes("igh_master")) ? " --accept-hostcheck igh_master" : ""}`,
    ].join("\n");
    return (plan.exit_code === 0 ? ok : refused)(`release of ${commit} to ${cell}: tier ${tier}; precheck ${precheck.exit_code === 0 ? "passes" : "FAILS"}; ${approvals.length} owner approval(s)`, { body, data: { cell, commit, tier, plan, precheck, olp_status: olpStatus, approvals } });
  },
};

export const commands: Command<Env>[] = [cellList, cellShow, diagnose, alarm, events, describe, discover, docsSearch, docsShow, deployInspect, deployReview];
