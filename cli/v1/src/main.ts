#!/usr/bin/env node
// rosie: the agent-first command line for RosieOS. See cli/v1/README.md.
import { commandHelp, exitCode, outcomeOf, parseArgs, renderJson, renderText, UsageError, type Command, type Outcome } from "./cli/command.js";
import { Env, type Globals } from "./cli/env.js";
import { GROUPS, allCommands } from "./cli/registry.js";

const VERSION = "0.1.0";

function splitGlobals(argv: string[]): { globals: Globals; rest: string[]; help: boolean } {
  const globals: Globals = { json: false, workspace: process.env.ROSIE_WORKSPACE || "current", quiet: false };
  const rest: string[] = [];
  let help = false;
  for (let i = 0; i < argv.length; i++) {
    const arg = argv[i]!;
    if (arg === "--json") globals.json = true;
    else if (arg === "--quiet" || arg === "-q") globals.quiet = true;
    else if (arg === "--help" || arg === "-h") help = true;
    else if (arg === "--target" || arg === "--ws") {
      const value = argv[++i];
      if (!value) throw new UsageError(`${arg} needs a value`);
      if (arg === "--target") globals.target = value;
      else globals.workspace = value;
    } else if (arg.startsWith("--target=")) globals.target = arg.slice(9);
    else if (arg.startsWith("--ws=")) globals.workspace = arg.slice(5);
    else rest.push(arg);
  }
  return { globals, rest, help };
}

function topHelp(commands: readonly Command<Env>[]): string {
  const lines = [
    `rosie ${VERSION} — RosieOS from the shell: author and plan OLP programs, verify them, look at the 3D scene, diagnose cells, run the (simulated) robot.`,
    "",
    "usage: rosie <group> <command> [args] [--json] [--target <name>] [--ws <workspace>]",
    "",
    "groups:",
    ...GROUPS.map((group) => `  ${group.name.padEnd(10)} ${group.summary}`),
    "",
    "start here:",
    "  rosie doctor              is every service up, and what is missing",
    "  rosie status              who has the robot, SIMULATION or REAL, armed, session",
    "  rosie program --help      write and plan a program; rosie view --help to look at it",
    "",
    "every command: one verdict line first; --json for {ok, verdict, data, artifacts, next};",
    "exit 0 ok, 1 refused / check failed, 2 usage, 3 service unreachable.",
    `${commands.length} commands; \`rosie <group> --help\` lists a group, \`rosie <group> <command> --help\` shows options and examples.`,
  ];
  return lines.join("\n");
}

function groupHelp(group: string, commands: readonly Command<Env>[]): string {
  const members = commands.filter((command) => command.path[0] === group);
  const summary = GROUPS.find((g) => g.name === group)?.summary ?? "";
  const width = Math.max(...members.map((command) => command.path.join(" ").length));
  return [`rosie ${group} — ${summary}`, "", ...members.map((command) => `  rosie ${command.path.join(" ").padEnd(width)}  ${command.summary}`), "", `\`rosie ${group} <command> --help\` for options and examples.`].join("\n");
}

/** The longest registered path that prefixes argv. */
function match(commands: readonly Command<Env>[], argv: string[]): { command: Command<Env>; rest: string[] } | null {
  let best: { command: Command<Env>; rest: string[] } | null = null;
  for (const command of commands) {
    const { path } = command;
    if (path.length <= argv.length && path.every((part, i) => argv[i] === part) && (!best || path.length > best.command.path.length)) {
      best = { command, rest: argv.slice(path.length) };
    }
  }
  return best;
}

function emit(outcome: Outcome, json: boolean): never {
  process.stdout.write(`${json ? renderJson(outcome) : renderText(outcome)}\n`);
  process.exit(exitCode(outcome));
}

async function main(): Promise<void> {
  let parsed: ReturnType<typeof splitGlobals>;
  try {
    parsed = splitGlobals(process.argv.slice(2));
  } catch (error) {
    emit(outcomeOf(error), process.argv.includes("--json"));
  }
  const { globals, rest, help } = parsed;
  const commands = allCommands();
  if (rest[0] === "--version" || rest[0] === "version") {
    process.stdout.write(`rosie ${VERSION}\n`);
    return;
  }
  if (rest.length === 0) {
    process.stdout.write(`${topHelp(commands)}\n`);
    process.exit(help ? 0 : 2);
  }
  const found = match(commands, rest);
  if (!found) {
    if (GROUPS.some((group) => group.name === rest[0])) {
      process.stdout.write(`${groupHelp(rest[0]!, commands)}\n`);
      process.exit(help || rest.length === 1 ? 0 : 2);
    }
    emit({ ok: false, verdict: `usage: unknown command "${rest.join(" ")}"`, exit: 2, next: ["rosie --help"] }, globals.json);
  }
  if (help) {
    process.stdout.write(`${commandHelp(found.command as Command<never>)}\n`);
    return;
  }
  const env = new Env(globals);
  let outcome: Outcome;
  try {
    const args = parseArgs(found.command.options ?? [], found.rest);
    outcome = await found.command.run(env, args);
  } catch (error) {
    outcome = outcomeOf(error);
    if (outcome.exit === 2) outcome.next = [`rosie ${found.command.path.join(" ")} --help`];
  } finally {
    await env.close();
  }
  emit(outcome, globals.json);
}

await main();
