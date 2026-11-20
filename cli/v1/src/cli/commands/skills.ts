// rosie skills: make the rosie* agent skills (the checkout's .claude/skills)
// available to every Claude Code session on this machine, not only sessions
// opened in this repository, by linking them into the user's skills directory
// (~/.claude/skills, or $CLAUDE_CONFIG_DIR/skills).
//
// Links, not copies, by default: the checkout stays the single source, so a
// `git pull` updates every session's skills at once and nothing drifts.
// --copy is for filesystems without symlinks (a copy carries a marker file
// and `rosie skills list` says when it is stale). The user's skills
// directory holds other people's skills: rosie touches only entries it made
// (a link into a rosie checkout, or a copy with its marker) and refuses to
// replace anything else.
import { cp, lstat, mkdir, readdir, readFile, readlink, rm, symlink, writeFile } from "node:fs/promises";
import { homedir } from "node:os";
import path from "node:path";

import { table } from "../../lib/text.js";
import { ok, refused, type Command } from "../command.js";
import type { Env } from "../env.js";

const MARKER = ".rosie-skill.json";

interface SkillEntry { name: string; source: string; dest: string; state: "linked" | "copied" | "stale copy" | "linked elsewhere" | "missing" | "foreign" }

function userSkillsDir(env: NodeJS.ProcessEnv, override?: unknown): string {
  if (override) return path.resolve(String(override));
  return path.join(env.CLAUDE_CONFIG_DIR ? path.resolve(env.CLAUDE_CONFIG_DIR) : path.join(homedir(), ".claude"), "skills");
}

/** The checkout's rosie skills: .claude/skills/rosie and .claude/skills/rosie-*. */
async function sourceSkills(repoRoot: string): Promise<{ name: string; dir: string }[]> {
  const root = path.join(repoRoot, ".claude", "skills");
  const names = (await readdir(root).catch(() => [] as string[])).filter((name) => name === "rosie" || name.startsWith("rosie-")).sort();
  const skills = [];
  for (const name of names) {
    const dir = path.join(root, name);
    if (await lstat(path.join(dir, "SKILL.md")).then(() => true, () => false)) skills.push({ name, dir });
  }
  return skills;
}

async function state(source: string, dest: string): Promise<SkillEntry["state"]> {
  let info;
  try {
    info = await lstat(dest);
  } catch {
    return "missing";
  }
  if (info.isSymbolicLink()) {
    const target = path.resolve(path.dirname(dest), await readlink(dest));
    if (target === source) return "linked";
    // A link into another rosie checkout is ours to replace; any other link is not.
    return /[\\/]\.claude[\\/]skills[\\/]rosie(-[a-z-]+)?$/.test(target) ? "linked elsewhere" : "foreign";
  }
  const marker = await readFile(path.join(dest, MARKER), "utf8").catch(() => null);
  if (!marker) return "foreign";
  const copied = JSON.parse(marker) as { source?: string; skill_md?: string };
  const current = await readFile(path.join(source, "SKILL.md"), "utf8").catch(() => "");
  return copied.source === source && copied.skill_md === current ? "copied" : "stale copy";
}

async function entries(env: Env, dest: unknown): Promise<{ dir: string; list: SkillEntry[] }> {
  const dir = userSkillsDir(env.processEnv, dest);
  const repoRoot = await env.repoRoot();
  const list: SkillEntry[] = [];
  for (const skill of await sourceSkills(repoRoot)) {
    const target = path.join(dir, skill.name);
    list.push({ name: skill.name, source: skill.dir, dest: target, state: await state(skill.dir, target) });
  }
  return { dir, list };
}

const DEST = { name: "dest", type: "string", description: "Skills directory (default ~/.claude/skills, or $CLAUDE_CONFIG_DIR/skills)" } as const;

const list: Command<Env> = {
  path: ["skills", "list"],
  summary: "The rosie agent skills in this checkout and whether every Claude Code session on this machine has them",
  options: [DEST],
  examples: ["rosie skills list"],
  async run(env, args) {
    const { dir, list: skills } = await entries(env, args.dest);
    if (!skills.length) return refused(`no rosie skills in ${path.join(await env.repoRoot(), ".claude", "skills")}`);
    const installed = skills.filter((s) => s.state === "linked" || s.state === "copied").length;
    return ok(`${installed}/${skills.length} installed in ${dir}`, {
      body: table(["Skill", "State", "Installed at"], skills.map((s) => [s.name, s.state, s.state === "missing" ? "" : s.dest])),
      data: { dir, skills },
      next: installed < skills.length ? ["rosie skills install"] : [],
    });
  },
};

const install: Command<Env> = {
  path: ["skills", "install"],
  summary: "Make the rosie skills available to every Claude Code session on this machine (links into ~/.claude/skills)",
  description: "Links ~/.claude/skills/<skill> to this checkout's .claude/skills/<skill>, so a git pull updates them everywhere. --copy copies instead (no symlinks on this filesystem); re-run after a pull. Replaces only entries rosie made (a link into a rosie checkout, a marked copy); refuses anything else with that name. Idempotent. Start a new Claude Code session to pick them up.",
  options: [DEST, { name: "copy", type: "boolean", description: "Copy instead of linking" }],
  examples: ["rosie skills install", "rosie skills install --copy"],
  async run(env, args) {
    const { dir, list: skills } = await entries(env, args.dest);
    if (!skills.length) return refused(`no rosie skills in ${path.join(await env.repoRoot(), ".claude", "skills")}`);
    const foreign = skills.filter((s) => s.state === "foreign");
    if (foreign.length) {
      return refused(`${foreign.map((s) => s.dest).join(", ")} already exist${foreign.length === 1 ? "s" : ""} and ${foreign.length === 1 ? "is" : "are"} not rosie's: nothing was changed. Move ${foreign.length === 1 ? "it" : "them"} aside, or install into another --dest`, { data: { dir, skills } });
    }
    await mkdir(dir, { recursive: true });
    const done: string[] = [];
    for (const skill of skills) {
      const wanted = args.copy ? "copied" : "linked";
      if (skill.state === wanted) continue;
      await rm(skill.dest, { recursive: true, force: true });
      if (args.copy) {
        await cp(skill.source, skill.dest, { recursive: true });
        await writeFile(path.join(skill.dest, MARKER), JSON.stringify({ source: skill.source, skill_md: await readFile(path.join(skill.source, "SKILL.md"), "utf8"), copied_at: new Date().toISOString() }));
      } else {
        await symlink(skill.source, skill.dest, "dir");
      }
      done.push(`${skill.name} (${skill.state === "missing" ? wanted : `${skill.state} -> ${wanted}`})`);
    }
    const after = await entries(env, args.dest);
    return ok(done.length ? `installed ${done.length} skill(s) in ${dir}${args.copy ? " (copies: re-run after a pull)" : ""}; start a new Claude Code session to load them` : `all ${skills.length} rosie skills already installed in ${dir}`, {
      body: table(["Skill", "State", "Source"], after.list.map((s) => [s.name, s.state, s.source])),
      data: after,
    });
  },
};

const uninstall: Command<Env> = {
  path: ["skills", "uninstall"],
  summary: "Remove the rosie skills from ~/.claude/skills (only entries rosie made)",
  options: [DEST],
  async run(env, args) {
    const { dir, list: skills } = await entries(env, args.dest);
    const ours = skills.filter((s) => s.state !== "missing" && s.state !== "foreign");
    for (const skill of ours) await rm(skill.dest, { recursive: true, force: true });
    const kept = skills.filter((s) => s.state === "foreign").map((s) => s.dest);
    return ok(`removed ${ours.length} rosie skill(s) from ${dir}${kept.length ? `; left ${kept.join(", ")} (not rosie's)` : ""}`);
  },
};

export const commands: Command<Env>[] = [list, install, uninstall];
