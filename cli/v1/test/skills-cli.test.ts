// `rosie skills`: the checkout's rosie* skills installed for every Claude
// Code session. The user's skills directory holds other people's skills, so
// what must hold is that rosie only ever creates, replaces or removes the
// entries it made, and that links point into this checkout (one source).
import assert from "node:assert/strict";
import { execFile } from "node:child_process";
import { lstatSync, mkdirSync, mkdtempSync, readdirSync, readFileSync, readlinkSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { test } from "node:test";

import { REPO_ROOT } from "./fakes.js";

const MAIN = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "src", "main.js");
const SOURCE = path.join(REPO_ROOT, ".claude", "skills");
const ROSIE_SKILLS = readdirSync(SOURCE).filter((name) => name === "rosie" || name.startsWith("rosie-")).sort();

function rosie(...args: string[]): Promise<{ ok: boolean; verdict: string; data: any; code: number }> {
  return new Promise((resolve, reject) => {
    execFile(process.execPath, [MAIN, ...args, "--json", "--quiet"], { env: { ...process.env, ROSIE_HOME: mkdtempSync(path.join(tmpdir(), "rosie-skills-home-")) } }, (error, stdout) => {
      try {
        resolve({ ...JSON.parse(stdout), code: error ? Number((error as { code?: number }).code ?? 1) : 0 });
      } catch {
        reject(new Error(`no envelope: ${stdout}`));
      }
    });
  });
}

test("install links every rosie skill into the checkout, is idempotent, and uninstall removes only rosie's", async () => {
  assert.ok(ROSIE_SKILLS.length >= 8, ROSIE_SKILLS.join(", "));
  const dest = mkdtempSync(path.join(tmpdir(), "claude-skills-"));
  mkdirSync(path.join(dest, "synced"));
  writeFileSync(path.join(dest, "synced", "SKILL.md"), "someone else's skill");

  const installed = await rosie("skills", "install", "--dest", dest);
  assert.equal(installed.ok, true, installed.verdict);
  for (const name of ROSIE_SKILLS) {
    const entry = path.join(dest, name);
    assert.ok(lstatSync(entry).isSymbolicLink(), `${name} is a link`);
    assert.equal(path.resolve(dest, readlinkSync(entry)), path.join(SOURCE, name), `${name} points into this checkout`);
    assert.match(readFileSync(path.join(entry, "SKILL.md"), "utf8"), new RegExp(`^---\\nname: ${name}\\n`));
  }
  const again = await rosie("skills", "install", "--dest", dest);
  assert.match(again.verdict, /already installed/);

  const removed = await rosie("skills", "uninstall", "--dest", dest);
  assert.equal(removed.ok, true);
  assert.deepEqual(readdirSync(dest), ["synced"], "only rosie's entries removed; the other skill is untouched");
  assert.equal(readFileSync(path.join(dest, "synced", "SKILL.md"), "utf8"), "someone else's skill");
});

test("a same-named skill rosie did not make is refused and nothing is changed", async () => {
  const dest = mkdtempSync(path.join(tmpdir(), "claude-skills-"));
  mkdirSync(path.join(dest, "rosie"));
  writeFileSync(path.join(dest, "rosie", "SKILL.md"), "a different rosie skill");
  const refused = await rosie("skills", "install", "--dest", dest);
  assert.equal(refused.code, 1);
  assert.match(refused.verdict, /not rosie's: nothing was changed/);
  assert.deepEqual(readdirSync(dest), ["rosie"], "no other skill was installed either");
  assert.equal(readFileSync(path.join(dest, "rosie", "SKILL.md"), "utf8"), "a different rosie skill");
});

test("--copy installs marked copies, list says when a copy is stale, and a re-install refreshes it", async () => {
  const dest = mkdtempSync(path.join(tmpdir(), "claude-skills-"));
  assert.equal((await rosie("skills", "install", "--dest", dest, "--copy")).ok, true);
  const entry = path.join(dest, "rosie");
  assert.equal(lstatSync(entry).isSymbolicLink(), false);
  assert.equal(readFileSync(path.join(entry, "SKILL.md"), "utf8"), readFileSync(path.join(SOURCE, "rosie", "SKILL.md"), "utf8"));
  const marker = path.join(entry, ".rosie-skill.json");
  writeFileSync(marker, JSON.stringify({ ...JSON.parse(readFileSync(marker, "utf8")), skill_md: "an older SKILL.md" }));
  const listed = await rosie("skills", "list", "--dest", dest);
  assert.equal(listed.data.skills.find((s: { name: string }) => s.name === "rosie").state, "stale copy");
  await rosie("skills", "install", "--dest", dest, "--copy");
  assert.equal((await rosie("skills", "list", "--dest", dest)).data.skills.find((s: { name: string }) => s.name === "rosie").state, "copied");
});
