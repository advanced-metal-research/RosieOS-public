#!/usr/bin/env node
import { execFileSync } from "node:child_process";
import { resolve } from "node:path";
import { fileURLToPath } from "node:url";

const fields = new Set(["runtime_paths", "deploy_allowed_roots", "artifact_scope", "daemon_index_url", "unit", "build_commands", "artifact_build_commands", "DAEMON_V1_DEPLOY_ALLOWED_ROOTS"]);
export function inventory(value, path = "$", result = new Set()) {
  if (!value || typeof value !== "object") return result;
  for (const [key, item] of Object.entries(value)) {
    const next = `${path}.${key}`;
    if (fields.has(key) || key.endsWith("_endpoint")) {
      const entries = Array.isArray(item) ? item : item && typeof item === "object" ? Object.entries(item) : [item];
      for (const entry of entries) result.add(`${next}: ${JSON.stringify(entry)}`);
    } else inventory(item, next, result);
  }
  return result;
}
export function difference(before, after) {
  return { added: [...after].filter(x => !before.has(x)).sort(), removed: [...before].filter(x => !after.has(x)).sort() };
}
function git(...args) { return execFileSync("git", args, { encoding: "utf8", maxBuffer: 32 * 1024 * 1024 }); }
export function report(base, head = "HEAD") {
  const pin = ref => git("rev-parse", "--verify", "--end-of-options", `${ref}^{commit}`).trim();
  base = pin(base); head = pin(head);
  const paths = ref => new Set(git("ls-tree", "-r", "--name-only", ref).trim().split("\n"));
  const beforePaths = paths(base), afterPaths = paths(head);
  const changes = [];
  const candidates = new Set([...beforePaths, ...afterPaths]);
  for (const path of [...candidates].sort()) {
    if (!path.startsWith("rt-core/config/cells/") && !path.includes("/manifest/") && !path.includes("/manifests/")) continue;
    if (!path.endsWith(".json")) continue;
    const before = beforePaths.has(path) ? git("show", `${base}:${path}`) : "{}";
    const after = afterPaths.has(path) ? git("show", `${head}:${path}`) : "{}";
    if (before === after) continue;
    const delta = difference(inventory(JSON.parse(before)), inventory(JSON.parse(after)));
    if (delta.added.length || delta.removed.length) changes.push({ path, ...delta });
  }
  const entrypoint = path => /\/cmd\/[^/]+\/main\.(go|rs)$|\.service$/.test(path);
  const entrypoints = difference(new Set([...beforePaths].filter(entrypoint)), new Set([...afterPaths].filter(entrypoint)));
  return { base, head, advisory: true, changes, entrypoints,
    limitations: "Declared manifest surface and named command/service files only. Not a count of executed services or config writers; dynamic/generated routes are not inferred. Array reordering may appear as changes." };
}
if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  try {
    if (!process.argv[2]) throw new Error("Usage: node quality/deployment-surface.mjs BASE [HEAD]");
    const result = report(process.argv[2], process.argv[3]);
    console.log("Deployment surface — advisory, not a merge gate");
    console.log(`${result.base} → ${result.head}`);
    for (const c of result.changes) {
      console.log(`\n${c.path}`);
      for (const v of c.added) console.log(`+ ${v}`);
      for (const v of c.removed) console.log(`- ${v}`);
    }
    for (const v of result.entrypoints.added) console.log(`+ command/service file: ${v}`);
    for (const v of result.entrypoints.removed) console.log(`- command/service file: ${v}`);
    console.log(`\n${result.limitations}`);
  } catch (error) { console.error(error.message); process.exitCode = 2; }
}
