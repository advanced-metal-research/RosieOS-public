import { readdir } from "node:fs/promises";
import { join, resolve } from "node:path";
import { spawnSync } from "node:child_process";

const uiRoot = resolve(import.meta.dirname, "..");
const tests = [
  ...(await discover(join(uiRoot, "src"))),
  ...(await discover(join(uiRoot, "test"))),
].sort();

if (tests.length === 0) {
  throw new Error("no Offline Programming TypeScript tests discovered");
}

// The robot catalogue the app fetches at boot, installed before each test
// file: projects name a robot and their placement hangs off that robot's work
// frame, so an empty catalogue would make every placement test a test about
// a missing description instead.
const seed = join(uiRoot, "test", "support", "seed.ts");
const result = spawnSync(
  process.execPath,
  ["--experimental-strip-types", "--import", seed, "--test", ...tests],
  { cwd: uiRoot, stdio: "inherit" },
);
if (result.error) throw result.error;
process.exitCode = result.status ?? 1;

async function discover(root) {
  const files = [];
  for (const entry of await readdir(root, { withFileTypes: true })) {
    const path = join(root, entry.name);
    if (entry.isDirectory()) {
      files.push(...await discover(path));
    } else if (entry.isFile() && entry.name.endsWith(".test.ts")) {
      files.push(path);
    }
  }
  return files;
}
