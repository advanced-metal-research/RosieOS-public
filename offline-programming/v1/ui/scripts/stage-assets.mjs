import { cp, mkdir, readdir, readFile, stat, writeFile } from "node:fs/promises";
import { basename, dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

import { STEP_EXAMPLES } from "./step-examples.mjs";

const uiRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const productRoot = resolve(uiRoot, "..");
const repoRoot = resolve(productRoot, "..", "..");
const stageRoot = join(uiRoot, ".staged-public");

// Keep the watched public directory in place. Removing it during a build while
// Vite is serving makes its asset index stale and robot URLs fall back to HTML.
await mkdir(stageRoot, { recursive: true });
await cp(join(uiRoot, "public"), stageRoot, { recursive: true });

// Every arm this tree describes: each robot_description/robots directory with
// a robot.urdf. The descriptions are the record of the models, so a tree that
// leaves a robot out (the public mirror) stages only what it has.
const robotsRoot = join(repoRoot, "robot_description", "robots");
const models = [];
for (const entry of (await readdir(robotsRoot, { withFileTypes: true })).sort((a, b) => a.name.localeCompare(b.name))) {
  if (!entry.isDirectory()) continue;
  try {
    if ((await stat(join(robotsRoot, entry.name, "robot.urdf"))).isFile()) models.push(entry.name);
  } catch {
    // A bench or rail description has no URDF and no renderer model.
  }
}

for (const model of models) {
  const source = join(repoRoot, "robot_description", "robots", model);
  const destination = join(stageRoot, "robot", model);
  await cp(source, destination, {
    recursive: true,
    filter: (path) => !path.includes(`${join(model, "tools")}`) && !path.includes("fusion_part_export"),
  });
}

const manifest = {
  schema: "offline-programming.asset-stage.v1",
  source: "robot_description/robots",
  models,
};
await mkdir(join(stageRoot, "robot"), { recursive: true });
await writeFile(join(stageRoot, "robot", "manifest.json"), `${JSON.stringify(manifest, null, 2)}\n`);

// Example parts for "Open example" under Import STEP: copied from their
// homes in the repository, listed in a manifest the UI fetches. A tree that
// leaves a part out (the public mirror keeps the planner's sample workpieces
// private) offers only the parts it has; test/step-examples.test.ts keeps
// every part present in the full tree.
const examplesRoot = join(stageRoot, "examples");
await mkdir(examplesRoot, { recursive: true });
const examples = [];
for (const example of STEP_EXAMPLES) {
  const source = join(repoRoot, example.source);
  const filename = basename(example.source);
  let size;
  try {
    ({ size } = await stat(source));
  } catch {
    continue;
  }
  await cp(source, join(examplesRoot, filename));
  examples.push({ id: example.id, filename, label: example.label, description: example.description, bytes: size });
}
await writeFile(
  join(examplesRoot, "manifest.json"),
  `${JSON.stringify({ schema: "offline-programming.step-examples.v1", examples }, null, 2)}\n`,
);

// Prove the staged sources are actual URDF bundles instead of placeholder folders.
for (const model of manifest.models) {
  const urdf = await readFile(join(stageRoot, "robot", model, "robot.urdf"), "utf8");
  if (!urdf.includes("<robot")) throw new Error(`staged ${model} robot.urdf is invalid`);
}
