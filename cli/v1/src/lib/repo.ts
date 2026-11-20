// Answers from this checkout alone: committed cell manifests, the A6-EC alarm
// tables and the component documentation. Nothing here contacts a host. The
// MCP server's repo tools and the CLI's `cell`, `alarm` and `docs` commands
// both read through these, so a lookup answers the same from either.
import { existsSync, readFileSync } from "node:fs";
import { readdir, readFile } from "node:fs/promises";
import path from "node:path";

import { record } from "./text.js";

const CELL_NAME = /^[A-Za-z0-9._-]+$/;

/**
 * The documents an agent needs to work on RosieOS, by stable id (also the MCP
 * resources). A listed document may be absent from a checkout: the site records
 * of the real cells stay in the private repository and the public mirror omits
 * them. Search skips an absent document and `docs show` refuses it by name, so
 * one list serves both trees.
 */
export const DOCS: ReadonlyArray<{ id: string; path: string; title: string }> = [
  { id: "repo-rules", path: "CLAUDE.md", title: "RosieOS working rules (owner rules, deploy, Home, safety)" },
  { id: "readme", path: "README.md", title: "RosieOS overview: how a cell is wired, components, ports" },
  { id: "deploy-and-test-workflow", path: "docs/DEPLOY_AND_TEST_WORKFLOW.md", title: "Deploying and testing on a robot cell" },
  { id: "planning-context", path: "docs/PLANNING-CONTEXT.md", title: "Planning context contract" },
  { id: "robot-description-contract", path: "docs/ROBOT-DESCRIPTION-CONTRACT.md", title: "Robot description contract" },
  { id: "rt-core", path: "rt-core/README.md", title: "rt-core: the native realtime controller" },
  { id: "rt-core-application-api", path: "rt-core/docs/APPLICATION_API.md", title: "rt-control public API" },
  { id: "rt-core-contracts", path: "rt-core/docs/CONTRACTS.md", title: "rt-core contracts" },
  { id: "rt-core-config", path: "rt-core/config/README.md", title: "rt-core configuration layers" },
  { id: "rt-core-cells", path: "rt-core/config/cells/README.md", title: "Cell bindings" },
  { id: "a6ec-drive", path: "rt-core/docs/drives/a6ec/README.md", title: "A6-EC drive reference and tuning a joint" },
  { id: "motion-server", path: "motion-server/v1/README.md", title: "Cartesian motion server (robot-v4-cartesiand)" },
  { id: "dense-daemon", path: "motion-server/joint-trajectory/v1/README.md", title: "Dense trajectory daemon" },
  { id: "dense-format", path: "motion-server/joint-trajectory/v1/docs/DENSE-JOINT-TRAJECTORY-V1.md", title: "robot.v4.dense-joint-trajectory.v1 format" },
  { id: "offline-programming", path: "offline-programming/v1/README.md", title: "Offline programming (OLP)" },
  { id: "weld-planner", path: "weld_planner/v1/README.md", title: "Weld planner" },
  { id: "tesseract", path: "tesseract/v1/README.md", title: "Tesseract collision environment" },
  { id: "robot-description", path: "robot_description/README.md", title: "Robot descriptions" },
  { id: "deploy-tool", path: "steamdeck/real/v4/README.md", title: "Deploy module (realtime-core, olp, pendant commands)" },
  { id: "pendant-v5", path: "steamdeck/real/v5/README.md", title: "Steam Deck pendant v5" },
  { id: "mcp", path: "mcp/v1/README.md", title: "The RosieOS MCP server" },
  { id: "cli", path: "cli/v1/README.md", title: "The rosie CLI" },
];

export function cellsDir(repoRoot: string): string {
  return path.join(repoRoot, "rt-core", "config", "cells");
}

export async function listCellNames(repoRoot: string): Promise<string[]> {
  return (await readdir(cellsDir(repoRoot))).filter((file) => file.endsWith(".json")).map((file) => file.slice(0, -5)).sort();
}

export async function readCell(repoRoot: string, name: string): Promise<Record<string, unknown>> {
  if (!CELL_NAME.test(name)) throw new Error(`cell name ${JSON.stringify(name)} is not a manifest name`);
  return record(JSON.parse(await readFile(path.join(cellsDir(repoRoot), `${name}.json`), "utf8")));
}

/** A committed cell by its manifest name or a short name that one manifest name contains (a robot id -> its cell's manifest). */
export async function resolveCellName(repoRoot: string, query: string): Promise<string> {
  const names = await listCellNames(repoRoot);
  if (names.includes(query)) return query;
  const matches = names.filter((name) => name.split("-").includes(query) || name.includes(query));
  if (matches.length === 1) return matches[0]!;
  throw new Error(matches.length ? `"${query}" matches several cells: ${matches.join(", ")}` : `no committed cell manifest "${query}"; known: ${names.join(", ")}`);
}

/** The facts an agent asks of a manifest: rt-core binding, OLP unit, policy. */
export function cellSummary(name: string, manifest: Record<string, unknown>): Record<string, unknown> {
  const nodes = Array.isArray(manifest.nodes) ? manifest.nodes.map(record) : [];
  const rtNode = nodes.find((node) => node.rt_core);
  const rt = record(rtNode?.rt_core);
  const olp = record(record(manifest.cell_deploy).olp);
  return {
    file: name,
    name: manifest.name ?? null,
    schema_version: manifest.schema_version ?? null,
    automation_mode: record(manifest.motion_policy).automation_mode ?? null,
    servo_host: rtNode?.host ?? rtNode?.node ?? null,
    machine_config: rt.machine_config ?? null,
    configuration_sha256: rt.configuration_sha256 ?? null,
    pair_id: rt.pair_id ?? null,
    remote_listen: rt.remote_listen ?? null,
    robot_axes: Array.isArray(manifest.robot_axes) ? manifest.robot_axes.length : null,
    has_cell_deploy: manifest.cell_deploy !== undefined,
    olp_unit: olp.unit ?? null,
    olp_listen: olp.listen ?? null,
    unattended_loop_allowed: olp.unattended_loop_allowed ?? null,
    program_store: record(olp.program_store).label ?? null,
  };
}

export interface AlarmMatch {
  table: string;
  key: string;
  code?: string;
  name?: string;
  resettable?: boolean;
  bus_fault_code_603f?: string;
  fault_code_203f?: string;
  alarm_code_203f?: string;
  exact: boolean;
  /** The drive profile's fault_reset list: whether Arm's CiA402 reset clears it (null for bus fault codes). */
  rt_core_profile_resettable: boolean | null;
}

/**
 * An A6-EC code or name from the vendor manual table, and whether rt-core's
 * drive profile classes it resettable. A code the profile does not list is
 * not resettable by software: it needs the drive or the hardware fixed, and
 * Home is never re-run to clear a fault. Exact code matches sort first.
 */
export async function lookupAlarm(repoRoot: string, query: string): Promise<AlarmMatch[]> {
  const manual = record(JSON.parse(await readFile(path.join(repoRoot, "rt-core", "docs", "drives", "a6ec", "a6ec_manual_codes.json"), "utf8")));
  const profile = record(JSON.parse(await readFile(path.join(repoRoot, "rt-core", "config", "drives", "a6ec.json"), "utf8")));
  const listed = record(profile.fault_reset).resettable_alarms;
  const resettable = new Set((Array.isArray(listed) ? listed : []).map((a) => String(record(a).code).toLowerCase()));
  const needle = query.trim().toLowerCase();
  const isHex = (value: string) => /^0x[0-9a-f]+$/.test(value);
  // The manual writes hex codes at mixed widths (0X010, 0X0F00): compare values.
  const hexValue = isHex(needle) ? parseInt(needle, 16) : null;
  const matches: AlarmMatch[] = [];
  for (const [tableName, entries] of Object.entries(record(manual.tables))) {
    for (const [key, raw] of Object.entries(record(entries))) {
      const entry = record(raw) as Omit<AlarmMatch, "table" | "key" | "exact" | "rt_core_profile_resettable">;
      const codes = [key, entry.code, entry.bus_fault_code_603f, entry.fault_code_203f, entry.alarm_code_203f].filter(Boolean).map((v) => String(v).toLowerCase());
      const exact = codes.includes(needle) || (hexValue !== null && codes.some((code) => isHex(code) && parseInt(code, 16) === hexValue));
      if (exact || String(entry.name ?? "").toLowerCase().includes(needle)) {
        matches.push({
          table: tableName, key, ...entry, exact,
          rt_core_profile_resettable: tableName === "bus_fault_codes" ? null : resettable.has(String(entry.code ?? key).toLowerCase()),
        });
      }
    }
  }
  return matches.sort((a, b) => Number(b.exact) - Number(a.exact));
}

export interface DocHit { doc: string; file: string; line: number; heading: string; text: string }

/**
 * The site's own documents, listed in docs/site/docs.json beside the private
 * notes ([{ "id", "path", "title" }]): records of the real cells that the
 * public mirror leaves out, so no site document is named in code. Absent in a
 * public checkout; a malformed list is an error, not an empty one.
 */
export function siteDocs(repoRoot: string): ReadonlyArray<{ id: string; path: string; title: string }> {
  const file = path.join(repoRoot, "docs", "site", "docs.json");
  if (!existsSync(file)) return [];
  const list: unknown = JSON.parse(readFileSync(file, "utf8"));
  if (!Array.isArray(list) || !list.every((d) => d && typeof d.id === "string" && /^[a-z0-9-]+$/.test(d.id) && typeof d.path === "string" && typeof d.title === "string"))
    throw new Error(`${file} must be a list of { id, path, title }`);
  return list;
}

/** Every document this checkout can search: the public list and the site's. */
export function docsFor(repoRoot: string): ReadonlyArray<{ id: string; path: string; title: string }> {
  return [...DOCS, ...siteDocs(repoRoot)];
}

/**
 * Every line of the listed documents containing `query` (case-insensitive),
 * with its nearest heading. A listed document absent from this checkout (a
 * private-only record in the public mirror) is skipped; any other read failure
 * is thrown, so a broken checkout is not reported as "no hits".
 */
export async function searchDocs(repoRoot: string, query: string, docs?: readonly string[]): Promise<DocHit[]> {
  const needle = query.toLowerCase();
  const hits: DocHit[] = [];
  for (const doc of docsFor(repoRoot).filter((d) => !docs || docs.includes(d.id))) {
    let content: string;
    try {
      content = await readFile(path.join(repoRoot, doc.path), "utf8");
    } catch (error) {
      if ((error as NodeJS.ErrnoException).code === "ENOENT") continue;
      throw error;
    }
    let heading = "";
    content.split(/\r?\n/).forEach((line, index) => {
      if (/^#{1,6} /.test(line)) heading = line.replace(/^#+ /, "");
      if (line.toLowerCase().includes(needle)) hits.push({ doc: doc.id, file: doc.path, line: index + 1, heading, text: line.trim().slice(0, 300) });
    });
  }
  return hits;
}
