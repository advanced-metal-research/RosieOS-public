// The deploy module's read-only commands (steamdeck/real/v4, run with
// `go run .`; CLAUDE.md "Deploying and testing on a robot cell"). Each one is
// documented as reading the cell without changing it. Commands that build,
// install, restart, roll back, lock or reboot are not offered: those are the
// owner's, run by hand from the operator Mac with the cell lock.
import { execFile } from "node:child_process";
import { existsSync } from "node:fs";
import path from "node:path";

import type { McpServer } from "@modelcontextprotocol/server";
import * as z from "zod/v4";

import type { Context } from "rosie-cli/lib/context.js";
import { failure, guarded, text, truncate } from "../format.js";
import { listCellNames } from "./repo.js";

const READ_ONLY_COMMANDS = {
  release_plan: { argv: ["realtime-core", "release", "--plan"], needsCommit: true, what: "the tier and steps a release of <commit> would run, and why each test suite runs or is skipped" },
  olp_status: { argv: ["olp", "status"], needsCommit: false, what: "the OLP and planner units: working directory, planner revision, pins, health" },
  precheck: { argv: ["realtime-core", "precheck"], needsCommit: false, what: "disarmed, no lease, OLP idle, Stop dumps copied, drive alarms" },
  verify: { argv: ["realtime-core", "verify"], needsCommit: false, what: "acceptance of the running controller against the manifest" },
  config_dry_run: { argv: ["realtime-core", "config", "--dry-run"], needsCommit: false, what: "a read-only probe of both hosts: the configuration pin an apply would install and every drive value it would change" },
  maintenance_show: { argv: ["realtime-core", "maintenance", "show"], needsCommit: false, what: "the maintenance notice on the cell's hosts" },
} as const;

type Command = keyof typeof READ_ONLY_COMMANDS;

function run(command: string, args: string[], cwd: string, timeoutMs: number): Promise<{ code: number | null; stdout: string; stderr: string; timedOut: boolean }> {
  return new Promise((resolve) => {
    execFile(command, args, { cwd, timeout: timeoutMs, maxBuffer: 16 * 1024 * 1024, windowsHide: true }, (error, stdout, stderr) => {
      const failed = error as (NodeJS.ErrnoException & { killed?: boolean; code?: number | string }) | null;
      if (failed && typeof failed.code === "string") {
        resolve({ code: null, stdout, stderr: `${stderr}\ncould not run ${command}: ${failed.message}`, timedOut: false });
        return;
      }
      resolve({ code: failed ? (typeof failed.code === "number" ? failed.code : 1) : 0, stdout, stderr, timedOut: Boolean(failed?.killed) });
    });
  });
}

export function registerDeployTools(server: McpServer, ctx: Context): void {
  const moduleDir = path.join(ctx.config.repoRoot, "steamdeck", "real", "v4");

  server.registerTool(
    "rosie_deploy_inspect",
    {
      title: "Run a read-only deploy command for a cell",
      description: `Run one of the deploy module's read-only commands for a committed cell, from steamdeck/real/v4 with \`${ctx.config.goCommand} run . <command> --manifest rt-core/config/cells/<cell>.json\`:
${Object.entries(READ_ONLY_COMMANDS).map(([name, c]) => `  - ${name}: \`${c.argv.join(" ")}\` — ${c.what}`).join("\n")}

They contact the cell's hosts over the operator machine's own SSH/Tailscale setup and change nothing. Releases, installs, restarts, rollbacks, locks and reboots are deliberately not available here: run them by hand, with the owner's approval where CLAUDE.md requires it. Run release_plan before proposing any release.

Args:
  - cell: manifest name, rt-core/config/cells/<cell>.json without .json (for example example-arm-1420).
  - command: one of the names above.
  - commit: required for release_plan (7-40 hex).
  - timeout_s: default 300.`,
      inputSchema: z
        .object({
          cell: z.string().regex(/^[A-Za-z0-9._-]+$/),
          command: z.enum(Object.keys(READ_ONLY_COMMANDS) as [Command, ...Command[]]),
          commit: z.string().regex(/^[0-9a-f]{7,40}$/).optional(),
          timeout_s: z.number().int().min(10).max(1800).default(300),
        })
        .strict(),
      annotations: { readOnlyHint: true, destructiveHint: false, idempotentHint: true, openWorldHint: true },
    },
    guarded(async ({ cell, command, commit, timeout_s }) => {
      if (!existsSync(moduleDir)) return failure(`the deploy module is not in this checkout (${moduleDir})`);
      const cells = await listCellNames(ctx);
      if (!cells.includes(cell)) return failure(`no committed cell manifest ${cell}; known: ${cells.join(", ")}`);
      const spec = READ_ONLY_COMMANDS[command];
      if (spec.needsCommit && !commit) return failure(`${command} needs commit`);
      const args = ["run", ".", ...spec.argv, "--manifest", `rt-core/config/cells/${cell}.json`, ...(spec.needsCommit ? ["--commit", commit!] : [])];
      const started = Date.now();
      const outcome = await run(ctx.config.goCommand, args, moduleDir, timeout_s * 1000);
      const elapsed = ((Date.now() - started) / 1000).toFixed(1);
      const data = { command: [ctx.config.goCommand, ...args].join(" "), cwd: moduleDir, exit_code: outcome.code, timed_out: outcome.timedOut, elapsed_s: Number(elapsed), stdout: truncate(outcome.stdout, 20_000), stderr: truncate(outcome.stderr, 5_000) };
      const body = `# ${command} for ${cell} (exit ${outcome.code ?? "none"}${outcome.timedOut ? ", TIMED OUT" : ""}, ${elapsed} s)\n\`${data.command}\`\n\n\`\`\`\n${data.stdout}\n\`\`\`${data.stderr.trim() ? `\n\nstderr:\n\`\`\`\n${data.stderr}\n\`\`\`` : ""}`;
      return outcome.code === 0 ? text(body, data) : { ...text(body, data), isError: true };
    }),
  );
}
