// Prompts: the three conversations people have with a cell, written as the
// order of reads and the rules that bound them (CLAUDE.md, owner rules).
import type { McpServer } from "@modelcontextprotocol/server";
import * as z from "zod/v4";

import type { Context } from "rosie-cli/lib/context.js";

function user(text: string) {
  return { messages: [{ role: "user" as const, content: { type: "text" as const, text } }] };
}

const RULES = `Rules that hold whatever the tools allow:
- Never re-home to clear a fault. Home (HM35) makes the current position Home and is run only by an operator with each joint at its mechanical home pose.
- Never route around a drive or hardware failure (no widened tolerance, override or alternate reference): report it and name the drive alarm.
- A drive alarm the rt-core profile does not class resettable is the owner's call, not a retry.
- Building, testing or deploying on a cell's hosts and using its robot are mutually exclusive; deploys run only through the steamdeck/real/v4 repo commands, by the owner.
- A Stop receipt is not proof of standstill; if Stop is unconfirmed, the hardware E-stop is next.`;

export function registerPrompts(server: McpServer, ctx: Context): void {
  server.registerPrompt(
    "rosie_diagnose_cell",
    {
      title: "Diagnose the connected cell",
      description: "Read every layer (OLP, rt-core, dense daemon, planner) and explain what state the cell is in and why",
      argsSchema: z.object({ symptom: z.string().max(500).optional().describe("What the operator saw, e.g. 'Arm refused' or 'J5 alarm'") }),
    },
    ({ symptom }) =>
      user(`Diagnose the RosieOS cell${symptom ? ` for this symptom: "${symptom}"` : ""}. Read, in order, and do not act on the robot:
1. rosie_olp_get_control — is a maintenance notice up, and who controls the robot?
2. rosie_olp_get_machine_status — selected cell (simulation or real), armed, session, stop reason, last error, axes.
3. rosie_rtcore_get_status and rosie_rtcore_list_events (if rt-control is configured) — faults, recovery classes, Home validity, drive alarms, recent events.
4. rosie_lookup_drive_alarm for every drive alarm seen.
5. rosie_planner_get_health and rosie_dense_daemon_status if planning or playback is involved.
6. rosie_search_docs for the refusal codes you saw (they are documented in the component READMEs).
Then answer: what state is the cell in, what caused it, and what the operator should do next, citing the code and document for each claim.

${RULES}`),
  );

  server.registerPrompt(
    "rosie_run_program_in_simulation",
    {
      title: "Plan and run a program on a simulated cell",
      description: "Take a planned trajectory through Load and Run program on a simulation target, with the operator's confirmations",
      argsSchema: z.object({ trajectory_file: z.string().max(1000).optional().describe("Absolute path to the planned .rdt, if you have one") }),
    },
    ({ trajectory_file }) =>
      user(`Run a planned program on a SIMULATED cell. This server's robot-action mode is '${ctx.config.robotActions}'${ctx.config.robotActions === "off" ? " (robot actions are off: stop after the checks and say what would happen)" : ""}.
1. rosie_olp_get_machine_status: confirm the selected target says SIMULATION. If not, stop and say so.
2. ${trajectory_file ? `rosie_inspect_dense_trajectory on ${trajectory_file}` : "If a program is open here (rosie_program_view), plan it with rosie_program_plan and use from_open_program=true below; otherwise find the plan's dense identity (rosie_planner_fetch_dense / rosie_inspect_dense_trajectory)"} and read its identity and robot binding.
3. rosie_olp_run_program with that identity (or from_open_program=true), fraction 0.1, confirm flags false. If it answers 'confirm', show me the action and every joint's move, and wait for my answer.
4. Only after I agree: call it again with the confirm flags it asked for and the joints list unchanged.
5. Watch with rosie_olp_get_machine_status until the session is done; rosie_olp_stop at once if anything looks wrong.

${RULES}`),
  );

  server.registerPrompt(
    "rosie_author_weld_program",
    {
      title: "Author and plan a weld program from a STEP part",
      description: "New program, import the part, place it, search and accept its weld seams, plan, save to the cell",
      argsSchema: z.object({
        step_path: z.string().max(1000).describe("Absolute path to the part's .step file"),
        name: z.string().max(120).optional().describe("Program name (default: the file's name)"),
      }),
    },
    ({ step_path, name }) =>
      user(`Author a weld program for ${step_path}. Nothing here moves the robot.
1. rosie_program_new${name ? ` name="${name}"` : ""} (the connected cell's model), then rosie_program_import_step path="${step_path}".
2. Ask me where the part sits in the cell's work frame if I have not said; rosie_program_place_part.
3. rosie_program_list_seams; for each seam I want welded, rosie_program_search_seam, and report state, clear % and blocked length. For a 'partial' seam say which clear runs exist and ask whether to weld them, trim, or skip.
4. rosie_program_add_weld for the seams I approve; rosie_program_view to show the spreadsheet.
5. rosie_program_plan; on a refusal, quote OLP's code and sentence and the row it names.
6. Only when I say so: rosie_program_save target=cell (it is revision-checked; on a conflict, tell me and do not save_as_copy without asking).

${RULES}`),
  );

  server.registerPrompt(
    "rosie_review_release",
    {
      title: "Review what a release of a commit would do to a cell",
      description: "Read-only: the release plan, the cell's state and what the owner must approve",
      argsSchema: z.object({
        cell: z.string().max(100).describe("Cell manifest name, for example example-arm-1420"),
        commit: z.string().max(40).describe("Commit to release"),
      }),
    },
    ({ cell, commit }) =>
      user(`Review a release of ${commit} to ${cell}. Read-only throughout:
1. rosie_get_cell_manifest ${cell} (cell_deploy section) and rosie_list_cells.
2. rosie_deploy_inspect command=release_plan cell=${cell} commit=${commit}: the tier, its steps and why each test suite runs or is skipped.
3. rosie_deploy_inspect command=precheck and command=olp_status for ${cell}.
4. rosie_olp_get_control: is the robot in use or a notice already up?
Then summarise for the owner: tier and duration estimate from the docs (rosie_search_docs "Deployment tiers"), what changes on the robot, any hostcheck acceptance it needs (only the failing hostcheck the plan names, approved by the owner per run), and the exact command the owner would run. Do not run it.

${RULES}`),
  );
}
