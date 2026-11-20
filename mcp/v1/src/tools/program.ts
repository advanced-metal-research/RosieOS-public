// Programs: OLP's spreadsheet, edited through OLP's own engine (core/entry.ts),
// so every row, field, limit and refusal is the browser's and the Deck's.
// Nothing here moves the robot: planning is a computation on the planner, and
// a planned program reaches the robot only through rosie_olp_load /
// rosie_olp_run_program and their gates.
import type { CallToolResult, McpServer } from "@modelcontextprotocol/server";
import * as z from "zod/v4";

import type { Context } from "rosie-cli/lib/context.js";
import type {
  Engine, FixtureEdit, FixtureField, RailsSettings, WaypointEdit, WaypointInput, WaypointWeldView, WeldTool,
} from "rosie-cli/lib/engine.js";
import * as programs from "rosie-cli/lib/programs.js";
import { describeProgram as describeOpen, fixtureLine, mm, spreadsheet as sheet } from "rosie-cli/lib/programs.js";
import { failure, guarded, json, responseFormat, result, table, text, type ResponseFormat } from "../format.js";

const EDITS = { readOnlyHint: false, destructiveHint: false, idempotentHint: false, openWorldHint: false } as const;
const EDITS_REMOTE = { readOnlyHint: false, destructiveHint: false, idempotentHint: false, openWorldHint: true } as const;
const READS = { readOnlyHint: true, destructiveHint: false, idempotentHint: true, openWorldHint: false } as const;

const rowId = z.string().min(1).max(200).describe("Row (node) id from rosie_program_view");
const vec3 = z.array(z.number().finite()).length(3);
const fixtureStepPaths = z.array(z.string().min(1)).max(50).optional()
  .describe("STEP files of workcell fixtures or other parts this server has not kept (each is matched by its sha256)");

function waypointTable(weld: WaypointWeldView): string {
  return [
    `**${weld.label}** (${weld.tool}${weld.startFrom ? `, starts at the end of \`${weld.startFrom}\`` : ""}) · travel ${weld.travelSpeedMmS ?? "profile"} mm/s · part frame`,
    table(["#", "xyz mm", "rpy °", "Continuity", "Standoff mm", "s"], weld.points.map((p) => [
      p.index, mm(p.xyzMm), mm(p.rpyDeg), p.continuity.toUpperCase(), p.standoffMm, p.s.toFixed(3) + (p.tied ? " (tied)" : ""),
    ])),
  ].join("\n");
}

function editValue() {
  return z.union([z.number(), z.string(), z.boolean(), z.null(), z.array(z.number())]).describe("New value in the field's display units (mm, degrees, %, s), as rosie_program_get_row lists it");
}

export function describeProgram(engine: Engine, ctx: Context): Record<string, unknown> {
  return describeOpen(engine, ctx.program);
}

function spreadsheet(engine: Engine, ctx: Context): string {
  return sheet(engine, ctx.program);
}

function answer(engine: Engine, ctx: Context, format: ResponseFormat, headline: string): CallToolResult {
  return result(format, describeProgram(engine, ctx), () => `${headline}\n\n${spreadsheet(engine, ctx)}`);
}

export function registerProgramTools(server: McpServer, ctx: Context): void {
  const session = ctx.program;

  /** A tool body on the open program, serialised through the session. */
  const onProgram = <A>(handler: (engine: Engine, args: A) => Promise<CallToolResult>) =>
    guarded((args: A) => session.run(async (engine) => {
      session.requireOpen(engine);
      return handler(engine, args);
    }));

  server.registerTool(
    "rosie_program_new",
    {
      title: "Start a new program",
      description: `Start a new, empty program, as OLP's New program: rails authoring on (welds come from the part's detected seams), pinned to the robot description OLP serves. It replaces the program this server has open (unsaved edits there are lost; save first).

Args:
  - name: program name.
  - model: robot model, a robot_description model id (for example rosie_1420_v1); default the connected cell's model, else OLP's first robot.
Then: rosie_program_add_waypoint / add_event / import_step, and rosie_program_save to put it on the cell.`,
      inputSchema: z.object({ name: z.string().min(1).max(120), model: z.string().max(80).optional(), response_format: responseFormat }).strict(),
      annotations: EDITS_REMOTE,
    },
    guarded(({ name, model, response_format }) => session.run(async (engine) => {
      const chosen = await programs.newProgram(ctx, engine, name, model);
      return answer(engine, ctx, response_format, `New program "${name}" for ${chosen}.`);
    })),
  );

  server.registerTool(
    "rosie_program_open",
    {
      title: "Open a program",
      description: `Open a program into this server: from the cell's program store (what the browser and both Decks open; rosie_olp_list_cell_programs lists them) or from a file OLP exported (Project > Export JSON). It replaces the program open here.

Args:
  - source: 'cell' (with program_id) or 'file' (with path).
  - revision: optional older store revision to open.
A program with a part needs its STEP for weld searches and planning: STEP bytes are never shared between clients, so pass step_path to rosie_program_import_step / rosie_program_plan if this server has not kept that part.`,
      inputSchema: z
        .object({
          source: z.enum(["cell", "file"]),
          program_id: z.string().min(1).max(200).optional(),
          revision: z.number().int().min(1).optional(),
          path: z.string().min(1).optional(),
          response_format: responseFormat,
        })
        .strict(),
      annotations: EDITS_REMOTE,
    },
    guarded(({ source, program_id, revision, path: file, response_format }) => session.run(async (engine) => {
      if (source === "cell" && !program_id) return failure("source 'cell' needs program_id (rosie_olp_list_cell_programs)");
      if (source === "file" && !file) return failure("source 'file' needs path");
      await programs.openProgram(ctx, engine, source === "cell" ? { source, programId: program_id!, revision } : { source, path: file! });
      return answer(engine, ctx, response_format, `Opened ${source === "cell" ? `${program_id} from the cell` : file}.`);
    })),
  );

  server.registerTool(
    "rosie_program_view",
    {
      title: "View the open program's spreadsheet",
      description: `The open program as OLP's spreadsheet: one row per node (weld, approach, transit, retract, home, move, dwell, io) with its id, op, label, geometry/process summary, plan status and enabled flag; for taught moves the motion type, TCP target (mm, degrees), TCP speed and joint speed %. Also the active part and its placement, the program's other parts, the workcell fixtures (stand, STEP obstacles), the kept plan (current or stale), problems, and where the program came from.

Read-only. Row ids are what every other program tool takes.`,
      inputSchema: z.object({ response_format: responseFormat }).strict(),
      annotations: READS,
    },
    onProgram(async (engine, { response_format }: { response_format: ResponseFormat }) => answer(engine, ctx, response_format, "")),
  );

  server.registerTool(
    "rosie_program_get_row",
    {
      title: "Every editable field of one row",
      description: `OLP's parameter dialog for one row: its groups and every field with path, label, kind (number, text, bool, select, scalar function...), unit, min/max/step, options, read-only reason and current value in display units. The path is what rosie_program_edit_row takes.

Read-only.`,
      inputSchema: z.object({ row_id: rowId, response_format: responseFormat }).strict(),
      annotations: READS,
    },
    onProgram(async (engine, { row_id, response_format }: { row_id: string; response_format: ResponseFormat }) => {
      const groups = engine.fields(row_id);
      const waypoints = engine.isWaypointWeld(row_id) ? engine.waypointWeldView(row_id) : null;
      return result(response_format, { ...groups, ...(waypoints ? { waypoint_weld: waypoints } : {}) }, () =>
        [
          `# ${groups.label} (${groups.op}) · \`${row_id}\``,
          ...groups.groups.map((group) =>
            [`\n## ${group.group}${group.note ? ` — ${group.note}` : ""}`, table(["Path", "Field", "Value", "Unit", "Range", "Notes"], group.fields.map((field) => [
              field.path, field.label, typeof field.value === "object" ? JSON.stringify(field.value) : field.value, field.unit ?? "",
              [field.min, field.max].some((v) => v !== undefined) ? `${field.min ?? ""}..${field.max ?? ""}` : "",
              [field.kind, field.readOnly ? `read-only: ${field.readOnly}` : "", field.options ? `options ${JSON.stringify(field.options)}` : "", field.hint ?? ""].filter(Boolean).join("; "),
            ]))].join("\n")),
          waypoints ? `\n${waypointTable(waypoints)}` : "",
        ].join("\n"),
      );
    }),
  );

  server.registerTool(
    "rosie_program_edit_row",
    {
      title: "Edit fields of one row",
      description: `Set one or more fields of a row by OLP field path (from rosie_program_get_row), in display units: e.g. a move's 'motion' ('joint'|'linear'|'circular'), 'speed_mm_s', 'speed_scale' (%), 'target.xyz_m.0' (mm; a Cartesian edit makes the target Cartesian and the planner resolves IK), 'target.rpy_rad.2' (degrees); a dwell's 'duration_s'; a weld's 'travel_speed_mm_s', 'weave.*', and for a rails weld 'geometry.span.start_s'/'end_s' and 'geometry.direction'; an approach/retract/transit's standoff lift; any row's 'label', 'comment', 'enabled'. A weld's torch angles are profiles: rosie_program_set_weld_profile. A weld's standoff_mm has no field in OLP's dialog (the angle search chooses it; a waypoint weld's is per point, rosie_program_edit_waypoint_weld).

OLP validates each value (ranges, read-only fields) and the whole program before it becomes a revision; edits apply in order, each one undo step, and the first refusal stops the rest and says which applied. Every edit makes the kept plan stale.`,
      inputSchema: z
        .object({
          row_id: rowId,
          edits: z.array(z.object({ path: z.string().min(1).max(200), value: editValue() }).strict()).min(1).max(50),
          response_format: responseFormat,
        })
        .strict(),
      annotations: EDITS,
    },
    onProgram(async (engine, { row_id, edits, response_format }: { row_id: string; edits: { path: string; value: unknown }[]; response_format: ResponseFormat }) => {
      const applied: string[] = [];
      for (const { path: field, value } of edits) {
        try {
          engine.edit(row_id, field, value);
          applied.push(field);
        } catch (error) {
          return failure(`${field} refused: ${error instanceof Error ? error.message : String(error)}${applied.length ? ` (applied before it: ${applied.join(", ")})` : " (nothing applied)"}`);
        }
      }
      return answer(engine, ctx, response_format, `Edited ${applied.join(", ")} on ${row_id}.`);
    }),
  );

  /** POST /robots/capture-model, then OLP's preview capture of these joints. */
  const capture = (engine: Engine, mode: "new" | "reteach" | "via", jointsDeg: number[], anchor: string | null): Promise<string> =>
    programs.capture(ctx, engine, mode, jointsDeg, anchor);

  server.registerTool(
    "rosie_program_add_waypoint",
    {
      title: "Add a taught waypoint (MoveJ / MoveL / MoveC)",
      description: `Record a taught waypoint from joint angles, as OLP records one in simulation: the TCP is OLP's FK of those joints on the capture model the server compiles (description + the cell's calibration pulled into the program). It goes after 'after_row_id' (default: the end, before a closing Home).

Args:
  - joints_deg: every joint of the program's robot in degrees, URDF order (rosie_program_view lists joint_names).
  - after_row_id: optional insertion point.
  - motion: 'joint' (MoveJ, default), 'linear' (MoveL), 'circular' (MoveC: then give it a via with rosie_program_reteach_waypoint as='via').
  - xyz_mm / rpy_deg: optional Cartesian target overriding the FK pose (the joints become the IK seed; the planner resolves IK).
  - speed_mm_s, speed_percent: optional TCP speed and joint speed %.
Returns the new row. Recording moves nothing; plan it with rosie_program_plan.`,
      inputSchema: z
        .object({
          joints_deg: z.array(z.number().finite()).min(1).max(9),
          after_row_id: rowId.optional(),
          motion: z.enum(["joint", "linear", "circular"]).default("joint"),
          xyz_mm: vec3.optional(),
          rpy_deg: vec3.optional(),
          speed_mm_s: z.number().positive().optional(),
          speed_percent: z.number().min(1).max(100).optional(),
          label: z.string().max(120).optional(),
          response_format: responseFormat,
        })
        .strict(),
      annotations: EDITS_REMOTE,
    },
    onProgram(async (engine, args: {
      joints_deg: number[]; after_row_id?: string; motion: string; xyz_mm?: number[]; rpy_deg?: number[];
      speed_mm_s?: number; speed_percent?: number; label?: string; response_format: ResponseFormat;
    }) => {
      const id = await programs.addWaypoint(ctx, engine, args.joints_deg, args.after_row_id ?? null, {
        motion: args.motion, xyzMm: args.xyz_mm, rpyDeg: args.rpy_deg, speedMmS: args.speed_mm_s, speedPercent: args.speed_percent, label: args.label,
      });
      return answer(engine, ctx, args.response_format, `Recorded waypoint ${id}.`);
    }),
  );

  server.registerTool(
    "rosie_program_reteach_waypoint",
    {
      title: "Reteach a waypoint or set its via pose",
      description: `Replace a taught waypoint's target with new joint angles ('target'), or record its MoveC via pose ('via', which makes it circular). Same capture rules as rosie_program_add_waypoint.`,
      inputSchema: z.object({ row_id: rowId, joints_deg: z.array(z.number().finite()).min(1).max(9), as: z.enum(["target", "via"]).default("target"), response_format: responseFormat }).strict(),
      annotations: EDITS_REMOTE,
    },
    onProgram(async (engine, { row_id, joints_deg, as, response_format }: { row_id: string; joints_deg: number[]; as: "target" | "via"; response_format: ResponseFormat }) => {
      await capture(engine, as === "via" ? "via" : "reteach", joints_deg, row_id);
      return answer(engine, ctx, response_format, `${as === "via" ? "Via pose recorded on" : "Retaught"} ${row_id}.`);
    }),
  );

  server.registerTool(
    "rosie_program_record_robot_pose",
    {
      title: "Teach a waypoint from where the robot is now",
      description: `Record a taught waypoint from the selected cell's MEASURED pose, as the browser's Record and the Deck's A do: OLP's teach rules (every described joint Home-valid, coordinates trusted, stationary; positions held within 0.00002 rad for 100 ms when the drives report no velocity), the cell's own calibration fetched for a machine that binds a robot description, OLP's capture model and FK, then recordMove. Reads the robot; moves nothing. Jog it there first (browser or Deck), release, then call this.

Args:
  - mode: 'new' (a waypoint after after_row_id, default the end), 'reteach' (replace row_id's target) or 'via' (row_id's MoveC via pose).
  - motion, speed_mm_s, speed_percent, label: optional, as rosie_program_add_waypoint.
Refused, as in the browser, while the selected cell's session is loaded, staged, playing or paused ("Editing is locked while the joint daemon is loaded on this program (STOP to release)."): Stop releases it. Other refusals are OLP's own sentences ("Release jog and wait for fresh stationary feedback before recording", "Adopt the matching robot description before recording a waypoint", "Pull machine calibration before recording"); rosie_program_robot_settings fixes the last two.`,
      inputSchema: z
        .object({
          mode: z.enum(["new", "reteach", "via"]).default("new"),
          row_id: rowId.optional(),
          after_row_id: rowId.optional(),
          motion: z.enum(["joint", "linear", "circular"]).optional(),
          speed_mm_s: z.number().positive().optional(),
          speed_percent: z.number().min(1).max(100).optional(),
          label: z.string().max(120).optional(),
          response_format: responseFormat,
        })
        .strict(),
      annotations: { readOnlyHint: false, destructiveHint: false, idempotentHint: false, openWorldHint: true },
    },
    onProgram(async (engine, args: { mode: "new" | "reteach" | "via"; row_id?: string; after_row_id?: string; motion?: string; speed_mm_s?: number; speed_percent?: number; label?: string; response_format: ResponseFormat }) => {
      if (args.mode !== "new" && !args.row_id) return failure(`mode '${args.mode}' needs row_id (the waypoint to ${args.mode === "via" ? "give a via pose" : "reteach"})`);
      const { id, cellId, joints } = await programs.recordRobotPose(ctx, engine, args.mode, args.row_id ?? null, args.after_row_id ?? null, {
        motion: args.motion, speedMmS: args.speed_mm_s, speedPercent: args.speed_percent, label: args.label,
      });
      return answer(engine, ctx, args.response_format, `${args.mode === "new" ? "Recorded" : args.mode === "via" ? "Via pose recorded on" : "Retaught"} ${id} from ${cellId}'s measured pose (${joints}).`);
    }),
  );

  server.registerTool(
    "rosie_program_robot_settings",
    {
      title: "The program's robot settings against the connected cell",
      description: `The program's robot model, the description it is pinned to, the cell settings pulled into it, and OLP's review of any drift from the robot description OLP serves now. Actions:
  - 'show' (default): read only.
  - 'pull': fetch the selected cell's settings and calibration (GET dense-execution/cell) and pull them into the program (Workspace > Cell > Pull cell settings); needed before recording from a machine that binds a description.
  - 'adopt': accept the current robot description after reviewing its changes (OLP refuses when there is no drift); plan again afterwards.
Pull and adopt are program edits (undoable), saved with the program.`,
      inputSchema: z.object({ action: z.enum(["show", "pull", "adopt"]).default("show"), response_format: responseFormat }).strict(),
      annotations: { readOnlyHint: false, destructiveHint: false, idempotentHint: true, openWorldHint: true },
    },
    onProgram(async (engine, { action, response_format }: { action: "show" | "pull" | "adopt"; response_format: ResponseFormat }) => {
      const { headline, settings } = await programs.robotSettings(ctx, engine, action);
      return result(response_format, settings, () => [
        headline,
        `# Robot settings: ${settings.model}`,
        `- **Pinned description**: \`${settings.pinned ?? "none"}\``,
        `- **Cell settings**: ${settings.cell ? json(settings.cell) : "none pulled"}`,
        settings.review ? `- **Drift**: ${settings.review.title} — ${settings.review.explanation}\n${json(settings.review.rows)}` : "- **Drift**: none",
      ].filter(Boolean).join("\n"));
    }),
  );

  server.registerTool(
    "rosie_program_add_event",
    {
      title: "Add a Dwell, IO or Home row",
      description: `Add a row with OLP's defaults: 'dwell' (0.5 s), 'io' (revpi:1 ON) after 'after_row_id' (default the end), or 'home' (always closes the program). Change its fields with rosie_program_edit_row (duration_s; the IO channel and state). IO rows plan only with a cell connected: they are scheduled against the cell's outputs.`,
      inputSchema: z.object({ kind: z.enum(["dwell", "io", "home"]), after_row_id: rowId.optional(), response_format: responseFormat }).strict(),
      annotations: EDITS,
    },
    onProgram(async (engine, { kind, after_row_id, response_format }: { kind: "dwell" | "io" | "home"; after_row_id?: string; response_format: ResponseFormat }) => {
      engine.select(after_row_id ?? null);
      const id = engine.addEvent(kind);
      return answer(engine, ctx, response_format, `Added ${kind} ${id}.`);
    }),
  );

  server.registerTool(
    "rosie_program_remove_row",
    {
      title: "Remove a row",
      description: `Delete a row (OLP refuses rows that move with their weld, e.g. an approach, and says why). Undoable with rosie_program_undo.`,
      inputSchema: z.object({ row_id: rowId, response_format: responseFormat }).strict(),
      annotations: EDITS,
    },
    onProgram(async (engine, { row_id, response_format }: { row_id: string; response_format: ResponseFormat }) => {
      const row = engine.view().rows.find((candidate) => candidate.id === row_id);
      if (!row) return failure(`no row ${row_id}`);
      if (row.deleteBlocked) return failure(`OLP will not delete ${row_id}: ${row.deleteBlocked}`);
      engine.remove(row_id);
      return answer(engine, ctx, response_format, `Removed ${row_id}.`);
    }),
  );

  server.registerTool(
    "rosie_program_move_row",
    {
      title: "Move a row up or down",
      description: `Reorder a row by 'steps' places (negative = up). OLP refuses moves that would separate a weld from its approach/retract or put a row after Home, and says why.`,
      inputSchema: z.object({ row_id: rowId, steps: z.number().int().min(-100).max(100).refine((v) => v !== 0, "steps must be nonzero"), response_format: responseFormat }).strict(),
      annotations: EDITS,
    },
    onProgram(async (engine, { row_id, steps, response_format }: { row_id: string; steps: number; response_format: ResponseFormat }) => {
      const delta = steps < 0 ? -1 : 1;
      for (let i = 0; i < Math.abs(steps); i++) {
        const row = engine.view().rows.find((candidate) => candidate.id === row_id);
        if (!row) return failure(`no row ${row_id}`);
        const blocked = delta < 0 ? row.upBlocked : row.downBlocked;
        if (blocked) return i === 0 ? failure(`OLP will not move ${row_id}: ${blocked}`) : answer(engine, ctx, response_format, `Moved ${row_id} ${i} place(s); further: ${blocked}.`);
        engine.move(row_id, delta);
      }
      return answer(engine, ctx, response_format, `Moved ${row_id} ${Math.abs(steps)} place(s) ${delta < 0 ? "up" : "down"}.`);
    }),
  );

  server.registerTool(
    "rosie_program_enable_row",
    {
      title: "Enable or disable a row",
      description: `Disable a row (it stays in the program and is skipped by planning) or enable it again.`,
      inputSchema: z.object({ row_id: rowId, enabled: z.boolean(), response_format: responseFormat }).strict(),
      annotations: EDITS,
    },
    onProgram(async (engine, { row_id, enabled, response_format }: { row_id: string; enabled: boolean; response_format: ResponseFormat }) => {
      engine.enable(row_id, enabled);
      return answer(engine, ctx, response_format, `${enabled ? "Enabled" : "Disabled"} ${row_id}.`);
    }),
  );

  server.registerTool(
    "rosie_program_set_plan_speed",
    {
      title: "Set the program's plan speed",
      description: `The spreadsheet's Plan speed: 1-100% of the planner's whole trajectory (taught and connecting moves; dwell times stay in seconds). Saved with the program; replan after changing it.`,
      inputSchema: z.object({ percent: z.number().min(1).max(100), response_format: responseFormat }).strict(),
      annotations: EDITS,
    },
    onProgram(async (engine, { percent, response_format }: { percent: number; response_format: ResponseFormat }) => {
      engine.setSpeedScale(percent);
      return answer(engine, ctx, response_format, `Plan speed ${percent}%.`);
    }),
  );

  server.registerTool(
    "rosie_program_undo",
    {
      title: "Undo or redo program edits",
      description: `Step the open program's history back ('undo') or forward ('redo') by 'steps' edits, as OLP's undo/redo.`,
      inputSchema: z.object({ direction: z.enum(["undo", "redo"]).default("undo"), steps: z.number().int().min(1).max(50).default(1), response_format: responseFormat }).strict(),
      annotations: EDITS,
    },
    onProgram(async (engine, { direction, steps, response_format }: { direction: "undo" | "redo"; steps: number; response_format: ResponseFormat }) => {
      let done = 0;
      for (; done < steps; done++) {
        const view = engine.view();
        if (direction === "undo" ? !view.canUndo : !view.canRedo) break;
        if (direction === "undo") engine.undo();
        else engine.redo();
      }
      return answer(engine, ctx, response_format, `${direction} ${done} step(s)${done < steps ? " (no more history)" : ""}.`);
    }),
  );

  server.registerTool(
    "rosie_program_save",
    {
      title: "Save the open program",
      description: `Save the open program to the cell's program store (what the browser and both Decks open) or to a file (OLP's Project > Export JSON).

The cell store is revision-checked: the save names the revision this program was opened from, and the cell refuses a stale one (409 program_conflict: someone saved since) instead of overwriting; nothing is merged. Then either reopen theirs (rosie_program_open) or save_as_copy (a new program). It refuses 423 program_executing while the robot holds this program. make_current switches the cell's current program (what the Decks show); default false.`,
      inputSchema: z
        .object({
          target: z.enum(["cell", "file"]).default("cell"),
          path: z.string().min(1).optional(),
          save_as_copy: z.boolean().default(false).describe("Save as a new program (new identity) instead of a new revision"),
          make_current: z.boolean().default(false),
          response_format: responseFormat,
        })
        .strict(),
      annotations: EDITS_REMOTE,
    },
    onProgram(async (engine, { target, path: file, save_as_copy, make_current, response_format }: { target: "cell" | "file"; path?: string; save_as_copy: boolean; make_current: boolean; response_format: ResponseFormat }) => {
      if (target === "file" && !file && !session.link?.path) return failure("target 'file' needs path");
      let saved: programs.SaveResult;
      try {
        saved = await programs.saveProgram(ctx, engine, { target, path: file, saveAsCopy: save_as_copy, makeCurrent: make_current });
      } catch (error) {
        if (error instanceof programs.ProgramRefusal && error.status) {
          const conflict = error.status === 409 ? " Someone saved this program on the cell since it was opened here: reopen it (rosie_program_open source=cell) or save_as_copy." : "";
          return failure(`${error.message.replace(/ Someone saved this program.*$/, "")}${conflict}`, { status: error.status, body: error.body });
        }
        throw error;
      }
      if (saved.kind === "file") return answer(engine, ctx, response_format, `Saved to ${saved.path}.`);
      return answer(engine, ctx, response_format, `Saved ${saved.id} to the cell as revision ${saved.revision}.${saved.refusedCurrent ? ` Not made current: ${JSON.stringify(saved.refusedCurrent)}` : ""}`);
    }),
  );

  /** OLP's Import STEP (main.ts importStepBytes, kind "weld"): every import after the first adds a part. */
  const importStep = async (engine: Engine, file: string, response_format: ResponseFormat): Promise<CallToolResult> => {
    const imported = await programs.importStep(ctx, engine, file);
    const detection = imported.railsAuthoring
      ? ` ${imported.joints} weld joints, ${imported.seams} seams detected.`
      : " This is a classic program: rails seams are not detected (start a New program for rails welds).";
    const added = imported.added ? ` Part ${imported.parts} of the program, now the active one; the other parts keep their welds and plan as obstacles (rosie_program_switch_part).` : "";
    return answer(engine, ctx, response_format, `Imported ${imported.filename}: ${imported.edges} edges.${detection}${added}`);
  };

  const importDescription = `CadQuery reads its topology (POST /cadquery/topology), OLP's appendCadQueryTopology adds the part and centres it on the cell's work frame (move it with rosie_program_place_part), and for a rails program the seam worker detects the weld joints and their default seams (POST /seam detect_joints). The STEP is kept by this server under its fingerprint for later searches and plans.

A program can hold several parts: every import after the first ADDS a part and makes it the active one (the one edited and planned); the earlier parts keep their welds and go to the planner as obstacles. rosie_program_list_parts, rosie_program_switch_part, rosie_program_remove_part.

Args:
  - path: absolute path to a .step/.stp file (at most 8 MiB, OLP's limit).
Then: rosie_program_list_seams, rosie_program_search_seam, rosie_program_add_weld.`;

  server.registerTool(
    "rosie_program_import_step",
    {
      title: "Import a STEP part and detect its weld seams",
      description: `Import a STEP file as a part of the program, exactly as OLP's Import STEP: ${importDescription}`,
      inputSchema: z.object({ path: z.string().min(1), response_format: responseFormat }).strict(),
      annotations: EDITS_REMOTE,
    },
    onProgram(async (engine, { path: file, response_format }: { path: string; response_format: ResponseFormat }) => importStep(engine, file, response_format)),
  );

  server.registerTool(
    "rosie_program_add_part",
    {
      title: "Add another STEP part to the program",
      description: `Add a STEP part beside the program's existing parts: the same as rosie_program_import_step (OLP has one Import STEP; a second import adds a part). ${importDescription}`,
      inputSchema: z.object({ path: z.string().min(1), response_format: responseFormat }).strict(),
      annotations: EDITS_REMOTE,
    },
    onProgram(async (engine, { path: file, response_format }: { path: string; response_format: ResponseFormat }) => importStep(engine, file, response_format)),
  );

  server.registerTool(
    "rosie_program_list_parts",
    {
      title: "List the program's parts",
      description: `The program's parts: id, STEP file, fingerprint, which is ACTIVE (edited and planned), its weld and row counts, its placement in the work frame (mm, degrees), and whether this server keeps its STEP. A plan carries only the active part's welds and STEP; every other part goes to the planner as a mesh obstacle.

Read-only.`,
      inputSchema: z.object({ response_format: responseFormat }).strict(),
      annotations: READS,
    },
    onProgram(async (engine, { response_format }: { response_format: ResponseFormat }) => {
      const parts = await Promise.all(engine.partsView().map(async (part) => ({
        ...part,
        step_kept: await session.getSource(part.fingerprint).then(() => true, () => false),
      })));
      return result(response_format, { count: parts.length, parts }, () => parts.length
        ? table(["Part id", "File", "Active", "Welds", "Rows", "Offset mm", "Rpy °", "STEP kept"], parts.map((p) => [
          p.id, p.filename, p.active ? "ACTIVE" : "", p.welds, p.rows, p.placement ? mm(p.placement.offsetFromWorkFrameMm) : "", p.placement ? mm(p.placement.rotationRpyDeg) : "", p.step_kept ? "yes" : "no (step_path)",
        ]))
        : "The program has no part: rosie_program_import_step.");
    }),
  );

  server.registerTool(
    "rosie_program_switch_part",
    {
      title: "Make another part the active one",
      description: `Select another of the program's parts for editing and planning, as clicking it in OLP's program tree (activateWorkpiece): its welds, seams and rows become the spreadsheet; the part that was active keeps its own and becomes an obstacle. Its STEP must be at hand (STEP bytes are never shared between clients): kept by this server, or step_path to the same file. The searched seams of this session are dropped, as the browser's rails mode resets.

Args: part_id (rosie_program_list_parts); step_path (optional).`,
      inputSchema: z.object({ part_id: z.string().min(1).max(200), step_path: z.string().optional(), response_format: responseFormat }).strict(),
      annotations: EDITS,
    },
    onProgram(async (engine, { part_id, step_path, response_format }: { part_id: string; step_path?: string; response_format: ResponseFormat }) => {
      const { part, switched } = await programs.switchPart(ctx, engine, part_id, step_path);
      return answer(engine, ctx, response_format, switched ? `${part.filename} is active for editing and planning.` : `${part.filename} is already the active part.`);
    }),
  );

  server.registerTool(
    "rosie_program_remove_part",
    {
      title: "Delete the active part",
      description: `Delete the ACTIVE part, as OLP's Delete part: its geometry, edges, seams, the welds authored on it and their rows leave the program; the other parts stay. The program then has no active part until rosie_program_switch_part or an import. One undo step (rosie_program_undo).`,
      inputSchema: z.object({ response_format: responseFormat }).strict(),
      annotations: { ...EDITS, destructiveHint: true },
    },
    onProgram(async (engine, { response_format }: { response_format: ResponseFormat }) => {
      const filename = engine.removePart();
      session.forgetPart();
      return answer(engine, ctx, response_format, `Removed ${filename} from the program.`);
    }),
  );

  server.registerTool(
    "rosie_program_list_fixtures",
    {
      title: "List the workcell fixtures",
      description: `Everything a plan clears besides the robot and the active part, in the cell's world frame: the program's workcell fixtures (a box stand, imported STEP fixtures) and its other parts. Each with id, shape (box size in mm, or STEP file), position (mm), rotation (roll, pitch, yaw °), whether it is the cell's placeholder, and for a STEP whether its mesh is tessellated in this session. Also the cell's own default (rosie_1420_v1: a 500 × 500 × 1000 mm box at 1000, 1000, 0 mm, unmeasured; other cells none).

Read-only.`,
      inputSchema: z.object({ response_format: responseFormat }).strict(),
      annotations: READS,
    },
    onProgram(async (engine, { response_format }: { response_format: ResponseFormat }) => {
      const view = engine.fixturesView();
      const fixtures = view.fixtures.map((fixture) => ({
        ...fixture,
        mesh: fixture.shape.kind === "step" ? (session.meshes.has(fixture.shape.fingerprint.toLowerCase()) ? "tessellated" : "not yet: tessellated from the kept STEP at plan") : null,
      }));
      return result(response_format, { fixtures, cell_default: view.cellDefault }, () => [
        fixtures.length ? fixtures.map((f) => `- ${f.kind === "part" ? "(other part) " : ""}${fixtureLine(f, session)}`).join("\n") : "No workcell fixtures: the planner clears only the robot and the active part.",
        `\nCell default: ${view.cellDefault.length ? view.cellDefault.map((f) => fixtureLine(f, session)).join("; ") : "none"}`,
      ].join("\n"));
    }),
  );

  const fixtureFields = z.object({
    sx: z.number(), sy: z.number(), sz: z.number(), x: z.number(), y: z.number(), z: z.number(), rx: z.number(), ry: z.number(), rz: z.number(),
  }).partial().strict();

  server.registerTool(
    "rosie_program_edit_fixture",
    {
      title: "Add, edit, replace or remove a workcell fixture",
      description: `The Stand card and the workspace menu's fixture actions, each one program edit (undoable; the plan is invalidated):
  - 'add_stand': a 500 × 500 × 1000 mm box stand at 1000, 1000, 0 mm, if the program has none ('stand').
  - 'edit' (fixture_id, fields): any of sx, sy, sz (box size, 1 to 5000 mm), x, y, z (position of the footprint's centre in the cell's world frame, ±5000 mm), rx, ry, rz (roll, pitch, yaw, ±360°). Out-of-range values are refused, not clamped. A STEP fixture has no size.
  - 'replace_with_step' (fixture_id, path): the fixture becomes that STEP's body, tessellated by CadQuery (POST /cadquery/topology tessellate), pose and id kept.
  - 'add_step_fixture' (path): import a STEP as an extra fixture or obstacle, placed by its own origin (move it with 'edit').
  - 'cell_default' (fixture_id): put the cell's placeholder back (a 500 mm box where the cell has none).
  - 'remove' (fixture_id): it is no longer included in collision checking.
STEP files are kept by this server under their fingerprint, so the mesh can be rebuilt for later plans.`,
      inputSchema: z
        .object({
          action: z.enum(["add_stand", "edit", "replace_with_step", "add_step_fixture", "cell_default", "remove"]),
          fixture_id: z.string().min(1).max(200).optional(),
          fields: fixtureFields.optional(),
          path: z.string().min(1).optional(),
          response_format: responseFormat,
        })
        .strict(),
      annotations: EDITS_REMOTE,
    },
    onProgram(async (engine, args: { action: FixtureEdit["action"]; fixture_id?: string; fields?: Partial<Record<FixtureField, number>>; path?: string; response_format: ResponseFormat }) => {
      const done = await programs.editFixture(ctx, engine, { action: args.action, fixtureId: args.fixture_id, fields: args.fields, path: args.path });
      const note = done.note ? ` · ${done.note}` : "";
      const headline = !done.changed
        ? `The program already has a stand (${done.id}); edit it with action 'edit'.`
        : args.action === "remove" ? `Removed ${done.id}: it is no longer included in collision checking.` : `${args.action.replace(/_/g, " ")}: ${done.id}${note}.`;
      return answer(engine, ctx, args.response_format, headline);
    }),
  );

  server.registerTool(
    "rosie_program_place_part",
    {
      title: "Place the part in the cell",
      description: `Set the part's pose in the cell's work frame (the 1400's table top, the 1420's floor), as OLP's Move part fields: the part's base centre (bounding-box x/y centre, lowest z) goes to offset_mm, turned by rotation_deg (roll, pitch, yaw). Import centres it at [0,0,0]. The browser's bounds apply: ±5000 mm, ±360°. Moving the part invalidates the plan; replan afterwards.`,
      inputSchema: z
        .object({
          offset_mm: z.array(z.number().min(-5000).max(5000)).length(3),
          rotation_deg: z.array(z.number().min(-360).max(360)).length(3).default([0, 0, 0]),
          response_format: responseFormat,
        })
        .strict(),
      annotations: EDITS,
    },
    onProgram(async (engine, { offset_mm, rotation_deg, response_format }: { offset_mm: number[]; rotation_deg: number[]; response_format: ResponseFormat }) => {
      await session.installRobots(engine);
      engine.placePart(offset_mm, rotation_deg);
      return answer(engine, ctx, response_format, "Part placed.");
    }),
  );

  server.registerTool(
    "rosie_program_list_seams",
    {
      title: "List the part's detected weld seams",
      description: `The weld joints and seams the seam worker detected on the program's part: seam id, label, joint type and weld symbol, length, closed or open, whether a weld row already exists for it, and the angle search's verdict if it has been searched in this session.

Args: step_path (optional) when this server has no copy of the part's STEP.`,
      inputSchema: z.object({ step_path: z.string().optional(), response_format: responseFormat }).strict(),
      annotations: { ...READS, openWorldHint: true },
    },
    onProgram(async (engine, { step_path, response_format }: { step_path?: string; response_format: ResponseFormat }) => {
      const seams = await programs.listSeams(ctx, engine, step_path);
      return result(response_format, { count: seams.length, seams }, () =>
        seams.length
          ? table(["Seam", "Label", "Joint", "Type", "Length", "Weld rows", "Search"], seams.map((s) => [s.seam_id, s.label, s.joint_id, `${s.type ?? ""} ${s.weld_symbol ?? ""}`, `${s.length_mm} mm${s.closed ? " (closed)" : ""}`, s.weld_rows.join(", ") || "-", s.searched ? `${s.searched.state}, ${s.searched.clear_percent}% clear, ${s.searched.runs} run(s)` : "not searched"]))
          : "The seam worker detected no weld seams on this part.",
      );
    }),
  );

  server.registerTool(
    "rosie_program_search_seam",
    {
      title: "Search torch angles for one seam",
      description: `Run rails mode's angle search on one detected seam with this session's fitted torch and angle search (rosie_program_rails_settings; default the shipped Rosie 1400 link_6 gun and the 'steady' search): both travel directions (plan_torch_path), the better coverage kept, then the clearance check at the chosen angles (check_torch_fits). Answers whether the torch clears the whole seam ('clear'), only some runs ('partial': each clear run can become its own weld) or none ('blocked'), with the chosen direction and standoff. Seconds to minutes on the planner host; the answer is kept for rosie_program_add_weld. Moves nothing.

Args: seam_id; fresh (search again even if searched); step_path when the part's STEP is not kept here.`,
      inputSchema: z.object({ seam_id: z.string().min(1).max(100), fresh: z.boolean().default(false), step_path: z.string().optional(), response_format: responseFormat }).strict(),
      annotations: { readOnlyHint: true, destructiveHint: false, idempotentHint: true, openWorldHint: true },
    },
    onProgram(async (engine, { seam_id, fresh, step_path, response_format }: { seam_id: string; fresh: boolean; step_path?: string; response_format: ResponseFormat }) => {
      const data = await programs.searchSeam(ctx, engine, seam_id, { fresh, stepPath: step_path });
      return result(response_format, data, () =>
        [
          `# ${seam_id}: ${data.state.toUpperCase()} · ${data.clear_percent}% clear (${data.direction ?? "forward"}, standoff ${data.standoff_mm} mm)`,
          `- Seam ${data.seam_length_mm} mm · usable ${data.usable_mm} mm · blocked ${data.blocked_mm} mm`,
          data.state === "partial" ? `- Clear runs: ${data.runs.map((r) => `#${r.run} s=${r.span.start_s.toFixed(3)}..${r.span.end_s.toFixed(3)}`).join(", ")}. rosie_program_add_weld adds each run you name as its own weld.` : "",
        ].filter(Boolean).join("\n"),
      );
    }),
  );

  server.registerTool(
    "rosie_program_add_weld",
    {
      title: "Add (accept) a rails weld for a seam",
      description: `Turn a searched seam into a weld row, as accepting it in OLP's Rails welds card: OLP's seamWeldNodeFor builds the node from the seam, the search's angles, direction and standoff, and the part's fingerprint; the first weld gets its approach and retract, later welds a transit. Reaccepting a seam updates its geometry and profiles but keeps what was authored on the row (enabled, standoff, process).

Args:
  - seam_id (searched automatically if needed).
  - choice: 'accept' (weld as searched) or 'trim' (weld only the longest clear run).
  - runs: for a 'partial' seam, which clear runs (indices from rosie_program_search_seam) become welds; default all of them. Each run is re-checked for clearance.
Then plan with rosie_program_plan.`,
      inputSchema: z
        .object({
          seam_id: z.string().min(1).max(100),
          choice: z.enum(["accept", "trim"]).default("accept"),
          runs: z.array(z.number().int().min(0).max(100)).max(100).optional(),
          step_path: z.string().optional(),
          response_format: responseFormat,
        })
        .strict(),
      annotations: EDITS_REMOTE,
    },
    onProgram(async (engine, { seam_id, choice, runs, step_path, response_format }: { seam_id: string; choice: "accept" | "trim"; runs?: number[]; step_path?: string; response_format: ResponseFormat }) => {
      const { added, notes } = await programs.addWeld(ctx, engine, seam_id, { choice, runs, stepPath: step_path });
      return answer(engine, ctx, response_format, `Added ${added.join(", ")}.${notes.length ? ` ${notes.join("; ")}.` : ""}`);
    }),
  );

  server.registerTool(
    "rosie_program_rails_settings",
    {
      title: "The fitted torch and the angle search",
      description: `The two rails parameter windows of OLP (Plan presets): the FITTED TORCH (a fact about the cell, sent with every seam search, clearance check and plan) and the ANGLE SEARCH (how far the search may move the drawn angles and what standoff departure costs; sent only to a search). Session state of this server, as in the browser: not saved in the program; an accepted weld keeps the values it was searched with.

Actions:
  - 'show' (default): presets, every value with unit and range, which differ from the preset.
  - 'set': torch_preset ('rosie_link6_gun' the shipped fallback, 'mig_gas_nozzle_16') and/or torch values {body_radius_mm 1-40, body_length_mm 20-600, standoff_min_mm / standoff_max_mm / default_standoff_mm 0.5-100}; search_preset ('steady') and/or search values {work_deviation_deg 0-90, travel_deviation_deg 0-90, standoff_cost_deg_per_mm 0.1-30}. A preset loads its values first; values given alone change the current ones. OLP's parameter model refuses out-of-range values (never clamps), and a torch whose standoff range crosses, whose barrel is not longer than its furthest standoff, or whose default standoff is outside its range is refused.
  - 'reset': back to the shipped presets.
A change drops this session's searched seams (search again); welds already accepted are untouched.`,
      inputSchema: z
        .object({
          action: z.enum(["show", "set", "reset"]).default("show"),
          torch_preset: z.string().min(1).max(100).optional(),
          torch: z.record(z.string(), z.number().finite()).optional(),
          search_preset: z.string().min(1).max(100).optional(),
          search: z.record(z.string(), z.number().finite()).optional(),
          response_format: responseFormat,
        })
        .strict(),
      annotations: { readOnlyHint: false, destructiveHint: false, idempotentHint: true, openWorldHint: false },
    },
    guarded((args: { action: "show" | "set" | "reset"; torch_preset?: string; torch?: Record<string, number>; search_preset?: string; search?: Record<string, number>; response_format: ResponseFormat }) => session.run(async (engine) => {
      let headline = "";
      let settings: RailsSettings;
      if (args.action === "show") {
        settings = session.rails(engine);
      } else {
        const changed = session.setRails(engine, args.action === "reset" ? "reset" : {
          torchPreset: args.torch_preset, torch: args.torch, searchPreset: args.search_preset, search: args.search,
        });
        settings = changed.settings;
        headline = `${args.action === "reset" ? "Back to the shipped presets." : "Settings changed."}${changed.dropped ? ` ${changed.dropped} searched seam(s) dropped: search them again with the new values.` : ""}`;
      }
      const sheet = (title: string, side: RailsSettings["torch"]) => [
        `## ${title}: ${side.preset.label} (\`${side.preset.id}\`)${side.modified.length ? ` · modified: ${side.modified.join(", ")}` : ""}`,
        table(["Key", "Field", "Value", "Preset", "Range"], side.fields.map((f) => [f.key, f.label, `${f.value} ${f.unit}`, `${f.presetValue} ${f.unit}`, f.min !== null ? `${f.min}..${f.max} ${f.unit}` : ""])),
      ].join("\n");
      return result(args.response_format, { ...settings, dropped_searches: headline.includes("dropped") }, () => [
        headline,
        sheet("Fitted torch", settings.torch),
        sheet("Angle search", settings.search),
        `\nPresets: torch ${settings.presets.torch.map((p) => `\`${p.id}\``).join(", ")}; search ${settings.presets.search.map((p) => `\`${p.id}\``).join(", ")}.`,
      ].filter(Boolean).join("\n\n"));
    })),
  );

  server.registerTool(
    "rosie_program_set_weld_profile",
    {
      title: "Set a rails weld's work or travel angle profile",
      description: `A rails (seam) weld's torch angle along the weld, as the parameter dialog's profile editor writes it: either one constant angle, or control points [s, degrees] with s the normalised arc length along the weld's span (first point s=0, last s=1, strictly increasing), linear in between (OLP's fromPoints; a flat set is written as a constant). Degrees within the editor's -90..90. OLP refuses bspline/samples profiles (not editable here) and validates the program before it becomes a revision. The weld planner samples the profile and may trade inside the weld's orientation band. One undo step; replan afterwards.

Args: row_id (a rails weld), angle 'work' | 'travel', and constant_deg OR points.`,
      inputSchema: z
        .object({
          row_id: rowId,
          angle: z.enum(["work", "travel"]),
          constant_deg: z.number().finite().optional(),
          points: z.array(z.object({ s: z.number().min(0).max(1), deg: z.number().finite() }).strict()).min(2).max(64).optional(),
          response_format: responseFormat,
        })
        .strict(),
      annotations: EDITS,
    },
    onProgram(async (engine, { row_id, angle, constant_deg, points, response_format }: { row_id: string; angle: "work" | "travel"; constant_deg?: number; points?: { s: number; deg: number }[]; response_format: ResponseFormat }) => {
      if ((constant_deg === undefined) === (points === undefined)) return failure("give exactly one of constant_deg or points");
      const { path: field, fn } = engine.weldProfile(row_id, angle, points ?? [{ s: 0, deg: constant_deg! }, { s: 1, deg: constant_deg! }]);
      engine.edit(row_id, field, fn);
      return answer(engine, ctx, response_format, `${angle} angle of ${row_id} is now ${JSON.stringify(fn)}.`);
    }),
  );

  const waypointPoint = z.object({
    xyz_mm: vec3.describe("Work point in the PART frame, mm"),
    rpy_deg: vec3.describe("Torch orientation, URDF roll, pitch, yaw in degrees, part frame; tool +Z points into the work ([180, 0, 0] is straight down)"),
    continuity: z.enum(["c0", "c1", "c2"]).optional().describe("How the path passes through an interior point: c0 corner, c1 smooth turn, c2 spline knot"),
    standoff_mm: z.number().finite().optional().describe("Contact tip to work point, 0.5-100 mm; default the program's process standoff"),
  }).strict();
  type PointArg = { xyz_mm: number[]; rpy_deg: number[]; continuity?: "c0" | "c1" | "c2"; standoff_mm?: number };
  const toInput = (point: PointArg): WaypointInput => ({ xyzMm: point.xyz_mm, rpyDeg: point.rpy_deg, continuity: point.continuity, standoffMm: point.standoff_mm });

  server.registerTool(
    "rosie_program_add_waypoint_weld",
    {
      title: "Add a waypoint, line or arc weld from torch poses",
      description: `A weld from torch poses placed by hand, as OLP's Waypoint welds / Line welds / Arc welds tools: each point is a work point in the PART's frame (mm; it moves with the part), the torch orientation there (roll, pitch, yaw °; tool +Z into the work) and the standoff the torch is held at; interior points say how the path passes them (c0 corner, c1 smooth, c2 spline). 'waypoint': 2 or more points, the spline rule; 'line': exactly 2; 'arc': exactly 3 non-collinear points, the circle through them. Consecutive points must be more than 1 µm apart. The weld gets OLP's next label ("Waypoint weld N", "Line N", "Arc N"), the program's process travel speed and weave, and its approach and retract. start_from_row_id ties its first point to the end of another such weld (it follows that weld's end). The planner receives it lowered to a seam. Needs a rails program with a part.

The classic posed-polyline weld is legacy in OLP (its planner refuses it beside taught moves) and is not offered.`,
      inputSchema: z
        .object({
          tool: z.enum(["waypoint", "line", "arc"]).default("waypoint"),
          points: z.array(waypointPoint).min(2).max(200),
          label: z.string().min(1).max(120).optional(),
          travel_speed_mm_s: z.number().positive().optional(),
          start_from_row_id: rowId.optional(),
          response_format: responseFormat,
        })
        .strict(),
      annotations: EDITS,
    },
    onProgram(async (engine, args: { tool: WeldTool; points: PointArg[]; label?: string; travel_speed_mm_s?: number; start_from_row_id?: string; response_format: ResponseFormat }) => {
      const id = engine.addWaypointWeld(args.tool, args.points.map(toInput), args.start_from_row_id ?? null);
      const edits: [string, unknown][] = [];
      if (args.label) edits.push(["label", args.label]);
      if (args.travel_speed_mm_s !== undefined) edits.push(["travel_speed_mm_s", args.travel_speed_mm_s]);
      for (const [field, value] of edits) {
        try {
          engine.edit(id, field, value);
        } catch (error) {
          return failure(`Added ${id}, but ${field} was refused: ${error instanceof Error ? error.message : String(error)}`, { row_id: id });
        }
      }
      const view = engine.waypointWeldView(id);
      return result(args.response_format, { row_id: id, waypoint_weld: view, program: describeProgram(engine, ctx) }, () =>
        `Added ${view.label} (\`${id}\`).\n\n${waypointTable(view)}\n\n${spreadsheet(engine, ctx)}`);
    }),
  );

  server.registerTool(
    "rosie_program_edit_waypoint_weld",
    {
      title: "Edit a waypoint, line or arc weld's poses",
      description: `One edit of a hand-placed weld, through OLP's Waypoint weld tool (WaypointSession), one undo step. Points are indexed from 0 (rosie_program_get_row shows them). Ops:
  - 'set_points' (points): replace every pose (a line keeps 2, an arc 3).
  - 'add_point' (point, after_index default the last): a waypoint weld only.
  - 'move_point' (index, xyz_mm and/or rpy_deg): a new pose; a first point tied to another weld's end takes the orientation only.
  - 'reorder_point' (index, by -1|1).
  - 'remove_point' (index): a waypoint weld only.
  - 'set_standoff' (standoff_mm 0.5-100, index; every point when index is omitted).
  - 'set_continuity' (continuity c0|c1|c2, index; every point when omitted).
  - 'set_start_from' (start_from_row_id, or null to untie).
OLP's program validator refuses an edit that leaves the weld invalid (coincident points, an arc on a line) and nothing is changed.`,
      inputSchema: z
        .object({
          row_id: rowId,
          op: z.enum(["set_points", "add_point", "move_point", "reorder_point", "remove_point", "set_standoff", "set_continuity", "set_start_from"]),
          points: z.array(waypointPoint).min(2).max(200).optional(),
          point: waypointPoint.optional(),
          index: z.number().int().min(0).max(199).optional(),
          after_index: z.number().int().min(0).max(199).optional(),
          xyz_mm: vec3.optional(),
          rpy_deg: vec3.optional(),
          by: z.union([z.literal(-1), z.literal(1)]).optional(),
          standoff_mm: z.number().finite().optional(),
          continuity: z.enum(["c0", "c1", "c2"]).optional(),
          start_from_row_id: rowId.nullable().optional(),
          response_format: responseFormat,
        })
        .strict(),
      annotations: EDITS,
    },
    onProgram(async (engine, args: {
      row_id: string; op: WaypointEdit["op"]; points?: PointArg[]; point?: PointArg; index?: number; after_index?: number; xyz_mm?: number[]; rpy_deg?: number[];
      by?: -1 | 1; standoff_mm?: number; continuity?: "c0" | "c1" | "c2"; start_from_row_id?: string | null; response_format: ResponseFormat;
    }) => {
      const need = <T>(value: T | undefined, name: string): T => {
        if (value === undefined) throw new Error(`'${args.op}' needs ${name}`);
        return value;
      };
      let edit: WaypointEdit;
      switch (args.op) {
        case "set_points": edit = { op: "set_points", points: need(args.points, "points").map(toInput) }; break;
        case "add_point": edit = { op: "add_point", afterIndex: args.after_index, point: toInput(need(args.point, "point")) }; break;
        case "move_point":
          if (!args.xyz_mm && !args.rpy_deg) return failure("'move_point' needs xyz_mm and/or rpy_deg");
          edit = { op: "move_point", index: need(args.index, "index"), xyzMm: args.xyz_mm, rpyDeg: args.rpy_deg };
          break;
        case "reorder_point": edit = { op: "reorder_point", index: need(args.index, "index"), by: need(args.by, "by (-1 or 1)") }; break;
        case "remove_point": edit = { op: "remove_point", index: need(args.index, "index") }; break;
        case "set_standoff": edit = { op: "set_standoff", index: args.index, standoffMm: need(args.standoff_mm, "standoff_mm") }; break;
        case "set_continuity": edit = { op: "set_continuity", index: args.index, continuity: need(args.continuity, "continuity") }; break;
        case "set_start_from": edit = { op: "set_start_from", startFrom: need(args.start_from_row_id, "start_from_row_id (null unties)") }; break;
      }
      engine.editWaypointWeld(args.row_id, edit);
      const view = engine.waypointWeldView(args.row_id);
      return result(args.response_format, { row_id: args.row_id, waypoint_weld: view, program: describeProgram(engine, ctx) }, () =>
        `${view.label}: ${args.op.replace(/_/g, " ")}.\n\n${waypointTable(view)}\n\n${spreadsheet(engine, ctx)}`);
    }),
  );

  server.registerTool(
    "rosie_program_plan",
    {
      title: "Plan the open program",
      description: `Plan the open program through OLP (POST /weld-plan), the request the browser's and the Deck's Plan button sends: OLP packs program.v2 with the robot's planning context, this session's fitted torch (rosie_program_rails_settings), the ACTIVE part's welds and STEP, and the workcell fixtures plus every other part as obstacles (STEP bodies as meshes, tessellated here from the kept STEP when not yet in this session; fixture_step_paths supplies STEPs this server never kept), the weld planner runs seam search, weld and connecting trajectory optimisation, taught MoveJ/MoveL/MoveC and the continuous verifier, and the dense trajectory comes back. The plan record is kept on the program and its bytes in this server's state dir. Moves nothing.

With a cell selected in OLP, a program with motion plans from the robot's measured pose (refused 409 planning_start_moving while it moves); from_program_start plans from the program's own start instead (then Go to plan start comes first: rosie_olp_run_program does it after confirmation). repeat 'count'/'loop' adds the planner's return to the start for Run N times / Loop.

Then: rosie_olp_load or rosie_olp_run_program with from_open_program=true. Seconds (taught moves) to minutes (welds on GPU).`,
      inputSchema: z
        .object({
          from_program_start: z.boolean().default(false),
          repeat: z.enum(["once", "count", "loop"]).default("once"),
          free_space_backend: z.enum(["bspline", "curobo", "legacy"]).default("bspline"),
          step_path: z.string().optional().describe("The active part's STEP, when this server has not kept it"),
          fixture_step_paths: fixtureStepPaths,
          response_format: responseFormat,
        })
        .strict(),
      annotations: { readOnlyHint: false, destructiveHint: false, idempotentHint: false, openWorldHint: true },
    },
    onProgram(async (engine, { from_program_start, repeat, free_space_backend, step_path, fixture_step_paths, response_format }: { from_program_start: boolean; repeat: string; free_space_backend: string; step_path?: string; fixture_step_paths?: string[]; response_format: ResponseFormat }) => {
      const planned = await programs.planProgram(ctx, engine, {
        fromProgramStart: from_program_start, repeat: repeat as programs.PlanOptions["repeat"], freeSpaceBackend: free_space_backend as programs.PlanOptions["freeSpaceBackend"],
        stepPath: step_path, fixtureStepPaths: fixture_step_paths,
      });
      const elapsed = planned.elapsedS.toFixed(1);
      const identity = planned.identity;
      const kept = planned.trajectoryFile;
      const connected = planned.startedFrom === "measured pose";
      return result(response_format, { elapsed_s: Number(elapsed), identity, plan: engine.planRecord(), trajectory_file: kept, program: describeProgram(engine, ctx) }, () =>
        `Planned in ${elapsed} s · ${engine.planRecord()?.totalSampleCount} samples, ${engine.planRecord()?.totalDurationS.toFixed(1)} s · ${from_program_start ? "from the program's start" : connected ? "from the robot's measured pose" : "offline"}.\nIdentity: ${json(identity)}${kept ? `\nTrajectory kept at \`${kept}\` (rosie_inspect_dense_trajectory reads it).` : ""}\n\n${spreadsheet(engine, ctx)}`);
    }),
  );

  server.registerTool(
    "rosie_program_preview_path",
    {
      title: "Preview the planned TCP path",
      description: `The open program's planned path as the TCP travels it: every sample of the kept trajectory through OLP's FK at the robot's TCP frame, on the capture model OLP compiles for the program (its description and pulled calibration). Per segment (one per planned row): kind, row id, duration, start and end TCP (mm), path length, straight-line chord, the largest deviation from that chord (about 0 for a MoveL; a MoveJ arcs), peak TCP speed (mm/s), bounds, and up to max_points sampled points (mm, world frame). A preview: it reads the plan and moves nothing.

Args: max_points per segment (default 20, up to 500); trajectory_path when neither this server nor the planner still has the plan's bytes.`,
      inputSchema: z.object({ max_points: z.number().int().min(2).max(500).default(20), trajectory_path: z.string().optional(), response_format: responseFormat }).strict(),
      annotations: { ...READS, openWorldHint: true },
    },
    onProgram(async (engine, { max_points, trajectory_path, response_format }: { max_points: number; trajectory_path?: string; response_format: ResponseFormat }) => {
      const preview = await programs.previewPath(ctx, engine, { maxPoints: max_points, trajectoryPath: trajectory_path });
      const { record, ...path3 } = preview;
      const round = (values: number[]) => values.map((v) => Math.round(v * 10) / 10);
      const data = { trajectory_digest: record.trajectoryDigest, current: record.current, ...path3 };
      return result(response_format, data, () => [
        `# Planned TCP path (${path3.frame}, ${record.totalDurationS.toFixed(1)} s, ${record.current ? "current" : "STALE"})`,
        "",
        table(["#", "Kind", "Row", "s", "Start mm", "End mm", "Path mm", "Chord mm", "Off chord mm", "Peak mm/s"], path3.segments.map((s) => [
          s.index, s.kind, s.nodeId ?? s.sourceId, s.durationS.toFixed(2), round(s.startMm).join(", "), round(s.endMm).join(", "),
          s.lengthMm.toFixed(1), s.chordMm.toFixed(1), s.maxChordDeviationMm.toFixed(2), s.peakTcpSpeedMmS.toFixed(0),
        ])),
      ].join("\n"));
    }),
  );

  server.registerTool(
    "rosie_program_export_weldplan",
    {
      title: "Export the open program as a .weldplan",
      description: `Pack the open program exactly as rosie_program_plan would (POST /weld-plan/export) and write the .weldplan container: program.v2, placement, the planning context, tooling and STEP, hashed together. It reproduces the plan elsewhere (rosie_planner_plan_weldplan, Weld Preflight). Moves nothing.`,
      inputSchema: z.object({
        path: z.string().min(1).describe("Where to write the .weldplan"), from_program_start: z.boolean().default(false), step_path: z.string().optional(), fixture_step_paths: fixtureStepPaths,
      }).strict(),
      annotations: EDITS_REMOTE,
    },
    onProgram(async (engine, { path: destination, from_program_start, step_path, fixture_step_paths }: { path: string; from_program_start: boolean; step_path?: string; fixture_step_paths?: string[] }) => {
      const exported = await programs.exportWeldplan(ctx, engine, destination, { fromProgramStart: from_program_start, stepPath: step_path, fixtureStepPaths: fixture_step_paths });
      return text(`Wrote ${destination} (sha256 ${exported.sha256 ?? "?"}).`, { path: destination, sha256: exported.sha256 });
    }),
  );
}
