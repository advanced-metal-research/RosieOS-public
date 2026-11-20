// rosie program: the workspace's OLP program, edited by OLP's own engine.
// Authoring and planning move nothing; a plan reaches the robot only through
// `rosie run` / `rosie robot load` and their gates.
import { OLP_API } from "../../lib/olp.js";
import * as programs from "../../lib/programs.js";
import { refusalText } from "../../lib/http.js";
import { json, record, table } from "../../lib/text.js";
import { ok, refused, UsageError, type Command, type OptionSpec, type Outcome } from "../command.js";
import type { Env } from "../env.js";
import { Workspace } from "../workspace.js";
import { assignments, onProgram, programLine, resolveRow, rowById, rowLine, vector3, type Program } from "./common.js";

const MOVE_OPTIONS: OptionSpec[] = [
  { name: "motion", type: "string", description: "joint (MoveJ), linear (MoveL), circular (MoveC)", choices: ["joint", "linear", "circular"] },
  { name: "speed_mm_s", type: "number", description: "TCP speed, mm/s" },
  { name: "speed_percent", type: "number", description: "Joint speed, 1-100 %" },
  { name: "label", type: "string", description: "Row label" },
];

function moveOptions(args: Record<string, unknown>): programs.MoveOptions {
  return {
    motion: args.motion as string | undefined, xyzMm: vector3(args.xyz, "--xyz"), rpyDeg: vector3(args.rpy, "--rpy"),
    speedMmS: args.speed_mm_s as number | undefined, speedPercent: args.speed_percent as number | undefined, label: args.label as string | undefined,
  };
}

/** A mutation's answer: the verdict, the program line, and the rows named. */
function changed(p: Program, verdict: string, rowIds: string[] = [], extra: Partial<Outcome> = {}): Outcome {
  const rows = rowIds.map((id) => rowById(p.engine, id)).filter((row) => row !== undefined);
  return ok(verdict, {
    body: [programLine(p), ...rows.map((row) => `  ${rowLine(row!)}`)].join("\n"),
    data: { program: programs.describeProgram(p.engine, p.ctx.program), rows },
    ...extra,
  });
}

const newProgram: Command<Env> = {
  path: ["program", "new"],
  summary: "Start a new program (rails authoring on) for the connected cell's robot or --model",
  description: "As OLP's New program: rails authoring on (welds from the part's detected seams), pinned to the robot description OLP serves. Replaces the workspace's program; refused while that one has unsaved edits unless --force (or use --ws <other>).",
  options: [
    { name: "name", type: "string", description: "Program name", positional: 0, required: true },
    { name: "model", type: "string", description: "Robot model, a robot_description model id (for example rosie_1420_v1); default the connected cell's" },
    { name: "force", type: "boolean", description: "Discard the workspace's unsaved program" },
  ],
  examples: ["rosie program new bracket-a", "rosie program new teach-test --model rosie_1420_v1 --ws scratch"],
  run: (env, args) => onProgram(env, { mutates: true, needsOpen: false, fresh: true }, async (p) => {
    if (p.ws.isUnsaved && !args.force) return refused(`workspace "${p.ws.name}" has an unsaved program (${programLine(p)}): save it, use --ws <other>, or --force`, { next: ["rosie program save", "rosie program save --file <path>"] });
    const model = await programs.newProgram(p.ctx, p.engine, String(args.name), args.model as string | undefined);
    return changed(p, `new program "${String(args.name)}" for ${model}`, [], { next: ["rosie part import <file.step>", "rosie program add-waypoint --joints <J1..J6 deg>"] });
  }),
};

const openProgram: Command<Env> = {
  path: ["program", "open"],
  summary: "Open a program from the cell's store (by id) or from an exported JSON file",
  options: [
    { name: "program_id", type: "string", description: "Cell program id (rosie program list)", positional: 0 },
    { name: "file", type: "string", description: "Instead: an OLP Export JSON file" },
    { name: "revision", type: "number", description: "An older store revision" },
    { name: "force", type: "boolean", description: "Discard the workspace's unsaved program" },
  ],
  examples: ["rosie program open bracket-a", "rosie program open --file ~/programs/bracket.olp.json"],
  run: (env, args) => onProgram(env, { mutates: true, needsOpen: false, fresh: true }, async (p) => {
    if (!args.program_id && !args.file) throw new UsageError("give a cell program id or --file");
    if (p.ws.isUnsaved && !args.force) return refused(`workspace "${p.ws.name}" has an unsaved program (${programLine(p)}): save it, use --ws <other>, or --force`);
    await programs.openProgram(p.ctx, p.engine, args.file ? { source: "file", path: String(args.file) } : { source: "cell", programId: String(args.program_id), revision: args.revision as number | undefined });
    const part = p.engine.partView();
    return changed(p, `opened ${args.file ? String(args.file) : `${String(args.program_id)} from the cell`}`, [], {
      next: part.cad ? ["rosie program show", "rosie view scene   (STEP bytes are never shared: pass --step <file> if this machine never imported the part)"] : ["rosie program show"],
    });
  }),
};

const listPrograms: Command<Env> = {
  path: ["program", "list"],
  summary: "The programs in the cell's store, and this machine's workspaces",
  options: [{ name: "query", type: "string", description: "Substring of id or name", positional: 0 }],
  examples: ["rosie program list", "rosie program list bracket"],
  async run(env, args) {
    const ctx = await env.ctx();
    const workspaces = await Workspace.list(env.home);
    const reply = await ctx.olp.get(`${OLP_API}/cell-programs`).catch((error: unknown) => error as Error);
    let cell = "";
    let programsList: Record<string, unknown>[] = [];
    if (reply instanceof Error) cell = `cell store unavailable: ${reply.message}`;
    else if (!reply.ok) cell = `cell store: ${refusalText(reply)}`;
    else {
      const body = record(reply.body);
      programsList = (Array.isArray(body.programs) ? body.programs : []).map(record);
      const needle = String(args.query ?? "").toLowerCase();
      if (needle) programsList = programsList.filter((x) => `${String(x.id ?? "")} ${String(x.name ?? "")}`.toLowerCase().includes(needle));
      cell = programsList.length ? table(["Id", "Name", "Rev", "Saved", "By", "Current"], programsList.map((x) => [x.id, x.name, x.revision, x.saved_at ?? x.updated_at, x.saved_by ?? x.by, x.current ? "yes" : ""])) : "no programs in the cell's store";
    }
    const body = [`cell store (${ctx.olp.baseUrl}):`, cell, "", "workspaces (this machine):", workspaces.length ? table(["Workspace", "Program", "Unsaved", "Updated"], workspaces.map((w) => [w.name === env.globals.workspace ? `${w.name} *` : w.name, w.program ?? "(empty)", w.unsaved ? "yes" : "", w.updated ?? ""])) : "none"].join("\n");
    return ok(`${programsList.length} cell program(s); ${workspaces.length} workspace(s)`, { body, data: { cell_programs: programsList, workspaces } });
  },
};

const show: Command<Env> = {
  path: ["program", "show"],
  summary: "The program as OLP's spreadsheet: rows, part, fixtures, plan state, problems",
  examples: ["rosie program show", "rosie program show --json"],
  run: (env) => onProgram(env, {}, async (p) => {
    const depth = await p.ws.depth();
    return ok(programLine(p), {
      body: `${programs.spreadsheet(p.engine, p.ctx.program)}\n\nworkspace ${p.ws.name} · undo ${depth.undo} · redo ${depth.redo}`,
      data: { ...programs.describeProgram(p.engine, p.ctx.program), workspace: p.ws.name, history: depth },
    });
  }),
};

const row: Command<Env> = {
  path: ["program", "row"],
  summary: "Every field of one row: path, value, unit, range, options (the paths edit takes)",
  options: [{ name: "row", type: "string", description: "Row id or number", positional: 0, required: true }],
  examples: ["rosie program row 2", "rosie program row rails-seam_0001"],
  run: (env, args) => onProgram(env, {}, async (p) => {
    const id = resolveRow(p.engine, args.row);
    const groups = p.engine.fields(id);
    const waypoints = p.engine.isWaypointWeld(id) ? p.engine.waypointWeldView(id) : null;
    const body = [
      ...groups.groups.map((group) => [`## ${group.group}${group.note ? ` — ${group.note}` : ""}`, table(["Path", "Field", "Value", "Unit", "Range", "Notes"], group.fields.map((field) => [
        field.path, field.label, typeof field.value === "object" ? JSON.stringify(field.value) : field.value, field.unit ?? "",
        [field.min, field.max].some((v) => v !== undefined) ? `${field.min ?? ""}..${field.max ?? ""}` : "",
        [field.readOnly ? `read-only: ${field.readOnly}` : "", field.options ? `options ${JSON.stringify(field.options)}` : "", field.kind !== "number" ? field.kind : ""].filter(Boolean).join("; "),
      ]))].join("\n")),
      waypoints ? `## Points (part frame)\n${table(["#", "xyz mm", "rpy °", "Continuity", "Standoff mm"], waypoints.points.map((pt) => [pt.index, programs.mm(pt.xyzMm), programs.mm(pt.rpyDeg), pt.continuity, pt.standoffMm]))}` : "",
    ].filter(Boolean).join("\n\n");
    return ok(`${groups.label} (${groups.op}) ${id}`, { body, data: { id, ...groups, waypoint_weld: waypoints } });
  }),
};

const edit: Command<Env> = {
  path: ["program", "edit"],
  summary: "Set fields of a row by OLP path, in display units (mm, °, %, s)",
  description: "Paths from `rosie program row <row>`: a move's motion, speed_mm_s, speed_scale (%), target.xyz_m.0..2 (mm, despite the name), target.rpy_rad.0..2 (degrees); a dwell's duration_s; a weld's travel_speed_mm_s, weave.*, geometry.span.start_s/end_s, geometry.direction; any row's label, comment, enabled. OLP validates each value (refuses, never clamps); edits apply in order and the first refusal stops the rest. Every edit makes the plan stale. Torch angles: rosie weld profile.",
  options: [
    { name: "row", type: "string", description: "Row id or number", positional: 0, required: true },
    { name: "set", type: "strings", description: "path=value pairs", positional: 1, rest: true, required: true },
  ],
  examples: ["rosie program edit 3 speed_mm_s=40 label=\"slow approach\"", "rosie program edit 2 motion=linear target.xyz_m.2=350", "rosie program edit dwell-1 duration_s=1.5"],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    const id = resolveRow(p.engine, args.row);
    const applied: string[] = [];
    for (const { path, value } of assignments(args.set)) {
      try {
        p.engine.edit(id, path, value);
        applied.push(path);
      } catch (error) {
        return refused(`${path} refused: ${error instanceof Error ? error.message : String(error)}${applied.length ? ` (applied before it: ${applied.join(", ")})` : " (nothing applied)"}`, { body: rowById(p.engine, id) ? rowLine(rowById(p.engine, id)!) : "" });
      }
    }
    return changed(p, `edited ${applied.join(", ")} on ${id}`, [id]);
  }),
};

const addWaypoint: Command<Env> = {
  path: ["program", "add-waypoint"],
  summary: "Add a taught MoveJ/MoveL/MoveC waypoint from joint angles (OLP's FK gives the TCP)",
  description: "Joints in degrees, URDF order, every joint of the program's robot (rosie program show --json lists joint_names). --xyz/--rpy make the target Cartesian (the joints become the IK seed). Recording moves nothing.",
  options: [
    { name: "joints", type: "numbers", description: "Joint angles in degrees, e.g. 0,-30,45,0,60,0", required: true },
    { name: "after", type: "string", description: "Insert after this row (default: the end, before a closing Home)" },
    { name: "xyz", type: "numbers", description: "Cartesian target x,y,z mm (overrides FK)" },
    { name: "rpy", type: "numbers", description: "Cartesian target roll,pitch,yaw °" },
    ...MOVE_OPTIONS,
  ],
  examples: ["rosie program add-waypoint --joints 0,-30,45,0,60,0", "rosie program add-waypoint --joints 10,-25,40,0,65,10 --motion linear --speed-mm-s 50"],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    const id = await programs.addWaypoint(p.ctx, p.engine, args.joints as number[], args.after ? resolveRow(p.engine, args.after) : null, moveOptions(args));
    return changed(p, `recorded waypoint ${id}`, [id], { next: ["rosie program plan", "rosie view scene --at row:" + id] });
  }),
};

const reteach: Command<Env> = {
  path: ["program", "reteach"],
  summary: "Replace a waypoint's target with new joint angles, or set its MoveC via pose",
  options: [
    { name: "row", type: "string", description: "Waypoint row", positional: 0, required: true },
    { name: "joints", type: "numbers", description: "Joint angles in degrees", required: true },
    { name: "as", type: "string", description: "target or via", choices: ["target", "via"], default: "target" },
  ],
  examples: ["rosie program reteach 2 --joints 5,-30,45,0,60,0", "rosie program reteach 3 --joints 0,-20,40,0,50,0 --as via"],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    const id = resolveRow(p.engine, args.row);
    await programs.capture(p.ctx, p.engine, args.as === "via" ? "via" : "reteach", args.joints as number[], id);
    return changed(p, `${args.as === "via" ? "via pose recorded on" : "retaught"} ${id}`, [id]);
  }),
};

const record_: Command<Env> = {
  path: ["program", "record"],
  summary: "Teach a waypoint from the selected cell's measured pose (jog there first; moves nothing)",
  description: "OLP's teach rules: every joint Home-valid, coordinates trusted, stationary; the cell's calibration for a machine that binds a description. Refused while the cell's session has a program loaded/staged/playing/paused (Stop releases it). 'Pull machine calibration' / 'Adopt the matching robot description' refusals: rosie program robot pull|adopt.",
  options: [
    { name: "mode", type: "string", description: "new | reteach | via", choices: ["new", "reteach", "via"], default: "new" },
    { name: "row", type: "string", description: "The waypoint (reteach, via)" },
    { name: "after", type: "string", description: "Insert after (new)" },
    ...MOVE_OPTIONS,
  ],
  examples: ["rosie program record", "rosie program record --mode reteach --row 3"],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    const recorded = await programs.recordRobotPose(p.ctx, p.engine, args.mode as "new" | "reteach" | "via", args.row ? resolveRow(p.engine, args.row) : null, args.after ? resolveRow(p.engine, args.after) : null, moveOptions(args));
    return changed(p, `recorded ${recorded.id} from ${recorded.cellId}'s measured pose (${recorded.joints})`, [recorded.id]);
  }),
};

const robotSettings: Command<Env> = {
  path: ["program", "robot"],
  summary: "The program's robot settings vs the cell: show, pull the cell's calibration, adopt the current description",
  options: [{ name: "action", type: "string", description: "show | pull | adopt", positional: 0, choices: ["show", "pull", "adopt"], default: "show" }],
  examples: ["rosie program robot", "rosie program robot pull"],
  run: (env, args) => onProgram(env, { mutates: args.action !== "show" }, async (p) => {
    const { headline, settings } = await programs.robotSettings(p.ctx, p.engine, args.action as "show" | "pull" | "adopt");
    return ok(headline || `${settings.model} · pinned ${settings.pinned ?? "none"} · ${settings.review ? `DRIFT: ${settings.review.title}` : "no drift"}`, {
      body: table(["Key", "Value"], [["model", settings.model], ["pinned description", settings.pinned ?? "none"], ["cell settings", settings.cell ? json(settings.cell) : "none pulled"], ["drift", settings.review ? `${settings.review.title} — ${settings.review.explanation}` : "none"]]),
      data: settings,
    });
  }),
};

const addEvent: Command<Env> = {
  path: ["program", "add-event"],
  summary: "Add a dwell (0.5 s), IO (revpi:1 ON) or Home row",
  options: [
    { name: "kind", type: "string", description: "dwell | io | home", positional: 0, required: true, choices: ["dwell", "io", "home"] },
    { name: "after", type: "string", description: "Insert after this row (default the end; Home always closes)" },
  ],
  examples: ["rosie program add-event dwell --after 2", "rosie program add-event home"],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    p.engine.select(args.after ? resolveRow(p.engine, args.after) : null);
    const id = p.engine.addEvent(args.kind as "dwell" | "io" | "home");
    return changed(p, `added ${String(args.kind)} ${id}`, [id], { next: args.kind === "dwell" ? [`rosie program edit ${id} duration_s=1.0`] : [] });
  }),
};

const remove: Command<Env> = {
  path: ["program", "remove"],
  summary: "Delete a row (OLP refuses rows that move with their weld)",
  options: [{ name: "row", type: "string", description: "Row id or number", positional: 0, required: true }],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    const id = resolveRow(p.engine, args.row);
    const target = rowById(p.engine, id)!;
    if (target.deleteBlocked) return refused(`OLP will not delete ${id}: ${target.deleteBlocked}`);
    p.engine.remove(id);
    return changed(p, `removed ${id}`);
  }),
};

const move: Command<Env> = {
  path: ["program", "move"],
  summary: "Move a row up (negative) or down by N places",
  options: [
    { name: "row", type: "string", description: "Row id or number", positional: 0, required: true },
    { name: "by", type: "number", description: "Places, negative = up", required: true },
  ],
  examples: ["rosie program move 4 --by -1"],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    const id = resolveRow(p.engine, args.row);
    const steps = Math.trunc(Number(args.by));
    if (!steps) throw new UsageError("--by must be a nonzero integer");
    const delta = steps < 0 ? -1 : 1;
    for (let i = 0; i < Math.abs(steps); i++) {
      const current = rowById(p.engine, id)!;
      const blocked = delta < 0 ? current.upBlocked : current.downBlocked;
      if (blocked) return i === 0 ? refused(`OLP will not move ${id}: ${blocked}`) : changed(p, `moved ${id} ${i} place(s); further: ${blocked}`, [id]);
      p.engine.move(id, delta);
    }
    return changed(p, `moved ${id} ${Math.abs(steps)} place(s) ${delta < 0 ? "up" : "down"}`, [id]);
  }),
};

function enableCommand(enabled: boolean): Command<Env> {
  return {
    path: ["program", enabled ? "enable" : "disable"],
    summary: enabled ? "Enable a row again" : "Disable a row (kept, skipped by planning)",
    options: [{ name: "row", type: "string", description: "Row id or number", positional: 0, required: true }],
    run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
      const id = resolveRow(p.engine, args.row);
      p.engine.enable(id, enabled);
      return changed(p, `${enabled ? "enabled" : "disabled"} ${id}`, [id]);
    }),
  };
}

const speed: Command<Env> = {
  path: ["program", "speed"],
  summary: "Plan speed: 1-100 % of the planner's whole trajectory (dwells stay in seconds)",
  options: [{ name: "percent", type: "number", description: "1-100", positional: 0, required: true }],
  examples: ["rosie program speed 50"],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    p.engine.setSpeedScale(Number(args.percent));
    return changed(p, `plan speed ${Number(args.percent)}%`);
  }),
};

function historyCommand(kind: "undo" | "redo"): Command<Env> {
  return {
    path: ["program", kind],
    summary: kind === "undo" ? "Undo program edits (OLP's restoreAsRevision; survives across commands)" : "Redo undone edits",
    options: [{ name: "steps", type: "number", description: "How many", positional: 0, default: 1 }],
    run: (env, args) => onProgram(env, {}, async (p) => {
      const done = await p.ws.step(p.ctx, p.engine, kind, Math.max(1, Number(args.steps)));
      const depth = await p.ws.depth();
      return (done ? ok : refused)(`${kind} ${done} step(s)${done < Number(args.steps) ? ` (no more ${kind === "undo" ? "history" : "redo"})` : ""}`, { body: `${programLine(p)} · undo ${depth.undo} · redo ${depth.redo}` });
    }),
  };
}

const save: Command<Env> = {
  path: ["program", "save"],
  summary: "Save to the cell's store (revision-checked) or --file",
  description: "The cell store refuses a stale save (409 program_conflict: someone saved since this workspace opened it) instead of overwriting; nothing is merged — reopen theirs or --copy (ask the user first). 423 while the robot holds this program. --make-current switches the cell's current program (what the Decks show).",
  options: [
    { name: "file", type: "string", description: "Save to this file instead of the cell" },
    { name: "copy", type: "boolean", description: "Save as a new program (new identity)" },
    { name: "make_current", type: "boolean", description: "Make it the cell's current program" },
  ],
  examples: ["rosie program save", "rosie program save --file ~/programs/bracket.olp.json"],
  run: (env, args) => onProgram(env, { mutates: true, saved: true }, async (p) => {
    try {
      const saved = await programs.saveProgram(p.ctx, p.engine, { target: args.file ? "file" : "cell", path: args.file as string | undefined, saveAsCopy: Boolean(args.copy), makeCurrent: Boolean(args.make_current) });
      return saved.kind === "file"
        ? ok(`saved to ${saved.path}`, { artifacts: [saved.path] })
        : ok(`saved ${saved.id} to the cell as revision ${saved.revision}${saved.refusedCurrent ? ` (not made current: ${JSON.stringify(saved.refusedCurrent)})` : ""}`);
    } catch (error) {
      if (error instanceof programs.ProgramRefusal && error.status === 409) {
        return refused(error.message, { data: { status: 409, body: error.body }, next: ["ask the user: reopen theirs (rosie program open <id> --force) or keep this as a copy (rosie program save --copy)"] });
      }
      throw error;
    }
  }),
};

const plan: Command<Env> = {
  path: ["program", "plan"],
  summary: "Plan through OLP's /weld-plan (seconds for taught moves, minutes for welds on GPU)",
  description: "As the browser's Plan button: the active part's welds and STEP, the session's fitted torch, workcell fixtures and other parts as obstacles. With a cell selected, a program with motion plans from the robot's measured pose (refused planning_start_moving while it moves); --from-start plans from the program's own start. --repeat count|loop adds the return to start (needed for Run N times / Loop). The trajectory is kept in the state dir. Moves nothing.",
  options: [
    { name: "from_start", type: "boolean", description: "Plan from the program's start, not the measured pose" },
    { name: "repeat", type: "string", description: "once | count | loop", choices: ["once", "count", "loop"], default: "once" },
    { name: "backend", type: "string", description: "Free-space planner", choices: ["bspline", "curobo", "legacy"], default: "bspline" },
    { name: "step", type: "string", description: "The active part's STEP, when this machine never imported it" },
    { name: "fixture_steps", type: "strings", description: "STEP files of fixtures/other parts never imported here" },
  ],
  examples: ["rosie program plan", "rosie program plan --from-start --repeat loop"],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    env.progress("planning through OLP /weld-plan…");
    const planned = await programs.planProgram(p.ctx, p.engine, {
      fromProgramStart: Boolean(args.from_start), repeat: args.repeat as programs.PlanOptions["repeat"], freeSpaceBackend: args.backend as programs.PlanOptions["freeSpaceBackend"],
      stepPath: args.step as string | undefined, fixtureStepPaths: args.fixture_steps as string[] | undefined,
    });
    const record = planned.plan;
    return ok(`planned in ${planned.elapsedS.toFixed(1)} s · ${record?.totalSampleCount ?? "?"} samples, ${record?.totalDurationS.toFixed(1) ?? "?"} s · from ${planned.startedFrom}`, {
      body: [programLine(p), "", record ? table(["#", "Kind", "Source", "Samples", "Duration s"], record.segments.map((s) => [s.index, s.kind, s.sourceId, s.sampleCount, s.durationS.toFixed(2)])) : "", "", `identity: ${json(planned.identity)}`].join("\n"),
      data: planned,
      artifacts: planned.trajectoryFile ? [planned.trajectoryFile] : [],
      next: ["rosie program check", "rosie view scene", "rosie run --sim"],
    });
  }),
};

const preview: Command<Env> = {
  path: ["program", "preview"],
  summary: "The planned TCP path per segment: length, chord deviation (≈0 for MoveL), peak TCP speed",
  options: [
    { name: "max_points", type: "number", description: "Sampled points per segment in --json", default: 20 },
    { name: "trajectory", type: "string", description: ".rdt file when neither this machine nor the planner has the plan's bytes" },
  ],
  examples: ["rosie program preview"],
  run: (env, args) => onProgram(env, {}, async (p) => {
    const planned = await programs.previewPath(p.ctx, p.engine, { maxPoints: Number(args.max_points), trajectoryPath: args.trajectory as string | undefined });
    const round = (values: number[]) => values.map((v) => Math.round(v * 10) / 10).join(", ");
    return ok(`${planned.segments.length} segments, ${planned.record.totalDurationS.toFixed(1)} s, plan ${planned.record.current ? "current" : "STALE"}`, {
      body: table(["#", "Kind", "Row", "s", "Start mm", "End mm", "Path mm", "Off chord mm", "Peak mm/s"], planned.segments.map((s) => [s.index, s.kind, s.nodeId ?? s.sourceId, s.durationS.toFixed(2), round(s.startMm), round(s.endMm), s.lengthMm.toFixed(1), s.maxChordDeviationMm.toFixed(2), s.peakTcpSpeedMmS.toFixed(0)])),
      data: planned,
    });
  }),
};

const exportWeldplan: Command<Env> = {
  path: ["program", "export"],
  summary: "Write the .weldplan container exactly as a plan would pack it (or --json for OLP's Export JSON)",
  options: [
    { name: "out", type: "string", description: "Output path (.weldplan, or .json for the program export)", positional: 0, required: true, short: "o" },
    { name: "from_start", type: "boolean", description: "Pack from the program's start" },
    { name: "step", type: "string", description: "The active part's STEP if not kept here" },
    { name: "fixture_steps", type: "strings", description: "Fixture/other-part STEPs not kept here" },
  ],
  examples: ["rosie program export bracket.weldplan", "rosie program export bracket.olp.json"],
  run: (env, args) => onProgram(env, {}, async (p) => {
    const out = String(args.out);
    if (out.endsWith(".json")) {
      const { writeFile } = await import("node:fs/promises");
      await writeFile(out, p.engine.exportProject());
      return ok(`exported the program to ${out}`, { artifacts: [out] });
    }
    const exported = await programs.exportWeldplan(p.ctx, p.engine, out, { fromProgramStart: Boolean(args.from_start), stepPath: args.step as string | undefined, fixtureStepPaths: args.fixture_steps as string[] | undefined });
    return ok(`wrote ${exported.path} (sha256 ${exported.sha256 ?? "?"})`, { artifacts: [exported.path], data: exported, next: [`rosie traj plan-weldplan ${exported.path}`] });
  }),
};

const workspaces: Command<Env> = {
  path: ["program", "workspaces"],
  summary: "This machine's workspaces (each holds one working program); --ws <name> selects one",
  async run(env) {
    const list = await Workspace.list(env.home);
    return ok(`${list.length} workspace(s); current ${env.globals.workspace}`, {
      body: list.length ? table(["Workspace", "Program", "Unsaved", "Updated"], list.map((w) => [w.name === env.globals.workspace ? `${w.name} *` : w.name, w.program ?? "(empty)", w.unsaved ? "yes" : "", w.updated ?? ""])) : "none yet",
      data: list,
    });
  },
};

export const commands: Command<Env>[] = [
  newProgram, openProgram, listPrograms, show, row, edit, addWaypoint, reteach, record_, robotSettings, addEvent, remove, move,
  enableCommand(true), enableCommand(false), speed, historyCommand("undo"), historyCommand("redo"), save, plan, preview, exportWeldplan, workspaces,
];
