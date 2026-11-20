// part, fixture, seam, rails, weld: authoring on the workspace's program,
// each a call into OLP's engine or the seam worker. Nothing here moves.
import type { FixtureField, WaypointEdit, WaypointInput, WeldTool } from "../../lib/engine.js";
import * as programs from "../../lib/programs.js";
import { table } from "../../lib/text.js";
import { ok, refused, UsageError, type Command, type Outcome } from "../command.js";
import type { Env } from "../env.js";
import { assignments, onProgram, programLine, resolveRow, rowById, rowLine, vector3, type Program } from "./common.js";

function changed(p: Program, verdict: string, rowIds: string[] = [], extra: Partial<Outcome> = {}): Outcome {
  const rows = rowIds.map((id) => rowById(p.engine, id)).filter((row) => row !== undefined);
  return ok(verdict, { body: [programLine(p), ...rows.map((row) => `  ${rowLine(row!)}`)].join("\n"), data: { program: programs.describeProgram(p.engine, p.ctx.program) }, ...extra });
}

// ---------------------------------------------------------------- part

const partImport: Command<Env> = {
  path: ["part", "import"],
  summary: "Import a STEP part (centred on the work frame) and detect its weld seams",
  description: "As OLP's Import STEP: CadQuery reads the topology, the part is centred on the cell's work frame, and for a rails program the seam worker detects weld joints and seams (~20 s). A second import ADDS a part and makes it active; earlier parts keep their welds and plan as obstacles. The STEP is kept under its fingerprint for later searches, plans and snapshots.",
  options: [{ name: "file", type: "string", description: ".step/.stp file, at most 8 MiB", positional: 0, required: true }],
  examples: ["rosie part import ~/parts/bracket_a.step"],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    env.progress("importing (CadQuery topology, then seam detection)…");
    const imported = await programs.importStep(p.ctx, p.engine, String(args.file));
    return changed(p, `imported ${imported.filename}: ${imported.edges} edges${imported.railsAuthoring ? `, ${imported.joints} weld joints, ${imported.seams} seams` : " (classic program: no seam detection)"}${imported.added ? ` — part ${imported.parts}, now active` : ""}`, [], {
      next: ["rosie part place --offset <x,y,z mm> [--rot <r,p,y °>]   (ask where the part sits)", "rosie seam list", "rosie view scene"],
    });
  }),
};

const partPlace: Command<Env> = {
  path: ["part", "place"],
  summary: "Place the active part in the work frame: base centre at --offset (mm), turned by --rot (°)",
  description: "OLP's Move part fields: the part's base centre (bounding-box x/y centre, lowest z) goes to the offset in the work frame (the 1400's table top, the 1420's floor). ±5000 mm, ±360°. The plan goes stale.",
  options: [
    { name: "offset", type: "numbers", description: "x,y,z mm", required: true },
    { name: "rot", type: "numbers", description: "roll,pitch,yaw °", default: [0, 0, 0] },
  ],
  examples: ["rosie part place --offset 600,0,0", "rosie part place --offset 600,-100,0 --rot 0,0,90"],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    const offset = vector3(args.offset, "--offset")!;
    const rot = vector3(args.rot, "--rot")!;
    if (offset.some((v) => Math.abs(v) > 5000) || rot.some((v) => Math.abs(v) > 360)) return refused("OLP's bounds: offset within ±5000 mm, rotation within ±360°");
    await p.ctx.program.installRobots(p.engine);
    p.engine.placePart(offset, rot);
    return changed(p, `part at [${programs.mm(offset)}] mm, rpy [${programs.mm(rot)}]°`, [], { next: ["rosie view scene", "rosie program plan"] });
  }),
};

const partList: Command<Env> = {
  path: ["part", "list"],
  summary: "The program's parts: which is active, welds, placement, whether its STEP is kept here",
  run: (env) => onProgram(env, {}, async (p) => {
    const parts = await Promise.all(p.engine.partsView().map(async (part) => ({ ...part, step_kept: await p.ctx.program.getSource(part.fingerprint).then(() => true, () => false) })));
    if (!parts.length) return ok("no part", { next: ["rosie part import <file.step>"] });
    return ok(`${parts.length} part(s); active ${parts.find((x) => x.active)?.filename ?? "none"}`, {
      body: table(["Part id", "File", "Active", "Welds", "Offset mm", "Rpy °", "STEP kept"], parts.map((x) => [x.id, x.filename, x.active ? "ACTIVE" : "", x.welds, x.placement ? programs.mm(x.placement.offsetFromWorkFrameMm) : "", x.placement ? programs.mm(x.placement.rotationRpyDeg) : "", x.step_kept ? "yes" : "no (--step)"])),
      data: parts,
    });
  }),
};

const partSwitch: Command<Env> = {
  path: ["part", "switch"],
  summary: "Make another part active (edited and planned); the previous one becomes an obstacle",
  options: [
    { name: "part", type: "string", description: "Part id (rosie part list)", positional: 0, required: true },
    { name: "step", type: "string", description: "Its STEP, if not kept here" },
  ],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    const { part, switched } = await programs.switchPart(p.ctx, p.engine, String(args.part), args.step as string | undefined);
    return changed(p, switched ? `${part.filename} is active` : `${part.filename} is already active`);
  }),
};

const partRemove: Command<Env> = {
  path: ["part", "remove"],
  summary: "Delete the ACTIVE part with its seams and welds (one undo step)",
  run: (env) => onProgram(env, { mutates: true }, async (p) => {
    const filename = p.engine.removePart();
    p.ctx.program.forgetPart();
    return changed(p, `removed ${filename}`, [], { next: ["rosie program undo   (to bring it back)"] });
  }),
};

// ---------------------------------------------------------------- fixture

const fixtureList: Command<Env> = {
  path: ["fixture", "list"],
  summary: "Everything a plan clears besides the robot and active part: stand, STEP fixtures, other parts",
  run: (env) => onProgram(env, {}, async (p) => {
    const view = p.engine.fixturesView();
    return ok(`${view.fixtures.length} fixture(s)`, {
      body: [view.fixtures.length ? view.fixtures.map((f) => `- ${f.kind === "part" ? "(other part) " : ""}${programs.fixtureLine(f, p.ctx.program)}`).join("\n") : "no workcell fixtures: the planner clears only the robot and the active part", `cell default: ${view.cellDefault.length ? view.cellDefault.map((f) => programs.fixtureLine(f, p.ctx.program)).join("; ") : "none"}`].join("\n"),
      data: view,
    });
  }),
};

function fixtureCommand(name: string, action: programs.FixtureArgs["action"], summary: string, options: Command<Env>["options"], examples: string[], toArgs: (args: Record<string, unknown>) => Omit<programs.FixtureArgs, "action">): Command<Env> {
  return {
    path: ["fixture", name], summary, options, examples,
    run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
      if (action === "replace_with_step" || action === "add_step_fixture") env.progress("tessellating (CadQuery)…");
      const done = await programs.editFixture(p.ctx, p.engine, { action, ...toArgs(args) });
      const verdict = !done.changed ? `the program already has a stand (${done.id}); edit it with rosie fixture edit ${done.id}` : action === "remove" ? `removed ${done.id}: no longer in collision checking` : `${name}: ${done.id}${done.note ? ` · ${done.note}` : ""}`;
      return changed(p, verdict, [], { next: ["rosie view scene"] });
    }),
  };
}

const FIXTURE_FIELDS = new Set(["sx", "sy", "sz", "x", "y", "z", "rx", "ry", "rz"]);

const fixtureCommands = [
  fixtureCommand("add-stand", "add_stand", "Add a 500 × 500 × 1000 mm box stand at 1000, 1000, 0 mm (if none)", [], ["rosie fixture add-stand"], () => ({})),
  fixtureCommand("edit", "edit", "Set a fixture's size (sx sy sz, 1-5000 mm), position (x y z, ±5000 mm) or rotation (rx ry rz, ±360°)",
    [{ name: "fixture", type: "string", description: "Fixture id", positional: 0, required: true }, { name: "set", type: "strings", description: "field=value pairs", positional: 1, rest: true, required: true }],
    ["rosie fixture edit stand sz=850 x=900", "rosie fixture edit stand rz=45"],
    (args) => ({
      fixtureId: String(args.fixture),
      fields: Object.fromEntries(assignments(args.set).map(({ path, value }) => {
        if (!FIXTURE_FIELDS.has(path) || typeof value !== "number") throw new UsageError(`${path}=${String(value)}: fields are ${[...FIXTURE_FIELDS].join(", ")} with numbers`);
        return [path as FixtureField, value];
      })),
    })),
  fixtureCommand("step", "add_step_fixture", "Add a STEP file as a fixture/obstacle (placed by its own origin; move it with fixture edit)",
    [{ name: "file", type: "string", description: ".step/.stp", positional: 0, required: true }], ["rosie fixture step ~/cell/clamp.step"], (args) => ({ path: String(args.file) })),
  fixtureCommand("replace", "replace_with_step", "Replace a fixture's shape with a STEP body (pose and id kept)",
    [{ name: "fixture", type: "string", description: "Fixture id", positional: 0, required: true }, { name: "file", type: "string", description: ".step/.stp", positional: 1, required: true }],
    ["rosie fixture replace stand ~/cell/stand.step"], (args) => ({ fixtureId: String(args.fixture), path: String(args.file) })),
  fixtureCommand("default", "cell_default", "Put the cell's placeholder back", [{ name: "fixture", type: "string", description: "Fixture id", positional: 0, required: true }], [], (args) => ({ fixtureId: String(args.fixture) })),
  fixtureCommand("remove", "remove", "Remove a fixture from collision checking", [{ name: "fixture", type: "string", description: "Fixture id", positional: 0, required: true }], [], (args) => ({ fixtureId: String(args.fixture) })),
];

// ---------------------------------------------------------------- seam

const seamList: Command<Env> = {
  path: ["seam", "list"],
  summary: "The part's detected weld seams: type, length, weld rows, search verdict",
  options: [{ name: "step", type: "string", description: "The part's STEP if not kept here" }],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    const seams = await programs.listSeams(p.ctx, p.engine, args.step as string | undefined);
    return ok(`${seams.length} seam(s); ${seams.filter((s) => s.weld_rows.length).length} welded, ${seams.filter((s) => s.searched).length} searched`, {
      body: seams.length ? table(["Seam", "Label", "Type", "Length", "Weld rows", "Search"], seams.map((s) => [s.seam_id, s.label, `${s.type ?? ""} ${s.weld_symbol ?? ""}`.trim(), `${s.length_mm} mm${s.closed ? " (closed)" : ""}`, s.weld_rows.join(", ") || "-", s.searched ? `${s.searched.state}, ${s.searched.clear_percent}% clear, ${s.searched.runs} run(s)` : "not searched"])) : "the seam worker detected no weld seams on this part",
      data: seams,
      next: seams.length ? ["rosie seam search --all", `rosie seam search ${seams[0]!.seam_id}`] : [],
    });
  }),
};

const seamSearch: Command<Env> = {
  path: ["seam", "search"],
  summary: "Search torch angles on a seam (or --all): clear, partial (clear runs) or blocked",
  description: "Rails mode's angle search with the session's fitted torch and search (rosie rails): both travel directions, the better coverage kept, then the clearance check. Seconds to minutes per seam; results are kept in the workspace for rosie seam weld. Moves nothing.",
  options: [
    { name: "seam", type: "string", description: "Seam id (rosie seam list)", positional: 0 },
    { name: "all", type: "boolean", description: "Search every detected seam" },
    { name: "fresh", type: "boolean", description: "Search again even if searched" },
    { name: "step", type: "string", description: "The part's STEP if not kept here" },
  ],
  examples: ["rosie seam search seam_0001", "rosie seam search --all"],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    if (!args.seam && !args.all) throw new UsageError("give a seam id or --all");
    const ids = args.all ? (await programs.listSeams(p.ctx, p.engine, args.step as string | undefined)).map((s) => s.seam_id) : [String(args.seam)];
    const results: programs.Coverage[] = [];
    for (const id of ids) {
      env.progress(`searching ${id}…`);
      results.push(await programs.searchSeam(p.ctx, p.engine, id, { fresh: Boolean(args.fresh), stepPath: args.step as string | undefined }));
      await p.ws.persist(p.ctx, p.engine);
    }
    const counts = ["clear", "partial", "blocked"].map((state) => `${results.filter((r) => r.state === state).length} ${state}`).join(", ");
    return ok(ids.length === 1 ? `${results[0]!.seam_id}: ${results[0]!.state.toUpperCase()} · ${results[0]!.clear_percent}% clear (${results[0]!.direction ?? "forward"}, standoff ${results[0]!.standoff_mm} mm)` : `${ids.length} seams: ${counts}`, {
      body: table(["Seam", "State", "Clear", "Direction", "Standoff", "Usable / blocked mm", "Clear runs"], results.map((r) => [r.seam_id, r.state, `${r.clear_percent}%`, r.direction ?? "forward", `${r.standoff_mm} mm`, `${r.usable_mm} / ${r.blocked_mm}`, r.state === "partial" ? r.runs.map((x) => `#${x.run} s=${x.span.start_s.toFixed(2)}..${x.span.end_s.toFixed(2)}`).join(" ") : ""])),
      data: results,
      next: results.some((r) => r.state !== "blocked") ? ["rosie seam weld <seam>   (ask the user before welding a partial seam's runs)", "rosie seam weld --clear   (accept every clear seam)"] : [],
    });
  }),
};

const seamWeld: Command<Env> = {
  path: ["seam", "weld"],
  summary: "Accept a searched seam as weld rows (or --clear: every seam searched clear)",
  description: "OLP's seamWeldNodeFor: the search's angles, direction and standoff; the first weld gets approach and retract, later welds a transit. 'trim' welds only the longest clear run; a partial seam's --runs each become their own weld (default all). Reaccepting keeps what was authored on the row.",
  options: [
    { name: "seam", type: "string", description: "Seam id", positional: 0 },
    { name: "trim", type: "boolean", description: "Weld only the longest clear run" },
    { name: "runs", type: "numbers", description: "Partial seam: which clear runs (from seam search)" },
    { name: "clear", type: "boolean", description: "Accept every seam searched 'clear' that has no weld yet" },
    { name: "step", type: "string", description: "The part's STEP if not kept here" },
  ],
  examples: ["rosie seam weld seam_0001", "rosie seam weld seam_0003 --runs 0,2", "rosie seam weld --clear"],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    if (!args.seam && !args.clear) throw new UsageError("give a seam id or --clear");
    const step = args.step as string | undefined;
    const ids = args.clear ? (await programs.listSeams(p.ctx, p.engine, step)).filter((s) => s.searched?.state === "clear" && !s.weld_rows.length).map((s) => s.seam_id) : [String(args.seam)];
    if (!ids.length) return refused("no seam searched clear without a weld", { next: ["rosie seam search --all"] });
    const added: string[] = [];
    const notes: string[] = [];
    for (const id of ids) {
      const done = await programs.addWeld(p.ctx, p.engine, id, { choice: args.trim ? "trim" : "accept", runs: args.runs as number[] | undefined, stepPath: step });
      added.push(...done.added);
      notes.push(...done.notes);
    }
    const rowIds = added.map((entry) => entry.split(" ")[0]!);
    return changed(p, `added ${added.length} weld(s)${notes.length ? `; ${notes.join("; ")}` : ""}`, rowIds, { next: ["rosie view scene", "rosie program plan"] });
  }),
};

// ---------------------------------------------------------------- rails

const railsShow: Command<Env> = {
  path: ["rails", "show"],
  summary: "The fitted torch and the angle search (workspace session state, as the browser's windows)",
  run: (env) => onProgram(env, { needsOpen: false }, async (p) => railsOutcome(p, "")),
};

function railsOutcome(p: Program, headline: string): Outcome {
  const settings = p.ctx.program.rails(p.engine);
  const sheet = (title: string, side: typeof settings.torch) => `${title}: ${side.preset.label} (${side.preset.id})${side.modified.length ? ` · modified ${side.modified.join(", ")}` : ""}\n${table(["Key", "Value", "Preset", "Range"], side.fields.map((f) => [f.key, `${String(f.value)} ${f.unit}`, `${String(f.presetValue)} ${f.unit}`, f.min !== null ? `${f.min}..${f.max} ${f.unit}` : ""]))}`;
  return ok(headline || `torch ${settings.torch.preset.id}${settings.torch.modified.length ? " (modified)" : ""} · search ${settings.search.preset.id}${settings.search.modified.length ? " (modified)" : ""}`, {
    body: [sheet("fitted torch", settings.torch), "", sheet("angle search", settings.search), "", `presets: torch ${settings.presets.torch.map((x) => x.id).join(", ")}; search ${settings.presets.search.map((x) => x.id).join(", ")}`].join("\n"),
    data: settings,
  });
}

const railsSet: Command<Env> = {
  path: ["rails", "set"],
  summary: "Change the torch / search (preset and/or values); drops searched seams, keeps accepted welds",
  description: "Torch keys: body_radius_mm 1-40, body_length_mm 20-600, standoff_min_mm / standoff_max_mm / default_standoff_mm 0.5-100. Search keys: work_deviation_deg 0-90, travel_deviation_deg 0-90, standoff_cost_deg_per_mm 0.1-30. OLP refuses out-of-range values (never clamps) and an inconsistent torch. Set it BEFORE searching.",
  options: [
    { name: "torch_preset", type: "string", description: "rosie_link6_gun (shipped fallback), mig_gas_nozzle_16" },
    { name: "search_preset", type: "string", description: "steady" },
    { name: "set", type: "strings", description: "key=value pairs (torch or search keys)", positional: 0, rest: true },
  ],
  examples: ["rosie rails set --torch-preset mig_gas_nozzle_16", "rosie rails set body_radius_mm=12 work_deviation_deg=20"],
  run: (env, args) => onProgram(env, { mutates: true, needsOpen: false }, async (p) => {
    const current = p.ctx.program.rails(p.engine);
    const torchKeys = new Set(current.torch.fields.map((f) => f.key));
    const searchKeys = new Set(current.search.fields.map((f) => f.key));
    const torch: Record<string, number> = {};
    const search: Record<string, number> = {};
    for (const { path, value } of args.set ? assignments(args.set) : []) {
      if (typeof value !== "number") throw new UsageError(`${path} takes a number`);
      if (torchKeys.has(path)) torch[path] = value;
      else if (searchKeys.has(path)) search[path] = value;
      else throw new UsageError(`unknown key ${path}; torch: ${[...torchKeys].join(", ")}; search: ${[...searchKeys].join(", ")}`);
    }
    const changed = p.ctx.program.setRails(p.engine, {
      torchPreset: args.torch_preset as string | undefined, torch: Object.keys(torch).length ? torch : undefined,
      searchPreset: args.search_preset as string | undefined, search: Object.keys(search).length ? search : undefined,
    });
    return railsOutcome(p, `settings changed${changed.dropped ? `; ${changed.dropped} searched seam(s) dropped — search again` : ""}`);
  }),
};

const railsReset: Command<Env> = {
  path: ["rails", "reset"],
  summary: "Back to the shipped torch and search presets",
  run: (env) => onProgram(env, { mutates: true, needsOpen: false }, async (p) => {
    const changed = p.ctx.program.setRails(p.engine, "reset");
    return railsOutcome(p, `back to the shipped presets${changed.dropped ? `; ${changed.dropped} searched seam(s) dropped` : ""}`);
  }),
};

// ---------------------------------------------------------------- weld

const weldProfile: Command<Env> = {
  path: ["weld", "profile"],
  summary: "A rails weld's work or travel angle: --constant ° or --points s:deg,s:deg,...",
  description: "s is the normalised arc length along the weld (first 0, last 1, increasing); degrees within -90..90; linear between points. One undo step; replan afterwards.",
  options: [
    { name: "row", type: "string", description: "Rails weld row", positional: 0, required: true },
    { name: "angle", type: "string", description: "work | travel", choices: ["work", "travel"], required: true },
    { name: "constant", type: "number", description: "One angle, degrees" },
    { name: "points", type: "string", description: "s:deg pairs, e.g. 0:40,0.5:45,1:42" },
  ],
  examples: ["rosie weld profile rails-seam_0001 --angle work --constant 45", "rosie weld profile 3 --angle travel --points 0:10,1:15"],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    const id = resolveRow(p.engine, args.row);
    if ((args.constant === undefined) === (args.points === undefined)) throw new UsageError("give exactly one of --constant or --points");
    const points = args.points !== undefined
      ? String(args.points).split(",").map((pair) => {
        const [s, deg] = pair.split(":").map(Number);
        if (!Number.isFinite(s) || !Number.isFinite(deg)) throw new UsageError(`"${pair}" is not s:deg`);
        return { s: s!, deg: deg! };
      })
      : [{ s: 0, deg: Number(args.constant) }, { s: 1, deg: Number(args.constant) }];
    const { path, fn } = p.engine.weldProfile(id, args.angle as "work" | "travel", points);
    p.engine.edit(id, path, fn);
    return changed(p, `${String(args.angle)} angle of ${id}: ${JSON.stringify(fn)}`, [id]);
  }),
};

function parsePoints(value: unknown, name: string): WaypointInput[] {
  if (!Array.isArray(value)) throw new UsageError(`${name} takes a JSON list of {"xyz_mm":[..],"rpy_deg":[..],"continuity"?,"standoff_mm"?}`);
  return value.map((point: Record<string, unknown>, i) => {
    const xyz = point.xyz_mm as number[];
    const rpy = point.rpy_deg as number[];
    if (!Array.isArray(xyz) || xyz.length !== 3 || !Array.isArray(rpy) || rpy.length !== 3) throw new UsageError(`${name}[${i}] needs xyz_mm and rpy_deg, three numbers each`);
    return { xyzMm: xyz, rpyDeg: rpy, continuity: point.continuity as WaypointInput["continuity"], standoffMm: point.standoff_mm as number | undefined };
  });
}

const weldAdd: Command<Env> = {
  path: ["weld", "add"],
  summary: "Add a waypoint / line / arc weld from torch poses in the PART frame (mm, °)",
  description: `Points: JSON list of {"xyz_mm":[x,y,z],"rpy_deg":[r,p,y],"continuity":"c0|c1|c2","standoff_mm":n}; part frame (moves with the part); rpy [180,0,0] is the torch straight down (tool +Z into the work). line = 2 points, arc = 3 non-collinear, waypoint ≥ 2. Needs a rails program with a part.`,
  options: [
    { name: "tool", type: "string", description: "waypoint | line | arc", choices: ["waypoint", "line", "arc"], default: "waypoint" },
    { name: "points", type: "json", description: "JSON list of points", required: true },
    { name: "label", type: "string", description: "Row label" },
    { name: "travel_speed_mm_s", type: "number", description: "Travel speed mm/s" },
    { name: "start_from", type: "string", description: "Tie the first point to the end of this weld row" },
  ],
  examples: [`rosie weld add --tool line --points '[{"xyz_mm":[0,0,5],"rpy_deg":[180,0,0]},{"xyz_mm":[40,0,5],"rpy_deg":[180,0,0]}]'`],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    const id = p.engine.addWaypointWeld(args.tool as WeldTool, parsePoints(args.points, "--points"), args.start_from ? resolveRow(p.engine, args.start_from) : null);
    if (args.label) p.engine.edit(id, "label", args.label);
    if (args.travel_speed_mm_s !== undefined) p.engine.edit(id, "travel_speed_mm_s", args.travel_speed_mm_s);
    return changed(p, `added ${p.engine.waypointWeldView(id).label} (${id})`, [id], { next: ["rosie view scene", `rosie program row ${id}`] });
  }),
};

const weldEdit: Command<Env> = {
  path: ["weld", "edit"],
  summary: "One edit of a waypoint/line/arc weld's poses (points indexed from 0)",
  description: "--op set_points (--points), add_point (--point, --after-index), move_point (--index, --xyz and/or --rpy), reorder_point (--index, --by -1|1), remove_point (--index), set_standoff (--standoff, --index or all), set_continuity (--continuity, --index or all), set_start_from (--start-from row or 'none').",
  options: [
    { name: "row", type: "string", description: "Waypoint weld row", positional: 0, required: true },
    { name: "op", type: "string", description: "Edit", required: true, choices: ["set_points", "add_point", "move_point", "reorder_point", "remove_point", "set_standoff", "set_continuity", "set_start_from"] },
    { name: "points", type: "json", description: "set_points: JSON list" },
    { name: "point", type: "json", description: "add_point: one JSON point" },
    { name: "index", type: "number", description: "Point index" },
    { name: "after_index", type: "number", description: "add_point: insert after" },
    { name: "xyz", type: "numbers", description: "move_point: x,y,z mm" },
    { name: "rpy", type: "numbers", description: "move_point: r,p,y °" },
    { name: "by", type: "number", description: "reorder_point: -1 or 1" },
    { name: "standoff", type: "number", description: "set_standoff: 0.5-100 mm" },
    { name: "continuity", type: "string", description: "set_continuity: c0 | c1 | c2", choices: ["c0", "c1", "c2"] },
    { name: "start_from", type: "string", description: "set_start_from: a weld row, or none" },
  ],
  examples: ["rosie weld edit 4 --op move_point --index 1 --xyz 45,0,5", "rosie weld edit 4 --op set_standoff --standoff 15"],
  run: (env, args) => onProgram(env, { mutates: true }, async (p) => {
    const id = resolveRow(p.engine, args.row);
    const need = <T>(value: T | undefined, name: string): T => {
      if (value === undefined) throw new UsageError(`--op ${String(args.op)} needs --${name}`);
      return value;
    };
    let editOp: WaypointEdit;
    switch (args.op) {
      case "set_points": editOp = { op: "set_points", points: parsePoints(need(args.points, "points"), "--points") }; break;
      case "add_point": editOp = { op: "add_point", afterIndex: args.after_index as number | undefined, point: parsePoints([need(args.point, "point")], "--point")[0]! }; break;
      case "move_point":
        if (!args.xyz && !args.rpy) throw new UsageError("move_point needs --xyz and/or --rpy");
        editOp = { op: "move_point", index: need(args.index as number | undefined, "index"), xyzMm: vector3(args.xyz, "--xyz"), rpyDeg: vector3(args.rpy, "--rpy") };
        break;
      case "reorder_point": editOp = { op: "reorder_point", index: need(args.index as number | undefined, "index"), by: need(args.by as -1 | 1 | undefined, "by") }; break;
      case "remove_point": editOp = { op: "remove_point", index: need(args.index as number | undefined, "index") }; break;
      case "set_standoff": editOp = { op: "set_standoff", index: args.index as number | undefined, standoffMm: need(args.standoff as number | undefined, "standoff") }; break;
      case "set_continuity": editOp = { op: "set_continuity", index: args.index as number | undefined, continuity: need(args.continuity as "c0" | "c1" | "c2" | undefined, "continuity") }; break;
      default: {
        const from = String(need(args.start_from, "start-from"));
        editOp = { op: "set_start_from", startFrom: from === "none" ? null : resolveRow(p.engine, from) };
      }
    }
    p.engine.editWaypointWeld(id, editOp);
    return changed(p, `${String(args.op).replace(/_/g, " ")} on ${id}`, [id]);
  }),
};

export const commands: Command<Env>[] = [
  partImport, partPlace, partList, partSwitch, partRemove,
  fixtureList, ...fixtureCommands,
  seamList, seamSearch, seamWeld,
  railsShow, railsSet, railsReset,
  weldProfile, weldAdd, weldEdit,
];
