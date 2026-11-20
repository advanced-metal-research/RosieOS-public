/**
 * The seam worker's parameters, as a schema the parameter window can draw.
 *
 * `presets.ts` holds the shipped values and says what each one is for; this
 * says what an operator may do to them -- the label, the unit, the bounds,
 * and the one sentence that explains the number. The two are kept apart
 * because the values are also what the worker is sent and what a weld records
 * in `search_provenance`, and a schema is only ever a description of a form.
 *
 * The bounds here are the ones that make a request MEANINGFUL, not the ones
 * the arithmetic survives. A standoff cost of zero is arithmetic; it also
 * says a millimetre off the work is free, which is a search that will hold
 * the gun anywhere.
 */

import type { ParameterPreset, ParameterSchema } from "../parameters/model.ts";
import { DEFAULT_STANDOFF_MM, SEARCH_PRESETS, TORCH_PRESETS } from "./presets.ts";

/** How hard the angle search looks, and what looking costs.
 *
 *  Reaches the worker only while a seam is being searched. Once a weld is
 *  accepted the values it used are recorded on the node, not looked up
 *  again -- so changing these changes the NEXT search and nothing already
 *  authored. */
export const SEARCH_SCHEMA: ParameterSchema = [
  {
    key: "work_deviation_deg",
    label: "Work angle deviation",
    kind: "number",
    unit: "°",
    min: 0,
    max: 90,
    step: 1,
    group: "How far the search may wander",
    help:
      "How far off the drawn work angle the search may go. A deviation and not "
      + "an absolute band, so the window follows a ramp instead of standing still. "
      + "The weld's own orientation limits bound the result on top of this.",
  },
  {
    key: "travel_deviation_deg",
    label: "Travel angle deviation",
    kind: "number",
    unit: "°",
    min: 0,
    max: 90,
    step: 1,
    group: "How far the search may wander",
    help:
      "How far off the drawn travel angle the search may go. Wider than work by "
      + "default: pushing or dragging the gun is a choice the metal often makes "
      + "for you, and the search is allowed to find it.",
  },
  {
    key: "standoff_cost_deg_per_mm",
    label: "Standoff cost",
    kind: "number",
    unit: "°/mm",
    min: 0.1,
    max: 30,
    step: 0.1,
    group: "What wandering costs",
    help:
      "What a millimetre of standoff departure is worth in degrees of angle "
      + "deviation. Low, and the search will lift the gun off the work to keep an "
      + "angle; high, and it will bend the angle to hold the stickout.",
  },
];

/** The gun that is fitted. A fact about the cell, so it travels with every
 *  measurement: a coverage number measured with the shipped nozzle in place
 *  of the one actually fitted is a number about hardware nobody has. */
export const TORCH_SCHEMA: ParameterSchema = [
  {
    key: "body_radius_mm",
    label: "Nozzle radius",
    kind: "number",
    unit: "mm",
    min: 1,
    max: 40,
    step: 0.5,
    group: "The gun",
    help: "Half the nozzle across its widest point. This is what has to fit in the groove.",
  },
  {
    key: "body_length_mm",
    label: "Barrel length",
    kind: "number",
    unit: "mm",
    min: 20,
    max: 600,
    step: 5,
    group: "The gun",
    help: "How much gun sits behind the nozzle, as a cylinder, for the clearance rays to miss.",
  },
  {
    key: "standoff_min_mm",
    label: "Standoff, closest",
    kind: "number",
    unit: "mm",
    min: 0.5,
    max: 100,
    step: 0.5,
    group: "How it is held",
    help:
      "The closest this gun is HELD to the work, not what the arithmetic permits. "
      + "Under about 6 mm the nozzle is inside the groove.",
  },
  {
    key: "standoff_max_mm",
    label: "Standoff, furthest",
    kind: "number",
    unit: "mm",
    min: 0.5,
    max: 100,
    step: 0.5,
    group: "How it is held",
    help: "Past about 25 mm the gas column stops covering the puddle.",
  },
  {
    key: "default_standoff_mm",
    label: "Default standoff",
    kind: "number",
    unit: "mm",
    min: 0.5,
    max: 100,
    step: 0.5,
    group: "How it is held",
    help: "Where a weld sits when it declares no stickout of its own.",
  },
];

/** The rules `reach/tooling.py` enforces on the other side, stated here so
 *  the window says what is wrong before a request does.
 *
 *  Advisory, not a refusal at the input: widening a range means editing two
 *  numbers, and a window that refused the first would make the second
 *  unreachable. */
export function torchProblems(values: Record<string, number | string | boolean>): string[] {
  const number = (key: string): number => Number(values[key] ?? 0);
  const problems: string[] = [];
  const min = number("standoff_min_mm");
  const max = number("standoff_max_mm");
  const barrel = number("body_length_mm");
  const fallback = number("default_standoff_mm");
  if (min > max) {
    problems.push(`the standoff range crosses: closest ${min} mm is past furthest ${max} mm`);
  }
  if (barrel <= max) {
    problems.push(
      `the barrel is ${barrel} mm and the gun is held out to ${max} mm — `
      + "the barrel has to be longer than the furthest standoff",
    );
  }
  if (fallback < min || fallback > max) {
    problems.push(`the default standoff ${fallback} mm is outside the range ${min}–${max} mm`);
  }
  return problems;
}

/** The torch as the seam worker reads one, from the window's values. */
export function torchRequest(
  values: Record<string, number | string | boolean>,
  preset: { id: string; label: string },
): Record<string, unknown> {
  return {
    id: preset.id,
    label: preset.label,
    body_radius_mm: Number(values.body_radius_mm),
    body_length_mm: Number(values.body_length_mm),
    standoff_range_mm: {
      min: Number(values.standoff_min_mm),
      max: Number(values.standoff_max_mm),
    },
  };
}

export const SEARCH_SCOPE = "seam.search";
export const TORCH_SCOPE = "seam.torch";

export const SHIPPED_SEARCH_PRESETS: readonly ParameterPreset[] = SEARCH_PRESETS.map((preset) => ({
  id: preset.id,
  label: preset.label,
  origin: "shipped" as const,
  values: {
    work_deviation_deg: preset.work_deviation_deg,
    travel_deviation_deg: preset.travel_deviation_deg,
    standoff_cost_deg_per_mm: preset.standoff_cost_deg_per_mm,
  },
}));

export const SHIPPED_TORCH_PRESETS: readonly ParameterPreset[] = TORCH_PRESETS.map((preset) => ({
  id: preset.id,
  label: preset.label,
  origin: "shipped" as const,
  values: {
    body_radius_mm: preset.body_radius_mm,
    body_length_mm: preset.body_length_mm,
    standoff_min_mm: preset.standoff_range_mm.min,
    standoff_max_mm: preset.standoff_range_mm.max,
    // The shipped default, and the number `tooling.json` carried. It lives on
    // this window rather than the search one because it is a fact about the
    // gun, not about how hard to look. A saved torch preset carries its own.
    default_standoff_mm: DEFAULT_STANDOFF_MM,
  },
}));
