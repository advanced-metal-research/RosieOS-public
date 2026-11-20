/**
 * The presets rails mode authors against.
 *
 * These used to live in the weld planner's settings store, resolved server-side
 * on every request. D24 deleted that store: the worker holds nothing between
 * calls, so what it is given has to come from here, and the OLP is what has an
 * operator to ask.
 *
 * Two kinds, and they do different jobs at different moments:
 *
 * - the TORCH is a fact about the cell -- which gun is fitted, how fat its
 *   barrel is, how far off the work it is held. It reaches the worker on every
 *   measurement, because a coverage number measured with the shipped nozzle in
 *   place of the one that is actually fitted is a number about hardware nobody
 *   has.
 * - the SEARCH is how hard to look: how far the search may move an angle from
 *   the profile that was drawn, and what a millimetre of standoff departure
 *   costs in degrees. It reaches the worker only while a seam is being
 *   searched, and once a weld is accepted the values it used are recorded on
 *   the node (`search_provenance`), not looked up again.
 *
 * VALUES, never an id to resolve later. A preset is a starting point and a
 * comparison; a weld that stored an id would change meaning the day the
 * library did, which is the property `resolve_weld_list` had and D24 gave up
 * deliberately. Here that is the point: editing a preset cannot reach back and
 * change a weld that was already accepted.
 *
 * These are the SHIPPED values, and they are only half of it now. The rest is
 * `parameters.ts`, which describes them as a form, and `src/parameters`, which
 * draws that form: the operator picks a preset, edits a field, puts one field
 * or all of them back, and saves what they arrived at under a name. Saved
 * presets live in the project store, per browser origin, beside the projects
 * rather than inside one -- a procedure worked out on one part is meant to be
 * reachable from the next.
 *
 * A shipped preset is read-only. It is the thing "revert to preset" reverts
 * TO, so overwriting one would remove the only fixed point in the window;
 * Save refuses on one and says to use Save as. That is not a permissions rule,
 * it is what makes revert mean anything.
 */

import { DEFAULT_TORCH_STANDOFF_MM } from "../domain/project.ts";

/** A gun, as `reach/tooling.py` reads one. */
export interface TorchPreset {
  id: string;
  label: string;
  body_radius_mm: number;
  body_length_mm: number;
  standoff_range_mm: { min: number; max: number };
}

/** How far the angle search may wander, and what wandering costs. */
export interface SearchPreset {
  id: string;
  label: string;
  work_deviation_deg: number;
  travel_deviation_deg: number;
  standoff_cost_deg_per_mm: number;
}

/** The gun baked into rosie_1400_v3's `link_6`: about 30 mm across all the way
 *  to the tip, because the mesh IS the hardware
 *  (`weld_planner/v1/data/presets/torch/rosie-link6-gun.json`). Shipped FIRST,
 *  so it is what a rails program authors against.
 *
 *  Measured on bracket_b_2x, 2026-09-11: authored against this gun, the seam
 *  search and its clearance check reproduce the seams that planned and ran on
 *  the motors to the last digit -- weld_joint_0004 trimmed 11.5 mm off its
 *  blocked end with travel 14.25 -> 10 deg, weld_joint_0002 likewise -- and the
 *  motion planner crosses both. Against the Ø16 nozzle the same clicks kept the
 *  full span and bent the travel angle to +35 / -30 deg to reach into the
 *  corners, which the torch model allows and the arm cannot follow: neither
 *  crossed. The weld planner's old workspace settings carried this radius. */
export const ROSIE_LINK6_GUN: TorchPreset = {
  id: "rosie_link6_gun",
  label: "Rosie 1400, link_6 gun",
  body_radius_mm: 16.0,
  body_length_mm: 150.0,
  standoff_range_mm: { min: 15.0, max: 30.0 },
};

/** A Ø16 gas nozzle with 150 mm of barrel behind it -- NOT the gun on this
 *  cell (see `ROSIE_LINK6_GUN`), and too thin to author rosie_1400_v3 against.
 *  The range is what this gun is HELD at and not what the arithmetic permits --
 *  under about 6 mm the nozzle is inside the groove, and past 25 mm the gas
 *  column stops covering the puddle. */
export const MIG_GAS_NOZZLE_16: TorchPreset = {
  id: "mig_gas_nozzle_16",
  label: "MIG, Ø16 gas nozzle",
  body_radius_mm: 8.0,
  body_length_mm: 150.0,
  standoff_range_mm: { min: 6.0, max: 25.0 },
};

/** A deviation and not an absolute band, so the window follows a ramp instead
 *  of standing still. A weld's own `orientation.limits` bound the result on
 *  top of this, and the admissible set is the intersection. */
export const STEADY_SEARCH: SearchPreset = {
  id: "steady",
  label: "Steady — ±15° work, ±40° travel",
  work_deviation_deg: 15.0,
  travel_deviation_deg: 40.0,
  standoff_cost_deg_per_mm: 3.0,
};

/** Where a weld sits when it declares no stickout of its own: the cell's one
 *  default, the same number classic authors at. */
export const DEFAULT_STANDOFF_MM = DEFAULT_TORCH_STANDOFF_MM;

export const TORCH_PRESETS: readonly TorchPreset[] = [ROSIE_LINK6_GUN, MIG_GAS_NOZZLE_16];
export const SEARCH_PRESETS: readonly SearchPreset[] = [STEADY_SEARCH];

/** What the worker is given for a search, and what is recorded beside the
 *  angles it chose. The two are the same numbers on purpose: a provenance
 *  block that could differ from what ran would answer the wrong question. */
export function searchKnobs(preset: SearchPreset = STEADY_SEARCH): {
  work_deviation_deg: number;
  travel_deviation_deg: number;
  standoff_cost_deg_per_mm: number;
} {
  return {
    work_deviation_deg: preset.work_deviation_deg,
    travel_deviation_deg: preset.travel_deviation_deg,
    standoff_cost_deg_per_mm: preset.standoff_cost_deg_per_mm,
  };
}
