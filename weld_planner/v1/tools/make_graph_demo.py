#!/usr/bin/env python
"""Write what M4 computed into the shape `ui/graph.html` draws.

`IMPLEMENTATION.md` MV: a stage whose output can only be judged by reading
numbers is a stage that will be judged wrong. The redundancy lattice is the
clearest case of that in the whole planner -- "3888 of 6912 nodes survived" is
a number that could describe a healthy seam or a seam hanging by a thread at
one sample, and the two look nothing alike when drawn.

Writes `ui/public/debug/<plan request>/graph.json`, next to the bundles
`weld_motion_planner.io.debug` already writes there. Same rules as those: schema `v0`,
nothing may depend on it, and it may break at any commit.

Run: `pixi run -e motion python tools/make_graph_demo.py`.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT_ROOT = ROOT / "ui" / "public" / "debug"
SCHEMA = "motion-graph.v0"


def one_seam(cell, seam, placement, *, step_s, spin_bins, positioner_bins,
             scene=None, threshold_rad=None, derived=False,
             k_best=4) -> dict[str, Any]:
    from weld_motion_planner.planner.dp_seam_search.engine import graph as graphmod
    from weld_motion_planner.planner.dp_seam_search.engine import nodes
    from weld_motion_planner.robot_cell.robot_cell import RobotCell

    started = time.perf_counter()
    lattice = nodes.build(RobotCell.of(cell), seam, step_s=step_s, spin_bins=spin_bins,
                          positioner_bins=positioner_bins,
                          work_to_piece=placement, scene=scene)
    elapsed = time.perf_counter() - started

    # The cause grid is what the page draws, and it goes out as a flat list of
    # small integers rather than nested arrays: 64x12x9 is 6912 numbers, and
    # JSON nesting triples the byte count for something the page reshapes in
    # one line anyway.
    cause = lattice.cause.cpu().numpy().astype(np.int8)
    out: dict[str, Any] = {
        "id": seam.id,
        "length_mm": round(seam.length_m * 1000.0, 2),
        "shape": list(cause.shape),
        "cause": cause.reshape(-1).tolist(),
        "fractions": [round(float(v), 6) for v in lattice.fractions],
        "spin_deg": [round(float(np.rad2deg(v)), 3) for v in lattice.spin_rad],
        "positioner_deg": [round(float(np.rad2deg(v)), 3) for v in lattice.positioner_rad],
        "positioner_axis": lattice.positioner_axis,
        "axis_names": lattice.axis_names,
        "summary": lattice.summary(),
        "per_layer": [int(v) for v in lattice.per_layer()],
        "build_s": round(elapsed, 4),
    }
    # Joint values, rounded hard. The page shows them on hover and nothing
    # computes from them, so four decimals of a radian -- 0.006 degrees -- is
    # more than a tooltip can use.
    out["q"] = np.round(lattice.q.cpu().numpy(), 4).reshape(-1).tolist()
    if lattice.weld_pos_m is not None:
        out["weld_point_m"] = np.round(
            lattice.weld_pos_m.cpu().numpy(), 5).reshape(-1).tolist()
    if lattice.clearance_m is not None:
        out["clearance_mm"] = np.round(
            lattice.clearance_m.cpu().numpy() * 1000.0, 2).reshape(-1).tolist()

    # The search, and the paths it found. Node indices per layer rather than
    # joint values: the page already has the joints, and an index is what lets
    # it highlight the chosen cell in the lattice as well as pose the robot.
    # PER AXIS, not the tightest of them. Collapsing the derived vector to its
    # minimum would hold the wrist to the positioner's 60 deg/s when it can do
    # 240 -- four times tighter than the cell allows, on the axes with the most
    # room. `search` has taken a vector since it was written.
    result = graphmod.search(lattice, threshold_rad=threshold_rad,
                             k_best=k_best, derived=derived)
    out["graph"] = {
        **result.summary(),
        "paths": result.paths.tolist(),
        "costs": [None if not np.isfinite(c) else round(float(c), 5) for c in result.costs],
        "reachable_per_layer": [int(v) for v in result.reachable],
        "threshold_deg_per_axis": [round(float(np.rad2deg(v)), 3)
                                   for v in np.atleast_1d(threshold_rad)],
    }
    return out


def build(path: Path, *, step_s: float, spin_bins: int, positioner_bins: int,
          limit: int, screen_collisions: bool, threshold_rad=None, derived: bool = False,
          k_best: int = 4) -> dict[str, Any]:
    import torch
    from weld_motion_planner.geometry.seam_frame import Seam
    from weld_motion_planner.io.plan_request import read
    from weld_motion_planner.robot_cell.collision.scene import placement_matrix

    request = read(path)
    cell = request.cell
    placement = placement_matrix(request.placement)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    joints = {str(j.get("id")): j for j in (request.program.get("weld_joints") or [])}

    scene = None
    if screen_collisions:
        scene = _scene_for(request, cell, device)

    seams = []
    for spec in request.seams[:limit]:
        seam = Seam(spec, joints.get(str(spec.get("weld_joint_ref"))), device=device)
        try:
            seams.append(one_seam(cell, seam, placement, step_s=step_s,
                                  spin_bins=spin_bins, positioner_bins=positioner_bins,
                                  scene=scene,
                                  threshold_rad=threshold_rad, derived=derived,
                                  k_best=k_best))
        except Exception as exc:  # noqa: BLE001 - a bad seam should not lose the rest
            seams.append({"id": seam.id, "error": f"{type(exc).__name__}: {exc}"})
        print(f"  {seams[-1]['id']}: "
              f"{seams[-1].get('summary', {}).get('rate', seams[-1].get('error'))}")

    from weld_motion_planner.planner.dp_seam_search.engine.nodes import (
        CAUSE_NAMES,
        MOUNT_NOTE,
    )
    from weld_motion_planner.robot_cell.robot.tool import require as require_tool

    return {
        "schema": SCHEMA,
        "name": path.stem,
        "cell_id": cell.get("id"),
        "note": "Debug output. Nothing may depend on this; it may break at any commit.",
        "assumptions": [MOUNT_NOTE],
        "tool_axis": list(require_tool(str(cell.get("id") or "")).electrode_axis),
        # No torch dimensions and no mount offset. The bundle used to carry
        # both so the viewer could DRAW a gun bridging the 150 mm between the
        # flange and the weld -- a gap that was itself the bug (the planner
        # applying `body_length_mm` as an offset on top of a `link_6` mesh
        # that already ends at the electrode tip). The arm's own mesh is the
        # torch; there is nothing separate to draw.
        "cause_names": {str(k): v for k, v in CAUSE_NAMES.items()},
        "collision_screened": scene is not None,
        "workpiece": _workpiece(request, cell, placement),
        "seams": seams,
    }


def _workpiece(request, cell, placement) -> dict[str, Any] | None:
    """The part, tessellated for display and told which link it rides.

    Sent as vertices plus indices rather than expanded triangles: this bracket
    is 548 faces either way, and the indexed form is a third of the bytes and
    what three.js wants anyway.

    `work_to_piece` and `work_link` travel with it because WHERE it sits is
    not a property of the mesh. The part rides the positioner, so the viewer
    parents it to that link and the placement rides along for free -- which is
    the same trick `scene.py` plays with the part's distance field, and for
    the same reason.
    """
    from weld_motion_planner.geometry import mesh as meshmod

    try:
        # Coarser than the field's bake on purpose. `DEFAULT_DEFLECTION_M` is
        # 0.2 mm because a distance field is only as good as its triangles;
        # a picture is not, and nobody should wait on OCCT for pixels.
        part = meshmod.tessellate(request.step_bytes, deflection_m=0.001)
    except Exception as exc:  # noqa: BLE001 - a part that will not tessellate
        return {"error": f"{type(exc).__name__}: {exc}"}
    return {
        "vertices_m": np.round(part.vertices, 5).reshape(-1).tolist(),
        "indices": np.asarray(part.faces, dtype=np.int32).reshape(-1).tolist(),
        "triangles": len(part.faces),
        "work_link": (cell.get("frames") or {}).get("work"),
        "work_to_piece": np.asarray(placement, dtype=float).reshape(-1).tolist(),
    }


MESH_ROOT = ROOT.parents[1] / "robot_description" / "robots" / "rosie_1400_v3" / "meshes"


def _scene_for(request, cell, device):
    """The collision model: the arm against the part AND against the cell.

    It used to be the part alone, on the argument that "the part is what the
    torch is closest to by a wide margin while welding". That argument is
    false and a picture is what showed it -- the torch is near the part and the
    FOREARM is inside the positioner. Re-judging screened-feasible nodes with
    the verifier's exact meshes, 60% of them were collisions: the arm against
    itself, against the table it was welding on, against the h-frame, against
    the floor. `docs/issues/0006`.

    `scene.cell_obstacles` assembles the rest of the cell, split by frame --
    what rides the positioner is baked in the work frame so its field turns
    with it. With that and self-clearance the sample drops to zero.
    """
    from weld_motion_planner.geometry import mesh as meshmod
    from weld_motion_planner.robot_cell.collision import scene as scenemod
    from weld_motion_planner.robot_cell.collision import spheres as spheremod

    model = spheremod.load(str(cell.get("id")))
    if model is None:
        raise RuntimeError(f"no sphere model for cell {cell.get('id')!r}; "
                           "author one with `pixi run motion-spheres`")
    part = meshmod.tessellate(request.step_bytes)
    env = fixtures = fixtures_link = None
    if MESH_ROOT.is_dir():
        env, fixtures, fixtures_link = scenemod.cell_obstacles(
            cell, MESH_ROOT, held={"J9": 0.0})
    else:
        print(f"  no meshes at {MESH_ROOT}; screening against the part alone")
    return scenemod.build(cell, model.links, device=device,
                          placement=request.placement, part=part,
                          env=env, fixtures=fixtures, fixtures_link=fixtures_link)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    # A TIME, not a count: the edge rule is a rate limit, so a fixed count
    # per seam makes one threshold mean a different thing on every seam.
    parser.add_argument("--step-ms", type=float, default=250.0,
                        help="milliseconds of travel between samples")
    # Sized from the edge threshold rather than typed in. Bins further apart
    # than the threshold cannot be joined by any edge, so the search freezes
    # that axis at whatever bin layer 0 landed in -- `docs/issues/0004`, which
    # is what these two numbers being 12 and 9 against a 25 degree threshold
    # cost the corpus for a milestone. `None` means "ask `bins_for`".
    parser.add_argument("--spin", type=int, default=None)
    parser.add_argument("--positioner", type=int, default=None)
    parser.add_argument("--seams", type=int, default=6, help="per plan request")
    # ON by default, since the day a user looked at an unscreened bundle's
    # chosen path standing through the positioner beam and asked "did you turn
    # off collision again?". A dev page drawing paths the cell cannot run is
    # not a lighter demo, it is a wrong one; the honest lighter mode is the
    # explicit flag below.
    parser.add_argument("--collide", action=argparse.BooleanOptionalAction,
                        default=True,
                        help="screen against the whole cell (--no-collide to skip; "
                             "needs a sphere model for the cell)")
    parser.add_argument("--threshold-deg", type=float, default=25.0,
                        help="edge threshold; STATED, since no cell here has rate limits")
    parser.add_argument("--k-best", type=int, default=4)
    parser.add_argument("--only", nargs="*", default=None)
    parser.add_argument("--stated-threshold", action="store_true",
                        help="use --threshold-deg instead of the profile's rates")
    args = parser.parse_args(argv)

    from weld_motion_planner.planner.dp_seam_search.engine.graph import (
        bins_for,
        threshold_from_limits,
    )
    from weld_motion_planner.robot_cell.robot.profile import load as load_profile

    # DERIVED where the cell can say, stated where it cannot. The profile now
    # carries velocity limits, so the edge rule is `rate x step` per joint
    # rather than one number somebody typed -- and the binding axis is the
    # positioner at 60 deg/s, four times slower than the 25 degrees that used
    # to be stated implied.
    profile = load_profile("rosie_1400_v3")
    axes = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]
    rule = threshold_from_limits(profile, axes, args.step_ms / 1000.0)
    from_cell = rule is not None and not args.stated_threshold
    if from_cell:
        print("edge rule derived from the cell profile: "
              + ", ".join(f"{a} {np.rad2deg(v):.1f}deg" for a, v in zip(axes, rule)))
    else:
        rule = np.full(len(axes), np.deg2rad(args.threshold_deg))
    # Bins from the POSITIONER's own threshold, since it is the only axis whose
    # bin index is the state. Spin's bins are advisory -- see `graph.pinned_axes`.
    needed = bins_for(float(rule[-1]))
    if args.spin is None:
        args.spin = needed
    if args.positioner is None:
        # 90 over the capped 180 degree span -- 2 degrees apart. Finer than
        # the threshold's floor on purpose: the positioner is the redundancy
        # whose bin IS the state, and the resolution here is what the pages
        # show as genuinely different ways to hold the work.
        args.positioner = 90
    print(f"{args.threshold_deg:.0f} deg threshold needs {needed} bins "
          f"({360.0 / needed:.1f} deg apart); using spin {args.spin}, "
          f"positioner {args.positioner}, {args.step_ms:.0f} ms a sample")

    requests = sorted((ROOT / "data" / "motion").glob("*.weldplan"))
    if args.only:
        requests = [p for p in requests if p.stem in args.only]
    if not requests:
        print("make-graph-demo: no plan requests matched", file=sys.stderr)
        return 1

    for path in requests:
        print(f"{path.stem}:")
        try:
            payload = build(path, step_s=args.step_ms / 1000.0, spin_bins=args.spin,
                            positioner_bins=args.positioner, limit=args.seams,
                            collide=args.collide, threshold_rad=rule,
                            derived=from_cell, k_best=args.k_best)
        except Exception as exc:  # noqa: BLE001
            print(f"  skipped: {type(exc).__name__}: {exc}")
            continue
        out = OUT_ROOT / path.stem
        out.mkdir(parents=True, exist_ok=True)
        target = out / "graph.json"
        target.write_text(json.dumps(payload))
        print(f"  -> {target.relative_to(ROOT)} ({target.stat().st_size / 1024:.0f} KiB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
