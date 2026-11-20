#!/usr/bin/env python
"""Where the workpiece sits in the cell.

A weld program says what to weld and deliberately not where the part is; the
same program should run on whichever table is free. This is the other half, and
it lives in the PROJECT beside the torch and the placement record -- never in
`weld-program.v1`.

## Mount-relative, not world-relative

A placement is stored against the cell's **work frame**, not against the world:

    T_workpiece = T_work(joints) . Translate(offset) . Rpy(rpy) . Translate(-anchor)

Read right to left. The anchor is brought to the origin, the part is turned,
the result is offset across the table, and the whole thing rides whatever the
work frame is doing. That last factor is the point: `T_work` depends on J7, J8
and J9, so a part clamped to the table stays clamped when the positioner turns.
Store a world pose instead and it is correct at exactly one set of joint values
-- and wrong silently at every other, which is the worst way for it to be
wrong.

On `rosie_1400_v3` the work frame is the middle of the table's TOP FACE, not
the table link's origin on J7's axis (`cell.WORK_SURFACES`). That is the frame
the OLP authors offsets in, and with the link origin instead every offset it
sent landed 622 mm along the axis and 100 mm down -- inside the bar.

## The anchor

The CAD point that is placed AT the offset. Default: the horizontal centre of
the bounding box and its LOWEST z -- the middle of the part's footprint, on its
underside. That puts the part standing on the table rather than half through
it, which is the only default that is right often enough to be a default.

It comes from the tessellation's bounding box, which the mesh sidecar already
carries, so no geometry is re-read to compute it.

## Why a twin

`ui/src/state/placement.ts` mirrors this, pinned by
`tests/programmer/test_placement_parity.py`. The browser has to recompute the pose on
every drag of a slider and every nudge of an offset field, and the server has
to write the same pose into a bundle. Composition order is the thing that goes
wrong here, and it goes wrong LOOKING RIGHT: swap two factors and the part is
still on the table, still the right way up, and a hundred millimetres from
where it was asked to be.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from typing import Any

import numpy as np

from seam_worker.robot.kinematics import (
    apply,
    frame,
    invert,
    rpy_from_matrix,
    rpy_matrix,
    transform,
)

SCHEMA = "amr-weld-planner-v1.placement.v1"


class PlacementError(ValueError):
    """A placement this refuses to compute, with the reason."""


def anchor_from_bbox(lo: Any, hi: Any) -> list[float]:
    """The CAD point that lands on the offset: footprint centre, underside.

    Horizontal centre so the part is balanced on the table rather than hanging
    off one edge, and lowest z so it STANDS on the table. Anchoring on the
    box's true centre would bury half the part in the positioner, which looks
    like a modelling error rather than a placement default.
    """
    low = [float(v) for v in lo]
    high = [float(v) for v in hi]
    if len(low) != 3 or len(high) != 3:
        raise PlacementError("a bounding box needs three numbers at each corner")
    return [0.5 * (low[0] + high[0]), 0.5 * (low[1] + high[1]), low[2]]


def default_placement(anchor: Any, cell_id: str | None = None) -> dict[str, Any]:
    """Sitting on the middle of the table, unturned."""
    return {
        "schema": SCHEMA,
        "anchor_xyz_m": [float(v) for v in anchor],
        "offset_xyz_m": [0.0, 0.0, 0.0],
        "rpy_rad": [0.0, 0.0, 0.0],
        "cell_id": cell_id,
    }


def local_transform(placement: dict[str, Any]) -> np.ndarray:
    """Work frame to workpiece: `Translate(offset) . Rpy(rpy) . Translate(-anchor)`.

    The anchor translation is INSIDE the rotation, so turning the part spins it
    about its own anchor rather than swinging it around the table centre. Put
    it outside and a 90 degree yaw throws the part off the edge -- which still
    looks like a placement, just not the one anybody asked for.
    """
    offset = placement.get("offset_xyz_m") or [0.0, 0.0, 0.0]
    rpy = placement.get("rpy_rad") or [0.0, 0.0, 0.0]
    anchor = placement.get("anchor_xyz_m") or [0.0, 0.0, 0.0]
    return (
        transform(offset, rpy)
        @ transform([-float(v) for v in anchor], [0.0, 0.0, 0.0])
    )


def world_transform(
    cell: dict[str, Any], placement: dict[str, Any], joints: dict[str, float] | None = None
) -> np.ndarray:
    """Cell world to workpiece, at these joint values.

    The whole reason placement needs kinematics: `T_work` moves with J7, J8 and
    J9, so this answer is different for every pose of the positioner and a
    stored world pose would be right at exactly one of them.
    """
    return frame(cell, "work", joints) @ local_transform(placement)


def _clip_axis(poly: list[list[float]], axis: int, limit: float, keep_above: bool) -> list[list[float]]:
    """Clip a convex polygon against one axis-aligned half-plane, carrying z.

    Sutherland-Hodgman, one edge at a time. `z` is interpolated at each crossing
    because it is linear over a triangle, so the clipped polygon's vertices
    carry the true height of the face at the footprint's edge -- which is the
    number being looked for when a table runs out from under a part.
    """
    if not poly:
        return poly
    out: list[list[float]] = []
    for index, a in enumerate(poly):
        b = poly[(index + 1) % len(poly)]
        a_in = a[axis] >= limit if keep_above else a[axis] <= limit
        b_in = b[axis] >= limit if keep_above else b[axis] <= limit
        if a_in:
            out.append(a)
        if a_in != b_in:
            span = b[axis] - a[axis]
            t = 0.0 if span == 0.0 else (limit - a[axis]) / span
            out.append([a[i] + (b[i] - a[i]) * t for i in range(3)])
    return out


def _highest_over(support: Any, box: tuple[float, float, float, float]) -> float | None:
    """How high the support FACES reach inside a footprint, or None if none do.

    Faces, not vertices, and that is the difference between a part on a table
    and a part in one. A table's top is a large flat quad whose corners sit well
    outside a 200 mm part, so scanning support VERTICES inside the footprint
    finds nothing belonging to the surface the part is actually on and settles
    for whatever lower feature happens to have one. Measured on
    `rosie_1400_v3`: top face at +100 mm, highest vertex under the part +79 mm,
    part 20.6 mm inside the bar.

    Clipping each triangle to the footprint and taking the maximum over what
    survives is exact rather than an approximation: z is linear over a triangle,
    so its greatest value on any convex sub-region is attained at a vertex of
    that sub-region.

    `support` is triangles -- nine floats each -- in work-frame coordinates.
    """
    tris = np.asarray(support if support is not None else [], dtype=float).reshape(-1, 3, 3)
    if tris.size == 0:
        return None
    min_x, max_x, min_y, max_y = box
    best: float | None = None
    for tri in tris:
        # Cheapest test first: nothing this face could contribute beats what is
        # already found.
        if best is not None and float(tri[:, 2].max()) <= best:
            continue
        if float(tri[:, 0].min()) > max_x or float(tri[:, 0].max()) < min_x:
            continue
        if float(tri[:, 1].min()) > max_y or float(tri[:, 1].max()) < min_y:
            continue
        poly = [[float(c) for c in point] for point in tri]
        poly = _clip_axis(poly, 0, min_x, True)
        poly = _clip_axis(poly, 0, max_x, False)
        poly = _clip_axis(poly, 1, min_y, True)
        poly = _clip_axis(poly, 1, max_y, False)
        for point in poly:
            if best is None or point[2] > best:
                best = point[2]
    return best


def rest_on_table(
    placement: dict[str, Any],
    points: Any,
    support: Any = None,
) -> dict[str, Any]:
    """Set the part down: same x, y and turn, lowered until it lands.

    Two things this is not.

    It is not "set z to zero". The anchor stops being the bottom the moment the
    part is turned -- the default anchor is the footprint centre at the CAD's
    lowest z, so tipping a part 25 degrees drives a corner through the table
    while the anchor sits obediently on it.

    And it is not "rest on the work plane", which is what this did first and
    was wrong on the only cell that has a positioner. `positioner_table_a` is a
    200 mm square bar and its origin is the bar's CENTRELINE, not a face, so
    z = 0 of a work frame at that origin is 100 mm inside solid steel. That
    cell's work frame is the top face now (`cell.WORK_SURFACES`), but no other
    cell is promised one. A work frame is a
    mounting reference and nothing says it is a surface, so the surface is
    measured: `support` carries the cell's own FACES -- triangles, nine floats
    each -- in work-frame
    coordinates.

    The part rests on its own lowest point, and the surface under it is the
    highest cell geometry within the part's FOOTPRINT -- the projected extent
    of the turned part in x and y.

    A per-column height map would be more faithful and was the first attempt.
    It does not survive real tessellation: a flat plate's underside is two
    triangles, so its four vertices share almost no columns with the table's,
    and a part plainly sitting on the table finds nothing under it and drops to
    the work plane. Vertices are not a height field.

    The footprint is therefore an over-approximation on purpose. Where it is
    wrong it is wrong upward -- an L-shaped part with its low corner clear of a
    step is held at the step's height and floats. That is the safe direction:
    floating is visible, and a millimetre of steel through the table is not.

    Both are measured over POINTS, not bounding boxes. A rotated box contains
    the part but touches it only at its corners, so a round part dropped
    against its box hovers by the difference between the box's diagonal and the
    part's own radius. The tessellation leaves its own gap, a sagitta on curved
    faces -- the mesh tolerance, well below any fixture tolerance.

    With no support -- no cell, or a part parked clear of everything -- it
    falls back to the work plane, which is right for a cell whose work frame
    really is the table and is the only answer available otherwise.
    """
    array = np.asarray(points, dtype=float).reshape(-1, 3)
    if array.size == 0:
        return dict(placement)
    rpy = placement.get("rpy_rad") or [0.0, 0.0, 0.0]
    anchor = np.asarray(placement.get("anchor_xyz_m") or [0.0, 0.0, 0.0], dtype=float)
    offset = [float(v) for v in (placement.get("offset_xyz_m") or [0.0, 0.0, 0.0])]

    turned = (array - anchor) @ rpy_matrix(*rpy)[:3, :3].T
    turned[:, 0] += offset[0]
    turned[:, 1] += offset[1]
    lowest = float(turned[:, 2].min())

    box = (
        float(turned[:, 0].min()),
        float(turned[:, 0].max()),
        float(turned[:, 1].min()),
        float(turned[:, 1].max()),
    )
    surface = _highest_over(support, box)
    # Nothing underneath: the work plane, which is right for a cell whose work
    # frame is the table and is the only answer available otherwise.
    offset[2] = surface - lowest if surface is not None else -lowest
    return {**placement, "offset_xyz_m": offset}


def placement_from_world(
    cell: dict[str, Any],
    world: Any,
    anchor_pos_m: Any,
    joints: dict[str, float] | None = None,
) -> dict[str, Any]:
    """The inverse of `world_transform`: a placement record from a world pose.

    What a drag handle in the viewport needs. The gizmo knows only where it put
    the part; the document stores an offset and an rpy measured against the WORK
    frame, so the table's own transform has to come back out -- otherwise
    dragging a part and then moving the positioner would take it somewhere
    nobody dragged it to.

    The anchor is carried through unchanged rather than re-derived. It is a fact
    about the CAD -- the footprint centre of its bounding box -- and re-deriving
    it from a posed matrix would let a drag silently redefine which point of the
    part is "the" reference.

    Raises when the cell has no work frame, matching `world_transform` -- the
    TypeScript twin returns null there instead, which is that language's
    existing convention in `frameOf` and not a disagreement about the maths.
    """
    work = frame(cell, "work", joints)
    local = invert(work) @ np.asarray(world, dtype=float)
    anchor = [float(v) for v in np.asarray(anchor_pos_m, dtype=float).flatten()[:3]]
    # `local = Translate(offset) . Rpy(rpy) . Translate(-anchor)`, so the offset
    # is where the anchor LANDED, not the translation column.
    turned = apply(local, anchor)
    return {
        "anchor_xyz_m": anchor,
        "offset_xyz_m": [float(v) for v in turned],
        "rpy_rad": rpy_from_matrix(local),
    }


def workpiece_point(
    cell: dict[str, Any],
    placement: dict[str, Any],
    pos_m: Any,
    joints: dict[str, float] | None = None,
) -> list[float]:
    """A point of the CAD, in cell world coordinates."""
    return apply(world_transform(cell, placement, joints), pos_m)


# Defined in `weldplan`, not here, and re-exported so callers keep their name.
#
# A placement record is one of the four documents inside a `.weldplan`, so what
# counts as a well-formed one is contract, not authoring: the reader of the
# plan request has to be able to reject a malformed placement without importing this
# module and everything behind it -- numpy, kinematics, and the whole CAD stack
# one import further on. The check itself is shape only (three triples of finite
# numbers and an optional cell name), which needs none of that.
from weldplan.plan_request import placement_problems as problems


def self_test() -> int:
    # A cell whose work frame is a metre up and turned 90 degrees about Z, so
    # neither factor can hide behind an identity.
    cell = {
        "id": "t",
        "frames": {"base": "world", "work": "table"},
        "links": [{"name": "world"}, {"name": "table"}],
        "joints": [
            {
                "name": "T", "type": "revolute", "parent": "world", "child": "table",
                "xyz_m": [2, 0, 1], "rpy_rad": [0, 0, 0], "axis_xyz": [0, 0, 1],
                "limits_rad": {"lower": -4, "upper": 4},
            }
        ],
    }
    # A 1 m cube sitting from z=5 to z=6 in CAD, centred on x=10.
    anchor = anchor_from_bbox([9.5, -0.5, 5.0], [10.5, 0.5, 6.0])
    assert anchor == [10.0, 0.0, 5.0], anchor

    placement = default_placement(anchor)

    # The anchor lands exactly on the work frame's origin.
    assert np.allclose(workpiece_point(cell, placement, anchor), [2, 0, 1])
    # And the top of the cube is a metre above it, still the right way up.
    assert np.allclose(workpiece_point(cell, placement, [10.0, 0.0, 6.0]), [2, 0, 2])

    # Offsetting moves it across the table.
    shifted = {**placement, "offset_xyz_m": [0.3, 0.0, 0.0]}
    assert np.allclose(workpiece_point(cell, shifted, anchor), [2.3, 0, 1])

    # Turning spins it about its OWN anchor, not about the table centre: the
    # anchor does not move.
    turned = {**placement, "rpy_rad": [0.0, 0.0, math.pi / 2]}
    assert np.allclose(workpiece_point(cell, turned, anchor), [2, 0, 1], atol=1e-12)
    # ...and a point 0.5 m along +x of the anchor swings to +y.
    assert np.allclose(
        workpiece_point(cell, turned, [10.5, 0.0, 5.0]), [2, 0.5, 1], atol=1e-12
    )

    # The part RIDES the positioner. This is the claim the whole module exists
    # for: SAME placement, different joints, different world position -- so the
    # contrast with the un-turned table is the assertion, not the value alone.
    still = workpiece_point(cell, placement, [10.5, 0.0, 5.0])
    rolled = workpiece_point(cell, placement, [10.5, 0.0, 5.0], {"T": math.pi / 2})
    assert np.allclose(still, [2.5, 0, 1], atol=1e-12), still
    assert np.allclose(rolled, [2, 0.5, 1], atol=1e-12), rolled
    assert not np.allclose(still, rolled), "the part did not ride the table"

    assert problems(placement) == []
    assert problems({**placement, "rpy_rad": [0, 0]})

    # The inverse, which the drag handle depends on. Round-tripped through a
    # POSED table, so a work frame left out of the inverse cannot pass.
    for rpy in ([0.0, 0.0, 0.0], [0.3, -0.7, 1.1], [0.0, math.pi / 2, 0.4]):
        posed = {"schema": SCHEMA, "anchor_xyz_m": [10, 0, 5],
                 "offset_xyz_m": [0.2, -0.4, 0.05], "rpy_rad": rpy}
        joints = {"T": 0.6}
        world = world_transform(cell, posed, joints)
        back = placement_from_world(cell, world, posed["anchor_xyz_m"], joints)
        assert back is not None
        again = world_transform(cell, back, joints)
        assert np.allclose(again, world, atol=1e-9), (rpy, again - world)

    print("placement self-test ok: anchor on the table, spins about itself, rides the table")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--urdf", default="")
    parser.add_argument("--bbox", default="", help='[[minx,miny,minz],[maxx,maxy,maxz]]')
    parser.add_argument("--joints", default="{}")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not (ns.urdf and ns.bbox):
        raise ValueError("--urdf and --bbox are required, or --self-test")

    from pathlib import Path

    from seam_worker.robot.cell import frames_beside, read_urdf

    cell = read_urdf(Path(ns.urdf), frames_beside(Path(ns.urdf)))
    lo, hi = json.loads(ns.bbox)
    placement = default_placement(anchor_from_bbox(lo, hi))
    matrix = world_transform(cell, placement, json.loads(ns.joints))
    print(json.dumps({**placement, "world": [float(v) for v in matrix.flatten()]}, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"placement: {exc}", file=sys.stderr)
        raise SystemExit(1)
