#!/usr/bin/env python
"""Refit the arm's sphere model so the screen stops hallucinating contact.

The committed model was authored by `tools/sphere_tool`, whose radius step sets
each sphere to cover the surface voxels ASSIGNED to it. On a link that is two
regimes at once -- `link_6` is a bulky wrist casting plus a 150 mm, 15 mm
radius torch tube -- that rule is the defect: a centre buried in the wrist gets
tube surface assigned to it and grows to reach it, producing 47 mm of radius
standing off a 15 mm tube. Everything between the sphere and the tube reads as
the arm. With the tool frame fixed (the tip ON the weld, `12ed9f7d`), that
over-cover sits exactly where the work is, and the screen rejects 100% of
weld points. `docs/issues/0006`.

This fit refuses over-cover by construction: **a sphere's radius is its
centre's interior depth**, the distance to the link's own surface, so every
sphere is inscribed and the model never reports the arm somewhere the arm is
not. The trade is under-cover between spheres -- and that trade is the one the
module's own doctrine prices: "under-cover costs speed, not safety; the CCD
verifier re-checks the winner against exact meshes. Over-cover and plans get
conservative." The old fit bought completeness with conservatism; at the
weld, conservatism turned out to be blindness.

Selection is greedy set-cover over surface voxels: repeatedly take the
candidate covering the most surface not yet covered, stop at the budget or
when marginal gain dies. Small links stop early instead of burying redundant
spheres inside covered volume.

Writes `robot_description/<cell>/spheres.json` in place, same schema, re-stamping the
`provenance` hashes of the URDF and meshes it fitted against. Run
`pixi run motion-spheres` after to see the coverage report, and the corpus
with `--collide` to see what the screen now says.

Run: `pixi run -e motion python tools/fit_spheres.py`.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
URDF_ROOT = ROOT.parents[1] / "robot_description" / "robots"

ARM_LINKS = ["link_1", "link_2", "link_3", "link_4", "link_5", "link_6"]


def surface_points(vertices: np.ndarray, faces: np.ndarray, spacing: float) -> np.ndarray:
    """Points on the mesh surface, at most `spacing` apart, by triangle raster.

    Vertex clouds alone under-sample big flat triangles, and a voxelizer fed
    only vertices leaves wall-sized holes the flood fill then pours through --
    marking the interior outside and the fit fitting nothing.
    """
    a, b, c = (vertices[faces[:, i]] for i in range(3))
    out = [vertices]
    ab, ac = b - a, c - a
    # Subdivision counts per triangle from its longest edge.
    longest = np.maximum(np.linalg.norm(ab, axis=1), np.linalg.norm(ac, axis=1))
    longest = np.maximum(longest, np.linalg.norm(c - b, axis=1))
    steps = np.clip(np.ceil(longest / spacing).astype(int), 1, 64)
    for n in np.unique(steps):
        rows = steps == n
        # Barycentric lattice: (i/n, j/n) with i + j <= n.
        grid = [(i / n, j / n) for i in range(n + 1) for j in range(n + 1 - i)]
        u = np.asarray([g[0] for g in grid])
        v = np.asarray([g[1] for g in grid])
        pts = (a[rows, None, :] + ab[rows, None, :] * u[None, :, None]
               + ac[rows, None, :] * v[None, :, None])
        out.append(pts.reshape(-1, 3))
    return np.concatenate(out, axis=0)


def fit_link(vertices: np.ndarray, faces: np.ndarray, *, budget: int,
             voxel: float, exclude_near: np.ndarray | None = None,
             exclude_radius: float = 0.0) -> tuple[list[dict], dict]:
    from scipy import ndimage

    pts = surface_points(vertices, faces, voxel * 0.5)
    lo = pts.min(axis=0) - 2 * voxel
    hi = pts.max(axis=0) + 2 * voxel
    shape = np.maximum(((hi - lo) / voxel).astype(int) + 1, 3)

    surface = np.zeros(shape, dtype=bool)
    index = ((pts - lo) / voxel).astype(int)
    surface[index[:, 0], index[:, 1], index[:, 2]] = True

    # Outside is what a flood from the boundary reaches without crossing the
    # surface; inside is the rest. Label connected components of ~surface and
    # keep every component that never touches the box wall.
    open_space = ~surface
    labels, _ = ndimage.label(open_space)
    edge = np.unique(np.concatenate([
        labels[0].ravel(), labels[-1].ravel(),
        labels[:, 0].ravel(), labels[:, -1].ravel(),
        labels[:, :, 0].ravel(), labels[:, :, -1].ravel()]))
    inside = open_space & ~np.isin(labels, edge)

    # Depth: distance from each interior voxel to the nearest non-interior
    # one, in metres. This IS the inscribed radius at that centre, up to a
    # voxel of quantisation, which is subtracted rather than added when the
    # radius is written -- the whole point is never to stand off the surface.
    depth = ndimage.distance_transform_edt(inside | surface, sampling=voxel)

    cand = np.argwhere(inside)
    if len(cand) == 0:
        return [], {"surface_voxels": int(surface.sum()), "covered": 0}
    cand_r = depth[cand[:, 0], cand[:, 1], cand[:, 2]]
    # Order by depth so the greedy loop tries fat, central spheres first.
    order = np.argsort(-cand_r)
    cand, cand_r = cand[order], cand_r[order]
    cand_xyz = lo + (cand + 0.5) * voxel

    surf_idx = np.argwhere(surface)
    surf_xyz = lo + (surf_idx + 0.5) * voxel
    covered = np.zeros(len(surf_xyz), dtype=bool)

    # A region handed to a DIFFERENT model. `link_6` bakes the torch into its
    # mesh, and a fitted sphere is the wrong shape for a torch: the fit sees a
    # 15 mm tube and covers it with 15 mm balls, but the thing near the work
    # is a nozzle of half that radius, so every fitted sphere near the tip
    # reads contact the real gun does not make. The tip region is excluded
    # here -- from candidates AND from the surface the fit is asked to cover,
    # so the greedy loop does not grow a distant sphere to reach it -- and the
    # caller appends `torch_spheres`' analytic set in its place.
    if exclude_near is not None and exclude_radius > 0:
        keep = np.linalg.norm(cand_xyz - exclude_near, axis=1) > exclude_radius
        cand, cand_r, cand_xyz = cand[keep], cand_r[keep], cand_xyz[keep]
        covered |= np.linalg.norm(surf_xyz - exclude_near, axis=1) <= exclude_radius

    chosen: list[dict] = []
    slack = voxel * np.sqrt(3) / 2
    while len(chosen) < budget and not covered.all():
        best_gain, best_at = 0, -1
        # Evaluate a moving window of the deepest untried candidates; the
        # full candidate set is hundreds of thousands and the gain of a
        # shallow sphere cannot beat a deep one covering fresh surface.
        for at in range(min(len(cand), 4000)):
            r = cand_r[at]
            if r <= voxel:
                continue
            d = np.linalg.norm(surf_xyz[~covered] - cand_xyz[at], axis=1)
            gain = int((d <= r + slack).sum())
            if gain > best_gain:
                best_gain, best_at = gain, at
        if best_at < 0 or best_gain == 0:
            break
        centre = cand_xyz[best_at]
        radius = float(max(cand_r[best_at] - 0.5 * voxel, voxel))
        d = np.linalg.norm(surf_xyz - centre, axis=1)
        covered |= d <= cand_r[best_at] + slack
        chosen.append({"center_m": [round(float(v), 6) for v in centre],
                       "radius_m": round(radius, 6)})
        # Drop candidates buried inside the sphere just placed: a centre
        # within half its radius adds nothing but budget spend.
        keep = np.linalg.norm(cand_xyz - centre, axis=1) > cand_r[best_at] * 0.5
        cand, cand_r, cand_xyz = cand[keep], cand_r[keep], cand_xyz[keep]
        if len(cand) == 0:
            break

    report = {"surface_voxels": int(len(surf_xyz)),
              "covered": int(covered.sum()),
              "coverage": round(float(covered.mean()), 4)}
    return chosen, report


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cell", default="rosie_1400_v3")
    parser.add_argument("--budget", type=int, default=50, help="spheres per link")
    parser.add_argument("--voxel-mm", type=float, default=4.0)
    # The WORKING END by default. The over-cover that blinds the screen sits
    # where the arm meets the part -- the forearm, wrist and torch tube -- and
    # an inscribed fit needs a closed mesh: `link_2` is an open shell with no
    # interior at any voxel size, so it KEEPS the authored fit, which never
    # put a sphere anywhere near a weld.
    parser.add_argument("--links", nargs="*",
                        default=["link_3", "link_4", "link_5", "link_6"])
    # For a model somebody AUTHORED and wants kept: touch nothing except the
    # tip zone. A hand fit covers the torch tube with tube-radius balls all
    # the way down, and the last of them stands inside the part at the true
    # weld pose -- measured at 8 mm on the 2026-08-13 hand model, which took
    # the screen from working back to 0.0% of weld points. The splice removes
    # only the spheres within the torch zone of the tip and puts the analytic
    # nozzle-and-body set there, wire left out, exactly as the full refit
    # does. Everything outside the zone stays byte-identical to the author's.
    parser.add_argument("--splice-torch-only", action="store_true",
                        help="do not refit anything; replace link_6's tip-zone "
                             "spheres with the analytic torch and keep the rest")
    # For a URDF edit that does not move a mesh: the spheres still fit, and
    # saying so is a deliberate act with a look at the change behind it,
    # which is what the provenance test is asking for.
    parser.add_argument("--restamp", action="store_true",
                        help="write today's URDF and meshes hashes into provenance "
                             "and change nothing else")
    args = parser.parse_args(argv)

    from weld_motion_planner.io.plan_request import read
    from weld_motion_planner.robot_cell.collision.spheres import torch_spheres
    from weld_motion_planner.verifier.verify import kinematics, load_dae
    from weldplan.plan_request import read_request as read_package

    # The tip, in link_6's own frame, from the cell's kinematics at zero --
    # the joint chain link_6 -> end_effector -> tool0 is fixed, so any pose
    # gives the same answer. And a tooling document for the analytic torch.
    source = next((ROOT / "data" / "motion").glob("*.weldplan"))
    plan = read(source)
    # The tooling document, opened here rather than off `PlanRequest`: the gun's
    # analytic dimensions are what AUTHORS this sphere model, and the planner
    # that reads the model has no use for them.
    tooling = read_package(source.read_bytes())["tooling"]
    kin = kinematics(plan.cell)
    frames = kin.transforms([0.0] * len(kin.axis_names))
    tool_in_l6 = np.linalg.inv(frames["link_6"]) @ frames["tool0"]
    tip = tool_in_l6[:3, 3]
    # torch_spheres' local frame is +Z along the electrode AWAY from the weld,
    # origin at the tip; the electrode in link_6's frame is tool0's +X.
    away = -tool_in_l6[:3, 0]
    TORCH_ZONE_M = 0.060

    voxel = args.voxel_mm / 1000.0
    target = URDF_ROOT / args.cell / "spheres.json"
    document = json.loads(target.read_text())

    def write(reason: str) -> None:
        """The model, with the hashes it was fitted from."""
        import hashlib

        from weldplan.robot_description import mesh_sha256

        document["provenance"] = {
            "robot_urdf_sha256": hashlib.sha256(
                (URDF_ROOT / args.cell / "robot.urdf").read_bytes()).hexdigest(),
            # The REGISTERED meshes (`robot_description_manifest.json`), hashed the way the
            # description's own identity is: a mesh the robot does not claim
            # cannot make a sphere model look stale, and one it does claim
            # cannot change without this number moving.
            "meshes_sha256": mesh_sha256(URDF_ROOT / args.cell),
            **{k: v for k, v in (document.get("provenance") or {}).items()
               if k not in ("robot_urdf_sha256", "meshes_tree", "meshes_sha256")},
        }
        target.write_text(json.dumps(document, indent=1) + "\n")
        print(f"-> {target} ({reason})")

    if args.restamp:
        write("provenance only")
        return 0

    def torch_set() -> list[dict]:
        stickout = float((tooling or {}).get("stickout_mm", 15.0)) / 1000.0
        out = []
        for centre, radius in torch_spheres(tooling):
            if centre[2] < stickout - 1e-9:
                continue  # the wire is meant to touch the weld
            point = tip + away * centre[2]
            out.append({"center_m": [round(float(v), 6) for v in point],
                        "radius_m": round(float(radius), 6)})
        return out

    if args.splice_torch_only:
        kept, dropped = [], []
        for sphere in document["links"].get("link_6", []):
            gap = float(np.linalg.norm(np.asarray(sphere["center_m"]) - tip))
            (dropped if gap <= TORCH_ZONE_M else kept).append(sphere)
        document["links"]["link_6"] = kept + torch_set()
        print(f"link_6: kept {len(kept)} authored spheres, replaced {len(dropped)} "
              f"in the tip zone with {len(torch_set())} analytic torch spheres")
        write("torch splice")
        return 0

    # `--cell`'s OWN meshes. This was a constant naming rosie_1400_v3 while
    # `--cell` chose which asset to overwrite, so `--cell rosie_1420_v1` fitted
    # the 1400's castings and wrote them into the 1420's file -- three of its
    # six links are the same part and the other three are not.
    mesh_root = URDF_ROOT / args.cell / "meshes"
    if not mesh_root.is_dir():
        print(f"no meshes for {args.cell} at {mesh_root}", file=sys.stderr)
        return 1
    for name in args.links:
        mesh = load_dae(mesh_root / f"{name}.dae")
        near = tip if name == "link_6" else None
        spheres, report = fit_link(np.asarray(mesh.vertices),
                                   np.asarray(mesh.faces, dtype=np.int64),
                                   budget=args.budget, voxel=voxel,
                                   exclude_near=near, exclude_radius=TORCH_ZONE_M)
        if not spheres:
            print(f"{name}: no interior at {args.voxel_mm:.0f} mm -- open mesh? "
                  "keeping the authored fit")
            continue
        if name == "link_6":
            # The analytic torch, in place of the fitted blobs the zone kept
            # out. The WIRE is left off: the electrode is meant to touch the
            # weld, and a screen that models the consumable rejects the exact
            # contact welding exists to make. The nozzle and body go in --
            # they are what must NOT touch, and at a 45 degree work angle the
            # nozzle's own radius clears a flat plate by about 8 mm, which is
            # the standoff doing its job.
            stickout = float((tooling or {}).get("stickout_mm", 15.0)) / 1000.0
            for centre, radius in torch_spheres(tooling):
                if centre[2] < stickout - 1e-9:
                    continue  # the wire
                point = tip + away * centre[2]
                spheres.append({"center_m": [round(float(v), 6) for v in point],
                                "radius_m": round(float(radius), 6)})
        document["links"][name] = spheres
        radii = [s["radius_m"] * 1000 for s in spheres]
        print(f"{name}: {len(spheres)} spheres, r {min(radii):.0f}..{max(radii):.0f} mm, "
              f"{report['coverage']:.1%} of surface within slack")

    write(f"refit {', '.join(args.links)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
