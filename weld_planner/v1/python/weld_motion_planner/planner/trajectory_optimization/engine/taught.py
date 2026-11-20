"""Taught destinations use full TCP poses, not the weld search's free spin.

Joint moves are synchronized rest-to-rest Hermites. Cartesian moves solve
the authored line/arc with the existing IK and default to constant TCP cruise
between rest-to-rest ramps. The interpolated path passes the same continuous
collision/rate verifier as other motion.
"""
from __future__ import annotations

import copy
import numpy as np
import torch
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation

from weld_motion_planner.robot_cell.robot import fk, ik
from weld_motion_planner.robot_cell.robot.mobility import jacobian
from weld_motion_planner.verifier.hermite_wire import hermite_wire_degrees
from weld_motion_planner.verifier.verify import kinematics, verify_free_space


# A Home in a program with no weld block is planned as the move operators
# taught by hand until 2026-10-11 ("record a return waypoint instead of
# Home"): a MoveJ to the reset pose at a freshly recorded waypoint's defaults
# (speed scale 1, acceleration scale 1, and the TCP bound the UI shows on the
# Home row, ui/src/program-v2/move-defaults.ts HOME_WAYPOINT_SPEED_MM_S; 25 mm/s
# until 2026-10-11), judged by the same continuous collision and limit verifier. No
# obstacle search: a direct move that is not clear fails its verdict and the
# dense export refuses it. Chosen by program content, not by machine, so a
# program plans the same on the CPU planner (motion-cpu has no Warp, so no
# free-space search; the relay runs it) and on a GPU one. A program with welds
# builds the planner's collision model and searches its Home as before.
HOME_AS_WAYPOINT = {"motion": "joint", "speed_scale": 1.0, "acceleration_scale": 1.0, "speed_mm_s": 250.0}

# A loop-return Home (OLP Run program loop mode, owner 2026-10-11: "run the
# program in a continuous loop"): a Home whose destination is the plan's
# first configuration instead of the reset pose, appended by the client as
# the program's last node when the operator plans with a repeat selected, so
# the next cycle starts where this one did. Planned exactly as a Home (the
# verified waypoint MoveJ without a planner collision model, else the
# free-space search), and skipped when the program already ends within
# LOOP_RETURN_SAME_RAD of its start. The segment's source id is the node id.
LOOP_RETURN = "rosie.loop_return"
# Unverified: far below any drive's completion tolerance (a Rosie 1430 arm: 0.034 deg =
# 5.9e-4 rad); a leg shorter than this is not a move.
LOOP_RETURN_SAME_RAD = 1e-6


def is_loop_return(node):
    return node.get("op") == "home" and bool((node.get("extensions") or {}).get(LOOP_RETURN))


# Ops whose legs the exact verifier alone can plan: no planner collision
# model (ESDF, Warp) is built for a program made only of these. An `io` row
# moves nothing.
DIRECT_OPS = ("move", "dwell", "home", "io")

# The tube a taught MoveL/MoveC must provably stay in, continuously, between
# and at every IK knot (`_certify_pose_path`). Owner decision 2026-10-12
# 00:47 PDT: the robot's quoted repeatability is 0.05 mm (or even 0.01 mm),
# so the plan must hit at least that -- 0.01 mm, 0.1 mm before. Measured on
# a Rosie 1430 arm's `test1` (2026-10-12): the planned spline is 0.05-0.11 um off the
# line at 25 mm/s and 0.24-2.6 um at 250-400 mm/s, so the 67-73 coarse knots
# still pass at the first try.
POSITION_TOLERANCE_M = 1e-5
# Orientation: repeatability (ISO 9283 RP) quotes TCP position only, and the
# TCP's position is bounded above. The angle moves a point d beyond the TCP
# (wire stick-out, nozzle) by d x angle: 0.001 rad keeps a point 10 mm out
# within the same 0.01 mm, and is 0.06 deg against degree-level torch-angle
# tolerances. Unchanged since 2026-10-11; not tightened because nothing that
# repeatability quotes depends on it.
ORIENTATION_TOLERANCE_RAD = 1e-3

# IK knots along a taught line/arc. The first try is one knot per 10 mm /
# 0.02 rad (at least 65); each refusal of the continuous tracking
# certificate halves the spacing, down to the 0.25 mm / 0.0025 rad
# every move used before 2026-10-11 and its 10000-interval budget. Measured
# on a Rosie 1430 arm's `test1` (four 0.42-0.70 m lines, 2026-10-11): every line passed
# at the first try with 67-73 knots, 0.0001 mm at worst off the line (the
# verifier's own FK, 40 points per interval), the same move durations,
# where 0.25 mm meant 1683-2785 knots. The knot count is what every later
# stage pays for -- the certificate, the rate judge, the collision verifier
# and its clearance report, and a 4.1 MB result: 63 s of plan on the relay.
# The tracking certificate, not the spacing, is the guarantee.
COARSE_KNOT_SPACING_M = 0.01
COARSE_KNOT_SPACING_RAD = 0.02
FINE_KNOT_SPACING_M = 0.00025
FINE_KNOT_SPACING_RAD = 0.0025
MAX_KNOT_INTERVALS = 10000


class TrackingRefused(ValueError):
    """The interpolated taught path left the POSITION_TOLERANCE_M /
    ORIENTATION_TOLERANCE_RAD tube (or could not be proved inside it) at this
    knot spacing."""


def tolerance_text():
    return f"{POSITION_TOLERANCE_M * 1000:g} mm / {ORIENTATION_TOLERANCE_RAD:g} rad"


def knot_counts(length, angular):
    """Knot counts to try for a taught line/arc, coarse first, finest last."""
    def count(spacing_m, spacing_rad):
        return max(65, int(np.ceil(max(length / spacing_m, angular / spacing_rad))) + 1)
    finest = count(FINE_KNOT_SPACING_M, FINE_KNOT_SPACING_RAD)
    counts = [min(count(COARSE_KNOT_SPACING_M, COARSE_KNOT_SPACING_RAD), finest)]
    while counts[-1] < finest:
        counts.append(min(2 * (counts[-1] - 1) + 1, finest))
    counts = [c for c in counts if c <= MAX_KNOT_INTERVALS + 1]
    if not counts:
        raise ValueError("Taught Cartesian move exceeds the 10000-interval planning budget")
    return counts


def in_program_order(request):
    """`weldplan.measured_start.in_program_order` for a planning request."""
    from weldplan.measured_start import in_program_order as ordered
    return ordered(request.program, getattr(request, "start_state", None))


def start_configuration(request, cell):
    """The plan's first configuration: the measured start when the request
    carries one, else the model reset pose (offline preview, plan from the
    program's start)."""
    start = getattr(request, "start_state", None)
    if start is None:
        return np.asarray([cell.profile.reset_pose[n] for n in cell.moved_axes], dtype=float)
    from weldplan.measured_start import validate
    validate(start, request.cell, request.program, getattr(request, "io_outputs", None))
    return joint_target(start, cell.moved_axes, cell.profile.held)


def connect_taught_program(request, reports, options, context, cell, roadmap=None):
    """Every free-space leg of an in-order program, from its first configuration.

    Legs are planned one node at a time from `here`, which starts at
    `start_configuration` and follows each node's end: a taught move ends at
    its destination, a weld at its last verified knot, an approach or
    transit at the next weld's first knot, a Home at the reset pose. Welds
    are spliced as verified (`here` must equal their first knot). `roadmap`,
    when the caller built one, seeds the free-space legs with a collision-free
    route as the weld-only connector does; the solve and verifier are the
    same either way."""
    from weldplan.measured_start import refuse_io
    from .freespace import MoveSpec, REWIND_ATTACH, plan_moves, move_json, roadmap_path, standoff_batch
    reset = np.asarray([cell.profile.reset_pose[n] for n in cell.moved_axes], dtype=float)
    here = start_configuration(request, cell)
    first = here.copy()
    rest = np.zeros_like(reset)
    nodes = [node for node in request.program["nodes"] if node.get("enabled", True)]
    authored_steps = {node["id"]: i for i, node in enumerate(request.program["nodes"], 1)}
    refuse_io(nodes, getattr(request, "io_outputs", None))
    welds = {r.seam_id: r for r in reports}
    out = []
    previous_weld = None

    def route_for(start, end):
        if roadmap is None:
            return None
        return roadmap_path(roadmap, context.planner_collision_model, context.device, start, end,
                            held=dict(cell.profile.held), attach=REWIND_ATTACH)

    from weldplan.taught_weld import spans
    weld_spans = spans(nodes)
    skipped = set()
    for i, node in enumerate(nodes):
        if node["id"] in skipped:
            continue
        if i in weld_spans:
            group = weld_spans[i]
            entries, here = plan_taught_weld(request, group, here, cell, context.verifier_collision_model, options.verify_margin_m)
            out.extend(entries)
            skipped.update(n["id"] for n in group)
            previous_weld = None
            continue
        op = node["op"]
        # Events execute at rest where the arm is; the dense export gives
        # each its segment (and an `io` row its output markers).
        if op in ("dwell", "io"):
            continue
        if op == "move":
            try:
                entry, here = plan_taught(request, node, here, cell, context.verifier_collision_model, options.verify_margin_m)
            except ValueError as exc:
                # A program-wide failure otherwise loses the destination: a
                # later MoveL refusal looks like a problem reaching the start.
                # Add authored context only after the leg exhausts its existing
                # solve/refinement behavior; retain the original refusal.
                motion = {"joint": "MoveJ", "linear": "MoveL", "circular": "MoveC"}.get(node.get("motion"), "Move")
                label = str(node.get("label") or "").strip()
                destination = f" to {label}" if label else ""
                raise ValueError(f"{motion}{destination} (program step {authored_steps[node['id']]}, "
                                 f"node {node['id']}): {exc}") from exc
            out.append(entry)
            previous_weld = None
            continue
        if op == "weld":
            report = welds.get(node["geometry"].get("id"))
            if report is None:
                raise ValueError(f"Weld {node['id']} has no verified trajectory")
            if not np.allclose(here, report.knots[0], atol=1e-8, rtol=0):
                raise ValueError(f"Weld {node['id']} is not connected to the preceding move")
            here = report.knots[-1].copy()
            previous_weld = report.seam_id
            continue
        home = first if is_loop_return(node) else reset
        end, to_id = home, None
        via = standoff_batch(request, [here], context.device, cell=cell)[0] if previous_weld else []
        following = next((n for n in nodes[i+1:] if n["op"] not in ("dwell", "io")), None)
        if op == "retract" and following is not None and following["op"] != "home":
            # An internal weld-block boundary is a local retreat, not a trip
            # home before the next authored move. Keep the established terminal
            # return-home behavior; an explicit Home also remains authoritative.
            if not via:
                raise ValueError(f"Retract {node['id']} has no reachable departure standoff")
            end, via = via[0], []
        if op in ("approach", "transit"):
            if following is None or following["op"] != "weld":
                raise ValueError(f"{op} has no destination weld")
            to_id = following["geometry"].get("id")
            if to_id not in welds:
                raise ValueError(f"Weld {to_id} could not be planned")
            end = welds[to_id].knots[0]
            via += standoff_batch(request, [end], context.device, cell=cell)[0]
        if op == "home" and np.array_equal(here, home):
            continue
        if is_loop_return(node) and np.max(np.abs(here - home)) <= LOOP_RETURN_SAME_RAD:
            continue
        if op == "home" and context.planner_collision_model is None:
            held = dict(cell.profile.held)
            pose = {"joint_names": [*cell.moved_axes, *held],
                    "joint_values_rad": [*home.tolist(), *held.values()]}
            # The reset pose is the program's own robot's, never a capture; a
            # loop return's destination is this plan's own first configuration.
            robot = getattr(request, "program", {}).get("robot") or {}
            capture = {key: robot[key] for key in ("model_id", "robot_description_sha256") if key in robot}
            entry, here = plan_taught(request, {**HOME_AS_WAYPOINT, "id": node["id"], "target": pose, "capture": capture},
                                      here, cell, context.verifier_collision_model, options.verify_margin_m)
            entry["authored_motion"] = "loop_return" if is_loop_return(node) else "home"
            out.append(entry)
            previous_weld = None
            continue
        spec = MoveSpec(op if op != "home" else "transit", previous_weld, to_id, here, rest, end, rest,
                        via=via, route=route_for(here, end), pin_start=True, pin_end=True)
        planned = plan_moves(request, context.planner_collision_model, context.device, specs=[spec],
                             held=dict(cell.profile.held), cell=cell, backend=options.backend,
                             verifier_collision_model=context.verifier_collision_model,
                             verify_margin_m=options.verify_margin_m, progress=context.progress)
        entry = move_json(planned[0]); entry["node_id"] = node["id"]
        out.append(entry); here = end.copy(); previous_weld = None
    return out


def joint_target(pose, names, held=None):
    stated = dict(zip(pose["joint_names"], pose["joint_values_rad"], strict=True))
    held = held or {}
    if set(stated) != set(names) | set(held):
        raise ValueError("Taught pose joint names do not match the planning cell")
    for name, value in held.items():
        if abs(stated[name] - value) > 1e-8:
            raise ValueError(f"Taught pose changes held axis {name}; select a matching planning profile")
    return np.asarray([stated[n] for n in names], dtype=float)


def cartesian_path(start_pos, start_rot, target, via=None):
    """Position/orientation by path fraction, and conservative derivative bounds.

    `length` is |dp/ds| and `position.curvature` |d2p/ds2| (0 on a line,
    length x sweep on an arc); `angular` bounds the slerp's rate and
    `rotation.breaks` are the fractions where that rate changes (the via):
    the certificate's second-order bound must not straddle one.
    """
    end = np.asarray(target["xyz_m"], dtype=float)
    rotations = [Rotation.from_matrix(start_rot), Rotation.from_euler("xyz", target["rpy_rad"])]
    fractions = [0., 1.]
    if via is None:
        length = float(np.linalg.norm(end - start_pos))
        position = lambda s: start_pos + np.asarray(s)[:, None] * (end - start_pos)
        position.curvature = 0.
    else:
        middle = np.asarray(via["xyz_m"], dtype=float)
        a, b = middle - start_pos, end - start_pos
        normal = np.cross(a, b)
        area = float(np.dot(normal, normal))
        if area <= 1e-20:
            raise ValueError("Circular move needs three distinct non-collinear TCP positions")
        centre = start_pos + (np.dot(a, a) * np.cross(b, normal) + np.dot(b, b) * np.cross(normal, a)) / (2 * area)
        radius = float(np.linalg.norm(start_pos - centre))
        x = (start_pos - centre) / radius
        y = np.cross(normal / np.sqrt(area), x)
        def angle(point):
            delta = (point - centre) / radius
            return float(np.arctan2(delta @ y, delta @ x) % (2 * np.pi))
        through, sweep = angle(middle), angle(end)
        if not 0 < through < sweep:
            raise ValueError("Circular via pose does not define an ordered arc")
        length = radius * sweep
        position = lambda s: centre + radius * (np.cos(np.asarray(s)[:, None] * sweep) * x + np.sin(np.asarray(s)[:, None] * sweep) * y)
        position.curvature = length * sweep
        fractions = [0., through / sweep, 1.]
        rotations.insert(1, Rotation.from_euler("xyz", via["rpy_rad"]))
    rotations = Rotation.concatenate(rotations)
    angular = max(float((rotations[i].inv() * rotations[i + 1]).magnitude()) / (fractions[i + 1] - fractions[i]) for i in range(len(fractions) - 1))
    return position, slerp(fractions, rotations), length, angular


def slerp(fractions, rotations):
    """SciPy's `Slerp(fractions, rotations)` as matrices: `R_k exp(w log(R_k^T R_k+1))`.

    The same interpolant, evaluated with a vectorised Rodrigues formula:
    SciPy's per-call object overhead was 0.4 s of a 5 s `test1` plan on
    the relay (2026-10-11), called once per certificate batch.
    """
    fractions = np.asarray(fractions, dtype=float)
    keys = rotations.as_matrix()
    steps = np.stack([(rotations[i].inv() * rotations[i + 1]).as_rotvec() for i in range(len(fractions) - 1)])

    def at(s):
        s = np.atleast_1d(np.asarray(s, dtype=float))
        k = np.clip(np.searchsorted(fractions, s, side="right") - 1, 0, len(fractions) - 2)
        v = steps[k] * ((s - fractions[k]) / (fractions[k + 1] - fractions[k]))[:, None]
        theta = np.linalg.norm(v, axis=1)
        small = theta < 1e-6
        safe = np.where(small, 1., theta)
        first = np.where(small, 1. - theta**2 / 6., np.sin(safe) / safe)
        second = np.where(small, .5 - theta**2 / 24., (1. - np.cos(safe)) / safe**2)
        skew = np.zeros((len(v), 3, 3))
        skew[:, 0, 1], skew[:, 0, 2], skew[:, 1, 2] = -v[:, 2], v[:, 1], -v[:, 0]
        skew[:, 1, 0], skew[:, 2, 0], skew[:, 2, 1] = v[:, 2], -v[:, 1], v[:, 0]
        step = np.eye(3) + first[:, None, None] * skew + second[:, None, None] * (skew @ skew)
        return keys[k] @ step
    at.breaks = fractions[1:-1]
    return at


def rotation_angle(a, b):
    """Angle of `a^T b` for stacks of rotation matrices, in closed form.

    `|a - b|_F = 2 sqrt(2) sin(theta / 2)` for rotations, and the arcsine is
    well conditioned at the small angles a 0.001 rad tolerance judges.
    SciPy's `from_matrix(...).inv() * ...` gave the same angle after an SVD
    per matrix: 1.7 s of a 4.4 s `test1` plan (2026-10-11).
    """
    return 2 * np.arcsin(np.minimum(1., np.linalg.norm(a - b, axis=(1, 2)) / (2 * np.sqrt(2))))


def _certify_pose_path(chain, link, order, knots, tangents, u, position, rotation, length, angular, device,
                       progress=None, slope_bound=1.5, curvature_bound=6.):
    """Bound error over every Hermite interval, including between IK samples.

    The pose error e between the joint spline's TCP and the taught path is
    evaluated at both ends of each sub-interval (width w, local coordinates)
    and bounded inside it by the smaller of two rigorous bounds:

    - first order, e Lipschitz with L: |e| <= (|e(lo)| + |e(hi)| + L w) / 2.
      A revolute joint moves the tip by at most chain reach times its angle;
      the triangle inequality adds the path's own rate.
    - second order, e C1 with |e''| <= M: |e| <= max(|e(lo)|, |e(hi)|) +
      M w^2 / 8 (linear interpolation's remainder, valid for vectors). The
      tip of a revolute chain has |p''| <= reach (|q''|_1 + 2 |q'|_1^2) and
      |R''|_F <= sqrt2 (|q''|_1 + 2 |q'|_1^2); the path adds |p''(s)| s'^2 +
      |p'(s)| |s''| and sqrt2 (angular^2 s'^2 + angular |s''|), with s' and
      s'' bounded by `slope_bound` and `curvature_bound` (per unit phase).
      Orientation goes through |R_fk - R_path|_F = 2 sqrt2 sin(angle / 2),
      and only on intervals no slerp break lies inside (its rate jumps).

    The first-order bound alone needs ~1/tolerance leaves: at 0.01 mm that
    is ~10x the ~65 k FK rows a `test1` line cost at 0.1 mm, past the budget
    below on its 73-knot line whatever the knot count. The second-order one
    needs ~1/sqrt(tolerance). Subdivision tightens the bound; it never turns
    a sample check into a claim of continuous coverage.
    """
    reach = float(np.linalg.norm(chain.origin_pos, axis=1).sum())
    h = np.diff(u)
    q0 = knots[:-1]
    d0, d1 = tangents[:-1] * h[:, None], tangents[1:] * h[:, None]
    a, b, c = 2*q0-2*knots[1:]+d0+d1, -3*q0+3*knots[1:]-2*d0-d1, d0
    # |q'|_1 and |q''|_1 in local interval coordinates, bounded on [0, 1].
    rate = (3*np.abs(a[:, order]) + 2*np.abs(b[:, order]) + np.abs(c[:, order])).sum(axis=1)
    accel = (6*np.abs(a[:, order]) + 2*np.abs(b[:, order])).sum(axis=1)
    s1, s2 = slope_bound * h, curvature_bound * h * h
    lipschitz_p = reach*rate + length*s1
    lipschitz_r = rate + angular*s1
    second_p = reach*(accel + 2*rate**2) + position.curvature*s1**2 + length*s2
    second_f = np.sqrt(2) * (accel + 2*rate**2 + angular**2 * s1**2 + angular*s2)
    clock = progress if progress is not None else (lambda phase: phase*phase*(3-2*phase))
    s_knots = clock(u)
    breaks = np.asarray(rotation.breaks, dtype=float)
    smooth = ~np.any((s_knots[:-1, None] < breaks) & (breaks < s_knots[1:, None]), axis=1)
    frobenius_tolerance = 2 * np.sqrt(2) * np.sin(ORIENTATION_TOLERANCE_RAD / 2)

    def errors(index, t):
        parts = []
        # Batches of 8192 FK rows: the per-interval Python loop was 0.6 s of
        # a 5 s plan on the relay.
        for at in range(0, len(index), 8192):
            i, x = index[at:at + 8192], t[at:at + 8192, None]
            q = ((a[i]*x + b[i])*x + c[i])*x + q0[i]
            rot, pos = fk.torch_link_transform(chain, link, torch.as_tensor(q[:, order], dtype=torch.float64, device=device))
            s = clock(u[i] + h[i] * x[:, 0])
            error_p = np.linalg.norm(pos.cpu().numpy() - position(s), axis=1)
            error_f = np.linalg.norm(rot.cpu().numpy() - rotation(s), axis=(1, 2))
            error_r = 2 * np.arcsin(np.minimum(1., error_f / (2 * np.sqrt(2))))
            if np.any(error_p > POSITION_TOLERANCE_M) or np.any(error_r > ORIENTATION_TOLERANCE_RAD):
                raise TrackingRefused(f"Taught Cartesian path exceeds the {tolerance_text()} tracking tolerance: "
                                      f"{error_p.max()*1000:.5f} mm, {error_r.max():.6f} rad")
            parts.append((error_p, error_f, error_r))
        return tuple(np.concatenate(column) for column in zip(*parts))

    # Every knot once: knot k is local 0 of interval k and the last is local 1.
    n = len(h)
    p, f, r = errors(np.r_[np.arange(n), n - 1], np.r_[np.zeros(n), 1.])
    # Pending sub-intervals as arrays: interval, local lo/hi, depth, and the
    # position/Frobenius/angle errors already measured at lo and hi.
    pending = (np.arange(n), np.zeros(n), np.ones(n), np.zeros(n, dtype=int), p[:-1], p[1:], f[:-1], f[1:], r[:-1], r[1:])
    checked = n + 1
    while len(pending[0]):
        index, lo, hi, depth, p_lo, p_hi, f_lo, f_hi, r_lo, r_hi = pending
        w = hi - lo
        p_bound = np.minimum((p_lo + p_hi + lipschitz_p[index]*w) / 2, np.maximum(p_lo, p_hi) + second_p[index]*w*w/8)
        r_proved = (((r_lo + r_hi + lipschitz_r[index]*w) / 2 <= ORIENTATION_TOLERANCE_RAD)
                    | (smooth[index] & (np.maximum(f_lo, f_hi) + second_f[index]*w*w/8 <= frobenius_tolerance)))
        open_ = ~((p_bound <= POSITION_TOLERANCE_M) & r_proved)
        if np.any(depth[open_] >= 20):
            raise TrackingRefused("Taught Cartesian path could not be continuously verified")
        index, lo, hi, depth, p_lo, p_hi, f_lo, f_hi, r_lo, r_hi = (part[open_] for part in pending)
        checked += len(index)
        if checked > 1000000:
            raise TrackingRefused("Cartesian verification exceeded its subdivision budget")
        middle = (lo + hi) / 2
        p_m, f_m, r_m = errors(index, middle) if len(index) else (middle, middle, middle)
        pending = tuple(np.concatenate(pair) for pair in (
            (index, index), (lo, middle), (middle, hi), (depth + 1, depth + 1),
            (p_lo, p_m), (p_m, p_hi), (f_lo, f_m), (f_m, f_hi), (r_lo, r_m), (r_m, r_hi)))
    return checked


def _constant_tcp_timing(knots, fractions, length, speed_m_s, names, limits, *, include_fractions=False):
    """One trapezoidal path clock: ramp, constant TCP cruise, ramp to rest.

    Joint derivatives estimate a feasible cruise/ramp. The existing exact
    Hermite rate judge and continuous path/collision verifiers remain final.
    A short move is triangular; curvature/joint limits can lower cruise speed.
    """
    derivative = np.gradient(knots, fractions, axis=0, edge_order=2)
    curvature = np.max(np.abs(np.gradient(derivative, fractions, axis=0, edge_order=2)), axis=0)
    slope = np.max(np.abs(derivative), axis=0)
    velocity = np.array([limits[n]["velocity_rad_s"] for n in names])
    acceleration = np.array([limits[n]["acceleration_rad_s2"] for n in names])
    moving = slope > 1e-10
    curved = curvature > 1e-10
    if not np.any(moving):
        raise ValueError("Constant TCP motion has no resolved joint displacement")
    v = min(speed_m_s / length, float(np.min(velocity[moving] / slope[moving])))
    if np.any(curved):
        v = min(v, float(np.min(np.sqrt(0.5 * acceleration[curved] / curvature[curved]))))
    a = float(np.min((acceleration - curvature * v*v)[moving] / slope[moving]))
    v = min(v, np.sqrt(a))
    ramp = v / a
    duration = 1 / v + ramp
    ramp_distance = 0.5 * v * ramp
    # Explicit ramp boundaries avoid interpolating across a change of phase.
    curve = CubicHermiteSpline(fractions, knots, derivative, axis=0)
    # A triangular ramp has one midpoint. Roundoff in v*v/a can put its
    # two boundaries on opposite sides of 0.5 by one ULP. Keeping both (or
    # a nearly identical original knot) creates equal timestamps and an
    # invalid Hermite interval, especially for nearly coincident waypoints.
    # Snap only machine-precision phase duplicates; retain the original
    # curve and both endpoints. The continuous verifiers still judge it.
    phase_eps = 8 * np.finfo(float).eps
    if abs(ramp_distance - 0.5) <= phase_eps:
        ramp_distance = 0.5
    boundaries = np.unique([0., ramp_distance, 1 - ramp_distance, 1.])
    fractions = np.array(fractions, copy=True)
    for boundary in boundaries:
        fractions[np.abs(fractions - boundary) <= phase_eps] = boundary
    fractions = np.unique(np.r_[fractions, boundaries])
    times = np.where(fractions < ramp_distance, np.sqrt(2 * fractions / a),
                     np.where(fractions > 1-ramp_distance, duration - np.sqrt(2 * (1-fractions) / a),
                              ramp + (fractions-ramp_distance) / v))
    speed = np.minimum(v, a * np.minimum(times, duration-times))
    rates = curve(fractions, 1) * speed[:, None]
    rates[[0, -1]] = 0.
    def progress(u):
        t = np.asarray(u) * duration
        return np.where(t < ramp, 0.5*a*t*t,
                        np.where(t > duration-ramp, 1-0.5*a*(duration-t)**2,
                                 ramp_distance + v*(t-ramp)))
    # |d2 progress / du2|: the ramps' constant acceleration, per unit phase.
    progress.curvature = a * duration * duration
    minimum_duration_scale = max(1., 0.1 / duration)
    result = (curve(fractions), times * minimum_duration_scale, rates / minimum_duration_scale, progress, v * duration)
    return (*result, fractions) if include_fractions else result


def plan_taught(request, node, start, cell, geometry, margin_m):
    from .freespace import retime_rest_to_rest_wire
    if geometry is None:
        raise ValueError("Taught motion requires the continuous collision verifier")
    names = cell.moved_axes
    target = joint_target(node["target"], names, cell.profile.held)
    capture = node.get("capture", {})
    robot = getattr(request, "program", {}).get("robot", {})
    if robot and any(capture.get(key) != robot.get(key) for key in ("model_id", "robot_description_sha256")):
        raise ValueError("Taught waypoint belongs to a different robot description; reteach it")
    calibration = capture.get("machine_planning_calibration_sha256")
    if calibration and calibration != request.cell.get("robot_cell", {}).get("machine_planning_calibration_sha256"):
        raise ValueError("Taught waypoint calibration changed; reteach it against the current machine")
    if node.get("motion") == "circular":
        if not node.get("via"):
            raise ValueError(f"MoveC {node['id']} needs a via pose; use Record via")
        joint_target(node["via"], names, cell.profile.held)
    limits = copy.deepcopy(cell.profile.limits)
    for name in names:
        limits[name]["velocity_rad_s"] *= node["speed_scale"]
        limits[name]["acceleration_rad_s2"] *= node.get("acceleration_scale", node["speed_scale"])
    chain = cell.arm.chain
    order = [names.index(name) for name in chain.axis_names]
    device = cell.arm.device
    if node["motion"] == "joint":
        link = chain.link_names.index(request.cell["frames"]["tool"])
        if node.get("target_space", "joint") == "cartesian":
            lower, upper = ik.limits_of(request.cell, chain, device, torch.float64)
            solved = ik.solve(chain, link,
                              torch.as_tensor(Rotation.from_euler("xyz", node["target"]["rpy_rad"]).as_matrix()[None], dtype=torch.float64, device=device),
                              torch.as_tensor([node["target"]["xyz_m"]], dtype=torch.float64, device=device),
                              torch.as_tensor(target[None, order], dtype=torch.float64, device=device),
                              lower=lower, upper=upper, pos_tol_m=1e-6, rot_tol_rad=1e-5)
            if not bool(solved.converged.all()):
                raise ValueError("MoveJ TCP destination is unreachable")
            target[order] = solved.q[0].cpu().numpy()
        knots = np.stack([start, target])
        # Smoothstep has peak slope 1.5. Downstream link lengths bound
        # each revolute joint's TCP lever arm over the whole joint move.
        begin = int(chain.start[link]); end = begin + int(chain.count[link])
        reach = 0.; travel_bound = 0.
        for edge in range(end - 1, begin - 1, -1):
            dof = int(chain.dof[edge])
            if dof >= 0:
                travel_bound += reach * abs(target[order[dof]] - start[order[dof]])
            reach += float(np.linalg.norm(chain.origin_pos[edge]))
        # Tighten with the actual Jacobian. The curvature bound covers the
        # half-interval between samples, so this remains a continuous limit.
        delta = target[order] - start[order]
        phases = np.linspace(0., 1., 65)
        q = torch.as_tensor(start[None, order] + phases[:, None] * delta, dtype=torch.float64, device=device)
        dp = jacobian(chain, link, q)[:, :3].cpu().numpy() @ delta
        curvature_bound = reach * float(np.abs(delta).sum())**2
        sampled_bound = float(np.linalg.norm(dp, axis=1).max()) + curvature_bound / 128
        travel_bound = min(travel_bound, sampled_bound) * (1 + 1e-7)
        duration = max(0.1, 1.5 * travel_bound / (node["speed_mm_s"] / 1000))
        times = np.array([0., duration])
        rates = np.zeros_like(knots)
    else:
        link = chain.link_names.index(request.cell["frames"]["tool"])
        rot, pos = fk.torch_link_transform(chain, link, torch.as_tensor(start[None, order], dtype=torch.float64, device=device))
        position, rotation, length, angular = cartesian_path(pos[0].cpu().numpy(), rot[0].cpu().numpy(), node["target"], node.get("via") if node["motion"] == "circular" else None)
        constant_tcp = node.get("constant_tcp_speed", True) and length > 1e-9
        lower, upper = ik.limits_of(request.cell, chain, device, torch.float64)
        counts = knot_counts(length, angular)
        for attempt, count in enumerate(counts):
            u = np.linspace(0., 1., count)
            s = u if constant_tcp else u*u*(3-2*u)
            seed = start[None, order] + s[:, None] * (target[order]-start[order])
            solved = ik.solve(chain, link, torch.as_tensor(rotation(s), dtype=torch.float64, device=device),
                              torch.as_tensor(position(s), dtype=torch.float64, device=device),
                              torch.as_tensor(seed, dtype=torch.float64, device=device), lower=lower, upper=upper,
                              pos_tol_m=1e-6, rot_tol_rad=1e-5)
            if not bool(solved.converged.all()):
                raise ValueError("Taught Cartesian path has unreachable poses")
            knots = np.tile(start, (count, 1)); knots[:, order] = solved.q.cpu().numpy()
            knots[0] = start
            try:
                if constant_tcp:
                    knots, times, rates, progress, slope_bound = _constant_tcp_timing(
                        knots, s, length, node["speed_mm_s"] / 1000, names, limits)
                    duration = times[-1]
                    _certify_pose_path(chain, link, order, knots, rates*duration, times/duration,
                                       position, rotation, length, angular, device, progress, slope_bound,
                                       progress.curvature)
                else:
                    tangents = np.gradient(knots, u, axis=0); tangents[[0, -1]] = 0
                    _certify_pose_path(chain, link, order, knots, tangents, u, position, rotation, length, angular, device)
                    duration = max(0.1, 1.5 * length / (node["speed_mm_s"] / 1000))
                    times = u * duration; rates = tangents / duration
                break
            except TrackingRefused:
                if attempt == len(counts) - 1:
                    raise
    times, rates, _ = retime_rest_to_rest_wire(names, knots, rates, times, limits)
    kin = kinematics(request.cell)
    full = np.zeros((len(knots), len(kin.axis_names))); velocities = np.zeros_like(full)
    for i, name in enumerate(kin.axis_names):
        if name in names:
            full[:, i] = knots[:, names.index(name)]; velocities[:, i] = rates[:, names.index(name)]
        elif name in cell.profile.held:
            full[:, i] = cell.profile.held[name]
        else:
            raise ValueError(f"No planned or held coordinate for {name}")
    verdict = verify_free_space(request.cell, geometry, full, motion="transit", times_s=times,
                                velocities=velocities, profile={"limits": limits}, margin_m=margin_m)
    return {"kind": "transit", "node_id": node["id"], "from": None, "to": None,
            "axis_names": names, "held": {n: float(np.rad2deg(v)) for n, v in cell.profile.held.items()},
            **hermite_wire_degrees(knots, rates, times), "duration_s": float(times[-1]),
            "verdict": verdict.summary(), "authored_motion": node["motion"]}, knots[-1]


def plan_taught_weld(request, nodes, start, cell, geometry, margin_m):
    """One path clock across authored weld destinations; no corner rounding.

    Every destination remains a segment boundary for Trace and the cursor. Only
    the complete span starts/stops. Non-tangent joins and a requested cruise
    the joint limits cannot support are refused, never silently blended/slowed.
    """
    if geometry is None:
        raise ValueError("Welding requires the continuous collision verifier")
    names, chain, device = cell.moved_axes, cell.arm.chain, cell.arm.device
    order = [names.index(name) for name in chain.axis_names]
    link = chain.link_names.index(request.cell["frames"]["tool"])
    rot, pos = fk.torch_link_transform(chain, link, torch.as_tensor(start[None, order], dtype=torch.float64, device=device))
    at_pos, at_rot = pos[0].cpu().numpy(), rot[0].cpu().numpy()
    paths, lengths, angulars, targets = [], [], [], []
    limits = copy.deepcopy(cell.profile.limits)
    for name in names:
        limits[name]["velocity_rad_s"] *= min(n["speed_scale"] for n in nodes)
        limits[name]["acceleration_rad_s2"] *= min(n.get("acceleration_scale", n["speed_scale"]) for n in nodes)
    for node in nodes:
        target = joint_target(node["target"], names, cell.profile.held)
        capture, robot = node.get("capture", {}), request.program.get("robot", {})
        if robot and any(capture.get(k) != robot.get(k) for k in ("model_id", "robot_description_sha256")):
            raise ValueError("Weld waypoint belongs to a different robot description; reteach it")
        calibration = capture.get("machine_planning_calibration_sha256")
        if calibration and calibration != request.cell.get("robot_cell", {}).get("machine_planning_calibration_sha256"):
            raise ValueError("Weld waypoint calibration changed; reteach it")
        if node["motion"] == "circular" and not node.get("via"):
            raise ValueError("Weld MoveC needs a via pose")
        position, rotation, length, angular = cartesian_path(at_pos, at_rot, node["target"], node.get("via") if node["motion"] == "circular" else None)
        if length <= 1e-9:
            raise ValueError("A weld leg needs nonzero TCP travel")
        paths.append((position, rotation)); lengths.append(length); angulars.append(angular); targets.append(target)
        at_pos, at_rot = position([1.])[0], rotation([1.])[0]
    total = sum(lengths)
    boundaries = np.r_[0., np.cumsum(lengths) / total]; boundaries[-1] = 1.
    # Numerical tangent comparison only; it never rounds a corner or changes a
    # taught pose. Sharp corners require a stop/start or an authored tangent arc.
    eps = 1e-6
    for i in range(1, len(paths)):
        pa, ra = paths[i-1]; pb, rb = paths[i]
        da = (pa([1.])[0]-pa([1.-eps])[0])/(eps*lengths[i-1])
        db = (pb([eps])[0]-pb([0.])[0])/(eps*lengths[i])
        wa = Rotation.from_matrix(ra([1.-eps])[0].T @ ra([1.])[0]).as_rotvec()/(eps*lengths[i-1])
        wb = Rotation.from_matrix(rb([0.])[0].T @ rb([eps])[0]).as_rotvec()/(eps*lengths[i])
        if np.linalg.norm(da-db) > 1e-5 or np.linalg.norm(wa-wb) > 1e-5:
            raise ValueError(f"Weld corner at {nodes[i-1]['id']} is not tangent: teach a tangent arc or end/start the weld; constant speed cannot pass a sharp corner")
    def piecewise(which):
        def value(s):
            s=np.atleast_1d(np.asarray(s,dtype=float))
            out=np.empty((len(s),3) if which==0 else (len(s),3,3))
            indices=np.clip(np.searchsorted(boundaries,s,side="right")-1,0,len(paths)-1)
            for i,path in enumerate(paths):
                selected=indices==i
                if selected.any(): out[selected]=path[which](np.clip((s[selected]-boundaries[i])/(boundaries[i+1]-boundaries[i]),0.,1.))
            return out
        return value
    position, rotation = piecewise(0), piecewise(1)
    position.curvature=max(p[0].curvature/(length/total)**2 for p,length in zip(paths,lengths))
    rotation.breaks=np.unique(np.concatenate([boundaries[1:-1]] + [boundaries[i]+np.asarray(p[1].breaks)*lengths[i]/total for i,p in enumerate(paths)]))
    angular=max(a*total/length for a,length in zip(angulars,lengths))
    lower,upper=ik.limits_of(request.cell,chain,device,torch.float64)
    requested=nodes[0]["speed_mm_s"]/1000
    counts=[knot_counts(length,a) for length,a in zip(lengths,angulars)]
    for attempt in range(max(len(c) for c in counts)):
        fractions=[]; seeds=[]; previous=start
        for i,target in enumerate(targets):
            local=np.linspace(0.,1.,counts[i][min(attempt,len(counts[i])-1)])
            fractions.extend((boundaries[i]+local*(boundaries[i+1]-boundaries[i]))[i>0:])
            seeds.extend((previous[None,order]+local[:,None]*(target[order]-previous[order]))[i>0:])
            previous=target
        fractions=np.asarray(fractions)
        solved=ik.solve(chain,link,torch.as_tensor(rotation(fractions),dtype=torch.float64,device=device),
                        torch.as_tensor(position(fractions),dtype=torch.float64,device=device),
                        torch.as_tensor(np.asarray(seeds),dtype=torch.float64,device=device),lower=lower,upper=upper,pos_tol_m=1e-6,rot_tol_rad=1e-5)
        if not bool(solved.converged.all()): raise ValueError("Weld path has unreachable poses")
        knots=np.tile(start,(len(fractions),1)); knots[:,order]=solved.q.cpu().numpy(); knots[0]=start
        knots,times,rates,progress,slope,phases=_constant_tcp_timing(knots,fractions,total,requested,names,limits,include_fractions=True)
        achieved=total*slope/times[-1]
        if achieved < requested*(1-1e-6):
            raise ValueError(f"Requested weld speed {requested*1000:g} mm/s cannot be reached on this span; feasible cruise is at most {achieved*1000:.3f} mm/s. Lower the authored speed or extend the path")
        try:
            _certify_pose_path(chain,link,order,knots,rates*times[-1],times/times[-1],position,rotation,total,angular,device,progress,slope,progress.curvature)
            break
        except TrackingRefused:
            if attempt == max(len(c) for c in counts)-1: raise
    kin=kinematics(request.cell)
    full=np.zeros((len(knots),len(kin.axis_names))); velocities=np.zeros_like(full)
    for i,name in enumerate(kin.axis_names):
        if name in names:
            full[:,i]=knots[:,names.index(name)]; velocities[:,i]=rates[:,names.index(name)]
        elif name in cell.profile.held: full[:,i]=cell.profile.held[name]
        else: raise ValueError(f"No planned or held coordinate for {name}")
    verdict=verify_free_space(request.cell,geometry,full,motion="transit",times_s=times,
                              velocities=velocities,profile={"limits":limits},margin_m=margin_m)
    result=[]; group="taught-weld:"+nodes[0]["id"]
    for i,node in enumerate(nodes):
        lo=int(np.argmin(np.abs(phases-boundaries[i]))); hi=int(np.argmin(np.abs(phases-boundaries[i+1])))
        if abs(phases[lo]-boundaries[i])>1e-12 or abs(phases[hi]-boundaries[i+1])>1e-12: raise ValueError("Weld waypoint disappeared from path clock")
        local=times[lo:hi+1]-times[lo]
        result.append({"kind":"transit","node_id":node["id"],"from":None,"to":None,"axis_names":names,
                       "held":{n:float(np.rad2deg(v)) for n,v in cell.profile.held.items()},
                       **hermite_wire_degrees(knots[lo:hi+1],rates[lo:hi+1],local),"duration_s":float(local[-1]),
                       "verdict":verdict.summary(),"authored_motion":node["motion"],"continuous_weld":group,
                       "requested_weld_speed_mm_s":requested*1000,"planned_cruise_mm_s":achieved*1000})
    return result,knots[-1]
