#!/usr/bin/env python
"""Free-space planning by cuRobo, as an alternative backend.

`freespace.FREE_SPACE_BACKEND` selects it. The point is comparison: cuRobo
solves the same connecting motions, on the same robot, against the same
obstacles, and returns them in the same shape, so a plan made by one can be put
beside a plan made by the other without anything else changing.

## The robot is not converted, it is REUSED

cuRobo reads the planning context's `robot.urdf` -- the file the OLP compiled
the cell onto, beside the meshes -- and the cell descriptor every other stage
walks was parsed from that same file, so the two kinematic models agree to
the floating point: fitting cuRobo's sphere positions against this planner's,
at several poses, comes back with the identity rotation and a zero
translation, residual 0.000 mm.

That is worth stating because the wrong URDF is easy to pick and expensive to
debug. This backend used to find one by model name on its own disk and patch
the cell's numbers into a copy, and `urdf/v4/rosie_1400_h/robot.urdf` (since
removed with the bring-up trees) had the same FILE name, a different robot (no
`link_6`, no `floor`, `tool0` where `link_6` belongs) and fit with a 463 mm
residual -- which reads as a planner
disagreement rather than as the model mismatch it is. With the context there
is nothing to find (`docs/PLANNING-CONTEXT.md`).

## The obstacles are BAKED FROM THIS PLANNER'S OWN MESHES

Not re-derived from the placement. The part is authored in the workpiece frame
-- that is the frame `collision/scene.build` bakes it in, untransformed -- and
`scene.workpiece_frame(q)` is the whole map from there into world, positioner
state included. Re-deriving that chain from `placement_matrix` put the part
303 mm off the weld it touches, and inverting it 604 mm.

The bake is checked, not trusted: the same poses are judged by this planner's
own collision model and by a mesh query on the baked geometry, and both must
report the same penetrations.
"""

from __future__ import annotations

import tempfile
from pathlib import Path
from typing import Any

import numpy as np
import torch

#: cuRobo attempts per motion, and the attempt at which its graph planner
#: starts seeding. Its own defaults for `plan_cspace`.
MAX_ATTEMPTS = 5
GRAPH_ATTEMPT = 1

#: Samples the returned trajectory is interpolated to, in seconds. Matches
#: `control_solver.SAMPLES` over a typical horizon closely enough that the two
#: backends' outputs are the same kind of object.
INTERPOLATION_DT_S = 0.02
INTERPOLATION_BUFFER = 4096


class CuroboUnavailable(RuntimeError):
    """cuRobo is not installed, or not installed for this device."""


#: The `wp.config` fields `curobo._src.util.warp.setup_warp` writes. Warp is
#: shared -- this planner's ESDF and FK kernels compile through the same
#: module -- so a mode or verbosity cuRobo picks must not outlive it.
_WARP_CONFIG_KEYS = ("quiet", "verbose", "lineinfo", "line_directives",
                     "print_launches", "mode", "verify_cuda")


class _torch_global_state:
    """Snapshot-and-restore of every process global cuRobo is known to touch.

    Three channels, each found the hard way:

    - **TF32** (`seed_ik_solver.py:39`, at import, process-wide).
      `torch.backends.cuda.matmul.allow_tf32 = True` truncates float32
      matmuls to a 10-bit mantissa; this planner's seam-search IK stops
      converging at that precision. Measured: ONE cuRobo request flipped a
      seam that crossed cleanly to `no ik` on 17,086 of 17,995 lattice nodes
      for every later request in the process's life. Two servers with
      byte-identical code disagreed on the same request for a day because
      one of them had once planned through cuRobo.
    - **`cudnn.benchmark = True`** (same module, line 37). Caught leaking
      through the first version of this guard by the before/after diff.
    - **Global RNG** (`path_finder_networkx.py: reset_seed`, at every graph
      plan). Reseeds numpy AND python `random` process-wide -- the source
      itself says "this is not a good idea". Nothing in this planner draws
      from either today (the IK uses an explicit `torch.Generator`), and
      restoring them keeps that a fact about the code rather than a fact
      about luck.

    `wp.config` is snapshotted for the same reason as cudnn: warp is shared
    with this planner's own kernels and `setup_warp` writes it.
    """

    def __enter__(self):
        import random as _random

        self.matmul_tf32 = torch.backends.cuda.matmul.allow_tf32
        self.cudnn_tf32 = torch.backends.cudnn.allow_tf32
        self.cudnn_benchmark = torch.backends.cudnn.benchmark
        self.np_state = np.random.get_state()
        self.py_state = _random.getstate()
        try:
            import warp as _wp
            self.wp_config = {key: getattr(_wp.config, key)
                              for key in _WARP_CONFIG_KEYS
                              if hasattr(_wp.config, key)}
        except ImportError:
            self.wp_config = None
        return self

    def __exit__(self, *_exc):
        import random as _random

        torch.backends.cuda.matmul.allow_tf32 = self.matmul_tf32
        torch.backends.cudnn.allow_tf32 = self.cudnn_tf32
        torch.backends.cudnn.benchmark = self.cudnn_benchmark
        np.random.set_state(self.np_state)
        _random.setstate(self.py_state)
        if self.wp_config is not None:
            import warp as _wp
            for key, value in self.wp_config.items():
                setattr(_wp.config, key, value)
        return False

def _require_curobo():
    try:
        with _torch_global_state():
            from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
            from curobo.scene import Mesh, Scene
            from curobo.types import JointState
    except ImportError as error:  # noqa: PERF203
        raise CuroboUnavailable(
            "cuRobo is not importable; install it into this environment or "
            "select another free-space backend") from error
    return MotionPlanner, MotionPlannerCfg, Scene, Mesh, JointState


def robot_config(request: Any, cell: Any, urdf_path: Path) -> dict:
    """cuRobo's kinematics block for this cell.

    Spheres, limits and joint names all come from the cell, so nothing here is
    a second description of the robot that could drift from the first.

    **Position bounds come from the URDF, which is the arm that exists.** The
    corpus bundles carry wider ones from before the arm was measured
    (`docs/issues/0014`), and cuRobo refusing motions that need `J4 = -3.562`
    is cuRobo being right.
    """
    limits = getattr(cell.profile, "limits", None) or {}
    arm = [axis for axis in cell.moved_axes if axis in limits][:6]
    spheres = cell.arm.spheres.links
    return {
        "urdf_path": str(urdf_path),
        "asset_root_path": str(urdf_path.parent),
        "base_link": "floor",
        "tool_frames": ["tool0"],
        "collision_link_names": list(spheres),
        "collision_spheres": {
            link: [{"center": [float(v) for v in centre], "radius": float(radius)}
                   for centre, radius in entries]
            for link, entries in spheres.items()
        },
        "collision_sphere_buffer": 0.0,
        # Adjacent links overlap by construction -- a sphere cover that stopped
        # short of the mating face would leave the real link uncovered -- which
        # is `collision/pairs.py`'s structural rule, restated in cuRobo's shape.
        "self_collision_ignore": {
            link: [nxt for nxt in list(spheres)[i + 1:i + 2]]
            for i, link in enumerate(spheres)
        },
        "self_collision_buffer": {link: 0.0 for link in spheres},
        "mesh_link_names": list(spheres),
        "cspace": {
            "joint_names": list(arm),
            "max_acceleration": [float(limits[a]["acceleration_rad_s2"]) for a in arm],
            "max_jerk": 500.0,
            "default_joint_position": [0.0] * len(arm),
            "cspace_distance_weight": [1.0] * len(arm),
            "null_space_weight": [1.0] * len(arm),
        },
    }


def world_meshes(request: Any, scene: Any, pose_check: Any, cell: Any,
                 at: np.ndarray, out_dir: Path, device: Any) -> list[Any]:
    """The obstacles, in WORLD, at the positioner state `at`.

    Written to disk because cuRobo's `Mesh` loads from a path. The directory is
    the caller's to clean up.
    """
    import trimesh

    from weld_motion_planner.geometry import mesh as meshmod
    from weld_motion_planner.robot_cell.collision import scene as scenemod
    from weld_motion_planner.robot_cell.robot.profile import mesh_root as mesh_root_of

    _, _, _, Mesh, _ = _require_curobo()
    rotation, translation = scene.workpiece_frame(
        pose_check.q_full(torch.as_tensor(at[None], dtype=torch.float64,
                                          device=device)))
    rotation = rotation[0].detach().cpu().numpy()
    translation = translation[0].detach().cpu().numpy()

    def write(name: str, vertices: np.ndarray, faces: np.ndarray) -> Any:
        """Repair the winding before writing.

        The tessellator's output is watertight but NOT winding-consistent
        (`is_winding_consistent` is False on this corpus), and an inconsistent
        winding makes inside/outside meaningless: `trimesh.signed_distance`
        reported a point 77 mm CLEAR of the part as 77 mm inside it, which is
        how a correct world bake came to look wrong. cuRobo builds its own SDF
        from these faces and needs the same thing to be true.
        """
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
        mesh.fix_normals()
        path = out_dir / f"{name}.obj"
        mesh.export(str(path))
        return Mesh(name=name, file_path=str(path), pose=[0, 0, 0, 1, 0, 0, 0])

    out: list[Any] = []
    if request.step_bytes is not None:
        part = meshmod.tessellate(request.step_bytes)
        vertices = np.asarray(part.vertices) @ rotation.T + translation
        out.append(write("part", vertices, np.asarray(part.faces)))

    # The cell's own geometry -- the table the arm stands over, the positioner
    # it reaches around. Without it cuRobo avoids the part by moving into the
    # frame, which is not an improvement.
    from weld_motion_planner.robot_cell.collision.fixtures import fixture_meshes

    env, fixtures, _ = scenemod.cell_obstacles(
        cell.descriptor, mesh_root_of(cell.descriptor.get("id") or ""),
        held=dict(cell.profile.held),
        extra_static=fixture_meshes(request.fixtures))
    if env is not None:
        out.append(write("env", np.asarray(env.vertices), np.asarray(env.faces)))
    if fixtures is not None:
        # `scene.build` moves the fixture mesh into the workpiece frame with
        # the INVERSE placement before baking it (`collision/scene.py`), because
        # it arrives in the WORK frame while the part arrives already in the
        # piece frame. Skipping that step put this cell's fixtures a metre above
        # everything else -- a 36-vertex bar standing in mid air -- and cuRobo
        # spent 16 s flying around it.
        inverse = np.linalg.inv(scenemod.placement_matrix(request.placement or {}))
        local = (np.asarray(fixtures.vertices) @ inverse[:3, :3].T + inverse[:3, 3])
        vertices = local @ rotation.T + translation
        out.append(write("fixtures", vertices, np.asarray(fixtures.faces)))
    return out


def solve_batch(request: Any, planner_collision_model: Any, device: Any, *,
                specs: list[Any], held: dict[str, float], cell: Any = None,
                **_ignored: Any):
    """Every motion in `specs`, planned by cuRobo.

    Returns what `freespace._finish_moves` reads, so the reporting, the
    verifier and the `Move` assembly are the ones already there.
    """
    with _torch_global_state():
        return _solve_batch(request, planner_collision_model, device,
                            specs=specs, held=held, cell=cell)


def _solve_batch(request: Any, planner_collision_model: Any, device: Any, *,
                 specs: list[Any], held: dict[str, float], cell: Any = None):
    MotionPlanner, MotionPlannerCfg, Scene, _Mesh, JointState = _require_curobo()

    from weld_motion_planner.planner.trajectory_optimization.engine.control_solver import (
        _AsSolved,
        Solved,
    )
    from weld_motion_planner.robot_cell.robot_cell import RobotCell

    cell = (cell or RobotCell.of(request.cell, device=device)).to(device)
    # The URDF cuRobo reads: the planning context's, which the OLP already
    # composed the cell onto, beside the meshes (docs/PLANNING-CONTEXT.md).
    # There is no other source: the copy this used to find by model name on
    # its own disk and patch is the second derivation the context ended.
    urdf = cell.context.urdf_path()
    axes = list(cell.moved_axes)
    arm = axes[:6]
    pose_check = (None if planner_collision_model is None else
                  planner_collision_model.checker(axes, held))

    limits = getattr(cell.profile, "limits", None) or {}
    vmax = np.array([float(limits[a]["velocity_rad_s"]) for a in axes])
    amax = np.array([float(limits[a]["acceleration_rad_s2"]) for a in axes])

    with tempfile.TemporaryDirectory(prefix="curobo-world-") as tmp:
        meshes = []
        if planner_collision_model is not None and pose_check is not None:
            meshes = world_meshes(request, planner_collision_model, pose_check,
                                  cell, np.asarray(specs[0].start, float),
                                  Path(tmp), device)
        planner = MotionPlanner(MotionPlannerCfg.create(
            robot={"kinematics": robot_config(request, cell, urdf)},
            scene_model=Scene(mesh=meshes) if meshes else None,
            interpolation_dt=INTERPOLATION_DT_S,
            interpolation_buffer_size=INTERPOLATION_BUFFER))
        names = planner.joint_names

        solved, notes = [], []
        for spec in specs:
            start = np.asarray(spec.start, dtype=np.float64)
            end = np.asarray(spec.end, dtype=np.float64)
            result = planner.plan_cspace(
                goal_state=JointState.from_position(
                    torch.tensor(end[:6], dtype=torch.float32,
                                 device="cuda").view(1, -1), joint_names=names),
                current_state=JointState.from_position(
                    torch.tensor(start[:6], dtype=torch.float32,
                                 device="cuda").view(1, -1), joint_names=names),
                max_attempts=MAX_ATTEMPTS, enable_graph_attempt=GRAPH_ATTEMPT)
            # `plan_cspace` does NOT return None on failure -- it returns a
            # result whose `success` is False and whose trajectory is a
            # placeholder at the interpolation buffer's full length (16.00 s at
            # `interpolation_dt` 0.02, every time). Reading that as an answer
            # put three failed plans into a comparison table as if they were
            # motions.
            if result is None:
                raise RuntimeError(
                    f"cuRobo returned no plan for the {spec.kind} motion")
            # `success` is cuRobo's OWN verdict, against its own sphere
            # padding and activation distances, and on this cell it is stricter
            # than the model that will actually judge the motion: measured, a
            # run flagged False returned 161 samples whose worst clearance by
            # THIS planner's collision model was +10.91 mm, penetrating nowhere,
            # with the endpoints hit to 2e-08.
            #
            # So the flag is recorded rather than obeyed. What disqualifies an
            # answer is a placeholder trajectory -- on failure cuRobo returns
            # one at the interpolation buffer's full length, 16.00 s every time
            # -- and that is what is tested.
            success = getattr(result, "success", None)
            claimed = (None if success is None
                       else bool(torch.as_tensor(success).any()))
            knots, duration = _trajectory(result)
            if len(knots) >= INTERPOLATION_BUFFER - 1:
                raise RuntimeError(
                    f"cuRobo returned a placeholder for the {spec.kind} motion "
                    f"({spec.from_id or 'home'} -> {spec.to_id or 'home'}): "
                    f"{len(knots)} samples fills the interpolation buffer")
            import os as _os
            if _os.environ.get("WELD_PLANNER_CUROBO_DEBUG"):
                print(f"[curobo] {spec.kind} start={np.round(start[:6],3)} "
                      f"end={np.round(end[:6],3)}", flush=True)
                print(f"[curobo] {spec.kind}: success={getattr(result,'success',None)} "
                      f"motion_time={duration:.3f} last_tstep="
                      f"{getattr(result,'interpolated_last_tstep',None)} "
                      f"raw={result.interpolated_trajectory.position.shape} "
                      f"kept={len(knots)}", flush=True)
            # The held axes ride along unchanged: cuRobo plans the arm, and
            # this cell's positioner is held for the duration of a connecting
            # motion either way.
            full = np.repeat(start[None, :], len(knots), axis=0)
            full[:, :6] = knots
            solved.append(_as_solved(full, duration, planner_collision_model,
                                     pose_check, vmax, amax, device))
            notes.append([f"curobo: {len(knots)} samples over {duration:.2f} s"
                          + ("" if claimed is not False
                             else " (cuRobo flagged success=False; judged on "
                                  "this planner's own collision model instead)")])
        return [_AsSolved(s) for s in solved], notes, solved


def _trajectory(result: Any) -> tuple[np.ndarray, float]:
    position = result.interpolated_trajectory.position
    while position.dim() > 2:
        position = position[0]
    knots = position.detach().cpu().numpy()
    last = getattr(result, "interpolated_last_tstep", None)
    if last is not None:
        count = int(torch.as_tensor(last).flatten()[0])
        if 2 <= count <= len(knots):
            knots = knots[:count]
    return knots, float(torch.as_tensor(result.motion_time()).flatten()[0])


def _as_solved(knots: np.ndarray, duration: float, model: Any, pose_check: Any,
               vmax: np.ndarray, amax: np.ndarray, device: Any):
    from weld_motion_planner.planner.trajectory_optimization.engine.control_solver import (
        Solved,
    )

    dt = duration / max(len(knots) - 1, 1)
    velocity = np.gradient(knots, dt, axis=0)
    acceleration = np.gradient(velocity, dt, axis=0)
    jerk = np.gradient(acceleration, dt, axis=0)
    env = own = float("inf")
    if model is not None and pose_check is not None:
        with torch.no_grad():
            q = torch.as_tensor(knots, dtype=torch.float64, device=device)
            gaps, mine = model.clearances(pose_check.q_full(q))
            env = float(gaps.min())
            own = float(mine.min()) if mine.numel() else float("inf")
    return Solved(
        control=knots, knots=knots, velocity=velocity,
        acceleration=acceleration, jerk=jerk,
        times_s=np.linspace(0.0, duration, len(knots)), duration_s=duration,
        clearance_m=min(env, own), env_clearance_m=env, self_clearance_m=own,
        cost_before=0.0, cost_after=0.0,
        v_ratio=float(np.abs(velocity / vmax).max()),
        a_ratio=float(np.abs(acceleration / amax).max()),
        j_ratio=0.0)
