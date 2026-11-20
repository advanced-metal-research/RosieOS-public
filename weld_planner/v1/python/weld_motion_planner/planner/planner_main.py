#!/usr/bin/env python
"""**The planner.** A `.weldplan` in, a plan the cell can run out.

Three stages, and this file is the order they go in:

    dp_seam_search_main.DpSeamSearchModule            (M4)  how each weld is welded
    weld_trajopt_main.WeldTrajoptModule               (M5)  each one as a curve
    connecting_trajopt_main.ConnectingTrajoptModule   (M6)  how the arm gets between

Each is callable and tested on its own. What only makes sense once they are in a
line is here, and it is three things.

**One collision model, built once.** It is a property of the cell and the
placement, not of a stage. Every stage can still build its own, but a run where
each did cost 164 ms of re-baking (608 ms cold, 63 ms warm) and, worse, only M4
honoured a caller's `mesh_root` -- so the welds could be screened against one set
of meshes and the moves between them against another, silently.

**Any stage can be started from.** M4 is seconds, M5 is minutes and M6 is most of
the rest, so `plan_motion` takes the earlier stages' results as inputs and skips
what it was handed. A skipped stage reports no timing.

**A later stage's failure is not fatal.** A plan whose welds are good but whose
transits could not be planned is still worth having, so M6's error is recorded
beside the trajectories rather than thrown over them.

Run: `pixi run -e motion python -m weld_motion_planner.planner.planner_main <request.weldplan>`.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Any

from weld_motion_planner.io.plan_request import PlanRequest
from weld_motion_planner.io.plan_request import read as read_request
from weld_motion_planner.io.results import SeamSearchResult
from weld_motion_planner.planner.base_types import PlannerContext
from weld_motion_planner.planner.dp_seam_search.dp_seam_search_main import (
    DpSeamSearchModule,
)
from weld_motion_planner.planner.dp_seam_search.options import (
    SeamSearchOptions,
)
from weld_motion_planner.planner.trajectory_optimization.options import (
    ConnectingTrajoptOptions,
    ConnectingTrajoptResult,
    WeldTrajoptOptions,
    WeldTrajoptResult,
)
from weld_motion_planner.robot_cell.collision import model as collision_model
from weld_motion_planner.robot_cell.context import mesh_root_for
from weld_motion_planner.robot_cell.robot_cell import RobotCell
from weld_motion_planner.utils.timing import Stopwatch, cuda_sync, report
from weld_motion_planner.verifier import model as verifier_model


@dataclass
class MotionPlanningOptions:
    """One options object per stage, plus which stages to run at all.

    A stage does not decide whether it is called, so the two flags live here and
    the rest lives on the stage's own object.
    """

    seam_search: SeamSearchOptions = field(default_factory=SeamSearchOptions)
    weld_trajopt: WeldTrajoptOptions = field(default_factory=WeldTrajoptOptions)
    connecting_trajopt: ConnectingTrajoptOptions = field(
        default_factory=ConnectingTrajoptOptions)

    # M5 off by default: minutes where the sweep is seconds.
    run_weld_trajopt: bool = False
    # M6 needs trajectories, either run or handed in, and does nothing without.
    run_connecting_trajopt: bool = True

    def as_json(self) -> dict[str, Any]:
        """FLAT, though the fields nest: `k_best` is a top-level key on the wire
        and `ui/src/api.ts` reads it there. Grouping by stage is an argument about
        the Python call, not a change to the response.

        The margin goes out in MILLIMETRES, the unit the operator reads every
        other clearance in.
        """
        out: dict[str, Any] = {"run_weld_trajopt": self.run_weld_trajopt,
                               "run_connecting_trajopt": self.run_connecting_trajopt}
        for stage in (self.seam_search, self.weld_trajopt, self.connecting_trajopt):
            out.update(asdict(stage))
        out["verify_margin_mm"] = out.pop("verify_margin_m") * 1000.0
        return out


@dataclass
class MotionPlanningResult:
    """What one plan request came to: one result per stage, side by side.

    A stage's result is absent -- `None`, or an empty result -- when the stage was
    neither run nor handed in. So a plan is a superset, never a different shape:
    asking for more ADDS fields and changes none of the existing ones.
    """

    options: MotionPlanningOptions
    seam_search_result: SeamSearchResult | None = None
    weld_trajopt_result: WeldTrajoptResult = field(default_factory=WeldTrajoptResult)
    connecting_trajopt_result: ConnectingTrajoptResult | None = None
    # Why M6 failed, if it did. The weld trajectories survive it, which is why
    # this is beside them rather than raised over them.
    connecting_trajopt_error: str | None = None
    timings_ms: dict[str, float] = field(default_factory=dict)

    # ---- Unpacked, because a reader wants the contents and not the boxes.
    @property
    def weld_trajectories(self) -> list[Any]:
        return self.weld_trajopt_result.trajectories

    @property
    def connecting_trajectories(self) -> list[dict[str, Any]] | None:
        if self.connecting_trajopt_result is None:
            return None
        return self.connecting_trajopt_result.connecting_trajectories


@dataclass
class _Modules:
    """Which modules this call actually runs, as opposed to is handed."""

    seam_search: bool
    weld_trajopt: bool
    connecting_trajopt: bool


def _modules_to_run(options: MotionPlanningOptions,
                   seam_search_result: SeamSearchResult | None,
                   weld_trajectories: list[Any] | None) -> _Modules:
    """Asked for, and not already done."""
    return _Modules(
        seam_search=seam_search_result is None and weld_trajectories is None,
        weld_trajopt=options.run_weld_trajopt and weld_trajectories is None,
        # Not gated on M5: with trajectories handed in, M6 stands alone.
        connecting_trajopt=options.run_connecting_trajopt)


def _reconciled(options: MotionPlanningOptions,
                seam_search_result: SeamSearchResult | None) -> MotionPlanningOptions:
    """The M4 settings a handed-in result actually ran under -- the result is the
    fact, and echoing `k_best: 5` over a result holding two would lie."""
    if seam_search_result is None:
        return options
    return replace(options, seam_search=replace(
        options.seam_search, k_best=seam_search_result.k_best,
        screen_collisions=seam_search_result.screened))


def _setup_collision_models(request: PlanRequest, cell: RobotCell,
                            options: MotionPlanningOptions,
                            modules: _Modules, *, mesh_root: Path) -> tuple[Any, Any]:
    """`(planner model, verifier model)`, or `None` for one nothing will ask.

    Built once for the whole request. Baking the distance field is the cold cost,
    so a call that only runs M6 must not pay for a model M4 would have wanted.
    """
    wants_planner = ((modules.seam_search and options.seam_search.screen_collisions)
                     or (modules.weld_trajopt and options.weld_trajopt.avoid_collisions)
                     or modules.connecting_trajopt)
    from weld_motion_planner.planner.trajectory_optimization.engine.taught import DIRECT_OPS
    if request.program.get("nodes") and all(n["op"] in DIRECT_OPS for n in request.program["nodes"]):
        # Direct taught curves (and a Home planned as a waypoint to the reset
        # pose, taught.HOME_AS_WAYPOINT) use the exact verifier, with no
        # obstacle-search stage.
        wants_planner = False
    wants_verifier = (options.weld_trajopt.verify
                      and (modules.weld_trajopt or modules.connecting_trajopt))
    return (
        collision_model.for_planner(request, cell, mesh_root=mesh_root)
        if wants_planner else None,
        verifier_model.for_verifier(request, mesh_root=mesh_root) if wants_verifier else None,
    )


def plan_motion(request: PlanRequest, options: MotionPlanningOptions | None = None, *,
                seam_search_result: SeamSearchResult | None = None,
                weld_trajectories: list[Any] | None = None,
                mesh_root: Path | None = None, device: Any = None,
                log: bool = True,
                progress: Any = None) -> MotionPlanningResult:
    """Plan one request, running the stages `options` asks for and was not handed.

    Hand in what a stage would have produced and it is used instead of run:

        M5 alone   seam_search_result=earlier.seam_search_result
        M6 alone   weld_trajectories=earlier.weld_trajectories

    Either may be given without the other. M6 on handed-in trajectories does not
    need the search they came from; the plan that comes back has no candidates.

    A skipped stage reports no timing, which is why the breakdown is READ from
    the result rather than assumed (`ui/.../motion-panel.ts`).

    `mesh_root`, `device` and `log` are not choices about the plan -- where the
    meshes are, which card to run on, and whether to narrate.
    """
    # Late, and the only torch this file touches: `planner/` is read for its
    # order, and paying for CUDA at import time to read it is not a trade to make.
    import torch

    if options is None:
        options = MotionPlanningOptions()
    if device is None:
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    options = _reconciled(options, seam_search_result)
    modules = _modules_to_run(options, seam_search_result, weld_trajectories)
    watch = Stopwatch(cuda_sync(device))

    cell = RobotCell.of(request.cell, device=device)
    # Resolved once, from the cell being planned, so the screen and the verifier
    # cannot end up reading different folders -- the failure the module header
    # describes, one cell wider.
    if mesh_root is None:
        mesh_root = mesh_root_for(request.cell)
    with watch.stage("collision_model"):
        planner_collision_model, verifier_collision_model = _setup_collision_models(
            request, cell, options, modules, mesh_root=mesh_root)

    context = PlannerContext(cell=cell,
                             planner_collision_model=planner_collision_model,
                             verifier_collision_model=verifier_collision_model,
                             device=device, mesh_root=mesh_root, log=log,
                             progress=progress)

    # ---- M4: a lattice per seam holding every way to weld each sample, swept by
    # dynamic programming for the K best ways across.
    if modules.seam_search:
        if progress is not None:
            progress("sweep")
        module = DpSeamSearchModule()
        with watch.stage(module.name):
            seam_search_result = module.plan(
                request, None, options.seam_search, context)

    # ---- M5: each weld smoothed into a continuous curve. The search result is
    # REUSED rather than re-run: running the sweep twice could hand a reader
    # candidates and trajectories from different runs of it.
    weld_trajopt_result = WeldTrajoptResult(
        trajectories=[] if weld_trajectories is None else weld_trajectories)
    if modules.weld_trajopt:
        from weld_motion_planner.planner.trajectory_optimization.weld_trajopt_main import (
            WeldTrajoptModule,
        )

        if progress is not None:
            progress("weldseam")
        module = WeldTrajoptModule()
        with watch.stage(module.name):
            weld_trajopt_result = module.plan(
                request, seam_search_result, options.weld_trajopt, context)

    # ---- M6: how the arm gets between them.
    connecting_trajopt_result: ConnectingTrajoptResult | None = None
    connecting_error: str | None = None
    # An in-order program (taught moves, or a measured start) has legs to
    # plan even with no weld: a Home-only program from the robot's pose is
    # one transit to the reset pose.
    from weldplan.measured_start import in_program_order
    if modules.connecting_trajopt and (weld_trajopt_result.trajectories
                                       or in_program_order(request.program, getattr(request, "start_state", None))):
        from weld_motion_planner.planner.trajectory_optimization.connecting_trajopt_main import (
            ConnectingTrajoptModule,
        )

        module = ConnectingTrajoptModule()
        with watch.stage(module.name):
            try:
                connecting_trajopt_result = module.plan(
                    request, weld_trajopt_result.trajectories,
                    options.connecting_trajopt, context)
                # M6 hands back REWOUND welds, and they replace the originals:
                # serialising the pre-rewind ones would put a 360-degree jump at
                # every handover.
                weld_trajopt_result = replace(
                    weld_trajopt_result,
                    trajectories=connecting_trajopt_result.weld_trajectories)
            except Exception as exc:  # noqa: BLE001 - recorded, not fatal
                connecting_error = f"{type(exc).__name__}: {exc}"

    out = MotionPlanningResult(
        options=options,
        seam_search_result=seam_search_result,
        weld_trajopt_result=weld_trajopt_result,
        connecting_trajopt_result=connecting_trajopt_result,
        connecting_trajopt_error=connecting_error,
        timings_ms=watch.ms())
    if log:
        _log(out)
    return out


def _log(out: MotionPlanningResult) -> None:
    """The plan's breakdown, on stderr, after M4 printed its own.

    Two blocks, and the difference between them is the point: `dp_seam_search`
    here is the whole of M4 as the plan paid for it, and the block above accounts
    for the same span from inside. What the two do not share is setup neither
    claimed.
    """
    for line in report(out.timings_ms):
        print(line, file=sys.stderr, flush=True)


# ---- CLI ---------------------------------------------------------------------


def _parse(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("request", type=Path)
    parser.add_argument("--k", type=int, default=SeamSearchOptions.k_best)
    parser.add_argument("--samples-per-seam", type=int, default=None)
    parser.add_argument("--no-collide", action="store_true")
    parser.add_argument("--trajectories", action="store_true",
                        help="also run M5 -- minutes where the sweep is seconds")
    parser.add_argument("--no-moves", action="store_true",
                        help="with --trajectories, skip the connecting moves")
    parser.add_argument("--no-verify", action="store_true")
    parser.add_argument("--output", type=Path, default=None,
                        help="write the reusable native result JSON here")
    parser.add_argument(
        "--source-revision",
        default="",
        help="full Git commit producing the result; otherwise read "
             "AMR_WELD_PLANNER_SOURCE_REVISION",
    )
    parser.add_argument(
        "--source-state",
        choices=("clean", "dirty"),
        default=None,
        help="required by --preflight; whether the producing checkout was clean",
    )
    parser.add_argument(
        "--source-diff-sha256",
        default=None,
        help="required by --preflight when --source-state=dirty",
    )
    parser.add_argument(
        "--preflight",
        action="store_true",
        help="hand the native result to Weld Preflight's ingest-candidate command",
    )
    parser.add_argument(
        "--weld-preflight",
        type=Path,
        default=Path("weld-preflight"),
        help="Weld Preflight executable (default: weld-preflight on PATH)",
    )
    parser.add_argument(
        "--preflight-store",
        type=Path,
        default=Path("runs/weld-preflight/candidates"),
        help="candidate receipt store passed to Weld Preflight",
    )
    parser.add_argument(
        "--weld-preflight-lock",
        type=Path,
        default=None,
        help="exact Weld Preflight package lock; defaults to the RosieOS v1 lock",
    )
    return parser.parse_args(argv)


def _options(args: argparse.Namespace) -> MotionPlanningOptions:
    screen = not args.no_collide
    return MotionPlanningOptions(
        seam_search=SeamSearchOptions(k_best=args.k,
                                      samples_per_seam=args.samples_per_seam,
                                      screen_collisions=screen),
        weld_trajopt=WeldTrajoptOptions(avoid_collisions=screen,
                                        verify=not args.no_verify),
        connecting_trajopt=ConnectingTrajoptOptions(avoid_collisions=screen,
                                                    verify=not args.no_verify),
        run_weld_trajopt=args.trajectories,
        run_connecting_trajopt=not args.no_moves)


def main(argv: list[str]) -> int:
    args = _parse(argv)
    if args.preflight and args.output is None:
        print(
            "preflight: capability_unavailable: --preflight requires --output",
            file=sys.stderr,
        )
        return 2

    request_bytes = args.request.read_bytes()
    request = read_request(args.request)
    result = plan_motion(request, _options(args))
    document = None
    source_revision = None
    if args.output is not None:
        from weld_motion_planner.io.native_result import (
            result_document,
            source_revision_from_environment,
        )

        source_revision = (
            args.source_revision or source_revision_from_environment()
        )
        document = result_document(
            request,
            result,
            request_sha256=hashlib.sha256(request_bytes).hexdigest(),
            source_revision=source_revision,
        )
        args.output.write_text(
            json.dumps(document, allow_nan=False, indent=2, sort_keys=True)
            + "\n",
            encoding="utf-8",
        )

    from weld_motion_planner.io.summary import print_answer

    print_answer(result)
    if args.preflight:
        from weld_motion_planner.io.preflight import (
            DEFAULT_PACKAGE_LOCK,
            PreflightUnavailable,
            ingest,
            write_candidate_manifest,
        )

        assert args.output is not None and document is not None
        manifest_path = args.output.with_name(
            f"{args.output.stem}.preflight.json"
        )
        try:
            write_candidate_manifest(
                manifest_path,
                args.output,
                document,
                source_revision=source_revision,
                source_state=args.source_state,
                source_diff_sha256=args.source_diff_sha256,
            )
            return ingest(
                args.weld_preflight,
                manifest_path,
                args.preflight_store,
                args.weld_preflight_lock or DEFAULT_PACKAGE_LOCK,
            )
        except PreflightUnavailable as error:
            print(
                f"preflight: capability_unavailable: {error}",
                file=sys.stderr,
            )
            return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
