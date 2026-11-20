#!/usr/bin/env python
"""The motion planner as a server: a `.weldplan` in, candidate paths out.

`IMPLEMENTATION.md` M10 wants this thin: one POST that calls
`planner.planner_main.plan_motion` and serialises what comes back. **No pipeline
lives here.** It used to -- a copy of it, which is how this file spent a day
screening against the part alone while reporting "screened", so the thinness is a
property to keep rather than an aesthetic. It grew back once as the ORDER of the
three stages plus a `plan_connecting_trajectories` living under `server/`, and that is
`planner/` now.

What that leaves is the HTTP surface and nothing else: the endpoint, every query
parameter and what it costs, handing the card's memory back, and the uvicorn
launcher. What the planner does is `planner/planner_main.py`; the bytes boundary
in both directions -- opening a plan request, and making the answer legal JSON --
is `weld_motion_planner.io.wire`; why this package has a server of its own at all
is the folder's `__init__`.

## What a candidate is

One of the sweep's K best ways across a seam: the unwound joint trajectory --
arm axes then the positioner, the values a controller would execute -- plus
its cost in degrees of accumulated joint motion. Five by default, because the
runner-up is routinely the best path's wrist-flipped twin (`docs/issues/0002`)
and a picker needs more than two to hold a genuinely different option.

Each admitted request runs in a fresh Python process. Disconnecting cancels
that process (TERM, then KILL after two seconds if necessary), so even a native
CUDA solve stops before another request acquires the GPU. The service
(`main`) keeps one such process started ahead of its request -- the spare,
which has done its imports and waits on stdin for a job directory -- and
starts the next spare when a plan's process is gone, so a request no longer
pays the interpreter and imports (2-3 s warm, up to a minute from a cold page
cache; measured on a Rosie 1430 arm's `test1`, 2026-10-11). The spare is still one
process per plan: nothing a plan leaves behind reaches the next one. Progress
remains available while it runs; dense artifacts are published only after
successful, connected completion.

Run: `pixi run -e motion motion-serve`.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import hashlib
import json
import os
import sys
import signal
import tempfile
import time
from pathlib import Path
from typing import Any

from weld_motion_planner.planner.dp_seam_search.options import SeamSearchOptions
from weld_motion_planner.planner.planner_main import MotionPlanningOptions
from weld_motion_planner.planner.trajectory_optimization.options import (
    ConnectingTrajoptOptions,
    WeldTrajoptOptions,
)

DEFAULT_PORT = 8796


def release_gpu() -> None:
    """Hand the card's memory back between requests.

    torch keeps every block it has ever allocated in a caching allocator, which
    is the right trade for a script and the wrong one for a process that lives
    for days: this server sat on 21.4 GiB of a 32.6 GiB card long after the
    request that needed it, and the next thing to want the GPU -- the test
    suite, another tool, a second planner -- failed to allocate 48 MB.

    Costs the next request its allocator warm-up, which is milliseconds against
    a plan measured in minutes. Idle memory that nothing can use is not a cache.
    """
    import torch

    if torch.cuda.is_available():
        torch.cuda.empty_cache()


DEFAULT_DENSE_STORE_DIR = "~/.cache/rosieos-olp/weld-planner-dense"

#: Refused plan requests and results kept under `<dense store>/refused`, oldest out.
#: Five is a session's worth of retries at a few MiB each; unmeasured beyond that.
REFUSED_KEPT = 5


def _execute_plan_job(data: bytes, parameters: dict[str, Any], dense_dir: Path,
                      progress) -> dict[str, Any]:
    """Run the existing pipeline inside one disposable, CUDA-owning process."""
    from weld_motion_planner.io.dense_output import emit_dense
    k_best = parameters["k_best"]
    screen_collisions = parameters["screen_collisions"]
    samples_per_seam = parameters["samples_per_seam"]
    run_weld_trajopt = parameters["run_weld_trajopt"]
    run_connecting_trajopt = parameters["run_connecting_trajopt"]
    free_space_backend = parameters["free_space_backend"]
    order_seams = parameters["order_seams"]
    verify = parameters["verify"]
    verify_margin_mm = parameters["verify_margin_mm"]
    dense = parameters["dense"]
    program_id = parameters["program_id"]
    manifest_revision = parameters["manifest_revision"]
    plan_revision = parameters["plan_revision"]

    from weld_motion_planner.io.native_result import (
        result_document,
        source_revision_from_environment,
    )
    from weld_motion_planner.io.wire import request_from_bytes
    from weld_motion_planner.planner.planner_main import plan_motion

    opened = request_from_bytes(data)
    motion_plan = plan_motion(opened, MotionPlanningOptions(
        seam_search=SeamSearchOptions(
            k_best=k_best, samples_per_seam=samples_per_seam,
            screen_collisions=screen_collisions),
        # ONE query param drives both: over HTTP "screen or do
        # not" is a single decision, and the response echoes the
        # two flags so a reader can see they agreed.
        weld_trajopt=WeldTrajoptOptions(
            avoid_collisions=screen_collisions, verify=verify,
            verify_margin_m=verify_margin_mm / 1000.0),
        connecting_trajopt=ConnectingTrajoptOptions(
            avoid_collisions=screen_collisions, verify=verify,
            verify_margin_m=verify_margin_mm / 1000.0,
            backend=free_space_backend,
            order_seams=order_seams),
        run_weld_trajopt=run_weld_trajopt,
        run_connecting_trajopt=run_connecting_trajopt),
        progress=progress)
    request_sha256 = hashlib.sha256(data).hexdigest()
    out = result_document(
        opened,
        motion_plan,
        request_sha256=request_sha256,
        source_revision=source_revision_from_environment(),
    )
    release_gpu()
    if dense:
        if not (run_weld_trajopt and run_connecting_trajopt):
            out["dense_error"] = {
                "reason": "result_empty",
                "detail": "a dense trajectory needs run_weld_trajopt "
                          "and run_connecting_trajopt",
                "segment_index": None}
        else:
            # The header's ceiling is the cell's own limit: the
            # profile this plan was made against, so a plan the
            # cell cannot play is refused here, with the segment
            # named, not at the daemon's gate.
            from weld_motion_planner.io.dense_output import (
                cell_axis_mask, cell_speed_limits)
            from weld_motion_planner.robot_cell.robot.profile import for_cell as profile_for_cell
            from weld_motion_planner.robot_cell.robot.profile import moved_axes
            from weldplan.robot_description import header_identity
            profile = profile_for_cell(opened.cell)
            # And the header's axis mask is the cell's too: the
            # axes it drives plus the ones it holds. The params
            # default is all nine, which the 1420's pair (0x3f)
            # refuses outright.
            summary, error = emit_dense(
                out, store_dir=dense_dir,
                request_sha256=request_sha256,
                program_id=program_id,
                manifest_revision=manifest_revision,
                plan_revision=plan_revision,
                # The request, not a Load-time guess about the
                # target, decides whether this is motion-only.
                execution_mode=opened.program.get("execution_mode", "production"),
                start_state=opened.start_state,
                io_outputs=getattr(opened, "io_outputs", None),
                max_abs_qd_rad_s=cell_speed_limits(
                    profile.limits if profile is not None else None),
                axis_mask=cell_axis_mask(
                    moved_axes(opened.cell),
                    profile.held if profile is not None else None),
                # What the plan was made against; the daemon
                # refuses it on any other robot or machine_planning_calibration.json.
                robot=header_identity(opened.cell))
            if summary is not None:
                out["dense"] = summary
            else:
                out["dense_error"] = error

    return {"out": out,
            "moves": (len(motion_plan.connecting_trajectories)
                      if motion_plan.connecting_trajectories is not None else None),
            "joining_error": (str(motion_plan.connecting_trajopt_error)
                              if motion_plan.connecting_trajopt_error else None)}


class _PlanningWorkerError(RuntimeError):
    """Already formatted worker error, preserving the HTTP error contract."""


class _ClientDisconnected(Exception):
    """The requester left; its process and unpublished output must go away."""


def _worker_command(job_dir: Path) -> list[str]:
    return [sys.executable, "-m", "weld_motion_planner.server.motion_planner_server",
            "--worker-job", str(job_dir)]


def _spare_command() -> list[str]:
    return [sys.executable, "-m", "weld_motion_planner.server.motion_planner_server",
            "--worker-spare"]


#: What a plan imports, loaded by the spare before it has a job: the modules a
#: `test1` plan on motion-cpu loaded after the server module (2026-10-11).
#: Importing touches no device; a missing optional one is skipped, never fatal.
WARM_MODULES = (
    "torch", "scipy.interpolate", "scipy.spatial.transform", "coal", "collada",
    "weld_motion_planner.io.wire",
    "weld_motion_planner.io.native_result",
    "weld_motion_planner.io.dense_output",
    "weld_motion_planner.planner.planner_main",
    "weld_motion_planner.planner.trajectory_optimization.connecting_trajopt_main",
    "weld_motion_planner.planner.trajectory_optimization.weld_trajopt_main",
    "weld_motion_planner.planner.trajectory_optimization.engine.freespace",
    "weld_motion_planner.planner.trajectory_optimization.engine.taught",
    "weld_motion_planner.verifier.model",
    "weld_motion_planner.verifier.verify",
    "weld_motion_planner.robot_cell.collision.fixtures",
    "weldplan.dense_joint_trajectory",
    "weldplan.io_schedule",
    "weldplan.measured_start",
    "weldplan.native_admission",
    "weldplan.robot_description",
)


def _spare_main() -> int:
    """Import what a plan needs, then wait for the one job this process runs.

    End of input without a job is the server going away: exit quietly.
    """
    import importlib
    for name in WARM_MODULES:
        try:
            importlib.import_module(name)
        except Exception as exc:  # noqa: BLE001 - the job imports it again and says why
            print(f"planning spare: {name} not preloaded ({type(exc).__name__}: {exc})",
                  file=sys.stderr, flush=True)
    line = sys.stdin.readline().strip()
    if not line:
        return 0
    return _worker_main(Path(line))


class _SpareWorker:
    """At most one planning process started ahead of its request.

    `hand` gives the waiting spare its job directory and returns it, or
    None when there is no live spare (it died, or was never started), and the
    caller then starts a fresh worker as before. `replenish` starts the next
    spare; the server calls it only once the previous plan's process is gone,
    so a spare's imports never compete with a running plan for the CPU.
    """

    def __init__(self) -> None:
        self._process = None
        self._starting: asyncio.Task | None = None

    def replenish(self) -> None:
        if self._starting is None or self._starting.done():
            self._starting = asyncio.ensure_future(self._start())

    async def _start(self) -> None:
        if self._process is not None and self._process.returncode is None:
            return
        try:
            self._process = await asyncio.create_subprocess_exec(
                *_spare_command(), stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE,
                start_new_session=os.name == "posix", limit=1024 * 1024)
        except OSError as exc:
            self._process = None
            print(f"planning spare: not started ({exc}); plans start their own worker",
                  file=sys.stderr, flush=True)

    @property
    def ready(self) -> bool:
        return self._process is not None and self._process.returncode is None

    async def hand(self, job_dir: Path):
        if self._starting is not None:
            await asyncio.shield(self._starting)
        process, self._process = self._process, None
        if process is None or process.returncode is not None:
            return None
        try:
            process.stdin.write(f"{job_dir}\n".encode())
            await process.stdin.drain()
            process.stdin.close()
        except (BrokenPipeError, ConnectionResetError):
            await _stop_worker(process)
            return None
        return process

    async def stop(self) -> None:
        if self._starting is not None:
            await asyncio.gather(self._starting, return_exceptions=True)
        process, self._process = self._process, None
        if process is not None:
            await _stop_worker(process)


def _worker_main(job_dir: Path) -> int:
    """A fresh interpreter owns CUDA: never fork an initialized CUDA context."""
    try:
        result = _execute_plan_job(
            (job_dir / "request.weldplan").read_bytes(),
            json.loads((job_dir / "parameters.json").read_text()),
            job_dir / "dense",
            lambda stage: print("ROSIE_PROGRESS:" + json.dumps(stage), flush=True))
        (job_dir / "result.json").write_text(json.dumps(result), encoding="utf-8")
        return 0
    except Exception as exc:
        (job_dir / "error.json").write_text(
            json.dumps({"error": f"{type(exc).__name__}: {exc}"}), encoding="utf-8")
        import traceback
        traceback.print_exc()
        return 1


async def _stop_worker(process) -> None:
    """Wait for GPU work to die before another plan may take the lock."""
    def stop(sig):
        try:
            if os.name == "posix":
                os.killpg(process.pid, sig)
            elif sig == signal.SIGTERM:
                process.terminate()
            else:
                process.kill()
        except ProcessLookupError:
            pass
    stop(signal.SIGTERM)
    try:
        await asyncio.wait_for(process.wait(), timeout=2.0)
    except asyncio.TimeoutError:
        pass
    # The leader can exit before its ProcessPool workers. Always kill the
    # remaining group, including workers ignoring TERM or retaining stdout.
    if os.name == "posix":
        stop(signal.SIGKILL)
    elif process.returncode is None:
        process.kill()
    await process.wait()


async def _run_plan_job(data: bytes, parameters: dict[str, Any], dense_dir: Path,
                        request, progress, spare: _SpareWorker | None = None) -> dict[str, Any]:
    """Disconnect kills the isolated solver, including native/GPU calls.

    A fresh process per plan, because a cancelled Python thread cannot
    interrupt a native solve. With a `spare` that process was started (and
    imported) ahead of the request; without one, or when it died, the plan
    starts its own. Dense files stay private until the worker succeeds and the
    requesting client is still here.
    """
    from weld_motion_planner.io.dense_output import dense_blob_path

    dense_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".planning-", dir=dense_dir) as directory:
        job_dir = Path(directory)
        (job_dir / "request.weldplan").write_bytes(data)
        (job_dir / "parameters.json").write_text(json.dumps(parameters), encoding="utf-8")
        if await request.is_disconnected():
            raise _ClientDisconnected()
        handing = asyncio.create_task(spare.hand(job_dir)) if spare is not None else None
        try:
            process = await asyncio.shield(handing) if handing is not None else None
        except asyncio.CancelledError:
            process = await handing
            if process is not None:
                draining = asyncio.create_task(process.communicate())
                await _stop_worker(process)
                await draining
            spare.replenish()
            raise
        if process is None:
            creating = asyncio.create_task(asyncio.create_subprocess_exec(
                *_worker_command(job_dir), stdout=asyncio.subprocess.PIPE,
                start_new_session=os.name == "posix", limit=1024 * 1024))
            try:
                process = await asyncio.shield(creating)
            except asyncio.CancelledError:
                # Cancellation during process creation must not orphan a solver
                # before the normal lifecycle below has acquired its handle.
                process = await creating
                draining = asyncio.create_task(process.communicate())
                await _stop_worker(process)
                await draining
                if spare is not None:
                    spare.replenish()
                raise

        async def read_progress():
            async for line in process.stdout:
                text = line.decode("utf-8", errors="replace").rstrip()
                if text.startswith("ROSIE_PROGRESS:"):
                    progress(json.loads(text[len("ROSIE_PROGRESS:"):]))
                elif text:
                    print(text, file=sys.stderr, flush=True)

        reader = asyncio.create_task(read_progress())
        stopped = False
        try:
            while process.returncode is None:
                if await request.is_disconnected():
                    raise _ClientDisconnected()
                await asyncio.sleep(0.1)
            await _stop_worker(process)
            stopped = True
            await reader
            if await request.is_disconnected():
                raise _ClientDisconnected()
            if process.returncode != 0:
                error = job_dir / "error.json"
                detail = (json.loads(error.read_text())["error"] if error.exists()
                          else f"planning worker exited with status {process.returncode}")
                raise _PlanningWorkerError(detail)
            result = json.loads((job_dir / "result.json").read_text())
            summary = result["out"].get("dense")
            if await request.is_disconnected():
                raise _ClientDisconnected()
            if summary is not None:
                digest = summary["trajectory_digest"]
                # No await between the final disconnect check and atomic
                # publication; cancelled output never becomes a stored plan.
                os.replace(dense_blob_path(job_dir / "dense", digest),
                           dense_blob_path(dense_dir, digest))
            return result
        finally:
            if not stopped:
                await _stop_worker(process)
            if not reader.done():
                reader.cancel()
            await asyncio.gather(reader, return_exceptions=True)
            if spare is not None:
                spare.replenish()


def make_app(dense_store_dir: str | os.PathLike[str] | None = None, *, warm_worker: bool = False):
    # Imported here so `--help` works without fastapi -- but re-exported into
    # module globals, because `from __future__ import annotations` makes every
    # endpoint annotation a STRING that FastAPI resolves against this module's
    # namespace. With `Request` known only inside this function, the body
    # parameter resolved as a required query field called "request" and every
    # POST bounced with 422 before the handler ran.
    from fastapi import FastAPI, HTTPException, Query, Request, Response
    from fastapi.responses import FileResponse, JSONResponse

    globals().update(Request=Request, Response=Response, Query=Query,
                     JSONResponse=JSONResponse)

    from weld_motion_planner.io.dense_output import (
        DIGEST_PATTERN,
        dense_blob_path,
    )

    # Where `dense=true` plans file the daemon's blob, by digest. Outside the
    # weld-planner workspace on purpose: the OLP fetches it by digest from
    # this server, and a workspace reset must not take the blob a program
    # still plays by.
    dense_dir = Path(os.path.expanduser(
        str(dense_store_dir or os.environ.get("WELD_PLANNER_DENSE_STORE_DIR")
            or DEFAULT_DENSE_STORE_DIR)))

    # A plan that came back without a dense trajectory keeps its full result
    # beside the request, so diagnosis does not repeat an expensive search.
    # The refusal names a seam and a verdict, and the request is
    # otherwise gone: it was built in a browser from a project that lives in
    # that browser's IndexedDB, so nobody but the person who pressed Plan can
    # reproduce it. Kept by request digest, newest `REFUSED_KEPT` only -- a
    # request is megabytes of STEP and this is a bench aid, not an archive.
    refused_dir = dense_dir / "refused"

    def keep_refused(data: bytes, request_sha256: str, out: dict[str, Any]) -> None:
        result_tmp = refused_dir / f"{request_sha256}.result.json.tmp"
        try:
            refused_dir.mkdir(parents=True, exist_ok=True)
            path = refused_dir / f"{request_sha256}.weldplan"
            path.write_bytes(data)
            result_path = path.with_suffix(".result.json")
            result_tmp.write_text(json.dumps(out, ensure_ascii=False, allow_nan=False), encoding="utf-8")
            result_tmp.replace(result_path)
            kept = sorted(refused_dir.glob("*.weldplan"), key=lambda f: f.stat().st_mtime)
            for stale in kept[:-REFUSED_KEPT]:
                stale.with_suffix(".result.json").unlink(missing_ok=True)
                stale.unlink(missing_ok=True)
            print(f"motion/plan: refused ({out['dense_error']}); request kept at {path}; result at {result_path}",
                  file=sys.stderr, flush=True)
        except (OSError, TypeError, ValueError) as exc:  # the refusal still answers; only the copy is lost
            print(f"motion/plan: refused, and diagnostic artifacts could not be kept: {exc}",
                  file=sys.stderr, flush=True)
        finally:
            try:
                result_tmp.unlink(missing_ok=True)
            except OSError:
                pass

    spare = _SpareWorker() if warm_worker else None

    @contextlib.asynccontextmanager
    async def lifespan(_app):
        if spare is not None:
            spare.replenish()
        try:
            yield
        finally:
            if spare is not None:
                await spare.stop()

    app = FastAPI(title="weld motion planner", docs_url=None, redoc_url=None, lifespan=lifespan)
    # One plan at a time. The pipeline saturates the GPU on its own, and two
    # interleaved requests would each run slower than the two in sequence --
    # plus compete for memory. The lock is released only after the worker exits.
    lock = asyncio.Lock()
    # The in-flight plan's current stage, for an operator watching a
    # minutes-long request. One slot, because the lock above means one plan.
    live = {"stage": None}

    @app.get("/api/motion/progress")
    def progress_endpoint() -> dict[str, Any]:
        return {"stage": live["stage"]}

    # source_revision is the commit the deploy tool installed this server
    # from (AMR_WELD_PLANNER_SOURCE_REVISION, null when unset). `olp install`
    # refuses a restart that answers from any other commit, and `olp status`
    # reports it: before 2026-10-11 a hand-made unit ran a 5-day-old tree
    # behind a current OLP and nothing could tell.
    @app.get("/api/motion/health")
    def health() -> dict[str, Any]:
        import torch
        from weld_motion_planner.io.native_result import source_revision_from_environment
        ok = torch.cuda.is_available()
        return {
            "cuda": ok,
            "device": torch.cuda.get_device_name(0) if ok else None,
            "source_revision": source_revision_from_environment(),
            "warm_worker": spare is not None and spare.ready,
        }

    @app.post("/api/motion/plan")
    async def plan_endpoint(
        request: Request,
        k_best: int = Query(default=SeamSearchOptions.k_best, ge=1, le=16),
        screen_collisions: bool = Query(default=SeamSearchOptions.screen_collisions),
        samples_per_seam: int | None = Query(default=SeamSearchOptions.samples_per_seam,
                                            ge=2, le=512),
        run_weld_trajopt: bool = Query(
            default=MotionPlanningOptions.run_weld_trajopt),
        run_connecting_trajopt: bool = Query(
            default=MotionPlanningOptions.run_connecting_trajopt),
        free_space_backend: str = Query(
            default="bspline",
            description="which solver runs the connecting motions: "
                        "'bspline' (the default), 'curobo', or 'legacy' "
                        "(DEPRECATED Catmull-Rom, kept to reproduce old "
                        "results)"),
        verify: bool = Query(default=WeldTrajoptOptions.verify),
        verify_margin_mm: float = Query(
            default=WeldTrajoptOptions.verify_margin_m * 1000.0, ge=0.0, le=50.0),
        order_seams: bool = Query(
            default=ConnectingTrajoptOptions.order_seams,
            description="visit the welds in the order that keeps the transits "
                        "short instead of the program's own order; off, the "
                        "plan walks the welds as the program lists them"),
        # The joint-space daemon's blob, built from this plan's own result in
        # the same request. Needs both refine stages: the exporter reads the
        # weld trajectories and the connecting moves. The three identity
        # fields are the caller's half of what the daemon fences on; the
        # other half (the request digest) is this server's.
        dense: bool = Query(default=False),
        program_id: str = Query(default="", max_length=120),
        manifest_revision: int = Query(default=1, ge=1),
        plan_revision: int = Query(default=1, ge=1),
    ) -> Response:
        # Validated HERE, not only where the connecting stage dispatches:
        # a request that never runs the connecting stage used to carry a
        # misspelled backend straight through and answer as if it had been
        # understood. No aliases -- "control" died with the rename.
        free_space_backend = free_space_backend.strip().lower()
        if free_space_backend not in ("bspline", "curobo", "legacy"):
            raise HTTPException(
                422, f"unknown free_space_backend {free_space_backend!r}; "
                     "expected 'bspline', 'curobo' or 'legacy'")
        """Plan every seam of the posted `.weldplan`.

        What each option MEANS and what it costs is on `MotionPlanningOptions`, and its
        defaults are read from there rather than restated -- they were written
        here as well until the verify margin ended up as `2.0` in millimetres
        beside `0.002` in metres, the same policy in two files agreeing by hand.

        What is genuinely this layer's is the RANGES. `MotionPlanningOptions` says what the
        planner does with a number; `ge=`/`le=` say which numbers a stranger may
        post, and 512 samples or a 50 mm pad are refusals about strangers.

        Every option is ADDITIVE. A reader that knows only about candidates
        keeps working whatever is asked for.
        """
        data = await request.body()
        if not data:
            return JSONResponse({"error": "an empty body is not a .weldplan"},
                                status_code=400)
        # The REQUEST's clock, which is a longer thing than the plan's.
        #
        # `plan` prints its own stage breakdown and reports it on the wire, and
        # neither covers what surrounds it here: opening the plan request, the wait
        # on `lock` while another request finishes, and encoding a response that
        # is megabytes of joint angles. Those are the three ways a plan that the
        # planner says took nine seconds arrives after thirty, and they are
        # invisible from inside it. `queued` is separated for the same reason
        # `scene` is: waiting is not working, and a server whose queue is the
        # problem must not read as a planner that got slower.
        started = time.perf_counter()


        parameters = {
            "k_best": k_best,
            "screen_collisions": screen_collisions,
            "samples_per_seam": samples_per_seam,
            "run_weld_trajopt": run_weld_trajopt,
            "run_connecting_trajopt": run_connecting_trajopt,
            "free_space_backend": free_space_backend,
            "order_seams": order_seams,
            "verify": verify,
            "verify_margin_mm": verify_margin_mm,
            "dense": dense,
            "program_id": program_id,
            "manifest_revision": manifest_revision,
            "plan_revision": plan_revision,
        }
        acquired = False
        try:
            # Queued requests remain cancellable without touching the active
            # plan's stage or GPU process.
            while True:
                if await request.is_disconnected():
                    raise _ClientDisconnected()
                try:
                    await asyncio.wait_for(lock.acquire(), timeout=0.1)
                    break
                except asyncio.TimeoutError:
                    continue
            acquired = True
            admitted = time.perf_counter()
            result = await _run_plan_job(
                data, parameters, dense_dir, request,
                lambda stage: live.__setitem__("stage", stage), spare)
            out = result["out"]
        except _ClientDisconnected:
            return JSONResponse({"error": "Planning cancelled"}, status_code=499)
        except Exception as exc:  # noqa: BLE001 - the reason travels to the UI
            print(f"motion/plan: {len(data) / 1048576:.2f} MiB in, failed after "
                  f"{time.perf_counter() - started:.1f} s: "
                  f"{type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
            detail = str(exc) if isinstance(exc, _PlanningWorkerError) else f"{type(exc).__name__}: {exc}"
            return JSONResponse({"error": detail}, status_code=422)
        finally:
            if acquired:
                live["stage"] = None
                lock.release()
        if out.get("dense_error"):
            keep_refused(data, hashlib.sha256(data).hexdigest(), out)
        body = JSONResponse(out)
        seams = out.get("seams") or []
        crossed = sum(1 for s in seams if s.get("crossed"))
        made = sum(1 for s in seams if s.get("trajectory"))
        print(f"motion/plan: {len(data) / 1048576:.2f} MiB in, "
              f"{crossed} of {len(seams)} seams crossed, "
              f"{f'{made} trajectories, ' if run_weld_trajopt else ''}"
              f"{str(result['moves']) + ' moves, ' if result['moves'] is not None else ''}"
              f"{'joining FAILED (' + result['joining_error'] + '), ' if result['joining_error'] else ''}"
              f"{len(body.body) / 1048576:.2f} MiB out, "
              f"queued {admitted - started:.1f} s, "
              f"served in {time.perf_counter() - started:.1f} s",
              file=sys.stderr, flush=True)
        return body

    @app.get("/api/motion/dense/{trajectory_digest}")
    def dense_endpoint(trajectory_digest: str) -> Response:
        """The blob a `dense=true` plan filed, byte for byte, by digest.

        The digest is validated before the filesystem is touched: the path is
        built from it, and a digest is the only name a blob has here.
        """
        if not DIGEST_PATTERN.match(trajectory_digest):
            raise HTTPException(422, "expected sha256:<64 hex>")
        path = dense_blob_path(dense_dir, trajectory_digest)
        if not path.is_file():
            raise HTTPException(404, f"no dense trajectory {trajectory_digest}")
        return FileResponse(path, media_type="application/octet-stream",
                            headers={"Cache-Control": "no-store"})

    return app


def main(argv: list[str]) -> int:
    if (len(argv) == 2 and argv[0] == "--worker-job") or argv == ["--worker-spare"]:
        code = _worker_main(Path(argv[1])) if len(argv) == 2 else _spare_main()
        # The result or error file is closed and the plan is over: skip the
        # interpreter's teardown of torch and friends, which the server
        # waits out before it answers.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(code)
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument(
        "--dense-store-dir", default=None,
        help="where dense=true plans file the daemon's blob "
             f"(default $WELD_PLANNER_DENSE_STORE_DIR or {DEFAULT_DENSE_STORE_DIR})")
    parser.add_argument(
        "--no-warm-worker", action="store_true",
        help="start every plan's process at its request instead of keeping one "
             "imported spare waiting (costs each plan its interpreter and imports)")
    args = parser.parse_args(argv)

    import uvicorn

    uvicorn.run(make_app(dense_store_dir=args.dense_store_dir, warm_worker=not args.no_warm_worker),
                host=args.host, port=args.port, log_level="warning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
