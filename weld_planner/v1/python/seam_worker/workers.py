"""What runs in the pool, and why it is a pool at all.

Split out of the server for one reason beyond size: **these functions must be
importable in a fresh interpreter.** `POOL_START_METHOD` is spawn, so every
worker re-imports the module holding them, and a child that had to import the
HTTP layer to reach `_work_topology` would be importing FastAPI, the routers
and everything they touch to do a CAD extraction. Module level, no closures,
no app.

There is nothing above this any more. The server and its engine are gone
(D24), and `offline-programming/v1/internal/seam` invokes these directly, the
way `internal/cad` invokes `extract_weld_edges.py`: one process per request,
with concurrency and cancellation owned by the caller. What used to be a rule
about imports is now simply the shape of the package.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

# The pool is started with SPAWN, not the Linux default of fork.
#
# OCCT's boolean algorithms run on an internal thread pool, and those threads
# outlive the operation. `fork()` copies only the calling thread, so any mutex
# an OCCT worker happened to be holding is copied in the LOCKED state and never
# released -- the child then blocks forever on its first OCCT call, with no
# error and no traceback. Reproduced exactly: `pytest tests/programmer/test_clearance.py
# tests/programmer/test_server.py` hangs where either file alone passes, because the
# clearance fixtures cut a wedge out of a block before the server forks.
#
# It would have reached production the first time anything ran a boolean in the
# web process. Spawn costs a fresh interpreter and a cold OCCT import per
# worker -- which is precisely what `_pool_init` and the readiness endpoint
# already exist to pay for at startup rather than inside a request.
POOL_START_METHOD = "spawn"


# --------------------------------------------------------------------------
# pool workers -- module level so they pickle
# --------------------------------------------------------------------------


def _pool_init() -> None:
    """Import OCCT once per worker, at startup, so no request pays for it.

    This is the whole reason the pool is warmed: the original defect was a cold
    2.5 GB environment bootstrap happening inside a request with a 90 s timeout
    on it.
    """
    import cadquery  # noqa: F401

    from seam_worker.cad import topology  # noqa: F401
    from seam_worker.reach import access, clearance  # noqa: F401
    from seam_worker.seam import detect  # noqa: F401


def _work_topology(step_path: str) -> dict[str, Any]:
    from seam_worker.cad.topology import extract_topology

    return extract_topology(Path(step_path))


def _work_detect(topology_doc: dict[str, Any], rail_offset_m: float | None) -> dict[str, Any]:
    from seam_worker.seam.detect import DEFAULT_RAIL_OFFSET_M, detect_joints

    # `detect_joints` has its own default and it is the one that matters: the
    # rail is the weld line moved along the groove bisector, and the OFFSET is
    # the only thing that gives it a direction. At zero the rail lies on the
    # seam, `seam_frames` reads (rail - point) as the zero vector, and the work
    # angle ends up measured from world up -- 45 degrees off on every 90 degree
    # fillet, and a degenerate zero reference wherever the fallback also
    # collapses. So absent means "use the default", never "use zero".
    return detect_joints(
        topology_doc,
        DEFAULT_RAIL_OFFSET_M if rail_offset_m is None else rail_offset_m,
    )


def _work_access_one(
    step_path: str,
    key_by_edge_id: dict[str, str],
    joint: dict[str, Any],
    intersector: dict[str, Any] | None = None,
) -> dict[str, Any]:
    from seam_worker.reach.access import annotate_one

    return annotate_one(Path(step_path), key_by_edge_id, joint, **_ray_kwargs(intersector))


def _ray_kwargs(intersector: dict[str, Any] | None) -> dict[str, Any]:
    """`settings.intersector_of`'s block as the arguments the workers take.

    `occt` has no tessellation, and both `access.make_engine` and
    `clearance.ray_engine` refuse a deflection alongside it rather than
    ignoring one -- so the tolerances are dropped here instead of being passed
    and rejected. Resolved in the parent and handed down: see `annotate_one`
    for why a worker must not read settings for itself.
    """
    if not intersector:
        return {}
    engine = intersector.get("engine")
    if engine == "occt":
        return {"engine": engine}
    return {
        k: v
        for k, v in (
            ("engine", engine),
            ("deflection_mm", intersector.get("deflection_mm")),
            ("dilation_mm", intersector.get("dilation_mm")),
        )
        if v is not None
    }


# One parsed STEP per worker, and only the most recent.
#
# This is intra-process memoisation of a parsed INPUT, not the result cache
# R-UI-6 withdrew: it holds no answer to any question, it dies with the worker,
# and re-reading the same file would produce the same shape by definition. It
# exists because the lattice is submitted as a dozen chunks of the same file,
# and parsing pipe_bracket costs about as much as a whole chunk of rays.
_LOADED: dict[str, Any] = {}


def _shape_for(step_path: str) -> Any:
    from seam_worker.cad.topology import load_step

    if step_path not in _LOADED:
        _LOADED.clear()
        _LOADED[step_path] = load_step(Path(step_path))
    return _LOADED[step_path]


def _work_clearance(
    step_path: str,
    joint: dict[str, Any],
    seam: dict[str, Any],
    torch: dict[str, Any],
    options: dict[str, Any],
) -> dict[str, Any]:
    """Torch clearance for one weld. On demand, never a pipeline stage.

    It depends on the torch and on the authored profiles, neither of which
    exists at analyze time -- and a stage would want to write its answer into
    `joints.json`, which is exactly the tooling-independent artifact the
    contract test guards.
    """
    from seam_worker.reach.clearance import (
        DEFAULT_SAMPLE_STEP_MM,
        FINE_RAY_COUNT,
        clearance,
    )

    return clearance(
        _shape_for(step_path),
        joint,
        seam,
        torch,
        ray_count=int(options.get("ray_count") or FINE_RAY_COUNT),
        sample_step_mm=float(options.get("sample_step_mm") or DEFAULT_SAMPLE_STEP_MM),
        standoff_mm=float(options["standoff_mm"]),
        # The STEP path, not `id(shape)`: the triangle index is cached under
        # this, and a recycled address after `_LOADED.clear()` would hand one
        # workpiece the index built for another.
        key=step_path,
        **_ray_kwargs(options.get("intersector")),
    )


def plan_provenance(torch: dict[str, Any], options: dict[str, Any]) -> dict[str, Any]:
    """What a coverage number has to carry to still mean something tomorrow.

    R-UI-2. Without this the field row can only say `optimizer`, and a result
    measured with a Ø16 nozzle would keep reading as fact after the operator
    switched to a Ø25 neck -- the same number, now describing a torch that
    cannot enter the groove at all.
    """
    return {
        **{k: torch[k] for k in ("id", "label", "body_radius_mm", "body_length_mm") if k in torch},
        "authored_standoff_mm": options["authored_standoff_mm"],
        "standoff_cost_deg_per_mm": options["standoff_cost_deg_per_mm"],
        "work_bin_deg": options["work_bin_deg"],
        "travel_bin_deg": options["travel_bin_deg"],
        "knot_spacing_mm": options["knot_spacing_mm"],
        "sample_step_mm": options["sample_step_mm"],
    }


def _work_lattice(
    step_path: str,
    joint: dict[str, Any],
    seam: dict[str, Any],
    torch: dict[str, Any],
    options: dict[str, Any],
    index_range: list[int],
) -> dict[str, Any]:
    """One chunk of the angle lattice: the expensive half of the optimiser.

    Submitted per chunk rather than whole for the same reason accessibility is
    submitted per joint -- it is where all the wall clock is, and one opaque
    submission gives neither progress nor parallelism.
    """
    from seam_worker.reach.clearance import build_lattice

    return build_lattice(
        _shape_for(step_path),
        joint,
        seam,
        torch,
        work_bins=options["work_bins"],
        travel_bins=options["travel_bins"],
        standoffs_mm=options["standoffs_mm"],
        sample_step_mm=options["sample_step_mm"],
        knot_spacing_mm=options["knot_spacing_mm"],
        ray_count=options.get("ray_count"),
        index_range=(int(index_range[0]), int(index_range[1])),
        # See `_work_clearance`: chunks of one lattice must share an index, and
        # sharing it by address is exactly what `occlusion.index_for` warns off.
        key=step_path,
        **_ray_kwargs(options.get("intersector")),
    )




# ---- the stdin entry point ---------------------------------------------------
#
# One request in, one response out, one process. There is no server above this
# any more (D24), so what used to be a route is an operation named in the
# request body. `offline-programming/v1/internal/seam` is the caller and owns
# the bounds -- payload size, timeout, concurrency, cancellation -- the way
# `internal/cad` does for `extract_weld_edges.py`.
#
# The STEP travels as base64 in the request rather than being resolved from a
# workspace by hash. That is the whole point of D24: this worker holds nothing
# between calls, so its answer is a function of its request and nothing else.


class WorkerError(ValueError):
    """A request this worker will not act on, named so the caller can classify."""

    def __init__(self, code: str, detail: str):
        self.code = code
        super().__init__(detail)


def _step_file(request: dict[str, Any], stack: Any) -> str:
    """The request's STEP, on disk, because OCCT reads paths and not bytes."""
    import base64
    import binascii
    import tempfile

    encoded = request.get("step_base64")
    if not isinstance(encoded, str) or not encoded:
        raise WorkerError("invalid_request", "step_base64 is required")
    try:
        payload = base64.b64decode(encoded, validate=True)
    except (binascii.Error, ValueError) as error:
        raise WorkerError("invalid_request", f"step_base64 is not base64: {error}") from error
    if not payload:
        raise WorkerError("invalid_request", "step_base64 decodes to nothing")
    directory = stack.enter_context(tempfile.TemporaryDirectory(prefix="seam-worker-"))
    path = Path(directory) / "part.step"
    path.write_bytes(payload)
    return str(path)


def _required(request: dict[str, Any], key: str) -> Any:
    value = request.get(key)
    if value is None:
        raise WorkerError("invalid_request", f"{key} is required")
    return value


def _op_detect_joints(request: dict[str, Any], stack: Any) -> dict[str, Any]:
    """Topology and joints in one call. They were two stages of a job because a
    server streamed progress between them; a caller that waits for one answer
    has no use for the seam."""
    step_path = _step_file(request, stack)
    topology = request.get("topology")
    if topology is None:
        topology = _work_topology(step_path)
    requested = request.get("rail_offset_m")
    joints = _work_detect(topology, None if requested is None else float(requested))

    # Accessibility, between the joints and the seams, exactly where the
    # authoring server's job ran it. `default_seams` makes one seam per
    # REACHABLE run, and without this every run is the whole loop: the stage
    # went missing when the server was deleted (c4cd1f1), and bracket_b_2x's
    # weld_joint_0002 came back as four corner-to-corner pieces with nothing
    # trimmed where no torch reaches. The motion planner crossed neither of the
    # operator's two untrimmed seams with angles that cross on the trimmed
    # span. Measured on bracket_b_2x: 0.27 s here against 0.50 s of topology.
    from seam_worker.reach.access import annotate

    joints = annotate(
        Path(step_path), topology, joints, **_ray_kwargs(request.get("intersector"))
    )

    # The default seams ride along. The server produced them in the route that
    # served the joints, and the OLP's `RailsJointsDoc` expects both in one
    # document -- a joints answer with no seams is a list of places a weld
    # COULD go and nothing an operator can click.
    from seam_worker.seam.seams import default_seams

    options = request.get("seam_options") or {}
    produced = default_seams(joints, **{
        key: float(options[key])
        for key in ("min_length_m", "max_corner_angle_deg")
        if key in options
    })
    return {"topology": topology, "joints": {**joints, **produced}}


def _op_check_torch_fits(request: dict[str, Any], stack: Any) -> dict[str, Any]:
    return {
        "clearance": _work_clearance(
            _step_file(request, stack),
            _required(request, "joint"),
            _required(request, "seam"),
            _required(request, "torch"),
            request.get("options") or {},
        )
    }


def _mirrored(inputs: dict[str, Any]) -> dict[str, Any]:
    """The same search, for the seam run the other way round.

    Reversing the run negates the tangent, and that is exactly a negation of
    both angles: a profile of +12 degrees travel welded forwards is -12
    welded backwards, against the same metal. So the window, the authored
    profile and the mask all mirror about zero, and nothing about the part
    changes.

    Done here rather than by widening the bins to hold both windows at once.
    That trick re-indexes one table for both directions and casts no second
    pass, but it only pays when the authored angle sits inside the deviation;
    outside it the union runs away -- +45 degrees at +/-10 needs 23 bins where
    two tight runs need 5 each. Two runs is the same arithmetic every time,
    and there is no mirroring left in the lattice to reason about.
    """
    out = dict(inputs)
    for axis in ("work", "travel"):
        out[f"{axis}_bins"] = sorted(-value for value in inputs[f"{axis}_bins"])
        out[f"authored_{axis}"] = [-value for value in inputs[f"authored_{axis}"]]
        mask = inputs.get(f"{axis}_limits")
        if isinstance(mask, dict):
            mirrored = {k: v for k, v in mask.items() if k not in ("min", "max")}
            # min and max swap as well as negate: the lower bound of the
            # mirrored window is the negation of the UPPER bound of this one.
            mirrored["min"] = [-value for value in mask["max"]]
            mirrored["max"] = [-value for value in mask["min"]]
            out[f"{axis}_limits"] = mirrored
    return out


def _negated_profile(profile: dict[str, Any]) -> dict[str, Any]:
    """A scalar function's values negated, on the same fractions."""
    flip = lambda value: -float(value) if value else 0.0  # noqa: E731 -- no -0.0 in a document
    kind = profile.get("kind")
    if kind == "constant":
        return {"kind": "constant", "value": flip(profile["value"])}
    if kind == "piecewise_linear":
        return {"kind": "piecewise_linear", "knots": [[s, flip(v)] for s, v in profile["knots"]]}
    raise WorkerError("internal", f"cannot reverse a {kind!r} profile")


def _as_reversed_weld(plan: dict[str, Any], authored_direction: str) -> dict[str, Any]:
    """A plan searched over the MIRRORED table, restated as the weld it describes.

    The search behind a `reverse` request runs forwards over negated angles, so
    its profiles are the reversed weld's poses written in the forward frame:
    `u(-t, work, travel) == u(t, -work, -travel)`, and a reversed seam changes
    only the sign of `t` -- fractions run the same way (`clearance.SeamGeometry`).
    Returned as it was, that plan said `direction: forward` with, say, -10 deg
    of travel: a PUSH weld run forwards, which nobody chose. The OLP applies a
    plan's direction only when `direction_changed` says so, so the winning
    reverse search was welded the wrong way round with the arc pushing.

    Negating both profiles and turning the direction gives the same torch poses
    as a reversed weld in the welder's own numbers, where +travel is drag.
    """
    out = dict(plan)
    for axis in ("work", "travel"):
        out[f"{axis}_angle_deg"] = _negated_profile(plan[f"{axis}_angle_deg"])
        out[f"{axis}_path_deg"] = [-float(v) if v else 0.0 for v in plan.get(f"{axis}_path_deg") or []]
    out["direction"] = "forward" if authored_direction == "reverse" else "reverse"
    out["direction_changed"] = True
    return out


def _op_plan_torch_path(request: dict[str, Any], stack: Any) -> dict[str, Any]:
    """Where the torch goes along one seam, and how it is held there.

    Two stages with nothing in common. `build_lattice` casts rays to measure
    how much arc is clear at every (knot, work, travel, standoff); `search`
    runs a dynamic program over that table to pick the profile that welds the
    most of it, lexicographically -- coverage first, then nearness to what was
    authored, then smoothness. The first is minutes of OCCT and the second is
    milliseconds of arithmetic on arrays, which is why they are separate
    modules and why `orient` has no OCCT in it at all.

    ONE direction per call. `direction: "reverse"` mirrors the window and
    searches the seam run the other way; a caller that wants both asks twice
    and keeps the better coverage. Nothing here decides between them, because
    the two answers are only comparable once both are measured the same way.
    """
    from seam_worker.reach.clearance import (
        DEFAULT_SAMPLE_STEP_MM,
        SeamGeometry,
        decision_knots,
        search_inputs,
    )
    from seam_worker.reach.orient import (
        DEFAULT_KNOT_SPACING_MM,
        DEFAULT_STANDOFF_COST_DEG_PER_MM,
        DEFAULT_TRAVEL_BIN_DEG,
        DEFAULT_WORK_BIN_DEG,
        search,
    )
    from seam_worker.reach.tooling import standoff_candidates, weld_standoff_mm

    step_path = _step_file(request, stack)
    joint = _required(request, "joint")
    seam = _required(request, "seam")
    torch = _required(request, "torch")
    knobs = request.get("search") or {}
    direction = str(request.get("direction") or "forward")
    if direction not in ("forward", "reverse"):
        raise WorkerError("invalid_request", f"direction must be forward or reverse, got {direction!r}")

    fallback_standoff_mm = request.get("default_standoff_mm")
    if fallback_standoff_mm is None:
        raise WorkerError(
            "invalid_request",
            "default_standoff_mm is required: a gun this worker guessed at would "
            "be a coverage number about hardware nobody has",
        )

    knot_spacing_mm = float(knobs.get("knot_spacing_mm") or DEFAULT_KNOT_SPACING_MM)
    sample_step_mm = float(knobs.get("sample_step_mm") or DEFAULT_SAMPLE_STEP_MM)
    geometry = SeamGeometry(joint, seam)
    knots, _ = decision_knots(geometry, knot_spacing_mm)
    inputs = search_inputs(
        seam,
        knots,
        work_deviation_deg=float(knobs["work_deviation_deg"]),
        travel_deviation_deg=float(knobs["travel_deviation_deg"]),
        work_bin_deg=float(knobs.get("work_bin_deg") or DEFAULT_WORK_BIN_DEG),
        travel_bin_deg=float(knobs.get("travel_bin_deg") or DEFAULT_TRAVEL_BIN_DEG),
        both_directions=False,
    )
    if direction == "reverse":
        inputs = _mirrored(inputs)

    authored_standoff_mm = weld_standoff_mm(seam, float(fallback_standoff_mm))
    standoffs_mm = standoff_candidates(seam, float(fallback_standoff_mm))
    samples = len(geometry.fractions(sample_step_mm))
    options = {
        **inputs,
        "standoffs_mm": standoffs_mm,
        "sample_step_mm": sample_step_mm,
        "knot_spacing_mm": knot_spacing_mm,
        "ray_count": knobs.get("ray_count"),
        "intersector": request.get("intersector"),
    }
    lattice = _work_lattice(step_path, joint, seam, torch, options, [0, samples])
    plan = search(
        lattice,
        authored_work=inputs["authored_work"],
        authored_travel=inputs["authored_travel"],
        authored_standoff_mm=authored_standoff_mm,
        work_limits=inputs.get("work_limits"),
        travel_limits=inputs.get("travel_limits"),
        standoff_cost_deg_per_mm=float(
            knobs.get("standoff_cost_deg_per_mm", DEFAULT_STANDOFF_COST_DEG_PER_MM)
        ),
        # The mirroring is already in `inputs`, so the DP searches one table
        # forwards. Asking it for both here would negate what is negated.
        directions=("forward",),
        authored_direction="forward",
    )
    if direction == "reverse":
        plan = _as_reversed_weld(plan, str(seam.get("direction") or "forward"))
    return {
        # What was asked for. The plan says the same thing now, in the weld's
        # own terms: a reverse search answers with a reversed weld.
        "direction": direction,
        "plan": plan,
        # What it cost, in the unit that does not depend on the machine.
        "rays_cast": lattice.get("rays_cast"),
        "samples": samples,
        "warnings": inputs.get("warnings") or [],
    }


def _op_sample_seam_frames(request: dict[str, Any], stack: Any) -> dict[str, Any]:
    """The `{P, t, r, b}` a viewer needs at each point along a seam.

    No STEP, no torch, no rays: a curve and the rail its work angle is measured
    from are the whole input. It is here rather than in the browser for the
    reason `frames.py` opens with -- if the client computes torch orientation
    in TypeScript and the server computes it in Python when saving, the two can
    diverge, and a sign-convention divergence is the bug class the seam model
    exists to avoid. Round-tripping on every slider drag would be unusable, so
    the split is: the server sends frames, the client applies angles. Frames do
    not depend on the work or travel angle, so a drag needs no call at all.
    """
    from seam_worker.seam.frames import seam_frames

    samples = request.get("samples")
    return {
        "frames": seam_frames(
            _required(request, "curve"),
            _required(request, "reference"),
            samples=int(samples) if samples is not None else None,
            max_chord_m=request.get("max_chord_m"),
            max_angle_rad=request.get("max_angle_rad"),
        )
    }


def _context_members(context: Any) -> dict[str, bytes] | None:
    """The planning context as the OLP sent it: members by path, base64. This
    decodes and packs; it opens none of them. The robot in there was compiled
    by the OLP, and a worker that read it would be a second reader."""
    import base64

    if context is None:
        return None
    if not isinstance(context, dict) or not isinstance(context.get("files"), dict):
        raise WorkerError("invalid_request", "planning_context must carry a files object")
    out: dict[str, bytes] = {}
    for path, encoded in context["files"].items():
        if not isinstance(encoded, str):
            raise WorkerError("invalid_request", f"planning_context.files[{path!r}] must be base64 text")
        try:
            out[str(path)] = base64.b64decode(encoded, validate=True)
        except Exception as error:
            raise WorkerError("invalid_request",
                              f"planning_context.files[{path!r}] is not base64: {error}") from error
    return out


def _op_build_plan_request(request: dict[str, Any], stack: Any) -> dict[str, Any]:
    """Everything a motion planner needs, packed into one `.weldplan`.

    The program, the placement inside it, the planning context, the torch and
    the geometry, as bytes that hash the same every time. It is here rather
    than in the browser for the reason `plan_request.py` gives: the container
    must be built by one implementation, so "did the request change?" has one
    answer.

    The robot arrives as the PLANNING CONTEXT (`docs/PLANNING-CONTEXT.md`):
    the OLP read the description and the cell's `machine_planning_calibration.json`, judged the
    deviation and composed it onto the URDF, once. This packs those members
    verbatim and reads the cell descriptor off the context's own URDF -- it
    is handed no URDF, settings or frames of its own to combine, so there is
    nothing here that could disagree with what the OLP compiled.
    """
    import base64

    from weldplan.plan_request import PlanRequestError, write_request
    from weldplan.urdf import CellError, descriptor_of

    absent = request.get("workpiece_absent") is True
    if absent and request.get("step_base64"):
        raise WorkerError("invalid_request", "workpiece_absent cannot accompany STEP geometry")
    step = None if absent else _step_file(request, stack)
    program = _required(request, "program")
    context = _context_members(_required(request, "planning_context"))

    # The operator's workcell fixtures, when the OLP sends them: a list (maybe
    # empty) replaces the cell's placeholder; absent, the placeholder stands.
    fixtures = request.get("fixtures")
    if fixtures is not None and not isinstance(fixtures, list):
        raise WorkerError("invalid_request", "fixtures must be a list of fixture entries")
    try:
        cell = descriptor_of(context, fixtures=fixtures)
    except CellError as error:
        raise WorkerError("invalid_request", f"planning_context refused: {error}") from error
    try:
        data = write_request(
            request_id=str(request.get("request_id") or "plan"),
            program=program,
            cell=cell,
            step_bytes=None if step is None else Path(step).read_bytes(),
            step_filename=str(request.get("step_filename") or "part.step"),
            tooling=request.get("tooling"),
            label=request.get("label"),
            created_utc=request.get("created_utc"),
            fixtures=fixtures,
            context=context,
            start_state=request.get("start_state"),
            # The connected cell's outputs (weldplan.io_schedule), captured by
            # the OLP from rt-core's Describe when the program switches any.
            io_outputs=request.get("io_outputs"),
        )
    except PlanRequestError as error:
        # A refusal with the reason. Everything after this container is
        # somebody else's process, and a request that is wrong in a way only
        # the planner notices comes back as a plan that does not fit.
        raise WorkerError("invalid_request", str(error)) from error

    import hashlib

    return {
        "weldplan_base64": base64.b64encode(data).decode(),
        "bytes": len(data),
        # What Weld Preflight and the dense daemon both fence on. Returned so a
        # caller can name the request it planned without re-hashing it.
        "sha256": hashlib.sha256(data).hexdigest(),
    }


OPERATIONS = {
    "detect_joints": _op_detect_joints,
    "check_torch_fits": _op_check_torch_fits,
    "plan_torch_path": _op_plan_torch_path,
    "sample_seam_frames": _op_sample_seam_frames,
    "build_plan_request": _op_build_plan_request,
}


def run(request: dict[str, Any]) -> dict[str, Any]:
    """Dispatch one request. Every failure comes back as a named code rather
    than a traceback, because the caller classifies before it reports."""
    import contextlib

    operation = request.get("operation")
    handler = OPERATIONS.get(operation) if isinstance(operation, str) else None
    if handler is None:
        raise WorkerError(
            "invalid_request",
            f"unknown operation {operation!r}; expected one of {', '.join(sorted(OPERATIONS))}",
        )
    with contextlib.ExitStack() as stack:
        return handler(request, stack)


def _preload() -> None:
    """Import everything an operation imports lazily, before any request exists.

    For a SPARE: `internal/seam` starts a worker ahead of its request, and a
    spare that has already paid for its imports answers in 0.39 s where a
    fresh process takes 1.04 s (bracket_b_2x, plan_torch_path; 0.40 s of the
    difference is `import OCP` and the reach modules). Modules only -- no part,
    no index, nothing a request supplies -- so the answer is still a function
    of the one request this process will ever see.
    """
    import OCP  # noqa: F401
    import weldplan.plan_request  # noqa: F401
    from seam_worker.cad import topology  # noqa: F401
    from seam_worker.reach import access, clearance, orient, tooling  # noqa: F401
    from seam_worker.robot import cell  # noqa: F401
    from seam_worker.seam import detect, frames, seams  # noqa: F401


def main(argv: list[str]) -> int:
    import argparse
    import json

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--stdin", action="store_true",
                        help="read one JSON request from stdin and write one JSON response")
    parser.add_argument("--preload", action="store_true",
                        help="import every module an operation needs before reading stdin")
    arguments = parser.parse_args(argv)
    if not arguments.stdin:
        parser.error("--stdin is the only mode")
    if arguments.preload:
        _preload()

    try:
        request = json.load(sys.stdin)
    except (json.JSONDecodeError, UnicodeDecodeError) as error:
        json.dump({"error": {"code": "invalid_request", "detail": f"request is not JSON: {error}"}},
                  sys.stdout)
        return 2
    if not isinstance(request, dict):
        json.dump({"error": {"code": "invalid_request", "detail": "request must be an object"}},
                  sys.stdout)
        return 2

    try:
        json.dump(run(request), sys.stdout, allow_nan=False)
    except WorkerError as error:
        json.dump({"error": {"code": error.code, "detail": str(error)}}, sys.stdout)
        return 2
    except Exception as error:  # noqa: BLE001 -- the caller gets a code, not a traceback
        print(f"{type(error).__name__}: {error}", file=sys.stderr)
        json.dump({"error": {"code": "worker_failed", "detail": f"{type(error).__name__}: {error}"}},
                  sys.stdout)
        return 1
    return 0


if __name__ == "__main__":
    import os

    code = main(sys.argv[1:])
    sys.stdout.flush()
    sys.stderr.flush()
    # Out at once, past interpreter teardown. The answer is already written,
    # and unwinding OCCT's and every module's state measured 0.23 s per
    # process on bracket_b_2x -- time the caller spends waiting for an exit
    # status after it already has everything it asked for.
    os._exit(code)
