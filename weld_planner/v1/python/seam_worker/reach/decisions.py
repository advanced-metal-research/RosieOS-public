#!/usr/bin/env python
"""What to do about the stretches the torch cannot reach.

`clearance.py` measures. This decides nothing -- it lays out the choices, with
the arc each one welds, and the operator picks. Three of them, and they are not
degrees of the same thing:

    TRIM     weld the longest clear run and give up the rest. One entry, one
             span. Already expressible today; the weld list renders it
             `. trimmed`.
    SPLIT    weld every clear run worth striking an arc for, as separate
             entries. Needs the disambiguation in `program.resolve_weld_list`,
             because all of them reference one candidate seam.
    ACCEPT   weld it as authored, fouling and all. A real answer -- the check
             is a model, the operator is looking at the part -- but it has to
             be RECORDED, so that reopening the project shows a decision rather
             than an unnoticed collision.

## Why this is a module and not four lines in the UI

The arithmetic converts between two axes and it is exactly the kind that goes
wrong quietly. Clearance reports in **fraction along the seam**; a weld's span
is in **`s` along the joint**, which may wrap through zero. Getting that
backwards produces spans that are plausible, ordered, inside [0, 1], and weld
the wrong metal. So the conversion lives in one place, has a Python and a
TypeScript twin, and is pinned by a parity test over a wrapping span.

## The floor is a process floor

A clear window shorter than `min_clear_run_m` is not somewhere anyone strikes
an arc, so it is not offered. Those runs are reported as `dropped` rather than
dropped silently: 4 mm of reachable seam that nobody will weld is a fact about
the plan, and a total that quietly excluded it would not add up.

## Named `decisions`, not `coverage`

`python/` is `sys.path[0]`, so a module here shadows an installed package of the
same name for the whole environment. `coverage` is not installed today, but it
is one `pytest-cov` away from being so -- the same trap that made the torch
model `tooling.py` rather than `torch.py`. The TypeScript twin is
`ui/src/state/decisions.ts`, matching this rather than the plan's placeholder.
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any

from seam_worker.seam.span import run_fraction, s_at, span_fraction

SCHEMA = "amr-weld-planner-v1.coverage-decision.v1"

# Below this, a clear window is not somewhere anyone strikes an arc. Matches
# `clearance.MIN_CLEAR_RUN_M` and `seams.DEFAULT_MIN_SEAM_LENGTH_M`; the default
# is only a fallback, because a clearance result states the floor it used and
# that one wins.
MIN_RUN_M = 0.005


class DecisionError(ValueError):
    """A coverage report this refuses to reason about, with the reason."""


def _sub_span(span: dict[str, Any], lo: float, hi: float) -> dict[str, float]:
    """A stretch of seam fraction `[lo, hi]`, as a span in joint `s`.

    `s_at` carries the wrap, so a run inside a span that crosses s=0 comes back
    with `start_s > end_s` -- exactly as the unsplit span did, and exactly what
    every other consumer of a span already handles.
    """
    return {"start_s": s_at(span, lo), "end_s": s_at(span, hi)}


def offer(
    clearance: dict[str, Any],
    span: dict[str, Any],
    *,
    min_run_m: float | None = None,
) -> dict[str, Any]:
    """The choices, and what each one welds.

    `clearance` is a result from `clearance.clearance`; `span` is the arc the
    weld actually runs, in joint `s`. The floor comes from the clearance result
    unless overridden, so the two cannot disagree about what counts as weldable.
    """
    length_m = float(clearance["seam_length_m"])
    if length_m <= 0:
        raise DecisionError("a seam of zero length has nothing to decide about")
    floor = float(
        min_run_m if min_run_m is not None else clearance.get("min_clear_run_m", MIN_RUN_M)
    )

    # Wrap-aware throughout. On a CLOSED weld a run may straddle the join and
    # is reported with `start > end`; subtracting naively makes it negative,
    # which then cancels part of another run in a sum. The result is a coverage
    # figure that is too low by however much wrapped, with nothing on screen to
    # say so -- and a `longest` that picks the wrong run to trim to.
    runs = [list(map(float, run)) for run in clearance.get("clear_runs") or []]
    usable = [r for r in runs if run_fraction(*r) * length_m >= floor]
    dropped = [r for r in runs if run_fraction(*r) * length_m < floor]

    usable_m = sum(run_fraction(*r) for r in usable) * length_m
    dropped_m = sum(run_fraction(*r) for r in dropped) * length_m
    blocked_m = sum(
        run_fraction(lo, hi) for lo, hi in (clearance.get("blocked_spans") or [])
    ) * length_m
    sealed_m = sum(
        run_fraction(lo, hi) for lo, hi in (clearance.get("sealed_spans") or [])
    ) * length_m

    longest = max(usable, key=lambda r: run_fraction(*r), default=None)

    if not runs or usable_m <= 0:
        state = "blocked"
    elif blocked_m + dropped_m <= 0:
        state = "clear"
    else:
        state = "partial"

    return {
        "schema": SCHEMA,
        # `clear` means there is nothing to decide. Distinguished from
        # `partial` so the UI can stay silent rather than offering three
        # buttons for a weld with no problem.
        "state": state,
        "seam_length_m": round(length_m, 9),
        "usable_length_m": round(usable_m, 9),
        # Not silently discarded: 4 mm of reachable seam nobody will weld is a
        # fact about the plan, and a total that quietly excluded it would not
        # add up.
        "dropped_length_m": round(dropped_m, 9),
        "blocked_length_m": round(blocked_m, 9),
        # A subset of blocked, and a different decision: no angle, no standoff
        # and no optimiser run recovers these.
        "sealed_length_m": round(sealed_m, 9),
        "min_run_m": floor,
        # Passed through so a caller can tell that a run with start > end is
        # one stretch across the join rather than a mistake.
        "closed": bool(clearance.get("closed")),
        # In seam fraction, for drawing on the same axis as the profiles.
        "usable_runs": usable,
        "dropped_runs": dropped,
        # In joint `s`, ready to become `entry.span` -- which is the whole
        # reason this module exists rather than the UI doing it twice.
        "trim": _sub_span(span, longest[0], longest[1]) if longest else None,
        "trim_length_m": round(run_fraction(*longest) * length_m, 9) if longest else 0.0,
        "split": [_sub_span(span, lo, hi) for lo, hi in usable],
        "split_length_m": round(usable_m, 9),
        # Accepting welds the whole arc, fouling included. Stated as a length so
        # the three options are compared in the same unit rather than by
        # adjective.
        "accept_length_m": round(length_m, 9),
    }


def conserves(report: dict[str, Any]) -> bool:
    """usable + dropped + blocked == the whole seam.

    The tripwire. Clear runs and blocked spans partition the seam by
    construction, and `usable`/`dropped` partition the clear runs, so this must
    hold exactly -- and it is what a fraction-versus-`s` mix-up cannot survive.
    """
    total = (
        report["usable_length_m"] + report["dropped_length_m"] + report["blocked_length_m"]
    )
    return abs(total - report["seam_length_m"]) < 1e-9


def pytest_approx(value: float, tol: float = 1e-12) -> Any:
    """A tolerance without importing pytest -- `self_test` runs from the CLI."""

    class _Close(float):
        def __eq__(self, other: object) -> bool:
            return abs(float(other) - value) <= tol  # type: ignore[arg-type]

        __hash__ = float.__hash__

    return _Close(value)


def self_test() -> int:
    whole = {"start_s": 0.0, "end_s": 1.0}
    length = 0.100

    def report(runs, blocked, span=whole, **extra):
        return offer(
            {
                "seam_length_m": length,
                "clear_runs": runs,
                "blocked_spans": blocked,
                "min_clear_run_m": MIN_RUN_M,
                **extra,
            },
            span,
        )

    # Nothing wrong: one run, the whole seam.
    fine = report([[0.0, 1.0]], [])
    assert fine["state"] == "clear", fine["state"]
    assert fine["trim"] == whole and len(fine["split"]) == 1
    assert conserves(fine)

    # Blocked in the middle: two runs, both worth welding.
    two = report([[0.0, 0.4], [0.6, 1.0]], [[0.4, 0.6]])
    assert two["state"] == "partial"
    assert two["split_length_m"] == 0.080 and two["blocked_length_m"] == 0.020
    assert two["trim_length_m"] == 0.040, "the longest run, not the sum"
    assert len(two["split"]) == 2
    assert conserves(two)

    # A 2 mm stub is under the 5 mm floor: not offered, but not lost either.
    stub = report([[0.0, 0.02], [0.3, 1.0]], [[0.02, 0.3]])
    assert len(stub["split"]) == 1, "the stub is not somewhere anyone strikes an arc"
    assert stub["dropped_length_m"] == 0.002
    assert conserves(stub)

    # Nothing reachable at all.
    none = report([], [[0.0, 1.0]])
    assert none["state"] == "blocked" and none["trim"] is None and none["split"] == []
    assert conserves(none)

    # The wrap. A span through s=0 must produce sub-spans covering the right
    # arc, not ones that quietly weld the long way round.
    #
    # The first half of a span from 0.9 to 0.1 ends at s=1.0, which is the same
    # point as s=0.0 -- `s_at` wraps only ABOVE 1.0 and returns its endpoints
    # exactly, so it says 1.0. Asserted as `s_at` actually behaves rather than
    # as it might have: this module must not hold a second opinion about span
    # arithmetic, which is the whole reason that arithmetic lives in one file
    # with a parity test on it.
    outer = {"start_s": 0.9, "end_s": 0.1}
    wrapped = report([[0.0, 0.5], [0.5, 1.0]], [], span=outer)
    assert span_fraction(outer) == pytest_approx(0.2), span_fraction(outer)
    assert wrapped["trim"] == {"start_s": 0.9, "end_s": 1.0}, wrapped["trim"]
    assert wrapped["split"][1] == {"start_s": 1.0, "end_s": 0.1}, wrapped["split"][1]
    # The two halves tile the original arc, which is the property that matters.
    halves = sum(span_fraction(s) for s in wrapped["split"])
    assert abs(halves - span_fraction(outer)) < 1e-12, halves

    print(
        "decisions self-test ok: "
        f"trim {two['trim_length_m'] * 1000:.0f} mm, split "
        f"{two['split_length_m'] * 1000:.0f} mm over {len(two['split'])} runs, "
        f"accept {two['accept_length_m'] * 1000:.0f} mm"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--clearance", default="", help="JSON from `clearance`")
    parser.add_argument("--span", default='{"start_s": 0.0, "end_s": 1.0}')
    parser.add_argument("--self-test", action="store_true")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not ns.clearance:
        raise ValueError("one of --clearance or --self-test is required")

    from pathlib import Path

    result = json.loads(Path(ns.clearance).read_text(encoding="utf-8"))
    print(json.dumps(offer(result.get("clearance", result), json.loads(ns.span)), indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"decisions: {exc}", file=sys.stderr)
        raise SystemExit(1)
