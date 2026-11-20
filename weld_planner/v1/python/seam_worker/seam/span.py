#!/usr/bin/env python
"""A seam's stretch of its joint: the arithmetic, server-side.

The twin of `ui/src/span.ts`, and it exists for the same reason `frames.py` has
a twin in `frame-math.ts` -- two implementations of the same arithmetic drift,
so they get written down once each and pinned against each other by a test.

The one rule worth restating here, because everything else follows from it:
**`start_s > end_s` means the span wraps through s=0.** A closed loop's
reachable run can straddle the arbitrary point `chain_edges` chose to begin at.
That is one continuous stretch of weld, so it is one span, and the arithmetic
has to read 0.9 -> 0.1 as a 0.2-long forward walk rather than a negative one.

Two axes meet here and confusing them is the classic mistake:

- **s** is position along the JOINT, in [0, 1]. Frames, blocked spans and the
  weld line all live on it.
- **fraction** is position along the SEAM, in [0, 1]. Angle profiles and
  clearance results live on it, because a profile drawn over one weld means
  nothing on its neighbour.

`s_at` converts fraction to s; `fraction_at` converts back.
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any

Span = dict[str, float]


def _clamp01(value: float) -> float:
    return 0.0 if value < 0.0 else min(value, 1.0)


def wraps_through_zero(span: Span) -> bool:
    """Does this span run through s=0?"""
    return span["end_s"] < span["start_s"]


def span_fraction(span: Span) -> float:
    """How much of the joint the span covers, as a fraction of its arc length."""
    return run_fraction(span["start_s"], span["end_s"])


def run_fraction(lo: float, hi: float) -> float:
    """Length of a `[lo, hi]` run in fraction space, wrap included.

    The same arithmetic as `span_fraction`, for the places that hold a run as a
    bare pair rather than as a span -- `clearance.clear_runs` reports a run
    straddling the join of a CLOSED weld with `start > end`, and subtracting
    naively makes it negative, which then cancels part of another run in a sum.
    """
    return (1.0 - lo + hi) if hi < lo else (hi - lo)


def span_length_m(span: Span, total_m: float) -> float:
    """Arc length of a span in metres."""
    return span_fraction(span) * total_m


def s_at(span: Span, fraction: float) -> float:
    """Joint position at `fraction` along the span. 0 is start_s, 1 is end_s."""
    f = _clamp01(fraction)
    # The ends are returned exactly rather than computed: `start + 1 * length`
    # lands an ulp past `end_s`, and a seam's end IS a blocked span's start, so
    # that last ulp reads as "inside the obstruction". Measured on pipe_bracket:
    # 1 of 1001 positions tripped it.
    if f <= 0.0:
        return span["start_s"]
    if f >= 1.0:
        return span["end_s"]
    s = span["start_s"] + f * span_fraction(span)
    return s - 1.0 if s > 1.0 else s


def fraction_at(span: Span, s: float) -> float:
    """Inverse of `s_at`. Positions outside the span clamp to its ends."""
    length = span_fraction(span)
    if length <= 0.0:
        return 0.0
    offset = s - span["start_s"]
    if offset < 0.0:
        offset += 1.0
    return _clamp01(offset / length)


def contains_s(span: Span, s: float) -> bool:
    """Is `s` on this span? Boundaries count as on it."""
    if wraps_through_zero(span):
        return s >= span["start_s"] or s <= span["end_s"]
    return span["start_s"] <= s <= span["end_s"]


def self_test() -> int:
    plain = {"start_s": 0.083083, "end_s": 1.0}
    wrapped = {"start_s": 0.695123, "end_s": 0.140764}

    assert not wraps_through_zero(plain)
    assert wraps_through_zero(wrapped)
    assert abs(span_fraction(plain) - 0.916917) < 1e-9
    assert abs(span_fraction(wrapped) - 0.445641) < 1e-9

    assert s_at(plain, 0.0) == plain["start_s"]
    assert s_at(plain, 1.0) == plain["end_s"]
    assert s_at(wrapped, 1.0) == wrapped["end_s"]
    assert abs(s_at(wrapped, 0.75) - 0.02935375) < 1e-9, "3/4 along crosses s=0"
    assert abs(fraction_at(wrapped, s_at(wrapped, 0.9)) - 0.9) < 1e-9

    assert contains_s(wrapped, 0.01) and contains_s(wrapped, 0.99)
    assert not contains_s(wrapped, 0.4)

    print("span self-test ok: wrap arithmetic, exact ends, round trip")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--fixture", default="", help="JSON with spans and fractions")
    parser.add_argument("--self-test", action="store_true")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not ns.fixture:
        raise ValueError("one of --fixture or --self-test is required")

    from pathlib import Path

    case: dict[str, Any] = json.loads(Path(ns.fixture).read_text(encoding="utf-8"))
    print(
        json.dumps(
            [
                [s_at(span, f) for f in case["fractions"]] + [fraction_at(span, s) for s in case["positions"]]
                for span in case["spans"]
            ]
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"span: {exc}", file=sys.stderr)
        raise SystemExit(1)
