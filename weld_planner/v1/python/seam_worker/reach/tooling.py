#!/usr/bin/env python
"""The torch: what it is, and the one inequality that decides where it fits.

Named `tooling` and not `torch` on purpose. `python/` is this project's
`sys.path[0]`, so a module named `torch.py` would shadow PyTorch for anything
that shares the environment -- an import error a long way from its cause.

**A torch is not part of a weld program.** The program says what weld is
required; which nozzle reaches into the groove is a property of the cell, and
`tests/programmer/test_pipe_bracket.py` greps the emitted analysis for exactly these words to
keep it that way. So the record here travels in global settings and in the
project bundle, beside the placement, and never in `weld-program.v1`.

The model is a **cylinder**, standing off the weld tip along the electrode axis:

        weld tip
           |<-- standoff -->|============ body, length L ============|
           P                            radius R about the axis

Crude, and deliberately so. The gas nozzle is the part that actually collides
with a rising flange, it really is a cylinder to within a millimetre, and a
mesh of a real torch would make the check slower without changing a single
answer at the 2 mm sampling the seam gets.

**The one piece of arithmetic worth knowing by heart.** Take the cross-section
perpendicular to the seam: the groove is two rays leaving the corner at
+/- theta/2 from the bisector, and the torch is a half-infinite strip of
half-width R starting a distance d out along that bisector. The binding point
is the strip's leading corner, at (d, R). It stays on the inside of the face
only when

    d * sin(theta/2) - R * cos(theta/2) >= 0    i.e.    theta >= 2 * atan(R / d)

`fits_dihedral_deg` returns that angle. It decides whether the feature works at
all, which is why it is stated in the settings dialog next to the radius rather
than discovered when an optimizer reports zero coverage:

    Ø25 neck   at 10 mm standoff -> cannot enter a groove tighter than 103 deg,
                                    so every 90 deg fillet reads fully blocked;
    Ø16 nozzle at 10 mm standoff -> 77 deg, and a 90 deg fillet is workable.

Pulling back buys clearance fast: the same Ø16 at 16 mm of standoff gets down to
53 deg. That is why the optimizer searches over standoff at all, and why the
only thing that can open a band for it to search is the PROCEDURE, in
`process.wire.stickout_limits_mm` -- stickout is a process parameter, not an
approach one.

**What a torch record does and does not say about standoff.** It says the RANGE
it can be held at, `standoff_range_mm`, which is a fact about the hardware: too
close and the nozzle is in the groove, too far and the gas stops covering the
puddle. It does not say the VALUE. That is the procedure's, and a gun with an
opinion about it is a second author of a number that already has one --
`weld_standoff_mm` has the history.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from typing import Any

SCHEMA = "amr-weld-planner-v1.torch.v1"

# A MIG gun's gas nozzle: 16 mm across, 150 mm of barrel behind it.
#
# `default_standoff_mm` is on its way out and is kept only until settings stop
# carrying a torch block; `standoff_range_mm` is what replaces it, and the two
# are different claims rather than the same one renamed. The first was a VALUE
# -- how far to hold the gun -- which is a procedure decision the gun has no
# vote in. The second is a RANGE the hardware can be held at, which is the only
# thing a gun is entitled to say about its own standoff. While it lasts it is
# `seam.seams.DEFAULT_STANDOFF_MM`, the cell's one default (17 mm); not imported
# from there because `seam` sits above `reach`.
DEFAULT_TORCH: dict[str, Any] = {
    "id": "mig_gas_nozzle_16",
    "label": "MIG, Ø16 gas nozzle",
    "body_radius_mm": 8.0,
    "body_length_mm": 150.0,
    "default_standoff_mm": 17.0,
    "standoff_range_mm": {"min": 6.0, "max": 25.0},
}


def problems(torch: dict[str, Any]) -> list[str]:
    """What is wrong with this torch record, as sentences. Empty means usable."""
    out: list[str] = []
    if not str(torch.get("id", "")).strip():
        out.append("a torch needs an id")
    for key in ("body_radius_mm", "body_length_mm"):
        value = torch.get(key)
        if not isinstance(value, int | float) or isinstance(value, bool):
            out.append(f"{key} must be a number, got {value!r}")
        elif not math.isfinite(float(value)) or float(value) <= 0.0:
            out.append(f"{key} must be positive, got {value}")

    # Checked only where it is still written. A torch preset does not carry one
    # -- how far the gun is held off the work is the procedure's to say -- and
    # the key exists solely until the settings block that holds it is gone.
    if "default_standoff_mm" in torch:
        value = torch["default_standoff_mm"]
        if not isinstance(value, int | float) or isinstance(value, bool):
            out.append(f"default_standoff_mm must be a number, got {value!r}")
        elif not math.isfinite(float(value)) or float(value) <= 0.0:
            out.append(f"default_standoff_mm must be positive, got {value}")

    span = torch.get("standoff_range_mm")
    if not isinstance(span, dict):
        out.append(f"standoff_range_mm must be an object with min and max, got {span!r}")
    else:
        for key in ("min", "max"):
            value = span.get(key)
            if not isinstance(value, int | float) or isinstance(value, bool):
                out.append(f"standoff_range_mm.{key} must be a number, got {value!r}")
            elif not math.isfinite(float(value)) or float(value) <= 0.0:
                out.append(f"standoff_range_mm.{key} must be positive, got {value}")
    if out:
        return out

    if float(span["min"]) > float(span["max"]):
        out.append(
            f"standoff_range_mm crosses: min {span['min']} > max {span['max']}"
        )
    # A body shorter than the furthest the gun is held models nothing: the
    # barrel would end before it reached the tip it is supposed to be behind.
    # This SUBSUMES the old check against `default_standoff_mm` -- any standoff
    # inside the range is automatically shorter than the barrel, and one
    # outside the range already warns -- so it is one inequality stated once,
    # on the document that owns both numbers.
    if float(span["max"]) >= float(torch["body_length_mm"]):
        out.append(
            "body_length_mm must exceed standoff_range_mm.max, "
            f"got {torch['body_length_mm']} <= {span['max']}"
        )
    return out


def standoff_warnings(torch: dict[str, Any], standoff_mm: float) -> list[str]:
    """Empty unless the procedure holds this gun outside the range it claims.

    A warning and not a refusal, and the shape is D17's: the range is an
    absolute band on the HARDWARE and the stickout is the PROCEDURE's own
    claim. Both apply, and reconciling them is the operator's call -- a welder
    holding a gun two millimetres past its stated range has a reason, and a
    planner deciding they are wrong does not.
    """
    span = torch.get("standoff_range_mm")
    if not isinstance(span, dict):
        return []
    label = torch.get("label") or torch.get("id") or "this torch"
    if standoff_mm < float(span["min"]):
        return [
            f"stickout {standoff_mm:.1f} mm is under {label}'s stated"
            f" {float(span['min']):.1f} mm: the nozzle is into the groove"
        ]
    if standoff_mm > float(span["max"]):
        return [
            f"stickout {standoff_mm:.1f} mm is over {label}'s stated"
            f" {float(span['max']):.1f} mm: the current drops and the gas"
            " column stops covering the puddle"
        ]
    return []


def fits_dihedral_deg(torch: dict[str, Any], standoff_mm: float) -> float:
    """Narrowest groove this torch can enter, in degrees, at this standoff.

    The bisector case, `2 * atan(R / d)`, derived in the module docstring. It
    approaches but never reaches 180 degrees, so a torch held at zero standoff
    -- where the arithmetic degenerates -- is reported as 180: nothing but a
    flat plate will take it.

    This is a NECESSARY condition on an idealized infinite groove, not the
    clearance check. A real seam ends, and a torch tilted along the seam
    escapes a corner that this arithmetic calls closed -- which is exactly the
    freedom the optimizer searches. Use it to explain a torch, not to reject a
    weld; `clearance.py` casts the rays that actually decide.
    """
    radius = float(torch["body_radius_mm"])
    standoff = float(standoff_mm)
    if standoff <= 0.0:
        return 180.0
    return math.degrees(2.0 * math.atan(radius / standoff))


def barrel_offsets(radius_mm: float, count: int) -> list[tuple[float, float]]:
    """`count` points over the disc of `radius_mm`, as (e1, e2) pairs.

    **Half on the rim, half inside**, and the split is the whole design.

    A cylinder pressed against a plane touches it on the RIM. So the accuracy of
    the whole clearance check is set by how well the rim is sampled, and putting
    points exactly on it makes the error second order: a ring of `m` points
    behaves like a cylinder of radius `R * cos(pi/m)`, which for 32 points on a
    Ø16 barrel is 0.04 mm narrow -- a fifth of a degree of reported fit angle.
    Spreading the same points evenly over the disc instead leaves gaps of
    `R * sqrt(pi / n)` near the rim, first order in the spacing, and measured on
    a synthetic 77 degree groove that read as fully clear when the closed form
    says the torch does not fit.

    The interior half is a Fibonacci sunflower, the golden-angle construction
    `access.hemisphere` uses on the sphere, equal-area so no ring is sampled
    more finely than another. It is what catches something poking up under the
    middle of the barrel -- a bolt head, a tack -- which no rim ray would touch.
    Its first point is the axis exactly, which is also the ray `clearance.py`'s
    buried-torch parity check most depends on.

    Two properties the callers rely on:

    - **Deterministic.** Same arguments, same list, in the same order -- an
      optimizer that is not reproducible cannot be debugged.
    - **Nested.** `barrel_offsets(R, n)[::k]` is a subset, drawn from both
      halves, so a coarse pass that finds a collision proves the fine pass finds
      one too. Note the polarity is the OPPOSITE of `access.py`'s escalation,
      where the cheap direction is the one that proves clear: here a hit is
      monotone in the number of rays, so a coarse fan can only ever
      under-report collisions. It is optimistic, and never conclusive on its own.
    """
    if count <= 0:
        return []
    if count == 1:
        return [(0.0, 0.0)]
    golden = math.pi * (3.0 - math.sqrt(5.0))
    rim_count = max(1, count // 2)
    inner_count = count - rim_count

    out: list[tuple[float, float]] = []
    for i in range(rim_count):
        angle = 2.0 * math.pi * i / rim_count
        out.append((radius_mm * math.cos(angle), radius_mm * math.sin(angle)))
    for i in range(inner_count):
        # i = 0 is the axis; the sequence approaches but never reaches the rim,
        # which the ring above already covers.
        r = radius_mm * math.sqrt(i / inner_count)
        angle = golden * i
        out.append((r * math.cos(angle), r * math.sin(angle)))
    return out


def weld_standoff_mm(seam: dict[str, Any], fallback_mm: float) -> float:
    """How far this weld holds the torch off the work, in millimetres.

    The weld's own `process.wire.stickout_mm` wins. That direction is the
    physical one: the torch's geometry is fixed, but how far it is held off is
    a procedure decision -- and every shipped preset sets it.

    **The fallback is a procedure's number too, and the caller resolves it.**
    This function used to take the torch and read `default_standoff_mm` off it,
    which made the gun a second author of a quantity the procedure owns. Two
    authors and a silent loser: a workspace asking for 25 mm was overridden by
    a preset asking for 20 and nothing said so. The caller now passes the
    default weld preset's stickout, so there is exactly one kind of document
    that states how far the torch is held off the work.

    All three now set it to 20 mm, which is a change from the 12/15/6 they
    carried and is owed to the CELL rather than to the procedures. The gun
    baked into `link_6` is about 30 mm across all the way to the tip -- the
    mesh is the hardware -- and at a 45 degree work angle it needs roughly
    20 mm before it clears a flat plate. The motion side poses at the same
    procedure-owned number now -- each seam's authored `process.standoff_mm`,
    falling back to this stickout -- so both halves of the pipeline answer
    about the same torch. The authoring side once checked clearance for a
    torch held 10-15 mm off, a Ø16 nozzle this cell does not have.

    The useful consequence is that changing stickout in the parameters dialog
    changes the clearance result, which is exactly the real relationship:
    pulling the torch back buys clearance and costs shielding.
    """
    wire = (seam.get("process") or {}).get("wire") or {}
    value = wire.get("stickout_mm")
    if isinstance(value, int | float) and not isinstance(value, bool) and float(value) > 0.0:
        return float(value)
    return float(fallback_mm)


# How finely the optimiser walks an opened stickout band, and how many rungs it
# will put on the ladder before coarsening. Millimetre steps because that is the
# resolution a procedure is written at.
STANDOFF_STEP_MM = 1.0
# Eleven, so a +/-5 mm band at 1 mm steps is walked exactly rather than
# coarsened to 2 mm and losing its own endpoints. Each rung is a millisecond
# DP over a table that is already built, so the count is close to free.
MAX_STANDOFF_CANDIDATES = 11


def standoff_candidates(
    seam: dict[str, Any],
    fallback_mm: float,
    step_mm: float = STANDOFF_STEP_MM,
    max_count: int = MAX_STANDOFF_CANDIDATES,
) -> list[float]:
    """The standoffs an optimiser may consider for this weld. Default: one.

    `process.wire.stickout_limits_mm` is absent by default and absent means
    ZERO freedom, so the list is just what the weld says and the search
    degenerates to a single DP. That default is the whole defence against the
    optimiser's one real hazard: it knows a longer stickout clears a gusset,
    and it does not know that the same change dropped the current, thinned the
    penetration and started losing gas coverage past ~20 mm. Opening the band
    is a procedure decision, so it is made in the procedure.

    The grid is anchored on the AUTHORED value rather than on the band's edge,
    which matters for two reasons: the authored standoff is then always a
    candidate -- `orient.search` requires it, or "never worse than authored"
    would be measured against a weld nobody authored -- and the departures come
    out as round numbers an operator can read.
    """
    authored = weld_standoff_mm(seam, fallback_mm)
    limits = ((seam.get("process") or {}).get("wire") or {}).get("stickout_limits_mm") or {}
    # The DECLARED band is the only band. A settings-wide default used to widen
    # this where the procedure was silent, which made the search free to alter
    # a process parameter on every weld; measured over pipe_bracket's 26 seams
    # it changed coverage on none and moved the stickout on three.
    lo = float(limits.get("min", authored))
    hi = float(limits.get("max", authored))
    if lo > hi:
        raise ValueError(f"stickout_limits_mm crosses: min {lo} > max {hi}")
    # A band that excludes the value it is a band around is a contradiction the
    # operator has to resolve; snapping the authored value into it would change
    # the weld to make the arithmetic work.
    if not lo - 1e-9 <= authored <= hi + 1e-9:
        raise ValueError(
            f"stickout {authored} mm is outside its own limits [{lo}, {hi}]"
        )
    if step_mm <= 0:
        raise ValueError(f"standoff step must be positive, got {step_mm}")

    below = math.floor((authored - lo) / step_mm + 1e-9)
    above = math.floor((hi - authored) / step_mm + 1e-9)
    # Widen the step rather than truncate the band: dropping the far end would
    # silently narrow a limit the procedure declared.
    coarsen = max(1, math.ceil((below + above + 1) / max(1, max_count)))
    step = step_mm * coarsen
    below = math.floor((authored - lo) / step + 1e-9)
    above = math.floor((hi - authored) / step + 1e-9)
    return [round(authored + k * step, 6) for k in range(-below, above + 1)]


def describe(torch: dict[str, Any], standoff_mm: float) -> str:
    """One sentence an operator can act on.

    The standoff is an argument and has no default, because there is no torch
    standoff to fall back to: a fit angle is a statement about a gun AND a gap,
    and one without the other is not a measurement (R-UI-2).
    """
    standoff = float(standoff_mm)
    return (
        f"{torch.get('label', torch.get('id', 'torch'))}: "
        f"Ø{2 * float(torch['body_radius_mm']):.0f} mm × "
        f"{float(torch['body_length_mm']):.0f} mm, at {standoff:.1f} mm standoff "
        f"it cannot enter a groove tighter than {fits_dihedral_deg(torch, standoff):.0f}°"
    )


def self_test() -> int:
    assert not problems(DEFAULT_TORCH), problems(DEFAULT_TORCH)
    assert problems({**DEFAULT_TORCH, "body_radius_mm": 0.0})
    assert problems({**DEFAULT_TORCH, "body_length_mm": 5.0}), "body shorter than standoff"

    # Closed form: radius equal to standoff is exactly a right angle.
    assert abs(fits_dihedral_deg({"body_radius_mm": 10.0}, 10.0) - 90.0) < 1e-9
    assert fits_dihedral_deg(DEFAULT_TORCH, 0.0) == 180.0
    # The three numbers quoted in the docstring.
    assert abs(fits_dihedral_deg(DEFAULT_TORCH, 10.0) - 77.32) < 0.01
    assert abs(fits_dihedral_deg(DEFAULT_TORCH, 16.0) - 53.13) < 0.01
    assert abs(fits_dihedral_deg({"body_radius_mm": 12.5}, 10.0) - 102.68) < 0.01
    # Monotone: pulling back always buys clearance, never costs it.
    angles = [fits_dihedral_deg(DEFAULT_TORCH, d) for d in (6.0, 10.0, 16.0, 40.0)]
    assert angles == sorted(angles, reverse=True), angles

    fine = barrel_offsets(8.0, 65)
    assert len(fine) == 65
    assert all(math.hypot(*p) <= 8.0 + 1e-12 for p in fine)
    assert barrel_offsets(8.0, 65) == fine, "not deterministic"
    on_rim = [p for p in fine if abs(math.hypot(*p) - 8.0) < 1e-9]
    assert len(on_rim) == 32, len(on_rim)
    assert fine[32] == (0.0, 0.0), "the interior half must start on the axis"
    # A 32-point ring behaves like a barrel 0.04 mm narrow, not 1.8 mm narrow.
    assert 8.0 * (1.0 - math.cos(math.pi / 32)) < 0.05

    # A torch states the range it is held at, not the distance it is held at.
    assert not problems({**DEFAULT_TORCH, "standoff_range_mm": {"min": 6.0, "max": 25.0}})
    assert problems({**DEFAULT_TORCH, "standoff_range_mm": {"min": 25.0, "max": 6.0}})
    assert problems({**DEFAULT_TORCH, "standoff_range_mm": None})
    # Subsumes the old body-length check: the furthest the gun is held must
    # still leave barrel behind it.
    assert problems({**DEFAULT_TORCH, "body_length_mm": 20.0}), "barrel shorter than the range"

    # Outside the range is a warning, never a refusal -- D17: the hardware's
    # band and the procedure's claim are different statements and both apply.
    assert standoff_warnings(DEFAULT_TORCH, 10.0) == []
    assert "into the groove" in standoff_warnings(DEFAULT_TORCH, 4.0)[0]
    assert "gas column" in standoff_warnings(DEFAULT_TORCH, 30.0)[0]

    # The weld wins over the fallback, and only a real number counts as said.
    said = {"process": {"wire": {"stickout_mm": 15.0}}}
    assert weld_standoff_mm(said, 10.0) == 15.0
    assert weld_standoff_mm({}, 10.0) == 10.0
    assert weld_standoff_mm({"process": {"wire": {}}}, 10.0) == 10.0

    # No band means no freedom, and that is the default.
    assert standoff_candidates(said, 10.0) == [15.0]
    banded = {"process": {"wire": {"stickout_mm": 12.0,
                                   "stickout_limits_mm": {"min": 10.0, "max": 16.0}}}}
    assert standoff_candidates(banded, 10.0) == [10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0]
    assert 12.0 in standoff_candidates(banded, 10.0), "the authored value must be there"
    # A wide band coarsens rather than truncating: the far end is a limit the
    # procedure declared, and quietly dropping it would narrow it.
    wide = {"process": {"wire": {"stickout_mm": 12.0,
                                 "stickout_limits_mm": {"min": 6.0, "max": 30.0}}}}
    ladder = standoff_candidates(wide, 10.0)
    assert len(ladder) <= MAX_STANDOFF_CANDIDATES and 12.0 in ladder, ladder
    assert ladder[0] >= 6.0 and ladder[-1] <= 30.0 and ladder[-1] >= 27.0, ladder

    print("tooling self-test ok: " + describe(DEFAULT_TORCH, DEFAULT_TORCH["default_standoff_mm"]))
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--standoff-mm", type=float,
                        default=DEFAULT_TORCH["default_standoff_mm"])
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    print(
        json.dumps(
            {
                "schema": SCHEMA,
                "torch": DEFAULT_TORCH,
                "fits_dihedral_deg": fits_dihedral_deg(DEFAULT_TORCH, ns.standoff_mm),
                "summary": describe(DEFAULT_TORCH, ns.standoff_mm),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"tooling: {exc}", file=sys.stderr)
        raise SystemExit(1)
