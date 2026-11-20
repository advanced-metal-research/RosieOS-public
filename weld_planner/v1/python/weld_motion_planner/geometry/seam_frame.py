#!/usr/bin/env python
"""The seam frame, the two angles, and where a seam sits on its joint.

`SEAM-MODEL.md` §5 is the whole specification and this is its second
implementation. The first is `seam_worker.seam.frames`; the TypeScript viewer is
a third. Three implementations of one convention is not duplication for its
own sake -- the authoring side needs it to draw and to search, the client
needs it to move a slider without a round trip, and the planner needs it
INSIDE the autodiff graph, because the achieved electrode direction is what
the band constraint is evaluated on and the optimizer differentiates that.

What holds them together is numbers, not discipline:
`data/motion/oracles/seams.*.json` for this one.

## The three coordinate stories, in the order they compose

1. **fraction -> s.** A seam is a SPAN of its joint's weld line, so
   `fraction` in [0,1] along the seam maps to `s` along the joint, and on a
   closed loop the span may wrap through zero. Getting this wrong puts every
   pose on the right curve at the wrong place.
2. **s -> {P, t, r, b}.** The rail supplies the work-angle datum: `r` is the
   direction from seam to rail, orthogonalized against the tangent. When
   seam and rail have the same segment count the correspondence is PER
   SEGMENT, which is what lets a bisector rail step discontinuously at a
   corner instead of smearing across it.
3. **{t, r} + angles -> u.** Work first, then travel. `u` points from the
   weld toward the torch body; the electrode axis is `-u`.

## Direction

`direction: "reverse"` flips the travel sense, which flips `t` and therefore
`b = t x r`. It is applied here rather than by reversing the curve, because
the curve is shared with the joint and other seams may traverse it the other
way.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from typing import Any

import torch

from weld_motion_planner.geometry.curves import Curve, CurveError

# Beyond this |cos| between the reference direction and the tangent, the
# orthogonalization has nothing left to work with. `SEAM-MODEL.md` §5 calls
# this the one detectable failure of the construction.
DEGENERATE_DOT = 0.999_999


def rotation(axis: torch.Tensor, angle: torch.Tensor) -> torch.Tensor:
    """Right-handed Rodrigues rotation, batched: (..., 3) and (...) -> (..., 3, 3)."""
    norm = torch.linalg.norm(axis, dim=-1, keepdim=True).clamp_min(1e-30)
    a = axis / norm
    x, y, z = a[..., 0], a[..., 1], a[..., 2]
    zero = torch.zeros_like(x)
    k = torch.stack(
        [
            torch.stack([zero, -z, y], dim=-1),
            torch.stack([z, zero, -x], dim=-1),
            torch.stack([-y, x, zero], dim=-1),
        ],
        dim=-2,
    )
    eye = torch.eye(3, dtype=axis.dtype, device=axis.device).expand(k.shape)
    s = torch.sin(angle)[..., None, None]
    c = torch.cos(angle)[..., None, None]
    return eye + s * k + (1.0 - c) * (k @ k)


def apply_angles(
    t: torch.Tensor, r: torch.Tensor, work_deg: torch.Tensor, travel_deg: torch.Tensor
) -> torch.Tensor:
    """The torch reference direction `u`, batched.

        r' = R(t,  work) r          roll within the plane perpendicular to travel
        b' = t x r'
        u  = R(b', -travel) r'      tilt along travel

    Work first, then travel, so travel angle means "tilt along travel"
    whatever work angle is already applied. Positive travel is DRAG: the
    torch leans back over completed weld.
    """
    work = torch.deg2rad(work_deg)
    travel = torch.deg2rad(travel_deg)
    r_prime = (rotation(t, work) @ r.unsqueeze(-1)).squeeze(-1)
    b_prime = torch.cross(t, r_prime, dim=-1)
    return (rotation(b_prime, -travel) @ r_prime.unsqueeze(-1)).squeeze(-1)


def transverse_axis(t: torch.Tensor, u: torch.Tensor) -> torch.Tensor:
    """`b'`, the seam frame's transverse axis.

    Perpendicular to both travel and the electrode, exactly, for any work and
    travel angle -- the definition is `WEAVE-AND-VERIFICATION.md` §2's, and
    it is the axis `μ_b'` measures mobility along whether or not anything
    oscillates along it. Recomputed from `t`
    and `u` rather than carried out of `apply_angles`, because the caller
    that needs it -- the transverse-mobility floor -- generally has the
    achieved `u` and not the angles that produced it.
    """
    b = torch.cross(t, u, dim=-1)
    return b / torch.linalg.norm(b, dim=-1, keepdim=True).clamp_min(1e-30)


def s_at_fraction(span: dict[str, float], fraction: torch.Tensor) -> torch.Tensor:
    """Joint `s` at `fraction` along a span, wrap included.

    The ends are returned EXACTLY rather than computed: `start + 1 * length`
    lands an ulp past `end_s`, and a seam's end is the next span's start, so
    that ulp reads as being inside the neighbour. The authoring side hit this
    on pipe_bracket -- 1 position in 1001 -- and the same arithmetic here
    would hit it in the same place.
    """
    start = float(span.get("start_s", 0.0))
    end = float(span.get("end_s", 1.0))
    length = (1.0 - start + end) if end < start else (end - start)
    f = torch.clamp(fraction, 0.0, 1.0)
    s = start + f * length
    s = torch.where(s > 1.0, s - 1.0, s)
    s = torch.where(f <= 0.0, torch.full_like(s, start), s)
    return torch.where(f >= 1.0, torch.full_like(s, end), s)


def span_length_m(span: dict[str, float], total_m: float) -> float:
    start = float(span.get("start_s", 0.0))
    end = float(span.get("end_s", 1.0))
    return ((1.0 - start + end) if end < start else (end - start)) * total_m


class Seam:
    """One seam of a weld program, evaluable at any fraction along itself.

    Built from the program's own documents rather than from a preprocessed
    form, so that what the planner samples is what the contract said.
    """

    def __init__(
        self,
        seam: dict[str, Any],
        joint: dict[str, Any] | None = None,
        *,
        dtype=torch.float64,
        device=None,
    ):
        spec = (seam.get("curve") or {}).get("segments") and seam["curve"]
        if not spec:
            spec = (joint or {}).get("weld_line")
        if not spec:
            raise CurveError(
                f"seam {seam.get('id')!r} has no curve of its own and no weld joint to take one from"
            )
        self.id = str(seam.get("id") or "")
        self.dtype, self.device = dtype, device
        self.curve = Curve(spec, dtype=dtype, device=device)
        self.span = dict(seam.get("span") or {"start_s": 0.0, "end_s": 1.0})
        self.reversed = str(seam.get("direction") or "forward") == "reverse"
        self.orientation = seam.get("orientation") or {}
        self.process = seam.get("process") or {}

        reference = seam.get("reference") or {}
        self.mode = str(reference.get("mode") or "rail_curve")
        rail_spec = reference.get("rail")
        self.rail = (
            Curve(rail_spec, dtype=dtype, device=device)
            if self.mode == "rail_curve" and rail_spec
            else None
        )
        self.fixed_direction = (
            reference.get("fixed_direction_xyz") or reference.get("seed_direction_xyz") or None
        )

    @property
    def length_m(self) -> float:
        return span_length_m(self.span, self.curve.total)

    def s_at(self, fraction: Any) -> torch.Tensor:
        f = torch.as_tensor(fraction, dtype=self.dtype, device=self.device)
        return s_at_fraction(self.span, f)

    def _rail_direction(
        self, s: torch.Tensor, point: torch.Tensor, left_limit: torch.Tensor
    ) -> torch.Tensor:
        """`Q(s) - P(s)`: the raw work-angle datum, before orthogonalization."""
        if self.rail is None:
            if self.fixed_direction is not None:
                fixed = torch.as_tensor(
                    self.fixed_direction, dtype=self.dtype, device=self.device
                )
                return fixed.expand_as(point)
            up = torch.zeros_like(point)
            up[..., 2] = 1.0
            return up
        if len(self.rail.segments) == len(self.curve.segments):
            # PER SEGMENT. A bisector rail genuinely steps at a corner, and
            # matching over the whole curve would smear that discontinuity
            # across the corner instead of respecting it. The same limit the
            # tangent used, or the rail would step a segment early at a join.
            index, within = self.curve.s_within_segment(s, left_limit)
            return self.rail.point_at_segment_fraction(index, within) - point
        return self.rail.point(s) - point

    def reference_direction(self, fraction: Any) -> torch.Tensor:
        """`Q(s) - P(s)`: the work-angle datum BEFORE orthogonalization.

        Public because the tracking certificate has to bound how fast `r`
        turns, and `r` is a normalized projection of this vector -- so the
        rate depends on the raw vector's LENGTH as much as on its direction,
        and the unit `r` alone cannot supply it. Everything else about the
        frame is derivable from what `frames` returns; this is the one input
        that is consumed and thrown away.
        """
        f = torch.as_tensor(fraction, dtype=self.dtype, device=self.device)
        s = self.s_at(f)
        left_limit = f >= 1.0
        point, _, _ = self.curve.evaluate(s, left_limit)
        return self._rail_direction(s, point, left_limit)

    def frames(self, fraction: Any) -> dict[str, torch.Tensor]:
        """`{s, P, t, r, b, curvature}` at each fraction along the seam."""
        f = torch.as_tensor(fraction, dtype=self.dtype, device=self.device)
        s = self.s_at(f)
        # At the very end of the span, take the ARRIVING tangent. A seam that
        # ends on a corner -- which is where seams are split, so a common
        # case rather than a corner case -- otherwise reports the direction
        # the next segment would have departed along, and the two differ by
        # the whole corner angle. See `Curve.locate`.
        left_limit = f >= 1.0
        point, d1, _ = self.curve.evaluate(s, left_limit)
        speed = torch.linalg.norm(d1, dim=-1, keepdim=True)
        fallback = torch.zeros_like(d1)
        fallback[..., 0] = 1.0
        t = torch.where(speed > 1e-30, d1 / speed.clamp_min(1e-30), fallback)
        if self.reversed:
            t = -t

        direction = self._rail_direction(s, point, left_limit)
        along = (direction * t).sum(dim=-1, keepdim=True)
        r = direction - along * t
        norm = torch.linalg.norm(r, dim=-1, keepdim=True)
        degenerate = (
            norm.squeeze(-1) < 1e-12
        ) | (
            along.squeeze(-1).abs()
            / torch.linalg.norm(direction, dim=-1).clamp_min(1e-30)
            > DEGENERATE_DOT
        )
        up = torch.zeros_like(r)
        up[..., 2] = 1.0
        rescue = up - t[..., 2:3] * t
        r = torch.where(degenerate.unsqueeze(-1), rescue, r)
        r = r / torch.linalg.norm(r, dim=-1, keepdim=True).clamp_min(1e-30)

        return {
            "s": s,
            "pos_m": point,
            "t": t,
            "r": r,
            "b": torch.cross(t, r, dim=-1),
            # The same limit as the tangent: curvature steps at a corner too,
            # and reading it off the far segment would describe an arc the
            # torch has not reached yet.
            "curvature_1_m": self.curve.curvature(s, left_limit),
            "fraction": f,
            "degenerate": degenerate,
        }


def self_test() -> int:
    """The sign conventions, worked by hand from `SEAM-MODEL.md` §5."""
    dtype = torch.float64

    # Positive travel is DRAG. With t=+x and r=+z, u must lean toward +t, so
    # the torch body sits AHEAD of the weld and the electrode points back
    # over completed weld.
    t = torch.tensor([[1.0, 0.0, 0.0]], dtype=dtype)
    r = torch.tensor([[0.0, 0.0, 1.0]], dtype=dtype)
    u = apply_angles(t, r, torch.tensor([0.0], dtype=dtype), torch.tensor([15.0], dtype=dtype))
    expected = torch.tensor([[math.sin(math.radians(15.0)), 0.0, math.cos(math.radians(15.0))]], dtype=dtype)
    assert torch.allclose(u, expected, atol=1e-12), f"travel sign is wrong: {u}"

    # Positive work rotates r toward +b = t x r.
    work = apply_angles(t, r, torch.tensor([20.0], dtype=dtype), torch.tensor([0.0], dtype=dtype))
    b = torch.cross(t, r, dim=-1)
    assert float((work * b).sum()) > 0.0, f"work angle turned the wrong way: {work}"
    assert abs(float((work * t).sum())) < 1e-12, "work angle should stay in the cross-section plane"

    # Zero angles leave the torch on the rail direction exactly.
    plain = apply_angles(t, r, torch.tensor([0.0], dtype=dtype), torch.tensor([0.0], dtype=dtype))
    assert torch.allclose(plain, r, atol=1e-14)

    # b' is perpendicular to travel and electrode for ANY pair of angles,
    # which is what makes it a well-defined transverse direction to measure
    # mobility along at every point of every seam.
    for w in (-60.0, -12.0, 0.0, 33.0, 60.0):
        for tr in (-30.0, 0.0, 10.0, 30.0):
            v = apply_angles(t, r, torch.tensor([w], dtype=dtype), torch.tensor([tr], dtype=dtype))
            bp = transverse_axis(t, v)
            assert abs(float((bp * t).sum())) < 1e-12, (w, tr)
            assert abs(float((bp * v).sum())) < 1e-12, (w, tr)

    # A span that wraps through zero, which is what a closed loop produces.
    span = {"start_s": 0.9, "end_s": 0.1}
    f = torch.tensor([0.0, 0.25, 0.5, 0.75, 1.0], dtype=dtype)
    s = s_at_fraction(span, f)
    # Midway lands on 1.0 rather than 0.0: the wrap is `s > 1.0`, strictly,
    # which is what the authoring side does, and on a closed curve 1.0 and
    # 0.0 are the same point. Matching its behaviour matters more than
    # preferring one spelling of that point.
    assert torch.allclose(s, torch.tensor([0.9, 0.95, 1.0, 0.05, 0.1], dtype=dtype), atol=1e-12), s
    assert float(s[0]) == 0.9 and float(s[-1]) == 0.1, "the ends must be exact, not computed"

    # A straight seam with a rail beside it: r points at the rail, b is
    # right-handed, and reversing direction flips both t and b but not r.
    seam_spec = {
        "id": "s",
        "curve": {
            "segments": [
                {"degree": 1, "knots_u": [0, 0, 1, 1], "control_points_xyz_m": [[0, 0, 0], [1, 0, 0]]}
            ]
        },
        "reference": {
            "mode": "rail_curve",
            "rail": {
                "segments": [
                    {
                        "degree": 1,
                        "knots_u": [0, 0, 1, 1],
                        "control_points_xyz_m": [[0, 0, 0.1], [1, 0, 0.1]],
                    }
                ]
            },
        },
    }
    seam = Seam(seam_spec)
    got = seam.frames(torch.tensor([0.0, 0.5, 1.0], dtype=dtype))
    assert torch.allclose(got["t"], torch.tensor([1.0, 0.0, 0.0], dtype=dtype).expand(3, 3), atol=1e-12)
    assert torch.allclose(got["r"], torch.tensor([0.0, 0.0, 1.0], dtype=dtype).expand(3, 3), atol=1e-12)
    # `b = t x r = x_hat x z_hat = -y_hat`, which is the same cross product
    # SEAM-MODEL.md §5 works by hand in its own sanity check.
    assert torch.allclose(
        got["b"], torch.tensor([0.0, -1.0, 0.0], dtype=dtype).expand(3, 3), atol=1e-12
    ), got["b"]
    assert not bool(got["degenerate"].any())

    backwards = Seam({**seam_spec, "direction": "reverse"}).frames(torch.tensor([0.5], dtype=dtype))
    assert torch.allclose(backwards["t"], -got["t"][1:2], atol=1e-12)
    assert torch.allclose(backwards["r"], got["r"][1:2], atol=1e-12), "reversing must not move the rail"
    assert torch.allclose(backwards["b"], -got["b"][1:2], atol=1e-12)

    # A rail directly along the tangent has no cross-section component, and
    # that is reported rather than silently producing a frame.
    degenerate = Seam(
        {**seam_spec, "reference": {"mode": "fixed", "fixed_direction_xyz": [1.0, 0.0, 0.0]}}
    ).frames(torch.tensor([0.5], dtype=dtype))
    assert bool(degenerate["degenerate"].all()), "a reference parallel to travel should be caught"

    print(
        "seam frame self-test ok: drag is positive travel, work turns toward t x r, b' stays "
        "perpendicular over the whole angle range, a wrapped span maps exactly, and a "
        "reference parallel to travel is reported"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--read", default="", help="a .weldplan whose seams to sample")
    parser.add_argument("--samples", type=int, default=5)
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not ns.read:
        raise CurveError("--read or --self-test")

    from weld_motion_planner.io.plan_request import read

    request = read(ns.read)
    joints = request.joints_by_id
    fractions = torch.linspace(0.0, 1.0, max(2, ns.samples), dtype=torch.float64)
    out = []
    for spec in request.seams:
        seam = Seam(spec, joints.get(spec.get("weld_joint_ref")))
        got = seam.frames(fractions)
        out.append(
            {
                "seam_id": seam.id,
                "length_mm": round(seam.length_m * 1000.0, 4),
                "reversed": seam.reversed,
                "degenerate_samples": int(got["degenerate"].sum()),
                "first_point_m": [round(v, 6) for v in got["pos_m"][0].tolist()],
                "first_t": [round(v, 6) for v in got["t"][0].tolist()],
                "first_r": [round(v, 6) for v in got["r"][0].tolist()],
            }
        )
    print(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"frames: {exc}", file=sys.stderr)
        raise SystemExit(1)
