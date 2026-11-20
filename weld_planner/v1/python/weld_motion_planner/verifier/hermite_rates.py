"""Exact joint extrema for an emitted cubic Hermite curve.

The emitted motion is the tuple ``(q, q_dot, t)``. Secants between the
positions describe a different curve, so they cannot certify its rate limits.
This module has no planner dependency and evaluates polynomial extrema directly.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np


class HermiteRateError(ValueError):
    """The emitted curve or its declared limits are not judgeable."""


def _interior_roots(a: float, b: float, c: float) -> list[float]:
    """Roots of ``a*u^2 + b*u + c`` strictly inside one segment."""
    if a == 0.0:
        if b == 0.0:
            return []
        root = -c / b
        return [float(root)] if 0.0 < root < 1.0 else []
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0.0:
        return []
    root_gap = np.sqrt(max(0.0, discriminant))
    if root_gap == 0.0:
        roots = [-b / (2.0 * a)]
    else:
        stable = -0.5 * (b + np.copysign(root_gap, b))
        roots = [stable / a, c / stable]
    return sorted({float(root) for root in roots if 0.0 < root < 1.0})


def _finite_array(value: Any, name: str, *, rank: int) -> np.ndarray:
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise HermiteRateError(f"{name} is not a finite numeric array") from exc
    if array.ndim != rank:
        raise HermiteRateError(f"{name} must have rank {rank}, got {array.ndim}")
    if not np.isfinite(array).all():
        raise HermiteRateError(f"{name} contains a nonfinite value")
    return array


def judge_hermite_rates(
    axis_names: Sequence[str],
    knots_rad: Any,
    knot_velocity_rad_s: Any,
    times_s: Any,
    limits: Mapping[str, Mapping[str, Any]],
    *,
    allow_missing_limits: bool = False,
) -> dict[str, Any]:
    """Return exact per-axis extrema and a fail-closed rate verdict.

    Position extrema occur at endpoints or roots of the quadratic velocity.
    Velocity is quadratic in normalized segment time. Its absolute maximum
    occurs at an endpoint or at the interior root of acceleration.
    Acceleration is affine, so its absolute maximum occurs at an endpoint.
    With allow_missing_limits, absent declarations become per-axis refusals;
    stated limits and curve inputs must still be valid. Extrema are always checked.
    """
    names = [str(name) for name in axis_names]
    if not names or len(set(names)) != len(names):
        raise HermiteRateError("axis_names must be nonempty and unique")
    q = _finite_array(knots_rad, "knots_rad", rank=2)
    d = _finite_array(knot_velocity_rad_s, "knot_velocity_rad_s", rank=2)
    t = _finite_array(times_s, "times_s", rank=1)
    if q.shape != d.shape:
        raise HermiteRateError(
            f"knots_rad shape {q.shape} does not match knot_velocity_rad_s {d.shape}"
        )
    if q.shape[0] < 2:
        raise HermiteRateError("the curve needs at least two knots")
    if q.shape[1] != len(names):
        raise HermiteRateError(
            f"curve has {q.shape[1]} axes but axis_names has {len(names)}"
        )
    if t.shape != (q.shape[0],):
        raise HermiteRateError(
            f"times_s shape {t.shape} does not match {q.shape[0]} knots"
        )
    durations = np.diff(t)
    if np.any(durations <= 0.0):
        raise HermiteRateError("times_s must be strictly increasing")

    stated: list[list[float | None]] = []
    unverifiable: list[str] = []
    for name in names:
        spec = limits.get(name, {})
        if not isinstance(spec, Mapping):
            raise HermiteRateError(f"{name}: missing declared rate limits")
        axis_limits: list[float | None] = []
        for kind, key in (("velocity", "velocity_rad_s"),
                          ("acceleration", "acceleration_rad_s2")):
            if spec.get(key) is None and allow_missing_limits:
                unverifiable.append(f"{name}: no {kind} limit")
                axis_limits.append(None)
                continue
            try:
                value = float(spec[key])
            except (KeyError, TypeError, ValueError) as exc:
                raise HermiteRateError(
                    f"{name}: missing declared velocity or acceleration limit"
                ) from exc
            if not np.isfinite(value) or value <= 0.0:
                raise HermiteRateError(f"{name}: rate limits must be finite and positive")
            axis_limits.append(value)
        stated.append(axis_limits)

    per_joint: list[dict[str, Any]] = []
    violations: list[dict[str, Any]] = []
    for joint, name in enumerate(names):
        best_lo = (float("inf"), -1, -1.0)
        best_hi = (-float("inf"), -1, -1.0)
        best_v = (-1.0, -1, -1.0, 0.0)
        best_a = (-1.0, -1, -1.0, 0.0)
        for segment, h in enumerate(durations):
            q0, q1 = q[segment, joint], q[segment + 1, joint]
            d0, d1 = d[segment, joint], d[segment + 1, joint]
            delta = (q0 - q1) / h
            qa = 6.0 * delta + 3.0 * (d0 + d1)
            qb = -6.0 * delta - 4.0 * d0 - 2.0 * d1
            qc = d0

            position_points = [0.0, 1.0, *_interior_roots(qa, qb, qc)]
            for u in position_points:
                if u == 0.0:
                    value = q0
                elif u == 1.0:
                    value = q1
                else:
                    value = q0 + h * (
                        (qa / 3.0) * u**3 + (qb / 2.0) * u**2 + qc * u
                    )
                candidate = (float(value), segment, u)
                if candidate[0] < best_lo[0]:
                    best_lo = candidate
                if candidate[0] > best_hi[0]:
                    best_hi = candidate

            velocity_points = [0.0, 1.0]
            if qa != 0.0:
                vertex = -qb / (2.0 * qa)
                if 0.0 < vertex < 1.0:
                    velocity_points.append(float(vertex))
            for u in velocity_points:
                signed = qa * u * u + qb * u + qc
                candidate = (abs(float(signed)), segment, u, float(signed))
                if candidate[0] > best_v[0]:
                    best_v = candidate

            for u in (0.0, 1.0):
                signed = (2.0 * qa * u + qb) / h
                candidate = (abs(float(signed)), segment, u, float(signed))
                if candidate[0] > best_a[0]:
                    best_a = candidate

        vmax, amax = stated[joint]
        position = {
            "minimum": {
                "value_rad": best_lo[0],
                "segment": best_lo[1],
                "u": best_lo[2],
                "time_s": float(
                    t[best_lo[1]] + best_lo[2] * durations[best_lo[1]]
                ),
            },
            "maximum": {
                "value_rad": best_hi[0],
                "segment": best_hi[1],
                "u": best_hi[2],
                "time_s": float(
                    t[best_hi[1]] + best_hi[2] * durations[best_hi[1]]
                ),
            },
        }
        velocity = {
            "max_abs_rad_s": best_v[0],
            "signed_rad_s": best_v[3],
            "limit_rad_s": vmax,
            "ratio": best_v[0] / vmax if vmax is not None else None,
            "segment": best_v[1],
            "u": best_v[2],
            "time_s": float(t[best_v[1]] + best_v[2] * durations[best_v[1]]),
        }
        acceleration = {
            "max_abs_rad_s2": best_a[0],
            "signed_rad_s2": best_a[3],
            "limit_rad_s2": amax,
            "ratio": best_a[0] / amax if amax is not None else None,
            "segment": best_a[1],
            "u": best_a[2],
            "time_s": float(t[best_a[1]] + best_a[2] * durations[best_a[1]]),
        }
        per_joint.append({
            "axis": name,
            "position": position,
            "velocity": velocity,
            "acceleration": acceleration,
        })
        if vmax is not None and best_v[0] > vmax:
            violations.append({"axis": name, "kind": "velocity", **velocity})
        if amax is not None and best_a[0] > amax:
            violations.append({"axis": name, "kind": "acceleration", **acceleration})

    return {
        "verdict": "FAIL" if violations else "REFUSED" if unverifiable else "PASS",
        "unverifiable": unverifiable,
        "segments": int(q.shape[0] - 1),
        "joint_order": names,
        "per_joint": per_joint,
        "violations": violations,
    }
