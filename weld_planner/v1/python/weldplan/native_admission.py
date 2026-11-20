"""Consume native motion verdicts before creating rows or dense samples.

This gate checks reports from the planner. It does not authenticate a result or
re-certify the different representation produced by an exporter.
"""

from __future__ import annotations

import math
from typing import Any


class MotionAdmissionError(ValueError):
    def __init__(self, reason: str, detail: str):
        self.reason = reason
        self.detail = detail
        super().__init__(f"{reason}: {detail}")


def _entries(result: dict[str, Any], key: str) -> list[dict[str, Any]]:
    entries = result.get(key)
    if entries is None:
        return []
    if not isinstance(entries, list) or any(not isinstance(e, dict) for e in entries):
        raise MotionAdmissionError("motion_result_invalid", f"{key} must be a list of objects")
    return entries


# What a failing report measured, in the order a person asks for it: how far
# off, against what, how much of the motion. A refusal that says only "FAIL"
# cannot be told apart from one that missed by a tenth of a millimetre, and the
# numbers are in the report already; they are repeated here, never recomputed.
_MEASURES = (
    ("worst_seen_mm", "worst seen {} mm"),
    ("tolerance_mm", "tolerance {} mm"),
    ("bound_mm", "certified bound {} mm"),
    ("segments_out", "{} segments out of tolerance"),
    ("undecided", "{} undecided"),
    ("collisions", "{} collisions"),
    ("penetrating", "{} penetrating"),
    ("limit_violations", "{} limit violations"),
)


def _measured(report: dict[str, Any]) -> str:
    """The failing report's own numbers as one clause, or nothing."""
    said = [text.format(report[key]) for key, text in _MEASURES
            if type(report.get(key)) in (int, float) and math.isfinite(report[key])
            and (report[key] != 0 or key in ("worst_seen_mm", "tolerance_mm", "bound_mm"))]
    reasons = report.get("unverifiable")
    if isinstance(reasons, list) and reasons:
        more = f" (+{len(reasons) - 1} more)" if len(reasons) > 1 else ""
        said.append(f"unverifiable: {reasons[0]}{more}")
    return f" ({'; '.join(said)})" if said else ""


def _number(value: Any) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def _at(finding: dict[str, Any]) -> str:
    where = []
    if type(finding.get("segment")) is int and finding["segment"] >= 0:
        where.append(f"segment {finding['segment']}")
    elif type(finding.get("index")) is int and finding["index"] >= 0:
        where.append(f"waypoint {finding['index']}")
    if _number(finding.get("time_s")):
        where.append(f"{finding['time_s']:.6g} s")
    return f" ({', '.join(where)})" if where else ""


def _limit_detail(finding: dict[str, Any]) -> str:
    axis = finding.get("axis")
    label = axis if isinstance(axis, str) and axis else "Joint"
    kind = finding.get("kind")
    if kind == "position":
        value, bounds = finding.get("value_rad"), finding.get("limit_rad")
        if (_number(value) and isinstance(bounds, (list, tuple)) and
                len(bounds) == 2 and all(_number(v) for v in bounds)):
            return (f"{label} position {math.degrees(value):.12g}° outside "
                    f"[{math.degrees(bounds[0]):.12g}°, {math.degrees(bounds[1]):.12g}°]" + _at(finding))
    elif kind in ("velocity", "acceleration"):
        suffix = "rad_s" if kind == "velocity" else "rad_s2"
        value = finding.get("value", finding.get(f"signed_{suffix}"))
        limit = finding.get("limit", finding.get(f"limit_{suffix}"))
        if _number(value) and _number(limit):
            unit = "°/s" if kind == "velocity" else "°/s²"
            return (f"{label} {kind} magnitude {math.degrees(abs(value)):.12g}{unit} "
                    f"exceeds {math.degrees(limit):.12g}{unit}" + _at(finding))
    elif kind == "singular":
        return "The robot cannot move along the weld direction at this pose" + _at(finding)
    elif kind == "travel_speed":
        return "The requested weld travel speed exceeds the available joint speeds at this pose" + _at(finding)
    return ""


def _findings(report: dict[str, Any], trajectory: dict[str, Any] | None) -> str:
    """Render existing finite evidence, never infer depth from a collision sentinel."""
    said = []
    collisions = report.get("collision_details")
    if isinstance(collisions, list):
        for finding in collisions[:3]:
            if not isinstance(finding, dict):
                continue
            pair = finding.get("pair")
            if not isinstance(pair, list) or len(pair) != 2 or not all(isinstance(v, str) for v in pair):
                continue
            if finding.get("reason") == "undecided":
                text = f"Clearance could not be verified between {pair[0]} and {pair[1]}"
            elif finding.get("penetrating") is True:
                # coal/FCL may return -1 for penetration, not a measured depth.
                text = f"Contact or overlap between {pair[0]} and {pair[1]}"
            else:
                text = f"Insufficient clearance between {pair[0]} and {pair[1]}"
                gap, margin = finding.get("worst_clearance_m"), finding.get("margin_m")
                if _number(gap) and gap > 0 and _number(margin) and margin >= 0:
                    text += f": {gap * 1000:.6g} mm; required {margin * 1000:.6g} mm"
            said.append(text + _at(finding))
        count = report.get("collisions")
        if type(count) is int and count > len(collisions[:3]):
            said.append(f"{count - len(collisions[:3])} more collision findings")
    limits = report.get("limit_details")
    if not isinstance(limits, list) and trajectory is not None:
        limits = trajectory.get("limit_violations")
    if isinstance(limits, list):
        for finding in limits[:6]:
            if isinstance(finding, dict):
                text = _limit_detail(finding)
                if text:
                    said.append(text)
        count = report.get("limit_violations")
        if type(count) is int and count > len(limits[:6]):
            said.append(f"{count - len(limits[:6])} more limit findings")
    return "; " + "; ".join(said) if said else ""


def _pass(report: Any, location: str, counts: tuple[str, ...],
          trajectory: dict[str, Any] | None = None) -> dict[str, Any]:
    if not isinstance(report, dict) or report.get("verdict") != "PASS":
        state = report.get("verdict", "missing") if isinstance(report, dict) else "missing/malformed"
        measured = _measured(report) if isinstance(report, dict) else ""
        findings = _findings(report, trajectory) if isinstance(report, dict) else ""
        raise MotionAdmissionError(
            "motion_not_verified", f"{location}: {state}{measured}{findings}; requires native PASS",
        )
    for name in counts:
        if type(report.get(name)) is not int or report[name] != 0:
            raise MotionAdmissionError(
                "motion_not_verified", f"{location}: PASS requires {name}=0",
            )
    if report.get("unverifiable") != []:
        raise MotionAdmissionError(
            "motion_not_verified", f"{location}: unverifiable checks are not empty",
        )
    return report


def _tracking(report: Any, location: str) -> None:
    checked = _pass(report, location, ("segments_out", "undecided"))
    values = [checked.get("bound_mm"), checked.get("tolerance_mm")]
    if any(type(v) not in (float, int) or not math.isfinite(v) for v in values):
        raise MotionAdmissionError("motion_not_verified", f"{location}: missing/invalid bound")
    bound, tolerance = values
    if not 0 <= bound <= tolerance or tolerance <= 0:
        raise MotionAdmissionError(
            "motion_not_verified", f"{location}: bound exceeds the declared tolerance",
        )


def _unplanned(seam: dict[str, Any], location: str) -> str:
    """Keep the discrete search's evidence without claiming a mesh certificate."""
    said = [f"{location}: no complete path was found through the sampled robot poses"]
    if isinstance(seam.get("error"), str) and seam["error"]:
        said.append(seam["error"])
    frontier, fractions = seam.get("frontier"), seam.get("fractions")
    if (type(frontier) is int and frontier >= 0 and isinstance(fractions, list)
            and frontier < len(fractions)):
        where = f"search stopped at sample {frontier + 1} of {len(fractions)}"
        fraction = fractions[frontier]
        if _number(fraction) and 0 <= fraction <= 1:
            where += f" ({fraction * 100:.6g}% along the seam"
            length = seam.get("length_mm")
            if _number(length) and length >= 0:
                where += f", {fraction * length:.6g} mm"
            where += ")"
        said.append(where)
    causes = seam.get("frontier_causes")
    if isinstance(causes, dict):
        for key, text in (
            ("joint limit", "poses outside joint travel limits"),
            ("collision", "poses rejected by sphere collision screening"),
            ("no ik", "poses unable to reach the requested tool position and orientation"),
            ("no edge", "poses with no allowed transition from the previous sample"),
            ("duplicate", "duplicate poses"),
        ):
            count = causes.get(key)
            if type(count) is int and count > 0:
                said.append(f"{count} {text}")
        if type(causes.get("collision")) is int and causes["collision"] > 0:
            said.append("these are sampled-pose screening results, not a continuous mesh collision certificate")
    return "; ".join(said)


def require_native_motion(result: dict[str, Any], *, candidate_only: bool = False) -> None:
    """Require consistent PASS reports for every supplied continuous seam/move.

    The explicit candidate-only mode is for M4 authoring rows, saved unplanned.
    It refuses continuous output even if its verdict passes, so it cannot be
    used to bypass a failed M5/M6 report. Strict admission is the default.
    """
    if result.get("connecting_trajectories_error"):
        raise MotionAdmissionError(
            "motion_join_failed", str(result["connecting_trajectories_error"]),
        )
    seams = _entries(result, "seams")
    moves = _entries(result, "connecting_trajectories")
    if candidate_only:
        if moves or any(s.get("trajectory") is not None for s in seams):
            raise MotionAdmissionError(
                "candidate_only_requires_m4", "continuous motion requires native verification",
            )
        return

    seen: set[str] = set()
    for seam in seams:
        name = seam.get("id")
        if not isinstance(name, str) or not name or name in seen:
            raise MotionAdmissionError("motion_result_invalid", "seam ids must be nonempty and unique")
        seen.add(name)
        location = f"seam {name}"
        if seam.get("crossed") is not True or seam.get("error"):
            raise MotionAdmissionError("seam_not_planned", _unplanned(seam, location))
        trajectory = seam.get("trajectory")
        if not isinstance(trajectory, dict):
            raise MotionAdmissionError(
                "motion_not_verified", f"{location}: missing continuous trajectory; generate and verify motion",
            )
        _pass(trajectory.get("verdict"), location, ("collisions", "penetrating", "limit_violations"), trajectory)
        _tracking(trajectory.get("tracking"), f"{location} tracking")
        for key in ("limit_violations", "limit_unverifiable"):
            if key in trajectory and trajectory[key] != []:
                raise MotionAdmissionError("motion_not_verified", f"{location}: nonempty {key}")

    for i, move in enumerate(moves):
        kind = move.get("kind")
        location = f"{kind} move {i} ({move.get('from')} -> {move.get('to')})"
        if kind not in ("approach", "transit", "retract"):
            raise MotionAdmissionError("motion_result_invalid", f"{location}: unknown move kind")
        _pass(move.get("verdict"), location, ("collisions", "penetrating", "limit_violations"))
