"""Lossless JSON projection of an emitted cubic Hermite joint curve."""

from __future__ import annotations

from typing import Any

import numpy as np


def hermite_wire_degrees(
    knots_rad: Any,
    knot_velocity_rad_s: Any,
    times_s: Any,
) -> dict[str, list[Any]]:
    """Return contract arrays without presentation rounding.

    JSON's binary64 round trip preserves these values. Rounding belongs at a
    display surface: changing any of ``q``, ``qdot``, or ``t`` changes the
    polynomial that downstream consumers reconstruct and can change a limit
    verdict near its boundary.
    """
    return {
        "knots_deg": np.rad2deg(np.asarray(knots_rad, dtype=np.float64)).tolist(),
        "knot_velocity_deg_s": np.rad2deg(
            np.asarray(knot_velocity_rad_s, dtype=np.float64)
        ).tolist(),
        "times_s": np.asarray(times_s, dtype=np.float64).tolist(),
    }
