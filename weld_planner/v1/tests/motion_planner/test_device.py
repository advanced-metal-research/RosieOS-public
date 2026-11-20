"""Which device the planner runs on, and the fp32 claim that rests on it.

`ARCHITECTURE.md` puts the search stages in fp32 on the GPU because a
consumer card runs fp64 at 1/64 rate. That is only sound if fp32 lands close
enough to pick the right basin, and if the GPU can actually run a kernel --
a torch built for an older CUDA imports fine on a newer card and fails at
the first matmul.
"""

from __future__ import annotations

import torch
from weld_motion_planner.geometry.curves import Curve
from weld_motion_planner.utils.device import EXACT_DTYPE, SEARCH_DTYPE, device, probe

LINE = {
    "segments": [
        {"degree": 1, "knots_u": [0, 0, 1, 1], "control_points_xyz_m": [[0, 0, 0], [3, 4, 0]]}
    ]
}


def test_the_device_choice_comes_with_a_reason():
    """A fallback nobody can explain is a fallback nobody notices. If the
    card is unusable this says why, in the same breath as saying so."""
    found = probe()
    assert found["device"] in {"cpu", "cuda"}
    assert found["reason"]
    assert found["torch"]


def test_a_gpu_is_only_claimed_if_a_kernel_ran_on_it():
    """`torch.cuda.is_available()` is not the question -- the 5090 is sm_120
    and any torch built for CUDA 12.6 or earlier reports it and then has no
    code for it."""
    found = probe()
    if found["device"] == "cuda":
        assert found["usable"] is True
        assert "kernel ran" in found["reason"]
    else:
        # Either there is no card, or there is one and it could not run
        # anything -- and the second case must name the capability it needed.
        assert "no CUDA device" in found["reason"] or "no kernels for" in found["reason"]


def test_the_curve_core_runs_on_whatever_was_chosen():
    curve = Curve(LINE, dtype=SEARCH_DTYPE, device=device())
    s = torch.linspace(0.0, 1.0, 9, dtype=SEARCH_DTYPE, device=device())
    points = curve.point(s)
    assert points.device.type == device().type
    assert points.shape == (9, 3)


def test_float32_lands_within_search_accuracy_of_float64():
    """Micrometres on a decimetre. The tracking constraint is millimetres and
    the fp64 polish and the verifier enforce it exactly, so this is the
    accuracy the basin choice needs and no more."""
    s32 = torch.linspace(0.0, 1.0, 33, dtype=SEARCH_DTYPE, device=device())
    cheap = Curve(LINE, dtype=SEARCH_DTYPE, device=device()).point(s32)
    exact = Curve(LINE, dtype=EXACT_DTYPE).point(
        torch.linspace(0.0, 1.0, 33, dtype=EXACT_DTYPE)
    )
    assert float((cheap.cpu().double() - exact).abs().max()) < 1e-5


def test_self_test_entry_point_passes():
    from weld_motion_planner.utils.device import self_test

    assert self_test() == 0
