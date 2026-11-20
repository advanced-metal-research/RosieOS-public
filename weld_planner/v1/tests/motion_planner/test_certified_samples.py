"""Certifying a free-space motion on its cubic pieces instead of its wire.

The controller replays the Hermite through consecutive 10 ms wire samples.
The verifier used to certify exactly that, one segment per 10 ms, and it was
most of the free-space stage (7.7 s of 9.2 s on an 8-weld request). It now
certifies the Hermites between a SUBSET of the samples -- the first and last
of each cubic piece of the B-spline -- and that is only sound if those
Hermites are the same curve the controller runs. That is the claim here,
measured in radians at instants neither sample set contains.
"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest
import torch

from weld_motion_planner.planner.trajectory_optimization.engine.control_solver import (
    CONTROL_POINTS,
    _AsSolved,
)
from weld_motion_planner.planner.trajectory_optimization.engine.freespace import (
    certified_samples,
)
from weld_motion_planner.verifier.verify import _hermite

DOF = 6


def _wire(duration_s: float, seed: int) -> _AsSolved:
    """The wire `control_solver` emits, for a random control polygon."""
    control = np.random.default_rng(seed).normal(size=(CONTROL_POINTS, DOF))
    answer = SimpleNamespace(
        duration_s=duration_s, control=torch.as_tensor(control),
        knots=np.stack([control[0], control[-1]]), pin_start=False, pin_end=False,
        clearance_m=0.0, env_clearance_m=0.0, self_clearance_m=0.0,
        cost_before=0.0, cost_after=0.0)
    return _AsSolved(answer)


def _curve(times: np.ndarray, q: np.ndarray, qd: np.ndarray, at: np.ndarray) -> np.ndarray:
    """The Hermite through `(times, q, qd)`, read at `at`."""
    out = np.zeros((len(at), q.shape[1]))
    for n, t in enumerate(at):
        k = int(np.clip(np.searchsorted(times, t, side="right") - 1, 0, len(times) - 2))
        h = float(times[k + 1] - times[k])
        out[n] = _hermite(q[k], q[k + 1], qd[k], qd[k + 1], h, (t - times[k]) / h)
    return out


# 2.6 s is a multiple of both the 10 ms clock and the 13 pieces, so every
# piece boundary IS a sample; 0.083 s is shorter than a piece is wide in wire
# intervals (8 intervals over 13 pieces); the rest land between.
@pytest.mark.parametrize("duration_s", [2.6, 1.337, 4.019, 0.083])
def test_the_kept_samples_trace_the_curve_the_controller_runs(duration_s):
    wire = _wire(duration_s, seed=int(duration_s * 1000))
    keep = certified_samples(wire.times_s, wire.span_times_s)
    assert keep[0] == 0 and keep[-1] == len(wire.times_s) - 1
    assert np.all(np.diff(keep) > 0)
    # Instants that are neither wire samples nor piece boundaries.
    at = np.sort(np.random.default_rng(7).uniform(0.0, duration_s, 4001))
    replayed = _curve(wire.times_s, wire.knots, wire.qdot, at)
    certified = _curve(wire.times_s[keep], wire.knots[keep], wire.qdot[keep], at)
    assert np.abs(replayed - certified).max() < 1e-9, np.abs(replayed - certified).max()


def test_the_kept_samples_are_a_few_per_piece_not_one_per_tick():
    """The point of the change, in counts: at most two samples per piece
    plus the ends, whatever the motion's length."""
    for duration_s in (0.8, 3.0, 12.0):
        wire = _wire(duration_s, seed=3)
        keep = certified_samples(wire.times_s, wire.span_times_s)
        pieces = len(wire.span_times_s) - 1
        assert len(keep) <= 2 * pieces + 2, (duration_s, len(keep))
    assert len(wire.times_s) > 1000


def test_a_solver_that_states_no_pieces_keeps_every_sample():
    times = np.linspace(0.0, 1.0, 51)
    assert np.array_equal(certified_samples(times, None), np.arange(51))


def test_dropping_a_boundary_sample_is_caught():
    """A guard on the guard: the curve comparison above is only worth its
    name if it fails when a piece boundary is skipped over."""
    wire = _wire(4.019, seed=11)
    keep = list(certified_samples(wire.times_s, wire.span_times_s))
    del keep[len(keep) // 2]
    at = np.linspace(0.0, 4.019, 4001)
    replayed = _curve(wire.times_s, wire.knots, wire.qdot, at)
    certified = _curve(wire.times_s[keep], wire.knots[keep], wire.qdot[keep], at)
    assert np.abs(replayed - certified).max() > 1e-6
