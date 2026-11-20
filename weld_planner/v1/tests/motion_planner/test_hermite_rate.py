"""The speed a Hermite segment actually reaches, between its knots.

`_clip` bounds the knot-to-knot step -- the chord -- and what ships is the
cubic through those knots. Between two knots it bulges past its own chord, so
a trajectory can satisfy the clamp at every knot and still command a joint
faster than its limit. On `rosie_1420_v1` seam_0008 the chords sat at 0.936 of
J1's limit while one segment of 131 peaked at 1.086, and T2 refused it; running
`_clip` more times changed nothing, because the thing it bounds was never the
thing that was over.
"""

from __future__ import annotations

import pytest
import torch

from weld_motion_planner.planner.trajectory_optimization.engine.weld_trajopt import (
    hermite_peak,
)


def dense_peak(m0: torch.Tensor, m1: torch.Tensor, chord: torch.Tensor,
               samples: int = 4001) -> torch.Tensor:
    """The same peak by brute force: the derivative sampled across the span."""
    u = torch.linspace(0.0, 1.0, samples, dtype=m0.dtype).reshape(-1, *([1] * m0.dim()))
    a = 3.0 * (m0 + m1) - 6.0 * chord
    b = -4.0 * m0 - 2.0 * m1 + 6.0 * chord
    return (a * u * u + b * u + m0).abs().amax(dim=0)


def test_the_closed_form_agrees_with_sampling_the_derivative():
    generator = torch.Generator().manual_seed(11)
    shape = (64, 6)
    m0 = torch.rand(shape, generator=generator, dtype=torch.float64) * 2 - 1
    m1 = torch.rand(shape, generator=generator, dtype=torch.float64) * 2 - 1
    chord = torch.rand(shape, generator=generator, dtype=torch.float64) * 2 - 1
    assert torch.allclose(hermite_peak(m0, m1, chord),
                          dense_peak(m0, m1, chord), atol=1e-6)


def test_a_straight_segment_never_exceeds_its_chord():
    """Both knot velocities at the chord makes the derivative constant. This is
    what the rate fit converges to, and why a fit always exists: `_clip` has
    already put the chord inside the limit."""
    chord = torch.tensor([[0.05, -0.09, 0.2]], dtype=torch.float64)
    assert torch.allclose(hermite_peak(chord, chord, chord), chord.abs())


def test_knot_velocities_inside_the_limit_do_not_bound_the_curve():
    """The property that makes this worth checking at all: a segment whose two
    endpoints AND whose chord are inside the limit can still leave it in
    between, so clamping the knots is not enough."""
    limit = 0.09
    # Both knots slow and the chord fast: the mean of the derivative over the
    # span IS the chord, so the middle has to make up what the ends give away.
    m0 = m1 = torch.tensor([[0.2 * limit]], dtype=torch.float64)
    chord = torch.tensor([[0.75 * limit]], dtype=torch.float64)
    peak = hermite_peak(m0, m1, chord)
    # 1.5 * chord - 0.5 * m at the midpoint, for equal endpoints.
    assert float(peak) == pytest.approx(1.025 * limit)
    assert float(peak) > limit
    assert torch.allclose(peak, dense_peak(m0, m1, chord), atol=1e-9)


def test_the_vertex_only_counts_when_it_falls_inside_the_segment():
    """A derivative whose turning point is off the span is monotone on it, so
    the peak is an endpoint. Taking the vertex regardless would report a speed
    the segment never reaches and slow a trajectory for nothing."""
    m0 = torch.tensor([[0.0]], dtype=torch.float64)
    m1 = torch.tensor([[1.0]], dtype=torch.float64)
    chord = torch.tensor([[0.5]], dtype=torch.float64)   # the linear ramp
    assert torch.allclose(hermite_peak(m0, m1, chord), m1.abs())
    assert torch.allclose(hermite_peak(m0, m1, chord),
                          dense_peak(m0, m1, chord), atol=1e-9)
