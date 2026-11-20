"""The cuRobo backend must leave the process the way it found it.

`docs` for the incident: cuRobo's `seed_ik_solver` sets
`torch.backends.*.allow_tf32 = True` at IMPORT TIME, process-wide. TF32
truncates float32 matmuls to a 10-bit mantissa, and this planner's seam-search
IK stops converging at that precision. Measured on a live server: one cuRobo
request flipped a seam that crossed cleanly to `no ik` on 17,086 of 17,995
lattice nodes -- for every later request, from any client, for the life of the
process. Two servers with byte-identical code disagreed on the same request
for a day because one of them had once planned through cuRobo.

These tests pin the guard, not cuRobo's behaviour: whatever cuRobo sets,
leaving the backend must restore it.
"""

from __future__ import annotations

import pytest
import torch

from weld_motion_planner.planner.trajectory_optimization.engine import (
    curobo_backend,
)


def test_the_guard_restores_what_its_body_mutates():
    import random

    import numpy as np

    before = (torch.backends.cuda.matmul.allow_tf32,
              torch.backends.cudnn.allow_tf32,
              torch.backends.cudnn.benchmark)
    try:
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False
        torch.backends.cudnn.benchmark = False
        np.random.seed(31337)
        random.seed(31337)
        np_mark = np.random.get_state()[1][:4].tolist()
        py_mark = random.getstate()[1][:4]
        with curobo_backend._torch_global_state():
            # Everything cuRobo does, done deliberately: the import-time
            # backend flags AND the graph planner's global reseed.
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
            torch.backends.cudnn.benchmark = True
            np.random.seed(123)
            random.seed(123)
        assert torch.backends.cuda.matmul.allow_tf32 is False
        assert torch.backends.cudnn.allow_tf32 is False
        assert torch.backends.cudnn.benchmark is False
        assert np.random.get_state()[1][:4].tolist() == np_mark
        assert random.getstate()[1][:4] == py_mark
    finally:
        (torch.backends.cuda.matmul.allow_tf32,
         torch.backends.cudnn.allow_tf32,
         torch.backends.cudnn.benchmark) = before


def test_importing_curobo_does_not_change_this_process():
    """The real import, if cuRobo is present. This is the exact write that
    poisoned the server: module-level, on first import, permanent."""
    before_matmul = torch.backends.cuda.matmul.allow_tf32
    before_cudnn = torch.backends.cudnn.allow_tf32
    before_benchmark = torch.backends.cudnn.benchmark
    try:
        curobo_backend._require_curobo()
    except curobo_backend.CuroboUnavailable:
        pytest.skip("cuRobo is not installed in this environment")
    assert torch.backends.cuda.matmul.allow_tf32 == before_matmul
    assert torch.backends.cudnn.allow_tf32 == before_cudnn
    # `cudnn.benchmark = True` sits beside the TF32 writes in the same module
    # and leaked through the first version of the guard.
    assert torch.backends.cudnn.benchmark == before_benchmark
