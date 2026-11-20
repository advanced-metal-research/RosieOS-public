#!/usr/bin/env python
"""Which device the planner runs on, and whether it actually works.

`torch.cuda.is_available()` is not the question. A torch built for an older
CUDA imports cleanly on a newer card, reports the device by name, and then
fails at the first real kernel with "no kernel image is available for
execution on the device" -- which surfaces halfway through a bake, reads as
a broken install, and sends whoever hits it looking in the wrong place. The
5090 is `sm_120` and exactly this trap: any torch built against CUDA 12.6 or
earlier has no code for it.

So the check here is a KERNEL, not a capability query. One small matmul,
once, and the answer is cached.

The fallback is CPU rather than an exception, because everything in this
package is correct on CPU and merely slow. A machine with no card should be
able to run the tests, and `ARCHITECTURE.md`'s budget is a target for the
dev workstation, not a precondition for the code being right.
"""

from __future__ import annotations

import argparse
import json
import sys
from functools import lru_cache
from typing import Any

import torch

# fp32 for search work on the GPU: a consumer card runs fp64 at 1/64 rate,
# and the graph and the refinement only need to land in the right basin.
# fp64 on CPU for the polish and the verifier, whose tolerances bind.
SEARCH_DTYPE = torch.float32
EXACT_DTYPE = torch.float64


@lru_cache(maxsize=1)
def probe() -> dict[str, Any]:
    """What this machine offers, established by running something on it."""
    if not torch.cuda.is_available():
        return {
            "device": "cpu",
            "reason": "torch reports no CUDA device",
            "torch": torch.__version__,
            "built_for_cuda": torch.version.cuda,
            "usable": True,
        }
    name = torch.cuda.get_device_name(0)
    major, minor = torch.cuda.get_device_capability(0)
    found: dict[str, Any] = {
        "device": "cuda",
        "name": name,
        "capability": f"sm_{major}{minor}",
        "torch": torch.__version__,
        "built_for_cuda": torch.version.cuda,
        "memory_gib": round(torch.cuda.get_device_properties(0).total_memory / 1024**3, 1),
    }
    try:
        # Small, and a matmul rather than an allocation: allocating succeeds
        # on a card whose kernels are missing, which is the whole point.
        a = torch.randn(64, 64, device="cuda", dtype=SEARCH_DTYPE)
        torch.cuda.synchronize()
        float((a @ a).sum())
    except RuntimeError as exc:
        return {
            **found,
            "device": "cpu",
            "usable": False,
            "reason": (
                f"this torch is built for CUDA {torch.version.cuda} and has no kernels for "
                f"{found['capability']} ({name}); falling back to CPU. Install a torch built "
                f"for a CUDA new enough for this card. Original error: {exc}"
            ),
        }
    return {**found, "usable": True, "reason": "a kernel ran on the device"}


def device() -> torch.device:
    """The device to put search work on. CPU if the GPU cannot run a kernel."""
    return torch.device(probe()["device"])


def warp_stream(where: torch.device):
    """Warp's handle to torch's CURRENT stream on `where`; None off the GPU.

    Warp launches land on warp's own per-device stream by default, and torch
    work lands on torch's current stream. On the DEFAULT stream the two
    happen to coincide, which is why every warp launch in this package worked
    without this -- until the first caller ran on a side stream. There the
    launch runs unordered against the tensors it reads, and inside CUDA graph
    capture (whose stream is never the default) it is silently left OUT of
    the graph: the capture succeeds and the replay reruns everything except
    the warp kernels. Pass the result as `wp.launch(..., stream=...)`.
    """
    if where.type != "cuda":
        return None
    import warp as wp

    return wp.stream_from_torch(torch.cuda.current_stream(where))


def self_test() -> int:
    found = probe()
    assert found["device"] in {"cpu", "cuda"}
    assert found["reason"], "a device choice with no reason is a device choice nobody can debug"

    # Whatever was chosen, the geometry core must run on it and agree with
    # the CPU answer to search accuracy -- which is the property that makes
    # the fp32/GPU path safe to use for the graph and the refinement.
    from weld_motion_planner.geometry.curves import Curve

    line = {
        "segments": [
            {"degree": 1, "knots_u": [0, 0, 1, 1], "control_points_xyz_m": [[0, 0, 0], [3, 4, 0]]}
        ]
    }
    here = Curve(line, dtype=SEARCH_DTYPE, device=device())
    s = torch.linspace(0.0, 1.0, 5, dtype=SEARCH_DTYPE, device=device())
    points = here.point(s)
    assert points.device.type == device().type, points.device
    assert abs(here.total - 5.0) < 1e-4, here.total

    exact = Curve(line, dtype=EXACT_DTYPE).point(torch.linspace(0.0, 1.0, 5, dtype=EXACT_DTYPE))
    assert float((points.cpu().double() - exact).abs().max()) < 1e-5

    print(
        f"device self-test ok: {found['device']} ({found.get('name', 'cpu')}), "
        f"{found['reason']}; the curve core agrees with fp64 on CPU"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    print(json.dumps(probe(), indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"device: {exc}", file=sys.stderr)
        raise SystemExit(1)
