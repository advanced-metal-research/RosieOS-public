#!/usr/bin/env python
"""The signed distance field the search collides against.

## Exact everywhere, which is a departure from cuRoboV2 and cheaper here

`ARCHITECTURE.md` planned a narrow band of exact mesh distances with a
propagated far field, following cuRoboV2 §5.4 -- because that is what you do
when the grid is a room and the source is a depth camera. This scene is
neither. It is known CAD, bounded by a part, and baked once offline, and a
GPU BVH answers a closest-point query in microseconds. A 300 mm part at
1.5 mm is about a million voxels; querying all of them exactly costs less
than building the seeding pass would have.

So there is no band and no Parallel Banding Algorithm. Every voxel holds the
true distance to the nearest triangle. The near field -- which is the only
part the torch ever visits, and the part cuRoboV2's voxel-stamped world
approximates -- is exact by construction rather than by budget.

Measured against their `JumpFloodingEDT` on the same grids, both are
milliseconds: 8M voxels in 5.2 ms here, 27M in 15 ms. Their propagation is
quantized to voxel centres -- a worst case of one voxel near the surface,
so ~1.5 mm at the resolution the part field uses, against 0 here -- and
matching this bake's near-field accuracy would take a ~3x finer grid at
~26x the memory. The table and its corrections are in
`docs/motion/BENCHMARKS.md`.

The first bake in a process costs ~370 ms more than the rest, all of it
Warp loading the kernel. That is a startup cost, not a per-part one.

## Sign by winding number, not by face orientation

A tessellated STEP has reversed faces in it: OCCT emits each face with its
own orientation, and a solid assembled from trimmed surfaces routinely has
some wound inward. Deciding inside-ness from the nearest triangle's normal
therefore flips the field on those faces, and a flipped voxel beside the
seam reads as "the torch is inside the part" exactly where clearance
matters most.

The generalized winding number does not care. It is a property of the
surface as a whole rather than of any one triangle, so mixed per-face
orientation and small gaps both come out right, and Warp computes it in the
same query as the distance. Verified on the real tessellation: dog_box's
base block reads -73 mm at its centre with a quarter of the grid inside.

The one thing it cannot rescue is a surface wound inside-out as a WHOLE --
that is a coherent solid describing the complement, and no local rule can
know it was not meant. The self-test's sphere hit exactly this when its
triangles were built the wrong way round, which is why it is now wound
explicitly and why the real part is checked for a negative interior rather
than assumed to have one.

## In the workpiece frame

The part's field is baked in the STEP's own coordinates, which is the
workpiece frame, so it rides the positioner for free. Queries transform the
robot's spheres INTO this frame rather than transforming the field.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from functools import cached_property
from typing import Any

import numpy as np
import torch

from weld_motion_planner.geometry.mesh import Mesh, MeshError
from weld_motion_planner.utils.device import warp_stream

# Default voxel for the PART field. `ARCHITECTURE.md` asks for 1-2 mm: fine
# enough that trilinear interpolation error is well under the millimetre
# clearances a weld runs at, coarse enough that a 300 mm part is ~1e6 voxels.
DEFAULT_VOXEL_M = 0.0015

# How far past the part's bounding box the field extends. The torch reaches
# in from outside, so the field has to describe the space it approaches
# through, not only the part -- and a query outside the grid has to clamp to
# something meaningful rather than to the edge of the part itself.
DEFAULT_MARGIN_M = 0.05


class FieldError(ValueError):
    """A field this refuses to bake or query, with the reason."""


@dataclass(frozen=True)
class DistanceField:
    """A dense signed distance grid, and the box it covers.

    Dense rather than block-sparse: cuRoboV2 hashes 8^3 blocks because a
    room is mostly empty and mapped incrementally. A part's bounding box is
    small and fully occupied by interest, so hashing would add an
    indirection to every query and save nothing worth having.
    """

    values: torch.Tensor  # (nx, ny, nz), metres, negative inside
    origin_pos_m: torch.Tensor  # (3,) the centre of voxel (0,0,0)
    voxel_m: float
    frame: str  # which frame the coordinates are in

    @property
    def shape(self) -> tuple[int, int, int]:
        return tuple(int(v) for v in self.values.shape)

    @property
    def upper_pos_m(self) -> torch.Tensor:
        counts = torch.tensor(self.shape, dtype=self.origin_pos_m.dtype, device=self.origin_pos_m.device)
        return self.origin_pos_m + (counts - 1) * self.voxel_m

    @cached_property
    def _fused_launch_args(self) -> tuple[Any, torch.Tensor, Any]:
        """The fused kernel's loop-invariant launch arguments, built once.

        The origin used to be read back to the host on EVERY query -- a
        device-to-host copy, which is a full sync in the middle of the
        optimizer's loop and the copy that keeps CUDA graph capture from
        recording anything that samples this field. The values wrapper is
        cached for the same reason `robot/fk.py` builds its warp wrappers
        once; the flat tensor rides along so the memory the wrapper points
        at cannot be collected. (`cached_property` writes to `__dict__`
        directly, which is what lets it live on a frozen dataclass.)
        """
        import warp as wp

        flat = self.values.detach().to(torch.float32).reshape(-1).contiguous()
        origin = self.origin_pos_m.detach().cpu()
        return (wp.from_torch(flat), flat,
                wp.vec3(float(origin[0]), float(origin[1]), float(origin[2])))

    def sample(self, pos_m: torch.Tensor) -> torch.Tensor:
        """Trilinear distance at arbitrary points, differentiably.

        Outside the grid the field is EXTENDED rather than clamped: a clamp
        would report the boundary's distance everywhere beyond it -- a wall
        of constant clearance, and a zero gradient exactly where the
        optimizer needs to be pushed back.

        The extension is `max(boundary value, distance to the box)`, and the
        max is load-bearing. Both terms are LOWER bounds on the true
        distance -- the mesh lies inside the box, so projecting onto the box
        cannot move a point further from the mesh, and no mesh point is
        closer than the box itself -- so their max is a lower bound too:
        clearance outside the grid is understated, never overstated. The
        obvious-looking alternative, boundary value PLUS exit distance, is
        the triangle-inequality UPPER bound; measured on bracket_a it
        overstated clearance by up to 88 mm, which for a collision field is
        the unsafe direction. The gradient survives: where the exit distance
        dominates, d(max)/dp is the unit vector back toward the box.
        """
        points = pos_m.to(dtype=self.values.dtype, device=self.values.device)
        flat = points.reshape(-1, 3)

        lower, upper = self.origin_pos_m, self.upper_pos_m
        inside = torch.clamp(flat, lower, upper)
        outside_m = torch.linalg.norm(flat - inside, dim=-1)

        # grid_sample wants normalized coordinates in [-1, 1] and indexes
        # (z, y, x) -- the reversal is the classic way to get a field that
        # looks right and is transposed.
        counts = torch.tensor(self.shape, dtype=flat.dtype, device=flat.device)
        index = (inside - lower) / self.voxel_m
        norm = 2.0 * index / (counts - 1).clamp_min(1) - 1.0
        grid = norm.flip(-1).reshape(1, -1, 1, 1, 3)
        sampled = torch.nn.functional.grid_sample(
            self.values[None, None], grid, mode="bilinear", align_corners=True
        ).reshape(-1)
        # Only where the point actually left the box. Inside it `outside_m`
        # is zero and a bare max would erase every negative value -- the
        # entire interior of the part would read as "on the surface".
        out = torch.where(outside_m > 0.0, torch.maximum(sampled, outside_m), sampled)
        return out.reshape(pos_m.shape[:-1])

    def sample_fused(self, pos_m: torch.Tensor) -> torch.Tensor:
        """`sample`, with the gradient computed inside the kernel.

        Same values, same conservative extension, same differentiability --
        but backward is a buffer read instead of a second interpolation
        pass, which is the whole difference measured against cuRoboV2's
        fused voxel kernel. Use this in the optimizer's loop; `sample` stays
        as the reference the parity tests hold it to, and as the path for a
        field living on CPU.

        The gradient buffer is written unconditionally: like cuRobo's, this
        kernel prices the derivative at a handful of extra multiplies over
        corners it already holds in registers, so a no-grad forward is not
        worth a second kernel.
        """
        if self.values.device.type != "cuda":
            return self.sample(pos_m)
        _ensure_kernel()
        points = pos_m.to(dtype=torch.float32, device=self.values.device)
        return _FusedSample.apply(points.reshape(-1, 3).contiguous(), self).reshape(
            pos_m.shape[:-1]
        )

    def summary(self) -> dict[str, Any]:
        return {
            "frame": self.frame,
            "shape": list(self.shape),
            "voxels": int(self.values.numel()),
            "voxel_mm": round(self.voxel_m * 1000.0, 4),
            "origin_m": [round(float(v), 6) for v in self.origin_pos_m],
            "upper_pos_m": [round(float(v), 6) for v in self.upper_pos_m],
            "min_m": round(float(self.values.min()), 6),
            "max_m": round(float(self.values.max()), 6),
            "inside_fraction": round(float((self.values < 0).float().mean()), 4),
            "memory_mib": round(self.values.numel() * self.values.element_size() / 1024**2, 1),
        }


class _FusedSample(torch.autograd.Function):
    """The torch face of the fused kernel: cuRobo's `wp_autograd` shape.

    Forward launches once and keeps the gradient buffer; backward is a
    multiply against it. No kernel runs in backward, which is the point.
    """

    @staticmethod
    def forward(ctx, points: torch.Tensor, field: DistanceField) -> torch.Tensor:
        import warp as wp

        n = len(points)
        value = torch.empty(n, dtype=torch.float32, device=points.device)
        grad = torch.empty(n, 3, dtype=torch.float32, device=points.device)
        nx, ny, nz = field.shape
        values_wp, _keepalive, origin = field._fused_launch_args
        wp.launch(
            _sample_with_grad,
            dim=n,
            inputs=[
                values_wp,
                nx,
                ny,
                nz,
                origin,
                float(field.voxel_m),
                # `requires_grad=False` on every wrapper: this Function carries
                # its own gradient in `grad`, and `wp.from_torch` would
                # otherwise ALLOCATE a warp grad buffer per call for a
                # differentiable tensor -- a device allocation that is illegal
                # inside CUDA graph capture and dead weight outside it.
                wp.from_torch(points, dtype=wp.vec3, requires_grad=False),
                wp.from_torch(value, requires_grad=False),
                wp.from_torch(grad, dtype=wp.vec3, requires_grad=False),
            ],
            # On torch's CURRENT stream, not warp's default: `sample_fused`
            # only reaches here on CUDA, and a launch that does not follow
            # torch's stream runs unordered on a side stream and is left out
            # of a CUDA graph capture -- see `utils.device.warp_stream`.
            stream=warp_stream(points.device),
        )
        ctx.save_for_backward(grad)
        return value

    @staticmethod
    def backward(ctx, grad_output: torch.Tensor):
        (grad,) = ctx.saved_tensors
        return grad_output.unsqueeze(-1) * grad, None


def bake(
    mesh: Mesh,
    *,
    voxel_m: float = DEFAULT_VOXEL_M,
    margin_m: float = DEFAULT_MARGIN_M,
    frame: str = "workpiece",
    device: torch.device | None = None,
    dtype: torch.dtype = torch.float32,
) -> DistanceField:
    """Signed distance at every voxel, exactly, on the GPU when there is one."""
    import warp as wp

    wp.init()
    # The kernels are built lazily so that importing this module costs
    # nothing. `bake` used to read the module global without building it,
    # which worked only because the CLI and the self-test happen to call
    # `_ensure_kernel` first -- so `esdf.bake(...)` from anything else died
    # with "can only launch functions decorated with @wp.kernel".
    _ensure_kernel()
    lo, hi = mesh.bounds
    lo = lo - margin_m
    hi = hi + margin_m
    counts = np.maximum(np.ceil((hi - lo) / voxel_m).astype(int) + 1, 2)
    if int(np.prod(counts)) > 400_000_000:
        raise FieldError(
            f"a {voxel_m * 1000:.1f} mm grid over this bounding box is {counts.tolist()} "
            f"= {np.prod(counts) / 1e6:.0f}M voxels; coarsen it or bound it"
        )

    torch_device = device or (torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu"))
    warp_device = "cuda" if torch_device.type == "cuda" else "cpu"

    axes = [
        torch.arange(int(n), dtype=torch.float32, device=torch_device) * voxel_m + float(o)
        for n, o in zip(counts, lo, strict=True)
    ]
    grid = torch.stack(torch.meshgrid(*axes, indexing="ij"), dim=-1).reshape(-1, 3)

    field = wp.Mesh(
        points=wp.array(mesh.vertices.astype(np.float32), dtype=wp.vec3, device=warp_device),
        indices=wp.array(mesh.faces.reshape(-1).astype(np.int32), dtype=wp.int32, device=warp_device),
    )
    queries = wp.from_torch(grid.contiguous(), dtype=wp.vec3)
    out = wp.zeros(len(grid), dtype=wp.float32, device=warp_device)

    # The query radius has to reach the far corner of the padded box, or
    # voxels beyond it silently keep their initial value -- zero, which reads
    # as "on the surface" and is the worst possible wrong answer.
    reach = float(np.linalg.norm(hi - lo))
    wp.launch(_signed_distance, dim=len(grid), inputs=[field.id, queries, reach, out], device=warp_device)
    wp.synchronize_device(warp_device)

    values = wp.to_torch(out).reshape(*counts.tolist()).to(dtype=dtype)
    return DistanceField(
        values=values.contiguous(),
        origin_pos_m=torch.tensor(lo, dtype=dtype, device=values.device),
        voxel_m=float(voxel_m),
        frame=frame,
    )


def _make_kernel():
    import warp as wp

    @wp.kernel
    def sample_with_grad(
        values: wp.array(dtype=float),
        nx: int,
        ny: int,
        nz: int,
        origin: wp.vec3,
        voxel: float,
        points: wp.array(dtype=wp.vec3),
        out_value: wp.array(dtype=float),
        out_grad: wp.array(dtype=wp.vec3),
    ):
        """Trilinear value AND its analytic gradient, one pass over memory.

        The point of this kernel is what it does NOT do: run a second pass.
        `sample` gets its gradient from torch autograd, which replays the
        interpolation backward and scatters through atomics -- measured 6x
        slower than cuRoboV2's fused voxel kernel on identical data. This is
        that kernel's shape on our field: the eight corners are in registers
        anyway, so the derivative of trilinear interpolation is a handful of
        extra multiplies, and backward becomes a buffer read.

        Semantics are `sample`'s exactly, conservative extension included:
        outside the box the result is max(boundary value, distance to box),
        never the sum, and the gradient follows whichever term won -- the
        unit vector back toward the box when the exit distance dominates,
        the boundary interpolation (zeroed on clamped axes, since moving
        along a clamped axis does not move the clamp) when it does not.
        """
        tid = wp.tid()
        p = points[tid]
        upper = wp.vec3(
            origin[0] + float(nx - 1) * voxel,
            origin[1] + float(ny - 1) * voxel,
            origin[2] + float(nz - 1) * voxel,
        )
        cx = wp.clamp(p[0], origin[0], upper[0])
        cy = wp.clamp(p[1], origin[1], upper[1])
        cz = wp.clamp(p[2], origin[2], upper[2])
        dx = p[0] - cx
        dy = p[1] - cy
        dz = p[2] - cz
        outside = wp.sqrt(dx * dx + dy * dy + dz * dz)

        fx = (cx - origin[0]) / voxel
        fy = (cy - origin[1]) / voxel
        fz = (cz - origin[2]) / voxel
        ix = wp.min(int(wp.floor(fx)), nx - 2)
        iy = wp.min(int(wp.floor(fy)), ny - 2)
        iz = wp.min(int(wp.floor(fz)), nz - 2)
        tx = fx - float(ix)
        ty = fy - float(iy)
        tz = fz - float(iz)

        base = (ix * ny + iy) * nz + iz
        c000 = values[base]
        c001 = values[base + 1]
        c010 = values[base + nz]
        c011 = values[base + nz + 1]
        c100 = values[base + ny * nz]
        c101 = values[base + ny * nz + 1]
        c110 = values[base + ny * nz + nz]
        c111 = values[base + ny * nz + nz + 1]

        c00 = c000 * (1.0 - tz) + c001 * tz
        c01 = c010 * (1.0 - tz) + c011 * tz
        c10 = c100 * (1.0 - tz) + c101 * tz
        c11 = c110 * (1.0 - tz) + c111 * tz
        c0 = c00 * (1.0 - ty) + c01 * ty
        c1 = c10 * (1.0 - ty) + c11 * ty
        value = c0 * (1.0 - tx) + c1 * tx

        # d/dt of the interpolation, per axis. Easy to cross: `gy` wants the
        # Y-differences (c01-c00, c11-c10), and writing the X-pattern there
        # produced gradients wrong by up to 35 in the first run -- caught by
        # the parity test against autograd, which is why that test exists.
        gx = (c1 - c0) / voxel
        gy = ((c01 - c00) * (1.0 - tx) + (c11 - c10) * tx) / voxel
        gz = (
            ((c001 - c000) * (1.0 - ty) + (c011 - c010) * ty) * (1.0 - tx)
            + ((c101 - c100) * (1.0 - ty) + (c111 - c110) * ty) * tx
        ) / voxel

        if outside > 0.0:
            if dx != 0.0:
                gx = 0.0
            if dy != 0.0:
                gy = 0.0
            if dz != 0.0:
                gz = 0.0
            if outside >= value:
                out_value[tid] = outside
                out_grad[tid] = wp.vec3(dx / outside, dy / outside, dz / outside)
            else:
                out_value[tid] = value
                out_grad[tid] = wp.vec3(gx, gy, gz)
        else:
            out_value[tid] = value
            out_grad[tid] = wp.vec3(gx, gy, gz)

    @wp.kernel
    def signed_distance(
        mesh: wp.uint64,
        points: wp.array(dtype=wp.vec3),
        max_dist: float,
        out: wp.array(dtype=float),
    ):
        i = wp.tid()
        point = points[i]
        query = wp.mesh_query_point_sign_winding_number(mesh, point, max_dist)
        if query.result:
            closest = wp.mesh_eval_position(mesh, query.face, query.u, query.v)
            # `sign` is -1 inside and +1 outside for the winding-number query.
            # Negative inside is this module's convention and cuRobo's, so a
            # margin is `distance - radius` in both.
            out[i] = query.sign * wp.length(point - closest)
        else:
            out[i] = max_dist

    return signed_distance, sample_with_grad


_signed_distance = None
_sample_with_grad = None


def _ensure_kernel():
    global _signed_distance, _sample_with_grad
    if _signed_distance is None:
        _signed_distance, _sample_with_grad = _make_kernel()
    return _signed_distance


def self_test() -> int:
    """Against a sphere and a box, whose distance fields are closed forms."""
    _ensure_kernel()

    # A unit-ish sphere as a triangle mesh, fine enough that faceting is well
    # under the tolerance asked of the field.
    radius = 0.05
    lat, lon = 64, 128
    thetas = np.linspace(0.0, np.pi, lat)
    phis = np.linspace(0.0, 2.0 * np.pi, lon, endpoint=False)
    points = np.array(
        [
            [radius * np.sin(t) * np.cos(p), radius * np.sin(t) * np.sin(p), radius * np.cos(t)]
            for t in thetas
            for p in phis
        ]
    )
    faces = []
    for i in range(lat - 1):
        for j in range(lon):
            a = i * lon + j
            b = i * lon + (j + 1) % lon
            c = (i + 1) * lon + j
            d = (i + 1) * lon + (j + 1) % lon
            # Wound OUTWARD. A globally inverted surface is the one case the
            # winding number cannot rescue -- see the module docstring -- so
            # a fixture that got this backwards would be testing the failure
            # rather than the field. It did, on the first run: every inside
            # probe came back positive.
            faces.append([a, c, b])
            faces.append([b, c, d])
    sphere = Mesh(points, np.array(faces, dtype=np.int32))

    field = bake(sphere, voxel_m=0.002, margin_m=0.02)
    assert field.values.min() < 0.0, "nothing was inside the sphere"

    # Closed form: |p| - r, sampled well away from the facets.
    probes = torch.tensor(
        [
            [0.0, 0.0, 0.0],
            [0.03, 0.0, 0.0],
            [0.0, 0.07, 0.0],
            [0.04, 0.04, 0.0],
            [-0.06, 0.0, 0.02],
        ],
        dtype=torch.float32,
        device=field.values.device,
    )
    got = field.sample(probes)
    want = torch.linalg.norm(probes, dim=-1) - radius
    worst = float((got - want).abs().max())
    assert worst < 0.0015, f"sphere field is off by {worst * 1000:.3f} mm"

    # Sign, explicitly: inside must be negative even though a tessellated
    # solid has reversed faces in it.
    assert float(field.sample(probes[:1])) < 0.0, "the centre of a sphere is not inside it"

    # Outside the grid the field keeps growing rather than flattening, so an
    # optimizer that wanders out still gets pushed back.
    far = torch.tensor([[10.0, 0.0, 0.0]], dtype=torch.float32, device=field.values.device)
    assert float(field.sample(far)) > 9.0, float(field.sample(far))

    # Differentiable, and the gradient of a sphere's distance field points
    # radially outward with unit length.
    probe = torch.tensor(
        [[0.03, 0.0, 0.0]], dtype=torch.float32, device=field.values.device, requires_grad=True
    )
    field.sample(probe).backward()
    slope = probe.grad[0]
    assert abs(float(torch.linalg.norm(slope)) - 1.0) < 0.05, slope
    assert float(slope[0]) > 0.9, slope

    print(
        f"esdf self-test ok: a sphere's field matches |p|-r to {worst * 1000:.3f} mm, is "
        f"negative inside, keeps growing outside the box, and has a unit radial gradient"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--read", default="", help="a .weldplan whose part to bake")
    parser.add_argument("--voxel-mm", type=float, default=DEFAULT_VOXEL_M * 1000.0)
    ns = parser.parse_args(argv)
    _ensure_kernel()
    if ns.self_test:
        return self_test()
    if not ns.read:
        raise FieldError("--read or --self-test")

    import time

    from weld_motion_planner.geometry.mesh import tessellate
    from weld_motion_planner.io.plan_request import read

    request = read(ns.read)
    started = time.time()
    mesh = tessellate(request.step_bytes)
    meshed = time.time()
    field = bake(mesh, voxel_m=ns.voxel_mm / 1000.0)
    print(
        json.dumps(
            {
                "source": request.step_filename,
                "mesh": mesh.summary(),
                "field": field.summary(),
                "seconds": {
                    "tessellate": round(meshed - started, 3),
                    "bake": round(time.time() - meshed, 3),
                },
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except (FieldError, MeshError) as exc:
        print(f"esdf: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
