#!/usr/bin/env python
"""Where the wall clock went, per stage -- measured, and measured honestly.

M9 asks for "the stage breakdown per run, against the budget", and until this
existed there was nothing to make one from: the planner reported what it
concluded and never what it cost, so "tens of seconds cold per plan request" was a
design intention with no instrument behind it. A stage whose time is only ever
felt is a stage that gets optimised by guess.

## The one thing that makes this hard: CUDA is asynchronous

A `perf_counter` around a torch call on the GPU measures how long it took to
QUEUE the work, not to do it. Left alone the numbers do not merely wobble --
they are systematically wrong in a specific and misleading way: the stage that
launches kernels reads as nearly free, and whichever later stage first needs a
result off the device inherits the whole bill. On this pipeline that would have
put the inverse kinematics' time under the collision screen, and sent anyone
reading the log to optimise the wrong stage.

So a `Stopwatch` takes a `sync` callable and calls it before every timestamp.
On CUDA that is `torch.cuda.synchronize()` -- wait for the device to actually
finish -- and the numbers become what they claim to be.

**It is nearly free here, and that is not luck.** The stages are data
dependent: the search cannot start until the lattice exists, the screen cannot
start until the poses do. Each sync waits for work the next stage was about to
wait for anyway. It would be a real cost on a pipeline that overlapped stages
on purpose, and M9's async per-seam DAG is exactly that -- when that lands,
this file is where the note about it goes.

`torch` is deliberately not imported here. This module is arithmetic over a
clock; the caller, which already holds a device, says how to synchronise it.
"""

from __future__ import annotations

import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager


def cuda_sync(device: object) -> Callable[[], None]:
    """A `sync` for `Stopwatch`, or a no-op when the work is not on a GPU.

    Takes the device rather than asking `torch.cuda.is_available()`, because a
    planner told to run on the CPU on a machine that HAS a GPU must not pay for
    a synchronise -- and, more to the point, must not look like it did.
    """
    if getattr(device, "type", None) != "cuda":
        return lambda: None
    import torch

    return torch.cuda.synchronize


class Stopwatch:
    """Named stages and their milliseconds, in the order they first ran.

    Re-entering a name ACCUMULATES rather than replacing, which is what makes
    one watch usable for a whole run: `plan` enters `seams` once per seam and
    reads off the total. Insertion order is pipeline order, and it survives
    into JSON -- so the UI draws a breakdown without being told what follows
    what, and a stage added here appears there with no second edit.
    """

    def __init__(self, sync: Callable[[], None] | None = None) -> None:
        self._sync = sync or (lambda: None)
        self.stages: dict[str, float] = {}
        self._started = self._now()
        self._marked = self._started

    def _now(self) -> float:
        self._sync()
        return time.perf_counter()

    @contextmanager
    def stage(self, name: str) -> Iterator[None]:
        start = self._now()
        try:
            yield
        finally:
            # In `finally`, so a stage that RAISED still reports what it spent
            # before it did. A seam that fails after eleven seconds of lattice
            # is the most interesting line in the log, and dropping its time
            # because it ended in an exception would hide exactly that.
            self.stages[name] = self.stages.get(name, 0.0) + (self._now() - start) * 1000.0

    def mark(self, name: str) -> None:
        """Close the stretch since the last mark and call it `name`.

        The alternative to `stage` for a long straight-line function. `build`
        in `nodes.py` is four hundred lines of which most are the comments
        explaining the decisions in them, and wrapping each stretch in a `with`
        would re-indent all of it -- turning a performance log into a diff
        nobody can review for the thing it actually changed. One line at each
        boundary says the same thing.

        A watch uses marks OR `stage` blocks, never both: a mark closes
        everything since the last one, so a `stage` inside a marked stretch
        would be counted by both of them.
        """
        now = self._now()
        self.stages[name] = self.stages.get(name, 0.0) + (now - self._marked) * 1000.0
        self._marked = now

    def add(self, name: str, ms: float) -> None:
        """Fold in a time measured by somebody else -- a stage that ran its own
        watch, like the lattice's, and hands its stages up."""
        self.stages[name] = self.stages.get(name, 0.0) + float(ms)

    def merge(self, other: dict[str, float], prefix: str = "") -> None:
        for name, ms in other.items():
            self.add(f"{prefix}{name}", ms)

    def elapsed_ms(self) -> float:
        """Everything since this watch was made, stages and the gaps between."""
        return (self._now() - self._started) * 1000.0

    def ms(self, total: bool = True) -> dict[str, float]:
        """The wire and log shape: stage names to milliseconds, one decimal.

        `total` is the whole span, not the sum of the stages, and the two are
        deliberately both present. Their DIFFERENCE is the time nobody claimed
        -- serialisation, imports, a copy off the device somebody forgot about
        -- and a breakdown whose bars add up to exactly its total by
        construction can never show it.
        """
        out = {name: round(ms, 1) for name, ms in self.stages.items()}
        if total:
            out["total"] = round(self.elapsed_ms(), 1)
        return out


def report(stages: dict[str, float], *, indent: str = "  ", width: int = 12) -> list[str]:
    """`{"ik": 388.2}` to `  ik            388 ms   77%`, biggest first.

    Percentages are of `total` when the dict carries one, so they answer "what
    is this run made of" rather than "what fraction of the stages I happened to
    name". Sorted by cost because the first line of a performance log should be
    the thing to look at, and a stage under 0.5% is left out of the ordering's
    way at the bottom rather than being hidden -- an omitted stage reads as a
    stage that did not run.
    """
    total = stages.get("total") or sum(v for k, v in stages.items() if k != "total")
    lines = []
    for name, ms in sorted(stages.items(), key=lambda kv: (kv[0] == "total", -kv[1])):
        share = f"{ms / total * 100:4.0f}%" if total > 0 and name != "total" else "     "
        lines.append(f"{indent}{name:<{width}} {ms:8.0f} ms {share}")
    return lines
