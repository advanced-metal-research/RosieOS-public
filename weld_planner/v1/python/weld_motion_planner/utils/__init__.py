"""Instruments with no opinion about welding.

    timing   the honest stopwatch: per-stage wall clock that synchronises the
             device before every timestamp, because on CUDA an unsynchronised
             clock bills each stage for the wrong work. Its header carries the
             full argument.
    device   which device the planner runs on, proven by running a kernel --
             a capability query says yes and then dies mid-bake on a torch
             built for an older card. Also the two dtype policies,
             `SEARCH_DTYPE` and `EXACT_DTYPE`.

The test for adding a file here: it may know about torch and about clocks,
and it may not know what a weld, a seam, or a cell is. The moment it does, it
belongs in the folder for that kind of thing.
"""
