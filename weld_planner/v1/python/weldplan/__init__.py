"""The `.weldplan` plan request: what crosses the line to a motion planner.

**Why this is its own package and not a module of `seam_worker`.** The
plan request is co-owned. One side writes it, the other reads it, and the reader is
a different program with a different job -- today the format's own CLI, later a
motion planner. A reader that had to `import seam_worker` to open one would be
taking on OCCT, numba and the whole authoring stack to parse a zip of JSON, and
would be coupled to the producer's internals besides. So this package imports
nothing but the standard library, and nothing in it may ever import
`seam_worker`. The arrow points one way: `seam_worker` depends on `weldplan`,
never the reverse. `tests/shared/test_plan_request.py` enforces both halves.

That rule is the same one GOAL.md states for `cadquery/v1` and `tesseract/v1`
-- read the data, do not import the module -- applied to a boundary inside this
repository rather than one at its edge.

Empty of re-exports, like `seam_worker.__init__`, and here for a second reason
on top of that one: `plan request` is a `python -m` entry point, and a package
`__init__` that imports it makes the module get loaded twice on that path --
once by the package, once by `runpy` -- which Python reports as a
`RuntimeWarning` about unpredictable behaviour. Importers name the module:
`from weldplan.plan_request import read_request`.

See `docs/motion/MOTION.md` for what a planner is expected to do with one.
"""
