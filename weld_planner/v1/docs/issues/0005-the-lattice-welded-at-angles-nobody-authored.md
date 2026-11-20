# 0005 · The lattice planned every weld at 0°/0° while the program asked for a ramp

**Status:** the nominal is fixed — `nodes.build` reads the authored profiles and
records what it planned at. The **tolerance bands are still unused**, and that
half is open.
**Found:** 2026-08-12, answering "why is the search space (spin, positioner)? you
don't necessarily have to leave those free." Looking for the answer in the data
turned up that the data says more than the code reads.
**Blast radius:** every lattice ever built. Not a feasibility bug — the corpus
crosses either way — a *correctness of the question* bug. The planner was
answering about a weld the program did not author.

## What is wrong

`nodes.build` signed off with

    work_deg: float = 0.0, travel_deg: float = 0.0

and every caller in the repo took the default. Meanwhile the seams say:

| seam | work angle | travel angle |
|---|---|---|
| `bracket_a_placed` 1 | 0° → 2° | **34.5° → 15° → 10° → 10°** |
| `bracket_a_placed` 2 | 2° | 10° |
| `dog_box` 1 | 0° | 10° |
| `pipe_bracket` 1 | 0° | 10° |

Across the fifteen-seam corpus the authored travel angle averages **10.5°** and
swings **24.5°** within a single seam. The lattice welded all of it flat at zero.

Travel angle is drag: the torch leaning back over completed weld. Ten degrees of
it is not a rounding — it is the difference between the bead the program
specifies and a different bead. Every downstream number — reachability,
clearance, the K-best path, the frontier report — answered about the second one.

## Why the search space is (spin, positioner), and what else the data offers

That was the question this came out of, and the data answers it directly:

    "spin": {"mode": "free"}

Spin being free is **declared in the seam**, not assumed by the code — the arc
off a round wire is axisymmetric, so rolling the torch about its own electrode
changes the arm and not the weld. The positioner is a machine axis with no weld
meaning at all. So both are free in the strong sense: the weld is indifferent.

But the program authorizes a third and fourth freedom that nothing reads:

    "tolerance":  {"work_angle_deg": 5.0, "travel_angle_deg": 5.0, "position_mm": 0.5}
    "orientation.limits": {
        "travel_angle_deg": {"min": 0.0, "max": 20.0, "max_rate_deg_per_mm": 0.5},
        "work_angle_deg":   {"min": -5.0, "max": 5.0, "max_rate_deg_per_mm": 0.5}}

Two bounded, rate-limited bands the weld engineer signed off on. They are not
free the way spin is — moving inside them changes the weld, within stated
tolerance — but they are *authorized*, which is the only thing the search needs.
`dog_box` and `pipe_bracket` carry them; `bracket_a` does not.

So the answer to "you don't have to leave those free" is that the code was not
making that choice at all. Spin's freedom is read from a `mode` the code ignores,
and two authorized freedoms are pinned at a value that is not even the nominal.

## What was done

- **`nodes.authored_angles(seam, fractions)`** evaluates the seam's own
  `work_angle_deg` and `travel_angle_deg` at each sample. Handles `constant` and
  `piecewise_linear`; **refuses** `bspline` and `samples` rather than flattening
  them, since a weld planned at an angle nobody authored is what this issue is.
  Parameterized on the seam's own 0..1, matching `clearance._angle_pair` on the
  authoring side.
- **`work_deg` and `travel_deg` are gone from `nodes.build` entirely**
  (2026-08-17). The first fix kept them as an override defaulting to `None`
  meaning "what the program asked for" — nothing could silently get zero, but a
  caller could still ask for a weld nobody wrote, which is this issue. Nothing
  in the repo ever passed them: `weld_trajopt` and `verify` both call
  `authored_angles` directly, and the tests that mention the names are building
  a `Lattice` by hand. So the parameters were the defect's own signature, kept
  for a caller that never arrived. `build` now reads the profiles and offers no
  way to disagree, and `_per_sample` went with them — it existed to accept a
  scalar, and `authored_angles` always returns one value per sample.
- **`Lattice.work_deg` / `travel_deg`** carry the series, and
  `summary()["angles_deg"]` carries the range. Same rule as `Lattice.held` from
  0003: an assumption that is not in the output is an assumption nobody will
  ever catch.
- A second implementation of the evaluator on purpose. `weld_motion_planner` may not
  import `weld_programmer` (`tests/motion_planner/test_boundary.py`), so the two agreeing
  is checked against the committed plan requests rather than inherited from shared
  code.

The corpus crosses 15 of 15 at the authored angles, same as at zero.

## What this cost 0004, which is the part worth reading

0004's headline was "the corpus goes 14 of 15 to 15 of 15 when the bins are
sized from the threshold". **That measurement was taken at the wrong weld
angles.** With the authored profile, `dog_box` seam 1 crosses on the coarse 8×6
lattice too, so the corpus no longer distinguishes the two fixes: either one
alone takes it to 15 of 15.

0004's mechanism survives; its evidence changed. The positioner is a component of
the search state compared against its own threshold, and the path through
`dog_box` seam 1 uses **one** positioner bin at 60° spacing and **six** at 24°.
That is the claim, measured directly, and it does not need a failing seam.

**And one sentence of 0004 was simply wrong.** It said a spin axis binned wider
than the threshold "can never be crossed". Spin is not a state component — a
spin change is paid in arm joints, and can be paid *jointly with a branch
change* — and the same path crosses **four** spin bins at 45° spacing. What
stays true is narrower: adjacent spin bins on the *same branch* are unreachable,
so a spin change is only available where a branch change happens to come with
it. `pinned_axes` now says exact for the positioner and advisory for spin, and
the test asserts only the exact half.

## What is still owed

- **The bands.** `orientation.limits` and `tolerance` authorize ±5° of work
  angle and 0–20° of travel, rate-limited to 0.5°/mm. Making them lattice
  dimensions is the M4 rescue pass, and unlike spin they need the rate limit as
  an EDGE rule as well as bins as a node dimension — a band you may sit anywhere
  in is not the same as a band you may move through at 0.5°/mm, and 0004's whole
  lesson is that bin spacing and edge rule have to be chosen together. **Note
  that the deleted `work_deg` was never the hook for this** — a band is bins plus
  an edge rule, not one number substituted for the authored profile, so restoring
  the parameter would not be a step toward it.
- **`spin.mode` is still not read.** Every seam in the corpus says `free` and
  the lattice enumerates spin unconditionally, so a seam that ever pins spin —
  an asymmetric nozzle, cable routing, a torch with a gas lens orientation —
  would be planned as if it had not.
- **`tolerance.position_mm: 0.5`** is a freedom too, and a bigger one than it
  looks: half a millimetre of tip position at every sample. Nothing reads it.
- `bspline` and `samples` profiles are refused rather than evaluated. That is
  correct today and will need doing the first time a program uses one.
