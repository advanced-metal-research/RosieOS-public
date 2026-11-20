# 0018 — A butt or corner joint is invisible to detection: only faces pressed together are joints

**Status:** open. Found 2026-09-11 from a part in front of an operator —
`bracket_b` has a 190 mm corner seam down its back that never appears as
something to click.

## What happens

`detect_joints` finds joints only where two faces are PRESSED TOGETHER. Every
joint it has ever emitted, on every fixture, carries
`evidence.method: "coincident_face_pair"`, and `classify_joint_type` returns
only `"lap"` or `"tee"`.

A butt or corner joint is the opposite configuration: two faces that are
COPLANAR and ADJACENT, meeting along a shared boundary edge rather than
overlapping. There is no code path for it, so those joints are not reported at
all — not with low confidence, not as a near miss, not at all.

On `bracket_b` (four solids: a 190x100x10 base plate, a 190x10x80 upright at
y 40..50, two 10x70x70 gussets) the upright is flush with the base plate's back
edge — both end at y = 50. That leaves

- `solid_0:face_0008`, normal `[0, 1, 0]`, y = 50, spanning z 0..10
- `solid_1:face_0007`, normal `[0, 1, 0]`, y = 50, spanning z 10..90

coplanar, meeting along a 190 mm line at (y = 50, z = 10). Two perpendicular
plates flush at one face: an outside corner, welded from the back much as a
butt is. Detection reports six joints on this part and this is not one of them.
`bracket_b_2x` misses two of them.

The contract already has the vocabulary and has had it all along.
`robot.v4.program.v2.schema.json` allows
`type: butt | tee | lap | corner | edge | cruciform | plug | unknown` and
`weld_symbol: fillet | square_butt | v_groove | bevel_groove | u_groove |
j_groove | plug | spot | seam | unspecified`. The detector emits two of the
first list and one of the second.

## How it was found

An operator said the back of `bracket_b` has a single-V — practically a butt —
and that it was not being detected. Confirmed by running the detector's own
`analyze_pair` on the two faces:

```json
{"face_a": "solid_0:face_0008", "face_b": "solid_1:face_0007",
 "normal_dot": 1.0, "verdict": "rejected",
 "reason": "normals 0.0 deg apart, not antiparallel"}
```

## Why it happens

`seam/detect.py:195`. `analyze_pair` rejects any pair whose normals are not
antiparallel:

```python
if dot > ANTIPARALLEL_DOT:      # -0.999
    return {**out, "verdict": "rejected",
            "reason": f"normals {angle:.1f} deg apart, not antiparallel"}
```

Coplanar faces score `dot = +1.0` and never reach the gap, overlap or
groove-face stages. It is not a threshold that is set wrong — the case is not
written. `_overlap_relation` downstream also assumes overlap, which an adjacent
pair does not have.

## What was done

Nothing yet. Three decisions have to be made before this is written, and they
are design rather than repair:

1. **The pairing rule.** Coplanar, non-overlapping, sharing a boundary edge.
   How far out of plane still counts (the contact path allows 3 mm,
   `NEAR_MISS_GAP_M`), and whether the two faces must share a topological edge
   or merely be geometrically adjacent.

2. **The groove, and therefore the bisector.** For a square corner the two
   groove normals are identical, so `_norm(n_a + n_b)` collapses to the plate
   normal. That happens to be the right torch direction from that side, but it
   arrives by accident rather than by the reasoning `groove_faces_for_edge`
   documents for a fillet. For a real single-V the groove faces are the bevel
   faces and the bisector is theirs. This is what the entire work-angle chain
   hangs off — see the commit that fixed `rail_offset_m`, where a degenerate
   bisector put the torch 45 degrees off every fillet and took coverage from
   99% to 0-12% without any error being raised.

3. **What to emit.** `bracket_b` as modelled has no bevel faces at the back —
   the only two oblique faces in the part are the gussets' hypotenuses, which
   are free edges. So it is `corner` / `square_butt`; `v_groove` needs bevel
   faces actually present in the CAD, and detecting those is a separate
   question from detecting the joint.

Until then the operator's route is the classic edge-draft flow, which does not
depend on detection.
