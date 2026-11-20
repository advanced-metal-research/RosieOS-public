"""From topology to the seams you would actually weld, and their geometry.

`detect` is stage 2 -- weld joints found in the topology -- and `seams` turns
those joints into seams, which is a different thing: a joint is a fact about
the part and a seam is a decision about how to weld it. `continuity` says how
smoothly one weld-line segment joins the next, which is what decides whether
two segments are one pass or two.

The rest is the arithmetic those decisions are made in: `curve` evaluates the
rational B-splines and their arc length and curvature, `frames` stands the
`{P, t, r, b}` frame on each point, `span` is a seam's stretch of its joint,
and `scalar` is the profiles authored along normalized arc length.

Nothing here knows about a torch or a robot.
"""
