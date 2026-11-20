"""What the arm must not hit, and how the planner asks.

`spheres` is the robot's own collision model, `scene` assembles everything
else into distance fields grouped by the frame each body is rigid in, and
`pairs` says which contacts the tree makes meaningless. `cost` is the
differentiable term the optimizer descends; `positioner` precomputes which
workpiece angles put the part in the room at all. `model` is the assembly of
all of it from one plan request -- the model every stage searches against,
and deliberately NOT the one the verifier judges by (`verifier/model.py`).
"""
