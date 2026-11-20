"""The machine: what it is, and where it goes.

`cell` reads the descriptor, `profile` and `tool` carry the two things a URDF
does not say (velocity limits, where the torch points). `fk` takes joint
angles to world spheres in one kernel; `ik` and `ik_closed` go the other way,
iteratively and in closed form. `mobility` is how fast the tip can move for a
unit of joint speed.
"""
