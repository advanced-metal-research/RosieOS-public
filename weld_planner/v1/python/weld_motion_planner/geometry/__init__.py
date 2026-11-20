"""Shapes and fields: what the planner measures against.

`curves` and `seam_frame` are the SEAM -- the line the electrode follows and
the frame that stands on it. `mesh` is the part, tessellated by the planner
rather than handed to it, and `esdf` is the distance field baked from those
triangles. Nothing here knows about a robot.
"""
