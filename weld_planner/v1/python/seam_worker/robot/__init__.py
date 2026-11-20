"""The cell, and where the part sits in it.

`cell` reads a URDF as DATA -- reduced to what placement needs, never
interpreted as a robot to command. `kinematics` puts every link somewhere for
a given set of joint values. `placement` is where the workpiece sits, which is
a human judgement made by dragging in the authoring UI and then recorded.

The same three concerns `weld_motion_planner.robot_cell.robot` carries, and deliberately the same
folder name: this side needs them to decide whether a weld is reachable, that
side needs them to drive an arm, and neither imports the other.
"""
