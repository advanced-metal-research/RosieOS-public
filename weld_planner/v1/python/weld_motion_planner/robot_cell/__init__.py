"""The station: the machine, what it must not hit, and the one assembly of both.

    robot_cell   `RobotCell`: the cell assembled ONCE -- descriptor, arm,
                 profile, sphere model -- and handed down. Fourteen call sites
                 assembling separately is how a run once screened the welds
                 against one set of meshes and the moves between them against
                 another.
    robot/       the machine -- descriptor, profile, tool, forward and inverse
                 kinematics, mobility. Imports nothing outside itself.
    collision/   what the arm must not hit and how the planner asks -- sphere
                 model, scene, pairs, cost, positioner, and `model`, the
                 assembly every stage searches against.

The arrow inside points one way: `collision/` reads `robot/`, never back. The
assembler is the only thing that needs both, which is why all three share a
folder rather than the two kinds sitting at the top with their assembler
orphaned above them.
"""
