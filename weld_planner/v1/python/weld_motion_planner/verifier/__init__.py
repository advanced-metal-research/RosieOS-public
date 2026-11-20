"""The judge, and the evidence it judges on.

    verify   the T2 verifier: conservative-advancement certificates over real
             triangles at fp64, with an independent forward kinematics. Its
             docstring argues why sampling is not verification.
    model    the evidence: the cell as real triangles, built from the same
             request the planner plans from and sharing nothing with the
             model the planner searches against.

Built before the contestants and answering to none of them. That is why this
folder is not under `planner/`, and why nothing in it may grow a dependency on
a stage: a judge assembled from the work it judges inherits the work's
mistakes. `robot_cell/collision/model.py` holds the planner's half of the pair and
the argument for keeping the two different.
"""
