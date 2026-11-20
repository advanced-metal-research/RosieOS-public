"""The STEP, and what OCCT gives back.

`topology` is stage 1: a STEP file read into B-rep topology with surface
geometry and per-solid face ownership, which every later stage indexes into.
`tessellate` is the display mesh the authoring UI draws, carrying the CAD face
each triangle came from so a click in the viewer means a face and not a
guess. `occlusion` is the indexed-triangle intersector `reach` asks whether a
line of sight is blocked.

**This is where the expensive imports live**, and that is the whole reason
this package's `__init__` files stay empty of them: OCCT and numba are pulled
in by the modules here, and a folder that re-exported them would make every
CLI entry point and every test that only needs span arithmetic pay for the CAD
stack at import.
"""
