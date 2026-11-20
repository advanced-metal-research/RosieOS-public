"""The planner over HTTP, and nothing more than that.

    motion_planner_server   the surface: one POST, its parameters, and the
                            uvicorn launcher behind `pixi run -e motion
                            motion-serve`
    response                what that POST does to build an answer -- join the
                            plan (M6 over M5), then make the result legal
                            JSON

**Why the planner needs a server of its own.** The authoring side
(`seam_worker.server`, :8795) runs in an environment with OCCT and no CUDA;
this package needs CUDA and may never import `seam_worker` -- a rule
`tests/motion_planner/test_boundary.py` enforces by reading the source. The seam
between the two sides is a FILE, the `.weldplan`, and this keeps it that way
over HTTP: the operator UI asks the authoring server to pack the plan request, then
hands those same bytes here. Two processes, two environments, one contract, and
neither side imports the other.

Empty of re-exports, like every folder in this package. The bytes boundary
itself -- plan request in, JSON-shaped dict out -- is `weld_motion_planner.io.wire`,
which is deliberately NOT here: it is what a caller exchanges with the planner,
and a CLI or a test may want it without starting a server.
"""
