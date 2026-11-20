"""The machinery under M5 and M6's stages.

`weld_trajopt` synthesizes the weld passes, `freespace` the moves between them,
`roadmap` the global routes `freespace` seeds from. The stages at the package
root (`weld_trajopt_main`, `connecting_trajopt_main`) are the only pipeline
callers; everything here also runs on its own (`python -m ...engine.freespace`).
"""
