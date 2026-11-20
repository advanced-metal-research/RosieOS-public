"""Can this torch actually weld it, and what to do where it cannot.

`access` is stage 3 -- which stretches of a weld line no torch can reach, and
it answers tooling-INDEPENDENTLY. `tooling` is the torch itself and the one
inequality that decides where it fits; `clearance` asks the specific question
that inequality allows: does THIS torch, at THESE angles, go in. `orient`
chooses the angles, by a lattice and a dynamic program, and deliberately with
no OCCT anywhere in it.

`decisions` is the other half of an answer nobody wants: what to do about the
stretches that came back blocked. A reachability report that ends in a number
and no options is a report the operator cannot act on.
"""
