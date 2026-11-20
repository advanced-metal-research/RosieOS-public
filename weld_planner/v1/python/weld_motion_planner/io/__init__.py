"""The boundary: what enters this package, what leaves it, and its shapes.

    plan_request  the way IN: a `.weldplan` opened and verified, with what this
                  planner still cannot plan from said as sentences.
    results       the vocabulary the answers are stated in: what a plan IS, as
                  opposed to how it is made. Declarations only, and the one
                  module here that touches no file -- it is the shape `wire`
                  serialises FROM, not the serialisation.
    wire          the way OUT: a finished result to the JSON a reader consumes,
                  and the only place degrees are spoken. `server` is its one
                  caller.
    debug         the side door: the bundles the dev pages draw, schema `v0`,
                  which nothing may depend on.
"""
