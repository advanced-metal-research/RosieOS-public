# Planning/control-loop regression

Compiles the production Motion Server translation unit and drives its real
UDP/NATS handlers with loopback fixtures. Collision is a controlled reply, not
a geometric judge. No robot endpoint is contacted.

Run through `go test ./test/motion-server -run TestAsyncPlanningPreservesControlAndCancellation`
from `robot/v4`, or invoke `run.py` with explicit source, output and model paths.
The Go entrypoint runs nine cases covering feedback/lease continuity, Stop,
Disarm, epoch/Home/URDF changes, exact retries, busy rejection, collision
rejection and owner replacement. Each case retains its result and source hashes.

Derived from the shared comparison fixture at RosieOS `01e2a26`, itself adapted
from JSLOP `1d65235` cold-command-result tests. Precision-only checks were removed
to compare the same control behavior across both baselines. Do not weaken the
feedback continuity or cancellation checks to accommodate synchronous planning.

The predecessor failed with zero feedback polls during pending collision work.
The initial async port also failed until fixed-target feedback polling was
decoupled from prior jog-client traffic. Both failures remain in the September
12 comparison packet. These tests establish software behavior, not physical
Stop time or qualified robot motion.
