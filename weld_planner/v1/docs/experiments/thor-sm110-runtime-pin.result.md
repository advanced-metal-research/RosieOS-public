# Thor sm_110 runtime pin result

Verdict: **passed** for the source-owned runtime capability only.

`motion-thor` now composes the locked non-Torch planner dependencies with the
pinned NVIDIA 26.04 image. The runner refuses pulls and executes a real CUDA
matrix multiplication before the requested command. On `thor-a` (a fleet Jetson Thor) it reported:

- device: `NVIDIA Thor`;
- capability: `sm_110`;
- Torch: `2.12.0a0+0291f960b6.nv26.04.48445190`;
- built for CUDA: `13.2`;
- usable: `true` because a kernel ran.

The fleet setup no longer installs conda Torch on ARM64 and then overwrites it
with an unpinned pip wheel.

With runtime ambiguity removed, untouched free-space tests still report six
source failures: five control-solver contract violations and one known winding
determinism failure. This branch does not change those algorithms, tests, or
tolerances. It does not claim a full-suite pass.

Execution authority remains `none`. Motion Server, hardware motion, process
behavior, and physical readiness were not evaluated.
