"""Run independent MuJoCo physics without exposing the retired controller ABI."""
from __future__ import annotations
import argparse
import pathlib
import sys
import time


def main(argv: list[str]) -> int:
    """Validate before model allocation; obsolete transport flags cannot bind a port."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf")
    parser.add_argument("--cache")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--steps", type=int, default=0)
    retired = ("--udp-listen", "--authority-json", "--ready-file", "--status-output", "--axes")
    if any(arg.split("=", 1)[0] in retired for arg in argv):
        parser.error("option_retired: controller transport removed; use dev-stack.sh with rt_core")
    args = parser.parse_args(argv)
    if args.steps < 0:
        parser.error("--steps must be nonnegative")
    import mujoco
    from cellsim import model
    cell = model.build(pathlib.Path(args.urdf) if args.urdf else model.default_urdf(),
                       pathlib.Path(args.cache) if args.cache else model.default_cache())
    physics = mujoco.MjModel.from_xml_string(cell.xml)
    data = mujoco.MjData(physics)
    if args.viewer:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(physics, data) as viewer:
            while viewer.is_running():
                mujoco.mj_step(physics, data)
                viewer.sync()
                time.sleep(physics.opt.timestep)
    else:
        for _ in range(args.steps):
            mujoco.mj_step(physics, data)
        print(f"MuJoCo model: {physics.njnt} joints; steps={args.steps}; transport=none")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
