#!/usr/bin/env python3
"""Run the native dense consumer gate; a missing TMPDIR cannot start a runtime.

The public simulator and C++ client exercise immutable uploads, NATS authority,
Stop, commissioning and refusal. No legacy follower or controller is launched.
"""

import os
from pathlib import Path
import subprocess
import sys


def main():
    if len(sys.argv) != 1:
        print("reason=option_retired: the native dense gate takes no legacy options", file=sys.stderr)
        return 2
    if not os.environ.get("TMPDIR"):
        print("reason=tmpdir_required: select Linux socket scratch", file=sys.stderr)
        return 2
    root = Path(__file__).resolve().parents[4]
    build = root / "rt-core" / "build"
    commands = [
        ["make", "-C", str(root / "rt-core"), "control", "sim"],
        ["make", "-C", str(root / "motion-server/joint-trajectory/v1"), "all", "test",
         f"BIN_DIR={build / 'dense'}", f"TEST_BUILD_DIR={build / 'dense-tests'}"],
    ]
    for command in commands:
        result = subprocess.run(command, check=False)
        if result.returncode:
            return result.returncode
    return 0


if __name__ == "__main__":
    sys.exit(main())
