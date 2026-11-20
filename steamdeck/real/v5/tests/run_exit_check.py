#!/usr/bin/env python3
"""The real launcher quits with fake network/process tools; no robot is contacted.

Only a freshly observed, same-cell unattended server loop may outlive exit.
Ordinary arming, attended runs (every loop on a cell whose manifest does not
allow unattended loops), ended loops, missing data and stale observations
still send exactly one Stop, after exactly one status read. Native safety and
renewal remain the server's concern.
"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


CELL = "example-arm-1420"
HERE = Path(__file__).resolve().parents[1]


def sample(stamp):
    return {
        "available": True, "armed": True, "daemon_error": "",
        "target": {"cell_id": CELL, "selection_generation": "7"},
        "loop": {"mode": "loop", "attended": False, "state": "running", "trajectory_digest": "sha256:fixture"},
        "session": {"state": "playing", "trajectory_digest": "sha256:fixture", "lease": {"held": True}},
        "rt_core": {"inhibited": False, "stop_uncertain": False, "reason": "", "status": {
            "time_ns": stamp, "daemon_incarnation": "native-1", "adapter_incarnation": "adapter-1",
            "grant": {"NowHostNs": stamp + 50_000_000},
            "core": {"time_ns": stamp, "configuration_epoch": 1, "home_epoch": 1,
                     "armed": 1, "lease_valid": 1, "safety_fault_mask": 0}}},
    }


class ExitChecks(unittest.TestCase):
    def run_case(self, observed, *, keep=False, expected=CELL, fail_read=False, missing_python=False):
        with tempfile.TemporaryDirectory(prefix="rosie-run-exit-") as name:
            root = Path(name)
            app, fake, runtime = root / "app", root / "fake", root / "runtime"
            for p in (app / "bin", fake, runtime):
                p.mkdir(parents=True)
            shutil.copyfile(HERE / "run.sh", app / "run.sh")
            (root / "observed.json").write_text(json.dumps(observed))

            def executable(path, content):
                path.write_text(content)
                path.chmod(0o755)

            executable(app / "bin/rosie-pendant-v5", "#!/bin/sh\nexit 0\n")
            executable(fake / "flock", "#!/bin/sh\nexit 0\n")
            executable(fake / "pgrep", "#!/bin/sh\nexit 1\n")
            executable(fake / "curl", "#!" + sys.executable + "\n" + '''
import json, os, sys
from pathlib import Path
root = Path(os.environ["EXIT_FIXTURE"])
url = sys.argv[-1]
if url.endswith("/status"):
    count = root / "reads"
    n = int(count.read_text()) + 1 if count.exists() else 1
    count.write_text(str(n))
    if os.environ.get("FAIL_READ") == "1":
        sys.exit(22)
    data = json.loads((root / "observed.json").read_text())
    print(data if isinstance(data, str) else json.dumps(data))
elif url.endswith("/stop"):
    with (root / "stops").open("a") as f:
        f.write("STOP\\n")
''')
            if missing_python:
                executable(fake / "python3", "#!/bin/sh\nexit 127\n")
            env = dict(os.environ, PATH=str(fake) + os.pathsep + os.environ["PATH"],
                       XDG_RUNTIME_DIR=str(runtime), EXIT_FIXTURE=str(root), FAIL_READ="1" if fail_read else "0",
                       ROSIE_PENDANT_OLP_ORIGIN="http://fixture.invalid", ROSIE_PENDANT_EXPECT_CELL=expected)
            result = subprocess.run(["bash", str(app / "run.sh")], env=env, capture_output=True, text=True, timeout=8)
            self.assertEqual(result.returncode, 0, result.stderr)
            stops = (root / "stops").read_text().splitlines() if (root / "stops").exists() else []
            self.assertEqual(stops, [] if keep else ["STOP"], result.stderr)
            self.assertEqual("unattended program loop remains running" in result.stderr, keep)
            reads = int((root / "reads").read_text()) if (root / "reads").exists() else 0
            self.assertLessEqual(reads, 1, "exit decides from one status read")

    def test_running_and_finishing_unattended_cycles_survive_exit(self):
        for mode, state, execution in (("loop", "running", "playing"), ("count", "finishing", "playing"),
                                       ("count", "running", "done"), ("count", "running", "loaded")):
            with self.subTest(mode=mode, state=state, execution=execution):
                observed = sample(1_000_000_000)
                observed["loop"].update(mode=mode, state=state)
                observed["session"]["state"] = execution
                self.run_case(observed, keep=True)

    def test_uncertain_ordinary_or_changed_ownership_stops(self):
        mutations = {
            "manual_arm": lambda s: s.pop("loop"),
            "attended": lambda s: s["loop"].update(attended=True),
            "missing_attended": lambda s: s["loop"].pop("attended"),
            "ended": lambda s: s["loop"].update(state="ended"),
            "stopped": lambda s: s["session"].update(state="stopped"),
            "fault": lambda s: s["rt_core"]["status"]["core"].update(safety_fault_mask=1),
            "inhibited": lambda s: s["rt_core"].update(inhibited=True),
            "stop_uncertain": lambda s: s["rt_core"].update(stop_uncertain=True),
            "unavailable": lambda s: s.update(available=False),
            "wrong_cell": lambda s: s["target"].update(cell_id="other-arm-1420"),
            "no_selection": lambda s: s["target"].update(selection_generation=""),
            "no_native_incarnation": lambda s: s["rt_core"]["status"].update(daemon_incarnation=""),
            "no_adapter_incarnation": lambda s: s["rt_core"]["status"].update(adapter_incarnation=""),
            "status_call_failed": lambda s: s["rt_core"].update(reason="rt_core_status_slow"),
            "stale_native": lambda s: s["rt_core"]["status"]["grant"].update(NowHostNs=4_000_000_000),
            "lost_lease": lambda s: s["session"]["lease"].update(held=False),
            "new_trajectory": lambda s: s["loop"].update(trajectory_digest="sha256:other"),
            "bare_unattended_setting": lambda s: s["loop"].clear(),
        }
        for name, mutate in mutations.items():
            with self.subTest(name=name):
                observed = sample(1_000_000_000)
                mutate(observed)
                self.run_case(observed)
        self.run_case("not JSON")
        self.run_case(None)
        self.run_case(sample(1_000_000_000), expected="")
        self.run_case(sample(1_000_000_000), missing_python=True)
        self.run_case(sample(1_000_000_000), fail_read=True)


if __name__ == "__main__":
    unittest.main()
