"""Description and sphere-search checks; no hardware or control connection."""
import hashlib
import json
from pathlib import Path
import sys
import unittest

import numpy as np

MODEL = Path(__file__).resolve().parents[1]
ROOT = MODEL.parents[2]
sys.path.insert(0, str(ROOT / "weld_planner/v1/python"))
from weldplan.urdf import read_urdf
from weldplan.robot_description import mesh_sha256, parse_config
from weld_motion_planner.robot_cell.collision.pairs import self_ignore
from weld_motion_planner.robot_cell.collision.spheres import SphereModel, problems
from weld_motion_planner.verifier.verify import kinematics


class DescriptionTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((MODEL / "config.json").read_text())
        self.cell = read_urdf(MODEL / "robot.urdf", frames=self.config["frames"])
        self.spheres = SphereModel.from_json(json.loads((MODEL / "spheres.json").read_text()))

    def test_identity_and_six_axis_configuration(self):
        raw = (MODEL / "robot.urdf").read_bytes()
        self.assertEqual(hashlib.sha256(raw).hexdigest(), "ae401a9385060c9595f415564b84b3ff9f6a9136b88c4025b7f434aefb4c1106")
        self.assertEqual(self.spheres.provenance["robot_urdf_sha256"], hashlib.sha256(raw).hexdigest())
        self.assertEqual(self.spheres.provenance["meshes_sha256"], mesh_sha256(MODEL))
        definition = json.loads((MODEL / "rtcore_definition.json").read_text())
        self.assertEqual([j["gear_ratio"]["numerator"] for j in definition["joints"]], [100, 100, 100, 18, 100, 100])
        self.assertEqual([j["sign"] for j in definition["joints"]], [-1] * 6)
        self.assertEqual([j["encoder_battery"] for j in definition["joints"]], ["present"] * 6)
        velocities = {j["name"]: j["velocity_rad_s"] for j in self.cell["joints"] if j["type"] == "revolute"}
        parse_config(self.config, model_id="rosie_1430_v1", velocities=velocities)
        self.assertEqual(problems(self.spheres, self.cell, driven=[f"link_{i}" for i in range(1, 7)]), [])

    def clearance(self, pose):
        frames = kinematics(self.cell).transforms(pose)
        names = list(self.spheres.links)
        mask = self_ignore(self.cell, names)
        gap = float("inf")
        for i, a in enumerate(names):
            for j, b in enumerate(names):
                if j <= i or not mask[i][j]:
                    continue
                av = np.asarray([c for c, _ in self.spheres.links[a]])
                bv = np.asarray([c for c, _ in self.spheres.links[b]])
                ar = np.asarray([r for _, r in self.spheres.links[a]])
                br = np.asarray([r for _, r in self.spheres.links[b]])
                aw = av @ frames[a][:3, :3].T + frames[a][:3, 3]
                bw = bv @ frames[b][:3, :3].T + frames[b][:3, 3]
                gap = min(gap, float(np.min(np.linalg.norm(aw[:, None, :] - bw, axis=2) - ar[:, None] - br)))
        return gap

    def test_home_clear_and_folded_pose_rejected(self):
        self.assertGreater(self.clearance([0.0] * 6), 0.005)
        self.assertLess(self.clearance([0, 1.9, 1.53, 0, 0, 0]), -0.001)


if __name__ == "__main__":
    unittest.main()
