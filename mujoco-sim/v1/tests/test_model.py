"""The MJCF is the URDF: same chain, same limits, same gear ratios."""

import numpy as np
import support
from cellsim import model


def test_mjcf_forward_kinematics_matches_the_urdf_chain(tmp_path):
    cell = model.build(support.URDF, tmp_path)
    assert cell.joint_names == ("J1", "J2", "J3", "J4", "J5", "J6", "J9", "J7", "J8")
    ratios = {j.name: j.gear_ratio for j in cell.joints}
    assert ratios["J1"] == 100 and ratios["J4"] == 18 and abs(ratios["J5"] - 100 / 11) < 1e-12 and ratios["J9"] == 10
    rng = np.random.default_rng(3)
    for _ in range(3):
        q = np.array([rng.uniform(j.lower, j.upper) for j in cell.joints])
        assert model.fk_check(cell, q) < 1e-9


def test_meshes_convert_once(tmp_path):
    cell = model.build(support.URDF, tmp_path)
    objs = sorted((tmp_path / "rosie_1400_v3").glob("*.obj"))
    assert len(objs) == 11
    stamp = (tmp_path / "rosie_1400_v3" / "link_1.stamp").stat().st_mtime_ns
    model.build(support.URDF, tmp_path)
    assert (tmp_path / "rosie_1400_v3" / "link_1.stamp").stat().st_mtime_ns == stamp
    assert cell.xml.count("<motor ") == 9
