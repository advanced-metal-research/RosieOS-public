"""Resolve the canonical URDF for the independent model tests."""
import pathlib
REPO = pathlib.Path(__file__).resolve().parents[3]
URDF = REPO / "urdf" / "v1" / "rosie_1400_v3" / "robot.urdf"
