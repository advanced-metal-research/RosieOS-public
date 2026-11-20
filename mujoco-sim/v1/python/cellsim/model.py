"""The cell as a MuJoCo model, built from the same URDF everything else reads.

MuJoCo's own URDF importer wants `package://` resolved and reads no Collada,
and the cell URDF carries no `<inertial>` at all (checked 2026-08-29: zero
`<inertial>` elements in `rosie_1400_v3/robot.urdf`). So the sim builds its
MJCF itself: one body per link, meshes converted once (`meshes.py`), mass and
inertia from the visual mesh volume at a configured density, and the motor's
rotor inertia reflected through the gearbox as joint `armature`. Joint order,
axes, origins and limits come from the URDF unchanged; `<transmission>`
`mechanicalReduction` is the gear ratio RTCore uses for counts per radian.

Rotations: URDF `rpy` is fixed-axis X, Y, Z. MJCF `euler` is *intrinsic* by
default, so the compiler is told `eulerseq="XYZ"` (uppercase = extrinsic);
`fk_check()` compares every link pose against a numpy chain built straight
from the URDF at a random pose, so a wrong convention fails loudly.
"""

from __future__ import annotations

import dataclasses
import hashlib
import math
import pathlib
import sys
import xml.etree.ElementTree as ET

import numpy as np

from cellsim import meshes

# Aluminium. The arm links are castings and the meshes are solid shells with
# no cavities, so this over-estimates mass a little; the drive cascade sees
# the load only through the inertia ratio it was tuned for, and the
# `density_kg_m3` field in the cell config is the knob to move. Not measured.
DEFAULT_DENSITY_KG_M3 = 2700.0

# Physics step. The drives' torque filter is 200 Hz (C01.03 default) and the
# speed loop 25 Hz; 4 kHz keeps the discretised filters well away from their
# poles and is 4 steps per 1 ms CSP cycle. `mj_step` on this 13-body model is
# ~3 us on the dev box (2026-08-29), so the step rate is not the cost.
PHYSICS_DT_S = 0.00025


class ModelError(ValueError):
    pass


@dataclasses.dataclass(frozen=True)
class JointSpec:
    name: str
    parent: str
    child: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float]
    lower: float
    upper: float
    gear_numerator: int
    gear_denominator: int

    @property
    def gear_ratio(self) -> float:
        return self.gear_numerator / self.gear_denominator


@dataclasses.dataclass(frozen=True)
class CellModel:
    urdf_path: pathlib.Path
    xml: str
    joints: tuple[JointSpec, ...]          # URDF order, J1..J6, J9, J7, J8 for rosie_1400_v3
    joint_names: tuple[str, ...]
    mesh_dir: pathlib.Path

    def joint_index(self, name: str) -> int:
        return self.joint_names.index(name)


def _floats(text: str | None, default: str) -> tuple[float, float, float]:
    a, b, c = (float(x) for x in (text or default).split())
    return (a, b, c)


def _parse_transmissions(root: ET.Element) -> dict[str, tuple[int, int]]:
    ratios: dict[str, tuple[int, int]] = {}
    for tr in root.findall("transmission"):
        joint = tr.find("joint")
        red = tr.find("actuator/mechanicalReduction")
        if joint is None or red is None:
            continue
        num = int(red.get("numerator", "1"))
        den = int(red.get("denominator", "1"))
        ratios[joint.get("name", "")] = (num, den)
    return ratios


def parse_urdf(urdf_path: pathlib.Path) -> tuple[dict[str, str | None], list[JointSpec], list[ET.Element]]:
    """Links (name -> visual mesh stem or None), movable joints, all joints."""
    root = ET.parse(urdf_path).getroot()
    links: dict[str, str | None] = {}
    for link in root.findall("link"):
        mesh = link.find("visual/geometry/mesh")
        stem = None
        if mesh is not None:
            stem = mesh.get("filename", "").split("/")[-1].rsplit(".", 1)[0]
        links[link.get("name", "")] = stem
    ratios = _parse_transmissions(root)
    joints: list[JointSpec] = []
    for j in root.findall("joint"):
        if j.get("type") != "revolute":
            continue
        origin = j.find("origin")
        limit = j.find("limit")
        axis = j.find("axis")
        if limit is None or axis is None:
            raise ModelError(f"{urdf_path}: joint {j.get('name')} lacks <limit> or <axis>")
        name = j.get("name", "")
        num, den = ratios.get(name, (1, 1))
        joints.append(JointSpec(
            name=name,
            parent=j.find("parent").get("link"),  # type: ignore[union-attr]
            child=j.find("child").get("link"),  # type: ignore[union-attr]
            xyz=_floats(origin.get("xyz") if origin is not None else None, "0 0 0"),
            rpy=_floats(origin.get("rpy") if origin is not None else None, "0 0 0"),
            axis=_floats(axis.get("xyz"), "0 0 1"),
            lower=float(limit.get("lower", "-3.141592653589793")),
            upper=float(limit.get("upper", "3.141592653589793")),
            gear_numerator=num,
            gear_denominator=den,
        ))
    return links, joints, root.findall("joint")


def build(urdf_path: pathlib.Path,
          cache_dir: pathlib.Path,
          density_kg_m3: float = DEFAULT_DENSITY_KG_M3,
          armature_kgm2: dict[str, float] | None = None,
          joint_damping_nms: float = 0.0,
          timestep_s: float = PHYSICS_DT_S) -> CellModel:
    """MJCF for the cell. `armature_kgm2` is per joint, already reflected to the joint."""
    urdf_path = pathlib.Path(urdf_path).resolve()
    links, movable, all_joints = parse_urdf(urdf_path)
    mesh_dir = urdf_path.parent / "meshes"
    objs = meshes.convert_all(mesh_dir, cache_dir / urdf_path.parent.name)
    armature = armature_kgm2 or {}

    children: dict[str, list[ET.Element]] = {}
    for j in all_joints:
        children.setdefault(j.find("parent").get("link"), []).append(j)  # type: ignore[union-attr]

    mj = ET.Element("mujoco", model=urdf_path.parent.name)
    ET.SubElement(mj, "compiler", angle="radian", eulerseq="XYZ",
                  inertiafromgeom="true", meshdir=str(objs and next(iter(objs.values())).parent))
    ET.SubElement(mj, "option", timestep=f"{timestep_s}", integrator="implicitfast",
                  gravity="0 0 -9.81")
    ET.SubElement(mj, "default").append(
        ET.Element("geom", contype="0", conaffinity="0", density=f"{density_kg_m3}"))
    asset = ET.SubElement(mj, "asset")
    used = sorted({stem for stem in links.values() if stem and stem in objs})
    for stem in used:
        ET.SubElement(asset, "mesh", name=stem, file=objs[stem].name)
    world = ET.SubElement(mj, "worldbody")
    ET.SubElement(world, "light", pos="1.5 0.5 3", dir="0 0 -1", diffuse="0.8 0.8 0.8")
    actuators = ET.SubElement(mj, "actuator")

    def emit(parent_el: ET.Element, link_name: str) -> None:
        stem = links.get(link_name)
        if stem and stem in objs:
            ET.SubElement(parent_el, "geom", type="mesh", mesh=stem, rgba="0.75 0.76 0.78 1")
        for j in children.get(link_name, []):
            child = j.find("child").get("link")  # type: ignore[union-attr]
            origin = j.find("origin")
            body = ET.SubElement(parent_el, "body", name=child,
                                 pos=(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"),
                                 euler=(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"))
            if j.get("type") == "revolute":
                spec = next(s for s in movable if s.name == j.get("name"))
                ET.SubElement(body, "joint", name=spec.name, type="hinge",
                              axis=" ".join(f"{a:g}" for a in spec.axis),
                              range=f"{spec.lower} {spec.upper}", limited="true",
                              armature=f"{armature.get(spec.name, 0.0):.6g}",
                              damping=f"{joint_damping_nms:g}")
                # Torque actuator; the drive cascade sets ctrl in N*m at the joint
                # every physics step. No ctrlrange: the drive's torque limit
                # is applied in the cascade where the real limit lives.
                ET.SubElement(actuators, "motor", name=spec.name, joint=spec.name, gear="1")
            emit(body, child)

    emit(world, "world")
    xml = ET.tostring(mj, encoding="unicode")
    return CellModel(urdf_path=urdf_path, xml=xml, joints=tuple(movable),
                     joint_names=tuple(s.name for s in movable), mesh_dir=mesh_dir)


# --- FK straight from the URDF, for the check ------------------------------

def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_angle(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=np.float64)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def urdf_link_poses(urdf_path: pathlib.Path, q: dict[str, float]) -> dict[str, np.ndarray]:
    root = ET.parse(urdf_path).getroot()
    poses: dict[str, np.ndarray] = {"world": np.eye(4)}
    pending = list(root.findall("joint"))
    while pending:
        progressed = False
        for j in list(pending):
            parent = j.find("parent").get("link")  # type: ignore[union-attr]
            if parent not in poses:
                continue
            origin = j.find("origin")
            t = np.eye(4)
            t[:3, :3] = rpy_matrix(*_floats(origin.get("rpy") if origin is not None else None, "0 0 0"))
            t[:3, 3] = _floats(origin.get("xyz") if origin is not None else None, "0 0 0")
            if j.get("type") == "revolute":
                m = np.eye(4)
                m[:3, :3] = axis_angle(_floats(j.find("axis").get("xyz"), "0 0 1"),  # type: ignore[union-attr]
                                       q.get(j.get("name", ""), 0.0))
                t = t @ m
            poses[j.find("child").get("link")] = poses[parent] @ t  # type: ignore[union-attr]
            pending.remove(j)
            progressed = True
        if not progressed:
            raise ModelError(f"{urdf_path}: joints with unreachable parents: "
                             f"{[j.get('name') for j in pending]}")
    return poses


def fk_check(model: CellModel, q: np.ndarray) -> float:
    """Worst link-frame position error (m) between MuJoCo and the URDF chain at q."""
    import mujoco

    m = mujoco.MjModel.from_xml_string(model.xml)
    d = mujoco.MjData(m)
    for i, name in enumerate(model.joint_names):
        d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)]] = q[i]
    mujoco.mj_forward(m, d)
    ref = urdf_link_poses(model.urdf_path, dict(zip(model.joint_names, q.tolist())))
    worst = 0.0
    for link, pose in ref.items():
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, link)
        if bid < 0:
            continue
        worst = max(worst, float(np.linalg.norm(d.xpos[bid] - pose[:3, 3])))
        rot_err = float(np.abs(d.xmat[bid].reshape(3, 3) - pose[:3, :3]).max())
        worst = max(worst, rot_err)  # unitless, but a wrong convention shows as O(1)
    return worst


def digest(model: CellModel) -> str:
    return hashlib.sha256(model.xml.encode()).hexdigest()[:16]


def main(argv: list[str]) -> int:
    import argparse

    import mujoco

    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("urdf", nargs="?", default=str(default_urdf()))
    ap.add_argument("--cache", default=str(default_cache()))
    ap.add_argument("--xml-out", help="write the MJCF here")
    args = ap.parse_args(argv)
    model = build(pathlib.Path(args.urdf), pathlib.Path(args.cache))
    m = mujoco.MjModel.from_xml_string(model.xml)
    print(f"{model.urdf_path}: {m.nbody} bodies, {m.njnt} joints, mjcf {digest(model)}")
    for i in range(m.nbody):
        print(f"  {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i):22s} mass {m.body_mass[i]:8.2f} kg")
    rng = np.random.default_rng(7)
    q = np.array([rng.uniform(j.lower, j.upper) for j in model.joints])
    err = fk_check(model, q)
    print(f"fk check at a random pose: worst error {err:.2e} ({'ok' if err < 1e-9 else 'MISMATCH'})")
    if args.xml_out:
        pathlib.Path(args.xml_out).write_text(model.xml)
    return 0 if err < 1e-9 else 1


def repo_root() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[4]


def default_urdf() -> pathlib.Path:
    return repo_root() / "robot_description" / "robots" / "rosie_1400_v3" / "robot.urdf"


def default_cache() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[2] / ".cache" / "meshes"


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
