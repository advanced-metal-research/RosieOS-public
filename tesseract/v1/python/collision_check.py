#!/usr/bin/env python
"""Discrete Tesseract collision check worker for the Robot v4 motion-server.

The motion-server's spreadsheet planner (robot_v4_cartesian_daemon.cpp) shells
out to this worker to run a REAL tesseract_collision discrete contact check
(BulletDiscreteBVHManager) at every planned waypoint of its dense trajectory.
The worker loads the exact pair-owned generated URDF selected by Motion Server
(visual meshes as collision meshes) and reports per-waypoint link-pair contacts
with penetration depths.

Request JSON (written by the daemon):
    {
      "schema": "robot-v4.tesseract-collision-check.v1",
      "urdf_path": "/abs/path/to/robot.urdf",
      "robot_model": "rosie_1420_v1",
      "mesh_package_root": "/repo/robot_description/robots/rosie_1420_v1",
      "joint_names": ["J1","J2","J3","J4","J5","J6"],
      "contact_distance_m": 0.0,
      "waypoints": [
        {"waypoint_index": 0, "time_s": 0.0, "positions_rad": [ ... ]},
        ...
      ]
    }

Response JSON:
    {
      "schema": "robot-v4.tesseract-collision-check-result.v1",
      "ok": true,
      "checker": "BulletDiscreteBVHManager",
      "tesseract_nanobind_version": "0.35.0.5",
      "collision_geometry_source": "urdf_visual_and_collision_meshes",
      "checked_waypoint_count": 51,
      "contact_count": 0,
      "acm_adjacent_pairs": ["floor|link_1", ...],
      "acm_reviewed_semantic_pairs": ["link_1|pedestal"],
      "contacts": [
        {"waypoint_index": 3, "time_s": 1.2, "link_a": "link_1",
         "link_b": "link_4", "depth_m": 0.0043},
        ...
      ],
      "check_duration_s": 0.25
    }

Contacts are deduplicated per (waypoint, link pair): only the deepest
penetration for each pair at each waypoint is reported.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.metadata
import json
import socket
import struct
import sys
import tempfile
import time
from pathlib import Path

RESULT_SCHEMA = "robot-v4.tesseract-collision-check-result.v1"
MAX_FRAME_BYTES = 64 * 1024 * 1024


def _identity_loader(module_name: str):
    def load():
        return importlib.import_module(module_name)
    return load


def _rosie_1430_identities() -> dict:
    """Rosie 1430 robots whose URDF and meshes are byte-pinned per robot id.

    Each robot's pins live in its own <model id>_rosie1430.py beside this
    file (their URDFs keep the rosie_1430_v1 mesh package), so no robot id
    is written here: a tree without a robot's module (the public mirror
    leaves the real robots out) does not admit that robot. Modules are
    imported on use, as before.
    """
    here = Path(__file__).resolve().parent
    return {
        path.name[: -len("_rosie1430.py")]: _identity_loader(path.stem)
        for path in sorted(here.glob("*_rosie1430.py"))
        if not path.name.startswith("test_")
    }


ROSIE_1430_IDENTITIES = _rosie_1430_identities()


def load_tesseract_collision_modules():
    from tesseract_robotics.tesseract_collision import (  # type: ignore
        ContactRequest,
        ContactResultMap,
        ContactTestType_ALL,
    )
    from tesseract_robotics.tesseract_common import (  # type: ignore
        CollisionMarginData,
        FilesystemPath,
        GeneralResourceLocator,
    )
    from tesseract_robotics.tesseract_environment import Environment  # type: ignore

    return {
        "CollisionMarginData": CollisionMarginData,
        "ContactRequest": ContactRequest,
        "ContactResultMap": ContactResultMap,
        "ContactTestType_ALL": ContactTestType_ALL,
        "Environment": Environment,
        "FilesystemPath": FilesystemPath,
        "GeneralResourceLocator": GeneralResourceLocator,
    }


def contact_manager_plugins_yaml_path() -> Path:
    """Bullet/FCL contact-manager factory config shipped inside the wheel."""
    import tesseract_robotics  # type: ignore

    package_root = Path(tesseract_robotics.__file__).resolve().parent
    plugins = package_root / "data" / "tesseract" / "support" / "urdf" / "contact_manager_plugins.yaml"
    if not plugins.is_file():
        raise FileNotFoundError(f"contact_manager_plugins.yaml not found in wheel: {plugins}")
    return plugins


def urdf_adjacent_link_pairs(urdf_path: Path) -> list[tuple[str, str]]:
    """Every joint's parent/child pair: the standard adjacent-link ACM."""
    import xml.etree.ElementTree as ET

    root = ET.parse(urdf_path).getroot()
    pairs: list[tuple[str, str]] = []
    for joint in root.iter("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        parent_link = (parent.get("link") or "").strip()
        child_link = (child.get("link") or "").strip()
        if parent_link and child_link:
            pairs.append((parent_link, child_link))
    return pairs


def urdf_reviewed_semantic_collision_pairs(
        urdf_path: Path, robot_model: str) -> list[tuple[str, str]]:
    """Model-specific fixed-base contacts reviewed as part of the URDF chain."""
    import xml.etree.ElementTree as ET

    if robot_model in ROSIE_1430_IDENTITIES:
        ROSIE_1430_IDENTITIES[robot_model]().verify_urdf(urdf_path)
    elif robot_model != "rosie_1420_v1":
        return []
    root = ET.parse(urdf_path).getroot()
    if (root.get("name") or "").strip() != robot_model:
        raise ValueError(
            f"cannot load reviewed collision semantics for {root.get('name')!r}")
    joints: dict[str, tuple[str, str, str]] = {}
    for joint in root.iter("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        joints[(joint.get("name") or "").strip()] = (
            (joint.get("type") or "").strip(),
            (parent.get("link") or "").strip(),
            (child.get("link") or "").strip(),
        )
    # The pedestal and first rotary-link meshes meet across the fixed base at
    # the J1 bearing interface. Bullet reports that designed interface at
    # exactly zero penetration for every J1 pose. Admit it only while the
    # reviewed pedestal -> fixed base -> J1 -> link_1 chain is unchanged.
    expected_chain = {
        "pedestal_to_base": ("fixed", "pedestal", "base"),
        "J1": ("revolute", "base", "link_1"),
    }
    if any(joints.get(name) != expected
           for name, expected in expected_chain.items()):
        raise ValueError(
            f"{robot_model} reviewed pedestal/link_1 collision chain changed")
    return [("pedestal", "link_1")]


def urdf_nearest_actuated_parent_joints(
        urdf_path: Path, active_joint_names: list[str]) -> dict[str, str]:
    """Map every link to its nearest active ancestor joint in the URDF tree."""
    import xml.etree.ElementTree as ET

    root = ET.parse(urdf_path).getroot()
    parent_by_child: dict[str, tuple[str, str]] = {}
    for joint in root.iter("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        parent_link = (parent.get("link") or "").strip()
        child_link = (child.get("link") or "").strip()
        joint_name = (joint.get("name") or "").strip()
        if parent_link and child_link:
            parent_by_child[child_link] = (parent_link, joint_name)
    active = set(active_joint_names)
    result: dict[str, str] = {}
    links = {link for pair in urdf_adjacent_link_pairs(urdf_path) for link in pair}
    for link in links:
        cursor = link
        visited: set[str] = set()
        while cursor in parent_by_child and cursor not in visited:
            visited.add(cursor)
            parent_link, joint_name = parent_by_child[cursor]
            if joint_name in active:
                result[link] = joint_name
                break
            cursor = parent_link
    return result


def materialize_collision_srdf(urdf_path: Path, joint_names: list[str],
                               acm_pairs: list[tuple[str, str]],
                               reviewed_semantic_pairs: list[tuple[str, str]],
                               temp_dir: Path, robot_model: str) -> Path:
    plugins_yaml = contact_manager_plugins_yaml_path()
    lines = ['<?xml version="1.0"?>', f'<robot name="{robot_model}">', '  <group name="manipulator">']
    for name in joint_names:
        lines.append(f'    <joint name="{name}"/>')
    lines.append("  </group>")
    lines.append(f'  <contact_managers_plugin_config filename="{plugins_yaml.as_posix()}"/>')
    reviewed = {tuple(sorted(pair)) for pair in reviewed_semantic_pairs}
    for link_a, link_b in acm_pairs:
        reason = (
            "ReviewedFixedBaseContact"
            if tuple(sorted((link_a, link_b))) in reviewed
            else "Adjacent"
        )
        lines.append(
            f'  <disable_collisions link1="{link_a}" link2="{link_b}" '
            f'reason="{reason}"/>')
    lines.append("</robot>")
    temp_dir.mkdir(parents=True, exist_ok=True)
    out_path = temp_dir / f"{urdf_path.stem}_collision_check.srdf"
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return out_path


class CollisionWorld:
    """A loaded Tesseract/Bullet world reused for every compatible request."""

    def __init__(self, request: dict):
        # Keep heavy imports out of proxy-client processes. Only the persistent
        # server (or the legacy one-shot mode) imports NumPy and Tesseract.
        import numpy as np
        import xml.etree.ElementTree as ET
        from ur10_demo import materialize_urdf_with_absolute_mesh_paths

        self.np = np
        self.mods = load_tesseract_collision_modules()
        self.urdf_path = Path(str(request["urdf_path"])).resolve()
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f"urdf not found: {self.urdf_path}")
        self.joint_names = [str(name) for name in request.get("joint_names", [])]
        if not self.joint_names:
            raise ValueError("joint_names must not be empty")
        self.robot_model = str(request.get("robot_model") or "").strip()
        if self.robot_model not in {"rosie_1400_v3", "rosie_1420_v1", *ROSIE_1430_IDENTITIES}:
            raise ValueError(f"unsupported robot_model: {self.robot_model!r}")
        urdf_robot_model = str(
            ET.parse(self.urdf_path).getroot().get("name") or "").strip()
        if urdf_robot_model != self.robot_model:
            raise ValueError(
                f"URDF robot name {urdf_robot_model!r} does not match "
                f"{self.robot_model!r}")
        self.mesh_package_root = Path(
            str(request.get("mesh_package_root") or "")).resolve()
        mesh_package_name = self.robot_model
        if self.robot_model in ROSIE_1430_IDENTITIES:
            ROSIE_1430_IDENTITIES[self.robot_model]().verify_model(
                self.urdf_path, self.mesh_package_root)
            mesh_package_name = "rosie_1430_v1"
        if (
            (self.robot_model not in ROSIE_1430_IDENTITIES
             and self.mesh_package_root.name != mesh_package_name)
            or not (self.mesh_package_root / "meshes").is_dir()
        ):
            raise ValueError(
                "mesh_package_root does not match "
                f"{self.robot_model}: {self.mesh_package_root}")

        # The workspace must live as long as the cached environment because
        # Tesseract may retain paths to its materialized URDF/SRDF resources.
        self.temp_workspace = tempfile.TemporaryDirectory(
            prefix="rosieos-tesseract-collision-")
        temp_dir = Path(self.temp_workspace.name)
        pair_package_dir = temp_dir / mesh_package_name
        pair_package_dir.mkdir(parents=True, exist_ok=False)
        pair_urdf = pair_package_dir / "robot.urdf"
        pair_urdf.write_bytes(self.urdf_path.read_bytes())
        (pair_package_dir / "meshes").symlink_to(
            self.mesh_package_root / "meshes", target_is_directory=True)
        absolute_urdf = materialize_urdf_with_absolute_mesh_paths(
            pair_urdf, temp_dir=temp_dir)
        self.adjacent_pairs = urdf_adjacent_link_pairs(self.urdf_path)
        self.reviewed_semantic_pairs = urdf_reviewed_semantic_collision_pairs(
            self.urdf_path, self.robot_model)
        self.acm_pairs = self.adjacent_pairs + self.reviewed_semantic_pairs
        self.parent_joint_by_link = urdf_nearest_actuated_parent_joints(
            self.urdf_path, self.joint_names)
        srdf_path = materialize_collision_srdf(
            pair_urdf, self.joint_names, self.acm_pairs,
            self.reviewed_semantic_pairs, temp_dir, self.robot_model)

        self.env = self.mods["Environment"]()
        if not self.env.init(
                self.mods["FilesystemPath"](str(absolute_urdf)),
                self.mods["FilesystemPath"](str(srdf_path)),
                self.mods["GeneralResourceLocator"]()):
            raise RuntimeError(
                f"tesseract Environment.init failed for {absolute_urdf}")
        self.manager = self.env.getDiscreteContactManager()
        if self.manager is None:
            raise RuntimeError(
                "tesseract environment returned no discrete contact manager")
        self.manager.setActiveCollisionObjects(self.env.getActiveLinkNames())
        self.library_version = importlib.metadata.version(
            "tesseract-robotics-nanobind")

    def compatible(self, request: dict) -> bool:
        return (
            Path(str(request.get("urdf_path", ""))).resolve() == self.urdf_path
            and [str(name) for name in request.get("joint_names", [])]
            == self.joint_names
            and str(request.get("robot_model") or "").strip()
            == self.robot_model
            and Path(str(request.get("mesh_package_root", ""))).resolve()
            == self.mesh_package_root
        )

    def check(self, request: dict) -> dict:
        if self.robot_model in ROSIE_1430_IDENTITIES:
            ROSIE_1430_IDENTITIES[self.robot_model]().verify_model(
                self.urdf_path, self.mesh_package_root)
        waypoints = request.get("waypoints", [])
        contact_distance_m = float(request.get("contact_distance_m", 0.0))
        self.manager.setCollisionMarginData(
            self.mods["CollisionMarginData"](contact_distance_m))

        contacts: list[dict] = []
        started = time.time()
        for waypoint in waypoints:
            positions = self.np.asarray(
                [float(v) for v in waypoint["positions_rad"]], dtype=float)
            if positions.shape[0] != len(self.joint_names):
                raise ValueError(
                    f"waypoint {waypoint.get('waypoint_index')} has "
                    f"{positions.shape[0]} joints, expected {len(self.joint_names)}")
            self.env.setState(self.joint_names, positions)
            state = self.env.getState()
            self.manager.setCollisionObjectsTransform(state.link_transforms)
            result_map = self.mods["ContactResultMap"]()
            self.manager.contactTest(
                result_map,
                self.mods["ContactRequest"](self.mods["ContactTestType_ALL"]))
            deepest: dict[tuple[str, str], float] = {}
            for contact in result_map.flattenCopyResults():
                pair = tuple(sorted(contact.link_names))
                depth_m = max(0.0, -float(contact.distance))
                if pair not in deepest or depth_m > deepest[pair]:
                    deepest[pair] = depth_m
            for (link_a, link_b), depth_m in sorted(deepest.items()):
                contacts.append({
                    "waypoint_index": int(waypoint.get("waypoint_index", -1)),
                    "time_s": float(waypoint.get("time_s", 0.0)),
                    "link_a": link_a,
                    "link_b": link_b,
                    "affected_joints": sorted({
                        joint for joint in (
                            self.parent_joint_by_link.get(link_a),
                            self.parent_joint_by_link.get(link_b),
                        ) if joint
                    }),
                    "depth_m": depth_m,
                })
        duration_s = time.time() - started

        return {
            "schema": RESULT_SCHEMA,
            "ok": True,
            "checker": self.manager.getName(),
            "tesseract_nanobind_version": self.library_version,
            "collision_geometry_source": "urdf_visual_and_collision_meshes",
            "checked_waypoint_count": len(waypoints),
            "contact_count": len(contacts),
            "acm_adjacent_pairs": [
                f"{a}|{b}" for a, b in self.adjacent_pairs
            ],
            "acm_reviewed_semantic_pairs": [
                "|".join(sorted((a, b)))
                for a, b in self.reviewed_semantic_pairs
            ],
            "contacts": contacts,
            "check_duration_s": duration_s,
        }


def run_collision_check(request: dict) -> dict:
    return CollisionWorld(request).check(request)


def error_result(error: Exception) -> dict:
    return {
        "schema": RESULT_SCHEMA,
        "ok": False,
        "error": f"{type(error).__name__}: {error}",
        "checked_waypoint_count": 0,
        "contact_count": 0,
        "contacts": [],
    }


def read_exact(connection: socket.socket, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = size
    while remaining:
        chunk = connection.recv(remaining)
        if not chunk:
            raise ConnectionError("collision worker connection closed early")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def read_frame(connection: socket.socket) -> bytes:
    size = struct.unpack(">Q", read_exact(connection, 8))[0]
    if size == 0 or size > MAX_FRAME_BYTES:
        raise ValueError(f"invalid collision worker frame size {size}")
    return read_exact(connection, size)


def write_frame(connection: socket.socket, payload: bytes) -> None:
    if not payload or len(payload) > MAX_FRAME_BYTES:
        raise ValueError("collision worker response exceeds frame bounds")
    connection.sendall(struct.pack(">Q", len(payload)) + payload)


def serve_loopback(ready_file: Path) -> int:
    world: CollisionWorld | None = None
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        host, port = listener.getsockname()
        ready_file.write_text(json.dumps({
            "schema": "robot-v4.tesseract-collision-server-ready.v1",
            "host": host,
            "port": port,
        }) + "\n", encoding="utf-8")
        while True:
            connection, _ = listener.accept()
            with connection:
                try:
                    request = json.loads(read_frame(connection))
                    if world is None or not world.compatible(request):
                        world = CollisionWorld(request)
                    result = world.check(request)
                except Exception as error:
                    result = error_result(error)
                write_frame(connection, json.dumps(result).encode("utf-8"))


def proxy_request(address: str, input_path: Path, output_path: Path) -> int:
    host, separator, port_text = address.rpartition(":")
    if not separator or host != "127.0.0.1":
        raise ValueError("collision server must use 127.0.0.1:PORT")
    port = int(port_text)
    payload = input_path.read_bytes()
    with socket.create_connection((host, port), timeout=240.0) as connection:
        write_frame(connection, payload)
        response = read_frame(connection)
    output_path.write_bytes(response)
    result = json.loads(response)
    return 0 if result.get("ok") else 1


def run_self_test() -> int:
    repo_root = Path(__file__).resolve().parents[3]
    urdf = repo_root / "robot_description" / "robots" / "rosie_1400_v3" / "robot.urdf"
    joint_names = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]
    request = {
        "urdf_path": str(urdf),
        "robot_model": "rosie_1400_v3",
        "mesh_package_root": str(urdf.parent),
        "joint_names": joint_names,
        "contact_distance_m": 0.0,
        "waypoints": [
            {"waypoint_index": 0, "time_s": 0.0, "positions_rad": [0.0] * 9},
            # Folded arm: forces a genuine link_1/link_4 mesh penetration.
            {"waypoint_index": 1, "time_s": 1.0,
             "positions_rad": [0.0, 1.9, 1.53, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
        ],
    }
    result = run_collision_check(request)
    home_contacts = [c for c in result["contacts"] if c["waypoint_index"] == 0]
    folded_contacts = [c for c in result["contacts"] if c["waypoint_index"] == 1]
    assert result["checker"].startswith("Bullet"), result["checker"]
    assert not home_contacts, f"home pose must be collision-free: {home_contacts}"
    assert folded_contacts, "folded arm must self-collide"
    assert any(c["depth_m"] > 0 for c in folded_contacts), folded_contacts
    assert all(c.get("affected_joints") for c in folded_contacts), folded_contacts
    urdf_1420 = repo_root / "robot_description" / "robots" / "rosie_1420_v1" / "robot.urdf"
    request_1420 = {
        "urdf_path": str(urdf_1420),
        "robot_model": "rosie_1420_v1",
        "mesh_package_root": str(urdf_1420.parent),
        "joint_names": ["J1", "J2", "J3", "J4", "J5", "J6"],
        "contact_distance_m": 0.0,
        "waypoints": [
            {"waypoint_index": 0, "time_s": 0.0, "positions_rad": [0.0] * 6},
        ],
    }
    result_1420 = run_collision_check(request_1420)
    assert not result_1420["contacts"], (
        "rosie_1420_v1 HM35 home must be collision-free after applying only "
        f"its reviewed fixed-base contact: {result_1420['contacts']}")
    assert result_1420["acm_reviewed_semantic_pairs"] == [
        "link_1|pedestal"
    ], result_1420["acm_reviewed_semantic_pairs"]
    print("self-test: BulletDiscreteBVHManager mesh contact check passed "
          f"(home clean, folded arm {folded_contacts[0]['link_a']}|{folded_contacts[0]['link_b']} "
          f"depth {folded_contacts[0]['depth_m']:.4f} m; "
          "rosie_1420_v1 reviewed fixed-base contact clean)")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", help="request JSON path")
    parser.add_argument("--output", help="result JSON path")
    parser.add_argument(
        "--serve-loopback", action="store_true",
        help="keep a Tesseract collision world warm on an ephemeral loopback port")
    parser.add_argument(
        "--ready-file", help="write the persistent server's loopback address here")
    parser.add_argument(
        "--connect", help="proxy one --input/--output request to HOST:PORT")
    parser.add_argument("--self-test", action="store_true", help="run the worker smoke test")
    args = parser.parse_args()
    if args.self_test:
        return run_self_test()
    if args.serve_loopback:
        if not args.ready_file:
            parser.error("--serve-loopback requires --ready-file")
        return serve_loopback(Path(args.ready_file))
    if args.connect:
        if not args.input or not args.output:
            parser.error("--connect requires --input and --output")
        try:
            return proxy_request(
                args.connect, Path(args.input), Path(args.output))
        except Exception as error:
            Path(args.output).write_text(
                json.dumps(error_result(error), indent=1) + "\n",
                encoding="utf-8")
            return 1
    if not args.input or not args.output:
        parser.error("--input and --output are required")
    request = json.loads(Path(args.input).read_text(encoding="utf-8"))
    try:
        result = run_collision_check(request)
    except Exception as error:  # the daemon needs a parseable failure report
        result = error_result(error)
    Path(args.output).write_text(json.dumps(result, indent=1) + "\n", encoding="utf-8")
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
