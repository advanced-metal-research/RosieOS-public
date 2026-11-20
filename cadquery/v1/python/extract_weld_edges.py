import argparse
import base64
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any


POSITIONER_TABLE_A_AXIS_ORIGIN_MM = [1913.0, 1307.80026, 550.8788]
POSITIONER_TABLE_A_TOP_CENTER_MM = [1913.0, 685.80026, 650.8788]
POSITIONER_TABLE_A_FRAME = {
    "robot": "rosie_1400_v3",
    "axis": "J7a",
    "table": "positioner_table_a",
    "frame": "world",
    "axis_origin_mm": POSITIONER_TABLE_A_AXIS_ORIGIN_MM,
    "origin_mm": POSITIONER_TABLE_A_TOP_CENTER_MM,
    "placement_note": "Dog Box bottom-center is placed on the center-top of positioner table A.",
}
DEFAULT_TCP_OFFSET_MM = [0.0, 0.0, 0.0]
MIN_REAL_CIRCLE_DIAMETER_MM = 75.0

# Tesseract-style motion plan time parameterization (robot/v4 motion-server
# pattern): weld moves run at weld feed, approach/retract/travel moves run at
# the faster travel feed, and every trajectory sample carries time_s + weld_on.
WELD_FEED_MM_S = 12.0
TRAVEL_FEED_MM_S = 90.0
APPROACH_OFFSET_MM = 50.0
APPROACH_SAMPLE_COUNT = 6
TRAVEL_SAMPLE_COUNT = 10
FRAME_SMOOTH_WINDOW = 2

# rosie_1400_v3 J1..J6 kinematic chain, copied from
# robot_description/robots/rosie_1400_v3/robot.urdf (meters, parent-relative origins). The
# fixed end_effector joint places the torch-tip TCP frame; its 45-degree pitch
# bakes the MIG torch neck bend into the flange->TCP transform.
ROBOT_JOINT_CHAIN: list[dict[str, Any]] = [
    {"name": "J1", "origin_m": [0.7874, 0.6858, 0.4808375], "axis": [0.0, 0.0, 1.0], "lower": -6.3, "upper": 6.3},
    {"name": "J2", "origin_m": [0.159999735, -0.125500489, 0.269888605], "axis": [0.0, 1.0, 0.0], "lower": -1.9, "upper": 1.9},
    {"name": "J3", "origin_m": [0.0, 0.0, 0.6], "axis": [0.0, 1.0, 0.0], "lower": -4.2, "upper": 1.53},
    {"name": "J4", "origin_m": [0.159, 0.125499999, 0.105000002], "axis": [1.0, 0.0, 0.0], "lower": -6.3, "upper": 6.3},
    {"name": "J5", "origin_m": [0.4877, -0.047833043, 0.0], "axis": [0.0, 1.0, 0.0], "lower": -6.3, "upper": 6.3},
    {"name": "J6", "origin_m": [0.0, 0.047833043, -0.0933], "axis": [0.0, 0.0, 1.0], "lower": -10.0, "upper": 10.0},
]
ROBOT_JOINT_NAMES = [joint["name"] for joint in ROBOT_JOINT_CHAIN]
ROBOT_TCP_ORIGIN_M = [0.004682724, 0.0, -0.398382724]
ROBOT_TCP_RPY = [3.141592654, 0.785398163, -3.141592654]
ROBOT_TCP_LINK = "end_effector"
IK_HOME_SEED_RAD = [0.0, 0.55, -1.1, 0.0, 0.55, 0.0]

# Real discrete collision checking (robot/v4 pattern): the motion plan's joint
# trajectory is checked by tesseract/v1's BulletDiscreteBVHManager worker
# against the rosie_1400_v3 mesh URDF. Report-only, like robot/v4's table-box
# program: contacts never gate plan generation, they are surfaced to the UI.
COLLISION_CHECK_TIMEOUT_S = 180.0
COLLISION_CHECK_ENV_FLAG = "CADQUERY_SKIP_COLLISION_CHECK"


def load_step(step_path: Path):
    import cadquery as cq

    return cq.importers.importStep(str(step_path))


def shape_from_import(imported):
    if hasattr(imported, "val"):
        return imported.val()
    return imported


def solids_from_shape(shape) -> list[Any]:
    for attr in ("Solids", "solids"):
        try:
            solids = list(getattr(shape, attr)())
            if solids:
                return solids
        except Exception:
            pass
    return [shape]


def edges_from_shape(shape) -> list[Any]:
    for attr in ("Edges", "edges"):
        try:
            edges = list(getattr(shape, attr)())
            if edges:
                return edges
        except Exception:
            pass
    return []


def faces_from_shape(shape) -> list[Any]:
    for attr in ("Faces", "faces"):
        try:
            faces = list(getattr(shape, attr)())
            if faces:
                return faces
        except Exception:
            pass
    return []


def wires_from_face(face) -> list[Any]:
    for attr in ("Wires", "wires"):
        try:
            wires = list(getattr(face, attr)())
            if wires:
                return wires
        except Exception:
            pass
    for attr in ("outerWire", "OuterWire"):
        try:
            wire = getattr(face, attr)()
            if wire is not None:
                return [wire]
        except Exception:
            pass
    return []


def edges_from_wire(wire) -> list[Any]:
    for attr in ("Edges", "edges"):
        try:
            edges = list(getattr(wire, attr)())
            if edges:
                return edges
        except Exception:
            pass
    return []


def bounds(points: list[list[float]]) -> tuple[list[float], list[float]]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    return [min(xs), min(ys), min(zs)], [max(xs), max(ys), max(zs)]


def topology_bounds(topology: dict[str, Any]) -> tuple[list[float], list[float]]:
    all_points = [point for edge in topology.get("edges", []) for point in edge.get("samples", [])]
    if not all_points:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    return bounds(all_points)


def dog_box_local_to_positioner_world(point: list[float], topology: dict[str, Any]) -> list[float]:
    p_min, p_max = topology_bounds(topology)
    bottom_center = [
        (p_min[0] + p_max[0]) * 0.5,
        (p_min[1] + p_max[1]) * 0.5,
        p_min[2],
    ]
    return [
        float(point[0] - bottom_center[0] + POSITIONER_TABLE_A_TOP_CENTER_MM[0]),
        float(point[1] - bottom_center[1] + POSITIONER_TABLE_A_TOP_CENTER_MM[1]),
        float(point[2] - bottom_center[2] + POSITIONER_TABLE_A_TOP_CENTER_MM[2]),
    ]


def dog_box_collision_object(topology: dict[str, Any]) -> dict[str, Any]:
    p_min, p_max = topology_bounds(topology)
    local_center = [
        (p_min[0] + p_max[0]) * 0.5,
        (p_min[1] + p_max[1]) * 0.5,
        (p_min[2] + p_max[2]) * 0.5,
    ]
    world_center = dog_box_local_to_positioner_world(local_center, topology)
    size = [
        float(p_max[0] - p_min[0]),
        float(p_max[1] - p_min[1]),
        float(p_max[2] - p_min[2]),
    ]
    return {
        "name": "dog_box_workpiece",
        "type": "box",
        "frame": "positioner_table_a",
        "parent_link": "positioner_table_a",
        "world_pose_xyz_mm": [round(value, 6) for value in world_center],
        "size_mm": [round(value, 6) for value in size],
        "note": "Collision proxy for the Dog Box workpiece attached to positioner table A for Tesseract planning.",
    }


def tesseract_environment(topology: dict[str, Any]) -> dict[str, Any]:
    return {
        "urdf": "rosie_1400_v3/robot.urdf",
        "workcell_frame": "world",
        "manipulator": "rosie_1400_v3",
        "tcp_frame": "weld_torch_tcp",
        "tcp_offset_mm": DEFAULT_TCP_OFFSET_MM,
        "positioner_axis": "J7a",
        "collision_objects": [dog_box_collision_object(topology)],
        "constraints": {
            "tool_x_axis": "locked_to_seam_tangent",
            "tool_z_axis": "45deg_out_of_seam_plane_away_from_workpiece",
            "tool_y_axis": "right_handed_cross_tool_z_tool_x",
            "tcp_position": "on_backend_sampled_seam_points",
            "collision_margin_mm": 25.0,
        },
    }


def distance(a: list[float], b: list[float]) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def polyline_length(points: list[list[float]]) -> float:
    if len(points) < 2:
        return 0.0
    return sum(distance(points[index - 1], points[index]) for index in range(1, len(points)))


def point_at_distance(points: list[list[float]], lengths: list[float], target: float) -> list[float]:
    if target <= 0:
        return list(points[0])
    total = lengths[-1]
    if target >= total:
        return list(points[-1])
    for index in range(1, len(lengths)):
        if lengths[index] < target:
            continue
        segment_start = lengths[index - 1]
        segment_length = lengths[index] - segment_start or 1.0
        local_t = (target - segment_start) / segment_length
        a = points[index - 1]
        b = points[index]
        return [
            float(a[0] + (b[0] - a[0]) * local_t),
            float(a[1] + (b[1] - a[1]) * local_t),
            float(a[2] + (b[2] - a[2]) * local_t),
        ]
    return list(points[-1])


def sample_polyline_interval(
    points: list[list[float]],
    start_s: float,
    end_s: float,
    count: int,
) -> list[list[float]]:
    if len(points) < 2:
        return points
    lengths = [0.0]
    for index in range(1, len(points)):
        lengths.append(lengths[index - 1] + distance(points[index - 1], points[index]))
    total = lengths[-1]
    if total <= 1e-9:
        return [points[0], points[-1]]
    a = max(0.0, min(1.0, float(start_s)))
    b = max(0.0, min(1.0, float(end_s)))
    start_distance = min(a, b) * total
    end_distance = max(a, b) * total
    sample_count = max(2, int(count))
    out = []
    for index in range(sample_count):
        t = index / (sample_count - 1)
        out.append(point_at_distance(points, lengths, start_distance + (end_distance - start_distance) * t))
    return dedupe_neighbor_points(out)


def interpolate_three_point_arc(control_points: list[list[float]], count: int) -> list[list[float]]:
    if len(control_points) < 3:
        return control_points
    p0, p1, p2 = control_points[0], control_points[1], control_points[2]
    a = sub(p1, p0)
    b = sub(p2, p0)
    normal = cross(a, b)
    normal_len2 = dot(normal, normal)
    if normal_len2 <= 1e-10:
        return sample_polyline_interval([p0, p1, p2], 0.0, 1.0, count)
    normal = normalize_vector(normal)
    u = normalize_vector(a)
    v = normalize_vector(cross(normal, u))
    x1 = dot(a, u)
    y1 = dot(a, v)
    x2 = dot(b, u)
    y2 = dot(b, v)
    denom = 2.0 * (x1 * y2 - y1 * x2)
    if abs(denom) <= 1e-10:
        return sample_polyline_interval([p0, p1, p2], 0.0, 1.0, count)
    cx = (y2 * (x1 * x1 + y1 * y1) - y1 * (x2 * x2 + y2 * y2)) / denom
    cy = (x1 * (x2 * x2 + y2 * y2) - x2 * (x1 * x1 + y1 * y1)) / denom
    center = add(p0, add(scale(u, cx), scale(v, cy)))
    radius = distance(center, p0)
    if radius <= 1e-9:
        return sample_polyline_interval([p0, p1, p2], 0.0, 1.0, count)

    def angle(point: list[float]) -> float:
        rel = sub(point, center)
        return math.atan2(dot(rel, v), dot(rel, u))

    a0, a1, a2 = angle(p0), angle(p1), angle(p2)

    def normalize_positive(delta: float) -> float:
        while delta < 0:
            delta += math.tau
        while delta >= math.tau:
            delta -= math.tau
        return delta

    ccw_mid = normalize_positive(a1 - a0)
    ccw_end = normalize_positive(a2 - a0)
    if ccw_mid <= ccw_end:
        total = ccw_end
    else:
        total = ccw_end - math.tau
    sample_count = max(3, int(count))
    out: list[list[float]] = []
    for index in range(sample_count):
        t = index / (sample_count - 1)
        theta = a0 + total * t
        out.append(add(center, add(scale(u, math.cos(theta) * radius), scale(v, math.sin(theta) * radius))))
    return dedupe_neighbor_points(out)


def dedupe_neighbor_points(points: list[list[float]]) -> list[list[float]]:
    if not points:
        return []
    out = [points[0]]
    for point in points[1:]:
        if distance(out[-1], point) > 1e-9:
            out.append(point)
    return out


def sample_edge_with_ocp(edge: Any, sample_count: int) -> list[list[float]]:
    from OCP.BRepAdaptor import BRepAdaptor_Curve

    wrapped = getattr(edge, "wrapped", edge)
    curve = BRepAdaptor_Curve(wrapped)
    first = float(curve.FirstParameter())
    last = float(curve.LastParameter())
    if not math.isfinite(first) or not math.isfinite(last) or math.isclose(first, last):
        return []

    count = max(2, int(sample_count))
    points: list[list[float]] = []
    for index in range(count):
        t = index / (count - 1)
        param = first + (last - first) * t
        p = curve.Value(param)
        points.append([float(p.X()), float(p.Y()), float(p.Z())])
    return dedupe_neighbor_points(points)


def sample_edge_with_cadquery(edge: Any, sample_count: int) -> list[list[float]]:
    count = max(2, int(sample_count))
    points: list[list[float]] = []
    for index in range(count):
        t = index / (count - 1)
        point = None
        for args in ((t,), (t, "parameter"), (t, "length")):
            try:
                point = edge.positionAt(*args)
                break
            except Exception:
                point = None
        if point is None:
            return []
        points.append([float(point.x), float(point.y), float(point.z)])
    return dedupe_neighbor_points(points)


def sample_edge(edge: Any, sample_count: int) -> list[list[float]]:
    try:
        return sample_edge_with_ocp(edge, sample_count)
    except Exception:
        return sample_edge_with_cadquery(edge, sample_count)


def geom_type(edge: Any) -> str:
    try:
        return str(edge.geomType())
    except Exception:
        return "EDGE"


def shape_hash(edge: Any) -> int | None:
    for target in (edge, getattr(edge, "wrapped", None)):
        if target is None:
            continue
        try:
            if hasattr(target, "hashCode"):
                return int(target.hashCode())
        except Exception:
            pass
        try:
            if hasattr(target, "HashCode"):
                return int(target.HashCode(2147483647))
        except Exception:
            pass
    return None


def fallback_edge_key(samples: list[list[float]]) -> tuple[Any, ...]:
    rounded = [
        tuple(round(coord, 6) for coord in point)
        for point in (samples[:1] + samples[-1:])
    ]
    return (round(polyline_length(samples), 6), *rounded)


def edge_label(index: int) -> str:
    # Spreadsheet-style labels: A..Z, AA..AZ, BA...
    n = int(index)
    out = ""
    while True:
        out = chr(ord("A") + (n % 26)) + out
        n = n // 26 - 1
        if n < 0:
            return out


def endpoint_key(point: list[float]) -> tuple[float, float, float]:
    return (round(float(point[0]), 4), round(float(point[1]), 4), round(float(point[2]), 4))


def orient_samples(samples: list[list[float]], start_key: tuple[float, float, float]) -> list[list[float]]:
    if endpoint_key(samples[0]) == start_key:
        return samples
    if endpoint_key(samples[-1]) == start_key:
        return list(reversed(samples))
    return samples


def order_component_edges(component: list[dict[str, Any]]) -> tuple[list[list[float]], bool]:
    endpoint_to_edges: dict[tuple[float, float, float], list[int]] = {}
    for index, edge in enumerate(component):
        samples = edge.get("samples") or []
        if len(samples) < 2:
            continue
        endpoint_to_edges.setdefault(endpoint_key(samples[0]), []).append(index)
        endpoint_to_edges.setdefault(endpoint_key(samples[-1]), []).append(index)

    degree_one = [key for key, indices in endpoint_to_edges.items() if len(indices) == 1]
    closed = len(degree_one) == 0 and bool(endpoint_to_edges)
    current = degree_one[0] if degree_one else next(iter(endpoint_to_edges))
    used: set[int] = set()
    ordered: list[list[float]] = []

    while len(used) < len(component):
        candidates = [edge_index for edge_index in endpoint_to_edges.get(current, []) if edge_index not in used]
        if not candidates:
            remaining = [index for index in range(len(component)) if index not in used]
            if not remaining:
                break
            next_edge = remaining[0]
            samples = component[next_edge].get("samples") or []
            current = endpoint_key(samples[0])
        else:
            next_edge = candidates[0]
            samples = component[next_edge].get("samples") or []
        if len(samples) < 2:
            used.add(next_edge)
            continue
        oriented = orient_samples(samples, current)
        if ordered and distance(ordered[-1], oriented[0]) <= 1e-6:
            ordered.extend(oriented[1:])
        else:
            ordered.extend(oriented)
        used.add(next_edge)
        current = endpoint_key(oriented[-1])

    if closed and len(ordered) >= 2 and distance(ordered[0], ordered[-1]) > 1e-6:
        ordered.append(list(ordered[0]))
    return dedupe_neighbor_points(ordered), closed


def build_composite_wire_edges(edges: list[dict[str, Any]]) -> list[dict[str, Any]]:
    composites: list[dict[str, Any]] = []
    by_part: dict[str, list[dict[str, Any]]] = {}
    for edge in edges:
        by_part.setdefault(str(edge.get("part_id", "part_0")), []).append(edge)

    for part_id, part_edges in by_part.items():
        endpoint_to_ids: dict[tuple[float, float, float], list[str]] = {}
        edge_by_id = {str(edge["id"]): edge for edge in part_edges}
        for edge in part_edges:
            samples = edge.get("samples") or []
            if len(samples) < 2:
                continue
            endpoint_to_ids.setdefault(endpoint_key(samples[0]), []).append(str(edge["id"]))
            endpoint_to_ids.setdefault(endpoint_key(samples[-1]), []).append(str(edge["id"]))

        seen: set[str] = set()
        for edge in part_edges:
            edge_id = str(edge["id"])
            if edge_id in seen:
                continue
            stack = [edge_id]
            component_ids: list[str] = []
            seen.add(edge_id)
            while stack:
                current_id = stack.pop()
                component_ids.append(current_id)
                samples = edge_by_id[current_id].get("samples") or []
                for key in (endpoint_key(samples[0]), endpoint_key(samples[-1])):
                    for neighbor in endpoint_to_ids.get(key, []):
                        if neighbor not in seen:
                            seen.add(neighbor)
                            stack.append(neighbor)
            if len(component_ids) < 2:
                continue
            component = [edge_by_id[item] for item in component_ids]
            samples, closed = order_component_edges(component)
            if len(samples) < 3:
                continue
            p_min, p_max = bounds(samples)
            composites.append(
                {
                    "id": f"{part_id}:wire_{len(composites):05d}",
                    "part_id": part_id,
                    "index": -1,
                    "local_index": len(composites),
                    "geom_type": "WIRE",
                    "sample_count": len(samples),
                    "samples": samples,
                    "length_mm": polyline_length(samples),
                    "bounds_min": p_min,
                    "bounds_max": p_max,
                    "closed": closed,
                    "source_edge_ids": component_ids,
                }
            )
    return composites


def build_face_wire_edges(shape: Any, part_id: str, sample_count: int, start_index: int = 0) -> list[dict[str, Any]]:
    composites: list[dict[str, Any]] = []
    seen_wire_keys: set[tuple[Any, ...]] = set()
    for face_index, face in enumerate(faces_from_shape(shape)):
        for wire_index, wire in enumerate(wires_from_face(face)):
            component: list[dict[str, Any]] = []
            source_types: list[str] = []
            for edge_index, edge in enumerate(edges_from_wire(wire)):
                samples = sample_edge(edge, sample_count=sample_count)
                if len(samples) < 2:
                    continue
                source_types.append(geom_type(edge))
                component.append(
                    {
                        "id": f"{part_id}:face_{face_index:03d}:wire_{wire_index:03d}:edge_{edge_index:03d}",
                        "part_id": part_id,
                        "samples": samples,
                        "geom_type": geom_type(edge),
                    }
                )
            if len(component) < 2:
                continue
            samples, closed = order_component_edges(component)
            if len(samples) < 3:
                continue
            key = fallback_edge_key(samples)
            if key in seen_wire_keys:
                continue
            seen_wire_keys.add(key)
            p_min, p_max = bounds(samples)
            length = polyline_length(samples)
            source_geom_counts: dict[str, int] = {}
            for source_type in source_types:
                key = "ARC" if source_type.upper() in {"CIRCLE", "ELLIPSE"} else source_type.upper()
                source_geom_counts[key] = source_geom_counts.get(key, 0) + 1
            composites.append(
                {
                    "id": f"{part_id}:wire_{start_index + len(composites):05d}",
                    "part_id": part_id,
                    "index": -1,
                    "local_index": start_index + len(composites),
                    "face_index": face_index,
                    "wire_index": wire_index,
                    "geom_type": "WIRE",
                    "sample_count": len(samples),
                    "samples": samples,
                    "length_mm": length,
                    "bounds_min": p_min,
                    "bounds_max": p_max,
                    "closed": closed,
                    "source_edge_ids": [item["id"] for item in component],
                    "source_geom_types": sorted(set(source_types)),
                    "source_geom_counts": source_geom_counts,
                }
            )
    return composites


def is_circle_like(edge: dict[str, Any]) -> bool:
    if str(edge.get("geom_type", "")).upper() not in {"CIRCLE", "ELLIPSE"}:
        return False
    samples = edge.get("samples") or []
    if len(samples) >= 2 and distance(samples[0], samples[-1]) > 0.5:
        return False
    bounds_min = edge.get("bounds_min") or [0.0, 0.0, 0.0]
    bounds_max = edge.get("bounds_max") or [0.0, 0.0, 0.0]
    diameter = max(abs(bounds_max[index] - bounds_min[index]) for index in range(3))
    return diameter >= MIN_REAL_CIRCLE_DIAMETER_MM


def is_dogbone_wire(edge: dict[str, Any]) -> bool:
    if str(edge.get("geom_type", "")).upper() != "WIRE" or not edge.get("closed"):
        return False
    counts = edge.get("source_geom_counts") or {}
    arc_count = int(counts.get("ARC") or counts.get("CIRCLE") or counts.get("ELLIPSE") or 0)
    line_count = int(counts.get("LINE") or 0)
    return arc_count == 4 and line_count == 4


def classify_edge(edge: dict[str, Any]) -> str:
    if is_circle_like(edge):
        return "circle"
    if is_dogbone_wire(edge):
        return "dogbone_4arc_4line"
    if str(edge.get("geom_type", "")).upper() == "WIRE":
        return "wire_non_seam"
    return "raw_fragment"


def annotate_seam_candidates(edges: list[dict[str, Any]]) -> None:
    for edge in edges:
        edge["path_class"] = classify_edge(edge)
        edge["is_seam_candidate"] = edge["path_class"] in {"circle", "dogbone_4arc_4line"}


def supported_path_methods(edge: dict[str, Any]) -> list[str]:
    if edge.get("path_class") == "circle":
        return ["sampled_edge", "three_point_arc"]
    return ["sampled_edge"]


def resolve_path_method(edge: dict[str, Any], requested: str) -> tuple[str, str | None]:
    normalized = requested if requested in {"sampled_edge", "three_point_arc"} else "sampled_edge"
    supported = supported_path_methods(edge)
    if normalized in supported:
        return normalized, None
    return "sampled_edge", f"{normalized} is only valid for standalone circular paths; using exact sampled CAD path"


def assign_edge_labels(edges: list[dict[str, Any]]) -> None:
    for index, edge in enumerate(edges):
        edge["label"] = edge_label(index)


def normalize_vector(values: list[float]) -> list[float]:
    length = math.sqrt(sum(value * value for value in values))
    if length <= 1e-12:
        return [1.0, 0.0, 0.0]
    return [float(value / length) for value in values]


def dot(a: list[float], b: list[float]) -> float:
    return float(sum(x * y for x, y in zip(a, b)))


def cross(a: list[float], b: list[float]) -> list[float]:
    return [
        float(a[1] * b[2] - a[2] * b[1]),
        float(a[2] * b[0] - a[0] * b[2]),
        float(a[0] * b[1] - a[1] * b[0]),
    ]


def sub(a: list[float], b: list[float]) -> list[float]:
    return [float(a[index] - b[index]) for index in range(3)]


def add(a: list[float], b: list[float]) -> list[float]:
    return [float(a[index] + b[index]) for index in range(3)]


def scale(a: list[float], amount: float) -> list[float]:
    return [float(value * amount) for value in a]


def centroid(points: list[list[float]]) -> list[float]:
    if not points:
        return [0.0, 0.0, 0.0]
    return [
        float(sum(point[axis] for point in points) / len(points))
        for axis in range(3)
    ]


def plane_normal_from_points(points: list[list[float]], fallback: list[float] | None = None) -> list[float]:
    if len(points) < 3:
        return normalize_vector(fallback or [0.0, 0.0, 1.0])
    # Start with representative points across the seam, then fall back to a
    # broader triplet search if the curve segment is nearly collinear.
    triplets: list[tuple[list[float], list[float], list[float]]] = [
        (points[0], points[len(points) // 2], points[-1]),
        (points[0], points[max(1, len(points) // 3)], points[-1]),
        (points[0], points[max(1, (2 * len(points)) // 3)], points[-1]),
    ]
    stride = max(1, len(points) // 8)
    for i in range(0, len(points), stride):
        for j in range(i + 1, len(points), stride):
            for k in range(j + 1, len(points), stride):
                triplets.append((points[i], points[j], points[k]))
    for a, b, c in triplets:
        normal = cross(sub(b, a), sub(c, a))
        if dot(normal, normal) > 1e-8:
            return normalize_vector(normal)
    return normalize_vector(fallback or [0.0, 0.0, 1.0])


def tangent_at(points: list[list[float]], index: int) -> list[float]:
    if len(points) < 2:
        return [1.0, 0.0, 0.0]
    if index <= 0:
        a, b = points[0], points[1]
    elif index >= len(points) - 1:
        a, b = points[-2], points[-1]
    else:
        a, b = points[index - 1], points[index + 1]
    return normalize_vector([b[0] - a[0], b[1] - a[1], b[2] - a[2]])


def is_closed_path(points: list[list[float]]) -> bool:
    if len(points) < 3:
        return False
    span = polyline_length(points)
    if span <= 1e-9:
        return False
    gap = math.sqrt(sum((points[0][axis] - points[-1][axis]) ** 2 for axis in range(3)))
    return gap <= max(1e-6, 0.02 * span)


def smoothed_tangents(points: list[list[float]], window: int = FRAME_SMOOTH_WINDOW) -> list[list[float]]:
    # Windowed average of raw central-difference tangents so line-to-arc corners
    # in the dog-bone path do not snap the torch frame. Closed paths wrap.
    count = len(points)
    if count < 2:
        return [[1.0, 0.0, 0.0] for _ in range(count)]
    raw = [tangent_at(points, index) for index in range(count)]
    closed = is_closed_path(points)
    smoothed: list[list[float]] = []
    for index in range(count):
        acc = [0.0, 0.0, 0.0]
        for offset in range(-window, window + 1):
            neighbor = index + offset
            if closed:
                neighbor %= count
            elif neighbor < 0 or neighbor >= count:
                continue
            weight = 1.0 / (1.0 + abs(offset))
            acc = add(acc, scale(raw[neighbor], weight))
        # Keep tangent forward-consistent with the local raw tangent.
        if dot(acc, raw[index]) < 0.0:
            acc = scale(acc, -1.0)
        smoothed.append(normalize_vector(acc if dot(acc, acc) > 1e-12 else raw[index]))
    return smoothed


def quaternion_from_axes(tool_x: list[float], tool_y: list[float], tool_z: list[float]) -> list[float]:
    # Row-major rotation columns are the tool axes in world frame; return xyzw.
    m00, m01, m02 = tool_x[0], tool_y[0], tool_z[0]
    m10, m11, m12 = tool_x[1], tool_y[1], tool_z[1]
    m20, m21, m22 = tool_x[2], tool_y[2], tool_z[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    return [x / norm, y / norm, z / norm, w / norm]


def quaternion_angle_deg(a: list[float], b: list[float]) -> float:
    d = abs(sum(a[axis] * b[axis] for axis in range(4)))
    d = max(-1.0, min(1.0, d))
    return math.degrees(2.0 * math.acos(d))


def matrix_to_rpy(matrix: list[list[float]]) -> list[float]:
    # Matrix is row-major rotation from tool frame to world frame.
    sy = math.sqrt(matrix[0][0] * matrix[0][0] + matrix[1][0] * matrix[1][0])
    singular = sy < 1e-9
    if not singular:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        pitch = math.atan2(-matrix[2][0], sy)
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        roll = math.atan2(-matrix[1][2], matrix[1][1])
        pitch = math.atan2(-matrix[2][0], sy)
        yaw = 0.0
    return [float(roll), float(pitch), float(yaw)]


def weld_torch_frame(
    tangent: list[float],
    seam_plane_normal: list[float],
    side_sign: float = 1.0,
) -> dict[str, Any]:
    tangent = normalize_vector(tangent)
    normal = normalize_vector(seam_plane_normal)
    side = cross(normal, tangent)
    if dot(side, side) <= 1e-10:
        side = cross([0.0, 0.0, 1.0], tangent)
    if dot(side, side) <= 1e-10:
        side = [0.0, 1.0, 0.0]
    side = normalize_vector(scale(side, side_sign))
    # 45 degrees out of the seam plane: equal normal and in-plane side
    # components. This starts from the orthogonal-to-plane normal and tilts
    # toward the in-plane side vector while keeping the travel tangent as tool X.
    tool_z = normalize_vector(add(scale(normal, math.sqrt(0.5)), scale(side, math.sqrt(0.5))))
    tool_x = normalize_vector(sub(tangent, scale(tool_z, dot(tangent, tool_z))))
    if dot(tool_x, tool_x) <= 1e-10:
        tool_x = tangent
    tool_y = normalize_vector(cross(tool_z, tool_x))
    tool_x = normalize_vector(cross(tool_y, tool_z))
    matrix = [
        [tool_x[0], tool_y[0], tool_z[0]],
        [tool_x[1], tool_y[1], tool_z[1]],
        [tool_x[2], tool_y[2], tool_z[2]],
    ]
    angle_from_plane_deg = math.degrees(math.asin(max(-1.0, min(1.0, abs(dot(tool_z, normal))))))
    tool_x_alignment = dot(tool_x, tangent)
    tool_z_normal_component = dot(tool_z, normal)
    return {
        "tool_x_axis": tool_x,
        "tool_y_axis": tool_y,
        "tool_z_axis": tool_z,
        "seam_plane_normal": normal,
        "seam_side_axis": side,
        "orientation_rpy_rad": matrix_to_rpy(matrix),
        "work_angle_deg": 45.0,
        "angle_from_seam_plane_deg": angle_from_plane_deg,
        "tool_x_tangent_alignment": tool_x_alignment,
        "tool_z_plane_normal_component": tool_z_normal_component,
    }


def axes_from_quaternion(quat: list[float]) -> tuple[list[float], list[float], list[float]]:
    x, y, z, w = quat
    tool_x = [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + z * w), 2.0 * (x * z - y * w)]
    tool_y = [2.0 * (x * y - z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + x * w)]
    tool_z = [2.0 * (x * z + y * w), 2.0 * (y * z - x * w), 1.0 - 2.0 * (x * x + y * y)]
    return normalize_vector(tool_x), normalize_vector(tool_y), normalize_vector(tool_z)


def quaternion_slerp(a: list[float], b: list[float], t: float) -> list[float]:
    d = sum(a[axis] * b[axis] for axis in range(4))
    b = list(b)
    if d < 0.0:
        b = [-value for value in b]
        d = -d
    if d > 0.9995:
        out = [a[axis] + t * (b[axis] - a[axis]) for axis in range(4)]
        norm = math.sqrt(sum(value * value for value in out)) or 1.0
        return [value / norm for value in out]
    theta_0 = math.acos(max(-1.0, min(1.0, d)))
    theta = theta_0 * t
    sin_theta_0 = math.sin(theta_0) or 1e-9
    s0 = math.sin(theta_0 - theta) / sin_theta_0
    s1 = math.sin(theta) / sin_theta_0
    return [s0 * a[axis] + s1 * b[axis] for axis in range(4)]


def rotation_from_axis_angle(axis: list[float], angle: float) -> list[list[float]]:
    x, y, z = normalize_vector(axis)
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def rpy_to_matrix(rpy: list[float]) -> list[list[float]]:
    # URDF fixed-axis convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def mat3_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)]
        for row in range(3)
    ]


def mat3_vec(a: list[list[float]], v: list[float]) -> list[float]:
    return [sum(a[row][k] * v[k] for k in range(3)) for row in range(3)]


def robot_fk_tcp(joint_positions_rad: list[float]) -> tuple[list[list[float]], list[float]]:
    # Forward kinematics of the rosie_1400_v3 J1..J6 chain from the URDF
    # (world frame, meters). Joint origins have zero rpy in this URDF so each
    # step is translate-then-rotate about the joint axis.
    rotation = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    translation = [0.0, 0.0, 0.0]
    for joint, angle in zip(ROBOT_JOINT_CHAIN, joint_positions_rad):
        step = mat3_vec(rotation, joint["origin_m"])
        translation = [translation[axis] + step[axis] for axis in range(3)]
        rotation = mat3_mul(rotation, rotation_from_axis_angle(joint["axis"], float(angle)))
    tcp_step = mat3_vec(rotation, ROBOT_TCP_ORIGIN_M)
    translation = [translation[axis] + tcp_step[axis] for axis in range(3)]
    rotation = mat3_mul(rotation, rpy_to_matrix(ROBOT_TCP_RPY))
    return rotation, translation


def orientation_error_vector(current: list[list[float]], target: list[list[float]]) -> list[float]:
    # Standard axis-error form: 0.5 * sum(cross(current_col_i, target_col_i)).
    error = [0.0, 0.0, 0.0]
    for col in range(3):
        current_col = [current[row][col] for row in range(3)]
        target_col = [target[row][col] for row in range(3)]
        crossed = cross(current_col, target_col)
        error = [error[axis] + 0.5 * crossed[axis] for axis in range(3)]
    return error


def solve_linear_6x6(matrix: list[list[float]], rhs: list[float]) -> list[float]:
    size = len(rhs)
    augmented = [list(matrix[row]) + [rhs[row]] for row in range(size)]
    for col in range(size):
        pivot_row = max(range(col, size), key=lambda row: abs(augmented[row][col]))
        if abs(augmented[pivot_row][col]) < 1e-12:
            continue
        augmented[col], augmented[pivot_row] = augmented[pivot_row], augmented[col]
        pivot = augmented[col][col]
        for row in range(size):
            if row == col:
                continue
            factor = augmented[row][col] / pivot
            for k in range(col, size + 1):
                augmented[row][k] -= factor * augmented[col][k]
    return [
        augmented[row][size] / augmented[row][row] if abs(augmented[row][row]) > 1e-12 else 0.0
        for row in range(size)
    ]


def torch_axis_error_vector(current: list[list[float]], target: list[list[float]]) -> list[float]:
    # Only align tool +Z (torch axis); rotation about the torch axis is a free
    # spin for a symmetric MIG torch, which greatly extends reachable poses.
    # Axis-angle form (axis * angle) instead of a raw cross product so the
    # error does not vanish at the 180-degree anti-parallel saddle.
    current_z = [current[row][2] for row in range(3)]
    target_z = [target[row][2] for row in range(3)]
    axis = cross(current_z, target_z)
    norm = math.sqrt(dot(axis, axis))
    cos_angle = max(-1.0, min(1.0, dot(current_z, target_z)))
    if norm < 1e-9:
        if cos_angle > 0.0:
            return [0.0, 0.0, 0.0]
        fallback = cross(current_z, [1.0, 0.0, 0.0])
        if dot(fallback, fallback) < 1e-9:
            fallback = cross(current_z, [0.0, 1.0, 0.0])
        return scale(normalize_vector(fallback), math.pi)
    angle = math.atan2(norm, cos_angle)
    return scale(axis, angle / norm)


def solve_tcp_ik(
    target_rotation: list[list[float]],
    target_translation_m: list[float],
    seed_rad: list[float],
    max_iterations: int = 80,
    rotation_weight: float = 0.5,
    damping: float = 0.05,
    orientation_mode: str = "full",
) -> dict[str, Any]:
    # Damped-least-squares 6DOF IK with a numeric Jacobian. Warm-started from
    # the previous trajectory sample so consecutive solutions stay continuous.
    q = list(seed_rad)
    epsilon = 1e-5
    orientation_error = orientation_error_vector if orientation_mode == "full" else torch_axis_error_vector

    def pose_error(joints: list[float]) -> list[float]:
        rotation, translation = robot_fk_tcp(joints)
        position_error = [target_translation_m[axis] - translation[axis] for axis in range(3)]
        rotation_error = orientation_error(rotation, target_rotation)
        return position_error + [rotation_weight * value for value in rotation_error]

    error = pose_error(q)
    for _ in range(max_iterations):
        position_norm = math.sqrt(sum(value * value for value in error[:3]))
        rotation_norm = math.sqrt(sum(value * value for value in error[3:]))
        if position_norm < 2e-5 and rotation_norm < 2e-4:
            break
        jacobian: list[list[float]] = [[0.0] * 6 for _ in range(6)]
        for joint_index in range(6):
            perturbed = list(q)
            perturbed[joint_index] += epsilon
            perturbed_error = pose_error(perturbed)
            for row in range(6):
                # d(error)/dq: error shrinks as the joint moves toward target.
                jacobian[row][joint_index] = (error[row] - perturbed_error[row]) / epsilon
        jjt = [
            [sum(jacobian[row][k] * jacobian[col][k] for k in range(6)) + (damping * damping if row == col else 0.0) for col in range(6)]
            for row in range(6)
        ]
        y = solve_linear_6x6(jjt, error)
        dq = [sum(jacobian[row][joint_index] * y[row] for row in range(6)) for joint_index in range(6)]
        max_step = max(abs(value) for value in dq) or 1.0
        step_scale = min(1.0, 0.35 / max_step)
        for joint_index, joint in enumerate(ROBOT_JOINT_CHAIN):
            q[joint_index] = max(float(joint["lower"]), min(float(joint["upper"]), q[joint_index] + dq[joint_index] * step_scale))
        error = pose_error(q)

    rotation, translation = robot_fk_tcp(q)
    position_error_m = math.sqrt(sum((target_translation_m[axis] - translation[axis]) ** 2 for axis in range(3)))
    rotation_error_vec = orientation_error_vector(rotation, target_rotation)
    rotation_error_rad = 2.0 * math.asin(max(-1.0, min(1.0, math.sqrt(sum(value * value for value in rotation_error_vec)))))
    current_z = [rotation[row][2] for row in range(3)]
    target_z = [target_rotation[row][2] for row in range(3)]
    torch_axis_error_rad = math.acos(max(-1.0, min(1.0, dot(current_z, target_z))))
    return {
        "joint_positions_rad": q,
        "position_error_mm": position_error_m * 1000.0,
        "rotation_error_deg": math.degrees(rotation_error_rad),
        "torch_axis_error_deg": math.degrees(torch_axis_error_rad),
        "orientation_mode": orientation_mode,
    }


def rotation_matrix_from_tool_axes(tool_x: list[float], tool_y: list[float], tool_z: list[float]) -> list[list[float]]:
    return [
        [tool_x[0], tool_y[0], tool_z[0]],
        [tool_x[1], tool_y[1], tool_z[1]],
        [tool_x[2], tool_y[2], tool_z[2]],
    ]


def append_joint_solutions(trajectory: list[dict[str, Any]]) -> dict[str, Any]:
    # Solve IK for every trajectory sample so the UI can drive the URDF robot
    # joints through the plan (robot/v4 pattern: the plan trajectory carries
    # the joint solution alongside the TCP pose). Orientation is constrained
    # to the torch axis (tool +Z) only; the spin about a symmetric MIG torch
    # is free, which keeps the whole dog-bone seam inside the arm's reach.
    seed = list(IK_HOME_SEED_RAD)
    max_position_error = 0.0
    max_torch_axis_error = 0.0
    position_error_sum = 0.0
    max_joint_step = 0.0
    previous: list[float] | None = None
    for sample in trajectory:
        tool_x, tool_y, tool_z = axes_from_quaternion([float(v) for v in sample["tool_quat_xyzw"]])
        target_rotation = rotation_matrix_from_tool_axes(tool_x, tool_y, tool_z)
        target_translation = [float(coord) for coord in sample["target_xyz_m"]]
        solution = solve_tcp_ik(target_rotation, target_translation, seed, orientation_mode="torch_axis")
        if solution["position_error_mm"] > 5.0:
            # Retry from the home seed in case the warm start walked into a
            # bad local minimum near a joint limit.
            retry = solve_tcp_ik(target_rotation, target_translation, IK_HOME_SEED_RAD, max_iterations=160, orientation_mode="torch_axis")
            if retry["position_error_mm"] < solution["position_error_mm"]:
                solution = retry
        seed = solution["joint_positions_rad"]
        sample["joint_names"] = ROBOT_JOINT_NAMES
        sample["joint_positions_rad"] = [round(value, 6) for value in solution["joint_positions_rad"]]
        sample["ik_position_error_mm"] = round(solution["position_error_mm"], 4)
        sample["ik_rotation_error_deg"] = round(solution["torch_axis_error_deg"], 4)
        max_position_error = max(max_position_error, solution["position_error_mm"])
        max_torch_axis_error = max(max_torch_axis_error, solution["torch_axis_error_deg"])
        position_error_sum += solution["position_error_mm"]
        if previous is not None:
            max_joint_step = max(
                max_joint_step,
                max(abs(seed[index] - previous[index]) for index in range(6)),
            )
        previous = list(seed)
    count = max(1, len(trajectory))
    return {
        "solver": "damped_least_squares",
        "robot": "rosie_1400_v3",
        "tcp_link": ROBOT_TCP_LINK,
        "joint_names": ROBOT_JOINT_NAMES,
        "orientation_constraint": "torch_axis",
        "max_position_error_mm": round(max_position_error, 4),
        "mean_position_error_mm": round(position_error_sum / count, 4),
        "max_rotation_error_deg": round(max_torch_axis_error, 4),
        "max_joint_step_rad": round(max_joint_step, 6),
    }


def gradient_repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def find_pixi_executable() -> str | None:
    exe = shutil.which("pixi")
    if exe:
        return exe
    name = "pixi.exe" if os.name == "nt" else "pixi"
    candidate = Path.home() / ".rosie" / "bin" / name
    if candidate.is_file():
        return str(candidate)
    return None


def run_tesseract_collision_check(trajectory: list[dict[str, Any]]) -> dict[str, Any]:
    # Discrete BulletDiscreteBVHManager contact check over every trajectory
    # sample, via the shared tesseract/v1 worker (the same worker the robot/v4
    # motion-server uses). Report-only: failures and missing environments are
    # reported, never raised, so seam planning keeps working without the
    # tesseract env. Contacts are attached to their trajectory samples as
    # collision_contacts=[{link_a, link_b, depth_mm}].
    if os.environ.get(COLLISION_CHECK_ENV_FLAG, "").strip() == "1":
        return {"status": "skipped", "reason": f"{COLLISION_CHECK_ENV_FLAG}=1"}
    repo = gradient_repo_root()
    urdf_path = repo / "robot_description" / "robots" / "rosie_1400_v3" / "robot.urdf"
    worker = repo / "tesseract" / "v1" / "python" / "collision_check.py"
    manifest = repo / "tesseract" / "v1" / "pixi.toml"
    if not urdf_path.is_file() or not worker.is_file() or not manifest.is_file():
        return {"status": "unavailable", "reason": "rosie_1400_v3 URDF or tesseract/v1 worker not found"}
    pixi = find_pixi_executable()
    if not pixi:
        return {"status": "unavailable", "reason": "pixi executable not found"}
    if not all(isinstance(sample.get("joint_positions_rad"), list) for sample in trajectory):
        return {"status": "unavailable", "reason": "trajectory has no joint solutions"}

    request = {
        "schema": "robot-v4.tesseract-collision-check.v1",
        "urdf_path": str(urdf_path),
        "joint_names": list(ROBOT_JOINT_NAMES),
        "contact_distance_m": 0.0,
        "waypoints": [
            {
                "waypoint_index": int(sample["index"]),
                "time_s": float(sample["time_s"]),
                "positions_rad": [float(v) for v in sample["joint_positions_rad"]],
            }
            for sample in trajectory
        ],
    }
    tmp_dir = Path(tempfile.mkdtemp(prefix="cadquery-collision-"))
    request_path = tmp_dir / "request.json"
    response_path = tmp_dir / "response.json"
    request_path.write_text(json.dumps(request), encoding="utf-8")
    # Nested pixi run: strip the cadquery env's pixi/conda context so the
    # tesseract env activates cleanly.
    env = {k: v for k, v in os.environ.items() if not k.startswith(("PIXI_", "CONDA_"))}
    try:
        proc = subprocess.run(
            [pixi, "run", "--manifest-path", str(manifest), "python", "python/collision_check.py",
             "--input", str(request_path), "--output", str(response_path)],
            cwd=str(repo / "tesseract" / "v1"),
            env=env,
            capture_output=True,
            timeout=COLLISION_CHECK_TIMEOUT_S,
        )
        if not response_path.is_file():
            stderr_tail = proc.stderr.decode("utf-8", "replace")[-400:]
            return {"status": "unavailable", "reason": f"collision worker wrote no result (exit {proc.returncode}): {stderr_tail}"}
        result = json.loads(response_path.read_text(encoding="utf-8"))
    except subprocess.TimeoutExpired:
        return {"status": "unavailable", "reason": f"collision worker timed out after {COLLISION_CHECK_TIMEOUT_S}s"}
    except Exception as exc:
        return {"status": "unavailable", "reason": f"{type(exc).__name__}: {exc}"}
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    if not result.get("ok"):
        return {"status": "unavailable", "reason": str(result.get("error") or "collision worker reported failure")}

    contacts_by_index: dict[int, list[dict[str, Any]]] = {}
    max_depth_mm = 0.0
    pairs: set[str] = set()
    for contact in result.get("contacts", []):
        index = int(contact["waypoint_index"])
        depth_mm = float(contact["depth_m"]) * 1000.0
        entry = {
            "link_a": str(contact["link_a"]),
            "link_b": str(contact["link_b"]),
            "depth_mm": round(depth_mm, 3),
        }
        contacts_by_index.setdefault(index, []).append(entry)
        max_depth_mm = max(max_depth_mm, depth_mm)
        pairs.add(f"{contact['link_a']}|{contact['link_b']}")
    for sample in trajectory:
        contacts = contacts_by_index.get(int(sample["index"]))
        if contacts:
            sample["collision_contacts"] = contacts
    return {
        "status": "checked",
        "checker": str(result.get("checker", "")),
        "collision_geometry_source": str(result.get("collision_geometry_source", "")),
        "checked_waypoint_count": int(result.get("checked_waypoint_count", 0)),
        "contact_count": int(result.get("contact_count", 0)),
        "contact_sample_count": len(contacts_by_index),
        "max_depth_mm": round(max_depth_mm, 3),
        "contact_pairs": sorted(pairs),
        "check_duration_s": round(float(result.get("check_duration_s", 0.0)), 3),
        "note": "Report-only, robot/v4 pattern: contacts are surfaced to the UI and never gate plan generation. The dog_box workpiece is not yet part of the checked URDF.",
    }


def pose_frame_fields(
    tool_x: list[float],
    tool_y: list[float],
    tool_z: list[float],
    normal: list[float],
    side: list[float],
    tangent: list[float],
) -> dict[str, Any]:
    matrix = [
        [tool_x[0], tool_y[0], tool_z[0]],
        [tool_x[1], tool_y[1], tool_z[1]],
        [tool_x[2], tool_y[2], tool_z[2]],
    ]
    angle_from_plane_deg = math.degrees(math.asin(max(-1.0, min(1.0, abs(dot(tool_z, normal))))))
    return {
        "seam_tangent": [round(value, 9) for value in tangent],
        "seam_plane_normal": [round(value, 9) for value in normal],
        "seam_side_axis": [round(value, 9) for value in side],
        "tool_x_axis": [round(value, 9) for value in tool_x],
        "tool_y_axis": [round(value, 9) for value in tool_y],
        "tool_z_axis": [round(value, 9) for value in tool_z],
        "tool_quat_xyzw": [round(value, 9) for value in quaternion_from_axes(tool_x, tool_y, tool_z)],
        "orientation_rpy_rad": [round(value, 9) for value in matrix_to_rpy(matrix)],
        "work_angle_deg": 45.0,
        "angle_from_seam_plane_deg": round(angle_from_plane_deg, 6),
        "tool_x_tangent_alignment": round(dot(tool_x, tangent), 9),
        "tool_z_plane_normal_component": round(dot(tool_z, normal), 9),
    }


def line_samples(start: list[float], stop: list[float], count: int) -> list[list[float]]:
    count = max(1, int(count))
    return [
        [start[axis] + (stop[axis] - start[axis]) * (step / count) for axis in range(3)]
        for step in range(count + 1)
    ]


def build_motion_trajectory(seams: list[dict[str, Any]]) -> dict[str, Any]:
    # Dense, time-parameterized Tesseract-style plan following the robot/v4
    # motion-server pattern: each seam plays approach -> weld -> retract, with
    # travel moves stitched between seams. Weld moves run at the weld feed;
    # approach/retract/travel run at the faster travel feed. Torch frames are
    # smoothed along the seam and slerped across travel so nothing snaps.
    trajectory: list[dict[str, Any]] = []
    metrics = {
        "time_s": 0.0,
        "weld_len": 0.0,
        "travel_len": 0.0,
        "max_step": 0.0,
        "max_rot": 0.0,
        "prev_point": None,
        "prev_quat": None,
        "prev_weld_on": None,
    }

    def emit(point: list[float], fields: dict[str, Any], seam_id: str, move: str, weld_on: bool) -> None:
        quat = [float(value) for value in fields["tool_quat_xyzw"]]
        prev_point = metrics["prev_point"]
        if prev_point is not None:
            step = math.sqrt(sum((point[axis] - prev_point[axis]) ** 2 for axis in range(3)))
            # Drop duplicate points that share a location with the previous
            # sample, but keep a coincident sample when it flips the weld state
            # (approach->weld / weld->retract boundary) so weld runs stay intact.
            if step <= 1e-9 and metrics["prev_weld_on"] == weld_on:
                return
            feed = WELD_FEED_MM_S if weld_on else TRAVEL_FEED_MM_S
            metrics["time_s"] += step / feed
            metrics["max_step"] = max(metrics["max_step"], step)
            if weld_on:
                metrics["weld_len"] += step
            else:
                metrics["travel_len"] += step
            if metrics["prev_quat"] is not None:
                metrics["max_rot"] = max(metrics["max_rot"], quaternion_angle_deg(metrics["prev_quat"], quat))
        pose = {
            "index": len(trajectory),
            "seam_id": seam_id,
            "move": move,
            "weld_on": weld_on,
            "time_s": round(metrics["time_s"], 4),
            "target_xyz_m": [round(coord / 1000.0, 9) for coord in point],
            "target_xyz_mm": [round(coord, 6) for coord in point],
        }
        pose.update(fields)
        trajectory.append(pose)
        metrics["prev_point"] = list(point)
        metrics["prev_quat"] = quat
        metrics["prev_weld_on"] = weld_on

    per_seam_frame_rotation: list[float] = []
    prev_retract: list[float] | None = None
    prev_stop_quat: list[float] | None = None

    for seam in seams:
        seam_id = str(seam["seam_id"])
        world_points = seam["world_points"]
        normal = seam["normal"]
        side_sign = float(seam["side_sign"])
        tangents = smoothed_tangents(world_points)
        weld_frames = [weld_torch_frame(tangents[index], normal, side_sign=side_sign) for index in range(len(world_points))]
        weld_fields = [
            pose_frame_fields(
                frame["tool_x_axis"],
                frame["tool_y_axis"],
                frame["tool_z_axis"],
                frame["seam_plane_normal"],
                frame["seam_side_axis"],
                tangents[index],
            )
            for index, frame in enumerate(weld_frames)
        ]
        # Per-seam smoothness score: max frame rotation between weld poses.
        seam_rot = 0.0
        for index in range(1, len(weld_fields)):
            seam_rot = max(
                seam_rot,
                quaternion_angle_deg(
                    [float(v) for v in weld_fields[index - 1]["tool_quat_xyzw"]],
                    [float(v) for v in weld_fields[index]["tool_quat_xyzw"]],
                ),
            )
        per_seam_frame_rotation.append(seam_rot)

        start = world_points[0]
        stop = world_points[-1]
        approach = [start[0], start[1], start[2] + APPROACH_OFFSET_MM]
        retract = [stop[0], stop[1], stop[2] + APPROACH_OFFSET_MM]
        start_fields = weld_fields[0]
        stop_fields = weld_fields[-1]
        start_quat = [float(v) for v in start_fields["tool_quat_xyzw"]]
        stop_quat = [float(v) for v in stop_fields["tool_quat_xyzw"]]

        # Travel move from the previous seam's retract to this approach, with a
        # slerped torch orientation so the frame glides between seams.
        if prev_retract is not None and prev_stop_quat is not None:
            travel_pts = line_samples(prev_retract, approach, TRAVEL_SAMPLE_COUNT)
            for step_index, point in enumerate(travel_pts):
                t = step_index / max(1, len(travel_pts) - 1)
                quat = quaternion_slerp(prev_stop_quat, start_quat, t)
                tool_x, tool_y, tool_z = axes_from_quaternion(quat)
                travel_tangent = normalize_vector(sub(approach, prev_retract))
                fields = pose_frame_fields(tool_x, tool_y, tool_z, normal, weld_frames[0]["seam_side_axis"], travel_tangent)
                emit(point, fields, seam_id, "travel", False)

        # Approach: straight descent to the weld start, holding start frame.
        approach_tangent = normalize_vector(sub(start, approach))
        for point in line_samples(approach, start, APPROACH_SAMPLE_COUNT):
            fields = dict(start_fields)
            fields["seam_tangent"] = [round(value, 9) for value in approach_tangent]
            emit(point, fields, seam_id, "approach", False)

        # Weld: dense seam trace with smoothed per-point frames.
        for index, point in enumerate(world_points):
            emit(point, weld_fields[index], seam_id, "weld", True)

        # Retract: straight climb away from the weld stop, holding stop frame.
        retract_tangent = normalize_vector(sub(retract, stop))
        for point in line_samples(stop, retract, APPROACH_SAMPLE_COUNT):
            fields = dict(stop_fields)
            fields["seam_tangent"] = [round(value, 9) for value in retract_tangent]
            emit(point, fields, seam_id, "retract", False)

        prev_retract = retract
        prev_stop_quat = stop_quat

    weld_samples = sum(1 for pose in trajectory if pose["weld_on"])
    ik_summary = append_joint_solutions(trajectory)
    collision_summary = run_tesseract_collision_check(trajectory)
    return {
        "trajectory": trajectory,
        "sample_count": len(trajectory),
        "weld_sample_count": weld_samples,
        "duration_s": round(metrics["time_s"], 4),
        "tool_path_length_mm": round(metrics["weld_len"] + metrics["travel_len"], 6),
        "weld_length_mm": round(metrics["weld_len"], 6),
        "travel_length_mm": round(metrics["travel_len"], 6),
        "max_tcp_step_mm": round(metrics["max_step"], 6),
        "max_frame_rotation_deg": round(metrics["max_rot"], 6),
        "per_seam_frame_rotation_deg": [round(value, 6) for value in per_seam_frame_rotation],
        "weld_feed_mm_s": WELD_FEED_MM_S,
        "travel_feed_mm_s": TRAVEL_FEED_MM_S,
        "ik": ik_summary,
        "collision": collision_summary,
    }


def seam_frame_rotation_deg(world_points: list[list[float]], normal: list[float], side_sign: float) -> float:
    tangents = smoothed_tangents(world_points)
    quats = [
        quaternion_from_axes(frame["tool_x_axis"], frame["tool_y_axis"], frame["tool_z_axis"])
        for frame in (weld_torch_frame(tangents[index], normal, side_sign=side_sign) for index in range(len(world_points)))
    ]
    max_rot = 0.0
    for index in range(1, len(quats)):
        max_rot = max(max_rot, quaternion_angle_deg(quats[index - 1], quats[index]))
    return max_rot


def choose_weld_geometry_method(
    world_points: list[list[float]],
    world_center: list[float],
    requested_id: str | None = None,
) -> dict[str, Any]:
    seam_center = centroid(world_points)
    center_index = max(0, min(len(world_points) - 1, len(world_points) // 2))
    tangent = tangent_at(world_points, center_index)
    base_normal = plane_normal_from_points(world_points, fallback=[0.0, 0.0, 1.0])
    away_from_part = normalize_vector(sub(seam_center, world_center))
    candidates: list[dict[str, Any]] = []
    for normal_sign in (1.0, -1.0):
        normal = normalize_vector(scale(base_normal, normal_sign))
        for side_sign in (1.0, -1.0):
            frame = weld_torch_frame(tangent, normal, side_sign=side_sign)
            away_score = dot(frame["tool_z_axis"], away_from_part)
            upward_score = frame["tool_z_axis"][2]
            x_error = max(0.0, 1.0 - float(frame["tool_x_tangent_alignment"]))
            angle_error = abs(float(frame["angle_from_seam_plane_deg"]) - 45.0)
            # Frame smoothness: max torch rotation between neighboring seam poses
            # for this candidate. Penalizing it keeps "optimal" strategies smooth
            # enough to trace without the torch snapping at line-to-arc corners.
            frame_rotation_deg = seam_frame_rotation_deg(world_points, normal, side_sign)
            # Geometry-only score: preserve constraints first, then prefer the
            # torch axis pointing away from the workpiece center and upward, and
            # finally prefer a smooth frame along the path.
            score = (
                angle_error * 10.0
                + x_error * 1000.0
                + max(0.0, -away_score) * 8.0
                + max(0.0, -upward_score) * 3.0
                - max(0.0, away_score) * 2.0
                - max(0.0, upward_score) * 0.75
                + frame_rotation_deg * 0.6
            )
            candidates.append(
                {
                    "id": f"normal_{'base' if normal_sign > 0 else 'flipped'}__side_{'positive' if side_sign > 0 else 'negative'}",
                    "normal_sign": normal_sign,
                    "side_sign": side_sign,
                    "score": round(score, 9),
                    "away_from_part": round(away_score, 9),
                    "upward": round(upward_score, 9),
                    "tool_x_tangent_alignment": round(float(frame["tool_x_tangent_alignment"]), 9),
                    "angle_from_seam_plane_deg": round(float(frame["angle_from_seam_plane_deg"]), 6),
                    "max_frame_rotation_deg": round(frame_rotation_deg, 6),
                    "seam_plane_normal": [round(value, 9) for value in frame["seam_plane_normal"]],
                    "tool_z_axis": [round(value, 9) for value in frame["tool_z_axis"]],
                }
            )
    candidates.sort(key=lambda item: float(item["score"]))
    winner = next((item for item in candidates if item["id"] == requested_id), candidates[0])
    return {
        "schema": "cadquery.v1.weld-geometry-method",
        "selected": winner["id"],
        "normal_sign": winner["normal_sign"],
        "side_sign": winner["side_sign"],
        "selection_reason": (
            "user-selected candidate"
            if requested_id and winner["id"] == requested_id
            else "lowest score preserving +X tangent and 45deg plane angle while pointing tool +Z away from the workpiece"
        ),
        "seam_center_mm": [round(value, 6) for value in seam_center],
        "part_center_mm": [round(value, 6) for value in world_center],
        "candidates": candidates,
    }


def build_tesseract_weld_plan(
    topology: dict[str, Any],
    edge: dict[str, Any],
    local_points: list[list[float]],
    geometry_method_id: str | None = None,
    path_method: str = "sampled_edge",
    control_points: list[list[float]] | None = None,
    path_method_reason: str | None = None,
) -> dict[str, Any]:
    world_points = [dog_box_local_to_positioner_world(point, topology) for point in local_points]
    local_center = centroid([point for item in topology.get("edges", []) for point in item.get("samples", [])])
    world_center = dog_box_local_to_positioner_world(local_center, topology)
    seam_plane_normal = plane_normal_from_points(world_points, fallback=[0.0, 0.0, 1.0])
    geometry_method = choose_weld_geometry_method(world_points, world_center, geometry_method_id)
    selected_normal = normalize_vector(scale(seam_plane_normal, float(geometry_method["normal_sign"])))
    selected_side_sign = float(geometry_method["side_sign"])
    tcp_poses = []
    for index, point in enumerate(world_points):
        tangent = tangent_at(world_points, index)
        torch = weld_torch_frame(tangent, selected_normal, side_sign=selected_side_sign)
        tcp_poses.append(
            {
                "index": index,
                "time_s": round(index * 0.18, 4),
                "target_xyz_m": [round(coord / 1000.0, 9) for coord in point],
                "target_xyz_mm": [round(coord, 6) for coord in point],
                "seam_tangent": [round(value, 9) for value in tangent],
                "seam_plane_normal": [round(value, 9) for value in torch["seam_plane_normal"]],
                "seam_side_axis": [round(value, 9) for value in torch["seam_side_axis"]],
                "tool_x_axis": [round(value, 9) for value in torch["tool_x_axis"]],
                "tool_y_axis": [round(value, 9) for value in torch["tool_y_axis"]],
                "tool_z_axis": [round(value, 9) for value in torch["tool_z_axis"]],
                "work_angle_deg": torch["work_angle_deg"],
                "angle_from_seam_plane_deg": round(torch["angle_from_seam_plane_deg"], 6),
                "tool_x_tangent_alignment": round(torch["tool_x_tangent_alignment"], 9),
                "tool_z_plane_normal_component": round(torch["tool_z_plane_normal_component"], 9),
                "orientation_rpy_rad": [round(value, 9) for value in torch["orientation_rpy_rad"]],
                "geometry_method": geometry_method["selected"],
            }
        )
    start = world_points[0]
    stop = world_points[-1]
    approach = [start[0], start[1], start[2] + APPROACH_OFFSET_MM]
    retract = [stop[0], stop[1], stop[2] + APPROACH_OFFSET_MM]
    motion_plan = build_motion_trajectory(
        [
            {
                "seam_id": edge["id"],
                "world_points": world_points,
                "normal": selected_normal,
                "side_sign": selected_side_sign,
            }
        ]
    )
    return {
        "schema": "cadquery.v1.tesseract-weld-plan",
        "planner": "tesseract",
        "robot": "rosie_1400_v3",
        "working_frame": "world",
        "positioner": POSITIONER_TABLE_A_FRAME,
        "environment": tesseract_environment(topology),
        "geometry_method": geometry_method,
        "path_method": {
            "selected": path_method,
            "supported": supported_path_methods(edge),
            "fallback_reason": path_method_reason,
            "control_pose_count": len(control_points or []),
            "control_points": [[round(float(coord), 6) for coord in point] for point in (control_points or [])],
        },
        "edge_id": edge["id"],
        "sequence": [
            {"kind": "approach", "target_xyz_mm": [round(v, 6) for v in approach]},
            {"kind": "weld_start", "target_xyz_mm": [round(v, 6) for v in start]},
            {"kind": "weld_stop", "target_xyz_mm": [round(v, 6) for v in stop]},
            {"kind": "retract", "target_xyz_mm": [round(v, 6) for v in retract]},
        ],
        "tcp_poses": tcp_poses,
        "motion_plan": motion_plan,
    }


def build_seam_payload(
    topology: dict[str, Any],
    edge_id: str,
    start_s: float,
    end_s: float,
    seam_sample_count: int,
    reverse: bool,
    geometry_method_id: str | None = None,
    path_method: str = "sampled_edge",
) -> dict[str, Any]:
    edge = next((item for item in topology["edges"] if item["id"] == edge_id), None)
    if edge is None:
        raise KeyError(f"edge_id '{edge_id}' not found")
    resolved_path_method, path_method_reason = resolve_path_method(edge, path_method)
    control_points = sample_polyline_interval(edge["samples"], start_s, end_s, 3)
    if resolved_path_method == "three_point_arc":
        points = interpolate_three_point_arc(control_points, seam_sample_count)
    else:
        points = sample_polyline_interval(edge["samples"], start_s, end_s, seam_sample_count)
    if reverse:
        points = list(reversed(points))
        control_points = list(reversed(control_points))
    world_points = [dog_box_local_to_positioner_world(point, topology) for point in points]
    world_control_points = [dog_box_local_to_positioner_world(point, topology) for point in control_points]
    tesseract_plan = build_tesseract_weld_plan(
        topology,
        edge,
        points,
        geometry_method_id,
        resolved_path_method,
        world_control_points,
        path_method_reason,
    )
    return {
        "schema_version": 1,
        "tool": "rosie cadquery v1 seam backend",
        "model_id": topology["model_id"],
        "filename": topology["filename"],
        "fingerprint": topology["fingerprint"],
        "unit": "mm",
        "workflow": {
            "source": "cadquery",
            "stage": "edge_to_weld_seam",
            "tour": "dog_box" if topology["filename"] == "dog_box.step" else None,
            "computed_by": "python/cadquery",
        },
        "placement": POSITIONER_TABLE_A_FRAME,
        "environment": tesseract_environment(topology),
        "seam": {
            "edge_id": edge["id"],
            "part_id": edge.get("part_id"),
            "geom_type": edge.get("geom_type", "EDGE"),
            "start_s": round(max(0.0, min(1.0, float(start_s))), 4),
            "end_s": round(max(0.0, min(1.0, float(end_s))), 4),
            "reversed": bool(reverse),
            "length_mm": edge.get("length_mm"),
            "point_count": len(points),
            "points": [[round(float(coord), 6) for coord in point] for point in points],
            "control_points": [[round(float(coord), 6) for coord in point] for point in control_points],
            "world_points_mm": [[round(float(coord), 6) for coord in point] for point in world_points],
            "world_control_points_mm": [[round(float(coord), 6) for coord in point] for point in world_control_points],
            "start_point_mm": [round(float(coord), 6) for coord in world_points[0]],
            "stop_point_mm": [round(float(coord), 6) for coord in world_points[-1]],
            "positioner_axis": "J7a",
        },
        "weld_sequence": {
            "schema": "cadquery.v1.weld-sequence",
            "robot": "rosie_1400_v3",
            "frame": "world",
            "positioner_axis": "J7a",
            "seams": [
                {
                    "id": "seam_01",
                    "edge_id": edge["id"],
                    "start_point_mm": [round(float(coord), 6) for coord in world_points[0]],
                    "stop_point_mm": [round(float(coord), 6) for coord in world_points[-1]],
                    "point_count": len(world_points),
                }
            ],
        },
        "tesseract_plan": tesseract_plan,
    }


def recommended_seam(topology: dict[str, Any]) -> dict[str, Any]:
    recommendations = recommended_seams(topology, limit=1)
    if recommendations:
        return recommendations[0]
    candidates = [edge for edge in topology["edges"] if float(edge.get("length_mm") or 0.0) > 20.0]
    if not candidates:
        candidates = list(topology["edges"])
    edge = max(candidates, key=lambda item: float(item.get("length_mm") or 0.0))
    return {
        "edge_id": edge["id"],
        "part_id": edge.get("part_id"),
        "geom_type": edge.get("geom_type", "EDGE"),
        "start_s": 0.12,
        "end_s": 0.88,
        "reversed": False,
        "reason": "longest usable sampled topology edge",
    }


def recommended_seams(topology: dict[str, Any], limit: int = 10) -> list[dict[str, Any]]:
    candidates = [
        edge
        for edge in topology["edges"]
        if edge.get("is_seam_candidate") and float(edge.get("length_mm") or 0.0) > 20.0
    ]
    if not candidates:
        candidates = [edge for edge in topology["edges"] if float(edge.get("length_mm") or 0.0) > 20.0]
    class_rank = {"dogbone_4arc_4line": 3, "circle": 2}
    candidates.sort(
        key=lambda item: (
            class_rank.get(str(item.get("path_class", "")), 0),
            float(item.get("length_mm") or 0.0),
        ),
        reverse=True,
    )
    out: list[dict[str, Any]] = []
    for edge in candidates[: max(1, int(limit))]:
        reason = (
            "closed dog-bone seam path: 4 arc/circle segments plus 4 line segments"
            if edge.get("path_class") == "dogbone_4arc_4line"
            else "standalone closed circular seam path"
            if edge.get("path_class") == "circle"
            else "fallback sampled topology edge"
        )
        out.append(
            {
                "edge_id": edge["id"],
                "part_id": edge.get("part_id"),
                "geom_type": edge.get("geom_type", "EDGE"),
                "path_class": edge.get("path_class", "raw_fragment"),
                "start_s": 0.12,
                "end_s": 0.88,
                "reversed": False,
                "reason": reason,
            }
        )
    return out


def build_weld_sequence_payload(
    topology: dict[str, Any],
    segments: list[dict[str, Any]],
    seam_sample_count: int,
) -> dict[str, Any]:
    seam_records: list[dict[str, Any]] = []
    tcp_poses: list[dict[str, Any]] = []
    sequence: list[dict[str, Any]] = []
    motion_seams: list[dict[str, Any]] = []
    time_s = 0.0
    local_center = centroid([point for item in topology.get("edges", []) for point in item.get("samples", [])])
    world_center = dog_box_local_to_positioner_world(local_center, topology)

    for segment_index, segment in enumerate(segments):
        edge_id = str(segment.get("edge_id") or segment.get("edgeId") or "").strip()
        if not edge_id:
            continue
        edge = next((item for item in topology["edges"] if item["id"] == edge_id), None)
        if edge is None:
            raise KeyError(f"edge_id '{edge_id}' not found")
        start_s = float(segment.get("start_s", segment.get("startS", 0.12)))
        end_s = float(segment.get("end_s", segment.get("endS", 0.88)))
        reverse = bool(segment.get("reversed", segment.get("reverse", False)))
        local_points = sample_polyline_interval(edge["samples"], start_s, end_s, seam_sample_count)
        if reverse:
            local_points = list(reversed(local_points))
        world_points = [dog_box_local_to_positioner_world(point, topology) for point in local_points]
        seam_plane_normal = plane_normal_from_points(world_points, fallback=[0.0, 0.0, 1.0])
        geometry_method = choose_weld_geometry_method(world_points, world_center)
        selected_normal = normalize_vector(scale(seam_plane_normal, float(geometry_method["normal_sign"])))
        selected_side_sign = float(geometry_method["side_sign"])
        seam_id = str(segment.get("id") or f"seam_{segment_index + 1:02d}")
        motion_seams.append(
            {
                "seam_id": seam_id,
                "world_points": world_points,
                "normal": selected_normal,
                "side_sign": selected_side_sign,
            }
        )
        start = world_points[0]
        stop = world_points[-1]
        approach = [start[0], start[1], start[2] + APPROACH_OFFSET_MM]
        retract = [stop[0], stop[1], stop[2] + APPROACH_OFFSET_MM]
        sequence.extend(
            [
                {"kind": "approach", "seam_id": seam_id, "target_xyz_mm": [round(v, 6) for v in approach]},
                {"kind": "weld_start", "seam_id": seam_id, "target_xyz_mm": [round(v, 6) for v in start]},
                {"kind": "weld_stop", "seam_id": seam_id, "target_xyz_mm": [round(v, 6) for v in stop]},
                {"kind": "retract", "seam_id": seam_id, "target_xyz_mm": [round(v, 6) for v in retract]},
            ]
        )
        seam_records.append(
            {
                "id": seam_id,
                "edge_id": edge["id"],
                "part_id": edge.get("part_id"),
                "geom_type": edge.get("geom_type", "EDGE"),
                "start_s": round(max(0.0, min(1.0, start_s)), 4),
                "end_s": round(max(0.0, min(1.0, end_s)), 4),
                "reversed": reverse,
                "start_point_mm": [round(float(coord), 6) for coord in start],
                "stop_point_mm": [round(float(coord), 6) for coord in stop],
                "point_count": len(world_points),
                "geometry_method": geometry_method,
            }
        )
        for point_index, point in enumerate(world_points):
            tangent = tangent_at(world_points, point_index)
            torch = weld_torch_frame(tangent, selected_normal, side_sign=selected_side_sign)
            tcp_poses.append(
                {
                    "index": len(tcp_poses),
                    "seam_id": seam_id,
                    "time_s": round(time_s, 4),
                    "target_xyz_m": [round(coord / 1000.0, 9) for coord in point],
                    "target_xyz_mm": [round(coord, 6) for coord in point],
                    "seam_tangent": [round(value, 9) for value in tangent],
                    "seam_plane_normal": [round(value, 9) for value in torch["seam_plane_normal"]],
                    "seam_side_axis": [round(value, 9) for value in torch["seam_side_axis"]],
                    "tool_x_axis": [round(value, 9) for value in torch["tool_x_axis"]],
                    "tool_y_axis": [round(value, 9) for value in torch["tool_y_axis"]],
                    "tool_z_axis": [round(value, 9) for value in torch["tool_z_axis"]],
                    "work_angle_deg": torch["work_angle_deg"],
                    "angle_from_seam_plane_deg": round(torch["angle_from_seam_plane_deg"], 6),
                    "tool_x_tangent_alignment": round(torch["tool_x_tangent_alignment"], 9),
                    "tool_z_plane_normal_component": round(torch["tool_z_plane_normal_component"], 9),
                    "orientation_rpy_rad": [round(value, 9) for value in torch["orientation_rpy_rad"]],
                    "geometry_method": geometry_method["selected"],
                }
            )
            time_s += 0.18

    return {
        "schema_version": 1,
        "tool": "rosie cadquery v1 weld sequence backend",
        "model_id": topology["model_id"],
        "filename": topology["filename"],
        "fingerprint": topology["fingerprint"],
        "unit": "mm",
        "workflow": {
            "source": "cadquery",
            "stage": "multi_seam_weld_sequence",
            "tour": "dog_box" if topology["filename"] == "dog_box.step" else None,
            "computed_by": "python/cadquery",
        },
        "placement": POSITIONER_TABLE_A_FRAME,
        "environment": tesseract_environment(topology),
        "weld_sequence": {
            "schema": "cadquery.v1.weld-sequence",
            "robot": "rosie_1400_v3",
            "frame": "world",
            "positioner_axis": "J7a",
            "seams": seam_records,
        },
        "tesseract_plan": {
            "schema": "cadquery.v1.tesseract-weld-plan",
            "planner": "tesseract",
            "robot": "rosie_1400_v3",
            "working_frame": "world",
            "positioner": POSITIONER_TABLE_A_FRAME,
            "environment": tesseract_environment(topology),
            "geometry_methods": [seam["geometry_method"] for seam in seam_records],
            "sequence": sequence,
            "tcp_poses": tcp_poses,
            "motion_plan": build_motion_trajectory(motion_seams),
        },
    }


def extract_topology(step_path: Path, sample_count: int) -> dict[str, Any]:
    imported = load_step(step_path)
    shape = shape_from_import(imported)
    solids = solids_from_shape(shape)
    fingerprint = hashlib.sha256(step_path.read_bytes()).hexdigest()

    parts: list[dict[str, Any]] = []
    edges: list[dict[str, Any]] = []
    composite_edges: list[dict[str, Any]] = []
    seen_hashes: set[int] = set()
    seen_fallback_keys: set[tuple[Any, ...]] = set()
    edge_index = 0

    for part_index, solid in enumerate(solids):
        part_id = f"part_{part_index}"
        local_count = 0
        composite_edges.extend(build_face_wire_edges(solid, part_id, sample_count, len(composite_edges)))
        for edge in edges_from_shape(solid):
            samples = sample_edge(edge, sample_count)
            if len(samples) < 2:
                continue

            hash_value = shape_hash(edge)
            if hash_value is not None:
                if hash_value in seen_hashes:
                    continue
                seen_hashes.add(hash_value)
            else:
                fallback_key = fallback_edge_key(samples)
                if fallback_key in seen_fallback_keys:
                    continue
                seen_fallback_keys.add(fallback_key)

            length = polyline_length(samples)
            p_min, p_max = bounds(samples)
            edge_id = f"{part_id}:edge_{edge_index:05d}"
            edges.append(
                {
                    "id": edge_id,
                    "part_id": part_id,
                    "index": edge_index,
                    "local_index": local_count,
                    "geom_type": geom_type(edge),
                    "sample_count": len(samples),
                    "samples": samples,
                    "length_mm": length,
                    "bounds_min": p_min,
                    "bounds_max": p_max,
                }
            )
            edge_index += 1
            local_count += 1
        parts.append({"id": part_id, "edge_count": local_count})

    if not edges:
        raise ValueError("No CAD edges found in STEP topology.")

    all_edges = composite_edges + edges
    annotate_seam_candidates(all_edges)
    assign_edge_labels(all_edges)
    topology = {
        "schema_version": 1,
        "tool": "rosie cadquery v1 seam topology",
        "model_id": f"cadquery-step-{fingerprint[:16]}",
        "filename": step_path.name,
        "fingerprint": fingerprint,
        "unit": "mm",
        "sample_count": sample_count,
        "parts": parts,
        "edges": all_edges,
        "raw_edge_count": len(edges),
        "composite_edge_count": len(composite_edges),
        "seam_candidate_count": len([edge for edge in all_edges if edge.get("is_seam_candidate")]),
    }
    topology["recommended_seams"] = recommended_seams(topology)
    return topology


def decode_step_payload(payload: dict[str, Any]) -> tuple[str, bytes, int]:
    filename = str(payload.get("filename") or "upload.step").strip() or "upload.step"
    raw_base64 = str(payload.get("step_base64") or payload.get("stepBase64") or "").strip()
    if "," in raw_base64:
        raw_base64 = raw_base64.split(",", 1)[1]
    if not raw_base64:
        raise ValueError("Field 'step_base64' is required.")
    try:
        step_bytes = base64.b64decode(raw_base64, validate=True)
    except Exception as exc:
        raise ValueError("Field 'step_base64' is not valid base64.") from exc
    if not step_bytes:
        raise ValueError("STEP payload is empty.")
    sample_count = max(8, min(256, int(payload.get("sample_count", payload.get("sampleCount", 64)))))
    return filename, step_bytes, sample_count


def extract_from_stdin() -> int:
    payload = json.load(sys.stdin)
    if not isinstance(payload, dict):
        raise ValueError("JSON object required.")
    filename, step_bytes, sample_count = decode_step_payload(payload)
    suffix = Path(filename).suffix.lower()
    if suffix not in {".step", ".stp"}:
        suffix = ".step"
    handle = tempfile.NamedTemporaryFile(suffix=suffix, delete=False)
    try:
        with handle:
            handle.write(step_bytes)
        result = extract_topology(Path(handle.name), sample_count)
        result["filename"] = filename
        operation = str(payload.get("operation") or "extract_topology")
        if operation == "build_seam":
            edge_id = str(payload.get("edge_id") or payload.get("edgeId") or "").strip()
            if not edge_id:
                recommendation = recommended_seam(result)
                edge_id = recommendation["edge_id"]
            start_s = float(payload.get("start_s", payload.get("startS", 0.0)))
            end_s = float(payload.get("end_s", payload.get("endS", 1.0)))
            seam_sample_count = max(2, min(512, int(payload.get("seam_sample_count", payload.get("seamSampleCount", 40)))))
            reverse = bool(payload.get("reversed", payload.get("reverse", False)))
            geometry_method_id = str(payload.get("geometry_method", payload.get("geometryMethod", "")) or "").strip() or None
            path_method = str(payload.get("path_method", payload.get("pathMethod", "sampled_edge")) or "sampled_edge").strip()
            result = build_seam_payload(result, edge_id, start_s, end_s, seam_sample_count, reverse, geometry_method_id, path_method)
        elif operation == "build_sequence":
            raw_segments = payload.get("segments")
            if not isinstance(raw_segments, list) or not raw_segments:
                raw_segments = recommended_seams(result)
            seam_sample_count = max(2, min(512, int(payload.get("seam_sample_count", payload.get("seamSampleCount", 40)))))
            result = build_weld_sequence_payload(result, raw_segments, seam_sample_count)
        print(json.dumps(result, separators=(",", ":")))
        return 0
    finally:
        try:
            os.remove(handle.name)
        except OSError:
            pass


def extract_from_path(args: argparse.Namespace) -> int:
    step_path = Path(args.step).expanduser().resolve()
    if not step_path.exists():
        raise FileNotFoundError(f"STEP file not found: {step_path}")
    result = extract_topology(step_path, sample_count=args.sample_count)
    print(json.dumps(result, indent=2))
    return 0


def self_test() -> int:
    # Synthetic unit plans skip the tesseract worker for speed/determinism;
    # the real Dog Box plan below runs it when the tesseract env is present.
    os.environ[COLLISION_CHECK_ENV_FLAG] = "1"
    points = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [3.0, 4.0, 0.0]]
    assert dedupe_neighbor_points(points) == [[0.0, 0.0, 0.0], [3.0, 4.0, 0.0]]
    assert math.isclose(polyline_length(dedupe_neighbor_points(points)), 5.0)
    payload = {
        "filename": "x.step",
        "step_base64": base64.b64encode(b"STEP").decode("ascii"),
        "sample_count": 4,
    }
    filename, decoded, sample_count = decode_step_payload(payload)
    assert filename == "x.step"
    assert decoded == b"STEP"
    assert sample_count == 8
    interval = sample_polyline_interval([[0, 0, 0], [10, 0, 0]], 0.2, 0.8, 4)
    assert interval == [[2.0, 0.0, 0.0], [4.0, 0.0, 0.0], [6.0, 0.0, 0.0], [8.0, 0.0, 0.0]]
    arc = interpolate_three_point_arc([[1, 0, 0], [0, 1, 0], [-1, 0, 0]], 5)
    assert len(arc) == 5
    assert math.isclose(arc[2][0], 0.0, abs_tol=1e-6)
    assert math.isclose(arc[2][1], 1.0, abs_tol=1e-6)
    topology = {
        "model_id": "cadquery-step-test",
        "filename": "unit.step",
        "fingerprint": "abc",
        "edges": [
            {
                "id": "part_0:edge_00000",
                "part_id": "part_0",
                "geom_type": "LINE",
                "length_mm": 10.0,
                "samples": [[0, 0, 0], [10, 0, 0]],
            }
        ],
    }
    seam = build_seam_payload(topology, "part_0:edge_00000", 0.2, 0.8, 4, False)
    assert seam["workflow"]["computed_by"] == "python/cadquery"
    assert seam["seam"]["points"][0] == [2.0, 0.0, 0.0]
    assert seam["placement"]["axis"] == "J7a"
    assert seam["weld_sequence"]["seams"][0]["start_point_mm"]
    assert len(seam["tesseract_plan"]["tcp_poses"]) == 4
    assert math.isclose(seam["tesseract_plan"]["tcp_poses"][0]["angle_from_seam_plane_deg"], 45.0, abs_tol=1e-6)
    assert seam["tesseract_plan"]["tcp_poses"][0]["tool_x_tangent_alignment"] > 0.999
    assert seam["tesseract_plan"]["tcp_poses"][0]["seam_plane_normal"]
    assert seam["tesseract_plan"]["environment"]["collision_objects"][0]["name"] == "dog_box_workpiece"
    seam_motion = seam["tesseract_plan"]["motion_plan"]
    assert seam_motion["sample_count"] > len(seam["tesseract_plan"]["tcp_poses"])
    assert seam_motion["weld_sample_count"] == 4
    assert seam_motion["duration_s"] > 0.0
    assert seam_motion["weld_length_mm"] > 0.0 and seam_motion["travel_length_mm"] > 0.0
    seam_moves = {sample["move"] for sample in seam_motion["trajectory"]}
    assert {"approach", "weld", "retract"}.issubset(seam_moves)
    assert all(len(sample["tool_quat_xyzw"]) == 4 for sample in seam_motion["trajectory"])
    # FK sanity: at all-zero joints the TCP is the plain sum of the URDF
    # origins plus the fixed end-effector offset (identity link rotations).
    _, home_translation = robot_fk_tcp([0.0] * 6)
    expected_home = [
        0.7874 + 0.159999735 + 0.159 + 0.4877 + 0.004682724,
        0.6858 - 0.125500489 + 0.125499999 - 0.047833043 + 0.047833043,
        0.4808375 + 0.269888605 + 0.6 + 0.105000002 - 0.0933 - 0.398382724,
    ]
    assert all(math.isclose(home_translation[axis], expected_home[axis], abs_tol=1e-9) for axis in range(3))
    # IK round-trip: solving for a random FK pose recovers that pose.
    probe_q = [0.3, 0.6, -0.9, 0.4, 0.5, -0.2]
    probe_rotation, probe_translation = robot_fk_tcp(probe_q)
    probe_solution = solve_tcp_ik(probe_rotation, probe_translation, IK_HOME_SEED_RAD)
    assert probe_solution["position_error_mm"] < 0.5, probe_solution
    assert probe_solution["rotation_error_deg"] < 0.5, probe_solution
    # Every trajectory sample carries a joint solution for the URDF robot.
    assert seam_motion["ik"]["joint_names"] == ["J1", "J2", "J3", "J4", "J5", "J6"]
    assert all(len(sample["joint_positions_rad"]) == 6 for sample in seam_motion["trajectory"])
    assert all(sample["joint_names"] == ["J1", "J2", "J3", "J4", "J5", "J6"] for sample in seam_motion["trajectory"])
    sequence = build_weld_sequence_payload(
        topology,
        [
            {"id": "seam_01", "edge_id": "part_0:edge_00000", "start_s": 0.0, "end_s": 0.4},
            {"id": "seam_02", "edge_id": "part_0:edge_00000", "start_s": 0.6, "end_s": 1.0},
        ],
        3,
    )
    assert len(sequence["weld_sequence"]["seams"]) == 2
    assert len(sequence["tesseract_plan"]["tcp_poses"]) == 6
    assert math.isclose(sequence["tesseract_plan"]["tcp_poses"][0]["angle_from_seam_plane_deg"], 45.0, abs_tol=1e-6)
    assert sequence["tesseract_plan"]["tcp_poses"][0]["tool_x_tangent_alignment"] > 0.999
    assert sequence["environment"]["collision_objects"][0]["parent_link"] == "positioner_table_a"
    seq_motion = sequence["tesseract_plan"]["motion_plan"]
    assert {"approach", "weld", "retract", "travel"}.issubset({sample["move"] for sample in seq_motion["trajectory"]})
    assert seq_motion["weld_sample_count"] == 6
    assert len(seq_motion["per_seam_frame_rotation_deg"]) == 2
    dog_box_step = Path(__file__).resolve().parents[1] / "step" / "dog_box.step"
    if dog_box_step.exists():
        dog_box_topology = extract_topology(dog_box_step, 32)
        class_counts = {
            path_class: len(
                [
                    edge
                    for edge in dog_box_topology["edges"]
                    if edge.get("path_class") == path_class and edge.get("is_seam_candidate")
                ]
            )
            for path_class in {"dogbone_4arc_4line", "circle"}
        }
        assert dog_box_topology["seam_candidate_count"] == 10
        assert class_counts == {"dogbone_4arc_4line": 6, "circle": 4}
        dogbone = next(edge for edge in dog_box_topology["edges"] if edge.get("path_class") == "dogbone_4arc_4line")
        dogbone_payload = build_seam_payload(dog_box_topology, dogbone["id"], 0.12, 0.88, 16, False, None, "three_point_arc")
        assert dogbone_payload["tesseract_plan"]["path_method"]["selected"] == "sampled_edge"
        assert "standalone circular paths" in dogbone_payload["tesseract_plan"]["path_method"]["fallback_reason"]
        circle = next(edge for edge in dog_box_topology["edges"] if edge.get("path_class") == "circle")
        circle_payload = build_seam_payload(dog_box_topology, circle["id"], 0.12, 0.88, 16, False, None, "three_point_arc")
        assert circle_payload["tesseract_plan"]["path_method"]["selected"] == "three_point_arc"
        os.environ.pop(COLLISION_CHECK_ENV_FLAG, None)
        dogbone_motion = build_seam_payload(dog_box_topology, dogbone["id"], 0.0, 1.0, 96, False)["tesseract_plan"]["motion_plan"]
        os.environ[COLLISION_CHECK_ENV_FLAG] = "1"
        # Smoothed frames keep the torch from snapping at the dog-bone corners.
        assert dogbone_motion["max_frame_rotation_deg"] < 90.0
        assert dogbone_motion["weld_sample_count"] == 96
        # Real Tesseract collision check (report-only, robot/v4 pattern). On
        # machines without the tesseract/v1 env the summary reports why.
        collision = dogbone_motion["collision"]
        assert collision["status"] in {"checked", "unavailable"}, collision
        if collision["status"] == "checked":
            assert collision["checker"].startswith("Bullet"), collision
            assert collision["checked_waypoint_count"] == dogbone_motion["sample_count"], collision
            contact_samples = [s for s in dogbone_motion["trajectory"] if s.get("collision_contacts")]
            assert len(contact_samples) == collision["contact_sample_count"], collision
            for sample in contact_samples:
                for contact in sample["collision_contacts"]:
                    assert contact["link_a"] and contact["link_b"] and contact["depth_mm"] >= 0.0
        # The URDF joint trajectory must track the TCP plan tightly (sub-mm
        # position, sub-degree torch axis) and stay continuous enough that the
        # lerped playback doesn't jump (free torch spin reorients the wrist by
        # up to ~0.6 rad through the dog-bone corners).
        assert dogbone_motion["ik"]["max_position_error_mm"] < 1.0, dogbone_motion["ik"]
        assert dogbone_motion["ik"]["max_rotation_error_deg"] < 1.0, dogbone_motion["ik"]
        assert dogbone_motion["ik"]["max_joint_step_rad"] < 0.75, dogbone_motion["ik"]
    print("extract_weld_edges self-test ok")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Extract sampled STEP edge topology for weld seam selection.")
    parser.add_argument("--stdin", action="store_true", help="Read a JSON request from stdin and emit JSON response.")
    parser.add_argument("--step", default="", help="STEP/STP path for CLI inspection.")
    parser.add_argument("--sample-count", type=int, default=64, help="Samples per edge.")
    parser.add_argument("--self-test", action="store_true", help=argparse.SUPPRESS)
    return parser


def main(argv: list[str]) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.self_test:
        return self_test()
    if args.stdin:
        return extract_from_stdin()
    if not args.step:
        parser.error("--step is required unless --stdin is set")
    return extract_from_path(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:
        print(f"extract_weld_edges: {exc}", file=sys.stderr)
        raise SystemExit(1)
