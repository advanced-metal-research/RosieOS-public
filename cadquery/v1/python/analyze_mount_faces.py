import argparse
import json
import math
import sys
from pathlib import Path


def load_shape(step_path: Path):
    import cadquery as cq

    imported = cq.importers.importStep(str(step_path))
    return imported.val() if hasattr(imported, "val") else imported


def vec(value) -> list[float]:
    return [float(value.x), float(value.y), float(value.z)]


def bbox_dict(box) -> dict:
    return {
        "min": [float(box.xmin), float(box.ymin), float(box.zmin)],
        "max": [float(box.xmax), float(box.ymax), float(box.zmax)],
    }


def bbox_size(box) -> list[float]:
    return [float(box.xmax - box.xmin), float(box.ymax - box.ymin), float(box.zmax - box.zmin)]


def normalize(values: list[float]) -> list[float]:
    length = math.sqrt(sum(value * value for value in values))
    if length <= 1e-12:
        return [0.0, 0.0, 0.0]
    return [float(value / length) for value in values]


def dot(left: list[float], right: list[float]) -> float:
    return float(sum(a * b for a, b in zip(left, right)))


def scale(values: list[float], amount: float) -> list[float]:
    return [float(value * amount) for value in values]


def sub(left: list[float], right: list[float]) -> list[float]:
    return [float(a - b) for a, b in zip(left, right)]


def axis_name(values: list[float]) -> str:
    axes = [("X", [1.0, 0.0, 0.0]), ("Y", [0.0, 1.0, 0.0]), ("Z", [0.0, 0.0, 1.0])]
    normalized = normalize(values)
    name, axis = max(axes, key=lambda item: abs(dot(normalized, item[1])))
    sign = "+" if dot(normalized, axis) >= 0 else "-"
    return f"{sign}{name}"


def projected_world_axis(axis: list[float], normal: list[float]) -> list[float]:
    projected = sub(axis, scale(normal, dot(axis, normal)))
    return normalize(projected)


def face_axes(normal: list[float], size: list[float]) -> tuple[list[float], list[float]]:
    world_axes = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
    candidates = []
    for axis in world_axes:
        projected = projected_world_axis(axis, normal)
        if dot(projected, projected) <= 1e-8:
            continue
        # Bbox extents are a rough but stable proxy for the face's visible axes.
        extent = abs(projected[0]) * size[0] + abs(projected[1]) * size[1] + abs(projected[2]) * size[2]
        candidates.append((extent, projected))
    if not candidates:
        return [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]
    candidates.sort(key=lambda item: item[0], reverse=True)
    u = candidates[0][1]
    v = candidates[1][1] if len(candidates) > 1 else normalize([
        normal[1] * u[2] - normal[2] * u[1],
        normal[2] * u[0] - normal[0] * u[2],
        normal[0] * u[1] - normal[1] * u[0],
    ])
    return u, v


def major_axes(box) -> list[dict]:
    center = [
        float((box.xmin + box.xmax) / 2),
        float((box.ymin + box.ymax) / 2),
        float((box.zmin + box.zmax) / 2),
    ]
    raw = [
        ("X", [1.0, 0.0, 0.0], float(box.xmax - box.xmin)),
        ("Y", [0.0, 1.0, 0.0], float(box.ymax - box.ymin)),
        ("Z", [0.0, 0.0, 1.0], float(box.zmax - box.zmin)),
    ]
    return [
        {
            "name": name,
            "direction": direction,
            "length": length,
            "center": center,
        }
        for name, direction, length in sorted(raw, key=lambda item: item[2], reverse=True)
    ]


def leaf_faces(shape):
    try:
        solids = list(shape.Solids())
    except Exception:
        solids = []
    if not solids:
        try:
            for face_index, face in enumerate(shape.Faces()):
                yield face_index, face, {"type": "shape", "index": 0, "face_index": face_index}
        except Exception:
            return
        return
    global_index = 0
    for solid_index, solid in enumerate(solids):
        try:
            solid_box = solid.BoundingBox()
            solid_bbox = bbox_dict(solid_box)
            solid_size = bbox_size(solid_box)
        except Exception:
            solid_bbox = None
            solid_size = None
        for solid_face_index, face in enumerate(solid.Faces()):
            yield global_index, face, {
                "type": "solid",
                "index": solid_index,
                "face_index": solid_face_index,
                "bbox": solid_bbox,
                "bbox_size": solid_size,
            }
            global_index += 1


def circular_edge_count(face) -> int:
    count = 0
    for edge in face.Edges():
        try:
            if edge.geomType() in {"CIRCLE", "ELLIPSE"}:
                count += 1
        except Exception:
            continue
    return count


def circular_edges(face) -> list[dict]:
    records = []
    for edge in face.Edges():
        try:
            if edge.geomType() not in {"CIRCLE", "ELLIPSE"}:
                continue
            center = vec(edge.Center())
            radius = None
            try:
                radius = float(edge.radius())
            except Exception:
                try:
                    radius = float(edge.Radius())
                except Exception:
                    radius = None
            records.append({"center": center, "radius": radius})
        except Exception:
            continue
    return records


def projected_wire_holes(face, face_center: list[float], u: list[float], v: list[float]) -> list[dict]:
    holes = []
    try:
        wires = list(face.innerWires())
    except Exception:
        wires = []
    for wire in wires:
        try:
            center = vec(wire.Center())
            delta = sub(center, face_center)
            box = wire.BoundingBox()
            size = sorted([abs(value) for value in bbox_size(box)], reverse=True)
            radius = (size[1] / 2.0) if len(size) > 1 else None
            holes.append({
                "point": [dot(delta, u), dot(delta, v)],
                "radius": radius,
                "source": "inner_wire",
            })
        except Exception:
            continue
    return holes


def clustered_circular_holes(face, face_center: list[float], u: list[float], v: list[float]) -> list[dict]:
    clusters: dict[tuple[int, int], list[float]] = {}
    for edge in circular_edges(face):
        delta = sub(edge["center"], face_center)
        point = [dot(delta, u), dot(delta, v)]
        key = (round(point[0] / 0.25), round(point[1] / 0.25))
        radius = edge.get("radius")
        if radius is not None:
            clusters.setdefault(key, []).append(float(radius))
    holes = []
    for key, radii in clusters.items():
        # Multiple concentric circular edges at one center are usually rings/gears,
        # not separate bolt holes. Keep one representative only if there are
        # enough distinct centers for a pattern.
        holes.append({
            "point": [key[0] * 0.25, key[1] * 0.25],
            "radius": min(radii) if radii else None,
            "source": "clustered_circle_edges",
        })
    if len(holes) < 3:
        return []
    return holes


def hole_pattern(face, center: list[float], normal: list[float], size: list[float]) -> dict:
    u, v = face_axes(normal, size)
    edges = circular_edges(face)
    holes = projected_wire_holes(face, center, u, v)
    if not holes:
        holes = clustered_circular_holes(face, center, u, v)
    projected = [hole["point"] for hole in holes]
    radii = [float(hole["radius"]) for hole in holes if hole.get("radius") is not None]
    radial_distances = [math.hypot(point[0], point[1]) for point in projected]
    return {
        "circle_count": len(edges),
        "hole_count": len(holes),
        "face_u": u,
        "face_v": v,
        "face_u_axis": axis_name(u),
        "face_v_axis": axis_name(v),
        "radial_min": min(radial_distances) if radial_distances else None,
        "radial_max": max(radial_distances) if radial_distances else None,
        "radial_avg": sum(radial_distances) / len(radial_distances) if radial_distances else None,
        "radius_min": min(radii) if radii else None,
        "radius_max": max(radii) if radii else None,
        "radius_avg": sum(radii) / len(radii) if radii else None,
        "sample_points": projected[:32],
        "sample_holes": holes[:64],
        "sample_circles": holes[:64],
    }


def face_record(face, index: int, leaf: dict) -> dict | None:
    try:
        geom_type = face.geomType()
        try:
            inner_wires = list(face.innerWires())
        except Exception:
            inner_wires = []
        try:
            wires = list(face.Wires())
        except Exception:
            wires = []
        circles = circular_edge_count(face)
        box = face.BoundingBox()
        try:
            normal = normalize(vec(face.normalAt()))
        except Exception:
            normal = normal_from_bbox(box)
        center = vec(face.Center())
        size = bbox_size(box)
        boundary = boundary_info(box, leaf)
        area = float(face.Area())
        if area <= 0:
            return None
        pattern = hole_pattern(face, center, normal, size)
        planar_size = sorted([abs(value) for value in size], reverse=True)[:2]
        thinness = min(abs(value) for value in size)
        ring_bonus = 0.0
        if pattern["radial_max"] is not None and pattern["radial_min"] is not None:
            ring_bonus = max(0.0, pattern["radial_max"] - pattern["radial_min"])
        geom_bonus = {
            "PLANE": 50_000,
            "CYLINDER": 35_000,
            "CONE": 15_000,
            "TORUS": 15_000,
            "SPHERE": 5_000,
        }.get(geom_type, 0)
        score = (
            len(inner_wires) * 1_000_000
            + circles * 240_000
            + geom_bonus
            + area
            + ring_bonus * 8_000
            - thinness * 20_000
        )
        return {
            "index": index,
            "score": score,
            "geom_type": geom_type,
            "center": center,
            "normal": normal,
            "bbox": bbox_dict(box),
            "bbox_size": size,
            "is_boundary": boundary["is_boundary"],
            "boundary_axes": boundary["axes"],
            "planar_size": planar_size,
            "area": area,
            "wire_count": len(wires),
            "leaf": leaf,
            "inner_wire_count": len(inner_wires),
            "circular_edge_count": circles,
            "normal_axis": axis_name(normal),
            "hole_pattern": pattern,
            "reason": face_reason(geom_type, len(inner_wires), circles),
            "_face": face,
        }
    except Exception:
        return None


def boundary_info(face_box, leaf: dict) -> dict:
    leaf_box = leaf.get("bbox") if isinstance(leaf, dict) else None
    if not leaf_box:
        return {"is_boundary": True, "axes": ["unknown"]}
    tol = 0.25
    axes = []
    face_min = [float(face_box.xmin), float(face_box.ymin), float(face_box.zmin)]
    face_max = [float(face_box.xmax), float(face_box.ymax), float(face_box.zmax)]
    leaf_min = leaf_box.get("min", [0, 0, 0])
    leaf_max = leaf_box.get("max", [0, 0, 0])
    for idx, name in enumerate(["x", "y", "z"]):
        if abs(face_min[idx] - leaf_min[idx]) <= tol:
            axes.append("-" + name)
        if abs(face_max[idx] - leaf_max[idx]) <= tol:
            axes.append("+" + name)
    return {"is_boundary": bool(axes), "axes": axes}


def normal_from_bbox(box) -> list[float]:
    sizes = [
        (float(box.xmax - box.xmin), [1.0, 0.0, 0.0]),
        (float(box.ymax - box.ymin), [0.0, 1.0, 0.0]),
        (float(box.zmax - box.zmin), [0.0, 0.0, 1.0]),
    ]
    return min(sizes, key=lambda item: item[0])[1]


def face_reason(geom_type: str, inner_wire_count: int, circle_count: int) -> str:
    if geom_type == "PLANE" and (inner_wire_count or circle_count):
        return "planar face with holes/circles; likely mounting, bearing, or gear interface candidate"
    if geom_type == "PLANE":
        return "planar face; included so flat mating faces without holes are not skipped"
    if circle_count:
        return f"{geom_type.lower()} face with circular edges; possible bearing/gear/shaft interface"
    return f"{geom_type.lower()} face; included for exhaustive connection scoring"


def analyze(args: argparse.Namespace) -> dict:
    step_path = Path(args.step).expanduser().resolve()
    shape = load_shape(step_path)
    shape_box = shape.BoundingBox()
    candidates = []
    leaves = set()
    for index, face, leaf in leaf_faces(shape):
        leaves.add(f"{leaf.get('type')}:{leaf.get('index')}")
        record = face_record(face, index, leaf)
        if record:
            candidates.append(record)
    candidates.sort(key=lambda item: item["score"], reverse=True)
    out_path = Path(args.out).expanduser().resolve()
    face_dir = out_path.parent / "faces"
    face_dir.mkdir(parents=True, exist_ok=True)
    from cadquery import exporters

    exported_candidates = []
    mesh_limit = args.limit
    export_all_meshes = mesh_limit < 0
    for index, item in enumerate(candidates):
        face = item.pop("_face")
        if export_all_meshes or index < mesh_limit:
            mesh_name = f"face_{item['index']}.stl"
            mesh_path = face_dir / mesh_name
            try:
                exporters.export(face, str(mesh_path), exportType="STL")
                item["highlight_mesh"] = f"faces/{mesh_name}"
            except Exception as exc:
                item["highlight_error"] = str(exc)
        else:
            item["highlight_mesh"] = None
            item["mesh_exported"] = False
        exported_candidates.append(item)
    selected = exported_candidates[0] if exported_candidates else None
    report = {
        "schema_version": 1,
        "part": args.part,
        "step": str(step_path),
        "part_bbox": bbox_dict(shape_box),
        "part_size": bbox_size(shape_box),
        "major_axes": major_axes(shape_box),
        "leaf_count": len(leaves),
        "candidate_count_total": len(exported_candidates),
        "highlight_mesh_count": sum(1 for item in exported_candidates if item.get("highlight_mesh")),
        "selected_face": selected,
        "candidates": exported_candidates,
        "notes": [
            "Selection is CadQuery-derived from all readable CAD faces, not only the display/export subset.",
            "Scores prefer bolt-hole and gear/bearing-like circle patterns over largest-face heuristics.",
            "Only the top --limit candidates get highlight meshes for UI speed; all candidates remain in JSON for matching.",
        ],
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"analyze-mount-faces: wrote {out_path}")
    if selected:
        print(
            "analyze-mount-faces: selected "
            f"face={selected['index']} inner_wires={selected['inner_wire_count']} "
            f"circles={selected['circular_edge_count']} center={selected['center']}"
        )
    return report


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Analyze STEP faces for likely mounting hole-pattern faces.")
    parser.add_argument("--step", required=True, help="STEP/STP path")
    parser.add_argument("--part", required=True, help="Part name")
    parser.add_argument("--out", required=True, help="Output JSON path")
    parser.add_argument("--limit", type=int, default=80, help="Candidate face meshes to export for UI; negative exports all, JSON keeps all")
    args = parser.parse_args(argv)
    analyze(args)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:
        print(f"analyze_mount_faces: {exc}", file=sys.stderr)
        raise SystemExit(1)
