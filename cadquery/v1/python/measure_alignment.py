import argparse
import json
import math
import struct
import sys
from pathlib import Path


def read_fusion_manifest(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def iter_ascii_stl_vertices(raw: bytes):
    for line in raw.decode("utf-8", errors="ignore").splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0].lower() == "vertex":
            try:
                yield [float(parts[1]), float(parts[2]), float(parts[3])]
            except ValueError:
                continue


def iter_binary_stl_vertices(raw: bytes):
    if len(raw) < 84:
        return
    tri_count = struct.unpack_from("<I", raw, 80)[0]
    expected = 84 + tri_count * 50
    if expected > len(raw):
        return
    offset = 84
    for _ in range(tri_count):
        offset += 12  # normal
        for _ in range(3):
            yield list(struct.unpack_from("<fff", raw, offset))
            offset += 12
        offset += 2


def stl_vertices(path: Path):
    raw = path.read_bytes()
    looks_ascii = raw[:80].lstrip().lower().startswith(b"solid") and b"\n" in raw[:512]
    vertices = list(iter_ascii_stl_vertices(raw)) if looks_ascii else []
    if not vertices:
        vertices = list(iter_binary_stl_vertices(raw) or [])
    if not vertices:
        raise ValueError(f"no STL vertices found in {path}")
    return vertices


def bounds(vertices: list[list[float]]) -> dict:
    mins = [min(v[i] for v in vertices) for i in range(3)]
    maxs = [max(v[i] for v in vertices) for i in range(3)]
    center = [(mins[i] + maxs[i]) / 2 for i in range(3)]
    size = [maxs[i] - mins[i] for i in range(3)]
    diag = math.sqrt(sum(s * s for s in size))
    return {"min": mins, "max": maxs, "center": center, "size": size, "diagonal": diag}


def resolve_mesh_path(mesh_dir: Path, mesh: str, mesh_path: str = "") -> Path:
    path = Path(mesh_path) if mesh_path else mesh_dir / mesh
    if path.is_absolute():
        return path.resolve()
    return (mesh_dir / mesh).resolve()


def iter_measurement_targets(manifest: dict, mesh_dir: Path):
    seen = set()

    def add_target(target: dict):
        mesh = str(target.get("mesh", ""))
        if not mesh:
            return
        mesh_path = resolve_mesh_path(mesh_dir, mesh, str(target.get("mesh_path") or ""))
        key = str(mesh_path).lower()
        if key in seen:
            return
        seen.add(key)
        entry = dict(target)
        entry["mesh"] = mesh
        entry["mesh_path"] = str(mesh_path)
        yield entry

    for part in manifest.get("parts", []):
        yield from add_target(
            {
                "target_type": "part",
                "index": part.get("index"),
                "name": part.get("name"),
                "mesh": part.get("mesh", ""),
                "mesh_path": part.get("mesh_path", ""),
                "logical_link_name": part.get("logical_link_name", ""),
                "occurrence_path": part.get("occurrence_path", ""),
                "component_name": part.get("component_name", ""),
                "bbox_min_m": part.get("bbox_min_m", []),
                "bbox_max_m": part.get("bbox_max_m", []),
                "bbox_source": part.get("bbox_source", ""),
                "transform_candidate": part.get("transform_candidate", ""),
                "transform_error_m": part.get("transform_error_m"),
            }
        )

    for link in manifest.get("links", []):
        render_mesh = link.get("render_mesh") or {}
        mesh = render_mesh.get("mesh") or link.get("mesh", "")
        mesh_path = render_mesh.get("mesh_path") or link.get("mesh_path", "")
        yield from add_target(
            {
                "target_type": "link",
                "index": link.get("index"),
                "name": link.get("name"),
                "mesh": mesh,
                "mesh_path": mesh_path,
                "logical_link_name": link.get("name", ""),
                "mesh_role": link.get("mesh_role") or render_mesh.get("role", ""),
                "source_occurrence_count": render_mesh.get("source_occurrence_count", link.get("source_link_count")),
            }
        )


def measure(args: argparse.Namespace) -> int:
    manifest_path = Path(args.fusion_manifest).expanduser().resolve()
    manifest = read_fusion_manifest(manifest_path)
    mesh_dir = Path(args.mesh_dir).expanduser().resolve() if args.mesh_dir else Path(manifest.get("mesh_dir", "")).expanduser()
    if not mesh_dir.is_absolute():
        mesh_dir = (manifest_path.parent / mesh_dir).resolve()
    out_path = Path(args.out).expanduser().resolve() if args.out else manifest_path.parent / "cadquery_measurements.json"

    measurements = []
    for target in iter_measurement_targets(manifest, mesh_dir):
        mesh = target["mesh"]
        mesh_path = Path(target["mesh_path"])
        entry = {
            "target_type": target.get("target_type"),
            "index": target.get("index"),
            "name": target.get("name"),
            "mesh": mesh,
            "mesh_path": str(mesh_path),
            "logical_link_name": target.get("logical_link_name", ""),
            "occurrence_path": target.get("occurrence_path", ""),
            "component_name": target.get("component_name", ""),
            "bbox_min_m": target.get("bbox_min_m", []),
            "bbox_max_m": target.get("bbox_max_m", []),
            "bbox_source": target.get("bbox_source", ""),
            "transform_candidate": target.get("transform_candidate", ""),
            "transform_error_m": target.get("transform_error_m"),
            "status": "missing",
        }
        if target.get("mesh_role"):
            entry["mesh_role"] = target.get("mesh_role")
        if target.get("source_occurrence_count") is not None:
            entry["source_occurrence_count"] = target.get("source_occurrence_count")
        if mesh_path.exists():
            verts = stl_vertices(mesh_path)
            entry.update(
                {
                    "status": "measured",
                    "vertex_count": len(verts),
                    "bounds_raw": bounds(verts),
                    "units": "Fusion STL raw units, normally millimeters",
                }
            )
        measurements.append(entry)

    report = {
        "schema_version": 1,
        "tool": "rosie cadquery v1 measure",
        "fusion_manifest": str(manifest_path),
        "mesh_dir": str(mesh_dir),
        "measurement_count": len(measurements),
        "measurements": measurements,
        "warnings": [
            "STL triangle bounds are mesh bounds, not exact BRep bounds.",
            "Use URDF mesh scale before comparing raw STL coordinates to Fusion meter-space metadata; Fusion STL exports are commonly millimeter-based.",
        ],
    }
    out_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"cadquery measure: wrote {out_path}")
    print(f"cadquery measure: measured {sum(1 for m in measurements if m['status'] == 'measured')} mesh(es)")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Measure exported Fusion STL meshes for alignment reports.")
    parser.add_argument("--fusion-manifest", required=True, help="Fusion export_manifest.json or cell_manifest.json.")
    parser.add_argument("--mesh-dir", default="", help="Override mesh directory. Defaults to manifest mesh_dir.")
    parser.add_argument("--out", default="", help="Output JSON path. Defaults beside manifest.")
    parser.add_argument("--self-test", action="store_true", help=argparse.SUPPRESS)
    return parser


def self_test() -> int:
    verts = [[0, 0, 0], [2, 4, 6], [1, -1, 3]]
    b = bounds(verts)
    assert b["min"] == [0, -1, 0]
    assert b["max"] == [2, 4, 6]
    assert b["center"] == [1, 1.5, 3]
    mesh_dir = Path("meshes").resolve()
    manifest = {
        "parts": [{"index": 1, "name": "source", "mesh": "source.stl", "occurrence_path": "root/source:1"}],
        "links": [
            {"index": 2, "name": "rendered", "mesh": "rendered.stl", "mesh_role": "render"},
            {"index": 3, "name": "duplicate", "mesh": "source.stl"},
        ],
    }
    targets = list(iter_measurement_targets(manifest, mesh_dir))
    assert [t["target_type"] for t in targets] == ["part", "link"]
    assert [t["mesh"] for t in targets] == ["source.stl", "rendered.stl"]
    assert targets[0]["occurrence_path"] == "root/source:1"
    print("measure_alignment self-test ok")
    return 0


def main(argv: list[str]) -> int:
    if "--self-test" in argv:
        return self_test()
    parser = build_parser()
    args = parser.parse_args(argv)
    return measure(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:
        print(f"measure_alignment: {exc}", file=sys.stderr)
        raise SystemExit(1)
