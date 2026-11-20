import argparse
import json
import math
import re
import sys
import xml.sax.saxutils as xml_escape
from pathlib import Path


def safe_name(value: str, fallback: str) -> str:
    text = str(value or "").strip()
    if not text:
        text = fallback
    text = re.sub(r"[^A-Za-z0-9_]+", "_", text)
    text = re.sub(r"_+", "_", text).strip("_")
    if not text:
        text = fallback
    if text[0].isdigit():
        text = "_" + text
    return text


def mesh_name(value: str) -> str:
    text = safe_name(value, "mesh")
    if not text.lower().endswith(".stl"):
        text += ".stl"
    return text


def load_step(step_path: Path):
    import cadquery as cq

    return cq.importers.importStep(str(step_path))


def shape_from_import(imported):
    if hasattr(imported, "val"):
        return imported.val()
    return imported


def solids_from_shape(shape):
    if hasattr(shape, "Solids"):
        solids = list(shape.Solids())
        if solids:
            return solids
    if hasattr(shape, "solids"):
        solids = list(shape.solids())
        if solids:
            return solids
    return [shape]


def export_stl(shape, out_path: Path, tolerance: float, angular_tolerance: float) -> None:
    from cadquery import exporters

    out_path.parent.mkdir(parents=True, exist_ok=True)
    exporters.export(
        shape,
        str(out_path),
        exportType="STL",
        tolerance=tolerance,
        angularTolerance=angular_tolerance,
    )


def mass_props(shape) -> dict:
    props = {}
    try:
        props["volume"] = float(shape.Volume())
    except Exception:
        pass
    try:
        center = shape.Center()
        props["center"] = [float(center.x), float(center.y), float(center.z)]
    except Exception:
        pass
    try:
        box = shape.BoundingBox()
        props["bounding_box"] = {
            "xmin": float(box.xmin),
            "ymin": float(box.ymin),
            "zmin": float(box.zmin),
            "xmax": float(box.xmax),
            "ymax": float(box.ymax),
            "zmax": float(box.zmax),
        }
    except Exception:
        pass
    return props


def write_urdf(robot_name: str, package_name: str, links: list[dict], out_path: Path) -> None:
    def esc(value: str) -> str:
        return xml_escape.escape(str(value), {'"': "&quot;"})

    lines = ['<?xml version="1.0"?>', f'<robot name="{esc(robot_name)}">']
    lines.append('  <material name="cadquery_default"><color rgba="0.65 0.65 0.65 1"/></material>')
    for link in links:
        lines.append(f'  <link name="{esc(link["name"])}">')
        lines.append("    <visual>")
        lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append(
            f'      <geometry><mesh filename="package://{esc(package_name)}/meshes/{esc(link["mesh"])}" scale="1 1 1"/></geometry>'
        )
        lines.append('      <material name="cadquery_default"/>')
        lines.append("    </visual>")
        lines.append("    <collision>")
        lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append(
            f'      <geometry><mesh filename="package://{esc(package_name)}/meshes/{esc(link["mesh"])}" scale="1 1 1"/></geometry>'
        )
        lines.append("    </collision>")
        lines.append("  </link>")
    if len(links) > 1:
        root = links[0]["name"]
        for child in links[1:]:
            joint_name = safe_name(f"{root}_to_{child['name']}", "fixed_joint")
            lines.append(f'  <joint name="{esc(joint_name)}" type="fixed">')
            lines.append(f'    <parent link="{esc(root)}"/>')
            lines.append(f'    <child link="{esc(child["name"])}"/>')
            lines.append('    <origin xyz="0 0 0" rpy="0 0 0"/>')
            lines.append("  </joint>")
    lines.append("</robot>")
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def convert(args: argparse.Namespace) -> int:
    step_path = Path(args.step).expanduser().resolve()
    if not step_path.exists():
        raise FileNotFoundError(f"STEP file not found: {step_path}")
    if step_path.suffix.lower() not in {".step", ".stp"}:
        raise ValueError(f"--step should end in .step or .stp: {step_path}")

    out_dir = Path(args.out).expanduser().resolve()
    mesh_dir = out_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    robot_name = safe_name(args.robot or step_path.stem, "cadquery_robot")
    package_name = safe_name(args.package, "assembly")

    imported = load_step(step_path)
    shape = shape_from_import(imported)
    solids = solids_from_shape(shape)
    links = []

    if args.split:
        for index, solid in enumerate(solids, start=1):
            link_name = safe_name(f"link_{index}", f"link_{index}")
            stl_name = mesh_name(f"mesh_{link_name}")
            export_stl(solid, mesh_dir / stl_name, args.tolerance, args.angular_tolerance)
            links.append(
                {
                    "name": link_name,
                    "mesh": stl_name,
                    "source": f"solid[{index}]",
                    "mass_properties": mass_props(solid),
                }
            )
    else:
        link_name = safe_name(args.link_name or "base_link", "base_link")
        stl_name = mesh_name(args.mesh_name or f"mesh_{robot_name}")
        export_stl(shape, mesh_dir / stl_name, args.tolerance, args.angular_tolerance)
        links.append(
            {
                "name": link_name,
                "mesh": stl_name,
                "source": "compound",
                "mass_properties": mass_props(shape),
            }
        )

    urdf_path = out_dir / "robot.urdf"
    write_urdf(robot_name, package_name, links, urdf_path)
    manifest = {
        "schema_version": 1,
        "tool": "rosie cadquery v1 convert",
        "step": str(step_path),
        "output_dir": str(out_dir),
        "robot_name": robot_name,
        "package": package_name,
        "split": bool(args.split),
        "solid_count": len(solids),
        "links": links,
        "warnings": [
            "STEP files do not preserve Fusion robot joints; generated URDF uses fixed joints only.",
            "Inspect mesh scale and frames before using the URDF for simulation or control.",
        ],
    }
    manifest_path = out_dir / "cadquery_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    print(f"cadquery: wrote {urdf_path}")
    print(f"cadquery: wrote {manifest_path}")
    print(f"cadquery: wrote {len(links)} STL mesh(es) under {mesh_dir}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Convert STEP/STP CAD into URDF-ready STL meshes.")
    parser.add_argument("--step", required=True, help="Input STEP/STP file.")
    parser.add_argument("--out", required=True, help="Output directory.")
    parser.add_argument("--robot", default="", help="URDF robot name. Defaults to STEP basename.")
    parser.add_argument("--package", default="assembly", help="URDF package path prefix.")
    parser.add_argument("--split", action="store_true", help="Export each solid as a fixed child link.")
    parser.add_argument("--link-name", default="base_link", help="Link name for non-split export.")
    parser.add_argument("--mesh-name", default="", help="Mesh filename for non-split export.")
    parser.add_argument("--tolerance", type=float, default=0.1, help="STL linear tolerance.")
    parser.add_argument(
        "--angular-tolerance",
        type=float,
        default=math.radians(5),
        help="STL angular tolerance in radians.",
    )
    parser.add_argument("--self-test", action="store_true", help=argparse.SUPPRESS)
    return parser


def self_test() -> int:
    assert safe_name("base link", "x") == "base_link"
    assert safe_name("123", "x") == "_123"
    assert mesh_name("mesh_base") == "mesh_base.stl"
    parser = build_parser()
    parsed = parser.parse_args(["--step", "in.step", "--out", "out"])
    assert parsed.package == "assembly"
    print("step_to_urdf self-test ok")
    return 0


def main(argv: list[str]) -> int:
    if "--self-test" in argv:
        return self_test()
    parser = build_parser()
    args = parser.parse_args(argv)
    return convert(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:
        print(f"step_to_urdf: {exc}", file=sys.stderr)
        raise SystemExit(1)
