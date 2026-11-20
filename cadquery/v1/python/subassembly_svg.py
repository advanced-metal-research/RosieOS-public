import argparse
import json
import re
import sys
from pathlib import Path


def safe_name(value: str) -> str:
    text = re.sub(r"[^A-Za-z0-9_]+", "_", value or "").strip("_")
    return text or "subassembly"


def load_step_shape(step_path: Path):
    import cadquery as cq

    imported = cq.importers.importStep(str(step_path))
    if hasattr(imported, "val"):
        return imported.val()
    return imported


def shape_bbox(shape) -> dict:
    box = shape.BoundingBox()
    return {
        "min": [float(box.xmin), float(box.ymin), float(box.zmin)],
        "max": [float(box.xmax), float(box.ymax), float(box.zmax)],
    }


def export_step_svg(shape, path: Path) -> None:
    from cadquery import exporters

    path.parent.mkdir(parents=True, exist_ok=True)
    exporters.export(
        shape,
        str(path),
        opt={
            "width": 1100,
            "height": 720,
            "marginLeft": 20,
            "marginTop": 20,
            "showAxes": False,
            "projectionDir": (0.55, -0.65, 0.5),
            "strokeWidth": 0.28,
            "strokeColor": (20, 32, 48),
            "hiddenColor": (120, 135, 160),
            "showHidden": True,
        },
    )


def build_report(step_path: Path, assembly: str, out_dir: Path) -> dict:
    shape = load_step_shape(step_path)
    svg_path = out_dir / "cad.svg"
    export_step_svg(shape, svg_path)
    report = {
        "schema_version": 1,
        "assembly": assembly,
        "step": str(step_path),
        "svg": {
            "cad": svg_path.name,
        },
        "cad_bbox_raw": shape_bbox(shape),
        "notes": [
            "SVG is generated from the STEP/CAD file exported from the live Autodesk subassembly.",
            "This is CAD geometry, not a bbox proxy drawing.",
        ],
    }
    (out_dir / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return report


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Export a CadQuery SVG drawing from a STEP/CAD subassembly.")
    parser.add_argument("--step", required=True, help="STEP/STP file exported from Autodesk.")
    parser.add_argument("--assembly", required=True, help="Occurrence path subtree")
    parser.add_argument("--out-dir", required=True, help="Output directory")
    args = parser.parse_args(argv)
    step_path = Path(args.step).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    report = build_report(step_path, args.assembly, out_dir)
    print(f"subassembly-svg: wrote {out_dir / 'report.json'}")
    print(f"subassembly-svg: step={step_path} assembly={args.assembly}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:
        print(f"subassembly_svg: {exc}", file=sys.stderr)
        raise SystemExit(1)
