import argparse
import json
import math
import re
import struct
import sys
import time
import zlib
from pathlib import Path


def safe_name(value: str) -> str:
    text = re.sub(r"[^A-Za-z0-9_]+", "_", value or "").strip("_")
    return text or "part"


def encode_png(path: Path, width: int, height: int, pixels: bytearray) -> None:
    def chunk(kind: bytes, data: bytes) -> bytes:
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    stride = width * 4
    rows = [b"\x00" + bytes(pixels[y * stride : (y + 1) * stride]) for y in range(height)]
    raw = b"".join(rows)
    png = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 7))
        + chunk(b"IEND", b"")
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(png)


def normalize(v):
    x, y, z = v
    length = math.sqrt(x * x + y * y + z * z) or 1.0
    return (x / length, y / length, z / length)


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec_tuple(v):
    if hasattr(v, "toTuple"):
        return tuple(float(x) for x in v.toTuple())
    return (float(v.x), float(v.y), float(v.z))


VIEWS = [
    {
        "id": "iso",
        "name": "Isometric",
        # Match the Autodesk/Three.js perspective view: +X, -Y, +Z.
        "u": (0.72, 0.54, 0.0),
        "v": (-0.28, 0.36, 0.82),
        "depth": (0.45, -0.55, 0.4),
    },
    {
        "id": "front",
        "name": "Front",
        "u": (1.0, 0.0, 0.0),
        "v": (0.0, 0.0, 1.0),
        "depth": (0.0, 1.0, 0.0),
    },
    {
        "id": "side",
        "name": "Side",
        "u": (0.0, 1.0, 0.0),
        "v": (0.0, 0.0, 1.0),
        "depth": (-1.0, 0.0, 0.0),
    },
    {
        "id": "top",
        "name": "Top",
        "u": (1.0, 0.0, 0.0),
        "v": (0.0, 1.0, 0.0),
        "depth": (0.0, 0.0, 1.0),
    },
]


def project(point, view):
    x, y, z = point
    u = view["u"][0] * x + view["u"][1] * y + view["u"][2] * z
    v = view["v"][0] * x + view["v"][1] * y + view["v"][2] * z
    depth = view["depth"][0] * x + view["depth"][1] * y + view["depth"][2] * z
    return u, v, depth


def view_from_depth(depth, name):
    d = normalize(depth)
    up = (0.0, 0.0, 1.0) if abs(d[2]) < 0.9 else (0.0, 1.0, 0.0)
    u = normalize(cross(up, d))
    v = normalize(cross(d, u))
    return {"id": safe_name(name), "name": name, "u": u, "v": v, "depth": d}


AXIS_COLORS = {
    "x": bytes([216, 74, 74, 255]),
    "y": bytes([69, 179, 107, 255]),
    "z": bytes([77, 121, 216, 255]),
}
JOINT_COLOR = bytes([245, 184, 72, 255])
JOINT_DARK = bytes([42, 28, 6, 255])
BLACK = bytes([18, 18, 18, 255])
LINE_FILL_BASE = (232, 239, 247)
LINE_FILL_SHADE = (214, 225, 237)
LINE_EDGE = bytes([42, 58, 75, 255])
LINE_CREASE = bytes([93, 112, 132, 255])
DRAWING_THEMES = {
    "light": {
        "background": (255, 255, 255),
        "fill_base": LINE_FILL_BASE,
        "fill_shade": LINE_FILL_SHADE,
        "edge": (42, 58, 75),
        "crease": (93, 112, 132),
        "origin": (18, 18, 18),
        "origin_core": (255, 255, 255),
        "joint": (245, 184, 72),
        "joint_dark": (42, 28, 6),
        "annotation": (34, 48, 64),
        "annotation_muted": (92, 106, 122),
        "annotation_bg": (255, 255, 255),
    },
    "dark": {
        "background": (7, 10, 16),
        "fill_base": (126, 132, 141),
        "fill_shade": (78, 86, 98),
        "edge": (246, 248, 252),
        "crease": (198, 205, 216),
        "origin": (245, 247, 250),
        "origin_core": (7, 10, 16),
        "joint": (255, 210, 105),
        "joint_dark": (58, 40, 7),
        "annotation": (246, 248, 252),
        "annotation_muted": (190, 198, 210),
        "annotation_bg": (7, 10, 16),
    },
}


def rgba(rgb):
    return bytes([int(rgb[0]), int(rgb[1]), int(rgb[2]), 255])


def drawing_themes(value: str) -> list[str]:
    theme = (value or "light").strip().lower()
    if theme == "both":
        return ["light", "dark"]
    if theme in DRAWING_THEMES:
        return [theme]
    raise ValueError("--theme must be light, dark, or both")

TINY_FONT = {
    " ": ["0", "0", "0", "0", "0", "0", "0"],
    ".": ["0", "0", "0", "0", "0", "0", "1"],
    ",": ["0", "0", "0", "0", "0", "1", "1"],
    "-": ["00000", "00000", "00000", "11111", "00000", "00000", "00000"],
    "/": ["00001", "00010", "00010", "00100", "01000", "01000", "10000"],
    ":": ["0", "1", "0", "0", "0", "1", "0"],
    "0": ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "3": ["11110", "00001", "00001", "01110", "00001", "00001", "11110"],
    "4": ["10010", "10010", "10010", "11111", "00010", "00010", "00010"],
    "5": ["11111", "10000", "10000", "11110", "00001", "00001", "11110"],
    "6": ["01111", "10000", "10000", "11110", "10001", "10001", "01110"],
    "7": ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    "8": ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
    "9": ["01110", "10001", "10001", "01111", "00001", "00001", "11110"],
    "A": ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    "B": ["11110", "10001", "10001", "11110", "10001", "10001", "11110"],
    "C": ["01111", "10000", "10000", "10000", "10000", "10000", "01111"],
    "D": ["11110", "10001", "10001", "10001", "10001", "10001", "11110"],
    "E": ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    "F": ["11111", "10000", "10000", "11110", "10000", "10000", "10000"],
    "G": ["01111", "10000", "10000", "10011", "10001", "10001", "01111"],
    "H": ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
    "I": ["11111", "00100", "00100", "00100", "00100", "00100", "11111"],
    "J": ["00111", "00010", "00010", "00010", "00010", "10010", "01100"],
    "K": ["10001", "10010", "10100", "11000", "10100", "10010", "10001"],
    "L": ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
    "M": ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
    "N": ["10001", "11001", "10101", "10011", "10001", "10001", "10001"],
    "O": ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
    "P": ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
    "Q": ["01110", "10001", "10001", "10001", "10101", "10010", "01101"],
    "R": ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    "S": ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
    "T": ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
    "U": ["10001", "10001", "10001", "10001", "10001", "10001", "01110"],
    "V": ["10001", "10001", "10001", "10001", "10001", "01010", "00100"],
    "W": ["10001", "10001", "10001", "10101", "10101", "10101", "01010"],
    "X": ["10001", "10001", "01010", "00100", "01010", "10001", "10001"],
    "Y": ["10001", "10001", "01010", "00100", "00100", "00100", "00100"],
    "Z": ["11111", "00001", "00010", "00100", "01000", "10000", "11111"],
}


def finite_vec3(value):
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return None
    try:
        out = (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in out):
        return None
    return out


def vector_m_to_mm(value):
    v = finite_vec3(value)
    return [v[0] * 1000.0, v[1] * 1000.0, v[2] * 1000.0] if v else None


def put_pixel(pixels, width, height, x, y, color):
    if 0 <= x < width and 0 <= y < height:
        i = (y * width + x) * 4
        pixels[i : i + 4] = color


def draw_disc(pixels, width, height, cx, cy, color, radius=2):
    cx = int(round(cx))
    cy = int(round(cy))
    r2 = radius * radius
    for y in range(cy - radius, cy + radius + 1):
        for x in range(cx - radius, cx + radius + 1):
            if (x - cx) * (x - cx) + (y - cy) * (y - cy) <= r2:
                put_pixel(pixels, width, height, x, y, color)


def draw_line_thick(pixels, width, height, x0, y0, x1, y1, color, radius=2):
    steps = max(1, int(math.hypot(x1 - x0, y1 - y0)))
    for i in range(steps + 1):
        t = i / steps
        draw_disc(pixels, width, height, x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, color, radius)


def draw_arrow_head(pixels, width, height, start, end, color, radius=1):
    x0, y0 = start
    x1, y1 = end
    angle = math.atan2(y1 - y0, x1 - x0)
    length = 13
    spread = 0.62
    for sign in (-1, 1):
        wing = angle + math.pi + sign * spread
        wx = x1 + math.cos(wing) * length
        wy = y1 + math.sin(wing) * length
        draw_line_thick(pixels, width, height, x1, y1, wx, wy, color, radius=radius)


def draw_rect(pixels, width, height, x0, y0, x1, y1, fill, border=None):
    left = max(0, int(round(min(x0, x1))))
    right = min(width - 1, int(round(max(x0, x1))))
    top = max(0, int(round(min(y0, y1))))
    bottom = min(height - 1, int(round(max(y0, y1))))
    for y in range(top, bottom + 1):
        for x in range(left, right + 1):
            if border and (x == left or x == right or y == top or y == bottom):
                put_pixel(pixels, width, height, x, y, border)
            else:
                put_pixel(pixels, width, height, x, y, fill)


def draw_text(pixels, width, height, text, x, y, color, scale=2):
    cursor = int(round(x))
    top = int(round(y))
    for ch in str(text).upper():
        glyph = TINY_FONT.get(ch)
        if not glyph:
            cursor += 4 * scale
            continue
        for row, bits in enumerate(glyph):
            for col, bit in enumerate(bits):
                if bit == "1":
                    draw_rect(
                        pixels,
                        width,
                        height,
                        cursor + col * scale,
                        top + row * scale,
                        cursor + (col + 1) * scale - 1,
                        top + (row + 1) * scale - 1,
                        color,
                    )
        cursor += (len(glyph[0]) + 1) * scale


def text_size(text: str, scale=2):
    width = 0
    height = 7 * scale
    for ch in str(text).upper():
        glyph = TINY_FONT.get(ch, TINY_FONT[" "])
        width += (max(len(row) for row in glyph) + 1) * scale
    return max(0, width - scale), height


def draw_label_box(pixels, width, height, text, x, y, theme, scale=2, align="center"):
    label = str(text).upper()
    text_w, text_h = text_size(label, scale)
    pad_x = 5
    pad_y = 3
    if align == "center":
        left = x - text_w / 2 - pad_x
    elif align == "right":
        left = x - text_w - 2 * pad_x
    else:
        left = x
    top = y - text_h / 2 - pad_y
    left = max(2, min(width - text_w - 2 * pad_x - 2, left))
    top = max(2, min(height - text_h - 2 * pad_y - 2, top))
    bg = rgba(theme["annotation_bg"])
    fg = rgba(theme["annotation"])
    draw_rect(pixels, width, height, left, top, left + text_w + 2 * pad_x, top + text_h + 2 * pad_y, bg, border=fg)
    draw_text(pixels, width, height, label, left + pad_x, top + pad_y, fg, scale=scale)
    return (left, top, left + text_w + 2 * pad_x, top + text_h + 2 * pad_y)


def format_mm(value: float) -> str:
    value = float(value)
    if abs(value) >= 100:
        return f"{value:.0f} MM"
    if abs(value) >= 10:
        return f"{value:.1f} MM"
    return f"{value:.2f} MM"


def fill_triangle(pixels, zbuf, width, height, p0, p1, p2, color):
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    min_x = max(0, int(math.floor(min(x0, x1, x2))))
    max_x = min(width - 1, int(math.ceil(max(x0, x1, x2))))
    min_y = max(0, int(math.floor(min(y0, y1, y2))))
    max_y = min(height - 1, int(math.ceil(max(y0, y1, y2))))
    area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
    if abs(area) < 1e-6:
        return
    inv_area = 1.0 / area
    for y in range(min_y, max_y + 1):
        py = y + 0.5
        row = y * width
        for x in range(min_x, max_x + 1):
            px = x + 0.5
            w0 = ((x1 - px) * (y2 - py) - (y1 - py) * (x2 - px)) * inv_area
            w1 = ((x2 - px) * (y0 - py) - (y2 - py) * (x0 - px)) * inv_area
            w2 = 1.0 - w0 - w1
            if w0 < -0.0001 or w1 < -0.0001 or w2 < -0.0001:
                continue
            z = w0 * z0 + w1 * z1 + w2 * z2
            idx = row + x
            if z <= zbuf[idx]:
                continue
            zbuf[idx] = z
            pi = idx * 4
            pixels[pi : pi + 4] = color


def draw_line_visible(pixels, zbuf, width, height, p0, p1, color, radius=0, depth_tolerance=1.0):
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    steps = max(1, int(math.hypot(x1 - x0, y1 - y0)))
    for i in range(steps + 1):
        t = i / steps
        x = x0 + (x1 - x0) * t
        y = y0 + (y1 - y0) * t
        z = z0 + (z1 - z0) * t
        ix = int(round(x))
        iy = int(round(y))
        if not (0 <= ix < width and 0 <= iy < height):
            continue
        if z < zbuf[iy * width + ix] - depth_tolerance:
            continue
        draw_disc(pixels, width, height, x, y, color, radius)


def projection_frame(projected, size, overlay_points=None, margin_fraction=0.01):
    points = list(projected)
    if overlay_points:
        points.extend(overlay_points)
    width = height = size
    margin = max(4, int(size * margin_fraction))
    min_u = min(p[0] for p in points)
    max_u = max(p[0] for p in points)
    min_v = min(p[1] for p in points)
    max_v = max(p[1] for p in points)
    span_u = max(max_u - min_u, 1.0)
    span_v = max(max_v - min_v, 1.0)
    scale = min((width - 2 * margin) / span_u, (height - 2 * margin) / span_v)
    offset_u = (width - span_u * scale) / 2
    offset_v = (height - span_v * scale) / 2
    return {
        "width": width,
        "height": height,
        "min_u": min_u,
        "min_v": min_v,
        "scale": scale,
        "scale_u": scale,
        "scale_v": scale,
        "offset_u": offset_u,
        "offset_v": offset_v,
    }


def world_to_screen(point, view, frame):
    u, v, depth = project(point, view)
    sx = frame["offset_u"] + (u - frame["min_u"]) * frame.get("scale_u", frame["scale"])
    sy = frame["height"] - (frame["offset_v"] + (v - frame["min_v"]) * frame.get("scale_v", frame["scale"]))
    return sx, sy, depth


def shade_color(normal, depth, min_depth, depth_span, view, theme):
    view_depth = normalize(view["depth"])
    view_up = normalize(view["v"])
    view_right = normalize(view["u"])
    light_a = normalize(
        (
            -0.45 * view_right[0] - 0.25 * view_up[0] + 0.95 * view_depth[0],
            -0.45 * view_right[1] - 0.25 * view_up[1] + 0.95 * view_depth[1],
            -0.45 * view_right[2] - 0.25 * view_up[2] + 0.95 * view_depth[2],
        )
    )
    light_b = normalize(
        (
            0.55 * view_right[0] + 0.55 * view_up[0] + 0.35 * view_depth[0],
            0.55 * view_right[1] + 0.55 * view_up[1] + 0.35 * view_depth[1],
            0.55 * view_right[2] + 0.55 * view_up[2] + 0.35 * view_depth[2],
        )
    )
    facing = abs(dot(normal, view_depth))
    diffuse = 0.35 * max(0.0, dot(normal, light_a)) + 0.20 * max(0.0, dot(normal, light_b)) + 0.20 * facing
    depth_tint = 0.08 * ((depth - min_depth) / depth_span)
    shade = max(0.0, min(1.0, diffuse + depth_tint))
    fill_shade = theme["fill_shade"]
    fill_base = theme["fill_base"]
    rgb = tuple(int(fill_shade[i] + (fill_base[i] - fill_shade[i]) * shade) for i in range(3))
    return bytes([rgb[0], rgb[1], rgb[2], 255])


def draw_feature_edges(pixels, zbuf, width, height, screen, triangles, face_records, depth_span, theme):
    edge_faces = {}
    for face_idx, (_, a, b, c, _, _, _) in enumerate(face_records):
        for edge in ((a, b), (b, c), (c, a)):
            key = tuple(sorted(edge))
            edge_faces.setdefault(key, []).append(face_idx)

    crease_threshold = math.cos(math.radians(32.0))
    depth_tolerance = max(0.5, depth_span * 0.018)
    for (a, b), face_indices in edge_faces.items():
        if len(face_indices) == 1:
            color = rgba(theme["edge"])
            radius = 0
        else:
            records = [face_records[i] for i in face_indices]
            front_flags = [record[5] for record in records]
            if any(front_flags) and not all(front_flags):
                color = rgba(theme["edge"])
                radius = 0
            else:
                normals = [record[4] for record in records]
                is_crease = any(dot(normals[i], normals[j]) < crease_threshold for i in range(len(normals)) for j in range(i + 1, len(normals)))
                if not is_crease or not any(front_flags):
                    continue
                color = rgba(theme["crease"])
                radius = 0
        draw_line_visible(pixels, zbuf, width, height, screen[a], screen[b], color, radius=radius, depth_tolerance=depth_tolerance)


def screen_bounds(screen):
    xs = [p[0] for p in screen]
    ys = [p[1] for p in screen]
    return min(xs), min(ys), max(xs), max(ys)


def draw_dimension_line(pixels, width, height, start, end, label, theme, label_offset=(0, 0)):
    color = rgba(theme["annotation"])
    muted = rgba(theme["annotation_muted"])
    x0, y0 = start
    x1, y1 = end
    draw_line_thick(pixels, width, height, x0, y0, x1, y1, color, radius=0)
    draw_arrow_head(pixels, width, height, (x0, y0), (x1, y1), color)
    draw_arrow_head(pixels, width, height, (x1, y1), (x0, y0), color)
    if abs(x1 - x0) >= abs(y1 - y0):
        draw_line_thick(pixels, width, height, x0, y0 - 8, x0, y0 + 8, muted, radius=0)
        draw_line_thick(pixels, width, height, x1, y1 - 8, x1, y1 + 8, muted, radius=0)
    else:
        draw_line_thick(pixels, width, height, x0 - 8, y0, x0 + 8, y0, muted, radius=0)
        draw_line_thick(pixels, width, height, x1 - 8, y1, x1 + 8, y1, muted, radius=0)
    draw_label_box(
        pixels,
        width,
        height,
        label,
        (x0 + x1) / 2 + label_offset[0],
        (y0 + y1) / 2 + label_offset[1],
        theme,
        scale=2,
        align="center",
    )


def draw_view_dimensions(pixels, width, height, screen, shape, view, theme):
    if view["id"] not in {"front", "side", "top"}:
        return []
    box = shape.BoundingBox()
    dims = {
        "X": float(box.xlen),
        "Y": float(box.ylen),
        "Z": float(box.zlen),
    }
    left, top, right, bottom = screen_bounds(screen)
    h_label = "X" if view["id"] in {"front", "top"} else "Y"
    v_label = "Z" if view["id"] in {"front", "side"} else "Y"
    h_y = min(height - 26, bottom + max(22, height * 0.045))
    v_x = max(24, left - max(22, width * 0.045))
    color = rgba(theme["annotation_muted"])
    draw_line_thick(pixels, width, height, left, bottom + 4, left, h_y, color, radius=0)
    draw_line_thick(pixels, width, height, right, bottom + 4, right, h_y, color, radius=0)
    draw_dimension_line(pixels, width, height, (left, h_y), (right, h_y), f"{h_label} {format_mm(dims[h_label])}", theme, label_offset=(0, -15))
    draw_line_thick(pixels, width, height, left - 4, top, v_x, top, color, radius=0)
    draw_line_thick(pixels, width, height, left - 4, bottom, v_x, bottom, color, radius=0)
    draw_dimension_line(pixels, width, height, (v_x, top), (v_x, bottom), f"{v_label} {format_mm(dims[v_label])}", theme, label_offset=(38, 0))
    return [
        {"axis": h_label, "value_mm": dims[h_label]},
        {"axis": v_label, "value_mm": dims[v_label]},
    ]


def compact_number(value, digits=2):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return ""
    if not math.isfinite(value):
        return ""
    return f"{value:.{digits}f}".rstrip("0").rstrip(".")


def callout_labels(metadata: dict, shape) -> list[str]:
    labels = []
    mass = compact_number(metadata.get("mass_kg"), 2)
    if mass:
        labels.append(f"MASS {mass} KG")
    materials = metadata.get("material_names") or []
    if materials:
        labels.append(f"MAT {str(materials[0])[:12]}")
    bodies = metadata.get("body_count")
    if bodies is not None:
        labels.append(f"BODIES {bodies}")
    volume = compact_number(metadata.get("volume_cm3"), 1)
    if volume:
        labels.append(f"VOL {volume} CM3")
    center = finite_vec3(metadata.get("center_of_gravity_mm")) or finite_vec3(metadata.get("geometry_center_mm"))
    if center:
        labels.append(f"COG {compact_number(center[0], 0)},{compact_number(center[1], 0)},{compact_number(center[2], 0)}")
    if not labels:
        box = shape.BoundingBox()
        labels.append(f"PART {compact_number(box.xlen, 0)}X{compact_number(box.ylen, 0)}X{compact_number(box.zlen, 0)} MM")
    return labels[:5]


def draw_callouts(pixels, width, height, screen, shape, view, frame, metadata, theme, mode):
    if mode == "dimensions" or view["id"] != "iso":
        return []
    left, top, right, _ = screen_bounds(screen)
    color = rgba(theme["annotation"])
    muted = rgba(theme["annotation_muted"])
    labels = callout_labels(metadata, shape)
    if mode == "basic":
        labels = labels[:3]
    box = shape.BoundingBox()
    anchor = world_to_screen(
        (
            (float(box.xmin) + float(box.xmax)) / 2,
            (float(box.ymin) + float(box.ymax)) / 2,
            (float(box.zmin) + float(box.zmax)) / 2,
        ),
        view,
        frame,
    )
    x = min(width - 16, max(right + 12, width * 0.66))
    y = max(18, top + 8)
    records = []
    for idx, label in enumerate(labels):
        box_rect = draw_label_box(pixels, width, height, label, x, y + idx * 26, theme, scale=2, align="left")
        records.append({"label": label})
        if idx == 0:
            tail_start = (box_rect[0], (box_rect[1] + box_rect[3]) / 2)
            draw_line_thick(pixels, width, height, tail_start[0], tail_start[1], anchor[0], anchor[1], muted, radius=0)
            draw_disc(pixels, width, height, anchor[0], anchor[1], color, radius=3)
    return records


def draw_annotations(pixels, width, height, screen, shape, view, frame, metadata, theme, mode):
    if mode == "none":
        return {"dimensions": [], "callouts": []}
    dimensions = draw_view_dimensions(pixels, width, height, screen, shape, view, theme)
    callouts = draw_callouts(pixels, width, height, screen, shape, view, frame, metadata, theme, mode)
    return {"dimensions": dimensions, "callouts": callouts}


def shape_bbox(shape) -> dict:
    box = shape.BoundingBox()
    return {
        "min": [float(box.xmin), float(box.ymin), float(box.zmin)],
        "max": [float(box.xmax), float(box.ymax), float(box.zmax)],
    }


def points_bbox(points: list[tuple[float, float, float]]) -> dict:
    if not points:
        return {"min": [], "max": []}
    return {
        "min": [min(p[i] for p in points) for i in range(3)],
        "max": [max(p[i] for p in points) for i in range(3)],
    }


def bbox_max_delta(a: dict, b: dict) -> float | None:
    amin = a.get("min") or []
    amax = a.get("max") or []
    bmin = b.get("min") or []
    bmax = b.get("max") or []
    if len(amin) < 3 or len(amax) < 3 or len(bmin) < 3 or len(bmax) < 3:
        return None
    values = [abs(amin[i] - bmin[i]) for i in range(3)]
    values.extend(abs(amax[i] - bmax[i]) for i in range(3))
    return max(values)


def shell_sample_views() -> list[dict]:
    directions = [
        ((1.0, 0.0, 0.0), "pos-x"),
        ((-1.0, 0.0, 0.0), "neg-x"),
        ((0.0, 1.0, 0.0), "pos-y"),
        ((0.0, -1.0, 0.0), "neg-y"),
        ((0.0, 0.0, 1.0), "pos-z"),
        ((0.0, 0.0, -1.0), "neg-z"),
        ((1.0, 1.0, 1.0), "diag-ppp"),
        ((1.0, 1.0, -1.0), "diag-ppm"),
        ((1.0, -1.0, 1.0), "diag-pmp"),
        ((1.0, -1.0, -1.0), "diag-pmm"),
        ((-1.0, 1.0, 1.0), "diag-mpp"),
        ((-1.0, 1.0, -1.0), "diag-mpm"),
        ((-1.0, -1.0, 1.0), "diag-mmp"),
        ((-1.0, -1.0, -1.0), "diag-mmm"),
    ]
    return [view_from_depth(direction, name) for direction, name in directions]


def raster_visible_faces(points, triangles, view: dict, size: int) -> set[int]:
    projected = [project(p, view) for p in points]
    if not projected:
        return set()
    frame = projection_frame(projected, size, margin_fraction=0.02)
    width = frame["width"]
    height = frame["height"]
    screen = []
    for u, v, depth in projected:
        sx = frame["offset_u"] + (u - frame["min_u"]) * frame["scale_u"]
        sy = height - (frame["offset_v"] + (v - frame["min_v"]) * frame["scale_v"])
        screen.append((sx, sy, depth))
    zbuf = [-1e30] * (width * height)
    facebuf = [-1] * (width * height)
    view_depth = normalize(view["depth"])
    for face_idx, tri in enumerate(triangles):
        a, b, c = (int(i) for i in tri)
        normal = normalize(cross(sub(points[b], points[a]), sub(points[c], points[a])))
        if dot(normal, view_depth) < -0.02:
            continue
        x0, y0, z0 = screen[a]
        x1, y1, z1 = screen[b]
        x2, y2, z2 = screen[c]
        min_x = max(0, int(math.floor(min(x0, x1, x2))))
        max_x = min(width - 1, int(math.ceil(max(x0, x1, x2))))
        min_y = max(0, int(math.floor(min(y0, y1, y2))))
        max_y = min(height - 1, int(math.ceil(max(y0, y1, y2))))
        area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
        if abs(area) < 1e-9:
            continue
        for y in range(min_y, max_y + 1):
            py = y + 0.5
            for x in range(min_x, max_x + 1):
                px = x + 0.5
                w0 = ((x1 - px) * (y2 - py) - (y1 - py) * (x2 - px)) / area
                w1 = ((x2 - px) * (y0 - py) - (y2 - py) * (x0 - px)) / area
                w2 = 1.0 - w0 - w1
                if w0 < -0.001 or w1 < -0.001 or w2 < -0.001:
                    continue
                depth = w0 * z0 + w1 * z1 + w2 * z2
                idx = y * width + x
                if depth > zbuf[idx]:
                    zbuf[idx] = depth
                    facebuf[idx] = face_idx
    return {idx for idx in facebuf if idx >= 0}


def external_shell_faces(points, triangles, raster_size=180) -> tuple[set[int], int]:
    visible = set()
    views = shell_sample_views()
    for view in views:
        visible.update(raster_visible_faces(points, triangles, view, raster_size))
    return visible, len(views)


def external_shell_report(points, triangles, cad_bbox: dict, visible: set[int], sample_view_count: int, raster_size=180) -> dict:
    total = len(triangles)
    if total <= 0 or not points:
        return {
            "method": "multi_view_depth_visibility",
            "triangle_count": 0,
            "total_triangle_count": total,
            "triangle_ratio": 0.0,
            "bbox": {"min": [], "max": []},
            "bbox_error_mm": None,
            "sample_view_count": 0,
        }
    shell_points = []
    for face_idx in sorted(visible):
        a, b, c = (int(i) for i in triangles[face_idx])
        shell_points.extend([points[a], points[b], points[c]])
    shell_bbox = points_bbox(shell_points)
    bbox_error = bbox_max_delta(cad_bbox, shell_bbox)
    return {
        "method": "multi_view_depth_visibility",
        "notes": [
            "Prototype external shell estimate: triangles visible in at least one sampled orthographic depth pass.",
            "Use PyMeshLab ambient-occlusion filtering or MeshLib booleans for production-grade hidden/internal face removal.",
        ],
        "triangle_count": len(visible),
        "total_triangle_count": total,
        "triangle_ratio": len(visible) / total if total else 0.0,
        "bbox": shell_bbox,
        "bbox_error_mm": bbox_error,
        "sample_view_count": sample_view_count,
        "raster_size": raster_size,
    }


def write_binary_stl(path: Path, points, triangles, face_indices: set[int]) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    ordered = sorted(face_indices)
    data = bytearray(80)
    data.extend(struct.pack("<I", len(ordered)))
    for face_idx in ordered:
        a, b, c = (int(i) for i in triangles[face_idx])
        p0, p1, p2 = points[a], points[b], points[c]
        normal = normalize(cross(sub(p1, p0), sub(p2, p0)))
        data.extend(struct.pack("<3f", *normal))
        data.extend(struct.pack("<3f", *p0))
        data.extend(struct.pack("<3f", *p1))
        data.extend(struct.pack("<3f", *p2))
        data.extend(struct.pack("<H", 0))
    path.write_bytes(data)
    return len(ordered)


def mass_props(shape) -> dict:
    props = {"source": "cadquery_geometry"}
    try:
        volume_mm3 = float(shape.Volume())
        props["volume_mm3"] = volume_mm3
        props["volume_cm3"] = volume_mm3 / 1000.0
    except Exception:
        pass
    try:
        center = shape.Center()
        props["geometry_center_mm"] = [float(center.x), float(center.y), float(center.z)]
    except Exception:
        pass
    return props


def load_fusion_metadata(path: str, source: Path) -> dict:
    candidates = []
    if path:
        candidates.append(Path(path).expanduser())
    candidates.extend(
        [
            source / "parts_metadata.json",
            source / "fusion_parts_metadata.json",
            source / "metadata.json",
        ]
    )
    for candidate in candidates:
        if not candidate.exists():
            continue
        data = json.loads(candidate.read_text(encoding="utf-8"))
        return {"path": str(candidate.resolve()), "data": data, "parts": build_part_metadata_index(data)}
    return {"path": "", "data": {}, "parts": {}}


def build_part_metadata_index(data: dict) -> dict:
    index = {}
    parts = data.get("parts", []) if isinstance(data, dict) else []
    for part in parts:
        for key in metadata_keys(part):
            index.setdefault(key, part)
    return index


def metadata_keys(part: dict) -> list[str]:
    keys = []
    for value in [
        part.get("id"),
        part.get("name"),
        part.get("occurrence_name"),
        part.get("component_name"),
        part.get("full_path"),
        *(part.get("aliases") or []),
    ]:
        if value is None:
            continue
        text = str(value).strip()
        if not text:
            continue
        keys.append(text.lower())
        keys.append(safe_name(text).lower())
        if ":" in text:
            keys.append(safe_name(text.split(":", 1)[0]).lower())
    deduped = []
    seen = set()
    for key in keys:
        if key not in seen:
            deduped.append(key)
            seen.add(key)
    return deduped


def part_metadata_for(index: dict, part: str) -> dict:
    for key in [part.lower(), safe_name(part).lower()]:
        if key in index:
            return dict(index[key])
    return {}


def metadata_vec_mm(record: dict, mm_key: str, m_key: str):
    value = finite_vec3(record.get(mm_key))
    if value:
        return [value[0], value[1], value[2]]
    value = vector_m_to_mm(record.get(m_key))
    if value:
        return value
    return None


def enriched_part_metadata(part_metadata: dict, geometry_props: dict) -> dict:
    out = dict(part_metadata or {})
    if "metadata_source" not in out and part_metadata:
        out["metadata_source"] = "autodesk_fusion_api"
    if "center_of_gravity_mm" not in out:
        center = metadata_vec_mm(out, "center_of_gravity_mm", "center_of_gravity_m")
        if center:
            out["center_of_gravity_mm"] = center
    if "origin_mm" not in out:
        origin = metadata_vec_mm(out, "origin_mm", "origin_m")
        if origin:
            out["origin_mm"] = origin
    if "volume_cm3" not in out and "volume_cm3" in geometry_props:
        out["volume_cm3"] = geometry_props["volume_cm3"]
    if "volume_mm3" not in out and "volume_mm3" in geometry_props:
        out["volume_mm3"] = geometry_props["volume_mm3"]
    if "geometry_center_mm" not in out and "geometry_center_mm" in geometry_props:
        out["geometry_center_mm"] = geometry_props["geometry_center_mm"]
    if "center_of_gravity_mm" not in out and "geometry_center_mm" in geometry_props:
        out["center_of_gravity_mm"] = geometry_props["geometry_center_mm"]
        out["center_of_gravity_source"] = "cadquery_geometry_center"
    if "density_kg_m3" in out and "mass_kg" not in out and "volume_mm3" in out:
        volume_m3 = float(out["volume_mm3"]) * 1.0e-9
        if volume_m3 > 0:
            out["mass_kg"] = float(out["density_kg_m3"]) * volume_m3
            out["mass_source"] = "density_times_cadquery_volume"
    if "mass_kg" in out and "density_kg_m3" not in out and "volume_mm3" in out:
        volume_m3 = float(out["volume_mm3"]) * 1.0e-9
        if volume_m3 > 0:
            out["density_kg_m3"] = float(out["mass_kg"]) / volume_m3
            out["density_source"] = "mass_over_cadquery_volume"
    return out


def joint_origin_mm(joint: dict):
    return metadata_vec_mm(joint, "origin_mm", "origin_m")


def overlay_anchor_points(metadata: dict, shape, view: dict) -> list[tuple[float, float, float]]:
    points = []
    box = shape.BoundingBox()
    max_dim = max(float(box.xlen), float(box.ylen), float(box.zlen), 1.0)
    axis_len = max(25.0, 0.14 * max_dim)
    origin = finite_vec3(metadata.get("origin_mm")) or (0.0, 0.0, 0.0)
    points.append(project(origin, view))
    for axis in ((axis_len, 0.0, 0.0), (0.0, axis_len, 0.0), (0.0, 0.0, axis_len)):
        points.append(project((origin[0] + axis[0], origin[1] + axis[1], origin[2] + axis[2]), view))
    for joint in metadata.get("joints") or []:
        joint_origin = joint_origin_mm(joint)
        if joint_origin:
            points.append(project(joint_origin, view))
            axis = finite_vec3(joint.get("axis")) or (0.0, 0.0, 1.0)
            points.append(
                project(
                    (
                        joint_origin[0] + axis[0] * axis_len * 0.45,
                        joint_origin[1] + axis[1] * axis_len * 0.45,
                        joint_origin[2] + axis[2] * axis_len * 0.45,
                    ),
                    view,
                )
            )
    return points


def screen_axis_vectors(view, axis_len_px):
    origin = project((0.0, 0.0, 0.0), view)
    axes = {
        "x": project((1.0, 0.0, 0.0), view),
        "y": project((0.0, 1.0, 0.0), view),
        "z": project((0.0, 0.0, 1.0), view),
    }
    vectors = {}
    for label, endpoint in axes.items():
        dx = endpoint[0] - origin[0]
        dy = -(endpoint[1] - origin[1])
        length = math.hypot(dx, dy)
        if length < 1.0e-6:
            vectors[label] = None
        else:
            vectors[label] = (dx / length * axis_len_px, dy / length * axis_len_px)
    return vectors


def draw_screen_gizmo(pixels, width, height, view, theme) -> dict:
    axis_len_px = max(46, int(min(width, height) * 0.075))
    vectors = screen_axis_vectors(view, axis_len_px)
    pad = max(18, int(min(width, height) * 0.025))
    ox = pad + axis_len_px * 0.75
    oy = height - pad - axis_len_px * 0.75

    endpoints = [(ox, oy)]
    for vector in vectors.values():
        if vector:
            endpoints.append((ox + vector[0], oy + vector[1]))
    min_x = min(point[0] for point in endpoints) - 20
    max_x = max(point[0] for point in endpoints) + 26
    min_y = min(point[1] for point in endpoints) - 26
    max_y = max(point[1] for point in endpoints) + 20
    if min_x < pad:
        ox += pad - min_x
    if max_x > width - pad:
        ox -= max_x - (width - pad)
    if min_y < pad:
        oy += pad - min_y
    if max_y > height - pad:
        oy -= max_y - (height - pad)

    draw_disc(pixels, width, height, ox, oy, rgba(theme["origin"]), radius=5)
    draw_disc(pixels, width, height, ox, oy, rgba(theme["origin_core"]), radius=3)
    for label, vector in vectors.items():
        color = AXIS_COLORS[label]
        if vector is None:
            draw_disc(pixels, width, height, ox + 12, oy - 12, color, radius=4)
            draw_text(pixels, width, height, label, ox + 21, oy - 21, color, scale=3)
            continue
        ex = ox + vector[0]
        ey = oy + vector[1]
        draw_line_thick(pixels, width, height, ox, oy, ex, ey, color, radius=3)
        draw_arrow_head(pixels, width, height, (ox, oy), (ex, ey), color)
        draw_text(pixels, width, height, label, ex + 7, ey - 7, color, scale=3)
    return {
        "screen_anchor": "bottom_left",
        "length_px": axis_len_px,
        "axes": {
            "x": {"color": "#d84a4a"},
            "y": {"color": "#45b36b"},
            "z": {"color": "#4d79d8"},
        },
    }


def is_screen_visible(x, y, width, height, pad=20):
    return -pad <= x <= width + pad and -pad <= y <= height + pad


def draw_overlays(pixels, width, height, view, frame, metadata, shape, theme) -> dict:
    box = shape.BoundingBox()
    max_dim = max(float(box.xlen), float(box.ylen), float(box.zlen), 1.0)
    axis_len = max(25.0, 0.14 * max_dim)
    origin = finite_vec3(metadata.get("origin_mm")) or (0.0, 0.0, 0.0)
    gizmo = draw_screen_gizmo(pixels, width, height, view, theme)

    joint_records = []
    for idx, joint in enumerate(metadata.get("joints") or [], start=1):
        joint_origin = joint_origin_mm(joint)
        if not joint_origin:
            continue
        jx, jy, _ = world_to_screen(joint_origin, view, frame)
        if not is_screen_visible(jx, jy, width, height):
            continue
        axis = finite_vec3(joint.get("axis")) or (0.0, 0.0, 1.0)
        tip = (
            joint_origin[0] + axis[0] * axis_len * 0.45,
            joint_origin[1] + axis[1] * axis_len * 0.45,
            joint_origin[2] + axis[2] * axis_len * 0.45,
        )
        tx, ty, _ = world_to_screen(tip, view, frame)
        if not is_screen_visible(tx, ty, width, height):
            tx, ty = jx + 22, jy - 22
        joint_color = rgba(theme["joint"])
        joint_dark = rgba(theme["joint_dark"])
        draw_line_thick(pixels, width, height, jx, jy, tx, ty, joint_color, radius=2)
        draw_disc(pixels, width, height, jx, jy, joint_dark, radius=5)
        draw_disc(pixels, width, height, jx, jy, joint_color, radius=3)
        flag_x = tx + 7
        flag_y = ty - 20
        draw_rect(pixels, width, height, flag_x, flag_y, flag_x + 44, flag_y + 20, joint_color, border=joint_dark)
        draw_text(pixels, width, height, f"J{idx}", flag_x + 5, flag_y + 5, joint_dark, scale=2)
        joint_records.append(
            {
                "name": joint.get("name", f"joint{idx}"),
                "type": joint.get("type", "unknown"),
                "origin_mm": joint_origin,
                "axis": list(axis),
            }
        )
    return {
        "origin_mm": [origin[0], origin[1], origin[2]],
        **gizmo,
        "length_mm": axis_len,
        "joints": joint_records,
    }


def render_view_png(shape, points, triangles, png_path: Path, size: int, view: dict, metadata: dict, theme: dict, annotations: str) -> dict:
    projected = [project(p, view) for p in points]
    if not projected:
        raise RuntimeError(f"{png_path} has zero projected vertices")

    frame = projection_frame(projected, size, margin_fraction=0.14 if annotations != "none" else 0.01)
    width = frame["width"]
    height = frame["height"]
    screen = []
    for u, v, depth in projected:
        sx = frame["offset_u"] + (u - frame["min_u"]) * frame["scale_u"]
        sy = height - (frame["offset_v"] + (v - frame["min_v"]) * frame["scale_v"])
        screen.append((sx, sy, depth))

    min_depth = min(p[2] for p in screen)
    max_depth = max(p[2] for p in screen)
    depth_span = max(max_depth - min_depth, 1.0)
    pixels = bytearray(rgba(theme["background"]) * width * height)
    zbuf = [-1e30] * (width * height)

    faces = []
    view_depth = normalize(view["depth"])
    for tri in triangles:
        a, b, c = (int(i) for i in tri)
        normal = normalize(cross(sub(points[b], points[a]), sub(points[c], points[a])))
        front_facing = dot(normal, view_depth) >= -0.02
        # Cull back faces for cleaner, surface-like images instead of x-ray mesh drawings.
        if not front_facing:
            continue
        depth = (screen[a][2] + screen[b][2] + screen[c][2]) / 3.0
        faces.append((depth, a, b, c, normal, front_facing, shade_color(normal, depth, min_depth, depth_span, view, theme)))
    faces.sort()

    for _, a, b, c, _, _, color in faces:
        fill_triangle(pixels, zbuf, width, height, screen[a], screen[b], screen[c], color)
    draw_feature_edges(pixels, zbuf, width, height, screen, triangles, faces, depth_span, theme)
    annotation_records = draw_annotations(pixels, width, height, screen, shape, view, frame, metadata, theme, annotations)

    overlay = draw_overlays(pixels, width, height, view, frame, metadata, shape, theme)
    encode_png(png_path, width, height, pixels)
    return {
        "id": view["id"],
        "name": view["name"],
        "image": png_path.name,
        "triangle_count": len(faces),
        "gizmo": overlay,
        "annotations": annotation_records,
    }


def render_step_images(step_path: Path, part_dir: Path, size: int, deflection: float, force: bool, metadata: dict, themes: list[str], annotations: str) -> dict:
    import cadquery as cq

    imported = cq.importers.importStep(str(step_path))
    shape = imported.val() if hasattr(imported, "val") else imported
    vertices, triangles = shape.tessellate(deflection)
    points = [vec_tuple(v) for v in vertices]
    if not points:
        raise RuntimeError(f"{step_path} tessellated to zero vertices")
    geometry_props = mass_props(shape)
    metadata = enriched_part_metadata(metadata, geometry_props)
    cad_bbox = shape_bbox(shape)
    shell_faces, shell_view_count = external_shell_faces(points, triangles)
    shell_path = part_dir / "external_shell.stl"
    shell_triangle_count = write_binary_stl(shell_path, points, triangles, shell_faces)
    shell_report = external_shell_report(points, triangles, cad_bbox, shell_faces, shell_view_count)
    shell_report["mesh"] = shell_path.name
    shell_report["mesh_triangle_count"] = shell_triangle_count

    images = []
    for view in VIEWS:
        theme_records = {}
        default_record = None
        for theme_name in themes:
            theme = DRAWING_THEMES[theme_name]
            theme_dir = part_dir if len(themes) == 1 else part_dir / theme_name
            png_path = theme_dir / f"{view['id']}.png"
            if force or not png_path.exists():
                record = render_view_png(shape, points, triangles, png_path, size, view, metadata, theme, annotations)
            else:
                record = {
                    "id": view["id"],
                    "name": view["name"],
                    "image": png_path.name,
                    "triangle_count": len(triangles),
                }
            record["image"] = str(png_path.relative_to(part_dir)).replace("\\", "/")
            theme_records[theme_name] = record["image"]
            if default_record is None:
                default_record = record
        if default_record is not None:
            default_record["theme_images"] = theme_records
            images.append(default_record)
    return {
        "vertex_count": len(points),
        "triangle_count": len(triangles),
        "bbox": cad_bbox,
        "external_shell": shell_report,
        "mass_properties": geometry_props,
        "fusion_metadata": metadata,
        "images": images,
    }


GROUP_LABELS = {
    "cell-floor": "Cell floor",
    "h-frame": "H-frame",
    "positioner": "Positioner tables",
    "robot-arm": "Rosie 1400 arm",
    "cell": "Cell parts",
}


def part_group(part: str) -> str:
    key = safe_name(part).lower()
    base_key = key.split("_", 1)[0]
    if key.startswith("fireball_f5_tcg10254_g2") or key == "floor":
        return "cell-floor"
    if key in {"h_frame", "h_frame_1", "base_v2_2_1", "positioner_gear_1", "old_h_block_assembly_1", "h_gear_1", "table_adapter_1"}:
        return "h-frame"
    if key.startswith("positioner_table"):
        return "positioner"
    if base_key in {"base", "l1", "l2", "l3", "l4", "l5"}:
        return "robot-arm"
    return "cell"


def parse_part_spec(value: str) -> dict:
    name, _, step_name = value.partition("=")
    name = name.strip()
    step_name = step_name.strip() or name
    return {"name": name, "step_name": step_name}


def find_step(source: Path, part: str) -> Path | None:
    candidates = [source / f"{part}.step", source / f"{part}.stp", source / safe_name(part) / "cad.step"]
    lower = part.lower()
    for child in source.glob("*"):
        if child.is_file() and child.suffix.lower() in {".step", ".stp"} and child.stem.lower() == lower:
            candidates.append(child)
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def build_tree(assembly: str, parts: list[dict]) -> dict:
    children = []
    for group_id, label in GROUP_LABELS.items():
        group_parts = [part for part in parts if part["group"] == group_id]
        if not group_parts:
            continue
        children.append(
            {
                "id": group_id,
                "name": label,
                "kind": "group",
                "children": [
                    {"id": part["id"], "name": part["name"], "kind": "part", "image": part["image"]}
                    for part in group_parts
                ],
            }
        )
    return {"id": safe_name(assembly), "name": assembly, "kind": "assembly", "children": children}


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Render shaded PNG images for a STEP part folder and write a browseable parts index.")
    parser.add_argument("--source", required=True, help="Directory containing <part>.step files")
    parser.add_argument("--out", required=True, help="Output public parts directory")
    parser.add_argument("--assembly", default="rosie_1400:1", help="Assembly/root label")
    parser.add_argument("--parts", default="base,L1,L2,L3,L4,L5", help="CSV part list to render")
    parser.add_argument("--metadata", default="", help="Autodesk Fusion parts metadata JSON")
    parser.add_argument("--size", type=int, default=500, help="Square PNG size")
    parser.add_argument("--deflection", type=float, default=20.0, help="CadQuery tessellation deflection")
    parser.add_argument("--theme", choices=["light", "dark", "both"], default="both", help="Drawing color theme")
    parser.add_argument("--annotations", choices=["none", "dimensions", "basic", "full"], default="basic", help="Dimension and callout overlays")
    parser.add_argument("--force", action="store_true", help="Re-render PNGs even when present")
    args = parser.parse_args(argv)

    source = Path(args.source).expanduser().resolve()
    out_dir = Path(args.out).expanduser().resolve()
    assembly_dir = out_dir / safe_name(args.assembly)
    parts = [parse_part_spec(part) for part in args.parts.split(",") if part.strip()]
    themes = drawing_themes(args.theme)
    fusion_metadata = load_fusion_metadata(args.metadata, source)
    rendered = []
    for part_spec in parts:
        part = part_spec["name"]
        step_name = part_spec["step_name"]
        step = find_step(source, step_name)
        if not step:
            print(f"parts-image: missing STEP for {part} ({step_name}) under {source}", file=sys.stderr)
            continue
        part_dir = assembly_dir / safe_name(part)
        default_image_path = part_dir / "iso.png"
        rel_image = str(default_image_path.relative_to(out_dir)).replace("\\", "/")
        metadata = {
            "vertex_count": 0,
            "triangle_count": 0,
            "bbox": {},
            "mass_properties": {},
            "fusion_metadata": {},
            "images": [],
        }
        part_metadata = part_metadata_for(fusion_metadata["parts"], part)
        if args.force or not default_image_path.exists():
            metadata = render_step_images(step, part_dir, args.size, args.deflection, args.force, part_metadata, themes, args.annotations)
            print(f"parts-image: wrote {part_dir}")
        else:
            metadata = render_step_images(step, part_dir, args.size, args.deflection, False, part_metadata, themes, args.annotations)
            print(f"parts-image: current {part_dir}")
        images = []
        for view in metadata.get("images", []):
            view_image = str((part_dir / view["image"]).relative_to(out_dir)).replace("\\", "/")
            theme_images = {
                theme: str((part_dir / image).relative_to(out_dir)).replace("\\", "/")
                for theme, image in (view.get("theme_images") or {}).items()
            }
            images.append({**view, "image": view_image, "theme_images": theme_images})
        item = {
            "id": safe_name(part),
            "name": part,
            "group": part_group(part),
            "assembly": args.assembly,
            "step_file": str(step),
            "image": rel_image,
            "images": images,
            **metadata,
        }
        item["images"] = images
        fusion_item = metadata.get("fusion_metadata") or {}
        for key in (
            "metadata_source",
            "origin_m",
            "origin_mm",
            "center_of_gravity_m",
            "center_of_gravity_mm",
            "center_of_gravity_source",
            "mass_kg",
            "mass_source",
            "density_kg_m3",
            "density_source",
            "volume_cm3",
            "area_cm2",
            "body_count",
            "material_names",
            "joints",
        ):
            if key in fusion_item:
                item[key] = fusion_item[key]
        (part_dir / "part.json").write_text(json.dumps(item, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        rendered.append(item)

    index = {
        "schema_version": 1,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "source_dir": str(source),
        "metadata": {
            "source": "autodesk_fusion_api" if fusion_metadata["path"] else "",
            "path": fusion_metadata["path"],
            "active_document": fusion_metadata["data"].get("active_document", ""),
            "generated_at": fusion_metadata["data"].get("generated_at", ""),
        },
        "assembly": args.assembly,
        "themes": themes,
        "default_theme": themes[0] if themes else "light",
        "annotations": args.annotations,
        "parts": rendered,
        "tree": build_tree(args.assembly, rendered),
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "index.json").write_text(json.dumps(index, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"parts-image: wrote {out_dir / 'index.json'} ({len(rendered)} parts)")
    return 0 if rendered else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:
        print(f"render_part_images: {exc}", file=sys.stderr)
        raise
