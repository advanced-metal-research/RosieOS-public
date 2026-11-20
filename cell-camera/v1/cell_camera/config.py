"""Settings for one camera, read from the environment and checked before any hardware is touched.

Everything that differs between cameras or hosts is here, so the capture and
HTTP code hold no device names, sizes or encoder choices of their own.
"""
from __future__ import annotations

import ipaddress
import re
from dataclasses import dataclass
from typing import Mapping, Optional, Tuple

PREFIX = "CELL_CAMERA_"
_DEVICE = re.compile(r"^/dev/[A-Za-z0-9._:+,/-]+$")
_MODE = re.compile(r"^(\d{2,5})x(\d{2,5})@(\d{1,3})$")
_SIZE = re.compile(r"^(\d{2,5})x(\d{2,5})$")
_CROP = re.compile(r"^(\d{2,5})x(\d{2,5})\+(\d{1,5})\+(\d{1,5})$")
_FORMAT = re.compile(r"^[A-Z0-9]{3,8}$")
_PATH = re.compile(r"^(/[A-Za-z0-9._-]+)*$")
_LABEL = re.compile(r"^[^\x00-\x1f<>\"&]{1,80}$")
_SERIAL = re.compile(r"^[A-Za-z0-9]{1,32}$")
ENCODERS = ("auto", "nvidia", "software")
SOURCES = ("v4l2", "realsense")


class ConfigError(ValueError):
    """A setting is missing or cannot describe a camera; the service refuses to start."""


@dataclass(frozen=True)
class Config:
    device: str                      # /dev/v4l/by-id/... : a stable name, never a bare index that moves; "" for a RealSense
    name: str                        # what status calls the camera
    title: str                       # the viewer page's heading
    capture: Tuple[int, int, int]    # width, height, frames per second the camera is asked for
    pixel_format: str                # the camera's raw format at that mode, for example YUY2
    output: Tuple[int, int]          # width, height of each JPEG; smaller than the crop scales down
    crop: Optional[Tuple[int, int, int, int]]  # width, height, left, top of the part of the capture kept; None keeps all
    jpeg_quality: int                # 1..100
    encoder: str                     # auto | nvidia | software
    listen: Tuple[str, int]          # loopback only: the OLP server on this host is the client
    path_prefix: str                 # routes live under it, for example /camera
    idle_stop_seconds: float         # capture stops this long after the last frame request
    max_frame_age_seconds: float     # a frame older than this is never served
    # Low-data H.264 video over WebRTC (video.py). None: no video branch, and
    # the pipeline is exactly the JPEG-only one.
    video: Optional["VideoConfig"] = None
    # v4l2: the camera at `device`. realsense: an Intel RealSense's colour
    # camera through librealsense (realsense.py), the one whose serial number
    # is `serial`, or the only one connected when that is None.
    source: str = "v4l2"
    serial: Optional[str] = None
    # A RealSense's depth stream as width, height, frames per second, read in
    # the same librealsense session as the colour (realsense.py) and served
    # raw at <prefix>/depth.z16. None: depth is not read.
    depth: Optional[Tuple[int, int, int]] = None


@dataclass(frozen=True)
class VideoConfig:
    width: int                       # pixels; the height keeps the crop's (or capture's) shape, rounded to even
    height: int
    fps: int                         # frames a second sent; the capture's extra frames are dropped before encoding
    kbps: int                        # x264 bit rate cap
    # The WebRTC sender's (cell-video, video/): it reads the same settings file.
    address: str                     # the one address WebRTC listens on and offers (the host's Tailscale address)
    port: int                        # the one UDP port every viewer's WebRTC traffic uses on that address


def _number(env: Mapping[str, str], key: str, default: float, low: float, high: float) -> float:
    raw = env.get(PREFIX + key, "").strip()
    if not raw:
        return default
    try:
        value = float(raw)
    except ValueError:
        raise ConfigError(f"{PREFIX}{key} must be a number, got {raw!r}") from None
    if not low <= value <= high:
        raise ConfigError(f"{PREFIX}{key} must be between {low:g} and {high:g}, got {raw!r}")
    return value


def load(env: Mapping[str, str]) -> Config:
    """Build the settings for one camera, or raise ConfigError naming the first bad one."""

    def text(key: str, default: Optional[str] = None) -> str:
        raw = env.get(PREFIX + key, "").strip()
        if raw:
            return raw
        if default is None:
            raise ConfigError(f"{PREFIX}{key} is required")
        return default

    source = text("SOURCE", "v4l2")
    if source not in SOURCES:
        raise ConfigError(f"{PREFIX}SOURCE must be one of {', '.join(SOURCES)}, got {source!r}")
    serial = env.get(PREFIX + "REALSENSE_SERIAL", "").strip() or None
    if source == "realsense":
        # librealsense finds the camera on the USB bus by itself; a V4L2 node
        # would be a second, conflicting way in to the same camera.
        if env.get(PREFIX + "DEVICE", "").strip():
            raise ConfigError(f"{PREFIX}DEVICE names a V4L2 camera; a RealSense is chosen by {PREFIX}REALSENSE_SERIAL, "
                              "or is the only one connected")
        if serial is not None and not _SERIAL.match(serial):
            raise ConfigError(f"{PREFIX}REALSENSE_SERIAL must be the camera's serial number (letters and digits), got {serial!r}")
        device = ""
    else:
        if serial is not None:
            raise ConfigError(f"{PREFIX}REALSENSE_SERIAL is only for {PREFIX}SOURCE=realsense")
        device = text("DEVICE")
        if not _DEVICE.match(device) or ".." in device:
            raise ConfigError(f"{PREFIX}DEVICE must be a device path under /dev, got {device!r}")

    mode = _MODE.match(text("CAPTURE"))
    if not mode:
        raise ConfigError(f"{PREFIX}CAPTURE must be WIDTHxHEIGHT@FPS, for example 1920x1200@30, got {text('CAPTURE')!r}")
    width, height, fps = (int(part) for part in mode.groups())
    if not (16 <= width <= 8192 and 16 <= height <= 8192 and 1 <= fps <= 240):
        raise ConfigError(f"{PREFIX}CAPTURE is outside 16..8192 pixels or 1..240 fps: {text('CAPTURE')!r}")

    # A lens whose image circle is smaller than the sensor darkens the corners
    # and tints the edges at the sensor's full-field modes (a second camera host's B0498,
    # 2026-10-13): keep the clean centre instead.
    crop = None
    raw_crop = env.get(PREFIX + "CROP", "").strip()
    if raw_crop:
        part = _CROP.match(raw_crop)
        if not part:
            raise ConfigError(f"{PREFIX}CROP must be WIDTHxHEIGHT+LEFT+TOP, got {raw_crop!r}")
        crop = tuple(int(value) for value in part.groups())
        crop_width, crop_height, left, top = crop
        if not (16 <= crop_width and 16 <= crop_height and left + crop_width <= width and top + crop_height <= height):
            raise ConfigError(f"{PREFIX}CROP must be 16 pixels or more and lie inside the {width}x{height} capture, got {raw_crop!r}")
    kept = (crop[0], crop[1]) if crop else (width, height)

    output = kept
    raw_output = env.get(PREFIX + "OUTPUT", "").strip()
    if raw_output:
        size = _SIZE.match(raw_output)
        if not size:
            raise ConfigError(f"{PREFIX}OUTPUT must be WIDTHxHEIGHT, got {raw_output!r}")
        output = (int(size.group(1)), int(size.group(2)))
        if not (16 <= output[0] <= kept[0] and 16 <= output[1] <= kept[1]):
            raise ConfigError(f"{PREFIX}OUTPUT must be 16 pixels or more and no larger than the capture or its crop, got {raw_output!r}")

    pixel_format = text("PIXEL_FORMAT", "YUY2")
    if not _FORMAT.match(pixel_format):
        raise ConfigError(f"{PREFIX}PIXEL_FORMAT must be a GStreamer raw format such as YUY2, got {pixel_format!r}")

    if pixel_format == "MJPG" and output != (width, height):
        raise ConfigError("MJPG passthrough requires OUTPUT to equal CAPTURE size")
    depth = None
    raw_depth = env.get(PREFIX + "REALSENSE_DEPTH", "").strip()
    if raw_depth:
        if source != "realsense":
            raise ConfigError(f"{PREFIX}REALSENSE_DEPTH is only for {PREFIX}SOURCE=realsense")
        part = _MODE.match(raw_depth)
        if not part:
            raise ConfigError(f"{PREFIX}REALSENSE_DEPTH must be WIDTHxHEIGHT@FPS, for example 640x480@15, got {raw_depth!r}")
        depth = tuple(int(n) for n in part.groups())
        if not (16 <= depth[0] <= 8192 and 16 <= depth[1] <= 8192 and 1 <= depth[2] <= 240):
            raise ConfigError(f"{PREFIX}REALSENSE_DEPTH is outside 16..8192 pixels or 1..240 fps: {raw_depth!r}")
    if source == "realsense" and pixel_format != "YUY2":
        # The colour camera's native format: librealsense hands its frames over
        # as they came off the USB, and the encoder converts them, not the CPU.
        raise ConfigError(f"{PREFIX}PIXEL_FORMAT must be YUY2 for a RealSense, got {pixel_format!r}")

    encoder = text("ENCODER", "auto")
    if encoder not in ENCODERS:
        raise ConfigError(f"{PREFIX}ENCODER must be one of {', '.join(ENCODERS)}, got {encoder!r}")

    host, _, port = text("LISTEN", "127.0.0.1:8088").rpartition(":")
    host = host.strip("[]")
    try:
        loopback = host == "localhost" or ipaddress.ip_address(host).is_loopback
        port_number = int(port)
    except ValueError:
        loopback, port_number = False, 0
    if not loopback or not 1024 <= port_number <= 65535:
        raise ConfigError(f"{PREFIX}LISTEN must be a loopback address and a port 1024..65535, got {text('LISTEN', '127.0.0.1:8088')!r}")

    prefix = text("PATH_PREFIX", "/camera").rstrip("/")
    if not _PATH.match(prefix):
        raise ConfigError(f"{PREFIX}PATH_PREFIX must be a path such as /camera, got {prefix!r}")

    default_name = device.rsplit("/", 1)[-1] if source == "v4l2" else " ".join(("RealSense", serial or "")).strip()
    name = text("NAME", default_name)
    title = text("TITLE", "Cell camera")
    for key, label in (("NAME", name), ("TITLE", title)):
        if not _LABEL.match(label):
            raise ConfigError(f"{PREFIX}{key} must be 1..80 printable characters without < > \" &, got {label!r}")

    video = _video(env, kept, fps)
    if video is not None and pixel_format == "MJPG":
        # An MJPG camera's frames are served as the camera made them (the
        # OV9281 wrist camera); video would decode every frame on the CPU first.
        raise ConfigError(f"{PREFIX}VIDEO_ADDRESS needs a raw pixel format; this camera is MJPG")

    return Config(
        device=device, name=name, title=title, capture=(width, height, fps), pixel_format=pixel_format,
        output=output, crop=crop, jpeg_quality=int(_number(env, "JPEG_QUALITY", 88, 1, 100)), encoder=encoder,
        listen=(host, port_number), path_prefix=prefix,
        idle_stop_seconds=_number(env, "IDLE_STOP_SECONDS", 8, 1, 3600),
        max_frame_age_seconds=_number(env, "MAX_FRAME_AGE_SECONDS", 0.5, 0.05, 10),
        video=video, source=source, serial=serial, depth=depth,
    )


def _video(env: Mapping[str, str], kept: Tuple[int, int], capture_fps: int) -> Optional[VideoConfig]:
    """The video settings, or None when CELL_CAMERA_VIDEO_ADDRESS is unset (video off).

    Measured on an Orin Nano Super relay (no hardware encoder), 2026-10-19,
    x264 superfast zerolatency at 960x720 15 fps capped at 800 kbit/s: 0.15 of
    a core and 75 kbit/s for a simple moving scene, 0.33 of a core at the cap
    for pure noise. Those are the defaults.
    """
    address = env.get(PREFIX + "VIDEO_ADDRESS", "").strip()
    if not address:
        return None
    try:
        ip = ipaddress.ip_address(address)
    except ValueError:
        raise ConfigError(f"{PREFIX}VIDEO_ADDRESS must be an IP address (the host's Tailscale address), got {address!r}") from None
    if ip.is_loopback or ip.is_unspecified or ip.is_multicast:
        raise ConfigError(f"{PREFIX}VIDEO_ADDRESS must be one interface's address, not {address!r}")
    width = int(_number(env, "VIDEO_WIDTH", min(960, kept[0]), 160, kept[0]))
    width -= width % 2
    height = max(2, round(width * kept[1] / kept[0] / 2) * 2)
    fps = int(_number(env, "VIDEO_FPS", min(15, capture_fps), 1, capture_fps))
    kbps = int(_number(env, "VIDEO_KBPS", 800, 50, 20000))
    port = int(_number(env, "VIDEO_PORT", 40400, 1024, 65535))
    return VideoConfig(width=width, height=height, fps=fps, kbps=kbps, address=address, port=port)


def read_env_file(text: str) -> dict:
    """Parse a settings file the way systemd's EnvironmentFile= reads the lines this service uses.

    One KEY=value per line; the value runs to the end of the line, spaces included,
    and is not quoted. Blank lines and lines starting with # or ; are ignored.
    """
    values = {}
    for number, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line or line[0] in "#;":
            continue
        key, separator, value = line.partition("=")
        key, value = key.strip(), value.strip()
        if not separator or not re.match(r"^[A-Z][A-Z0-9_]*$", key):
            raise ConfigError(f"line {number} is not KEY=value: {raw!r}")
        if value[:1] in "\"'" or "\\" in value:
            raise ConfigError(f"line {number}: write {key}'s value without quotes or backslashes")
        values[key] = value
    return values

