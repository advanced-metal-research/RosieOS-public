"""Run one cell camera: `python3 -m cell_camera`, settings from CELL_CAMERA_* in the environment."""
from __future__ import annotations

import os
import sys

from .capture import producer
from .config import ConfigError, load
from .hub import DepthFrames, FrameHub
from .server import Server


def main() -> int:
    try:
        config = load(os.environ)
    except ConfigError as error:
        print(f"cell-camera: {error}", file=sys.stderr)
        return 2
    facts: dict = {}
    # The hub and the video service refer to each other (viewers keep capture
    # running; capture hands viewers the pipeline), so they meet through `parts`.
    parts: dict = {}
    video = None
    if config.video is not None:
        from .video import VideoStream
        video = VideoStream(config, lambda: parts["hub"].touch(), lambda kind, n, r: parts["server"].count(kind, n, r))
    # A depth request keeps capture running the way a frame request does.
    depth = DepthFrames(lambda: parts["hub"].touch(), config.max_frame_age_seconds) if config.depth is not None else None
    hub = parts["hub"] = FrameHub(producer(config, facts, video, depth=depth), config.idle_stop_seconds,
                                  config.max_frame_age_seconds)
    server = parts["server"] = Server(config, hub, facts, video, depth)
    width, height, fps = config.capture
    camera = config.device if config.source == "v4l2" else f"RealSense {config.serial or '(the only one connected)'}"
    print(f"cell-camera: {config.name} on {camera}, {width}x{height}@{fps} {config.pixel_format} -> "
          f"{config.output[0]}x{config.output[1]} JPEG q{config.jpeg_quality}, "
          f"http://{config.listen[0]}:{config.listen[1]}{config.path_prefix}/ (capture starts on the first frame request)",
          flush=True)
    if config.depth is not None:
        print("cell-camera: depth {}x{}@{} Z16 at {}/depth.z16".format(*config.depth, config.path_prefix), flush=True)
    if config.video is not None:
        v = config.video
        print(f"cell-camera: video {v.width}x{v.height}@{v.fps} H.264 {v.kbps} kbit/s at {config.path_prefix}/video.h264 "
              f"for the WebRTC sender (on {v.address} UDP {v.port})", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
