"""Capture from a V4L2 camera or a RealSense with GStreamer and publish JPEG frames to a FrameHub.

The pipeline is built from Config alone. GStreamer is imported only when a
capture starts, so everything else in this package runs without it.
"""
from __future__ import annotations

import time
from typing import Callable, Optional

from . import realsense
from .config import Config
from .hub import FrameHub

# Unverified bounds, from the first service on the first camera host: a sample is waited for
# 200 ms at a time, and a camera that produces nothing for 3 s has stopped.
PULL_TIMEOUT_SECONDS = 0.2
STALLED_SECONDS = 3.0


def pipeline(config: Config, nvidia: bool, source: Optional[str] = None) -> str:
    """The GStreamer launch line for this camera, ending in an appsink named `frames`.

    With video configured the camera's frames split after the queue: one branch
    is the JPEG stream below, the other the video encoder (video.video_branch).
    Without it the line is the JPEG-only one. A RealSense's frames enter
    through an appsrc in the camera's place (realsense.element); `source`
    replaces the camera element (tests use videotestsrc).
    """
    width, height, fps = config.capture
    out_width, out_height = config.output
    if config.pixel_format == "MJPG":
        return (f"{source or 'v4l2src device=' + config.device} ! image/jpeg,width={width},height={height},framerate={fps}/1 ! "
                "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
                "appsink name=frames sync=false max-buffers=1 drop=true")
    camera = source or (realsense.element(config) if config.source == "realsense" else "v4l2src device=" + config.device)
    head = (f"{camera} ! "
            f"video/x-raw,format={config.pixel_format},width={width},height={height},framerate={fps}/1 ! "
            "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! ")
    if config.video is not None:
        head += ("tee name=split split. ! "
                 "queue name=branch_jpeg max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! ")
    kept = (config.crop[0], config.crop[1]) if config.crop else (width, height)
    if nvidia:
        # nvvidconv converts, crops and scales on the Jetson's VIC, so a crop or
        # a smaller output costs no CPU. Its crop is the source rectangle.
        crop = ""
        if config.crop:
            crop_width, crop_height, left, top = config.crop
            crop = f" left={left} right={left + crop_width} top={top} bottom={top + crop_height}"
        body = (f"nvvidconv compute-hw=2{crop} ! video/x-raw(memory:NVMM),format=I420,width={out_width},height={out_height} ! "
                f"nvjpegenc quality={config.jpeg_quality} ! ")
    else:
        crop = ""
        if config.crop:
            crop_width, crop_height, left, top = config.crop
            crop = (f"videocrop left={left} right={width - left - crop_width} "
                    f"top={top} bottom={height - top - crop_height} ! ")
        scale = "" if (out_width, out_height) == kept else (
            f"videoscale ! video/x-raw,width={out_width},height={out_height} ! ")
        body = f"{crop}videoconvert ! {scale}jpegenc quality={config.jpeg_quality} ! "
    line = head + body + "appsink name=frames sync=false max-buffers=1 drop=true"
    if config.video is not None:
        from .video import video_branch
        line += " " + video_branch(config, nvidia)
    return line


# Where a frame's time in the pipeline goes, in order. Each stage ends when the
# buffer leaves that stage's element (stage_elements); `capture` starts at the buffer's
# timestamp (when the driver took the frame) and `sink` ends when this service
# pulls the JPEG. Their sum is the frame's source age.
STAGES = ("capture", "queue", "convert", "encode")


def stage_elements(config: Config, nvidia: bool) -> dict:
    """{stage: the kind of element (its GStreamer factory) whose output ends that stage} for pipeline().

    `convert` is whichever element hands the frame to the encoder. Kinds, not
    names: GStreamer names nvvidconv's element nvvconv0, and may rename others.
    """
    if config.pixel_format == "MJPG":
        return {"capture": "v4l2src", "queue": "queue"}
    camera = "appsrc" if config.source == "realsense" else "v4l2src"
    if nvidia:
        return {"capture": camera, "queue": "queue", "convert": "nvvidconv", "encode": "nvjpegenc"}
    scaled = config.output != ((config.crop[0], config.crop[1]) if config.crop else config.capture[:2])
    return {"capture": camera, "queue": "queue", "convert": "videoscale" if scaled else "videoconvert",
            "encode": "jpegenc"}


# A leaky queue drops buffers, so their marks are never collected by a pull.
MAX_MARKED_BUFFERS = 64


def stage_durations(marks: dict, pulled: float) -> tuple:
    """((stage, seconds), ...) for one buffer from its marks {stage: running time it left}, pulled at `pulled`.

    Empty unless every stage was marked in order: a partial breakdown would
    put one stage's time on another.
    """
    if any(stage not in marks for stage in STAGES) or "pts" not in marks:
        return ()
    edges = [marks["pts"]] + [marks[stage] for stage in STAGES] + [pulled]
    if any(later < earlier for earlier, later in zip(edges, edges[1:])):
        return ()
    return tuple(zip(STAGES + ("sink",), (later - earlier for earlier, later in zip(edges, edges[1:]))))


def use_nvidia(config: Config, has_element: Callable[[str], bool]) -> bool:
    """Whether to encode on the Jetson's hardware; `auto` takes it when the host has it."""
    available = has_element("nvjpegenc") and has_element("nvvidconv")
    if config.encoder == "nvidia" and not available:
        raise RuntimeError("CELL_CAMERA_ENCODER=nvidia, but this host has no nvjpegenc/nvvidconv")
    return config.encoder != "software" and available


def encoding(nvidia: bool) -> str:
    return "NVIDIA JPEG / VIC conversion" if nvidia else "software JPEG"


def stream_failure(message, Gst) -> str:
    """What ended the stream, in GStreamer's own words: the element and its error, not just "ended"."""
    if message.type == Gst.MessageType.ERROR:
        error, _debug = message.parse_error()
        source = message.src.get_name() if message.src is not None else "pipeline"
        return f"Camera stream error in {source}: {error.message}"
    return "Camera stream ended"


def jpeg_frame(data: bytes, camera_native: bool = False) -> bytes:
    """Drop USB buffer alignment padding, preserving the camera's JPEG bytes.

    OV9281 MJPEG buffers have 0–7 zero bytes after EOI. Only camera-native
    frames permit this padding; truncation and nonzero trailers still fail.
    """
    if camera_native:
        data = data.rstrip(b"\x00")
    if not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9"):
        raise RuntimeError("Invalid camera frame")
    return data


def next_jpeg(data: bytes, camera_native: bool, last_valid: float, now: float):
    """Skip an isolated damaged USB frame, but fail a persistently broken stream."""
    try:
        return jpeg_frame(data, camera_native)
    except RuntimeError:
        if now - last_valid >= STALLED_SECONDS:
            raise RuntimeError("Camera stopped producing valid frames")
        return None


def producer(config: Config, facts: dict, video=None, source: Optional[str] = None,
             depth=None) -> Callable[[FrameHub], None]:
    """A FrameHub producer for this camera. Each call captures until the hub stops wanting frames.

    `facts["encoding"]` is set to what the host actually encodes with, for status.
    `video` (video.VideoPeers) gets the running pipeline to add viewers to.
    `depth` (hub.DepthFrames) gets a RealSense's raw depth frames, when configured.
    """

    def produce(hub: FrameHub) -> None:
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
        Gst.init(None)
        nvidia = config.pixel_format != "MJPG" and use_nvidia(config, lambda name: Gst.ElementFactory.find(name) is not None)
        facts["encoding"] = "camera-jpeg" if config.pixel_format == "MJPG" else encoding(nvidia)
        line = Gst.parse_launch(pipeline(config, nvidia, source))
        # Stage timings: when each buffer left each named element, by its
        # timestamp, which every element here carries through unchanged.
        marks: dict = {}

        def running() -> float:
            return (line.get_clock().get_time() - line.get_base_time()) / Gst.SECOND

        def mark(stage):
            def probe(_pad, info):
                pts = info.get_buffer().pts
                if line.get_clock() is not None and pts != Gst.CLOCK_TIME_NONE:
                    marks.setdefault(pts, {"pts": pts / Gst.SECOND})[stage] = running()
                return Gst.PadProbeReturn.OK
            return probe

        kinds = {kind: stage for stage, kind in stage_elements(config, nvidia).items()}
        # Each probe is removed when capture stops: a probe's callback holds
        # this pipeline, so a probe left on its pad kept every stopped pipeline
        # alive with its VIC, encoder and poll handles (about 6 file handles a
        # start and stop, measured on a second camera host 2026-10-15; a third
        # host's service reached its 1024 limit after two days of viewers
        # coming and going).
        probes = []
        elements = line.iterate_elements()
        while True:
            result, element = elements.next()
            if result != Gst.IteratorResult.OK:
                break
            factory = element.get_factory()
            # The JPEG stream's own elements only: the video branch and the
            # split's queue carry the same buffers and would overwrite its marks.
            if element.get_name().startswith(("video_", "branch_")):
                continue
            if factory is not None and factory.get_name() in kinds:
                pad = element.get_static_pad("src")
                probes.append((pad, pad.add_probe(Gst.PadProbeType.BUFFER, mark(kinds[factory.get_name()]))))
        # A RealSense feeds the appsrc from a thread of its own (realsense.py);
        # a test's `source` stands in for the camera and needs no feeder.
        feeder = realsense.Feeder(config, facts, depth) if config.source == "realsense" and source is None else None
        try:
            sink = line.get_by_name("frames")
            bus = line.get_bus()
            if line.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("Camera could not start")
            if feeder is not None:
                feeder.start(line.get_by_name("camera"), running, Gst)
            if video is not None:
                video.attach(line, Gst)
            last_frame = time.monotonic()
            while hub.wanted():
                if feeder is not None:
                    feeder.check()
                message = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
                if message is not None:
                    raise RuntimeError(stream_failure(message, Gst))
                sample = sink.emit("try-pull-sample", int(PULL_TIMEOUT_SECONDS * Gst.SECOND))
                if sample is None:
                    if time.monotonic() - last_frame > STALLED_SECONDS:
                        raise RuntimeError("Camera stopped producing frames")
                    continue
                buffer = sample.get_buffer()
                data = buffer.extract_dup(0, buffer.get_size())
                now = time.monotonic()
                data = next_jpeg(data, config.pixel_format == "MJPG", last_frame, now)
                if data is None:
                    facts["dropped_invalid_frames"] = facts.get("dropped_invalid_frames", 0) + 1
                    marks.pop(buffer.pts, None)
                    if len(marks) > MAX_MARKED_BUFFERS:
                        marks.clear()
                    continue
                last_frame = now
                pulled = running()
                age = max(0.0, pulled - buffer.pts / Gst.SECOND)
                frame_marks = marks.pop(buffer.pts, {})
                if config.pixel_format == "MJPG" and "queue" in frame_marks:
                    frame_marks["convert"] = frame_marks["encode"] = frame_marks["queue"]
                stages = stage_durations(frame_marks, pulled)
                if len(marks) > MAX_MARKED_BUFFERS:
                    for pts in sorted(marks)[:-MAX_MARKED_BUFFERS // 2]:
                        marks.pop(pts, None)
                hub.publish(data, age, stages)
        finally:
            if feeder is not None:
                feeder.stop()
            if video is not None:
                video.detach()
            for pad, probe in probes:
                pad.remove_probe(probe)
            probes.clear()
            marks.clear()
            line.set_state(Gst.State.NULL)

    return produce
