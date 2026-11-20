"""Colour frames from an Intel RealSense, through librealsense, into the camera's GStreamer pipeline; raw depth beside them.

A RealSense is read with librealsense's RSUSB backend (host/build-pyrealsense2.sh)
rather than as a V4L2 node. Its V4L2 nodes deliver each frame's metadata on a
second node that librealsense pairs back up with the frame; when colour frames
arrive cut short, each costs a metadata buffer, and once they are gone the
colour stream stops for good. RSUSB reads the frame and its metadata in one
USB transfer and has nothing to pair. A thread here pushes each colour frame,
as YUY2 straight off the USB, into an appsrc that takes v4l2src's place: crop,
scale, the JPEG encoder, the video branch and the stage timings are the V4L2
camera's own (capture.pipeline).

librealsense lets one process hold the camera, so depth, when configured
(CELL_CAMERA_REALSENSE_DEPTH), is read in the same session as the colour and
published raw to a hub.DepthFrames: Z16, unaligned, in the depth camera's own
frame. Status carries the depth scale, both cameras' intrinsics and the depth
to colour extrinsics, so a client can deproject or align it itself; aligning
here would cost the relay a reprojection of every frame nobody may read.

pyrealsense2 is imported only when a capture starts, so the settings, the
pipeline text and the tests run without it.
"""
from __future__ import annotations

import sys
import threading
import time
from typing import Callable, Optional

from .config import Config

# Unverified bound: librealsense is waited for a second at a time, so stop()
# is never stuck behind a camera that sends nothing. A camera that stays
# silent is reported by the capture loop's own stall check (capture.py).
WAIT_MS = 1000
# Unverified bound: the feeding thread is waited for this long at stop.
STOP_SECONDS = 3.0


def element(config: Config) -> str:
    """The appsrc that replaces v4l2src: live, timed by the frames' own timestamps, and never queueing.

    It drops the oldest frame rather than block librealsense's thread, like the
    leaky queue behind a V4L2 camera.
    """
    width, height, fps = config.capture
    return ("appsrc name=camera is-live=true format=time block=false max-buffers=1 leaky-type=downstream "
            f"caps=video/x-raw,format=YUY2,width={width},height={height},framerate={fps}/1")


def find_device(rs, serial: Optional[str]):
    """The RealSense `serial` names, else the only one connected. Raises RuntimeError saying what is connected."""
    devices = list(rs.context().query_devices())
    serials = [device.get_info(rs.camera_info.serial_number) for device in devices]
    if serial is not None:
        for device, found in zip(devices, serials):
            if found == serial:
                return device
        raise RuntimeError(f"No RealSense with serial number {serial} is connected (connected: {', '.join(serials) or 'none'})")
    if not devices:
        raise RuntimeError("No RealSense is connected")
    if len(devices) > 1:
        raise RuntimeError(f"{len(devices)} RealSense cameras are connected ({', '.join(serials)}): "
                           "name one with CELL_CAMERA_REALSENSE_SERIAL")
    return devices[0]


def describe(rs, device) -> dict:
    """What status reports about the camera: which one it is and how it is connected."""
    info = rs.camera_info
    return {"name": device.get_info(info.name), "serial": device.get_info(info.serial_number),
            "firmware": device.get_info(info.firmware_version), "usb": device.get_info(info.usb_type_descriptor),
            "librealsense": rs.__version__}


def start_failure(config: Config, usb: str, error: BaseException) -> str:
    """Why the streams did not start, in librealsense's words, with the USB link when it is the likely cause."""
    width, height, fps = config.capture
    asked = f"colour {width}x{height}@{fps} YUY2"
    if config.depth is not None:
        asked += " and depth {}x{}@{} Z16".format(*config.depth)
    text = f"RealSense could not stream {asked}: {error}"
    if usb.startswith("2"):
        # A D435i on a USB 2 link offers fewer and slower modes than on USB 3
        # (Intel's D400 datasheet); which ones is read from the camera.
        smaller = "CELL_CAMERA_CAPTURE" + (" or CELL_CAMERA_REALSENSE_DEPTH" if config.depth is not None else "")
        text += (f" (it is connected over USB {usb}, which offers fewer modes than USB 3: "
                 f"use a USB 3 port or a smaller {smaller})")
    return text


def intrinsics(stream_profile) -> dict:
    """A video stream's pinhole model and distortion, as librealsense reports it (pixels)."""
    i = stream_profile.as_video_stream_profile().get_intrinsics()
    return {"width": i.width, "height": i.height, "fx": i.fx, "fy": i.fy, "ppx": i.ppx, "ppy": i.ppy,
            "model": str(i.model).rsplit(".", 1)[-1], "coeffs": list(i.coeffs)}


def depth_facts(rs, profile) -> dict:
    """What a client needs to use the raw depth: units, both cameras' models and where the colour camera is."""
    depth_stream, colour_stream = profile.get_stream(rs.stream.depth), profile.get_stream(rs.stream.color)
    to_colour = depth_stream.get_extrinsics_to(colour_stream)
    return {"format": "z16", "byte_order": sys.byteorder, "units": "depth units; metres = units x depth_scale_m, 0 = no depth",
            "depth_scale_m": profile.get_device().first_depth_sensor().get_depth_scale(),
            "intrinsics": intrinsics(depth_stream), "color_intrinsics": intrinsics(colour_stream),
            # librealsense's convention: rotation is 3x3 column-major, translation in metres.
            "extrinsics_to_color": {"rotation": list(to_colour.rotation), "translation_m": list(to_colour.translation)}}


def presentation(running_now: float, wall_now: float, arrival_ms: Optional[float], previous: Optional[float]) -> float:
    """A frame's presentation time in seconds of pipeline running time.

    It is when librealsense had the whole frame (its time_of_arrival, wall
    clock milliseconds), carried onto the pipeline's clock, so the `capture`
    stage is the frame's wait before this service took it, as a V4L2 driver's
    timestamp makes it for v4l2src. Never before zero, and always after the
    previous frame's, which a step of the wall clock could otherwise reverse.
    """
    pts = running_now if arrival_ms is None else running_now - max(0.0, wall_now - arrival_ms / 1000)
    pts = max(0.0, pts)
    if previous is not None and pts <= previous:
        pts = previous + 1e-6
    return pts


class Feeder:
    """Starts the RealSense's colour stream and pushes its frames into the pipeline's appsrc from a thread of its own."""

    def __init__(self, config: Config, facts: dict, depth=None) -> None:
        if (config.depth is None) != (depth is None):
            raise ValueError("a RealSense's depth is read only with both CELL_CAMERA_REALSENSE_DEPTH and a DepthFrames to publish to")
        self.config, self.facts, self.depth = config, facts, depth
        self._scale = 0.0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._pipe = None
        self.error: Optional[str] = None

    def start(self, appsrc, running: Callable[[], float], Gst) -> None:
        """Open the camera and start feeding `appsrc`. Raises RuntimeError naming the camera's refusal."""
        import pyrealsense2.pyrealsense2 as rs  # the compiled module itself, __version__ included
        device = find_device(rs, self.config.serial)
        info = self.facts["realsense"] = describe(rs, device)
        width, height, fps = self.config.capture
        request = rs.config()
        request.enable_device(info["serial"])
        request.enable_stream(rs.stream.color, width, height, rs.format.yuyv, fps)
        if self.config.depth is not None:
            request.enable_stream(rs.stream.depth, *self.config.depth[:2], rs.format.z16, self.config.depth[2])
        pipe = rs.pipeline()
        try:
            profile = pipe.start(request)
        except RuntimeError as error:
            raise RuntimeError(start_failure(self.config, info["usb"], error)) from None
        self._pipe = pipe
        if self.depth is not None:
            info["depth"] = depth_facts(rs, profile)
            self._scale = info["depth"]["depth_scale_m"]
        self._thread = threading.Thread(target=self._feed, args=(rs, pipe, appsrc, running, Gst),
                                         name="realsense", daemon=True)
        self._thread.start()

    def _feed(self, rs, pipe, appsrc, running, Gst) -> None:
        arrival = rs.frame_metadata_value.time_of_arrival
        last_number = previous = last_depth = None
        try:
            while not self._stop.is_set():
                try:
                    frames = pipe.wait_for_frames(WAIT_MS)
                except RuntimeError:
                    continue  # no frame within WAIT_MS; the capture loop reports a camera that stays silent
                if self.depth is not None:
                    last_depth = self._publish_depth(frames.get_depth_frame(), arrival, last_depth)
                frame = frames.get_color_frame()
                if not frame:
                    continue
                # librealsense can hand the last frame over again while it waits
                # for the next; the picture would not change, so it is skipped.
                number = frame.get_frame_number()
                if last_number is not None and number <= last_number:
                    continue
                last_number = number
                arrival_ms = frame.get_frame_metadata(arrival) if frame.supports_frame_metadata(arrival) else None
                previous = presentation(running(), time.time(), arrival_ms, previous)
                buffer = Gst.Buffer.new_wrapped(bytes(frame.get_data()))
                buffer.pts = int(previous * Gst.SECOND)
                appsrc.emit("push-buffer", buffer)
        except Exception as error:  # noqa: BLE001  reported by check() on the capture loop's thread
            self.error = f"RealSense stream error: {error}"

    def _publish_depth(self, frame, arrival, last_number: Optional[int]) -> Optional[int]:
        """Publish a depth frame once, as librealsense handed it over. Returns the newest frame number published."""
        if not frame:
            return last_number
        # As for colour, a set can carry the last depth frame again (a slower
        # depth rate repeats it in every colour set until the next one).
        number = frame.get_frame_number()
        if last_number is not None and number <= last_number:
            return last_number
        width, height = self.config.depth[:2]
        arrival_ms = frame.get_frame_metadata(arrival) if frame.supports_frame_metadata(arrival) else None
        age = 0.0 if arrival_ms is None else max(0.0, time.time() - arrival_ms / 1000)
        self.depth.publish(bytes(frame.get_data()), width, height, self._scale, age, number, frame.get_timestamp())
        return number

    def check(self) -> None:
        """Raise the feeding thread's failure on the caller's thread, so the capture loop ends with its reason."""
        if self.error is not None:
            raise RuntimeError(self.error)

    def stop(self) -> None:
        """Stop feeding and release the camera. Safe when start() failed or never ran."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(STOP_SECONDS)
            self._thread = None
        if self._pipe is not None:
            self._pipe.stop()
            self._pipe = None
        if self.depth is not None:
            self.depth.clear()
