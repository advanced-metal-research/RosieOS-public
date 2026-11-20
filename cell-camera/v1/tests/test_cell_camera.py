"""The cell camera without a camera: settings, the pipeline it would run, the frame hub and the HTTP contract.

Run: python3 -m unittest discover -s tests   (from cell-camera/v1; standard library only)
"""
import json
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cell_camera import capture, config, realsense  # noqa: E402
from cell_camera.hub import DepthFrames, FrameHub  # noqa: E402
from cell_camera.server import Server, server_timing  # noqa: E402

DEVICE = "/dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP__Arducam_202500915_0001-video-index0"
BASE = {"CELL_CAMERA_DEVICE": DEVICE, "CELL_CAMERA_CAPTURE": "1920x1200@30"}
JPEG = b"\xff\xd8 frame \xff\xd9"


def settings(**overrides):
    return config.load({**BASE, **{"CELL_CAMERA_" + key: value for key, value in overrides.items()}})


class ConfigTest(unittest.TestCase):
    def test_defaults_are_the_first_service(self):
        c = settings()
        self.assertEqual((c.capture, c.output, c.pixel_format, c.jpeg_quality, c.encoder),
                         ((1920, 1200, 30), (1920, 1200), "YUY2", 88, "auto"))
        self.assertEqual((c.listen, c.path_prefix, c.idle_stop_seconds, c.max_frame_age_seconds),
                         (("127.0.0.1", 8088), "/camera", 8, 0.5))
        self.assertEqual(c.name, DEVICE.rsplit("/", 1)[-1])

    def test_what_differs_per_camera_is_a_setting(self):
        c = settings(OUTPUT="960x600", JPEG_QUALITY="75", ENCODER="software", LISTEN="[::1]:8090",
                     PATH_PREFIX="/camera2/", NAME="Arducam OV9281", TITLE="rosie_1420_v1 / Example relay", PIXEL_FORMAT="GRAY8")
        self.assertEqual((c.output, c.jpeg_quality, c.encoder, c.listen, c.path_prefix, c.name, c.title, c.pixel_format),
                         ((960, 600), 75, "software", ("::1", 8090), "/camera2", "Arducam OV9281", "rosie_1420_v1 / Example relay", "GRAY8"))

    def test_bad_settings_are_refused_by_name(self):
        for key, value in {
            "DEVICE": "video0", "CAPTURE": "1920x1200", "OUTPUT": "3840x2400", "PIXEL_FORMAT": "yuy2 !",
            "JPEG_QUALITY": "0", "ENCODER": "gpu", "LISTEN": "0.0.0.0:8088", "PATH_PREFIX": "/a b",
            "NAME": "<b>", "IDLE_STOP_SECONDS": "soon", "MAX_FRAME_AGE_SECONDS": "60",
        }.items():
            with self.subTest(key=key), self.assertRaisesRegex(config.ConfigError, "CELL_CAMERA_" + key):
                settings(**{key: value})
        for bad in ("/dev/video0 ! filesink location=/etc/x", "/dev/../etc/passwd", "/etc/passwd"):
            with self.subTest(device=bad), self.assertRaisesRegex(config.ConfigError, "CELL_CAMERA_DEVICE"):
                settings(DEVICE=bad)
        with self.assertRaisesRegex(config.ConfigError, "CELL_CAMERA_DEVICE is required"):
            config.load({"CELL_CAMERA_CAPTURE": "640x480@30"})
        with self.assertRaisesRegex(config.ConfigError, "CELL_CAMERA_LISTEN"):
            settings(LISTEN="192.168.1.10:8088")


    def test_settings_file_is_read_as_systemd_reads_it(self):
        example = (Path(__file__).resolve().parents[1] / "host" / "example.env").read_text()
        values = config.read_env_file(example)
        self.assertEqual(set(values), {"CELL_CAMERA_DEVICE", "CELL_CAMERA_CAPTURE"})
        self.assertEqual(config.load(values).capture, (1920, 1200, 30))
        values = config.read_env_file("# a camera\n\nCELL_CAMERA_TITLE=r0000 / Bench\n CELL_CAMERA_NAME = Test camera \n")
        self.assertEqual(values, {"CELL_CAMERA_TITLE": "r0000 / Bench", "CELL_CAMERA_NAME": "Test camera"})
        for bad in ("export CELL_CAMERA_DEVICE=/dev/video0", "CELL_CAMERA_TITLE=\"quoted\"", "just words"):
            with self.subTest(line=bad), self.assertRaises(config.ConfigError):
                config.read_env_file(bad)


class PipelineTest(unittest.TestCase):
    def test_isolated_damage_is_skipped_but_persistent_damage_fails(self):
        self.assertIsNone(capture.next_jpeg(JPEG[:-2], True, 10, 10.1))
        self.assertEqual(capture.next_jpeg(JPEG, True, 10, 10.2), JPEG)
        with self.assertRaisesRegex(RuntimeError, "Camera stopped producing valid frames"):
            capture.next_jpeg(JPEG[:-2], True, 10, 13)

    def test_native_usb_jpeg_alignment_padding(self):
        for count in range(8):
            self.assertEqual(capture.jpeg_frame(JPEG + b"\x00" * count, True), JPEG)
        self.assertEqual(capture.jpeg_frame(JPEG), JPEG)
        for malformed in (JPEG + b"junk", JPEG[:-2], b"no header\xff\xd9", b"\x00" * 8):
            with self.assertRaisesRegex(RuntimeError, "Invalid camera frame"):
                capture.jpeg_frame(malformed, True)
        with self.assertRaisesRegex(RuntimeError, "Invalid camera frame"):
            capture.jpeg_frame(JPEG + b"\x00")

    def test_jetson_pipeline_is_the_first_service_at_full_size(self):
        line = capture.pipeline(settings(), nvidia=True)
        self.assertEqual(line, (
            f"v4l2src device={DEVICE} ! video/x-raw,format=YUY2,width=1920,height=1200,framerate=30/1 ! "
            "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
            "nvvidconv compute-hw=2 ! video/x-raw(memory:NVMM),format=I420,width=1920,height=1200 ! "
            "nvjpegenc quality=88 ! appsink name=frames sync=false max-buffers=1 drop=true"))

    def test_output_size_scales_on_either_encoder(self):
        small = settings(OUTPUT="960x600", JPEG_QUALITY="75")
        self.assertIn("format=I420,width=960,height=600 ! nvjpegenc quality=75", capture.pipeline(small, nvidia=True))
        self.assertIn("videoconvert ! videoscale ! video/x-raw,width=960,height=600 ! jpegenc quality=75",
                      capture.pipeline(small, nvidia=False))
        self.assertNotIn("videoscale", capture.pipeline(settings(), nvidia=False))

    def test_crop_keeps_the_centre_on_either_encoder(self):
        cropped = settings(CAPTURE="1920x1080@30", CROP="1080x810+420+75")
        self.assertEqual(cropped.output, (1080, 810))
        self.assertIn("nvvidconv compute-hw=2 left=420 right=1500 top=75 bottom=885 ! "
                      "video/x-raw(memory:NVMM),format=I420,width=1080,height=810", capture.pipeline(cropped, nvidia=True))
        line = capture.pipeline(cropped, nvidia=False)
        self.assertIn("videocrop left=420 right=420 top=75 bottom=195 ! videoconvert ! jpegenc", line)
        self.assertNotIn("videoscale", line)
        smaller = settings(CAPTURE="1920x1080@30", CROP="1080x810+420+75", OUTPUT="720x540")
        self.assertIn("videocrop left=420 right=420 top=75 bottom=195 ! videoconvert ! videoscale ! "
                      "video/x-raw,width=720,height=540", capture.pipeline(smaller, nvidia=False))
        self.assertEqual(capture.stage_elements(smaller, nvidia=False)["convert"], "videoscale")
        self.assertEqual(capture.stage_elements(cropped, nvidia=False)["convert"], "videoconvert")
        self.assertNotIn("left=", capture.pipeline(settings(), nvidia=True))
        self.assertNotIn("videocrop", capture.pipeline(settings(), nvidia=False))
        for bad in ("1080x810", "1080x810+900+75", "1080x810+420+300", "8x8+0+0", "1080x810+-1+0"):
            with self.assertRaises(config.ConfigError, msg=bad):
                settings(CAPTURE="1920x1080@30", CROP=bad)
        with self.assertRaises(config.ConfigError):
            settings(CAPTURE="1920x1080@30", CROP="1080x810+420+75", OUTPUT="1456x1088")

    def test_encoder_choice(self):
        jetson, other = (lambda name: True), (lambda name: False)
        self.assertTrue(capture.use_nvidia(settings(), jetson))
        self.assertFalse(capture.use_nvidia(settings(), other))
        self.assertFalse(capture.use_nvidia(settings(ENCODER="software"), jetson))
        with self.assertRaisesRegex(RuntimeError, "no nvjpegenc"):
            capture.use_nvidia(settings(ENCODER="nvidia"), other)


def frames_every(period):
    """A producer that publishes a frame every `period` seconds until nobody wants one."""
    started = []

    def produce(hub):
        started.append(time.monotonic())
        while hub.wanted():
            hub.publish(JPEG)
            time.sleep(period)

    return produce, started


class VideoBranchTest(unittest.TestCase):
    def test_a_closed_gate_cannot_stall_the_jpeg_stream(self):
        # The default valve dropped caps too and the video sink waited for a
        # first buffer: the whole capture stalled on other-relay (2026-10-19).
        c = settings(VIDEO_ADDRESS="100.64.0.11")
        line = capture.pipeline(c, True)
        self.assertIn("valve name=video_gate drop=true drop-mode=1 ", line)
        self.assertIn("appsink name=video_units sync=false async=false ", line)
        self.assertIn("tee name=split", line)
        self.assertNotIn("tee name=split", capture.pipeline(settings(), True))


class HubTest(unittest.TestCase):
    def test_a_client_gets_only_frames_newer_than_its_cursor(self):
        produce, _ = frames_every(0.01)
        hub = FrameHub(produce, idle_stop_seconds=1, max_frame_age_seconds=0.5)
        first = hub.frame(0)
        self.assertEqual((first.data, first.sequence >= 1), (JPEG, True))
        second = hub.frame(first.sequence)
        self.assertGreater(second.sequence, first.sequence)
        self.assertTrue(hub.status()["active"])

    def test_capture_starts_on_demand_and_stops_when_nobody_asks(self):
        produce, started = frames_every(0.01)
        hub = FrameHub(produce, idle_stop_seconds=0.15, max_frame_age_seconds=0.5)
        self.assertEqual((started, hub.status()["active"]), ([], False))
        self.assertIsNotNone(hub.frame(0))
        deadline = time.monotonic() + 2
        while hub.status()["active"] and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertFalse(hub.status()["active"])
        self.assertIsNotNone(hub.frame(0))  # and starts again for the next viewer
        self.assertEqual(len(started), 2)

    def test_a_failing_camera_is_a_status_and_no_frame(self):
        def broken(hub):
            raise RuntimeError("Camera could not start")

        hub = FrameHub(broken, idle_stop_seconds=1, max_frame_age_seconds=0.5)
        self.assertIsNone(hub.frame(0, timeout=1))
        self.assertEqual(hub.status()["error"], "Camera could not start")

    def test_a_stale_frame_is_never_served(self):
        def once(hub):
            hub.publish(JPEG)
            hub.publish(JPEG, source_age=5)  # too old on arrival: dropped
            while hub.wanted():
                time.sleep(0.01)

        hub = FrameHub(once, idle_stop_seconds=1, max_frame_age_seconds=0.1)
        self.assertEqual(hub.frame(0).sequence, 1)
        time.sleep(0.15)
        self.assertIsNone(hub.frame(0, timeout=0.05))


class ServerTest(unittest.TestCase):
    def setUp(self):
        self.config = settings(LISTEN="127.0.0.1:18088", NAME="Test camera", TITLE="r0000 / Bench", OUTPUT="960x600")
        produce, _ = frames_every(0.01)
        self.hub = FrameHub(produce, idle_stop_seconds=1, max_frame_age_seconds=0.5)
        Server.allow_reuse_address = True
        self.server = Server(self.config, self.hub, {"encoding": "software JPEG"})
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.origin = f"http://127.0.0.1:{self.server.server_address[1]}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def get(self, path):
        try:
            with urllib.request.urlopen(self.origin + path, timeout=5) as reply:
                return reply.status, reply.headers, reply.read()
        except urllib.error.HTTPError as error:
            with error:
                return error.code, error.headers, error.read()

    def test_contract(self):
        status, headers, body = self.get("/camera/frame.jpg?after=0")
        self.assertEqual((status, headers["Content-Type"], headers["Cache-Control"], body), (200, "image/jpeg", "no-store", JPEG))
        sequence = int(headers["X-Frame-Sequence"])
        self.assertGreaterEqual(float(headers["X-Source-Age-Ms"]), 0)
        status, headers, _ = self.get(f"/camera/frame.jpg?after={sequence}")
        self.assertGreater(int(headers["X-Frame-Sequence"]), sequence)

        status, _, body = self.get("/camera/status")
        document = json.loads(body)
        self.assertEqual((status, document["camera"], document["title"], document["resolution"], document["capture_resolution"],
                          document["configured_fps"], document["active"], document["encoding"], document["error"]),
                         (200, "Test camera", "r0000 / Bench", [960, 600], [1920, 1200], 30, True, "software JPEG", None))
        self.assertGreater(document["sequence"], 0)

        status, headers, body = self.get("/camera/")
        self.assertEqual((status, headers["Content-Type"]), (200, "text/html; charset=utf-8"))
        self.assertIn(b"frame.jpg?after=", body)
        self.assertNotIn(b"r0000", body)  # the page takes its labels from status

    def test_status_counts_what_was_served(self):
        _, headers, body = self.get("/camera/frame.jpg?after=0")
        self.get("/camera/frame.jpg?after=0")
        served = json.loads(self.get("/camera/status")[2])["served"]
        jpeg = served["jpeg"]
        self.assertEqual(jpeg["replies"], 2)
        # Body plus status line and headers; the headers are a few hundred bytes.
        self.assertGreater(jpeg["bytes"], 2 * len(body))
        self.assertLess(jpeg["bytes"], 2 * (len(body) + 1000))
        self.assertLessEqual(served["since"], time.time())

    def test_refusals(self):
        self.assertEqual(self.get("/camera/frame.jpg?after=x")[0], 400)
        for path in ("/camera/nothing", "/elsewhere/frame.jpg", "/camerax/status", "/"):
            self.assertEqual(self.get(path)[0], 404, path)


class VideoStreamTest(unittest.TestCase):
    def test_a_reader_that_stops_reading_frees_the_stream(self):
        # A reader that keeps its connection open but takes no bytes must not
        # hold the encoder and a request slot for ever: its write times out.
        ended = threading.Event()

        class Video:
            video = config.VideoConfig(width=960, height=600, fps=15, kbps=800, address="100.64.0.14", port=40400)

            def stream(self, write):
                try:
                    while True:
                        write(b"\x00" * (64 << 10))
                finally:
                    ended.set()

        from cell_camera import server as server_module
        original = server_module.SOCKET_TIMEOUT_SECONDS
        server_module.SOCKET_TIMEOUT_SECONDS = 0.3
        self.addCleanup(setattr, server_module, "SOCKET_TIMEOUT_SECONDS", original)
        produce, _ = frames_every(0.01)
        Server.allow_reuse_address = True
        server = Server(settings(LISTEN="127.0.0.1:18089"), FrameHub(produce, idle_stop_seconds=1, max_frame_age_seconds=0.5), {}, video=Video())
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        import socket
        reader = socket.create_connection(server.server_address)
        self.addCleanup(reader.close)
        reader.sendall(b"GET /camera/video.h264 HTTP/1.1\r\nHost: camera\r\n\r\n")
        self.assertTrue(ended.wait(10), "the stream still writes 10 s after its reader stopped reading")


if __name__ == "__main__":
    unittest.main()


class StageTimingTest(unittest.TestCase):
    def test_stages_sum_to_the_frame_age_and_need_every_mark(self):
        marks = {"pts": 10.000, "capture": 10.004, "queue": 10.005, "convert": 10.012, "encode": 10.030}
        stages = dict(capture.stage_durations(marks, pulled=10.032))
        self.assertEqual(list(stages), ["capture", "queue", "convert", "encode", "sink"])
        self.assertAlmostEqual(sum(stages.values()), 0.032)
        self.assertAlmostEqual(stages["encode"], 0.018)
        # A missing or out-of-order mark gives no breakdown, never a wrong one.
        self.assertEqual(capture.stage_durations({k: v for k, v in marks.items() if k != "convert"}, 10.032), ())
        self.assertEqual(capture.stage_durations({**marks, "encode": 10.001}, 10.032), ())
        self.assertEqual(capture.stage_durations({}, 10.032), ())

    def test_stage_elements_follow_the_pipeline(self):
        for nvidia, config in ((True, settings()), (False, settings()), (False, settings(CELL_CAMERA_OUTPUT="960x600")),
                               (True, realsense_settings()), (False, realsense_settings(OUTPUT="320x240"))):
            line = capture.pipeline(config, nvidia)
            kinds = capture.stage_elements(config, nvidia)
            self.assertEqual(tuple(kinds), capture.STAGES)
            for kind in kinds.values():
                self.assertEqual(sum(part.split()[0] == kind for part in line.split(" ! ")), 1, kind)

    def test_server_timing_header(self):
        self.assertEqual(server_timing((("capture", 0.0041), ("held", 0.0005), ("wait", 0.02))),
                         "capture;dur=4.10, held;dur=0.50, wait;dur=20.00")


class ProfilerTest(unittest.TestCase):
    """tools/camera_latency.py: the arithmetic that turns clocks into stages."""

    def setUp(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
        import camera_latency
        self.tool = camera_latency

    def test_server_timing_and_derived_stages(self):
        stages = self.tool.parse_server_timing("capture;dur=22.2, encode;dur=2.3;desc=x, wait;dur=20, relay;dur=21.5, junk")
        self.assertEqual(stages, {"capture": 22.2, "encode": 2.3, "wait": 20.0, "relay": 21.5})
        frame = {"server": stages, "total_ms": 40.0, "download_ms": 9.0, "source_age_ms": 46.0}
        self.tool.derive(frame, ping_ms=8.0)
        # relay overhead 1.5; network = 40 - 20 wait - 1.5; age at receive = 46 + 9 + 8/2.
        self.assertEqual((frame["relay_overhead_ms"], frame["network_ms"], frame["age_at_receive_ms"]), (1.5, 18.5, 59.0))
        bare = {"server": {}, "total_ms": 40.0, "download_ms": 9.0, "source_age_ms": None}
        self.tool.derive(bare, ping_ms=8.0)
        self.assertNotIn("network_ms", bare)
        self.assertNotIn("age_at_receive_ms", bare)

    def test_percentiles(self):
        row = self.tool.summarize([float(v) for v in range(1, 101)])
        self.assertEqual((row["n"], row["p50"], row["p99"], row["max"]), (100, 51.0, 99.0, 100.0))



class JpegCameraTest(unittest.TestCase):
    def test_camera_jpeg_passes_through_without_decoding_or_encoding(self):
        c = settings(CAPTURE="1280x800@100", PIXEL_FORMAT="MJPG")
        line = capture.pipeline(c, False)
        self.assertIn("image/jpeg,width=1280,height=800,framerate=100/1", line)
        self.assertNotIn("jpegenc", line)
        self.assertNotIn("jpegdec", line)
        self.assertIn("max-buffers=1 drop=true", line)
        with self.assertRaises(config.ConfigError):
            settings(CAPTURE="1280x800@100", PIXEL_FORMAT="MJPG", OUTPUT="640x400")


def realsense_settings(**overrides):
    env = {"CELL_CAMERA_SOURCE": "realsense", "CELL_CAMERA_CAPTURE": "640x480@30"}
    return config.load({**env, **{"CELL_CAMERA_" + key: value for key, value in overrides.items()}})


class FakeRealSense:
    """Just enough of pyrealsense2 for the feeder: cameras by serial number, one colour stream, frames by number."""

    __version__ = "2.59.1"

    class camera_info:
        name, serial_number, firmware_version, usb_type_descriptor = "name", "serial", "firmware", "usb"

    class stream:
        color, depth = "color", "depth"

    class format:
        yuyv, z16 = "yuyv", "z16"

    class frame_metadata_value:
        time_of_arrival = "time_of_arrival"

    def __init__(self, cameras, numbers=(), refuse=None, depth=None):
        self.cameras = [{"name": "Intel RealSense D435I", "serial": s, "firmware": "5.16.0.1", "usb": usb} for s, usb in cameras]
        # depth: (width, height) of the depth frames each set carries, each
        # numbered half its colour frame's, so a depth frame repeats as at
        # half the colour rate; None: sets carry no depth.
        self.numbers, self.refuse, self.depth = list(numbers), refuse, depth
        self.requests, self.started, self.stopped = [], [], []
        fake = self

        class Device:
            def __init__(self, info):
                self.info = info

            def get_info(self, key):
                return self.info[key]

        class Context:
            def query_devices(self):
                return [Device(info) for info in fake.cameras]

        class Config:
            def enable_device(self, serial):
                fake.requests.append(("device", serial))

            def enable_stream(self, *args):
                fake.requests.append(("stream",) + args)

        class Frame:
            def __init__(self, number):
                self.number = number

            def __bool__(self):
                return True

            def get_frame_number(self):
                return self.number

            def supports_frame_metadata(self, key):
                return key == "time_of_arrival"

            def get_frame_metadata(self, key):
                return time.time() * 1000 - 20  # librealsense had the frame 20 ms ago

            def get_data(self):
                return bytes([self.number % 256]) * 16

            def get_timestamp(self):
                return 1000.0 + self.number

        class DepthFrame(Frame):
            def get_data(self):
                width, height = fake.depth
                return bytes([self.number % 256, 0]) * (width * height)

        class Frames:
            def __init__(self, number):
                self.number = number

            def get_color_frame(self):
                return Frame(self.number)

            def get_depth_frame(self):
                return DepthFrame(self.number // 2) if fake.depth else None

        class Intrinsics:
            def __init__(self, width, height):
                self.width, self.height, self.fx, self.fy, self.ppx, self.ppy = width, height, 380.5, 380.5, width / 2, height / 2
                self.model, self.coeffs = "distortion.brown_conrady", [0.0] * 5

        class StreamProfile:
            def __init__(self, kind):
                self.kind = kind

            def as_video_stream_profile(self):
                return self

            def get_intrinsics(self):
                return Intrinsics(*(fake.depth if self.kind == "depth" else (640, 480)))

            def get_extrinsics_to(self, other):
                return type("Extrinsics", (), {"rotation": [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0], "translation": [0.015, 0, 0]})()

        class Profile:
            def get_stream(self, kind):
                return StreamProfile(kind)

            def get_device(self):
                sensor = type("Sensor", (), {"get_depth_scale": lambda self: 0.001})()
                return type("Device", (), {"first_depth_sensor": lambda self: sensor})()

        class Pipeline:
            def start(self, request):
                if fake.refuse:
                    raise RuntimeError(fake.refuse)
                fake.started.append(request)
                return Profile()

            def wait_for_frames(self, timeout_ms):
                if not fake.numbers:
                    time.sleep(timeout_ms / 1000 / 50)
                    raise RuntimeError(f"Frame didn't arrive within {timeout_ms}")
                return Frames(fake.numbers.pop(0))

            def stop(self):
                fake.stopped.append(True)

        self.context, self.config, self.pipeline = Context, Config, Pipeline


class FakeGst:
    SECOND = 1_000_000_000

    class Buffer:
        def __init__(self, data):
            self.data, self.pts = data, None

        @classmethod
        def new_wrapped(cls, data):
            return cls(data)


class AppSrc:
    def __init__(self):
        self.pushed = []

    def emit(self, signal, buffer):
        assert signal == "push-buffer"
        self.pushed.append(buffer)


class RealSenseTest(unittest.TestCase):
    def use(self, fake):
        package = type(sys)("pyrealsense2")
        package.pyrealsense2 = fake
        for name, module in (("pyrealsense2", package), ("pyrealsense2.pyrealsense2", fake)):
            sys.modules[name] = module
            self.addCleanup(sys.modules.pop, name, None)
        return fake

    def test_settings_choose_a_realsense_by_serial_not_by_device(self):
        c = realsense_settings()
        self.assertEqual((c.source, c.serial, c.device, c.name, c.pixel_format), ("realsense", None, "", "RealSense", "YUY2"))
        self.assertEqual(realsense_settings(REALSENSE_SERIAL="215122251234").name, "RealSense 215122251234")
        self.assertEqual(settings().source, "v4l2")
        for overrides, refused in (({"DEVICE": DEVICE}, "CELL_CAMERA_DEVICE"), ({"REALSENSE_SERIAL": "21512 2251234"}, "CELL_CAMERA_REALSENSE_SERIAL"),
                                   ({"PIXEL_FORMAT": "RGB"}, "CELL_CAMERA_PIXEL_FORMAT")):
            with self.subTest(overrides=overrides), self.assertRaisesRegex(config.ConfigError, refused):
                realsense_settings(**overrides)
        with self.assertRaisesRegex(config.ConfigError, "CELL_CAMERA_REALSENSE_SERIAL is only for"):
            settings(REALSENSE_SERIAL="215122251234")
        with self.assertRaisesRegex(config.ConfigError, "CELL_CAMERA_SOURCE"):
            settings(SOURCE="librealsense")

    def test_the_pipeline_after_the_camera_is_the_v4l2_cameras(self):
        c = realsense_settings()
        line = capture.pipeline(c, nvidia=True)
        self.assertTrue(line.startswith(
            "appsrc name=camera is-live=true format=time block=false max-buffers=1 leaky-type=downstream "
            "caps=video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! "
            "video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! "), line)
        v4l2 = capture.pipeline(settings(CAPTURE="640x480@30"), nvidia=True)
        self.assertEqual(line.split(" ! ", 2)[2], v4l2.split(" ! ", 2)[2])
        self.assertIn("tee name=split", capture.pipeline(realsense_settings(VIDEO_ADDRESS="100.64.0.11"), nvidia=True))

    def test_the_camera_is_the_named_one_or_the_only_one(self):
        rs = FakeRealSense([("111", "3.2"), ("222", "2.1")])
        self.assertEqual(realsense.find_device(rs, "222").get_info("serial"), "222")
        with self.assertRaisesRegex(RuntimeError, "No RealSense with serial number 333 is connected \\(connected: 111, 222\\)"):
            realsense.find_device(rs, "333")
        with self.assertRaisesRegex(RuntimeError, "2 RealSense cameras are connected \\(111, 222\\): name one"):
            realsense.find_device(rs, None)
        with self.assertRaisesRegex(RuntimeError, "No RealSense is connected"):
            realsense.find_device(FakeRealSense([]), None)
        self.assertEqual(realsense.find_device(FakeRealSense([("111", "3.2")]), None).get_info("serial"), "111")

    def test_frames_are_pushed_once_each_at_the_capture_mode(self):
        rs = self.use(FakeRealSense([("215122251234", "3.2")], numbers=[1, 2, 2, 3]))
        facts = {}
        feeder = realsense.Feeder(realsense_settings(), facts)
        appsrc = AppSrc()
        clock = time.monotonic()
        feeder.start(appsrc, lambda: time.monotonic() - clock + 5.0, FakeGst)
        deadline = time.monotonic() + 2
        while len(appsrc.pushed) < 3 and time.monotonic() < deadline:
            time.sleep(0.01)
        feeder.stop()
        self.assertEqual(rs.requests, [("device", "215122251234"), ("stream", "color", 640, 480, "yuyv", 30)])
        self.assertEqual([buffer.data[0] for buffer in appsrc.pushed], [1, 2, 3])  # the repeated frame 2 is skipped
        stamps = [buffer.pts for buffer in appsrc.pushed]
        self.assertEqual(stamps, sorted(stamps))
        # Stamped when librealsense had the frame, 20 ms before it was pushed.
        self.assertLess(stamps[0], 5.0 * FakeGst.SECOND)
        self.assertEqual((facts["realsense"]["serial"], facts["realsense"]["usb"]), ("215122251234", "3.2"))
        self.assertEqual(rs.stopped, [True])
        feeder.check()  # the camera's silence after its frames is the capture loop's to report, not an error

    def test_a_mode_the_link_cannot_carry_is_refused_with_the_link(self):
        self.use(FakeRealSense([("215122251234", "2.1")], refuse="Couldn't resolve requests"))
        feeder = realsense.Feeder(realsense_settings(CAPTURE="1920x1080@30"), {})
        with self.assertRaisesRegex(RuntimeError, "RealSense could not stream colour 1920x1080@30 YUY2: Couldn't resolve requests "
                                                  "\\(it is connected over USB 2.1, .* use a USB 3 port or a smaller CELL_CAMERA_CAPTURE\\)"):
            feeder.start(AppSrc(), lambda: 0.0, FakeGst)
        feeder.stop()
        self.assertNotIn("USB 3.2", realsense.start_failure(realsense_settings(), "3.2", RuntimeError("busy")))

    def test_depth_is_a_setting_of_a_realsense_only(self):
        self.assertIsNone(realsense_settings().depth)
        self.assertEqual(realsense_settings(REALSENSE_DEPTH="640x480@15").depth, (640, 480, 15))
        for value in ("640x480", "8x8@15", "640x480@0"):
            with self.subTest(value=value), self.assertRaisesRegex(config.ConfigError, "CELL_CAMERA_REALSENSE_DEPTH"):
                realsense_settings(REALSENSE_DEPTH=value)
        with self.assertRaisesRegex(config.ConfigError, "CELL_CAMERA_REALSENSE_DEPTH is only for"):
            settings(REALSENSE_DEPTH="640x480@15")
        with self.assertRaisesRegex(ValueError, "both"):
            realsense.Feeder(realsense_settings(REALSENSE_DEPTH="640x480@15"), {})
        with self.assertRaisesRegex(ValueError, "both"):
            realsense.Feeder(realsense_settings(), {}, DepthFrames(lambda: None, 0.5))

    def test_depth_comes_raw_from_the_colours_session_once_per_frame(self):
        rs = self.use(FakeRealSense([("215122251234", "3.2")], numbers=[2, 3, 4, 5, 6], depth=(64, 48)))
        facts, published = {}, []
        depth = DepthFrames(lambda: None, max_frame_age_seconds=0.5)
        original = depth.publish
        depth.publish = lambda *args: (published.append(args), original(*args))
        feeder = realsense.Feeder(realsense_settings(REALSENSE_DEPTH="64x48@15"), facts, depth)
        appsrc = AppSrc()
        feeder.start(appsrc, lambda: 5.0, FakeGst)
        deadline = time.monotonic() + 2
        while len(appsrc.pushed) < 5 and time.monotonic() < deadline:
            time.sleep(0.01)
        latest = depth.frame(0, timeout=0)
        feeder.stop()
        self.assertEqual(rs.requests, [("device", "215122251234"), ("stream", "color", 640, 480, "yuyv", 30),
                                       ("stream", "depth", 64, 48, "z16", 15)])
        # One session, both streams; each depth frame once though the sets repeat it.
        self.assertEqual([args[5] for args in published], [1, 2, 3])
        self.assertEqual((latest.width, latest.height, latest.scale, latest.number, latest.timestamp_ms, len(latest.data)),
                         (64, 48, 0.001, 3, 1003.0, 64 * 48 * 2))
        self.assertEqual(latest.data[:4], bytes([3, 0, 3, 0]))
        self.assertGreater(latest.source_age, 0.01)  # librealsense had it 20 ms before
        facts_depth = facts["realsense"]["depth"]
        self.assertEqual((facts_depth["depth_scale_m"], facts_depth["intrinsics"]["width"], facts_depth["intrinsics"]["model"],
                          facts_depth["color_intrinsics"]["width"], facts_depth["extrinsics_to_color"]["translation_m"]),
                         (0.001, 64, "brown_conrady", 640, [0.015, 0, 0]))
        self.assertIsNone(depth.frame(0, timeout=0))  # a stopped camera serves no stale depth

    def test_a_depth_frame_of_the_wrong_size_is_refused(self):
        with self.assertRaisesRegex(ValueError, "not 64x48"):
            DepthFrames(lambda: None, 0.5).publish(b"\0" * 10, 64, 48, 0.001, 0.0, 1, 0.0)

    def test_a_refused_depth_mode_is_named_with_the_colour(self):
        text = realsense.start_failure(realsense_settings(REALSENSE_DEPTH="640x480@30"), "2.1", RuntimeError("Couldn't resolve requests"))
        self.assertIn("colour 640x480@30 YUY2 and depth 640x480@30 Z16: Couldn't resolve requests", text)
        self.assertIn("a smaller CELL_CAMERA_CAPTURE or CELL_CAMERA_REALSENSE_DEPTH", text)

    def test_presentation_times_follow_arrival_and_never_go_back(self):
        self.assertAlmostEqual(realsense.presentation(10.0, 1000.0, 999_980.0, None), 9.98)
        self.assertEqual(realsense.presentation(10.0, 1000.0, None, None), 10.0)
        self.assertEqual(realsense.presentation(0.01, 1000.0, 999_000.0, None), 0.0)
        self.assertGreater(realsense.presentation(10.0, 1000.0, 999_000.0, 9.5), 9.5)  # a wall clock step back


class DepthServerTest(unittest.TestCase):
    def serve(self, depth, **overrides):
        Server.allow_reuse_address = True
        c = realsense_settings(LISTEN="127.0.0.1:18091", **overrides)
        hub = FrameHub(lambda hub: None, idle_stop_seconds=1, max_frame_age_seconds=0.5)
        server = Server(c, hub, {"realsense": {"serial": "215122251234"}}, depth=depth)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        return server, f"http://127.0.0.1:{server.server_address[1]}"

    def get(self, url):
        try:
            with urllib.request.urlopen(url, timeout=5) as reply:
                return reply.status, reply.headers, reply.read()
        except urllib.error.HTTPError as error:
            with error:
                return error.code, error.headers, error.read()

    def test_raw_depth_is_served_with_what_reads_it(self):
        touched = []
        depth = DepthFrames(lambda: touched.append(True), max_frame_age_seconds=0.5)
        server, origin = self.serve(depth, REALSENSE_DEPTH="64x48@15")
        depth.publish(bytes(range(256)) * 24, 64, 48, 0.001, 0.02, 41, 123456.789)
        status, headers, body = self.get(origin + "/camera/depth.z16?after=0")
        self.assertEqual((status, headers["Content-Type"], len(body)), (200, "application/octet-stream", 64 * 48 * 2))
        self.assertEqual(body, bytes(range(256)) * 24)
        self.assertEqual((headers["X-Frame-Sequence"], headers["X-Depth-Width"], headers["X-Depth-Height"], headers["X-Depth-Scale"],
                          headers["X-Camera-Frame-Number"], headers["X-Camera-Timestamp-Ms"]),
                         ("1", "64", "48", "0.001", "41", "123456.789"))
        self.assertGreaterEqual(float(headers["X-Source-Age-Ms"]), 20)
        self.assertTrue(touched)  # asking for depth keeps capture running
        self.assertEqual(self.get(origin + "/camera/depth.z16?after=x")[0], 400)
        started = time.monotonic()
        self.assertEqual(self.get(origin + "/camera/depth.z16?after=1")[0], 503)  # nothing newer within 3 s
        self.assertGreater(time.monotonic() - started, 2.5)
        status = json.loads(self.get(origin + "/camera/status")[2])
        self.assertEqual((status["depth"], status["realsense"]["serial"]), ({"enabled": True, "sequence": 1}, "215122251234"))
        self.assertEqual(status["served"]["depth"]["replies"], 1)

    def test_a_camera_without_depth_says_so(self):
        server, origin = self.serve(None)
        self.assertEqual(self.get(origin + "/camera/depth.z16")[0], 404)
        self.assertEqual(json.loads(self.get(origin + "/camera/status")[2])["depth"], {"enabled": False})
