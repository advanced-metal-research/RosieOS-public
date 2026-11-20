"""The camera's H.264 stream end to end, off the robot (run in tests/video/Dockerfile's image).

A camera service on a test picture streams video.h264 to one reader. The test
checks the stream starts at a key frame, runs at the configured rate, decodes
as H.264, is counted in status, and that the encoder's gate closes when the
reader leaves.
"""
from __future__ import annotations

import json
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from cell_camera import capture, config  # noqa: E402
from cell_camera.hub import FrameHub  # noqa: E402
from cell_camera.server import Server  # noqa: E402
from cell_camera.video import VideoStream, is_key_unit  # noqa: E402

PORT = 18192


class StreamTest(unittest.TestCase):
    def setUp(self):
        env = {"CELL_CAMERA_DEVICE": "/dev/video0", "CELL_CAMERA_CAPTURE": "1280x720@30", "CELL_CAMERA_PIXEL_FORMAT": "I420",
               "CELL_CAMERA_LISTEN": f"127.0.0.1:{PORT}", "CELL_CAMERA_ENCODER": "software",
               "CELL_CAMERA_VIDEO_ADDRESS": "100.64.0.1", "CELL_CAMERA_VIDEO_WIDTH": "640"}
        self.config = config.load(env)
        parts: dict = {}
        self.video = VideoStream(self.config, lambda: parts["hub"].touch(), lambda k, n, r: parts["server"].count(k, n, r))
        produce = capture.producer(self.config, {}, self.video, source="videotestsrc is-live=true pattern=ball")
        self.hub = parts["hub"] = FrameHub(produce, self.config.idle_stop_seconds, self.config.max_frame_age_seconds)
        Server.allow_reuse_address = True
        self.server = parts["server"] = Server(self.config, self.hub, {}, self.video)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.origin = f"http://127.0.0.1:{PORT}/camera"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def status(self) -> dict:
        with urllib.request.urlopen(self.origin + "/status", timeout=5) as reply:
            return json.load(reply)

    def test_the_stream_starts_at_a_key_frame_runs_at_the_video_rate_and_decodes(self):
        units = []
        with urllib.request.urlopen(self.origin + "/video.h264", timeout=15) as reply:
            # The sender times its samples from this, not from a setting.
            self.assertEqual(reply.headers["X-Video-FPS"], "15")
            first = time.monotonic()
            while time.monotonic() - first < 4:
                head = reply.read(4)
                (length,) = struct.unpack(">I", head)
                units.append(reply.read(length))
                if len(units) == 1:
                    first = time.monotonic()
            self.assertTrue(self.status()["video"]["encoding"])
        rate = (len(units) - 1) / 4
        self.assertTrue(is_key_unit(units[0]), "the stream must start at a key frame")
        self.assertTrue(13 <= rate <= 17, f"{rate:.1f} units a second, want about 15")
        with tempfile.NamedTemporaryFile(suffix=".h264") as f:
            f.write(b"".join(units))
            f.flush()
            decoded = subprocess.run(["gst-launch-1.0", "-q", "filesrc", f"location={f.name}", "!", "h264parse", "!",
                                      "avdec_h264", "!", "fakesink", "sync=false"], capture_output=True, timeout=30)
        self.assertEqual(decoded.returncode, 0, decoded.stderr[-500:])
        kbps = sum(len(u) for u in units) * 8 / 1000 / 4
        print(f"\nvideo: {len(units)} units in 4 s, {kbps:.0f} kbit/s, first unit {len(units[0])} B", file=sys.stderr)
        deadline = time.monotonic() + 5
        while self.status()["video"]["readers"] and time.monotonic() < deadline:
            time.sleep(0.2)
        status = self.status()
        self.assertEqual(status["video"]["readers"], 0)
        self.assertFalse(status["video"]["encoding"], "the encoder's gate must close when the reader leaves")
        self.assertGreater(status["served"]["video"]["bytes"], sum(len(u) for u in units))
        self.assertEqual(status["served"]["video"]["replies"], 1)

    def test_a_keyframe_request_is_answered(self):
        request = urllib.request.Request(self.origin + "/video/keyframe", method="POST", data=b"")
        try:
            urllib.request.urlopen(request, timeout=5)
        except urllib.error.HTTPError as error:
            self.assertEqual(error.code, 503)  # no pipeline yet: nobody asked for frames


if __name__ == "__main__":
    unittest.main()
