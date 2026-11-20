# Cell camera

A view-only camera service for a cell host. It serves the newest JPEG frame
from one V4L2 camera or an Intel RealSense's colour camera, on demand, to the
OLP server beside it, and a RealSense's raw depth when configured. The OLP server
relays it to its clients, so the Steam Deck pendant shows it under Jog >
Camera and a browser can open it. It has no robot or cabinet interface.

It started as `amr-camera`, written for one cell host. This is
that service with everything camera- or host-specific moved into settings, so
a second cell can run it without editing code. The HTTP contract is unchanged.

## How it works

```text
camera ──► GStreamer pipeline ──► FrameHub ──► HTTP (loopback) ──► OLP relay ──► pendant, browser
(V4L2)     capture.py             hub.py       server.py           cell_camera.go
```

- **On demand.** Nothing is captured until a client asks for a frame, and
  capture stops `CELL_CAMERA_IDLE_STOP_SECONDS` (8 s) after the last request.
- **Newest frame only.** A client asks for the frame after the one it last
  drew (`?after=<sequence>`). Frames are replaced, never queued, so a slow
  client gets a lower frame rate, not a growing delay. A frame older than
  `CELL_CAMERA_MAX_FRAME_AGE_SECONDS` (0.5 s) is never served.
- **JPEG frames, not a video stream.** Each frame stands alone: low latency,
  no decoder state, and any client can show it. The cost is bandwidth: a
  1920 x 1200 frame at quality 88 is about 0.5 MB (measured on a Jetson
  cell host: 21 frames a second and 10.7 MB/s to a Steam Deck through its
  tunnel). `CELL_CAMERA_OUTPUT` and `CELL_CAMERA_JPEG_QUALITY` trade picture
  for bandwidth. Measured with an Arducam B0495 on a Jetson, NVIDIA encoder:
  1920 x 1200 q88 487 KB a frame; 1280 x 800 q80 180 KB;
  960 x 600 q88 158 KB. The software encoder gave 164 KB at 960 x 600 with
  about 90 ms in the pipeline instead of about 33 ms. Camera start to first
  frame was 3.3 s.
- **Loopback only.** The service listens on this host alone; the OLP server
  is its client. It has no authentication of its own and needs none there.

| Module | What it holds |
|---|---|
| `cell_camera/config.py` | Every setting, read from `CELL_CAMERA_*` and checked before hardware is touched |
| `cell_camera/hub.py` | The newest frame, the wait for a newer one, start on demand, stop when idle. No GStreamer |
| `cell_camera/capture.py` | The GStreamer pipeline built from the settings; NVIDIA or software JPEG |
| `cell_camera/realsense.py` | A RealSense's colour frames, through librealsense, into that pipeline in v4l2src's place, and its raw depth frames to the hub's `DepthFrames` |
| `cell_camera/server.py` | The HTTP routes, bounded to eight requests at once |
| `host/` | The systemd template unit, the installer, an example settings file, the RealSense SDK build and udev rule |

## HTTP contract

All under `CELL_CAMERA_PATH_PREFIX` (default `/camera`):

| Route | Answer |
|---|---|
| `GET /status` | JSON: `camera`, `title`, `resolution`, `capture_resolution`, `configured_fps`, `capture_fps`, `active`, `error`, `sequence`, `encoding`, `jpeg_quality`, `idle_stop_seconds` |
| `GET /frame.jpg?after=<sequence>` | The newest JPEG after `<sequence>`, waiting up to 3 s, with `X-Frame-Sequence` and `X-Source-Age-Ms`. 503 while the camera starts or has failed; 400 for a bad cursor |
| `GET /depth.z16?after=<sequence>` | A RealSense with `CELL_CAMERA_REALSENSE_DEPTH`: the newest raw depth frame after `<sequence>`, waiting up to 3 s (see "Raw depth"). 503 while the camera starts or has failed; 400 for a bad cursor; 404 without depth |
| `GET /` | A view-only page that takes its labels from `/status` |

## Settings

One file per camera, `/etc/rosie-cell-camera/<name>.env`, one `KEY=value` per
line, values unquoted (`host/example.env`). The service refuses to start on a
setting it cannot use and names it.

| Setting | Default | Meaning |
|---|---|---|
| `CELL_CAMERA_SOURCE` | `v4l2` | `v4l2`: the camera at `CELL_CAMERA_DEVICE`. `realsense`: an Intel RealSense's colour camera (see "RealSense") |
| `CELL_CAMERA_DEVICE` | required for `v4l2` | The camera, by its `/dev/v4l/by-id/...-video-index0` name. A bare `/dev/videoN` can move when cameras are replugged. The by-id name can also change with the port: a B0641 reports a serial number, and 44 frames a second instead of 10, only on a USB 3 port. Refused for a RealSense |
| `CELL_CAMERA_REALSENSE_SERIAL` | the only RealSense connected | Which RealSense, by the serial number librealsense reports; refused with several connected and none named |
| `CELL_CAMERA_REALSENSE_DEPTH` | none: colour only | `WIDTHxHEIGHT@FPS` of a RealSense's depth stream, read with the colour and served raw at `/depth.z16` (see "Raw depth"); refused for `v4l2` |
| `CELL_CAMERA_CAPTURE` | required | `WIDTHxHEIGHT@FPS` the camera is asked for, for example `1920x1200@30` |
| `CELL_CAMERA_PIXEL_FORMAT` | `YUY2` | The camera's raw format at that mode |
| `CELL_CAMERA_OUTPUT` | the capture size, or the crop's | `WIDTHxHEIGHT` of each JPEG; smaller scales down |
| `CELL_CAMERA_CROP` | none | `WIDTHxHEIGHT+LEFT+TOP`: keep only that part of the capture, in capture pixels. For a lens whose image circle is smaller than the sensor: at its full-field modes a B0498 has dark corners and a pink band along the bottom, and `1920x1080@30` with `1080x810+420+75` keeps the clean centre at 4:3. Cropped on the VIC on a Jetson, with `videocrop` elsewhere. |
| `CELL_CAMERA_JPEG_QUALITY` | `88` | 1 to 100 |
| `CELL_CAMERA_ENCODER` | `auto` | `nvidia` (Jetson `nvvidconv` + `nvjpegenc`), `software` (`videoconvert` + `jpegenc`), or `auto`: NVIDIA when the host has it |
| `CELL_CAMERA_LISTEN` | `127.0.0.1:8088` | Loopback address and port |
| `CELL_CAMERA_PATH_PREFIX` | `/camera` | Where the routes live |
| `CELL_CAMERA_NAME` | the device's file name | What status and the page call the camera |
| `CELL_CAMERA_TITLE` | `Cell camera` | The page's heading, for example `Cell 3 / Bench` |
| `CELL_CAMERA_IDLE_STOP_SECONDS` | `8` | Capture stops this long after the last frame request |
| `CELL_CAMERA_MAX_FRAME_AGE_SECONDS` | `0.5` | A frame older than this is never served |

Find a camera's device name and modes on the host:

```sh
ls /dev/v4l/by-id/
gst-device-monitor-1.0 Video/Source        # or: v4l2-ctl -d <device> --list-formats-ext
```

## Install on a cell host

For a cell with a manifest, the deploy tool does this from the cell's
`cell_deploy.camera` block (`go run . camera install --manifest <cell>`,
`steamdeck/real/v4/README.md`, "Cell camera"). By hand:

Needs `python3`, `python3-gi` and GStreamer with the V4L2 and JPEG plugins
(present on a Jetson). As root, from a checkout or a copied `cell-camera/v1`:

```sh
host/install.sh <name> <settings.env> --start
```

It checks the settings with the service's own code first, copies the package
to `/opt/rosie-cell-camera/<revision>` (selected as `current`), installs
`rosie-cell-camera@.service` and `/etc/rosie-cell-camera/<name>.env`, enables
`rosie-cell-camera@<name>` and, with `--start`, restarts it. A host can run
several instances, one per camera, each with its own port.

The unit runs as a throwaway user in the `video` and `render` groups, at low
CPU and IO priority, capped at half a core and 384 MB, with a read-only
system: a camera view must never compete with the OLP server or the planner.

```sh
systemctl status rosie-cell-camera@<name>
curl -s http://127.0.0.1:8088/camera/status
```

To open the viewer page from another machine on the tailnet, publish the
service with its prefix on both sides, because `tailscale serve` strips the
path it matches and this service answers only under its prefix:

```sh
sudo tailscale serve --bg --https=443 --set-path=/camera http://127.0.0.1:8088/camera
```

The page is then at `https://<host>.<tailnet>/camera/`.

## Add a camera to a cell

1. Plug the camera into the cell host (the relay) and
   find its `/dev/v4l/by-id/...-video-index0` name and a capture mode.
2. Add a `cell_deploy.camera` block to the cell manifest
   (`steamdeck/real/v4/README.md`, "Adding a new robot cell").
3. Build that commit on the cell (`realtime-core build`), then
   `go run . camera install --manifest <cell> --deployment <sha>`: it
   installs the service and proves a frame.
4. `go run . olp install --manifest <cell> --deployment <sha>`: the OLP unit
   then carries `OFFLINE_PROGRAMMING_CELL_CAMERA_ORIGIN` and relays the camera
   (`offline-programming/v1/README.md`, "Cell camera").
5. Any pendant built with the camera view shows it under Jog > Camera. No
   pendant setting changes: the Deck already reaches that OLP server.

Nothing in the service, the relay or the pendant knows which camera or
which cell it is: a second camera model is another settings block.

## Camera-native JPEG

`CELL_CAMERA_PIXEL_FORMAT=MJPG` passes the camera's JPEG directly to the newest-frame hub without decode, scale or re-encode. Output must equal capture size; JPEG quality is camera-controlled (status reports null). Zero-byte USB buffer padding after the JPEG end marker is removed (observed on OV9281); truncated JPEGs and nonzero trailers are dropped. An isolated damaged USB frame is skipped without restarting capture; three seconds without valid frames reports a camera failure. Status exposes the dropped-frame count. This avoids a raw USB bandwidth limit and encoder load for cameras such as OV9281. The service still captures on demand and bounds stale frames and queue length.

## RealSense

`CELL_CAMERA_SOURCE=realsense` reads an Intel RealSense's colour camera with
librealsense's RSUSB (libusb) backend instead of a V4L2 node. Its V4L2 nodes
deliver each frame's metadata on a second node that librealsense pairs back up
with the frame; colour frames that arrive cut short each cost a metadata buffer
until the colour stream stops for good. RSUSB reads the frame and its metadata
in one USB transfer. Colour frames go into an `appsrc` in `v4l2src`'s place, as
YUY2 straight off the USB, so crop, scale, the encoder, the low-data video and
the stage timings are the V4L2 camera's own. `capture` is the time from
librealsense having the whole frame until the pipeline takes it.

- **Depth is optional and raw.** librealsense lets one process hold the
  camera, so depth is read in the colour's session or not at all (see "Raw
  depth"). `PIXEL_FORMAT` stays `YUY2`.
- **The camera refuses a mode with its reason.** A mode the camera cannot
  stream fails the capture with librealsense's words in `/status`'s `error`,
  and names the USB link when it is USB 2, which offers fewer colour modes.
  `/status` has a `realsense` object: model, serial number, firmware, USB link
  and librealsense version, once a capture has opened the camera.
- **The SDK is built for the host.** `host/build-pyrealsense2.sh` builds
  pyrealsense2 (librealsense v2.59.1, pinned with all its inputs, RSUSB,
  linked statically) for the host's `/usr/bin/python3`, the unit's
  interpreter: CPython 3.10 on the Jetsons' Ubuntu 22.04, for which PyPI has
  no RSUSB build. On the host it needs `build-essential cmake git
  libusb-1.0-0-dev pkg-config python3-dev` and writes
  `~/.rosie/realsense/pyrealsense2-2.59.1-<arch>`. With `--arch aarch64` on an
  x86_64 host it builds the same package in an emulated arm64 Ubuntu 22.04
  (qemu-user-static and bubblewrap, no root) and keeps it only if it is AArch64
  and needs no glibc past 2.35. `--download-only` and `--offline-prepare` split
  fetching from building.
- **Install.** `host/install.sh <name> <settings.env> --start --realsense-sdk
  <dir>` refuses a package `/usr/bin/python3` cannot load, copies it beside
  the service in the selected revision (the unit reads nothing else), and
  installs `host/99-rosie-realsense.rules`, which gives the `video` group the
  D435i's USB device (8086:0b3a; another model is another line). A revision
  keeps the SDK of the one it replaces, so installing another camera does not
  take it from a RealSense.

Unverified on a camera: every mode, the udev rule under the unit's dynamic
user, the D435i's colour modes on a USB 2 link, and colour and depth together
on one. The tests cover the settings, the pipeline, the camera choice, the
feeding and the depth with a stand-in for pyrealsense2.

### Raw depth

With `CELL_CAMERA_REALSENSE_DEPTH=640x480@15` the colour's librealsense
session also streams depth (Z16), and `GET <prefix>/depth.z16?after=<sequence>`
answers the newest depth frame after `<sequence>`, as `/frame.jpg` answers
JPEGs. It is for programs on the backend (the OLP relays it at
`.../cell-camera/depth.z16`); no client shows it.

- **Body:** `application/octet-stream`, width x height unsigned 16-bit depth
  units, row by row, in the host's byte order (little-endian on arm64 and
  x86_64; `/status` says `byte_order`). 0 is no depth at that pixel. Metres
  are units x `X-Depth-Scale`.
- **Headers:** `X-Frame-Sequence` (the cursor), `X-Depth-Width`,
  `X-Depth-Height`, `X-Depth-Scale` (metres per unit), `X-Source-Age-Ms` (since
  librealsense had the whole frame), `X-Camera-Frame-Number` and
  `X-Camera-Timestamp-Ms` (librealsense's own, to pair a depth frame with
  others).
- **Unaligned.** The frame is in the depth camera's own pixels.
  `/status`'s `realsense.depth` has the depth scale, both cameras'
  intrinsics (`intrinsics`, `color_intrinsics`: `fx`, `fy`, `ppx`, `ppy`,
  distortion `model` and `coeffs`) and `extrinsics_to_color` (3x3 rotation,
  column-major, and translation in metres, librealsense's convention), so a
  client can deproject a pixel or align depth to colour itself. Aligning here
  would reproject every frame on the cell host whether anyone reads it or not.
- **Capture is shared.** A depth request keeps the camera running as a frame
  request does, so the colour pipeline runs too, and while anyone watches the
  colour the depth is read as well: the USB link carries both whenever the
  camera runs. A frame older than `CELL_CAMERA_MAX_FRAME_AGE_SECONDS` is not
  served.

## Tests

```sh
python3 -m unittest discover -s tests      # from cell-camera/v1; standard library only
```

They cover the settings and their refusals, the pipeline each encoder would
run, the frame hub (cursor, start on demand, stop when idle, a failing
camera, a stale frame) and the HTTP contract, with a stand-in for the camera.
Capture itself needs a camera: check it on the host with the two `curl`
lines above.

## Measuring latency

A frame crosses six stages. Each has a number, so a change is judged by the
stage it moves, not by how the picture feels.

| Stage | Measured by | Where it is reported |
|---|---|---|
| `capture`: the driver's timestamp until the frame leaves `v4l2src` | pad probe | `Server-Timing` |
| `queue`, `convert`, `encode`, `sink`: the rest of the pipeline, until this service has the JPEG | pad probes | `Server-Timing` |
| `held`: published until a reply takes it | service clock | `Server-Timing` |
| `wait`: a request waiting for a frame newer than its cursor | service clock | `Server-Timing` |
| `relay`: the OLP relay's round trip to this service (includes `wait`) | OLP clock | `Server-Timing`, added by the relay |
| network: the request's time outside both servers | client clock minus `wait` and the relay's overhead | profiler, pendant trace |
| decode and paint | pendant clock | pendant trace |

`X-Source-Age-Ms` is the frame's age when the reply left the service: the
pipeline stages plus `held`. Durations come from one clock each, so no two
hosts' clocks are compared.

Not measured: the sensor's exposure and readout before the driver's
timestamp, and the Deck's compositor and panel after the pendant paints. To
see those, point the camera at a millisecond clock on a screen and photograph
the Deck beside it.

`tools/camera_latency.py` asks for frames at the pendant's pace from one
vantage point and writes a per-frame trace (`.trace.jsonl`, and `.trace.json`
for Perfetto), `.metrics.json` and a `.log`. Run it at each hop; the
difference between two hops is the hop:

```sh
# on the cell host, at this service, then at the OLP relay
python3 tools/camera_latency.py --url http://127.0.0.1:8088/camera --label service
python3 tools/camera_latency.py --url http://127.0.0.1:8794/api/offline-programming/v1/cell-camera --label relay
# on a Deck, through its tunnel
python3 camera_latency.py --url http://127.0.0.1:18794/api/offline-programming/v1/cell-camera --label deck
```

The pendant writes its own trace, one line per frame painted, when
`ROSIE_PENDANT_CAMERA_TRACE` names a file. Summarize it with
`tools/camera_latency.py --pendant-trace <file> --label pendant`.

## Low-data video

The JPEG stream is about 235 KB a frame. One browser viewer used about 25 GB
an hour and a Deck about 12.7 GB (on a Rosie 1430 cell), and a conference hotspot
ran out of data while the cell was teleoperated. With video on, the camera
also sends H.264 at 960 pixels wide and 15 frames a second, capped at
800 kbit/s: at most 0.36 GB an hour per viewer, and much less for a still
cell. The JPEG stream stays, for clients without WebRTC and full quality.

How it is built, and why:

- **The camera service encodes once** (`video.py`). The capture splits after
  the camera; the video branch drops to the video frame rate, scales on the
  VIC and encodes with x264, because the Orin Nano has no hardware video
  encoder. A valve keeps x264 idle until something reads the stream.
  Measured on an Orin Nano cell host: 0.15 of a core for a simple scene, 0.33
  at the bit rate cap for pure noise.
- **`cell-video` sends it to the viewers** (`video/`, Go, Pion), as its own
  unit `rosie-cell-video@<instance>`, so a video fault cannot stop the OLP
  server that carries jog control. It reads the camera's stream only while
  someone watches. Video goes to each viewer over WebRTC: UDP on the host's
  Tailscale address only, one port for every viewer, outside the SSH tunnel
  that carries jog replies. Lost packets are resent and a viewer gets a key
  frame when it joins or asks.
- **Browsers set up a session through the OLP** with WHEP at
  `/api/offline-programming/v1/cell-camera/video/whep`, a few kilobytes each.
- **The Steam Deck gets the stream relayed by the OLP** at
  `/api/offline-programming/v1/cell-camera/video.h264`, through the SSH tunnel
  that carries its jog replies, and decodes it with OpenH264. The Decks run
  Tailscale in userspace-networking mode, so apps on them have no route for
  the sender's UDP. The OLP drops units to the next
  key frame when the Deck falls behind (`offline-programming/v1`
  `cell_video_stream.go`).

WebRTC is not done in the camera service because GStreamer 1.20's Python
binding crashed reading webrtcbin's replies on the Jetsons' Ubuntu 22.04.

Turn it on with the camera block's `video` in the cell manifest:

```json
"video": {"address": "tailscale", "port": 40400, "listen": "127.0.0.1:8090"}
```

`camera install` reads the relay's Tailscale address for `"tailscale"`,
writes the `CELL_CAMERA_VIDEO_*` settings (`host/example.env`), installs the
tree's `bin/cell-video` (built by `realtime-core build`), and checks the
sender answers. `olp install` gives the OLP unit
`OFFLINE_PROGRAMMING_CELL_VIDEO_ORIGIN`. Optional `width`, `fps` and `kbps`
override 960, 15 and 800.

### The stream format

This is the one description of the format. Every reader below parses it on
its own; change them together.

  GET  <prefix>/video.h264       200, then per access unit a 4-byte big-endian length and
                                 the unit's H.264 Annex-B bytes. The first unit is a key frame
                                 (SPS, PPS, IDR). X-Video-FPS is the encoded frame rate. The
                                 stream ends with no unit if the camera gives none in 8 s.
  POST <prefix>/video/keyframe   asks the encoder for a key frame (200, or 503 without a pipeline)

H.264 constrained baseline, no B-frames, so units arrive in display order.
Every reader refuses a unit of 0 bytes or over 4 MiB and ends the stream.

| Writes or reads it | Where |
|---|---|
| writes: the camera service | `cell_camera/video.py` `frame_unit`, `server.py` `_stream_video` |
| reads: the WebRTC sender | `video/sender.go` `readStream` |
| reads and re-sends to the Deck: the OLP relay | `offline-programming/v1/cell_video_stream.go` `readUnits` |
| reads: the Deck pendant | `steamdeck/real/v5/src/camera_view.cpp` `videoData` |

Tests: `tests/video` runs the stream end to end in an Ubuntu 22.04 container
with the Jetsons' GStreamer (see its Dockerfile); `go test` in `video/` runs a
WebRTC viewer against a fake camera.

## Next steps

In order. Nothing here is built yet.

1. **Measure jog lag with the Deck's video on.** The OLP holds at most about
   half a second of video for a slow Deck, but the SSH tunnel holds more
   before the OLP sees the Deck slow down. OpenSSH's default channel window is
   2 MB (`CHAN_TCP_WINDOW_DEFAULT` in OpenSSH's `channels.h`), about 20 s of
   video, and the Deck's jog replies share that one TCP connection, so on a
   slow link a jog reply can wait behind video. Measure jog reply latency over
   a phone hotspot with Cell cam on and off. If video delays jog, the cheap
   fix is a second SSH connection for video only, so jog replies never queue
   behind it.
2. **Move the Deck to WebRTC.** WebRTC sends video over UDP, outside the jog
   tunnel, and recovers from loss without stalling. The Deck needs a UDP route
   to the sender: kernel-mode Tailscale on each Deck (today userspace, with no
   sudo for the `deck` user). The Deck's WebRTC client is ready and tested,
   unused: `video/viewer` and `video/cmd/cell-video-view`. Then the OLP relay
   is the fallback for a Deck without the route.
3. **Pin the stream format with one recorded sample.** A file of a few real
   units (a key frame and the frames after it), read by each reader's tests in
   the table above, so a change to one side fails the others' tests. Today
   each test makes its own units.

## Not here yet

- A RealSense's depth in a client: the service and the OLP serve it raw;
  nothing shows it, aligns it to colour, or reads the D435i's IMU.
- A cell's further cameras (`cell_deploy.extra_cameras`, installed with
  `camera install --camera <instance>`) are served by the OLP at
  `/cell-cameras/<instance>`. The pendant shows the cell camera and the
  `wrist` camera; no client lists the others yet.
- One JPEG output size per camera: a client cannot ask for a smaller frame
  (the low-data video is the smaller choice).
- The video bit rate is fixed: it does not adapt to a viewer's link.
- No video for an MJPG camera (the OV9281 wrist camera): its frames would be
  decoded on the CPU before encoding.
- No recording, no audio, no camera controls (exposure, focus).
