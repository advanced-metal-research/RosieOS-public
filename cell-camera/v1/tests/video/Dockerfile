# The camera's H.264 stream off the robot: Ubuntu 22.04 with the GStreamer
# 1.20 packages the Jetsons run (example-relay and other-relay, L4T R36.4), x264, and
# a software H.264 decoder to check the stream decodes. No NVIDIA elements:
# the service takes its software path here.
#   docker build -t rosie-camera-video cell-camera/v1/tests/video
#   docker run --rm -v "$PWD/cell-camera/v1":/src -w /src rosie-camera-video python3 -m unittest tests/video/test_stream.py
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3 python3-gi gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 \
      gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
      gstreamer1.0-plugins-ugly gstreamer1.0-libav \
    && rm -rf /var/lib/apt/lists/*
