# Ubuntu 22.04 linux/amd64 build host for steamdeck/real/v5: package.sh needs
# Ubuntu 22.04's Qt 5.15, Assimp and dpkg copyright files, and the Deck is
# x86_64. On an arm64 Mac this image runs under emulation; `pendant build`
# cross-compiles the Go OLP server natively and passes it in, so only the C++
# app, the node bundle and the checks run emulated.
# Built by: go run . pendant build (steamdeck/real/v4), tag from the cell
# manifest's pendant_deploy.builder.image; rebuilt when this file changes
# (the image carries its sha256 as the rosie.container.sha256 label).
# libopenh264-dev: the camera view's H.264 decoder (src/h264_decoder.cpp);
# package.sh bundles libopenh264.so.6.
FROM --platform=linux/amd64 ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential pkg-config qtbase5-dev qtdeclarative5-dev libassimp-dev libopenh264-dev \
      curl ca-certificates git python3 xz-utils \
    && rm -rf /var/lib/apt/lists/*
RUN mkdir -p /root/.rosie/bin \
    && curl -fsSL https://go.dev/dl/go1.26.2.linux-amd64.tar.gz | tar -C /root/.rosie -xz \
    && mv /root/.rosie/go /root/.rosie/go-1.26.2 \
    && curl -fsSL https://nodejs.org/dist/v22.13.1/node-v22.13.1-linux-x64.tar.xz | tar -C /root/.rosie -xJ \
    && mv /root/.rosie/node-v22.13.1-linux-x64 /root/.rosie/node-22.13.1 \
    && curl -fsSL -o /root/.rosie/bin/pixi https://github.com/prefix-dev/pixi/releases/latest/download/pixi-x86_64-unknown-linux-musl \
    && chmod +x /root/.rosie/bin/pixi
ENV PATH=/root/.rosie/bin:/root/.rosie/go-1.26.2/bin:/root/.rosie/node-22.13.1/bin:$PATH
