#include "h264_decoder.hpp"
#include <QElapsedTimer>
#include <algorithm>
#include <wels/codec_api.h>

namespace {
ISVCDecoder *as(void *p) { return static_cast<ISVCDecoder *>(p); }
inline unsigned char clamp8(int v) { return static_cast<unsigned char>(std::clamp(v, 0, 255)); }
} // namespace

H264Decoder::H264Decoder() {
  ISVCDecoder *d = nullptr;
  if (WelsCreateDecoder(&d) != 0 || !d) return;
  SDecodingParam param{};
  param.sVideoProperty.eVideoBsType = VIDEO_BITSTREAM_AVC;
  // No concealment: a unit with a lost reference is reported broken (the view
  // asks for a key frame and keeps the last good picture) instead of drawn
  // with invented blocks.
  param.eEcActiveIdc = ERROR_CON_DISABLE;
  if (d->Initialize(&param) != 0) {
    WelsDestroyDecoder(d);
    return;
  }
  decoder = d;
}

H264Decoder::~H264Decoder() {
  if (!decoder) return;
  as(decoder)->Uninitialize();
  WelsDestroyDecoder(as(decoder));
}

H264Decoder::Result H264Decoder::decode(const QByteArray &unit) {
  Result r;
  if (!decoder || unit.isEmpty()) return r;
  QElapsedTimer timer;
  timer.start();
  unsigned char *planes[3] = {nullptr, nullptr, nullptr};
  SBufferInfo info{};
  const DECODING_STATE state =
      as(decoder)->DecodeFrameNoDelay(reinterpret_cast<const unsigned char *>(unit.constData()), unit.size(), planes, &info);
  if (state != dsErrorFree) {
    r.broken = true;
  } else if (info.iBufferStatus == 1 && planes[0] && planes[1] && planes[2]) {
    const auto &b = info.UsrData.sSystemBuffer;
    r.picture = i420ToImage(planes[0], planes[1], planes[2], b.iWidth, b.iHeight, b.iStride[0], b.iStride[1]);
  }
  r.decodeMs = timer.nsecsElapsed() / 1e6;
  return r;
}

QImage i420ToImage(const unsigned char *y, const unsigned char *u, const unsigned char *v, int width, int height, int yStride,
                   int uvStride) {
  if (width <= 0 || height <= 0) return {};
  QImage image(width, height, QImage::Format_RGB32);
  // BT.601 limited range, 8-bit fixed point (x 256).
  for (int row = 0; row < height; ++row) {
    const unsigned char *yr = y + row * yStride;
    const unsigned char *ur = u + (row / 2) * uvStride;
    const unsigned char *vr = v + (row / 2) * uvStride;
    auto *out = reinterpret_cast<QRgb *>(image.scanLine(row));
    for (int col = 0; col < width; ++col) {
      const int c = 298 * (yr[col] - 16);
      const int d = ur[col / 2] - 128;
      const int e = vr[col / 2] - 128;
      out[col] = qRgb(clamp8((c + 409 * e + 128) >> 8), clamp8((c - 100 * d - 208 * e + 128) >> 8), clamp8((c + 516 * d + 128) >> 8));
    }
  }
  return image;
}
