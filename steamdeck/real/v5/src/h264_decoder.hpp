#pragma once
#include <QByteArray>
#include <QImage>

// Decodes the cell camera's low-data H.264 stream (constrained baseline, the
// camera's x264, cell-camera/v1 video.py) to pictures, with OpenH264 (bundled
// in the package's lib/). One access unit in, at most one picture out, with
// no frame of delay: the pendant draws each unit as it arrives.
class H264Decoder {
public:
  H264Decoder();
  ~H264Decoder();
  H264Decoder(const H264Decoder &) = delete;
  H264Decoder &operator=(const H264Decoder &) = delete;

  // OpenH264 started; false only when its library failed to initialise.
  bool ok() const { return decoder != nullptr; }

  struct Result {
    QImage picture;      // null when the unit made no picture (parameter sets, or broken)
    bool broken = false; // the decoder reported an error: ask the sender for a key frame
    double decodeMs = 0;
  };
  // Decodes one Annex-B access unit.
  Result decode(const QByteArray &unit);

private:
  void *decoder = nullptr; // ISVCDecoder*, kept out of this header
};

// I420 planes to an RGB32 picture, BT.601 limited range (16-235), the range
// the Jetson's I420 conversion produces (unverified against a full-range
// signal in the stream's VUI, which OpenH264 does not report).
QImage i420ToImage(const unsigned char *y, const unsigned char *u, const unsigned char *v, int width, int height, int yStride,
                   int uvStride);
