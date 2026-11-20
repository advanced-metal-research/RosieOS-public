// The cell camera as low-data video. H264Decoder against OpenH264's own
// encoder (constrained baseline, like the camera's x264): colours come back,
// a unit without its key frame is reported broken rather than drawn. Then the
// Jog page's camera slot against a stand-in for the OLP server's relayed
// stream (cell_video_stream.go): it draws each unit, sends this app's
// identity, closes the stream when hidden, asks for a key frame when it
// joined without one, and shows JPEG frames when the cell has no video or
// the video never starts.
// usage: QT_QPA_PLATFORM=offscreen video-check
#include "camera_view.hpp"
#include "h264_decoder.hpp"
#include <QApplication>
#include <QBuffer>
#include <QElapsedTimer>
#include <QFile>
#include <QTcpServer>
#include <QTcpSocket>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <wels/codec_api.h>

#define CHECK(condition)                                                                 \
  do {                                                                                   \
    if (!(condition)) {                                                                  \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
      std::exit(1);                                                                      \
    }                                                                                    \
  } while (false)

namespace {
template <typename Done> bool spin(int limitMs, Done done) {
  QElapsedTimer waited;
  waited.start();
  while (!done() && waited.elapsed() < limitMs) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
  return done();
}

constexpr int W = 320, H = 240;

// BT.601 limited-range YUV for a colour, as a camera's conversion makes it.
struct Yuv {
  unsigned char y, u, v;
};
constexpr Yuv red{81, 90, 240}, blue{41, 240, 110};

// Annex-B access units of `count` frames, the first `split` red and the rest
// blue; `wasRed` says which (the encoder may skip a frame).
QList<QByteArray> encode(int count, int split, QList<bool> &wasRed) {
  ISVCEncoder *enc = nullptr;
  CHECK(WelsCreateSVCEncoder(&enc) == 0 && enc);
  SEncParamBase p{};
  p.iUsageType = CAMERA_VIDEO_REAL_TIME;
  p.fMaxFrameRate = 15;
  p.iPicWidth = W;
  p.iPicHeight = H;
  p.iTargetBitrate = 800000;
  p.iRCMode = RC_BITRATE_MODE;
  CHECK(enc->Initialize(&p) == 0);
  QByteArray y(W * H, 0), u(W * H / 4, 0), v(W * H / 4, 0);
  QList<QByteArray> units;
  for (int i = 0; i < count; ++i) {
    const Yuv c = i < split ? red : blue;
    y.fill(char(c.y));
    u.fill(char(c.u));
    v.fill(char(c.v));
    SSourcePicture pic{};
    pic.iPicWidth = W;
    pic.iPicHeight = H;
    pic.iColorFormat = videoFormatI420;
    pic.iStride[0] = W;
    pic.iStride[1] = pic.iStride[2] = W / 2;
    pic.pData[0] = reinterpret_cast<unsigned char *>(y.data());
    pic.pData[1] = reinterpret_cast<unsigned char *>(u.data());
    pic.pData[2] = reinterpret_cast<unsigned char *>(v.data());
    pic.uiTimeStamp = i * 66;
    SFrameBSInfo info{};
    CHECK(enc->EncodeFrame(&pic, &info) == cmResultSuccess);
    if (info.eFrameType == videoFrameTypeSkip) continue;
    QByteArray unit;
    for (int l = 0; l < info.iLayerNum; ++l) {
      const SLayerBSInfo &layer = info.sLayerInfo[l];
      int size = 0;
      for (int n = 0; n < layer.iNalCount; ++n) size += layer.pNalLengthInByte[n];
      unit.append(reinterpret_cast<const char *>(layer.pBsBuf), size);
    }
    units << unit;
    wasRed << (i < split);
  }
  enc->Uninitialize();
  WelsDestroySVCEncoder(enc);
  return units;
}

bool near(QRgb a, int r, int g, int b) { return std::abs(qRed(a) - r) < 24 && std::abs(qGreen(a) - g) < 24 && std::abs(qBlue(a) - b) < 24; }

QByteArray framed(const QList<QByteArray> &units) {
  QByteArray out;
  for (const QByteArray &u : units) {
    const quint32 n = u.size();
    const char head[4] = {char(n >> 24), char(n >> 16), char(n >> 8), char(n)};
    out.append(head, 4).append(u);
  }
  return out;
}


} // namespace

int main(int argc, char **argv) {
  QApplication app(argc, argv);

  // The decoder: each unit makes its picture at once, in its colour.
  QList<bool> wasRed;
  const QList<QByteArray> units = encode(30, 15, wasRed);
  CHECK(units.size() >= 25);
  {
    H264Decoder decoder;
    CHECK(decoder.ok());
    int pictures = 0;
    for (int i = 0; i < units.size(); ++i) {
      const H264Decoder::Result r = decoder.decode(units[i]);
      CHECK(!r.broken && !r.picture.isNull() && r.picture.size() == QSize(W, H));
      const QRgb centre = r.picture.pixel(W / 2, H / 2);
      CHECK(wasRed[i] ? near(centre, 255, 0, 0) : near(centre, 0, 0, 255));
      ++pictures;
    }
    CHECK(pictures == units.size());
  }
  // Joining after the key frame: nothing drawn, and the caller hears it.
  {
    H264Decoder decoder;
    const H264Decoder::Result r = decoder.decode(units[3]);
    CHECK(r.picture.isNull() && r.broken);
  }

  // A stand-in for the OLP server: the relayed stream (or 404 / 503), key
  // frame requests, and JPEG frames for the fallback.
  enum class Video { Stream, Broken, NoVideo, Fail };
  Video video = Video::Stream;
  QImage picture(640, 400, QImage::Format_RGB32);
  picture.fill(QColor(40, 90, 140));
  QByteArray jpeg;
  {
    QBuffer out(&jpeg);
    out.open(QIODevice::WriteOnly);
    CHECK(picture.save(&out, "JPG", 88));
  }
  int jpegAsked = 0, streams = 0, openStreams = 0, keyAsked = 0;
  QByteArray streamClient;
  QTcpServer server;
  CHECK(server.listen(QHostAddress::LocalHost, 0));
  QObject::connect(&server, &QTcpServer::newConnection, [&] {
    auto *socket = server.nextPendingConnection();
    auto request = std::make_shared<QByteArray>();
    QObject::connect(socket, &QTcpSocket::readyRead, socket, [&, socket, request] {
      *request += socket->readAll();
      if (!request->contains("\r\n\r\n")) return;
      const QByteArray line = request->left(request->indexOf("\r\n"));
      if (line.startsWith("GET ") && line.contains("/cell-camera/video.h264")) {
        ++streams;
        const int at = request->indexOf("X-Rosie-Client: ");
        streamClient = at < 0 ? QByteArray() : request->mid(at + 16, request->indexOf("\r\n", at) - at - 16);
        if (video == Video::NoVideo || video == Video::Fail) {
          const QByteArray body = video == Video::NoVideo ? R"({"error":"cell_video_not_configured"})" : R"({"error":"cell_camera_unavailable"})";
          socket->write((video == Video::NoVideo ? "HTTP/1.1 404 Not Found" : "HTTP/1.1 503 Service Unavailable") +
                        QByteArray("\r\nContent-Type: application/json\r\nContent-Length: ") + QByteArray::number(body.size()) +
                        "\r\nConnection: close\r\n\r\n" + body);
          socket->disconnectFromHost();
          return;
        }
        // Open-ended, like the relay: the body is the stream until it closes.
        ++openStreams;
        QObject::connect(socket, &QTcpSocket::disconnected, socket, [&] { --openStreams; });
        socket->write("HTTP/1.1 200 OK\r\nContent-Type: video/h264\r\nConnection: close\r\n\r\n" +
                      framed(video == Video::Broken ? units.mid(3) : units));
      } else if (line.startsWith("POST ") && line.contains("/cell-camera/video/keyframe")) {
        ++keyAsked;
        socket->write("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 14\r\nConnection: close\r\n\r\n{\"asked\":true}");
        socket->disconnectFromHost();
      } else if (line.contains("frame.jpg?after=")) {
        ++jpegAsked;
        socket->write("HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nX-Frame-Sequence: " + QByteArray::number(jpegAsked) +
                      "\r\nContent-Length: " + QByteArray::number(jpeg.size()) + "\r\nConnection: close\r\n\r\n" + jpeg);
        socket->disconnectFromHost();
      }
    });
    QObject::connect(socket, &QTcpSocket::disconnected, socket, &QObject::deleteLater);
  });
  const QString origin = QString("http://127.0.0.1:%1").arg(server.serverPort());

  // The relayed stream: every unit drawn, with this app's identity, no JPEG.
  {
    CameraView view(origin);
    view.resize(640, 400);
    view.show();
    CHECK(spin(5000, [&] { return view.live(); }));
    CHECK(view.video() && view.frameSize() == QSize(W, H) && jpegAsked == 0 && keyAsked == 0);
    CHECK(spin(2000, [&] { return view.sequence() == quint64(units.size()); }));
    CHECK(streamClient.startsWith("rosie-pendant-v5 ("));
    // Looking away closes the stream: the relay stops reading the camera.
    view.hide();
    CHECK(spin(2000, [&] { return openStreams == 0; }));
    CHECK(view.frameSize().isEmpty() && jpegAsked == 0);
  }

  // Joining after a key frame: nothing drawn until one comes, and the view
  // asks for one, at most once a second.
  video = Video::Broken;
  {
    CameraView view(origin);
    view.resize(640, 400);
    view.show();
    CHECK(spin(3000, [&] { return keyAsked >= 1; }));
    CHECK(keyAsked == 1 && view.frameSize().isEmpty() && view.video());
    view.hide();
  }

  // A cell camera without video: 404, JPEG frames instead.
  video = Video::NoVideo;
  {
    CameraView view(origin);
    view.resize(640, 400);
    view.show();
    CHECK(spin(5000, [&] { return view.live() && !view.video(); }));
    CHECK(jpegAsked > 0);
    view.hide();
  }

  // Video that never starts: JPEG after videoTries tries.
  jpegAsked = 0;
  streams = 0;
  video = Video::Fail;
  {
    CameraView view(origin);
    view.resize(640, 400);
    view.show();
    CHECK(spin(300, [&] { return view.video() && jpegAsked == 0; }));
    CHECK(spin((CameraView::videoTries + 1) * CameraView::retryMs + 2000, [&] { return view.live() && !view.video(); }));
    CHECK(streams == CameraView::videoTries);
    view.hide();
  }

  // ROSIE_PENDANT_CAMERA_VIDEO=off: JPEG frames, no stream asked for.
  streams = 0;
  video = Video::Stream;
  qputenv("ROSIE_PENDANT_CAMERA_VIDEO", "off");
  {
    CameraView view(origin);
    view.resize(640, 400);
    view.show();
    CHECK(spin(3000, [&] { return view.live(); }));
    CHECK(!view.video() && streams == 0);
  }
  std::puts("PASS camera video: OpenH264 round trip, broken unit reported, relayed stream drawn with identity and closed when "
            "hidden, key frame asked for, JPEG when the cell has no video, when video never starts, and when switched off");
  return 0;
}
