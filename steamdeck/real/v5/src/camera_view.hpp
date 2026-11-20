#pragma once
#include "h264_decoder.hpp"
#include <QElapsedTimer>
#include <QFile>
#include <QImage>
#include <QNetworkAccessManager>
#include <QPointer>
#include <QTimer>
#include <QWidget>
#include <memory>

class QNetworkReply;

// The cell camera, as the OLP server relays it (cell_camera.go), asked for
// only while this view shows. It is a look at the cell, not a safety
// function: a frame that stops being replaced is marked late and then
// removed, so a frozen picture never passes for the cell as it is now.
//
// Two ways to get the picture. Video: when the cell camera has video, its
// low-data H.264 stream as the OLP server relays it (cell_video_stream.go),
// about 0.36 GB an hour instead of 12.7 GB for JPEG on a Rosie 1430 cell (2026-10-16),
// decoded here (H264Decoder). It comes through the same tunnel as the JPEG
// frames: the Deck's userspace Tailscale cannot carry the browser's WebRTC
// (a Steam Deck, 2026-10-19). JPEG: the newest frame after the last one
// drawn, one request at a time; for the cell's other cameras, a cell without
// video, ROSIE_PENDANT_CAMERA_VIDEO=off, or video that never started.
class CameraView : public QWidget {
public:
  // Unverified bounds, chosen for a pendant on Wi-Fi through an SSH tunnel:
  // at most 15 requests a second; the next frame is asked for only after the
  // last one was drawn, so a slow link lowers the rate and not the freshness.
  static constexpr int minIntervalMs = 66;
  // The camera service waits up to 3 s for a frame and the relay 5 s.
  static constexpr int requestTimeoutMs = 6000;
  static constexpr int retryMs = 1000;        // camera starting or gone
  static constexpr int unconfiguredRetryMs = 5000;
  static constexpr int lateMs = 1000;         // mark the picture late
  static constexpr int goneMs = 3000;         // remove the picture
  // Unverified bounds. A video that never showed a picture after this many
  // tries falls back to JPEG for the rest of this look. One that did show
  // pictures keeps retrying video: JPEG costs 35 times the data.
  static constexpr int videoTries = 3;
  static constexpr int videoStallMs = 6000;   // no unit this long: reconnect
  static constexpr int keyRequestMs = 1000;   // key frame requests, at most
  static constexpr int maxUnitBytes = 4 << 20;

  // `role` is "cell" for the cell camera, else a camera instance of the cell
  // (cell_deploy.extra_cameras), for example "wrist" on one cell.
  explicit CameraView(QString origin, QWidget *parent = nullptr, QString role = "cell");
  ~CameraView() override;
  // The picture on screen is newer than lateMs.
  bool live() const { return !frame.isNull() && sinceFrame.isValid() && sinceFrame.elapsed() < lateMs; }
  quint64 sequence() const { return lastSequence; }
  QSize frameSize() const { return frame.size(); }
  QString status() const { return message; }
  // This look gets the picture as video (else JPEG frames).
  bool video() const { return mode == Mode::Video; }

protected:
  void showEvent(QShowEvent *) override;
  void hideEvent(QHideEvent *) override;
  void paintEvent(QPaintEvent *) override;

private:
  enum class Mode { Jpeg, Video };
  void request();
  void received(QNetworkReply *reply);
  void later(int ms);
  void say(const QString &text);
  void shown(QImage image, QSize source);
  void startVideo();
  void stopVideo();
  void askKeyframe();
  void videoData();
  void videoEnded(QNetworkReply *reply);
  void fallBackToJpeg(const QString &why);

  Mode mode = Mode::Jpeg;
  QPointer<QNetworkReply> stream;
  QElapsedTimer sinceKeyRequest;
  QByteArray videoBuffer;
  std::unique_ptr<H264Decoder> decoder;
  QTimer restartVideo;
  QElapsedTimer sinceUnit;
  int videoFailures = 0;
  bool videoShown = false; // this look's video drew a picture
  QString jpegReason;      // why this look shows JPEG frames, for the caption

  QNetworkAccessManager network;
  QString origin, message, role;
  QPointer<QNetworkReply> pending;
  QTimer next, watch;
  QElapsedTimer sinceRequest, sinceFrame, rateWindow;
  QImage frame;
  QSize sourceSize;
  quint64 lastSequence = 0;
  int framesInWindow = 0;
  double fps = 0;

  // Latency trace, written only when ROSIE_PENDANT_CAMERA_TRACE names a file:
  // one JSON line per frame painted, with this view's own timings beside the
  // age and stage timings the servers sent (cell-camera/v1/tools/camera_latency.py
  // reads it). "Painted" is this widget's paint, not the panel's refresh.
  struct Traced {
    quint64 sequence = 0;
    qint64 bytes = 0, headersMs = -1, replyMs = 0;
    double decodeMs = 0;
    QByteArray sourceAge, serverTiming;
  };
  QFile trace;
  QElapsedTimer traceClock, sinceDecoded, sincePaint;
  Traced traced;
  qint64 headersMs = -1;
  bool unpainted = false;
};
