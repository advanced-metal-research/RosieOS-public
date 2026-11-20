#include "camera_view.hpp"
#include "olp_client.hpp"
#include "theme.hpp"
#include <QBuffer>
#include <QImageReader>
#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPainter>

namespace {
const char *const noCamera = "CAMERA\n\nNo camera stream is configured for this cell.\n"
                             "A cell camera streams here once the cell configuration names one.";
}

CameraView::CameraView(QString olpOrigin, QWidget *parent, QString cameraRole) : QWidget(parent), origin(std::move(olpOrigin)), message(noCamera), role(std::move(cameraRole)) {
  setMinimumSize(160, 100);
  if (const QString path = qEnvironmentVariable("ROSIE_PENDANT_CAMERA_TRACE"); !path.isEmpty()) {
    trace.setFileName(path);
    if (trace.open(QIODevice::WriteOnly | QIODevice::Append)) traceClock.start();
  }
  next.setSingleShot(true);
  connect(&next, &QTimer::timeout, this, [this] { request(); });
  restartVideo.setSingleShot(true);
  connect(&restartVideo, &QTimer::timeout, this, [this] { startVideo(); });
  // The picture's age is this view's to show even when no reply arrives.
  watch.setInterval(250);
  connect(&watch, &QTimer::timeout, this, [this] {
    if (!frame.isNull() && sinceFrame.elapsed() >= goneMs) {
      frame = {};
      say("Camera picture lost; reconnecting");
    }
    // A stream that stopped delivering is restarted, whatever TCP believes.
    if (mode == Mode::Video && stream && sinceUnit.isValid() && sinceUnit.elapsed() >= videoStallMs) {
      sinceUnit.invalidate();
      stream->abort();
    }
    update();
  });
}

CameraView::~CameraView() { stopVideo(); }

void CameraView::showEvent(QShowEvent *) {
  // Start from the newest frame, never from where an earlier look stopped.
  lastSequence = 0;
  frame = {};
  fps = 0;
  framesInWindow = 0;
  rateWindow.start();
  videoFailures = 0;
  videoShown = false;
  jpegReason.clear();
  say("Connecting to the cell camera");
  watch.start();
  const QString wanted = qEnvironmentVariable("ROSIE_PENDANT_CAMERA_VIDEO").trimmed().toLower();
  if (role != "cell") {
    mode = Mode::Jpeg; // only the cell camera has video
  } else if (wanted == "off" || wanted == "0" || wanted == "jpeg") {
    mode = Mode::Jpeg;
    jpegReason = "video off";
  } else {
    mode = Mode::Video;
    return startVideo();
  }
  request();
}

void CameraView::hideEvent(QHideEvent *) {
  // Nobody is looking: stop asking, so the camera service stops capturing
  // and encoding.
  next.stop();
  watch.stop();
  restartVideo.stop();
  stopVideo();
  unpainted = false;
  sincePaint.invalidate();
  if (pending) pending->abort();
  frame = {};
}

void CameraView::startVideo() {
  if (!isVisible() || mode != Mode::Video || stream) return;
  decoder = std::make_unique<H264Decoder>(); // a new stream starts at a key frame
  if (!decoder->ok()) return fallBackToJpeg("no H.264 decoder");
  QNetworkRequest get(QUrl(origin + "/api/offline-programming/v1/cell-camera/video.h264"));
  get.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::ManualRedirectPolicy);
  get.setAttribute(QNetworkRequest::CacheLoadControlAttribute, QNetworkRequest::AlwaysNetwork);
  OlpClient::identify(get);
  videoBuffer.clear();
  sinceUnit.start();
  QNetworkReply *reply = network.get(get);
  stream = reply;
  connect(reply, &QNetworkReply::readyRead, this, [this, reply] {
    if (reply == stream) videoData();
  });
  connect(reply, &QNetworkReply::finished, this, [this, reply] {
    reply->deleteLater();
    if (reply == stream) videoEnded(reply);
  });
}

void CameraView::stopVideo() {
  QNetworkReply *reply = stream;
  stream = nullptr;
  if (reply) reply->abort(); // the relay stops reading the camera; its encoder idles
}

void CameraView::askKeyframe() {
  if (sinceKeyRequest.isValid() && sinceKeyRequest.elapsed() < keyRequestMs) return;
  sinceKeyRequest.start();
  QNetworkRequest post(QUrl(origin + "/api/offline-programming/v1/cell-camera/video/keyframe"));
  post.setHeader(QNetworkRequest::ContentTypeHeader, "text/plain");
  OlpClient::identify(post);
  QNetworkReply *reply = network.post(post, QByteArray());
  connect(reply, &QNetworkReply::finished, reply, &QObject::deleteLater);
}

void CameraView::videoData() {
  // An error's body (404 without video, 503 while the camera starts) is not
  // the stream: videoEnded reads the status when the reply finishes.
  if (stream->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt() != 200) return;
  videoBuffer += stream->readAll();
  while (videoBuffer.size() >= 4) {
    const auto *head = reinterpret_cast<const unsigned char *>(videoBuffer.constData());
    const quint32 n = quint32(head[0]) << 24 | quint32(head[1]) << 16 | quint32(head[2]) << 8 | head[3];
    if (n == 0 || n > quint32(maxUnitBytes)) {
      qWarning("camera: the video stream sent a %u-byte unit; reconnecting", n);
      videoBuffer.clear();
      stream->abort();
      return;
    }
    if (videoBuffer.size() < int(4 + n)) return;
    const QByteArray unit = videoBuffer.mid(4, int(n));
    videoBuffer.remove(0, int(4 + n));
    sinceUnit.start();
    H264Decoder::Result r = decoder->decode(unit);
    if (r.broken) askKeyframe(); // keep the last good picture until the key frame
    if (r.picture.isNull()) continue;
    if (trace.isOpen()) {
      traced = {lastSequence + 1, unit.size(), -1, 0, r.decodeMs, {}, {}};
      unpainted = true;
      sinceDecoded.start();
    }
    ++lastSequence;
    videoShown = true;
    videoFailures = 0;
    const QSize source = r.picture.size();
    shown(std::move(r.picture), source);
  }
}

void CameraView::videoEnded(QNetworkReply *reply) {
  stream = nullptr;
  if (!isVisible() || mode != Mode::Video) return;
  const int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
  // 404: this cell's camera has no video, or this OLP server has no relay.
  if (status == 404) return fallBackToJpeg("no video at this cell");
  if (!videoShown && ++videoFailures >= videoTries) return fallBackToJpeg("video did not start");
  if (frame.isNull()) say("Camera video reconnecting");
  restartVideo.start(retryMs);
}

void CameraView::fallBackToJpeg(const QString &why) {
  qInfo("camera: showing JPEG frames: %s", qUtf8Printable(why));
  stopVideo();
  mode = Mode::Jpeg;
  jpegReason = why;
  request();
}

void CameraView::say(const QString &text) {
  if (message == text) return;
  message = text;
  update();
}

void CameraView::later(int ms) {
  if (isVisible()) next.start(ms);
}

void CameraView::request() {
  if (pending || !isVisible() || mode != Mode::Jpeg) return;
  // The cell camera keeps its own route; any other camera is one of the
  // cell's cameras by its instance (offline-programming/v1 cell_camera.go).
  const QString path = role == "cell" ? QString("/api/offline-programming/v1/cell-camera/frame.jpg?after=")
                                      : "/api/offline-programming/v1/cell-cameras/" + role + "/frame.jpg?after=";
  QNetworkRequest get(QUrl(origin + path + QString::number(lastSequence)));
  get.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::ManualRedirectPolicy);
  get.setAttribute(QNetworkRequest::CacheLoadControlAttribute, QNetworkRequest::AlwaysNetwork);
  get.setTransferTimeout(requestTimeoutMs);
  OlpClient::identify(get);
  sinceRequest.start();
  headersMs = -1;
  pending = network.get(get);
  if (trace.isOpen())
    connect(pending, &QNetworkReply::metaDataChanged, this, [this, reply = pending.data()] {
      if (reply == pending && headersMs < 0) headersMs = sinceRequest.elapsed();
    });
  connect(pending, &QNetworkReply::finished, this, [this, reply = pending.data()] { received(reply); });
}

void CameraView::received(QNetworkReply *reply) {
  reply->deleteLater();
  if (reply != pending) return;
  pending = nullptr;
  if (!isVisible()) return;
  const qint64 replyMs = sinceRequest.elapsed();
  const int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
  const QByteArray body = reply->readAll();
  if (status == 404) {
    // This OLP server names no camera, or predates the camera relay.
    frame = {};
    say(noCamera);
    return later(unconfiguredRetryMs);
  }
  if (status != 200 || !reply->header(QNetworkRequest::ContentTypeHeader).toString().startsWith("image/jpeg")) {
    const auto detail = QJsonDocument::fromJson(body).object()["detail"].toString();
    say(status == 0 ? "Camera unreachable through the OLP server; retrying"
                    : "Camera unavailable; retrying" + (detail.isEmpty() ? QString() : "\n" + detail));
    return later(retryMs);
  }
  QBuffer buffer;
  buffer.setData(body);
  buffer.open(QIODevice::ReadOnly);
  QImageReader reader(&buffer, "jpeg");
  // Decode straight to the pane: the JPEG decoder scales as it decodes, which
  // is both sharper and cheaper than decoding 1920 x 1200 and shrinking it.
  const QSize source = reader.size();
  const QSize pane = size() * devicePixelRatioF();
  if (source.isValid() && !pane.isEmpty() && (source.width() > pane.width() || source.height() > pane.height()))
    reader.setScaledSize(source.scaled(pane, Qt::KeepAspectRatio));
  QElapsedTimer decoding;
  decoding.start();
  QImage image = reader.read();
  const double decodeMs = decoding.nsecsElapsed() / 1e6;
  if (image.isNull()) {
    say("Camera frame could not be decoded; retrying");
    return later(retryMs);
  }
  bool numbered = false;
  const quint64 sequence = reply->rawHeader("X-Frame-Sequence").toULongLong(&numbered);
  lastSequence = numbered ? sequence : lastSequence + 1;
  if (trace.isOpen()) {
    traced = {lastSequence, body.size(), headersMs, replyMs, decodeMs, reply->rawHeader("X-Source-Age-Ms"),
              reply->rawHeader("Server-Timing")};
    unpainted = true;
    sinceDecoded.start();
  }
  shown(std::move(image), source);
  later(std::max(0, minIntervalMs - int(sinceRequest.elapsed())));
}

void CameraView::shown(QImage image, QSize source) {
  frame = std::move(image);
  sourceSize = source;
  sinceFrame.start();
  if (++framesInWindow, rateWindow.elapsed() >= 1000) {
    fps = framesInWindow * 1000. / rateWindow.elapsed();
    framesInWindow = 0;
    rateWindow.start();
  }
  message.clear();
  update();
}

void CameraView::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.fillRect(rect(), QColor("#22262a"));
  if (frame.isNull()) {
    p.setPen(QColor("#cfd2d5"));
    p.drawText(rect().adjusted(24, 24, -24, -24), Qt::AlignCenter | Qt::TextWordWrap, message);
    return;
  }
  const QSize fit = frame.size().scaled(size(), Qt::KeepAspectRatio);
  const QRect target(QPoint((width() - fit.width()) / 2, (height() - fit.height()) / 2), fit);
  p.setRenderHint(QPainter::SmoothPixmapTransform);
  p.drawImage(target, frame);
  if (unpainted) {
    unpainted = false;
    QJsonObject line{{"t_ms", double(traceClock.elapsed())}, {"sequence", double(traced.sequence)}, {"bytes", double(traced.bytes)},
                     {"mode", mode == Mode::Video ? "video" : "jpeg"},
                     {"headers_ms", double(traced.headersMs)}, {"reply_ms", double(traced.replyMs)}, {"decode_ms", traced.decodeMs},
                     {"paint_ms", sinceDecoded.nsecsElapsed() / 1e6}, {"server_timing", QString::fromLatin1(traced.serverTiming)},
                     {"frame", QString("%1x%2").arg(sourceSize.width()).arg(sourceSize.height())},
                     {"drawn", QString("%1x%2").arg(frame.width()).arg(frame.height())}};
    bool aged = false;
    if (const double age = traced.sourceAge.toDouble(&aged); aged) line["source_age_ms"] = age;
    // How long the frame before this one stayed on screen.
    if (sincePaint.isValid()) line["since_last_paint_ms"] = sincePaint.nsecsElapsed() / 1e6;
    sincePaint.start();
    trace.write(QJsonDocument(line).toJson(QJsonDocument::Compact) + '\n');
    trace.flush();
  }
  const qint64 age = sinceFrame.elapsed();
  const bool late = age >= lateMs;
  const QString caption = late ? QString("NO NEW FRAME FOR %1 s").arg(age / 1000., 0, 'f', 1)
                               : QString("LIVE · %1 · %2 fps · %3 × %4")
                                     .arg(mode == Mode::Video ? QString("VIDEO") : "JPEG" + (jpegReason.isEmpty() ? QString() : " (" + jpegReason + ")"))
                                     .arg(fps, 0, 'f', 0)
                                     .arg(sourceSize.width())
                                     .arg(sourceSize.height());
  QFont small = font();
  small.setPointSizeF(std::max(8., small.pointSizeF() * .85));
  small.setBold(late);
  p.setFont(small);
  const QRect box = p.fontMetrics().boundingRect(caption).adjusted(-8, -4, 8, 4).translated(target.left() + 16, target.bottom() - 12);
  p.fillRect(box, late ? QColor(theme::cautionSoft) : QColor(0, 0, 0, 150));
  p.setPen(late ? QColor(theme::caution) : QColor("#f2f3f4"));
  p.drawText(box, Qt::AlignCenter, caption);
  if (late) {
    p.setPen(QPen(QColor(theme::caution), 3));
    p.drawRect(target.adjusted(1, 1, -2, -2));
  }
}
