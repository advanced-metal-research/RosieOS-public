// The pendant's capture TCP against OLP's own (urdf-loader) for the same URDF
// and joints. usage: capture-check <capture-fk.json> <robot_description/robots>
#include "capture_fk.hpp"
#include <QCoreApplication>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <cmath>
#include <cstdio>

int main(int argc, char **argv) {
  QCoreApplication app(argc, argv);
  if (argc != 3) return 2;
  QFile fixture(argv[1]);
  if (!fixture.open(QIODevice::ReadOnly)) return 2;
  const auto document = QJsonDocument::fromJson(fixture.readAll()).object();
  double worst = 0;
  int count = 0;
  for (const auto &entry : document.value("cases").toArray()) {
    const auto c = entry.toObject();
    QFile urdf(QString("%1/%2/robot.urdf").arg(argv[2], c.value("model").toString()));
    if (!urdf.open(QIODevice::ReadOnly)) return 2;
    CaptureChain chain;
    QString error;
    if (!chain.parse(QString::fromUtf8(urdf.readAll()), document.value("frame").toString(), error)) {
      std::fprintf(stderr, "parse: %s\n", qPrintable(error));
      return 1;
    }
    QHash<QString, double> joints;
    const auto values = c.value("joints").toObject();
    for (auto it = values.begin(); it != values.end(); ++it) joints[it.key()] = it.value().toDouble();
    const auto pose = chain.pose(joints, error);
    if (!pose) { std::fprintf(stderr, "pose: %s\n", qPrintable(error)); return 1; }
    const auto expected = c.value("pose").toArray();
    for (int i = 0; i < 6; ++i) {
      double difference = std::abs((*pose)[i] - expected[i].toDouble());
      if (i >= 3) difference = std::abs(std::remainder(difference, 2 * M_PI));
      worst = std::max(worst, difference);
    }
    ++count;
  }
  // Unverified tolerance: 1e-9 m / rad, far below any capture's meaning.
  std::printf("%s: %d captures, worst difference %.3g against OLP's urdf-loader\n", worst < 1e-9 ? "PASS" : "FAIL", count, worst);
  return worst < 1e-9 && count > 0 ? 0 : 1;
}
