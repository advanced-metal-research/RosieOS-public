#include "kinematics.hpp"
#include <QCryptographicHash>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QRegularExpression>
#include <QSet>
#include <QXmlStreamReader>
#include <cmath>

static QVector3D vec(QString s, QVector3D fallback = {}) {
  auto p = s.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
  if (p.size() != 3)
    return fallback;
  return {p[0].toFloat(), p[1].toFloat(), p[2].toFloat()};
}
bool Kinematics::load(const QString &path, QString &error) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    error = file.errorString();
    return false;
  }
  const auto bytes = file.readAll();
  QXmlStreamReader xml(bytes);
  std::vector<Joint> all;
  QString robot;
  while (!xml.atEnd()) {
    xml.readNext();
    if (!xml.isStartElement())
      continue;
    if (xml.name() == "robot")
      robot = xml.attributes().value("name").toString();
    if (xml.name() != "joint" || !xml.attributes().hasAttribute("type"))
      continue;
    Joint j;
    j.name = xml.attributes().value("name").toString();
    j.type = xml.attributes().value("type").toString();
    j.axis = {1, 0, 0};
    while (xml.readNextStartElement()) {
      auto a = xml.attributes();
      if (xml.name() == "parent")
        j.parent = a.value("link").toString();
      if (xml.name() == "child")
        j.child = a.value("link").toString();
      if (xml.name() == "origin") {
        j.xyz = vec(a.value("xyz").toString());
        j.rpy = vec(a.value("rpy").toString());
      }
      if (xml.name() == "axis")
        j.axis = vec(a.value("xyz").toString(), {1, 0, 0});
      xml.skipCurrentElement();
    }
    all.push_back(j);
  }
  std::vector<Joint> next;
  QString child = "tool0";
  QSet<QString> seen;
  while (child != "world") {
    if (seen.contains(child)) {
      error = "URDF contains a cycle";
      return false;
    }
    seen.insert(child);
    auto i = std::find_if(all.begin(), all.end(),
                          [&](const Joint &j) { return j.child == child; });
    if (i == all.end()) {
      error = "URDF needs a world-to-tool0 chain";
      return false;
    }
    next.insert(next.begin(), *i);
    child = i->parent;
  }
  if (xml.hasError()) {
    error = xml.errorString();
    return false;
  }
  chain = next;
  this->all = all;
  model = robot;
  urdfSha256 =
      "sha256:" +
      QString::fromLatin1(
          QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex());
  return true;
}
bool Kinematics::loadCalibration(const QString &path, QString &error) {
  QByteArray bytes;
  calibrationSha256.clear();
  corrections = {};
  if (path.isEmpty()) {
    bytes = "{\"schema\":\"rosie.machine-planning-calibration.v1\",\"model_"
            "id\":\"" +
            model.toUtf8() + "\"}\n";
  } else {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
      error = f.errorString();
      return false;
    }
    bytes = f.readAll();
  }
  const auto doc = QJsonDocument::fromJson(bytes).object();
  if (doc["schema"] != "rosie.machine-planning-calibration.v1" ||
      doc["model_id"] != model) {
    error = "Calibration does not match robot description";
    return false;
  }
  corrections = doc["kinematics"].toObject();
  for (auto it = corrections.begin(); it != corrections.end(); ++it) {
    if (!it.value().isObject()) {
      error = "Invalid calibration correction";
      return false;
    }
    const auto correction = it.value().toObject();
    for (auto key : {"xyz_m", "rpy_rad"}) {
      const auto value = correction[key];
      if (value.isUndefined())
        continue;
      auto a = value.toArray();
      if (!value.isArray() || a.size() != 3) {
        error = "Invalid calibration transform";
        return false;
      }
      for (auto v : a)
        if (!v.isDouble() || !std::isfinite(v.toDouble())) {
          error = "Invalid calibration number";
          return false;
        }
    }
  }
  calibrationSha256 =
      "sha256:" +
      QString::fromLatin1(
          QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex());
  return true;
}
bool Kinematics::covers(const QJsonObject &joints) const {
  for (const auto &j : chain)
    if (j.type != "fixed" && (!joints[j.name].isDouble() ||
                              !std::isfinite(joints[j.name].toDouble())))
      return false;
  return !chain.empty();
}
std::vector<QVector3D> Kinematics::points(const QJsonObject &joints,
                                          QMatrix4x4 *tcp) const {
  QMatrix4x4 t;
  std::vector<QVector3D> p{{0, 0, 0}};
  for (const auto &j : chain) {
    t *= transform(j, joints);
    p.push_back(t.map(QVector3D{}));
  }
  if (tcp)
    *tcp = t;
  return p;
}
QMatrix4x4 Kinematics::transform(const Joint &j,
                                 const QJsonObject &joints) const {
  constexpr double deg = 180. / 3.141592653589793;
  QMatrix4x4 t;
  t.translate(j.xyz);
  t.rotate(j.rpy.z() * deg, 0, 0, 1);
  t.rotate(j.rpy.y() * deg, 0, 1, 0);
  t.rotate(j.rpy.x() * deg, 1, 0, 0);
  auto correction = corrections[j.name].toObject();
  auto xyz = correction["xyz_m"].toArray(),
       rpy = correction["rpy_rad"].toArray();
  if (xyz.size() == 3)
    t.translate(xyz[0].toDouble(), xyz[1].toDouble(), xyz[2].toDouble());
  if (rpy.size() == 3) {
    t.rotate(rpy[2].toDouble() * deg, 0, 0, 1);
    t.rotate(rpy[1].toDouble() * deg, 0, 1, 0);
    t.rotate(rpy[0].toDouble() * deg, 1, 0, 0);
  }
  if (j.type == "revolute" || j.type == "continuous")
    t.rotate(joints[j.name].toDouble() * deg, j.axis);
  if (j.type == "prismatic")
    t.translate(j.axis * joints[j.name].toDouble());
  return t;
}
QMap<QString, QMatrix4x4> Kinematics::frames(const QJsonObject &values) const {
  QMap<QString, QMatrix4x4> result{{"world", QMatrix4x4{}}};
  for (size_t pass = 0; pass < all.size(); ++pass) {
    bool changed = false;
    for (const auto &j : all)
      if (!result.contains(j.child) && result.contains(j.parent)) {
        result[j.child] = result[j.parent] * transform(j, values);
        changed = true;
      }
    if (!changed)
      break;
  }
  return result;
}
