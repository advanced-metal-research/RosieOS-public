#include "capture_fk.hpp"
#include <QRegularExpression>
#include <QSet>
#include <QXmlStreamReader>
#include <algorithm>
#include <cmath>

namespace {
using Matrix = std::array<std::array<double, 4>, 4>;

Matrix identity() {
  Matrix m{};
  for (int i = 0; i < 4; ++i) m[i][i] = 1;
  return m;
}

Matrix multiply(const Matrix &a, const Matrix &b) {
  Matrix out{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      for (int k = 0; k < 4; ++k) out[r][c] += a[r][k] * b[k][c];
  return out;
}

// Rotation about a unit axis (Rodrigues), as three.js makeRotationAxis.
Matrix rotation(std::array<double, 3> axis, double angle) {
  const double c = std::cos(angle), s = std::sin(angle), t = 1 - c;
  const auto [x, y, z] = axis;
  Matrix m = identity();
  m[0][0] = t * x * x + c;     m[0][1] = t * x * y - s * z; m[0][2] = t * x * z + s * y;
  m[1][0] = t * x * y + s * z; m[1][1] = t * y * y + c;     m[1][2] = t * y * z - s * x;
  m[2][0] = t * x * z - s * y; m[2][1] = t * y * z + s * x; m[2][2] = t * z * z + c;
  return m;
}

// URDF origin: translate, then fixed-axis roll/pitch/yaw, i.e. Rz(yaw)·Ry(pitch)·Rx(roll).
Matrix origin(const std::array<double, 3> &xyz, const std::array<double, 3> &rpy) {
  Matrix m = multiply(rotation({0, 0, 1}, rpy[2]), multiply(rotation({0, 1, 0}, rpy[1]), rotation({1, 0, 0}, rpy[0])));
  for (int i = 0; i < 3; ++i) m[i][3] = xyz[i];
  return m;
}

std::optional<std::array<double, 3>> triple(const QString &text, std::array<double, 3> fallback) {
  if (text.trimmed().isEmpty()) return fallback;
  const auto parts = text.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
  if (parts.size() != 3) return std::nullopt;
  std::array<double, 3> out{};
  for (int i = 0; i < 3; ++i) {
    bool ok = false;
    out[i] = parts[i].toDouble(&ok);
    if (!ok || !std::isfinite(out[i])) return std::nullopt;
  }
  return out;
}
} // namespace

bool CaptureChain::parse(const QString &urdf, const QString &frame, QString &error) {
  struct Parsed { Joint joint; QString parent, child; };
  std::vector<Parsed> joints;
  QXmlStreamReader xml(urdf);
  while (!xml.atEnd()) {
    xml.readNext();
    if (!xml.isStartElement() || xml.name() != QLatin1String("joint") || !xml.attributes().hasAttribute("type")) continue;
    Parsed parsed;
    parsed.joint.name = xml.attributes().value("name").toString();
    parsed.joint.type = xml.attributes().value("type").toString();
    while (xml.readNextStartElement()) {
      const auto a = xml.attributes();
      if (xml.name() == QLatin1String("parent")) parsed.parent = a.value("link").toString();
      else if (xml.name() == QLatin1String("child")) parsed.child = a.value("link").toString();
      else if (xml.name() == QLatin1String("origin")) {
        const auto xyz = triple(a.value("xyz").toString(), {0, 0, 0}), rpy = triple(a.value("rpy").toString(), {0, 0, 0});
        if (!xyz || !rpy) { error = "Invalid origin on joint " + parsed.joint.name; return false; }
        parsed.joint.xyz = *xyz;
        parsed.joint.rpy = *rpy;
      } else if (xml.name() == QLatin1String("axis")) {
        const auto axis = triple(a.value("xyz").toString(), {1, 0, 0});
        const double norm = axis ? std::sqrt((*axis)[0] * (*axis)[0] + (*axis)[1] * (*axis)[1] + (*axis)[2] * (*axis)[2]) : 0;
        if (!axis || !(norm > 0)) { error = "Invalid axis on joint " + parsed.joint.name; return false; }
        parsed.joint.axis = {(*axis)[0] / norm, (*axis)[1] / norm, (*axis)[2] / norm};
      } else if (xml.name() == QLatin1String("mimic")) {
        error = "Mimic joint " + parsed.joint.name + " is not supported for capture";
        return false;
      }
      xml.skipCurrentElement();
    }
    joints.push_back(parsed);
  }
  if (xml.hasError()) { error = xml.errorString(); return false; }
  std::vector<Joint> path;
  QSet<QString> seen;
  for (QString link = frame;;) {
    if (seen.contains(link)) { error = "URDF contains a cycle"; return false; }
    seen.insert(link);
    const auto parent = std::find_if(joints.begin(), joints.end(), [&](const Parsed &p) { return p.child == link; });
    if (parent == joints.end()) break; // the root link, at the world origin
    path.insert(path.begin(), parent->joint);
    link = parent->parent;
  }
  if (path.empty()) { error = "The robot description has no TCP frame " + frame; return false; }
  for (const auto &joint : path)
    if (joint.type != "fixed" && joint.type != "revolute" && joint.type != "continuous" && joint.type != "prismatic") {
      error = "Joint " + joint.name + " of type " + joint.type + " is not supported for capture";
      return false;
    }
  chain = path;
  moving.clear();
  for (const auto &joint : chain)
    if (joint.type != "fixed") moving.push_back(joint.name);
  return true;
}

std::optional<std::array<double, 6>> CaptureChain::pose(const QHash<QString, double> &joints, QString &error) const {
  if (chain.empty()) { error = "Capture model is not loaded"; return std::nullopt; }
  Matrix world = identity();
  for (const auto &joint : chain) {
    world = multiply(world, origin(joint.xyz, joint.rpy));
    if (joint.type == "fixed") continue;
    const auto value = joints.find(joint.name);
    if (value == joints.end() || !std::isfinite(*value)) { error = "Capture is missing " + joint.name; return std::nullopt; }
    if (joint.type == "prismatic") {
      Matrix slide = identity();
      for (int i = 0; i < 3; ++i) slide[i][3] = joint.axis[i] * *value;
      world = multiply(world, slide);
    } else {
      world = multiply(world, rotation(joint.axis, *value));
    }
  }
  // three.js Euler.setFromRotationMatrix(m, "ZYX"), branch for branch.
  const double m31 = world[2][0];
  const double pitch = std::asin(-std::clamp(m31, -1.0, 1.0));
  double roll = 0, yaw = 0;
  if (std::abs(m31) < 0.9999999) {
    roll = std::atan2(world[2][1], world[2][2]);
    yaw = std::atan2(world[1][0], world[0][0]);
  } else {
    yaw = std::atan2(-world[0][1], world[1][1]);
  }
  return std::array<double, 6>{world[0][3], world[1][3], world[2][3], roll, pitch, yaw};
}
