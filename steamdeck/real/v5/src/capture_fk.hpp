#pragma once
#include <QHash>
#include <QString>
#include <array>
#include <optional>
#include <vector>

// The TCP a recorded waypoint stores, computed the way OLP computes it: forward
// kinematics of the capture model's URDF (POST /robots/capture-model, the
// calibration already compiled in) in double precision, joint limits ignored,
// the frame's world pose as xyz metres and three.js Euler "ZYX" roll/pitch/yaw
// (workstation-renderer.ts tcpPoseFor). A missing joint value is refused, never
// read as zero.
class CaptureChain {
public:
  bool parse(const QString &urdf, const QString &frame, QString &error);
  std::optional<std::array<double, 6>> pose(const QHash<QString, double> &joints, QString &error) const;
  const std::vector<QString> &movingJoints() const { return moving; }

private:
  struct Joint {
    QString name, type;
    std::array<double, 3> xyz{}, rpy{}, axis{1, 0, 0};
  };
  std::vector<Joint> chain; // root link to frame
  std::vector<QString> moving;
};
