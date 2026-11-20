#pragma once
#include <QJsonObject>
#include <QMap>
#include <QMatrix4x4>
#include <QString>
#include <QVector3D>
#include <vector>

// Display kinematics for the 3D view: every link frame of a robot description,
// with its machine_planning_calibration.json joint-origin corrections applied.
// Float precision is ample for drawing; recorded TCPs use CaptureChain.
struct Kinematics {
  struct Joint {
    QString name, parent, child, type;
    QVector3D xyz, rpy, axis;
  };
  std::vector<Joint> chain, all;
  QString model, urdfSha256, calibrationSha256;
  QJsonObject corrections;
  bool load(const QString &path, QString &error);
  bool loadCalibration(const QString &path, QString &error);
  bool covers(const QJsonObject &joints) const;
  QMatrix4x4 transform(const Joint &joint, const QJsonObject &values) const;
  QMap<QString, QMatrix4x4> frames(const QJsonObject &values) const;
  std::vector<QVector3D> points(const QJsonObject &joints, QMatrix4x4 *tcp = nullptr) const;
};
