#pragma once
#include "kinematics.hpp"
#include "diagnostics.hpp"
#include "orbit_gesture.hpp"
#include <cmath>
#include <QColor>
#include <QElapsedTimer>
#include <QOpenGLBuffer>
#include <QOpenGLFramebufferObject>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>
#include <memory>

struct RobotVertex {
  QVector3D position, normal;
};
struct RobotMesh {
  QString link;
  QMatrix4x4 origin;
  QColor color;
  int first = 0, count = 0;
  int edgeFirst = 0, edgeCount = 0, detailFirst = 0, detailCount = 0;
  bool pedestal = false;
};
struct RobotAssets {
  Kinematics model;
  QString identity, error, directory, visualSource;
  std::vector<RobotVertex> vertices;
  std::vector<RobotMesh> meshes;
  std::vector<RobotVertex> edges;
};
// Derive display-only CAD edges; never changes imported triangles or frames.
void appendMeshEdges(RobotAssets &assets, RobotMesh &mesh);
// Same manifest identity and content-addressed resource contract as OLP.
QString descriptionIdentity(const QJsonArray &resources);
// A description directory (robot.urdf, manifest, meshes) drawn with an optional
// machine_planning_calibration.json; no network, no substitute robot.
// `packages` are the URDF mesh package names that mean this directory (OLP's
// packageMap for the model); empty means the URDF's own robot name.
RobotAssets loadRobotMeshes(const QString &directory, const QString &calibration = {},
                            const QStringList &packages = {}, const QString &visualDirectory = {});

class RobotView : public QOpenGLWidget, protected QOpenGLFunctions {
public:
  explicit RobotView(QWidget *parent = nullptr);
  ~RobotView() override;
  Kinematics model;
  QJsonObject joints;
  // OLP's stage overlays, in world metres (viewer/overlays.ts): the authored
  // path as weld/travel runs with a dot on every target and sparse row labels,
  // faded while a node is selected; and the kept plan's TCP path, one line per
  // planned segment. Drawn over the robot, as OLP draws them without depth.
  struct PathRun {
    std::vector<QVector3D> points;
    bool weld = false;
  };
  struct PathMark {
    QVector3D at;
    QString text;
    bool weld = false, emphasis = false;
  };
  std::vector<PathRun> authored, planned;
  std::vector<PathMark> targets, labels;
  bool authoredFocused = false;
  bool live = false;
  // Where the view looks from: the orbit camera, or OLP's J6 and TCP cameras,
  // hung on the robot by the mounts in `cameras` (OLP viewer/models.ts).
  enum class Camera { Orbit, Tcp, J6 };
  QJsonObject cameras;
  void setCamera(Camera c);
  Camera cameraMode() const { return camera; }
  QString message = "Waiting for the selected cell's robot description";
  void install(RobotAssets assets);
  void clear();
  void resetCamera() { home(); }
  void controllerCamera(double x,double y,bool zoom) {
    if(zoom)zoomAt({width()/2.0,height()/2.0},float(std::exp(y*.025)));
    else {yaw-=x;pitch=std::clamp(pitch+float(y),5.f,85.f);}
    update();
  }
  // Frame the program's path (authored and planned) in the orbit view; the
  // whole robot when there is no path.
  void fitPath();

protected:
  void initializeGL() override;
  void paintGL() override;
  void mousePressEvent(QMouseEvent *) override;
  void mouseMoveEvent(QMouseEvent *) override;
  void mouseDoubleClickEvent(QMouseEvent *) override;
  void wheelEvent(QWheelEvent *) override;
  bool event(QEvent *) override;
  void resizeEvent(QResizeEvent *) override;

private:
  DiagnosticTimingGate renderTiming;
  RobotAssets scene;
  QOpenGLBuffer buffer, edgeBuffer;
  QOpenGLShaderProgram shader;
  QOpenGLShaderProgram depthShader;
  std::unique_ptr<QOpenGLFramebufferObject> shadowMap;
  bool upload = false;
  QPoint pointer;
  float yaw = -58, pitch = 25, distance = 6;
  Camera camera = Camera::Orbit;
  QVector3D target{.9f, .55f, .65f};
  void home();
  // Orbit zoom by `factor` toward the scene under `at` (widget pixels), and a
  // pan that keeps the scene under the fingers.
  void zoomAt(QPointF at, float factor);
  void panBy(QPointF pixels);
  void orbitAxes(QVector3D &backward, QVector3D &right, QVector3D &up) const;
  QWidget *zoomTools = nullptr;
  OrbitGesture touchGesture;
  QElapsedTimer lastTap;
  QPointF lastTapAt;
};
