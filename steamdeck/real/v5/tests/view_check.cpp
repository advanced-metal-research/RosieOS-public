// The pendant's input interlock, display kinematics, description identity and
// both robots' mesh sets. Run from steamdeck/real/v5 (reads the repo's
// robot_description). QT_QPA_PLATFORM=offscreen ./build/view-check
#include "input.hpp"
#include "robot_view.hpp"
#include <QRegularExpression>
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDirIterator>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTemporaryDir>
#include <cassert>
#include <iostream>

namespace {
QTouchEvent::TouchPoint contact(int id, QPointF pos, Qt::TouchPointState state = Qt::TouchPointMoved) {
  QTouchEvent::TouchPoint point(id);
  point.setPos(pos);
  point.setState(state);
  point.setLastPos({-900, -900}); // Intentionally stale Qt history: never a gesture delta.
  return point;
}
void gestures() {
  using Change = OrbitGesture::Change;
  OrbitGesture gesture;
  assert(gesture.update(QEvent::TouchBegin, {contact(2, {100,100})}).kind == Change::None);
  assert(gesture.update(QEvent::TouchUpdate, {contact(2, {100,100}), contact(5, {300,100})}).kind == Change::None);
  auto change = gesture.update(QEvent::TouchUpdate, {contact(5, {360,120}), contact(2, {60,120})});
  assert(change.kind == Change::PanZoom && std::abs(change.scale - 2.f/3) < 1e-6);
  assert(change.anchor == QPointF(200,100) && change.delta == QPointF(10,20));
  change = gesture.update(QEvent::TouchUpdate, {contact(2, {60,120}, Qt::TouchPointStationary), contact(5, {360,120}, Qt::TouchPointStationary)});
  assert(change.scale == 1 && change.delta.isNull()); // Stationary fingers do not replay zoom/pan.
  change = gesture.update(QEvent::TouchUpdate, {contact(2, {60,120}, Qt::TouchPointStationary), contact(5, {390,120})});
  assert(std::abs(change.scale - 300.f/330) < 1e-6 && change.delta == QPointF(15,0));
  assert(gesture.update(QEvent::TouchUpdate, {contact(2, {60,120}), contact(5, {390,120}, Qt::TouchPointReleased)}).kind == Change::None);
  change = gesture.update(QEvent::TouchUpdate, {contact(2, {65,122})});
  assert(change.kind == Change::Orbit && change.delta == QPointF(5,2));
  assert(gesture.update(QEvent::TouchEnd, {contact(2, {65,122}, Qt::TouchPointReleased)}).kind == Change::None);
  assert(!gesture.active()); // Ending a pinch cannot count as a double-tap reset.
  gesture.update(QEvent::TouchBegin, {contact(1, {20,20})});
  assert(gesture.update(QEvent::TouchEnd, {contact(1, {21,20}, Qt::TouchPointReleased)}).kind == Change::Tap);
  gesture.update(QEvent::TouchBegin, {contact(1, {20,20})});
  assert(gesture.update(QEvent::TouchCancel, {}).kind == Change::None && !gesture.active());
  gesture.update(QEvent::TouchBegin, {contact(1, {20,20}), contact(2, {100,20})});
  assert(gesture.update(QEvent::TouchUpdate, {contact(3, {20,20}), contact(2, {100,20})}).kind == Change::None);
  assert(gesture.update(QEvent::TouchUpdate, {contact(3, {20,20}), contact(2, {100,20}), contact(4, {80,40})}).kind == Change::None);
  assert(gesture.update(QEvent::TouchUpdate, {contact(3, {30,20}), contact(2, {110,20}), contact(4, {90,40})}).kind == Change::None);
}
void meshEdges() {
  RobotAssets square;
  const QVector3D n(0,0,1);
  square.vertices = {{{0,0,0},n}, {{1,0,0},n}, {{1,1,0},n},
                     {{0,0,0},n}, {{1,1,0},n}, {{0,1,0},n}};
  RobotMesh mesh; mesh.count = 6;
  appendMeshEdges(square, mesh);
  assert(square.vertices.size() == 6); // Display edges never modify source geometry.
  assert(mesh.edgeCount == 8 && mesh.detailCount == 2); // Four boundaries; faint coplanar diagonal.
  square.edges.clear();
  square.vertices[3] = {{0,0,0},{1,0,0}};
  square.vertices[4] = {{0,1,0},{1,0,0}};
  square.vertices[5] = {{0,0,1},{1,0,0}};
  square.vertices[0] = {{0,0,0},n}; square.vertices[1] = {{0,1,0},n}; square.vertices[2] = {{1,0,0},n};
  appendMeshEdges(square, mesh);
  assert(mesh.edgeCount == 10 && mesh.detailCount == 0); // A shared 90-degree crease stays bright.
}
} // namespace

int main(int argc, char **argv) {
  QCoreApplication app(argc, argv);
  gestures();
  meshEdges();
  PadInput pad;
  assert(!pad.enabled() && !pad.rotate() && pad.magnitude() == 0);
  pad.axis(0, 2000); // Ignore stick drift.
  assert(pad.magnitude() == 0);
  pad.axis(1, -32767);
  pad.axis(3, 32767);
  pad.axis(4, -32767);
  assert((pad.twist() == std::array<double,6>{1,0,1,0,0,1}));
  assert(!pad.enabled()); // Stick movement alone never enables a jog.
  pad.axis(5, 0);
  pad.axis(2, 0);
  assert(pad.enabled() && pad.rotate());
  assert((pad.twist() == std::array<double,6>{0,0,1,0,1,1}));
  pad.axis(0,-32767);
  assert((pad.twist() == std::array<double,6>{0,0,1,-1,1,1}));
  pad.axis(5,-32767); // R2 releases the tilt modifier.
  assert((pad.twist() == std::array<double,6>{1,1,1,0,0,1}));
  assert(pad.enabled()); // Releasing R2 does not disarm.
  pad.axis(2, -32767); // L2 releases the arm hold.
  assert(!pad.enabled());
  pad = {}; // Reconnection starts with both triggers released.
  assert(!pad.enabled() && pad.magnitude() == 0);
  InputGate gate;
  assert(gate.sample(true, true, 1) == 0); // startup while held
  assert(gate.sample(true, false, 0) == 0);
  assert(gate.sample(true, true, 1) == 1);
  assert(gate.sample(false, true, 1) == 0); // focus/page/authority lost
  assert(gate.sample(true, true, 1) == 0);  // returning does not restart
  gate.sample(true, false, 0);
  assert(gate.sample(true, true, -1) == -1);
  gate.cancel();
  assert(gate.sample(true, true, -1) == 0);
  QString error;
  QTemporaryDir dir;
  QFile urdf(dir.filePath("robot.urdf"));
  urdf.open(QIODevice::WriteOnly);
  urdf.write(
      R"(<robot name="test"><joint name="J1" type="revolute"><parent link="world"/><child link="arm"/><axis xyz="0 0 1"/></joint><joint name="tcp" type="fixed"><parent link="arm"/><child link="tool0"/><origin xyz="1 0 0"/></joint></robot>)");
  urdf.close();
  Kinematics k;
  assert(k.load(urdf.fileName(), error));
  assert(!k.covers({}));
  assert(k.covers({{"J1", 0.}}));
  assert(k.loadCalibration("", error));
  assert(!k.calibrationSha256.isEmpty());
  auto end = k.points({{"J1", 3.141592653589793 / 2}}).back();
  assert(std::abs(end.x()) < 1e-5 && std::abs(end.y() - 1) < 1e-5);
  QFile calibration(dir.filePath("calibration.json"));
  calibration.open(QIODevice::WriteOnly);
  calibration.write(
      R"({"schema":"rosie.machine-planning-calibration.v1","model_id":"test","kinematics":{"tcp":{"xyz_m":[0.1,0,0]}}})");
  calibration.close();
  assert(k.loadCalibration(calibration.fileName(), error));
  end = k.points({{"J1", 3.141592653589793 / 2}}).back();
  assert(std::abs(end.x()) < 1e-5 && std::abs(end.y() - 1.1) < 1e-5);
  assert(!k.loadCalibration(dir.filePath("missing"), error));
  assert(k.calibrationSha256.isEmpty());
  // Existing cross-language identity vector, shared with robot_description/go.
  auto resource = [](QString path, QByteArray data) {
    return QJsonObject{
        {"path", path},
        {"sha256", QString::fromLatin1(QCryptographicHash::hash(
                                           data, QCryptographicHash::Sha256)
                                           .toHex())}};
  };
  QJsonArray resources{
      resource("robot.urdf", R"(<robot name="cell_a"/>)"),
      resource("meshes/base.dae", "<COLLADA/>"),
      resource(
          "robot_description_manifest.json",
          R"({"schema":"rosie.robot-manifest.v1","model_id":"cell_a","files":["meshes/base.dae","robot.urdf"]})")};
  assert(descriptionIdentity(resources) ==
         "sha256:"
         "9c3105ce99e2cbbc138f7051596f69d4a14b71dab01d98732274c37207901483");
  resources.append(resources[0]);
  assert(descriptionIdentity(resources).isEmpty());
  assert(descriptionIdentity({resource("../robot.urdf", "bad")}).isEmpty());
  // Every robot description in the repository that has visual meshes, so a
  // new robot is covered when its directory lands. The packages are the ones
  // its own URDF names, which is what OLP's packageMap (viewer/models.ts)
  // gives the pendant for that model.
  const QDir robots("../../../robot_description/robots");
  int drawn = 0;
  // The Rosie 1430's detailed CAD visuals (assets/robot-visuals/rosie1430) are
  // checked once, on the first description that names the 1430 package; a
  // tree without a 1430 (the public mirror) has neither.
  bool detailed1430 = false;
  for (const auto &id : robots.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name)) {
    QFile urdf(robots.filePath(id + "/robot.urdf"));
    if (!urdf.open(QIODevice::ReadOnly)) continue;
    const QString text = QString::fromUtf8(urdf.readAll());
    if (!text.contains("<visual")) continue; // a bench description has nothing to draw
    QStringList packages;
    auto names = QRegularExpression("package://([^/\"]+)/").globalMatch(text);
    while (names.hasNext()) {
      const auto name = names.next().captured(1);
      if (!packages.contains(name)) packages << name;
    }
    ++drawn;
    QTemporaryDir assets;
    const QString source =
        QDir("../../../robot_description/robots/" + id).absolutePath();
    QDirIterator it(source, QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) {
      auto file = it.next(), relative = QDir(source).relativeFilePath(file);
      QDir(assets.path()).mkpath(QFileInfo(relative).path());
      assert(QFile::copy(file, assets.filePath(relative)));
    }
    QFile calibration(assets.filePath("machine_planning_calibration.json"));
    assert(calibration.open(QIODevice::WriteOnly));
    calibration.write(("{\"schema\":\"rosie.machine-planning-calibration.v1\","
                       "\"model_id\":\"" +
                       id + "\"}\n")
                          .toUtf8());
    calibration.close();
    auto loaded = loadRobotMeshes(assets.path(), assets.filePath("machine_planning_calibration.json"), packages);
    // A description whose meshes are not named after the robot has nothing
    // drawable without its package name: the pendant showed an empty 3D view
    // for such a robot this way.
    if (packages != QStringList{id})
      assert(loadRobotMeshes(assets.path(), assets.filePath("machine_planning_calibration.json")).error ==
             "Invalid URDF mesh path or frame");
    if (!loaded.error.isEmpty())
      std::cerr << loaded.error.toStdString() << '\n';
    assert(loaded.error.isEmpty() && loaded.meshes.size() >= 7 &&
           loaded.vertices.size() > 1000);
    assert(!loaded.edges.empty());
    bool pedestal = false;
    for (const auto &mesh : loaded.meshes) {
      assert(mesh.edgeFirst >= 0 && mesh.detailFirst + mesh.detailCount <= int(loaded.edges.size()));
      assert(mesh.edgeCount > 0 && mesh.edgeCount % 2 == 0 && mesh.detailCount % 2 == 0);
      pedestal |= mesh.pedestal;
    }
    if (packages.contains("rosie_1430_v1") && !detailed1430) {
      detailed1430 = true;
      assert(pedestal && loaded.visualSource == "JSLOP Rosie 1430 detailed CAD");
      assert(loaded.vertices.size() > 1200000); // Detailed machining, not collision proxies.
      Kinematics original;
      assert(original.load(assets.filePath("robot.urdf"), error));
      assert(original.loadCalibration(calibration.fileName(), error));
      assert(original.urdfSha256 == loaded.model.urdfSha256);
      assert(original.calibrationSha256 == loaded.model.calibrationSha256);
      for (double angle : {-.5, 0., .5}) {
        QJsonObject pose;
        for (int axis = 1; axis <= 6; ++axis) pose[QString("J%1").arg(axis)] = angle;
        assert(original.frames(pose) == loaded.model.frames(pose));
        assert(original.points(pose) == loaded.model.points(pose));
      }
      QFile provenance("assets/robot-visuals/rosie1430/cad-provenance.json");
      assert(provenance.open(QIODevice::ReadOnly));
      const auto sources = QJsonDocument::fromJson(provenance.readAll()).object()["links"].toObject();
      // The STEP has J5 at -90 degrees; JSLOP baked +90 degrees into that
      // mesh to express it in the URDF's joint-local zero frame (provenance).
      const auto frames = loaded.model.frames({{"J5", -3.141592653589793 / 2}});
      const auto baseInverse = frames["base"].inverted();
      for (const auto &mesh : loaded.meshes) {
        const auto source = sources[mesh.link + ".dae"].toObject();
        if (source.isEmpty()) continue; // Original pedestal and J6 remain intact.
        assert(mesh.count == source["triangles"].toInt() * 3);
        QVector3D lower(1e6,1e6,1e6), upper(-1e6,-1e6,-1e6);
        const auto transform = baseInverse * frames[mesh.link] * mesh.origin;
        for (int i = mesh.first; i < mesh.first + mesh.count; ++i) {
          const auto p = transform.map(loaded.vertices[i].position);
          for (int a = 0; a < 3; ++a) { lower[a] = std::min(lower[a],p[a]); upper[a] = std::max(upper[a],p[a]); }
        }
        const auto bounds = source["source_world_bounds_m"].toArray();
        for (int a = 0; a < 3; ++a) {
          assert(std::abs(lower[a] - bounds[0].toArray()[a].toDouble()) < .0005);
          assert(std::abs(upper[a] - bounds[1].toArray()[a].toDouble()) < .0005);
        }
      }
      // A corrupt detail package is explicit, never silently replaced by the old proxy.
      QTemporaryDir damaged;
      assert(QFile::copy("assets/robot-visuals/rosie1430/manifest.json", damaged.filePath("manifest.json")));
      QFile bad(damaged.filePath("base.dae")); assert(bad.open(QIODevice::WriteOnly)); bad.write("corrupt"); bad.close();
      assert(loadRobotMeshes(assets.path(), calibration.fileName(), packages, damaged.path()).error.contains("checksum mismatch"));
      QFile modified(assets.filePath("robot.urdf"));
      assert(modified.open(QIODevice::ReadOnly)); auto bytes = modified.readAll(); modified.close();
      assert(bytes.contains("xyz=\"0 -1 0\"")); bytes.replace("xyz=\"0 -1 0\"", "xyz=\"1 0 0\"");
      assert(modified.open(QIODevice::WriteOnly)); modified.write(bytes); modified.close();
      assert(loadRobotMeshes(assets.path(), calibration.fileName(), packages).error.contains("nominal joint frames"));
    }
    auto joints = QJsonObject{{"J1", .4}, {"J2", -.2}, {"J7", .5}};
    auto tcp = loaded.model.points(joints).back();
    assert(
        (loaded.model.frames(joints)["tool0"].map(QVector3D{}) - tcp).length() <
        1e-5);
    if (id == "rosie_1400_v3") {
      auto base = loaded.model.frames({});
      auto moved = loaded.model.frames({{"J7", .5}});
      assert(base["tool0"] == moved["tool0"]);
      assert(base !=
             moved); // Positioner branch is rendered, not just the TCP chain.
    }
  }
  assert(drawn >= 2);
  std::cout << "PASS pinch contact transitions, CAD edges, input neutral interlock, URDF transform and calibration, description identity, "
            << drawn << " robot mesh sets\n";
}
