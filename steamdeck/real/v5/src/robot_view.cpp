#include "robot_view.hpp"
#include "theme.hpp"
#include <QCryptographicHash>
#include <QDir>
#include <QDomDocument>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QMouseEvent>
#include <QPushButton>
#include <QTouchEvent>
#include <QVBoxLayout>
#include <QPainter>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSet>
#include <QTemporaryFile>
#include <QtEndian>
#include <QtMath>
#include <array>
#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <cmath>
#include <unordered_map>
#include <QCoreApplication>

namespace {
QString hash(const QByteArray &bytes) {
  return QString::fromLatin1(
      QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex());
}
bool safePath(const QString &path) {
  return !path.isEmpty() && !path.contains('\\') && !path.contains(':') &&
         !path.startsWith('/') && QDir::cleanPath(path) == path &&
         !path.split('/').contains("..") && !path.split('/').contains(".");
}
QVector3D vector(const QString &s, QVector3D fallback = {}) {
  const auto values = s.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
  if (values.size() != 3)
    return fallback;
  return {values[0].toFloat(), values[1].toFloat(), values[2].toFloat()};
}
QMatrix4x4 matrix(const aiMatrix4x4 &m) {
  return QMatrix4x4(m.a1, m.a2, m.a3, m.a4, m.b1, m.b2, m.b3, m.b4, m.c1, m.c2,
                    m.c3, m.c4, m.d1, m.d2, m.d3, m.d4);
}
// OLP viewer/urdf.ts uses this palette instead of imported CAD materials.
QColor color(const QString &path) {
  auto name = QFileInfo(path).fileName().toLower();
  if (name.contains(QRegularExpression("floor|pedestal|base")))
    return QColor(0x69, 0x79, 0x85);
  if (name.contains(QRegularExpression("positioner|h_frame")))
    return QColor(0x82, 0x97, 0xa8);
  if (name.contains(QRegularExpression("end_effector|tool")))
    return QColor(0xe3, 0xe9, 0xed);
  uint32_t n = 0;
  for (auto c : name)
    n = n * 31 + c.unicode();
  const QRgb palette[] = {0xff5c646b, 0xff4a5259, 0xff6b737a, 0xff3f474e};
  return QColor(palette[n % 4]);
}
void triangles(const aiScene *scene, const aiNode *node, QMatrix4x4 parent,
               std::vector<RobotVertex> &out) {
  const auto t = parent * matrix(node->mTransformation);
  for (unsigned i = 0; i < node->mNumMeshes; ++i) {
    const auto *mesh = scene->mMeshes[node->mMeshes[i]];
    for (unsigned f = 0; f < mesh->mNumFaces; ++f) {
      const auto &face = mesh->mFaces[f];
      if (face.mNumIndices != 3)
        continue;
      // OLP's Collada geometry is non-indexed: computeVertexNormals produces
      // the geometric triangle normal, not the CAD file's smoothed normals.
      QVector3D points[3];
      for (unsigned v = 0; v < 3; ++v) {
        const auto p = mesh->mVertices[face.mIndices[v]];
        points[v] = t.map({p.x, p.y, p.z});
      }
      const auto normal =
          QVector3D::normal(points[1] - points[0], points[2] - points[0]);
      for (auto point : points)
        out.push_back({point, normal});
    }
  }
  for (unsigned i = 0; i < node->mNumChildren; ++i)
    triangles(scene, node->mChildren[i], t, out);
}
} // namespace

void appendMeshEdges(RobotAssets &assets, RobotMesh &mesh) {
  using Key = std::array<qint64, 6>;
  struct Hash {
    size_t operator()(const Key &key) const {
      size_t result = 0;
      for (auto value : key) result ^= std::hash<qint64>{}(value) + 0x9e3779b9 + (result << 6) + (result >> 2);
      return result;
    }
  };
  struct Edge { QVector3D a, b, normal; int faces = 1; bool crease = false; };
  std::unordered_map<Key, Edge, Hash> edges;
  edges.reserve(size_t(mesh.count));
  auto keyPoint = [](QVector3D p) {
    return std::array<qint64, 3>{std::llround(p.x() * 100000.), std::llround(p.y() * 100000.), std::llround(p.z() * 100000.)};
  };
  // Match JSLOP's 8-degree CAD creases, leaving triangulation diagonals faint.
  const float threshold = std::cos(qDegreesToRadians(8.f));
  for (int i = mesh.first; i + 2 < mesh.first + mesh.count; i += 3) {
    const auto normal = assets.vertices[i].normal;
    if (normal.lengthSquared() < .5f) continue;
    for (int j = 0; j < 3; ++j) {
      auto a = assets.vertices[i + j].position, b = assets.vertices[i + (j + 1) % 3].position;
      auto ka = keyPoint(a), kb = keyPoint(b);
      if (ka == kb) continue;
      if (kb < ka) { std::swap(ka, kb); std::swap(a, b); }
      const Key key{ka[0], ka[1], ka[2], kb[0], kb[1], kb[2]};
      auto [it, inserted] = edges.emplace(key, Edge{a, b, normal});
      if (!inserted) {
        ++it->second.faces;
        it->second.crease |= QVector3D::dotProduct(it->second.normal, normal) < threshold;
      }
    }
  }
  auto append = [&](bool sharp) {
    for (const auto &[key, edge] : edges) {
      (void)key;
      if ((edge.faces == 1 || edge.crease) != sharp) continue;
      assets.edges.push_back({edge.a, {}});
      assets.edges.push_back({edge.b, {}});
    }
  };
  mesh.edgeFirst = int(assets.edges.size()); append(true);
  mesh.edgeCount = int(assets.edges.size()) - mesh.edgeFirst;
  mesh.detailFirst = int(assets.edges.size()); append(false);
  mesh.detailCount = int(assets.edges.size()) - mesh.detailFirst;
}

QString descriptionIdentity(const QJsonArray &resources) {
  QMap<QString, QByteArray> entries;
  for (auto r : resources) {
    auto o = r.toObject();
    auto path = o["path"].toString(), sha = o["sha256"].toString();
    if (!safePath(path) || entries.contains(path) ||
        !QRegularExpression("^[0-9a-f]{64}$").match(sha).hasMatch())
      return {};
    if (path != "machine_planning_calibration.json")
      entries[path] = QByteArray::fromHex(sha.toLatin1());
  }
  QByteArray bytes("rosie.robot-description.identity.v1\0", 36);
  auto number = [&](quint64 n) {
    n = qToBigEndian(n);
    bytes.append(reinterpret_cast<const char *>(&n), 8);
  };
  number(entries.size());
  for (auto i = entries.begin(); i != entries.end(); ++i) {
    auto path = i.key().toUtf8();
    number(path.size());
    bytes += path;
    bytes += i.value();
  }
  return "sha256:" + hash(bytes);
}

// The detailed visual set installed for model, if any: a robot-visuals
// directory beside the app (share/, or the in-tree assets/ of a development
// build) whose manifest names that model. A set is made for one model and says
// so, so no model is named here; a build without one (the public tree) draws
// the description's own meshes. A set that names the model but is damaged is
// still an error below, never replaced by those meshes.
static QString installedVisualDirectory(const QString &model) {
  const auto root = QCoreApplication::applicationDirPath() + "/../";
  for (const auto &base : {root + "share/robot-visuals", root + "assets/robot-visuals"}) {
    for (const auto &entry : QDir(base).entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name)) {
      QFile manifest(base + "/" + entry + "/manifest.json");
      if (manifest.open(QIODevice::ReadOnly) &&
          QJsonDocument::fromJson(manifest.readAll()).object()["model"].toString() == model)
        return base + "/" + entry;
    }
  }
  return {};
}

RobotAssets loadRobotMeshes(const QString &directory, const QString &calibration,
                            const QStringList &packages, const QString &visualDirectory) {
  RobotAssets out;
  out.directory = directory;
  QString error;
  // No calibration file is the model's empty document, as in OLP.
  if (!out.model.load(directory + "/robot.urdf", error) ||
      !out.model.loadCalibration(QFile::exists(calibration) ? calibration : QString(), error)) {
    out.error = error;
    return out;
  }
  QFile file(directory + "/robot.urdf");
  file.open(QIODevice::ReadOnly);
  QDomDocument doc;
  if (!doc.setContent(file.readAll())) {
    out.error = "Invalid URDF XML";
    return out;
  }
  QJsonObject detailed;
  const QString artDirectory =
      visualDirectory.isEmpty() ? installedVisualDirectory(out.model.model) : visualDirectory;
  if (!artDirectory.isEmpty()) {
    QFile art(artDirectory + "/manifest.json");
    if (!art.open(QIODevice::ReadOnly)) {
      out.error = "Detailed robot visual assets are missing";
      return out;
    }
    const auto metadata = QJsonDocument::fromJson(art.readAll()).object();
    const auto joints = metadata["joints"].toArray();
    if (metadata["schema"] != "rosie.pendant-visuals.v1" || metadata["model"] != out.model.model || joints.size() != 6) {
      out.error = "Invalid detailed robot visual manifest";
      return out;
    }
    auto same = [](const QJsonValue &value, QVector3D expected) {
      const auto v = value.toArray();
      if (v.size() != 3) return false;
      for (int i = 0; i < 3; ++i)
        if (!v[i].isDouble() || !std::isfinite(v[i].toDouble()) || std::abs(v[i].toDouble() - expected[i]) > 1e-6) return false;
      return true;
    };
    for (const auto &value : joints) {
      const auto j = value.toObject();
      const auto match = std::find_if(out.model.all.begin(), out.model.all.end(), [&](const auto &entry) { return entry.name == j["name"].toString(); });
      if (match == out.model.all.end() || match->type != "revolute" || match->parent != j["parent"].toString() || match->child != j["child"].toString() ||
          !same(j["xyz"], match->xyz) || !same(j["rpy"], match->rpy) || !same(j["axis"], match->axis)) {
        out.error = "Detailed robot visuals do not match the nominal joint frames";
        return out;
      }
    }
    detailed = metadata["meshes"].toObject();
    if (detailed.size() != 6) {
      out.error = "Detailed robot visual mesh list is incomplete";
      return out;
    }
    for (auto it = detailed.begin(); it != detailed.end(); ++it) {
      const auto m = it.value().toObject();
      const QString name = m["file"].toString();
      if (name != it.key() + ".dae" || !safePath(name)) {
        out.error = "Invalid detailed robot visual path";
        return out;
      }
      QFile mesh(artDirectory + "/" + name);
      if (!mesh.open(QIODevice::ReadOnly) || hash(mesh.readAll()) != m["sha256"].toString()) {
        out.error = "Detailed robot visual checksum mismatch: " + name;
        return out;
      }
    }
    out.visualSource = metadata["label"].toString();
  }
  const auto links = doc.elementsByTagName("link");
  const auto frames = out.model.frames({});
  for (int i = 0; i < links.size(); ++i) {
    auto link = links.at(i).toElement();
    for (auto visual = link.firstChildElement("visual"); !visual.isNull();
         visual = visual.nextSiblingElement("visual")) {
      auto mesh =
          visual.firstChildElement("geometry").firstChildElement("mesh");
      if (mesh.isNull()) {
        out.error = "Unsupported visual geometry in robot description";
        return out;
      }
      auto path = mesh.attribute("filename");
      // A robot-specific description (one arm's own) keeps its product's
      // package name, so the robot name alone does not name its meshes.
      for (const auto &package : packages.isEmpty() ? QStringList{out.model.model} : packages) {
        const auto prefix = "package://" + package + "/";
        if (path.startsWith(prefix)) {
          path = path.mid(prefix.size());
          break;
        }
      }
      if (!safePath(path) || !frames.contains(link.attribute("name"))) {
        out.error = "Invalid URDF mesh path or frame";
        return out;
      }
      // Mesh files must belong to the verified description, not just exist next
      // to it.
      QFile manifest(directory + "/robot_description_manifest.json");
      manifest.open(QIODevice::ReadOnly);
      if (!QJsonDocument::fromJson(manifest.readAll())
               .object()["files"]
               .toArray()
               .contains(path)) {
        out.error = "URDF mesh is absent from manifest: " + path;
        return out;
      }
      Assimp::Importer importer;
      // OLP resets Collada's up-axis rotation and keeps its unit scale.
      importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION,
                               true);
      const auto art = detailed[link.attribute("name")].toObject();
      const QString renderPath = art.isEmpty() ? directory + "/" + path
                                               : artDirectory + "/" + art["file"].toString();
      const auto *scene = importer.ReadFile(renderPath.toStdString(), aiProcess_Triangulate);
      if (!scene || !scene->mRootNode) {
        out.error =
            "Mesh load failed: " + path + " · " + importer.GetErrorString();
        return out;
      }
      RobotMesh draw;
      draw.link = link.attribute("name");
      draw.color = color(path);
      draw.pedestal = QFileInfo(path).fileName().contains(QRegularExpression("pedestal|floor", QRegularExpression::CaseInsensitiveOption));
      draw.first = out.vertices.size();
      auto origin = visual.firstChildElement("origin");
      auto xyz = vector(origin.attribute("xyz")),
           rpy = vector(origin.attribute("rpy"));
      draw.origin.translate(xyz);
      draw.origin.rotate(qRadiansToDegrees(rpy.z()), 0, 0, 1);
      draw.origin.rotate(qRadiansToDegrees(rpy.y()), 0, 1, 0);
      draw.origin.rotate(qRadiansToDegrees(rpy.x()), 1, 0, 0);
      draw.origin.scale(vector(mesh.attribute("scale"), {1, 1, 1}));
      triangles(scene, scene->mRootNode, {}, out.vertices);
      draw.count = int(out.vertices.size()) - draw.first;
      if (!draw.count || out.vertices.size() > 4000000) {
        out.error = "Invalid or excessive mesh geometry";
        return out;
      }
      appendMeshEdges(out, draw);
      out.meshes.push_back(draw);
    }
  }
  if (out.meshes.empty())
    out.error = "Description contains no visual meshes";
  return out;
}

RobotView::RobotView(QWidget *parent) : QOpenGLWidget(parent) {
  auto surface = format();
  surface.setSamples(4);
  surface.setDepthBufferSize(24);
  setFormat(surface);
  setMinimumHeight(220);
  setToolTip("Drag to orbit · pinch or scroll to zoom · two fingers pan · double-tap to reset");
  // The Deck's screen is touch: pinch zooms and two fingers pan (event()).
  // The buttons zoom without a pinch; FIT always frames the whole robot.
  setAttribute(Qt::WA_AcceptTouchEvents);
  zoomTools = new QWidget(this);
  auto *tools = new QVBoxLayout(zoomTools);
  tools->setContentsMargins(0, 0, 0, 0);
  tools->setSpacing(6);
  auto tool = [&](const QString &text, const QString &tip, auto action) {
    auto *b = new QPushButton(text, zoomTools);
    b->setFixedSize(52, 44);
    b->setFocusPolicy(Qt::NoFocus);
    b->setToolTip(tip);
    b->setStyleSheet("font-size: 20px; font-weight: 700; padding: 0;");
    connect(b, &QPushButton::clicked, this, action);
    tools->addWidget(b);
    return b;
  };
  tool("+", "Zoom in", [this] { zoomAt({width() / 2.0, height() / 2.0}, 1 / 1.4f); });
  tool("−", "Zoom out", [this] { zoomAt({width() / 2.0, height() / 2.0}, 1.4f); });
  auto *fit = tool("FIT", "Fit the whole robot and pedestal", [this] { home(); });
  fit->setObjectName("fitWholeRobot");
  fit->setAccessibleName("Fit the whole robot and pedestal");
  fit->setStyleSheet("font-size: 12px; font-weight: 700; padding: 0;");
  zoomTools->adjustSize();
}
void RobotView::setCamera(Camera c) {
  camera = c;
  // The TCP and J6 cameras hang on the robot: only the orbit view zooms.
  zoomTools->setVisible(c == Camera::Orbit);
  update();
}
void RobotView::resizeEvent(QResizeEvent *e) {
  QOpenGLWidget::resizeEvent(e);
  zoomTools->move(width() - zoomTools->width() - 12, height() - zoomTools->height() - 12);
}
void RobotView::orbitAxes(QVector3D &backward, QVector3D &right, QVector3D &up) const {
  const float y = qDegreesToRadians(yaw), p = qDegreesToRadians(pitch);
  backward = QVector3D(std::cos(p) * std::cos(y), std::cos(p) * std::sin(y), std::sin(p));
  right = QVector3D::crossProduct({0, 0, 1}, backward).normalized();
  up = QVector3D::crossProduct(backward, right);
}
// 12 cm from the target still frames a single weld joint; the far bound keeps
// a whole cell in view without losing the robot to the near plane.
constexpr float minDistanceM = .12f, maxDistanceM = 15.f;
void RobotView::zoomAt(QPointF at, float factor) {
  if (camera != Camera::Orbit) return;
  const float next = std::clamp(distance * factor, minDistanceM, maxDistanceM);
  QVector3D backward, right, up;
  orbitAxes(backward, right, up);
  // The point under `at` on the plane through the target facing the camera
  // (the orbit lens is 46 degrees vertical) stays under `at` as distance changes.
  const float half = distance * std::tan(qDegreesToRadians(23.f));
  const float aspect = float(width()) / std::max(1, height());
  const float nx = float(2 * at.x() / std::max(1, width()) - 1), ny = float(1 - 2 * at.y() / std::max(1, height()));
  const QVector3D under = target + right * (nx * half * aspect) + up * (ny * half);
  target += (under - target) * (1 - next / distance);
  distance = next;
  update();
}
void RobotView::panBy(QPointF pixels) {
  if (camera != Camera::Orbit) return;
  QVector3D backward, right, up;
  orbitAxes(backward, right, up);
  const float metresPerPixel = 2 * distance * std::tan(qDegreesToRadians(23.f)) / std::max(1, height());
  target -= (right * float(pixels.x()) - up * float(pixels.y())) * metresPerPixel;
  update();
}
void RobotView::fitPath() {
  std::vector<QVector3D> points;
  for (const auto *runs : {&authored, &planned})
    for (const auto &run : *runs) points.insert(points.end(), run.points.begin(), run.points.end());
  for (const auto &mark : targets) points.push_back(mark.at);
  if (points.empty()) {
    home();
    return;
  }
  QVector3D lower = points[0], upper = points[0];
  for (const auto &point : points)
    for (int a = 0; a < 3; ++a) {
      lower[a] = std::min(lower[a], point[a]);
      upper[a] = std::max(upper[a], point[a]);
    }
  target = (lower + upper) * .5f;
  // The path's bounding sphere inside the narrower half-angle, with a margin
  // for the labels; a single target still gets a 25 cm view round it.
  const float radius = std::max((upper - lower).length() * .5f, .08f);
  const float half = std::tan(qDegreesToRadians(23.f)) * std::min(1.f, float(width()) / std::max(1, height()));
  distance = std::clamp(radius / half * 1.4f, minDistanceM, maxDistanceM);
  update();
}
RobotView::~RobotView() {
  if (context()) {
    makeCurrent();
    buffer.destroy();
    edgeBuffer.destroy();
    shadowMap.reset();
    doneCurrent();
  }
}
void RobotView::clear() {
  model = {};
  scene = {};
  joints = {};
  authored.clear();
  planned.clear();
  targets.clear();
  labels.clear();
  live = false;
  upload = true;
  update();
}
void RobotView::install(RobotAssets assets) {
  scene = std::move(assets);
  model = scene.model;
  message = scene.error;
  upload = true;
  home();
  update();
}
void RobotView::home() {
  yaw = -60.5;
  pitch = 20.6;
  // Use OLP's viewing direction and lens, fitting the verified geometry to
  // the pendant pane rather than inheriting a desktop-sized camera distance.
  target = model.model == "rosie_1420_v1" ? QVector3D(.7, .2, 1.05)
                                          : QVector3D(.9, .55, .65);
  const auto frames = model.frames(joints);
  QVector3D lower(1e6, 1e6, 1e6), upper(-1e6, -1e6, -1e6);
  for (const auto &mesh : scene.meshes) {
    const auto transform = frames.value(mesh.link) * mesh.origin;
    for (int i = mesh.first; i < mesh.first + mesh.count; ++i) {
      const auto point = transform.map(scene.vertices[i].position);
      for (int a = 0; a < 3; ++a) {
        lower[a] = std::min(lower[a], point[a]);
        upper[a] = std::max(upper[a], point[a]);
      }
    }
  }
  if (!scene.meshes.empty()) {
    target = (lower + upper) * .5f;
    const float y = qDegreesToRadians(yaw), p = qDegreesToRadians(pitch);
    const QVector3D backward(std::cos(p) * std::cos(y),
                             std::cos(p) * std::sin(y), std::sin(p));
    const auto right =
        QVector3D::crossProduct({0, 0, 1}, backward).normalized();
    const auto up = QVector3D::crossProduct(backward, right);
    const float vertical = std::tan(qDegreesToRadians(23.f));
    const float horizontal = vertical * float(width()) / std::max(1, height());
    distance = .5;
    for (const auto &mesh : scene.meshes) {
      const auto transform = frames.value(mesh.link) * mesh.origin;
      for (int i = mesh.first; i < mesh.first + mesh.count; ++i) {
        const auto v = transform.map(scene.vertices[i].position) - target;
        const float z = QVector3D::dotProduct(v, backward);
        distance = std::max(
            distance,
            z + std::max(std::abs(QVector3D::dotProduct(v, right)) / horizontal,
                         std::abs(QVector3D::dotProduct(v, up)) / vertical));
      }
    }
    distance *= 1.05f;
  }
  update();
}
void RobotView::initializeGL() {
  initializeOpenGLFunctions();
  // Keep OLP geometry/overlays; shade the arm with JSLOP's blue CAD finish.
  const char *vertex = R"(#version 120
    attribute vec3 position; attribute vec3 normal;
    uniform mat4 mvp, modelMatrix, modelView; uniform mat3 normals;
    varying vec3 n, world; varying float depth;
    void main(){gl_Position=mvp*vec4(position,1.0);n=normals*normal;
      world=(modelMatrix*vec4(position,1.0)).xyz;
      depth=-(modelView*vec4(position,1.0)).z;gl_PointSize=7.0;})";
  const char *fragment = R"(#version 120
    uniform vec3 color, eye; uniform bool unlit, hologram; uniform float opacity, edgeDetail;
    uniform mat4 lightVP; uniform sampler2D shadowTexture;
    varying vec3 n, world; varying float depth;
    vec3 linear(vec3 c){return mix(c/12.92,pow((c+.055)/1.055,vec3(2.4)),step(vec3(.04045),c));}
    vec3 srgb(vec3 c){return mix(c*12.92,1.055*pow(max(c,vec3(0)),vec3(1.0/2.4))-.055,step(vec3(.0031308),c));}
    // Three.js ACES tone-mapping fit (MIT); exposure matches OLP's 1.25.
    vec3 aces(vec3 c){
      c=mat3(.59719,.07600,.02840,.35458,.90834,.13383,.04823,.01566,.83777)*(c*1.25/.6);
      c=(c*(c+.0245786)-.000090537)/(c*(.983729*c+.4329510)+.238081);
      return clamp(mat3(1.60475,-.10208,-.00327,-.53108,1.10813,-.07276,-.07367,-.00605,1.07602)*c,0.0,1.0);
    }
    vec3 light(vec3 N,vec3 V,vec3 L,vec3 radiance,vec3 albedo){
      vec3 H=normalize(V+L);float nl=max(dot(N,L),0.0),nv=max(dot(N,V),.0001);
      float nh=max(dot(N,H),0.0),vh=max(dot(V,H),0.0);
      float a2=pow(.62,4.0),d=nh*nh*(a2-1.0)+1.0;
      float D=a2/(3.14159265*d*d);
      float vis=.5/max(nl*sqrt(nv*nv*(1.0-a2)+a2)+nv*sqrt(nl*nl*(1.0-a2)+a2),.0001);
      vec3 f0=mix(vec3(.04),albedo,.08);
      vec3 F=f0+(1.0-f0)*pow(1.0-vh,5.0);
      return radiance*nl*(albedo*.92/3.14159265+F*D*vis);
    }
    float visibility(vec3 N){
      vec3 p=(lightVP*vec4(world,1.0)).xyz*.5+.5;
      if(any(lessThan(p,vec3(0)))||any(greaterThan(p,vec3(1)))) return 1.0;
      float bias=max(.000015,.00008*(1.0-max(dot(N,normalize(vec3(5,-4,7))),0.0)));
      float lit=0.0;
      for(int x=-1;x<=1;x++) for(int y=-1;y<=1;y++)
        lit+=step(p.z-bias,texture2D(shadowTexture,p.xy+vec2(x,y)/1024.0).r);
      return lit/9.0;
    }
    void main(){
      vec3 c=linear(color);
      if(hologram){
        vec3 N=normalize(n), V=normalize(eye-world);
        float rim=pow(1.0-abs(dot(N,V)),2.4);
        float face=.5+.5*dot(N,normalize(vec3(-.3,.6,.8)));
        c=mix(vec3(.004,.025,.11),vec3(.008,.08,.34),face)
          +vec3(.025,.5,5.0)*rim*mix(.3,.75,edgeDetail);
      } else if(!unlit){
        vec3 N=normalize(n)*(gl_FrontFacing?1.0:-1.0), V=normalize(eye-world);
        vec3 hemi=mix(linear(vec3(.561,.592,.620)),linear(vec3(.965,.969,.973)),N.y*.5+.5)*1.4;
        c=c*.035+c*.92*(vec3(1.3)+hemi)/3.14159265
          +visibility(N)*light(N,V,normalize(vec3(5,-4,7)),vec3(2.6),linear(color))
          +light(N,V,normalize(vec3(-4,3,2)),linear(vec3(.863,.875,.886))*1.1,linear(color));
      }
      if(!unlit) c=aces(c);
      c=mix(c,linear(vec3(.031,.043,.063)),smoothstep(8.0,30.0,depth));
      gl_FragColor=vec4(srgb(c),opacity);
    })";
  if (!shader.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex) ||
      !shader.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment) ||
      !shader.link())
    message = "Robot renderer unavailable: " + shader.log();
  buffer.create();
  edgeBuffer.create();
  depthShader.addShaderFromSourceCode(
      QOpenGLShader::Vertex,
      "#version 120\nattribute vec3 position; uniform mat4 mvp; void "
      "main(){gl_Position=mvp*vec4(position,1.0);}");
  depthShader.addShaderFromSourceCode(
      QOpenGLShader::Fragment,
      "#version 120\nvoid main(){gl_FragColor=vec4(gl_FragCoord.z);}");
  if (!depthShader.link())
    message = "Robot shadows unavailable: " + depthShader.log();
  QOpenGLFramebufferObjectFormat depthFormat;
  depthFormat.setAttachment(QOpenGLFramebufferObject::Depth);
  depthFormat.setInternalTextureFormat(GL_RGBA32F);
  shadowMap =
      std::make_unique<QOpenGLFramebufferObject>(1024, 1024, depthFormat);
  upload = true;
}
void RobotView::paintGL() {
  DiagnosticSpan timing("ui_render_timing", renderTiming);
  glDepthMask(GL_TRUE);
  glDisable(GL_SCISSOR_TEST);
  glDisable(GL_BLEND);
  const QColor ground(theme::viewport);
  glClearColor(ground.redF(), ground.greenF(), ground.blueF(), 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  if (shader.isLinked()) {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glDisable(GL_CULL_FACE);
    QMatrix4x4 projection, view;
    const auto transforms = model.frames(joints);
    timing.mark("kinematics_ms");
    float y = qDegreesToRadians(yaw), p = qDegreesToRadians(pitch);
    QVector3D eye =
        target + distance * QVector3D(std::cos(p) * std::cos(y),
                                      std::cos(p) * std::sin(y), std::sin(p));
    QVector3D look = target, up{0, 0, 1};
    float fov = 46, nearM = .01f, farM = 200;
    QStringList hidden;
    auto vec = [](const QJsonValue &v) {
      const auto a = v.toArray();
      return QVector3D(float(a.at(0).toDouble()), float(a.at(1).toDouble()), float(a.at(2).toDouble()));
    };
    // OLP's mounts: a lens offset in the mount frame; the J6 camera looks at
    // its target frame, the TCP camera along a local axis; up is local too.
    const auto mount = cameras.value(camera == Camera::J6 ? "j6" : "tcp").toObject();
    const QString frame = mount.value("frame").toString();
    bool mounted = false;
    if (camera != Camera::Orbit && transforms.contains(frame)) {
      const auto f = transforms[frame];
      eye = f.map(vec(mount.value("lensOffsetLocalM")));
      up = f.mapVector(vec(mount.value("upLocal"))).normalized();
      if (mount.value("kind") == "target" && transforms.contains(mount.value("targetFrame").toString())) {
        look = transforms[mount.value("targetFrame").toString()].map(QVector3D());
        mounted = (look - eye).lengthSquared() > 1e-10f;
      } else if (mount.value("kind") == "forward") {
        look = eye + f.mapVector(vec(mount.value("forwardLocal"))).normalized();
        mounted = true;
      }
      // Up parallel to the view: OLP falls back to the frame's local Y.
      if (mounted && QVector3D::crossProduct(look - eye, up).lengthSquared() < 1e-8f) up = f.mapVector(QVector3D(0, 1, 0));
      if (mounted) {
        fov = float(mount.value("fovDeg").toDouble(fov));
        nearM = float(mount.value("nearM").toDouble(nearM));
        farM = float(mount.value("farM").toDouble(farM));
        for (const auto &h : mount.value("hideFrames").toArray()) hidden << h.toString();
      } else {
        eye = target + distance * QVector3D(std::cos(p) * std::cos(y), std::cos(p) * std::sin(y), std::sin(p));
        look = target;
        up = {0, 0, 1};
      }
    }
    projection.perspective(fov, float(width()) / std::max(1, height()), nearM, farM);
    view.lookAt(eye, look, up);
    auto vp = projection * view;
    shader.bind();
    shader.setUniformValue("eye", eye);
    // At the Deck's fitted scale, dense CAD edges otherwise merge into white
    // bands. Keep the full mesh, but reveal finer linework as it grows on screen.
    const float pixelsPerMetre = height() / (2.f * std::max(.12f, (eye-look).length()) * std::tan(qDegreesToRadians(fov/2)));
    const float edgeDetail = std::clamp((pixelsPerMetre - 180.f) / 620.f, 0.f, 1.f);
    shader.setUniformValue("edgeDetail", edgeDetail);
    shader.setUniformValue("hologram", false);
    shader.setUniformValue("opacity", 1.f);
    auto bind = [&](QOpenGLBuffer &b) {
      b.bind();
      shader.enableAttributeArray("position");
      shader.enableAttributeArray("normal");
      shader.setAttributeBuffer("position", GL_FLOAT,
                                offsetof(RobotVertex, position), 3,
                                sizeof(RobotVertex));
      shader.setAttributeBuffer("normal", GL_FLOAT,
                                offsetof(RobotVertex, normal), 3,
                                sizeof(RobotVertex));
    };
    bind(buffer);
    if (upload) {
      buffer.allocate(scene.vertices.data(),
                      int(scene.vertices.size() * sizeof(RobotVertex)));
      edgeBuffer.bind();
      edgeBuffer.allocate(scene.edges.data(), int(scene.edges.size() * sizeof(RobotVertex)));
      bind(buffer);
      upload = false;
    }
    timing.mark("camera_upload_ms");
    QMatrix4x4 lightProjection, lightView;
    lightProjection.ortho(-5, 5, -5, 5, .5, 500);
    lightView.lookAt({5, -4, 7}, {}, {0, 1, 0});
    const auto lightVP = lightProjection * lightView;
    if (shadowMap->isValid() && depthShader.isLinked()) {
      shadowMap->bind();
      glViewport(0, 0, 1024, 1024);
      glClearColor(1, 1, 1, 1);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      depthShader.bind();
      buffer.bind();
      depthShader.enableAttributeArray("position");
      depthShader.setAttributeBuffer("position", GL_FLOAT,
                                     offsetof(RobotVertex, position), 3,
                                     sizeof(RobotVertex));
      for (const auto &m : scene.meshes) {
        if (!transforms.contains(m.link))
          continue;
        depthShader.setUniformValue("mvp",
                                    lightVP * transforms[m.link] * m.origin);
        glDrawArrays(GL_TRIANGLES, m.first, m.count);
      }
      depthShader.release();
      glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebufferObject());
      glViewport(0, 0, qRound(width() * devicePixelRatioF()),
                 qRound(height() * devicePixelRatioF()));
      shader.bind();
      bind(buffer);
    }
    shader.setUniformValue("lightVP", lightVP);
    shader.setUniformValue("shadowTexture", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, shadowMap->texture());
    shader.setUniformValue("unlit", false);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.f, 1.f);
    for (const auto &m : scene.meshes) {
      if (!transforms.contains(m.link) || hidden.contains(m.link))
        continue;
      auto t = transforms[m.link] * m.origin;
      shader.setUniformValue("hologram", !m.pedestal);
      shader.setUniformValue("mvp", vp * t);
      shader.setUniformValue("modelMatrix", t);
      shader.setUniformValue("modelView", view * t);
      shader.setUniformValue("normals", t.normalMatrix());
      shader.setUniformValue(
          "color",
          QVector3D(m.color.redF(), m.color.greenF(), m.color.blueF()) *
              (live ? 1.f : .9f));
      glDrawArrays(GL_TRIANGLES, m.first, m.count);
    }
    glDisable(GL_POLYGON_OFFSET_FILL);
    shader.setUniformValue("hologram", false);
    shader.setUniformValue("unlit", true);
    bind(edgeBuffer);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glDepthFunc(GL_LEQUAL);
    for (const auto &m : scene.meshes) {
      if (!transforms.contains(m.link) || hidden.contains(m.link)) continue;
      const auto t = transforms[m.link] * m.origin;
      shader.setUniformValue("mvp", vp * t);
      shader.setUniformValue("modelMatrix", t);
      shader.setUniformValue("modelView", view * t);
      shader.setUniformValue("normals", t.normalMatrix());
      if (!m.pedestal) {
        glLineWidth(1);
        shader.setUniformValue("color", QVector3D(.14f, .42f, .85f));
        shader.setUniformValue("opacity", .02f + .08f * edgeDetail);
        glDrawArrays(GL_LINES, m.detailFirst, m.detailCount);
        glLineWidth(2);
        shader.setUniformValue("color", QVector3D(.2f, .42f, 1.f));
        shader.setUniformValue("opacity", .03f + .10f * edgeDetail);
        glDrawArrays(GL_LINES, m.edgeFirst, m.edgeCount);
      }
      glLineWidth(1);
      shader.setUniformValue("color", m.pedestal ? QVector3D(.4f, .48f, .56f) : QVector3D(.36f, .58f, 1.f) + QVector3D(.26f, .16f, 0.f) * edgeDetail);
      shader.setUniformValue("opacity", m.pedestal ? .5f : .34f + .5f * edgeDetail);
      glDrawArrays(GL_LINES, m.edgeFirst, m.edgeCount);
    }
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    shader.setUniformValue("opacity", 1.f);
    timing.mark("robot_ms");
    std::vector<RobotVertex> lines;
    for (int i = -24; i <= 24; ++i) {
      float v = i * .25f;
      for (auto point : {QVector3D(v, -6, -.002), QVector3D(v, 6, -.002),
                         QVector3D(-6, v, -.002), QVector3D(6, v, -.002)})
        lines.push_back({point, {0, 0, 1}});
    }
    QOpenGLBuffer overlay;
    overlay.create();
    bind(overlay);
    overlay.allocate(lines.data(), int(lines.size() * sizeof(RobotVertex)));
    shader.setUniformValue("mvp", vp);
    shader.setUniformValue("modelMatrix", QMatrix4x4{});
    shader.setUniformValue("modelView", view);
    shader.setUniformValue("normals", QMatrix4x4{}.normalMatrix());
    shader.setUniformValue("unlit", true);
    const QColor grid(theme::viewportGrid);
    shader.setUniformValue("color", QVector3D(grid.redF(), grid.greenF(), grid.blueF()));
    glLineWidth(1);
    glDrawArrays(GL_LINES, 0, lines.size());
    shader.setUniformValue("color",
                           QVector3D(143.f / 255, 151.f / 255, 158.f / 255));
    glDrawArrays(GL_LINES, 24 * 4, 4);
    // OLP's palettes: the authored path in value (travel) and ok (weld), the
    // planned TCP path in accent (travel) and ok (weld); a faded path is
    // mixed toward the viewport, as OLP's 30 % opacity reads over its stage.
    const QColor stage(theme::viewport);
    auto rgb = [&](const char *token, float opacity = 1) {
      const QColor c(token);
      const QVector3D ink(c.redF(), c.greenF(), c.blueF()), under(stage.redF(), stage.greenF(), stage.blueF());
      return under + (ink - under) * opacity;
    };
    auto strip = [&](const PathRun &run, const QVector3D &color) {
      lines.clear();
      for (auto v : run.points)
        lines.push_back({v, {0, 0, 1}});
      overlay.allocate(lines.data(), int(lines.size() * sizeof(RobotVertex)));
      shader.setUniformValue("color", color);
      glDrawArrays(GL_LINE_STRIP, 0, lines.size());
    };
    glDisable(GL_DEPTH_TEST);
    glLineWidth(2);
    const float fade = authoredFocused ? .3f : 1.f;
    for (const auto &run : authored)
      strip(run, rgb(run.weld ? theme::ok : theme::value, fade));
    glLineWidth(3);
    for (const auto &run : planned)
      strip(run, rgb(run.weld ? theme::ok : theme::accent));
    glEnable(GL_DEPTH_TEST);
    // Live TCP axes use the same metre scale and axis colours as OLP.
    if (live && transforms.contains("tool0")) {
      const auto tcp = transforms["tool0"];
      for (int axis = 0; axis < 3; ++axis) {
        QVector3D end;
        end[axis] = .1;
        lines = {{tcp.map(QVector3D{}), {}}, {tcp.map(end), {}}};
        overlay.allocate(lines.data(), int(lines.size() * sizeof(RobotVertex)));
        const QVector3D colors[] = {{1, .35, 0}, {.7, .85, 0}, {0, .65, 1}};
        shader.setUniformValue("color", colors[axis]);
        glDrawArrays(GL_LINES, 0, 2);
      }
    }
    overlay.destroy();
    shader.release();
    timing.mark("paths_ms");
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::TextAntialiasing);
    auto onScreen = [&](const QVector3D &point, QPointF &at) {
      const auto clip = vp * QVector4D(point, 1);
      if (clip.w() <= 0) return false;
      const auto ndc = clip.toVector3DAffine();
      at = QPointF((ndc.x() + 1) * width() / 2, (1 - ndc.y()) * height() / 2);
      return true;
    };
    // A dot on every spreadsheet target (OLP's 8 px points), then the row
    // labels OLP keeps: endpoints and weld boundaries, or the selected row's
    // neighbourhood.
    auto ink = [&](const char *token, float opacity) {
      const auto c = rgb(token, opacity);
      return QColor::fromRgbF(c.x(), c.y(), c.z());
    };
    painter.setPen(Qt::NoPen);
    for (const auto &mark : targets) {
      QPointF at;
      if (!onScreen(mark.at, at)) continue;
      painter.setBrush(ink(mark.weld ? theme::ok : theme::value, fade));
      painter.drawEllipse(at, 4, 4);
    }
    painter.setFont(QFont("DejaVu Sans Mono", 9, QFont::Bold));
    const QFontMetrics labelMetrics(painter.font());
    std::vector<QRectF> placed;
    for (const auto &mark : labels) {
      QPointF at;
      if (!onScreen(mark.at, at)) continue;
      const QColor color(mark.emphasis ? theme::accent : mark.weld ? theme::ok : theme::value);
      QRectF box(at + QPointF(8, -22), QSizeF(labelMetrics.horizontalAdvance(mark.text) + 10, 17));
      // Close targets would stack their labels: step each one clear of those placed.
      for (bool clash = true; clash;) {
        clash = false;
        for (const auto &other : placed)
          if (other.intersects(box)) {
            box.moveTop(other.bottom() + 2);
            clash = true;
          }
      }
      placed.push_back(box);
      painter.setPen(QPen(color, mark.emphasis ? 2 : 1));
      painter.setBrush(QColor(theme::chrome));
      painter.drawRoundedRect(box, 3, 3);
      painter.setPen(QColor(mark.emphasis ? theme::accentText : theme::fg));
      painter.drawText(box, Qt::AlignCenter, mark.text);
    }
    if (camera == Camera::Tcp) {
      // OLP's TCP pane marks the tool point with a small orange cross.
      const QPointF c(width() / 2.0, height() / 2.0);
      painter.setPen(QPen(QColor("#f08c2a"), 1.5));
      painter.drawLine(c - QPointF(9, 0), c + QPointF(9, 0));
      painter.drawLine(c - QPointF(0, 9), c + QPointF(0, 9));
    }
    const QRectF axes(width() - 108, 12, 96, 96);
    painter.setPen(QColor(theme::lineStrong));
    painter.setBrush(QColor(theme::chrome));
    painter.drawRoundedRect(axes, 3, 3);
    painter.setFont(QFont("DejaVu Sans Mono", 8));
    painter.drawText(axes.adjusted(7, 5, -4, -4), Qt::AlignTop, "WORLD XYZ");
    const QPointF center = axes.center() + QPointF(0, 9);
    const QColor colors[] = {QColor("#a82c30"), QColor("#00813e"),
                             QColor("#126bad")};
    // Draw far axes first so the indicator follows the orbit camera.
    std::array<int, 3> order{0, 1, 2};
    auto direction = [&](int axis) {
      QVector3D v;
      v[axis] = 1;
      return view.mapVector(v);
    };
    std::sort(order.begin(), order.end(), [&](int a, int b) {
      return direction(a).z() < direction(b).z();
    });
    for (int axis : order) {
      const auto d = direction(axis);
      const auto end = center + QPointF(d.x() * 29, -d.y() * 29);
      painter.setPen(QPen(colors[axis], 2));
      painter.drawLine(center, end);
      painter.setBrush(QColor(theme::gizmo));
      painter.drawEllipse(end, 8, 8);
      painter.setPen(Qt::white);
      painter.drawText(QRectF(end - QPointF(8, 8), QSizeF(16, 16)),
                       Qt::AlignCenter, QString(QString("XYZ")[axis]));
    }
  }
  QPainter painter(this);
  painter.setPen(QColor(theme::dim));
  painter.setFont(QFont("DejaVu Sans", 10));
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(QColor(theme::lineStrong));
  painter.setBrush(QColor(theme::chrome));
  const QString badge = camera == Camera::Tcp ? "TCP CAMERA" : camera == Camera::J6 ? "J6 CAMERA" : "ORBIT";
  const QRectF badgeRect(12, 12, camera == Camera::Orbit ? 60 : 100, 23);
  painter.drawRoundedRect(badgeRect, 3, 3);
  painter.setPen(QColor(theme::dim));
  painter.setFont(QFont("DejaVu Sans Mono", 9, QFont::Bold));
  painter.drawText(badgeRect, Qt::AlignCenter, badge);
  painter.drawText(QRect(16, 40, width() - 32, height() - 65),
                   Qt::AlignCenter | Qt::TextWordWrap, message);
  painter.drawText(16, height() - 15,
                   live ? "Measured pose · blue TCP history"
                        : "Position unavailable · model reference only");
}
void RobotView::mousePressEvent(QMouseEvent *e) { pointer = e->pos(); }
void RobotView::mouseMoveEvent(QMouseEvent *e) {
  if (!touchGesture.active() && camera == Camera::Orbit && (e->buttons() & Qt::LeftButton)) {
    auto d = e->pos() - pointer;
    yaw -= d.x() * .35f;
    pitch = std::clamp(pitch + d.y() * .35f, 5.f, 85.f);
    pointer = e->pos();
    update();
  }
}
void RobotView::mouseDoubleClickEvent(QMouseEvent *) { home(); }
void RobotView::wheelEvent(QWheelEvent *e) {
  zoomAt(e->position(), std::exp(-e->angleDelta().y() * .001f));
}
bool RobotView::event(QEvent *e) {
  if (e->type() != QEvent::TouchBegin && e->type() != QEvent::TouchUpdate && e->type() != QEvent::TouchEnd &&
      e->type() != QEvent::TouchCancel)
    return QOpenGLWidget::event(e);
  const auto *touch = static_cast<QTouchEvent *>(e);
  const auto change = touchGesture.update(e->type(), touch->touchPoints());
  if (touch->touchPoints().size() >= 2) lastTap.invalidate();
  if (change.kind == OrbitGesture::Change::PanZoom) {
    // Zoom at the OLD midpoint, then pan using the new scale. The point that
    // was under the old midpoint now stays under the new one, without drift.
    zoomAt(change.anchor, change.scale);
    panBy(change.delta);
    lastTap.invalidate();
  } else if (change.kind == OrbitGesture::Change::Orbit) {
    if (camera == Camera::Orbit) {
      yaw -= float(change.delta.x()) * .35f;
      pitch = std::clamp(pitch + float(change.delta.y()) * .35f, 5.f, 85.f);
      update();
    }
  } else if (change.kind == OrbitGesture::Change::Tap) {
    if (lastTap.isValid() && lastTap.elapsed() < 400 && QLineF(lastTapAt, change.anchor).length() < 40) {
      home();
      lastTap.invalidate();
    } else {
      lastTap.start();
      lastTapAt = change.anchor;
    }
  } else if (e->type() == QEvent::TouchCancel) {
    lastTap.invalidate();
  }
  e->accept();
  return true;
}
