#include "current_contract_fixture.hpp"
// The screen holds still and one record press records one waypoint: the real
// Window, offscreen, against a fake OLP server answering with cell A's own
// bodies (tests/fixtures/example-live, read from example-relay on 2026-10-11) and a
// program store that saves, long-polls and lists two browser tabs beside the
// Deck, as the cell did that night.
//
// 2026-10-11, deck-b on cell A: the owner saw the Deck "flicker
// occasionally" and needed two presses to record. The header's new program
// store label ("EXAMPLE-RELAY · R14 · BROWSER (CHROME ON MACOS) VIEWING") set the
// window's minimum width to 1688, then 1543 (WM_NORMAL_HINTS): Qt resized the
// window past the 1280 x 800 screen at every label change (every save, every
// presence change) and gamescope put it back. Pages also re-polished their
// overlays at every poll, and a press dropped for focus or an in-flight
// capture said nothing.
// usage: QT_QPA_PLATFORM=offscreen steady-check <olp-core.js> <robots dir> <fixtures dir>
#include "jog_live.hpp"
#include "jogend_overlay.hpp"
#include "control_overlay.hpp"
#include "maintenance_overlay.hpp"
#include "program_picker.hpp"
#include "window.hpp"
#include "in_window_dialog.hpp"
#include "workspace_split.hpp"
#include "theme.hpp"
#include <QScrollArea>
#include <QScrollBar>
#include <QMenu>
#include <QSettings>
#include <QMouseEvent>
#include <QApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLayout>
#include <QPointer>
#include <QPushButton>
#include <QRegularExpression>
#include <QStandardPaths>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>
#include <QUrlQuery>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <memory>

#define CHECK(condition)                                                                 \
  do {                                                                                   \
    if (!(condition)) {                                                                  \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
      std::exit(1);                                                                      \
    }                                                                                    \
  } while (false)

namespace {
QJsonObject readObject(const QString &path) {
  QFile f(path);
  CHECK(f.open(QIODevice::ReadOnly));
  return QJsonDocument::fromJson(currentContractFixture(f.readAll())).object();
}
QByteArray compact(const QJsonObject &o) { return QJsonDocument(o).toJson(QJsonDocument::Compact); }

// The OLP routes the pendant reads, answered from the fixtures. The status's
// native clock follows the wall clock, as rt-core's does; the store saves,
// answers a held change feed when its version moves, and counts requests.
struct FakeOlp {
  QTcpServer server;
  QString dir;
  QJsonObject status, store, program, head;
  QElapsedTimer clock;
  int revision = 0, version = 100, captureDelayMs = 0;
  QMap<QString, int> counts;
  QList<int> savedNodes;
  struct Held { QPointer<QTcpSocket> socket; qint64 until; };
  QList<Held> held;
  QTimer heldTimer;

  void reply(QPointer<QTcpSocket> socket, int code, const QByteArray &body) {
    if (!socket) return;
    socket->write("HTTP/1.1 " + QByteArray::number(code) + (code == 200 ? " OK" : " Conflict") +
                  "\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: " + QByteArray::number(body.size()) +
                  "\r\n\r\n" + body);
    socket->disconnectFromHost();
  }
  QByteArray file(const QString &name) {
    QFile f(dir + "/" + name);
    CHECK(f.open(QIODevice::ReadOnly));
    return currentContractFixture(f.readAll());
  }
  QJsonObject storeView() {
    QJsonObject s = store;
    s["version"] = version;
    s["programs"] = QJsonArray{head};
    s["watched"] = head;
    return s;
  }
  QByteArray statusBody() {
    QJsonObject s = status, rt = s["rt_core"].toObject(), st = rt["status"].toObject();
    const double ns = 423765371403217.0 + double(clock.nsecsElapsed());
    st["time_ns"] = ns;
    QJsonArray axes = st["axes"].toArray();
    for (int i = 0; i < axes.size(); ++i) {
      QJsonObject a = axes[i].toObject();
      a["time_ns"] = ns;
      axes[i] = a;
    }
    st["axes"] = axes;
    rt["status"] = st;
    s["rt_core"] = rt;
    return compact(s);
  }
  void handle(QTcpSocket *socket, const QByteArray &method, const QByteArray &target, const QByteArray &body) {
    const QUrl url("http://olp" + QString::fromLatin1(target));
    const QString path = url.path().remove("/api/offline-programming/v1");
    counts[method + " " + path]++;
    if (path == "/targets") return reply(socket, 200, file("targets.json"));
    if (path == "/targets/select") {
      const auto first = readObject(dir + "/targets.json")["targets"].toArray().first().toObject();
      return reply(socket, 200, compact({{"selected", QJsonObject{{"cell_id", first["cell_id"]}}}, {"target", first}}));
    }
    if (path == "/robots") return reply(socket, 200, file("robots.json"));
    if (path == "/robots/capture-model") {
      QPointer<QTcpSocket> s(socket);
      const QByteArray answer = file("capture-model.json");
      QTimer::singleShot(captureDelayMs, &server, [this, s, answer] { reply(s, 200, answer); });
      return;
    }
    if (path == "/dense-execution/capabilities") return reply(socket, 200, file("caps.json"));
    if (path == "/dense-execution/status") return reply(socket, 200, statusBody());
    if (path == "/dense-execution/cell") return reply(socket, 200, file("cell.json"));
    if (path == "/maintenance") return reply(socket, 200, file("maintenance-idle.json"));
    if (path == "/health") return reply(socket, 200, file("health.json"));
    if (path == "/cell-programs/changes") {
      const QUrlQuery q(url);
      if (q.queryItemValue("since").toInt() == version && q.queryItemValue("timeout_ms").toInt() > 0 &&
          q.queryItemValue("instance") == store["store"].toObject()["instance"].toString()) {
        held << Held{socket, clock.elapsed() + std::min(q.queryItemValue("timeout_ms").toInt(), 5000)};
        return;
      }
      return reply(socket, 200, compact(storeView()));
    }
    if (path.endsWith("/save")) {
      const auto request = QJsonDocument::fromJson(body).object();
      if (request["base_revision"].toInt() != revision)
        return reply(socket, 409, compact({{"error", "program_conflict"}, {"current", head}}));
      head["revision"] = ++revision;
      program["document"] = request["document"];
      program["program"] = head;
      savedNodes << request["document"].toObject()["nodes"].toArray().size();
      reply(socket, 200, compact({{"ok", true}, {"program", head}}));
      ++version;
      for (auto &h : held) reply(h.socket, 200, compact(storeView()));
      held.clear();
      return;
    }
    if (path.startsWith("/cell-programs/programs/")) return reply(socket, 200, compact(program));
    reply(socket, 200, "{}");
  }
  explicit FakeOlp(const QString &fixtures) : dir(fixtures) {
    clock.start();
    status = readObject(dir + "/status.json");
    store = readObject(dir + "/store.json");
    program = readObject(dir + "/program.json");
    head = program["program"].toObject();
    revision = head["revision"].toInt();
    QObject::connect(&heldTimer, &QTimer::timeout, &server, [this] {
      for (int i = held.size() - 1; i >= 0; --i)
        if (!held[i].socket || clock.elapsed() >= held[i].until) {
          reply(held[i].socket, 200, compact(storeView()));
          held.removeAt(i);
        }
    });
    heldTimer.start(50);
    CHECK(server.listen(QHostAddress::LocalHost));
    QObject::connect(&server, &QTcpServer::newConnection, &server, [this] {
      while (auto *socket = server.nextPendingConnection()) {
        auto buffer = std::make_shared<QByteArray>();
        QObject::connect(socket, &QTcpSocket::readyRead, socket, [this, socket, buffer] {
          *buffer += socket->readAll();
          const int end = buffer->indexOf("\r\n\r\n");
          if (end < 0) return;
          const QList<QByteArray> lines = buffer->left(end).split('\n');
          int length = 0;
          for (const auto &raw : lines.mid(1))
            if (raw.trimmed().toLower().startsWith("content-length:")) length = raw.trimmed().mid(15).trimmed().toInt();
          if (buffer->size() < end + 4 + length) return;
          const QList<QByteArray> first = lines.value(0).trimmed().split(' ');
          const QByteArray request = buffer->mid(end + 4, length);
          buffer->clear();
          handle(socket, first.value(0), first.value(1), request);
        });
        QObject::connect(socket, &QTcpSocket::disconnected, socket, &QObject::deleteLater);
      }
    });
  }
  QString origin() const { return QString("http://127.0.0.1:%1").arg(server.serverPort()); }
};

// Every Resize, Show, Hide and StyleChange inside one widget tree.
struct Events : QObject {
  QWidget *root = nullptr;
  QMap<QString, int> seen;
  QMap<QWidget *, QMap<QString, int>> byWidget;
  bool eventFilter(QObject *object, QEvent *event) override {
    auto *w = qobject_cast<QWidget *>(object);
    if (!w || !root || (w != root && !root->isAncestorOf(w))) return false;
    const char *kind = event->type() == QEvent::Resize ? "resize" : event->type() == QEvent::Show ? "show"
                       : event->type() == QEvent::Hide ? "hide" : event->type() == QEvent::StyleChange ? "style" : nullptr;
    if (kind) {
      seen[QString("%1 %2#%3").arg(kind, w->metaObject()->className(), w->objectName())]++;
      byWidget[w][kind]++;
    }
    return false;
  }
  int count(const QString &kind, QWidget *only = nullptr) const {
    // Anonymous child QWidgets share the root's class/objectName. Count an
    // exact widget by identity; a child form resizing is not a window resize.
    if (only) return byWidget.value(only).value(kind);
    int n = 0;
    for (auto it = seen.begin(); it != seen.end(); ++it)
      if (it.key().startsWith(kind + " "))
        n += it.value();
    return n;
  }
  void print(const char *what) const {
    for (auto it = seen.begin(); it != seen.end(); ++it) std::fprintf(stderr, "  %s: %s x%d\n", what, qPrintable(it.key()), it.value());
  }
};

void spin(int ms) {
  QElapsedTimer t;
  t.start();
  while (t.elapsed() < ms) QApplication::processEvents(QEventLoop::AllEvents, 10);
}
bool waitFor(const std::function<bool()> &done, int ms = 5000) {
  QElapsedTimer t;
  t.start();
  while (t.elapsed() < ms) {
    if (done()) return true;
    QApplication::processEvents(QEventLoop::AllEvents, 10);
  }
  return done();
}
QLabel *statusLine(QWidget &window) {
  return window.findChild<QLabel *>("notificationStatus");
}
QPushButton *buttonNamed(QWidget &window, const QString &text) {
  for (auto *b : window.findChildren<QPushButton *>())
    if (b->isVisible() && b->text().contains(text, Qt::CaseInsensitive)) return b;
  return nullptr;
}
bool inside(QWidget &window, QWidget *w) { return w && window.rect().contains(QRect(w->mapTo(&window, QPoint()), w->size())); }

// The same view drawn twice: the second draw re-polishes, shows and hides nothing.
template <typename Draw> void drawsOnce(QWidget &widget, Draw draw) {
  Events events;
  events.root = &widget;
  draw();
  QApplication::processEvents();
  qApp->installEventFilter(&events);
  draw();
  QApplication::processEvents();
  qApp->removeEventFilter(&events);
  if (!events.seen.isEmpty()) events.print(widget.metaObject()->className());
  CHECK(events.seen.isEmpty());
}
} // namespace

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  QStandardPaths::setTestModeEnabled(true);
  QDir(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation)).removeRecursively();
  CHECK(argc == 4);
  QSettings().remove("jog/workspaceSplit");
  QSettings().remove("jog/workspaceCollapsed");
  QSettings().setValue("jog/cartesian", true);
  QSettings().setValue("jog/linear", 400.);
  QSettings().setValue("jog/angular", 30.);
  QSettings().remove("jog/linearStep");
  QSettings().remove("jog/angularStep");
  theme::setDark(true);
  app.setStyleSheet(theme::styleSheet());

  // Overlays redrawn at every poll with an unchanged view change nothing.
  {
    QWidget host;
    host.resize(1280, 800);
    host.show();
    JogLiveStrip strip(&host);
    const QJsonObject live{{"visible", true}, {"active", true}, {"tone", "caution"}, {"kicker", "CARTESIAN +X · BASE"},
                           {"speed", "62 mm/s"}, {"speedFraction", 0.62}, {"text", "Slowing near the wrist singularity"},
                           {"meter", QJsonObject{{"visible", true}, {"closeness", 0.4}, {"label", "J5 12°"}}}};
    drawsOnce(strip, [&] { strip.render(live); });
    JogEndOverlay toast(&host);
    const QJsonObject end{{"active", true}, {"tone", "toast"}, {"kicker", "JOG STOPPED"}, {"text", "Stopped at the joint limit"},
                          {"expanded", false}, {"details", QJsonArray{"J2 at +1.90 rad"}}};
    drawsOnce(toast, [&] { toast.render(end); });
    ControlOverlay control(&host);
    const QJsonObject other{{"active", true}, {"kicker", "IN USE"}, {"title", "Another controller is using the robot"},
                            {"who", "Browser (Chrome on macOS)"}, {"what", "jogging"}, {"since", "since 21:32"}, {"note", ""},
                            {"stopNote", ""}, {"stopAvailable", true}, {"canTake", false}, {"summary", "Browser is jogging"},
                            {"expanded", true}};
    drawsOnce(control, [&] { control.render(other); });
    MaintenanceOverlay notice(&host);
    const QJsonObject update{{"blocking", true}, {"title", "Update in progress"}, {"reason", "release 395ebbe2 (olp tier)"},
                             {"phase", "Step 1/2 · Build and test gate"}, {"countdownLabel", "About"}, {"countdown", "7 min left"},
                             {"elapsed", "0:14"}, {"source", "realtime-core release"}, {"progress", 0.1},
                             {"progressText", "10 %"}, {"stepLine", "Step 1/2 · Build and test gate"}, {"suiteLine", ""}};
    drawsOnce(notice, [&] { notice.render(update, true, QJsonArray{}); });
  }

  FakeOlp olp(argv[3]);
  // The startup question (the cell's programs): open the cell's current one.
  QTimer picker;
  QObject::connect(&picker, &QTimer::timeout, [] {
    if (auto *p = dynamic_cast<ProgramPicker *>(InWindowDialog::activeDialog())) p->accept();
  });
  picker.start(100);
  Window window({olp.origin(), argv[1], argv[2], QDir::tempPath(), "example-arm-1420"});
  window.resize(1280, 800);
  window.show();
  QApplication::setActiveWindow(&window);
  CHECK(waitFor([&] { return olp.counts.value("POST /cell-programs/current") > 0; }));
  window.showPage(3);
  QPushButton *connectButton = nullptr;
  CHECK(waitFor([&] { return (connectButton = buttonNamed(window, "EXAMPLE RELAY")) != nullptr; }));
  connectButton->click();
  CHECK(waitFor([&] { return olp.counts.value("GET /dense-execution/status") > 4; }));
  spin(100);
  CHECK(window.size() == QSize(1280, 800));
  CHECK(inside(window, statusLine(window)));
  CHECK(!window.findChild<QPushButton *>("statusDetailsButton")->isVisible());
  window.showPage(0);
  QApplication::setActiveWindow(&window);
  spin(1500);

  // The window is the screen: as gamescope keeps it, 1280 x 800, with the
  // store label as long as cell A's (two browser tabs and this Deck) and STOP
  // and Arm on it.
  CHECK(window.size() == QSize(1280, 800));
  CHECK(window.minimumWidth() <= 1280 && window.minimumHeight() <= 800);
  // The bars whose text changes at run time fit the screen whatever they say.
  for (auto *bar : window.findChildren<QWidget *>(QRegularExpression("^(chrome|status)$"))) CHECK(bar->minimumSizeHint().width() <= 1280);
  CHECK(inside(window, buttonNamed(window, "STOP")));
  CHECK(inside(window, buttonNamed(window, "RECORD WAYPOINT")));

  // The Jog editor and its teaching actions move together, leaving the camera
  // full height. Collapsing Program hides its actions; controller shortcuts remain.
  auto *split = dynamic_cast<WorkspaceSplit *>(window.findChild<QWidget *>("jogWorkspace"));
  auto *sheet = window.findChild<QTableWidget *>("jogProgramTable");
  CHECK(split && sheet && sheet->isVisible());
  CHECK(window.findChild<QStackedWidget *>()->count() == 4);
  CHECK(!sheet->isColumnHidden(5));
  CHECK(sheet->item(0,5)->text().endsWith("%") || sheet->item(0,5)->text().endsWith("mm/s"));
  CHECK(window.findChild<QWidget *>("workspaceDivider")->width() == 60);
  auto screenshot = [&](const QString &name) {
    const QString dir = qEnvironmentVariable("ROSIE_LAYOUT_SNAPSHOTS");
    if (!dir.isEmpty()) { QDir().mkpath(dir); CHECK(window.grab().save(dir + "/" + name + ".png")); }
  };
  auto fits = [&] {
    CHECK(window.size() == QSize(1280, 800));
    CHECK(inside(window, buttonNamed(window, "STOP")));
    auto *zero = window.findChild<QPushButton *>("jogZeroAll");
    CHECK(zero && zero->isVisible() && inside(window, zero));
    CHECK(zero->fontMetrics().horizontalAdvance(zero->text()) + 20 <= zero->width());
    if (sheet->isVisible()) {
      CHECK(inside(window, window.findChild<QPushButton *>("jogRecord")));
      CHECK(inside(window, sheet));
      CHECK(sheet->viewport()->height() >= sheet->rowCount() * 40);
      CHECK(inside(window, window.findChild<QPushButton *>("jogInsert")));
      auto *remove = window.findChild<QPushButton *>("jogDelete");
      CHECK(remove && remove->isVisible() && inside(window, remove));
      CHECK(remove->fontMetrics().horizontalAdvance(remove->text().split('\n').first()) + 8 <= remove->width());
    }
  };
  for (double fraction : {.24, .5, .76}) {
    split->setSplit(fraction); spin(100); fits();
    screenshot(QString("split-%1").arg(int(fraction * 100)));
  }
  split->setSplit(.43);
  for (int side : {-1, 1}) {
    split->collapse(side); spin(100); fits();
    CHECK(sheet->isVisible() == (side < 0));
    screenshot(side < 0 ? "camera-collapsed" : "program-collapsed");
    auto *handle = window.findChild<QWidget *>("workspaceDivider");
    const QPoint at = handle->rect().center(), global = handle->mapToGlobal(at);
    QMouseEvent press(QEvent::MouseButtonPress, at, global, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QMouseEvent release(QEvent::MouseButtonRelease, at, global, Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
    QApplication::sendEvent(handle, &press); QApplication::sendEvent(handle, &release);
    CHECK(split->collapsedSide() == 0 && std::abs(split->splitFraction() - .43) < .001);
  }
  {
    QWidget left, right;
    WorkspaceSplit restored(&left, &right);
    CHECK(std::abs(restored.splitFraction() - .43) < .001);
    // Children are stack-owned in this check, so release them before the splitter.
    left.setParent(nullptr); right.setParent(nullptr);
  }
  split->equalize(); spin(100); fits();
  // A speed entered in mm/s or degrees/s must never become a step distance.
  JogPage *jogPage = nullptr;
  for (auto *w : window.findChildren<QWidget *>()) if (auto *j = dynamic_cast<JogPage *>(w)) jogPage = j;
  CHECK(jogPage);
  window.findChild<QPushButton *>("jogModeHold")->click();
  CHECK(jogPage->linear() == 400. && jogPage->angular() == 30.);
  window.findChild<QPushButton *>("jogModeStep")->click();
  CHECK(jogPage->linear() == 1. && jogPage->angular() == 1.);
  window.findChild<QPushButton *>("jogModeHold")->click();
  CHECK(jogPage->linear() == 400. && jogPage->angular() == 30.);

  QTimer::singleShot(100, [&] {
    auto *dialog = qobject_cast<QDialog *>(InWindowDialog::activeDialog());
    CHECK(dialog && dialog->objectName() == "touchJogControls");
    CHECK(dialog->width() <= 1280 && dialog->height() <= 800);
    const QString dir = qEnvironmentVariable("ROSIE_LAYOUT_SNAPSHOTS");
    if (!dir.isEmpty()) CHECK(dialog->grab().save(dir + "/touch-controls.png"));
    dialog->reject();
  });
  window.findChild<QPushButton *>("touchJogButton")->click();
  QTimer::singleShot(100, [&] {
    auto *dialog = qobject_cast<QDialog *>(InWindowDialog::activeDialog());
    CHECK(dialog && dialog->objectName() == "jogSpeedEditor");
    CHECK(dialog->width() <= 1280 && dialog->height() <= 800);
    const QString dir = qEnvironmentVariable("ROSIE_LAYOUT_SNAPSHOTS");
    if (!dir.isEmpty()) CHECK(dialog->grab().save(dir + "/speed-presets.png"));
    dialog->reject();
  });
  window.findChild<QPushButton *>("jogSpeedSettings")->click();
  QApplication::setActiveWindow(&window); spin(100);
  // Approved hierarchy: title and Save above the table; edit/record below it,
  // all inside Program. Details remains visible outside the resizable panels.
  auto *record = window.findChild<QPushButton *>("jogRecord");
  auto *save = window.findChild<QPushButton *>("jogSave");
  auto *edit = window.findChild<QPushButton *>("jogEdit");
  auto *title = window.findChild<QPushButton *>("compactProgramTitle");
  CHECK(title->isVisible() && save->isVisible());
  CHECK(save->mapTo(&window, QPoint()).y() < sheet->mapTo(&window, QPoint()).y());
  CHECK(edit->mapTo(&window, QPoint()).y() >= sheet->mapTo(&window, QPoint(0, sheet->height())).y());
  CHECK(record->mapTo(&window, QPoint()).x() >= sheet->mapTo(&window, QPoint()).x());
  CHECK(window.findChild<QLabel *>("observedArmStatus")->text().contains("DISARMED"));
  CHECK(window.findChild<QLabel *>("observedArmStatus")->text().contains("lease released"));
  // Keep eight full 40 px rows even with the controller-offline warning.
  // With neutral controller input the normal workspace fits nine rows.
  CHECK(window.findChild<QLabel *>("armBlocked")->isVisible());
  CHECK(sheet->viewport()->height() >= 320);
  CHECK(sheet->rowCount() >= 8);
  window.controllerAxes({});
  // Injection changes pad state; the next asynchronous status refresh paints it.
  CHECK(waitFor([&] { return !window.findChild<QLabel *>("armBlocked")->isVisible(); }, 1500));
  CHECK(waitFor([&] { return sheet->viewport()->height() >= 360 && sheet->rowCount() >= 9; }, 1500));
  screenshot("compact-controller-ready");
  auto *footer = window.findChild<QWidget *>("status");
  CHECK(footer->height() <= 46);
  for (const char *name : {"connectionStatus", "observedArmStatus", "motionStatus"}) {
    auto *label = window.findChild<QLabel *>(name);
    CHECK(label && label->isVisible() && inside(window, label));
    CHECK(label->mapTo(&window, QPoint()).y() >= footer->y());
  }
  for (auto *button : window.findChildren<QPushButton *>()) {
    if (!button->property("teaching").toBool()) continue;
    CHECK(button->height() >= 48);
    CHECK(QFontMetricsF(button->font()).height() + button->property("glyphHeight").toDouble() + 5 <= button->height());
  }
  auto closeDetails = [&] {
    auto *dialog = qobject_cast<QDialog *>(InWindowDialog::activeDialog());
    CHECK(dialog && dialog->objectName() == "statusDetails");
    CHECK(dialog->width() <= 1280 && dialog->height() <= 800);
    const QString dir = qEnvironmentVariable("ROSIE_LAYOUT_SNAPSHOTS");
    if (!dir.isEmpty()) CHECK(dialog->grab().save(dir + "/status-details.png"));
    CHECK(dialog->findChild<QWidget *>("jogLive")->isVisible());
    dialog->reject();
  };
  const QRect beforeLive = split->geometry();
  olp.status["jog_live"] = QJsonObject{{"active", true}, {"axis", "-X"}, {"frame", "base"},
      {"speed_unit", "mm/s"}, {"requested_speed", 100}, {"speed", 62}, {"governing", "cap"},
      {"near", true}, {"limit_kind", "singularity"}, {"limit", "singularity"},
      {"gate_scale", .4}, {"min_gate_scale_ahead", .1}, {"j5_deg", -78},
      {"wrist_singular_deg", -90}, {"wrist_distance_deg", 12},
      {"text", "Limited by wrist singularity"}};
  spin(350);
  CHECK(split->geometry() == beforeLive);
  for (auto *bar : window.findChildren<QWidget *>("jogLive")) CHECK(!bar->isVisible());
  QTimer::singleShot(100, closeDetails);
  window.findChild<QPushButton *>("statusDetailsButton")->click();
  CHECK(split->geometry() == beforeLive);
  olp.status.remove("jog_live");
  QApplication::setActiveWindow(&window); spin(100);
  save->click();
  CHECK(statusLine(window)->text().startsWith("Saved on this Deck"));
  spin(100);
  CHECK(window.findChild<QLabel *>("motionStatus")->text().startsWith("Saved on this Deck"));
  CHECK(split->geometry() == beforeLive);

  // Popup menus are navigation: A must not record behind one; B still stops.
  const int beforeMenuCapture = olp.counts.value("POST /robots/capture-model");
  QMenu menu(&window);
  menu.addAction("Layout options"); menu.popup(window.mapToGlobal(QPoint(600, 300)));
  spin(100);
  window.controllerButton(Controller::A);
  spin(100);
  CHECK(olp.counts.value("POST /robots/capture-model") == beforeMenuCapture);
  menu.close(); QApplication::setActiveWindow(&window); spin(100);

  // Idle on Jog with an unchanging robot: nothing re-polished, shown, hidden or resized.
  {
    Events idle;
    idle.root = &window;
    app.installEventFilter(&idle);
    spin(3000);
    app.removeEventFilter(&idle);
    if (idle.count("style") || idle.count("show") || idle.count("hide") || idle.count("resize", &window)) idle.print("idle");
    CHECK(idle.count("style") == 0 && idle.count("show") == 0 && idle.count("hide") == 0);
    CHECK(idle.count("resize", &window) == 0);
  }

  // One press, focused: exactly one capture and one more waypoint on the
  // cell, and the store label's SAVING -> R<n> never resizes the window.
  const int capturesBefore = olp.counts.value("POST /robots/capture-model");
  const int savesBefore = olp.savedNodes.size();
  Events recording;
  recording.root = &window;
  app.installEventFilter(&recording);
  QTimer review;
  QObject::connect(&review, &QTimer::timeout, [&] {
    if (auto *dialog = qobject_cast<QDialog *>(InWindowDialog::activeDialog()); dialog && dialog->objectName() == "nodeEditor") {
      CHECK(olp.savedNodes.size() == savesBefore);
      CHECK(dialog->width() == 1000 && dialog->height() == 540);
      CHECK(!dialog->isWindow());
      CHECK(buttonNamed(*dialog,"Motion profile")->isChecked());
      CHECK(buttonNamed(*dialog,"Weld start") && buttonNamed(*dialog,"Weld stop"));
      const QString dir = qEnvironmentVariable("ROSIE_LAYOUT_SNAPSHOTS");
      if (!dir.isEmpty()) CHECK(dialog->grab().save(dir + "/record-review.png"));
      dialog->accept();
    }
  });
  review.start(20);
  window.controllerButton(Controller::A);
  CHECK(waitFor([&] { return olp.savedNodes.size() == savesBefore + 1; }));
  review.stop();
  spin(1500);
  app.removeEventFilter(&recording);
  CHECK(olp.counts.value("POST /robots/capture-model") == capturesBefore + 1);
  CHECK(olp.savedNodes.size() == savesBefore + 1);
  const int nodes = olp.savedNodes.last();
  CHECK(statusLine(window)->text() == "Waypoint saved");
  if (recording.count("resize", &window)) recording.print("record");
  CHECK(recording.count("resize", &window) == 0);
  CHECK(window.size() == QSize(1280, 800));

  CHECK(sheet->isVisible());
  screenshot("jog-recorded");
  // Every field group stays in the same bounded dialog, including destination.
  QTimer::singleShot(100, [&] {
    auto *dialog = qobject_cast<QDialog *>(InWindowDialog::activeDialog());
    CHECK(dialog && dialog->objectName() == "nodeEditor");
    const QSize size = dialog->size();
    CHECK(size == QSize(1000, 540));
    const auto tabs = dialog->findChildren<QPushButton *>();
    QStringList names;
    for (auto *tab : tabs) if (tab->property("role") == "tab") names << tab->text();
    for (const auto &name : names) {
      buttonNamed(*dialog, name)->click(); spin(50);
      CHECK(dialog->size() == size);
      auto *scroll = dialog->findChild<QScrollArea *>("nodeFields");
      CHECK(scroll && inside(*dialog, scroll));
      CHECK(inside(*dialog, buttonNamed(*dialog, "Done")));
      const QString dir = qEnvironmentVariable("ROSIE_LAYOUT_SNAPSHOTS");
      if (!dir.isEmpty()) CHECK(dialog->grab().save(dir + "/editor-" + name + ".png"));
    }
    auto *stop = window.findChild<QPushButton *>("headerStop");CHECK(stop&&stop->isVisible());
    const int beforeStop=olp.counts.value("POST /dense-execution/stop");
    const QPoint at=stop->rect().center(), global=stop->mapToGlobal(at);
    QMouseEvent press(QEvent::MouseButtonPress,at,global,Qt::LeftButton,Qt::LeftButton,Qt::NoModifier);
    QMouseEvent release(QEvent::MouseButtonRelease,at,global,Qt::LeftButton,Qt::NoButton,Qt::NoModifier);
    QApplication::sendEvent(stop,&press);QApplication::sendEvent(stop,&release);
    CHECK(waitFor([&]{return olp.counts.value("POST /dense-execution/stop")>beforeStop;}));
    dialog->accept();
  });
  window.findChild<QPushButton *>("jogEdit")->click();
  QApplication::setActiveWindow(&window); spin(100);


  // A double press while capture is in flight skips review and saves exactly once.
  olp.captureDelayMs = 600;
  window.controllerButton(Controller::A);
  spin(100);
  window.controllerButton(Controller::A);
  CHECK(olp.savedNodes.size() == savesBefore + 1);
  CHECK(waitFor([&] { return olp.savedNodes.size() == savesBefore + 2; }));
  spin(1500);
  CHECK(olp.savedNodes.last() == nodes + 1);
  CHECK(olp.counts.value("POST /robots/capture-model") == capturesBefore + 2);
  olp.captureDelayMs = 0;

  // The visible Delete action removes exactly the selected fake waypoint;
  // Undo restores it using the existing program-edit path. No live robot here.
  const auto beforeDelete = olp.program["document"].toObject();
  auto *remove = window.findChild<QPushButton *>("jogDelete");
  CHECK(remove->isEnabled());
  window.controllerButton(Controller::L4);
  CHECK(waitFor([&] { return olp.program["document"].toObject()["nodes"].toArray().size() == nodes; }));
  QTimer::singleShot(100, [&] {
    auto *menu = qobject_cast<QMenu *>(QApplication::activePopupWidget());
    CHECK(menu);
    for (auto *action : menu->actions()) {
      if (action->text() != "Undo") continue;
      CHECK(action->isEnabled());
      action->trigger();
      menu->close();
      return;
    }
    CHECK(false); // Undo belongs in the program-name menu.
  });
  title->click();
  CHECK(waitFor([&] { return olp.program["document"].toObject()["nodes"] == beforeDelete["nodes"]; }));

  // In the background a press records nothing and says why; B still Stops.
  QApplication::setActiveWindow(nullptr);
  window.controllerButton(Controller::A);
  spin(500);
  CHECK(olp.counts.value("POST /robots/capture-model") == capturesBefore + 2);
  CHECK(statusLine(window)->text().startsWith("Controller press ignored: this pendant does not have input focus"));
  window.controllerButton(Controller::B);
  CHECK(waitFor([&] { return olp.counts.value("POST /dense-execution/stop") == 1; }));
  CHECK(window.size() == QSize(1280, 800));
  std::printf("steady-check: ok (%d waypoints on the cell, %d status polls)\n", nodes + 1, olp.counts.value("GET /dense-execution/status"));
  return 0;
}
