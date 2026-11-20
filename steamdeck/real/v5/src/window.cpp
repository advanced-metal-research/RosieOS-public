#include "io_panel.hpp"
#include "diagnostics.hpp"
#include "window.hpp"
#include <QMenu>
#include "workspace_split.hpp"
#include "capture_fk.hpp"
#include "controller_focus.hpp"
#include "program_files.hpp"
#include "program_picker.hpp"
#include "theme.hpp"
#include "widgets.hpp"
#include "glyphs.hpp"
#include "stick_hold.hpp"
#include "jog_live.hpp"
#include <QApplication>
#include <QDateTime>
#include <QDebug>
#include <QDialog>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QKeyEvent>
#include <QLayout>
#include <QLineEdit>
#include <QPainter>
#include <QPainterPath>
#include <QSaveFile>
#include <QSettings>
#include <QStandardPaths>
#include <QUuid>
#include <QVBoxLayout>
#include <QWindow>
#include <cmath>
#include <cstdio>
#include <future>
#include <thread>

namespace {
// The Advanced Metal Research mark, as OLP's title bar draws it
// (main.ts brand-mark, viewBox 975 x 370), in the brand red.
class BrandMark : public QWidget {
public:
  BrandMark() { setFixedSize(62, 24); }

protected:
  void paintEvent(QPaintEvent *) override {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.scale(width() / 975.0, height() / 370.0);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(theme::brand));
    const std::vector<std::vector<QPointF>> shapes{
        {{459.958, 337.987}, {255.11, 0}, {131.807, 0}, {112.273, 30.0028}, {319.11, 370}, {439.116, 370}},
        {{660.958, 337.987}, {456.11, 0}, {332.807, 0}, {313.273, 30.0028}, {520.11, 370}, {640.116, 370}},
        {{860.958, 337.987}, {656.11, 0}, {532.807, 0}, {513.273, 30.0028}, {720.11, 370}, {840.116, 370}},
        {{135.359, 132}, {279.109, 370}, {19.1094, 370}, {0, 337.93}},
        {{838.859, 238}, {695.109, 0}, {955.109, 0}, {974.288, 31.9951}}};
    for (const auto &shape : shapes) p.drawPolygon(shape.data(), int(shape.size()));
  }
};

// JSLOP Xbox's input sampling period; physical Deck latency is unverified.
constexpr int tickMs = 4;
} // namespace


namespace {
// OLP's MAINTENANCE_POLL_MS (execution/maintenance.ts): the owner's 2 s. A poll
// slower than 1.5 s counts as the server not answering (unverified bound).
const QString maintenancePath = "/api/offline-programming/v1/maintenance";
constexpr int maintenancePollMs = 2000, maintenanceTickMs = 500, maintenanceTimeoutMs = 1500;
} // namespace

Window::Window(const Options &options_)
    : options(options_), client(options_.origin), core(options_.bundle), machine(client, core, this),
      context{core, machine, client, [this](const QString &t, bool e) { notify(t, e); }, [this](const QString &m) { record(m); },
              [this] { autosave(); refresh(); }, [this] { return runView(); },
              [this](const QString &digest) { return trajectoryPath(digest); }} {
  setWindowTitle("Rosie / Teach pendant");
  machine.requireArmHold();
  auto *root = new QVBoxLayout(this);
  // The window is the screen: no text or page may set its minimum size, or Qt
  // resizes it past 1280 x 800 and gamescope bounces it back (theme::elidedLabel).
  root->setSizeConstraint(QLayout::SetNoConstraint);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  header = new QWidget;
  header->setObjectName("chrome");
  auto *top = new QHBoxLayout(header);
  top->setContentsMargins(14, 4, 10, 4);
  top->setSpacing(10);
  top->addWidget(new BrandMark);
  auto *titles = new QVBoxLayout;
  titles->setSpacing(0);
  titles->addWidget(theme::label("ROSIE / TEACH PENDANT", "kicker"));
  programName = theme::elidedLabel("", "title");
  titles->addWidget(programName);
  top->addLayout(titles);
  saveState = theme::label("", "kicker");
  saveState->hide();
  syncState = theme::elidedLabel("", "kicker");
  syncState->hide();
  top->addSpacing(8);
  cellState = theme::elidedLabel("", "value");
  top->addWidget(cellState, 1);
  armPill = theme::button("DISARMED", {}, "pill");
  armPill->setFixedWidth(246);
  connect(armPill, &QPushButton::clicked, this, [this] {
    if (machine.programArmPermitted() || machine.view()["flow"].toObject()["armed"].toBool()) machine.disarm();
    else if (page == RunPageIndex && machine.programArmOffered()) notify("Use Arm for program on Run. Jog and Trace still use hold L2.", false);
    else notify("Hold L2 to arm; release L2 to disarm. Centre the sticks before jogging.", false);
  });
  armCancel = theme::button("CANCEL", {}, "pill");
  connect(armCancel, &QPushButton::clicked, this, [this] { machine.cancelArm(); });
  top->addWidget(armPill);
  top->addWidget(armCancel);
  auto *stop = theme::button("■  STOP", "B · R5", "stop");
  stop->setObjectName("headerStop");
  stop->setFixedSize(152, 56);
  connect(stop, &QPushButton::clicked, this, [this] { machine.stop(); });
  top->addWidget(stop);
  root->addWidget(header);

  auto *nav = new QWidget;
  nav->setObjectName("chrome");
  auto *navigation = new QHBoxLayout(nav);
  navigation->setContentsMargins(10, 0, 10, 0);
  const QStringList names{"Jog", "Run", "Telemetry", "Cell"};
  for (int i = 0; i < names.size(); ++i) {
    auto *tab = theme::button(names[i], {}, "tab");
    tab->setCheckable(true);
    tab->setMinimumHeight(46);
    connect(tab, &QPushButton::clicked, this, [this, i] { selectPage(i); });
    tabs << tab;
    navigation->addWidget(tab);
  }

  navigation->addStretch();
  hint = new GlyphLabel("", "caption");
  hint->hide();
  auto *viewButton = theme::button("Layout & appearance ▾");
  viewButton->setMinimumWidth(88);
  connect(viewButton, &QPushButton::clicked, this, [this, viewButton] {
    machine.endHold("ui_navigation");
    gate.cancel();
    QMenu menu(this);
    menu.addAction("Collapse camera", this, [this] { jog->workspace->collapse(-1); });
    menu.addAction("Equal panels", this, [this] { jog->workspace->equalize(); });
    menu.addAction("Collapse program", this, [this] { jog->workspace->collapse(1); });
    menu.addAction("Restore previous split", this, [this] { jog->workspace->collapse(0); });
    menu.addSeparator();
    auto *fitPath = menu.addAction("Fit program path", this, [this] { jog->robot->fitPath(); });
    fitPath->setEnabled(jog->robot->cameraMode() == RobotView::Camera::Orbit);
    menu.addSeparator();
    menu.addAction(theme::dark() ? "Light appearance" : "Dark appearance", this, [this] {
      theme::setDark(!theme::dark());
      QSettings().setValue("theme/dark", theme::dark());
      qApp->setStyleSheet(theme::styleSheet());
      for (auto *w : QApplication::allWidgets()) w->update();
      refresh();
    });
    menu.exec(viewButton->mapToGlobal(QPoint(0, viewButton->height())));
  });
  navigation->addWidget(viewButton);
  auto *controls = theme::button("Controller guide");
  connect(controls, &QPushButton::clicked, this, [this] { showControls(); });
  navigation->addWidget(controls);
  auto* io=theme::button("I/O setup");
  connect(io,&QPushButton::clicked,this,[this]{ machine.endHold("ui_navigation"); IOPanel panel(this,context); panel.exec(); });
  navigation->addWidget(io);
  root->addWidget(nav);

  pages = new QStackedWidget;
  jog = new JogPage(context);
  cell = new CellPage(context);
  runPage = new RunPage(context);
  runPage->toggleProgramArm = [this] { toggleProgramArm(); };
  telemetryPage = new TelemetryPage(context);
  for (QWidget *w : std::initializer_list<QWidget *>{jog, runPage, telemetryPage, cell}) pages->addWidget(w);
  root->addWidget(pages, 1);
  jog->programPane->jog = jog;
  for (auto *editor : {jog->programPane}) {
    editor->newProgram = [this] { newProgram(); };
    editor->openProgram = [this] { openAnyProgram(); };
    editor->saveProgram = [this] { saveProgramAs(); };
    editor->saveCurrent = [this] {
      if (autosave()) {
        if (sync) sync->tick();
        notify("Saved on this Deck · cell sync status is shown beside the program name", false);
      }
      refresh();
    };
  }
  cell->pullSettings = [this] { pullSettings(); };
  cell->adoptSettings = [this] { adoptSettings(); };

  armBlocked = theme::label("", "reason");
  armBlocked->setObjectName("armBlocked");
  armBlocked->setWordWrap(true);
  armBlocked->setContentsMargins(14, 4, 14, 4);
  theme::setState(armBlocked, "tone", "danger");
  armBlocked->hide();
  root->addWidget(armBlocked);

  statusBar = new QWidget;
  statusBar->setObjectName("status");
  auto *footer = new QVBoxLayout(statusBar);
  footer->setContentsMargins(14, 4, 14, 6);
  footer->setSpacing(2);
  // Keep statusLine first for the notification APIs and accessibility order.
  statusLine = theme::elidedLabel("", "caption");
  statusLine->setObjectName("notificationStatus");
  statusSummary = new QWidget;
  auto *bar = new QHBoxLayout(statusSummary);
  bar->setContentsMargins(0, 0, 0, 0);
  bar->setSpacing(12);
  connectionState = theme::elidedLabel("Not connected", "connection");
  connectionState->setObjectName("connectionStatus");
  armStatus = theme::elidedLabel("Arm state unknown", "connection");
  armStatus->setObjectName("observedArmStatus");
  bar->addWidget(connectionState, 2);
  bar->addWidget(armStatus, 3);
  motionStatus = theme::elidedLabel("", "footerReason");
  motionStatus->setObjectName("motionStatus");
  bar->addWidget(motionStatus, 5);
  auto *details = theme::button("DETAILS ▾");
  details->setObjectName("statusDetailsButton");
  details->setProperty("compact", true);
  details->setFixedWidth(112);
  connect(details, &QPushButton::clicked, this, [this] { showStatusDetails(); });
  bar->addWidget(details);
  footer->addWidget(statusSummary);
  // Jog keeps connection, lease and the current reason in one fixed-height
  // row. Other pages use the notification line; Details retains full text.
  footer->addWidget(statusLine);
  root->addWidget(statusBar);

  machine.status = [this](const QString &t, bool e) { notify(t, e); };
  machine.runProgressEnded = [this](const QString &message) {
    const QString notice = "Run program: " + message;
    // A newer error/save notice owns its own lifetime. Retire only the
    // accepted Run message which no longer describes this execution.
    if (statusLine->text() == notice) { statusLine->clear(); notificationAge.invalidate(); }
    if (motionStatus->text() == notice || motionStatus->text() == message) motionStatus->clear();
    runPage->clearRunProgress(message);
  };
  machine.keptTraceBytes = [this](const QString &digest) {
    QFile file(trajectoryPath(digest));
    return file.open(QIODevice::ReadOnly) ? QString::fromLatin1(file.readAll().toBase64()) : QString();
  };
  machine.changed = [this] { refresh(); };
  machine.pendingInputRelease = [this](bool trace) { return pad.pendingRelease(trace); };
  machine.confirmRecover = [this](const QJsonObject &confirmation) {
    return confirmMotion(this, confirmation);
  };
  machine.pressNotice = [this](const QJsonObject &refusal) {
    if (refusal.isEmpty()) core.call("pressNoticeDismiss");
    else core.call("pressNoticeShow", {refusal, double(QDateTime::currentMSecsSinceEpoch())});
    renderPressNotice();
  };
  // Every measured pose, straight to the robot view: smooth at the fast poll.
  machine.pose = [this](const QJsonArray &axes) {
    if (runPage->previewActive() || !jog->robot->isVisible()) return;
    QJsonObject joints;
    for (const auto &a : axes) {
      const auto axis = a.toObject();
      if (!axis.value("position").isDouble()) return;
      joints[axis.value("id").toString()] = axis.value("position").toDouble();
    }
    if (joints.isEmpty()) return;
    jog->robot->joints = joints;
    jog->robot->live = true;
    jog->robot->update();
  };
  pad.pressed = [this](int b) { controllerButton(b); };
  pad.dpad = [this](int dx, int dy) { controllerDpad(dx, dy); };
  OlpClient::setExpectedCell(options.expectCell);
  OlpClient::setFocusProbe([this] { return controllerfocus::focused(this); });
  machine.setExpectedCell(options.expectCell);
  if (!options.expectCell.isEmpty()) {
    setWindowTitle("Rosie / Teach pendant · " + options.expectCell);
    connect(&identityPoll, &QTimer::timeout, this, [this] { pollIdentity(); });
    identityPoll.start(maintenancePollMs);
    pollIdentity();
  }
  pad.lost = [this] { controllerLost(); };
  qApp->installEventFilter(this);

  // Dialogs and reopening are queued: these run inside the sync timer's slot
  // and network callbacks, where a modal dialog would stall them.
  sync = std::make_unique<ProgramSync>(client, core, QFileInfo(currentPath()).absolutePath() + "/current.olp.sync.json");
  sync->notify = [this](const QString &t, bool e) { notify(t, e); };
  sync->opened = [this](const QString &m) { QMetaObject::invokeMethod(this, [this, m] { cellProgramOpened(m); }, Qt::QueuedConnection); };
  sync->changed = [this](const QJsonObject &view) { renderSync(view); };
  sync->offer = [this](const QJsonObject &offer) { QMetaObject::invokeMethod(this, [this, offer] { askSyncOffer(offer); }, Qt::QueuedConnection); };

  // OLP's catalogue first: pinning and adoption read it.
  client.get("/api/offline-programming/v1/robots", [this](const OlpClient::Reply &reply) {
    if (!reply.ok()) {
      notify("OLP server has no robot catalogue: " + reply.error, true);
      return;
    }
    robotCatalogue = reply.body.toObject();
    core.call("setRobots", {robotCatalogue, QJsonValue::Null});
    openStartupProgram();
    sync->start();
    refresh();
  });
  machine.refreshCatalogue();
  connect(&tick, &QTimer::timeout, this, [this] { poll(); });
  tick.setTimerType(Qt::PreciseTimer);
  tick.start(tickMs);
  controlOverlay = new ControlOverlay(this);
  const auto now = [] { return double(QDateTime::currentMSecsSinceEpoch()); };
  controlOverlay->wait = [this, now] { core.call("controlWait", {now()}); renderControl(); };
  controlOverlay->expand = [this] { core.call("controlExpand"); renderControl(); };
  controlOverlay->stop = [this] { machine.stop(); };
  controlOverlay->take = [this] { takeControl(); };
  client.controlRefusal = [this, now](const QJsonObject &body) {
    try {
      if (core.call("controlRefused", {body, now()}).toBool()) renderControl();
    } catch (const OlpError &) {
    }
  };
  jogEndOverlay = new JogEndOverlay(this);
  jogEndOverlay->dismiss = [this] { core.call("jogEndDismiss"); renderJogEnd(); };
  jogEndOverlay->details = [this, now] { core.call("jogEndToggleDetails", {now()}); renderJogEnd(); };
  machine.jogEnded = [this] { renderJogEnd(); };
  machine.jogLive = [this](const QJsonObject &view) {
    // Keep live diagnostics in Details. A path hold must never add a bar or
    // move the program/camera panels while the operator is jogging.
    liveJogStatus = view;
  };
  machine.outputs = [this](const QJsonObject &view) { runPage->renderOutputs(view); };
  connect(&jogEndTick, &QTimer::timeout, this, [this] { renderJogEnd(); });
  pressNoticeOverlay = new JogEndOverlay(this);
  pressNoticeOverlay->dismiss = [this] { core.call("pressNoticeDismiss"); renderPressNotice(); };
  pressNoticeOverlay->details = [this, now] { core.call("pressNoticeToggleDetails", {now()}); renderPressNotice(); };
  connect(&pressNoticeTick, &QTimer::timeout, this, [this] { renderPressNotice(); });
  maintenanceOverlay = new MaintenanceOverlay(this);
  maintenanceOverlay->stop = [this] { machine.stop(); };
  connect(&maintenancePoll, &QTimer::timeout, this, [this] { pollMaintenance(); });
  connect(&maintenanceTick, &QTimer::timeout, this, [this] { renderMaintenance(); });
  maintenancePoll.start(maintenancePollMs);
  pollMaintenance();
  selectPage(0);
}

void Window::pollMaintenance() {
  if (maintenancePolling) return;
  maintenancePolling = true;
  client.get(maintenancePath, [this](const OlpClient::Reply &reply) {
    bool needsHealth = false;
    try {
      needsHealth = core.call("maintenanceObserve", {reply.status, reply.body, reply.error,
                                                     double(QDateTime::currentMSecsSinceEpoch())}).toBool();
      if (reply.ok()) core.call("controlObserve", {reply.body, double(QDateTime::currentMSecsSinceEpoch())});
    } catch (const OlpError &error) {
      notify(QString("Maintenance notice unreadable: %1").arg(error.what()), true);
    }
    renderControl();
    if (!needsHealth) {
      maintenancePolling = false;
      renderMaintenance();
      return;
    }
    client.get("/api/offline-programming/v1/health", [this](const OlpClient::Reply &health) {
      const auto body = health.body.toObject();
      const bool ok = health.ok() && body["ok"].toBool();
      try {
        core.call("maintenanceHealth", {ok, ok ? body["build_id"].toString() : QString()});
      } catch (const OlpError &) {
      }
      maintenancePolling = false;
      renderMaintenance();
    }, maintenanceTimeoutMs);
  }, maintenanceTimeoutMs);
}

void Window::pollIdentity() {
  if (identityPolling) return;
  identityPolling = true;
  client.get("/api/offline-programming/v1/targets", [this](const OlpClient::Reply &reply) {
    identityPolling = false;
    // An unanswered poll keeps the last verdict: an unreachable server
    // refuses arming on its own.
    if (!reply.ok()) return;
    const bool wasWrong = !machine.cellMismatch().isEmpty();
    machine.observeTargets(reply.body.toObject());
    const QString wrong = machine.cellMismatch();
    if (!wrong.isEmpty() && !wasWrong) notify(wrong + " Arming and motion are refused; STOP still works.", true);
    // observeTargets already refreshes Machine and invokes changed. Repeating
    // the full page/core refresh here stalls input a second time every 2 s.
  }, maintenanceTimeoutMs);
}

void Window::announceReplaced(const QString &holder) {
  notify(QString("Closed the pendant that was already running (%1); only one pendant runs on a Deck. A verified unattended loop can continue on its cell.").arg(holder),
         false);
}

void Window::renderMaintenance() {
  QJsonObject view;
  try {
    view = core.call("maintenanceView", {double(QDateTime::currentMSecsSinceEpoch())}).toObject();
  } catch (const OlpError &) {
    return;
  }
  if (view["blocking"].toBool()) {
    QJsonArray sessions;
    try {
      sessions = core.call("maintenanceSessions", {double(QDateTime::currentMSecsSinceEpoch())}).toArray();
    } catch (const OlpError &) {
    }
    maintenanceOverlay->render(view, machine.connected, sessions);
    if (maintenanceShown) return;
    maintenanceShown = true;
    // Nothing waits under the notice: an open dialog is cancelled, as B
    // cancels it, and a held jog ends (Machine::setMaintenance).
    for (int i = 0; i < 8; ++i) {
      auto *modal = qobject_cast<QDialog *>(InWindowDialog::activeDialog());
      if (!modal) break;
      modal->reject();
    }
    padHold = false;
    gate.cancel();
    machine.setMaintenance("Update in progress: " + view["reason"].toString() +
                           "; robot actions are refused until it ends (STOP still works)");
    maintenanceOverlay->setGeometry(rect());
    controlOverlay->hide();
    maintenanceOverlay->show();
    maintenanceOverlay->raise();
    maintenanceOverlay->setFocus();
    maintenanceTick.start(maintenanceTickMs);
    notify("Update in progress: robot actions are refused until it ends", true);
    return;
  }
  if (!maintenanceShown) return;
  maintenanceShown = false;
  maintenanceTick.stop();
  maintenanceOverlay->hide();
  machine.setMaintenance({});
  if (view["reloadSuggested"].toBool()) {
    notify(QString("Update finished: the OLP server was updated (%1 → %2). Restart the pendant if its package was rebuilt.")
               .arg(view["buildBefore"].toString(), view["buildAfter"].toString()), false);
    core.call("maintenanceDismissReload");
  } else {
    notify("Update finished", false);
  }
  renderControl();
  refresh();
}

void Window::renderControl() {
  if (!controlOverlay) return;
  // The maintenance notice covers everything; the control card waits under it.
  if (maintenanceShown) {
    controlOverlay->hide();
    return;
  }
  QJsonObject view;
  try {
    view = core.call("controlView", {double(QDateTime::currentMSecsSinceEpoch())}).toObject();
  } catch (const OlpError &) {
    return;
  }
  controlOverlay->setTaking(taking);
  controlOverlay->render(view);
  if (!view["active"].toBool()) return;
  // Between the header (STOP) and the status line: never over either.
  const int top = header ? header->geometry().bottom() + 1 : 0;
  const int bottom = statusBar ? statusBar->geometry().top() - 1 : height() - 1;
  controlOverlay->place(QRect(0, top, width(), std::max(0, bottom - top + 1)));
}

void Window::renderJogEnd() {
  if (!jogEndOverlay) return;
  QJsonObject view;
  try {
    view = core.call("jogEndView", {double(QDateTime::currentMSecsSinceEpoch())}).toObject();
  } catch (const OlpError &) {
    return;
  }
  // The maintenance notice covers everything; the toast waits under it.
  if (maintenanceShown) view["active"] = false;
  jogEndOverlay->render(view);
  if (!view["active"].toBool()) {
    jogEndTick.stop();
    return;
  }
  if (!jogEndTick.isActive()) jogEndTick.start(500);
  const int top = header ? header->geometry().bottom() + 1 : 0;
  const int bottom = statusBar ? statusBar->geometry().top() - 1 : height() - 1;
  jogEndOverlay->place(QRect(0, top, width(), std::max(0, bottom - top + 1)));
  renderPressNotice();
}

// "Run program refused" (olp-core pressNoticeView): the Jog stopped overlay's
// widget, under it when both show. Takes no focus and no controller input.
void Window::renderPressNotice() {
  if (!pressNoticeOverlay) return;
  QJsonObject view;
  try {
    view = core.call("pressNoticeView", {double(QDateTime::currentMSecsSinceEpoch())}).toObject();
  } catch (const OlpError &) {
    return;
  }
  if (maintenanceShown) view["active"] = false;
  pressNoticeOverlay->render(view);
  if (!view["active"].toBool()) {
    pressNoticeTick.stop();
    return;
  }
  if (!pressNoticeTick.isActive()) pressNoticeTick.start(500);
  int top = header ? header->geometry().bottom() + 1 : 0;
  if (jogEndOverlay && jogEndOverlay->isVisible()) top = jogEndOverlay->geometry().bottom() + 1;
  const int bottom = statusBar ? statusBar->geometry().top() - 1 : height() - 1;
  pressNoticeOverlay->place(QRect(0, top, width(), std::max(0, bottom - top + 1)));
}

void Window::takeControl() {
  if (taking) return;
  taking = true;
  renderControl();
  client.post("/api/offline-programming/v1/control/take", QJsonObject{}, [this](const OlpClient::Reply &reply) {
    taking = false;
    try {
      if (reply.ok()) core.call("controlTook", {reply.body, double(QDateTime::currentMSecsSinceEpoch())});
    } catch (const OlpError &) {
    }
    // A refusal arrived through OlpClient::controlRefusal with the reason.
    notify(reply.ok() ? "Control taken: this pendant controls the robot (nothing moved)"
                      : "Take control refused: " + (reply.detail.isEmpty() ? reply.error : reply.detail), !reply.ok());
    renderControl();
  });
}

void Window::resizeEvent(QResizeEvent *event) {
  QWidget::resizeEvent(event);
  renderControl();
  renderJogEnd();
  if (maintenanceOverlay) maintenanceOverlay->setGeometry(rect());
}

Window::~Window() {
  qApp->removeEventFilter(this);
  releaseIdleProgramArm("app_closed");
  machine.endHold("app_closed");
}

void Window::notify(const QString &text, bool error) {
  diagnostic(error ? "notice_error" : "notice", {{"text", text.left(600)}});
  notificationAge.start();
  statusLine->setText(text);
  theme::setState(statusLine, "role", error ? "reason" : "caption");
  if (motionStatus) { motionStatus->setText(text); motionStatus->setToolTip(text); }
}

void Window::selectPage(int index) {
  // A page change ends any hold and needs neutral controls before the next.
  machine.endHold("ui_navigation");
  padHold = false;
  gate.cancel();
  page = (index % PageCount + PageCount) % PageCount;
  if (page != RunPageIndex) releaseIdleProgramArm("page_changed");
  // One 3D view, shown on Jog and on Run; a preview ends with its page.
  if (page != RunPageIndex) runPage->stopPreview();
  if (page == RunPageIndex) runPage->attachRobot(jog->robot);
  else jog->attachRobot();
  pages->setCurrentIndex(page);
  machine.setFastPoll(jogsHere() || page == RunPageIndex || page == TelemetryPageIndex);
  for (int i = 0; i < tabs.size(); ++i) tabs[i]->setChecked(i == page);
  refresh();
}

void Window::refresh() {
  DiagnosticSpan timing("ui_refresh_timing", refreshTiming);
  machine.prepareTrace();
  timing.mark("trace_prepare_ms");
  const auto inputHealth = pad.acquisitionHealth();
  const bool sampleLag = inputHealth.online && inputHealth.consumedAgeMs >= DeckInput::staleMs;
  if (!inputHealthKnown || inputHealth.online != inputAcquisitionOnline || (sampleLag && !inputSampleLag))
    diagnostic("controller_input_health", {{"native_deck", pad.usesNativeDeck()}, {"online", inputHealth.online},
      {"report_age_ms", double(inputHealth.reportAgeMs)}, {"consumed_age_ms", double(inputHealth.consumedAgeMs)},
      {"losses", double(inputHealth.losses)}, {"ui_sample_lag", sampleLag}});
  inputHealthKnown = true; inputAcquisitionOnline = inputHealth.online; inputSampleLag = sampleLag;
  const auto flow = machine.view()["flow"].toObject();
  const bool confirm = machine.armConfirming();
  const auto arm = flow["arm"].toObject();
  const QString armState = arm["state"].toString("unknown");
  armPill->setText(machine.programExecutionActive() ? "PROGRAM RUNNING · TAP TO STOP"
                  : machine.programArmPermitted() ? (flow["armed"].toBool() ? "PROGRAM ARMED · TAP TO DISARM" : "PROGRAM ARMING · TAP TO CANCEL")
                  : armWaiting ? "WAIT · KEEP L2 HELD"
                  : armGate.active && machine.homeHoldActive() ? "L2 HELD · PRESS HOME"
                  : armGate.active && machine.armRequestBusy() ? "ARMING…"
                  : flow["armed"].toBool() ? "ARMED · RELEASE L2"
                  : armState == "disarmed" ? (page == RunPageIndex && machine.programArmOffered() ? "DISARMED · ARM ON RUN" : "DISARMED   HOLD L2 TO ARM") : arm["token"].toString("STATUS UNKNOWN"));
  armCancel->setVisible(confirm);
  theme::setState(armPill, "tone", flow["armed"].toBool() ? "armed" : "");
  QStringList blockers;
  for (const auto &line : flow["armBlockers"].toArray()) blockers << line.toString();
  // The launcher's cell first: a pendant on the wrong cell says so above
  // every other reason, connected or not.
  const QString expect = machine.expected(), wrongCell = machine.cellMismatch();
  const bool checked = machine.cellChecked();
  if (!wrongCell.isEmpty() && checked) {
    armBlocked->setText("WRONG CELL · " + wrongCell);
    armBlocked->setVisible(true);
  } else if (!inputHealth.online && !injectedInput && !machine.programArmPermitted() && !(page == RunPageIndex && machine.programArmOffered())) {
    armBlocked->setText("CONTROLS OFFLINE · Waiting for controller input; STOP remains available");
    armBlocked->setVisible(true);
  } else {
    armBlocked->setText((flow["canArm"].toBool() ? "BEFORE ARMING · " : "ARM BLOCKED · ") + blockers.join("  ·  "));
    armBlocked->setVisible(machine.connected && !blockers.isEmpty());
  }
  const auto target = machine.view()["target"].toObject();
  QString cellText = machine.connected ? QString("%1 · %2").arg(target["label"].toString(machine.connectedCell), flow["postureLabel"].toString())
                                       : "NO CELL";
  if (!expect.isEmpty() && !checked) cellText = "CHECKING CELL · " + expect;
  else if (!wrongCell.isEmpty()) cellText = "WRONG CELL · this pendant is for " + expect;
  else if (!expect.isEmpty() && !machine.connected) cellText = "NO CELL · for " + expect;
  cellState->setText(cellText);
  theme::setState(cellState, "tone", checked && !wrongCell.isEmpty() ? "danger" : "");
  const QString cellModel = machine.connected ? target["model_id"].toString() : QString();
  if (!cellModel.isEmpty()) {
    if (cellModel != lastCellModel) {
      lastCellModel = cellModel;
      QSettings().setValue("robot/model", cellModel);
    }
    followCell(cellModel);
  }
  timing.mark("header_ms");
  QJsonObject program;
  try {
    program = core.call("view").toObject();
  } catch (const OlpError &) {
  }
  timing.mark("program_view_ms");
  programName->setText(program["name"].toString());
  const QString model = machine.connected && !target["model_id"].toString().isEmpty() ? target["model_id"].toString()
                                                                                         : program["model"].toString();
  if (!model.isEmpty() && model != robotModel) loadRobotModel(model);
  // The live robot: measured joints by axis id, drawn as measured only when fresh.
  QJsonObject joints;
  bool measured = machine.connected && !machine.view()["joints"].toArray().isEmpty();
  for (const auto &j : machine.view()["joints"].toArray()) {
    const auto joint = j.toObject();
    if (joint["positionRad"].isDouble()) joints[joint["id"].toString()] = joint["positionRad"].toDouble();
    else measured = false;
  }
  if (runPage->previewActive()) {
    // The preview poses the view; the machine's pose returns when it ends.
  } else if (measured) {
    jog->robot->joints = joints;
    jog->robot->live = true;
    jog->robot->update();
  } else if (jog->robot->live) {
    jog->robot->live = false;
    jog->robot->update();
  }
  syncPaths(program);
  timing.mark("paths_ms");
  hint->setText(inputHealth.online ? "{L2} arm · {R2} tilt · {DPAD↕} joint/row · {DPAD↔} speed · {B} stop"
                                : "Controller not detected · touch controls only");
  if (page == JogPageIndex) jog->refresh();
  else if (page == RunPageIndex) runPage->refresh();
  else if (page == TelemetryPageIndex) telemetryPage->refresh();
  else if (page == CellPageIndex) cell->refresh();
  timing.mark("page_ms");
  QString syncText = syncState->text();
  syncText.replace(QRegularExpression("\\bR([0-9]+)"), "Rev \\1");
  const QString saved = saveState->text() == "UNSAVED" ? "Unsaved changes" : "Saved";
  jog->programPane->setSaveInfo(saved + " · " + syncText,
      saved + QString(" · edit %1\n").arg(program["revision"].toInt()) +
      (programPath.isEmpty() ? currentPath() : programPath) + "\n" + syncState->toolTip());
  const bool fresh = machine.connected && armState != "unknown";
  const auto leaseValid = machine.view()["leaseValid"];
  const QString lease = !fresh || !leaseValid.isBool() ? "lease unknown" : leaseValid.toBool() ? "lease held" : "lease released";
  connectionState->setText(fresh ? "●  Connected · " + target["model_id"].toString(machine.connectedCell)
                               : machine.connected ? "●  Connection stale" : "●  Not connected");
  theme::setState(connectionState, "tone", fresh ? "ok" : "unknown");
  armStatus->setText("●  " + arm["token"].toString("UNKNOWN") + " · " + lease);
  armStatus->setToolTip(arm["detail"].toString());
  theme::setState(armStatus, "tone", flow["armed"].toBool() ? "armed" : "");
  QString reason = machine.motionReason;
  if (reason.isEmpty() || reason == "done") reason = machine.view()["jogDisabled"].toString();
  const bool armHint = reason.startsWith("No grant") && armState == "disarmed";
  if (armHint) reason = page == RunPageIndex && machine.programArmOffered() ? "Use Arm for program on Run" : "Hold L2 to arm · release L2 to disarm";
  const QString notification = statusLine->text();
  const bool showNotification = (notificationAge.isValid() && notificationAge.elapsed() < 5000) || reason.isEmpty();
  // The arm pill already explains L2. Preserve the notification/reason
  // priority and full tooltip, suppressing only the repeated idle hint.
  motionStatus->setText(showNotification ? displayReason(notification) : armHint ? "" : displayReason(reason));
  theme::setState(motionStatus, "tone", reason.startsWith("operator_stop:") && !showNotification ? "neutral" : "");
  motionStatus->setToolTip(reason + "\n" + notification);
  statusLine->setVisible(page != JogPageIndex);
  statusSummary->setVisible(page == JogPageIndex);
  motionStatus->setVisible(page == JogPageIndex);
  timing.mark("status_ms");
}

void Window::showStatusDetails() {
  machine.endHold("ui_navigation");
  InWindowDialog dialog(this);
  dialog.setObjectName("statusDetails");
  dialog.setProperty("boundedForm", true);
  dialog.setWindowTitle("Machine status");
  auto *layout = new QVBoxLayout(&dialog);
  layout->setContentsMargins(32, 24, 32, 24);
  layout->addWidget(theme::label("Machine status", "title"));
  auto *content = new QWidget;
  auto *body = new QVBoxLayout(content);
  auto *live = new JogLiveStrip;
  body->addWidget(live);
  auto *grid = new QGridLayout;
  grid->setHorizontalSpacing(24); grid->setVerticalSpacing(16);
  QList<QLabel *> values;
  const QStringList names{"Robot", "Connection", "Arm / lease", "Feedback", "Execution", "Joint reference", "Last stop"};
  for (int i = 0; i < names.size(); ++i) {
    grid->addWidget(theme::label(names[i], "caption"), i, 0, Qt::AlignTop);
    auto *value = theme::label("", ""); value->setWordWrap(true);
    grid->addWidget(value, i, 1); values << value;
  }
  grid->setColumnStretch(1, 1); body->addLayout(grid);
  auto *more = theme::button("Technical details ▾"); more->setCheckable(true);
  body->addWidget(more);
  auto *detail = theme::label("", "value"); detail->setWordWrap(true); detail->hide();
  connect(more, &QPushButton::toggled, detail, &QWidget::setVisible);
  body->addWidget(detail); body->addStretch();
  layout->addWidget(scrollingPanel(content), 1);
  auto update = [this, values, detail, live] {
    live->render(liveJogStatus);
    const auto view = machine.view(), flow = view["flow"].toObject();
    values[0]->setText(cellState->text()); values[1]->setText(connectionState->text());
    values[2]->setText(armStatus->text());
    const auto age = machine.statusAgeMs();
    values[3]->setText(QString("Status age %1 · round trip %2 ms").arg(age ? QString::number(*age) + " ms" : "unknown")
      .arg(view["roundTripMs"].isDouble() ? QString::number(view["roundTripMs"].toDouble(), 'f', 1) : "unknown"));
    values[4]->setText(view["executionState"].toString("unknown"));
    QStringList home;
    for (const auto &j : view["joints"].toArray()) home << j.toObject()["id"].toString() + ": " + j.toObject()["home"].toString();
    values[5]->setText(home.join("\n"));
    values[6]->setText(displayReason(machine.motionReason));
    QStringList lines{flow["arm"].toObject()["detail"].toString()};
    for (const auto &blocker : flow["armBlockers"].toArray()) lines << blocker.toString();
    lines << machine.motionReason << view["jogDisabled"].toString() << machine.lastError() << statusLine->text();
    detail->setText(lines.join("\n\n"));
  };
  update();
  QTimer timer(&dialog); connect(&timer, &QTimer::timeout, &dialog, update); timer.start(250);
  auto *done = theme::button("Done"); connect(done, &QPushButton::clicked, &dialog, &QDialog::accept);
  layout->addWidget(done); dialog.resize(1100, 740); dialog.exec();
}

QJsonObject Window::runView() {
  const auto age = machine.statusAgeMs();
  return core.call("runView", {QJsonObject{
      {"connected", machine.connected}, {"cellPending", false},
      {"inFlight", machine.requestInFlight()}, {"statusAgeMs", age ? QJsonValue(double(*age)) : QJsonValue()},
      {"lastRequestError", machine.lastError().isEmpty() ? QJsonValue() : machine.lastError()},
      {"planStartActive", runPage && runPage->movingToStart()}, {"planning", runPage && runPage->planningNow()},
      {"cellModel", machine.view()["target"].toObject()["model_id"].toString()},
      {"bytesKept", QFile::exists(trajectoryPath(core.call("planDigest").toString()))},
      {"holding", machine.holdActive() || padHold}, {"moving", machine.moveBusy()}, {"running", machine.runningProgram()}}}).toObject();
}

void Window::followCell(const QString &cellModel) {
  QJsonObject program, result;
  try {
    program = core.call("view").toObject();
  } catch (const OlpError &) {
    return; // no program yet: the catalogue has not answered
  }
  if (program["model"].toString() == cellModel) return;
  try {
    const QString key = QString("%1|%2|%3").arg(cellModel, core.call("projectIdentity").toString()).arg(program["revision"].toInt());
    if (key == followKey) return;
    followKey = key;
    result = core.call("followCellModel", {cellModel}).toObject();
  } catch (const OlpError &error) {
    notify(QString("Program does not follow the cell: %1").arg(error.what()), true);
    return;
  }
  if (result["action"] == "repinned") {
    autosave();
    notify(result["message"].toString(), false);
  } else if (result["action"] == "mismatch") {
    notify(result["message"].toString(), true);
  }
}

void Window::syncPaths(const QJsonObject &program) {
  auto *view = jog->robot;
  auto point = [](const QJsonValue &v) {
    const auto a = v.toArray();
    return QVector3D(a[0].toDouble(), a[1].toDouble(), a[2].toDouble());
  };
  const QString key = program.isEmpty() ? QString()
                                        : QString("%1/%2/%3").arg(program["name"].toString()).arg(program["revision"].toInt())
                                              .arg(program["selected"].toString());
  if (key != pathKey) {
    pathKey = key;
    view->authored.clear();
    view->targets.clear();
    view->labels.clear();
    view->authoredFocused = false;
    try {
      const auto overlay = program.isEmpty() ? QJsonObject() : core.call("pathOverlay").toObject();
      for (const auto &r : overlay["runs"].toArray()) {
        RobotView::PathRun run;
        run.weld = r.toObject()["weld"].toBool();
        for (const auto &p : r.toObject()["pointsM"].toArray()) run.points.push_back(point(p));
        view->authored.push_back(std::move(run));
      }
      for (const auto &p : overlay["points"].toArray())
        view->targets.push_back({point(p.toObject()["positionM"]), {}, p.toObject()["weld"].toBool(), false});
      for (const auto &l : overlay["labels"].toArray()) {
        const auto label = l.toObject();
        view->labels.push_back({point(label["positionM"]), label["text"].toString(), label["weld"].toBool(), label["emphasis"].toBool()});
      }
      view->authoredFocused = overlay["focused"].toBool();
    } catch (const OlpError &) {
      // A program the path helpers refuse draws no path; the table still shows it.
    }
    view->update();
  }
  // The planned TCP path: every kept sample posed through the view's own FK.
  QString digest;
  try {
    if (!program.isEmpty()) digest = core.call("planDigest").toString();
  } catch (const OlpError &) {
  }
  const QString planned = digest.isEmpty() ? QString() : digest + "@" + robotModel;
  if (planned == plannedKey) return;
  QFile file(trajectoryPath(digest));
  if (!digest.isEmpty() && (view->model.chain.empty() || !file.open(QIODevice::ReadOnly))) {
    // No bytes on the pendant, or no model yet: keep trying on later refreshes.
    if (!view->planned.empty()) {
      view->planned.clear();
      view->update();
    }
    return;
  }
  plannedKey = planned;
  view->planned.clear();
  if (!digest.isEmpty()) {
    try {
      const auto runs = core.callBytes("plannedPathRuns", file.readAll()).toObject();
      const auto order = runs["jointOrder"].toArray();
      const QString frame = runs["frame"].toString();
      for (const auto &r : runs["runs"].toArray()) {
        RobotView::PathRun run;
        run.weld = r.toObject()["weld"].toBool();
        for (const auto &sample : r.toObject()["q"].toArray()) {
          const auto q = sample.toArray();
          QJsonObject joints;
          for (int i = 0; i < order.size() && i < q.size(); ++i) joints[order[i].toString()] = q[i].toDouble();
          const auto frames = view->model.frames(joints);
          if (frames.contains(frame)) run.points.push_back(frames[frame].map(QVector3D()));
        }
        if (run.points.size() >= 2) view->planned.push_back(std::move(run));
      }
    } catch (const OlpError &) {
    }
  }
  view->update();
}

QString Window::trajectoryPath(const QString &digest) const {
  if (digest.isEmpty()) return {};
  // Created by the writer (RunPage::plan), not on every lookup.
  const QString dir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + "/trajectories";
  return dir + "/" + QString(digest).remove("sha256:") + ".dense";
}

void Window::releaseArmHold(const QString &cause) {
  // A refusal or explicit Stop already revoked the permit. Do not overwrite
  // its explanation with another redundant Stop's success message.
  const bool active = armGate.active && machine.armHoldPermitted();
  armWaiting = false;
  armNeutralRequired = true;
  armGate.cancel();
  machine.endHold(cause);
  padHold = false;
  gate.cancel();
  if (active) {
    qInfo() << "L2 released:" << cause;
    if (cause != "trigger_released") {
      const auto *activeWindow = QApplication::activeWindow();
      const auto *modal = InWindowDialog::activeDialog();
      qInfo() << "Arm release context: app-state=" << QGuiApplication::applicationState()
              << "active=" << (activeWindow ? activeWindow->metaObject()->className() : "none")
              << (activeWindow ? activeWindow->objectName() : QString())
              << "modal=" << (modal ? modal->objectName() : QString())
              << "focus=" << (QGuiApplication::focusWindow() ? QGuiApplication::focusWindow()->title() : QString())
              << "L2=" << pad.pad.enabled() << "status-age=" << machine.statusAgeMs().value_or(-1);
    }
    // Stop (disarm), or only clear a Home permit with nothing energised.
    machine.releaseArmHold();
  }
}

void Window::releaseIdleProgramArm(const QString &cause) {
  // An accepted program owns its execution. Merely leaving the Run page
  // cannot turn it into joystick authority or terminate an unattended loop.
  // Before that acceptance, leaving cancels Arm/confirmation/startup via Stop.
  if (machine.programArmPermitted() && !machine.programExecutionActive())
    machine.releaseProgramArm(cause);
}

void Window::toggleProgramArm() {
  if (machine.programArmPermitted()) {
    machine.releaseProgramArm("operator_disarm");
    return;
  }
  if (!machine.programArmOffered()) {
    notify("Arm for program is not available on this cell; hold L2 to arm.", true);
    return;
  }
  const auto age = machine.statusAgeMs();
  if (page != RunPageIndex || !controllerfocus::focused(this) || maintenanceShown ||
      !machine.connected || !machine.cellMismatch().isEmpty() || !age || *age >= 1000) {
    notify("Program arm needs the Run page connected with fresh status and focus.", true);
    return;
  }
  if (pad.pad.enabled() || armGate.active || armWaiting || machine.armHoldPermitted() ||
      pad.pad.magnitude() != 0 || pad.systemButtonHeld() || machine.armRequestBusy() || machine.programExecutionActive()) {
    notify("Release L2 and centre the sticks before selecting Arm for program.", true);
    return;
  }
  armGate.cancel();
  gate.cancel();
  traceGate.cancel();
  armNeutralRequired = true;
  machine.armForProgram();
  refresh();
}

void Window::controllerLost() {
  injectedInput = false;
  pad.pad = {};
  releaseArmHold("controller_disconnected");
  releaseIdleProgramArm("controller_disconnected");
  // Only a program-armed run outlives the controller; an L2-held one was
  // just disarmed above.
  notify(machine.programArmPermitted() && machine.programExecutionActive() ? "Controller disconnected; program execution continues. STOP remains available."
                                          : "Controller disconnected; disarm requested", true);
}

void Window::poll() {
  // Read every tick, focused or not: the pad's state stays current and a
  // background pendant builds no backlog of presses to act on later.
  pad.poll();
  // Retain a release even if L2 was re-pressed during a busy UI frame.
  // The old hold ends; a currently held trigger cannot become a new press.
  if (triggerReleaseSerial != pad.triggerReleaseSerial) {
    triggerReleaseSerial = pad.triggerReleaseSerial;
    releaseArmHold("trigger_released");
    armWaiting = false;
  }
  const auto &p = pad.pad;
  const bool focused = controllerfocus::focused(this);
  const bool modal = InWindowDialog::activeDialog() != nullptr || QApplication::activePopupWidget() != nullptr || maintenanceShown;
  const auto age = machine.statusAgeMs();
  if (machine.programArmPermitted() && (page != RunPageIndex || !focused || pad.systemButtonHeld() ||
      !machine.connected || maintenanceShown || !machine.cellMismatch().isEmpty() || !age || *age >= 1000))
    releaseIdleProgramArm(!focused ? "focus_lost" : page != RunPageIndex ? "page_changed" : "state_poll_failed");
  // Where program arming is offered, an executing program never lends its
  // authority to a new L2 press. Elsewhere L2 behaves as it always has.
  const bool armAllowed = !machine.programArmPermitted() && (!machine.programArmOffered() || !machine.programExecutionActive() || armGate.active) && (pad.connected() || injectedInput) && !pad.systemButtonHeld() && focused && machine.connected && !maintenanceShown && machine.cellMismatch().isEmpty() && age && *age < 1000;
  if (armGate.active && (!p.enabled() || !armAllowed || !machine.armHoldPermitted()))
    releaseArmHold(!p.enabled() ? "trigger_released" : !focused ? "focus_lost" : "state_poll_failed");
  if (armWaiting && (!p.enabled() || !armAllowed || modal || armWaitCancellation != machine.armCancellationRevision())) {
    qInfo() << "L2 pending arm cancelled:" << p.enabled() << armAllowed << modal << armWaitCancellation << machine.armCancellationRevision();
    armWaiting = false;
    armGate.cancel();
    refresh();
  }
  if (armGate.press(armAllowed && !modal, p.enabled())) {
    qInfo().noquote() << "L2 arm press: waiting=" << machine.armRequestBusy() << "sticks=" << p.magnitude();
    armWaiting = true;
    armNeutralRequired = true;
    armNeutralNotified = false;
    gate.cancel();
    armWait.restart();
    armWaitCancellation = machine.armCancellationRevision();
    if (machine.armRequestBusy()) notify("Finishing the previous command — keep L2 held to arm, release to cancel", false);
    refresh();
  }
  if (armWaiting) {
    // Retain only this deliberate press, briefly, across an in-flight Stop.
    // Faults are never retried; release/focus/Stop/cell changes cancel it.
    // 1.5 s is a bounded UI grace period, not a measured robot stop time.
    if (armWait.elapsed() >= 1500) {
      armWaiting = false;
      armGate.cancel();
      notify("Arm timed out waiting for the previous command. Release L2 and try again.", true);
      refresh();
    } else if (!machine.armRequestBusy()) {
      armWaiting = false;
      armGate.active = true;
      machine.permitArmHold();
      machine.arm();
    }
  }
  const bool armedReady = armGate.active && machine.armHoldPermitted() && !machine.armRequestBusy() && machine.view()["flow"].toObject()["armed"].toBool();
  if (armNeutralRequired && armedReady && p.magnitude() == 0) {
    armNeutralRequired = false;
    if (armNeutralNotified) notify("Armed — hold L2; move a stick to jog", false);
  } else if (armNeutralRequired && armedReady && !armNeutralNotified) {
    // Separate Arm from motion: pressing L2 with a thumb on either stick
    // energises the drives but never turns that old deflection into a jog.
    notify("Armed — centre both sticks before jogging; release L2 to disarm", false);
    armNeutralNotified = true;
  }
  const bool allowed = jogsHere() && armAllowed && !modal && armedReady && !armNeutralRequired;
  const bool traceAllowed = allowed && p.magnitude() == 0 && !padHold && !machine.moveBusy() && !machine.holdEnding();
  if (machine.traceActive() && (!traceGate.keep(traceAllowed, p.hatY) || traceReleaseSerial != pad.dpadReleaseSerial)) {
    machine.endHold("button_released");
    traceGate.cancel();
    gate.cancel();
  }
  traceReleaseSerial = pad.dpadReleaseSerial;
  const int tracePress = traceGate.sample(traceAllowed, p.hatY);
  if (tracePress && !machine.holdActive()) {
    gate.cancel();
    machine.beginTrace(tracePress, jog->linear(), jog->angular());
  }
  if (machine.traceActive() || (allowed && p.hatY != 0)) {
    gate.cancel();
    return;
  }
  // With L2 released the right stick orbits the view; R2 modifies it to zoom.
  if (focused && !p.enabled() && !modal && page == JogPageIndex && (std::abs(p.rx) > 0 || std::abs(p.ry) > 0))
    jog->robot->controllerCamera(p.rx * 1.6 * tickMs / 16., -p.ry * 1.2 * tickMs / 16., p.rotate());
  // One axis at a time (OLP's machine Cartesian jog), kept for the hold
  // until its stick returns below the release threshold (stick_hold.hpp).
  const double held = padHold && !jog->stepMode() ? padCode : 0;
  const bool proportional = !(jog->cartesian() && jog->stepMode());
  const StickHold::Sample stick = jog->cartesian() ? StickHold::sample(p.twist(), held, proportional)
                                                   : StickHold::sample(std::array<double, 1>{-p.ly}, held, proportional);
  const double code = stick.code;
  if (holdEndFailureRevision != machine.holdEndFailureRevision()) {
    holdEndFailureRevision = machine.holdEndFailureRevision();
    gate.cancel(); // uncertain retirement must not auto-start another axis
  }
  const double admitted = gate.sample(allowed, p.enabled(), code, p.magnitude() == 0);
  if (padHold && !machine.holdActive()) {
    // The machine or the server ended it (a refusal, the deadman): neutral first.
    padHold = false;
    gate.cancel();
    return;
  }
  if (jog->cartesian() && jog->stepMode()) {
    if (admitted != 0 && !stepLatched && !machine.moveBusy()) {
      const int axis = int(std::abs(admitted)) - 1;
      machine.cartesianStep(jog->frame(), axis, admitted > 0 ? 1 : -1, jog->linear(), jog->angular());
      stepLatched = true;
    }
    if (code == 0) stepLatched = false;
    return;
  }
  if (admitted != 0) {
    if (padHold && admitted != padCode) {
      // Defensive mismatch: never replace an active hold with a new axis.
      machine.endHold("stick_neutral");
      padHold = false;
      gate.cancel();
    } else if (!padHold && !machine.holdActive() && !machine.holdEnding() && !machine.moveBusy()) {
      if (jog->cartesian()) {
        const int axis = int(std::abs(admitted)) - 1;
        machine.beginCartesianHold(jog->frame(), axis, admitted > 0 ? 1 : -1, jog->linear(), jog->angular(), stick.scale);
      } else {
        machine.beginJointHold(jog->selectedJoint(), admitted > 0 ? 1 : -1, stick.scale);
      }
      padHold = machine.holdActive();
      padCode = admitted;
      if (!padHold) gate.cancel();
    } else if (padHold) {
      machine.setHoldScale(stick.scale);
    }
  } else if (padHold) {
    // Why the input let go, for the server's journal (jogEndCause.ts words).
    const char *cause = !p.enabled()                      ? "trigger_released"
                        : maintenanceShown                ? "maintenance"
                        : !jogsHere() || modal            ? "ui_navigation"
                        : !focused                        ? "focus_lost"
                        : !machine.connected || !machine.cellMismatch().isEmpty() ? "cell_change"
                                                          : "stick_neutral";
    machine.endHold(cause);
    padHold = false;
    // Ordinary release leaves the input gate open. After End is acknowledged,
    // the latest held direction can take over; no saved/queued stick command.
    if (QString::fromLatin1(cause) != "stick_neutral") gate.cancel();
  }
}

void Window::controllerButton(int b) {
  // Queued, not called from poll(): a press that opens a dialog (Menu's guide)
  // would otherwise run the dialog inside the tick timer's slot, and Qt does
  // not re-enter a timer's slot, so the controller -- B's STOP included --
  // went dead until the dialog closed. Focus is read at the press and again
  // when it is handled; both must hold.
  const bool focused = controllerfocus::focused(this);
  QMetaObject::invokeMethod(this, [this, b, focused] { button(b, focused); }, Qt::QueuedConnection);
}

void Window::controllerDpad(int dx, int dy) {
  const bool focused = controllerfocus::focused(this);
  QMetaObject::invokeMethod(this, [this, dx, dy, focused] { dpad(dx, dy, focused); }, Qt::QueuedConnection);
}

void Window::button(int b, bool focusedAtPress) {
  diagnostic("button", {{"button", b}, {"focusedAtPress", focusedAtPress}, {"dialog", InWindowDialog::activeDialog() != nullptr}, {"popup", QApplication::activePopupWidget() != nullptr}});
  const bool focused = focusedAtPress && controllerfocus::focused(this);
  if (!controllerfocus::admits(focused, b)) {
    // Said, never acted on: a press that did nothing must not look like a
    // pendant that needs two (the journal shows how often it happens).
    std::fprintf(stderr, "rosie-pendant-v5: controller button %d ignored: this pendant does not have input focus\n", b);
    notify("Controller press ignored: this pendant does not have input focus. Touch its screen, then press again.", true);
    return;
  }
  auto *modal = qobject_cast<QDialog *>(InWindowDialog::activeDialog());
  if (b == Controller::B) {
    if (!focused) std::fprintf(stderr, "rosie-pendant-v5: B (Stop) from the controller while this pendant is in the background\n");
    machine.stop();
    if (capturing) { diagnostic("record_cancel", {{"attempt", recordAttempt}, {"reason", "stop"}}); ++captureEpoch; capturing = false; recordPressClock.invalidate(); core.call("cancelPendingRecord"); }
    if (modal) modal->reject();
    if (auto *popup = QApplication::activePopupWidget()) popup->close();
    return;
  }
  if (QApplication::activePopupWidget()) return;
  // During an update only STOP (B, above) acts.
  if (maintenanceShown) return;
  if (modal) {
    if (b == Controller::A) modal->accept();
    return;
  }
  switch (b) {
    case Controller::A:
      record("new", "button-A");
      break;
    case Controller::L4: if (jogsHere() && !capturing) { jog->programPane->deleteSelected(); refresh(); } break;
    case Controller::X: record("reteach", "button-X"); break;
    case Controller::Y: record("via", "button-Y"); break;
    case Controller::LB: selectPage(page - 1); break;
    case Controller::RB: selectPage(page + 1); break;
    case Controller::View: selectPage(CellPageIndex); break;
    case Controller::Menu: showControls(); break;
    case Controller::L3:
      machine.endHold("mode_switch");
      gate.cancel();
      jog->setCartesian(!jog->cartesian());
      refresh();
      break;
    case Controller::R3: jog->robot->resetCamera(); break;
  }
}

void Window::dpad(int dx, int dy, bool focusedAtPress) {
  if (!focusedAtPress || !controllerfocus::focused(this)) return;
  // The program picker is the one dialog the D-pad moves through (A opens).
  if (auto *picker = dynamic_cast<ProgramPicker *>(InWindowDialog::activeDialog())) {
    if (dy) picker->move(dy);
    return;
  }
  if (InWindowDialog::activeDialog() || QApplication::activePopupWidget() || maintenanceShown) return;
  if (jogsHere()) {
    // Vertical navigation is disarmed-only. A held arrow is sampled by
    // poll() for trace; this queued edge must never also move its cursor.
    const bool disarmed = !pad.pad.enabled() && !armGate.active && !armWaiting &&
      !machine.view()["flow"].toObject()["armed"].toBool();
    if (dy && disarmed) { jog->programPane->selectRow(dy); refresh(); }
    if (dx) jog->stepSpeed(dx);
  }
}

bool Window::eventFilter(QObject *object, QEvent *event) {
  // In-window forms have no native modal window. Block inputs aimed at the
  // covered page while preserving releases and the global Stop keys below.
  if (auto *panel = InWindowDialog::activePanel()) {
    auto *widget = qobject_cast<QWidget *>(object);
    const bool outside = widget && widget != panel && !panel->isAncestorOf(widget);
    const auto *key = event->type() == QEvent::KeyPress ? static_cast<QKeyEvent *>(event) : nullptr;
    const bool stop = key && (key->key() == Qt::Key_F4 || key->key() == Qt::Key_Escape);
    // A compact child form leaves the header visible. Its STOP must remain
    // operable; all other background controls remain blocked.
    const bool headerStop = widget && widget->objectName() == "headerStop";
    if (outside && !stop && !headerStop) {
      switch (event->type()) {
        case QEvent::KeyPress: case QEvent::MouseButtonPress: case QEvent::MouseButtonDblClick:
        case QEvent::TouchBegin: case QEvent::Wheel: return true;
        default: break;
      }
    }
  }
  // Gamescope shows one window at a time, scaled: a dialog smaller than the
  // screen would be letterboxed and its touch points offset. Every dialog
  // covers the pendant exactly instead.
  if (event->type() == QEvent::Show)
    if (auto *dialog = qobject_cast<QDialog *>(object); dialog && dialog->isWindow() && isFullScreen()) {
      // Its own layout at its own size, centred in the screen-sized window.
      if (auto *layout = dialog->layout(); layout && !dialog->property("centred").toBool() && !dialog->property("boundedForm").toBool()) {
        const QSize hint = dialog->sizeHint();
        const int x = std::max(12, (width() - hint.width()) / 2), y = std::max(12, (height() - hint.height()) / 2);
        const QMargins m = layout->contentsMargins();
        layout->setContentsMargins(std::max(x, m.left()), std::max(y, m.top()), std::max(x, m.right()), std::max(y, m.bottom()));
        dialog->setProperty("centred", true);
      }
      dialog->setFixedSize(size());
      dialog->setGeometry(geometry());
    }
  if (event->type() == QEvent::KeyPress) {
    auto *key = static_cast<QKeyEvent *>(event);
    if (key->isAutoRepeat()) return false;
    // The rear buttons arrive as F1-F4 (controller.vdf).
    if (key->key() == Qt::Key_F4 || key->key() == Qt::Key_Escape) {
      machine.stop();
      return key->key() == Qt::Key_F4;
    }
    // During an update no other key acts (B, R5, Esc and F4 stop, above).
    if (maintenanceShown) return true;
    if (InWindowDialog::activeDialog() || QApplication::activePopupWidget()) return false;
    if (key->key() == Qt::Key_F1) {
      if (!key->isAutoRepeat() && !pad.usesNativeDeck() && jogsHere() && !capturing)
        jog->programPane->deleteSelected();
      return true;
    }
    if (key->key() == Qt::Key_F2) { record("reteach"); return true; }
    if (key->key() == Qt::Key_F3) { record("via"); return true; }
  }
  // A press that lands on anything but the notice -- another top-level
  // window, a widget it does not cover -- goes nowhere during an update.
  // Releases pass, so a button held when the notice appeared lets go.
  if (maintenanceShown) {
    switch (event->type()) {
      case QEvent::MouseButtonPress:
      case QEvent::MouseButtonDblClick:
      case QEvent::TouchBegin:
      case QEvent::Wheel:
        if (auto *widget = qobject_cast<QWidget *>(object); widget && widget != maintenanceOverlay && !maintenanceOverlay->isAncestorOf(widget))
          return true;
        break;
      default:
        break;
    }
  }
  return QWidget::eventFilter(object, event);
}

void Window::changeEvent(QEvent *event) {
  if (event->type() == QEvent::ActivationChange && !isActiveWindow()) {
    // Focus loss (Steam overlay, another window) ends any hold.
    machine.endHold("focus_lost");
    padHold = false;
    gate.cancel();
    // A dialog of our own keeps focus; another app consumes the L2 hold.
    if (!controllerfocus::focused(this)) {
      releaseArmHold("focus_lost");
      releaseIdleProgramArm("focus_lost");
    }
  }
  QWidget::changeEvent(event);
}

void Window::record(const QString &mode, const QString &source) {
  if (capturing && recordMode == "new" && mode == "new" && recordPressClock.isValid() && recordPressClock.elapsed() <= 350) {
    diagnostic("record_double", {{"attempt", recordAttempt}});
    recordQuick = true; return;
  }
  if (capturing) {
    notify("Still recording the previous waypoint; this press recorded nothing", true);
    return;
  }
  const unsigned epoch = ++captureEpoch;
  capturing = true; recordQuick = false; recordMode = mode;
  recordAttempt = QUuid::createUuid().toString(QUuid::WithoutBraces);
  recordPressClock.start();
  recordProgram = core.call("projectIdentity");
  recordSelection = core.call("view").toObject()["selected"];
  recordRevision = core.call("view").toObject()["revision"].toInt();
  recordTarget = machine.view()["target"].toObject(); recordPage = page;
  diagnostic("record_received", {{"attempt", recordAttempt}, {"source", source}, {"mode", mode}, {"program", recordProgram}});
  tryRecord(epoch);
}

void Window::tryRecord(unsigned epoch) {
  if (epoch != captureEpoch || !capturing) return;
  const QString mode = recordMode;
  const QString attempt = recordAttempt;
  QJsonObject measured, start;
  const auto age = machine.statusAgeMs();
  try {
    if (!controllerfocus::focused(this) || maintenanceShown || page != recordPage ||
        core.call("projectIdentity") != recordProgram || core.call("view").toObject()["selected"] != recordSelection ||
        core.call("view").toObject()["revision"].toInt() != recordRevision ||
        machine.view()["target"].toObject() != recordTarget)
      throw OlpError("Recording cancelled: focus, program or cell changed; press A again");
    if (pad.pad.magnitude() != 0 || pad.pad.hatY != 0)
      throw OlpError("Release motion controls and wait for the jog to end before recording");
    if (machine.holdEnding()) throw OlpError("Release jog and wait for fresh stationary position samples before recording");
    // A program for another robot is refused first, whatever the robot is doing.
    const QString mismatch = machine.connected ? core.call("modelMismatch", {machine.view()["target"].toObject()["model_id"].toString()}).toString() : QString();
    if (!mismatch.isEmpty()) throw OlpError(mismatch.toStdString());
    measured = core.call("teachObservation", {QJsonObject{
        {"selected", machine.connected}, {"busy", machine.teachBusy()},
        {"statusAgeMs", age ? QJsonValue(double(*age)) : QJsonValue()},
        {"cellModel", machine.view()["target"].toObject()["model_id"].toString()}, {"cellId", machine.connectedCell}}}).toObject();
    start = core.call("captureStart", {mode, measured}).toObject();
    start["programId"] = core.call("projectIdentity");
    start["selection"] = core.call("view").toObject()["selected"];
  } catch (const OlpError &error) {
    const QString reason = QString::fromUtf8(error.what());
    const auto details = core.call("teachDiagnostics").toObject();
    diagnostic("record_readiness", {{"attempt", attempt}, {"reason", reason}, {"ageMs", age ? double(*age) : -1.}, {"elapsedMs", double(recordPressClock.elapsed())}, {"settle", details}});
    if (reason.contains("stationary position samples") && recordPressClock.elapsed() < 1500) {
      if (recordPressClock.elapsed() < 80) notify("Waiting for the robot to settle…", false);
      QTimer::singleShot(50, this, [this, epoch] { tryRecord(epoch); });
      return;
    }
    capturing = false; recordPressClock.invalidate();
    diagnostic("record_failed", {{"attempt", attempt}, {"reason", reason}});
    notify(QString("Record blocked: %1").arg(reason), true);
    return;
  }
  diagnostic("record_pose", {{"attempt", attempt}, {"elapsedMs", double(recordPressClock.elapsed())}, {"settle", core.call("teachDiagnostics")}});
  auto fail = [this, attempt](const QString &why) {
    capturing = false; recordPressClock.invalidate();
    diagnostic("record_failed", {{"attempt", attempt}, {"reason", why}});
    notify("Record blocked: " + why, true);
  };
  auto resolve = [this, epoch, mode, measured, start, fail](const QJsonValue &calibration) {
    const QJsonObject body{{"model_id", start["modelId"]}, {"robot_description_sha256", start["description"]},
                           {"machine_planning_calibration_base64",
                            calibration.toObject()["machinePlanningCalibrationBase64"].toString()}};
    client.post("/api/offline-programming/v1/robots/capture-model", body,
                [this, epoch, mode, measured, start, calibration, fail](const OlpClient::Reply &reply) {
      if (epoch != captureEpoch) return;
      if (!reply.ok()) { fail(reply.body.toObject()["error"].toString(reply.error)); return; }
      diagnostic("record_model", {{"attempt", recordAttempt}});
      try {
        const auto model = reply.body.toObject();
        const auto frame = core.call("captureFrame", {start["modelId"]}).toObject();
        CaptureChain chain;
        QString error;
        if (!chain.parse(model["urdf"].toString(), frame["frame"].toString(), error)) { fail(error); return; }
        const auto pose = measured["pose"].toObject();
        const auto order = pose["jointOrder"].toArray();
        const auto values = pose["jointsRad"].toArray();
        QHash<QString, double> joints;
        for (int i = 0; i < order.size() && i < values.size(); ++i) joints[order[i].toString()] = values[i].toDouble();
        const auto tcp = chain.pose(joints, error);
        if (!tcp) { fail(error); return; }
        QJsonArray tcpArray;
        for (double v : *tcp) tcpArray << v;
        if (core.call("projectIdentity") != start["programId"] || core.call("view").toObject()["selected"] != start["selection"]) { fail("Program changed during recording; record again"); return; }
        core.call("captureRecord", {mode, measured, start["revision"], start["description"], calibration, model["identity"], tcpArray,
                                    QUuid::createUuid().toString(QUuid::WithoutBraces), mode == "new"});
        if (mode == "new") {
          // Delay only the dialog until the double-press window closes;
          // the measured pose and network capture already began at press one.
          const int wait = std::max(0, 350 - int(recordPressClock.elapsed()));
          QTimer::singleShot(wait, this, [this, epoch, fail] {
            if (epoch != captureEpoch) return;
            try {
              bool save = recordQuick && controllerfocus::focused(this);
              if (!save && controllerfocus::focused(this) && !maintenanceShown)
                save = jog->programPane->reviewPendingRecord();
              diagnostic("record_review", {{"attempt", recordAttempt}, {"quick", recordQuick}, {"accepted", save}});
              if (save && !maintenanceShown) {
                core.call("commitPendingRecord");
                const bool saved = autosave();
                diagnostic("record_local_save", {{"attempt", recordAttempt}, {"ok", saved}, {"program", core.call("projectIdentity")}, {"revision", core.call("view").toObject()["revision"]}});
                if (saved) {
                  machine.prepareTrace(true);
                  notify("Waypoint saved", false);
                }
              } else { core.call("cancelPendingRecord"); notify("Waypoint not saved", false); }
              capturing = false; recordPressClock.invalidate(); refresh();
            } catch (const OlpError &error) { core.call("cancelPendingRecord"); recordPressClock.invalidate(); fail(error.what()); }
          });
          return;
        }
        capturing = false;
        notify(QString("Measured %1 recorded").arg(mode == "via" ? "via pose" : "waypoint"), false);
        if (autosave()) machine.prepareTrace(true);
        refresh();
      } catch (const OlpError &error) {
        fail(error.what());
      }
    });
  };
  if (!start["machineBound"].toBool()) {
    resolve(QJsonValue::Null);
    return;
  }
  // The machine's own calibration, fetched now, with the identity it hashed it under.
  client.get("/api/offline-programming/v1/dense-execution/cell", [this, epoch, age, resolve, fail](const OlpClient::Reply &reply) {
    if (epoch != captureEpoch) return;
    if (!reply.ok()) { fail(reply.error); return; }
    try {
      resolve(core.call("fetchedCellSettings", {machine.connected, age ? QJsonValue(double(*age)) : QJsonValue(), reply.body}));
    } catch (const OlpError &error) {
      fail(error.what());
    }
  });
}

void Window::pullSettings() {
  const auto age = machine.statusAgeMs();
  client.get("/api/offline-programming/v1/dense-execution/cell", [this, age](const OlpClient::Reply &reply) {
    if (!reply.ok()) {
      notify("Pull cell settings blocked: " + reply.error, true);
      return;
    }
    try {
      const auto settings = core.call("fetchedCellSettings", {machine.connected, age ? QJsonValue(double(*age)) : QJsonValue(), reply.body});
      if (settings.isNull()) {
        notify("Pull cell settings blocked: the cell has not reported its status yet", true);
        return;
      }
      const bool changed = core.call("pullCellSettings", {settings}).toBool();
      notify(changed ? "Pulled cell settings from " + settings.toObject()["cell"].toString()
                     : "Cell settings already match " + settings.toObject()["cell"].toString(), false);
      autosave();
      refresh();
    } catch (const OlpError &error) {
      notify(QString("Pull cell settings blocked: %1").arg(error.what()), true);
    }
  });
}

void Window::adoptSettings() {
  try {
    core.call("adoptDescription");
    notify("Adopted the current robot description; plan again and verify before running", false);
    autosave();
    refresh();
  } catch (const OlpError &error) {
    notify(error.what(), true);
  }
}

void Window::loadRobotModel(const QString &model) {
  robotModel = model;
  const QString directory = options.robots + "/" + model;
  // Mesh import takes seconds; the view keeps its message until it lands.
  auto *view = jog->robot;
  view->clear();
  try {
    view->cameras = core.call("cameras", {model}).toObject();
  } catch (const OlpError &) {
    view->cameras = {};
  }
  QStringList packages;
  try {
    for (const auto &package : core.call("meshPackages", {model}).toArray()) packages << package.toString();
  } catch (const OlpError &) {
  }
  view->message = "Loading " + model;
  // Mesh import runs off the UI thread; a load that a later model change
  // superseded must not install over the model the pendant now shows.
  const unsigned load = ++modelLoad;
  std::thread([this, directory, view, packages, load] {
    RobotAssets assets = loadRobotMeshes(directory, {}, packages);
    QMetaObject::invokeMethod(this, [this, view, load, assets = std::move(assets)]() mutable {
      if (load != modelLoad) return;
      if (!assets.error.isEmpty()) {
        view->message = "Robot description unavailable: " + assets.error;
        view->update();
        return;
      }
      view->install(std::move(assets));
    }, Qt::QueuedConnection);
  }).detach();
}

QString Window::currentPath() const {
  const QString dir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
  QDir().mkpath(dir);
  return dir + "/current.olp.json";
}

bool Window::autosave() {
  try {
    const QString text = core.call("exportProject").toString();
    QStringList paths;
    if (!autosaveRefused) paths << currentPath();
    if (!programPath.isEmpty()) paths << programPath;
    for (const QString &path : paths) {
      QSaveFile file(path);
      const auto bytes = text.toUtf8();
      if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit()) {
        notify("Save failed: " + file.errorString(), true);
        saveState->setText("UNSAVED");
        return false;
      }
    }
    if (paths.isEmpty()) { notify("Save failed: no writable program destination", true); return false; }
    saveState->setText(!programPath.isEmpty() ? "SAVED" : autosaveRefused ? "UNSAVED" : "LOCAL");
    return true;
  } catch (const OlpError &error) {
    notify(error.what(), true);
    return false;
  }
}

void Window::setProgramPath(const QString &path) {
  programPath = path;
  QSettings settings;
  if (path.isEmpty()) settings.remove("program/path");
  else settings.setValue("program/path", path);
}

void Window::openStartupProgram() {
  QSettings settings;
  const QString autosavePath = currentPath(), remembered = settings.value("program/path").toString();
  auto read = [this](const QString &path, QString &error) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
      error = file.errorString();
      return false;
    }
    try {
      core.call("openProject", {QString::fromUtf8(file.readAll())});
      return true;
    } catch (const OlpError &e) {
      error = e.what();
      return false;
    }
  };
  QStringList notes;
  bool problem = false, opened = false;
  if (QFileInfo::exists(autosavePath)) {
    QString error, moveError;
    opened = read(autosavePath, error);
    if (!opened) {
      problem = true;
      const QString aside = programfiles::setAside(autosavePath, QDateTime::currentDateTime(), &moveError);
      if (aside.isEmpty()) {
        autosaveRefused = true;
        notes << QString("%1 could not be read (%2) or moved aside (%3); it is left as it is and nothing is autosaved over it")
                     .arg(autosavePath, error, moveError);
      } else {
        notes << QString("The saved program could not be read (%1); it was moved to %2").arg(error, aside);
      }
    }
  }
  if (opened) {
    const QString kept = programfiles::keepNamed(remembered, core.call("projectIdentity").toString(), [this](const QString &path) {
      QFile file(path);
      return file.open(QIODevice::ReadOnly) ? core.call("projectIdentityOf", {QString::fromUtf8(file.readAll())}).toString() : QString();
    });
    if (kept.isEmpty() && !remembered.isEmpty())
      notes << QString("%1 no longer holds this program; it saves on the pendant only until SAVE AS").arg(QFileInfo(remembered).fileName());
    setProgramPath(kept);
  } else if (!remembered.isEmpty() && QFileInfo::exists(remembered)) {
    QString error;
    opened = read(remembered, error);
    if (opened) {
      setProgramPath(remembered);
      notes << "Reopened " + QFileInfo(remembered).fileName();
    } else {
      notes << QString("%1 could not be read either (%2)").arg(QFileInfo(remembered).fileName(), error);
    }
  }
  if (!opened) {
    setProgramPath({});
    try {
      core.call("newProject", {"New pendant program", core.call("startupModel", {settings.value("robot/model").toString()})});
      opened = true;
      if (problem) notes << "a new program was started";
    } catch (const OlpError &error) {
      notes << QString("No program: %1").arg(error.what());
      problem = true;
    }
  }
  if (opened) autosave();
  if (!notes.isEmpty()) notify(notes.join("; "), problem);
}

bool Window::openFile(const QString &path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    notify(QString("Could not open %1: %2").arg(QFileInfo(path).fileName(), file.errorString()), true);
    return false;
  }
  try {
    core.call("openProject", {QString::fromUtf8(file.readAll())});
  } catch (const OlpError &error) {
    notify(QString("Could not open %1: %2").arg(QFileInfo(path).fileName(), error.what()), true);
    return false;
  }
  machine.clearRunProgress();
  setProgramPath(path);
  autosave();
  // An opened file becomes the cell's current program (a conflict if the
  // cell already has another revision of it).
  sync->upload();
  refresh();
  return true;
}

namespace {
std::optional<QString> askText(QWidget *parent, const QString &title, const QString &initial) {
  QDialog dialog(parent);
  auto *layout = new QVBoxLayout(&dialog);
  layout->addWidget(theme::label(title, "section"));
  layout->addWidget(theme::label("Steam + X opens the on-screen keyboard.", "caption"));
  auto *line = new QLineEdit(initial);
  line->setMinimumWidth(520);
  layout->addWidget(line);
  auto *row = new QHBoxLayout;
  auto *cancel = theme::button("Cancel");
  auto *ok = theme::button(title.startsWith("New") ? "Create program" : "Save copy", "A", "primary");
  QObject::connect(cancel, &QPushButton::clicked, &dialog, &QDialog::reject);
  QObject::connect(ok, &QPushButton::clicked, &dialog, &QDialog::accept);
  // Enter on the Steam keyboard (or a real one) submits, as OK does.
  QObject::connect(line, &QLineEdit::returnPressed, &dialog, &QDialog::accept);
  row->addWidget(cancel);
  row->addWidget(ok);
  layout->addLayout(row);
  line->setFocus();
  if (dialog.exec() != QDialog::Accepted || line->text().trimmed().isEmpty()) return std::nullopt;
  return line->text().trimmed();
}

// A yes/no question; A confirms, B (and Cancel) declines.
bool askConfirm(QWidget *parent, const QString &title, const QString &text, const QString &confirm) {
  QDialog dialog(parent);
  auto *layout = new QVBoxLayout(&dialog);
  layout->addWidget(theme::label(title, "section"));
  auto *body = theme::label(text, "");
  body->setWordWrap(true);
  body->setMinimumWidth(520);
  layout->addWidget(body);
  auto *row = new QHBoxLayout;
  auto *cancel = theme::button("Cancel", "B");
  auto *ok = theme::button(confirm, "A", "primary");
  QObject::connect(cancel, &QPushButton::clicked, &dialog, &QDialog::reject);
  QObject::connect(ok, &QPushButton::clicked, &dialog, &QDialog::accept);
  row->addWidget(cancel);
  row->addWidget(ok);
  layout->addLayout(row);
  return dialog.exec() == QDialog::Accepted;
}
} // namespace


void Window::newProgram() {
  const auto name = askText(this, "New program name", "Untitled robot program");
  if (!name) return;
  // The connected cell's robot; without one, the robot on screen or the last cell's.
  QString model = machine.connected ? machine.view()["target"].toObject()["model_id"].toString() : QString();
  try {
    if (model.isEmpty()) model = core.call("startupModel", {robotModel.isEmpty() ? QSettings().value("robot/model").toString() : robotModel}).toString();
    core.call("newProject", {*name, model});
  } catch (const OlpError &error) {
    notify(QString("New program blocked: %1").arg(error.what()), true);
    return;
  }
  machine.clearRunProgress();
  setProgramPath({});
  autosave();
  sync->upload();
  notify(QString("New program for %1 · empty until you record or add rows").arg(model), false);
  refresh();
}

void Window::saveProgramAs() {
  const QString current = core.call("view").toObject()["name"].toString();
  const auto name = askText(this, "Save program as", current);
  if (!name) return;
  QDir().mkpath(options.programs);
  const QString target = programfiles::saveAsPath(options.programs, *name);
  const QString file = QFileInfo(target).fileName();
  if (programfiles::replacesOther(target, programPath) &&
      !askConfirm(this, "Replace " + file + "?",
                  QString("%1 already holds a program (saved %2). Replace it with this one?")
                      .arg(file, QFileInfo(target).lastModified().toString("yyyy-MM-dd hh:mm")), "REPLACE")) {
    notify("Save As cancelled; " + file + " is unchanged", false);
    return;
  }
  const QString previous = programPath;
  setProgramPath(target);
  if (autosave()) {
    notify("Saved " + target, false);
  } else {
    setProgramPath(previous);
    saveState->setText("UNSAVED");
  }
}

void Window::openProgram() {
  QDir dir(options.programs);
  const auto files = dir.entryInfoList({"*.json"}, QDir::Files, QDir::Time);
  if (files.isEmpty()) {
    QDialog empty(this);
    auto *body = new QVBoxLayout(&empty);
    body->addWidget(theme::label("No programs on this Deck", "title"));
    auto *explanation = theme::label("Export a program from OLP using Project → Export, then place its JSON file in:\n" + options.programs, "");
    explanation->setWordWrap(true); explanation->setMinimumWidth(620); body->addWidget(explanation);
    auto *close = theme::button("Close"); body->addWidget(close);
    connect(close, &QPushButton::clicked, &empty, &QDialog::accept); empty.exec();
    return;
  }
  QDialog dialog(this);
  auto *layout = new QVBoxLayout(&dialog);
  layout->addWidget(theme::label("OPEN PROGRAM · " + options.programs, "section"));
  auto *list = new QVBoxLayout;
  layout->addLayout(list);
  auto *pager = new QHBoxLayout;
  auto *previous = theme::button("◀ PAGE");
  auto *next = theme::button("PAGE ▶");
  auto *cancel = theme::button("Cancel");
  pager->addWidget(previous);
  pager->addWidget(next);
  pager->addStretch();
  pager->addWidget(cancel);
  layout->addLayout(pager);
  QString chosen;
  int first = 0;
  constexpr int perPage = 8;
  std::function<void()> fill = [&] {
    while (auto *item = list->takeAt(0)) {
      delete item->widget();
      delete item;
    }
    for (int i = first; i < std::min(int(files.size()), first + perPage); ++i) {
      auto *b = theme::button(QString("%1   ·   %2").arg(files[i].completeBaseName(),
                                                        files[i].lastModified().toString("yyyy-MM-dd hh:mm")));
      b->setMinimumSize(640, 48);
      connect(b, &QPushButton::clicked, &dialog, [&, path = files[i].absoluteFilePath()] { chosen = path; dialog.accept(); });
      list->addWidget(b);
    }
    previous->setEnabled(first > 0);
    next->setEnabled(first + perPage < files.size());
  };
  connect(previous, &QPushButton::clicked, &dialog, [&] { first = std::max(0, first - perPage); fill(); });
  connect(next, &QPushButton::clicked, &dialog, [&] { first += perPage; fill(); });
  connect(cancel, &QPushButton::clicked, &dialog, &QDialog::reject);
  fill();
  if (dialog.exec() == QDialog::Accepted && !chosen.isEmpty() && openFile(chosen))
    notify("Opened " + QFileInfo(chosen).fileName(), false);
}

void Window::showControls() {
  machine.endHold("ui_navigation");
  gate.cancel();
  InWindowDialog dialog(this);
  dialog.setObjectName("controllerGuide");
  auto *layout = new QVBoxLayout(&dialog);
  layout->setSpacing(12);
  auto *head = new QHBoxLayout;
  head->addWidget(theme::label("Controller guide", "title"));
  head->addStretch();
  head->addWidget(new GlyphLabel("{Menu} opens this guide   ·   {A} closes it   ·   {B} stops and closes it", "caption"));
  layout->addLayout(head);
  // Each row: the Deck's controls as drawn on the buttons, then what they do.
  using Rows = std::initializer_list<std::pair<const char *, const char *>>;
  auto card = [](const char *title, Rows rows, const char *note = nullptr, const char *accent = nullptr) {
    auto c = theme::card(title);
    if (accent) c.frame->setStyleSheet(QString("QFrame#card { border: 2px solid %1; }").arg(accent));
    auto *grid = new QGridLayout;
    grid->setHorizontalSpacing(16);
    grid->setVerticalSpacing(8);
    int r = 0;
    for (const auto &[keys, words] : rows) {
      grid->addWidget(new GlyphLabel(keys, "guide"), r, 0, Qt::AlignLeft | Qt::AlignVCenter);
      auto *text = theme::label(words, "");
      text->setWordWrap(true);
      grid->addWidget(text, r++, 1);
    }
    grid->setColumnMinimumWidth(0, 150);
    grid->setColumnStretch(1, 1);
    c.body->addLayout(grid);
    if (note) {
      auto *n = theme::label(note, "caption");
      n->setWordWrap(true);
      c.body->addWidget(n);
    }
    return c.frame;
  };
  auto *columns = new QHBoxLayout;
  columns->setSpacing(14);
  auto *left = new QVBoxLayout;
  left->setSpacing(14);
  left->addWidget(card("JOG", Rows{
      {"{L2}", "Hold to arm, or to Home when required; release to disarm"},
      {"{LS↕}", "Joints: jog the selected joint"},
      {"{LS}", "Cartesian: move X and Y"},
      {"{RS↕}", "Cartesian: move Z"},
      {"{R2} + {LS}", "Cartesian: tilt about X and Y"},
      {"{RS↔}", "Cartesian: turn about Z"},
  }, "On the Jog page (teach: jog, record, jog, record); never while a program plays or a loop runs. "
     "One axis at a time, as OLP. Step mode makes one bounded move per push; centre the stick before the next. "
     "Controls must be back at neutral after any page, mode or axis change."));
  left->addWidget(card("TEACH", Rows{
      {"{A}", "Record: review settings first; double press repeats the previous settings"},
      {"{L4}", "Delete the selected waypoint (Undo in the program menu)"},
      {"{X}  {L5}", "Reteach the selected waypoint"},
      {"{Y}  {R4}", "Record a via point"},
  }));
  auto *right = new QVBoxLayout;
  right->setSpacing(14);
  right->addWidget(card("NAVIGATE", Rows{
      {"{L1}  {R1}", "Previous and next page"},
      {"{View}", "Cell page"},
      {"{DPAD↕}", "Disarmed: select row · L2 held: trace one waypoint"},
      {"{DPAD↔}", "Jog speed"},
      {"{L3}", "Joints or Cartesian"},
      {"{RS}", "Without L2: orbit the 3D view; with R2, zoom"},
      {"{R3}", "Reset the 3D view"},
  }, "Right trackpad: pointer and click. Run, Telemetry and Cell are touch. "
     "In the 3D view: drag orbits, pinch zooms, two fingers pan, double-tap resets; + − FIT zoom by touch."));
  right->addWidget(card("STOP", Rows{
      {"{B}  {R5}", "Stop the robot, from every page and dialog"},
  }, "Esc and the red STOP in the header do the same. STOP also disarms.", theme::stopFill));
  right->addStretch();
  columns->addLayout(left, 1);
  columns->addLayout(right, 1);
  layout->addLayout(columns);
  bool touch = false;
  auto *touchButton = theme::button("Touch jog…");
  connect(touchButton, &QPushButton::clicked, &dialog, [&] { touch = true; dialog.accept(); });
  layout->addWidget(touchButton);
  auto *close = theme::button("Close", "A", "primary");
  connect(close, &QPushButton::clicked, &dialog, &QDialog::accept);
  layout->addWidget(close);
  dialog.setMinimumWidth(1180);
  dialog.exec();
  if (touch) jog->showTouchControls();
}

namespace {
// A choice among labelled actions; returns the index chosen, or -1 (B, the
// cancel button). The controller's A accepts the dialog as choice `onA`
// (labelled A); with -1 A picks nothing, as B does.
int askChoice(QWidget *parent, const QString &title, const QString &text, const QStringList &choices,
              const QString &cancelText = "Later", int onA = -1) {
  QDialog dialog(parent);
  auto *layout = new QVBoxLayout(&dialog);
  layout->addWidget(theme::label(title, "section"));
  auto *body = theme::label(text, "");
  body->setWordWrap(true);
  body->setMinimumWidth(620);
  layout->addWidget(body);
  auto *row = new QHBoxLayout;
  int chosen = onA;
  for (int i = 0; i < choices.size(); ++i) {
    auto *b = theme::button(choices[i], i == onA ? "A" : QString(), i == 0 ? "primary" : "");
    QObject::connect(b, &QPushButton::clicked, &dialog, [&dialog, &chosen, i] { chosen = i; dialog.accept(); });
    row->addWidget(b);
  }
  auto *later = theme::button(cancelText, "B");
  QObject::connect(later, &QPushButton::clicked, &dialog, &QDialog::reject);
  row->addWidget(later);
  layout->addLayout(row);
  return dialog.exec() == QDialog::Accepted ? chosen : -1;
}
} // namespace

void Window::renderSync(const QJsonObject &view) {
  const QString state = view["state"].toString();
  QString text = view["short"].toString();
  const QString editors = view["editorsText"].toString();
  if (!editors.isEmpty()) text += " · " + editors.toUpper();
  syncState->setText(text);
  syncState->setToolTip(view["label"].toString() + "\n" + view["detail"].toString());
  theme::setState(syncState, "tone", state == "conflict" || state == "error" ? "danger" : "");
  if (state == "conflict") {
    const QJsonObject conflict = view["conflict"].toObject();
    const QString key = conflict["programId"].toString() + "@" + QString::number(conflict["revision"].toDouble());
    if (key != conflictAsked) {
      conflictAsked = key;
      QMetaObject::invokeMethod(this, [this, conflict] { askSyncConflict(conflict); }, Qt::QueuedConnection);
    }
  } else {
    conflictAsked.clear();
  }
}

void Window::askSyncConflict(const QJsonObject &conflict) {
  const bool deleted = conflict["kind"].toString() == "deleted";
  const int choice = askChoice(this, "PROGRAM CONFLICT", conflict["text"].toString(),
                               {deleted ? "KEEP IT ON THE DECK" : QString("RELOAD THEIRS (REV %1)").arg(conflict["revision"].toDouble()),
                                "KEEP MINE AS A COPY"});
  if (choice == 0) sync->resolve("theirs");
  else if (choice == 1) sync->resolve("copy");
  else notify("Conflict left open: your edits stay on the Deck; the header says CONFLICT until you choose", true);
}

void Window::askSyncOffer(const QJsonObject &offer) {
  ProgramPicker picker(offer, this);
  if (picker.exec() != QDialog::Accepted) {
    const QString local = offer["local"].toObject()["name"].toString();
    notify(local.isEmpty() ? QString("OPEN PROGRAM lists %1's programs").arg(offer["host"].toString())
                           : QString("Kept \"%1\" on the Deck only; OPEN PROGRAM lists %2's programs").arg(local, offer["host"].toString()),
           false);
    return;
  }
  if (picker.action() == "new") {
    newProgram();
    return;
  }
  try {
    notify(core.call("cellSyncPick", {picker.action(), picker.programId(), double(QDateTime::currentMSecsSinceEpoch())}).toString(), false);
  } catch (const OlpError &error) {
    notify(error.what(), true);
  }
  sync->tick();
}

void Window::cellProgramOpened(const QString &message) {
  machine.clearRunProgress();
  // The named SAVE AS file is kept only while it holds the open program.
  const QString kept = programfiles::keepNamed(programPath, core.call("projectIdentity").toString(), [this](const QString &path) {
    QFile file(path);
    return file.open(QIODevice::ReadOnly) ? core.call("projectIdentityOf", {QString::fromUtf8(file.readAll())}).toString() : QString();
  });
  if (kept != programPath) setProgramPath(kept);
  autosave();
  refresh();
  notify(message, false);
}

QString Window::programSwitchLock() {
  try {
    return core.call("cellSyncRunLock", {runView(), machine.runningProgram()}).toString();
  } catch (const OlpError &error) {
    return QString("The robot's state cannot be read (%1); STOP before opening another program").arg(error.what());
  }
}

void Window::openAnyProgram() {
  // No store on this OLP server: the Deck's own files are all there is.
  if (sync->view()["state"].toString() == "disabled") {
    openProgram();
    return;
  }
  QJsonObject picker;
  try {
    picker = core.call("cellSyncPicker", {double(QDateTime::currentMSecsSinceEpoch()), programSwitchLock()}).toObject();
  } catch (const OlpError &error) {
    notify(error.what(), true);
    return;
  }
  ProgramPicker dialog(picker, this, ProgramPicker::Mode::Open, options.programs);
  if (dialog.exec() != QDialog::Accepted) return;
  if (dialog.action() == "files") openProgram();
  else if (dialog.action() == "open") openCellProgram(dialog.programId());
}

void Window::openCellProgram(const QString &programId) {
  // Asked again at the press: the robot may have started since the list opened.
  const QString lock = programSwitchLock();
  QJsonObject check;
  try {
    check = core.call("cellSyncOpenCheck", {programId, double(QDateTime::currentMSecsSinceEpoch()), lock}).toObject();
  } catch (const OlpError &error) {
    notify(error.what(), true);
    return;
  }
  if (!check["blocked"].toString().isEmpty()) {
    notify(check["blocked"].toString(), true);
    return;
  }
  if (check["here"].toBool()) {
    notify("That program is already open", false);
    return;
  }
  QString choice = "open";
  const QJsonObject draft = check["draft"].toObject();
  if (!draft.isEmpty()) {
    // The Deck keeps one program file: opening another replaces the edits.
    // Save first (A) or discard them; B keeps them and opens nothing.
    const QString host = sync->view()["host"].toString().toUpper();
    const QString refused = draft["saveRefused"].toString();
    QStringList choices;
    if (refused.isEmpty()) choices << "SAVE TO " + host + ", THEN OPEN";
    choices << "DISCARD MY EDITS";
    const QString text = draft["text"].toString() + (refused.isEmpty() ? QString() : "\nSaving is not possible now: " + refused + ".");
    const int c = askChoice(this, "UNSAVED EDITS", text, choices, "Cancel", refused.isEmpty() ? 0 : -1);
    if (c < 0) {
      notify("Nothing opened: the edits stay on the Deck", false);
      return;
    }
    choice = refused.isEmpty() && c == 0 ? "save" : "discard";
  }
  try {
    notify(sync->open(programId, choice, lock), false);
  } catch (const OlpError &error) {
    notify(error.what(), true);
  }
}
