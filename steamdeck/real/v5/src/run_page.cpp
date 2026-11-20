#include "pages.hpp"
#include "theme.hpp"
#include "widgets.hpp"
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QSaveFile>
#include <QSlider>
#include <cmath>

namespace {
const QString weldPlan = "/api/offline-programming/v1/weld-plan";
// A plan is the weld planner's seam search, both trajopt stages and the
// verifier on the workstation's GPU; OLP's browser waits without a bound.
// Unverified bound: 30 minutes.
constexpr int planTimeoutMs = 30 * 60 * 1000;
constexpr int visibleNodes = 4;
// Preview frames every 1/30 s of plan time, replayed at real time.
constexpr int previewTickMs = 33;
} // namespace

RunPage::RunPage(PageContext &context) : ctx(context) {
  auto *layout = new QHBoxLayout(this);
  layout->setContentsMargins(12, 10, 12, 8);
  layout->setSpacing(14);

  auto *left = new QVBoxLayout;
  viewLayout = new QVBoxLayout;
  viewLayout->setContentsMargins(0, 0, 0, 0);
  left->addLayout(viewLayout, 1);
  auto *preview = new QHBoxLayout;
  previewButton = theme::button("▶  Preview plan");
  previewButton->setMinimumWidth(170);
  scrubber = new QSlider(Qt::Horizontal);
  scrubber->setRange(0, 1000);
  scrubber->setMinimumHeight(44);
  previewTime = theme::label("", "value");
  previewTime->setMinimumWidth(120);
  preview->addWidget(previewButton);
  preview->addWidget(scrubber, 1);
  preview->addWidget(previewTime);
  left->addLayout(preview);
  layout->addLayout(left, 11);
  previewTimer = new QTimer(this);
  previewTimer->setInterval(previewTickMs);
  connect(previewButton, &QPushButton::clicked, this, [this] { previewing && previewT < durationS ? stopPreview() : startPreview(); });
  connect(previewTimer, &QTimer::timeout, this, [this] {
    const double t = previewFrom + (QDateTime::currentMSecsSinceEpoch() - previewStartMs) / 1000.0;
    if (t >= durationS) {
      showFrame(durationS);
      previewTimer->stop();
      previewButton->setText("▶  REPLAY");
      return;
    }
    showFrame(t);
  });
  connect(scrubber, &QSlider::sliderMoved, this, [this](int value) {
    if (frames.isEmpty()) startPreview();
    previewTimer->stop();
    previewing = true;
    previewButton->setText("■  END PREVIEW");
    showFrame(durationS * value / 1000.0);
  });

  // Three cards, top to bottom in the order they are used: the plan, the
  // machine run, and the program with its executing node.
  auto *rightContent = new QWidget;
  auto *right = new QVBoxLayout(rightContent);
  right->setContentsMargins(0, 0, 8, 0);
  right->setSpacing(10);
  auto planCard = theme::card("");
  auto runCard = theme::card("2 · Arm and run");
  auto nodesCard = theme::card("");
  auto *head = new QHBoxLayout;
  head->addWidget(theme::label("1 · Plan", "section"));
  planBadge = theme::label("", "badge");
  head->addWidget(planBadge);
  head->addStretch();
  planCard.body->addLayout(head);
  planText = theme::label("", "caption");
  planText->setWordWrap(true);
  planCard.body->addWidget(planText);
  auto *planRow = new QHBoxLayout;
  planButton = theme::button("Plan program", "weld planner", "primary");
  planButton->setMinimumHeight(52);
  connect(planButton, &QPushButton::clicked, this, [this] { plan(); });
  planRow->addWidget(planButton, 1);
  // A connected PLAN starts from the robot's measured pose. This is the
  // operator's alternative for the next PLAN only: the program's own start,
  // then Go to plan start before Load and Play (weld_plan.go).
  fromStartButton = theme::button("From program start", "not the robot's pose");
  fromStartButton->setCheckable(true);
  fromStartButton->setMinimumHeight(52);
  connect(fromStartButton, &QPushButton::toggled, this, [this](bool on) {
    ctx.core.call("setPlanFromProgramStart", {on});
    // A failed attempt described its old start mode. Do not leave advice to
    // choose program start beside an already selected button. Retained-plan
    // success and independent machine/Run fault evidence keep their owners.
    if (!planning && planSuccessDigest.isEmpty()) {
      planNote.clear();
      planNoteProgram.clear();
    }
    refresh();
  });
  planRow->addWidget(fromStartButton);
  planCard.body->addLayout(planRow);
  planStatus = theme::label("", "reason");
  planStatus->setObjectName("planStatus");
  planStatus->setWordWrap(true);
  planCard.body->addWidget(planStatus);
  right->addWidget(planCard.frame);
  auto *armRow = new QHBoxLayout;
  programArmButton = theme::button("Arm for program", "no L2 hold needed");
  programArmButton->setObjectName("programArmButton");
  programArmButton->setMinimumHeight(56);
  programArmNote = theme::label("Jog and Trace still use L2. STOP disarms.", "caption");
  programArmNote->setWordWrap(true);
  armRow->addWidget(programArmButton, 1);
  armRow->addWidget(programArmNote, 1);
  // Shown only once the connected cell's OLP offers program arming.
  programArmButton->setVisible(false);
  programArmNote->setVisible(false);
  runCard.body->addLayout(armRow);
  connect(programArmButton, &QPushButton::clicked, this, [this] {
    if (toggleProgramArm) toggleProgramArm();
    refresh();
  });
  auto *mode = new QHBoxLayout;
  dryButton = theme::button("Dry run", "real motion · outputs off");
  outputsButton = theme::button("Program outputs", "as planned");
  for (auto *b : {dryButton, outputsButton}) {
    b->setCheckable(true);
    mode->addWidget(b, 1);
  }
  connect(dryButton, &QPushButton::clicked, this, [this] { ctx.core.call("setRunMode", {false}); refresh(); });
  connect(outputsButton, &QPushButton::clicked, this, [this] { ctx.core.call("setRunMode", {true}); refresh(); });
  runCard.body->addLayout(mode);
  // Run once, Run N times or Loop until stopped (owner 2026-10-11): chosen
  // before PLAN, which then plans the planner's return to the start, and RUN.
  auto *repeat = new QHBoxLayout;
  onceButton = theme::button("Once");
  countButton = theme::button("× 5");
  loopButton = theme::button("Loop", "until stopped");
  lessButton = theme::button("−");
  moreButton = theme::button("+");
  for (auto *b : {onceButton, countButton, loopButton}) {
    b->setCheckable(true);
    repeat->addWidget(b, 2);
  }
  for (auto *b : {lessButton, moreButton}) repeat->addWidget(b, 1);
  runCard.body->addLayout(repeat);
  connect(onceButton, &QPushButton::clicked, this, [this] { ctx.core.call("setRepeat", {"once"}); refresh(); });
  connect(countButton, &QPushButton::clicked, this, [this] { ctx.core.call("setRepeat", {"count"}); refresh(); });
  connect(loopButton, &QPushButton::clicked, this, [this] { ctx.core.call("setRepeat", {"loop"}); refresh(); });
  const auto step = [this](int by) {
    const auto chosen = ctx.runView()["repeat"].toObject();
    ctx.core.call("setRepeat", {"count", chosen["count"].toInt() + by});
    refresh();
  };
  connect(lessButton, &QPushButton::clicked, this, [step] { step(-1); });
  connect(moreButton, &QPushButton::clicked, this, [step] { step(1); });
  // Run program is the one press (owner 2026-10-11): OLP moves to the
  // program's start, loads, arms and plays, asking before it arms or moves.
  // Go to plan start, Load to robot and PLAY fold under ADVANCED.
  auto *go = new QHBoxLayout;
  runButton = theme::button("▶  Run program", "asks before it arms or moves", "primary");
  finishButton = theme::button("Finish cycle", "ends at start");
  stopButton = theme::button("■  STOP", "B · R5", "stop");
  for (auto *b : {runButton, finishButton, stopButton}) b->setMinimumHeight(56);
  go->addWidget(runButton, 2);
  go->addWidget(finishButton, 1);
  go->addWidget(stopButton, 1);
  runCard.body->addLayout(go);
  connect(runButton, &QPushButton::clicked, this, [this] { runProgram(); });
  connect(finishButton, &QPushButton::clicked, this, [this] { ctx.machine.finishLoop(); });
  loopLine = theme::label("", "value");
  loopLine->setWordWrap(true);
  runCard.body->addWidget(loopLine);
  outputsLine = theme::label("", "caption");
  outputsLine->setObjectName("runOutputs");
  outputsLine->setWordWrap(true);
  outputsLine->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  outputsLine->hide();
  runCard.body->addWidget(outputsLine);
  advancedButton = theme::button("Advanced · step by step");
  advancedButton->setCheckable(true);
  runCard.body->addWidget(advancedButton);
  advanced = new QWidget;
  auto *steps = new QHBoxLayout(advanced);
  steps->setContentsMargins(0, 0, 0, 0);
  startButton = theme::button("Go to plan start");
  loadButton = theme::button("Load to robot");
  playButton = theme::button("▶  PLAY");
  steps->addWidget(startButton, 1);
  steps->addWidget(loadButton, 1);
  steps->addWidget(playButton, 1);
  advanced->setVisible(false);
  runCard.body->addWidget(advanced);
  connect(advancedButton, &QPushButton::toggled, this, [this](bool on) {
    advanced->setVisible(on);
    refresh();
  });
  connect(startButton, &QPushButton::clicked, this, [this] { goToPlanStart(); });
  connect(loadButton, &QPushButton::clicked, this, [this] { load(); });
  connect(playButton, &QPushButton::clicked, this, [this] {
    stopPreview();
    if (!planning) ctx.machine.playProgram();
  });
  connect(stopButton, &QPushButton::clicked, this, [this] { ctx.machine.stop(); });
  readout = theme::label("", "value");
  blocker = theme::label("", "reason");
  result = theme::label("", "caption");
  result->setObjectName("runResult");
  for (auto *l : {readout, blocker, result}) {
    l->setWordWrap(true);
    runCard.body->addWidget(l);
  }
  right->addWidget(runCard.frame);

  nodes = new QTableWidget(visibleNodes, 3);
  nodes->setHorizontalHeaderLabels({"#", "Step", "Status"});
  nodes->setShowGrid(false);
  nodes->verticalHeader()->hide();
  nodes->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
  nodes->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  nodes->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Fixed);
  nodes->setColumnWidth(0, 44);
  nodes->setColumnWidth(2, 200);
  nodes->verticalHeader()->setDefaultSectionSize(28);
  nodes->setEditTriggers(QAbstractItemView::NoEditTriggers);
  nodes->setSelectionMode(QAbstractItemView::NoSelection);
  nodes->setFocusPolicy(Qt::NoFocus);
  nodes->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  nodes->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  nodes->setFixedHeight(28 * visibleNodes + nodes->horizontalHeader()->sizeHint().height() + 2);
  auto *nodesHead = new QHBoxLayout;
  nodesHead->addWidget(theme::label("2 · Review program", "section"));
  nodesHead->addStretch();
  nodesCard.body->addLayout(nodesHead);
  nodesCard.body->addWidget(nodes);
  right->insertWidget(1, nodesCard.frame);
  right->addStretch();
  auto *scroll = scrollingPanel(rightContent);
  scroll->setObjectName("runControlsScroll");
  layout->addWidget(scroll, 10);
}

void RunPage::attachRobot(RobotView *view) {
  robot = view;
  viewLayout->addWidget(view);
  view->setCamera(RobotView::Camera::Orbit);
  view->show();
}

QString RunPage::keptBytes(const QString &digest) const {
  return QString::fromLatin1(keptBinary(digest).toBase64());
}

QByteArray RunPage::keptBinary(const QString &digest) const {
  if (digest.isEmpty()) return {};
  QFile file(ctx.trajectoryPath(digest));
  if (!file.open(QIODevice::ReadOnly)) return {};
  return file.readAll();
}

bool RunPage::kept(const QString &digest) const {
  return !digest.isEmpty() && QFile::exists(ctx.trajectoryPath(digest));
}

void RunPage::plan() {
  if (planning) return;
  // OLP's reason PLAN cannot run now (another robot's program, a held jog or
  // a move in flight): said, not posted.
  const QString blocked = ctx.runView()["planBlocked"].toString();
  if (!blocked.isEmpty()) {
    ctx.notify(blocked, true);
    refresh();
    return;
  }
  stopPreview();
  const auto program = ctx.core.call("view").toObject();
  const int revision = program["revision"].toInt();
  const QString programId = ctx.core.call("projectIdentity").toString();
  // What this PLAN asks for: the program's start (chosen), else the robot's
  // measured pose whenever a machine is connected.
  const bool fromStart = ctx.runView()["planFromProgramStart"].toBool();
  const bool measured = ctx.machine.connected && !fromStart;
  planning = true;
  planNoteProgram = programId;
  planSuccessDigest.clear();
  ctx.machine.setForegroundPlanning(true);
  planNote = "Packing the program for the weld planner";
  refresh();
  ctx.core.callAsync("planBody", {}, [this, programId, revision, fromStart, measured](std::optional<QJsonValue> body, const QString &error) {
    QString failure = error;
    // New/Open may run while the digest is being packed. Do not queue a
    // planner request for a program the operator has already left.
    if (body) try {
      if (ctx.core.call("projectIdentity").toString() != programId ||
          ctx.core.call("view").toObject()["revision"].toInt() != revision)
        failure = "The program changed while it was planning; plan it again";
    } catch (const OlpError &e) { failure = e.what(); }
    if (!body || !failure.isEmpty()) {
      planning = false;
      ctx.machine.setForegroundPlanning(false);
      planNote = "Planning failed: " + failure;
      ctx.notify(planNote, true);
      refresh();
      return;
    }
    planNote = "Planning · seam search, weld and connecting trajopt, verify";
    refresh();
    ctx.client.post(weldPlan, *body, [this, programId, revision, fromStart, measured](const OlpClient::Reply &reply) {
      planning = false;
      ctx.machine.setForegroundPlanning(false);
      if (!reply.ok()) {
        // The server's code and sentence, verbatim, on the status line; the
        // plan card says what to do (one instruction for a jog still held).
        const QString code = reply.body.toObject().value("code").toString();
        QString message = reply.error, verbatim = code.isEmpty() ? reply.error : code + ": " + reply.error;
        try {
          const auto failure = ctx.core.call("planFailure", {code, reply.error, reply.body.toObject().value("dense_error"), measured}).toObject();
          message = failure["message"].toString();
          verbatim = failure["verbatim"].toString();
        } catch (const OlpError &) {
        }
        planNote = code == "planning_start_moving" ? message : "Planning failed: " + message;
        ctx.notify("Planning refused · " + verbatim, true);
        refresh();
        return;
      }
      try {
        auto metadata = reply.body.toObject();
        metadata.remove("dense_blob_base64");
        const QString digest = ctx.core.call("applyPlan", {metadata, revision, programId}).toString();
        // The bytes, kept here as the browser keeps them: the motion server
        // owes nobody a copy later; Go to plan start, Preview and Load read them.
        const QByteArray bytes = QByteArray::fromBase64(reply.body.toObject()["dense_blob_base64"].toString().toLatin1());
        if (!bytes.isEmpty()) {
          const QString path = ctx.trajectoryPath(digest);
          QDir().mkpath(QFileInfo(path).absolutePath());
          QSaveFile file(path);
          if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
            ctx.notify("Keeping the trajectory failed: " + file.errorString(), true);
        }
        const auto dense = reply.body.toObject()["dense"].toObject();
        planNote = ctx.core.call("plannedNote", {dense["total_sample_count"].toInt(), dense["total_duration_s"].toDouble(), fromStart}).toString();
        planSuccessProgram = programId; planSuccessRevision = revision; planSuccessDigest = digest;
        ctx.notify(planNote, false);
        ctx.programChanged();
      } catch (const OlpError &error) {
        planNote = QString("Planning failed: %1").arg(error.what());
        ctx.notify(planNote, true);
        refresh();
      }
    }, planTimeoutMs);
  });
}

void RunPage::load() {
  if (planning) return;
  QJsonObject identity;
  try {
    identity = ctx.core.call("loadBody").toObject();
  } catch (const OlpError &error) {
    ctx.notify(error.what(), true);
    return;
  }
  stopPreview();
  loading = true;
  result->setText("Loading the trajectory to the robot…");
  refresh();
  ctx.machine.loadProgram(identity, [this, identity](bool loaded) {
    // Only when the planner no longer holds the digest does the copy kept
    // here go along (the server checks it is this plan). An OLP server
    // older than that fallback refuses a body that large.
    const QString bytes = loaded || ctx.machine.lastError() != "dense_blob_not_found" ? QString() : keptBytes(identity["trajectory_digest"].toString());
    if (bytes.isEmpty()) {
      loading = false;
      result->setText(loaded ? "Loaded. Play when ready; STOP ends it." : "");
      refresh();
      return;
    }
    QJsonObject withCopy = identity;
    withCopy["dense_blob_base64"] = bytes;
    result->setText("The planner no longer has this plan; loading the copy kept on the pendant…");
    ctx.machine.loadProgram(withCopy, [this](bool again) {
      loading = false;
      result->setText(again ? "Loaded. The pendant's kept copy stood in for the planner's; Play when ready, STOP ends it." : "");
      refresh();
    });
  });
}

// One press of OLP's Run program (Run again once this plan completed). Only
// when the planner no longer holds the digest does the copy kept here go
// along, as on Load.
void RunPage::runProgram(const QString &keptCopy) {
  if (planning) return;
  QJsonObject base;
  try {
    base = ctx.core.call("loadBody").toObject();
  } catch (const OlpError &error) {
    ctx.notify(error.what(), true);
    return;
  }
  if (!keptCopy.isEmpty()) base["dense_blob_base64"] = keptCopy;
  stopPreview();
  result->setText(keptCopy.isEmpty() ? "Run program…" : "The planner no longer has this plan; running the copy kept on the pendant…");
  ctx.machine.runProgram(base, [this, keptCopy](const QJsonObject &answer) {
    if (answer["state"] == "refused" && answer["code"] == "dense_blob_not_found" && keptCopy.isEmpty()) {
      const QString bytes = keptBytes(ctx.runView()["planDigest"].toString());
      if (!bytes.isEmpty()) {
        runProgram(bytes);
        return;
      }
    }
    result->setText(answer["text"].toString());
    refresh();
  });
  refresh();
}

void RunPage::clearRunProgress(const QString &message) {
  if (result->text() == message) { result->clear(); result->hide(); }
}

void RunPage::goToPlanStart() {
  if (planning) return;
  const auto run = ctx.runView();
  const QByteArray bytes = keptBinary(run["planDigest"].toString());
  if (bytes.isEmpty()) {
    result->setText("Selected plan data is unavailable");
    return;
  }
  const auto start = ctx.core.callBytes("planStart", bytes, {ctx.machine.view()["target"].toObject()["model_id"].toString()}).toObject();
  if (start.contains("error")) {
    result->setText(start["error"].toString());
    return;
  }
  stopPreview();
  planStartActive = true;
  result->setText("Moving to plan start…");
  refresh();
  ctx.machine.moveToPositions(start["positions"].toArray(), [this](bool moved) {
    planStartActive = false;
    result->setText(moved ? "Move completed. Load to robot, then Play." : ctx.machine.motionReason);
    refresh();
  });
  if (!ctx.machine.moveBusy()) {
    planStartActive = false;
    result->setText(ctx.machine.motionReason);
    refresh();
  }
}

void RunPage::startPreview() {
  const auto run = ctx.runView();
  const QString digest = run["planDigest"].toString();
  if (digest != previewDigest || frames.isEmpty()) {
    const QByteArray bytes = keptBinary(digest);
    if (bytes.isEmpty()) {
      result->setText(digest.isEmpty() ? "Plan the program first" : "This plan's trajectory is not on the pendant; plan it here to preview it");
      return;
    }
    try {
      const auto path = ctx.core.callBytes("previewPath", bytes).toObject();
      frames = path["frames"].toArray();
      jointOrder.clear();
      for (const auto &id : path["jointOrder"].toArray()) jointOrder << id.toString();
      durationS = path["durationS"].toDouble();
      previewDigest = digest;
    } catch (const OlpError &error) {
      result->setText(error.what());
      return;
    }
  }
  if (frames.isEmpty() || !robot) return;
  previewing = true;
  if (previewT >= durationS) previewT = 0;
  previewFrom = previewT;
  previewStartMs = QDateTime::currentMSecsSinceEpoch();
  previewTimer->start();
  previewButton->setText("■  END PREVIEW");
}

void RunPage::stopPreview() {
  previewTimer->stop();
  previewing = false;
  previewT = 0;
  previewButton->setText("▶  Preview plan");
  if (robot) robot->update();
}

void RunPage::renderOutputs(const QJsonObject &view) {
  if (!view["visible"].toBool()) {
    outputsLine->clear();
    outputsLine->hide();
    return;
  }
  QStringList lines;
  for (const auto &line : view["lines"].toArray()) lines << line.toString();
  const QString tone = view["tone"].toString();
  const QString colour = tone == "fault" ? theme::danger : tone == "on" ? theme::ok : tone == "armed" ? theme::accent : theme::value;
  theme::setSheet(outputsLine, QString("QLabel#runOutputs { color: %1; font-size: 14px; border-left: 3px solid %1; padding-left: 8px; }").arg(colour));
  outputsLine->setText(lines.join("\n"));
  outputsLine->show();
}

QString RunPage::outputsText() const { return outputsLine->isVisibleTo(this) ? outputsLine->text() : QString(); }

void RunPage::showFrame(double t) {
  if (frames.isEmpty() || !robot) return;
  previewT = std::clamp(t, 0.0, durationS);
  // Frames are in time order: the last one at or before t.
  int lo = 0, hi = int(frames.size()) - 1;
  while (lo < hi) {
    const int mid = (lo + hi + 1) / 2;
    if (frames[mid].toObject()["t"].toDouble() <= previewT) lo = mid;
    else hi = mid - 1;
  }
  const auto q = frames[lo].toObject()["q"].toArray();
  QJsonObject joints;
  for (int i = 0; i < jointOrder.size() && i < q.size(); ++i) joints[jointOrder[i]] = q[i].toDouble();
  robot->joints = joints;
  robot->live = false;
  robot->update();
  scrubber->blockSignals(true);
  scrubber->setValue(durationS > 0 ? int(1000 * previewT / durationS) : 0);
  scrubber->blockSignals(false);
  previewTime->setText(QString("%1 / %2 s").arg(previewT, 0, 'f', 1).arg(durationS, 0, 'f', 1));
}

void RunPage::refresh() {
  // Program arming exists only on a cell whose manifest allows unattended
  // loops; elsewhere the Run card is held-L2 only. A latched permit stays
  // visible so it can always be disarmed here.
  const bool programArmShown = ctx.machine.programArmOffered() || ctx.machine.programArmPermitted();
  programArmButton->setVisible(programArmShown);
  programArmNote->setVisible(programArmShown);
  QJsonObject run;
  try {
    run = ctx.runView();
    // Every attempt belongs to its program. Keep refusal evidence through
    // edits, but do not present another program's error as this one's state.
    // Successful notes additionally describe one exact retained revision.
    const QString programId = ctx.core.call("projectIdentity").toString();
    if ((!planNoteProgram.isEmpty() && programId != planNoteProgram) ||
        (!planSuccessDigest.isEmpty() && (run["planDigest"].toString() != planSuccessDigest ||
        programId != planSuccessProgram ||
        ctx.core.call("view").toObject()["revision"].toInt() != planSuccessRevision))) {
      planNote.clear(); planSuccessDigest.clear();
    }
  } catch (const OlpError &error) {
    readout->setText(error.what());
    return;
  }
  const auto controls = run["controls"].toObject();
  const auto flow = ctx.machine.view()["flow"].toObject();
  const bool programArmed = ctx.machine.programArmPermitted();
  const bool armBusy = ctx.machine.armRequestBusy();
  programArmButton->setText(programArmed ? (flow["armed"].toBool() ? "Disarm program\nSTOP also disarms" : "Cancel program arm\narming…")
                                       : "Arm for program\nno L2 hold needed");
  programArmButton->setEnabled(programArmed || (ctx.machine.connected && !armBusy &&
      !ctx.machine.maintenance() && ctx.machine.cellMismatch().isEmpty() && !ctx.machine.armHoldPermitted() &&
      !ctx.machine.programExecutionActive() && flow["canArm"].toBool()));
  theme::setState(programArmButton, "tone", programArmed ? "armed" : "");
  programArmNote->setText(programArmed ? "Program controls enabled without L2. STOP or Disarm ends it."
      : ctx.machine.armHoldPermitted() ? "Release L2 before selecting program arm."
      : "Jog and Trace still use L2. STOP disarms.");
  const QString plan = run["plan"].toString();
  planBadge->setText(plan);
  theme::setState(planBadge, "tone", plan == "PLANNED" ? "ok" : plan == "STALE" ? "caution" : "");
  planText->setText(run["planText"].toString());
  const QString planBlocked = run["planBlocked"].toString();
  planButton->setEnabled(planBlocked.isEmpty() && !planning && !controls["live"].toBool());
  planButton->setText(planning ? "Planning…\nweld planner" : "Plan program\nweld planner");
  // Offered only while connected (offline plans always start at the
  // program's start); cleared by OLP once a plan is kept.
  fromStartButton->setVisible(ctx.machine.connected);
  fromStartButton->setEnabled(!planning && !controls["live"].toBool());
  fromStartButton->blockSignals(true);
  fromStartButton->setChecked(run["planFromProgramStart"].toBool());
  fromStartButton->blockSignals(false);
  planStatus->setText(!planning && !planBlocked.isEmpty() ? planBlocked : planNote);
  const bool locked = planning || ctx.machine.requestInFlight() || ctx.machine.moveBusy() || loading;
  dryButton->setChecked(run["dryRun"].toBool());
  outputsButton->setChecked(!run["dryRun"].toBool());
  for (auto *b : {dryButton, outputsButton}) b->setEnabled(!locked && !controls["live"].toBool());
  const auto planStart = run["planStart"].toObject();
  startButton->setText(planStart["text"].toString());
  startButton->setEnabled(!planning && !planStart["disabled"].toBool() && !ctx.machine.moveBusy());
  startButton->setToolTip(planStart["title"].toString());
  const auto loadControl = controls["load"].toObject();
  loadButton->setText(loading ? "LOADING…" : loadControl["text"].toString());
  loadButton->setEnabled(!planning && !loadControl["disabled"].toBool() && !loading);
  const auto play = controls["play"].toObject();
  playButton->setEnabled(!planning && !play["disabled"].toBool());
  playButton->setText(controls["state"] == "paused" ? "▶  RESUME" : "▶  PLAY");
  const auto runControl = run["runProgram"].toObject();
  runButton->setText(QString("▶  %1\nasks before it arms or moves").arg(runControl["text"].toString()));
  runButton->setEnabled(!runControl["disabled"].toBool() && !locked && !ctx.machine.runningProgram());
  runButton->setToolTip(runControl["title"].toString());
  const auto chosen = run["repeat"].toObject();
  const QString mode = chosen["mode"].toString();
  const bool looping = run["looping"].toBool();
  onceButton->setChecked(mode == "once");
  countButton->setChecked(mode == "count");
  loopButton->setChecked(mode == "loop");
  countButton->setText(QString("× %1").arg(chosen["count"].toInt()));
  for (auto *b : {onceButton, countButton, loopButton}) b->setEnabled(!looping && !locked);
  for (auto *b : {lessButton, moreButton}) b->setEnabled(mode == "count" && !looping && !locked);
  const auto finish = run["finish"].toObject();
  finishButton->setVisible(finish["visible"].toBool());
  finishButton->setEnabled(!finish["disabled"].toBool());
  finishButton->setToolTip(finish["title"].toString());
  loopLine->setText(run["loop"].toString());
  stopButton->setEnabled(!controls["stop"].toObject()["disabled"].toBool());
  const auto session = run["session"].toObject();
  readout->setText(ctx.machine.connected
                       ? (session.isEmpty() ? QString("No run session on this pendant") : QString("%1 · session %2").arg(controls["state"].toString(), session["id"].toString().left(8)))
                       : "No machine connected · Cell page");
  QString why = controls["loadBlocker"].toString();
  if (why.isEmpty() && !ctx.machine.connected) why = "Connect a machine on the Cell page to run a program";
  else if (why.isEmpty() && runControl["disabled"].toBool() && !controls["live"].toBool() && !ctx.machine.runningProgram())
    why = runControl["title"].toString();
  else if (why.isEmpty() && advanced->isVisible() && loadControl["disabled"].toBool() && !controls["live"].toBool())
    why = loadControl["title"].toString();
  else if (why.isEmpty() && advanced->isVisible() && play["disabled"].toBool() && controls["current"].toBool() && !controls["live"].toBool())
    why = play["title"].toString();
  blocker->setText(why);
  // The load note belongs to the staged plan; once it runs, the state line says what is happening.
  const QString state = controls["state"].toString();
  if (state != lastState && (state == "playing" || state == "done" || state == "stopped" || state == "aborted"))
    if (result->text().startsWith("Loaded.")) result->clear();
  lastState = state;
  // Empty lines take no room; long states scroll inside the run panel.
  for (QLabel *l : {planText, planStatus, readout, blocker, result, loopLine}) l->setVisible(!l->text().isEmpty());

  // Once per plan and machine, read the plan start so "At plan start" can show.
  const QString key = run["planDigest"].toString() + "|" + ctx.machine.connectedCell;
  if (planStart["eligible"].toBool() && key != planStartKey) {
    planStartKey = key;
    const QByteArray bytes = keptBinary(run["planDigest"].toString());
    if (!bytes.isEmpty()) {
      try {
        ctx.core.callBytes("planStart", bytes, {ctx.machine.view()["target"].toObject()["model_id"].toString()});
      } catch (const OlpError &) {
      }
    }
  } else if (!planStart["eligible"].toBool()) {
    planStartKey.clear();
  }
  const bool onPendant = kept(run["planDigest"].toString());
  previewButton->setEnabled(onPendant || previewing);
  if (!previewing) previewTime->setText(onPendant ? "" : run["planDigest"].toString().isEmpty() ? "no plan" : "plan not on pendant");
  scrubber->setEnabled(onPendant);

  // The program, paged so the executing node is on screen.
  const auto rows = ctx.core.call("view").toObject()["rows"].toArray();
  const QString executing = run["executing"].toString();
  int at = 0;
  for (int i = 0; i < rows.size(); ++i)
    if (rows[i].toObject()["id"] == executing) at = i;
  const int first = executing.isEmpty() ? 0 : std::max(0, at - visibleNodes / 2);
  for (int r = 0; r < visibleNodes; ++r) {
    const int i = first + r;
    const auto row = i < rows.size() ? rows[i].toObject() : QJsonObject{};
    const bool now = !executing.isEmpty() && row["id"] == executing;
    const QStringList texts{row.isEmpty() ? "" : QString::number(row["number"].toInt()),
                            row.isEmpty() ? "" : QString("%1  %2").arg(row["label"].toString(), row["freespace"].toString()),
                            row.isEmpty() ? "" : now ? "▶ EXECUTING" : row["status"].toString()};
    for (int c = 0; c < 3; ++c) {
      auto *item = nodes->item(r, c);
      if (!item) nodes->setItem(r, c, item = new QTableWidgetItem);
      item->setText(texts[c]);
      item->setBackground(now ? QColor(theme::accentSoft) : QColor(theme::content));
      item->setForeground(row.isEmpty() || row["enabled"].toBool(true) ? QColor(theme::fg) : QColor(theme::faint));
    }
  }
}
