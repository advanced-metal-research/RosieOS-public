#include "pages.hpp"
#include "workspace_split.hpp"
#include <QDialog>
#include <QMenu>
#include "jog_live.hpp"
#include "camera_view.hpp"
#include "theme.hpp"
#include "widgets.hpp"
#include "glyphs.hpp"
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QSettings>
#include <QVBoxLayout>
#include <cmath>

namespace {
constexpr double degrees = 180.0 / M_PI;
QString percent(double fraction) { return QString::number(std::round(fraction * 100)) + "%"; }
} // namespace

JogPage::JogPage(PageContext &context) : ctx(context) {
  QSettings settings;
  linearValue = settings.value("jog/linear", 100.).toDouble();
  angularValue = settings.value("jog/angular", 15.).toDouble();
  linearStep = settings.value("jog/linearStep", 1.).toDouble();
  angularStep = settings.value("jog/angularStep", 1.).toDouble();
  for (int i = 0; i < 3; ++i) {
    const QString key = QString("jog/preset/%1/").arg(i);
    SpeedPreset preset{settings.value(key + "linear", linearValue).toDouble(),
                       settings.value(key + "angular", angularValue).toDouble(),
                       settings.value(key + "fraction", QList<double>{.1, .25, 1.}[i]).toDouble()};
    if (!std::isfinite(preset.linear) || preset.linear < .001 || preset.linear > 1000 ||
        !std::isfinite(preset.angular) || preset.angular < .001 || preset.angular > 180 ||
        !std::isfinite(preset.fraction) || preset.fraction < .01 || preset.fraction > 1)
      preset = {100, 15, QList<double>{.1, .25, 1.}[i]};
    presets << preset;
  }
  speedPreset = std::clamp(settings.value("jog/preset/selected", 0).toInt(), 0, 2);
  linearValue = presets[speedPreset].linear; angularValue = presets[speedPreset].angular;
  ctx.machine.setFraction(presets[speedPreset].fraction);
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(12, 4, 12, 4);
  layout->setSpacing(4);
  auto *toolbar = new QHBoxLayout;
  toolbar->setSpacing(8);
  auto *options = theme::button("Touch jog…");
  options->setFixedWidth(110);
  options->setToolTip("On-screen jog buttons, continuous speed and step-distance settings");
  options->setObjectName("touchJogButton");
  connect(options, &QPushButton::clicked, this, [this] { showTouchControls(); });
  toolbar->addWidget(options);
  zeroMain = theme::button("All joints to 0°");
  zeroMain->setObjectName("jogZeroAll");
  zeroMain->setFixedWidth(150);
  zeroMain->setToolTip("Return every joint to zero at the selected jog tool travel and rotation speeds");
  connect(zeroMain, &QPushButton::clicked, this, [this] { ctx.machine.zeroAll(linearValue, angularValue); });
  toolbar->addWidget(zeroMain);
  auto group = [&](const QString &label, const QStringList &names, QList<QPushButton *> &buttons,
                   std::function<void(int)> chosen, int stretch) {
    auto *box = new QWidget;
    auto *column = new QVBoxLayout(box);
    column->setContentsMargins(0, 0, 0, 0);
    column->setSpacing(2);
    if (label == "Jog" || label == "Jog speed") column->addWidget(new GlyphLabel(label + (label == "Jog" ? " {L3}" : " {DPAD↔}"), "toolbarLabel"));
    else column->addWidget(theme::label(label, "toolbarLabel"));
    column->addLayout(segments(names, buttons, chosen));
    for (auto *button : buttons) button->setMinimumHeight(36);
    toolbar->addWidget(box, stretch);
    return box;
  };
  QList<QPushButton *> coordinates;
  group("Jog", {"Joints", "Cartesian"}, coordinates, [this](int i) { setCartesian(i == 1); }, 2);
  jointsTab = coordinates[0]; cartesianTab = coordinates[1];
  jointChoice = new QWidget;
  auto *jointColumn = new QVBoxLayout(jointChoice);
  jointColumn->setContentsMargins(0, 0, 0, 0); jointColumn->setSpacing(2);
  jointColumn->addWidget(theme::label("Selected joint", "toolbarLabel"));
  auto *jointButton = theme::button("J1 ▾");
  jointButton->setObjectName("selectedJogJoint");
  jointColumn->addWidget(jointButton);
  connect(jointButton, &QPushButton::clicked, this, [this, jointButton] {
    ctx.machine.endHold("ui_navigation");
    QMenu menu(this);
    const int count = ctx.machine.view()["joints"].toArray().size();
    for (int i = 0; i < count; ++i) {
      auto *action = menu.addAction(QString("J%1").arg(i + 1), this, [this, i] { selected = i; refresh(); });
      action->setCheckable(true); action->setChecked(i == selected);
    }
    menu.exec(jointButton->mapToGlobal(QPoint(0, jointButton->height())));
  });
  toolbar->addWidget(jointChoice, 2);
  frameChoice = group("Frame", {"Base", "Tool"}, frameButtons, [this](int i) {
    ctx.machine.endHold("frame_change"); tool = i == 1; refresh();
  }, 2);
  modeChoice = group("Mode", {"Hold", "Step"}, modeButtons, [this](int i) {
    ctx.machine.endHold("mode_switch"); mode = i == 1 ? "step" : "hold"; refresh();
  }, 2);
  modeButtons[0]->setObjectName("jogModeHold");
  modeButtons[1]->setObjectName("jogModeStep");
  group("Jog speed", {"Slow", "Medium", "Fast"}, speedButtons, [this](int i) { applySpeedPreset(i); }, 3);
  auto *speedSettings = theme::button("▾");
  speedSettings->setObjectName("jogSpeedSettings");
  speedSettings->setAccessibleName("Edit jog speed presets");
  speedSettings->setToolTip("Edit Slow, Medium and Fast speeds");
  speedSettings->setFixedWidth(36);
  connect(speedSettings, &QPushButton::clicked, this, [this] { showSpeedSettings(); });
  toolbar->addWidget(speedSettings);
  layout->addLayout(toolbar);

  auto *leftPane = new QWidget;
  auto *left = new QVBoxLayout(leftPane);
  left->setContentsMargins(0, 0, 8, 0);
  left->setSpacing(0);
  auto *viewHeader = new QWidget;
  viewHeader->setObjectName("workspaceHeader");
  auto *viewRow = new QVBoxLayout(viewHeader);
  viewRow->setContentsMargins(8, 6, 8, 6);
  viewRow->addLayout(segments({"3D", "Tool view", "Wrist cam", "Cell cam"}, viewTabs, [this](int i) { setLeftView(i); }));
  for (auto *button : viewTabs) button->setMinimumHeight(36);
  left->addWidget(viewHeader);
  leftViews = new QStackedWidget;
  auto *robotPage = new QWidget;
  robot = new RobotView;
  viewLayout = new QVBoxLayout(robotPage);
  viewLayout->setContentsMargins(0, 0, 0, 0);
  viewLayout->addWidget(robot);
  leftViews->addWidget(robotPage);
  leftViews->addWidget(new CameraView(ctx.client.origin(), nullptr, "wrist"));
  leftViews->addWidget(new CameraView(ctx.client.origin()));
  left->addWidget(leftViews, 1);
  programPane = new ProgramPage(ctx, true);
  workspace = new WorkspaceSplit(leftPane, programPane);
  workspace->interacting = [this] { ctx.machine.endHold("ui_navigation"); };
  layout->addWidget(workspace, 1);

  live = new JogLiveStrip;
  layout->addWidget(live);
  // The persistent status footer renders the reason; this label is kept in
  // the touch-controls dialog so that the same refusal is visible there.
  reason = theme::label("", "reason");
  reason->setWordWrap(true);

  // Reuse the established touch controls and their machine gates in a dialog.
  touchDialog = new InWindowDialog(this);
  touchDialog->setObjectName("touchJogControls");
  // A persistent child panel must stay hidden when the main window is first
  // shown; only the Touch jog button should open its modal event loop.
  touchDialog->hide();
  // This panel is reused; keep its existing layout margins across reopenings.
  touchDialog->setProperty("boundedForm", true);
  auto *dialogLayout = new QVBoxLayout(touchDialog);
  auto *dialogHead = new QHBoxLayout;
  dialogHead->addWidget(theme::label("Touch jog", "title"));
  dialogHead->addStretch();
  auto *stop = theme::button("■ STOP", "B · R5", "stop");
  connect(stop, &QPushButton::clicked, this, [this] { ctx.machine.stop(); });
  auto *close = theme::button("Done");
  connect(close, &QPushButton::clicked, touchDialog, &QDialog::accept);
  dialogHead->addWidget(stop);
  dialogHead->addWidget(close);
  dialogLayout->addLayout(dialogHead);
  speedNote = theme::label("", "caption");
  speedNote->setWordWrap(true);
  dialogLayout->addWidget(speedNote);
  dialogLayout->addWidget(reason);
  modes = new QStackedWidget;
  modes->addWidget(jointPage());
  modes->addWidget(cartesianPage());
  dialogLayout->addWidget(modes, 1);
  touchDialog->resize(1180, 650);
  setCartesian(settings.value("jog/cartesian", false).toBool());
  setLeftView(settings.value("jog/view", 0).toInt());
}

void JogPage::showTouchControls() {
  ctx.machine.endHold("ui_navigation");
  refresh();
  touchDialog->exec();
  ctx.machine.endHold("ui_navigation");
}

void JogPage::showViewMenu() {
  workspace->showMenu();
}

void JogPage::attachRobot() {
  viewLayout->addWidget(robot);
  robot->show();
  robot->setCamera(RobotView::Camera(std::min(leftView, 1)));
}

QHBoxLayout *JogPage::segments(const QStringList &names, QList<QPushButton *> &buttons, std::function<void(int)> chosen) {
  auto *row = new QHBoxLayout;
  row->setSpacing(0);
  for (int i = 0; i < names.size(); ++i) {
    auto *b = theme::button(names[i]);
    b->setCheckable(true);
    b->setProperty("compact", true);
    b->setProperty("segment", names.size() == 1 ? "" : i == 0 ? "first" : i == names.size() - 1 ? "last" : "middle");
    b->setMinimumWidth(52);
    connect(b, &QPushButton::clicked, this, [chosen, i] { chosen(i); });
    buttons << b;
    row->addWidget(b, 1);
  }
  return row;
}

void JogPage::setLeftView(int index) {
  leftView = std::clamp(index, 0, 3);
  QSettings().setValue("jog/view", leftView);
  leftViews->setCurrentIndex(leftView < 2 ? 0 : leftView - 1);
  if (leftView < 2) robot->setCamera(RobotView::Camera(leftView));
  for (int i = 0; i < viewTabs.size(); ++i) viewTabs[i]->setChecked(i == leftView);
  refresh();
}

void JogPage::holdTouch(QPushButton *button, std::function<void()> begin) {
  connect(button, &QPushButton::pressed, this, [this, begin] {
    // A fresh touch chooses its own motion: never inherit a controller hold.
    if (ctx.machine.holdActive()) { ctx.machine.endHold("button_released"); return; }
    begin();
  });
  connect(button, &QPushButton::released, this, [this] { ctx.machine.endHold("button_released"); });
}

QWidget *JogPage::jointPage() {
  auto *page = new QWidget;
  auto *layout = new QVBoxLayout(page);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(2);
  auto *header = new QHBoxLayout;
  header->setContentsMargins(4, 0, 4, 0);
  header->setSpacing(4);
  auto column = [&](const QString &text, int width) {
    auto *l = theme::label(text, "kicker");
    if (width) l->setFixedWidth(width);
    header->addWidget(l, width ? 0 : 1);
  };
  column("AXIS", 44);
  column("POSITION", 96);
  header->addWidget(new GlyphLabel("JOG {LS↕}  ·  choose joint above", "kicker"), 1);
  column("MOVE BY ANGLE", 142);
  column("HOME", 150);
  layout->addLayout(header);
  for (int i = 0; i < 9; ++i) {
    JointRow row;
    row.row = new QWidget;
    row.row->setObjectName("jointRow");
    // Rows share the column's height: taller targets on the Deck, never a scroll bar.
    row.row->setMinimumHeight(40);
    row.row->setMaximumHeight(62);
    row.row->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    auto *h = new QHBoxLayout(row.row);
    h->setContentsMargins(4, 2, 4, 2);
    h->setSpacing(4);
    row.name = theme::label(QString("J%1").arg(i + 1), "title");
    row.name->setFixedWidth(44);
    row.position = theme::label("—", "value");
    row.position->setFixedWidth(96);
    row.position->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    row.minus = theme::button("−", {}, "jog");
    row.plus = theme::button("+", {}, "jog");
    auto *angle = row.angle = theme::button("± 10°");
    row.go = theme::button("GO");
    row.home = theme::label("", "caption");
    row.home->setFixedWidth(150);
    row.home->setWordWrap(true);
    row.home->setStyleSheet("font-size: 11px;");
    for (auto *b : {row.minus, row.plus, angle, row.go}) {
      b->setProperty("compact", true);
      b->setSizePolicy(b->sizePolicy().horizontalPolicy(), QSizePolicy::Expanding);
    }
    angle->setFixedWidth(88);
    row.go->setFixedWidth(50);
    h->addWidget(row.name);
    h->addWidget(row.position);
    h->addSpacing(8);
    h->addWidget(row.minus, 1);
    h->addWidget(row.plus, 1);
    h->addSpacing(8);
    h->addWidget(angle);
    h->addWidget(row.go);
    h->addWidget(row.home);
    holdTouch(row.minus, [this, i] { selected = i; ctx.machine.beginJointHold(i, -1); });
    holdTouch(row.plus, [this, i] { selected = i; ctx.machine.beginJointHold(i, 1); });
    const int index = rows.size();
    connect(angle, &QPushButton::clicked, this, [this, index, angle] {
      if (auto value = NumberPad::ask(this, QString("Move J%1 by angle").arg(index + 1), rows[index].degrees, "°")) {
        rows[index].degrees = *value;
        angle->setText(QString("%1%2°").arg(*value >= 0 ? "+" : "").arg(*value));
      }
    });
    connect(row.go, &QPushButton::clicked, this, [this, index] {
      selected = index;
      ctx.machine.moveJoint(index, rows[index].degrees);
    });
    rows << row;
    layout->addWidget(row.row);
  }
  auto *actions = new QHBoxLayout;
  homeJoints = theme::button("HOME JOINTS");
  zero = theme::button("ALL JOINTS TO 0°");
  connect(homeJoints, &QPushButton::clicked, this, [this] { ctx.machine.home(); });
  connect(zero, &QPushButton::clicked, this, [this] { ctx.machine.zeroAll(linearValue, angularValue); });
  actions->addWidget(homeJoints);
  actions->addWidget(zero);
  actions->addStretch();
  layout->addLayout(actions);
  return page;
}

QWidget *JogPage::cartesianPage() {
  auto *page = new QWidget;
  auto *layout = new QVBoxLayout(page);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(6);
  auto *choices = new QHBoxLayout;
  choices->setSpacing(6);
  QList<QPushButton *> frames, modesButtons;
  choices->addWidget(theme::label("Frame", "kicker"));
  choices->addLayout(segments({"Base", "Tool"}, frames, [this](int i) { ctx.machine.endHold("frame_change"); tool = i == 1; refresh(); }));
  baseButton = frames[0];
  toolButton = frames[1];
  choices->addSpacing(10);
  choices->addWidget(theme::label("Mode", "kicker"));
  choices->addLayout(segments({"Hold", "Step"}, modesButtons, [this](int i) { ctx.machine.endHold("mode_switch"); mode = i == 1 ? "step" : "hold"; refresh(); }));
  holdButton = modesButtons[0];
  stepButton = modesButtons[1];
  choices->addStretch();
  // The step sizes: the value and its unit on the key; OLP's label titles the keypad.
  linearLabel = theme::label("", "caption");
  angularLabel = theme::label("", "caption");
  linearLabel->hide();
  angularLabel->hide();
  linearButton = theme::button("");
  angularButton = theme::button("");
  for (auto *b : {linearButton, angularButton}) {
    b->setProperty("compact", true);
    b->setMinimumWidth(96);
    choices->addWidget(b);
  }
  connect(linearButton, &QPushButton::clicked, this, [this] {
    if (!stepMode()) { showSpeedSettings(); return; }
    if (auto v = NumberPad::ask(this, linearLabel->text(), linear(), stepMode() ? "mm" : "mm/s", 0.001, 1000)) {
      (stepMode() ? linearStep : linearValue) = *v;
      QSettings().setValue(stepMode() ? "jog/linearStep" : "jog/linear", *v);
      refresh();
    }
  });
  connect(angularButton, &QPushButton::clicked, this, [this] {
    if (!stepMode()) { showSpeedSettings(); return; }
    if (auto v = NumberPad::ask(this, angularLabel->text(), angular(), stepMode() ? "°" : "°/s", 0.001, 180)) {
      (stepMode() ? angularStep : angularValue) = *v;
      QSettings().setValue(stepMode() ? "jog/angularStep" : "jog/angular", *v);
      refresh();
    }
  });
  layout->addLayout(choices);

  // Two pads, each a 3 x 3 cross. MOVE: X forward at the top and Y to the
  // left as the base frame puts it, Z up and down in the top corners. ROTATE:
  // tilt about X and Y on the cross, turn about Z in the top corners. X, Y and
  // Z carry the axis gizmo's colours.
  auto *pads = new QHBoxLayout;
  pads->setSpacing(16);
  const QList<QColor> axisColor{QColor("#d23f31"), QColor("#2c9a4b"), QColor("#2f6fd6")};
  auto padOf = [&](const QString &title, const QString &binding, QLabel *&centre) {
    auto *column = new QVBoxLayout;
    column->setSpacing(4);
    auto *head = new QHBoxLayout;
    head->addWidget(theme::label(title, "kicker"));
    head->addStretch();
    head->addWidget(new GlyphLabel(binding, "caption"));
    column->addLayout(head);
    auto *grid = new QGridLayout;
    grid->setSpacing(6);
    centre = theme::label("", "kicker");
    centre->setAlignment(Qt::AlignCenter);
    grid->addWidget(centre, 1, 1);
    for (int i = 0; i < 3; ++i) grid->setRowStretch(i, 1);
    column->addLayout(grid, 1);
    pads->addLayout(column, 1);
    return grid;
  };
  auto *move = padOf("MOVE", "{LS}  ·  {RS↕}", moveCentre);
  auto *rotate = padOf("ROTATE", "{R2}+{LS}  ·  {RS↔}", rotateCentre);
  auto key = [&](QGridLayout *pad, int row, int column, int axis, int direction, const QString &binding) {
    const QString name = QStringList{"X", "Y", "Z", "RX", "RY", "RZ"}[axis] + (direction > 0 ? "+" : "−");
    auto *b = theme::button(name, binding, "jog");
    b->setMinimumWidth(100);
    b->setMaximumHeight(150);
    b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    b->setProperty("pad", true);
    // One stick reads large; a modifier plus a stick must still fit the key.
    b->setProperty("glyphHeight", binding.contains('+') ? 24.0 : 30.0);
    b->setProperty("glyphGap", 4.0);
    static_cast<BindingButton *>(b)->setAccent(axisColor[axis % 3]);
    pad->addWidget(b, row, column);
    keys << Key{b, axis, direction};
    connect(b, &QPushButton::pressed, this, [this, axis, direction] {
      if (ctx.machine.holdActive()) { ctx.machine.endHold("button_released"); return; }
      if (stepMode()) ctx.machine.cartesianStep(frame(), axis, direction, linear(), angular());
      else ctx.machine.beginCartesianHold(frame(), axis, direction, linear(), angular());
    });
    connect(b, &QPushButton::released, this, [this] { if (!stepMode()) ctx.machine.endHold("button_released"); });
  };
  key(move, 0, 0, 2, 1, "RS↑");
  key(move, 0, 1, 0, 1, "LS↑");
  key(move, 0, 2, 2, -1, "RS↓");
  key(move, 1, 0, 1, 1, "LS←");
  key(move, 1, 2, 1, -1, "LS→");
  key(move, 2, 1, 0, -1, "LS↓");
  key(rotate, 0, 0, 5, 1, "RS→");
  key(rotate, 0, 1, 4, 1, "R2+LS↑");
  key(rotate, 0, 2, 5, -1, "RS←");
  key(rotate, 1, 0, 3, -1, "R2+LS←");
  key(rotate, 1, 2, 3, 1, "R2+LS→");
  key(rotate, 2, 1, 4, -1, "R2+LS↓");
  layout->addLayout(pads, 1);
  auto *actions = new QHBoxLayout;
  zeroCartesian = theme::button("ALL JOINTS TO 0°");
  connect(zeroCartesian, &QPushButton::clicked, this, [this] { ctx.machine.zeroAll(linearValue, angularValue); });
  actions->addWidget(zeroCartesian);
  actions->addStretch();
  layout->addLayout(actions);
  return page;
}

void JogPage::setCartesian(bool on) {
  if (cartesianMode != on) ctx.machine.endHold("mode_switch");
  cartesianMode = on;
  QSettings().setValue("jog/cartesian", on);
  modes->setCurrentIndex(on ? 1 : 0);
  jointsTab->setChecked(!on);
  cartesianTab->setChecked(on);
  refresh();
}

void JogPage::selectJoint(int delta) {
  if (cartesianMode) return;
  ctx.machine.endHold("ui_navigation");
  selected = std::clamp(selected + delta, 0, std::max(0, ctx.machine.view()["joints"].toArray().size() - 1));
  refresh();
}

void JogPage::stepSpeed(int delta) {
  applySpeedPreset(std::clamp(speedPreset + delta, 0, 2));
}

bool JogPage::stepMode() const {
  const QString effective = mode.isEmpty() ? ctx.machine.view()["cartesianDefaultMode"].toString("hold") : mode;
  return effective == "step";
}

void JogPage::showHeld(int axis, int direction, bool cartesianHold) {
  for (int i = 0; i < rows.size(); ++i) {
    theme::setState(rows[i].minus, "held", !cartesianHold && axis == i && direction < 0);
    theme::setState(rows[i].plus, "held", !cartesianHold && axis == i && direction > 0);
  }
  for (const auto &k : keys) theme::setState(k.button, "held", cartesianHold && k.axis == axis && k.direction == direction);
}

void JogPage::refresh() {
  const auto &view = ctx.machine.view();
  const QString disabled = view["jogDisabled"].toString();
  const bool busy = ctx.machine.moveBusy();
  for (int i = 0; i < speedButtons.size(); ++i) {
    speedButtons[i]->setChecked(i == speedPreset);
    const auto &p = presets[i];
    speedButtons[i]->setToolTip(QString("%1 mm/s · %2 °/s at %3 · joint limits scaled by %3")
      .arg(p.linear * p.fraction).arg(p.angular * p.fraction).arg(percent(p.fraction)));
  }
  speedNote->setText(cartesianMode ? (stepMode()
      ? QString("Step distance and angle per push. Jog speed: %1 of the motion limit.").arg(percent(ctx.machine.fraction()))
      : QString("Speed limits at 100%. Jog speed: %1; joystick travel scales the speed.").arg(percent(ctx.machine.fraction())))
                                   // OLP reads it after its speed select: "10% of each joint's ..."
                                   : view["speedNote"].toString().isEmpty() ? QString()
                                   : percent(ctx.machine.fraction()) + " " + view["speedNote"].toString());
  const auto joints = view["joints"].toArray();
  for (int i = 0; i < rows.size(); ++i) {
    auto &row = rows[i];
    const auto joint = joints.size() > i ? joints[i].toObject() : QJsonObject{};
    row.row->setVisible(joints.isEmpty() || i < joints.size());
    row.name->setText(joint["id"].toString(QString("J%1").arg(i + 1)));
    const auto position = joint["positionRad"];
    const bool rotary = joint["unit"].toString("rad") == "rad";
    row.position->setText(!position.isDouble() ? "—"
                          : rotary ? QString::number(position.toDouble() * degrees, 'f', 2) + "°"
                                   : QString("%1 %2").arg(position.toDouble(), 0, 'f', 4).arg(joint["unit"].toString()));
    row.home->setText(joint["home"].toString());
    const bool isSelected = i == selected;
    theme::setState(row.row, "selected", isSelected);
    theme::setSheet(row.row, isSelected ? QString("QWidget#jointRow { background: %1; border-radius: 3px; }").arg(theme::accentSoft) : "");
    for (auto *b : {row.minus, row.plus}) b->setEnabled(disabled.isEmpty() && !busy);
    row.go->setEnabled(disabled.isEmpty() && !busy && !ctx.machine.holdActive());
  }
  const auto flow = view["flow"].toObject();
  homeJoints->setVisible(flow["nativeHome"].toBool());
  homeJoints->setEnabled(flow["canHome"].toBool());
  // All joints to 0° also recovers a disarmed or stuck robot: its own gate.
  for (auto *b : {zero, zeroCartesian, zeroMain}) b->setEnabled(view["zeroDisabled"].toString().isEmpty() && !busy && !ctx.machine.holdActive());
  const bool step = stepMode();
  baseButton->setChecked(!tool);
  moveCentre->setText(tool ? "Tool" : "Base");
  rotateCentre->setText(tool ? "Tool" : "Base");
  toolButton->setChecked(tool);
  holdButton->setChecked(!step);
  stepButton->setChecked(step);
  QJsonObject labels;
  try {
    labels = ctx.core.call("stepLabels", {step ? "step" : "hold"}).toObject();
  } catch (const OlpError &) {
  }
  linearLabel->setText(labels["linear"].toString(step ? "Distance (mm)" : "Speed at 100% (mm/s)"));
  angularLabel->setText(labels["angular"].toString(step ? "Angle (degrees)" : "Rate at 100% (°/s)"));
  linearButton->setText(QString("%1 %2").arg(linear()).arg(step ? "mm" : "mm/s"));
  angularButton->setText(QString("%1 %2").arg(angular()).arg(step ? "°" : "°/s"));
  linearButton->setToolTip(linearLabel->text());
  angularButton->setToolTip(angularLabel->text());
  for (const auto &k : keys) k.button->setEnabled(disabled.isEmpty() && !(step && busy));
  QString text = ctx.machine.motionReason;
  if (text.isEmpty() || text == "done") text = disabled.isEmpty() ? (text == "done" ? "Move done" : "") : disabled;
  if (text.startsWith("No grant") && flow["arm"].toObject()["state"] == "disarmed")
    text = "Hold L2 to arm · release L2 to disarm";
  reason->setText(displayReason(text));
  theme::setState(reason, "tone", disabled.isEmpty() ? "ok" : "");
  showHeld(ctx.machine.heldAxis(), ctx.machine.heldDirection(), ctx.machine.holdCartesian());
  jointChoice->setVisible(!cartesianMode);
  jointChoice->findChild<QPushButton *>("selectedJogJoint")->setText(QString("J%1 ▾").arg(selected + 1));
  frameChoice->setVisible(cartesianMode);
  modeChoice->setVisible(cartesianMode);
  frameButtons[0]->setChecked(!tool); frameButtons[1]->setChecked(tool);
  modeButtons[0]->setChecked(!step); modeButtons[1]->setChecked(step);
  if (text.isEmpty()) reason->setText("Ready · hold L2 to arm; release to disarm · A records a waypoint");
  programPane->refresh();
}

void JogPage::applySpeedPreset(int index) {
  if (index < 0 || index >= presets.size()) return;
  // Rate changes invalidate a pinned joystick ceiling. Trace keeps its
  // authored path and receives the existing fraction update on its next tick.
  const auto &p = presets[index];
  if (linearValue != p.linear || angularValue != p.angular) ctx.machine.endHold("speed_change");
  speedPreset = index; linearValue = p.linear; angularValue = p.angular;
  QSettings().setValue("jog/preset/selected", index);
  ctx.machine.setFraction(p.fraction);
  refresh();
}

void JogPage::showSpeedSettings() {
  ctx.machine.endHold("ui_navigation");
  InWindowDialog dialog(this);
  dialog.setObjectName("jogSpeedEditor");
  dialog.setProperty("boundedForm", true);
  dialog.setFixedSize(1000, 540);
  auto draft = presets;
  auto *layout = new QVBoxLayout(&dialog);
  layout->setContentsMargins(24, 20, 24, 20);
  layout->addWidget(theme::label("Jog speed presets", "title"));
  auto *note = theme::label("Set base speeds and a percentage. The percentage scales TCP translation, rotation and native joint limits.\nJoystick travel remains proportional; program speeds are unchanged.", "caption");
  note->setWordWrap(true); layout->addWidget(note);
  auto *grid = new QGridLayout;
  const QStringList names{"Slow", "Medium", "Fast"};
  const QStringList labels{"Preset", "Translation · mm/s", "Rotation · °/s", "Percentage", "Effective TCP limits"};
  for (int c = 0; c < labels.size(); ++c) grid->addWidget(theme::label(labels[c], "kicker"), 0, c);
  QList<QLabel *> effective;
  auto update = [&] {
    for (int i = 0; i < 3; ++i) effective[i]->setText(QString("%1 mm/s · %2 °/s").arg(draft[i].linear * draft[i].fraction).arg(draft[i].angular * draft[i].fraction));
  };
  for (int i = 0; i < 3; ++i) {
    grid->addWidget(theme::label(names[i], "title"), i+1, 0);
    for (int c = 0; c < 3; ++c) {
      const double value = c == 0 ? draft[i].linear : c == 1 ? draft[i].angular : draft[i].fraction * 100;
      auto *b = theme::button(QString::number(value));
      b->setObjectName(QString("preset-%1-%2").arg(i).arg(c));
      grid->addWidget(b, i+1, c+1);
      connect(b, &QPushButton::clicked, &dialog, [&, i, c, b] {
        double value = c == 0 ? draft[i].linear : c == 1 ? draft[i].angular : draft[i].fraction * 100;
        if (auto v = NumberPad::ask(&dialog, names[i] + " · " + labels[c+1], value, c == 0 ? "mm/s" : c == 1 ? "°/s" : "%", c == 2 ? 1 : .001, c == 0 ? 1000 : c == 1 ? 180 : 100)) {
          if (c == 0) draft[i].linear = *v; else if (c == 1) draft[i].angular = *v; else draft[i].fraction = *v / 100;
          b->setText(QString::number(*v)); update();
        }
      });
    }
    effective << theme::label("", "value"); grid->addWidget(effective.last(), i+1, 4);
  }
  update(); layout->addLayout(grid); layout->addStretch();
  auto *error = theme::label("", "reason"); layout->addWidget(error);
  auto *actions = new QHBoxLayout;
  auto *cancel = theme::button("Cancel"), *save = theme::button("Save presets", "A", "primary");
  actions->addWidget(cancel); actions->addWidget(save); layout->addLayout(actions);
  connect(cancel, &QPushButton::clicked, &dialog, &QDialog::reject);
  // Both A and touch use the dialog's acceptance gate below.
  connect(save, &QPushButton::clicked, &dialog, &QDialog::accept);
  while (dialog.exec() == QDialog::Accepted) {
    bool ordered = true;
    for (int i = 1; i < 3; ++i)
      ordered &= draft[i-1].fraction <= draft[i].fraction && draft[i-1].linear*draft[i-1].fraction <= draft[i].linear*draft[i].fraction && draft[i-1].angular*draft[i-1].fraction <= draft[i].angular*draft[i].fraction;
    if (!ordered) { error->setText("Keep effective speeds and percentages in Slow ≤ Medium ≤ Fast order."); continue; }
    presets = draft;
    for (int i = 0; i < 3; ++i) {
      const QString key = QString("jog/preset/%1/").arg(i);
      QSettings().setValue(key + "linear", presets[i].linear);
      QSettings().setValue(key + "angular", presets[i].angular);
      QSettings().setValue(key + "fraction", presets[i].fraction);
    }
    applySpeedPreset(speedPreset); break;
  }
}
