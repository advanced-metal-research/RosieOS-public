#include "theme.hpp"
#include "glyphs.hpp"
#include <QPainter>
#include <QStyle>
#include <QVariant>

namespace theme {
namespace {
bool darkTheme = false;
}

bool dark() { return darkTheme; }

void setDark(bool on) {
  darkTheme = on;
  // Dark tokens sampled from the owner's approved October 15 pendant image.
  // Neutral charcoal surfaces; blue is reserved for selection and primary actions.
  bg = on ? "#1a1c1e" : "#eceef0";
  chrome = on ? "#202225" : "#f5f6f7";
  content = on ? "#212427" : "#ffffff";
  hover = on ? "#2c2f33" : "#d5d8db";
  input = on ? "#1a1c1e" : "#f6f7f8";
  line = on ? "#35383d" : "#d3d8de";
  lineStrong = on ? "#555c63" : "#8f979e";
  fg = on ? "#e4e7ea" : "#22262a";
  dim = on ? "#adb4bb" : "#4a5259";
  faint = on ? "#7d868e" : "#8f979e";
  value = on ? "#a9c3da" : "#3f5669";
  accent = on ? "#4058d0" : "#1a45c4";
  accentSoft = on ? "#31374c" : "#d3daf2";
  accentText = on ? "#9fadf1" : "#1a45c4";
  caution = on ? "#e0a458" : "#7f4200";
  cautionSoft = on ? "#3a2e1f" : "#f1dfc6";
  ok = on ? "#74c088" : "#2a6238";
  okSoft = on ? "#1e3326" : "#d5e6d8";
  danger = on ? "#f07a70" : "#a8231c";
  dangerSoft = on ? "#43231f" : "#f2d3d0";
  stopFill = on ? "#9c2f21" : "#a8231c";
  buttonFill = on ? "#27292e" : "#ffffff";
  viewport = "#080b10";
  viewportGrid = "#1b2733";
  gizmo = on ? "#11151a" : "#263744";
}

QString styleSheet() {
  return QString(R"(
QWidget { background: %1; color: %2; font-family: "DejaVu Sans"; font-size: 15px; }
QLabel { background: transparent; }
QLabel[role="kicker"] { color: %3; font-family: "DejaVu Sans Mono"; font-size: 11px; font-weight: 600; letter-spacing: 0px; }
QLabel[role="section"] { color: %3; font-size: 13px; font-weight: 600; letter-spacing: 0px; }
QLabel[role="title"] { font-size: 17px; font-weight: 600; }
QLabel[role="caption"] { color: %3; font-size: 12px; }
QLabel[role="value"] { color: %4; font-family: "DejaVu Sans Mono"; font-size: 15px; }
QLabel[role="value"][tone="danger"] { color: %7; font-weight: 700; }
QLabel[role="big"] { color: %4; font-family: "DejaVu Sans Mono"; font-size: 24px; }
QLabel[role="reason"] { color: %5; font-size: 13px; }
QLabel[role="reason"][tone="ok"] { color: %6; }
QLabel[role="reason"][tone="danger"] { color: %7; }
QLabel[role="toolbarLabel"] { color: %4; font-size: 12px; }
QLabel[role="programKicker"] { color: %3; font-size: 12px; font-weight: 500; }
QLabel[role="programTitle"] { font-size: 20px; font-weight: 700; }
QLabel[role="programMeta"] { color: %4; font-size: 12px; }
QLabel[role="insertion"] { color: %4; font-size: 13px; padding-left: 10px; }
QLabel[role="connection"] { color: %3; font-size: 14px; }
QLabel[role="connection"][tone="ok"] { color: %6; }
QLabel[role="connection"][tone="armed"] { color: %5; }
QLabel[role="footerReason"] { color: %5; font-size: 14px; }
QWidget#workspaceHeader { background: %8; border: 0; border-radius: 8px; }
QTableWidget#jogProgramTable { font-size: 15px; }
QTableWidget#jogProgramTable QHeaderView::section { font-size: 13px; font-weight: 500; }
QPushButton#jogSave { font-size: 14px; }
QWidget#chrome { background: %8; border-bottom: 1px solid %9; }
QWidget#status { background: %8; border-top: 1px solid %9; }
QWidget#panel { background: %10; border: 1px solid %9; border-radius: 8px; }
QFrame#card { background: %10; border: 1px solid %9; border-radius: 6px; }
QFrame#card QLabel { background: transparent; }
QPushButton { background: %21; border: 1px solid %9; border-radius: 8px; min-height: 42px; padding: 0 12px; font-size: 14px; }
QPushButton:hover { background: %11; }
QPushButton:pressed, QPushButton:checked, QPushButton[held="true"] { background: %12; border-color: %13; color: %20; }
QPushButton:disabled { color: %14; background: %8; border-color: %9; }
QPushButton[role="primary"] { background: %13; border-color: %13; color: white; font-weight: 600; }
QPushButton[role="primary"]:disabled { background: %8; color: %14; border-color: %9; }
QPushButton[role="stop"] { background: %19; border-color: %19; color: white; font-weight: 700; font-size: 16px; min-height: 54px; }
QPushButton[role="tab"] { border: 0; border-bottom: 3px solid transparent; border-radius: 0; background: transparent; font-weight: 600; font-size: 14px; padding: 0 18px; color: %3; }
QPushButton[role="tab"]:checked { color: %20; border-bottom-color: %13; background: transparent; }
QPushButton[role="pill"] { min-height: 36px; font-family: "DejaVu Sans Mono"; font-size: 12px; font-weight: 700; }
QPushButton[role="pill"][tone="armed"] { background: %15; color: %5; border-color: %5; }
QPushButton[role="pill"][tone="ok"] { background: %16; color: %6; border-color: %6; }
QPushButton[role="jog"] { min-height: 48px; font-size: 15px; font-weight: 600; }
QPushButton[compact="true"] { min-height: 32px; padding: 0 8px; }
QPushButton[teaching="true"] { min-height: 46px; max-height: 46px; padding: 0 6px; }
QPushButton[jogAction="true"] { min-height: 38px; max-height: 38px; }
QPushButton[segment="first"] { border-top-right-radius: 0; border-bottom-right-radius: 0; }
QPushButton[segment="middle"] { border-radius: 0; border-left: 0; }
QPushButton[segment="last"] { border-top-left-radius: 0; border-bottom-left-radius: 0; border-left: 0; }
QPushButton[segment="middle"]:checked, QPushButton[segment="last"]:checked { border-left: 1px solid %13; }
QPushButton[pad="true"] { min-height: 66px; font-size: 24px; font-weight: 700; padding: 0 4px; }
QLabel[role="badge"] { font-family: "DejaVu Sans Mono"; font-size: 12px; font-weight: 700; padding: 3px 8px; border: 1px solid %9; border-radius: 8px; color: %3; }
QLabel[role="badge"][tone="ok"] { color: %6; border-color: %6; background: %16; }
QLabel[role="badge"][tone="caution"] { color: %5; border-color: %5; background: %15; }
QSlider::groove:horizontal { height: 8px; background: %8; border: 1px solid %9; border-radius: 4px; }
QSlider::handle:horizontal { width: 28px; margin: -12px 0; background: %13; border-radius: 4px; }
QLineEdit, QComboBox, QDoubleSpinBox, QSpinBox { background: %17; border: 1px solid %9; border-radius: 8px; min-height: 38px; padding: 0 8px; font-size: 15px; }
QComboBox::drop-down { width: 28px; border: 0; }
QComboBox QAbstractItemView { background: %17; selection-background-color: %12; selection-color: %13; font-size: 15px; }
QCheckBox::indicator { width: 26px; height: 26px; }
QTableWidget { background: %10; border: 0; gridline-color: %9; selection-background-color: %12; selection-color: %2; font-size: 14px; }
QTableWidget::item { padding: 0 6px; }
QHeaderView::section { background: %8; color: %3; border: 0; border-bottom: 1px solid %18; padding: 6px; font-size: 12px; font-weight: 600; }
QScrollBar { width: 0; height: 0; }
QScrollArea QScrollBar:vertical { width: 8px; background: transparent; }
QScrollArea QScrollBar::handle:vertical { background: %14; border-radius: 4px; min-height: 30px; }
QScrollArea QScrollBar::add-line:vertical, QScrollArea QScrollBar::sub-line:vertical { height: 0; }
QScrollArea QScrollBar::add-page:vertical, QScrollArea QScrollBar::sub-page:vertical { background: transparent; }
QMenu { background: %8; border: 1px solid %9; border-radius: 8px; padding: 6px; }
QMenu::separator { height: 1px; background: %9; margin: 5px 10px; }
QLabel[role="footerReason"][tone="neutral"] { color: %3; }
QMenu::item { padding: 12px 24px; min-width: 160px; }
QMenu::item:selected { background: %12; color: %20; }
QMenu::item:disabled { color: %14; }
QToolTip { background: %17; color: %2; border: 1px solid %9; padding: 6px; }
QDialog { background: %8; }
)")
      .arg(bg, fg, dim, value, caution, ok, danger, chrome, line)
      .arg(content, hover, accentSoft, accent, faint, cautionSoft, okSoft, input, lineStrong)
      .arg(stopFill, accentText, buttonFill);
}

QPushButton *button(const QString &text, const QString &binding, const char *role) {
  auto *b = new BindingButton(binding.isEmpty() ? text : text + "\n" + binding);
  b->setProperty("role", role);
  b->setFocusPolicy(Qt::NoFocus);
  return b;
}

Card card(const QString &title) {
  auto *frame = new QFrame;
  frame->setObjectName("card");
  auto *body = new QVBoxLayout(frame);
  body->setContentsMargins(14, 10, 14, 12);
  body->setSpacing(8);
  if (!title.isEmpty()) body->addWidget(label(title, "section"));
  return {frame, body};
}

QLabel *label(const QString &text, const char *role) {
  auto *l = new QLabel(text);
  l->setProperty("role", role);
  return l;
}

void setSheet(QWidget *widget, const QString &sheet) {
  if (widget->styleSheet() != sheet) widget->setStyleSheet(sheet);
}

namespace {
class ElidedLabel : public QLabel {
public:
  using QLabel::QLabel;
  QSize minimumSizeHint() const override { return {0, QLabel::minimumSizeHint().height()}; }

protected:
  void paintEvent(QPaintEvent *) override {
    QPainter painter(this);
    const QRect area = contentsRect();
    const QString shown = fontMetrics().elidedText(text(), Qt::ElideRight, area.width());
    style()->drawItemText(&painter, area, int(alignment()), palette(), isEnabled(), shown, foregroundRole());
  }
};
} // namespace

QLabel *elidedLabel(const QString &text, const char *role) {
  auto *l = new ElidedLabel(text);
  l->setProperty("role", role);
  l->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  return l;
}

void setState(QWidget *widget, const char *property, const QVariant &value) {
  if (widget->property(property) == value) return;
  widget->setProperty(property, value);
  widget->style()->unpolish(widget);
  widget->style()->polish(widget);
}
} // namespace theme
