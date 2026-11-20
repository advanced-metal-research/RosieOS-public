#pragma once
#include <QFrame>
#include <QLabel>
#include <QVBoxLayout>
#include <QPushButton>
#include <QString>

// OLP's design tokens (offline-programming/v1/ui/src/style.css, its light
// and dark themes), with the type scale raised for the Deck's 7" 1280x800
// panel (~215 ppi): OLP's 12 px body would stand 1.4 mm tall there. Touch
// targets are 44 px. The tokens are read wherever something is painted, so
// setDark() followed by a new styleSheet() re-themes the running app.
namespace theme {
inline const char *bg = "#cfd2d5", *chrome = "#dcdfe2", *content = "#e6e8ea", *hover = "#d5d8db", *input = "#f6f7f8",
                  *line = "#b4b9be", *lineStrong = "#8f979e", *fg = "#22262a", *dim = "#4a5259", *faint = "#8f979e",
                  *value = "#3f5669", *accent = "#1a45c4", *accentSoft = "#d3daf2", *accentText = "#1a45c4",
                  *caution = "#7f4200", *cautionSoft = "#f1dfc6", *ok = "#2a6238", *okSoft = "#d5e6d8",
                  *danger = "#a8231c", *dangerSoft = "#f2d3d0", *brand = "#b8020d",
                  // STOP's fill: OLP's danger in light; a red that white text reads on in dark.
                  *stopFill = "#a8231c", *buttonFill = "#ffffff",
                  // The 3D view: OLP's --viewport-bg and grid.
                  *viewport = "#cfd2d5", *viewportGrid = "#b4b9be",
                  // The axis gizmo's discs.
                  *gizmo = "#263744";
bool dark();
void setDark(bool on);
QString styleSheet();
// A control that names its controller binding under its text ("RECORD" / "A · L4").
QPushButton *button(const QString &text, const QString &binding = {}, const char *role = "");
QLabel *label(const QString &text, const char *role = "");
// A titled card, OLP's panel: the frame to add to a layout and the body to fill.
struct Card { QFrame *frame; QVBoxLayout *body; };
Card card(const QString &title);
// Sets a dynamic style property and re-polishes, so QSS [prop="..."] rules apply.
void setState(QWidget *widget, const char *property, const QVariant &value);
// A widget's own style sheet, only when it differs: QWidget::setStyleSheet
// re-polishes the widget and every child even for the same text, and the
// status poll redraws the jog strip 40 times a second.
void setSheet(QWidget *widget, const QString &sheet);
// A one-line label that never widens its bar: at most its text's width, as
// narrow as the bar needs, the text elided to fit. The Deck's window is the
// screen (gamescope keeps it at 1280 x 800); a header wider than that made Qt
// resize the window past the screen at every text change (2026-10-11,
// a Steam Deck: minimum width 1688 then 1543, the whole UI bounced).
// text() stays the whole text.
QLabel *elidedLabel(const QString &text, const char *role = "");
} // namespace theme
