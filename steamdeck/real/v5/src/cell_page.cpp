#include "pages.hpp"
#include "theme.hpp"
#include "widgets.hpp"
#include <QDialog>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QPainter>
#include <QVBoxLayout>

namespace {
QString shortHash(const QString &sha) {
  const QString bare = QString(sha).remove("sha256:");
  return bare.isEmpty() ? "—" : bare.left(12);
}

// A step's marker in OLP's Machine steps: a filled disc with a tick when
// done, the step number on blue when it is the one to take, a cross on red
// when blocked, an outlined number while pending.
class StepDot : public QWidget {
public:
  StepDot() {
    setFixedSize(30, 30);
    setAttribute(Qt::WA_TranslucentBackground);
    setStyleSheet("background: transparent;");
  }
  void set(int number, const QString &state) {
    if (number == n && state == s) return;
    n = number;
    s = state;
    update();
  }

protected:
  void paintEvent(QPaintEvent *) override {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF r = QRectF(rect()).adjusted(2, 2, -2, -2);
    const QColor colour(s == "done" ? theme::ok : s == "current" ? theme::accent : s == "blocked" ? theme::danger : theme::lineStrong);
    const bool filled = s != "pending";
    p.setPen(QPen(colour, 2));
    p.setBrush(filled ? QBrush(colour) : Qt::NoBrush);
    p.drawEllipse(r);
    QFont f("DejaVu Sans");
    f.setPixelSize(14);
    f.setBold(true);
    p.setFont(f);
    p.setPen(filled ? Qt::white : colour);
    p.drawText(r, Qt::AlignCenter, s == "done" ? "✓" : s == "blocked" ? "✕" : QString::number(n));
  }

private:
  int n = 0;
  QString s;
};
} // namespace

CellPage::CellPage(PageContext &context) : ctx(context) {
  auto *layout = new QHBoxLayout(this);
  layout->setContentsMargins(14, 12, 14, 10);
  layout->setSpacing(14);

  auto *left = new QVBoxLayout;
  left->setSpacing(12);
  auto cellsCard = theme::card("Connected cell");
  cellsCard.body->addWidget(theme::label("Connect to one machine. Connecting verifies its identity; it never arms.", "caption"));
  cells = new QWidget;
  auto *list = new QVBoxLayout(cells);
  list->setContentsMargins(0, 0, 0, 0);
  list->setSpacing(6);
  cellsCard.body->addWidget(cells);
  // Cells other OLP servers advertise on the server's network: listed, never
  // connected. Adding one is the cell host's OLP Manage cells… (desktop).
  auto *networkToggle = theme::button("Network discovery details ▾");
  networkToggle->setCheckable(true); networkToggle->setProperty("compact", true);
  cellsCard.body->addWidget(networkToggle);
  network = new QWidget;
  auto *networkList = new QVBoxLayout(network);
  networkList->setContentsMargins(0, 0, 0, 0);
  networkList->setSpacing(4);
  cellsCard.body->addWidget(network);
  networkStatus = theme::label("", "caption");
  networkStatus->setWordWrap(true);
  cellsCard.body->addWidget(networkStatus);
  network->hide(); networkStatus->hide();
  connect(networkToggle, &QPushButton::toggled, this, [this](bool on) { network->setVisible(on); networkStatus->setVisible(on); });
  auto *actions = new QHBoxLayout;
  disconnect = theme::button("Disconnect");
  refreshButton = theme::button("Refresh");
  home = theme::button("Home axes");
  connect(disconnect, &QPushButton::clicked, this, [this] { ctx.machine.disconnectCell(); });
  connect(refreshButton, &QPushButton::clicked, this, [this] {
    ctx.machine.refreshCatalogue();
    ctx.machine.refreshDiscovery(true);
  });
  connect(home, &QPushButton::clicked, this, [this] { ctx.machine.home(); });
  actions->addWidget(disconnect);
  actions->addWidget(refreshButton);
  actions->addWidget(home);
  actions->addStretch();
  cellsCard.body->addLayout(actions);
  left->addWidget(cellsCard.frame);

  auto machineCard = theme::card("Machine readiness");
  stepRows = new QVBoxLayout;
  stepRows->setSpacing(10);
  machineCard.body->addLayout(stepRows);
  machineCard.body->addStretch();
  left->addWidget(machineCard.frame, 1);
  layout->addLayout(left, 1);

  auto *right = new QVBoxLayout;
  right->setSpacing(12);
  auto settingsCard = theme::card("Robot settings");
  robotSettings = theme::label("", "");
  robotSettings->setWordWrap(true);
  robotSettings->setTextFormat(Qt::RichText);
  robotSettings->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  settingsCard.body->addWidget(robotSettings, 1);
  auto *details = theme::button("Configuration details…");
  settingsCard.body->addWidget(details);
  connect(details, &QPushButton::clicked, this, [this] {
    InWindowDialog dialog(this); auto *layout = new QVBoxLayout(&dialog);
    dialog.setObjectName("configurationDetails");
    auto *text = theme::label(robotSettings->property("details").toString(), "");
    text->setWordWrap(true); text->setMinimumWidth(700);
    layout->addWidget(theme::label("Configuration details", "title")); layout->addWidget(text);
    auto *done = theme::button("Done"); layout->addWidget(done);
    connect(done, &QPushButton::clicked, &dialog, &QDialog::accept); dialog.exec();
  });
  auto *adoptRow = new QHBoxLayout;
  pull = theme::button("Pull cell settings");
  adopt = theme::button("Accept current settings", {}, "primary");
  connect(pull, &QPushButton::clicked, this, [this] { if (pullSettings) pullSettings(); });
  connect(adopt, &QPushButton::clicked, this, [this] { if (adoptSettings) adoptSettings(); });
  adoptRow->addWidget(pull);
  adoptRow->addWidget(adopt);
  adoptRow->addStretch();
  settingsCard.body->addLayout(adoptRow);
  right->addWidget(settingsCard.frame, 1);
  auto connectionCard = theme::card("Connection");
  server = theme::label("", "value");
  server->setWordWrap(true);
  connectionCard.body->addWidget(server);
  right->addWidget(connectionCard.frame);
  layout->addLayout(right, 1);
}

void CellPage::refresh() {
  auto &machine = ctx.machine;
  const auto flow = machine.view().value("flow").toObject();
  // One button per catalogue cell, its model and backend beneath; the connected one is checked.
  auto *list = static_cast<QVBoxLayout *>(cells->layout());
  while (auto *item = list->takeAt(0)) {
    delete item->widget();
    delete item;
  }
  if (machine.catalogue.isEmpty()) list->addWidget(theme::label("No cells in the OLP server's catalogue (olp-cells.json).", "reason"));
  for (const auto &entry : machine.catalogue) {
    const auto cell = entry.toObject();
    const QString id = cell["cell_id"].toString();
    auto *b = theme::button(cell["label"].toString().toUpper(),
                            QString("%1 · %2").arg(cell["models"].toArray().first().toString(), cell["backend"].toString()));
    b->setStyleSheet("min-height: 54px;");
    b->setCheckable(true);
    b->setChecked(machine.connected && machine.connectedCell == id);
    b->setEnabled(cell["reason"].toString() != "cell_configuration_missing");
    b->setToolTip(cell["reason"].toString());
    connect(b, &QPushButton::clicked, this, [this, id] { ctx.machine.connectCell(id); });
    list->addWidget(b);
  }
  // OLP's discovered-cells reading (olp-core discoveredCells), one line per cell.
  auto *networkList = static_cast<QVBoxLayout *>(network->layout());
  while (auto *item = networkList->takeAt(0)) {
    delete item->widget();
    delete item;
  }
  QJsonObject found;
  try {
    found = ctx.core.call("discoveredCells", QJsonArray{machine.discovered, machine.discoveryError}).toObject();
  } catch (const OlpError &error) {
    found = QJsonObject{{"status", QString(error.what())}};
  }
  networkStatus->setText(found["status"].toString());
  for (const auto &entry : found["lines"].toArray()) {
    const auto line = entry.toObject();
    auto *text = theme::label(QString("<b>%1</b> · %2<br>%3").arg(line["title"].toString().toHtmlEscaped(), line["subtitle"].toString().toHtmlEscaped(),
                                                                   line["badges"].toString().toHtmlEscaped()),
                              "reason");
    text->setTextFormat(Qt::RichText);
    text->setWordWrap(true);
    if (line["flagged"].toBool()) text->setProperty("tone", "danger");
    networkList->addWidget(text);
  }
  disconnect->setEnabled(machine.connected);
  home->setVisible(flow["nativeHome"].toBool());
  home->setEnabled(flow["canHome"].toBool());

  // OLP's Machine steps as a stepper: marker, title and the step's own words.
  const auto stepList = flow["steps"].toArray();
  while (steps.size() < stepList.size()) {
    StepRow row;
    auto *h = new QHBoxLayout;
    h->setSpacing(10);
    row.dot = new StepDot;
    h->addWidget(row.dot, 0, Qt::AlignTop);
    auto *text = new QVBoxLayout;
    text->setSpacing(1);
    row.title = theme::label("", "title");
    row.summary = theme::label("", "caption");
    row.summary->setWordWrap(true);
    text->addWidget(row.title);
    text->addWidget(row.summary);
    h->addLayout(text, 1);
    stepRows->addLayout(h);
    steps << row;
  }
  for (int i = 0; i < steps.size(); ++i) {
    const auto s = i < stepList.size() ? stepList[i].toObject() : QJsonObject{};
    const QString state = s["state"].toString("pending");
    static_cast<StepDot *>(steps[i].dot)->set(i + 1, state);
    const QString headline = s["headline"].toString();
    steps[i].title->setText(s["title"].toString() + (headline.isEmpty() ? QString() : "   " + headline));
    theme::setSheet(steps[i].title, QString("color: %1;").arg(state == "blocked" ? theme::danger : state == "current" ? theme::accentText
                                                           : state == "done" ? theme::ok : theme::dim));
    const QString summary = i == 3 ? "Hold L2 to arm; release L2 to disarm. Centre the sticks before jogging." : s["summary"].toString();
    steps[i].summary->setText(displayReason(summary));
    steps[i].summary->setToolTip(summary + " [" + s["code"].toString() + "]");
    for (QWidget *w : {steps[i].dot, static_cast<QWidget *>(steps[i].title), static_cast<QWidget *>(steps[i].summary)})
      w->setVisible(i < stepList.size());
  }

  QJsonObject settings;
  try {
    settings = ctx.core.call("robotSettings").toObject();
  } catch (const OlpError &error) {
    robotSettings->setText(QString(error.what()).toHtmlEscaped());
    return;
  }
  const auto cell = settings["cell"].toObject();
  const auto drift = settings["drift"].toObject();
  const auto review = settings["review"].toObject();
  const QString row = "<tr><td style='color:%1; padding-right:14px'>%2</td><td style='font-family:\"DejaVu Sans Mono\"'>%3</td></tr>";
  QString text = "<table cellspacing=0 cellpadding=2>" + row.arg(theme::dim, "MODEL", settings["model"].toString()) +
                 row.arg(theme::dim, "PROGRAM DESCRIPTION", shortHash(settings["pinned"].toString()));
  if (!cell.isEmpty())
    text += row.arg(theme::dim, "FROM", cell["cell"].toString() + " · " + cell["runtimeMode"].toString().toUpper()) +
            row.arg(theme::dim, "DESCRIPTION", shortHash(cell["robotDescriptionSha256"].toString())) +
            row.arg(theme::dim, "CALIBRATION", shortHash(cell["machinePlanningCalibrationSha256"].toString())) +
            row.arg(theme::dim, "PULLED", cell["pulledAt"].toString().left(19).replace('T', ' '));
  text += "</table>";
  if (cell.isEmpty()) text += QString("<p style='color:%1'>No cell settings pulled into this program yet.</p>").arg(theme::dim);
  if (!drift.isEmpty())
    text += QString("<p style='color:%1'><b>%2</b><br>Robot settings have changed: %3 → %4. %5 Accepting keeps your program; plan again and review the simulation before running.</p>")
                .arg(theme::caution, review["title"].toString("Robot settings changed"), shortHash(drift["from"].toString()),
                     shortHash(drift["to"].toString()), review["explanation"].toString().toHtmlEscaped());
  else if (!settings["pinned"].toString().isEmpty())
    text += QString("<p style='color:%1'><b>✓ Robot settings up to date.</b></p>").arg(theme::ok);
  robotSettings->setProperty("details", text);
  const QString summary = cell.isEmpty() ? "No cell settings pulled into this program yet." : drift.isEmpty()
      ? "✓ Robot settings up to date."
      : "Robot settings changed. Review configuration details before accepting.";
  robotSettings->setText("<b>" + settings["model"].toString().toHtmlEscaped() + "</b><br><br>" + summary +
      (drift.isEmpty() ? "" : "<br><br>" + review["explanation"].toString().toHtmlEscaped()));
  pull->setEnabled(machine.connected);
  adopt->setEnabled(!drift.isEmpty());
  const auto target = machine.view().value("target").toObject();
  server->setText(machine.connected
                      ? QString("%1 · %2 · %3 · RTT %4 ms").arg(target["label"].toString(), target["backend"].toString(), target["host"].toString())
                            .arg(machine.view().value("roundTripMs").isDouble() ? QString::number(machine.view().value("roundTripMs").toDouble(), 'f', 1) : "—")
                      : "Not connected. Control and arming always require explicit actions.");
}
