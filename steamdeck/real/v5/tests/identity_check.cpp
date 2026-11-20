// The cell a pendant was launched for (--expect-cell) against what its OLP
// server reports (cell_identity.hpp), and Machine refusing arm, motion and
// Connect to another cell while STOP still goes out. 2026-10-11: deck-a's
// launchers for cell A and cell B ran side by side and an cell B Arm armed
// cell A (rt-core/docs/history/2026-10-11-pendant-crossarm.md).
// usage: QT_QPA_PLATFORM=offscreen identity-check <olp-core.js>
#include "cell_identity.hpp"
#include "machine.hpp"
#include <QApplication>
#include <cstdio>
#include <cstdlib>

#define CHECK(condition)                                                                 \
  do {                                                                                   \
    if (!(condition)) {                                                                  \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
      std::exit(1);                                                                      \
    }                                                                                    \
  } while (false)

namespace {
const QString cellA = "example-arm-1420", cellB = "other-arm-1420";
QJsonObject targets(const QStringList &cells, const QString &selected = {}) {
  QJsonArray list;
  for (const auto &cell : cells) list << QJsonObject{{"cell_id", cell}};
  return {{"targets", list}, {"selected", selected.isEmpty() ? QJsonValue() : QJsonObject{{"cell_id", selected}}}};
}
} // namespace

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  CHECK(argc == 2);
  using cellidentity::mismatch;

  // No launcher cell: nothing is checked, whatever the server serves.
  CHECK(mismatch({}, targets({cellB}, cellB), cellB, cellB).isEmpty());
  // The launcher's cell served, selected, connected and reported: admitted.
  CHECK(mismatch(cellA, targets({cellA}, cellA), cellA, cellA).isEmpty());
  CHECK(mismatch(cellA, targets({cellA, cellB}), {}, {}).isEmpty());
  // A tunnel to the other cell's OLP: it serves only cell B.
  CHECK(mismatch(cellA, targets({cellB}), {}, {}) ==
        "This pendant was started for " + cellA + ", but its OLP server serves only " + cellB + ". Check the launcher's origin and its tunnel.");
  CHECK(mismatch(cellA, targets({}), {}, {}).contains("its OLP server lists no cells"));
  // A server that serves both but drives the other one.
  CHECK(mismatch(cellA, targets({cellA, cellB}, cellB), {}, {}).endsWith("its OLP server has " + cellB + " selected."));
  CHECK(mismatch(cellA, targets({cellA, cellB}), cellB, {}).endsWith("it is connected to " + cellB + "."));
  CHECK(mismatch(cellA, targets({cellA, cellB}), {}, cellB).endsWith("the robot its OLP server reports is " + cellB + "."));

  OlpCore core(argv[1]);
  // Nothing listens here: a request that goes out stays in flight.
  OlpClient client("http://127.0.0.1:9");
  Machine machine(client, core);
  QStringList said;
  machine.status = [&](const QString &text, bool) { said << text; };
  auto refusesEverything = [&](const QString &words) {
    said.clear();
    machine.arm();
    CHECK(!machine.requestInFlight());
    CHECK(!said.isEmpty() && said.last().contains(words));
    machine.beginJointHold(0, 1);
    CHECK(!machine.holdActive());
    machine.beginCartesianHold("base", 0, 1, 10, 5);
    CHECK(!machine.holdActive());
    machine.moveJoint(0, 5);
    machine.zeroAll(100, 15);
    machine.cartesianStep("base", 0, 1, 10, 5);
    CHECK(!machine.moveBusy());
    machine.home();
    machine.playProgram();
    bool loaded = true, moved = true;
    machine.loadProgram({}, [&](bool ok) { loaded = ok; });
    machine.moveToPositions({0, 0, 0, 0, 0, 0}, [&](bool ok) { moved = ok; });
    CHECK(!loaded && !moved);
    CHECK(!machine.requestInFlight());
  };

  // Without a launcher cell Machine checks nothing.
  CHECK(machine.cellMismatch().isEmpty());
  machine.setExpectedCell(cellA);
  // Until the server's /targets has answered once, nothing arms or moves.
  CHECK(!machine.cellChecked());
  CHECK(machine.cellMismatch().startsWith("Checking that the OLP server drives " + cellA));
  refusesEverything("Checking that the OLP server drives");

  // The OLP behind the tunnel is cell B's.
  machine.observeTargets(targets({cellB}, cellB));
  CHECK(machine.cellChecked());
  CHECK(machine.cellMismatch().contains("serves only " + cellB));
  refusesEverything("serves only " + cellB);
  // Connect to another cell is refused whatever the server says.
  said.clear();
  machine.catalogue = QJsonArray{QJsonObject{{"cell_id", cellB}, {"models", QJsonArray{"rosie_1400_v3"}}}};
  machine.connectCell(cellB);
  CHECK(!machine.requestInFlight());
  CHECK(!said.isEmpty() && said.last() == "This pendant was started for " + cellA + "; connecting " + cellB +
                                               " is refused. Start the pendant's launcher for " + cellB + ".");

  // STOP (and Disarm, the same request) is never refused.
  machine.stop();
  CHECK(machine.requestInFlight());

  // The right server: the cell check passes and Arm goes on to OLP's own
  // flow (here: no cell connected), never the cell's words.
  OlpClient client2("http://127.0.0.1:9");
  Machine right(client2, core);
  QStringList rightSaid;
  right.status = [&](const QString &text, bool) { rightSaid << text; };
  right.setExpectedCell(cellA);
  right.observeTargets(targets({cellA, cellB}, cellA));
  CHECK(right.cellMismatch().isEmpty());
  right.arm();
  CHECK(!rightSaid.isEmpty() && !rightSaid.last().contains("started for") && !rightSaid.last().contains("Checking"));
  std::puts("identity-check: ok");
  return 0;
}
