// OLP's program logic, run the way the pendant runs it: the bundled OLP
// sources inside Qt's engine. usage: core-check <olp-core.js> <robots.json> <reference.rdt>
#include "olp_core.hpp"
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <cmath>
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
QJsonObject pose(double x) {
  QJsonArray names, values;
  for (int i = 0; i < 9; ++i) {
    names << QString("J%1").arg(i + 1);
    values << (i < 6 ? .1 * i : 0.);
  }
  return {{"frame_id", "world"}, {"xyz_m", QJsonArray{x, .7, .9}}, {"rpy_rad", QJsonArray{M_PI, .6, 3.1}},
          {"joint_names", names}, {"joint_values_rad", values}};
}
QJsonObject capture(double x, const QString &description) {
  return {{"source", "machine"}, {"observed_at", "2026-09-27T12:00:00Z"}, {"model_id", "rosie_1400_v3"},
          {"robot_description_sha256", description}, {"cell_id", "local-simulation"}, {"pose", pose(x)}};
}
bool refused(OlpCore &core, const QString &function, const QJsonArray &args, const char *words) {
  try {
    core.call(function, args);
  } catch (const OlpError &error) {
    return QString(error.what()).contains(words);
  }
  return false;
}
} // namespace

int main(int argc, char **argv) {
  QCoreApplication app(argc, argv);
  CHECK(argc == 4);
  OlpCore core(argv[1]);
  QFile robots(argv[2]);
  CHECK(robots.open(QIODevice::ReadOnly));
  core.call("setRobots", {QJsonDocument::fromJson(robots.readAll()).object(), QJsonValue::Null});

  core.call("newProject", {"Deck teach", "rosie_1400_v3"});
  auto view = core.call("view").toObject();
  const QString pinned = view["description"].toString();
  CHECK(pinned.startsWith("sha256:"));
  CHECK(view["railsAuthoring"].toBool());

  core.call("record", {"new", pose(1.6), capture(1.6, pinned), "move-1"});
  core.call("record", {"new", pose(1.7), capture(1.7, pinned), "move-2"});
  core.call("record", {"via", pose(1.65), capture(1.65, pinned), "unused"});
  core.call("record", {"reteach", pose(1.75), capture(1.75, pinned), "unused"});
  view = core.call("view").toObject();
  auto rows = view["rows"].toArray();
  CHECK(rows.size() == 2);
  CHECK(rows[0].toObject()["label"] == "P1" && rows[1].toObject()["freespace"] == "MoveC");
  CHECK(std::abs(rows[1].toObject()["cells"].toObject()["target.xyz_m.0"].toDouble() - 1750) < 1e-9);

  core.call("select", {"move-1"});
  const QString dwell = core.call("addEvent", {"dwell"}).toString();
  core.call("addEvent", {"home"});
  core.call("edit", {"move-1", "speed_mm_s", 40});
  core.call("edit", {dwell, "duration_s", 1.5});
  CHECK(refused(core, "edit", {"move-1", "speed_scale", 250}, "at most 100"));
  CHECK(refused(core, "record", {"reteach", pose(1), capture(1, pinned), "x"}, "Select a taught waypoint"));
  rows = core.call("view").toObject()["rows"].toArray();
  QStringList ops;
  for (const auto &row : rows) ops << row.toObject()["op"].toString();
  CHECK(ops.join(",") == "move,dwell,move,home");

  // A field in OLP's dialog groups, its value in display units.
  bool found = false;
  for (const auto &group : core.call("fields", {"move-1"}).toObject()["groups"].toArray())
    for (const auto &field : group.toObject()["fields"].toArray())
      if (field.toObject()["path"] == "speed_mm_s") found = field.toObject()["value"].toDouble() == 40;
  CHECK(found);

  core.call("undo");
  CHECK(core.call("view").toObject()["canRedo"].toBool());
  core.call("redo");

  // The saved file is an OLP project export, and it reopens node for node.
  // value(), not operator[]: a QJsonValueRef into a temporary object dangles.
  const auto nodes = [](const QString &text) {
    return QJsonDocument::fromJson(text.toUtf8()).object().value("project").toObject().value("nodes");
  };
  const QString exported = core.call("exportProject").toString();
  CHECK(QJsonDocument::fromJson(exported.toUtf8()).object().value("schema") == "offline-programming.project-export.v1");
  const QJsonValue before = nodes(exported);
  CHECK(before.toArray().size() == 4);
  core.call("openProject", {exported});
  CHECK(nodes(core.call("exportProject").toString()) == before);

  // The plan request the browser would send: packed asynchronously, its
  // authored digest the SHA-256 of OLP's canonical program JSON -- hashed by
  // the engine's crypto.subtle and checked here against Qt's own SHA-256.
  auto run = [&](const QJsonObject &input) {
    QJsonObject full{{"connected", false}, {"cellPending", false}, {"inFlight", false}, {"statusAgeMs", QJsonValue()},
                     {"lastRequestError", QJsonValue()}, {"planStartActive", false}, {"planning", false},
                     {"cellModel", "rosie_1400_v3"}, {"bytesKept", false}};
    for (auto it = input.begin(); it != input.end(); ++it) full[it.key()] = it.value();
    return core.call("runView", {full}).toObject();
  };
  CHECK(run({})["plan"] == "NO PLAN");
  CHECK(run({})["planBlocked"].toString().isEmpty());
  // RUN PROGRAM is the one press (runProgram.ts); offline it says what it needs.
  CHECK(run({})["runProgram"].toObject()["text"] == "Run program");
  CHECK(run({})["runProgram"].toObject()["disabled"].toBool());
  CHECK(run({})["runProgram"].toObject()["title"] == "Connect a machine first");
  CHECK(run({{"connected", true}, {"running", true}})["runProgram"].toObject()["text"] == "Running…");
  // Its answers in OLP's words: a question carries the dialog and the raw
  // result the confirmed press sends back with its flags and joints.
  const QJsonObject question{{"state", "confirm"}, {"action", "Move to program start and run"}, {"needs_move", true},
                             {"joints", QJsonArray{QJsonObject{{"axis", 0}, {"id", "J1"}, {"from_deg", 40.0}, {"to_deg", 10.0}, {"delta_deg", -30.0}, {"moves", true}}}}};
  const auto asked = core.call("runAnswer", {question, true}).toObject();
  CHECK(asked["state"] == "confirm");
  CHECK(asked["confirmation"].toObject()["title"] == "Move to program start and run");
  CHECK(asked["confirmation"].toObject()["confirm"] == "Move and run");
  CHECK(asked["confirmation"].toObject()["lines"].toArray().at(0) == "J1  40.00° → 10.00°  Δ −30.00°");
  CHECK(asked["cancelled"] == "Run program cancelled: nothing was armed, moved or played");
  const auto confirmed = core.call("runBody", {QJsonObject{{"trajectory_digest", "sha256:x"}}, 0.25, asked["result"]}).toObject();
  CHECK(confirmed["confirm_move"] == true && confirmed["confirm_arm"] == false && confirmed["fraction"] == 0.25);
  CHECK(confirmed["trajectory_digest"] == "sha256:x");
  CHECK(confirmed["confirmed"].toArray().at(0).toObject()["to_deg"] == 10.0);
  // A refusal is the server's sentence; its code lets the pendant resend
  // with the kept bytes when the planner lost them.
  const QJsonObject lost{{"error", "dense_blob_not_found"}, {"detail", "Load refused: no trajectory"},
                         {"run", QJsonObject{{"state", "refused"}, {"refusal", QJsonObject{{"step", "load"}, {"code", "dense_blob_not_found"}, {"detail", "Load refused: no trajectory"}}}}}};
  const auto refusedRun = core.call("runAnswer", {lost, false, "dense_blob_not_found", "Load refused: no trajectory"}).toObject();
  CHECK(refusedRun["state"] == "refused" && refusedRun["code"] == "dense_blob_not_found");
  CHECK(refusedRun["text"] == "Load refused: no trajectory");
  // Run N times / Loop: the repeat rides in RUN's body, and the one loop
  // question shows the planned return move.
  core.call("setRepeat", {"count", 3});
  CHECK(run({})["repeat"].toObject()["mode"] == "count" && run({})["repeat"].toObject()["count"] == 3);
  CHECK(run({})["runProgram"].toObject()["text"] == "Run 3×");
  const auto repeated = core.call("runBody", {QJsonObject{{"trajectory_digest", "sha256:x"}}, 0.25}).toObject();
  CHECK(repeated["repeat"] == "count" && repeated["count"] == 3);
  const QJsonObject loopQuestion{{"state", "confirm"}, {"action", "Run 3 times"}, {"joints", QJsonArray{}},
                                 {"loop", QJsonObject{{"mode", "count"}, {"count", 3}, {"return_move", QJsonObject{{"duration_s", 2.0},
                                  {"joints", QJsonArray{QJsonObject{{"axis", 0}, {"id", "J1"}, {"from_deg", 30.0}, {"to_deg", 0.0}, {"delta_deg", -30.0}, {"moves", true}}}}}}}}};
  const auto loopAsked = core.call("runAnswer", {loopQuestion, true}).toObject();
  CHECK(loopAsked["confirmation"].toObject()["eyebrow"] == "RUN 3 TIMES");
  CHECK(loopAsked["confirmation"].toObject()["lines"].toArray().at(0) == "J1  30.00° → 0.00°  Δ −30.00°");
  CHECK(core.call("runBody", {QJsonObject{}, 0.25, loopAsked["result"]}).toObject()["confirm_loop"] == true);
  core.call("setRepeat", {"once"});
  CHECK(run({})["runProgram"].toObject()["text"] == "Run program");
  // PLAN waits for this pendant's own jog or move, in one instruction; a
  // program for another robot names both models instead.
  CHECK(run({{"holding", true}})["planBlocked"] == "Release the sticks and wait for the robot to stop, then PLAN");
  CHECK(run({{"moving", true}})["planBlocked"] == "Release the sticks and wait for the robot to stop, then PLAN");
  const auto otherCell = run({{"connected", true}, {"cellModel", "rosie_1420_v1"}});
  CHECK(otherCell["planBlocked"].toString().startsWith("This program is for rosie_1400_v3; the connected cell is rosie_1420_v1."));
  CHECK(otherCell["modelMismatch"] == otherCell["planBlocked"]);
  // A jog refusal is the instruction; the server's code and words stay verbatim beside it.
  const auto moving = core.call("planFailure", {"planning_start_moving", "Release the jog and wait for the robot to stop before planning", QJsonValue()}).toObject();
  CHECK(moving["message"] == "Release the sticks and wait for the robot to stop, then PLAN");
  CHECK(moving["verbatim"] == "planning_start_moving: Release the jog and wait for the robot to stop before planning");
  const auto other = core.call("planFailure", {"motion_plan_failed", "the motion server is unreachable", QJsonValue()}).toObject();
  CHECK(other["verbatim"] == "motion_plan_failed: the motion server is unreachable");
  // A failed plan from the robot's pose names the alternative; a held jog does not.
  const auto measured = core.call("planFailure", {"planning_start_unverified", "Home is running (axis mask 0x1); plan after it finishes", QJsonValue(), true}).toObject();
  CHECK(measured["message"].toString().endsWith(" — or choose FROM PROGRAM START and PLAN again, then GO TO PLAN START"));
  CHECK(!core.call("planFailure", {"planning_start_moving", "Release the jog and wait for the robot to stop before planning", QJsonValue(), true})
             .toObject()["message"].toString().contains("FROM PROGRAM START"));
  // FROM PROGRAM START is one PLAN's choice, offered while connected.
  core.call("setPlanFromProgramStart", {true});
  CHECK(run({{"connected", true}})["planFromProgramStart"].toBool());
  CHECK(!run({})["planFromProgramStart"].toBool());
  CHECK(core.call("plannedNote", {720, 7.0, true}).toString() == "Planned · 720 samples, 7.0 s · from the program's start: Go to plan start, then Load and Play");
  std::optional<QJsonValue> body;
  QString failure;
  bool settled = false;
  core.callAsync("planBody", {}, [&](std::optional<QJsonValue> value, const QString &error) {
    body = value;
    failure = error;
    settled = true;
  });
  // Promise reactions and callAsync's poll run from the event loop: wait on
  // it for real time, not a count of passes that may each return at once.
  QElapsedTimer waited;
  waited.start();
  while (!settled && waited.elapsed() < 10000) QCoreApplication::processEvents(QEventLoop::WaitForMoreEvents, 50);
  if (!failure.isEmpty()) std::fprintf(stderr, "planBody: %s\n", qPrintable(failure));
  CHECK(settled && body && failure.isEmpty());
  const auto request = body->toObject();
  const QByteArray canonical = core.call("authoredProgramJSON").toString().toUtf8();
  CHECK(request["program_digest"].toString() == "sha256:" + QString::fromLatin1(QCryptographicHash::hash(canonical, QCryptographicHash::Sha256).toHex()));
  CHECK(request["workpiece_absent"].toBool() && request["step_base64"] == "");
  CHECK(request["cell_id"] == "rosie_1400_v3" && request["free_space_backend"] == "bspline");
  CHECK(request["plan_from_program_start"].toBool() && !request["require_measured_start"].toBool());
  CHECK(request["program"].toObject()["program_digest"].toString().startsWith("sha256:"));

  // The planner's answer is kept as the browser keeps it, for this revision.
  const int revision = QJsonDocument::fromJson(core.call("exportProject").toString().toUtf8())
                           .object().value("project").toObject().value("identity").toObject().value("revision").toInt();
  const QString planProjectId = core.call("projectIdentity").toString();
  const QJsonObject dense{{"schema", "dense"}, {"trajectory_digest", "sha256:" + QString(64, 'c')}, {"request_sha256", "r"},
                          {"plan_id", "plan-1"}, {"program_id", "p"}, {"program_digest", "d"}, {"manifest_revision", 1},
                          {"plan_revision", 1}, {"created_at", "2026-09-28T00:00:00Z"}, {"dt_s", .001},
                          {"total_sample_count", 3000}, {"total_duration_s", 3.0}, {"bytes", 100},
                          {"segments", QJsonArray{QJsonObject{{"index", 0}, {"kind", "freespace"}, {"source_id", "move-2"},
                                                              {"sample_count", 3000}, {"duration_s", 3.0}}}}};
  CHECK(refused(core, "applyPlan", {QJsonObject{{"dense", dense}}, revision + 1, planProjectId}, "changed while it was planning"));
  CHECK(refused(core, "applyPlan", {QJsonObject{{"dense", dense}}, revision, "another-program"}, "changed while it was planning"));
  CHECK(core.call("applyPlan", {QJsonObject{{"dense", dense}, {"program_digest", request["program_digest"]}}, revision, planProjectId})
            .toString() == dense["trajectory_digest"].toString());
  auto planned = run({});
  CHECK(planned["plan"] == "PLANNED" && planned["planText"].toString().startsWith("3000 samples · 3.0 s"));
  CHECK(!run({{"connected", true}})["planFromProgramStart"].toBool()); // a kept plan clears the choice
  CHECK(planned["controls"].toObject()["stop"].toObject()["disabled"].toBool()); // no machine selected
  core.call("edit", {"move-1", "speed_mm_s", 45});
  CHECK(run({})["plan"] == "NO PLAN"); // reviseProject drops the trajectory on a motion edit

  // OLP's trajectory reader on the motion server's reference blob, through
  // the engine's atob and TextDecoder: every sample, segments end to end.
  QFile blob(argv[3]);
  CHECK(blob.open(QIODevice::ReadOnly));
  const auto path = core.call("previewPath", {QString::fromLatin1(blob.readAll().toBase64()), 0}).toObject();
  const auto frames = path["frames"].toArray();
  CHECK(frames.size() == 11);
  CHECK(frames[0].toObject()["q"].toArray().size() == 9);
  double t = -1;
  for (const auto &frame : frames) {
    CHECK(frame.toObject()["t"].toDouble() >= t);
    t = frame.toObject()["t"].toDouble();
  }
  CHECK(t == path["durationS"].toDouble() && t > 0);
  std::puts("PASS: OLP program logic in QJSEngine: record/via/reteach, events, edits, undo, OLP export round trip, "
            "plan request (SHA-256 = Qt's), plan kept, dropped by a motion edit");
}
