#pragma once
#include "hold_pacer.hpp"
#include "olp_client.hpp"
#include "olp_core.hpp"
#include <QElapsedTimer>
#include <QJsonArray>
#include <QJsonObject>
#include <QList>
#include <QNetworkReply>
#include <QObject>
#include <QPointer>
#include <QSet>
#include <QTimer>
#include <functional>
#include <optional>

// The selected machine, driven the way OLP's browser Dense panel drives it
// through the OLP server: the cell catalogue and Connect, the 250 ms status
// poll, the 1 s heartbeat the server's deadman expects (it stops the machine
// after 5 s of silence), Arm/Disarm/Stop/Home, joint and Cartesian holds at
// OLP's 50 ms cadence, move by angle, All joints to 0°, Cartesian steps,
// Go to plan start and program Load/Play.
// Whether motion is allowed, and every word shown about it, comes from OLP's
// code (OlpCore machineView) and the server; this class transports and times.
class Machine : public QObject {
public:
  Machine(OlpClient &client, OlpCore &core, QObject *parent = nullptr);
  ~Machine() override;

  void refreshCatalogue();
  // Cells other OLP servers advertise on the server's network (read-only;
  // `force` asks the server for a new search).
  void refreshDiscovery(bool force);
  void connectCell(const QString &cell);
  void disconnectCell();

  // Arm asks for the operator's second press when OLP's flow requires it.
  void arm();
  // Explicit Run-page authority, independent of the physical L2 permit.
  // It admits program preparation/play only, never teaching/jog/Trace/Home.
  // Offered only on a cell whose OLP reports the manifest's
  // unattended_loop_allowed (capabilities; owner decision 2026-11-13); the
  // server refuses {"program": true} elsewhere. Every other cell keeps
  // the held-L2 deadman for every motion, as before program arming existed.
  bool programArmOffered() const { return connected && unattendedLoopAllowed; }
  void armForProgram();
  bool programArmPermitted() const { return programPermit; }
  void releaseProgramArm(const QString &cause);
  // Confirmed execution, including pauses and between-cycle loop work.
  // A pending Run confirmation or move-to-start is deliberately not active.
  bool programExecutionActive() const { return observedProgramActive; }
  // Window opts into a continuously held physical arm control. All action
  // paths, including Home, recovery and Run's implicit Arm, honour this permit.
  void requireArmHold() { armHoldRequired = true; armPermit = false; homeHold = false; }
  void permitArmHold() { armPermit = true; homeHold = false; }
  bool armHoldPermitted() const { return armPermit; }
  // The held permit was granted for Home: arm() found the machine flow's Home
  // step current (a described axis without a valid Home, native Home
  // offered, disarmed) and sent no Arm.
  bool homeHoldActive() const { return armPermit && homeHold; }
  // The held control was released while it held the permit. Stop (as
  // disarm()), unless the permit was granted for Home and nothing could be
  // energised: no request in flight, no Home in progress, observed disarmed.
  void releaseArmHold();
  bool armRequestBusy() const { return inFlight || armPending || stopPending != 0 || armStatusPending; }
  unsigned armCancellationRevision() const { return armCancellation; }
  void cancelArm();
  bool armConfirming() const { return armConfirm; }
  void disarm();
  void stop();
  void home(const QList<int> &axes = {});

  void setFraction(double fraction);
  double fraction() const { return speed; }
  void beginJointHold(int axis, int direction, std::optional<double> scale = {});
  void beginCartesianHold(const QString &frame, int axis, int direction, double linearMM, double angularDeg, std::optional<double> scale = {});
  void setHoldScale(double scale);
  void beginTrace(int direction, double linearMM, double angularDeg);
  // A completed teach transaction may skip only the initial edit debounce.
  // This hint is not retained if motion or foreground planning owns the lane.
  void prepareTrace(bool committedTeach = false);
  // Explicit Plan has priority over speculative Trace planning. Retain a
  // completed path; cancel only unfinished preparation, then resume when idle.
  void setForegroundPlanning(bool active);
  std::function<QString(const QString &)> keptTraceBytes;
  // A queued network timer must not renew old input ahead of the UI tick
  // that consumes an already-observed physical release or Stop.
  std::function<QString(bool trace)> pendingInputRelease;
  bool traceActive() const { return hold.active && hold.trace; }
  // Release wins over any reply still in flight; a late reply cannot restart it.
  // Ends the hold; cause is the End's end_cause (trigger_released,
  // stick_direction_change, focus_lost, ...), empty for a plain release.
  void endHold(const QString &cause = {});
  bool holdActive() const { return hold.active; }
  bool holdEnding() const { return holdEndPending != 0; }
  unsigned holdEndFailureRevision() const { return holdEndFailure; }
  int heldAxis() const { return hold.active ? hold.axis : -1; }
  int heldDirection() const { return hold.active ? hold.direction : 0; }
  bool holdCartesian() const { return hold.active && hold.cartesian; }
  void moveJoint(int axis, double degrees);
  // All joints to 0°, the one-press recovery. Before the server arms it,
  // confirmRecover shows olp-core's confirmation (title, intro, lines,
  // warning, confirm, cancel) and answers whether the operator confirmed.
  void zeroAll(double linearSpeedMMs, double angularSpeedDegS);
  std::function<bool(const QJsonObject &confirmation)> confirmRecover;
  // Go to plan start: every axis to the plan's first position (radians).
  void moveToPositions(const QJsonArray &positions, std::function<void(bool moved)> done = {});
  // Program execution through the OLP server: stage the kept plan, then play it.
  void loadProgram(const QJsonObject &identity, std::function<void(bool loaded)> done = {});
  void playProgram();
  // Run program / Run again: one press OLP sequences (Go to plan start when
  // the arm is elsewhere, Load unless staged, Arm when disarmed, Play). A
  // confirm answer shows olp-core's confirmation through confirmRecover, then
  // presses again; STOP ends the press, the question included. `base` is the
  // Load body (olp-core loadBody, maybe with the kept bytes); `done` gets
  // olp-core's runAnswer for the last reply.
  void runProgram(const QJsonObject &base, std::function<void(const QJsonObject &answer)> done = {});
  bool runningProgram() const { return running; }
  // Retire only the accepted Run's progress text, never a later refusal or
  // fault. Document switches also end that text's relevance, not execution.
  void clearRunProgress();
  std::function<void(const QString &message)> runProgressEnded;
  // Finish cycle: the running loop ends at the program start after this
  // cycle (the server admits only the client that started it).
  void finishLoop();
  // A Run program or Finish cycle press that started nothing: olp-core's
  // refusal (kicker, text, details, code) for the toast; an empty object
  // clears it when a later press goes through. Never only the line under
  // the button (2026-10-12: "Run 5 loops did nothing" on a Steam Deck).
  std::function<void(const QJsonObject &refusal)> pressNotice;
  bool requestInFlight() const { return inFlight; }
  // Poll at OLP's fast rate while the live robot is on screen; `pose` gets
  // every measured pose ([{id, position}]) as it arrives.
  void setFastPoll(bool on);
  std::function<void(const QJsonArray &)> pose;
  QString lastError() const { return lastRequestError; }
  void cartesianStep(const QString &frame, int axis, int direction, double distanceMM, double angleDeg);
  bool moveBusy() const { return moving; }
  // OLP's teach busy, as far as the pendant holds it: a request in flight, a
  // held jog or a move. The bundle adds a playing program from the status.
  bool teachBusy() const { return inFlight || hold.active || moving || holdEndPending; }
  // Milliseconds since the server last answered the status poll; nullopt before it has.
  std::optional<qint64> statusAgeMs() const { return contacted ? std::optional<qint64>(lastContact.elapsed()) : std::nullopt; }

  // An update in progress (the OLP server's maintenance notice; empty clears).
  // While set, everything that could arm, start motion, Home, load, play or
  // take a cell is refused here as the server refuses it. Stop, Disarm,
  // ending a hold and Disconnect still run: a notice never stops or disarms
  // by itself. Setting it ends a held jog, as a dialog opening over the
  // controls does.
  void setMaintenance(const QString &reason);
  bool maintenance() const { return !maintenanceReason.isEmpty(); }

  // The cell this pendant's launcher named (--expect-cell; empty checks
  // nothing). Until the server's /targets has been read once, and whenever
  // it, this pendant's Connect or the status names another cell
  // (cell_identity.hpp), everything the maintenance notice refuses is
  // refused here too and a held jog ends; Connect is refused for any other
  // cell. Stop, Disarm, ending a hold and Disconnect still run.
  void setExpectedCell(const QString &cell);
  const QString &expected() const { return expectedCell; }
  void observeTargets(const QJsonObject &body);
  QString cellMismatch() const;
  // The server's /targets has been read at least once.
  bool cellChecked() const { return targetsSeen; }

  // OLP's machineView. Read-only outside: a [] read on a non-const
  // QJsonObject inserts a null key, which once made an empty selection look
  // like a chosen cell.
  const QJsonObject &view() const { return lastView; }
  void configureIOClient(OlpClient& transport) const { const auto t=target(); transport.setTarget(t["cell_id"].toString(),t["selection_generation"].toString()); }
  QString ioSettingsCell;
  QJsonObject ioSettings;
  QJsonObject ioState, ioDefinition; // populated by the existing status poll, never a second poller
  QJsonArray catalogue;        // /targets
  QJsonValue discovered;       // /discovered-cells, null until answered
  QString discoveryError;
  QString connectedCell, motionReason;
  bool connected = false;
  std::function<void()> changed;
  std::function<void(const QString &, bool error)> status;
  // The server published a new last jog end (olp-core jogEndObserve said
  // the "Jog stopped" overlay must redraw).
  std::function<void()> jogEnded;
  // Every status poll: the live path-hold line's view (olp-core
  // jogLiveFromStatus).
  std::function<void(const QJsonObject &)> jogLive;
  // Every status poll: the cell's outputs view (olp-core
  // cellOutputsFromStatus); an empty object when the server is silent or
  // the machine disconnects, so no output state outlives its observation.
  std::function<void(const QJsonObject &)> outputs;

private:
  struct Hold {
    bool active = false, cartesian = false;
    int axis = -1, direction = 0;
    unsigned generation = 0;
    QJsonObject body;
    QString route;
    HoldPacer pacer;
    // In-flight intents, aborted on release so End never queues behind them.
    QList<QPointer<QNetworkReply>> intents;
    bool trace = false;
    QString traceTarget;
    QPointer<QNetworkReply> planning;
    bool proportional = false;
    double inputScale = 1;
    quint64 inputSequence = 0;
  };
  OlpClient &client;
  OlpCore &core;
  QTimer pollTimer, heartbeatTimer, holdTimer, catalogueRetry;
  QElapsedTimer lastContact, fullCycle;
  bool contacted = false, inFlight = false, armConfirm = false, moving = false, polling = false, fast = false;
  QString connectError, catalogueError, pendingRequest, lastRequest, lastRequestError;
  QJsonObject posture, lastView;
  double speed = .1;
  // The cell's OLP reports fraction_updates: a Cartesian (path) hold takes a
  // new speed within the hold instead of ending (owner decision 2026-10-06).
  bool fractionUpdates = false;
  Hold hold;
  unsigned holdEndPending = 0, holdEndFailure = 0;
  QSet<QString> gateWords; // every jogDisabled text since the gate last stood open
  unsigned epoch = 0;
  unsigned actionGeneration = 0;
  bool armHoldRequired = false, armPermit = false, armPending = false, homeHold = false;
  // /home requests not yet answered, a stale (Stopped) one included.
  unsigned homePending = 0;
  // The machine flow's Home step is current (olp-core deriveMachineFlow).
  bool homeStepCurrent() const;
  // A /home request unanswered, or the controller reporting Home active on
  // any axis (olp-core joints[].homeState "homing").
  bool homeInProgress() const;
  bool programPermit = false, observedProgramActive = false, unattendedLoopAllowed = false;
  QString runProgressMessage;
  unsigned stopPending = 0, armCancellation = 0;
  unsigned observationRevision = 0;
  bool armStatusPending = false;
  QString maintenanceReason;
  bool refusedForMaintenance();
  QString expectedCell;
  QJsonObject servedTargets;
  bool targetsSeen = false;
  bool refusedForCell();
  void enforceCell();
  // A robot action: refused during maintenance, for the wrong cell, while
  // a program arm holds the permit (unless `programAction`), or without the
  // held permit (saying `withoutHold`).
  bool refused(bool programAction = false, const QString &withoutHold = "Hold L2 to arm; release L2 to disarm");
  void armWithPermit(bool programAction);
  void observeProgramStatus(const QJsonObject &body);

  void poll();
  void heartbeat();
  void tick();
  void refresh();
  void run(const QString &request, const QString &path, const QJsonObject &body, const QString &accepted,
           int timeoutMs = 4000, std::function<void(const OlpClient::Reply &)> done = {});
  void moveAxes(const QJsonArray &moves, QString refusal, std::function<void(bool)> done = {});
  // One recovery request; a confirm_arm answer asks, then presses again.
  // `recovering` spans the whole press, the question included; STOP clears it.
  void recoverPress(const QJsonObject &body);
  bool recovering = false;
  // One Run program request; `running` spans the press, the question included.
  void runPress(const QJsonObject &body);
  // STOP or DISARM ended the press: tell the page why.
  void endRun(const QString &why);
  bool running = false;
  QJsonObject runBase;
  std::function<void(const QJsonObject &)> runDone;
  void startTracePlan(unsigned generation, const QJsonObject &load, bool reverse);
  bool traceCompleted(const QJsonObject &body);
  QString tracePreparationKey, tracePreparedKey, tracePreparingKey, tracePreparationError;
  QJsonObject tracePreparationLoad;
  bool tracePreparationRetryable = false;
  bool tracePreparationDeferred = false;
  bool foregroundPlanning = false;
  unsigned tracePreparationGeneration = 0;
  unsigned tracePreparationEpoch = 0;
  QElapsedTimer traceDebounce;
  QPointer<QNetworkReply> tracePreparation;
  void prepareTraceLoad(const QString &key, const QJsonObject &result);
  bool tracePreparationIdle() const;
  void sendTracePreparation(const QString &key);
  void tracePreparationFailed(const OlpClient::Reply &reply);
  void beginHold(bool cartesian, int axis, int direction, const QString &frame, double linearMM, double angularDeg, std::optional<double> scale);
  void observeReason(const QString &reason);
  QString session() const;
  QJsonObject target() const;
  QString text(const QString &code, const QString &detail = {});
  // A refused Arm, led by the standing causes (olp-core armRefusalText).
  QString armRefusalText(const QString &code, const QString &detail);
};
