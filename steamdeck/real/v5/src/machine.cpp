#include "machine.hpp"
#include "cell_identity.hpp"
#include <QDateTime>
#include <QDebug>
#include <cmath>

namespace {
const QString dense = "/api/offline-programming/v1/dense-execution";
const QString targets = "/api/offline-programming/v1/targets";
const QString discoveredCells = "/api/offline-programming/v1/discovered-cells";
// OLP's cadences (densePanel.ts, jointJog.ts): status four times a second, a
// heartbeat every second against the server's 5 s deadman, hold intents every
// 50 ms against the server's 100 ms input expiry.
constexpr int statusPollMs = 250, heartbeatMs = 1000, holdCadenceMs = 50, catalogueRetryMs = 2000;
// OLP's DENSE_STATUS_POLL_FAST_MS (densePanel.ts) while the live robot is on
// screen: the server streams status at 20 Hz from a cache (~0.2 ms a read),
// and polling at twice that puts every frame on screen within half a period
// of its arrival; polling at exactly 20 Hz beats against the stream and
// stutters. The full view and the hold checks keep the 250 ms cadence.
constexpr int fastPollMs = 25;
// A move by angle answers only when it has finished; a whole-arm zero at 5 %
// takes minutes. Unverified bound: 600 s.
constexpr int moveTimeoutMs = 600000;
} // namespace

Machine::Machine(OlpClient &client_, OlpCore &core_, QObject *parent)
    : QObject(parent), client(client_), core(core_) {
  pollTimer.setSingleShot(true);
  catalogueRetry.setSingleShot(true);
  connect(&pollTimer, &QTimer::timeout, this, [this] { poll(); });
  connect(&heartbeatTimer, &QTimer::timeout, this, [this] { heartbeat(); });
  connect(&holdTimer, &QTimer::timeout, this, [this] { tick(); });
  connect(&catalogueRetry, &QTimer::timeout, this, [this] { refreshCatalogue(); });
  refresh();
}

Machine::~Machine() {
  // Closing ends a held lane; the pages being torn down hear nothing of it.
  changed = nullptr;
  status = nullptr;
  runProgressEnded = nullptr;
  endHold("app_closed");
}

QString Machine::text(const QString &code, const QString &detail) {
  try {
    return core.call("reasonText", {code, detail}).toString();
  } catch (const OlpError &) {
    return detail.isEmpty() ? code : code + ": " + detail;
  }
}

QString Machine::armRefusalText(const QString &code, const QString &detail) {
  try {
    return core.call("armRefusalText", {code, detail, maintenanceReason}).toString();
  } catch (const OlpError &) {
    return "Arm refused: " + text(code, detail);
  }
}

void Machine::setMaintenance(const QString &reason) {
  if (reason == maintenanceReason) return;
  maintenanceReason = reason;
  if (!reason.isEmpty()) {
    if (programPermit) {
      if (programExecutionActive()) programPermit = false; // server finishes unattended cycle
      else releaseProgramArm("maintenance");
    }
    ++armCancellation;
    armConfirm = false;
    endHold("maintenance");
  }
  refresh();
}

bool Machine::refused(bool programAction, const QString &withoutHold) {
  if (refusedForMaintenance() || refusedForCell()) return true;
  if ((!programAction && programPermit) || (armHoldRequired && !armPermit && !(programAction && programPermit))) {
    motionReason = programPermit ? QString("Program armed: disarm before Jog, Trace or Home")
                   : programAction && programArmOffered() ? QString("Arm for program on Run, or hold L2")
                                   : withoutHold;
    if (status) status(motionReason, true);
    return true;
  }
  return false;
}

bool Machine::refusedForMaintenance() {
  if (maintenanceReason.isEmpty()) return false;
  motionReason = maintenanceReason;
  if (status) status(maintenanceReason, true);
  if (changed) changed();
  return true;
}

void Machine::setExpectedCell(const QString &cell) {
  expectedCell = cell;
  refresh();
}

QString Machine::cellMismatch() const {
  if (expectedCell.isEmpty()) return {};
  if (!targetsSeen) return QString("Checking that the OLP server drives %1; arming and motion wait for it").arg(expectedCell);
  return cellidentity::mismatch(expectedCell, servedTargets, connectedCell, target().value("cell_id").toString());
}

void Machine::observeTargets(const QJsonObject &body) {
  targetsSeen = true;
  servedTargets = body;
  enforceCell();
  refresh();
}

void Machine::enforceCell() {
  if (cellMismatch().isEmpty()) return;
  if (programPermit) releaseProgramArm("cell_change");
  armConfirm = false;
  endHold("cell_change");
}

bool Machine::refusedForCell() {
  const QString why = cellMismatch();
  if (why.isEmpty()) return false;
  motionReason = why;
  if (status) status(why, true);
  if (changed) changed();
  return true;
}

QString Machine::session() const { return lastView.value("session").toObject()["id"].toString(); }
QJsonObject Machine::target() const { return lastView.value("target").toObject(); }

void Machine::refresh() {
  // As the Machine panel's select: the connected cell, else the first listed.
  const QJsonObject highlighted = !posture.isEmpty() ? posture : catalogue.isEmpty() ? QJsonObject{} : catalogue.first().toObject();
  QJsonObject input{
      {"catalogue", catalogue}, {"catalogueError", catalogueError.isEmpty() ? QJsonValue() : catalogueError},
      {"highlighted", highlighted.isEmpty() ? QJsonValue() : highlighted}, {"connectedCell", connectedCell},
      {"connectedLabel", posture.value("label").toString()}, {"modelId", highlighted["models"].toArray().first().toString()},
      {"connected", connected}, {"cellPending", false},
      {"connectError", connectError.isEmpty() ? QJsonValue() : connectError}, {"inFlight", inFlight},
      {"posture", posture.isEmpty() ? QJsonValue() : posture}, {"armConfirmPending", armConfirm},
      {"statusAgeMs", contacted ? QJsonValue(double(lastContact.elapsed())) : QJsonValue()},
      {"pendingRequest", pendingRequest.isEmpty() ? QJsonValue() : pendingRequest},
      {"lastRequest", lastRequest.isEmpty() ? QJsonValue()
                                            : QJsonObject{{"request", lastRequest},
                                                          {"error", lastRequestError.isEmpty() ? QJsonValue() : lastRequestError}}},
      {"jogBusy", inFlight}, {"fraction", speed},
      {"maintenance", maintenanceReason.isEmpty() ? QJsonValue() : maintenanceReason}};
  try {
    lastView = core.call("machineView", {input}).toObject();
    // As the Dense panel: a confirmation outlives neither the ability to arm
    // nor an arm that already happened.
    const auto flow = lastView.value("flow").toObject();
    if (armConfirm && (!flow["canArm"].toBool() || flow["armed"].toBool())) {
      armConfirm = false;
      input["armConfirmPending"] = false;
      lastView = core.call("machineView", {input}).toObject();
    }
  } catch (const OlpError &error) {
    lastView = {};
    if (status) status(QString("Machine view failed: %1").arg(error.what()), true);
  }
  const QString disabled = lastView.value("jogDisabled").toString();
  if (!disabled.isEmpty() && hold.active) {
    motionReason = disabled;
    endHold("jog_disabled");
  }
  // A refusal that only repeated the gate's words goes when the gate opens
  // (a program stopped, a loop ended): the reason line says ready again.
  if (!disabled.isEmpty()) {
    gateWords.insert(disabled);
  } else if (!gateWords.isEmpty()) {
    if (gateWords.contains(motionReason)) motionReason.clear();
    gateWords.clear();
  }
  if (changed) changed();
}

void Machine::refreshCatalogue() {
  client.get(targets, [this](const OlpClient::Reply &reply) {
    if (!reply.ok()) {
      catalogueError = reply.error;
      catalogueRetry.start(catalogueRetryMs);
    } else {
      catalogueError.clear();
      catalogue = reply.body.toObject()["targets"].toArray();
      refreshDiscovery(false);
    }
    refresh();
  });
}

void Machine::refreshDiscovery(bool force) {
  // The server listens about 1.5 s per search and probes health for up to 0.8 s.
  client.get(discoveredCells + (force ? "?refresh=1" : ""), [this](const OlpClient::Reply &reply) {
    discoveryError = reply.ok() ? QString() : reply.error;
    discovered = reply.ok() ? reply.body : QJsonValue();
    refresh();
  }, 8000);
}

void Machine::connectCell(const QString &cell) {
  if (inFlight || refusedForMaintenance()) return;
  if (!expectedCell.isEmpty() && cell != expectedCell) {
    motionReason = QString("This pendant was started for %1; connecting %2 is refused. Start the pendant's launcher for %2.").arg(expectedCell, cell);
    if (status) status(motionReason, true);
    if (changed) changed();
    return;
  }
  QJsonObject chosen;
  for (const auto &entry : catalogue)
    if (entry.toObject()["cell_id"] == cell) chosen = entry.toObject();
  if (chosen.isEmpty()) return;
  if (programPermit) { releaseProgramArm("cell_change"); return; }
  observedProgramActive = false;
  unattendedLoopAllowed = false;
  clearRunProgress();
  ++armCancellation;
  endHold("cell_change");
  armConfirm = false;
  inFlight = true;
  connectError.clear();
  const unsigned started = ++epoch;
  refresh();
  const QJsonObject selection{{"cell_id", cell}, {"model_id", chosen["models"].toArray().first().toString()}};
  client.post(targets + "/select", selection, [this, started, chosen](const OlpClient::Reply &reply) {
    if (started != epoch) return;
    if (!reply.ok()) {
      inFlight = false;
      connectError = reply.error;
      if (status) status(text(reply.error, reply.detail), true);
      refresh();
      return;
    }
    connectedCell = reply.body.toObject()["selected"].toObject()["cell_id"].toString();
    const QJsonValue selected = reply.body.toObject().value("target");
    posture = selected.isObject() ? selected.toObject() : chosen;
    client.get(dense + "/capabilities", [this, started](const OlpClient::Reply &caps) {
      if (started != epoch) return;
      inFlight = false;
      if (!caps.ok()) {
        connectError = caps.error;
        if (status) status(text(caps.error, caps.detail), true);
        refresh();
        return;
      }
      try {
        core.call("observeCapabilities", {caps.body});
      } catch (const OlpError &error) {
        connectError = error.what();
      }
      // Only a literal true offers program arming; an older OLP omits it.
      const QJsonValue unattended = caps.body.toObject().value("unattended_loop_allowed");
      unattendedLoopAllowed = unattended.isBool() && unattended.toBool();
      connected = true;
      contacted = false;
      lastRequest.clear();
      if (status) status("Connected · control and arming require explicit actions", false);
      heartbeatTimer.start(heartbeatMs);
      poll();
    });
  });
}

void Machine::disconnectCell() {
  if (programPermit) { releaseProgramArm("cell_change"); return; }
  observedProgramActive = false;
  clearRunProgress();
  ++armCancellation;
  endHold("cell_change");
  armConfirm = false;
  const unsigned started = ++epoch;
  inFlight = true;
  refresh();
  client.post(dense + "/target", QJsonObject{{"cell", ""}}, [this, started](const OlpClient::Reply &reply) {
    if (started != epoch) return;
    inFlight = false;
    if (!reply.ok()) {
      // A failed release keeps the selection: the machine stays ours to stop.
      if (status) status(text(reply.error, reply.detail), true);
      refresh();
      return;
    }
    connected = false;
    unattendedLoopAllowed = false;
    connectedCell.clear();
    posture = {};
    contacted = false;
    pollTimer.stop();
    heartbeatTimer.stop();
    client.setTarget({}, {});
    core.call("forgetMachine");
    core.call("telemetryForget");
    ioState={}; if (outputs) outputs({});
    if (status) status("Disconnected", false);
    refresh();
  });
}

void Machine::poll() {
  if (!connected || polling) return;
  polling = true;
  const unsigned started = epoch;
  const unsigned action = actionGeneration, observation = observationRevision;
  // Control polls need current execution evidence, not every past loop's
  // handle. Older OLP servers ignore the query and return the full status;
  // the compact view retains the same live fields and freshness checks.
  client.get(dense + "/status?view=control", [this, started, action, observation](const OlpClient::Reply &reply) {
    polling = false;
    if (started != epoch || !connected) return;
    // A status sampled before an Arm/Stop transition cannot acknowledge its
    // outcome. In particular, an old "armed" must not swallow the next Arm.
    if (action != actionGeneration || observation != observationRevision) {
      pollTimer.start(fast ? fastPollMs : statusPollMs);
      return;
    }
    const QJsonObject before = target();
    const QString beforeSession = session();
    try {
      if (reply.ok()) {
        core.call("observeStatus", {reply.body, teachBusy()});
        observeProgramStatus(reply.body.toObject());
        if (core.call("jogEndObserve", {reply.body, double(QDateTime::currentMSecsSinceEpoch()), OlpClient::clientId()}).toBool() && jogEnded)
          jogEnded();
        if (jogLive) jogLive(core.call("jogLiveFromStatus", {reply.body}).toObject());
        const auto rt=reply.body.toObject()["rt_core"].toObject();
        ioState=rt["status"].toObject()["core"].toObject()["io"].toObject();
        ioDefinition=rt["description"].toObject()["io"].toObject();
        if (outputs) outputs(core.call("cellOutputsFromStatus", {reply.body}).toObject());
        core.call("telemetrySample");
        lastContact.restart();
        contacted = true;
        if (!armPending && stopPending == 0) armStatusPending = false;
        if (pose) pose(core.call("livePose").toArray());
      } else {
        core.call("observeSilence", {reply.error});
        ioState={}; if (outputs) outputs({});
      }
    } catch (const OlpError &error) {
      core.call("observeSilence", {QString(error.what())});
      ioState={}; if (outputs) outputs({});
    }
    // Between full cycles a fast poll only moves the robot on screen.
    if (fast && reply.ok() && fullCycle.isValid() && fullCycle.elapsed() < statusPollMs) {
      pollTimer.start(fastPollMs);
      return;
    }
    fullCycle.restart();
    refresh();
    const QJsonObject now = target();
    client.setTarget(now["cell_id"].toString(), now["selection_generation"].toString());
    const QString ioCell=now["cell_id"].toString();
    if(!ioCell.isEmpty()&&ioCell!=ioSettingsCell){
      ioSettingsCell=ioCell;ioSettings={};core.call("setWeldDefaults",{QJsonObject{}});
      client.get("/api/offline-programming/v1/io-settings",[this,ioCell](const OlpClient::Reply& r){
        if(r.ok()&&target()["cell_id"].toString()==ioCell){ioSettings=r.body.toObject()["settings"].toObject();core.call("setWeldDefaults",{ioSettings});}
      });
    }

    // The status names another robot than the launcher's: let go now, not at
    // the next /targets check.
    enforceCell();
    if (hold.active && !before.isEmpty() &&
        (now["cell_id"] != before["cell_id"] || now["selection_generation"] != before["selection_generation"])) {
      motionReason = text("selection_changed");
      endHold("selection_changed");
    } else if (hold.active && !beforeSession.isEmpty() && session() != beforeSession) {
      motionReason = text("session_changed");
      endHold("session_changed");
    }
    // A held lane also reads its own state: a refusal or the deadman may have
    // retired it on the server, which must end the hold here too.
    if (hold.active && !hold.pacer.starting() && !hold.body.isEmpty()) {
      const unsigned generation = hold.generation;
      client.get(dense + (hold.cartesian ? "/cartesian/state" : "/jog/state"),
                 [this, generation](const OlpClient::Reply &state) {
        if (!hold.active || generation != hold.generation) return;
        const auto body = state.body.toObject();
        if (!state.ok()) { motionReason = text(state.error, state.detail); endHold("state_poll_failed"); return; }
        if (hold.trace && traceCompleted(body)) return;
        if (!body["reason"].toString().isEmpty() && body["revision"].toInt() > hold.body.value("revision").toInt())
          observeReason(body["reason"].toString());
      });
    }
    pollTimer.start(fast ? fastPollMs : statusPollMs);
  }, 2000);
}

void Machine::setFastPoll(bool on) {
  if (fast == on) return;
  fast = on;
  if (on && connected && !polling && pollTimer.isActive()) pollTimer.start(fastPollMs);
}

void Machine::heartbeat() {
  const auto current = lastView.value("session").toObject();
  if (!connected || current["id"].toString().isEmpty() || current["state"] == "stopped") return;
  client.post(dense + "/heartbeat", QJsonObject{{"session_id", current["id"].toString()}}, [](const OlpClient::Reply &) {});
}

void Machine::run(const QString &request, const QString &path, const QJsonObject &body, const QString &accepted,
                  int timeoutMs, std::function<void(const OlpClient::Reply &)> done) {
  const bool stopping = request == "stop" || request == "disarm";
  if (armRequestBusy() && !stopping) return;
  inFlight = true;
  if (request == "arm") armPending = true;
  if (path == "/home") ++homePending;
  if (stopping) { ++stopPending; armStatusPending = true; }
  const unsigned generation = ++actionGeneration;
  pendingRequest = request;
  const unsigned started = epoch;
  refresh();
  QElapsedTimer requestClock;
  requestClock.start();
  if (request == "arm" || stopping) qInfo() << "Control request:" << request << "sequence=" << generation;
  client.post(dense + path, body, [this, request, path, accepted, started, generation, done, requestClock](const OlpClient::Reply &reply) {
    if (request == "arm" || request == "stop" || request == "disarm")
      qInfo() << "Control reply:" << request << "sequence=" << generation << "elapsed_ms=" << requestClock.elapsed()
              << "http=" << reply.status;
    if (request == "arm") armPending = false;
    if (path == "/home") --homePending;
    if (request == "stop" || request == "disarm") --stopPending;
    if (request == "arm" || request == "stop" || request == "disarm") ++observationRevision;
    if (started != epoch) return;
    if (generation != actionGeneration) {
      // Release already sent Stop. Fence even an Arm whose HTTP delivery
      // raced that Stop; never publish its old armed status or allow a new
      // press while this Arm is unresolved.
      if (request == "arm") {
        // Fence the old Arm, without consuming a newer deliberate L2 press
        // waiting for this fence. No permit is granted until all Stops return.
        armPermit = false;
        programPermit = false;
        run("disarm", "/stop", {}, "Robot: disarmed");
      }
      return;
    }
    inFlight = false;
    pendingRequest.clear();
    lastRequest = request;
    const auto answer = reply.body.toObject();
    lastRequestError = !reply.ok() ? reply.error
                       : answer.contains("accepted") && !answer["accepted"].toBool() ? answer["error"].toString("arm_refused")
                       : answer.contains("stopped") && !answer["stopped"].toBool()   ? QString("stop_not_acknowledged")
                                                                                     : QString();
    // A poll sent during PLAY may still contain the pre-play loaded state.
    // Once PLAY is accepted it cannot revoke the observed program authority
    // and turn a later page/focus change into an unintended Stop.
    if (path == "/play" && lastRequestError.isEmpty()) ++observationRevision;
    if (request == "arm" && !lastRequestError.isEmpty()) { armPermit = false; programPermit = false; }
    if (request == "arm") qInfo().noquote() << "L2 arm result:" << (lastRequestError.isEmpty() ? "accepted" : lastRequestError);
    try {
      // /play and /home answer with the status itself; the others carry it.
      // Either was taken while this request was in flight: busy for teach, so
      // stillness restarts from the next poll.
      if (answer["status"].isObject()) { core.call("observeStatus", {answer["status"], true}); observeProgramStatus(answer["status"].toObject()); }
      else if (reply.ok() && (path == "/home" || path == "/play")) { core.call("observeStatus", {reply.body, true}); observeProgramStatus(answer); }
    } catch (const OlpError &) {
    }
    QString said = accepted;
    if (lastRequestError == "stop_not_acknowledged")
      said = QString("Robot: STOP NOT ACKNOWLEDGED after %1 attempts — use the hardware stop").arg(answer["attempts"].toInt());
    else if (!lastRequestError.isEmpty() && request == "arm")
      said = armRefusalText(lastRequestError, reply.detail);
    else if (!lastRequestError.isEmpty())
      said = text(lastRequestError, reply.detail);
    else if (path == "/stop" && answer.contains("attempts"))
      said = QString("Robot: stopped (acknowledged after %1 attempt%2), lease released")
                 .arg(answer["attempts"].toInt()).arg(answer["attempts"].toInt() == 1 ? "" : "s");
    if (status) status(said, !lastRequestError.isEmpty());
    if (done) done(reply);
    refresh();
    if (request == "arm" && !lastRequestError.isEmpty()) {
      // A lost/error reply does not prove Arm was unsent. Fence it with Stop,
      // retaining the useful refusal text after an acknowledged Stop.
      run("disarm", "/stop", {}, "Robot: disarmed", 4000, [this, said](const OlpClient::Reply &stopped) {
        if (stopped.ok() && stopped.body.toObject().value("stopped").toBool() && status) status(said, true);
      });
    }
  }, timeoutMs);
}

void Machine::loadProgram(const QJsonObject &identity, std::function<void(bool)> done) {
  // The upload to the cell earns its bytes' time: the server bounds one
  // exchange at 240 s (denseexec.ExchangeDeadline) after fetching the blob.
  constexpr int loadTimeoutMs = 300000;
  if (refused(true)) {
    if (done) done(false);
    return;
  }
  run("other", "/load", identity, "Robot: trajectory staged on the joint daemon", loadTimeoutMs,
      [done](const OlpClient::Reply &reply) {
    if (done) done(reply.ok());
  });
}

void Machine::playProgram() {
  if (refused(true)) return;
  const QString id = session();
  if (id.isEmpty()) {
    if (status) status("Load to robot first", true);
    return;
  }
  run("other", "/play", QJsonObject{{"session_id", id}}, "Robot: playing");
}

void Machine::arm() { armWithPermit(false); }

void Machine::armForProgram() {
  if (programPermit) return;
  if (!programArmOffered()) {
    motionReason = "Arm for program is not available on this cell (its manifest does not allow unattended loops); hold L2 to arm";
    if (status) status(motionReason, true);
    return;
  }
  if (armPermit || armRequestBusy() || hold.active || moving || running) {
    motionReason = "Release L2 and finish the current command before arming for a program";
    if (status) status(motionReason, true);
    return;
  }
  if (!connected || !statusAgeMs() || *statusAgeMs() >= 1000 || refusedForMaintenance() || refusedForCell()) {
    if (motionReason.isEmpty()) motionReason = "Connect and wait for fresh robot status before arming for a program";
    if (status) status(motionReason, true);
    return;
  }
  programPermit = true;
  armWithPermit(true);
}

void Machine::releaseProgramArm(const QString &cause) {
  if (!programPermit) return;
  qInfo() << "Program arm released:" << cause;
  programPermit = false;
  disarm();
}

void Machine::observeProgramStatus(const QJsonObject &body) {
  const QString state = body["session"].toObject()["state"].toString();
  const QString loop = body["loop"].toObject()["state"].toString();
  observedProgramActive = state == "playing" || state == "paused" || loop == "running" || loop == "finishing";
  const auto rt = body["rt_core"].toObject();
  const auto native = rt["status"].toObject()["core"].toObject();
  const bool fault = native["safety_fault_mask"].toDouble() != 0 || native["execution_fault_reasons"].toDouble() != 0 ||
                     rt["inhibited"].toBool() || rt["stop_uncertain"].toBool();
  if (fault) {
    observedProgramActive = false;
    if (programPermit) releaseProgramArm("controller_fault");
  } else if (programPermit && !armPending && !inFlight && body["armed"].isBool() && !body["armed"].toBool() && !observedProgramActive) {
    // A fresh disarmed observation consumes this explicit choice. Reconnection
    // and reset never restore program arming automatically.
    programPermit = false;
    ++armCancellation;
    endRun("Program arm ended by the controller");
  }
  if (!observedProgramActive) clearRunProgress();
}

void Machine::armWithPermit(bool programAction) {
  auto revoke = [this, programAction] { if (programAction) programPermit = false; else armPermit = false; };
  if (refused(programAction)) { revoke(); return; }
  if (armRequestBusy()) {
    revoke();
    if (status) status("Arm waiting for the previous command; release L2 and try again", true);
    return;
  }
  refresh();
  const auto flow = lastView.value("flow").toObject();
  if (flow["armed"].toBool() && flow["arm"].toObject()["controlled"].toBool()) {
    qInfo() << "L2 arm: already armed";
    return;
  }
  if (!programAction && armHoldRequired && homeStepCurrent()) {
    // Home runs disarmed (rt-core refuses it armed, mode_conflict) and Arm is
    // refused until Home is valid, so this press sends no Arm: it keeps the
    // permit for Home, as its deadman (owner decision 2026-11-03: Home briefly
    // energises the drives). 2026-11-03, a Steam Deck on the mini bench after
    // a gear-ratio change: holding L2 sent Arm, Arm was refused "Home first"
    // and the permit went with it, so Home could never be sent.
    homeHold = true;
    motionReason = "L2 held: press HOME JOINTS to Home";
    qInfo() << "L2 held for Home: no Arm sent";
    if (status) status(motionReason, false);
    if (changed) changed();
    return;
  }
  if (!flow["canArm"].toBool()) {
    revoke();
    // The Arm step says why (an unconfirmed Stop, a drive alarm, Home), as
    // the browser's arm popover does; the fact's detail only names the state.
    QString why;
    for (const auto &step : flow["steps"].toArray())
      if (step.toObject()["id"] == "arm") why = step.toObject()["summary"].toString();
    qInfo().noquote() << "L2 arm unavailable:" << why;
    if (status) status(why.isEmpty() ? flow["arm"].toObject()["detail"].toString("Connect to a cell first") : "Arm unavailable: " + why, true);
    return;
  }
  if (!programAction && !armHoldRequired && flow["armConfirmRequired"].toBool() && !armConfirm) {
    armConfirm = true;
    refresh();
    return;
  }
  armConfirm = false;
  // "program" lets OLP refuse a program arm on a cell without unattended
  // loops, whatever this client believes (dense_execution.go).
  QJsonObject body{{"armed", true}};
  if (programAction) body["program"] = true;
  run("arm", "/arm", body, "Robot: arm accepted by RTCore");
}

void Machine::cancelArm() {
  armConfirm = false;
  refresh();
}

void Machine::disarm() {
  ++armCancellation;
  armPermit = false;
  programPermit = false;
  armConfirm = false;
  recovering = false;
  endHold("operator_disarm");
  moving = false;
  endRun("Run program ended by DISARM");
  run("disarm", "/stop", {}, "Robot: disarmed");
}

bool Machine::homeStepCurrent() const {
  for (const auto &step : lastView.value("flow").toObject().value("steps").toArray())
    if (step.toObject().value("id").toString() == "home") return step.toObject().value("state").toString() == "current";
  return false;
}

bool Machine::homeInProgress() const {
  if (homePending) return true;
  for (const auto &joint : lastView.value("joints").toArray())
    if (joint.toObject().value("homeState").toString() == "homing") return true;
  return false;
}

void Machine::releaseArmHold() {
  // A Home permit with nothing that could be energised clears without a
  // request. Anything else Stops: a Home in flight or reported in progress
  // (OLP cancels the Home operation and Stops rt-core, which drains the
  // commissioning job and clears its service permission), a request in
  // flight, an arm state that is not observed disarmed.
  const QJsonValue armed = lastView.value("flow").toObject().value("armed");
  if (homeHoldActive() && !inFlight && !homeInProgress() && armed.isBool() && !armed.toBool()) {
    armPermit = false;
    homeHold = false;
    if (motionReason.startsWith("L2 held:")) motionReason.clear();
    qInfo() << "L2 released: Home permit cleared, nothing to stop";
    refresh();
    return;
  }
  const bool home = homeHoldActive() && homeInProgress();
  homeHold = false;
  disarm();
  if (home) {
    motionReason = "Home stopped: L2 released";
    qInfo() << "L2 released during Home: Stop sent";
    if (changed) changed();
  }
}

void Machine::stop() {
  ++armCancellation;
  armPermit = false;
  programPermit = false;
  // Stop is always sent, whatever else is in flight.
  endHold("operator_stop");
  moving = false;
  recovering = false;
  endRun("Run program ended by STOP");
  armConfirm = false;
  run("stop", "/stop", {}, "Stopped by operator");
}

void Machine::endRun(const QString &why) {
  clearRunProgress();
  if (!running) return;
  running = false;
  motionReason = why;
  if (runDone) runDone(QJsonObject{{"state", "stopped"}, {"text", why}});
}

void Machine::clearRunProgress() {
  if (runProgressMessage.isEmpty()) return;
  const QString message = runProgressMessage;
  runProgressMessage.clear();
  if (motionReason == message) motionReason.clear();
  if (runProgressEnded) runProgressEnded(message);
}

// Home needs the held permit like every robot action (owner decision
// 2026-11-03: it briefly energises the drives). While Home is required an L2
// press grants that permit without arming (arm()); releasing L2 during Home
// Stops it (releaseArmHold()).
void Machine::home(const QList<int> &axes) {
  if (refused(false, "Hold L2, then press Home")) return;
  QJsonArray list;
  for (int axis : axes) list << axis;
  run("other", "/home", axes.isEmpty() ? QJsonObject{} : QJsonObject{{"axes", list}}, "Home requested");
}

void Machine::setFraction(double fraction) {
  // A path hold keeps its line and changes speed with the next cadence
  // intent; proportional holds pin their ceiling, and changing that ceiling
  // ends the hold. Fixed joint holds and cells without path holds also end.
  if (hold.active && (hold.trace || (!hold.proportional && hold.cartesian && fractionUpdates)) && !hold.body.isEmpty() && fraction > 0 && fraction <= 1)
    hold.body["fraction"] = fraction;
  else if (hold.active)
    endHold("speed_change");
  speed = fraction;
  refresh();
}

void Machine::setHoldScale(double scale) {
  if (!hold.active || !hold.proportional) return;
  if (!std::isfinite(scale) || scale <= 0 || scale > 1) {
    endHold("stick_neutral");
    return;
  }
  // Keep only the latest sample; the cadence never drains an input queue.
  hold.inputScale = scale;
}

void Machine::beginJointHold(int axis, int direction, std::optional<double> scale) { beginHold(false, axis, direction, {}, 0, 0, scale); }

void Machine::beginCartesianHold(const QString &frame, int axis, int direction, double linearMM, double angularDeg, std::optional<double> scale) {
  beginHold(true, axis, direction, frame, linearMM, angularDeg, scale);
}

void Machine::beginHold(bool cartesian, int axis, int direction, const QString &frame, double linearMM, double angularDeg, std::optional<double> scale) {
  if (hold.active || holdEnding() || moving || !direction || refused()) return;
  core.call("traceForget", {true});
  if (scale && (!std::isfinite(*scale) || *scale <= 0 || *scale > 1)) return;
  refresh();
  const QString disabled = lastView.value("jogDisabled").toString();
  if (!disabled.isEmpty()) {
    motionReason = disabled;
    if (changed) changed();
    return;
  }
  QJsonArray twist;
  if (cartesian) {
    try {
      twist = core.call("holdTwist", {axis, direction, linearMM, angularDeg, speed}).toArray();
    } catch (const OlpError &error) {
      motionReason = error.what();
      if (changed) changed();
      return;
    }
  }
  hold = Hold{true, cartesian, axis, direction, hold.generation + 1, {}, cartesian ? "/cartesian" : "/jog", {}, {}};
  hold.proportional = scale.has_value();
  hold.inputScale = scale.value_or(1);
  hold.pacer.pressSent();
  motionReason.clear();
  const unsigned generation = hold.generation;
  const QString cell = lastView.value("cell").toString(), id = session();
  refresh();
  client.get(dense + hold.route + (cartesian ? "/state" : "/state"),
             [this, generation, cartesian, axis, direction, frame, twist, cell, id](const OlpClient::Reply &state) {
    if (!hold.active || generation != hold.generation) return;
    const auto body = state.body.toObject();
    if (!state.ok() || body["active"].toBool()) {
      motionReason = state.ok() ? text("jog_mode_conflict") : text(state.error, state.detail);
      hold.pacer.pressAnswered();
      endHold(state.ok() ? QString("intent_refused:jog_mode_conflict") : "intent_refused:" + state.error);
      return;
    }
    fractionUpdates = cartesian && body["fraction_updates"].toBool();
    if (hold.proportional && (!body["proportional_input"].toBool() || (cartesian && !fractionUpdates))) {
      motionReason = "This OLP needs an update for proportional joystick jogging";
      hold.pacer.pressAnswered();
      endHold("intent_refused:proportional_input_unavailable");
      return;
    }
    hold.pacer.allowPipelining(body["pipelined_intents"].toBool());
    QJsonObject intent{{"target_id", cell}, {"session_id", id}, {"revision", body["revision"]}, {"fraction", speed}};
    if (hold.proportional) {
      intent["fraction"] = speed * hold.inputScale;
      intent["max_fraction"] = speed;
      intent["input_sequence"] = double(++hold.inputSequence);
    }
    if (cartesian) {
      intent["frame"] = frame;
      intent["twist"] = twist;
    } else {
      intent["axis"] = axis;
      intent["direction"] = direction;
    }
    hold.body = intent;
    // Match JSLOP Xbox's 20 ms speed cadence; the native acceleration/jerk
    // limits still shape motion. This is not a measured physical latency.
    holdTimer.setTimerType(Qt::PreciseTimer);
    holdTimer.start(hold.proportional ? 20 : holdCadenceMs);
    client.post(dense + hold.route + (cartesian ? "/start" : "/begin"), intent,
                [this, generation](const OlpClient::Reply &reply) {
      if (generation != hold.generation) return;
      hold.pacer.pressAnswered();
      if (!reply.ok()) {
        motionReason = text(reply.error, reply.detail);
        endHold("intent_refused:" + reply.error);
      }
      if (changed) changed();
    });
  });
}

void Machine::tick() {
  // The cadence runs while the press or intents wait on a slow reply; only the
  // in-flight bound skips a tick (hold_pacer.hpp). Intents leave only while
  // the input is held: release stops this timer and aborts them.
  if (!hold.active || hold.body.isEmpty() || !hold.pacer.maySendIntent()) return;
  const QString release = pendingInputRelease ? pendingInputRelease(hold.trace) : QString();
  if (!release.isEmpty()) { endHold(release); return; }
  const bool early = hold.pacer.intentSent();
  const unsigned generation = hold.generation;
  if (hold.trace && !core.call("traceCurrent").toBool()) { endHold("program_changed"); return; }
  if (hold.trace) {
    hold.body["fraction"] = speed;
    hold.body["input_sequence"] = double(++hold.inputSequence);
  }
  if (hold.proportional) {
    hold.body["fraction"] = hold.body["max_fraction"].toDouble() * hold.inputScale;
    hold.body["input_sequence"] = double(++hold.inputSequence);
  }
  hold.intents.removeAll(QPointer<QNetworkReply>());
  hold.intents << client.post(dense + hold.route + (hold.cartesian ? "/intent" : "/update"), hold.body,
                              [this, generation, early](const OlpClient::Reply &reply) {
    if (generation != hold.generation) return;
    hold.pacer.intentAnswered();
    if (hold.trace && reply.ok() && traceCompleted(reply.body.toObject())) return;
    if (!reply.ok() && HoldPacer::endsHold(reply.error, early)) {
      const bool reconcile = hold.trace && reply.error == "jog_session_stale";
      motionReason = text(reply.error, reply.detail);
      endHold("intent_refused:" + reply.error);
      if (reconcile) {
        const unsigned ended = hold.generation;
        const unsigned connection = epoch;
        client.get(dense + "/trace/state", [this, ended, connection](const OlpClient::Reply &state) {
          if (epoch == connection && hold.generation == ended && !hold.active && state.ok() && core.call("traceCurrent").toBool())
            traceCompleted(state.body.toObject());
        });
      }
    }
  });
}

void Machine::observeReason(const QString &reason) {
  motionReason = text(reason);
  endHold("server_reason:" + reason);
}

void Machine::endHold(const QString &cause) {
  if (!hold.active) return;
  hold.active = false;
  ++hold.generation;
  holdTimer.stop();
  if (hold.planning) OlpClient::cancel(hold.planning);
  // Their replies belong to the released generation and are ignored.
  for (const auto &reply : std::as_const(hold.intents))
    if (reply) OlpClient::cancel(reply);
  hold.intents.clear();
  // Why the pendant let go (the server journals and classifies it; OLP's
  // execution/jogEndCause.ts lists the words). None is a plain release.
  QJsonObject body = hold.body;
  if (!cause.isEmpty()) body["end_cause"] = cause;
  if (!hold.body.isEmpty()) {
    ++holdEndPending;
    const unsigned connection = epoch;
    client.post(dense + hold.route + (hold.cartesian ? "/halt" : "/end"), body, [this, connection](const OlpClient::Reply &reply) {
      --holdEndPending;
      // A path End acknowledges native closure, not just receipt. Failure or
      // a stale reply is not permission for an automatic axis handoff.
      if (connection == epoch && !reply.ok()) ++holdEndFailure;
      // A deadman or refusal may already have retired this revision; keep its reason.
      if (!reply.ok() && reply.error != "jog_session_stale" && motionReason.isEmpty())
        motionReason = text(reply.error, reply.detail);
      if (changed) changed();
    });
  }
  refresh();
}

void Machine::moveJoint(int axis, double degrees) {
  if (moving || hold.active || refused()) return;
  refresh();
  const QString disabled = lastView.value("jogDisabled").toString();
  QString refusal = disabled;
  if (refusal.isEmpty() && (!std::isfinite(degrees) || degrees == 0)) refusal = "Enter a finite absolute position or a nonzero signed angle";
  if (refusal.isEmpty()) refusal = core.call("moveRefusal", {axis, degrees}).toString();
  if (!refusal.isEmpty()) {
    motionReason = refusal;
    if (changed) changed();
    return;
  }
  QJsonArray moves{QJsonObject{{"axis", axis}, {"degrees", degrees}}};
  moving = true;
  motionReason = "moving";
  const unsigned started = epoch;
  const QString cell = lastView.value("cell").toString(), id = session();
  refresh();
  client.get(dense + "/jog/state", [this, started, cell, id, axis, degrees](const OlpClient::Reply &state) {
    if (started != epoch || !moving) return;
    const auto body = state.body.toObject();
    if (!state.ok() || body["active"].toBool() || body["move"].toObject()["state"] == "moving") {
      moving = false;
      motionReason = state.ok() ? text("jog_mode_conflict") : text(state.error, state.detail);
      refresh();
      return;
    }
    const QJsonObject intent{{"target_id", cell}, {"session_id", id}, {"revision", body["revision"]},
                             {"axis", axis}, {"degrees", degrees}, {"fraction", speed}};
    core.call("traceForget", {true});
    client.post(dense + "/move", intent, [this, started](const OlpClient::Reply &reply) {
      if (started != epoch || !moving) return;
      moving = false;
      motionReason = reply.ok() ? "done" : text(reply.error, reply.detail);
      refresh();
    }, moveTimeoutMs);
  });
}

// OLP's All joints to 0°, which is also the one-press recovery: the server
// confirms an unconfirmed Stop, resets faults whose recovery is a reset and,
// only after the operator confirms (A; B stops and declines), arms and runs
// the move. It stops at the first step that does not clear; the reason line
// shows the server's sentence. olp-core writes every word (recoverToZero.ts).
void Machine::zeroAll(double linearSpeedMMs, double angularSpeedDegS) {
  if (moving || hold.active || refused()) return;
  refresh();
  const QString refusal = lastView.value("zeroDisabled").toString();
  if (!refusal.isEmpty()) {
    motionReason = refusal;
    if (changed) changed();
    return;
  }
  if (!std::isfinite(linearSpeedMMs) || linearSpeedMMs <= 0 || !std::isfinite(angularSpeedDegS) || angularSpeedDegS <= 0) {
    motionReason = "Choose positive tool travel and rotation speeds before returning to zero";
    if (changed) changed();
    return;
  }
  recovering = true;
  const QJsonObject rates{{"linearSpeedMMs", linearSpeedMMs}, {"angularSpeedDegS", angularSpeedDegS}};
  recoverPress(core.call("recoverBody", {speed, QJsonValue(), rates}).toObject());
}

void Machine::recoverPress(const QJsonObject &body) {
  moving = true;
  motionReason = "moving";
  const unsigned started = epoch;
  refresh();
  // An armed request moves directly; otherwise the first request only asks
  // for confirmation. A declined question must not disturb the Trace cursor.
  if (body["confirm_arm"].toBool() || lastView["flow"].toObject()["armed"].toBool())
    core.call("traceForget", {true});
  client.post(dense + "/recover-to-zero", body, [this, started, body](const OlpClient::Reply &reply) {
    if (started != epoch || !moving || !recovering) return;
    moving = false;
    QJsonObject answer;
    try {
      answer = core.call("recoverAnswer", {reply.body, reply.ok(), reply.error, reply.detail, body}).toObject();
    } catch (const OlpError &error) {
      answer = QJsonObject{{"state", "refused"}, {"text", QString(error.what())}};
    }
    if (answer["state"] == "confirm_arm") {
      motionReason = answer["text"].toString();
      refresh();
      const bool confirmed = confirmRecover && confirmRecover(answer["confirmation"].toObject());
      // STOP (B) during the question ends the press as well as declining it.
      if (started != epoch || !recovering) return;
      if (!confirmed) {
        recovering = false;
        motionReason = "All joints to 0° cancelled: nothing was armed or moved";
        refresh();
        return;
      }
      // Keep the speed shown in the confirmation even if settings change
      // while the question is open. Only the confirmed joint evidence changes.
      const QJsonObject rates{{"linearSpeedMMs", body["linear_speed_mm_s"]}, {"angularSpeedDegS", body["angular_speed_deg_s"]}};
      recoverPress(core.call("recoverBody", {body["fraction"], answer["result"], rates}).toObject());
      return;
    }
    recovering = false;
    motionReason = answer["text"].toString();
    if (status) status("All joints to 0°: " + motionReason, answer["state"] == "refused");
    refresh();
  }, moveTimeoutMs);
}

// OLP's Run program: the server runs the sequence and stops at the first step
// that does not clear with its sentence; the operator confirms (A; B stops
// and declines) before it arms or moves. olp-core writes every word
// (runProgram.ts) and STOP ends the press: the server fences the rest by
// STOP's epoch, and a question answered after STOP sends nothing.
void Machine::runProgram(const QJsonObject &base, std::function<void(const QJsonObject &)> done) {
  const QString busy = running ? "Run program is already in progress; STOP ends it"
                       : moving ? "A move from this pendant is still in progress; press again when it has finished"
                       : hold.active ? "This pendant holds a jog; release it, then press again"
                                     : QString();
  if (!busy.isEmpty() || refused(true)) {
    if (pressNotice) pressNotice(core.call("runNotSentRefusal", {busy.isEmpty() ? motionReason : busy}).toObject());
    return;
  }
  if (pressNotice) pressNotice({});
  running = true;
  runBase = base;
  runDone = std::move(done);
  runPress(core.call("runBody", {runBase, speed}).toObject());
}

void Machine::runPress(const QJsonObject &body) {
  // Go to plan start and the upload to the cell each earn their own bound on
  // the server. Unverified client bound: 10 minutes.
  constexpr int runTimeoutMs = 10 * 60 * 1000;
  clearRunProgress();
  moving = true;
  motionReason = "Run program…";
  const unsigned started = epoch;
  refresh();
  client.post(dense + "/run-program", body, [this, started](const OlpClient::Reply &reply) {
    if (started != epoch || !running) return;
    moving = false;
    QJsonObject answer;
    try {
      const auto payload = reply.body.toObject();
      const QJsonValue observed = payload["status"].isObject() ? payload["status"] : payload["run"].toObject()["status"];
      if (observed.isObject()) core.call("observeStatus", {observed, true});
      answer = core.call("runAnswer", {reply.body, reply.ok(), reply.error, reply.detail}).toObject();
    } catch (const OlpError &error) {
      answer = QJsonObject{{"state", "refused"}, {"text", QString(error.what())},
                           {"refusal", QJsonObject{{"kicker", "RUN PROGRAM REFUSED"}, {"text", QString(error.what())}, {"details", QJsonArray{}}, {"code", "olp_core_error"}}}};
    }
    if (answer["state"] == "confirm") {
      motionReason = answer["text"].toString();
      refresh();
      const bool confirmed = confirmRecover && confirmRecover(answer["confirmation"].toObject());
      // STOP (B) during the question ends the press as well as declining it.
      if (started != epoch || !running) return;
      if (!confirmed) {
        running = false;
        motionReason = answer["cancelled"].toString();
        if (runDone) runDone(QJsonObject{{"state", "cancelled"}, {"text", motionReason}});
        refresh();
        return;
      }
      if (refused(true)) { endRun("Run program cancelled: arm permission ended"); return; }
      runPress(core.call("runBody", {runBase, speed, answer["result"]}).toObject());
      return;
    }
    if (answer["state"] == "playing") {
      observedProgramActive = true;
      ++observationRevision; // ignore a status sampled before this accepted start
    }
    running = false;
    motionReason = answer["text"].toString();
    if (answer["state"] == "playing") runProgressMessage = motionReason;
    if (status) status("Run program: " + motionReason, answer["state"] == "refused");
    if (pressNotice && answer["refusal"].isObject()) pressNotice(answer["refusal"].toObject());
    if (runDone) runDone(answer);
    refresh();
  }, runTimeoutMs);
}

void Machine::finishLoop() {
  client.post(dense + "/run-program/finish", QJsonObject{}, [this](const OlpClient::Reply &reply) {
    if (!reply.ok()) {
      motionReason = text(reply.error, reply.detail);
      if (status) status("Finish cycle: " + motionReason, true);
      if (pressNotice) pressNotice(core.call("finishRefusalOf", {reply.error, reply.detail.isEmpty() ? motionReason : reply.detail}).toObject());
    }
    refresh();
  });
}

void Machine::moveToPositions(const QJsonArray &positions, std::function<void(bool)> done) {
  if (moving || hold.active) return;
  if (refused(true)) {
    if (done) done(false);
    return;
  }
  refresh();
  // QJsonValue(...): a bare QJsonArray in braces copies it, passing its elements as the arguments.
  const auto planned = core.call("absoluteMoves", {QJsonValue(positions)}).toObject();
  moveAxes(planned["moves"].toArray(), planned["refusal"].toString(), std::move(done));
}

void Machine::moveAxes(const QJsonArray &moves, QString refusal, std::function<void(bool)> done) {
  if (refusal.isEmpty()) refusal = lastView.value("jogDisabled").toString();
  if (refusal.isEmpty() && session().isEmpty()) refusal = "No grant; ARM the machine (step 4) before jogging";
  if (!refusal.isEmpty() || moves.isEmpty()) {
    motionReason = refusal;
    if (done) done(false);
    if (changed) changed();
    return;
  }
  moving = true;
  motionReason = "moving";
  const unsigned started = epoch;
  const QString cell = lastView.value("cell").toString(), id = session();
  refresh();
  client.get(dense + "/jog/state", [this, started, cell, id, moves, done](const OlpClient::Reply &state) {
    if (started != epoch || !moving) return;
    const auto body = state.body.toObject();
    if (!state.ok() || body["active"].toBool()) {
      moving = false;
      motionReason = state.ok() ? text("jog_mode_conflict") : text(state.error, state.detail);
      if (done) done(false);
      refresh();
      return;
    }
    const auto first = moves.first().toObject();
    const QJsonObject intent{{"target_id", cell}, {"session_id", id}, {"revision", body["revision"]},
                             {"axis", first["axis"]}, {"degrees", first["degrees"]}, {"fraction", speed}, {"axes", moves}};
    core.call("traceForget", {true});
    client.post(dense + "/move", intent, [this, started, done](const OlpClient::Reply &reply) {
      if (started != epoch || !moving) return;
      moving = false;
      motionReason = reply.ok() ? "done" : text(reply.error, reply.detail);
      if (done) done(reply.ok());
      refresh();
    }, moveTimeoutMs);
  });
}

void Machine::cartesianStep(const QString &frame, int axis, int direction, double distanceMM, double angleDeg) {
  if (moving || hold.active || !direction || refused()) return;
  refresh();
  QString refusal = lastView.value("jogDisabled").toString();
  if (!refusal.isEmpty()) {
    motionReason = refusal;
    if (changed) changed();
    return;
  }
  try {
    core.call("holdTwist", {axis, direction, distanceMM, angleDeg, speed}); // OLP's range check on the boxes
  } catch (const OlpError &error) {
    motionReason = error.what();
    if (changed) changed();
    return;
  }
  moving = true;
  motionReason = "moving";
  const unsigned started = epoch;
  const QString cell = lastView.value("cell").toString(), id = session();
  refresh();
  client.get(dense + "/cartesian/state", [=](const OlpClient::Reply &state) {
    if (started != epoch || !moving) return;
    const auto body = state.body.toObject();
    if (!state.ok() || body["active"].toBool()) {
      moving = false;
      motionReason = state.ok() ? text("jog_mode_conflict") : text(state.error, state.detail);
      refresh();
      return;
    }
    const QJsonObject intent{{"target_id", cell}, {"session_id", id}, {"revision", body["revision"]}, {"axis", axis},
                             {"degrees", direction * (axis < 3 ? distanceMM : angleDeg)}, {"fraction", speed}};
    const QJsonValue request = core.call("stepBody", {intent, frame});
    core.call("traceForget", {true});
    client.post(dense + "/cartesian/move", request, [this, started](const OlpClient::Reply &reply) {
      if (started != epoch || !moving) return;
      moving = false;
      motionReason = reply.ok() ? "done" : text(reply.error, reply.detail);
      refresh();
    }, moveTimeoutMs);
  });
}

void Machine::setForegroundPlanning(bool active) {
  if (foregroundPlanning == active) return;
  foregroundPlanning = active;
  // The async JS packer and a cancelled HTTP reply may finish after Plan
  // starts. Neither may publish an old result or restart background work.
  ++tracePreparationGeneration;
  if (active) {
    tracePreparingKey.clear();
    tracePreparationRetryable = false;
    tracePreparationDeferred = false;
    if (tracePreparation) OlpClient::cancel(tracePreparation);
    tracePreparation = nullptr;
  }
  traceDebounce.restart();
}

bool Machine::tracePreparationIdle() const {
  if (foregroundPlanning || !connected || !cellMismatch().isEmpty() || maintenance() || armRequestBusy() ||
      hold.active || holdEnding() || moving || running || recovering || observedProgramActive) return false;
  const QString execution = lastView["executionState"].toString();
  return execution != "playing" && execution != "running" && execution != "paused";
}

void Machine::prepareTrace(bool committedTeach) {
  if (!tracePreparationIdle()) return;
  QString key;
  try { key = core.call("traceKey").toString(); }
  catch (const OlpError &error) {
    // Preparation is advisory: a key/readiness error must not turn an
    // already committed and locally saved waypoint into a Record failure.
    const QString reason = QString::fromUtf8(error.what());
    if (tracePreparationError != reason) qInfo() << "Trace preparation unavailable:" << reason;
    tracePreparationError = reason;
    if (!tracePreparationKey.isEmpty() || !tracePreparedKey.isEmpty() || !tracePreparingKey.isEmpty() ||
        tracePreparation || !tracePreparationLoad.isEmpty() || tracePreparationDeferred) {
      // Retire callbacks before cancelling their transport. They must not
      // re-read a persistently failing key or publish the previous revision.
      ++tracePreparationGeneration;
      if (tracePreparation) OlpClient::cancel(tracePreparation);
      tracePreparation = nullptr;
      tracePreparationKey.clear(); tracePreparedKey.clear(); tracePreparingKey.clear();
      tracePreparationLoad = {}; tracePreparationRetryable = false; tracePreparationDeferred = false;
      traceDebounce.restart();
    }
    return;
  }
  if (key != tracePreparationKey || tracePreparationEpoch != epoch) {
    tracePreparationEpoch = epoch;
    tracePreparationKey = key; tracePreparedKey.clear(); tracePreparingKey.clear(); tracePreparationError.clear();
    tracePreparationLoad = {}; tracePreparationRetryable = false; tracePreparationDeferred = false;
    ++tracePreparationGeneration;
    if (tracePreparation) OlpClient::cancel(tracePreparation);
    tracePreparation = nullptr; traceDebounce.restart();
    if (!committedTeach) return;
  }
  // Planning can finish while a new joystick hold owns the lane. Its one
  // validated load stays here, without a conflicting HTTP prepare or a second
  // planner request. Reconcile the current identity above before sending it.
  if (tracePreparationDeferred) { sendTracePreparation(key); return; }
  if (tracePreparingKey == key && tracePreparationRetryable && traceDebounce.elapsed() >= 750)
    tracePreparingKey.clear();
  if (tracePreparedKey == key || tracePreparingKey == key || !traceDebounce.isValid() ||
      (!committedTeach && traceDebounce.elapsed() < 750)) return;
  tracePreparationError.clear(); tracePreparationRetryable = false; traceDebounce.restart();
  tracePreparingKey = key;
  const unsigned generation = ++tracePreparationGeneration;
  // A jog can start while planning runs. Keep the validated geometry and
  // retry only its non-moving preparation after that jog ends.
  if (!tracePreparationLoad.isEmpty()) { sendTracePreparation(key); return; }
  core.callAsync("tracePlanRequest", {}, [this, key, generation](std::optional<QJsonValue> value, const QString &error) {
    if (foregroundPlanning || tracePreparationEpoch != epoch || generation != tracePreparationGeneration || key != tracePreparationKey || key != core.call("traceKey").toString()) return;
    if (!value) { tracePreparationError = error; return; }
    const auto request = value->toObject();
    if (request.contains("load")) { prepareTraceLoad(key, request); return; }
    tracePreparation = client.post("/api/offline-programming/v1/weld-plan", request["planning"],
      [this, key, generation](const OlpClient::Reply &reply) {
        if (foregroundPlanning || tracePreparationEpoch != epoch || generation != tracePreparationGeneration || key != tracePreparationKey || key != core.call("traceKey").toString()) return;
        tracePreparation = nullptr;
        if (!reply.ok()) { tracePreparationFailed(reply); return; }
        prepareTraceLoad(key, reply.body.toObject());
      }, 120000);
  });
}

void Machine::prepareTraceLoad(const QString &key, const QJsonObject &result) {
  QJsonObject load;
  // The cursor needs identities and segment boundaries, not megabytes of
  // encoded trajectory. Keep transport bytes out of QJSEngine/UI conversion.
  auto metadata = result;
  const auto bytes = metadata.take("dense_blob_base64");
  try { load = core.call("tracePrepared", {key, metadata}).toObject(); }
  catch (const OlpError &e) { tracePreparationError = e.what(); return; }
  if (bytes.isString()) load["dense_blob_base64"] = bytes;
  if (!load.contains("dense_blob_base64") && keptTraceBytes) {
    const QString bytes = keptTraceBytes(load["trajectory_digest"].toString());
    if (!bytes.isEmpty()) load["dense_blob_base64"] = bytes;
  }
  tracePreparationLoad = load;
  sendTracePreparation(key);
}

void Machine::tracePreparationFailed(const OlpClient::Reply &reply) {
  tracePreparationError = text(reply.error, reply.detail);
  // Only transient availability conditions are retried, never a rejected
  // path, changed calibration, limits or a safety fault. This prepares bytes;
  // it cannot queue a Trace press, acquire a grant or start motion.
  tracePreparationRetryable = reply.error == "jog_mode_conflict" || reply.error == "planning_start_moving" ||
    reply.error == "client_not_focused" || reply.error == "olp_server_unreachable" || reply.error == "dense_unavailable";
  traceDebounce.restart();
  qInfo() << "Trace preparation failed:" << reply.error << reply.detail << "retry when idle=" << tracePreparationRetryable;
}

void Machine::sendTracePreparation(const QString &key) {
  if (!tracePreparationIdle()) { tracePreparationDeferred = true; return; }
  tracePreparationDeferred = false;
  const unsigned generation = tracePreparationGeneration;
  tracePreparation = client.post(dense + "/trace/prepare",
    QJsonObject{{"target_id", lastView["cell"]}, {"trajectory_digest", tracePreparationLoad["trajectory_digest"]}, {"plan", tracePreparationLoad}},
    [this, key, generation](const OlpClient::Reply &reply) {
      if (foregroundPlanning || tracePreparationEpoch != epoch || generation != tracePreparationGeneration || key != tracePreparationKey || key != core.call("traceKey").toString()) return;
      tracePreparation = nullptr;
      if (!reply.ok()) { tracePreparationFailed(reply); return; }
      tracePreparationError.clear(); tracePreparationRetryable = false;
      core.call("traceReady", {key}); tracePreparedKey = key;
      qInfo() << "Trace path prepared";
    }, 15000);
}

void Machine::beginTrace(int direction, double linearMM, double angularDeg) {
  if (hold.active || moving || refused() || !armHoldPermitted()) return;
  refresh();
  if (!lastView["jogDisabled"].toString().isEmpty()) {
    motionReason = lastView["jogDisabled"].toString();
    if (changed) changed();
    return;
  }
  QJsonObject request;
  try {
    if (tracePreparedKey != core.call("traceKey").toString())
      throw OlpError((tracePreparationError.isEmpty() ? QString("Trace is preparing this program; release the D-pad and press again when ready") : tracePreparationError).toStdString());
    request = core.call("traceRequest", {direction, linearMM, angularDeg}).toObject();
  } catch (const OlpError &e) {
    motionReason = e.what(); qInfo() << "Trace press refused:" << motionReason; if (status) status(motionReason, true); return;
  }
  const unsigned generation = hold.generation + 1;
  hold = Hold{true, false, -1, direction, generation, {}, "/trace", {}, {}};
  hold.trace = true; hold.traceTarget = request["target"].toString();
  qInfo() << "Trace press" << request;
  startTracePlan(generation, request, request["reverse"].toBool());
  if (changed) changed();
}

void Machine::startTracePlan(unsigned generation, const QJsonObject &load, bool reverse) {
  if (!hold.active || hold.generation != generation) return;
  if (!core.call("traceCurrent").toBool()) { endHold("program_changed"); return; }
  const QString cell = lastView["cell"].toString(), id = session();
  hold.pacer.pressSent();
  client.get(dense + "/trace/state", [this, generation, load, reverse, cell, id](const OlpClient::Reply &reply) {
    if (!hold.active || hold.generation != generation) return;
    if (!core.call("traceCurrent").toBool()) { endHold("program_changed"); return; }
    const auto state = reply.body.toObject();
    if (!reply.ok() || state["active"].toBool() || state["move"].toObject()["state"] == "moving") {
      motionReason = reply.ok() ? text("jog_mode_conflict") : text(reply.error, reply.detail);
      endHold();
      if (status) status(motionReason, true);
      return;
    }
    hold.pacer.allowPipelining(state["pipelined_intents"].toBool());
    hold.body = {{"target_id", cell}, {"session_id", id}, {"revision", state["revision"]},
      {"fraction", speed}, {"input_sequence", double(++hold.inputSequence)},
      {"trajectory_digest", load["trajectory_digest"]}, {"reverse", reverse},
      {"allow_entry", load["allow_entry"]}, {"endpoint_only", load["endpoint_only"]},
      {"linear_speed", load["linear_speed"]}, {"angular_speed", load["angular_speed"]},
      {"time_scale", load["time_scale"]}, {"first_segment", load["first_segment"]}, {"last_segment", load["last_segment"]}};
    QJsonObject begin = hold.body;
    holdTimer.setTimerType(Qt::PreciseTimer);
    holdTimer.start(20);
    motionReason.clear();
    client.post(dense + "/trace/begin", begin, [this, generation](const OlpClient::Reply &started) {
      if (!hold.active || hold.generation != generation) return;
      hold.pacer.pressAnswered();
      if (!started.ok()) {
        motionReason = text(started.error, started.detail);
        if (started.error.startsWith("trace_not_prepared")) {
          tracePreparedKey.clear(); tracePreparingKey.clear(); traceDebounce.restart();
        }
        core.call("traceForget");
        endHold("intent_refused:" + started.error);
        if (status) status(motionReason, true);
      } else traceCompleted(started.body.toObject());
      if (changed) changed();
    }, 12000);
  });
}

bool Machine::traceCompleted(const QJsonObject &state) {
  const auto end = state["last_end"].toObject();
  if (!hold.trace || end["cause"] != "trace_complete" || end["revision"] != hold.body["revision"]) return false;
  const QString target = hold.traceTarget;
  endHold("button_released");
  core.call("traceReached", {target});
  motionReason = "Waypoint reached — release the D-pad before tracing the next point";
  if (changed) changed();
  return true;
}
