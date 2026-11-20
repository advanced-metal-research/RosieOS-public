import assert from "node:assert/strict";
import test from "node:test";
import {
  CARTESIAN_JOG_DISABLED_TITLE,
  CARTESIAN_JOG_DISARMED_TITLE,
  CARTESIAN_JOG_ENABLED_TITLE,
  LIVE_JOG_API,
  JOG_PAD_STORAGE_KEY,
  ROTARY_JOG_SCALE,
  TESSERACT_PLAN_API,
  cartesianJogView,
  jogButtonAxes,
  jogSessionIsLive,
  jogUsesMotionServerNotTesseract,
  liveJogUrl,
  nextJogRetryDelayMs,
  parseJogResponse,
  readJogPadVisible,
  shouldKeepTryingJogStart,
  writeJogPadVisible,
} from "./live-jog.ts";

test("jog pad visibility defaults off and persists only an explicit true", () => {
  const entries = new Map<string, string>();
  const storage = () => ({
    getItem: (key: string) => entries.get(key) ?? null,
    setItem: (key: string, value: string) => { entries.set(key, value); },
  });
  assert.equal(JOG_PAD_STORAGE_KEY, "rosie.offline-programming.v1.jog-pad-visible");
  assert.equal(readJogPadVisible(storage), false);
  writeJogPadVisible(true, storage);
  assert.equal(readJogPadVisible(storage), true);
  writeJogPadVisible(false, storage);
  assert.equal(readJogPadVisible(storage), false);
  entries.set(JOG_PAD_STORAGE_KEY, "invalid");
  assert.equal(readJogPadVisible(storage), false);
});

test("jog pad storage access and read/write failures do not break rendering", () => {
  const unavailable = () => { throw new Error("storage denied"); };
  const throwingStorage = () => ({ getItem: unavailable, setItem: unavailable });
  assert.equal(readJogPadVisible(unavailable), false);
  assert.equal(readJogPadVisible(throwingStorage), false);
  assert.doesNotThrow(() => writeJogPadVisible(true, unavailable));
  assert.doesNotThrow(() => writeJogPadVisible(false, throwingStorage));
});

test("cartesian jog HTTP path is Motion Server jog, not Tesseract plans", () => {
  assert.equal(liveJogUrl("start"), "/api/offline-programming/v1/local-simulator/jog/start");
  assert.equal(liveJogUrl("intent"), "/api/offline-programming/v1/local-simulator/jog/intent");
  assert.equal(liveJogUrl("state"), "/api/offline-programming/v1/local-simulator/jog/state");
  assert.equal(liveJogUrl("halt"), "/api/offline-programming/v1/local-simulator/jog/halt");
  assert.equal(jogUsesMotionServerNotTesseract(), true);
  assert.equal(jogUsesMotionServerNotTesseract(LIVE_JOG_API), true);
  assert.equal(jogUsesMotionServerNotTesseract(TESSERACT_PLAN_API), false);
  assert.doesNotMatch(LIVE_JOG_API, /tesseract/i);
  assert.notEqual(LIVE_JOG_API, TESSERACT_PLAN_API);
});

test("BASE/TOOL buttons enable only when a jog session is running AND armed", () => {
  const live = parseJogResponse({
    ok: true,
    state: { running: true, daemonAddress: "127.0.0.1:9" },
  });
  assert.equal(jogSessionIsLive(live), true);
  const enabled = cartesianJogView(true, true);
  assert.equal(enabled.frameDisabled, false);
  assert.equal(enabled.gridDisabled, false);
  assert.equal(enabled.buttonDisabled, false);
  assert.equal(enabled.title, CARTESIAN_JOG_ENABLED_TITLE);

  const idle = parseJogResponse({
    ok: true,
    state: { running: false },
  });
  assert.equal(jogSessionIsLive(idle), false);
  const disabled = cartesianJogView(false, true, idle.error);
  assert.equal(disabled.buttonDisabled, true);
  assert.equal(disabled.title, CARTESIAN_JOG_DISABLED_TITLE);
});

test("a live jog session with DISARMED drives never enables the jog buttons", () => {
  const view = cartesianJogView(true, false);
  assert.equal(view.live, true);
  assert.equal(view.armed, false);
  assert.equal(view.frameDisabled, true);
  assert.equal(view.gridDisabled, true);
  assert.equal(view.buttonDisabled, true);
  assert.equal(view.title, CARTESIAN_JOG_DISARMED_TITLE);
  // The title names the control that lifts the gate: the Jog panel's own
  // Arm preview toggle, never the machine's Arm.
  assert.match(view.title, /Arm preview/);
  assert.match(view.title, /Machine panel/);

  const dead = cartesianJogView(false, false);
  assert.equal(dead.buttonDisabled, true);
});

test("failed jog/start keeps buttons disabled and surfaces the Motion Server error", () => {
  const failed = parseJogResponse({
    ok: false,
    error: "jog runtime unavailable: robot-v4-cartesiand binary is not built",
  });
  assert.equal(jogSessionIsLive(failed), false);
  const view = cartesianJogView(false, true, failed.error);
  assert.equal(view.live, false);
  assert.equal(view.buttonDisabled, true);
  assert.match(view.title, /robot-v4-cartesiand/);
  assert.doesNotMatch(view.title, /tesseract/i);
});

test("server jog refusal uses the error string as the disabled title without a typed code", () => {
  for (const error of [
    "jog runtime is not configured",
    "Local simulator jog is disabled for remote execution",
  ]) {
    const response = parseJogResponse({ ok: false, error });
    assert.equal(jogSessionIsLive(response), false);
    for (const armed of [false, true]) {
      const view = cartesianJogView(jogSessionIsLive(response), armed, response.error);
      assert.equal(view.frameDisabled, true);
      assert.equal(view.gridDisabled, true);
      assert.equal(view.buttonDisabled, true);
      assert.equal(view.title, error);
    }
  }
});

test("jog state errors surface and missing details retain the disabled fallback", () => {
  const response = parseJogResponse({ ok: false, state: { running: false, error: "simulator unavailable" } });
  assert.equal(cartesianJogView(false, true, response.error).title, "simulator unavailable");
  assert.equal(cartesianJogView(false, true, "  ").title, CARTESIAN_JOG_DISABLED_TITLE);
});

test("jog button axes: linear full stick, rotary scaled for 10 Hz tool-frame staleness", () => {
  // Axes vector layout is [X, Y, Z, RX, RY, RZ] — the pendant packet order.
  assert.deepEqual(jogButtonAxes([1, 0, 0, 0, 0, 0]), [1, 0, 0, 0, 0, 0]);
  assert.deepEqual(jogButtonAxes([0, -1, 0, 0, 0, 0]), [0, -1, 0, 0, 0, 0]);
  assert.deepEqual(jogButtonAxes([0, 0, 1, 0, 0, 0]), [0, 0, 1, 0, 0, 0]);
  // Full rotary stick is 180°/s at the daemon (MAX_JOG_ANGULAR_RAD_S = π):
  // it outruns the UI's per-send tool-orientation rotation and drags the TCP.
  assert.deepEqual(jogButtonAxes([0, 0, 0, 1, 0, 0]), [0, 0, 0, ROTARY_JOG_SCALE, 0, 0]);
  assert.deepEqual(jogButtonAxes([0, 0, 0, 0, -1, 0]), [0, 0, 0, 0, -ROTARY_JOG_SCALE, 0]);
  assert.deepEqual(jogButtonAxes([0, 0, 0, 0, 0, -1]), [0, 0, 0, 0, 0, -ROTARY_JOG_SCALE]);
  assert.ok(ROTARY_JOG_SCALE > 0 && ROTARY_JOG_SCALE < 1);
  // The Go serve rejects axes outside [-1, 1]; scaling must never widen them.
  for (const value of jogButtonAxes([1, -1, 1, -1, 1, -1])) {
    assert.ok(value >= -1 && value <= 1);
  }
});

test("boot start retries while the isolated cartesiand session is not live", () => {
  assert.equal(shouldKeepTryingJogStart(false, 0), true);
  assert.equal(shouldKeepTryingJogStart(false, 7), true);
  assert.equal(shouldKeepTryingJogStart(false, 8), false);
  assert.equal(shouldKeepTryingJogStart(true, 0), false);
  assert.equal(nextJogRetryDelayMs(0), 400);
  assert.equal(nextJogRetryDelayMs(6), 5000);
  assert.equal(nextJogRetryDelayMs(99), 5000);
});
