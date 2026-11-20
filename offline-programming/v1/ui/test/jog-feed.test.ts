import assert from "node:assert/strict";
import test from "node:test";

import { createJogFeedGate } from "../src/viewer/jog-feed.ts";

const RESET = [0, 0, 0, 0, 0, 0, 0, 0, 0];

test("the first jog pose is shown", () => {
  const gate = createJogFeedGate();
  assert.equal(gate.admit(RESET, false), true);
});

test("an idle jog session does not rewrite the pose on every poll", () => {
  // The flashing replay: the session sat at the reset pose and was written
  // ten times a second over the preview's own samples.
  const gate = createJogFeedGate();
  gate.admit(RESET, false);
  for (let poll = 0; poll < 10; poll += 1) assert.equal(gate.admit(RESET, false), false);
});

test("a playing preview owns the pose even if the jog pose moves", () => {
  const gate = createJogFeedGate();
  gate.admit(RESET, false);
  assert.equal(gate.admit([0.1, ...RESET.slice(1)], true), false);
});

test("a paused preview keeps its pose while the jogged robot stays put", () => {
  const gate = createJogFeedGate();
  gate.admit(RESET, false);
  gate.admit(RESET, true); // playing
  assert.equal(gate.admit(RESET, false), false, "pausing snapped the viewer back to the jog pose");
});

test("a jog that moves is shown at once", () => {
  const gate = createJogFeedGate();
  gate.admit(RESET, false);
  assert.equal(gate.admit([0.01, ...RESET.slice(1)], false), true);
});

test("forget shows the jogged robot again on the next poll", () => {
  const gate = createJogFeedGate();
  gate.admit(RESET, false);
  gate.forget();
  assert.equal(gate.admit(RESET, false), true);
});
