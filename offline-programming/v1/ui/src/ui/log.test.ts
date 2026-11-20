import assert from "node:assert/strict";
import test from "node:test";

import { LogBus, describe, levelOfStatus } from "./log.ts";

test("the bar shows the worst unseen level and clears when the panel is opened", () => {
  const bus = new LogBus();
  const rails = bus.logger("rails");
  assert.equal(bus.unseenWorst(), null);
  rails.info("26 seams over 5 joints");
  assert.equal(bus.unseenWorst(), null, "information never colours the bar");
  rails.warn("a seam has no reachable stretch");
  assert.equal(bus.unseenWorst(), "warn");
  rails.error("the weld planner refused the request");
  assert.equal(bus.unseenWorst(), "error");
  assert.equal(bus.unseenCount("warn"), 2);
  bus.markSeen();
  assert.equal(bus.unseenWorst(), null);
  assert.equal(bus.unseenCount("warn"), 0);
  rails.warn("again");
  assert.equal(bus.unseenWorst(), "warn", "a new warning after seeing counts again");
});

test("entries carry time, level, module and message; the ring keeps the newest", () => {
  const bus = new LogBus(3);
  const before = Date.now();
  bus.logger("cad").info("one");
  bus.logger("cad").info("two");
  bus.logger("viewer").warn("three");
  bus.logger("viewer").error("four", new Error("boom"));
  assert.deepEqual(bus.entries.map((entry) => entry.message), ["two", "three", "four"]);
  const last = bus.entries.at(-1)!;
  assert.equal(last.module, "viewer");
  assert.equal(last.level, "error");
  assert.ok(last.timeMs >= before);
  assert.match(last.detail ?? "", /boom/);
});

test("a status line's wording picks its level", () => {
  assert.equal(levelOfStatus("Autosave blocked: quota"), "warn");
  assert.equal(levelOfStatus("Delete blocked: node not found"), "warn");
  assert.equal(levelOfStatus("Rails welds planned"), "info");
  assert.equal(describe({ a: 1 }), "{\n \"a\": 1\n}");
});
