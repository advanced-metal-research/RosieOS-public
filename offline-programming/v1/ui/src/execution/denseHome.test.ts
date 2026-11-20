import assert from "node:assert/strict";
import test from "node:test";
import { isMappedAxis, unhomedMappedAxes, type DenseAxisStatus } from "./denseClient.ts";

function axis(index: number, homed: boolean | null): DenseAxisStatus {
  return {
    index,
    qRad: 0,
    qCmdRad: null,
    posCounts: 0,
    homeCounts: 0,
    statusword: 0,
    operational: false,
    fault: false,
    error: false,
    homed,
  } as DenseAxisStatus;
}

// rosie_1420_v1 on bench-relay (2026-09-13): mapped 0x3f, J1–J6 homed, J7–J9
// unmapped and reported unhomed. The daemon's go_home only refuses for a
// mapped unhomed axis, so the panel must count the same way.
const sixAxisCell = [
  ...[0, 1, 2, 3, 4, 5].map((i) => axis(i, true)),
  ...[6, 7, 8].map((i) => axis(i, false)),
];

test("unmapped unhomed axes do not hold GO HOME back", () => {
  assert.equal(unhomedMappedAxes(sixAxisCell, 0x3f).length, 0);
});

test("a mapped unhomed axis still counts", () => {
  const cell = sixAxisCell.map((a) => (a.index === 3 ? axis(3, false) : a));
  assert.deepEqual(unhomedMappedAxes(cell, 0x3f).map((a) => a.index), [3]);
});

test("without a mask every axis counts, so an unknown authority fails closed", () => {
  assert.equal(unhomedMappedAxes(sixAxisCell, null).length, 3);
  assert.equal(unhomedMappedAxes(sixAxisCell, 0).length, 3);
  assert.equal(unhomedMappedAxes(null, 0x3f).length, 0);
});

test("isMappedAxis reads the bit for the axis index", () => {
  assert.equal(isMappedAxis(0x3f, 5), true);
  assert.equal(isMappedAxis(0x3f, 6), false);
  assert.equal(isMappedAxis(0x1ff, 8), true);
  assert.equal(isMappedAxis(null, 8), true);
});
