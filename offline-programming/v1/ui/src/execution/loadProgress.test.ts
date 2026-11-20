import assert from "node:assert/strict";
import test from "node:test";

import {
  UPLOAD_ESTIMATE_BYTES_PER_S,
  expectedUploadSeconds,
  loadDoneText,
  loadFailedText,
  loadProgressFraction,
  loadProgressOverdue,
  loadProgressText,
} from "./loadProgress.ts";

// The LA bench program: 4.9 MB at the measured 0.8 MB/s is about six seconds.
test("the estimate is the size over the measured link rate, and the bar stops short of done", () => {
  assert.equal(UPLOAD_ESTIMATE_BYTES_PER_S, 800_000);
  assert.equal(expectedUploadSeconds(4_900_000), 6.125);
  assert.equal(loadProgressFraction(4_900_000, 0), 0);
  assert.ok(Math.abs(loadProgressFraction(4_900_000, 3.0625) - 0.5) < 1e-12);
  // Never 100% from the clock alone: the cell's answer finishes the bar.
  assert.equal(loadProgressFraction(4_900_000, 60), 0.96);
  assert.equal(loadProgressFraction(0, 10), 0);
  assert.equal(loadProgressFraction(4_900_000, Number.NaN), 0);
});

test("past the estimate the text says so and the bar is marked overdue", () => {
  assert.equal(loadProgressOverdue(4_900_000, 6), false);
  assert.equal(loadProgressOverdue(4_900_000, 6.2), true);
  assert.equal(loadProgressText(4_900_000, 2), "Uploading 4.9 MB to the robot · 2 s of about 6 s");
  assert.equal(
    loadProgressText(4_900_000, 20),
    "Uploading 4.9 MB to the robot · 20 s, past the 6 s estimate; the cell has not answered yet",
  );
  assert.equal(loadProgressText(0, 3), "Uploading to the robot · 3 s");
  assert.equal(loadDoneText(4_900_000, 5.87), "Loaded 4.9 MB in 5.9 s");
  assert.equal(loadFailedText(4_900_000, 1.04), "Load of 4.9 MB refused after 1.0 s");
});
