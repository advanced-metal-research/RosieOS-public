import assert from "node:assert/strict";
import test from "node:test";
import { seedRobotCatalogue } from "../../test/support/robot-catalogue.ts";
import { pinRobotDescription } from "../domain/project.ts";
import { createBlankSpreadsheetProject } from "../domain/spreadsheet.ts";
import { CaptureProgramMismatch, captureDescription, programModelMismatch } from "./capture.ts";

seedRobotCatalogue();

// A program written for one robot model is refused on another cell's robot
// in words that say which two models, and what to do: another program.
test("a capture on another model's cell names both models and the remedy", () => {
  assert.equal(programModelMismatch("rosie_1420_v1", "rosie_1420_v1"), "");
  assert.equal(programModelMismatch("rosie_1400_v3", ""), "", "no cell, no mismatch");
  const project = pinRobotDescription(createBlankSpreadsheetProject({ name: "Deck teach", robotModel: "rosie_1400_v3", railsAuthoring: true }));
  const snapshot = { modelId: "rosie_1420_v1" } as Parameters<typeof captureDescription>[2];
  assert.throws(() => captureDescription(project, null, snapshot),
    { message: "This program is for rosie_1400_v3; the connected cell is rosie_1420_v1. New starts a program for it, Open picks another" });
  const own = { modelId: "rosie_1400_v3" } as Parameters<typeof captureDescription>[2];
  assert.equal(captureDescription(project, null, own), project.robot.description?.robotDescriptionSha256);
});

// The desktop offers the remedy in a dialog keyed on why the capture was
// refused, so the refusal has to say which of the two it is.
test("a refused capture says whether the model or the description differs", () => {
  const project = pinRobotDescription(createBlankSpreadsheetProject({ name: "Old program", robotModel: "rosie_1400_v3", railsAuthoring: true }));
  const other = { modelId: "rosie_1420_v1" } as Parameters<typeof captureDescription>[2];
  assert.throws(() => captureDescription(project, null, other),
    (error: unknown) => error instanceof CaptureProgramMismatch && error.kind === "model");
  const own = { modelId: "rosie_1400_v3" } as Parameters<typeof captureDescription>[2];
  const measured = { pose: own, cellId: "cell", settings: { robotDescriptionSha256: "sha256:not-the-programs" } } as unknown as Parameters<typeof captureDescription>[1];
  assert.throws(() => captureDescription(project, measured, own),
    (error: unknown) => error instanceof CaptureProgramMismatch && error.kind === "description"
      && error.message === "Adopt the matching robot description before recording a waypoint");
});
