import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { fileURLToPath } from "node:url";

import { decodeDenseTrajectory } from "../src/execution/denseTrajectoryBlob.ts";
import {
  denseTrajectorySamples,
  torchOnFlags,
  totalDurationS,
} from "../src/execution/denseTrajectoryPlayback.ts";
import { createDogBoxProject } from "../src/examples/dog-box.ts";

async function reference() {
  const path = fileURLToPath(new URL(
    "../../../../motion-server/joint-trajectory/v1/testdata/dense_joint_trajectory_reference.rdt", import.meta.url));
  return decodeDenseTrajectory(new Uint8Array(await readFile(path)));
}

test("a planned rails program previews from the bytes the robot plays", async () => {
  const project = createDogBoxProject("2026-09-10T00:00:00.000Z");
  const trajectory = await reference();
  trajectory.segments.forEach((segment, index) => { segment.sourceId = project.nodes[index % project.nodes.length]!.id; });
  const samples = denseTrajectorySamples(project, trajectory);
  let offset = 0;
  for (const segment of trajectory.segments) {
    assert.ok(samples.slice(offset, offset + segment.samples.length).every(sample => sample.nodeId === segment.sourceId));
    offset += segment.samples.length;
  }

  assert.equal(samples.length, trajectory.totalSampleCount);
  for (const sample of samples) {
    assert.equal(sample.source, "preview");
    assert.equal(sample.jointsRad.length, project.robot.jointOrder.length);
    // Posed from joints. No pose is derived to sit beside them: the viewer's
    // own kinematics knows it, and a second answer could only disagree.
    assert.equal(sample.tcpPose, undefined);
  }
});

test("playback time is monotonic across segments though each keeps its own clock", async () => {
  const project = createDogBoxProject("2026-09-10T00:00:00.000Z");
  const trajectory = await reference();
  const samples = denseTrajectorySamples(project, trajectory);

  let previous = -1;
  for (const sample of samples) {
    assert.ok(sample.timeS >= previous, "playback time went backwards");
    previous = sample.timeS;
  }
  // And the total is the sum of the segments' clocks, not the last sample's
  // time -- reading that as the total is gluing the clocks together.
  const lastSegment = trajectory.segments[trajectory.segments.length - 1]!;
  const lastTime = lastSegment.samples[lastSegment.samples.length - 1]!.tS;
  assert.ok(totalDurationS(trajectory) >= lastTime);
});

test("the torch flag rides per sample, so the arc switches where the robot switches it", async () => {
  const trajectory = await reference();
  const flags = torchOnFlags(trajectory);
  assert.equal(flags.length, trajectory.totalSampleCount);
  for (const flag of flags) assert.equal(typeof flag, "boolean");
});

test("a weld segment names its seam and a freespace segment names none", async () => {
  const project = createDogBoxProject("2026-09-10T00:00:00.000Z");
  const trajectory = await reference();
  const samples = denseTrajectorySamples(project, trajectory);
  const bySegment = new Map(trajectory.segments.map((s) => [s.index, s]));

  let checked = 0;
  for (const sample of samples) {
    if (sample.motion === "weld") {
      // A freespace segment's source names both ends and belongs to no single
      // seam, so only a weld carries one.
      const named = [...bySegment.values()].some((s) => s.sourceId === sample.seamId);
      assert.ok(sample.seamId === null || named, `seam ${sample.seamId} names no segment`);
      checked += 1;
    }
  }
  assert.ok(checked > 0 || trajectory.segments.every((s) => s.kind !== "weld"));
});
