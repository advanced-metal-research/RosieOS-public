import { test } from "node:test";
import assert from "node:assert/strict";
import { inventory, difference, report } from "./deployment-surface.mjs";

test("surface reports additions and removals without interpreting them as approval", () => {
  const before = inventory({ nodes: [{ runtime_paths: ["/old"], daemon_environment: { DAEMON_V1_DEPLOY_ALLOWED_ROOTS: "/a" } }] });
  const after = inventory({ nodes: [{ runtime_paths: ["/new"], daemon_environment: { DAEMON_V1_DEPLOY_ALLOWED_ROOTS: "/a:/b" } }] });
  const delta = difference(before, after);
  assert.equal(delta.added.length, 2);
  assert.equal(delta.removed.length, 2);
  assert.ok(delta.added.some(x => x.includes("/new")));
  assert.deepEqual(difference(after, after), { added: [], removed: [] });
  assert.deepEqual(report("HEAD", "HEAD").changes, []);
});

test("surface tracks build commands and deployment endpoints", () => {
  const keys = ["request_endpoint", "cache_probe_endpoint", "blob_probe_endpoint", "blob_ticket_endpoint", "blob_upload_endpoint", "publish_endpoint", "record_endpoint"];
  for (const key of keys) {
    const before = inventory({ deployment: { [key]: ["old"] } });
    const after = inventory({ deployment: { [key]: ["new"] } });
    assert.deepEqual(difference(before, after), {
      added: [`$.deployment.${key}: "new"`],
      removed: [`$.deployment.${key}: "old"`],
    }, key);
  }
  const before = inventory({ artifact_build_commands: { linux: "make old" } });
  const after = inventory({ artifact_build_commands: { linux: "make new" } });
  assert.deepEqual(difference(before, after), {
    added: ['$.artifact_build_commands: ["linux","make new"]'],
    removed: ['$.artifact_build_commands: ["linux","make old"]'],
  });
});
