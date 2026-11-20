import { test } from 'node:test';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';

import {
  admissionExitCode,
  createAdmissionPlan,
  formatAdmissionSummary,
  loadAdmissionPolicy,
  resolveGitCommit,
  validateAdmissionPolicy,
} from './pr-admission.mjs';

const policyPath = fileURLToPath(new URL('./pr-admission-policy.json', import.meta.url));
const policy = loadAdmissionPolicy(policyPath);

test('the checked-in policy satisfies the admission contract', () => {
  assert.doesNotThrow(() => validateAdmissionPolicy(policy));
});

test('gate changes select the contract and workflow checks', () => {
  const plan = createAdmissionPlan(policy, [
    'quality/pr-admission.mjs',
    '.github/workflows/pr-admission.yml',
  ]);

  assert.deepEqual(plan.checks, [
    'diff-integrity',
    'admission-contract',
    'workflow-syntax',
  ]);
  assert.deepEqual(plan.pendingProofPaths, []);
  assert.deepEqual(plan.unknownPaths, []);
  assert.equal(plan.wouldBlockBeforeChecks, false);
});

test('build worker changes select the fixed worker contract', () => {
  const plan = createAdmissionPlan(policy, [
    'tools/build1-release-worker/rosie_build1_worker/core.py',
  ]);

  assert.deepEqual(plan.checks, [
    'diff-integrity',
    'admission-contract',
    'build1-contract',
  ]);
  assert.deepEqual(plan.matchedRules, ['build1-release-worker']);
  assert.equal(plan.wouldBlockBeforeChecks, false);
});

test('documentation under a runtime directory uses the documentation rule', () => {
  const plan = createAdmissionPlan(policy, ['rt-core/docs/ARCHITECTURE.md']);

  assert.deepEqual(plan.matchedRules, ['documentation']);
  assert.deepEqual(plan.pendingProofPaths, []);
  assert.equal(plan.wouldBlockBeforeChecks, false);
});

test('runtime changes remain visible until their proof mapping exists', () => {
  const plan = createAdmissionPlan(policy, ['rt-core/engine/cycle_machine.hpp']);

  assert.deepEqual(plan.matchedRules, ['runtime-control']);
  assert.deepEqual(plan.pendingProofPaths, ['rt-core/engine/cycle_machine.hpp']);
  assert.equal(plan.wouldBlockBeforeChecks, true);
});

test('retired trees and repository tooling stay pending, not unknown', () => {
  const pending = [
    'robot/v3/rtcore/main.go',
    'urdf/v4/rosie_1400_h/robot.urdf',
    'nats/v1/cli.go',
    'docs/evidence/nix-host-checks-36231ad9.log',
    'steamdeck/virtual/bridge/main.go',
    '.gitignore',
    'tools/public-mirror/export.mjs',
    'quality/leak-check.mjs',
    'cadquery/v1/pixi.toml',
    'cell-camera/v1/host/example.env',
    'dev-stack.sh',
    'tools/sphere_tool/src/sphere-fit.ts',
    'releases/robot-stack/v0.1.0/BUILD-OBSERVATIONS.txt',
  ];
  const plan = createAdmissionPlan(policy, [...pending, 'LICENSE', 'NOTICE']);

  assert.deepEqual(plan.unknownPaths, []);
  assert.deepEqual(plan.pendingProofPaths, [...pending].sort());
  assert.ok(plan.matchedRules.includes('retired-trees'));
  assert.ok(plan.matchedRules.includes('repository-notices'));
  assert.equal(plan.wouldBlockBeforeChecks, true);
});

test('unknown paths fail closed in the admission decision', () => {
  const plan = createAdmissionPlan(policy, ['new-subsystem/main.go']);

  assert.deepEqual(plan.unknownPaths, ['new-subsystem/main.go']);
  assert.equal(plan.wouldBlockBeforeChecks, true);
});

test('policy commands cannot come from policy data', () => {
  const invalid = structuredClone(policy);
  invalid.always_checks.push('shell-from-policy');

  assert.throws(
    () => validateAdmissionPolicy(invalid),
    /unknown check shell-from-policy/,
  );
});

test('Git revisions cannot inject command options', () => {
  assert.throws(
    () => resolveGitCommit('--output=/tmp/result'),
    /unsafe Git revision/,
  );
});

test('shadow reports a blocking verdict without blocking the pull request', () => {
  assert.equal(admissionExitCode('shadow', true), 0);
  assert.equal(admissionExitCode('enforce', true), 1);
  assert.equal(admissionExitCode('enforce', false), 0);
});

test('summary names pending proof and unknown paths', () => {
  const plan = createAdmissionPlan(policy, [
    'rt-core/engine/cycle_machine.hpp',
    'new-subsystem/main.go',
  ]);
  const summary = formatAdmissionSummary({
    mode: 'shadow',
    plan,
    results: [],
    wouldBlock: true,
  });

  assert.match(summary, /Shadow verdict: would block/);
  assert.match(summary, /rt-core\/engine\/cycle_machine\.hpp/);
  assert.match(summary, /new-subsystem\/main\.go/);
});
