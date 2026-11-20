import { test } from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync, mkdirSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';

import {
  formatReleaseTestCoverage,
  loadReleaseTestCoverage,
  validateReleaseTestCoverage,
} from './release-test-coverage.mjs';

const contract = loadReleaseTestCoverage();

test('the checked-in release coverage contract matches the managed gate catalogue', () => {
  assert.doesNotThrow(() => validateReleaseTestCoverage(contract));
  assert.equal(contract.suites.length, 12);
});

test('the report shows every suite has an implementation owner', () => {
  const report = formatReleaseTestCoverage(contract);

  assert.match(report, /12 suites: 12 covered, 0 partial, and 0 unmapped/);
  assert.match(report, /`rtctl` \| `build1_candidate` \| `covered`/);
  assert.match(report, /`cells` \| `pull_request` \| `covered`/);
});

test('a missing managed gate suite fails closed', () => {
  const invalid = structuredClone(contract);
  invalid.suites = invalid.suites.filter((suite) => suite.id !== 'cells');

  assert.throws(
    () => validateReleaseTestCoverage(invalid),
    /suite catalogue mismatch: missing=\[cells\]/,
  );
});

test('an unknown proof owner fails closed', () => {
  const invalid = structuredClone(contract);
  invalid.suites[0].owner = 'developer_laptop';

  assert.throws(
    () => validateReleaseTestCoverage(invalid),
    /invalid owner developer_laptop/,
  );
});

test('a covered suite must name existing evidence', () => {
  const invalid = structuredClone(contract);
  invalid.suites[0].evidence = [];

  assert.throws(
    () => validateReleaseTestCoverage(invalid),
    /covered suite make_test must name evidence/,
  );
});

test('a stale workflow job reference fails closed', () => {
  const root = mkdtempSync(join(tmpdir(), 'release-test-coverage-'));
  mkdirSync(join(root, 'catalog'), { recursive: true });
  mkdirSync(join(root, '.github', 'workflows'), { recursive: true });
  writeFileSync(join(root, 'catalog', 'gate.go'), 'var gateSuites = []gateSuite{{Name: "one"}}\n');
  writeFileSync(join(root, '.github', 'workflows', 'proof.yml'), 'jobs:\n  actual:\n    steps:\n      - run: proof command\n');
  const isolated = {
    schema: 'rosie_release_test_coverage_v1',
    catalog: { source: 'catalog/gate.go', collection: 'gateSuites' },
    suites: [{
      id: 'one',
      owner: 'pull_request',
      coverage: 'covered',
      evidence: [{ workflow: '.github/workflows/proof.yml', job: 'stale', command: 'proof command' }],
    }],
  };

  assert.throws(
    () => validateReleaseTestCoverage(isolated, root),
    /workflow has no job stale/,
  );
});

test('a stale fixed-worker source reference fails closed', () => {
  const root = mkdtempSync(join(tmpdir(), 'release-test-coverage-'));
  mkdirSync(join(root, 'catalog'), { recursive: true });
  mkdirSync(join(root, 'worker'), { recursive: true });
  writeFileSync(join(root, 'catalog', 'gate.go'), 'var gateSuites = []gateSuite{{Name: "one"}}\n');
  writeFileSync(join(root, 'worker', 'gates.py'), 'proof command\n');
  const isolated = {
    schema: 'rosie_release_test_coverage_v1',
    catalog: { source: 'catalog/gate.go', collection: 'gateSuites' },
    suites: [{
      id: 'one',
      owner: 'build1_candidate',
      coverage: 'covered',
      evidence: [{ source: 'worker/gates.py', command: 'missing command' }],
    }],
  };

  assert.throws(
    () => validateReleaseTestCoverage(isolated, root),
    /source does not contain command: missing command/,
  );
});

test('a tree without the build worker skips only its proof commands', () => {
  const root = mkdtempSync(join(tmpdir(), 'release-test-coverage-'));
  mkdirSync(join(root, 'catalog'), { recursive: true });
  writeFileSync(join(root, 'catalog', 'gate.go'), 'var gateSuites = []gateSuite{{Name: "one"}, {Name: "two"}}\n');
  const suite = (id, owner, source) => ({ id, owner, coverage: 'covered', evidence: [{ source, command: 'proof' }] });
  const isolated = {
    schema: 'rosie_release_test_coverage_v1',
    catalog: { source: 'catalog/gate.go', collection: 'gateSuites' },
    suites: [suite('one', 'build1_candidate', 'tools/build1-release-worker/gates.py'), suite('two', 'pull_request', 'missing.py')],
  };

  assert.throws(() => validateReleaseTestCoverage(isolated, root), /suite two source does not exist: missing\.py/);
  isolated.suites[1] = suite('two', 'pull_request', 'catalog/gate.go');
  writeFileSync(join(root, 'catalog', 'gate.go'), 'var gateSuites = []gateSuite{{Name: "one"}, {Name: "two"}} // proof\n');
  assert.doesNotThrow(() => validateReleaseTestCoverage(isolated, root));
  mkdirSync(join(root, 'tools', 'build1-release-worker'), { recursive: true });
  assert.throws(() => validateReleaseTestCoverage(isolated, root), /suite one source does not exist/);
});
