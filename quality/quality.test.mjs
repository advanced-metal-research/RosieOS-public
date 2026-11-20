import { test } from 'node:test';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { existsSync, mkdtempSync, writeFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';

const runner = fileURLToPath(new URL('./quality.mjs', import.meta.url));
// The public mirror has no daemon/v1, so there is no Rust tree to run Clippy on.
const rustTree = existsSync(fileURLToPath(new URL('../daemon/v1', import.meta.url)));
test('successful Clippy execution with warnings is not labeled passed', { skip: !rustTree && 'daemon/v1 is not in this tree' }, () => {
  const tools = mkdtempSync(join(tmpdir(), 'rosie-quality-tools-'));
  try {
    writeFileSync(join(tools, 'rustup'), `#!${process.execPath}\nif (process.argv.includes('clippy')) console.error('warning: existing lint finding');\n`, { mode: 0o755 });
    const result = spawnSync(process.execPath, [runner, 'rust'], {
      env: { ...process.env, PATH: tools }, encoding: 'utf8',
    });
    assert.equal(result.status, 0);
    assert.match(result.stderr, /warning: existing lint finding/);
    assert.match(result.stderr, /report_completed_not_qualified: .*clippy/);
    assert.doesNotMatch(result.stderr, /passed: .*clippy/);
  } finally {
    rmSync(tools, { recursive: true });
  }
});
test('missing Rust tooling is unavailable, never passed', () => {
  const result = spawnSync(process.execPath, [runner, 'rust'], {
    env: { ...process.env, PATH: '/nonexistent-quality-toolchain' }, encoding: 'utf8',
  });
  assert.equal(result.status, 1);
  assert.match(result.stderr, /capability_unavailable/);
  assert.doesNotMatch(result.stderr, /passed:/);
});
test('C++ requires a compile command for the selected translation unit', () => {
  const database = mkdtempSync(join(tmpdir(), 'rosie-quality-test-'));
  try {
    writeFileSync(join(database, 'compile_commands.json'), '[]');
    const source = fileURLToPath(new URL('../motion-server/joint-trajectory/v1/src/joint_trajectory_daemon.cpp', import.meta.url));
    const result = spawnSync(process.execPath, [runner, 'cpp', database, source], { encoding: 'utf8' });
    assert.equal(result.status, 1);
    assert.match(result.stderr, /no command for this translation unit/);
    assert.doesNotMatch(result.stderr, /passed:/);
  } finally {
    rmSync(database, { recursive: true });
  }
});
