#!/usr/bin/env node
// Explicit allowlists: never walk the historical repository for formatting.
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { existsSync, realpathSync, readFileSync } from 'node:fs';
import { resolve, relative } from 'node:path';

const root = fileURLToPath(new URL('../', import.meta.url));
const coordination = resolve(root, 'coordination/v1');
const binary = (name) => resolve(coordination, 'node_modules/.bin', name);
const sources = ['src', 'test', 'scripts', 'public/app.js'];
let failed = false;
function run(command, args, cwd = root, executionOnly = false) {
  const start = performance.now();
  const result = spawnSync(command, args, { cwd, stdio: 'inherit' });
  const status = result.error ? 'capability_unavailable' : result.status !== 0 ? 'rejected' : executionOnly ? 'report_completed_not_qualified' : 'passed';
  console.error(`${status}: ${command} ${args.join(' ')} (${((performance.now() - start) / 1000).toFixed(2)}s)`);
  if (result.error) console.error(result.error.message);
  if (result.error || result.status !== 0) failed = true;
}
function npm(...args) { run('npm', ['run', ...args], coordination); }
const [command = 'check', ...extra] = process.argv.slice(2);
switch (command) {
  case 'format':
    run(binary('oxfmt'), ['--write', ...sources], coordination);
    break;
  case 'format-check':
    run(binary('oxfmt'), ['--check', ...sources], coordination);
    break;
  case 'lint':
    npm('lint');
    run(binary('ast-grep'), ['test', '-c', 'quality/sgconfig.yml', '--skip-snapshot-tests']);
    run(binary('ast-grep'), ['scan', '-c', 'quality/sgconfig.yml', 'coordination/v1/src']);
    break;
  case 'typecheck':
    npm('types');
    npm('typecheck');
    break;
  case 'check':
    run(process.execPath, ['--test', 'quality/quality.test.mjs']);
    for (const step of ['format-check', 'lint', 'typecheck']) run(process.execPath, [resolve(root, 'quality/quality.mjs'), step]);
    npm('test');
    npm('test:schema');
    break;
  case 'python':
    for (const args of [['format', '--check'], ['check']]) run('uvx', ['--from', 'ruff==0.15.6', 'ruff', ...args, 'python', 'tests', 'tools'], resolve(root, 'weld_planner/v1'));
    break;
  case 'rust':
    // The public mirror leaves out daemon/v1, the only Rust component: say so
    // rather than report a checked tree.
    if (!existsSync(resolve(root, 'daemon/v1'))) {
      console.error('capability_unavailable: daemon/v1 is not in this tree');
      failed = true;
      break;
    }
    run('rustup', ['run', '1.95.0', 'cargo', 'fmt', '--check'], resolve(root, 'daemon/v1'));
    // Clippy warnings do not change cargo's exit code. Completion is not qualification.
    run('rustup', ['run', '1.95.0', 'cargo', 'clippy', '--locked', '--all-targets'], resolve(root, 'daemon/v1'), true);
    break;
  case 'go': {
    // gofmt -l alone exits zero on differences; reject a nonempty listing.
    const result = spawnSync('git', ['ls-files', 'steamdeck/real/v4/*.go'], { cwd: root, encoding: 'utf8' });
    if (result.status !== 0) throw new Error('Cannot enumerate tracked Go sources');
    const files = result.stdout.trim().split('\n').filter(Boolean);
    const formatted = spawnSync('gofmt', ['-l', ...files], { cwd: root, encoding: 'utf8' });
    process.stdout.write(formatted.stdout ?? '');
    if (formatted.error || formatted.status !== 0 || formatted.stdout.trim()) failed = true;
    console.error(`${failed ? 'rejected' : 'passed'}: gofmt tracked steamdeck/real/v4 sources`);
    run('go', ['run', 'github.com/golangci/golangci-lint/v2/cmd/golangci-lint@v2.13.2', 'run', '--config', resolve(root, 'quality/golangci.yml')], resolve(root, 'steamdeck/real/v4'));
    break;
  }
  case 'workflows':
    run('go', ['run', 'github.com/rhysd/actionlint/cmd/actionlint@v1.7.12', '-shellcheck=']);
    break;
  case 'shell':
    run('shellcheck', ['--version']);
    run('shellcheck', ['dev-stack.sh']);
    break;
  case 'cpp': {
    const [database, source] = extra;
    if (!database || !source || !existsSync(resolve(database, 'compile_commands.json'))) throw new Error('cpp requires an existing compile database directory and one motion-server/joint-trajectory/v1/src C++ source');
    const file = realpathSync(source);
    const local = relative(resolve(root, 'motion-server/joint-trajectory/v1/src'), file);
    if (local.startsWith('..') || !local.endsWith('.cpp')) throw new Error('C++ translation unit outside owned pilot');
    const entries = JSON.parse(readFileSync(resolve(database, 'compile_commands.json'), 'utf8'));
    if (!Array.isArray(entries) || !entries.some((entry) =>
      typeof entry.directory === 'string' && typeof entry.file === 'string' &&
      resolve(entry.directory, entry.file) === file &&
      (typeof entry.command === 'string' || Array.isArray(entry.arguments)))) {
      throw new Error('Compile database has no command for this translation unit');
    }
    run('clang-format', ['--version']);
    run('clang-tidy', ['--version']);
    run('clang-format', ['--dry-run', '--Werror', `--style=file:${resolve(root, 'quality/clang-format.yml')}`, file]);
    run('clang-tidy', [`--config-file=${resolve(root, 'quality/clang-tidy.yml')}`, '-p', resolve(database), file]);
    break;
  }
  default:
    throw new Error('Usage: node quality/quality.mjs format|format-check|lint|typecheck|check|python|rust|go|workflows|shell|cpp <compile-db-dir> <source>');
}
process.exitCode = failed ? 1 : 0;
