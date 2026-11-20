#!/usr/bin/env node
// Builds the public mirror's tree from one commit of this repository: the
// committed files (never the working tree), minus private-paths.txt, then the
// leak check over the result. The mirror is published from this tree as a
// single squashed commit, so nothing in this repository's history -- deleted
// files, author emails, commit trailers, PRs -- reaches it.
//
//   node tools/public-mirror/export.mjs <empty-out-dir> [--rev <commit>] [--allowlisted]
//
// Without --allowlisted the leak check is strict: any hit fails the export.
import { spawn, spawnSync } from 'node:child_process';
import { existsSync, mkdirSync, readdirSync, readFileSync, rmSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { privateMatcher } from '../../quality/leak-check.mjs';

const root = fileURLToPath(new URL('../../', import.meta.url));
const args = process.argv.slice(2);
const out = args.find((a, i) => !a.startsWith('--') && args[i - 1] !== '--rev');
const rev = args.includes('--rev') ? args[args.indexOf('--rev') + 1] : 'HEAD';
if (!out) { console.error('usage: export.mjs <empty-out-dir> [--rev <commit>] [--allowlisted]'); process.exit(2); }
const outDir = resolve(out);
if (existsSync(outDir) && readdirSync(outDir).length) { console.error(`export: ${outDir} is not empty`); process.exit(2); }
mkdirSync(outDir, { recursive: true });

const commit = spawnSync('git', ['rev-parse', '--verify', `${rev}^{commit}`], { cwd: root, encoding: 'utf8' });
if (commit.status !== 0) { console.error(`export: unknown revision ${rev}`); process.exit(2); }
const sha = commit.stdout.trim();

// git archive | tar -x, so file modes and symlinks come from the commit.
await new Promise((done, fail) => {
  const archive = spawn('git', ['archive', '--format=tar', sha], { cwd: root, stdio: ['ignore', 'pipe', 'inherit'] });
  // Run inside outDir: GNU tar reads a "C:" in a -C argument as a remote host.
  const tar = spawn('tar', ['-x', '-f', '-'], { cwd: outDir, stdio: ['pipe', 'inherit', 'inherit'] });
  archive.stdout.pipe(tar.stdin);
  let pending = 2;
  for (const child of [archive, tar]) child.on('close', (code) => (code ? fail(new Error(`${child.spawnargs[0]} exited ${code}`)) : --pending || done()));
});

const isPrivate = privateMatcher(readFileSync(join(root, 'tools/public-mirror/private-paths.txt'), 'utf8').split('\n'));
const listed = spawnSync('git', ['ls-tree', '-r', '-z', '--name-only', sha], { cwd: root, encoding: 'utf8', maxBuffer: 64 << 20 }).stdout.split('\0').filter(Boolean);
const removed = listed.filter(isPrivate);
for (const path of removed) {
  rmSync(join(outDir, path), { force: true });
  // Drop directories the exclusion emptied, so the mirror has no husks.
  for (let dir = dirname(path); dir !== '.'; dir = dirname(dir)) {
    const full = join(outDir, dir);
    if (!existsSync(full) || readdirSync(full).length) break;
    rmSync(full, { recursive: true });
  }
}
console.error(`export: ${sha.slice(0, 12)} -> ${outDir}: ${listed.length - removed.length} files, ${removed.length} private files left out`);

const check = spawnSync(process.execPath, [join(root, 'quality/leak-check.mjs'), '--dir', outDir, ...(args.includes('--allowlisted') ? [] : ['--strict'])], { stdio: 'inherit' });
process.exit(check.status ?? 1);
