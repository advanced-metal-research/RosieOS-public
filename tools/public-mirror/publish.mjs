#!/usr/bin/env node
// Publishes one commit of this repository to the public mirror,
// advanced-metal-research/RosieOS-public, as a single new commit on its main:
// export.mjs (private paths left out, strict leak check), then gitleaks over
// the exported tree, then the tree committed on top of the mirror's previous
// commit under a noreply identity. Nothing from this repository's history
// travels: the mirror's history is one commit per publish.
//
// Prepares and reports by default; pushes only with --push. Refuses a commit
// that is not on origin/main, so an unmerged branch is never published.
//
//   node tools/public-mirror/publish.mjs [--rev <commit>] [--push]
//        [--author "Name <email>"] [--message <text>] [--allow-unmerged]
import { spawnSync } from 'node:child_process';
import { mkdtempSync, rmSync } from 'node:fs';
import { homedir, tmpdir } from 'node:os';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';

const MIRROR = 'https://github.com/advanced-metal-research/RosieOS-public.git';
const root = fileURLToPath(new URL('../../', import.meta.url));
const args = process.argv.slice(2);
const flag = (name) => args.includes(name);
const value = (name, fallback) => (args.includes(name) ? args[args.indexOf(name) + 1] : fallback);

function run(cmd, argv, opts = {}) {
  const r = spawnSync(cmd, argv, { encoding: 'utf8', maxBuffer: 256 << 20, ...opts });
  if (r.error) throw r.error;
  return r;
}
function must(cmd, argv, opts = {}) {
  const r = run(cmd, argv, opts);
  if (r.status !== 0) {
    console.error(`publish: ${cmd} ${argv.join(' ')} failed\n${r.stderr || r.stdout}`);
    process.exit(1);
  }
  return (r.stdout ?? '').trim();
}

if (flag('--push') && flag('--allow-unmerged')) {
  console.error('publish: --allow-unmerged is for dry runs; publish only commits on origin/main');
  process.exit(2);
}
const rev = value('--rev', 'HEAD');
const sha = must('git', ['rev-parse', '--verify', `${rev}^{commit}`], { cwd: root });
if (!flag('--allow-unmerged') && run('git', ['merge-base', '--is-ancestor', sha, 'origin/main'], { cwd: root }).status !== 0) {
  console.error(`publish: ${sha.slice(0, 12)} is not on origin/main (fetch first, or --allow-unmerged for a dry run)`);
  process.exit(1);
}

// The identity on the mirror's commit: given, or the publisher's GitHub noreply address.
let author = value('--author');
if (!author) {
  const user = run('gh', ['api', 'user', '-q', '.login + " " + (.id|tostring)']);
  if (user.status !== 0) { console.error('publish: pass --author "Name <email>" (gh api user failed)'); process.exit(1); }
  const [login, id] = user.stdout.trim().split(' ');
  author = `${login} <${id}+${login}@users.noreply.github.com>`;
}
const identity = /^(.+?) <([^>]+)>$/.exec(author);
if (!identity) { console.error(`publish: --author must be "Name <email>", got ${author}`); process.exit(1); }

// The scanner: gitleaks on PATH, or the pinned one `go install` puts in ~/go/bin.
const gitleaks = ['gitleaks', join(homedir(), 'go', 'bin', process.platform === 'win32' ? 'gitleaks.exe' : 'gitleaks')]
  .find((g) => spawnSync(g, ['version'], { stdio: 'ignore' }).status === 0);
if (!gitleaks) {
  console.error('publish: gitleaks not found; go install github.com/zricethezav/gitleaks/v8@v8.28.0');
  process.exit(1);
}

const work = mkdtempSync(join(tmpdir(), 'rosieos-publish-'));
const tree = join(work, 'tree');
const gitDir = join(work, 'mirror.git');
try {
  must(process.execPath, [join(root, 'tools/public-mirror/export.mjs'), tree, '--rev', sha], { stdio: ['ignore', 'inherit', 'inherit'] });
  must(gitleaks, ['dir', tree, '--config', join(root, 'tools/public-mirror/gitleaks.toml'), '--no-banner', '--redact', '--exit-code', '1'],
    { stdio: ['ignore', 'inherit', 'inherit'] });

  // A repository whose work tree is the export: staging it stages exactly the
  // exported files, and deletions relative to the mirror's previous commit.
  const git = (...argv) => must('git', ['--git-dir', gitDir, '--work-tree', tree, ...argv]);
  must('git', ['init', '-q', '--bare', gitDir]);
  must('git', ['--git-dir', gitDir, 'config', 'core.bare', 'false']);
  git('remote', 'add', 'public', MIRROR);
  const fetched = run('git', ['--git-dir', gitDir, 'fetch', '-q', 'public', 'main']);
  const first = fetched.status !== 0;
  if (first && !/couldn't find remote ref|does not appear to be a git repository|empty/i.test(fetched.stderr)) {
    console.error(`publish: fetching ${MIRROR} failed\n${fetched.stderr}`);
    process.exit(1);
  }
  if (!first) git('update-ref', 'refs/heads/main', 'FETCH_HEAD');
  git('symbolic-ref', 'HEAD', 'refs/heads/main');
  if (!first) git('read-tree', 'main');
  git('add', '-A');
  if (!first && run('git', ['--git-dir', gitDir, '--work-tree', tree, 'diff', '--cached', '--quiet']).status === 0) {
    console.log('publish: the mirror already has this tree; nothing to publish');
    process.exit(0);
  }
  const message = value('--message', first ? 'RosieOS: first public release' : `RosieOS: update ${new Date().toISOString().slice(0, 10)}`);
  const env = { ...process.env, GIT_AUTHOR_NAME: identity[1], GIT_AUTHOR_EMAIL: identity[2], GIT_COMMITTER_NAME: identity[1], GIT_COMMITTER_EMAIL: identity[2] };
  must('git', ['--git-dir', gitDir, '--work-tree', tree, 'commit', '-q', '-m', message], { env });
  const summary = must('git', ['--git-dir', gitDir, 'log', '-1', '--format=%h %an <%ae>%n%s']);
  const stat = first
    ? `${must('git', ['--git-dir', gitDir, 'ls-tree', '-r', '--name-only', 'HEAD']).split('\n').length} files`
    : must('git', ['--git-dir', gitDir, 'diff', '--shortstat', 'HEAD~1', 'HEAD']);
  console.log(`publish: ${sha.slice(0, 12)} -> mirror commit ${summary.replace('\n', ' "')}" (${first ? 'first publish' : 'on top of the previous publish'}, ${stat})`);
  if (!flag('--push')) {
    console.log(`publish: not pushed (dry run). Re-run with --push to publish to ${MIRROR}`);
  } else {
    must('git', ['--git-dir', gitDir, 'push', '-q', 'public', 'main:main'], { stdio: ['ignore', 'inherit', 'inherit'] });
    console.log(`publish: pushed to ${MIRROR}`);
  }
} finally {
  rmSync(work, { recursive: true, force: true });
}
