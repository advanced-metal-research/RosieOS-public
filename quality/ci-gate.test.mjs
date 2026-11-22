// Contract tests for the CI gate (quality/ci-gate.mjs): the trigger reader
// against every real workflow file, the GitHub path-pattern matcher, the
// pass/fail decision (including the all-skipped run it exists to catch), and
// both runner halves against a fake GitHub API with a fake clock. "Code"
// comes from the real path filters, not from a list here. The verdict half
// runs from the default branch, so on a PR these tests are its only check.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync, readFileSync } from 'node:fs';
import {
  TransientApiError, collectRuns, commandText, decide, fetchClient, filterMatches, gateScope, globToRegExp,
  loadWorkflows, outOfScope, parsePullRequestTrigger, parseWatchedWorkflows, startCheck, statusContext,
  uncoveredFiles, verdict,
} from './ci-gate.mjs';

const workflows = loadWorkflows();
const rtCore = '.github/workflows/rt-core.yml';
const consumers = '.github/workflows/rt-core-consumers.yml';
const surface = '.github/workflows/deployment-surface.yml';
const leak = '.github/workflows/leak-check.yml';
const admission = '.github/workflows/pr-admission.yml';
const start = '.github/workflows/ci-gate-start.yml';
const rtFile = 'rt-core/src/control/loop.cc';
const docFile = 'docs/DEPLOY_AND_TEST_WORKFLOW.md';
const sha = 'a'.repeat(40);
const branch = 'sl/ci/example';

const job = (name, conclusion) => ({ name, conclusion });
const done = (...jobs) => ({ status: 'completed', conclusion: 'success', jobs });

/** Every workflow the real filters start for `files`, each run with `jobsOf(path)`. */
function runsFor(files, jobsOf) {
  const runs = {};
  for (const workflow of workflows) {
    if (filterMatches(workflow.filter, files)) runs[workflow.path] = done(...jobsOf(workflow.path));
  }
  return runs;
}

// ------------------------------------------------------------ triggers

test('every real workflow trigger is readable', () => {
  assert.ok(workflows.length >= 10, `${workflows.length} pull_request workflows`);
  const byPath = Object.fromEntries(workflows.map((w) => [w.path, w.filter]));
  assert.deepEqual(byPath[leak], { paths: null, pathsIgnore: null }, 'leak check always runs');
  assert.ok(byPath[rtCore].paths.includes('rt-core/**'));
  assert.ok(byPath[start], 'the start half is a pull_request workflow');
  assert.ok(!byPath['.github/workflows/ci-gate.yml'], 'the verdict half is not');
  assert.ok(!byPath['.github/workflows/build1-release-request.yml'], 'push only');
});

// The names ci-gate.yml watches: every in-scope pull_request workflow.
const watched = new Set(workflows.filter((w) => !outOfScope.has(w.path)).map((w) => w.name));

test('ci-gate.yml watches every pull_request workflow, by name', () => {
  const watched = parseWatchedWorkflows(readFileSync(new URL('../.github/workflows/ci-gate.yml', import.meta.url), 'utf8'));
  assert.ok(workflows.every((workflow) => workflow.name), 'every pull_request workflow has a name');
  assert.ok(outOfScope.has('.github/workflows/build1-release-worker.yml'), 'the release worker is out of scope');
  for (const workflow of workflows.filter((w) => !outOfScope.has(w.path))) {
    assert.ok(watched.includes(workflow.name), `add "${workflow.name}" to ci-gate.yml workflow_run.workflows`);
  }
  // The public mirror leaves the private workflows out; only the full tree has all of them.
  if (existsSync(new URL('../tools/public-mirror/private-paths.txt', import.meta.url))) {
    const names = new Set(workflows.map((workflow) => workflow.name));
    for (const name of watched) assert.ok(names.has(name), `ci-gate.yml watches "${name}", which no pull_request workflow is named`);
  }
});

test('trigger shapes: flow list, block list, scalar, list, pull_request_target', () => {
  assert.deepEqual(parsePullRequestTrigger('on:\n  pull_request:\n    paths: ["a/**", b/*.go] # c\n'),
    { paths: ['a/**', 'b/*.go'], pathsIgnore: null });
  assert.deepEqual(parsePullRequestTrigger(
    'name: x\non:\n  push:\n    paths:\n      - "z/**"\n  pull_request:\n    paths-ignore:\n      # docs\n      - docs/**\n      - \'*.md\'\njobs: {}\n'),
  { paths: null, pathsIgnore: ['docs/**', '*.md'] });
  assert.deepEqual(parsePullRequestTrigger('on: pull_request\n'), { paths: null, pathsIgnore: null });
  assert.deepEqual(parsePullRequestTrigger('on: [push, pull_request]\n'), { paths: null, pathsIgnore: null });
  assert.equal(parsePullRequestTrigger('on:\n  pull_request_target:\n    types: [opened]\n'), null);
  assert.equal(parsePullRequestTrigger('on: push\n'), null);
  assert.deepEqual(parseWatchedWorkflows('on:\n  workflow_run:\n    workflows: ["A · b", C]\n    types: [completed]\n'),
    ['A · b', 'C']);
});

test('a filter the gate cannot model fails loudly, not as "always runs"', () => {
  assert.throws(() => parsePullRequestTrigger('on:\n  pull_request:\n    types: [opened]\n'), /types/);
  assert.throws(() => parsePullRequestTrigger('on:\n  pull_request:\n    branches: [main]\n'), /branches/);
  assert.throws(() => parsePullRequestTrigger('jobs: {}\n'), /on:/);
});

test('path patterns follow GitHub filter syntax', () => {
  assert.ok(globToRegExp('**/*.service').test('a.service'), '**/ matches the root');
  assert.ok(globToRegExp('**/*.service').test('x/y/a.service'));
  assert.ok(globToRegExp('**/cmd/**').test('cmd/main.go'));
  assert.ok(globToRegExp('quality/deployment-*').test('quality/deployment-tools.test.mjs'));
  assert.ok(!globToRegExp('quality/deployment-*').test('quality/deployment/x'), '* stays in a segment');
  assert.ok(globToRegExp('.github/workflows/preflight*.yml').test('.github/workflows/preflight-release.yml'));
  assert.ok(!globToRegExp('.github/workflows/preflight*.yml').test('.github/workflows/preflightXyml'), '. is literal');
  assert.ok(globToRegExp('rt-core/**').test('rt-core/a/b/c.cc'));
  assert.ok(!globToRegExp('rt-core/**').test('rt-core-x/a'));
  assert.ok(globToRegExp('*.jsx?').test('page.js'), '? is zero or one of the preceding character');
  assert.ok(globToRegExp('*.jsx?').test('page.jsx'));
  assert.ok(!globToRegExp('*.jsx?').test('page.jsy'));
  assert.ok(globToRegExp('v[0-9]+.txt').test('v12.txt'), '+ is one or more');
  assert.ok(filterMatches({ paths: ['a/**', '!a/docs/**'], pathsIgnore: null }, ['a/src/x']));
  assert.ok(!filterMatches({ paths: ['a/**', '!a/docs/**'], pathsIgnore: null }, ['a/docs/x']));
  assert.ok(!filterMatches({ paths: null, pathsIgnore: ['docs/**'] }, ['docs/x.md']));
  assert.ok(filterMatches({ paths: null, pathsIgnore: ['docs/**'] }, ['docs/x.md', 'src/y']));
});

test('the real filters start rt-core.yml for an rt-core change and nothing filtered for docs', () => {
  const started = (files) => workflows.filter((w) => filterMatches(w.filter, files)).map((w) => w.path);
  const rt = started([rtFile]);
  for (const path of [rtCore, consumers, leak, admission, start]) assert.ok(rt.includes(path), path);
  assert.deepEqual(started([docFile]).sort(), [leak, admission, start].sort());
});

test('code no path-filtered workflow covers is listed, docs are not', () => {
  assert.deepEqual(uncoveredFiles(workflows, ['offline-programming/v2/x.go', docFile, 'README.md', rtFile]),
    ['offline-programming/v2/x.go']);
});

// ------------------------------------------------------------- decision

test('all success passes', () => {
  const result = decide({ files: [rtFile], workflows, runs: runsFor([rtFile], (p) => [job(`${p} a`, 'success')]) });
  assert.equal(result.state, 'pass', result.reason);
});

test('some skipped jobs pass when each started workflow ran one', () => {
  const result = decide({ files: [rtFile], workflows, runs: runsFor([rtFile], () => [job('a', 'success'), job('b', 'skipped')]) });
  assert.equal(result.state, 'pass', result.reason);
  assert.match(result.reason, /skipped/);
});

test('one failed job fails and names the workflow', () => {
  const runs = runsFor([rtFile], () => [job('a', 'success')]);
  runs[rtCore] = { status: 'completed', conclusion: 'failure', jobs: [job('a', 'success'), job('b', 'failure')] };
  const result = decide({ files: [rtFile], workflows, runs });
  assert.equal(result.state, 'fail');
  assert.match(result.reason, /rt-core\.yml: b: failure/);
});

test('one cancelled job fails', () => {
  const runs = runsFor([rtFile], () => [job('a', 'success')]);
  runs[consumers] = { status: 'completed', conclusion: 'cancelled', jobs: [job('a', 'success'), job('b', 'cancelled')] };
  const result = decide({ files: [rtFile], workflows, runs });
  assert.equal(result.state, 'fail');
  assert.match(result.reason, /cancelled/);
});

test('a continue-on-error failure in a green run stays advisory', () => {
  const files = ['steamdeck/real/v4/main.go'];
  const runs = runsFor(files, () => [job('a', 'success')]);
  assert.ok(runs[surface], 'the deployment surface starts for this change');
  runs[surface] = { status: 'completed', conclusion: 'success', jobs: [job('report', 'failure')] };
  const result = decide({ files, workflows, runs });
  assert.equal(result.state, 'pass', result.reason);
  assert.match(result.reason, /1 advisory failed/);
});

test('an rt-core change whose test jobs all skipped fails', () => {
  const result = decide({ files: [rtFile], workflows, runs: runsFor([rtFile], () => [job('a', 'skipped'), job('b', 'skipped')]) });
  assert.equal(result.state, 'fail');
  assert.match(result.reason, /every job skipped/);
});

test('rt-core.yml all skipped fails even when every other workflow ran', () => {
  const runs = runsFor([rtFile], () => [job('a', 'success')]);
  runs[rtCore] = done(job('standalone (native)', 'skipped'), job('standalone (sanitize)', 'skipped'));
  const result = decide({ files: [rtFile], workflows, runs });
  assert.equal(result.state, 'fail');
  assert.equal(result.reason, `${rtCore} matched the change but every job skipped`);
});

test('a docs-only change with every job skipped passes', () => {
  const result = decide({ files: [docFile], workflows, runs: runsFor([docFile], () => [job('a', 'skipped')]) });
  assert.equal(result.state, 'pass', result.reason);
});

test('uncovered code passes with the paths listed', () => {
  const files = ['weld_planner/v1/python/x.py'];
  const result = decide({ files, workflows, runs: runsFor(files, () => [job('a', 'success')]) });
  assert.equal(result.state, 'pass', result.reason);
  assert.deepEqual(result.uncovered, files);
});

test('a matched workflow with no run waits, then fails once runs stop appearing', () => {
  const runs = runsFor([rtFile], () => [job('a', 'success')]);
  delete runs[rtCore];
  assert.equal(decide({ files: [rtFile], workflows, runs }).state, 'pending');
  const closed = decide({ files: [rtFile], workflows, runs, startClosed: true });
  assert.equal(closed.state, 'fail');
  assert.match(closed.reason, /rt-core\.yml matched the change but did not start/);
});

test('an unfinished run waits, and fails at the deadline', () => {
  const runs = runsFor([rtFile], () => [job('a', 'success')]);
  runs[rtCore] = { status: 'in_progress', conclusion: null };
  assert.equal(decide({ files: [rtFile], workflows, runs, startClosed: true }).state, 'pending');
  assert.equal(decide({ files: [rtFile], workflows, runs, final: true }).state, 'fail');
});

test('a run that failed with no job (workflow file error) fails', () => {
  const runs = runsFor([rtFile], () => [job('a', 'success')]);
  runs[rtCore] = { status: 'completed', conclusion: 'failure', jobs: [] };
  const result = decide({ files: [rtFile], workflows, runs });
  assert.equal(result.state, 'fail');
  assert.match(result.reason, /concluded failure with no failed job/);
});

// --------------------------------------------------- fake GitHub and clock

/**
 * A fake GitHub API. `state.runs` is the run listing, `state.jobs[id]` a
 * run's latest-attempt jobs, `state.files` the PR files, `state.head` the
 * PR's current head. `state.fail` lists responses to fail before answering.
 */
function fakeGitHub(state) {
  const posted = [];
  const calls = [];
  const client = {
    get: async (resource) => {
      calls.push(resource);
      if (state.fail?.length) {
        const status = state.fail.shift();
        throw status >= 500 || status === 429 ? new TransientApiError(`HTTP ${status}`) : new Error(`HTTP ${status}`);
      }
      if (resource.includes('/files')) return resource.endsWith('page=1') ? state.files.map((filename) => ({ filename })) : [];
      if (/\/actions\/runs\/\d+\/jobs/.test(resource)) {
        return { jobs: page(state.jobs[Number(/runs\/(\d+)\/jobs/.exec(resource)[1])] ?? [], resource),
          total_count: (state.jobs[Number(/runs\/(\d+)\/jobs/.exec(resource)[1])] ?? []).length };
      }
      if (resource.includes('/actions/runs?')) return { workflow_runs: page(state.runs(), resource), total_count: state.runs().length };
      if (resource.includes('/commits/')) return state.commitPulls ?? [];
      if (/\/pulls\/\d+$/.test(resource)) return { head: { sha: state.head ?? sha, ref: branch } };
      throw new Error(`unexpected ${resource}`);
    },
    post: async (resource, body) => { posted.push({ resource, body }); return {}; },
  };
  return { client, posted, calls };
}

/** One 100-item page of `items`, as GitHub serves `page=N&per_page=100`. */
function page(items, resource) {
  const n = Number(/[?&]page=(\d+)/.exec(resource)?.[1] ?? 1);
  assert.match(resource, /per_page=100/);
  return items.slice((n - 1) * 100, n * 100);
}

function fakeClock() {
  let t = 1_000_000;
  return { now: () => t, sleep: async (ms) => { t += ms; } };
}

const run = (id, path, status, conclusion, extra = {}) => ({
  id, path, name: workflows.find((w) => w.path === path.split('@')[0])?.name ?? path, status, conclusion, head_sha: sha, head_branch: branch, event: 'pull_request', run_attempt: 1, ...extra,
});

test('collectRuns keeps the newest run per path and drops other branches, events and SHAs', async () => {
  const { client } = fakeGitHub({
    runs: () => [
      run(10, rtCore, 'completed', 'failure'),
      run(12, `${rtCore}@refs/pull/1/merge`, 'completed', 'success'),
      run(13, consumers, 'queued', null),
      run(14, leak, 'completed', 'failure', { head_branch: 'someone/else' }),
      run(15, admission, 'completed', 'failure', { event: 'pull_request_target' }),
      run(16, start, 'completed', 'failure', { head_sha: 'b'.repeat(40) }),
    ],
    jobs: { 12: [job('standalone (native)', 'success')] },
  });
  const runs = await collectRuns({ get: client.get, repo: 'o/r', sha, branch });
  assert.deepEqual(Object.keys(runs).sort(), [consumers, rtCore].sort());
  assert.equal(runs[rtCore].conclusion, 'success');
  assert.deepEqual(runs[rtCore].jobs, [job('standalone (native)', 'success')]);
  assert.equal(runs[consumers].jobs, undefined);
});

test('fetchClient: 5xx and rate limits are transient, 404 is not', async () => {
  const respond = (status, headers = {}) => async () => ({ ok: false, status, headers: new Map(Object.entries(headers)) });
  const get = (fetchImpl) => fetchClient({ fetchImpl, api: 'https://api.example', token: 't' }).get('/x');
  await assert.rejects(get(respond(502)), TransientApiError);
  await assert.rejects(get(respond(429)), TransientApiError);
  await assert.rejects(get(respond(403, { 'x-ratelimit-remaining': '0' })), TransientApiError);
  await assert.rejects(get(async () => { throw new Error('socket hang up'); }), TransientApiError);
  await assert.rejects(get(respond(404)), (error) => !(error instanceof TransientApiError));
  await assert.rejects(get(respond(403)), (error) => !(error instanceof TransientApiError));
});

// ---------------------------------------------------------------- start half

test('start passes once every matched workflow has a run, waiting through a late one', async () => {
  const clock = fakeClock();
  const t0 = clock.now();
  const present = () => [run(1, leak, 'in_progress', null), run(2, admission, 'queued', null), run(3, start, 'in_progress', null),
    run(4, consumers, 'queued', null), run(6, '.github/workflows/robot-runtime-contracts.yml', 'queued', null)];
  const { client } = fakeGitHub({
    files: [rtFile],
    runs: () => (clock.now() - t0 >= 20_000 ? [...present(), run(5, rtCore, 'queued', null)] : present()),
  });
  const result = await startCheck({ watched, client, repo: 'o/r', pr: 1, sha, branch, workflows, ...clock, pollMs: 10_000 });
  assert.equal(result.state, 'pass', result.reason);
  assert.equal(clock.now() - t0, 20_000);
});

test('start fails at the grace period when a matched workflow never starts', async () => {
  const clock = fakeClock();
  const t0 = clock.now();
  const { client } = fakeGitHub({ files: [rtFile], runs: () => [run(1, leak, 'queued', null)] });
  const result = await startCheck({ watched, client, repo: 'o/r', pr: 1, sha, branch, workflows, ...clock, graceMs: 90_000, pollMs: 10_000 });
  assert.equal(result.state, 'fail');
  assert.match(result.reason, /did not start: .*rt-core\.yml/);
  assert.equal(clock.now() - t0, 90_000);
});

test('start waits out transient API errors, and gives up at the deadline', async () => {
  const clock = fakeClock();
  const ok = fakeGitHub({ files: [docFile], fail: [502, 429], runs: () => [run(1, leak, 'queued', null), run(2, admission, 'queued', null), run(3, start, 'queued', null)] });
  assert.equal((await startCheck({ watched, client: ok.client, repo: 'o/r', pr: 1, sha, branch, workflows, ...clock })).state, 'pass');
  const down = fakeGitHub({ files: [docFile], fail: Array(100).fill(503), runs: () => [] });
  await assert.rejects(startCheck({ watched, client: down.client, repo: 'o/r', pr: 1, sha, branch, workflows, ...fakeClock(), deadlineMs: 60_000 }),
    TransientApiError);
  const denied = fakeGitHub({ files: [docFile], fail: [404], runs: () => [] });
  await assert.rejects(startCheck({ watched, client: denied.client, repo: 'o/r', pr: 1, sha, branch, workflows, ...fakeClock() }),
    (error) => !(error instanceof TransientApiError), 'a 404 is not waited out');
});

// -------------------------------------------------------------- verdict half

/** Runs for an rt-core change: every matched workflow completed with `rtCoreJobs` for rt-core.yml. */
function rtCoreState(rtCoreConclusion, rtCoreJobs) {
  const matched = workflows.filter((w) => filterMatches(w.filter, [rtFile]));
  const jobs = {};
  const runs = matched.map((w, i) => {
    const id = 100 + i;
    const isRt = w.path === rtCore;
    jobs[id] = isRt ? rtCoreJobs : [job('a', 'success')];
    return run(id, w.path, 'completed', isRt ? rtCoreConclusion : 'success');
  });
  return { files: [rtFile], jobs, runs: () => runs };
}

test('verdict posts failure, then success after a passing re-run of the same run', async () => {
  const state = rtCoreState('failure', [job('standalone (native)', 'failure')]);
  const github = fakeGitHub(state);
  const first = await verdict({ watched, client: github.client, repo: 'o/r', sha, branch, pr: 1, workflows, targetUrl: 'https://x/run', ...fakeClock() });
  assert.equal(first.state, 'fail');
  assert.deepEqual(github.posted[0], {
    resource: `/repos/o/r/statuses/${sha}`,
    body: { state: 'failure', context: statusContext, description: first.status.description, target_url: 'https://x/run' },
  });
  // The re-run is the same run id with a new attempt: the listing and jobs now report it.
  const rtId = state.runs().find((r) => r.path === rtCore).id;
  state.runs().find((r) => r.id === rtId).conclusion = 'success';
  state.jobs[rtId] = [job('standalone (native)', 'success')];
  const second = await verdict({ watched, client: github.client, repo: 'o/r', sha, branch, pr: 1, workflows, ...fakeClock() });
  assert.equal(second.state, 'pass', second.reason);
  assert.equal(github.posted[1].body.state, 'success');
});

test('verdict turns a green gate red when a re-run fails', async () => {
  const state = rtCoreState('success', [job('standalone (native)', 'success')]);
  const github = fakeGitHub(state);
  assert.equal((await verdict({ watched, client: github.client, repo: 'o/r', sha, branch, pr: 1, workflows, ...fakeClock() })).state, 'pass');
  const rt = state.runs().find((r) => r.path === rtCore);
  rt.conclusion = 'failure';
  state.jobs[rt.id] = [job('standalone (native)', 'failure')];
  assert.equal((await verdict({ watched, client: github.client, repo: 'o/r', sha, branch, pr: 1, workflows, ...fakeClock() })).state, 'fail');
  assert.deepEqual(github.posted.map((p) => p.body.state), ['success', 'failure']);
});

test('verdict fails a matched workflow with no run at once, and posts pending while runs are open', async () => {
  const state = rtCoreState('success', [job('a', 'success')]);
  const all = state.runs();
  const github = fakeGitHub({ ...state, runs: () => all.filter((r) => r.path !== rtCore) });
  const missing = await verdict({ watched, client: github.client, repo: 'o/r', sha, branch, pr: 1, workflows, ...fakeClock() });
  assert.match(missing.reason, /rt-core\.yml matched the change but did not start/);
  const open = fakeGitHub({ ...state, runs: () => all.map((r) => (r.path === rtCore ? { ...r, status: 'in_progress', conclusion: null } : r)) });
  const pending = await verdict({ watched, client: open.client, repo: 'o/r', sha, branch, pr: 1, workflows, ...fakeClock() });
  assert.equal(pending.status.state, 'pending');
});

test('verdict skips a head its PR has moved on from, and finds the PR when the event names none', async () => {
  const moved = fakeGitHub({ ...rtCoreState('success', []), head: 'c'.repeat(40) });
  const stale = await verdict({ watched, client: moved.client, repo: 'o/r', sha, branch, pr: 1, workflows, ...fakeClock() });
  assert.equal(stale.state, 'stale');
  assert.equal(moved.posted.length, 0);
  const lookup = fakeGitHub({ ...rtCoreState('success', [job('a', 'success')]),
    commitPulls: [{ number: 7, head: { sha, ref: 'other' } }, { number: 9, head: { sha, ref: branch } }] });
  const found = await verdict({ watched, client: lookup.client, repo: 'o/r', sha, branch, pr: null, workflows, ...fakeClock() });
  assert.equal(found.pr, 9);
  assert.equal(found.state, 'pass', found.reason);
});

test('verdict status description fits GitHub\'s 140 characters and counts uncovered paths', async () => {
  const files = ['weld_planner/v1/python/x.py'];
  const github = fakeGitHub({ files, jobs: { 1: [job('a', 'success')], 2: [job('a', 'success')], 3: [job('a', 'success')] },
    runs: () => [run(1, leak, 'completed', 'success'), run(2, admission, 'completed', 'success'), run(3, start, 'completed', 'success')] });
  const result = await verdict({ watched, client: github.client, repo: 'o/r', sha, branch, pr: 1, workflows, ...fakeClock() });
  assert.equal(result.status.state, 'success');
  assert.ok(result.status.description.length <= 140);
  assert.match(result.status.description, /1 paths no test workflow covers/);
});

test('the release worker is out of scope: never judged, never awaited, never warned about', async () => {
  const worker = '.github/workflows/build1-release-worker.yml';
  const files = ['tools/build1-release-worker/README.md'];
  const { gated, unwatched } = gateScope(workflows, watched);
  assert.ok(!gated.some((w) => w.path === worker) && !unwatched.some((w) => w.path === worker));
  const clock = fakeClock();
  const t0 = clock.now();
  const github = fakeGitHub({
    files,
    jobs: { 1: [job('a', 'success')], 2: [job('a', 'success')], 3: [job('a', 'success')], 4: [job('t', 'failure')] },
    runs: () => [run(1, leak, 'completed', 'success'), run(2, admission, 'completed', 'success'),
      run(3, start, 'completed', 'success'), run(4, worker, 'completed', 'failure')],
  });
  const result = await verdict({ watched, client: github.client, repo: 'o/r', sha, branch, pr: 1, workflows, ...clock });
  assert.equal(result.state, 'pass', result.reason);
  assert.ok(!result.rows.some((row) => row.path === worker));
  assert.equal(clock.now(), t0, 'the verdict never waits');
  const queued = fakeGitHub({ files, runs: () => [run(1, leak, 'queued', null), run(2, admission, 'queued', null), run(3, start, 'queued', null)] });
  const started = await startCheck({ watched, client: queued.client, repo: 'o/r', pr: 1, sha, branch, workflows, ...fakeClock() });
  assert.equal(started.state, 'pass', 'a worker that never starts is not missing');
  assert.deepEqual(started.unwatched, []);
});

test('a new workflow ci-gate.yml does not watch is ignored by the verdict and warned about by start', async () => {
  const fresh = { path: '.github/workflows/new-tests.yml', name: 'New tests', filter: { paths: ['newpkg/**'], pathsIgnore: null } };
  const all = [...workflows, fresh];
  const files = ['newpkg/a.go'];
  const github = fakeGitHub({
    files,
    jobs: { 1: [job('a', 'success')], 2: [job('a', 'success')], 3: [job('a', 'success')] },
    runs: () => [run(1, leak, 'completed', 'success'), run(2, admission, 'completed', 'success'),
      run(3, start, 'completed', 'success'), run(4, fresh.path, 'in_progress', null, { name: fresh.name })],
  });
  const result = await verdict({ watched, client: github.client, repo: 'o/r', sha, branch, pr: 1, workflows: all, ...fakeClock() });
  assert.equal(result.state, 'pass', 'an open unwatched run cannot leave the gate pending');
  const begun = await startCheck({ watched, client: github.client, repo: 'o/r', pr: 1, sha, branch, workflows: all, ...fakeClock() });
  assert.deepEqual(begun.unwatched, [fresh.path]);
});

test('runs and jobs are read past the first 100', async () => {
  const many = Array.from({ length: 150 }, (_, i) => run(1000 + i, `.github/workflows/w${i}.yml`, 'completed', 'success'));
  const jobs = Array.from({ length: 120 }, (_, i) => job(`leg ${i}`, i === 119 ? 'failure' : 'success'));
  const github = fakeGitHub({ runs: () => many, jobs: Object.fromEntries(many.map((r) => [r.id, jobs])) });
  const runs = await collectRuns({ get: github.client.get, repo: 'o/r', sha, branch });
  assert.equal(Object.keys(runs).length, 150);
  assert.equal(runs['.github/workflows/w149.yml'].jobs.length, 120);
  assert.equal(runs['.github/workflows/w0.yml'].jobs[119].conclusion, 'failure');
});

test('warning text escapes the workflow-command characters', () => {
  assert.equal(commandText('a%b\rc\nd'), 'a%25b%0Dc%0Ad');
});
