#!/usr/bin/env node
// The CI gate: one verdict per pull request head over every workflow run the
// PR started.
//
// Most PR workflows filter on `on.pull_request.paths`, so on a PR outside
// their paths they never start and report nothing. A check inside them cannot
// report on every PR, and nothing notices when a workflow that should have
// run did not start, or started and skipped every job: the PR shows green
// with no test run. The gate has two halves:
//   - `start` runs on the PR (ci-gate-start.yml). It checks that every
//     workflow whose path filter matches the change has a run, and warns
//     about changed code no path-filtered workflow covers.
//   - `verdict` runs on every watched run's completion (ci-gate.yml,
//     workflow_run, default-branch code and filters, so a PR cannot change
//     its own verdict). It recomputes the head's verdict from the latest
//     attempt of every run and posts it as the `CI gate` commit status, so a
//     re-run that passes or fails moves the gate with it.
// The verdict fails when a run concluded badly (a job that failed under
// continue-on-error leaves the run green and stays advisory), when a
// path-filtered workflow matched the change but did not start, or when one
// ran and skipped every job. It judges runs by workflow path and conclusion,
// never by job name.
//
// Scope: only the workflows ci-gate.yml watches by name are judged, so every
// judged run's completion wakes the verdict and a new or renamed workflow
// cannot strand it. `start` warns when an unwatched path-filtered workflow
// matches the change. The build host's release worker
// (build1-release-worker.yml) is out of scope: it is release automation, not
// a test workflow.

import { appendFileSync, readFileSync, readdirSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = fileURLToPath(new URL('../', import.meta.url));
const workflowDir = join(root, '.github/workflows');
export const statusContext = 'CI gate';

// GitHub's own run and job conclusions that do not fail a PR.
const passingConclusions = new Set(['success', 'skipped', 'neutral']);

// Warning only: changed files that need no test workflow. Never used to
// decide pass or fail.
const noTestNeeded = ['**/*.md', 'docs/**'];

// Pull request workflows the gate never judges and never warns about.
export const outOfScope = new Set(['.github/workflows/build1-release-worker.yml']);

// ---------------------------------------------------------------- YAML reading

/** Remove a YAML comment and surrounding quotes from one scalar. */
function scalar(text) {
  const value = text.replace(/(^|\s)#.*$/, '').trim();
  const quoted = /^(['"])(.*)\1$/.exec(value);
  return quoted ? quoted[2] : value;
}

/** Parse a one-line YAML flow list such as `["a/**", b]`. */
function flowList(text) {
  const value = text.replace(/(^|\s)#.*$/, '').trim();
  if (!value.startsWith('[') || !value.endsWith(']')) {
    throw new Error(`expected a one-line [list], got: ${value}`);
  }
  const body = value.slice(1, -1).trim();
  return body ? body.split(',').map(scalar) : [];
}

const indentOf = (line) => line.length - line.trimStart().length;
const meaningful = (text) => text.split(/\r?\n/)
  .filter((line) => line.trim() !== '' && !line.trimStart().startsWith('#'));

/** The top-level `name:` of a workflow file, or null. */
export function workflowName(text) {
  const line = meaningful(text).find((candidate) => /^name\s*:/.test(candidate));
  return line ? scalar(line.slice(line.indexOf(':') + 1)) : null;
}

/**
 * Find one event under the top-level `on:` key.
 *
 * Returns null when the workflow does not have the event, otherwise
 * `{ keys }`: the event's child keys, each a list (flow or block) or a
 * scalar.
 */
function eventKeys(text, event) {
  const lines = meaningful(text);
  const onIndex = lines.findIndex((line) => /^(on|"on"|'on'):/.test(line));
  if (onIndex < 0) throw new Error('no top-level on: key');
  const inline = scalar(lines[onIndex].slice(lines[onIndex].indexOf(':') + 1));
  if (inline !== '') {
    const events = inline.startsWith('[') ? flowList(inline) : [inline];
    return events.includes(event) ? { keys: {} } : null;
  }
  const block = [];
  for (const line of lines.slice(onIndex + 1)) {
    if (indentOf(line) === 0) break;
    block.push(line);
  }
  if (block.length === 0) throw new Error('empty on: block');
  const eventIndent = indentOf(block[0]);
  const start = block.findIndex((line) => indentOf(line) === eventIndent &&
    new RegExp(`^${event}\\s*:`).test(line.trim()));
  if (start < 0) return null;
  const rest = scalar(block[start].trim().slice(event.length).replace(/^\s*:/, ''));
  if (rest !== '' && rest !== '{}' && rest !== 'null' && rest !== '~') {
    throw new Error(`unsupported ${event} value: ${rest}`);
  }
  const children = [];
  for (const line of block.slice(start + 1)) {
    if (indentOf(line) <= eventIndent) break;
    children.push(line);
  }
  const keys = {};
  if (children.length === 0) return { keys };
  const keyIndent = indentOf(children[0]);
  for (let i = 0; i < children.length; i += 1) {
    if (indentOf(children[i]) !== keyIndent) continue;
    const match = /^([\w-]+)\s*:(.*)$/.exec(children[i].trim());
    if (!match) throw new Error(`unreadable ${event} line: ${children[i].trim()}`);
    const [, key, value] = match;
    if (scalar(value) !== '') {
      keys[key] = scalar(value).startsWith('[') ? flowList(value) : scalar(value);
      continue;
    }
    const list = [];
    for (let j = i + 1; j < children.length && indentOf(children[j]) > keyIndent; j += 1) {
      const item = /^-\s+(.*)$/.exec(children[j].trim());
      if (!item) throw new Error(`expected a "- item" under ${key}: ${children[j].trim()}`);
      list.push(scalar(item[1]));
    }
    keys[key] = list;
  }
  return { keys };
}

/**
 * Read the pull_request trigger of one workflow file.
 *
 * Returns null when the workflow does not run on pull_request, otherwise
 * `{ paths, pathsIgnore }` (either may be null). Throws on any filter it does
 * not model, so a new filter kind fails the gate loudly instead of being read
 * as "always runs".
 */
export function parsePullRequestTrigger(text) {
  const event = eventKeys(text, 'pull_request');
  if (!event) return null;
  for (const key of Object.keys(event.keys)) {
    if (key !== 'paths' && key !== 'paths-ignore') {
      throw new Error(`unsupported pull_request filter "${key}": teach quality/ci-gate.mjs about it`);
    }
  }
  const filter = { paths: event.keys.paths ?? null, pathsIgnore: event.keys['paths-ignore'] ?? null };
  if (filter.paths && filter.pathsIgnore) {
    throw new Error('paths and paths-ignore together are not allowed by GitHub');
  }
  return filter;
}

/** The workflow names a workflow_run trigger watches, or null. */
export function parseWatchedWorkflows(text) {
  const event = eventKeys(text, 'workflow_run');
  return event ? event.keys.workflows ?? [] : null;
}

// ------------------------------------------------------------- path filters

/** Translate one GitHub filter pattern (*, **, ?, +, [..]) into a RegExp. */
export function globToRegExp(pattern) {
  let source = '';
  for (let i = 0; i < pattern.length; i += 1) {
    const c = pattern[i];
    if (c === '*' && pattern[i + 1] === '*') {
      i += 1;
      if (pattern[i + 1] === '/') {
        i += 1;
        source += '(?:.*/)?';
      } else {
        source += '.*';
      }
    } else if (c === '*') {
      source += '[^/]*';
    } else if (c === '?' || c === '+') {
      // GitHub: zero or one / one or more of the preceding character.
      source += c;
    } else if (c === '[') {
      const end = pattern.indexOf(']', i);
      if (end < 0) throw new Error(`unclosed [ in pattern: ${pattern}`);
      source += pattern.slice(i, end + 1);
      i = end;
    } else {
      source += c.replace(/[.^${}()|\\]/g, '\\$&');
    }
  }
  return new RegExp(`^${source}$`);
}

/** Whether a pattern list selects one file; a later `!pattern` unselects it. */
function selects(patterns, file) {
  let selected = false;
  for (const pattern of patterns) {
    const negated = pattern.startsWith('!');
    if (globToRegExp(negated ? pattern.slice(1) : pattern).test(file)) selected = !negated;
  }
  return selected;
}

/** Whether a path-filtered trigger counts one file as its own. */
function covers(filter, file) {
  if (filter.paths) return selects(filter.paths, file);
  if (filter.pathsIgnore) return !selects(filter.pathsIgnore, file);
  return false;
}

const isPathFiltered = (filter) => Boolean(filter && (filter.paths || filter.pathsIgnore));

/** Whether a workflow's trigger filter starts it for this list of changed files. */
export function filterMatches(filter, files) {
  return isPathFiltered(filter) ? files.some((file) => covers(filter, file)) : true;
}

/** Changed files that no path-filtered workflow covers (docs left out). */
export function uncoveredFiles(workflows, files) {
  const filtered = workflows.filter((workflow) => isPathFiltered(workflow.filter));
  return files.filter((file) => !selects(noTestNeeded, file) &&
    !filtered.some((workflow) => covers(workflow.filter, file)));
}

/** Read every workflow under `dir` that runs on pull_request. */
export function loadWorkflows(dir = workflowDir) {
  return readdirSync(dir)
    .filter((name) => /\.ya?ml$/.test(name))
    .sort()
    .map((name) => {
      const path = `.github/workflows/${name}`;
      const text = readFileSync(join(dir, name), 'utf8');
      try {
        return { path, name: workflowName(text), filter: parsePullRequestTrigger(text) };
      } catch (error) {
        throw new Error(`${path}: ${error.message}`);
      }
    })
    .filter((workflow) => workflow.filter);
}

// ------------------------------------------------------------------ verdict

/**
 * Judge one PR head from its changed files, workflows and runs.
 *
 * `workflows` is `[{ path, filter }]` from loadWorkflows. `runs` maps a
 * workflow path to `{ status, conclusion, jobs: [{ name, conclusion }] }`.
 * `startClosed` is true once a run that has not been created will not be:
 * a matched workflow with no run then fails instead of being pending.
 * `final` is true once the caller stops waiting: an unfinished run then fails
 * too. `uncoveredBy` (default `workflows`) are the workflows whose filters
 * count a file as covered. Returns `{ state, reason, rows, uncovered }` with
 * state "pass", "fail" or "pending" and a one-line reason.
 */
export function decide({ files, workflows, runs, startClosed = false, final = false, uncoveredBy = workflows }) {
  const closed = startClosed || final;
  const byPath = new Map(workflows.map((workflow) => [workflow.path, workflow]));
  const expected = new Set(workflows
    .filter((workflow) => filterMatches(workflow.filter, files))
    .map((workflow) => workflow.path));
  const paths = [...new Set([...expected, ...Object.keys(runs)])].sort();
  const problems = [];
  const pending = [];
  const rows = [];
  let passed = 0;
  let skipped = 0;
  let advisory = 0;
  for (const path of paths) {
    const run = runs[path];
    const filtered = isPathFiltered(byPath.get(path)?.filter);
    if (!run) {
      if (closed) problems.push(`${path} matched the change but did not start`);
      else pending.push(path);
      rows.push({ path, verdict: closed ? 'did not start' : 'waiting to start' });
      continue;
    }
    if (run.status !== 'completed') {
      if (final) problems.push(`${path} did not finish before the gate stopped waiting`);
      else pending.push(path);
      rows.push({ path, verdict: run.status });
      continue;
    }
    const jobs = run.jobs ?? [];
    const bad = jobs.filter((job) => !passingConclusions.has(job.conclusion));
    const ran = jobs.filter((job) => job.conclusion !== 'skipped').length;
    passed += jobs.filter((job) => job.conclusion === 'success').length;
    skipped += jobs.length - ran;
    let verdict = `${ran} ran, ${jobs.length - ran} skipped`;
    if (!passingConclusions.has(run.conclusion)) {
      // The run's conclusion is GitHub's verdict; the jobs only name the cause.
      verdict = bad.length > 0
        ? bad.map((job) => `${job.name}: ${job.conclusion}`).join(', ')
        : `run concluded ${run.conclusion} with no failed job`;
      problems.push(`${path}: ${verdict}`);
    } else if (filtered && ran === 0) {
      verdict = 'matched the change but every job skipped';
      problems.push(`${path} ${verdict}`);
    } else if (bad.length > 0) {
      // A green run around a failed job: continue-on-error, advisory only.
      advisory += bad.length;
      verdict += `; advisory: ${bad.map((job) => `${job.name}: ${job.conclusion}`).join(', ')}`;
    }
    rows.push({ path, verdict });
  }
  const uncovered = uncoveredFiles(uncoveredBy, files);
  if (problems.length > 0) {
    const more = problems.length > 1 ? ` (+${problems.length - 1} more)` : '';
    return { state: 'fail', reason: `${problems[0]}${more}`, rows, uncovered };
  }
  if (pending.length > 0) {
    return { state: 'pending', reason: `waiting for ${pending.join(', ')}`, rows, uncovered };
  }
  const tests = paths.filter((path) => isPathFiltered(byPath.get(path)?.filter)).length;
  const notes = advisory > 0 ? `, ${advisory} advisory failed` : '';
  return {
    state: 'pass',
    reason: `${paths.length} workflows (${tests} path-filtered), ${passed} jobs passed, ${skipped} skipped${notes}`,
    rows,
    uncovered,
  };
}

// --------------------------------------------------------------- GitHub API

/** A read the gate waits out (5xx, rate limit, network): not a verdict. */
export class TransientApiError extends Error {}

/** A GitHub REST client over fetch: `get(resource)` and `post(resource, body)`. */
export function fetchClient({ fetchImpl = fetch, api, token }) {
  const call = async (method, resource, body) => {
    let response;
    try {
      response = await fetchImpl(`${api}${resource}`, {
        method,
        headers: {
          Accept: 'application/vnd.github+json',
          Authorization: `Bearer ${token}`,
          'X-GitHub-Api-Version': '2022-11-28',
          ...(body ? { 'Content-Type': 'application/json' } : {}),
        },
        ...(body ? { body: JSON.stringify(body) } : {}),
      });
    } catch (error) {
      throw new TransientApiError(`${method} ${resource}: ${error.message}`);
    }
    if (response.ok) return response.json();
    const limited = response.status === 429 || (response.status === 403 &&
      (response.headers?.get?.('x-ratelimit-remaining') === '0' || response.headers?.get?.('retry-after')));
    const message = `${method} ${resource}: HTTP ${response.status}`;
    if (response.status >= 500 || limited) throw new TransientApiError(message);
    throw new Error(message);
  };
  return { get: (resource) => call('GET', resource), post: (resource, body) => call('POST', resource, body) };
}

/** A read-only client through the local `gh` login, for `verdict --dry-run`. */
function ghClient() {
  return {
    get: async (resource) => {
      const result = spawnSync('gh', ['api', resource.replace(/^\//, '')], { encoding: 'utf8', maxBuffer: 64 << 20 });
      if (result.status !== 0) throw new TransientApiError(`gh api ${resource}: ${result.stderr.trim()}`);
      return JSON.parse(result.stdout);
    },
    post: async () => { throw new Error('dry run never posts'); },
  };
}

/** List the PR's changed file names (GitHub returns at most 3000). */
export async function changedFiles({ get, repo, pr }) {
  const files = [];
  for (let page = 1; page <= 30; page += 1) {
    const batch = await get(`/repos/${repo}/pulls/${pr}/files?per_page=100&page=${page}`);
    files.push(...batch.map((file) => file.filename));
    if (batch.length < 100) break;
  }
  return files;
}

/**
 * Collect the latest pull_request run of each workflow at one PR head.
 *
 * Runs are matched on both head SHA and head branch, so two PRs on one SHA
 * do not mix. Each run reports its latest attempt; finished runs carry that
 * attempt's jobs.
 */
export async function collectRuns({ get, repo, sha, branch }) {
  const listed = await allPages(get, `/repos/${repo}/actions/runs?head_sha=${sha}` +
    `&branch=${encodeURIComponent(branch)}&event=pull_request`, 'workflow_runs');
  const latest = new Map();
  for (const run of listed) {
    const path = run.path.split('@')[0];
    if (run.head_sha !== sha || run.head_branch !== branch || run.event !== 'pull_request') continue;
    if (!latest.has(path) || run.id > latest.get(path).id) latest.set(path, run);
  }
  const runs = {};
  for (const [path, run] of latest) {
    const entry = { name: run.name, status: run.status, conclusion: run.conclusion };
    if (run.status === 'completed') {
      const jobs = await allPages(get, `/repos/${repo}/actions/runs/${run.id}/jobs?filter=latest`, 'jobs');
      entry.jobs = jobs.map((job) => ({ name: job.name, conclusion: job.conclusion }));
    }
    runs[path] = entry;
  }
  return runs;
}

/** Read every page of a listing whose items sit under `key` beside `total_count`. */
async function allPages(get, resource, key) {
  const items = [];
  for (let page = 1; page <= 50; page += 1) {
    const body = await get(`${resource}&per_page=100&page=${page}`);
    items.push(...body[key]);
    if (body[key].length < 100 || items.length >= body.total_count) return items;
  }
  throw new Error(`${resource}: more than 5000 items`);
}

/**
 * Split the pull_request workflows by the gate's watch list (names).
 *
 * Returns `{ gated, unwatched }`: the workflows the gate judges, and the
 * ones it should judge but does not watch (out-of-scope ones are in neither).
 */
export function gateScope(workflows, watched) {
  const inScope = workflows.filter((workflow) => !outOfScope.has(workflow.path));
  return {
    gated: inScope.filter((workflow) => watched.has(workflow.name)),
    unwatched: inScope.filter((workflow) => !watched.has(workflow.name)),
  };
}

/** Call `read` until it returns, waiting out transient API errors until `deadline`. */
async function readUntil(read, { now, sleep, deadline, pollMs }) {
  for (;;) {
    try {
      return await read();
    } catch (error) {
      if (!(error instanceof TransientApiError) || now() >= deadline) throw error;
      console.log(`API read failed, waiting: ${error.message}`);
      await sleep(pollMs);
    }
  }
}

/**
 * The PR half: check that every workflow the change matches has started.
 *
 * Polls until each matched workflow has a run (pass) or `graceMs` passes
 * (fail: it will not start). Transient API errors are waited out until
 * `deadlineMs`. `workflows` is every pull_request workflow and `watched` the
 * gate's watch list. Returns `{ state, reason, uncovered, unwatched }`:
 * `unwatched` lists the path-filtered workflows that match the change but
 * that the gate does not watch, a warning.
 */
export async function startCheck({
  client, repo, pr, sha, branch, workflows, watched,
  now = Date.now, sleep = defaultSleep, graceMs = 90_000, deadlineMs = 300_000, pollMs = 10_000,
}) {
  const started = now();
  const timing = { now, sleep, deadline: started + deadlineMs, pollMs };
  const files = await readUntil(() => changedFiles({ get: client.get, repo, pr }), timing);
  const { gated, unwatched } = gateScope(workflows, watched);
  const expected = gated.filter((workflow) => filterMatches(workflow.filter, files));
  const warnings = {
    uncovered: uncoveredFiles(workflows, files),
    unwatched: unwatched.filter((workflow) => isPathFiltered(workflow.filter) && filterMatches(workflow.filter, files))
      .map((workflow) => workflow.path),
  };
  for (;;) {
    const runs = await readUntil(() => collectRuns({ get: client.get, repo, sha, branch }), timing);
    const missing = expected.filter((workflow) => !runs[workflow.path]).map((workflow) => workflow.path);
    if (missing.length === 0) {
      return { state: 'pass', reason: `${expected.length} matched workflows started`, ...warnings };
    }
    if (now() - started >= graceMs) {
      return { state: 'fail', reason: `matched the change but did not start: ${missing.join(', ')}`, ...warnings };
    }
    await sleep(pollMs);
  }
}

/**
 * The workflow_run half: recompute the head's verdict and post it.
 *
 * Skips (state "stale") a head that is no longer its PR's head: the newer
 * head has its own runs. Every run of the head exists by the time any of
 * them completes, so a missing run fails at once. Only workflows named in
 * `watched` are judged: every judged run's completion wakes the verdict
 * again, so a pending status is always replaced. Returns the decision plus
 * the posted `status`.
 */
export async function verdict({
  client, repo, sha, branch, pr, workflows, targetUrl, post = true, watched,
  now = Date.now, sleep = defaultSleep, deadlineMs = 300_000, pollMs = 15_000,
}) {
  const timing = { now, sleep, deadline: now() + deadlineMs, pollMs };
  let number = pr;
  if (!number) {
    const pulls = await readUntil(() => client.get(`/repos/${repo}/commits/${sha}/pulls`), timing);
    number = pulls.find((pull) => pull.head?.sha === sha && pull.head?.ref === branch)?.number;
    if (!number) return { state: 'stale', reason: `no open PR has head ${sha} on ${branch}` };
  }
  const pull = await readUntil(() => client.get(`/repos/${repo}/pulls/${number}`), timing);
  if (pull.head?.sha !== sha) {
    return { state: 'stale', reason: `PR #${number} moved on from ${sha} to ${pull.head?.sha}` };
  }
  const files = await readUntil(() => changedFiles({ get: client.get, repo, pr: number }), timing);
  const { gated } = gateScope(workflows, watched);
  const listed = await readUntil(() => collectRuns({ get: client.get, repo, sha, branch }), timing);
  const runs = Object.fromEntries(Object.entries(listed).filter(([, run]) => watched.has(run.name)));
  const result = decide({ files, workflows: gated, runs, startClosed: true, uncoveredBy: workflows });
  const uncovered = result.uncovered.length > 0 ? `; ${result.uncovered.length} paths no test workflow covers` : '';
  const status = {
    state: { pass: 'success', fail: 'failure', pending: 'pending' }[result.state],
    context: statusContext,
    description: `${result.reason}${uncovered}`.slice(0, 140),
    ...(targetUrl ? { target_url: targetUrl } : {}),
  };
  if (post) await readUntil(() => client.post(`/repos/${repo}/statuses/${sha}`, status), timing);
  return { ...result, pr: number, status };
}

// ---------------------------------------------------------------------- CLI

function defaultSleep(ms) {
  return new Promise((resolve) => { setTimeout(resolve, ms); });
}

/** Escape text for a GitHub workflow command's message (%, CR, LF). */
export function commandText(text) {
  return text.replace(/%/g, '%25').replace(/\r/g, '%0D').replace(/\n/g, '%0A');
}

/** Print one list as a GitHub warning annotation. */
function warnList(label, items) {
  if (!items?.length) return;
  const shown = items.slice(0, 10).join(', ');
  const more = items.length > 10 ? ` (+${items.length - 10} more)` : '';
  console.log(`::warning title=CI gate::${commandText(`${label}: ${shown}${more}`)}`);
}

function writeSummary(summary, title, result) {
  if (!summary) return;
  const lines = [`### ${title}: ${result.state}`, '', result.reason, ''];
  if (result.rows) {
    lines.push('| Workflow | Result |', '| --- | --- |',
      ...result.rows.map((row) => `| \`${row.path}\` | ${row.verdict} |`), '');
  }
  if (result.uncovered?.length) lines.push(`Not covered by any test workflow: ${result.uncovered.join(', ')}`, '');
  appendFileSync(summary, `${lines.join('\n')}\n`);
}

/** The workflow names ci-gate.yml watches. */
function watchList() {
  return new Set(parseWatchedWorkflows(readFileSync(join(workflowDir, 'ci-gate.yml'), 'utf8')));
}

function required(names) {
  for (const name of names) if (!process.env[name]) throw new Error(`${name} is required`);
}

/** Run one CLI command; returns the process exit code. */
async function main([command, ...args]) {
  const env = process.env;
  const summary = env.GITHUB_STEP_SUMMARY;
  const workflows = loadWorkflows();
  if (command === 'start') {
    required(['GITHUB_TOKEN', 'GITHUB_REPOSITORY', 'PR_NUMBER', 'HEAD_SHA', 'HEAD_BRANCH']);
    const client = fetchClient({ api: env.GITHUB_API_URL ?? 'https://api.github.com', token: env.GITHUB_TOKEN });
    const result = await startCheck({
      client, repo: env.GITHUB_REPOSITORY, pr: env.PR_NUMBER, sha: env.HEAD_SHA, branch: env.HEAD_BRANCH, workflows,
      watched: watchList(),
    });
    warnList('changed paths no test workflow covers', result.uncovered);
    warnList('path-filtered workflows this change starts that ci-gate.yml does not watch', result.unwatched);
    writeSummary(summary, 'CI gate start', result);
    console.log(`CI gate start ${result.state}: ${result.reason}`);
    return result.state === 'pass' ? 0 : 1;
  }
  if (command === 'verdict') {
    const dryRun = args.includes('--dry-run');
    required(dryRun ? ['GITHUB_REPOSITORY', 'HEAD_SHA', 'HEAD_BRANCH']
      : ['GITHUB_TOKEN', 'GITHUB_REPOSITORY', 'HEAD_SHA', 'HEAD_BRANCH']);
    const client = dryRun ? ghClient()
      : fetchClient({ api: env.GITHUB_API_URL ?? 'https://api.github.com', token: env.GITHUB_TOKEN });
    const targetUrl = env.GITHUB_RUN_ID
      ? `${env.GITHUB_SERVER_URL ?? 'https://github.com'}/${env.GITHUB_REPOSITORY}/actions/runs/${env.GITHUB_RUN_ID}` : null;
    const result = await verdict({
      client, repo: env.GITHUB_REPOSITORY, sha: env.HEAD_SHA, branch: env.HEAD_BRANCH, pr: env.PR_NUMBER || null,
      workflows, targetUrl, post: !dryRun, watched: watchList(),
    });
    warnList('changed paths no test workflow covers', result.uncovered);
    writeSummary(summary, 'CI gate', result);
    console.log(`CI gate ${result.state}: ${result.reason}`);
    for (const row of result.rows ?? []) console.log(`  ${row.path}: ${row.verdict}`);
    if (result.status) console.log(`${dryRun ? 'would post' : 'posted'} ${result.status.state}: ${result.status.description}`);
    // The status carries the verdict; this job fails only when it could not post one.
    return 0;
  }
  console.error('Usage: node quality/ci-gate.mjs start | verdict [--dry-run]');
  return 2;
}

if (process.argv[1] === fileURLToPath(import.meta.url)) {
  main(process.argv.slice(2)).then((code) => { process.exitCode = code; }, (error) => {
    console.error(`CI gate error: ${error.message}`);
    process.exitCode = 1;
  });
}
