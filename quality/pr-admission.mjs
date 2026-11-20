#!/usr/bin/env node
// Select and run the small deterministic check set for one pull request.

import { spawnSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import {
  appendFileSync,
  readFileSync,
  writeFileSync,
} from 'node:fs';
import { resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = fileURLToPath(new URL('../', import.meta.url));
const defaultPolicyPath = fileURLToPath(
  new URL('./pr-admission-policy.json', import.meta.url),
);
const schema = 'rosie_pr_admission_policy_v1';
const resultSchema = 'rosie_pr_admission_result_v1';

const checkRegistry = Object.freeze({
  'diff-integrity': {
    description: 'Reject whitespace errors in the exact pull request diff.',
    failureClass: 'contract',
    command: ({ base, head }) => ['git', ['diff', '--check', base, head], root],
  },
  'admission-contract': {
    description: 'Test policy validation, path selection, and shadow behavior.',
    failureClass: 'contract',
    command: () => [
      process.execPath,
      ['--test', 'quality/pr-admission.test.mjs'],
      root,
    ],
  },
  'workflow-syntax': {
    description: 'Validate every GitHub Actions workflow with pinned actionlint.',
    failureClass: 'contract',
    command: () => [
      process.execPath,
      ['quality/quality.mjs', 'workflows'],
      root,
    ],
  },
  'build1-contract': {
    description: 'Run the fixed build host worker contract suite.',
    failureClass: 'contract',
    command: () => [
      'python3',
      ['-m', 'unittest', 'discover', '-s', 'tests', '-v'],
      resolve(root, 'tools/build1-release-worker'),
    ],
  },
});

/** Load and parse one versioned admission policy. */
export function loadAdmissionPolicy(path = defaultPolicyPath) {
  return JSON.parse(readFileSync(path, 'utf8'));
}

function assertStringArray(value, field) {
  if (!Array.isArray(value) || value.some((item) => typeof item !== 'string')) {
    throw new Error(`${field} must be an array of strings`);
  }
}

function validateCheckIds(checks, field) {
  assertStringArray(checks, field);
  for (const check of checks) {
    if (!Object.hasOwn(checkRegistry, check)) {
      throw new Error(`${field} names unknown check ${check}`);
    }
  }
}

/** Validate policy shape and every reference to executable code. */
export function validateAdmissionPolicy(policy) {
  if (!policy || typeof policy !== 'object' || Array.isArray(policy)) {
    throw new Error('policy must be an object');
  }
  if (policy.schema !== schema) {
    throw new Error(`policy schema must be ${schema}`);
  }
  if (!['shadow', 'enforce'].includes(policy.mode)) {
    throw new Error('policy mode must be shadow or enforce');
  }
  validateCheckIds(policy.always_checks, 'always_checks');
  if (!Array.isArray(policy.rules) || policy.rules.length === 0) {
    throw new Error('policy rules must be a nonempty array');
  }

  const ids = new Set();
  for (const [index, rule] of policy.rules.entries()) {
    const field = `rules[${index}]`;
    if (!rule || typeof rule !== 'object' || Array.isArray(rule)) {
      throw new Error(`${field} must be an object`);
    }
    if (typeof rule.id !== 'string' || !rule.id) {
      throw new Error(`${field}.id must be a nonempty string`);
    }
    if (ids.has(rule.id)) {
      throw new Error(`duplicate rule id ${rule.id}`);
    }
    ids.add(rule.id);
    if (!['mapped', 'pending'].includes(rule.proof)) {
      throw new Error(`${field}.proof must be mapped or pending`);
    }
    validateCheckIds(rule.checks, `${field}.checks`);

    const matchers = ['paths', 'prefixes', 'suffixes'];
    let hasMatcher = false;
    for (const matcher of matchers) {
      if (rule[matcher] === undefined) continue;
      assertStringArray(rule[matcher], `${field}.${matcher}`);
      hasMatcher ||= rule[matcher].length > 0;
    }
    if (!hasMatcher) {
      throw new Error(`${field} must define at least one path matcher`);
    }
  }
  return policy;
}

function normalizeChangedPaths(paths) {
  assertStringArray(paths, 'changed paths');
  const normalized = new Set();
  for (const path of paths) {
    if (
      !path ||
      path.includes('\0') ||
      path.startsWith('/') ||
      path.split('/').includes('..')
    ) {
      throw new Error(`invalid changed path ${JSON.stringify(path)}`);
    }
    normalized.add(path);
  }
  return [...normalized].sort();
}

function matchesRule(path, rule) {
  return (
    (rule.paths ?? []).includes(path) ||
    (rule.prefixes ?? []).some((prefix) => path.startsWith(prefix)) ||
    (rule.suffixes ?? []).some((suffix) => path.endsWith(suffix))
  );
}

/** Create a deterministic admission plan from changed repository paths. */
export function createAdmissionPlan(policy, changedPaths) {
  validateAdmissionPolicy(policy);
  const paths = normalizeChangedPaths(changedPaths);
  const checks = [...policy.always_checks];
  const matchedRules = [];
  const pendingProofPaths = [];
  const unknownPaths = [];

  for (const path of paths) {
    const rule = policy.rules.find((candidate) => matchesRule(path, candidate));
    if (!rule) {
      unknownPaths.push(path);
      continue;
    }
    if (!matchedRules.includes(rule.id)) matchedRules.push(rule.id);
    if (rule.proof === 'pending') pendingProofPaths.push(path);
    for (const check of rule.checks) {
      if (!checks.includes(check)) checks.push(check);
    }
  }

  const emptyDiff = paths.length === 0;
  return {
    changedPaths: paths,
    matchedRules,
    checks,
    pendingProofPaths,
    unknownPaths,
    emptyDiff,
    wouldBlockBeforeChecks:
      emptyDiff || pendingProofPaths.length > 0 || unknownPaths.length > 0,
  };
}

function changedPathsFromGit(base, head) {
  const result = spawnSync(
    'git',
    ['diff', '--name-only', '-z', base, head],
    { cwd: root, encoding: 'utf8', maxBuffer: 16 * 1024 * 1024 },
  );
  if (result.error || result.status !== 0) {
    const detail = result.error?.message ?? result.stderr.trim();
    throw new Error(`cannot read changed paths: ${detail}`);
  }
  return result.stdout.split('\0').filter(Boolean);
}

/** Resolve one safe revision name to an exact commit SHA. */
export function resolveGitCommit(revision) {
  if (
    typeof revision !== 'string' ||
    !/^[A-Za-z0-9][A-Za-z0-9._/@{}^~:-]*$/.test(revision)
  ) {
    throw new Error(`unsafe Git revision ${JSON.stringify(revision)}`);
  }
  const result = spawnSync(
    'git',
    ['rev-parse', '--verify', `${revision}^{commit}`],
    { cwd: root, encoding: 'utf8', maxBuffer: 1024 * 1024 },
  );
  const commit = result.stdout.trim();
  if (result.error || result.status !== 0 || !/^[0-9a-f]{40,64}$/.test(commit)) {
    const detail = result.error?.message ?? result.stderr.trim();
    throw new Error(`cannot resolve Git revision ${revision}: ${detail}`);
  }
  return commit;
}

function emitCheckOutput(result) {
  if (result.stdout) process.stdout.write(result.stdout);
  if (result.stderr) process.stderr.write(result.stderr);
}

function runCheck(id, context) {
  const definition = checkRegistry[id];
  const [command, args, cwd] = definition.command(context);
  const start = performance.now();
  const result = spawnSync(command, args, {
    cwd,
    encoding: 'utf8',
    env: { ...process.env, CI: '1' },
    maxBuffer: 32 * 1024 * 1024,
    timeout: 10 * 60 * 1000,
  });
  emitCheckOutput(result);

  let status = 'passed';
  let failureClass = null;
  if (result.error) {
    status = 'unavailable';
    failureClass = 'runner';
  } else if (result.status !== 0) {
    status = 'failed';
    failureClass = definition.failureClass;
  }
  return {
    id,
    description: definition.description,
    status,
    failureClass,
    exitCode: result.status,
    signal: result.signal,
    durationMs: Math.round(performance.now() - start),
  };
}

function markdownCell(value) {
  return String(value).replaceAll('|', '\\|').replaceAll('\n', ' ');
}

function pathList(title, paths) {
  if (paths.length === 0) return '';
  return `\n### ${title}\n\n${paths.map((path) => `- \`${path}\``).join('\n')}\n`;
}

/** Format a human-readable admission result for GitHub and local use. */
export function formatAdmissionSummary({ mode, plan, results, wouldBlock }) {
  const verdict = wouldBlock ? 'would block' : 'would admit';
  const lines = [
    '# PR admission',
    '',
    `Mode: \`${mode}\``,
    '',
    mode === 'shadow'
      ? `Shadow verdict: ${verdict}. This result does not block merge.`
      : `Enforced verdict: ${wouldBlock ? 'blocked' : 'admitted'}.`,
    '',
    `Changed paths: ${plan.changedPaths.length}`,
    '',
    `Matched rules: ${plan.matchedRules.length ? plan.matchedRules.map((id) => `\`${id}\``).join(', ') : 'none'}`,
    '',
    '| Check | Status | Failure class | Duration |',
    '| --- | --- | --- | ---: |',
  ];
  for (const result of results) {
    lines.push(
      `| ${markdownCell(result.id)} | ${markdownCell(result.status)} | ${markdownCell(result.failureClass ?? '')} | ${(result.durationMs / 1000).toFixed(2)} s |`,
    );
  }
  if (results.length === 0) lines.push('| none | not run | | 0.00 s |');
  if (plan.emptyDiff) lines.push('', 'No changed paths were found. The policy fails closed.');
  lines.push(pathList('Proof mapping pending', plan.pendingProofPaths));
  lines.push(pathList('Unknown paths', plan.unknownPaths));
  return `${lines.join('\n')}\n`;
}

/** Return the process status for one admission verdict. */
export function admissionExitCode(mode, wouldBlock) {
  return mode === 'enforce' && wouldBlock ? 1 : 0;
}

function parseArguments(argv) {
  const [command = 'run', ...rest] = argv;
  if (!['plan', 'run'].includes(command)) {
    throw new Error('usage: pr-admission.mjs plan|run --base SHA --head SHA');
  }
  const options = { command, policy: defaultPolicyPath };
  for (let index = 0; index < rest.length; index += 2) {
    const flag = rest[index];
    const value = rest[index + 1];
    if (!flag?.startsWith('--') || value === undefined) {
      throw new Error(`invalid argument ${flag ?? ''}`);
    }
    const key = flag.slice(2).replaceAll('-', '_');
    if (!['base', 'head', 'policy', 'result', 'summary', 'paths_json'].includes(key)) {
      throw new Error(`unknown argument ${flag}`);
    }
    options[key] = value;
  }
  return options;
}

function readChangedPaths(options) {
  if (options.paths_json) {
    return JSON.parse(readFileSync(options.paths_json, 'utf8'));
  }
  if (!options.base || !options.head) {
    throw new Error('--base and --head are required without --paths-json');
  }
  return changedPathsFromGit(
    resolveGitCommit(options.base),
    resolveGitCommit(options.head),
  );
}

function policySha256(path) {
  return createHash('sha256').update(readFileSync(path)).digest('hex');
}

function writeSummary(path, summary) {
  if (!path) return;
  appendFileSync(path, summary);
}

async function main(argv) {
  const options = parseArguments(argv);
  const policyPath = resolve(options.policy);
  const policy = validateAdmissionPolicy(loadAdmissionPolicy(policyPath));
  const plan = createAdmissionPlan(policy, readChangedPaths(options));

  if (options.command === 'plan') {
    process.stdout.write(`${JSON.stringify(plan, null, 2)}\n`);
    return 0;
  }
  if (!options.base || !options.head) {
    throw new Error('--base and --head are required to run selected checks');
  }

  const context = {
    base: resolveGitCommit(options.base),
    head: resolveGitCommit(options.head),
  };
  const results = plan.checks.map((id) => runCheck(id, context));
  const checkFailure = results.some((result) => result.status !== 'passed');
  const wouldBlock = plan.wouldBlockBeforeChecks || checkFailure;
  const record = {
    schema: resultSchema,
    generatedAt: new Date().toISOString(),
    mode: policy.mode,
    policySha256: policySha256(policyPath),
    baseSha: context.base,
    headSha: context.head,
    plan,
    results,
    wouldBlock,
  };
  const summary = formatAdmissionSummary({
    mode: policy.mode,
    plan,
    results,
    wouldBlock,
  });
  process.stdout.write(summary);
  if (options.result) writeFileSync(options.result, `${JSON.stringify(record, null, 2)}\n`);
  writeSummary(options.summary, summary);
  return admissionExitCode(policy.mode, wouldBlock);
}

const invokedPath = process.argv[1] ? resolve(process.argv[1]) : '';
if (invokedPath === fileURLToPath(import.meta.url)) {
  main(process.argv.slice(2)).then(
    (status) => {
      process.exitCode = status;
    },
    (error) => {
      console.error(`admission_unavailable: ${error.message}`);
      process.exitCode = 2;
    },
  );
}
