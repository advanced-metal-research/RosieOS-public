/**
 * Validate the checked-in map from managed release suites to proof stages.
 *
 * The map records coverage. It does not execute the release suites.
 */

import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const moduleDirectory = dirname(fileURLToPath(import.meta.url));
const repositoryRoot = resolve(moduleDirectory, '..');
const defaultContractPath = resolve(moduleDirectory, 'release-test-coverage.json');
const allowedStages = new Set(['pull_request', 'build1_candidate', 'target_qualification']);
const allowedCoverage = new Set(['covered', 'partial', 'gap']);
// The build host's worker is site infrastructure the public mirror leaves out
// (quality/qualification.mjs refuses a candidate without it). A tree without it
// cannot read the worker's proof commands, so it skips only those; the full
// tree checks them.
const buildWorker = 'tools/build1-release-worker/';

/** Load a release test coverage contract from disk. */
export function loadReleaseTestCoverage(path = defaultContractPath) {
  return JSON.parse(readFileSync(path, 'utf8'));
}

/** Read the suite names from the managed release gate catalogue. */
export function readManagedGateSuites(root, source) {
  const text = readFileSync(resolve(root, source), 'utf8');
  return [...text.matchAll(/\{Name:\s*"([a-z0-9_]+)"/g)].map((match) => match[1]);
}

function workflowHasJob(text, job) {
  const jobsOffset = text.search(/^jobs:\s*$/m);
  if (jobsOffset < 0) {
    return false;
  }
  return new RegExp(`^  ${job}:\\s*$`, 'm').test(text.slice(jobsOffset));
}

function requireString(value, field) {
  if (typeof value !== 'string' || value.trim() === '') {
    throw new Error(`${field} must be a non-empty string`);
  }
}

/** Validate the contract and its references to release suites and workflows. */
export function validateReleaseTestCoverage(contract, root = repositoryRoot) {
  if (contract?.schema !== 'rosie_release_test_coverage_v1') {
    throw new Error('schema must be rosie_release_test_coverage_v1');
  }
  requireString(contract.catalog?.source, 'catalog.source');
  if (contract.catalog?.collection !== 'gateSuites') {
    throw new Error('catalog.collection must be gateSuites');
  }
  if (!Array.isArray(contract.suites) || contract.suites.length === 0) {
    throw new Error('suites must be a non-empty array');
  }

  const expected = readManagedGateSuites(root, contract.catalog.source);
  if (expected.length === 0) {
    throw new Error('managed release gate catalogue has no suites');
  }

  const seen = new Set();
  for (const suite of contract.suites) {
    requireString(suite.id, 'suite.id');
    if (seen.has(suite.id)) {
      throw new Error(`duplicate suite ${suite.id}`);
    }
    seen.add(suite.id);
    if (!allowedStages.has(suite.owner)) {
      throw new Error(`suite ${suite.id} has invalid owner ${suite.owner}`);
    }
    if (!allowedCoverage.has(suite.coverage)) {
      throw new Error(`suite ${suite.id} has invalid coverage ${suite.coverage}`);
    }
    if (!Array.isArray(suite.evidence)) {
      throw new Error(`suite ${suite.id} evidence must be an array`);
    }
    if (suite.coverage === 'covered' && suite.evidence.length === 0) {
      throw new Error(`covered suite ${suite.id} must name evidence`);
    }
    if (suite.coverage !== 'covered') {
      requireString(suite.gap, `suite ${suite.id} gap`);
    }
    for (const evidence of suite.evidence) {
      requireString(evidence.command, `suite ${suite.id} evidence.command`);
      const hasWorkflow = typeof evidence.workflow === 'string';
      const hasSource = typeof evidence.source === 'string';
      if (hasWorkflow === hasSource) {
        throw new Error(`suite ${suite.id} evidence must name one workflow or source`);
      }
      if (hasWorkflow) {
        requireString(evidence.workflow, `suite ${suite.id} evidence.workflow`);
        requireString(evidence.job, `suite ${suite.id} evidence.job`);
        const workflowPath = resolve(root, evidence.workflow);
        if (!existsSync(workflowPath)) {
          throw new Error(`suite ${suite.id} workflow does not exist: ${evidence.workflow}`);
        }
        const workflow = readFileSync(workflowPath, 'utf8');
        if (!workflowHasJob(workflow, evidence.job)) {
          throw new Error(`suite ${suite.id} workflow has no job ${evidence.job}`);
        }
        if (!workflow.includes(evidence.command)) {
          throw new Error(`suite ${suite.id} workflow does not contain command: ${evidence.command}`);
        }
        continue;
      }
      requireString(evidence.source, `suite ${suite.id} evidence.source`);
      if (suite.owner === 'build1_candidate' && evidence.source.startsWith(buildWorker) &&
          !existsSync(resolve(root, buildWorker))) {
        continue;
      }
      const sourcePath = resolve(root, evidence.source);
      if (!existsSync(sourcePath)) {
        throw new Error(`suite ${suite.id} source does not exist: ${evidence.source}`);
      }
      const source = readFileSync(sourcePath, 'utf8');
      if (!source.includes(evidence.command)) {
        throw new Error(`suite ${suite.id} source does not contain command: ${evidence.command}`);
      }
    }
  }

  const missing = expected.filter((suite) => !seen.has(suite));
  const extra = [...seen].filter((suite) => !expected.includes(suite));
  if (missing.length > 0 || extra.length > 0) {
    throw new Error(`suite catalogue mismatch: missing=[${missing}] extra=[${extra}]`);
  }
  return contract;
}

/** Format a short Markdown report for GitHub Actions. */
export function formatReleaseTestCoverage(contract) {
  const counts = { covered: 0, partial: 0, gap: 0 };
  for (const suite of contract.suites) {
    counts[suite.coverage] += 1;
  }
  const lines = [
    '## Managed release test coverage',
    '',
    `The managed gate has ${contract.suites.length} suites: ${counts.covered} covered, ${counts.partial} partial, and ${counts.gap} unmapped.`,
    '',
    '| Suite | Owner | Coverage |',
    '|---|---|---|',
  ];
  for (const suite of contract.suites) {
    lines.push(`| \`${suite.id}\` | \`${suite.owner}\` | \`${suite.coverage}\` |`);
  }
  return `${lines.join('\n')}\n`;
}

function parseArguments(arguments_) {
  let summaryPath = '';
  for (let index = 0; index < arguments_.length; index += 1) {
    if (arguments_[index] === '--summary') {
      summaryPath = arguments_[index + 1] ?? '';
      index += 1;
      continue;
    }
    throw new Error(`unknown argument ${arguments_[index]}`);
  }
  return { summaryPath };
}

function main() {
  const [command, ...arguments_] = process.argv.slice(2);
  if (command !== 'check') {
    throw new Error('usage: node quality/release-test-coverage.mjs check [--summary PATH]');
  }
  const { summaryPath } = parseArguments(arguments_);
  const contract = validateReleaseTestCoverage(loadReleaseTestCoverage());
  const report = formatReleaseTestCoverage(contract);
  if (summaryPath !== '') {
    writeFileSync(summaryPath, report, { flag: 'a' });
  }
  process.stdout.write(report);
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  try {
    main();
  } catch (error) {
    process.stderr.write(`release test coverage: ${error.message}\n`);
    process.exitCode = 1;
  }
}
