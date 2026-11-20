#!/usr/bin/env node
// Derive qualification requirements and scenario evidence from reviewed source.

import { spawnSync } from 'node:child_process';
import { createHash, randomUUID } from 'node:crypto';
import {
  closeSync,
  existsSync,
  mkdirSync,
  mkdtempSync,
  openSync,
  readFileSync,
  readSync,
  readdirSync,
  rmSync,
  statSync,
  writeFileSync,
} from 'node:fs';
import { tmpdir } from 'node:os';
import { basename, dirname, join, resolve } from 'node:path';
import { StringDecoder } from 'node:string_decoder';
import { fileURLToPath } from 'node:url';
import {
  loadReleaseTestCoverage,
  validateReleaseTestCoverage,
} from './release-test-coverage.mjs';

const root = fileURLToPath(new URL('../', import.meta.url));
const policyPath = fileURLToPath(new URL('./qualification-policy.json', import.meta.url));
const comparisonStartsPath = fileURLToPath(new URL('./qualification-comparison-starts.json', import.meta.url));
const schema = 'rosie.qualification-policy.v1';
const scenarioSchema = 'rosie.qualification-scenarios.v1';
const environments = ['source', 'simulation', 'servo_hil', 'robot', 'welding'];

function hash(value) {
  return createHash('sha256').update(value).digest('hex');
}

function canonical(value) {
  if (Array.isArray(value)) return '[' + value.map(canonical).join(',') + ']';
  if (value && typeof value === 'object') {
    return '{' + Object.keys(value).sort().map((key) =>
      JSON.stringify(key) + ':' + canonical(value[key])).join(',') + '}';
  }
  return JSON.stringify(value);
}

function command(program, args, cwd = root, limit = 16 * 1024 * 1024) {
  const result = spawnSync(program, args, {
    cwd, encoding: 'utf8', maxBuffer: limit, timeout: 15 * 60 * 1000,
  });
  if (result.error?.code === 'ENOBUFS') {
    throw new Error(program + ' output exceeds ' + limit / (1024 * 1024) + ' MiB');
  }
  if (result.error || result.status !== 0) throw commandFailure(program, result);
  return result.stdout;
}

// Name every way a command can fail: a spawn error, a signal (status is null
// when git is killed, often with no stderr), or a nonzero status.
function commandFailure(program, result) {
  const detail = [
    result.error?.message,
    result.signal ? 'signal ' + result.signal : 'exit status ' + result.status,
    typeof result.stderr === 'string' ? result.stderr.trim() : '',
  ].filter(Boolean).join('; ');
  return new Error(program + ' failed: ' + detail);
}

// Write a command's standard output to a file instead of spawnSync's output
// buffer. The unified diff goes this way: a retirement that deleted about 2.2
// million lines (2,778 files) made a 98 MiB diff that overran the 32 MiB
// buffer (spawnSync ENOBUFS), and any fixed buffer, even the later 256 MiB
// one, makes the plan unavailable for a diff past it. A failed command
// throws, so a diff is never classified from a partial file.
export function commandToFile(program, args, path, cwd = root) {
  const output = openSync(path, 'w');
  let result;
  try {
    result = spawnSync(program, args, {
      cwd, encoding: 'utf8', stdio: ['ignore', output, 'pipe'],
      // Only stderr is buffered: git's warnings, 0 bytes on the 101 MiB
      // comparison diff and 172 bytes (a rename-limit warning) on a 143 MiB
      // one. An overflow is a spawn error and throws below.
      maxBuffer: 16 * 1024 * 1024, timeout: 15 * 60 * 1000,
    });
  } finally {
    closeSync(output);
  }
  if (result.error || result.status !== 0) throw commandFailure(program, result);
}

// Yield a file's lines as String.prototype.split('\n') would, holding one read
// chunk and one line in memory, so a diff larger than V8's string limit
// (512 MiB) still scans. Measured 2026-11-20 on macOS, node 22.13: a 750 MiB
// diff planned in 3.4 s at 302 MB peak RSS, and a 101 MiB diff from the
// reviewed comparison start gave a plan byte-identical to the whole-string
// reader's in the same 2.8 s. The decoder keeps a UTF-8 character that a
// chunk boundary splits whole. Only the new chunk is searched for '\n', so a
// long line costs linear time.
//
// 1 MiB reads: one pass over a 300 MiB diff took 57-68 ms with 64 KiB reads,
// 69 ms with 1 MiB and 72 ms with 8 MiB (same Mac), far below the
// classifier's regex cost, so the size only bounds the buffer's memory. A
// 100 MiB single line takes 25 ms.
function* fileLines(path, chunkBytes = 1024 * 1024) {
  const input = openSync(path, 'r');
  try {
    const decoder = new StringDecoder('utf8');
    const buffer = Buffer.alloc(chunkBytes);
    let pending = '';
    for (;;) {
      const count = readSync(input, buffer, 0, buffer.length, null);
      const text = count > 0 ? decoder.write(buffer.subarray(0, count)) : decoder.end();
      let start = 0;
      for (let end = text.indexOf('\n'); end !== -1; end = text.indexOf('\n', start)) {
        yield pending + text.slice(start, end);
        pending = '';
        start = end + 1;
      }
      pending += text.slice(start);
      if (count === 0) break;
    }
    yield pending;
  } finally {
    closeSync(input);
  }
}

// A diff on disk that each iteration reads again from the start: the
// classifier makes one pass for deletions and one for changed text.
export function diffFileLines(path, chunkBytes) {
  return { [Symbol.iterator]: () => fileLines(path, chunkBytes) };
}

function exactCommit(revision) {
  if (typeof revision !== 'string' ||
      !/^[A-Za-z0-9][A-Za-z0-9._/@{}^~:-]*$/.test(revision)) {
    throw new Error('unsafe Git revision');
  }
  const commit = command('git', ['rev-parse', '--verify', revision + '^{commit}']).trim();
  if (!/^[0-9a-f]{40,64}$/.test(commit)) throw new Error('invalid resolved commit');
  return commit;
}

function nonemptyString(value, name) {
  if (typeof value !== 'string' || !value.trim()) throw new Error(name + ' must be a string');
}

function stringArray(value, name) {
  if (!Array.isArray(value) || value.some((item) => typeof item !== 'string' || !item)) {
    throw new Error(name + ' must be a string array');
  }
}

export function loadPolicy(path = policyPath) {
  const raw = readFileSync(path);
  const policy = JSON.parse(raw);
  if (policy.schema !== schema || policy.mode !== 'shadow' ||
      canonical(policy.environments) !== canonical(environments)) {
    throw new Error('qualification policy schema, mode, or environments differ');
  }
  for (const group of ['classes', 'modifiers']) {
    if (!policy[group] || typeof policy[group] !== 'object') {
      throw new Error('missing policy ' + group);
    }
    for (const [name, requirements] of Object.entries(policy[group])) {
      nonemptyString(name, group + ' name');
      if (!Array.isArray(requirements)) throw new Error('invalid requirements for ' + name);
      for (const item of requirements) {
        nonemptyString(item.claim, 'claim');
        nonemptyString(item.reason, 'reason');
        if (!environments.includes(item.environment)) {
          throw new Error('unknown requirement environment');
        }
      }
    }
  }
  if (!Array.isArray(policy.classification_rules)) throw new Error('missing classification rules');
  for (const rule of policy.classification_rules) {
    nonemptyString(rule.id, 'rule ID');
    for (const name of rule.classes ?? []) {
      if (!Object.hasOwn(policy.classes, name)) throw new Error('unknown class ' + name);
    }
    for (const name of rule.modifiers ?? []) {
      if (!Object.hasOwn(policy.modifiers, name)) throw new Error('unknown modifier ' + name);
    }
    if (rule.scan_diff_impact !== undefined &&
        typeof rule.scan_diff_impact !== 'boolean') {
      throw new Error('invalid diff impact rule: ' + rule.id);
    }
    if (rule.deleted_only !== undefined && typeof rule.deleted_only !== 'boolean') {
      throw new Error('invalid deleted-only rule: ' + rule.id);
    }
    if (rule.path_patterns !== undefined) {
      stringArray(rule.path_patterns, 'path patterns');
      rule.path_patterns.forEach((pattern) => new RegExp(pattern));
    }
    if (!['paths', 'prefixes', 'suffixes', 'path_patterns'].some((key) => rule[key]?.length)) {
      throw new Error('classification rule has no matcher: ' + rule.id);
    }
  }
  for (const [name, patterns] of Object.entries(policy.diff_modifiers ?? {})) {
    if (!Object.hasOwn(policy.modifiers, name)) throw new Error('unknown diff modifier ' + name);
    stringArray(patterns, 'diff patterns');
    patterns.forEach((pattern) => new RegExp(pattern, 'i'));
  }
  for (const [name, patterns] of Object.entries(policy.diff_classes ?? {})) {
    if (!Object.hasOwn(policy.classes, name)) throw new Error('unknown diff class ' + name);
    stringArray(patterns, 'diff class patterns');
    patterns.forEach((pattern) => new RegExp(pattern, 'i'));
  }
  stringArray(policy.beta_floor, 'beta floor');
  stringArray(policy.install_floor?.required_workflows, 'install workflows');
  nonemptyString(policy.install_floor?.observation_scenario_id,
    'install observation scenario');
  if (!Number.isInteger(policy.install_floor.release_suite_count) ||
      !Number.isInteger(policy.install_floor.build1_suite_count) ||
      policy.install_floor.release_suite_count <= 0 ||
      policy.install_floor.build1_suite_count <= 0) {
    throw new Error('install suite counts must be positive integers');
  }
  if (new Set(policy.install_floor.required_workflows).size !==
      policy.install_floor.required_workflows.length) {
    throw new Error('duplicate install workflow');
  }
  stringArray(policy.exceptions?.required_fields, 'exception fields');
  return { policy, digest: hash(raw) };
}

function validPath(path) {
  if (typeof path !== 'string' || !path || path.startsWith('/') ||
      path.includes('\0') || path.split('/').includes('..')) {
    throw new Error('invalid repository path');
  }
  return path;
}

function listedPaths() {
  return [...new Set(command('git',
    ['ls-files', '--cached', '--others', '--exclude-standard', '-z'])
    .split('\0').filter(Boolean))].sort();
}

export function discoverScenarios(paths = listedPaths()) {
  const scenarios = [];
  const ids = new Set();
  for (const file of paths.filter((path) => path.endsWith('.scenarios.json')).sort()) {
    validPath(file);
    const declaration = JSON.parse(readFileSync(resolve(root, file)));
    if (declaration.schema !== scenarioSchema || !Array.isArray(declaration.scenarios)) {
      throw new Error('invalid scenario declaration: ' + file);
    }
    for (const scenario of declaration.scenarios) {
      nonemptyString(scenario.id, 'scenario ID');
      nonemptyString(scenario.claim, 'scenario claim');
      nonemptyString(scenario.test, 'scenario test');
      if (ids.has(scenario.id)) throw new Error('duplicate scenario ID: ' + scenario.id);
      ids.add(scenario.id);
      if (dirname(file) !== dirname(scenario.test) ||
          !existsSync(resolve(root, validPath(scenario.test)))) {
        throw new Error('scenario test must exist beside declaration: ' + scenario.id);
      }
      nonemptyString(scenario.owner, 'scenario owner');
      nonemptyString(scenario.finalSafe, 'scenario final-safe rule');
      stringArray(scenario.requiredCapabilities, 'scenario capabilities');
      stringArray(scenario.evidenceOutputs, 'scenario evidence outputs');
      if (!environments.includes(scenario.environment) ||
          scenario.deterministic !== true ||
          typeof scenario.motionAllowed !== 'boolean' ||
          typeof scenario.timeoutSeconds !== 'number' ||
          !Number.isFinite(scenario.timeoutSeconds) ||
          scenario.timeoutSeconds <= 0 ||
          scenario.evidenceOutputs.length === 0) {
        throw new Error('invalid scenario execution contract: ' + scenario.id);
      }
      if (scenario.environment === 'simulation' &&
          !Object.hasOwn(simulationRunners, scenario.runner)) {
        throw new Error('unknown simulation runner: ' + scenario.id);
      }
      if (scenario.environment === 'source' &&
          !Object.hasOwn(sourceRunners, scenario.runner)) {
        throw new Error('unknown source runner: ' + scenario.id);
      }
      scenarios.push({
        ...scenario,
        declaration: file,
        identity: 'sha256:' + hash(canonical(scenario)),
      });
    }
  }
  return scenarios.sort((a, b) => a.id.localeCompare(b.id));
}

function matches(path, rule) {
  return (rule.paths ?? []).includes(path) ||
    (rule.prefixes ?? []).some((prefix) => path.startsWith(prefix)) ||
    (rule.suffixes ?? []).some((suffix) => path.endsWith(suffix)) ||
    (rule.path_patterns ?? []).some((pattern) => new RegExp(pattern).test(path));
}

// A deleted_only rule classifies a removal only when the diff records it. A
// path list without a diff cannot prove a deletion, so the path stays unknown,
// and a file added again under a retired tree is unknown until reviewed.
function deletedPaths(lines) {
  const deleted = new Set();
  let previous = null;
  for (const line of lines) {
    if (previous !== null && line.startsWith('deleted file mode ') &&
        previous.startsWith('diff --git a/')) {
      const header = previous.slice('diff --git a/'.length);
      const path = header.slice(0, (header.length - ' b/'.length) / 2);
      if (header === path + ' b/' + path) deleted.add(path);
    }
    previous = line;
  }
  return deleted;
}

// The diff is a string or a re-iterable sequence of its lines (diffFileLines);
// it is read twice and never held whole.
export function classify(policy, paths, diff = '', additions = {}) {
  stringArray(paths, 'changed paths');
  // A one-shot iterator (a generator) would leave the second pass empty.
  if (typeof diff !== 'string' && (typeof diff?.[Symbol.iterator] !== 'function' ||
      typeof diff.next === 'function')) {
    throw new Error('diff must be a string or a re-iterable sequence of lines');
  }
  const classes = new Set(additions.classes ?? []);
  const modifiers = new Set(additions.modifiers ?? []);
  const reasons = [];
  const unknownPaths = [];
  const pathsWithoutDiffImpact = new Set();
  const lines = typeof diff === 'string' ? diff.split('\n') : diff;
  const deleted = deletedPaths(lines);
  for (const value of classes) {
    if (!Object.hasOwn(policy.classes, value)) throw new Error('unknown added class ' + value);
    reasons.push({ class: value, because: 'explicit semantic impact' });
  }
  for (const value of modifiers) {
    if (!Object.hasOwn(policy.modifiers, value)) throw new Error('unknown added modifier ' + value);
    reasons.push({ modifier: value, because: 'explicit semantic impact' });
  }
  for (const path of [...new Set(paths.map(validPath))].sort()) {
    let found = false;
    for (const rule of policy.classification_rules) {
      if (!matches(path, rule) || (rule.deleted_only && !deleted.has(path))) continue;
      found = true;
      for (const value of rule.classes ?? []) {
        classes.add(value);
        reasons.push({ path, class: value, because: 'path rule ' + rule.id });
      }
      for (const value of rule.modifiers ?? []) {
        modifiers.add(value);
        reasons.push({ path, modifier: value, because: 'path rule ' + rule.id });
      }
      if (rule.scan_diff_impact === false) pathsWithoutDiffImpact.add(path);
      if (rule.terminal) break;
    }
    if (!found) unknownPaths.push(path);
  }
  // Compile each text pattern once: a retirement diff of 2.2 million lines
  // spent about 36 s rebuilding the same expressions line by line.
  const textRules = [
    ...Object.entries(policy.diff_classes ?? {}).map(([name, patterns]) =>
      ({ key: 'class', name, found: classes, patterns })),
    ...Object.entries(policy.diff_modifiers).map(([name, patterns]) =>
      ({ key: 'modifier', name, found: modifiers, patterns })),
  ].map((rule) => ({ ...rule,
    patterns: rule.patterns.map((value) => ({ value, regex: new RegExp(value, 'i') })) }));
  const seen = new Set(reasons.map(canonical));
  let diffPath = '';
  for (const line of lines) {
    if (line.startsWith('diff --git ')) diffPath = '';
    if (line.startsWith('+++ b/')) diffPath = line.slice(6);
    else if (line.startsWith('--- a/') && !diffPath) diffPath = line.slice(6);
    if (!diffPath || diffPath.endsWith('.md') ||
        pathsWithoutDiffImpact.has(diffPath) ||
        line.startsWith('+++') || line.startsWith('---') ||
        (!line.startsWith('+') && !line.startsWith('-'))) continue;
    const text = line.slice(1);
    for (const rule of textRules) {
      const pattern = rule.patterns.find((item) => item.regex.test(text));
      if (!pattern) continue;
      rule.found.add(rule.name);
      const reason = { path: diffPath, [rule.key]: rule.name,
        because: 'changed text matched ' + pattern.value };
      const key = canonical(reason);
      if (!seen.has(key)) {
        seen.add(key);
        reasons.push(reason);
      }
    }
  }
  return {
    classes: [...classes].sort(),
    modifiers: [...modifiers].sort(),
    reasons: reasons.sort((left, right) => {
      const a = canonical(left);
      const b = canonical(right);
      return a < b ? -1 : a > b ? 1 : 0;
    }),
    unknownPaths,
  };
}

function requirementsFor(policy, classification) {
  const result = new Map();
  for (const [group, names] of [
    ['classes', classification.classes],
    ['modifiers', classification.modifiers],
  ]) {
    for (const name of names) {
      for (const item of policy[group][name]) {
        const key = item.environment + ':' + item.claim;
        const existing = result.get(key) ?? {
          claim: item.claim, environment: item.environment, reasons: [],
        };
        existing.reasons.push({ source: name, reason: item.reason });
        result.set(key, existing);
      }
    }
  }
  return [...result.values()].sort((a, b) =>
    environments.indexOf(a.environment) - environments.indexOf(b.environment) ||
    a.claim.localeCompare(b.claim));
}

export function createPlan(policyRecord, paths, diff, scenarios, source = {}, additions = {}) {
  const { policy, digest } = policyRecord;
  const classification = classify(policy, paths, diff, additions);
  const requirements = requirementsFor(policy, classification);
  const selectedScenarios = scenarios.filter((scenario) =>
    requirements.some((item) =>
      item.claim === scenario.claim && item.environment === scenario.environment));
  const uncoveredClaims = requirements.filter((item) =>
    !selectedScenarios.some((scenario) =>
      item.claim === scenario.claim && item.environment === scenario.environment));
  const requiredEnvironments = [...new Set(requirements.map((item) => item.environment))]
    .sort((a, b) => environments.indexOf(a) - environments.indexOf(b));
  return {
    schema: 'rosie.qualification-plan.v1',
    mode: policy.mode,
    policySha256: digest,
    source,
    changedPaths: [...new Set(paths)].sort(),
    classification,
    requirements,
    requiredEnvironments,
    selectedScenarios,
    uncoveredClaims,
    missingEvidence: requirements.map((item) =>
      ({ claim: item.claim, environment: item.environment, reason: 'no candidate receipt supplied' })),
    wouldBlock: classification.unknownPaths.length > 0 || uncoveredClaims.length > 0,
    canStopAfterSimulation: !requiredEnvironments.some((item) =>
      ['servo_hil', 'robot', 'welding'].includes(item)),
  };
}

// Give use() the changed paths and the diff; the diff file lives only for the
// call.
function withSourceDiff(options, use) {
  if (options.paths_json) {
    const paths = JSON.parse(readFileSync(options.paths_json, 'utf8'));
    stringArray(paths, 'paths JSON');
    return use({ paths, diff: '', source: { kind: 'paths' } });
  }
  const head = exactCommit(options.head ?? options.commit ?? 'HEAD');
  const base = exactCommit(options.base ?? (options.commit ? head + '^' : 'HEAD^'));
  const paths = command('git', ['diff', '--name-only', '-z', base, head])
    .split('\0').filter(Boolean);
  const directory = mkdtempSync(join(tmpdir(), 'rosie-qualification-diff-'));
  try {
    const diffPath = join(directory, 'diff');
    // Fixed prefixes: the classifier reads a/ and b/ headers whatever the local
    // diff.noprefix or diff.mnemonicPrefix setting is.
    commandToFile('git', ['diff', '--no-ext-diff', '--unified=0',
      '--src-prefix=a/', '--dst-prefix=b/', base, head], diffPath);
    return use({ paths, diff: diffFileLines(diffPath), source: { kind: 'git', base, head } });
  } finally {
    rmSync(directory, { recursive: true, force: true });
  }
}

export function deriveCapabilities(cellPath) {
  const relative = validPath(cellPath);
  if (!relative.startsWith('rt-core/config/cells/') || !relative.endsWith('.json')) {
    throw new Error('cell path must name a repository cell config');
  }
  const cell = JSON.parse(readFileSync(resolve(root, relative)));
  const relayNodes = (cell.nodes ?? []).filter((node) => node.role === 'motion-server');
  const runtime = (cell.nodes ?? []).find((node) => node.role === 'realtime-core');
  const targetProfile = relayNodes.length === 1 &&
    typeof relayNodes[0].peer === 'string' && typeof runtime?.peer === 'string'
    ? (relayNodes[0].peer + '-' + runtime.peer).toLowerCase() : null;
  const machinePath = runtime?.rt_core?.machine_config;
  const machine = machinePath ? JSON.parse(readFileSync(resolve(root, validPath(machinePath)))) : null;
  const axes = machine?.axes?.length ?? cell.robot_axes?.axes?.length ?? null;
  const description = machine?.robot?.robot_description ?? null;
  const descriptionPath = description?.path ? validPath(description.path) : null;
  const descriptionManifest = descriptionPath
    ? JSON.parse(readFileSync(resolve(root, descriptionPath,
      'robot_description_manifest.json'))) : null;
  const model = descriptionManifest?.model_id ?? null;
  return {
    schema: 'rosie.qualification-capabilities.v1',
    cell: relative,
    targetProfile,
    fixture: cell.motion_policy?.fixture ?? null,
    nativeRuntime: Boolean(runtime?.rt_core),
    machineConfig: machinePath ?? null,
    machineBackend: machine?.backend ?? null,
    axes,
    robotDescription: description,
    robotModel: model,
    capabilities: [
      ...(runtime?.rt_core ? ['native-rt-core'] : []),
      ...(machine?.backend === 'live' && axes > 0 ? ['ethercat', 'servo'] : []),
      ...(description ? ['robot-description'] : []),
      ...(machinePath ? ['machine-config:' + machinePath] : []),
      ...(model ? ['robot-model:' + model] : []),
    ],
    physicalRobot: 'unknown',
    observedHealth: 'unknown',
  };
}

// Check declared bench fit only. A match never admits physical work.
export function assessDeclaredHilFit(plan, cell) {
  if (plan.schema !== 'rosie.qualification-plan.v1' ||
      cell.schema !== 'rosie.qualification-capabilities.v1') {
    throw new Error('invalid qualification plan or cell capability view');
  }
  const required = plan.requirements.filter((item) => item.environment === 'servo_hil');
  const uncovered = plan.uncoveredClaims.filter((item) => item.environment === 'servo_hil');
  const available = new Set(cell.capabilities);
  const scenarios = plan.selectedScenarios.filter((item) => item.environment === 'servo_hil')
    .map((item) => ({
      id: item.id,
      claim: item.claim,
      identity: item.identity,
      motionAllowed: item.motionAllowed,
      missingCapabilities: item.requiredCapabilities.filter((name) => !available.has(name)).sort(),
    }));
  const declaredFit = required.length === 0 ? 'not_required' :
    uncovered.length > 0 ? 'unknown' :
      scenarios.some((item) => item.missingCapabilities.length > 0) ? 'mismatch' : 'match';
  return {
    schema: 'rosie.qualification-hil-fit.v1',
    source: plan.source,
    cell: cell.cell,
    requiredClaims: required.map((item) => item.claim),
    uncoveredClaims: uncovered.map((item) => item.claim),
    scenarios,
    declaredFit,
    admission: 'not_evaluated',
  };
}

const simulationRunners = Object.freeze({
  'native-home-arm': [
    ['make', ['-C', 'rt-core', 'control', 'sim'], root],
    ['go', ['test', '-count=1', '-run', '^TestSimulatedA6ecRobotHomesAndArmsUnmodified$', './adapters/rosie/control'], join(root, 'rt-core')],
  ],
  'motion-composed': [
    ['make', ['-C', 'motion-server/v1', 'test-composed'], root],
  ],
  'native-runtime-restart': [
    ['make', ['-C', 'rt-core', 'control', 'sim'], root],
    ['go', ['test', '-count=1', '-run',
      '^Test(DaemonRestartFencesSessionsAndHandles|RestartNeverReplaysPlanOrRestoresPermission)$',
      './ipcclient'], join(root, 'rt-core')],
  ],
  'operator-composed': [
    ['make', ['-C', 'rt-core', 'control', 'sim'], root],
    ['npm', ['ci'], join(root, 'offline-programming/v1/ui')],
    ['go', ['test', '-count=1', '-run', '^TestCellProgramsComposedBrowserAndDeck$', '.'],
      join(root, 'offline-programming/v1')],
  ],
});

const sourceRunners = Object.freeze({
  'fleet-reservation-contract': [
    ['npm', ['ci'], join(root, 'coordination/v1')],
    ['npm', ['test'], join(root, 'coordination/v1')],
  ],
  'pr-admission-contract': [
    ['node', ['--test', 'quality/pr-admission.test.mjs'], root],
  ],
  'qualification-policy-contract': [
    ['node', ['--test', 'quality/qualification.test.mjs'], root],
  ],
  'registered-candidate-transaction': [
    ['go', ['test', '-count=1', '.', '-run',
      '^(TestReleaseCandidate.*|TestReleasePlanOnlyBench.*)$'], join(root, 'steamdeck/real/v4')],
  ],
  'managed-release-transaction': [
    ['go', ['test', '-count=1', '.', '-run', '^TestRelease'], join(root, 'steamdeck/real/v4')],
  ],
  'operator-runtime-contract': [
    ['make', ['-C', 'rt-core', 'control', 'sim'], root],
    ['go', ['test', '-race', '-count=1', './internal/denseexec'],
      join(root, 'offline-programming/v1')],
  ],
  'cell-config-compatibility': [
    ['go', ['test', '-count=1', './config/cells'], join(root, 'rt-core')],
  ],
});

function optionsFrom(argv) {
  const options = { classes: [], modifiers: [] };
  for (let index = 0; index < argv.length; index += 2) {
    const key = argv[index]?.replace(/^--/, '').replaceAll('-', '_');
    const value = argv[index + 1];
    if (!key || !argv[index].startsWith('--') || value === undefined) {
      throw new Error('expected --name value pairs');
    }
    if (key === 'add_class') options.classes.push(value);
    else if (key === 'add_modifier') options.modifiers.push(value);
    else if (['base', 'head', 'commit', 'paths_json', 'out', 'summary',
      'candidate_dir', 'plan', 'receipts_dir', 'cell', 'publication_receipt'].includes(key)) {
      options[key] = value;
    }
    else throw new Error('unknown option --' + key);
  }
  return options;
}

function writeJson(path, value) {
  mkdirSync(dirname(path), { recursive: true });
  writeFileSync(path, JSON.stringify(value, null, 2) + '\n', { flag: 'wx' });
}

function summary(plan) {
  const lines = ['## Qualification plan (shadow)', '',
    'Source: ' + (plan.source.head ?? 'path fixture'),
    'Classes: ' + (plan.classification.classes.join(', ') || 'none'),
    'Modifiers: ' + (plan.classification.modifiers.join(', ') || 'none'),
    'Required environments: ' + (plan.requiredEnvironments.join(', ') || 'none'),
    'Would block: ' + plan.wouldBlock, '',
    '| Environment | Claim | Scenario | Reason |',
    '| --- | --- | --- | --- |'];
  for (const item of plan.requirements) {
    const selected = plan.selectedScenarios.filter((scenario) =>
      scenario.claim === item.claim && scenario.environment === item.environment);
    lines.push('| ' + item.environment + ' | ' + item.claim + ' | ' +
      (selected.map((scenario) => scenario.id).join(', ') || 'missing') + ' | ' +
      item.reasons.map((reason) => reason.source + ': ' + reason.reason).join('; ') + ' |');
  }
  if (plan.classification.unknownPaths.length) {
    lines.push('', 'Unknown paths: ' + plan.classification.unknownPaths.join(', '));
  }
  return lines.join('\n') + '\n';
}

function planFromOptions(options) {
  const policyRecord = loadPolicy();
  const scenarios = discoverScenarios();
  return withSourceDiff(options, (input) =>
    createPlan(policyRecord, input.paths, input.diff, scenarios, input.source,
      { classes: options.classes, modifiers: options.modifiers }));
}

export function comparisonStart(targetProfile, source) {
  nonemptyString(targetProfile, 'candidate target profile');
  // The reviewed starts are keyed by the build host's target profiles: site
  // data, kept out of the public mirror. Without the record there is none.
  if (!existsSync(comparisonStartsPath)) {
    throw new Error('no reviewed comparison start for target ' + targetProfile + ' (this tree has no comparison-start record)');
  }
  const record = JSON.parse(readFileSync(comparisonStartsPath, 'utf8'));
  if (record.schema !== 'rosie.qualification-comparison-starts.v1' ||
      !record.targets || typeof record.targets !== 'object' ||
      Array.isArray(record.targets)) {
    throw new Error('invalid qualification comparison-start record');
  }
  const configured = record.targets[targetProfile];
  if (typeof configured !== 'string' || !/^[0-9a-f]{40}$/.test(configured)) {
    throw new Error('no reviewed comparison start for target ' + targetProfile);
  }
  const base = exactCommit(configured);
  const head = exactCommit(source);
  if (base === head || command('git', ['merge-base', base, head]).trim() !== base) {
    throw new Error('comparison start must be an earlier ancestor of candidate source');
  }
  return base;
}

// The verifier is the build host's worker (tools/build1-release-worker). It is
// site infrastructure, kept out of the public mirror; without it a candidate
// is refused, never taken unverified.
function verifiedCandidate(path) {
  if (!path) throw new Error('candidate directory is required');
  if (!existsSync(join(root, 'tools/build1-release-worker'))) {
    throw new Error('candidate verification needs tools/build1-release-worker, the build host worker, which this tree does not have');
  }
  const output = command('python3',
    ['-m', 'rosie_build1_candidate', 'verify', '--candidate', resolve(path)],
    join(root, 'tools/build1-release-worker'));
  const manifest = JSON.parse(output);
  if (!/^sha256:[0-9a-f]{64}$/.test(manifest.candidate_id)) {
    throw new Error('verified candidate ID is invalid');
  }
  return manifest;
}

export function explicitAdditions(plan) {
  const reasons = plan.classification?.reasons;
  if (!Array.isArray(reasons)) throw new Error('plan classification reasons are missing');
  return {
    classes: reasons.filter((item) => item.because === 'explicit semantic impact' && item.class)
      .map((item) => item.class),
    modifiers: reasons.filter((item) => item.because === 'explicit semantic impact' && item.modifier)
      .map((item) => item.modifier),
  };
}

function checkedPlan(options, manifest) {
  const source = manifest.identity.source.commit;
  const base = comparisonStart(manifest.identity.target.profile, source);
  const plan = options.plan
    ? JSON.parse(readFileSync(options.plan, 'utf8'))
    : planFromOptions({ base, head: source, classes: [], modifiers: [] });
  if (plan.schema !== 'rosie.qualification-plan.v1' ||
      plan.policySha256 !== loadPolicy().digest ||
      plan.source?.head !== source ||
      exactCommit('HEAD') !== source) {
    throw new Error('plan, policy, checkout, and candidate source differ');
  }
  if (command('git', ['status', '--porcelain', '--untracked-files=normal']).trim()) {
    throw new Error('candidate source checkout is not clean');
  }
  const expected = planFromOptions({ base, head: source, ...explicitAdditions(plan) });
  for (const field of ['source', 'changedPaths', 'classification', 'requirements',
    'requiredEnvironments', 'selectedScenarios', 'uncoveredClaims',
    'missingEvidence', 'wouldBlock', 'canStopAfterSimulation']) {
    if (canonical(plan[field]) !== canonical(expected[field])) {
      throw new Error('provided plan changes required evidence: ' + field);
    }
  }
  const current = new Map(discoverScenarios().map((scenario) =>
    [scenario.id, scenario.identity]));
  for (const scenario of plan.selectedScenarios) {
    if (current.get(scenario.id) !== scenario.identity) {
      throw new Error('scenario declaration changed after plan: ' + scenario.id);
    }
  }
  return plan;
}

function writeContentAddressed(directory, suffix, raw) {
  mkdirSync(directory, { recursive: true });
  const digest = hash(raw);
  const path = join(directory, digest + suffix);
  if (!existsSync(path)) writeFileSync(path, raw, { flag: 'wx', mode: 0o600 });
  if (hash(readFileSync(path)) !== digest) throw new Error('content-addressed evidence changed');
  return { path: basename(path), sha256: digest };
}

function executeScenario(scenario, manifest, policySha, receiptsDir, runners) {
  const startedAt = new Date().toISOString();
  const deadline = Date.now() + scenario.timeoutSeconds * 1000;
  let result = 'pass';
  let output = '';
  for (const [program, args, cwd] of runners[scenario.runner]) {
    const remaining = deadline - Date.now();
    if (remaining <= 0) {
      result = 'incomplete';
      output += '\nscenario deadline expired\n';
      break;
    }
    const execution = spawnSync(program, args, {
      cwd, encoding: 'utf8', maxBuffer: 8 * 1024 * 1024,
      timeout: remaining, env: { ...process.env, CI: '1' },
    });
    output += '\n$ ' + program + ' ' + args.join(' ') + '\n' +
      (execution.stdout ?? '') + (execution.stderr ?? '');
    if (execution.error) {
      result = 'incomplete';
      output += '\nrunner error: ' + execution.error.message + '\n';
      break;
    }
    if (execution.status !== 0) {
      result = 'fail';
      output += '\nexit status: ' + execution.status + '\n';
      break;
    }
  }
  const log = writeContentAddressed(receiptsDir, '.log', output);
  const receipt = {
    schema: 'rosie.qualification-receipt.v1',
    runId: randomUUID(),
    candidateId: manifest.candidate_id,
    sourceSha: manifest.identity.source.commit,
    targetProfile: manifest.identity.target.profile,
    policySha256: policySha,
    scenarioId: scenario.id,
    scenarioIdentity: scenario.identity,
    environment: scenario.environment,
    runner: scenario.runner,
    runnerSha256: hash(readFileSync(fileURLToPath(import.meta.url))),
    startedAt,
    finishedAt: new Date().toISOString(),
    result,
    behaviorVerdict: result,
    finalSafeVerdict: 'not_applicable',
    log,
  };
  const saved = writeContentAddressed(receiptsDir, '.receipt.json',
    canonical(receipt) + '\n');
  return { ...receipt, receiptSha256: saved.sha256 };
}

function receiptFiles(directory) {
  if (!existsSync(directory)) return [];
  return readdirSync(directory).filter((name) => /^[0-9a-f]{64}\.receipt\.json$/.test(name))
    .sort().map((name) => {
      const raw = readFileSync(join(directory, name));
      if (hash(raw) !== name.slice(0, 64)) throw new Error('receipt digest differs: ' + name);
      const receipt = JSON.parse(raw);
      const log = receipt.log;
      if (!/^[0-9a-f]{64}$/.test(log?.sha256 ?? '') ||
          log.path !== log.sha256 + '.log' ||
          hash(readFileSync(join(directory, log.path))) !== log.sha256) {
        throw new Error('receipt log digest differs: ' + name);
      }
      return receipt;
    });
}

function evaluateEnvironmentReceipts(plan, candidateId, receipts, environment) {
  const required = plan.requirements.filter((item) => item.environment === environment);
  const missingClaims = required.filter((item) => {
    const scenarios = plan.selectedScenarios.filter((scenario) =>
      scenario.claim === item.claim && scenario.environment === environment);
    return scenarios.length === 0 || !scenarios.every((scenario) =>
      receipts.some((receipt) =>
        receipt.candidateId === candidateId &&
        receipt.sourceSha === plan.source.head &&
        receipt.policySha256 === plan.policySha256 &&
        receipt.scenarioId === scenario.id &&
        receipt.scenarioIdentity === scenario.identity &&
        receipt.environment === environment &&
        receipt.result === 'pass' &&
        receipt.behaviorVerdict === 'pass' &&
        receipt.finalSafeVerdict === 'not_applicable'));
  });
  return {
    schema: environment === 'source' ? 'rosie.source-qualification.v1' :
      'rosie.simulation-qualification.v1',
    candidateId,
    policySha256: plan.policySha256,
    state: required.length === 0 ? 'not_required' :
      missingClaims.length === 0 ? environment + '-qualified' : 'incomplete',
    requiredClaims: required.map((item) => item.claim),
    missingClaims: missingClaims.map((item) => item.claim),
  };
}

export function evaluateReceipts(plan, candidateId, receipts) {
  return evaluateEnvironmentReceipts(plan, candidateId, receipts, 'simulation');
}

export function evaluateSourceReceipts(plan, candidateId, receipts) {
  return evaluateEnvironmentReceipts(plan, candidateId, receipts, 'source');
}

// Combine verified inputs only. This result never admits physical work.
export function assessHilInputs(plan, manifest, receipts, cell) {
  if (plan.schema !== 'rosie.qualification-plan.v1' ||
      manifest.schema !== 'rosie.release-candidate.v1' ||
      cell.schema !== 'rosie.qualification-capabilities.v1' ||
      plan.source?.head !== manifest.identity?.source?.commit) {
    throw new Error('invalid HIL input contract');
  }
  const fit = assessDeclaredHilFit(plan, cell);
  const source = evaluateSourceReceipts(plan, manifest.candidate_id, receipts);
  const simulation = evaluateReceipts(plan, manifest.candidate_id, receipts);
  const actualProfile = manifest.identity?.target?.profile ?? null;
  const targetMatches = Boolean(cell.targetProfile && actualProfile === cell.targetProfile);
  const uncoveredSourceClaims = plan.uncoveredClaims
    .filter((item) => item.environment === 'source')
    .map((item) => item.claim);
  const blockers = [];
  if (uncoveredSourceClaims.length) blockers.push('source_claims_uncovered');
  if (source.state === 'incomplete') blockers.push('source_evidence_incomplete');
  if (fit.declaredFit !== 'not_required') {
    if (!targetMatches) blockers.push('candidate_target_mismatch');
    if (simulation.state === 'incomplete') blockers.push('simulation_evidence_incomplete');
    if (fit.declaredFit === 'unknown') blockers.push('hil_scenarios_missing');
    if (fit.declaredFit === 'mismatch') blockers.push('declared_capability_mismatch');
  }
  return {
    schema: 'rosie.qualification-hil-inputs.v1',
    candidateId: manifest.candidate_id,
    policySha256: plan.policySha256,
    cell: cell.cell,
    targetProfile: { expected: cell.targetProfile, actual: actualProfile, matches: targetMatches },
    source,
    simulation,
    declaredFit: fit.declaredFit,
    uncoveredSourceClaims,
    scenarios: fit.scenarios,
    blockers,
    inputStatus: fit.declaredFit === 'not_required' ? 'not_required' :
      blockers.length ? 'blocked' : 'inputs_checked',
    admission: 'not_evaluated',
  };
}

function installSoftwareFloor(candidateDir, publicationReceipt, manifest, policy) {
  const coverage = validateReleaseTestCoverage(loadReleaseTestCoverage());
  const blockers = [];
  if (coverage.suites.length !== policy.install_floor.release_suite_count ||
      coverage.suites.some((suite) => suite.coverage !== 'covered')) {
    blockers.push('release_suite_coverage_incomplete');
  }
  const requiredGates = coverage.suites.filter((suite) => suite.owner === 'build1_candidate')
    .map((suite) => suite.id).sort();
  if (requiredGates.length !== policy.install_floor.build1_suite_count) {
    blockers.push('build1_suite_coverage_incomplete');
  }
  const gateLine = readFileSync(join(candidateDir, 'build.log'), 'utf8').split('\n', 1)[0];
  const gateMatch = /^candidate gates status=(passed|failed) contract_sha256=([0-9a-f]{64}) result_sha256=([0-9a-f]{64}) suites=([^\n]+)$/.exec(gateLine);
  const gateResults = gateMatch?.[4].split(',').map((entry) => entry.split(':')) ?? [];
  if (!gateMatch || gateMatch[1] !== 'passed' ||
      gateResults.some((entry) => entry.length !== 2 || entry[1] !== 'passed') ||
      canonical(gateResults.map((entry) => entry[0]).sort()) !== canonical(requiredGates)) {
    blockers.push('build1_gate_evidence_incomplete');
  }

  if (statSync(publicationReceipt).size > 1024 * 1024) {
    throw new Error('publication receipt exceeds 1 MiB');
  }
  const raw = readFileSync(publicationReceipt);
  const receipt = JSON.parse(raw);
  if (!receipt || typeof receipt !== 'object' || Array.isArray(receipt)) {
    throw new Error('publication receipt must be a JSON object');
  }
  const publication = manifest.publication;
  const source = manifest.identity.source.commit;
  const checks = receipt.source_admission?.checks;
  const expectedWorkflows = [...policy.install_floor.required_workflows].sort();
  const actualWorkflows = Array.isArray(checks)
    ? checks.map((check) => check.workflow_path).sort() : [];
  const assets = receipt.assets;
  const names = readdirSync(candidateDir).sort();
  const assetMatch = assets && typeof assets === 'object' && !Array.isArray(assets) &&
    canonical(Object.keys(assets).sort()) === canonical(names) && names.every((name) => {
      const bytes = readFileSync(join(candidateDir, name));
      return assets[name]?.sha256 === hash(bytes) && assets[name]?.bytes === bytes.length;
    });
  if (receipt.schema !== 'rosie.release-publication-receipt.v1' ||
      receipt.status !== 'published' || receipt.candidate_id !== manifest.candidate_id ||
      receipt.operation_id !== publication.operation_id ||
      receipt.provider !== publication.provider ||
      receipt.repository !== publication.repository ||
      receipt.source_admission?.source_commit !== source ||
      receipt.release?.tag !== publication.release_ref ||
      receipt.release?.target_commit !== source ||
      canonical(actualWorkflows) !== canonical(expectedWorkflows) ||
      !Array.isArray(checks) || checks.some((check) =>
        !Number.isInteger(check.run_id) || check.run_id <= 0 ||
        check.source_commit !== source || check.conclusion !== 'success' ||
        !check.url?.startsWith('https://github.com/')) || !assetMatch) {
    blockers.push('publication_evidence_mismatch');
  }
  return {
    blockers,
    releaseSuites: coverage.suites.length,
    build1Suites: requiredGates,
    build1GateResultSha256: gateMatch?.[3] ?? null,
    publicationReceiptSha256: hash(raw),
    sourceChecks: actualWorkflows,
  };
}

// Check installation inputs without changing motion or robot qualification.
export function assessInstallInputs(plan, manifest, receipts, cell, softwareFloor) {
  if (plan.schema !== 'rosie.qualification-plan.v1' ||
      manifest.schema !== 'rosie.release-candidate.v1' ||
      cell.schema !== 'rosie.qualification-capabilities.v1' ||
      plan.source?.head !== manifest.identity?.source?.commit) {
    throw new Error('invalid install input contract');
  }
  const policy = loadPolicy().policy;
  const fit = assessDeclaredHilFit(plan, cell);
  const source = evaluateSourceReceipts(plan, manifest.candidate_id, receipts);
  const simulation = evaluateReceipts(plan, manifest.candidate_id, receipts);
  const actualProfile = manifest.identity.target.profile;
  const targetMatches = Boolean(cell.targetProfile && actualProfile === cell.targetProfile);
  const observation = fit.scenarios.find((item) =>
    item.id === policy.install_floor.observation_scenario_id &&
    item.claim === 'installed-identity' && item.motionAllowed === false);
  const blockers = [...softwareFloor.blockers];
  if (!targetMatches) blockers.push('candidate_target_mismatch');
  if (simulation.state !== 'simulation-qualified') blockers.push('simulation_evidence_incomplete');
  if (!observation || observation.missingCapabilities.length) {
    blockers.push('motion_free_observation_unavailable');
  }
  return {
    schema: 'rosie.qualification-install-inputs.v1',
    candidateId: manifest.candidate_id,
    policySha256: plan.policySha256,
    cell: cell.cell,
    targetProfile: { expected: cell.targetProfile, actual: actualProfile, matches: targetMatches },
    softwareFloor,
    source,
    simulation,
    uncoveredSourceClaims: plan.uncoveredClaims
      .filter((item) => item.environment === 'source').map((item) => item.claim),
    scenarios: observation ? [observation] : [],
    motionAllowed: false,
    blockers,
    inputStatus: blockers.length ? 'blocked' : 'install-inputs-checked',
    admission: 'not_evaluated',
  };
}

function evaluate(options) {
  if (!options.receipts_dir) throw new Error('receipt directory is required');
  const manifest = verifiedCandidate(options.candidate_dir);
  const plan = checkedPlan(options, manifest);
  return evaluateReceipts(plan, manifest.candidate_id, receiptFiles(options.receipts_dir));
}

function evaluateSource(options) {
  if (!options.receipts_dir) throw new Error('receipt directory is required');
  const manifest = verifiedCandidate(options.candidate_dir);
  const plan = checkedPlan(options, manifest);
  return evaluateSourceReceipts(plan, manifest.candidate_id,
    receiptFiles(options.receipts_dir));
}

function inspectHilInputs(options) {
  if (!options.cell || !options.receipts_dir) {
    throw new Error('hil-inputs needs --cell and --receipts-dir');
  }
  const manifest = verifiedCandidate(options.candidate_dir);
  const plan = checkedPlan(options, manifest);
  return assessHilInputs(plan, manifest, receiptFiles(options.receipts_dir),
    deriveCapabilities(options.cell));
}

function inspectInstallInputs(options) {
  if (!options.cell || !options.receipts_dir || !options.publication_receipt) {
    throw new Error('install-inputs needs --cell, --receipts-dir, and --publication-receipt');
  }
  const manifest = verifiedCandidate(options.candidate_dir);
  const plan = checkedPlan(options, manifest);
  const floor = installSoftwareFloor(options.candidate_dir, options.publication_receipt,
    manifest, loadPolicy().policy);
  return assessInstallInputs(plan, manifest, receiptFiles(options.receipts_dir),
    deriveCapabilities(options.cell), floor);
}

function runSimulation(options) {
  if (!options.receipts_dir) throw new Error('receipt directory is required');
  const manifest = verifiedCandidate(options.candidate_dir);
  const plan = checkedPlan(options, manifest);
  const selected = plan.selectedScenarios.filter((scenario) =>
    scenario.environment === 'simulation');
  const receipts = [];
  for (const scenario of selected) {
    receipts.push(executeScenario(scenario, manifest, plan.policySha256,
      options.receipts_dir, simulationRunners));
  }
  const evaluation = evaluateReceipts(plan, manifest.candidate_id,
    receiptFiles(options.receipts_dir));
  process.stdout.write(JSON.stringify({ receipts, evaluation }, null, 2) + '\n');
  return evaluation.state === 'incomplete' ? 1 : 0;
}

function runSource(options) {
  if (!options.receipts_dir) throw new Error('receipt directory is required');
  const manifest = verifiedCandidate(options.candidate_dir);
  const plan = checkedPlan(options, manifest);
  const selected = plan.selectedScenarios.filter((scenario) =>
    scenario.environment === 'source');
  const receipts = selected.map((scenario) => executeScenario(
    scenario, manifest, plan.policySha256, options.receipts_dir, sourceRunners));
  const evaluation = evaluateSourceReceipts(plan, manifest.candidate_id,
    receiptFiles(options.receipts_dir));
  process.stdout.write(JSON.stringify({ receipts, evaluation }, null, 2) + '\n');
  return receipts.some((receipt) => receipt.result !== 'pass') ? 1 : 0;
}

function main(argv) {
  const [verb, ...rest] = argv;
  const options = optionsFrom(rest);
  if (verb === 'catalog') {
    process.stdout.write(JSON.stringify(discoverScenarios(), null, 2) + '\n');
    return;
  }
  if (verb === 'capabilities') {
    if (!options.cell) throw new Error('capabilities needs --cell');
    process.stdout.write(JSON.stringify(deriveCapabilities(options.cell), null, 2) + '\n');
    return;
  }
  if (verb === 'hil-fit') {
    if (!options.cell) throw new Error('hil-fit needs --cell');
    process.stdout.write(JSON.stringify(assessDeclaredHilFit(
      planFromOptions(options), deriveCapabilities(options.cell)), null, 2) + '\n');
    return;
  }
  if (verb === 'hil-inputs') {
    process.stdout.write(JSON.stringify(inspectHilInputs(options), null, 2) + '\n');
    return;
  }
  if (verb === 'install-inputs') {
    process.stdout.write(JSON.stringify(inspectInstallInputs(options), null, 2) + '\n');
    return;
  }
  if (verb === 'plan') {
    const plan = planFromOptions(options);
    if (options.out) writeJson(options.out, plan);
    if (options.summary) {
      mkdirSync(dirname(options.summary), { recursive: true });
      writeFileSync(options.summary, summary(plan), { flag: 'a' });
    }
    process.stdout.write(JSON.stringify(plan, null, 2) + '\n');
    return;
  }
  if (verb === 'simulate') {
    process.exitCode = runSimulation(options);
    return;
  }
  if (verb === 'source') {
    process.exitCode = runSource(options);
    return;
  }
  if (verb === 'evaluate') {
    process.stdout.write(JSON.stringify(evaluate(options), null, 2) + '\n');
    return;
  }
  if (verb === 'evaluate-source') {
    process.stdout.write(JSON.stringify(evaluateSource(options), null, 2) + '\n');
    return;
  }
  throw new Error('usage: qualification.mjs plan|catalog|capabilities|hil-fit|hil-inputs|install-inputs|source|evaluate-source|simulate|evaluate');
}

const invoked = process.argv[1] ? resolve(process.argv[1]) : '';
if (invoked === fileURLToPath(import.meta.url)) {
  try {
    main(process.argv.slice(2));
  } catch (error) {
    console.error('qualification_unavailable: ' + error.message);
    process.exitCode = 2;
  }
}
