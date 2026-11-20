const BOUNDARIES = new Set([
  'client', 'configuration', 'motion-control', 'runtime', 'simulation',
  'deployment', 'physical-cell', 'planning', 'architecture',
]);

const SCOPES = new Map([
  ['deck', 'client'], ['steamdeck', 'client'], ['steam deck', 'client'], ['olp', 'client'], ['ui', 'client'],
  ['sim', 'simulation'], ['simulator', 'simulation'], ['mujoco', 'simulation'],
  ['motion', 'motion-control'], ['motion-server', 'motion-control'], ['motion server', 'motion-control'],
  ['runtime', 'runtime'], ['daemon', 'runtime'], ['rt-core', 'runtime'],
  ['config', 'configuration'], ['calibration', 'configuration'], ['robot-description', 'configuration'], ['robot description', 'configuration'],
  ['deploy', 'deployment'], ['deployment', 'deployment'], ['release', 'deployment'], ['build', 'deployment'], ['ci', 'deployment'],
  ['planner', 'planning'], ['planning', 'planning'], ['weld-planner', 'planning'],
  ['architecture', 'architecture'], ['repo', 'architecture'],
  ['physical', 'physical-cell'], ['cell', 'physical-cell'],
]);

function issueBoundary(title) {
  const scope = /^\w+\(([^)]+)\):/.exec(title)?.[1]?.toLowerCase();
  if (scope) return SCOPES.get(scope.split('/')[0]) ?? null;
  const lead = /^([a-z][a-z -]{1,30}):/i.exec(title)?.[1]?.trim().toLowerCase();
  return lead ? SCOPES.get(lead) ?? null : null;
}

function pathBoundary(path) {
  if (/^(robot_description|rt-core\/config)\//.test(path)) return 'configuration';
  if (path.startsWith('rt-core/')) return 'runtime';
  if (path.startsWith('motion-server/')) return 'motion-control';
  if (path.startsWith('offline-programming/v1/internal/simulator/')) return 'simulation';
  if (/^(offline-programming|steamdeck)\//.test(path)) return 'client';
  if (path.startsWith('weld_planner/')) return 'planning';
  if (/^(releases|deploy)\//.test(path) || path.startsWith('.github/workflows/')) return 'deployment';
  return null;
}

function pullBoundary(paths) {
  const pathsToClassify = paths.filter(path => !path.endsWith('.md'));
  const mapped = pathsToClassify.map(pathBoundary);
  if (mapped.includes(null)) return null;
  const candidates = new Set(mapped);
  return candidates.size === 1 ? [...candidates][0] : null;
}

async function labelItem({ github, context, core, item, kind }) {
  const labels = new Set(item.labels.map(label => label.name));
  if ([...labels].some(label => BOUNDARIES.has(label))) {
    core.info(`#${item.number}: existing boundary label preserved`);
    return;
  }

  let boundary;
  if (kind === 'issue') {
    boundary = issueBoundary(item.title);
  } else {
    if (item.changed_files === undefined || item.changed_files < 100) {
      const { data: files } = await github.rest.pulls.listFiles({
        ...context.repo, pull_number: item.number, per_page: 100,
      });
      if (files.length < 100) boundary = pullBoundary(files.map(file => file.filename));
    }
  }

  const label = boundary ?? 'needs-triage';
  if (labels.has(label)) return;
  await github.rest.issues.addLabels({ ...context.repo, issue_number: item.number, labels: [label] });
  core.info(`#${item.number}: added ${label}`);
}

async function run({ github, context, core }) {
  if (context.eventName === 'issues' || context.eventName === 'pull_request_target') {
    const kind = context.eventName === 'issues' ? 'issue' : 'pull';
    const item = context.payload.issue ?? context.payload.pull_request;
    if (!item) throw new Error(`${kind} event payload required`);
    await labelItem({ github, context, core, item, kind });
    return;
  }
  if (context.eventName !== 'schedule' && context.eventName !== 'workflow_dispatch') {
    throw new Error(`unsupported event: ${context.eventName}`);
  }

  const [issues, pulls] = await Promise.all([
    github.paginate(github.rest.issues.listForRepo, { ...context.repo, state: 'open', per_page: 100 }),
    github.paginate(github.rest.pulls.list, { ...context.repo, state: 'open', per_page: 100 }),
  ]);
  const unlabeled = [
    ...issues.filter(item => !item.pull_request).map(item => ({ item, kind: 'issue' })),
    ...pulls.map(item => ({ item, kind: 'pull' })),
  ].filter(({ item }) => !item.labels.some(label => BOUNDARIES.has(label.name) || label.name === 'needs-triage'))
    .sort((a, b) => Date.parse(a.item.updated_at) - Date.parse(b.item.updated_at));
  for (const entry of unlabeled.slice(0, 50)) {
    await labelItem({ github, context, core, ...entry });
  }
  core.info(`reviewed ${Math.min(unlabeled.length, 50)} of ${unlabeled.length} open items without a boundary label`);
}

module.exports = { issueBoundary, pullBoundary, run };
