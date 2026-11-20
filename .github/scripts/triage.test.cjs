const assert = require('node:assert/strict');
const test = require('node:test');
const { issueBoundary, pullBoundary, run } = require('./triage.cjs');

test('issue scope maps one explicit system boundary', () => {
  assert.equal(issueBoundary('fix(motion-server/v1): Stop receipt'), 'motion-control');
  assert.equal(issueBoundary('fix(motion-server): Stop receipt'), 'motion-control');
  assert.equal(issueBoundary('test(mujoco): separate plant timing'), 'simulation');
  assert.equal(issueBoundary('Release: native stack'), 'deployment');
  assert.equal(issueBoundary('Reconcile Rosie gear ratios across URDF and simulator'), null);
});

test('PR paths abstain when multiple owners or no known owner changed', () => {
  assert.equal(pullBoundary(['motion-server/v1/src/daemon.cpp', 'motion-server/v1/tests/test.cpp']), 'motion-control');
  assert.equal(pullBoundary(['robot_description/robot.urdf', 'rt-core/config/cells/rosie.json']), 'configuration');
  assert.equal(pullBoundary(['motion-server/v1/src/daemon.cpp', 'rt-core/engine/main.cpp']), null);
  assert.equal(pullBoundary(['.github/workflows/ci.yml', 'preflight/src/run.py']), null);
  assert.equal(pullBoundary(['motion-server/v1/src/daemon.cpp', 'README.md']), 'motion-control');
  assert.equal(pullBoundary(['README.md']), null);
});

test('existing human boundary is never overwritten', async () => {
  let writes = 0;
  const github = { rest: { issues: { addLabels: async () => { writes++; } } } };
  const context = {
    eventName: 'issues', repo: { owner: 'advanced-metal-research', repo: 'RosieOS' },
    payload: { issue: { number: 1, title: 'fix(motion-server): Stop', labels: [{ name: 'runtime' }] } },
  };
  await run({ github, context, core: { info() {} } });
  assert.equal(writes, 0);
});

test('ambiguous issue receives a triage label', async () => {
  const writes = [];
  const github = { rest: { issues: { addLabels: async args => { writes.push(args); } } } };
  const context = {
    eventName: 'issues', repo: { owner: 'advanced-metal-research', repo: 'RosieOS' },
    payload: { issue: { number: 2, title: 'Reconcile Rosie gear ratios', labels: [] } },
  };
  await run({ github, context, core: { info() {} } });
  assert.deepEqual(writes[0].labels, ['needs-triage']);
});

test('weekly sweep labels old unlabeled items and preserves assigned boundaries', async () => {
  const writes = [];
  const github = {
    paginate: async (_method, options) => options.state === 'open' && _method === github.rest.issues.listForRepo
      ? [
          { number: 1, title: 'fix(motion-server): Stop', updated_at: '2026-08-01', labels: [] },
          { number: 2, title: 'test(mujoco): plant timing', updated_at: '2026-08-02', labels: [{ name: 'simulation' }] },
          { number: 4, title: 'unclear', updated_at: '2026-07-01', labels: [{ name: 'needs-triage' }] },
        ]
      : [{ number: 3, changed_files: 1, updated_at: '2026-08-03', labels: [] }],
    rest: {
      issues: {
        listForRepo() {},
        addLabels: async args => { writes.push([args.issue_number, args.labels[0]]); },
      },
      pulls: {
        list() {},
        listFiles: async () => ({ data: [{ filename: 'rt-core/engine/main.cpp' }] }),
      },
    },
  };
  const context = { eventName: 'schedule', repo: { owner: 'advanced-metal-research', repo: 'RosieOS' } };
  await run({ github, context, core: { info() {} } });
  assert.deepEqual(writes, [[1, 'motion-control'], [3, 'runtime']]);
});
