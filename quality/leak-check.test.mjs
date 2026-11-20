// Contract tests for the public-mirror leak gate (quality/leak-check.mjs): the
// private-path matcher, the secret and password rules, and the site-name rule
// that reads its names from private site files instead of spelling them out.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync, mkdirSync, writeFileSync } from 'node:fs';
import { deflateRawSync, gzipSync } from 'node:zlib';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { archiveMembers, hitsIn, patterns, privateLinkPattern, privateMatcher, scan, siteNames, sitePattern } from './leak-check.mjs';

// Joined at run time, so this file holds no string the gate (or gitleaks) flags.
const j = (...parts) => parts.join('');

test('private paths: dir/ prefix, * within a segment, **/ in any directory', () => {
  const isPrivate = privateMatcher(['# comment', 'docs/site/', 'rt-core/config/cells/jetson-*.json', '**/*_r0*_test.go', '**/r0*_test.go']);
  assert.ok(isPrivate('docs/site/CLAUDE.md'));
  assert.ok(isPrivate('rt-core/config/cells/jetson-77-zz9001-demo.json'));
  assert.ok(!isPrivate('rt-core/config/cells/sub/jetson-1.json'), '* must not cross a directory');
  assert.ok(isPrivate('cell_r0099_test.go'), '**/ matches at the root');
  assert.ok(isPrivate('offline-programming/v1/internal/targets/cell_r0099_test.go'));
  assert.ok(isPrivate('rt-core/tools/rtctl/r0099_j5_limit_test.go'));
  assert.ok(!isPrivate('rt-core/tools/rtctl/limit_test.go'));
  assert.ok(!isPrivate('offline-programming/v1/internal/targets/cell_r0099.go'), 'only tests');
  assert.ok(isPrivate('rt-core/tools/rtctl/r0099_j5_limit_test.go.scenarios.json'), 'a declaration follows its test');
  assert.ok(!isPrivate('rt-core/tools/rtctl/limit_test.go.scenarios.json'));
  assert.ok(!isPrivate('docs/site.scenarios.json'), 'a prefix rule names a directory, not a declaration');
});

test('secret: token formats and private keys; placeholders pass', () => {
  const token = (prefix, n) => prefix + 'A1b2C3d4'.repeat(8).slice(0, n);
  for (const text of [
    j('-----BEGIN OPENSSH ', 'PRIVATE KEY-----'),
    token('ghp_', 36), token('gho_', 36), token('hf_', 34), token('npm_', 36),
    token('sk-proj-', 40), token('xoxb-', 24), 'AIza' + 'A1b2C3d4'.repeat(5).slice(0, 35),
    j('https://hooks.', 'slack.com/services/T000/B000/XXXX'),
    j('aws_secret_', 'access_key = ') + 'A1b2C3d4'.repeat(5),
  ]) assert.ok(hitsIn(text).has('secret'), text.slice(0, 12));
  assert.ok(!hitsIn('akia' + 'abcd'.repeat(4)).has('secret'), 'AWS key ids are upper case');
});

test('password: a literal assigned to a password-like name; placeholders pass', () => {
  assert.ok(hitsIn(j('pass', 'word = "hunter2hunter2"')).has('password'));
  assert.ok(hitsIn(j('{"api_', 'key": "q8Zr2mLk91"}')).has('password'));
  for (const text of ['password = "changeme"', 'password: "<your password>"', 'token = "${TOKEN}"', 'secret_ref = "jetson-cell"', 'password = ""'])
    assert.ok(!hitsIn(text).has('password'), text);
});

test('url-credentials: a login inside a URL; placeholder passwords pass', () => {
  assert.ok(hitsIn(j('nats://robot:Zq81mx', '@nats.internal:4222')).has('url-credentials'));
  for (const text of ['http://user:pass@example.test', 'https://user:secret@host', 'http://u:p@127.0.0.1:8090', 'git@github.com:org/repo'])
    assert.ok(!hitsIn(text).has('url-credentials'), text);
});

test('site-name: names come from the private site files only', () => {
  const dir = mkdtempSync(join(tmpdir(), 'leak-site-'));
  mkdirSync(join(dir, 'rt-core/config/cells'), { recursive: true });
  mkdirSync(join(dir, 'robot_description/robots/zz9001'), { recursive: true });
  mkdirSync(join(dir, 'robot_description/robots/demo_arm'), { recursive: true });
  writeFileSync(join(dir, 'rt-core/config/cells/hostq-7-zz9001.json'), JSON.stringify({
    name: 'hostq-7',
    nodes: [{ host: 'ServoQ42', ssh_host: 'fe80::1%eth0', remote_user: 'deck' }],
    devices: { 'padq-3': { management_host: 'padq-3.example.ts.net', ssh_user: 'qperson' } },
  }));
  writeFileSync(join(dir, 'rt-core/config/cells/public-demo.json'), JSON.stringify({ name: 'demo-host' }));
  mkdirSync(join(dir, 'tools/public-mirror'), { recursive: true });
  writeFileSync(join(dir, 'tools/public-mirror/site-names.txt'), '# fleet\nfleetq-1\n\nqlogin\n');
  const isPrivate = privateMatcher(['rt-core/config/cells/hostq-*.json', 'robot_description/robots/zz9001/', 'tools/public-mirror/site-names.txt']);
  const names = siteNames(dir, isPrivate);
  assert.deepEqual(names, ['ServoQ42', 'fleetq-1', 'hostq-7', 'hostq-7-zz9001', 'padq-3', 'qlogin', 'qperson', 'zz9001']);
  assert.ok(!siteNames(dir, privateMatcher(['rt-core/config/cells/hostq-*.json'])).includes('fleetq-1'), 'a published names file teaches nothing');
  const site = sitePattern(names);
  const list = [...patterns, site];
  assert.ok(hitsIn('ssh servoq42 as qperson', '', list).has('site-name'), 'any case');
  assert.ok(hitsIn('cell hostq-7-zz9001', '', list).has('site-name'));
  const dotted = [sitePattern(['q.dot'])];
  assert.ok(hitsIn(String.raw`assert.match(s, /^by q\.dot@host/)`, '', dotted).has('site-name'), 'a dot escaped in a regex');
  assert.ok(!hitsIn('qxdot', '', dotted).has('site-name'), 'the dot still means a dot');
  assert.ok(!hitsIn('hostq-70 and demo-host and deck', '', list).has('site-name'), 'whole words, public cells, placeholders');
  for (const text of ['measured on Hostq 7', 'const hostq7 = {}', 'subject robot.hostq-7.command', 'padq_3'])
    assert.ok(hitsIn(text, '', list).has('site-name'), `another spelling or after a dot: ${text}`);
  assert.ok(!hitsIn('hostq-77 and xhostq-7', '', list).has('site-name'), 'still whole words');
  assert.equal(sitePattern([]), null, 'no private files, no rule');
});

test('login: a person recorded as who did something; placeholders and addresses pass', () => {
  assert.ok(hitsIn(j('realtime-core deploy (', 'qperson@Laptop-4', ')')).has('login'));
  for (const text of ['realtime-core deploy (operator@mac)', 'contact (support@vendor.example)', 'view front@row'])
    assert.ok(!hitsIn(text).has('login'), text);
});

// A ZIP of stored or deflated members, as a .weldplan is. The CRCs are left
// zero: the gate reads members, it does not verify them.
function zip(members) {
  const locals = [], centrals = [];
  let offset = 0;
  for (const [name, text, deflate] of members) {
    const raw = Buffer.from(text), data = deflate ? deflateRawSync(raw) : raw, nameBytes = Buffer.from(name);
    const local = Buffer.alloc(30); local.writeUInt32LE(0x04034b50, 0); local.writeUInt16LE(deflate ? 8 : 0, 8);
    local.writeUInt32LE(data.length, 18); local.writeUInt32LE(raw.length, 22); local.writeUInt16LE(nameBytes.length, 26);
    const central = Buffer.alloc(46); central.writeUInt32LE(0x02014b50, 0); central.writeUInt16LE(deflate ? 8 : 0, 10);
    central.writeUInt32LE(data.length, 20); central.writeUInt32LE(raw.length, 24); central.writeUInt16LE(nameBytes.length, 28);
    central.writeUInt32LE(offset, 42);
    locals.push(local, nameBytes, data); centrals.push(central, nameBytes);
    offset += 30 + nameBytes.length + data.length;
  }
  const directory = Buffer.concat(centrals), end = Buffer.alloc(22);
  end.writeUInt32LE(0x06054b50, 0); end.writeUInt16LE(members.length, 8); end.writeUInt16LE(members.length, 10);
  end.writeUInt32LE(directory.length, 12); end.writeUInt32LE(offset, 16);
  return Buffer.concat([...locals, directory, end]);
}

test('archives: ZIP and gzip members are checked like files; CAD and foreign meshes are refused', () => {
  assert.deepEqual(archiveMembers(zip([['a.json', '{}', false], ['b/c.txt', 'hello', true]])).map((m) => [m.name, m.bytes.toString()]),
    [['a.json', '{}'], ['b/c.txt', 'hello']]);
  assert.equal(archiveMembers(Buffer.from('plain text')), null);
  const dir = mkdtempSync(join(tmpdir(), 'leak-archive-'));
  const write = (path, bytes) => { mkdirSync(join(dir, path, '..'), { recursive: true }); writeFileSync(join(dir, path), bytes); return { path, file: join(dir, path) }; };
  const robotMesh = 'solid public-link';
  const files = [
    write('robots/demo/meshes/link_1.stl', robotMesh),
    write('data/plan.weldplan', zip([['program.json', j('{"path": "/home/', 'qperson/part.step"}'), true],
      ['source/part.step', 'ISO-10303-21;', false], ['context/meshes/link_1.stl', robotMesh, false], ['context/meshes/cell.dae', '<COLLADA/>', true]])),
    write('evidence/run.jsonl.gz', gzipSync(j('{"by": "deploy (', 'qperson@Laptop-4', ')"}'))),
    write('cad/part.step', 'ISO-10303-21;'),
    write('meshes/copy.dae', '<COLLADA private/>'),
  ];
  const privateFiles = [write('private/robot/link.dae', '<COLLADA private/>')];
  assert.deepEqual(scan(files, () => false, patterns, privateFiles), [
    'archive-mesh\tdata/plan.weldplan!context/meshes/cell.dae',
    'cad\tcad/part.step',
    'cad\tdata/plan.weldplan!source/part.step',
    'home-path\tdata/plan.weldplan!program.json',
    'login\tevidence/run.jsonl.gz!run.jsonl',
    'private-copy\tmeshes/copy.dae',
  ]);
  assert.deepEqual(scan([write('broken.weldplan', Buffer.concat([zip([['a', 'b', false]]).subarray(0, 40), Buffer.alloc(30)]))], () => false, patterns),
    ['archive\tbroken.weldplan'], 'an archive that cannot be read cannot be cleared');
});

test('private-link: a public doc linking a file the mirror leaves out', () => {
  const rule = [privateLinkPattern(privateMatcher(['rt-core/docs/tracker.html', 'rt-core/docs/evidence/', 'docs/site/']))];
  const has = (text, path) => hitsIn(text, path, rule).has('private-link');
  assert.ok(has('See [P8-45](tracker.html#P8-45).', 'rt-core/docs/MIGRATION.md'), 'an anchor still names the file');
  assert.ok(has('[log](../evidence/run-1/step.log#L14)', 'rt-core/docs/history/README.md'), 'a file inside a private directory');
  assert.ok(has('[runs](evidence/run-1/)', 'rt-core/docs/BENCH.md'), 'a link to the directory itself');
  assert.ok(has('[notes](site/CLAUDE.md)', 'docs/README.md'));
  assert.ok(has('[a%20b](evidence/a%20b.json)', 'rt-core/docs/X.md'), 'percent-encoded targets are decoded');
  assert.ok(!has('[contracts](CONTRACTS.md) and [#160](https://github.com/o/r/issues/160)', 'rt-core/docs/MIGRATION.md'));
  assert.ok(!has('P8-45 in tracker.html', 'rt-core/docs/MIGRATION.md'), 'a plain mention is not a link');
  assert.ok(!has('[P8-45](tracker.html#P8-45)', 'rt-core/docs/tracker-notes.txt'), 'only Markdown files');
  assert.ok(!has('[up](../../outside.md)', 'a.md'), 'a target outside the tree is not the rule\'s business');
  const repo = j('https://github.com/advanced-metal', '-research/RosieOS');
  assert.ok(has(`[GitHub #160](${repo}/issues/160)`, 'rt-core/docs/MIGRATION.md'), 'this private repository');
  assert.ok(has(`[AMR-1188](${j('https://linear.app/advanced', 'metalresearch/issue/AMR-1188')})`, 'rt-core/docs/MIGRATION.md'), "the team's Linear");
  assert.ok(!has(`[mirror](${repo}-public)`, 'README.md'), 'the public mirror is public');
});
