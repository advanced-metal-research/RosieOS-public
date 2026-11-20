#!/usr/bin/env node
// Leak gate for the public mirror (#153). Fails when a site- or person-specific
// pattern appears in a file the mirror would publish and that (pattern, path)
// pair is not in quality/leak-allowlist.txt. The allowlist records today's known
// hits so the cleanup cannot regress while it is in progress; each cleanup
// shrinks it, and the publish step runs with --strict (allowlist ignored).
//
// The patterns are generic on purpose: a check that spelled out the tailnet
// name or an address would publish exactly what it guards. The one exception,
// site-name, takes its names at run time from the private site files (the real
// cells' manifests and robots), so this file names no cell, host or person; in
// a tree without those files it matches nothing.
//
//   node quality/leak-check.mjs                 tracked files, minus private paths
//   node quality/leak-check.mjs --dir <export>  an exported mirror tree
//   node quality/leak-check.mjs --strict        ignore the allowlist
//   node quality/leak-check.mjs --update        rewrite the allowlist from today's hits
//
// ZIP and gzip files are opened and each member is checked as a file of its
// own (path "outer!member"): a .weldplan is a ZIP, and the text patterns cannot
// see through compression. CAD source files are refused wherever they are, a
// mesh inside an archive is refused unless it is byte-identical to a public
// mesh, and any public file or member identical to a private mesh or CAD file
// is refused as a copy of it.
import { spawnSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { existsSync, readFileSync, readdirSync, writeFileSync, statSync } from 'node:fs';
import { basename, join, posix, relative, sep } from 'node:path';
import { fileURLToPath } from 'node:url';
import { gunzipSync, inflateRawSync } from 'node:zlib';

const root = fileURLToPath(new URL('../', import.meta.url));
const allowlistPath = join(root, 'quality/leak-allowlist.txt');
const privatePathsPath = join(root, 'tools/public-mirror/private-paths.txt');

// Login names that are platform defaults or placeholders, not people.
const placeholderUsers = new Set(['user', 'users', 'runner', 'changeme', 'ci', 'op', 'controller', 'pi', 'ubuntu', 'deck', 'operator', 'example', 'name', 'me', 'you', 'shared', 'public', 'root']);
// Placeholder domains (any "example" label, RFC 2606/6761 names, systemd .service
// units, GitHub noreply) and the drive vendor's published support address.
const placeholderEmailDomain = /(^|\.)example(\.|$)|(^|\.)(invalid|test|local|localhost|service|users\.noreply\.github\.com|omc-stepperonline\.com)$/i;
// Lockfiles carry third-party package authors' addresses.
const lockfile = /(^|\/)(package-lock\.json|uv\.lock|Cargo\.lock|pixi\.lock)$/;
// Values that only stand in for a password or token in tests and examples.
const placeholderSecret = /^(?:pass|password|passwd|secret|token|changeme|example|placeholder|redacted|dummy|test|x+|\*+|<[^>]*>|\$\{[^}]*\}|your[-_].*)$/i;

export const patterns = [
  // Tailscale tailnet identifiers: <host>.tail<hex>.ts.net and the bare tailnet name.
  { id: 'tailnet', regex: /\btail[0-9a-f]{4,}\b/g },
  // Tailscale CGNAT addresses (the 100.64/10 block). 100.64.0.x is the
  // placeholder range: fixtures that must stay tailnet-classified use it, and
  // 100.64.0.0 is how code recognises the block.
  { id: 'tailscale-ip', regex: /\b100\.(?:6[4-9]|[7-9]\d|1[01]\d|12[0-7])\.\d{1,3}\.\d{1,3}\b/g, ignore: (m) => m.startsWith('100.64.0.') },
  // EUI-48 hardware addresses. Not identities: all-zero, broadcast, the
  // 02:00:00:00:xx:xx locally administered placeholders and aa:bb:cc:dd:ee:ff.
  { id: 'mac', regex: /\b(?:[0-9a-f]{2}:){5}[0-9a-f]{2}\b/gi, ignore: (m) => /^(00:){5}00$|^(ff:){5}ff$|^02:00:00:00:[0-9a-f]{2}:[0-9a-f]{2}$|^aa:bb:cc:dd:ee:ff$/i.test(m) },
  { id: 'email', skip: (path) => lockfile.test(path), regex: /\b[A-Za-z0-9._%+-]+@[A-Za-z0-9-]+(?:\.[A-Za-z0-9-]+)*\.[A-Za-z]{2,}\b/g, ignore: (m) => m === 'git@github.com' || placeholderEmailDomain.test(m.split('@')[1]) },
  // Home directories that name a person: C:\Users\<name>, /home/<name>, /Users/<name>.
  // A path starts the token: "jog/home/limits" in prose is not a directory.
  { id: 'home-path', regex: /(?:\b[A-Za-z]:[\\/]{1,2}Users[\\/]{1,2}|(?<![\w./-])\/home\/|(?<![\w./-])\/Users\/)([A-Za-z][\w.-]*)/g, ignore: (m, name) => placeholderUsers.has(name.toLowerCase()) },
  { id: 'cloudflare-id', regex: /"(?:account_id|database_id)"\s*:\s*"[0-9a-f-]{32,36}"/g },
  // Private keys and the token formats of services the team uses or might:
  // GitHub, Tailscale, Anthropic, xAI, OpenAI, Hugging Face, npm, Slack,
  // Google, Stripe, Linear, AWS; Slack and Discord webhook URLs; JSON Web Tokens.
  { id: 'secret', regex: /-----BEGIN [A-Z ]*PRIVATE KEY-----|\b(?:gh[oprsu]_[A-Za-z0-9]{36}|github_pat_[A-Za-z0-9_]{22,}|tskey-[a-z]+-[A-Za-z0-9-]{10,}|sk-ant-[A-Za-z0-9_-]{20,}|xai-[A-Za-z0-9]{20,}|sk-(?:proj-)?[A-Za-z0-9_-]{32,}|hf_[A-Za-z0-9]{30,}|npm_[A-Za-z0-9]{36}|xox[abposr]-[A-Za-z0-9-]{10,}|AIza[0-9A-Za-z_-]{35}|[rs]k_live_[A-Za-z0-9]{20,}|lin_api_[A-Za-z0-9]{30,}|AKIA[0-9A-Z]{16})\b|hooks\.slack\.com\/services\/[A-Za-z0-9/]+|discord(?:app)?\.com\/api\/webhooks\/\d+\/[\w-]+|\beyJ[A-Za-z0-9_-]{10,}\.eyJ[A-Za-z0-9_-]{10,}\.[A-Za-z0-9_-]{10,}|(?<![\w])(?:aws_secret_access_key|AWS_SECRET_ACCESS_KEY)\s*[:=]\s*\S{20,}/g },
  // A literal assigned to a password-like name: password = "...", "api_key": "...".
  { id: 'password', regex: /\b(?:password|passwd|pwd|passphrase|secret|api[_-]?key|auth[_-]?token|access[_-]?token|client[_-]?secret)\b["']?\s*[:=]\s*["']([^"'\s]{6,})["']/gi, ignore: (m, value) => placeholderSecret.test(value) },
  // A login inside a URL: scheme://user:password@host. A password under four
  // characters ("u:p@" in a refusal test) is a placeholder.
  // Who did something, as the deploy tool records it: "realtime-core deploy
  // (<login>@<machine>)". A login there is a person unless it is a
  // placeholder; a dotted machine is an address, which the email rule checks.
  { id: 'login', regex: /\(([A-Za-z][\w.-]*)@[A-Za-z][\w-]*\)/g, ignore: (m, login) => placeholderUsers.has(login.toLowerCase()) },
  { id: 'url-credentials', regex: /\b[a-z][a-z0-9+.-]*:\/\/[^/\s:@"'`]+:([^/\s@"'`]{4,})@[^\s"'`]+/gi, ignore: (m, password) => placeholderSecret.test(password) },
];

// Fields of the private site files whose values are host or login names.
const siteHostKeys = new Set(['host', 'peer', 'ssh_host', 'management_host', 'hostname']);
const siteLoginKeys = new Set(['remote_user', 'ssh_user', 'user']);
const siteNameShape = /^[A-Za-z][A-Za-z0-9-]*(?:\.[A-Za-z0-9-]+)*$/;
// Names the site files use that the code also uses as ordinary words ("corner"
// of a box); matching them would flag geometry, not the site.
const siteNameCommonWords = new Set(['corner']);
// Spellings of a site name that public identifiers keep on purpose: the build
// host's worker directory and workflow files (tools/build1-release-worker,
// .github/workflows/build1-*). Renaming them would move the installed worker.
const siteNameKeptSpellings = new Set(['build1']);

// siteNames lists the cell ids, hosts, logins and robot ids that the private
// site files under dir name: every rt-core/config/cells manifest (aliases/
// included), tools/public-mirror/site-names.txt, and every
// robot_description/robots entry that isPrivate keeps out of the mirror. A
// host is its first DNS label; placeholders and names under four characters
// are left out. Without the private files (the public mirror) it is empty.
export function siteNames(dir, isPrivate) {
  const names = new Set();
  const add = (value, login = false) => {
    if (typeof value !== 'string' || !siteNameShape.test(value)) return;
    const name = login ? value : value.split('.')[0];
    const lower = name.toLowerCase();
    if (name.length >= 4 && !placeholderUsers.has(lower) && !siteNameCommonWords.has(lower) && lower !== 'localhost') names.add(name);
  };
  const walkJson = (node) => {
    if (Array.isArray(node)) return node.forEach(walkJson);
    if (!node || typeof node !== 'object') return;
    for (const [key, value] of Object.entries(node)) {
      if (siteHostKeys.has(key)) add(value);
      else if (siteLoginKeys.has(key)) add(value, true);
      else if (key === 'devices' && value && typeof value === 'object' && !Array.isArray(value)) Object.keys(value).forEach((k) => add(k));
      walkJson(value);
    }
  };
  const cells = join(dir, 'rt-core/config/cells');
  const list = (sub) => (existsSync(join(cells, sub)) ? readdirSync(join(cells, sub)).map((f) => (sub ? `${sub}/${f}` : f)) : []);
  for (const file of [...list(''), ...list('aliases')]) {
    if (!file.endsWith('.json') || !isPrivate(`rt-core/config/cells/${file}`)) continue;
    add(basename(file, '.json'));
    let manifest;
    try { manifest = JSON.parse(readFileSync(join(cells, file), 'utf8')); } catch { continue; }
    if (manifest && typeof manifest === 'object') add(manifest.name);
    walkJson(manifest);
  }
  // Names no manifest carries (fleet machines, logins), one per line.
  const extra = 'tools/public-mirror/site-names.txt';
  if (existsSync(join(dir, extra)) && isPrivate(extra)) {
    for (const line of readFileSync(join(dir, extra), 'utf8').split('\n')) if (line.trim() && !line.startsWith('#')) add(line.trim(), true);
  }
  const robots = join(dir, 'robot_description/robots');
  for (const entry of existsSync(robots) ? readdirSync(robots) : []) {
    if (isPrivate(`robot_description/robots/${entry}/`)) add(entry);
  }
  return [...names].sort();
}

// sitePattern matches any of names as a whole word, any case: "host-2" in
// "host-2-arm9" counts (the longer id is a name too), "host-20" does not. A
// dash in a name matches a dash, underscore, space or nothing, so prose
// ("Host 2") and identifiers ("host2", "host_2") count, and a name after a dot
// counts too ("subject.host-2.command").
export function sitePattern(names) {
  if (!names.length) return null;
  // A dot may appear escaped, as in a regular expression in a test: a\.person.
  const alternatives = [...names].sort((a, b) => b.length - a.length)
    .map((n) => n.replace(/[.*+?^${}()|[\]\\]/g, '\\$&').replace(/\\\./g, '\\\\?\\.').replace(/-/g, '[-_ ]?'));
  // The allowlist names public files by path; a path is no new disclosure.
  return { id: 'site-name', skip: (path) => path === 'quality/leak-allowlist.txt', regex: new RegExp(`(?<![\\w])(?:${alternatives.join('|')})(?![\\w])`, 'gi'),
    ignore: (m) => siteNameKeptSpellings.has(m.toLowerCase()) };
}

// CAD source formats: a part or assembly, never published (STEP, IGES,
// Parasolid, ACIS, Rhino, Fusion, SolidWorks, Inventor, CATIA, Creo/NX).
export const cadFile = /\.(?:step|stp|iges|igs|x_t|x_b|sat|3dm|f3d|f3z|sldprt|sldasm|ipt|iam|catpart|catproduct|prt)$/i;
// Mesh formats: the public robot descriptions have their own; one inside an
// archive must be one of those.
export const meshFile = /\.(?:dae|stl|obj|ply|glb|gltf|fbx|3mf)$/i;
// Reviewed files the CAD and copy rules admit: the one STEP part the public
// app bundles (the Dog Box, offline-programming/v1/ui/scripts/step-examples.mjs)
// and the pedestal mesh every robot description shares, the private cells' too.
const reviewedCadOrMesh = new Set(['cadquery/v1/step/dog_box.step', 'robot_description/robots/rosie_1420_v1/meshes/pedestal.dae']);
const sha256 = (bytes) => createHash('sha256').update(bytes).digest('hex');

// archiveMembers lists a ZIP's or a gzip file's members as { name, bytes },
// or null when bytes are neither. ZIP members stored or deflated are read;
// any other method, or a damaged archive, is an error: what cannot be read
// cannot be cleared.
export function archiveMembers(bytes, name = '') {
  if (bytes.length >= 2 && bytes[0] === 0x1f && bytes[1] === 0x8b) {
    return [{ name: basename(name).replace(/\.(?:gz|tgz)$/i, '') || 'member', bytes: gunzipSync(bytes) }];
  }
  if (bytes.length < 22 || bytes.readUInt32LE(0) !== 0x04034b50) return null;
  // The end of central directory record is within the last 64 KiB + 22 bytes.
  let end = -1;
  for (let i = bytes.length - 22; i >= Math.max(0, bytes.length - 65557); i--) {
    if (bytes.readUInt32LE(i) === 0x06054b50) { end = i; break; }
  }
  if (end < 0) throw new Error('ZIP without a central directory');
  const count = bytes.readUInt16LE(end + 10);
  let at = bytes.readUInt32LE(end + 16);
  const members = [];
  for (let n = 0; n < count; n++) {
    if (bytes.readUInt32LE(at) !== 0x02014b50) throw new Error('damaged ZIP central directory');
    const method = bytes.readUInt16LE(at + 10);
    const packed = bytes.readUInt32LE(at + 20);
    const nameLength = bytes.readUInt16LE(at + 28), extra = bytes.readUInt16LE(at + 30), comment = bytes.readUInt16LE(at + 32);
    const local = bytes.readUInt32LE(at + 42);
    const member = bytes.subarray(at + 46, at + 46 + nameLength).toString('utf8');
    at += 46 + nameLength + extra + comment;
    if (member.endsWith('/')) continue;
    const start = local + 30 + bytes.readUInt16LE(local + 26) + bytes.readUInt16LE(local + 28);
    const data = bytes.subarray(start, start + packed);
    if (method === 0) members.push({ name: member, bytes: data });
    else if (method === 8) members.push({ name: member, bytes: inflateRawSync(data) });
    else throw new Error(`ZIP member ${member} uses compression method ${method}`);
  }
  return members;
}

// Absolute links into places only the team can open: this private
// repository's issues, releases and runs, and the team's Linear workspace. The
// public mirror (RosieOS-public) is not one of them.
const privateLinkTarget = /^https?:\/\/(?:www\.)?(?:github\.com\/advanced-metal-research\/RosieOS(?![-\w.])|linear\.app\/advancedmetalresearch\b)/i;

// privateLinkPattern matches a Markdown link, in a file the mirror publishes,
// into this private repository or Linear, or whose relative target the mirror
// leaves out (tracker.html, evidence/, a
// vendor PDF): in the public repository it is a dead link to a file nobody
// outside can read. Keep the reference as plain text, or move the passage to
// docs/site/. Without the private list (the mirror itself) it matches nothing.
export function privateLinkPattern(isPrivate) {
  const leftOut = (path, target) => {
    if (privateLinkTarget.test(target)) return true;
    if (/^[a-z][a-z0-9+.-]*:/i.test(target) || target.startsWith('/')) return false;
    let decoded;
    try { decoded = decodeURI(target); } catch { decoded = target; }
    const resolved = posix.normalize(posix.join(posix.dirname(path), decoded));
    return !resolved.startsWith('../') && (isPrivate(resolved) || isPrivate(resolved.replace(/\/?$/, '/')));
  };
  return { id: 'private-link', skip: (path) => !/\.md$/i.test(path), regex: /\[[^\]\n]*\]\(([^)\s#]+)(?:#[^)\s]*)?\)/g, ignore: (m, target, path) => !leftOut(path, target) };
}

export function hitsIn(text, path = "", list = patterns) {
  const found = new Set();
  for (const { id, regex, ignore, skip } of list) {
    if (skip && skip(path)) continue;
    for (const match of text.matchAll(regex)) {
      if (!ignore || !ignore(match[0], match[1], path)) { found.add(id); break; }
    }
  }
  return found;
}

// Gitignore-like subset: "dir/" is a prefix, "*" stays within one path segment,
// and a leading "**/" matches any number of directories, none included, so one
// rule can name a kind of file wherever it lives. A test's scenario declaration
// (<test>.scenarios.json) follows its test: the public qualification refuses a
// declaration whose test is missing, so naming the test is enough.
export function privateMatcher(lines) {
  const rules = lines.map((l) => l.trim()).filter((l) => l && !l.startsWith('#')).map((rule) => {
    const anywhere = rule.startsWith('**/');
    const escaped = (anywhere ? rule.slice(3) : rule).replace(/[.+^${}()|[\]\\]/g, '\\$&').replace(/\*/g, '[^/]*');
    const body = (anywhere ? '(?:[^/]+/)*' : '') + escaped;
    return new RegExp(rule.endsWith('/') ? `^${body}` : `^${body}$`);
  });
  const listed = (path) => rules.some((r) => r.test(path));
  return (path) => listed(path) || (path.endsWith('.scenarios.json') && listed(path.slice(0, -'.scenarios.json'.length)));
}

function trackedFiles() {
  const result = spawnSync('git', ['ls-files', '-z'], { cwd: root, encoding: 'utf8', maxBuffer: 64 << 20 });
  if (result.status !== 0) throw new Error(`git ls-files failed: ${result.stderr}`);
  return result.stdout.split('\0').filter(Boolean).map((p) => ({ path: p, file: join(root, p) }));
}

function walk(dir, base = dir, out = []) {
  for (const entry of readdirSync(dir, { withFileTypes: true })) {
    const file = join(dir, entry.name);
    if (entry.isDirectory()) { if (entry.name !== '.git') walk(file, base, out); }
    else if (entry.isFile()) out.push({ path: relative(base, file).split(sep).join('/'), file });
  }
  return out;
}

// scanBytes checks one file or archive member; members of an archive are
// checked the same way, nested archives up to three deep.
function scanBytes(path, bytes, list, fingerprints, hits, depth = 0) {
  if (!reviewedCadOrMesh.has(path)) {
    if (cadFile.test(path)) hits.push(`cad\t${path}`);
    if ((cadFile.test(path) || meshFile.test(path)) && fingerprints.private.has(sha256(bytes))) hits.push(`private-copy\t${path}`);
  }
  let members;
  try { members = archiveMembers(bytes, path); } catch (error) { hits.push(`archive\t${path}`); return; }
  if (members) {
    if (depth >= 3) { hits.push(`archive\t${path}`); return; }
    for (const member of members) {
      const inner = `${path}!${member.name}`;
      if (meshFile.test(member.name) && !fingerprints.public.has(sha256(member.bytes))) hits.push(`archive-mesh\t${inner}`);
      scanBytes(inner, member.bytes, list, fingerprints, hits, depth + 1);
    }
    return;
  }
  if (bytes.subarray(0, 8192).includes(0) || /\.(ppm|pgm|png|jpe?g|gif|webp|stl|dae|glb)$/i.test(path)) return; // binary or image data
  for (const id of hitsIn(bytes.toString('utf8'), path, list)) hits.push(`${id}\t${path}`);
}

// fingerprintsOf hashes the mesh and CAD files: the public ones (what an
// archive's mesh may equal) and the private ones (what nothing public may).
function fingerprintsOf(files, privateFiles) {
  const hashes = (list) => new Set(list.filter(({ path }) => meshFile.test(path) || cadFile.test(path)).map(({ file }) => {
    try { return sha256(readFileSync(file)); } catch { return ''; }
  }).filter(Boolean));
  return { public: hashes(files), private: hashes(privateFiles) };
}

// scan returns the "<rule>\t<path>" hits of the files the mirror would
// publish; privateFiles are the ones it leaves out, for the copy rule.
export function scan(files, isPrivate, list, privateFiles = []) {
  const hits = [];
  const published = files.filter(({ path }) => !isPrivate(path));
  const fingerprints = fingerprintsOf(published, privateFiles);
  for (const { path, file } of published) {
    let bytes;
    try { if (statSync(file).size > 64 << 20) continue; bytes = readFileSync(file); } catch { continue; } // deleted in the working tree
    scanBytes(path, bytes, list, fingerprints, hits);
  }
  return [...new Set(hits)].sort();
}

function main(argv) {
  const dirIndex = argv.indexOf('--dir');
  const strict = argv.includes('--strict');
  const update = argv.includes('--update');
  // The private paths decide what the mirror leaves out and where the site
  // names come from; a tree without the list (the mirror itself) has neither.
  const privatePaths = privateMatcher(existsSync(privatePathsPath) ? readFileSync(privatePathsPath, 'utf8').split('\n') : []);
  const isPrivate = dirIndex >= 0 ? () => false : privatePaths;
  const site = sitePattern(siteNames(root, privatePaths));
  const files = dirIndex >= 0 ? walk(argv[dirIndex + 1]) : trackedFiles();
  // The private files exist only in the full tree; the mirror has none to copy.
  const privateFiles = existsSync(privatePathsPath) ? trackedFiles().filter(({ path }) => privatePaths(path)) : [];
  const hits = scan(files, isPrivate, [...patterns, privateLinkPattern(privatePaths), ...(site ? [site] : [])], privateFiles);
  if (update) {
    const header = '# (pattern, path) pairs the leak check tolerates until each cleanup removes them.\n# Regenerate with: node quality/leak-check.mjs --update. Must be empty before the mirror is published.\n';
    writeFileSync(allowlistPath, header + hits.map((h) => `${h}\n`).join(''));
    console.log(`leak-check: allowlist rewritten with ${hits.length} entries`);
    return 0;
  }
  const allowed = strict ? new Set() : new Set(readFileSync(allowlistPath, 'utf8').split('\n').filter((l) => l && !l.startsWith('#')));
  const fresh = hits.filter((h) => !allowed.has(h));
  const seen = new Set(hits);
  const stale = [...allowed].filter((a) => !seen.has(a));
  for (const h of fresh) console.error(`leak: ${h.replace('\t', '  ')}`);
  if (stale.length) console.error(`leak-check: ${stale.length} allowlist entries no longer hit; shrink the list with --update`);
  const summary = Object.entries(hits.reduce((n, h) => ((n[h.split('\t')[0]] = (n[h.split('\t')[0]] ?? 0) + 1), n), {})).map(([k, v]) => `${k}=${v}`).join(' ');
  console.error(`leak-check: ${files.length} files, ${hits.length} hits (${summary || 'none'}), ${fresh.length} not allowlisted${strict ? ' (strict)' : ''}`);
  return fresh.length ? 1 : 0;
}

if (process.argv[1] && fileURLToPath(import.meta.url) === process.argv[1]) process.exitCode = main(process.argv.slice(2));
