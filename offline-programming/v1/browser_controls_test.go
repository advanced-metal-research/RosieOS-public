package main

import (
	"os/exec"
	"testing"
)

// runBrowserContract executes the shipped TypeScript in Node so Go review probes
// catch browser behaviour regressions without maintaining a second implementation.
func runBrowserContract(t *testing.T, script string) {
	t.Helper()
	node, err := exec.LookPath("node")
	if err != nil {
		t.Skip("browser contract requires Node with TypeScript stripping")
	}
	cmd := exec.Command(node, "--experimental-strip-types", "--import", "./ui/test/support/seed.ts", "--input-type=module", "-e", script)
	if output, err := cmd.CombinedOutput(); err != nil {
		t.Fatalf("browser contract: %v\n%s", err, output)
	}
}

// TestReviewBrowserRequestIdentity retains the rendered SIM target until the UI
// applies a new status; fetching REAL alone must not retarget an old ARM click.
func TestReviewBrowserRequestIdentity(t *testing.T) {
	runBrowserContract(t, `
import assert from 'node:assert/strict';
import { DenseExecutionClient } from './ui/src/execution/denseClient.ts';
const sent = [];
const client = new DenseExecutionClient(async (url, init) => {
  sent.push({url, init});
  return new Response(JSON.stringify({backend: 'rt_core', target: {cell_id: 'real', selection_generation: '2'}}));
});
const displayed = {cell_id: 'sim', selection_generation: '1'};
client.observeTarget(displayed);
displayed.cell_id = 'mutated';
const refreshed = await client.status();
for (const action of [() => client.arm(true), () => client.home(), () => client.goHome(), () => client.play('s')]) {
  await action();
  assert.equal(sent.at(-1).init.headers['X-RT-Target-Cell'], 'sim');
  assert.equal(sent.at(-1).init.headers['X-RT-Target-Generation'], '1');
}

client.observeTarget(refreshed.target);
await client.arm(true);
assert.equal(sent.at(-1).init.headers['X-RT-Target-Cell'], 'real');
assert.equal(sent.at(-1).init.headers['X-RT-Target-Generation'], '2');
assert.equal(sent.length, 6);
`)
}

// TestReviewArmControls renders the current Arm fact and target posture,
// preventing browser preview modes from relabelling physical ARM authority.
func TestReviewArmControls(t *testing.T) {
	runBrowserContract(t, `
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { setImmediate } from 'node:timers/promises';
import { mock } from 'node:test';
import { machineStatePill } from './ui/src/execution/machineFlow.ts';
import { DenseExecutionClient } from './ui/src/execution/denseClient.ts';
import { DensePanel } from './ui/src/execution/densePanel.ts';
import { CellTargetsClient } from './ui/src/targets/cells.ts';
import { createDogBoxProject } from './ui/src/examples/dog-box.ts';
class Element extends EventTarget {
  innerHTML = ''; textContent = ''; title = ''; value = ''; hidden = false; disabled = false;
  dataset = {}; options = []; nodes = new Map();
  classList = {toggle: (_name, force) => force ?? false};
  querySelector(s) { if (!this.nodes.has(s)) this.nodes.set(s, new Element()); return this.nodes.get(s); }
  append(...o) { this.options.push(...o); }
  setAttribute() {}
  replaceChildren() { this.options = []; this.textContent = ''; }
  remove() {}
}
mock.timers.enable({apis: ['setTimeout']});
globalThis.window = new EventTarget();
globalThis.document = {createElement: () => new Element(), addEventListener() {}, removeEventListener() {}, querySelector: () => new Element()};
globalThis.sessionStorage = {getItem: () => '', setItem: () => {}};
const target = {selection_generation: '2', cell_id: 'example-bench-1420', label: 'Test bench (Rosie 1420, six motors)', role: 'bench',
  models: ['rosie_1420_v1'], model_id: 'rosie_1420_v1', requested_mode: 'real', backend: 'ethercat', simulation: false, host: '127.0.0.1:8443'};
const client = new DenseExecutionClient(async url => Response.json(String(url).endsWith('/status') ? {
  backend: 'rt_core', available: true, target, rt_core: {status: {time_ns: 1, adapter_incarnation: 'a', daemon_incarnation: 'd',
  axes: [], core: {armed: 1, safety_fault_mask: 0}, execution: {state: 'idle', error: ''}}}
} : {backend: 'rt_core', available: true}));
const cells = new CellTargetsClient(async () => Response.json({targets: [target], selected: {cell_id: target.cell_id, model_id: target.model_id}}));
const panel = new DensePanel({client, cells, getProject: () => createDogBoxProject(),
  highlightNode() {}, status() {}, onLockChange() {}, onTargetChange() {}, onRobotPose() {}, onRobotMotion() {}, wantsFastPoll: () => false});
const host = new Element();
panel.mount(host, host);
await setImmediate();
mock.timers.tick(0);
await setImmediate();
const arm = host.querySelector('#dense-arm-state');
assert.equal(arm.textContent, 'ARMED');
assert.equal(arm.dataset.armed, 'armed');
assert.equal(arm.dataset.posture, 'real');
assert.match(arm.title, /REAL target/);
assert.equal(host.querySelector('#dense-arm-on').textContent, 'Arm Test bench (Rosie 1420, six motors)');
for (const mode of ['simulation', 'dry_run']) {
  const pill = machineStatePill(panel.flowView(), {executionMode: mode, robotLive: false, connectedArmed: false});
  assert.match(pill.label, /^REAL · Test bench \(Rosie 1420, six motors\) · ARMED$/);
  assert.equal(pill.className, 'is-real');
  assert.doesNotMatch(pill.label, /SIM/);
  assert.match(pill.title, /real machine/);
}
const css = readFileSync('./ui/src/style.css', 'utf8');
assert.match(css, /\.dense-arm-state\[data-armed="armed"\]\[data-posture="real"\]\s*\{/);
assert.doesNotMatch(css, /data-armed="true"/);
assert.match(css, /\.mode-badge\.is-real\b[^{}]*\{[^}]*color:\s*var\(--(?:caution|danger)\)/);
mock.timers.reset();
`)
}
