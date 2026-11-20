import assert from "node:assert/strict";
import test from "node:test";
import { Window } from "happy-dom";

const browser = new Window();
Object.assign(globalThis, {
  window: browser,
  document: browser.document,
  HTMLElement: browser.HTMLElement,
  Node: browser.Node,
  KeyboardEvent: browser.KeyboardEvent,
  MouseEvent: browser.MouseEvent,
  Event: browser.Event,
  localStorage: browser.localStorage,
});

const { RobotDialog, validateCellInput } = await import("./robotDialog.ts");

function click(el: Element): void {
  el.dispatchEvent(new browser.MouseEvent("click", { bubbles: true }) as unknown as MouseEvent);
}

const benchCell = {
  id: "bench-relay-dense", label: "Test cell", address: "https://127.0.0.1:8443",
  host: "127.0.0.1", credentialRef: "bench-relay", modelId: "rosie_1400_v3",
  backend: "ethercat", configurationDigest: "pinned", source: "commissioned" as const,
};
const devCell = {
  ...benchCell, id: "dev-cell", label: "Local simulation", address: "",
  host: "localhost", credentialRef: "", backend: "simulation",
};

test("the cell list marks the selected cell, describes the rest, and preserves commissioned entries", () => {
  const calls: string[] = [];
  const card = new RobotDialog({
    onClose: () => calls.push("close"),
    onSelect: (id) => calls.push(`select:${id}`),
    onRefresh: () => calls.push("refresh"),
    onRemove: (id) => calls.push(`remove:${id}`),
    onAdd: async () => true,
    onPull: () => calls.push("pull"),
  });
  card.mount(document.body);
  card.renderCells({
    cells: [devCell, benchCell],
    probes: [{ ...devCell, reachable: true, error: null }, { ...benchCell, reachable: false, error: "dial tcp: timeout" }],
    activeId: "dev-cell",
    daemonReachable: true,
    selecting: false,
    status: "Joint daemon · dev-cell via 127.0.0.1:8797 · reachable",
  });
  const rows = [...card.el.querySelectorAll<HTMLElement>(".robot-cell")];
  assert.deepEqual(rows.map((row) => row.dataset.cell), ["", "dev-cell", "bench-relay-dense"]);
  assert.deepEqual(rows.map((row) => row.dataset.reach), ["none", "up", "down"]);
  assert.ok(rows[1]!.classList.contains("is-selected"));
  assert.equal(rows[1]!.querySelector(".robot-cell-source")?.textContent, "commissioned");
  // Decommissioning is outside the dialog contract.
  assert.equal(rows[1]!.querySelector(".robot-cell-remove"), null);
  assert.equal(rows[2]!.querySelector(".robot-cell-remove"), null);
  click(rows[2]!.querySelector(".robot-cell-pick")!);
  click(rows[1]!.querySelector(".robot-cell-pick")!);
  click(rows[0]!.querySelector(".robot-cell-pick")!);
  assert.deepEqual(calls, ["select:bench-relay-dense", "select:"], "the selected row is not re-selected");
  card.renderSettings({ held: null, heldCell: null, warning: null, canPull: false, pullReason: "Connect to a cell first" });
  const pull = card.el.querySelector<HTMLButtonElement>(".robot-dialog-pull")!;
  assert.equal(pull.disabled, true);
  assert.equal(pull.title, "Connect to a cell first");
  card.renderSettings({ held: "rosie_1400_v3 · on bench-relay", heldCell: "bench-relay-dense", warning: "Connected to dev-cell, not bench-relay-dense.", canPull: true, pullReason: "" });
  assert.equal(pull.disabled, false);
  assert.match(card.el.querySelector(".robot-dialog-held")!.textContent ?? "", /From bench-relay-dense: rosie_1400_v3/);
  assert.equal(card.el.querySelector<HTMLElement>(".inspector-note.is-warn")!.hidden, false);
  click(pull);
  assert.equal(calls.at(-1), "pull");
});

test("the Add cell form validates before the round trip and clears on success", async () => {
  const added: unknown[] = [];
  const card = new RobotDialog({
    onClose: () => undefined,
    onSelect: () => undefined,
    onRefresh: () => undefined,
    onRemove: () => undefined,
    onAdd: async (input) => {
      added.push(input);
      return input.id !== "refused";
    },
    onPull: () => undefined,
  });
  card.mount(document.body);
  const input = (key: string): HTMLInputElement => card.el.querySelector<HTMLInputElement>(`input[data-cell-field="${key}"]`)!;
  const submit = card.el.querySelector<HTMLButtonElement>(".robot-add-submit")!;
  const type = (key: string, value: string): void => {
    input(key).value = value;
    input(key).dispatchEvent(new browser.Event("input", { bubbles: true }) as unknown as Event);
  };
  assert.equal(submit.disabled, true, "nothing to add yet");
  type("id", "bench 9");
  type("address", "https://127.0.0.1:8443");
  type("host", "127.0.0.1");
  type("credentialRef", "bench-relay");
  assert.equal(submit.disabled, false);
  click(submit);
  await Promise.resolve();
  assert.equal(added.length, 0, "an invalid id never reaches the server");
  assert.match(card.el.querySelector(".robot-add-error")!.textContent ?? "", /Cell id/);
  type("id", "bench-relay-dense");
  click(submit);
  await new Promise((resolve) => setTimeout(resolve, 0));
  assert.equal(added.length, 1);
  assert.deepEqual(added[0], {
    id: "bench-relay-dense", address: "https://127.0.0.1:8443", host: "127.0.0.1", credentialRef: "bench-relay",
  });
  assert.equal(input("id").value, "", "the form clears once the server took the cell");
  assert.equal(card.el.querySelector<HTMLElement>(".robot-add-error")!.hidden, true);
});

test("cell input validation requires commissioned HTTPS metadata", () => {
  const ok = { id: "bench-relay", address: "https://127.0.0.1:8443", host: "127.0.0.1", credentialRef: "bench-relay" };
  assert.equal(validateCellInput(ok), null);
  assert.match(validateCellInput({ ...ok, id: "-bad" }) ?? "", /Cell id/);
  assert.match(validateCellInput({ ...ok, address: "http://127.0.0.1:8443" }) ?? "", /HTTPS/);
  assert.match(validateCellInput({ ...ok, host: "unknown" }) ?? "", /HTTPS/);
  assert.match(validateCellInput({ ...ok, credentialRef: "" }) ?? "", /PKI/);
});

test("Search local network lists discovered cells, flags incompatible ones and Add only fills the form for confirmation", async () => {
  const { discoveredRows } = await import("../targets/discoveredCells.ts");
  const added: unknown[] = [];
  const sections: boolean[] = [];
  const searches: boolean[] = [];
  const card = new RobotDialog({
    onClose: () => undefined,
    onSelect: () => undefined,
    onRefresh: () => undefined,
    onRemove: () => undefined,
    onAdd: async (input) => { added.push(input); return true; },
    onPull: () => undefined,
    onNetworkSearch: (force) => searches.push(force),
    onNetworkSectionOpen: (open) => sections.push(open),
  });
  card.mount(document.body);
  const here = { configured: false, pin: "unknown" as const, known_manifest: true, commissioned_address: "https://127.0.0.1:28443", commissioned_host: "127.0.0.1", reason: "Not in this server's cell catalogue" };
  const rows = discoveredRows({
    schema: "offline-programming.discovered-cells.v1", enabled: true, sources: [], servers: [{
      instance: "bench-relay-olp-123456", hostname: "bench-relay", address: "192.168.50.140", port: 8794, reach: "loopback",
      build_id: "git:0123abcd", started_at: "", maintenance: false, source: "mdns", health: "not_probed",
      cells: [
        { cell_id: "example-bench-1400", label: "Bench", model: "bench_nine_motors_1to1", mode: "real", capabilities_digest: "b".repeat(64), here },
        { cell_id: "made-up", label: "Made up", model: "rosie_1420_v1", mode: "real", here: { configured: false, pin: "unknown", known_manifest: false, reason: "No manifest for this cell id in rt-core/config/cells" } },
      ],
    }],
  });
  card.openNetworkSection();
  assert.deepEqual(sections, [true], "opening the section starts live refresh");
  card.renderDiscovery({ rows, status: "2 cells on 1 OLP server via mDNS on wlan0", searching: false });
  const found = [...card.el.querySelectorAll<HTMLElement>(".robot-discovery-row")];
  assert.equal(found.length, 2);
  assert.equal(found[0]!.dataset.compatibility, "incompatible");
  assert.ok([...found[0]!.querySelectorAll(".robot-discovery-badge.is-bad")].some((b) => b.textContent === "Incompatible"));
  assert.ok([...found[0]!.querySelectorAll(".robot-discovery-badge")].some((b) => b.textContent === "Unverified"));
  const unknownAdd = found[1]!.querySelector<HTMLButtonElement>(".robot-discovery-add")!;
  assert.equal(unknownAdd.disabled, true);
  assert.match(unknownAdd.title, /No manifest/);

  click(card.el.querySelector<HTMLButtonElement>(".robot-discovery-search")!);
  assert.deepEqual(searches, [true], "SEARCH forces a new search");
  click(found[0]!.querySelector(".robot-discovery-add")!);
  await new Promise((resolve) => setTimeout(resolve, 0));
  assert.equal(added.length, 0, "Add from discovery never submits by itself");
  const value = (key: string): string => card.el.querySelector<HTMLInputElement>(`input[data-cell-field="${key}"]`)!.value;
  assert.deepEqual([value("id"), value("host"), value("address"), value("credentialRef")],
    ["example-bench-1400", "127.0.0.1", "https://127.0.0.1:28443", "example-bench-1400"]);
  const note = card.el.querySelector<HTMLElement>(".robot-add-prefill")!;
  assert.equal(note.hidden, false);
  assert.match(note.textContent ?? "", /press ADD TO REGISTRY to confirm.*Flagged: incompatible/);

  click(card.el.querySelector(".robot-add-submit")!);
  await new Promise((resolve) => setTimeout(resolve, 0));
  assert.deepEqual(added, [{ id: "example-bench-1400", address: "https://127.0.0.1:28443", host: "127.0.0.1", credentialRef: "example-bench-1400" }]);
  assert.equal(note.hidden, true, "the note goes with the form once the server took the cell");
});
