import assert from "node:assert/strict";
import test from "node:test";
import { cellOutputsFromStatus } from "./outputs.ts";

test("the Run page's outputs from a status poll while gas is ON, nothing for a cell without outputs", () => {
  const body = {
    backend: "rt_core",
    session: { id: "ds-1", kind: "trajectory", state: "playing", dry_run: false, process_outputs: ["gas"] },
    rt_core: {
      description: { io: { torch_qualified: false, outputs: [{ name: "gas", class: "process", expiry_ns: 200000000 }] } },
      status: { core: { io: { time_ns: 2000000000, cycle: 2000, armed: true, reason: 0, inputs: [], outputs: [
        { name: "gas", intent: true, commanded: true, readback: true, valid: true, expiry_ns: 2150000000, observed_ns: 0,
          changed_ns: 0, marker_sample_ns: 0, marker_cycle: 0, bit: 0, torch: false },
      ] } } },
    },
  };
  const view = cellOutputsFromStatus(body);
  assert.equal(view.visible, true);
  assert.equal(view.tone, "on");
  assert.deepEqual(view.lines, ["Outputs armed · this program switches gas", "gas: ON · readback ON · expires in 150 ms unless refreshed"]);
  assert.equal(cellOutputsFromStatus({ backend: "rt_core", rt_core: { status: { core: {} } } }).visible, false);
});
