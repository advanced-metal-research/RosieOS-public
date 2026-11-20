// drives.html: the drive health page renders one row per joint from the
// OLP's drive-health route, puts controller text in as text, builds its 30 s
// and 60 s figures from the route's 1 s windows, and shows the route's
// refusal instead of an empty table (docs/decisions/0001).
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { Window } from "happy-dom";

const html = readFileSync(new URL("../drives.html", import.meta.url), "utf8");
const script = html.slice(html.indexOf("<script>") + "<script>".length, html.lastIndexOf("</script>"));

type Reply = { status: number; body: unknown };

// Loads the page (JavaScript evaluation is off in happy-dom by default) and
// runs its script against a stubbed fetch that answers each reply in turn;
// the wait after the last one never resolves, so each reply is read once.
async function render(replies: Reply[]) {
  const browser = new Window({ url: "http://127.0.0.1:5189/offline-programming/v1/ui/drives.html" });
  browser.document.write(html);
  const requests: string[] = [];
  const fetch = async (url: string) => {
    const reply = replies[requests.length]!;
    requests.push(url);
    return { ok: reply.status === 200, status: reply.status, json: async () => reply.body };
  };
  const wait = (resolve: () => void) => { if (requests.length < replies.length) setImmediate(resolve); };
  const run = new Function("window", "document", "fetch", "performance", "setTimeout", script);
  run(browser, browser.document, fetch, performance, wait);
  for (let i = 0; i < 10 * replies.length; i++) await new Promise((resolve) => setImmediate(resolve));
  const rows = [...browser.document.querySelectorAll("tbody tr")];
  const cells = (row: (typeof rows)[number]) => [...row.querySelectorAll("td")].map((td) => td.textContent.trim().replace(/\s+/g, " "));
  return { document: browser.document, requests, rows, cells };
}

function joint(name: string, rms: number | null, peak: number | null, fe: number | null, extra: object = {}) {
  return { name, readiness: "ready", window_ms: 1000, torque_rms_percent: rms, torque_peak_percent: peak,
    following_error_unit: "deg", following_error_peak: fe,
    monitor: [{ key: "heatsink_temperature", unit: "degC", value: 34.5, age_ms: 812 }], ...extra };
}

test("TestDrivesPageRendersEachJoint", async () => {
  const { document, requests, rows, cells } = await render([{
    status: 200,
    body: {
      cell: "Rosie 1430 rosie_1420_v1",
      time_ns: 1,
      joints: [
        joint("J1", 12.34, 45.6, 0.25),
        joint("J2", null, null, null, { readiness: "drive_alarm", drive_alarm: "Er41.1", drive_alarm_text: "<img src=x onerror=alert(1)>",
          monitor: [{ key: "heatsink_temperature", unit: "degC", value: 40.1, age_ms: 60000 }] }),
      ],
    },
  }]);
  assert.deepEqual(requests, ["/api/offline-programming/v1/dense-execution/drive-health"]);
  assert.equal(rows.length, 2);
  const ready = rows[0]!, alarmed = rows[1]!;
  assert.deepEqual(cells(ready), ["J1", "94.1 °F 34.5 °C", "", "12.3 / 45.6 %", "12.3 %", "45.6 %", "", "0.250 deg", "0.250 deg", "", "ready"]);
  assert.match(document.querySelector("#cell")!.textContent, /Rosie 1430 rosie_1420_v1/);
  // An old reading is shown as stale, an empty window as a dash.
  assert.ok(alarmed.querySelector(".stale"));
  assert.equal(cells(alarmed)[3], "– / – %");
  // Controller text is text: the alarm detail renders literally, no element.
  assert.equal(alarmed.querySelector(".alarm")!.textContent, "Er41.1");
  assert.ok(cells(alarmed)[10]!.includes("<img src=x onerror=alert(1)>"));
  assert.equal(document.querySelector("img"), null);
  // An Er code is a fault, named in the banner.
  const banner = document.querySelector("#alarms")!;
  assert.equal(banner.hasAttribute("hidden"), false);
  assert.equal(banner.getAttribute("data-state"), "fault");
  assert.match(banner.textContent, /^J2: Er41\.1/);
  // One second of data ranks no joint.
  assert.equal(document.querySelector(".top"), null);
});

test("TestDrivesPageBuildsLongerWindowsFromSeconds", async () => {
  // 40 one-second windows: J1 steady at 10 % with one 70 % peak 35 s ago,
  // J2 steady at 20 % with an AL warning. A repeated status time adds nothing.
  const replies: Reply[] = [];
  for (let s = 0; s < 40; s++) {
    const body = { time_ns: (s + 1) * 1e9, joints: [
      joint("J1", 10, s === 4 ? 70 : 12, s === 4 ? 0.5 : 0.1),
      joint("J2", 20, 25, 0.2, { drive_alarm: "ALFA.0", drive_alarm_text: "Drive high temperature warning" }),
    ] };
    replies.push({ status: 200, body });
    if (s === 20) replies.push({ status: 200, body });
  }
  const { document, rows, cells } = await render(replies);
  const [j1, j2] = [cells(rows[0]!), cells(rows[1]!)];
  assert.equal(j1[4], "10.0 %");  // 30 s RMS
  assert.equal(j1[5], "70.0 %");  // the 60 s peak still holds the peak from 35 s ago
  assert.equal(j1[8], "0.500 deg");
  assert.equal(j2[4], "20.0 %");
  assert.equal(rows[1]!.querySelector(".top")!.textContent, "Most loaded");
  assert.equal(rows[0]!.querySelector(".top"), null);
  assert.equal(rows[1]!.querySelector(".warning")!.textContent, "ALFA.0");
  assert.equal(document.querySelector("#alarms")!.getAttribute("data-state"), "warning");
});

test("TestDrivesPageShowsTheRefusal", async () => {
  const { document } = await render([{ status: 503, body: { error: "drive_health_unavailable", detail: "the cell's controller status is not available" } }]);
  assert.equal(document.querySelector("table")!.hidden, true);
  assert.equal(document.querySelector("#message")!.textContent, "the cell's controller status is not available");
});
