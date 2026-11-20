import type { JointMotionProfile, TcpMotionProfile } from "../domain/motion-profile.ts";
import { telemetryColor } from "../execution/telemetryStrip.ts";
import { el } from "../rails/dom.ts";

export interface PlannedMoveProfile {
  profile: JointMotionProfile;
  tcp: TcpMotionProfile;
  current: boolean;
  planRevision: number;
  planSpeed: number | null;
  jointNames: string[];
}

const number = (value: number): string => Number(value.toPrecision(4)).toString();
const peak = (values: readonly number[]): number => values.reduce((max, value) => Math.max(max, Math.abs(value)), 0);

export function tcpProfileChart(data: PlannedMoveProfile): HTMLElement {
  const { tcp } = data;
  return el("section", { class: "move-profile", "aria-label": "Planned TCP motion profile" },
    el("div", { class: "move-profile-heading" }, el("strong", {}, "TCP MOTION PROFILE"),
      el("span", {}, `${number(data.profile.durationS)} s · plan rev ${data.planRevision}${data.planSpeed === null ? "" : ` · plan speed ${number(data.planSpeed * 100)}%`}`)),
    el("div", { class: "move-profile-plots" },
      profilePlot("TCP speed", "mm/s", tcp.timesS, [tcp.speedMmS], [0], true),
      profilePlot("TCP acceleration", "mm/s²", tcp.timesS, [tcp.accelerationMmS2], [0])),
    el("div", { class: "move-profile-summary" }, `Peak speed ${number(peak(tcp.speedMmS))} mm/s · Peak |acceleration| ${number(peak(tcp.accelerationMmS2))} mm/s²`),
    el("p", { class: "muted" }, "TCP translation from the planned path, sampled using robot kinematics. Acceleration is the change in speed; negative values show deceleration."));
}

/** Two plots on one time axis, using the same joint colours as telemetry. */
export function moveProfileChart(data: PlannedMoveProfile): HTMLElement {
  const { profile } = data;
  const axes = data.jointNames.map(name => ({ name, index: Number(name.slice(1)) - 1 }))
    .filter(axis => axis.index >= 0 && axis.index < profile.velocityDegS.length);
  const visible = new Set(axes.filter(axis => peak(profile.velocityDegS[axis.index]!) > 1e-7).map(axis => axis.index));
  if (!visible.size && axes[0]) visible.add(axes[0].index);
  const plots = el("div", { class: "move-profile-plots" });
  const summary = el("div", { class: "move-profile-summary" });
  const legend = el("div", { class: "move-profile-legend", role: "group", "aria-label": "Profile joints" });
  const root = el("section", { class: "move-profile", "aria-label": "Planned joint motion profile" },
    el("div", { class: "move-profile-heading" },
      el("strong", {}, "JOINT MOTION PROFILE"),
      el("span", {}, `${number(profile.durationS)} s · plan rev ${data.planRevision}${data.planSpeed === null ? "" : ` · plan speed ${number(data.planSpeed * 100)}%`}`)),
    legend, plots, summary,
    el("p", { class: "muted" }, "Velocity is from the planned trajectory. Acceleration is derived between its velocity samples. Negative values indicate joint direction."));
  for (const axis of axes) {
    const button = el("button", { type: "button", "aria-pressed": String(visible.has(axis.index)),
      on: { click: () => {
        if (visible.has(axis.index)) visible.delete(axis.index); else visible.add(axis.index);
        button.setAttribute("aria-pressed", String(visible.has(axis.index)));
        draw();
      } } }, axis.name);
    button.style.setProperty("--joint-color", telemetryColor(axis.index));
    legend.append(button);
  }
  function draw(): void {
    plots.replaceChildren(profilePlot("Joint velocity", "°/s", profile.timesS, profile.velocityDegS, [...visible]), profilePlot("Joint acceleration", "°/s²", profile.timesS, profile.accelerationDegS2, [...visible]));
    const max = (values: number[][]) => Math.max(0, ...[...visible].map(axis => peak(values[axis]!)));
    summary.textContent = `Peak |velocity| ${number(max(profile.velocityDegS))} °/s · Peak |acceleration| ${number(max(profile.accelerationDegS2))} °/s²`;
  }
  draw();
  return root;
}

function profilePlot(label: string, unit: string, timesS: number[], series: number[][], visible: number[], positive = false): SVGSVGElement {
    const durationS = timesS.at(-1)!;
    const svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
    svg.setAttribute("viewBox", "0 0 900 200");
    svg.setAttribute("role", "img");
    svg.setAttribute("aria-label", `${label} over ${number(durationS)} seconds`);
    const add = (tag: string, attrs: Record<string, string | number>, text?: string) => {
      const child = document.createElementNS(svg.namespaceURI, tag);
      for (const [key, value] of Object.entries(attrs)) child.setAttribute(key, String(value));
      if (text !== undefined) child.textContent = text;
      svg.append(child);
    };
    const max = Math.max(1e-6, ...[...visible].map(axis => peak(series[axis]!))) * 1.1;
    const x = (i: number) => 65 + timesS[i]! / durationS * 815;
    const y = (value: number) => positive ? 165 - value / max * 130 : 100 - value / max * 65;
    add("text", { x: 8, y: 15, class: "profile-axis-label" }, `${label} · ${unit}`);
    for (const value of positive ? [0, max / 2, max] : [-max, 0, max]) {
      add("line", { x1: 65, x2: 880, y1: y(value), y2: y(value), class: "profile-grid" });
      add("text", { x: 58, y: y(value) + 4, "text-anchor": "end" }, number(value));
    }
    for (let tick = 0; tick <= 4; tick++) {
      const px = 65 + 815 * tick / 4;
      add("line", { x1: px, x2: px, y1: 35, y2: 165, class: "profile-grid" });
      add("text", { x: px, y: 186, "text-anchor": "middle" }, `${number(durationS * tick / 4)} s`);
    }
    for (const axis of visible) {
      const values = series[axis]!;
      // Keep minima/maxima in each pixel bucket so short ramps are visible.
      const indices = new Set([0, values.length - 1]);
      const stride = Math.max(1, Math.ceil(values.length / 815));
      for (let a = 0; a < values.length; a += stride) {
        let lo = a, hi = a;
        for (let i = a + 1; i < Math.min(a + stride, values.length); i++) {
          if (values[i]! < values[lo]!) lo = i;
          if (values[i]! > values[hi]!) hi = i;
        }
        indices.add(lo); indices.add(hi);
      }
      const d = [...indices].sort((a, b) => a - b).map((i, n) => `${n ? "L" : "M"}${x(i).toFixed(2)},${y(values[i]!).toFixed(2)}`).join(" ");
      add("path", { d, fill: "none", stroke: telemetryColor(axis), "stroke-width": 1.8 });
    }
    return svg;
}
