import type { OfflineProgrammingProject } from "./project.ts";
import { currentDescriptionSha256, parseRobotSettings, robotSettingsSnapshot,
  type RobotSettings, type RobotSettingsSnapshot } from "./robots.ts";

export interface RobotSettingsReview {
  title: string;
  /** Date only when backed by a dated change record, never a load or pin time. */
  date: string | null;
  provenance: string;
  rows: { label: string; before: string; after: string }[];
  unchanged: string[];
  explanation: string | null;
  technical: { modelId: string; from: string; to: string; commit?: string };
}

/** Audited from the complete description-directory diff in this commit, not
 * inferred from a hash prefix or filename. Keep outside identity-bound robot
 * resources so adding explanatory history does not change robot identity.
 * robot.urdf changes ONLY J7/J8/J9 velocity; rtcore_definition changes ONLY
 * those gear ratios and revision/source notes; spheres changes ONLY its URDF
 * provenance. All other registered description resources are unchanged. */
const POSITIONER_CORRECTION = {
  modelId: "rosie_1400_v3",
  from: "sha256:803ab172c4e2e7f93f33162d3060a51cc86014747b7617864090f09aaeb9ecd7",
  to: "sha256:686029493a224a51a506d7ae0a056f9080d941167e64911cc03c7d6ac1ed24a9",
  commit: "29658345a222864ab9eece859061cce44b10834f",
  date: "2026-09-21T23:32:45+10:00",
};

export function robotSettingsReview(project: OfflineProgrammingProject): RobotSettingsReview {
  const modelId = project.robot.model;
  const from = project.robot.description?.robotDescriptionSha256 ?? "";
  const to = currentDescriptionSha256(modelId);
  const technical = { modelId, from, to };
  const base: RobotSettingsReview = {
    title: "Robot settings changed", date: null, provenance: "Exact robot description identities",
    rows: [], unchanged: [], explanation: null, technical,
  };
  if (!from || !to || from === to) return {
    ...base, title: from && to ? "Robot settings are current" : "Robot settings comparison unavailable",
    explanation: from && to ? "This program already uses the current robot settings."
      : "The saved or current robot description identity is unavailable, so its settings cannot be compared.",
  };
  if (modelId === POSITIONER_CORRECTION.modelId && from === POSITIONER_CORRECTION.from && to === POSITIONER_CORRECTION.to) {
    return {
      ...base, title: "Positioner gearing and rated speeds changed", date: POSITIONER_CORRECTION.date,
      provenance: "Repository change recorded on 21 September 2026 at 23:32:45 +10:00. This is the commit date, not the deployment date.",
      rows: [
        { label: "Robot definition revision", before: "6", after: "7" },
        { label: "J7, J8 and J9 gear ratio", before: "10:1", after: "100:1" },
        { label: "J7, J8 and J9 rated joint speed", before: "1,800 °/s", after: "180 °/s" },
      ],
      unchanged: ["Joint position limits", "Robot geometry and collision shapes", "Torch configuration", "Description planning speed and acceleration settings"],
      explanation: "The positioner gearing was corrected. Its rated joint speeds now match the 100:1 reduction. Review these changes before adopting the current description and planning again.",
      technical: { ...technical, commit: POSITIONER_CORRECTION.commit },
    };
  }
  const saved = project.robot.description?.settingsSnapshot;
  const savedMatches = saved?.modelId === modelId && saved.robotDescriptionSha256 === from;
  const savedSnapshot = savedMatches ? sanitizeSnapshot(saved) : undefined;
  const before = savedSnapshot ?? robotSettingsSnapshot(modelId, from);
  const after = robotSettingsSnapshot(modelId, to);
  if (!before || !after) return {
    ...base,
    title: after ? "Current settings (previous values unavailable)" : "Robot settings comparison unavailable",
    rows: after ? currentRows(after) : [],
    provenance: "No complete settings comparison is available for these exact robot descriptions.",
    explanation: !before
      ? "This program has no usable saved settings snapshot, and the exact previous description is not available. The individual changes cannot be reconstructed. Any values shown are current settings, not confirmed changes."
      : "The current connected robot's exact settings are not available in the catalogue. The workstation's different version cannot be used to explain this change.",
  };
  const result = compare(before, after);
  return {
    ...base, ...result,
    provenance: savedSnapshot ? "Compared settings saved with this program against the current exact robot description. Change date is not recorded."
      : "Compared the catalogue's exact saved and current robot descriptions. Change date is not recorded.",
    explanation: result.rows.length ? "Review the changed settings before adopting the current description and planning again."
      : "The description identity changed, but the available settings do not show the underlying change. No change date is recorded.",
  };
}

function sanitizeSnapshot(snapshot: RobotSettingsSnapshot): RobotSettingsSnapshot | undefined {
  const settings = parseRobotSettings(snapshot.settings);
  const frames = snapshot.frames;
  if (!settings || typeof frames !== "object" || frames === null) return undefined;
  if (![frames.base, frames.tool, frames.work].every(value => typeof value === "string")) return undefined;
  if (![frames.workWorldM, frames.armBaseWorldM].every(value => Array.isArray(value) && value.length === 3 &&
    value.every(component => typeof component === "number" && Number.isFinite(component)))) return undefined;
  return { ...snapshot, settings };
}

/** These are available current values, not an asserted change list. */
function currentRows(snapshot: RobotSettingsSnapshot): RobotSettingsReview["rows"] {
  const empty = { ...snapshot, settings: { joints: [] }, frames: {} } as unknown as RobotSettingsSnapshot;
  // Empty joint entries let the shared formatter describe every numeric field
  // while retaining each joint's correct angular or linear units.
  empty.settings.joints = snapshot.settings.joints.map(({ name, type }) => ({ name, type }));
  return compare(empty, snapshot).rows.map(row => ({
    ...row, before: "Not recorded", after: row.label.startsWith("Description file:") ? "Current version available" : row.after,
  }));
}

function compare(before: RobotSettingsSnapshot, after: RobotSettingsSnapshot): Pick<RobotSettingsReview, "rows" | "unchanged"> {
  const rows: RobotSettingsReview["rows"] = [], unchanged: string[] = [];
  const row = (label: string, left: unknown, right: unknown, format: (value: unknown) => string = plain) => {
    if (JSON.stringify(left) !== JSON.stringify(right)) rows.push({ label, before: format(left), after: format(right) });
  };
  const a = before.settings, b = after.settings;
  row("Robot definition revision", a.revision, b.revision);
  const names = [...new Set([...a.joints, ...b.joints].map(joint => joint.name))];
  for (const name of names) {
    const left = a.joints.find(joint => joint.name === name), right = b.joints.find(joint => joint.name === name);
    if (!left || !right) { row(`${name} joint`, left?.type, right?.type); continue; }
    const linear = left.type === "prismatic" && right.type === "prismatic";
    const length = (value: unknown) => unit(value, linear ? 1000 : 180 / Math.PI, linear ? "mm" : "°");
    const speed = (value: unknown) => unit(value, linear ? 1000 : 180 / Math.PI, linear ? "mm/s" : "°/s");
    row(`${name} joint type`, left.type, right.type);
    row(`${name} lower position limit`, left.lower, right.lower, length);
    row(`${name} upper position limit`, left.upper, right.upper, length);
    row(`${name} rated joint speed`, left.rated_velocity, right.rated_velocity, speed);
    row(`${name} gear ratio`, left.gear_ratio, right.gear_ratio, value => value === undefined ? "Not recorded" : `${plain(value)}:1`);
  }
  row("Planning speed ceiling", a.planning?.speed_ceiling_rad_s, b.planning?.speed_ceiling_rad_s, value => unit(value, 180 / Math.PI, "°/s"));
  for (const name of new Set([...Object.keys(a.planning?.joints ?? {}), ...Object.keys(b.planning?.joints ?? {})])) {
    row(`${name} planning speed`, a.planning?.joints[name]?.velocity_rad_s, b.planning?.joints[name]?.velocity_rad_s, value => unit(value, 180 / Math.PI, "°/s"));
    row(`${name} planning acceleration`, a.planning?.joints[name]?.acceleration_rad_s2, b.planning?.joints[name]?.acceleration_rad_s2, value => unit(value, 180 / Math.PI, "°/s²"));
  }
  row("Torch tool frame", a.torch?.tool_frame, b.torch?.tool_frame);
  row("Torch electrode direction", a.torch?.electrode_axis, b.torch?.electrode_axis);
  for (const key of ["base", "tool", "work"] as const) row(`${key[0]!.toUpperCase()}${key.slice(1)} frame`, before.frames[key], after.frames[key]);
  for (const [key, label] of [["workWorldM", "Work frame origin"], ["armBaseWorldM", "Arm base origin"]] as const) {
    row(label, before.frames[key], after.frames[key], value => Array.isArray(value) ? value.map(component => unit(component, 1000, "mm")).join(", ") : "Not recorded");
  }
  const resources = [...new Set([...Object.keys(a.resources ?? {}), ...Object.keys(b.resources ?? {})])];
  for (const name of resources) if (a.resources?.[name] !== b.resources?.[name]) rows.push({
    label: `Description file: ${name}`, before: a.resources?.[name] ? "Saved version" : "Not recorded",
    after: b.resources?.[name] ? "Changed version" : "Not recorded",
  });
  const limits = (settings: RobotSettings) => settings.joints.map(({ name, type, lower, upper }) => ({ name, type, lower, upper })).sort((a, b) => a.name.localeCompare(b.name));
  if (names.length && JSON.stringify(limits(a)) === JSON.stringify(limits(b))) unchanged.push("Recorded joint position limits");
  if (a.planning && b.planning && JSON.stringify(a.planning) === JSON.stringify(b.planning)) unchanged.push("Recorded planning speed and acceleration settings");
  if (a.torch && b.torch && JSON.stringify(a.torch) === JSON.stringify(b.torch)) unchanged.push("Recorded torch configuration");
  return { rows, unchanged };
}

function plain(value: unknown): string {
  if (value === undefined || value === null) return "Not recorded";
  if (typeof value === "number") return Number.isFinite(value) ? String(Number(value.toFixed(3))) : "Not recorded";
  return Array.isArray(value) ? value.map(plain).join(", ") : String(value);
}

function unit(value: unknown, scale: number, suffix: string): string {
  return typeof value === "number" && Number.isFinite(value) ? `${plain(value * scale)} ${suffix}` : "Not recorded";
}
