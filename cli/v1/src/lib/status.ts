// The operator-facing summary of OLP's machine status (GET
// dense-execution/status), shared by every front end: whoever asks, the
// selected cell reads SIMULATION or REAL ROBOT the same way.
import type { DenseStatus } from "./olp.js";
import { deg, record, table, yesNo } from "./text.js";

/** GET dense-execution/status as bullet lines and an axis table. */
export function describeStatus(status: DenseStatus): string {
  const t = status.target;
  const s = status.session;
  const lines = [
    t
      ? `- **Cell**: ${t.label || t.cell_id} (\`${t.cell_id}\`, ${t.model_id ?? "model ?"}) on ${t.host ?? "?"} · backend ${t.backend ?? "?"}${t.simulation ? " · **SIMULATION**" : t.simulation === false ? " · **REAL ROBOT**" : ""} · selection ${t.selection_generation}`
      : "- **Cell**: none selected",
    `- **Available**: ${yesNo(status.available)} · **Armed**: ${yesNo(status.armed)}`,
    s
      ? `- **Session**: \`${s.id}\` ${s.kind}/${s.state}${s.dry_run ? " (dry run)" : ""} · lease ${s.lease?.held ? `held (${s.lease.expires_in_ms} ms)` : "not held"} · heartbeat ${s.heartbeat_age_ms ?? "?"}/${s.heartbeat_deadline_ms ?? "?"} ms${s.trajectory_digest ? ` · trajectory \`${s.trajectory_digest.slice(0, 19)}…\`` : ""}${s.stop_reason ? ` · stop reason: ${s.stop_reason}` : ""}${s.last_error ? ` · last error: ${s.last_error}` : ""}`
      : "- **Session**: none",
  ];
  const cursor = record(status.cursor);
  if (cursor.total_sample_count) {
    lines.push(`- **Cursor**: segment ${cursor.segment_index}/${cursor.segment_count} (${cursor.kind} ${cursor.source_id}) · ${Number(cursor.global_t_s).toFixed(2)} / ${Number(cursor.total_duration_s).toFixed(2)} s`);
  }
  const loop = record(status.loop);
  if (loop.mode) lines.push(`- **Loop**: ${loop.mode} ${loop.state} · ${loop.cycles} cycles${loop.end_reason ? ` · ended: ${loop.end_reason}` : ""}`);
  const jogEnd = record(status.last_jog_end);
  if (jogEnd.text || jogEnd.code) lines.push(`- **Last jog end**: ${String(jogEnd.text ?? jogEnd.code)}`);
  if (status.daemon_error) lines.push(`- **Daemon error**: ${status.daemon_error}`);
  if (Array.isArray(status.axes) && status.axes.length) {
    const axes = status.axes.map(record);
    lines.push(
      "",
      table(
        ["Axis", "Readiness", "Position", "Home", "Drive alarm"],
        axes.map((a, i) => [
          String(a.id ?? a.name ?? `J${i + 1}`),
          String(a.readiness ?? a.state ?? ""),
          typeof a.position_rad === "number" ? deg(a.position_rad) : typeof a.logical_position === "number" ? deg(a.logical_position) : String(a.position ?? ""),
          yesNo(a.home_valid),
          a.drive_alarm ? `${String(a.drive_alarm)} ${String(a.drive_alarm_text ?? "")}` : "",
        ]),
      ),
    );
  }
  return lines.join("\n");
}

