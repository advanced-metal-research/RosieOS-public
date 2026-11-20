// rt-core, the realtime OS: observation through rt-control's public API only.
// Every route here is a GET the API lists as an observation (APPLICATION_API.md
// "Operations and capability states"); no lease is taken and nothing is sent
// to the cyclic controller. Mutations go through OLP, which holds the grant.
import type { McpServer } from "@modelcontextprotocol/server";
import * as z from "zod/v4";

import type { Context } from "rosie-cli/lib/context.js";
import { deg, failure, guarded, maskAxes, responseFormat, result, table, yesNo } from "../format.js";
import { refusalText } from "rosie-cli/lib/http.js";

const READ_ONLY = { readOnlyHint: true, destructiveHint: false, idempotentHint: true, openWorldHint: true } as const;

interface AxisDescription {
  index: number;
  id: string;
  position_unit: string;
  min_position: number;
  max_position: number;
  max_velocity: number;
  require_home: boolean;
  native_home: boolean;
  completion_tolerance: number;
}

interface Description {
  backend: string;
  protocol_major: number;
  protocol_minor: number;
  configuration_sha256: string;
  machine_sha256: string;
  deployment_sha256: string;
  cycle_ns: number;
  axes: AxisDescription[];
  capabilities_digest?: string;
  capabilities?: Array<{ name: string; state: string; transport: string }>;
  robot?: Record<string, unknown> | null;
  drives?: Array<Record<string, unknown>>;
}

interface LogicalAxisStatus {
  readiness: string;
  logical_position: number;
  logical_valid: boolean;
  logical_target: number;
  home_valid: boolean;
  drive_alarm: string;
  drive_alarm_text: string;
  statusword: number;
  following_error_counts: number;
  following_error_valid: boolean;
  torque_raw: number;
  brake_state: number;
}

interface ProcessStatus {
  daemon_incarnation: string;
  adapter_incarnation: string;
  grant: { GrantActive: number; AxisMask: number } | null;
  core: {
    backend: string;
    configuration_sha256: string;
    armed: number;
    lease_valid: number;
    home_valid_mask: number;
    safety_fault_mask: number;
    execution_fault_reasons: number;
    axis_enable_mask: number;
    recovery: { reason: string; faults: Array<{ name: string; axis_mask: number; recovery: string; rehome_axis_mask: number }> } | null;
  };
  axes: LogicalAxisStatus[];
}

function radianUnit(unit: string | undefined): boolean {
  return unit === "rad" || unit === "radian" || unit === "radians";
}

export function registerRtCoreTools(server: McpServer, ctx: Context): void {
  const describe = async (): Promise<Description> => {
    const reply = await ctx.rt.get("/v1/describe");
    if (!reply.ok) throw new Error(`rt-control refused describe: ${refusalText(reply)}`);
    return reply.body as Description;
  };

  server.registerTool(
    "rosie_rtcore_describe",
    {
      title: "Describe the rt-core controller",
      description: `Read rt-control's Describe (GET /v1/describe): backend (simulation or ethercat), protocol version, the configuration/machine/deployment digests the controller runs, cycle period, every axis's unit, position and velocity limits and Home requirement, and the capabilities digest.

Read-only. Takes no lease and never contacts the cyclic controller beyond the adapter's own observation. Requires ROSIE_RT_CONTROL_SOCKET or ROSIE_RT_CONTROL_URL (+ CA/cert/key).

Use it to answer "which configuration does this controller run", "what are J5's limits", or to compare a cell's running pin with its manifest (rosie_get_cell_manifest).`,
      inputSchema: z.object({ response_format: responseFormat }).strict(),
      annotations: READ_ONLY,
    },
    guarded(async ({ response_format }) => {
      const d = await describe();
      return result(response_format, d, () => {
        const caps = d.capabilities ?? [];
        return [
          `# rt-core Describe (${ctx.rt.target})`,
          `- **Backend**: ${d.backend} · protocol ${d.protocol_major}.${d.protocol_minor} · cycle ${(d.cycle_ns / 1e6).toFixed(3)} ms`,
          `- **Configuration**: \`${d.configuration_sha256}\``,
          `- **Machine**: \`${d.machine_sha256}\` · **Deployment**: \`${d.deployment_sha256}\``,
          d.capabilities_digest ? `- **Capabilities digest**: \`${d.capabilities_digest}\` (${caps.filter((c) => c.state === "implemented").length}/${caps.length} implemented)` : "",
          d.robot ? `- **Robot**: ${JSON.stringify(d.robot)}` : "",
          "",
          table(
            ["Axis", "Unit", "Min", "Max", "Max velocity", "Home required", "Completion tol."],
            d.axes.map((a) => {
              const rad = radianUnit(a.position_unit);
              return [
                `${a.id} (#${a.index})`,
                a.position_unit,
                rad ? deg(a.min_position) : a.min_position,
                rad ? deg(a.max_position) : a.max_position,
                rad ? `${deg(a.max_velocity)}/s` : a.max_velocity,
                yesNo(a.require_home),
                rad ? deg(a.completion_tolerance) : a.completion_tolerance,
              ];
            }),
          ),
        ]
          .filter((line) => line !== "")
          .join("\n");
      });
    }),
  );

  server.registerTool(
    "rosie_rtcore_get_status",
    {
      title: "Get rt-core controller status",
      description: `Read rt-control's Status (GET /v1/status) and summarise the machine: armed, lease and grant, Home-valid and fault masks, latched fault recovery classes, and per axis its readiness, position, Home validity and any A6-EC drive alarm.

Read-only. Axis names and units come from a Describe read in the same call. Positions are the controller's logical positions (radians shown in degrees). Numbers above 2^53 (nanosecond clocks) lose precision in JSON; use them only for display.

When an axis shows a drive alarm (e.g. Er41.1), look it up with rosie_lookup_drive_alarm. Never treat a fault as cleared by re-homing: Home (HM35) is an operator action at the mechanical home pose only.`,
      inputSchema: z.object({ response_format: responseFormat }).strict(),
      annotations: READ_ONLY,
    },
    guarded(async ({ response_format }) => {
      const [statusReply, d] = await Promise.all([ctx.rt.get("/v1/status"), describe()]);
      if (!statusReply.ok) return failure(`rt-control refused status: ${refusalText(statusReply)}`);
      const s = statusReply.body as ProcessStatus;
      return result(response_format, s, () => {
        const core = s.core;
        const faults = core.recovery?.faults ?? [];
        return [
          `# rt-core status (${ctx.rt.target})`,
          `- **Backend**: ${core.backend} · configuration \`${core.configuration_sha256.slice(0, 12)}…\``,
          `- **Armed**: ${yesNo(core.armed)} · **lease valid**: ${yesNo(core.lease_valid)} · **grant active**: ${yesNo(s.grant?.GrantActive)}`,
          `- **Enabled axes**: ${maskAxes(core.axis_enable_mask)} · **Home valid**: ${maskAxes(core.home_valid_mask)}`,
          `- **Safety fault mask**: ${core.safety_fault_mask} (${maskAxes(core.safety_fault_mask)}) · **execution fault reasons**: 0x${core.execution_fault_reasons.toString(16)}`,
          faults.length
            ? `- **Latched faults**: ${faults.map((f) => `${f.name} on ${maskAxes(f.axis_mask)} → ${f.recovery}${f.rehome_axis_mask ? ` (Home lost on ${maskAxes(f.rehome_axis_mask)})` : ""}`).join("; ")}`
            : "- **Latched faults**: none",
          "",
          table(
            ["Axis", "Readiness", "Position", "Target", "Home", "Drive alarm"],
            s.axes.map((axis, i) => {
              const a = d.axes[i];
              const rad = radianUnit(a?.position_unit);
              return [
                a?.id ?? `#${i}`,
                axis.readiness,
                axis.logical_valid ? (rad ? deg(axis.logical_position) : axis.logical_position) : "invalid",
                rad ? deg(axis.logical_target) : axis.logical_target,
                yesNo(axis.home_valid),
                axis.drive_alarm ? `${axis.drive_alarm} ${axis.drive_alarm_text}` : "",
              ];
            }),
          ),
        ].join("\n");
      });
    }),
  );

  server.registerTool(
    "rosie_rtcore_get_maintenance",
    {
      title: "Get the servo host's maintenance notice",
      description: `Read rt-control's maintenance observation (GET /v1/maintenance): whether the deploy tooling's maintenance notice is on the servo host (arming, Home, jog and motion starts are then refused with "busy"; Stop, Release and Halt pass) and which controller holds the grant (never its session).

Read-only. Use it to tell who has the robot before anyone acts, or why an Arm was refused "busy".`,
      inputSchema: z.object({ response_format: responseFormat }).strict(),
      annotations: READ_ONLY,
    },
    guarded(async ({ response_format }) => {
      const reply = await ctx.rt.get("/v1/maintenance");
      if (!reply.ok) return failure(`rt-control refused maintenance: ${refusalText(reply)}`);
      return result(response_format, reply.body, () => `# rt-control maintenance (${ctx.rt.target})\n\n\`\`\`json\n${JSON.stringify(reply.body, null, 2)}\n\`\`\``);
    }),
  );

  server.registerTool(
    "rosie_rtcore_list_events",
    {
      title: "List rt-core events",
      description: `Read rt-control's event log (GET /v1/events?after=<sequence>): grant changes, handle transitions, executions, jog ends, faults, brake warnings, drive supervisor diagnostics, enable/arm, Home epochs and incarnation changes, oldest first.

Args:
  - after (integer): return events with sequence greater than this (default 0 = from the oldest retained).
  - limit (1-200): most events to return (default 50).
Returns next_sequence: pass it as 'after' to continue. An 'events_dropped' entry means the adapter's ring lost events; reconcile with rosie_rtcore_get_status before drawing conclusions.`,
      inputSchema: z
        .object({
          after: z.number().int().min(0).default(0).describe("Return events after this sequence number"),
          limit: z.number().int().min(1).max(200).default(50).describe("Maximum number of events to return"),
          response_format: responseFormat,
        })
        .strict(),
      annotations: READ_ONLY,
    },
    guarded(async ({ after, limit, response_format }) => {
      const reply = await ctx.rt.get(`/v1/events?after=${after}`);
      if (!reply.ok) return failure(`rt-control refused events: ${refusalText(reply)}`);
      const batch = reply.body as { events?: Array<Record<string, unknown>>; next_sequence?: number; latest_sequence?: number };
      const events = (batch.events ?? []).slice(0, limit);
      const last = events.at(-1);
      const data = {
        count: events.length,
        has_more: (batch.events?.length ?? 0) > events.length || (batch.latest_sequence ?? 0) > Number(last?.sequence ?? after),
        next_sequence: last ? Number(last.sequence) : batch.next_sequence ?? after,
        latest_sequence: batch.latest_sequence ?? null,
        events,
      };
      return result(response_format, data, () => {
        if (events.length === 0) return `No rt-core events after sequence ${after} (latest ${data.latest_sequence ?? "unknown"}).`;
        const detail = (event: Record<string, unknown>) => {
          const type = String(event.type);
          const payload = event[type] ?? Object.fromEntries(Object.entries(event).filter(([k]) => !["sequence", "time_ns", "type", "daemon_incarnation", "adapter_incarnation"].includes(k)));
          return JSON.stringify(payload);
        };
        return [
          `# rt-core events after ${after} (${events.length}${data.has_more ? `, more from ${data.next_sequence}` : ""})`,
          "",
          table(["Seq", "Type", "Detail"], events.map((event) => [event.sequence, event.type, detail(event)])),
        ].join("\n");
      });
    }),
  );
}
