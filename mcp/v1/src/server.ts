import { McpServer } from "@modelcontextprotocol/server";

import type { Context } from "rosie-cli/lib/context.js";
import { registerPrompts } from "./prompts.js";
import { registerResources } from "./resources.js";
import { registerDeployTools } from "./tools/deploy.js";
import { registerJogTools } from "./tools/jog.js";
import { registerMotionTools } from "./tools/motion.js";
import { registerOlpTools } from "./tools/olp.js";
import { registerPlannerTools } from "./tools/planner.js";
import { registerProgramTools } from "./tools/program.js";
import { registerRepoTools } from "./tools/repo.js";
import { registerRobotTools } from "./tools/robot.js";
import { registerRtCoreTools } from "./tools/rtcore.js";

export const SERVER_NAME = "rosieos-mcp-server";
export const SERVER_VERSION = "0.1.0";

const INSTRUCTIONS = `RosieOS controls a welding robot cell: rt-core (the native realtime controller, reached through rt-control's public API), the motion servers (Cartesian daemon and dense trajectory daemon), offline programming (OLP, the controller of record that holds the robot's grant) and the weld planner.

Programs (rosie_program_*) are edited by OLP's own TypeScript engine, the one the browser and the Steam Deck run: new/open (cell store or file), view the spreadsheet, every row's fields, taught waypoints from joint angles, dwell/IO/home rows, STEP import with weld seam detection, torch-angle search and weld rows, part placement, plan through OLP's /weld-plan, save to the cell store (revision-checked) or a file. Authoring and planning move nothing.

Read before you act: rosie_olp_get_control and rosie_olp_get_machine_status say who has the robot and whether the selected cell is a SIMULATION or a REAL ROBOT. Robot actions go through OLP only. Stop (rosie_olp_stop) is always available. Starting motion needs ROSIE_MCP_ROBOT_ACTIONS (now: see the rosieos://mcp/config resource); with it, jogging is bounded per call (rosie_olp_move_joints, rosie_olp_cartesian_step, rosie_olp_jog_hold: travel, speed and hold time capped here before OLP's own limits). Home, fault reset and recover-to-zero are never offered: Home is an operator action at the mechanical home pose, never a recovery. Report refusals in OLP's own words; do not work around them. The documents (rosie_search_docs, rosieos://docs/*) carry the measured numbers and owner rules behind every constant.`;

export function createServer(ctx: Context): McpServer {
  const server = new McpServer({ name: SERVER_NAME, version: SERVER_VERSION }, { instructions: INSTRUCTIONS });
  registerRtCoreTools(server, ctx);
  registerOlpTools(server, ctx);
  registerRobotTools(server, ctx);
  registerJogTools(server, ctx);
  registerMotionTools(server, ctx);
  registerPlannerTools(server, ctx);
  registerProgramTools(server, ctx);
  registerRepoTools(server, ctx);
  registerDeployTools(server, ctx);
  registerResources(server, ctx);
  registerPrompts(server, ctx);
  return server;
}
