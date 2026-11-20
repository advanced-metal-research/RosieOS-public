// Every rosie command, by group. A group is what `rosie <group> --help` lists.
import type { Command } from "./command.js";
import { commands as authoring } from "./commands/authoring.js";
import { commands as cell } from "./commands/cell.js";
import { commands as program } from "./commands/program.js";
import { commands as robot } from "./commands/robot.js";
import { commands as skills } from "./commands/skills.js";
import { commands as traj } from "./commands/traj.js";
import { commands as view } from "./commands/view.js";
import { commands as system } from "./commands/system.js";
import type { Env } from "./env.js";

export const GROUPS: ReadonlyArray<{ name: string; summary: string }> = [
  { name: "doctor", summary: "every service and setup trap, with fixes" },
  { name: "status", summary: "who has the robot, SIMULATION or REAL, armed, session" },
  { name: "target", summary: "where rosie points (local stack, a cell's tunnel)" },
  { name: "cell", summary: "cells: list, show, diagnose, alarm lookup, events, describe, discover" },
  { name: "docs", summary: "search and read the component documentation" },
  { name: "deploy", summary: "read-only release review and deploy inspection (the owner deploys)" },
  { name: "program", summary: "the workspace's OLP program: new, open, show, edit rows, waypoints, plan, check, save, undo" },
  { name: "part", summary: "STEP parts: import (seam detection), place in the work frame, switch, remove" },
  { name: "seam", summary: "detected weld seams: list, search torch angles, accept as welds" },
  { name: "weld", summary: "weld angle profiles; waypoint, line and arc welds from torch poses" },
  { name: "rails", summary: "the fitted torch and the angle search" },
  { name: "fixture", summary: "workcell fixtures: stand box, STEP obstacles" },
  { name: "view", summary: "LOOK at it: 3D scene snapshots from OLP's own renderer (PNG)" },
  { name: "report", summary: "self-contained HTML reports: program (views, checks, path), cell" },
  { name: "run", summary: "run the planned program: preview the moves, then --confirm (simulation by default)" },
  { name: "robot", summary: "stop, disarm, pause, finish; select, arm, load, play; bounded jogs; watch; keepalive" },
  { name: "traj", summary: "dense trajectories (.rdt): inspect, daemon validate, fetch; plan a .weldplan" },
  { name: "planner", summary: "the weld planner: CUDA or CPU, revision, plan in flight" },
  { name: "kin", summary: "kinematics: FK, would a straight Cartesian line be accepted" },
  { name: "skills", summary: "the rosie agent skills: install for every Claude Code session on this machine, list, uninstall" },
];

export function allCommands(): Command<Env>[] {
  return [...system, ...cell, ...program, ...authoring, ...view, ...robot, ...traj, ...skills];
}
