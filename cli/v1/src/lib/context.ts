// Process-wide state, shared by every front end in the process: the MCP
// server (serveStdio may build more than one McpServer from the factory, one
// per protocol era) and the CLI. The clients, the heartbeat keeper and the
// open program live here, once.
import type { Config } from "./config.js";
import { HeartbeatKeeper } from "./heartbeat.js";
import { RtControlClient } from "./http.js";
import { OlpClient } from "./olp.js";
import { ProgramSession } from "./program.js";

export interface Context {
  config: Config;
  olp: OlpClient;
  rt: RtControlClient;
  keeper: HeartbeatKeeper;
  program: ProgramSession;
}

export function createContext(config: Config, options: { clientId?: string } = {}): Context {
  const olp = new OlpClient(config.olpUrl, config.clientLabel, config.requestTimeoutMs, options.clientId);
  return {
    config,
    olp,
    rt: new RtControlClient(config.rtControl, config.requestTimeoutMs),
    keeper: new HeartbeatKeeper(olp, config.heartbeatWindowS * 1000),
    program: new ProgramSession(config, olp),
  };
}
