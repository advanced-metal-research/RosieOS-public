// What every command receives: the global flags, the resolved target and a
// context built on first use, so `rosie alarm Er41.1` never touches the
// network and `rosie status` builds its clients once.
import { createContext, type Context } from "../lib/context.js";
import { clientId, resolveTarget, rosieHome, type Resolved } from "./targets.js";

export interface Globals {
  json: boolean;
  target?: string;
  workspace: string;
  quiet: boolean;
}

export class Env {
  private resolvedTarget?: Promise<Resolved>;
  private context?: Promise<Context>;

  constructor(
    readonly globals: Globals,
    readonly processEnv: NodeJS.ProcessEnv = process.env,
  ) {}

  get home(): string {
    return rosieHome(this.processEnv);
  }

  resolved(): Promise<Resolved> {
    this.resolvedTarget ??= resolveTarget(this.globals.target, this.processEnv);
    return this.resolvedTarget;
  }

  ctx(): Promise<Context> {
    this.context ??= (async () => createContext((await this.resolved()).config, { clientId: await clientId(this.processEnv) }))();
    return this.context;
  }

  async repoRoot(): Promise<string> {
    return (await this.resolved()).config.repoRoot;
  }

  /** Progress for long steps, on stderr so stdout stays the answer (and --json stays parseable). */
  progress(message: string): void {
    if (!this.globals.quiet) process.stderr.write(`· ${message}\n`);
  }

  /** Stop timers the context started (the heartbeat keeper) so the process can exit. */
  async close(): Promise<void> {
    if (this.context) (await this.context.catch(() => null))?.keeper.stop("command ended");
  }
}
