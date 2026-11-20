export const TELEOP_CAPABILITY = Object.freeze({
  schema: "offline-programming.teleop-capability.v1" as const,
  enabled: false as const,
  reason: "reserved_for_future_server_implementation" as const,
  serverOwned: true as const,
  rawBrowserTransport: false as const,
});

export interface FutureTeleopSessionRequest {
  targetProfileId: string;
  requestedAxisCount: number;
  deadmanRequired: true;
  releaseZeroRequired: true;
  maximumInputAgeMs: number;
  maximumCommandRateHz: number;
  maximumAxisVelocityRadS: number[];
}

export interface FutureTeleopSession {
  sessionId: string;
  expiresAt: string;
}

// This interface represents an eventual server session API, not a browser
// UDP/NATS packet service. There is deliberately no hot-command send method.
export interface TeleopClient {
  requestSession(request: FutureTeleopSessionRequest): Promise<FutureTeleopSession>;
  closeSession(sessionId: string): Promise<void>;
}

export const TELEOP_DISABLED = "tele-op is disabled";

export class DisabledTeleopClient implements TeleopClient {
  async requestSession(_request: FutureTeleopSessionRequest): Promise<FutureTeleopSession> {
    throw new Error(TELEOP_DISABLED);
  }

  async closeSession(_sessionId: string): Promise<void> {
    throw new Error(TELEOP_DISABLED);
  }
}

// ServerHotCommandContract documents the future server-only boundary so it
// can be implemented without changing the project/plan contracts.
export interface ServerHotCommandContract {
  commandId: string;
  inputSequence: number;
  sampledAt: string;
  expiresAt: string;
  deadmanHeld: boolean;
  releaseZero: boolean;
  commandRateHz: number;
  axisVelocityRadS: number[];
  controllerBootId: string;
  leaseId: string;
  fenceEpoch: number;
}
