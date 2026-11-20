package denseexec

import "context"

// Backend preserves the operator HTTP contract across execution transports;
// selecting a backend cannot silently fall back to another motion owner.
type Backend interface {
	Available() bool
	Close(context.Context)
	Capabilities(context.Context) Capabilities
	Status(context.Context) *StatusBody
	Load(context.Context, LoadRequest) (*LoadResult, error)
	Play(context.Context, string) (*StatusBody, error)
	Pause(context.Context, string) (*StatusBody, error)
	Stop(context.Context) (*StopResult, error)
	Arm(context.Context, bool) (*ArmResult, error)
	GoHome(context.Context) (*GoHomeResult, error)
	Heartbeat(string) (*HeartbeatResult, error)
	// The newest seconds of the machine's telemetry ring, every stride-th
	// record (rt_core_telemetry.go).
	TelemetryWindow(context.Context, float64, int) (*TelemetryWindow, error)
}

// ControlStatusReader optionally obtains current control evidence without
// fetching historical execution handles. Status remains the full diagnostic
// read, and backends without this optimization retain their existing behavior.
type ControlStatusReader interface {
	ControlStatus(context.Context) *StatusBody
}
