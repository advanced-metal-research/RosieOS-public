package targets

import (
	"context"

	"rosieos/offline-programming/v1/internal/denseexec"
)

// RunProgram keeps the selected backend stable for the whole press: its
// Arm, Go to plan start, Load and Play all reach the cell the operator saw.
func (p *Picker) RunProgram(ctx context.Context, in denseexec.RunProgramRequest) (*denseexec.RunProgramResult, error) {
	return withBackend(p, func(b denseexec.Backend) (*denseexec.RunProgramResult, error) {
		backend, ok := b.(denseexec.RunProgramBackend)
		if !ok {
			return nil, &denseexec.Error{Status: 409, Code: "capability_unimplemented", Detail: "Run program needs an rt-core cell"}
		}
		return backend.RunProgram(ctx, in)
	})
}

// FinishLoop asks the selected backend to end its running loop at the
// program start.
func (p *Picker) FinishLoop(ctx context.Context) (*denseexec.LoopStatus, error) {
	return withBackend(p, func(b denseexec.Backend) (*denseexec.LoopStatus, error) {
		backend, ok := b.(interface {
			FinishLoop(context.Context) (*denseexec.LoopStatus, error)
		})
		if !ok {
			return nil, &denseexec.Error{Status: 409, Code: "capability_unimplemented", Detail: "Loops need an rt-core cell"}
		}
		return backend.FinishLoop(ctx)
	})
}

// HeartbeatFrom names the client behind a heartbeat for the selected
// backend's attended loop; a backend without loops gets the plain beat.
func (p *Picker) HeartbeatFrom(ctx context.Context, id string) (*denseexec.HeartbeatResult, error) {
	return withBackend(p, func(b denseexec.Backend) (*denseexec.HeartbeatResult, error) {
		if from, ok := b.(interface {
			HeartbeatFrom(context.Context, string) (*denseexec.HeartbeatResult, error)
		}); ok {
			return from.HeartbeatFrom(ctx, id)
		}
		return b.Heartbeat(id)
	})
}
