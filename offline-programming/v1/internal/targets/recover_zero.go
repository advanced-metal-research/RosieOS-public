package targets

import (
	"context"

	"rosieos/offline-programming/v1/internal/denseexec"
)

// RecoverToZero keeps the selected backend stable for the whole press: its
// confirm, Reset, Arm and move all reach the cell the operator saw.
func (p *Picker) RecoverToZero(ctx context.Context, in denseexec.RecoverToZeroRequest) (*denseexec.RecoverToZeroResult, error) {
	return withBackend(p, func(b denseexec.Backend) (*denseexec.RecoverToZeroResult, error) {
		backend, ok := b.(denseexec.RecoverToZeroBackend)
		if !ok {
			return nil, &denseexec.Error{Status: 409, Code: "capability_unimplemented", Detail: "Recover to zero needs an rt-core cell"}
		}
		return backend.RecoverToZero(ctx, in)
	})
}
