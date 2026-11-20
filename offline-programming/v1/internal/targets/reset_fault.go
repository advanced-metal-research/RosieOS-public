package targets

import (
	"context"

	"rosieos/offline-programming/v1/internal/denseexec"
)

func (p *Picker) ResetFault(ctx context.Context) (*denseexec.ArmResult, error) {
	return withBackend(p, func(b denseexec.Backend) (*denseexec.ArmResult, error) {
		reset, ok := b.(interface {
			ResetFault(context.Context) (*denseexec.ArmResult, error)
		})
		if !ok {
			return nil, &denseexec.Error{Status: 409, Code: "reset_fault_unsupported", Detail: "selected backend does not support disarmed fault reset"}
		}
		return reset.ResetFault(ctx)
	})
}
