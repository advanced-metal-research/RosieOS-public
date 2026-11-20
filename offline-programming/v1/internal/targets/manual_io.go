//go:build linux

package targets

import (
	"context"
	"rosieos/offline-programming/v1/internal/denseexec"
)

func (p *Picker) ManualIO(ctx context.Context, in denseexec.ManualIORequest) error {
	_, err := withBackend(p, func(b denseexec.Backend) (bool, error) {
		io, ok := b.(interface {
			ManualIO(context.Context, denseexec.ManualIORequest) error
		})
		if !ok {
			return false, &denseexec.Error{Status: 409, Code: "io_test_unavailable", Detail: "Selected controller does not support manual I/O testing"}
		}
		return true, io.ManualIO(ctx, in)
	})
	return err
}

func (p *Picker) BeginManualIO(ctx context.Context) (string, error) {
	return withBackend(p, func(b denseexec.Backend) (string, error) {
		io, ok := b.(interface {
			BeginManualIO(context.Context) (string, error)
		})
		if !ok {
			return "", &denseexec.Error{Status: 409, Code: "io_test_unavailable", Detail: "Selected controller does not support manual I/O testing"}
		}
		return io.BeginManualIO(ctx)
	})
}
