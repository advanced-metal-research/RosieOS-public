//go:build linux

package ipcclient

import "context"

// FaultReset submits an inhibited native reset; callers must await its result
// before treating faults as cleared. A zero mask selects all configured axes.
func (c *Client) FaultReset(ctx context.Context, axisMask uint32) (uint64, error) {
	if axisMask == 0 {
		axisMask = (uint32(1) << c.axes) - 1
	}
	return c.maskCommand(ctx, MsgFaultReset, axisMask)
}
