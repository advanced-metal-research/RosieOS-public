//go:build linux

package ipcclient

import "context"

// Halt requests controlled hold under the exact mirrored application generation.
// Its receipt proves native admission; motion completion remains an observation.
func (c *Client) Halt(ctx context.Context, applicationGeneration uint64) (uint64, error) {
	generation := c.revocation.Load()
	grant, err := c.FastObservation()
	if err != nil {
		return 0, err
	}
	payload := EncodeHaltRequest(HaltRequest{NativeGeneration: grant.NativeGeneration, ApplicationGeneration: applicationGeneration})
	sequence, err := c.commandGeneration(ctx, generation, MsgHalt, payload[:])
	if err == nil {
		_, err = c.waitResult(ctx, sequence, generation)
	}
	return sequence, err
}
