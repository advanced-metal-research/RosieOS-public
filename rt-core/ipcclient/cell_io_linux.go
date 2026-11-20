//go:build linux

package ipcclient

import "context"

// IOTest is a bounded explicit sample. It never arms motion or renews itself.
func (c *Client) IOTest(ctx context.Context, applicationGeneration uint64, index uint32, on bool, sourceSequence uint64, begin ...bool) (uint64, error) {
	generation := c.revocation.Load()
	grant, err := c.FastObservation()
	if err != nil {
		return 0, err
	}
	value := uint32(0)
	if on {
		value = 1
	}
	if on && len(begin) > 0 && begin[0] {
		value |= 2
	}
	payload := EncodeIoTest(IoTest{NativeGeneration: grant.NativeGeneration, ApplicationGeneration: applicationGeneration,
		SourceSequence: sourceSequence, DeadlineNs: monotonicNS() + 200000000, Index: index, Value: value})
	seq, err := c.commandGeneration(ctx, generation, MsgIoTest, payload[:])
	if err == nil {
		_, err = c.waitResult(ctx, seq, generation)
	}
	return seq, err
}

// IOArm grants or removes native I/O permission under the exact application
// generation. A successful receipt is permission, never physical feedback.
func (c *Client) IOArm(ctx context.Context, applicationGeneration uint64, arm bool) (uint64, error) {
	generation := c.revocation.Load()
	grant, err := c.FastObservation()
	if err != nil {
		return 0, err
	}
	payload := EncodeIoRequest(IoRequest{NativeGeneration: grant.NativeGeneration, ApplicationGeneration: applicationGeneration})
	op := uint16(MsgIoDisarm)
	if arm {
		op = MsgIoArm
	}
	seq, err := c.commandGeneration(ctx, generation, op, payload[:])
	if err == nil {
		_, err = c.waitResult(ctx, seq, generation)
	}
	return seq, err
}
