//go:build linux

package control

import (
	"context"
	"encoding/json"
	"errors"
	"rosieos/rt-core/ipcclient"
	"sort"
)

type ioNative interface {
	IOArm(context.Context, uint64, bool) (uint64, error)
}

type ioTestNative interface {
	IOTest(context.Context, uint64, uint32, bool, uint64, ...bool) (uint64, error)
}

func (n *restartNative) IOTest(ctx context.Context, generation uint64, index uint32, on bool, sourceSequence uint64, begin ...bool) (uint64, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	return n.client.IOTest(ctx, generation, index, on, sourceSequence, begin...)
}
func (c *Controller) IOTest(f Fence, name string, on bool, sourceSequence uint64, begin ...bool) (uint64, error) {
	if on {
		if err := c.checkMaintenance(OperationIoTest); err != nil {
			return 0, err
		}
	}
	return c.operation(f, func(ctx context.Context) (uint64, error) {
		native, ok := c.core.(ioTestNative)
		if !ok {
			return 0, errors.New(ReasonCapabilityUnimplemented)
		}
		state, err := c.core.Status()
		if err != nil {
			return 0, err
		}
		for index, output := range state.IO.Outputs {
			if output.Name != name {
				continue
			}
			seq, err := native.IOTest(ctx, f.Generation, uint32(index), on, sourceSequence, begin...)
			return seq, publicIOError(err)
		}
		return 0, errors.New(ReasonIoNotConfigured)
	})
}

func (n *restartNative) IOArm(ctx context.Context, generation uint64, arm bool) (uint64, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	return n.client.IOArm(ctx, generation, arm)
}

// Bind marker names against native output metadata and exact normalised samples;
// reject duplicates and torch aliases before publishing any prepared handle.
func (c *Controller) bindIOMarkers(program *Program) error {
	if !program.RequiresProcessIO {
		return nil
	}
	state, err := c.core.Status()
	if err != nil {
		return err
	}
	if len(state.IO.Outputs) == 0 && len(state.IO.Inputs) == 0 {
		return errors.New(ReasonIoNotConfigured)
	}
	if err := bindInputMonitors(program, state.IO); err != nil {
		return err
	}
	for _, m := range program.Markers {
		if m.Monitor != "" {
			continue
		}
		if m.TimeoutMS != 0 {
			index := sort.Search(len(program.points), func(i int) bool { return int64(program.points[i].Time) >= m.TimeNS })
			if m.StableMS > m.TimeoutMS || index >= len(program.points)-1 || int64(program.points[index].Time) != m.TimeNS || program.points[index].WaitTimeoutNS != 0 {
				return errors.New(ReasonIoMarkerInvalid)
			}
			input := -1
			for i, in := range state.IO.Inputs {
				if in.Name == m.Action {
					input = i
					break
				}
			}
			if input < 0 {
				return errors.New(ReasonIoNotConfigured)
			}
			p := &program.points[index]
			p.WaitInput = uint32(input)
			if m.Value {
				p.WaitValue = 1
			}
			p.WaitTimeoutNS = uint64(m.TimeoutMS) * 1000000
			p.WaitStableNS = uint64(m.StableMS) * 1000000
			continue
		}
		if m.StableMS != 0 {
			return errors.New(ReasonIoMarkerInvalid)
		}
		if m.Action == "torch" && !m.Value && !program.torch {
			continue
		} // Ordinary dry programs retain the corpus OFF marker.
		output := -1
		for i, o := range state.IO.Outputs {
			if o.Name == m.Action || (m.Action == "torch" && o.Torch && program.torchQualified) {
				if o.Torch && !program.torchQualified {
					return errors.New(ReasonIoTorchUnqualified)
				}
				output = i
				break
			}
		}
		if output < 0 || output >= 16 {
			return errors.New(ReasonIoMarkerInvalid)
		}
		index := sort.Search(len(program.points), func(i int) bool { return int64(program.points[i].Time) >= m.TimeNS })
		if index == len(program.points) || int64(program.points[index].Time) != m.TimeNS || program.points[index].IOMask&(1<<output) != 0 {
			return errors.New(ReasonIoMarkerInvalid)
		}
		program.points[index].IOMask |= 1 << output
		if m.Value {
			program.points[index].IOValues |= 1 << output
		}
	}
	if program.legacyTorch && program.torchQualified {
		output := -1
		for i, o := range state.IO.Outputs {
			if o.Torch {
				output = i
				break
			}
		}
		if output < 0 {
			return errors.New(ReasonIoNotConfigured)
		}
		// Legacy weld samples carry torch bits, not expiry-refresh markers.
		// Fill the immutable native plan while preserving authored transitions.
		on := false
		cursor := 0
		var transitions []ProcessMarker
		for _, m := range program.Markers {
			if m.Action == "torch" {
				transitions = append(transitions, m)
			}
		}
		for i := range program.points {
			p := &program.points[i]
			for cursor < len(transitions) && transitions[cursor].TimeNS <= int64(p.Time) {
				on = transitions[cursor].Value
				cursor++
			}
			if on {
				p.IOMask |= 1 << output
				p.IOValues |= 1 << output
			}
		}
	}

	// A qualified torch must have an OFF boundary and a refresh schedule
	// strictly inside its configured expiry. Sparse legacy samples fail closed.
	if program.torchQualified {
		c.mu.Lock()
		raw := append([]byte(nil), c.description.IO...)
		c.mu.Unlock()
		var spec struct {
			Outputs []struct {
				Name   string `json:"name"`
				Expiry uint64 `json:"expiry_ns"`
			} `json:"outputs"`
		}
		if json.Unmarshal(raw, &spec) != nil {
			return errors.New(ReasonIoMarkerInvalid)
		}
		for oi, o := range state.IO.Outputs {
			if !o.Torch {
				continue
			}
			var expiry uint64
			for _, s := range spec.Outputs {
				if s.Name == o.Name {
					expiry = s.Expiry
				}
			}
			on := false
			var last uint64
			for _, p := range program.points {
				if on && (expiry == 0 || uint64(p.Time)-last >= expiry) {
					return errors.New(ReasonIoMarkerInvalid)
				}
				if p.IOMask&(1<<oi) != 0 {
					on = p.IOValues&(1<<oi) != 0
					last = uint64(p.Time)
				}
			}
			if on {
				return errors.New(ReasonIoMarkerInvalid)
			}
		}
	}

	return nil
}

// IOArm preserves the application fence and lets the native cycle decide I/O
// readiness, preventing a delayed adapter command from arming after Stop.
func (c *Controller) IOArm(f Fence, arm bool) (uint64, error) {
	return c.operation(f, func(ctx context.Context) (uint64, error) {
		native, ok := c.core.(ioNative)
		if !ok {
			return 0, errors.New(ReasonCapabilityUnimplemented)
		}
		seq, err := native.IOArm(ctx, f.Generation, arm)
		return seq, publicIOError(err)
	})
}

func publicIOError(err error) error {
	var rejected *ipcclient.RejectedError
	if errors.As(err, &rejected) {
		reasons := map[uint32]string{ipcclient.ReasonNoGrant: ReasonNoGrant, ipcclient.ReasonWrongGeneration: ReasonWrongGeneration, ipcclient.ReasonInhibited: ReasonInhibited,
			ipcclient.ReasonIoNotConfigured: ReasonIoNotConfigured, ipcclient.ReasonIoNotArmed: ReasonIoNotArmed, ipcclient.ReasonIoFastInputUnsatisfied: ReasonIoFastInputUnsatisfied,
			ipcclient.ReasonIoReadbackDisagreement: ReasonIoReadbackDisagreement, ipcclient.ReasonIoTorchUnqualified: ReasonIoTorchUnqualified, ipcclient.ReasonIoMarkerLate: ReasonIoMarkerLate, ipcclient.ReasonIoExchangeLost: ReasonIoExchangeLost, ipcclient.ReasonIoWaitTimeout: ReasonIoWaitTimeout, ipcclient.ReasonIoWaitInputInvalid: ReasonIoWaitInputInvalid}
		if reason := reasons[rejected.Result.Reason]; reason != "" {
			return &publicRejection{reason, err}
		}
	}
	return err
}

// Compile scoped monitors off-cycle into immutable masks. The native loop checks
// the active mask on every motion cycle, without scanning a marker history.
func bindInputMonitors(program *Program, state ipcclient.IoState) error {
	events := make(map[int][]ProcessMarker)
	for _, m := range program.Markers {
		if m.Monitor == "" {
			continue
		}
		if (m.Monitor != "start" && m.Monitor != "end") || m.TimeoutMS != 0 || m.StableMS != 0 {
			return errors.New(ReasonIoMarkerInvalid)
		}
		index := sort.Search(len(program.points), func(i int) bool { return int64(program.points[i].Time) >= m.TimeNS })
		if index == len(program.points) || int64(program.points[index].Time) != m.TimeNS {
			return errors.New(ReasonIoMarkerInvalid)
		}
		events[index] = append(events[index], m)
	}
	var mask, values uint32
	for i := range program.points {
		seen := uint32(0)
		for _, m := range events[i] {
			input := -1
			for j, in := range state.Inputs {
				if in.Name == m.Action {
					input = j
					break
				}
			}
			if input < 0 || input >= 16 {
				return errors.New(ReasonIoNotConfigured)
			}
			bit := uint32(1) << input
			if seen&bit != 0 {
				return errors.New(ReasonIoMarkerInvalid)
			}
			seen |= bit
			if m.Monitor == "start" {
				if mask&bit != 0 {
					return errors.New(ReasonIoMarkerInvalid)
				}
				mask |= bit
				if m.Value {
					values |= bit
				} else {
					values &^= bit
				}
			} else {
				if mask&bit == 0 {
					return errors.New(ReasonIoMarkerInvalid)
				}
				mask &^= bit
				values &^= bit
			}
		}
		program.points[i].MonitorMask = mask
		program.points[i].MonitorValues = values
	}
	if mask != 0 {
		return errors.New(ReasonIoMarkerInvalid)
	}
	return nil
}
