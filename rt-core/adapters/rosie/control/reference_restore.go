//go:build linux

package control

import (
	"context"
	"errors"
	"math"
	"math/bits"
	"rosieos/rt-core/adapters/rosie/control/anchors"
	"rosieos/rt-core/ipcclient"
	"strings"
	"time"
)

const verifiedDeltaMethod = "independent_absolute_delta_v1"

func (n *restartNative) RestoreReferenceVerifiedDelta(ctx context.Context, mask uint32) (uint64, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	return n.client.RestoreReferenceVerifiedDelta(ctx, mask)
}

func verifiedDeltaReceipt(s, before ipcclient.Status, axis uint32, sequence uint64, saved anchors.Record) (*VerifiedDeltaRestoreReceipt, error) {
	now := ipcclient.HostMonotonicNS()
	if s.TimeNS == 0 || s.TimeNS > now || now-s.TimeNS > 250000000 {
		return nil, anchors.ErrSource
	}
	if int(axis) >= len(s.Axes) || s.DaemonIncarnation != before.DaemonIncarnation || s.ConfigurationSHA256 != before.ConfigurationSHA256 || s.ConfigurationEpoch != before.ConfigurationEpoch || s.HomeEpoch != before.HomeEpoch+1 || s.CommissioningPhase != 0 || s.Armed != 0 || s.AxisEnableMask != 0 || s.SafetyFaultMask != 0 || s.ExecutionFaultReasons != 0 || s.HomeValidMask&(1<<axis) == 0 {
		return nil, anchors.ErrSource
	}
	a := s.Axes[axis]
	if a.CoordinateIdentity != saved.CoordinateIdentity || a.CoordinateIdentitySHA256 != saved.CoordinateIdentitySHA256 || a.VendorID != saved.Vendor || a.ProductCode != saved.Product || a.Revision != saved.Revision || (saved.SerialValid != 0 && a.Serial != saved.Serial) || a.SerialValid != saved.SerialValid || a.RevisionValid == 0 || a.HomeOffset != saved.HomeOffset || a.AnchorToleranceCounts != saved.ToleranceCounts {
		return nil, anchors.ErrIdentity
	}
	if a.PDOFresh == 0 || a.CoordinateValid == 0 || a.CoordinateSourceValid == 0 || s.ConfigVerifiedMask&(1<<axis) == 0 {
		return nil, anchors.ErrSource
	}
	validBinding := func(job, home, config, axisEpoch, completed, pdo uint64) bool {
		return sequence != 0 && job == sequence && home == s.HomeEpoch && config == s.ConfigurationEpoch && axisEpoch != 0 && completed >= before.TimeNS && completed <= s.TimeNS && pdo != 0
	}
	// Only an HM35 v3 anchor is restorable; v4 single-turn frames are refused.
	v := a.AnchorDeltaRestore
	if v == nil || !v.Valid || v.Method != verifiedDeltaMethod || v.AnchorVersion != 3 || !validBinding(v.BoundRestoreCommandSequence, v.BoundHomeEpoch, v.BoundConfigurationEpoch, v.BoundAxisCoordinateEpoch, v.BoundCompletedNS, v.BoundPDOReceipt) {
		return nil, anchors.ErrSource
	}
	sign := int64(0)
	for _, line := range strings.Split(saved.CoordinateIdentity, "\n") {
		if strings.HasPrefix(line, "home.policy=") {
			for _, part := range strings.Split(line, ";") {
				if part == "sign|1" {
					sign = 1
				}
				if part == "sign|-1" {
					sign = -1
				}
			}
		}
	}
	if sign == 0 || sign*int64(v.SavedDriveHomeOffsetRaw) != int64(saved.HomeOffset) || v.SavedCommandCounts != saved.AnchorCounts || v.SavedEncoderCounts != saved.TurnsCounts || v.SavedLogicalOffsetCounts != saved.HomeOffset || v.SavedHomeEpoch != saved.HomeEpoch {
		return nil, anchors.ErrSource
	}
	commandDelta, encoderDelta := v.BoundDeltaCommandCounts, v.BoundDeltaEncoderCounts
	copy := *v
	receipt := &VerifiedDeltaRestoreReceipt{AnchorDeltaRestore: &copy}

	// Bound proof comes from the native acquisition, not a later moving PDO.
	// There is no delta cap; native checked the restored position against the
	// joint's hard limits. Native aligns the command delta to the encoder's
	// full-turn representative, so the two agree within the anchor tolerance.
	abs := func(x int64) uint64 {
		if x < 0 {
			return uint64(-(x + 1)) + 1
		}
		return uint64(x)
	}
	if abs(commandDelta) > math.MaxUint32 || abs(encoderDelta) > math.MaxUint32 ||
		abs(commandDelta-encoderDelta) > saved.ToleranceCounts {
		return nil, anchors.ErrDisagrees
	}
	return receipt, nil
}

// RestoreReferenceVerifiedDelta never saves an anchor or redefines logical zero.
// Admission and response proof are explicit, independent of RestoreAnchor.
func (c *Controller) RestoreReferenceVerifiedDelta(f Fence, mask uint32) (uint64, *VerifiedDeltaRestoreReceipt, error) {
	var receipt *VerifiedDeltaRestoreReceipt
	sequence, err := c.operation(f, func(ctx context.Context) (uint64, error) {
		if bits.OnesCount32(mask) != 1 {
			return 0, errors.New("invalid_axis_mask")
		}
		axis := uint32(bits.TrailingZeros32(mask))
		if int(axis) >= len(c.description.Axes) {
			return 0, errors.New("invalid_axis_mask")
		}
		policy := c.description.Axes[axis].ReferenceVerifiedDeltaRestore
		if policy == nil || policy.Method != verifiedDeltaMethod {
			return 0, errors.New(ReasonCapabilityUnimplemented)
		}
		restorer, ok := c.core.(interface {
			RestoreReferenceVerifiedDelta(context.Context, uint32) (uint64, error)
		})
		if !ok {
			return 0, errors.New(ReasonCapabilityUnimplemented)
		}
		if c.anchorStore.Directory == "" {
			return 0, anchors.ErrMissing
		}
		saved, err := c.anchorStore.Load(axis)
		if err != nil {
			return 0, err
		}
		before, err := c.core.Status()
		if err != nil {
			return 0, err
		}
		now := ipcclient.HostMonotonicNS()
		if before.TimeNS == 0 || before.TimeNS > now || now-before.TimeNS > 250000000 || before.Armed != 0 || before.AxisEnableMask != 0 || before.SafetyFaultMask != 0 || before.ExecutionFaultReasons != 0 || before.CommissioningPhase != 0 {
			return 0, errors.New(ReasonModeConflict)
		}
		current, err := restoreAnchorRecord(before, axis, c.binding)
		if err != nil {
			return 0, err
		}
		if err = saved.QualifyVerifiedDelta(current, true); err != nil {
			return 0, err
		}
		restoreCtx, cancel := context.WithTimeout(ctx, 8*time.Second)
		defer cancel()
		seq, err := restorer.RestoreReferenceVerifiedDelta(restoreCtx, mask)
		if err == nil {
			_, err = c.core.WaitResult(restoreCtx, seq)
		}
		if err == nil {
			ticker := time.NewTicker(10 * time.Millisecond)
			defer ticker.Stop()
			for {
				if err = restoreCtx.Err(); err != nil {
					break
				}
				c.mu.Lock()
				err = c.authorize(f)
				c.mu.Unlock()
				if err != nil {
					break
				}
				state, e := c.core.Status()
				if e != nil {
					err = e
					break
				}
				now = ipcclient.HostMonotonicNS()
				if state.TimeNS == 0 || state.TimeNS > now || now-state.TimeNS > 250000000 {
					err = anchors.ErrSource
					break
				}
				if state.DaemonIncarnation != before.DaemonIncarnation || state.ConfigurationEpoch != before.ConfigurationEpoch || state.ConfigurationSHA256 != before.ConfigurationSHA256 || state.HomeEpoch > before.HomeEpoch+1 || state.Armed != 0 || state.AxisEnableMask != 0 || state.SafetyFaultMask != 0 || state.ExecutionFaultReasons != 0 {
					err = anchors.ErrSource
					break
				}
				if state.HomeEpoch == before.HomeEpoch+1 && state.CommissioningPhase == 0 {
					value, e := verifiedDeltaReceipt(state, before, axis, seq, saved)
					if e == nil {
						unchanged, loadErr := c.anchorStore.Load(axis)
						if loadErr != nil {
							err = loadErr
							break
						}
						if unchanged != saved {
							err = anchors.ErrIdentity
							break
						}
						c.mu.Lock()
						err = c.authorize(f)
						c.mu.Unlock()
						if err == nil {
							err = restoreCtx.Err()
						}
						if err == nil {
							receipt = value
						}
						break
					}
					if !errors.Is(e, anchors.ErrSource) {
						err = e
						break
					}
				}
				select {
				case <-restoreCtx.Done():
					err = restoreCtx.Err()
				case <-ticker.C:
				}
				if err != nil {
					break
				}
			}
		}
		if err != nil {
			_, _ = c.core.Stop(context.Background())
		}
		return seq, err
	})
	return sequence, receipt, err
}
