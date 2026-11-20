//go:build linux

package denseexec

import (
	"context"
	"fmt"
	"log"
	"math"
	"rosieos/offline-programming/v1/internal/robotmodel"
)

// SetCartesianResolver supplies computation only; it cannot acquire authority.
// Reconfiguration retires input so results from the previous service are stale.
func (r *RTCoreController) SetCartesianResolver(resolve CartesianResolver) {
	r.mu.Lock()
	var lane jointJogLane
	if r.jog != nil {
		lane = r.retireJointJogLocked("cartesian_resolver_changed")
	}
	previous := r.cartesianService
	r.cartesianService = nil
	r.cartesianResolve = resolve
	r.mu.Unlock()
	if previous != nil {
		previous.Close()
	}
	if lane != nil {
		if err := r.endJointJog(lane); err != nil {
			_ = r.fail(err)
		}
	}
}

// SetCartesianResolverFactory gives each connection its own model and process;
// closing the controller also closes computation, preventing orphan resolvers.
func (r *RTCoreController) SetCartesianResolverFactory(factory CartesianResolverFactory) {
	r.SetCartesianResolver(nil)
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.closed || factory == nil {
		return
	}
	service := factory(r.cfg.CartesianModel)
	if service != nil {
		r.cartesianService, r.cartesianResolve = service, service.Resolve
	}
	log.Printf("cartesian resolver: attached=%v model=%q", service != nil, r.cfg.CartesianModel)
}

// CartesianJog shares the joint hold slot, input age and retirement; the hold
// itself is a path hold (cartesian_path_hold.go). No Cartesian operation arms
// or acquires; a stopped revision cannot restart.
func (r *RTCoreController) CartesianJog(ctx context.Context, action string, in CartesianJogIntent) (JointJogState, error) {
	switch action {
	case "start":
		action = "begin"
	case "intent":
		action = "update"
	case "stop", "halt":
		action = "end"
	}
	joint := JointJogIntent{TargetID: in.TargetID, SessionID: in.SessionID, Revision: in.Revision, Axis: 0, Direction: 1, Fraction: in.Fraction, MaxFraction: in.MaxFraction, InputSequence: in.InputSequence, EndCause: in.EndCause, cartesian: &in}
	return r.JointJog(ctx, action, joint)
}

// cartesianAxesLocked requires named rotary arm axes in Describe. Missing axes
// and ambiguous mappings fail closed instead of inheriting positional defaults.
func (r *RTCoreController) cartesianAxesLocked() ([6]int, uint32, error) {
	var axes [6]int
	var mask uint32
	if !robotmodel.IsArm(r.cfg.CartesianModel) || r.cartesianResolve == nil {
		log.Printf("cartesian resolver: unavailable model=%q resolver_nil=%v", r.cfg.CartesianModel, r.cartesianResolve == nil)
		return axes, 0, rtRefused("cartesian_resolver_unavailable")
	}
	for joint := range axes {
		found := -1
		for i, a := range r.description.Axes {
			if a.ID != fmt.Sprintf("J%d", joint+1) {
				continue
			}
			if found != -1 || a.Index != uint32(i) || a.PositionUnit != "rad" || i >= len(r.observation.Axes) || r.cfg.AxisMask&(1<<i) == 0 {
				return axes, 0, rtRefused("cartesian_axes_invalid")
			}
			found = i
		}
		if found < 0 {
			return axes, 0, rtRefused("cartesian_axes_invalid")
		}
		axes[joint] = found
		mask |= 1 << found
	}
	return axes, mask, nil
}

// The operator bounds (cartesianJogMaxLinearMMs, cartesianJogMaxAngularDegS)
// in SI: m/s and rad/s. A larger twist is refused before the resolver.
func validCartesian(in *CartesianJogIntent) bool {
	if in.Frame != "base" && in.Frame != "tool" {
		return false
	}
	nonzero := false
	for i, v := range in.Twist {
		limit := cartesianJogMaxLinearMMs / 1000
		if i >= 3 {
			limit = cartesianJogMaxAngularDegS * math.Pi / 180
		}
		if !finiteJoint(v) || math.Abs(v) > limit {
			return false
		}
		nonzero = nonzero || v != 0
	}
	return nonzero
}

// Every arm joint must have a native publication within the negotiated status
// age. Fresh aggregate status cannot renew a frozen axis measurement.
func (r *RTCoreController) cartesianPoseFreshLocked(axis int) bool {
	sample, current := r.observation.Axes[axis].TimeNS, r.observation.Core.TimeNS
	return sample != 0 && current != 0 && (current <= sample || current-sample <= uint64(r.timingLocked().StatusAge))
}
