//go:build linux

package denseexec

import (
	"fmt"
	"log"
	"math"
	"strings"
	"time"
)

// SetRobotGeometrySource supplies the robot description's geometry by model
// (the server reads its robot store); the controller asks once for its model.
func (r *RTCoreController) SetRobotGeometrySource(source func(model string) (RobotGeometry, error)) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.geometrySource, r.geometryKnown = source, false
}

// robotGeometryLocked is the selected model's geometry, read once. A model
// the store does not hold (only a cell has it) gives no wrist measure; the
// readout then names J5 without a singular angle. Hold r.mu.
func (r *RTCoreController) robotGeometryLocked() RobotGeometry {
	if !r.geometryKnown && r.geometrySource != nil && r.cfg.CartesianModel != "" {
		g, err := r.geometrySource(r.cfg.CartesianModel)
		if err != nil {
			log.Printf("cartesian path hold: no robot geometry for model %q: %v", r.cfg.CartesianModel, err)
		}
		r.geometry, r.geometryKnown = g, true
	}
	return r.geometry
}

var limitWords = map[string]string{
	"command": "your speed", "joint_velocity": "a joint speed limit", "curvature": "joint acceleration on the path's curve",
	"jerk": "joint jerk", "singularity": "singularity", "lookahead": "path look-ahead (still computing the line)",
	"stopping": "stopping", "none": "",
}

func signedAxis(axis, direction int) string {
	if direction < 0 {
		return "-" + axisNames[axis]
	}
	return "+" + axisNames[axis]
}

func fptr(v float64) *float64 { return &v }

// pathLiveLocked is the hold's readout from its last native observation and
// the samples it streamed. Hold r.mu.
func (r *RTCoreController) pathLiveLocked(h *jointJogHold, active bool) *PathLive {
	p := h.path
	if p == nil {
		return nil
	}
	o := p.observation
	scale := p.scale()
	p.geomMu.Lock()
	defer p.geomMu.Unlock()
	live := &PathLive{Active: active, Revision: h.intent.Revision, Axis: signedAxis(p.line.Axis, p.line.Direction), Frame: p.line.Frame,
		SpeedUnit: p.unit + "/s", RequestedSpeed: p.lastSpeed * scale, Speed: o.PathSpeed * scale, Governing: pathName(pathGoverningNames, o.Governing),
		DistanceUnit: p.unit, Escape: signedAxis(p.line.Axis, -p.line.Direction), LineEnd: p.endReason}
	i := int(math.Floor(o.PathS/p.spacing + 1e-9))
	have := i >= 0 && i < len(p.samples)
	if have {
		live.GateScale = fptr(p.scales[i])
		lowest := p.scales[i]
		for _, s := range p.scales[i:] {
			lowest = math.Min(lowest, s)
		}
		live.MinGateScaleAhead = fptr(lowest)
		live.Near = lowest < 1
		q5 := p.samples[i][4]
		live.J5Deg = fptr(degrees(q5))
		if angle, distance, ok := p.geometry.nearestWristSingular(q5); ok {
			live.WristSingularDeg, live.WristDistanceDeg = fptr(degrees(angle)), fptr(degrees(distance))
			if distance >= 0 {
				live.JointHint = "+J5"
			} else {
				live.JointHint = "-J5"
			}
		}
	}
	if p.ended && active {
		live.DistanceToEnd = fptr(math.Max(0, float64(o.PlayableEnd)*p.spacing-o.PathS) * scale)
	}
	kind := map[string]string{"command": "command", "velocity": "joint_velocity", "curvature": "curvature", "jerk": "jerk",
		"buffer_end": "lookahead", "path_end": "line_end", "stopping": "stopping"}[live.Governing]
	limit := limitWords[kind]
	if live.Governing == "cap" && have {
		// The segment's cap is the lower of its two knots' caps (PathJog):
		// the binding sample is the lower of this one and the next.
		k := i
		if i+1 < len(p.caps) && p.caps[i+1] < p.caps[i] {
			k = i + 1
		}
		if s := p.scales[k]; s > 0 && s < 1 && math.Abs(p.caps[k]-s*p.box) <= 1e-9*p.box {
			kind, limit = "singularity", "singularity"
		} else if k > 0 {
			kind, limit = "joint_reserve", "joint speed reserve"
			worst, joint := math.Inf(1), -1
			for j := range p.velocity {
				if slope := math.Abs(p.samples[k][j]-p.samples[k-1][j]) / p.spacing; slope > 0 && p.velocity[j] > 0 {
					if c := p.reserve * p.velocity[j] / slope; c < worst {
						worst, joint = c, j
					}
				}
			}
			if joint >= 0 {
				limit = fmt.Sprintf("joint speed reserve (J%d)", joint+1)
			}
		}
	}
	if kind == "line_end" && live.LineEnd != "" {
		limit = "end of line (" + strings.ReplaceAll(live.LineEnd, "_", " ") + ")"
	}
	if kind == "" {
		kind = "none"
	}
	live.LimitKind, live.Limit = kind, limit
	live.Text = pathLiveText(live)
	return live
}

// pathLiveText words the readout, e.g. "limited by: singularity (J5
// -88.3°, 1.7° from the wrist-singular -90.0°) — slowing, stops in ~3 mm;
// back out with +X, or joint-jog +J5".
func pathLiveText(l *PathLive) string {
	wrist := ""
	if l.J5Deg != nil {
		wrist = fmt.Sprintf("J5 %.1f°", *l.J5Deg)
		if l.WristDistanceDeg != nil {
			wrist += fmt.Sprintf(", %.1f° from the wrist-singular %.1f°", math.Abs(*l.WristDistanceDeg), *l.WristSingularDeg)
		}
	}
	escape := "back out with " + l.Escape
	if l.JointHint != "" && (l.Near || l.LimitKind == "singularity") {
		escape += ", or joint-jog " + l.JointHint
	}
	ahead := ""
	if l.DistanceToEnd != nil {
		ahead = fmt.Sprintf(", stops in ~%s %s", roundDistance(*l.DistanceToEnd), l.DistanceUnit)
	}
	switch {
	case !l.Active && l.Near:
		return fmt.Sprintf("stopped near a singularity (%s); %s", wrist, escape)
	case !l.Active:
		return ""
	case l.LimitKind == "singularity":
		return fmt.Sprintf("limited by: singularity (%s) — slowing%s; %s", wrist, ahead, escape)
	case l.Near:
		scale := 1.0
		if l.MinGateScaleAhead != nil {
			scale = *l.MinGateScaleAhead
		}
		return fmt.Sprintf("near a singularity ahead (%s, gate scale %.2f) — limited by: %s%s; %s", wrist, scale, l.Limit, ahead, escape)
	case l.LimitKind == "line_end" || l.DistanceToEnd != nil:
		return fmt.Sprintf("limited by: %s%s; %s", firstNonEmpty(l.Limit, "end of line"), ahead, escape)
	case l.Limit != "":
		return "limited by: " + l.Limit
	}
	return ""
}

func roundDistance(d float64) string {
	if d < 10 {
		return fmt.Sprintf("%.1f", d)
	}
	return fmt.Sprintf("%.0f", d)
}

// jogLiveLocked is the running path hold's readout, or the last one kept
// after it ended. Hold r.mu.
func (r *RTCoreController) jogLiveLocked() *PathLive {
	if h := r.jog; h != nil && h.path != nil {
		return r.pathLiveLocked(h, true)
	}
	if r.lastPathLive == nil {
		return nil
	}
	live := *r.lastPathLive
	live.AgeMS = time.Since(r.lastPathLiveAt).Milliseconds()
	return &live
}
