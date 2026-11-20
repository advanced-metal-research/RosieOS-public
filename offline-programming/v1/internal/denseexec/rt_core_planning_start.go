//go:build linux

package denseexec

import (
	"fmt"
	"math"

	"rosieos/offline-programming/v1/internal/densejointtraj"
	"rosieos/rt-core/sdk/control"
)

// Load is nonmotion staging, but must not rebind an older planning observation
// to a new Home/configuration or restarted controller. Native admission fences
// subsequent changes; Play still checks current positions against the first
// sample. Legacy offline previews without a measured capture remain supported.
func validatePlanningCapture(plan *densejointtraj.Plan, status control.Status, description control.Description) error {
	start := plan.Header.StartState
	if start == nil {
		return nil
	}
	fail := func() error {
		return rtRefused("planning_start_stale: replan from verified current feedback after a Home, configuration, or controller identity change")
	}
	if start.Schema != "rosie.measured-planning-start.v1" || status.Core.ConfigurationSHA256 == "" || status.Core.ConfigurationEpoch == 0 || status.Core.HomeEpoch == 0 || status.AdapterIncarnation == "" || status.DaemonIncarnation == "" ||
		start.CoordinateIdentity != fmt.Sprintf("%s:%d:%d", status.Core.ConfigurationSHA256, status.Core.ConfigurationEpoch, status.Core.HomeEpoch) ||
		start.ObservationIdentity != status.AdapterIncarnation+":"+status.DaemonIncarnation {
		return fail()
	}
	if len(start.JointNames) != len(description.Axes) || len(start.JointValues) != len(description.Axes) || len(start.JointNames) == 0 || len(plan.Segments) == 0 || len(plan.Segments[0].Samples) == 0 {
		return fail()
	}
	first := plan.Segments[0].Samples[0]
	for i, axis := range description.Axes {
		if i >= len(first.QRad) || start.JointNames[i] != axis.ID || math.IsNaN(start.JointValues[i]) || math.IsInf(start.JointValues[i], 0) || math.Abs(start.JointValues[i]-first.QRad[i]) > 1e-10 {
			return fail()
		}
	}
	return nil
}
