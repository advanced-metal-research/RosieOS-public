//go:build linux

package rtctl

import control "rosieos/rt-core/adapters/rosie/control"

// On Linux the compiled resource manifest is the adapter's own type and bound,
// so what rtctl writes is exactly what the adapter admits and serves.
type (
	resourceManifest = control.ResourceManifest
	driveResource    = control.DriveDescription
	robotResources   = control.RobotDescription
	resourceInfo     = control.ResourceInfo
)

const (
	maxResourceBytes    = control.MaxResourceBytes
	maxResourceSetBytes = control.MaxResourceSetBytes
	maxResourceEntries  = control.MaxResourceEntries
)
