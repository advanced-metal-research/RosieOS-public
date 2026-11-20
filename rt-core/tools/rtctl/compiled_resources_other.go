//go:build !linux

package rtctl

// The adapter (adapters/rosie/control) is Linux-only, so off Linux the
// compiler carries field-for-field copies of its generated resource types and
// admission bounds. They reach only resources.json, never an identity or
// digest; packages are compiled on Linux, where the adapter's own types are
// used (compiled_resources_linux.go). This lets analysis tools on the Mac
// (tools/tuningreport, the OLP's cartesianjog-harness) take their bounds from
// the compiler instead of re-reading the machine config.
type resourceManifest struct {
	ConfigurationSHA256 string          `json:"configuration_sha256"`
	MachineSHA256       string          `json:"machine_sha256"`
	Robot               *robotResources `json:"robot"`
	Drives              []driveResource `json:"drives"`
}

type resourceInfo struct {
	Path      string `json:"path"`
	SHA256    string `json:"sha256"`
	Bytes     uint64 `json:"bytes"`
	MediaType string `json:"media_type"`
}

type robotResources struct {
	ModelID                          string         `json:"model_id"`
	RobotDescriptionSHA256           string         `json:"robot_description_sha256"`
	MachinePlanningCalibrationSHA256 string         `json:"machine_planning_calibration_sha256"`
	Resources                        []resourceInfo `json:"resources"`
}

type driveIdentity struct {
	ExpectedVendorID    uint32 `json:"expected_vendor_id"`
	ExpectedProductCode uint32 `json:"expected_product_code"`
	ObservedCoERevision uint32 `json:"observed_coe_revision"`
}

type driveResource struct {
	Axis             string         `json:"axis"`
	ConfigName       string         `json:"config_name"`
	ConfigSHA256     string         `json:"config_sha256"`
	SlavePosition    uint16         `json:"slave_position"`
	VerifiedIdentity *driveIdentity `json:"verified_identity"`
}

// The adapter's unverified admission bounds (adapters/rosie/control/resources.go):
// 32 MiB per file, 128 MiB per set, 1024 references.
const (
	maxResourceBytes    = 32 << 20
	maxResourceSetBytes = 128 << 20
	maxResourceEntries  = 1024
)
