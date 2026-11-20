//go:build linux

// Package ipcclient connects a Rosie application adapter to the local realtime
// daemon. Network authentication, pair identity and controller authority remain
// the adapter's responsibility. Successful writes are submission, not evidence
// that the daemon accepted a command or that a drive performed it.
package ipcclient

import (
	"context"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"net"
	"os"
	"path/filepath"
	"sync"
	"sync/atomic"
	"time"
	"unsafe"

	"golang.org/x/sys/unix"
)

// Measured native schema axis capacity.
const maxAxes = MaxAxes

var wire = binary.LittleEndian
var ErrClosed = errors.New("RTCore connection closed")
var ErrRevoked = errors.New("RTCore upload interrupted by disarm")

// ErrDiagnosticsNotPublished permits bootstrap retry without accepting any
// predecessor state. Established clients never return this retryable error.
var ErrDiagnosticsNotPublished = errors.New("RTCore diagnostics not yet published")
var errDiagnosticIncarnation = errors.New("RTCore diagnostic incarnation mismatch")

type Options struct {
	SocketPath string
	// Bind to the independently compiled machine/profile digest, not a value
	// learned from the daemon being connected to.
	ConfigurationSHA256 string
	MachineSHA256       string
	Backend             string // "ethercat" or "simulation"; no live-to-simulation fallback.
	// SimulationTest selects the explicit legacy test role. Normal adapters must
	// mirror application authority through the native grant region before motion.
	SimulationTest bool
}

// Metrics is the daemon's diagnostic JSON extension. Explicit fields prevent
// tools from silently decoding timing counters into anonymous, divergent types.
type Metrics struct {
	TimeNS             uint64 `json:"time_ns"`
	ConfigVerifiedMask uint32 `json:"config_verified_mask,omitempty"`
	MaxJitterNS        uint64 `json:"rt_max_abs_jitter_ns"`
	MaxWorkNS          uint64 `json:"rt_max_cycle_work_ns"`
	Overruns           uint64 `json:"rt_overrun_count"`
	DiagnosticsDropped uint64 `json:"diagnostics_dropped"`
}

type Status struct {
	Bus                     json.RawMessage `json:"bus"`
	IO                      IoState         `json:"io"`
	ConfigurationEpoch      uint64          `json:"configuration_epoch"`
	BufferHealth            BufferHealth    `json:"buffer_health"`
	PlanCursor              PlanCursor      `json:"plan_cursor"`
	DaemonIncarnation       string          `json:"daemon_incarnation"`
	RTCPU                   uint32          `json:"rt_cpu"`
	HousekeepingCPUs        string          `json:"housekeeping_cpus"`
	AffinityApplied         map[string]bool `json:"affinity_applied"`
	Recovery                *RecoveryStatus `json:"recovery"`
	TimeNS                  uint64          `json:"time_ns"`
	Backend                 string          `json:"backend"`
	ConfigurationSHA256     string          `json:"configuration_sha256"`
	ControlGeneration       uint64          `json:"control_generation"`
	LeaseValid              uint32          `json:"lease_valid"`
	ConfigVerifiedMask      uint32          `json:"config_verified_mask"`
	HomeValidMask           uint32          `json:"home_valid_mask"`
	HomeEpoch               uint64          `json:"home_epoch"`
	CommissioningPhase      uint32          `json:"commissioning_phase"` // 0 idle, 1 active, 2 draining.
	SafetyFaultMask         uint32          `json:"safety_fault_mask"`
	ExecutionFaultReasons   uint32          `json:"execution_fault_reasons"`
	LastBusFailureOperation uint32          `json:"last_bus_failure_operation"`
	LastBusFailureCode      int64           `json:"last_bus_failure_code"`
	Armed                   uint32          `json:"armed"`
	AxisEnableMask          uint32          `json:"axis_enable_mask"`
	NativeHomeActiveMask    uint32          `json:"native_home_active_axis_mask"`
	Axes                    []AxisStatus    `json:"axes"`
}

// FaultRecovery is the native table's per-bit observation. Axis masks identify
// affected evidence without inferring recovery policy from a generic fault bit.
type FaultRecovery struct {
	Bit            uint32 `json:"bit"`
	Name           string `json:"name"`
	AxisMask       uint32 `json:"axis_mask"`
	Recovery       string `json:"recovery"`
	Outcome        string `json:"outcome"`
	RehomeAxisMask uint32 `json:"rehome_axis_mask"`
	// Current full-cell Reset admission preserving already-qualified Home.
	// Missing native fields decode as false, including older runtimes and
	// historical outcomes. Public JSON emits false explicitly; never auto-arm.
	ResetPreservesHome bool `json:"reset_preserves_home"`
}

// RecoveryStatus binds the last reset outcome to its native command sequence,
// preventing an older metrics publication from being reported as this reset.
type RecoveryStatus struct {
	HomeValidMask uint32          `json:"home_valid_mask"`
	Faults        []FaultRecovery `json:"faults"`
	ResetSequence uint64          `json:"reset_sequence"`
	Reason        string          `json:"reason"`
	Outcomes      []FaultRecovery `json:"outcomes"`
	// Message is native's operator sentence for the reset's drive alarms that
	// stay faulted ("J5 drive alarm Er41.1 persists after reset: ..."). It is
	// read from native status only and never marshalled: the adapter carries
	// it in its fault_persists refusal, so the public schema is unchanged.
	Message string `json:"-"`
}

// UnmarshalJSON reads the native-only message beside the public fields.
func (r *RecoveryStatus) UnmarshalJSON(data []byte) error {
	type public RecoveryStatus
	var wire struct {
		public
		Message string `json:"message"`
	}
	if err := json.Unmarshal(data, &wire); err != nil {
		return err
	}
	*r = RecoveryStatus(wire.public)
	r.Message = wire.Message
	return nil
}

// CollisionWatchdogStatus retains native consecutive counts and latched trip
// evidence; a low-rate consumer must never infer a trip from its own samples.
type CollisionWatchdogStatus struct {
	LastTripPeakTorqueRaw            uint32 `json:"last_trip_peak_torque_raw"`
	LastTripPeakFollowingErrorCounts uint64 `json:"last_trip_peak_following_error_counts"`
	Armed                            bool   `json:"armed"`
	TorqueCycles                     uint32 `json:"torque_cycles"`
	FollowingErrorCycles             uint32 `json:"following_error_cycles"`
	PeakTorqueRaw                    uint32 `json:"peak_torque_raw"`
	PeakFollowingErrorCounts         uint64 `json:"peak_following_error_counts"`
	LastTripQuantity                 string `json:"last_trip_quantity"`
	TripSustainedCycles              uint32 `json:"trip_sustained_cycles"`
	TripCount                        uint64 `json:"trip_count"`
	Samples                          uint64 `json:"samples"`
}

// IndependentAnchorSource is an acquired raw encoder observation, not a bound
// command coordinate or permission to move. Native rechecks it during Restore.
type IndependentAnchorSource struct {
	Valid         bool   `json:"valid"`
	EncoderCounts int64  `json:"encoder_counts"`
	CompletedNS   uint64 `json:"completed_ns"`
	MaximumAgeNS  uint64 `json:"maximum_age_ns"`
}

// AnchorDeltaRestoreStatus attests explicit restoration of an unchanged HM35 v3 anchor.
type AnchorDeltaRestoreStatus struct {
	Method                      string `json:"method"`
	AnchorVersion               uint32 `json:"anchor_version"`
	Valid                       bool   `json:"valid"`
	SavedCommandCounts          int32  `json:"saved_command_counts"`
	SavedEncoderCounts          int32  `json:"saved_encoder_counts"`
	SavedLogicalOffsetCounts    int32  `json:"saved_logical_offset_counts"`
	SavedDriveHomeOffsetRaw     int32  `json:"saved_drive_home_offset_raw"`
	SavedHomeEpoch              uint64 `json:"saved_home_epoch"`
	BoundRestoreCommandSequence uint64 `json:"bound_restore_command_sequence"`
	BoundConfigurationEpoch     uint64 `json:"bound_configuration_epoch"`
	BoundHomeEpoch              uint64 `json:"bound_home_epoch"`
	BoundAxisCoordinateEpoch    uint64 `json:"bound_axis_coordinate_epoch"`
	BoundDriveFrameEpoch        uint64 `json:"bound_drive_frame_epoch"`
	BoundCompletedNS            uint64 `json:"bound_completed_ns"`
	BoundPDOReceipt             uint64 `json:"bound_pdo_receipt"`
	BoundDeltaCommandCounts     int64  `json:"bound_delta_command_counts"`
	BoundDeltaEncoderCounts     int64  `json:"bound_delta_encoder_counts"`
}

// BrakeHoldDiagnostic is the latest native brake-hold WARNING for one axis. The
// drive manages its brake; a warning implies no fault, reset, restart, Home loss
// or Arm refusal. Counts are raw feedback; peak/settled are relative to anchor.
// Kind slip marks repeated same-direction warnings in one braked period. The
// energized fields record movement while the drive still held torque after the
// inhibit; it is never judged, and closing is judged from the torque-off anchor.
type BrakeHoldDiagnostic struct {
	ObservedNS                      uint64 `json:"observed_ns"`
	Severity                        string `json:"severity"`
	WarningSequence                 uint64 `json:"warning_sequence"`
	Kind                            string `json:"kind"`
	Transition                      string `json:"transition"`
	Message                         string `json:"message"`
	SameDirectionCount              uint32 `json:"same_direction_count"`
	SameDirectionDisplacementCounts int64  `json:"same_direction_displacement_counts"`
	Cause                           string `json:"cause"`
	Phase                           string `json:"phase"`
	AnchorCounts                    int32  `json:"anchor_counts"`
	FeedbackCounts                  int32  `json:"feedback_counts"`
	DisplacementCounts              int64  `json:"displacement_counts"`
	VelocityCountsPerS              int32  `json:"velocity_counts_per_s"`
	VelocityValid                   bool   `json:"velocity_valid"`
	PeakDisplacementCounts          int64  `json:"peak_displacement_counts"`
	PeakVelocityCountsPerS          int32  `json:"peak_velocity_counts_per_s"`
	Settled                         bool   `json:"settled"`
	SettledDisplacementCounts       int64  `json:"settled_displacement_counts"`
	TorqueOffNS                     uint64 `json:"torque_off_ns"`
	TorqueOffAnchorCounts           int32  `json:"torque_off_anchor_counts"`
	EnergizedPeakDisplacementCounts int64  `json:"energized_peak_displacement_counts"`
	EnergizedPeakVelocityCountsPerS int32  `json:"energized_peak_velocity_counts_per_s"`
	TransitionCapsValid             bool   `json:"transition_caps_valid"`
	EffectiveVelocityCapCountsPerS  uint32 `json:"effective_velocity_cap_counts_per_s"`
	EffectiveDisplacementCapCounts  uint32 `json:"effective_displacement_cap_counts"`
	ToleranceCounts                 uint32 `json:"tolerance_counts"`
	HoldDelayNS                     uint64 `json:"hold_delay_ns"`
	ReleaseDelayNS                  uint64 `json:"release_delay_ns"`
}

// DriveSupervisorDiagnostic is the latest native drive-state supervisor event
// for one axis: an ended Arm attempt (enable/release timeout, drop out of
// Operation Enabled, cancelled Arm stage) or a disarm-motion warning. It is
// identical on every axis, independent of brake configuration, never a latched
// fault, and Arm can be retried at once.
type DriveSupervisorDiagnostic struct {
	ObservedNS         uint64 `json:"observed_ns"`
	Severity           string `json:"severity"`
	EventSequence      uint64 `json:"event_sequence"`
	Kind               string `json:"kind"`
	Cause              string `json:"cause"`
	Message            string `json:"message"`
	Statusword         uint32 `json:"statusword"`
	ArmEnded           bool   `json:"arm_ended"`
	Current            bool   `json:"current"`
	ElapsedNS          uint64 `json:"elapsed_ns"`
	BoundNS            uint64 `json:"bound_ns"`
	DisplacementCounts int64  `json:"displacement_counts"`
	BoundCounts        uint32 `json:"bound_counts"`
}

// AxisLoad is the last second of one axis's torque and following error,
// summarised by the daemon's metrics thread from the telemetry ring
// (docs/decisions/0001). Diagnostics only.
type AxisLoad struct {
	WindowMS                 uint64  `json:"window_ms"`
	TorqueSamples            uint32  `json:"torque_samples"`
	TorqueRMSRaw             float64 `json:"torque_rms_raw"`
	TorquePeakRaw            uint32  `json:"torque_peak_raw"`
	FollowingErrorSamples    uint32  `json:"following_error_samples"`
	FollowingErrorPeakCounts uint64  `json:"following_error_peak_counts"`
}

// AxisMonitor is one drive-profile monitor object's latest reading, for
// example the drive's heatsink temperature; Value and AgeMS are absent until
// the first successful read.
type AxisMonitor struct {
	Key   string   `json:"key"`
	Unit  string   `json:"unit"`
	Value *float64 `json:"value,omitempty"`
	AgeMS *uint64  `json:"age_ms,omitempty"`
}

type AxisStatus struct {
	AnchorDeltaRestore             *AnchorDeltaRestoreStatus `json:"anchor_delta_restore,omitempty"`
	DriveAlarm                     string                    `json:"drive_alarm"`
	DriveAlarmText                 string                    `json:"drive_alarm_text"`
	DriveAlarmAuxCode              uint32                    `json:"drive_alarm_aux_code"`
	DriveAlarmAuxValid             bool                      `json:"drive_alarm_aux_valid"`
	TorqueRaw                      int32                     `json:"torque_raw"`
	CollisionWatchdog              CollisionWatchdogStatus   `json:"collision_watchdog"`
	Load                           AxisLoad                  `json:"load"`
	Monitor                        []AxisMonitor             `json:"monitor"`
	BrakeHoldDiagnostic            BrakeHoldDiagnostic       `json:"brake_hold_diagnostic"`
	DriveSupervisor                DriveSupervisorDiagnostic `json:"drive_supervisor"`
	DIBits                         uint32                    `json:"di_bits"`
	DIValid                        bool                      `json:"di_valid"`
	ExternalEnableActive           bool                      `json:"external_enable_active"`
	ExternalEnableValid            bool                      `json:"external_enable_valid"`
	VelocityActualCountsPerS       int32                     `json:"velocity_actual_counts_per_s"`
	FollowingErrorCounts           int32                     `json:"following_error_counts"`
	VelocityActualValid            bool                      `json:"velocity_actual_valid"`
	FollowingErrorValid            bool                      `json:"following_error_valid"`
	MaxAbsVelocityActualCountsPerS uint32                    `json:"max_abs_velocity_actual_counts_per_s"`
	MaxAbsFollowingErrorCounts     uint32                    `json:"max_abs_following_error_counts"`
	TimeNS                         uint64                    `json:"time_ns"`
	LogicalTarget                  float64                   `json:"logical_target"`
	LogicalVelocity                *float64                  `json:"logical_velocity,omitempty"`
	Readiness                      string                    `json:"readiness"`
	BrakeState                     uint32                    `json:"brake_state"`
	HomeValid                      bool                      `json:"home_valid"`
	LogicalPosition                float64                   `json:"logical_position"`
	LogicalValid                   bool                      `json:"logical_valid"`
	CoordinateCounts               int64                     `json:"coordinate_counts"`
	AbsoluteSourceCounts           int32                     `json:"absolute_source_counts"`
	IndependentAnchorSource        *IndependentAnchorSource  `json:"independent_anchor_source,omitempty"`
	CoordinateValid                uint32                    `json:"coordinate_valid"`
	CoordinateSourceValid          uint32                    `json:"coordinate_source_valid"`
	CalibrationValid               *bool                     `json:"calibration_valid,omitempty"`
	PositionState                  string                    `json:"position_state,omitempty"`
	PositionReason                 string                    `json:"position_reason,omitempty"`
	AuditStatus                    string                    `json:"audit_status,omitempty"`
	AuditReason                    string                    `json:"audit_reason,omitempty"`
	AuditLastVerifiedNS            uint64                    `json:"audit_last_verified_ns,omitempty"`
	PDOFresh                       uint32                    `json:"pdo_fresh"`
	HomeOffset                     int32                     `json:"native_home_position_offset"`
	Revision                       uint32                    `json:"observed_revision_no"`
	RevisionValid                  uint32                    `json:"revision_readback_valid"`
	Serial                         uint32                    `json:"serial_no"`
	SerialValid                    uint32                    `json:"serial_readback_valid"`
	AnchorToleranceCounts          uint64                    `json:"anchor_tolerance_counts"`
	CoordinateIdentitySHA256       string                    `json:"coordinate_identity_sha256"`
	CoordinateReason               uint32                    `json:"coordinate_reason"`
	CoordinateIdentity             string                    `json:"coordinate_identity"`
	AnchorIdentityChangedField     string                    `json:"anchor_identity_changed_field"`
	AnchorReason                   string                    `json:"anchor_reason"`
	RestoredAnchorHomeEpoch        uint64                    `json:"restored_anchor_home_epoch"`
	PositionCounts                 int32                     `json:"pos_counts"`
	Statusword                     uint16                    `json:"statusword"`
	ErrorCode                      uint16                    `json:"error_code"`
	VendorID                       uint32                    `json:"vendor_id"`
	ProductCode                    uint32                    `json:"product_code"`
	SlavePosition                  uint16                    `json:"slave_position"`
}

// LogicalAxisStatus carries one daemon publication unchanged. Optional velocity
// is absent unless native publishes it; the adapter never estimates motion.
type LogicalAxisStatus struct {
	AnchorDeltaRestore             *AnchorDeltaRestoreStatus `json:"anchor_delta_restore,omitempty"`
	DriveAlarm                     string                    `json:"drive_alarm"`
	DriveAlarmText                 string                    `json:"drive_alarm_text"`
	DriveAlarmAuxCode              uint32                    `json:"drive_alarm_aux_code"`
	DriveAlarmAuxValid             bool                      `json:"drive_alarm_aux_valid"`
	TorqueRaw                      int32                     `json:"torque_raw"`
	CollisionWatchdog              CollisionWatchdogStatus   `json:"collision_watchdog"`
	BrakeHoldDiagnostic            BrakeHoldDiagnostic       `json:"brake_hold_diagnostic"`
	DriveSupervisor                DriveSupervisorDiagnostic `json:"drive_supervisor"`
	DIBits                         uint32                    `json:"di_bits"`
	DIValid                        bool                      `json:"di_valid"`
	ExternalEnableActive           bool                      `json:"external_enable_active"`
	ExternalEnableValid            bool                      `json:"external_enable_valid"`
	VelocityActualCountsPerS       int32                     `json:"velocity_actual_counts_per_s"`
	FollowingErrorCounts           int32                     `json:"following_error_counts"`
	VelocityActualValid            bool                      `json:"velocity_actual_valid"`
	FollowingErrorValid            bool                      `json:"following_error_valid"`
	MaxAbsVelocityActualCountsPerS uint32                    `json:"max_abs_velocity_actual_counts_per_s"`
	MaxAbsFollowingErrorCounts     uint32                    `json:"max_abs_following_error_counts"`
	TimeNS                         uint64                    `json:"time_ns"`
	LogicalPosition                float64                   `json:"logical_position"`
	LogicalValid                   bool                      `json:"logical_valid"`
	LogicalTarget                  float64                   `json:"logical_target"`
	LogicalVelocity                *float64                  `json:"logical_velocity,omitempty"`
	Readiness                      string                    `json:"readiness"`
	PositionCounts                 int32                     `json:"position_counts"`
	Statusword                     uint16                    `json:"statusword"`
	BrakeState                     uint32                    `json:"brake_state"`
	HomeValid                      bool                      `json:"home_valid"`
	CalibrationValid               *bool                     `json:"calibration_valid,omitempty"`
	PositionState                  string                    `json:"position_state,omitempty"`
	PositionReason                 string                    `json:"position_reason,omitempty"`
	AuditStatus                    string                    `json:"audit_status,omitempty"`
	AuditReason                    string                    `json:"audit_reason,omitempty"`
	AuditLastVerifiedNS            uint64                    `json:"audit_last_verified_ns,omitempty"`
}

// PlanCursor retains native sample time, preventing adapter request time from
// being mistaken for the execution clock.
type PlanCursor struct {
	TimeNS        uint64 `json:"time_ns"`
	ActiveHandle  uint64 `json:"active_handle"`
	SampleIndex   uint32 `json:"sample_index"`
	NativeClockNS uint64 `json:"native_clock_ns"`
}

// BufferHealth exposes bounded native store occupancy without reading mutable
// plan contents or inferring native residency from adapter handles.
type BufferHealth struct {
	TimeNS              uint64            `json:"time_ns"`
	SlotsFree           uint32            `json:"slots_free"`
	StagingInProgress   bool              `json:"staging_in_progress"`
	RingDrops           uint64            `json:"ring_drops"`
	NativeSlotOccupancy map[string]uint32 `json:"native_slot_occupancy"`
}

// MotionState is a daemon observation from the shared-memory status ring.
// Done describes execution completion; Statusword/readback still determines
// the physical drive state. It must not be inferred from a successful write.
type MotionState struct {
	Mode            uint32 `json:"mode"`
	State           uint32 `json:"state"`
	TrajectoryID    uint64 `json:"trajectory_id"`
	PointIndex      uint32 `json:"point_index"`
	QueueDepth      uint32 `json:"queue_depth"`
	LastEvent       uint32 `json:"last_event"`
	Done            bool   `json:"done"`
	CommandSequence uint64 `json:"command_sequence"`
	TimeNS          uint64 `json:"time_ns"`
}

type Point struct {
	MonitorMask   uint32        `json:"monitor_mask,omitempty"`
	MonitorValues uint32        `json:"monitor_values,omitempty"`
	WaitTimeoutNS uint64        `json:"wait_timeout_ns,omitempty"`
	WaitStableNS  uint64        `json:"wait_stable_ns,omitempty"`
	WaitInput     uint32        `json:"wait_input,omitempty"`
	WaitValue     uint32        `json:"wait_value,omitempty"`
	IOMask        uint32        `json:"io_mask,omitempty"`
	IOValues      uint32        `json:"io_values,omitempty"`
	Time          time.Duration `json:"time_ns"`
	Position      []float64     `json:"position"`           // rad for rotary axes; metres for linear axes.
	Velocity      []float64     `json:"velocity,omitempty"` // nil omits feedforward; otherwise rad/s or m/s.
}

type ring struct {
	data             []byte
	offset, capacity uint32
}

func (r ring) counter(offset uint32) *uint32 {
	return (*uint32)(unsafe.Pointer(&r.data[r.offset+offset]))
}

func (r ring) indices() (uint32, uint32) {
	return atomic.LoadUint32(r.counter(RingHeaderV1WriteIdxOffset)), atomic.LoadUint32(r.counter(RingHeaderV1ReadIdxOffset))
}

func mapRing(fd int, kind, axes uint32) (ring, error) {
	var stat unix.Stat_t
	if err := unix.Fstat(fd, &stat); err != nil {
		return ring{}, err
	}
	// Measured geometry: 1024 status slots of 4096 bytes plus shared headers fit 5 MiB.
	if stat.Size < 160 || stat.Size > 5*1024*1024 {
		return ring{}, errors.New("invalid RTCore shared-memory size")
	}
	mapping, err := unix.Mmap(fd, 0, int(stat.Size), unix.PROT_READ|unix.PROT_WRITE, unix.MAP_SHARED)
	if err != nil {
		return ring{}, err
	}
	header, decodeErr := DecodeShmHeaderV1(mapping[:ShmHeaderV1Bytes])
	r := ring{data: mapping, offset: header.RingOffset, capacity: header.RingCapacity}
	expectedCapacity := uint32(GRADIENT_CMD_RING_CAPACITY)
	if kind == kShmKindStatus {
		expectedCapacity = GRADIENT_STATUS_RING_CAPACITY
	}
	valid := decodeErr == nil && header.Magic == kMagicGshm && header.VerMinor == VersionMinor &&
		header.Bytes == ShmHeaderV1Bytes && header.Kind == kind && header.NumAxes == axes &&
		r.offset >= ShmHeaderV1Bytes && r.offset%4 == 0 && r.capacity == expectedCapacity && header.RingMsgBytes == GRADIENT_RING_MSG_BYTES &&
		uint64(r.offset)+32+uint64(r.capacity)*GRADIENT_RING_MSG_BYTES <= uint64(len(mapping))
	if valid {
		// Validate immutable geometry without reading the concurrently owned counters.
		var private [RingHeaderV1Bytes]byte
		copy(private[:RingHeaderV1WriteIdxOffset], mapping[r.offset:r.offset+RingHeaderV1WriteIdxOffset])
		copy(private[RingHeaderV1Reserved0Offset:], mapping[r.offset+RingHeaderV1Reserved0Offset:r.offset+RingHeaderV1Bytes])
		h, err := DecodeRingHeaderV1(private[:])
		valid = err == nil && h.Magic == kMagicRing && h.Capacity == r.capacity && h.MsgBytes == GRADIENT_RING_MSG_BYTES
	}
	if !valid {
		_ = unix.Munmap(mapping)
		return ring{}, errors.New("invalid RTCore ring header/geometry")
	}
	return r, nil
}

type Client struct {
	options                      Options
	conn                         *net.UnixConn
	fds                          []int
	commands, statuses           ring
	axes                         uint32
	cycle                        time.Duration
	incarnation                  [16]byte // Immutable identity received in WELCOME.
	commandMu, socketMu, stateMu sync.Mutex
	drainMu                      sync.Mutex
	sequence                     uint64
	revocation                   atomic.Uint64
	stopOnce, releaseOnce        sync.Once
	done                         chan struct{}
	wg                           sync.WaitGroup
	failure                      error
	motion                       MotionState
	jogDebug                     StatusJogDebugV1
	results                      map[uint64]CommandResult
	upload                       *trajectoryUpload // stateMu protects receipt capture; commandMu owns its lifetime.
	stopAck                      atomic.Uint64
	fastMu                       sync.RWMutex
	fastPublishMu, grantMu       sync.Mutex
	fast                         *applicationGrantRegion
	grantTicket                  uint64 // grantMu owns request/receipt correlation.
	admittedGrant                GrantReceipt
	grantUncertain               bool
	grantUncertainStopAck        uint64
	jogControlMu                 sync.Mutex // Only Begin/End receipts; never bulk or updates.
	jogPublishMu                 sync.Mutex // One bounded latest-value publication only.
	pathPublishMu                sync.Mutex // One path chunk in flight until native acknowledges it.
	jogTicket                    uint64
}

func validIdentityDigest(value string) bool {
	digest, err := hex.DecodeString(value)
	return err == nil && len(digest) == 32 && value == hex.EncodeToString(digest)
}

func Dial(ctx context.Context, options Options) (_ *Client, err error) {
	if options.MachineSHA256 != "" && !validIdentityDigest(options.MachineSHA256) {
		return nil, errors.New("RTCore requires a lowercase machine SHA256")
	}
	digest, err := hex.DecodeString(options.ConfigurationSHA256)
	if err != nil || len(digest) != 32 || options.ConfigurationSHA256 != hex.EncodeToString(digest) {
		return nil, errors.New("RTCore requires an explicit lowercase configuration SHA256")
	}
	if options.Backend != "ethercat" && options.Backend != "simulation" {
		return nil, errors.New("invalid RTCore backend")
	}
	if options.SimulationTest && options.Backend != "simulation" {
		return nil, errors.New("RTCore simulation test role requires the simulation backend")
	}
	if options.SocketPath == "" {
		options.SocketPath = "/run/rosie-rt-core/ipc.sock"
	}
	connection, err := (&net.Dialer{}).DialContext(ctx, "unixpacket", options.SocketPath)
	if err != nil {
		return nil, err
	}
	c := &Client{options: options, conn: connection.(*net.UnixConn), done: make(chan struct{}), results: make(map[uint64]CommandResult)}
	defer func() {
		if err != nil {
			_ = c.Close()
		}
	}()
	deadline := time.Now().Add(2 * time.Second)
	if d, ok := ctx.Deadline(); ok && d.Before(deadline) {
		deadline = d
	}
	if err = c.conn.SetDeadline(deadline); err != nil {
		return nil, err
	}
	role, descriptorCount := uint32(kRoleAdapter), 5
	if options.SimulationTest {
		role, descriptorCount = kRoleController, 4
	}
	hello := EncodeHelloV1(HelloV1{Magic: kMagicGipc, VerMajor: VersionMajor, VerMinor: VersionMinor,
		Bytes: HelloV1Bytes, Role: role, Pid: uint64(os.Getpid())})
	if _, err = c.conn.Write(hello[:]); err != nil {
		return nil, err
	}
	welcome, ancillary := make([]byte, WelcomeV1Bytes), make([]byte, unix.CmsgSpace(5*4))
	n, oobn, flags, _, readErr := c.conn.ReadMsgUnix(welcome, ancillary)
	// Always collect received descriptors before rejecting a truncated welcome.
	messages, parseErr := unix.ParseSocketControlMessage(ancillary[:oobn])
	for _, message := range messages {
		if message.Header.Level != unix.SOL_SOCKET || message.Header.Type != unix.SCM_RIGHTS {
			continue
		}
		rights, e := unix.ParseUnixRights(&message)
		if e != nil {
			parseErr = e
			continue
		}
		for _, fd := range rights {
			unix.CloseOnExec(fd)
		}
		c.fds = append(c.fds, rights...)
	}
	if readErr != nil {
		return nil, readErr
	}
	if parseErr != nil {
		return nil, parseErr
	}
	welcomeFrame, frameErr := DecodeWelcomeV1(welcome[:n])
	if frameErr != nil || flags&(unix.MSG_TRUNC|unix.MSG_CTRUNC) != 0 || len(c.fds) != descriptorCount ||
		welcomeFrame.Magic != kMagicGipc || welcomeFrame.VerMinor != VersionMinor || welcomeFrame.Bytes != WelcomeV1Bytes {
		return nil, errors.New("invalid RTCore welcome/version")
	}
	c.axes = welcomeFrame.NumAxes
	c.cycle = time.Duration(welcomeFrame.CycleNs)
	if c.axes == 0 || c.axes > maxAxes || c.cycle < 250*time.Microsecond || c.cycle > 10*time.Millisecond ||
		welcomeFrame.CmdRingCapacity != GRADIENT_CMD_RING_CAPACITY || welcomeFrame.CmdMsgBytes != GRADIENT_RING_MSG_BYTES ||
		welcomeFrame.StatusRingCapacity != GRADIENT_STATUS_RING_CAPACITY || welcomeFrame.StatusMsgBytes != GRADIENT_RING_MSG_BYTES {
		return nil, errors.New("invalid RTCore welcome geometry")
	}
	if welcomeFrame.DaemonIncarnation == [16]byte{} {
		return nil, errors.New("invalid RTCore welcome incarnation")
	}
	c.incarnation = welcomeFrame.DaemonIncarnation
	if c.commands, err = mapRing(c.fds[0], 1, c.axes); err != nil {
		return nil, err
	}
	if c.statuses, err = mapRing(c.fds[1], 2, c.axes); err != nil {
		return nil, err
	}
	if !options.SimulationTest {
		if c.fast, err = mapApplicationGrantRegion(c.fds[4], digest, c.axes, c.cycle); err != nil {
			return nil, err
		}
		if c.fast.header.DaemonIncarnation != c.incarnation {
			return nil, errors.New("RTCore application grant incarnation mismatch")
		}
	}
	if err = c.conn.SetDeadline(time.Time{}); err != nil {
		return nil, err
	}
	if _, err = c.Status(); err != nil {
		if errors.Is(err, errDiagnosticIncarnation) {
			return nil, fmt.Errorf("%w: %v", ErrDiagnosticsNotPublished, err)
		}
		return nil, err
	}
	if options.MachineSHA256 != "" {
		if _, err = c.Describe(); err != nil {
			return nil, err
		}
	}
	if err = c.socketPacket("RTHEART1"); err != nil {
		return nil, err
	}
	c.wg.Add(1)
	go c.service()
	return c, nil
}

func monotonicNS() uint64 {
	var ts unix.Timespec
	if err := unix.ClockGettime(unix.CLOCK_MONOTONIC, &ts); err != nil {
		panic(err)
	}
	return uint64(ts.Nano())
}

func (c *Client) NumAxes() int         { return int(c.axes) }
func (c *Client) Cycle() time.Duration { return c.cycle }

// Incarnation returns the immutable native handshake identity for either role,
// preventing observer bindings from depending on application grant authority.
func (c *Client) Incarnation() string { return hex.EncodeToString(c.incarnation[:]) }

func (c *Client) Err() error {
	c.stateMu.Lock()
	defer c.stateMu.Unlock()
	return c.failure
}

func (c *Client) fail(err error) {
	c.stopOnce.Do(func() {
		c.stateMu.Lock()
		c.failure = err
		c.stateMu.Unlock()
		c.fastPublishMu.Lock()
		c.revocation.Add(1)
		c.fastPublishMu.Unlock()
		close(c.done)
		_ = c.conn.Close() // Closing the control connection revokes the daemon lease.
	})
}

func (c *Client) Close() error {
	c.fail(ErrClosed)
	c.closeApplicationGrantRegion()
	c.wg.Wait()
	c.releaseOnce.Do(func() {
		c.commandMu.Lock()
		defer c.commandMu.Unlock()
		c.drainMu.Lock()
		defer c.drainMu.Unlock()
		for _, mapping := range [][]byte{c.commands.data, c.statuses.data} {
			if mapping != nil {
				_ = unix.Munmap(mapping)
			}
		}
		for _, fd := range c.fds {
			_ = unix.Close(fd)
		}
	})
	return nil
}

func (c *Client) socketPacket(packet string) error {
	c.socketMu.Lock()
	defer c.socketMu.Unlock()
	if err := c.conn.SetWriteDeadline(time.Now().Add(100 * time.Millisecond)); err != nil {
		return err
	}
	n, err := c.conn.Write([]byte(packet))
	if err == nil && n != len(packet) {
		err = errors.New("short RTCore control packet")
	}
	return err
}

func (c *Client) service() {
	defer c.wg.Done()
	ticker := time.NewTicker(10 * time.Millisecond)
	defer ticker.Stop()
	nextHeartbeat := time.Now().Add(50 * time.Millisecond)
	for {
		select {
		case <-c.done:
			return
		case now := <-ticker.C:
			_ = c.conn.SetReadDeadline(time.Now().Add(time.Millisecond))
			var urgent [17]byte
			n, readErr := c.conn.Read(urgent[:])
			if readErr == nil {
				if n != 16 || string(urgent[:8]) != "RTSTOPAK" {
					c.fail(errors.New("invalid RTCore urgent response"))
					return
				}
				c.stopAck.Store(wire.Uint64(urgent[8:]))
			} else if timeout, ok := readErr.(net.Error); !ok || !timeout.Timeout() {
				c.fail(readErr)
				return
			}
			if err := c.drain(); err != nil {
				c.fail(err)
				return
			}
			if !now.Before(nextHeartbeat) {
				if err := c.socketPacket("RTHEART1"); err != nil {
					c.fail(err)
					return
				}
				nextHeartbeat = now.Add(50 * time.Millisecond)
			}
		}
	}
}

func (c *Client) drain() error {
	c.drainMu.Lock()
	defer c.drainMu.Unlock()
	select {
	case <-c.done:
		return ErrClosed
	default:
	}
	written, read := c.statuses.indices()
	if written-read > c.statuses.capacity {
		return errors.New("RTCore status ring overrun")
	}
	for read != written {
		offset := c.statuses.offset + 32 + (read%c.statuses.capacity)*GRADIENT_RING_MSG_BYTES
		slot := c.statuses.data[offset : offset+GRADIENT_RING_MSG_BYTES]
		header, err := DecodeMsgHeader(slot[:MsgHeaderBytes])
		if err != nil {
			return err
		}
		kind, size := header.Type, header.Bytes
		if size < MsgHeaderBytes || size > GRADIENT_RING_MSG_BYTES {
			return errors.New("invalid RTCore status envelope")
		}
		if kind == MsgCommandResult {
			if size != MsgHeaderBytes+CommandResultBytes {
				return errors.New("invalid RTCore result size")
			}
			result, err := DecodeCommandResultFrame(slot[MsgHeaderBytes:size])
			if err != nil {
				return err
			}
			c.stateMu.Lock()
			if u := c.upload; u != nil && result.Operation == MSG_CMD_TRAJECTORY_POINT &&
				result.Result == ResultRejected && result.Sequence >= u.firstSequence &&
				result.Sequence-u.firstSequence < uint64(u.pointCount) &&
				(u.refusal == nil || result.Sequence < u.refusal.Rejection.Result.Sequence) {
				u.refusal = &TrajectoryPointError{PointIndex: uint32(result.Sequence - u.firstSequence), Rejection: &RejectedError{result}}
			}
			c.results[result.Sequence] = result
			if len(c.results) > 128 {
				oldest := ^uint64(0)
				for seq := range c.results {
					if seq < oldest {
						oldest = seq
					}
				}
				delete(c.results, oldest)
			}
			c.stateMu.Unlock()
		}
		if kind == MSG_STATUS_MOTION_STATE {
			if size != MsgHeaderBytes+StatusMotionStateV1Bytes {
				return errors.New("invalid RTCore motion-state size")
			}
			p, err := DecodeStatusMotionStateV1(slot[MsgHeaderBytes:size])
			if err != nil {
				return err
			}
			state := MotionState{Mode: p.ActiveMode, State: p.State, TrajectoryID: p.ActiveTrajId,
				PointIndex: p.CurrentPointIndex, QueueDepth: p.QueueDepth, LastEvent: p.LastEventCode,
				Done: p.MotionDone != 0, CommandSequence: p.ActiveCommandSeq, TimeNS: p.LastUpdateNs}
			c.stateMu.Lock()
			c.motion = state
			c.stateMu.Unlock()
		}
		if kind == MsgStatusJogDebug {
			debug, err := DecodeStatusJogDebugV1Frame(slot[24:size])
			if err != nil {
				return errors.New("invalid RTCore jog-debug size")
			}
			c.stateMu.Lock()
			c.jogDebug = debug
			c.stateMu.Unlock()
		}
		read++
	}
	atomic.StoreUint32(c.statuses.counter(RingHeaderV1ReadIdxOffset), read)
	return nil
}

// RejectedError contains the native request result, including its generation.
type RejectedError struct{ Result CommandResult }

func (e *RejectedError) Error() string {
	return fmt.Sprintf("RTCore rejected operation %#x: reason %d", e.Result.Operation, e.Result.Reason)
}

// TrajectoryPointError retains the refusing point's zero-based upload index and
// native receipt, preventing a later COMMIT refusal from hiding the cause.
type TrajectoryPointError struct {
	PointIndex uint32
	Rejection  *RejectedError
}

func (e *TrajectoryPointError) Error() string {
	return fmt.Sprintf("RTCore trajectory point %d: %v", e.PointIndex, e.Rejection)
}
func (e *TrajectoryPointError) Unwrap() error { return e.Rejection }

type trajectoryUpload struct {
	firstSequence uint64
	pointCount    int
	refusal       *TrajectoryPointError
}

// uploadError retains the first refusal independently of bounded result history,
// so backpressure and later revoked receipts cannot erase its diagnostic.
func (c *Client) uploadError(upload *trajectoryUpload) error {
	if upload == nil {
		return nil
	}
	c.stateMu.Lock()
	defer c.stateMu.Unlock()
	if upload.refusal != nil {
		return upload.refusal
	}
	return nil
}

func (c *Client) WaitResult(ctx context.Context, sequence uint64) (CommandResult, error) {
	return c.waitResult(ctx, sequence, c.revocation.Load())
}

// Shared grant state can revoke an upload before the service drains its refusal.
// Read only receipts already in the ring; never wait for future acknowledgement.
func (c *Client) uploadTerminalError(upload *trajectoryUpload, cause error) error {
	if upload != nil {
		if refusal := c.uploadError(upload); refusal != nil {
			return refusal
		}
		drainErr := c.drain()
		if refusal := c.uploadError(upload); refusal != nil {
			return refusal
		}
		return errors.Join(cause, drainErr)
	}
	return cause
}
func (c *Client) waitResult(ctx context.Context, sequence, generation uint64) (CommandResult, error) {
	return c.waitUploadResult(ctx, sequence, generation, nil)
}

func (c *Client) waitUploadResult(ctx context.Context, sequence, generation uint64, upload *trajectoryUpload) (CommandResult, error) {
	if _, bounded := ctx.Deadline(); !bounded {
		var cancel context.CancelFunc
		ctx, cancel = context.WithTimeout(ctx, 3*time.Second)
		defer cancel()
	}
	ticker := time.NewTicker(5 * time.Millisecond)
	defer ticker.Stop()
	for {
		if err := c.uploadError(upload); err != nil {
			return CommandResult{}, err
		}
		if generation != c.revocation.Load() {
			return CommandResult{}, c.uploadTerminalError(upload, ErrRevoked)
		}
		c.stateMu.Lock()
		result, ok := c.results[sequence]
		if ok {
			delete(c.results, sequence)
		}
		c.stateMu.Unlock()
		if ok {
			if err := c.uploadError(upload); err != nil {
				return CommandResult{}, err
			}
			if result.Result == ResultRejected {
				return result, &RejectedError{result}
			}
			return result, nil
		}
		if err := c.Err(); err != nil {
			return CommandResult{}, err
		}
		select {
		case <-ctx.Done():
			return CommandResult{}, c.uploadTerminalError(upload, ctx.Err())
		case <-c.done:
			return CommandResult{}, ErrClosed
		case <-ticker.C:
		}
	}
}

func (c *Client) Motion() (MotionState, error) {
	c.stateMu.Lock()
	state, err := c.motion, c.failure
	c.stateMu.Unlock()
	if err != nil {
		return state, err
	}
	now := monotonicNS()
	if state.TimeNS == 0 || state.TimeNS > now || now-state.TimeNS > uint64(time.Second) {
		return state, errors.New("RTCore motion observation is stale")
	}
	return state, nil
}

// Status binds diagnostic evidence to the connected daemon. A recent file from
// a predecessor cannot restore its Home mask while a successor starts publishing.
func (c *Client) Status() (Status, error) {
	var state Status
	if err := c.Err(); err != nil {
		return state, err
	}
	raw, err := os.ReadFile(filepath.Join(filepath.Dir(c.options.SocketPath), "metrics.json"))
	if err != nil {
		return state, err
	}
	if err = json.Unmarshal(raw, &state); err != nil {
		return state, err
	}
	if !c.options.SimulationTest {
		clock, err := c.LocalJogClock()
		if err != nil {
			return Status{}, err
		}
		if state.DaemonIncarnation != clock.Incarnation {
			return Status{}, errDiagnosticIncarnation
		}
	}
	now := monotonicNS()
	if state.TimeNS == 0 || state.TimeNS > now || now-state.TimeNS > uint64(time.Second) {
		return state, errors.New("RTCore diagnostic observation is stale")
	}
	if state.ConfigurationSHA256 != c.options.ConfigurationSHA256 || state.Backend != c.options.Backend || len(state.Axes) != int(c.axes) {
		return state, errors.New("RTCore configuration/backend identity mismatch")
	}
	return state, nil
}

// Wait observes a fresh diagnostic snapshot. Callers must check the precise
// effect they requested, including masks; a successful socket/ring write is not
// an acknowledgement. Context expiry does not automatically stop motion.
func (c *Client) Wait(ctx context.Context, predicate func(Status) bool) (Status, error) {
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	for {
		state, err := c.Status()
		if err != nil {
			return state, err
		}
		if predicate(state) {
			return state, nil
		}
		select {
		case <-ctx.Done():
			return state, ctx.Err()
		case <-c.done:
			return state, c.Err()
		case <-ticker.C:
		}
	}
}

func (c *Client) validMask(mask uint32) bool { return mask != 0 && mask & ^((1<<c.axes)-1) == 0 }

// sendLocked is the only command-ring writer. Sequence and timestamp are made
// at publication, after backpressure. Disarm bypasses this lock and invalidates
// any upload waiting here, even during a long trajectory upload.
func (c *Client) sendLocked(ctx context.Context, generation uint64, kind uint16, payload []byte) (uint64, error) {
	return c.sendUploadLocked(ctx, generation, kind, payload, nil)
}

func (c *Client) sendUploadLocked(ctx context.Context, generation uint64, kind uint16, payload []byte, upload *trajectoryUpload) (uint64, error) {
	for {
		if err := c.uploadError(upload); err != nil {
			return 0, err
		}
		if err := ctx.Err(); err != nil {
			return 0, err
		}
		if err := c.Err(); err != nil {
			return 0, err
		}
		if generation != c.revocation.Load() {
			return 0, ErrRevoked
		}
		written, read := c.commands.indices()
		if written-read > c.commands.capacity {
			c.fail(errors.New("RTCore command ring corrupt"))
			return 0, c.Err()
		}
		if written-read < c.commands.capacity {
			offset := c.commands.offset + 32 + (written%c.commands.capacity)*GRADIENT_RING_MSG_BYTES
			slot := c.commands.data[offset : offset+GRADIENT_RING_MSG_BYTES]
			c.sequence++
			header := EncodeMsgHeader(MsgHeader{Type: kind, Bytes: uint32(MsgHeaderBytes + len(payload)), Seq: c.sequence, TimeNs: monotonicNS()})
			copy(slot, header[:])
			copy(slot[MsgHeaderBytes:], payload)
			atomic.StoreUint32(c.commands.counter(RingHeaderV1WriteIdxOffset), written+1)
			var wake [8]byte
			wire.PutUint64(wake[:], 1)
			if _, err := unix.Write(c.fds[2], wake[:]); err != nil && err != unix.EAGAIN {
				c.fail(err)
				return 0, err
			}
			return c.sequence, nil
		}
		select {
		case <-ctx.Done():
			return 0, ctx.Err()
		case <-c.done:
			return 0, c.Err()
		case <-time.After(time.Millisecond):
		}
	}
}

func (c *Client) command(ctx context.Context, kind uint16, payload []byte) (uint64, error) {
	return c.commandGeneration(ctx, c.revocation.Load(), kind, payload)
}
func (c *Client) commandGeneration(ctx context.Context, generation uint64, kind uint16, payload []byte) (uint64, error) {
	c.commandMu.Lock()
	defer c.commandMu.Unlock()
	if _, err := c.Status(); err != nil {
		return 0, err
	}
	return c.sendLocked(ctx, generation, kind, payload)
}

func (c *Client) maskCommand(ctx context.Context, kind uint16, mask uint32) (uint64, error) {
	if !c.validMask(mask) {
		return 0, errors.New("invalid RTCore axis mask")
	}
	p := EncodeCmdAxisMaskV1(CmdAxisMaskV1{AxisMask: mask})
	return c.command(ctx, kind, p[:])
}

func (c *Client) Enable(ctx context.Context, mask uint32) (uint64, error) {
	return c.maskCommand(ctx, MSG_CMD_AXIS_ENABLE, mask)
}
func (c *Client) RestoreReferenceVerifiedDelta(ctx context.Context, mask uint32) (uint64, error) {
	if !c.validMask(mask) || mask&(mask-1) != 0 {
		return 0, errors.New("invalid RTCore single-axis mask")
	}
	p := EncodeCmdRestoreReferenceVerifiedDeltaV1(CmdRestoreReferenceVerifiedDeltaV1{AxisMask: mask})
	return c.command(ctx, MSG_CMD_RESTORE_REFERENCE_VERIFIED_DELTA, p[:])
}

func (c *Client) Home(ctx context.Context, mask uint32) (uint64, error) {
	if !c.validMask(mask) {
		return 0, errors.New("invalid RTCore axis mask")
	}
	p := EncodeCmdNativeHomeV1(CmdNativeHomeV1{AxisMask: mask})
	return c.command(ctx, MSG_CMD_NATIVE_HOME, p[:])
}

// RestoreAnchor requests an explicit inhibited commissioning job. Bit 31 of
// the existing 32-bit mask is outside the 16 physical axes; the native 1.6
// generated eight-byte payload and zero flags remain unchanged.
func (c *Client) RestoreAnchor(ctx context.Context, mask uint32) (uint64, error) {
	if !c.validMask(mask) {
		return 0, errors.New("invalid RTCore axis mask")
	}
	// Native 1.6 has 16 physical axis bits; bit 31 selects restoration.
	const restoreAnchorSelector uint32 = 1 << 31
	p := EncodeCmdNativeHomeV1(CmdNativeHomeV1{AxisMask: mask | restoreAnchorSelector})
	return c.command(ctx, MSG_CMD_NATIVE_HOME, p[:])
}
func (c *Client) Arm(ctx context.Context) (uint64, error) {
	p := EncodeCmdArmV1(CmdArmV1{Arm: 1})
	return c.command(ctx, MSG_CMD_ARM, p[:])
}

// Disarm sends the urgent socket request, independent of queued commands and
// the daemon's mailbox helper. Wait for the observed inhibited state afterward.
func (c *Client) Disarm() error {
	if err := c.disarmUrgent(); err != nil {
		return err
	}
	// Fence the last publication of an already-running writer. Urgent revoke
	// is sent before waiting, and ring backpressure checks revocation each ms.
	c.commandMu.Lock()
	defer c.commandMu.Unlock()
	ticket := monotonicNS()
	var packet [16]byte
	copy(packet[:], "RTSTOP02")
	wire.PutUint64(packet[8:], ticket)
	if err := c.socketPacket(string(packet[:])); err != nil {
		return err
	}
	deadline := time.Now().Add(2 * time.Second)
	for c.stopAck.Load() != ticket {
		if err := c.Err(); err != nil {
			return err
		}
		if time.Now().After(deadline) {
			return errors.New("RTCore output-inhibition acknowledgement unavailable")
		}
		time.Sleep(time.Millisecond)
	}
	return nil
}

func (c *Client) disarmUrgent() error {
	c.fastPublishMu.Lock()
	c.revocation.Add(1)
	c.fastPublishMu.Unlock()
	if err := c.Err(); err != nil {
		return err
	}
	if err := c.socketPacket("RTDISARM"); err != nil {
		c.fail(err)
		return err
	}
	return nil
}

func (c *Client) Jog(ctx context.Context, mask uint32, velocity []float64, timeout time.Duration) (uint64, error) {
	if !c.validMask(mask) || len(velocity) != int(c.axes) || timeout <= 0 || timeout > 250*time.Millisecond {
		return 0, errors.New("invalid RTCore jog dimensions/timeout")
	}
	jog := CmdJogV1{AxisMask: mask, Flags: JOG_FLAG_ACTIVE, TimeoutNs: uint64(timeout)}
	for i, qd := range velocity {
		if math.IsNaN(qd) || math.IsInf(qd, 0) {
			return 0, errors.New("non-finite RTCore jog velocity")
		}
		jog.Qd[i] = qd
	}
	p := EncodeCmdJogV1(jog)
	return c.command(ctx, MSG_CMD_JOG, p[:])
}

// Stop cancels every motion mode and inhibits outputs through the urgent path.
// It returns no command sequence (zero): observe inhibited status, then explicitly
// Enable/Arm for new motion. It is not a controlled ramp or standstill proof.
// Cancellation of ctx must not suppress an output-inhibition request.
func (c *Client) Stop(ctx context.Context) (uint64, error) {
	return 0, c.Disarm()
}

func (c *Client) PrepareTrajectory(ctx context.Context, id uint64, mask uint32, points []Point) (uint64, error) {
	return c.prepareTrajectory(ctx, c.revocation.Load(), id, mask, points)
}
func (c *Client) prepareTrajectory(ctx context.Context, generation, id uint64, mask uint32, points []Point) (uint64, error) {
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	if generation != c.revocation.Load() {
		return 0, ErrRevoked
	}
	if id == 0 || !c.validMask(mask) || len(points) < 2 || len(points) > MaxTrajectoryPoints {
		return 0, errors.New("invalid RTCore trajectory identity/size")
	}
	// Encode and validate every sample before BEGIN. The core independently
	// checks limits, implied segment speeds, state and complete upload count.
	encoded := make([][]byte, len(points))
	for i, point := range points {
		if generation != c.revocation.Load() {
			return 0, ErrRevoked
		}
		if err := ctx.Err(); err != nil {
			return 0, err
		}
		if len(point.Position) != int(c.axes) || (point.Velocity != nil && len(point.Velocity) != int(c.axes)) ||
			point.Time < 0 || (i > 0 && point.Time <= points[i-1].Time) {
			return 0, fmt.Errorf("invalid RTCore trajectory point %d", i)
		}
		sample := TrajectoryPointV1{TrajId: id, PointIndex: uint32(i), TFromStartNs: uint64(point.Time), AxisMask: mask}
		if point.IOMask > 65535 || point.IOValues & ^point.IOMask != 0 {
			return 0, errors.New("invalid I/O marker mask")
		}
		flags := (point.IOMask&255)<<IO_MARKER_MASK_SHIFT | (point.IOValues&255)<<IO_MARKER_VALUE_SHIFT
		sample.IoHigh = (point.IOMask >> 8) | ((point.IOValues >> 8) << 8)
		sample.WaitTimeoutNs = point.WaitTimeoutNS
		sample.WaitStableNs = point.WaitStableNS
		sample.MonitorMask = point.MonitorMask
		sample.MonitorValues = point.MonitorValues
		sample.WaitInput = point.WaitInput
		sample.WaitValue = point.WaitValue
		if point.Velocity != nil {
			flags |= 1
		}
		if i == len(points)-1 {
			flags |= 2
		}
		sample.Flags = flags
		for axis, q := range point.Position {
			if math.IsNaN(q) || math.IsInf(q, 0) {
				return 0, errors.New("non-finite RTCore position")
			}
			sample.Q[axis] = q
		}
		for axis, qd := range point.Velocity {
			if math.IsNaN(qd) || math.IsInf(qd, 0) {
				return 0, errors.New("non-finite RTCore feedforward")
			}
			sample.Qd[axis] = qd
		}
		p := EncodeTrajectoryPointV1(sample)
		encoded[i] = p[:]
	}
	c.commandMu.Lock()
	defer c.commandMu.Unlock()
	if _, err := c.Status(); err != nil {
		return 0, err
	}
	abort := func(err error) (uint64, error) {
		// A failed publication may have left native upload state ambiguous.
		// Tell the adapter that the urgent fallback invalidated every handle,
		// even when the original error was a caller timeout or transport error.
		return 0, errors.Join(err, ErrRevoked, c.disarmUrgent())
	}
	begin := EncodeCmdTrajectoryBeginV1(CmdTrajectoryBeginV1{TrajId: id, AxisMask: mask, ExpectedPoints: uint32(len(points))})
	beginSeq, err := c.sendLocked(ctx, generation, MSG_CMD_TRAJECTORY_BEGIN, begin[:])
	if err != nil {
		return abort(err)
	}
	if _, err := c.waitResult(ctx, beginSeq, generation); err != nil {
		return 0, err
	}
	upload := &trajectoryUpload{firstSequence: beginSeq + 1, pointCount: len(points)}
	c.stateMu.Lock()
	c.upload = upload
	c.stateMu.Unlock()
	defer func() {
		c.stateMu.Lock()
		c.upload = nil
		c.stateMu.Unlock()
	}()
	uploadFailure := func(err error) (uint64, error) {
		err = c.uploadTerminalError(upload, err)
		var refusal *TrajectoryPointError
		if errors.As(err, &refusal) {
			return 0, err
		}
		return abort(err)
	}
	for _, p := range encoded {
		if _, err := c.sendUploadLocked(ctx, generation, MSG_CMD_TRAJECTORY_POINT, p, upload); err != nil {
			return uploadFailure(err)
		}
	}
	commit := EncodeCmdTrajectoryControlV1(CmdTrajectoryControlV1{TrajId: id})
	seq, err := c.sendUploadLocked(ctx, generation, MSG_CMD_TRAJECTORY_COMMIT, commit[:], upload)
	if err != nil {
		return uploadFailure(err)
	}
	result, err := c.waitUploadResult(ctx, seq, generation, upload)
	if err != nil {
		return 0, err
	}
	return result.Handle, nil
}

func (c *Client) StartTrajectory(ctx context.Context, handle uint64) (uint64, error) {
	return c.startTrajectory(ctx, c.revocation.Load(), handle)
}
func (c *Client) startTrajectory(ctx context.Context, generation, handle uint64) (uint64, error) {
	if handle == 0 {
		return 0, errors.New("invalid RTCore trajectory handle")
	}
	payload := EncodeTrajectoryHandle(TrajectoryHandle{Handle: handle})
	seq, err := c.commandGeneration(ctx, generation, MsgTrajectoryStart, payload[:])
	if err != nil {
		return 0, err
	}
	_, err = c.waitResult(ctx, seq, generation)
	return seq, err
}

// DiscardTrajectory releases a prepared handle without cancelling motion. An
// absent or retired handle is already discarded; a published Start or active
// execution is rejected with ReasonModeConflict. Stop is the cancellation path.
func (c *Client) DiscardTrajectory(ctx context.Context, handle uint64) (uint64, error) {
	if handle == 0 {
		return 0, errors.New("invalid RTCore trajectory handle")
	}
	generation := c.revocation.Load()
	payload := EncodeTrajectoryHandle(TrajectoryHandle{Handle: handle})
	seq, err := c.commandGeneration(ctx, generation, MsgTrajectoryDiscard, payload[:])
	if err != nil {
		return 0, err
	}
	_, err = c.waitResult(ctx, seq, generation)
	return seq, err
}

func (c *Client) Trajectory(ctx context.Context, id uint64, mask uint32, points []Point) (uint64, error) {
	generation := c.revocation.Load()
	handle, err := c.prepareTrajectory(ctx, generation, id, mask, points)
	if err != nil {
		return 0, err
	}
	return c.startTrajectory(ctx, generation, handle)
}

// AllOperationEnabled observes every supplied axis without changing authority.
// The empty slice remains vacuously enabled; callers retain their count/mask gates.
func AllOperationEnabled(s Status) bool {
	for _, axis := range s.Axes {
		if axis.Statusword&0x6f != 0x27 {
			return false
		}
	}
	return true
}
