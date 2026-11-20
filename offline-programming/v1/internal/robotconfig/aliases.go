package robotconfig

// The robot description contract itself lives in rt-core (rosieos/rt-core/
// robotdesc): the identity, the URDF scan, the description store and the
// machine_planning_calibration.json rules are shared with rtctl, which pins a description into a
// cell's compiled configuration, so the two sides cannot disagree. What stays
// here is what only the OLP does with them: compiling the planning context
// (context.go) and keeping descriptions fetched from cells (fetched.go).
//
// The names below are that package's, re-exported so the OLP reads as it
// did; nothing here adds a rule.

import "rosieos/robotdesc"

type (
	Identity     = robotdesc.Identity
	Settings     = robotdesc.Settings
	JointLimit   = robotdesc.JointLimit
	Correction   = robotdesc.Correction
	Description  = robotdesc.Description
	Store        = robotdesc.Store
	Config       = robotdesc.Config
	Caps         = robotdesc.Caps
	Frames       = robotdesc.Frames
	WorkSurface  = robotdesc.WorkSurface
	PlanningRate = robotdesc.PlanningRate
	Robot        = robotdesc.Robot
	URDF         = robotdesc.URDF
	URDFJoint    = robotdesc.URDFJoint
)

const (
	CellSchema     = robotdesc.CellSchema
	ConfigSchema   = robotdesc.ConfigSchema
	Manifest       = robotdesc.Manifest
	ManifestSchema = robotdesc.ManifestSchema
	SourceTarget   = robotdesc.SourceTarget
	SourceEmpty    = robotdesc.SourceEmpty
)

var (
	At                                   = robotdesc.At
	Resolve                              = robotdesc.Resolve
	EmptyMachinePlanningCalibrationBytes = robotdesc.EmptyMachinePlanningCalibrationBytes
	MachinePlanningCalibrationSHA256     = robotdesc.MachinePlanningCalibrationSHA256
	RobotDescriptionSHA256               = robotdesc.RobotDescriptionSHA256
	HashFiles                            = robotdesc.HashFiles
	DescriptionFiles                     = robotdesc.DescriptionFiles
	ParseURDF                            = robotdesc.ParseURDF
)
