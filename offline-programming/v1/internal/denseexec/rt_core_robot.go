//go:build linux

package denseexec

import (
	"context"
	"encoding/base64"
	"encoding/hex"
	"fmt"
	"io"
	"net/http"
	"sync"

	"rosieos/offline-programming/v1/internal/densejointtraj"
	"rosieos/offline-programming/v1/internal/robotconfig"
	"rosieos/robotdesc"
	"rosieos/rt-core/sdk/control"
)

// The rt-core side of the robot description contract
// (docs/ROBOT-DESCRIPTION-CONTRACT.md, docs/PLANNING-CONTEXT.md): what the
// machine describes is the description it was compiled with, file for file,
// and its machine_planning_calibration.json beside them, each as a content-addressed resource. This
// file reads that as the CellTarget packing asks and as the gate Load
// judges a plan's robot block by. Nothing here is asked of the joint
// daemon; that path is gone.

// resourceServer is the SDK client's resource read, asserted rather than
// added to RTCoreClient so a fake that never serves resources is a machine
// that binds no robot, not a compile error.
type resourceServer interface {
	Resource(context.Context, string) (io.ReadCloser, error)
}

// robotCellOf is the machine's identity under the contract, as it describes
// it: the model, the description identity and the machine_planning_calibration.json identity, with
// the served files listed beside them by description-relative path. The
// identity is not taken on faith: it is recomputed over the listing (the
// same entries, by path and digest, that rtctl hashed when it pinned them)
// and must agree with what the machine says, so a listing that lies about
// itself is not a robot. No bytes are fetched here.
//
// A machine that binds no robot description answers Configured false with
// the reason: it has no robot to plan against and says so.
//
// A Describe with no robot AND no drives is something else: an adapter
// started without its compiled configuration (rt-control --compiled-config,
// or ROSIE_RT_COMPILED_CONFIG). Both blocks come out of that one snapshot
// (rt-core/adapters/rosie/control/resources.go describeResources), so a
// robot-bound machine launched that way describes itself exactly like a
// flat simulator would -- and would play any plan unjudged. It answers
// Configured true and Valid false, so Load refuses it as unavailable rather
// than passing the plan; a genuinely flat machine has drives to show.
func robotCellOf(d control.Description) RobotCell {
	if d.Robot == nil && len(d.Drives) == 0 {
		return RobotCell{Configured: true, Error: "the machine's adapter serves no compiled configuration (rt-control started " +
			"without --compiled-config or ROSIE_RT_COMPILED_CONFIG), so it cannot say which robot it is"}
	}
	if d.Robot == nil {
		return RobotCell{Error: "the machine binds no robot description"}
	}
	model := d.Robot.ModelID
	var entries []robotdesc.Entry
	var cellSHA string
	manifest := false
	for _, resource := range d.Robot.Resources {
		if resource.Path == "machine_planning_calibration.json" {
			cellSHA = resource.SHA256
			continue
		}
		raw, err := hex.DecodeString(resource.SHA256)
		if err != nil || len(raw) != 32 {
			return RobotCell{Configured: true, ModelID: model,
				Error: fmt.Sprintf("the machine lists %s under a digest that is not a sha256", resource.Path)}
		}
		var entry robotdesc.Entry
		entry.Path = resource.Path
		copy(entry.SHA256[:], raw)
		entries = append(entries, entry)
		manifest = manifest || resource.Path == robotdesc.Manifest
	}
	switch {
	case model == "":
		return RobotCell{Configured: true, Error: "the machine names no model"}
	case !manifest:
		return RobotCell{Configured: true, ModelID: model,
			Error: "the machine serves a description without its robot_description_manifest.json"}
	case cellSHA == "":
		return RobotCell{Configured: true, ModelID: model,
			Error: "the machine serves a description and no machine_planning_calibration.json beside it"}
	case robotdesc.HashEntries(entries) != d.Robot.RobotDescriptionSHA256:
		return RobotCell{Configured: true, ModelID: model,
			Error: fmt.Sprintf("the machine advertises description %s and lists files that hash to %s", d.Robot.RobotDescriptionSHA256, robotdesc.HashEntries(entries))}
	case "sha256:"+cellSHA != d.Robot.MachinePlanningCalibrationSHA256:
		return RobotCell{Configured: true, ModelID: model,
			Error: fmt.Sprintf("the machine advertises machine_planning_calibration.json %s and serves %s", d.Robot.MachinePlanningCalibrationSHA256, "sha256:"+cellSHA)}
	}
	configuration := ""
	if digest(d.ConfigurationSHA256) {
		configuration = "sha256:" + d.ConfigurationSHA256
	}
	// Present means the machine pins a calibration of its own: the served
	// bytes are not the model's empty document (the one a machine that pins
	// none serves, docs/ROBOT-DESCRIPTION-CONTRACT.md "No file").
	present := d.Robot.MachinePlanningCalibrationSHA256 !=
		robotdesc.MachinePlanningCalibrationSHA256(robotdesc.EmptyMachinePlanningCalibrationBytes(model))
	return RobotCell{Configured: true, Valid: true, ModelID: model, MachineConfigurationSHA256: configuration,
		RobotDescriptionSHA256: d.Robot.RobotDescriptionSHA256, MachinePlanningCalibrationSHA256: d.Robot.MachinePlanningCalibrationSHA256, MachinePlanningCalibrationPresent: present}
}

// robotResources is a cache of the small resources the contract reads
// again and again (the machine_planning_calibration.json; the description files on adoption), by
// digest: a content-addressed read never changes, so one fetch is enough
// for the life of the controller.
type robotResources struct {
	mu    sync.Mutex
	bytes map[string][]byte
}

// resource is one content-addressed read through the SDK, which verifies
// the body against the digest before handing it over (sdk/control/resources.go).
func (r *RTCoreController) resource(ctx context.Context, sha string) ([]byte, error) {
	r.resources.mu.Lock()
	cached, ok := r.resources.bytes[sha]
	r.resources.mu.Unlock()
	if ok {
		return cached, nil
	}
	server, ok := r.c.(resourceServer)
	if !ok {
		return nil, httpError(http.StatusConflict, ReasonRobotCellUnavailable, "the machine's client serves no resources")
	}
	ctx, cancel := context.WithTimeout(ctx, r.callBudget())
	defer cancel()
	body, err := server.Resource(ctx, sha)
	if err != nil {
		return nil, err
	}
	defer body.Close()
	data, err := io.ReadAll(body)
	if err != nil {
		return nil, err
	}
	r.resources.mu.Lock()
	if r.resources.bytes == nil {
		r.resources.bytes = map[string][]byte{}
	}
	r.resources.bytes[sha] = data
	r.resources.mu.Unlock()
	return data, nil
}

// robotCell is the machine's identity from its verified description, or
// the reason it has none as a typed refusal.
func (r *RTCoreController) robotCell(ctx context.Context) (control.Description, RobotCell, error) {
	d, err := r.describe(ctx)
	if err != nil {
		return d, RobotCell{}, err
	}
	cell := robotCellOf(d)
	if !cell.Valid {
		return d, cell, httpError(http.StatusConflict, ReasonRobotCellUnavailable, "%s", cell.Error)
	}
	return d, cell, nil
}

// Cell is the machine's identity and the exact machine_planning_calibration.json bytes it was
// compiled with, which the planner is handed verbatim (CellTarget).
func (r *RTCoreController) Cell(ctx context.Context) (*CellBody, error) {
	d, err := r.describe(ctx)
	if err != nil {
		return nil, err
	}
	cell := robotCellOf(d)
	// Flat-axis simulation has no calibration resources, but its verified
	// native execution limits still constrain planning against the local model.
	if !cell.Valid && !(d.Backend == "simulation" && !cell.Configured) {
		return nil, httpError(http.StatusConflict, ReasonRobotCellUnavailable, "%s", cell.Error)
	}
	var data []byte
	var resources []control.ResourceInfo
	if d.Robot != nil {
		resources = d.Robot.Resources
	}
	for _, resource := range resources {
		if resource.Path == "machine_planning_calibration.json" {
			data, err = r.resource(ctx, resource.SHA256)
			if err != nil {
				return nil, fmt.Errorf("the machine's machine_planning_calibration.json could not be read: %w", err)
			}
		}
	}
	limits := make(robotconfig.ExecutionLimits, len(d.Axes))
	for _, axis := range d.Axes {
		if _, duplicate := limits[axis.ID]; duplicate {
			return nil, httpError(http.StatusConflict, ReasonRobotCellUnavailable, "duplicate native axis %s", axis.ID)
		}
		var acceleration *float64
		if axis.MaxAcceleration != nil {
			value := *axis.MaxAcceleration
			acceleration = &value
		}
		limits[axis.ID] = robotconfig.ExecutionAxisLimit{
			PositionUnit: axis.PositionUnit, LowerRad: axis.MinPosition, UpperRad: axis.MaxPosition,
			VelocityRadS: axis.MaxVelocity, AccelerationRadS2: acceleration,
		}
	}
	return &CellBody{Robot: cell, MachinePlanningCalibrationBase64: base64.StdEncoding.EncodeToString(data), ExecutionLimits: limits}, nil
}

// Description is the description the machine holds, file for file by
// description-relative path, for the workstation to adopt under the
// identity the machine advertises (CellTarget). The adoption recomputes the
// identity over these bytes; this only fetches them.
func (r *RTCoreController) Description(ctx context.Context) (*DescriptionBody, error) {
	d, cell, err := r.robotCell(ctx)
	if err != nil {
		return nil, err
	}
	files := map[string][]byte{}
	for _, resource := range d.Robot.Resources {
		if resource.Path == "machine_planning_calibration.json" {
			continue
		}
		data, err := r.resource(ctx, resource.SHA256)
		if err != nil {
			return nil, fmt.Errorf("the machine's description file %s could not be read: %w", resource.Path, err)
		}
		files[resource.Path] = data
	}
	return &DescriptionBody{Robot: cell, Files: files}, nil
}

// robotGate is Load's early check of the plan's robot block against the
// machine (checkRobot), in the contract's words. A machine that binds no
// robot description has nothing to judge the block by and passes it, the
// way a daemon from before the contract did: it is a flat-axis machine, and
// which robot the plan was made for is not a question it can answer. An
// adapter that serves no compiled configuration at all is not that machine
// (robotCellOf) and is refused.
//
// Describe is asked FRESH here, not read from the controller's cache: the
// cache is filled once and never cleared, so after a recompile and restart
// of the adapter a Load would otherwise be judged against the identity the
// machine had when this controller first saw it. One round trip per Load.
func (r *RTCoreController) robotGate(ctx context.Context, plan *densejointtraj.Robot) (note string, err error) {
	d, err := r.describe(ctx)
	if err != nil {
		return "", err
	}
	if d.Robot == nil && len(d.Drives) > 0 {
		return "", nil
	}
	cell := robotCellOf(d)
	if refusal := checkRobot(plan, &cell); refusal != nil {
		return "", refusal
	}
	return machineConfigurationNote(plan, &cell), nil
}
