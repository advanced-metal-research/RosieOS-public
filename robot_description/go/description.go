package robotdesc

// The description as this process reads it: the directory, once, into values.
//
// Nothing on the planning path opens `robot_description/<model>/` but this: a
// second reader would be a second answer to "which robot is this?", and the
// whole contract is that there is one. (`internal/simulator` still names a
// URDF path of its own, for the Cartesian daemon it launches locally; that is
// the sim's own model file and never reaches a plan's identity.)

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"regexp"
	"sort"
	"strings"
)

// ConfigSchema is the only schema config.json may carry.
const ConfigSchema = "rosie.robot-config.v1"

// Caps is the largest kinematic correction a description admits, per
// component. A description without a config.json has caps of zero: without an
// agreed cap no correction is bounded, so none is accepted.
type Caps struct {
	XYZMetres  float64 `json:"xyz_m"`
	RPYRadians float64 `json:"rpy_rad"`
}

// Config is config.json, read for what the OLP needs of it: the caps, and the
// planning policy it hands on unchanged.
type Config struct {
	Caps             Caps
	Frames           Frames
	SpeedCeilingRadS float64
	PlanningJoints   map[string]PlanningRate
	Driven           []string
	Held             map[string]float64
	ResetPose        map[string]float64
	ToolFrame        string
	ElectrodeAxis    []float64
	Present          bool
}

// Frames names the links that play a role no URDF has a word for: what
// everything is measured in, where the torch is, and what a workpiece is
// fixtured to. A cell with a positioner fixtures to a surface ON a link
// rather than to its origin, so WorkSurface adds that point as a fixed child
// and Work names it.
type Frames struct {
	Base        string       `json:"base"`
	Tool        string       `json:"tool"`
	Work        string       `json:"work"`
	WorkSurface *WorkSurface `json:"work_surface,omitempty"`
}

// WorkSurface is the point on Parent a workpiece sits at.
type WorkSurface struct {
	Parent    string     `json:"parent"`
	XYZMetres [3]float64 `json:"xyz_m"`
}

// PlanningRate is one joint's planning policy: what the planner may
// parametrize against, never above the URDF's own rating.
type PlanningRate struct {
	VelocityRadS      float64 `json:"velocity_rad_s"`
	AccelerationRadS2 float64 `json:"acceleration_rad_s2"`
}

// Description is one robot model, read from disk.
type Description struct {
	ModelID  string
	Dir      string
	Identity string // "sha256:" + 64 hex, over the files the manifest names
	URDFPath string
	URDF     URDF
	URDFRaw  []byte
	Config   Config
}

// A Store is where the robot descriptions live. WHERE is the deployment's
// answer and not this package's: a workstation reads them out of the
// checkout, and a cell reads the same layout out of what was deployed to it.
// Baking `<repo>/robot_description` in here would make that a code change.
type Store struct{ Root string }

// At names the directory the model directories sit in.
func At(root string) Store { return Store{Root: root} }

// List is every model the store holds, by directory name, sorted. A directory
// without a manifest is not a model and is passed over rather than refused:
// the store is a place, and places collect things.
func (store Store) List() ([]string, error) {
	entries, err := os.ReadDir(store.Root)
	if err != nil {
		return nil, fmt.Errorf("no robot descriptions at %s: %w", store.Root, err)
	}
	models := make([]string, 0, len(entries))
	for _, entry := range entries {
		if !entry.IsDir() {
			continue
		}
		if _, err := os.Stat(filepath.Join(store.Root, entry.Name(), Manifest)); err == nil {
			models = append(models, entry.Name())
		}
	}
	sort.Strings(models)
	return models, nil
}

// HasGeometry is whether this description can be planned against: it has a
// robot.urdf. A bench robot -- motors on a table, no arm -- is a description
// with an identity and an rt-core definition and nothing to plan; the OLP
// lists it and refuses to plan for it, rt-core runs it.
func (description Description) HasGeometry() bool { return description.URDFPath != "" }

// Load reads <root>/<modelID>/ once and returns it. The model id is a
// directory name and nothing else: it arrives from a browser, so a path
// separator or a dot in it is a refusal, not a lookup.
func (store Store) Load(modelID string) (Description, error) {
	modelID = strings.TrimSpace(modelID)
	if modelID == "" {
		return Description{}, fmt.Errorf("cell_id is required")
	}
	if strings.ContainsAny(modelID, `/\.`) {
		return Description{}, fmt.Errorf("cell_id %q is not a cell name", modelID)
	}
	dir := filepath.Join(store.Root, modelID)
	if _, err := os.Stat(filepath.Join(dir, Manifest)); err != nil {
		return Description{}, fmt.Errorf("no cell %q under %s", modelID, store.Root)
	}
	identity, err := RobotDescriptionSHA256(dir)
	if err != nil {
		return Description{}, fmt.Errorf("cell %q: the description has no identity: %w", modelID, err)
	}
	urdfPath := filepath.Join(dir, "robot.urdf")
	raw, err := os.ReadFile(urdfPath)
	if os.IsNotExist(err) {
		// No geometry: a bench robot. Its identity is whole; there is
		// nothing here to plan against, and HasGeometry says so.
		return Description{ModelID: modelID, Dir: dir, Identity: identity}, nil
	}
	if err != nil {
		// A robot.urdf that is there and cannot be read is not a bench: it
		// is a description this process cannot see, and the catalogue must
		// name it as a problem rather than list a robot with nothing to plan.
		return Description{}, fmt.Errorf("cell %q: %w", modelID, err)
	}
	urdf, err := ParseURDF(string(raw))
	if err != nil {
		return Description{}, fmt.Errorf("cell %q: %w", modelID, err)
	}
	if urdf.RobotName != modelID {
		return Description{}, fmt.Errorf(
			"cell %q: robot.urdf names robot %q; a description directory and its robot are one name",
			modelID, urdf.RobotName)
	}
	config, err := loadConfig(dir, modelID, urdf)
	if err != nil {
		return Description{}, fmt.Errorf("cell %q: %w", modelID, err)
	}
	return Description{
		ModelID: modelID, Dir: dir, Identity: identity,
		URDFPath: urdfPath, URDF: urdf, URDFRaw: raw, Config: config,
	}, nil
}

type configDocument struct {
	Schema    string             `json:"schema"`
	ModelID   string             `json:"model_id"`
	Caps      *Caps              `json:"kinematic_correction_caps"`
	Frames    *Frames            `json:"frames"`
	Planning  *planningDocument  `json:"planning"`
	Axes      *axesDocument      `json:"axes"`
	ResetPose map[string]float64 `json:"reset_pose"`
	Torch     *torchDocument     `json:"torch"`
}

type planningDocument struct {
	SpeedCeilingRadS float64                 `json:"speed_ceiling_rad_s"`
	Joints           map[string]PlanningRate `json:"joints"`
}

type axesDocument struct {
	Driven []string           `json:"driven"`
	Held   map[string]float64 `json:"held"`
}

type torchDocument struct {
	ToolFrame     string    `json:"tool_frame"`
	ElectrodeAxis []float64 `json:"electrode_axis"`
}

// loadConfig reads config.json. Absent is a state, not an error -- a model
// nobody has configured still plans, at zero caps -- but a config.json that IS
// there and disagrees with the URDF is refused: a planning rate above the
// robot's rating, or a joint the URDF does not have.
func loadConfig(dir, modelID string, urdf URDF) (Config, error) {
	raw, err := os.ReadFile(filepath.Join(dir, "config.json"))
	if err != nil {
		return Config{}, nil
	}
	var document configDocument
	decoder := json.NewDecoder(strings.NewReader(string(raw)))
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&document); err != nil {
		return Config{}, fmt.Errorf("config.json: %w", err)
	}
	if document.Schema != ConfigSchema {
		return Config{}, fmt.Errorf("config.json schema %q is not %q", document.Schema, ConfigSchema)
	}
	if document.ModelID != modelID {
		return Config{}, fmt.Errorf("config.json is for model %q, the directory is %q",
			document.ModelID, modelID)
	}
	config := Config{Present: true, PlanningJoints: map[string]PlanningRate{},
		Held: map[string]float64{}, ResetPose: map[string]float64{}}
	if document.Caps != nil {
		if document.Caps.XYZMetres < 0 || document.Caps.RPYRadians < 0 {
			return Config{}, fmt.Errorf("config.json: kinematic_correction_caps must not be negative")
		}
		config.Caps = *document.Caps
	}
	fastest, anyRated := urdf.FastestRevolute()
	if document.Planning != nil {
		config.SpeedCeilingRadS = document.Planning.SpeedCeilingRadS
		if config.SpeedCeilingRadS != 0 && (!anyRated || config.SpeedCeilingRadS > fastest) {
			return Config{}, fmt.Errorf(
				"config.json: planning.speed_ceiling_rad_s %g is above the URDF's fastest joint",
				config.SpeedCeilingRadS)
		}
		for name, rate := range document.Planning.Joints {
			joint, known := urdf.Joints[name]
			if !known || !joint.Revolute() {
				return Config{}, fmt.Errorf(
					"config.json: planning.joints names %q, which the URDF has no revolute joint for", name)
			}
			if !(rate.VelocityRadS > 0) || !(rate.AccelerationRadS2 > 0) {
				return Config{}, fmt.Errorf("config.json: planning.joints.%s must state positive rates", name)
			}
			if joint.HasVelocity && rate.VelocityRadS > joint.Velocity {
				return Config{}, fmt.Errorf(
					"config.json: planning.joints.%s.velocity_rad_s %g is above the URDF's rating %g; "+
						"a description may lower the rate it plans at, never raise it",
					name, rate.VelocityRadS, joint.Velocity)
			}
			config.PlanningJoints[name] = rate
		}
	}
	if document.Axes != nil {
		config.Driven = append([]string(nil), document.Axes.Driven...)
		for name, value := range document.Axes.Held {
			if !math.IsInf(value, 0) && !math.IsNaN(value) {
				config.Held[name] = value
			} else {
				return Config{}, fmt.Errorf("config.json: axes.held.%s is not a finite number", name)
			}
		}
		for _, name := range append(append([]string(nil), config.Driven...), keysOf(config.Held)...) {
			if joint, known := urdf.Joints[name]; !known || !joint.Revolute() {
				return Config{}, fmt.Errorf(
					"config.json: axes names %q, which the URDF has no revolute joint for", name)
			}
		}
	}
	for name, value := range document.ResetPose {
		if joint, known := urdf.Joints[name]; !known || !joint.Revolute() {
			return Config{}, fmt.Errorf(
				"config.json: reset_pose names %q, which the URDF has no revolute joint for", name)
		}
		config.ResetPose[name] = value
	}
	if document.Frames != nil {
		config.Frames = *document.Frames
	}
	if document.Torch != nil {
		config.ToolFrame = document.Torch.ToolFrame
		config.ElectrodeAxis = append([]float64(nil), document.Torch.ElectrodeAxis...)
	}
	return config, nil
}

func keysOf(values map[string]float64) []string {
	out := make([]string, 0, len(values))
	for name := range values {
		out = append(out, name)
	}
	return out
}

// A Robot is one model as a catalogue entry: what it is called, what it
// hashes to, and the few facts a client needs before it can author against
// it. The work frame's position is computed from the URDF at the zero pose
// rather than restated, because a number restated in a browser is a number
// that drifts from the robot.
type Robot struct {
	ModelID                string `json:"model_id"`
	RobotDescriptionSHA256 string `json:"robot_description_sha256"`
	// URDFPackage is the package the URDF's meshes are named under
	// (package://<name>/meshes/...), which can differ from the model id: a
	// viewer needs it to find the meshes and to know the model's family.
	URDFPackage string               `json:"urdf_package,omitempty"`
	Frames      catalogueFrames      `json:"frames"`
	Axes        catalogueAxes        `json:"axes"`
	ResetPose   map[string]float64   `json:"reset_pose,omitempty"`
	Settings    *DescriptionSettings `json:"settings,omitempty"`
}

type catalogueFrames struct {
	Base       string     `json:"base"`
	Tool       string     `json:"tool"`
	Work       string     `json:"work"`
	WorkWorldM [3]float64 `json:"work_world_m"`
	// Where the arm starts: the first driven joint's child link at the zero
	// pose. Motion Server's TCP contract is expressed in that frame while a
	// viewer is expressed in the world, so a client converts between them --
	// and the offset is the robot's, not the client's, to state.
	ArmBaseWorldM [3]float64 `json:"arm_base_world_m"`
}

type catalogueAxes struct {
	Driven []string           `json:"driven"`
	Held   map[string]float64 `json:"held,omitempty"`
}

// Catalogue is every model in the store, as a client sees it. A model that
// cannot be read is left out with its reason returned beside the rest: one
// broken description is not an empty catalogue.
func (store Store) Catalogue() ([]Robot, []string) {
	names, err := store.List()
	if err != nil {
		return nil, []string{err.Error()}
	}
	robots := make([]Robot, 0, len(names))
	problems := []string{}
	for _, name := range names {
		description, err := store.Load(name)
		if err != nil {
			problems = append(problems, err.Error())
			continue
		}
		// A robot with nothing to plan against -- a bench of motors -- is not
		// a robot a program can be authored for, and is not offered as one.
		if !description.HasGeometry() {
			continue
		}
		robots = append(robots, description.Robot())
	}
	return robots, problems
}

// Robot is this description as a catalogue entry.
func (d Description) Robot() Robot {
	frames := catalogueFrames{
		Base: d.Config.Frames.Base, Tool: d.Config.Frames.Tool, Work: d.Config.Frames.Work,
	}
	// The work frame may be a point on a link rather than a link: then the
	// position is that link's, plus the stated offset in the LINK's axes --
	// rotated into the base frame, the way the planner places it as a fixed
	// child of that link. Adding the offset unrotated agreed with the planner
	// only while every joint on the chain had a zero rpy, which is a fact
	// about today's two robots and not a rule.
	if surface := d.Config.Frames.WorkSurface; surface != nil {
		if at, rotation, found := d.URDF.LinkPose(frames.Base, surface.Parent); found {
			for row := 0; row < 3; row++ {
				frames.WorkWorldM[row] = at[row]
				for column := 0; column < 3; column++ {
					frames.WorkWorldM[row] += rotation[row][column] * surface.XYZMetres[column]
				}
			}
		}
	} else if at, found := d.URDF.LinkOrigin(frames.Base, frames.Work); found {
		frames.WorkWorldM = at
	}
	if len(d.Config.Driven) > 0 {
		if joint, known := d.URDF.Joints[d.Config.Driven[0]]; known {
			if at, found := d.URDF.LinkOrigin(frames.Base, joint.Child); found {
				frames.ArmBaseWorldM = at
			}
		}
	}
	return Robot{
		ModelID:                d.URDF.RobotName,
		RobotDescriptionSHA256: d.Identity,
		URDFPackage:            urdfMeshPackage(d.URDFRaw),
		Frames:                 frames,
		Axes:                   catalogueAxes{Driven: d.Config.Driven, Held: d.Config.Held},
		ResetPose:              d.Config.ResetPose,
		Settings:               d.settingsSnapshot(),
	}
}

// urdfMeshPackage is the package of the first description-local mesh URI in a
// URDF (MeshResourcePath's rule), "" when it names none.
func urdfMeshPackage(raw []byte) string {
	for _, match := range meshPackageURI.FindAllSubmatch(raw, -1) {
		if _, ok := MeshResourcePath(string(match[0])); ok {
			return string(match[1])
		}
	}
	return ""
}

var meshPackageURI = regexp.MustCompile(`package://([A-Za-z0-9_]+)/[^"'<>\s]+`)
