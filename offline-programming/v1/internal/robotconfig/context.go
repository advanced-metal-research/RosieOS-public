package robotconfig

// The planning context: the robot a plan is made against, compiled once.
//
// A plan is made against a description and a cell's deviation from it, and
// deriving "the corrected robot" from those two is one act. It used to
// happen twice on the planner's side -- once onto the cell descriptor, once
// onto a copy of the URDF for cuRobo, the second found by globbing a
// directory for a name -- and the two could disagree without anyone noticing
// (docs/PLANNING-CONTEXT.md). This is the one derivation. The planner and the
// seam worker read what it produces and never a `config.json`, a `machine_planning_calibration.json`
// or a description directory.
//
// The context is EPHEMERAL: compiled right before a plan request, packed into
// it, and discarded. Nothing stores it, hashes it or pins it. Its identity is
// the pair it was compiled from, which the header carries.

import (
	"crypto/sha256"
	"encoding/base64"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"

	"rosieos/robotdesc"
)

// ContextSchema is the schema of `planner.json`, the compiled numbers.
const ContextSchema = "rosie.planning-context.v1"

// The members every context carries. Everything else is the description's
// registered files, verbatim, minus the two this replaces.
const (
	ContextURDF     = "robot.urdf"
	ContextPlanner  = "planner.json"
	ContextIdentity = "identity.json"
)

// Context is one compiled planning context: the corrected URDF, the numbers
// beside it, and the description's other registered files.
type Context struct {
	Identity Identity
	// Files by context-relative path: `robot.urdf`, `planner.json`,
	// `identity.json`, and the description's spheres, meshes and SRDF as
	// registered.
	Files map[string][]byte
}

// PlannerDocument is `planner.json`: `config.json` merged with the cell's
// settled settings, in the shape the planner's profile takes. Every number is
// final; a reader applies nothing.
type PlannerDocument struct {
	Schema                string   `json:"schema"`
	ModelID               string   `json:"model_id"`
	Identity              Identity `json:"identity"`
	ExecutionLimitsSHA256 string   `json:"execution_limits_sha256,omitempty"`
	// Bounds is every revolute joint's position limit as this cell enforces
	// it -- the URDF's, narrowed where the cell narrowed it. The same numbers
	// are on `robot.urdf`'s <limit> elements.
	Bounds map[string]JointBounds `json:"bounds"`
	// Limits is the rate policy per driven axis, after the cell's own rate and
	// ceiling: the planning rate never above the enforced rate, the enforced
	// rate the cell's own or the planning rate when the cell states none.
	Limits           map[string]PlannerRate `json:"limits"`
	SpeedCeilingRadS float64                `json:"speed_ceiling_rad_s,omitempty"`
	Driven           []string               `json:"driven"`
	Held             map[string]float64     `json:"held"`
	ResetPose        map[string]float64     `json:"reset_pose"`
	Frames           Frames                 `json:"frames"`
	Torch            *Torch                 `json:"torch,omitempty"`
	// JointCorrections is what was composed into `robot.urdf`'s origins, so a
	// reader can see the cell's deviation without diffing two URDFs.
	JointCorrections map[string]Correction `json:"joint_corrections,omitempty"`
}

// JointBounds is one joint's enforced position limit.
type JointBounds struct {
	LowerRad float64 `json:"lower_rad"`
	UpperRad float64 `json:"upper_rad"`
}

// PlannerRate is one axis's rates as the planner reads them
// (`weld_motion_planner.robot_cell.robot.profile.CellProfile.limits`).
type PlannerRate struct {
	VelocityRadS         float64 `json:"velocity_rad_s"`
	EnforcedVelocityRadS float64 `json:"enforced_velocity_rad_s"`
	AccelerationRadS2    float64 `json:"acceleration_rad_s2"`
}

// Torch is the description's torch block.
type Torch struct {
	ToolFrame     string    `json:"tool_frame"`
	ElectrodeAxis []float64 `json:"electrode_axis"`
}

// identityDocument is `identity.json`: the pair the context was compiled
// from, plus the hash of the base URDF before the cell was applied, for a
// reader that wants to check a sphere model's provenance against the robot
// it was fitted to rather than the corrected copy.
type identityDocument struct {
	Identity
	BaseURDFSHA256        string `json:"base_urdf_sha256"`
	ExecutionLimitsSHA256 string `json:"execution_limits_sha256,omitempty"`
}

// Compile derives the planning context for `description` deviated by
// `settings`. The settings are already judged (`Resolve`); this composes and
// packs. A supplied native execution snapshot further narrows the compiled
// bounds/rates and binds the packed context to that envelope. It refuses a description
// without a `config.json` has no rates, no axis roles and no reset pose, so
// there is nothing to plan against.
func Compile(description Description, settings Settings, execution ...ExecutionLimits) (Context, error) {
	if !description.HasGeometry() {
		return Context{}, fmt.Errorf(
			"cell %q has no robot.urdf; a bench of motors is a robot rt-core runs and nothing a plan can be made for",
			description.ModelID)
	}
	if !description.Config.Present {
		return Context{}, fmt.Errorf(
			"cell %q has no config.json; without axis roles, rates and a reset pose there is nothing to plan against",
			description.ModelID)
	}
	if settings.Identity.ModelID != description.ModelID {
		return Context{}, fmt.Errorf("settings are for model %q, the description is %q",
			settings.Identity.ModelID, description.ModelID)
	}
	planner, err := compilePlanner(description, settings)
	if err != nil {
		return Context{}, err
	}
	if len(execution) > 1 {
		return Context{}, fmt.Errorf("planning context needs one execution-limit snapshot")
	}
	if len(execution) == 1 && execution[0] != nil {
		if err := applyExecutionLimits(&planner, execution[0]); err != nil {
			return Context{}, err
		}
	}
	urdf, err := rewriteURDF(string(description.URDFRaw), description.URDF, settings, planner)
	if err != nil {
		return Context{}, err
	}
	registered, err := DescriptionFiles(description.Dir)
	if err != nil {
		return Context{}, err
	}
	files := map[string][]byte{}
	for _, path := range registered {
		// The URDF is replaced by its corrected copy and config.json by
		// planner.json: a context that also carried the originals would
		// invite a reader to read them, which is the habit this ends.
		if path == "robot.urdf" || path == "config.json" {
			continue
		}
		data, err := os.ReadFile(filepath.Join(description.Dir, filepath.FromSlash(path)))
		if err != nil {
			return Context{}, fmt.Errorf("description %s: %w", path, err)
		}
		files[path] = data
	}
	files[ContextURDF] = []byte(urdf)
	files[ContextPlanner] = canonicalJSON(planner)
	base := sha256.Sum256(description.URDFRaw)
	files[ContextIdentity] = canonicalJSON(identityDocument{
		Identity: settings.Identity, BaseURDFSHA256: "sha256:" + hex.EncodeToString(base[:]),
		ExecutionLimitsSHA256: planner.ExecutionLimitsSHA256,
	})
	return Context{Identity: settings.Identity, Files: files}, nil
}

// Wire is the context as the seam worker is handed it: every member as
// base64, so it travels inside the JSON request the worker already takes.
func (c Context) Wire() map[string]any {
	files := make(map[string]string, len(c.Files))
	for path, data := range c.Files {
		files[path] = base64.StdEncoding.EncodeToString(data)
	}
	return map[string]any{"identity": c.Identity, "files": files}
}

// compilePlanner merges config.json with the settled settings the way the
// planner's `profile.for_cell` did, so the numbers it reads are the ones it
// read before -- now written once, here, instead of derived on every plan.
func compilePlanner(description Description, settings Settings) (PlannerDocument, error) {
	config := description.Config
	urdf := description.URDF
	out := PlannerDocument{
		Schema:           ContextSchema,
		ModelID:          description.ModelID,
		Identity:         settings.Identity,
		Bounds:           map[string]JointBounds{},
		Limits:           map[string]PlannerRate{},
		SpeedCeilingRadS: settings.SpeedCeilingRadS,
		Held:             map[string]float64{},
		ResetPose:        map[string]float64{},
		Frames:           config.Frames,
		JointCorrections: settings.JointCorrections,
	}
	for name, joint := range urdf.Joints {
		if !joint.Revolute() || !joint.HasLower || !joint.HasUpper {
			continue
		}
		bounds := JointBounds{LowerRad: joint.Lower, UpperRad: joint.Upper}
		if narrowed, ok := settings.JointLimits[name]; ok {
			bounds = JointBounds{LowerRad: narrowed.LowerRad, UpperRad: narrowed.UpperRad}
		}
		out.Bounds[name] = bounds
	}
	// The model's own ceiling caps its planning rates (the Python twin,
	// `weldplan.robot_description.parse_config`, does the same); the cell's
	// ceiling then caps the enforced rate, and the planning rate follows it
	// down. A rate only ever goes down.
	for axis, rate := range config.PlanningJoints {
		planning := rate.VelocityRadS
		if config.SpeedCeilingRadS > 0 {
			planning = math.Min(planning, config.SpeedCeilingRadS)
		}
		enforced := planning
		if narrowed, ok := settings.JointLimits[axis]; ok && narrowed.VelocityRad != nil {
			enforced = *narrowed.VelocityRad
		}
		if settings.SpeedCeilingRadS > 0 {
			enforced = math.Min(enforced, settings.SpeedCeilingRadS)
		}
		out.Limits[axis] = PlannerRate{
			VelocityRadS:         math.Min(planning, enforced),
			EnforcedVelocityRadS: enforced,
			AccelerationRadS2:    rate.AccelerationRadS2,
		}
	}
	for name, value := range config.Held {
		out.Held[name] = value
	}
	// A hold from the cell may only name a positioner joint: the arm solver
	// has six unknowns and a held arm joint is not a cell it can plan. The
	// arm is the chain from the base frame to the tool frame.
	arm := map[string]bool{}
	for _, name := range urdf.ChainJoints(config.Frames.Base, config.Frames.Tool) {
		arm[name] = true
	}
	for name, value := range settings.Held {
		if arm[name] {
			return PlannerDocument{}, fmt.Errorf(
				"the cell holds %q, which is not a positioner joint of %q; only the positioner may be held",
				name, description.ModelID)
		}
		out.Held[name] = value
	}
	for _, name := range config.Driven {
		if _, held := out.Held[name]; !held {
			out.Driven = append(out.Driven, name)
		}
	}
	if out.Driven == nil {
		out.Driven = []string{}
	}
	for name, value := range config.ResetPose {
		out.ResetPose[name] = value
	}
	if config.ToolFrame != "" || len(config.ElectrodeAxis) > 0 {
		out.Torch = &Torch{ToolFrame: config.ToolFrame,
			ElectrodeAxis: append([]float64(nil), config.ElectrodeAxis...)}
	}
	return out, nil
}

// rewriteURDF is the base URDF with the cell applied, by ATTRIBUTE
// SUBSTITUTION: a joint the cell corrected gets its composed origin, a joint
// the cell narrowed gets the narrowed bounds, a driven axis gets the planning
// rate as its velocity, and every other byte -- comments, formatting, the
// joints the cell did not touch -- is preserved. Not an XML round trip: the
// planner's parser should see the file the description's author wrote.
//
// The velocity written is the PLANNING rate, not the enforced one: cuRobo
// parametrizes against the URDF's <limit velocity>, and what the planner may
// plan at is the planning rate. The gap up to the enforced rate is the joint
// daemon's follower authority and belongs in the header, not here.
func rewriteURDF(text string, urdf URDF, settings Settings, planner PlannerDocument) (string, error) {
	var out strings.Builder
	var stack []string
	current := ""       // the top-level <joint> being read
	originSeen := false // whether `current` has had its <origin>
	copied := 0         // text[:copied] is already in `out`

	flushTo := func(at int) {
		out.WriteString(text[copied:at])
		copied = at
	}
	// A joint that needs a correction but states no <origin> gets one
	// inserted right before its closing tag; its URDF origin is the identity.
	// Before the CLOSING tag and not after the opening one: by the time the
	// joint closes, its <limit> may already have been rewritten and copied,
	// and writing at an earlier offset would slice text backwards -- a panic
	// in the request handler for a joint shape no real URDF has, but a
	// dropped plan request is not the answer to one.
	closeJoint := func(closingAt int) {
		if current != "" && !originSeen {
			if correction, ok := settings.JointCorrections[current]; ok {
				flushTo(closingAt)
				xyz, rpy := composeOrigin([3]float64{}, [3]float64{}, correction.XYZMetres, correction.RPYRadians)
				out.WriteString(`<origin xyz="` + triple(xyz) + `" rpy="` + triple(rpy) + `"/>`)
			}
		}
		current, originSeen = "", false
	}

	for i := 0; ; {
		open := strings.IndexByte(text[i:], '<')
		if open < 0 {
			break
		}
		open += i
		if strings.HasPrefix(text[open:], "<!--") {
			close := strings.Index(text[open+4:], "-->")
			if close < 0 {
				return "", fmt.Errorf("robot.urdf: unterminated comment")
			}
			i = open + 4 + close + 3
			continue
		}
		if strings.HasPrefix(text[open:], "<?") || strings.HasPrefix(text[open:], "<!") {
			close := strings.IndexByte(text[open:], '>')
			if close < 0 {
				return "", fmt.Errorf("robot.urdf: unterminated declaration")
			}
			i = open + close + 1
			continue
		}
		name, attributes, selfClosing, closing, next, err := robotdesc.ParseTag(text, open)
		if err != nil {
			return "", err
		}
		i = next
		if closing {
			if len(stack) == 0 || stack[len(stack)-1] != name {
				return "", fmt.Errorf("robot.urdf: </%s> does not close an open tag", name)
			}
			stack = stack[:len(stack)-1]
			if name == "joint" && len(stack) == 1 {
				closeJoint(open)
			}
			continue
		}
		switch depth := len(stack); {
		case depth == 1 && name == "joint":
			if selfClosing {
				break
			}
			current, originSeen = attributes["name"], false
		case depth == 2 && name == "origin" && current != "":
			originSeen = true
			correction, ok := settings.JointCorrections[current]
			if !ok {
				break
			}
			joint := urdf.Joints[current]
			xyz, rpy := composeOrigin(joint.XYZMetres, joint.RPYRadians,
				correction.XYZMetres, correction.RPYRadians)
			tag := text[open:next]
			tag = setAttribute(tag, "xyz", triple(xyz))
			tag = setAttribute(tag, "rpy", triple(rpy))
			flushTo(open)
			out.WriteString(tag)
			copied = next
		case depth == 2 && name == "limit" && current != "":
			joint := urdf.Joints[current]
			tag := text[open:next]
			changed := false
			if bounds, ok := planner.Bounds[current]; ok {
				// Only what differs, so a joint the cell narrowed on one side
				// keeps the author's text on the other.
				if bounds.LowerRad != joint.Lower {
					tag, changed = setAttribute(tag, "lower", number(bounds.LowerRad)), true
				}
				if bounds.UpperRad != joint.Upper {
					tag, changed = setAttribute(tag, "upper", number(bounds.UpperRad)), true
				}
			}
			if rate, ok := planner.Limits[current]; ok && joint.HasVelocity && rate.VelocityRadS != joint.Velocity {
				tag, changed = setAttribute(tag, "velocity", number(rate.VelocityRadS)), true
			}
			if changed {
				flushTo(open)
				out.WriteString(tag)
				copied = next
			}
		}
		if !selfClosing {
			stack = append(stack, name)
		}
	}
	if len(stack) != 0 {
		return "", fmt.Errorf("robot.urdf: <%s> is never closed", stack[len(stack)-1])
	}
	flushTo(len(text))
	return out.String(), nil
}

// setAttribute is `tag` with attribute `key` set to `value`: replaced in
// place when present, appended before the closing bracket when not. The rest
// of the tag's bytes are untouched.
func setAttribute(tag, key, value string) string {
	for _, quote := range []string{`"`, `'`} {
		for _, lead := range []string{" ", "\t", "\n"} {
			marker := lead + key + "=" + quote
			at := strings.Index(tag, marker)
			if at < 0 {
				continue
			}
			start := at + len(marker)
			end := strings.Index(tag[start:], quote)
			if end < 0 {
				continue
			}
			return tag[:start] + value + tag[start+end:]
		}
	}
	insert := ` ` + key + `="` + value + `"`
	if strings.HasSuffix(tag, "/>") {
		return strings.TrimSuffix(tag, "/>") + insert + "/>"
	}
	return strings.TrimSuffix(tag, ">") + insert + ">"
}

// composeOrigin is `T_origin * Trans(dxyz) * R(drpy)` back as a URDF origin,
// the same arithmetic as `weldplan.robot_description.compose_origin`.
func composeOrigin(xyz, rpy, dxyz, drpy [3]float64) ([3]float64, [3]float64) {
	rotation := robotdesc.RPYMatrix(rpy)
	var moved [3]float64
	for i := 0; i < 3; i++ {
		moved[i] = xyz[i]
		for k := 0; k < 3; k++ {
			moved[i] += rotation[i][k] * dxyz[k]
		}
	}
	return moved, matrixRPY(robotdesc.Multiply(rotation, robotdesc.RPYMatrix(drpy)))
}

// matrixRPY is the inverse of rpyMatrix, pitch in [-pi/2, pi/2]. At gimbal
// lock roll and yaw share an axis and it all goes into yaw, as the Python twin
// does.
func matrixRPY(m [3][3]float64) [3]float64 {
	pitch := math.Asin(math.Max(-1, math.Min(1, -m[2][0])))
	if math.Abs(math.Cos(pitch)) > 1e-9 {
		return [3]float64{math.Atan2(m[2][1], m[2][2]), pitch, math.Atan2(m[1][0], m[0][0])}
	}
	return [3]float64{0, pitch, math.Atan2(-m[0][1], m[1][1])}
}

func number(value float64) string { return strconv.FormatFloat(value, 'f', -1, 64) }

func triple(values [3]float64) string {
	return number(values[0]) + " " + number(values[1]) + " " + number(values[2])
}

// canonicalJSON is indented JSON with a trailing newline: struct fields in
// declaration order, map keys sorted (encoding/json's own rule). Not an
// identity -- nothing hashes a context member -- but the same bytes every
// run, so two contexts compiled from the same pair can be diffed.
func canonicalJSON(value any) []byte {
	out, err := json.MarshalIndent(value, "", "  ")
	if err != nil {
		panic(err) // every caller passes a struct of its own
	}
	return append(out, '\n')
}
