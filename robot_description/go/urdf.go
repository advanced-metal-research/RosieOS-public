package robotdesc

// The URDF, read only as far as the cell check needs: the robot's name and
// each top-level joint's type and limits. The twin of `parse_urdf` in the
// daemon's motion-server/joint-trajectory/v1/src/robot_config.hpp, rule for
// rule, because the two must refuse the same machine_planning_calibration.json -- one on the cell and
// one here, before a plan is ever made.
//
// Not a general XML parser. It refuses what it cannot tokenize rather than
// guessing: a URDF this cannot read is a description whose limits are unknown,
// and planning against unknown limits is the failure the contract exists to
// prevent.

import (
	"fmt"
	"math"
	"strconv"
	"strings"
	"unicode"
)

// URDFJoint is one top-level <joint> as the cell check reads it.
type URDFJoint struct {
	Type   string
	Parent string
	Child  string
	// The joint's fixed origin in its parent: what the chain is made of. The
	// zero pose is every joint at zero, so walking these alone places any
	// link -- which is how the work frame's position is known without a
	// kinematics library.
	XYZMetres   [3]float64
	RPYRadians  [3]float64
	HasLimit    bool
	HasLower    bool
	HasUpper    bool
	HasVelocity bool
	Lower       float64
	Upper       float64
	Velocity    float64
}

// Revolute reports whether the planner may move this joint at all.
func (j URDFJoint) Revolute() bool { return j.Type == "revolute" }

// URDF is the summary: the robot's name and its joints by name.
type URDF struct {
	RobotName string
	Joints    map[string]URDFJoint
}

// FastestRevolute is the highest velocity any revolute joint is rated for, and
// whether any states one. A speed ceiling is checked against it.
func (u URDF) FastestRevolute() (float64, bool) {
	fastest, any := 0.0, false
	for _, joint := range u.Joints {
		if joint.Revolute() && joint.HasVelocity {
			fastest = math.Max(fastest, joint.Velocity)
			any = true
		}
	}
	return fastest, any
}

var xmlEntities = strings.NewReplacer(
	"&lt;", "<", "&gt;", ">", "&quot;", `"`, "&apos;", "'", "&amp;", "&")

// ParseURDF reads a URDF's joints. Everything else in the document is skipped,
// including the links and the meshes: this is not a model loader.
func ParseURDF(text string) (URDF, error) {
	out := URDF{Joints: map[string]URDFJoint{}}
	var stack []string
	current := "" // a top-level <joint> being read
	for i := 0; ; {
		open := strings.IndexByte(text[i:], '<')
		if open < 0 {
			break
		}
		open += i
		if strings.HasPrefix(text[open:], "<!--") {
			close := strings.Index(text[open+4:], "-->")
			if close < 0 {
				return URDF{}, fmt.Errorf("robot.urdf: unterminated comment")
			}
			i = open + 4 + close + 3
			continue
		}
		if strings.HasPrefix(text[open:], "<?") || strings.HasPrefix(text[open:], "<!") {
			close := strings.IndexByte(text[open:], '>')
			if close < 0 {
				return URDF{}, fmt.Errorf("robot.urdf: unterminated declaration")
			}
			i = open + close + 1
			continue
		}
		name, attributes, selfClosing, closing, next, err := ParseTag(text, open)
		if err != nil {
			return URDF{}, err
		}
		i = next

		if closing {
			if len(stack) == 0 || stack[len(stack)-1] != name {
				return URDF{}, fmt.Errorf("robot.urdf: </%s> does not close an open tag", name)
			}
			stack = stack[:len(stack)-1]
			if name == "joint" && len(stack) == 1 {
				current = ""
			}
			continue
		}
		switch depth := len(stack); {
		case depth == 0:
			if name != "robot" {
				return URDF{}, fmt.Errorf("robot.urdf: root is <%s>, expected <robot>", name)
			}
			out.RobotName = attributes["name"]
		case depth == 1 && name == "joint":
			jointName := attributes["name"]
			if jointName == "" {
				return URDF{}, fmt.Errorf("robot.urdf: a <joint> has no name")
			}
			if _, seen := out.Joints[jointName]; seen {
				return URDF{}, fmt.Errorf("robot.urdf: joint %s is defined twice", jointName)
			}
			out.Joints[jointName] = URDFJoint{Type: attributes["type"]}
			if !selfClosing {
				current = jointName
			}
		case depth == 2 && name == "parent" && current != "":
			joint := out.Joints[current]
			joint.Parent = attributes["link"]
			out.Joints[current] = joint
		case depth == 2 && name == "child" && current != "":
			joint := out.Joints[current]
			joint.Child = attributes["link"]
			out.Joints[current] = joint
		case depth == 2 && name == "origin" && current != "":
			joint := out.Joints[current]
			for key, target := range map[string]*[3]float64{
				"xyz": &joint.XYZMetres, "rpy": &joint.RPYRadians,
			} {
				raw, present := attributes[key]
				if !present {
					continue
				}
				values, err := parseTriple(raw)
				if err != nil {
					return URDF{}, fmt.Errorf("robot.urdf: joint %s has a bad origin %s: %w", current, key, err)
				}
				*target = values
			}
			out.Joints[current] = joint
		case depth == 2 && name == "limit" && current != "":
			joint := out.Joints[current]
			joint.HasLimit = true
			for key, target := range map[string]*struct {
				has   *bool
				value *float64
			}{
				"lower":    {&joint.HasLower, &joint.Lower},
				"upper":    {&joint.HasUpper, &joint.Upper},
				"velocity": {&joint.HasVelocity, &joint.Velocity},
			} {
				raw, present := attributes[key]
				if !present {
					continue
				}
				value, err := strconv.ParseFloat(strings.TrimSpace(raw), 64)
				if err != nil || math.IsInf(value, 0) || math.IsNaN(value) {
					return URDF{}, fmt.Errorf("robot.urdf: joint %s has a non-numeric limit", current)
				}
				*target.has = true
				*target.value = value
			}
			out.Joints[current] = joint
		}
		if !selfClosing {
			stack = append(stack, name)
		}
	}
	if len(stack) != 0 {
		return URDF{}, fmt.Errorf("robot.urdf: <%s> is never closed", stack[len(stack)-1])
	}
	if out.RobotName == "" {
		return URDF{}, fmt.Errorf("robot.urdf: <robot> has no name")
	}
	return out, nil
}

// parseTag reads one element at `open`, returning its name, attributes and
// where the document continues.
// ParseTag reads one element at `open`: its name, attributes, whether it
// self-closes or closes, and where the document continues. Exported for the
// OLP's context compiler, which rewrites attributes in place with it.
func ParseTag(text string, open int) (
	name string, attributes map[string]string, selfClosing, closing bool, next int, err error,
) {
	p := open + 1
	if p < len(text) && text[p] == '/' {
		closing = true
		p++
	}
	start := p
	for p < len(text) && !unicode.IsSpace(rune(text[p])) && text[p] != '>' && text[p] != '/' {
		p++
	}
	name = text[start:p]
	if name == "" {
		return "", nil, false, false, 0, fmt.Errorf("robot.urdf: malformed tag at byte %d", open)
	}
	attributes = map[string]string{}
	for {
		for p < len(text) && unicode.IsSpace(rune(text[p])) {
			p++
		}
		if p >= len(text) {
			return "", nil, false, false, 0, fmt.Errorf("robot.urdf: unterminated tag <%s>", name)
		}
		if text[p] == '>' {
			p++
			break
		}
		if text[p] == '/' && p+1 < len(text) && text[p+1] == '>' {
			selfClosing = true
			p += 2
			break
		}
		keyStart := p
		for p < len(text) && text[p] != '=' && !unicode.IsSpace(rune(text[p])) {
			p++
		}
		key := text[keyStart:p]
		for p < len(text) && unicode.IsSpace(rune(text[p])) {
			p++
		}
		if p >= len(text) || text[p] != '=' {
			return "", nil, false, false, 0,
				fmt.Errorf("robot.urdf: attribute %s of <%s> has no value", key, name)
		}
		p++
		for p < len(text) && unicode.IsSpace(rune(text[p])) {
			p++
		}
		if p >= len(text) || (text[p] != '"' && text[p] != '\'') {
			return "", nil, false, false, 0,
				fmt.Errorf("robot.urdf: attribute %s of <%s> is not quoted", key, name)
		}
		quote := text[p]
		p++
		end := strings.IndexByte(text[p:], quote)
		if end < 0 {
			return "", nil, false, false, 0,
				fmt.Errorf("robot.urdf: unterminated attribute %s", key)
		}
		attributes[key] = xmlEntities.Replace(text[p : p+end])
		p += end + 1
	}
	return name, attributes, selfClosing, closing, p, nil
}

// parseTriple reads `"x y z"`, the only shape a URDF origin comes in.
func parseTriple(raw string) ([3]float64, error) {
	fields := strings.Fields(raw)
	if len(fields) != 3 {
		return [3]float64{}, fmt.Errorf("%q is not three numbers", raw)
	}
	var out [3]float64
	for i, field := range fields {
		value, err := strconv.ParseFloat(field, 64)
		if err != nil || math.IsInf(value, 0) || math.IsNaN(value) {
			return [3]float64{}, fmt.Errorf("%q is not three numbers", raw)
		}
		out[i] = value
	}
	return out, nil
}

// LinkOrigin is where `link` sits in `base` with every joint at zero, walking
// the fixed origins up the tree. The zero pose is what the work frame is
// stated at, and it is the only pose this needs.
func (u URDF) LinkOrigin(base, link string) ([3]float64, bool) {
	position, _, found := u.LinkPose(base, link)
	return position, found
}

// LinkPose is LinkOrigin with the link's orientation in `base` beside the
// position: the rotation an offset stated in the link's own axes must be
// turned by before it is added in the base frame.
func (u URDF) LinkPose(base, link string) ([3]float64, [3][3]float64, bool) {
	rotation := [3][3]float64{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
	// From the link back to the base, then compose forwards.
	chain := []URDFJoint{}
	for at := link; at != base; {
		parent, found := URDFJoint{}, false
		for _, joint := range u.Joints {
			if joint.Child == at {
				parent, found = joint, true
				break
			}
		}
		if !found || len(chain) > len(u.Joints) {
			return [3]float64{}, rotation, false
		}
		chain = append(chain, parent)
		at = parent.Parent
	}
	position := [3]float64{}
	for i := len(chain) - 1; i >= 0; i-- {
		joint := chain[i]
		for row := 0; row < 3; row++ {
			for column := 0; column < 3; column++ {
				position[row] += rotation[row][column] * joint.XYZMetres[column]
			}
		}
		rotation = Multiply(rotation, RPYMatrix(joint.RPYRadians))
	}
	return position, rotation, true
}

// RPYMatrix is the URDF convention: fixed-axis roll-pitch-yaw, Rz(yaw) Ry(pitch) Rx(roll).
func RPYMatrix(rpy [3]float64) [3][3]float64 {
	cr, sr := math.Cos(rpy[0]), math.Sin(rpy[0])
	cp, sp := math.Cos(rpy[1]), math.Sin(rpy[1])
	cy, sy := math.Cos(rpy[2]), math.Sin(rpy[2])
	return [3][3]float64{
		{cy * cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr},
		{sy * cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr},
		{-sp, cp * sr, cp * cr},
	}
}

// Multiply is the 3x3 product a*b.
func Multiply(a, b [3][3]float64) [3][3]float64 {
	var out [3][3]float64
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			for k := 0; k < 3; k++ {
				out[i][j] += a[i][k] * b[k][j]
			}
		}
	}
	return out
}

// ChainJoints is every revolute joint on the path from `base` down to
// `link`, walking child->parent, in root-to-leaf order. Empty when the two
// are not connected or either is unnamed.
func (u URDF) ChainJoints(base, link string) []string {
	if base == "" || link == "" {
		return nil
	}
	var chain []string
	for at := link; at != base; {
		parent, found := URDFJoint{}, false
		for name, joint := range u.Joints {
			if joint.Child == at {
				parent, found = joint, true
				if joint.Revolute() {
					chain = append(chain, name)
				}
				break
			}
		}
		if !found || len(chain) > len(u.Joints) {
			return nil
		}
		at = parent.Parent
	}
	for i, j := 0, len(chain)-1; i < j; i, j = i+1, j-1 {
		chain[i], chain[j] = chain[j], chain[i]
	}
	return chain
}
