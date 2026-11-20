// Package cartesian is the TCP geometry the offline tools measure with: the
// resolver's forward kinematics in Go and the deviation of a pose from a frozen
// Cartesian line. tools/tuningreport (Stop dumps) and the OLP's
// cartesianjog-harness (resolver + native generator replay) both use it, so a
// hold measured on the robot and the same hold replayed offline are judged by
// one definition. Pure computation: it builds on every platform and talks to
// no controller.
package cartesian

import (
	"encoding/json"
	"encoding/xml"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"
)

type mat3 [3][3]float64
type vec3 [3]float64

// Kinematics is the resolver's forward kinematics (motion-server/v1
// urdf_forward_kinematics_j1_base, the fk operation of robot-v4-cartesiand
// --resolve-only): the chain from the first arm joint with that joint's own
// origin dropped, revolute joints about their URDF axes, then the fixed joints
// down to the tool frame. The base frame is therefore the world-aligned J1
// base. TestFKMatchesResolver pins it to the compiled resolver. Asking the
// resolver itself was measured on the Mac on 2026-10-07: one persistent
// --resolve-only process starts in 0.36 s and answers 12k fk requests/s one at
// a time, 30k/s pipelined, so the 105,225 poses of the 2026-10-07 15:33
// full-dump report would take 3.6-8.7 s; the whole report takes 2.1 s with
// this port.
type Kinematics struct {
	origR []mat3
	origP []vec3
	axis  []vec3
	toolR mat3
	toolP vec3
	// Joints are the URDF joint names of the chain, in q order; ToolFrame is
	// the link the TCP is.
	Joints    []string
	ToolFrame string
}

type urdfDoc struct {
	Joints []struct {
		Name   string `xml:"name,attr"`
		Type   string `xml:"type,attr"`
		Parent struct {
			Link string `xml:"link,attr"`
		} `xml:"parent"`
		Child struct {
			Link string `xml:"link,attr"`
		} `xml:"child"`
		Origin *struct {
			XYZ string `xml:"xyz,attr"`
			RPY string `xml:"rpy,attr"`
		} `xml:"origin"`
		Axis *struct {
			XYZ string `xml:"xyz,attr"`
		} `xml:"axis"`
	} `xml:"joint"`
}

func parse3(s string, def vec3) (vec3, error) {
	f := strings.Fields(s)
	if len(f) == 0 {
		return def, nil
	}
	if len(f) != 3 {
		return def, fmt.Errorf("expected 3 numbers, got %q", s)
	}
	var v vec3
	for i := range v {
		x, err := strconv.ParseFloat(f[i], 64)
		if err != nil {
			return def, err
		}
		v[i] = x
	}
	return v, nil
}

func rpy(r, p, y float64) mat3 {
	cr, sr, cp, sp, cy, sy := math.Cos(r), math.Sin(r), math.Cos(p), math.Sin(p), math.Cos(y), math.Sin(y)
	return mat3{{cy * cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr},
		{sy * cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr},
		{-sp, cp * sr, cp * cr}}
}

func ident() mat3 { return mat3{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} }

func (a mat3) mul(b mat3) mat3 {
	var c mat3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			c[i][j] = a[i][0]*b[0][j] + a[i][1]*b[1][j] + a[i][2]*b[2][j]
		}
	}
	return c
}

func (a mat3) apply(v vec3) vec3 {
	return vec3{a[0][0]*v[0] + a[0][1]*v[1] + a[0][2]*v[2], a[1][0]*v[0] + a[1][1]*v[1] + a[1][2]*v[2], a[2][0]*v[0] + a[2][1]*v[1] + a[2][2]*v[2]}
}

func (a vec3) add(b vec3) vec3      { return vec3{a[0] + b[0], a[1] + b[1], a[2] + b[2]} }
func (a vec3) scale(s float64) vec3 { return vec3{a[0] * s, a[1] * s, a[2] * s} }
func (a vec3) dot(b vec3) float64   { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] }
func (a vec3) norm() float64        { return math.Sqrt(a.dot(a)) }

func axisRot(ax vec3, q float64) mat3 {
	x, y, z := ax[0], ax[1], ax[2]
	c, s := math.Cos(q), math.Sin(q)
	C := 1 - c
	return mat3{{c + x*x*C, x*y*C - z*s, x*z*C + y*s},
		{y*x*C + z*s, c + y*y*C, y*z*C - x*s},
		{z*x*C - y*s, z*y*C + x*s, c + z*z*C}}
}

// NewKinematics builds the chain through armJoints (URDF names, q order) down
// to toolFrame. A joint that is not revolute, does not follow its predecessor,
// or a tool frame not reached by one fixed chain is refused: a wrong chain
// would measure a different TCP than the resolver commands.
func NewKinematics(urdf []byte, armJoints []string, toolFrame string) (*Kinematics, error) {
	var doc urdfDoc
	if err := xml.Unmarshal(urdf, &doc); err != nil {
		return nil, err
	}
	idx := map[string]int{}
	byParent := map[string][]int{}
	for i, j := range doc.Joints {
		idx[j.Name] = i
		byParent[j.Parent.Link] = append(byParent[j.Parent.Link], i)
	}
	k := &Kinematics{Joints: append([]string(nil), armJoints...), ToolFrame: toolFrame}
	link := ""
	for n, name := range armJoints {
		i, ok := idx[name]
		if !ok {
			return nil, fmt.Errorf("urdf has no joint %s", name)
		}
		j := doc.Joints[i]
		if j.Type != "revolute" && j.Type != "continuous" {
			return nil, fmt.Errorf("joint %s is %s, not revolute", name, j.Type)
		}
		if n > 0 && j.Parent.Link != link {
			return nil, fmt.Errorf("joint %s does not follow %s in the chain", name, armJoints[n-1])
		}
		R, P := ident(), vec3{}
		if n > 0 && j.Origin != nil {
			xyz, err := parse3(j.Origin.XYZ, vec3{})
			if err != nil {
				return nil, err
			}
			r, err := parse3(j.Origin.RPY, vec3{})
			if err != nil {
				return nil, err
			}
			R, P = rpy(r[0], r[1], r[2]), xyz
		}
		ax := vec3{1, 0, 0}
		if j.Axis != nil {
			a, err := parse3(j.Axis.XYZ, vec3{1, 0, 0})
			if err != nil {
				return nil, err
			}
			ax = a.scale(1 / a.norm())
		}
		k.origR, k.origP, k.axis = append(k.origR, R), append(k.origP, P), append(k.axis, ax)
		link = j.Child.Link
	}
	// Fixed joints from the last arm link down to the tool frame.
	k.toolR = ident()
	for link != toolFrame {
		var next []int
		for _, i := range byParent[link] {
			if doc.Joints[i].Type == "fixed" {
				next = append(next, i)
			}
		}
		if len(next) != 1 {
			return nil, fmt.Errorf("no unique fixed chain from %s to tool frame %q", link, toolFrame)
		}
		j := doc.Joints[next[0]]
		xyz, r := vec3{}, vec3{}
		if j.Origin != nil {
			xyz, _ = parse3(j.Origin.XYZ, vec3{})
			r, _ = parse3(j.Origin.RPY, vec3{})
		}
		k.toolP = k.toolP.add(k.toolR.apply(xyz))
		k.toolR = k.toolR.mul(rpy(r[0], r[1], r[2]))
		link = j.Child.Link
	}
	return k, nil
}

// LoadKinematics reads a robot description directory
// (robot_description/robots/<model>): robot.urdf, the tool frame from
// config.json frames.tool, and the chain from rtcore_definition.json, its
// arm joints that take part in Cartesian motion, in definition order (the
// order of q in rt-core and the resolver).
func LoadKinematics(descriptionDir string) (*Kinematics, error) {
	var def struct {
		Joints []struct {
			URDFJoint             string `json:"urdf_joint"`
			Kind                  string `json:"kind"`
			CartesianParticipates bool   `json:"cartesian_participates"`
		} `json:"joints"`
	}
	var cfg struct {
		Frames struct {
			Tool string `json:"tool"`
		} `json:"frames"`
	}
	for name, into := range map[string]any{"rtcore_definition.json": &def, "config.json": &cfg} {
		raw, err := os.ReadFile(filepath.Join(descriptionDir, name))
		if err != nil {
			return nil, err
		}
		if err = json.Unmarshal(raw, into); err != nil {
			return nil, fmt.Errorf("%s: %w", name, err)
		}
	}
	var joints []string
	for _, j := range def.Joints {
		if j.Kind == "arm" && j.CartesianParticipates {
			joints = append(joints, j.URDFJoint)
		}
	}
	if len(joints) == 0 || cfg.Frames.Tool == "" {
		return nil, fmt.Errorf("%s: no Cartesian arm joints or no frames.tool", descriptionDir)
	}
	urdf, err := os.ReadFile(filepath.Join(descriptionDir, "robot.urdf"))
	if err != nil {
		return nil, err
	}
	k, err := NewKinematics(urdf, joints, cfg.Frames.Tool)
	if err != nil {
		return nil, fmt.Errorf("%s: %w", descriptionDir, err)
	}
	return k, nil
}

// FK returns the TCP pose in the J1 base frame as the resolver's fk
// "transform": row-major 3x4 [R | p], p in metres.
func (k *Kinematics) FK(q []float64) [12]float64 {
	R, P := ident(), vec3{}
	for i := range k.axis {
		P = P.add(R.apply(k.origP[i]))
		R = R.mul(k.origR[i]).mul(axisRot(k.axis[i], q[i]))
	}
	P = P.add(R.apply(k.toolP))
	R = R.mul(k.toolR)
	var t [12]float64
	for row := 0; row < 3; row++ {
		t[row*4], t[row*4+1], t[row*4+2], t[row*4+3] = R[row][0], R[row][1], R[row][2], P[row]
	}
	return t
}
