package robotdesc

// Human-readable comparison data derived from the same registered files as
// the description identity. These summaries are catalogue metadata, never a
// replacement for the identity fence and never another robot definition.
import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"io"
	"math"
	"os"
	"path/filepath"
	"sort"
)

type DescriptionSettings struct {
	Revision  *int                       `json:"revision,omitempty"`
	Joints    []DescriptionJointSettings `json:"joints"`
	Planning  *planningDocument          `json:"planning,omitempty"`
	Torch     *torchDocument             `json:"torch,omitempty"`
	Resources map[string]string          `json:"resources,omitempty"`
}

type DescriptionJointSettings struct {
	Name          string   `json:"name"`
	Type          string   `json:"type"`
	Lower         *float64 `json:"lower,omitempty"`
	Upper         *float64 `json:"upper,omitempty"`
	RatedVelocity *float64 `json:"rated_velocity,omitempty"`
	GearRatio     *float64 `json:"gear_ratio,omitempty"`
}

// No timestamp is inferred from modification times or provenance prose.
// Existing descriptions do not record a structured publication date.
func (d Description) settingsSnapshot() *DescriptionSettings {
	files, err := DescriptionFiles(d.Dir)
	if err != nil {
		return nil
	}
	out := &DescriptionSettings{Joints: []DescriptionJointSettings{}, Resources: map[string]string{}}
	entries := make([]Entry, 0, len(files)+1)
	registered := map[string]bool{}
	for _, name := range append(files, Manifest) {
		registered[name] = true
		file, err := os.Open(filepath.Join(d.Dir, filepath.FromSlash(name)))
		if err != nil {
			return nil
		}
		digest := sha256.New()
		_, readErr := io.Copy(digest, file)
		closeErr := file.Close()
		if readErr != nil || closeErr != nil {
			return nil
		}
		var sum [32]byte
		copy(sum[:], digest.Sum(nil))
		entries = append(entries, Entry{Path: name, SHA256: sum})
		out.Resources[name] = "sha256:" + hex.EncodeToString(sum[:])
	}
	// A file replaced since Load must not be labelled with the earlier identity.
	if HashEntries(entries) != d.Identity {
		return nil
	}
	ratios := map[string]float64{}
	if registered[Definition] {
		raw, err := os.ReadFile(filepath.Join(d.Dir, Definition))
		var definition struct {
			Schema   string `json:"schema"`
			Revision *int   `json:"revision"`
			Joints   []struct {
				Name      string `json:"name"`
				URDFJoint string `json:"urdf_joint"`
				GearRatio struct {
					Numerator   float64 `json:"numerator"`
					Denominator float64 `json:"denominator"`
				} `json:"gear_ratio"`
			} `json:"joints"`
		}
		if err == nil && resourceMatches(raw, out.Resources[Definition]) && json.Unmarshal(raw, &definition) == nil && definition.Schema == "rosie.robot-definition.v1" {
			if definition.Revision != nil && *definition.Revision > 0 {
				out.Revision = definition.Revision
			}
			for _, joint := range definition.Joints {
				if joint.GearRatio.Numerator <= 0 || joint.GearRatio.Denominator <= 0 {
					continue
				}
				ratio := joint.GearRatio.Numerator / joint.GearRatio.Denominator
				if math.IsInf(ratio, 0) || math.IsNaN(ratio) {
					continue
				}
				name := joint.URDFJoint
				if name == "" {
					name = joint.Name
				}
				ratios[name] = ratio
			}
		}
	}
	names := make([]string, 0, len(d.URDF.Joints))
	for name := range d.URDF.Joints {
		names = append(names, name)
	}
	sort.Strings(names)
	for _, name := range names {
		joint := d.URDF.Joints[name]
		item := DescriptionJointSettings{Name: name, Type: joint.Type}
		if joint.HasLower {
			value := joint.Lower
			item.Lower = &value
		}
		if joint.HasUpper {
			value := joint.Upper
			item.Upper = &value
		}
		if joint.HasVelocity {
			value := joint.Velocity
			item.RatedVelocity = &value
		}
		if value, ok := ratios[name]; ok {
			item.GearRatio = &value
		}
		out.Joints = append(out.Joints, item)
	}
	if registered["config.json"] {
		raw, err := os.ReadFile(filepath.Join(d.Dir, "config.json"))
		var config configDocument
		if err == nil && resourceMatches(raw, out.Resources["config.json"]) && json.Unmarshal(raw, &config) == nil {
			out.Planning, out.Torch = config.Planning, config.Torch
		}
	}
	return out
}

func resourceMatches(raw []byte, expected string) bool {
	sum := sha256.Sum256(raw)
	return "sha256:"+hex.EncodeToString(sum[:]) == expected
}
