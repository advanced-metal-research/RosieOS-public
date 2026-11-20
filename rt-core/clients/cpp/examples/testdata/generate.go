// Generates the shared simulation fixtures; run from the rt-core directory.
// Explicit samples keep rejection input independent of daemon-reported limits.
package main

import (
	"bytes"
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"rosieos/rt-core/adapters/rosie/densejointtraj"
)

func main() {
	check := flag.Bool("check", false, "compare regenerated fixture bytes without writing")
	flag.Parse()
	dir := "clients/cpp/examples/testdata"
	for name, raw := range fixtures() {
		if *check {
			stored, err := os.ReadFile(dir + "/" + name)
			must(err)
			if !bytes.Equal(raw, stored) {
				panic(fmt.Sprintf("stale profile fixture: %s", name))
			}
		} else {
			must(os.WriteFile(dir+"/"+name, raw, 0644))
		}
	}
}

// fixtures regenerates producer inputs in memory so tests detect stale samples
// and machine declarations without rewriting the immutable integration corpus.
func fixtures() map[string][]byte {
	result := make(map[string][]byte)
	raw, err := os.ReadFile("config/machines/simulation/simulation-program.json")
	must(err)
	var machine map[string]any
	must(json.Unmarshal(raw, &machine))
	for _, value := range machine["axes"].([]any) {
		axis := value.(map[string]any)
		axis["profile"] = "cia402_sim_home"
		axis["require_home"] = true
		// This is a synthetic admission fixture, not a robot configuration.
		// Keep the declared envelope paired with the explicit rejection samples
		// below instead of inheriting changing local-simulation robot limits.
		limits := axis["limits"].(map[string]any)
		limits["min"], limits["max"] = -.5, .5
		limits["max_velocity"], limits["jog_acceleration"] = .1, 1.0
		delete(limits, "max_source")
		delete(limits, "max_velocity_source")
	}
	raw, err = json.MarshalIndent(machine, "", "  ")
	must(err)
	result["profile-machine.json"] = append(raw, '\n')
	raw, err = os.ReadFile("adapters/rosie/densejointtraj/testdata/corpus/dense_joint_trajectory_reference.rdt")
	must(err)
	plan, err := densejointtraj.Decode(raw)
	must(err)
	for _, variant := range []string{"valid", "acceleration", "velocity"} {
		samples := make([]densejointtraj.Sample, 11)
		dt := .1
		if variant == "acceleration" {
			dt = .01
		}
		for i := range samples {
			samples[i].TimeS = float64(i) * dt
			samples[i].QRad[0] = float64(i) * .0001
			if i > 0 && i < 10 {
				samples[i].QdRadS[0] = .001
			}
		}
		if variant == "acceleration" {
			samples[1].QdRadS[0] = .05
		}
		if variant == "velocity" {
			samples[1].QdRadS[0] = .2
		}
		header := plan.Header
		header.PlanID = "integration-profile-" + variant
		header.Sampling.DtS = dt
		header.Sampling.TotalSampleCount = 11
		header.Sampling.TotalDurationS = 10 * dt
		header.Segments = []densejointtraj.SegmentMeta{{Index: 0, Kind: "freespace", SourceID: "profile", SampleCount: 11, DurationS: 10 * dt}}
		for i := range header.Limits.MaxAbsQdRadS {
			header.Limits.MaxAbsQdRadS[i] = .5
		}
		raw, err = densejointtraj.Encode(header, []densejointtraj.Segment{{Kind: "freespace", SourceID: "profile", Samples: samples}})
		must(err)
		result[variant+".rdt"] = raw
	}
	return result
}
func must(err error) {
	if err != nil {
		panic(err)
	}
}
