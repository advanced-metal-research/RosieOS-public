//go:build linux

// rdtcheck diagnoses verified dense trajectory bytes without sending motion.
package main

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"

	"rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/ipcclient"
)

type boundaryReport struct {
	Boundary           int                               `json:"boundary"`
	FromSegment        int                               `json:"from_segment"`
	ToSegment          int                               `json:"to_segment"`
	GapRad             [densejointtraj.AxisCount]float64 `json:"gap_rad"`
	GapCounts          [densejointtraj.AxisCount]float64 `json:"gap_counts"`
	LocalTimeGapS      float64                           `json:"local_time_gap_s"`
	AssembledTimeGapNS int64                             `json:"assembled_time_gap_ns"`
	BothStationary     bool                              `json:"both_stationary"`
	PrecedingDwellNS   int64                             `json:"preceding_dwell_ns"`
	AdapterVerdict     string                            `json:"adapter_verdict"`
}

type fileReport struct {
	File                 string           `json:"file"`
	Bytes                int              `json:"bytes"`
	SHA256               string           `json:"sha256"`
	SourceDigest         string           `json:"source_digest,omitempty"`
	NormalisedDigest     string           `json:"normalised_digest,omitempty"`
	SourceSamples        int              `json:"source_samples"`
	NormalisedSamples    int              `json:"normalised_samples"`
	SegmentDurationNS    []int64          `json:"segment_duration_ns"`
	NormalisationVerdict string           `json:"normalisation_verdict"`
	AdapterVerdict       string           `json:"adapter_verdict"`
	Boundaries           []boundaryReport `json:"boundaries"`
}

type report struct {
	Profile             string       `json:"profile"`
	ProfileSHA256       string       `json:"profile_sha256"`
	Configuration       string       `json:"configuration"`
	ConfigurationSHA256 string       `json:"configuration_sha256"`
	AnalysisMapping     string       `json:"analysis_mapping"`
	AxisIDs             []string     `json:"axis_ids"`
	CountsPerRad        float64      `json:"counts_per_rad"`
	Files               []fileReport `json:"files"`
}

func digest(blob []byte) string {
	sum := sha256.Sum256(blob)
	return "sha256:" + hex.EncodeToString(sum[:])
}

// a6Description reads the actual scaling and limits; repeating the bench axis is
// an analysis map only, preventing a single-axis configuration being misreported
// as native admission evidence for a nine-axis robot.
func a6Description(root string) (report, ipcclient.Description, error) {
	r := report{Profile: "config/drives/a6ec_bench_1to1.json", Configuration: "config/machines/a6ec-bench.example.json",
		AnalysisMapping: "bench_axis scaling and limits repeated in source J1..J9 order; analysis only"}
	profile, err := os.ReadFile(filepath.Join(root, r.Profile))
	if err != nil {
		return r, ipcclient.Description{}, err
	}
	config, err := os.ReadFile(filepath.Join(root, r.Configuration))
	if err != nil {
		return r, ipcclient.Description{}, err
	}
	var p struct {
		ID string `json:"id"`
	}
	var c struct {
		Axes []struct {
			Profile            string  `json:"profile"`
			Type               string  `json:"type"`
			Sign               int     `json:"sign"`
			WireCountsPerRev   float64 `json:"wire_counts_per_rev"`
			WireRevsPerAxisRev float64 `json:"wire_revs_per_axis_rev"`
			Limits             struct {
				Min      float64 `json:"min"`
				Max      float64 `json:"max"`
				Velocity float64 `json:"max_velocity"`
			} `json:"limits"`
		} `json:"axes"`
	}
	if err := json.Unmarshal(profile, &p); err != nil {
		return r, ipcclient.Description{}, err
	}
	if err := json.Unmarshal(config, &c); err != nil {
		return r, ipcclient.Description{}, err
	}
	// The bench file retains the A6 family id; its filename selects the 1:1 evidence policy.
	if len(c.Axes) != 1 || c.Axes[0].Profile != "a6ec_bench_1to1" || p.ID != "a6ec" || c.Axes[0].Type != "rotary" {
		return r, ipcclient.Description{}, fmt.Errorf("expected one rotary A6 bench axis")
	}
	a := c.Axes[0]
	r.CountsPerRad = a.WireCountsPerRev * a.WireRevsPerAxisRev / (2 * math.Pi)
	if !(r.CountsPerRad > 0) {
		return r, ipcclient.Description{}, fmt.Errorf("invalid A6 scaling")
	}
	r.ProfileSHA256, r.ConfigurationSHA256 = digest(profile), digest(config)
	var description ipcclient.Description
	for i := 0; i < densejointtraj.AxisCount; i++ {
		id := fmt.Sprintf("J%d", i+1)
		r.AxisIDs = append(r.AxisIDs, id)
		description.Axes = append(description.Axes, ipcclient.AxisDescription{
			ID: id, PositionUnit: "rad", CountsPerUnit: r.CountsPerRad, Sign: a.Sign,
			MinPosition: a.Limits.Min, MaxPosition: a.Limits.Max, MaxVelocity: a.Limits.Velocity})
	}
	return r, description, nil
}

// analyse preserves each local clock in the report. The assembled seam has zero
// elapsed time; a negative local gap records the clock reset, not backward motion.
func analyse(path string, description ipcclient.Description, ids []string, counts float64) (fileReport, error) {
	blob, err := os.ReadFile(path)
	if err != nil {
		return fileReport{}, err
	}
	f := fileReport{File: filepath.Base(path), Bytes: len(blob), SHA256: digest(blob), Boundaries: []boundaryReport{}}
	_, adapterErr := control.DecodeProgram(blob, description, ids)
	f.AdapterVerdict = "accept"
	if adapterErr != nil {
		f.AdapterVerdict = adapterErr.Error()
	}
	plan, decodeErr := densejointtraj.Inspect(blob)
	if plan == nil {
		f.NormalisationVerdict = decodeErr.Error()
		return f, nil
	}
	f.SourceDigest, f.SourceSamples = plan.Header.TrajectoryDigest, plan.Header.Sampling.TotalSampleCount
	f.NormalisationVerdict = "accept"
	normalised, normaliseErr := densejointtraj.NormaliseStationary(plan)
	if normaliseErr != nil {
		f.NormalisationVerdict = normaliseErr.Error()
	} else {
		f.NormalisedSamples = f.SourceSamples - len(normalised) + 1
		f.NormalisedDigest = densejointtraj.TrajectoryDigest(plan.Header.Axes.AxisMask, plan.Header.Sampling.DtS, plan.Header.Limits.MaxAbsQdRadS, normalised)
	}
	for i, segment := range plan.Segments {
		samples := segment.Samples
		last := samples[len(samples)-1]
		f.SegmentDurationNS = append(f.SegmentDurationNS, int64(math.Round(last.TimeS*1e9)))
		if i+1 == len(plan.Segments) {
			continue
		}
		next := plan.Segments[i+1].Samples[0]
		boundary := boundaryReport{Boundary: i, FromSegment: i, ToSegment: i + 1,
			LocalTimeGapS: next.TimeS - last.TimeS, BothStationary: densejointtraj.Stationary(last) && densejointtraj.Stationary(next), AdapterVerdict: f.AdapterVerdict}
		for axis := range boundary.GapRad {
			boundary.GapRad[axis] = next.QRad[axis] - last.QRad[axis]
			boundary.GapCounts[axis] = boundary.GapRad[axis] * counts
		}
		first := len(samples) - 1
		for first > 0 && densejointtraj.Stationary(samples[first]) && densejointtraj.Stationary(samples[first-1]) && samples[first-1].QRad == last.QRad {
			first--
		}
		boundary.PrecedingDwellNS = int64(math.Round((last.TimeS - samples[first].TimeS) * 1e9))
		f.Boundaries = append(f.Boundaries, boundary)
	}
	return f, nil
}

func generate(root string, paths []string) ([]byte, report, error) {
	r, description, err := a6Description(root)
	if err != nil {
		return nil, r, err
	}
	for _, path := range paths {
		f, err := analyse(path, description, r.AxisIDs, r.CountsPerRad)
		if err != nil {
			return nil, r, err
		}
		r.Files = append(r.Files, f)
	}
	raw, err := json.MarshalIndent(r, "", "  ")
	return append(raw, '\n'), r, err
}

func table(w io.Writer, r report) {
	fmt.Fprintln(w, "file boundary gap_rad[J1..J9] gap_counts[J1..J9] local_gap_s assembled_gap_ns stationary dwell_ns adapter_verdict")
	for _, f := range r.Files {
		for _, b := range f.Boundaries {
			fmt.Fprintf(w, "%s %d %v %v %.9g %d %t %d %s\n", f.File, b.Boundary, b.GapRad, b.GapCounts, b.LocalTimeGapS, b.AssembledTimeGapNS, b.BothStationary, b.PrecedingDwellNS, b.AdapterVerdict)
		}
	}
}

func run() error {
	root := flag.String("root", ".", "rt-core root for A6 profile and configuration")
	output := flag.String("json", "", "JSON output path (default stdout); boundary table goes to stderr")
	flag.Parse()
	paths := flag.Args()
	if len(paths) == 0 {
		var err error
		paths, err = filepath.Glob(filepath.Join(*root, "adapters/rosie/densejointtraj/testdata/corpus/*.rdt"))
		if err != nil {
			return err
		}
	}
	if len(paths) == 0 {
		return fmt.Errorf("no .rdt inputs")
	}
	raw, r, err := generate(*root, paths)
	if err != nil {
		return err
	}
	table(os.Stderr, r)
	if *output != "" {
		return os.WriteFile(*output, raw, 0644)
	}
	_, err = os.Stdout.Write(raw)
	return err
}

func main() {
	if err := run(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}
