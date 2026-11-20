//go:build linux

package main

import (
	"bytes"
	"context"
	"strings"
	"testing"

	"rosieos/rt-core/ipcclient"
)

func TestCollisionTuningPeaksAndSummary(t *testing.T) {
	for _, phase := range []string{"move", "program"} {
		var sink, out bytes.Buffer
		r := &runner{o: options{phase: phase, tuneCollision: true, collisionMargin: 1.5}, e: &evidence{sink: &sink}, out: &out}
		a := ipcclient.AxisStatus{PDOFresh: 1, CollisionWatchdog: ipcclient.CollisionWatchdogStatus{Samples: 100, PeakTorqueRaw: 400, PeakFollowingErrorCounts: 63}}
		if err := r.observeCollision(phase, 0, a); err != nil {
			t.Fatal(err)
		}
		a.CollisionWatchdog.PeakTorqueRaw = 1
		a.CollisionWatchdog.PeakFollowingErrorCounts = 1
		if err := r.observeCollision(phase, 0, a); err != nil {
			t.Fatal(err)
		}
		row := r.collisionTuning[0]
		if row.PeakTorque != 400 || row.PeakError != 63 || row.SuggestedTorque != 600 || row.SuggestedError != 95 || !row.Usable {
			t.Fatal(row)
		}
		if err := r.summary(nil); err != nil {
			t.Fatal(err)
		}
		if !strings.Contains(sink.String(), `"collision_watchdog_thresholds"`) || !strings.Contains(out.String(), "suggested_following_error_counts_max=95") {
			t.Fatal("missing tuning summary")
		}
		a.CollisionWatchdog.PeakTorqueRaw = 1000
		if err := r.observeCollision(phase, 1, a); err != nil {
			t.Fatal(err)
		}
		if r.collisionTuning[1].Usable || r.collisionTuning[1].SuggestedTorque != 1500 {
			t.Fatal("silently clamped torque suggestion")
		}
		a.PDOFresh = 0
		if r.observeCollision(phase, 0, a) == nil {
			t.Fatal("stale samples accepted")
		}
		a.PDOFresh = 1
		a.CollisionWatchdog.Samples = 0
		if r.observeCollision(phase, 0, a) == nil {
			t.Fatal("missing native peaks accepted")
		}
	}
}

func TestCollisionMarginAdmission(t *testing.T) {
	base := []string{"--pair-id", "bench", "--pair-revision", "1", "--configuration-sha256", strings.Repeat("0", 64),
		"--evidence", "unused.jsonl", "--phase", "move", "--axis", "0", "--delta-rad", "0.02", "--tune-collision"}
	for _, margin := range []string{"NaN", "+Inf", "0", "1", "-1", "11"} {
		if _, err := parseOptions(append(append([]string{}, base...), "--collision-margin", margin), &bytes.Buffer{}); err == nil {
			t.Fatal("accepted margin", margin)
		}
	}
	o, err := parseOptions(base, &bytes.Buffer{})
	if err != nil || !o.tuneCollision || o.collisionMargin != 1.5 {
		t.Fatal(o, err)
	}
}

// Real simulator and public adapter observations prove that disabled watchdogs
// still accumulate cycle peaks and both motion phases retain tuning summaries.
func TestSimulationCollisionTuning(t *testing.T) {
	for _, phase := range []string{"move", "program"} {
		t.Run(phase, func(t *testing.T) {
			f := startFixtureMeasurement(t, false, true, "--sim-following-error-counts", "-63")
			f.phase(t, "home")
			args := f.args(phase, "collision-"+phase, true)
			if phase == "program" {
				args = append(args, "--axis-mask", "1", "--delta-rad", "0.02", "--velocity-fraction", "0.5", "--accel-fraction", "0.5")
			}
			args = append(args, "--tune-collision", "--collision-margin", "2")
			var out bytes.Buffer
			if code := run(context.Background(), args, &out); code != 0 {
				t.Fatalf("exit=%d\n%s", code, out.String())
			}
			if !strings.Contains(out.String(), "suggested_following_error_counts_max=126") || !strings.Contains(out.String(), "_unverified=true") {
				t.Fatal(out.String())
			}
		})
	}
}
