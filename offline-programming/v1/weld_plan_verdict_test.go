package main

import (
	"encoding/json"
	"strings"
	"testing"
)

func verdictOf(t *testing.T, raw string) planVerdict {
	t.Helper()
	var verdict planVerdict
	if err := json.Unmarshal([]byte(raw), &verdict); err != nil {
		t.Fatalf("unmarshal: %v", err)
	}
	return verdict
}

func TestAWeldWithNoPathIsNamedBeforeTheTransitsItBroke(t *testing.T) {
	// The operator's program, 2026-09-11: four welds, two authored against the
	// Ø16 nozzle crossed nothing, so the join refused its move list. The reply
	// said "the welds planned but nothing connects them". Two did not plan.
	verdict := verdictOf(t, `{
	  "seams": [
	    {"id": "seam_0006", "crossed": false, "feasible_rate": 0.0165,
	     "frontier_causes": {"no ik": 41668, "joint limit": 43593, "collision": 3842}},
	    {"id": "seam_0013", "crossed": false, "feasible_rate": 0.0145,
	     "frontier_causes": {"joint limit": 53396, "no ik": 31344}},
	    {"id": "seam_0010", "crossed": true, "feasible_rate": 0.0216},
	    {"id": "seam_0014", "crossed": true, "feasible_rate": 0.0153}
	  ],
	  "connecting_trajectories_error": "FreeSpaceError: the program states approach, transit, transit, transit, retract between 2 welds",
	  "dense": null
	}`)
	code, message := verdict.failure()
	if code != "motion_plan_seam_refused" {
		t.Fatalf("code %q, want motion_plan_seam_refused: %s", code, message)
	}
	for _, want := range []string{
		"2 of 4 weld(s)",
		"seam_0006: no path the arm can follow (1.65% of poses feasible, joint limit 43593, no ik 41668, collision 3842)",
		"seam_0013",
	} {
		if !strings.Contains(message, want) {
			t.Errorf("message lacks %q:\n%s", want, message)
		}
	}
	if strings.Contains(message, "seam_0010") || strings.Contains(message, "seam_0014") {
		t.Errorf("a weld that planned was named as refused:\n%s", message)
	}
}

func TestARefusalWithAReasonIsNamedWithIt(t *testing.T) {
	verdict := verdictOf(t, `{"seams": [{"id": "seam_0001", "error": "no standoff authored"}], "dense": null}`)
	if code, message := verdict.failure(); code != "motion_plan_seam_refused" || !strings.Contains(message, "seam_0001: no standoff authored") {
		t.Fatalf("got %q %q", code, message)
	}
}

func TestWeldsThatAllPlannedButCannotBeJoinedSaySo(t *testing.T) {
	verdict := verdictOf(t, `{
	  "seams": [{"id": "a", "crossed": true}, {"id": "b", "crossed": true}],
	  "connecting_trajectories_error": "no route between a and b", "dense": null}`)
	if code, message := verdict.failure(); code != "motion_plan_unjoined" || !strings.Contains(message, "The welds planned, but connecting motion could not be planned: no route between a and b") {
		t.Fatalf("got %q %q", code, message)
	}
}

func TestTaughtMoveRefusalNamesTheMotionWithoutInventingWelds(t *testing.T) {
	const detail = "MoveL to P2 (program step 2, node p2): Taught Cartesian path exceeds the 0.01 mm / 0.001 rad tracking tolerance: 0.13064 mm, 0.000262 rad"
	raw, err := json.Marshal(detail)
	if err != nil {
		t.Fatal(err)
	}
	verdict := planVerdict{ConnectingError: raw}
	code, message := verdict.failure()
	if code != "motion_plan_unjoined" || message != "Program motion could not be planned: "+detail {
		t.Fatalf("got %q %q", code, message)
	}
	if strings.Contains(strings.ToLower(message), "weld") {
		t.Fatalf("a zero-seam program was described as a weld failure: %s", message)
	}
}

func TestConnectingRefusalKeepsNonStringPlannerEvidence(t *testing.T) {
	verdict := verdictOf(t, `{"seams": [], "connecting_trajectories_error": {"reason":"unsupported","node":"p2"}, "dense": null}`)
	code, message := verdict.failure()
	if code != "motion_plan_unjoined" || !strings.Contains(message, `{"reason":"unsupported","node":"p2"}`) {
		t.Fatalf("lost planner evidence: %q %q", code, message)
	}
}

func TestAPlanWithNoTrajectoryCarriesTheDenseError(t *testing.T) {
	verdict := verdictOf(t, `{"seams": [{"id": "a", "crossed": true}], "dense": null,
	  "dense_error": {"reason": "result_empty"}}`)
	code, message := verdict.failure()
	if code != "motion_plan_no_trajectory" || !strings.Contains(message, "result_empty") {
		t.Fatalf("got %q %q", code, message)
	}
	if strings.Contains(message, "{") {
		t.Fatalf("JSON leaked into the sentence: %s", message)
	}
}

func TestAPlanWithNoTrajectorySaysTheDenseRefusalAsASentence(t *testing.T) {
	verdict := verdictOf(t, `{
	  "seams": [{"id": "seam_0011", "crossed": true}],
	  "dense": null,
	  "dense_error": {
	    "reason": "motion_not_verified",
	    "detail": "seam seam_0011: FAIL (1 collisions; 2 limit violations); requires native PASS",
	    "segment_index": null
	  }
	}`)
	code, message := verdict.failure()
	if code != "motion_plan_no_trajectory" {
		t.Fatalf("code %q", code)
	}
	want := "seam seam_0011: FAIL (1 collisions; 2 limit violations); requires native PASS · motion_not_verified"
	if message != want {
		t.Fatalf("got %q, want %q", message, want)
	}
}

func TestAPlanRefusalNamesTheSegmentWhenItHasOne(t *testing.T) {
	verdict := verdictOf(t, `{
	  "seams": [{"id": "a", "crossed": true}],
	  "dense": null,
	  "dense_error": {"reason": "motion_not_verified", "detail": "limit", "segment_index": 3}
	}`)
	_, message := verdict.failure()
	want := "limit · motion_not_verified · segment 3"
	if message != want {
		t.Fatalf("got %q, want %q", message, want)
	}
}

func TestAPlanThatPlannedHasNoFailure(t *testing.T) {
	verdict := verdictOf(t, `{"seams": [{"id": "a", "crossed": true}], "dense": {"trajectory_digest": "x"}}`)
	if code, message := verdict.failure(); code != "" {
		t.Fatalf("a good plan failed: %q %q", code, message)
	}
}
