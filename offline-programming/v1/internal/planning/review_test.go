package planning

import (
	"testing"
	"time"
)

func readyReviewInput(t *testing.T) ExecutionReviewInput {
	t.Helper()
	now := time.Date(2026, 7, 30, 8, 0, 0, 0, time.UTC)
	evidence, err := NormalizeMotionServerReport(ReportContext{
		Request:      validPlanRequest(now),
		PlanID:       "plan-17",
		PlanRevision: 23,
		ReceivedAt:   now.Add(-100 * time.Millisecond),
	}, []byte(validMotionServerReport()))
	if err != nil {
		t.Fatalf("normalize ready fixture: %v", err)
	}
	return ExecutionReviewInput{
		Now:             now,
		Mode:            ModeRealConnected,
		ProjectID:       "project-dog-box",
		ProjectRevision: 7,
		RobotModel:      RobotModelRosie1400V3,
		SelectedTarget:  evidence.Identity.Target,
		Evidence:        evidence,
		Runtime: RuntimePlanStatus{
			Connected:       true,
			StatusSequence:  51,
			ObservedAt:      now.Add(-100 * time.Millisecond),
			FreshUntil:      now.Add(time.Second),
			Target:          evidence.Identity.Target,
			RobotModel:      RobotModelRosie1400V3,
			Plan:            evidence.Identity,
			StartPose:       StartPoseConverged,
			RuntimePlayable: true,
			Output: RuntimeOutputState{
				Known:               true,
				EtherCATEnabled:     true,
				MotionOutputEnabled: true,
				ServosArmed:         true,
				EmergencyStopClear:  true,
				FaultFree:           true,
				Stationary:          true,
				WeldOutputEnabled:   false,
			},
		},
		Authority: AuthorityProof{
			ControllerID:      "offline-programming:server-1",
			ControllerBootID:  "boot-1",
			TargetProfileID:   evidence.Identity.Target.ProfileID,
			LeaseID:           "motion-server-lease-1",
			FenceEpoch:        4,
			StatusSequence:    52,
			FreshUntil:        now.Add(time.Second),
			TransitionPending: false,
		},
	}
}

func TestReviewExecutionRejectsStaleMismatchedAndUnvalidatedPlans(t *testing.T) {
	tests := []struct {
		name string
		edit func(*ExecutionReviewInput)
		code string
	}{
		{
			name: "stale project revision",
			edit: func(input *ExecutionReviewInput) { input.ProjectRevision++ },
			code: "plan_project_must_be_current",
		},
		{
			name: "mismatched target",
			edit: func(input *ExecutionReviewInput) {
				input.SelectedTarget.ProfileID = "other-relay"
			},
			code: "plan_target_must_match",
		},
		{
			name: "mismatched robot model",
			edit: func(input *ExecutionReviewInput) { input.RobotModel = RobotModelRosie1420V1 },
			code: "plan_model_must_match",
		},
		{
			name: "unvalidated evidence",
			edit: func(input *ExecutionReviewInput) {
				input.Evidence.State = PlanUnvalidated
				input.Evidence.Accepted = false
			},
			code: "server_validated_plan_required",
		},
		{
			name: "stale runtime",
			edit: func(input *ExecutionReviewInput) {
				input.Runtime.FreshUntil = input.Now
			},
			code: "fresh_runtime_status_required",
		},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			input := readyReviewInput(t)
			test.edit(&input)
			review := ReviewExecution(input)
			if review.PrerequisitesReady || review.CanSubmit || review.CanExecute {
				t.Fatalf("%s plan became executable", test.name)
			}
			if !containsReason(review.Reasons, test.code) {
				t.Fatalf("review reasons = %v, want %q", review.Reasons, test.code)
			}
		})
	}
}

func TestReviewExecutionCanRepresentCompleteServerPrerequisites(t *testing.T) {
	review := ReviewExecution(readyReviewInput(t))
	if !review.PrerequisitesReady || review.CanSubmit || review.CanExecute ||
		!containsReason(review.Reasons, "real_execution_adapter_not_installed") {
		t.Fatalf("complete prerequisites unexpectedly blocked: %+v", review)
	}
}
