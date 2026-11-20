//go:build linux

package denseexec

import (
	"context"
	"encoding/json"
	"testing"

	"rosieos/rt-core/sdk/control"
	"rosieos/rt-core/tools/fixturedelta"
)

// Dense acceptance follows the same absolute fixture oracle as the three
// displays, preventing unknown recovery or cross-kind retention from drifting.
func TestReviewRound2DenseUnknownMatchesSharedMatrix(t *testing.T) {
	raw, err := fixturedelta.ExpandFile("testdata/operator_feedback_fixtures.delta.json")
	if err != nil {
		t.Fatal(err)
	}
	var fixtures []struct {
		Name      string `json:"name"`
		Rejection *struct {
			Kind string `json:"commandKind"`
			Code string `json:"code"`
		} `json:"rejection"`
		Accepted string `json:"acceptedKind"`
		Expected struct {
			Rejection *struct {
				Code string `json:"code"`
			} `json:"rejection"`
		} `json:"expected"`
	}
	if err := json.Unmarshal(raw, &fixtures); err != nil {
		t.Fatal(err)
	}
	count := 0
	for _, fixture := range fixtures {
		if fixture.Accepted == "" || fixture.Rejection == nil {
			continue
		}
		count++
		t.Run(fixture.Name, func(t *testing.T) {
			r := &RTCoreController{feedbackKind: fixture.Rejection.Kind, feedbackCode: fixture.Rejection.Code}
			accept := func(context.Context) error { return nil }
			if fixture.Accepted == "unknown" {
				err = r.operation(context.Background(), accept)
			} else {
				err = r.operationKind(context.Background(), fixture.Accepted, accept)
			}
			if err != nil {
				t.Fatal(err)
			}
			want := ""
			if fixture.Expected.Rejection != nil {
				want = fixture.Expected.Rejection.Code
			}
			if r.feedbackCode != want || r.feedbackReceipt != 1 || !r.feedbackPending {
				t.Fatalf("got code=%q receipt=%d pending=%v, want code=%q receipt=1 pending=true", r.feedbackCode, r.feedbackReceipt, r.feedbackPending, want)
			}
		})
	}
	if count != 81 {
		t.Fatalf("matrix has %d cases, want 81", count)
	}
}

func TestRTCoreFeedbackStatusRetainsDescriptionAndTypedStartRefusal(t *testing.T) {
	r, f, id := rtFake(t)
	ctx := context.Background()
	initial := r.Status(ctx)
	if initial.RTCore.Status == nil || len(initial.RTCore.Description.Axes) != 9 {
		t.Fatal("read-only status before acquiring control omitted public observations")
	}
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	loaded, err := r.Load(ctx, LoadRequest{Identity: id})
	if err != nil {
		t.Fatal(err)
	}
	f.mu.Lock()
	f.refuse = "start_program"
	f.description.Axes[0].MinPosition = -5
	f.description.Axes[0].MaxPosition = 5
	f.mu.Unlock()
	if _, err = r.Play(ctx, loaded.SessionID); err == nil {
		t.Fatal("start was not refused")
	}
	f.mu.Lock()
	f.status.TimeNS = 1000000
	f.mu.Unlock()
	status := r.Status(ctx)
	if status.RTCore.RefusalAfterNS != 1000000 {
		t.Fatal("remote command receipt used a local clock instead of the server observation")
	}
	if status.RTCore.Refusal != "native_rejected" || len(status.RTCore.Description.Axes) != 9 ||
		status.RTCore.Description.Axes[0].MinPosition != -5 || status.RTCore.Description.Axes[0].MaxPosition != 5 {
		t.Fatalf("feedback missing: %+v", status.RTCore)
	}
	raw, err := json.Marshal(status)
	if err != nil {
		t.Fatal(err)
	}
	var wire struct {
		RTCore struct {
			Refusal     string `json:"refusal"`
			Description struct {
				Axes []any `json:"axes"`
			} `json:"description"`
		} `json:"rt_core"`
	}
	if err = json.Unmarshal(raw, &wire); err != nil {
		t.Fatal(err)
	}
	if wire.RTCore.Refusal != "native_rejected" || len(wire.RTCore.Description.Axes) != 9 {
		t.Fatal(string(raw))
	}
	if _, err = r.Stop(ctx); err != nil {
		t.Fatal(err)
	}
	if r.Status(ctx).RTCore.Refusal != "native_rejected" {
		t.Fatal("accepted Stop cleared unrelated program refusal")
	}
}

func TestReviewProgramRefusalSurvivesAcceptedArm(t *testing.T) {
	r, f, id := rtFake(t)
	ctx := context.Background()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	loaded, err := r.Load(ctx, LoadRequest{Identity: id})
	if err != nil {
		t.Fatal(err)
	}
	f.mu.Lock()
	f.refuse = "start_program"
	f.mu.Unlock()
	if _, err := r.Play(ctx, loaded.SessionID); err == nil {
		t.Fatal("expected refused start")
	}
	if got := r.statusSnapshot().RTCore.Refusal; got != "native_rejected" {
		t.Fatal(got)
	}
	f.mu.Lock()
	f.refuse = ""
	f.mu.Unlock()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	if got := r.statusSnapshot().RTCore.Refusal; got != "native_rejected" {
		t.Fatalf("accepted Arm erased program-start refusal: %q", got)
	}
}

func TestRTCoreFeedbackNativeCommandKinds(t *testing.T) {
	for _, tc := range []struct{ op, kind string }{{"enable", "enable"}, {"arm", "arm"}, {"home", "home"}, {"reset_fault", "reset"}, {"start_program", "program_start"}} {
		t.Run(tc.kind, func(t *testing.T) {
			r, f, _ := rtFake(t)
			f.mu.Lock()
			f.refuse = tc.op
			f.mu.Unlock()
			err := r.operationKind(context.Background(), "arm", func(ctx context.Context) error { return r.command(ctx, tc.op, control.Identity{}) })
			if err == nil || r.statusSnapshot().RTCore.RefusalKind != tc.kind {
				t.Fatal(err, r.statusSnapshot().RTCore)
			}
			f.mu.Lock()
			f.refuse = ""
			f.mu.Unlock()
			if err = r.command(context.Background(), tc.op, control.Identity{}); err != nil {
				t.Fatal(err)
			}
			if r.statusSnapshot().RTCore.Refusal != "" {
				t.Fatal("matching acceptance retained refusal")
			}
		})
	}
}

type reviewDelayedStatus struct {
	*fakeRTCore
	entered chan struct{}
	release chan struct{}
}

func (f *reviewDelayedStatus) Status(ctx context.Context) (control.Status, error) {
	observed, err := f.fakeRTCore.Status(ctx)
	close(f.entered)
	<-f.release
	return observed, err
}

func TestReviewPreCommandStatusCannotFenceAcceptedReceipt(t *testing.T) {
	r, f, _ := rtFake(t)
	f.mu.Lock()
	f.status.TimeNS, f.status.Motion.TimeNS = 1000000, 1000000
	f.mu.Unlock()
	delayed := &reviewDelayedStatus{fakeRTCore: f, entered: make(chan struct{}), release: make(chan struct{})}
	r.c = delayed
	done := make(chan *StatusBody, 1)
	go func() { done <- r.Status(context.Background()) }()
	<-delayed.entered
	if err := r.operation(context.Background(), func(context.Context) error { return nil }); err != nil {
		t.Fatal(err)
	}
	close(delayed.release)
	observed := <-done
	r.mu.Lock()
	pending, watermark := r.feedbackPending, r.feedbackAfterNS
	r.mu.Unlock()
	if !pending || watermark != 0 {
		t.Fatalf("pre-command sample fenced accepted receipt: pending=%v watermark=%d published=%d", pending, watermark, observed.RTCore.RefusalAfterNS)
	}
	r.c = f
	observed = r.Status(context.Background())
	r.mu.Lock()
	pending, watermark = r.feedbackPending, r.feedbackAfterNS
	receipt := r.feedbackReceipt
	r.mu.Unlock()
	if pending || watermark != 1000000 || receipt != 1 || observed.RTCore.RefusalAfterNS != 1000000 {
		t.Fatalf("post-receipt read did not establish the fence: pending=%v watermark=%d receipt=%d", pending, watermark, receipt)
	}
}
