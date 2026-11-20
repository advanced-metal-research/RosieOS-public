//go:build linux

package control

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"log"
	"os"
	"regexp"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

func captureJournal(t *testing.T) *bytes.Buffer {
	t.Helper()
	var journal bytes.Buffer
	log.SetOutput(&journal)
	t.Cleanup(func() { log.SetOutput(os.Stderr) })
	return &journal
}

// the 1430 cell 2026-10-10 22:01:13: a held jog ended because one speed frame met a
// second publisher in rt-control and was refused as not_ready (code 3) with
// 195 ms of its lifetime left. A collision shorter than the retry window must
// publish the frame, record no refusal and leave the stream open.
func TestRemoteJogPublisherCollisionIsRetried(t *testing.T) {
	f := startRemoteMotion(t)
	generation := f.begin(t)
	w := f.dial(t, generation)
	w.calibrate(t)
	w.update(t, w.frame(f, generation, 1))
	waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence == 1 })
	// Another publisher owns the adapter's publication lock for 2 ms.
	f.controller.jogUpdateMu.Lock()
	w.update(t, w.frame(f, generation, 2))
	time.Sleep(2 * time.Millisecond)
	f.controller.jogUpdateMu.Unlock()
	waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence == 2 })
	w.update(t, w.frame(f, generation, 3))
	waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence == 3 })
	if o := f.controller.JogIngress(); o.Refused != 0 {
		t.Fatalf("a collision inside the retry window was refused: %+v", o)
	}
}

// A collision that outlasts the retry window is refused as jog_publisher_busy
// (transient), not not_ready; it is journaled with its full text; the stream
// stays open; and the producer's resend of the same input (new sequence, same
// origin and deadline) is admitted. Safety refusals keep their native codes.
func TestRemoteJogLongCollisionIsTransientAndJournaled(t *testing.T) {
	f := startRemoteMotion(t)
	journal := captureJournal(t)
	generation := f.begin(t)
	w := f.dial(t, generation)
	w.calibrate(t)
	w.update(t, w.frame(f, generation, 1))
	waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence == 1 })
	held := w.frame(f, generation, 2)
	f.controller.jogUpdateMu.Lock()
	w.update(t, held)
	w.rejection(t, 2, ReasonJogPublisherBusy)
	f.controller.jogUpdateMu.Unlock()
	if o := f.controller.JogIngress(); o.Refused != 1 || o.LatestRefusal.SourceSequence != 2 || o.LatestRefusal.Reason != ipcclient.JogReasonNotReady {
		t.Fatalf("collision accounting: %+v", o)
	}
	line := ""
	for _, candidate := range strings.Split(journal.String(), "\n") {
		if strings.Contains(candidate, "rt-control: jog refusal") && strings.Contains(candidate, "sequence=2 ") {
			line = candidate
		}
	}
	for _, want := range []string{"transport=wss", "reason=3", "name=not_ready", "transient=true",
		fmt.Sprintf("grant_generation=%d", f.fence.Generation), fmt.Sprintf("jog_generation=%d", generation),
		`cause="jog_publisher_busy"`, "refused_total=1"} {
		if !strings.Contains(line, want) {
			t.Fatalf("journal line lacks %q: %q", want, line)
		}
	}
	// Measured in absolute time: the frame's 200 ms lifetime minus delivery and the 5 ms window.
	left := regexp.MustCompile(`deadline_left_ms=(-?[0-9.]+)`).FindStringSubmatch(line)
	if left == nil {
		t.Fatalf("journal line lacks deadline_left_ms: %q", line)
	}
	if ms, err := strconv.ParseFloat(left[1], 64); err != nil || ms <= 0 || ms >= 200 {
		t.Fatalf("deadline left %q ms, want within (0, 200)", left[1])
	}
	if attempts := regexp.MustCompile(`attempts=([0-9]+)`).FindStringSubmatch(line); attempts == nil || attempts[1] == "1" {
		t.Fatalf("collision was not retried before refusal: %q", line)
	}
	resend := held
	resend.SourceSequence = 3
	w.update(t, resend)
	o := waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence == 3 })
	if o.UpdateReason != ipcclient.JogReasonNone && o.UpdateReason != ipcclient.JogReasonLimited {
		t.Fatalf("resend refused natively: %+v", o)
	}
	// A repeated sequence is still a terminal stale_sequence, never transient.
	w.update(t, resend)
	w.rejection(t, 3, "jog_native_rejected_6")
	if !strings.Contains(journal.String(), "name=stale_sequence transient=false") {
		t.Fatalf("stale sequence not journaled as terminal:\n%s", journal.String())
	}
}

// collidingCore makes the next publications and observation reads meet
// another writer exactly as ipcclient reports it: the fast-region lock held
// by a path chunk, Begin/End or grant request, or a jog observation slot
// caught mid-publication on every bounded attempt.
type collidingCore struct {
	*ipcclient.Client
	publish, observe atomic.Int32
	mu               sync.Mutex
	publishErr       error
}

func (c *collidingCore) collision() error {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.publishErr
}

func (c *collidingCore) UpdateLocalJog(ctx context.Context, session ipcclient.LocalJogSession, sequence, origin, deadline uint64, mask uint32, velocity []float64) error {
	if c.publish.Add(-1) >= 0 {
		return c.collision()
	}
	return c.Client.UpdateLocalJog(ctx, session, sequence, origin, deadline, mask, velocity)
}

func (c *collidingCore) UpdateLocalPathSpeed(ctx context.Context, session ipcclient.LocalJogSession, sequence, origin, deadline uint64, speed float64) error {
	if c.publish.Add(-1) >= 0 {
		return c.collision()
	}
	return c.Client.UpdateLocalPathSpeed(ctx, session, sequence, origin, deadline, speed)
}

func (c *collidingCore) JogObservation() (ipcclient.JogObservation, error) {
	if c.observe.Add(-1) >= 0 {
		return ipcclient.JogObservation{}, fmt.Errorf("%w: %w", ipcclient.ErrJogUnavailable, ipcclient.ErrSharedSlotBusy)
	}
	return c.Client.JogObservation()
}

// The 22:01:13 sub-causes inside the client: a fast-region publication
// collision and a busy observation slot (read by the stream pre-check, by
// validation and by publication) are each retried; the frame is published,
// nothing is refused and the stream stays open.
func TestRemoteJogClientCollisionsAreRetried(t *testing.T) {
	f := startRemoteMotion(t)
	core := &collidingCore{Client: f.client, publishErr: ipcclient.ErrFastPublisherBusy}
	replaceNativeTestClient(f.controller, core)
	generation := f.begin(t)
	w := f.dial(t, generation)
	w.calibrate(t)
	core.publish.Store(3)
	w.update(t, w.frame(f, generation, 1))
	waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence == 1 })
	core.observe.Store(4)
	w.update(t, w.frame(f, generation, 2))
	waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence == 2 })
	if core.publish.Load() >= 0 || core.observe.Load() >= 0 {
		t.Fatalf("collisions not exercised: publish %d observe %d", core.publish.Load(), core.observe.Load())
	}
	if o := f.controller.JogIngress(); o.Refused != 0 {
		t.Fatalf("client collision refused: %+v", o)
	}
	// A refusal that is not a collision keeps its native code and is not retried.
	core.mu.Lock()
	core.publishErr = &ipcclient.JogRejectedError{Reason: ipcclient.JogReasonNotReady}
	core.mu.Unlock()
	core.publish.Store(1)
	w.update(t, w.frame(f, generation, 3))
	w.rejection(t, 3, "jog_native_rejected_3")
	if core.publish.Load() != 0 {
		t.Fatalf("a terminal refusal was retried: %d", core.publish.Load())
	}
}

// UpdateJog shares the retry; a collision inside the window is not refused.
func TestUpdateJogPublisherCollisionIsRetried(t *testing.T) {
	f := startRemoteMotion(t)
	generation := f.begin(t)
	f.controller.jogUpdateMu.Lock()
	done := make(chan error, 1)
	now := ipcclient.HostMonotonicNS()
	go func() {
		velocity := make([]float64, 9)
		velocity[0] = .05
		done <- f.controller.UpdateJog(f.fence, generation, 1, now, now+200_000_000, 511, velocity)
	}()
	time.Sleep(2 * time.Millisecond)
	f.controller.jogUpdateMu.Unlock()
	if err := <-done; err != nil {
		t.Fatalf("UpdateJog after a 2 ms collision: %v", err)
	}
	waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence == 1 })
	if o := f.controller.JogIngress(); o.Refused != 0 {
		t.Fatalf("collision refused: %+v", o)
	}
}

// Only a busy publication lock or observation slot is transient. Identity,
// sequence, timing, authority and readiness refusals stay terminal.
func TestJogTransientClassification(t *testing.T) {
	transient := []error{errors.New(ReasonJogPublisherBusy), ipcclient.ErrJogPublisherBusy, ipcclient.ErrFastPublisherBusy,
		fmt.Errorf("%w: %w", ipcclient.ErrJogUnavailable, ipcclient.ErrSharedSlotBusy),
		fmt.Errorf("%w: %w", ipcclient.ErrFastObservationUnavailable, ipcclient.ErrSharedSlotBusy)}
	for _, err := range transient {
		if !jogTransient(err) || jogIngressReason(err) != ipcclient.JogReasonNotReady {
			t.Fatalf("%v: transient=%t reason=%d", err, jogTransient(err), jogIngressReason(err))
		}
	}
	terminal := []error{nil, ipcclient.ErrJogUnavailable, ipcclient.ErrFastObservationUnavailable, ipcclient.ErrRevoked,
		ipcclient.ErrClosed, context.Canceled, context.DeadlineExceeded, ErrJogClockBackward, ErrJogClockInputStale,
		errors.New(ReasonJogSessionOrSequenceStale), errors.New(ReasonJogSessionStale), errors.New(ReasonControlSessionStale),
		errors.New("invalid or expired jog update"), errors.New(ReasonIndependentJogUnavailable),
		&ipcclient.JogRejectedError{Reason: ipcclient.JogReasonNotReady}, &ipcclient.JogRejectedError{Reason: ipcclient.JogReasonWrongIdentity}}
	for _, err := range terminal {
		if jogTransient(err) {
			t.Fatalf("%v classified transient", err)
		}
	}
}

// The retry never outlives the input: it stops at the deadline or the window,
// whichever is first, and a terminal refusal or success is never repeated.
func TestRetryTransientJogBounds(t *testing.T) {
	busy := func() (uint32, error) { return ipcclient.JogReasonNotReady, ipcclient.ErrFastPublisherBusy }
	started := time.Now()
	deadline := ipcclient.HostMonotonicNS() + uint64(time.Millisecond)
	_, err, attempts := retryTransientJog(deadline, busy)
	if !jogTransient(err) || attempts < 2 || ipcclient.HostMonotonicNS() < deadline {
		t.Fatalf("deadline-bounded retry: %v after %d attempts", err, attempts)
	}
	if elapsed := time.Since(started); elapsed > time.Duration(jogTransientRetryNS)+50*time.Millisecond {
		t.Fatalf("retry ran %v past a 1 ms deadline", elapsed)
	}
	started = time.Now()
	retryTransientJog(^uint64(0), busy)
	if elapsed := time.Since(started); elapsed < time.Duration(jogTransientRetryNS) || elapsed > time.Duration(jogTransientRetryNS)+50*time.Millisecond {
		t.Fatalf("window-bounded retry ran %v, want about %v", elapsed, time.Duration(jogTransientRetryNS))
	}
	if _, _, attempts := retryTransientJog(ipcclient.HostMonotonicNS()-1, busy); attempts != 1 {
		t.Fatalf("expired input retried %d times", attempts)
	}
	stale := func() (uint32, error) {
		return ipcclient.JogReasonStaleSequence, errors.New(ReasonJogSessionOrSequenceStale)
	}
	if reason, _, attempts := retryTransientJog(^uint64(0), stale); attempts != 1 || reason != ipcclient.JogReasonStaleSequence {
		t.Fatalf("terminal refusal retried: %d attempts, reason %d", attempts, reason)
	}
	calls := 0
	clears := func() (uint32, error) {
		if calls++; calls < 3 {
			return busy()
		}
		return ipcclient.JogReasonNone, nil
	}
	if reason, err, attempts := retryTransientJog(^uint64(0), clears); reason != ipcclient.JogReasonNone || err != nil || attempts != 3 {
		t.Fatalf("cleared collision: reason %d %v after %d attempts", reason, err, attempts)
	}
}

// The journal budget writes every refusal up to its rate and then reports
// what it suppressed on the next written line.
func TestJogRefusalJournalBudget(t *testing.T) {
	journal := captureJournal(t)
	c := &Controller{}
	for i := 0; i < jogRefusalLogPerSecond+5; i++ {
		c.recordJogIngress(uint64(i+1), ipcclient.JogReasonInvalidVector, jogRefusal{transport: "datagram", cause: errors.New("invalid jog vector")})
	}
	if got := strings.Count(journal.String(), "rt-control: jog refusal"); got != jogRefusalLogPerSecond {
		t.Fatalf("%d lines in one second, want %d", got, jogRefusalLogPerSecond)
	}
	c.mu.Lock()
	c.jogRefusalLog.windowStart -= uint64(time.Second)
	c.mu.Unlock()
	c.recordJogIngress(99, ipcclient.JogReasonPathFault)
	lines := strings.Split(strings.TrimSpace(journal.String()), "\n")
	last := lines[len(lines)-1]
	for _, want := range []string{"sequence=99", "reason=21", "name=path_fault", "suppressed=5", "transport=unknown", "deadline_left_ms=none"} {
		if !strings.Contains(last, want) {
			t.Fatalf("line after the window lacks %q: %q", want, last)
		}
	}
	if o := c.JogIngress(); o.Refused != uint64(jogRefusalLogPerSecond+6) || o.LatestRefusal.Reason != ipcclient.JogReasonNotReady {
		t.Fatalf("counters must still count every refusal (path reasons clamp to not_ready): %+v", o)
	}
}
