//go:build linux

package ipcclient

import (
	"context"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"golang.org/x/sys/unix"
)

func TestREV02PendingHomeFailureRetiresOldRegionOnce(t *testing.T) {
	// This uses the ordinary pending Home transaction. Without an explicit
	// blocked-SDO hook, it does not qualify shutdown-before-blocked-join timing.
	t.Setenv("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "0")
	f := startCoreWithRole(t, "simulation-home.json", false)
	c := f.client
	identity := GrantIdentity{CapabilityID: [16]byte{2, 4, 2}, Generation: 1}
	if _, err := c.MirrorAcquire(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease); err != nil {
		t.Fatal(err)
	}
	// Retain an independent mapping after the ordinary client releases its own.
	// This models a stale peer still holding the retired memfd, without touching
	// unmapped pointers or asking native to reuse an old region in place.
	retiredMapping, err := unix.Mmap(c.fds[4], 0, applicationGrantRegionBytes, unix.PROT_READ|unix.PROT_WRITE, unix.MAP_SHARED)
	if err != nil {
		t.Fatal(err)
	}
	defer unix.Munmap(retiredMapping)
	oldSlot, status := bindSharedWords(retiredMapping[applicationGrantObserveAt:applicationGrantObserveAt+8+GrantObservationBytes], GrantObservationBytes/8)
	if status != sharedWordsOK {
		t.Fatal(status)
	}
	initial := waitGrantObservation(t, c, func(o GrantObservation) bool { return o.GrantActive == 1 })
	sequence, err := c.Home(context.Background(), 1)
	if err != nil {
		t.Fatal(err)
	}
	active := wait(t, c, func(s Status) bool {
		return s.CommissioningPhase == 1 && s.NativeHomeActiveMask == 1 && s.HomeValidMask == 0
	})
	if _, err := c.MirrorRenew(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease); err != nil {
		t.Fatal(err)
	}
	c.stateMu.Lock()
	_, homeAlreadyReturned := c.results[sequence]
	c.stateMu.Unlock()
	if homeAlreadyReturned {
		t.Fatal("Home completed before terminal publication-failure injection")
	}
	injected := HostMonotonicNS()
	for {
		previous := atomic.LoadUint64(oldSlot.word(0))
		if previous&1 == 0 && atomic.CompareAndSwapUint64(oldSlot.word(0), previous, previous|1) {
			break
		}
		if HostMonotonicNS()-injected > uint64(250*time.Millisecond) {
			t.Fatal("could not inject the test-only failed observation publisher")
		}
	}
	for deadline := time.Now().Add(time.Second); c.Err() == nil; {
		if time.Now().After(deadline) {
			t.Fatal("terminal observation failure left a pending-Home connection open")
		}
		time.Sleep(time.Millisecond)
	}
	closed := HostMonotonicNS()
	_ = c.Close()
	// Replacement can wait for Home/helper cleanup; closure does not authorize
	// early unmapping. A new bootstrap proves the old helper/mapping retired.
	var replacement *Client
	for deadline := time.Now().Add(3 * time.Second); ; {
		ctx, cancel := context.WithTimeout(context.Background(), 200*time.Millisecond)
		replacement, err = Dial(ctx, f.options)
		cancel()
		if err == nil {
			break
		}
		if time.Now().After(deadline) {
			t.Fatalf("replacement blocked after pending-Home cleanup: %v", err)
		}
		time.Sleep(10 * time.Millisecond)
	}
	defer replacement.Close()
	newIdentity := GrantIdentity{CapabilityID: [16]byte{2, 4, 3}, Generation: 1}
	grant, err := replacement.MirrorAcquire(context.Background(), newIdentity, HostMonotonicNS()+applicationGrantMaxLease)
	if err != nil {
		t.Fatal(err)
	}
	current := waitGrantObservation(t, replacement, func(o GrantObservation) bool { return o.GrantActive == 1 })
	if current.ConnectionId <= initial.ConnectionId {
		t.Fatalf("replacement reused failed connection identity: old=%d new=%d", initial.ConnectionId, current.ConnectionId)
	}
	until := HostMonotonicNS() + uint64(300*time.Millisecond)
	nextRenew := HostMonotonicNS() + uint64(100*time.Millisecond)
	latestObservationNS := current.NowHostNs
	for HostMonotonicNS() < until {
		// Deliberately corrupt only the independently retained OLD object. A
		// successor must not consume its events or borrow its mapping again.
		atomic.StoreUint64(oldSlot.word(0), ^uint64(0)-1)
		atomic.StoreUint64(oldSlot.word(0), 1)
		now := HostMonotonicNS()
		if now >= nextRenew {
			if _, err := replacement.MirrorRenew(context.Background(), newIdentity, now+applicationGrantMaxLease); err != nil {
				t.Fatalf("retired-region mutation revoked successor: %v", err)
			}
			nextRenew = now + uint64(100*time.Millisecond)
		}
		observation, err := replacement.FastObservation()
		if err != nil && !errors.Is(err, ErrFastObservationUnavailable) {
			t.Fatalf("invalid successor observation after old-region mutation: %v", err)
		}
		if err == nil && (observation.ConnectionId != current.ConnectionId || observation.NativeGeneration != grant.NativeGeneration || observation.GrantActive != 1) {
			t.Fatalf("stale region affected successor authority: %+v", observation)
		}
		if err == nil {
			latestObservationNS = observation.NowHostNs
		}
		if replacement.Err() != nil {
			t.Fatal(replacement.Err())
		}
		time.Sleep(time.Millisecond)
	}
	if latestObservationNS <= current.NowHostNs+uint64(200*time.Millisecond) {
		t.Fatal("no advancing successor observations while the retired region was mutated")
	}
	final := wait(t, replacement, func(s Status) bool { return s.TimeNS > current.NowHostNs })
	if final.CommissioningPhase != 0 || final.NativeHomeActiveMask != 0 || final.HomeValidMask != 0 ||
		final.Armed != 0 || final.AxisEnableMask != 0 || final.Axes[0].Statusword&0x6f != 0x40 {
		t.Fatalf("retired Home published stale success or service permission: %+v", final)
	}
	raw, err := os.ReadFile(filepath.Join(filepath.Dir(f.options.SocketPath), "core.log"))
	if err != nil {
		t.Fatal(err)
	}
	// Match only this structured failure diagnostic, not unrelated daemon logs.
	connectionField := fmt.Sprintf("connection_id=%d", initial.ConnectionId)
	failureLines := 0
	for _, line := range strings.Split(string(raw), "\n") {
		if strings.Contains(line, "fast-control observation publication failed") && strings.Contains(line, connectionField) {
			failureLines++
		}
	}
	if failureLines != 1 {
		t.Fatalf("expected one terminal publication diagnostic for %s, got %d", connectionField, failureLines)
	}
	t.Logf("pending Home epoch=%d; failed connection %d closed after %d ns; replacement %d retained authority while old memfd was mutated; one diagnostic",
		active.HomeEpoch, initial.ConnectionId, closed-injected, current.ConnectionId)
}
