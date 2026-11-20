//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"sync/atomic"
	"testing"
	"time"
)

// Draft regression loaded through a Go overlay, with no production edits.
// Test-only atomic fault injection models an interrupted/exhausted publisher;
// a normal client must never write the daemon-owned observation sequence.
func TestREV02ObservationPublicationFailureRetiresConnection(t *testing.T) {
	for _, mode := range []string{"busy", "exhausted"} {
		t.Run(mode, func(t *testing.T) {
			f := startCoreWithRole(t, "simulation.json", false)
			c := f.client
			// Diagnostic observation remains available after correct server-side
			// teardown; Client.Status intentionally rejects a closed connection.
			waitDiagnostic := func(predicate func(Status) bool) Status {
				t.Helper()
				deadline := time.Now().Add(2 * time.Second)
				for {
					raw, err := os.ReadFile(filepath.Join(filepath.Dir(f.options.SocketPath), "metrics.json"))
					var state Status
					if err != nil || json.Unmarshal(raw, &state) != nil {
						t.Fatalf("diagnostic read failed: %v", err)
					}
					if state.ConfigurationSHA256 != f.options.ConfigurationSHA256 || state.Backend != "simulation" || len(state.Axes) != 2 {
						t.Fatalf("diagnostic identity changed: %+v", state)
					}
					if state.TimeNS <= HostMonotonicNS() && predicate(state) {
						return state
					}
					if time.Now().After(deadline) {
						t.Fatalf("diagnostic predicate did not converge: %+v", state)
					}
					time.Sleep(10 * time.Millisecond)
				}
			}
			identity := GrantIdentity{CapabilityID: [16]byte{2, 0, 2}, Generation: 1}
			receipt, err := c.MirrorAcquire(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease)
			if err != nil {
				t.Fatal(err)
			}
			arm(t, c, 3)
			if _, err := c.MirrorRenew(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease); err != nil {
				t.Fatal(err)
			}
			ready := wait(t, c, allOperationEnabled)
			if ready.SafetyFaultMask != 0 || ready.ExecutionFaultReasons != 0 {
				t.Fatalf("healthy precondition: %+v", ready)
			}
			injectedNS := HostMonotonicNS()
			c.fastMu.RLock()
			sequence := c.fast.observation.word(0)
			var poison uint64
			for {
				before := atomic.LoadUint64(sequence)
				poison = before | 1
				if mode == "exhausted" {
					poison = ^uint64(0) - 1
				}
				if before&1 == 0 && atomic.CompareAndSwapUint64(sequence, before, poison) {
					break
				}
				if HostMonotonicNS()-injectedNS > uint64(250*time.Millisecond) {
					c.fastMu.RUnlock()
					t.Fatal("could not acquire the sequence for test-only corruption")
				}
			}
			c.fastMu.RUnlock()
			inhibited := waitDiagnostic(func(s Status) bool {
				return s.TimeNS > injectedNS && s.Armed == 0 && s.AxisEnableMask == 0 && s.LeaseValid == 0 &&
					s.Axes[0].Statusword&0x4f == 0x40 && s.Axes[1].Statusword&0x4f == 0x40
			})
			later := waitDiagnostic(func(s Status) bool { return s.TimeNS > inhibited.TimeNS+uint64(100*time.Millisecond) })
			t.Logf("injection=%s sequence=%#x; inhibited DS402=%#x/%#x; native generation %d -> %d -> %d; client error=%v",
				mode, poison, inhibited.Axes[0].Statusword, inhibited.Axes[1].Statusword,
				receipt.NativeGeneration, inhibited.ControlGeneration, later.ControlGeneration, c.Err())
			if inhibited.ControlGeneration != receipt.NativeGeneration+1 {
				t.Errorf("REV02: one terminal publication failure fenced more than once: before=%d after=%d", receipt.NativeGeneration, inhibited.ControlGeneration)
			}
			if later.ControlGeneration != inhibited.ControlGeneration {
				t.Errorf("REV02: repeated failed publications kept advancing the same connection fence: %d -> %d", inhibited.ControlGeneration, later.ControlGeneration)
			}
			// A terminal publisher failure must cause off-RT socket teardown. Mere
			// permission revocation cannot leave an unusable client owning the slot.
			for until := time.Now().Add(400 * time.Millisecond); c.Err() == nil && time.Now().Before(until); {
				time.Sleep(10 * time.Millisecond)
			}
			if c.Err() == nil {
				t.Errorf("REV02: failed %s publisher left the heartbeat connection open instead of retiring it", mode)
			}
			ctx, cancel := context.WithTimeout(context.Background(), time.Second)
			replacement, replaceErr := Dial(ctx, f.options)
			cancel()
			if replaceErr != nil {
				t.Errorf("REV02: failed %s publisher still owns the sole connection slot: %v", mode, replaceErr)
			} else {
				_ = replacement.Close()
			}
			// Baseline control: explicit peer closure must release the slot. This
			// excludes a missing daemon, bad credentials, or incompatible fixture.
			_ = c.Close()
			deadline := time.Now().Add(2 * time.Second)
			if !simfixture.PollUntil(deadline, 10*time.Millisecond, func() bool {
				ctx, cancel := context.WithTimeout(context.Background(), 200*time.Millisecond)
				replacement, replaceErr = Dial(ctx, f.options)
				cancel()
				if replaceErr == nil {
					return true
				}
				if time.Now().After(deadline) {
					t.Fatalf("manual-close recovery control failed: %v", replaceErr)
				}
				return false
			}) {
				t.Fatal("observation deadline exceeded")
			}
			defer replacement.Close()
			fresh, err := replacement.MirrorAcquire(context.Background(), GrantIdentity{CapabilityID: [16]byte{9, 2}, Generation: 1}, HostMonotonicNS()+applicationGrantMaxLease)
			if err != nil {
				t.Fatalf("manual-close fresh-authority control failed: %v", err)
			}
			t.Logf("manual close released failed region; fresh connection granted at native generation %d", fresh.NativeGeneration)
		})
	}
}

func TestREV02ApplicationExpiryAdvancesNativeFenceOnce(t *testing.T) {
	c := applicationGrantNativeClient(t)
	identity := GrantIdentity{CapabilityID: [16]byte{2, 2, 2}, Generation: 1}
	receipt, err := c.MirrorAcquire(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease)
	if err != nil {
		t.Fatal(err)
	}
	// Ensure the previous_lease_valid integration latch has observed this grant
	// before the deliberate expiry. The fast receipt is already RT-owned.
	active := waitGrantObservation(t, c, func(o GrantObservation) bool {
		return o.GrantActive == 1 && o.NativeGeneration == receipt.NativeGeneration
	})
	if _, err := c.MirrorRenew(context.Background(), identity, HostMonotonicNS()+uint64(100*time.Millisecond)); err != nil {
		t.Fatal(err)
	}
	expired := waitGrantObservation(t, c, func(o GrantObservation) bool {
		return o.GrantActive == 0 && o.NativeGeneration > receipt.NativeGeneration
	})
	t.Logf("application expiry advanced native generation %d -> %d (delta %d), heartbeat connection error=%v",
		active.NativeGeneration, expired.NativeGeneration, expired.NativeGeneration-active.NativeGeneration, c.Err())
	if expired.NativeGeneration != active.NativeGeneration+1 {
		t.Error(fmt.Sprintf("REV02: one application expiry must have one native revoke effect: before=%d after=%d", active.NativeGeneration, expired.NativeGeneration))
	}
}

func TestREV02IndependentNativeStopsEachAdvanceFence(t *testing.T) {
	c := applicationGrantNativeClient(t)
	identity := GrantIdentity{CapabilityID: [16]byte{2, 3, 2}, Generation: 1}
	receipt, err := c.MirrorAcquire(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease)
	if err != nil {
		t.Fatal(err)
	}
	before := receipt.NativeGeneration
	for request := 1; request <= 2; request++ {
		// Send exactly one native Stop packet. Client.Stop deliberately sends
		// RTDISARM followed by RTSTOP02, which are two independent wire fences;
		// it would obscure a duplicate caused by the RT falling-edge gate.
		ticket := HostMonotonicNS()
		var packet [16]byte
		copy(packet[:], "RTSTOP02")
		wire.PutUint64(packet[8:], ticket)
		if err := c.socketPacket(string(packet[:])); err != nil {
			t.Fatal(err)
		}
		observed := waitGrantObservation(t, c, func(o GrantObservation) bool {
			return o.StopAckId == ticket && o.GrantActive == 0 && o.NativeGeneration > before
		})
		t.Logf("independent native Stop %d acknowledged ticket %d, generation %d -> %d", request, ticket, before, observed.NativeGeneration)
		if observed.NativeGeneration != before+1 {
			t.Errorf("REV02: native Stop %d had duplicate revoke effects: before=%d after=%d", request, before, observed.NativeGeneration)
		}
		before = observed.NativeGeneration
	}
}
