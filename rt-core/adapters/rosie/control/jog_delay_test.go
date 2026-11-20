//go:build linux

package control

import (
	"encoding/hex"
	"errors"
	"rosieos/rt-core/ipcclient"
	"testing"
)

// TestRemoteJogTransportDelay injects asymmetric transport timestamps into the
// production WSS calibration and conversion. A source clock five seconds ahead
// cannot hide network age; jitter changes age, never the calibrated clock offset.
func TestRemoteJogTransportDelay(t *testing.T) {
	clock := remoteJogClock{sent: 1000000000, binding: JogClockBinding{Host: JogClockIncarnation{1}},
		limits: JogClockLimits{Qualified: true, MaxInputAgeNS: 750000000, MaxInputDurationNS: 750000000,
			MaxCalibrationAgeNS: 10000000000, MaxUncertaintyNS: 600000000, DriftPPB: 1}}
	source := JogClockIncarnation{2}
	// Inject 120 ms host-to-source and 280 ms source-to-host calibration delay.
	if err := clock.calibrate(jogCalibration{Type: "calibrate", Seq: 1, SourceIncarnation: hex.EncodeToString(source[:]), SourceNS: 6120000000, HostTXNS: 1000000000}, 1400000000); err != nil {
		t.Fatal(err)
	}
	if clock.mapping.lo != -5120000000 || clock.mapping.hi != -4720000000 {
		t.Fatalf("offset=[%d,%d]", clock.mapping.lo, clock.mapping.hi)
	}
	for _, delay := range []uint64{120000000, 400000000, 240000000, 310000000, 630000000, 800000000} {
		frame := ipcclient.LocalJogUpdate{ClockIncarnation: [16]byte(source), SourceSequence: 1, OriginHostNs: 7000000000, DeadlineHostNs: 7750000000}
		converted, admission, err := clock.convert(frame, 2000000000+delay, 6000000000)
		if delay >= 630000000 {
			if !errors.Is(err, ErrJogClockInputStale) || jogIngressReason(err) != ipcclient.JogReasonExpired {
				t.Fatalf("delay=%d refusal=%v", delay, err)
			}
			continue
		}
		if err != nil {
			t.Fatalf("delay=%d: %v", delay, err)
		}
		// Absolute expected age includes 120 ms calibration uncertainty and 1 ns drift.
		if admission.WorstCaseAgeNS() != delay+120000001 || converted.OriginHostNs != 1879999999 || converted.DeadlineHostNs != 2629999999 {
			t.Fatalf("delay=%d age=%d frame=%+v", delay, admission.WorstCaseAgeNS(), converted)
		}
		if converted.ClockIncarnation != [16]byte(clock.binding.Host) {
			t.Fatal("source clock reached native")
		}
	}
	t.Log("120/240/310/400 ms input delay admitted with 240000001/360000001/430000001/520000001 ns conservative age; 630/800 ms refused jog_input_too_old")
}
