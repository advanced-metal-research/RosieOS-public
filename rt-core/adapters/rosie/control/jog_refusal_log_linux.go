//go:build linux

package control

import (
	"fmt"
	"log"
	"runtime"
	"time"

	"rosieos/rt-core/ipcclient"
)

// Unverified journal budget: 50 refusal lines per second. Producers send
// 20-50 inputs/s, so a hold whose every input is refused is still logged line
// by line; a faster flood is summarised by its suppressed count, never dropped
// without trace.
const jogRefusalLogPerSecond = 50

// Unverified transient retry window: 5 ms. Every writer that can hold a jog
// publication lock holds it for one shared-memory publish, and native
// rewrites an observation slot for microseconds once per cycle; 5 ms spans
// several cycles yet returns the stream reader long before the producer's
// next input (20-50 ms). The input's own deadline bounds the window first.
const jogTransientRetryNS = uint64(5 * time.Millisecond)

// jogRefusal is what a transport knows about one refused input beyond its
// sequence and reason. The journal line is the only record that survives an
// rt-control restart: /v1/jog/ingress counters are per process (a
// Rosie 1430 arm, 2026-10-10: the 22:01:13 refusal's cause was lost with
// its process).
type jogRefusal struct {
	transport  string
	generation uint64 // Jog generation the input named; 0 = the current session's.
	deadline   uint64 // The input's host deadline; 0 = none (setup, chunks).
	cause      error
	attempts   int
}

// jogRefusalLimiter is the Controller's journal budget; c.mu protects it.
type jogRefusalLimiter struct {
	windowStart uint64
	lines       int
	suppressed  uint64
}

var jogReasonNames = [...]string{
	ipcclient.JogReasonNone: "none", ipcclient.JogReasonClosed: "closed", ipcclient.JogReasonModeConflict: "mode_conflict",
	ipcclient.JogReasonNotReady: "not_ready", ipcclient.JogReasonWrongIdentity: "wrong_identity",
	ipcclient.JogReasonInvalidGeneration: "invalid_generation", ipcclient.JogReasonStaleSequence: "stale_sequence",
	ipcclient.JogReasonInvalidVector: "invalid_vector", ipcclient.JogReasonInvalidTiming: "invalid_timing",
	ipcclient.JogReasonExpired: "expired", ipcclient.JogReasonAuthorityLost: "authority_lost",
	ipcclient.JogReasonClockReset: "clock_reset", ipcclient.JogReasonGenerationExhausted: "generation_exhausted",
	ipcclient.JogReasonRamping: "ramping", ipcclient.JogReasonLimited: "limited",
	ipcclient.JogReasonOutsideLimitsOutward: "outside_limits_outward", ipcclient.JogReasonPathStartMismatch: "path_start_mismatch",
	ipcclient.JogReasonPathCapacity: "path_capacity", ipcclient.JogReasonPathOutOfOrder: "path_out_of_order",
	ipcclient.JogReasonPathInvalidSample: "path_invalid_sample", ipcclient.JogReasonPathEnded: "path_ended",
	ipcclient.JogReasonPathFault: "path_fault", ipcclient.JogReasonPathUnavailable: "path_unavailable",
	ipcclient.JogReasonPathInvalidConfig: "path_invalid_config",
}

func jogReasonName(reason uint32) string {
	if reason < uint32(len(jogReasonNames)) {
		return jogReasonNames[reason]
	}
	return fmt.Sprintf("unknown_%d", reason)
}

// jogTransient is true only for a refusal caused by a busy publication lock or
// observation slot, adapter or client side. It never covers a native decision
// or an identity, sequence, timing, authority or readiness refusal.
func jogTransient(err error) bool {
	return err != nil && (ipcclient.TransientJogRefusal(err) || err.Error() == ReasonJogPublisherBusy)
}

// retryTransientJog repeats attempt while it fails only transiently, until the
// retry window or the input's deadline. Each attempt reruns every admission
// check; nothing is published by a failed attempt, so nothing is replayed.
func retryTransientJog(deadline uint64, attempt func() (uint32, error)) (uint32, error, int) {
	limit := ipcclient.HostMonotonicNS() + jogTransientRetryNS
	if deadline < limit {
		limit = deadline
	}
	for attempts := 1; ; attempts++ {
		reason, err := attempt()
		if reason == ipcclient.JogReasonNone || !jogTransient(err) || ipcclient.HostMonotonicNS() >= limit {
			return reason, err, attempts
		}
		jogRetryPause(attempts)
	}
}

// jogStatusWithinRetry is JogStatus retrying only a busy observation slot,
// within the transient window. Kept apart from retryTransientJog so the
// adapter's private publication strings never share its error path.
func (c *Controller) jogStatusWithinRetry() (ipcclient.JogObservation, error) {
	limit := ipcclient.HostMonotonicNS() + jogTransientRetryNS
	for attempts := 1; ; attempts++ {
		observation, err := c.JogStatus()
		if err == nil || !ipcclient.TransientJogRefusal(err) || ipcclient.HostMonotonicNS() >= limit {
			return observation, err
		}
		jogRetryPause(attempts)
	}
}

// jogRetryPause yields for the first attempts (collisions last microseconds),
// then sleeps briefly so a longer one does not spin a core on the servo host.
func jogRetryPause(attempts int) {
	if attempts < 8 {
		runtime.Gosched()
	} else {
		time.Sleep(100 * time.Microsecond)
	}
}

// admitJogRefusalLogLocked applies the journal budget. Caller holds c.mu.
func (c *Controller) admitJogRefusalLogLocked(now uint64) (bool, uint64) {
	l := &c.jogRefusalLog
	if now-l.windowStart >= uint64(time.Second) {
		l.windowStart, l.lines = now, 0
	}
	if l.lines >= jogRefusalLogPerSecond {
		l.suppressed++
		return false, 0
	}
	l.lines++
	suppressed := l.suppressed
	l.suppressed = 0
	return true, suppressed
}

func logJogRefusal(sequence uint64, reason uint32, d jogRefusal, grantGeneration, jogGeneration, refused, suppressed, now uint64) {
	transport := d.transport
	if transport == "" {
		transport = "unknown"
	}
	deadline := "none"
	if d.deadline != 0 {
		deadline = fmt.Sprintf("%.3f", (float64(d.deadline)-float64(now))/1e6)
	}
	cause := ""
	if d.cause != nil {
		cause = d.cause.Error()
	}
	log.Printf("rt-control: jog refusal transport=%s reason=%d name=%s transient=%t attempts=%d grant_generation=%d jog_generation=%d sequence=%d deadline_left_ms=%s cause=%q refused_total=%d suppressed=%d",
		transport, reason, jogReasonName(reason), jogTransient(d.cause), max(d.attempts, 1), grantGeneration, jogGeneration,
		sequence, deadline, cause, refused, suppressed)
}
