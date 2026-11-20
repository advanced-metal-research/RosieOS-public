//go:build linux

// Package publisher observes the public API without authority and bounds NATS
// backpressure independently of sampling; unavailable evidence remains unknown.
package publisher

import (
	"context"
	"errors"
	"io"
	"net"
	"os"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"golang.org/x/sys/unix"
	contract "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// Measured deployment cadence: a Rosie 1430 cell's manifest sets
// ROBOT_V4_RTCORE_NATS_BATCH_MS=500; follow the fleet rather than the bridge default.
const DefaultCadence = 500 * time.Millisecond

// Unverified bounds: one pending sample, 500 ms I/O/source age, reconnect from
// 250 ms to 4000 ms. These budgets never extend native leases or motion authority.
const (
	queueCapacity    = 1
	operationTimeout = 500 * time.Millisecond
	maxSourceAge     = 500 * time.Millisecond
	reconnectMin     = 250 * time.Millisecond
	reconnectMax     = 4 * time.Second
)

var (
	ErrContract             = errors.New("describe does not match the generated contract")
	ErrAuthRequired         = errors.New("NATS authentication requires explicit provisioning")
	ErrTLSRequired          = errors.New("NATS TLS requires explicit provisioning")
	ErrJetStreamUnavailable = errors.New("JetStream is unavailable for this endpoint or account")
	ErrStreamConfig         = errors.New("JetStream stream configuration is incompatible or refused")
	ErrPublisherActive      = errors.New("another publisher is active on the deployment status subject")
)

func permanent(err error) bool {
	var mismatch *ipcclient.TelemetryLayoutMismatchError
	if errors.As(err, &mismatch) {
		return true
	}
	for _, sentinel := range []error{ErrContract, ErrAuthRequired, ErrTLSRequired, ErrJetStreamUnavailable, ErrStreamConfig, ErrPublisherActive} {
		if errors.Is(err, sentinel) {
			return true
		}
	}
	return false
}

// Observer deliberately excludes grant-bearing operations; SDK clients satisfy
// this interface without acquiring, renewing or releasing a session.
type Observer interface {
	Describe(context.Context) (control.Description, error)
	Status(context.Context) (control.Status, error)
}

type Config struct {
	URL, Host         string
	Cadence           time.Duration
	TelemetryCadence  time.Duration
	TelemetryMaxBytes int64
}

// TopicToken implements v4 bridge_commands.go:2139, preserving subject spelling.
func TopicToken(value string) string {
	var b strings.Builder
	for _, r := range strings.TrimSpace(value) {
		if r >= 'a' && r <= 'z' || r >= 'A' && r <= 'Z' || r >= '0' && r <= '9' || r == '-' || r == '_' {
			b.WriteRune(r)
		} else if b.Len() > 0 {
			b.WriteByte('-')
		}
	}
	token := strings.Trim(b.String(), "-_")
	if token == "" {
		return "unknown"
	}
	return token
}

// Validate requires a concrete deployment identity; an absent role mapping must
// not silently publish under the local OS hostname or the v4 "unknown" token.
func (c Config) Validate() error {
	if c.TelemetryCadence < 0 || c.TelemetryCadence > 5*time.Second || c.TelemetryCadence%time.Millisecond != 0 {
		return errors.New("telemetry cadence must be whole milliseconds from 1 through 5000 (zero selects status cadence)")
	}
	if c.TelemetryMaxBytes < 0 || (c.TelemetryMaxBytes > 0 && c.TelemetryMaxBytes < ipcclient.TelemetryBatchHeaderV1Bytes+ipcclient.CycleCaptureRecordV2Bytes) {
		return errors.New("telemetry retention must hold at least one header and record")
	}
	// Unverified configuration bound: whole milliseconds, at most 5000 ms to
	// stay within the v4 pair reader's measured status freshness window.
	if c.Cadence < 0 || c.Cadence > 5*time.Second || c.Cadence%time.Millisecond != 0 {
		return errors.New("cadence must be whole milliseconds from 1 through 5000 (zero selects the default)")
	}
	if err := ValidateURL(c.URL); err != nil {
		return err
	}
	host := strings.TrimSpace(c.Host)
	if host == "" || len(host) > 253 || TopicToken(host) == "unknown" || strings.HasPrefix(host, "-") || strings.ContainsAny(host, "@/?#=,\\\"' \t\r\n*>") {
		return errors.New("concrete deployment host is required")
	}
	if strings.Contains(host, ":") {
		address := strings.Trim(host, "[]")
		if zone := strings.LastIndex(address, "%"); zone >= 0 {
			address = address[:zone]
		}
		if net.ParseIP(address) == nil {
			return errors.New("invalid deployment peer address")
		}
	}
	semantic := strings.TrimSuffix(strings.TrimRight(strings.ToLower(host), "."), ".local")
	switch semantic {
	case "manifest", "service", "motion-server", "realtime-core", "steamdeck-teach-pendent", "build-server", "motion-authority", "servo-authority", "human-client-ui", "os-arch-build-server", "builder-dashboard-artifact-node", "rtcore", "localhost":
		return errors.New("host must be a concrete deployment peer, not a role")
	}
	return nil
}

type Publisher struct {
	observer                          Observer
	config                            Config
	dropped                           atomic.Uint64
	mu                                sync.Mutex
	log                               io.Writer
	transport, sourceError, lastError string
	dropping                          bool
	reconnects                        uint64
}

// New validates configuration before network activity; Run is single-use and
// its observer remains owned by the caller, preventing hidden session cleanup.
func New(observer Observer, config Config) (*Publisher, error) {
	if observer == nil {
		return nil, errors.New("observer is required")
	}
	if err := config.Validate(); err != nil {
		return nil, err
	}
	config.Host = strings.TrimSpace(config.Host)
	if config.Cadence == 0 {
		config.Cadence = DefaultCadence
	}
	if config.TelemetryCadence == 0 {
		config.TelemetryCadence = config.Cadence
	}
	if config.TelemetryMaxBytes == 0 {
		config.TelemetryMaxBytes = DefaultTelemetryMaxBytes
	}
	return &Publisher{observer: observer, config: config, log: os.Stderr}, nil
}

// Dropped counts samples replaced, rejected or not fully acknowledged, including
// uncertain delivery; an acknowledgement loss can already be present in archives.
func (p *Publisher) Dropped() uint64 { return p.dropped.Load() }

type snapshot struct {
	description control.Description
	status      control.Status
	sequence    uint64
	updated     time.Time
}

func sourceAge(ns uint64) (time.Duration, error) {
	var ts unix.Timespec
	if err := unix.ClockGettime(unix.CLOCK_MONOTONIC, &ts); err != nil {
		return 0, err
	}
	now := uint64(ts.Nano())
	if ns == 0 || ns > now || now-ns > uint64(maxSourceAge) {
		return 0, errors.New("native status is stale or has an invalid clock")
	}
	return time.Duration(now - ns), nil
}

func validDescription(d control.Description) bool {
	return d.ContractVersion == contract.ContractVersion && d.CapabilitiesDigest == contract.CapabilitiesDigest && d.ProtocolMajor == ipcclient.VersionMajor && d.ProtocolMinor == ipcclient.VersionMinor
}

func (p *Publisher) sample(ctx context.Context, sequence uint64) (snapshot, error) {
	ctx, cancel := context.WithTimeout(ctx, operationTimeout)
	defer cancel()
	d, err := p.observer.Describe(ctx)
	if err != nil {
		return snapshot{}, err
	}
	if !validDescription(d) {
		return snapshot{}, ErrContract
	}
	if d.Backend != "simulation" && d.Backend != "ethercat" {
		return snapshot{}, errors.New("unknown runtime backend")
	}
	s, err := p.observer.Status(ctx)
	if err != nil {
		return snapshot{}, err
	}
	if s.Core.ConfigurationSHA256 != d.ConfigurationSHA256 || s.Core.Backend != d.Backend || len(s.Core.Axes) != len(d.Axes) || len(d.Axes) == 0 || len(d.Axes) > ipcclient.MaxAxes {
		return snapshot{}, errors.New("status and describe identity differ")
	}
	age, err := sourceAge(s.Core.TimeNS)
	if err != nil {
		return snapshot{}, err
	}
	return snapshot{d, s, sequence, time.Now().Add(-age)}, nil
}

// Run polls describe/status every cadence, rechecking digests across reconnects.
// It never refreshes stale native evidence with a new wall timestamp. Cancellation
// joins the network worker; permanent refusals reach the CLI instead of retrying silently.
func (p *Publisher) Run(ctx context.Context) (result error) {
	ctx, cancel := context.WithCancel(ctx)
	defer cancel()
	if observer, ok := p.observer.(TelemetryObserver); ok {
		telemetryDone := make(chan error, 1)
		go func() { telemetryDone <- p.sendTelemetry(ctx, observer); cancel() }()
		defer func() {
			cancel()
			if err := <-telemetryDone; result == nil {
				result = err
			}
		}()
	}
	queue := make(chan snapshot, queueCapacity)
	done := make(chan struct{})
	failure := make(chan error, 1)
	p.setTransport("connecting", nil)
	go func() { defer close(done); failure <- p.send(ctx, queue); cancel() }()
	defer func() {
		cancel()
		<-done
		if err := <-failure; result == nil {
			result = err
		}
		if result != nil {
			p.setTransport("refused", result)
		} else {
			p.setTransport("stopped", nil)
		}
	}()
	ticker := time.NewTicker(p.config.Cadence)
	defer ticker.Stop()
	for sequence := uint64(1); ; sequence++ {
		s, err := p.sample(ctx, sequence)
		if errors.Is(err, ErrContract) {
			return err
		}
		if ctx.Err() != nil {
			return nil
		}
		if err != nil {
			p.drop(err.Error())
		}
		p.setSource(err)
		select {
		case queue <- s:
		default:
			select {
			case old := <-queue:
				if old.status.Core.TimeNS != 0 {
					p.drop("pending sample replaced")
				}
			default:
			}
			queue <- s // Single producer; removing the old sample guarantees space.
		}
		select {
		case <-ctx.Done():
			return nil
		case <-ticker.C:
		}
	}
}

func (p *Publisher) send(ctx context.Context, queue <-chan snapshot) error {
	var conn *natsConn
	defer func() {
		if conn != nil {
			conn.close()
		}
	}()
	backoff := reconnectMin
	attempted := false
	for {
		if ctx.Err() != nil {
			return nil
		}
		if conn == nil {
			if attempted {
				p.mu.Lock()
				p.reconnects++
				p.mu.Unlock()
			}
			attempted = true
			var err error
			conn, err = dialNATS(ctx, p.config.URL)
			stage := "dial_failed"
			if conn == nil && permanent(err) {
				stage = "handshake_refused"
			}
			if err == nil {
				stage = "stream_rejected"
				err = conn.ensureStream()
			}
			if err == nil {
				p.setTransport("checking_publisher", nil)
				stage = "publisher_check_failed"
				err = conn.checkPublisher(subject(p.config.Host, "status"), p.config.Cadence)
			}
			if err != nil {
				if conn != nil {
					conn.close()
					conn = nil
				}
				if ctx.Err() != nil {
					return nil
				}
				p.setTransport(stage, err)
				if permanent(err) {
					return err
				}
				if !delay(ctx, backoff) {
					return nil
				}
				backoff = min(backoff*2, reconnectMax)
				continue
			}
			p.setTransport("publishing", nil)
		}
		var s snapshot
		select {
		case <-ctx.Done():
			return nil
		case s = <-queue:
		}
		if s.status.Core.TimeNS != 0 {
			if _, err := sourceAge(s.status.Core.TimeNS); err != nil {
				p.drop(err.Error())
				s = snapshot{}
			}
		}
		before := p.Dropped()
		status, batch, err := payloads(p.config.Host, s, p.diagnostics(), p.config.Cadence)
		if err == nil {
			err = conn.publish(subject(p.config.Host, "status"), status)
		}
		if err == nil && batch != nil {
			if _, err = sourceAge(s.status.Core.TimeNS); err == nil {
				err = conn.publish(subject(p.config.Host, "telemetry.batch"), batch)
			}
		}
		if err != nil {
			if ctx.Err() != nil {
				return nil
			}
			if s.status.Core.TimeNS != 0 {
				p.drop(err.Error())
			}
			p.setTransport("publish_failed", err)
			if permanent(err) {
				return err
			}
			conn.close()
			conn = nil
			if !delay(ctx, backoff) {
				return nil
			}
			backoff = min(backoff*2, reconnectMax)
		} else {
			backoff = reconnectMin
			if batch != nil {
				p.recovered(before)
			}
		}
	}
}
func subject(host, suffix string) string { return "robot/v4/rtcore." + TopicToken(host) + "." + suffix }
func delay(ctx context.Context, d time.Duration) bool {
	timer := time.NewTimer(d)
	defer timer.Stop()
	select {
	case <-ctx.Done():
		return false
	case <-timer.C:
		return true
	}
}
