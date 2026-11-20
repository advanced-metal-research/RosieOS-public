//go:build linux

package control

import (
	"bufio"
	"context"
	"crypto/tls"
	"errors"
	"io"
	"net"
	"net/http"
	"strings"
	"sync"
	"sync/atomic"
	"time"
)

// ErrWarmConnection marks a lost reservation before transmission. The bounded
// reopen path may replace it without reporting uncertain motion to callers.
var ErrWarmConnection = errors.New("control warm connection unavailable")

type warmConnectionKey struct{}

// observationHTTP holds one reservation until its body closes. Streaming reads
// must not secretly occupy the socket offered to a concurrent status request.
func (c *Client) observationHTTP(request *http.Request) (*http.Response, error) {
	return c.reservedObservationHTTP(request, c.warmLanes[c.control], false)
}

func (c *Client) reservedObservationHTTP(request *http.Request, slots chan *http.Client, telemetry bool) (*http.Response, error) {
	budget := c.requestBudget()
	queue, cancelQueue := context.WithTimeout(request.Context(), 2*budget)
	defer cancelQueue()
	overall, _ := queue.Deadline()
	var pool *http.Client
	select {
	case pool = <-slots:
	case <-queue.Done():
		return nil, queue.Err()
	}
	release := func(complete bool) {
		c.mu.Lock()
		broken := !telemetry && c.warmReady && !complete
		if broken {
			c.warmBroken[pool] = slots
		}
		c.mu.Unlock()
		if !broken {
			slots <- pool
		} else {
			c.wakeKeepAlive()
		}
	}
	var uncertain bool
	for attempt := 0; ; attempt++ {
		ctx, cancel := context.WithCancel(request.Context())
		if attempt != 0 || telemetry {
			ctx = context.WithValue(ctx, warmConnectionKey{}, true)
		}
		r := request.Clone(ctx)
		deadline := time.Now().Add(budget)
		if overall.Before(deadline) {
			deadline = overall
		}
		var expired atomic.Bool
		timer := time.AfterFunc(time.Until(deadline), func() { expired.Store(true); cancel() })
		progress := traceRequest(r, deadline)
		response, err := pool.Do(r)
		timer.Stop()
		uncertain = uncertain || progress.sent()
		if err == nil && !expired.Load() {
			// Streams retain caller lifetime, but every body read has a negotiated
			// idle deadline; a silent peer cannot retain an observation reservation.
			response.Body = &reservedBody{ReadCloser: response.Body, beforeRead: func() {
				d := time.Now().Add(budget)
				if parent, ok := request.Context().Deadline(); ok && parent.Before(d) {
					d = parent
				}
				_ = progress.wire.SetReadDeadline(d)
			}, release: func(complete bool) {
				progress.finish(!complete)
				cancel()
				release(complete)
			}}
			return response, nil
		}
		if response != nil {
			response.Body.Close()
		}
		progress.finish(true)
		timedOut := expired.Load() || !time.Now().Before(deadline)
		cancel()
		if timedOut {
			err = errors.Join(ErrRequestTimeout, context.DeadlineExceeded, err)
		}
		if attempt == 0 && request.Context().Err() == nil && time.Now().Before(overall) && (timedOut || connectionClosed(err)) {
			pool.CloseIdleConnections()
			c.connectionReopened(request.Method, request.URL.Path)
			continue
		}
		release(false)
		if !uncertain {
			return nil, errors.Join(ErrConnectionUnsent, request.Context().Err(), err)
		}
		if timedOut || errors.Is(request.Context().Err(), context.DeadlineExceeded) {
			return nil, errors.Join(context.DeadlineExceeded, err)
		}
		return nil, errors.Join(ErrTransport, err)
	}
}

// reservedBody returns fully consumed HTTP responses to their lane and
// quarantines interrupted streams until the keeper repairs their connection.
type reservedBody struct {
	io.ReadCloser
	release    func(bool)
	beforeRead func()
	complete   bool
	once       sync.Once
}

func (b *reservedBody) Read(p []byte) (int, error) {
	if b.beforeRead != nil {
		b.beforeRead()
	}
	n, err := b.ReadCloser.Read(p)
	if err == io.EOF {
		b.complete = true
	}
	return n, err
}

func (b *reservedBody) Close() (err error) {
	b.once.Do(func() {
		err = b.ReadCloser.Close()
		b.release(b.complete && err == nil)
	})
	return err
}

// warmConnections reserves every lane before Acquire. During renewal it touches
// only idle slots, so an upload or Home cannot delay Stop or the lease receipt.
func (c *Client) warmConnections(ctx context.Context, initial bool, snapshot ...*AcquisitionSnapshot) error {
	ctx, cancel := context.WithTimeout(ctx, 2*c.requestBudget())
	defer cancel()
	if err := c.warmMu.LockContext(ctx); err != nil {
		return err
	}
	defer c.warmMu.Unlock()
	ctx = context.WithValue(ctx, warmConnectionKey{}, true)
	c.mu.Lock()
	c.warmReady = true
	broken := make(map[*http.Client]chan *http.Client, len(c.warmBroken))
	for pool, slots := range c.warmBroken {
		broken[pool] = slots
	}
	c.mu.Unlock()
	// A cancelled HTTP/1 request loses its socket. Quarantine that slot until
	// keeper repair, so the single retry selects another verified warm socket.
	for pool, slots := range broken {
		if _, err := c.call(ctx, pool, "GET", "/v1/jog/ingress", nil, nil, true); err == nil {
			c.mu.Lock()
			delete(c.warmBroken, pool)
			c.mu.Unlock()
			slots <- pool
		} else if initial {
			return err
		}
	}
	var wg sync.WaitGroup
	// Unverified reservation cap: six HTTP slots, renewal and a jog socket.
	errs := make(chan error, 8)
	if strings.HasPrefix(c.base, "https://") {
		wg.Add(1)
		go func() { defer wg.Done(); errs <- c.warmJogConnection(ctx) }()
	}
	for _, slots := range c.warmLanes {
		available := len(slots)
		if initial {
			available = cap(slots)
		}
		// Hold each selected slot until selection finishes; the same socket must
		// not satisfy more than one reservation in a warm-up sweep.
		selected := make([]*http.Client, 0, available)
	selectSlots:
		for i := 0; i < available; i++ {
			if !initial {
				select {
				case pool := <-slots:
					selected = append(selected, pool)
				default:
					break selectSlots
				}
				continue
			}
			select {
			case pool := <-slots:
				selected = append(selected, pool)
			case <-ctx.Done():
				for _, pool := range selected {
					slots <- pool
				}
				wg.Wait()
				return ctx.Err()
			}
		}
		for i, pool := range selected {
			path := "/v1/describe"
			if !initial {
				path = "/v1/jog/ingress"
			}
			if len(snapshot) != 0 && slots == c.warmLanes[c.control] {
				if i == 0 {
					path = "/v1/status"
				}
				if i == 1 {
					path = "/v1/events?after=0"
				}
			}
			wg.Add(1)
			go func(pool *http.Client, slots chan *http.Client, path string) {
				defer wg.Done()
				response, err := c.call(ctx, pool, "GET", path, nil, nil, true)
				if path == "/v1/status" {
					snapshot[0].Status, err = data[Status](response, err)
				}
				if path == "/v1/events?after=0" {
					snapshot[0].Events, err = data[EventBatch](response, err)
				}
				if err != nil && !initial {
					c.mu.Lock()
					c.warmBroken[pool] = slots
					c.mu.Unlock()
				} else {
					slots <- pool
				}
				errs <- err
			}(pool, slots, path)
		}
	}
	wg.Wait()
	close(errs)
	var result error
	for err := range errs {
		result = errors.Join(result, err)
	}
	return result
}

// AcquisitionSnapshot binds fresh incarnation and event cursor observations to
// connection preparation, preventing separate serial preflight round trips.
type AcquisitionSnapshot struct {
	Status Status
	Events EventBatch
}

// PrepareAcquisition warms all remote lanes using the required status and event
// reads as two of the warm-up requests. It grants no authority and must precede
// the caller's serialized Acquire; failed preparation cannot qualify a lease.
func (c *Client) PrepareAcquisition(ctx context.Context) (AcquisitionSnapshot, error) {
	c.mu.Lock()
	remote := c.roundTrip > lanRoundTripBound
	c.mu.Unlock()
	var snapshot AcquisitionSnapshot
	var err error
	if remote {
		err = c.warmConnections(ctx, true, &snapshot)
	} else {
		snapshot.Status, err = c.Status(ctx)
		if err == nil {
			snapshot.Events, err = c.EventsContext(ctx, 0)
		}
	}
	c.mu.Lock()
	c.acquisitionPrepared = err == nil && remote
	c.mu.Unlock()
	return snapshot, err
}

// warmJogConnection retains a verified HTTP socket for the next WSS upgrade.
// Acquisition and maintenance call it; fresh jog input never initiates TLS.
func (c *Client) warmJogConnection(ctx context.Context) error {
	ctx, cancel := context.WithTimeout(ctx, 2*c.requestBudget())
	defer cancel()
	if err := c.jogSpareMu.LockContext(ctx); err != nil {
		return err
	}
	defer c.jogSpareMu.Unlock()
	err := c.probeJogConnection(ctx)
	if (connectionClosed(err) || errors.Is(err, ErrRequestTimeout)) && ctx.Err() == nil {
		c.connectionReopened("GET", "/v1/jog/ingress")
		err = c.probeJogConnection(ctx)
	}
	return err
}

// takeWarmJogConnection consumes a reservation during setup, before any WSS
// upgrade, native Begin or velocity input. Each ended WSS stream is single-use;
// a second button press must not wait for the idle keeper's next periodic sweep.
// Missing reservations are prepared within the caller's existing setup budget.
// Failure here proves no motion request was sent and cannot justify replaying it.
func (c *Client) takeWarmJogConnection(ctx context.Context) (net.Conn, error) {
	if err := c.jogSpareMu.LockContext(ctx); err != nil {
		return nil, errors.Join(ErrConnectionUnsent, err)
	}
	defer c.jogSpareMu.Unlock()
	c.mu.Lock()
	closed := c.closed
	c.mu.Unlock()
	if closed {
		return nil, errors.Join(ErrConnectionUnsent, ErrClosed)
	}
	if c.jogSpare == nil {
		if err := c.probeJogConnection(ctx); err != nil {
			return nil, errors.Join(ErrConnectionUnsent, ctx.Err(), err)
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, errors.Join(ErrConnectionUnsent, err)
	}
	conn := c.jogSpare
	c.jogSpare = nil
	// Replenish for the next hold now, independently of motion/lease renewal.
	c.wakeKeepAlive()
	return conn, nil
}

func (c *Client) probeJogConnection(ctx context.Context) error {
	ctx, cancel := context.WithTimeout(ctx, c.requestBudget())
	defer cancel()
	r, _ := http.NewRequestWithContext(ctx, "GET", c.base+"/v1/jog/ingress", nil)
	if c.jogSpare == nil {
		address := r.URL.Host
		if r.URL.Port() == "" {
			address = net.JoinHostPort(r.URL.Hostname(), "443")
		}
		dialer := tls.Dialer{Config: c.control.Transport.(*http.Transport).TLSClientConfig.Clone()}
		conn, err := dialer.DialContext(ctx, "tcp", address)
		if err != nil {
			return err
		}
		c.jogSpare = conn
	}
	conn := c.jogSpare
	stop := context.AfterFunc(ctx, func() { conn.Close() })
	defer stop()
	if d, ok := ctx.Deadline(); ok {
		conn.SetDeadline(d)
	}
	err := r.Write(conn)
	if err == nil {
		var response *http.Response
		response, err = http.ReadResponse(bufio.NewReader(conn), r)
		if err == nil {
			_, err = io.Copy(io.Discard, response.Body)
			response.Body.Close()
			if response.Close || response.StatusCode != http.StatusOK {
				err = ErrProtocol
			}
		}
	}
	if err != nil {
		conn.Close()
		c.jogSpare = nil
		if errors.Is(ctx.Err(), context.DeadlineExceeded) || deadlineExpired(ctx) {
			return errors.Join(ErrRequestTimeout, context.DeadlineExceeded, err)
		}
		return err
	}
	conn.SetDeadline(time.Time{})
	return nil
}
