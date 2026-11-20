//go:build linux

package control

import (
	"bytes"
	"context"
	"crypto/rand"
	"crypto/tls"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net"
	"net/http"
	"net/url"
	"strconv"
	"strings"
	"sync"
	"time"

	"rosieos/rt-core/internal/unixhttp"
	"rosieos/rt-core/ipcclient"
)

var (
	ErrTransport = errors.New("control transport lost; outcome may be unknown")
	ErrClosed    = errors.New("control client closed")
	ErrProtocol  = errors.New("invalid control response")
)

type Rejected struct {
	Reason          string
	NativeResult    *ipcclient.CommandResult
	NativeJogResult *ipcclient.JogObservation
	// ResponseData retains operation-specific refusal evidence without
	// interpreting it or losing JSON numeric precision.
	ResponseData json.RawMessage
}

func (e *Rejected) Error() string { return e.Reason }

type UnixPath string

// Client owns independent lifecycle, urgent and bulk connection pools. Its
// mutex protects local state only; network I/O cannot hold up Stop or renewal.
type Client struct {
	base                           string
	unixPath                       string
	control, urgent, bulk, renewal *http.Client
	maxJogInputAge                 uint64
	roundTrip                      time.Duration
	leaseCeiling                   time.Duration
	mu                             sync.Mutex
	grant                          Grant
	recoveryFence                  Fence
	closed                         bool
	renewCancel                    context.CancelFunc
	renewDone                      chan struct{}
	warmLanes                      map[*http.Client]chan *http.Client
	warmClients                    []*http.Client
	telemetrySlots                 chan *http.Client
	warmBroken                     map[*http.Client]chan *http.Client
	warmReady                      bool
	acquisitionPrepared            bool
	warmMu                         contextMutex
	jogSpareMu                     contextMutex
	jogSpare                       net.Conn
	keepCancel                     context.CancelFunc
	keepDone                       chan struct{}
	keepWake                       chan struct{}
	idleTimeout                    time.Duration
}

// Dial constructs a Unix HTTP client. Connection failures are reported by each
// context-bounded operation, avoiding a hidden acquisition during construction.
func Dial(path UnixPath) (*Client, error) {
	if path == "" {
		return nil, errors.New("empty control socket")
	}
	c := newClient("http://localhost", NewUnixHTTPClient(string(path)).Transport.(*http.Transport))
	c.unixPath = string(path)
	return c, nil
}

// DialTLS requires explicit server trust and client credentials. Disabling
// certificate verification or following redirects could disclose a grant.
// The pointer is cloned to avoid copying tls.Config's synchronization state.
func DialTLS(addr string, config *tls.Config) (*Client, error) {
	if config == nil || config.InsecureSkipVerify || config.RootCAs == nil || (len(config.Certificates) == 0 && config.GetClientCertificate == nil) {
		return nil, fmt.Errorf("%w: verified CA and client certificate required", ErrTransport)
	}
	if !strings.Contains(addr, "://") {
		addr = "https://" + addr
	}
	u, err := url.Parse(addr)
	if err != nil || u.Scheme != "https" || u.Host == "" || u.User != nil || (u.Path != "" && u.Path != "/") || u.RawQuery != "" || u.Fragment != "" {
		return nil, errors.New("remote endpoint must be an HTTPS origin")
	}
	clone := config.Clone()
	if clone.MinVersion < tls.VersionTLS13 {
		clone.MinVersion = tls.VersionTLS13
	}
	clone.NextProtos = []string{"http/1.1"}
	if clone.ClientSessionCache == nil {
		// Unverified cache capacity: one server identity, shared across all lanes.
		clone.ClientSessionCache = tls.NewLRUClientSessionCache(1)
	}
	return newClient(strings.TrimRight(addr, "/"), &http.Transport{TLSClientConfig: clone}), nil
}

func newClient(base string, transport *http.Transport) *Client {
	c := &Client{base: base, keepWake: make(chan struct{}, 1)}
	pool := func() *http.Client {
		t := transport.Clone()
		// Unverified per-slot limit: one request and one retained connection.
		t.MaxIdleConnsPerHost, t.MaxConnsPerHost = 1, 1
		// The connected keeper owns idle retention; an independent timer could
		// discard a reservation between maintenance and an operation.
		t.IdleConnTimeout = 0
		dial := t.DialContext
		if dial == nil {
			dial = (&net.Dialer{}).DialContext
		}
		t.DialContext = func(ctx context.Context, network, address string) (net.Conn, error) {
			// net/http may detach a queued dial from the request's cancellation.
			// Bound that dial and the TLS reads before GotConn installs its deadline.
			budget := c.requestBudget()
			ctx, cancel := context.WithTimeout(ctx, budget)
			defer cancel()
			c.mu.Lock()
			guard := c.warmReady
			c.mu.Unlock()
			if guard && ctx.Value(warmConnectionKey{}) == nil {
				return nil, ErrWarmConnection
			}
			conn, err := dial(ctx, network, address)
			if err != nil {
				return nil, err
			}
			if err := conn.SetDeadline(time.Now().Add(budget)); err != nil {
				conn.Close()
				return nil, err
			}
			return &countedConnection{Conn: conn}, nil
		}
		return &http.Client{Transport: t, CheckRedirect: func(*http.Request, []*http.Request) error { return http.ErrUseLastResponse }}
	}
	c.control, c.urgent, c.bulk, c.renewal = pool(), pool(), pool(), pool()
	// Read-only captures must not hold a reservation required by Acquire.
	c.telemetrySlots = make(chan *http.Client, 2)
	for i := 0; i < cap(c.telemetrySlots); i++ {
		client := pool()
		c.telemetrySlots <- client
		c.warmClients = append(c.warmClients, client)
	}
	c.warmLanes = make(map[*http.Client]chan *http.Client)
	c.warmBroken = make(map[*http.Client]chan *http.Client)
	for _, lane := range []*http.Client{c.control, c.urgent, c.bulk, c.renewal} {
		// Unverified concurrency reservation: four status/operation slots plus
		// independent urgent, bulk and renewal connections. Composed tests count use.
		size := 1
		if lane == c.control {
			size = 4
		}
		slots := make(chan *http.Client, size)
		slots <- lane
		for i := 1; i < size; i++ {
			client := pool()
			c.warmClients = append(c.warmClients, client)
			slots <- client
		}
		c.warmLanes[lane] = slots
	}
	return c
}

// SetFence resumes a shell-acquired session without granting or renewing it.
// Call before starting concurrent work; an explicit replacement invalidates the
// local grant metadata and cannot silently reuse another session's lease.
func (c *Client) SetFence(f Fence) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.grant = Grant{Session: f.Session, Generation: f.Generation}
	c.recoveryFence = Fence{}
}

func (c *Client) Grant() Grant { c.mu.Lock(); defer c.mu.Unlock(); return c.grant }
func (c *Client) Fence() Fence {
	g := c.Grant()
	return Fence{Session: g.Session, Generation: g.Generation}
}

// NewRequestID returns 16 random bytes as hexadecimal, preventing accidental
// reuse between independent mutations in the adapter's session retry table.
func NewRequestID() (string, error) {
	var b [16]byte
	if _, err := rand.Read(b[:]); err != nil {
		return "", err
	}
	return hex.EncodeToString(b[:]), nil
}

// Command fills the ID and fence once, then retains them in request. Retry a
// cancelled/lost reply with the same request and a fresh context, never a new ID.
// Requests must not be shared concurrently or modified between retry attempts.
// The adapter detaches outcomes on release/expiry and has a bounded retry table;
// retries are only safe within that live session's retained outcome window.
func (c *Client) Command(ctx context.Context, request *Request) (Response, error) {
	if request == nil {
		return Response{}, errors.New("nil control request")
	}
	if request.RequestID == nil {
		id, err := NewRequestID()
		if err != nil {
			return Response{}, err
		}
		request.RequestID = &id
	}
	if request.Operation != "acquire" && request.Fence == (Fence{}) {
		request.Fence = c.Fence()
		if request.Fence.Session == "" && (request.Operation == "stop" || request.Operation == "release") {
			c.mu.Lock()
			request.Fence = c.recoveryFence
			c.mu.Unlock()
		}
	}
	request.Schema = "rosie.rt-control.request.v1"
	body, err := json.Marshal(request)
	if err != nil {
		return Response{}, err
	}
	pool := c.control
	switch request.Operation {
	case "acquire":
		c.mu.Lock()
		remote := c.roundTrip > lanRoundTripBound
		c.mu.Unlock()
		// Remote Acquire warms the keeper's dedicated connection before authority
		// starts. ARM and status traffic cannot evict or borrow that connection.
		if remote {
			pool = c.renewal
		}
	case "prepare_trajectory":
		pool = c.bulk
	case "stop", "release":
		pool = c.urgent
	case "renew":
		pool = c.renewal
	}
	response, err := c.call(ctx, pool, "POST", "/v1/control", body, nil, request.Operation != "release", c.commandBudget(request.Operation, len(body)))
	// A renewal deadline loses lease evidence; an ordinary request deadline
	// does not. Preserve both typed causes for existing keeper consumers.
	if request.Operation == "renew" && errors.Is(err, context.DeadlineExceeded) {
		err = errors.Join(ErrTransport, err)
	}
	// A submitted Reset retires its adapter grant, including a native refusal.
	// Forget only that acknowledged fence so reacquisition and its keeper cannot
	// silently retain the old session. An unsent/uncertain Reset retains recovery.
	var rejected *Rejected
	if request.Operation == "reset_fault" && response.Operation == request.Operation && response.Sequence != 0 &&
		(err == nil || errors.As(err, &rejected)) {
		c.remember("reset_fault", request.Fence, Grant{})
	}
	if err == nil {
		switch request.Operation {
		case "acquire", "renew", "stop", "release":
			var grant Grant
			if err = json.Unmarshal(response.Data, &grant); err == nil {
				c.remember(request.Operation, request.Fence, grant)
			} else {
				err = errors.Join(ErrProtocol, err)
			}
		}
	}
	return response, err
}

func (c *Client) remember(operation string, sent Fence, grant Grant) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.closed {
		return
	}
	if operation == "reset_fault" {
		if (Fence{Session: c.grant.Session, Generation: c.grant.Generation}) == sent {
			c.grant = Grant{}
			c.recoveryFence = Fence{}
		}
		return
	}
	if operation == "acquire" {
		if c.grant.Session == "" {
			c.grant = grant
			c.recoveryFence = Fence{}
		}
		return
	}
	if operation == "release" && c.recoveryFence == sent {
		c.recoveryFence = Fence{}
		c.grant = Grant{}
		return
	}
	if c.grant.Session != sent.Session {
		return
	}
	if operation == "release" {
		c.grant = Grant{}
		return
	}
	// An acknowledged expired Stop has already retired motion authority. Keep
	// its old fence only for Stop/Release, preventing anonymous recovery Release
	// and preventing late renewal receipts from restoring a usable grant.
	if operation == "stop" && grant.Session == "" {
		c.grant = grant
		c.recoveryFence = sent
		return
	}
	// A late old renewal cannot replace a Stop fence or a newer expiry receipt.
	if grant.Generation == c.grant.Generation && grant.DeadlineHostNS == c.grant.DeadlineHostNS && !c.grant.Stopping && grant.Stopping {
		return
	}
	if grant.Generation > c.grant.Generation || (grant.Generation == c.grant.Generation && grant.DeadlineHostNS >= c.grant.DeadlineHostNS) {
		c.grant = grant
	}
}

func (c *Client) call(ctx context.Context, pool *http.Client, method, path string, body []byte, headers http.Header, retry bool, allowance ...time.Duration) (result Response, callErr error) {
	budget := c.requestBudget()
	if len(allowance) != 0 {
		budget = allowance[0]
	}
	ctx, cancel := context.WithTimeout(ctx, 2*budget)
	defer cancel()
	c.mu.Lock()
	warm := c.warmReady
	c.mu.Unlock()
	if ctx.Value(warmConnectionKey{}) == nil {
		if slots := c.warmLanes[pool]; slots != nil {
			select {
			case pool = <-slots:
				defer func() {
					if warm && (errors.Is(callErr, context.DeadlineExceeded) || errors.Is(callErr, ErrTransport)) {
						c.mu.Lock()
						c.warmBroken[pool] = slots
						c.mu.Unlock()
						c.wakeKeepAlive()
					} else {
						slots <- pool
					}
				}()
			case <-ctx.Done():
				return Response{}, errors.Join(ErrConnectionUnsent, ctx.Err())
			}
		}
	}
	if pool == c.renewal {
		ctx = context.WithValue(ctx, warmConnectionKey{}, true)
	}
	var uncertain bool
	for attempt := 0; ; attempt++ {
		c.mu.Lock()
		closed := c.closed
		c.mu.Unlock()
		if closed {
			return Response{}, ErrClosed
		}
		attemptCtx, cancelAttempt := context.WithTimeout(ctx, budget)
		r, err := http.NewRequestWithContext(attemptCtx, method, c.base+path, bytes.NewReader(body))
		if err != nil {
			cancelAttempt()
			return Response{}, err
		}
		r.Header = headers.Clone()
		if r.Header == nil {
			r.Header = make(http.Header)
		}
		if path == "/v1/control" {
			r.Header.Set("Content-Type", "application/json")
		}
		// Disable net/http's implicit replay: only this bounded retry policy may
		// resend a mutation, and never a program upload or a released session.
		r.GetBody = nil
		progress := traceRequest(r)
		wire, err := pool.Do(r)
		var response Response
		if err == nil {
			// Unverified response allocation cap: 8 MiB, independent of upload size.
			const responseLimit = 8 << 20
			raw, readErr := io.ReadAll(io.LimitReader(wire.Body, responseLimit+1))
			wire.Body.Close()
			if readErr != nil {
				err = readErr
			} else {
				progress.finish(false)
				cancelAttempt()
				if len(raw) > responseLimit || json.Unmarshal(raw, &response) != nil || response.Schema != "rosie.rt-control.response.v1" {
					return Response{}, ErrProtocol
				}
				if wire.StatusCode == http.StatusConflict {
					return response, &Rejected{Reason: response.Error, NativeResult: response.NativeResult,
						NativeJogResult: response.NativeJogResult, ResponseData: append(json.RawMessage(nil), response.Data...)}
				}
				if wire.StatusCode != http.StatusOK || response.Error != "" {
					return response, ErrProtocol
				}
				return response, nil
			}
		}
		uncertain = uncertain || progress.sent()
		expired := errors.Is(attemptCtx.Err(), context.DeadlineExceeded) || deadlineExpired(attemptCtx)
		progress.finish(true)
		cancelAttempt()
		if expired {
			err = errors.Join(ErrRequestTimeout, context.DeadlineExceeded, err)
		}
		if ctx.Err() != nil {
			if !uncertain {
				return Response{}, errors.Join(ErrConnectionUnsent, ctx.Err(), err)
			}
			if errors.Is(ctx.Err(), context.DeadlineExceeded) {
				return Response{}, errors.Join(ctx.Err(), err)
			}
			return Response{}, errors.Join(ErrTransport, ctx.Err(), err)
		}
		if attempt >= 1 || (!retry && uncertain) || (!expired && !connectionClosed(err)) {
			if !uncertain {
				return Response{}, errors.Join(ErrConnectionUnsent, err)
			}
			if expired {
				return Response{}, err
			}
			return Response{}, errors.Join(ErrTransport, err)
		}
		pool.CloseIdleConnections()
		ctx = context.WithValue(ctx, warmConnectionKey{}, true)
		c.connectionReopened(method, path)
	}
}

// PrepareProgram sends immutable .rdt bytes on the bulk pool. The current
// adapter ignores X-Request-ID here, so uncertain uploads are never replayed.
func (c *Client) PrepareProgram(ctx context.Context, blob []byte) (Program, error) {
	id, err := NewRequestID()
	if err != nil {
		return Program{}, err
	}
	f := c.Fence()
	h := http.Header{"Content-Type": {"application/octet-stream"}, "X-Control-Session": {f.Session}, "X-Control-Generation": {strconv.FormatUint(f.Generation, 10)}, "X-Request-Id": {id}}
	// A program is the one large body the client sends: a multi-megabyte
	// trajectory over a slow link cannot finish inside two round trips, so the
	// budget grows with the bytes at the upload throughput floor, as
	// prepare_trajectory's does. A slow link lengthens the wait; it does not
	// expire an upload that is still moving.
	r, err := c.call(ctx, c.bulk, "POST", "/v1/program", blob, h, false, c.requestBudget()+UploadAllowance(len(blob)))
	return data[Program](r, err)
}

func data[T any](r Response, err error) (T, error) {
	var v T
	if err == nil {
		if decodeErr := json.Unmarshal(r.Data, &v); decodeErr != nil {
			err = errors.Join(ErrProtocol, decodeErr)
		}
	}
	return v, err
}

func get[T any](c *Client, ctx context.Context, path string) (T, error) {
	r, err := c.call(ctx, c.control, "GET", path, nil, nil, true)
	return data[T](r, err)
}

// Close stops renewal and releases idle HTTP resources. It does not release the
// application grant: shell invocations must be able to continue that fence.
// Call Release explicitly when relinquishing authority; expiry remains native.
func (c *Client) Close() error {
	c.mu.Lock()
	c.closed = true
	c.mu.Unlock()
	c.StopRenewal()
	c.stopKeepAlive()
	c.control.CloseIdleConnections()
	c.urgent.CloseIdleConnections()
	c.bulk.CloseIdleConnections()
	c.renewal.CloseIdleConnections()
	c.jogSpareMu.Lock()
	if c.jogSpare != nil {
		c.jogSpare.Close()
		c.jogSpare = nil
	}
	c.jogSpareMu.Unlock()
	for _, pool := range c.warmClients {
		pool.CloseIdleConnections()
	}
	return nil
}

// NewUnixHTTPClient constructs a raw HTTP client with an independent Unix pool.
// Callers set their own timeout and close idle connections when finished.
func NewUnixHTTPClient(path string) *http.Client { return unixhttp.NewClient(path) }
