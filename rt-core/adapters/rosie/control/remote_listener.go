//go:build linux

package control

import (
	"context"
	"crypto/tls"
	"errors"
	"net"
	"net/http"
	"sync"
	"time"
)

type remotePrincipalKey struct{}

// A zero principal denotes the existing local socket authority. Both URI SANs
// identify a remote owner; neither a session token nor a header changes domains.
type sessionPrincipal struct{ ID, Pair string }

type sessionRecord struct {
	incarnation string
	principal   sessionPrincipal
	granted     time.Time // Wall clock at acquire; GET /v1/maintenance names it.
}

// NewRemoteListener suppresses net/http's plaintext HTTP 400 fallback after a
// failed TLS handshake. An unauthenticated peer receives TLS alerts or closure,
// never an HTTP response, including when it sends a plaintext Stop request.
func NewRemoteListener(listener net.Listener, config *tls.Config) net.Listener {
	return tls.NewListener(remoteTransportListener{listener}, config)
}

type remoteTransportListener struct{ net.Listener }

func (l remoteTransportListener) Accept() (net.Conn, error) {
	conn, err := l.Listener.Accept()
	if err != nil {
		return nil, err
	}
	return remoteTransportConn{conn}, nil
}

type remoteTransportConn struct{ net.Conn }

func (c remoteTransportConn) Write(payload []byte) (int, error) {
	if len(payload) >= 5 && string(payload[:5]) == "HTTP/" {
		_ = c.Conn.Close()
		return 0, net.ErrClosed
	}
	return c.Conn.Write(payload)
}

func requestPrincipal(ctx context.Context) sessionPrincipal {
	principal, _ := ctx.Value(remotePrincipalKey{}).(sessionPrincipal)
	return principal
}

// authorizePrincipal preserves ownership for retired sessions as well as live
// grants, so recovery Stop cannot cross principals or local/remote boundaries.
// It never renews a lease, changes a generation, or grants motion authority.
func (c *Controller) authorizePrincipal(ctx context.Context, f Fence) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	if session, known := c.sessions[f.Session]; known && session.principal != requestPrincipal(ctx) {
		return errors.New(ReasonSessionPrincipalMismatch)
	}
	return nil
}

// dispatchPrincipal binds Acquire inside its grant transaction. Other commands
// keep the shared dispatcher and cannot replay a cached response across owners.
// The principal check already ran on the envelope fence, which the caller has
// proven equal to request.Fence.
func (h *Handler) dispatchPrincipal(r *http.Request, request Request) (Response, error) {
	if err := h.Controller.checkMaintenance(request.Operation); err != nil {
		return Response{Schema: ResponseSchema, Operation: request.Operation}, err
	}
	if request.Operation == OperationAcquire {
		response := Response{Schema: ResponseSchema, Operation: request.Operation}
		if err := h.Controller.CheckRestartFence(request.Fence); err != nil {
			return response, err
		}
		var err error
		response.Data, err = h.Controller.acquire(request.Controller, request.Binding, requestPrincipal(r.Context()), request.RequestedLeaseMS)
		return response, err
	}
	return h.dispatch(request)
}

// checkEventPrincipal leaves session-free observations unchanged. If a caller
// supplies session headers or a session query, each named owner must match.
func (h *Handler) checkEventPrincipal(w http.ResponseWriter, r *http.Request) bool {
	for _, session := range append(r.Header.Values("X-Control-Session"), r.URL.Query()["session"]...) {
		if err := h.Controller.authorizePrincipal(r.Context(), Fence{Session: session}); err != nil {
			outcome := encodeResponse(Response{Schema: ResponseSchema, Operation: OperationSubscribeEvents}, err)
			w.WriteHeader(outcome.status)
			_, _ = w.Write(outcome.body)
			return false
		}
	}
	return true
}

type RemoteHandler struct {
	controller *Controller
	local      http.Handler
	limits     JogClockLimits
	mu         sync.Mutex
	active     *remoteJogConnection
	closed     bool
}

// NewRemoteHandler authenticates the transport before the shared public handler
// binds sessions, preventing unauthenticated requests from becoming local calls.
func NewRemoteHandler(c *Controller, local http.Handler, limits JogClockLimits) *RemoteHandler {
	ceiling := c.maxJogInputAge()
	if limits.MaxInputAgeNS == 0 || limits.MaxInputAgeNS > ceiling {
		limits.MaxInputAgeNS = ceiling
	}
	if limits.MaxInputDurationNS == 0 || limits.MaxInputDurationNS > ceiling {
		limits.MaxInputDurationNS = ceiling
	}
	return &RemoteHandler{controller: c, local: local, limits: limits}
}

func (h *RemoteHandler) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	if r.TLS == nil || len(r.TLS.VerifiedChains) == 0 || len(r.TLS.PeerCertificates) == 0 {
		http.Error(w, "", http.StatusUnauthorized)
		return
	}
	principal, err := remoteIdentity(r.TLS.PeerCertificates[0], h.controller.binding.PairID)
	if err != nil {
		http.Error(w, "", http.StatusUnauthorized)
		return
	}
	r = r.WithContext(context.WithValue(r.Context(), remotePrincipalKey{}, sessionPrincipal{ID: principal, Pair: h.controller.binding.PairID}))
	if r.Method == http.MethodGet && (r.URL.Path == "/v1/events" || r.URL.Path == "/v1/events/stream") {
		h.local.ServeHTTP(w, r)
		return
	}
	if r.URL.Path == "/v1/jog" {
		h.serveJog(w, r)
		return
	}
	h.local.ServeHTTP(w, r)
}

// Close retires hijacked connections, which http.Server.Close does not own.
func (h *RemoteHandler) Close() {
	h.mu.Lock()
	h.closed = true
	active := h.active
	h.active = nil
	h.mu.Unlock()
	if active != nil {
		active.ws.close(1001)
	}
}
