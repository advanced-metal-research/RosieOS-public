//go:build linux

package control

import (
	"context"
	"crypto/tls"
	"errors"
	"io"
	"log"
	"net"
	"net/http"
	"net/http/httptrace"
	"os"
	"sync/atomic"
	"syscall"
	"time"
)

// ErrConnectionUnsent proves no request bytes were written on either attempt;
// callers must not turn this transport refusal into uncertain motion evidence.
var ErrConnectionUnsent = errors.New("rt_core_connection_reopened: request not sent")

type countedConnection struct {
	net.Conn
	written atomic.Uint64
}

func (c *countedConnection) Write(p []byte) (int, error) {
	n, err := c.Conn.Write(p)
	c.written.Add(uint64(n))
	return n, err
}

type requestProgress struct {
	wire    net.Conn
	conn    *countedConnection
	before  uint64
	unknown bool
}

// traceRequest counts successful socket writes, including partial headers, so
// only a proven unsent upload or Release may bypass uncertain-outcome handling.
func traceRequest(r *http.Request, limit ...time.Time) *requestProgress {
	p := &requestProgress{}
	trace := &httptrace.ClientTrace{GotConn: func(info httptrace.GotConnInfo) {
		p.wire = info.Conn
		deadline, ok := r.Context().Deadline()
		if len(limit) != 0 && (!ok || limit[0].Before(deadline)) {
			deadline, ok = limit[0], true
		}
		if ok {
			_ = info.Conn.SetDeadline(deadline)
		}
		conn := info.Conn
		if secure, ok := conn.(*tls.Conn); ok {
			conn = secure.NetConn()
		}
		if counted, ok := conn.(*countedConnection); ok {
			p.conn, p.before = counted, counted.written.Load()
		} else {
			p.unknown = true
		}
	}}
	*r = *r.WithContext(httptrace.WithClientTrace(r.Context(), trace))
	return p
}

// finish closes failed attempts before reopening and clears completed deadlines
// before the lane is returned, so an old timer cannot poison its next borrower.
func (p *requestProgress) finish(failed bool) {
	if p.wire != nil {
		if failed {
			_ = p.wire.Close()
		} else {
			_ = p.wire.SetDeadline(time.Time{})
		}
	}
}

func (p *requestProgress) sent() bool {
	return p.unknown || (p.conn != nil && p.conn.written.Load() > p.before)
}

// connectionClosed requires peer-close evidence. A timeout or cancellation takes
// precedence over wrapped EOFs, preventing replay of a merely delayed response.
func connectionClosed(err error) bool {
	var networkError net.Error
	if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) || errors.Is(err, os.ErrDeadlineExceeded) ||
		(errors.As(err, &networkError) && networkError.Timeout()) {
		return false
	}
	return errors.Is(err, ErrWarmConnection) || errors.Is(err, io.EOF) ||
		errors.Is(err, io.ErrUnexpectedEOF) || errors.Is(err, syscall.EPIPE) || errors.Is(err, syscall.ECONNRESET)
}

func (c *Client) connectionReopened(method, path string) {
	log.Printf("rt_core_connection_reopened method=%s path=%s", method, path)
}
