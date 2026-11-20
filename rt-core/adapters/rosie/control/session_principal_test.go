//go:build linux

package control

import (
	"bufio"
	"bytes"
	"context"
	"crypto/tls"
	"encoding/json"
	"fmt"
	"io"
	"net"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/internal/unixhttp"
	"rosieos/rt-core/ipcclient"
)

type principalEndpoint struct {
	client *http.Client
	base   string
}

type principalFixture struct {
	lifecycleFixture
	first, second, local principalEndpoint
	remote               *RemoteHandler
}

// newPrincipalFixture shares one daemon across two verified certificate owners
// and a Unix listener; no background renewal can hide a refused lease mutation.
func newPrincipalFixture(t *testing.T) *principalFixture {
	t.Helper()
	native := startLifecycleCore(t)
	if _, err := native.controller.Stop(native.fence, true); err != nil {
		t.Fatal(err)
	}
	pki := newRemoteTestPKI(t, native.controller.binding.PairID)
	first := pki.certificate(t, "first", pki.files.PairID, false, false)
	second := pki.certificate(t, "second", pki.files.PairID, false, false)
	local := NewHandler(native.controller)
	remote := NewRemoteHandler(native.controller, local, JogClockLimits{})
	address := remoteTestServer(t, pki, remote)
	t.Cleanup(remote.Close)
	dir, err := os.MkdirTemp("", "sp-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	path := filepath.Join(dir, "control.sock")
	listener, err := net.Listen("unix", path)
	if err != nil {
		t.Fatal(err)
	}
	server := &http.Server{Handler: local}
	done := make(chan struct{})
	go func() { defer close(done); _ = server.Serve(listener) }()
	t.Cleanup(func() { _ = server.Close(); <-done })
	client := unixhttp.NewClient(path)
	t.Cleanup(client.CloseIdleConnections)
	return &principalFixture{lifecycleFixture: native,
		first:  principalEndpoint{pki.client(t, &first), "https://" + address},
		second: principalEndpoint{pki.client(t, &second), "https://" + address},
		local:  principalEndpoint{client, "http://local"}, remote: remote}
}

func (p principalEndpoint) control(t *testing.T, request Request, reason string) RawResponse {
	t.Helper()
	request.Schema = RequestSchema
	body, err := json.Marshal(request)
	if err != nil {
		t.Fatal(err)
	}
	response := p.send(t, http.MethodPost, "/v1/control", body, nil)
	defer response.Body.Close()
	var reply RawResponse
	if err := json.NewDecoder(response.Body).Decode(&reply); err != nil {
		t.Fatal(err)
	}
	if reply.Error != reason || reply.Schema != ResponseSchema || reply.Operation != request.Operation {
		t.Fatalf("%s: %+v; want reason %q", request.Operation, reply, reason)
	}
	return reply
}

func (p principalEndpoint) send(t *testing.T, method, path string, body []byte, headers http.Header) *http.Response {
	t.Helper()
	request, err := http.NewRequest(method, p.base+path, bytes.NewReader(body))
	if err != nil {
		t.Fatal(err)
	}
	request.Header = headers.Clone()
	response, err := p.client.Do(request)
	if err != nil {
		t.Fatal(err)
	}
	return response
}

func principalGrant(t *testing.T, reply RawResponse) Grant {
	t.Helper()
	var grant Grant
	if err := json.Unmarshal(reply.Data, &grant); err != nil {
		t.Fatal(err)
	}
	return grant
}

func (f *principalFixture) acquire(t *testing.T, endpoint principalEndpoint) Fence {
	t.Helper()
	grant := principalGrant(t, endpoint.control(t, Request{Operation: OperationAcquire, Controller: "principal-test", Binding: f.controller.binding}, ""))
	return Fence{Session: grant.Session, Generation: grant.Generation}
}

type principalAttempt struct {
	name, method, path string
	body               []byte
	headers            http.Header
}

func principalAttempts(t *testing.T, fence Fence, websocket bool) []principalAttempt {
	t.Helper()
	var attempts []principalAttempt
	for _, operation := range []string{OperationAcquire, OperationRenew, OperationRelease, OperationStop, OperationEnable, OperationArm,
		OperationHome, OperationRestoreAnchor, OperationRestoreReferenceVerifiedDelta, OperationResetFault, OperationRecoveryStatus, OperationJog, OperationBeginJog,
		OperationEndJog, OperationPrepareTrajectory, OperationStartTrajectory, OperationDiscardTrajectory, OperationStartProgram} {
		id := "same-owner-request"
		body, err := json.Marshal(Request{Schema: RequestSchema, Operation: operation, Fence: fence, RequestID: &id, AxisMask: 511})
		if err != nil {
			t.Fatal(err)
		}
		attempts = append(attempts, principalAttempt{name: operation, method: http.MethodPost, path: "/v1/control", body: body})
	}
	headers := http.Header{"X-Control-Session": {fence.Session}, "X-Control-Generation": {strconv.FormatUint(fence.Generation, 10)}}
	attempts = append(attempts, principalAttempt{name: "program", method: http.MethodPost, path: "/v1/program", body: stationaryProgram(t, "principal"), headers: headers})
	for _, path := range []string{"/v1/events", "/v1/events/stream"} {
		attempts = append(attempts,
			principalAttempt{name: path + " headers", method: http.MethodGet, path: path, headers: headers},
			principalAttempt{name: path + " query", method: http.MethodGet, path: path + "?session=" + fence.Session})
	}
	if websocket {
		attempts = append(attempts, principalAttempt{name: "wss", method: http.MethodGet,
			path:    fmt.Sprintf("/v1/jog?session=%s&generation=%d&jog_generation=1", fence.Session, fence.Generation),
			headers: http.Header{"Connection": {"Upgrade"}, "Upgrade": {"websocket"}, "Sec-WebSocket-Version": {"13"}, "Sec-WebSocket-Key": {"dGhlIHNhbXBsZSBub25jZQ=="}}})
	}
	return attempts
}

// refuseAll checks exact lease and generation preservation after each rejected
// family, preventing denial paths from changing authority or consuming slots.
func (f *principalFixture) refuseAll(t *testing.T, owner, intruder principalEndpoint, fence Fence, websocket bool) {
	t.Helper()
	for _, attempt := range principalAttempts(t, fence, websocket) {
		t.Run(attempt.name, func(t *testing.T) {
			grant := principalGrant(t, owner.control(t, Request{Operation: OperationRenew, Fence: fence}, ""))
			if grant.Session != fence.Session || grant.Generation != fence.Generation {
				t.Fatalf("renew changed owner fence: %+v", grant)
			}
			f.controller.mu.Lock()
			before, expires, record := f.controller.grant, f.controller.expires, f.controller.sessions[fence.Session]
			f.controller.mu.Unlock()
			response := intruder.send(t, attempt.method, attempt.path, attempt.body, attempt.headers)
			body, err := io.ReadAll(response.Body)
			response.Body.Close()
			if err != nil {
				t.Fatal(err)
			}
			if attempt.name == "wss" {
				if response.StatusCode != http.StatusForbidden || strings.TrimSpace(string(body)) != ReasonSessionPrincipalMismatch {
					t.Fatalf("WSS refusal: %d %s", response.StatusCode, body)
				}
			} else {
				var reply RawResponse
				if err := json.Unmarshal(body, &reply); err != nil || response.StatusCode != http.StatusConflict || reply.Error != ReasonSessionPrincipalMismatch {
					t.Fatalf("refusal: %d %s %v", response.StatusCode, body, err)
				}
			}
			f.controller.mu.Lock()
			defer f.controller.mu.Unlock()
			if f.controller.grant != before || f.controller.expires != expires || f.controller.sessions[fence.Session] != record {
				t.Fatalf("rejection altered session: before=%+v after=%+v", before, f.controller.grant)
			}
			if f.controller.preparations.json.Load() || f.controller.preparations.program.Load() {
				t.Fatal("rejection consumed a preparation slot")
			}
		})
	}
}

func TestSessionPrincipalRemoteOperations(t *testing.T) {
	f := newPrincipalFixture(t)
	fence := f.acquire(t, f.first)
	id := "same-owner-request"
	f.first.control(t, Request{Operation: OperationEnable, Fence: fence, AxisMask: 511, RequestID: &id}, "")
	f.refuseAll(t, f.first, f.second, fence, true)
	f.controller.mu.Lock()
	principal := f.controller.sessions[fence.Session].principal
	f.controller.mu.Unlock()
	if principal != (sessionPrincipal{ID: "first", Pair: f.controller.binding.PairID}) {
		t.Fatalf("renew lost certificate identity: %+v", principal)
	}
	f.first.control(t, Request{Operation: OperationArm, Fence: fence}, "")
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	if _, err := f.client.Wait(ctx, func(s ipcclient.Status) bool { return s.Armed == 1 && ipcclient.AllOperationEnabled(s) }); err != nil {
		t.Fatal(err)
	}
	headers := http.Header{"X-Control-Session": {fence.Session}, "X-Control-Generation": {strconv.FormatUint(fence.Generation, 10)}}
	response := f.first.send(t, http.MethodPost, "/v1/program", stationaryProgram(t, "owner"), headers)
	var reply RawResponse
	err := json.NewDecoder(response.Body).Decode(&reply)
	response.Body.Close()
	if err != nil || reply.Error != "" || response.StatusCode != http.StatusOK {
		t.Fatalf("owner program: %+v %v", reply, err)
	}
	for _, path := range []string{"/v1/events", "/v1/events/stream"} {
		response := f.first.send(t, http.MethodGet, path, nil, headers)
		if response.StatusCode != http.StatusOK {
			t.Fatalf("owner events: %s", response.Status)
		}
		if path == "/v1/events/stream" {
			if line, err := bufio.NewReader(response.Body).ReadString('\n'); err != nil || line != ": rt-core events\n" {
				t.Fatalf("owner SSE: %q %v", line, err)
			}
		}
		response.Body.Close()
	}
	f.first.control(t, Request{Operation: OperationRelease, Fence: fence}, "")
	successor := f.acquire(t, f.second)
	f.second.control(t, Request{Operation: OperationStop, Fence: fence}, ReasonSessionPrincipalMismatch)
	f.second.control(t, Request{Operation: OperationRenew, Fence: successor}, "")
}

// Local control.sock retains its existing authority model. A TLS session is
// unusable there, and even the same user cannot carry a local session onto TLS.
func TestSessionPrincipalLocalRemoteIsolation(t *testing.T) {
	f := newPrincipalFixture(t)
	remote := f.acquire(t, f.first)
	f.refuseAll(t, f.first, f.local, remote, false)
	f.first.control(t, Request{Operation: OperationRelease, Fence: remote}, "")
	local := f.acquire(t, f.local)
	f.refuseAll(t, f.local, f.first, local, true)
	f.local.control(t, Request{Operation: OperationStop, Fence: remote}, ReasonSessionPrincipalMismatch)
	f.local.control(t, Request{Operation: OperationRenew, Fence: local}, "")
}

func TestSessionPrincipalDisconnectRequiresAcquireAfterExpiry(t *testing.T) {
	f := newPrincipalFixture(t)
	fence := f.acquire(t, f.first)
	// The fixture closes every HTTPS connection. Reconnection retains only the
	// existing lease, and expiry cannot be undone by replaying its session token.
	f.first.client.CloseIdleConnections()
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	after := uint64(0)
	expired := false
	for !expired {
		batch, wake, err := f.controller.events.batch(after)
		if err != nil {
			t.Fatal(err)
		}
		for _, event := range batch.Events {
			if event.Type == "grant_expired" && event.Grant.Generation == fence.Generation {
				expired = true
			}
		}
		after = batch.NextSequence
		if !expired && after == batch.LatestSequence {
			select {
			case <-wake:
			case <-ctx.Done():
				t.Fatal(ctx.Err())
			}
		}
	}
	f.first.control(t, Request{Operation: OperationRenew, Fence: fence}, ReasonExpired)
	f.second.control(t, Request{Operation: OperationRenew, Fence: fence}, ReasonSessionPrincipalMismatch)
	next := f.acquire(t, f.first)
	if next.Session == fence.Session || next.Generation <= fence.Generation {
		t.Fatalf("reconnect restored old authority: %+v -> %+v", fence, next)
	}
}

func TestSessionPrincipalMissingRemoteIdentityFailsClosed(t *testing.T) {
	f := newPrincipalFixture(t)
	fence := f.acquire(t, f.local)
	body, err := json.Marshal(Request{Schema: RequestSchema, Operation: OperationRenew, Fence: fence})
	if err != nil {
		t.Fatal(err)
	}
	for _, state := range []*tls.ConnectionState{nil, {}} {
		r := httptest.NewRequest(http.MethodPost, "https://remote/v1/control", bytes.NewReader(body))
		r.TLS = state
		w := httptest.NewRecorder()
		f.remote.ServeHTTP(w, r)
		if w.Code != http.StatusUnauthorized {
			t.Fatalf("missing TLS identity admitted as local: %d %s", w.Code, w.Body.String())
		}
	}
}

func TestSessionPrincipalPairIsPartOfOwner(t *testing.T) {
	c := &Controller{sessions: map[string]sessionRecord{"session": {principal: sessionPrincipal{ID: "owner", Pair: "one"}}}}
	ctx := context.WithValue(context.Background(), remotePrincipalKey{}, sessionPrincipal{ID: "owner", Pair: "two"})
	if err := c.authorizePrincipal(ctx, Fence{Session: "session"}); err == nil || err.Error() != ReasonSessionPrincipalMismatch {
		t.Fatalf("pair substitution: %v", err)
	}
}

type principalAcquireGate struct {
	nativeClient
	entered chan struct{}
	release chan struct{}
}

func (g *principalAcquireGate) MirrorAcquire(ctx context.Context, identity ipcclient.GrantIdentity, deadline uint64) (ipcclient.GrantReceipt, error) {
	g.entered <- struct{}{}
	select {
	case <-g.release:
		return g.nativeClient.MirrorAcquire(ctx, identity, deadline)
	case <-ctx.Done():
		return ipcclient.GrantReceipt{}, ctx.Err()
	}
}

// A grant is owned before its native acknowledgement or HTTP response. Neither
// the session token nor an in-flight/cached acquire ID can switch that owner.
func TestSessionPrincipalAcquirePublicationAndRetry(t *testing.T) {
	f := newPrincipalFixture(t)
	gate := &principalAcquireGate{nativeClient: f.controller.core, entered: make(chan struct{}, 1), release: make(chan struct{})}
	replaceNativeTestClient(f.controller, gate)
	defer close(gate.release)
	id := "acquire-retry"
	request := Request{Schema: RequestSchema, Operation: OperationAcquire, Controller: "retry", Binding: f.controller.binding, RequestID: &id}
	body, err := json.Marshal(request)
	if err != nil {
		t.Fatal(err)
	}
	type result struct {
		response *http.Response
		err      error
	}
	completed := make(chan result, 1)
	go func() {
		response, err := f.first.client.Post(f.first.base+"/v1/control", "application/json", bytes.NewReader(body))
		completed <- result{response, err}
	}()
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	select {
	case <-gate.entered:
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
	f.controller.mu.Lock()
	fence := Fence{Session: f.controller.grant.Session, Generation: f.controller.grant.Generation}
	record := f.controller.sessions[fence.Session]
	f.controller.mu.Unlock()
	if fence.Session == "" || record.principal.ID != "first" || record.principal.Pair != f.controller.binding.PairID {
		t.Fatalf("unbound pending acquire: %+v %+v", fence, record)
	}
	f.local.control(t, Request{Operation: OperationRenew, Fence: fence}, ReasonSessionPrincipalMismatch)
	f.second.control(t, request, ReasonSessionPrincipalMismatch)
	f.local.control(t, request, ReasonSessionPrincipalMismatch)
	select {
	case gate.release <- struct{}{}:
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
	var pending result
	select {
	case pending = <-completed:
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
	if pending.err != nil {
		t.Fatal(pending.err)
	}
	defer pending.response.Body.Close()
	var reply RawResponse
	if err := json.NewDecoder(pending.response.Body).Decode(&reply); err != nil || reply.Error != "" {
		t.Fatalf("pending acquire: %+v %v", reply, err)
	}
	grant := principalGrant(t, reply)
	if grant.Session != fence.Session || grant.Generation != fence.Generation {
		t.Fatalf("pending grant changed: %+v", grant)
	}
	f.second.control(t, request, ReasonSessionPrincipalMismatch)
	f.local.control(t, request, ReasonSessionPrincipalMismatch)
	if replay := principalGrant(t, f.first.control(t, request, "")); replay != grant {
		t.Fatalf("owner retry changed the grant: %+v -> %+v", grant, replay)
	}
}

func TestSessionPrincipalJogOwnerRemainsUsable(t *testing.T) {
	f := startRemoteMotion(t)
	generation := f.begin(t)
	certificate := f.pki.certificate(t, "intruder", f.controller.binding.PairID, false, false)
	intruder := principalEndpoint{f.pki.client(t, &certificate), f.url}
	response := intruder.send(t, http.MethodGet, fmt.Sprintf("/v1/jog?session=%s&generation=%d&jog_generation=%d", f.fence.Session, f.fence.Generation, generation), nil, nil)
	body, err := io.ReadAll(response.Body)
	response.Body.Close()
	if err != nil || response.StatusCode != http.StatusForbidden || strings.TrimSpace(string(body)) != ReasonSessionPrincipalMismatch {
		t.Fatalf("intruder WSS: %s %s %v", response.Status, body, err)
	}
	w := f.dial(t, generation)
	w.calibrate(t)
	w.update(t, w.frame(f, generation, 1))
	observation := waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence == 1 })
	if observation.JogGeneration != generation || observation.UpdateReason != ipcclient.JogReasonNone || observation.Open != 1 {
		t.Fatalf("owner jog affected by rejection: %+v", observation)
	}
}
