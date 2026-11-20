//go:build linux

package control

import (
	"bufio"
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"io"
	"math"
	"net"
	"net/http"
	"net/http/httptest"
	"runtime"
	"strings"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/ipcclient"
)

func TestAdmissionWireMeasurements(t *testing.T) {
	point := ipcclient.Point{Time: time.Duration(math.MaxInt64), Position: make([]float64, 16), Velocity: make([]float64, 16)}
	for i := range point.Position {
		point.Position[i], point.Velocity[i] = math.Nextafter(-1e-6, math.Inf(-1)), math.Nextafter(-1e-6, math.Inf(-1))
	}
	raw, err := json.Marshal(point)
	if err != nil {
		t.Fatal(err)
	}
	t.Logf("compact point including separator=%d bytes; dense sample=%d bytes", len(raw)+1, len(densejointtraj.EncodeBlock([]densejointtraj.Sample{{}})))
	if int64(len(raw)+1) != jsonPointWireBytes {
		t.Fatalf("wire measurement: %d, constant %d", len(raw)+1, jsonPointWireBytes)
	}
	if len(densejointtraj.EncodeBlock([]densejointtraj.Sample{{}})) != 153 {
		t.Fatal("dense geometry changed")
	}
	// Check both sides of every decimal exponent boundary, including the
	// decimal/scientific transition, against the measured 25-byte float bound.
	for exponent := -323; exponent <= 308; exponent++ {
		v := math.Pow10(exponent)
		for _, candidate := range []float64{v, math.Nextafter(v, 0), math.Nextafter(v, math.Inf(1))} {
			if math.IsInf(candidate, 0) {
				continue
			}
			encoded, err := json.Marshal(-candidate)
			if err != nil || len(encoded) > 25 {
				t.Fatalf("float wire bound: %s %v", encoded, err)
			}
		}
	}
}

func TestAdmissionRejectsPayloadBeforeEnvelopeAndChangedFence(t *testing.T) {
	c, f, native := admissionController()
	for _, body := range []string{
		`{"points":[`,
		`{"schema":"rosie.rt-control.request.v1","operation":"unknown","points":[`,
		strings.TrimSuffix(admissionJSON(f), "}") + `,"generation":2}`,
	} {
		w := httptest.NewRecorder()
		NewHandler(c).ServeHTTP(w, httptest.NewRequest(http.MethodPost, "/v1/control", strings.NewReader(body)))
		if w.Code == http.StatusOK {
			t.Fatalf("accepted %s", body)
		}
	}
	if native.prepares.Load() != 0 {
		t.Fatalf("native preparations=%d", native.prepares.Load())
	}
}

func TestAdmissionProgramFenceBeforeBody(t *testing.T) {
	c, f, _ := admissionController()
	for _, size := range []int64{-1, 1} {
		r := httptest.NewRequest(http.MethodPost, "/v1/program", &admissionNoRead{t})
		r.ContentLength = size
		r.Header.Set("X-Control-Session", f.Session)
		r.Header.Set("X-Control-Generation", "2")
		w := httptest.NewRecorder()
		NewHandler(c).ServeHTTP(w, r)
		admissionResponse(t, w, "control_session_stale")
	}
}

func TestAdmissionPreservesFullIdempotencePayload(t *testing.T) {
	c, f, native := admissionController()
	body := strings.Replace(admissionJSON(f), `"points":`, `"request_id":"full-points","points":`, 1)
	call := func(body, reason string) Response {
		w := httptest.NewRecorder()
		NewHandler(c).ServeHTTP(w, httptest.NewRequest(http.MethodPost, "/v1/control", strings.NewReader(body)))
		return admissionResponse(t, w, reason)
	}
	first := call(body, "")
	if replay := call(body, ""); replay.Handle != first.Handle {
		t.Fatal("preparation retry changed handle")
	}
	call(strings.Replace(body, `"time_ns":1000000`, `"time_ns":2000000`, 1), "request_id_conflict")
	if native.prepares.Load() != 1 {
		t.Fatalf("native preparations=%d", native.prepares.Load())
	}
}

func TestAdmissionAcceptsSmallUnorderedEnvelope(t *testing.T) {
	c, f, native := admissionController()
	var unordered map[string]any
	if err := json.Unmarshal([]byte(admissionJSON(f)), &unordered); err != nil {
		t.Fatal(err)
	}
	body, err := json.Marshal(unordered)
	if err != nil {
		t.Fatal(err)
	}
	w := httptest.NewRecorder()
	NewHandler(c).ServeHTTP(w, httptest.NewRequest(http.MethodPost, "/v1/control", bytes.NewReader(body)))
	admissionResponse(t, w, "")
	if native.prepares.Load() != 1 {
		t.Fatalf("preparations=%d", native.prepares.Load())
	}
}

func TestAdmissionCancellationDuringBodyReleasesSlot(t *testing.T) {
	for _, path := range []string{"/v1/control", "/v1/program"} {
		t.Run(path, func(t *testing.T) {
			c, f, native := admissionController()
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			prefix := admissionJSON(f)
			prefix = prefix[:strings.Index(prefix, `"points":`)+len(`"points":`)]
			if path == "/v1/program" {
				prefix = ""
			}
			gate := &admissionGate{prefix: strings.NewReader(prefix), tail: bytes.NewReader(nil), reached: make(chan struct{}), release: make(chan struct{}), ctx: ctx}
			r := httptest.NewRequest(http.MethodPost, path, gate).WithContext(ctx)
			r.Header.Set("X-Control-Session", f.Session)
			r.Header.Set("X-Control-Generation", "1")
			w := httptest.NewRecorder()
			done := make(chan struct{})
			go func() { defer close(done); NewHandler(c).ServeHTTP(w, r) }()
			admissionWait(t, gate.reached)
			cancel()
			admissionWait(t, done)
			admissionResponse(t, w, "context canceled")
			release, err := c.admitPreparation(f, path == "/v1/program")
			if err != nil {
				t.Fatal(err)
			}
			release()
			if native.prepares.Load() != 0 {
				t.Fatal("partial upload reached native")
			}
		})
	}
}

type admissionNative struct {
	nativeClient
	prepares atomic.Uint64
}

func (n *admissionNative) PrepareTrajectory(_ context.Context, id uint64, _ uint32, _ []ipcclient.Point) (uint64, error) {
	n.prepares.Add(1)
	return id, nil
}
func (*admissionNative) DiscardTrajectory(context.Context, uint64) (uint64, error) { return 1, nil }
func (*admissionNative) Err() error                                                { return nil }

func admissionController() (*Controller, Fence, *admissionNative) {
	n := &admissionNative{}
	f := Fence{"admission-session", 1}
	c := &Controller{core: n, grant: Grant{Session: f.Session, Generation: f.Generation}, expires: time.Now().Add(time.Hour), sessionCtx: context.Background()}
	return c, f, n
}

func admissionJSON(f Fence) string {
	return fmt.Sprintf(`{"schema":"rosie.rt-control.request.v1","operation":"prepare_trajectory","session":%q,"generation":%d,"axis_mask":1,"points":[{"time_ns":0,"position":[0]},{"time_ns":1000000,"position":[0]}]}`, f.Session, f.Generation)
}

func admissionResponse(t *testing.T, recorder *httptest.ResponseRecorder, reason string) Response {
	t.Helper()
	var response Response
	if err := json.Unmarshal(recorder.Body.Bytes(), &response); err != nil {
		t.Fatal(err)
	}
	if response.Error != reason {
		t.Fatalf("response=%s want=%q", recorder.Body.String(), reason)
	}
	return response
}

// The client sends only the envelope of a declared 60 MiB upload, then waits
// for the reply. A handler that reads points first deadlocks until the deadline.
func TestUnauthorizedLargeBodyIsRejectedBeforeDecode(t *testing.T) {
	for _, path := range []string{"/v1/control", "/v1/program"} {
		t.Run(path, func(t *testing.T) {
			c, f, _ := admissionController()
			api := serveHandleAPI(t, c, nil)
			conn, err := net.Dial("unix", api.socket)
			if err != nil {
				t.Fatal(err)
			}
			defer conn.Close()
			if err := conn.SetDeadline(time.Now().Add(2 * time.Second)); err != nil {
				t.Fatal(err)
			}
			runtime.GC()
			var before, after runtime.MemStats
			runtime.ReadMemStats(&before)
			prefix := fmt.Sprintf(`{"schema":"rosie.rt-control.request.v1","operation":"prepare_trajectory","session":%q,"generation":2,"points":`, f.Session)
			if path == "/v1/program" {
				prefix = ""
			}
			_, err = fmt.Fprintf(conn, "POST %s HTTP/1.1\r\nHost: unix\r\nContent-Length: 62914560\r\nX-Control-Session: %s\r\nX-Control-Generation: 2\r\n\r\n%s", path, f.Session, prefix)
			if err != nil {
				t.Fatal(err)
			}
			reply, err := http.ReadResponse(bufio.NewReader(conn), nil)
			if err != nil {
				t.Fatalf("reply waited for unsent body: %v", err)
			}
			defer reply.Body.Close()
			var response Response
			if err := json.NewDecoder(reply.Body).Decode(&response); err != nil {
				t.Fatal(err)
			}
			runtime.ReadMemStats(&after)
			want := "control_session_stale"
			if path == "/v1/program" {
				want = "body_too_large"
			}
			if response.Error != want {
				t.Fatalf("%+v", response)
			}
			if after.TotalAlloc-before.TotalAlloc >= 1<<20 {
				t.Fatalf("rejection allocated %d bytes", after.TotalAlloc-before.TotalAlloc)
			}
			if path == "/v1/program" && reply.StatusCode != 413 {
				t.Fatalf("status=%d", reply.StatusCode)
			}
		})
	}
}

type admissionFill byte

func (b admissionFill) Read(p []byte) (int, error) {
	for i := range p {
		p[i] = byte(b)
	}
	return len(p), nil
}

func TestBodyCapIsMachineDerived(t *testing.T) {
	if jsonPreparationBytes != 250000*890+16384 || programPreparationBytes != 250000*153+1048580 {
		t.Fatal("caps no longer match measured machine geometry")
	}
	for _, test := range []struct {
		path string
		cap  int64
	}{{"/v1/control", jsonPreparationBytes}, {"/v1/program", programPreparationBytes}} {
		t.Run(test.path, func(t *testing.T) {
			c, f, _ := admissionController()
			// A declared oversize upload must not touch its body.
			r := httptest.NewRequest(http.MethodPost, test.path, &admissionNoRead{t})
			r.ContentLength = test.cap + 1
			w := httptest.NewRecorder()
			NewHandler(c).ServeHTTP(w, r)
			admissionResponse(t, w, "body_too_large")
			if w.Code != 413 {
				t.Fatalf("status %d", w.Code)
			}
			// Unknown and false short lengths cannot bypass the observed cap.
			for _, hint := range []int64{-1, 1} {
				prefix := admissionJSON(f)
				if test.path == "/v1/program" {
					prefix = ""
				}
				r = httptest.NewRequest(http.MethodPost, test.path, io.MultiReader(strings.NewReader(prefix), io.LimitReader(admissionFill(' '), test.cap+1-int64(len(prefix)))))
				r.ContentLength = hint
				r.Header.Set("X-Control-Session", f.Session)
				r.Header.Set("X-Control-Generation", "1")
				w = httptest.NewRecorder()
				NewHandler(c).ServeHTTP(w, r)
				admissionResponse(t, w, "body_too_large")
				if w.Code != 413 {
					t.Fatalf("status %d", w.Code)
				}
				runtime.GC()
			}
			// At the cap, the admitted body reader accepts every byte; semantic
			// validation is independent of the size boundary.
			r = httptest.NewRequest(http.MethodPost, test.path, io.LimitReader(admissionFill(' '), test.cap))
			r.ContentLength = test.cap
			body, err := readAdmittedBody(httptest.NewRecorder(), r, nil, test.cap)
			if err != nil || int64(len(body)) != test.cap {
				t.Fatalf("cap rejected: bytes=%d err=%v", len(body), err)
			}
		})
		runtime.GC()
	}
}

type admissionNoRead struct{ t *testing.T }

func (r *admissionNoRead) Read([]byte) (int, error) {
	r.t.Error("inadmissible body was read")
	return 0, io.EOF
}

type admissionGate struct {
	prefix  *strings.Reader
	tail    *bytes.Reader
	reached chan struct{}
	release chan struct{}
	once    sync.Once
	ctx     context.Context
}

func (g *admissionGate) Read(p []byte) (int, error) {
	if g.prefix.Len() > 0 {
		return g.prefix.Read(p)
	}
	g.once.Do(func() { close(g.reached) })
	select {
	case <-g.release:
		return g.tail.Read(p)
	case <-g.ctx.Done():
		return 0, g.ctx.Err()
	}
}

func admissionWait(t *testing.T, ch <-chan struct{}) {
	t.Helper()
	select {
	case <-ch:
	case <-time.After(3 * time.Second):
		t.Fatal("admission barrier not reached")
	}
}

// A completed Stop between envelope admission and preparation must refuse the
// old fence before native storage is touched, even after the new grant is ready.
func TestPrepareAfterCompletedStop(t *testing.T) {
	f := startLifecycleCore(t)
	native := &admissionNative{nativeClient: f.client}
	replaceNativeTestClient(f.controller, native)
	a := serveHandleAPI(t, f.controller, nil)
	body := admissionJSON(f.fence)
	prefix := strings.Index(body, `"points":`) + len(`"points":`)
	gate := &admissionGate{prefix: strings.NewReader(body[:prefix]), tail: bytes.NewReader([]byte(body[prefix:])), reached: make(chan struct{}), release: make(chan struct{}), ctx: context.Background()}
	var release sync.Once
	unblock := func() { release.Do(func() { close(gate.release) }) }
	defer unblock()
	response := httptest.NewRecorder()
	done := make(chan struct{})
	go func() {
		defer close(done)
		NewHandler(f.controller).ServeHTTP(response, httptest.NewRequest(http.MethodPost, "/v1/control", gate))
	}()
	admissionWait(t, gate.reached)
	if !f.controller.preparations.json.Load() {
		t.Fatal("body gate did not follow envelope admission")
	}
	grant := grantFrom(t, a.call(t, controlRequest(f.fence, "stop"), ""))
	if grant.Generation != f.fence.Generation+1 || grant.Stopping {
		t.Fatalf("Stop did not complete: %+v", grant)
	}
	// A successor may own the lifecycle lane after Stop completes. The old
	// fence must still report stale authority, rather than a retryable busy.
	func() {
		f.controller.operationMu.Lock()
		defer f.controller.operationMu.Unlock()
		unblock()
		admissionWait(t, done)
	}()
	admissionResponse(t, response, ReasonControlSessionStale)
	if native.prepares.Load() != 0 || f.controller.preparations.json.Load() {
		t.Fatalf("late native prepares=%d, slot retained=%v", native.prepares.Load(), f.controller.preparations.json.Load())
	}
	request := controlRequest(Fence{grant.Session, grant.Generation}, "prepare_trajectory")
	request.AxisMask, request.Points = 1, handlePoints(time.Second)
	if a.call(t, request, "").Handle == 0 || native.prepares.Load() != 1 {
		t.Fatal("fresh fence could not prepare after late refusal")
	}
}

func TestConcurrentPreparationsAreBounded(t *testing.T) {
	f := startLifecycleCore(t)
	for _, path := range []string{"/v1/control", "/v1/program"} {
		t.Run(path, func(t *testing.T) {
			body := []byte(admissionJSON(f.fence))
			// The real daemon has nine axes.
			body = bytes.ReplaceAll(body, []byte(`"position":[0]`), []byte(`"position":[0,0,0,0,0,0,0,0,0]`))
			prefix := strings.Index(string(body), `"points":`) + len(`"points":`)
			if path == "/v1/program" {
				body = stationaryProgram(t, "bounded")
				prefix = 0
			}
			gate := &admissionGate{prefix: strings.NewReader(string(body[:prefix])), tail: bytes.NewReader(body[prefix:]), reached: make(chan struct{}), release: make(chan struct{}), ctx: context.Background()}
			var release sync.Once
			unblock := func() { release.Do(func() { close(gate.release) }) }
			defer unblock()
			request := func(reader io.Reader) *http.Request {
				r := httptest.NewRequest(http.MethodPost, path, reader)
				r.Header.Set("X-Control-Session", f.fence.Session)
				r.Header.Set("X-Control-Generation", fmt.Sprint(f.fence.Generation))
				return r
			}
			first := httptest.NewRecorder()
			done := make(chan struct{})
			go func() { defer close(done); NewHandler(f.controller).ServeHTTP(first, request(gate)) }()
			admissionWait(t, gate.reached)
			second := httptest.NewRecorder()
			NewHandler(f.controller).ServeHTTP(second, request(bytes.NewReader(body)))
			admissionResponse(t, second, "busy")
			unblock()
			admissionWait(t, done)
			response := admissionResponse(t, first, "")
			if response.Handle != 0 {
				if _, err := f.controller.Discard(f.fence, response.Handle); err != nil {
					t.Fatal(err)
				}
			}
			third := httptest.NewRecorder()
			NewHandler(f.controller).ServeHTTP(third, request(bytes.NewReader(body)))
			response = admissionResponse(t, third, "")
			if response.Handle != 0 {
				if _, err := f.controller.Discard(f.fence, response.Handle); err != nil {
					t.Fatal(err)
				}
			}
		})
	}
}

func TestCancelledPreparationReleasesSlotAndStaging(t *testing.T) {
	for _, path := range []string{"/v1/control", "/v1/program"} {
		t.Run(path, func(t *testing.T) {
			f := startLifecycleCore(t)
			barrier := &admissionCancelledNative{nativeClient: f.client, committed: make(chan uint64, 1)}
			replaceNativeTestClient(f.controller, barrier)
			done := make(chan struct{}, 2)
			api := serveHandleAPI(t, f.controller, func(h http.Handler) http.Handler {
				return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) { h.ServeHTTP(w, r); done <- struct{}{} })
			})
			body := bytes.ReplaceAll([]byte(admissionJSON(f.fence)), []byte(`"position":[0]`), []byte(`"position":[0,0,0,0,0,0,0,0,0]`))
			if path == "/v1/program" {
				body = stationaryProgram(t, "cancelled")
			}
			conn, err := net.Dial("unix", api.socket)
			if err != nil {
				t.Fatal(err)
			}
			defer conn.Close()
			_, err = fmt.Fprintf(conn, "POST %s HTTP/1.1\r\nHost: unix\r\nContent-Length: %d\r\nX-Control-Session: %s\r\nX-Control-Generation: %d\r\n\r\n%s", path, len(body), f.fence.Session, f.fence.Generation, body)
			if err != nil {
				t.Fatal(err)
			}
			var handle uint64
			select {
			case handle = <-barrier.committed:
			case <-time.After(3 * time.Second):
				t.Fatal("native preparation not committed")
			}
			_ = conn.Close()
			admissionWait(t, done)
			assertNativeUnknown(t, f.client, handle)
			f.controller.mu.Lock()
			if h := f.controller.handles[handle]; h != nil && h.State != HandleRetired {
				t.Errorf("cancelled handle=%+v", h.HandleRecord)
			}
			f.controller.mu.Unlock()
			replaceNativeTestClient(f.controller, f.client)
			r := httptest.NewRequest(http.MethodPost, path, bytes.NewReader(body))
			r.Header.Set("X-Control-Session", f.fence.Session)
			r.Header.Set("X-Control-Generation", fmt.Sprint(f.fence.Generation))
			w := httptest.NewRecorder()
			NewHandler(f.controller).ServeHTTP(w, r)
			admissionResponse(t, w, "")
		})
	}
}

type admissionCancelledNative struct {
	nativeClient
	committed chan uint64
}

func (n *admissionCancelledNative) PrepareTrajectory(ctx context.Context, id uint64, mask uint32, points []ipcclient.Point) (uint64, error) {
	handle, err := n.nativeClient.PrepareTrajectory(ctx, id, mask, points)
	if err != nil {
		return handle, err
	}
	n.committed <- handle
	<-ctx.Done()
	return handle, nil // Exercise cancellation racing a successful native reply.
}
