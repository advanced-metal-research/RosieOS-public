//go:build linux

package ipcclient

import (
	"bytes"
	"context"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"net"
	"net/http"
	"path/filepath"
	"rosieos/rt-core/internal/controltest"
	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/internal/unixhttp"
	"sort"
	"sync"
	"testing"
	"time"
)

type jogReply struct {
	Handle          uint64          `json:"handle"`
	Error           string          `json:"error"`
	Data            json.RawMessage `json:"data"`
	NativeResult    *CommandResult  `json:"native_result"`
	NativeJogResult *JogObservation `json:"native_jog_result"`
}

type jogService struct {
	http      *http.Client
	dgram     *net.UnixConn
	session   string
	grant     uint64
	clock     LocalJogClock
	stopRenew func()
	t         *testing.T
}

func (s *jogService) request(operation string, fields map[string]any) (jogReply, error) {
	if fields == nil {
		fields = make(map[string]any)
	}
	fields["schema"], fields["operation"] = "rosie.rt-control.request.v1", operation
	if _, ok := fields["session"]; !ok {
		fields["session"] = s.session
	}
	if _, ok := fields["generation"]; !ok {
		fields["generation"] = s.grant
	}
	body, err := json.Marshal(fields)
	if err != nil {
		return jogReply{}, err
	}
	var reply jogReply
	_, err = (controltest.Client{HTTP: s.http, Base: "http://local"}).Decode(context.Background(), http.MethodPost, "/v1/control", body, http.Header{"Content-Type": {"application/json"}}, &reply)
	return reply, err
}

func (s *jogService) must(operation string, fields map[string]any) jogReply {
	s.t.Helper()
	r, err := s.request(operation, fields)
	if err != nil || r.Error != "" {
		s.t.Fatalf("%s: %+v %v", operation, r, err)
	}
	return r
}

func (s *jogService) get(path string, value any) error {
	var reply jogReply
	_, err := (controltest.Client{HTTP: s.http, Base: "http://local"}).Decode(context.Background(), http.MethodGet, path, nil, nil, &reply)
	if err != nil {
		return err
	}
	if reply.Error != "" {
		return fmt.Errorf("%s", reply.Error)
	}
	return json.Unmarshal(reply.Data, value)
}

func (s *jogService) observe(predicate func(JogObservation) bool) JogObservation {
	s.t.Helper()
	deadline := time.Now().Add(time.Second)
	var pollResult JogObservation
	if !simfixture.PollUntil(deadline, time.Millisecond, func() bool {
		var o JogObservation
		err := s.get("/v1/jog", &o)
		if err == nil && predicate(o) {
			pollResult = o
			return true
		}
		if time.Now().After(deadline) {
			s.t.Fatalf("jog observation: %+v %v", o, err)
		}
		return false
	}) {
		s.t.Fatal("observation deadline exceeded")
	}
	return pollResult
}

func (s *jogService) status(predicate func(Status) bool) Status {
	s.t.Helper()
	deadline := time.Now().Add(time.Second)
	var pollResult Status
	if !simfixture.PollUntil(deadline, time.Millisecond, func() bool {
		var status struct {
			Core Status `json:"core"`
		}
		err := s.get("/v1/status", &status)
		if err == nil && predicate(status.Core) {
			pollResult = status.Core
			return true
		}
		if time.Now().After(deadline) {
			s.t.Fatalf("status: %+v %v", status.Core, err)
		}
		return false
	}) {
		s.t.Fatal("observation deadline exceeded")
	}
	return pollResult
}

func (s *jogService) beginReply() jogReply {
	s.t.Helper()
	now := HostMonotonicNS()
	r, err := s.request("begin_jog", map[string]any{"axis_mask": 3, "clock_incarnation": s.clock.Incarnation,
		"source_origin_host_ns": now, "deadline_host_ns": now + 200_000_000})
	if err != nil {
		s.t.Fatal(err)
	}
	return r
}

func (s *jogService) begin() uint64 {
	s.t.Helper()
	r := s.beginReply()
	if r.Error != "" || r.Handle == 0 {
		s.t.Fatalf("begin: %+v", r)
	}
	return r.Handle
}

func (s *jogService) frame(generation, sequence uint64) LocalJogUpdate {
	s.t.Helper()
	frame := LocalJogUpdate{GrantGeneration: s.grant, JogGeneration: generation, SourceSequence: sequence,
		OriginHostNs: HostMonotonicNS(), AxisMask: 3, Velocity: [16]float64{.001, .001}}
	frame.DeadlineHostNs = frame.OriginHostNs + 200_000_000
	token, err := hex.DecodeString(s.session)
	if err != nil {
		s.t.Fatal(err)
	}
	copy(frame.SessionId[:], token)
	incarnation, err := hex.DecodeString(s.clock.Incarnation)
	if err != nil {
		s.t.Fatal(err)
	}
	copy(frame.ClockIncarnation[:], incarnation)
	return frame
}

func (s *jogService) send(frame LocalJogUpdate) {
	s.t.Helper()
	encoded := EncodeLocalJogUpdate(frame)
	if err := s.dgram.SetWriteDeadline(time.Now().Add(20 * time.Millisecond)); err != nil {
		s.t.Fatal(err)
	}
	if n, err := s.dgram.Write(encoded[:]); err != nil || n != len(encoded) {
		s.t.Fatalf("datagram: %d %v", n, err)
	}
}

// The real service and simulator exercise both process boundaries. Renewal
// runs independently so bulk preparation cannot hide behind a revoked grant.
func startJogService(t *testing.T) *jogService {
	t.Helper()
	f := startCoreWithRole(t, "simulation.json", false)
	if err := f.client.Close(); err != nil {
		t.Fatal(err)
	}
	dir := filepath.Dir(f.options.SocketPath)
	socket := filepath.Join(dir, "control.sock")
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}

	cmd := simfixture.StartControl(root, f.options.SocketPath, socket, f.options.ConfigurationSHA256, "jog-test", 1)
	simfixture.SpawnLogged(t, cmd, filepath.Join(dir, "control.log"), 3*time.Second, 0, nil)

	transport := unixhttp.NewClient(socket).Transport.(*http.Transport)
	s := &jogService{http: &http.Client{Transport: transport, Timeout: 60 * time.Second}, t: t}
	t.Cleanup(transport.CloseIdleConnections)
	deadline := time.Now().Add(3 * time.Second)
	if !simfixture.PollUntil(deadline, time.Millisecond, func() bool {
		err := s.get("/v1/jog/clock", &s.clock)
		if err == nil {
			return true
		}
		if time.Now().After(deadline) {
			t.Fatal(err)
		}
		return false
	}) {
		t.Fatal("observation deadline exceeded")
	}
	if s.clock.Domain != "CLOCK_MONOTONIC" || s.clock.MappingGeneration != 1 {
		t.Fatalf("clock: %+v", s.clock)
	}
	r := s.must("acquire", map[string]any{"controller": "jog-test", "binding": map[string]any{
		"pair_id": "jog-test", "revision": 1, "configuration_sha256": f.options.ConfigurationSHA256}})
	var grant struct {
		Session    string
		Generation uint64
	}
	if err := json.Unmarshal(r.Data, &grant); err != nil {
		t.Fatal(err)
	}
	s.session, s.grant = grant.Session, grant.Generation
	renewContext, cancelRenew := context.WithCancel(context.Background())
	joined := simfixture.Renew(renewContext, 50*time.Millisecond, func() bool {
		r, err := s.request("renew", nil)
		if err != nil || r.Error != "" {
			if renewContext.Err() == nil {
				t.Errorf("renew: %+v %v", r, err)
			}
			return false
		}
		return true
	})
	var stopOnce sync.Once
	s.stopRenew = func() { stopOnce.Do(func() { cancelRenew(); joined() }) }
	t.Cleanup(s.stopRenew)
	s.must("enable", map[string]any{"axis_mask": 3})
	s.must("arm", nil)
	s.status(allOperationEnabled)
	s.dgram, err = net.DialUnix("unixgram", nil, &net.UnixAddr{Name: filepath.Join(dir, "jog.sock"), Net: "unixgram"})
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { s.dgram.Close() })
	return s
}

func TestJogUpdatesAreNotDelayedByBulkUpload(t *testing.T) {
	s := startJogService(t)
	points := make([]Point, 250_000)
	for i := range points {
		points[i] = Point{Time: time.Duration(i) * time.Millisecond, Position: []float64{0, 0}}
	}
	// Encode before Begin so the measurement covers service ingestion and
	// native preparation, rather than the producer's JSON encoder. A large
	// preparation must carry its envelope (schema, operation, fence) before the
	// points: the adapter admits the fence from a bounded prefix and refuses to
	// buffer an unauthorized body, so a map (sorted keys) would place "points"
	// before "schema" and be rejected body_too_large.
	body, err := json.Marshal(struct {
		Schema     string  `json:"schema"`
		Operation  string  `json:"operation"`
		Session    string  `json:"session"`
		Generation uint64  `json:"generation"`
		AxisMask   uint32  `json:"axis_mask"`
		Points     []Point `json:"points"`
	}{"rosie.rt-control.request.v1", "prepare_trajectory", s.session, s.grant, 3, points})
	if err != nil {
		t.Fatal(err)
	}
	generation := s.begin()
	type result struct {
		end   uint64
		reply jogReply
		err   error
	}
	done := make(chan result, 1)
	started := HostMonotonicNS()
	go func() {
		var reply jogReply
		response, err := s.http.Post("http://local/v1/control", "application/json", bytes.NewReader(body))
		if err == nil {
			err = json.NewDecoder(response.Body).Decode(&reply)
			response.Body.Close()
		}
		done <- result{HostMonotonicNS(), reply, err}
	}()
	ticker := time.NewTicker(10 * time.Millisecond)
	defer ticker.Stop()
	ages := make([]uint64, 0, 20)
	observations := make([]JogObservation, 0, 20)
	for sequence := uint64(1); sequence <= 20; sequence++ {
		<-ticker.C
		frame := s.frame(generation, sequence)
		s.send(frame)
		o := s.observe(func(o JogObservation) bool { return o.ObservedSourceSequence >= sequence })
		if o.ObservedSourceSequence != sequence || o.SourceSequence != sequence || o.UpdateReason != JogReasonNone || o.HasInput != 1 {
			t.Fatalf("update %d: %+v", sequence, o)
		}
		if o.ObservedOriginHostNs != frame.OriginHostNs || o.InputDeadlineHostNs > frame.DeadlineHostNs || o.FirstObservedHostNs < frame.OriginHostNs {
			t.Fatalf("clock/deadline changed: frame=%+v observation=%+v", frame, o)
		}
		ages = append(ages, o.FirstObservedHostNs-frame.OriginHostNs)
		observations = append(observations, o)
	}
	s.must("end_jog", map[string]any{"jog_generation": generation})
	upload := <-done
	if upload.err != nil || upload.reply.Error != "" || upload.reply.Handle == 0 {
		t.Fatalf("upload: %+v", upload)
	}
	if upload.end-started < 100_000_000 {
		t.Fatalf("bulk contention was not exercised: %d ns", upload.end-started)
	}
	for _, o := range observations {
		if o.FirstObservedHostNs >= upload.end {
			t.Fatalf("sequence %d waited until bulk completion", o.ObservedSourceSequence)
		}
		t.Logf("sequence=%d origin_host_ns=%d first_rt_now_host_ns=%d", o.ObservedSourceSequence, o.ObservedOriginHostNs, o.FirstObservedHostNs)
	}
	sort.Slice(ages, func(i, j int) bool { return ages[i] < ages[j] })
	t.Logf("jog input age ns: p50=%d p99=%d max=%d; upload_ns=%d", ages[9], ages[19], ages[19], upload.end-started)
	if ages[19] > 20_000_000 {
		t.Fatalf("max jog age %d ns exceeds 20,000,000 ns", ages[19])
	}
}

func TestExpiredJogUpdateNeverApplies(t *testing.T) {
	s := startJogService(t)
	generation := s.begin()
	before := s.status(allOperationEnabled)
	frame := s.frame(generation, 1)
	frame.OriginHostNs -= 10_000_000
	frame.DeadlineHostNs = frame.OriginHostNs + 1_000_000
	s.send(frame)
	deadline := time.Now().Add(time.Second)
	if !simfixture.PollUntil(deadline, time.Millisecond, func() bool {
		var ingress struct {
			SourceSequence uint64 `json:"source_sequence"`
			Reason         uint32 `json:"reason"`
		}
		if err := s.get("/v1/jog/ingress", &ingress); err == nil && ingress.SourceSequence == 1 && ingress.Reason == JogReasonExpired {
			return true
		}
		if time.Now().After(deadline) {
			t.Fatalf("expired ingress not observed: %+v", ingress)
		}
		return false
	}) {
		t.Fatal("observation deadline exceeded")
	}
	until := HostMonotonicNS() + 250_000_000
	for {
		state := s.status(func(Status) bool { return true })
		if state.Axes[0].PositionCounts != before.Axes[0].PositionCounts || state.Axes[1].PositionCounts != before.Axes[1].PositionCounts {
			t.Fatal("expired datagram moved feedback")
		}
		if state.TimeNS >= until {
			break
		}
		time.Sleep(time.Millisecond)
	}
	o := s.observe(func(o JogObservation) bool { return o.Open == 0 })
	if o.SourceSequence != 0 || o.HasInput != 0 {
		t.Fatalf("expired input admitted: %+v", o)
	}
}

func TestJogModeConflictIsNativeDecision(t *testing.T) {
	s := startJogService(t)
	handle := s.must("prepare_trajectory", map[string]any{"axis_mask": 3, "points": []Point{
		{Position: []float64{0, 0}}, {Time: 5 * time.Second, Position: []float64{0, 0}},
	}}).Handle
	generation := s.begin()
	r, err := s.request("start_trajectory", map[string]any{"handle": handle})
	if err != nil || r.NativeResult == nil || r.NativeResult.Reason != ReasonModeConflict || r.NativeResult.Result != ResultRejected {
		t.Fatalf("Start must be rejected by RT: %+v %v", r, err)
	}
	s.must("end_jog", map[string]any{"jog_generation": generation})
	s.must("enable", map[string]any{"axis_mask": 3})
	s.must("arm", nil)
	s.status(allOperationEnabled)
	// A rejected native Start retires its attempted handle; prepare a new one.
	handle = s.must("prepare_trajectory", map[string]any{"axis_mask": 3, "points": []Point{
		{Position: []float64{0, 0}}, {Time: 5 * time.Second, Position: []float64{0, 0}},
	}}).Handle
	s.must("start_trajectory", map[string]any{"handle": handle})
	r = s.beginReply()
	if r.NativeJogResult == nil || r.NativeJogResult.ControlReason != JogReasonModeConflict || r.NativeJogResult.ControlAccepted != 0 {
		t.Fatalf("Begin must be rejected by RT: %+v", r)
	}
}

func TestStopClosesJogAndFencesLateUpdates(t *testing.T) {
	s := startJogService(t)
	generation := s.begin()
	frame := s.frame(generation, 1)
	frame.Velocity[0], frame.Velocity[1] = .05, .05
	s.send(frame)
	s.observe(func(o JogObservation) bool { return o.SourceSequence == 1 && o.HasInput == 1 })
	s.status(func(state Status) bool {
		return state.Axes[0].PositionCounts >= 10 && state.Axes[1].PositionCounts <= -10
	})
	s.stopRenew()
	s.must("stop", nil)
	stoppedAt := HostMonotonicNS()
	s.status(func(state Status) bool {
		return state.TimeNS >= stoppedAt && state.Armed == 0 && state.AxisEnableMask == 0
	})
	s.observe(func(o JogObservation) bool { return o.Open == 0 && o.HasInput == 0 })
	s.send(s.frame(generation, 2))
	deadline := time.Now().Add(time.Second)
	if !simfixture.PollUntil(deadline, time.Millisecond, func() bool {
		var ingress struct {
			SourceSequence uint64 `json:"source_sequence"`
			Reason         uint32 `json:"reason"`
		}
		if err := s.get("/v1/jog/ingress", &ingress); err == nil && ingress.SourceSequence == 2 &&
			(ingress.Reason == JogReasonClosed || ingress.Reason == JogReasonInvalidGeneration) {
			return true
		}
		if time.Now().After(deadline) {
			t.Fatalf("late generation not rejected: %+v", ingress)
		}
		return false
	}) {
		t.Fatal("observation deadline exceeded")
	}
	until := HostMonotonicNS() + 100_000_000
	for {
		state := s.status(func(Status) bool { return true })
		if state.Armed != 0 || state.AxisEnableMask != 0 {
			t.Fatalf("late update enabled drives: %+v", state)
		}
		if state.TimeNS >= until {
			break
		}
		time.Sleep(time.Millisecond)
	}
}
