// Package main is cell-video: the cell camera's WebRTC sender.
//
// Why: the camera's JPEG stream cost about 25 GB an hour per browser viewer
// and 12.7 GB per Deck (a Rosie 1430 arm, 2026-10-16), and those bytes shared the SSH
// tunnel with jog replies. The camera service (cell-camera/v1, video.py) now
// encodes one low-data H.264 stream on loopback; this program reads it only
// while someone watches and sends it to each viewer over WebRTC: UDP over
// Tailscale, outside the tunnel, with lost packets resent (NACK) and key
// frames on a viewer's request (PLI).
//
// It is its own process so that a video fault can never stop the OLP server
// that carries jog control (owner decision 2026-10-19). WebRTC is done here in
// Go (Pion) rather than in the camera service because GStreamer 1.20's Python
// binding crashed reading webrtcbin's replies on the Jetsons' Ubuntu 22.04.
//
// Interface, on loopback only (the OLP relays it as the cell camera's WHEP):
//
//	POST   /whep        an SDP offer (application/sdp) in; 201 with the SDP answer, Location /whep/<id>
//	DELETE /whep/<id>   ends that viewer's session
//	GET    /status      viewers, bytes sent, the camera stream's state
//
// WebRTC media uses one UDP port on one address (the host's Tailscale
// address), so every viewer shares one socket and no other interface listens.
package main

import (
	"bufio"
	"context"
	"crypto/rand"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"fmt"
	"io"
	"log"
	"net"
	"net/http"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"github.com/pion/interceptor"
	"github.com/pion/rtcp"
	"github.com/pion/rtp"
	"github.com/pion/webrtc/v4"
	"github.com/pion/webrtc/v4/pkg/media"
)

// Unverified bounds. A viewer that has not connected this long after its
// answer, or whose connection has been lost this long, is removed (its client
// retries with a new offer).
const (
	connectTimeout   = 15 * time.Second
	lostTimeout      = 10 * time.Second
	gatherTimeout    = 2 * time.Second
	maxOfferBytes    = 64 << 10
	maxViewers       = 8
	maxUnitBytes     = 4 << 20
	keyframeInterval = time.Second // at most one key frame request a second, whatever the viewers ask
	sourceRetry      = time.Second
)

// Config is what cell-video needs, from the camera's settings file.
type Config struct {
	Camera  string // the camera service's origin with its prefix, http://127.0.0.1:8088/camera
	Listen  string // loopback host:port for the WHEP interface
	Address net.IP // the one address WebRTC uses (the host's Tailscale address)
	Port    int    // the one UDP port every viewer's media uses
}

type viewer struct {
	id, client string
	pc         *webrtc.PeerConnection
	sent       *byteCounter
	nacks      atomic.Uint32
	created    time.Time
	connected  bool
	lostAt     time.Time
}

// byteCounter counts the RTP bytes (header and payload) and packets one
// viewer's connection writes. Pion 4.2 reports no outbound stream stats.
type byteCounter struct {
	interceptor.NoOp
	bytes, packets atomic.Uint64
}

func (c *byteCounter) BindLocalStream(_ *interceptor.StreamInfo, w interceptor.RTPWriter) interceptor.RTPWriter {
	return interceptor.RTPWriterFunc(func(h *rtp.Header, payload []byte, a interceptor.Attributes) (int, error) {
		n, err := w.Write(h, payload, a)
		if err == nil {
			c.bytes.Add(uint64(n))
			c.packets.Add(1)
		}
		return n, err
	})
}

// counterFactory hands the next connection its viewer's counter: Offer sets
// `next` while it creates that connection.
type counterFactory struct {
	mu   sync.Mutex
	next *byteCounter
}

func (f *counterFactory) NewInterceptor(string) (interceptor.Interceptor, error) {
	if f.next == nil {
		return &interceptor.NoOp{}, nil
	}
	return f.next, nil
}

// Sender serves the viewers of one camera.
type Sender struct {
	counters *counterFactory
	cfg      Config
	api      *webrtc.API
	track    *webrtc.TrackLocalStaticSample
	http     *http.Client

	mu        sync.Mutex
	viewers   map[string]*viewer
	sourceOn  bool
	cancel    context.CancelFunc
	units     int64
	unitBytes int64
	sourceErr string
	lastKey   time.Time
	ended     int64
}

// New builds a Sender whose WebRTC listens only on conn (one UDP socket on
// cfg.Address:cfg.Port).
func New(cfg Config, conn net.PacketConn) (*Sender, error) {
	m := &webrtc.MediaEngine{}
	feedback := []webrtc.RTCPFeedback{{Type: "nack"}, {Type: "nack", Parameter: "pli"}, {Type: "ccm", Parameter: "fir"}}
	// Constrained baseline with packetization-mode 1: what the camera's x264
	// sends (video.py) and what every browser and the Deck decode.
	if err := m.RegisterCodec(webrtc.RTPCodecParameters{
		RTPCodecCapability: webrtc.RTPCodecCapability{MimeType: webrtc.MimeTypeH264, ClockRate: 90000,
			SDPFmtpLine: "level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42e01f", RTCPFeedback: feedback},
		PayloadType: 102,
	}, webrtc.RTPCodecTypeVideo); err != nil {
		return nil, err
	}
	registry := &interceptor.Registry{}
	if err := webrtc.RegisterDefaultInterceptors(m, registry); err != nil {
		return nil, err
	}
	counters := &counterFactory{}
	registry.Add(counters)
	var s webrtc.SettingEngine
	s.SetNetworkTypes([]webrtc.NetworkType{webrtc.NetworkTypeUDP4, webrtc.NetworkTypeUDP6})
	s.SetIPFilter(func(ip net.IP) bool { return ip.Equal(cfg.Address) })
	s.SetICEUDPMux(webrtc.NewICEUDPMux(nil, conn))
	if cfg.Address.IsLoopback() {
		s.SetIncludeLoopbackCandidate(true) // tests only: Run refuses a loopback address
	}
	track, err := webrtc.NewTrackLocalStaticSample(webrtc.RTPCodecCapability{MimeType: webrtc.MimeTypeH264}, "video", "cell-camera")
	if err != nil {
		return nil, err
	}
	return &Sender{
		counters: counters,
		cfg:      cfg,
		api:      webrtc.NewAPI(webrtc.WithMediaEngine(m), webrtc.WithInterceptorRegistry(registry), webrtc.WithSettingEngine(s)),
		track:    track,
		http:     &http.Client{},
		viewers:  map[string]*viewer{},
	}, nil
}

// Refusal is an offer the sender will not serve, with its HTTP status.
type Refusal struct {
	Status int
	Reason string
}

func (r *Refusal) Error() string { return r.Reason }

// Offer answers one viewer's SDP offer and starts sending to it.
func (s *Sender) Offer(offer, client string) (id, answer string, err error) {
	if len(offer) > maxOfferBytes || !strings.Contains(offer, "m=video") {
		return "", "", &Refusal{http.StatusBadRequest, "expected an SDP offer with a video section"}
	}
	s.mu.Lock()
	full := len(s.viewers) >= maxViewers
	s.mu.Unlock()
	if full {
		return "", "", &Refusal{http.StatusServiceUnavailable, fmt.Sprintf("%d video viewers already; try again later", maxViewers)}
	}
	counter := &byteCounter{}
	s.counters.mu.Lock()
	s.counters.next = counter
	pc, err := s.api.NewPeerConnection(webrtc.Configuration{})
	s.counters.next = nil
	s.counters.mu.Unlock()
	if err != nil {
		return "", "", err
	}
	fail := func(status int, format string, args ...any) (string, string, error) {
		pc.Close()
		return "", "", &Refusal{status, fmt.Sprintf(format, args...)}
	}
	rtpSender, err := pc.AddTrack(s.track)
	if err != nil {
		return fail(http.StatusInternalServerError, "add track: %v", err)
	}
	if err := pc.SetRemoteDescription(webrtc.SessionDescription{Type: webrtc.SDPTypeOffer, SDP: offer}); err != nil {
		return fail(http.StatusBadRequest, "the offer was refused: %v", err)
	}
	if !sendsVideo(pc) {
		return fail(http.StatusNotAcceptable, "the offer has no H.264 constrained-baseline format with packetization-mode 1")
	}
	reply, err := pc.CreateAnswer(nil)
	if err != nil {
		return fail(http.StatusBadRequest, "no answer to this offer: %v", err)
	}
	gathered := webrtc.GatheringCompletePromise(pc)
	if err := pc.SetLocalDescription(reply); err != nil {
		return fail(http.StatusInternalServerError, "set local description: %v", err)
	}
	select {
	case <-gathered:
	case <-time.After(gatherTimeout):
	}
	id = newID()
	v := &viewer{id: id, client: client, pc: pc, sent: counter, created: time.Now()}
	pc.OnConnectionStateChange(func(state webrtc.PeerConnectionState) {
		s.mu.Lock()
		defer s.mu.Unlock()
		switch state {
		case webrtc.PeerConnectionStateConnected:
			v.connected, v.lostAt = true, time.Time{}
			s.keyframeLocked() // the viewer's picture starts at once
		case webrtc.PeerConnectionStateDisconnected, webrtc.PeerConnectionStateFailed, webrtc.PeerConnectionStateClosed:
			if v.lostAt.IsZero() {
				v.lostAt = time.Now()
			}
		}
	})
	s.mu.Lock()
	// Checked again here: offers made at the same moment all pass the check
	// above before any is added.
	if len(s.viewers) >= maxViewers {
		s.mu.Unlock()
		return fail(http.StatusServiceUnavailable, "%d video viewers already; try again later", maxViewers)
	}
	s.viewers[id] = v
	s.startSourceLocked()
	s.mu.Unlock()
	go s.readRTCP(rtpSender, v)
	return id, pc.LocalDescription().SDP, nil
}

// sendsVideo reports whether negotiation gave the track a codec, that is
// whether the offer had a format this sender sends.
func sendsVideo(pc *webrtc.PeerConnection) bool {
	for _, t := range pc.GetTransceivers() {
		if t.Sender() != nil && t.Direction() != webrtc.RTPTransceiverDirectionInactive {
			if params := t.Sender().GetParameters(); len(params.Codecs) > 0 {
				return true
			}
		}
	}
	return false
}

func newID() string {
	b := make([]byte, 8)
	rand.Read(b)
	return hex.EncodeToString(b)
}

// End removes a viewer; false when there is no such session.
func (s *Sender) End(id string) bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.removeLocked(id)
}

func (s *Sender) removeLocked(id string) bool {
	v, ok := s.viewers[id]
	if !ok {
		return false
	}
	delete(s.viewers, id)
	s.ended++
	go v.pc.Close()
	if len(s.viewers) == 0 && s.cancel != nil {
		s.cancel() // nobody watches: stop reading, and the camera's encoder idles
	}
	return true
}

// readRTCP answers a viewer's picture-loss and full-intra requests with a key
// frame from the camera; the interceptors handle NACK and reports.
func (s *Sender) readRTCP(sender *webrtc.RTPSender, v *viewer) {
	for {
		packets, _, err := sender.ReadRTCP()
		if err != nil {
			return
		}
		for _, p := range packets {
			switch p.(type) {
			case *rtcp.TransportLayerNack:
				v.nacks.Add(1)
			case *rtcp.PictureLossIndication, *rtcp.FullIntraRequest:
				s.mu.Lock()
				s.keyframeLocked()
				s.mu.Unlock()
			}
		}
	}
}

// keyframeLocked asks the camera for a key frame, at most once a second.
func (s *Sender) keyframeLocked() {
	if time.Since(s.lastKey) < keyframeInterval {
		return
	}
	s.lastKey = time.Now()
	go func() {
		resp, err := s.http.Post(s.cfg.Camera+"/video/keyframe", "text/plain", nil)
		if err == nil {
			io.Copy(io.Discard, resp.Body)
			resp.Body.Close()
		}
	}()
}

func (s *Sender) startSourceLocked() {
	if s.sourceOn {
		return
	}
	ctx, cancel := context.WithCancel(context.Background())
	s.sourceOn, s.cancel = true, cancel
	go s.source(ctx)
}

// source reads the camera's stream while viewers exist and writes each access
// unit to the shared track, which sends it to every viewer.
func (s *Sender) source(ctx context.Context) {
	defer func() {
		s.mu.Lock()
		s.sourceOn, s.cancel = false, nil
		if len(s.viewers) > 0 { // a viewer came while this one was stopping
			s.startSourceLocked()
		}
		s.mu.Unlock()
	}()
	for ctx.Err() == nil {
		err := s.readStream(ctx)
		s.mu.Lock()
		if err != nil && ctx.Err() == nil {
			s.sourceErr = err.Error()
		}
		s.mu.Unlock()
		select {
		case <-ctx.Done():
		case <-time.After(sourceRetry):
		}
	}
}

func (s *Sender) readStream(ctx context.Context) error {
	req, err := http.NewRequestWithContext(ctx, http.MethodGet, s.cfg.Camera+"/video.h264", nil)
	if err != nil {
		return err
	}
	resp, err := s.http.Do(req)
	if err != nil {
		return err
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		return fmt.Errorf("camera answered %s", resp.Status)
	}
	// The camera's encoded rate, not a setting of this program: the camera
	// lowers it to its capture rate, and a second guess here would stamp
	// samples at the wrong pace. A camera process started before the header
	// existed (the shared install link can put a new sender beside it until
	// its restart) sends none: its default was 15.
	header := resp.Header.Get("X-Video-FPS")
	if header == "" {
		header = "15"
	}
	fps, err := strconv.Atoi(header)
	if err != nil || fps < 1 || fps > 120 {
		return fmt.Errorf("camera stream has an invalid X-Video-FPS %q", header)
	}
	duration := time.Second / time.Duration(fps)
	s.mu.Lock()
	s.sourceErr = ""
	s.mu.Unlock()
	r := bufio.NewReaderSize(resp.Body, 256<<10)
	var head [4]byte
	for {
		if _, err := io.ReadFull(r, head[:]); err != nil {
			return err
		}
		n := binary.BigEndian.Uint32(head[:])
		if n == 0 || n > maxUnitBytes {
			return fmt.Errorf("camera sent a %d-byte unit", n)
		}
		unit := make([]byte, n)
		if _, err := io.ReadFull(r, unit); err != nil {
			return err
		}
		if err := s.track.WriteSample(media.Sample{Data: unit, Duration: duration}); err != nil && !errors.Is(err, io.ErrClosedPipe) {
			return err
		}
		s.mu.Lock()
		s.units++
		s.unitBytes += int64(n)
		s.mu.Unlock()
	}
}

// Sweep removes viewers that never connected or stayed lost; Run calls it
// every second.
func (s *Sender) Sweep(now time.Time) {
	s.mu.Lock()
	defer s.mu.Unlock()
	for id, v := range s.viewers {
		never := !v.connected && now.Sub(v.created) > connectTimeout
		lost := !v.lostAt.IsZero() && now.Sub(v.lostAt) > lostTimeout
		if never || lost {
			s.removeLocked(id)
		}
	}
}

// ViewerStatus is one viewer in /status.
type ViewerStatus struct {
	ID        string  `json:"id"`
	Client    string  `json:"client"`
	Connected bool    `json:"connected"`
	Seconds   float64 `json:"seconds"`
	BytesSent uint64  `json:"bytes_sent"`
	Packets   uint64  `json:"packets_sent"`
	NACKs     uint32  `json:"nacks"`
}

// Status is GET /status.
type Status struct {
	Address   string         `json:"address"`
	Port      int            `json:"port"`
	Reading   bool           `json:"reading"`
	Units     int64          `json:"units"`
	UnitBytes int64          `json:"unit_bytes"`
	Error     string         `json:"error,omitempty"`
	Ended     int64          `json:"sessions_ended"`
	Viewers   []ViewerStatus `json:"viewers"`
}

func (s *Sender) Status() Status {
	s.mu.Lock()
	st := Status{Address: s.cfg.Address.String(), Port: s.cfg.Port, Reading: s.sourceOn, Units: s.units, UnitBytes: s.unitBytes,
		Error: s.sourceErr, Ended: s.ended, Viewers: []ViewerStatus{}}
	// Under the lock: OnConnectionStateChange writes v.connected under it.
	for _, v := range s.viewers {
		st.Viewers = append(st.Viewers, ViewerStatus{ID: v.id, Client: v.client, Connected: v.connected,
			Seconds: time.Since(v.created).Seconds(), BytesSent: v.sent.bytes.Load(), Packets: v.sent.packets.Load(), NACKs: v.nacks.Load()})
	}
	s.mu.Unlock()
	return st
}

// Close ends every session.
func (s *Sender) Close() {
	s.mu.Lock()
	defer s.mu.Unlock()
	for id := range s.viewers {
		s.removeLocked(id)
	}
}

func logf(format string, args ...any) { log.Printf("cell-video: "+format, args...) }
