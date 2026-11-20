// Package viewer receives the cell camera's WebRTC video for a native client
// (cmd/cell-video-view). It needs a kernel route to the cell's Tailscale
// address: the Steam Deck pendant does not use it while the Decks' Tailscale
// runs in userspace-networking mode (a Steam Deck, 2026-10-19).
//
// Why: the Deck pulled the cell camera as JPEG frames through its SSH tunnel,
// about 12.7 GB an hour on a Rosie 1430 arm (2026-10-16). The browser already plays the
// sender's low-data H.264 stream (cell-video, about 800 kbit/s); this is the
// same session for a client without a browser's WebRTC stack. Signalling is
// WHEP through the OLP server (the tunnel); media is UDP straight to the
// cell's Tailscale address, outside the tunnel that carries jog control.
package viewer

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"io"
	"net/http"
	"net/url"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"github.com/pion/interceptor"
	"github.com/pion/rtcp"
	"github.com/pion/webrtc/v4"
)

// Unverified bounds. The sender removes a viewer that has not connected 15 s
// after its answer; this gives up first and lets its caller start again. A
// connection lost this long is over (Pion reports disconnected after about
// 5 s without traffic).
const (
	connectTimeout = 12 * time.Second
	lostTimeout    = 8 * time.Second
	keyInterval    = time.Second // key frame requests, at most
	requestTimeout = 10 * time.Second
	maxAnswerBytes = 64 << 10
)

// ErrNoVideo is a cell that serves no video: no sender configured (404), or
// one that refused this client's formats (406). The caller shows JPEG frames.
var ErrNoVideo = errors.New("the cell serves no video to this client")

// Options name the cell's WHEP endpoint and this client.
type Options struct {
	WHEP   string      // for example http://127.0.0.1:18795/api/offline-programming/v1/cell-camera/video/whep
	Header http.Header // sent with every WHEP request (X-Rosie-Client and its kin)
	// Loopback lets ICE use loopback addresses: tests only.
	Loopback bool
}

// Stats are the bytes and units this session received.
type Stats struct {
	Bytes, Packets, Units, Dropped, KeyRequests uint64
}

// Session is one viewer's WebRTC session.
type Session struct {
	opts    Options
	pc      *webrtc.PeerConnection
	ssrc    atomic.Uint32
	lastKey atomic.Int64
	stats   struct{ bytes, packets, units, dropped, keys atomic.Uint64 }
}

// Run offers to receive the cell camera, then calls deliver with each Annex-B
// access unit, a key frame first, until ctx ends (nil), the connection fails
// or deliver returns an error. It ends the WHEP session on return. A request
// on keyframe asks the sender for a key frame (the decoder lost its place).
func Run(ctx context.Context, opts Options, keyframe <-chan struct{}, deliver func([]byte) error) (*Session, error) {
	s := &Session{opts: opts}
	pc, err := newPeerConnection(opts)
	if err != nil {
		return s, err
	}
	s.pc = pc
	defer pc.Close()
	if _, err := pc.AddTransceiverFromKind(webrtc.RTPCodecTypeVideo, webrtc.RTPTransceiverInit{Direction: webrtc.RTPTransceiverDirectionRecvonly}); err != nil {
		return s, err
	}
	offer, err := pc.CreateOffer(nil)
	if err != nil {
		return s, err
	}
	gathered := webrtc.GatheringCompletePromise(pc)
	if err := pc.SetLocalDescription(offer); err != nil {
		return s, err
	}
	select {
	case <-gathered:
	case <-time.After(2 * time.Second):
	case <-ctx.Done():
		return s, nil
	}
	answer, location, err := s.post(ctx, pc.LocalDescription().SDP)
	if err != nil {
		return s, err
	}
	defer s.end(location)
	if err := pc.SetRemoteDescription(webrtc.SessionDescription{Type: webrtc.SDPTypeAnswer, SDP: answer}); err != nil {
		return s, fmt.Errorf("the sender's answer was refused: %w", err)
	}

	failed := make(chan error, 2)
	var once sync.Once
	fail := func(err error) { once.Do(func() { failed <- err }) }
	var lostAt atomic.Int64
	connected := make(chan struct{})
	var connectedOnce sync.Once
	pc.OnConnectionStateChange(func(state webrtc.PeerConnectionState) {
		switch state {
		case webrtc.PeerConnectionStateConnected:
			lostAt.Store(0)
			connectedOnce.Do(func() { close(connected) })
		case webrtc.PeerConnectionStateDisconnected:
			lostAt.CompareAndSwap(0, time.Now().UnixNano())
		case webrtc.PeerConnectionStateFailed, webrtc.PeerConnectionStateClosed:
			fail(fmt.Errorf("the video connection %s", state))
		}
	})
	pc.OnTrack(func(track *webrtc.TrackRemote, _ *webrtc.RTPReceiver) {
		s.ssrc.Store(uint32(track.SSRC()))
		s.requestKey(true)
		a := newAssembler()
		for {
			p, _, err := track.ReadRTP()
			if err != nil {
				fail(fmt.Errorf("reading video: %w", err))
				return
			}
			s.stats.packets.Add(1)
			s.stats.bytes.Add(uint64(p.MarshalSize()))
			units, lost := a.push(p, time.Now())
			if lost {
				s.stats.dropped.Add(1)
				s.requestKey(false)
			}
			for _, unit := range units {
				s.stats.units.Add(1)
				if err := deliver(unit); err != nil {
					fail(err)
					return
				}
			}
		}
	})

	select {
	case <-connected:
	case err := <-failed:
		return s, err
	case <-ctx.Done():
		return s, nil
	case <-time.After(connectTimeout):
		return s, fmt.Errorf("no video connection within %s: UDP to the cell's Tailscale address may be blocked", connectTimeout)
	}
	tick := time.NewTicker(time.Second)
	defer tick.Stop()
	for {
		select {
		case <-ctx.Done():
			return s, nil
		case err := <-failed:
			return s, err
		case <-keyframe:
			s.requestKey(false)
		case <-tick.C:
			if at := lostAt.Load(); at != 0 && time.Since(time.Unix(0, at)) > lostTimeout {
				return s, fmt.Errorf("the video connection was lost for %s", lostTimeout)
			}
		}
	}
}

// Stats reports what this session received so far.
func (s *Session) Stats() Stats {
	return Stats{Bytes: s.stats.bytes.Load(), Packets: s.stats.packets.Load(), Units: s.stats.units.Load(),
		Dropped: s.stats.dropped.Load(), KeyRequests: s.stats.keys.Load()}
}

// requestKey sends the sender a picture loss indication, at most once a
// second unless forced (the first one, when the track starts).
func (s *Session) requestKey(force bool) {
	now := time.Now().UnixNano()
	if last := s.lastKey.Load(); !force && now-last < int64(keyInterval) {
		return
	}
	s.lastKey.Store(now)
	if ssrc := s.ssrc.Load(); ssrc != 0 && s.pc != nil {
		if s.pc.WriteRTCP([]rtcp.Packet{&rtcp.PictureLossIndication{MediaSSRC: ssrc}}) == nil {
			s.stats.keys.Add(1)
		}
	}
}

func newPeerConnection(opts Options) (*webrtc.PeerConnection, error) {
	m := &webrtc.MediaEngine{}
	feedback := []webrtc.RTCPFeedback{{Type: "nack"}, {Type: "nack", Parameter: "pli"}}
	// The one format the sender sends (cell-video sender.go).
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
	var settings webrtc.SettingEngine
	settings.SetNetworkTypes([]webrtc.NetworkType{webrtc.NetworkTypeUDP4, webrtc.NetworkTypeUDP6})
	if opts.Loopback {
		settings.SetIncludeLoopbackCandidate(true)
	}
	return webrtc.NewAPI(webrtc.WithMediaEngine(m), webrtc.WithInterceptorRegistry(registry), webrtc.WithSettingEngine(settings)).
		NewPeerConnection(webrtc.Configuration{})
}

// post sends the offer and returns the answer and the session's URL.
func (s *Session) post(ctx context.Context, offer string) (answer, location string, err error) {
	ctx, cancel := context.WithTimeout(ctx, requestTimeout)
	defer cancel()
	req, err := http.NewRequestWithContext(ctx, http.MethodPost, s.opts.WHEP, strings.NewReader(offer))
	if err != nil {
		return "", "", err
	}
	for k, v := range s.opts.Header {
		req.Header[k] = v
	}
	req.Header.Set("Content-Type", "application/sdp")
	resp, err := http.DefaultClient.Do(req)
	if err != nil {
		return "", "", fmt.Errorf("WHEP offer: %w", err)
	}
	defer resp.Body.Close()
	body, _ := io.ReadAll(io.LimitReader(resp.Body, maxAnswerBytes))
	switch {
	case resp.StatusCode == http.StatusNotFound || resp.StatusCode == http.StatusNotAcceptable:
		return "", "", fmt.Errorf("%w (%s: %s)", ErrNoVideo, resp.Status, bytes.TrimSpace(body))
	case resp.StatusCode != http.StatusCreated:
		return "", "", fmt.Errorf("WHEP offer answered %s: %s", resp.Status, bytes.TrimSpace(body))
	}
	base, err := url.Parse(s.opts.WHEP)
	if err != nil {
		return "", "", err
	}
	ref, err := url.Parse(resp.Header.Get("Location"))
	if err != nil || resp.Header.Get("Location") == "" {
		return "", "", fmt.Errorf("WHEP answer has no session Location")
	}
	return string(body), base.ResolveReference(ref).String(), nil
}

// end deletes the WHEP session, so the sender stops at once rather than when
// its timeout notices the silence.
func (s *Session) end(location string) {
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	req, err := http.NewRequestWithContext(ctx, http.MethodDelete, location, nil)
	if err != nil {
		return
	}
	for k, v := range s.opts.Header {
		req.Header[k] = v
	}
	if resp, err := http.DefaultClient.Do(req); err == nil {
		io.Copy(io.Discard, resp.Body)
		resp.Body.Close()
	}
}
