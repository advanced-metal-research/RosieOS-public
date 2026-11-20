package main

import (
	"bytes"
	"encoding/binary"
	"encoding/json"
	"io"
	"net"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"github.com/pion/rtp/codecs"
	"github.com/pion/webrtc/v4"
)

// fakeCamera serves video.h264 like the camera service: framed Annex-B access
// units at 15 a second, a key frame (SPS, PPS, IDR) every 30th, and counts
// key frame requests and open streams.
type fakeCamera struct {
	keyframes atomic.Int32
	open      atomic.Int32
	served    atomic.Int32
}

func unit(key bool, n int) []byte {
	var b bytes.Buffer
	if key {
		b.Write([]byte{0, 0, 0, 1, 0x67, 0x42, 0xe0, 0x1f, 1, 2, 3}) // SPS, constrained baseline 3.1
		b.Write([]byte{0, 0, 0, 1, 0x68, 0xce, 0x38, 0x80})          // PPS
		b.Write([]byte{0, 0, 0, 1, 0x65})                            // IDR slice
	} else {
		b.Write([]byte{0, 0, 0, 1, 0x41}) // non-IDR slice
	}
	b.Write(bytes.Repeat([]byte{byte(n)}, 3000)) // spans several RTP packets
	return b.Bytes()
}

func (c *fakeCamera) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	switch r.URL.Path {
	case "/camera/video/keyframe":
		c.keyframes.Add(1)
	case "/camera/video.h264":
		c.open.Add(1)
		defer c.open.Add(-1)
		w.Header().Set("X-Video-FPS", "15")
		flusher := w.(http.Flusher)
		for n := 0; ; n++ {
			u := unit(n%30 == 0, n)
			var head [4]byte
			binary.BigEndian.PutUint32(head[:], uint32(len(u)))
			if _, err := w.Write(append(head[:], u...)); err != nil {
				return
			}
			flusher.Flush()
			c.served.Add(1)
			select {
			case <-r.Context().Done():
				return
			case <-time.After(time.Second / 15):
			}
		}
	default:
		http.NotFound(w, r)
	}
}

// viewerPC offers to receive H.264 the way a browser does and counts the
// access units it reassembles (an RTP marker bit ends one).
func viewerPC(t *testing.T) (*webrtc.PeerConnection, *atomic.Int32, *atomic.Bool) {
	t.Helper()
	m := &webrtc.MediaEngine{}
	for _, c := range []struct {
		pt   webrtc.PayloadType
		fmtp string
	}{
		{96, "level-asymmetry-allowed=1;packetization-mode=0;profile-level-id=42001f"},
		{102, "level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42e01f"},
		{127, "level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=640032"},
	} {
		if err := m.RegisterCodec(webrtc.RTPCodecParameters{RTPCodecCapability: webrtc.RTPCodecCapability{
			MimeType: webrtc.MimeTypeH264, ClockRate: 90000, SDPFmtpLine: c.fmtp}, PayloadType: c.pt}, webrtc.RTPCodecTypeVideo); err != nil {
			t.Fatal(err)
		}
	}
	var s webrtc.SettingEngine
	s.SetIncludeLoopbackCandidate(true)
	s.SetNetworkTypes([]webrtc.NetworkType{webrtc.NetworkTypeUDP4})
	s.SetIPFilter(func(ip net.IP) bool { return ip.IsLoopback() })
	pc, err := webrtc.NewAPI(webrtc.WithMediaEngine(m), webrtc.WithSettingEngine(s)).NewPeerConnection(webrtc.Configuration{})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := pc.AddTransceiverFromKind(webrtc.RTPCodecTypeVideo, webrtc.RTPTransceiverInit{Direction: webrtc.RTPTransceiverDirectionRecvonly}); err != nil {
		t.Fatal(err)
	}
	units, sawKey := &atomic.Int32{}, &atomic.Bool{}
	pc.OnTrack(func(track *webrtc.TrackRemote, _ *webrtc.RTPReceiver) {
		var depay codecs.H264Packet
		for {
			p, _, err := track.ReadRTP()
			if err != nil {
				return
			}
			data, err := depay.Unmarshal(p.Payload)
			if err == nil && bytes.Contains(data, []byte{0, 0, 0, 1, 0x65}) {
				sawKey.Store(true)
			}
			if p.Marker {
				units.Add(1)
			}
		}
	})
	return pc, units, sawKey
}

func TestAViewerGetsTheCameraStreamOverWebRTC(t *testing.T) {
	camera := &fakeCamera{}
	cameraServer := httptest.NewServer(camera)
	defer cameraServer.Close()
	conn, err := net.ListenUDP("udp4", &net.UDPAddr{IP: net.IPv4(127, 0, 0, 1)})
	if err != nil {
		t.Fatal(err)
	}
	defer conn.Close()
	sender, err := New(Config{Camera: cameraServer.URL + "/camera", Address: net.IPv4(127, 0, 0, 1), Port: conn.LocalAddr().(*net.UDPAddr).Port}, conn)
	if err != nil {
		t.Fatal(err)
	}
	defer sender.Close()
	whep := httptest.NewServer(sender.Handler())
	defer whep.Close()

	pc, units, sawKey := viewerPC(t)
	defer pc.Close()
	offer, err := pc.CreateOffer(nil)
	if err != nil {
		t.Fatal(err)
	}
	gathered := webrtc.GatheringCompletePromise(pc)
	pc.SetLocalDescription(offer)
	<-gathered
	req, _ := http.NewRequest(http.MethodPost, whep.URL+"/whep", strings.NewReader(pc.LocalDescription().SDP))
	req.Header.Set("Content-Type", "application/sdp")
	req.Header.Set("X-Rosie-Client", "rosie-pendant steamdeck-viewer")
	resp, err := http.DefaultClient.Do(req)
	if err != nil {
		t.Fatal(err)
	}
	answer, _ := io.ReadAll(resp.Body)
	resp.Body.Close()
	if resp.StatusCode != http.StatusCreated || !strings.HasPrefix(resp.Header.Get("Location"), "/whep/") {
		t.Fatalf("WHEP answered %d %q: %s", resp.StatusCode, resp.Header.Get("Location"), answer)
	}
	for _, line := range strings.Split(string(answer), "\n") {
		if strings.HasPrefix(line, "a=candidate:") && (!strings.Contains(line, "127.0.0.1") || !strings.Contains(strings.ToLower(line), " udp ")) {
			t.Errorf("candidate off the configured address or not UDP: %s", line)
		}
		if strings.HasPrefix(line, "a=rtpmap:") && strings.Contains(line, "H264") && !strings.HasPrefix(line, "a=rtpmap:102 ") {
			t.Errorf("the answer chose another H.264 format than constrained baseline mode 1: %s", line)
		}
	}
	if err := pc.SetRemoteDescription(webrtc.SessionDescription{Type: webrtc.SDPTypeAnswer, SDP: string(answer)}); err != nil {
		t.Fatal(err)
	}
	deadline := time.Now().Add(10 * time.Second)
	for units.Load() < 30 && time.Now().Before(deadline) {
		time.Sleep(50 * time.Millisecond)
	}
	if units.Load() < 30 || !sawKey.Load() {
		t.Fatalf("viewer reassembled %d units (key frame seen: %v); camera served %d", units.Load(), sawKey.Load(), camera.served.Load())
	}
	if camera.keyframes.Load() == 0 {
		t.Errorf("the sender did not ask the camera for a key frame when the viewer connected")
	}
	status := sender.Status()
	if len(status.Viewers) != 1 || !status.Viewers[0].Connected || status.Viewers[0].Client != "rosie-pendant steamdeck-viewer" ||
		status.Viewers[0].BytesSent < 30*3000 || !status.Reading {
		raw, _ := json.Marshal(status)
		t.Fatalf("status %s", raw)
	}

	location := resp.Header.Get("Location")
	del, _ := http.NewRequest(http.MethodDelete, whep.URL+location, nil)
	if r, err := http.DefaultClient.Do(del); err != nil || r.StatusCode != http.StatusOK {
		t.Fatalf("DELETE %s: %v %v", location, err, r)
	}
	deadline = time.Now().Add(5 * time.Second)
	for camera.open.Load() > 0 && time.Now().Before(deadline) {
		time.Sleep(50 * time.Millisecond)
	}
	if camera.open.Load() != 0 {
		t.Fatal("the sender kept reading the camera after its last viewer left")
	}
	if st := sender.Status(); len(st.Viewers) != 0 || st.Ended != 1 {
		t.Fatalf("after DELETE: %+v", st)
	}
}

// Offers made at the same moment cannot pass maxViewers: the bound protects
// the shared UDP socket and the host's CPU. Status runs alongside them (go
// test -race checks its reads of each viewer).
func TestParallelOffersStayWithinMaxViewers(t *testing.T) {
	conn, err := net.ListenUDP("udp4", &net.UDPAddr{IP: net.IPv4(127, 0, 0, 1)})
	if err != nil {
		t.Fatal(err)
	}
	defer conn.Close()
	sender, err := New(Config{Camera: "http://127.0.0.1:1/camera", Address: net.IPv4(127, 0, 0, 1), Port: conn.LocalAddr().(*net.UDPAddr).Port}, conn)
	if err != nil {
		t.Fatal(err)
	}
	defer sender.Close()
	offers := make([]string, maxViewers+4)
	for i := range offers {
		pc, _, _ := viewerPC(t)
		defer pc.Close()
		offer, _ := pc.CreateOffer(nil)
		offers[i] = offer.SDP
	}
	var accepted atomic.Int32
	var wg sync.WaitGroup
	for _, offer := range offers {
		wg.Add(1)
		go func() {
			defer wg.Done()
			if _, _, err := sender.Offer(offer, "test"); err == nil {
				accepted.Add(1)
			}
			sender.Status()
		}()
	}
	wg.Wait()
	if n := accepted.Load(); n != maxViewers || len(sender.Status().Viewers) != maxViewers {
		t.Fatalf("accepted %d offers, %d viewers; want %d", n, len(sender.Status().Viewers), maxViewers)
	}
}

func TestAnOfferWithoutUsableH264IsRefused(t *testing.T) {
	conn, _ := net.ListenUDP("udp4", &net.UDPAddr{IP: net.IPv4(127, 0, 0, 1)})
	defer conn.Close()
	sender, err := New(Config{Camera: "http://127.0.0.1:1/camera", Address: net.IPv4(127, 0, 0, 1)}, conn)
	if err != nil {
		t.Fatal(err)
	}
	m := &webrtc.MediaEngine{}
	m.RegisterCodec(webrtc.RTPCodecParameters{RTPCodecCapability: webrtc.RTPCodecCapability{MimeType: webrtc.MimeTypeVP8, ClockRate: 90000}, PayloadType: 96}, webrtc.RTPCodecTypeVideo)
	pc, _ := webrtc.NewAPI(webrtc.WithMediaEngine(m)).NewPeerConnection(webrtc.Configuration{})
	defer pc.Close()
	pc.AddTransceiverFromKind(webrtc.RTPCodecTypeVideo, webrtc.RTPTransceiverInit{Direction: webrtc.RTPTransceiverDirectionRecvonly})
	offer, _ := pc.CreateOffer(nil)
	_, _, err = sender.Offer(offer.SDP, "test")
	refusal, ok := err.(*Refusal)
	if !ok || refusal.Status != http.StatusNotAcceptable {
		t.Fatalf("a VP8-only offer: %v", err)
	}
}

func TestConfigComesFromTheCameraSettings(t *testing.T) {
	env := map[string]string{"CELL_CAMERA_VIDEO_ADDRESS": "100.64.0.11", "CELL_CAMERA_VIDEO_LISTEN": "127.0.0.1:8090",
		"CELL_CAMERA_LISTEN": "127.0.0.1:8088", "CELL_CAMERA_PATH_PREFIX": "/camera"}
	c, err := configFrom(func(k string) string { return env[k] })
	if err != nil || c.Camera != "http://127.0.0.1:8088/camera" || c.Port != 40400 || !c.Address.Equal(net.ParseIP("100.64.0.11")) {
		t.Fatalf("%+v %v", c, err)
	}
	for key, bad := range map[string]string{"CELL_CAMERA_VIDEO_ADDRESS": "tailscale", "CELL_CAMERA_VIDEO_LISTEN": "0.0.0.0:8090", "CELL_CAMERA_VIDEO_PORT": "80"} {
		copied := map[string]string{}
		for k, v := range env {
			copied[k] = v
		}
		copied[key] = bad
		if _, err := configFrom(func(k string) string { return copied[k] }); err == nil {
			t.Errorf("%s=%s was accepted", key, bad)
		}
	}
}
