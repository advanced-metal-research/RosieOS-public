//go:build linux

package control

import (
	"context"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"net"
	"net/http"
	"strconv"
	"time"

	"rosieos/rt-core/ipcclient"
)

type remoteJogConnection struct {
	ws         *websocket
	fence      Fence
	generation uint64
}
type remoteJogClock struct {
	mapping    JogClockMapping
	binding    JogClockBinding
	limits     JogClockLimits
	sent       uint64
	lastSeq    uint64
	lastSource uint64
	previous   *JogClockAdmission
}
type jogCalibration struct {
	Type              string `json:"type"`
	SourceIncarnation string `json:"source_incarnation"`
	SourceNS          uint64 `json:"source_ns"`
	Seq               uint64 `json:"seq"`
	HostTXNS          uint64 `json:"host_tx_ns,omitempty"`
	AxisMask          uint32 `json:"axis_mask,omitempty"`
	Path              bool   `json:"path,omitempty"` // begin a path jog session (protocol 1.14)
}

// calibrate requires a second source sample taken after receipt of the previous
// reply, echoing its host_tx_ns. A one-way timestamp cannot establish a lower
// offset bound and remains unqualified. The source sample represents both receive
// and send in the four-timestamp policy, conservatively including source delay.
func (c *remoteJogClock) calibrate(message jogCalibration, rx uint64) error {
	if message.Type != "calibrate" || message.Seq == 0 || message.Seq <= c.lastSeq || message.SourceNS == 0 {
		return ErrJogClockInvalid
	}
	raw, err := hex.DecodeString(message.SourceIncarnation)
	if err != nil || len(raw) != 16 {
		return ErrJogClockIdentity
	}
	var source JogClockIncarnation
	copy(source[:], raw)
	if source == (JogClockIncarnation{}) || (c.binding.Source != (JogClockIncarnation{}) && source != c.binding.Source) {
		return ErrJogClockIdentity
	}
	c.binding.Source = source
	c.lastSeq = message.Seq
	c.lastSource = message.SourceNS
	if c.sent == 0 || message.HostTXNS != c.sent {
		return ErrJogClockUnqualified
	}
	exchange := JogClockExchange{HostSendNS: c.sent, SourceReceiveNS: message.SourceNS, SourceSendNS: message.SourceNS, HostReceiveNS: rx}
	var mapping JogClockMapping
	if c.mapping.qualified {
		mapping, err = c.mapping.Refresh(c.binding, exchange)
	} else {
		mapping, err = NewRemoteJogClock(c.binding, 1, exchange, c.limits)
	}
	if err != nil {
		c.mapping = JogClockMapping{}
		return err
	}
	c.mapping = mapping
	return nil
}

// convert retains source expiry and the earliest possible origin. Recalibration
// and grant renewal cannot extend an already accepted sample's native deadline.
func (c *remoteJogClock) convert(frame ipcclient.LocalJogUpdate, now, grantDeadline uint64) (ipcclient.LocalJogUpdate, JogClockAdmission, error) {
	if !c.mapping.qualified {
		return frame, JogClockAdmission{}, ErrJogClockUnqualified
	}
	if JogClockIncarnation(frame.ClockIncarnation) != c.binding.Source {
		return frame, JogClockAdmission{}, ErrJogClockIdentity
	}
	if frame.DeadlineHostNs <= frame.OriginHostNs {
		return frame, JogClockAdmission{}, ErrJogClockInvalid
	}
	admission, err := c.mapping.Admit(JogClockInput{Binding: c.binding, MappingGeneration: c.mapping.Generation(), SourceOriginNS: frame.OriginHostNs,
		HostNowNS: now, RequestedDurationNS: frame.DeadlineHostNs - frame.OriginHostNs, GrantExpiresHostNS: grantDeadline}, c.previous)
	if err != nil {
		return frame, admission, err
	}
	frame.OriginHostNs = admission.EarliestHostOriginNS()
	frame.DeadlineHostNs = admission.DeadlineNS()
	frame.ClockIncarnation = [16]byte(c.binding.Host)
	return frame, admission, nil
}

// remoteJogGrant pre-checks every stream frame; a failure closes the stream.
// The observation is read outside c.mu and a busy slot is retried within the
// transient window: one torn read must not end a hold. Publication repeats
// every check under c.mu, so the pre-check grants nothing by itself.
func (c *Controller) remoteJogGrant(f Fence, generation uint64) (context.Context, uint64, error) {
	c.mu.Lock()
	if err := c.authorize(f); err != nil {
		c.mu.Unlock()
		return nil, 0, err
	}
	// Generation zero reserves only authenticated transport, never native motion.
	if generation != 0 && (c.localJog == nil || c.localJog.Identity.JogGeneration != generation) {
		c.mu.Unlock()
		return nil, 0, errors.New("jog_session_stale")
	}
	ctx, expires := c.sessionCtx, c.expires
	c.mu.Unlock()
	observation, err := c.jogStatusWithinRetry()
	if err != nil {
		return nil, 0, err
	}
	if generation != 0 && (observation.Open != 1 || observation.JogGeneration != generation) {
		return nil, 0, errors.New("jog_session_stale")
	}
	now := ipcclient.HostMonotonicNS()
	remaining := time.Until(expires)
	if remaining <= 0 {
		return nil, 0, errors.New("control_session_stale")
	}
	return ctx, now + uint64(remaining), nil
}

// serveJog has one reader/publication goroutine per connection. Grant cancellation
// reports a typed refusal before closing, interrupting partial input without replay.
func (h *RemoteHandler) serveJog(w http.ResponseWriter, r *http.Request) {
	refuseHTTP := func(err error, status int) {
		h.controller.recordJogIngress(0, jogIngressReason(err), jogRefusal{transport: "wss_setup", cause: err})
		http.Error(w, err.Error(), status)
	}
	generation, err := strconv.ParseUint(r.URL.Query().Get("jog_generation"), 10, 64)
	grant, grantErr := strconv.ParseUint(r.URL.Query().Get("generation"), 10, 64)
	fence := Fence{r.URL.Query().Get("session"), grant}
	if err := h.controller.authorizePrincipal(r.Context(), fence); err != nil {
		refuseHTTP(err, http.StatusForbidden)
		return
	}
	if err != nil || grantErr != nil {
		refuseHTTP(errors.New(ReasonJogSessionStale), http.StatusConflict)
		return
	}
	h.mu.Lock()
	if h.closed {
		h.mu.Unlock()
		refuseHTTP(errors.New(ReasonIndependentJogUnavailable), http.StatusServiceUnavailable)
		return
	}
	ctx, _, err := h.controller.remoteJogGrant(fence, generation)
	if err != nil {
		h.mu.Unlock()
		refuseHTTP(err, http.StatusConflict)
		return
	}
	ws, err := upgradeWebsocket(w, r)
	if err != nil {
		h.mu.Unlock()
		h.controller.recordJogIngress(0, ipcclient.JogReasonInvalidVector)
		return
	}
	if h.active != nil {
		older := h.active
		h.active = nil
		if generation != 0 {
			_ = h.controller.EndJog(fence, generation)
		}
		older.ws.close(1008)
		ws.close(1008)
		h.mu.Unlock()
		h.controller.recordJogIngress(0, ipcclient.JogReasonWrongIdentity)
		return
	}
	connection := &remoteJogConnection{ws: ws, fence: fence, generation: generation}
	h.active = connection
	h.mu.Unlock()
	writeRejection := func(seq uint64, reason error) error {
		body, _ := json.Marshal(struct {
			Type   string `json:"type"`
			Seq    uint64 `json:"seq"`
			Reason string `json:"reason"`
		}{"rejected", seq, reason.Error()})
		return ws.writeFrame(1, body)
	}
	reject := func(seq uint64, reason error) error {
		h.controller.recordJogIngress(seq, jogIngressReason(reason), jogRefusal{transport: "wss", generation: generation, cause: reason})
		return writeRejection(seq, reason)
	}
	cancelDone := make(chan struct{})
	stopCancel := context.AfterFunc(ctx, func() {
		defer close(cancelDone)
		_ = reject(0, errors.New(ReasonControlSessionStale))
		ws.conn.Close()
	})
	defer func() {
		// If cancellation started, let its bounded write finish before cleanup
		// closes the socket; otherwise a racing reader could discard the reason.
		if !stopCancel() {
			<-cancelDone
		} else if ctx.Err() != nil {
			_ = reject(0, errors.New(ReasonControlSessionStale))
		}
		ws.conn.Close()
		if generation != 0 {
			_ = h.controller.EndJog(fence, generation)
		}
		h.mu.Lock()
		if h.active == connection {
			h.active = nil
		}
		h.mu.Unlock()
	}()
	clock, err := h.controller.JogClock()
	if err != nil {
		_ = reject(0, err)
		return
	}
	host, err := hex.DecodeString(clock.Incarnation)
	if err != nil || len(host) != 16 {
		_ = reject(0, ErrJogClockIdentity)
		return
	}
	model := remoteJogClock{limits: h.limits}
	copy(model.binding.Host[:], host)
	var buffer [websocketPayloadLimit]byte
	var mapped ipcclient.LocalJogUpdate // Reused storage also backs the native velocity slice.
	var previous JogClockAdmission
	prepared := false
	var sourceCutoff uint64
	// Path chunk acknowledgements are written by their own goroutine as the
	// writer reports them; the read loop never waits for native.
	var acks chan pathAck
	acksDone := make(chan struct{})
	defer close(acksDone)
	for {
		// The cell freshness ceiling bounds stalled peers independently of narrower clock qualification.
		idle := time.Duration(h.controller.maxJogInputAge())
		if generation == 0 {
			// Unverified setup ceiling: 5000 ms; grant cancellation still closes setup.
			idle = 5 * time.Second
		}
		_ = ws.conn.SetReadDeadline(time.Now().Add(idle))
		opcode, payload, err := ws.readFrame(buffer[:])
		if ctx.Err() != nil {
			return
		}
		if err != nil {
			var protocol *websocketError
			var network net.Error
			if errors.As(err, &network) && network.Timeout() {
				_ = reject(0, errors.New(ReasonJogStreamIdle))
			} else if errors.As(err, &protocol) {
				h.controller.recordJogIngress(0, ipcclient.JogReasonInvalidVector)
				ws.close(protocol.code)
			}
			return
		}
		rx := ipcclient.HostMonotonicNS()
		switch opcode {
		case 8:
			_ = ws.writeFrame(8, payload)
			return
		case 9:
			if ws.writeFrame(10, payload) != nil {
				return
			}
		case 10:
		case 1:
			var message jogCalibration
			if json.Unmarshal(payload, &message) != nil {
				if reject(0, ErrJogClockInvalid) != nil {
					return
				}
				continue
			}
			if message.Type == "begin" {
				// Only the post-calibration acknowledgement starts native first-input
				// expiry. It carries no captured input and cannot refresh a generation.
				if generation != 0 || !prepared || message.Seq != 3 || message.HostTXNS != model.sent ||
					message.SourceNS == 0 || message.SourceNS < model.lastSource {
					_ = reject(message.Seq, ErrJogClockInvalid)
					return
				}
				now := ipcclient.HostMonotonicNS()
				age := h.controller.maxJogInputAge()
				if now > ^uint64(0)-age {
					_ = reject(message.Seq, ErrJogClockInvalid)
					return
				}
				// Map the producer's post-calibration Begin sample as the cutoff.
				// Comparing input against receipt time would reject fresh captures
				// whose uncertainty interval starts before this Begin request.
				origin := now
				if model.mapping.qualified {
					lo, _, mappingErr := model.mapping.intervalAt(message.SourceNS)
					var ok bool
					origin, ok = jogClockAddOffset(message.SourceNS, lo)
					if mappingErr != nil || !ok || origin == 0 || origin > now || now-origin >= age {
						_ = reject(message.Seq, ErrJogClockInvalid)
						return
					}
				}
				if message.Path {
					generation, err = h.controller.BeginPathJog(fence, message.AxisMask, origin, now+age)
				} else {
					generation, err = h.controller.BeginJog(fence, message.AxisMask, origin, now+age)
				}
				if err != nil {
					_ = reject(message.Seq, err)
					return
				}
				if message.Path {
					acks = make(chan pathAck, pathQueueChunks)
					h.controller.setPathSink(generation, acks)
					go func(acks <-chan pathAck) {
						for {
							select {
							case <-acksDone:
								return
							case ack := <-acks:
								body, _ := json.Marshal(struct {
									Type   string `json:"type"`
									Seq    uint64 `json:"seq"`
									Reason uint32 `json:"reason"`
								}{"path_ack", ack.sequence, ack.reason})
								if ws.writeFrame(1, body) != nil {
									return
								}
							}
						}
					}(acks)
				}
				sourceCutoff = message.SourceNS
				h.mu.Lock()
				connection.generation = generation
				h.mu.Unlock()
				body, _ := json.Marshal(struct {
					Type       string `json:"type"`
					Seq        uint64 `json:"seq"`
					Generation uint64 `json:"jog_generation"`
				}{"begun", message.Seq, generation})
				if ws.writeFrame(1, body) != nil {
					return
				}
				continue
			}
			acknowledged := message.Seq == 2 && model.lastSeq == 1 && model.sent != 0 && message.HostTXNS == model.sent
			prepared = false
			err := model.calibrate(message, rx)
			if err != nil && !errors.Is(err, ErrJogClockUnqualified) {
				if reject(message.Seq, err) != nil {
					return
				}
				continue
			}
			prepared = acknowledged
			model.sent = ipcclient.HostMonotonicNS()
			body, _ := json.Marshal(struct {
				Type     string `json:"type"`
				Seq      uint64 `json:"seq"`
				SourceNS uint64 `json:"source_ns"`
				HostRXNS uint64 `json:"host_rx_ns"`
				HostTXNS uint64 `json:"host_tx_ns"`
			}{"calibrated", message.Seq, message.SourceNS, rx, model.sent})
			if ws.writeFrame(1, body) != nil {
				return
			}
		case 2:
			if generation == 0 {
				_ = reject(0, errors.New(ReasonJogSessionStale))
				return
			}
			if len(payload) == ipcclient.LocalPathSamplesBytes {
				// Sample chunks carry no time, so no clock conversion applies.
				chunk, err := ipcclient.DecodeLocalPathSamplesFrame(payload)
				if err != nil {
					if reject(0, errors.New("jog_invalid_frame")) != nil {
						return
					}
					continue
				}
				if _, _, err := h.controller.remoteJogGrant(fence, generation); err != nil {
					if ctx.Err() != nil {
						return
					}
					_ = reject(chunk.ChunkSequence, err)
					return
				}
				if reason := h.controller.appendLocalPath(chunk); reason != ipcclient.JogReasonNone {
					h.controller.recordPathIngress(chunk.ChunkSequence, reason)
					select {
					case acks <- pathAck{chunk.ChunkSequence, reason}:
					default:
						if writeRejection(chunk.ChunkSequence, fmt.Errorf("jog_native_rejected_%d", reason)) != nil {
							return
						}
					}
				}
				continue
			}
			frame, ok := ipcclient.DecodeLocalJogUpdate(payload)
			if !ok {
				if reject(0, errors.New("jog_invalid_frame")) != nil {
					return
				}
				continue
			}
			_, expiry, err := h.controller.remoteJogGrant(fence, generation)
			if err != nil {
				if ctx.Err() != nil {
					return
				}
				_ = reject(frame.SourceSequence, err)
				return
			}
			var session [64]byte
			hex.Encode(session[:], frame.SessionId[:])
			if string(session[:]) != fence.Session || frame.GrantGeneration != fence.Generation || frame.JogGeneration != generation {
				if reject(frame.SourceSequence, errors.New("jog_session_stale")) != nil {
					return
				}
				continue
			}
			if sourceCutoff != 0 && frame.OriginHostNs < sourceCutoff {
				if reject(frame.SourceSequence, ErrJogClockBackward) != nil {
					return
				}
				continue
			}
			var admission JogClockAdmission
			mapped, admission, err = model.convert(frame, rx, expiry)
			if err != nil {
				if reject(frame.SourceSequence, err) != nil {
					return
				}
				continue
			}
			if reason, cause := h.controller.publishLocalJog(&mapped, clock.Incarnation); reason != ipcclient.JogReasonNone {
				// Shared publication already recorded and logged this frame's
				// refusal. A collision that outlasted the retry window is named
				// jog_publisher_busy, never not_ready: the producer may resend
				// the same input with a new sequence before its deadline.
				label := fmt.Errorf("jog_native_rejected_%d", reason)
				if jogTransient(cause) {
					label = errors.New(ReasonJogPublisherBusy)
				}
				if writeRejection(frame.SourceSequence, label) != nil {
					return
				}
				continue
			}
			previous = admission
			model.previous = &previous
		}
	}
}
