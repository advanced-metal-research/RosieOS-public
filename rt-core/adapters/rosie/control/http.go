//go:build linux

package control

import (
	"bytes"
	"encoding/json"
	"errors"
	"io"
	"net/http"
	"strconv"
	"strings"
	"time"

	"rosieos/rt-core/ipcclient"
)

// HTTP runs on a filesystem-protected Unix socket. Remote authentication belongs
// at the Rosie boundary; this service never opens an unauthenticated TCP port.
type Handler struct {
	Controller *Controller
}

func NewHandler(c *Controller) *Handler { return &Handler{Controller: c} }

// UnmarshalJSON keeps the strict envelope contract and distinguishes an absent
// optional ID from null or another JSON type, preventing untracked mutations
// when a caller believed it supplied an idempotence key.
func (request *Request) UnmarshalJSON(body []byte) error {
	type plainRequest Request
	var decoded plainRequest
	wire := struct {
		*plainRequest
		ID json.RawMessage `json:"request_id"`
	}{plainRequest: &decoded}
	decoder := json.NewDecoder(bytes.NewReader(body))
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&wire); err != nil {
		return err
	}
	if wire.ID != nil {
		var id string
		if bytes.Equal(bytes.TrimSpace(wire.ID), []byte("null")) || json.Unmarshal(wire.ID, &id) != nil {
			return errors.New(ReasonInvalidRequestId)
		}
		decoded.RequestID = &id
	}
	*request = Request(decoded)
	return nil
}

func (h *Handler) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	if r.Method == http.MethodGet && strings.HasPrefix(r.URL.Path, "/v1/resources/") {
		h.serveResource(w, r)
		return
	}
	if r.Method == http.MethodGet && (r.URL.Path == "/v1/telemetry" || r.URL.Path == "/v1/telemetry/stream") {
		h.serveTelemetry(w, r)
		return
	}
	if r.Method == http.MethodGet && (r.URL.Path == "/v1/events" || r.URL.Path == "/v1/events/stream") {
		h.serveEvents(w, r)
		return
	}
	response := Response{Schema: ResponseSchema}
	var err error
	if r.Method == http.MethodGet && r.URL.Path == "/v1/describe" {
		response.Operation = OperationDescribe
		description := h.Controller.Describe()
		description.Program = h.Controller.DescribeProgram()
		response.Data = description
	} else if r.Method == http.MethodGet && r.URL.Path == "/v1/jog/clock" {
		response.Operation = OperationJogClock
		response.Data, err = h.Controller.JogClock()
	} else if r.Method == http.MethodGet && r.URL.Path == "/v1/jog" {
		response.Operation = OperationJogStatus
		response.Data, err = h.Controller.JogStatus()
	} else if r.Method == http.MethodGet && r.URL.Path == "/v1/jog/path" {
		response.Operation = OperationPathJogStatus
		response.Data, err = h.Controller.PathJogStatus()
	} else if r.Method == http.MethodGet && r.URL.Path == "/v1/jog/ingress" {
		response.Operation = OperationJogIngress
		response.Data = h.Controller.JogIngress()
	} else if r.Method == http.MethodGet && r.URL.Path == "/v1/maintenance" {
		response.Operation = OperationMaintenance
		response.Data = h.Controller.Maintenance()
	} else if r.Method == http.MethodGet && r.URL.Path == "/v1/status" {
		response.Operation = OperationStatus
		// The optional view changes only payload history, never native evidence.
		// Older servers ignore this query and continue returning full status.
		response.Data, err = h.Controller.logicalStatusWithHistory(r.URL.Query().Get("view") != "control")
	} else if r.Method == http.MethodPost && r.URL.Path == "/v1/program" {
		response.Operation = OperationPrepareProgram
		generation, parseErr := strconv.ParseUint(r.Header.Get("X-Control-Generation"), 10, 64)
		fence := Fence{r.Header.Get("X-Control-Session"), generation}
		if r.ContentLength > programPreparationBytes {
			err = errBodyTooLarge
		} else if parseErr != nil {
			err = errors.New(ReasonInvalidControlGeneration)
		} else if err = h.Controller.authorizePrincipal(r.Context(), fence); err == nil {
			var release func()
			release, err = h.Controller.admitPreparation(fence, true)
			if err == nil {
				defer release()
				var body []byte
				body, err = readAdmittedBody(w, r, nil, programPreparationBytes)
				if err == nil {
					response.Data, err = h.Controller.prepareProgramRequest(r.Context(), fence, body)
				}
			}
		}
	} else if r.Method == http.MethodPost && r.URL.Path == "/v1/control" {
		var request Request
		var envelope Request
		var prefix []byte
		if r.ContentLength > jsonPreparationBytes {
			err = errBodyTooLarge
		} else {
			envelope, prefix, err = readEnvelope(r.Body)
			// The frozen admission layer knows implemented control operations only.
			// Admit named unavailable capabilities for explicit dispatch rejection;
			// all other envelope failures retain their existing precedence.
			if err != nil && err.Error() == ReasonUnknownOperation {
				if capability, known := Capability(envelope.Operation); known && capability.State == "unimplemented" {
					err = nil
				}
			}
		}
		response.Operation = envelope.Operation
		if err == nil {
			err = h.Controller.authorizePrincipal(r.Context(), envelope.Fence)
		}
		cap := controlEnvelopeBytes
		if err == nil && envelope.Operation == OperationPrepareTrajectory {
			cap = jsonPreparationBytes
			var release func()
			release, err = h.Controller.admitPreparation(envelope.Fence, false)
			if err == nil {
				defer release()
			}
		}
		if err == nil {
			var body []byte
			body, err = readAdmittedBody(w, r, prefix, cap)
			if err == nil {
				decoder := json.NewDecoder(bytes.NewReader(body))
				decoder.DisallowUnknownFields()
				err = decoder.Decode(&request)
				if err == nil {
					var extra any
					if decoder.Decode(&extra) != io.EOF {
						err = errors.New(ReasonTrailingRequestData)
					}
				}
			}
		}
		if err == nil && (request.Schema != envelope.Schema || request.Operation != envelope.Operation || request.Fence != envelope.Fence) {
			err = errors.New(ReasonRequestEnvelopeChanged)
		}
		if err == nil {
			outcome, cacheErr := h.Controller.idempotent(request, func() requestOutcome {
				if request.Operation == OperationPrepareTrajectory {
					handle, prepareErr := h.Controller.prepareRequest(r.Context(), request)
					return encodeResponse(Response{Schema: response.Schema, Operation: request.Operation, Handle: handle, Data: h.Controller.handleReceipt(handle)}, prepareErr)
				}
				response, dispatchErr := h.dispatchPrincipal(r, request)
				return encodeResponse(response, dispatchErr)
			}, requestPrincipal(r.Context()))
			if cacheErr == nil {
				w.WriteHeader(outcome.status)
				_, _ = w.Write(outcome.body)
				return
			}
			err = cacheErr
		}
	} else {
		err = errors.New(ReasonUnknownEndpoint)
	}
	if err != nil && r.Method == http.MethodPost {
		rejectUnreadBody(w, r)
	}
	outcome := encodeResponse(response, err)
	w.WriteHeader(outcome.status)
	_, _ = w.Write(outcome.body)
}

func (h *Handler) dispatch(request Request) (Response, error) {
	response := Response{Schema: ResponseSchema, Operation: request.Operation}
	if capability, known := Capability(request.Operation); known && capability.State == "unimplemented" {
		response.Data = capability
		return response, errors.New(ReasonCapabilityUnimplemented)
	}
	if err := h.Controller.CheckRestartFence(request.Fence); err != nil {
		return response, err
	}
	var err error
	switch request.Operation {
	case OperationMarkTelemetry:
		response.Sequence, err = h.Controller.MarkTelemetry(request.Fence, request.Label)
	case OperationAcquire:
		response.Data, err = h.Controller.Acquire(request.Controller, request.Binding, request.RequestedLeaseMS)
	case OperationRenew:
		response.Data, err = h.Controller.Renew(request.Fence)
	case OperationRelease:
		response.Data, err = h.Controller.Stop(request.Fence, true)
	case OperationStop:
		response.Data, err = h.Controller.Stop(request.Fence, false)
	case OperationHalt:
		response.Sequence, err = h.Controller.Halt(request.Fence)
	case OperationIoArm:
		response.Sequence, err = h.Controller.IOArm(request.Fence, true)
	case OperationIoTest:
		response.Sequence, err = h.Controller.IOTest(request.Fence, request.Label, request.Value, request.SourceSequence, request.Begin)
	case OperationIoDisarm:
		response.Sequence, err = h.Controller.IOArm(request.Fence, false)
	case OperationEnable:
		response.Sequence, err = h.Controller.Enable(request.Fence, request.AxisMask)
	case OperationArm:
		response.Sequence, err = h.Controller.Arm(request.Fence)
	case OperationHome:
		response.Sequence, err = h.Controller.Home(request.Fence, request.AxisMask)
	case OperationRestoreReferenceVerifiedDelta:
		response.Sequence, response.Data, err = h.Controller.RestoreReferenceVerifiedDelta(request.Fence, request.AxisMask)
	case OperationRestoreAnchor:
		response.Sequence, response.Data, err = h.Controller.RestoreAnchor(request.Fence, request.AxisMask)
	case OperationResetFault:
		response.Sequence, response.NativeResult, response.Data, err = h.Controller.FaultResetRecovery(request.Fence, request.AxisMask)
	case OperationRecoveryStatus:
		response.Data, err = h.Controller.RecoveryStatus(request.Fence)
	case OperationJog:
		response.Sequence, err = h.Controller.Jog(request.Fence, request.AxisMask, request.Velocity, time.Duration(request.TimeoutMS)*time.Millisecond)
	case OperationBeginJog:
		err = h.Controller.checkJogClock(request.ClockIncarnation)
		if err == nil {
			response.Handle, err = h.Controller.BeginJog(request.Fence, request.AxisMask, request.SourceOriginHostNS, request.DeadlineHostNS)
		}
	case OperationBeginPathJog:
		err = h.Controller.checkJogClock(request.ClockIncarnation)
		if err == nil {
			response.Handle, err = h.Controller.BeginPathJog(request.Fence, request.AxisMask, request.SourceOriginHostNS, request.DeadlineHostNS)
		}
	case OperationEndJog:
		err = h.Controller.EndJog(request.Fence, request.JogGeneration)
		if err == nil {
			response.Handle = request.JogGeneration
		}
	case OperationPrepareTrajectory:
		response.Handle, err = h.Controller.Prepare(request.Fence, request.AxisMask, request.Points, request.Identity)
		response.Data = h.Controller.handleReceipt(response.Handle)
	case OperationStartTrajectory:
		response.Sequence, err = h.Controller.Start(request.Fence, request.Handle, request.Identity)
		response.Data = h.Controller.handleReceipt(request.Handle)
	case OperationDiscardTrajectory:
		response.Sequence, err = h.Controller.Discard(request.Fence, request.Handle)
	case OperationStartProgram:
		response.Sequence, err = h.Controller.StartProgram(request.Fence, request.Identity)
	default:
		err = errors.New(ReasonUnknownOperation)
	}
	return response, err
}

func encodeResponse(response Response, err error) requestOutcome {
	status := http.StatusOK
	if err != nil {
		response.Error = err.Error()
		if detail := programLimitDetails(err); detail != nil {
			response.Data = detail
		}
		var jog *ipcclient.JogRejectedError
		if errors.As(err, &jog) {
			response.NativeJogResult = &jog.Observation
			if jog.Reason == ipcclient.JogReasonModeConflict {
				response.Error = ReasonModeConflict
			}
			if jog.Reason == ipcclient.JogReasonOutsideLimitsOutward {
				response.Error = ReasonOutsideLimitsOutward
			}
		}
		var native *ipcclient.RejectedError
		if errors.As(err, &native) {
			// Preserve contextual/rollback failures while still exposing the
			// structured native result. A plain rejection keeps its short code.
			if err == native {
				response.Error = ReasonNativeRejected
			}
			response.NativeResult = &native.Result
			if err == native && native.Result.Reason == ipcclient.ReasonOutsideLimitsOutward {
				response.Error = ReasonOutsideLimitsOutward
			}
		}
		status = http.StatusConflict
		if errors.Is(err, errBodyTooLarge) {
			status = http.StatusRequestEntityTooLarge
		}
	}
	body, _ := json.Marshal(response)
	return requestOutcome{status: status, body: append(body, '\n')}
}
