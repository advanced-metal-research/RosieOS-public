package main

import (
	"context"
	"errors"
	"net/http/httptest"
	"strings"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
)

type resetFacade struct {
	identityBackend
	calls   int
	refusal bool
}

func (b *resetFacade) ResetFault(context.Context) (*denseexec.ArmResult, error) {
	b.calls++
	result := &denseexec.ArmResult{OK: true, Accepted: !b.refusal, Status: &denseexec.StatusBody{Backend: "rt_core", Available: true, DaemonError: "rt_core_fault: 8"}}
	if b.refusal {
		return result, errors.New("restart_required")
	}
	return result, nil
}
func TestDenseResetFaultRoutePreservesCompletedRefusalStatus(t *testing.T) {
	for _, refused := range []bool{false, true} {
		b := &resetFacade{refusal: refused}
		h := NewServer(ServerOptions{DenseExecution: b}).Handler()
		w := httptest.NewRecorder()
		h.ServeHTTP(w, httptest.NewRequest("POST", denseExecutionPath+"/reset-fault", strings.NewReader("{}")))
		if w.Code != 200 || b.calls != 1 || !strings.Contains(w.Body.String(), `"status":`) {
			t.Fatal(w.Code, w.Body.String(), b.calls)
		}
		if refused && (!strings.Contains(w.Body.String(), `"accepted":false`) || !strings.Contains(w.Body.String(), "restart_required")) {
			t.Fatal(w.Body.String())
		}
		w = httptest.NewRecorder()
		h.ServeHTTP(w, httptest.NewRequest("POST", denseExecutionPath+"/reset-fault", strings.NewReader(`{"armed":true}`)))
		if w.Code != 400 || b.calls != 1 {
			t.Fatal("invalid request reached reset", w.Code, b.calls)
		}
	}
}
