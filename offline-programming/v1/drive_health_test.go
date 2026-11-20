package main

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// driveHealthBackend answers status with one rt-core observation.
type driveHealthBackend struct {
	identityBackend
	body *denseexec.StatusBody
}

func (b *driveHealthBackend) Status(context.Context) *denseexec.StatusBody { return b.body }

// The drive page's route reads the selected cell's status once and answers
// per joint; without a connected controller it is 503, never an empty page
// that looks healthy. It is a GET, so maintenance and control gates pass it.
func TestDriveHealthRoute(t *testing.T) {
	temperature := 41.2
	var status control.Status
	status.Core.Axes = []ipcclient.AxisStatus{{Readiness: "ready",
		Load:    ipcclient.AxisLoad{WindowMS: 1000, TorqueSamples: 1000, TorqueRMSRaw: 200, TorquePeakRaw: 300},
		Monitor: []ipcclient.AxisMonitor{{Key: "heatsink_temperature", Unit: "degC", Value: &temperature}}}}
	var description control.Description
	description.Axes = []ipcclient.AxisDescription{{ID: "J1", PositionUnit: "rad", CountsPerUnit: 1000}}
	backend := &driveHealthBackend{body: &denseexec.StatusBody{Backend: "rt_core", Available: true,
		RTCore: &denseexec.RTCoreObservation{Description: description, Status: &status}}}
	handler := NewServer(ServerOptions{DistDir: t.TempDir(), DenseExecution: backend}).Handler()

	get := func() *httptest.ResponseRecorder {
		rec := httptest.NewRecorder()
		handler.ServeHTTP(rec, httptest.NewRequest(http.MethodGet, denseExecutionPath+"/drive-health", nil))
		return rec
	}
	rec := get()
	var health denseexec.DriveHealth
	if rec.Code != http.StatusOK || json.Unmarshal(rec.Body.Bytes(), &health) != nil || len(health.Joints) != 1 {
		t.Fatalf("%d %s", rec.Code, rec.Body.String())
	}
	j := health.Joints[0]
	if j.Name != "J1" || j.TorqueRMSPercent == nil || *j.TorqueRMSPercent != 20 || *j.Monitor[0].Value != 41.2 {
		t.Fatalf("%+v", j)
	}
	backend.body = &denseexec.StatusBody{Backend: "rt_core", Available: true}
	if rec := get(); rec.Code != http.StatusServiceUnavailable {
		t.Fatalf("no controller status: %d %s", rec.Code, rec.Body.String())
	}
}
