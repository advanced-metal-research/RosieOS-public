package main

import (
	"context"
	"errors"
	"io"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"
)

func TestWeldPlanCancellationDisconnectsMotionPlanner(t *testing.T) {
	entered, disconnected := make(chan struct{}), make(chan struct{})
	cleanup, stop := context.WithCancel(context.Background())
	planner := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		_, _ = io.Copy(io.Discard, r.Body)
		close(entered)
		select {
		case <-r.Context().Done():
			close(disconnected)
		case <-cleanup.Done():
		}
	}))
	defer planner.Close()
	defer stop()
	server := &Server{motionOrigin: planner.URL}
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	result := make(chan error, 1)
	go func() {
		_, err := server.postMotionPlan(ctx, []byte("packed request"), weldPlanRequest{})
		result <- err
	}()
	select {
	case <-entered:
	case <-time.After(3 * time.Second):
		t.Fatal("motion planner did not receive request")
	}
	cancel()
	select {
	case err := <-result:
		if !errors.Is(err, context.Canceled) {
			t.Fatalf("cancellation not propagated: %v", err)
		}
	case <-time.After(3 * time.Second):
		t.Fatal("OLP planner request did not stop")
	}
	select {
	case <-disconnected:
	case <-time.After(3 * time.Second):
		t.Fatal("motion planner connection remained active after cancellation")
	}
}
