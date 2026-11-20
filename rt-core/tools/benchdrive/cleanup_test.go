//go:build linux

package main

import (
	"context"
	"encoding/json"
	"errors"
	"net/http"
	"strings"
	"sync"
	"testing"

	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// TestReleaseStaleRequiresConfirmedRetirement permits an already retired grant
// only after a successful Stop and fresh inhibition, preventing an unrelated
// refusal or missing Stop observation from being reported as successful cleanup.
func TestReleaseStaleRequiresConfirmedRetirement(t *testing.T) {
	for _, name := range []string{"retired", "no_stop", "live_session", "stop_refused", "stop_generation", "stop_observation", "later_failed_stop", "new_session", "release_refused", "release_protocol", "release_observation", "evidence_failure"} {
		t.Run(name, func(t *testing.T) {
			var mu sync.Mutex
			stage := "stop"
			stops, releases := 0, 0
			var sentRelease control.Fence
			f := &fixture{hash: strings.Repeat("a", 64), build: testBuildDir(t)}
			f.socket = unixServer(t, func(w http.ResponseWriter, req *http.Request) {
				mu.Lock()
				defer mu.Unlock()
				response := control.Response{Schema: "rosie.rt-control.response.v1"}
				var data any
				switch req.URL.Path {
				case "/v1/status":
					s := control.Status{Core: ipcclient.Status{TimeNS: ipcclient.HostMonotonicNS()}}
					if name == "stop_observation" && stage == "stop" || name == "release_observation" && stage == "release" {
						s.Core.TimeNS = 0
					}
					data = s
				case "/v1/control":
					var command control.Request
					if err := json.NewDecoder(req.Body).Decode(&command); err != nil {
						t.Error(err)
						return
					}
					response.Operation = command.Operation
					switch command.Operation {
					case "stop":
						stops++
						grant := control.Grant{Generation: 2}
						if name == "live_session" {
							grant.Session = command.Session
						}
						if name == "stop_generation" {
							grant.Generation = 1
						}
						if name == "stop_refused" || stops == 2 {
							response.Error = "control_session_stale"
						}
						data = grant
					case "release":
						releases++
						sentRelease = command.Fence
						response.Error = "control_session_stale"
						if name == "release_refused" {
							response.Error = "daemon_restarted"
						}
						if name == "release_protocol" {
							response.Schema = "invalid"
						}
					default:
						t.Errorf("unexpected command %s", command.Operation)
					}
				default:
					t.Errorf("unexpected endpoint %s", req.URL.Path)
				}
				var err error
				response.Data, err = json.Marshal(data)
				if err != nil {
					t.Error(err)
					return
				}
				if response.Error != "" {
					w.WriteHeader(http.StatusConflict)
				}
				if err := json.NewEncoder(w).Encode(response); err != nil {
					t.Error(err)
				}
			})
			r := testRunner(t, f, "program", name)
			r.c.SetFence(control.Fence{Session: "0123456789abcdef0123456789abcdef", Generation: 1})
			if name != "no_stop" {
				err := r.stop(context.Background())
				wantFailure := name == "stop_refused" || name == "stop_generation" || name == "stop_observation"
				if (err != nil) != wantFailure {
					t.Fatalf("Stop: %v, want failure=%t", err, wantFailure)
				}
			}
			if name == "later_failed_stop" {
				if err := r.stop(context.Background()); err == nil {
					t.Fatal("second Stop unexpectedly succeeded")
				}
			}
			if name == "new_session" {
				r.c.SetFence(control.Fence{Session: "fedcba9876543210fedcba9876543210", Generation: 3})
			}
			mu.Lock()
			stage = "release"
			mu.Unlock()
			storageErr := errors.New("injected evidence failure")
			if name == "evidence_failure" {
				r.e.sink = cleanupFailureSink{storageErr}
			}
			err := r.release(context.Background())
			if (err == nil) != (name == "retired") {
				t.Fatalf("Release: %v, want success=%t", err, name == "retired")
			}
			if name == "evidence_failure" {
				if !errors.Is(err, storageErr) {
					t.Fatalf("evidence failure lost: %v", err)
				}
				// The injected error is asserted above; let testRunner close its file.
				r.e.err = nil
			}
			mu.Lock()
			defer mu.Unlock()
			if releases != 1 || (name == "retired" && (stops != 1 || sentRelease != (control.Fence{Session: "0123456789abcdef0123456789abcdef", Generation: 1}) || !r.expectations["release"])) {
				t.Fatalf("cleanup requests: Stop=%d Release=%d fence=%+v", stops, releases, sentRelease)
			}
			if name == "retired" {
				simfixture.Must(t, r.e.failure())
			}
		})
	}
}

type cleanupFailureSink struct{ err error }

func (s cleanupFailureSink) Write([]byte) (int, error) { return 0, s.err }
