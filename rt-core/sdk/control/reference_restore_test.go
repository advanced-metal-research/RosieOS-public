//go:build linux

package control

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"
)

func TestSDKVerifiedDeltaRequiresExactNativeReceipt(t *testing.T) {
	for _, trial := range []string{"valid", "missing", "invalid", "wrong_method", "wrong_version", "old_sequence"} {
		t.Run(trial, func(t *testing.T) {
			server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				var request Request
				if json.NewDecoder(r.Body).Decode(&request) != nil || request.Operation != "restore_reference_verified_delta" || request.AxisMask != 32 {
					t.Error("wrong typed command")
				}
				v := AnchorDeltaRestoreStatus{Valid: true, Method: "independent_absolute_delta_v1", AnchorVersion: 3, SavedLogicalOffsetCounts: -77, BoundRestoreCommandSequence: 19}
				switch trial {
				case "invalid":
					v.Valid = false
				case "wrong_method":
					v.Method = "same_pose_only"
				case "wrong_version":
					v.AnchorVersion = 5
				case "old_sequence":
					v.BoundRestoreCommandSequence = 18
				}
				envelope := map[string]any{"schema": "rosie.rt-control.response.v1", "operation": "restore_reference_verified_delta", "sequence": 19}
				if trial != "missing" {
					envelope["data"] = VerifiedDeltaRestoreReceipt{AnchorDeltaRestore: &v}
				}
				json.NewEncoder(w).Encode(envelope)
			}))
			defer server.Close()
			client := newClient(server.URL, &http.Transport{})
			defer client.Close()
			client.SetFence(Fence{Session: "owner", Generation: 7})
			receipt, err := client.RestoreReferenceVerifiedDelta(context.Background(), 32)
			if trial == "valid" {
				if err != nil || receipt.AnchorDeltaRestore == nil || receipt.AnchorDeltaRestore.SavedLogicalOffsetCounts != -77 {
					t.Fatalf("%+v %v", receipt, err)
				}
			} else if err == nil {
				t.Fatal("invalid native receipt accepted")
			}
		})
	}
}
