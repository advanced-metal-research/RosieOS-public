//go:build linux

package control

import (
	"encoding/json"
	"net/http/httptest"
	"testing"

	"rosieos/rt-core/ipcclient"
)

type alarmStatusCore struct {
	nativeClient
	status ipcclient.Status
}

func (n *alarmStatusCore) Status() (ipcclient.Status, error) { return n.status, nil }
func (n *alarmStatusCore) Motion() (ipcclient.MotionState, error) {
	return ipcclient.MotionState{}, nil
}
func (n *alarmStatusCore) Err() error { return nil }

// TestDriveAlarmStatusPassthrough preserves native alarm names, full auxiliary
// words and validity through HTTP, preventing truncation or stale public clears.
func TestDriveAlarmStatusPassthrough(t *testing.T) {
	native := &alarmStatusCore{}
	controller := &Controller{core: native}
	for _, test := range []struct {
		input    string
		auxCode  uint32
		auxValid bool
	}{
		{`{"time_ns":1000000000,"axes":[{"error_code":25376,"statusword":5840,"readiness":"faulted","drive_alarm":"ALF1.0","drive_alarm_text":"Re-power-on required for parameter settings to take effect","drive_alarm_aux_code":2882342672,"drive_alarm_aux_valid":true}]}`, 0xabcd0f10, true},
		{`{"time_ns":1000000001,"axes":[{"error_code":25376,"drive_alarm":"ALF1.0","drive_alarm_text":"Re-power-on required for parameter settings to take effect","drive_alarm_aux_code":3856,"drive_alarm_aux_valid":true}]}`, 0x0f10, true},
		{`{"time_ns":1000000002,"axes":[{"error_code":0,"drive_alarm":"","drive_alarm_text":"","drive_alarm_aux_code":0,"drive_alarm_aux_valid":false}]}`, 0, false},
		{`{"time_ns":1000000003,"axes":[{"error_code":25376,"drive_alarm":"ALF1.0 | ALF1.2","drive_alarm_text":"Re-power-on required for parameter settings to take effect | Torque reached parameter error","drive_alarm_aux_code":0,"drive_alarm_aux_valid":false}]}`, 0, false},
	} {
		if err := json.Unmarshal([]byte(test.input), &native.status); err != nil {
			t.Fatal(err)
		}
		recorder := httptest.NewRecorder()
		NewHandler(controller).ServeHTTP(recorder, httptest.NewRequest("GET", "/v1/status", nil))
		if recorder.Code != 200 {
			t.Fatalf("HTTP %d: %s", recorder.Code, recorder.Body.String())
		}
		var result struct {
			Data ProcessStatus `json:"data"`
		}
		if err := json.Unmarshal(recorder.Body.Bytes(), &result); err != nil {
			t.Fatal(err)
		}
		if len(result.Data.Axes) != 1 || len(result.Data.Core.Axes) != 1 {
			t.Fatal("missing axes")
		}
		want := native.status.Axes[0]
		for _, got := range []struct {
			code, text string
			auxCode    uint32
			auxValid   bool
		}{
			{result.Data.Axes[0].DriveAlarm, result.Data.Axes[0].DriveAlarmText, result.Data.Axes[0].DriveAlarmAuxCode, result.Data.Axes[0].DriveAlarmAuxValid},
			{result.Data.Core.Axes[0].DriveAlarm, result.Data.Core.Axes[0].DriveAlarmText, result.Data.Core.Axes[0].DriveAlarmAuxCode, result.Data.Core.Axes[0].DriveAlarmAuxValid},
		} {
			if got.code != want.DriveAlarm || got.text != want.DriveAlarmText {
				t.Fatalf("lost alarm: %+v want %+v", got, want)
			}
			if got.auxCode != test.auxCode || got.auxValid != test.auxValid {
				t.Fatalf("lost auxiliary receipt: %+v want code=%#x valid=%t", got, test.auxCode, test.auxValid)
			}
		}
		if result.Data.Axes[0].Readiness != want.Readiness || result.Data.Axes[0].Statusword != want.Statusword || result.Data.TimeNS != native.status.TimeNS {
			t.Fatal("alarm passthrough changed existing fields")
		}
	}
}
