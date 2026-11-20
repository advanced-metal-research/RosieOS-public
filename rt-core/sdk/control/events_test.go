//go:build linux

package control

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"net/http/httptest"
	"reflect"
	"testing"

	adapter "rosieos/rt-core/adapters/rosie/control"
)

func TestSDKEventsPreserveCursorAndDroppedRange(t *testing.T) {
	// Unverified boundary fixture beyond float64 precision catches cursor rounding.
	const after uint64 = 9007199254740993
	drop := Event{Sequence: after + 3, TimeNS: 123456789, Type: "events_dropped", AdapterIncarnation: "adapter", DaemonIncarnation: "daemon", Dropped: &EventsDropped{FirstLostSequence: after + 1, LastLostSequence: after + 3}}
	next := Event{Sequence: after + 4, TimeNS: 123456790, Type: "grant_renewed", AdapterIncarnation: "adapter", DaemonIncarnation: "daemon", Grant: &adapter.GrantEvent{Generation: 7}}
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Query().Get("after") != fmt.Sprint(after) {
			t.Errorf("cursor changed: %s", r.URL.RawQuery)
		}
		if r.URL.Path == "/v1/events" {
			body, _ := json.Marshal(EventBatch{Events: []Event{drop, next}, NextSequence: next.Sequence, LatestSequence: next.Sequence, AdapterIncarnation: "adapter"})
			_ = json.NewEncoder(w).Encode(Response{Schema: adapter.ResponseSchema, Operation: "subscribe_events", Data: body})
			return
		}
		w.Header().Set("Content-Type", "text/event-stream")
		for _, event := range []Event{drop, next} {
			body, _ := json.Marshal(event)
			fmt.Fprintf(w, "id: %d\nevent: %s\ndata: %s\n\n", event.Sequence, event.Type, body)
		}
	}))
	defer server.Close()
	client := newClient(server.URL, &http.Transport{})
	defer client.Close()
	batch, err := client.Events(after)
	if err != nil || !reflect.DeepEqual(batch.Events, []Event{drop, next}) {
		t.Fatalf("poll events: %+v %v", batch, err)
	}
	stop := errors.New("handled")
	var handled []Event
	err = client.StreamEvents(context.Background(), after, func(event Event) error {
		handled = append(handled, event)
		if event.Sequence == next.Sequence {
			return stop
		}
		return nil
	})
	if !errors.Is(err, stop) || !reflect.DeepEqual(handled, []Event{drop, next}) {
		t.Fatalf("stream: %+v %v", handled, err)
	}
}

func TestSDKRejectsUnsignalledEventGaps(t *testing.T) {
	for _, event := range []Event{{Sequence: 12, TimeNS: 1, Type: "grant_renewed"}, {Sequence: 12, TimeNS: 1, Type: "events_dropped", Dropped: &EventsDropped{FirstLostSequence: 11, LastLostSequence: 13}}, {Sequence: 11, TimeNS: 0, Type: "grant_renewed"}} {
		if err := validateEvent(event, 10); !errors.Is(err, ErrProtocol) {
			t.Fatalf("accepted invalid event: %+v", event)
		}
	}
}
