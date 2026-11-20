package densejointtraj

import "testing"

// The digests are Python's own (json.dumps(markers, separators=(",", ":"),
// sort_keys=True), the planner's markers_digest), computed 2026-10-12. A Go
// json.Marshal of the same list orders keys by field and escapes '<', '>' and
// '&', so a plan record and its plan would never agree.
func TestProcessMarkersDigestIsThePlannersCanonicalDigest(t *testing.T) {
	cases := []struct {
		name    string
		markers []ProcessMarker
		want    string
	}{
		{"gas on, refreshed, off", []ProcessMarker{
			{Segment: 1, Sample: 0, Action: "gas", Value: true},
			{Segment: 1, Sample: 10, Action: "gas", Value: true},
			{Segment: 3, Sample: 4, Action: "gas", Value: false},
		}, "sha256:9dc628dd0046cd391f76716e15656f79a80d2878c283683a6e9e791924916332"},
		{"empty", []ProcessMarker{}, "sha256:4f53cda18c2baa0c0354bb5f9a3ecbe5ed12ab4d8e11ba873c2f11161202b945"},
		{"escapes", []ProcessMarker{{Segment: 0, Sample: 2, Action: "g<a>s&\"\\\n\u00e9\U0001F525", Value: false}},
			"sha256:3f3382fd824602a42b9856b3ef5aa3014f63c06e86a696b640ba947f65b808b7"},
	}
	for _, c := range cases {
		if got := ProcessMarkersDigest(c.markers); got != c.want {
			t.Errorf("%s: %s, the planner computes %s", c.name, got, c.want)
		}
	}
}
