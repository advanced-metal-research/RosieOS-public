package control

import (
	"reflect"
	"testing"

	"rosieos/rt-core/ipcclient"
)

// The dense axis map is the description's own: every rotary axis, in native
// index order, one wire column each; a linear axis is left out. No start flag
// or cell config field states the count (owner direction 2026-09-20).
func TestDescribedDenseAxesAreTheRotaryAxesInOrder(t *testing.T) {
	description := ipcclient.Description{Axes: []ipcclient.AxisDescription{
		{Index: 0, ID: "J1", PositionUnit: "rad"},
		{Index: 1, ID: "J2", PositionUnit: "rad"},
		{Index: 2, ID: "slide", PositionUnit: "m"},
		{Index: 3, ID: "J3", PositionUnit: "rad"},
	}}
	if got := DescribedDenseAxes(description); !reflect.DeepEqual(got, []string{"J1", "J2", "J3"}) {
		t.Fatalf("dense axes %v", got)
	}
	if got := DescribedDenseAxes(ipcclient.Description{}); len(got) != 0 {
		t.Fatalf("a description without axes maps nothing, got %v", got)
	}
	// Six described rotary axes map six wire columns: the reference program
	// written for a six-axis cell decodes against the derived map exactly as
	// it did against an explicit six-id list.
	six := ipcclient.Description{}
	for i, id := range []string{"J1", "J2", "J3", "J4", "J5", "J6"} {
		six.Axes = append(six.Axes, ipcclient.AxisDescription{Index: uint32(i), ID: id, PositionUnit: "rad"})
	}
	if got := DescribedDenseAxes(six); len(got) != 6 || got[5] != "J6" {
		t.Fatalf("six-axis map %v", got)
	}
}
