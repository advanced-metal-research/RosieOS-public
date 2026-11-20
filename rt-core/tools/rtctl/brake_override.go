package rtctl

import (
	"fmt"
	"strings"
)

// BrakeOverrideError names the rejected field and configuration class so callers
// cannot mistake a refused safety override for an ordinary profile read failure.
type BrakeOverrideError struct {
	Field, Class, Detail string
}

func (e *BrakeOverrideError) Error() string {
	return fmt.Sprintf("brake_override_refused: %s in %s class: %s", e.Field, e.Class, e.Detail)
}

// validateBrakeOverrides admits only explicit bench hold policies and disabling
// overrides with a reason. Missing class evidence must never disable a brake.
func validateBrakeOverrides(machine object) {
	class := "robot/non-bench"
	if bus, ok := machine.optional("bus", nil).(object); ok && bus.optional("unknown_slaves", nil) == "hold" {
		class = "bench"
	}
	for i, value := range asArray(machine.get("axes")) {
		axis := asObject(value)
		for _, field := range axis {
			if field.key != "brake_override" {
				continue
			}
			label := fmt.Sprintf("axes[%d].brake_override", i)
			refuse := func(detail string) { panic(&BrakeOverrideError{label, class, detail}) }
			if class != "bench" {
				refuse("requires bus.unknown_slaves=hold with its validated bench notes and robot acknowledgement")
			}
			validateBus(machine)
			block, ok := field.value.(object)
			if !ok {
				refuse("must be an object with present=false and a nonempty reason")
			}
			for _, entry := range block {
				if entry.key != "present" && entry.key != "reason" {
					refuse("unknown field " + entry.key)
				}
			}
			if block.optional("present", nil) != false {
				refuse("present must be false")
			}
			reason, ok := block.optional("reason", nil).(string)
			if !ok || strings.TrimSpace(reason) == "" {
				refuse("reason must be nonempty text")
			}
		}
	}
}
