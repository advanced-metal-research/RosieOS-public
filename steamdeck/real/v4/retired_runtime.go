package main

import "fmt"

// retiredRuntimeError identifies an obsolete selection before build inputs,
// credentials or remote resources can be acquired.
type retiredRuntimeError struct {
	Selection string
	Detail    string
}

func (e *retiredRuntimeError) Error() string {
	if e.Detail != "" {
		return fmt.Sprintf("reason=backend_retired selection=%s; %s", e.Selection, e.Detail)
	}
	return fmt.Sprintf("reason=backend_retired selection=%s; select rt-core with an approved cell binding", e.Selection)
}
