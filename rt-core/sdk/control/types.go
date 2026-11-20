//go:build linux

// Package control provides the application HTTP boundary. It never acquires
// native IPC authority, so applications cannot bypass the adapter's fences.
package control

import (
	adapter "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/ipcclient"
)

// Aliases use the generated application contract so SDK wire types cannot diverge.
type Request = adapter.Request
type Fence = adapter.Fence
type Binding = adapter.Binding
type Grant = adapter.Grant
type Identity = adapter.Identity
type Program = adapter.Program
type Execution = adapter.Execution
type HandleRecord = adapter.HandleRecord
type ProcessMarker = adapter.ProcessMarker
type Description = adapter.Description
type RecoveryStatus = adapter.RecoveryStatus
type AnchorDeltaRestoreStatus = adapter.AnchorDeltaRestoreStatus
type VerifiedDeltaRestoreReceipt = adapter.VerifiedDeltaRestoreReceipt
type CapabilityInfo = adapter.CapabilityInfo

// Capability labels are unverified outside the tested application boundary.
const (
	CapabilityStateImplemented   = adapter.CapabilityStateImplemented
	CapabilityStateInterim       = adapter.CapabilityStateInterim
	CapabilityStateTestOnly      = adapter.CapabilityStateTestOnly
	CapabilityStateUnimplemented = adapter.CapabilityStateUnimplemented
)

type Point = ipcclient.Point
type JogClock = ipcclient.LocalJogClock
type JogObservation = ipcclient.JogObservation
type JogIngressObservation = adapter.JogIngressObservation
type JogIngressRefusal = adapter.JogIngressRefusal
type MaintenanceObservation = adapter.MaintenanceObservation
type MaintenanceHolder = adapter.MaintenanceHolder

// Response preserves the wire envelope and defers operation-specific data
// decoding, preventing JSON numbers from losing integer precision through any.
type Response = adapter.RawResponse

type Status = adapter.ProcessStatus

type Event = adapter.Event
type EventBatch = adapter.EventBatch
type EventsDropped = adapter.EventsDropped
