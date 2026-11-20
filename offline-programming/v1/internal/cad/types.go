// Package cad provides a bounded adapter around the CadQuery v1 STEP topology
// and weld-path worker. It contains no HTTP or product-specific server wiring.
package cad

import (
	"context"
	"errors"
	"fmt"
)

// Operation is an operation implemented by one of the bounded worker scripts:
// extract_topology, build_seam, and build_sequence by extract_weld_edges.py;
// tessellate by tessellate_step.py.
type Operation string

const (
	OperationExtractTopology Operation = "extract_topology"
	OperationBuildSeam       Operation = "build_seam"
	OperationBuildSequence   Operation = "build_sequence"
	OperationTessellate      Operation = "tessellate"
)

// ErrorCode is a stable, transport-independent failure category.
type ErrorCode string

const (
	ErrorInvalidRequest  ErrorCode = "invalid_request"
	ErrorPayloadTooLarge ErrorCode = "payload_too_large"
	ErrorBusy            ErrorCode = "busy"
	ErrorTimeout         ErrorCode = "timeout"
	ErrorCanceled        ErrorCode = "canceled"
	ErrorUnavailable     ErrorCode = "unavailable"
	ErrorWorkerFailed    ErrorCode = "worker_failed"
	ErrorInvalidResponse ErrorCode = "invalid_response"
)

var (
	ErrInvalidRequest  = errors.New("invalid CAD request")
	ErrPayloadTooLarge = errors.New("CAD payload too large")
	ErrBusy            = errors.New("CAD worker busy")
	ErrTimeout         = errors.New("CAD worker timeout")
	ErrCanceled        = errors.New("CAD request canceled")
	ErrUnavailable     = errors.New("CAD worker unavailable")
	ErrWorkerFailed    = errors.New("CAD worker failed")
	ErrInvalidResponse = errors.New("invalid CAD worker response")
)

// WorkerError carries a stable code while retaining an errors.Is-compatible
// cause for Go callers.
type WorkerError struct {
	Code      ErrorCode
	Operation Operation
	Message   string
	Cause     error
}

func (err *WorkerError) Error() string {
	if err == nil {
		return ""
	}
	if err.Operation == "" {
		return err.Message
	}
	return fmt.Sprintf("CAD %s: %s", err.Operation, err.Message)
}

func (err *WorkerError) Unwrap() error {
	if err == nil {
		return nil
	}
	return err.Cause
}

// STEPInput is either an uploaded STEP byte slice or a registered fixture.
// Exactly one of Bytes and FixtureID must be provided.
type STEPInput struct {
	Filename  string
	Bytes     []byte
	FixtureID string
}

// Fixture is a reusable, explicitly registered STEP source. The worker has no
// fixture names or product behavior built into its execution path.
type Fixture struct {
	Filename string
	Path     string
}

// TopologyRequest extracts sampled topology from one STEP document.
type TopologyRequest struct {
	STEP        STEPInput
	SampleCount int
}

// MeshRequest tessellates one STEP document into surface triangles for the
// workstation viewport. Deflections are the OCCT meshing tolerances; zero means
// the worker default.
type MeshRequest struct {
	STEP                 STEPInput
	LinearDeflectionMM   float64
	AngularDeflectionRad float64
	MaxTriangles         int
}

// SeamWindow is a normalized arclength interval in [0, 1].
type SeamWindow struct {
	Start float64
	End   float64
}

// SeamRequest generates a constrained weld seam and Tesseract-style sequence.
type SeamRequest struct {
	STEP            STEPInput
	SampleCount     int
	EdgeID          string
	Window          *SeamWindow
	SeamSampleCount int
	Reversed        bool
	GeometryMethod  string
	PathMethod      string
}

// SequenceSegment selects one trimmed topology edge in a weld sequence.
type SequenceSegment struct {
	ID       string  `json:"id,omitempty"`
	EdgeID   string  `json:"edge_id"`
	Start    float64 `json:"start_s"`
	End      float64 `json:"end_s"`
	Reversed bool    `json:"reversed,omitempty"`
}

// SequenceRequest generates a multi-seam weld sequence.
type SequenceRequest struct {
	STEP            STEPInput
	SampleCount     int
	SeamSampleCount int
	Segments        []SequenceSegment
}

// Result is the validated JSON response from the Python worker. JSON is copied
// on return so callers cannot mutate a cached value.
type Result struct {
	Operation   Operation
	Fingerprint string
	JSON        []byte
	CacheHit    bool
}

// Command describes one bounded subprocess invocation.
type Command struct {
	Dir            string
	Name           string
	Args           []string
	Stdin          []byte
	MaxOutputBytes int64
}

// CommandRunner makes process execution replaceable in focused tests.
type CommandRunner interface {
	Run(ctx context.Context, command Command) (stdout []byte, stderr []byte, err error)
}

// CacheStats exposes bounded-cache state without its contents.
type CacheStats struct {
	Entries int
	Bytes   int64
}
