package simulator

import (
	"errors"
	"fmt"
)

type ErrorCode string

const (
	ErrorInvalidConfiguration ErrorCode = "invalid_configuration"
	ErrorInvalidLifecycle     ErrorCode = "invalid_lifecycle"
	ErrorBackendUnavailable   ErrorCode = "backend_unavailable"
	ErrorUnsafeRuntime        ErrorCode = "unsafe_runtime"
	ErrorNotReady             ErrorCode = "not_ready"
	ErrorBusy                 ErrorCode = "busy"
	ErrorInvalidPlan          ErrorCode = "invalid_plan"
	ErrorPlanRejected         ErrorCode = "plan_rejected"
	ErrorUncorrelatedReport   ErrorCode = "uncorrelated_report"
	ErrorResourceLimit        ErrorCode = "resource_limit"
)

var (
	ErrInvalidConfiguration = errors.New("simulator: invalid configuration")
	ErrInvalidLifecycle     = errors.New("simulator: invalid lifecycle")
	ErrBackendUnavailable   = errors.New("simulator: backend unavailable")
	ErrUnsafeRuntime        = errors.New("simulator: unsafe runtime")
	ErrNotReady             = errors.New("simulator: not ready")
	ErrBusy                 = errors.New("simulator: busy")
	ErrInvalidPlan          = errors.New("simulator: invalid plan")
	ErrPlanRejected         = errors.New("simulator: plan rejected")
	ErrUncorrelatedReport   = errors.New("simulator: uncorrelated report")
	ErrResourceLimit        = errors.New("simulator: resource limit exceeded")
)

type ServiceError struct {
	Code      ErrorCode `json:"code"`
	Operation string    `json:"operation"`
	Message   string    `json:"message"`
	Cause     error     `json:"-"`
}

func (err *ServiceError) Error() string {
	if err == nil {
		return "<nil>"
	}
	if err.Message != "" {
		return fmt.Sprintf("simulator %s: %s", err.Operation, err.Message)
	}
	return fmt.Sprintf("simulator %s: %v", err.Operation, err.Cause)
}

func (err *ServiceError) Unwrap() error {
	if err == nil {
		return nil
	}
	return err.Cause
}

func serviceError(code ErrorCode, operation, message string, cause error) error {
	if cause == nil {
		cause = sentinelFor(code)
	}
	return &ServiceError{
		Code:      code,
		Operation: operation,
		Message:   message,
		Cause:     cause,
	}
}

func sentinelFor(code ErrorCode) error {
	switch code {
	case ErrorInvalidConfiguration:
		return ErrInvalidConfiguration
	case ErrorInvalidLifecycle:
		return ErrInvalidLifecycle
	case ErrorBackendUnavailable:
		return ErrBackendUnavailable
	case ErrorUnsafeRuntime:
		return ErrUnsafeRuntime
	case ErrorNotReady:
		return ErrNotReady
	case ErrorBusy:
		return ErrBusy
	case ErrorInvalidPlan:
		return ErrInvalidPlan
	case ErrorPlanRejected:
		return ErrPlanRejected
	case ErrorUncorrelatedReport:
		return ErrUncorrelatedReport
	case ErrorResourceLimit:
		return ErrResourceLimit
	default:
		return errors.New("simulator: service failure")
	}
}
