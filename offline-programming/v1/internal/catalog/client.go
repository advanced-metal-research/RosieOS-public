// Package catalog implements the server-only bridge from Offline Programming
// to the Motion Server-owned Robot V4 program catalog.
package catalog

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net/http"
	"net/url"
	"regexp"
	"strconv"
	"strings"
	"time"
)

const (
	UpstreamPath       = "/api/program-catalog"
	MaxRequestBytes    = 8 << 20
	MaxResponseBytes   = 16 << 20
	DefaultHTTPTimeout = 8 * time.Second
)

var (
	ErrUnavailable     = errors.New("program catalog is unavailable")
	ErrInvalidResponse = errors.New("program catalog returned an invalid response")
	identityPattern    = regexp.MustCompile(`^[A-Za-z0-9._-]{1,128}$`)
)

type MutationContext struct {
	RequestID      string
	CorrelationID  string
	IdempotencyKey string
}

type Response struct {
	Status int
	Body   []byte
}

type Gateway interface {
	Available() bool
	List(context.Context) (Response, error)
	Get(context.Context, string, *uint64) (Response, error)
	Revisions(context.Context, string) (Response, error)
	Save(context.Context, []byte, MutationContext) (Response, error)
}

type HTTPGateway struct {
	origin *url.URL
	client *http.Client
}

func NewHTTPGateway(rawOrigin string, client *http.Client) (*HTTPGateway, error) {
	origin, err := url.Parse(strings.TrimSpace(rawOrigin))
	if err != nil {
		return nil, fmt.Errorf("program catalog origin: %w", err)
	}
	if (origin.Scheme != "http" && origin.Scheme != "https") ||
		origin.Hostname() == "" || origin.User != nil ||
		(origin.Path != "" && origin.Path != "/") ||
		origin.RawQuery != "" || origin.Fragment != "" {
		return nil, errors.New("program catalog origin must be an HTTP(S) origin with a concrete host")
	}
	origin.Path = ""
	if client == nil {
		client = &http.Client{Timeout: DefaultHTTPTimeout}
	} else {
		copy := *client
		client = &copy
		if client.Timeout <= 0 {
			client.Timeout = DefaultHTTPTimeout
		}
	}
	client.CheckRedirect = func(*http.Request, []*http.Request) error {
		return errors.New("program catalog redirects are disabled")
	}
	return &HTTPGateway{origin: origin, client: client}, nil
}

func (gateway *HTTPGateway) Available() bool {
	return gateway != nil && gateway.origin != nil && gateway.client != nil
}

func (gateway *HTTPGateway) List(ctx context.Context) (Response, error) {
	return gateway.do(ctx, http.MethodGet, UpstreamPath, nil, MutationContext{})
}

func (gateway *HTTPGateway) Get(
	ctx context.Context,
	programID string,
	revision *uint64,
) (Response, error) {
	path, err := programPath(programID)
	if err != nil {
		return Response{}, err
	}
	if revision != nil {
		if *revision == 0 {
			return Response{}, errors.New("program revision must be positive")
		}
		path += "?revision=" + strconv.FormatUint(*revision, 10)
	}
	return gateway.do(ctx, http.MethodGet, path, nil, MutationContext{})
}

func (gateway *HTTPGateway) Revisions(ctx context.Context, programID string) (Response, error) {
	path, err := programPath(programID)
	if err != nil {
		return Response{}, err
	}
	return gateway.do(ctx, http.MethodGet, path+"/revisions", nil, MutationContext{})
}

func (gateway *HTTPGateway) Save(
	ctx context.Context,
	body []byte,
	mutation MutationContext,
) (Response, error) {
	if len(body) == 0 || len(body) > MaxRequestBytes || !json.Valid(body) {
		return Response{}, errors.New("program catalog save body is invalid")
	}
	if err := mutation.Validate(); err != nil {
		return Response{}, err
	}
	// The daemon's POST operation is an optimistic single-entry upsert. PUT
	// replaces the entire catalog and is deliberately never used here.
	return gateway.do(ctx, http.MethodPost, UpstreamPath, body, mutation)
}

func (mutation MutationContext) Validate() error {
	for name, value := range map[string]string{
		"request id":      mutation.RequestID,
		"correlation id":  mutation.CorrelationID,
		"idempotency key": mutation.IdempotencyKey,
	} {
		value = strings.TrimSpace(value)
		if len(value) < 8 || len(value) > 160 {
			return fmt.Errorf("program catalog %s is required and must be bounded", name)
		}
		for _, char := range value {
			if char < 0x21 || char > 0x7e {
				return fmt.Errorf("program catalog %s contains unsupported characters", name)
			}
		}
	}
	return nil
}

func ValidProgramID(value string) bool {
	return identityPattern.MatchString(strings.TrimSpace(value))
}

func programPath(programID string) (string, error) {
	programID = strings.TrimSpace(programID)
	if !ValidProgramID(programID) {
		return "", errors.New("program identity is invalid")
	}
	return UpstreamPath + "/" + url.PathEscape(programID), nil
}

func (gateway *HTTPGateway) do(
	ctx context.Context,
	method string,
	path string,
	body []byte,
	mutation MutationContext,
) (Response, error) {
	if !gateway.Available() {
		return Response{}, ErrUnavailable
	}
	endpoint := *gateway.origin
	endpoint.Path = path
	if queryIndex := strings.IndexByte(path, '?'); queryIndex >= 0 {
		endpoint.Path = path[:queryIndex]
		endpoint.RawQuery = path[queryIndex+1:]
	}
	var reader io.Reader
	if body != nil {
		reader = bytes.NewReader(body)
	}
	request, err := http.NewRequestWithContext(ctx, method, endpoint.String(), reader)
	if err != nil {
		return Response{}, fmt.Errorf("%w: build request", ErrUnavailable)
	}
	request.Header.Set("Accept", "application/json")
	if body != nil {
		request.Header.Set("Content-Type", "application/json")
		request.Header.Set("X-Request-ID", mutation.RequestID)
		request.Header.Set("X-Correlation-ID", mutation.CorrelationID)
		request.Header.Set("Idempotency-Key", mutation.IdempotencyKey)
	}
	response, err := gateway.client.Do(request)
	if err != nil {
		return Response{}, fmt.Errorf("%w: request failed", ErrUnavailable)
	}
	defer response.Body.Close()
	raw, err := io.ReadAll(io.LimitReader(response.Body, MaxResponseBytes+1))
	if err != nil {
		return Response{}, fmt.Errorf("%w: read failed", ErrInvalidResponse)
	}
	if len(raw) > MaxResponseBytes || !json.Valid(raw) {
		return Response{}, ErrInvalidResponse
	}
	return Response{Status: response.StatusCode, Body: raw}, nil
}
