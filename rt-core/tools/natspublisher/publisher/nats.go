//go:build linux

package publisher

import (
	"bufio"
	"context"
	"crypto/rand"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net"
	"net/url"
	"reflect"
	"strconv"
	"strings"
	"time"
)

// Measured v4 configuration: bridge.go:25,341 and rtcoreJetStreamMaxBytes.
const Stream = "ROBOT_V4_RTCORE"

// Unverified resource ceilings: 8 KiB protocol lines and 1 MiB payloads.
const lineLimit, payloadLimit = 8192, 1 << 20

type streamConfig struct {
	Name      string   `json:"name"`
	Subjects  []string `json:"subjects"`
	Retention string   `json:"retention"`
	Storage   string   `json:"storage"`
	Discard   string   `json:"discard"`
	MaxMsgs   int64    `json:"max_msgs"`
	MaxBytes  int64    `json:"max_bytes"`
	MaxAge    int64    `json:"max_age"`
}

func desiredStream() streamConfig {
	return streamConfig{Stream, []string{"robot/v4/rtcore.>"}, "limits", "file", "old", 250000, 256 * 1024 * 1024, int64(10 * time.Minute)}
}

// ValidateURL accepts an explicit core NATS endpoint; unsupported authentication
// and TLS fail closed instead of silently using plaintext or leaking credentials.
func ValidateURL(raw string) error {
	u, err := url.Parse(raw)
	if err != nil || u.Scheme != "nats" || u.Hostname() == "" || u.Port() == "" || u.User != nil || u.Path != "" || u.RawQuery != "" || u.Fragment != "" || u.Opaque != "" {
		return errors.New("an explicit nats://host:port URL without credentials is required")
	}
	port, err := strconv.Atoi(u.Port())
	if err != nil || port < 1 || port > 65535 {
		return errors.New("invalid NATS port")
	}
	return nil
}

type natsConn struct {
	payloadAdvertised bool
	maxPayload        int
	conn              net.Conn
	reader            *bufio.Reader
	inbox             string
	stop              func() bool
}

func dialNATS(ctx context.Context, raw string) (_ *natsConn, err error) {
	u, _ := url.Parse(raw) // Config validation precedes any connection.
	c, err := (&net.Dialer{Timeout: operationTimeout}).DialContext(ctx, "tcp", u.Host)
	if err != nil {
		return nil, err
	}
	n := &natsConn{conn: c, reader: bufio.NewReaderSize(c, lineLimit)}
	n.maxPayload = payloadLimit
	n.stop = context.AfterFunc(ctx, func() { c.Close() })
	defer func() {
		if err != nil {
			n.close()
		}
	}()
	if err = c.SetDeadline(time.Now().Add(operationTimeout)); err != nil {
		return nil, err
	}
	line, err := n.line()
	if err != nil {
		return nil, err
	}
	var info struct {
		MaxPayload   int  `json:"max_payload"`
		TLSRequired  bool `json:"tls_required"`
		AuthRequired bool `json:"auth_required"`
		JetStream    bool `json:"jetstream"`
	}
	if !strings.HasPrefix(line, "INFO ") || json.Unmarshal([]byte(strings.TrimPrefix(line, "INFO ")), &info) != nil {
		return nil, errors.New("invalid NATS handshake")
	}
	if info.TLSRequired {
		return nil, ErrTLSRequired
	}
	if info.MaxPayload > 0 {
		n.maxPayload = min(payloadLimit, info.MaxPayload)
		n.payloadAdvertised = true
	}
	if info.AuthRequired {
		return nil, ErrAuthRequired
	}
	if !info.JetStream {
		return nil, ErrJetStreamUnavailable
	}
	var nonce [16]byte
	if _, err = rand.Read(nonce[:]); err != nil {
		return nil, err
	}
	n.inbox = "_INBOX.rtcore." + hex.EncodeToString(nonce[:])
	if err = n.write([]byte("CONNECT {\"verbose\":false,\"pedantic\":true,\"headers\":true,\"no_responders\":true,\"lang\":\"go\",\"version\":\"1\"}\r\nSUB " + n.inbox + " 1\r\nPING\r\n")); err != nil {
		return nil, err
	}
	for {
		line, err = n.line()
		if err != nil {
			return nil, err
		}
		if line == "PONG" {
			return n, nil
		}
		if err = n.control(line); err != nil {
			return nil, err
		}
	}
}

func (n *natsConn) close() { n.stop(); n.conn.Close() }
func (n *natsConn) line() (string, error) {
	b, err := n.reader.ReadSlice('\n')
	if err != nil {
		// A partial frame is not idle: do not let the startup guard treat its
		// timeout as successful observation of an empty subject.
		if len(b) != 0 {
			return "", fmt.Errorf("incomplete NATS frame: %v", err)
		}
		return "", err
	}
	if len(b) < 2 || b[len(b)-2] != '\r' {
		return "", errors.New("invalid NATS framing")
	}
	return string(b[:len(b)-2]), nil
}
func (n *natsConn) write(raw []byte) error {
	for len(raw) > 0 {
		count, err := n.conn.Write(raw)
		if err != nil {
			return err
		}
		if count == 0 {
			return io.ErrShortWrite
		}
		raw = raw[count:]
	}
	return nil
}
func (n *natsConn) control(line string) error {
	switch {
	case line == "PING":
		return n.write([]byte("PONG\r\n"))
	case strings.HasPrefix(line, "INFO "):
		var info struct {
			MaxPayload *int `json:"max_payload"`
		}
		if json.Unmarshal([]byte(strings.TrimPrefix(line, "INFO ")), &info) != nil {
			return errors.New("invalid NATS INFO")
		}
		if info.MaxPayload != nil {
			if *info.MaxPayload <= 0 {
				return ErrStreamConfig
			}
			n.maxPayload = min(payloadLimit, *info.MaxPayload)
			n.payloadAdvertised = true
		}
		return nil
	case line == "+OK", line == "PONG":
		return nil
	case strings.HasPrefix(line, "-ERR"):
		return fmt.Errorf("NATS refusal: %s", strings.TrimPrefix(line, "-ERR "))
	default:
		return errors.New("unexpected NATS protocol response")
	}
}

// checkPublisher observes live traffic after a subscription flush for one full
// cadence. Any message on this identity's status subject fails closed; archives
// are not replayed. Unsubscribe is flushed before any publication can echo back.
func (n *natsConn) checkPublisher(subject string, cadence time.Duration) error {
	if err := n.conn.SetDeadline(time.Now().Add(operationTimeout)); err != nil {
		return err
	}
	if err := n.write([]byte("SUB " + subject + " 2\r\nPING\r\n")); err != nil {
		return err
	}
	flush := func() error {
		for {
			line, err := n.line()
			if err != nil {
				return err
			}
			if strings.HasPrefix(line, "MSG "+subject+" 2 ") || strings.HasPrefix(line, "HMSG "+subject+" 2 ") {
				return ErrPublisherActive
			}
			if line == "PONG" {
				return nil
			}
			if err := n.control(line); err != nil {
				return err
			}
		}
	}
	if err := flush(); err != nil {
		return err
	}
	if err := n.conn.SetReadDeadline(time.Now().Add(cadence)); err != nil {
		return err
	}
	for {
		line, err := n.line()
		if err != nil {
			var timeout net.Error
			if errors.As(err, &timeout) && timeout.Timeout() {
				break
			}
			return err
		}
		if strings.HasPrefix(line, "MSG "+subject+" 2 ") || strings.HasPrefix(line, "HMSG "+subject+" 2 ") {
			return ErrPublisherActive
		}
		if err := n.conn.SetWriteDeadline(time.Now().Add(operationTimeout)); err != nil {
			return err
		}
		if err := n.control(line); err != nil {
			return err
		}
	}
	if err := n.conn.SetDeadline(time.Now().Add(operationTimeout)); err != nil {
		return err
	}
	if err := n.write([]byte("UNSUB 2\r\nPING\r\n")); err != nil {
		return err
	}
	return flush()
}

// request permits one outstanding reply and one absolute I/O deadline. A caller
// closes on failure so a late acknowledgement cannot satisfy a later publication.
func (n *natsConn) request(subject string, raw []byte) ([]byte, error) {
	if len(raw) > n.maxPayload {
		return nil, errors.New("NATS payload exceeds negotiated bound")
	}
	if err := n.conn.SetDeadline(time.Now().Add(operationTimeout)); err != nil {
		return nil, err
	}
	if err := n.write([]byte(fmt.Sprintf("PUB %s %s %d\r\n", subject, n.inbox, len(raw)))); err != nil {
		return nil, err
	}
	if err := n.write(raw); err != nil {
		return nil, err
	}
	if err := n.write([]byte("\r\n")); err != nil {
		return nil, err
	}
	for {
		line, err := n.line()
		if err != nil {
			return nil, err
		}
		f := strings.Fields(line)
		if len(f) > 0 && (f[0] == "MSG" || f[0] == "HMSG") {
			minimum := 4
			if f[0] == "HMSG" {
				minimum = 5
			}
			if (len(f) != minimum && len(f) != minimum+1) || f[1] != n.inbox || f[2] != "1" {
				return nil, errors.New("unexpected NATS reply")
			}
			size, err := strconv.Atoi(f[len(f)-1])
			if err != nil || size < 0 || size > payloadLimit {
				return nil, errors.New("invalid NATS reply size")
			}
			body := make([]byte, size+2)
			if _, err = io.ReadFull(n.reader, body); err != nil {
				return nil, err
			}
			if string(body[size:]) != "\r\n" {
				return nil, errors.New("invalid NATS reply terminator")
			}
			if f[0] == "HMSG" {
				headerSize, err := strconv.Atoi(f[len(f)-2])
				if err != nil || headerSize < 12 || headerSize > size {
					return nil, errors.New("invalid NATS header size")
				}
				header := string(body[:headerSize])
				if (!strings.HasPrefix(header, "NATS/1.0\r\n") && !strings.HasPrefix(header, "NATS/1.0 ")) || !strings.HasSuffix(header, "\r\n\r\n") {
					return nil, errors.New("invalid NATS headers")
				}
				if strings.HasPrefix(header, "NATS/1.0 503\r\n") || strings.HasPrefix(header, "NATS/1.0 503 ") {
					return nil, ErrJetStreamUnavailable
				}
				return body[headerSize:size], nil
			}
			return body[:size], nil
		}
		if err = n.control(line); err != nil {
			return nil, err
		}
	}
}

type jsResponse struct {
	Config streamConfig `json:"config"`
	Error  *struct {
		Code        int    `json:"code"`
		Description string `json:"description"`
	} `json:"error"`
	Stream   string `json:"stream"`
	Sequence uint64 `json:"seq"`
}

func (n *natsConn) api(subject string, body []byte) (jsResponse, error) {
	var response jsResponse
	raw, err := n.request(subject, body)
	if err == nil {
		err = json.Unmarshal(raw, &response)
	}
	return response, err
}

// ensureStream repairs limits in place and verifies the returned configuration;
// transport failures and incompatible storage never delete archived history.
func (n *natsConn) ensureStream() error {
	return n.ensureStreamConfig(desiredStream())
}

func (n *natsConn) ensureStreamConfig(want streamConfig) error {
	info, err := n.api("$JS.API.STREAM.INFO."+want.Name, []byte("{}"))
	if err != nil {
		return err
	}
	operation := "UPDATE"
	if info.Error != nil {
		if info.Error.Code == 503 {
			return ErrJetStreamUnavailable
		}
		if info.Error.Code != 404 {
			return fmt.Errorf("%w: inspection code=%d description=%s", ErrStreamConfig, info.Error.Code, info.Error.Description)
		}
		operation = "CREATE"
	} else if info.Config.Name != want.Name || info.Config.Storage != want.Storage || info.Config.Retention != want.Retention {
		return fmt.Errorf("%w: name=%q storage=%q retention=%q", ErrStreamConfig, info.Config.Name, info.Config.Storage, info.Config.Retention)
	} else if reflect.DeepEqual(info.Config, want) {
		return nil
	}
	raw, _ := json.Marshal(want)
	info, err = n.api("$JS.API.STREAM."+operation+"."+want.Name, raw)
	if err != nil {
		return err
	}
	if info.Error != nil {
		if info.Error.Code == 503 {
			return ErrJetStreamUnavailable
		}
		return fmt.Errorf("%w: %s code=%d description=%s", ErrStreamConfig, operation, info.Error.Code, info.Error.Description)
	}
	if !reflect.DeepEqual(info.Config, want) {
		return ErrStreamConfig
	}
	return nil
}
func (n *natsConn) publish(subject string, body []byte) error {
	return n.publishStream(subject, body, Stream)
}
func (n *natsConn) publishStream(subject string, body []byte, stream string) error {
	ack, err := n.api(subject, body)
	if err != nil {
		return err
	}
	if ack.Error != nil || ack.Stream != stream || ack.Sequence == 0 {
		return errors.New("JetStream publication was not acknowledged")
	}
	return nil
}
