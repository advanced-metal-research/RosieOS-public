package nats

import (
	"bufio"

	"fmt"
	"io"
	"net"

	"strconv"
	"strings"
	"time"
)

const (
	TestSubject = "robot/v4/test"

	DefaultDaemonStateURL = "http://127.0.0.1:8787/api/state"
)

type Client struct {
	conn    net.Conn
	reader  *bufio.Reader
	nextSID int
	timeout time.Duration
}

// SubscriptionMessage preserves the NATS request reply subject. The older
// NextSubscriptionMessage API intentionally remains payload-only so existing
// telemetry and test consumers do not change behavior.
type SubscriptionMessage struct {
	Subject string
	SID     int
	Reply   string
	Data    []byte
}

func TestIDFromTitle(title string) string {
	trimmed := strings.TrimSpace(title)
	dot := strings.Index(trimmed, ".")
	if dot <= 0 {
		return "unknown"
	}
	prefix := strings.TrimSpace(trimmed[:dot])
	if prefix == "" {
		return "unknown"
	}
	for _, char := range prefix {
		if char < '0' || char > '9' {
			return "unknown"
		}
	}
	return prefix
}

func TestSubjectForTitle(title string) string {
	return TestSubjectForTestID(TestIDFromTitle(title))
}

func TestSubjectForTestID(testID string) string {
	token := strings.TrimSpace(testID)
	if token == "" {
		token = "unknown"
	}
	var builder strings.Builder
	for _, char := range token {
		switch {
		case char >= 'a' && char <= 'z':
			builder.WriteRune(char)
		case char >= 'A' && char <= 'Z':
			builder.WriteRune(char)
		case char >= '0' && char <= '9':
			builder.WriteRune(char)
		case char == '-' || char == '_':
			builder.WriteRune(char)
		}
	}
	if builder.Len() == 0 {
		return TestSubject + ".unknown"
	}
	return TestSubject + "." + builder.String()
}

// PublishReply writes a Core NATS reply without a PING flush. The command
// listener owns the connection and reads serially, so avoiding flush prevents
// it from accidentally consuming and discarding another subscribed command.
func (client *Client) PublishReply(subject string, payload []byte) error {
	subject = strings.TrimSpace(subject)
	if subject == "" || strings.ContainsAny(subject, " \t\r\n") {
		return fmt.Errorf("invalid NATS reply subject %q", subject)
	}
	if err := client.conn.SetWriteDeadline(time.Now().Add(client.timeout)); err != nil {
		return err
	}
	if _, err := fmt.Fprintf(client.conn, "PUB %s %d\r\n", subject, len(payload)); err != nil {
		return err
	}
	if _, err := client.conn.Write(payload); err != nil {
		return err
	}
	_, err := io.WriteString(client.conn, "\r\n")
	return err
}

func (client *Client) readMSGEnvelope(line string) (SubscriptionMessage, error) {
	fields := strings.Fields(line)
	if len(fields) != 4 && len(fields) != 5 {
		return SubscriptionMessage{}, fmt.Errorf("invalid NATS MSG line: %q", line)
	}
	sid, err := strconv.Atoi(fields[2])
	if err != nil {
		return SubscriptionMessage{}, fmt.Errorf("invalid NATS MSG sid in %q: %w", line, err)
	}
	size, err := strconv.Atoi(fields[len(fields)-1])
	if err != nil || size < 0 {
		return SubscriptionMessage{}, fmt.Errorf("invalid NATS MSG size in %q", line)
	}
	raw := make([]byte, size+2)
	if _, err := io.ReadFull(client.reader, raw); err != nil {
		return SubscriptionMessage{}, err
	}
	if string(raw[size:]) != "\r\n" {
		return SubscriptionMessage{}, fmt.Errorf("invalid NATS MSG terminator")
	}
	message := SubscriptionMessage{
		Subject: fields[1],
		SID:     sid,
		Data:    raw[:size],
	}
	if len(fields) == 5 {
		message.Reply = fields[3]
	}
	return message, nil
}
