package simulator

// Minimal in-process NATS broker: just enough of the text protocol
// (INFO/CONNECT/PING/SUB/PUB/MSG) for robot-v4-cartesiand's command and
// status subscribers, so the jog session can run the daemon's REAL leader
// lease authority plane without an external NATS deployment. Reference for
// the shape: motion-server/v1/test/cartesian_solver_support_test.go.

import (
	"bufio"
	"fmt"
	"io"
	"net"
	"strconv"
	"strings"
	"sync"
)

type jogNATSBroker struct {
	listener net.Listener
	address  string

	mu          sync.Mutex
	subscribers map[string][]*jogNATSSubscription
	publishes   map[string]int
	connections int
	closed      bool
}

type jogNATSSubscription struct {
	subject string
	sid     string
	conn    net.Conn // nil for broker-local subscriptions
	// connMu serializes MSG frames: concurrent publishes must not interleave
	// bytes on the shared TCP stream.
	connMu *sync.Mutex
	local  chan jogNATSMessage
}

type jogNATSMessage struct {
	Subject string
	Reply   string
	Payload []byte
}

func startJogNATSBroker() (*jogNATSBroker, error) {
	listener, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		return nil, fmt.Errorf("bind jog NATS broker: %w", err)
	}
	broker := &jogNATSBroker{
		listener:    listener,
		address:     listener.Addr().String(),
		subscribers: make(map[string][]*jogNATSSubscription),
		publishes:   make(map[string]int),
	}
	go broker.acceptLoop()
	return broker, nil
}

func (broker *jogNATSBroker) URL() string {
	return "nats://" + broker.address
}

func (broker *jogNATSBroker) Close() {
	broker.mu.Lock()
	broker.closed = true
	broker.mu.Unlock()
	broker.listener.Close()
}

// SubscribeLocal delivers messages published on subject to a broker-local
// channel (the session's own ear on the plane).
func (broker *jogNATSBroker) SubscribeLocal(subject string) <-chan jogNATSMessage {
	channel := make(chan jogNATSMessage, 64)
	broker.mu.Lock()
	broker.subscribers[subject] = append(broker.subscribers[subject], &jogNATSSubscription{
		subject: subject,
		local:   channel,
	})
	broker.mu.Unlock()
	return channel
}

// PublishLocal routes a payload to every subscriber of subject.
func (broker *jogNATSBroker) PublishLocal(subject, reply string, payload []byte) {
	broker.route(jogNATSMessage{Subject: subject, Reply: reply, Payload: payload})
}

func (broker *jogNATSBroker) route(message jogNATSMessage) {
	broker.mu.Lock()
	subscriptions := append([]*jogNATSSubscription(nil), broker.subscribers[message.Subject]...)
	broker.mu.Unlock()
	for _, subscription := range subscriptions {
		if subscription.local != nil {
			select {
			case subscription.local <- message:
			default:
			}
			continue
		}
		header := "MSG " + message.Subject + " " + subscription.sid
		if message.Reply != "" {
			header += " " + message.Reply
		}
		frame := fmt.Sprintf("%s %d\r\n%s\r\n", header, len(message.Payload), message.Payload)
		subscription.connMu.Lock()
		_, _ = io.WriteString(subscription.conn, frame)
		subscription.connMu.Unlock()
	}
}

func (broker *jogNATSBroker) acceptLoop() {
	for {
		conn, err := broker.listener.Accept()
		if err != nil {
			return
		}
		go broker.handle(conn)
	}
}

func (broker *jogNATSBroker) handle(conn net.Conn) {
	broker.mu.Lock()
	broker.connections++
	broker.mu.Unlock()
	defer func() {
		broker.dropConnection(conn)
		conn.Close()
	}()
	connMu := &sync.Mutex{}
	writeString := func(text string) {
		connMu.Lock()
		_, _ = io.WriteString(conn, text)
		connMu.Unlock()
	}
	writeString("INFO {\"server_id\":\"olp-jog-nats\",\"max_payload\":1048576}\r\n")
	reader := bufio.NewReader(conn)
	for {
		line, err := reader.ReadString('\n')
		if err != nil {
			return
		}
		line = strings.TrimRight(line, "\r\n")
		switch {
		case line == "PING":
			writeString("PONG\r\n")
		case strings.HasPrefix(line, "CONNECT"):
			writeString("+OK\r\n")
		case strings.HasPrefix(line, "SUB "):
			fields := strings.Fields(line[4:])
			if len(fields) < 2 {
				continue
			}
			broker.mu.Lock()
			broker.subscribers[fields[0]] = append(broker.subscribers[fields[0]], &jogNATSSubscription{
				subject: fields[0],
				sid:     fields[len(fields)-1],
				conn:    conn,
				connMu:  connMu,
			})
			broker.mu.Unlock()
		case strings.HasPrefix(line, "PUB "):
			fields := strings.Fields(line[4:])
			if len(fields) < 2 {
				continue
			}
			size, err := strconv.Atoi(fields[len(fields)-1])
			if err != nil || size < 0 || size > 1<<20 {
				return
			}
			reply := ""
			if len(fields) == 3 {
				reply = fields[1]
			}
			payload := make([]byte, size+2)
			if _, err := io.ReadFull(reader, payload); err != nil {
				return
			}
			broker.mu.Lock()
			broker.publishes[fields[0]]++
			broker.mu.Unlock()
			broker.route(jogNATSMessage{
				Subject: fields[0],
				Reply:   reply,
				Payload: payload[:size],
			})
		}
	}
}

// Traffic summarizes publish counts + connection total (diagnostics).
func (broker *jogNATSBroker) Traffic() string {
	broker.mu.Lock()
	defer broker.mu.Unlock()
	return fmt.Sprintf("conns=%d publishes=%v", broker.connections, broker.publishes)
}

// Subjects lists current subscriptions (diagnostics).
func (broker *jogNATSBroker) Subjects() []string {
	broker.mu.Lock()
	defer broker.mu.Unlock()
	subjects := make([]string, 0, len(broker.subscribers))
	for subject, subscriptions := range broker.subscribers {
		if len(subscriptions) > 0 {
			subjects = append(subjects, fmt.Sprintf("%s(%d)", subject, len(subscriptions)))
		}
	}
	return subjects
}

func (broker *jogNATSBroker) dropConnection(conn net.Conn) {
	broker.mu.Lock()
	defer broker.mu.Unlock()
	for subject, subscriptions := range broker.subscribers {
		kept := subscriptions[:0]
		for _, subscription := range subscriptions {
			if subscription.conn != conn {
				kept = append(kept, subscription)
			}
		}
		broker.subscribers[subject] = kept
	}
}
