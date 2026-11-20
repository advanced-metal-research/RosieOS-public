package nats

import (
	"bufio"
	"bytes"
	"fmt"
	"io"
	"net"
	"testing"
	"time"
)

func TestPublishReplyWritesOneCoreNATSMessageWithoutFlush(t *testing.T) {
	clientConn, serverConn := net.Pipe()
	defer clientConn.Close()
	defer serverConn.Close()
	client := &Client{
		conn:    clientConn,
		reader:  bufio.NewReader(clientConn),
		timeout: time.Second,
	}

	payload := []byte(`{"schema":"robot.v4.command-reply.v1","accepted":true}`)
	readDone := make(chan []byte, 1)
	go func() {
		reader := bufio.NewReader(serverConn)
		line, _ := reader.ReadString('\n')
		body := make([]byte, len(payload)+2)
		_, _ = io.ReadFull(reader, body)
		readDone <- append([]byte(line), body...)
	}()

	if err := client.PublishReply("_INBOX.deck.2", payload); err != nil {
		t.Fatal(err)
	}
	want := append([]byte(fmt.Sprintf("PUB _INBOX.deck.2 %d\r\n", len(payload))), payload...)
	want = append(want, '\r', '\n')
	got := <-readDone
	if !bytes.Equal(got, want) {
		t.Fatalf("reply frame = %q, want %q", got, want)
	}
}
