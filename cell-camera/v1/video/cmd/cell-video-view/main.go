// Command cell-video-view receives the cell camera's WebRTC video for a
// native client, which decodes what it writes and draws it. Not used by the
// Steam Deck pendant yet: the Decks' Tailscale runs in userspace-networking
// mode, so they have no route for the sender's UDP (a Steam Deck, 2026-10-19),
// and the pendant plays the OLP's relayed stream instead (cell_video_stream.go).
// With kernel Tailscale on a Deck, this is its separate camera lane.
//
//	cell-video-view --whep <url> [--header 'Name: value' ...]
//
// stdout: each H.264 access unit (Annex-B, a key frame first) as a 4-byte
// big-endian length and the unit: the camera service's video.h264 framing.
// stdin: a line "keyframe" asks the sender for a key frame (the decoder lost
// its place); end of input ends the session (WHEP DELETE) and the program.
// stderr: what happened, for the pendant's log.
//
// Exit status: 0 when stdin closed or a signal stopped it; 3 when the cell
// serves no video to this client (the pendant shows JPEG frames instead);
// 1 on any other failure (the pendant starts it again).
package main

import (
	"bufio"
	"context"
	"encoding/binary"
	"errors"
	"flag"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	"rosieos/cell-video/viewer"
)

type headers []string

func (h *headers) String() string     { return strings.Join(*h, ", ") }
func (h *headers) Set(v string) error { *h = append(*h, v); return nil }

func main() {
	var extra headers
	whep := flag.String("whep", "", "the cell's WHEP endpoint, through the OLP server")
	flag.Var(&extra, "header", "a request header 'Name: value' (repeatable)")
	flag.Parse()
	log.SetPrefix("cell-video-view: ")
	log.SetFlags(0)
	if *whep == "" || flag.NArg() != 0 {
		fmt.Fprintln(os.Stderr, "usage: cell-video-view --whep <url> [--header 'Name: value' ...]")
		os.Exit(2)
	}
	h := http.Header{}
	for _, line := range extra {
		name, value, ok := strings.Cut(line, ":")
		if !ok {
			log.Fatalf("--header %q is not 'Name: value'", line)
		}
		h.Add(strings.TrimSpace(name), strings.TrimSpace(value))
	}

	ctx, stop := signal.NotifyContext(context.Background(), syscall.SIGTERM, syscall.SIGINT)
	defer stop()
	ctx, cancel := context.WithCancel(ctx)
	keyframe := make(chan struct{}, 1)
	go func() {
		lines := bufio.NewScanner(os.Stdin)
		for lines.Scan() {
			if strings.TrimSpace(lines.Text()) == "keyframe" {
				select {
				case keyframe <- struct{}{}:
				default:
				}
			}
		}
		cancel() // the pendant closed our input, or exited
	}()

	out := bufio.NewWriterSize(os.Stdout, 256<<10)
	var head [4]byte
	deliver := func(unit []byte) error {
		binary.BigEndian.PutUint32(head[:], uint32(len(unit)))
		if _, err := out.Write(head[:]); err != nil {
			return err
		}
		if _, err := out.Write(unit); err != nil {
			return err
		}
		return out.Flush()
	}
	started := time.Now()
	session, err := viewer.Run(ctx, viewer.Options{WHEP: *whep, Header: h}, keyframe, deliver)
	st := session.Stats()
	log.Printf("ended after %.0f s: %d bytes in %d packets, %d units, %d losses, %d key frame requests",
		time.Since(started).Seconds(), st.Bytes, st.Packets, st.Units, st.Dropped, st.KeyRequests)
	switch {
	case errors.Is(err, viewer.ErrNoVideo):
		log.Print(err)
		os.Exit(3)
	case err != nil:
		log.Print(err)
		os.Exit(1)
	}
}
