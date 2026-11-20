package denseexec

import (
	"bufio"
	"context"
	"encoding/json"
	"errors"
	"io"
	"log"
	"os"
	"os/exec"
	"strings"
	"time"
)

type CartesianResolverService interface {
	Resolve(context.Context, CartesianResolveRequest) (CartesianResolution, error)
	Close()
}
type CartesianResolverFactory func(model string) CartesianResolverService
type resolverCall struct {
	ctx   context.Context
	in    CartesianResolveRequest
	reply chan resolverReply
}
type resolverReply struct {
	result CartesianResolution
	err    error
}

// CartesianProcess owns a model-pinned computation process for one connection.
// Serialized bounded exchanges prevent a late reply from satisfying another call.
type CartesianProcess struct {
	calls  chan resolverCall
	cancel context.CancelFunc
	done   chan struct{}
}

// resolverArgv builds the resolver command. binary is a command line, not only
// a path: the cell's own motion server runs the kinematics when it reads
// "ssh <host> <path>/robot-v4-cartesiand", so the heavy computation stays on the
// device and the offline-programming host only relays one request per step.
// repo is the tree the resolver reads its URDF from, on the host that runs it.
func resolverArgv(binary, repo, model string) []string {
	argv := strings.Fields(binary)
	if len(argv) == 0 {
		argv = []string{binary}
	}
	return append(argv, "--resolve-only", repo, model)
}

// NewCartesianProcess starts at connection setup and restarts after exit. It
// passes no controller address or authority to the computation-only process.
func NewCartesianProcess(binary, repo, model string) *CartesianProcess {
	ctx, cancel := context.WithCancel(context.Background())
	p := &CartesianProcess{calls: make(chan resolverCall), cancel: cancel, done: make(chan struct{})}
	go p.run(ctx, binary, repo, model)
	return p
}

func (p *CartesianProcess) Close() { p.cancel(); <-p.done }

func (p *CartesianProcess) Resolve(ctx context.Context, in CartesianResolveRequest) (CartesianResolution, error) {
	call := resolverCall{ctx: ctx, in: in, reply: make(chan resolverReply, 1)}
	select {
	case <-ctx.Done():
		return CartesianResolution{}, ctx.Err()
	case <-p.done:
		return CartesianResolution{}, errors.New("resolver closed")
	case p.calls <- call:
	}
	select {
	case <-ctx.Done():
		return CartesianResolution{}, ctx.Err()
	case <-p.done:
		return CartesianResolution{}, errors.New("resolver closed")
	case reply := <-call.reply:
		return reply.result, reply.err
	}
}

func (p *CartesianProcess) run(ctx context.Context, binary, repo, model string) {
	defer close(p.done)
	for ctx.Err() == nil {
		argv := resolverArgv(binary, repo, model)
		command := exec.CommandContext(ctx, argv[0], argv[1:]...)
		// The child's diagnostics (an ssh refusal, a missing URDF) belong in the
		// server log; a silent exit 255 is not actionable.
		command.Stderr = os.Stderr
		input, err := command.StdinPipe()
		var output io.ReadCloser
		if err == nil {
			output, err = command.StdoutPipe()
		}
		if err == nil {
			err = command.Start()
		}
		if err != nil {
			log.Printf("cartesian resolver: start failed for %q: %v", argv[0], err)
			if input != nil {
				input.Close()
			}
			if output != nil {
				output.Close()
			}
		} else {
			exited := make(chan struct{})
			log.Printf("cartesian resolver: started pid=%d %q", command.Process.Pid, argv)
			go func() {
				err := command.Wait()
				log.Printf("cartesian resolver: process pid=%d exited: %v", command.Process.Pid, err)
				close(exited)
			}()
			// Unverified response bound: 1048576 bytes covers 1001 six-axis waypoints.
			reader := bufio.NewReaderSize(output, 1048576)
			running := true
			for running {
				select {
				case <-ctx.Done():
					running = false
				case <-exited:
					running = false
				case call := <-p.calls:
					if call.ctx.Err() != nil {
						call.reply <- resolverReply{err: call.ctx.Err()}
						continue
					}
					finished := make(chan resolverReply, 1)
					go func() {
						var reply resolverReply
						if call.in.Model != model {
							reply.err = errors.New("resolver model mismatch")
						} else {
							reply.err = json.NewEncoder(input).Encode(call.in)
							if reply.err == nil {
								var raw []byte
								raw, reply.err = reader.ReadSlice('\n')
								if reply.err == nil {
									reply.err = json.Unmarshal(raw, &reply.result)
								}
							}
						}
						finished <- reply
					}()
					var reply resolverReply
					select {
					case reply = <-finished:
					case <-call.ctx.Done():
						_ = command.Process.Kill()
						<-finished
						reply.err = call.ctx.Err()
					case <-ctx.Done():
						_ = command.Process.Kill()
						<-finished
						reply.err = ctx.Err()
					}
					if reply.err != nil {
						log.Printf("cartesian resolver: exchange failed, restarting %q: %v", argv[0], reply.err)
					}
					call.reply <- reply
					running = reply.err == nil
				}
			}
			_ = command.Process.Kill()
			input.Close()
			output.Close()
			<-exited
		}
		// Unverified restart backoff: 1000 ms prevents a broken executable spinning.
		retry := time.NewTimer(1000 * time.Millisecond)
		waiting := true
		for waiting {
			select {
			case <-ctx.Done():
				retry.Stop()
				return
			case <-retry.C:
				waiting = false
			case call := <-p.calls:
				call.reply <- resolverReply{err: errors.New("resolver unavailable: restarting after a failed exchange")}
			}
		}
	}
}
