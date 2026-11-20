//go:build linux

package denseexec

import (
	"context"
	"fmt"
	"log"
	"strings"
	"time"
)

// Loop mode (owner 2026-10-11: "what if we want the robot to run the program
// in a continuous loop?") repeats a Run program press: Run N times or Loop
// until stopped. The press asks once, before the first cycle, showing the
// plan's planned return to its start; every later cycle starts by itself.
// rt-core retires a program when it completes, so between cycles the loop
// stages the same bytes again through Load's live checks and native
// preparation (loadFetched) and plays them at once: no operator Load, no
// wait beyond the reload. A plan loops only when it ends where it starts
// (loopClosure): the planner adds the return leg when the client plans with
// the repeat selected.
//
// It ends, in every case with the reason in the status and the journal:
//   - at the program start after a cycle: N cycles done, Finish cycle, the
//     server's own check (a maintenance notice), or a reload or Play that
//     refused (never retried);
//   - at once, by the existing paths that stop the robot: STOP, DISARM, a
//     fault, and, for an attended loop, a lost session heartbeat or a lost
//     heartbeat from the client that started it.
//
// Attended is the default: the loop belongs to the client that started it,
// needs that client's heartbeat, and only that client may finish it. On a
// cell whose manifest sets cell_deploy.olp.unattended_loop_allowed (owner
// decision 2026-10-11 23:32 PDT; two Rosie 1430 cells) every loop is unattended: it keeps
// running when its starter, or every client, disconnects or goes quiet (no
// heartbeat deadman of either kind), and any connected client may finish it.
// STOP from any client, the hardware E-stop, a fault, a drive alarm and a
// refused reload or Play still end it at once, never retried.
type loopRun struct {
	mode, owner string
	count       int
	client      Client
	attended    bool
	returnMove  bool
	epoch       uint64
	request     RunProgramRequest
	fetched     loadBytes
	cycles      int
	cycleStart  time.Time
	lastCycle   time.Duration
	ownerBeat   time.Time
	finishing   bool
}

// startLoop records the loop whose first cycle Play just started and starts
// its supervisor. The press's epoch fences everything it does.
func (r *RTCoreController) startLoop(ctx context.Context, in RunProgramRequest, fetched loadBytes, mode string, count int, returnMove bool, epoch uint64) *LoopStatus {
	client := ClientOf(ctx)
	owner := client.Label
	if owner == "" {
		owner = "unnamed client"
	}
	now := time.Now()
	l := &loopRun{mode: mode, owner: owner, count: count, client: client, attended: !r.cfg.UnattendedLoopAllowed,
		returnMove: returnMove, epoch: epoch, request: in, fetched: fetched, cycleStart: now, ownerBeat: now}
	r.mu.Lock()
	r.loop = l
	r.lastLoop = nil
	status := r.loopStatusLocked()
	r.mu.Unlock()
	r.journalLoop(l, "cycle=1 start mode=%s count=%d attended=%t return_move=%t trajectory=%s", mode, count, l.attended, returnMove, in.TrajectoryDigest)
	go r.superviseLoop(l)
	return status
}

func (r *RTCoreController) journalLoop(l *loopRun, format string, args ...any) {
	log.Printf("dense-execution: run-program loop client=%s %s", l.owner, fmt.Sprintf(format, args...))
}

// superviseLoop waits for each cycle to complete, then starts the next one
// or ends the loop. It never retries a refused step.
func (r *RTCoreController) superviseLoop(l *loopRun) {
	ctx := WithClient(context.WithValue(context.Background(), pressEpochKey{}, l.epoch), l.client)
	ticker := time.NewTicker(rtPollInterval)
	defer ticker.Stop()
	end := func(reason string) {
		r.mu.Lock()
		if r.loop == l {
			r.loop = nil
			if !l.attended {
				// The session deadman resumes from the loop's end, not from
				// the last beat before an unattended loop began.
				r.lastHeartbeat = time.Now()
			}
			status := r.loopStatusOfLocked(l)
			status.State, status.EndReason, status.CycleElapsedS = "ended", reason, 0
			r.lastLoop = status
		}
		r.mu.Unlock()
		r.journalLoop(l, "end cycles=%d reason=%s", l.cycles, reason)
	}
	for range ticker.C {
		r.mu.Lock()
		closed, epoch, reason := r.closed, r.epoch, r.reason
		state := ""
		if r.session != nil {
			state = r.session.State
		}
		finishing, ownerBeat := l.finishing, l.ownerBeat
		r.mu.Unlock()
		switch {
		case closed:
			end("the controller closed")
			return
		case epoch != l.epoch:
			end(loopStopWords(reason))
			return
		case l.attended && time.Since(ownerBeat) > DefaultHeartbeatDeadline:
			// The existing deadman's own Stop, for the one client the loop
			// belongs to: another client's heartbeat does not keep it running.
			_ = r.fail(rtRefused("ui_heartbeat_lost: %s, which started the loop, stopped sending heartbeats", l.owner))
			end("stopped at once: " + l.owner + ", which started the loop, stopped sending heartbeats")
			return
		case state == "playing":
			continue
		case state != "done":
			end("the program ended " + state + " during cycle " + fmt.Sprint(l.cycles+1))
			return
		}
		now := time.Now()
		r.mu.Lock()
		l.cycles++
		l.lastCycle = now.Sub(l.cycleStart)
		cycles, last := l.cycles, l.lastCycle
		r.mu.Unlock()
		r.journalLoop(l, "cycle=%d end duration_s=%.3f result=done", cycles, last.Seconds())
		switch {
		case finishing:
			end(fmt.Sprintf("Finish cycle: ended at the program start after cycle %d", cycles))
			return
		case l.mode == "count" && cycles >= l.count:
			end(fmt.Sprintf("ran %d cycles", cycles))
			return
		}
		if l.request.Continue != nil {
			if why := l.request.Continue(); why != "" {
				end("ended at the program start after cycle " + fmt.Sprint(cycles) + ": " + why)
				return
			}
		}
		loaded, err := r.loadFetched(ctx, l.request.LoadRequest, l.fetched)
		if err != nil {
			if stopped, why := r.loopStopped(l); stopped {
				end(why)
				return
			}
			_, detail := denseErrorWords(err)
			end(fmt.Sprintf("Load refused before cycle %d: %s", cycles+1, detail))
			return
		}
		r.mu.Lock()
		l.cycleStart = time.Now()
		r.mu.Unlock()
		if _, err := r.Play(ctx, loaded.SessionID); err != nil {
			if stopped, why := r.loopStopped(l); stopped {
				end(why)
				return
			}
			_, detail := denseErrorWords(err)
			end(fmt.Sprintf("Play refused before cycle %d: %s", cycles+1, detail))
			return
		}
		r.journalLoop(l, "cycle=%d start session=%s", cycles+1, loaded.SessionID)
	}
}

// loopStopped: STOP, DISARM or a lost heartbeat ended the loop's epoch while
// a step ran, and why. A step's own failure also stops the robot (a new
// epoch) and keeps its words.
func (r *RTCoreController) loopStopped(l *loopRun) (bool, string) {
	r.mu.Lock()
	defer r.mu.Unlock()
	cause, _, _ := strings.Cut(r.reason, ":")
	outside := cause == "operator_stop" || cause == "stop" || cause == "disarm" || cause == "ui_heartbeat_lost"
	return r.epoch != l.epoch && outside, loopStopWords(r.reason)
}

// loopStopWords is why a Stop ended the loop at once, from the reason the
// Stop recorded.
func loopStopWords(reason string) string {
	cause, _, _ := strings.Cut(reason, ":")
	switch cause {
	case "operator_stop", "stop":
		return "stopped at once by STOP"
	case "disarm":
		return "stopped at once by DISARM"
	case "ui_heartbeat_lost":
		return "stopped at once: no client heartbeat (" + reason + ")"
	case "":
		return "stopped at once"
	}
	return "stopped at once: " + reason
}

// FinishLoop ends the running loop cleanly: the current cycle completes and
// the arm stays at the program start. Only the client that started an
// attended loop may finish it; any client may finish an unattended one.
// STOP stops either from anywhere.
func (r *RTCoreController) FinishLoop(ctx context.Context) (*LoopStatus, error) {
	r.mu.Lock()
	defer r.mu.Unlock()
	l := r.loop
	if l == nil {
		return r.lastLoop, httpError(409, "loop_not_running", "No loop is running")
	}
	if l.attended && !isLoopOwner(l.client, ClientOf(ctx)) {
		return r.loopStatusLocked(), httpError(409, "loop_owner_only", "Only %s, which started the loop, can finish it; STOP stops it from anywhere", l.owner)
	}
	if !l.finishing {
		l.finishing = true
		log.Printf("dense-execution: run-program loop client=%s finish requested cycle=%d", l.owner, l.cycles+1)
	}
	return r.loopStatusLocked(), nil
}

// HeartbeatFrom is Heartbeat, also noting a beat from the client that started
// an attended loop (its deadman).
func (r *RTCoreController) HeartbeatFrom(ctx context.Context, id string) (*HeartbeatResult, error) {
	result, err := r.Heartbeat(id)
	if err == nil {
		r.mu.Lock()
		if r.loop != nil && isLoopOwner(r.loop.client, ClientOf(ctx)) {
			r.loop.ownerBeat = time.Now()
		}
		r.mu.Unlock()
	}
	return result, err
}

// isLoopOwner: the client id when the owner sent one (strictly), else its label.
func isLoopOwner(owner, c Client) bool {
	if owner.ID != "" {
		return owner.ID == c.ID
	}
	return owner.Label != "" && owner.Label == c.Label
}

// loopStatusLocked is the running loop, else the last ended one. Hold mu.
func (r *RTCoreController) loopStatusLocked() *LoopStatus {
	if r.loop == nil {
		if r.lastLoop == nil {
			return nil
		}
		ended := *r.lastLoop
		return &ended
	}
	return r.loopStatusOfLocked(r.loop)
}

func (r *RTCoreController) loopStatusOfLocked(l *loopRun) *LoopStatus {
	state := "running"
	if l.finishing {
		state = "finishing"
	}
	status := &LoopStatus{Mode: l.mode, Count: l.count, Cycles: l.cycles, State: state, Owner: l.owner, Attended: l.attended,
		ReturnMove: l.returnMove, LastCycleS: l.lastCycle.Seconds(), TrajectoryDigest: l.request.TrajectoryDigest}
	if !l.cycleStart.IsZero() {
		status.CycleElapsedS = time.Since(l.cycleStart).Seconds()
	}
	return status
}
