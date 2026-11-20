package simulator

import (
	"errors"
	"runtime"
	"sync"
)

const (
	HTTPJogStartPath   = "/api/offline-programming/v1/local-simulator/jog/start"
	HTTPJogStopPath    = "/api/offline-programming/v1/local-simulator/jog/stop"
	HTTPJogIntentPath  = "/api/offline-programming/v1/local-simulator/jog/intent"
	HTTPJogStatePath   = "/api/offline-programming/v1/local-simulator/jog/state"
	HTTPJogHomePath    = "/api/offline-programming/v1/local-simulator/jog/home"
	HTTPJogHaltPath    = "/api/offline-programming/v1/local-simulator/jog/halt"
	HTTPJogSetZeroPath = "/api/offline-programming/v1/local-simulator/jog/set-zero"
	HTTPJogStepPath    = "/api/offline-programming/v1/local-simulator/jog/step"
)

type JogManager struct {
	mu       sync.Mutex
	repoRoot string
	binary   string
	session  *jogSession
	stepBusy bool
}

func NewJogManager(repoRoot string) *JogManager {
	return &JogManager{repoRoot: repoRoot}
}

// Start launches the jog daemon + pendant loops; idempotent while running.
func (manager *JogManager) Start() (JogState, error) {
	manager.mu.Lock()
	defer manager.mu.Unlock()
	if manager.session != nil {
		state := manager.session.State()
		if state.Running {
			return state, nil
		}
		manager.session.Stop()
		manager.session = nil
	}
	binary, err := ResolveCanonicalJogBinary(manager.repoRoot)
	if err != nil {
		return JogState{}, err
	}
	if runtime.GOOS == "windows" {
		return JogState{}, errors.New(
			"jog runtime requires WSL: native Windows cannot exec robot-v4-cartesiand (Linux ELF). Use start-offline-programming.ps1")
	}
	manager.binary = binary
	session, err := startJogSession(manager.repoRoot, manager.binary, nil)
	if err != nil {
		return JogState{}, err
	}
	manager.session = session
	return session.State(), nil
}

// Intent forwards a pendant input; the session streams it at pendant cadence.
func (manager *JogManager) Intent(intent JogIntent) (JogState, error) {
	manager.mu.Lock()
	session := manager.session
	manager.mu.Unlock()
	if session == nil {
		return JogState{}, errors.New("jog session is not running — start it first")
	}
	if err := session.SetIntent(intent); err != nil {
		return session.State(), err
	}
	return session.State(), nil
}

// State reports the current jog session snapshot.
func (manager *JogManager) State() JogState {
	manager.mu.Lock()
	session := manager.session
	manager.mu.Unlock()
	if session == nil {
		return JogState{Running: false}
	}
	return session.State()
}

// SetZero refuses coordinate changes through the Cartesian jog endpoint.
func (manager *JogManager) SetZero(axisMask uint32) (JogState, error) {
	manager.mu.Lock()
	session := manager.session
	manager.mu.Unlock()
	if session == nil {
		return JogState{}, errors.New("jog session is not running — start it first")
	}
	return session.State(), errors.New("set-zero is unavailable for rt_core Cartesian jog")
}

// Halt is the dominant stop: release edge now, cancel any homing.
func (manager *JogManager) Halt() JogState {
	manager.mu.Lock()
	session := manager.session
	manager.mu.Unlock()
	if session == nil {
		return JogState{Running: false}
	}
	session.StopMotion()
	return session.State()
}

// Stop tears the session down; safe when idle.
func (manager *JogManager) Stop() {
	manager.mu.Lock()
	session := manager.session
	manager.session = nil
	manager.mu.Unlock()
	if session != nil {
		session.Stop()
	}
}
