//go:build windows

package simulator

import (
	"errors"
	"fmt"
	"os"
	"os/exec"
	"syscall"
	"time"
	"unsafe"
)

const (
	createNewProcessGroup                  = 0x00000200
	jobObjectExtendedLimitInformationClass = 9
	jobObjectLimitKillOnJobClose           = 0x00002000
	processTerminate                       = 0x0001
	processSetQuota                        = 0x0100
	processQueryInformation                = 0x0400
)

var (
	kernel32                 = syscall.NewLazyDLL("kernel32.dll")
	createJobObjectW         = kernel32.NewProc("CreateJobObjectW")
	setInformationJobObject  = kernel32.NewProc("SetInformationJobObject")
	assignProcessToJobObject = kernel32.NewProc("AssignProcessToJobObject")
	terminateJobObject       = kernel32.NewProc("TerminateJobObject")
	openProcess              = kernel32.NewProc("OpenProcess")
	closeHandle              = kernel32.NewProc("CloseHandle")
)

type jobObjectBasicLimitInformation struct {
	PerProcessUserTimeLimit int64
	PerJobUserTimeLimit     int64
	LimitFlags              uint32
	MinimumWorkingSetSize   uintptr
	MaximumWorkingSetSize   uintptr
	ActiveProcessLimit      uint32
	Affinity                uintptr
	PriorityClass           uint32
	SchedulingClass         uint32
}

type ioCounters struct {
	ReadOperationCount  uint64
	WriteOperationCount uint64
	OtherOperationCount uint64
	ReadTransferCount   uint64
	WriteTransferCount  uint64
	OtherTransferCount  uint64
}

type jobObjectExtendedLimitInformation struct {
	BasicLimitInformation jobObjectBasicLimitInformation
	IoInfo                ioCounters
	ProcessMemoryLimit    uintptr
	JobMemoryLimit        uintptr
	PeakProcessMemoryUsed uintptr
	PeakJobMemoryUsed     uintptr
}

type commandContainment struct {
	command *exec.Cmd
	job     syscall.Handle
}

func boundedProcessSupported() bool {
	return true
}

func prepareBoundedCommand(command *exec.Cmd) (*commandContainment, error) {
	jobRaw, _, callErr := createJobObjectW.Call(0, 0)
	if jobRaw == 0 {
		return nil, fmt.Errorf("create Windows planner job object: %w", callErr)
	}
	job := syscall.Handle(jobRaw)
	information := jobObjectExtendedLimitInformation{}
	information.BasicLimitInformation.LimitFlags = jobObjectLimitKillOnJobClose
	ok, _, callErr := setInformationJobObject.Call(
		uintptr(job),
		jobObjectExtendedLimitInformationClass,
		uintptr(unsafe.Pointer(&information)),
		unsafe.Sizeof(information),
	)
	if ok == 0 {
		_, _, _ = closeHandle.Call(uintptr(job))
		return nil, fmt.Errorf("configure Windows planner job object: %w", callErr)
	}
	command.SysProcAttr = &syscall.SysProcAttr{
		CreationFlags: createNewProcessGroup,
	}
	containment := &commandContainment{command: command, job: job}
	command.Cancel = func() error {
		if containment.job != 0 {
			ok, _, callErr := terminateJobObject.Call(uintptr(containment.job), 1)
			if ok != 0 {
				return nil
			}
			return callErr
		}
		if command.Process == nil {
			return os.ErrProcessDone
		}
		return command.Process.Kill()
	}
	command.WaitDelay = 2 * time.Second
	return containment, nil
}

func (containment *commandContainment) AfterStart() error {
	if containment == nil || containment.command == nil ||
		containment.command.Process == nil || containment.job == 0 {
		return errors.New("Windows planner job containment is incomplete")
	}
	processRaw, _, callErr := openProcess.Call(
		processTerminate|processSetQuota|processQueryInformation,
		0,
		uintptr(containment.command.Process.Pid),
	)
	if processRaw == 0 {
		return callErr
	}
	defer closeHandle.Call(processRaw)
	ok, _, callErr := assignProcessToJobObject.Call(uintptr(containment.job), processRaw)
	if ok == 0 {
		return callErr
	}
	return nil
}

func (containment *commandContainment) Close() {
	if containment == nil || containment.job == 0 {
		return
	}
	_, _, _ = closeHandle.Call(uintptr(containment.job))
	containment.job = 0
}
