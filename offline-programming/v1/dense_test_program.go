// Test programs: a dense joint program a tool generated (tuningreport
// inertia-plan), uploaded by the operator and run through the same Load and
// Play as a planner artifact, so this server keeps the grant, the heartbeat
// deadman and Stop. The upload moves nothing and acquires nothing; it is a
// robot route all the same (maintenance and control ownership refuse it like
// Load), because its only purpose is the Load that follows.
package main

import (
	"context"
	"encoding/base64"
	"encoding/json"
	"log"
	"net/http"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
)

// The base64 of MaxTestProgramBytes plus a plan.json; refused before decoding.
const maxTestProgramRequestBytes = denseexec.MaxTestProgramBytes*4/3 + 2<<20

type testProgramUpload struct {
	FileName      string          `json:"file_name"`
	ProgramBase64 string          `json:"program_base64"`
	PlanJSON      json.RawMessage `json:"plan_json,omitempty"`
	PlanFileName  string          `json:"plan_file_name,omitempty"`
}

// testProgramsCapability rides on the dense capabilities answer the Machine
// panel already reads, absent while test programs are off (that answer is
// then unchanged).
type testProgramsCapability struct {
	Enabled  bool `json:"enabled"`
	MaxBytes int  `json:"max_bytes"`
}

func (server *Server) testProgramsCapability() *testProgramsCapability {
	if !server.testPrograms.Enabled() {
		return nil
	}
	return &testProgramsCapability{Enabled: true, MaxBytes: denseexec.MaxTestProgramBytes}
}

// denseTestPrograms: GET -> whether test programs are on here, the size bound
// and the programs uploaded so far.
func (server *Server) denseTestPrograms(writer http.ResponseWriter, _ *http.Request) {
	programs := server.testPrograms.List()
	if programs == nil {
		programs = []denseexec.TestProgram{}
	}
	writeJSON(writer, http.StatusOK, map[string]any{
		"enabled":   server.testPrograms.Enabled(),
		"max_bytes": denseexec.MaxTestProgramBytes,
		"programs":  programs,
	})
}

// denseTestProgramUpload: POST {file_name, program_base64, plan_json?,
// plan_file_name?} -> {ok, program, checks}. The program is validated offline
// (format, digest, robot and configuration named, no process outputs, a
// plan.json naming exactly it) and then against the selected cell with
// Load's own checks, without authority. Load names it with
// source "test_program"; Play is unchanged.
func (server *Server) denseTestProgramUpload(writer http.ResponseWriter, request *http.Request) {
	if !server.testPrograms.Enabled() {
		writeDenseError(writer, http.StatusForbidden, "test_programs_disabled",
			"test programs are off on this server; the cell manifest's cell_deploy.olp.test_programs (OFFLINE_PROGRAMMING_TEST_PROGRAMS=1) turns them on")
		return
	}
	if !server.denseExecutionReady(writer) {
		return
	}
	var input testProgramUpload
	if err := decodeBoundedJSON(writer, request, maxTestProgramRequestBytes, &input); err != nil {
		writeDenseError(writer, http.StatusBadRequest, "test_program_invalid", "upload is not valid JSON or is larger than the bound: "+err.Error())
		return
	}
	blob, err := base64.StdEncoding.DecodeString(input.ProgramBase64)
	if err != nil {
		writeDenseError(writer, http.StatusBadRequest, "test_program_invalid", "program_base64 is not base64: "+err.Error())
		return
	}
	if status := server.denseExecution.Status(request.Context()); status != nil && status.Session != nil && status.Session.State == "playing" {
		writeDenseError(writer, http.StatusConflict, "program_already_executing",
			"a program is running; Stop it before loading a test program")
		return
	}
	client := controlClientOf(request)
	program, err := server.testPrograms.Add(blob, input.FileName, input.PlanJSON, input.PlanFileName, client.Label, time.Now())
	if err != nil {
		writeDenseFailure(writer, "test program", err)
		return
	}
	checker, ok := server.denseExecution.(interface {
		CheckTestProgram(context.Context, string) ([]string, error)
	})
	if !ok {
		server.testPrograms.Remove(program.TrajectoryDigest)
		writeDenseError(writer, http.StatusConflict, "capability_unimplemented", "this execution backend cannot check a test program")
		return
	}
	checks, err := checker.CheckTestProgram(request.Context(), program.TrajectoryDigest)
	if err != nil {
		server.testPrograms.Remove(program.TrajectoryDigest)
		writeDenseFailure(writer, "test program", err)
		return
	}
	log.Printf("dense-execution: test program %q %s (file %s, %d bytes, %.1f s) uploaded by %s (%s)",
		program.FileName, program.TrajectoryDigest, program.FileSHA256, program.Bytes, program.Trajectory.TotalDurationS, client.Label, client.Address)
	writeJSON(writer, http.StatusOK, map[string]any{"ok": true, "program": program, "checks": checks})
}
