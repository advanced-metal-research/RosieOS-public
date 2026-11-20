package main

// Which robots can be planned for, and the few facts a client needs before it
// can author against one.
//
// The browser used to know the answer: two model names written into its own
// source, beside a table of each one's work frame. Adding a robot meant
// editing TypeScript, and a number restated there is a number that drifts
// from the robot it describes. The store holds the descriptions, so the store
// answers -- and the work frame's position comes out of the URDF at the zero
// pose rather than out of a constant.

import (
	"context"
	"encoding/base64"
	"encoding/json"
	"errors"
	"net/http"
	"os"
	"path/filepath"

	"rosieos/offline-programming/v1/internal/robotconfig"
)

// RobotsAPIPath lists the robot descriptions this server can plan against.
const RobotsAPIPath = "/api/offline-programming/v1/robots"

func (server *Server) robots(writer http.ResponseWriter, request *http.Request) {
	robots, problems := server.catalogue()
	if robots == nil && len(problems) > 0 {
		// The store itself is unreadable: that is this server's misconfiguration,
		// not a bad request.
		writeJSON(writer, http.StatusInternalServerError, map[string]any{
			"ok": false, "code": "robot_store_unreadable", "error": problems[0],
		})
		return
	}
	// A description that will not load is named rather than hidden: one broken
	// robot is not an empty catalogue, and the person needs to know which.
	writeJSON(writer, http.StatusOK, map[string]any{
		"robots": robots, "problems": problems,
	})
}

// catalogue is every robot this server can plan for: the workstation's own
// descriptions, then the ones fetched from cells and kept by identity
// (docs/PLANNING-CONTEXT.md). A model the cell holds at another revision is
// listed twice, once per identity, and the browser picks the connected
// cell's; a model only a cell ever had is listed at all, which is what lets
// a program be authored for it.
func (server *Server) catalogue() ([]robotconfig.Robot, []string) {
	robots, problems := server.robotStore.Catalogue()
	if robots == nil && len(problems) > 0 {
		return nil, problems
	}
	seen := map[string]bool{}
	for _, robot := range robots {
		seen[robot.ModelID+"@"+robot.RobotDescriptionSHA256] = true
	}
	if server.robotCache == "" {
		return robots, problems
	}
	entries, err := os.ReadDir(server.robotCache)
	if err != nil {
		return robots, problems // an empty cache is a cache
	}
	for _, entry := range entries {
		if !entry.IsDir() || len(entry.Name()) != 64 {
			continue
		}
		fetched, fetchedProblems := robotconfig.At(filepath.Join(server.robotCache, entry.Name())).Catalogue()
		problems = append(problems, fetchedProblems...)
		for _, robot := range fetched {
			if key := robot.ModelID + "@" + robot.RobotDescriptionSHA256; !seen[key] {
				seen[key] = true
				robots = append(robots, robot)
			}
		}
	}
	return robots, problems
}

// Capture FK uses the same corrected URDF as planning, without loading meshes
// or querying/moving the machine. The caller supplies its immutable observation's identity.
func (server *Server) captureModel(writer http.ResponseWriter, request *http.Request) {
	var body struct {
		ModelID     string `json:"model_id"`
		Description string `json:"robot_description_sha256"`
		Calibration string `json:"machine_planning_calibration_base64"`
	}
	fail := func(err error) { writeJSON(writer, http.StatusBadRequest, map[string]any{"error": err.Error()}) }
	if err := json.NewDecoder(http.MaxBytesReader(writer, request.Body, 1<<20)).Decode(&body); err != nil {
		fail(err)
		return
	}
	description, err := server.robotStore.Load(body.ModelID)
	if err != nil || description.Identity != body.Description {
		var found bool
		description, found = robotconfig.Cached(server.robotCache, body.ModelID, body.Description)
		if !found {
			if description, err = server.selectedMachineDescription(request.Context(), body.ModelID, body.Description); err != nil {
				fail(err)
				return
			}
		}
	}
	calibration, err := base64.StdEncoding.DecodeString(body.Calibration)
	if err != nil {
		fail(err)
		return
	}
	source := robotconfig.SourceEmpty
	if len(calibration) > 0 {
		source = robotconfig.SourceTarget
	}
	settings, err := robotconfig.Resolve(description, calibration, source)
	if err != nil {
		fail(err)
		return
	}
	compiled, err := robotconfig.Compile(description, settings)
	if err != nil {
		fail(err)
		return
	}
	writeJSON(writer, http.StatusOK, map[string]any{"urdf": string(compiled.Files[robotconfig.ContextURDF]), "identity": compiled.Identity})
}

var errCaptureDescription = errors.New("adopt the matching robot description before recording")

// selectedMachineDescription is the connected machine's own description for a
// capture, fetched and verified exactly as planning fetches it (descriptionFor),
// and only when that machine advertises the identity being captured. Planning
// used to be the only fetch, so a new program on a cell holding another
// description revision could not record its first waypoint: there was nothing
// to plan until something was recorded.
func (server *Server) selectedMachineDescription(ctx context.Context, modelID, identity string) (robotconfig.Description, error) {
	target := server.packCellTarget()
	if target == nil || !target.Available() {
		return robotconfig.Description{}, errCaptureDescription
	}
	cell, err := target.Cell(ctx)
	if err != nil || cell == nil || !cell.Robot.Valid || cell.Robot.RobotDescriptionSHA256 != identity {
		return robotconfig.Description{}, errCaptureDescription
	}
	description, err := server.descriptionFor(ctx, modelID, target, cell)
	if err != nil {
		return robotconfig.Description{}, err
	}
	if description.Identity != identity {
		return robotconfig.Description{}, errCaptureDescription
	}
	return description, nil
}
