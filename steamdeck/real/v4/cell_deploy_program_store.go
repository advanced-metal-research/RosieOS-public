package main

import (
	"fmt"
	"regexp"
	"strings"
)

// The cell's program store (offline-programming/v1 cell_programs.go): the
// programs the browser OLP and the Steam Deck share, kept by the OLP on the
// relay. Its directory is named by cell_deploy.olp.program_store and lives
// outside every deployment, like the planner's state_dir: an olp install,
// rollback or prune never takes programs with it. olp install creates the
// directory (as the relay user, mode 0700), archives what is in it into the
// new tree's olp-install/ before the swap -- the backup an install keeps --
// and refuses, restoring the previous unit, unless the restarted OLP serves
// the store. olp rollback leaves it alone: a tree from before the store keeps
// running without it, and the programs wait in the directory.

type cellDeployOLPProgramStore struct {
	// Dir is the store's directory on the relay.
	Dir string `json:"dir"`
	// Label is the name clients show ("synced with <relay>"); empty means the
	// relay's host name from the manifest.
	Label string `json:"label,omitempty"`
}

var cellDeployProgramStoreLabel = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$`)

func (p cellDeployOLPProgramStore) validate(c cellDeployConfig) error {
	if err := cellDeployCleanAbs("olp.program_store.dir", p.Dir); err != nil {
		return err
	}
	inside := func(dir, parent string) bool { return dir == parent || strings.HasPrefix(dir+"/", parent+"/") }
	if inside(p.Dir, c.DeploymentsDir) || inside(c.DeploymentsDir, p.Dir) {
		return fmt.Errorf("cell_deploy olp.program_store.dir must be outside deployments_dir: programs outlive every deployment, and prune deletes deployments")
	}
	for _, other := range []string{c.ScratchDir, c.DumpDir} {
		if other != "" && (inside(p.Dir, other) || inside(other, p.Dir)) {
			return fmt.Errorf("cell_deploy olp.program_store.dir must not share scratch_dir or dump_dir (%s): those are emptied by the commands", other)
		}
	}
	if p.Label != "" && !cellDeployProgramStoreLabel.MatchString(p.Label) {
		return fmt.Errorf("cell_deploy olp.program_store.label must be a host-like name of [A-Za-z0-9._-], got %q", p.Label)
	}
	return nil
}

// cellDeployProgramStoreLabelOf is what clients call the store.
func cellDeployProgramStoreLabelOf(ctx cellDeployContext) string {
	if p := ctx.Deploy.OLP.ProgramStore; p != nil && p.Label != "" {
		return p.Label
	}
	if cellDeployProgramStoreLabel.MatchString(ctx.RelayHost) {
		return ctx.RelayHost
	}
	return ""
}

// cellDeployProgramStoreEnv is the OLP unit's lines for the store.
func cellDeployProgramStoreEnv(ctx cellDeployContext) []string {
	p := ctx.Deploy.OLP.ProgramStore
	if p == nil {
		return nil
	}
	lines := []string{"Environment=OFFLINE_PROGRAMMING_PROGRAM_STORE=" + p.Dir}
	if label := cellDeployProgramStoreLabelOf(ctx); label != "" {
		lines = append(lines, "Environment=OFFLINE_PROGRAMMING_PROGRAM_STORE_LABEL="+label)
	}
	return lines
}

// cellDeployProgramStoreDir is the store's directory, or "" without the block.
func cellDeployProgramStoreDir(ctx cellDeployContext) string {
	if p := ctx.Deploy.OLP.ProgramStore; p != nil {
		return p.Dir
	}
	return ""
}

// cellDeployProgramStorePrepShell runs before the unit swap, inside install's
// unstage-able section: create the directory, archive its contents. tar
// exits 1 when a file changed while it was read (the running OLP saving);
// the store's atomic renames keep every file whole, so that is accepted.
const cellDeployProgramStorePrepShell = `# ---- program store (cell_deploy.olp.program_store): outlives every deployment ----
if [ -n "$PSTORE" ]; then
  install -d -m 0700 "$PSTORE" || unstage "cannot create the program store $PSTORE"
  if [ -n "$(ls -A "$PSTORE" 2> /dev/null)" ]; then
    rc=0; tar -C "$(dirname "$PSTORE")" --exclude=LOCK --exclude='.*.tmp-*' -czf "$W/program-store-pre-install.tgz" "$(basename "$PSTORE")" || rc=$?
    [ "$rc" -le 1 ] || unstage "cannot archive the program store $PSTORE (tar exit $rc)"
    step "program store $PSTORE archived to $W/program-store-pre-install.tgz ($(du -h "$W/program-store-pre-install.tgz" | cut -f1)); it stays where it is"
  else
    step "program store $PSTORE ready (empty)"
  fi
fi
`

// cellDeployProgramStoreCheckShell runs after the restarted OLP is healthy:
// it must serve the store it was given, or the install restores the unit.
const cellDeployProgramStoreCheckShell = `if [ -n "$PSTORE" ]; then
  curl -fsS -m 5 "$API/cell-programs" > "$W/cell-programs.json" 2> /dev/null || restore "the OLP does not serve the program store ($API/cell-programs)"
  python3 -c 'import json,sys
o = json.load(open(sys.argv[1]))
assert o.get("schema") == "offline-programming.cell-programs.v1" and o.get("enabled") is True, o.get("schema")
c = (o.get("current") or {}).get("program_id") or "none"
print("program store served: store %s label %s, %d programs, current %s" % (o["store"]["id"], o["store"].get("label"), len(o.get("programs") or []), c))' "$W/cell-programs.json" || restore "the OLP's program store answer is not the store (above)"
fi
`

// cellDeployProgramStoreReportShell, for rollback and status: what the
// running OLP says about the store, never a refusal.
const cellDeployProgramStoreReportShell = `if [ -n "$PSTORE" ]; then
  pc=$(curl -sS -m 5 -o "${TMPDIR:-/tmp}/rosie-cell-programs.$$" -w '%{http_code}' "$API/cell-programs" 2> /dev/null || echo 000)
  if [ "$pc" = 200 ]; then
    echo "program_store $PSTORE served: $(python3 -c 'import json,sys
o = json.load(open(sys.argv[1])); print("%d programs, current %s" % (len(o.get("programs") or []), (o.get("current") or {}).get("program_id") or "none"))' "${TMPDIR:-/tmp}/rosie-cell-programs.$$" 2> /dev/null)"
  else
    echo "program_store $PSTORE not served by the running OLP (http $pc); its programs stay in the directory ($(ls -1 "$PSTORE/programs" 2> /dev/null | wc -l) programs)"
  fi
  rm -f "${TMPDIR:-/tmp}/rosie-cell-programs.$$"
fi
`
