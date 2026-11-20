package main

// cell_deploy.olp.replaces: services on the relay that the OLP has taken over
// (one cell: amr-servo-temperature-feed, the hand-installed servo dashboard that
// decoded telemetry on the servo host, replaced by the OLP's drive health
// page). The owner rule is that nothing on a host is removed by hand, so the
// deploy command retires them, the way cell_deploy.camera.replaces retired
// amr-camera on another cell's relay.

import "strings"

// cellDeployOLPRetireShell stops and disables each replaced unit. It runs
// only after the new OLP passed olp_check and olp_select, so a refused or
// restored install never touches them. The unit files stay installed as the
// rollback; a unit that will not stop is reported, not a reason to undo a
// working OLP.
func cellDeployOLPRetireShell(ctx cellDeployContext) string {
	if len(ctx.Deploy.OLP.Replaces) == 0 {
		return ""
	}
	return rtCoreConfigReplace(`for u in @UNITS@; do
  if systemctl is-active --quiet "$u" || systemctl is-enabled --quiet "$u" 2> /dev/null; then
    if sudo -n systemctl disable --now "$u" > /dev/null 2>&1; then step "olp replaced unit $u stopped and disabled (left installed as the rollback)"
    else step "olp WARNING replaced unit $u is still $(systemctl is-active "$u" 2> /dev/null) $(systemctl is-enabled "$u" 2> /dev/null): stop and disable failed"; fi
  else step "olp replaced unit $u already stopped and disabled"; fi
done
`, "@UNITS@", cellDeployOLPReplacedUnits(ctx))
}

// cellDeployOLPReplacedStatusShell is read-only: one line per replaced unit.
func cellDeployOLPReplacedStatusShell(ctx cellDeployContext) string {
	if len(ctx.Deploy.OLP.Replaces) == 0 {
		return ""
	}
	return rtCoreConfigReplace(`for u in @UNITS@; do echo "replaced $u $(systemctl is-active "$u" 2> /dev/null) $(systemctl is-enabled "$u" 2> /dev/null)"; done
`, "@UNITS@", cellDeployOLPReplacedUnits(ctx))
}

func cellDeployOLPReplacedUnits(ctx cellDeployContext) string {
	quoted := make([]string, len(ctx.Deploy.OLP.Replaces))
	for i, unit := range ctx.Deploy.OLP.Replaces {
		quoted[i] = shellQuote(unit)
	}
	return strings.Join(quoted, " ")
}
