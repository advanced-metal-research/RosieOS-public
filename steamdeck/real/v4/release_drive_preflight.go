package main

import (
	"fmt"
	"strings"
)

// releasePowerCyclePreflight compares the new compiled drive settings with the
// readbacks reported by the running controller. It cannot replace the startup
// guard: the controller alone checks the drive again during activation.
func releasePowerCyclePreflight(target rtCoreCompiledFiles, facts releaseProbeFacts) error {
	_, axes := rtCoreParseAxesConf(target.Files["axes.conf"])
	for _, profile := range axes {
		for _, raw := range strings.Split(profile["startup_sdos"], ";") {
			fields := strings.Split(raw, "|")
			if len(fields) == 6 && fields[5] == "power_cycle" {
				return releasePowerCyclePreflightSettings(target, facts)
			}
		}
	}
	return nil
}

func releasePowerCyclePreflightSettings(target rtCoreCompiledFiles, facts releaseProbeFacts) error {
	if len(facts.DriveReadbacks) == 0 {
		return fmt.Errorf("drive preflight has no native drive readbacks; check the running controller's metrics snapshot before release")
	}
	settings, err := releaseDriveSettings(target, facts)
	if err != nil {
		return err
	}
	var issues []string
	for _, setting := range settings {
		if setting.Before != setting.After {
			issues = append(issues, fmt.Sprintf("axis %d slave %d %s is %d, target requires %d", setting.Axis, setting.SlavePosition, setting.Key, setting.Before, setting.After))
		}
	}
	if len(issues) > 0 {
		return fmt.Errorf("drive preflight blocked: %s; check missing evidence or perform a reviewed stopped-drive migration and physical drive power cycle before release", strings.Join(issues, "; "))
	}
	return nil
}
