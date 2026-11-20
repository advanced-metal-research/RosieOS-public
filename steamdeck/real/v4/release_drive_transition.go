package main

import (
	"fmt"
	"path"
	"strconv"
	"strings"
)

// releaseDrivePreInstallRetry permits a fresh prepare only when the old
// controller still runs and every selected drive still reports mode 4.
func releaseDrivePreInstallRetry(saved releaseState, facts releaseProbeFacts, commit string) bool {
	if saved.Commit != commit || (saved.DriveTransitionPhase != "preparing" && saved.DriveTransitionPhase != "applying") ||
		saved.DriveTransitionPackageSHA256 != "" || saved.DriveTransitionRunID == "" ||
		!cellDeployHexCommit.MatchString(saved.FromRTCommit) || !rtCoreHexSHA256.MatchString(saved.FromConfiguration) ||
		!cellDeployHexCommit.MatchString(saved.FromOLPCommit) ||
		path.Base(facts.Current) != saved.FromRTCommit || facts.ControlEnvPin != saved.FromConfiguration ||
		facts.OLPHead != saved.FromOLPCommit || facts.MaintenanceRunID != "" || facts.ServoMaintenanceRunID != "" ||
		facts.unitActive("rosie-rt-core.service") != "active" || facts.unitActive("rosie-rt-control.service") != "active" ||
		len(saved.DriveTransitionSlaves) == 0 {
		return false
	}
	seen := make(map[int]bool, len(saved.DriveTransitionSlaves))
	for _, slave := range saved.DriveTransitionSlaves {
		if seen[slave] {
			return false
		}
		seen[slave] = true
		count := 0
		for _, readback := range facts.DriveReadbacks {
			if readback.SlavePosition == slave && readback.Key == "a6ec_encoder_position_tracking_mode" &&
				readback.Type == "u16" && readback.Index == 0x2000 && readback.Subindex == 8 {
				if !readback.ReadbackValid || !readback.Verified || readback.Readback != 4 {
					return false
				}
				count++
			}
		}
		if count != 1 {
			return false
		}
	}
	return true
}

// releaseDriveSetting binds one compiled power-cycle setting to the last
// verified readback from the running controller. A transition may write only
// entries in this list, and must verify them again after the physical cycle.
type releaseDriveSetting struct {
	Axis          int    `json:"axis"`
	SlavePosition int    `json:"slave_position"`
	Key           string `json:"key"`
	Type          string `json:"type"`
	Index         int    `json:"index"`
	Subindex      int    `json:"subindex"`
	Before        int64  `json:"before"`
	After         int64  `json:"after"`
}

// releaseDriveSettings refuses ambiguous or unverified evidence before a
// stopped-drive transition can be offered. It does not contact hardware.
func releaseDriveSettings(target rtCoreCompiledFiles, facts releaseProbeFacts) ([]releaseDriveSetting, error) {
	_, axes := rtCoreParseAxesConf(target.Files["axes.conf"])
	positions := strings.Split(rtCoreArgvValue(target.Files["argv.json"], "--slave-positions"), ",")
	if len(axes) == 0 || len(positions) != len(axes) {
		return nil, fmt.Errorf("drive preflight cannot read the target's compiled axes and slave positions")
	}
	var settings []releaseDriveSetting
	seen := map[[3]int]bool{}
	for axis, profile := range axes {
		position, err := strconv.Atoi(positions[axis])
		if err != nil || position < 0 || position > 65535 {
			return nil, fmt.Errorf("drive preflight has an invalid slave position for axis %d", axis)
		}
		for _, raw := range strings.Split(profile["startup_sdos"], ";") {
			fields := strings.Split(raw, "|")
			if len(fields) != 6 || fields[5] != "power_cycle" {
				continue
			}
			index, indexErr := strconv.Atoi(fields[2])
			subindex, subErr := strconv.Atoi(fields[3])
			after, valueErr := strconv.ParseInt(fields[4], 10, 64)
			if indexErr != nil || subErr != nil || valueErr != nil || index < 1 || index > 65535 || subindex < 0 || subindex > 255 {
				return nil, fmt.Errorf("drive preflight cannot parse %s on axis %d", fields[0], axis)
			}
			id := [3]int{position, index, subindex}
			if seen[id] {
				return nil, fmt.Errorf("drive preflight repeats object 0x%04x:%02x on slave %d", index, subindex, position)
			}
			seen[id] = true
			var matches []cellDeployDriveReadback
			for _, observed := range facts.DriveReadbacks {
				if observed.Axis == axis && observed.SlavePosition == position && observed.Key == fields[0] && observed.Type == fields[1] && observed.Index == index && observed.Subindex == subindex {
					matches = append(matches, observed)
				}
			}
			if len(matches) == 0 {
				return nil, fmt.Errorf("axis %d slave %d %s has no readback", axis, position, fields[0])
			}
			if len(matches) != 1 {
				return nil, fmt.Errorf("axis %d slave %d %s has ambiguous readbacks", axis, position, fields[0])
			}
			if !matches[0].ReadbackValid || !matches[0].Verified {
				return nil, fmt.Errorf("axis %d slave %d %s has no verified readback", axis, position, fields[0])
			}
			settings = append(settings, releaseDriveSetting{axis, position, fields[0], fields[1], index, subindex, matches[0].Readback, after})
		}
	}
	if len(settings) == 0 {
		return nil, fmt.Errorf("target has no power-cycle drive settings")
	}
	return settings, nil
}

// releaseDriveModeTransitionSettings limits the first managed transition to
// the A6-EC mode 4 to mode 1 change that rtctl can apply and verify.
func releaseDriveModeTransitionSettings(settings []releaseDriveSetting) ([]int, error) {
	var slaves []int
	for _, setting := range settings {
		if setting.Before == setting.After {
			continue
		}
		if setting.Key != "a6ec_encoder_position_tracking_mode" || setting.Type != "u16" || setting.Index != 0x2000 || setting.Subindex != 8 || setting.Before != 4 || setting.After != 1 {
			return nil, fmt.Errorf("drive transition does not support axis %d slave %d %s: %d to %d", setting.Axis, setting.SlavePosition, setting.Key, setting.Before, setting.After)
		}
		slaves = append(slaves, setting.SlavePosition)
	}
	if len(slaves) == 0 {
		return nil, fmt.Errorf("drive transition has no mode 4 to mode 1 changes")
	}
	return slaves, nil
}

// releaseDriveModeScript checks the exact installed release before asking its
// rtctl to apply or verify a declared mode change. rtctl performs the local
// stopped-unit, Idle-master, drive-identity and disabled-state checks.
func releaseDriveModeScript(ctx cellDeployContext, commit, pin string, slaves []int, phase string) string {
	var body strings.Builder
	body.WriteString("set -eu\n")
	body.WriteString("test \"$(readlink -f /opt/rosie-rt-core/current)\" = " + shellQuote("/opt/rosie-rt-core/"+commit) + "\n")
	body.WriteString("sudo -n grep -Fxq " + shellQuote("ROSIE_RT_CONFIGURATION_SHA256="+pin) + " /etc/rosie-rt-core/control.env\n")
	for _, slave := range slaves {
		body.WriteString("sudo -n /opt/rosie-rt-core/current/bin/rtctl drive-mode-transition --slave " + strconv.Itoa(slave) + " --phase " + phase + " --direction to-linear\n")
	}
	return cellDeployRelayPrologue(ctx) + "servos <<'ROSIE_DRIVE_MODE_TRANSITION'\n" + body.String() + "ROSIE_DRIVE_MODE_TRANSITION\n"
}
