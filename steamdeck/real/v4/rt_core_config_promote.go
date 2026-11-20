package main

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
)

// Promoting a RAM-only trial to a stored value. A startup_trial key is written
// with C13.10 = 0 and lives in drive RAM; EEPROM keeps the pre-trial value.
// When a configuration takes the key out of startup_trial with the value the
// trial wrote, write-on-change reads that value back from RAM, finds it
// identical and skips the write: EEPROM keeps the pre-trial value and a drive
// power cycle would silently bring it back, with no trial warning left to say
// so (an arm's feedforward 0/0 over a stored 1/500, 2026-10-10). The A6-EC has no
// "store parameters" object and no way to read what EEPROM holds (chapter 11,
// ESI), so the deploy makes the stored write happen: before the new
// configuration it starts a bridge configuration, the new machine JSON with each
// promoted key kept RAM-only one raw step away. The new configuration then
// reads the bridge value, differs and writes the target with C13.10 = 1. The
// robot is disarmed under the cell lock and the maintenance notice throughout,
// so the bridge value never drives a motion; a failure in the bridge rolls back
// to the old configuration, whose own trial write restores RAM exactly.

// rtCorePromotion is one key leaving one axis's startup_trial unchanged.
type rtCorePromotion struct {
	Axis   int    `json:"axis"`
	Name   string `json:"name"` // axes.conf name= (the machine axis robot_joint or name)
	Key    string `json:"key"`
	Raw    int64  `json:"raw"`
	Bridge int64  `json:"bridge"`
}

func rtCoreSplitList(value string) map[string]bool {
	out := map[string]bool{}
	for _, item := range strings.Split(value, ";") {
		if item != "" {
			out[item] = true
		}
	}
	return out
}

// rtCoreConfigPromotions lists the keys that are RAM-only trials in the running
// configuration and stored in the next one with the same compiled value. A key
// whose value also changes needs nothing: write-on-change sees it differ.
func rtCoreConfigPromotions(current, next rtCoreCompiledFiles) ([]rtCorePromotion, error) {
	_, oldAxes := rtCoreParseAxesConf(current.Files["axes.conf"])
	_, newAxes := rtCoreParseAxesConf(next.Files["axes.conf"])
	var out []rtCorePromotion
	for i := 0; i < len(oldAxes) && i < len(newAxes); i++ {
		oldTrial, newTrial := rtCoreSplitList(oldAxes[i]["startup_trial"]), rtCoreSplitList(newAxes[i]["startup_trial"])
		oldSDO, newSDO := rtCoreParseStartup(oldAxes[i]["startup_sdos"]), rtCoreParseStartup(newAxes[i]["startup_sdos"])
		for _, key := range rtCoreSortedKeys(oldTrial) {
			if newTrial[key] || newSDO[key] == "" || oldSDO[key] != newSDO[key] {
				continue
			}
			raw, err := strconv.ParseInt(newSDO[key], 10, 64)
			if err != nil {
				return nil, fmt.Errorf("axis %d %s: compiled value %q is not an integer", i, key, newSDO[key])
			}
			out = append(out, rtCorePromotion{Axis: i, Name: newAxes[i]["name"], Key: key, Raw: raw})
		}
	}
	return out, nil
}

// rtCoreTuningRange is a catalog entry's raw range (config/drives/*.tuning.json).
type rtCoreTuningRange struct {
	Min     int64   `json:"min"`
	Max     int64   `json:"max"`
	Allowed []int64 `json:"allowed"`
}

// rtCoreLoadTuningRanges reads every drive tuning catalog in the checkout. Only
// catalog keys can be trials, so every promotion has an entry.
func rtCoreLoadTuningRanges(repo string) (map[string]rtCoreTuningRange, error) {
	paths, err := filepath.Glob(filepath.Join(repo, "rt-core", "config", "drives", "*.tuning.json"))
	if err != nil {
		return nil, err
	}
	out := map[string]rtCoreTuningRange{}
	for _, path := range paths {
		raw, err := os.ReadFile(path)
		if err != nil {
			return nil, err
		}
		var doc struct {
			Parameters map[string]rtCoreTuningRange `json:"parameters"`
		}
		if err := json.Unmarshal(raw, &doc); err != nil {
			return nil, fmt.Errorf("%s: %v", path, err)
		}
		for key, r := range doc.Parameters {
			out[key] = r
		}
	}
	return out, nil
}

// rtCoreBridgeValue is one raw step from raw that the catalog admits: raw+1,
// else raw-1, else (an allowed set) the nearest other allowed value.
func rtCoreBridgeValue(raw int64, r rtCoreTuningRange) (int64, bool) {
	valid := func(v int64) bool {
		if v < r.Min || v > r.Max || v == raw {
			return false
		}
		if len(r.Allowed) == 0 {
			return true
		}
		for _, a := range r.Allowed {
			if a == v {
				return true
			}
		}
		return false
	}
	for _, v := range []int64{raw + 1, raw - 1} {
		if valid(v) {
			return v, true
		}
	}
	allowed := append([]int64(nil), r.Allowed...)
	sort.Slice(allowed, func(i, j int) bool {
		di, dj := allowed[i]-raw, allowed[j]-raw
		if di < 0 {
			di = -di
		}
		if dj < 0 {
			dj = -dj
		}
		return di < dj || (di == dj && allowed[i] > allowed[j])
	})
	for _, v := range allowed {
		if valid(v) {
			return v, true
		}
	}
	return 0, false
}

// rtCoreConfigBridge builds the bridge machine JSON: the new machine with each
// promoted key set to its bridge value (a raw integer) and listed in that
// axis's startup_trial again. Nothing else changes.
func rtCoreConfigBridge(machine []byte, promotions []rtCorePromotion, ranges map[string]rtCoreTuningRange) ([]byte, []rtCorePromotion, error) {
	dec := json.NewDecoder(bytes.NewReader(machine))
	dec.UseNumber()
	var doc map[string]any
	if err := dec.Decode(&doc); err != nil {
		return nil, nil, err
	}
	axes, _ := doc["axes"].([]any)
	out := append([]rtCorePromotion(nil), promotions...)
	for i := range out {
		p := &out[i]
		r, ok := ranges[p.Key]
		if !ok {
			return nil, nil, fmt.Errorf("promotion %s on axis %d (%s): no tuning-catalog entry gives its range", p.Key, p.Axis, p.Name)
		}
		bridge, ok := rtCoreBridgeValue(p.Raw, r)
		if !ok {
			return nil, nil, fmt.Errorf("promotion %s on axis %d (%s): no catalog value one step from %d", p.Key, p.Axis, p.Name, p.Raw)
		}
		p.Bridge = bridge
		var axis map[string]any
		if p.Axis < len(axes) {
			axis, _ = axes[p.Axis].(map[string]any)
		}
		name := ""
		if axis != nil {
			name, _ = axis["robot_joint"].(string)
			if name == "" {
				name, _ = axis["name"].(string)
			}
		}
		if axis == nil || name != p.Name {
			return nil, nil, fmt.Errorf("promotion %s: machine axis %d is %q, the compiled axis is %q", p.Key, p.Axis, name, p.Name)
		}
		startup, _ := axis["startup"].(map[string]any)
		if _, ok := startup[p.Key]; !ok {
			return nil, nil, fmt.Errorf("promotion %s: machine axis %s does not write it in startup", p.Key, p.Name)
		}
		startup[p.Key] = json.Number(strconv.FormatInt(bridge, 10))
		trial, _ := axis["startup_trial"].([]any)
		listed := false
		for _, k := range trial {
			listed = listed || k == p.Key
		}
		if !listed {
			axis["startup_trial"] = append(trial, p.Key)
		}
	}
	bridged, err := json.MarshalIndent(doc, "", "  ")
	if err != nil {
		return nil, nil, err
	}
	return append(bridged, '\n'), out, nil
}

func rtCoreSHA256Hex(data []byte) string {
	sum := sha256.Sum256(data)
	return hex.EncodeToString(sum[:])
}

// rtCorePromotionText is the one-line list the servo host and the report print.
func rtCorePromotionText(promotions []rtCorePromotion) string {
	parts := make([]string, 0, len(promotions))
	for _, p := range promotions {
		parts = append(parts, fmt.Sprintf("%s %s %d (bridge %d)", p.Name, p.Key, p.Raw, p.Bridge))
	}
	return strings.Join(parts, ", ")
}
