package rtctl

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
)

// Drive tuning catalogs are drive data, not machine data: each
// config/drives/<name>.tuning.json declares, for one drive identity, the servo
// tuning objects a machine configuration may write at startup (per axis) or
// observe (read at startup and journal). Owner decision 2026-10-07: tuning is
// per joint; safety bounds stay one joint-unit value on every axis, and no
// catalog may reach them. A catalog is matched to a profile by its EtherCAT
// vendor/product, so adding it changes no profile bytes and no existing
// configuration identity.

// Per-axis and whole-machine descriptor bounds mirror native admission
// (include/axis_profiles.hpp, include/daemon_options.hpp kMaxStartupSdoDescriptors).
const (
	startupWritesPerAxisMaximum = 32
	startupWritesTotalMaximum   = 128
	startupObservesPerAxis      = 64
)

type tuningEntry struct {
	key, code, scalar, unit, class, access, identity string
	index, subindex                                  int
	minimum, maximum                                 int64
	allowed                                          []int64
	scale                                            float64
	source                                           object
}

type tuningCatalog struct {
	id                    string
	vendorID, productCode uint64
	entries               map[string]*tuningEntry
	forbidden             map[[2]int]string // subindex -1 forbids the whole index
	storage               *parameterStorage
}

// parameterStorage is the drive object that selects whether SDO writes are
// stored (A6-EC C13.10: 1 stores to EEPROM, 0 writes RAM only). It is not a
// tuning key: a machine with verify_parameter_storage has rt-core read it at
// every start and write the store value only if it differs (startup readback
// then gates config_verified_mask on it), and startup_trial keys are written
// between store=off and store=on so a drive power cycle reverts them.
type parameterStorage struct {
	key, scalar     string
	index, subindex int
	store, off      int64
	source          object
}

// tuningUse accumulates, over one compile, the catalog entries a machine names
// and every axis's written value, for uniformity and configuration identity.
type tuningUse struct {
	catalogs   map[string]*tuningCatalog
	entries    map[string]*tuningEntry
	values     map[string]map[int]int64
	storage    map[string]*parameterStorage
	newSyntax  bool
	totalWrite int
}

// Generic CoE ranges no catalog may name: 0x1000..0x1FFF is the communication
// area (PDO mapping and assignment, identity) and 0x6000..0x9FFF the CiA 402
// device profile (controlword, modes, homing 0x6098/0x607C, limits 0x607D/
// 0x607F, gearing 0x6091). Neither holds servo tuning on any CiA 402 drive.
func coeAreaForbidden(index int) bool {
	return index >= 0x1000 && index <= 0x1fff || index >= 0x6000 && index <= 0x9fff
}

func loadTuningCatalogs(profilesDir string) []*tuningCatalog {
	paths, err := filepath.Glob(filepath.Join(profilesDir, "*.tuning.json"))
	if err != nil {
		reject("tuning catalogs: %v", err)
	}
	sort.Strings(paths)
	var catalogs []*tuningCatalog
	for _, path := range paths {
		data, err := os.ReadFile(path)
		if err != nil {
			reject("tuning catalog %s: %v", filepath.Base(path), err)
		}
		catalog := parseTuningCatalog(asObject(decode(data)))
		for _, other := range catalogs {
			if other.vendorID == catalog.vendorID && other.productCode == catalog.productCode {
				reject("tuning catalogs %s and %s both apply to vendor %d product %d", other.id, catalog.id, catalog.vendorID, catalog.productCode)
			}
		}
		catalogs = append(catalogs, catalog)
	}
	return catalogs
}

// parseTuningCatalog validates the whole catalog before any machine uses it:
// a bad entry is refused even if no configuration names it yet.
func parseTuningCatalog(c object) *tuningCatalog {
	if !numericEqual(c.get("schema_version"), 1) {
		reject("tuning catalog schema_version must be 1")
	}
	catalog := &tuningCatalog{id: token(c.get("id")), entries: map[string]*tuningEntry{}, forbidden: map[[2]int]string{}}
	applies := asObject(c.get("applies_to"))
	vendor, _ := strconv.ParseUint(integer(applies.get("vendor_id"), 1, 0xffffffff, "tuning catalog vendor_id"), 10, 64)
	product, _ := strconv.ParseUint(integer(applies.get("product_code"), 1, 0xffffffff, "tuning catalog product_code"), 10, 64)
	catalog.vendorID, catalog.productCode = vendor, product
	for _, value := range asArray(c.get("forbidden")) {
		f := asObject(value)
		index, _ := strconv.Atoi(integer(f.get("index"), 1, 65535, "forbidden index"))
		sub := -1
		if s := f.optional("subindex", nil); s != nil {
			sub, _ = strconv.Atoi(integer(s, 0, 255, "forbidden subindex"))
		}
		if strings.TrimSpace(asString(f.get("reason"))) == "" {
			reject("tuning catalog %s forbidden 0x%04x needs a reason", catalog.id, index)
		}
		catalog.forbidden[[2]int{index, sub}] = asString(f.get("reason"))
	}
	objects := map[[2]int]string{}
	for _, m := range asObject(c.get("parameters")) {
		e := parseTuningEntry(catalog, token(m.key), asObject(m.value))
		object := [2]int{e.index, e.subindex}
		if other, ok := objects[object]; ok {
			reject("tuning catalog %s: %s and %s name the same object 0x%04x:%02x", catalog.id, other, e.key, e.index, e.subindex)
		}
		objects[object] = e.key
		catalog.entries[e.key] = e
	}
	if len(catalog.entries) == 0 {
		reject("tuning catalog %s declares no parameters", catalog.id)
	}
	if s := c.optional("parameter_storage", nil); s != nil {
		catalog.storage = parseParameterStorage(catalog, asObject(s), objects)
	}
	return catalog
}

func parseParameterStorage(catalog *tuningCatalog, s object, objects map[[2]int]string) *parameterStorage {
	label := catalog.id + " parameter_storage"
	p := &parameterStorage{key: token(s.get("key")), source: s}
	p.index, _ = strconv.Atoi(integer(s.get("index"), 1, 65535, label+" index"))
	p.subindex, _ = strconv.Atoi(integer(s.get("subindex"), 0, 255, label+" subindex"))
	if strings.TrimSpace(asString(s.get("code"))) == "" || strings.TrimSpace(asString(s.get("manual"))) == "" {
		reject("%s needs its manual code and section", label)
	}
	if asString(s.get("object")) != fmt.Sprintf("0x%04X:%02X", p.index, p.subindex) {
		reject("%s object label %s does not match index/subindex", label, asString(s.get("object")))
	}
	if coeAreaForbidden(p.index) {
		reject("%s 0x%04x:%02x is in the CoE communication or CiA 402 profile area", label, p.index, p.subindex)
	}
	if catalog.entries[p.key] != nil {
		reject("%s key %s is also a tuning key", label, p.key)
	}
	if other, ok := objects[[2]int{p.index, p.subindex}]; ok {
		reject("%s 0x%04x:%02x is also tuning key %s", label, p.index, p.subindex, other)
	}
	p.scalar = asString(s.get("type"))
	lo, hi := scalarRange(p.scalar)
	p.store, _ = strconv.ParseInt(integer(s.get("store"), lo, hi, label+" store"), 10, 64)
	p.off, _ = strconv.ParseInt(integer(s.get("ram_only"), lo, hi, label+" ram_only"), 10, 64)
	if p.store == p.off {
		reject("%s store and ram_only must differ", label)
	}
	return p
}

func parseTuningEntry(catalog *tuningCatalog, key string, e object) *tuningEntry {
	entry := &tuningEntry{key: key, source: e}
	label := catalog.id + " " + key
	entry.code = strings.TrimSpace(asString(e.get("code")))
	entry.index, _ = strconv.Atoi(integer(e.get("index"), 1, 65535, label+" index"))
	entry.subindex, _ = strconv.Atoi(integer(e.get("subindex"), 0, 255, label+" subindex"))
	if entry.code == "" || strings.TrimSpace(asString(e.get("manual"))) == "" {
		reject("%s needs its manual code and section", label)
	}
	if asString(e.get("object")) != fmt.Sprintf("0x%04X:%02X", entry.index, entry.subindex) {
		reject("%s object label %s does not match index/subindex", label, asString(e.get("object")))
	}
	if coeAreaForbidden(entry.index) {
		reject("%s 0x%04x:%02x is in the CoE communication or CiA 402 profile area, never servo tuning", label, entry.index, entry.subindex)
	}
	for _, sub := range []int{-1, entry.subindex} {
		if reason, ok := catalog.forbidden[[2]int{entry.index, sub}]; ok {
			reject("%s 0x%04x:%02x is forbidden by its catalog: %s", label, entry.index, entry.subindex, reason)
		}
	}
	entry.scalar = asString(e.get("type"))
	lo, hi := scalarRange(entry.scalar)
	parse := func(value any, what string) int64 {
		v, _ := strconv.ParseInt(integer(value, lo, hi, label+" "+what), 10, 64)
		return v
	}
	entry.minimum, entry.maximum = parse(e.get("min"), "min"), parse(e.get("max"), "max")
	if entry.minimum > entry.maximum {
		reject("%s min exceeds max", label)
	}
	inRange := func(v int64) bool { return v >= entry.minimum && v <= entry.maximum }
	if allowed := e.optional("allowed", nil); allowed != nil {
		for _, value := range asArray(allowed) {
			v := parse(value, "allowed value")
			if !inRange(v) {
				reject("%s allowed value %d outside min..max", label, v)
			}
			entry.allowed = append(entry.allowed, v)
		}
		if len(entry.allowed) == 0 {
			reject("%s allowed list is empty", label)
		}
	}
	if d := e.get("default"); !inRange(parse(d, "default")) {
		reject("%s default outside min..max", label)
	}
	entry.unit = asString(e.get("unit"))
	entry.scale = finite(e.get("scale"), label+" scale")
	if !(entry.scale > 0) {
		reject("%s scale must be positive", label)
	}
	if m := asString(e.get("modification")); m != "at_stop" && m != "during_operation" {
		reject("%s modification must be at_stop or during_operation", label)
	}
	entry.class = asString(e.get("class"))
	if entry.class != "tuning" && entry.class != "uniform" {
		reject("%s class must be tuning or uniform", label)
	}
	entry.access = asString(e.get("access"))
	if entry.access != "read_write" && entry.access != "read" {
		reject("%s access must be read_write or read", label)
	}
	if identity := e.optional("identity", nil); identity != nil {
		if entry.identity = asString(identity); entry.identity != "pre_catalog" {
			reject("%s identity must be pre_catalog", label)
		}
	}
	return entry
}

func catalogFor(catalogs []*tuningCatalog, profile object) *tuningCatalog {
	drive := asObject(profile.get("ethercat"))
	for _, catalog := range catalogs {
		if numericEqual(drive.get("vendor_id"), int(catalog.vendorID)) && numericEqual(drive.get("product_code"), int(catalog.productCode)) {
			return catalog
		}
	}
	return nil
}

// profileOwnedObjects are the objects a profile already drives or reads for
// its own contract (startup schema, Home transaction and truth, absolute
// feedback, PDO entries). A tuning key must never write or shadow one of them.
func profileOwnedObjects(profile object) map[[2]int]string {
	owned := map[[2]int]string{}
	add := func(o object, what string) {
		index, _ := strconv.Atoi(integer(o.get("index"), 0, 65535, what+" index"))
		sub, _ := strconv.Atoi(integer(o.get("subindex"), 0, 255, what+" subindex"))
		owned[[2]int{index, sub}] = what
	}
	for _, m := range asObject(profile.optional("startup_schema", object{})) {
		add(asObject(asObject(m.value).get("object")), "startup_schema "+m.key)
	}
	if home := profile.optional("native_home", nil); home != nil {
		h := asObject(home)
		add(asObject(h.get("truth_source")), "native_home truth_source")
		for _, value := range asArray(h.get("transaction")) {
			step := asObject(value)
			if step.optional("index", nil) != nil {
				add(step, "native_home "+asString(step.get("op")))
			}
		}
	}
	for _, value := range asArray(profile.optional("absolute_feedback", []any{})) {
		add(asObject(value), "absolute_feedback "+asString(asObject(value).get("key")))
	}
	drive := asObject(profile.get("ethercat"))
	for _, layout := range []string{"rx_layout", "tx_layout"} {
		for _, value := range asArray(drive.optional(layout, []any{})) {
			add(asObject(value), layout+" "+asString(asObject(value).get("semantic")))
		}
	}
	return owned
}

func (u *tuningUse) entry(catalog *tuningCatalog, profile object, key, axis string) *tuningEntry {
	if catalog == nil {
		reject("axis %s %s: no tuning catalog applies to this drive", axis, key)
	}
	e := catalog.entries[key]
	if e == nil {
		reject("axis %s unknown startup/observe key %s (not in the drive profile's startup_schema or tuning catalog %s)", axis, key, catalog.id)
	}
	if what, ok := profileOwnedObjects(profile)[[2]int{e.index, e.subindex}]; ok {
		reject("axis %s %s 0x%04x:%02x is owned by the drive profile (%s)", axis, key, e.index, e.subindex, what)
	}
	if u.entries == nil {
		u.entries, u.values, u.catalogs = map[string]*tuningEntry{}, map[string]map[int]int64{}, map[string]*tuningCatalog{}
	}
	u.catalogs[catalog.id] = catalog
	u.entries[catalog.id+"/"+key] = e
	if e.identity != "pre_catalog" {
		u.newSyntax = true
	}
	return e
}

// tuningRawValue accepts either a raw drive integer or {"value": x, "unit":
// u} in the catalog's unit; x / scale must be integral, so 6.2 Hz at 0.1 Hz
// is 62 and 6.25 Hz is refused rather than rounded.
func tuningRawValue(e *tuningEntry, value any, axis string) int64 {
	label := "axis " + axis + " " + e.key
	lo, hi := scalarRange(e.scalar)
	var raw int64
	switch v := value.(type) {
	case object:
		knownFields(v, "value", "unit")
		if e.unit == "" || asString(v.get("unit")) != e.unit {
			reject("%s unit must be %q (catalog %s); write a raw integer for unitless keys", label, e.unit, e.code)
		}
		scaled := finite(v.get("value"), label) / e.scale
		rounded := math.Round(scaled)
		if math.Abs(scaled-rounded) > 1e-9*math.Max(1, math.Abs(rounded)) {
			reject("%s %v %s is not a whole multiple of %g %s", label, v.get("value"), e.unit, e.scale, e.unit)
		}
		raw, _ = strconv.ParseInt(integer(json.Number(strconv.FormatFloat(rounded, 'f', 0, 64)), lo, hi, label), 10, 64)
	default:
		raw, _ = strconv.ParseInt(integer(value, lo, hi, label), 10, 64)
	}
	if raw < e.minimum || raw > e.maximum {
		reject("%s %d outside %s range %d..%d (raw units of %g %s)", label, raw, e.code, e.minimum, e.maximum, e.scale, e.unit)
	}
	if e.allowed != nil {
		ok := false
		for _, a := range e.allowed {
			ok = ok || a == raw
		}
		if !ok {
			reject("%s %d is not one of %s's values %v", label, raw, e.code, e.allowed)
		}
	}
	return raw
}

// renderObserve emits "key|type|index|sub;..." for catalog keys rt-core reads
// once per startup epoch and journals; it never writes them and they never
// gate config_verified_mask. A machine-level observe list applies to every
// axis, minus the keys that axis writes (their startup readback journals
// them); an axis's own list replaces it and may not name a written key.
func (u *tuningUse) renderObserve(catalog *tuningCatalog, profile, axis object, machineObserve any, written map[string]bool) string {
	name := asString(axis.get("name"))
	value, inherited := axis.optional("observe", nil), false
	if value == nil {
		value, inherited = machineObserve, true
	}
	if value == nil {
		return ""
	}
	keys := asArray(value)
	u.newSyntax = true
	if len(keys) == 0 || len(keys) > startupObservesPerAxis {
		reject("axis %s observe must list 1..%d catalog keys", name, startupObservesPerAxis)
	}
	seen := map[string]bool{}
	var rendered []string
	for _, k := range keys {
		key := token(k)
		if inherited && written[key] && !seen[key] {
			seen[key] = true
			continue
		}
		if seen[key] || written[key] {
			reject("axis %s observe %s is duplicated or already written (its startup readback is journalled)", name, key)
		}
		seen[key] = true
		e := u.entry(catalog, profile, key, name)
		rendered = append(rendered, fmt.Sprintf("%s|%s|%d|%d", key, e.scalar, e.index, e.subindex))
	}
	if len(rendered) == 0 {
		return ""
	}
	return strings.Join(rendered, ";")
}

func (u *tuningUse) record(e *tuningEntry, axis int, value int64) {
	key := e.key
	if u.values[key] == nil {
		u.values[key] = map[int]int64{}
	}
	u.values[key][axis] = value
}

// finish refuses per-axis differences for any non-tuning catalog key and
// returns the configuration-identity term: nil while the machine uses only
// pre-catalog keys, so those machines keep their earlier identity.
func (u *tuningUse) finish(axes int) any {
	if u.totalWrite > startupWritesTotalMaximum {
		reject("%d startup settings exceed the native capacity of %d", u.totalWrite, startupWritesTotalMaximum)
	}
	for _, e := range u.entries {
		if e.class == "tuning" {
			continue
		}
		values := u.values[e.key]
		distinct := map[int64]bool{}
		for _, v := range values {
			distinct[v] = true
		}
		if len(values) != 0 && (len(values) != axes || len(distinct) != 1) {
			reject("startup key %s is class %s: one value on every axis (written on %d of %d axes, %d distinct values)", e.key, e.class, len(values), axes, len(distinct))
		}
	}
	if !u.newSyntax {
		return nil
	}
	sources := map[string]object{}
	for id, e := range u.entries {
		sources[id] = e.source
	}
	for id, s := range u.storage {
		sources[id+"/parameter_storage"] = s.source
	}
	ids := make([]string, 0, len(sources))
	for id := range sources {
		ids = append(ids, id)
	}
	sort.Strings(ids)
	used := object{}
	for _, id := range ids {
		used = append(used, member{id, sources[id]})
	}
	return used
}

// renderStorage prepends the catalog's storage-mode object, at its store
// value, to an axis's startup list when the machine verifies parameter
// storage, and returns the axis's parameter_storage and startup_trial lines.
// Trial keys must be catalog keys this axis writes; gearing, direction and
// tracking mode (the profile's own startup schema) are never trials.
func (u *tuningUse) renderStorage(catalog *tuningCatalog, profile, axis object, verify bool, startup string, written map[string]bool) (string, string, string) {
	name := asString(axis.get("name"))
	trials := asArray(axis.optional("startup_trial", []any{}))
	if !verify {
		if len(trials) != 0 {
			reject("axis %s startup_trial requires the machine's verify_parameter_storage", name)
		}
		return startup, "", ""
	}
	if catalog == nil || catalog.storage == nil {
		reject("axis %s verify_parameter_storage: the drive's tuning catalog declares no parameter_storage", name)
	}
	s := catalog.storage
	if what, ok := profileOwnedObjects(profile)[[2]int{s.index, s.subindex}]; ok {
		reject("axis %s parameter_storage 0x%04x:%02x is owned by the drive profile (%s)", name, s.index, s.subindex, what)
	}
	if written[s.key] || startupSchema(profile).optional(s.key, nil) != nil {
		reject("axis %s parameter_storage key %s is also a startup key", name, s.key)
	}
	if u.storage == nil {
		u.storage = map[string]*parameterStorage{}
	}
	u.storage[catalog.id] = s
	u.newSyntax = true
	u.totalWrite++
	descriptor := fmt.Sprintf("%s|%s|%d|%d|%d", s.key, s.scalar, s.index, s.subindex, s.store)
	if startup != "" {
		descriptor += ";" + startup
	}
	if strings.Count(descriptor, ";")+1 > startupWritesPerAxisMaximum {
		reject("axis %s has %d startup settings with parameter_storage; at most %d per axis", name, strings.Count(descriptor, ";")+1, startupWritesPerAxisMaximum)
	}
	seen := map[string]bool{}
	var trial []string
	for _, value := range trials {
		key := token(value)
		if seen[key] || !written[key] || catalog.entries[key] == nil {
			reject("axis %s startup_trial %s must be a tuning-catalog key this axis writes in startup, listed once", name, key)
		}
		seen[key] = true
		trial = append(trial, key)
	}
	return descriptor, fmt.Sprintf("%s|%d", s.key, s.off), strings.Join(trial, ";")
}

// tuningKeys lists, for compiled tuning.json, every catalog key of class
// tuning that applies to this machine's drives: the configuration-only deploy
// allows per-axis differences only for these.
func tuningKeys(catalogs []*tuningCatalog, profiles []any) []string {
	seen := map[string]bool{}
	for _, value := range profiles {
		catalog := catalogFor(catalogs, asObject(value))
		if catalog == nil {
			continue
		}
		for key, e := range catalog.entries {
			if e.class == "tuning" && e.access == "read_write" {
				seen[key] = true
			}
		}
	}
	keys := make([]string, 0, len(seen))
	for key := range seen {
		keys = append(keys, key)
	}
	sort.Strings(keys)
	return keys
}
