package rtctl

import (
	"fmt"
	"strconv"
	"strings"
)

// CompiledAxis is one axis as the compiler emitted it for the native runtime:
// the count conversion (argv) and every bound in joint units (axes.conf).
// Offline tools that judge recorded or replayed motion against the
// configuration (tools/tuningreport, the OLP's cartesianjog-harness) read
// these instead of re-reading the machine config and robot description, so
// they cannot apply a bound the runtime does not enforce.
type CompiledAxis struct {
	Name string
	// CountsPerRev (--counts-per-rev) and GearRatio (--gear-ratio) give native
	// counts_per_unit = CountsPerRev x GearRatio / 2 pi for a rotary axis;
	// Sign (--sign) is the drive direction.
	CountsPerRev, GearRatio, Sign float64
	// limits: position range (rad) and velocity limit (rad/s).
	Lower, Upper, MaxVelocity float64
	// tracking: jog acceleration (rad/s^2), target-lead warning, following-error
	// fault and completion tolerance (rad).
	JogAcceleration, MaxTargetLead, FollowingError, CompletionTolerance float64
	// TrackingLagAllowanceMs grows the target-lead and following-error bounds
	// by |commanded velocity| x allowance; 0 when the machine declares none.
	TrackingLagAllowanceMs float64
	// MaxAcceleration (rad/s^2) and JogJerk (rad/s^3) are 0 when not compiled.
	MaxAcceleration, JogJerk float64
	// StartupSDOs and StartupObserve are the drive writes and readbacks as
	// compiled (name|type|index|subindex|value;...).
	StartupSDOs, StartupObserve string
}

// CountsPerRad is native counts_per_unit for this rotary axis.
func (a CompiledAxis) CountsPerRad() float64 {
	// Measured mathematical constant: radians per revolution, rounded to binary64.
	return a.CountsPerRev * a.GearRatio / 6.28318530717958647692
}

// CompiledAxes reads the axes back out of a compilation: argv and axis
// profiles as in Compiled (or argv.json and axes.conf of a compiled
// directory). A missing or malformed field is an error, never a default: a
// bound the tool does not have must not become zero.
func CompiledAxes(argv []string, axisProfiles string) (cycleNs uint64, axes []CompiledAxis, err error) {
	options := map[string]string{}
	for i := 0; i+1 < len(argv); i += 2 {
		if strings.HasPrefix(argv[i], "--") {
			options[argv[i][2:]] = argv[i+1]
		}
	}
	if cycleNs, err = strconv.ParseUint(options["cycle-ns"], 10, 64); err != nil {
		return 0, nil, fmt.Errorf("compiled argv --cycle-ns: %w", err)
	}
	perAxis := map[string][]float64{}
	n, err := strconv.Atoi(options["num-axes"])
	if err != nil || n < 1 {
		return 0, nil, fmt.Errorf("compiled argv --num-axes %q", options["num-axes"])
	}
	for _, key := range []string{"counts-per-rev", "gear-ratio", "sign"} {
		fields := strings.Split(options[key], ",")
		if len(fields) != n {
			return 0, nil, fmt.Errorf("compiled argv --%s has %d values for %d axes", key, len(fields), n)
		}
		for _, f := range fields {
			v, err := strconv.ParseFloat(f, 64)
			if err != nil {
				return 0, nil, fmt.Errorf("compiled argv --%s: %w", key, err)
			}
			perAxis[key] = append(perAxis[key], v)
		}
	}
	numbers := func(axis, key, value string, want int) ([]float64, error) {
		parts := strings.Split(value, "|")
		if len(parts) < want {
			return nil, fmt.Errorf("compiled axis %s %s=%q has fewer than %d fields", axis, key, value, want)
		}
		out := make([]float64, want)
		for i := range out {
			v, err := strconv.ParseFloat(parts[i], 64)
			if err != nil {
				return nil, fmt.Errorf("compiled axis %s %s: %w", axis, key, err)
			}
			out[i] = v
		}
		return out, nil
	}
	for _, line := range strings.Split(strings.TrimSuffix(axisProfiles, "\n"), "\n") {
		key, value, ok := strings.Cut(line, "=")
		if !ok {
			return 0, nil, fmt.Errorf("compiled axis profile line %q", line)
		}
		if key == "axis" {
			if value != strconv.Itoa(len(axes)) {
				return 0, nil, fmt.Errorf("compiled axes out of order at axis=%s", value)
			}
			i := len(axes)
			if i >= n {
				return 0, nil, fmt.Errorf("compiled axis profiles list more than %d axes", n)
			}
			axes = append(axes, CompiledAxis{CountsPerRev: perAxis["counts-per-rev"][i], GearRatio: perAxis["gear-ratio"][i], Sign: perAxis["sign"][i]})
			continue
		}
		if len(axes) == 0 {
			return 0, nil, fmt.Errorf("compiled axis field %s before axis=", key)
		}
		a := &axes[len(axes)-1]
		var v []float64
		switch key {
		case "name":
			a.Name = value
		case "limits":
			if v, err = numbers(a.Name, key, value, 3); err != nil {
				return 0, nil, err
			}
			a.Lower, a.Upper, a.MaxVelocity = v[0], v[1], v[2]
		case "tracking":
			if v, err = numbers(a.Name, key, value, 4); err != nil {
				return 0, nil, err
			}
			a.JogAcceleration, a.MaxTargetLead, a.FollowingError, a.CompletionTolerance = v[0], v[1], v[2], v[3]
		case "tracking_lag_allowance":
			if v, err = numbers(a.Name, key, value, 1); err != nil {
				return 0, nil, err
			}
			a.TrackingLagAllowanceMs = v[0]
		case "max_acceleration":
			if v, err = numbers(a.Name, key, value, 1); err != nil {
				return 0, nil, err
			}
			a.MaxAcceleration = v[0]
		case "jog_jerk":
			if v, err = numbers(a.Name, key, value, 1); err != nil {
				return 0, nil, err
			}
			a.JogJerk = v[0]
		case "startup_sdos":
			a.StartupSDOs = value
		case "startup_observe":
			a.StartupObserve = value
		}
	}
	if len(axes) != n {
		return 0, nil, fmt.Errorf("compiled axis profiles list %d axes, argv %d", len(axes), n)
	}
	for _, a := range axes {
		if a.Name == "" || !(a.MaxVelocity > 0) || !(a.FollowingError > 0) || !(a.MaxTargetLead > 0) || !(a.CompletionTolerance > 0) {
			return 0, nil, fmt.Errorf("compiled axis %q lacks name, limits or tracking", a.Name)
		}
	}
	return cycleNs, axes, nil
}
