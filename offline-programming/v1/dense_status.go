package main

import (
	"strconv"
	"strings"
)

// An explicit gzip;q=0 overrides a wildcard. Invalid qualities do not opt a
// client into compression; the ordinary uncompressed representation remains.
func acceptsGzip(values []string) bool {
	explicit, gzipQ, wildcardQ := false, 0.0, 0.0
	for _, value := range values {
		for _, encoding := range strings.Split(value, ",") {
			parts := strings.Split(encoding, ";")
			name := strings.ToLower(strings.TrimSpace(parts[0]))
			if name != "gzip" && name != "*" {
				continue
			}
			q := 1.0
			for _, parameter := range parts[1:] {
				key, value, ok := strings.Cut(parameter, "=")
				if !ok || !strings.EqualFold(strings.TrimSpace(key), "q") {
					continue
				}
				quality, err := strconv.ParseFloat(strings.TrimSpace(value), 64)
				if err != nil || !(quality >= 0 && quality <= 1) {
					quality = 0
				}
				q = quality
			}
			if name == "gzip" {
				explicit, gzipQ = true, q
			} else {
				wildcardQ = q
			}
		}
	}
	if explicit {
		return gzipQ > 0
	}
	return wildcardQ > 0
}
