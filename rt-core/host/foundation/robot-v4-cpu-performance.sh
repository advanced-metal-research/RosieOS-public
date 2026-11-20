#!/bin/sh
set -eu

# Best-effort governor tuning. Systems without cpufreq simply no-op.
for governor in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  if [ -f "${governor}" ]; then
    echo performance > "${governor}" 2>/dev/null || true
  fi
done

exit 0
