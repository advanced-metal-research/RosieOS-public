#!/bin/sh
set -eu

resolve_ethercat_iface() {
  device=""
  if [ -r /etc/ethercat.conf ]; then
    device="$(awk -F'"' '/^MASTER0_DEVICE=/{print $2; exit}' /etc/ethercat.conf)"
  fi

  if [ -n "${device}" ] && ip link show "${device}" >/dev/null 2>&1; then
    printf '%s\n' "${device}"
    return 0
  fi

  if [ -n "${device}" ]; then
    iface_by_mac="$(ip -o link | awk -v want="${device}" '
      BEGIN { want = tolower(want) }
      {
        name = $2
        sub(/:$/, "", name)
        mac = ""
        for (i = 1; i <= NF; ++i) {
          if ($i == "link/ether") {
            mac = tolower($(i + 1))
            break
          }
        }
        if (mac == want) {
          print name
          exit
        }
      }
    ')"
    if [ -n "${iface_by_mac}" ]; then
      printf '%s\n' "${iface_by_mac}"
      return 0
    fi
  fi

  for candidate in ethercat0 eth0 eth1; do
    if ip link show "${candidate}" >/dev/null 2>&1; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done

  return 1
}

iface="$(resolve_ethercat_iface || true)"
[ -n "${iface}" ] || exit 0

ip link set "${iface}" up || true

if command -v ethtool >/dev/null 2>&1; then
  ethtool -K "${iface}" gro off gso off tso off lro off || true
  ethtool --set-eee "${iface}" eee off || true
fi

exit 0
