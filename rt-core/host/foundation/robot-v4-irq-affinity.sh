#!/bin/sh
set -eu

# CPU0 is isolated for robot-v4-rtcore. Keep EtherCAT IRQ handling on a
# housekeeping CPU so the realtime loop is not interrupted by NIC IRQ work.

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

mask_hex="${ROBOT_V4_ETHERCAT_IRQ_AFFINITY_MASK:-8}"

if [ -d "/sys/class/net/${iface}/device/msi_irqs" ]; then
  irqs="$(for path in /sys/class/net/"${iface}"/device/msi_irqs/*; do
    [ -e "${path}" ] || continue
    basename "${path}"
  done)"
else
  driver="$(ethtool -i "${iface}" 2>/dev/null | awk -F': *' '/^driver:/{print $2; exit}')"
  irqs="$(awk -v iface="${iface}" -v driver="${driver}" '
    ($0 ~ iface) || (driver != "" && $0 ~ driver) {
      gsub(/:/, "", $1)
      print $1
    }
  ' /proc/interrupts)"
fi

for irq in ${irqs}; do
  if [ -n "${irq}" ] && [ -d "/proc/irq/${irq}" ]; then
    printf '%s' "${mask_hex}" > "/proc/irq/${irq}/smp_affinity" || true
  fi
done

exit 0
