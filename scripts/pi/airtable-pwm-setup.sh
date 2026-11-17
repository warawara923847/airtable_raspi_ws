#!/usr/bin/env bash
# Export the Pi 4 (BCM2711) PWM0 channel that drives the SG90-HV and let the gpio group write it,
# so that airtable_hardware runs without sudo. Run as root at boot (airtable-pwm.service).
# The chip is found by its device (fe20c000.pwm), not by its pwmchip number.
# The Pi 5 used RP1 1f00098000.pwm channel 2 (docs/adr/0021).
# The channel is left as it is (disabled after boot); the hardware node refuses to take
# over an enabled channel.
set -euo pipefail

CHANNEL="${AIRTABLE_PWM_CHANNEL:-0}"   # GPIO18 (header pin 12) = PWM0 channel 0
GROUP="${AIRTABLE_PWM_GROUP:-gpio}"

chip=""
for _ in $(seq 50); do
  for candidate in /sys/class/pwm/pwmchip*; do
    case "$(readlink -f "$candidate/device")" in
      */fe20c000.pwm) chip="$candidate" ;;
    esac
  done
  [ -n "$chip" ] && break
  sleep 0.1
done
if [ -z "$chip" ]; then
  echo "PWM0 (fe20c000.pwm) not found; check dtoverlay=pwm in config.txt" >&2
  exit 1
fi

pwm="$chip/pwm$CHANNEL"
[ -d "$pwm" ] || echo "$CHANNEL" > "$chip/export"
for _ in $(seq 50); do
  [ -w "$pwm/enable" ] && break
  sleep 0.1
done

attributes=("$pwm/enable" "$pwm/period" "$pwm/duty_cycle" "$pwm/polarity")
chgrp "$GROUP" "${attributes[@]}"
chmod g+w "${attributes[@]}"
echo "$pwm ready for group $GROUP (enable=$(cat "$pwm/enable"))"
