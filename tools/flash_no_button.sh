#!/usr/bin/env bash
set -euo pipefail

PORT="${1:-}"
HEX="${2:-output/release.hex}"
MCU="${MCU:-TEENSY41}"
LOADER="${TEENSY_LOADER_CLI:-teensy_loader_cli}"
LOADER_ARGS="${TEENSY_LOADER_ARGS:--v}"

if [[ -z "${PORT}" ]]; then
  PORT="$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -n1 || true)"
fi

if [[ -z "${PORT}" ]]; then
  echo "Nessuna porta seriale trovata (/dev/ttyACM* o /dev/ttyUSB*)" >&2
  exit 1
fi

if [[ ! -f "${HEX}" ]]; then
  echo "HEX non trovato: ${HEX}" >&2
  exit 1
fi

echo "Trigger bootloader su ${PORT}"
# Fast path: CLI command (if shell is active).
printf '\rbootloader go\r' > "${PORT}" 2>/dev/null || true
# Compatibility path: Teensy-style soft reboot trigger (SET_LINE_CODING 134 baud).
stty -F "${PORT}" 134 cs8 -cstopb -parenb -ixon -ixoff raw -echo 2>/dev/null || true
sleep 0.25

echo "Flashing ${HEX} (${MCU})"
exec "${LOADER}" ${LOADER_ARGS} -w --mcu="${MCU}" "${HEX}"
