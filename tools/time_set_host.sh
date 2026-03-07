#!/usr/bin/env bash
set -euo pipefail

DEV="${1:-}"
if [[ -z "${DEV}" ]]; then
  DEV="$(ls -1t /dev/ttyACM* 2>/dev/null | head -n1 || true)"
fi

if [[ -z "${DEV}" ]]; then
  echo "Nessuna porta /dev/ttyACM* trovata"
  exit 1
fi

if [[ ! -e "${DEV}" ]]; then
  echo "Device non trovato: ${DEV}"
  exit 1
fi

EPOCH="$(date -u +%s)"

echo "Sync RTC su ${DEV} con epoch UTC ${EPOCH}"

stty -F "${DEV}" 115200 raw -echo -echoe -echok
printf '\rtime set %s\r' "${EPOCH}" > "${DEV}"

# Mostra risposta shell (best-effort)
timeout 2s cat "${DEV}" || true
