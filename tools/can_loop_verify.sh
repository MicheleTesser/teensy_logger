#!/usr/bin/env bash
set -euo pipefail

TX_IF="can0"
RX_IF="can1"
CAN_ID_HEX="123"
DLC=8
RATE=100
BITRATE=1000000
TAIL8="CA11CA22"
DO_SETUP=0
SENDER="cangen"
POLL_MS=0
SUDO_CMD="${SUDO_CMD:-}"

usage() {
  cat <<'USAGE'
Usage:
  can_loop_verify.sh [options]

Options:
  --tx <ifname>            TX interface (default: can0)
  --rx <ifname>            RX interface (default: can1)
  --id <hex_id>            CAN ID in hex, no '#' (default: 123)
  --dlc <1|8>              Payload length (default: 8)
  --rate <msg/s>           Send rate in messages/s (default: 100)
  --sender <cangen|cansend> Sender engine (default: cangen)
  --poll-ms <ms>           cangen poll timeout on ENOBUFS (0=off, default: 0)
  --tail8 <8hex>           Last 4 bytes when sender=cansend and dlc=8 (default: CA11CA22)
  --setup                  Bring interfaces down/up with bitrate
  --bitrate <bps>          Bitrate used with --setup (default: 1000000)
  -h, --help               Show this help

Examples:
  ./tools/can_loop_verify.sh --tx can0 --rx can1 --dlc 1 --rate 200
  ./tools/can_loop_verify.sh --sender cangen --dlc 8 --rate 4000
  ./tools/can_loop_verify.sh --sender cansend --dlc 8 --rate 200 --tail8 CA11CA22

Stop with Ctrl+C.
USAGE
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

read_can_state() {
  local ifname="$1"
  ip -details link show "$ifname" 2>/dev/null | awk '/can state/ {print $3; exit}'
}

read_tx_packets() {
  local stat_file="/sys/class/net/${TX_IF}/statistics/tx_packets"
  if [[ -r "$stat_file" ]]; then
    cat "$stat_file"
  else
    echo 0
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --tx) TX_IF="${2:-}"; shift 2 ;;
    --rx) RX_IF="${2:-}"; shift 2 ;;
    --id) CAN_ID_HEX="${2:-}"; shift 2 ;;
    --dlc) DLC="${2:-}"; shift 2 ;;
    --rate) RATE="${2:-}"; shift 2 ;;
    --sender) SENDER="${2:-}"; shift 2 ;;
    --poll-ms) POLL_MS="${2:-}"; shift 2 ;;
    --bitrate) BITRATE="${2:-}"; shift 2 ;;
    --tail8) TAIL8="${2:-}"; shift 2 ;;
    --setup) DO_SETUP=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "Unknown argument: $1" ;;
  esac
done

command -v candump >/dev/null 2>&1 || die "candump not found (install can-utils)"

[[ "$SENDER" =~ ^(cangen|cansend)$ ]] || die "--sender must be cangen or cansend"
if [[ "$SENDER" == "cangen" ]]; then
  command -v cangen >/dev/null 2>&1 || die "cangen not found (install can-utils)"
else
  command -v cansend >/dev/null 2>&1 || die "cansend not found (install can-utils)"
fi

[[ "$DLC" =~ ^(1|8)$ ]] || die "--dlc must be 1 or 8"
[[ "$RATE" =~ ^[0-9]+$ ]] || die "--rate must be an integer"
(( RATE > 0 )) || die "--rate must be > 0"
[[ "$BITRATE" =~ ^[0-9]+$ ]] || die "--bitrate must be an integer"
(( BITRATE > 0 )) || die "--bitrate must be > 0"
[[ "$POLL_MS" =~ ^[0-9]+$ ]] || die "--poll-ms must be an integer >= 0"

CAN_ID_HEX="${CAN_ID_HEX#0x}"
CAN_ID_HEX="${CAN_ID_HEX#0X}"
CAN_ID_HEX="${CAN_ID_HEX^^}"
[[ "$CAN_ID_HEX" =~ ^[0-9A-F]+$ ]] || die "--id must be hex"

TAIL8="${TAIL8^^}"
[[ "$TAIL8" =~ ^[0-9A-F]{8}$ ]] || die "--tail8 must be exactly 8 hex chars (4 bytes)"

if (( ${#CAN_ID_HEX} <= 3 )); then
  FILTER_MASK="7FF"
else
  FILTER_MASK="1FFFFFFF"
fi
FILTER_SPEC="${CAN_ID_HEX}:${FILTER_MASK}"

PERIOD_S="$(awk -v r="$RATE" 'BEGIN{printf "%.6f", 1.0/r}')"
GAP_MS="$(awk -v r="$RATE" 'BEGIN{printf "%.6f", 1000.0/r}')"

TMP_DIR="$(mktemp -d /tmp/can_loop_verify.XXXXXX)"
CANDUMP_PIPE="${TMP_DIR}/candump.pipe"
TX_STATE_FILE="${TMP_DIR}/tx_state"
mkfifo "$CANDUMP_PIPE"

sender_pid=""
candump_pid=""
STOP_REQUESTED=0

on_signal() {
  STOP_REQUESTED=1
}

cleanup() {
  exec 3<&- 2>/dev/null || true
  if [[ -n "$candump_pid" ]]; then
    kill "$candump_pid" >/dev/null 2>&1 || true
    wait "$candump_pid" 2>/dev/null || true
  fi
  if [[ -n "$sender_pid" ]]; then
    kill "$sender_pid" >/dev/null 2>&1 || true
    wait "$sender_pid" 2>/dev/null || true
  fi
  rm -rf "$TMP_DIR"
}
trap on_signal INT TERM
trap cleanup EXIT

if (( DO_SETUP )); then
  ${SUDO_CMD} ip link set "$TX_IF" down || true
  ${SUDO_CMD} ip link set "$RX_IF" down || true
  ${SUDO_CMD} ip link set "$TX_IF" type can bitrate "$BITRATE"
  ${SUDO_CMD} ip link set "$RX_IF" type can bitrate "$BITRATE"
  ${SUDO_CMD} ip link set "$TX_IF" up
  ${SUDO_CMD} ip link set "$RX_IF" up
fi

build_payload() {
  local seq="$1"
  if (( DLC == 1 )); then
    printf -v payload "%02X" "$((seq & 0xFF))"
  else
    printf -v payload "%02X%02X%02X%02X%s" \
      "$((seq & 0xFF))" \
      "$(((seq >> 8) & 0xFF))" \
      "$(((seq >> 16) & 0xFF))" \
      "$(((seq >> 24) & 0xFF))" \
      "$TAIL8"
  fi
}

sender_loop_cansend() {
  local seq=0
  local tx_total=0
  local payload=""

  while true; do
    build_payload "$seq"
    if cansend "$TX_IF" "${CAN_ID_HEX}#${payload}" 2>/dev/null; then
      tx_total=$((tx_total + 1))
      seq=$(((seq + 1) & 0xFFFFFFFF))
      if (( (tx_total & 0x0F) == 0 )); then
        printf "%s\n" "$payload" > "$TX_STATE_FILE"
      fi
    fi
    sleep "$PERIOD_S"
  done
}

echo "Starting CAN loop verify"
echo "  TX=$TX_IF RX=$RX_IF ID=0x$CAN_ID_HEX DLC=$DLC RATE=${RATE}/s FILTER=$FILTER_SPEC SENDER=$SENDER"
if [[ "$SENDER" == "cansend" ]]; then
  echo "  Note: cansend forks per frame and often caps below 1 kmsg/s"
fi
echo "  Press Ctrl+C to stop"

if [[ "$SENDER" == "cangen" ]]; then
  if (( POLL_MS > 0 )); then
    cangen "$TX_IF" -g "$GAP_MS" -I "$CAN_ID_HEX" -L "$DLC" -D i -i -p "$POLL_MS" >/dev/null 2>&1 &
  else
    cangen "$TX_IF" -g "$GAP_MS" -I "$CAN_ID_HEX" -L "$DLC" -D i -i >/dev/null 2>&1 &
  fi
  sender_pid="$!"
else
  sender_loop_cansend &
  sender_pid="$!"
fi

RX_TOTAL=0
RX_WINDOW=0
RX_OK=0
RX_MISMATCH=0
RX_DROP_EST=0
LAST_SEQ=""
LAST_PAYLOAD=""
TX_LAST_PAYLOAD="NA"
LAST_TX_PKTS="$(read_tx_packets)"
WINDOW_SEC="$SECONDS"

if [[ "$SENDER" == "cansend" && -f "$TX_STATE_FILE" ]]; then
  TX_LAST_PAYLOAD="$(cat "$TX_STATE_FILE" 2>/dev/null || echo "NA")"
fi

stdbuf -oL candump -L "${RX_IF},${FILTER_SPEC}" > "$CANDUMP_PIPE" 2>/dev/null &
candump_pid="$!"
exec 3<"$CANDUMP_PIPE"

while true; do
  if (( STOP_REQUESTED != 0 )); then
    break
  fi

  if IFS= read -r -u 3 -t 0.2 line; then
    local_seq=0
    [[ "$line" == *"#"* ]] || continue
    payload="${line##*#}"
    payload="${payload//[[:space:]]/}"
    payload="${payload^^}"

    expected_len=$((DLC * 2))
    (( ${#payload} >= expected_len )) || continue
    payload="${payload:0:expected_len}"

    if [[ "$DLC" == "8" ]]; then
      b0="${payload:0:2}"
      b1="${payload:2:2}"
      b2="${payload:4:2}"
      b3="${payload:6:2}"
      local_seq=$(((16#${b0}) | (16#${b1} << 8) | (16#${b2} << 16) | (16#${b3} << 24)))
      seq_mod="u32"
    else
      b0="${payload:0:2}"
      local_seq=$((16#${b0}))
      seq_mod="u8"
    fi

    seq="$local_seq"
    RX_TOTAL=$((RX_TOTAL + 1))
    RX_WINDOW=$((RX_WINDOW + 1))
    LAST_PAYLOAD="$payload"

    if [[ -n "$LAST_SEQ" ]]; then
      if [[ "$seq_mod" == "u32" ]]; then
        expected=$(((LAST_SEQ + 1) & 0xFFFFFFFF))
        if (( seq == expected )); then
          RX_OK=$((RX_OK + 1))
        else
          RX_MISMATCH=$((RX_MISMATCH + 1))
          # Count drops only on forward jumps; backward jumps are duplicates/reordering.
          if (( seq > expected )); then
            RX_DROP_EST=$((RX_DROP_EST + (seq - expected)))
          fi
        fi
      else
        expected=$(((LAST_SEQ + 1) & 0xFF))
        diff=$(((seq - expected + 256) % 256))
        if (( diff == 0 )); then
          RX_OK=$((RX_OK + 1))
        else
          RX_MISMATCH=$((RX_MISMATCH + 1))
          RX_DROP_EST=$((RX_DROP_EST + diff))
        fi
      fi
    fi
    LAST_SEQ="$seq"
  fi

  now_sec="$SECONDS"
  if [[ "$now_sec" != "$WINDOW_SEC" ]]; then
    cur_tx_pkts="$(read_tx_packets)"
    TX_RATE=$((cur_tx_pkts - LAST_TX_PKTS))
    LAST_TX_PKTS="$cur_tx_pkts"
    TX_STATE="$(read_can_state "$TX_IF")"
    RX_STATE="$(read_can_state "$RX_IF")"
    [[ -n "${TX_STATE}" ]] || TX_STATE="NA"
    [[ -n "${RX_STATE}" ]] || RX_STATE="NA"
    SENDER_ALIVE=1
    if ! kill -0 "$sender_pid" 2>/dev/null; then
      SENDER_ALIVE=0
    fi

    if [[ "$SENDER" == "cansend" && -f "$TX_STATE_FILE" ]]; then
      TX_LAST_PAYLOAD="$(cat "$TX_STATE_FILE" 2>/dev/null || echo "NA")"
    fi

    if [[ -z "$LAST_PAYLOAD" ]]; then
      LAST_PAYLOAD="NA"
    fi
    printf "[%(%H:%M:%S)T] tx/s=%4d rx/s=%4d rx_total=%d mismatch=%d drop_est=%d tx_state=%s rx_state=%s sender=%d last_rx=%s last_tx=%s\n" \
      -1 "$TX_RATE" "$RX_WINDOW" "$RX_TOTAL" "$RX_MISMATCH" "$RX_DROP_EST" "$TX_STATE" "$RX_STATE" "$SENDER_ALIVE" "$LAST_PAYLOAD" "$TX_LAST_PAYLOAD"

    RX_WINDOW=0
    WINDOW_SEC="$now_sec"

    if (( SENDER_ALIVE == 0 )) && (( STOP_REQUESTED == 0 )); then
      echo "Sender process terminated. Check CAN link state/counters and dmesg."
      exit 1
    fi
  fi
done
