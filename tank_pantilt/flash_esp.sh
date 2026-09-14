#!/usr/bin/env bash
# Flash pantilt ESP32-WROOM firmware used by tank_pantilt (COBS @ 921600).
# Not ESP32-S3. Firmware unchanged vs pantilt_slave prod sketch.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CANDIDATES=(
  "$ROOT_DIR/firmware/pantilt_state_machine"
  "$ROOT_DIR/../share/tank_pantilt/firmware/pantilt_state_machine"
  "$ROOT_DIR/../../share/tank_pantilt/firmware/pantilt_state_machine"
)
SKETCH_DIR="${SKETCH_DIR:-}"
if [[ -z "$SKETCH_DIR" ]]; then
  for d in "${CANDIDATES[@]}"; do
    if [[ -d "$d" ]]; then
      SKETCH_DIR="$d"
      break
    fi
  done
fi

CLI_BIN="${ARDUINO_CLI_BIN:-$HOME/.local/bin/arduino-cli}"
# ESP32-WROOM (Dev Module). Override: FQBN=esp32:esp32:...
FQBN="${FQBN:-esp32:esp32:esp32}"

if [[ ! -x "$CLI_BIN" ]]; then
  CLI_BIN="$(command -v arduino-cli || true)"
fi
if [[ -z "${CLI_BIN:-}" ]]; then
  echo "arduino-cli not found (install or set ARDUINO_CLI_BIN=...)"
  exit 1
fi

if [[ -z "${SKETCH_DIR:-}" || ! -d "$SKETCH_DIR" ]]; then
  echo "Sketch not found. Tried:"
  printf '  %s\n' "${CANDIDATES[@]}"
  exit 3
fi

pick_first_usb_port() {
  "$CLI_BIN" board list --format json | python3 -c '
import json, re, sys
try:
    data = json.load(sys.stdin)
except Exception:
    print("")
    raise SystemExit(0)
ports = data.get("detected_ports", [])
preferred, others = [], []
for p in ports:
    addr = p.get("port", {}).get("address", "")
    if not addr:
        continue
    (preferred if re.match(r"^/dev/tty(USB|ACM)", addr) else others).append(addr)
print((preferred + others)[0] if (preferred or others) else "")
'
}

echo "Compile: $SKETCH_DIR"
echo "FQBN:    $FQBN  (ESP32-WROOM)"
"$CLI_BIN" compile --fqbn "$FQBN" "$SKETCH_DIR"

MAX_TRIES="${MAX_TRIES:-8}"
TRY=1
while (( TRY <= MAX_TRIES )); do
  PORT="${PORT:-$(pick_first_usb_port)}"
  if [[ -z "$PORT" ]]; then
    echo "[$TRY/$MAX_TRIES] No board. Set PORT=/dev/ttyUSB0 or hold BOOT. Retry 3s..."
    sleep 3
    ((TRY++)) || true
    continue
  fi

  echo "[$TRY/$MAX_TRIES] Upload: $PORT"
  if "$CLI_BIN" upload -p "$PORT" --fqbn "$FQBN" "$SKETCH_DIR"; then
    echo "Flash done — serial @ 921600 COBS (pantilt_bridge)"
    exit 0
  fi

  echo "Upload failed (WROOM UART). Hold BOOT, retry 3s..."
  unset PORT
  sleep 3
  ((TRY++)) || true
done

echo "Upload failed after $MAX_TRIES tries."
exit 2
