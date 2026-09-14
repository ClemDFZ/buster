#!/usr/bin/env bash
# Copy OSTrack TRT engine into this package (standalone).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="${ROOT}/models/ostrack"
mkdir -p "${DEST}"

NAME="ostrack_vitb256_ce.engine"
CANDIDATES=(
  "${HOME}/pantilt_slave/weights/ostrack/${NAME}"
  "${TANK_OSTRACK_ENGINE:-}"
)

src=""
for c in "${CANDIDATES[@]}"; do
  [[ -n "${c}" && -f "${c}" ]] && src="${c}" && break
done

if [[ -z "${src}" ]]; then
  echo "ERROR: ${NAME} not found. Set TANK_OSTRACK_ENGINE or place under pantilt_slave/weights/ostrack/"
  exit 1
fi

cp -v "${src}" "${DEST}/${NAME}"
ls -lh "${DEST}/${NAME}"
echo "OK → ${DEST}/${NAME}"
