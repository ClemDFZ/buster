#!/usr/bin/env bash
# Copy pantilt_slave torch/TRT venv into this package (standalone).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="${ROOT}/.venv"

CANDIDATES=(
  "${HOME}/pantilt_slave/.venv"
  "${TANK_TRACK_VENV_SRC:-}"
)

src=""
for c in "${CANDIDATES[@]}"; do
  [[ -n "${c}" && -x "${c}/bin/python" ]] && src="${c}" && break
done

if [[ -z "${src}" ]]; then
  echo "ERROR: no source venv. Set TANK_TRACK_VENV_SRC or install pantilt_slave/.venv"
  exit 1
fi

echo "Copying ${src} → ${DEST} (large)…"
rm -rf "${DEST}"
cp -a "${src}" "${DEST}"
echo "OK $(du -sh "${DEST}" | awk '{print $1}')"
"${DEST}/bin/python" -c "import torch; print('torch', torch.__version__)" || \
  echo "NOTE: validate torch with scripts/track_server LD path (cusparselt)."
