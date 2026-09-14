#!/usr/bin/env bash
# Populate tank_vlm/.venv with llama-cpp (CUDA) — one-time setup.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${ROOT}/.venv"

SRC=""
for cand in \
  "${TANK_VLM_VENV_SRC:-}" \
  "${ROOT}/../llm-server/.venv" \
  "${HOME}/llm-server/.venv" \
  "${HOME}/vlm-server/.venv"
do
  [[ -z "${cand}" ]] && continue
  if [[ -x "${cand}/bin/python" ]] && "${cand}/bin/python" -c "import llama_cpp" >/dev/null 2>&1; then
    SRC="$(cd "${cand}" && pwd)"
    break
  fi
done

if [[ -z "${SRC}" ]]; then
  echo "No source venv with llama_cpp found."
  echo "Set TANK_VLM_VENV_SRC=/path/to/.venv or create one:"
  echo "  python3 -m venv ${DEST}"
  echo "  source ${DEST}/bin/activate"
  echo "  export CMAKE_ARGS='-DGGML_CUDA=on'"
  echo "  pip install -U pip wheel 'llama-cpp-python' opencv-python-headless numpy"
  exit 1
fi

echo "Copying venv:"
echo "  ${SRC}  →  ${DEST}"
mkdir -p "${DEST}"
rsync -a --delete "${SRC}/" "${DEST}/"
echo "OK — $("${DEST}/bin/python" -c 'import llama_cpp; print(llama_cpp.__file__)')"
