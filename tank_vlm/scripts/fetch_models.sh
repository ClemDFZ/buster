#!/usr/bin/env bash
# Ensure Qwen3-VL-2B GGUF weights exist under tank_vlm/models/.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${ROOT}/models"
mkdir -p "${DEST}"

MODEL="Qwen3-VL-2B-Instruct-Q4_K_M.gguf"
CLIP="Qwen3-VL-2B-Instruct-mmproj-F16.gguf"

copy_or_skip() {
  local name="$1"
  if [[ -f "${DEST}/${name}" && $(stat -c%s "${DEST}/${name}") -gt 1000000 ]]; then
    echo "OK  ${name}"
    return
  fi
  for src in \
    "${HOME}/vlm-server/models/${name}" \
    "${HOME}/llm-server/models/${name}"
  do
    if [[ -f "${src}" ]]; then
      echo "CP  ${src} → ${DEST}/${name}"
      cp -f "${src}" "${DEST}/${name}"
      return
    fi
  done
  echo "MISS ${name} — download manually from HuggingFace unsloth/Qwen3-VL-2B-Instruct-GGUF"
  echo "     into ${DEST}/"
}

copy_or_skip "${MODEL}"
copy_or_skip "${CLIP}"
ls -lah "${DEST}"
