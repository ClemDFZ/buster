#!/usr/bin/env bash
# Serve tank_sim so package://urdf_export/... resolves as /urdf_export/...
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PORT="${1:-8000}"

if ss -ltn 2>/dev/null | grep -q ":${PORT} "; then
  echo "port ${PORT} already in use — open http://localhost:${PORT}/web/"
  echo "or: ./serve.sh <other-port>"
  exit 0
fi

echo "serving $ROOT on http://localhost:${PORT}/web/"
exec python3 -m http.server "$PORT" --directory "$ROOT"
