#!/usr/bin/env bash
set -Eeuo pipefail

ENV_NAME="${LITEARM_ENV_NAME:-panthera}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="${ROOT_DIR}/robot_param/litearm_full.yaml"
MODE="live"
PORT="5001"

fail() {
  printf '\nERROR: %s\n' "$*" >&2
  exit 1
}

detect_conda() {
  if command -v conda >/dev/null 2>&1; then
    return 0
  fi
  local candidates=(
    "${HOME}/miniconda3/etc/profile.d/conda.sh"
    "${HOME}/anaconda3/etc/profile.d/conda.sh"
    "/opt/conda/etc/profile.d/conda.sh"
  )
  local conda_sh
  for conda_sh in "${candidates[@]}"; do
    if [[ -f "${conda_sh}" ]]; then
      source "${conda_sh}"
      command -v conda >/dev/null 2>&1 && return 0
    fi
  done
  return 1
}

usage() {
  cat <<EOF
Usage:
  ./backend.sh [--demo|--live] [--config PATH] [--port PORT]

Options:
  --demo           Start without robot hardware (simulated state).
  --live           Start with real robot hardware. Default.
  --config PATH    Robot config YAML. Default: robot_param/litearm_full.yaml
  --port PORT      Backend port. Default: ${PORT}
  -h, --help       Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --demo)
      MODE="demo"; shift ;;
    --live)
      MODE="live"; shift ;;
    --config)
      [[ $# -ge 2 ]] || fail "--config requires a path argument."
      CONFIG="$2"; shift 2 ;;
    --port)
      [[ $# -ge 2 ]] || fail "--port requires a port number."
      PORT="$2"; shift 2 ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      fail "Unknown argument: $1. Run ./backend.sh --help for usage." ;;
  esac
done

detect_conda || fail "conda not found. Install Miniconda/Anaconda first."

# Verify conda environment exists
if ! conda env list | awk '{print $1}' | grep -qx "${ENV_NAME}"; then
  fail "conda env '${ENV_NAME}' not found. Create it or set LITEARM_ENV_NAME."
fi

# Add motor_driver.py to PYTHONPATH
TEACH_DIR="${ROOT_DIR}/../src/litearm_robot/teach"
export PYTHONPATH="${TEACH_DIR}:${PYTHONPATH:-}"

cd "${ROOT_DIR}"

if [[ "${MODE}" == "demo" ]]; then
  echo "Starting LiteArm backend in DEMO mode on http://localhost:${PORT}"
  echo "Config: ${CONFIG}"
  exec conda run --no-capture-output -n "${ENV_NAME}" python app.py \
    --demo --port "${PORT}" --config "${CONFIG}"
fi

echo "Starting LiteArm backend in LIVE mode on http://localhost:${PORT}"
echo "Config: ${CONFIG}"
exec conda run --no-capture-output -n "${ENV_NAME}" python app.py \
  --config "${CONFIG}" --port "${PORT}"
