#!/usr/bin/env bash
# LiteArm Arms-only Backend (左臂+右臂，不含腰部头部)
# Usage: ./backend_arms.sh [--demo] [--port PORT]

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${ROOT_DIR}/backend.sh" --config "${ROOT_DIR}/robot_param/litearm_arms.yaml" "$@"
