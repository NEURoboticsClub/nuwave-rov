#!/bin/bash

# Combined launcher for both topside and bottomside terminator layouts.
# Keeps the individual scripts intact and delegates to them.
#
# Usage: setup_terminals_all.sh [-d] [-s] [-h]
#   -d: skip colcon build step (forwarded to both scripts)
#   -s: run bottomside over SSH (forwarded only to bottomside script)
#   -h: show help

set -e

WS="$HOME/nuwave-rov"
TOPSIDE_SCRIPT="$WS/setup_terminals_topside.sh"
BOTTOMSIDE_SCRIPT="$WS/setup_terminals_bottomside.sh"

usage() {
    cat <<EOF
Usage: ./$(basename "$0") [-d] [-s] [-h]

Runs both launch scripts:
  1) setup_terminals_topside.sh
  2) setup_terminals_bottomside.sh

Options:
  -d    Skip colcon build step in both scripts
  -s    Run bottomside script over SSH (passes -s to bottomside only)
  -h    Show this help menu

Examples:
  ./$(basename "$0")
  ./$(basename "$0") -d
  ./$(basename "$0") -d -s
EOF
}

SKIP_BUILD=false
USE_SSH_BOTTOMSIDE=false

while getopts "dsh" opt; do
    case $opt in
        d) SKIP_BUILD=true ;;
        s) USE_SSH_BOTTOMSIDE=true ;;
        h) usage; exit 0 ;;
        \?) usage; exit 1 ;;
    esac
done

if [ ! -x "$TOPSIDE_SCRIPT" ]; then
    echo "Topside launcher not found or not executable: $TOPSIDE_SCRIPT"
    exit 1
fi

if [ ! -x "$BOTTOMSIDE_SCRIPT" ]; then
    echo "Bottomside launcher not found or not executable: $BOTTOMSIDE_SCRIPT"
    exit 1
fi

TOPSIDE_ARGS=()
BOTTOMSIDE_ARGS=()

if [ "$SKIP_BUILD" = true ]; then
    TOPSIDE_ARGS+=("-d")
    BOTTOMSIDE_ARGS+=("-d")
fi

if [ "$USE_SSH_BOTTOMSIDE" = true ]; then
    BOTTOMSIDE_ARGS+=("-s")
fi

echo "Launching topside terminals..."
"$TOPSIDE_SCRIPT" "${TOPSIDE_ARGS[@]}"

echo "Launching bottomside terminals..."
"$BOTTOMSIDE_SCRIPT" "${BOTTOMSIDE_ARGS[@]}"

echo "Both launch scripts have been started."
